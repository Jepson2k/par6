//! Queued-command execution: `par6-motion` behind the server's
//! [`Planner`] trait.
//!
//! `move_j` is planned with [`ProgramBuilder`] (Ruckig profile, EXEC
//! limits) from the latest measured pose, converted sample-for-sample
//! into the RT ring format, and fed into the SPSC ring under
//! backpressure. Completion is observed through the RT snapshot: the
//! EXEC playback publishes a high-water `completed_index` over the
//! per-command ring indexes this planner allocates, and the settle
//! policy (commanded/settled/strict) runs RT-side — so a `poll()`
//! outcome means the arm actually finished, not merely that samples
//! were emitted. `home` runs the real homing FSM via a mode request and
//! watches the snapshot; `delay` counts RT ticks.

use std::time::{Duration, Instant};

use par6_config::ConfigBundle;
use par6_motion::{MotionError, MotionLimits, MoveParams, ProfileKind, ProgramBuilder};
use par6_proto::{make_error, Command, ErrorCode, WireError, UNATTRIBUTED};
use par6_rt::{
    ExecHeartbeat, Mode, RtCommand, Sample as RingSample, SampleMeta, SampleProducer,
    SnapshotReader, SpecSettle, StateSnapshot, MAX_JOINTS,
};
use par6_server::{CommandOutcome, Enablement, PlanContext, Planner};

use crate::bridge::CoreLink;

/// How long a started command may wait for its RT mode to engage before
/// the planner declares the start failed.
const MODE_GRACE: Duration = Duration::from_secs(2);

enum InFlightKind {
    Exec {
        ring_index: u32,
        samples: Vec<RingSample>,
        cursor: usize,
        seen_exec: bool,
    },
    Home {
        seen_homing: bool,
    },
    Delay {
        target_tick: u64,
    },
    Instant,
}

struct InFlight {
    server_index: u64,
    started: Instant,
    kind: InFlightKind,
}

/// The `Planner` implementation `par6d` hands to the server.
pub(crate) struct Par6Planner {
    link: CoreLink,
    producer: SampleProducer,
    heartbeat: ExecHeartbeat,
    snapshots: SnapshotReader<StateSnapshot>,
    exec_limits: MotionLimits,
    dt: f64,
    ticks_per_s: f64,
    next_ring_index: u32,
    policy: par6_proto::CompletionPolicy,
    inflight: Option<InFlight>,
    enablement: Enablement,
}

impl Par6Planner {
    pub(crate) fn new(
        link: CoreLink,
        producer: SampleProducer,
        heartbeat: ExecHeartbeat,
        snapshots: SnapshotReader<StateSnapshot>,
        bundle: &ConfigBundle,
    ) -> Result<Self, MotionError> {
        let exec_limits = MotionLimits::from_config(&bundle.robot, par6_config::LimitMode::Exec)?;
        let dt = bundle.robot.robot.tick_dt_s;
        Ok(Self {
            link,
            producer,
            heartbeat,
            snapshots,
            exec_limits,
            dt,
            ticks_per_s: 1.0 / dt,
            next_ring_index: 1,
            policy: par6_proto::CompletionPolicy::Settled,
            inflight: None,
            enablement: Enablement::default(),
        })
    }

    fn start_move_j(
        &mut self,
        cmd: &par6_proto::command::MoveJ,
    ) -> Result<InFlightKind, WireError> {
        let snap = self.snapshots.latest();
        let start = snap.q;
        let mut target = [0.0; MAX_JOINTS];
        for (i, t) in target.iter_mut().enumerate() {
            let a = cmd.angles[i].to_radians();
            *t = if cmd.rel { start[i] + a } else { a };
        }
        let mut limits = self.exec_limits;
        if let Some(accel) = cmd.accel {
            for a in limits.acceleration.iter_mut() {
                *a *= accel;
            }
        }
        if cmd.blend_radius.is_some() {
            log::debug!("move_j blend_radius ignored: cross-command blending is a follow-up");
        }
        let mut builder = ProgramBuilder::new(start, limits, self.dt).map_err(planning_error)?;
        builder
            .move_j(
                target,
                MoveParams {
                    profile: ProfileKind::Ruckig,
                    speed_fraction: cmd.speed.unwrap_or(1.0),
                    min_duration_s: cmd.duration,
                    blend_with_next: false,
                    checkpoint_id: None,
                },
            )
            .map_err(planning_error)?;
        let plan = builder.plan().map_err(planning_error)?;

        let ring_index = self.next_ring_index;
        self.next_ring_index = self.next_ring_index.checked_add(1).unwrap_or(1);
        let n = plan.len();
        let samples: Vec<RingSample> = plan
            .samples()
            .iter()
            .enumerate()
            .map(|(k, s)| RingSample {
                q: s.q,
                qd: s.qd,
                tau_ff: s.tau_ff,
                meta: SampleMeta {
                    command_index: ring_index,
                    checkpoint_id: ring_index,
                    blend_continues: false,
                    is_last: k + 1 == n,
                },
            })
            .collect();
        self.link.send(RtCommand::SetMode(Mode::Exec));
        self.heartbeat.feed();
        Ok(InFlightKind::Exec {
            ring_index,
            samples,
            cursor: 0,
            seen_exec: snap.mode == Mode::Exec,
        })
    }

    /// Feed pending samples into the ring, up to its free capacity.
    fn pump_ring(&mut self) {
        let Some(InFlight {
            kind: InFlightKind::Exec {
                samples, cursor, ..
            },
            ..
        }) = &mut self.inflight
        else {
            return;
        };
        while *cursor < samples.len() && self.producer.try_push(&samples[*cursor]) {
            *cursor += 1;
        }
    }

    fn discard_planned(&mut self) {
        self.inflight = None;
        self.link.send(RtCommand::ExecFlush);
        self.link.send(RtCommand::SetMode(Mode::Idle));
    }

    /// Poll-time verdict for the in-flight command; `None` = keep going.
    fn verdict(fl: &mut InFlight, snap: &StateSnapshot) -> Option<Result<(), WireError>> {
        if snap.error_active {
            return Some(Err(rt_error(snap)));
        }
        match &mut fl.kind {
            InFlightKind::Exec {
                ring_index,
                seen_exec,
                ..
            } => {
                if !*seen_exec {
                    if snap.mode == Mode::Exec {
                        *seen_exec = true;
                    } else if fl.started.elapsed() > MODE_GRACE {
                        return Some(Err(make_error(
                            ErrorCode::MotnSetupFailed,
                            UNATTRIBUTED,
                            &[("detail", "the RT core refused EXEC mode")],
                        )));
                    }
                }
                if snap.exec.completed_index >= *ring_index {
                    return Some(Ok(()));
                }
                None
            }
            InFlightKind::Home { seen_homing } => {
                if !*seen_homing {
                    if snap.mode == Mode::Homing {
                        *seen_homing = true;
                    } else if fl.started.elapsed() > MODE_GRACE {
                        return Some(Err(make_error(
                            ErrorCode::MotnSetupFailed,
                            UNATTRIBUTED,
                            &[("detail", "the RT core refused HOMING mode")],
                        )));
                    }
                    None
                } else if snap.mode != Mode::Homing {
                    if snap.homed {
                        Some(Ok(()))
                    } else {
                        Some(Err(make_error(
                            ErrorCode::MotnTickFailed,
                            UNATTRIBUTED,
                            &[("detail", "the homing sequence failed")],
                        )))
                    }
                } else {
                    None
                }
            }
            InFlightKind::Delay { target_tick } => (snap.tick >= *target_tick).then_some(Ok(())),
            InFlightKind::Instant => Some(Ok(())),
        }
    }

    fn update_enablement(&mut self, snap: &StateSnapshot) {
        // Direction freedom against the soft window. Cartesian flags
        // stay at their permissive default until the par6-kin FK/IK
        // adapter lands (follow-up).
        let mut en = Enablement::default();
        for j in 0..MAX_JOINTS {
            en.joint_en[2 * j] = u8::from(snap.q[j] > self.exec_limits.soft_min[j]);
            en.joint_en[2 * j + 1] = u8::from(snap.q[j] < self.exec_limits.soft_max[j]);
        }
        self.enablement = en;
    }
}

impl Planner for Par6Planner {
    fn start(&mut self, index: u64, cmd: &Command) -> Result<(), WireError> {
        let kind = match cmd {
            Command::MoveJ(p) => self.start_move_j(p)?,
            Command::Home(_) => {
                self.link.send(RtCommand::SetMode(Mode::Homing));
                InFlightKind::Home { seen_homing: false }
            }
            Command::Delay(p) => {
                let snap = self.snapshots.latest();
                let ticks = (p.seconds * self.ticks_per_s).round().max(1.0) as u64;
                InFlightKind::Delay {
                    target_tick: snap.tick + ticks,
                }
            }
            Command::Checkpoint(_) | Command::SelectTool(_) => InFlightKind::Instant,
            Command::ToolAction(_) => {
                return Err(make_error(
                    ErrorCode::MotnSetupFailed,
                    UNATTRIBUTED,
                    &[(
                        "detail",
                        "tool_action is not wired to the gripper yet (par6d follow-up)",
                    )],
                ));
            }
            Command::MoveJPose(_)
            | Command::MoveL(_)
            | Command::MoveC(_)
            | Command::MoveS(_)
            | Command::MoveP(_) => {
                return Err(make_error(
                    ErrorCode::MotnSetupFailed,
                    UNATTRIBUTED,
                    &[(
                        "detail",
                        "cartesian planning requires the par6-kin IK adapter (follow-up)",
                    )],
                ));
            }
            other => {
                return Err(make_error(
                    ErrorCode::CommValidationError,
                    UNATTRIBUTED,
                    &[(
                        "detail",
                        &format!("{:?} is not a queued command", other.tag()),
                    )],
                ));
            }
        };
        self.inflight = Some(InFlight {
            server_index: index,
            started: Instant::now(),
            kind,
        });
        self.pump_ring();
        Ok(())
    }

    fn poll(&mut self) -> Option<CommandOutcome> {
        let snap = self.snapshots.latest();
        self.update_enablement(&snap);
        self.inflight.as_ref()?;
        if matches!(
            self.inflight,
            Some(InFlight {
                kind: InFlightKind::Exec { .. },
                ..
            })
        ) {
            self.heartbeat.feed();
            self.pump_ring();
        }
        let fl = self.inflight.as_mut()?;
        let index = fl.server_index;
        match Self::verdict(fl, &snap) {
            None => None,
            Some(Ok(())) => {
                self.inflight = None;
                Some(CommandOutcome { index, error: None })
            }
            Some(Err(e)) => {
                self.discard_planned();
                Some(CommandOutcome {
                    index,
                    error: Some(e),
                })
            }
        }
    }

    fn cancel(&mut self) {
        if self.inflight.is_some() {
            self.discard_planned();
        } else {
            // Idempotent: still flush anything queued in the ring.
            self.link.send(RtCommand::ExecFlush);
        }
    }

    fn sync(&mut self, ctx: PlanContext<'_>) {
        if ctx.completion_policy != self.policy {
            self.policy = ctx.completion_policy;
            let rt_policy = match ctx.completion_policy {
                par6_proto::CompletionPolicy::Commanded => par6_rt::CompletionPolicy::Commanded,
                par6_proto::CompletionPolicy::Settled => par6_rt::CompletionPolicy::Settled,
                par6_proto::CompletionPolicy::Strict => par6_rt::CompletionPolicy::Strict,
            };
            let dt = self.dt;
            self.link.op(Box::new(move |core| {
                core.set_settle_policy(Box::new(SpecSettle::new(rt_policy, dt)));
            }));
        }
        // profile / tool / tcp_offset / shapes are stored and reported by
        // the server; the planner will consume them once cartesian
        // planning and collision checking land with par6-kin (follow-up).
    }

    fn enablement(&self) -> Enablement {
        self.enablement
    }
}

fn planning_error(e: MotionError) -> WireError {
    let code = match e {
        MotionError::InvalidInput { .. } | MotionError::TargetOutsideSoftLimits { .. } => {
            ErrorCode::CommValidationError
        }
        _ => ErrorCode::MotnSetupFailed,
    };
    make_error(code, UNATTRIBUTED, &[("detail", &e.to_string())])
}

/// Map the RT error latch to the closest wire error.
fn rt_error(snap: &StateSnapshot) -> WireError {
    use par6_rt::ErrorCode as Rt;
    let errs = snap.errors.as_slice();
    let has = |c: Rt| errs.iter().any(|e| e.code == c);
    if has(Rt::ExecSettleTimeout) {
        make_error(
            ErrorCode::MotnSettleTimeout,
            UNATTRIBUTED,
            &[("residual", "unknown")],
        )
    } else if has(Rt::Estop) || has(Rt::SwEstop) {
        make_error(ErrorCode::SysEstopActive, UNATTRIBUTED, &[])
    } else if has(Rt::ExecLinkLost) {
        make_error(ErrorCode::SysExecLinkLost, UNATTRIBUTED, &[])
    } else if has(Rt::LoopCritical) {
        make_error(ErrorCode::SysLoopCritical, UNATTRIBUTED, &[])
    } else if let Some(e) = errs.iter().find(|e| e.joint.is_some()) {
        make_error(
            ErrorCode::SysJointFault,
            UNATTRIBUTED,
            &[
                ("joint", &format!("{}", e.joint.unwrap_or(0))),
                ("kind", &format!("{:?}", e.code)),
            ],
        )
    } else {
        make_error(
            ErrorCode::MotnTickFailed,
            UNATTRIBUTED,
            &[("detail", "the RT core latched a hard error")],
        )
    }
}
