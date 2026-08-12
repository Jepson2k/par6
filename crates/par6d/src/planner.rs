//! Queued-command execution: `par6-motion` behind the server's
//! [`Planner`] trait.
//!
//! `move_j` is planned from the latest measured pose under the selected
//! [`Profile`] (EXEC limits), converted sample-for-sample into the RT
//! ring format, and fed into the SPSC ring under backpressure —
//! [`ProgramBuilder`] for RUCKIG/TRAPEZOID, the TOPPRA path parameterizer
//! for TOPPRA. Completion is observed through the RT snapshot: the
//! EXEC playback publishes a high-water `completed_index` over the
//! per-command ring indexes this planner allocates, and the settle
//! policy (commanded/settled/strict) runs RT-side — so a `poll()`
//! outcome means the arm actually finished, not merely that samples
//! were emitted. `home` runs the real homing FSM via a mode request and
//! watches the snapshot; `delay` counts RT ticks.
//!
//! `tool_action` drives the fitted CAN gripper: `move` puts a firmware
//! "go to position" frame on the RT gripper slot, `calibrate` runs the
//! cmd-62 sweep, and both finish on what the gripper's own cmd-60 reply
//! says (travel finished / object detected / calibrated), never on a
//! timer alone.
//!
//! With feature `ffi` the cartesian surface is live: `move_j_pose` runs
//! seeded IK on the target pose and rides the `move_j` pipeline;
//! `move_l` samples the straight cartesian segment (position lerp +
//! orientation slerp), solves seeded IK per sample, times the joint
//! waypoints with TOPPRA ([`pinokin_sys::Trajectory`]) and streams the
//! timed trajectory into the ring at tick dt. Any IK or timing failure
//! is a command error — there is no silent joint-space fallback.

use std::time::{Duration, Instant};

use par6_bus::ObjectDetection;
use par6_config::ConfigBundle;
use par6_motion::{MotionError, MotionLimits, MoveParams, ProfileKind, ProgramBuilder};
use par6_proto::command::ToolParam;
use par6_proto::{make_error, Command, ErrorCode, WireError, UNATTRIBUTED};
use par6_rt::{
    ExecHeartbeat, Mode, RtCommand, Sample as RingSample, SampleMeta, SampleProducer,
    SnapshotReader, SpecSettle, StateSnapshot, MAX_JOINTS,
};
use par6_server::{CommandOutcome, Enablement, PlanContext, Planner};

use crate::bridge::{gripper_move_command, CoreLink};

/// How long a started command may wait for its RT mode to engage before
/// the planner declares the start failed.
const MODE_GRACE: Duration = Duration::from_secs(2);
/// Settling time before a gripper reply is trusted as the answer to the
/// command just sent \[s\]: the RT loop consumes one command per tick and
/// the reply arrives a frame later.
const TOOL_COMMAND_GRACE_S: f64 = 0.05;
/// How long a gripper move may run before the planner fails it.
const TOOL_MOVE_TIMEOUT: Duration = Duration::from_secs(5);
/// Gripper firmware calibrate timeout (vendor: 10 s).
const TOOL_CALIBRATE_TIMEOUT: Duration = Duration::from_secs(12);
/// Minimum calibration wait \[s\] — the `calibrated` bit can still be set
/// from the previous run (same rule as the homing FSM).
const TOOL_CALIBRATE_MIN_WAIT_S: f64 = 2.0;
/// Joint displacement below which a move has no path to time \[rad\].
#[cfg(feature = "ffi")]
const NULL_MOVE_RAD: f64 = 1e-9;

/// `move_l` cartesian sampling pitch: one IK waypoint per this much
/// translation \[m\] …
#[cfg(feature = "ffi")]
const MOVE_L_STEP_M: f64 = 0.005;
/// … or per this much rotation \[rad\], whichever yields more waypoints.
#[cfg(feature = "ffi")]
const MOVE_L_STEP_RAD: f64 = 0.05;
/// Waypoint-count ceiling for one `move_l` (bounds planning cost).
#[cfg(feature = "ffi")]
const MOVE_L_MAX_STEPS: usize = 400;
/// Below this much translation AND rotation a `move_l` is already at
/// its target.
#[cfg(feature = "ffi")]
const MOVE_L_NULL_M: f64 = 1e-6;
/// Largest joint change allowed between consecutive `move_l` IK
/// waypoints \[rad\]; a bigger jump means the solver hopped to another
/// IK branch and the "straight line" would whip the arm.
#[cfg(feature = "ffi")]
const MOVE_L_MAX_JOINT_STEP_RAD: f64 = 0.35;

/// The planned-move profiles this planner really implements, in the
/// upper-case spelling clients use on the wire. The server refuses any
/// name outside [`profile_names`], so a stored profile is always one of
/// these.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) enum Profile {
    /// Jerk-limited point-to-point (rsruckig).
    #[default]
    Ruckig,
    /// Trapezoid on the path coordinate; no jerk limiting.
    Trapezoid,
    /// Time-optimal path parameterization (toppra-cpp): the velocity and
    /// acceleration limits bind, nothing else.
    #[cfg(feature = "ffi")]
    Toppra,
}

impl Profile {
    fn from_name(name: &str) -> Option<Self> {
        match name {
            "RUCKIG" => Some(Self::Ruckig),
            "TRAPEZOID" => Some(Self::Trapezoid),
            #[cfg(feature = "ffi")]
            "TOPPRA" => Some(Self::Toppra),
            _ => None,
        }
    }
}

/// The profile registry the command plane advertises and validates
/// `select_profile` against. TOPPRA rides the C++ shim, so a build
/// without feature `ffi` must not offer it.
pub(crate) fn profile_names() -> Vec<String> {
    #[cfg_attr(not(feature = "ffi"), allow(unused_mut))]
    let mut names = vec!["RUCKIG".to_owned(), "TRAPEZOID".to_owned()];
    #[cfg(feature = "ffi")]
    names.push("TOPPRA".to_owned());
    names
}

/// Name of the profile a fresh runtime plans with.
pub(crate) const DEFAULT_PROFILE: &str = "RUCKIG";

/// What the planner needs to know about the fitted CAN gripper.
struct ToolSpec {
    /// Current limit \[mA\] — the ceiling for a `move` action.
    ilim_ma: f64,
}

/// A gripper action in flight, and what finishing looks like.
enum ToolWait {
    /// Jaw travel: done when the firmware stops reporting motion and
    /// reports why it stopped (target reached, or an object detected).
    Move,
    /// Firmware calibration sweep: done when the `calibrated` bit is set,
    /// no earlier than the minimum wait.
    Calibrate,
}

enum InFlightKind {
    Tool {
        wait: ToolWait,
        /// RT tick the command was queued on; replies older than the
        /// grace still describe the PREVIOUS command.
        start_tick: u64,
        timeout: Duration,
    },
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
    profile: Profile,
    tool: Option<ToolSpec>,
    tool_grace_ticks: u64,
    tool_cal_min_ticks: u64,
    inflight: Option<InFlight>,
    enablement: Enablement,
    #[cfg(feature = "ffi")]
    kin: crate::kin::CartKin,
}

impl Par6Planner {
    pub(crate) fn new(
        link: CoreLink,
        producer: SampleProducer,
        heartbeat: ExecHeartbeat,
        snapshots: SnapshotReader<StateSnapshot>,
        bundle: &ConfigBundle,
        #[cfg(feature = "ffi")] kin: crate::kin::CartKin,
    ) -> Result<Self, MotionError> {
        let exec_limits = MotionLimits::from_config(&bundle.robot, par6_config::LimitMode::Exec)?;
        let dt = bundle.robot.robot.tick_dt_s;
        let ticks = |s: f64| (s / dt).round() as u64;
        let tool = bundle
            .active_gripper()
            .and_then(|g| g.driver.as_ref())
            .map(|d| ToolSpec { ilim_ma: d.ilim_ma });
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
            profile: Profile::default(),
            tool,
            tool_grace_ticks: ticks(TOOL_COMMAND_GRACE_S).max(2),
            tool_cal_min_ticks: ticks(TOOL_CALIBRATE_MIN_WAIT_S),
            inflight: None,
            enablement: Enablement::default(),
            #[cfg(feature = "ffi")]
            kin,
        })
    }

    /// Wrap fully-timed tick-dt samples as the next EXEC in-flight
    /// command: allocate a ring index, stamp the metadata, request EXEC.
    fn start_exec(&mut self, samples: Vec<[f64; 2 * MAX_JOINTS]>, seen_exec: bool) -> InFlightKind {
        // A fresh fill generation: a flush already queued for an earlier
        // command can no longer reach these samples, however far behind
        // the RT command queue is running.
        self.producer.begin_generation();
        let ring_index = self.next_ring_index;
        self.next_ring_index = self.next_ring_index.checked_add(1).unwrap_or(1);
        let n = samples.len();
        let samples: Vec<RingSample> = samples
            .into_iter()
            .enumerate()
            .map(|(k, qqd)| {
                let mut s = RingSample {
                    q: [0.0; MAX_JOINTS],
                    qd: [0.0; MAX_JOINTS],
                    tau_ff: [0.0; MAX_JOINTS],
                    meta: SampleMeta {
                        command_index: ring_index,
                        checkpoint_id: ring_index,
                        blend_continues: false,
                        is_last: k + 1 == n,
                    },
                };
                s.q.copy_from_slice(&qqd[..MAX_JOINTS]);
                s.qd.copy_from_slice(&qqd[MAX_JOINTS..]);
                s
            })
            .collect();
        self.link.send(RtCommand::SetMode(Mode::Exec));
        self.heartbeat.feed();
        InFlightKind::Exec {
            ring_index,
            samples,
            cursor: 0,
            seen_exec,
        }
    }

    /// Plan a joint-space move from the measured pose in `snap` to
    /// `target` \[rad\] under the SELECTED profile and start it.
    fn start_joint_move(
        &mut self,
        snap: &StateSnapshot,
        target: [f64; MAX_JOINTS],
        duration: Option<f64>,
        speed: Option<f64>,
        accel: Option<f64>,
    ) -> Result<InFlightKind, WireError> {
        let start = snap.q;
        let kind = match self.profile {
            Profile::Ruckig => ProfileKind::Ruckig,
            Profile::Trapezoid => ProfileKind::Trapezoid,
            // TOPPRA times the straight joint-space path instead of
            // shaping a point-to-point profile: same waypoints, a
            // different (time-optimal) parameterization.
            #[cfg(feature = "ffi")]
            Profile::Toppra => {
                self.exec_limits
                    .require_inside_soft(&target)
                    .map_err(planning_error)?;
                // toppra needs a path to time; identical waypoints have none.
                if start
                    .iter()
                    .zip(target.iter())
                    .all(|(a, b)| (a - b).abs() < NULL_MOVE_RAD)
                {
                    return Ok(InFlightKind::Instant);
                }
                let mut waypoints = Vec::with_capacity(2 * MAX_JOINTS);
                waypoints.extend_from_slice(&start);
                waypoints.extend_from_slice(&target);
                let samples = self.toppra_samples(&waypoints, speed, accel, duration)?;
                return Ok(self.start_exec(samples, snap.mode == Mode::Exec));
            }
        };
        let mut limits = self.exec_limits;
        if let Some(accel) = accel {
            for a in limits.acceleration.iter_mut() {
                *a *= accel;
            }
        }
        let mut builder = ProgramBuilder::new(start, limits, self.dt).map_err(planning_error)?;
        builder
            .move_j(
                target,
                MoveParams {
                    profile: kind,
                    speed_fraction: speed.unwrap_or(1.0),
                    min_duration_s: duration,
                    blend_with_next: false,
                    checkpoint_id: None,
                },
            )
            .map_err(planning_error)?;
        let plan = builder.plan().map_err(planning_error)?;
        let samples = plan
            .samples()
            .iter()
            .map(|s| {
                let mut qqd = [0.0; 2 * MAX_JOINTS];
                qqd[..MAX_JOINTS].copy_from_slice(&s.q);
                qqd[MAX_JOINTS..].copy_from_slice(&s.qd);
                qqd
            })
            .collect();
        Ok(self.start_exec(samples, snap.mode == Mode::Exec))
    }

    /// TOPPRA-time a joint waypoint list and sample it at tick dt.
    /// A requested `min_duration` is a minimum: TOPPRA's optimum bounds
    /// how fast the path can be driven, a longer request time-scales the
    /// whole trajectory (velocities scale with it, so limits still hold).
    #[cfg(feature = "ffi")]
    fn toppra_samples(
        &self,
        waypoints: &[f64],
        speed: Option<f64>,
        accel: Option<f64>,
        min_duration: Option<f64>,
    ) -> Result<Vec<[f64; 2 * MAX_JOINTS]>, WireError> {
        let speed_frac = speed.unwrap_or(1.0);
        let accel_frac = accel.unwrap_or(1.0);
        let vel: Vec<f64> = self
            .exec_limits
            .velocity
            .iter()
            .map(|v| v * speed_frac)
            .collect();
        let acc: Vec<f64> = self
            .exec_limits
            .acceleration
            .iter()
            .map(|a| a * accel_frac)
            .collect();
        let traj = pinokin_sys::Trajectory::parameterize(waypoints, MAX_JOINTS, &vel, &acc, None)
            .map_err(|e| {
            make_error(
                ErrorCode::TrajNoSteps,
                UNATTRIBUTED,
                &[("detail", &e.to_string())],
            )
        })?;
        let t_path = traj.duration();
        if !t_path.is_finite() || t_path <= 0.0 {
            return Err(make_error(
                ErrorCode::TrajNoSteps,
                UNATTRIBUTED,
                &[("detail", &format!("TOPPRA produced duration {t_path}"))],
            ));
        }
        let t_eff = t_path.max(min_duration.unwrap_or(0.0));
        let scale = t_path / t_eff;
        let n = ((t_eff / self.dt).ceil() as usize).max(1);
        let mut samples = Vec::with_capacity(n);
        let (mut q, mut qd, mut qdd) = ([0.0; MAX_JOINTS], [0.0; MAX_JOINTS], [0.0; MAX_JOINTS]);
        for k in 1..=n {
            let t = ((k as f64) * self.dt).min(t_eff) * scale;
            traj.sample_into(t, &mut q, &mut qd, &mut qdd)
                .map_err(|e| {
                    make_error(
                        ErrorCode::MotnSetupFailed,
                        UNATTRIBUTED,
                        &[("detail", &format!("trajectory sampling failed: {e}"))],
                    )
                })?;
            let mut qqd = [0.0; 2 * MAX_JOINTS];
            qqd[..MAX_JOINTS].copy_from_slice(&q);
            for (out, v) in qqd[MAX_JOINTS..].iter_mut().zip(qd.iter()) {
                *out = v * scale;
            }
            samples.push(qqd);
        }
        Ok(samples)
    }

    /// Start a gripper action: validate the verb and its parameters, put
    /// the firmware command on the RT gripper slot, and wait for the
    /// cmd-60 reply to say it finished.
    fn start_tool_action(
        &mut self,
        snap: &StateSnapshot,
        cmd: &par6_proto::command::ToolAction,
    ) -> Result<InFlightKind, WireError> {
        let invalid = |detail: String| {
            make_error(
                ErrorCode::CommValidationError,
                UNATTRIBUTED,
                &[("detail", &detail)],
            )
        };
        let Some(tool) = &self.tool else {
            return Err(invalid(format!(
                "tool '{}' has no driver: it takes no actions",
                cmd.tool_key
            )));
        };
        let (wait, timeout) = match cmd.action.as_str() {
            "move" => {
                let [position, speed, current] = scalars(&cmd.params)
                    .ok_or_else(|| invalid("move takes [position, speed, current_ma]".into()))?;
                for (what, v, hi) in [
                    ("position", position, 1.0),
                    ("speed", speed, 1.0),
                    ("current", current, tool.ilim_ma),
                ] {
                    if !v.is_finite() || v < 0.0 || v > hi {
                        return Err(invalid(format!("{what} = {v} is outside [0, {hi}]")));
                    }
                }
                self.link.send(RtCommand::Gripper(gripper_move_command(
                    position, speed, current,
                )));
                (ToolWait::Move, TOOL_MOVE_TIMEOUT)
            }
            "calibrate" => {
                if !cmd.params.is_empty() {
                    return Err(invalid("calibrate takes no parameters".into()));
                }
                self.link.send(RtCommand::GripperCalibrate);
                (ToolWait::Calibrate, TOOL_CALIBRATE_TIMEOUT)
            }
            other => {
                return Err(invalid(format!(
                    "tool '{}' has no action '{other}' (move | calibrate)",
                    cmd.tool_key
                )));
            }
        };
        Ok(InFlightKind::Tool {
            wait,
            start_tick: snap.tick,
            timeout,
        })
    }

    fn start_move_j(
        &mut self,
        cmd: &par6_proto::command::MoveJ,
    ) -> Result<InFlightKind, WireError> {
        let snap = self.snapshots.latest();
        let mut target = [0.0; MAX_JOINTS];
        for (i, t) in target.iter_mut().enumerate() {
            let a = cmd.angles[i].to_radians();
            *t = if cmd.rel { snap.q[i] + a } else { a };
        }
        self.start_joint_move(&snap, target, cmd.duration, cmd.speed, cmd.accel)
    }

    /// MOVE_J_POSE: seeded IK on the target pose, then the joint-move
    /// pipeline. IK failure is a command error, never a silent no-op.
    #[cfg(feature = "ffi")]
    fn start_move_j_pose(
        &mut self,
        cmd: &par6_proto::command::MoveJPose,
    ) -> Result<InFlightKind, WireError> {
        use crate::kin::{wire_pose_to_matrix, IkResult};
        let snap = self.snapshots.latest();
        let target_pose = wire_pose_to_matrix(&cmd.pose);
        let target = match self.kin.ik(&snap.q, &target_pose) {
            IkResult::Solved(q) => q,
            IkResult::Unreachable => {
                return Err(make_error(
                    ErrorCode::IkTargetUnreachable,
                    UNATTRIBUTED,
                    &[(
                        "detail",
                        "The solver did not converge from the current configuration.",
                    )],
                ));
            }
            IkResult::Failed(e) => {
                return Err(make_error(
                    ErrorCode::MotnSetupFailed,
                    UNATTRIBUTED,
                    &[("detail", &e)],
                ));
            }
        };
        self.start_joint_move(&snap, target, cmd.duration, cmd.speed, cmd.accel)
    }

    /// MOVE_L: straight cartesian segment → seeded IK waypoints → TOPPRA
    /// timing → ring samples at tick dt. Every failure (IK, branch flip,
    /// soft limits, timing) errors the command; nothing falls back to a
    /// joint-space move.
    #[cfg(feature = "ffi")]
    fn start_move_l(
        &mut self,
        cmd: &par6_proto::command::MoveL,
    ) -> Result<InFlightKind, WireError> {
        use crate::kin::{wire_pose_to_matrix, CartSegment, IkResult};
        use par6_proto::Frame;

        let snap = self.snapshots.latest();
        let start_q = snap.q;
        let start_pose = self
            .kin
            .fk(&start_q)
            .map_err(|e| make_error(ErrorCode::MotnSetupFailed, UNATTRIBUTED, &[("detail", &e)]))?;
        let wire = wire_pose_to_matrix(&cmd.pose);
        let target_pose = match (cmd.frame, cmd.rel) {
            (Frame::Wrf, false) => wire,
            // World-frame delta: translation adds, rotation applies
            // about the world axes.
            (Frame::Wrf, true) => {
                let mut t = crate::kin::mat_mul(&wire, &start_pose);
                t[3] = start_pose[3] + wire[3];
                t[7] = start_pose[7] + wire[7];
                t[11] = start_pose[11] + wire[11];
                t
            }
            // A tool-frame pose is inherently relative to the current
            // tool frame.
            (Frame::Trf, _) => crate::kin::mat_mul(&start_pose, &wire),
        };

        let seg = CartSegment::new(&start_pose, &target_pose);
        let (len, ang) = (seg.length_m(), seg.angle_rad());
        if len < MOVE_L_NULL_M && ang < MOVE_L_NULL_M {
            return Ok(InFlightKind::Instant);
        }
        let steps = ((len / MOVE_L_STEP_M).ceil() as usize)
            .max((ang / MOVE_L_STEP_RAD).ceil() as usize)
            .clamp(2, MOVE_L_MAX_STEPS);

        // The endpoint decides reachable-at-all before the path decides
        // reachable-along-the-line.
        match self.kin.ik(&start_q, &target_pose) {
            IkResult::Solved(_) => {}
            IkResult::Unreachable => {
                return Err(make_error(
                    ErrorCode::IkTargetUnreachable,
                    UNATTRIBUTED,
                    &[(
                        "detail",
                        "The solver did not converge from the current configuration.",
                    )],
                ));
            }
            IkResult::Failed(e) => {
                return Err(make_error(
                    ErrorCode::MotnSetupFailed,
                    UNATTRIBUTED,
                    &[("detail", &e)],
                ));
            }
        }

        let total = steps + 1;
        let mut waypoints = Vec::with_capacity(total * MAX_JOINTS);
        waypoints.extend_from_slice(&start_q);
        let mut seed = start_q;
        for k in 1..=steps {
            let pose = seg.sample(k as f64 / steps as f64);
            let q = match self.kin.ik(&seed, &pose) {
                IkResult::Solved(q) => q,
                IkResult::Unreachable => {
                    return Err(make_error(
                        ErrorCode::IkPartialPath,
                        UNATTRIBUTED,
                        &[("valid", &k.to_string()), ("total", &total.to_string())],
                    ));
                }
                IkResult::Failed(e) => {
                    return Err(make_error(
                        ErrorCode::MotnSetupFailed,
                        UNATTRIBUTED,
                        &[("detail", &e)],
                    ));
                }
            };
            for j in 0..MAX_JOINTS {
                if q[j] < self.exec_limits.soft_min[j] || q[j] > self.exec_limits.soft_max[j] {
                    return Err(make_error(
                        ErrorCode::CommValidationError,
                        UNATTRIBUTED,
                        &[(
                            "detail",
                            &format!(
                                "the line leaves joint {j}'s soft window at sample {k}/{total}"
                            ),
                        )],
                    ));
                }
                if (q[j] - seed[j]).abs() > MOVE_L_MAX_JOINT_STEP_RAD {
                    return Err(make_error(
                        ErrorCode::IkPartialPath,
                        UNATTRIBUTED,
                        &[("valid", &k.to_string()), ("total", &total.to_string())],
                    ));
                }
            }
            waypoints.extend_from_slice(&q);
            seed = q;
        }

        let samples = self.toppra_samples(&waypoints, cmd.speed, cmd.accel, cmd.duration)?;
        Ok(self.start_exec(samples, snap.mode == Mode::Exec))
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
        // Mark BEFORE queueing the flush: the mark rides the ring, so it
        // is pinned to what is queued now and cannot swallow the fill of
        // whatever the client sends next.
        self.producer.flush_marker().mark();
        self.link.send(RtCommand::ExecFlush);
        self.link.send(RtCommand::SetMode(Mode::Idle));
    }

    /// Poll-time verdict for a gripper action, read off the cmd-60 reply
    /// in the snapshot. Replies from before the command reached the bus
    /// describe the previous action, so nothing counts until the grace.
    fn tool_verdict(
        wait: &ToolWait,
        start_tick: u64,
        grace_ticks: u64,
        cal_min_ticks: u64,
        snap: &StateSnapshot,
    ) -> Option<Result<(), WireError>> {
        let elapsed = snap.tick.saturating_sub(start_tick);
        if elapsed < grace_ticks {
            return None;
        }
        let reply = snap.gripper.reply?;
        let fault = i32::from(reply.temperature_error)
            | (i32::from(reply.timeout_error) << 1)
            | (i32::from(reply.estop_error) << 2)
            | (i32::from(snap.gripper.live_error_bit) << 3);
        if fault != 0 {
            return Some(Err(make_error(
                ErrorCode::MotnToolFault,
                UNATTRIBUTED,
                &[("fault_code", &fault.to_string())],
            )));
        }
        match wait {
            ToolWait::Move => (!reply.action_status
                && reply.object_detection != ObjectDetection::Moving)
                .then_some(Ok(())),
            ToolWait::Calibrate => (elapsed >= cal_min_ticks && reply.calibrated).then_some(Ok(())),
        }
    }

    /// Poll-time verdict for the in-flight command; `None` = keep going.
    fn verdict(&self, fl: &mut InFlight, snap: &StateSnapshot) -> Option<Result<(), WireError>> {
        if snap.error_active {
            return Some(Err(rt_error(snap)));
        }
        match &mut fl.kind {
            InFlightKind::Tool {
                wait,
                start_tick,
                timeout,
            } => {
                let verdict = Self::tool_verdict(
                    wait,
                    *start_tick,
                    self.tool_grace_ticks,
                    self.tool_cal_min_ticks,
                    snap,
                );
                match verdict {
                    None if fl.started.elapsed() > *timeout => {
                        let state = match wait {
                            ToolWait::Move => "move",
                            ToolWait::Calibrate => "calibrate",
                        };
                        Some(Err(make_error(
                            ErrorCode::MotnToolTimeout,
                            UNATTRIBUTED,
                            &[("state", state)],
                        )))
                    }
                    other => other,
                }
            }
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
        // stay at their permissive default: there is no workspace model
        // to bound them against yet (follow-up).
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
            Command::ToolAction(p) => {
                let snap = self.snapshots.latest();
                self.start_tool_action(&snap, p)?
            }
            #[cfg(feature = "ffi")]
            Command::MoveJPose(p) => self.start_move_j_pose(p)?,
            #[cfg(feature = "ffi")]
            Command::MoveL(p) => self.start_move_l(p)?,
            #[cfg(not(feature = "ffi"))]
            Command::MoveJPose(_) | Command::MoveL(_) => {
                return Err(make_error(
                    ErrorCode::MotnSetupFailed,
                    UNATTRIBUTED,
                    &[(
                        "detail",
                        "cartesian planning needs a par6d build with feature `ffi`",
                    )],
                ));
            }
            Command::MoveC(_) | Command::MoveS(_) | Command::MoveP(_) => {
                return Err(make_error(
                    ErrorCode::MotnSetupFailed,
                    UNATTRIBUTED,
                    &[(
                        "detail",
                        "arc/spline/process moves are not implemented yet (par6d follow-up)",
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
        // `verdict` reads planner-wide constants, so the in-flight
        // command is taken out of `self` for the call and put back.
        let mut fl = self.inflight.take()?;
        let index = fl.server_index;
        let verdict = self.verdict(&mut fl, &snap);
        self.inflight = Some(fl);
        match verdict {
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
        // Only an in-flight command can own samples in the ring: one
        // that completed drained it, one already discarded flushed it.
        // The flush is generation-bounded, so a stray one can no longer
        // erase the next command's samples, but sending it with nothing
        // in flight would still cost an RT command slot for nothing.
        if self.inflight.is_some() {
            self.discard_planned();
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
        match Profile::from_name(ctx.profile) {
            Some(p) => self.profile = p,
            // The server validates against `profile_names()` before it
            // gets here, so this can only be a wiring mistake.
            None => log::error!(
                "unknown motion profile '{}'; keeping the current one",
                ctx.profile
            ),
        }
        // tool / tcp_offset / shapes are stored and reported by the
        // server; the planner will consume them once TCP-offset
        // retargeting and collision checking land (follow-up). Cartesian
        // targets currently resolve at the URDF's TCP frame.
    }

    fn enablement(&self) -> Enablement {
        self.enablement
    }
}

/// Exactly `N` numeric tool parameters (the wire allows int or float for
/// any of them); `None` if the count or a type does not match.
fn scalars<const N: usize>(params: &[ToolParam]) -> Option<[f64; N]> {
    if params.len() != N {
        return None;
    }
    let mut out = [0.0; N];
    for (slot, p) in out.iter_mut().zip(params) {
        *slot = match p {
            ToolParam::Float(v) => *v,
            ToolParam::Int(v) => *v as f64,
            ToolParam::Bool(_) | ToolParam::Str(_) => return None,
        };
    }
    Some(out)
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
