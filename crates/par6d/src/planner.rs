//! Queued-command execution: `par6-motion` behind the server's
//! [`Planner`] trait.
//!
//! `move_j` is planned from the latest measured pose under the selected
//! [`Profile`] (EXEC limits), converted sample-for-sample into the RT
//! ring format, and fed into the SPSC ring under backpressure —
//! [`ProgramBuilder`] for RUCKIG/TRAPEZOID/QUINTIC, the TOPPRA path parameterizer
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
//! With feature `ffi` the cartesian surface is live. `move_j_pose` runs
//! seeded IK on the target pose and rides the `move_j` pipeline; every
//! other cartesian move rides ONE pipeline
//! ([`Par6Planner::start_cart_path`]): `par6-motion`'s [`cart`] geometry
//! produces the pose list, seeded IK turns each pose into a joint
//! waypoint (guarded against soft-window exits and IK branch flips),
//! TOPPRA ([`pinokin_sys::Trajectory`]) times the waypoint chain, and
//! the timed trajectory streams into the ring at tick dt. The geometry
//! is all that differs between them:
//!
//! - `move_l` — one straight segment (position lerp + orientation slerp).
//! - `move_c` — the arc through the via point, on the circle the three
//!   poses define; a repeated start point means the whole circle.
//! - `move_s` — a cubic spline through the waypoints.
//! - `move_p` — the waypoints as straight segments with every interior
//!   corner rounded, so the TCP sweeps the path without stopping.
//!
//! Any IK or timing failure is a command error — there is no silent
//! joint-space fallback and no second timing path.
//!
//! **Blending.** A queued move whose blend radius is positive is planned
//! together with the moves QUEUED BEHIND IT (the server hands them over
//! as lookahead): a chain of `move_l`s becomes one cartesian path with
//! Bézier corners, a chain of `move_j` / `move_j_pose` becomes one joint
//! path with the corner zones sized from the TCP distance the radius
//! names. One motion covers the whole chain, so the arm never comes to
//! rest at an interior waypoint, and the commands it consumed all
//! complete when it does.
//!
//! [`cart`]: par6_motion::cart

use std::time::{Duration, Instant};

use par6_config::ConfigBundle;
use par6_kin::NQ;
use par6_motion::cart::Pose;
use par6_motion::{MotionError, MotionLimits, MoveParams, ProfileKind, ProgramBuilder};
use par6_proto::command::ToolParam;
use par6_proto::{make_error, Command, ErrorCode, WireError, EN_SLOTS, UNATTRIBUTED};
use par6_rt::gripper_settle::ToolSettle;
use par6_rt::{
    ExecHeartbeat, Mode, RtCommand, Sample as RingSample, SampleMeta, SampleProducer,
    SnapshotReader, SpecSettle, StateSnapshot, MAX_JOINTS,
};
use par6_server::{
    CollisionState, CommandOutcome, Enablement, PlanContext, Planner, QueuedCommand, ShapeLayer,
};

use crate::bridge::ESCAPE_TOL_M;
use crate::bridge::{gripper_move_command, CoreLink};
use crate::collision_world::{first_duplicate, is_world_name, kin_layer, ShapeNames};

/// How long a started command may wait for its RT mode to engage before
/// the planner declares the start failed.
const MODE_GRACE: Duration = Duration::from_secs(2);
/// Joint displacement below which a move has no path to time \[rad\].
const NULL_MOVE_RAD: f64 = 1e-9;
/// Speed fraction the return-to-home move runs at when the arm is
/// already referenced (vendor `HOME_RETURN_SPEED_FRAC`).
const HOME_RETURN_SPEED_FRAC: f64 = 0.5;

/// Waypoint-count ceiling for one `move_l` (bounds planning cost).
const MOVE_L_MAX_STEPS: usize = 400;
/// Waypoint-count ceiling for a multi-segment cartesian path — an arc, a
/// spline, a process move, or a blended chain of straight moves. Higher
/// than a single `move_l`'s because the path is that much longer; the
/// sampler spreads the budget over the whole path rather than sampling
/// each piece as if it were alone.
const CART_PATH_MAX_STEPS: usize = 3000;
/// Below this much translation AND rotation a cartesian move is already
/// at its target.
const MOVE_L_NULL_M: f64 = 1e-6;
/// How far a waypoint list's first pose may sit from where the arm
/// actually is before the current pose is PREPENDED to the path instead
/// of replacing that first waypoint \[m\].
///
/// parol6's rule (`commands/curved_commands.py`, `MoveSCommand` /
/// `MovePCommand`): a client that starts its waypoint list at the pose
/// it believes the arm is at means the path to begin there, and the
/// small FK/IK discrepancy must not become a spurious first segment; a
/// list that starts somewhere else means the arm to travel there first.
const WAYPOINT_SNAP_M: f64 = 5e-3;
/// Corner radius `move_p` rounds its interior waypoints with, as a
/// fraction of the shorter adjacent segment.
///
/// `move_p` is the one move whose corners are blended without the client
/// naming a radius — "process move — constant TCP speed, auto-blended
/// corners" is what `par6-proto` and the client API both promise — so
/// the radius has to come from the path itself. A quarter of the shorter
/// neighbour keeps half the segment straight on both sides of every
/// corner, which is the same shape as the zone clamp
/// ([`par6_motion::cart::corner_trims`]) applied to the largest radius a
/// corner could take.
const MOVE_P_AUTO_BLEND_FRAC: f64 = 0.25;

/// Joint-space pitch of the collision gate \[rad\]: consecutive checked
/// configurations along a planned path never differ by more than this on
/// any joint. At PAR6's ~0.45 m reach a 0.02 rad shoulder step sweeps the
/// wrist under 10 mm, so a keep-out thicker than that cannot be tunneled
/// through; the cost is bounded by the path's joint-space length rather
/// than by the sample count (a 90° single-joint move costs ~79 checks).
const COLLISION_STEP_RAD: f64 = 0.02;

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
    /// Quintic on the path coordinate: zero velocity AND acceleration at
    /// both ends, no cruise, no jerk limiting, point-to-point only.
    Quintic,
    /// Time-optimal path parameterization (toppra-cpp): the velocity and
    /// acceleration limits bind, nothing else.
    Toppra,
}

impl Profile {
    fn from_name(name: &str) -> Option<Self> {
        match name {
            "RUCKIG" => Some(Self::Ruckig),
            "TRAPEZOID" => Some(Self::Trapezoid),
            "QUINTIC" => Some(Self::Quintic),
            "TOPPRA" => Some(Self::Toppra),
            _ => None,
        }
    }
}

/// The profile registry the command plane advertises and validates
/// `select_profile` against.
pub(crate) fn profile_names() -> Vec<String> {
    let mut names = vec![
        "RUCKIG".to_owned(),
        "TRAPEZOID".to_owned(),
        "QUINTIC".to_owned(),
    ];
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

/// The tool action on the side channel. It owns no ring samples and no
/// planner state, which is what lets it run beside a motion.
struct ToolInFlight {
    server_index: u64,
    /// The settle epoch read before the command was sent. The RT bumps
    /// it when it arms, so a verdict still carrying this value belongs
    /// to the PREVIOUS action, not to ours.
    epoch_at_send: u32,
}

/// Which coordinate a cartesian path is timed against.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CartTiming {
    /// As fast as the joint limits allow (TOPPRA). What `move_l`,
    /// `move_c` and `move_s` promise.
    TimeOptimal,
    /// At a constant tool speed along the path. What `move_p` promises,
    /// and the reason it is a separate command.
    ConstantToolSpeed,
}

/// The planner's kinematics kit (feature `ffi`): its own model instance,
/// the enforced collision world, and the shared TCP-offset cell it
/// publishes into.
pub(crate) struct PlannerKin {
    pub(crate) kin: crate::kin::CartKin,
    pub(crate) collision: par6_kin::Collision,
    pub(crate) tool_offset: crate::kin::ToolOffset,
}

/// The `Planner` implementation `par6d` hands to the server.
pub(crate) struct Par6Planner {
    link: CoreLink,
    producer: SampleProducer,
    heartbeat: ExecHeartbeat,
    snapshots: SnapshotReader<StateSnapshot>,
    exec_limits: MotionLimits,
    /// The configured home pose \[rad\] — where a HOME on an already
    /// referenced arm returns to.
    home_pose_rad: [f64; MAX_JOINTS],
    dt: f64,
    ticks_per_s: f64,
    next_ring_index: u32,
    policy: par6_proto::CompletionPolicy,
    profile: Profile,
    tool: Option<ToolSpec>,
    inflight: Option<InFlight>,
    tool_inflight: Option<ToolInFlight>,
    enablement: Enablement,
    /// Latched near-singularity warning for the cart path in flight
    /// (vendor thresholds; STATUS `warnings` carries it).
    near_singularity: Option<WireError>,
    kin: crate::kin::CartKin,
    /// The enforced collision world. Planner-side by construction: coal's
    /// C++ narrow phase allocates on deep interpenetration, so no check
    /// may ever run on the RT thread.
    collision: par6_kin::Collision,
    /// Reporting names for the applied keep-out shapes.
    shape_names: ShapeNames,
    /// The pairs the last refused motion would have collided at — the
    /// `collision_active` / `collision_pairs` STATUS fields. Latched, not
    /// sampled: it describes the configuration a move was blocked AT, and
    /// the server drops it when it accepts the next motion command.
    collision_latch: CollisionState,
    /// Outcome of a command a world change invalidated mid-flight, handed
    /// to the server by the next [`Planner::poll`].
    invalidated: Option<CommandOutcome>,
    /// The commanded TCP offset, shared with the bridge's and
    /// housekeeping's models and with the RT FK hook.
    tool_offset: crate::kin::ToolOffset,
    /// Rate/change gate for the cartesian enablement probe.
    probe: EnablementProbe,
    /// The `[motion]` feel constants (sampling pitch, IK step guard,
    /// settle parameters).
    motion: par6_config::MotionConfig,
    /// The applied runtime payload, mirrored so `sync` only touches the
    /// model on a change.
    payload: par6_server::PayloadSpec,
}

impl Par6Planner {
    pub(crate) fn new(
        link: CoreLink,
        producer: SampleProducer,
        heartbeat: ExecHeartbeat,
        snapshots: SnapshotReader<StateSnapshot>,
        bundle: &ConfigBundle,
        models: PlannerKin,
    ) -> Result<Self, MotionError> {
        let exec_limits = MotionLimits::from_config(&bundle.robot, par6_config::LimitMode::Exec)?;
        let PlannerKin {
            kin,
            collision,
            tool_offset,
        } = models;
        let dt = bundle.robot.robot.tick_dt_s;
        let tool = bundle
            .active_gripper()
            .and_then(|g| g.driver.as_ref())
            .map(|d| ToolSpec { ilim_ma: d.ilim_ma });
        let mut home_pose_rad = [0.0; MAX_JOINTS];
        for (out, rad) in home_pose_rad
            .iter_mut()
            .zip(bundle.robot.robot.park_pose_rad.iter())
        {
            *out = *rad;
        }
        let motion = bundle.robot.motion;
        Ok(Self {
            link,
            producer,
            heartbeat,
            snapshots,
            exec_limits,
            near_singularity: None,
            home_pose_rad,
            dt,
            ticks_per_s: 1.0 / dt,
            next_ring_index: 1,
            policy: par6_proto::CompletionPolicy::Settled,
            profile: Profile::default(),
            tool,
            inflight: None,
            tool_inflight: None,
            // Nothing measured yet, and the wire has no "unknown": claim
            // no freedom until the first probe runs (the next poll).
            enablement: NO_FREEDOM,
            probe: EnablementProbe::new(
                Duration::from_secs_f64(
                    1.0 / f64::from(bundle.robot.protocol.status_rate_hz.max(1)),
                )
                .max(EN_MIN_PERIOD),
            ),
            tool_offset,
            kin,
            collision,
            shape_names: ShapeNames::default(),
            collision_latch: CollisionState::default(),
            invalidated: None,
            motion,
            payload: par6_server::PayloadSpec::default(),
        })
    }

    /// Refuse a planned path that would drive the arm into the collision
    /// world, before a single sample reaches the RT ring.
    ///
    /// `from` is the sample the arm will start at, so a world change can
    /// re-gate the REMAINDER of a running trajectory with the same rule.
    ///
    /// The path is walked at [`COLLISION_STEP_RAD`] joint-space pitch —
    /// the endpoints of a move are usually clear while its interior is
    /// not, and the samples ARE the trajectory the arm will run, so this
    /// gates what actually happens rather than a straight line between
    /// the two endpoints.
    ///
    /// Normally any collision along the path refuses the move. The
    /// exception is a path that STARTS in collision — a keep-out
    /// dropped on top of the arm. Refusing outright would trap the arm,
    /// so a move that adds no colliding pair the arm is not already in
    /// is allowed: a move may not CREATE a collision, it may leave one.
    /// (Self pairs the arm legitimately rests in are excluded
    /// model-side by the variant's SRDF, so they never reach this rule.)
    ///
    /// Leaving one is bounded by depth: from a start in world
    /// collision, a sample whose pair set stays inside the baseline is
    /// still refused when its deepest world penetration exceeds the
    /// start's (the hull-vs-world `world_distance` drops below the start
    /// value by more than [`ESCAPE_TOL_M`]) — the pair half alone cannot
    /// tell an escaping path from one grinding deeper through the same
    /// pair.
    ///
    /// Streaming (`jog_*` / `servo_*`) is gated separately at datagram
    /// admission in the bridge's `StreamGate`, which applies the same
    /// two-halves rule: the jog/servo ramp is integrated on the RT
    /// thread, where a coal check cannot go.
    fn gate_collisions<const W: usize>(
        &mut self,
        samples: &[[f64; W]],
        from: usize,
    ) -> Result<(), WireError> {
        let started = Instant::now();
        let q_now = self.snapshots.latest().q;
        // Disjoint field borrows: the name tables are read while the
        // collision model is being driven.
        let names = &self.shape_names;
        let col = &mut self.collision;
        let named = |report: &par6_kin::CollisionReport<'_>| -> Vec<(String, String)> {
            names.render(report)
        };
        let start_pairs = named(&col.check(&q_now, false).map_err(collision_error)?);
        // The depth half of the escape rule engages only when the start
        // penetrates a WORLD shape (a keep-out dropped over the arm) —
        // the case escape exists for, and the only case the signal
        // speaks about: `world_distance` covers world pairs only, so an
        // arm-arm start collision has no depth to watch and stays
        // guarded by the pair half — the move may not contact anything
        // new.
        let start_depth = if start_pairs
            .iter()
            .any(|p| is_world_name(&p.0) || is_world_name(&p.1))
        {
            Some(col.world_distance(&q_now).map_err(collision_error)?)
        } else {
            None
        };
        let baseline = start_pairs;

        let total = samples.len();
        let mut checked = 0usize;
        let mut last: Option<[f64; NQ]> = None;
        for (k, sample) in samples.iter().enumerate().skip(from) {
            let mut q = [0.0; NQ];
            q.copy_from_slice(&sample[..NQ]);
            let coarse = last.is_some_and(|prev| {
                q.iter()
                    .zip(prev.iter())
                    .all(|(a, b)| (a - b).abs() <= COLLISION_STEP_RAD)
            });
            // The last sample is where the arm comes to rest, so it is
            // checked however close it sits to its predecessor.
            if coarse && k + 1 != total {
                continue;
            }
            last = Some(q);
            checked += 1;
            let touching = named(&col.check(&q, false).map_err(collision_error)?);
            let offending: Vec<(String, String)> = touching
                .iter()
                .filter(|p| !baseline.contains(p))
                .cloned()
                .collect();
            if !offending.is_empty() {
                let pairs = format_pairs(&offending);
                log::info!("collision gate: rejected sample {k}/{total}: {pairs}");
                self.collision_latch = CollisionState {
                    active: true,
                    pairs: offending,
                };
                return Err(make_error(
                    ErrorCode::SysSelfCollision,
                    UNATTRIBUTED,
                    &[
                        ("sample", &k.to_string()),
                        ("total", &total.to_string()),
                        ("pairs", &pairs),
                    ],
                ));
            }
            if let Some(d0) = start_depth {
                let depth = col.world_distance(&q).map_err(collision_error)?;
                if depth < d0 - ESCAPE_TOL_M {
                    let pairs = format_pairs(&touching);
                    log::info!(
                        "collision gate: rejected sample {k}/{total}: \
                         penetration deepens ({depth:.4} m < start {d0:.4} m): {pairs}"
                    );
                    self.collision_latch = CollisionState {
                        active: true,
                        pairs: touching,
                    };
                    return Err(make_error(
                        ErrorCode::SysSelfCollision,
                        UNATTRIBUTED,
                        &[
                            ("sample", &k.to_string()),
                            ("total", &total.to_string()),
                            ("pairs", &pairs),
                        ],
                    ));
                }
            }
        }
        log::debug!(
            "collision gate: {checked} checks over samples {from}..{total} in {:.2} ms",
            started.elapsed().as_secs_f64() * 1e3
        );
        Ok(())
    }

    /// Re-gate the in-flight trajectory against a world that just
    /// changed, halting it where the new world makes its remainder
    /// illegal — a keep-out dropped onto a moving arm stops the move
    /// instead of being enforced only for the NEXT one.
    ///
    /// The remainder starts at the planned sample closest to where the
    /// arm actually is: the part already driven cannot be un-driven, and
    /// gating it would fail a move over a keep-out placed behind it.
    fn revalidate_inflight(&mut self) {
        let Some(InFlight {
            server_index,
            kind: InFlightKind::Exec { samples, .. },
            ..
        }) = &self.inflight
        else {
            return;
        };
        let index = *server_index;
        let q = self.snapshots.latest().q;
        let nearest = samples
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| joint_distance(&a.q, &q).total_cmp(&joint_distance(&b.q, &q)))
            .map_or(0, |(i, _)| i);
        let planned: Vec<[f64; 2 * MAX_JOINTS]> = samples
            .iter()
            .map(|s| {
                let mut qqd = [0.0; 2 * MAX_JOINTS];
                qqd[..MAX_JOINTS].copy_from_slice(&s.q);
                qqd[MAX_JOINTS..].copy_from_slice(&s.qd);
                qqd
            })
            .collect();
        if let Err(error) = self.gate_collisions(&planned, nearest) {
            log::warn!(
                "command {index} invalidated by a world change: {}",
                error.cause
            );
            self.discard_planned();
            self.invalidated = Some(CommandOutcome {
                index,
                error: Some(error),
                verdict: None,
            });
        }
    }

    /// Wrap fully-timed tick-dt samples as the next EXEC in-flight
    /// command: allocate a ring index, stamp the metadata, request EXEC.
    ///
    /// Two gates run first, and nothing is queued — no mode change is
    /// even requested — unless both pass: the path must not collide, and
    /// its commanded velocity steps must stay inside the acceleration
    /// limits. The second one guards every planner equally, because
    /// every planner can be internally consistent and still emit a
    /// stream the arm must not follow; see [`par6_motion::gate`].
    fn start_exec(
        &mut self,
        samples: Vec<[f64; 3 * MAX_JOINTS]>,
        seen_exec: bool,
    ) -> Result<InFlightKind, WireError> {
        par6_motion::gate::check_commanded_accel(
            samples.iter().map(|qqa| {
                let mut qd = [0.0; MAX_JOINTS];
                qd.copy_from_slice(&qqa[MAX_JOINTS..2 * MAX_JOINTS]);
                qd
            }),
            &self.exec_limits.acceleration,
            self.dt,
            par6_motion::gate::ACCEL_TOLERANCE,
        )
        .map_err(planning_error)?;
        self.gate_collisions(&samples, 0)?;
        // A fresh fill generation: a flush already queued for an earlier
        // command can no longer reach these samples, however far behind
        // the RT command queue is running.
        self.producer.begin_generation();
        let ring_index = self.next_ring_index;
        self.next_ring_index = self.next_ring_index.checked_add(1).unwrap_or(1);
        let n = samples.len();
        // The planned acceleration becomes the ring's torque feedforward
        // (`M(q)·q̈ + C(q,q̇)·q̇`; the law adds G(q) itself). A feedforward
        // is not a safety path: a model failure degrades to zero torque
        // for that sample and the PID loop carries the move.
        let kin = &mut self.kin;
        let mut id_failed = false;
        let samples: Vec<RingSample> = samples
            .into_iter()
            .enumerate()
            .map(|(k, qqa)| {
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
                s.q.copy_from_slice(&qqa[..MAX_JOINTS]);
                s.qd.copy_from_slice(&qqa[MAX_JOINTS..2 * MAX_JOINTS]);
                let mut qdd = [0.0; MAX_JOINTS];
                qdd.copy_from_slice(&qqa[2 * MAX_JOINTS..]);
                match kin.dyn_feedforward(&s.q, &s.qd, &qdd) {
                    Ok(tau) => {
                        for (out, t) in s.tau_ff.iter_mut().zip(tau.iter()) {
                            *out = *t as f32;
                        }
                    }
                    Err(e) if !id_failed => {
                        id_failed = true;
                        log::warn!("torque feedforward degraded to zero: {e}");
                    }
                    Err(_) => {}
                }
                s
            })
            .collect();
        self.link.send(RtCommand::SetMode(Mode::Exec));
        self.heartbeat.feed();
        Ok(InFlightKind::Exec {
            ring_index,
            samples,
            cursor: 0,
            seen_exec,
        })
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
        let samples = self.joint_move_samples(&snap.q, &target, duration, speed, accel)?;
        if samples.is_empty() {
            return Ok(InFlightKind::Instant);
        }
        self.start_exec(samples, snap.mode == Mode::Exec)
    }

    /// The tick-rate samples a joint-space move from `start` to `target`
    /// \[rad\] compiles to under the SELECTED profile. Empty = the move
    /// has no path to run (start and target are the same configuration).
    ///
    /// Planning without starting is also how a queued move is TIMED for
    /// the queue ETA, so this stays free of side effects.
    fn joint_move_samples(
        &self,
        start: &[f64; MAX_JOINTS],
        target: &[f64; MAX_JOINTS],
        duration: Option<f64>,
        speed: Option<f64>,
        accel: Option<f64>,
    ) -> Result<Vec<[f64; 3 * MAX_JOINTS]>, WireError> {
        let kind = match self.profile {
            Profile::Ruckig => ProfileKind::Ruckig,
            Profile::Trapezoid => ProfileKind::Trapezoid,
            Profile::Quintic => ProfileKind::Quintic,
            // TOPPRA times the straight joint-space path instead of
            // shaping a point-to-point profile: same waypoints, a
            // different (time-optimal) parameterization.
            Profile::Toppra => {
                self.exec_limits
                    .require_inside_soft(target)
                    .map_err(planning_error)?;
                // toppra needs a path to time; identical waypoints have none.
                if start
                    .iter()
                    .zip(target.iter())
                    .all(|(a, b)| (a - b).abs() < NULL_MOVE_RAD)
                {
                    return Ok(Vec::new());
                }
                let mut waypoints = Vec::with_capacity(2 * MAX_JOINTS);
                waypoints.extend_from_slice(start);
                waypoints.extend_from_slice(target);
                return self.toppra_samples(&waypoints, speed, accel, duration);
            }
        };
        let mut limits = self.exec_limits;
        if let Some(accel) = accel {
            for a in limits.acceleration.iter_mut() {
                *a *= accel;
            }
            // Jerk rides the acceleration fraction, matching the streaming
            // path (`MotionStream::set_scale`): a move asked to accelerate
            // gently that kept the full jerk ceiling would reach the lower
            // acceleration just as abruptly, which is the jolt the fraction
            // is asking to avoid. An infinite jerk stays infinite.
            for j in limits.jerk.iter_mut() {
                *j *= accel;
            }
        }
        let mut builder = ProgramBuilder::new(*start, limits, self.dt).map_err(planning_error)?;
        builder
            .move_j(
                *target,
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
        Ok(plan
            .samples()
            .iter()
            .map(|s| {
                let mut qqa = [0.0; 3 * MAX_JOINTS];
                qqa[..MAX_JOINTS].copy_from_slice(&s.q);
                qqa[MAX_JOINTS..2 * MAX_JOINTS].copy_from_slice(&s.qd);
                qqa[2 * MAX_JOINTS..].copy_from_slice(&s.qdd);
                qqa
            })
            .collect())
    }

    /// Joint-space blend-chain pitch: half the per-tick travel at the
    /// full EXEC velocity norm (the vendor rule), floored at 10 mrad;
    /// `motion.joint_step_rad` overrides.
    fn joint_step_rad(&self) -> f64 {
        self.motion.joint_step_rad.unwrap_or_else(|| {
            let norm = self
                .exec_limits
                .velocity
                .iter()
                .map(|v| v * v)
                .sum::<f64>()
                .sqrt();
            (0.5 * norm * self.dt).max(0.01)
        })
    }

    /// TOPPRA-time a joint waypoint list and sample it at tick dt.
    /// A requested `min_duration` is a minimum: TOPPRA's optimum bounds
    /// how fast the path can be driven, a longer request time-scales the
    /// whole trajectory (velocities scale with it, so limits still hold).
    fn toppra_samples(
        &self,
        waypoints: &[f64],
        speed: Option<f64>,
        accel: Option<f64>,
        min_duration: Option<f64>,
    ) -> Result<Vec<[f64; 3 * MAX_JOINTS]>, WireError> {
        self.toppra_samples_with(
            waypoints,
            speed,
            accel,
            min_duration,
            None,
            pinokin_sys::PathDegree::Cubic,
            None,
        )
    }

    /// The one solver call both cartesian lanes go through.
    ///
    /// `knots` places each waypoint on the path parameter (`None` spaces
    /// them evenly) and `max_path_speed` caps `ds/dt`. The path is
    /// degree-1 by default: the poses are already spaced a couple of
    /// millimetres apart by the resampler, and a spline through them
    /// would bow off the chain IK actually solved — inventing curvature,
    /// which is acceleration, between the samples that were checked.
    #[allow(clippy::too_many_arguments)]
    fn toppra_samples_with(
        &self,
        waypoints: &[f64],
        speed: Option<f64>,
        accel: Option<f64>,
        min_duration: Option<f64>,
        knots: Option<&[f64]>,
        degree: pinokin_sys::PathDegree,
        max_path_speed: Option<f64>,
    ) -> Result<Vec<[f64; 3 * MAX_JOINTS]>, WireError> {
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
        let traj = pinokin_sys::Trajectory::parameterize_with(
            waypoints,
            MAX_JOINTS,
            &vel,
            &acc,
            None,
            knots,
            degree,
            max_path_speed,
        )
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
            let mut qqa = [0.0; 3 * MAX_JOINTS];
            qqa[..MAX_JOINTS].copy_from_slice(&q);
            for (out, v) in qqa[MAX_JOINTS..2 * MAX_JOINTS].iter_mut().zip(qd.iter()) {
                *out = v * scale;
            }
            // A min-duration stretch is a time reparameterization t → t/scale:
            // velocities pick up one factor of `scale`, accelerations two.
            for (out, a) in qqa[2 * MAX_JOINTS..].iter_mut().zip(qdd.iter()) {
                *out = a * scale * scale;
            }
            samples.push(qqa);
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
    ) -> Result<u32, WireError> {
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
        // Read before sending: the RT arms on receipt and bumps the
        // epoch, which is how the verdict below is attributed.
        let epoch_at_send = snap.tool.epoch;
        match cmd.action.as_str() {
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
                // The RT gate never streams a move to an uncalibrated
                // gripper (the firmware's own gate drops it), so admitting
                // one here could only time out or trivially "succeed"
                // without moving a jaw.
                if !snap.gripper.reply.is_some_and(|r| r.calibrated) {
                    return Err(invalid(
                        "the gripper is not calibrated: run the calibrate action first".into(),
                    ));
                }
                self.link.send(RtCommand::Gripper(gripper_move_command(
                    position, speed, current,
                )));
            }
            "calibrate" => {
                if !cmd.params.is_empty() {
                    return Err(invalid("calibrate takes no parameters".into()));
                }
                self.link.send(RtCommand::GripperCalibrate);
            }
            // Halt in place: the RT re-targets the freshest reported jaw
            // byte with the standing command's speed/current (already in
            // tolerance, so it holds). Degrades to a release when nothing
            // is standing or the byte is out of range — an uncalibrated
            // gripper reports 0, which the firmware maps to fully open.
            // The wait is the stop's actual promise — the jaws are no
            // longer travelling — not the move-settle verdict, whose
            // object-detection term can predate any commanded motion.
            "stop" => {
                if !cmd.params.is_empty() {
                    return Err(invalid("stop takes no parameters".into()));
                }
                self.link.send(RtCommand::GripperStop);
            }
            // Release: action = 0 — limp on spectral-bldc, velocity-0
            // hold on stepfoc.
            "idle" => {
                if !cmd.params.is_empty() {
                    return Err(invalid("idle takes no parameters".into()));
                }
                self.link.send(RtCommand::GripperIdle);
            }
            other => {
                return Err(invalid(format!(
                    "tool '{}' has no action '{other}' (move | calibrate | stop | idle)",
                    cmd.tool_key
                )));
            }
        }
        Ok(epoch_at_send)
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

    /// The TCP pose the arm is standing at — where every cartesian move
    /// starts from.
    pub(crate) fn current_pose(&mut self, q: &[f64; MAX_JOINTS]) -> Result<Pose, WireError> {
        self.kin
            .fk(q)
            .map_err(|e| make_error(ErrorCode::MotnSetupFailed, UNATTRIBUTED, &[("detail", &e)]))
    }

    /// Seeded IK on one pose, with the cartesian failure vocabulary.
    fn ik_pose(&mut self, seed: &[f64; NQ], pose: &Pose) -> Result<[f64; NQ], WireError> {
        use crate::kin::IkResult;
        match self.kin.ik(seed, pose) {
            IkResult::Solved(q) => Ok(q),
            IkResult::Unreachable => Err(make_error(
                ErrorCode::IkTargetUnreachable,
                UNATTRIBUTED,
                &[(
                    "detail",
                    "The solver did not converge from the current configuration.",
                )],
            )),
            IkResult::Failed(e) => Err(make_error(
                ErrorCode::MotnSetupFailed,
                UNATTRIBUTED,
                &[("detail", &e)],
            )),
        }
    }

    /// Seeded IK along a pose chain, with the cartesian failure
    /// vocabulary and the per-sample soft-limit and branch-flip checks.
    ///
    /// Returns the joint waypoints row-major, plus the worst singularity
    /// measures seen along the way. `poses[0]` is where the arm already
    /// is, so it contributes the measured configuration rather than an
    /// IK solution.
    fn ik_path(
        &mut self,
        start_q: &[f64; MAX_JOINTS],
        poses: &[Pose],
    ) -> Result<(Vec<f64>, f64, f64), WireError> {
        let total = poses.len();
        let mut waypoints = Vec::with_capacity(total * MAX_JOINTS);
        waypoints.extend_from_slice(start_q);
        let mut seed = *start_q;
        let (mut worst_sigma, mut worst_cond) = (f64::INFINITY, 0.0f64);
        for (k, pose) in poses.iter().enumerate().skip(1) {
            let partial = || {
                make_error(
                    ErrorCode::IkPartialPath,
                    UNATTRIBUTED,
                    &[("valid", &k.to_string()), ("total", &total.to_string())],
                )
            };
            let q = match self.kin.ik(&seed, pose) {
                crate::kin::IkResult::Solved(q) => q,
                crate::kin::IkResult::Unreachable => return Err(partial()),
                crate::kin::IkResult::Failed(e) => {
                    return Err(make_error(
                        ErrorCode::MotnSetupFailed,
                        UNATTRIBUTED,
                        &[("detail", &e)],
                    ));
                }
            };
            if let Ok((sigma, cond)) = self.kin.singularity(&q) {
                worst_sigma = worst_sigma.min(sigma);
                worst_cond = worst_cond.max(cond);
            }
            for j in 0..MAX_JOINTS {
                if q[j] < self.exec_limits.soft_min[j] || q[j] > self.exec_limits.soft_max[j] {
                    return Err(make_error(
                        ErrorCode::CommValidationError,
                        UNATTRIBUTED,
                        &[(
                            "detail",
                            &format!(
                                "the path leaves joint {j}'s soft window at sample {k}/{total}"
                            ),
                        )],
                    ));
                }
                if (q[j] - seed[j]).abs() > self.motion.move_l_max_joint_step_rad {
                    return Err(partial());
                }
            }
            waypoints.extend_from_slice(&q);
            seed = q;
        }
        Ok((waypoints, worst_sigma, worst_cond))
    }

    /// The shared cartesian pipeline every cartesian move rides:
    /// pose list → seeded IK per pose → timing → ring samples at tick
    /// dt. Every failure (IK, branch flip, soft limits, timing) is a
    /// command error; nothing falls back to a joint-space move.
    ///
    /// `timing` picks which coordinate the path is timed against, and
    /// that is the only thing that differs between the cartesian moves:
    /// everything up to it is shared.
    ///
    /// `poses[0]` is where the arm already is, so it contributes the
    /// measured configuration rather than an IK solution.
    fn start_cart_path(
        &mut self,
        snap: &StateSnapshot,
        poses: &[Pose],
        speed: Option<f64>,
        accel: Option<f64>,
        duration: Option<f64>,
        timing: CartTiming,
    ) -> Result<InFlightKind, WireError> {
        use par6_motion::cart::LineSegment;

        let start_q = snap.q;
        let Some(target_pose) = poses.last() else {
            return Ok(InFlightKind::Instant);
        };
        let moved = poses.windows(2).any(|w| {
            let seg = LineSegment::new(&w[0], &w[1]);
            seg.length_m() >= MOVE_L_NULL_M || seg.angle_rad() >= MOVE_L_NULL_M
        });
        if !moved {
            return Ok(InFlightKind::Instant);
        }

        // The endpoint decides reachable-at-all before the path decides
        // reachable-along-the-way.
        self.ik_pose(&start_q, target_pose)?;

        let (waypoints, worst_sigma, worst_cond) = self.ik_path(&start_q, poses)?;

        // Every sample solved: the path runs — but a pass near a
        // singular configuration degrades cartesian accuracy, and the
        // operator hears about it (vendor thresholds; warning only).
        // Latched only once the motion is actually queued: a timing or
        // collision refusal below runs nothing, and a warning standing
        // in STATUS with nothing in flight would be attributed to
        // whatever move runs next.
        let samples = match timing {
            CartTiming::TimeOptimal => self.toppra_samples(&waypoints, speed, accel, duration)?,
            CartTiming::ConstantToolSpeed => {
                self.arclen_samples(&waypoints, poses, speed, accel, duration)?
            }
        };
        let kind = self.start_exec(samples, snap.mode == Mode::Exec)?;
        self.near_singularity = singularity_verdict(worst_sigma, worst_cond);
        Ok(kind)
    }

    /// MOVE_L: one straight cartesian segment.
    fn start_move_l(
        &mut self,
        cmd: &par6_proto::command::MoveL,
    ) -> Result<InFlightKind, WireError> {
        let snap = self.snapshots.latest();
        let start_pose = self.current_pose(&snap.q)?;
        let target = target_pose(&start_pose, &cmd.pose, cmd.frame, cmd.rel);
        let poses = par6_motion::cart::line(&start_pose, &target, line_sampling(&self.motion));
        self.start_cart_path(
            &snap,
            &poses,
            cmd.speed,
            cmd.accel,
            cmd.duration,
            CartTiming::TimeOptimal,
        )
    }

    /// MOVE_C: circular arc through the via pose to the end pose, with
    /// the circle derived from the three points.
    fn start_move_c(
        &mut self,
        cmd: &par6_proto::command::MoveC,
    ) -> Result<InFlightKind, WireError> {
        let snap = self.snapshots.latest();
        let start_pose = self.current_pose(&snap.q)?;
        let via = target_pose(&start_pose, &cmd.via, cmd.frame, cmd.rel);
        let end = target_pose(&start_pose, &cmd.end, cmd.frame, cmd.rel);
        let poses = par6_motion::cart::arc(&start_pose, &via, &end, path_sampling(&self.motion))
            .map_err(planning_error)?;
        self.start_cart_path(
            &snap,
            &poses,
            cmd.speed,
            cmd.accel,
            cmd.duration,
            CartTiming::TimeOptimal,
        )
    }

    /// MOVE_S: cubic spline through the waypoint list.
    fn start_move_s(
        &mut self,
        cmd: &par6_proto::command::MoveS,
    ) -> Result<InFlightKind, WireError> {
        let snap = self.snapshots.latest();
        let start_pose = self.current_pose(&snap.q)?;
        let waypoints = waypoint_poses(&start_pose, &cmd.waypoints, cmd.frame, cmd.rel);
        let poses = par6_motion::cart::spline(&waypoints, path_sampling(&self.motion))
            .map_err(planning_error)?;
        self.start_cart_path(
            &snap,
            &poses,
            cmd.speed,
            cmd.accel,
            cmd.duration,
            CartTiming::TimeOptimal,
        )
    }

    /// MOVE_P: process move — the waypoint list as straight segments
    /// with every interior corner rounded, so the TCP sweeps the path
    /// without stopping at a single waypoint.
    fn start_move_p(
        &mut self,
        cmd: &par6_proto::command::MoveP,
    ) -> Result<InFlightKind, WireError> {
        use par6_motion::cart::LineSegment;
        let snap = self.snapshots.latest();
        let start_pose = self.current_pose(&snap.q)?;
        let waypoints = waypoint_poses(&start_pose, &cmd.waypoints, cmd.frame, cmd.rel);
        let lengths: Vec<f64> = waypoints
            .windows(2)
            .map(|w| LineSegment::new(&w[0], &w[1]).length_m())
            .collect();
        let radii: Vec<f64> = lengths
            .windows(2)
            .map(|w| MOVE_P_AUTO_BLEND_FRAC * w[0].min(w[1]))
            .collect();

        let poses =
            par6_motion::cart::blended_polyline(&waypoints, &radii, path_sampling(&self.motion))
                .map_err(planning_error)?;
        self.start_cart_path(
            &snap,
            &poses,
            cmd.speed,
            cmd.accel,
            cmd.duration,
            CartTiming::ConstantToolSpeed,
        )
    }

    /// Time a cartesian path so the TOOL crosses it at a constant
    /// speed, rather than as fast as the joints allow.
    ///
    /// Same solver as every other cartesian move; two things differ.
    /// The knots sit at cumulative tool distance rather than at even
    /// spacing, so the path parameter IS tool distance — and `ds/dt` is
    /// then capped at one value for the whole path, which is what holds
    /// the tool to a single speed instead of letting it run away over
    /// the stretches where the joints have room. That cap is the
    /// fastest constant the steepest part of the path allows, so this is
    /// never faster than the time-optimal answer and usually slower.
    /// That is what MOVE_P promises and what the others do not.
    fn arclen_samples(
        &self,
        waypoints: &[f64],
        poses: &[Pose],
        speed: Option<f64>,
        accel: Option<f64>,
        duration: Option<f64>,
    ) -> Result<Vec<[f64; 3 * MAX_JOINTS]>, WireError> {
        use par6_motion::cart::LineSegment;
        let steps: Vec<(f64, f64)> = poses
            .windows(2)
            .map(|w| {
                let seg = LineSegment::new(&w[0], &w[1]);
                (seg.length_m(), seg.angle_rad())
            })
            .collect();
        let cart_s = par6_motion::arclen::tool_arc_lengths(&steps, PATH_ROT_WEIGHT_M_PER_RAD);
        let q: Vec<[f64; MAX_JOINTS]> = waypoints
            .chunks_exact(MAX_JOINTS)
            .map(|c| {
                let mut a = [0.0; MAX_JOINTS];
                a.copy_from_slice(c);
                a
            })
            .collect();
        let no_extent = || {
            make_error(
                ErrorCode::TrajNoSteps,
                UNATTRIBUTED,
                &[(
                    "detail",
                    "the path covers no tool distance to hold a speed along",
                )],
            )
        };
        let knots = par6_motion::arclen::ArcKnots::new(&q, &cart_s).ok_or_else(no_extent)?;
        let cap = par6_motion::arclen::max_path_speed(
            &knots.max_slope(),
            &self.exec_limits,
            speed.unwrap_or(1.0),
        )
        .ok_or_else(no_extent)?;
        self.toppra_samples_with(
            &knots.waypoints_flat(),
            speed,
            accel,
            duration,
            Some(knots.knots()),
            pinokin_sys::PathDegree::Cubic,
            Some(cap),
        )
    }

    /// A chain of `move_l`s linked by blend radii, planned as ONE
    /// cartesian path whose interior corners are rounded.
    ///
    /// Each move's target resolves against its PREDECESSOR's target, not
    /// against the live pose: a relative or tool-frame move in the
    /// middle of a chain means "from where the move before it ends",
    /// which is where the arm will be (parol6 does the same in
    /// `commands/cartesian_commands.py`, `do_setup_with_blend`).
    ///
    /// The chain runs under the slowest speed and acceleration fraction
    /// in it; durations add up when every move carries one, and are
    /// dropped when they are mixed with speed-parameterised moves —
    /// there is no meaningful total otherwise.
    fn start_move_l_chain(
        &mut self,
        chain: &[&par6_proto::command::MoveL],
    ) -> Result<InFlightKind, WireError> {
        use par6_motion::cart::LineSegment;

        let snap = self.snapshots.latest();
        let start_pose = self.current_pose(&snap.q)?;
        let mut waypoints = Vec::with_capacity(chain.len() + 1);
        waypoints.push(start_pose);
        for cmd in chain {
            let previous = *waypoints.last().expect("seeded with the start pose");
            waypoints.push(target_pose(&previous, &cmd.pose, cmd.frame, cmd.rel));
        }
        let radii: Vec<f64> = chain[..chain.len() - 1]
            .iter()
            .map(|c| c.blend_radius.unwrap_or(0.0).max(0.0) / 1000.0)
            .collect();
        let poses =
            par6_motion::cart::blended_polyline(&waypoints, &radii, path_sampling(&self.motion))
                .map_err(planning_error)?;

        let speed = chain
            .iter()
            .filter_map(|c| c.speed)
            .fold(None::<f64>, |acc, s| Some(acc.map_or(s, |a: f64| a.min(s))));
        let accel = chain
            .iter()
            .filter_map(|c| c.accel)
            .fold(None::<f64>, |acc, a| Some(acc.map_or(a, |x: f64| x.min(a))));
        let duration = chain
            .iter()
            .try_fold(0.0, |acc, c| c.duration.map(|d| acc + d))
            .filter(|_| chain.iter().all(|c| c.duration.is_some()));
        log::debug!(
            "blended cartesian chain: {} moves, {} poses, {:.1} mm of path",
            chain.len(),
            poses.len(),
            poses
                .windows(2)
                .map(|w| LineSegment::new(&w[0], &w[1]).length_m())
                .sum::<f64>()
                * 1e3
        );
        self.start_cart_path(
            &snap,
            &poses,
            speed,
            accel,
            duration,
            CartTiming::TimeOptimal,
        )
    }

    /// A chain of joint-space moves (`move_j` / `move_j_pose`) linked by
    /// blend radii, planned as ONE joint path whose interior corners are
    /// rounded.
    ///
    /// The radius is a CARTESIAN quantity and a joint segment has no
    /// length in millimetres, so each corner's zone is sized by the TCP
    /// distance between the waypoints it joins — FK at the waypoints
    /// turns `r` into the fraction of each adjacent joint segment the
    /// zone eats. Same conversion as parol6
    /// (`commands/joint_commands.py`, `do_setup_with_blend`).
    fn start_joint_chain(&mut self, chain: &[JointTarget<'_>]) -> Result<InFlightKind, WireError> {
        let snap = self.snapshots.latest();
        let mut waypoints: Vec<[f64; NQ]> = Vec::with_capacity(chain.len() + 1);
        waypoints.push(snap.q);
        for target in chain {
            let previous = *waypoints.last().expect("seeded with the measured pose");
            let q = match target.goal {
                JointGoal::Angles { angles, rel } => {
                    let mut q = [0.0; NQ];
                    for (j, out) in q.iter_mut().enumerate() {
                        let a = angles[j].to_radians();
                        *out = if rel { previous[j] + a } else { a };
                    }
                    q
                }
                JointGoal::Pose(pose) => {
                    let m = crate::kin::wire_pose_to_matrix(pose);
                    self.ik_pose(&previous, &m)?
                }
            };
            self.exec_limits
                .require_inside_soft(&q)
                .map_err(planning_error)?;
            waypoints.push(q);
        }

        let mut tcp = Vec::with_capacity(waypoints.len());
        for q in &waypoints {
            let pose = self.current_pose(q)?;
            tcp.push(par6_motion::cart::translation(&pose));
        }
        let distance = |a: [f64; 3], b: [f64; 3]| {
            ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
        };
        let fracs: Vec<(f64, f64)> = (1..waypoints.len() - 1)
            .map(|i| {
                let r = chain[i - 1].blend_radius.unwrap_or(0.0).max(0.0) / 1000.0;
                let before = distance(tcp[i - 1], tcp[i]);
                let after = distance(tcp[i], tcp[i + 1]);
                (
                    if before > 1e-9 { r / before } else { 0.0 },
                    if after > 1e-9 { r / after } else { 0.0 },
                )
            })
            .collect();

        let path = par6_motion::cart::blended_polyline_joint(
            &waypoints,
            &fracs,
            self.joint_step_rad(),
            CART_PATH_MAX_STEPS,
        )
        .map_err(planning_error)?;
        let mut flat = Vec::with_capacity(path.len() * MAX_JOINTS);
        for q in &path {
            flat.extend_from_slice(q);
        }
        let speed = chain
            .iter()
            .filter_map(|c| c.speed)
            .fold(None::<f64>, |acc, s| Some(acc.map_or(s, |a: f64| a.min(s))));
        let accel = chain
            .iter()
            .filter_map(|c| c.accel)
            .fold(None::<f64>, |acc, a| Some(acc.map_or(a, |x: f64| x.min(a))));
        let duration = chain
            .iter()
            .try_fold(0.0, |acc, c| c.duration.map(|d| acc + d))
            .filter(|_| chain.iter().all(|c| c.duration.is_some()));
        let samples = self.toppra_samples(&flat, speed, accel, duration)?;
        self.start_exec(samples, snap.mode == Mode::Exec)
    }

    /// Plan `cmd`, looking at the queue standing behind it (`rest`, in
    /// order) for moves it can blend with. Returns what is now in flight
    /// and how many commands it covers — 1 unless a blend chain formed.
    fn plan(
        &mut self,
        cmd: &Command,
        rest: &[QueuedCommand<'_>],
    ) -> Result<(InFlightKind, usize), WireError> {
        // Rounding a corner means re-planning both of its segments as
        // one path, which takes IK and TOPPRA.
        if let Some(consumed) = self.blend_chain_len(cmd, rest) {
            let kind = match cmd {
                Command::MoveL(head) => {
                    let mut chain = vec![head];
                    chain.extend(rest[..consumed].iter().map(|q| match q.cmd {
                        Command::MoveL(p) => p,
                        _ => unreachable!("the chain only accepts move_l"),
                    }));
                    self.start_move_l_chain(&chain)?
                }
                _ => {
                    let mut chain = vec![JointTarget::of(cmd).expect("a joint move")];
                    chain.extend(
                        rest[..consumed]
                            .iter()
                            .map(|q| JointTarget::of(q.cmd).expect("a joint move")),
                    );
                    self.start_joint_chain(&chain)?
                }
            };
            return Ok((kind, consumed + 1));
        }
        let kind = match cmd {
            Command::MoveJ(p) => self.start_move_j(p)?,
            Command::Home(p) => {
                let snap = self.snapshots.latest();
                if snap.homed && !p.calibrate {
                    // An arm that already holds its references does not
                    // need them re-established: HOME is a normal planned
                    // return to the configured home pose, which is what
                    // makes a Home button press cost seconds instead of
                    // a full referencing seek (parol6 routes an
                    // already-referenced `HomeCmd` to exactly this move,
                    // `server/motion_planner.py:239-241`). `calibrate`
                    // asks for the seek regardless.
                    self.start_joint_move(
                        &snap,
                        self.home_pose_rad,
                        None,
                        Some(HOME_RETURN_SPEED_FRAC),
                        None,
                    )?
                } else {
                    // The RT core only enters Homing from Idle; after a
                    // completed planned move it is still holding in Exec.
                    self.link.send(RtCommand::SetMode(Mode::Idle));
                    self.link.send(RtCommand::SetMode(Mode::Homing));
                    InFlightKind::Home { seen_homing: false }
                }
            }
            Command::Delay(p) => {
                let snap = self.snapshots.latest();
                let ticks = (p.seconds * self.ticks_per_s).round().max(1.0) as u64;
                InFlightKind::Delay {
                    target_tick: snap.tick + ticks,
                }
            }
            Command::Checkpoint(_) | Command::SelectTool(_) => InFlightKind::Instant,
            Command::MoveJPose(p) => self.start_move_j_pose(p)?,
            Command::MoveL(p) => self.start_move_l(p)?,
            Command::MoveC(p) => self.start_move_c(p)?,
            Command::MoveS(p) => self.start_move_s(p)?,
            Command::MoveP(p) => self.start_move_p(p)?,
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
        Ok((kind, 1))
    }

    /// How many of the commands behind `cmd` blend into it, or `None`
    /// when `cmd` does not start a chain.
    ///
    /// A chain grows while the move already in it asks for a rounded
    /// corner (positive blend radius) AND the next queued command is a
    /// move of the SAME family — straight cartesian moves round corners
    /// against straight cartesian moves, joint moves against joint
    /// moves. Anything else (an arc, a delay, a move with no radius)
    /// ends the chain: the arm stops at that target, which is exactly
    /// what "no blend radius" asks for. A tool action cannot end one —
    /// it runs on the side channel and never joins the queue, so a
    /// gripper command between two blended moves no longer breaks the
    /// corner it had no reason to break.
    ///
    /// A positive radius on the LAST move of a chain has nothing to
    /// round — there is no following segment — so that move stops at its
    /// target like any other. That is also what a lone blended move
    /// does after the server's blend hold expires.
    fn blend_chain_len(&self, cmd: &Command, rest: &[QueuedCommand<'_>]) -> Option<usize> {
        let cartesian = matches!(cmd, Command::MoveL(_));
        let same_family = |c: &Command| {
            if cartesian {
                matches!(c, Command::MoveL(_))
            } else {
                matches!(c, Command::MoveJ(_) | Command::MoveJPose(_))
            }
        };
        if !cartesian && JointTarget::of(cmd).is_none() {
            return None;
        }
        let mut previous = cmd;
        let mut n = 0usize;
        while par6_server::blend_radius_mm(previous).is_some_and(|r| r > 0.0) {
            let Some(next) = rest.get(n).filter(|q| same_family(q.cmd)) else {
                break;
            };
            previous = next.cmd;
            n += 1;
        }
        (n > 0).then_some(n)
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

    /// Poll-time verdict for the in-flight command; `None` = keep going,
    /// `Ok(Some(_))` = success with a tool settle verdict to report.
    fn verdict(
        &self,
        fl: &mut InFlight,
        snap: &StateSnapshot,
    ) -> Option<Result<Option<u8>, WireError>> {
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
                    return Some(Ok(None));
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
                        Some(Ok(None))
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
            InFlightKind::Delay { target_tick } => (snap.tick >= *target_tick).then_some(Ok(None)),
            InFlightKind::Instant => Some(Ok(None)),
        }
    }

    /// Recompute the enablement flags for STATUS and the REACHABLE query.
    ///
    /// Rate- and change-gated ([`EnablementProbe`]): the cartesian half
    /// costs 24 seeded IK solves and a collision check per solution, which
    /// belongs nowhere near the RT thread and not on every 500 Hz planner
    /// poll either — and a configuration that has not moved cannot have
    /// changed its answer.
    fn update_enablement(&mut self, snap: &StateSnapshot) {
        if !self.probe.due(&snap.q) {
            return;
        }
        // POSITIVE slot first (`[j1+, j1−, …]`) — the order the waldoctl
        // frontend unpacks and parol6 publishes; filling the pair the
        // other way round greys out the opposite jog button. The margin
        // is parol6's: a joint a fraction of a degree from its stop has
        // no usable freedom left, so the button greys before the jog is
        // refused.
        let mut en = NO_FREEDOM;
        for j in 0..MAX_JOINTS {
            en.joint_en[2 * j] =
                u8::from(snap.q[j] + EN_JOINT_DELTA_RAD <= self.exec_limits.soft_max[j]);
            en.joint_en[2 * j + 1] =
                u8::from(snap.q[j] - EN_JOINT_DELTA_RAD >= self.exec_limits.soft_min[j]);
        }
        self.probe_directions(snap, &mut en);
        self.enablement = en;
    }

    /// Fill the cartesian slots, and withdraw joint directions the
    /// collision world blocks.
    ///
    /// Per parol6's IK worker: each of the twelve directions gets a
    /// [`EN_STEP_M`] / [`EN_STEP_RAD`] delta transform applied `dT·T`
    /// (world frame) or `T·dT` (tool frame), and the slot is whether
    /// seeded IK at the measured `q` reaches it. A solution outside the
    /// EXEC soft window does not count — that is where par6 refuses the
    /// motion — and neither does one the collision world rejects.
    fn probe_directions(&mut self, snap: &StateSnapshot, en: &mut Enablement) {
        let started = Instant::now();
        let mut q = [0.0; NQ];
        q.copy_from_slice(&snap.q[..NQ]);
        let pose = match self.kin.fk(&q) {
            Ok(p) => p,
            Err(e) => {
                log::warn!("enablement: FK failed at the measured pose ({e}); no freedom reported");
                return;
            }
        };
        let baseline = match self.baseline_pairs(&q) {
            Ok(b) => b,
            Err(e) => {
                log::warn!("enablement: collision baseline failed ({e}); no freedom reported");
                return;
            }
        };

        for wrf in [true, false] {
            let mut slots = [0u8; EN_SLOTS];
            for (slot, out) in slots.iter_mut().enumerate() {
                let axis = slot / 2;
                let step = if axis < 3 { EN_STEP_M } else { EN_STEP_RAD };
                let d = crate::kin::axis_delta(axis, if slot % 2 == 0 { step } else { -step });
                let target = if wrf {
                    crate::kin::mat_mul(&d, &pose)
                } else {
                    crate::kin::mat_mul(&pose, &d)
                };
                *out = u8::from(self.direction_free(&q, &target, &baseline));
            }
            if wrf {
                en.cart_en_wrf = slots;
            } else {
                en.cart_en_trf = slots;
            }
        }

        // A joint direction whose small step drives into the world is not
        // free either, whatever the limits say.
        for j in 0..NQ {
            for (slot, sign) in [(2 * j, 1.0), (2 * j + 1, -1.0)] {
                if en.joint_en[slot] == 0 {
                    continue;
                }
                let mut step = q;
                step[j] = (q[j] + sign * EN_JOINT_STEP_RAD)
                    .clamp(self.exec_limits.soft_min[j], self.exec_limits.soft_max[j]);
                en.joint_en[slot] = u8::from(self.adds_no_collision(&step, &baseline));
            }
        }
        // Trace, not debug: this fires on a timer whenever the arm is
        // moving, and a per-probe line at debug drowns the log the
        // one-per-move collision-gate timing lives in.
        log::trace!(
            "enablement probe: {:.2} ms",
            started.elapsed().as_secs_f64() * 1e3
        );
    }

    /// The colliding pairs the arm is ALREADY in — the ones a probed
    /// direction may keep without being blocked for them. Same escape rule
    /// as the planner's collision gate: a direction may not CREATE a
    /// collision, it may leave one.
    fn baseline_pairs(
        &mut self,
        q: &[f64; NQ],
    ) -> Result<Vec<(String, String)>, par6_kin::KinError> {
        let names = &self.shape_names;
        let col = &mut self.collision;
        Ok(names.render(&col.check(q, false)?))
    }

    /// Whether a small step to `target` is reachable, inside the soft
    /// window, and clear of the collision world.
    fn direction_free(
        &mut self,
        seed: &[f64; NQ],
        target: &par6_kin::Pose,
        baseline: &[(String, String)],
    ) -> bool {
        let solved = match self.kin.ik_within(seed, target, EN_IK_ITERS) {
            crate::kin::IkResult::Solved(q) => q,
            crate::kin::IkResult::Unreachable => return false,
            crate::kin::IkResult::Failed(e) => {
                log::warn!("enablement: IK call failed ({e})");
                return false;
            }
        };
        let inside = (0..NQ).all(|j| {
            solved[j] >= self.exec_limits.soft_min[j] && solved[j] <= self.exec_limits.soft_max[j]
        });
        inside && self.adds_no_collision(&solved, baseline)
    }

    /// Whether `q` collides in no pair the arm is not already in.
    fn adds_no_collision(&mut self, q: &[f64; NQ], baseline: &[(String, String)]) -> bool {
        let names = &self.shape_names;
        let col = &mut self.collision;
        match col.check(q, false) {
            Ok(report) => report.pairs().all(|(a, b)| {
                let (a, b) = (names.display(a), names.display(b));
                baseline.iter().any(|(x, y)| x == a && y == b)
            }),
            Err(e) => {
                log::warn!("enablement: collision check failed ({e})");
                false
            }
        }
    }
}

/// The vendor's rotation weight in the combined path metric \[m/rad\].
const PATH_ROT_WEIGHT_M_PER_RAD: f64 = 0.15;

/// Sampling of a single straight `move_l`.
fn line_sampling(motion: &par6_config::MotionConfig) -> par6_motion::cart::CartSampling {
    par6_motion::cart::CartSampling {
        step_m: motion.cart_step_m,
        rotation: par6_motion::cart::RotationPitch::Independent(motion.cart_step_rad),
        max_points: MOVE_L_MAX_STEPS + 1,
    }
}

/// Sampling of a multi-segment cartesian path (arc, spline, process
/// move, blended chain): the vendor's much finer pitch on the combined
/// translation+rotation metric, with a budget sized for the longer path.
fn path_sampling(motion: &par6_config::MotionConfig) -> par6_motion::cart::CartSampling {
    par6_motion::cart::CartSampling {
        step_m: motion.path_step_m,
        rotation: par6_motion::cart::RotationPitch::Weighted(PATH_ROT_WEIGHT_M_PER_RAD),
        max_points: CART_PATH_MAX_STEPS,
    }
}

/// The vendor's near-singularity thresholds: a path is flagged when its
/// worst sample's jacobian condition exceeds 1000 or its smallest
/// singular value drops under 1e-4 (condition capped at 1e12 upstream).
fn singularity_verdict(worst_sigma: f64, worst_cond: f64) -> Option<WireError> {
    if worst_cond > 1000.0 || worst_sigma < 1e-4 {
        Some(make_error(
            ErrorCode::TrajNearSingularity,
            UNATTRIBUTED,
            &[
                ("cond", &format!("{worst_cond:.0}")),
                ("sigma", &format!("{worst_sigma:.6}")),
            ],
        ))
    } else {
        None
    }
}

/// Where a cartesian move's wire pose puts the TCP, resolved against the
/// pose the move starts from.
fn target_pose(start: &Pose, wire_pose: &[f64; 6], frame: par6_proto::Frame, rel: bool) -> Pose {
    use par6_proto::Frame;
    let wire = crate::kin::wire_pose_to_matrix(wire_pose);
    match (frame, rel) {
        (Frame::Wrf, false) => wire,
        // World-frame delta: translation adds, rotation applies about
        // the world axes.
        (Frame::Wrf, true) => {
            let mut t = crate::kin::mat_mul(&wire, start);
            t[3] = start[3] + wire[3];
            t[7] = start[7] + wire[7];
            t[11] = start[11] + wire[11];
            t
        }
        // A tool-frame pose is inherently relative to the tool frame the
        // move starts in.
        (Frame::Trf, _) => crate::kin::mat_mul(start, &wire),
    }
}

/// A waypoint list as poses, starting at where the arm is.
///
/// TRF waypoints are all resolved against the STARTING tool frame — the
/// list describes one shape in one frame, not a chain of successive
/// tool-relative hops (parol6's `_transform_waypoints_trf_to_wrf`).
/// `rel` waypoints resolve the same way: every delta is against the
/// START pose, never chained onto the previous waypoint.
///
/// The first waypoint is replaced by the measured pose when it is within
/// [`WAYPOINT_SNAP_M`] of it, and the measured pose is prepended
/// otherwise: a client that starts its list where it believes the arm is
/// gets its shape, not that shape plus a millimetre-long lead-in
/// segment.
fn waypoint_poses(
    start: &Pose,
    waypoints: &[[f64; 6]],
    frame: par6_proto::Frame,
    rel: bool,
) -> Vec<Pose> {
    use par6_motion::cart::translation;
    let mut poses = Vec::with_capacity(waypoints.len() + 1);
    poses.push(*start);
    let mut wire = waypoints
        .iter()
        .map(|w| target_pose(start, w, frame, rel))
        .peekable();
    if let Some(first) = wire.peek() {
        let (a, b) = (translation(first), translation(start));
        let far = ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
            > WAYPOINT_SNAP_M;
        if !far {
            wire.next();
        }
    }
    poses.extend(wire);
    poses
}

/// One move of a joint-space blend chain: where it goes, and the
/// parameters the chain has to reconcile.
struct JointTarget<'a> {
    goal: JointGoal<'a>,
    blend_radius: Option<f64>,
    speed: Option<f64>,
    accel: Option<f64>,
    duration: Option<f64>,
}

#[derive(Clone, Copy)]
enum JointGoal<'a> {
    /// `move_j`: joint angles \[deg\], absolute or relative to where the
    /// preceding move ends.
    Angles {
        angles: &'a [f64; MAX_JOINTS],
        rel: bool,
    },
    /// `move_j_pose`: a cartesian target reached by IK, travelled in
    /// joint space.
    Pose(&'a [f64; 6]),
}

impl<'a> JointTarget<'a> {
    fn of(cmd: &'a Command) -> Option<Self> {
        match cmd {
            Command::MoveJ(p) => Some(Self {
                goal: JointGoal::Angles {
                    angles: &p.angles,
                    rel: p.rel,
                },
                blend_radius: p.blend_radius,
                speed: p.speed,
                accel: p.accel,
                duration: p.duration,
            }),
            Command::MoveJPose(p) => Some(Self {
                goal: JointGoal::Pose(&p.pose),
                blend_radius: p.blend_radius,
                speed: p.speed,
                accel: p.accel,
                duration: p.duration,
            }),
            _ => None,
        }
    }
}

/// Nothing is known to be free — the state the flags start in and fall
/// back to. The wire slots are 0/1 with no "unknown" spelling, and 1
/// means "you may move that way", so an unmeasured direction reports 0.
const NO_FREEDOM: Enablement = Enablement {
    joint_en: [0; EN_SLOTS],
    cart_en_wrf: [0; EN_SLOTS],
    cart_en_trf: [0; EN_SLOTS],
};

/// Translation probe step for cartesian enablement \[m\] (parol6: 0.5 mm).
const EN_STEP_M: f64 = 0.0005;
/// Rotation probe step for cartesian enablement \[rad\] (parol6: 0.5°).
const EN_STEP_RAD: f64 = 0.5 * std::f64::consts::PI / 180.0;
/// Iteration budget for one probe solve (parol6's solver runs 20 per
/// attempt). A 0.5 mm / 0.5° step off the measured configuration
/// converges in a handful of iterations when it converges at all, so the
/// full planning budget would be spent only on the directions that have
/// no answer — which is exactly where the probe must stay cheap.
const EN_IK_ITERS: i32 = 20;
/// Joint probe step for the collision half of the joint gate \[rad\]
/// (parol6: 2°) — big enough that a step actually enters what it is about
/// to enter, and clamped into the soft window so a pose past the stop
/// cannot grey a button the jog could still use.
const EN_JOINT_STEP_RAD: f64 = 2.0 * std::f64::consts::PI / 180.0;
/// Floor on the enablement probe's period.
///
/// parol6 recomputes enablement at the status cadence, but it does so in a
/// separate process; par6 computes it on the command-plane task, between
/// feeding the EXEC link heartbeat and filling the RT sample ring.
/// Measured on the sim rig, one probe costs ~6 ms typically and up to
/// ~50 ms where most directions have no IK solution — at a 50 Hz status
/// rate that is most of the task's budget, so the probe is capped here
/// instead. It still refreshes far faster than an operator can act on a
/// jog button, and it only runs at all while the arm is moving.
const EN_MIN_PERIOD: Duration = Duration::from_millis(100);
/// Margin a joint must still have inside its soft window to count as free
/// \[rad\] (parol6: 0.2° against `qlim`).
const EN_JOINT_DELTA_RAD: f64 = 0.2 * std::f64::consts::PI / 180.0;

/// Rate/change gate for [`Par6Planner::update_enablement`].
///
/// parol6 recomputes enablement at the status cadence and skips
/// submission entirely while the measured configuration is unchanged. par6
/// keeps the change gate — a still arm's answer cannot change on its own,
/// so a still arm pays nothing — and floors the period at
/// [`EN_MIN_PERIOD`], because par6 runs the probe on the command-plane
/// task rather than in a separate process.
struct EnablementProbe {
    period: Duration,
    due_at: Option<Instant>,
    last_q: Option<[f64; MAX_JOINTS]>,
}

impl EnablementProbe {
    fn new(period: Duration) -> Self {
        Self {
            period,
            due_at: None,
            last_q: None,
        }
    }

    fn due(&mut self, q: &[f64; MAX_JOINTS]) -> bool {
        let now = Instant::now();
        if self.due_at.is_some_and(|t| now < t) || self.last_q == Some(*q) {
            return false;
        }
        self.last_q = Some(*q);
        self.due_at = Some(now + self.period);
        true
    }

    /// Something other than the configuration changed what the answer
    /// would be (the collision world, the TCP the probe measures from):
    /// recompute at the next poll even though the arm has not moved.
    /// Only the cartesian half has such inputs, so only `ffi` builds
    /// have a reason to call it.
    fn invalidate(&mut self) {
        self.due_at = None;
        self.last_q = None;
    }
}

/// What the in-flight command would do to the arm — the offline preview's
/// read on a plan it will never execute.
pub(crate) enum PlannedMotion<'a> {
    /// A sampled trajectory (tick-dt EXEC samples).
    Exec(&'a [RingSample]),
    /// The homing sequence; on a referenced arm it lands at the
    /// configured home pose.
    Home,
    /// No motion (tool actions, delays, checkpoints, null moves).
    Still,
}

impl Par6Planner {
    /// The in-flight command's planned motion, for the offline preview.
    pub(crate) fn planned_motion(&self) -> PlannedMotion<'_> {
        match &self.inflight {
            Some(InFlight {
                kind: InFlightKind::Exec { samples, .. },
                ..
            }) => PlannedMotion::Exec(samples),
            Some(InFlight {
                kind: InFlightKind::Home { .. },
                ..
            }) => PlannedMotion::Home,
            _ => PlannedMotion::Still,
        }
    }

    /// The configured home pose \[rad\].
    pub(crate) fn home_pose(&self) -> [f64; MAX_JOINTS] {
        self.home_pose_rad
    }
}

impl Planner for Par6Planner {
    fn start(&mut self, batch: &[QueuedCommand<'_>]) -> Result<usize, WireError> {
        let Some(head) = batch.first() else {
            return Err(make_error(
                ErrorCode::CommValidationError,
                UNATTRIBUTED,
                &[("detail", "the planner was started with an empty batch")],
            ));
        };
        let (kind, consumed) = self.plan(head.cmd, &batch[1..])?;
        self.inflight = Some(InFlight {
            server_index: head.index,
            started: Instant::now(),
            kind,
        });
        self.pump_ring();
        Ok(consumed)
    }

    fn poll(&mut self) -> Option<CommandOutcome> {
        let snap = self.snapshots.latest();
        // While a planned trajectory is running, this task's budget belongs
        // to the EXEC link watchdog and the sample ring, both fed from
        // here — so the enablement probe, the one expensive thing on this
        // loop, stands down for the duration. Nothing is lost: a client
        // cannot jog through a running move (a streamable cancels the
        // queue), and the configuration the move ends in is a change, so
        // the probe runs the moment it finishes.
        if matches!(
            self.inflight,
            Some(InFlight {
                kind: InFlightKind::Exec { .. },
                ..
            })
        ) {
            self.heartbeat.feed();
            self.pump_ring();
        } else {
            self.update_enablement(&snap);
        }
        if let Some(out) = self.invalidated.take() {
            return Some(out);
        }
        // `verdict` reads planner-wide constants, so the in-flight
        // command is taken out of `self` for the call and put back.
        let mut fl = self.inflight.take()?;
        let index = fl.server_index;
        let verdict = self.verdict(&mut fl, &snap);
        self.inflight = Some(fl);
        match verdict {
            None => None,
            Some(Ok(v)) => {
                self.inflight = None;
                self.near_singularity = None;
                Some(CommandOutcome {
                    index,
                    error: None,
                    verdict: v,
                })
            }
            Some(Err(e)) => {
                self.discard_planned();
                self.near_singularity = None;
                Some(CommandOutcome {
                    index,
                    error: Some(e),
                    verdict: None,
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
        self.near_singularity = None;
    }

    fn start_tool(
        &mut self,
        index: u64,
        cmd: &par6_proto::command::ToolAction,
    ) -> Result<(), WireError> {
        let snap = self.snapshots.latest();
        let epoch_at_send = self.start_tool_action(&snap, cmd)?;
        self.tool_inflight = Some(ToolInFlight {
            server_index: index,
            epoch_at_send,
        });
        Ok(())
    }

    /// Read the RT's settle verdict for the tool action in flight.
    ///
    /// Deliberately narrow: it touches `tool_inflight` and nothing else.
    /// The motion lane's failure path flushes the sample ring and forces
    /// IDLE, and reaching it from here would stop an arm move because a
    /// gripper faulted.
    fn poll_tool(&mut self) -> Option<CommandOutcome> {
        let fl = self.tool_inflight.as_ref()?;
        let snap = self.snapshots.latest();
        if snap.tool.epoch == fl.epoch_at_send {
            return None; // the RT has not armed it yet
        }
        let index = fl.server_index;
        let (error, verdict) = match snap.tool.verdict {
            ToolSettle::Running => return None,
            ToolSettle::Done => (None, None),
            ToolSettle::Settled(od) => (None, Some(od as u8)),
            ToolSettle::Timeout(w) => (
                Some(make_error(
                    ErrorCode::MotnToolTimeout,
                    UNATTRIBUTED,
                    &[("state", w.as_str())],
                )),
                None,
            ),
            ToolSettle::Fault(bits) => (
                Some(make_error(
                    ErrorCode::MotnToolFault,
                    UNATTRIBUTED,
                    &[("fault_code", &bits.to_string())],
                )),
                None,
            ),
            // Another owner (homing, a flashing window) took the tool
            // and released it on our behalf. Nothing is left to
            // complete, and waiting would hang the client.
            ToolSettle::Unarmed => (
                Some(make_error(
                    ErrorCode::MotnCancelled,
                    UNATTRIBUTED,
                    &[("scope", "the tool changed owner")],
                )),
                None,
            ),
        };
        self.tool_inflight = None;
        Some(CommandOutcome {
            index,
            error,
            verdict,
        })
    }

    fn cancel_tool(&mut self, halt: bool) {
        if self.tool_inflight.take().is_some() && halt {
            // Halt in place rather than release: a stop must never drop
            // whatever the jaws are holding.
            self.link.send(RtCommand::GripperStop);
        }
    }

    fn warnings(&self) -> Vec<WireError> {
        self.near_singularity.iter().cloned().collect()
    }

    fn sync(&mut self, ctx: PlanContext<'_>) {
        if ctx.payload != self.payload {
            // The torque feedforward (and TOPPRA's dynamics, when that
            // profile runs) must carry what the arm carries.
            match self
                .kin
                .set_tool(ctx.payload.mass, ctx.payload.com, ctx.payload.inertia)
            {
                Ok(()) => self.payload = ctx.payload,
                Err(e) => log::error!("planner payload update refused: {e}"),
            }
        }
        if ctx.completion_policy != self.policy {
            self.policy = ctx.completion_policy;
            let rt_policy = match ctx.completion_policy {
                par6_proto::CompletionPolicy::Commanded => par6_rt::CompletionPolicy::Commanded,
                par6_proto::CompletionPolicy::Settled => par6_rt::CompletionPolicy::Settled,
                par6_proto::CompletionPolicy::Strict => par6_rt::CompletionPolicy::Strict,
            };
            let dt = self.dt;
            let motion = self.motion;
            self.link.op(Box::new(move |core| {
                core.set_settle_policy(Box::new(SpecSettle::new(rt_policy, dt, motion)));
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
        // The offset composes AFTER the variant's own TCP frame, which
        // the URDF already carries — so publishing the commanded
        // translation is the whole application. The cell is shared with
        // the bridge's and housekeeping's models and with the RT FK hook,
        // so planning, streaming and the reported pose all resolve at the
        // same point; `TCP_OFFSET` still reads back the COMMANDED value,
        // which the server owns.
        {
            let mm = ctx.tcp_offset_mm;
            self.tool_offset
                .set([mm[0] / 1000.0, mm[1] / 1000.0, mm[2] / 1000.0]);
            // The workspace the enablement probe measured is the old TCP's.
            self.probe.invalidate();
        }
    }

    fn set_shapes(
        &mut self,
        layer: ShapeLayer,
        shapes: &[par6_proto::Shape],
    ) -> Result<Option<u64>, WireError> {
        let refuse = |detail: String| {
            make_error(
                ErrorCode::CommValidationError,
                UNATTRIBUTED,
                &[("detail", &detail)],
            )
        };
        // Convert everything BEFORE touching the world: a set with one
        // bad shape in it is refused whole, so a client can never end up
        // enforcing the half of its keep-outs that happened to parse.
        let converted = shapes
            .iter()
            .map(par6_kin::Shape::from_proto)
            .collect::<Result<Vec<_>, _>>()
            .map_err(|e| refuse(e.to_string()))?;
        if let Some(dup) = first_duplicate(&converted) {
            return Err(refuse(format!("duplicate shape name {dup:?}")));
        }
        // The epoch is the collision world's, not a parallel counter:
        // `set_layer` moves it only for a world it actually applied.
        let epoch = self
            .collision
            .set_layer(kin_layer(layer), &converted)
            .map_err(collision_error)?;
        self.shape_names.set_layer(layer, &converted);
        // The measured freedom was measured against the previous world.
        self.probe.invalidate();
        // Committed motion is not exempt from a world it now violates.
        self.revalidate_inflight();
        Ok(Some(epoch))
    }

    fn collision(&mut self) -> Option<CollisionState> {
        Some(self.collision_latch.clone())
    }

    fn clear_collision(&mut self) {
        self.collision_latch = CollisionState::default();
    }

    fn enablement(&self) -> Enablement {
        self.enablement
    }

    fn queued_duration(&mut self, pending: &[QueuedCommand<'_>]) -> f64 {
        let snap = self.snapshots.latest();
        // Where the queue will start from: the end of the motion in
        // flight when there is one, the measured pose otherwise.
        let mut from = match &self.inflight {
            Some(InFlight {
                kind: InFlightKind::Exec { samples, .. },
                ..
            }) => samples.last().map_or(snap.q, |s| s.q),
            _ => snap.q,
        };
        let mut total = 0.0;
        for queued in pending {
            match queued.cmd {
                Command::Delay(p) => total += p.seconds,
                Command::MoveJ(p) => {
                    let mut target = [0.0; MAX_JOINTS];
                    for (i, t) in target.iter_mut().enumerate() {
                        let a = p.angles[i].to_radians();
                        *t = if p.rel { from[i] + a } else { a };
                    }
                    // The real plan under the selected profile: a move
                    // this planner cannot compile is one it will refuse
                    // when it starts, and an unplannable move has no
                    // honest duration to report.
                    if let Ok(samples) =
                        self.joint_move_samples(&from, &target, p.duration, p.speed, p.accel)
                    {
                        total += samples.len() as f64 * self.dt;
                    }
                    from = target;
                }
                // Cartesian targets are poses: timing one means IK over
                // its whole waypoint chain, which is the planning the
                // move itself will do. Only an explicitly requested
                // duration is known ahead of that.
                Command::MoveJPose(p) => total += p.duration.unwrap_or(0.0),
                Command::MoveL(p) => total += p.duration.unwrap_or(0.0),
                Command::MoveC(p) => total += p.duration.unwrap_or(0.0),
                Command::MoveS(p) => total += p.duration.unwrap_or(0.0),
                Command::MoveP(p) => total += p.duration.unwrap_or(0.0),
                // Homing, gripper actions and the instant commands run
                // against hardware that answers when it answers.
                _ => {}
            }
        }
        total
    }

    fn inflight_duration(&self, snap: &StateSnapshot) -> f64 {
        match &self.inflight {
            Some(InFlight {
                kind:
                    InFlightKind::Exec {
                        ring_index,
                        samples,
                        cursor,
                        ..
                    },
                ..
            }) => {
                // Until the RT is seen PLAYING this trajectory, none of
                // it has been consumed — the snapshot describing the
                // ring predates the samples going in, and reading its
                // `samples_remaining` would report a whole move as no
                // work left.
                let ticks = if snap.exec.active_command_index == *ring_index {
                    // The tail this planner has not fed the ring yet (it
                    // feeds under backpressure, so a long trajectory is
                    // only ever partly in the ring), plus what the ring
                    // still holds.
                    samples.len().saturating_sub(*cursor) as u64 + snap.exec.samples_remaining
                } else {
                    samples.len() as u64
                };
                ticks as f64 * self.dt
            }
            Some(InFlight {
                kind: InFlightKind::Delay { target_tick },
                ..
            }) => target_tick.saturating_sub(snap.tick) as f64 * self.dt,
            _ => 0.0,
        }
    }
}

/// Cap on the pairs spelled out in an error payload; a report can carry
/// up to `par6_kin::MAX_REPORTED_PAIRS` of them and the first few are the
/// actionable ones.
const MAX_REPORTED_PAIRS: usize = 4;

/// Format colliding pairs the way the v2 error catalog's `{pairs}` slot
/// and the golden status fixture spell them: `[a, b], [c, d]`.
fn format_pairs(pairs: &[(String, String)]) -> String {
    let mut out = pairs
        .iter()
        .take(MAX_REPORTED_PAIRS)
        .map(|(a, b)| format!("[{a}, {b}]"))
        .collect::<Vec<_>>()
        .join(", ");
    if pairs.len() > MAX_REPORTED_PAIRS {
        out.push_str(&format!(" (+{} more)", pairs.len() - MAX_REPORTED_PAIRS));
    }
    out
}

/// Max-norm joint distance \[rad\] between two configurations.
fn joint_distance(a: &[f64; MAX_JOINTS], b: &[f64; MAX_JOINTS]) -> f64 {
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

/// A collision-world call the shim refused: a malformed shape (negative
/// radius, zero-length plane normal) or a broken model.
fn collision_error(e: par6_kin::KinError) -> WireError {
    make_error(
        ErrorCode::CommValidationError,
        UNATTRIBUTED,
        &[("detail", &format!("collision world: {e}"))],
    )
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

/// The RT error latch as the failure of the command that was in flight.
/// One mapping serves this path and the standing error the command plane
/// reports, so a fault cannot read one way when it kills a command and
/// another way in STATUS.
fn rt_error(snap: &StateSnapshot) -> WireError {
    par6_server::rt_standing_error(snap).unwrap_or_else(|| {
        make_error(
            ErrorCode::MotnTickFailed,
            UNATTRIBUTED,
            &[("detail", "the RT core latched a hard error")],
        )
    })
}
