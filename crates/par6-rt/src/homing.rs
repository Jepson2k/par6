//! Homing sequence orchestrator + per-joint FSM.
//!
//! Runs as mode HOMING (SELF_MANAGED): each tick this module fills the
//! complete per-joint command array and the gripper slot — every
//! non-active joint receives an idle frame (vel 0 / cur 0) and the
//! gripper slot replays the last gripper move or the DLC-0 empty poll,
//! keeping the freshness detector fed on idle nodes.
//!
//! Two levels:
//! - the sequence orchestrator steps the config sequence: `pre_moves`
//!   (idle / nudge / position / gripper_move, run in parallel within the
//!   step), the `home` group (per-joint FSMs in parallel, gripper
//!   firmware calibrate / motor homing), `move_to` cubic-Hermite position
//!   moves, `post_moves`, then the global trailing moves. Pre/post/
//!   move_to timeouts warn and continue; home-phase failures FAIL the
//!   sequence.
//! - the per-joint FSM: approach (stall = windowed displacement plateau
//!   AND current-ratio window, both required; hall = trigger/edge with
//!   the pre-clear guard, on cmd-32 bits dropped at every approach entry
//!   so only a reply this approach asked for can be a hit) → dwell →
//!   backoff → pause → second pass at the rehome speed → optional release
//!   (current-only, latch at the sample percentage) → settle
//!   (position-never-valid = FAILURE, two-pass mismatch = FAILURE, then
//!   the home reference is applied and normal limits restored) →
//!   optional post-move.
//!
//! Current limits: entry swaps every involved node to its homing current
//! (Limits frames ×4 — the runtime keeps the full config reload for exit,
//! via the bus's stored-config resend); each FSM start re-applies it (the
//! only path that also covers the gripper motor); completion restores the
//! normal Ilim ×4; exit resends the full stored node config. The
//! EFFECTIVE per-node limit is published every tick.
//!
//! All plan storage is allocated at construction; `tick` is
//! allocation-free.

use par6_bus::spectral::{trunc_to_wire, JointConversion};
use par6_bus::{
    BusState, DriverBus, FirmwareGripperCommand, GripperCommand, JointCommand, NodeId, NodeState,
};
use par6_config::{
    ConfigBundle, GripperHomeMode, HomingStrategy, JointHoming, PreMove, RobotConfig,
};

use crate::state::{HomingJointStatus, HomingPhase, HomingStatus};
use crate::{MAX_JOINTS, NUM_NODES};

/// Pre/post-move timeout \[s\] (warn and continue).
const PRE_POST_TIMEOUT_S: f64 = 4.0;
/// move_to timeout = duration + this \[s\] (warn and continue).
const MOVE_TO_EXTRA_S: f64 = 2.0;
/// Stopped dwell between the passes \[s\].
const DWELL_S: f64 = 0.08;
/// Pause after backoff before the re-approach \[s\].
const PAUSE_S: f64 = 0.15;
/// Settle hold before the reference is applied \[s\].
const SETTLE_S: f64 = 0.08;
/// Stall/current detection window \[s\].
const DETECT_WINDOW_S: f64 = 0.08;
/// Current-detector startup guard \[s\].
const STARTUP_GUARD_S: f64 = 0.15;
/// Hall pre-clear guard: a trigger this early means "started on the
/// sensor" \[s\].
const PRECLEAR_GUARD_S: f64 = 0.5;
/// Pass-2 speed factor (vendor `rehome_speed_factor` default).
const REHOME_SPEED_FACTOR: f64 = 0.3;
/// Current-ratio threshold on the homing current.
const CURRENT_RATIO: f64 = 0.70;
/// Fraction of the detection window that must be above threshold.
const CURRENT_WINDOW_FRACTION: f64 = 0.6;
/// Stall displacement threshold: `max(10, |speed| · 0.08 · 0.25)` ticks,
/// against the speed COMMANDED this tick (pass 2 runs at
/// `REHOME_SPEED_FACTOR`, so a fixed pass-1 threshold would call a joint
/// still travelling at 83 % of the commanded speed stalled).
const STALL_DISP_FACTOR: f64 = 0.08 * 0.25;
/// In-position tolerance for position moves \[ticks\].
const POS_TOL_TICKS: i64 = 50;
/// Gripper firmware calibrate timeout \[s\].
const CAL_TIMEOUT_S: f64 = 10.0;
/// Gripper firmware calibrate minimum wait \[s\] (the calibrated bit may
/// still be set from a previous run).
const CAL_MIN_WAIT_S: f64 = 2.0;
/// Limits-frame repeats around homing.
const LIMIT_REPEATS: u8 = 4;
/// HALL pack trigger value (vendor homing constant).
const HALL_TRIGGER_VALUE: u8 = 2;
/// Idle pre-move: cmd-12 sends before switching to encoder polls. The
/// driver never replies to cmd 12, so it is sent twice to survive a lost
/// frame (vendor `Send_Idle` cadence).
const IDLE_CMD_REPEATS: u32 = 2;
/// Encoder counts (14-bit) used for the gripper `ticks_per_meter`.
const GRIPPER_ENCODER_COUNTS: f64 = 16384.0;

/// Sequence progress reported to the tick loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeqStatus {
    /// No sequence running.
    Inactive,
    /// Sequence in progress.
    Running,
    /// Sequence finished successfully — the runtime sets `homed`.
    Complete,
    /// Sequence failed — the runtime clears `homed` and returns to IDLE.
    Failed,
}

/// Actuator index space: arm joints `0..MAX_JOINTS`, gripper last.
const GRIPPER_SLOT: usize = MAX_JOINTS;

// ---------------------------------------------------------------- params

#[derive(Debug, Clone)]
struct HomerParams {
    node: NodeId,
    strategy: HomingStrategy,
    /// Signed approach speed \[motor ticks/s\].
    speed: f64,
    current_ma: f64,
    timeout_ticks: u32,
    backoff_ticks: u32,
    two_pass: bool,
    max_diff_ticks: i64,
    release: Option<ReleasePlan>,
    post: Option<PostPlan>,
    stall_needed: u32,
    startup_guard: u32,
    cur_window: u32,
    cur_needed: u32,
    dwell_ticks: u32,
    pause_ticks: u32,
    settle_ticks: u32,
    preclear_ticks: u32,
    in_pos_streak: u32,
    pre_post_timeout: u32,
    normal_vel_limit: f32,
    normal_ilim: f32,
    dt: f64,
}

#[derive(Debug, Clone, Copy)]
struct ReleasePlan {
    current_ma: i16,
    dur_ticks: u32,
    sample_tick: u32,
}

#[derive(Debug, Clone, Copy)]
struct PostPlan {
    position_rad: f64,
    speed: f64,
}

impl HomerParams {
    fn from_config(
        node: NodeId,
        jh: &JointHoming,
        normal_vel_limit: f64,
        normal_ilim: f64,
        dt: f64,
    ) -> Self {
        let ticks = |s: f64| (s / dt).round() as u32;
        let dir_sign = if jh.direction == 1 { -1.0 } else { 1.0 };
        let cur_window = ticks(DETECT_WINDOW_S).max(2);
        Self {
            node,
            strategy: jh.strategy,
            speed: dir_sign * jh.speed_ticks_s,
            current_ma: jh.current_ma,
            timeout_ticks: ticks(jh.timeout_s).max(1),
            backoff_ticks: ticks(jh.backoff_s),
            two_pass: jh.two_pass && jh.strategy == HomingStrategy::Stall,
            max_diff_ticks: i64::from(jh.two_pass_max_diff_ticks),
            release: (jh.strategy == HomingStrategy::Stall)
                .then(|| {
                    jh.release.as_ref().map(|r| {
                        let dur = ticks(r.duration_s).max(1);
                        ReleasePlan {
                            current_ma: r.current_ma as i16,
                            dur_ticks: dur,
                            sample_tick: (((dur as f64) * r.sample_pct).round() as u32)
                                .clamp(1, dur),
                        }
                    })
                })
                .flatten(),
            post: jh.post_home.as_ref().map(|p| PostPlan {
                position_rad: p.position_rad,
                speed: p.speed_ticks_s,
            }),
            stall_needed: ticks(DETECT_WINDOW_S).max(5),
            startup_guard: ticks(STARTUP_GUARD_S),
            cur_window,
            cur_needed: ((f64::from(cur_window) * CURRENT_WINDOW_FRACTION).ceil() as u32).max(1),
            dwell_ticks: ticks(DWELL_S).max(1),
            pause_ticks: ticks(PAUSE_S).max(1),
            settle_ticks: ticks(SETTLE_S).max(1),
            preclear_ticks: ticks(PRECLEAR_GUARD_S),
            in_pos_streak: ticks(DETECT_WINDOW_S).max(1),
            pre_post_timeout: ticks(PRE_POST_TIMEOUT_S).max(1),
            normal_vel_limit: normal_vel_limit as f32,
            normal_ilim: normal_ilim as f32,
            dt,
        }
    }
}

// ---------------------------------------------------------------- per-joint FSM

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum HPhase {
    Approach,
    /// Reversing off a hall band that triggered right at approach
    /// start, until the sensor actually reads clear (bounded by the
    /// joint's timeout budget).
    Preclear,
    Dwell,
    /// Post-dwell backoff before the pass-2 pause.
    Backoff,
    Pause,
    Release,
    Settle,
    PostMove,
    Finished,
    Failed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum HomerEvent {
    /// The reference position was latched at settle; the orchestrator
    /// applies it and restores limits.
    Reference { latched_ticks: i32 },
    /// The FSM failed (timeout / two-pass mismatch / never-valid
    /// position).
    Failed,
}

#[derive(Debug)]
struct Homer {
    phase: HPhase,
    /// The phase the FSM was in when it failed — what the published
    /// `HomingPhase` reports for a `Failed` status, so the operator can
    /// tell an approach timeout from a settle mismatch.
    failed_in: HPhase,
    elapsed: u32,
    pass: u8,
    pass_hits: [i64; 2],
    latched: Option<i32>,
    ref_pos: Option<i32>,
    still: u32,
    cur_ring: Vec<bool>,
    cur_idx: usize,
    cur_filled: u32,
    cur_hits: u32,
    /// The current approach verifiably began OFF the hall band (set by
    /// a completed pre-clear); its trigger is trusted however early.
    started_clear: bool,
    /// Timeout budget already consumed by earlier approach segments and
    /// pre-clears — the per-joint timeout bounds the WHOLE attempt, so
    /// a sensor that never clears exhausts it instead of looping.
    spent: u32,
    post_target_ticks: i32,
    post_streak: u32,
    /// Position at PostMove entry (captured from the first tick with a
    /// valid reading) and the Hermite profile sized from it.
    post_start_ticks: Option<i32>,
    post_dur_ticks: u32,
    post_elapsed: u32,
}

impl Homer {
    fn new(p: &HomerParams) -> Self {
        Self {
            phase: HPhase::Finished,
            failed_in: HPhase::Finished,
            elapsed: 0,
            pass: 1,
            pass_hits: [0; 2],
            latched: None,
            ref_pos: None,
            still: 0,
            cur_ring: vec![false; p.cur_window as usize],
            cur_idx: 0,
            cur_filled: 0,
            cur_hits: 0,
            started_clear: false,
            spent: 0,
            post_target_ticks: 0,
            post_streak: 0,
            post_start_ticks: None,
            post_dur_ticks: 0,
            post_elapsed: 0,
        }
    }

    fn start(&mut self) {
        self.phase = HPhase::Approach;
        self.elapsed = 0;
        self.pass = 1;
        self.pass_hits = [0; 2];
        self.latched = None;
        self.started_clear = false;
        self.spent = 0;
        self.post_streak = 0;
        self.post_start_ticks = None;
        self.post_dur_ticks = 0;
        self.post_elapsed = 0;
        self.reset_detectors();
    }

    fn reset_detectors(&mut self) {
        self.ref_pos = None;
        self.still = 0;
        self.cur_ring.fill(false);
        self.cur_idx = 0;
        self.cur_filled = 0;
        self.cur_hits = 0;
    }

    fn running(&self) -> bool {
        !matches!(self.phase, HPhase::Finished | HPhase::Failed)
    }

    /// The published phase; `Failed` reports the phase the FSM failed in.
    fn public_phase(&self) -> HomingPhase {
        let map = |p: &HPhase| match p {
            HPhase::Approach => HomingPhase::Approach,
            HPhase::Preclear => HomingPhase::Backoff,
            HPhase::Dwell => HomingPhase::Dwell,
            HPhase::Backoff => HomingPhase::Backoff,
            HPhase::Pause => HomingPhase::Pause,
            HPhase::Release => HomingPhase::Release,
            HPhase::Settle => HomingPhase::Settle,
            HPhase::PostMove => HomingPhase::PostMove,
            HPhase::Finished => HomingPhase::Finished,
            HPhase::Failed => HomingPhase::Finished,
        };
        match self.phase {
            HPhase::Failed => map(&self.failed_in),
            ref p => map(p),
        }
    }

    /// `speed` is the signed speed commanded THIS tick — the threshold
    /// tracks it, so pass 2 is judged against pass-2 travel.
    fn detect_stall(&mut self, p: &HomerParams, node: &NodeState, speed: f64) -> bool {
        let disp_ticks = (speed.abs() * STALL_DISP_FACTOR).max(10.0) as i64;
        // Windowed displacement plateau; the window resets on movement.
        let stalled = if let Some(pos) = node.position_ticks {
            match self.ref_pos {
                Some(r) if (i64::from(pos) - i64::from(r)).abs() < disp_ticks => {
                    self.still += 1;
                    self.still >= p.stall_needed
                }
                _ => {
                    self.ref_pos = Some(pos);
                    self.still = 0;
                    false
                }
            }
        } else {
            false
        };
        // Current ratio, gated behind the startup guard.
        let current_hit = if self.elapsed > p.startup_guard {
            let above = node
                .current_ma
                .map(|c| f64::from(c).abs() >= CURRENT_RATIO * p.current_ma)
                .unwrap_or(false);
            let old = self.cur_ring[self.cur_idx];
            self.cur_ring[self.cur_idx] = above;
            self.cur_idx = (self.cur_idx + 1) % self.cur_ring.len();
            if self.cur_filled < p.cur_window {
                self.cur_filled += 1;
            } else if old {
                self.cur_hits -= 1;
            }
            if above {
                self.cur_hits += 1;
            }
            self.cur_filled >= p.cur_window && self.cur_hits >= p.cur_needed
        } else {
            false
        };
        stalled && current_hit
    }

    /// One FSM tick. Returns the wire command for this joint and at most
    /// one event. Takes the node state mutably: entering an approach
    /// drops the node's cached cmd-32 bits (see below).
    fn tick(
        &mut self,
        p: &HomerParams,
        node: &mut NodeState,
    ) -> (JointCommand, Option<HomerEvent>) {
        match self.phase {
            HPhase::Approach => {
                self.elapsed += 1;
                if self.spent.saturating_add(self.elapsed) > p.timeout_ticks {
                    self.failed_in = self.phase;
                    self.phase = HPhase::Failed;
                    return (JointCommand::idle(), Some(HomerEvent::Failed));
                }
                if self.elapsed == 1 {
                    // `hall` is written only by a cmd-32 reply and nothing
                    // else ever invalidates it, so a hit cached from an
                    // earlier approach — the pre-clear trigger, or the
                    // previous home() in this process — would read as a
                    // hit on tick 1 and latch the reference wherever the
                    // joint happens to be. Only a reply solicited by THIS
                    // approach is evidence; `None` stays "no reply yet".
                    node.hall = None;
                }
                let speed = if self.pass == 2 {
                    p.speed * REHOME_SPEED_FACTOR
                } else {
                    p.speed
                };
                let (cmd, hit) = match p.strategy {
                    HomingStrategy::Stall => (
                        JointCommand::velocity(trunc_to_wire(speed), 0),
                        self.detect_stall(p, node, speed),
                    ),
                    HomingStrategy::Hall => (
                        JointCommand::hall(trunc_to_wire(speed), HALL_TRIGGER_VALUE),
                        node.hall.map(|h| !h.trigger || h.edge).unwrap_or(false),
                    ),
                };
                if !hit {
                    return (cmd, None);
                }
                // Hall pre-clear guard: a trigger right at start means
                // the joint began ON the sensor — reverse until the
                // sensor reads clear, then re-approach. An approach that
                // verifiably began off-band trusts its trigger however
                // early it comes.
                if p.strategy == HomingStrategy::Hall
                    && self.elapsed <= p.preclear_ticks
                    && !self.started_clear
                {
                    self.spent = self.spent.saturating_add(self.elapsed);
                    self.elapsed = 0;
                    self.phase = HPhase::Preclear;
                    // Only replies solicited by the pre-clear itself may
                    // prove the band clear.
                    node.hall = None;
                    return (JointCommand::idle(), None);
                }
                let hit_pos = i64::from(node.position_ticks.unwrap_or(0));
                match p.strategy {
                    HomingStrategy::Hall => {
                        // Position latched AT trigger; hall skips two-pass
                        // and release.
                        self.latched = node.position_ticks;
                        self.phase = HPhase::Settle;
                        self.elapsed = 0;
                    }
                    HomingStrategy::Stall => {
                        self.pass_hits[usize::from(self.pass - 1)] = hit_pos;
                        if self.pass == 1 && p.two_pass {
                            self.phase = HPhase::Dwell;
                            self.elapsed = 0;
                        } else if p.release.is_some() {
                            self.phase = HPhase::Release;
                            self.elapsed = 0;
                        } else {
                            self.phase = HPhase::Settle;
                            self.elapsed = 0;
                        }
                    }
                }
                (JointCommand::idle(), None)
            }
            HPhase::Dwell => {
                self.elapsed += 1;
                if self.elapsed >= p.dwell_ticks {
                    self.phase = HPhase::Backoff;
                    self.elapsed = 0;
                }
                (JointCommand::idle(), None)
            }
            HPhase::Preclear => {
                self.elapsed += 1;
                if self.spent.saturating_add(self.elapsed) > p.timeout_ticks {
                    self.failed_in = self.phase;
                    self.phase = HPhase::Failed;
                    return (JointCommand::idle(), Some(HomerEvent::Failed));
                }
                // Reverse WITH the hall pack: each tick's cmd-32 reply
                // carries the live band state, and only an off-band
                // reply with no pending edge proves the next approach
                // starts clear. A sensor that never clears runs the
                // budget out and fails.
                if matches!(node.hall, Some(h) if h.trigger && !h.edge) {
                    self.spent = self.spent.saturating_add(self.elapsed);
                    self.elapsed = 0;
                    self.started_clear = true;
                    self.phase = HPhase::Approach;
                    self.reset_detectors();
                    return (JointCommand::idle(), None);
                }
                (
                    JointCommand::hall(trunc_to_wire(-p.speed), HALL_TRIGGER_VALUE),
                    None,
                )
            }
            HPhase::Backoff => {
                self.elapsed += 1;
                let cmd = JointCommand::velocity(trunc_to_wire(-p.speed), 0);
                if self.elapsed >= p.backoff_ticks {
                    self.elapsed = 0;
                    self.phase = HPhase::Pause;
                }
                (cmd, None)
            }
            HPhase::Pause => {
                self.elapsed += 1;
                if self.elapsed >= p.pause_ticks {
                    self.phase = HPhase::Approach;
                    self.elapsed = 0;
                    self.pass = 2;
                    self.reset_detectors();
                }
                (JointCommand::idle(), None)
            }
            HPhase::Release => {
                self.elapsed += 1;
                let r = p.release.expect("release phase without a plan");
                if self.elapsed == r.sample_tick {
                    if let Some(pos) = node.position_ticks {
                        self.latched = Some(pos);
                    }
                }
                if self.elapsed >= r.dur_ticks {
                    self.phase = HPhase::Settle;
                    self.elapsed = 0;
                }
                (JointCommand::current(r.current_ma), None)
            }
            HPhase::Settle => {
                self.elapsed += 1;
                if self.latched.is_none() {
                    // Retry latching while the position is unknown, up to
                    // 2× the settle time; never-valid = FAILURE (the
                    // vendor silently marked DONE without a reference).
                    self.latched = node.position_ticks;
                }
                if self.elapsed >= p.settle_ticks {
                    match self.latched {
                        None if self.elapsed < 2 * p.settle_ticks => {}
                        None => {
                            self.failed_in = self.phase;
                            self.phase = HPhase::Failed;
                            return (JointCommand::idle(), Some(HomerEvent::Failed));
                        }
                        Some(latched) => {
                            if p.two_pass {
                                let diff = (self.pass_hits[1] - self.pass_hits[0]).abs();
                                if diff > p.max_diff_ticks {
                                    log::warn!(
                                        "homing node {}: two-pass diff {diff} > {}",
                                        p.node,
                                        p.max_diff_ticks
                                    );
                                    self.failed_in = self.phase;
                                    self.phase = HPhase::Failed;
                                    return (JointCommand::idle(), Some(HomerEvent::Failed));
                                }
                            }
                            self.phase = if p.post.is_some() {
                                HPhase::PostMove
                            } else {
                                HPhase::Finished
                            };
                            self.elapsed = 0;
                            return (
                                JointCommand::idle(),
                                Some(HomerEvent::Reference {
                                    latched_ticks: latched,
                                }),
                            );
                        }
                    }
                }
                (JointCommand::idle(), None)
            }
            HPhase::PostMove => {
                self.elapsed += 1;
                let post = p.post.expect("post move without a plan");
                // Drive a Hermite profile to the post-home target instead
                // of a bare (target, speed) frame: the wire speed channel
                // is an additive velocity feedforward, so a standing
                // nonzero speed against a fixed target parks the joint
                // speed/KPP ticks off it (the vendor runtime does exactly
                // that, and its own 50-tick arrival check then times
                // out). The profile's tangent decays to zero at the
                // target, where the position loop closes the landing.
                if self.post_start_ticks.is_none() {
                    self.post_start_ticks = node.position_ticks;
                    if let Some(start) = self.post_start_ticks {
                        let d = f64::from(self.post_target_ticks - start).abs();
                        // Peak Hermite tangent is 1.5·d/span — size the
                        // span so the peak feedforward is the configured
                        // post-home speed.
                        let speed = post.speed.abs().max(1.0);
                        self.post_dur_ticks = ((1.5 * d / speed) / p.dt).ceil().max(1.0) as u32;
                    }
                }
                let cmd = match self.post_start_ticks {
                    Some(start) => {
                        self.post_elapsed += 1;
                        let (pos, vel) = hermite(
                            f64::from(start),
                            f64::from(self.post_target_ticks),
                            self.post_elapsed,
                            self.post_dur_ticks,
                            p.dt,
                        );
                        JointCommand::position(trunc_to_wire(pos), trunc_to_wire(vel), 0)
                    }
                    None => JointCommand::idle(),
                };
                let in_pos = node
                    .position_ticks
                    .map(|pos| {
                        (i64::from(pos) - i64::from(self.post_target_ticks)).abs() <= POS_TOL_TICKS
                    })
                    .unwrap_or(false);
                self.post_streak = if in_pos { self.post_streak + 1 } else { 0 };
                if self.post_streak >= p.in_pos_streak {
                    self.phase = HPhase::Finished;
                } else if self.elapsed > p.pre_post_timeout {
                    log::warn!("homing node {}: post-move timeout (continuing)", p.node);
                    self.phase = HPhase::Finished;
                }
                (cmd, None)
            }
            HPhase::Finished | HPhase::Failed => (JointCommand::idle(), None),
        }
    }
}

// ---------------------------------------------------------------- moves

#[derive(Debug, Clone)]
struct MoveState {
    spec: PreMove,
    dur_ticks: u32,
    elapsed: u32,
    done: bool,
    warned: bool,
    start_ticks: Option<i32>,
}

#[derive(Debug, Clone)]
struct MoveToState {
    joint: usize,
    position_rad: f64,
    dur_ticks: u32,
    timeout_ticks: u32,
    elapsed: u32,
    done: bool,
    warned: bool,
    start_ticks: Option<i32>,
    target_ticks: i32,
    streak: u32,
}

/// Cubic Hermite (zero end velocities) between two tick positions:
/// returns (position, signed profile velocity ticks/s) at `elapsed` of
/// `dur` ticks. The velocity slot of a position frame is the driver's
/// additive velocity FEEDFORWARD, so the returned value is
/// the profile's true tangent: zero at both ends, where the position
/// loop alone closes the landing residual.
fn hermite(start: f64, end: f64, elapsed: u32, dur: u32, dt: f64) -> (f64, f64) {
    let s = (f64::from(elapsed) / f64::from(dur.max(1))).clamp(0.0, 1.0);
    let d = end - start;
    let pos = start + d * (3.0 * s * s - 2.0 * s * s * s);
    let span_s = f64::from(dur.max(1)) * dt;
    let vel = d * (6.0 * s - 6.0 * s * s) / span_s;
    (pos, vel)
}

// ---------------------------------------------------------------- orchestrator

#[derive(Debug, Clone)]
struct StepPlan {
    pre: Vec<MoveState>,
    /// Joints homed in parallel by this step (`Copy` mask, not a `Vec` —
    /// the Home part reads it every tick and must not allocate).
    home_joints: [bool; MAX_JOINTS],
    home_gripper: Option<GripperHomeMode>,
    move_to: Vec<MoveToState>,
    post: Vec<MoveState>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Part {
    Pre,
    HomeStart,
    Home,
    MoveTo,
    Post,
    GlobalPost,
}

#[derive(Debug, Clone, Copy)]
struct CalRun {
    sent: bool,
    elapsed: u32,
}

/// The homing subsystem: owns the parsed sequence and all FSM state.
pub struct HomingSystem {
    dt: f64,
    params: [HomerParams; MAX_JOINTS],
    gripper_params: Option<HomerParams>,
    homers: Vec<Homer>,
    steps: Vec<StepPlan>,
    global_post: Vec<MoveState>,
    eff_offset: [f64; MAX_JOINTS],
    node_of: [NodeId; MAX_JOINTS],
    gripper_node: NodeId,
    has_can_gripper: bool,
    gripper_gear_r_m: f64,
    cal_timeout: u32,
    cal_min_wait: u32,

    active: bool,
    step_idx: usize,
    part: Part,
    statuses: [HomingJointStatus; NUM_NODES],
    last_fw_cmd: Option<FirmwareGripperCommand>,
    cal: Option<CalRun>,
    cal_failed: bool,
    endstop_ticks: Option<i32>,
    ticks_per_meter: Option<f64>,
}

impl HomingSystem {
    /// Build from the config bundle (allocates all plan storage here).
    pub fn new(bundle: &ConfigBundle) -> Self {
        let robot: &RobotConfig = &bundle.robot;
        let dt = robot.robot.tick_dt_s;
        let ticks = |s: f64| (s / dt).round() as u32;
        let params: [HomerParams; MAX_JOINTS] = std::array::from_fn(|i| {
            HomerParams::from_config(
                robot.joints[i].node_id,
                &robot.homing.joints[i],
                robot.joints[i].velocity_limit_ticks_s,
                robot.joints[i].ilim_ma,
                dt,
            )
        });
        let active_gripper = bundle.active_gripper();
        let gripper_driver = active_gripper.and_then(|g| g.driver.as_ref());
        let gripper_params = active_gripper.and_then(|g| {
            let (d, h) = (g.driver.as_ref()?, g.homing.as_ref()?);
            Some(HomerParams::from_config(
                robot.bus.gripper_node,
                h,
                d.velocity_limit_ticks_s,
                d.ilim_ma,
                dt,
            ))
        });
        let mut homers: Vec<Homer> = params.iter().map(Homer::new).collect();
        homers.push(
            gripper_params
                .as_ref()
                .map(Homer::new)
                .unwrap_or_else(|| Homer::new(&params[0])),
        );
        let plan_move = |m: &PreMove| MoveState {
            spec: *m,
            dur_ticks: ticks(match m {
                PreMove::Idle { duration_s, .. }
                | PreMove::Nudge { duration_s, .. }
                | PreMove::Position { duration_s, .. }
                | PreMove::GripperMove { duration_s, .. } => *duration_s,
            })
            .max(1),
            elapsed: 0,
            done: false,
            warned: false,
            start_ticks: None,
        };
        let steps = robot
            .homing
            .sequence
            .iter()
            .map(|s| StepPlan {
                pre: s.pre_moves.iter().map(plan_move).collect(),
                home_joints: {
                    let mut mask = [false; MAX_JOINTS];
                    if let Some(h) = s.home.as_ref() {
                        for j in &h.joints {
                            mask[usize::from(*j)] = true;
                        }
                    }
                    mask
                },
                home_gripper: s.home.as_ref().and_then(|h| h.gripper),
                move_to: s
                    .move_to
                    .iter()
                    .map(|m| MoveToState {
                        joint: usize::from(m.joint),
                        position_rad: m.position_rad,
                        dur_ticks: ticks(m.duration_s).max(1),
                        timeout_ticks: ticks(m.duration_s + MOVE_TO_EXTRA_S),
                        elapsed: 0,
                        done: false,
                        warned: false,
                        start_ticks: None,
                        target_ticks: 0,
                        streak: 0,
                    })
                    .collect(),
                post: s.post_moves.iter().map(plan_move).collect(),
            })
            .collect();
        let global_post = robot.homing.post_moves.iter().map(plan_move).collect();
        let eff_offset = std::array::from_fn(|i| {
            bundle
                .effective_home_offset(i)
                .unwrap_or(robot.homing.joints[i].home_offset_rad)
        });
        Self {
            dt,
            params,
            gripper_params,
            homers,
            steps,
            global_post,
            eff_offset,
            node_of: std::array::from_fn(|i| robot.joints[i].node_id),
            gripper_node: robot.bus.gripper_node,
            has_can_gripper: gripper_driver.is_some(),
            gripper_gear_r_m: gripper_driver.map(|d| d.gear_r_m).unwrap_or(0.0),
            cal_timeout: ticks(CAL_TIMEOUT_S).max(1),
            cal_min_wait: ticks(CAL_MIN_WAIT_S),
            active: false,
            step_idx: 0,
            part: Part::Pre,
            statuses: [HomingJointStatus::Idle; NUM_NODES],
            last_fw_cmd: None,
            cal: None,
            cal_failed: false,
            endstop_ticks: None,
            ticks_per_meter: None,
        }
    }

    /// Whether a sequence is currently running.
    pub fn active(&self) -> bool {
        self.active
    }

    /// Whether the last sequence failed on the gripper firmware
    /// calibration specifically.
    pub fn calibration_failed(&self) -> bool {
        self.cal_failed
    }

    /// Drop the latched calibration failure (the user clear sequence).
    /// Without this the flag outlives the clear, `check_errors` re-latches
    /// `GRIPPER_CALIBRATION_FAILED` on the next tick, and the hard-error
    /// gate refuses the HOMING entry that is the only other way to clear
    /// it — a restart-only lockout on a transient.
    pub fn clear_faults(&mut self) {
        self.cal_failed = false;
    }

    /// Gripper motor-homing results: `(endstop_ticks, ticks_per_meter)`.
    pub fn gripper_reference(&self) -> (Option<i32>, Option<f64>) {
        (self.endstop_ticks, self.ticks_per_meter)
    }

    /// Start the sequence: reset all state, swap every involved node to
    /// its homing current (Limits ×4).
    pub fn start<B: DriverBus>(&mut self, bus: &mut B) {
        self.active = true;
        self.step_idx = 0;
        self.part = Part::Pre;
        self.statuses = [HomingJointStatus::Idle; NUM_NODES];
        self.cal = None;
        self.cal_failed = false;
        self.last_fw_cmd = None;
        for h in &mut self.homers {
            h.phase = HPhase::Finished;
        }
        self.reset_step_states(0);
        for p in &self.params {
            let _ = bus.send_limits(
                p.node,
                p.normal_vel_limit,
                p.current_ma as f32,
                LIMIT_REPEATS,
            );
        }
        if let Some(gp) = &self.gripper_params {
            let _ = bus.send_limits(
                gp.node,
                gp.normal_vel_limit,
                gp.current_ma as f32,
                LIMIT_REPEATS,
            );
        }
    }

    /// Abort (hard error or mode exit): zero statuses, restore every
    /// node's full stored config. The caller un-homes the robot.
    pub fn abort<B: DriverBus>(&mut self, bus: &mut B) {
        if !self.active {
            return;
        }
        self.active = false;
        self.statuses = [HomingJointStatus::Idle; NUM_NODES];
        self.cal = None;
        self.restore_all(bus);
    }

    fn restore_all<B: DriverBus>(&self, bus: &mut B) {
        for p in &self.params {
            let _ = bus.resend_node_config(p.node, 1);
        }
        if self.has_can_gripper {
            let _ = bus.resend_node_config(self.gripper_node, 1);
        }
    }

    fn reset_step_states(&mut self, idx: usize) {
        if let Some(step) = self.steps.get_mut(idx) {
            for m in step.pre.iter_mut().chain(step.post.iter_mut()) {
                m.elapsed = 0;
                m.done = false;
                m.warned = false;
                m.start_ticks = None;
            }
            for m in &mut step.move_to {
                m.elapsed = 0;
                m.done = false;
                m.warned = false;
                m.start_ticks = None;
                m.streak = 0;
            }
        }
    }

    /// Published status (also valid while inactive).
    pub fn status(&self) -> HomingStatus {
        let mut eff = [0.0f32; NUM_NODES];
        for (i, e) in eff.iter_mut().enumerate().take(MAX_JOINTS) {
            let homing_now = self.active && self.statuses[i] != HomingJointStatus::Done;
            *e = if homing_now {
                self.params[i].current_ma as f32
            } else {
                self.params[i].normal_ilim
            };
        }
        if let Some(gp) = &self.gripper_params {
            let homing_now = self.active && self.statuses[GRIPPER_SLOT] != HomingJointStatus::Done;
            eff[GRIPPER_SLOT] = if homing_now {
                gp.current_ma as f32
            } else {
                gp.normal_ilim
            };
        }
        // Phase is meaningful only once that actuator's FSM has been
        // started (a firmware-calibrated gripper never drives a Homer and
        // stays at Idle).
        let mut phase = [HomingPhase::Idle; NUM_NODES];
        for (i, out) in phase.iter_mut().enumerate() {
            if self.statuses[i] != HomingJointStatus::Idle {
                if let Some(h) = self.homers.get(i) {
                    *out = h.public_phase();
                }
            }
        }
        HomingStatus {
            active: self.active,
            sequence_step: self.step_idx.min(u8::MAX as usize) as u8,
            per_joint: self.statuses,
            phase,
            effective_current_limit_ma: eff,
        }
    }

    /// Per-actuator statuses (arm joints then gripper).
    /// The last firmware gripper frame this sequence put on the bus
    /// (`None` when the sequence never commanded the gripper). The
    /// hand-back to the normal path announces idle from these bytes, so
    /// the release carries the same speed/current the hold did.
    pub fn last_fw_cmd(&self) -> Option<FirmwareGripperCommand> {
        self.last_fw_cmd
    }

    pub fn statuses(&self) -> &[HomingJointStatus; NUM_NODES] {
        &self.statuses
    }

    fn fail<B: DriverBus>(&mut self, bus: &mut B) -> SeqStatus {
        self.active = false;
        self.restore_all(bus);
        SeqStatus::Failed
    }

    /// Advance one move (pre/post kind); fills the actuator command.
    /// Returns whether the move is complete.
    #[allow(clippy::too_many_arguments)]
    fn tick_move(
        m: &mut MoveState,
        dt: f64,
        pre_post_timeout: u32,
        nodes: &[NodeState],
        node_of: &[NodeId; MAX_JOINTS],
        conv: &[JointConversion; MAX_JOINTS],
        cmds: &mut [JointCommand; MAX_JOINTS],
        gcmd: &mut GripperCommand,
        last_fw: &mut Option<FirmwareGripperCommand>,
    ) -> bool {
        if m.done {
            return true;
        }
        m.elapsed += 1;
        match m.spec {
            PreMove::Idle { joint, .. } => {
                // Firmware idle, not the vel-0 keep-alive: the joint hangs
                // limp for the duration so its drivetrain unloads while a
                // neighbour homes. Encoder polls keep feedback and the
                // freshness detector alive without re-arming the loop.
                cmds[usize::from(joint)] = if m.elapsed <= IDLE_CMD_REPEATS {
                    JointCommand::drop_to_idle()
                } else {
                    JointCommand::encoder_poll()
                };
                if m.elapsed >= m.dur_ticks {
                    m.done = true;
                }
            }
            PreMove::Nudge {
                joint,
                speed_ticks_s,
                ..
            } => {
                let j = usize::from(joint);
                if m.elapsed <= m.dur_ticks {
                    cmds[j] = JointCommand::velocity(trunc_to_wire(speed_ticks_s), 0);
                }
                if m.elapsed >= m.dur_ticks {
                    m.done = true;
                }
            }
            PreMove::Position {
                joint,
                position_rad,
                ..
            } => {
                let j = usize::from(joint);
                let node = &nodes[usize::from(node_of[j])];
                if m.start_ticks.is_none() {
                    m.start_ticks = node.position_ticks;
                }
                let target = conv[j].motor_ticks(position_rad);
                if let Some(start) = m.start_ticks {
                    let (pos, vel) = hermite(
                        f64::from(start),
                        f64::from(target),
                        m.elapsed,
                        m.dur_ticks,
                        dt,
                    );
                    cmds[j] = JointCommand::position(trunc_to_wire(pos), trunc_to_wire(vel), 0);
                    let in_pos = node
                        .position_ticks
                        .map(|p| (i64::from(p) - i64::from(target)).abs() <= POS_TOL_TICKS)
                        .unwrap_or(false);
                    if m.elapsed >= m.dur_ticks && in_pos {
                        m.done = true;
                    }
                }
                if !m.done && m.elapsed > pre_post_timeout {
                    if !m.warned {
                        log::warn!("homing position pre/post-move timed out (continuing)");
                        m.warned = true;
                    }
                    m.done = true;
                }
            }
            PreMove::GripperMove {
                position,
                speed,
                current_ma,
                activate,
                action,
                estop,
                release_dir,
                ..
            } => {
                let cmd = FirmwareGripperCommand {
                    position,
                    speed,
                    current_ma,
                    activate,
                    action,
                    estop,
                    release_dir,
                };
                *gcmd = GripperCommand::Firmware(cmd);
                *last_fw = Some(cmd);
                if m.elapsed >= m.dur_ticks {
                    m.done = true;
                }
            }
        }
        m.done
    }

    /// One HOMING tick: fill the full per-joint command array (idle
    /// frames on every non-active joint) and the gripper slot; drive the
    /// sequence. Needs `&mut conv` to apply home references, and `&mut
    /// state` to invalidate cached endstop-detector replies (cmd 32) that
    /// a new approach must not read as its own.
    #[allow(clippy::too_many_arguments)]
    pub fn tick<B: DriverBus>(
        &mut self,
        bus: &mut B,
        state: &mut BusState,
        conv: &mut [JointConversion; MAX_JOINTS],
        cmds: &mut [JointCommand; MAX_JOINTS],
        gcmd: &mut GripperCommand,
    ) -> SeqStatus {
        // Bus liveness defaults: idle frames everywhere, gripper replay.
        cmds.fill(JointCommand::idle());
        *gcmd = if self.has_can_gripper {
            match self.last_fw_cmd {
                Some(cmd) => GripperCommand::Firmware(cmd),
                None => GripperCommand::FirmwarePoll,
            }
        } else {
            GripperCommand::NoGripper
        };
        if !self.active {
            return SeqStatus::Inactive;
        }
        if self.step_idx >= self.steps.len() {
            return self.tick_global_post(state, conv, cmds, gcmd);
        }

        match self.part {
            Part::Pre => {
                let mut all_done = true;
                let mut step = std::mem::take(&mut self.steps[self.step_idx].pre);
                for m in &mut step {
                    let done = Self::tick_move(
                        m,
                        self.dt,
                        self.params[0].pre_post_timeout,
                        &state.nodes,
                        &self.node_of,
                        conv,
                        cmds,
                        gcmd,
                        &mut self.last_fw_cmd,
                    );
                    all_done &= done;
                }
                self.steps[self.step_idx].pre = step;
                if all_done {
                    self.part = Part::HomeStart;
                }
                SeqStatus::Running
            }
            Part::HomeStart => {
                // FSM start: homing current limits ×4 (the only path that
                // also applies to the gripper motor), reset FSMs.
                let joints = self.steps[self.step_idx].home_joints;
                for j in (0..MAX_JOINTS).filter(|&j| joints[j]) {
                    let p = &self.params[j];
                    let _ = bus.send_limits(
                        p.node,
                        p.normal_vel_limit,
                        p.current_ma as f32,
                        LIMIT_REPEATS,
                    );
                    self.homers[j].start();
                    self.statuses[j] = HomingJointStatus::Running;
                }
                match self.steps[self.step_idx].home_gripper {
                    Some(GripperHomeMode::Motor) => {
                        if let Some(gp) = &self.gripper_params {
                            let _ = bus.send_limits(
                                gp.node,
                                gp.normal_vel_limit,
                                gp.current_ma as f32,
                                LIMIT_REPEATS,
                            );
                            self.homers[GRIPPER_SLOT].start();
                            self.statuses[GRIPPER_SLOT] = HomingJointStatus::Running;
                        }
                    }
                    Some(GripperHomeMode::Firmware) => {
                        self.cal = Some(CalRun {
                            sent: false,
                            elapsed: 0,
                        });
                        self.statuses[GRIPPER_SLOT] = HomingJointStatus::Running;
                    }
                    None => {}
                }
                self.part = Part::Home;
                SeqStatus::Running
            }
            Part::Home => self.tick_home(bus, state, conv, cmds, gcmd),
            Part::MoveTo => {
                let mut all_done = true;
                let mut moves = std::mem::take(&mut self.steps[self.step_idx].move_to);
                for m in &mut moves {
                    all_done &= self.tick_move_to(m, state, conv, cmds);
                }
                self.steps[self.step_idx].move_to = moves;
                if all_done {
                    self.part = Part::Post;
                }
                SeqStatus::Running
            }
            Part::Post => {
                let mut all_done = true;
                let mut post = std::mem::take(&mut self.steps[self.step_idx].post);
                for m in &mut post {
                    let done = Self::tick_move(
                        m,
                        self.dt,
                        self.params[0].pre_post_timeout,
                        &state.nodes,
                        &self.node_of,
                        conv,
                        cmds,
                        gcmd,
                        &mut self.last_fw_cmd,
                    );
                    all_done &= done;
                }
                self.steps[self.step_idx].post = post;
                if all_done {
                    self.step_idx += 1;
                    if self.step_idx < self.steps.len() {
                        self.reset_step_states(self.step_idx);
                        self.part = Part::Pre;
                    } else {
                        self.part = Part::GlobalPost;
                        for m in &mut self.global_post {
                            m.elapsed = 0;
                            m.done = false;
                            m.warned = false;
                            m.start_ticks = None;
                        }
                    }
                }
                SeqStatus::Running
            }
            Part::GlobalPost => self.tick_global_post(state, conv, cmds, gcmd),
        }
    }

    fn tick_global_post(
        &mut self,
        state: &BusState,
        conv: &[JointConversion; MAX_JOINTS],
        cmds: &mut [JointCommand; MAX_JOINTS],
        gcmd: &mut GripperCommand,
    ) -> SeqStatus {
        let mut all_done = true;
        let mut post = std::mem::take(&mut self.global_post);
        for m in &mut post {
            let done = Self::tick_move(
                m,
                self.dt,
                self.params[0].pre_post_timeout,
                &state.nodes,
                &self.node_of,
                conv,
                cmds,
                gcmd,
                &mut self.last_fw_cmd,
            );
            all_done &= done;
        }
        self.global_post = post;
        if all_done {
            self.active = false;
            return SeqStatus::Complete;
        }
        SeqStatus::Running
    }

    fn tick_move_to(
        &mut self,
        m: &mut MoveToState,
        state: &BusState,
        conv: &[JointConversion; MAX_JOINTS],
        cmds: &mut [JointCommand; MAX_JOINTS],
    ) -> bool {
        if m.done {
            return true;
        }
        m.elapsed += 1;
        let j = m.joint;
        let node = &state.nodes[usize::from(self.node_of[j])];
        if m.start_ticks.is_none() {
            m.start_ticks = node.position_ticks;
            m.target_ticks = conv[j].motor_ticks(m.position_rad);
        }
        if let Some(start) = m.start_ticks {
            let (pos, vel) = hermite(
                f64::from(start),
                f64::from(m.target_ticks),
                m.elapsed,
                m.dur_ticks,
                self.dt,
            );
            cmds[j] = JointCommand::position(trunc_to_wire(pos), trunc_to_wire(vel), 0);
            let in_pos = node
                .position_ticks
                .map(|p| (i64::from(p) - i64::from(m.target_ticks)).abs() <= POS_TOL_TICKS)
                .unwrap_or(false);
            m.streak = if in_pos { m.streak + 1 } else { 0 };
            if m.elapsed >= m.dur_ticks && m.streak >= self.params[j].in_pos_streak {
                m.done = true;
            }
        }
        if !m.done && m.elapsed > m.timeout_ticks {
            if !m.warned {
                log::warn!("homing move_to joint {j} timed out (continuing)");
                m.warned = true;
            }
            m.done = true;
        }
        m.done
    }

    fn tick_home<B: DriverBus>(
        &mut self,
        bus: &mut B,
        state: &mut BusState,
        conv: &mut [JointConversion; MAX_JOINTS],
        cmds: &mut [JointCommand; MAX_JOINTS],
        gcmd: &mut GripperCommand,
    ) -> SeqStatus {
        let mut failed = false;
        let joints = self.steps[self.step_idx].home_joints;
        for j in (0..MAX_JOINTS).filter(|&j| joints[j]) {
            if !self.homers[j].running() {
                continue;
            }
            let p = &self.params[j];
            let node = &mut state.nodes[usize::from(p.node)];
            let (cmd, event) = self.homers[j].tick(p, node);
            cmds[j] = cmd;
            match event {
                Some(HomerEvent::Reference { latched_ticks }) => {
                    conv[j].set_home(latched_ticks, self.eff_offset[j]);
                    let _ =
                        bus.send_limits(p.node, p.normal_vel_limit, p.normal_ilim, LIMIT_REPEATS);
                    self.statuses[j] = HomingJointStatus::Done;
                    if let Some(post) = &p.post {
                        self.homers[j].post_target_ticks = conv[j].motor_ticks(post.position_rad);
                    }
                }
                Some(HomerEvent::Failed) => {
                    self.statuses[j] = HomingJointStatus::Failed;
                    failed = true;
                }
                None => {}
            }
        }
        // Gripper homing, if this step includes it.
        match self.steps[self.step_idx].home_gripper {
            Some(GripperHomeMode::Motor) => {
                if self.homers[GRIPPER_SLOT].running() {
                    if let Some(gp) = &self.gripper_params {
                        let node = &mut state.nodes[usize::from(gp.node)];
                        let (cmd, event) = self.homers[GRIPPER_SLOT].tick(gp, node);
                        *gcmd = GripperCommand::Motor(cmd);
                        match event {
                            Some(HomerEvent::Reference { latched_ticks }) => {
                                self.endstop_ticks = Some(latched_ticks);
                                self.ticks_per_meter = Some(
                                    GRIPPER_ENCODER_COUNTS
                                        / (4.0 * std::f64::consts::PI * self.gripper_gear_r_m),
                                );
                                let _ = bus.send_limits(
                                    gp.node,
                                    gp.normal_vel_limit,
                                    gp.normal_ilim,
                                    LIMIT_REPEATS,
                                );
                                self.statuses[GRIPPER_SLOT] = HomingJointStatus::Done;
                            }
                            Some(HomerEvent::Failed) => {
                                self.statuses[GRIPPER_SLOT] = HomingJointStatus::Failed;
                                failed = true;
                            }
                            None => {}
                        }
                    }
                }
            }
            Some(GripperHomeMode::Firmware) => {
                if let Some(cal) = &mut self.cal {
                    cal.elapsed += 1;
                    if !cal.sent {
                        *gcmd = GripperCommand::Calibrate;
                        cal.sent = true;
                    } else {
                        *gcmd = GripperCommand::FirmwarePoll;
                        let calibrated = state.gripper.reply.map(|r| r.calibrated).unwrap_or(false);
                        if cal.elapsed >= self.cal_min_wait && calibrated {
                            self.statuses[GRIPPER_SLOT] = HomingJointStatus::Done;
                            self.cal = None;
                        } else if cal.elapsed >= self.cal_timeout {
                            self.statuses[GRIPPER_SLOT] = HomingJointStatus::Failed;
                            self.cal_failed = true;
                            self.cal = None;
                            failed = true;
                        }
                    }
                }
            }
            None => {}
        }
        if failed {
            return self.fail(bus);
        }
        // Part complete when every FSM in the group is finished.
        let arm_done = (0..MAX_JOINTS)
            .filter(|&j| joints[j])
            .all(|j| !self.homers[j].running());
        let gripper_done = match self.steps[self.step_idx].home_gripper {
            Some(GripperHomeMode::Motor) => !self.homers[GRIPPER_SLOT].running(),
            Some(GripperHomeMode::Firmware) => self.cal.is_none(),
            None => true,
        };
        if arm_done && gripper_done {
            self.part = Part::MoveTo;
        }
        SeqStatus::Running
    }
}
