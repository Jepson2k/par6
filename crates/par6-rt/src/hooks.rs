//! Integration seams between the RT core and the motion stack.
//!
//! The tick loop consumes jog ramps, streaming target tracking, and EXEC
//! completion policies through these SMALL per-tick traits instead of
//! depending on `par6-motion` directly — `par6d` adapts the real engines
//! (`par6_motion::JogEngine`, `StreamingExecutor`, `CompletionMonitor`)
//! onto them at wiring time. The trait shapes mirror those engines'
//! lifecycles (activate / command / tick) so the adapters are thin.
//!
//! Built-in implementations ship alongside for tests and the simulated
//! runtime: [`RampJog`] (linear ramp + soft-limit clamp with direction
//! blocks), [`ClampStream`] (soft-limit clamp passthrough), and
//! [`SpecSettle`] — the FULL spec completion-policy state machine
//! (commanded / settled / strict with the blend-continues bypass), which
//! is not a stub but the reference implementation of spec/RT.md's
//! completion table.
//!
//! All per-tick methods must be allocation-free.

use par6_config::RobotConfig;

use crate::MAX_JOINTS;

// ---------------------------------------------------------------- jog

/// Per-tick jog ramp engine (spec/RT.md "Jog").
pub trait JogEngine: Send {
    /// JOG-mode entry: sync the integrator to the measured pose and clear
    /// direction blocks.
    fn activate(&mut self, q_meas: &[f64; MAX_JOINTS]);
    /// Command a jog: joint index and signed speed fraction
    /// (`dir · pct`, in \[-1, 1\]). Replaces any previous command.
    fn command(&mut self, joint: usize, signed_pct: f64);
    /// Release the jog button: ramp every joint down to zero.
    fn release(&mut self);
    /// One tick: write integrated target positions and ramped velocities.
    /// Returns the direction-block latch mask (bit `2i` = negative
    /// direction of joint `i` blocked, bit `2i+1` = positive).
    fn tick(
        &mut self,
        q_meas: &[f64; MAX_JOINTS],
        q_out: &mut [f64; MAX_JOINTS],
        qd_out: &mut [f64; MAX_JOINTS],
    ) -> u16;
}

/// Built-in jog engine: trapezoid velocity ramp (`Δv ≤ a·dt`), target
/// integration, hard clamp + direction-block latch at the soft limits.
/// The jerk-aware lookahead lives in `par6-motion`; this one is the
/// simple reference used by tests and the sim runtime.
pub struct RampJog {
    dt: f64,
    vmax: [f64; MAX_JOINTS],
    accel: [f64; MAX_JOINTS],
    soft_min: [f64; MAX_JOINTS],
    soft_max: [f64; MAX_JOINTS],
    target_q: [f64; MAX_JOINTS],
    vel: [f64; MAX_JOINTS],
    request: Option<(usize, f64)>,
    blocked: u16,
}

impl RampJog {
    /// Build from the robot's jog-mode limits.
    pub fn new(cfg: &RobotConfig) -> Self {
        let mut vmax = [0.0; MAX_JOINTS];
        let mut accel = [0.0; MAX_JOINTS];
        let mut soft_min = [0.0; MAX_JOINTS];
        let mut soft_max = [0.0; MAX_JOINTS];
        for (i, j) in cfg.joints.iter().enumerate().take(MAX_JOINTS) {
            let l = j.limits.for_mode(par6_config::LimitMode::Jog);
            vmax[i] = l.velocity_rad_s;
            accel[i] = l.acceleration_rad_s2;
            soft_min[i] = j.limits.soft_min_rad;
            soft_max[i] = j.limits.soft_max_rad;
        }
        Self {
            dt: cfg.robot.tick_dt_s,
            vmax,
            accel,
            soft_min,
            soft_max,
            target_q: [0.0; MAX_JOINTS],
            vel: [0.0; MAX_JOINTS],
            request: None,
            blocked: 0,
        }
    }
}

impl JogEngine for RampJog {
    fn activate(&mut self, q_meas: &[f64; MAX_JOINTS]) {
        self.target_q = *q_meas;
        self.vel = [0.0; MAX_JOINTS];
        self.request = None;
        self.blocked = 0;
    }

    fn command(&mut self, joint: usize, signed_pct: f64) {
        if joint < MAX_JOINTS && signed_pct.is_finite() {
            if let Some((prev, _)) = self.request {
                if prev != joint {
                    // Joint switch clears the previous joint's blocks.
                    self.blocked &= !(0b11 << (2 * prev));
                }
            }
            self.request = Some((joint, signed_pct.clamp(-1.0, 1.0)));
        }
    }

    fn release(&mut self) {
        self.request = None;
    }

    fn tick(
        &mut self,
        q_meas: &[f64; MAX_JOINTS],
        q_out: &mut [f64; MAX_JOINTS],
        qd_out: &mut [f64; MAX_JOINTS],
    ) -> u16 {
        for i in 0..MAX_JOINTS {
            let mut want = 0.0;
            if let Some((j, pct)) = self.request {
                if j == i {
                    want = pct * self.vmax[i];
                    // A latched block in the commanded direction zeroes the
                    // request; the opposite direction clears the latch.
                    let neg_bit = 1u16 << (2 * i);
                    let pos_bit = 2u16 << (2 * i);
                    if want > 0.0 {
                        if self.blocked & pos_bit != 0 {
                            want = 0.0;
                        } else {
                            self.blocked &= !neg_bit;
                        }
                    } else if want < 0.0 {
                        if self.blocked & neg_bit != 0 {
                            want = 0.0;
                        } else {
                            self.blocked &= !pos_bit;
                        }
                    }
                }
            }
            let dv = (want - self.vel[i]).clamp(-self.accel[i] * self.dt, self.accel[i] * self.dt);
            self.vel[i] += dv;
            self.target_q[i] += self.vel[i] * self.dt;
            // Hard clamp at the soft limits; latch the outward direction.
            if self.target_q[i] >= self.soft_max[i] && self.vel[i] > 0.0 {
                self.target_q[i] = self.soft_max[i];
                self.vel[i] = 0.0;
                self.blocked |= 2u16 << (2 * i);
            } else if self.target_q[i] <= self.soft_min[i] && self.vel[i] < 0.0 {
                self.target_q[i] = self.soft_min[i];
                self.vel[i] = 0.0;
                self.blocked |= 1u16 << (2 * i);
            }
            // Measured position past a soft limit moving outward: clamp.
            if q_meas[i] > self.soft_max[i] && self.vel[i] > 0.0 {
                self.vel[i] = 0.0;
                self.blocked |= 2u16 << (2 * i);
            } else if q_meas[i] < self.soft_min[i] && self.vel[i] < 0.0 {
                self.vel[i] = 0.0;
                self.blocked |= 1u16 << (2 * i);
            }
        }
        *q_out = self.target_q;
        *qd_out = self.vel;
        self.blocked
    }
}

// ---------------------------------------------------------------- stream

/// Per-tick streaming target tracker (spec/RT.md "Streaming"): newest
/// external target in, limited setpoint out. The full OTG limiter lives
/// in `par6-motion` (`StreamingExecutor`).
pub trait StreamTracker: Send {
    /// Stream-mode entry: hold at the measured pose until a target arrives.
    fn activate(&mut self, q_meas: &[f64; MAX_JOINTS]);
    /// Newest raw target for this tick (only the newest is applied —
    /// superseded targets are discarded upstream).
    fn set_target(&mut self, q_target: &[f64; MAX_JOINTS]);
    /// One tick: write the post-limiter position/velocity setpoint.
    fn step(&mut self, q_out: &mut [f64; MAX_JOINTS], qd_out: &mut [f64; MAX_JOINTS]);
}

/// Built-in tracker: unconditional soft-limit clamp, no rate limiting
/// (the clamp is the invariant that must survive even with limiting off).
pub struct ClampStream {
    dt: f64,
    soft_min: [f64; MAX_JOINTS],
    soft_max: [f64; MAX_JOINTS],
    current: [f64; MAX_JOINTS],
    target: [f64; MAX_JOINTS],
}

impl ClampStream {
    /// Build from the robot's soft limits.
    pub fn new(cfg: &RobotConfig) -> Self {
        let mut soft_min = [0.0; MAX_JOINTS];
        let mut soft_max = [0.0; MAX_JOINTS];
        for (i, j) in cfg.joints.iter().enumerate().take(MAX_JOINTS) {
            soft_min[i] = j.limits.soft_min_rad;
            soft_max[i] = j.limits.soft_max_rad;
        }
        Self {
            dt: cfg.robot.tick_dt_s,
            soft_min,
            soft_max,
            current: [0.0; MAX_JOINTS],
            target: [0.0; MAX_JOINTS],
        }
    }
}

impl StreamTracker for ClampStream {
    fn activate(&mut self, q_meas: &[f64; MAX_JOINTS]) {
        self.current = *q_meas;
        self.target = *q_meas;
    }

    fn set_target(&mut self, q_target: &[f64; MAX_JOINTS]) {
        self.target = *q_target;
    }

    fn step(&mut self, q_out: &mut [f64; MAX_JOINTS], qd_out: &mut [f64; MAX_JOINTS]) {
        for i in 0..MAX_JOINTS {
            let clamped = self.target[i].clamp(self.soft_min[i], self.soft_max[i]);
            qd_out[i] = (clamped - self.current[i]) / self.dt;
            self.current[i] = clamped;
            q_out[i] = clamped;
        }
    }
}

// ---------------------------------------------------------------- completion

/// Verdict of one settling tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SettleVerdict {
    /// Still settling — EXEC keeps holding at the boundary target.
    Settling,
    /// The command is complete; playback resumes.
    Complete,
    /// The settle failed (strict policy timeout) — the tick loop latches
    /// a hard error.
    Fault,
}

/// EXEC completion policy (spec/RT.md "EXEC", completion policies).
pub trait SettlePolicy: Send {
    /// Arm at a segment boundary, passing the boundary sample's
    /// `blend_continues`. Returns `true` when the boundary completes
    /// immediately (commanded policy, or any policy with blend-continues
    /// set — blended corners must stay velocity-continuous).
    fn arm(&mut self, blend_continues: bool) -> bool;
    /// One settling tick with measured and boundary-target positions.
    fn tick(
        &mut self,
        q_meas: &[f64; MAX_JOINTS],
        q_target: &[f64; MAX_JOINTS],
    ) -> SettleVerdict;
}

/// Which completion policy [`SpecSettle`] runs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CompletionPolicy {
    /// Complete at the last sample of the command.
    Commanded,
    /// Hold until every joint tracks within tolerance, or the timeout
    /// elapses (then complete anyway). The spec default.
    #[default]
    Settled,
    /// Like `Settled`, but the timeout is an ERROR.
    Strict,
}

/// All-joint settle tolerance \[rad\] (spec default).
pub const SETTLE_TOLERANCE_RAD: f64 = 0.01;
/// Settle timeout \[s\] (spec: 500 ticks at 4 ms).
pub const SETTLE_TIMEOUT_S: f64 = 2.0;

/// The spec completion-policy state machine — the reference
/// implementation, not a test stub.
#[derive(Debug, Clone, Copy)]
pub struct SpecSettle {
    policy: CompletionPolicy,
    tolerance_rad: f64,
    timeout_ticks: u32,
    elapsed: u32,
}

impl SpecSettle {
    /// Policy runner at tick period `dt` \[s\], spec tolerances.
    pub fn new(policy: CompletionPolicy, dt: f64) -> Self {
        Self {
            policy,
            tolerance_rad: SETTLE_TOLERANCE_RAD,
            timeout_ticks: ((SETTLE_TIMEOUT_S / dt).round() as u32).max(1),
            elapsed: 0,
        }
    }

    /// The policy this runner enforces.
    pub fn policy(&self) -> CompletionPolicy {
        self.policy
    }
}

impl SettlePolicy for SpecSettle {
    fn arm(&mut self, blend_continues: bool) -> bool {
        self.elapsed = 0;
        blend_continues || self.policy == CompletionPolicy::Commanded
    }

    fn tick(
        &mut self,
        q_meas: &[f64; MAX_JOINTS],
        q_target: &[f64; MAX_JOINTS],
    ) -> SettleVerdict {
        let max_err = q_meas
            .iter()
            .zip(q_target)
            .map(|(m, t)| (m - t).abs())
            .fold(0.0f64, f64::max);
        if max_err <= self.tolerance_rad {
            return SettleVerdict::Complete;
        }
        self.elapsed += 1;
        if self.elapsed >= self.timeout_ticks {
            return match self.policy {
                CompletionPolicy::Strict => SettleVerdict::Fault,
                _ => SettleVerdict::Complete,
            };
        }
        SettleVerdict::Settling
    }
}

// ---------------------------------------------------------------- flashing

/// Flash-marker hook consulted ONCE on FLASHING exit: firmware was
/// actually written during the maintenance window, so homing must be
/// invalidated robot-wide. `par6d` wires this to the flasher's marker
/// file; tests use [`SharedFlashMarker`].
pub trait FlashMarker: Send {
    /// Whether a flash happened since FLASHING was entered.
    fn flashed(&mut self) -> bool;
}

/// Shared-flag marker for tests and the sim runtime.
#[derive(Debug, Clone)]
pub struct SharedFlashMarker {
    flag: std::sync::Arc<std::sync::atomic::AtomicBool>,
}

impl SharedFlashMarker {
    /// A marker plus the handle used to set it.
    pub fn new() -> (Self, std::sync::Arc<std::sync::atomic::AtomicBool>) {
        let flag = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        (Self { flag: flag.clone() }, flag)
    }
}

impl FlashMarker for SharedFlashMarker {
    fn flashed(&mut self) -> bool {
        self.flag.load(std::sync::atomic::Ordering::Relaxed)
    }
}
