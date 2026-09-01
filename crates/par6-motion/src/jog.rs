//! Manual jog engine.
//!
//! Per joint, per tick: target velocity from `(axis, direction, speed
//! pct)`; jerk-aware lookahead against the soft limits with
//! direction-block latching; trapezoid or s-curve velocity ramp; hard
//! clamp past soft limits; target position integration.
//!
//! The lookahead stopping distance follows the vendor formulas — trapezoid
//! `v²/2a`, s-curve `v²/2a + v·a/2j` — extended with the current
//! acceleration state: a trip can fire mid-ramp where the joint is still
//! accelerating outward, and a jerk-limited stop must first reverse that
//! acceleration (velocity keeps rising by `a₀²/2j` meanwhile). Without
//! the reversal terms the stop overshoots the soft limit; with `a₀ = 0`
//! the extension reduces exactly to the vendor formula. The ×1.5 safety
//! factor is applied on top, per the vendor firmware.

use par6_config::{JogProfile, LimitMode, RobotConfig};

use crate::{MotionError, MotionLimits, NUM_JOINTS};

/// Runtime floor for the jog ramp time \[s\].
pub const MIN_ACCEL_TIME_S: f64 = 0.05;
/// Runtime floor for the s-curve jerk factor.
pub const MIN_JERK_FACTOR: f64 = 0.5;
/// Safety factor on the lookahead stopping distance.
const STOP_MARGIN: f64 = 1.5;

/// Direction of a jog command along a joint axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JogDirection {
    /// Toward the positive soft limit.
    Positive,
    /// Toward the negative soft limit.
    Negative,
}

impl JogDirection {
    fn from_sign(v: f64) -> Self {
        if v >= 0.0 {
            Self::Positive
        } else {
            Self::Negative
        }
    }
}

/// One tick of jog output: the integrated target state for all joints.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JogTick {
    /// Integrated target positions \[rad\].
    pub q: [f64; NUM_JOINTS],
    /// Ramped target velocities \[rad/s\].
    pub qd: [f64; NUM_JOINTS],
}

/// Jog velocity-ramp engine with soft-limit lookahead.
///
/// Lifecycle: [`activate`] on JOG mode entry (syncs the integrator to the
/// measured pose and clears latches), then [`command`]/[`release`] from
/// the command plane and [`tick`] once per RT tick.
///
/// [`activate`]: JogEngine::activate
/// [`command`]: JogEngine::command
/// [`release`]: JogEngine::release
/// [`tick`]: JogEngine::tick
#[derive(Debug, Clone)]
pub struct JogEngine {
    dt: f64,
    limits: MotionLimits,
    profile: JogProfile,
    accel_time_s: f64,
    jerk_factor: f64,
    /// Active command: per-joint signed speed fraction, 0 = not driven.
    active: [f64; NUM_JOINTS],
    /// The joint SET of the last command; survives release so a change of
    /// set (the block-clearing event) is detected across button releases.
    last_set: [bool; NUM_JOINTS],
    v: [f64; NUM_JOINTS],
    acc: [f64; NUM_JOINTS],
    q: [f64; NUM_JOINTS],
    blocked: [Option<JogDirection>; NUM_JOINTS],
    activated: bool,
}

impl JogEngine {
    /// Build from a validated robot config: JOG-mode limits (fallback to
    /// ceiling per `JointLimits::for_mode`) and the `[jog]` defaults with
    /// the runtime floors applied.
    pub fn new(cfg: &RobotConfig) -> Result<Self, MotionError> {
        let limits = MotionLimits::from_config(cfg, LimitMode::Jog)?;
        Ok(Self {
            dt: cfg.robot.tick_dt_s,
            limits,
            profile: cfg.jog.profile,
            accel_time_s: cfg.jog.accel_time_s.max(MIN_ACCEL_TIME_S),
            jerk_factor: cfg.jog.jerk_factor.max(MIN_JERK_FACTOR),
            active: [0.0; NUM_JOINTS],
            last_set: [false; NUM_JOINTS],
            v: [0.0; NUM_JOINTS],
            acc: [0.0; NUM_JOINTS],
            q: [0.0; NUM_JOINTS],
            blocked: [None; NUM_JOINTS],
            activated: false,
        })
    }

    /// Sync to the measured pose on JOG mode entry: zeros the ramp state,
    /// clears direction blocks and any active command.
    pub fn activate(&mut self, q_meas: &[f64; NUM_JOINTS]) {
        self.q = *q_meas;
        self.v = [0.0; NUM_JOINTS];
        self.acc = [0.0; NUM_JOINTS];
        self.blocked = [None; NUM_JOINTS];
        self.active = [0.0; NUM_JOINTS];
        self.last_set = [false; NUM_JOINTS];
        self.activated = true;
    }

    /// Start (or retarget) a jog: `speeds[j]` is joint `j`'s signed
    /// fraction of its jog velocity limit, and 0 leaves that joint still.
    /// Joints move together, each on its own ramp.
    ///
    /// A latch belongs to its joint: leaving the driven set clears a
    /// joint's own block, and so does commanding it the opposite way —
    /// the only two ways a latched block clears. A joint still driven
    /// the same way keeps its latch whatever the rest of the set does
    /// (wiping every block on any set change would let a limit-parked
    /// joint drive further out the moment a second axis joined the jog).
    pub fn command(&mut self, speeds: &[f64; NUM_JOINTS]) -> Result<(), MotionError> {
        for (j, v) in speeds.iter().enumerate() {
            if !(v.is_finite() && (-1.0..=1.0).contains(v)) {
                return Err(MotionError::InvalidInput {
                    what: "speeds",
                    reason: format!("speeds[{j}] must be finite and in [-1, 1], got {v}"),
                });
            }
        }
        let set = speeds.map(|v| v != 0.0);
        for (j, v) in speeds.iter().enumerate() {
            let leaving = self.last_set[j] && !set[j];
            let reversed =
                *v != 0.0 && self.blocked[j].is_some_and(|b| b != JogDirection::from_sign(*v));
            if leaving || reversed {
                self.blocked[j] = None;
            }
        }
        self.active = *speeds;
        self.last_set = set;
        Ok(())
    }

    /// Release the jog button: target velocity drops to zero everywhere.
    /// Direction blocks survive release.
    pub fn release(&mut self) {
        self.active = [0.0; NUM_JOINTS];
    }

    /// Latched direction block for `joint`, if any (telemetry).
    pub fn blocked_direction(&self, joint: usize) -> Option<JogDirection> {
        self.blocked.get(joint).copied().flatten()
    }

    /// Current ramped velocity of `joint` \[rad/s\] (telemetry).
    pub fn velocity(&self, joint: usize) -> f64 {
        self.v[joint]
    }

    /// Select the ramp shape.
    pub fn set_profile(&mut self, profile: JogProfile) {
        self.profile = profile;
    }

    /// Set the 0→full-speed ramp time \[s\]; the runtime floor
    /// [`MIN_ACCEL_TIME_S`] applies.
    pub fn set_accel_time_s(&mut self, accel_time_s: f64) -> Result<(), MotionError> {
        if !(accel_time_s.is_finite() && accel_time_s > 0.0) {
            return Err(MotionError::InvalidInput {
                what: "accel_time_s",
                reason: format!("must be finite and > 0, got {accel_time_s}"),
            });
        }
        self.accel_time_s = accel_time_s.max(MIN_ACCEL_TIME_S);
        Ok(())
    }

    /// Set the s-curve jerk factor (`jerk = accel × factor`); the runtime
    /// floor [`MIN_JERK_FACTOR`] applies.
    pub fn set_jerk_factor(&mut self, jerk_factor: f64) -> Result<(), MotionError> {
        if !(jerk_factor.is_finite() && jerk_factor > 0.0) {
            return Err(MotionError::InvalidInput {
                what: "jerk_factor",
                reason: format!("must be finite and > 0, got {jerk_factor}"),
            });
        }
        self.jerk_factor = jerk_factor.max(MIN_JERK_FACTOR);
        Ok(())
    }

    /// Advance one tick: lookahead + block latching, velocity ramp, hard
    /// clamp, position integration. `q_meas` is the measured joint state
    /// used by the hard clamp.
    pub fn tick(&mut self, q_meas: &[f64; NUM_JOINTS]) -> JogTick {
        if !self.activated {
            self.activate(q_meas);
        }
        for (j, &qm) in q_meas.iter().enumerate() {
            let v_full = self.limits.velocity[j];
            let a = (v_full / self.accel_time_s).min(self.limits.acceleration[j]);
            let jerk = a * self.jerk_factor;

            let mut v_t = self.active[j] * v_full;

            // Jerk-aware lookahead on the direction of motion (or of the
            // command, when starting from rest).
            let probe = if self.v[j] != 0.0 { self.v[j] } else { v_t };
            if probe != 0.0 {
                let sgn = probe.signum();
                // Remaining travel is measured, not integrated: the arm
                // is what approaches the limit, and a stalled joint
                // whose integrator ran ahead must not latch its whole
                // direction while the plant is still far away.
                let remaining = if sgn > 0.0 {
                    self.limits.soft_max[j] - qm
                } else {
                    qm - self.limits.soft_min[j]
                };
                let speed = self.v[j].abs();
                let stop = match self.profile {
                    JogProfile::Trapezoid => speed * speed / (2.0 * a),
                    JogProfile::Scurve => {
                        let a0 = (self.acc[j] * sgn).max(0.0);
                        let v_peak = speed + a0 * a0 / (2.0 * jerk);
                        speed * a0 / jerk
                            + a0 * a0 * a0 / (3.0 * jerk * jerk)
                            + v_peak * v_peak / (2.0 * a)
                            + v_peak * a / (2.0 * jerk)
                    }
                };
                if STOP_MARGIN * stop >= remaining {
                    self.blocked[j] = Some(JogDirection::from_sign(sgn));
                }
            }
            if v_t != 0.0 && self.blocked[j] == Some(JogDirection::from_sign(v_t)) {
                v_t = 0.0;
            }

            // Velocity ramp.
            match self.profile {
                JogProfile::Trapezoid => {
                    let dv = (v_t - self.v[j]).clamp(-a * self.dt, a * self.dt);
                    self.acc[j] = dv / self.dt;
                    self.v[j] += dv;
                }
                JogProfile::Scurve => {
                    let v_err = v_t - self.v[j];
                    if v_err.abs() <= jerk * self.dt * self.dt
                        && self.acc[j].abs() <= 1.5 * jerk * self.dt
                    {
                        self.v[j] = v_t;
                        self.acc[j] = 0.0;
                    } else {
                        // Back off (ramp accel to zero) once the remaining
                        // velocity error equals what the ramp-down itself
                        // will deliver: |v_err| ≤ a₀²/2j.
                        let backoff = self.acc[j] * v_err > 0.0
                            && v_err.abs() <= self.acc[j] * self.acc[j] / (2.0 * jerk);
                        let a_des = if backoff { 0.0 } else { v_err.signum() * a };
                        let da = (a_des - self.acc[j]).clamp(-jerk * self.dt, jerk * self.dt);
                        self.acc[j] = (self.acc[j] + da).clamp(-a, a);
                        self.v[j] += self.acc[j] * self.dt;
                    }
                }
            }

            // Integrate; never step the target across a soft limit.
            let q_prev = self.q[j];
            self.q[j] += self.v[j] * self.dt;
            if self.v[j] > 0.0
                && self.q[j] > self.limits.soft_max[j]
                && q_prev <= self.limits.soft_max[j]
            {
                self.q[j] = self.limits.soft_max[j];
                self.v[j] = 0.0;
                self.acc[j] = 0.0;
            } else if self.v[j] < 0.0
                && self.q[j] < self.limits.soft_min[j]
                && q_prev >= self.limits.soft_min[j]
            {
                self.q[j] = self.limits.soft_min[j];
                self.v[j] = 0.0;
                self.acc[j] = 0.0;
            }

            // Hard clamp: measured position past a soft limit while moving
            // outward.
            if qm > self.limits.soft_max[j] && self.v[j] > 0.0 {
                self.v[j] = 0.0;
                self.acc[j] = 0.0;
                self.q[j] = self.q[j].min(self.limits.soft_max[j]);
            } else if qm < self.limits.soft_min[j] && self.v[j] < 0.0 {
                self.v[j] = 0.0;
                self.acc[j] = 0.0;
                self.q[j] = self.q[j].max(self.limits.soft_min[j]);
            }
        }
        JogTick {
            q: self.q,
            qd: self.v,
        }
    }
}
