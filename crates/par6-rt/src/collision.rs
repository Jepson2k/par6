//! Collision detection.
//!
//! A generalized-momentum observer estimates, per joint, the torque the
//! model does not explain: what the drives deliver less gravity, inertia
//! (rotors included), Coriolis and the arm's measured friction. The monitor
//! compares how far that estimate has moved since the arm last stood still
//! with each joint's contact and collision thresholds, while a jog, a
//! stream or a program moves the arm.
//!
//! With `p = M(q)·q̇` and `β = g(q) − C(q,q̇)ᵀ·q̇` the observer is
//! `r = K·(p − p₀ − ∫(τ_m − τ_f − β + r) dt)`, so `ṙ = K·(τ_ext − r)`: a
//! first-order lag of the external torque that needs no acceleration
//! measurement. This is the vendor's observer, its gain and its gating.

use std::f64::consts::PI;

use par6_config::{CollisionMode, RobotConfig};

use crate::MAX_JOINTS;

/// Observer gain \[1/s\]: a 20 ms lag, `K·dt` = 0.2 at 250 Hz, well inside
/// explicit Euler's bound of 2.
const OBSERVER_GAIN_PER_S: f64 = 50.0;
/// A tick period past this resets the observer \[s\]: its integral would
/// carry a step it never saw.
const MAX_GAP_S: f64 = 0.05;
/// No verdicts this long after the observer starts \[s\]: it starts from
/// zero, and the estimate needs a few time constants to reach the load.
const OBSERVER_SETTLE_S: f64 = 0.1;
/// Friction's sign is smoothed below this speed \[rad/s\], so the
/// Coulomb term does not flip on encoder noise at rest.
const FRICTION_SMOOTHING_RAD_S: f64 = 0.02;
/// A commanded joint speed above this is motion \[rad/s\] (0.5 deg/s).
const MOVING_RAD_S: f64 = 0.5 * PI / 180.0;
/// Checking runs on this long after the commanded motion stops \[s\]: a
/// move that ends against an obstacle is still pressing on it.
const SETTLE_S: f64 = 0.5;
/// At rest the baseline follows the estimate with this time constant \[s\],
/// absorbing model error and stiction at the pose the arm stands at.
const BASELINE_TAU_S: f64 = 0.1;
/// How long a joint stays over a threshold before it counts \[s\].
const SUSTAIN_S: f64 = 0.012;
/// A joint is clear again below this fraction of its contact threshold,
/// held for [`CLEAR_HOLD_S`].
const REARM_FRACTION: f64 = 0.5;
/// How long a joint stays under the rearm level before its warnings clear
/// \[s\]: an estimate ringing about its threshold would otherwise raise
/// and clear them every few ticks, and every edge is a log write.
const CLEAR_HOLD_S: f64 = 0.25;

/// The external torque the model does not explain.
#[derive(Debug, Clone)]
pub struct MomentumObserver {
    coulomb_nm: [f64; MAX_JOINTS],
    viscous_nm_s: [f64; MAX_JOINTS],
    /// Rotor inertia reflected to each joint, `G²·J_m` \[kg·m²\]: the
    /// drive accelerates it, and the link model does not carry it.
    armature: [f64; MAX_JOINTS],
    p0: [f64; MAX_JOINTS],
    integral: [f64; MAX_JOINTS],
    r: [f64; MAX_JOINTS],
    running: bool,
    ticks: u32,
    settle_ticks: u32,
}

impl MomentumObserver {
    /// An observer for `robot`'s joints, with the friction and rotor
    /// inertia its `[sim]` section measures.
    pub fn new(robot: &RobotConfig) -> Self {
        let per_joint = |values: &[f64], scale: &dyn Fn(usize) -> f64| {
            std::array::from_fn(|j| values.get(j).map_or(0.0, |v| v * scale(j)))
        };
        let gear = |j: usize| robot.joints.get(j).map_or(0.0, |c| c.gear_ratio);
        Self {
            coulomb_nm: per_joint(&robot.sim.coulomb_nm, &|_| 1.0),
            viscous_nm_s: per_joint(&robot.sim.viscous_nm_s, &|_| 1.0),
            armature: per_joint(&robot.sim.motor_jm_kg_m2, &|j| gear(j) * gear(j)),
            p0: [0.0; MAX_JOINTS],
            integral: [0.0; MAX_JOINTS],
            r: [0.0; MAX_JOINTS],
            running: false,
            ticks: 0,
            settle_ticks: robot.ticks(OBSERVER_SETTLE_S).max(1),
        }
    }

    /// Start over on the next step: the model changed under the integral
    /// (a payload), or the estimate has no meaning (the arm unreferenced).
    pub fn reset(&mut self) {
        self.running = false;
    }

    /// One tick. `p` and `ctv` are the model's `M(q)·q̇` and
    /// `C(q,q̇)ᵀ·q̇` at this tick's `q` and `qd`, `g` its gravity, `tau_m`
    /// the torque the drives report delivering, `dt` the tick's period.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        p: &[f64; MAX_JOINTS],
        ctv: &[f64; MAX_JOINTS],
        g: &[f64; MAX_JOINTS],
        tau_m: &[f64; MAX_JOINTS],
        qd: &[f64; MAX_JOINTS],
        dt: f64,
    ) {
        let finite = [p, ctv, g, tau_m, qd]
            .iter()
            .all(|a| a.iter().all(|v| v.is_finite()));
        if !finite || !(dt > 0.0 && dt <= MAX_GAP_S) {
            self.running = false;
            return;
        }
        let momentum: [f64; MAX_JOINTS] = std::array::from_fn(|j| p[j] + self.armature[j] * qd[j]);
        if !self.running {
            self.p0 = momentum;
            self.integral = [0.0; MAX_JOINTS];
            self.r = [0.0; MAX_JOINTS];
            self.running = true;
            self.ticks = 0;
            return;
        }
        for j in 0..MAX_JOINTS {
            let friction = self.coulomb_nm[j] * (qd[j] / FRICTION_SMOOTHING_RAD_S).tanh()
                + self.viscous_nm_s[j] * qd[j];
            let beta = g[j] - ctv[j];
            self.integral[j] += (tau_m[j] - friction - beta + self.r[j]) * dt;
            self.r[j] = OBSERVER_GAIN_PER_S * (momentum[j] - self.p0[j] - self.integral[j]);
        }
        self.ticks = self.ticks.saturating_add(1);
    }

    /// The estimate in par6's external-torque sense, measured minus model
    /// (what a hand holding the arm up reads as: negative where it bears
    /// load). `false`, and `out` untouched, while the observer is off.
    pub fn estimate(&self, out: &mut [f64; MAX_JOINTS]) -> bool {
        if !self.running {
            return false;
        }
        for (o, r) in out.iter_mut().zip(self.r) {
            *o = -r;
        }
        true
    }

    /// Whether the estimate has had long enough to reach the load.
    pub fn settled(&self) -> bool {
        self.running && self.ticks >= self.settle_ticks
    }
}

/// Per-joint contact and collision verdicts on the observer's estimate.
#[derive(Debug, Clone)]
pub struct CollisionMonitor {
    mode: CollisionMode,
    contact_nm: [f64; MAX_JOINTS],
    collision_nm: [f64; MAX_JOINTS],
    settle_ticks: u32,
    sustain_ticks: u32,
    baseline_alpha: f64,
    baseline: [f64; MAX_JOINTS],
    baseline_taken: bool,
    settle_left: u32,
    over_contact: [u32; MAX_JOINTS],
    over_collision: [u32; MAX_JOINTS],
    clear_hold_ticks: u32,
    under: [u32; MAX_JOINTS],
    contact: [bool; MAX_JOINTS],
    collision: [bool; MAX_JOINTS],
}

impl CollisionMonitor {
    /// The monitor `robot`'s `[collision]` section describes.
    pub fn new(robot: &RobotConfig) -> Self {
        let c = &robot.collision;
        let per_joint = |v: &[f64]| std::array::from_fn(|j| v.get(j).copied().unwrap_or(0.0));
        let dt = robot.robot.tick_dt_s;
        Self {
            mode: c.mode,
            contact_nm: per_joint(&c.contact_nm),
            collision_nm: per_joint(&c.collision_nm),
            settle_ticks: robot.ticks(SETTLE_S).max(1),
            sustain_ticks: robot.ticks(SUSTAIN_S).max(1),
            baseline_alpha: 1.0 - (-dt / BASELINE_TAU_S).exp(),
            baseline: [0.0; MAX_JOINTS],
            baseline_taken: false,
            settle_left: 0,
            over_contact: [0; MAX_JOINTS],
            over_collision: [0; MAX_JOINTS],
            clear_hold_ticks: robot.ticks(CLEAR_HOLD_S).max(1),
            under: [0; MAX_JOINTS],
            contact: [false; MAX_JOINTS],
            collision: [false; MAX_JOINTS],
        }
    }

    /// The configured reaction to a collision.
    pub fn mode(&self) -> CollisionMode {
        self.mode
    }

    /// One tick. `active`: the arm is referenced, enabled, in a motion
    /// mode and the estimate has settled. `moving`: some joint is
    /// commanded faster than [`MOVING_RAD_S`].
    pub fn step(&mut self, active: bool, moving: bool, estimate: &[f64; MAX_JOINTS]) {
        if !active || self.mode == CollisionMode::Off {
            self.baseline_taken = false;
            self.settle_left = 0;
            self.clear();
            return;
        }
        if !self.baseline_taken {
            self.baseline = *estimate;
            self.baseline_taken = true;
        }
        if moving {
            self.settle_left = self.settle_ticks;
        } else {
            self.settle_left = self.settle_left.saturating_sub(1);
        }
        if !moving && self.settle_left == 0 {
            for (b, e) in self.baseline.iter_mut().zip(estimate) {
                *b += self.baseline_alpha * (e - *b);
            }
            self.clear();
            return;
        }
        let sustain = self.sustain_ticks;
        for (j, e) in estimate.iter().enumerate() {
            let d = (e - self.baseline[j]).abs();
            let over = |threshold: f64, count: &mut u32| {
                *count = if threshold > 0.0 && d > threshold {
                    count.saturating_add(1)
                } else {
                    0
                };
                *count >= sustain
            };
            let contact = over(self.contact_nm[j], &mut self.over_contact[j]);
            let collision = over(self.collision_nm[j], &mut self.over_collision[j]);
            self.contact[j] |= contact || collision;
            self.collision[j] |= collision;
            let clear_below = REARM_FRACTION
                * if self.contact_nm[j] > 0.0 {
                    self.contact_nm[j]
                } else {
                    self.collision_nm[j]
                };
            self.under[j] = if d < clear_below {
                self.under[j].saturating_add(1)
            } else {
                0
            };
            if self.under[j] >= self.clear_hold_ticks {
                self.contact[j] = false;
                self.collision[j] = false;
            }
        }
    }

    /// Whether joint `j` is in contact: over its contact threshold, and not
    /// yet back under half of it for [`CLEAR_HOLD_S`].
    pub fn contact(&self, j: usize) -> bool {
        self.contact[j]
    }

    /// Whether joint `j` collided during the contact it is in.
    pub fn collision(&self, j: usize) -> bool {
        self.collision[j]
    }

    /// `true` when some joint's commanded speed is motion.
    pub fn moving(qd_target: &[f64; MAX_JOINTS]) -> bool {
        qd_target.iter().any(|v| v.abs() > MOVING_RAD_S)
    }

    fn clear(&mut self) {
        self.over_contact = [0; MAX_JOINTS];
        self.over_collision = [0; MAX_JOINTS];
        self.under = [0; MAX_JOINTS];
        self.contact = [false; MAX_JOINTS];
        self.collision = [false; MAX_JOINTS];
    }
}
