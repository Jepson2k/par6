//! External-torque observer: what the motors carry beyond what the arm's
//! own dynamics explain.
//!
//! The estimate is the generalized-momentum observer in its discrete
//! form. With `p = (M(q) + B)·q̇` the arm's momentum (rotor inertia `B`
//! reflected to the joint), the torque nothing in the model accounts for
//! is
//!
//! ```text
//! e = τ_m − τ_f(q̇) − G(q) − C(q, q̇)·q̇ − (M(q) + B)·q̈
//! ```
//!
//! and the observer is a first-order low-pass on it. Integrating the
//! momentum balance instead of differentiating velocity is what the
//! continuous observer is known for; at a fixed tick the two are the same
//! recursion, with `q̈` the tick-to-tick change of the drives' reported
//! velocity and the filter absorbing that difference's noise:
//!
//! ```text
//! r[k] = (r[k−1] + K·dt·e[k]) / (1 + K·dt)
//! ```
//!
//! Backward Euler, so the pole `1/(1 + K·dt)` stays inside the unit
//! circle at any tick rate the runtime is configured for.
//!
//! The sign is "measured minus model": a joint that has to push harder
//! than its dynamics call for reads positive in its direction of effort.
//! Nothing here depends on what was commanded, so the estimate means the
//! same thing in a planned move, a jog, a hold and a floating arm.

use par6_config::RobotConfig;

use crate::gravity::GravityModel;
use crate::MAX_JOINTS;

/// Joint friction as the controller models it: Coulomb plus viscous,
/// joint side.
#[derive(Debug, Clone, Copy)]
pub struct Friction {
    coulomb_nm: [f64; MAX_JOINTS],
    viscous_nm_s: [f64; MAX_JOINTS],
    /// Speed at which the Coulomb term has reached ~76 % of its value
    /// \[rad/s\]. Below it the direction of sliding is not trusted: a
    /// joint at rest sits anywhere inside its stiction band, and a hard
    /// sign there would flip the whole term on encoder noise.
    stick_rad_s: f64,
}

impl Friction {
    /// From `[dynamics]`.
    pub fn from_config(robot: &RobotConfig) -> Self {
        let d = &robot.dynamics;
        Self {
            coulomb_nm: std::array::from_fn(|i| d.friction_coulomb_nm[i]),
            viscous_nm_s: std::array::from_fn(|i| d.friction_viscous_nm_s[i]),
            stick_rad_s: d.friction_stick_rad_s,
        }
    }

    /// Friction torque opposing joint `i` moving at `qd` \[Nm\], signed
    /// with the motion it opposes.
    pub fn torque(&self, i: usize, qd: f64) -> f64 {
        self.coulomb_nm[i] * (qd / self.stick_rad_s).tanh() + self.viscous_nm_s[i] * qd
    }
}

/// The per-tick external-torque estimate.
#[derive(Debug)]
pub struct TorqueObserver {
    dt: f64,
    /// `K·dt`.
    gain: f64,
    friction: Friction,
    rotor_kg_m2: [f64; MAX_JOINTS],
    qd_prev: [f64; MAX_JOINTS],
    qdd: [f64; MAX_JOINTS],
    dynamic: [f64; MAX_JOINTS],
    estimate: [f64; MAX_JOINTS],
    seeded: bool,
    inertial: bool,
}

impl TorqueObserver {
    /// From `[dynamics]`, at the robot's tick period.
    pub fn new(robot: &RobotConfig) -> Self {
        let dt = robot.robot.tick_dt_s;
        let d = &robot.dynamics;
        Self {
            dt,
            gain: std::f64::consts::TAU * d.observer_hz * dt,
            friction: Friction::from_config(robot),
            rotor_kg_m2: std::array::from_fn(|i| d.rotor_inertia_kg_m2[i]),
            qd_prev: [0.0; MAX_JOINTS],
            qdd: [0.0; MAX_JOINTS],
            dynamic: [0.0; MAX_JOINTS],
            estimate: [0.0; MAX_JOINTS],
            seeded: false,
            inertial: false,
        }
    }

    /// One tick: fold this tick's measurement into the estimate.
    ///
    /// `g` is the gravity torque the tick already computed at `q`, with
    /// whatever correction the runtime applies to it. The first call only
    /// takes `qd` as the velocity to difference against.
    pub fn update(
        &mut self,
        model: &mut dyn GravityModel,
        q: &[f64; MAX_JOINTS],
        qd: &[f64; MAX_JOINTS],
        tau: &[f64; MAX_JOINTS],
        g: &[f64; MAX_JOINTS],
    ) {
        if !self.seeded {
            self.qd_prev = *qd;
            self.seeded = true;
            return;
        }
        for (qdd, (now, before)) in self.qdd.iter_mut().zip(qd.iter().zip(&self.qd_prev)) {
            *qdd = (now - before) / self.dt;
        }
        self.qd_prev = *qd;
        self.inertial = model.dynamics(q, qd, &self.qdd, &mut self.dynamic);
        for i in 0..MAX_JOINTS {
            let unexplained = tau[i]
                - self.friction.torque(i, qd[i])
                - g[i]
                - self.dynamic[i]
                - self.rotor_kg_m2[i] * self.qdd[i];
            self.estimate[i] = (self.estimate[i] + self.gain * unexplained) / (1.0 + self.gain);
        }
    }

    /// The external-torque estimate \[Nm\], measured minus model.
    pub fn estimate(&self) -> &[f64; MAX_JOINTS] {
        &self.estimate
    }

    /// Whether the model behind the last update described the arm's
    /// inertia. Without it the estimate carries every acceleration the
    /// arm makes, and nothing may read a contact into it.
    pub fn inertial(&self) -> bool {
        self.inertial
    }

    /// The friction model the estimate subtracts.
    pub fn friction(&self) -> &Friction {
        &self.friction
    }
}
