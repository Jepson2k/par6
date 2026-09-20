//! Streaming (servo) executor: the jerk-limited per-tick OTG limiter for
//! online position targets.
//!
//! rsruckig tracks the newest target under the stream-mode limits;
//! retargeting mid-motion re-plans from the current kinematic state, so
//! moving targets stay smooth. Soft-limit clamping of targets and outputs
//! is the RT streaming pipeline's job (it clamps unconditionally, before
//! and after this limiter) — this type owns only the OTG step.

use par6_config::RobotConfig;
use rsruckig::prelude::*;

use crate::cart::{self, Pose, IDENTITY_POSE};
use crate::{MotionError, MotionLimits, NUM_JOINTS};

/// One tick of streaming output.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StreamStep {
    /// Commanded joint positions \[rad\].
    pub q: [f64; NUM_JOINTS],
    /// Commanded joint velocities \[rad/s\].
    pub qd: [f64; NUM_JOINTS],
    /// True once the current target is reached (position and derivatives).
    pub finished: bool,
}

/// Online jerk-limited target tracker for servo-style streaming.
///
/// Lifecycle: [`activate`] on session claim (syncs to the measured pose,
/// at rest), [`set_target`] whenever a new setpoint arrives, [`step`]
/// once per RT tick.
///
/// [`activate`]: StreamingExecutor::activate
/// [`set_target`]: StreamingExecutor::set_target
/// [`step`]: StreamingExecutor::step
pub struct StreamingExecutor {
    otg: Ruckig<NUM_JOINTS, ThrowErrorHandler>,
    input: InputParameter<NUM_JOINTS>,
    output: OutputParameter<NUM_JOINTS>,
    active: bool,
}

impl StreamingExecutor {
    /// Build for tick period `dt` \[s\] under `limits` (normally the
    /// stream block). Requires finite jerk limits — the OTG is
    /// jerk-limited by design.
    pub fn new(dt: f64, limits: &MotionLimits) -> Result<Self, MotionError> {
        if !(dt.is_finite() && dt > 0.0 && dt < 1.0) {
            return Err(MotionError::InvalidInput {
                what: "dt",
                reason: format!("must be a finite tick period in (0, 1) s, got {dt}"),
            });
        }
        let mut exec = Self {
            otg: Ruckig::<NUM_JOINTS, ThrowErrorHandler>::new(None, dt),
            input: InputParameter::<NUM_JOINTS>::new(None),
            output: OutputParameter::<NUM_JOINTS>::new(None),
            active: false,
        };
        exec.set_limits(limits)?;
        Ok(exec)
    }

    /// Apply (new) kinodynamic limits; takes effect from the next step.
    pub fn set_limits(&mut self, limits: &MotionLimits) -> Result<(), MotionError> {
        limits.require_finite_jerk()?;
        for j in 0..NUM_JOINTS {
            self.input.max_velocity[j] = limits.velocity[j];
            self.input.max_acceleration[j] = limits.acceleration[j];
            self.input.max_jerk[j] = limits.jerk[j];
        }
        Ok(())
    }

    /// Sync to the measured pose on session activation: current state and
    /// target are set to `q_meas` at rest, and any previous trajectory is
    /// dropped.
    pub fn activate(&mut self, q_meas: &[f64; NUM_JOINTS]) {
        for (j, &q) in q_meas.iter().enumerate() {
            self.input.current_position[j] = q;
            self.input.current_velocity[j] = 0.0;
            self.input.current_acceleration[j] = 0.0;
            self.input.target_position[j] = q;
            self.input.target_velocity[j] = 0.0;
            self.input.target_acceleration[j] = 0.0;
        }
        self.input.control_interface = ControlInterface::Position;
        self.otg.reset();
        self.active = true;
    }

    /// Set a new position target (to be reached at rest). The tracker
    /// re-plans from its current kinematic state on the next [`step`].
    ///
    /// [`step`]: StreamingExecutor::step
    pub fn set_target(&mut self, q_target: &[f64; NUM_JOINTS]) -> Result<(), MotionError> {
        if !self.active {
            return Err(MotionError::InvalidInput {
                what: "set_target",
                reason: "streaming executor is not activated".into(),
            });
        }
        if q_target.iter().any(|v| !v.is_finite()) {
            return Err(MotionError::InvalidInput {
                what: "q_target",
                reason: format!("joint positions must be finite, got {q_target:?}"),
            });
        }
        // A position target is a position-interface request. After a
        // `release` the executor is on the velocity interface, where a
        // target position is never read: a session resumed there would
        // hold at rest and ignore every target it was handed.
        self.input.control_interface = ControlInterface::Position;
        for (j, &q) in q_target.iter().enumerate() {
            self.input.target_position[j] = q;
            self.input.target_velocity[j] = 0.0;
            self.input.target_acceleration[j] = 0.0;
        }
        Ok(())
    }

    /// Brake to rest from wherever the arm is, under the configured
    /// acceleration and jerk limits.
    ///
    /// The streaming counterpart of `JogEngine::release`. A position
    /// target cannot express this: targeting the current pose while the
    /// arm is moving asks Ruckig to stop AND come back, so the arm
    /// overshoots and reverses. Switching to the velocity interface with
    /// a zero target says "shed the velocity you have" and nothing about
    /// where that leaves the arm, which is what a stop is.
    ///
    /// Whoever calls this owns the mode afterwards: the executor keeps
    /// stepping, and the caller ends the session once the ramp reports
    /// rest — a stream dropped straight into IDLE at speed is not
    /// commanded to stop by anything, and coasts on its own momentum.
    pub fn release(&mut self) {
        self.input.control_interface = ControlInterface::Velocity;
        for j in 0..NUM_JOINTS {
            self.input.target_velocity[j] = 0.0;
            self.input.target_acceleration[j] = 0.0;
        }
    }

    /// Advance one tick toward the current target.
    pub fn step(&mut self) -> Result<StreamStep, MotionError> {
        if !self.active {
            return Err(MotionError::InvalidInput {
                what: "step",
                reason: "streaming executor is not activated".into(),
            });
        }
        let res = self
            .otg
            .update(&self.input, &mut self.output)
            .map_err(|e| MotionError::Ruckig(e.to_string()))?;
        let finished = matches!(res, RuckigResult::Finished);
        if !finished && !matches!(res, RuckigResult::Working) {
            return Err(MotionError::Ruckig(format!(
                "streaming step failed: {res:?}"
            )));
        }
        let mut q = [0.0; NUM_JOINTS];
        let mut qd = [0.0; NUM_JOINTS];
        for j in 0..NUM_JOINTS {
            q[j] = self.output.new_position[j];
            qd[j] = self.output.new_velocity[j];
        }
        self.output.pass_to_input(&mut self.input);
        Ok(StreamStep { q, qd, finished })
    }
}

/// Below this the remaining tangent delta is too small to take a
/// direction from: metres and radians mixed, so it is a magnitude, not
/// a tolerance on either.
const DIRECTION_EPS: f64 = 1e-12;

/// TCP kinodynamic ceilings for [`CartesianStreamingExecutor`], split
/// into the linear and angular halves of the SE(3) tangent.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CartLimits {
    /// Linear TCP velocity \[m/s\].
    pub linear_velocity: f64,
    /// Angular TCP velocity \[rad/s\].
    pub angular_velocity: f64,
    /// Linear TCP acceleration \[m/s²\].
    pub linear_acceleration: f64,
    /// Angular TCP acceleration \[rad/s²\].
    pub angular_acceleration: f64,
    /// Linear TCP jerk \[m/s³\].
    pub linear_jerk: f64,
    /// Angular TCP jerk \[rad/s³\].
    pub angular_jerk: f64,
}

impl CartLimits {
    /// Read the `[motion]` cartesian envelope from a validated config.
    pub fn from_config(cfg: &RobotConfig) -> Self {
        let m = &cfg.motion;
        Self {
            linear_velocity: m.jog_l_linear_max_m_s,
            angular_velocity: m.jog_l_angular_max_rad_s,
            linear_acceleration: m.cart_linear_accel_max_m_s2,
            angular_acceleration: m.cart_angular_accel_max_rad_s2,
            linear_jerk: m.cart_linear_jerk_max_m_s3,
            angular_jerk: m.cart_angular_jerk_max_rad_s3,
        }
    }
}

/// One tick of cartesian streaming output.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CartStep {
    /// The smoothed TCP pose for this tick.
    pub pose: Pose,
    /// True once the target pose is reached, at rest.
    pub finished: bool,
}

/// Online jerk-limited TCP pose tracker: the cartesian counterpart of
/// [`StreamingExecutor`], and what makes `servo_l` a LINE rather than a
/// joint interpolation that merely ends in the right place.
///
/// The limiter runs on the SE(3) tangent taken about the pose the
/// stream was activated at: [`activate`] pins that reference, so the
/// current tangent is the origin and a target's tangent is
/// `log(T_ref⁻¹·T_target)`. Driving that six-vector to zero velocity
/// traces the screw geodesic between the two poses — a straight TCP
/// line whenever the rotation is zero, and the shortest SE(3) path
/// otherwise. Smoothing in joint space cannot do this: it produces a
/// smooth JOINT trajectory whose TCP bows off the line.
///
/// What comes out is a pose per tick. Turning that into joints is the
/// caller's job and stays an exact global solve — par6 has closed-form
/// OPW IK, so there is nothing a differential (CLIK-style) step would
/// buy here: it would be a first-order approximation of a solve that is
/// already both cheaper and exact.
///
/// Lifecycle: [`activate`] on session claim, [`set_target`] whenever a
/// setpoint arrives, [`step`] once per RT tick, [`release`] to brake.
///
/// [`activate`]: CartesianStreamingExecutor::activate
/// [`set_target`]: CartesianStreamingExecutor::set_target
/// [`step`]: CartesianStreamingExecutor::step
/// [`release`]: CartesianStreamingExecutor::release
pub struct CartesianStreamingExecutor {
    otg: Ruckig<6, ThrowErrorHandler>,
    input: InputParameter<6>,
    output: OutputParameter<6>,
    limits: CartLimits,
    /// The pose the tangent coordinates are taken about.
    reference: Pose,
    /// Unit direction of the tangent to the current target; all zeroes
    /// when there is no target, which leaves the envelope isotropic.
    direction: [f64; 6],
    /// The target currently being tracked, so setting the same one
    /// again can be recognized and skipped.
    last_target: Option<Pose>,
    speed: f64,
    accel: f64,
    active: bool,
}

impl CartesianStreamingExecutor {
    /// Build for tick period `dt` \[s\] under the TCP envelope `limits`.
    pub fn new(dt: f64, limits: CartLimits) -> Result<Self, MotionError> {
        if !(dt.is_finite() && dt > 0.0 && dt < 1.0) {
            return Err(MotionError::InvalidInput {
                what: "dt",
                reason: format!("must be a finite tick period in (0, 1) s, got {dt}"),
            });
        }
        for (v, what) in [
            (limits.linear_velocity, "linear_velocity"),
            (limits.angular_velocity, "angular_velocity"),
            (limits.linear_acceleration, "linear_acceleration"),
            (limits.angular_acceleration, "angular_acceleration"),
            (limits.linear_jerk, "linear_jerk"),
            (limits.angular_jerk, "angular_jerk"),
        ] {
            if !(v.is_finite() && v > 0.0) {
                return Err(MotionError::InvalidInput {
                    what: "cart limits",
                    reason: format!("{what} must be finite and > 0, got {v}"),
                });
            }
        }
        let mut exec = Self {
            otg: Ruckig::<6, ThrowErrorHandler>::new(None, dt),
            input: InputParameter::<6>::new(None),
            output: OutputParameter::<6>::new(None),
            limits,
            reference: IDENTITY_POSE,
            direction: [0.0; 6],
            last_target: None,
            speed: 1.0,
            accel: 1.0,
            active: false,
        };
        // Ruckig's default (`Time`) only makes the six components FINISH
        // together; each still takes its own time-optimal route there, so
        // the tangent bows and the TCP leaves the line by millimetres.
        // `Phase` holds them to one shared profile, which is what makes
        // the interpolation the screw geodesic rather than merely an
        // interpolation that ends in the right place. Ruckig falls back
        // to time synchronization by itself when the limits make a shared
        // profile impossible.
        exec.input.synchronization = Synchronization::Phase;
        exec.apply_limits();
        Ok(exec)
    }

    /// Scale the envelope by a speed and an acceleration fraction, as
    /// the per-setpoint `speed`/`accel` carry. Jerk is not scaled: it
    /// bounds how sharply the profile may change shape, which is a
    /// property of the arm rather than of how fast this move is asked
    /// to run.
    pub fn set_scale(&mut self, speed: f64, accel: f64) {
        self.speed = speed.clamp(f64::MIN_POSITIVE, 1.0);
        self.accel = accel.clamp(f64::MIN_POSITIVE, 1.0);
        self.apply_limits();
    }

    /// Push the envelope into Ruckig, converted from TCP norms to the
    /// per-component ceilings Ruckig enforces.
    ///
    /// The configured ceilings are TCP speeds — what the tool may travel
    /// at, not what each axis may. Ruckig bounds each component on its
    /// own, so an isotropic envelope lets a diagonal move run the norm up
    /// to √3 times the ceiling. Under phase synchronization the six
    /// components share one profile, so the tangent runs along a fixed
    /// direction `d̂` at some scalar rate: the component Ruckig binds on
    /// is the largest `|d̂ₖ|`, and the norm this crate cares about is
    /// `|d̂|` over that half. Scaling by their ratio makes the two agree,
    /// so a ceiling of 0.08 m/s means 0.08 m/s of TOOL travel whichever
    /// way the move points.
    fn apply_limits(&mut self) {
        let d = &self.direction;
        let half = |a: f64, b: f64, c: f64| {
            let norm = (a * a + b * b + c * c).sqrt();
            if norm > 0.0 {
                a.abs().max(b.abs()).max(c.abs()) / norm
            } else {
                1.0
            }
        };
        let lin = half(d[0], d[1], d[2]);
        let ang = half(d[3], d[4], d[5]);
        for k in 0..3 {
            self.input.max_velocity[k] = self.limits.linear_velocity * self.speed * lin;
            self.input.max_acceleration[k] = self.limits.linear_acceleration * self.accel * lin;
            self.input.max_jerk[k] = self.limits.linear_jerk * lin;
            self.input.max_velocity[3 + k] = self.limits.angular_velocity * self.speed * ang;
            self.input.max_acceleration[3 + k] =
                self.limits.angular_acceleration * self.accel * ang;
            self.input.max_jerk[3 + k] = self.limits.angular_jerk * ang;
        }
    }

    /// Pin the tangent reference to `pose` and start from rest there.
    ///
    /// Every tangent the executor handles afterwards is taken about
    /// this pose, so it is re-pinned on each stream claim rather than
    /// carried across: tangent coordinates degenerate as the relative
    /// rotation approaches π, and a reference left behind by an earlier
    /// session is exactly how a stream drifts into that.
    pub fn activate(&mut self, pose: &Pose) {
        self.reference = *pose;
        for k in 0..6 {
            self.input.current_position[k] = 0.0;
            self.input.current_velocity[k] = 0.0;
            self.input.current_acceleration[k] = 0.0;
            self.input.target_position[k] = 0.0;
            self.input.target_velocity[k] = 0.0;
            self.input.target_acceleration[k] = 0.0;
        }
        self.input.control_interface = ControlInterface::Position;
        self.otg.reset();
        self.last_target = None;
        self.active = true;
    }

    /// Track `target`, to be reached at rest. Re-plans from the current
    /// kinematic state, so retargeting mid-motion stays smooth.
    pub fn set_target(&mut self, target: &Pose) -> Result<(), MotionError> {
        if !self.active {
            return Err(MotionError::InvalidInput {
                what: "set_target",
                reason: "cartesian streaming executor is not activated".into(),
            });
        }
        if target.iter().any(|v| !v.is_finite()) {
            return Err(MotionError::InvalidInput {
                what: "target",
                reason: "target pose must be finite".into(),
            });
        }
        // Re-planning a target the OTG is already tracking costs the
        // phase synchronization that keeps the tool on its line: the
        // re-plan tests the current velocity and acceleration against a
        // fresh profile and drops to time synchronization unless they
        // line up exactly, and once out of phase the state drifts
        // further out, so the next tick fails the test too. A servo
        // stream repeats its target at the tick rate, so this is the
        // common case rather than an edge one.
        if self.last_target == Some(*target) {
            return Ok(());
        }
        self.last_target = Some(*target);
        let tangent = cart::se3_log(&cart::se3_mul(&cart::se3_inverse(&self.reference), target));
        // As in `StreamingExecutor::set_target`: a pose target is a
        // position-interface request, and a stream resumed after a
        // `release` is on the velocity interface, where it would be
        // ignored.
        self.input.control_interface = ControlInterface::Position;
        for (k, &v) in tangent.iter().enumerate() {
            self.input.target_position[k] = v;
            self.input.target_velocity[k] = 0.0;
            self.input.target_acceleration[k] = 0.0;
        }
        // The envelope is direction-dependent (see `apply_limits`), and
        // the direction is the one from where the arm IS to the target.
        let mut delta = [0.0; 6];
        let mut norm = 0.0;
        for k in 0..6 {
            delta[k] = tangent[k] - self.input.current_position[k];
            norm += delta[k] * delta[k];
        }
        let norm = norm.sqrt();
        // Retargeting every tick is normal for a servo stream, and the
        // remaining delta shrinks to nothing as the move lands. Keeping
        // the last direction there stops the envelope flipping back to
        // isotropic on the final approach, where it would change the
        // limits under a move that is still running.
        if norm > DIRECTION_EPS {
            self.direction = std::array::from_fn(|k| delta[k] / norm);
        }
        self.apply_limits();
        Ok(())
    }

    /// Brake the TCP to rest from wherever it is, under the configured
    /// acceleration and jerk.
    ///
    /// The cartesian counterpart of [`StreamingExecutor::release`], and
    /// the same reasoning: targeting the current pose while the TCP is
    /// moving asks the OTG to stop AND come back, which overshoots and
    /// reverses. The velocity interface with a zero target says only
    /// "shed the velocity you have", which is what a stop is.
    pub fn release(&mut self) {
        // Braking leaves the position interface, so the target it was
        // tracking no longer holds.
        self.last_target = None;
        self.input.control_interface = ControlInterface::Velocity;
        for k in 0..6 {
            self.input.target_velocity[k] = 0.0;
            self.input.target_acceleration[k] = 0.0;
        }
    }

    /// Advance one tick along the geodesic toward the current target.
    pub fn step(&mut self) -> Result<CartStep, MotionError> {
        if !self.active {
            return Err(MotionError::InvalidInput {
                what: "step",
                reason: "cartesian streaming executor is not activated".into(),
            });
        }
        let res = self
            .otg
            .update(&self.input, &mut self.output)
            .map_err(|e| MotionError::Ruckig(e.to_string()))?;
        let finished = matches!(res, RuckigResult::Finished);
        if !finished && !matches!(res, RuckigResult::Working) {
            return Err(MotionError::Ruckig(format!(
                "cartesian streaming step failed: {res:?}"
            )));
        }
        let mut tangent = [0.0; 6];
        for (k, out) in tangent.iter_mut().enumerate() {
            *out = self.output.new_position[k];
        }
        self.output.pass_to_input(&mut self.input);
        Ok(CartStep {
            pose: cart::se3_mul(&self.reference, &cart::se3_exp(&tangent)),
            finished,
        })
    }
}
