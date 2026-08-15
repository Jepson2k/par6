//! Streaming (servo) executor: the jerk-limited per-tick OTG limiter for
//! online position targets.
//!
//! rsruckig tracks the newest target under the stream-mode limits;
//! retargeting mid-motion re-plans from the current kinematic state, so
//! moving targets stay smooth. Soft-limit clamping of targets and outputs
//! is the RT streaming pipeline's job (it clamps unconditionally, before
//! and after this limiter) — this type owns only the OTG step.

use rsruckig::prelude::*;

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
        for (j, &q) in q_target.iter().enumerate() {
            self.input.target_position[j] = q;
            self.input.target_velocity[j] = 0.0;
            self.input.target_acceleration[j] = 0.0;
        }
        Ok(())
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
