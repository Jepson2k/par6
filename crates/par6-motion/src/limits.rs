//! Per-joint kinodynamic limits resolved from config for one control mode.

use par6_config::{LimitMode, RobotConfig};

use crate::{MotionError, NUM_JOINTS};

/// Fully-resolved per-joint limits plus the soft position window, in the
/// fixed-size arrays the motion generators run on.
///
/// Built from a [`RobotConfig`] with [`MotionLimits::from_config`], which
/// applies the per-mode fallback-to-ceiling rule (`JointLimits::for_mode`).
/// A missing jerk limit is stored as `f64::INFINITY`; profiles that need a
/// finite jerk (ruckig) reject it with [`MotionError::MissingJerkLimit`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionLimits {
    /// Joint velocity limits \[rad/s\].
    pub velocity: [f64; NUM_JOINTS],
    /// Joint acceleration limits \[rad/s^2\].
    pub acceleration: [f64; NUM_JOINTS],
    /// Joint jerk limits \[rad/s^3\]; `INFINITY` = unconstrained.
    pub jerk: [f64; NUM_JOINTS],
    /// Soft position limit, negative side \[rad\].
    pub soft_min: [f64; NUM_JOINTS],
    /// Soft position limit, positive side \[rad\].
    pub soft_max: [f64; NUM_JOINTS],
}

impl MotionLimits {
    /// Resolve the limits `mode` runs under from a validated robot config.
    ///
    /// Fails with [`MotionError::JointCountMismatch`] when the config does
    /// not have exactly [`NUM_JOINTS`] joints.
    pub fn from_config(cfg: &RobotConfig, mode: LimitMode) -> Result<Self, MotionError> {
        if cfg.joints.len() != NUM_JOINTS {
            return Err(MotionError::JointCountMismatch {
                actual: cfg.joints.len(),
                expected: NUM_JOINTS,
            });
        }
        let mut out = Self {
            velocity: [0.0; NUM_JOINTS],
            acceleration: [0.0; NUM_JOINTS],
            jerk: [0.0; NUM_JOINTS],
            soft_min: [0.0; NUM_JOINTS],
            soft_max: [0.0; NUM_JOINTS],
        };
        for (j, joint) in cfg.joints.iter().enumerate() {
            let r = joint.limits.for_mode(mode);
            out.velocity[j] = r.velocity_rad_s;
            out.acceleration[j] = r.acceleration_rad_s2;
            out.jerk[j] = r.jerk_rad_s3.unwrap_or(f64::INFINITY);
            out.soft_min[j] = joint.limits.soft_min_rad;
            out.soft_max[j] = joint.limits.soft_max_rad;
        }
        Ok(out)
    }

    /// Error unless every joint carries a finite jerk limit.
    pub(crate) fn require_finite_jerk(&self) -> Result<(), MotionError> {
        for (j, &jerk) in self.jerk.iter().enumerate() {
            if !jerk.is_finite() {
                return Err(MotionError::MissingJerkLimit { joint: j });
            }
        }
        Ok(())
    }

    /// Error unless `q` lies inside the soft window on every joint.
    /// [`ProgramBuilder`](crate::ProgramBuilder) applies this to every
    /// queued move; planners that time a path themselves apply it to
    /// their own targets.
    pub fn require_inside_soft(&self, q: &[f64; NUM_JOINTS]) -> Result<(), MotionError> {
        for (j, &v) in q.iter().enumerate() {
            if !(v >= self.soft_min[j] && v <= self.soft_max[j]) {
                return Err(MotionError::TargetOutsideSoftLimits {
                    joint: j,
                    value: v,
                    min: self.soft_min[j],
                    max: self.soft_max[j],
                });
            }
        }
        Ok(())
    }
}
