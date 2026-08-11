//! Motion-planning error type.

/// Error produced by trajectory planning, jogging, completion tracking, or
/// streaming execution.
#[derive(Debug, thiserror::Error)]
pub enum MotionError {
    /// An input value is outside its contract (NaN/inf/zero/negative or out
    /// of range). `what` names the offending parameter.
    #[error("invalid value for `{what}`: {reason}")]
    InvalidInput {
        /// Name of the offending parameter.
        what: &'static str,
        /// Constraint that was violated.
        reason: String,
    },
    /// The robot config does not match the compile-time joint count the
    /// motion types are dimensioned for.
    #[error("config has {actual} joints, motion types are dimensioned for {expected}")]
    JointCountMismatch {
        /// Joints in the config.
        actual: usize,
        /// Compile-time joint count ([`crate::NUM_JOINTS`]).
        expected: usize,
    },
    /// A planned move target lies outside the soft limit window.
    #[error("move target for joint {joint} ({value} rad) is outside soft limits [{min}, {max}]")]
    TargetOutsideSoftLimits {
        /// Joint index (0-based).
        joint: usize,
        /// Requested target \[rad\].
        value: f64,
        /// Soft limit, negative side \[rad\].
        min: f64,
        /// Soft limit, positive side \[rad\].
        max: f64,
    },
    /// Two consecutive moves with different profiles were linked by a blend.
    /// Corner blending is generated per profile family (trapezoid overlap
    /// vs. ruckig waypoint chain), so a blend chain must use one profile.
    #[error("moves {first} and {second} blend across different profiles; a blend chain must use a single profile")]
    MixedProfileBlend {
        /// Index of the earlier move in the chain.
        first: usize,
        /// Index of the later move.
        second: usize,
    },
    /// A jerk-limited profile was requested but the resolved mode limits
    /// carry no finite jerk limit for this joint.
    #[error("joint {joint} has no finite jerk limit; required by the ruckig profile")]
    MissingJerkLimit {
        /// Joint index (0-based).
        joint: usize,
    },
    /// rsruckig rejected the trajectory input or failed to solve.
    #[error("ruckig: {0}")]
    Ruckig(String),
    /// Strict completion policy: the arm did not settle within the timeout.
    #[error(
        "settle timeout: joint {worst_joint} error {error_rad} rad still above tolerance {tolerance_rad} rad"
    )]
    SettleTimeout {
        /// Joint with the largest position error at timeout.
        worst_joint: usize,
        /// That joint's |q_meas − q_target| \[rad\].
        error_rad: f64,
        /// The settle tolerance in force \[rad\].
        tolerance_rad: f64,
    },
}
