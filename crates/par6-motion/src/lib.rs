//! Motion planning and streaming execution.
//!
//! Division of labor (mirrors parol6):
//! - [`ProgramBuilder`] compiles queued joint-space moves into tick-rate
//!   [`Sample`] streams in the EXEC ring's format: trapezoid
//!   (accel–cruise–decel, slowest-joint synchronized) and rsruckig
//!   (jerk-limited point-to-point, waypoint chains) profiles, corner
//!   blending with `blend_continues` metadata, duration/speed
//!   parameterization.
//! - [`PathSampler`] is the geometry seam for cartesian paths: the
//!   planner will implement it over IK-solved waypoints from `par6-kin`;
//!   joint-space moves run through it today via [`JointLinePath`].
//! - [`JogEngine`]: per-joint velocity ramps (trapezoid / s-curve) with
//!   jerk-aware soft-limit lookahead and direction-block latching.
//! - [`CompletionMonitor`]: commanded / settled / strict completion
//!   policies as controller-side state machines.
//! - [`StreamingExecutor`]: rsruckig online target tracker for
//!   servo-style streaming targets, stepped at tick rate.
//!
//! # TOPPRA slot
//!
//! Time-optimal path parameterization (move_l / move_s / move_p under
//! joint constraints, curvature-aware) belongs to C++ toppra behind the
//! shared FFI shim — its entry points (`par6_traj_*`) are stubbed
//! NOT_IMPLEMENTED until conda-forge ships C++ toppra, so nothing here
//! calls them yet. When the shim lands, a `Toppra` variant slots into
//! [`ProfileKind`]: it consumes [`PathSampler`] geometry (which the
//! trapezoid profile already exercises), produces the same tick-rate
//! sample streams, and needs no changes to the ring metadata contract.
//!
//! Generation is planner-side and may allocate; only [`JogEngine::tick`],
//! [`CompletionMonitor::tick`], and [`StreamingExecutor::step`] are meant
//! for the RT thread.

mod completion;
mod error;
mod jog;
mod limits;
mod path;
mod plan;
mod sample;
mod stream;

pub use completion::{CompletionEvent, CompletionMonitor, CompletionPolicy, SettleParams};
pub use error::MotionError;
pub use jog::{JogDirection, JogEngine, JogTick, MIN_ACCEL_TIME_S, MIN_JERK_FACTOR};
pub use limits::MotionLimits;
pub use path::{JointLinePath, PathSampler};
pub use plan::{MoveParams, Plan, ProfileKind, ProgramBuilder};
pub use sample::{Sample, SampleMeta, NUM_JOINTS};
pub use stream::{StreamStep, StreamingExecutor};
