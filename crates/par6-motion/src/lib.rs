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
//! - [`cart`] is the cartesian geometry the queued moves trace before
//!   IK: straight segments (`move_l`), three-point arcs (`move_c`),
//!   cubic splines (`move_s`), and polylines whose corners are rounded
//!   by Bézier blend zones (`move_p`, and blend-radius chains).
//! - [`JogEngine`]: per-joint velocity ramps (trapezoid / s-curve) with
//!   jerk-aware soft-limit lookahead and direction-block latching.
//! - [`StreamingExecutor`]: rsruckig online target tracker for
//!   servo-style streaming targets, stepped at tick rate.
//!
//! # TOPPRA
//!
//! Time-optimal path parameterization lives in C++ toppra behind the
//! shared FFI shim (`par6_traj_*`, safe wrapper `pinokin_sys::Trajectory`)
//! and is driven by `par6d`'s planner, which owns the geometry it times:
//! cartesian `move_l` waypoint chains, and joint-space paths under the
//! TOPPRA motion profile. It is deliberately NOT a [`ProfileKind`] —
//! that keeps this crate free of the FFI dependency — and both paths
//! produce the same tick-rate sample streams under the same ring
//! metadata contract.
//!
//! Generation is planner-side and may allocate; only [`JogEngine::tick`]
//! and [`StreamingExecutor::step`] are meant for the RT thread.

pub mod cart;
mod error;
mod jog;
mod limits;
mod path;
mod plan;
mod sample;
mod stream;

pub use error::MotionError;
pub use jog::{JogDirection, JogEngine, JogTick, MIN_ACCEL_TIME_S, MIN_JERK_FACTOR};
pub use limits::MotionLimits;
pub use path::{JointLinePath, PathSampler};
pub use plan::{MoveParams, Plan, ProfileKind, ProgramBuilder};
pub use sample::{Sample, SampleMeta, NUM_JOINTS};
pub use stream::{StreamStep, StreamingExecutor};
