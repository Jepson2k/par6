//! FFI bindings to `par6_shim` — the C-ABI shim over Pinocchio
//! (kinematics/dynamics, consumed by `par6-kin`) and toppra-cpp
//! (time-optimal path parameterization, for `par6-motion`).
//!
//! Two layers:
//! - [`ffi`]: raw `extern "C"` declarations mirroring `cpp/include/par6_shim.h`.
//! - [`Model`] / [`Trajectory`] / [`CollisionModel`]: minimal safe wrappers
//!   with dimension checking and RAII. [`Model`] covers create / fk /
//!   jacobian / gravity / aba / ik_step; [`Trajectory`] covers TOPPRA
//!   parameterize / duration / allocation-free sampling;
//!   [`CollisionModel`] covers the coal geometry world — installation and
//!   program shape layers, in-collision verdict and colliding pairs.
//!
//! The shim is a hard prerequisite: build it with `scripts/ffi/setup.sh`
//! once, then `source .ffi/env.sh` in each shell.
//!
//! Conventions (fixed by the C header):
//! - poses: 4x4 homogeneous, row-major, 16 `f64`
//! - jacobians: 6 x nq, row-major, rows `[linear; angular]`,
//!   LOCAL_WORLD_ALIGNED frame
//! - gravity: RNEA at zero velocity/acceleration, `nq` torques

pub mod ffi;

mod model;

mod traj;

mod collision;

pub use model::{Error, IkOptions, Model, ToolParams};

pub use traj::{PathDegree, Trajectory};

pub use collision::{CollisionModel, Layer, ShapeDesc};
