//! Raw FFI to `par6_shim` (the C-ABI shim over Pinocchio, coal and
//! toppra-cpp) plus the RAII handles over it.
//!
//! [`ffi`] mirrors `cpp/include/par6_shim.h` declaration for declaration.
//! [`Model`], [`CollisionModel`] and [`Trajectory`] own a shim handle each
//! and check dimensions before crossing.
//!
//! Conventions (fixed by the C header):
//! - poses: 4x4 homogeneous, row-major, 16 `f64`
//! - jacobians: 6 x nq, row-major, rows `[linear; angular]`,
//!   LOCAL_WORLD_ALIGNED frame
//! - gravity: RNEA at zero velocity/acceleration, `nq` torques

mod collision;
pub mod ffi;
mod model;
mod traj;

pub use collision::{CollisionModel, Layer, ShapeDesc};
pub use model::{Error, IkOptions, Model, ToolParams};
pub use traj::{PathDegree, Trajectory};
