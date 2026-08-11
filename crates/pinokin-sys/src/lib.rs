//! FFI bindings to `par6_shim` — the C-ABI shim over Pinocchio shared by
//! `par6-kin` (and, once wired, toppra-cpp for `par6-motion`).
//!
//! Two layers:
//! - [`ffi`]: raw `extern "C"` declarations mirroring `cpp/include/par6_shim.h`.
//! - [`Model`]: a minimal safe wrapper (create / fk / jacobian / gravity /
//!   aba / ik_step) with dimension checking and RAII.
//!
//! Everything is gated behind the `ffi` feature (default off) so plain
//! `cargo check` succeeds without the C++ toolchain. Build the shim with
//! `scripts/ffi/setup.sh`, then:
//!
//! ```sh
//! source .ffi/env.sh
//! cargo test --manifest-path crates/pinokin-sys/Cargo.toml --features ffi
//! ```
//!
//! Conventions (fixed by the C header):
//! - poses: 4x4 homogeneous, row-major, 16 `f64`
//! - jacobians: 6 x nq, row-major, rows `[linear; angular]`,
//!   LOCAL_WORLD_ALIGNED frame
//! - gravity: RNEA at zero velocity/acceleration, `nq` torques

#[cfg(feature = "ffi")]
pub mod ffi;

#[cfg(feature = "ffi")]
mod model;

#[cfg(feature = "ffi")]
pub use model::{Error, IkOptions, Model, ToolParams};
