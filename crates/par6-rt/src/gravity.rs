//! Gravity model seam: the G(q) source for the per-mode output laws.
//!
//! G(q) is computed EVERY tick and published always; it is applied as a
//! current feedforward only when `homed ∧ enabled ∧ mode allows ∧ comp
//! enabled` — that gating lives in the tick loop. This module only
//! defines where the torques come from:
//!
//! - [`GravityModel`]: the per-tick contract (`q` → `τ`), allocation-free.
//! - [`ZeroGravity`]: the no-model fallback (all zeros) used in tests and
//!   when no dynamics stack is wired.
//! - `PinokinGravity` (feature `ffi`): RNEA at zero velocity/acceleration
//!   through the Pinocchio shim, including the active tool's inertial
//!   contribution via `pinokin_sys::ToolParams`.

use crate::MAX_JOINTS;

/// Per-tick gravity torque source.
pub trait GravityModel: Send {
    /// Write G(q) \[Nm\] for the arm joints into `out`. Must not allocate
    /// and must not block — it runs on the RT thread every tick.
    fn gravity(&mut self, q: &[f64; MAX_JOINTS], out: &mut [f64; MAX_JOINTS]);
}

/// Zero-torque model: gravity compensation contributes nothing.
#[derive(Debug, Default, Clone, Copy)]
pub struct ZeroGravity;

impl GravityModel for ZeroGravity {
    fn gravity(&mut self, _q: &[f64; MAX_JOINTS], out: &mut [f64; MAX_JOINTS]) {
        out.fill(0.0);
    }
}

#[cfg(feature = "ffi")]
mod pinokin {
    use super::{GravityModel, MAX_JOINTS};
    use std::path::Path;

    /// Gravity via the Pinocchio FFI shim ([`pinokin_sys::Model`]):
    /// RNEA at zero velocity/acceleration over the arm URDF plus an
    /// optional rigid tool. `gravity_into` is allocation-free after
    /// construction, satisfying the RT contract.
    pub struct PinokinGravity {
        model: pinokin_sys::Model,
        scratch: [f64; MAX_JOINTS],
        last_good: [f64; MAX_JOINTS],
    }

    impl PinokinGravity {
        /// Build from a URDF whose `nq` equals the arm joint count.
        /// `tool` attaches the active gripper's inertial contribution.
        pub fn from_urdf(
            urdf: &Path,
            ee_frame: Option<&str>,
            tool: Option<&pinokin_sys::ToolParams>,
        ) -> Result<Self, pinokin_sys::Error> {
            let model = pinokin_sys::Model::from_urdf(urdf, ee_frame, tool)?;
            if model.nq() != MAX_JOINTS {
                return Err(pinokin_sys::Error::Dimension {
                    expected: MAX_JOINTS,
                    got: model.nq(),
                });
            }
            Ok(Self {
                model,
                scratch: [0.0; MAX_JOINTS],
                last_good: [0.0; MAX_JOINTS],
            })
        }
    }

    impl GravityModel for PinokinGravity {
        fn gravity(&mut self, q: &[f64; MAX_JOINTS], out: &mut [f64; MAX_JOINTS]) {
            // A shim failure must not kill the RT thread: hold the last
            // good value (gravity is a feedforward, not a safety path).
            if self.model.gravity_into(q, &mut self.scratch).is_ok() {
                self.last_good = self.scratch;
            }
            *out = self.last_good;
        }
    }
}

#[cfg(feature = "ffi")]
pub use pinokin::PinokinGravity;
