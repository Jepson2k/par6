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

use crate::MAX_JOINTS;

/// Per-tick gravity torque source.
pub trait GravityModel: Send {
    /// Write G(q) \[Nm\] for the arm joints into `out`. Must not allocate
    /// and must not block — it runs on the RT thread every tick.
    fn gravity(&mut self, q: &[f64; MAX_JOINTS], out: &mut [f64; MAX_JOINTS]);

    /// Replace the runtime payload carried at the TCP (mass \[kg\], COM
    /// \[m\] in ee-frame coordinates, rotational inertia about the COM,
    /// `None` = point mass). Inputs are validated at the wire before
    /// they reach here. Models without a payload notion ignore it.
    fn set_payload(&mut self, _mass: f64, _com: [f64; 3], _inertia: Option<[f64; 6]>) {}
}

/// Zero-torque model: gravity compensation contributes nothing.
#[derive(Debug, Default, Clone, Copy)]
pub struct ZeroGravity;

impl GravityModel for ZeroGravity {
    fn gravity(&mut self, _q: &[f64; MAX_JOINTS], out: &mut [f64; MAX_JOINTS]) {
        out.fill(0.0);
    }
}
