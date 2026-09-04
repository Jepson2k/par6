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
//!
//! The real model lives in `par6d` (`KinGravity` over `par6_kin::Kin`),
//! which is where the payload the wire declares is applied.

use crate::MAX_JOINTS;

/// Per-tick gravity torque source.
pub trait GravityModel: Send {
    /// Write G(q) \[Nm\] for the arm joints into `out`. Must not allocate
    /// and must not block — it runs on the RT thread every tick.
    fn gravity(&mut self, q: &[f64; MAX_JOINTS], out: &mut [f64; MAX_JOINTS]);

    /// Replace the runtime payload carried at the TCP (mass \[kg\], COM
    /// \[m\] in ee-frame coordinates, rotational inertia about the COM,
    /// `None` = point mass). Inputs are validated at the wire before
    /// they reach here.
    ///
    /// No default body on purpose. A model that inherited one would
    /// accept a declared payload, drop it, and hold the arm against a
    /// load it does not know about — with the command acked all the way
    /// back to the caller. An implementation with no payload notion says
    /// so here, in one line, deliberately.
    fn set_payload(&mut self, mass: f64, com: [f64; 3], inertia: Option<[f64; 6]>);
}

/// Zero-torque model: gravity compensation contributes nothing.
#[derive(Debug, Default, Clone, Copy)]
pub struct ZeroGravity;

impl GravityModel for ZeroGravity {
    fn gravity(&mut self, _q: &[f64; MAX_JOINTS], out: &mut [f64; MAX_JOINTS]) {
        out.fill(0.0);
    }

    /// Nothing to carry it: this model compensates no gravity at all, so
    /// a payload changes nothing rather than being quietly lost.
    fn set_payload(&mut self, _mass: f64, _com: [f64; 3], _inertia: Option<[f64; 6]>) {}
}
