//! Geometric path abstraction: the seam between profile generation and
//! path geometry.
//!
//! A [`PathSampler`] maps a normalized path coordinate `s ∈ [0, 1]` to
//! joint positions. Profile generation (trapezoid today, TOPPRA when the
//! C++ shim lands) runs a scalar velocity profile on `s` and samples the
//! path at tick rate — it never needs to know where the geometry came
//! from.
//!
//! Joint-space moves use [`JointLinePath`] (a straight line in joint
//! space). Cartesian paths (move_l / move_s / move_p) are NOT implemented
//! here: the planner will implement `PathSampler` over an IK-solved
//! waypoint chain from `par6-kin` and feed it through the same profile
//! machinery. That implementation — and the curvature-aware constraint
//! handling it needs (TOPPRA) — lives with the planner workstream; this
//! trait is the contract it codes against.

use crate::NUM_JOINTS;

/// Joint-space geometry sampled by normalized path coordinate.
///
/// Implementations must be defined on all of `s ∈ [0, 1]` and continuous;
/// values outside the interval are clamped by callers before sampling.
pub trait PathSampler {
    /// Write the joint positions at path coordinate `s ∈ [0, 1]` \[rad\].
    fn sample(&self, s: f64, q_out: &mut [f64; NUM_JOINTS]);

    /// Write `dq/ds` at `s` \[rad per unit path\].
    ///
    /// The default computes a central finite difference of [`sample`]
    /// (`h = 1e-6`, one-sided at the interval ends); implementations with
    /// an analytic derivative should override it.
    ///
    /// [`sample`]: PathSampler::sample
    fn derivative(&self, s: f64, dq_ds_out: &mut [f64; NUM_JOINTS]) {
        const H: f64 = 1e-6;
        let lo = (s - H).max(0.0);
        let hi = (s + H).min(1.0);
        let mut q_lo = [0.0; NUM_JOINTS];
        let mut q_hi = [0.0; NUM_JOINTS];
        self.sample(lo, &mut q_lo);
        self.sample(hi, &mut q_hi);
        let span = hi - lo;
        for j in 0..NUM_JOINTS {
            dq_ds_out[j] = if span > 0.0 {
                (q_hi[j] - q_lo[j]) / span
            } else {
                0.0
            };
        }
    }
}

/// Straight line in joint space from `start` to `end` — the geometry of a
/// joint-space move (move_j).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointLinePath {
    start: [f64; NUM_JOINTS],
    end: [f64; NUM_JOINTS],
}

impl JointLinePath {
    /// Line from `start` to `end` \[rad\].
    pub fn new(start: [f64; NUM_JOINTS], end: [f64; NUM_JOINTS]) -> Self {
        Self { start, end }
    }
}

impl PathSampler for JointLinePath {
    fn sample(&self, s: f64, q_out: &mut [f64; NUM_JOINTS]) {
        let s = s.clamp(0.0, 1.0);
        for (out, (a, b)) in q_out.iter_mut().zip(self.start.iter().zip(self.end.iter())) {
            *out = a + s * (b - a);
        }
    }

    fn derivative(&self, _s: f64, dq_ds_out: &mut [f64; NUM_JOINTS]) {
        for (out, (a, b)) in dq_ds_out
            .iter_mut()
            .zip(self.start.iter().zip(self.end.iter()))
        {
            *out = b - a;
        }
    }
}
