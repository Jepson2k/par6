//! Timing a cartesian path against the distance the TOOL travels.
//!
//! A cartesian move exists to control the tool: welding, dispensing,
//! cutting and gluing all care how fast the tip crosses the work, not
//! how fast the joints turn. Time the path against joint displacement —
//! which is what handing a joint-limited solver a waypoint list does —
//! and the tool gets whatever speed the joint-optimal answer happens to
//! yield: quick along a straight stretch, slow through a wrist
//! reconfiguration, and never the constant a process move asks for.
//!
//! So the knots here sit at TRUE cumulative tool arc length rather than
//! at even parameter spacing. The path parameter then IS tool distance,
//! a constant `ds/dt` IS a constant tool speed, and the solver can be
//! held to one by a ceiling on `ds/dt` alone.
//!
//! What this module does NOT do is time the path. The geometry is
//! handed to TOPPRA as a degree-1 path — straight lines between the
//! poses IK actually solved, so nothing is invented between them — and
//! TOPPRA prices the turning at the knots itself. A scalar profile over
//! the same coordinate cannot: it sees only `|dq/ds|`, which is the cost
//! of going ALONG the path and says nothing about the cost of turning.

use crate::limits::MotionLimits;
use crate::NUM_JOINTS;

/// Arc-length increments below this are a repeated waypoint: no extent,
/// and a zero-width interval to divide by.
const MIN_SPAN: f64 = 1e-12;

/// Cumulative distance the tool travels along a pose chain.
///
/// `steps` are per-segment `(translation_m, rotation_rad)`; rotation is
/// folded in as the arc a point `rot_weight_m_per_rad` from the tool
/// sweeps, so a reorientation in place still covers distance and a pure
/// rotation is commensurable with a pure translation.
pub fn tool_arc_lengths(steps: &[(f64, f64)], rot_weight_m_per_rad: f64) -> Vec<f64> {
    let mut out = Vec::with_capacity(steps.len() + 1);
    let mut acc = 0.0;
    out.push(acc);
    for (d_trans, d_rot) in steps {
        acc += d_trans.hypot(rot_weight_m_per_rad * d_rot);
        out.push(acc);
    }
    out
}

/// A joint chain keyed to the normalized tool distance at each waypoint.
pub struct ArcKnots {
    /// Normalized cumulative tool arc length per knot, strictly
    /// increasing from 0.0 to 1.0.
    s: Vec<f64>,
    q: Vec<[f64; NUM_JOINTS]>,
    /// Total tool distance the path covers \[m\].
    length_m: f64,
}

impl ArcKnots {
    /// Key `q` to the tool distance in `cart_s`, dropping waypoints that
    /// cover no distance so the knots are strictly increasing. `None`
    /// when the path has no extent to time.
    pub fn new(q: &[[f64; NUM_JOINTS]], cart_s: &[f64]) -> Option<Self> {
        debug_assert_eq!(q.len(), cart_s.len());
        let total = *cart_s.last()? - *cart_s.first()?;
        if !total.is_finite() || total <= MIN_SPAN {
            return None;
        }
        let base = cart_s[0];
        let mut s = Vec::with_capacity(q.len());
        let mut qs = Vec::with_capacity(q.len());
        for (qi, si) in q.iter().zip(cart_s) {
            let sn = (si - base) / total;
            if s.last().is_some_and(|&p: &f64| sn - p <= MIN_SPAN) {
                continue; // a repeated pose covers no distance
            }
            s.push(sn);
            qs.push(*qi);
        }
        (s.len() >= 2).then_some(Self {
            s,
            q: qs,
            length_m: total,
        })
    }

    /// The knots, in normalized tool distance.
    pub fn knots(&self) -> &[f64] {
        &self.s
    }

    /// The waypoints, row-major, one row per knot.
    pub fn waypoints_flat(&self) -> Vec<f64> {
        self.q.iter().flatten().copied().collect()
    }

    /// Total tool distance the path covers \[m\].
    pub fn length_m(&self) -> f64 {
        self.length_m
    }

    /// The steepest `|dq/ds|` each joint reaches anywhere on the path —
    /// the per-joint magnitude a path-speed ceiling divides its limits
    /// by.
    ///
    /// Exact rather than probed: the path is affine between knots, so
    /// `dq/ds` is constant within a segment and the extremes are the
    /// segment slopes themselves.
    pub fn max_slope(&self) -> [f64; NUM_JOINTS] {
        let mut worst = [0.0f64; NUM_JOINTS];
        for i in 0..self.s.len() - 1 {
            let h = self.s[i + 1] - self.s[i];
            for (j, w) in worst.iter_mut().enumerate() {
                *w = w.max(((self.q[i + 1][j] - self.q[i][j]) / h).abs());
            }
        }
        worst
    }
}

/// The fastest `ds/dt` the joints allow anywhere on a path of this
/// steepness, at `speed` of their velocity limits.
///
/// Holding the whole path to this one value is what "constant tool
/// speed" costs: the steepest stretch sets the rate, and the rest of the
/// path runs at the same rate rather than at whatever it could manage.
/// On a process move that is the point rather than the price.
pub fn max_path_speed(slope: &[f64; NUM_JOINTS], limits: &MotionLimits, speed: f64) -> Option<f64> {
    let mut v_s = f64::INFINITY;
    for (j, &sl) in slope.iter().enumerate() {
        if sl > MIN_SPAN {
            v_s = v_s.min(limits.velocity[j] * speed / sl);
        }
    }
    (v_s.is_finite() && v_s > 0.0).then_some(v_s)
}
