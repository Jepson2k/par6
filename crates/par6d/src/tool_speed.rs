//! Keeping a planned move's tool under `[limits] tcp_speed_max_m_s`.
//!
//! A joint-space plan is timed against joint limits, and at reach a joint
//! well inside its own limit swings the tool several times faster than any
//! cartesian move is allowed to carry it. The plan is not re-planned for
//! that: it is the same path played back slower, the whole of it by the
//! one factor that brings its fastest instant down to the ceiling. Every
//! velocity on it scales by that factor and every acceleration by its
//! square, so a plan that respected its joint limits still does.

use par6_rt::MAX_JOINTS;

/// One tick of a plan: `q`, `q̇`, `q̈`.
pub(crate) type PlanSample = [f64; 3 * MAX_JOINTS];

/// The fastest the tool moves between consecutive `points` a tick `dt`
/// apart \[m/s\].
pub(crate) fn peak_speed(points: &[[f64; 3]], dt: f64) -> f64 {
    points
        .windows(2)
        .map(|w| {
            let d: f64 = (0..3).map(|k| (w[1][k] - w[0][k]).powi(2)).sum();
            d.sqrt() / dt
        })
        .fold(0.0, f64::max)
}

/// `samples` — a plan leaving `start_q`, one sample per tick `dt` — played
/// back `factor` (> 1) times slower, resampled at the same tick.
///
/// Between ticks the plan is read as the cubic through each pair of
/// samples' positions and velocities, which is what makes the stretched
/// velocities continuous. The plan's first tick has no sample before it,
/// so the velocity it left `start_q` with is the one that makes that tick
/// a constant acceleration: zero for a move from rest.
pub(crate) fn stretched(
    start_q: &[f64; MAX_JOINTS],
    samples: &[PlanSample],
    factor: f64,
    dt: f64,
) -> Vec<PlanSample> {
    let Some(first) = samples.first() else {
        return Vec::new();
    };
    let mut start = [0.0; 3 * MAX_JOINTS];
    for j in 0..MAX_JOINTS {
        start[j] = start_q[j];
        start[MAX_JOINTS + j] = 2.0 * (first[j] - start_q[j]) / dt - first[MAX_JOINTS + j];
        start[2 * MAX_JOINTS + j] = first[2 * MAX_JOINTS + j];
    }
    let knot = |i: usize| if i == 0 { &start } else { &samples[i - 1] };
    let n = samples.len();
    let count = ((n as f64) * factor).ceil() as usize;
    let mut out = Vec::with_capacity(count);
    for k in 1..=count {
        // Position on the original plan, in ticks.
        let at = ((k as f64) / factor).min(n as f64);
        let i = (at.floor() as usize).min(n - 1);
        let u = at - i as f64;
        let (a, b) = (knot(i), knot(i + 1));
        let (u2, u3) = (u * u, u * u * u);
        let h = [
            2.0 * u3 - 3.0 * u2 + 1.0,
            u3 - 2.0 * u2 + u,
            -2.0 * u3 + 3.0 * u2,
            u3 - u2,
        ];
        let dh = [
            6.0 * u2 - 6.0 * u,
            3.0 * u2 - 4.0 * u + 1.0,
            -6.0 * u2 + 6.0 * u,
            3.0 * u2 - 2.0 * u,
        ];
        let mut sample = [0.0; 3 * MAX_JOINTS];
        for j in 0..MAX_JOINTS {
            let (qa, qb) = (a[j], b[j]);
            let (va, vb) = (a[MAX_JOINTS + j] * dt, b[MAX_JOINTS + j] * dt);
            sample[j] = h[0] * qa + h[1] * va + h[2] * qb + h[3] * vb;
            sample[MAX_JOINTS + j] =
                (dh[0] * qa + dh[1] * va + dh[2] * qb + dh[3] * vb) / (dt * factor);
            sample[2 * MAX_JOINTS + j] = (a[2 * MAX_JOINTS + j]
                + u * (b[2 * MAX_JOINTS + j] - a[2 * MAX_JOINTS + j]))
                / (factor * factor);
        }
        out.push(sample);
    }
    out
}
