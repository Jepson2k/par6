//! Keeping a planned move's tool under `[limits] tcp_speed_max_m_s`.
//!
//! A joint-space plan is timed against joint limits, and at reach a joint
//! well inside its own limit swings the tool several times faster than any
//! cartesian move is allowed to carry it. The plan is not re-planned for
//! that: it is the same path, played back slower wherever the tool would
//! otherwise be over the ceiling and at its own pace everywhere else. The
//! clock the plan is read on only ever runs slow, so every joint velocity
//! on it is at most what the plan asked for.
//!
//! Slowing the whole plan by the one factor its fastest instant needs
//! would be simpler and is wrong: a move asked to go flat out peaks high
//! for a moment and would be slowed everywhere for it, ending up behind
//! the same move asked to go at a quarter of the speed.

use par6_rt::MAX_JOINTS;

/// One tick of a plan: `q`, `q̇`, `q̈`.
pub(crate) type PlanSample = [f64; 3 * MAX_JOINTS];

/// The fastest the tool moves between consecutive `points` a tick `dt`
/// apart \[m/s\].
pub(crate) fn peak_speed(points: &[[f64; 3]], dt: f64) -> f64 {
    segment_speeds(points, dt).fold(0.0, f64::max)
}

fn segment_speeds(points: &[[f64; 3]], dt: f64) -> impl Iterator<Item = f64> + '_ {
    points.windows(2).map(move |w| {
        let d: f64 = (0..3).map(|k| (w[1][k] - w[0][k]).powi(2)).sum();
        d.sqrt() / dt
    })
}

/// `samples` — a plan leaving `start_q`, one sample per tick `dt`, whose
/// tool passes through `points` (the start's, then one per sample) —
/// replayed so the tool never exceeds `ceiling` \[m/s\], resampled at the
/// same tick.
///
/// The plan's own clock is advanced each tick by the fraction of a tick
/// the ceiling allows where the plan then is: one where the tool is under
/// it, `ceiling / speed` where it is over. Between ticks the plan is read
/// as the cubic through each pair of samples' positions and velocities,
/// which keeps the replayed velocities continuous. The plan's first tick
/// has no sample before it, so the velocity it left `start_q` with is the
/// one that makes that tick a constant acceleration: zero for a move from
/// rest.
pub(crate) fn held_under(
    start_q: &[f64; MAX_JOINTS],
    samples: &[PlanSample],
    points: &[[f64; 3]],
    ceiling: f64,
    dt: f64,
) -> Vec<PlanSample> {
    let Some(first) = samples.first() else {
        return Vec::new();
    };
    let n = samples.len();
    debug_assert_eq!(points.len(), n + 1, "a tool point per knot");
    let mut start = [0.0; 3 * MAX_JOINTS];
    for j in 0..MAX_JOINTS {
        start[j] = start_q[j];
        start[MAX_JOINTS + j] = 2.0 * (first[j] - start_q[j]) / dt - first[MAX_JOINTS + j];
        start[2 * MAX_JOINTS + j] = first[2 * MAX_JOINTS + j];
    }
    let knot = |i: usize| if i == 0 { &start } else { &samples[i - 1] };

    // How fast the plan's clock may run at each knot: the tool's speed
    // there is taken as the faster of the two ticks that meet at it.
    let speeds: Vec<f64> = segment_speeds(points, dt).collect();
    let rate: Vec<f64> = (0..=n)
        .map(|k| {
            let before = speeds[k.saturating_sub(1).min(n - 1)];
            let after = speeds[k.min(n - 1)];
            let speed = before.max(after);
            if speed > ceiling {
                ceiling / speed
            } else {
                1.0
            }
        })
        .collect();
    let rate_at = |at: f64| {
        let i = (at.floor() as usize).min(n - 1);
        let u = at - i as f64;
        rate[i] + u * (rate[i + 1] - rate[i])
    };

    let end = n as f64;
    let mut out = Vec::with_capacity(n);
    let mut at = 0.0_f64;
    let mut previous_qd: [f64; MAX_JOINTS] = std::array::from_fn(|j| start[MAX_JOINTS + j]);
    while at < end {
        let half = (at + 0.5 * rate_at(at)).min(end);
        at = (at + rate_at(half)).min(end);
        if end - at < 1e-9 {
            at = end;
        }
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
        let running = rate_at(at);
        let mut sample = [0.0; 3 * MAX_JOINTS];
        for j in 0..MAX_JOINTS {
            let (qa, qb) = (a[j], b[j]);
            let (va, vb) = (a[MAX_JOINTS + j] * dt, b[MAX_JOINTS + j] * dt);
            sample[j] = h[0] * qa + h[1] * va + h[2] * qb + h[3] * vb;
            let qd = (dh[0] * qa + dh[1] * va + dh[2] * qb + dh[3] * vb) / dt * running;
            sample[MAX_JOINTS + j] = qd;
            sample[2 * MAX_JOINTS + j] = (qd - previous_qd[j]) / dt;
            previous_qd[j] = qd;
        }
        out.push(sample);
    }
    out
}
