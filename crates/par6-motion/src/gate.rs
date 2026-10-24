//! A last check on the samples a planner is about to queue.
//!
//! Every generator in this crate respects the acceleration limits it was
//! handed — inside its own model. The models are not the drive. A scalar
//! profile over a curved path prices only the tangential term and drops
//! the centripetal `q'' · ṡ²` one; a time-optimal parameterization
//! satisfies its constraints at its gridpoints and says nothing about
//! the trajectory between them, where a spline can bulge well past them.
//! Both are correct implementations that can still emit a stream the arm
//! must not be asked to follow.
//!
//! So the check here is not on any planner's internal state: it is on
//! the finished sample stream, differenced the way the drive experiences
//! it. The ring carries position and velocity, and the velocity channel
//! is what the joint controller tracks, so the quantity that matters is
//! the step between consecutive commanded velocities over one tick —
//! not the `qdd` column, which is a planner's own opinion and only ever
//! feeds the torque feedforward.
//!
//! The check has exactly two outcomes: the stream queues byte-for-byte
//! unchanged, or the move is refused naming the joint, the sample, the
//! value and the limit. It never clamps and never rescales. A clamp
//! would hand back a trajectory that no longer ends where the client
//! asked, under a name that says it does.

use crate::{MotionError, NUM_JOINTS};

/// Headroom over the acceleration limit before a stream is refused.
///
/// Differencing velocity samples reads slightly high — the difference is
/// the mean acceleration across a tick, and lands on the limit exactly
/// when a profile saturates it — so a bare comparison refuses correct
/// saturated moves on rounding alone.
///
/// The size of that artifact is measured, not guessed. The reference
/// runtime fuzzed 420 moves across every lane and profile and saw a
/// worst case of 101.9% (its solver-timed lane; the scalar profiles came
/// in at 100.6%), then set its own backstop at 15% — comfortably above
/// the artifact, and still an order of magnitude below the blowouts this
/// exists to catch, which run to several hundred percent. Tightening
/// toward zero does not buy safety; it starts refusing legitimate
/// saturated moves, which is how this constant was first set too low
/// here.
pub const ACCEL_TOLERANCE: f64 = 0.15;

/// The worst commanded acceleration in a sample stream, as a fraction of
/// the joint's limit, with the joint and sample it lands on.
///
/// `velocities` is the stream's commanded velocity column in order, one
/// row per tick of `dt`. The first row is not differenced against
/// anything before it: a stream begins where the arm already is.
///
/// `None` for a stream too short to difference, or one whose limits are
/// all non-positive. Planners use this to price a path before emitting
/// it; [`check_commanded_accel`] is the refusal built on it.
pub fn worst_commanded_accel(
    velocities: impl IntoIterator<Item = [f64; NUM_JOINTS]>,
    limits: &[f64; NUM_JOINTS],
    dt: f64,
) -> Option<WorstAccel> {
    if !dt.is_finite() || dt <= 0.0 {
        return None;
    }
    let mut prev: Option<[f64; NUM_JOINTS]> = None;
    let mut worst: Option<WorstAccel> = None;
    for (k, qd) in velocities.into_iter().enumerate() {
        if let Some(before) = prev {
            for j in 0..NUM_JOINTS {
                let limit = limits[j];
                if !limit.is_finite() || limit <= 0.0 {
                    continue;
                }
                let commanded = (qd[j] - before[j]) / dt;
                let ratio = commanded.abs() / limit;
                if worst.as_ref().is_none_or(|w| ratio > w.ratio) {
                    worst = Some(WorstAccel {
                        ratio,
                        joint: j,
                        sample: k,
                        commanded,
                        limit,
                    });
                }
            }
        }
        prev = Some(qd);
    }
    worst
}

/// The steepest commanded velocity step a stream contains.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorstAccel {
    /// `|commanded| / limit`; at most 1.0 for a stream inside its limits.
    pub ratio: f64,
    /// Joint index (0-based).
    pub joint: usize,
    /// Index of the sample the step lands on.
    pub sample: usize,
    /// Commanded acceleration across that tick \[rad/s^2\].
    pub commanded: f64,
    /// That joint's acceleration limit \[rad/s^2\].
    pub limit: f64,
}

/// Refuse a sample stream whose commanded velocity steps imply an
/// acceleration past the limits, reporting the single worst offender.
pub fn check_commanded_accel(
    velocities: impl IntoIterator<Item = [f64; NUM_JOINTS]>,
    limits: &[f64; NUM_JOINTS],
    dt: f64,
    tolerance: f64,
) -> Result<(), MotionError> {
    if !dt.is_finite() || dt <= 0.0 {
        return Err(MotionError::InvalidInput {
            what: "dt",
            reason: format!("must be positive, got {dt}"),
        });
    }
    match worst_commanded_accel(velocities, limits, dt) {
        Some(w) if w.ratio > 1.0 + tolerance => Err(MotionError::CommandedAccelExceeded {
            joint: w.joint,
            sample: w.sample,
            commanded: w.commanded,
            limit: w.limit,
        }),
        _ => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.004;

    fn rows(j0: &[f64]) -> Vec<[f64; NUM_JOINTS]> {
        j0.iter()
            .map(|v| {
                let mut r = [0.0; NUM_JOINTS];
                r[0] = *v;
                r
            })
            .collect()
    }

    /// The gate lets a profile that rides its limit exactly through —
    /// otherwise the tolerance does nothing and every hard move is
    /// refused — and refuses a stream that steps hard at one interior
    /// sample, naming the worst step (the first of equals), each joint
    /// judged against its own limit.
    #[test]
    fn the_gate_passes_the_limit_and_names_the_worst_step_past_it() {
        let limits = [4.0; NUM_JOINTS];
        let ramp: Vec<f64> = (0..50).map(|k| k as f64 * 4.0 * DT).collect();
        assert!(check_commanded_accel(rows(&ramp), &limits, DT, ACCEL_TOLERANCE).is_ok());

        // Fine on average and at its endpoints, one spike in between.
        let mut spiked = ramp.clone();
        spiked[30] += 0.5;
        let err = check_commanded_accel(rows(&spiked), &limits, DT, ACCEL_TOLERANCE)
            .expect_err("the spike must be refused");
        let MotionError::CommandedAccelExceeded {
            joint,
            sample,
            commanded,
            limit,
        } = err
        else {
            panic!("wrong error: {err}");
        };
        assert_eq!((joint, sample), (0, 30));
        assert!(
            commanded > 100.0,
            "should name the value it saw: {commanded}"
        );
        assert!((limit - 4.0).abs() < 1e-12);

        // The worst offender, not the first one walked past: the number
        // an operator needs is how far out the stream got. Of equal
        // steps, the first.
        let worst = |steps: &[(usize, f64)]| {
            let mut v = vec![0.0; 40];
            for &(k, x) in steps {
                v[k] = x;
            }
            match check_commanded_accel(rows(&v), &limits, DT, ACCEL_TOLERANCE) {
                Err(MotionError::CommandedAccelExceeded { sample, .. }) => sample,
                other => panic!("both steps are past the limit: {other:?}"),
            }
        };
        assert_eq!(worst(&[(10, 0.1), (11, 0.1), (20, 0.4), (21, 0.4)]), 20);
        assert_eq!(worst(&[(20, 0.4), (30, -0.4)]), 20, "a tie names the first");

        // A slow joint's step is measured against its own budget.
        let mut limits = [4.0; NUM_JOINTS];
        limits[1] = 0.5;
        let step = 0.01; // 2.5 rad/s^2 over one tick
        let mut b = [0.0; NUM_JOINTS];
        b[0] = step;
        assert!(
            check_commanded_accel(vec![[0.0; NUM_JOINTS], b], &limits, DT, ACCEL_TOLERANCE).is_ok()
        );
        let mut b = [0.0; NUM_JOINTS];
        b[1] = step;
        let err = check_commanded_accel(vec![[0.0; NUM_JOINTS], b], &limits, DT, ACCEL_TOLERANCE)
            .expect_err("joint 1 cannot take that step");
        let MotionError::CommandedAccelExceeded { joint, .. } = err else {
            panic!("wrong error: {err}");
        };
        assert_eq!(joint, 1);
    }
}
