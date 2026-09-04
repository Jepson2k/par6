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

    /// A profile that saturates its acceleration limit exactly is a
    /// correct profile, and the gate has to let it through — otherwise
    /// the tolerance is doing nothing and every hard move is refused.
    #[test]
    fn a_stream_that_rides_the_limit_is_allowed_through() {
        let limits = [4.0; NUM_JOINTS];
        let ramp: Vec<f64> = (0..50).map(|k| k as f64 * 4.0 * DT).collect();
        assert!(check_commanded_accel(rows(&ramp), &limits, DT, ACCEL_TOLERANCE).is_ok());
    }

    /// The failure the gate exists for: a stream that is fine on average
    /// and fine at its endpoints, but steps hard at one interior sample.
    #[test]
    fn one_bulging_sample_refuses_the_whole_stream() {
        let limits = [4.0; NUM_JOINTS];
        let mut ramp: Vec<f64> = (0..50).map(|k| k as f64 * 4.0 * DT).collect();
        ramp[30] += 0.5; // a spike between two otherwise legal samples
        let err = check_commanded_accel(rows(&ramp), &limits, DT, ACCEL_TOLERANCE)
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
    }

    /// It reports the worst offender, not the first one it walks past —
    /// the number an operator needs is how far out the stream got.
    #[test]
    fn the_reported_violation_is_the_worst_one() {
        let limits = [4.0; NUM_JOINTS];
        let mut ramp: Vec<f64> = vec![0.0; 40];
        ramp[10] = 0.1;
        ramp[11] = 0.1;
        ramp[20] = 0.4;
        ramp[21] = 0.4;
        let err = check_commanded_accel(rows(&ramp), &limits, DT, ACCEL_TOLERANCE)
            .expect_err("both steps are past the limit");
        let MotionError::CommandedAccelExceeded { sample, .. } = err else {
            panic!("wrong error: {err}");
        };
        assert_eq!(sample, 20, "the 0.4 step is four times the 0.1 one");
    }

    /// Each joint is judged against its own limit, so a slow joint's
    /// legal step is not measured against a fast joint's budget.
    #[test]
    fn joints_are_judged_against_their_own_limits() {
        let mut limits = [4.0; NUM_JOINTS];
        limits[1] = 0.5;
        let step = 0.01; // 2.5 rad/s^2 over one tick
        let mut a = [0.0; NUM_JOINTS];
        let mut b = [0.0; NUM_JOINTS];
        b[0] = step;
        assert!(check_commanded_accel(vec![a, b], &limits, DT, ACCEL_TOLERANCE).is_ok());
        a[1] = 0.0;
        b = [0.0; NUM_JOINTS];
        b[1] = step;
        let err = check_commanded_accel(vec![a, b], &limits, DT, ACCEL_TOLERANCE)
            .expect_err("joint 1 cannot take that step");
        let MotionError::CommandedAccelExceeded { joint, .. } = err else {
            panic!("wrong error: {err}");
        };
        assert_eq!(joint, 1);
    }
}
