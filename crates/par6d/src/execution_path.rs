//! Planner-side validation of the positions emitted by fractional playback.

use par6_motion::{MotionError, MotionLimits};
use par6_rt::exec::SampleInterval;
use par6_rt::{Sample, MAX_JOINTS};

use crate::planner::COLLISION_STEP_RAD;

const MAX_POINTS: usize = 1_000_000;

pub(crate) fn positions(
    samples: &[Sample],
    dt: f64,
    limits: &MotionLimits,
) -> Result<Vec<[f64; MAX_JOINTS]>, MotionError> {
    let Some(first) = samples.first() else {
        return Ok(Vec::new());
    };
    let mut left = first.start.map_or(*first, |start| Sample {
        q: start.q,
        ..Sample::default()
    });
    if !dt.is_finite() || dt <= 0.0 || !left.q.iter().chain(&left.qd).all(|v| v.is_finite()) {
        return Err(MotionError::InvalidInput {
            what: "execution samples",
            reason: "initial state must be finite and tick duration positive".into(),
        });
    }
    let mut bounds = *limits;
    // A measured start outside the execution window may move back in,
    // but interpolation must not take it farther outside that window.
    for i in 0..MAX_JOINTS {
        bounds.soft_min[i] = bounds.soft_min[i].min(left.q[i]) - 1e-10;
        bounds.soft_max[i] = bounds.soft_max[i].max(left.q[i]) + 1e-10;
    }
    let mut points = vec![left.q];
    let mut travel = 0.0;
    for right in samples.iter().skip(usize::from(first.start.is_none())) {
        if !right.q.iter().chain(&right.qd).all(|v| v.is_finite()) {
            return Err(MotionError::InvalidInput {
                what: "execution samples",
                reason: "position and velocity must be finite".into(),
            });
        }
        let interval = SampleInterval::new(&left, right, dt);
        let (phases, count) = interval.position_extrema();
        let bound = interval.travel_bound();
        if !bound.is_finite() || bound > MAX_POINTS as f64 * COLLISION_STEP_RAD {
            return Err(too_many_points());
        }
        for phases in phases[..count].windows(2) {
            let mut phase = phases[0];
            let end = phases[1];
            let end_q = interval.sample(end).0;
            bounds.require_inside_soft(&end_q)?;
            while bound > 0.0 && travel + bound * (end - phase) >= COLLISION_STEP_RAD {
                phase = (phase + (COLLISION_STEP_RAD - travel) / bound).min(end);
                push(&mut points, interval.sample(phase).0)?;
                travel = 0.0;
            }
            travel += bound * (end - phase);
            if end < 1.0 {
                push(&mut points, end_q)?;
                travel = 0.0;
            }
        }
        left = *right;
    }
    push(&mut points, left.q)?;
    Ok(points)
}

fn push(points: &mut Vec<[f64; MAX_JOINTS]>, q: [f64; MAX_JOINTS]) -> Result<(), MotionError> {
    if points.len() >= MAX_POINTS {
        return Err(too_many_points());
    }
    points.push(q);
    Ok(())
}

fn too_many_points() -> MotionError {
    MotionError::InvalidInput {
        what: "execution samples",
        reason: "interpolated path exceeds the collision-check point limit".into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn validation_covers_the_fractional_path_and_its_turning_points() {
        let mut left = Sample::default();
        left.qd[0] = 1.0;
        let mut right = Sample::default();
        right.qd[0] = -1.0;
        let samples = [left, right];
        let mut limits = MotionLimits {
            velocity: [2.0; MAX_JOINTS],
            acceleration: [4.0; MAX_JOINTS],
            jerk: [10.0; MAX_JOINTS],
            soft_min: [-1.0; MAX_JOINTS],
            soft_max: [1.0; MAX_JOINTS],
        };
        let points = positions(&samples, 1.0, &limits).unwrap();
        // Equal endpoints and opposite velocities describe q(t)=t-t².
        // The arm reaches 0.25 rad although both stored positions are zero.
        assert!(points.iter().any(|q| (q[0] - 0.25).abs() < 1e-12));
        assert_eq!(points.first().unwrap()[0], 0.0);
        assert_eq!(points.last().unwrap()[0], 0.0);
        assert!(points.windows(2).all(|pair| {
            (pair[1][0] - pair[0][0]).abs() <= super::super::planner::COLLISION_STEP_RAD + 1e-12
        }));
        limits.soft_max[0] = 0.24;
        assert!(matches!(
            positions(&samples, 1.0, &limits),
            Err(MotionError::TargetOutsideSoftLimits { joint: 0, .. })
        ));

        right.qd[0] = 1.0;
        let points = positions(&[left, right], 1.0, &limits).unwrap();
        let turning_position = 3.0_f64.sqrt() / 18.0;
        for sign in [-1.0, 1.0] {
            assert!(points
                .iter()
                .any(|q| (q[0] - sign * turning_position).abs() < 1e-12));
        }

        // Sampling density depends on travel, not duration or tick count.
        let slow: Vec<_> = (0..=1000)
            .map(|k| {
                let mut sample = Sample::default();
                sample.q[0] = k as f64 / 1000.0;
                sample.qd[0] = 1.0;
                sample
            })
            .collect();
        limits.soft_max[0] = 1.0;
        let points = positions(&slow, 0.001, &limits).unwrap();
        assert!(points.len() <= 53, "a slow move was oversampled");
        assert_eq!(points.last().unwrap()[0], 1.0);
        assert!(points
            .windows(2)
            .all(|pair| { (pair[1][0] - pair[0][0]).abs() <= COLLISION_STEP_RAD + 1e-12 }));
    }
}
