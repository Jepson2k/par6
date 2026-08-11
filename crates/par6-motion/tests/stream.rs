//! Streaming executor tests against the real PAR6 stream limits:
//! convergence to a target moved mid-flight, no overshoot, limit
//! adherence, limit rescaling, and lifecycle errors.

mod common;

use common::{assert_within_limits, max_err, par6_config};
use par6_config::LimitMode;
use par6_motion::{MotionError, MotionLimits, StreamingExecutor, NUM_JOINTS};

const HOME: [f64; NUM_JOINTS] = [0.0, -1.5, 3.0, 0.0, 0.0, 3.1];
const DELTA: [f64; NUM_JOINTS] = [0.3, 0.2, -0.3, 0.5, 0.4, -0.5];

fn stream_setup() -> (StreamingExecutor, MotionLimits, f64) {
    let cfg = par6_config();
    let limits = MotionLimits::from_config(&cfg, LimitMode::Stream).unwrap();
    let dt = cfg.robot.tick_dt_s;
    (StreamingExecutor::new(dt, &limits).unwrap(), limits, dt)
}

#[test]
fn converges_to_moved_target_without_overshoot() {
    let (mut exec, limits, dt) = stream_setup();
    exec.activate(&HOME);

    // Idle after activation: finished at the sync pose.
    let idle = exec.step().unwrap();
    assert!(idle.finished);
    assert_eq!(idle.q, HOME);
    assert!(idle.qd.iter().all(|&v| v == 0.0));

    let t1: [f64; NUM_JOINTS] = std::array::from_fn(|j| HOME[j] + DELTA[j]);
    let t2: [f64; NUM_JOINTS] = std::array::from_fn(|j| HOME[j] + 2.0 * DELTA[j]);

    let mut qs = vec![HOME];
    exec.set_target(&t1).unwrap();
    for _ in 0..50 {
        let s = exec.step().unwrap();
        qs.push(s.q);
    }
    // Move the target mid-flight, further along the same direction.
    exec.set_target(&t2).unwrap();
    let mut finished = false;
    for _ in 0..20_000 {
        let s = exec.step().unwrap();
        qs.push(s.q);
        if s.finished {
            finished = true;
            break;
        }
    }
    assert!(finished, "must converge to the moved target");
    assert!(
        max_err(qs.last().unwrap(), &t2) < 1e-6,
        "final pose off target by {}",
        max_err(qs.last().unwrap(), &t2)
    );

    // No overshoot past the final target on any joint.
    for q in &qs {
        for j in 0..NUM_JOINTS {
            if DELTA[j] > 0.0 {
                assert!(
                    q[j] <= t2[j] + 1e-9,
                    "joint {j} overshot: {} > {}",
                    q[j],
                    t2[j]
                );
            } else {
                assert!(
                    q[j] >= t2[j] - 1e-9,
                    "joint {j} overshot: {} < {}",
                    q[j],
                    t2[j]
                );
            }
        }
    }

    // Retargeting kept the whole stream inside the stream-mode limits.
    assert_within_limits(
        &qs,
        dt,
        &limits.velocity,
        &limits.acceleration,
        Some(&limits.jerk),
        "stream",
    );
}

#[test]
fn set_limits_rescales_the_tracker() {
    let (mut exec, limits, dt) = stream_setup();
    let mut reduced = limits;
    for v in reduced.velocity.iter_mut() {
        *v *= 0.3;
    }
    exec.set_limits(&reduced).unwrap();
    exec.activate(&HOME);
    let target: [f64; NUM_JOINTS] = std::array::from_fn(|j| HOME[j] + DELTA[j]);
    exec.set_target(&target).unwrap();
    let mut qs = vec![HOME];
    for _ in 0..20_000 {
        let s = exec.step().unwrap();
        qs.push(s.q);
        if s.finished {
            break;
        }
    }
    assert!(max_err(qs.last().unwrap(), &target) < 1e-6);
    assert_within_limits(
        &qs,
        dt,
        &reduced.velocity,
        &limits.acceleration,
        Some(&limits.jerk),
        "stream reduced",
    );

    // Jerk-limited OTG refuses a limit set with no finite jerk.
    let mut no_jerk = limits;
    no_jerk.jerk[2] = f64::INFINITY;
    assert!(matches!(
        exec.set_limits(&no_jerk),
        Err(MotionError::MissingJerkLimit { joint: 2 })
    ));
}

#[test]
fn lifecycle_and_input_validation() {
    let (mut exec, _, _) = stream_setup();

    // Not activated: servo calls are refused.
    assert!(matches!(
        exec.set_target(&HOME),
        Err(MotionError::InvalidInput { .. })
    ));
    assert!(matches!(exec.step(), Err(MotionError::InvalidInput { .. })));

    exec.activate(&HOME);
    let mut nan = HOME;
    nan[1] = f64::NAN;
    assert!(matches!(
        exec.set_target(&nan),
        Err(MotionError::InvalidInput {
            what: "q_target",
            ..
        })
    ));
    let mut inf = HOME;
    inf[5] = f64::NEG_INFINITY;
    assert!(exec.set_target(&inf).is_err());
    // The rejected targets left the tracker at the sync pose.
    let s = exec.step().unwrap();
    assert_eq!(s.q, HOME);
}
