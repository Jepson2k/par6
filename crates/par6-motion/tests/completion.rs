//! Completion policy state-machine tests: settled convergence, settle
//! timeout (500 ticks at the PAR6 tick rate), strict timeout escalation,
//! commanded immediacy, and blend bypass.

mod common;

use common::par6_config;
use par6_motion::{
    CompletionEvent, CompletionMonitor, CompletionPolicy, MotionError, SettleParams, NUM_JOINTS,
};

const TARGET: [f64; NUM_JOINTS] = [1.0, -0.5, 2.5, 1.0, 0.8, 1.0];

fn offset(base: &[f64; NUM_JOINTS], joint: usize, by: f64) -> [f64; NUM_JOINTS] {
    let mut q = *base;
    q[joint] += by;
    q
}

#[test]
fn settled_policy_converges_or_times_out() {
    let dt = par6_config().robot.tick_dt_s;
    let mut mon =
        CompletionMonitor::new(CompletionPolicy::Settled, SettleParams::default(), dt).unwrap();

    // Converging: pending while any joint is off by more than 0.01 rad,
    // complete once all are within.
    assert_eq!(mon.arm(false), CompletionEvent::Pending);
    assert!(mon.is_armed());
    let mut q_meas = offset(&TARGET, 2, 0.1);
    for _ in 0..10 {
        assert_eq!(
            mon.tick(&q_meas, &TARGET).unwrap(),
            CompletionEvent::Pending
        );
    }
    q_meas = offset(&TARGET, 2, 0.009);
    assert_eq!(
        mon.tick(&q_meas, &TARGET).unwrap(),
        CompletionEvent::Complete
    );
    assert!(!mon.is_armed());

    // Stuck: completes anyway at the timeout — 2.0 s = 500 ticks at the
    // PAR6 4 ms tick (round(s/dt), never a hardcoded tick count).
    let expected_ticks = (2.0_f64 / dt).round() as usize;
    assert_eq!(mon.arm(false), CompletionEvent::Pending);
    let stuck = offset(&TARGET, 4, 0.05);
    for k in 1..expected_ticks {
        assert_eq!(
            mon.tick(&stuck, &TARGET).unwrap(),
            CompletionEvent::Pending,
            "tick {k} must still be settling"
        );
    }
    assert_eq!(
        mon.tick(&stuck, &TARGET).unwrap(),
        CompletionEvent::Complete
    );
}

#[test]
fn strict_policy_errors_on_timeout_and_blend_bypasses() {
    let dt = par6_config().robot.tick_dt_s;
    let mut mon =
        CompletionMonitor::new(CompletionPolicy::Strict, SettleParams::default(), dt).unwrap();

    // blend_continues bypasses settling entirely, even under strict.
    assert_eq!(mon.arm(true), CompletionEvent::Complete);
    assert!(!mon.is_armed());

    // A stuck strict settle errors at the timeout, naming the worst joint.
    assert_eq!(mon.arm(false), CompletionEvent::Pending);
    let expected_ticks = (2.0_f64 / dt).round() as usize;
    let stuck = offset(&TARGET, 3, -0.2);
    for _ in 1..expected_ticks {
        assert_eq!(mon.tick(&stuck, &TARGET).unwrap(), CompletionEvent::Pending);
    }
    match mon.tick(&stuck, &TARGET) {
        Err(MotionError::SettleTimeout {
            worst_joint,
            error_rad,
            ..
        }) => {
            assert_eq!(worst_joint, 3);
            assert!((error_rad - 0.2).abs() < 1e-12);
        }
        other => panic!("strict timeout must error, got {other:?}"),
    }
    assert!(!mon.is_armed());

    // Strict completes normally when tracking converges in time.
    assert_eq!(mon.arm(false), CompletionEvent::Pending);
    assert_eq!(
        mon.tick(&offset(&TARGET, 0, 0.005), &TARGET).unwrap(),
        CompletionEvent::Complete
    );
}

#[test]
fn commanded_policy_and_validation() {
    let dt = par6_config().robot.tick_dt_s;
    let mut mon =
        CompletionMonitor::new(CompletionPolicy::Commanded, SettleParams::default(), dt).unwrap();
    // Commanded completes at the boundary regardless of tracking error.
    assert_eq!(mon.arm(false), CompletionEvent::Complete);
    assert!(!mon.is_armed());
    // Unarmed ticks report Pending and never error.
    let far = offset(&TARGET, 0, 1.0);
    assert_eq!(mon.tick(&far, &TARGET).unwrap(), CompletionEvent::Pending);

    for (tol, timeout, dt) in [
        (0.0, 2.0, 0.004),
        (-0.01, 2.0, 0.004),
        (f64::NAN, 2.0, 0.004),
        (0.01, 0.0, 0.004),
        (0.01, f64::INFINITY, 0.004),
        (0.01, 2.0, 0.0),
        (0.01, 2.0, f64::NAN),
    ] {
        assert!(
            CompletionMonitor::new(
                CompletionPolicy::Settled,
                SettleParams {
                    tolerance_rad: tol,
                    timeout_s: timeout,
                },
                dt,
            )
            .is_err(),
            "tol {tol} timeout {timeout} dt {dt} must be rejected"
        );
    }
}
