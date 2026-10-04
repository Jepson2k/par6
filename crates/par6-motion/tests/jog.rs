//! The jog engine's own contracts against the real PAR6 config: the hard
//! clamp on a measured pose past a soft limit, the configuration floors,
//! and input validation. How a jog stops short of a limit on a moving
//! arm is tested through the RT core onto the simulated plant
//! (`par6-rt/tests/jog.rs`).

mod common;

use common::par6_config;
use par6_config::JogProfile;
use par6_motion::{JogEngine, MotionError, NUM_JOINTS};

const HOME: [f64; NUM_JOINTS] = [0.0, -1.5, 3.0, 0.0, 0.0, 3.1];

/// A speed array driving `joint` alone at `signed_pct`.
fn one(joint: usize, signed_pct: f64) -> [f64; NUM_JOINTS] {
    let mut speeds = [0.0; NUM_JOINTS];
    speeds[joint] = signed_pct;
    speeds
}

fn jog_accels(cfg: &par6_config::RobotConfig) -> [f64; NUM_JOINTS] {
    let accel_time = cfg.jog.accel_time_s.max(par6_motion::MIN_ACCEL_TIME_S);
    std::array::from_fn(|j| {
        let l = &cfg.joints[j].limits;
        (l.velocity_rad_s / accel_time).min(l.acceleration_rad_s2)
    })
}

/// A joint still moving outward when its MEASURED pose is already past
/// the soft limit — tracking lag carried it there — gets no further
/// outward velocity and a target clamped to the limit, and an inward jog
/// brings the target back in.
#[test]
fn a_measured_pose_past_the_soft_limit_stops_the_outward_jog() {
    let cfg = par6_config();
    let soft_max = cfg.joints[0].limits.soft_max_rad;
    let mut engine = JogEngine::new(&cfg).unwrap();
    let mut q = HOME;
    q[0] = soft_max - 1.0;
    engine.activate(&q);
    engine.command(&one(0, 1.0)).unwrap();
    for _ in 0..50 {
        engine.tick(&q);
    }
    assert!(engine.velocity(0) > 0.0, "the jog is moving outward");

    let mut past = q;
    past[0] = soft_max + 0.05;
    for _ in 0..20 {
        let out = engine.tick(&past);
        assert_eq!(out.qd[0], 0.0, "no outward velocity past the soft limit");
        assert!(out.q[0] <= soft_max, "the target is clamped to the limit");
    }

    engine.command(&one(0, -0.5)).unwrap();
    let mut out = engine.tick(&past);
    for _ in 0..100 {
        out = engine.tick(&past);
    }
    assert!(out.qd[0] < 0.0, "an inward jog moves the target back in");
    assert!(out.q[0] < soft_max, "and below the limit");
}

#[test]
fn config_floors_and_input_validation() {
    let cfg = par6_config();
    let dt = cfg.robot.tick_dt_s;

    // accel_time floor 0.05 s: requesting 0.001 s ramps at the floor rate
    // (capped by the jog acceleration limit).
    // An acceleration ceiling the floor sits under, so it is the floor
    // that binds: a 1 ms ramp time is held to MIN_ACCEL_TIME_S.
    let mut roomy = cfg.clone();
    roomy.joints[0].limits.acceleration_rad_s2 = 1.0e6;
    let mut engine = JogEngine::new(&roomy).unwrap();
    engine.activate(&HOME);
    engine.set_profile(JogProfile::Trapezoid);
    engine.set_accel_time_s(0.001).unwrap();
    engine.command(&one(0, 1.0)).unwrap();
    let a_floor = roomy.joints[0].limits.velocity_rad_s / par6_motion::MIN_ACCEL_TIME_S;
    let out = engine.tick(&HOME);
    assert!(
        (out.qd[0] - a_floor * dt).abs() < 1e-9,
        "floored ramp must run at {a_floor} rad/s^2, got {}",
        out.qd[0] / dt
    );

    // jerk_factor floor 0.5: the first s-curve tick accrues jerk*dt^2 with
    // jerk = a * 0.5.
    let mut engine = JogEngine::new(&cfg).unwrap();
    engine.activate(&HOME);
    engine.set_profile(JogProfile::Scurve);
    engine.set_jerk_factor(0.01).unwrap();
    engine.command(&one(0, 1.0)).unwrap();
    let a = jog_accels(&cfg)[0];
    let expected_dv = (a * par6_motion::MIN_JERK_FACTOR) * dt * dt;
    let out = engine.tick(&HOME);
    assert!(
        (out.qd[0] - expected_dv).abs() < 1e-12,
        "floored jerk factor must give first dv {expected_dv}, got {}",
        out.qd[0]
    );

    // Requirement-derived rejections: NaN, infinite and over-unity
    // speeds. Zero is now a legitimate entry — it is how a multi-joint
    // command says "leave this axis alone".
    let mut engine = JogEngine::new(&cfg).unwrap();
    for bad in [-1.2, 1.5, f64::NAN, f64::INFINITY] {
        assert!(matches!(
            engine.command(&one(0, bad)),
            Err(MotionError::InvalidInput { what: "speeds", .. })
        ));
    }
    assert!(
        engine.command(&[0.0; NUM_JOINTS]).is_ok(),
        "an all-zero command is a release, not an error"
    );
    for bad in [0.0, -1.0, f64::NAN] {
        assert!(engine.set_accel_time_s(bad).is_err());
        assert!(engine.set_jerk_factor(bad).is_err());
    }
}
