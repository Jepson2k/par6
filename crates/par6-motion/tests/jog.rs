//! Jog engine tests against the real PAR6 config: lookahead stopping
//! before the soft limit at max speed, direction-block latching and
//! clearing, ramp shape adherence, hard clamp, and config floors.

mod common;

use common::par6_config;
use par6_config::JogProfile;
use par6_motion::{JogDirection, JogEngine, MotionError, NUM_JOINTS};

const HOME: [f64; NUM_JOINTS] = [0.0, -1.5, 3.0, 0.0, 0.0, 3.1];

/// Drive `n` ticks with ideal tracking (measured = previous commanded
/// target), asserting the per-tick velocity slew stays within a·dt.
/// Returns the last measured pose.
fn run_tracked(
    engine: &mut JogEngine,
    q_meas: &mut [f64; NUM_JOINTS],
    n: usize,
    max_dv: &[f64; NUM_JOINTS],
) {
    let mut prev_qd: [f64; NUM_JOINTS] = std::array::from_fn(|j| engine.velocity(j));
    for _ in 0..n {
        let out = engine.tick(q_meas);
        for j in 0..NUM_JOINTS {
            let dv = (out.qd[j] - prev_qd[j]).abs();
            assert!(
                dv <= max_dv[j] * (1.0 + 1e-9) + 1e-12,
                "joint {j} velocity slew {dv} exceeds a*dt {}",
                max_dv[j]
            );
        }
        prev_qd = out.qd;
        *q_meas = out.q;
    }
}

fn jog_accels(cfg: &par6_config::RobotConfig) -> [f64; NUM_JOINTS] {
    let accel_time = cfg.jog.accel_time_s.max(par6_motion::MIN_ACCEL_TIME_S);
    std::array::from_fn(|j| {
        let l = &cfg.joints[j].limits;
        (l.velocity_rad_s / accel_time).min(l.acceleration_rad_s2)
    })
}

#[test]
fn lookahead_blocks_before_soft_limit_and_latches() {
    let cfg = par6_config();
    let dt = cfg.robot.tick_dt_s;
    let soft_max = cfg.joints[0].limits.soft_min_rad.abs();
    let a = jog_accels(&cfg);
    let max_dv: [f64; NUM_JOINTS] = std::array::from_fn(|j| a[j] * dt);

    let mut engine = JogEngine::new(&cfg).unwrap();
    engine.activate(&HOME);
    let mut q = HOME;

    // Jog J0 positive at max speed until the lookahead stops it.
    engine.command(0, JogDirection::Positive, 1.0).unwrap();
    run_tracked(&mut engine, &mut q, 3000, &max_dv);
    assert!(
        engine.velocity(0) == 0.0,
        "jog must have come to rest, v = {}",
        engine.velocity(0)
    );
    assert!(
        q[0] < soft_max,
        "target {} must stop short of the soft limit {}",
        q[0],
        soft_max
    );
    assert!(q[0] > 0.5, "jog must actually have moved, got {}", q[0]);
    assert_eq!(engine.blocked_direction(0), Some(JogDirection::Positive));

    // The block holds while the button stays pressed...
    let stopped_at = q[0];
    run_tracked(&mut engine, &mut q, 200, &max_dv);
    assert_eq!(q[0], stopped_at, "blocked jog must not move");

    // ...and survives release + re-press in the same direction.
    engine.release();
    run_tracked(&mut engine, &mut q, 50, &max_dv);
    engine.command(0, JogDirection::Positive, 1.0).unwrap();
    run_tracked(&mut engine, &mut q, 200, &max_dv);
    assert_eq!(q[0], stopped_at, "latch must survive button release");
    assert_eq!(engine.blocked_direction(0), Some(JogDirection::Positive));

    // The opposite direction clears the latch and moves.
    engine.command(0, JogDirection::Negative, 0.5).unwrap();
    assert_eq!(engine.blocked_direction(0), None);
    run_tracked(&mut engine, &mut q, 500, &max_dv);
    assert!(q[0] < stopped_at - 0.1, "opposite jog must move away");

    // After moving away, the positive direction works again until the
    // lookahead re-latches.
    engine.command(0, JogDirection::Positive, 1.0).unwrap();
    let before = q[0];
    run_tracked(&mut engine, &mut q, 3000, &max_dv);
    assert!(q[0] > before, "cleared direction must jog again");
    assert!(q[0] < soft_max);
    assert_eq!(engine.blocked_direction(0), Some(JogDirection::Positive));

    // Switching joints clears the latch.
    engine.command(3, JogDirection::Positive, 0.2).unwrap();
    assert_eq!(engine.blocked_direction(0), None);
    run_tracked(&mut engine, &mut q, 100, &max_dv);
    assert!(q[3] > 0.0, "switched joint must jog");

    // Negative side blocks too.
    engine.activate(&HOME);
    let mut q = HOME;
    engine.command(4, JogDirection::Negative, 1.0).unwrap();
    run_tracked(&mut engine, &mut q, 3000, &max_dv);
    assert!(q[4] > cfg.joints[4].limits.soft_min_rad);
    assert!(q[4] < -0.5, "negative jog must have moved");
    assert_eq!(engine.blocked_direction(4), Some(JogDirection::Negative));
}

#[test]
fn trapezoid_ramp_and_hard_clamp() {
    let cfg = par6_config();
    let dt = cfg.robot.tick_dt_s;
    let a = jog_accels(&cfg);
    let max_dv: [f64; NUM_JOINTS] = std::array::from_fn(|j| a[j] * dt);

    let mut engine = JogEngine::new(&cfg).unwrap();
    engine.activate(&HOME);
    engine.set_profile(JogProfile::Trapezoid);
    let mut q = HOME;
    engine.command(0, JogDirection::Positive, 1.0).unwrap();

    // Trapezoid ramp: the very first tick slews at exactly a·dt.
    let out = engine.tick(&q);
    assert!(
        (out.qd[0] - a[0] * dt).abs() < 1e-12,
        "first trapezoid tick must ramp at a*dt, got {}",
        out.qd[0]
    );
    q = out.q;

    // Runs to the block and stops short of the limit.
    run_tracked(&mut engine, &mut q, 3000, &max_dv);
    assert_eq!(engine.blocked_direction(0), Some(JogDirection::Positive));
    assert!(q[0] < cfg.joints[0].limits.soft_max_rad);

    // Hard clamp: measured position past the soft limit while moving
    // outward zeroes the command.
    let mut engine = JogEngine::new(&cfg).unwrap();
    engine.activate(&HOME);
    let mut q = HOME;
    engine.command(0, JogDirection::Positive, 1.0).unwrap();
    run_tracked(&mut engine, &mut q, 50, &max_dv);
    assert!(engine.velocity(0) > 0.0);
    let mut overrun = q;
    overrun[0] = cfg.joints[0].limits.soft_max_rad + 0.01;
    let out = engine.tick(&overrun);
    assert_eq!(out.qd[0], 0.0, "hard clamp must zero the jog velocity");
    assert!(out.q[0] <= cfg.joints[0].limits.soft_max_rad);
}

#[test]
fn config_floors_and_input_validation() {
    let cfg = par6_config();
    let dt = cfg.robot.tick_dt_s;

    // accel_time floor 0.05 s: requesting 0.001 s ramps at the floor rate
    // (capped by the jog acceleration limit).
    let mut engine = JogEngine::new(&cfg).unwrap();
    engine.activate(&HOME);
    engine.set_profile(JogProfile::Trapezoid);
    engine.set_accel_time_s(0.001).unwrap();
    engine.command(0, JogDirection::Positive, 1.0).unwrap();
    let l = &cfg.joints[0].limits;
    let a_floor = (l.velocity_rad_s / par6_motion::MIN_ACCEL_TIME_S).min(l.acceleration_rad_s2);
    let out = engine.tick(&HOME);
    assert!(
        (out.qd[0] - a_floor * dt).abs() < 1e-12,
        "floored ramp must run at {} rad/s^2, got dv {}",
        a_floor,
        out.qd[0] / dt
    );

    // jerk_factor floor 0.5: the first s-curve tick accrues jerk*dt^2 with
    // jerk = a * 0.5.
    let mut engine = JogEngine::new(&cfg).unwrap();
    engine.activate(&HOME);
    engine.set_profile(JogProfile::Scurve);
    engine.set_jerk_factor(0.01).unwrap();
    engine.command(0, JogDirection::Positive, 1.0).unwrap();
    let a = jog_accels(&cfg)[0];
    let expected_dv = (a * par6_motion::MIN_JERK_FACTOR) * dt * dt;
    let out = engine.tick(&HOME);
    assert!(
        (out.qd[0] - expected_dv).abs() < 1e-12,
        "floored jerk factor must give first dv {expected_dv}, got {}",
        out.qd[0]
    );

    // Requirement-derived rejections: bad joint index, zero/negative/NaN/
    // over-unity speed.
    let mut engine = JogEngine::new(&cfg).unwrap();
    assert!(matches!(
        engine.command(NUM_JOINTS, JogDirection::Positive, 0.5),
        Err(MotionError::InvalidInput { what: "joint", .. })
    ));
    for bad in [0.0, -0.2, 1.5, f64::NAN, f64::INFINITY] {
        assert!(matches!(
            engine.command(0, JogDirection::Positive, bad),
            Err(MotionError::InvalidInput {
                what: "speed_pct",
                ..
            })
        ));
    }
    for bad in [0.0, -1.0, f64::NAN] {
        assert!(engine.set_accel_time_s(bad).is_err());
        assert!(engine.set_jerk_factor(bad).is_err());
    }
}
