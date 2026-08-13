//! Jog engine tests against the real PAR6 config: lookahead stopping
//! before the soft limit at max speed, direction-block latching and
//! clearing, ramp shape adherence, hard clamp, and config floors.
//!
//! The engine is fed through a first-order tracking-lag plant, not a
//! perfect `q_meas = out.q` echo: the RT loop hands the engine the
//! MEASURED pose while the lookahead runs on the integrated target, and
//! the two disagreeing under real tracking lag is exactly the seam the
//! measured-pose hard clamp exists for. Every stop-short assertion below
//! is therefore about where the *plant* came to rest, not where the
//! target did.

mod common;

use common::par6_config;
use par6_config::JogProfile;
use par6_motion::{JogDirection, JogEngine, MotionError, NUM_JOINTS};

const HOME: [f64; NUM_JOINTS] = [0.0, -1.5, 3.0, 0.0, 0.0, 3.1];

/// Velocity tracking-lag time constant \[s\] for the test plant. Sized
/// like the sim's closed-loop driver cascade (~tens of ms): at J0's
/// 4.8 rad/s jog speed this trails the target by ~0.3 rad — the
/// "substantial at full speed" regime the lookahead has to stay safe in.
const TAU_S: f64 = 0.06;

/// First-order lag on velocity tracking: measured velocity chases the
/// commanded velocity with time constant [`TAU_S`], measured position
/// integrates it. The plant can also carry an initial velocity of its
/// own — the state a jog activation inherits when it preempts a move
/// still in flight.
struct LagPlant {
    q: [f64; NUM_JOINTS],
    v: [f64; NUM_JOINTS],
    dt: f64,
    alpha: f64,
}

impl LagPlant {
    fn new(q0: &[f64; NUM_JOINTS], dt: f64) -> Self {
        Self {
            q: *q0,
            v: [0.0; NUM_JOINTS],
            dt,
            alpha: dt / (TAU_S + dt),
        }
    }

    fn step(&mut self, qd_cmd: &[f64; NUM_JOINTS]) {
        for (j, cmd) in qd_cmd.iter().enumerate() {
            self.v[j] += (cmd - self.v[j]) * self.alpha;
            self.q[j] += self.v[j] * self.dt;
        }
    }
}

/// Drive `n` ticks through the lag plant, asserting the per-tick
/// commanded-velocity slew stays within a·dt throughout.
fn run_tracked(engine: &mut JogEngine, plant: &mut LagPlant, n: usize, max_dv: &[f64; NUM_JOINTS]) {
    let mut prev_qd: [f64; NUM_JOINTS] = std::array::from_fn(|j| engine.velocity(j));
    for _ in 0..n {
        let out = engine.tick(&plant.q);
        for j in 0..NUM_JOINTS {
            let dv = (out.qd[j] - prev_qd[j]).abs();
            assert!(
                dv <= max_dv[j] * (1.0 + 1e-9) + 1e-12,
                "joint {j} velocity slew {dv} exceeds a*dt {}",
                max_dv[j]
            );
        }
        prev_qd = out.qd;
        plant.step(&out.qd);
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
    let soft_max = cfg.joints[0].limits.soft_max_rad;
    let a = jog_accels(&cfg);
    let max_dv: [f64; NUM_JOINTS] = std::array::from_fn(|j| a[j] * dt);

    let mut engine = JogEngine::new(&cfg).unwrap();
    engine.activate(&HOME);
    let mut plant = LagPlant::new(&HOME, dt);

    // Jog J0 positive at max speed until the lookahead stops it. The
    // MEASURED pose — trailing the target through the lag — must come to
    // rest short of the soft limit.
    engine.command(0, JogDirection::Positive, 1.0).unwrap();
    run_tracked(&mut engine, &mut plant, 3000, &max_dv);
    assert!(
        engine.velocity(0) == 0.0,
        "jog must have come to rest, v = {}",
        engine.velocity(0)
    );
    assert!(
        plant.q[0] < soft_max,
        "measured pose {} must stop short of the soft limit {}",
        plant.q[0],
        soft_max
    );
    assert!(
        plant.q[0] > 0.5,
        "jog must actually have moved, got {}",
        plant.q[0]
    );
    assert_eq!(engine.blocked_direction(0), Some(JogDirection::Positive));

    // The block holds while the button stays pressed...
    let stopped_at = plant.q[0];
    run_tracked(&mut engine, &mut plant, 200, &max_dv);
    assert!(
        (plant.q[0] - stopped_at).abs() < 1e-3,
        "blocked jog must not move: {} -> {}",
        stopped_at,
        plant.q[0]
    );

    // ...and survives release + re-press in the same direction.
    engine.release();
    run_tracked(&mut engine, &mut plant, 50, &max_dv);
    engine.command(0, JogDirection::Positive, 1.0).unwrap();
    run_tracked(&mut engine, &mut plant, 200, &max_dv);
    assert!(
        (plant.q[0] - stopped_at).abs() < 1e-3,
        "latch must survive button release: {} -> {}",
        stopped_at,
        plant.q[0]
    );
    assert_eq!(engine.blocked_direction(0), Some(JogDirection::Positive));

    // The opposite direction clears the latch and moves.
    engine.command(0, JogDirection::Negative, 0.5).unwrap();
    assert_eq!(engine.blocked_direction(0), None);
    run_tracked(&mut engine, &mut plant, 500, &max_dv);
    assert!(
        plant.q[0] < stopped_at - 0.1,
        "opposite jog must move away, got {}",
        plant.q[0]
    );

    // After moving away, the positive direction works again until the
    // lookahead re-latches.
    engine.command(0, JogDirection::Positive, 1.0).unwrap();
    let before = plant.q[0];
    run_tracked(&mut engine, &mut plant, 3000, &max_dv);
    assert!(plant.q[0] > before, "cleared direction must jog again");
    assert!(plant.q[0] < soft_max);
    assert_eq!(engine.blocked_direction(0), Some(JogDirection::Positive));

    // Switching joints clears the latch.
    engine.command(3, JogDirection::Positive, 0.2).unwrap();
    assert_eq!(engine.blocked_direction(0), None);
    run_tracked(&mut engine, &mut plant, 100, &max_dv);
    assert!(plant.q[3] > 0.0, "switched joint must jog");

    // Negative side blocks too.
    engine.activate(&HOME);
    let mut plant = LagPlant::new(&HOME, dt);
    engine.command(4, JogDirection::Negative, 1.0).unwrap();
    run_tracked(&mut engine, &mut plant, 3000, &max_dv);
    assert!(plant.q[4] > cfg.joints[4].limits.soft_min_rad);
    assert!(plant.q[4] < -0.5, "negative jog must have moved");
    assert_eq!(engine.blocked_direction(4), Some(JogDirection::Negative));
}

#[test]
fn trapezoid_ramp_runs_to_the_block() {
    let cfg = par6_config();
    let dt = cfg.robot.tick_dt_s;
    let a = jog_accels(&cfg);
    let max_dv: [f64; NUM_JOINTS] = std::array::from_fn(|j| a[j] * dt);

    let mut engine = JogEngine::new(&cfg).unwrap();
    engine.activate(&HOME);
    engine.set_profile(JogProfile::Trapezoid);
    let mut plant = LagPlant::new(&HOME, dt);
    engine.command(0, JogDirection::Positive, 1.0).unwrap();

    // Trapezoid ramp: the very first tick slews at exactly a·dt.
    let out = engine.tick(&plant.q);
    assert!(
        (out.qd[0] - a[0] * dt).abs() < 1e-12,
        "first trapezoid tick must ramp at a*dt, got {}",
        out.qd[0]
    );
    plant.step(&out.qd);

    // Runs to the block; the lagging measured pose stops short too.
    run_tracked(&mut engine, &mut plant, 3000, &max_dv);
    assert_eq!(engine.blocked_direction(0), Some(JogDirection::Positive));
    assert!(plant.q[0] < cfg.joints[0].limits.soft_max_rad);
}

/// The measured-pose hard clamp, reached by physics instead of a
/// hand-built overrun: jog activation inherits a plant still carrying
/// velocity toward the limit (the state a jog leaves in when it preempts
/// a planned move in flight), close enough that the momentum coasts the
/// MEASURED pose past the soft limit while the engine's own integrated
/// target is still inside it. From the first tick the overrun is
/// visible, the engine must command zero outward velocity — it may not
/// keep feeding a plant that is already beyond the limit — and the
/// inward direction must still work as the escape route.
#[test]
fn hard_clamp_stops_commanding_a_plant_carried_past_the_limit() {
    let cfg = par6_config();
    let dt = cfg.robot.tick_dt_s;
    let soft_max = cfg.joints[0].limits.soft_max_rad;
    let soft_min = cfg.joints[0].limits.soft_min_rad;

    let mut q0 = HOME;
    q0[0] = soft_max - 0.05;
    let mut engine = JogEngine::new(&cfg).unwrap();
    engine.activate(&q0);
    let mut plant = LagPlant::new(&q0, dt);
    // A legal EXEC-mode speed the preempted move was still carrying.
    plant.v[0] = 3.0;
    engine.command(0, JogDirection::Positive, 1.0).unwrap();

    let mut overran = false;
    let mut max_meas = plant.q[0];
    for _ in 0..1500 {
        let measured_out = plant.q[0] > soft_max;
        let out = engine.tick(&plant.q);
        if measured_out {
            overran = true;
            assert_eq!(
                out.qd[0], 0.0,
                "an outward command may not be emitted while the measured \
                 pose is past the soft limit"
            );
        }
        assert!(
            out.q[0] <= soft_max,
            "the integrated target may never cross the soft limit, got {}",
            out.q[0]
        );
        plant.step(&out.qd);
        max_meas = max_meas.max(plant.q[0]);
    }
    assert!(
        overran,
        "the scenario must actually carry the measured pose past the limit \
         (max measured {max_meas}, soft limit {soft_max})"
    );
    // The excursion is the decaying initial momentum, nothing more: the
    // engine never accelerated the plant outward after the overrun.
    assert!(
        max_meas < soft_max + 0.2,
        "overrun {} must be bounded by the inherited momentum, not fed \
         by the engine",
        max_meas - soft_max
    );

    // Inward is the escape route: the same engine jogs the joint back
    // inside the soft window.
    engine.command(0, JogDirection::Negative, 0.5).unwrap();
    for _ in 0..400 {
        let out = engine.tick(&plant.q);
        plant.step(&out.qd);
    }
    assert!(
        plant.q[0] < soft_max && plant.q[0] > soft_min,
        "inward jog must recover the joint into the soft window, got {}",
        plant.q[0]
    );
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
