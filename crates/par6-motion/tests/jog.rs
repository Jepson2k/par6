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

/// A speed array driving `joint` alone at `signed_pct`.
fn one(joint: usize, signed_pct: f64) -> [f64; NUM_JOINTS] {
    let mut speeds = [0.0; NUM_JOINTS];
    speeds[joint] = signed_pct;
    speeds
}

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
    engine.command(&one(0, 1.0)).unwrap();
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
    engine.command(&one(0, 1.0)).unwrap();
    run_tracked(&mut engine, &mut plant, 200, &max_dv);
    assert!(
        (plant.q[0] - stopped_at).abs() < 1e-3,
        "latch must survive button release: {} -> {}",
        stopped_at,
        plant.q[0]
    );
    assert_eq!(engine.blocked_direction(0), Some(JogDirection::Positive));

    // The opposite direction clears the latch and moves.
    engine.command(&one(0, -0.5)).unwrap();
    assert_eq!(engine.blocked_direction(0), None);
    run_tracked(&mut engine, &mut plant, 500, &max_dv);
    assert!(
        plant.q[0] < stopped_at - 0.1,
        "opposite jog must move away, got {}",
        plant.q[0]
    );

    // After moving away, the positive direction works again until the
    // lookahead re-latches.
    engine.command(&one(0, 1.0)).unwrap();
    let before = plant.q[0];
    run_tracked(&mut engine, &mut plant, 3000, &max_dv);
    assert!(plant.q[0] > before, "cleared direction must jog again");
    assert!(plant.q[0] < soft_max);
    assert_eq!(engine.blocked_direction(0), Some(JogDirection::Positive));

    // Switching joints clears the latch.
    engine.command(&one(3, 0.2)).unwrap();
    assert_eq!(engine.blocked_direction(0), None);
    run_tracked(&mut engine, &mut plant, 100, &max_dv);
    assert!(plant.q[3] > 0.0, "switched joint must jog");

    // Negative side blocks too.
    engine.activate(&HOME);
    let mut plant = LagPlant::new(&HOME, dt);
    engine.command(&one(4, -1.0)).unwrap();
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
    engine.command(&one(0, 1.0)).unwrap();

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
    engine.command(&one(0, 1.0)).unwrap();

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
    engine.command(&one(0, -0.5)).unwrap();
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
    engine.command(&one(0, 1.0)).unwrap();
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

/// Several joints jog at once, each on its own ramp, and the direction
/// blocks stay per joint.
///
/// The engine's `v`/`acc`/`q`/`blocked` arrays were already six-wide with
/// a six-wide tick loop; only the command was scalar, so the fifth of the
/// engine that decided WHICH joint to drive threw the rest away. A
/// diagonal jog is the ordinary pendant gesture that could not be
/// expressed.
#[test]
fn several_joints_jog_together_with_independent_ramps_and_blocks() {
    let cfg = par6_config();
    let dt = cfg.robot.tick_dt_s;
    let a = jog_accels(&cfg);
    let v_full: [f64; NUM_JOINTS] = std::array::from_fn(|j| cfg.joints[j].limits.velocity_rad_s);

    let mut engine = JogEngine::new(&cfg).unwrap();
    engine.activate(&HOME);
    engine.set_profile(JogProfile::Trapezoid);

    // Gentle fractions on three joints, chosen so every one of them
    // reaches terminal velocity with room to spare before its own
    // brake-at-limits lookahead has anything to say.
    let driven = [(0usize, 0.1), (3, -0.2), (5, 0.3)];
    let mut speeds = [0.0; NUM_JOINTS];
    for (j, pct) in driven {
        speeds[j] = pct;
    }
    engine.command(&speeds).unwrap();

    // Each joint slews at its OWN a·dt on the first tick — a shared ramp
    // would put them all on one.
    let out = engine.tick(&HOME);
    for (j, pct) in driven {
        let want = pct.signum() * a[j] * dt;
        assert!(
            (out.qd[j] - want).abs() < 1e-12,
            "J{j} first tick must slew at its own a*dt {want}, got {}",
            out.qd[j]
        );
    }

    let mut plant = LagPlant::new(&HOME, dt);
    plant.step(&out.qd);
    for _ in 0..100 {
        let out = engine.tick(&plant.q);
        plant.step(&out.qd);
    }
    for (j, pct) in driven {
        let want = pct * v_full[j];
        assert!(
            (engine.velocity(j) - want).abs() < 1e-9,
            "J{j} must cruise at {want} rad/s, got {}",
            engine.velocity(j)
        );
    }
    for j in [1usize, 2, 4] {
        assert_eq!(
            engine.velocity(j),
            0.0,
            "J{j} was never commanded and must not move"
        );
    }

    // Blocks are per joint. Same driven SET, so no block clearing: J0 goes
    // to full speed and runs into its wall while J3 crawls nowhere near
    // its own.
    speeds[0] = 1.0;
    speeds[3] = -0.005;
    speeds[5] = 0.0;
    engine.command(&speeds).unwrap();
    let max_dv: [f64; NUM_JOINTS] = std::array::from_fn(|j| a[j] * dt);
    run_tracked(&mut engine, &mut plant, 3000, &max_dv);
    assert_eq!(engine.blocked_direction(0), Some(JogDirection::Positive));
    assert_eq!(
        engine.blocked_direction(3),
        None,
        "J3 is nowhere near its limit; J0's block must not spill onto it"
    );

    // Changing the driven SET clears every block — the multi-joint reading
    // of "switching joints clears the blocks".
    let mut other = [0.0; NUM_JOINTS];
    other[1] = 0.3;
    engine.command(&other).unwrap();
    assert_eq!(engine.blocked_direction(0), None);
}

/// (a) A latch belongs to its joint: a joint JOINING the driven set
/// must not wipe a still-driven joint's block — only leaving the set
/// (or the opposite command) clears a joint's own latch. Wiping every
/// block on any set change let a limit-parked joint drive further out
/// for a whole lookahead the moment a second axis joined the jog.
/// (b) The lookahead's remaining distance is the MEASURED pose, not the
/// integrated target: a stalled joint whose integrator ran ahead to the
/// soft limit must not latch its whole direction while the arm itself
/// is still far away.
#[test]
fn latches_are_per_joint_and_the_lookahead_reads_the_measured_pose() {
    let cfg = par6_config();
    let dt = cfg.robot.tick_dt_s;
    let a = jog_accels(&cfg);
    let max_dv: [f64; NUM_JOINTS] = std::array::from_fn(|j| a[j] * dt);

    // (a) Latch J0 at its soft limit, then ADD J3 to the driven set.
    let mut engine = JogEngine::new(&cfg).unwrap();
    engine.activate(&HOME);
    let mut plant = LagPlant::new(&HOME, dt);
    engine.command(&one(0, 1.0)).unwrap();
    run_tracked(&mut engine, &mut plant, 3000, &max_dv);
    assert_eq!(engine.blocked_direction(0), Some(JogDirection::Positive));
    let stopped_at = plant.q[0];

    let mut both = one(0, 1.0);
    both[3] = 0.2;
    engine.command(&both).unwrap();
    assert_eq!(
        engine.blocked_direction(0),
        Some(JogDirection::Positive),
        "a joint still driven the same way keeps its latch when the set grows"
    );
    run_tracked(&mut engine, &mut plant, 300, &max_dv);
    assert!(
        (plant.q[0] - stopped_at).abs() < 1e-3,
        "the latched joint must stay put while its neighbour jogs: {} -> {}",
        stopped_at,
        plant.q[0]
    );
    assert!(plant.q[3] > HOME[3] + 1e-3, "the joining joint must jog");

    // Leaving the driven set is what clears the latch.
    engine.command(&one(3, 0.2)).unwrap();
    assert_eq!(
        engine.blocked_direction(0),
        None,
        "a joint leaving the driven set gets a fresh start"
    );

    // (b) A stalled plant: the measured pose never moves while the
    // integrated target runs all the way to the soft limit (where the
    // never-cross clamp parks it). The direction must stay un-latched —
    // the ARM is nowhere near the limit.
    // A fraction whose stopping distance is well inside the soft range
    // (the shipped jog ramp cannot stop a FULL-speed J0 within it, so a
    // full-speed latch is honest even from the measured pose).
    let mut engine = JogEngine::new(&cfg).unwrap();
    engine.activate(&HOME);
    engine.command(&one(0, 0.2)).unwrap();
    for _ in 0..3000 {
        let out = engine.tick(&HOME);
        assert!(
            out.q[0] <= cfg.joints[0].limits.soft_max_rad,
            "the integrated target still never crosses the soft limit"
        );
    }
    assert_eq!(
        engine.blocked_direction(0),
        None,
        "a stalled joint far from its limit must not latch the direction"
    );
}
