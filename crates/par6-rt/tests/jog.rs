//! Jogging through the core onto the simulated arm: `RtCommand::Jog` into
//! the production jog engine, its frames into the drives, the plant's
//! measured pose back into the lookahead. Stop-short claims are about
//! where the measured joint came to rest, not where the target did.

mod common;

use common::{bundle_at, SimCore};
use par6_config::{ConfigBundle, JogProfile};
use par6_rt::{Mode, RtCommand, StateSnapshot, MAX_JOINTS};

const HOME: [f64; MAX_JOINTS] = [0.0, -1.5, 3.0, 0.0, 0.0, 3.1];

fn one(joint: usize, pct: f64) -> [f64; MAX_JOINTS] {
    let mut speeds = [0.0; MAX_JOINTS];
    speeds[joint] = pct;
    speeds
}

fn blocked(s: &StateSnapshot, joint: usize, positive: bool) -> bool {
    s.jog.blocked_mask & (if positive { 2 } else { 1 } << (2 * joint)) != 0
}

/// A simulated arm referenced at `q0`, in JOG.
fn jogging_at(b: &ConfigBundle, q0: &[f64; MAX_JOINTS]) -> SimCore {
    let mut sim = SimCore::landed_at(b, q0);
    assert_eq!(sim.cmd(RtCommand::SetMode(Mode::Jog)).mode, Mode::Jog);
    sim
}

fn jog(sim: &mut SimCore, speeds: [f64; MAX_JOINTS], accel: f64) -> StateSnapshot {
    sim.cmd(RtCommand::Jog { speeds, accel })
}

/// Tick for `secs`, asserting every tick that the target never crosses
/// joint 0's soft maximum and that the commanded velocity never slews
/// faster than the jog acceleration allows. Returns the last snapshot.
fn run(sim: &mut SimCore, secs: f64, soft_max: f64, max_dv: f64) -> StateSnapshot {
    let mut prev = sim.tick();
    for _ in 0..(secs / sim.dt) as u32 {
        let s = sim.tick();
        assert!(
            s.q_commanded[0] < soft_max,
            "tick {}: the target reached the soft limit ({} >= {soft_max})",
            s.tick,
            s.q_commanded[0]
        );
        let dv = (s.qd_commanded[0] - prev.qd_commanded[0]).abs();
        assert!(
            dv <= max_dv * (1.0 + 1e-6) + 1e-9,
            "tick {}: J0 commanded velocity slewed {dv} rad/s in one tick (limit {max_dv})",
            s.tick
        );
        prev = s;
    }
    prev
}

fn j0_jog_accel(b: &ConfigBundle) -> f64 {
    let l = &b.robot.joints[0].limits;
    let t = b.robot.jog.accel_time_s.max(par6_motion::MIN_ACCEL_TIME_S);
    (l.velocity_rad_s / t).min(l.acceleration_rad_s2)
}

/// A jog stops the joint short of its soft limit and latches that
/// direction for its own joint only; the latch outlasts letting go of
/// the button, a neighbour joining the jog, and is cleared by jogging the
/// other way or by the joint leaving the driven set.
#[test]
fn a_jog_stops_short_of_the_soft_limit_and_latches_that_joint_and_direction() {
    let b = bundle_at(0.004);
    let soft_max = b.robot.joints[0].limits.soft_max_rad;
    let max_dv = j0_jog_accel(&b) * b.robot.robot.tick_dt_s;
    let mut sim = jogging_at(&b, &HOME);

    jog(&mut sim, one(0, 1.0), 1.0);
    let s = run(&mut sim, 6.0, soft_max, max_dv);
    assert!(s.qd_commanded[0] == 0.0, "the jog came to rest");
    assert!(
        s.q[0] < soft_max,
        "the arm rests short: {} vs {soft_max}",
        s.q[0]
    );
    assert!(s.q[0] > 0.5, "the jog actually moved: {}", s.q[0]);
    assert!(blocked(&s, 0, true), "the positive direction latched");
    let stopped_at = s.q[0];

    // Letting go and pressing again: still blocked, still put.
    jog(&mut sim, [0.0; MAX_JOINTS], 1.0);
    run(&mut sim, 0.2, soft_max, max_dv);
    jog(&mut sim, one(0, 1.0), 1.0);
    let s = run(&mut sim, 0.8, soft_max, max_dv);
    assert!(blocked(&s, 0, true), "the latch outlasts the release");
    assert!(
        (s.q[0] - stopped_at).abs() < 0.01,
        "a latched jog does not move"
    );

    // A neighbour joins: J0 keeps its latch, J3 moves.
    let mut both = one(0, 1.0);
    both[3] = 0.2;
    jog(&mut sim, both, 1.0);
    let s = run(&mut sim, 1.0, soft_max, max_dv);
    assert!(
        blocked(&s, 0, true),
        "a growing driven set keeps J0's latch"
    );
    assert!(
        (s.q[0] - stopped_at).abs() < 0.01,
        "J0 stays put while J3 jogs"
    );
    assert!(
        s.q[3] > HOME[3] + 0.05,
        "the joining joint jogs: {}",
        s.q[3]
    );

    // J0 leaving the set clears its latch.
    let s = jog(&mut sim, one(3, 0.2), 1.0);
    assert!(
        !blocked(&s, 0, true),
        "leaving the driven set clears the latch"
    );

    // The other way clears it too, and moves away.
    jog(&mut sim, one(0, 1.0), 1.0);
    run(&mut sim, 0.5, soft_max, max_dv);
    let s = jog(&mut sim, one(0, -0.5), 1.0);
    assert!(!blocked(&s, 0, true), "the opposite jog clears the latch");
    let s = run(&mut sim, 1.0, soft_max, max_dv);
    assert!(
        s.q[0] < stopped_at - 0.1,
        "the opposite jog moves away: {}",
        s.q[0]
    );
}

/// At any tick rate, profile and jog fraction, the lookahead stops the
/// target short of the limit and the arm comes to rest short of it.
#[test]
fn the_lookahead_stops_short_at_any_tick_rate_profile_and_fraction() {
    for dt in [0.004, 0.02] {
        for profile in [JogProfile::Scurve, JogProfile::Trapezoid] {
            for accel in [1.0, 0.25] {
                let mut b = bundle_at(dt);
                b.robot.jog.profile = profile;
                let soft_max = b.robot.joints[0].limits.soft_max_rad;
                let mut start = HOME;
                start[0] = soft_max - 40f64.to_radians();
                let case = format!("dt {dt} {profile:?} accel {accel}");
                let mut sim = jogging_at(&b, &start);
                jog(&mut sim, one(0, 1.0), accel);
                let max_dv = j0_jog_accel(&b) * accel * dt;
                let s = run(&mut sim, 6.0, soft_max, max_dv);
                assert_eq!(s.qd_commanded[0], 0.0, "{case}: the jog comes to rest");
                assert!(blocked(&s, 0, true), "{case}: the direction latches");
                assert!(s.q[0] < soft_max, "{case}: the arm rests short: {}", s.q[0]);
            }
        }
    }
}

/// An arm already past its soft limit — placed there, or carried there —
/// gets nothing outward from a jog, and a jog inward brings it back.
#[test]
fn a_jog_past_the_soft_limit_commands_nothing_outward_and_recovers_inward() {
    let b = bundle_at(0.004);
    let l = &b.robot.joints[0].limits;
    let mut start = HOME;
    start[0] = 0.5 * (l.soft_max_rad + l.hard_max_rad);
    let mut sim = jogging_at(&b, &start);
    jog(&mut sim, one(0, 1.0), 1.0);
    for _ in 0..250 {
        let s = sim.tick();
        assert!(
            s.qd_commanded[0] <= 0.0,
            "tick {}: an outward velocity was commanded past the soft limit",
            s.tick
        );
        assert!(
            s.q_commanded[0] <= start[0] + 1e-6,
            "tick {}: the target moved further out",
            s.tick
        );
    }
    jog(&mut sim, one(0, -0.5), 1.0);
    for _ in 0..(2.0 / sim.dt) as u32 {
        sim.tick();
    }
    let s = sim.tick();
    assert!(
        s.q[0] < l.soft_max_rad && s.q[0] > l.soft_min_rad,
        "the inward jog brings the joint back into its window: {}",
        s.q[0]
    );
}

/// The lookahead judges the distance left from the MEASURED pose: a joint
/// held where it is by an obstruction, whose target has run on toward
/// the limit, has not reached it and must not latch the direction.
#[test]
fn an_obstructed_joint_is_judged_where_it_is_not_where_its_target_ran() {
    let b = bundle_at(0.004);
    let soft_max = b.robot.joints[0].limits.soft_max_rad;
    let node = b.robot.joints[0].node_id;
    let mut sim = jogging_at(&b, &HOME);
    // A load the drive's whole current limit only balances: the joint
    // is held where it is.
    let ilim = b.robot.joints[0].ilim_ma;
    sim.core.bus_mut().set_joint_load_ma(node, ilim);
    let held = sim.tick().q[0];
    jog(&mut sim, one(0, 0.2), 1.0);
    let mut s = sim.tick();
    for _ in 0..(6.0 / sim.dt) as u32 {
        s = sim.tick();
    }
    assert!(
        (s.q[0] - held).abs() < 0.2,
        "the obstruction holds the joint: {} from {held}",
        s.q[0]
    );
    assert!(
        s.q_commanded[0] - s.q[0] > 0.5,
        "the target ran on ahead of the held joint: target {}, joint {}",
        s.q_commanded[0],
        s.q[0]
    );
    assert!(
        s.q_commanded[0] <= soft_max + 1e-9,
        "the target parks at the limit, never past it: {}",
        s.q_commanded[0]
    );
    assert!(
        !blocked(&s, 0, true),
        "a joint far from its limit must not latch the direction"
    );
}
