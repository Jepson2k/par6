//! The `[freedrive]` drift lock on the IDLE gravity hold, asserted on
//! the snapshot and on the frames the bus received: it arms after the
//! settle window, holds through the drive's impedance frame with a
//! clamped integral, dissolves on the first tick of motion, re-arms at
//! the NEW pose, and adds nothing when off or outside freedrive.

mod common;

use common::{bundle, ConstGravity, Rig};
use par6_bus::spectral::{torque_to_ma_factor, trunc_to_wire};
use par6_bus::{JointCommand, Pack};
use par6_config::FreedriveConfig;
use par6_rt::{CompletionPolicy, RtCommand, MAX_JOINTS};

const G: [f64; MAX_JOINTS] = [0.5, -1.2, 0.8, 0.05, -0.02, 0.01];

fn lock_cfg() -> FreedriveConfig {
    FreedriveConfig {
        drift_lock: true,
        release_rad_s: 0.05,
        settle_s: 0.1,
        ki_nm_rad_s: 2.0,
        integral_limit_nm: 0.3,
    }
}

/// IDLE, homed, enabled, gravity on — freedrive — with the lock configured.
fn locked_rig() -> Rig {
    let mut b = bundle();
    b.robot.freedrive = lock_cfg();
    let mut rig = Rig::build_bundle(
        b,
        CompletionPolicy::Settled,
        Box::new(ConstGravity(G)),
        true,
    );
    rig.ready();
    rig
}

fn settle_ticks(rig: &Rig) -> u32 {
    (lock_cfg().settle_s / rig.dt).round() as u32
}

/// Tick until the lock arms; the ticks it took, bounded by the settle
/// window. Every frame before that is the plain gravity hold.
fn tick_until_armed(rig: &mut Rig) -> u32 {
    let limit = settle_ticks(rig) + 1;
    for n in 1..=limit {
        let s = rig.snap_after_tick();
        if s.drift_lock.armed {
            return n;
        }
        assert_torque_only(&rig.last_joints(), &torque_only_ma(&G), "before arming");
    }
    panic!("lock never armed within the settle window ({limit} ticks)");
}

/// The mA each joint gets for a torque of `nm`.
fn torque_only_ma(nm: &[f64; MAX_JOINTS]) -> [i16; MAX_JOINTS] {
    let robot = &bundle().robot;
    std::array::from_fn(|i| {
        let j = &robot.joints[i];
        let f = torque_to_ma_factor(j.gear_ratio, j.gear_efficiency, j.kt_nm_a, j.dir);
        trunc_to_wire(nm[i] * f) as i16
    })
}

fn assert_torque_only(frames: &[JointCommand], expect_ma: &[i16; MAX_JOINTS], ctx: &str) {
    for (i, f) in frames.iter().enumerate() {
        assert_eq!(f.pos, None, "{ctx}: J{i} pos omitted (no position hold)");
        assert_eq!(f.vel, None, "{ctx}: J{i} vel omitted (torque-only)");
        assert_eq!(f.cur_ma, Some(expect_ma[i]), "{ctx}: J{i} current");
        assert_eq!(f.pack, Pack::Pid, "{ctx}: J{i} pid pack");
    }
}

/// The drive's impedance hold at `hold` with `ff` as the feedforward.
fn assert_pd_hold(rig: &mut Rig, hold: &[f64; MAX_JOINTS], ff: &[f64; MAX_JOINTS], ctx: &str) {
    let expect_ma = torque_only_ma(ff);
    for (i, f) in rig.last_joints().iter().enumerate() {
        assert_eq!(f.pack, Pack::Pd, "{ctx}: J{i} impedance pack");
        assert_eq!(
            f.pos,
            Some(rig.conv[i].motor_ticks(hold[i])),
            "{ctx}: J{i} holds the pose"
        );
        assert_eq!(f.vel, Some(0), "{ctx}: J{i} zero velocity target");
        assert_eq!(f.cur_ma, Some(expect_ma[i]), "{ctx}: J{i} feedforward");
    }
}

fn assert_zero_velocity(frames: &[JointCommand], ctx: &str) {
    for (i, f) in frames.iter().enumerate() {
        assert_eq!(f.vel, Some(0), "{ctx}: J{i} active zero velocity");
        assert_eq!(f.cur_ma, Some(0), "{ctx}: J{i} zero current");
        assert_eq!(f.pack, Pack::Pid, "{ctx}: J{i} pid pack");
    }
}

#[test]
fn arms_after_the_settle_window_holds_and_dissolves_on_the_first_tick_of_motion() {
    let mut rig = locked_rig();
    let dt = rig.dt;

    // Still from enable: the pose is captured once the window has passed.
    let took = tick_until_armed(&mut rig);
    assert!(
        took >= settle_ticks(&rig) - 1,
        "armed after {took} ticks, before the {} tick settle window",
        settle_ticks(&rig)
    );
    let s = rig.snap();
    assert_eq!(
        s.drift_lock.hold_rad, s.q,
        "the hold is the pose it was still at"
    );
    assert_eq!(
        s.drift_lock.integral_nm, [0.0; MAX_JOINTS],
        "no error, no integral"
    );
    assert_pd_hold(&mut rig, &s.drift_lock.hold_rad, &G, "armed");

    // The arm sags 0.02 rad on J0 without ever moving fast: the drive
    // keeps holding the captured pose and the integral accumulates the
    // error, one tick at a time.
    rig.pose[0] += 0.02;
    let s = rig.snap_after_tick();
    let err = s.drift_lock.hold_rad[0] - s.q[0];
    assert!(err < -0.019 && err > -0.021, "measured error {err}");
    assert!(
        (s.drift_lock.integral_nm[0] - 2.0 * err * dt).abs() < 1e-12,
        "first-tick integral {} must be one tick of ki·e",
        s.drift_lock.integral_nm[0]
    );
    let s = rig.tick_until(100, |s| s.drift_lock.integral_nm[0] < -0.01);
    assert_eq!(
        s.drift_lock.integral_nm[1..],
        [0.0; MAX_JOINTS - 1],
        "other joints carry no error"
    );
    let mut ff = G;
    ff[0] += s.drift_lock.integral_nm[0];
    assert_pd_hold(
        &mut rig,
        &s.drift_lock.hold_rad,
        &ff,
        "integral on the feedforward",
    );
    // Observable without any new wire field: commanded minus gravity.
    assert!(
        (s.tau_commanded[0] - s.gravity_torque_nm[0] - s.drift_lock.integral_nm[0]).abs() < 0.02,
        "commanded {} vs gravity {} vs integral {}",
        s.tau_commanded[0],
        s.gravity_torque_nm[0],
        s.drift_lock.integral_nm[0]
    );

    // The operator pushes: the first tick above the release speed drops
    // the hold AND the integral, and the frame is pure gravity again.
    rig.vel[0] = 0.1;
    let s = rig.snap_after_tick();
    assert!(
        !s.drift_lock.armed,
        "motion dissolves the lock on that tick"
    );
    assert_eq!(
        s.drift_lock.integral_nm, [0.0; MAX_JOINTS],
        "integral zeroed with it"
    );
    assert_torque_only(&rig.last_joints(), &torque_only_ma(&G), "pushed: pure G(q)");
    rig.tick_n(settle_ticks(&rig) * 2);
    assert!(!rig.snap().drift_lock.armed, "never re-arms while moving");

    // Left still at the new pose: re-armed THERE, pulling nowhere.
    rig.vel[0] = 0.0;
    rig.pose[0] += 0.3;
    tick_until_armed(&mut rig);
    let s = rig.snap();
    assert_eq!(s.drift_lock.hold_rad, s.q, "re-armed at the new pose");
    assert_eq!(
        s.drift_lock.integral_nm, [0.0; MAX_JOINTS],
        "nothing toward the old pose"
    );
    assert_pd_hold(&mut rig, &s.q, &G, "re-armed");
}

#[test]
fn the_integral_clamp_is_hit_exactly_and_never_exceeded() {
    let mut rig = locked_rig();
    tick_until_armed(&mut rig);
    let limit = lock_cfg().integral_limit_nm;

    // A full radian of error: ki·e·dt per tick reaches the clamp in a
    // fraction of a second and must sit there.
    rig.pose[1] -= 1.0;
    let mut peak = 0.0f64;
    for _ in 0..400 {
        let s = rig.snap_after_tick();
        peak = peak.max(s.drift_lock.integral_nm[1].abs());
    }
    let s = rig.snap();
    assert_eq!(
        s.drift_lock.integral_nm[1], limit,
        "integral sits exactly on the clamp"
    );
    assert_eq!(peak, limit, "integral never exceeded the clamp");
    // ... and the wire carries the hold with G(q) plus exactly the clamp.
    let mut ff = G;
    ff[1] += limit;
    assert_pd_hold(&mut rig, &s.drift_lock.hold_rad, &ff, "clamped");
}

#[test]
fn the_lock_adds_nothing_when_off_or_outside_freedrive() {
    // Shipped default: off. Freedrive frames are exactly G(q) however
    // long the arm sits off any pose.
    let mut rig = Rig::with_gravity(Box::new(ConstGravity(G)));
    rig.ready();
    rig.pose[2] += 0.5;
    rig.tick_n(200);
    assert!(!rig.snap().drift_lock.armed);
    assert_torque_only(
        &rig.last_joints(),
        &torque_only_ma(&G),
        "lock off: pure G(q)",
    );

    // Configured on, but freedrive ends: gravity comp off is the active
    // zero-velocity idle, and the lock state is gone with it.
    let mut rig = locked_rig();
    tick_until_armed(&mut rig);
    rig.pose[2] += 0.5;
    rig.tick_n(5);
    assert!(
        rig.snap().drift_lock.integral_nm[2] < 0.0,
        "armed and integrating"
    );
    rig.cmd(RtCommand::SetGravityComp(false));
    let s = rig.snap();
    assert!(!s.drift_lock.armed);
    assert_eq!(s.drift_lock.integral_nm, [0.0; MAX_JOINTS]);
    assert_zero_velocity(&rig.last_joints(), "grav-off idle");
    // Back on: it re-arms where the arm is now, not where it was.
    rig.cmd(RtCommand::SetGravityComp(true));
    tick_until_armed(&mut rig);
    let s = rig.snap();
    assert_eq!(s.drift_lock.hold_rad, s.q);

    // Disabled: no gravity hold, no lock.
    rig.cmd(RtCommand::Disable);
    assert!(!rig.snap().drift_lock.armed);
    assert_zero_velocity(&rig.last_joints(), "disabled idle");
}
