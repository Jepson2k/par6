//! The process-exit stop sequence: halt to IDLE, wait for measured
//! rest, then one terminal SAFETY_STOP frame that idles the drives on
//! purpose — instead of leaving them to act on the last motion frame
//! until the CAN watchdog expires and drops them out mid-hold.

mod common;

use common::{bundle_at, ConstGravity, Rig};
use par6_bus::{FirmwareGripperCommand, GripperCommand, Reply, TxRecord};
use par6_rt::{CompletionPolicy, Mode, RtCommand, ZeroGravity, MAX_JOINTS};

fn one(joint: usize, pct: f64) -> [f64; MAX_JOINTS] {
    let mut speeds = [0.0; MAX_JOINTS];
    speeds[joint] = pct;
    speeds
}

/// One tick whose injected bus replies measure `qd_rad_s` on every
/// joint (the rig's own injection always measures zero speed).
fn tick_with_speed(f: &mut Rig, qd_rad_s: f64) {
    f.inject_pose();
    for i in 0..MAX_JOINTS {
        let node = f.node_of[i];
        f.core.bus_mut().inject(
            false,
            Reply::Motion {
                node,
                position_ticks: f.conv[i].motor_ticks(f.pose[i]),
                speed_ticks_s: f.conv[i].motor_speed_ticks_s(qd_rad_s) as i32,
                current_ma: 0,
            },
        );
    }
    let dt = f.dt;
    f.core.tick(dt, false);
}

#[test]
fn the_exit_sequence_waits_for_rest_then_idles_the_drives_with_a_limp_frame() {
    let mut f = Rig::with_gravity(Box::new(ConstGravity([1.5; MAX_JOINTS])));
    f.ready();
    f.cmd(RtCommand::SetMode(Mode::Jog));
    f.cmd(RtCommand::Jog {
        speeds: one(0, 0.6),
        accel: 1.0,
    });
    for _ in 0..5 {
        tick_with_speed(&mut f, 0.4);
    }
    assert_eq!(f.snap().mode, Mode::Jog);
    assert!(
        !f.core.at_rest(),
        "a measured 0.4 rad/s must hold the rest gate closed"
    );

    // Phase 1: the halt drops the working mode to IDLE, and the rest
    // gate stays closed until the measured speed has decayed away.
    f.core.shutdown_halt();
    f.tick();
    assert_eq!(f.snap().mode, Mode::Idle);
    let mut settle = 0;
    while !f.core.at_rest() {
        f.tick();
        settle += 1;
        assert!(settle < 100, "the rest gate never opened on a resting arm");
    }
    assert!(
        settle > 1,
        "the filtered speed cannot vanish inside one tick"
    );
    // The standing IDLE frame is the gravity float — real current on the
    // bus, so the terminal zero below is a change, not a repeat.
    let hold = f.last_joints();
    assert!(
        hold.iter().any(|c| c.cur_ma.unwrap_or(0) != 0),
        "the gravity float must carry current: {hold:?}"
    );

    // Phase 2: terminal limp — the last frame on the bus idles every
    // drive: no position hold, no velocity, zero current.
    f.core.shutdown_limp();
    f.tick();
    assert_eq!(f.snap().mode, Mode::SafetyStop);
    let last = f.last_joints();
    assert_eq!(last.len(), MAX_JOINTS);
    for (i, c) in last.iter().enumerate() {
        assert_eq!(
            c.pos, None,
            "J{i}: the terminal frame must not hold a position"
        );
        assert_eq!(
            c.vel, None,
            "J{i}: the terminal frame must not command a velocity"
        );
        assert_eq!(
            c.cur_ma,
            Some(0),
            "J{i}: the terminal frame must command zero current"
        );
    }
}

#[test]
fn the_exit_sequence_keeps_a_flashing_window_bus_silent() {
    let mut f = Rig::new();
    f.boot_to_idle();
    f.cmd(RtCommand::Disable);
    f.cmd(RtCommand::AssertParked);
    f.cmd(RtCommand::SetMode(Mode::Flashing));
    assert_eq!(f.snap().mode, Mode::Flashing);

    f.clear_tx();
    f.core.shutdown_halt();
    f.core.shutdown_limp();
    f.tick_n(3);
    assert_eq!(
        f.snap().mode,
        Mode::Flashing,
        "shutdown must not yank a flashing node out of its window"
    );
    assert!(
        f.core.bus_mut().tx_log.is_empty(),
        "not one frame may go out during FLASHING"
    );
}

/// A rig whose `[shutdown]` section asks for the retreat.
fn parking_rig(dt: f64) -> Rig {
    let mut bundle = bundle_at(dt);
    bundle.robot.shutdown.safe_park = true;
    bundle.robot.shutdown.timeout_s = 1.0;
    Rig::build_bundle(
        bundle,
        CompletionPolicy::Settled,
        Box::new(ZeroGravity),
        true,
    )
}

/// The rest pose in the rig's joint units, and a pose 0.3 rad off it on
/// every joint (inside every PAR6 soft range).
fn park_and_away(f: &Rig) -> ([f64; MAX_JOINTS], [f64; MAX_JOINTS]) {
    let cfg = bundle_at(f.dt).robot;
    let mut park = [0.0; MAX_JOINTS];
    for (p, q) in park.iter_mut().zip(cfg.safe_park_q()) {
        *p = *q;
    }
    let mut away = park;
    for (a, j) in away.iter_mut().zip(&cfg.joints) {
        *a = (*a + 0.3).min(j.limits.soft_max_rad - 0.05);
    }
    (park, away)
}

/// The exit retreat with the measured pose tracking every commanded
/// position frame: the plant with no lag.
fn retreat(f: &mut Rig, follow: bool) -> (bool, u32) {
    assert!(f.core.shutdown_park_begin(), "the retreat must start");
    let mut ticks = 0;
    let mut reached = false;
    for _ in 0..f.core.shutdown_park_timeout_ticks() {
        if f.core.shutdown_park_feed() {
            reached = true;
            break;
        }
        f.tick();
        ticks += 1;
        if follow {
            let last = f.last_joints();
            for (i, c) in last.iter().enumerate() {
                if let Some(pos) = c.pos {
                    f.pose[i] = f.conv[i].joint_rad(pos);
                }
            }
        }
    }
    f.core.shutdown_park_end();
    (reached, ticks)
}

/// Standing gripper frames on the bus after `tick`, newest last.
fn gripper_frames_since(f: &mut Rig, tick: u64) -> Vec<GripperCommand> {
    f.core
        .bus_mut()
        .tx_log
        .iter()
        .filter(|(t, _)| *t > tick)
        .filter_map(|(_, r)| match r {
            TxRecord::Gripper(g) => Some(*g),
            _ => None,
        })
        .collect()
}

/// `[shutdown] safe_park = true`: the exit drives the arm to the rest
/// pose under position control before the halt, holds the jaws where
/// they are on the way, and lands on the terminal limp frame like any
/// other exit.
///
/// Measured before this landed: `shutdown_stop` halted in place, so an
/// arm left mid-air by a process exit dropped from there when the limp
/// frame landed; `park_pose_rad` was read by the homing return only.
#[test]
fn the_exit_retreats_to_the_rest_pose_holding_the_jaws_then_idles() {
    let mut f = parking_rig(0.004);
    f.ready();
    let (park, away) = park_and_away(&f);
    f.pose = away;
    f.tick_n(3);
    // The jaws are mid-stroke under a standing move.
    f.gripper_reply.position = 120;
    f.cmd(RtCommand::Gripper(FirmwareGripperCommand {
        position: 200,
        speed: 90,
        current_ma: 400,
        activate: true,
        action: true,
        estop: false,
        release_dir: false,
    }));
    f.tick_n(3);
    let t0 = f.bus_tick();

    let (reached, ticks) = retreat(&mut f, true);
    assert!(reached, "the retreat must reach the rest pose");
    assert!(
        ticks >= 1,
        "reaching a pose 0.3 rad away takes at least a tick"
    );
    assert_eq!(f.snap().mode, Mode::Stream, "the retreat runs under STREAM");
    for (j, (pose, target)) in f.pose.iter().zip(&park).enumerate() {
        assert!(
            (pose - target).abs() < 0.03,
            "J{j}: the commanded pose must land on the rest pose: {pose} vs {target}"
        );
    }
    let frames = gripper_frames_since(&mut f, t0);
    assert!(
        frames.iter().all(|g| matches!(
            g,
            GripperCommand::Firmware(fw) if fw.action && fw.position == 120
        )),
        "the retreat holds the jaws at their reported position, never releasing: {frames:?}"
    );

    // The rest of the exit is unchanged: halt, settle, terminal limp.
    f.core.shutdown_halt();
    f.tick();
    assert_eq!(f.snap().mode, Mode::Idle);
    f.core.shutdown_limp();
    f.tick();
    assert_eq!(f.snap().mode, Mode::SafetyStop);
    let last = f.last_joints();
    assert!(
        last.iter().all(|c| c.pos.is_none() && c.cur_ma == Some(0)),
        "the terminal frame must still idle every drive: {last:?}"
    );
}

/// A retreat that never arrives — the plant does not follow — expires at
/// its configured timeout, restores the stream scale it overrode, and
/// the exit still reaches the terminal limp frame.
#[test]
fn a_retreat_that_never_arrives_times_out_and_still_idles_the_drives() {
    let mut f = parking_rig(0.004);
    f.ready();
    let (_, away) = park_and_away(&f);
    f.pose = away;
    f.tick_n(3);

    let (reached, ticks) = retreat(&mut f, false);
    assert!(!reached, "a plant that never moves cannot arrive");
    assert_eq!(
        ticks,
        f.core.shutdown_park_timeout_ticks(),
        "the retreat runs exactly its timeout, then gives up"
    );
    assert_eq!(f.core.shutdown_park_timeout_ticks(), 250, "1.0 s at 250 Hz");

    f.core.shutdown_halt();
    f.tick();
    f.core.shutdown_limp();
    f.tick();
    assert_eq!(f.snap().mode, Mode::SafetyStop);
    let last = f.last_joints();
    assert!(
        last.iter().all(|c| c.pos.is_none() && c.cur_ma == Some(0)),
        "the terminal frame must still idle every drive after a timeout: {last:?}"
    );
}

/// The retreat is a motion to an absolute pose, so it needs exactly what
/// a planned move needs: a homed, enabled, error-free arm. Anything else
/// halts where it is, and a config that never asked for it never moves.
#[test]
fn an_unhomed_errored_or_unconfigured_arm_does_not_retreat() {
    // Not asked for: the shipped default.
    let mut f = Rig::new();
    f.ready();
    let t0 = f.bus_tick();
    assert!(!f.core.shutdown_park_begin());
    f.tick();
    assert_eq!(f.snap().mode, Mode::Idle);
    assert!(
        f.joints_since(t0)
            .iter()
            .all(|(_, v)| v.iter().all(|c| c.pos.is_none())),
        "an unconfigured exit must not command a position"
    );

    // Unhomed.
    let mut f = parking_rig(0.004);
    f.boot_to_idle();
    f.cmd(RtCommand::Enable);
    assert!(!f.core.shutdown_park_begin(), "no reference, no retreat");
    f.tick();
    assert_eq!(f.snap().mode, Mode::Idle);

    // Hard error latched.
    let mut f = parking_rig(0.004);
    f.ready();
    f.cmd(RtCommand::SetSoftEstop(true));
    f.tick_n(2);
    assert_eq!(f.snap().mode, Mode::ActiveError);
    assert!(
        !f.core.shutdown_park_begin(),
        "an errored arm is not driven anywhere"
    );
    f.tick();
    assert_eq!(f.snap().mode, Mode::ActiveError);
}
