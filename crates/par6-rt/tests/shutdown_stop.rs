//! The process-exit stop sequence: halt to IDLE, wait for measured
//! rest, then one terminal SAFETY_STOP frame that idles the drives on
//! purpose — instead of leaving them to act on the last motion frame
//! until the CAN watchdog expires and drops them out mid-hold.

mod common;

use common::{ConstGravity, Rig};
use par6_bus::Reply;
use par6_rt::{Mode, RtCommand, MAX_JOINTS};

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
