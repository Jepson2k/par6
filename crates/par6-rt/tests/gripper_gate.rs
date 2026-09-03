//! The firmware-gripper send gate at the RtCore seam: what the tick's
//! gripper slot actually puts on the bus. A standing move streams DLC-5
//! only while active (action set AND the gripper reports calibrated);
//! going idle announces `action = 0` with a bounded pack of real frames
//! and then falls back to the DLC-0 poll; ownership hand-backs (homing)
//! run the same announcement so a hold commanded outside the gate is
//! never left standing. Streaming DLC-5 while the firmware side is idle
//! toggles the driver SLEEP/RESET lines every tick on real hardware —
//! the failure these tests pin out.

mod common;

use par6_bus::{FirmwareGripperCommand, GripperCommand};
use par6_config::{ConfigBundle, PreMove, SequenceStep};
use par6_rt::{Mode, RtCommand};

fn close_cmd() -> FirmwareGripperCommand {
    FirmwareGripperCommand {
        position: 180,
        speed: 80,
        current_ma: 400,
        activate: true,
        action: true,
        estop: false,
        release_dir: false,
    }
}

/// The active→idle handshake: a move streams real DLC-5 frames every
/// tick; a release announces `action = 0` with exactly three real frames
/// and then polls forever; a new move re-arms the whole cycle.
#[test]
fn a_release_announces_three_idle_frames_then_polls() {
    let mut rig = common::Rig::new();
    rig.ready();

    rig.clear_tx();
    rig.cmd(RtCommand::Gripper(close_cmd()));
    rig.tick_n(4);
    let sends = rig.gripper_sends();
    assert_eq!(sends.len(), 5);
    for s in &sends {
        assert_eq!(
            *s,
            GripperCommand::Firmware(close_cmd()),
            "an active move streams its DLC-5 frame every tick: {sends:?}"
        );
    }

    rig.clear_tx();
    rig.cmd(RtCommand::GripperIdle);
    rig.tick_n(9);
    let sends = rig.gripper_sends();
    let idle_frame = FirmwareGripperCommand {
        action: false,
        ..close_cmd()
    };
    assert_eq!(
        &sends[..3],
        &[GripperCommand::Firmware(idle_frame); 3],
        "the release is announced with real action=0 frames carrying the \
         standing command's bytes: {sends:?}"
    );
    assert!(
        sends[3..]
            .iter()
            .all(|s| *s == GripperCommand::FirmwarePoll),
        "after the announcement only the DLC-0 watchdog poll goes out: {sends:?}"
    );

    // A new move re-arms both halves of the cycle.
    rig.clear_tx();
    rig.cmd(RtCommand::Gripper(close_cmd()));
    rig.tick_n(2);
    assert_eq!(
        rig.gripper_sends(),
        vec![GripperCommand::Firmware(close_cmd()); 3],
        "a move after a completed release streams again"
    );
    rig.clear_tx();
    rig.cmd(RtCommand::GripperIdle);
    rig.tick_n(4);
    let sends = rig.gripper_sends();
    assert_eq!(
        &sends[..3],
        &[GripperCommand::Firmware(idle_frame); 3],
        "the announcement pack re-arms with the move: {sends:?}"
    );
    assert_eq!(sends[3..], [GripperCommand::FirmwarePoll; 2]);
}

/// The calibrated term of the gate: a standing move must not put a
/// single DLC-5 frame on the bus while the gripper reports uncalibrated
/// (the firmware drops it and the len-5 traffic toggles the driver
/// SLEEP/RESET lines every tick), and the same standing move starts
/// streaming on the first calibrated reply.
#[test]
fn an_uncalibrated_gripper_gets_polls_not_dlc5_frames() {
    let mut rig = common::Rig::new();
    rig.gripper_reply.calibrated = false;
    rig.ready();

    rig.clear_tx();
    rig.cmd(RtCommand::Gripper(close_cmd()));
    rig.tick_n(49);
    let sends = rig.gripper_sends();
    assert_eq!(sends.len(), 50);
    assert!(
        sends.iter().all(|s| *s == GripperCommand::FirmwarePoll),
        "no DLC-5 frame may reach an uncalibrated gripper: {sends:?}"
    );

    rig.gripper_reply.calibrated = true;
    rig.clear_tx();
    rig.tick_n(3);
    assert_eq!(
        rig.gripper_sends(),
        vec![GripperCommand::Firmware(close_cmd()); 3],
        "the standing move starts streaming on the first calibrated reply"
    );
}

/// `stop` with a live jaw byte re-targets it in place — the DLC-5
/// stream continues at the reported position with the standing
/// command's speed/current, byte 255 (fully closed, a thin part gripped
/// at the end of the stroke) included. Only byte 0 is untrusted: an
/// uncalibrated gripper reports 0, which the firmware maps to fully
/// open, so there stop degrades to the release announcement instead of
/// commanding a full-open travel.
#[test]
fn stop_retargets_the_reported_jaw_byte_or_degrades_to_release() {
    for byte in [120u8, 255] {
        let mut rig = common::Rig::new();
        rig.gripper_reply.position = byte;
        rig.ready();
        rig.cmd(RtCommand::Gripper(close_cmd()));
        rig.tick_n(2);

        rig.clear_tx();
        rig.cmd(RtCommand::GripperStop);
        rig.tick_n(3);
        let held = FirmwareGripperCommand {
            position: byte,
            ..close_cmd()
        };
        assert_eq!(
            rig.gripper_sends(),
            vec![GripperCommand::Firmware(held); 4],
            "stop holds at reported byte {byte} with the standing speed/current \
             — releasing at 255 would drop a part gripped at the stroke end"
        );
    }

    // Same stop against a gripper whose byte cannot be trusted.
    let mut rig = common::Rig::new();
    rig.gripper_reply.calibrated = false;
    rig.ready();
    rig.cmd(RtCommand::Gripper(close_cmd()));
    rig.tick_n(2);

    rig.clear_tx();
    rig.cmd(RtCommand::GripperStop);
    rig.tick_n(5);
    let sends = rig.gripper_sends();
    let idle_frame = FirmwareGripperCommand {
        action: false,
        ..close_cmd()
    };
    assert_eq!(
        &sends[..3],
        &[GripperCommand::Firmware(idle_frame); 3],
        "an untrusted byte degrades the stop to a release: {sends:?}"
    );
    assert!(
        sends[3..]
            .iter()
            .all(|s| *s == GripperCommand::FirmwarePoll),
        "and the announcement still ends in polls: {sends:?}"
    );
    assert!(
        !sends
            .iter()
            .any(|s| matches!(s, GripperCommand::Firmware(f) if f.action)),
        "a degraded stop never commands a travel: {sends:?}"
    );
}

/// A homing sequence whose only work is a firmware gripper hold.
fn gripper_hold_bundle(hold: FirmwareGripperCommand, duration_s: f64) -> ConfigBundle {
    let mut bundle = common::bundle();
    bundle.robot.homing.sequence = vec![SequenceStep {
        pre_moves: vec![PreMove::GripperMove {
            position: hold.position,
            speed: hold.speed,
            current_ma: hold.current_ma,
            activate: hold.activate,
            action: hold.action,
            estop: hold.estop,
            release_dir: hold.release_dir,
            duration_s,
        }],
        home: None,
        move_to: vec![],
        post_moves: vec![],
    }];
    bundle.robot.homing.post_moves = vec![];
    bundle
}

/// Homing streams its own DLC-5 frames outside the gate (its park hold
/// included), so on the sequence's exit the firmware is holding a grip
/// the normal path never commanded. The hand-back must announce idle
/// from the homing hold's own bytes — replaying the hold, or dropping
/// straight to polls, both strand the jaws holding forever.
#[test]
fn homing_exit_hands_the_hold_back_as_a_release() {
    let hold = FirmwareGripperCommand {
        position: 200,
        speed: 50,
        current_ma: 500,
        activate: true,
        action: true,
        estop: false,
        release_dir: false,
    };
    let mut rig = common::Rig::build_bundle(
        gripper_hold_bundle(hold, 0.1),
        par6_rt::CompletionPolicy::Settled,
        Box::new(par6_rt::ZeroGravity),
        true,
    );
    rig.boot_to_idle();
    rig.cmd(RtCommand::Enable);
    rig.cmd(RtCommand::SetMode(Mode::Homing));
    assert_eq!(rig.snap().mode, Mode::Homing);

    let done = rig.tick_until(1000, |s| !s.homing.active && s.mode == Mode::Idle);
    assert!(!done.error_active, "the hold-only sequence exits cleanly");
    assert!(
        rig.gripper_sends()
            .contains(&GripperCommand::Firmware(hold)),
        "the sequence itself streamed the hold"
    );

    rig.clear_tx();
    rig.tick_n(6);
    let sends = rig.gripper_sends();
    let released = FirmwareGripperCommand {
        action: false,
        ..hold
    };
    assert_eq!(
        &sends[..3],
        &[GripperCommand::Firmware(released); 3],
        "the exit announces idle from the homing hold's own bytes: {sends:?}"
    );
    assert!(
        sends[3..]
            .iter()
            .all(|s| *s == GripperCommand::FirmwarePoll),
        "and settles on the watchdog poll: {sends:?}"
    );
}

/// The same hand-back on the abort path: leaving HOMING mid-hold (a
/// user-requested exit) must not leave the aborted hold standing.
#[test]
fn a_homing_abort_hands_the_hold_back_too() {
    let hold = FirmwareGripperCommand {
        position: 90,
        speed: 40,
        current_ma: 300,
        activate: true,
        action: true,
        estop: false,
        release_dir: false,
    };
    let mut rig = common::Rig::build_bundle(
        gripper_hold_bundle(hold, 30.0),
        par6_rt::CompletionPolicy::Settled,
        Box::new(par6_rt::ZeroGravity),
        true,
    );
    rig.boot_to_idle();
    rig.cmd(RtCommand::Enable);
    rig.cmd(RtCommand::SetMode(Mode::Homing));
    rig.tick_n(20);
    assert!(
        rig.snap().homing.active,
        "the long hold keeps the sequence up"
    );
    assert!(
        rig.gripper_sends()
            .contains(&GripperCommand::Firmware(hold)),
        "the hold reached the bus before the abort"
    );

    rig.clear_tx();
    rig.cmd(RtCommand::SetMode(Mode::Idle));
    rig.tick_n(5);
    let sends = rig.gripper_sends();
    let released = FirmwareGripperCommand {
        action: false,
        ..hold
    };
    let announced = sends
        .iter()
        .filter(|s| **s == GripperCommand::Firmware(released))
        .count();
    assert_eq!(
        announced, 3,
        "the abort announces idle from the aborted hold's bytes: {sends:?}"
    );
    assert!(
        !sends
            .iter()
            .any(|s| matches!(s, GripperCommand::Firmware(f) if f.action)),
        "no frame after the abort re-commands the hold: {sends:?}"
    );
    assert_eq!(
        sends.last(),
        Some(&GripperCommand::FirmwarePoll),
        "the announcement has already settled on polls: {sends:?}"
    );
}

/// A stop holds at the reported jaw byte when the gripper is calibrated
/// — 0 (fully open) is a legitimate hold — and degrades to a release
/// when it is not, whatever stale byte the reply carries.
#[test]
fn a_stop_holds_at_any_calibrated_byte_and_releases_an_uncalibrated_one() {
    let mut rig = common::Rig::new();
    rig.ready();
    rig.cmd(RtCommand::Gripper(close_cmd()));
    rig.gripper_reply.calibrated = true;
    rig.gripper_reply.position = 0;
    rig.tick_n(2);

    rig.clear_tx();
    rig.cmd(RtCommand::GripperStop);
    rig.tick();
    let held = FirmwareGripperCommand {
        position: 0,
        ..close_cmd()
    };
    assert_eq!(
        rig.gripper_sends(),
        vec![GripperCommand::Firmware(held); 2],
        "a calibrated gripper parked fully open is held there, not released"
    );

    rig.cmd(RtCommand::Gripper(close_cmd()));
    rig.gripper_reply.calibrated = false;
    rig.gripper_reply.position = 120;
    rig.tick_n(2);

    rig.clear_tx();
    rig.cmd(RtCommand::GripperStop);
    let sends = rig.gripper_sends();
    assert!(
        matches!(sends.first(), Some(GripperCommand::Firmware(f)) if !f.action),
        "an uncalibrated gripper's byte is not a pose: the stop must release, got {sends:?}"
    );
}
