//! The process-exit stop sequence, run as `par6d` runs it: the optional
//! retreat to the rest pose, halt to IDLE, wait for measured rest, then
//! one terminal SAFETY_STOP frame that idles the drives on purpose —
//! instead of leaving them to act on the last motion frame until the CAN
//! watchdog expires and drops them out mid-hold.

mod common;

use common::{bundle, SimCore};
use par6_bus::FirmwareGripperCommand;
use par6_config::ConfigBundle;
use par6_rt::{Mode, RtCommand, StateSnapshot, MAX_JOINTS};

/// Run the exit sequence under a virtual clock, recording the snapshot
/// of every paced tick. Returns the trace and the terminal tick's
/// snapshot.
fn exit(sim: &mut SimCore) -> (Vec<StateSnapshot>, StateSnapshot) {
    let SimCore { core, handles, .. } = sim;
    let mut trace = Vec::new();
    core.shutdown_stop_paced(|| trace.push(handles.snapshots.latest()));
    (trace, handles.snapshots.latest())
}

fn max_speed(s: &StateSnapshot) -> f64 {
    s.qd.iter().fold(0.0, |m, v| m.max(v.abs()))
}

/// The shipped config, retreat on or off.
fn shipped(safe_park: bool) -> ConfigBundle {
    let mut b = bundle();
    b.robot.shutdown.safe_park = safe_park;
    b
}

/// A pose 0.3 rad off the rest pose on every joint, inside every soft
/// window, and clear of the endstops the retreat parks on.
fn away_from_rest(b: &ConfigBundle) -> [f64; MAX_JOINTS] {
    let mut q = [0.0; MAX_JOINTS];
    for ((q, park), j) in q.iter_mut().zip(b.robot.safe_park_q()).zip(&b.robot.joints) {
        *q = (park + 0.3).min(j.limits.soft_max_rad - 0.3);
    }
    q
}

/// The rest pose the retreat must land on, from the `[shutdown]`
/// requirement: the park pose, with each `endstop_joints` entry on its
/// homing endstop.
fn rest_pose(b: &ConfigBundle) -> [f64; MAX_JOINTS] {
    let mut q = [0.0; MAX_JOINTS];
    for (q, p) in q.iter_mut().zip(&b.robot.robot.park_pose_rad) {
        *q = *p;
    }
    for &j in &b.robot.shutdown.endstop_joints {
        let j = usize::from(j);
        q[j] = b.robot.homing.joints[j].home_offset_rad;
    }
    q
}

/// The drives idle once the limp frame is out: SAFETY_STOP holds, and
/// no joint is commanded any torque.
fn assert_limp(sim: &mut SimCore) {
    let s = sim.tick();
    assert_eq!(s.mode, Mode::SafetyStop);
    assert_eq!(
        s.tau_commanded, [0.0; MAX_JOINTS],
        "the drives are commanded nothing after the exit"
    );
}

/// A moving arm is brought to rest before the drives are idled.
#[test]
fn the_exit_waits_for_rest_then_idles_the_drives() {
    let b = shipped(false);
    let mut sim = SimCore::landed_at(&b, &away_from_rest(&b));
    sim.cmd(RtCommand::SetMode(Mode::Jog));
    let mut speeds = [0.0; MAX_JOINTS];
    speeds[0] = 0.6;
    sim.cmds
        .send(RtCommand::Jog { speeds, accel: 1.0 })
        .expect("command channel");
    let mut s = sim.tick();
    for _ in 0..(1.0 / sim.dt) as u32 {
        s = sim.tick();
    }
    assert!(max_speed(&s) > 0.2, "the jog is under way: {:?}", s.qd);

    let (trace, terminal) = exit(&mut sim);
    assert!(
        trace
            .iter()
            .any(|s| s.mode == Mode::Idle && max_speed(s) > 0.2),
        "the halt drops to IDLE while the arm is still moving"
    );
    assert_eq!(terminal.mode, Mode::SafetyStop);
    assert!(
        max_speed(&terminal) < 0.05,
        "the limp frame waited for rest: {:?} rad/s",
        terminal.qd
    );
    assert_limp(&mut sim);
}

/// `[shutdown] safe_park`: from any working mode the exit drives the arm
/// to the rest pose at the configured speed, holding the jaws where they
/// are, then idles it like any other exit.
#[test]
fn the_retreat_lands_on_the_rest_pose_at_its_speed_from_every_working_mode() {
    let b = shipped(true);
    let cfg = &b.robot.shutdown;
    for entry in [Mode::Idle, Mode::Exec, Mode::Jog] {
        let mut sim = SimCore::landed_at(&b, &away_from_rest(&b));
        sim.cmd(RtCommand::GripperCalibrate);
        let calibrated = |s: &StateSnapshot| s.gripper.reply.is_some_and(|r| r.calibrated);
        let mut s = sim.tick();
        for _ in 0..(10.0 / sim.dt) as u32 {
            if calibrated(&s) {
                break;
            }
            s = sim.tick();
        }
        assert!(calibrated(&s), "{entry:?}: the jaws calibrate");
        // A slow close, still under way when the exit begins.
        sim.cmd(RtCommand::Gripper(FirmwareGripperCommand {
            position: 220,
            speed: 10,
            current_ma: 400,
            activate: true,
            action: true,
            estop: false,
            release_dir: false,
        }));
        for _ in 0..10 {
            sim.tick();
        }
        if entry != Mode::Idle {
            assert_eq!(sim.cmd(RtCommand::SetMode(entry)).mode, entry);
        }
        let before = sim.tick();

        let (trace, _) = exit(&mut sim);
        let retreat: Vec<&StateSnapshot> =
            trace.iter().filter(|s| s.mode == Mode::Stream).collect();
        assert!(!retreat.is_empty(), "{entry:?}: the exit retreats");
        let landed = retreat.last().expect("retreat ticks");
        for (j, (q, want)) in landed.q.iter().zip(rest_pose(&b)).enumerate() {
            assert!(
                (q - want).abs() < cfg.tolerance_rad,
                "{entry:?}: J{j} retreated to {q} rad, the rest pose is {want}"
            );
        }
        for s in &retreat {
            for (j, qd) in s.qd_commanded.iter().enumerate() {
                assert!(
                    qd.abs() <= cfg.velocity_limit_rad_s + 1e-9,
                    "{entry:?}: J{j} is commanded {qd} rad/s, over the configured {}",
                    cfg.velocity_limit_rad_s
                );
            }
        }
        let jaw = |s: &StateSnapshot| s.gripper.reply.map(|r| i32::from(r.position));
        let held = jaw(&before).expect("the gripper reports its jaws");
        for s in &retreat {
            let at = jaw(s).expect("the gripper reports its jaws");
            assert!(
                (at - held).abs() <= 2,
                "{entry:?}: the jaws moved from {held} to {at} during the retreat"
            );
        }
        assert_limp(&mut sim);
    }
}

/// A retreat that never arrives gives up after its configured window and
/// still idles the drives.
#[test]
fn a_retreat_that_never_arrives_times_out_at_its_configured_window() {
    let mut b = shipped(true);
    b.robot.shutdown.timeout_s = 0.5;
    // Nothing is ever close enough to count as arrived.
    b.robot.shutdown.tolerance_rad = 0.0;
    let mut sim = SimCore::landed_at(&b, &away_from_rest(&b));
    let (trace, terminal) = exit(&mut sim);
    let retreating = trace.iter().filter(|s| s.mode == Mode::Stream).count();
    assert_eq!(
        retreating,
        (b.robot.shutdown.timeout_s / sim.dt).round() as usize,
        "the retreat runs its configured window and no longer"
    );
    assert_eq!(terminal.mode, Mode::SafetyStop);
    assert_limp(&mut sim);
}

/// Only a referenced, enabled, error-free arm retreats; the others go
/// straight to the halt.
#[test]
fn an_unhomed_errored_or_unconfigured_arm_does_not_retreat() {
    let b = shipped(true);
    let mut unhomed = SimCore::new(&b, Box::new(par6_rt::ZeroGravity));
    for _ in 0..100 {
        unhomed.tick();
    }
    unhomed.cmd(RtCommand::Enable);
    let mut errored = SimCore::landed_at(&b, &away_from_rest(&b));
    errored.cmd(RtCommand::SetSoftEstop(true));
    let off = shipped(false);
    let mut unconfigured = SimCore::landed_at(&off, &away_from_rest(&off));
    for (case, sim) in [
        ("unhomed", &mut unhomed),
        ("errored", &mut errored),
        ("safe_park off", &mut unconfigured),
    ] {
        let (trace, _) = exit(sim);
        assert!(
            trace.iter().all(|s| s.mode != Mode::Stream),
            "{case}: the arm must not retreat"
        );
    }
}

/// In FLASHING the bus is silent by contract, and the exit does not end
/// the window.
#[test]
fn the_exit_keeps_a_flashing_window_bus_silent() {
    let b = shipped(true);
    let mut sim = SimCore::landed_at(&b, &away_from_rest(&b));
    sim.cmd(RtCommand::Disable);
    sim.cmd(RtCommand::AssertParked);
    assert_eq!(
        sim.cmd(RtCommand::SetMode(Mode::Flashing)).mode,
        Mode::Flashing
    );
    sim.core.bus_mut().reset_tx_peak();
    let (_, terminal) = exit(&mut sim);
    assert_eq!(terminal.mode, Mode::Flashing);
    assert_eq!(
        sim.core.bus_mut().peak_tx_frames_per_tick(),
        0,
        "not one frame may go out during FLASHING"
    );
}
