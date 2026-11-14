//! Swapping the bus backend under a running core.
//!
//! The command plane opens the new backend itself — that is where a
//! failure has a client to answer — and hands the core a bus that
//! already exists. What the core owes in return is a clean bring-up:
//! the arm on the other side of the swap is a DIFFERENT arm, and every
//! belief the old one produced has to go with it.

mod common;

use common::{bundle, Rig, SimCore};
use par6_bus::sim::SimBus;
use par6_bus::spectral::codec::Readback;
use par6_bus::{ConfigKind, DriverBus, LoopbackBus, PollAction};
use par6_rt::core::BOOT_SELFCHECK_S;
use par6_rt::{ArmState, ErrorCode, HeldArm, Mode, RtCommand, ZeroGravity, MAX_JOINTS};

/// A swapped-in backend gets the same bring-up a backend opened at
/// startup gets, and the old arm's state does not survive it.
///
/// The bug this is the shape of: boot one-shots keyed off the ABSOLUTE
/// tick. A backend swapped in at tick 90 000 would never see its
/// selfcheck (tick 8), never have its kt fetched, and never get the
/// config re-sends — so its nodes would go un-scanned, the core would
/// sit in BOOTING with no path to IDLE, and every motion command would
/// be refused by the transition table with nothing in the log to say
/// why.
#[test]
fn a_swapped_bus_re_runs_the_boot_sequence_and_drops_the_old_arms_state() {
    for dt in [0.004, 0.01] {
        let mut rig = Rig::at_tick_dt(dt);
        // Reach a working state on the first bus: homed, enabled, out of
        // BOOTING, with fresh readings from every node.
        rig.tick_n(40);
        rig.core.set_homed(true);
        rig.send(RtCommand::Enable);
        rig.tick_n(4);
        let before = rig.snap();
        assert_eq!(before.mode, Mode::Idle, "the first bus finished booting");
        assert!(before.homed && !before.error_active, "{before:?}");

        let swapped_at = before.tick;
        rig.core
            .replace_bus(LoopbackBus::new())
            .expect("the loopback backend configures");
        // The boot gate discards the old bus's identity; the new bus must
        // report the fitted tool itself.
        let configured = bundle();
        rig.core.bus_mut().report_tool(
            configured.robot.bus.gripper_node,
            configured.active_tool().unwrap().can_tool_id.unwrap(),
        );

        // Immediately after: a different arm, and the core says so rather
        // than carrying the old one's beliefs into the new readings.
        let fresh = rig.snap_after_tick();
        assert_eq!(
            fresh.mode,
            Mode::Booting,
            "the core re-boots on the new bus"
        );
        assert!(!fresh.homed, "the home reference did not survive the swap");

        // The selfcheck runs relative to the SWAP, not to process start.
        // Keyed off the absolute tick it would never fire again, and the
        // core would sit in BOOTING for the life of the process.
        let selfcheck = u64::from((BOOT_SELFCHECK_S / dt).round().max(1.0) as u32);
        let idle = rig.tick_until(120, |s| s.mode == Mode::Idle);
        assert_eq!(
            idle.tick,
            swapped_at + selfcheck,
            "at {dt} s, IDLE is reached on the selfcheck tick, counted from the \
             SWAP (swapped at {swapped_at})"
        );
        assert!(!idle.homed, "and it is still un-homed until it is homed");
    }
}

/// Per-node freshness restarts at the swap.
///
/// The failure it rules out is the quiet one: carry the old bus's
/// recency across and a swapped-in backend that answers NOTHING looks
/// healthy for a whole `lost_s` window, because every node's last
/// reading is recent — from hardware that is no longer on the other end.
/// The link would read green while the arm was unreachable.
#[test]
fn a_swapped_bus_starts_every_node_from_never_seen() {
    let mut rig = Rig::new();
    rig.tick_n(40);
    let before = rig.snap();
    assert!(
        before.nodes.iter().all(|n| n.data_age_ticks == 0),
        "the first bus is answering on every node: {:?}",
        before
            .nodes
            .iter()
            .map(|n| n.data_age_ticks)
            .collect::<Vec<_>>()
    );

    // Swap, and let the new bus stay silent.
    rig.auto_inject = false;
    rig.core
        .replace_bus(LoopbackBus::new())
        .expect("the loopback backend configures");
    // The hazard is a node that looks recent for a whole lost_s window, so
    // the silence has to outlast one tick to rule it out.
    let robot = &bundle().robot;
    for _ in 0..robot.ticks(robot.bus.lost_s) + 1 {
        let silent = rig.snap_after_tick();
        assert!(
            silent.nodes.iter().all(|n| n.data_age_ticks == u64::MAX),
            "a node kept the OLD arm's recency across the swap at tick {}: {:?}",
            silent.tick,
            silent
                .nodes
                .iter()
                .map(|n| n.data_age_ticks)
                .collect::<Vec<_>>()
        );
    }

    // And it comes back the moment the new bus answers.
    rig.auto_inject = true;
    let live = rig.tick_until(20, |s| s.nodes[0].data_age_ticks == 0);
    assert!(
        live.nodes.iter().all(|n| n.data_age_ticks == 0),
        "the new bus is answering on every node: {:?}",
        live.nodes
            .iter()
            .map(|n| n.data_age_ticks)
            .collect::<Vec<_>>()
    );
}

/// The new bus is brought up with the core's own config — every arm
/// node with its configured values and repeat count, and the CAN
/// gripper — and the core
/// reads the new arm, not the old one.
#[test]
fn a_swapped_bus_is_configured_from_the_cores_own_config() {
    let b = bundle();
    let mut sim = SimCore::new(&b, Box::new(ZeroGravity));
    let before = sim.tick_until_idle();
    let old_pose = sim.core.bus_mut().true_joint_rad();

    // The new arm's base stands 0.2 rad round from the old one's — the
    // joint gravity leaves where it is placed.
    let mut new_pose = old_pose.clone();
    new_pose[0] += 0.2;
    let mut bus = SimBus::new(common::scene(&b));
    bus.set_initial_joint_rad(&new_pose);
    sim.core
        .replace_bus(bus)
        .expect("the sim backend configures");

    // Every pass of the bring-up reaches the wire: one lost frame on a
    // real bus is covered only by the configured repeats.
    let burst = |repeats: u8| {
        let mut bus = SimBus::new(common::scene(&b));
        bus.boot_configure(&b.robot, b.active_tool(), repeats)
            .expect("the sim backend configures");
        bus.peak_tx_frames_per_tick()
    };
    let repeats = b.robot.bus.boot_config_repeats;
    assert!(repeats > 1, "a single pass cannot show the repeat count");
    assert_eq!(
        sim.core.bus_mut().peak_tx_frames_per_tick(),
        burst(repeats),
        "the swap must send the configured {repeats} config passes"
    );

    let after = sim.tick_until_idle();
    let moved = after.q[0] - before.q[0];
    assert!(
        (moved - 0.2).abs() < 0.01,
        "J0 moved {moved} rad across the swap: the core is not reading the new arm"
    );

    // The CAN gripper is part of the configured arm: it answers on the
    // new bus, and nothing reads as lost.
    assert!(
        after.gripper.reply.is_some() && after.gripper.data_age_ticks == 0,
        "the gripper answers on the new bus: {:?}",
        after.gripper
    );
    assert!(
        !after
            .errors
            .as_slice()
            .iter()
            .any(|e| e.code == ErrorCode::CanLost),
        "{:?}",
        after.errors
    );
    let expected: Vec<(u8, f32)> = b
        .robot
        .joints
        .iter()
        .map(|j| (j.node_id, j.ilim_ma as f32))
        .collect();
    for (node, ilim) in expected {
        sim.core.bus_mut().queue_poll_override(
            PollAction::ConfigRead {
                node,
                kind: ConfigKind::Limits,
            },
            1,
        );
        for _ in 0..3 {
            sim.tick();
        }
        let s = sim.tick();
        match s.nodes[usize::from(node)].readback(ConfigKind::Limits) {
            Some(Readback::Limits { current_ma, .. }) => assert_eq!(
                current_ma, ilim,
                "node {node} was brought up with another current limit"
            ),
            other => panic!("node {node} holds no configured limits: {other:?}"),
        }
    }
}

const READY: [f64; MAX_JOINTS] = [0.0, -1.5, 3.0, 0.0, 0.0, 3.1];

/// The daemon's simulator swap: a simulator stands in for the arm,
/// starting at the arm's ready pose. The arm comes back for the caller to
/// keep, as the daemon's holder thread does.
fn stand_in(sim: &mut SimCore, b: &par6_config::ConfigBundle) -> HeldArm<SimBus> {
    let mut handed = None;
    sim.core
        .stand_in(SimBus::new(common::scene(b)), |arm| handed = Some(arm))
        .expect("the simulator configures");
    let arm = handed.expect("the arm is handed off");
    sim.core
        .bus_mut()
        .teleport_joint_rad(&READY)
        .expect("sim re-seed");
    sim.core.adopt_landed_pose(&b.robot, &READY);
    arm
}

/// One tick of the core and one period of the holder.
fn tick_both(sim: &mut SimCore, arm: &mut HeldArm<SimBus>) -> par6_rt::StateSnapshot {
    arm.keep();
    sim.tick()
}

/// A simulator standing in does not let go of the arm, and hands it back
/// as it was.
///
/// The bug this is the shape of: the toggle closed the arm's bus, its
/// drives stopped their motors on their watchdogs 100 ms later, and a
/// homed arm holding its ready pose fell (2026-11-10). The arm here is a
/// simulated plant with those watchdogs and its own weight, so frames
/// that stop reaching it drop it the same way.
#[test]
fn an_arm_stood_in_for_is_held_where_it_stands_and_comes_back_as_it_was() {
    let b = bundle();
    let shoulder = |pct: f64| {
        let mut speeds = [0.0; MAX_JOINTS];
        speeds[1] = pct;
        RtCommand::Jog { speeds, accel: 1.0 }
    };
    let secs = |sim: &SimCore, s: f64| (s / sim.dt) as u32;

    for moving in [false, true] {
        let mut sim = SimCore::landed_at(&b, &READY);
        assert_eq!(sim.cmd(RtCommand::SetMode(Mode::Jog)).mode, Mode::Jog);
        sim.cmd(shoulder(0.3));
        for _ in 0..secs(&sim, 0.5) {
            sim.tick();
        }
        if !moving {
            sim.cmd(RtCommand::JogRelease);
            for _ in 0..secs(&sim, 2.0) {
                sim.tick();
            }
            assert_eq!(
                sim.tick().mode,
                Mode::Exec,
                "a released jog rests in a hold"
            );
        }
        let left = sim.tick();
        let stood = sim.core.bus_mut().true_joint_rad();

        let mut arm = stand_in(&mut sim, &b);
        // The simulator is what answers and what moves, for far longer
        // than a drive waits before stopping its motor.
        let mut last = tick_both(&mut sim, &mut arm);
        for _ in 0..100 {
            if last.mode == Mode::Idle {
                break;
            }
            last = tick_both(&mut sim, &mut arm);
        }
        assert!(last.mode == Mode::Idle && last.homed, "{last:?}");
        for cmd in [
            RtCommand::Enable,
            RtCommand::SetMode(Mode::Jog),
            shoulder(0.3),
        ] {
            sim.cmds.send(cmd).expect("command channel");
            tick_both(&mut sim, &mut arm);
        }
        for _ in 0..secs(&sim, 2.0) {
            last = tick_both(&mut sim, &mut arm);
        }
        assert!(
            (last.q[1] - READY[1]).abs() > 0.05,
            "the stand-in is the arm being jogged: {} rad",
            last.q[1]
        );

        sim.core.resume(arm);
        let back = sim.tick();
        let now = sim.core.bus_mut().true_joint_rad();
        // A moving joint is stopped as an e-stop stops it, within the
        // travel that takes; a resting one does not move at all.
        let band = if moving { 0.1 } else { 2e-3 };
        for j in 0..MAX_JOINTS {
            assert!(
                (now[j] - stood[j]).abs() < band,
                "moving={moving}: J{} went {} rad while the simulator stood in",
                j + 1,
                now[j] - stood[j]
            );
            assert!(
                (back.q[j] - left.q[j]).abs() < band,
                "moving={moving}: J{} reads {} rad where it read {}: its reference did \
                 not come back",
                j + 1,
                back.q[j],
                left.q[j]
            );
        }
        assert!(back.homed, "moving={moving}: the arm kept its reference");
        assert_eq!(back.state, ArmState::Enabled, "and comes back enabled");
        assert_eq!(back.mode, Mode::Exec, "resting in a hold");
        if !moving {
            assert_eq!(
                back.q_commanded, left.q_commanded,
                "on the target it was resting on"
            );
        }
        // Still where it stood once the core has been driving it again.
        for _ in 0..secs(&sim, 1.0) {
            sim.tick();
        }
        let settled = sim.tick();
        let rest = sim.core.bus_mut().true_joint_rad();
        for j in 0..MAX_JOINTS {
            assert!(
                (rest[j] - now[j]).abs() < 2e-3,
                "moving={moving}: J{} moved {} rad once the core took the arm back",
                j + 1,
                rest[j] - now[j]
            );
        }
        assert!(
            !settled
                .errors
                .as_slice()
                .iter()
                .any(|e| matches!(e.code, ErrorCode::CanLost | ErrorCode::RangeLost)),
            "{:?}",
            settled.errors
        );
    }
}

/// A held arm whose drives go quiet comes back unreferenced, with the
/// error that says why: a drive that returns may count from another zero,
/// and the position its hold names is no longer where the joint stands.
#[test]
fn a_held_arm_whose_drives_go_quiet_comes_back_unreferenced_and_says_why() {
    let b = bundle();
    let mut sim = SimCore::landed_at(&b, &READY);
    sim.core.bus_mut().set_deaf(true);
    let mut arm = stand_in(&mut sim, &b);
    for _ in 0..b.robot.ticks(b.robot.bus.lost_s) + 2 {
        tick_both(&mut sim, &mut arm);
    }
    // The simulator's own drives answer: nothing of the arm's is on it.
    assert!(sim.tick().errors.as_slice().is_empty());

    // Heard again from here on: what is read next is what the hold found
    // while the arm was quiet, not a silence still going on.
    sim.core.resume(arm);
    sim.core.bus_mut().set_deaf(false);
    let back = sim.tick();
    assert!(!back.homed, "the quiet arm kept its reference");
    assert!(
        back.errors
            .as_slice()
            .iter()
            .any(|e| e.code == ErrorCode::CanLost),
        "{:?}",
        back.errors
    );
}

/// A simulator swapped in under a core that has been running starts at
/// the core's tick.
///
/// The bug this is the shape of: a new simulator counted from zero, and
/// its first tick stepped the plant once for every tick the process had
/// run — 17 s inside one tick after two minutes of uptime, with the loop
/// stopped for all of it (2026-11-10). Unattended for those steps, the
/// plant here sags from the pose it was handed.
#[test]
fn a_simulator_swapped_in_late_does_not_replay_the_ticks_before_it() {
    let b = bundle();
    let mut sim = SimCore::landed_at(&b, &READY);
    for _ in 0..b.robot.ticks(8.0) {
        sim.tick();
    }
    let mut reach = READY;
    reach[1] = -0.3;
    reach[2] = 1.9;
    let mut late = SimBus::new(common::scene(&b));
    late.set_initial_joint_rad(&reach);
    sim.core
        .replace_bus(late)
        .expect("the simulator configures");
    sim.tick();
    let now = sim.core.bus_mut().true_joint_rad();
    // One step takes up the drivetrain's slack, milliradians; the replay
    // let the wrist sag half a radian.
    for j in 0..MAX_JOINTS {
        assert!(
            (now[j] - reach[j]).abs() < 0.05,
            "J{} is {} rad from where the simulator started, one tick in",
            j + 1,
            now[j] - reach[j]
        );
    }
}

/// A held joint that was forced off its target is taken back where it
/// stands. Its hold names the pose it was left in, and resuming on that
/// would throw the joint back across however far it went.
#[test]
fn a_held_joint_forced_off_its_target_is_taken_back_where_it_stands() {
    let b = bundle();
    let mut sim = SimCore::landed_at(&b, &READY);
    assert_eq!(sim.cmd(RtCommand::SetMode(Mode::Jog)).mode, Mode::Jog);
    sim.cmd(RtCommand::JogRelease);
    for _ in 0..b.robot.ticks(1.0) {
        sim.tick();
    }
    let left = sim.tick();
    assert_eq!(left.mode, Mode::Exec, "resting in a position hold");

    // More than the wrist's drive can hold against.
    let wrist = b.robot.joints[4].node_id;
    let overload = 4.0 * b.robot.joints[4].ilim_ma;
    sim.core.bus_mut().set_joint_load_ma(wrist, overload);
    let mut arm = stand_in(&mut sim, &b);
    for _ in 0..b.robot.ticks(1.0) {
        tick_both(&mut sim, &mut arm);
    }

    sim.core.resume(arm);
    let back = sim.tick();
    assert!(
        (back.q[4] - left.q[4]).abs() > 0.1,
        "the load did not move the wrist: {} rad",
        back.q[4] - left.q[4]
    );
    assert!(
        (back.q_commanded[4] - back.q[4]).abs() < 0.02,
        "the wrist stands at {} rad and is commanded to {}",
        back.q[4],
        back.q_commanded[4]
    );
}
