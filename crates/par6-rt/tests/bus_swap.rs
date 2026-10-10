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
use par6_rt::{ErrorCode, Mode, RtCommand, ZeroGravity};

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
