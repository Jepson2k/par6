//! Swapping the bus backend under a running core.
//!
//! The command plane opens the new backend itself — that is where a
//! failure has a client to answer — and hands the core a bus that
//! already exists. What the core owes in return is a clean bring-up:
//! the arm on the other side of the swap is a DIFFERENT arm, and every
//! belief the old one produced has to go with it.

mod common;

use common::{bundle, Rig};
use par6_bus::{LoopbackBus, TxRecord};
use par6_rt::{Mode, RtCommand};

/// Ticks from a bus coming up to its selfcheck, mirroring
/// `par6_rt::core::BOOT_SELFCHECK_TICK` — private there, and a
/// literal here is the point: the test pins the count, so moving it
/// in the core without meaning to fails rather than passes quietly.
const BOOT_SELFCHECK_TICKS: u64 = 8;

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
    let mut rig = Rig::new();
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
    let idle = rig.tick_until(120, |s| s.mode == Mode::Idle);
    assert_eq!(
        idle.tick,
        swapped_at + BOOT_SELFCHECK_TICKS,
        "IDLE is reached on the selfcheck tick, counted from the SWAP \
         (swapped at {swapped_at})"
    );
    assert!(!idle.homed, "and it is still un-homed until it is homed");
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

/// The config the new bus is brought up with is the one the core was
/// built from — a swap is not a chance to reconfigure.
#[test]
fn a_swapped_bus_is_configured_from_the_cores_own_config() {
    let mut rig = Rig::new();
    rig.tick_n(10);
    rig.core
        .replace_bus(LoopbackBus::new())
        .expect("configures");

    let node0 = bundle().robot.joints[0].node_id;
    let passes: Vec<_> = rig
        .core
        .bus_mut()
        .tx_log
        .iter()
        .filter_map(|(_, r)| match r {
            TxRecord::ConfigPass { node } => Some(*node),
            _ => None,
        })
        .collect();
    assert!(
        passes.contains(&node0),
        "the new bus was never config-passed for J0 (node {node0}): {passes:?}"
    );
}
