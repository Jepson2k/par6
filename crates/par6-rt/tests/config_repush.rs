//! The config re-push schedule: the boot shots at ticks 50/150/300 run
//! `bus.boot_config_repeats` passes per node per shot, and the FLASHING
//! exit — whose `rebase_freshness` deliberately masks every disconnect
//! edge from the silent window — pushes a full pass immediately and
//! re-arms the same schedule for the drivers that were power-cycled or
//! reflashed during the window.

mod common;

use par6_bus::TxRecord;
use par6_rt::{Mode, RtCommand, MAX_JOINTS};

fn config_passes(rig: &mut common::Rig) -> usize {
    rig.core
        .bus_mut()
        .tx_log
        .iter()
        .filter(|(_, r)| matches!(r, TxRecord::ConfigPass { .. }))
        .count()
}

/// One pass per shot leaves a node that lost a frame of that single
/// pass unconfigured until the next shot 100+ ticks later; the config
/// key governs the redundancy (the vendor pushes 4 per shot).
#[test]
fn each_scheduled_boot_shot_pushes_config_repeats_passes_per_node() {
    let repeats = common::bundle().robot.bus.boot_config_repeats as usize;
    assert!(repeats > 1, "the shipped config must exercise redundancy");
    let nodes = MAX_JOINTS + 1; // six joints + the CAN gripper motor
    let mut rig = common::Rig::new();
    rig.boot_to_idle();
    for shot in [50u64, 150, 300] {
        while rig.snap().tick < shot - 1 {
            rig.tick();
        }
        rig.clear_tx();
        rig.tick();
        assert_eq!(
            config_passes(&mut rig),
            nodes * repeats,
            "the shot at tick {shot} must run {repeats} passes per node"
        );
    }
}

/// The silent window is the one place a driver can power-cycle or come
/// back reflashed without the freshness clock noticing — `rebase()`
/// stamps every node SEEN NOW on exit, so the reconnect path never
/// fires. The exit itself must therefore push the stored config, now
/// and again on the 50/150/300 schedule.
#[test]
fn flashing_exit_repushes_config_now_and_rearms_the_schedule() {
    let repeats = common::bundle().robot.bus.boot_config_repeats as usize;
    let nodes = MAX_JOINTS + 1;
    let mut rig = common::Rig::new();
    rig.ready();
    rig.cmd(RtCommand::Disable);
    rig.cmd(RtCommand::AssertParked);
    rig.cmd(RtCommand::SetMode(Mode::Flashing));
    assert_eq!(rig.snap().mode, Mode::Flashing);
    rig.tick_n(5);

    rig.clear_tx();
    rig.cmd(RtCommand::SetMode(Mode::Idle));
    assert_eq!(rig.snap().mode, Mode::Idle);
    assert_eq!(
        config_passes(&mut rig),
        nodes * repeats,
        "the exit itself must push every node's stored config"
    );

    // The schedule re-armed at the exit: a full shot fires 50 ticks on.
    let exit_tick = rig.snap().tick;
    while rig.snap().tick < exit_tick + 49 {
        rig.tick();
    }
    rig.clear_tx();
    rig.tick();
    assert_eq!(
        config_passes(&mut rig),
        nodes * repeats,
        "the 50-tick shot must fire relative to the exit"
    );

    // `bus_booted_at` was NOT reset: the boot selfcheck did not re-run,
    // so the ride to the shot stayed IDLE with no CAN_LOST relatch.
    let s = rig.snap();
    assert_eq!(s.mode, Mode::Idle);
    assert!(!s.error_active, "no selfcheck relatch after the exit");
}
