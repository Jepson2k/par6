//! The config re-push schedule: the boot shots at ticks 50/150/300 run
//! `bus.boot_config_repeats` passes per node per shot, and the FLASHING
//! exit — whose `rebase_freshness` deliberately masks every disconnect
//! edge from the silent window — pushes a full pass immediately and
//! re-arms the same schedule for the drivers that were power-cycled or
//! reflashed during the window.

mod common;

use par6_bus::{DriveTune, TxRecord};
use par6_rt::{Mode, RtCommand, MAX_JOINTS};

/// The scheduled shots, in ticks after arming, from the config.
fn shots() -> Vec<u64> {
    let robot = common::bundle().robot;
    robot
        .bus
        .config_resend_offsets_s
        .iter()
        .map(|s| u64::from(robot.ticks(*s)))
        .collect()
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
    let shots = shots();
    assert!(
        shots.len() > 1,
        "the shipped config schedules more than one shot"
    );
    for shot in shots {
        while rig.snap().tick < shot - 1 {
            rig.tick();
        }
        rig.clear_tx();
        rig.tick();
        assert_eq!(
            rig.config_passes(),
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
        rig.config_passes(),
        nodes * repeats,
        "the exit itself must push every node's stored config"
    );

    // The schedule re-armed at the exit: a full shot fires 50 ticks on.
    let exit_tick = rig.snap().tick;
    let first_shot = shots()[0];
    while rig.snap().tick < exit_tick + first_shot - 1 {
        rig.tick();
    }
    rig.clear_tx();
    rig.tick();
    assert_eq!(
        rig.config_passes(),
        nodes * repeats,
        "the first shot must fire relative to the exit"
    );

    // `bus_booted_at` was NOT reset: the boot selfcheck did not re-run,
    // so the ride to the shot stayed IDLE with no CAN_LOST relatch.
    let s = rig.snap();
    assert_eq!(s.mode, Mode::Idle);
    assert!(!s.error_active, "no selfcheck relatch after the exit");
}

/// `SET_PID_GAINS` through the RT command path: the tune replaces the
/// node's STORED config and goes out as `boot_config_repeats` passes now
/// — so a later resend (reconnect, FLASHING exit) carries the new
/// values, exactly like a boot pass. A node the bus never configured is
/// refused and pushes nothing.
#[test]
fn a_retune_stores_the_tune_and_pushes_it_like_a_boot_shot() {
    let bundle = common::bundle();
    let repeats = bundle.robot.bus.boot_config_repeats as usize;
    let mut rig = common::Rig::new();
    rig.ready();
    rig.clear_tx();

    let tune = DriveTune {
        gains: par6_config::Gains {
            kpp: 9.0,
            kpv: 0.05,
            kiv: 0.005,
            kpiq: 1.2,
            kiiq: 1.0,
            kp: 0.12,
            kd: 0.002,
        },
        ilim_ma: 2200.0,
        velocity_limit_ticks_s: 150_000.0,
        voltage_limit_mv: 0,
    };
    let node = bundle.robot.joints[2].node_id;
    rig.cmd(RtCommand::RetuneNode { node, tune });

    let log = &rig.core.bus_mut().tx_log;
    assert!(
        log.iter().any(
            |(_, r)| matches!(r, TxRecord::Retune { node: n, tune: t } if *n == node && *t == tune)
        ),
        "the stored tune must carry the wire's values verbatim"
    );
    let passes = log
        .iter()
        .filter(|(_, r)| matches!(r, TxRecord::ConfigPass { node: n } if *n == node))
        .count();
    assert_eq!(
        passes, repeats,
        "the push must run boot-shot redundancy, not a single pass"
    );

    // An unconfigured node: refused at the bus, no config traffic.
    rig.clear_tx();
    rig.cmd(RtCommand::RetuneNode { node: 13, tune });
    assert!(
        !rig.core
            .bus_mut()
            .tx_log
            .iter()
            .any(|(_, r)| matches!(r, TxRecord::Retune { .. } | TxRecord::ConfigPass { .. })),
        "a refused retune must push nothing"
    );
}

/// A stored-config shot the bus refuses (TX queue full) is counted and
/// retried until the node's push goes through, instead of leaving that
/// node on firmware defaults with nothing recorded.
#[test]
fn a_refused_config_shot_is_counted_and_retried_until_it_lands() {
    let repeats = common::bundle().robot.bus.boot_config_repeats as usize;
    let node = common::bundle().robot.joints[3].node_id;
    let mut rig = common::Rig::new();
    rig.boot_to_idle();
    rig.core.bus_mut().refuse_config_sends = 1 << node;

    let first_shot = shots()[0];
    while rig.snap().tick < first_shot - 1 {
        rig.tick();
    }
    rig.clear_tx();
    rig.tick();
    assert_eq!(
        rig.config_passes_for(node),
        0,
        "the refused node got nothing"
    );
    assert!(
        rig.snap().loop_stats.config_resend_failures >= 1,
        "the refusal must be counted"
    );

    // The queue drains: the pending node is pushed without waiting for
    // the next scheduled shot.
    rig.core.bus_mut().refuse_config_sends = 0;
    rig.clear_tx();
    rig.tick_n(3);
    assert_eq!(
        rig.config_passes_for(node),
        repeats,
        "the retry must push the full redundancy for the node"
    );
}
