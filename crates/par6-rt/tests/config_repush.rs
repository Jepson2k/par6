//! The config re-push schedule: the boot shots at ticks 50/150/300 run
//! `bus.boot_config_repeats` passes per node per shot, and the FLASHING
//! exit — whose `rebase_freshness` deliberately masks every disconnect
//! edge from the silent window — pushes a full pass immediately and
//! re-arms the same schedule for the drivers that were power-cycled or
//! reflashed during the window.

mod common;

use common::SimCore;
use par6_bus::spectral::codec::Readback;
use par6_bus::{ConfigKind, DriveTune, DriverBus, PollAction};
use par6_rt::{Mode, RtCommand, ZeroGravity};

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

/// The configured nodes: six joints and the CAN gripper motor.
fn nodes() -> Vec<u8> {
    let robot = common::bundle().robot;
    robot
        .joints
        .iter()
        .map(|j| j.node_id)
        .chain([robot.bus.gripper_node])
        .collect()
}

/// Every node gets `boot_config_repeats` passes on every shot: at boot on
/// the 50/150/300 schedule, and again when a FLASHING window ends — the
/// one place a driver can power-cycle or come back reflashed without the
/// freshness clock noticing, since `rebase()` stamps every node seen on
/// exit. The exit pushes at once and re-arms the schedule from itself.
#[test]
fn every_config_shot_pushes_each_node_its_redundancy_at_boot_and_after_flashing() {
    let repeats = common::bundle().robot.bus.boot_config_repeats as usize;
    assert!(repeats > 1, "the shipped config must exercise redundancy");
    let shots = shots();
    assert!(
        shots.len() > 1,
        "the shipped config schedules more than one shot"
    );
    let each_node_got = |rig: &mut common::Rig, what: &str| {
        for node in nodes() {
            assert_eq!(
                rig.config_passes_for(node),
                repeats,
                "{what}: node {node} must get {repeats} passes"
            );
        }
    };

    let mut rig = common::Rig::new();
    rig.boot_to_idle();
    for &shot in &shots {
        while rig.snap().tick < shot - 1 {
            rig.tick();
        }
        rig.clear_tx();
        rig.tick();
        each_node_got(&mut rig, &format!("the boot shot at tick {shot}"));
    }

    rig.cmd(RtCommand::Enable);
    rig.cmd(RtCommand::Disable);
    rig.cmd(RtCommand::AssertParked);
    rig.cmd(RtCommand::SetMode(Mode::Flashing));
    assert_eq!(rig.snap().mode, Mode::Flashing);
    rig.tick_n(5);
    rig.clear_tx();
    rig.cmd(RtCommand::SetMode(Mode::Idle));
    assert_eq!(rig.snap().mode, Mode::Idle);
    each_node_got(&mut rig, "the FLASHING exit");
    let exit_tick = rig.snap().tick;
    while rig.snap().tick < exit_tick + shots[0] - 1 {
        rig.tick();
    }
    rig.clear_tx();
    rig.tick();
    each_node_got(&mut rig, "the first shot after the exit");
    // The boot selfcheck did not re-run: no CAN_LOST relatch on the way.
    let s = rig.snap();
    assert_eq!(s.mode, Mode::Idle);
    assert!(!s.error_active, "no selfcheck relatch after the exit");
}

/// What a drive holds for one of its config frames, read back over the
/// bus.
fn read_back(sim: &mut SimCore, node: u8, kind: ConfigKind) -> Option<Readback> {
    sim.core
        .bus_mut()
        .queue_poll_override(PollAction::ConfigRead { node, kind }, 1);
    for _ in 0..3 {
        sim.tick();
    }
    sim.tick().nodes[usize::from(node)].readback(kind)
}

/// `SET_PID_GAINS` reaches the drive, and replaces the node's STORED
/// config: a later resend — here a FLASHING exit — carries the tune, not
/// the boot values.
#[test]
fn a_retune_is_what_the_drive_holds_and_what_later_resends_carry() {
    let b = common::bundle();
    let node = b.robot.joints[2].node_id;
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
        ilim_ma: 1111.0,
        velocity_limit_ticks_s: 150_000.0,
        voltage_limit_mv: 0,
    };
    assert_ne!(tune.ilim_ma, b.robot.joints[2].ilim_ma, "the tune differs");
    let mut sim = SimCore::new(&b, Box::new(ZeroGravity));
    sim.tick_until_idle();
    sim.cmd(RtCommand::RetuneNode { node, tune });
    let tuned = |sim: &mut SimCore, when: &str| {
        assert_eq!(
            read_back(sim, node, ConfigKind::Limits),
            Some(Readback::Limits {
                velocity_ticks_s: 150_000.0,
                current_ma: 1111.0
            }),
            "{when}"
        );
        assert_eq!(
            read_back(sim, node, ConfigKind::PositionGains),
            Some(Readback::PositionGains { kpp: 9.0 }),
            "{when}"
        );
    };
    tuned(&mut sim, "the drive holds the tune");

    sim.cmd(RtCommand::AssertParked);
    sim.cmd(RtCommand::SetMode(Mode::Flashing));
    sim.cmd(RtCommand::SetMode(Mode::Idle));
    tuned(&mut sim, "the FLASHING exit's resend carries the tune");
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
