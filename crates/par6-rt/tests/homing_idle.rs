//! L2 — the `kind = "idle"` pre-move does what it claims.
//!
//! The vendor's `<idle>` pre-move drops the driver to firmware Idle
//! (cmd 12) so the joint hangs limp while a neighbour homes, then keeps
//! encoder feedback flowing with cmd-28 RTR polls (the driver never
//! replies to cmd 12). Before this landed, `PreMove::Idle` emitted the
//! vel-0 keep-alive — byte-identical to what every non-active joint gets
//! anyway — so the config knob was a pure delay. These tests land with
//! the fix and fail against the pre-fix code: the orchestrator test sees
//! keep-alive frames where cmd 12 must be, and the sim test sees the
//! joint held against the load instead of yielding.

mod common;

use par6_bus::sim::SimBus;
use par6_bus::spectral::convert::torque_to_ma_factor;
use par6_bus::spectral::JointConversion;
use par6_bus::{BusState, DriverBus, GripperCommand, JointCommand, LoopbackBus, Pack};
use par6_config::{PreMove, SequenceStep};
use par6_rt::homing::{HomingSystem, SeqStatus};
use par6_rt::MAX_JOINTS;

/// The orchestrator's idle pre-move on the wire: cmd 12 exactly twice
/// (the driver never acks it), then encoder polls for the remainder of
/// the window, while every other joint keeps its vel-0 keep-alive.
#[test]
fn the_idle_pre_move_drops_the_driver_then_polls_the_encoder() {
    let mut bundle = common::bundle();
    bundle.robot.homing.sequence = vec![SequenceStep {
        pre_moves: vec![PreMove::Idle {
            joint: 1,
            duration_s: 0.2,
        }],
        home: None,
        move_to: vec![],
        post_moves: vec![],
    }];
    bundle.robot.homing.post_moves = vec![];
    let dt = bundle.robot.robot.tick_dt_s;
    let dur_ticks = (0.2 / dt).round() as u32;

    let mut bus = LoopbackBus::new();
    bus.boot_configure(&bundle.robot, bundle.active_gripper(), 1)
        .unwrap();
    let mut sys = HomingSystem::new(&bundle);
    sys.start(&mut bus);
    let mut state = BusState::new();
    let mut conv: [JointConversion; MAX_JOINTS] =
        std::array::from_fn(|i| JointConversion::from_config(&bundle.robot.joints[i]));
    let mut cmds = [JointCommand::idle(); MAX_JOINTS];
    let mut gcmd = GripperCommand::NoGripper;

    let mut idle_frames = 0u32;
    let mut polls = 0u32;
    let mut idles_before_polls = true;
    let mut finished = false;
    for t in 1..1000u64 {
        bus.begin_tick(t);
        let status = sys.tick(&mut bus, &mut state, &mut conv, &mut cmds, &mut gcmd);
        match cmds[1].pack {
            Pack::Idle => {
                idle_frames += 1;
                idles_before_polls &= polls == 0;
            }
            Pack::EncoderPoll => polls += 1,
            _ => {}
        }
        for (i, c) in cmds.iter().enumerate() {
            if i != 1 {
                assert_eq!(*c, JointCommand::idle(), "J{i} keeps its keep-alive");
            }
        }
        if status == SeqStatus::Complete {
            finished = true;
            break;
        }
    }
    assert!(finished, "the idle-only sequence completes");
    assert_eq!(idle_frames, 2, "cmd 12 goes out exactly twice");
    assert_eq!(
        polls,
        dur_ticks - 2,
        "encoder polls fill the rest of the idle window"
    );
    assert!(idles_before_polls, "the drop precedes the polls");
}

fn step(bus: &mut SimBus, state: &mut BusState, t: &mut u64, cmds: &[JointCommand; MAX_JOINTS]) {
    *t += 1;
    bus.begin_tick(*t);
    bus.drain_rx(state).expect("drain");
    bus.send_joint_commands(cmds).expect("joint TX");
    bus.send_gripper(&GripperCommand::FirmwarePoll)
        .expect("gripper TX");
}

/// Through the real codec and the closed-loop sim: a loaded joint held
/// by the armed velocity loop stays put; the same joint dropped with
/// cmd 12 has no holding torque of its own and yields to the load (limp
/// — the point of the idle pre-move), while the encoder polls keep its
/// position reported and its freshness green the whole time. The load
/// exceeds the gearbox's holding friction (below it a self-locking
/// drivetrain holds an unpowered joint whatever the driver does) and
/// stays inside the loop's current authority; the base joint is where
/// that window is wide, and gravity plays no part on its vertical axis.
#[test]
fn a_dropped_driver_hangs_limp_under_load_while_polls_keep_it_fresh() {
    let bundle = common::bundle();
    let robot = &bundle.robot;
    let mut bus = SimBus::new(common::scene(&bundle));
    bus.boot_configure(robot, bundle.active_gripper(), 1)
        .unwrap();
    let jc = &robot.joints[0];
    let node1 = jc.node_id;
    let n1 = usize::from(node1);
    let ma_per_nm =
        torque_to_ma_factor(jc.gear_ratio, jc.gear_efficiency, jc.kt_nm_a, jc.dir).abs();
    bus.set_joint_load_ma(node1, 1.5 * robot.sim.holding_friction_nm[0] * ma_per_nm);
    let mut state = BusState::new();
    let mut t = 0u64;

    // Armed hold: the velocity loop's integral winds up against the
    // load; after the transient the position stands still.
    let hold = [JointCommand::idle(); MAX_JOINTS];
    for _ in 0..150 {
        step(&mut bus, &mut state, &mut t, &hold);
    }
    let held_start = i64::from(state.nodes[n1].position_ticks.expect("reported"));
    for _ in 0..200 {
        step(&mut bus, &mut state, &mut t, &hold);
    }
    let held_end = i64::from(state.nodes[n1].position_ticks.unwrap());
    assert!(
        (held_end - held_start).abs() < 200,
        "the armed loop holds against the load (drifted {})",
        held_end - held_start
    );

    // Drop to idle (twice, the pre-move cadence), then poll.
    let mut cmds = hold;
    cmds[0] = JointCommand::drop_to_idle();
    for _ in 0..2 {
        step(&mut bus, &mut state, &mut t, &cmds);
    }
    cmds[0] = JointCommand::encoder_poll();
    let idle_start = i64::from(state.nodes[n1].position_ticks.unwrap());
    let mut max_age = 0u64;
    for _ in 0..200 {
        step(&mut bus, &mut state, &mut t, &cmds);
        max_age = max_age.max(state.nodes[n1].data_age_ticks);
    }
    let idle_end = i64::from(state.nodes[n1].position_ticks.unwrap());
    // The discriminator is the order of magnitude — an armed loop drifts
    // tens of ticks (and the pre-fix vel-0 keep-alive would keep holding
    // here), a limp joint hangs hundreds before its hard stop catches it.
    assert!(
        (idle_end - idle_start).abs() >= 400,
        "an idled driver must yield to the load (moved {})",
        idle_end - idle_start
    );
    assert!(
        max_age <= 2,
        "encoder polls keep the idled node fresh (max age {max_age})"
    );
    assert_eq!(
        state.nodes[n1].data_age_ticks, 0,
        "steady-state polling answers every tick"
    );
}
