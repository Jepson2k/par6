//! Homing: the full PAR6 sequence driven closed-loop against the sim bus
//! (spec/HOMING.md "Sim requirements"), the mid-homing hard-error abort,
//! and the failure signatures (two-pass mismatch, position-never-valid)
//! from scripted NodeState evolutions at the HomingSystem seam.

mod common;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};

use par6_bus::sim::SimBus;
use par6_bus::spectral::JointConversion;
use par6_bus::{
    BusState, DriverBus, GripperCommand, HallState, JointCommand, LoopbackBus, Pack, TxRecord,
};
use par6_config::{ConfigBundle, HomeGroup, SequenceStep};
use par6_rt::homing::{HomingSystem, SeqStatus};
use par6_rt::hooks::{ClampStream, RampJog};
use par6_rt::{
    sample_ring, ArmState, CompletionPolicy, HomingJointStatus, Mode, NoFk, RtCommand, RtCore,
    RtHandles, RtHooks, SharedFlashMarker, SharedLineGpio, SpecSettle, ZeroGravity, MAX_JOINTS,
};

/// An RtCore over the closed-loop sim bus. J5's hall band is moved onto
/// its approach path: the sim's default band sits at the home offset in
/// the unwrapped joint frame, which the shipped sequence approaches from
/// the other side of the revolution.
fn sim_core() -> (
    RtCore<SimBus>,
    RtHandles,
    mpsc::Sender<RtCommand>,
    Arc<AtomicBool>,
) {
    let bundle = common::bundle();
    let robot = &bundle.robot;
    let dt = robot.robot.tick_dt_s;
    let (tx, rx) = mpsc::channel();
    let (gpio, line) = SharedLineGpio::new(true);
    let (marker, _flash) = SharedFlashMarker::new();
    let (_producer, consumer) = sample_ring(64);
    let hooks = RtHooks {
        gravity: Box::new(ZeroGravity),
        jog: Box::new(RampJog::new(robot)),
        stream: Box::new(ClampStream::new(robot)),
        settle: Box::new(SpecSettle::new(CompletionPolicy::Settled, dt)),
        estop: Box::new(gpio),
        flash: Box::new(marker),
        commands: Box::new(rx),
        fk: Box::new(NoFk),
        samples: consumer,
    };
    let (mut core, handles) = RtCore::new(&bundle, SimBus::new(), hooks).expect("sim core");
    core.bus_mut().set_hall_trigger(5, -0.3, 0.02);
    (core, handles, tx, line)
}

fn start_homing(core: &mut RtCore<SimBus>, handles: &mut RtHandles, tx: &mpsc::Sender<RtCommand>) {
    let dt = core.tick_dt_s();
    for _ in 0..10 {
        core.tick(dt, false);
    }
    assert_eq!(handles.snapshots.latest().mode, Mode::Idle);
    tx.send(RtCommand::Enable).unwrap();
    core.tick(dt, false);
    tx.send(RtCommand::SetMode(Mode::Homing)).unwrap();
    core.tick(dt, false);
    assert_eq!(handles.snapshots.latest().mode, Mode::Homing);
    assert!(handles.snapshots.latest().homing.active);
}

#[test]
fn full_par6_sequence_homes_closed_loop_to_the_ready_pose() {
    let (mut core, mut handles, tx, _line) = sim_core();
    let bundle = common::bundle();
    let dt = core.tick_dt_s();
    start_homing(&mut core, &mut handles, &tx);

    let mut saw_j0_running_at_homing_current = false;
    let mut finished = false;
    for _ in 0..30_000 {
        core.tick(dt, false);
        let s = handles.snapshots.latest();
        if s.homing.active
            && s.homing.per_joint[0] == HomingJointStatus::Running
            && s.homing.effective_current_limit_ma[0]
                == bundle.robot.homing.joints[0].current_ma as f32
        {
            saw_j0_running_at_homing_current = true;
        }
        if !s.homing.active && s.mode == Mode::Idle {
            finished = true;
            break;
        }
    }
    assert!(finished, "sequence must finish within the tick budget");
    assert!(
        saw_j0_running_at_homing_current,
        "effective current limit publishes the homing value while running"
    );

    let s = handles.snapshots.latest();
    assert!(s.homed, "sequence success sets homed");
    assert!(!s.error_active, "no errors from a clean sequence");
    for (i, st) in s.homing.per_joint.iter().enumerate() {
        assert_eq!(*st, HomingJointStatus::Done, "actuator {i} done");
    }
    // Normal current limits published again after completion.
    for i in 0..MAX_JOINTS {
        assert_eq!(
            s.homing.effective_current_limit_ma[i], bundle.robot.joints[i].ilim_ma as f32,
            "J{i} back to the normal Ilim"
        );
    }

    // The ready pose reached through the HOMED references — J3's offset
    // is the config fallback, J4's comes from the ACTIVE gripper (MSG),
    // so a wrong gripper-dependent offset would miss these targets.
    let want = [1.57, -1.85, 2.85, 0.0, -0.5, std::f64::consts::PI];
    for (i, (got, want)) in s.q.iter().zip(&want).enumerate() {
        assert!(
            (got - want).abs() < 0.05,
            "J{i}: measured {got} want {want} after homing"
        );
    }
}

#[test]
fn hard_error_mid_homing_aborts_unhomes_and_zeroes_statuses() {
    let (mut core, mut handles, tx, line) = sim_core();
    let dt = core.tick_dt_s();
    start_homing(&mut core, &mut handles, &tx);

    // Deep into the sequence (step 1 pre-moves + J0 homing running).
    for _ in 0..1500 {
        core.tick(dt, false);
    }
    let s = handles.snapshots.latest();
    assert!(s.homing.active, "still homing");
    assert!(
        s.homing.per_joint.contains(&HomingJointStatus::Running),
        "an FSM is running"
    );

    // Hardware e-stop mid-homing: abort, un-home, zero statuses.
    line.store(false, Ordering::Relaxed);
    for _ in 0..8 {
        core.tick(dt, false);
    }
    let s = handles.snapshots.latest();
    assert_eq!(s.mode, Mode::ActiveError);
    assert_eq!(s.state, ArmState::Disabled);
    assert!(s.error_active);
    assert!(!s.homed, "abort clears homed");
    assert!(!s.homing.active, "sequence aborted");
    for st in &s.homing.per_joint {
        assert_eq!(*st, HomingJointStatus::Idle, "statuses zeroed");
    }
}

// ------------------------------------------------------------------
// Failure signatures via scripted NodeState evolutions (HomingSystem
// seam — the exact surface the core drives every HOMING tick).
// ------------------------------------------------------------------

/// A bundle whose sequence is a single step homing exactly `joint`.
fn single_joint_bundle(joint: u8) -> ConfigBundle {
    let mut bundle = common::bundle();
    bundle.robot.homing.sequence = vec![SequenceStep {
        pre_moves: vec![],
        home: Some(HomeGroup {
            joints: vec![joint],
            gripper: None,
        }),
        move_to: vec![],
        post_moves: vec![],
    }];
    bundle.robot.homing.post_moves = vec![];
    bundle
}

struct HomingHarness {
    sys: HomingSystem,
    bus: LoopbackBus,
    state: BusState,
    conv: [JointConversion; MAX_JOINTS],
    cmds: [JointCommand; MAX_JOINTS],
    gcmd: GripperCommand,
    dt: f64,
}

impl HomingHarness {
    fn new(bundle: &ConfigBundle) -> Self {
        let mut bus = LoopbackBus::new();
        bus.boot_configure(&bundle.robot, bundle.active_gripper(), 1)
            .unwrap();
        bus.tx_log.clear();
        let conv = std::array::from_fn(|i| JointConversion::from_config(&bundle.robot.joints[i]));
        let mut sys = HomingSystem::new(bundle);
        sys.start(&mut bus);
        Self {
            sys,
            bus,
            state: BusState::new(),
            conv,
            cmds: [JointCommand::idle(); MAX_JOINTS],
            gcmd: GripperCommand::NoGripper,
            dt: bundle.robot.robot.tick_dt_s,
        }
    }

    fn tick(&mut self, t: u64) -> SeqStatus {
        self.bus.begin_tick(t);
        self.sys.tick(
            &mut self.bus,
            &self.state,
            &mut self.conv,
            &mut self.cmds,
            &mut self.gcmd,
        )
    }

    fn limits_count(&self, node: u8, current_ma: f32) -> usize {
        self.bus
            .tx_log
            .iter()
            .filter(|(_, r)| {
                matches!(r, TxRecord::Limits { node: n, current_limit_ma, .. }
                    if *n == node && *current_limit_ma == current_ma)
            })
            .count()
    }

    fn config_passes(&self) -> usize {
        self.bus
            .tx_log
            .iter()
            .filter(|(_, r)| matches!(r, TxRecord::ConfigPass { .. }))
            .count()
    }
}

#[test]
fn two_pass_mismatch_fails_the_joint_and_restores_config() {
    let bundle = single_joint_bundle(0);
    let jh = &bundle.robot.homing.joints[0];
    let mut h = HomingHarness::new(&bundle);

    // Entry swap: Limits(normal vel, homing current) ×4 to every arm
    // node and the gripper motor.
    for i in 0..MAX_JOINTS {
        let node = bundle.robot.joints[i].node_id;
        let ma = bundle.robot.homing.joints[i].current_ma as f32;
        assert_eq!(h.limits_count(node, ma), 4, "entry limit swap for J{i}");
    }
    let gripper_ma = bundle
        .active_gripper()
        .unwrap()
        .homing
        .as_ref()
        .unwrap()
        .current_ma as f32;
    assert_eq!(
        h.limits_count(bundle.robot.bus.gripper_node, gripper_ma),
        4,
        "entry limit swap covers the gripper motor"
    );

    // Scripted plant for J0: velocity integrates; a plateau at `stop`
    // with saturated current is the stall signature; the endstop MOVES
    // by more than two_pass_max_diff before the re-approach, so pass 2
    // latches a mismatching position.
    let master = bundle.robot.joints[0].sector_master_position_ticks;
    let mut pos = f64::from(master);
    let mut stop = pos + 3000.0;
    let mut shifted = false;
    let n0 = usize::from(bundle.robot.joints[0].node_id);
    h.state.nodes[n0].position_ticks = Some(master);
    h.state.nodes[n0].current_ma = Some(0);

    let mut outcome = SeqStatus::Running;
    let mut fsm_started = false;
    for t in 1..6000u64 {
        let status = h.tick(t);
        let cmd = h.cmds[0];
        if let Some(v) = cmd.vel {
            if v < 0 && !shifted {
                // First backoff observed: shift the endstop for pass 2.
                stop += f64::from(jh.two_pass_max_diff_ticks) + 500.0;
                shifted = true;
            }
            pos = (pos + f64::from(v) * h.dt).min(stop);
            if v > 0 {
                fsm_started = true;
            }
        }
        let seated = pos >= stop - 0.5 && matches!(cmd.vel, Some(v) if v > 0);
        h.state.nodes[n0].position_ticks = Some(pos as i32);
        h.state.nodes[n0].speed_ticks_s = Some(cmd.vel.unwrap_or(0));
        h.state.nodes[n0].current_ma = Some(if seated { jh.current_ma as i16 } else { 100 });
        match status {
            SeqStatus::Running => {}
            other => {
                outcome = other;
                break;
            }
        }
    }
    assert!(fsm_started, "the approach drove the joint");
    assert!(shifted, "pass 2 ran against a moved endstop");
    assert_eq!(outcome, SeqStatus::Failed, "two-pass mismatch fails");
    assert_eq!(h.sys.statuses()[0], HomingJointStatus::Failed);
    assert!(!h.sys.active());
    // Failure restores every node's full stored config (6 joints + the
    // CAN gripper).
    assert!(
        h.config_passes() > MAX_JOINTS,
        "config reload on failure (got {})",
        h.config_passes()
    );
}

#[test]
fn hall_position_never_valid_at_settle_is_a_failure() {
    let bundle = single_joint_bundle(5);
    let mut h = HomingHarness::new(&bundle);
    let n5 = usize::from(bundle.robot.joints[5].node_id);

    // The node never reports a position (encoder silent); the hall
    // trigger fires well past the pre-clear guard. The vendor marked
    // DONE without a reference here — [OURS] makes it a FAILURE.
    let mut outcome = SeqStatus::Running;
    let mut saw_hall_drive = false;
    for t in 1..2000u64 {
        let status = h.tick(t);
        if let Pack::Hall { trigger_value } = h.cmds[5].pack {
            assert_eq!(trigger_value, 2, "vendor hall trigger value");
            saw_hall_drive = true;
        }
        // Bus liveness: every non-active joint gets an idle keep-alive.
        for (i, c) in h.cmds.iter().enumerate() {
            if i != 5 {
                assert_eq!(*c, JointCommand::idle(), "J{i} keep-alive");
            }
        }
        if t == 250 {
            h.state.nodes[n5].hall = Some(HallState {
                trigger: false,
                pin2: false,
                edge: true,
            });
        }
        match status {
            SeqStatus::Running => {}
            other => {
                outcome = other;
                break;
            }
        }
    }
    assert!(saw_hall_drive, "hall joints drive with the HALL pack");
    assert_eq!(
        outcome,
        SeqStatus::Failed,
        "never-valid position must FAIL, not silently mark done"
    );
    assert_eq!(h.sys.statuses()[5], HomingJointStatus::Failed);
}
