//! Homing: the full PAR6 sequence driven closed-loop against the sim
//! bus, the mid-homing hard-error abort,
//! the home reference the hall FSM latches against the sim's own sensor,
//! the failure signatures (two-pass mismatch, position-never-valid,
//! approach timeout), the stall false-positive guards (startup inrush,
//! current-window duty), and the release phase's sign/duration/sample
//! contract — the scripted cases at the HomingSystem seam. The latched
//! reference itself is checked against plant ground truth in
//! homing_reference.rs.

mod common;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};

use par6_bus::sim::SimBus;
use par6_bus::spectral::{trunc_to_wire, JointConversion};
use par6_bus::{
    BusState, DriverBus, GripperCommand, GripperReply, HallState, JointCommand, LoopbackBus, Pack,
    Reply, TxRecord,
};
use par6_config::{ConfigBundle, GripperHomeMode, HomeGroup, SequenceStep};
use par6_rt::homing::{HomingSystem, SeqStatus};
use par6_rt::hooks::{ClampStream, RampJog};
use par6_rt::{
    sample_ring, ArmState, CompletionPolicy, ErrorCode, HomingJointStatus, Mode, NoFk, RtCommand,
    RtCore, RtHandles, RtHooks, SharedDigitalIo, SharedFlashMarker, SharedLineGpio, SpecSettle,
    ZeroGravity, MAX_JOINTS,
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
    let (io, _io_lines) = SharedDigitalIo::new(robot.io.inputs.len(), robot.io.outputs.len());
    let (_producer, consumer) = sample_ring(64);
    let hooks = RtHooks {
        gravity: Box::new(ZeroGravity),
        jog: Box::new(RampJog::new(robot)),
        stream: Box::new(ClampStream::new(robot)),
        settle: Box::new(SpecSettle::new(CompletionPolicy::Settled, dt, robot.motion)),
        estop: Box::new(gpio),
        io: Box::new(io),
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

    // Closed-loop tracking check only: the move_to targets and `s.q`
    // both convert through the JointConversion that set_home re-based
    // moments earlier, so this holds for essentially ANY latched
    // reference — it proves the sequence completes and the position
    // loops track, not that the reference is right. The reference is
    // checked against the sim plant's ground truth in
    // homing_reference.rs.
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
            &mut self.state,
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

#[test]
fn a_joint_still_travelling_on_pass_two_is_not_a_stall() {
    let bundle = single_joint_bundle(0);
    let jh = &bundle.robot.homing.joints[0];
    let eff = bundle
        .effective_home_offset(0)
        .unwrap_or(jh.home_offset_rad);
    let mut h = HomingHarness::new(&bundle);
    let n0 = usize::from(bundle.robot.joints[0].node_id);

    // A loaded J0: it draws its homing current the whole time it is
    // driven (real drivers do at velocity-mode start), and free travel
    // runs at 80 % of the commanded speed. Only the endstop stops it.
    const TRACKING: f64 = 0.8;
    let master = bundle.robot.joints[0].sector_master_position_ticks;
    let mut pos = f64::from(master);
    let stop = pos + 3000.0;
    h.state.nodes[n0].position_ticks = Some(master);
    h.state.nodes[n0].current_ma = Some(0);

    let mut outcome = SeqStatus::Running;
    let mut pass2_ticks = 0u32;
    for t in 1..12_000u64 {
        let status = h.tick(t);
        let v = h.cmds[0].vel.unwrap_or(0);
        // Pass 2 is the only phase commanding the reduced approach speed.
        if v > 0 && f64::from(v) < jh.speed_ticks_s * 0.9 {
            pass2_ticks += 1;
        }
        pos = (pos + f64::from(v) * TRACKING * h.dt).min(stop);
        h.state.nodes[n0].position_ticks = Some(pos as i32);
        h.state.nodes[n0].speed_ticks_s = Some(v);
        h.state.nodes[n0].current_ma = Some(if v != 0 { jh.current_ma as i16 } else { 0 });
        match status {
            SeqStatus::Running => {}
            other => {
                outcome = other;
                break;
            }
        }
    }

    assert_eq!(outcome, SeqStatus::Complete, "the genuine stall completes");
    assert_eq!(h.sys.statuses()[0], HomingJointStatus::Done);
    // Pass 2 re-covers the backoff distance at the rehome speed factor
    // (the tracking factor scales both legs, so it cancels). A gate still
    // sized for pass 1 calls this travel a stall a quarter of the way in.
    let rehome_speed_factor = 0.3;
    let expected = jh.backoff_s / (rehome_speed_factor * h.dt);
    assert!(
        f64::from(pass2_ticks) > 0.8 * expected,
        "pass 2 must travel the backoff distance before it counts as stalled \
         ({pass2_ticks} ticks, expected about {expected:.0})"
    );
    // The reference is the endstop, not wherever a false stall fired.
    let latched = i64::from(h.conv[0].motor_ticks(eff));
    assert!(
        (latched - stop as i64).abs() <= 50,
        "home reference latched at {latched}, endstop at {stop}"
    );
}

// ------------------------------------------------------------------
// Stall false-positive guards (G4): the startup guard and the 60 %
// current-window duty requirement, scripted at the HomingSystem seam.
// ------------------------------------------------------------------

/// Spin-up inrush: real drivers draw saturated current at velocity-mode
/// start while the rotor has not yet moved — a displacement plateau AND
/// high current, the full stall signature. The 0.15 s startup guard is
/// what keeps that from latching the home reference at the start pose.
/// Without the guard this run latches pass 1 at the boot pose, pass 2
/// hits the real endstop 5000 ticks away, and the two-pass check FAILS
/// the sequence — so `Complete` binds the guard.
#[test]
fn spinup_inrush_does_not_false_latch_a_stall() {
    let bundle = single_joint_bundle(0);
    let jh = &bundle.robot.homing.joints[0];
    let eff = bundle
        .effective_home_offset(0)
        .unwrap_or(jh.home_offset_rad);
    let mut h = HomingHarness::new(&bundle);
    let n0 = usize::from(bundle.robot.joints[0].node_id);

    // ~0.14 s of saturated current with the rotor parked (inrush), then
    // normal travel at 80 % tracking and 100 mA to the endstop.
    let inrush_ticks = (0.14 / h.dt).round() as u32;
    const TRACKING: f64 = 0.8;
    let master = bundle.robot.joints[0].sector_master_position_ticks;
    let mut pos = f64::from(master);
    let stop = pos + 5000.0;
    h.state.nodes[n0].position_ticks = Some(master);
    h.state.nodes[n0].current_ma = Some(0);

    let mut outcome = SeqStatus::Running;
    let mut drive_ticks = 0u32;
    let mut pos_at_first_hit: Option<f64> = None;
    for t in 1..20_000u64 {
        let status = h.tick(t);
        let v = h.cmds[0].vel.unwrap_or(0);
        if v > 0 {
            drive_ticks += 1;
        } else if drive_ticks > 0 && pos_at_first_hit.is_none() {
            // First non-approach command after driving = the pass-1 hit.
            pos_at_first_hit = Some(pos);
        }
        let spinning_up = v > 0 && drive_ticks <= inrush_ticks;
        if !spinning_up {
            pos = (pos + f64::from(v) * TRACKING * h.dt).min(stop);
        }
        let seated = pos >= stop - 0.5 && v > 0;
        h.state.nodes[n0].position_ticks = Some(pos as i32);
        h.state.nodes[n0].speed_ticks_s = Some(v);
        h.state.nodes[n0].current_ma = Some(if seated || spinning_up {
            jh.current_ma as i16
        } else {
            100
        });
        match status {
            SeqStatus::Running => {}
            other => {
                outcome = other;
                break;
            }
        }
    }

    assert_eq!(
        outcome,
        SeqStatus::Complete,
        "an inrush-latched pass 1 would fail the two-pass check"
    );
    assert_eq!(h.sys.statuses()[0], HomingJointStatus::Done);
    assert!(
        pos_at_first_hit.expect("pass 1 must hit") >= stop - 200.0,
        "pass 1 hit at {} — the endstop is {stop}, the inrush plateau was {master}",
        pos_at_first_hit.unwrap()
    );
    let latched = i64::from(h.conv[0].motor_ticks(eff));
    assert!(
        (latched - stop as i64).abs() <= 50,
        "home reference latched at {latched}, endstop at {stop}"
    );
}

/// A jammed joint whose current is above threshold only 40 % of the time
/// (an oscillating load) must not read as a stall: the detector demands
/// ≥ 60 % of the window above `0.70 · homing_current`. The same jam under
/// 100 % duty must latch promptly — same displacement plateau, so the
/// duty cycle is the only variable.
#[test]
fn a_forty_percent_current_duty_is_not_a_stall() {
    let bundle = single_joint_bundle(0);
    let jh = &bundle.robot.homing.joints[0];
    let eff = bundle
        .effective_home_offset(0)
        .unwrap_or(jh.home_offset_rad);
    let mut h = HomingHarness::new(&bundle);
    let n0 = usize::from(bundle.robot.joints[0].node_id);

    let duty_ticks = 250u32;
    let master = bundle.robot.joints[0].sector_master_position_ticks;
    let mut pos = f64::from(master);
    let stop = pos + 3000.0;
    h.state.nodes[n0].position_ticks = Some(master);
    h.state.nodes[n0].current_ma = Some(0);

    let mut outcome = SeqStatus::Running;
    let mut jam_ticks = 0u32; // ticks seated during pass 1
    let mut full_duty_from: Option<u64> = None;
    let mut first_hit_tick: Option<u64> = None;
    let mut drive_seen = false;
    for t in 1..20_000u64 {
        let status = h.tick(t);
        let v = h.cmds[0].vel.unwrap_or(0);
        if v > 0 {
            drive_seen = true;
        } else if drive_seen && first_hit_tick.is_none() {
            first_hit_tick = Some(t);
        }
        pos = (pos + f64::from(v) * h.dt).clamp(f64::from(master) - 4000.0, stop);
        let seated = pos >= stop - 0.5 && v > 0;
        let cur = if seated && first_hit_tick.is_none() && jam_ticks < duty_ticks {
            // Pass-1 jam, phase B: 2-on / 3-off — 40 % of the window.
            jam_ticks += 1;
            if jam_ticks == duty_ticks {
                full_duty_from = Some(t + 1);
            }
            if jam_ticks % 5 < 2 {
                jh.current_ma as i16
            } else {
                0
            }
        } else if seated {
            jh.current_ma as i16
        } else {
            100
        };
        h.state.nodes[n0].position_ticks = Some(pos as i32);
        h.state.nodes[n0].speed_ticks_s = Some(v);
        h.state.nodes[n0].current_ma = Some(cur);

        // Through the whole 40 %-duty window the approach must persist.
        if jam_ticks > 0 && jam_ticks <= duty_ticks && first_hit_tick.is_none() {
            assert_eq!(status, SeqStatus::Running);
            assert_eq!(
                h.cmds[0].vel,
                Some(jh.speed_ticks_s as i32),
                "tick {t}: 40 % duty latched a stall {jam_ticks} jam ticks in"
            );
        }
        match status {
            SeqStatus::Running => {}
            other => {
                outcome = other;
                break;
            }
        }
    }

    let full_from = full_duty_from.expect("the jam must reach the duty window");
    let hit = first_hit_tick.expect("100 % duty must latch");
    assert!(
        hit >= full_from,
        "stall latched at tick {hit}, before full duty began at {full_from}"
    );
    assert!(
        hit <= full_from + 60,
        "100 % duty should latch within a detection window (hit {hit}, full duty from {full_from})"
    );
    assert_eq!(outcome, SeqStatus::Complete);
    assert_eq!(h.sys.statuses()[0], HomingJointStatus::Done);
    let latched = i64::from(h.conv[0].motor_ticks(eff));
    assert!(
        (latched - stop as i64).abs() <= 50,
        "home reference latched at {latched}, endstop at {stop}"
    );
}

// ------------------------------------------------------------------
// Approach timeout (G7): the only guard against a joint driving forever.
// ------------------------------------------------------------------

/// A free-running joint (detached endstop: normal travel, low current,
/// nothing ever stalls) must fail at exactly `round(timeout_s / dt)`
/// approach ticks — not before — with the joint marked Failed and the
/// full node config (normal current limits included) resent. Run at two
/// tick rates so the seconds→ticks conversion is pinned, not an
/// accident of the shipped dt.
#[test]
fn a_free_running_approach_fails_at_the_configured_timeout_exactly() {
    for dt in [0.004, 0.01] {
        let mut bundle = single_joint_bundle(0);
        bundle.robot.robot.tick_dt_s = dt;
        let timeout_ticks = (bundle.robot.homing.joints[0].timeout_s / dt).round() as u64;
        let mut h = HomingHarness::new(&bundle);
        let n0 = usize::from(bundle.robot.joints[0].node_id);

        let master = bundle.robot.joints[0].sector_master_position_ticks;
        let mut pos = f64::from(master);
        h.state.nodes[n0].position_ticks = Some(master);
        h.state.nodes[n0].current_ma = Some(0);

        let mut first_drive: Option<u64> = None;
        let mut failed_at: Option<u64> = None;
        for t in 1..=timeout_ticks + 10 {
            let status = h.tick(t);
            let v = h.cmds[0].vel.unwrap_or(0);
            if v > 0 && first_drive.is_none() {
                first_drive = Some(t);
            }
            // Perfect free travel, telemetry current well below threshold.
            pos += f64::from(v) * dt;
            h.state.nodes[n0].position_ticks = Some(pos as i32);
            h.state.nodes[n0].speed_ticks_s = Some(v);
            h.state.nodes[n0].current_ma = Some(50);
            match status {
                SeqStatus::Running => {}
                SeqStatus::Failed => {
                    failed_at = Some(t);
                    break;
                }
                other => panic!("dt {dt}: unexpected status {other:?} at tick {t}"),
            }
        }

        let first_drive = first_drive.expect("the approach must drive");
        let failed_at = failed_at.unwrap_or_else(|| panic!("dt {dt}: timeout never fired"));
        // elapsed == timeout is still within budget; the tick after is
        // the failure — exactly `round(timeout_s / dt)` driven ticks.
        // 13.0 is the shipped J0 timeout (config/PAR6.toml), spelled out
        // so the conversion is pinned against the config seconds.
        assert_eq!(timeout_ticks, (13.0f64 / dt).round() as u64, "dt {dt}");
        assert_eq!(
            failed_at - first_drive,
            timeout_ticks,
            "dt {dt}: timeout fired after the wrong number of approach ticks"
        );
        assert_eq!(h.sys.statuses()[0], HomingJointStatus::Failed, "dt {dt}");
        assert!(!h.sys.active(), "dt {dt}: sequence stopped");
        assert!(
            h.config_passes() > MAX_JOINTS,
            "dt {dt}: failure must resend every node's stored config (got {})",
            h.config_passes()
        );
    }
}

// ------------------------------------------------------------------
// Release phase (G8): current sign, duration, and the sample tick.
// ------------------------------------------------------------------

/// Scripted release-phase plant for J1: seat against the endstop through
/// both passes, then apply sign-sensitive release physics — positive
/// current moves the motor positive (away from the low stop, relaxing
/// the wound gearbox), negative current presses further in. Returns
/// (release frames sent, position exposed at each release tick, latched
/// reference, outcome).
fn run_release_scenario(bundle: &ConfigBundle) -> (Vec<i16>, Vec<i32>, i64, SeqStatus) {
    let jh = &bundle.robot.homing.joints[1];
    let eff = bundle
        .effective_home_offset(1)
        .unwrap_or(jh.home_offset_rad);
    let mut h = HomingHarness::new(bundle);
    let n1 = usize::from(bundle.robot.joints[1].node_id);

    const TRACKING: f64 = 0.8;
    /// Windup relax/wind rate under release current \[ticks per tick\].
    const RELEASE_STEP: f64 = 3.0;
    let master = bundle.robot.joints[1].sector_master_position_ticks;
    let mut pos = f64::from(master);
    let stop = pos - 3000.0; // J1 approaches with negative motor speed
    h.state.nodes[n1].position_ticks = Some(master);
    h.state.nodes[n1].current_ma = Some(0);

    let mut release_cmds: Vec<i16> = Vec::new();
    let mut release_seen: Vec<i32> = Vec::new();
    let mut outcome = SeqStatus::Running;
    for t in 1..30_000u64 {
        let exposed = h.state.nodes[n1].position_ticks.unwrap();
        let status = h.tick(t);
        let cmd = h.cmds[1];
        if cmd.pos.is_none() && cmd.vel.is_none() {
            // Current-only frame (cmd 2 DLC 2) — the release drive.
            let c = cmd.cur_ma.expect("current-only frame carries current");
            release_cmds.push(c);
            release_seen.push(exposed);
            // Sign-sensitive plant: the current's sign decides whether
            // the gearbox relaxes (away from the stop) or winds tighter.
            pos += RELEASE_STEP * f64::from(c.signum());
        } else {
            let v = cmd.vel.unwrap_or(0);
            pos = (pos + f64::from(v) * TRACKING * h.dt).max(stop);
            let seated = pos <= stop + 0.5 && v < 0;
            h.state.nodes[n1].current_ma = Some(if seated { -(jh.current_ma as i16) } else { 100 });
            h.state.nodes[n1].speed_ticks_s = Some(v);
        }
        h.state.nodes[n1].position_ticks = Some(pos as i32);
        match status {
            SeqStatus::Running => {}
            other => {
                outcome = other;
                break;
            }
        }
    }
    let latched = i64::from(h.conv[1].motor_ticks(eff));
    (release_cmds, release_seen, latched, outcome)
}

#[test]
fn release_commands_the_config_sign_and_duration_and_samples_at_eighty_percent() {
    let bundle = single_joint_bundle(1);
    let r = bundle.robot.homing.joints[1]
        .release
        .expect("J1 ships a release plan");
    let dt = bundle.robot.robot.tick_dt_s;
    let dur_ticks = (r.duration_s / dt).round().max(1.0) as usize;
    let sample_tick = ((dur_ticks as f64 * r.sample_pct).round() as usize).clamp(1, dur_ticks);

    let (cmds, seen, latched, outcome) = run_release_scenario(&bundle);
    assert_eq!(outcome, SeqStatus::Complete);
    // Exactly `duration_s` worth of current-only frames, every one with
    // the CONFIG sign (+150 mA for J1: away from the stop).
    assert_eq!(cmds.len(), dur_ticks, "release runs for round(duration/dt)");
    assert!(
        cmds.iter().all(|&c| c == r.current_ma as i16),
        "every release frame carries the config current verbatim: {cmds:?}"
    );
    // The reference is the position the joint had relaxed to at the
    // sample tick — the scripted plant moves a distinct 3 ticks per
    // release tick, so the latched value identifies the tick exactly.
    assert_eq!(
        latched,
        i64::from(seen[sample_tick - 1]),
        "reference sampled at round(dur · sample_pct) = tick {sample_tick}"
    );
    // And it is the RELAXED position: well away from the seated stop in
    // the releasing direction.
    let stop = i64::from(bundle.robot.joints[1].sector_master_position_ticks) - 3000;
    assert!(
        latched - stop >= 500,
        "latched {latched} must sit relaxed above the stop {stop}"
    );

    // Inverting the config sign must move the reference the other way —
    // the plant winds tighter instead of relaxing, and the relaxed-side
    // assertion above would reject it. This pins that the test (and the
    // FSM) are sign-sensitive, not |current|-sensitive.
    let mut inverted = single_joint_bundle(1);
    let rel = inverted.robot.homing.joints[1]
        .release
        .as_mut()
        .expect("J1 ships a release plan");
    rel.current_ma = -rel.current_ma;
    let (inv_cmds, _, inv_latched, inv_outcome) = run_release_scenario(&inverted);
    assert_eq!(inv_outcome, SeqStatus::Complete);
    assert!(
        inv_cmds.iter().all(|&c| c == -(r.current_ma as i16)),
        "the FSM forwards the inverted sign verbatim"
    );
    assert!(
        inv_latched - stop <= -400,
        "inverted release must latch WOUND-IN ({inv_latched} vs stop {stop}) — \
         the relaxed-side assertion would fail on it"
    );
}

// ------------------------------------------------------------------
// Cached-reply regressions, driven against the closed-loop sim bus so
// the hall bits come from its own sensor emulation and cmd-32 replies.
// ------------------------------------------------------------------

/// The homing subsystem in the RT tick order — drain → FSM → send — over
/// the sim bus, plus a way to drive one joint outside the sequence.
struct SimHomingHarness {
    sys: HomingSystem,
    bus: SimBus,
    state: BusState,
    conv: [JointConversion; MAX_JOINTS],
    cmds: [JointCommand; MAX_JOINTS],
    gcmd: GripperCommand,
    t: u64,
}

impl SimHomingHarness {
    /// Boots the sim at `q0` with joint `joint`'s hall band at
    /// `center ± half` \[rad\].
    fn new(
        bundle: &ConfigBundle,
        q0: &[f64; MAX_JOINTS],
        joint: usize,
        center: f64,
        half: f64,
    ) -> Self {
        let mut bus = SimBus::new();
        bus.set_initial_joint_rad(q0);
        bus.boot_configure(&bundle.robot, bundle.active_gripper(), 1)
            .expect("sim boot");
        bus.set_hall_trigger(joint, center, half);
        Self {
            sys: HomingSystem::new(bundle),
            bus,
            state: BusState::new(),
            conv: std::array::from_fn(|i| JointConversion::from_config(&bundle.robot.joints[i])),
            cmds: [JointCommand::idle(); MAX_JOINTS],
            gcmd: GripperCommand::FirmwarePoll,
            t: 0,
        }
    }

    fn drain(&mut self) {
        self.t += 1;
        self.bus.begin_tick(self.t);
        self.bus.drain_rx(&mut self.state).expect("drain");
    }

    fn send(&mut self) {
        self.bus.send_joint_commands(&self.cmds).expect("joint TX");
        self.bus.send_gripper(&self.gcmd).expect("gripper TX");
    }

    fn tick(&mut self) -> SeqStatus {
        self.drain();
        let status = self.sys.tick(
            &mut self.bus,
            &mut self.state,
            &mut self.conv,
            &mut self.cmds,
            &mut self.gcmd,
        );
        self.send();
        status
    }

    /// Run the sequence to its terminal status.
    fn run(&mut self, budget: u32) -> SeqStatus {
        for _ in 0..budget {
            match self.tick() {
                SeqStatus::Running => {}
                other => return other,
            }
        }
        panic!("the sequence did not finish within {budget} ticks");
    }

    /// One tick outside the sequence driving `joint` with `cmd` — the
    /// test's own motion source; every other joint keeps its keep-alive.
    fn drive(&mut self, joint: usize, cmd: JointCommand) {
        self.drain();
        self.cmds = [JointCommand::idle(); MAX_JOINTS];
        self.cmds[joint] = cmd;
        self.send();
    }

    /// Where the sim's hall sensor physically is, in wire ticks: drive
    /// the joint along the approach direction with the HALL pack until
    /// the driver answers in-band, and take the position it latched AT
    /// the trigger. The cached reading goes first, for the same reason
    /// the FSM drops its own: it predates the question.
    fn sensor_ticks(&mut self, joint: usize, node: usize, speed: f64) -> i32 {
        self.state.nodes[node].hall = None;
        for _ in 0..4000 {
            self.drive(joint, JointCommand::hall(trunc_to_wire(speed), 2));
            if let Some(hall) = self.state.nodes[node].hall {
                if !hall.trigger {
                    return self.state.nodes[node]
                        .position_ticks
                        .expect("a hall reply carries a position");
                }
            }
        }
        panic!("the sim's hall sensor was never reached");
    }
}

#[test]
fn hall_homing_latches_at_the_sensor_not_on_a_cached_trigger() {
    let bundle = single_joint_bundle(5);
    let jh = &bundle.robot.homing.joints[5];
    let eff = bundle
        .effective_home_offset(5)
        .unwrap_or(jh.home_offset_rad);
    let n5 = usize::from(bundle.robot.joints[5].node_id);
    let q0: [f64; MAX_JOINTS] =
        std::array::from_fn(|i| bundle.robot.joints[i].sector_home_offset_rad);

    // (a) J5 boots ON its sensor — the case the pre-clear guard exists
    // for. The trigger it fires on tick 1 must not survive the backoff.
    let mut h = SimHomingHarness::new(&bundle, &q0, 5, q0[5], 0.02);
    h.sys.start(&mut h.bus);
    assert_eq!(h.run(6000), SeqStatus::Complete, "first home completes");

    let sensor = h.sensor_ticks(5, n5, jh.speed_ticks_s);
    let sensor_rad = h.conv[5].joint_rad(sensor);
    assert!(
        (sensor_rad - eff).abs() < 0.01,
        "the sensor must read as the home offset: {sensor_rad} vs {eff}"
    );
    let first_ref = i64::from(h.conv[5].motor_ticks(eff));

    // (b) A second home() in the same process, the normal bring-up case:
    // the reply from run 1 is still the node's `hall` while the joint is
    // parked well clear of the sensor.
    for _ in 0..250 {
        h.drive(
            5,
            JointCommand::velocity(trunc_to_wire(-jh.speed_ticks_s), 0),
        );
    }
    assert!(
        matches!(h.state.nodes[n5].hall, Some(hall) if !hall.trigger),
        "the parked joint still carries run 1's trigger"
    );
    h.sys.start(&mut h.bus);
    assert_eq!(h.run(6000), SeqStatus::Complete, "second home completes");

    let second_ref = i64::from(h.conv[5].motor_ticks(eff));
    assert!(
        // One approach tick of travel is 48 ticks; the cached-hit failure
        // is the whole park distance, ~12 000.
        (second_ref - first_ref).abs() <= 150,
        "both homes must reference the same sensor: {first_ref} then {second_ref}"
    );
}

// ------------------------------------------------------------------
// Gripper calibration failure: recoverable without a restart.
// ------------------------------------------------------------------

/// A bundle whose whole sequence is the firmware gripper calibration.
fn gripper_cal_bundle() -> ConfigBundle {
    let mut bundle = common::bundle();
    bundle.robot.homing.sequence = vec![SequenceStep {
        pre_moves: vec![],
        home: Some(HomeGroup {
            joints: vec![],
            gripper: Some(GripperHomeMode::Firmware),
        }),
        move_to: vec![],
        post_moves: vec![],
    }];
    bundle.robot.homing.post_moves = vec![];
    bundle
}

/// A live bus whose gripper never reports `calibrated` — jaws jammed,
/// gripper unpowered, or the wrong node id on a first bring-up.
fn tick_uncalibrated(rig: &mut common::Rig, n: u32) {
    for _ in 0..n {
        for i in 0..MAX_JOINTS {
            let node = rig.node_of[i];
            let position_ticks = rig.conv[i].motor_ticks(rig.pose[i]);
            rig.core.bus_mut().inject(
                false,
                Reply::Motion {
                    node,
                    position_ticks,
                    speed_ticks_s: 0,
                    current_ma: 0,
                },
            );
        }
        rig.core.bus_mut().inject(
            false,
            Reply::Gripper {
                reply: GripperReply {
                    calibrated: false,
                    ..GripperReply::default()
                },
            },
        );
        rig.tick();
    }
}

#[test]
fn a_gripper_calibration_timeout_clears_without_a_restart() {
    let mut rig = common::Rig::build_bundle(
        gripper_cal_bundle(),
        CompletionPolicy::Settled,
        Box::new(ZeroGravity),
        true,
    );
    rig.auto_inject = false;
    tick_uncalibrated(&mut rig, 10);
    assert_eq!(rig.snap().mode, Mode::Idle, "boot one-shot reaches IDLE");
    rig.send(RtCommand::Enable);
    tick_uncalibrated(&mut rig, 1);
    rig.send(RtCommand::SetMode(Mode::Homing));
    tick_uncalibrated(&mut rig, 1);
    assert_eq!(rig.snap().mode, Mode::Homing);

    // cmd 62 goes out, the calibrated bit never comes back: after the
    // 10 s calibrate timeout, the sequence fails and the
    // hard key latches on the next error pass.
    let timeout_ticks = (10.0 / rig.dt).round() as u32;
    tick_uncalibrated(&mut rig, timeout_ticks + 40);
    let s = rig.snap();
    assert_eq!(s.mode, Mode::ActiveError, "the calibration failure reacts");
    assert!(!s.homed);
    assert!(
        s.errors
            .as_slice()
            .iter()
            .any(|e| e.code == ErrorCode::GripperCalibrationFailed),
        "the operator gets a key naming the failure"
    );

    // Clear Errors must actually clear it — the flag behind the key is
    // re-read every tick, and the key it re-latches gates the HOMING
    // entry that is the only other way to reset the flag.
    rig.send(RtCommand::ClearErrors);
    tick_uncalibrated(&mut rig, 80);
    let s = rig.snap();
    assert!(!s.error_active, "the latch stays wiped after the settle");
    assert_eq!(s.mode, Mode::Idle, "ACTIVE_ERROR auto-recovers");

    // ... and the runtime is homable again, not restart-only.
    rig.send(RtCommand::Enable);
    tick_uncalibrated(&mut rig, 1);
    rig.send(RtCommand::SetMode(Mode::Homing));
    tick_uncalibrated(&mut rig, 1);
    let s = rig.snap();
    assert_eq!(s.state, ArmState::Enabled, "enable is granted again");
    assert_eq!(s.mode, Mode::Homing, "homing can be retried after a clear");
}
