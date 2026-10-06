//! Homing: the full PAR6 sequence driven closed-loop against the sim
//! bus, the mid-homing hard-error abort,
//! the home reference the hall FSM latches against the sim's own sensor,
//! the failure signatures (two-pass mismatch, position-never-valid,
//! approach timeout), the stall false-positive guards (startup inrush,
//! current-window duty, pass-2 travel), and the release phase's sign/duration/sample
//! contract — the scripted cases at the HomingSystem seam. The latched
//! reference itself is checked against plant ground truth in
//! homing_reference.rs.

mod common;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};

use par6_bus::sim::SimBus;
use par6_bus::spectral::codec::Readback;
use par6_bus::spectral::{trunc_to_wire, JointConversion};
use par6_bus::{
    BusState, ConfigKind, DriverBus, GripperCommand, GripperReply, HallState, JointCommand,
    LoopbackBus, NodeState, Pack, PollAction, Reply, TxRecord,
};
use par6_config::{ConfigBundle, GripperHomeMode, HomeGroup, MoveTo, SequenceStep};
use par6_rt::adapters::{MotionJog, MotionStream};
use par6_rt::homing::{HomingSystem, SeqStatus, DETECT_WINDOW_S, REHOME_SPEED_FACTOR};
use par6_rt::hooks::ClampStream;
use par6_rt::{
    sample_ring, ArmState, CompletionPolicy, ErrorCode, GravityModel, HomingJointStatus,
    HomingPhase, Mode, NoFk, RtCommand, RtCore, RtHandles, RtHooks, SharedDigitalIo,
    SharedFlashMarker, SharedLineGpio, SpecSettle, ZeroGravity, MAX_JOINTS,
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
    sim_core_with_bundle(&common::bundle())
}

fn sim_core_with_bundle(
    bundle: &ConfigBundle,
) -> (
    RtCore<SimBus>,
    RtHandles,
    mpsc::Sender<RtCommand>,
    Arc<AtomicBool>,
) {
    sim_core_with_gravity(bundle, Box::new(ZeroGravity))
}

fn sim_core_with_gravity(
    bundle: &ConfigBundle,
    gravity: Box<dyn GravityModel>,
) -> (
    RtCore<SimBus>,
    RtHandles,
    mpsc::Sender<RtCommand>,
    Arc<AtomicBool>,
) {
    let robot = &bundle.robot;
    let dt = robot.robot.tick_dt_s;
    let (tx, rx) = mpsc::channel();
    let (gpio, line) = SharedLineGpio::new(true);
    let (marker, _flash) = SharedFlashMarker::new();
    let (io, _io_lines) = SharedDigitalIo::new(robot.io.inputs.len(), robot.io.outputs.len());
    let (_producer, consumer) = sample_ring(64);
    let hooks = RtHooks {
        gravity,
        jog: Box::new(MotionJog::from_config(robot).expect("jog engine")),
        stream: Box::new(MotionStream::from_config(robot).expect("stream limiter")),
        stream_shaped: Box::new(ClampStream::new(robot)),
        settle: Box::new(SpecSettle::new(CompletionPolicy::Settled, dt, robot.motion)),
        estop: Box::new(gpio),
        io: Box::new(io),
        flash: Box::new(marker),
        commands: Box::new(rx),
        fk: Box::new(NoFk),
        samples: consumer,
    };
    let (mut core, handles) =
        RtCore::new(bundle, SimBus::new(common::scene(bundle)), hooks).expect("sim core");
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

/// A tool changed under a running core homes as that tool, exactly as one
/// booted with it does: the same sequence (no gripper work for a tool with
/// no CAN driver) and the same tool-dependent J4 reference, so the arm ends
/// where it would have.
#[test]
fn a_home_after_a_tool_change_homes_as_the_tool_now_fitted() {
    let driven = common::bundle();
    assert!(
        driven.active_tool().is_some_and(|t| t.driver.is_some()),
        "the premise: the shipped tool is driven"
    );
    let mut flanged = driven.clone();
    flanged.robot.robot.active_tool = "Flange".to_owned();
    let flange = flanged.active_tool().expect("the flange").clone();
    assert_ne!(
        driven.effective_home_offset(4),
        flanged.effective_home_offset(4),
        "the premise: the two tools reference J4 differently"
    );

    // How far a home gets through its plan, and where J4 truly ends.
    let home =
        |core: &mut RtCore<SimBus>, handles: &mut RtHandles, tx: &mpsc::Sender<RtCommand>| {
            let dt = core.tick_dt_s();
            start_homing(core, handles, tx);
            let mut steps = 0;
            for _ in 0..(200.0 / dt) as usize {
                core.tick(dt, false);
                let s = handles.snapshots.latest();
                steps = steps.max(s.homing.sequence_step);
                if !s.homing.active {
                    assert!(s.homed, "the home failed: {:?}", s.homing);
                    return (steps, core.bus_mut().true_joint_rad()[4]);
                }
            }
            panic!("the home never finished");
        };
    let (mut core, mut handles, tx, _line) = sim_core_with_bundle(&flanged);
    let booted = home(&mut core, &mut handles, &tx);

    let (mut core, mut handles, tx, _line) = sim_core_with_bundle(&driven);
    for _ in 0..10 {
        core.tick(core.tick_dt_s(), false);
    }
    let before = handles.snapshots.latest().q[4];
    core.set_gripper_tool(Some(&flange), driven.robot.bus.gripper_node, 1);
    core.set_tool_home_offset(4, flanged.effective_home_offset(4).expect("J4 offset"));
    core.tick(core.tick_dt_s(), false);
    assert_eq!(
        handles.snapshots.latest().q[4],
        before,
        "the encoder reads the same angle whatever tool is bolted on"
    );
    let changed = home(&mut core, &mut handles, &tx);

    assert_eq!(
        changed.0, booted.0,
        "the changed tool ran another tool's sequence"
    );
    assert!(
        (changed.1 - booted.1).abs() < 0.01,
        "J4 ends at {:.4} rad after the change, {:.4} rad booted with the flange",
        changed.1,
        booted.1
    );
}

/// A driven tool taken off leaves nothing reading on its node, whichever
/// tick the change lands on: a reply in flight as the jaw came off is not
/// a reading of anything on the arm, and with no poll after it would read
/// on forever.
#[test]
fn a_removed_jaw_leaves_no_reading_behind() {
    let driven = common::bundle();
    let jaw = driven.active_tool().expect("the shipped tool").clone();
    assert!(
        jaw.driver.is_some(),
        "the premise: the shipped tool is driven"
    );
    let mut flanged = driven.clone();
    flanged.robot.robot.active_tool = "Flange".to_owned();
    let flange = flanged.active_tool().expect("the flange").clone();
    let gnode = driven.robot.bus.gripper_node;
    let (mut core, handles, _tx, _line) = sim_core_with_bundle(&driven);
    let dt = core.tick_dt_s();
    let mut reads = handles.snapshots;
    let mut tick = |core: &mut RtCore<SimBus>| {
        core.tick(dt, false);
        reads.latest().nodes
    };
    // Every phase of the telemetry round, so one of them catches a reply
    // in flight.
    for phase in 0..2 * par6_bus::MAX_NODES {
        core.set_gripper_tool(Some(&jaw), gnode, 1);
        let reported = (0..500).any(|_| tick(&mut core)[MAX_JOINTS].temperature_c.is_some());
        assert!(reported, "the fitted jaw never reported a temperature");
        for _ in 0..phase {
            tick(&mut core);
        }
        core.set_gripper_tool(Some(&flange), gnode, 1);
        for _ in 0..50 {
            tick(&mut core);
        }
        let nodes = tick(&mut core);
        let left = nodes[MAX_JOINTS];
        assert!(
            left.temperature_c.is_none()
                && left.current_ma.is_none()
                && left.voltage_mv.is_none()
                && left.error_flags.is_none(),
            "phase {phase}: the removed jaw still reads {left:?}"
        );
        assert!(
            nodes[0].temperature_c.is_some(),
            "phase {phase}: the arm's drives stopped reporting"
        );
    }
}

#[test]
fn shoulder_reference_finishes_before_the_base_seek() {
    fn shoulder_done_when_base_starts(bundle: &ConfigBundle) -> bool {
        let (mut core, mut handles, tx, _line) = sim_core_with_bundle(bundle);
        let dt = core.tick_dt_s();
        start_homing(&mut core, &mut handles, &tx);
        for _ in 0..30_000 {
            core.tick(dt, false);
            let s = handles.snapshots.latest();
            assert!(!s.error_active, "homing failed before the base seek");
            if s.homing.per_joint[0] == HomingJointStatus::Running {
                return s.homing.per_joint[1] == HomingJointStatus::Done;
            }
        }
        panic!("homing never reached the base seek");
    }
    let bundle = common::bundle();
    assert!(shoulder_done_when_base_starts(&bundle));

    // Negative control: the previous base-first sequence reaches J1 while
    // the shoulder is still unreferenced, even when its home later succeeds.
    let mut base_first = bundle.clone();
    base_first.robot.homing.sequence[0].home = Some(HomeGroup {
        joints: vec![0],
        gripper: None,
    });
    base_first.robot.homing.sequence[1].home = Some(HomeGroup {
        joints: vec![1, 2],
        gripper: None,
    });
    assert!(!shoulder_done_when_base_starts(&base_first));
}

/// The whole-sequence deadline is the plan's own worst case, so it never
/// cuts a slow but healthy home short: step 0 nudges J0, step 1 seeks J5
/// toward a sensor it never reaches on a seek budget longer than any fixed
/// deadline sized on a typical run, and the run ends on that budget, after
/// both steps' time. A short run aborted first proves each run starts
/// afresh.
#[test]
fn the_sequence_deadline_leaves_every_step_its_own_budget() {
    const NUDGE_S: f64 = 10.0;
    const SEEK_S: f64 = 100.0;
    let mut bundle = common::bundle();
    bundle.robot.homing.sequence = vec![
        SequenceStep {
            pre_moves: vec![par6_config::PreMove::Nudge {
                joint: 0,
                speed_ticks_s: 500.0,
                duration_s: NUDGE_S,
            }],
            home: None,
            move_to: vec![],
            post_moves: vec![],
        },
        SequenceStep {
            pre_moves: vec![],
            home: Some(HomeGroup {
                joints: vec![5],
                gripper: None,
            }),
            move_to: vec![],
            post_moves: vec![],
        },
    ];
    bundle.robot.homing.post_moves.clear();
    bundle.robot.homing.joints[5].timeout_s = SEEK_S;
    let (mut core, mut handles, tx, _line) = sim_core_with_bundle(&bundle);
    core.bus_mut().set_hall_trigger(5, 3.0, 0.0);
    let dt = core.tick_dt_s();

    start_homing(&mut core, &mut handles, &tx);
    for _ in 0..(1.0 / dt).round() as usize {
        core.tick(dt, false);
    }
    tx.send(RtCommand::ExecStop).unwrap();
    core.tick(dt, false);
    assert!(!handles.snapshots.latest().homing.active, "the stop aborts");

    start_homing(&mut core, &mut handles, &tx);
    let started = handles.snapshots.latest().tick;
    while handles.snapshots.latest().homing.active {
        core.tick(dt, false);
    }
    let s = handles.snapshots.latest();
    let ran_s = (s.tick - started) as f64 * dt;
    assert!(
        ran_s >= NUDGE_S + SEEK_S - 1.0,
        "the sequence was cut short {ran_s:.1} s in, before J5's own {SEEK_S} s seek ran out"
    );
    assert_eq!(s.homing.per_joint[5], HomingJointStatus::Failed);
    assert_eq!(s.mode, Mode::Idle);
    assert!(!s.homed, "a timed-out sequence establishes no reference");
    for _ in 0..(0.5 / dt).round() as usize {
        core.tick(dt, false);
    }
    let s = handles.snapshots.latest();
    assert!(
        s.qd[5].abs() < 0.01,
        "the seeking joint must actually stop: {}",
        s.qd[5]
    );
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
    // A sim teleport mid-sequence declares the arm homed; entering HOMING
    // cleared it once, so only the abort itself can clear it again.
    core.set_homed(true);

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
    // These cases drive the HomingSystem seam alone; the reference check
    // is the core's (it needs the torque model), so its hold is left out.
    bundle.robot.homing.reference_check_nm.clear();
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
        bus.boot_configure(&bundle.robot, bundle.active_tool(), 1)
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

    fn config_passes(&self) -> usize {
        self.bus
            .tx_log
            .iter()
            .filter(|(_, r)| matches!(r, TxRecord::ConfigPass { .. }))
            .count()
    }
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
    let expected = jh.backoff_s / (REHOME_SPEED_FACTOR * h.dt);
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

/// A loaded J0 — a load of 0.8× its homing current opposing its approach,
/// so the drive sits near its homing current the whole time it seeks —
/// reaches the same endstop reference as a free one. Each approach
/// starts with the current saturated and the rotor barely moving, the
/// full stall signature, which only the startup guard keeps from
/// latching at the start pose.
#[test]
fn a_loaded_joint_spinning_up_is_not_a_stall() {
    let bundle = single_joint_bundle(0);
    let jh = &bundle.robot.homing.joints[0];
    let eff = bundle
        .effective_home_offset(0)
        .unwrap_or(jh.home_offset_rad);
    let start = short_of_j0_stop(&bundle, 0.5);

    let (free, h) = home_j0(&bundle, start, |_, _, _| 0.0);
    assert_eq!(free, SeqStatus::Complete, "the free joint homes");
    let free_ref = i64::from(h.conv[0].motor_ticks(eff));

    let load_ma = 0.8 * jh.current_ma;
    let mut approach = None;
    let (loaded, h) = home_j0(&bundle, start, |cmd, _, _| {
        let v = f64::from(cmd.vel.unwrap_or(0));
        if v == 0.0 || v.signum() != *approach.get_or_insert(v.signum()) {
            return 0.0;
        }
        load_ma * v.signum()
    });
    assert_eq!(loaded, SeqStatus::Complete, "the loaded joint homes");
    assert_eq!(h.sys.statuses()[0], HomingJointStatus::Done);
    let loaded_ref = i64::from(h.conv[0].motor_ticks(eff));
    assert!(
        (loaded_ref - free_ref).abs() <= 50,
        "the loaded reference {loaded_ref} is not the free one {free_ref}: \
         a false stall latched short of the endstop"
    );
}

// ------------------------------------------------------------------
// Stall false-positive guard (G4): the 60 % current-window duty
// requirement, scripted at the HomingSystem seam.
// ------------------------------------------------------------------

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

    let window = (DETECT_WINDOW_S / h.dt).round().max(5.0) as u64;
    // Several windows of 40 % duty, each of which must not latch.
    let duty_ticks = 3 * window as u32;
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
        hit <= full_from + window,
        "100 % duty should latch within a detection window of {window} ticks \
         (hit {hit}, full duty from {full_from})"
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
/// nothing ever stalls) gets one full-range crossing at its seek speed,
/// with a quarter on top for the ramp and the stall confirmation, or
/// its configured `timeout_s` if that is longer. It fails on the tick
/// after that budget, with the joint marked Failed and the full node
/// config (normal current limits included) resent. Run at two tick
/// rates so the seconds→ticks conversion is pinned, not an accident of
/// the shipped dt.
#[test]
fn a_free_running_approach_fails_after_one_full_range_crossing() {
    for dt in [0.004, 0.01] {
        let mut bundle = single_joint_bundle(0);
        bundle.robot.robot.tick_dt_s = dt;
        let (j, jh) = (&bundle.robot.joints[0], &bundle.robot.homing.joints[0]);
        let span_ticks = (j.limits.hard_max_rad - j.limits.hard_min_rad) * j.gear_ratio
            / std::f64::consts::TAU
            * f64::from(1u32 << j.encoder_bits);
        let crossing_s = span_ticks / jh.speed_ticks_s;
        assert!(
            1.25 * crossing_s > jh.timeout_s,
            "the range, not the configured floor, sets this budget"
        );
        let timeout_ticks = (1.25 * crossing_s / dt).round() as u64;
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
        // the failure — exactly `round(seek_timeout_s / dt)` driven ticks.
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
/// the wound gearbox), negative current presses further in. Returns each
/// unbroken run of current-only frames with the position exposed at each
/// of its ticks, the latched reference, and the outcome.
type CurrentRuns = Vec<(Vec<i16>, Vec<i32>)>;
fn run_release_scenario(bundle: &ConfigBundle) -> (CurrentRuns, i64, SeqStatus) {
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

    let mut runs: CurrentRuns = Vec::new();
    let mut in_run = false;
    let mut outcome = SeqStatus::Running;
    for t in 1..30_000u64 {
        let exposed = h.state.nodes[n1].position_ticks.unwrap();
        let status = h.tick(t);
        let cmd = h.cmds[1];
        if cmd.pos.is_none() && cmd.vel.is_none() {
            // Current-only frame (cmd 2 DLC 2) — the release drive.
            let c = cmd.cur_ma.expect("current-only frame carries current");
            if !in_run {
                runs.push((Vec::new(), Vec::new()));
                in_run = true;
            }
            let run = runs.last_mut().unwrap();
            run.0.push(c);
            run.1.push(exposed);
            // Sign-sensitive plant: the current's sign decides whether
            // the gearbox relaxes (away from the stop) or winds tighter.
            pos += RELEASE_STEP * f64::from(c.signum());
        } else {
            in_run = false;
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
    (runs, latched, outcome)
}

#[test]
fn release_ramps_from_the_stall_push_holds_the_config_current_and_samples_at_eighty_percent() {
    let bundle = single_joint_bundle(1);
    let r = bundle.robot.homing.joints[1]
        .release
        .expect("J1 ships a release plan");
    let stall_ma = -(bundle.robot.homing.joints[1].current_ma as i16);
    let dt = bundle.robot.robot.tick_dt_s;
    let dur_ticks = (r.duration_s / dt).round().max(1.0) as usize;
    let sample_tick = ((dur_ticks as f64 * r.sample_pct).round() as usize).clamp(1, dur_ticks);
    let target = r.current_ma as i16;

    let (runs, latched, outcome) = run_release_scenario(&bundle);
    assert_eq!(outcome, SeqStatus::Complete);
    // The release is the only current-mode phase: leaving current mode
    // re-applies whatever the drive's velocity loop wound up during the
    // push, so the pass-1 hit stays in velocity mode (2026-09-23).
    let [(cmds, seen)] = runs.as_slice() else {
        panic!("exactly one run of current-only frames, the release: {runs:?}");
    };
    // Ramp in, hold, ramp out: the hold is exactly `duration_s` of the
    // config current, and the two ramps are equal and outside it.
    let ramp = (cmds.len() - dur_ticks) / 2;
    assert!(ramp > 1, "the release must ramp, not step: {cmds:?}");
    assert_eq!(cmds.len(), 2 * ramp + dur_ticks, "ramps of equal length");
    let (ramp_in, rest) = cmds.split_at(ramp);
    let (hold, ramp_out) = rest.split_at(dur_ticks);
    assert!(
        hold.iter().all(|&c| c == target),
        "the hold carries the config current verbatim: {hold:?}"
    );
    // Stepping straight from the stall push to the release current is what
    // flung the joint off its stop: the ramp starts at the push and moves
    // monotonically to the target, then back monotonically to zero.
    assert!(
        (ramp_in[0] - stall_ma).abs() <= 5,
        "the ramp starts at the stall push {stall_ma} mA, not at {} mA",
        ramp_in[0]
    );
    assert!(ramp_in
        .windows(2)
        .all(|w| (w[1] - w[0]) * (target - stall_ma).signum() >= 0));
    assert_eq!(
        *ramp_in.last().unwrap(),
        target,
        "the ramp in ends at the config current"
    );
    assert!(ramp_out
        .windows(2)
        .all(|w| (w[1] - w[0]) * target.signum() <= 0));
    assert_eq!(
        *ramp_out.last().unwrap(),
        0,
        "the ramp out ends at zero current"
    );
    // The reference is the position the joint had relaxed to at the
    // sample tick of the hold — the scripted plant moves a distinct 3 ticks
    // per release tick, so the latched value identifies the tick exactly.
    assert_eq!(
        latched,
        i64::from(seen[ramp + sample_tick - 1]),
        "reference sampled at round(dur · sample_pct) = tick {sample_tick} of the hold"
    );
    // The FSM is sign-sensitive, not |current|-sensitive: an inverted
    // config sign goes out inverted.
    let mut inverted = single_joint_bundle(1);
    let rel = inverted.robot.homing.joints[1]
        .release
        .as_mut()
        .expect("J1 ships a release plan");
    rel.current_ma = -rel.current_ma;
    let (inv_runs, _, inv_outcome) = run_release_scenario(&inverted);
    assert_eq!(inv_outcome, SeqStatus::Complete);
    assert!(
        inv_runs[0].0[ramp..ramp + dur_ticks]
            .iter()
            .all(|&c| c == -target),
        "the FSM forwards the inverted sign verbatim"
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
        let mut bus = SimBus::new(common::scene(bundle));
        bus.set_initial_joint_rad(q0);
        bus.boot_configure(&bundle.robot, bundle.active_tool(), 1)
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

    // The pre-clear's until-clear reversal lets the final approach
    // carry enough speed to coast OUT the top of this narrow band, so
    // park below it first — the probe assumes it approaches from the
    // clear side (part (b) parks the same way).
    for _ in 0..250 {
        h.drive(
            5,
            JointCommand::velocity(trunc_to_wire(-jh.speed_ticks_s), 0),
        );
    }
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

    // A hall band WIDER than one backoff's travel, booted at its center:
    // one backoff deep per side, the approach leaves it inside the
    // pre-clear guard, so its exit edge reads as booting on the sensor,
    // and a blind fixed backoff from that edge lands back at the band
    // center. Accepting the re-approach's early exit edge then latches
    // the wrong side of the band. The pre-clear must keep backing off
    // until the sensor actually reads clear, then approach and latch the
    // true edge.
    {
        let mut h = SimHomingHarness::new(&bundle, &q0, 5, q0[5], 0.02);
        let half = jh.backoff_s * jh.speed_ticks_s / j5_ticks_per_rad(&h, q0[5]);
        h.bus.set_hall_trigger(5, q0[5], half);

        h.sys.start(&mut h.bus);
        assert_eq!(h.run(20_000), SeqStatus::Complete, "the wide band homes");

        // The reference must put the SENSOR EDGE at the home offset. The
        // probe crosses that edge along the approach direction, and the brake
        // left the joint just past it, so park one backoff's travel back
        // inside the band (a quarter of its width) before measuring.
        let backoff_ticks = (jh.backoff_s / bundle.robot.robot.tick_dt_s).round() as u32;
        for _ in 0..backoff_ticks {
            h.drive(
                5,
                JointCommand::velocity(trunc_to_wire(-jh.speed_ticks_s), 0),
            );
        }
        let sensor = h.sensor_ticks(5, n5, jh.speed_ticks_s);
        let sensor_rad = h.conv[5].joint_rad(sensor);
        assert!(
            (sensor_rad - eff).abs() < 0.02,
            "the reference must be the band edge, not the boot pose: the \
             sensor edge reads {sensor_rad:.4} rad, home offset is {eff:.4} \
             (band half {half:.4})"
        );
    }

    // A sensor that never clears — shorted, or the magnet fell onto the
    // band — must FAIL the joint, never latch a reference. The drive's
    // hall trigger fires on an edge, and a sensor that never changes gives
    // none: the approach finds nothing to latch and fails on the tick its
    // seek budget runs out.
    {
        // A band as wide as J5's whole travel: in band wherever it goes.
        let limits = &bundle.robot.joints[5].limits;
        let half = limits.hard_max_rad - limits.hard_min_rad;
        let mut h = SimHomingHarness::new(&bundle, &q0, 5, q0[5], half);
        let budget = (jh.seek_timeout_s(&bundle.robot.joints[5]) / bundle.robot.robot.tick_dt_s)
            .round() as u32;
        h.sys.start(&mut h.bus);
        let (mut ticks, mut approach_from) = (0u32, None);
        let status = loop {
            ticks += 1;
            assert!(ticks <= 2 * budget, "the approach never gave up");
            if h.sys.status().phase[5] == HomingPhase::Approach {
                approach_from.get_or_insert(ticks);
            }
            match h.tick() {
                SeqStatus::Running => {}
                other => break other,
            }
        };
        let seeking = ticks - approach_from.expect("the joint approached");
        assert_eq!(
            status,
            SeqStatus::Failed,
            "an always-triggered sensor must fail, never latch a reference"
        );
        assert_eq!(h.sys.statuses()[5], HomingJointStatus::Failed);
        assert_eq!(h.sys.status().phase[5], HomingPhase::Approach);
        assert_eq!(seeking, budget, "failed after {seeking} seeking ticks");
    }
}

// ------------------------------------------------------------------
// Gripper calibration failure: recoverable without a restart.
// ------------------------------------------------------------------

/// A bundle whose whole sequence is the firmware gripper calibration.
fn gripper_cal_bundle() -> ConfigBundle {
    let mut bundle = common::bundle();
    bundle.robot.homing.reference_check_nm.clear();
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
    // The firmware calibrates the gripper itself: no Homer ever ran for
    // that slot, so its phase is Idle beside the Failed status, not a
    // never-started FSM's constructor phase.
    assert_eq!(s.homing.per_joint[MAX_JOINTS], HomingJointStatus::Failed);
    assert_eq!(
        s.homing.phase[MAX_JOINTS],
        HomingPhase::Idle,
        "a firmware-calibrated gripper reports no FSM phase"
    );
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

// ------------------------------------------------------------------
// Hall pre-clear: a joint that BOOTS on its sensor must reference the
// band edge, never wherever it happens to stand, and a sensor that
// never clears must fail rather than latch.
// ------------------------------------------------------------------

/// Motor ticks per joint radian around `q` for joint 5 — sizes hall
/// bands relative to the config backoff travel.
fn j5_ticks_per_rad(h: &SimHomingHarness, q: f64) -> f64 {
    f64::from(h.conv[5].motor_ticks(q + 0.1) - h.conv[5].motor_ticks(q)).abs() / 0.1
}

/// Every position phase must fail if a physical stop blocks its target,
/// even when that axis has already acquired a valid home reference — and
/// a clearance `move_to` that fails stops the sequence before the wrist
/// group it was clearing for.
#[test]
fn unreachable_positions_fail_in_every_homing_phase() {
    let mut failures = Vec::new();
    for phase in ["pre", "post", "global_post", "joint_post", "move_to"] {
        let mut bundle = single_joint_bundle(0);
        let target = bundle.robot.joints[0].limits.hard_max_rad + 1.0;
        let position = par6_config::PreMove::Position {
            joint: 0,
            position_rad: target,
            duration_s: 0.5,
        };
        let mut last_step = 0;
        match phase {
            "pre" => {
                bundle.robot.homing.sequence.push(SequenceStep {
                    pre_moves: vec![position],
                    home: Some(HomeGroup {
                        joints: vec![3, 5],
                        gripper: None,
                    }),
                    move_to: vec![],
                    post_moves: vec![],
                });
                last_step = 1;
            }
            "post" => bundle.robot.homing.sequence[0].post_moves.push(position),
            "global_post" => {
                bundle.robot.homing.post_moves.push(position);
                last_step = 1;
            }
            "joint_post" => {
                bundle.robot.homing.joints[0].post_home = Some(par6_config::PostHomeConfig {
                    position_rad: target,
                    speed_ticks_s: 30_000.0,
                })
            }
            "move_to" => {
                bundle.robot.homing.sequence = vec![
                    SequenceStep {
                        pre_moves: vec![],
                        home: None,
                        move_to: vec![MoveTo {
                            joint: 0,
                            position_rad: target,
                            duration_s: 0.5,
                        }],
                        post_moves: vec![],
                    },
                    SequenceStep {
                        pre_moves: vec![],
                        home: Some(HomeGroup {
                            joints: vec![3, 5],
                            gripper: None,
                        }),
                        move_to: vec![],
                        post_moves: vec![],
                    },
                ];
            }
            _ => unreachable!(),
        }
        let (mut core, mut handles, tx, _line) = sim_core_with_bundle(&bundle);
        let dt = core.tick_dt_s();
        start_homing(&mut core, &mut handles, &tx);
        let mut referenced = false;
        let mut stopped = false;
        for _ in 0..(25.0 / dt).round() as usize {
            core.tick(dt, false);
            let s = handles.snapshots.latest();
            referenced |= s.homing.per_joint[0] == HomingJointStatus::Done;
            if s.homing.per_joint[3] != HomingJointStatus::Idle
                || s.homing.per_joint[5] != HomingJointStatus::Idle
            {
                failures.push(format!("{phase}: wrist homing started without clearance"));
                break;
            }
            if !s.homing.active {
                stopped = true;
                if s.homed || s.homing.per_joint[0] != HomingJointStatus::Failed {
                    failures.push(format!(
                        "{phase}: unreachable target was accepted: {:?}",
                        s.homing
                    ));
                }
                if s.homing.sequence_step > last_step {
                    failures.push(format!(
                        "{phase}: advanced beyond the failed positioning step"
                    ));
                }
                break;
            }
        }
        assert_eq!(
            referenced,
            phase != "move_to",
            "{phase}: positioning runs after referencing J0, except the clearance move"
        );
        if !stopped {
            failures.push(format!(
                "{phase}: did not stop after the positioning failure"
            ));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// A valid slow positioning profile must reach its destination; the
/// settling budget starts after the planned travel, not at its start.
#[test]
fn homing_positioning_waits_for_profiles_longer_than_four_seconds() {
    let mut failures = Vec::new();
    for phase in ["pre", "post", "global_post", "joint_post"] {
        let mut bundle = single_joint_bundle(0);
        let target = bundle.robot.homing.joints[0].home_offset_rad - 0.3;
        let position = par6_config::PreMove::Position {
            joint: 0,
            position_rad: target,
            duration_s: 6.0,
        };
        match phase {
            "pre" => bundle.robot.homing.sequence.push(SequenceStep {
                pre_moves: vec![position],
                home: None,
                move_to: vec![],
                post_moves: vec![],
            }),
            "post" => bundle.robot.homing.sequence[0].post_moves.push(position),
            "global_post" => bundle.robot.homing.post_moves.push(position),
            "joint_post" => {
                bundle.robot.homing.joints[0].post_home = Some(par6_config::PostHomeConfig {
                    position_rad: target,
                    // Six-second Hermite profile across 0.3 rad.
                    speed_ticks_s: JointConversion::from_config(&bundle.robot.joints[0])
                        .motor_speed_ticks_s(1.5 * 0.3 / 6.0)
                        .abs(),
                })
            }
            _ => unreachable!(),
        }
        let (mut core, mut handles, tx, _line) = sim_core_with_bundle(&bundle);
        let dt = core.tick_dt_s();
        start_homing(&mut core, &mut handles, &tx);
        let mut stopped = false;
        for _ in 0..(25.0 / dt).round() as usize {
            core.tick(dt, false);
            let s = handles.snapshots.latest();
            if !s.homing.active {
                stopped = true;
                if s.homing.per_joint[0] != HomingJointStatus::Done
                    || (s.q[0] - target).abs() > 0.01
                {
                    failures.push(format!(
                        "{phase}: ended at {} instead of {target}, {:?}",
                        s.q[0], s.homing
                    ));
                }
                break;
            }
        }
        assert!(stopped, "{phase}: positioning never finished");
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// J0 homed alone on the simulator from `j0_rad`, its motor loaded by
/// whatever `load` asks for given the command J0 was last sent, its
/// latest reported state, and whether a backoff has happened yet.
fn home_j0(
    bundle: &ConfigBundle,
    j0_rad: f64,
    mut load: impl FnMut(&JointCommand, &NodeState, bool) -> f64,
) -> (SeqStatus, SimHomingHarness) {
    let mut q0: [f64; MAX_JOINTS] = std::array::from_fn(|i| {
        let j = &bundle.robot.joints[i];
        JointConversion::from_config(j).joint_rad(j.sector_master_position_ticks)
    });
    q0[0] = j0_rad;
    // J5's hall band is out of the way: only J0 homes.
    let mut h = SimHomingHarness::new(bundle, &q0, 5, 3.0, 0.0);
    h.sys.start(&mut h.bus);
    let node = bundle.robot.joints[0].node_id;
    let mut backed_off = false;
    let budget = (120.0 / bundle.robot.robot.tick_dt_s) as u32;
    for _ in 0..budget {
        backed_off |= h.cmds[0].vel.is_some_and(|v| v < 0);
        let ma = load(&h.cmds[0], &h.state.nodes[usize::from(node)], backed_off);
        h.bus.set_joint_load_ma(node, ma);
        match h.tick() {
            SeqStatus::Running => {}
            other => return (other, h),
        }
    }
    panic!("J0's homing did not finish");
}

/// J0's approach side and a start `rad` short of its stop on that side.
fn short_of_j0_stop(bundle: &ConfigBundle, rad: f64) -> f64 {
    let j = &bundle.robot.joints[0];
    let jh = &bundle.robot.homing.joints[0];
    let toward = JointConversion::from_config(j).joint_speed_rad_s(jh.speed_ticks_s);
    if toward > 0.0 {
        j.limits.hard_max_rad - rad
    } else {
        j.limits.hard_min_rad + rad
    }
}

/// A first pass that stalls against something other than the endstop
/// disagrees with the second, which finds the real one: the joint fails
/// instead of keeping either reference, and the drive gets its normal
/// limits back.
#[test]
fn two_passes_that_disagree_fail_the_joint_and_restore_its_limits() {
    let bundle = single_joint_bundle(0);
    let jh = &bundle.robot.homing.joints[0];
    // Held in place through pass 1 by a load that matches the homing
    // current; free once the backoff starts, half a radian from the
    // endstop.
    let (outcome, mut h) = home_j0(
        &bundle,
        short_of_j0_stop(&bundle, 0.5),
        |_, _, backed_off| {
            if backed_off {
                0.0
            } else {
                jh.current_ma
            }
        },
    );
    assert_eq!(outcome, SeqStatus::Failed, "two disagreeing passes fail");
    assert_eq!(h.sys.statuses()[0], HomingJointStatus::Failed);
    assert_eq!(
        h.sys.status().phase[0],
        HomingPhase::Settle,
        "it fails comparing the passes, not on the way to the stop"
    );
    assert!(!h.sys.active());

    let node = bundle.robot.joints[0].node_id;
    h.bus.queue_poll_override(
        PollAction::ConfigRead {
            node,
            kind: ConfigKind::Limits,
        },
        1,
    );
    h.drain();
    h.bus.poll_step().expect("poll");
    for _ in 0..3 {
        h.drain();
    }
    match h.state.nodes[usize::from(node)].readback(ConfigKind::Limits) {
        Some(Readback::Limits { current_ma, .. }) => assert_eq!(
            current_ma, bundle.robot.joints[0].ilim_ma as f32,
            "the drive is back on its normal current limit"
        ),
        other => panic!("no limits read back: {other:?}"),
    }
}
