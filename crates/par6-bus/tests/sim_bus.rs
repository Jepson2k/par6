//! Closed-loop sim backend integration tests.
//!
//! Every test drives [`SimBus`] exactly as the RT loop would — through the
//! [`DriverBus`] trait, with commands encoded by the production codec and
//! state decoded from the sim's real reply frames — and asserts the plant
//! and virtual-driver behaviors that spec/HOMING.md's "Sim requirements"
//! section names as acceptance criteria: endstop stall signatures for
//! both homing detection conditions, hall trigger/edge emulation,
//! release-phase preload relaxation, plus the driver watchdog, fault
//! injection, wrong-DLC discard, boot 14-bit-wrap semantics, the gripper
//! model and bit-exact determinism.

use std::collections::VecDeque;
use std::path::PathBuf;

use par6_bus::sim::{FaultKind, SimBus};
use par6_bus::spectral::codec::{pack_can_id, CanFrame, CommandId};
use par6_bus::spectral::convert::{ticks_per_radian, JointConversion};
use par6_bus::{
    BusState, DriverBus, FirmwareGripperCommand, Freshness, GripperCommand, JointCommand,
    ObjectDetection, PollAction, PollKind,
};
use par6_config::{GripperConfig, HomingStrategy, RobotConfig};

fn par6() -> RobotConfig {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
    RobotConfig::load(&path).expect("PAR6.toml")
}

fn msg_gripper() -> GripperConfig {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../config/grippers/MSG_small_motor_150mm_rail.toml");
    GripperConfig::load(&path).expect("MSG gripper TOML")
}

/// True boot pose with every joint at its config calibration pose (what
/// the sim defaults to when no override is given).
fn calibration_pose(robot: &RobotConfig) -> Vec<f64> {
    robot
        .joints
        .iter()
        .map(|j| JointConversion::from_config(j).joint_rad(j.sector_master_position_ticks))
        .collect()
}

/// Drives the bus in the spec/RT.md per-tick call order.
struct Rig {
    bus: SimBus,
    state: BusState,
    tick: u64,
    joints: usize,
}

impl Rig {
    fn boot(robot: &RobotConfig, gripper: Option<&GripperConfig>, q0: Option<&[f64]>) -> Self {
        let mut bus = SimBus::new();
        if let Some(q) = q0 {
            bus.set_initial_joint_rad(q);
        }
        bus.boot_configure(robot, gripper, robot.bus.boot_config_repeats)
            .expect("boot_configure");
        Rig {
            bus,
            state: BusState::new(),
            tick: 0,
            joints: robot.joints.len(),
        }
    }

    /// One RT tick: begin → drain → joint frames → gripper slot → poll.
    /// Returns the frames drained this tick.
    fn step(&mut self, cmds: &[JointCommand], gripper: &GripperCommand) -> usize {
        self.tick += 1;
        self.bus.begin_tick(self.tick);
        let drained = self.bus.drain_rx(&mut self.state).expect("drain_rx");
        self.bus.send_joint_commands(cmds).expect("joint send");
        self.bus.send_gripper(gripper).expect("gripper send");
        self.bus.poll_step().expect("poll_step");
        drained
    }

    /// A tick where the RT side fails to send the gripper its frame
    /// (models the mandatory-empty-poll rule being broken).
    fn step_without_gripper_frame(&mut self, cmds: &[JointCommand]) {
        self.tick += 1;
        self.bus.begin_tick(self.tick);
        self.bus.drain_rx(&mut self.state).expect("drain_rx");
        self.bus.send_joint_commands(cmds).expect("joint send");
        self.bus.poll_step().expect("poll_step");
    }

    fn idle_cmds(&self) -> Vec<JointCommand> {
        vec![JointCommand::idle(); self.joints]
    }
}

// ---------------------------------------------------------------------------
// HOMING.md detection oracles (transcribed from the spec text, not the code)
// ---------------------------------------------------------------------------

/// Windowed stall: displacement from a reference stays below
/// `max(10, |speed|·0.08·0.25)` ticks; window resets on movement; stalled
/// at `round(0.08/dt)` consecutive ticks (min 5).
struct StallWindow {
    thresh: f64,
    needed: u32,
    reference: Option<i32>,
    count: u32,
}

impl StallWindow {
    fn new(speed_ticks_s: f64, dt: f64) -> Self {
        Self {
            thresh: (speed_ticks_s.abs() * 0.08 * 0.25).max(10.0),
            needed: ((0.08 / dt).round() as u32).max(5),
            reference: None,
            count: 0,
        }
    }

    fn update(&mut self, pos: i32) -> bool {
        match self.reference {
            Some(r) if f64::from(pos - r).abs() < self.thresh => {
                self.count += 1;
                self.count >= self.needed
            }
            _ => {
                self.reference = Some(pos);
                self.count = 0;
                false
            }
        }
    }
}

/// Current ratio: after a 0.15 s startup guard, ticks with current above
/// `0.70 · homing_current_ma`; fires at `round(0.08/dt)` ticks (min 2)
/// with ≥60% of the window above threshold.
struct CurrentWindow {
    guard: u32,
    seen: u32,
    thresh: f64,
    len: usize,
    window: VecDeque<bool>,
}

impl CurrentWindow {
    fn new(homing_current_ma: f64, dt: f64) -> Self {
        Self {
            guard: (0.15 / dt).round() as u32,
            seen: 0,
            thresh: 0.70 * homing_current_ma,
            len: ((0.08 / dt).round() as usize).max(2),
            window: VecDeque::new(),
        }
    }

    fn update(&mut self, cur_ma: i16) -> bool {
        if self.seen < self.guard {
            self.seen += 1;
            return false;
        }
        self.window.push_back(f64::from(cur_ma).abs() > self.thresh);
        if self.window.len() > self.len {
            self.window.pop_front();
        }
        self.window.len() == self.len
            && self.window.iter().filter(|b| **b).count() * 10 >= self.len * 6
    }
}

/// Drive one actuator velocity-mode into its endstop exactly like the
/// homing FSM (homing current limit applied, cur 0 on the wire).
/// `drive_slot = Some(j)` drives arm joint `j`; `None` drives the gripper
/// motor through the gripper slot (`gripper_cmd` then carries the drive).
/// Returns `(gated-hit position, rest position, peak |current|)` and
/// asserts the HOMING.md detection fired at the stop, not in free travel.
#[allow(clippy::too_many_arguments)]
fn run_stall_approach(
    rig: &mut Rig,
    cmds: &mut [JointCommand],
    gripper_cmd: &GripperCommand,
    drive_slot: Option<usize>,
    node: usize,
    drive: JointCommand,
    speed_ticks_s: f64,
    homing_current_ma: f64,
    timeout_ticks: u64,
    dt: f64,
) -> (i32, i32, i32) {
    let mut stall = StallWindow::new(speed_ticks_s, dt);
    let mut ratio = CurrentWindow::new(homing_current_ma, dt);
    let mut ratio_pos = None;
    let mut hit_pos = None;
    let mut peak_cur = 0i32;
    if let Some(slot) = drive_slot {
        cmds[slot] = drive;
    }
    for _ in 0..timeout_ticks {
        rig.step(cmds, gripper_cmd);
        let ns = &rig.state.nodes[node];
        let (Some(pos), Some(cur)) = (ns.position_ticks, ns.current_ma) else {
            continue;
        };
        peak_cur = peak_cur.max(i32::from(cur).abs());
        let stalled = stall.update(pos);
        let over_current = ratio.update(cur);
        if over_current && ratio_pos.is_none() {
            ratio_pos = Some(pos);
        }
        // HOMING.md gates the two conditions together (current primary,
        // stall secondary): the hit is where BOTH hold.
        if stalled && over_current {
            hit_pos = Some(pos);
            break;
        }
    }
    let hit_pos = hit_pos.expect("gated stall detection never fired within the homing timeout");
    let ratio_pos = ratio_pos.expect("current-ratio condition never fired");
    // Let the seat settle (the FSM's dwell would run here).
    for _ in 0..((0.3 / dt).round() as u64) {
        rig.step(cmds, gripper_cmd);
        if let Some(cur) = rig.state.nodes[node].current_ma {
            peak_cur = peak_cur.max(i32::from(cur).abs());
        }
    }
    let rest = rig.state.nodes[node].position_ticks.expect("rest position");
    // Detection must have happened AT the endstop, not in free travel —
    // and the PRIMARY (current) condition must not have false-fired
    // mid-approach either (drag current stays under the threshold).
    assert!(
        (hit_pos - rest).abs() < 500,
        "stall detection fired at {hit_pos}, {} ticks from the endstop rest {rest}",
        (hit_pos - rest).abs()
    );
    assert!(
        (ratio_pos - rest).abs() < 500,
        "current ratio first fired at {ratio_pos}, {} ticks from the endstop rest {rest} — \
         free-travel drag current crossed the detection threshold",
        (ratio_pos - rest).abs()
    );
    (hit_pos, rest, peak_cur)
}

// ---------------------------------------------------------------------------
// 1. Endstop stall signatures + release-phase preload (HOMING.md sim reqs)
// ---------------------------------------------------------------------------

#[test]
fn stall_endstop_signatures_and_release_preload() {
    let robot = par6();
    let dt = robot.robot.tick_dt_s;
    // J1: stall strategy, direction 1 (negative motor drive), 250 mA
    // homing current, release +150 mA for 1 s sampling at 80%.
    let j = 1usize;
    let jc = &robot.joints[j];
    let h = &robot.homing.joints[j];
    assert_eq!(h.strategy, HomingStrategy::Stall);
    let node = usize::from(jc.node_id);
    let sign = if h.direction == 1 { -1.0 } else { 1.0 };

    // Boot close to the endstop so the approach stays short but still has
    // a real free-travel phase for the detectors to reject.
    let mut q0 = calibration_pose(&robot);
    q0[j] = jc.limits.hard_min_rad + 0.04;
    let mut rig = Rig::boot(&robot, None, Some(&q0));
    // Homing entry: Limits(normal vel, homing current) — spec/HOMING.md.
    rig.bus
        .send_limits(
            jc.node_id,
            jc.velocity_limit_ticks_s as f32,
            h.current_ma as f32,
            4,
        )
        .unwrap();

    let drive = JointCommand::velocity((sign * h.speed_ticks_s) as i32, 0);
    let mut cmds = rig.idle_cmds();
    let (_, rest, peak_cur) = run_stall_approach(
        &mut rig,
        &mut cmds,
        &GripperCommand::NoGripper,
        Some(j),
        node,
        drive,
        h.speed_ticks_s,
        h.current_ma,
        u64::from(robot.ticks(h.timeout_s)),
        dt,
    );

    // Stalled against the stop, the loop winds up to its saturated output
    // — the homing current limit — pressing toward the stop.
    assert!(
        f64::from(peak_cur) >= 0.9 * h.current_ma,
        "stall current peaked at {peak_cur} mA, expected to wind toward {} mA",
        h.current_ma
    );
    let last_cur = f64::from(rig.state.nodes[node].current_ma.unwrap());
    assert!(
        last_cur * sign > 0.0,
        "seated current {last_cur} mA does not press in the drive direction"
    );

    // Release phase: current-only frame (cmd 2 DLC 2) at the config value
    // relaxes the gearbox windup, moving the REPORTED encoder position —
    // exactly what HOMING.md's release phase samples.
    let rel = h.release.expect("J1 homing config carries a release phase");
    let rel_ticks = u64::from(robot.ticks(rel.duration_s));
    let sample_at = (rel_ticks as f64 * rel.sample_pct).round() as u64;
    cmds[j] = JointCommand::current(rel.current_ma as i16);
    let mut sampled = None;
    let mut end_pos = 0i32;
    for k in 1..=rel_ticks {
        rig.step(&cmds, &GripperCommand::NoGripper);
        end_pos = rig.state.nodes[node].position_ticks.unwrap();
        if k == sample_at {
            sampled = Some(end_pos);
        }
    }
    let sampled = sampled.expect("release sample point inside the phase");
    // Relaxation moves the encoder BACK toward the stop (opposite the
    // approach direction) by the accumulated windup...
    let relaxed = f64::from(sampled - rest) * -sign;
    assert!(
        relaxed >= 80.0,
        "release relaxed only {relaxed} ticks of preload"
    );
    // ...without detaching the joint from the endstop...
    assert!(
        relaxed <= 400.0,
        "release detached the joint ({relaxed} ticks of travel)"
    );
    // ...and has settled by the configured sample point.
    assert!(
        (end_pos - sampled).abs() <= 5,
        "windup still relaxing at the {}% sample point ({} → {})",
        rel.sample_pct * 100.0,
        sampled,
        end_pos
    );
}

// ---------------------------------------------------------------------------
// 2. Hall emulation (cmd 31 drive → cmd 32 trigger/edge/latched position)
// ---------------------------------------------------------------------------

#[test]
fn hall_joint_trigger_edge_and_latched_position() {
    let robot = par6();
    let dt = robot.robot.tick_dt_s;
    let j = 5usize; // J5: the hall-strategy joint
    let jc = &robot.joints[j];
    let h = &robot.homing.joints[j];
    assert_eq!(h.strategy, HomingStrategy::Hall);
    let node = usize::from(jc.node_id);
    let conv = JointConversion::from_config(jc);
    let tau = std::f64::consts::TAU;

    // The shipped PAR6 sequence nudges J5 to ~+0.6 and homes it from
    // there with the DEFAULT config: direction 0 (positive motor, dir=1
    // joint) moves the joint DOWN, away from `home_offset` itself — the
    // physical sensor is met at its circular alias `home_offset − 2π`.
    // Boot in the sequence's approach region to prove the default band
    // is reachable exactly as the vendor sequence drives it.
    let sensor_alias = h.home_offset_rad - tau;
    let mut q0 = calibration_pose(&robot);
    q0[j] = 0.6;
    let mut rig = Rig::boot(&robot, None, Some(&q0));
    let true0 = conv.motor_ticks(q0[j]);
    let emax = 1i32 << jc.encoder_bits;
    let wrap_off = true0.rem_euclid(emax) - true0;

    let sign = if h.direction == 1 { -1.0 } else { 1.0 };
    let mut cmds = rig.idle_cmds();
    cmds[j] = JointCommand::hall((sign * h.speed_ticks_s) as i32, 2);

    // Phase 1 — default band from the homing config: approach off-sensor,
    // then trigger with an edge and a latched position near the sensor.
    let mut recs: Vec<(i32, bool, bool)> = Vec::new(); // (pos, trigger, edge)
    let mut post_exit = 0u32;
    for _ in 0..u64::from(robot.ticks(h.timeout_s)) {
        rig.step(&cmds, &GripperCommand::NoGripper);
        let ns = &rig.state.nodes[node];
        let (Some(pos), Some(hall)) = (ns.position_ticks, ns.hall) else {
            continue;
        };
        recs.push((pos, hall.trigger, hall.edge));
        // Stop a few replies after the drive has crossed and left the band.
        if hall.trigger && recs.iter().any(|(_, t, _)| !t) {
            post_exit += 1;
            if post_exit >= 5 {
                break;
            }
        }
    }
    let hit = recs
        .iter()
        .position(|(_, trigger, _)| !trigger)
        .expect("hall never triggered within the homing timeout");
    assert!(hit > 5, "started on the sensor — no off-sensor approach");
    assert!(
        recs[..hit]
            .iter()
            .all(|(_, trigger, edge)| *trigger && !edge),
        "trigger/edge asserted during the off-sensor approach"
    );
    let (latched, _, edge) = recs[hit];
    assert!(edge, "no edge bit on the band-entry reply");
    assert_eq!(
        recs.iter().filter(|(_, _, e)| *e).count(),
        1,
        "edge must be a one-shot on band entry"
    );
    // Position is latched AT trigger: frozen while the trigger is active
    // even though the drive keeps moving through the band.
    let in_band: Vec<_> = recs[hit..].iter().take_while(|(_, t, _)| !t).collect();
    assert!(
        in_band.iter().all(|(p, _, _)| *p == latched),
        "in-band replies did not hold the latched position {latched}"
    );
    // The latch sits at the sensor (loose bound: the exact band half-width
    // is the sim's default, asserted precisely in phase 2).
    let latched_joint = conv.joint_rad(latched - wrap_off);
    assert!(
        (latched_joint - sensor_alias).abs() < 0.05,
        "latched at {latched_joint} rad, sensor at {sensor_alias} rad"
    );
    // After the band, live positions resume past the latch.
    let after = recs.last().unwrap();
    assert!(after.1 && after.0 > latched, "live position did not resume");

    // Phase 2 — a moved sensor (set_hall_trigger) triggers at the exact
    // band-entry edge: joint decreasing enters at `center + half`.
    let cur_joint = conv.joint_rad(recs.last().unwrap().0 - wrap_off);
    let (center, half) = (cur_joint - 0.1, 0.02);
    rig.bus.set_hall_trigger(j, center, half);
    let mut latched2 = None;
    for _ in 0..u64::from(robot.ticks(2.0)) {
        rig.step(&cmds, &GripperCommand::NoGripper);
        let ns = &rig.state.nodes[node];
        if let (Some(pos), Some(hall)) = (ns.position_ticks, ns.hall) {
            if hall.edge {
                latched2 = Some(pos);
                break;
            }
        }
    }
    let latched2 = latched2.expect("moved hall band never triggered");
    let expected = conv.motor_ticks(center + half) + wrap_off;
    let tol = h.speed_ticks_s * dt + 2.0; // one control step of travel
    assert!(
        f64::from((latched2 - expected).abs()) <= tol,
        "latched {latched2}, expected band entry at {expected} (±{tol})"
    );
}

// ---------------------------------------------------------------------------
// 3. Driver watchdog: command silence → Idle (configured WatchdogAction)
// ---------------------------------------------------------------------------

#[test]
fn watchdog_silence_drops_driver_to_idle() {
    let mut robot = par6();
    robot.joints[0].watchdog_timeout_ms = 200; // 50 ticks at 250 Hz
    let wd_ticks = u64::from(robot.ticks(f64::from(robot.joints[0].watchdog_timeout_ms) / 1000.0));
    let mut rig = Rig::boot(&robot, None, None);

    let mut cmds = rig.idle_cmds();
    cmds[0] = JointCommand::velocity(-8000, 0);
    for _ in 0..120 {
        rig.step(&cmds, &GripperCommand::NoGripper);
    }
    let speed = rig.state.nodes[0].speed_ticks_s.unwrap();
    assert!(speed <= -6000, "drive never reached speed (at {speed})");

    // Silence node 0 entirely (no data frame at all) while watching it
    // through encoder RTR polls — RTR polls keep freshness alive but must
    // NOT feed the driver watchdog.
    cmds[0] = JointCommand::default();
    let mut speeds = Vec::new();
    for _ in 0..(wd_ticks + 90) {
        rig.bus.queue_poll_override(
            PollAction::Poll {
                node: 0,
                kind: PollKind::Encoder,
            },
            1,
        );
        rig.step(&cmds, &GripperCommand::NoGripper);
        speeds.push(rig.state.nodes[0].speed_ticks_s.unwrap());
    }
    assert_eq!(
        rig.bus.freshness(0),
        Freshness::Fresh,
        "polls keep freshness"
    );
    // Driven right up to the configured deadline, idle-decayed after it.
    let before_fire = speeds[wd_ticks as usize - 3];
    assert!(
        before_fire <= -6000,
        "driver dropped out {before_fire} before the configured watchdog window"
    );
    let final_speed = *speeds.last().unwrap();
    assert!(
        final_speed.abs() <= 60,
        "velocity {final_speed} did not decay to idle after the watchdog fired"
    );
    // The fire latches the watchdog + aggregate error flags and the
    // per-frame live err bit.
    rig.bus.queue_poll_override(
        PollAction::Poll {
            node: 0,
            kind: PollKind::Errors,
        },
        1,
    );
    rig.step(&cmds, &GripperCommand::NoGripper);
    rig.step(&cmds, &GripperCommand::NoGripper);
    let flags = rig.state.nodes[0].error_flags.unwrap();
    assert!(
        flags.watchdog && flags.error,
        "watchdog flags not set: {flags:?}"
    );
    assert!(rig.state.nodes[0].live_error_bit, "live err bit not set");

    // Clear-error resets the flags; new commands re-arm the driver.
    rig.bus.send_clear_error(0, 3).unwrap();
    cmds[0] = JointCommand::velocity(-8000, 0);
    for _ in 0..120 {
        rig.step(&cmds, &GripperCommand::NoGripper);
    }
    assert!(
        rig.state.nodes[0].speed_ticks_s.unwrap() <= -6000,
        "drive did not resume after clear-error"
    );
    assert!(!rig.state.nodes[0].live_error_bit, "err bit survived clear");
    rig.bus.queue_poll_override(
        PollAction::Poll {
            node: 0,
            kind: PollKind::Errors,
        },
        1,
    );
    rig.step(&cmds, &GripperCommand::NoGripper);
    rig.step(&cmds, &GripperCommand::NoGripper);
    let flags = rig.state.nodes[0].error_flags.unwrap();
    assert!(
        !flags.watchdog && !flags.error && flags.calibrated && flags.activated,
        "flags not reset by clear-error: {flags:?}"
    );
}

// ---------------------------------------------------------------------------
// 4. Fault injection: per-type cmd-26 flags + per-frame live err bit
// ---------------------------------------------------------------------------

#[test]
fn fault_injection_err_bit_flags_and_clear() {
    let robot = par6();
    let gripper = msg_gripper();
    let gnode = robot.bus.gripper_node;
    let mut rig = Rig::boot(&robot, Some(&gripper), None);
    let cmds = rig.idle_cmds();
    for _ in 0..3 {
        rig.step(&cmds, &GripperCommand::FirmwarePoll);
    }
    assert!((0..6).all(|n| !rig.state.nodes[n].live_error_bit));
    assert!(!rig.state.gripper.live_error_bit);

    rig.bus.inject_fault(2, FaultKind::Encoder);
    rig.bus.inject_fault(gnode, FaultKind::Temperature);
    // Replies drain one tick after their command; skip the pre-injection
    // frames still in flight.
    rig.step(&cmds, &GripperCommand::FirmwarePoll);
    // EVERY reply from a faulted node now carries the err bit; healthy
    // nodes stay clean.
    for _ in 0..5 {
        rig.step(&cmds, &GripperCommand::FirmwarePoll);
        assert!(
            rig.state.nodes[2].live_error_bit,
            "motion reply lost the err bit"
        );
        assert!(
            rig.state.gripper.live_error_bit,
            "gripper reply lost the err bit"
        );
        assert!(!rig.state.nodes[1].live_error_bit && !rig.state.nodes[3].live_error_bit);
    }
    // The cmd-26 flags carry the per-type bit plus the aggregate.
    rig.bus.queue_poll_override(
        PollAction::Poll {
            node: 2,
            kind: PollKind::Errors,
        },
        1,
    );
    rig.step(&cmds, &GripperCommand::FirmwarePoll);
    rig.step(&cmds, &GripperCommand::FirmwarePoll);
    let flags = rig.state.nodes[2].error_flags.unwrap();
    assert!(
        flags.encoder && flags.error && !flags.temperature && flags.calibrated && flags.activated,
        "unexpected fault flags: {flags:?}"
    );
    // The gripper's fault surfaces on the cmd-60 temperature_error bit.
    assert!(rig.state.gripper.reply.unwrap().temperature_error);

    // Clear_Error (cmd 1) ends the fault on both nodes.
    rig.bus.send_clear_error(2, 3).unwrap();
    rig.bus.send_clear_error(gnode, 3).unwrap();
    for _ in 0..3 {
        rig.step(&cmds, &GripperCommand::FirmwarePoll);
    }
    assert!(!rig.state.nodes[2].live_error_bit && !rig.state.gripper.live_error_bit);
    assert!(!rig.state.gripper.reply.unwrap().temperature_error);
    rig.bus.queue_poll_override(
        PollAction::Poll {
            node: 2,
            kind: PollKind::Errors,
        },
        1,
    );
    rig.step(&cmds, &GripperCommand::FirmwarePoll);
    rig.step(&cmds, &GripperCommand::FirmwarePoll);
    let flags = rig.state.nodes[2].error_flags.unwrap();
    assert!(
        !flags.encoder && !flags.error,
        "fault survived clear: {flags:?}"
    );
}

// ---------------------------------------------------------------------------
// 5. Wrong-DLC frames are discarded whole
// ---------------------------------------------------------------------------

#[test]
fn wrong_dlc_frames_discarded_whole() {
    let mut robot = par6();
    robot.joints[3].watchdog_timeout_ms = 200; // 50 ticks
    let wd_ticks = u64::from(robot.ticks(0.2));
    let gripper = msg_gripper();
    let mut rig = Rig::boot(&robot, Some(&gripper), None);
    let node = usize::from(robot.joints[3].node_id);

    let mut cmds = rig.idle_cmds();
    cmds[3] = JointCommand::velocity(5000, 0);
    let mut baseline = 0usize;
    for _ in 0..120 {
        baseline = rig.step(&cmds, &GripperCommand::FirmwarePoll);
    }
    let p0 = rig.state.nodes[node].position_ticks.unwrap();

    // A cmd-2 frame with DLC 6 (a truncated position pack, bytes that
    // would decode as a huge position if partially applied) must change
    // NOTHING: no mode switch, no reply, no state jump.
    let bad_motion = CanFrame::data_frame(
        pack_can_id(robot.joints[3].node_id, CommandId::DataPack1, false),
        &[0x7F; 6],
    );
    // A cmd-61 frame with DLC 3 must not move the firmware gripper state.
    let bad_gripper = CanFrame::data_frame(
        pack_can_id(robot.bus.gripper_node, CommandId::GripperDataPack, false),
        &[250, 250, 250],
    );
    let gripper_pos_before = rig.state.gripper.reply.unwrap().position;
    let mut last_pos = p0;
    for _ in 0..40 {
        rig.bus.inject_host_frame(&bad_motion);
        rig.bus.inject_host_frame(&bad_gripper);
        let drained = rig.step(&cmds, &GripperCommand::FirmwarePoll);
        assert_eq!(drained, baseline, "a discarded frame produced a reply");
        let pos = rig.state.nodes[node].position_ticks.unwrap();
        let step = pos - last_pos;
        assert!(
            (0..100).contains(&step),
            "velocity drive disturbed by a wrong-DLC frame (step {step} ticks)"
        );
        last_pos = pos;
    }
    assert_eq!(
        rig.state.gripper.reply.unwrap().position,
        gripper_pos_before,
        "wrong-DLC cmd 61 moved the gripper"
    );

    // Discarded frames must not feed the watchdog either: silence the
    // node except for wrong-DLC frames and the watchdog still fires.
    cmds[3] = JointCommand::default();
    for _ in 0..(wd_ticks + 90) {
        rig.bus.inject_host_frame(&bad_motion);
        rig.bus.queue_poll_override(
            PollAction::Poll {
                node: 3,
                kind: PollKind::Encoder,
            },
            1,
        );
        rig.step(&cmds, &GripperCommand::FirmwarePoll);
    }
    assert!(
        rig.state.nodes[node].speed_ticks_s.unwrap().abs() <= 60,
        "wrong-DLC frames fed the watchdog (drive still running)"
    );
    rig.bus.queue_poll_override(
        PollAction::Poll {
            node: 3,
            kind: PollKind::Errors,
        },
        1,
    );
    rig.step(&cmds, &GripperCommand::FirmwarePoll);
    rig.step(&cmds, &GripperCommand::FirmwarePoll);
    assert!(
        rig.state.nodes[node].error_flags.unwrap().watchdog,
        "watchdog never fired under wrong-DLC-only traffic"
    );
}

// ---------------------------------------------------------------------------
// 6. Boot 14-bit wrap, sector recovery, accumulation, wire-frame positions
// ---------------------------------------------------------------------------

#[test]
fn boot_wrap_sector_semantics_and_position_mode_in_wire_coords() {
    let robot = par6();
    // J0 (gear 6.4, master 3969): a pose 0.24 rad below the calibration
    // pose puts the true motor position just below zero, so the boot
    // reading wraps to the top of the 14-bit range.
    let j = 0usize;
    let jc = &robot.joints[j];
    let node = usize::from(jc.node_id);
    let conv = JointConversion::from_config(jc);
    let emax = 1i32 << jc.encoder_bits;
    let mut q0 = calibration_pose(&robot);
    q0[j] -= 0.24;
    let true0 = conv.motor_ticks(q0[j]);
    assert!(true0 < 0, "premise: boot position must wrap ({true0})");

    let mut rig = Rig::boot(&robot, None, Some(&q0));
    let cmds = rig.idle_cmds();
    rig.step(&cmds, &GripperCommand::NoGripper);
    rig.step(&cmds, &GripperCommand::NoGripper);

    // kt fetch (kt_source = auto) answered at boot; scan saw all joints.
    // This proves the cmd-33 round trip reaches `BusState`, nothing more:
    // `VirtualDriver` is seeded from the same config kt, so the value is
    // equal by construction. Whether the RT ADOPTS a driver kt that
    // differs from config is `core_modes.rs`'s job — the sim cannot pose
    // that question.
    assert_eq!(rig.bus.connected_nodes(), 0b0011_1111);
    for (i, joint) in robot.joints.iter().enumerate() {
        let n = usize::from(joint.node_id);
        assert_eq!(
            rig.state.nodes[n].kt_nm_a,
            Some(joint.kt_nm_a as f32),
            "kt of joint {i}"
        );
    }

    // The first reported position is the boot reading wrapped to 14 bits…
    let reported0 = rig.state.nodes[node].position_ticks.unwrap();
    assert!(
        (reported0 - true0.rem_euclid(emax)).abs() <= 1,
        "boot report {reported0} is not the 14-bit wrap of {true0}"
    );
    // …and the production boot calibration recovers the true pose from it.
    let mut boot_conv = JointConversion::from_config(jc);
    boot_conv.determine_sector(reported0);
    let tick_rad = 1.0 / ticks_per_radian(emax, jc.gear_ratio);
    let recovered = boot_conv.joint_rad(reported0);
    assert!(
        (recovered - q0[j]).abs() <= 2.0 * tick_rad,
        "sector recovery gave {recovered}, true pose {}",
        q0[j]
    );

    // Positions ACCUMULATE from the wrapped base: driving through the
    // encoder-top boundary must not re-wrap the report.
    let mut cmds = rig.idle_cmds();
    cmds[j] = JointCommand::velocity(6000, 0);
    let mut prev = reported0;
    for _ in 0..400 {
        rig.step(&cmds, &GripperCommand::NoGripper);
        let pos = rig.state.nodes[node].position_ticks.unwrap();
        assert!(pos >= prev, "reported position wrapped ({prev} → {pos})");
        prev = pos;
    }
    assert!(prev > emax, "never crossed the encoder boundary ({prev})");

    // Position mode operates in WIRE coordinates: commanding a value in
    // the reported frame settles the reported position there. Speed 0 —
    // the channel is a velocity feedforward, and a standing nonzero
    // feedforward against a fixed target parks the plant offset by
    // ff/KPP past it.
    let target = reported0 + 3000;
    cmds[j] = JointCommand::position(target, 0, 0);
    let mut positions = Vec::new();
    for _ in 0..2500 {
        rig.step(&cmds, &GripperCommand::NoGripper);
        positions.push(rig.state.nodes[node].position_ticks.unwrap());
    }
    let last = *positions.last().unwrap();
    assert!(
        (last - target).abs() <= 30,
        "position loop settled at {last}, commanded {target} — the driver \
         is not closing its loop in wire coordinates"
    );
    let settle_band = positions[positions.len() - 100..]
        .iter()
        .map(|p| (p - last).abs())
        .max()
        .unwrap();
    assert!(
        settle_band <= 10,
        "still moving at the end ({settle_band} ticks)"
    );

    // The periodic device-info sweep has covered every node by now
    // (~1006 poll slots into the run).
    for joint in &robot.joints {
        let n = usize::from(joint.node_id);
        let info = rig.state.nodes[n]
            .device_info
            .unwrap_or_else(|| panic!("no device info for node {n} after the sweep"));
        assert_eq!(info.serial, 1_000 + i32::from(joint.node_id));
    }
}

/// The vendor runtime's hold is a position frame with Speed=0 (last
/// target, gravity current) — and the firmware's position mode treats the
/// speed channel as a velocity FEEDFORWARD, so that frame still closes
/// position error at full authority. A driver model that reads the speed
/// channel as a per-command velocity cap freezes whatever error exists
/// when a profile ends at speed 0: the plant settles short permanently
/// (issues #22/#26).
#[test]
fn zero_speed_position_frames_still_close_position_error() {
    let robot = par6();
    let j = 0usize;
    let node = usize::from(robot.joints[j].node_id);
    let mut rig = Rig::boot(&robot, None, None);
    let idle = rig.idle_cmds();
    rig.step(&idle, &GripperCommand::NoGripper);
    rig.step(&idle, &GripperCommand::NoGripper);
    let start = rig.state.nodes[node].position_ticks.unwrap();

    // The hold shape verbatim: a fixed position target, speed 0, no
    // current feedforward — repeated every tick, as EXEC hold does.
    let target = start + 2000;
    let mut cmds = rig.idle_cmds();
    cmds[j] = JointCommand::position(target, 0, 0);
    for _ in 0..2500 {
        rig.step(&cmds, &GripperCommand::NoGripper);
    }
    let last = rig.state.nodes[node].position_ticks.unwrap();
    assert!(
        (last - target).abs() <= 30,
        "a zero-speed position frame left {} ticks of error unclosed \
         (settled at {last}, commanded {target}) — the speed channel is a \
         feedforward, not a velocity cap",
        (last - target).abs(),
    );
}

// ---------------------------------------------------------------------------
// 7a. Gripper firmware mode: calibrate, empty polls, moves, objects
// ---------------------------------------------------------------------------

#[test]
fn gripper_firmware_calibrate_empty_polls_and_moves() {
    let robot = par6();
    let mut gripper = msg_gripper();
    // Short watchdog so completing calibration on polls alone PROVES the
    // DLC-0 empty poll feeds it (375 calibration ticks >> 50).
    gripper.driver.as_mut().unwrap().watchdog_timeout_ms = 200;
    let wd_ticks = u64::from(robot.ticks(0.2));
    let mut rig = Rig::boot(&robot, Some(&gripper), None);
    let cmds = rig.idle_cmds();

    // cmd 62 once, then DLC-0 empty polls every tick (HOMING.md).
    let run_calibration = |rig: &mut Rig| -> Option<u64> {
        rig.step(&cmds.clone(), &GripperCommand::Calibrate);
        for k in 1..=u64::from(robot.ticks(10.0)) {
            rig.step(&cmds.clone(), &GripperCommand::FirmwarePoll);
            if rig.state.gripper.reply.is_some_and(|r| r.calibrated) {
                return Some(k);
            }
        }
        None
    };
    let took = run_calibration(&mut rig).expect("calibration never completed on empty polls");
    assert!(
        took > u64::from(robot.ticks(1.0)),
        "calibration finished implausibly fast ({took} ticks)"
    );
    let r = rig.state.gripper.reply.unwrap();
    assert_eq!(r.position, 0, "calibration must end fully open");
    assert!(!r.action_status, "still moving after calibration");

    // Breaking the mandatory every-tick poll mid-calibration halts the
    // sequence uncalibrated.
    rig.step(&cmds, &GripperCommand::Calibrate);
    for _ in 0..20 {
        rig.step(&cmds, &GripperCommand::FirmwarePoll);
    }
    for _ in 0..(wd_ticks + 20) {
        rig.step_without_gripper_frame(&cmds);
    }
    for _ in 0..u64::from(robot.ticks(3.0)) {
        rig.step(&cmds, &GripperCommand::FirmwarePoll);
    }
    assert!(
        !rig.state.gripper.reply.unwrap().calibrated,
        "calibration survived a broken poll stream"
    );

    // A cmd-61 command overwrites an in-progress calibration: the move
    // runs, the calibration never completes.
    rig.step(&cmds, &GripperCommand::Calibrate);
    for _ in 0..50 {
        rig.step(&cmds, &GripperCommand::FirmwarePoll);
    }
    let move_to = |position: u8| {
        GripperCommand::Firmware(FirmwareGripperCommand {
            position,
            speed: 150,
            current_ma: 500,
            activate: true,
            action: true,
            estop: false,
            release_dir: false,
        })
    };
    for _ in 0..u64::from(robot.ticks(1.0)) {
        rig.step(&cmds, &move_to(200));
    }
    let r = rig.state.gripper.reply.unwrap();
    assert_eq!(r.position, 200, "cmd 61 did not take over the calibration");
    assert!(!r.calibrated, "aborted calibration reported calibrated");

    // Full firmware move with per-tick replay (how the homing sequence
    // replays its gripper_move): calibrate, then close to 252.
    run_calibration(&mut rig).expect("re-calibration failed");
    let mut prev = rig.state.gripper.reply.unwrap().position;
    let mut saw_moving = false;
    for _ in 0..u64::from(robot.ticks(2.5)) {
        rig.step(&cmds, &move_to(252));
        let r = rig.state.gripper.reply.unwrap();
        assert!(r.position >= prev, "close travel reversed");
        if r.action_status {
            saw_moving = true;
            assert_eq!(r.object_detection, ObjectDetection::Moving);
        }
        prev = r.position;
    }
    let r = rig.state.gripper.reply.unwrap();
    assert!(saw_moving, "no moving phase observed");
    assert_eq!(r.position, 252);
    assert_eq!(r.object_detection, ObjectDetection::ReachedNoObject);
    assert!(!r.action_status);
    assert_eq!(r.current_ma, 0, "current at rest");

    // An object between the jaws jams the close early: detection code 1,
    // pressing at the commanded current.
    for _ in 0..u64::from(robot.ticks(2.0)) {
        rig.step(&cmds, &move_to(20)); // open first
    }
    rig.bus.set_gripper_object_closing(Some(180));
    for _ in 0..u64::from(robot.ticks(2.0)) {
        rig.step(&cmds, &move_to(252));
    }
    let r = rig.state.gripper.reply.unwrap();
    assert_eq!(r.position, 180, "jaws passed through the object");
    assert_eq!(r.object_detection, ObjectDetection::DetectedClosing);
    assert_eq!(r.current_ma, 500, "pressing current is the commanded limit");
    // Removing the object lets the replayed move finish.
    rig.bus.set_gripper_object_closing(None);
    for _ in 0..u64::from(robot.ticks(2.0)) {
        rig.step(&cmds, &move_to(252));
    }
    let r = rig.state.gripper.reply.unwrap();
    assert_eq!(r.position, 252);
    assert_eq!(r.object_detection, ObjectDetection::ReachedNoObject);
}

// ---------------------------------------------------------------------------
// 7b. Gripper motor mode: stall homing detectability + backoff
// ---------------------------------------------------------------------------

#[test]
fn gripper_motor_mode_homing_stall() {
    let robot = par6();
    let gripper = msg_gripper();
    let dt = robot.robot.tick_dt_s;
    let gh = gripper.homing.clone().expect("MSG homing table");
    let gd = gripper.driver.clone().expect("MSG driver table");
    let gnode = robot.bus.gripper_node;
    let mut rig = Rig::boot(&robot, Some(&gripper), None);
    // Homing entry Limits — the only path that applies the homing current
    // to the gripper motor (spec/HOMING.md).
    rig.bus
        .send_limits(
            gnode,
            gd.velocity_limit_ticks_s as f32,
            gh.current_ma as f32,
            4,
        )
        .unwrap();

    let sign = if gh.direction == 1 { -1.0 } else { 1.0 };
    let drive = JointCommand::velocity((sign * gh.speed_ticks_s) as i32, 0);
    let mut cmds = rig.idle_cmds();
    let (_, rest, peak_cur) = run_stall_approach(
        &mut rig,
        &mut cmds,
        &GripperCommand::Motor(drive),
        None,
        usize::from(gnode),
        drive,
        gh.speed_ticks_s,
        gh.current_ma,
        u64::from(robot.ticks(gh.timeout_s)),
        dt,
    );
    assert!(
        f64::from(peak_cur) >= 0.9 * gh.current_ma,
        "gripper stall current peaked at {peak_cur} mA (homing limit {} mA)",
        gh.current_ma
    );

    // Backoff (two-pass pass 1 → reverse) must break the seat.
    let backoff = JointCommand::velocity((-sign * gh.speed_ticks_s) as i32, 0);
    for _ in 0..u64::from(robot.ticks(gh.backoff_s)) {
        rig.step(&cmds, &GripperCommand::Motor(backoff));
    }
    let pos = rig.state.nodes[usize::from(gnode)].position_ticks.unwrap();
    let travel = f64::from(pos - rest) * -sign;
    assert!(
        travel >= 1000.0,
        "backoff did not detach from the endstop (moved {travel} ticks)"
    );
}

// ---------------------------------------------------------------------------
// 8. Determinism: identical tick+command streams → bit-identical states
// ---------------------------------------------------------------------------

#[test]
fn identical_streams_are_bit_identical() {
    fn run() -> (Vec<BusState>, u64) {
        let mut robot = par6();
        robot.joints[0].watchdog_timeout_ms = 200;
        let gripper = msg_gripper();
        let mut q0 = calibration_pose(&robot);
        q0[5] = -4.3;
        let mut rig = Rig::boot(&robot, Some(&gripper), Some(&q0));
        let bad = CanFrame::data_frame(pack_can_id(2, CommandId::DataPack1, false), &[9; 7]);
        let mut states = Vec::new();
        for t in 1..=400u64 {
            let mut cmds = rig.idle_cmds();
            match t {
                1..=60 => {
                    cmds[0] = JointCommand::velocity(4000, 0);
                    cmds[1] = JointCommand::velocity(-3000, 100);
                    cmds[5] = JointCommand::hall(9000, 2);
                }
                61..=200 => {
                    cmds[2] = JointCommand::position(5000, 20000, 0);
                    cmds[3] = JointCommand::pd(200, 0, 40);
                    cmds[4] = JointCommand::current(-120);
                }
                _ => {
                    cmds[0] = JointCommand::default(); // silence → watchdog
                }
            }
            let grip = match t {
                1 => GripperCommand::Calibrate,
                2..=120 => GripperCommand::FirmwarePoll,
                _ => GripperCommand::Firmware(FirmwareGripperCommand {
                    position: 200,
                    speed: 120,
                    current_ma: 400,
                    activate: true,
                    action: true,
                    estop: false,
                    release_dir: false,
                }),
            };
            match t {
                30 => rig.bus.inject_fault(4, FaultKind::Vbus),
                45 => rig.bus.send_clear_error(4, 3).unwrap(),
                70 => rig.bus.send_limits(1, 80000.0, 250.0, 4).unwrap(),
                90 => rig.bus.inject_host_frame(&bad),
                130 => rig.bus.queue_poll_override(
                    PollAction::Poll {
                        node: 2,
                        kind: PollKind::Encoder,
                    },
                    5,
                ),
                _ => {}
            }
            rig.step(&cmds, &grip);
            states.push(rig.state.clone());
        }
        (states, rig.bus.dropped_rx_frames())
    }

    let (a, dropped_a) = run();
    let (b, dropped_b) = run();
    assert_eq!(dropped_a, dropped_b);
    assert_eq!(a.len(), b.len());
    for (t, (sa, sb)) in a.iter().zip(&b).enumerate() {
        assert!(sa == sb, "state streams diverge at tick {}", t + 1);
    }
}

/// Teleport (the sim's fast homing) re-seeds the arm mid-session: the
/// plant lands at the commanded configuration — reported exactly as a
/// boot reading there would be — while the bus AROUND it keeps running.
/// Placing the arm by re-running `boot_configure` lands it too, but
/// rebuilds the drivers (limits pushed at runtime revert to config) and
/// the gripper front end (calibration gone) around it.
#[test]
fn teleport_reseeds_the_arm_without_rebooting_the_bus() {
    let robot = par6();
    let gripper = msg_gripper();
    let mut rig = Rig::boot(&robot, Some(&gripper), None);
    let cmds = rig.idle_cmds();

    // State the bus is carrying when the teleport arrives: a calibrated
    // gripper, and a current limit pushed onto J0 at runtime (what the
    // homing sequence does before an approach).
    rig.step(&cmds, &GripperCommand::Calibrate);
    for _ in 0..u64::from(robot.ticks(10.0)) {
        rig.step(&cmds, &GripperCommand::FirmwarePoll);
        if rig.state.gripper.reply.is_some_and(|r| r.calibrated) {
            break;
        }
    }
    assert!(
        rig.state.gripper.reply.unwrap().calibrated,
        "the gripper must be calibrated before the teleport"
    );
    let j0 = &robot.joints[0];
    let pushed_ilim_ma = 250.0f64;
    assert!(
        pushed_ilim_ma < j0.ilim_ma,
        "the pushed limit must be lower"
    );
    rig.bus
        .send_limits(
            j0.node_id,
            j0.velocity_limit_ticks_s as f32,
            pushed_ilim_ma as f32,
            2,
        )
        .expect("send_limits");

    // Teleport: every joint a few degrees off its calibration pose.
    let mut target = calibration_pose(&robot);
    for (q, j) in target.iter_mut().zip(&robot.joints) {
        *q = (*q + 0.2).clamp(j.limits.hard_min_rad, j.limits.hard_max_rad);
    }
    rig.bus.teleport_joint_rad(&target).expect("teleport");
    rig.step(&cmds, &GripperCommand::FirmwarePoll);
    rig.step(&cmds, &GripperCommand::FirmwarePoll);

    for (j, jc) in robot.joints.iter().enumerate() {
        let conv = JointConversion::from_config(jc);
        let want = conv
            .motor_ticks(target[j])
            .rem_euclid(1i32 << jc.encoder_bits);
        let got = rig.state.nodes[usize::from(jc.node_id)]
            .position_ticks
            .expect("position after teleport");
        assert!(
            (got - want).abs() <= 2,
            "joint {j} reported {got} ticks after a teleport to {want}"
        );
    }

    // The bus around the plant is untouched: the gripper is still
    // calibrated, and J0 still saturates at the limit pushed before the
    // teleport instead of the config ceiling.
    assert!(
        rig.state.gripper.reply.unwrap().calibrated,
        "the teleport reset the gripper front end"
    );
    let mut drive = rig.idle_cmds();
    drive[0] = JointCommand::current((j0.ilim_ma as i32) as i16);
    for _ in 0..10 {
        rig.step(&drive, &GripperCommand::FirmwarePoll);
    }
    let cur = f64::from(
        rig.state.nodes[usize::from(j0.node_id)]
            .current_ma
            .expect("current reply"),
    );
    assert!(
        (cur - pushed_ilim_ma).abs() < 1.0,
        "J0 drove {cur} mA: the teleport reverted the runtime current limit \
         (pushed {pushed_ilim_ma} mA, config ceiling {} mA)",
        j0.ilim_ma
    );
}

// ---------------------------------------------------------------------------
// Dynamics plant (feature sim-dynamics): same DriverBus surface, torque-
// level physics. Gated: needs the C++ shim from scripts/ffi/setup.sh.
// ---------------------------------------------------------------------------

#[cfg(feature = "sim-dynamics")]
mod dynamics {
    use super::*;
    use par6_bus::spectral::convert::{torque_to_ma_factor, trunc_to_wire};

    fn urdf() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/par6_description/URDF/par6_flange/urdf/par6_flange.urdf")
    }

    fn boot(robot: &RobotConfig, q0: Option<&[f64]>) -> Rig {
        let mut bus = SimBus::with_dynamics(urdf());
        if let Some(q) = q0 {
            bus.set_initial_joint_rad(q);
        }
        bus.boot_configure(robot, None, robot.bus.boot_config_repeats)
            .expect("boot_configure (dynamics)");
        Rig {
            bus,
            state: BusState::new(),
            tick: 0,
            joints: robot.joints.len(),
        }
    }

    /// Idle drivers + gravity: the arm sags, so reported positions drift
    /// — torque-level physics is live behind the same DriverBus surface.
    #[test]
    fn gravity_sags_idle_arm() {
        let robot = par6();
        let mut q0 = calibration_pose(&robot);
        q0[1] = -1.5; // shoulder off vertical → nonzero gravity torque
        let mut rig = boot(&robot, Some(&q0));
        let cmds: Vec<JointCommand> = vec![JointCommand::default(); rig.joints];
        let mut first = None;
        for _ in 0..u64::from(robot.ticks(0.5)) {
            rig.step(&cmds, &GripperCommand::NoGripper);
            rig.bus.queue_poll_override(
                PollAction::Poll {
                    node: 1,
                    kind: PollKind::Encoder,
                },
                1,
            );
            if first.is_none() {
                first = rig.state.nodes[1].position_ticks;
            }
        }
        let first = first.expect("no shoulder position");
        let last = rig.state.nodes[1].position_ticks.unwrap();
        assert!(
            (last - first).abs() > 200,
            "idle arm did not sag under gravity ({first} → {last})"
        );
    }

    /// The endstop stall signatures HOMING.md requires hold on the
    /// dynamics plant too (J0: vertical axis, gravity-neutral).
    #[test]
    fn dynamics_endstop_stall_signatures() {
        let robot = par6();
        let dt = robot.robot.tick_dt_s;
        let j = 0usize;
        let jc = &robot.joints[j];
        let h = &robot.homing.joints[j];
        let mut q0 = calibration_pose(&robot);
        q0[j] = jc.limits.hard_max_rad - 0.08; // short approach to the stop
        let mut rig = boot(&robot, Some(&q0));
        rig.bus
            .send_limits(
                jc.node_id,
                jc.velocity_limit_ticks_s as f32,
                h.current_ma as f32,
                4,
            )
            .unwrap();
        let sign = if h.direction == 1 { -1.0 } else { 1.0 };
        let drive = JointCommand::velocity((sign * h.speed_ticks_s) as i32, 0);
        let mut cmds = rig.idle_cmds();
        let (_, _, peak_cur) = run_stall_approach(
            &mut rig,
            &mut cmds,
            &GripperCommand::NoGripper,
            Some(j),
            usize::from(jc.node_id),
            drive,
            h.speed_ticks_s,
            h.current_ma,
            u64::from(robot.ticks(h.timeout_s)),
            dt,
        );
        assert!(
            f64::from(peak_cur) >= 0.9 * h.current_ma,
            "dynamics stall current peaked at {peak_cur} mA (limit {} mA)",
            h.current_ma
        );
    }

    /// Every joint held by its own gravity torque stays put — the wrist
    /// included. The drive is the REAL controller path: G(q) from the
    /// same model the plant integrates, through the config
    /// torque↔current factor, truncated to whole mA like the RT's
    /// commit, sent as cmd-2 current frames. The light wrist joints are
    /// the ones this can fail on: their smoothed Coulomb friction is a
    /// stiff explicit damper next to their inertia, and at the shared
    /// smoothing width they oscillate instead of damping and the joint
    /// drifts degrees per second under perfect compensation.
    #[test]
    fn gravity_compensated_joints_hold_including_the_wrist() {
        /// Hold tolerance \[deg\] over the watch window.
        const HOLD_TOL_DEG: f64 = 1.0;
        let robot = par6();
        // Inside every soft window with gravity on every joint that can
        // carry it: G ~ [0, -5.5, 1.4, -0.05, 0.013, 0] Nm — the wrist
        // value is the physical ceiling for the flange-tipped arm.
        let q0: Vec<f64> = [-40.0f64, -15.0, 195.0, 0.0, 60.0, 90.0]
            .iter()
            .map(|d| d.to_radians())
            .collect();
        let mut rig = boot(&robot, Some(&q0));
        let conv: Vec<JointConversion> = robot
            .joints
            .iter()
            .map(JointConversion::from_config)
            .collect();
        let factor: Vec<f64> = robot
            .joints
            .iter()
            .map(|j| torque_to_ma_factor(j.gear_ratio, j.gear_efficiency, j.kt_nm_a, j.dir))
            .collect();
        let mut model = pinokin_sys::Model::from_urdf(&urdf(), None, None).expect("model");

        let n = robot.joints.len();
        let mut q = q0.clone();
        let mut g = vec![0.0; n];
        let mut cmds = rig.idle_cmds();
        let mut offset = vec![0i32; n];
        let mut seeded = false;
        let mut drift_deg = vec![0.0f64; n];
        for _ in 0..u64::from(robot.ticks(4.0)) {
            // Measured pose off the real reply frames (the sim reports a
            // wrapped boot reading, so the first one fixes the offset the
            // RT would install as its home reference).
            let mut have = true;
            for (j, jc) in robot.joints.iter().enumerate() {
                match rig.state.nodes[usize::from(jc.node_id)].position_ticks {
                    Some(t) => {
                        if !seeded {
                            offset[j] = t - conv[j].motor_ticks(q0[j]);
                        }
                        q[j] = conv[j].joint_rad(t - offset[j]);
                    }
                    None => have = false,
                }
            }
            seeded |= have;
            model.gravity_into(&q, &mut g).expect("G(q)");
            for j in 0..n {
                cmds[j] = JointCommand::current(trunc_to_wire(g[j] * factor[j]) as i16);
            }
            rig.step(&cmds, &GripperCommand::NoGripper);
            if seeded {
                for j in 0..n {
                    drift_deg[j] = drift_deg[j].max((q[j] - q0[j]).abs().to_degrees());
                }
            }
        }
        assert!(seeded, "no measured pose ever arrived");
        for j in 0..n {
            assert!(
                drift_deg[j] < HOLD_TOL_DEG,
                "joint {j} drifted {:.2}° under its own gravity torque \
                 (all joints: {drift_deg:?})",
                drift_deg[j]
            );
        }
    }

    #[test]
    fn dynamics_streams_are_bit_identical() {
        fn run() -> Vec<BusState> {
            let robot = par6();
            let mut rig = boot(&robot, None);
            let mut states = Vec::new();
            for t in 1..=200u64 {
                let mut cmds = rig.idle_cmds();
                if t > 20 {
                    cmds[0] = JointCommand::velocity(3000, 0);
                    cmds[2] = JointCommand::position(2000, 15000, 0);
                }
                rig.step(&cmds, &GripperCommand::NoGripper);
                states.push(rig.state.clone());
            }
            states
        }
        let a = run();
        let b = run();
        for (t, (sa, sb)) in a.iter().zip(&b).enumerate() {
            assert!(sa == sb, "dynamics streams diverge at tick {}", t + 1);
        }
    }
}

// ---------------------------------------------------------------------------
// MuJoCo plant (feature sim-mujoco): same DriverBus surface, contact-level
// physics in a full scene (floor + graspable object). Gated: needs
// libmujoco from scripts/ffi/setup.sh.
// ---------------------------------------------------------------------------

#[cfg(feature = "sim-mujoco")]
mod mujoco {
    use super::*;

    /// Reach-down pose over the scene's grasp object (config frame).
    const GRASP_POSE: [f64; 6] = [0.0, -0.25, 4.35, 0.0, -1.28, 0.0];

    fn scene() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("sim-assets/PAR6_MSG_scene.xml")
    }

    fn boot(robot: &RobotConfig, gripper: Option<&GripperConfig>, q0: Option<&[f64]>) -> Rig {
        let mut bus = SimBus::with_mujoco(scene());
        if let Some(q) = q0 {
            bus.set_initial_joint_rad(q);
        }
        bus.boot_configure(robot, gripper, robot.bus.boot_config_repeats)
            .expect("boot_configure (mujoco)");
        Rig {
            bus,
            state: BusState::new(),
            tick: 0,
            joints: robot.joints.len(),
        }
    }

    /// Idle drivers + gravity: the arm sags, so reported positions drift
    /// — MuJoCo physics is live behind the same DriverBus surface.
    #[test]
    fn mujoco_gravity_sags_idle_arm() {
        let robot = par6();
        let mut q0 = calibration_pose(&robot);
        q0[1] = -1.5; // shoulder off vertical → nonzero gravity torque
        let mut rig = boot(&robot, None, Some(&q0));
        let cmds: Vec<JointCommand> = vec![JointCommand::default(); rig.joints];
        let mut first = None;
        for _ in 0..u64::from(robot.ticks(0.5)) {
            rig.step(&cmds, &GripperCommand::NoGripper);
            rig.bus.queue_poll_override(
                PollAction::Poll {
                    node: 1,
                    kind: PollKind::Encoder,
                },
                1,
            );
            if first.is_none() {
                first = rig.state.nodes[1].position_ticks;
            }
        }
        let first = first.expect("no shoulder position");
        let last = rig.state.nodes[1].position_ticks.unwrap();
        assert!(
            (last - first).abs() > 200,
            "idle arm did not sag under gravity ({first} → {last})"
        );
    }

    /// The endstop stall signatures HOMING.md requires hold on the MuJoCo
    /// plant too (J0: vertical axis, gravity-neutral).
    #[test]
    fn mujoco_endstop_stall_signatures() {
        let robot = par6();
        let dt = robot.robot.tick_dt_s;
        let j = 0usize;
        let jc = &robot.joints[j];
        let h = &robot.homing.joints[j];
        let mut q0 = calibration_pose(&robot);
        q0[j] = jc.limits.hard_max_rad - 0.08; // short approach to the stop
        let mut rig = boot(&robot, None, Some(&q0));
        rig.bus
            .send_limits(
                jc.node_id,
                jc.velocity_limit_ticks_s as f32,
                h.current_ma as f32,
                4,
            )
            .unwrap();
        let sign = if h.direction == 1 { -1.0 } else { 1.0 };
        let drive = JointCommand::velocity((sign * h.speed_ticks_s) as i32, 0);
        let mut cmds = rig.idle_cmds();
        let (_, _, peak_cur) = run_stall_approach(
            &mut rig,
            &mut cmds,
            &GripperCommand::NoGripper,
            Some(j),
            usize::from(jc.node_id),
            drive,
            h.speed_ticks_s,
            h.current_ma,
            u64::from(robot.ticks(h.timeout_s)),
            dt,
        );
        assert!(
            f64::from(peak_cur) >= 0.9 * h.current_ma,
            "mujoco stall current peaked at {peak_cur} mA (limit {} mA)",
            h.current_ma
        );
    }

    fn close_cmd(position: u8) -> GripperCommand {
        GripperCommand::Firmware(FirmwareGripperCommand {
            position,
            speed: 150,
            current_ma: 600,
            activate: true,
            action: true,
            estop: false,
            release_dir: false,
        })
    }

    /// Read the boot wire positions (one velocity-0 tick produces motion
    /// replies), then return position-hold commands for them.
    fn hold_commands(rig: &mut Rig, robot: &RobotConfig) -> Vec<JointCommand> {
        let zero: Vec<JointCommand> = vec![JointCommand::velocity(0, 0); rig.joints];
        rig.step(&zero, &GripperCommand::FirmwarePoll);
        rig.step(&zero, &GripperCommand::FirmwarePoll);
        robot
            .joints
            .iter()
            .map(|jc| {
                let pos = rig.state.nodes[usize::from(jc.node_id)]
                    .position_ticks
                    .expect("boot position");
                // The hold shape: speed 0 — the channel is a velocity
                // feedforward, and a standing one would drive the joint
                // off the held pose.
                JointCommand::position(pos, 0, 0)
            })
            .collect()
    }

    /// The grasp scenario end to end through the REAL status path:
    /// closing on the scene's free object jams the jaws mid-travel and
    /// the cmd-60 reply reports DetectedClosing at the commanded pressing
    /// current; opening away reports ReachedNoObject. No MuJoCo state is
    /// inspected — only decoded bus replies.
    #[test]
    fn mujoco_grasp_detected_through_status_bits() {
        let robot = par6();
        let gripper = msg_gripper();
        let mut rig = boot(&robot, Some(&gripper), Some(&GRASP_POSE));
        let cmds = hold_commands(&mut rig, &robot);

        // Ring down the boot transient with the jaws held open: engaging
        // the position hold from a cold start wobbles the wrist enough to
        // sweep the jaws centimetres, which would bat the object off its
        // pedestal if the close ran through it.
        for _ in 0..u64::from(robot.ticks(1.5)) {
            rig.step(&cmds, &close_cmd(20));
        }

        // Close on the object (per-tick replay, homing-style).
        for _ in 0..u64::from(robot.ticks(2.0)) {
            rig.step(&cmds, &close_cmd(252));
        }
        let r = rig.state.gripper.reply.expect("no gripper reply");
        assert_eq!(
            r.object_detection,
            ObjectDetection::DetectedClosing,
            "no object detected while closing (reply {r:?})"
        );
        assert!(
            !r.action_status,
            "still reported moving while pressing the object"
        );
        assert!(
            r.position > 100 && r.position < 240,
            "jam position byte {} not in mid-travel — jaws passed through or \
             never reached the object",
            r.position
        );
        assert_eq!(r.current_ma, 600, "pressing current is the commanded limit");
        // Pressing is stable: the jam position holds under continued replay.
        let jam = r.position;
        for _ in 0..u64::from(robot.ticks(0.5)) {
            rig.step(&cmds, &close_cmd(252));
        }
        let r = rig.state.gripper.reply.unwrap();
        assert_eq!(r.object_detection, ObjectDetection::DetectedClosing);
        assert!(
            (i16::from(r.position) - i16::from(jam)).abs() <= 2,
            "jam position drifted while pressing ({jam} → {})",
            r.position
        );

        // Open away from the object: free travel completes, no detection.
        for _ in 0..u64::from(robot.ticks(2.0)) {
            rig.step(&cmds, &close_cmd(20));
        }
        let r = rig.state.gripper.reply.unwrap();
        assert_eq!(r.position, 20, "open move did not complete");
        assert_eq!(r.object_detection, ObjectDetection::ReachedNoObject);
    }

    /// Identical tick/command streams — including a contact grasp — must
    /// produce bit-identical state streams.
    #[test]
    fn mujoco_streams_are_bit_identical() {
        fn run() -> Vec<BusState> {
            let robot = par6();
            let gripper = msg_gripper();
            let mut rig = boot(&robot, Some(&gripper), Some(&GRASP_POSE));
            let cmds = hold_commands(&mut rig, &robot);
            let mut states = Vec::new();
            for t in 1..=300u64 {
                let g = if t < 20 {
                    GripperCommand::FirmwarePoll
                } else {
                    close_cmd(252)
                };
                rig.step(&cmds, &g);
                states.push(rig.state.clone());
            }
            states
        }
        let a = run();
        let b = run();
        for (t, (sa, sb)) in a.iter().zip(&b).enumerate() {
            assert!(sa == sb, "mujoco streams diverge at tick {}", t + 1);
        }
    }
}
