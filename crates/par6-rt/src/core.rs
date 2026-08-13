//! The tick-loop assembly: [`RtCore`] wires the per-tick modules
//! (dispatch laws, homing FSM, error latch, e-stop debounce, timing
//! bands, EXEC playback) over a [`DriverBus`] in the \[OURS\] phase order
//! from spec/RT.md — measure-then-command within one tick:
//!
//! 1. `begin_tick` — advance the bus time base
//! 2. GPIO read + debounce (e-stop condition)
//! 3. loop-timing stats → degradation bands
//! 4. boot one-shots (tick-8 selfcheck + IDLE request, vendor config
//!    re-sends at ticks 50/150/300)
//! 5. at most ONE external command (via the [`CommandSource`] seam)
//! 6. RX drain → state pipeline (motor arrays → joint derivation →
//!    filtered history)
//! 7. freshness / reconnect / error checks → reactions (DISABLED,
//!    ACTIVE_ERROR, homing abort, auto ACTIVE_ERROR→IDLE recovery)
//! 8. mode dispatch (setpoint law → commit) → single TX per joint +
//!    gripper slot + telemetry poll slot
//! 9. clear-sequence settle countdown
//! 10. snapshot publish (triple buffer)
//!
//! [`RtCore::tick`] is the TESTABLE core: time enters only as the
//! caller-supplied measured period, so tests drive virtual ticks
//! deterministically. The thin real-time wrapper (absolute-deadline
//! `clock_nanosleep`, SCHED_FIFO) lives in [`crate::rt`].
//!
//! Everything on the tick path is preallocated in [`RtCore::new`];
//! `tick` allocates nothing (asserted by the counting-allocator test).

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;

use par6_bus::spectral::{torque_to_ma_factor, JointConversion};
use par6_bus::{
    BusError, BusState, DriverBus, Freshness, GripperCommand, JointCommand, NodeId, Pack,
};
use par6_config::{ConfigBundle, ControlMode, KtSource};

use crate::dispatch::{self, CommandMirror, JointSetpoint};
use crate::errors::ErrorManager;
use crate::exec::{ExecPlayback, ExecTick};
use crate::gpio::{EstopGpio, EstopMonitor};
use crate::gravity::GravityModel;
use crate::homing::{HomingSystem, SeqStatus};
use crate::hooks::{
    CommandSource, FlashMarker, ForwardKin, JogEngine, RtCommand, SettlePolicy, StreamTracker,
};
use crate::ring::SampleConsumer;
use crate::snapshot::{snapshot_channel, SnapshotReader, SnapshotWriter};
use crate::state::{
    ArmState, ErrorCode, JogStatus, Mode, StateSnapshot, StreamStatus, StreamSubstate,
};
use crate::timing::{LoopHealth, LoopTiming};
use crate::MAX_JOINTS;

/// Boot one-shot: bus-scan selfcheck, then request IDLE (exit BOOTING).
const BOOT_SELFCHECK_TICK: u64 = 8;
/// Vendor boot workaround: full config re-sends at these ticks (may be
/// dropped after HIL validation — spec/RT.md).
const BOOT_CONFIG_RESEND_TICKS: [u64; 3] = [50, 150, 300];
/// Clear_Error frame repeats per faulted node during the clear sequence.
const CLEAR_ERROR_REPEATS: u8 = 3;
/// EXEC link watchdog: heartbeat silence while samples pending that
/// latches `EXEC_LINK_LOST` \[s\].
const EXEC_HEARTBEAT_TIMEOUT_S: f64 = 0.5;
/// First-order EMA coefficient for the `*_filtered` measured-state
/// mirrors (light smoothing for telemetry/external-torque estimation).
const MEAS_FILTER_ALPHA: f64 = 0.2;
/// Gripper slot index in per-joint error keys (`J6:` = the gripper node).
const GRIPPER_ERR_IDX: u8 = MAX_JOINTS as u8;
/// Minimum spacing between two bus-fault log lines from the RT thread
/// \[s\] (see [`FaultLog`]).
const BUS_FAULT_LOG_PERIOD_S: f64 = 1.0;

/// Rate limiter for one RT-thread failure log site: the first failure
/// after a healthy tick is logged, then at most one line per
/// [`BUS_FAULT_LOG_PERIOD_S`] until the site succeeds again.
///
/// A bus failure is permanent while the link is down (`send`/`recv` map
/// ENETDOWN/ENOBUFS straight to `LinkDown`/`TxQueueFull`) and the tick
/// keeps commanding through `ACTIVE_ERROR`, so an ungated `warn!` takes
/// the logger's writer lock and issues a `write(2)` on EVERY tick of a
/// SCHED_FIFO 99 thread. Under systemd that fd is a pipe to journald: a
/// stalled reader then blocks the tick loop instead of degrading it, and
/// even when it does not, the per-tick syscall pushes p99 into the
/// critical band and latches `LOOP_CRITICAL` against the wrong subsystem.
#[derive(Debug)]
struct FaultLog {
    period_ticks: u64,
    /// Tick of the last emitted line; `None` while the site is healthy.
    last: Option<u64>,
    suppressed: u32,
}

impl FaultLog {
    fn new(period_ticks: u64) -> Self {
        Self {
            period_ticks,
            last: None,
            suppressed: 0,
        }
    }

    /// Whether this failure may be logged now, and how many failures were
    /// suppressed since the previous line.
    fn admit(&mut self, tick: u64) -> Option<u32> {
        match self.last {
            Some(last) if tick.saturating_sub(last) < self.period_ticks => {
                self.suppressed = self.suppressed.saturating_add(1);
                None
            }
            _ => {
                self.last = Some(tick);
                Some(std::mem::take(&mut self.suppressed))
            }
        }
    }

    /// The site succeeded: the next failure is a fresh edge.
    fn healthy(&mut self) {
        self.last = None;
        self.suppressed = 0;
    }
}

/// The three bus-failure log sites on the tick path, throttled
/// independently so a permanent TX fault cannot hide a new RX one.
#[derive(Debug)]
struct BusFaultLogs {
    rx: FaultLog,
    joint_tx: FaultLog,
    gripper_tx: FaultLog,
}

impl BusFaultLogs {
    fn new(period_ticks: u64) -> Self {
        Self {
            rx: FaultLog::new(period_ticks),
            joint_tx: FaultLog::new(period_ticks),
            gripper_tx: FaultLog::new(period_ticks),
        }
    }
}

/// Per-joint torque-scale inputs, kept so the boot kt resolution can
/// rebuild [`RtCore::torque_ma_factor`] around a driver-reported kt.
#[derive(Debug, Clone, Copy)]
struct TorqueCal {
    gear_ratio: f64,
    gear_efficiency: f64,
    kt_nm_a: f64,
    dir: u8,
}

impl TorqueCal {
    fn factor(&self, kt_nm_a: f64) -> f64 {
        torque_to_ma_factor(self.gear_ratio, self.gear_efficiency, kt_nm_a, self.dir)
    }
}

/// Why a mode request was refused (logged; requests are fire-and-forget).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GateRefusal {
    /// The transition table forbids `from → to`.
    InvalidTransition,
    /// Target requires `state == ENABLED`.
    NotEnabled,
    /// Target requires an empty hard-error latch.
    ErrorsActive,
    /// Target is a motion mode and the robot is not homed (also raises
    /// the `NOT_HOMED` warning).
    NotHomed,
    /// FLASHING requires a prior human park assertion.
    ParkAssertionRequired,
    /// The mode's output law is not implemented yet (HAND_GUIDING,
    /// IMPEDANCE) — explicit refusal, never a silent no-op mode.
    NotImplemented,
}

/// Construction failure.
#[derive(Debug, thiserror::Error)]
pub enum CoreError {
    /// The robot config's joint count does not match the fixed RT
    /// dimension.
    #[error("robot config has {got} joints; par6-rt is dimensioned for {expected}")]
    JointCount {
        /// Joints in the config.
        got: usize,
        /// Compile-time RT dimension ([`MAX_JOINTS`]).
        expected: usize,
    },
    /// Bus boot configuration failed.
    #[error("bus boot configuration failed: {0}")]
    Bus(#[from] BusError),
}

/// The pluggable seams [`RtCore`] runs on. `par6d` wires the real
/// engines (par6-motion, pinokin FK, GPIO chips, command-plane queue);
/// tests and the sim runtime use the built-ins from [`crate::hooks`],
/// [`crate::gravity`], [`crate::gpio`].
pub struct RtHooks {
    /// Gravity model G(q) (computed every tick, published always).
    pub gravity: Box<dyn GravityModel>,
    /// Jog ramp engine.
    pub jog: Box<dyn JogEngine>,
    /// Streaming target tracker (rate limiter).
    pub stream: Box<dyn StreamTracker>,
    /// EXEC completion policy.
    pub settle: Box<dyn SettlePolicy>,
    /// ESTOP_1 GPIO line.
    pub estop: Box<dyn EstopGpio>,
    /// FLASHING-exit flash marker.
    pub flash: Box<dyn FlashMarker>,
    /// External command intake (one consumed per tick).
    pub commands: Box<dyn CommandSource>,
    /// TCP forward kinematics for the snapshot.
    pub fk: Box<dyn ForwardKin>,
    /// Consumer half of the planner sample ring.
    pub samples: SampleConsumer,
}

/// Command-plane handle feeding the EXEC link watchdog. Cloneable and
/// wait-free; call [`feed`](Self::feed) at ≥the heartbeat rate while an
/// EXEC program is queued.
#[derive(Debug, Clone)]
pub struct ExecHeartbeat {
    flag: Arc<AtomicBool>,
}

impl ExecHeartbeat {
    /// Mark the link alive for this heartbeat period.
    pub fn feed(&self) {
        self.flag.store(true, Ordering::Relaxed);
    }
}

/// Command-plane handle for streaming setpoints: latest-wins slot (the
/// RT applies only the newest target per tick; superseded targets count
/// toward the published discard percentage).
pub struct StreamInput {
    writer: SnapshotWriter<[f64; MAX_JOINTS]>,
    sent: Arc<AtomicU64>,
}

impl StreamInput {
    /// Publish a new joint-position target \[rad\].
    pub fn send(&mut self, q_target: &[f64; MAX_JOINTS]) {
        self.writer.publish(q_target);
        self.sent.fetch_add(1, Ordering::Relaxed);
    }
}

/// The non-RT halves of the core's channels, handed to `par6d` wiring.
pub struct RtHandles {
    /// Reader half of the per-tick state snapshot (command plane).
    pub snapshots: SnapshotReader<StateSnapshot>,
    /// EXEC link heartbeat feeder.
    pub heartbeat: ExecHeartbeat,
    /// Streaming setpoint input.
    pub stream: StreamInput,
}

/// The RT core: all per-tick state, preallocated. Generic over the bus
/// backend so the identical loop runs on SocketCAN, the closed-loop sim,
/// and the loopback used by tests.
pub struct RtCore<B: DriverBus> {
    dt: f64,
    tick: u64,
    bus: B,
    bus_state: BusState,

    // Per-joint wire conversion + calibration.
    conv: [JointConversion; MAX_JOINTS],
    sector_done: [bool; MAX_JOINTS],
    torque_ma_factor: [f64; MAX_JOINTS],
    torque_cal: [TorqueCal; MAX_JOINTS],
    /// `kt_source = "auto"`: adopt the drivers' own kt at the boot
    /// one-shot, before anything can be enabled.
    kt_auto: bool,
    node_of: [NodeId; MAX_JOINTS],
    gripper_node: NodeId,
    has_can_gripper: bool,
    jog_pack: Pack,

    // Seams.
    gravity: Box<dyn GravityModel>,
    jog: Box<dyn JogEngine>,
    stream: Box<dyn StreamTracker>,
    exec: ExecPlayback,
    estop: EstopMonitor,
    flash: Box<dyn FlashMarker>,
    commands: Box<dyn CommandSource>,
    fk: Box<dyn ForwardKin>,

    // Subsystems.
    homing: HomingSystem,
    errors: ErrorManager,
    timing: LoopTiming,
    bus_faults: BusFaultLogs,

    // Bus-failure accounting: the backend PROPAGATES send/drain errors
    // (spec/CAN.md stance) and the tick loop counts every one into the
    // published loop stats. Consecutive-failure streaks drive the
    // disconnect latch below.
    bus_tx_failures: u32,
    bus_rx_failures: u32,
    tx_fail_streak: u32,
    gripper_tx_fail_streak: u32,
    /// Consecutive failed ticks after which a TX streak latches the
    /// per-node disconnect errors — the freshness lost window, so an
    /// outbound-dead link disables on the same clock as a silent one.
    tx_fault_latch_ticks: u32,

    // State-machine variables (spec/RT.md: mode, state, homed, errors).
    mode: Mode,
    state: ArmState,
    enable_seq: u64,
    homed: bool,
    soft_estop: bool,
    hw_estop: bool,
    park_asserted: bool,
    gravity_comp: bool,
    not_homed_refused: bool,

    // Measured state pipeline.
    q: [f64; MAX_JOINTS],
    qd: [f64; MAX_JOINTS],
    tau: [f64; MAX_JOINTS],
    q_filt: [f64; MAX_JOINTS],
    qd_filt: [f64; MAX_JOINTS],
    tau_filt: [f64; MAX_JOINTS],
    filters_seeded: bool,

    // Per-tick compute buffers.
    g: [f64; MAX_JOINTS],
    setpoints: [JointSetpoint; MAX_JOINTS],
    cmds: [JointCommand; MAX_JOINTS],
    mirror: CommandMirror,
    q_target: [f64; MAX_JOINTS],
    qd_target: [f64; MAX_JOINTS],
    scratch_q: [f64; MAX_JOINTS],
    scratch_qd: [f64; MAX_JOINTS],
    scratch_tau: [f64; MAX_JOINTS],
    gripper_cmd: GripperCommand,
    homing_gcmd: GripperCommand,

    // Jog live state.
    jog_active: bool,
    jog_joint: u8,
    jog_blocked: u16,

    // EXEC link watchdog.
    heartbeat: Arc<AtomicBool>,
    hb_silence: u32,
    hb_timeout_ticks: u32,

    // Streaming.
    stream_rx: SnapshotReader<[f64; MAX_JOINTS]>,
    stream_sent: Arc<AtomicU64>,
    stream_last_rx_tick: u64,
    stream_timeout_ticks: u32,
    stream_window_ticks: u32,
    stream_window_pos: u32,
    stream_window_applied: u32,
    stream_sent_base: u64,
    stream_success: f32,
    stream_discard: f32,

    // Snapshot.
    writer: SnapshotWriter<StateSnapshot>,
    snap: StateSnapshot,
}

impl<B: DriverBus> RtCore<B> {
    /// Build the core, boot-configure the bus (config repeats + scan per
    /// the robot config), and hand back the command-plane handles. All
    /// tick-path storage is allocated here.
    pub fn new(
        bundle: &ConfigBundle,
        mut bus: B,
        hooks: RtHooks,
    ) -> Result<(Self, RtHandles), CoreError> {
        let robot = &bundle.robot;
        if robot.joints.len() != MAX_JOINTS {
            return Err(CoreError::JointCount {
                got: robot.joints.len(),
                expected: MAX_JOINTS,
            });
        }
        let dt = robot.robot.tick_dt_s;
        let gripper = bundle.active_gripper().filter(|g| g.driver.is_some());
        bus.boot_configure(robot, gripper, robot.bus.boot_config_repeats)?;

        let conv: [JointConversion; MAX_JOINTS] =
            std::array::from_fn(|i| JointConversion::from_config(&robot.joints[i]));
        let torque_cal: [TorqueCal; MAX_JOINTS] = std::array::from_fn(|i| {
            let j = &robot.joints[i];
            TorqueCal {
                gear_ratio: j.gear_ratio,
                gear_efficiency: j.gear_efficiency,
                kt_nm_a: j.kt_nm_a,
                dir: j.dir,
            }
        });
        // Config kt until the boot one-shot resolves the drivers' own
        // (`kt_source = "auto"`), which happens while still BOOTING.
        let torque_ma_factor: [f64; MAX_JOINTS] =
            std::array::from_fn(|i| torque_cal[i].factor(torque_cal[i].kt_nm_a));
        let (writer, snapshots) = snapshot_channel::<StateSnapshot>();
        let (stream_tx, stream_rx) = snapshot_channel::<[f64; MAX_JOINTS]>();
        let heartbeat = Arc::new(AtomicBool::new(false));
        let stream_sent = Arc::new(AtomicU64::new(0));
        let has_can_gripper = gripper.is_some();
        let gripper_cmd = if has_can_gripper {
            GripperCommand::FirmwarePoll
        } else {
            GripperCommand::NoGripper
        };
        let core = Self {
            dt,
            tick: 0,
            bus,
            bus_state: BusState::new(),
            conv,
            sector_done: [false; MAX_JOINTS],
            torque_ma_factor,
            torque_cal,
            kt_auto: robot.robot.kt_source == KtSource::Auto,
            node_of: std::array::from_fn(|i| robot.joints[i].node_id),
            gripper_node: robot.bus.gripper_node,
            has_can_gripper,
            jog_pack: match robot.jog.control_mode {
                ControlMode::Pid => Pack::Pid,
                ControlMode::Pd => Pack::Pd,
            },
            gravity: hooks.gravity,
            jog: hooks.jog,
            stream: hooks.stream,
            exec: ExecPlayback::new(hooks.samples, hooks.settle),
            estop: EstopMonitor::new(hooks.estop),
            flash: hooks.flash,
            commands: hooks.commands,
            fk: hooks.fk,
            homing: HomingSystem::new(bundle),
            errors: ErrorManager::new(dt),
            timing: LoopTiming::new(dt, robot.loop_timing()),
            bus_faults: BusFaultLogs::new(u64::from(robot.ticks(BUS_FAULT_LOG_PERIOD_S).max(1))),
            bus_tx_failures: 0,
            bus_rx_failures: 0,
            tx_fail_streak: 0,
            gripper_tx_fail_streak: 0,
            tx_fault_latch_ticks: robot.ticks(robot.bus.lost_s).max(1),
            mode: Mode::Booting,
            state: ArmState::Disabled,
            enable_seq: 0,
            homed: false,
            soft_estop: false,
            hw_estop: false,
            park_asserted: false,
            gravity_comp: true,
            not_homed_refused: false,
            q: [0.0; MAX_JOINTS],
            qd: [0.0; MAX_JOINTS],
            tau: [0.0; MAX_JOINTS],
            q_filt: [0.0; MAX_JOINTS],
            qd_filt: [0.0; MAX_JOINTS],
            tau_filt: [0.0; MAX_JOINTS],
            filters_seeded: false,
            g: [0.0; MAX_JOINTS],
            setpoints: [JointSetpoint::zero_velocity(); MAX_JOINTS],
            cmds: [JointCommand::idle(); MAX_JOINTS],
            mirror: CommandMirror::default(),
            q_target: [0.0; MAX_JOINTS],
            qd_target: [0.0; MAX_JOINTS],
            scratch_q: [0.0; MAX_JOINTS],
            scratch_qd: [0.0; MAX_JOINTS],
            scratch_tau: [0.0; MAX_JOINTS],
            gripper_cmd,
            homing_gcmd: gripper_cmd,
            jog_active: false,
            jog_joint: 0,
            jog_blocked: 0,
            heartbeat: heartbeat.clone(),
            hb_silence: 0,
            hb_timeout_ticks: robot.ticks(EXEC_HEARTBEAT_TIMEOUT_S).max(1),
            stream_rx,
            stream_sent: stream_sent.clone(),
            stream_last_rx_tick: 0,
            // The watchdog is READ in phase 7 and FED in phase 8 (the
            // setpoint intake), so even a stream that lands a fresh
            // target on every single tick always shows one tick of age
            // at the check. A one-tick window is therefore unsatisfiable
            // — it latches RTI_LINK_LOST on the second tick of every
            // stream, however fast the client is — and two ticks is the
            // smallest window a live stream can meet. Tick rates where
            // `round(command_timeout_s / dt) >= 2` keep the configured
            // window exactly; only rates whose tick period approaches
            // the window itself are raised.
            stream_timeout_ticks: robot.ticks(robot.stream.command_timeout_s).max(2),
            stream_window_ticks: robot.ticks(robot.stream.success_window_s).max(1),
            stream_window_pos: 0,
            stream_window_applied: 0,
            stream_sent_base: 0,
            stream_success: 0.0,
            stream_discard: 0.0,
            writer,
            snap: StateSnapshot::default(),
        };
        Ok((
            core,
            RtHandles {
                snapshots,
                heartbeat: ExecHeartbeat { flag: heartbeat },
                stream: StreamInput {
                    writer: stream_tx,
                    sent: stream_sent,
                },
            },
        ))
    }

    /// Current operating mode.
    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// Tick period \[s\] from the robot config.
    pub fn tick_dt_s(&self) -> f64 {
        self.dt
    }

    /// Whether the home references are valid.
    pub fn homed(&self) -> bool {
        self.homed
    }

    /// The bus backend (sim scenario hooks, backend switching in `par6d`).
    pub fn bus_mut(&mut self) -> &mut B {
        &mut self.bus
    }

    /// Simulator/teleport path: declare the home references valid (or
    /// invalidate them) without running the homing sequence. `par6d`
    /// exposes this only in simulator mode (`SYS_NOT_SIMULATOR` gate) —
    /// on hardware, `homed` is owned by the homing FSM.
    pub fn set_homed(&mut self, homed: bool) {
        self.homed = homed;
        if homed {
            self.not_homed_refused = false;
        }
    }

    /// Simulator/teleport path: re-base joint `joint`'s wire conversion
    /// so encoder reading `motor_ticks` maps to `joint_rad`, and seed the
    /// measured mirrors at that value. Stale wire readings from before
    /// the re-reference are dropped so they are never re-interpreted
    /// under the new mapping. `par6d` calls this after re-seeding the sim
    /// plant; on hardware, references are owned by the homing FSM.
    pub fn set_joint_reference(&mut self, joint: usize, motor_ticks: i32, joint_rad: f64) {
        if joint >= MAX_JOINTS {
            return;
        }
        self.conv[joint].set_home(motor_ticks, joint_rad);
        self.sector_done[joint] = true;
        let node = &mut self.bus_state.nodes[usize::from(self.node_of[joint])];
        node.position_ticks = None;
        node.speed_ticks_s = None;
        self.q[joint] = joint_rad;
        self.qd[joint] = 0.0;
        self.q_filt[joint] = joint_rad;
        self.qd_filt[joint] = 0.0;
    }

    /// Replace the EXEC completion policy (takes effect at the next
    /// command boundary) — the `set_completion_policy` follow-through
    /// from the command plane.
    pub fn set_settle_policy(&mut self, policy: Box<dyn SettlePolicy>) {
        self.exec.set_policy(policy);
    }

    /// Reset the loop timing statistics (the `reset_loop_stats`
    /// follow-through); the warmup gate re-arms.
    pub fn reset_loop_stats(&mut self) {
        self.timing.reset();
    }

    /// One tick of the core. `period_s` is the measured loop period (the
    /// virtual-tick tests feed the nominal `dt`); `overrun` marks a
    /// missed deadline as measured by the caller.
    pub fn tick(&mut self, period_s: f64, overrun: bool) {
        self.tick += 1;
        self.bus.begin_tick(self.tick);

        // Phase 2: e-stop GPIO read + debounce (reaction in phase 7).
        self.hw_estop = self.estop.pressed();

        // Phase 3: loop-period statistics and degradation bands.
        let health = self.timing.record(period_s, overrun);

        // Phase 4: boot one-shots.
        self.boot_oneshots();

        // Phase 5: at most one external command.
        if let Some(cmd) = self.commands.poll() {
            self.apply_command(cmd);
        }

        // Phase 6: RX drain → state pipeline (measure-then-command).
        self.drain_and_derive();

        // Gravity: computed every tick, published always.
        self.gravity.gravity(&self.q, &mut self.g);

        // Phase 7: error checks and reactions.
        self.check_errors(health);

        // Phase 8: mode dispatch → TX → poll slot.
        self.dispatch_and_send();

        // Phase 9: clear-sequence settle countdown.
        self.errors.tick();

        // Phase 10: snapshot publish.
        self.publish();
    }

    // ------------------------------------------------------------ boot

    fn boot_oneshots(&mut self) {
        if self.tick == BOOT_SELFCHECK_TICK {
            let connected = self.bus.connected_nodes();
            for i in 0..MAX_JOINTS {
                if connected & (1 << u16::from(self.node_of[i])) == 0 {
                    self.errors.latch(ErrorCode::CanLost, Some(i as u8));
                }
            }
            if self.has_can_gripper && connected & (1 << u16::from(self.gripper_node)) == 0 {
                self.errors.latch(ErrorCode::CanLost, Some(GRIPPER_ERR_IDX));
            }
            if self.kt_auto {
                self.adopt_driver_kt();
            }
            if self.mode == Mode::Booting {
                let _ = self.request_mode(Mode::Idle);
            }
        }
        if BOOT_CONFIG_RESEND_TICKS.contains(&self.tick) && !self.bus.is_silent() {
            for i in 0..MAX_JOINTS {
                let _ = self.bus.resend_node_config(self.node_of[i], 1);
            }
            if self.has_can_gripper {
                let _ = self.bus.resend_node_config(self.gripper_node, 1);
            }
        }
    }

    /// Rebuild the torque scale around the drivers' own torque constants
    /// (`kt_source = "auto"`, spec/CAN.md boot step 3): the fetched value
    /// governs, config is the fallback for a driver that did not answer.
    ///
    /// Boot one-shot, by which tick the backend's boot replies have been
    /// published through `drain_rx`. It runs while the core is still
    /// BOOTING — enable is refused there — so no torque command can have
    /// gone out on the config factor. `nodes[i].kt_nm_a` rides the
    /// snapshot, so `Some` vs `None` is the per-joint provenance.
    ///
    /// A non-positive, non-finite, or out-of-family reply is REJECTED,
    /// not adopted: the factor divides by kt, so garbage here scales
    /// every commanded torque, every gravity feedforward and every
    /// reported `tau`. "Out of family" means more than
    /// [`KT_FAMILY_FACTOR`]× from the config value in either direction —
    /// a real driver's calibration differs from the config by percent,
    /// not by multiples, and a multiple-off answer is a corrupt reply or
    /// a mis-flashed driver, either of which the config value serves
    /// better than adopting.
    fn adopt_driver_kt(&mut self) {
        const KT_FAMILY_FACTOR: f64 = 3.0;
        for i in 0..MAX_JOINTS {
            let cal = self.torque_cal[i];
            let kt = self.bus_state.nodes[usize::from(self.node_of[i])]
                .kt_nm_a
                .map(f64::from)
                .filter(|kt| kt.is_finite() && *kt > 0.0)
                .filter(|kt| {
                    let in_family = *kt <= cal.kt_nm_a * KT_FAMILY_FACTOR
                        && *kt >= cal.kt_nm_a / KT_FAMILY_FACTOR;
                    if !in_family {
                        log::warn!(
                            "J{i}: driver kt {kt} Nm/A rejected (config {} Nm/A; \
                             outside the {KT_FAMILY_FACTOR}x family band)",
                            cal.kt_nm_a
                        );
                    }
                    in_family
                });
            match kt {
                Some(kt) => {
                    self.torque_ma_factor[i] = cal.factor(kt);
                    log::info!("J{i}: kt {kt} Nm/A (driver)");
                }
                None => log::warn!(
                    "J{i}: kt {} Nm/A (config; no usable driver reply)",
                    cal.kt_nm_a
                ),
            }
        }
    }

    // ------------------------------------------------------------ commands

    fn estop_condition(&self) -> bool {
        self.hw_estop || self.soft_estop
    }

    fn apply_command(&mut self, cmd: RtCommand) {
        match cmd {
            RtCommand::SetMode(target) => {
                if let Err(e) = self.request_mode(target) {
                    log::warn!("mode request {target:?} refused: {e:?}");
                }
            }
            RtCommand::Enable => {
                // Counted whether granted or refused: the counter is what
                // ties the published `state` to THIS request.
                self.enable_seq += 1;
                if self.mode == Mode::Booting {
                    // ENABLED before the bus selfcheck lets the command
                    // plane accept motion the transition table then
                    // refuses — every working mode is unreachable from
                    // BOOTING, so the command dies at setup instead.
                    log::warn!("enable refused: the core is still BOOTING");
                } else if self.estop_condition() || self.errors.any_hard() {
                    log::warn!("enable refused: e-stop or errors active");
                } else {
                    self.state = ArmState::Enabled;
                }
            }
            RtCommand::Disable => self.state = ArmState::Disabled,
            RtCommand::ClearErrors => self.begin_clear(),
            RtCommand::SetSoftEstop(on) => self.soft_estop = on,
            RtCommand::AssertParked => {
                log::info!("human park assertion received (arms FLASHING entry)");
                self.park_asserted = true;
            }
            RtCommand::Jog { joint, signed_pct } => {
                if self.mode == Mode::Jog && usize::from(joint) < MAX_JOINTS {
                    self.jog.command(usize::from(joint), signed_pct);
                    self.jog_active = true;
                    self.jog_joint = joint;
                }
            }
            RtCommand::JogRelease => {
                self.jog.release();
                self.jog_active = false;
            }
            RtCommand::ExecSetPaused(paused) => self.exec.set_paused(paused),
            RtCommand::ExecFlush => {
                let n = self.exec.flush();
                log::info!("EXEC flush discarded {n} samples");
            }
            RtCommand::Gripper(fw) => {
                if self.has_can_gripper {
                    self.gripper_cmd = GripperCommand::Firmware(fw);
                }
            }
            RtCommand::GripperCalibrate => {
                if self.has_can_gripper {
                    self.gripper_cmd = GripperCommand::Calibrate;
                }
            }
            RtCommand::SetGravityComp(on) => self.gravity_comp = on,
        }
    }

    /// Mode transition request with the spec/RT.md gates, in order:
    /// maintenance park assertion, SAFETY_STOP/IDLE unconditional
    /// reachability, then enabled ∧ no-errors ∧ homed-if-motion.
    fn request_mode(&mut self, target: Mode) -> Result<(), GateRefusal> {
        if target == self.mode {
            return Ok(());
        }
        // Never request targets: BOOTING is boot-only, ACTIVE_ERROR is a
        // reaction state.
        if matches!(target, Mode::Booting | Mode::ActiveError) {
            return Err(GateRefusal::InvalidTransition);
        }
        if matches!(target, Mode::HandGuiding | Mode::Impedance) {
            return Err(GateRefusal::NotImplemented);
        }
        let allowed = match self.mode {
            Mode::Booting => matches!(target, Mode::Idle | Mode::SafetyStop),
            Mode::Idle => true,
            Mode::ActiveError => matches!(target, Mode::Idle | Mode::Flashing | Mode::SafetyStop),
            Mode::SafetyStop => target == Mode::Idle,
            // Working modes (incl. FLASHING as the maintenance working
            // mode): only IDLE and SAFETY_STOP.
            _ => matches!(target, Mode::Idle | Mode::SafetyStop),
        };
        if !allowed {
            return Err(GateRefusal::InvalidTransition);
        }
        match target {
            // →IDLE always allowed; SAFETY_STOP always reachable, no checks.
            Mode::Idle | Mode::SafetyStop => {}
            // Maintenance exemption: FLASHING skips enabled/errors/homed,
            // gated ONLY on the human park assertion.
            Mode::Flashing => {
                if !self.park_asserted {
                    return Err(GateRefusal::ParkAssertionRequired);
                }
            }
            _ => {
                if self.state != ArmState::Enabled {
                    return Err(GateRefusal::NotEnabled);
                }
                if self.errors.any_hard() {
                    return Err(GateRefusal::ErrorsActive);
                }
                let needs_home = matches!(target, Mode::Jog | Mode::Stream | Mode::Exec);
                if needs_home && !self.homed {
                    self.not_homed_refused = true;
                    return Err(GateRefusal::NotHomed);
                }
            }
        }
        self.enter_mode(target);
        Ok(())
    }

    fn enter_mode(&mut self, target: Mode) {
        self.leave_mode(target);
        match target {
            Mode::Homing => self.homing.start(&mut self.bus),
            Mode::Jog => {
                self.jog.activate(&self.q);
                self.jog_active = false;
                self.jog_blocked = 0;
            }
            Mode::Exec => {
                self.exec.activate(&self.q);
                self.hb_silence = 0;
            }
            Mode::Stream => {
                self.stream.activate(&self.q);
                self.stream_last_rx_tick = self.tick;
                self.stream_window_pos = 0;
                self.stream_window_applied = 0;
                self.stream_sent_base = self.stream_sent.load(Ordering::Relaxed);
                self.stream_success = 0.0;
                self.stream_discard = 0.0;
            }
            Mode::Flashing => {
                log::info!("entering FLASHING: bus-silent maintenance window");
                self.bus.set_silent(true);
            }
            _ => {}
        }
        self.mode = target;
    }

    /// Exit side effects of the CURRENT mode, given the target.
    fn leave_mode(&mut self, target: Mode) {
        match self.mode {
            Mode::Homing if self.homing.active() => {
                // A user-requested exit mid-sequence aborts: references
                // are partially applied, so homing is invalidated.
                self.homing.abort(&mut self.bus);
                self.homed = false;
            }
            Mode::Flashing => {
                self.bus.set_silent(false);
                // The silent window must not read as a mass disconnect.
                self.bus.rebase_freshness();
                if self.flash.flashed() {
                    log::info!("firmware was flashed: homing invalidated");
                    self.homed = false;
                }
            }
            _ => {}
        }
        // The park assertion is one-shot: consumed by FLASHING entry,
        // dropped by any other transition.
        self.park_asserted = false;
        let _ = target;
    }

    /// The user clear sequence (spec/RT.md): Clear_Error ×3 to each
    /// faulted node (+ gripper), stale per-type flags zeroed, lost
    /// latches reset, then the settle countdown that outlasts the poll
    /// cycle before the latch wipes.
    fn begin_clear(&mut self) {
        for entry in self.errors.list().as_slice() {
            let Some(j) = entry.joint else { continue };
            let node = if usize::from(j) < MAX_JOINTS {
                self.node_of[usize::from(j)]
            } else {
                self.gripper_node
            };
            let _ = self.bus.send_clear_error(node, CLEAR_ERROR_REPEATS);
            self.bus.clear_lost_latch(node);
        }
        if self.has_can_gripper {
            let _ = self
                .bus
                .send_clear_error(self.gripper_node, CLEAR_ERROR_REPEATS);
        }
        // Zero the stale per-type flags so only a fresh report (with its
        // live fault bit) can re-latch — fixes the two-press race.
        for node in &mut self.bus_state.nodes {
            node.error_flags = None;
        }
        self.not_homed_refused = false;
        // Subsystem flags that `check_errors` re-asserts every tick have
        // to go with the latch, or the clear is undone before the settle
        // countdown even finishes.
        self.homing.clear_faults();
        self.errors.begin_clear();
    }

    // ------------------------------------------------------------ state

    fn drain_and_derive(&mut self) {
        match self.bus.drain_rx(&mut self.bus_state) {
            Ok(_) => self.bus_faults.rx.healthy(),
            Err(e) => {
                self.bus_rx_failures = self.bus_rx_failures.saturating_add(1);
                if let Some(n) = self.bus_faults.rx.admit(self.tick) {
                    log::warn!("bus RX drain failed: {e} (+{n} suppressed)");
                }
            }
        }
        for i in 0..MAX_JOINTS {
            let node = &self.bus_state.nodes[usize::from(self.node_of[i])];
            if let Some(pos) = node.position_ticks {
                if !self.sector_done[i] {
                    // Boot sector selection from the first (wrapped)
                    // encoder reading, before any position conversion.
                    self.conv[i].determine_sector(pos);
                    self.sector_done[i] = true;
                }
                self.q[i] = self.conv[i].joint_rad(pos);
            }
            if let Some(spd) = node.speed_ticks_s {
                self.qd[i] = self.conv[i].joint_speed_rad_s(f64::from(spd));
            }
            if let Some(cur) = node.current_ma {
                self.tau[i] = f64::from(cur) / self.torque_ma_factor[i];
            }
        }
        if !self.filters_seeded {
            if self.sector_done.iter().all(|&d| d) {
                self.q_filt = self.q;
                self.qd_filt = self.qd;
                self.tau_filt = self.tau;
                self.filters_seeded = true;
            }
        } else {
            for i in 0..MAX_JOINTS {
                self.q_filt[i] += MEAS_FILTER_ALPHA * (self.q[i] - self.q_filt[i]);
                self.qd_filt[i] += MEAS_FILTER_ALPHA * (self.qd[i] - self.qd_filt[i]);
                self.tau_filt[i] += MEAS_FILTER_ALPHA * (self.tau[i] - self.tau_filt[i]);
            }
        }
    }

    // ------------------------------------------------------------ errors

    fn check_errors(&mut self, health: LoopHealth) {
        // E-stop: hardware line (debounced, first-read seeded) and the
        // software flag, distinct hard-latching keys. Motors stay
        // energized — the reaction below is the ACTIVE_ERROR zero-velocity
        // hold, never a CAN ESTOP frame.
        if self.hw_estop {
            self.errors.latch(ErrorCode::Estop, None);
        }
        if self.soft_estop {
            self.errors.latch(ErrorCode::SwEstop, None);
        }

        // Loop degradation bands.
        self.errors.condition(
            ErrorCode::LoopDegraded,
            None,
            health == LoopHealth::Degraded,
        );
        if health == LoopHealth::Critical {
            self.errors.latch(ErrorCode::LoopCritical, None);
        }

        if !self.bus.is_silent() {
            // A sustained TX-failure streak is a disconnect: the arm has
            // stopped receiving commands even if RX freshness still reads
            // green (e.g. a full TX queue while polls trickle through).
            // Latched on the freshness lost window via the per-node
            // disconnect keys. Home references stay valid — the encoders
            // were never lost; if RX actually goes silent too, the
            // freshness path below invalidates homing as usual.
            if self.tx_fail_streak >= self.tx_fault_latch_ticks {
                for i in 0..MAX_JOINTS {
                    self.errors.latch(ErrorCode::CanLost, Some(i as u8));
                }
                if self.has_can_gripper {
                    self.errors.latch(ErrorCode::CanLost, Some(GRIPPER_ERR_IDX));
                }
            }
            if self.has_can_gripper && self.gripper_tx_fail_streak >= self.tx_fault_latch_ticks {
                self.errors.latch(ErrorCode::CanLost, Some(GRIPPER_ERR_IDX));
            }
            // Freshness: stale warns (self-clears), lost latches; any
            // disconnect while homed invalidates homing.
            for i in 0..MAX_JOINTS {
                self.freshness_check(self.node_of[i], i as u8);
            }
            if self.has_can_gripper {
                self.freshness_check(self.gripper_node, GRIPPER_ERR_IDX);
            }
            // Reconnect path: re-send stored config to nodes whose
            // stale→fresh edge happened in this drain.
            let mask = self.bus_state.reconnected_mask;
            if mask != 0 {
                for n in 0..par6_bus::MAX_NODES {
                    if mask & (1 << n) != 0 {
                        let _ = self.bus.resend_node_config(n as NodeId, 1);
                    }
                }
            }
            // Per-type motor fault flags, trusted only while the node's
            // per-frame live fault bit is set (live-bit gating).
            for i in 0..MAX_JOINTS {
                self.motor_flag_check(self.node_of[i], i as u8);
            }
            if self.has_can_gripper {
                self.motor_flag_check(self.gripper_node, GRIPPER_ERR_IDX);
                let g = &self.bus_state.gripper;
                if g.live_error_bit {
                    if let Some(r) = g.reply {
                        if r.temperature_error || r.timeout_error || r.estop_error {
                            self.errors
                                .latch(ErrorCode::GripperFault, Some(GRIPPER_ERR_IDX));
                        }
                    }
                }
            }
        }

        // EXEC link watchdog: heartbeat silence while samples pending.
        if self.mode == Mode::Exec {
            if self.heartbeat.swap(false, Ordering::Relaxed) {
                self.hb_silence = 0;
            } else {
                self.hb_silence = self.hb_silence.saturating_add(1);
            }
            if self.exec.samples_remaining() > 0 && self.hb_silence >= self.hb_timeout_ticks {
                self.errors.latch(ErrorCode::ExecLinkLost, None);
            }
        }

        // Stream watchdog.
        if self.mode == Mode::Stream
            && self.tick.saturating_sub(self.stream_last_rx_tick)
                >= u64::from(self.stream_timeout_ticks)
        {
            self.errors.latch(ErrorCode::RtiLinkLost, None);
        }

        // Homing-failed warnings track the per-joint statuses.
        let statuses = *self.homing.statuses();
        for (i, s) in statuses.iter().enumerate() {
            self.errors.condition(
                ErrorCode::HomingFailed,
                Some(i as u8),
                *s == crate::state::HomingJointStatus::Failed,
            );
        }
        if self.homing.calibration_failed() {
            self.errors
                .latch(ErrorCode::GripperCalibrationFailed, Some(GRIPPER_ERR_IDX));
        }
        self.errors
            .condition(ErrorCode::NotHomed, None, self.not_homed_refused);

        // Reactions.
        if self.errors.any_hard() {
            if self.mode != Mode::Booting {
                self.state = ArmState::Disabled;
            }
            if !matches!(self.mode, Mode::ActiveError | Mode::Flashing) {
                if self.mode == Mode::Homing && self.homing.active() {
                    // Abort: un-home, zero statuses, restore limits/config.
                    self.homing.abort(&mut self.bus);
                    self.homed = false;
                }
                log::warn!("hard error: mode {:?} -> ActiveError", self.mode);
                self.mode = Mode::ActiveError;
            }
        } else if self.mode == Mode::ActiveError {
            // Auto-recovery once the latch is clean (state stays DISABLED
            // until the user re-enables).
            self.mode = Mode::Idle;
        }
    }

    fn freshness_check(&mut self, node: NodeId, err_idx: u8) {
        let f = self.bus.freshness(node);
        self.errors
            .condition(ErrorCode::CanStale, Some(err_idx), f == Freshness::Stale);
        if f == Freshness::Lost {
            self.errors.latch(ErrorCode::CanLost, Some(err_idx));
            if self.homed {
                log::warn!("node {node} disconnected: homing invalidated");
                self.homed = false;
            }
        }
    }

    fn motor_flag_check(&mut self, node: NodeId, err_idx: u8) {
        let n = &self.bus_state.nodes[usize::from(node)];
        if !n.live_error_bit {
            return;
        }
        let Some(flags) = n.error_flags else { return };
        let joint = Some(err_idx);
        let map = [
            (flags.temperature, ErrorCode::Temperature),
            (flags.encoder, ErrorCode::Encoder),
            (flags.vbus, ErrorCode::Vbus),
            (flags.driver, ErrorCode::Driver),
            (flags.velocity, ErrorCode::Velocity),
            (flags.current, ErrorCode::Current),
            (flags.estop, ErrorCode::EstopMotor),
            (flags.watchdog, ErrorCode::Watchdog),
        ];
        for (set, code) in map {
            if set {
                self.errors.latch(code, joint);
            }
        }
    }

    // ------------------------------------------------------------ dispatch

    fn gravity_applied(&self) -> bool {
        self.homed && self.state == ArmState::Enabled && self.gravity_comp
    }

    fn dispatch_and_send(&mut self) {
        if self.mode == Mode::Flashing {
            // Bus-silent: not a single frame, polls included.
            self.mirror = CommandMirror::default();
            return;
        }
        if self.mode == Mode::Homing {
            // SELF_MANAGED: the homing subsystem fills the complete
            // per-joint command array (idle keep-alives on non-active
            // joints) and the gripper slot.
            let status = self.homing.tick(
                &mut self.bus,
                &mut self.bus_state,
                &mut self.conv,
                &mut self.cmds,
                &mut self.homing_gcmd,
            );
            self.mirror = CommandMirror::default();
            self.send_joints();
            self.send_gripper(self.homing_gcmd);
            let _ = self.bus.poll_step();
            match status {
                SeqStatus::Complete => {
                    log::info!("homing sequence complete");
                    self.homed = true;
                    self.not_homed_refused = false;
                    // Exit: full normal config reload for every node.
                    for i in 0..MAX_JOINTS {
                        let _ = self.bus.resend_node_config(self.node_of[i], 1);
                    }
                    if self.has_can_gripper {
                        let _ = self.bus.resend_node_config(self.gripper_node, 1);
                    }
                    self.mode = Mode::Idle;
                }
                SeqStatus::Failed => {
                    log::warn!("homing sequence FAILED");
                    self.homed = false;
                    self.mode = Mode::Idle;
                }
                SeqStatus::Running | SeqStatus::Inactive => {}
            }
            return;
        }

        match self.mode {
            Mode::Booting => dispatch::law_booting(&mut self.setpoints),
            Mode::Idle => {
                let hold = self.gravity_applied();
                dispatch::law_idle(hold, &self.g, &mut self.setpoints);
            }
            Mode::ActiveError => dispatch::law_active_error(&mut self.setpoints),
            Mode::SafetyStop => dispatch::law_safety_stop(&mut self.setpoints),
            Mode::Jog => {
                self.jog_blocked =
                    self.jog
                        .tick(&self.q, &mut self.scratch_q, &mut self.scratch_qd);
                self.q_target = self.scratch_q;
                self.qd_target = self.scratch_qd;
                dispatch::law_jog(
                    &self.scratch_q,
                    &self.scratch_qd,
                    self.gravity_applied(),
                    &self.g,
                    self.jog_pack,
                    &mut self.setpoints,
                );
            }
            Mode::Exec => {
                let outcome = self.exec.tick(
                    &self.q,
                    &mut self.scratch_q,
                    &mut self.scratch_qd,
                    &mut self.scratch_tau,
                );
                if outcome == ExecTick::Fault {
                    // Strict-policy settle timeout: hard error; playback
                    // froze in a hold, the reaction lands next tick.
                    self.errors.latch(ErrorCode::ExecSettleTimeout, None);
                }
                self.q_target = self.scratch_q;
                self.qd_target = self.scratch_qd;
                dispatch::law_exec(
                    &self.scratch_q,
                    &self.scratch_qd,
                    &self.scratch_tau,
                    self.gravity_applied(),
                    &self.g,
                    &mut self.setpoints,
                );
            }
            Mode::Stream => {
                let mut applied = false;
                if let Some(target) = self.stream_rx.take() {
                    self.q_target = target;
                    self.stream.set_target(&target);
                    self.stream_last_rx_tick = self.tick;
                    applied = true;
                }
                self.stream_window(applied);
                self.stream.step(&mut self.scratch_q, &mut self.scratch_qd);
                dispatch::law_stream(
                    &self.scratch_q,
                    &self.scratch_qd,
                    self.gravity_applied(),
                    &self.g,
                    &mut self.setpoints,
                );
            }
            // HAND_GUIDING/IMPEDANCE are refused at the gate; HOMING and
            // FLASHING returned above. Defensive zero-velocity.
            _ => dispatch::law_booting(&mut self.setpoints),
        }

        dispatch::commit(
            &self.setpoints,
            &self.conv,
            &self.torque_ma_factor,
            &mut self.cmds,
            &mut self.mirror,
        );
        self.send_joints();
        self.send_gripper(self.gripper_cmd);
        if self.gripper_cmd == GripperCommand::Calibrate {
            // cmd 62 goes out once; the empty poll then carries the
            // sweep (a repeated cmd 62 would restart it every tick).
            self.gripper_cmd = GripperCommand::FirmwarePoll;
        }
        let _ = self.bus.poll_step();
    }

    /// The tick's single per-joint send, with its failure log throttled
    /// (see [`FaultLog`]).
    fn send_joints(&mut self) {
        match self.bus.send_joint_commands(&self.cmds) {
            Ok(()) => {
                self.bus_faults.joint_tx.healthy();
                self.tx_fail_streak = 0;
            }
            Err(e) => {
                self.bus_tx_failures = self.bus_tx_failures.saturating_add(1);
                self.tx_fail_streak = self.tx_fail_streak.saturating_add(1);
                if let Some(n) = self.bus_faults.joint_tx.admit(self.tick) {
                    log::warn!("joint TX failed: {e} (+{n} suppressed)");
                }
            }
        }
    }

    /// The tick's gripper-slot send, with its failure log throttled.
    fn send_gripper(&mut self, cmd: GripperCommand) {
        match self.bus.send_gripper(&cmd) {
            Ok(()) => {
                self.bus_faults.gripper_tx.healthy();
                self.gripper_tx_fail_streak = 0;
            }
            Err(e) => {
                self.bus_tx_failures = self.bus_tx_failures.saturating_add(1);
                self.gripper_tx_fail_streak = self.gripper_tx_fail_streak.saturating_add(1);
                if let Some(n) = self.bus_faults.gripper_tx.admit(self.tick) {
                    log::warn!("gripper TX failed: {e} (+{n} suppressed)");
                }
            }
        }
    }

    fn stream_window(&mut self, applied: bool) {
        self.stream_window_pos += 1;
        if applied {
            self.stream_window_applied += 1;
        }
        if self.stream_window_pos >= self.stream_window_ticks {
            let sent_now = self.stream_sent.load(Ordering::Relaxed);
            let sent = sent_now.saturating_sub(self.stream_sent_base);
            let applied_n = u64::from(self.stream_window_applied);
            self.stream_success =
                self.stream_window_applied as f32 / self.stream_window_ticks as f32;
            self.stream_discard = if sent > 0 {
                (100.0 * (sent.saturating_sub(applied_n)) as f64 / sent as f64) as f32
            } else {
                0.0
            };
            self.stream_sent_base = sent_now;
            self.stream_window_pos = 0;
            self.stream_window_applied = 0;
        }
    }

    // ------------------------------------------------------------ snapshot

    fn publish(&mut self) {
        let s = &mut self.snap;
        s.tick = self.tick;
        s.mode = self.mode;
        s.state = self.state;
        s.enable_seq = self.enable_seq;
        s.homed = self.homed;
        s.q = self.q;
        s.qd = self.qd;
        s.tau = self.tau;
        s.q_filtered = self.q_filt;
        s.qd_filtered = self.qd_filt;
        s.tau_filtered = self.tau_filt;
        s.q_commanded = self.mirror.q;
        s.qd_commanded = self.mirror.qd;
        s.tau_commanded = self.mirror.tau;
        s.q_target = self.q_target;
        s.qd_target = self.qd_target;
        self.fk.tcp(&self.q, &mut s.tcp);
        self.fk.tcp(&self.mirror.q, &mut s.tcp_commanded);
        self.fk.tcp(&self.q_target, &mut s.tcp_target);
        s.gravity_torque_nm = self.g;
        for i in 0..MAX_JOINTS {
            s.nodes[i] = self.bus_state.nodes[usize::from(self.node_of[i])];
        }
        s.nodes[MAX_JOINTS] = self.bus_state.nodes[usize::from(self.gripper_node)];
        s.gripper = self.bus_state.gripper;
        s.homing = self.homing.status();
        s.errors = *self.errors.list();
        s.error_active = self.errors.any_hard();
        s.loop_stats = self.timing.stats();
        s.loop_stats.can_frame_age_max_ticks = self.bus_state.frame_age_max_ticks;
        s.loop_stats.can_frame_age_min_ticks = self.bus_state.frame_age_min_ticks;
        s.loop_stats.bus_tx_failures = self.bus_tx_failures;
        s.loop_stats.bus_rx_failures = self.bus_rx_failures;
        s.link = self.bus.link_health();
        s.exec = self.exec.status();
        s.jog = JogStatus {
            active: self.jog_active,
            joint: self.jog_joint,
            blocked_mask: self.jog_blocked,
        };
        s.stream = StreamStatus {
            substate: if self.mode == Mode::Stream {
                StreamSubstate::ControlActive
            } else {
                StreamSubstate::Unpaired
            },
            success_rate: self.stream_success,
            discard_pct: self.stream_discard,
        };
        self.writer.publish(&self.snap);
    }
}
