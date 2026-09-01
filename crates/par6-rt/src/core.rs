//! The tick-loop assembly: [`RtCore`] wires the per-tick modules
//! (dispatch laws, homing FSM, error latch, e-stop debounce, timing
//! bands, EXEC playback) over a [`DriverBus`] in the \[OURS\] phase order
//! — measure-then-command within one tick:
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
    PollAction,
};
use par6_config::{ConfigBundle, ControlMode, KtSource, LimitMode, MAX_IO_LINES};

use crate::dispatch::{self, CommandMirror, JointSetpoint};
use crate::errors::ErrorManager;
use crate::exec::{ExecPlayback, ExecTick};
use crate::gpio::{Debouncer, DigitalIo, EstopGpio, EstopMonitor};
use crate::gravity::GravityModel;
use crate::gripper_gate::GripperGate;
use crate::homing::{HomingSystem, SeqStatus};
use crate::hooks::{
    CommandSource, FlashMarker, ForwardKin, JogEngine, RtCommand, SettlePolicy, StreamTracker,
};
use crate::ring::SampleConsumer;
use crate::snapshot::{snapshot_channel, SnapshotReader, SnapshotWriter};
use crate::state::{
    ArmState, ErrorCode, HomingJointStatus, JogStatus, Mode, StateSnapshot, StreamStatus,
    StreamSubstate,
};
use crate::timing::{LoopHealth, LoopTiming};
use crate::MAX_JOINTS;

/// The `boot_configure` arguments, retained for a live bus swap.
struct BootConfig {
    robot: par6_config::RobotConfig,
    gripper: Option<par6_config::GripperConfig>,
    config_repeats: u8,
}

/// Boot one-shot: bus-scan selfcheck, then request IDLE (exit BOOTING).
const BOOT_SELFCHECK_TICK: u64 = 8;
/// Vendor boot workaround: full config re-sends at these ticks (may be
/// dropped after HIL validation).
const BOOT_CONFIG_RESEND_TICKS: [u64; 3] = [50, 150, 300];
/// Clear_Error frame repeats per faulted node during the clear sequence.
const CLEAR_ERROR_REPEATS: u8 = 3;
/// EXEC link watchdog: heartbeat silence while samples pending that
/// latches `EXEC_LINK_LOST` \[s\].
const EXEC_HEARTBEAT_TIMEOUT_S: f64 = 0.5;
/// First-order EMA coefficient for the `*_filtered` measured-state
/// mirrors (light smoothing for telemetry/external-torque estimation).
const MEAS_FILTER_ALPHA: f64 = 0.2;
/// Measured-speed band \[rad/s\] under which every joint counts as at
/// rest for the shutdown exit path. Above encoder quantization noise,
/// far below any commanded motion.
const SHUTDOWN_REST_RAD_S: f64 = 0.02;
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

/// Bitmask of the joints a per-joint speed array actually drives.
fn joint_mask(speeds: &[f64; MAX_JOINTS]) -> u8 {
    let mut mask = 0u8;
    for (i, v) in speeds.iter().enumerate() {
        if *v != 0.0 {
            mask |= 1 << i;
        }
    }
    mask
}

/// Per-tick coefficient of a first-order low-pass at *cutoff_hz*, or 0
/// when the filter is off.
///
/// `y += alpha · (x − y)` with `alpha = dt / (dt + 1/(2π·fc))`. A cutoff
/// at or above the Nyquist of the tick rate cannot filter anything, so it
/// is reported as off rather than as an alpha of ~1 that pretends to.
fn lowpass_alpha(cutoff_hz: f64, dt: f64) -> f64 {
    if cutoff_hz <= 0.0 || cutoff_hz >= 0.5 / dt {
        return 0.0;
    }
    let tau = 1.0 / (2.0 * std::f64::consts::PI * cutoff_hz);
    dt / (dt + tau)
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
    /// The box's general-purpose digital lines (`[io]` config).
    pub io: Box<dyn DigitalIo>,
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

    /// A heartbeat no watchdog consumes — for driving planning machinery
    /// outside a running RT core (offline preview).
    pub fn unmonitored() -> Self {
        Self {
            flag: Arc::new(AtomicBool::new(false)),
        }
    }
}

/// One streamed setpoint: where to go, and how hard to push getting there.
///
/// The fractions ride with the target rather than being separate commands
/// because a stream is latest-wins — a speed sent out of band could be
/// applied to a target that was already superseded.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StreamSetpoint {
    /// Joint-position target \[rad\].
    pub q: [f64; MAX_JOINTS],
    /// Velocity fraction of the STREAM limits, in `(0, 1]`.
    pub speed: f64,
    /// Acceleration fraction of the STREAM limits, in `(0, 1]`.
    pub accel: f64,
}

impl Default for StreamSetpoint {
    fn default() -> Self {
        Self {
            q: [0.0; MAX_JOINTS],
            speed: 1.0,
            accel: 1.0,
        }
    }
}

/// Command-plane handle for streaming setpoints: latest-wins slot (the
/// RT applies only the newest target per tick; superseded targets count
/// toward the published discard percentage).
pub struct StreamInput {
    writer: SnapshotWriter<StreamSetpoint>,
    sent: Arc<AtomicU64>,
}

impl StreamInput {
    /// Publish a new setpoint.
    pub fn send(&mut self, setpoint: &StreamSetpoint) {
        self.writer.publish(setpoint);
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
    /// Tick the ACTIVE bus was brought up at; the boot one-shots are
    /// measured from here so a live backend swap re-runs them.
    bus_booted_at: u64,
    /// Tick the 50/150/300 config-shot schedule counts from: the bus
    /// boot, or the last FLASHING exit (which must not reset
    /// `bus_booted_at` — the selfcheck already ran).
    config_repush_armed_at: u64,
    /// What `boot_configure` needs, kept so a bus swapped in later is
    /// configured exactly as the one opened at startup was.
    boot: BootConfig,
    torque_ma_factor: [f64; MAX_JOINTS],
    torque_slew: dispatch::TorqueSlew,
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
    io: Box<dyn DigitalIo>,
    /// Debounced input levels then driven output levels, in `[io]`
    /// order — the snapshot's `io_lines` layout, kept here so the
    /// publish is a copy.
    io_lines: [u8; MAX_IO_LINES],
    io_debounce: [Debouncer; MAX_IO_LINES],
    n_io_inputs: usize,
    n_io_outputs: usize,
    /// An output level changed and has not reached the pins yet. The
    /// vendor re-drives every output every tick; a level that did not
    /// move is the same pin state either way, and skipping it keeps an
    /// idle box off the ioctl path entirely.
    io_out_dirty: bool,
    flash: Box<dyn FlashMarker>,
    commands: Box<dyn CommandSource>,
    fk: Box<dyn ForwardKin>,

    // Subsystems.
    homing: HomingSystem,
    errors: ErrorManager,
    timing: LoopTiming,
    bus_faults: BusFaultLogs,

    // Bus-failure accounting: the backend PROPAGATES send/drain errors
    // and the tick loop counts every one into the published loop stats.
    // Consecutive-failure streaks drive the disconnect latch below.
    bus_tx_failures: u32,
    /// Scheduling setup outcome recorded by the run loop (SCHED_FIFO
    /// applied, CPU pinned) — published through `LoopStats`.
    rt_fifo: bool,
    rt_pinned: bool,
    /// Per-node self-heal one-shot (indexed by error index): armed while
    /// the node is fresh, fired once when it goes stale/lost, re-armed on
    /// recovery.
    self_heal_armed: [bool; crate::NUM_NODES],
    bus_rx_failures: u32,
    tx_fail_streak: u32,
    gripper_tx_fail_streak: u32,
    /// Consecutive failed ticks after which a TX streak latches the
    /// per-node disconnect errors — the freshness lost window, so an
    /// outbound-dead link disables on the same clock as a silent one.
    tx_fault_latch_ticks: u32,

    // State-machine variables: mode, state, homed, errors.
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
    /// External torque estimate \[Nm\]: filtered measured torque minus
    /// the model's gravity torque. What a hand pushing the arm looks
    /// like in the torque domain.
    tau_ext: [f64; MAX_JOINTS],
    filters_seeded: bool,
    /// `[limits] tau_ext_margin_nm`; 0 disables the envelope.
    tau_env_margin_nm: f64,
    /// Ticks a joint must stay beyond the margin before latching.
    tau_env_window_ticks: u32,
    /// Per-joint consecutive over-margin tick counters.
    tau_env_over: [u32; MAX_JOINTS],

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
    gripper_gate: GripperGate,
    calibrate_pending: bool,
    homing_gcmd: GripperCommand,

    // Jog live state.
    jog_active: bool,
    jog_joints: u8,
    jog_blocked: u16,

    // EXEC link watchdog.
    heartbeat: Arc<AtomicBool>,
    hb_silence: u32,
    hb_timeout_ticks: u32,

    // Streaming.
    stream_rx: SnapshotReader<StreamSetpoint>,
    /// Fractions currently applied to the streaming executor's limits.
    stream_scale: (f64, f64),
    /// Acceleration fraction currently applied to the jog engine.
    jog_accel_scale: f64,
    stream_sent: Arc<AtomicU64>,
    stream_last_rx_tick: u64,
    stream_timeout_ticks: u32,
    /// True until the session's first setpoint passes the start-pose
    /// gate; re-armed by every STREAM entry.
    stream_first_rx: bool,
    /// Worst-joint gap allowed on that first setpoint \[rad\].
    stream_start_tol_rad: f64,
    stream_window_ticks: u32,
    stream_window_pos: u32,
    stream_window_applied: u32,
    stream_sent_base: u64,
    stream_success: f32,
    stream_discard: f32,
    /// Command low-pass coefficient per tick; 0 = the filter is off.
    stream_lp_alpha: f64,
    /// Filter state, in joint space, carried across ticks.
    stream_filt: [f64; MAX_JOINTS],

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
        // Per-tick torque budget from each joint's declared slew ceiling.
        // A joint that declares none is unlimited, which is the behaviour
        // every joint had before the limit was enforced.
        let torque_slew = dispatch::TorqueSlew::new(std::array::from_fn(|i| {
            robot.joints[i]
                .limits
                .for_mode(LimitMode::Exec)
                .torque_rate_nm_s
                .map_or(f64::INFINITY, |rate| rate * dt)
        }));
        let (writer, snapshots) = snapshot_channel::<StateSnapshot>();
        let (stream_tx, stream_rx) = snapshot_channel::<StreamSetpoint>();
        let heartbeat = Arc::new(AtomicBool::new(false));
        let stream_sent = Arc::new(AtomicU64::new(0));
        let has_can_gripper = gripper.is_some();
        let homing_gcmd = if has_can_gripper {
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
            bus_booted_at: 0,
            config_repush_armed_at: 0,
            boot: BootConfig {
                robot: robot.clone(),
                gripper: gripper.cloned(),
                config_repeats: robot.bus.boot_config_repeats,
            },
            torque_ma_factor,
            torque_slew,
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
            io: hooks.io,
            io_lines: [0; MAX_IO_LINES],
            io_debounce: [Debouncer::new(); MAX_IO_LINES],
            n_io_inputs: robot.io.inputs.len(),
            n_io_outputs: robot.io.outputs.len(),
            io_out_dirty: !robot.io.outputs.is_empty(),
            flash: hooks.flash,
            commands: hooks.commands,
            fk: hooks.fk,
            homing: HomingSystem::new(bundle),
            errors: ErrorManager::new(dt),
            timing: LoopTiming::new(dt, robot.loop_timing()),
            bus_faults: BusFaultLogs::new(u64::from(robot.ticks(BUS_FAULT_LOG_PERIOD_S).max(1))),
            bus_tx_failures: 0,
            rt_fifo: false,
            rt_pinned: false,
            self_heal_armed: [true; crate::NUM_NODES],
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
            tau_ext: [0.0; MAX_JOINTS],
            filters_seeded: false,
            tau_env_margin_nm: robot.limits.tau_ext_margin_nm,
            tau_env_window_ticks: robot.ticks(robot.limits.tau_ext_window_s).max(1),
            tau_env_over: [0; MAX_JOINTS],
            g: [0.0; MAX_JOINTS],
            setpoints: [JointSetpoint::zero_velocity(); MAX_JOINTS],
            cmds: [JointCommand::idle(); MAX_JOINTS],
            mirror: CommandMirror::default(),
            q_target: [0.0; MAX_JOINTS],
            qd_target: [0.0; MAX_JOINTS],
            scratch_q: [0.0; MAX_JOINTS],
            scratch_qd: [0.0; MAX_JOINTS],
            scratch_tau: [0.0; MAX_JOINTS],
            gripper_gate: GripperGate::default(),
            calibrate_pending: false,
            homing_gcmd,
            jog_active: false,
            jog_joints: 0,
            jog_blocked: 0,
            heartbeat: heartbeat.clone(),
            hb_silence: 0,
            hb_timeout_ticks: robot.ticks(EXEC_HEARTBEAT_TIMEOUT_S).max(1),
            stream_rx,
            stream_scale: (1.0, 1.0),
            jog_accel_scale: 1.0,
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
            stream_first_rx: true,
            stream_start_tol_rad: robot.stream.start_pose_tol_rad,
            stream_window_ticks: robot.ticks(robot.stream.success_window_s).max(1),
            stream_window_pos: 0,
            stream_window_applied: 0,
            stream_sent_base: 0,
            stream_success: 0.0,
            stream_discard: 0.0,
            stream_lp_alpha: lowpass_alpha(robot.stream.lowpass_cutoff_hz, dt),
            stream_filt: [0.0; MAX_JOINTS],
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

    /// Measured joint positions \[rad\] — what a backend swap seeds the
    /// incoming simulator from, so the toggle does not move the model.
    pub fn measured_q(&self) -> [f64; MAX_JOINTS] {
        self.q
    }

    /// Swap the bus backend under a running core.
    ///
    /// The arm this core is talking to becomes a DIFFERENT arm, so
    /// everything derived from the old one is dropped rather than
    /// carried across: per-node freshness, the encoder sector each
    /// joint's conversion was boot-calibrated to, the measurement
    /// filters, and the home reference. The boot one-shots re-run from
    /// this tick, so the new backend gets the same selfcheck, kt fetch
    /// and config re-sends it would have got at startup.
    ///
    /// Motion is NOT stopped here — the command plane cancels the queue
    /// and the active stream before it calls this, because a swap that
    /// left a move running would resume it against an arm whose position
    /// is not yet known. What is left running is the mode: an ENABLED
    /// core comes out of this in BOOTING, which every motion mode is
    /// unreachable from, and reaches IDLE again when the selfcheck says
    /// the new bus answered.
    ///
    /// A backend that cannot be opened never reaches this — callers
    /// build it first, so the failure has a reply channel — and one that
    /// cannot be `boot_configure`d is refused here with the OLD bus
    /// still installed and still driving the arm.
    pub fn replace_bus(&mut self, mut bus: B) -> Result<(), CoreError> {
        bus.boot_configure(
            &self.boot.robot,
            self.boot.gripper.as_ref(),
            self.boot.config_repeats,
        )?;
        self.bus = bus;
        self.bus_state = BusState::new();
        self.sector_done = [false; MAX_JOINTS];
        self.filters_seeded = false;
        self.bus_booted_at = self.tick;
        self.config_repush_armed_at = self.tick;
        self.homed = false;
        self.not_homed_refused = false;
        self.mode = Mode::Booting;
        self.state = ArmState::Disabled;
        Ok(())
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

    /// Simulator/teleport path: re-aim every motion hold at the landed
    /// pose, after the [`set_joint_reference`](Self::set_joint_reference)
    /// calls have moved `q`. The mode laws hold position by re-sending
    /// their last target every tick — EXEC's starved-ring hold, STREAM's
    /// tracker state, JOG's integrated target — and the wire speed
    /// channel is only a feedforward, so a hold left aimed at the
    /// pre-teleport pose would actively drag the arm back to it.
    pub fn reseed_motion_targets(&mut self) {
        let q = self.q;
        self.exec.reseed_hold(&q);
        self.stream.activate(&q);
        self.jog.activate(&q);
        self.q_target = q;
    }

    /// Replace the EXEC completion policy (takes effect at the next
    /// command boundary) — the `set_completion_policy` follow-through
    /// from the command plane.
    pub fn set_settle_policy(&mut self, policy: Box<dyn SettlePolicy>) {
        self.exec.set_policy(policy);
    }

    /// Reset the loop timing statistics (the `reset_loop_stats`
    /// follow-through); the warmup gate re-arms. The scheduling flags are
    /// state, not statistics, and survive.
    pub fn reset_loop_stats(&mut self) {
        self.timing.reset();
    }

    /// Record whether the run loop's scheduling setup took effect.
    pub fn record_rt_sched(&mut self, fifo: bool, pinned: bool) {
        self.rt_fifo = fifo;
        self.rt_pinned = pinned;
    }

    /// Shutdown phase 1: force the standing halt. Any working mode drops
    /// to IDLE, whose law commands active zero velocity (or the gravity
    /// float), so subsequent ticks bring the arm to a commanded stop
    /// before the process exits. BOOTING / ACTIVE_ERROR / SAFETY_STOP
    /// already run a stationary law and are left in place; FLASHING is a
    /// bus-silent maintenance window and must stay silent.
    pub fn shutdown_halt(&mut self) {
        if matches!(
            self.mode,
            Mode::Idle | Mode::Booting | Mode::ActiveError | Mode::SafetyStop | Mode::Flashing
        ) {
            return;
        }
        let _ = self.request_mode(Mode::Idle);
    }

    /// Whether every joint's measured speed is inside the shutdown rest
    /// band — the condition the exit path waits on before idling the
    /// drives.
    pub fn at_rest(&self) -> bool {
        self.qd_filt.iter().all(|v| v.abs() < SHUTDOWN_REST_RAD_S)
    }

    /// Shutdown phase 2: terminal limp. SAFETY_STOP's law is torque-only
    /// 0 N·m, so the tick after this call puts a frame on the bus that
    /// idles the drives on purpose — instead of leaving them to act on
    /// the last motion frame until the CAN watchdog expires and drops
    /// them out mid-hold. No-op in FLASHING (the bus is silent by
    /// contract there, and the arm is parked and asserted).
    pub fn shutdown_limp(&mut self) {
        if self.mode == Mode::Flashing {
            return;
        }
        let _ = self.request_mode(Mode::SafetyStop);
    }

    /// One tick of the core. `period_s` is the measured loop period (the
    /// virtual-tick tests feed the nominal `dt`); `overrun` marks a
    /// missed deadline as measured by the caller.
    pub fn tick(&mut self, period_s: f64, overrun: bool) {
        self.tick += 1;
        self.bus.begin_tick(self.tick);

        // Phase 2: GPIO read + debounce — the e-stop (reaction in phase
        // 7) and the box's own inputs, which mean nothing to the runtime
        // and are published verbatim.
        self.hw_estop = self.estop.pressed();
        self.read_io_inputs();

        // Phase 3: loop-period statistics and degradation bands.
        let health = self.timing.record(period_s, overrun);

        // Phase 4: boot one-shots.
        self.boot_oneshots();

        // Phase 5: at most one external command.
        if let Some(cmd) = self.commands.poll() {
            self.apply_command(cmd);
        }

        // Phase 5b: a level this tick's command changed reaches the pins
        // in the same tick, so a client that writes and then reads the
        // next STATUS sees what it asked for.
        self.drive_io_outputs();

        // Phase 6: RX drain → state pipeline (measure-then-command).
        self.drain_and_derive();

        // Gravity: computed every tick, published always.
        self.gravity.gravity(&self.q, &mut self.g);

        // External torque: what the measured (filtered) torque carries
        // beyond the model's gravity — a contact, a payload the model
        // does not know, a hand on the arm.
        for i in 0..MAX_JOINTS {
            self.tau_ext[i] = self.tau_filt[i] - self.g[i];
        }

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
        // Ticks since this BUS came up, not since the process did: a
        // backend swapped in at tick 90 000 needs the same selfcheck and
        // the same config re-sends a backend opened at boot got.
        let since_boot = self.tick - self.bus_booted_at;
        if since_boot == BOOT_SELFCHECK_TICK {
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
        // The scheduled shots ride their own arm point, not the bus
        // boot: a FLASHING exit re-arms them without re-running the
        // selfcheck above.
        let since_armed = self.tick - self.config_repush_armed_at;
        if BOOT_CONFIG_RESEND_TICKS.contains(&since_armed) && !self.bus.is_silent() {
            self.config_repush();
        }
    }

    /// One full stored-config shot: `bus.boot_config_repeats` passes to
    /// every arm node and the CAN gripper motor (the vendor pushes 4 per
    /// shot; the config key governs the redundancy).
    fn config_repush(&mut self) {
        for i in 0..MAX_JOINTS {
            let _ = self
                .bus
                .resend_node_config(self.node_of[i], self.boot.config_repeats);
        }
        if self.has_can_gripper {
            let _ = self
                .bus
                .resend_node_config(self.gripper_node, self.boot.config_repeats);
        }
    }

    /// Rebuild the torque scale around the drivers' own torque constants
    /// (`kt_source = "auto"`, the boot cmd-33 fetch): the fetched value
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
            RtCommand::Jog { speeds, accel } => {
                if self.mode == Mode::Jog {
                    if self.jog_accel_scale != accel {
                        self.jog.set_accel_scale(accel);
                        self.jog_accel_scale = accel;
                    }
                    self.jog.command(&speeds);
                    self.jog_active = true;
                    self.jog_joints = joint_mask(&speeds);
                }
            }
            RtCommand::JogRelease => {
                self.jog.release();
                self.jog_active = false;
                self.jog_joints = 0;
            }
            RtCommand::ExecSetPaused(paused) => self.exec.set_paused(paused),
            RtCommand::ExecFlush => {
                let n = self.exec.flush();
                log::info!("EXEC flush discarded {n} samples");
            }
            RtCommand::Gripper(fw) => {
                if self.has_can_gripper {
                    self.gripper_gate.set(fw);
                }
            }
            RtCommand::GripperCalibrate => {
                if self.has_can_gripper {
                    self.gripper_gate.reset_to_poll();
                    self.calibrate_pending = true;
                }
            }
            RtCommand::GripperStop => {
                if self.has_can_gripper {
                    // The freshest jaw byte is read RT-side: routing a
                    // snapshot byte back through the command plane would
                    // race the reply stream and stop at a stale position.
                    let byte = self.bus_state.gripper.reply.map(|r| r.position);
                    match byte {
                        Some(b @ 1..=254) if self.gripper_gate.has_standing() => {
                            self.gripper_gate.stop_at(b);
                        }
                        // An uncalibrated gripper reports 0, which the
                        // firmware maps to fully open — a naive stop
                        // would fling the jaws open. No standing command
                        // means no speed/current budget to hold with.
                        // Both degrade to a release.
                        _ => self.gripper_gate.idle(),
                    }
                }
            }
            RtCommand::GripperIdle => {
                if self.has_can_gripper {
                    self.gripper_gate.idle();
                }
            }
            RtCommand::SetGravityComp(on) => self.gravity_comp = on,
            RtCommand::SetPayload { mass, com, inertia } => {
                self.gravity.set_payload(mass, com, inertia);
            }
            RtCommand::WriteIo { port, value } => self.set_io_output(port, value),
        }
    }

    /// Read and debounce every declared input.
    ///
    /// The vendor debounces its general inputs with the same
    /// five-consecutive-reads filter it uses on the e-stop, so a
    /// contact bounce cannot be seen by a client polling STATUS at
    /// 50 Hz; the first-read seeding matters here too, because an
    /// unseeded filter would publish LOW for the first four ticks of
    /// an input that was high the whole time.
    fn read_io_inputs(&mut self) {
        if self.n_io_inputs == 0 {
            return;
        }
        let mut raw = [0u8; MAX_IO_LINES];
        let raw = &mut raw[..self.n_io_inputs];
        self.io.read_inputs(raw);
        for (i, level) in raw.iter().enumerate() {
            self.io_lines[i] = u8::from(self.io_debounce[i].update(*level != 0));
        }
    }

    /// Drive the outputs when a level has moved since the last write.
    fn drive_io_outputs(&mut self) {
        if !self.io_out_dirty {
            return;
        }
        let start = self.n_io_inputs;
        self.io
            .write_outputs(&self.io_lines[start..start + self.n_io_outputs]);
        self.io_out_dirty = false;
    }

    /// Set one output by `[io]` port index. Out-of-range ports are
    /// refused by the command plane against the same declared count, so
    /// reaching one here is a wiring bug rather than a client error.
    fn set_io_output(&mut self, port: u8, value: u8) {
        let port = usize::from(port);
        if port >= self.n_io_outputs {
            log::error!(
                "write_io port {port} past the {} declared outputs",
                self.n_io_outputs
            );
            return;
        }
        let slot = self.n_io_inputs + port;
        let level = u8::from(value != 0);
        if self.io_lines[slot] != level {
            self.io_lines[slot] = level;
            self.io_out_dirty = true;
        }
    }

    /// Mode transition request with the gates applied in order:
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
                // JOG is deliberately absent: an arm can need jogging clear
                // of an obstruction before it can be homed at all, and a
                // jog asks only for a direction and a speed — nothing about
                // it is expressed in absolute coordinates. STREAM and EXEC
                // do target absolute positions, so they still need a
                // reference. The soft-limit brake bounds an unhomed jog the
                // same as any other.
                let needs_home = matches!(target, Mode::Stream | Mode::Exec);
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
                self.stream_first_rx = true;
                self.stream_window_pos = 0;
                self.stream_window_applied = 0;
                self.stream_sent_base = self.stream_sent.load(Ordering::Relaxed);
                self.stream_success = 0.0;
                self.stream_discard = 0.0;
                // Seeding the filter at the measured pose is what keeps a
                // filtered session from starting with a ramp out of
                // whatever the last session left behind.
                self.stream_filt = self.q;
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
                if self.has_can_gripper {
                    self.gripper_gate.force_idle(self.homing.last_fw_cmd());
                }
            }
            Mode::Flashing => {
                self.bus.set_silent(false);
                // The silent window must not read as a mass disconnect.
                self.bus.rebase_freshness();
                // ... which also masks every real power-cycle or reflash
                // that happened during it from the reconnect path, so
                // the exit pushes the stored config itself — one pass
                // now, then the re-armed 50/150/300 shots.
                self.config_repush();
                self.config_repush_armed_at = self.tick;
                if self.flash.flashed() {
                    log::info!("firmware was flashed: homing invalidated");
                    self.homed = false;
                }
                if self.has_can_gripper {
                    // A reflashed or power-cycled gripper driver may hold
                    // whatever its NVM state says; the announcement runs
                    // on the first non-silent ticks after exit.
                    self.gripper_gate.force_idle(None);
                }
            }
            Mode::Stream => {
                // Only the Stream arm drains the latest-wins slot, so a
                // setpoint published in the tick this session ended would
                // stay FRESH indefinitely and retarget the NEXT session's
                // first tick toward an abandoned pose — one the admission
                // gate never re-checked against the current world.
                let _ = self.stream_rx.take();
            }
            _ => {}
        }
        // The park assertion is one-shot: consumed by FLASHING entry,
        // dropped by any other transition.
        self.park_asserted = false;
        let _ = target;
    }

    /// The user clear sequence: Clear_Error ×3 to each
    /// faulted node (+ gripper), stale per-type flags zeroed, lost
    /// latches reset, then the settle countdown that outlasts the poll
    /// cycle before the latch wipes.
    fn begin_clear(&mut self) {
        // Once per NODE, not once per error entry: a node with several
        // latched fault types would otherwise get a triple apiece, and a
        // full error list can run to ~99 frames in a single tick — past
        // the classic-CAN budget, with the overflow dropped silently.
        let mut cleared: u32 = 0;
        for entry in self.errors.list().as_slice() {
            let Some(j) = entry.joint else { continue };
            let node = if usize::from(j) < MAX_JOINTS {
                self.node_of[usize::from(j)]
            } else {
                self.gripper_node
            };
            let bit = 1u32 << u32::from(node);
            if cleared & bit != 0 {
                continue;
            }
            cleared |= bit;
            let _ = self.bus.send_clear_error(node, CLEAR_ERROR_REPEATS);
            self.bus.clear_lost_latch(node);
        }
        if self.has_can_gripper && cleared & (1u32 << u32::from(self.gripper_node)) == 0 {
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

        // Motor-bus controller state: bus-off is a hard latch (nothing
        // reaches the drives until the kernel restarts the interface),
        // error-passive the self-clearing warning on the way there.
        let link = self.bus.link_health();
        if link.state == par6_bus::LinkState::BusOff {
            self.errors.latch(ErrorCode::BusOff, None);
        }
        self.errors.condition(
            ErrorCode::LinkErrorPassive,
            None,
            link.state == par6_bus::LinkState::ErrorPassive,
        );

        // External-torque envelope: a joint whose external-torque
        // estimate stays beyond the margin for the window latches hard —
        // unexpected contact or an unmodeled payload. Enforced only
        // where the estimate is meaningful: homed (the gravity model
        // needs referenced angles), enabled, and in a mode whose drives
        // actively bear the load — a limp or protective-hold arm reads
        // `-g(q)` as "external" torque and would re-latch forever. The
        // EXEC feed-forward is subtracted so planned acceleration does
        // not count against the margin (one tick stale, matching the
        // measurement's own lag). Hand-guiding and impedance modes are
        // exempt: external torque is their input, not a fault.
        if self.tau_env_margin_nm > 0.0 {
            let enforce = self.homed
                && self.state == ArmState::Enabled
                && matches!(
                    self.mode,
                    Mode::Idle | Mode::Jog | Mode::Stream | Mode::Exec
                );
            for i in 0..MAX_JOINTS {
                let ff = if self.mode == Mode::Exec {
                    self.scratch_tau[i]
                } else {
                    0.0
                };
                if enforce && (self.tau_ext[i] - ff).abs() > self.tau_env_margin_nm {
                    self.tau_env_over[i] = self.tau_env_over[i].saturating_add(1);
                    if self.tau_env_over[i] >= self.tau_env_window_ticks {
                        self.errors.latch(ErrorCode::TorqueEnvelope, Some(i as u8));
                    }
                } else {
                    self.tau_env_over[i] = 0;
                }
            }
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
        // Self-heal: a node gone quiet may just have dropped its config
        // (brown-out, watchdog reset) — a config resend provokes a reply
        // where plain polls stay unanswered. One shot per silence episode,
        // through the poll slot so the tick's TX budget is untouched;
        // re-armed only by an actual recovery, so a node that is truly
        // gone costs one frame, not one per tick.
        let armed = &mut self.self_heal_armed[usize::from(err_idx)];
        match f {
            Freshness::Fresh => *armed = true,
            Freshness::Stale | Freshness::Lost => {
                if *armed {
                    *armed = false;
                    log::warn!("node {node}: {f:?}; queueing a config resend to provoke recovery");
                    self.bus
                        .queue_poll_override(PollAction::ResendConfig { node }, 1);
                }
            }
            Freshness::Unknown => {}
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
                    // Complete only says the sequence ran to its end, not
                    // that it referenced anything: a config whose home
                    // groups omit a joint still completes. Claiming
                    // `homed` then unlocks JOG/STREAM/EXEC on axes whose
                    // absolute position is still the boot sector guess.
                    let statuses = self.homing.statuses();
                    let unreferenced: Vec<usize> = (0..MAX_JOINTS)
                        .filter(|&i| statuses[i] != HomingJointStatus::Done)
                        .collect();
                    if unreferenced.is_empty() {
                        log::info!("homing sequence complete");
                        self.homed = true;
                        self.not_homed_refused = false;
                    } else {
                        log::warn!(
                            "homing sequence completed without referencing {unreferenced:?}; \
                             the arm stays unhomed"
                        );
                        self.homed = false;
                    }
                    // Exit: full normal config reload for every node.
                    for i in 0..MAX_JOINTS {
                        let _ = self.bus.resend_node_config(self.node_of[i], 1);
                    }
                    if self.has_can_gripper {
                        let _ = self.bus.resend_node_config(self.gripper_node, 1);
                        // Homing streamed its own DLC-5 frames (its park
                        // move included), so the firmware is holding a
                        // grip the normal path never commanded — and the
                        // watchdog polls would keep it held forever.
                        self.gripper_gate.force_idle(self.homing.last_fw_cmd());
                    }
                    self.mode = Mode::Idle;
                }
                SeqStatus::Failed => {
                    log::warn!("homing sequence FAILED");
                    self.homed = false;
                    if self.has_can_gripper {
                        self.gripper_gate.force_idle(self.homing.last_fw_cmd());
                    }
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
                if let Some(sp) = self.stream_rx.take() {
                    if self.stream_first_rx && !self.stream_admit(&sp) {
                        // Dropped un-applied; the latch's reaction lands
                        // on the next error pass.
                    } else {
                        self.stream_first_rx = false;
                        // Scale first: the limits have to be in force for
                        // the tick that consumes this target, not the one
                        // after. Only on a change — `set_limits` rewrites
                        // the OTG's whole input block and most streams
                        // never move the sliders at all.
                        if self.stream_scale != (sp.speed, sp.accel) {
                            self.stream.set_scale(sp.speed, sp.accel);
                            self.stream_scale = (sp.speed, sp.accel);
                        }
                        // `q_target` carries the raw request; the filter
                        // sits between it and the executor, so the pair
                        // makes the smoothing visible.
                        self.q_target = sp.q;
                        let target = self.filtered_target(&sp.q);
                        self.stream.set_target(&target);
                        self.stream_last_rx_tick = self.tick;
                        applied = true;
                    }
                }
                self.stream_window(applied);
                self.stream.step(&mut self.scratch_q, &mut self.scratch_qd);
                if self.stream.faulted() {
                    // The limiter is holding in place instead of
                    // tracking; a stream that silently stops following
                    // its setpoints must become a visible hard error.
                    self.errors.latch(ErrorCode::StreamFault, None);
                }
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

        // The protective laws must reach the drive this tick, so they are
        // never rate-limited; they snap the slew state instead, which is
        // what makes the mode they hand back to ramp from zero.
        let rate_limit = !matches!(self.mode, Mode::ActiveError | Mode::SafetyStop);
        dispatch::commit(
            &self.setpoints,
            &self.conv,
            &self.torque_ma_factor,
            &mut self.torque_slew,
            rate_limit,
            &mut self.cmds,
            &mut self.mirror,
        );
        self.send_joints();
        let gcmd = if !self.has_can_gripper {
            GripperCommand::NoGripper
        } else if std::mem::take(&mut self.calibrate_pending) {
            // cmd 62 goes out once; the gate then carries the sweep on
            // DLC-0 polls (a repeated cmd 62 would restart it every
            // tick, and any DLC-5 frame would disturb it).
            GripperCommand::Calibrate
        } else {
            let calibrated = self.bus_state.gripper.reply.is_some_and(|r| r.calibrated);
            self.gripper_gate.tick(calibrated)
        };
        self.send_gripper(gcmd);
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

    /// The streamed target as the executor should see it: the raw request
    /// when `stream.lowpass_cutoff_hz` is off, otherwise one step of the
    /// first-order filter.
    fn filtered_target(&mut self, q: &[f64; MAX_JOINTS]) -> [f64; MAX_JOINTS] {
        if self.stream_lp_alpha == 0.0 {
            return *q;
        }
        for (y, x) in self.stream_filt.iter_mut().zip(q.iter()) {
            *y += self.stream_lp_alpha * (x - *y);
        }
        self.stream_filt
    }

    /// The start-pose gate on a session's first setpoint: worst-joint
    /// gap to the measured pose within `stream.start_pose_tol_rad`, or
    /// the setpoint is refused and the hard `StreamStartPose` key
    /// latches on the worst joint — the executor would otherwise ramp
    /// the arm to wherever the client happened to start publishing.
    fn stream_admit(&mut self, sp: &StreamSetpoint) -> bool {
        let mut worst = 0usize;
        let mut gap = 0.0f64;
        for i in 0..MAX_JOINTS {
            let d = (sp.q[i] - self.q[i]).abs();
            if d > gap {
                gap = d;
                worst = i;
            }
        }
        if gap <= self.stream_start_tol_rad {
            return true;
        }
        // Refusal path: runs at most a tick or two before ACTIVE_ERROR.
        log::warn!(
            "stream refused: first setpoint {gap:.3} rad from the measured pose on J{worst} \
             (tolerance {:.3})",
            self.stream_start_tol_rad
        );
        self.errors
            .latch(ErrorCode::StreamStartPose, Some(worst as u8));
        false
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
        s.tau_ext = self.tau_ext;
        s.q_commanded = self.mirror.q;
        s.qd_commanded = self.mirror.qd;
        s.tau_commanded = self.mirror.tau;
        s.gravity_comp = self.gravity_comp;
        s.q_target = self.q_target;
        s.qd_target = self.qd_target;
        self.fk.tcp(&self.q, &mut s.tcp);
        self.fk.tcp(&self.mirror.q, &mut s.tcp_commanded);
        self.fk.tcp(&self.q_target, &mut s.tcp_target);
        s.gravity_torque_nm = self.g;
        for i in 0..MAX_JOINTS {
            s.nodes[i] = self.bus_state.nodes[usize::from(self.node_of[i])];
            s.node_freshness[i] = self.bus.freshness(self.node_of[i]);
        }
        s.nodes[MAX_JOINTS] = self.bus_state.nodes[usize::from(self.gripper_node)];
        s.node_freshness[MAX_JOINTS] = self.bus.freshness(self.gripper_node);
        s.gripper = self.bus_state.gripper;
        s.homing = self.homing.status();
        s.errors = *self.errors.list();
        s.error_active = self.errors.any_hard();
        s.loop_stats = self.timing.stats();
        s.loop_stats.rt_fifo = self.rt_fifo;
        s.loop_stats.rt_pinned = self.rt_pinned;
        s.loop_stats.can_frame_age_max_ticks = self.bus_state.frame_age_max_ticks;
        s.loop_stats.can_frame_age_min_ticks = self.bus_state.frame_age_min_ticks;
        s.loop_stats.bus_tx_failures = self.bus_tx_failures;
        s.loop_stats.bus_rx_failures = self.bus_rx_failures;
        s.link = self.bus.link_health();
        s.exec = self.exec.status();
        s.jog = JogStatus {
            active: self.jog_active,
            joints: self.jog_joints,
            blocked_mask: self.jog_blocked,
        };
        s.io_lines = self.io_lines;
        s.io_inputs = self.n_io_inputs as u8;
        s.io_outputs = self.n_io_outputs as u8;
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
