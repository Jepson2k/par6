//! The `Err` arm of the [`DriverBus`] contract, driven through real
//! `RtCore` ticks. The bus contract: **propagate send errors** — the
//! vendor swallowed them and shipped production bugs because of it.
//!
//! Neither `LoopbackBus` nor `SimBus` can fail, so [`FailingBus`]
//! delegates everything to the loopback reference backend with a
//! per-method failure switch on each per-tick call. What the runtime
//! owes on failure: every refused send/drain is counted into the
//! published loop stats, a TX-failure streak spanning the freshness
//! lost window latches the per-node disconnect errors (DISABLED +
//! ACTIVE_ERROR within a bounded tick count), transient failures latch
//! nothing, and the whole failure arm allocates nothing.

use std::alloc::{GlobalAlloc, Layout, System};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc;

use par6_bus::spectral::JointConversion;
use par6_bus::{
    BusError, BusState, DriverBus, Freshness, GripperCommand, GripperReply, JointCommand,
    LinkHealth, LoopbackBus, NodeId, PollAction, Reply,
};
use par6_config::{ConfigBundle, GripperConfig, RobotConfig};
use par6_rt::hooks::{ClampStream, RampJog};
use par6_rt::{
    sample_ring, ArmState, CompletionPolicy, ErrorCode, Mode, NoFk, RtCommand, RtCore, RtHandles,
    RtHooks, SharedDigitalIo, SharedFlashMarker, SharedLineGpio, SpecSettle, StateSnapshot,
    ZeroGravity, MAX_JOINTS,
};

// ------------------------------------------------------- counting allocator

static ALLOCS: AtomicU64 = AtomicU64::new(0);

struct CountingAlloc;

// SAFETY: delegates every operation to the system allocator unchanged;
// the counter is a relaxed atomic side effect.
unsafe impl GlobalAlloc for CountingAlloc {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::Relaxed);
        unsafe { System.alloc(layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::Relaxed);
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::Relaxed);
        unsafe { System.realloc(ptr, layout, new_size) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static A: CountingAlloc = CountingAlloc;

// ------------------------------------------------------------- failing bus

/// A `DriverBus` delegating everything to the loopback reference
/// backend, with each per-tick call switchable to the transport error
/// the SocketCAN backend really returns for that call.
struct FailingBus {
    inner: LoopbackBus,
    fail_joint_tx: bool,
    fail_gripper_tx: bool,
    fail_rx: bool,
    fail_poll: bool,
}

impl FailingBus {
    fn new() -> Self {
        Self {
            inner: LoopbackBus::new(),
            fail_joint_tx: false,
            fail_gripper_tx: false,
            fail_rx: false,
            fail_poll: false,
        }
    }
}

impl DriverBus for FailingBus {
    fn begin_tick(&mut self, tick: u64) {
        self.inner.begin_tick(tick);
    }

    fn drain_rx(&mut self, state: &mut BusState) -> Result<usize, BusError> {
        if self.fail_rx {
            return Err(BusError::LinkDown);
        }
        self.inner.drain_rx(state)
    }

    fn send_joint_commands(&mut self, commands: &[JointCommand]) -> Result<(), BusError> {
        if self.fail_joint_tx {
            return Err(BusError::TxQueueFull);
        }
        self.inner.send_joint_commands(commands)
    }

    fn send_gripper(&mut self, command: &GripperCommand) -> Result<(), BusError> {
        if self.fail_gripper_tx {
            return Err(BusError::TxQueueFull);
        }
        self.inner.send_gripper(command)
    }

    fn poll_step(&mut self) -> Result<(), BusError> {
        if self.fail_poll {
            return Err(BusError::LinkDown);
        }
        self.inner.poll_step()
    }

    fn queue_poll_override(&mut self, action: PollAction, repeats: u16) {
        self.inner.queue_poll_override(action, repeats);
    }

    fn boot_configure(
        &mut self,
        robot: &RobotConfig,
        gripper: Option<&GripperConfig>,
        repeats: u8,
    ) -> Result<(), BusError> {
        self.inner.boot_configure(robot, gripper, repeats)
    }

    fn resend_node_config(&mut self, node: NodeId, repeats: u8) -> Result<(), BusError> {
        self.inner.resend_node_config(node, repeats)
    }

    fn retune_node(
        &mut self,
        node: NodeId,
        tune: &par6_bus::DriveTune,
        repeats: u8,
    ) -> Result<(), BusError> {
        self.inner.retune_node(node, tune, repeats)
    }

    fn set_can_id(&mut self, node: NodeId, new_id: NodeId) -> Result<(), BusError> {
        self.inner.set_can_id(node, new_id)
    }

    fn save_config(&mut self, node: NodeId) -> Result<(), BusError> {
        self.inner.save_config(node)
    }

    fn send_limits(
        &mut self,
        node: NodeId,
        velocity_limit_ticks_s: f32,
        current_limit_ma: f32,
        repeats: u8,
    ) -> Result<(), BusError> {
        self.inner
            .send_limits(node, velocity_limit_ticks_s, current_limit_ma, repeats)
    }

    fn send_clear_error(&mut self, node: NodeId, repeats: u8) -> Result<(), BusError> {
        self.inner.send_clear_error(node, repeats)
    }

    fn set_silent(&mut self, silent: bool) {
        self.inner.set_silent(silent);
    }

    fn is_silent(&self) -> bool {
        self.inner.is_silent()
    }

    fn freshness(&self, node: NodeId) -> Freshness {
        self.inner.freshness(node)
    }

    fn clear_lost_latch(&mut self, node: NodeId) {
        self.inner.clear_lost_latch(node);
    }

    fn rebase_freshness(&mut self) {
        self.inner.rebase_freshness();
    }

    fn connected_nodes(&self) -> u16 {
        self.inner.connected_nodes()
    }

    fn link_health(&self) -> LinkHealth {
        self.inner.link_health()
    }
}

// ------------------------------------------------------------------- rig

/// Gripper slot in per-joint error keys.
const GRIPPER_ERR_IDX: u8 = MAX_JOINTS as u8;

struct Rig {
    core: RtCore<FailingBus>,
    handles: RtHandles,
    cmds: mpsc::Sender<RtCommand>,
    conv: [JointConversion; MAX_JOINTS],
    node_of: [NodeId; MAX_JOINTS],
    pose: [f64; MAX_JOINTS],
    dt: f64,
    /// The freshness lost window — the TX-streak latch bound — in ticks.
    lost_ticks: u32,
}

impl Rig {
    fn new() -> Self {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
        let bundle = ConfigBundle::load(&path).expect("PAR6 config bundle");
        let robot = &bundle.robot;
        let dt = robot.robot.tick_dt_s;
        let (tx, rx) = mpsc::channel();
        let (gpio, _line) = SharedLineGpio::new(true);
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
        let mut conv: [JointConversion; MAX_JOINTS] =
            std::array::from_fn(|i| JointConversion::from_config(&robot.joints[i]));
        for (c, j) in conv.iter_mut().zip(&robot.joints) {
            c.determine_sector(j.sector_master_position_ticks);
        }
        let pose = std::array::from_fn(|i| robot.joints[i].sector_home_offset_rad);
        let node_of = std::array::from_fn(|i| robot.joints[i].node_id);
        let lost_ticks = robot.ticks(robot.bus.lost_s);
        let (core, handles) = RtCore::new(&bundle, FailingBus::new(), hooks).expect("core");
        Self {
            core,
            handles,
            cmds: tx,
            conv,
            node_of,
            pose,
            dt,
            lost_ticks,
        }
    }

    /// One virtual tick with a healthy-bus RX injection, so freshness
    /// stays green and any latch a test observes came from the failure
    /// switch under test, not from RX silence.
    fn tick(&mut self) {
        for i in 0..MAX_JOINTS {
            let node = self.node_of[i];
            let ticks = self.conv[i].motor_ticks(self.pose[i]);
            self.core.bus_mut().inner.inject(
                false,
                Reply::Motion {
                    node,
                    position_ticks: ticks,
                    speed_ticks_s: 0,
                    current_ma: 0,
                },
            );
        }
        self.core.bus_mut().inner.inject(
            false,
            Reply::Gripper {
                reply: GripperReply {
                    calibrated: true,
                    ..GripperReply::default()
                },
            },
        );
        self.core.tick(self.dt, false);
    }

    fn tick_n(&mut self, n: u32) {
        for _ in 0..n {
            self.tick();
        }
    }

    fn snap(&mut self) -> StateSnapshot {
        self.handles.snapshots.latest()
    }

    fn cmd(&mut self, cmd: RtCommand) {
        self.cmds.send(cmd).expect("command channel");
        self.tick();
    }

    /// Boot to IDLE, declare homed (simulator path), enable.
    fn ready(&mut self) {
        self.tick_n(10);
        assert_eq!(self.snap().mode, Mode::Idle, "boot must reach IDLE");
        self.core.set_homed(true);
        self.cmd(RtCommand::Enable);
        assert_eq!(self.snap().state, ArmState::Enabled);
    }
}

fn can_lost(s: &StateSnapshot, joint: u8) -> bool {
    s.errors
        .as_slice()
        .iter()
        .any(|e| e.code == ErrorCode::CanLost && e.joint == Some(joint))
}

// ------------------------------------------------------------------ tests

/// The counting allocator is process-global, so the four tests here must
/// not run concurrently: a sibling test allocating inside the measured
/// window would fail `the_failure_arm_allocates_nothing` spuriously.
/// (The repo's other allocator tests isolate by living alone in their
/// binary; this file shares its `FailingBus` rig instead and serializes.)
static SERIAL: std::sync::Mutex<()> = std::sync::Mutex::new(());

fn serial() -> std::sync::MutexGuard<'static, ()> {
    SERIAL
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
}

/// An arm whose commands stop reaching the wire must disable within the
/// freshness lost window, even while RX freshness reads green — and the
/// failures must be visible in STATUS, not only in a log line.
#[test]
fn sustained_joint_tx_failure_latches_disconnect_and_disables() {
    let _serial = serial();
    let mut rig = Rig::new();
    rig.ready();
    assert_eq!(rig.snap().loop_stats.bus_tx_failures, 0);

    rig.core.bus_mut().fail_joint_tx = true;
    let lost = rig.lost_ticks;

    // Inside the tolerance window a failing link is degraded, not dead.
    rig.tick_n(lost / 2);
    let s = rig.snap();
    assert!(
        !s.error_active,
        "a TX-failure streak shorter than the lost window must not latch"
    );
    assert_eq!(s.state, ArmState::Enabled);

    // A streak spanning the lost window is a disconnect: hard latch,
    // DISABLED, ACTIVE_ERROR — bounded by lost + latch-check slack.
    rig.tick_n(lost / 2 + 4);
    let s = rig.snap();
    assert!(s.error_active, "sustained TX failure must hard-latch");
    assert_eq!(s.state, ArmState::Disabled);
    assert_eq!(s.mode, Mode::ActiveError);
    for j in 0..MAX_JOINTS as u8 {
        assert!(can_lost(&s, j), "J{j}: CAN-lost must be latched");
    }
    assert!(
        can_lost(&s, GRIPPER_ERR_IDX),
        "a TX-dead link cannot reach the gripper either"
    );

    // Discrimination: RX stayed green the whole time, so the latch came
    // from the TX path, not from freshness.
    assert_eq!(s.nodes[0].data_age_ticks, 0, "RX freshness stayed green");
    assert!(
        !s.errors
            .as_slice()
            .iter()
            .any(|e| e.code == ErrorCode::CanStale),
        "no staleness was involved"
    );
    // TX-only fault: the encoders were never lost, references survive.
    assert!(s.homed, "a TX-only fault must not invalidate homing");

    // Every refused send was counted into the published stats.
    let failing_ticks = lost / 2 + lost / 2 + 4;
    assert_eq!(s.loop_stats.bus_tx_failures, failing_ticks);
    assert_eq!(s.loop_stats.bus_rx_failures, 0);

    // Recovery: link back + user clear wipes the latch (the streak reset
    // on the first successful send), and the arm re-enables.
    rig.core.bus_mut().fail_joint_tx = false;
    rig.cmd(RtCommand::ClearErrors);
    let settle = (par6_rt::errors::CLEAR_SETTLE_S / rig.dt).round() as u32;
    rig.tick_n(settle + 4);
    let s = rig.snap();
    assert!(s.errors.is_empty(), "clear wipes the latch: {:?}", s.errors);
    assert_eq!(s.mode, Mode::Idle, "auto-recovery from ACTIVE_ERROR");
    rig.cmd(RtCommand::Enable);
    assert_eq!(rig.snap().state, ArmState::Enabled);
}

/// A gripper-slot-only TX failure latches the gripper node's disconnect
/// and nothing else.
#[test]
fn gripper_only_tx_failure_latches_only_the_gripper_node() {
    let _serial = serial();
    let mut rig = Rig::new();
    rig.ready();
    rig.core.bus_mut().fail_gripper_tx = true;
    let lost = rig.lost_ticks;
    rig.tick_n(lost + 4);
    let s = rig.snap();
    assert!(s.error_active);
    assert_eq!(s.state, ArmState::Disabled);
    assert!(can_lost(&s, GRIPPER_ERR_IDX));
    for j in 0..MAX_JOINTS as u8 {
        assert!(
            !can_lost(&s, j),
            "J{j}: joint sends kept working — no joint latch"
        );
    }
    assert_eq!(s.loop_stats.bus_tx_failures, lost + 4);
}

/// Transient failures are tolerated: counted into the stats, never
/// latched. The backend's own `LinkHealth` rides the snapshot.
#[test]
fn transient_failures_are_counted_but_never_latch() {
    let _serial = serial();
    let mut rig = Rig::new();
    rig.ready();
    let lost = rig.lost_ticks;

    // A TX blip shorter than the lost window, then a long healthy run:
    // the streak reset on the first successful send, so nothing latches
    // no matter how much later the check runs.
    rig.core.bus_mut().fail_joint_tx = true;
    rig.tick_n(10);
    rig.core.bus_mut().fail_joint_tx = false;
    rig.tick_n(3 * lost);
    let s = rig.snap();
    assert!(!s.error_active, "a transient TX blip must not latch");
    assert_eq!(s.state, ArmState::Enabled);
    assert_eq!(s.mode, Mode::Idle);
    assert_eq!(s.loop_stats.bus_tx_failures, 10, "every refusal counted");

    // Same for the RX drain.
    rig.core.bus_mut().fail_rx = true;
    rig.tick_n(7);
    rig.core.bus_mut().fail_rx = false;
    rig.tick_n(5);
    let s = rig.snap();
    assert!(!s.error_active);
    assert_eq!(s.loop_stats.bus_rx_failures, 7);

    // The backend's aggregated link health reaches STATUS.
    assert!(
        s.link.rx_frames > 0,
        "the backend's LinkHealth counters must ride the snapshot"
    );
}

/// The failure arm is on the RT tick path: a fully dead bus — every
/// per-tick call failing, the streak crossing the latch threshold, the
/// mode reaction, freshness going lost — must allocate nothing.
#[test]
fn the_failure_arm_allocates_nothing() {
    let _serial = serial();
    let mut rig = Rig::new();
    // Warmup past boot one-shots, vendor config re-sends (ticks 50/150/
    // 300) and transient buffer growth anywhere in the stack.
    rig.tick_n(900);
    assert_eq!(rig.snap().mode, Mode::Idle);

    let bus = rig.core.bus_mut();
    bus.fail_joint_tx = true;
    bus.fail_gripper_tx = true;
    bus.fail_rx = true;
    bus.fail_poll = true;

    // No RX injection inside the window: the drain fails anyway, and the
    // window isolates the tick itself. 3× the lost window covers the
    // streak latch, the ACTIVE_ERROR transition and the freshness lost
    // crossing.
    let ticks = 3 * rig.lost_ticks;
    let dt = rig.dt;
    let before = ALLOCS.load(Ordering::Relaxed);
    for _ in 0..ticks {
        rig.core.tick(dt, false);
    }
    let after = ALLOCS.load(Ordering::Relaxed);
    assert_eq!(after - before, 0, "the failure arm must not allocate");

    // The window really exercised the latch path.
    let s = rig.snap();
    assert!(s.error_active);
    assert_eq!(s.state, ArmState::Disabled);
    assert_eq!(s.loop_stats.bus_tx_failures, 2 * ticks); // joint + gripper
    assert_eq!(s.loop_stats.bus_rx_failures, ticks);
}
