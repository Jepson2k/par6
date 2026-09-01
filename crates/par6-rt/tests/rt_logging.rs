//! Nothing on the RT tick path may log unthrottled.
//!
//! A bus fault is permanent while the link is down — `SocketCanBus` maps
//! ENETDOWN/ENOBUFS straight to `LinkDown`/`TxQueueFull` and `recv` maps
//! anything but `WouldBlock` to `LinkDown` — and the tick keeps
//! commanding through ACTIVE_ERROR, so the storm does not self-limit. At
//! 250 Hz the three bus failure sites are ~750 records/s of writer lock
//! plus `write(2)` on a SCHED_FIFO 99 thread whose stderr is a journald
//! pipe: a stalled reader stops the loop rather than degrading it.
//!
//! This is the only place the `Err` arm of the `DriverBus` contract runs
//! at all — neither `LoopbackBus` nor `SimBus` can fail — so the wrapper
//! below delegates every method to the real loopback backend and flips
//! only the three per-tick sends/drains to the errors the SocketCAN
//! backend really returns.

use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc;

use par6_bus::{
    BusError, BusState, DriverBus, Freshness, GripperCommand, JointCommand, LinkHealth,
    LoopbackBus, NodeId, PollAction,
};
use par6_config::{ConfigBundle, GripperConfig, RobotConfig};
use par6_rt::hooks::{ClampStream, RampJog};
use par6_rt::{
    sample_ring, CompletionPolicy, NoFk, RtCore, SharedDigitalIo, SharedFlashMarker,
    SharedLineGpio, SpecSettle, ZeroGravity,
};

/// Records emitted by the sites this test is about.
static BUS_FAULT_RECORDS: AtomicUsize = AtomicUsize::new(0);

struct CountingLogger;

static LOGGER: CountingLogger = CountingLogger;

impl log::Log for CountingLogger {
    fn enabled(&self, _meta: &log::Metadata) -> bool {
        true
    }

    fn log(&self, record: &log::Record) {
        let text = record.args().to_string();
        if text.contains("TX failed") || text.contains("RX drain failed") {
            BUS_FAULT_RECORDS.fetch_add(1, Ordering::Relaxed);
        }
    }

    fn flush(&self) {}
}

/// A `DriverBus` that delegates everything to the loopback reference
/// backend, with the per-tick RX drain and the two sends switchable to
/// the transport failures a down link really produces.
struct FailingBus {
    inner: LoopbackBus,
    down: bool,
}

impl DriverBus for FailingBus {
    fn begin_tick(&mut self, tick: u64) {
        self.inner.begin_tick(tick);
    }

    fn drain_rx(&mut self, state: &mut BusState) -> Result<usize, BusError> {
        if self.down {
            return Err(BusError::LinkDown);
        }
        self.inner.drain_rx(state)
    }

    fn send_joint_commands(&mut self, commands: &[JointCommand]) -> Result<(), BusError> {
        if self.down {
            return Err(BusError::LinkDown);
        }
        self.inner.send_joint_commands(commands)
    }

    fn send_gripper(&mut self, command: &GripperCommand) -> Result<(), BusError> {
        if self.down {
            return Err(BusError::TxQueueFull);
        }
        self.inner.send_gripper(command)
    }

    fn poll_step(&mut self) -> Result<(), BusError> {
        if self.down {
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

/// A link that is down for thousands of ticks must cost a bounded number
/// of log records — one per site per throttle window — and must still
/// report the fault at all.
#[test]
fn a_permanent_bus_fault_does_not_log_once_per_tick() {
    log::set_logger(&LOGGER).expect("this binary owns the logger");
    log::set_max_level(log::LevelFilter::Trace);

    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
    let bundle = ConfigBundle::load(&path).expect("PAR6 config bundle");
    let dt = bundle.robot.robot.tick_dt_s;
    let (_tx, rx) = mpsc::channel();
    let (gpio, _line) = SharedLineGpio::new(true);
    let (marker, _flash) = SharedFlashMarker::new();
    let (io, _io_lines) =
        SharedDigitalIo::new(bundle.robot.io.inputs.len(), bundle.robot.io.outputs.len());
    let (_producer, consumer) = sample_ring(64);
    let hooks = par6_rt::RtHooks {
        gravity: Box::new(ZeroGravity),
        jog: Box::new(RampJog::new(&bundle.robot)),
        stream: Box::new(ClampStream::new(&bundle.robot)),
        settle: Box::new(SpecSettle::new(
            CompletionPolicy::Settled,
            dt,
            bundle.robot.motion,
        )),
        estop: Box::new(gpio),
        io: Box::new(io),
        flash: Box::new(marker),
        commands: Box::new(rx),
        fk: Box::new(NoFk),
        samples: consumer,
    };
    let bus = FailingBus {
        inner: LoopbackBus::new(),
        down: false,
    };
    let (mut core, _handles) = RtCore::new(&bundle, bus, hooks).expect("core");

    core.tick(dt, false);
    core.bus_mut().down = true;
    BUS_FAULT_RECORDS.store(0, Ordering::Relaxed);

    let ticks = 2000u32;
    for _ in 0..ticks {
        core.tick(dt, false);
    }
    let records = BUS_FAULT_RECORDS.load(Ordering::Relaxed);

    // Three sites, one line each per one-second window.
    let window_ticks = (1.0 / dt).round() as u32;
    let ceiling = 3 * (ticks / window_ticks + 2) as usize;
    assert!(
        records <= ceiling,
        "{records} bus-fault log records over {ticks} ticks (ceiling {ceiling}); \
         the RT thread must not log once per tick"
    );
    assert!(
        records >= 3,
        "the fault must still be reported at least once per site (got {records})"
    );

    // Recovery re-arms the edge: the next fault reports immediately
    // rather than waiting out a window.
    core.bus_mut().down = false;
    core.tick(dt, false);
    BUS_FAULT_RECORDS.store(0, Ordering::Relaxed);
    core.bus_mut().down = true;
    core.tick(dt, false);
    assert_eq!(
        BUS_FAULT_RECORDS.load(Ordering::Relaxed),
        3,
        "a fresh fault after a healthy tick reports on the edge"
    );
}
