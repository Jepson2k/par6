//! SocketCAN hardware [`DriverBus`] backend: link layer, boot sequence,
//! and RT tick discipline.
//!
//! One classic-CAN 2.0A raw socket carries the whole driver plane. The
//! wire format is the frozen [`crate::spectral::codec`] — this module
//! only owns transport, scheduling and bookkeeping:
//!
//! - **Bring-up** (`open`): interface up at the configured bitrate /
//!   restart-ms / txqueuelen, `SO_SNDBUF` raised, `SO_TIMESTAMPNS` on
//!   (frame ages come from kernel RX timestamps), socket non-blocking.
//! - **Boot** ([`DriverBus::boot_configure`]): the seven config message
//!   types to every node, `repeats` passes, **paced per message-type
//!   batch** so the load cannot overrun the interface TX queue; optional
//!   kt fetch; RTR bus scan; an encoder sweep that puts each driver's
//!   first (14-bit-wrapped) accumulated reading in front of the RT loop
//!   for boot sector selection.
//! - **Tick**: non-blocking RX drain first, then TX — measure, then
//!   command. Nothing on that path allocates or blocks: every buffer is
//!   sized at boot, the drain stops at `EWOULDBLOCK` or the configured
//!   cap, and sends fail fast instead of waiting for queue space.
//! - **Health**: per-node freshness (10-tick warn, 50-tick latch at
//!   250 Hz), the per-frame live fault bit harvested from the
//!   arbitration id, and kernel link state sampled off the RT thread
//!   ([`link`]).
//!
//! Steady-state TX is one motion frame per joint + one gripper-slot
//! frame + one round-robin poll — 8 frames per tick for PAR6, against
//! the ~14-frame / 1.8 ms classic-CAN exchange the 250 Hz budget allows.
//!
//! Send failures are PROPAGATED, never swallowed (the vendor swallowed
//! them — a documented production bug class).

mod link;
/// The freshness clock in here is shared by every backend; the rest is
/// SocketCAN-only (see the module doc).
pub(crate) mod sched;
mod xstats;

use std::io::ErrorKind;
use std::time::{Duration, Instant, SystemTime};

use par6_config::{GripperConfig, KtSource, RobotConfig};
use socketcan::{CanSocket, EmbeddedFrame, Frame as _, Socket, SocketOptions};

use crate::bus::DriverBus;
use crate::spectral::codec::{
    decode_frame, encode_clear_error, encode_gripper_command, encode_joint_command, encode_limits,
    encode_poll, unpack_can_id, CanFrame, CommandId, DecodedFrame, Payload, CAN_MAX_DATA,
};
use crate::types::{
    BusError, BusState, DriveTune, Freshness, GripperCommand, JointCommand, LinkHealth, NodeId,
    PollAction, PollKind, MAX_NODES,
};

use sched::{
    boot_config_plan, config_frame, BootStep, FreshnessClock, NodeConfig, PollScheduler, PollStep,
    CONFIG_ORDER,
};

pub use link::OpenError;

/// RX frames drained per tick while bus-silent (FLASHING): bootloader
/// page frames alias application ids and arrive far faster than the
/// application plane.
const SILENT_RX_CAP: usize = 64;

/// SocketCAN [`DriverBus`] backend. Build with [`SocketCanBus::open`],
/// then [`DriverBus::boot_configure`] before any per-tick call.
#[derive(Debug)]
pub struct SocketCanBus {
    sock: CanSocket,
    monitor: link::LinkMonitor,
    interface: String,

    // Node map, installed by boot_configure.
    joint_nodes: Vec<NodeId>,
    gripper_node: NodeId,
    timing_dummy_node: NodeId,
    node_configs: Vec<NodeConfig>,
    /// Poll targets: the joint nodes, then the gripper slot.
    poll_nodes: Vec<NodeId>,
    /// Reusable boot plan buffer (boot-time allocation only).
    boot_plan: Vec<BootStep>,
    /// Decoded boot-sequence replies (kt, device info, first encoder
    /// readings), published to the caller by the first `drain_rx`.
    boot_state: BusState,
    boot_state_pending: bool,

    // Time base.
    tick: u64,
    dt: f64,
    /// Wall clock at the current tick's `begin_tick`, the reference for
    /// kernel RX timestamps.
    tick_start: SystemTime,

    // Per-tick bookkeeping.
    fresh: FreshnessClock,
    poll: PollScheduler,
    rx_cap: usize,
    config_pace: Duration,
    silent: bool,
    configured: bool,
    joints_sent_this_tick: bool,
    tx_frames_this_tick: u32,
    connected: u16,
    tx_errors: u64,
    rx_frames: u64,
}

impl SocketCanBus {
    /// Bring the configured interface up (when it is not already) and
    /// open the raw CAN socket on it.
    ///
    /// The socket is non-blocking with kernel RX timestamps enabled and
    /// `SO_SNDBUF` raised to the configured size (the kernel caps at
    /// `wmem_max`; falling short is logged, not fatal).
    pub fn open(cfg: &par6_config::BusConfig) -> Result<Self, OpenError> {
        link::ensure_up(cfg)?;
        let io = |source| OpenError::Io {
            iface: cfg.interface.clone(),
            source,
        };
        let sock = CanSocket::open(&cfg.interface).map_err(io)?;
        sock.set_nonblocking(true).map_err(io)?;
        // Frame ages come from the kernel arrival time, not from when the
        // drain got around to the frame — that is what separates "one
        // slow frame class" from "genuine backlog" in telemetry.
        sock.set_recv_timestamp(true).map_err(io)?;
        let want = cfg.sndbuf_bytes as usize;
        if let Err(e) = sock.as_raw_socket().set_send_buffer_size(want) {
            log::warn!("CAN '{}': SO_SNDBUF {want} refused ({e})", cfg.interface);
        } else if let Ok(got) = sock.as_raw_socket().send_buffer_size() {
            // The kernel doubles the request for bookkeeping and clamps
            // at wmem_max, so a smaller result is normal, not an error.
            log::info!("CAN '{}': SO_SNDBUF {got} (asked {want})", cfg.interface);
        }
        Ok(Self {
            sock,
            monitor: link::LinkMonitor::spawn(&cfg.interface),
            interface: cfg.interface.clone(),
            joint_nodes: Vec::new(),
            gripper_node: 0,
            timing_dummy_node: 0,
            node_configs: Vec::new(),
            poll_nodes: Vec::new(),
            boot_plan: Vec::new(),
            boot_state: BusState::new(),
            boot_state_pending: false,
            tick: 0,
            dt: 0.004,
            tick_start: SystemTime::now(),
            fresh: FreshnessClock::default(),
            poll: PollScheduler::default(),
            rx_cap: 32,
            config_pace: Duration::from_micros(500),
            silent: false,
            configured: false,
            joints_sent_this_tick: false,
            tx_frames_this_tick: 0,
            connected: 0,
            tx_errors: 0,
            rx_frames: 0,
        })
    }

    /// The interface this bus is bound to.
    pub fn interface(&self) -> &str {
        &self.interface
    }

    /// Frames transmitted during the current tick — the per-tick frame
    /// budget the classic-CAN ceiling constrains. Steady state is
    /// joints + gripper + one poll.
    pub fn tx_frames_this_tick(&self) -> u32 {
        self.tx_frames_this_tick
    }

    // ------------------------------------------------------------------
    // transport
    // ------------------------------------------------------------------

    /// Put one frame on the wire. Allocation-free; never blocks (the
    /// socket is non-blocking, so a full TX queue is an error, not a
    /// wait).
    fn send(&mut self, frame: &CanFrame) -> Result<(), BusError> {
        let (node, ..) = unpack_can_id(frame.id);
        let raw = if frame.rtr {
            socketcan::CanFrame::remote_from_raw_id(u32::from(frame.id), 0)
        } else {
            socketcan::CanFrame::from_raw_id(u32::from(frame.id), frame.payload())
        };
        let Some(raw) = raw else {
            return Err(BusError::InvalidCommand {
                reason: "frame does not fit a classic CAN 2.0A frame",
            });
        };
        match self.sock.write_frame(&raw) {
            Ok(()) => {
                self.tx_frames_this_tick += 1;
                Ok(())
            }
            Err(e) => {
                self.tx_errors += 1;
                Err(match e.kind() {
                    // Non-blocking write with no queue space: the kernel
                    // would otherwise drop the frame silently.
                    ErrorKind::WouldBlock => BusError::TxQueueFull,
                    _ if e.raw_os_error() == Some(105) => BusError::TxQueueFull, // ENOBUFS
                    ErrorKind::NotConnected | ErrorKind::BrokenPipe => BusError::LinkDown,
                    _ if e.raw_os_error() == Some(100) => BusError::LinkDown, // ENETDOWN
                    _ => BusError::Tx { node },
                })
            }
        }
    }

    /// Read one pending frame. `Ok(None)` = the queue is empty.
    fn recv(&mut self) -> Result<Option<(CanFrame, Option<SystemTime>)>, BusError> {
        loop {
            return match self.sock.read_frame_with_timestamps() {
                Ok((raw, ts)) => {
                    self.rx_frames += 1;
                    // 29-bit ids are not part of this protocol, and a
                    // kernel error frame's id word is error bits, not an
                    // arbitration id — neither may reach the decoder.
                    if raw.is_extended() || raw.is_error_frame() {
                        continue;
                    }
                    let data = raw.data();
                    let n = data.len().min(CAN_MAX_DATA);
                    let mut frame = CanFrame {
                        id: (raw.raw_id() & 0x7FF) as u16,
                        rtr: raw.is_remote_frame(),
                        dlc: n as u8,
                        data: [0u8; CAN_MAX_DATA],
                    };
                    frame.data[..n].copy_from_slice(&data[..n]);
                    Ok(Some((frame, ts.socket)))
                }
                Err(e) => match e.kind() {
                    ErrorKind::WouldBlock => Ok(None),
                    ErrorKind::Interrupted => continue,
                    _ => Err(BusError::LinkDown),
                },
            };
        }
    }

    /// Kernel arrival time → age in ticks, saturating at zero for clock
    /// jitter that puts a frame marginally in the future.
    fn age_ticks(&self, ts: Option<SystemTime>) -> u64 {
        let Some(ts) = ts else { return 0 };
        match self.tick_start.duration_since(ts) {
            Ok(d) => (d.as_secs_f64() / self.dt) as u64,
            Err(_) => 0,
        }
    }

    // ------------------------------------------------------------------
    // RX bookkeeping
    // ------------------------------------------------------------------

    /// Decode one frame into `state` and update this node's freshness.
    /// The arbitration id's err bit and node are harvested BEFORE payload
    /// dispatch, so a refused frame still refreshes the live fault signal
    /// and the data-age clock.
    fn apply_rx(&mut self, frame: &CanFrame, state: &mut BusState) {
        let node = match decode_frame(frame) {
            Ok(d) => {
                apply_payload(&d, state);
                d.node
            }
            Err(e) => {
                state.nodes[usize::from(e.node())].live_error_bit = e.err_bit();
                e.node()
            }
        };
        let n = usize::from(node);
        if self.fresh.mark(node, self.tick) {
            state.reconnected_mask |= 1 << n;
        }
        let (_, raw_cmd, _) = unpack_can_id(frame.id);
        if raw_cmd == CommandId::RespondGripperData.raw() {
            self.fresh.mark_gripper(self.tick);
        }
    }

    /// Refresh every node's published data age from the freshness clock.
    fn publish_ages(&self, state: &mut BusState) {
        for n in 0..MAX_NODES {
            state.nodes[n].data_age_ticks = self.fresh.age(n as NodeId, self.tick);
        }
        state.gripper.data_age_ticks = self.fresh.gripper_age(self.tick);
    }

    /// Merge everything the boot sequence learned (kt, device identity,
    /// the first encoder readings the RT loop needs for boot sector
    /// selection) into the caller's state, once.
    fn publish_boot_state(&mut self, state: &mut BusState) {
        for n in 0..MAX_NODES {
            let b = &self.boot_state.nodes[n];
            let s = &mut state.nodes[n];
            s.position_ticks = b.position_ticks.or(s.position_ticks);
            s.speed_ticks_s = b.speed_ticks_s.or(s.speed_ticks_s);
            s.current_ma = b.current_ma.or(s.current_ma);
            s.temperature_c = b.temperature_c.or(s.temperature_c);
            s.voltage_mv = b.voltage_mv.or(s.voltage_mv);
            s.error_flags = b.error_flags.or(s.error_flags);
            s.kt_nm_a = b.kt_nm_a.or(s.kt_nm_a);
            s.device_info = b.device_info.or(s.device_info);
        }
        if self.boot_state.gripper.reply.is_some() {
            state.gripper.reply = self.boot_state.gripper.reply;
        }
        self.boot_state_pending = false;
    }

    // ------------------------------------------------------------------
    // boot sequence
    // ------------------------------------------------------------------

    /// Drain replies for `wait`, decoding into `boot_state` and marking
    /// which nodes answered. Boot-time only (it sleeps).
    fn collect_for(&mut self, wait: Duration) {
        let deadline = Instant::now() + wait;
        loop {
            match self.recv() {
                Ok(Some((frame, _))) => {
                    let (node, ..) = unpack_can_id(frame.id);
                    self.connected |= 1 << u16::from(node);
                    // Boot replies precede the first tick; they enter the
                    // freshness clock at tick 0 so a node that answers
                    // boot and then dies latches like any other.
                    self.fresh.mark(node, 0);
                    if let Ok(d) = decode_frame(&frame) {
                        apply_payload(&d, &mut self.boot_state);
                        self.boot_state_pending = true;
                    }
                }
                Ok(None) => {
                    if Instant::now() >= deadline {
                        return;
                    }
                    std::thread::sleep(Duration::from_micros(200));
                }
                Err(_) => return,
            }
        }
    }

    /// Fetch each node's torque constant from the driver (cmd 33 RTR),
    /// with the configured timeout / retries / rounds. Values land in
    /// `boot_state` and reach the RT loop with the first drain.
    fn fetch_kt(&mut self, robot: &RobotConfig) {
        let f = robot.bus.kt_fetch;
        let timeout = Duration::from_secs_f64(f.timeout_s);
        let nodes: Vec<NodeId> = self.node_configs.iter().map(|c| c.node).collect();
        for _round in 0..f.rounds {
            for node in &nodes {
                if self.boot_state.nodes[usize::from(*node)].kt_nm_a.is_some() {
                    continue;
                }
                for _retry in 0..f.retries {
                    let frame = encode_poll(*node, PollKind::Kt);
                    if let Err(e) = self.send(&frame) {
                        log::warn!("kt fetch: node {node}: {e}");
                        break;
                    }
                    self.collect_for(timeout);
                    if self.boot_state.nodes[usize::from(*node)].kt_nm_a.is_some() {
                        break;
                    }
                }
            }
        }
        for node in &nodes {
            match self.boot_state.nodes[usize::from(*node)].kt_nm_a {
                Some(kt) => log::info!("node {node}: kt {kt} Nm/A (from driver)"),
                None => log::warn!(
                    "node {node}: kt fetch got no reply; the configured kt stays in effect"
                ),
            }
        }
    }

    /// RTR-ping every node id, `rounds` times, and record who answers.
    fn bus_scan(&mut self, robot: &RobotConfig) {
        let wait = Duration::from_secs_f64(robot.bus.scan.wait_s);
        for _round in 0..robot.bus.scan.rounds {
            for node in 0..MAX_NODES as NodeId {
                if self.send(&encode_poll(node, PollKind::Ping)).is_err() {
                    continue;
                }
                self.collect_for(wait);
            }
        }
        log::info!(
            "CAN '{}': bus scan found nodes {:#06x}",
            self.interface,
            self.connected
        );
    }

    /// Ask every configured node for its accumulated encoder position
    /// (cmd 28 RTR) so the RT loop's boot sector selection runs on a real
    /// first reading rather than on whatever the first motion reply
    /// happens to carry.
    fn seed_encoders(&mut self, wait: Duration) {
        let nodes: Vec<NodeId> = self.node_configs.iter().map(|c| c.node).collect();
        for node in nodes {
            if self.send(&encode_poll(node, PollKind::Encoder)).is_err() {
                continue;
            }
            self.collect_for(wait);
        }
    }

    fn ensure_ready(&self) -> Result<(), BusError> {
        if !self.configured {
            return Err(BusError::NotConfigured);
        }
        if self.silent {
            return Err(BusError::InvalidCommand {
                reason: "TX while bus-silent (FLASHING)",
            });
        }
        Ok(())
    }

    fn node_config(&self, node: NodeId) -> Option<NodeConfig> {
        self.node_configs.iter().copied().find(|c| c.node == node)
    }
}

/// Write one decoded reply into `state`, live fault bit included. Shared
/// by the tick drain and the boot collector.
fn apply_payload(decoded: &DecodedFrame, state: &mut BusState) {
    let n = usize::from(decoded.node);
    match decoded.payload {
        Payload::Motion {
            position_ticks,
            speed_ticks_s,
            current_ma,
        } => {
            state.nodes[n].position_ticks = Some(position_ticks);
            state.nodes[n].speed_ticks_s = Some(speed_ticks_s);
            state.nodes[n].current_ma = Some(current_ma);
        }
        Payload::Encoder {
            position_ticks,
            speed_ticks_s,
        } => {
            state.nodes[n].position_ticks = Some(position_ticks);
            state.nodes[n].speed_ticks_s = Some(speed_ticks_s);
        }
        Payload::Hall {
            position_ticks,
            state: hall,
        } => {
            state.nodes[n].position_ticks = Some(position_ticks);
            state.nodes[n].hall = Some(hall);
        }
        Payload::Temperature { deg_c } => state.nodes[n].temperature_c = Some(deg_c),
        Payload::Voltage { mv } => state.nodes[n].voltage_mv = Some(mv),
        Payload::IqCurrent { ma } => state.nodes[n].current_ma = Some(ma),
        Payload::Errors(flags) => state.nodes[n].error_flags = Some(flags),
        Payload::DeviceInfo(info) => state.nodes[n].device_info = Some(info),
        Payload::Kt { nm_per_a } => state.nodes[n].kt_nm_a = Some(nm_per_a),
        Payload::Gripper(reply) => {
            state.gripper.reply = Some(reply);
            state.gripper.live_error_bit = decoded.err_bit;
        }
        Payload::Ping | Payload::Heartbeat => {}
    }
    state.nodes[n].live_error_bit = decoded.err_bit;
}

impl DriverBus for SocketCanBus {
    fn begin_tick(&mut self, tick: u64) {
        debug_assert!(tick >= self.tick, "tick must be non-decreasing");
        self.tick = tick;
        self.tick_start = SystemTime::now();
        self.joints_sent_this_tick = false;
        self.tx_frames_this_tick = 0;
        if !self.silent {
            self.fresh.latch_lost(tick);
        }
    }

    fn drain_rx(&mut self, state: &mut BusState) -> Result<usize, BusError> {
        if !self.configured {
            return Err(BusError::NotConfigured);
        }
        if self.boot_state_pending {
            self.publish_boot_state(state);
        }
        state.frames_last_drain = 0;
        state.frame_age_min_ticks = 0;
        state.frame_age_max_ticks = 0;
        state.reconnected_mask = 0;
        let cap = if self.silent {
            SILENT_RX_CAP
        } else {
            self.rx_cap
        };
        let mut count = 0usize;
        let mut age_min = u64::MAX;
        let mut age_max = 0u64;
        while count < cap {
            let Some((frame, ts)) = self.recv()? else {
                break;
            };
            count += 1;
            if self.silent {
                // FLASHING: drain-and-DISCARD. Bootloader page frames
                // alias application ids — nothing here may decode.
                continue;
            }
            let age = self.age_ticks(ts);
            age_min = age_min.min(age);
            age_max = age_max.max(age);
            self.apply_rx(&frame, state);
        }
        state.frames_last_drain = count as u32;
        if count > 0 && !self.silent {
            state.frame_age_min_ticks = age_min;
            state.frame_age_max_ticks = age_max;
        }
        self.publish_ages(state);
        Ok(count)
    }

    fn send_joint_commands(&mut self, commands: &[JointCommand]) -> Result<(), BusError> {
        self.ensure_ready()?;
        if commands.len() != self.joint_nodes.len() {
            return Err(BusError::InvalidCommand {
                reason: "command slice length != configured joint count",
            });
        }
        if self.joints_sent_this_tick {
            return Err(BusError::InvalidCommand {
                reason: "second joint send in one tick (single-send invariant)",
            });
        }
        self.joints_sent_this_tick = true;
        for (i, cmd) in commands.iter().enumerate() {
            let node = self.joint_nodes[i];
            let frame = encode_joint_command(node, cmd).map_err(|_| BusError::InvalidCommand {
                reason: "cmd 2 has no wire form for position without velocity",
            })?;
            if let Some(f) = frame {
                self.send(&f)?;
            }
        }
        Ok(())
    }

    fn send_gripper(&mut self, command: &GripperCommand) -> Result<(), BusError> {
        self.ensure_ready()?;
        let frame = encode_gripper_command(self.gripper_node, self.timing_dummy_node, command)
            .map_err(|_| BusError::InvalidCommand {
                reason: "cmd 2 has no wire form for position without velocity",
            })?;
        match frame {
            Some(f) => self.send(&f),
            None => Ok(()),
        }
    }

    fn poll_step(&mut self) -> Result<(), BusError> {
        if !self.configured {
            return Err(BusError::NotConfigured);
        }
        if self.silent {
            return Ok(());
        }
        let Some(step) = self.poll.step() else {
            return Ok(());
        };
        match step {
            PollStep::Override(PollAction::Poll { node, kind }) => {
                let f = encode_poll(node, kind);
                self.send(&f)
            }
            PollStep::Override(PollAction::ClearError { node }) => {
                let f = encode_clear_error(node);
                self.send(&f)
            }
            PollStep::Override(PollAction::ResendConfig { node }) => {
                self.resend_node_config(node, 1)
            }
            PollStep::Poll { target, kind } => {
                let node = self.poll_nodes[target];
                let f = encode_poll(node, kind);
                self.send(&f)
            }
        }
    }

    fn queue_poll_override(&mut self, action: PollAction, repeats: u16) {
        self.poll.queue_override(action, repeats);
    }

    fn boot_configure(
        &mut self,
        robot: &RobotConfig,
        gripper: Option<&GripperConfig>,
        repeats: u8,
    ) -> Result<(), BusError> {
        self.dt = robot.robot.tick_dt_s;
        self.joint_nodes = robot.joints.iter().map(|j| j.node_id).collect();
        self.gripper_node = robot.bus.gripper_node;
        self.timing_dummy_node = robot.bus.timing_dummy_node;
        let has_can_gripper = gripper.is_some_and(|g| g.driver.is_some());
        self.rx_cap = robot.bus.rx_frames_per_tick_cap as usize;
        self.config_pace = Duration::from_secs_f64(robot.bus.config_pace_s);
        self.fresh.configure(
            u64::from(robot.ticks(robot.bus.stale_warn_s)),
            u64::from(robot.ticks(robot.bus.lost_s)),
        );
        self.poll_nodes = self.joint_nodes.clone();
        self.poll_nodes.push(self.gripper_node);
        self.poll.configure(self.poll_nodes.len());
        self.boot_state = BusState::new();
        self.boot_state_pending = false;
        self.connected = 0;

        self.node_configs = robot
            .joints
            .iter()
            .map(|j| NodeConfig {
                node: j.node_id,
                watchdog_ms: j.watchdog_timeout_ms,
                watchdog_action: robot.bus.watchdog_action,
                velocity_limit_ticks_s: j.velocity_limit_ticks_s,
                ilim_ma: j.ilim_ma,
                voltage_limit_mv: j.voltage_limit_mv,
                gains: j.gains,
            })
            .collect();
        if has_can_gripper {
            let d = gripper
                .and_then(|g| g.driver.as_ref())
                .expect("has_can_gripper");
            self.node_configs.push(NodeConfig {
                node: self.gripper_node,
                watchdog_ms: d.watchdog_timeout_ms,
                watchdog_action: robot.bus.watchdog_action,
                velocity_limit_ticks_s: d.velocity_limit_ticks_s,
                ilim_ma: d.ilim_ma,
                voltage_limit_mv: d.voltage_limit_mv,
                gains: d.gains,
            });
        }
        self.configured = true;

        // Paced config load. The whole load enqueues in microseconds
        // against a ~10 frames/ms drain, so an unpaced burst silently
        // overruns the interface TX queue.
        boot_config_plan(&self.node_configs, repeats, &mut self.boot_plan);
        let plan = std::mem::take(&mut self.boot_plan);
        let mut result = Ok(());
        for step in &plan {
            match step {
                BootStep::Frame(f) => {
                    if let Err(e) = self.send(f) {
                        result = Err(e);
                        break;
                    }
                }
                BootStep::Pace => std::thread::sleep(self.config_pace),
            }
        }
        self.boot_plan = plan;
        result?;

        if robot.robot.kt_source == KtSource::Auto {
            self.fetch_kt(robot);
        }
        self.bus_scan(robot);
        self.seed_encoders(Duration::from_secs_f64(robot.bus.scan.wait_s));

        let expected: u16 = self
            .node_configs
            .iter()
            .fold(0u16, |m, c| m | (1 << u16::from(c.node)));
        let missing = expected & !self.connected;
        if missing != 0 {
            log::warn!(
                "CAN '{}': configured nodes {missing:#06x} did not answer the bus scan",
                self.interface
            );
        }
        Ok(())
    }

    fn resend_node_config(&mut self, node: NodeId, repeats: u8) -> Result<(), BusError> {
        self.ensure_ready()?;
        let Some(c) = self.node_config(node) else {
            return Err(BusError::InvalidCommand {
                reason: "resend_node_config for a node with no stored configuration",
            });
        };
        for _ in 0..repeats {
            for kind in CONFIG_ORDER {
                let f = config_frame(kind, &c);
                self.send(&f)?;
            }
        }
        Ok(())
    }

    fn retune_node(&mut self, node: NodeId, tune: &DriveTune, repeats: u8) -> Result<(), BusError> {
        self.ensure_ready()?;
        let Some(c) = self.node_configs.iter_mut().find(|c| c.node == node) else {
            return Err(BusError::InvalidCommand {
                reason: "retune_node for a node with no stored configuration",
            });
        };
        c.gains = tune.gains;
        c.ilim_ma = tune.ilim_ma;
        c.velocity_limit_ticks_s = tune.velocity_limit_ticks_s;
        c.voltage_limit_mv = tune.voltage_limit_mv;
        self.resend_node_config(node, repeats)
    }

    fn send_limits(
        &mut self,
        node: NodeId,
        velocity_limit_ticks_s: f32,
        current_limit_ma: f32,
        repeats: u8,
    ) -> Result<(), BusError> {
        self.ensure_ready()?;
        for _ in 0..repeats {
            let f = encode_limits(node, velocity_limit_ticks_s, current_limit_ma);
            self.send(&f)?;
        }
        Ok(())
    }

    fn send_clear_error(&mut self, node: NodeId, repeats: u8) -> Result<(), BusError> {
        self.ensure_ready()?;
        for _ in 0..repeats {
            let f = encode_clear_error(node);
            self.send(&f)?;
        }
        Ok(())
    }

    fn set_silent(&mut self, silent: bool) {
        self.silent = silent;
    }

    fn is_silent(&self) -> bool {
        self.silent
    }

    fn freshness(&self, node: NodeId) -> Freshness {
        self.fresh.classify(node, self.tick)
    }

    fn clear_lost_latch(&mut self, node: NodeId) {
        self.fresh.clear_latch(node, self.tick);
    }

    fn rebase_freshness(&mut self) {
        self.fresh.rebase(self.tick);
    }

    fn connected_nodes(&self) -> u16 {
        self.connected
    }

    fn link_health(&self) -> LinkHealth {
        LinkHealth {
            tx_errors: self.tx_errors,
            rx_frames: self.rx_frames,
            ..self.monitor.health()
        }
    }
}

#[cfg(test)]
mod tests;
