//! In-memory loopback [`DriverBus`] — the reference implementation the
//! contract unit tests run against.
//!
//! It models frame TRANSPORT and bookkeeping (tick time base, freshness
//! warn/latch, reconnect edges, round-robin polling, the single-slot
//! override queue, silent mode) but no wire codec: tests inject
//! already-decoded [`Reply`] values and read back a TX log of
//! already-typed [`TxRecord`]s. It allocates for the TX log and is NOT a
//! production backend — real backends (SocketCAN, sim) live elsewhere and
//! must honor the no-alloc-per-tick contract.

use std::collections::VecDeque;

use par6_config::{GripperConfig, RobotConfig};

use crate::bus::DriverBus;
use crate::hw::sched::FreshnessClock;
use crate::types::{
    BusError, BusState, DeviceInfo, DriveTune, ErrorFlags, Freshness, GripperCommand, GripperReply,
    HallState, JointCommand, LinkHealth, NodeId, PollAction, PollKind, MAX_NODES,
};

/// A decoded reply a test injects into the loopback RX queue.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Reply {
    /// cmd 3: position/speed/current.
    Motion {
        /// Source node.
        node: NodeId,
        /// Motor position \[ticks\].
        position_ticks: i32,
        /// Motor speed \[ticks/s\].
        speed_ticks_s: i32,
        /// Motor current \[mA\].
        current_ma: i16,
    },
    /// cmd 23 reply.
    Temperature {
        /// Source node.
        node: NodeId,
        /// Temperature \[°C\].
        deg_c: i16,
    },
    /// cmd 24 reply.
    Voltage {
        /// Source node.
        node: NodeId,
        /// Bus voltage \[mV\].
        mv: i16,
    },
    /// cmd 26 reply.
    Errors {
        /// Source node.
        node: NodeId,
        /// Decoded flag bits.
        flags: ErrorFlags,
    },
    /// cmd 32 reply (HALL homing).
    Hall {
        /// Source node.
        node: NodeId,
        /// Position latched at trigger \[ticks\].
        position_ticks: i32,
        /// Decoded hall bits.
        hall: HallState,
    },
    /// cmd 33 reply.
    Kt {
        /// Source node.
        node: NodeId,
        /// Torque constant \[Nm/A\].
        kt_nm_a: f32,
    },
    /// cmd 25 reply.
    DeviceInfo {
        /// Source node.
        node: NodeId,
        /// Identity payload.
        info: DeviceInfo,
    },
    /// cmd 60 reply (firmware gripper).
    Gripper {
        /// Decoded reply payload.
        reply: GripperReply,
    },
}

impl Reply {
    fn node(&self, gripper_node: NodeId) -> NodeId {
        match *self {
            Reply::Motion { node, .. }
            | Reply::Temperature { node, .. }
            | Reply::Voltage { node, .. }
            | Reply::Errors { node, .. }
            | Reply::Hall { node, .. }
            | Reply::Kt { node, .. }
            | Reply::DeviceInfo { node, .. } => node,
            Reply::Gripper { .. } => gripper_node,
        }
    }
}

/// One frame everything the loopback transmitted, for test assertions.
#[derive(Debug, Clone, PartialEq)]
pub enum TxRecord {
    /// One motion frame per arm joint, config order.
    Joints(Vec<JointCommand>),
    /// The gripper-slot frame (`NoGripper` = RTR ping to the timing
    /// dummy node).
    Gripper(GripperCommand),
    /// A telemetry poll (RTR).
    Poll {
        /// Target node.
        node: NodeId,
        /// Request kind.
        kind: PollKind,
    },
    /// Clear_Error (cmd 1).
    ClearError {
        /// Target node.
        node: NodeId,
    },
    /// Limits frame (cmd 20).
    Limits {
        /// Target node.
        node: NodeId,
        /// Velocity limit \[ticks/s\].
        velocity_limit_ticks_s: f32,
        /// Current limit \[mA\].
        current_limit_ma: f32,
    },
    /// One full configuration pass for a node.
    ConfigPass {
        /// Target node.
        node: NodeId,
    },
    /// A live drive retune preceding its config passes.
    Retune {
        /// Target node.
        node: NodeId,
        /// The tune stored and pushed.
        tune: DriveTune,
    },
}

struct Injected {
    enqueued_tick: u64,
    err_bit: bool,
    reply: Reply,
}

/// The loopback bus. Construct with [`LoopbackBus::new`], then
/// [`DriverBus::boot_configure`] with a real [`RobotConfig`] before any
/// per-tick call.
pub struct LoopbackBus {
    tick: u64,
    silent: bool,
    configured: bool,
    joint_nodes: Vec<NodeId>,
    gripper_node: NodeId,
    timing_dummy_node: NodeId,
    rx_cap: usize,
    fresh: FreshnessClock,
    connected: u16,
    rx_queue: VecDeque<Injected>,
    /// Everything transmitted, as `(tick, record)` — the test oracle.
    pub tx_log: Vec<(u64, TxRecord)>,
    poll_cursor: u64,
    override_slot: Option<(PollAction, u16)>,
    joints_sent_this_tick: bool,
    health: LinkHealth,
}

impl LoopbackBus {
    /// An unconfigured loopback bus.
    pub fn new() -> Self {
        Self {
            tick: 0,
            silent: false,
            configured: false,
            joint_nodes: Vec::new(),
            gripper_node: 0,
            timing_dummy_node: 0,
            rx_cap: 32,
            fresh: FreshnessClock::default(),
            connected: 0,
            rx_queue: VecDeque::new(),
            tx_log: Vec::new(),
            poll_cursor: 0,
            override_slot: None,
            joints_sent_this_tick: false,
            health: LinkHealth::default(),
        }
    }

    /// Inject a decoded reply into the RX queue; it is consumed by the
    /// next [`DriverBus::drain_rx`]. `err_bit` models the arbitration-id
    /// live fault bit of that frame.
    pub fn inject(&mut self, err_bit: bool, reply: Reply) {
        self.rx_queue.push_back(Injected {
            enqueued_tick: self.tick,
            err_bit,
            reply,
        });
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

    fn poll_targets(&self) -> usize {
        self.joint_nodes.len() + 1 // + gripper slot
    }

    fn poll_target_node(&self, idx: usize) -> NodeId {
        if idx < self.joint_nodes.len() {
            self.joint_nodes[idx]
        } else {
            self.gripper_node
        }
    }

    fn record_config_pass(&mut self, node: NodeId) {
        let tick = self.tick;
        self.tx_log.push((tick, TxRecord::ConfigPass { node }));
    }

    fn apply(reply: &Reply, err_bit: bool, state: &mut BusState, gripper_node: NodeId) {
        let node = usize::from(reply.node(gripper_node));
        match *reply {
            Reply::Motion {
                position_ticks,
                speed_ticks_s,
                current_ma,
                ..
            } => {
                state.nodes[node].position_ticks = Some(position_ticks);
                state.nodes[node].speed_ticks_s = Some(speed_ticks_s);
                state.nodes[node].current_ma = Some(current_ma);
            }
            Reply::Temperature { deg_c, .. } => state.nodes[node].temperature_c = Some(deg_c),
            Reply::Voltage { mv, .. } => state.nodes[node].voltage_mv = Some(mv),
            Reply::Errors { flags, .. } => state.nodes[node].error_flags = Some(flags),
            Reply::Hall {
                position_ticks,
                hall,
                ..
            } => {
                state.nodes[node].position_ticks = Some(position_ticks);
                state.nodes[node].hall = Some(hall);
            }
            Reply::Kt { kt_nm_a, .. } => state.nodes[node].kt_nm_a = Some(kt_nm_a),
            Reply::DeviceInfo { info, .. } => state.nodes[node].device_info = Some(info),
            Reply::Gripper { reply } => {
                state.gripper.reply = Some(reply);
                state.gripper.live_error_bit = err_bit;
            }
        }
        state.nodes[node].live_error_bit = err_bit;
    }
}

impl Default for LoopbackBus {
    fn default() -> Self {
        Self::new()
    }
}

impl DriverBus for LoopbackBus {
    fn begin_tick(&mut self, tick: u64) {
        debug_assert!(tick >= self.tick, "tick must be non-decreasing");
        self.tick = tick;
        self.joints_sent_this_tick = false;
        if !self.silent {
            self.fresh.latch_lost(tick);
        }
    }

    fn drain_rx(&mut self, state: &mut BusState) -> Result<usize, BusError> {
        if !self.configured {
            return Err(BusError::NotConfigured);
        }
        state.frames_last_drain = 0;
        state.frame_age_min_ticks = 0;
        state.frame_age_max_ticks = 0;
        state.reconnected_mask = 0;
        let cap = if self.silent { 64 } else { self.rx_cap };
        let mut count = 0usize;
        let mut age_min = u64::MAX;
        let mut age_max = 0u64;
        while count < cap {
            let Some(frame) = self.rx_queue.pop_front() else {
                break;
            };
            count += 1;
            self.health.rx_frames += 1;
            if self.silent {
                // FLASHING: drain-and-discard, bootloader frames alias
                // application ids — never decode.
                continue;
            }
            let age = self.tick.saturating_sub(frame.enqueued_tick);
            age_min = age_min.min(age);
            age_max = age_max.max(age);
            let node = frame.reply.node(self.gripper_node);
            if self.fresh.mark(node, self.tick) {
                state.reconnected_mask |= 1 << usize::from(node);
            }
            if matches!(frame.reply, Reply::Gripper { .. }) {
                self.fresh.mark_gripper(self.tick);
            }
            Self::apply(&frame.reply, frame.err_bit, state, self.gripper_node);
        }
        state.frames_last_drain = count as u32;
        if count > 0 && !self.silent {
            state.frame_age_min_ticks = age_min;
            state.frame_age_max_ticks = age_max;
        }
        // Refresh every node's data age against the current tick.
        for n in 0..MAX_NODES {
            state.nodes[n].data_age_ticks = self.fresh.age(n as NodeId, self.tick);
        }
        state.gripper.data_age_ticks = self.fresh.gripper_age(self.tick);
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
        let tick = self.tick;
        self.tx_log
            .push((tick, TxRecord::Joints(commands.to_vec())));
        Ok(())
    }

    fn send_gripper(&mut self, command: &GripperCommand) -> Result<(), BusError> {
        self.ensure_ready()?;
        let tick = self.tick;
        self.tx_log.push((tick, TxRecord::Gripper(*command)));
        Ok(())
    }

    fn poll_step(&mut self) -> Result<(), BusError> {
        if !self.configured {
            return Err(BusError::NotConfigured);
        }
        if self.silent {
            return Ok(()); // bus-silent: polls suppressed
        }
        if let Some((action, repeats)) = self.override_slot.take() {
            match action {
                PollAction::Poll { node, kind } => {
                    let tick = self.tick;
                    self.tx_log.push((tick, TxRecord::Poll { node, kind }));
                }
                PollAction::ClearError { node } => {
                    let tick = self.tick;
                    self.tx_log.push((tick, TxRecord::ClearError { node }));
                }
                PollAction::ResendConfig { node } => self.record_config_pass(node),
            }
            if repeats > 1 {
                self.override_slot = Some((action, repeats - 1));
            }
            return Ok(());
        }
        let idx = (self.poll_cursor / 3) as usize % self.poll_targets();
        let node = self.poll_target_node(idx);
        let kind = match self.poll_cursor % 3 {
            0 => PollKind::Temperature,
            1 => PollKind::Voltage,
            _ => PollKind::Errors,
        };
        self.poll_cursor += 1;
        let tick = self.tick;
        self.tx_log.push((tick, TxRecord::Poll { node, kind }));
        Ok(())
    }

    fn queue_poll_override(&mut self, action: PollAction, repeats: u16) {
        if repeats == 0 {
            return;
        }
        self.override_slot = Some((action, repeats));
    }

    fn boot_configure(
        &mut self,
        robot: &RobotConfig,
        gripper: Option<&GripperConfig>,
        repeats: u8,
    ) -> Result<(), BusError> {
        self.joint_nodes = robot.joints.iter().map(|j| j.node_id).collect();
        self.gripper_node = robot.bus.gripper_node;
        self.timing_dummy_node = robot.bus.timing_dummy_node;
        self.fresh.configure(
            u64::from(robot.ticks(robot.bus.stale_warn_s)),
            u64::from(robot.ticks(robot.bus.lost_s)),
        );
        self.rx_cap = robot.bus.rx_frames_per_tick_cap as usize;
        self.connected = self
            .joint_nodes
            .iter()
            .fold(0u16, |m, n| m | (1 << u16::from(*n)));
        let has_can_gripper = gripper.is_some_and(|g| g.driver.is_some());
        if has_can_gripper {
            self.connected |= 1 << u16::from(self.gripper_node);
        }
        self.configured = true;
        for _pass in 0..repeats {
            let nodes: Vec<NodeId> = self.joint_nodes.clone();
            for node in nodes {
                self.record_config_pass(node);
            }
            if has_can_gripper {
                let node = self.gripper_node;
                self.record_config_pass(node);
            }
        }
        Ok(())
    }

    fn resend_node_config(&mut self, node: NodeId, repeats: u8) -> Result<(), BusError> {
        self.ensure_ready()?;
        for _ in 0..repeats {
            self.record_config_pass(node);
        }
        Ok(())
    }

    fn retune_node(&mut self, node: NodeId, tune: &DriveTune, repeats: u8) -> Result<(), BusError> {
        self.ensure_ready()?;
        if !self.joint_nodes.contains(&node) && node != self.gripper_node {
            return Err(BusError::InvalidCommand {
                reason: "retune_node for a node with no stored configuration",
            });
        }
        let tick = self.tick;
        self.tx_log
            .push((tick, TxRecord::Retune { node, tune: *tune }));
        for _ in 0..repeats {
            self.record_config_pass(node);
        }
        Ok(())
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
            let tick = self.tick;
            self.tx_log.push((
                tick,
                TxRecord::Limits {
                    node,
                    velocity_limit_ticks_s,
                    current_limit_ma,
                },
            ));
        }
        Ok(())
    }

    fn send_clear_error(&mut self, node: NodeId, repeats: u8) -> Result<(), BusError> {
        self.ensure_ready()?;
        for _ in 0..repeats {
            let tick = self.tick;
            self.tx_log.push((tick, TxRecord::ClearError { node }));
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
        self.health
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::ObjectDetection;
    use std::path::PathBuf;

    fn configured_bus() -> (LoopbackBus, RobotConfig) {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
        let robot = RobotConfig::load(&path).expect("PAR6.toml");
        let gpath =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/grippers/SSG48.toml");
        let gripper = GripperConfig::load(&gpath).expect("SSG48.toml");
        let mut bus = LoopbackBus::new();
        bus.boot_configure(&robot, Some(&gripper), 3).unwrap();
        (bus, robot)
    }

    #[test]
    fn boot_configure_passes_and_send_contracts() {
        let mut bus = LoopbackBus::new();
        // Nothing works before boot_configure.
        assert_eq!(
            bus.send_joint_commands(&[JointCommand::idle(); 6]),
            Err(BusError::NotConfigured)
        );
        let (mut bus, _) = configured_bus();
        // 3 passes × (6 joints + gripper) config passes recorded.
        let passes = bus
            .tx_log
            .iter()
            .filter(|(_, r)| matches!(r, TxRecord::ConfigPass { .. }))
            .count();
        assert_eq!(passes, 3 * 7);
        assert_eq!(bus.connected_nodes(), 0b0000_0000_0111_1111);

        bus.begin_tick(1);
        // Option channels survive the send verbatim: None is "omitted",
        // never coerced to zero.
        let cmds = [
            JointCommand::position(1000, 2000, 300),
            JointCommand::velocity(-500, 250),
            JointCommand::current(-150),
            JointCommand::hall(4500, 2),
            JointCommand::pd(10, 0, 50),
            JointCommand::idle(),
        ];
        bus.send_joint_commands(&cmds).unwrap();
        let Some((_, TxRecord::Joints(sent))) = bus
            .tx_log
            .iter()
            .find(|(_, r)| matches!(r, TxRecord::Joints(_)))
        else {
            panic!("no joint send recorded");
        };
        assert_eq!(sent[1].pos, None);
        assert_eq!(sent[1].vel, Some(-500));
        assert_eq!(sent[2].pos, None);
        assert_eq!(sent[2].vel, None);
        assert_eq!(sent[2].cur_ma, Some(-150));
        assert_eq!(sent[3].pack, Pack::Hall { trigger_value: 2 });
        assert_eq!(sent.as_slice(), &cmds);

        // Single-send-per-tick invariant.
        assert!(matches!(
            bus.send_joint_commands(&cmds),
            Err(BusError::InvalidCommand { .. })
        ));
        // Wrong joint count is rejected.
        bus.begin_tick(2);
        assert!(matches!(
            bus.send_joint_commands(&cmds[..5]),
            Err(BusError::InvalidCommand { .. })
        ));
        // Gripper slot accepts every variant.
        bus.send_gripper(&GripperCommand::Calibrate).unwrap();
        bus.begin_tick(3);
        bus.send_gripper(&GripperCommand::FirmwarePoll).unwrap();
    }

    use crate::types::Pack;

    #[test]
    fn drain_decodes_and_freshness_warns_then_latches() {
        let (mut bus, robot) = configured_bus();
        let mut state = BusState::new();
        let stale = u64::from(robot.ticks(robot.bus.stale_warn_s)); // 10
        let lost = u64::from(robot.ticks(robot.bus.lost_s)); // 50

        bus.begin_tick(1);
        bus.inject(
            true,
            Reply::Motion {
                node: 0,
                position_ticks: 12345,
                speed_ticks_s: -678,
                current_ma: 90,
            },
        );
        bus.inject(false, Reply::Temperature { node: 0, deg_c: 41 });
        bus.inject(
            false,
            Reply::Errors {
                node: 0,
                flags: ErrorFlags {
                    error: true,
                    current: true,
                    ..ErrorFlags::default()
                },
            },
        );
        bus.inject(
            false,
            Reply::Gripper {
                reply: GripperReply {
                    position: 252,
                    current_ma: 120,
                    activated: true,
                    object_detection: ObjectDetection::DetectedClosing,
                    calibrated: true,
                    ..GripperReply::default()
                },
            },
        );
        let n = bus.drain_rx(&mut state).unwrap();
        assert_eq!(n, 4);
        assert_eq!(state.nodes[0].position_ticks, Some(12345));
        assert_eq!(state.nodes[0].speed_ticks_s, Some(-678));
        assert_eq!(state.nodes[0].current_ma, Some(90));
        assert_eq!(state.nodes[0].temperature_c, Some(41));
        assert!(state.nodes[0].error_flags.unwrap().current);
        // err bit of the LAST frame wins (per-frame live signal).
        assert!(!state.nodes[0].live_error_bit);
        assert_eq!(state.nodes[0].data_age_ticks, 0);
        let g = state.gripper.reply.unwrap();
        assert_eq!(g.position, 252);
        assert_eq!(g.object_detection, ObjectDetection::DetectedClosing);
        assert_eq!(bus.freshness(0), Freshness::Fresh);
        assert_eq!(bus.freshness(1), Freshness::Unknown);

        // Age past the warn threshold: stale (self-clearing warning).
        bus.begin_tick(1 + stale);
        bus.drain_rx(&mut state).unwrap();
        assert_eq!(bus.freshness(0), Freshness::Stale);
        assert_eq!(state.nodes[0].data_age_ticks, stale);

        // A frame while stale clears it and reports the reconnect edge.
        bus.inject(
            false,
            Reply::Motion {
                node: 0,
                position_ticks: 1,
                speed_ticks_s: 0,
                current_ma: 0,
            },
        );
        bus.drain_rx(&mut state).unwrap();
        assert_eq!(state.reconnected_mask, 1 << 0);
        assert_eq!(bus.freshness(0), Freshness::Fresh);

        // Age past the lost threshold: LATCHED.
        bus.begin_tick(1 + stale + lost);
        bus.drain_rx(&mut state).unwrap();
        assert_eq!(bus.freshness(0), Freshness::Lost);
        // Frames resuming do NOT clear the latch...
        bus.inject(
            false,
            Reply::Motion {
                node: 0,
                position_ticks: 2,
                speed_ticks_s: 0,
                current_ma: 0,
            },
        );
        bus.drain_rx(&mut state).unwrap();
        assert_eq!(
            state.reconnected_mask,
            1 << 0,
            "reconnect edge still reported"
        );
        assert_eq!(bus.freshness(0), Freshness::Lost);
        // ...only the user clear path does, and it re-arms the clock at
        // "seen now" so a node that stays silent re-latches on its own.
        bus.clear_lost_latch(0);
        assert_eq!(bus.freshness(0), Freshness::Fresh);
        bus.begin_tick(1 + stale + 2 * lost);
        assert_eq!(
            bus.freshness(0),
            Freshness::Lost,
            "a still-silent node re-latches after the clear"
        );
        bus.clear_lost_latch(0);
        bus.inject(
            false,
            Reply::Motion {
                node: 0,
                position_ticks: 3,
                speed_ticks_s: 0,
                current_ma: 0,
            },
        );
        bus.drain_rx(&mut state).unwrap();
        assert_eq!(bus.freshness(0), Freshness::Fresh);
    }

    #[test]
    fn drain_caps_frames_per_tick() {
        let (mut bus, robot) = configured_bus();
        let cap = robot.bus.rx_frames_per_tick_cap as usize; // 32
        let mut state = BusState::new();
        bus.begin_tick(1);
        for i in 0..(cap + 8) {
            bus.inject(
                false,
                Reply::Motion {
                    node: (i % 6) as NodeId,
                    position_ticks: i as i32,
                    speed_ticks_s: 0,
                    current_ma: 0,
                },
            );
        }
        assert_eq!(bus.drain_rx(&mut state).unwrap(), cap);
        assert_eq!(state.frames_last_drain as usize, cap);
        // The surplus clears on the next tick's drain (backlog recovery).
        bus.begin_tick(2);
        assert_eq!(bus.drain_rx(&mut state).unwrap(), 8);
        assert_eq!(
            state.frame_age_max_ticks, 1,
            "backlogged frames aged one tick"
        );
    }

    #[test]
    fn poll_round_robin_covers_all_nodes_and_override_preempts() {
        let (mut bus, robot) = configured_bus();
        bus.tx_log.clear();
        let total = robot.joints.len() + 1; // 6 joints + gripper
        for t in 0..(3 * total as u64) {
            bus.begin_tick(t);
            bus.poll_step().unwrap();
        }
        // Every node got each of temp/voltage/errors exactly once per
        // 3×total_nodes ticks.
        let mut seen = std::collections::HashMap::new();
        for (_, rec) in &bus.tx_log {
            let TxRecord::Poll { node, kind } = rec else {
                panic!("unexpected record {rec:?}");
            };
            *seen.entry((*node, *kind)).or_insert(0) += 1;
        }
        assert_eq!(seen.len(), 3 * total);
        assert!(seen.values().all(|&c| c == 1));
        let polled_nodes: std::collections::BTreeSet<_> = seen.keys().map(|(n, _)| *n).collect();
        assert!(polled_nodes.contains(&robot.bus.gripper_node));

        // Override preempts for exactly `repeats` steps, then the
        // round-robin resumes.
        bus.tx_log.clear();
        bus.queue_poll_override(PollAction::ClearError { node: 2 }, 3);
        for t in 100..105 {
            bus.begin_tick(t);
            bus.poll_step().unwrap();
        }
        let kinds: Vec<bool> = bus
            .tx_log
            .iter()
            .map(|(_, r)| matches!(r, TxRecord::ClearError { node: 2 }))
            .collect();
        assert_eq!(kinds, vec![true, true, true, false, false]);
    }

    #[test]
    fn silent_mode_is_bus_silent_and_discards_rx() {
        let (mut bus, robot) = configured_bus();
        bus.tx_log.clear();
        bus.begin_tick(1);
        bus.set_silent(true);
        assert!(bus.is_silent());
        // Any send is a contract violation.
        assert!(matches!(
            bus.send_joint_commands(&[JointCommand::idle(); 6]),
            Err(BusError::InvalidCommand { .. })
        ));
        assert!(matches!(
            bus.send_gripper(&GripperCommand::FirmwarePoll),
            Err(BusError::InvalidCommand { .. })
        ));
        // Polls are suppressed silently (tick structure stays uniform).
        bus.poll_step().unwrap();
        assert!(bus.tx_log.is_empty());
        // RX is drained but DISCARDED — bootloader frames alias
        // application ids, nothing may decode.
        let mut state = BusState::new();
        bus.inject(
            false,
            Reply::Motion {
                node: 0,
                position_ticks: 999,
                speed_ticks_s: 0,
                current_ma: 0,
            },
        );
        assert_eq!(bus.drain_rx(&mut state).unwrap(), 1);
        assert_eq!(state.nodes[0].position_ticks, None);
        // Exit: re-base freshness so the silence never reads as disconnect.
        bus.set_silent(false);
        bus.rebase_freshness();
        for n in 0..6 {
            assert_eq!(bus.freshness(n), Freshness::Fresh);
        }
        // "Seen now", not "never seen": a driver that did not survive the
        // flash still latches after the normal lost window.
        let lost = u64::from(robot.ticks(robot.bus.lost_s));
        bus.begin_tick(1 + lost);
        for n in 0..6 {
            assert_eq!(bus.freshness(n), Freshness::Lost);
        }
    }

    #[test]
    fn homing_limit_and_clear_error_hooks_record_repeats() {
        let (mut bus, robot) = configured_bus();
        bus.tx_log.clear();
        bus.begin_tick(1);
        // Homing entry: Limits(normal vel, homing current) ×4 to the joint.
        let vel = robot.joints[0].velocity_limit_ticks_s as f32;
        let cur = robot.homing.joints[0].current_ma as f32;
        bus.send_limits(0, vel, cur, 4).unwrap();
        let limits: Vec<_> = bus
            .tx_log
            .iter()
            .filter(|(_, r)| matches!(r, TxRecord::Limits { node: 0, .. }))
            .collect();
        assert_eq!(limits.len(), 4);
        // Clear sequence: cmd 1 ×3.
        bus.send_clear_error(3, 3).unwrap();
        let clears = bus
            .tx_log
            .iter()
            .filter(|(_, r)| matches!(r, TxRecord::ClearError { node: 3 }))
            .count();
        assert_eq!(clears, 3);
        // Reconnect path re-sends the stored config.
        bus.resend_node_config(2, 2).unwrap();
        let passes = bus
            .tx_log
            .iter()
            .filter(|(_, r)| matches!(r, TxRecord::ConfigPass { node: 2 }))
            .count();
        assert_eq!(passes, 2);
    }
}
