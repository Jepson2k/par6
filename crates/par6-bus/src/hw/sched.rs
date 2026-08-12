//! The socket-free half of the SocketCAN backend: the round-robin
//! telemetry schedule, the per-node freshness clock, and the paced boot
//! configuration plan.
//!
//! Nothing here touches a file descriptor, so the schedulers that decide
//! WHAT goes on the wire are exercised directly by unit tests, while the
//! hardware module only has to get the transport right.

use par6_config::{Gains, WatchdogAction};

use crate::spectral::codec::{
    encode_current_gains, encode_limits, encode_pd_gains, encode_position_gains,
    encode_velocity_gains, encode_voltage_limit, encode_watchdog, CanFrame,
};
use crate::types::{Freshness, NodeId, PollAction, PollKind, MAX_NODES};

/// Poll slots between device-info sweeps (~4 s at 250 Hz, spec/CAN.md).
pub(super) const DEVICE_INFO_PERIOD_SLOTS: u64 = 1006;

/// What one poll slot resolves to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum PollStep {
    /// The single-slot override queue preempted the round robin.
    Override(PollAction),
    /// The round-robin (or device-info sweep) request, by target index.
    Poll {
        /// Index into the backend's poll-target list (joints, then gripper).
        target: usize,
        /// Telemetry request kind.
        kind: PollKind,
    },
}

/// Round-robin telemetry schedule: each target gets temperature /
/// voltage / errors once every `3 × targets` slots, a device-info sweep
/// replaces the round robin for `targets` slots every
/// [`DEVICE_INFO_PERIOD_SLOTS`], and a single-slot override queue
/// preempts everything (spec/CAN.md per-tick pattern).
///
/// One slot per RT tick keeps the steady-state TX budget at joints +
/// gripper + 1 — inside the classic-CAN ceiling.
#[derive(Debug, Default)]
pub(super) struct PollScheduler {
    targets: usize,
    cursor: u64,
    slot: u64,
    device_info_remaining: usize,
    override_slot: Option<(PollAction, u16)>,
}

impl PollScheduler {
    /// Re-arm for `targets` poll targets (boot configuration).
    pub(super) fn configure(&mut self, targets: usize) {
        self.targets = targets;
        self.cursor = 0;
        self.slot = 0;
        self.device_info_remaining = 0;
        self.override_slot = None;
    }

    /// Queue an override; it preempts the round robin for `repeats`
    /// slots. The slot is single: a pending override is REPLACED.
    pub(super) fn queue_override(&mut self, action: PollAction, repeats: u16) {
        if repeats == 0 {
            return;
        }
        self.override_slot = Some((action, repeats));
    }

    /// Resolve this tick's poll slot. `None` before configuration.
    pub(super) fn step(&mut self) -> Option<PollStep> {
        if self.targets == 0 {
            return None;
        }
        if let Some((action, repeats)) = self.override_slot.take() {
            if repeats > 1 {
                self.override_slot = Some((action, repeats - 1));
            }
            return Some(PollStep::Override(action));
        }
        self.slot += 1;
        if self.device_info_remaining > 0 {
            let target = self.targets - self.device_info_remaining;
            self.device_info_remaining -= 1;
            return Some(PollStep::Poll {
                target,
                kind: PollKind::DeviceInfo,
            });
        }
        if self.slot.is_multiple_of(DEVICE_INFO_PERIOD_SLOTS) {
            self.device_info_remaining = self.targets;
        }
        let target = (self.cursor / 3) as usize % self.targets;
        let kind = match self.cursor % 3 {
            0 => PollKind::Temperature,
            1 => PollKind::Voltage,
            _ => PollKind::Errors,
        };
        self.cursor += 1;
        Some(PollStep::Poll { target, kind })
    }
}

/// Per-node data-age clock (spec/CAN.md freshness layer 1): stale is a
/// self-clearing warning, lost LATCHES until the user clear path.
#[derive(Debug)]
pub(super) struct FreshnessClock {
    stale_warn_ticks: u64,
    lost_ticks: u64,
    last_rx_tick: [Option<u64>; MAX_NODES],
    lost_latched: [bool; MAX_NODES],
    last_gripper_rx_tick: Option<u64>,
}

impl Default for FreshnessClock {
    fn default() -> Self {
        Self {
            stale_warn_ticks: u64::MAX,
            lost_ticks: u64::MAX,
            last_rx_tick: [None; MAX_NODES],
            lost_latched: [false; MAX_NODES],
            last_gripper_rx_tick: None,
        }
    }
}

impl FreshnessClock {
    /// Install the thresholds (config seconds converted to ticks by the
    /// caller) and forget every observation.
    pub(super) fn configure(&mut self, stale_warn_ticks: u64, lost_ticks: u64) {
        self.stale_warn_ticks = stale_warn_ticks;
        self.lost_ticks = lost_ticks;
        self.rebase();
    }

    /// Latch every node whose age has reached the lost threshold. Called
    /// once per tick, before the drain.
    pub(super) fn latch_lost(&mut self, tick: u64) {
        for n in 0..MAX_NODES {
            if let Some(last) = self.last_rx_tick[n] {
                if tick.saturating_sub(last) >= self.lost_ticks {
                    self.lost_latched[n] = true;
                }
            }
        }
    }

    /// Record a frame from `node`. Returns `true` when it is a
    /// stale→fresh edge (the reconnect signal that re-sends config).
    pub(super) fn mark(&mut self, node: NodeId, tick: u64) -> bool {
        let n = usize::from(node);
        let reconnected = self.last_rx_tick[n]
            .is_some_and(|last| tick.saturating_sub(last) >= self.stale_warn_ticks);
        self.last_rx_tick[n] = Some(tick);
        reconnected
    }

    /// Record a firmware-gripper reply (cmd 60), which ages separately
    /// from the node's other traffic.
    pub(super) fn mark_gripper(&mut self, tick: u64) {
        self.last_gripper_rx_tick = Some(tick);
    }

    /// Ticks since `node`'s last frame; `u64::MAX` = never seen.
    pub(super) fn age(&self, node: NodeId, tick: u64) -> u64 {
        match self.last_rx_tick[usize::from(node)] {
            Some(last) => tick.saturating_sub(last),
            None => u64::MAX,
        }
    }

    /// Ticks since the last firmware-gripper reply.
    pub(super) fn gripper_age(&self, tick: u64) -> u64 {
        match self.last_gripper_rx_tick {
            Some(last) => tick.saturating_sub(last),
            None => u64::MAX,
        }
    }

    /// Freshness classification of one node at `tick`.
    pub(super) fn classify(&self, node: NodeId, tick: u64) -> Freshness {
        let n = usize::from(node);
        if self.lost_latched[n] {
            return Freshness::Lost;
        }
        match self.last_rx_tick[n] {
            None => Freshness::Unknown,
            Some(last) => {
                let age = tick.saturating_sub(last);
                if age >= self.lost_ticks {
                    Freshness::Lost
                } else if age >= self.stale_warn_ticks {
                    Freshness::Stale
                } else {
                    Freshness::Fresh
                }
            }
        }
    }

    /// User clear-errors path for one node.
    pub(super) fn clear_latch(&mut self, node: NodeId) {
        let n = usize::from(node);
        self.lost_latched[n] = false;
        self.last_rx_tick[n] = None;
    }

    /// Forget every observation (FLASHING exit).
    pub(super) fn rebase(&mut self) {
        self.last_rx_tick = [None; MAX_NODES];
        self.lost_latched = [false; MAX_NODES];
        self.last_gripper_rx_tick = None;
    }
}

/// One node's stored driver configuration — what the boot passes and the
/// reconnect resends put on the wire.
#[derive(Debug, Clone, Copy)]
pub(super) struct NodeConfig {
    pub(super) node: NodeId,
    pub(super) watchdog_ms: u32,
    pub(super) watchdog_action: WatchdogAction,
    pub(super) velocity_limit_ticks_s: f64,
    pub(super) ilim_ma: f64,
    pub(super) voltage_limit_mv: u32,
    pub(super) gains: Gains,
}

/// The seven configuration message types, in spec/CAN.md boot order.
/// One pass = these seven frames to one node; one paced batch = one
/// message type to every node.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum ConfigKind {
    Watchdog,
    Limits,
    VoltageLimit,
    PdGains,
    CurrentGains,
    VelocityGains,
    PositionGains,
}

/// Boot order (spec/CAN.md step 2).
pub(super) const CONFIG_ORDER: [ConfigKind; 7] = [
    ConfigKind::Watchdog,
    ConfigKind::Limits,
    ConfigKind::VoltageLimit,
    ConfigKind::PdGains,
    ConfigKind::CurrentGains,
    ConfigKind::VelocityGains,
    ConfigKind::PositionGains,
];

/// Encode one configuration frame.
pub(super) fn config_frame(kind: ConfigKind, c: &NodeConfig) -> CanFrame {
    let node = c.node;
    match kind {
        ConfigKind::Watchdog => encode_watchdog(node, c.watchdog_ms, c.watchdog_action),
        ConfigKind::Limits => {
            encode_limits(node, c.velocity_limit_ticks_s as f32, c.ilim_ma as f32)
        }
        ConfigKind::VoltageLimit => encode_voltage_limit(node, c.voltage_limit_mv),
        ConfigKind::PdGains => encode_pd_gains(node, c.gains.kp as f32, c.gains.kd as f32),
        ConfigKind::CurrentGains => {
            encode_current_gains(node, c.gains.kpiq as f32, c.gains.kiiq as f32)
        }
        ConfigKind::VelocityGains => {
            encode_velocity_gains(node, c.gains.kpv as f32, c.gains.kiv as f32)
        }
        ConfigKind::PositionGains => encode_position_gains(node, c.gains.kpp as f32),
    }
}

/// One step of the paced boot configuration load.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum BootStep {
    /// Put this frame on the wire.
    Frame(CanFrame),
    /// Wait `bus.config_pace_s` before the next batch — the interface TX
    /// queue drops silently on overflow, and the whole load enqueues in
    /// microseconds against a ~10 frames/ms drain (spec/CAN.md).
    Pace,
}

/// Build the boot configuration load: `repeats` passes, each pass one
/// paced batch per message type, each batch one frame per node in
/// configuration order.
///
/// Boot-time only — it allocates into `out` (which the caller reuses).
pub(super) fn boot_config_plan(configs: &[NodeConfig], repeats: u8, out: &mut Vec<BootStep>) {
    out.clear();
    if configs.is_empty() {
        return;
    }
    for _pass in 0..repeats {
        for kind in CONFIG_ORDER {
            for c in configs {
                out.push(BootStep::Frame(config_frame(kind, c)));
            }
            out.push(BootStep::Pace);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spectral::codec::{unpack_can_id, CommandId};
    use std::collections::{BTreeSet, HashMap};

    fn node_config(node: NodeId) -> NodeConfig {
        NodeConfig {
            node,
            watchdog_ms: 5000,
            watchdog_action: WatchdogAction::Idle,
            velocity_limit_ticks_s: 80000.0,
            ilim_ma: 1200.0,
            voltage_limit_mv: 6000,
            gains: Gains {
                kp: 1.0,
                kd: 2.0,
                kpiq: 3.0,
                kiiq: 4.0,
                kpv: 5.0,
                kiv: 6.0,
                kpp: 7.0,
            },
        }
    }

    /// Every target must get temperature, voltage and errors exactly once
    /// per `3 × targets` slots — that cadence is what bounds the poll to
    /// ONE frame per tick while still refreshing the ~84 ms telemetry.
    #[test]
    fn round_robin_covers_every_target_once_per_cycle() {
        let targets = 7;
        let mut s = PollScheduler::default();
        s.configure(targets);
        let mut seen: HashMap<(usize, PollKind), u32> = HashMap::new();
        for _ in 0..(3 * targets) {
            match s.step().expect("configured") {
                PollStep::Poll { target, kind } => *seen.entry((target, kind)).or_default() += 1,
                other => panic!("unexpected {other:?}"),
            }
        }
        assert_eq!(seen.len(), 3 * targets);
        assert!(seen.values().all(|&c| c == 1));
        let covered: BTreeSet<usize> = seen.keys().map(|(t, _)| *t).collect();
        assert_eq!(covered, (0..targets).collect::<BTreeSet<_>>());
    }

    /// The device-info sweep replaces the round robin for exactly one
    /// slot per target, then the round robin resumes where it left off —
    /// the sweep must never cost more than one frame in any tick.
    #[test]
    fn device_info_sweep_replaces_one_cycle_and_resumes() {
        let targets = 7;
        let mut s = PollScheduler::default();
        s.configure(targets);
        let mut kinds = Vec::new();
        // Run past the first sweep boundary.
        for _ in 0..(DEVICE_INFO_PERIOD_SLOTS + targets as u64 + 3) {
            let PollStep::Poll { target, kind } = s.step().expect("configured") else {
                panic!("no override queued");
            };
            kinds.push((target, kind));
        }
        let sweep: Vec<usize> = kinds
            .iter()
            .enumerate()
            .filter(|(_, (_, k))| *k == PollKind::DeviceInfo)
            .map(|(i, _)| i)
            .collect();
        assert_eq!(sweep.len(), targets, "one device-info frame per target");
        // Contiguous, and immediately after the boundary slot.
        assert_eq!(sweep[0], DEVICE_INFO_PERIOD_SLOTS as usize);
        assert!(sweep.windows(2).all(|w| w[1] == w[0] + 1));
        let swept: BTreeSet<usize> = sweep.iter().map(|i| kinds[*i].0).collect();
        assert_eq!(swept, (0..targets).collect::<BTreeSet<_>>());
        // The round robin picks up its own cursor, not the sweep's.
        let before = kinds[DEVICE_INFO_PERIOD_SLOTS as usize - 1];
        let after = kinds[DEVICE_INFO_PERIOD_SLOTS as usize + targets];
        assert_eq!(
            after,
            match before.1 {
                PollKind::Temperature => (before.0, PollKind::Voltage),
                PollKind::Voltage => (before.0, PollKind::Errors),
                _ => ((before.0 + 1) % targets, PollKind::Temperature),
            }
        );
    }

    /// An override owns the slot for exactly `repeats` steps, a later
    /// override replaces an unfinished one (single slot), and the round
    /// robin never loses a target across the interruption.
    #[test]
    fn override_preempts_for_its_repeats_then_yields() {
        let mut s = PollScheduler::default();
        s.configure(7);
        s.step();
        s.queue_override(PollAction::ClearError { node: 2 }, 3);
        let mut steps = Vec::new();
        for _ in 0..5 {
            steps.push(s.step().expect("configured"));
        }
        assert_eq!(
            steps
                .iter()
                .filter(|s| matches!(s, PollStep::Override(PollAction::ClearError { node: 2 })))
                .count(),
            3
        );
        assert!(matches!(steps[3], PollStep::Poll { .. }));
        // Second target's temperature: slot 1 of the round robin (slot 0
        // was consumed before the override).
        assert_eq!(
            steps[3],
            PollStep::Poll {
                target: 0,
                kind: PollKind::Voltage
            },
            "the round robin resumes at its own cursor"
        );
        // Replacement: the pending 2 remaining repeats are dropped.
        s.queue_override(PollAction::ClearError { node: 4 }, 2);
        s.queue_override(
            PollAction::Poll {
                node: 1,
                kind: PollKind::Kt,
            },
            1,
        );
        assert_eq!(
            s.step(),
            Some(PollStep::Override(PollAction::Poll {
                node: 1,
                kind: PollKind::Kt
            }))
        );
        assert!(matches!(s.step(), Some(PollStep::Poll { .. })));
    }

    /// The three-layer freshness contract: warn at the stale threshold
    /// (self-clearing), latch at the lost threshold (survives resumed
    /// traffic), reconnect edge reported on stale→fresh.
    #[test]
    fn freshness_warns_then_latches_and_reports_reconnect_edges() {
        let (stale, lost) = (10u64, 50u64);
        let mut f = FreshnessClock::default();
        f.configure(stale, lost);
        assert_eq!(f.classify(0, 0), Freshness::Unknown);
        assert_eq!(f.age(0, 0), u64::MAX);

        assert!(!f.mark(0, 1), "first frame is not a reconnect");
        assert_eq!(f.classify(0, 1), Freshness::Fresh);
        assert_eq!(f.classify(0, 1 + stale - 1), Freshness::Fresh);
        assert_eq!(f.classify(0, 1 + stale), Freshness::Stale);
        assert_eq!(f.age(0, 1 + stale), stale);

        // A frame while stale clears the warning and reports the edge.
        assert!(f.mark(0, 1 + stale));
        assert_eq!(f.classify(0, 1 + stale), Freshness::Fresh);

        // Reaching the lost threshold latches, and traffic does NOT clear it.
        let t = 1 + stale + lost;
        f.latch_lost(t);
        assert_eq!(f.classify(0, t), Freshness::Lost);
        f.mark(0, t);
        assert_eq!(f.classify(0, t), Freshness::Lost, "lost is latched");
        // Only the user clear path resets it.
        f.clear_latch(0);
        assert_eq!(f.classify(0, t), Freshness::Unknown);
        f.mark(0, t);
        assert_eq!(f.classify(0, t), Freshness::Fresh);

        // A node that was never seen never latches, however long we run.
        f.latch_lost(t + 10 * lost);
        assert_eq!(f.classify(3, t + 10 * lost), Freshness::Unknown);

        // Re-base (FLASHING exit) forgets everything, latch included.
        f.mark(1, t);
        f.latch_lost(t + lost);
        assert_eq!(f.classify(1, t + lost), Freshness::Lost);
        f.rebase();
        assert_eq!(f.classify(1, t + lost), Freshness::Unknown);
        assert_eq!(f.gripper_age(t), u64::MAX);
        f.mark_gripper(t);
        assert_eq!(f.gripper_age(t + 4), 4);
    }

    /// The boot load is batched BY MESSAGE TYPE with a pace between
    /// batches (not one long burst per node): that is what keeps the
    /// ~170-frame load from overrunning the interface TX queue.
    #[test]
    fn boot_plan_is_paced_per_message_type_batch_in_spec_order() {
        let configs: Vec<NodeConfig> = (0..7).map(node_config).collect();
        let mut plan = Vec::new();
        boot_config_plan(&configs, 3, &mut plan);

        let paces = plan.iter().filter(|s| **s == BootStep::Pace).count();
        let frames = plan.len() - paces;
        assert_eq!(frames, 3 * 7 * 7, "repeats × message types × nodes");
        assert_eq!(paces, 3 * 7, "one pace per message-type batch");

        // Each batch: one frame per node, same command, nodes in order.
        let want = [
            CommandId::Watchdog,
            CommandId::Limits,
            CommandId::VoltageLimit,
            CommandId::PdGains,
            CommandId::CurrentGains,
            CommandId::VelocityGains,
            CommandId::PositionGains,
        ];
        let mut batch = 0usize;
        let mut in_batch: Vec<(NodeId, u8)> = Vec::new();
        for step in &plan {
            match step {
                BootStep::Frame(f) => {
                    let (node, cmd, err) = unpack_can_id(f.id);
                    assert!(!err, "host frames never set the err bit");
                    in_batch.push((node, cmd));
                }
                BootStep::Pace => {
                    let expect = want[batch % want.len()];
                    assert_eq!(
                        in_batch,
                        (0..7).map(|n| (n, expect.raw())).collect::<Vec<_>>(),
                        "batch {batch} must be {expect:?} to every node in order"
                    );
                    in_batch.clear();
                    batch += 1;
                }
            }
        }
        assert_eq!(batch, paces);

        // No nodes configured: nothing to send (and no stray pacing).
        boot_config_plan(&[], 3, &mut plan);
        assert!(plan.is_empty());
    }

    /// Config frames carry the stored values, so a reconnect resend
    /// restores exactly what boot installed.
    #[test]
    fn config_frames_carry_the_stored_values() {
        let c = node_config(4);
        let wd = config_frame(ConfigKind::Watchdog, &c);
        assert_eq!(wd.payload(), &[0, 0, 0x13, 0x88, 0]); // 5000 ms BE + Idle
        let lim = config_frame(ConfigKind::Limits, &c);
        assert_eq!(&lim.payload()[0..4], &80000f32.to_be_bytes());
        assert_eq!(&lim.payload()[4..8], &1200f32.to_be_bytes());
        let vl = config_frame(ConfigKind::VoltageLimit, &c);
        assert_eq!(vl.payload(), &6000u32.to_be_bytes());
        let pd = config_frame(ConfigKind::PdGains, &c);
        assert_eq!(&pd.payload()[0..4], &1f32.to_be_bytes());
        assert_eq!(&pd.payload()[4..8], &2f32.to_be_bytes());
        let pos = config_frame(ConfigKind::PositionGains, &c);
        assert_eq!(pos.payload(), &7f32.to_be_bytes());
    }
}
