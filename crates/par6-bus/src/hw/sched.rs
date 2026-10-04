//! The socket-free half of the SocketCAN backend: the round-robin
//! telemetry schedule, the per-node freshness clock, and the paced boot
//! configuration plan.
//!
//! Nothing here touches a file descriptor, so the schedulers that decide
//! WHAT goes on the wire are exercised directly by unit tests, while the
//! hardware module only has to get the transport right.
//!
//! [`FreshnessClock`] is the exception to "SocketCAN backend": data age
//! and its warn/latch/re-arm rules are bus health semantics, not
//! transport, so [`crate::sim::SimBus`] and [`crate::LoopbackBus`] run
//! the same clock rather than each carrying a copy of it.

use crate::node_config::NodeConfig;
use crate::spectral::codec::{
    encode_current_gains, encode_limits, encode_pd_gains, encode_position_gains,
    encode_velocity_gains, encode_voltage_limit, encode_watchdog, CanFrame,
};
use crate::types::{Freshness, NodeId, PollAction, PollKind, MAX_NODES};

/// Poll slots between device-info sweeps.
///
/// One slot goes out per RT tick, so the sweep period is
/// `DEVICE_INFO_PERIOD_SLOTS · dt` — ~4 s at the shipped 250 Hz and
/// proportionally longer at a slower one. Left a count deliberately:
/// this is identity/telemetry refresh cadence with no deadline riding
/// on it, unlike the freshness windows above.
///
/// Shared with [`crate::sim`], which schedules its polls on the same
/// rhythm — the same reason [`FreshnessClock`] lives here rather than in
/// each backend.
pub const DEVICE_INFO_PERIOD_SLOTS: u64 = 1006;

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

/// Round-robin telemetry schedule: each target gets one combined
/// telemetry poll per cycle, or temperature / voltage / errors in three
/// slots when it runs a firmware without the combined reply; a
/// device-info sweep replaces the round robin for `targets` slots every
/// [`DEVICE_INFO_PERIOD_SLOTS`], and a single-slot override queue
/// preempts everything.
///
/// One slot per RT tick keeps the steady-state TX budget at joints +
/// gripper + 1 — inside the classic-CAN ceiling.
#[derive(Debug, Default)]
pub(super) struct PollScheduler {
    targets: usize,
    /// Targets polled the vendor way, three kinds a cycle.
    legacy: Vec<bool>,
    /// One cycle of the round robin, rebuilt when a target's way changes.
    cycle: Vec<(usize, PollKind)>,
    cursor: u64,
    slot: u64,
    device_info_remaining: usize,
    override_slot: Option<(PollAction, u16)>,
}

impl PollScheduler {
    /// Re-arm for `targets` poll targets (boot configuration), every one
    /// on the combined poll until [`set_legacy`](Self::set_legacy) says
    /// otherwise.
    pub(super) fn configure(&mut self, targets: usize) {
        self.targets = targets;
        self.legacy = vec![false; targets];
        self.cursor = 0;
        self.slot = 0;
        self.device_info_remaining = 0;
        self.override_slot = None;
        self.rebuild_cycle();
    }

    /// Poll `target` the vendor way (three kinds a cycle) or the
    /// combined way; a target's answer to the boot probe decides.
    pub(super) fn set_legacy(&mut self, target: usize, legacy: bool) {
        if target < self.targets && self.legacy[target] != legacy {
            self.legacy[target] = legacy;
            self.rebuild_cycle();
        }
    }

    fn rebuild_cycle(&mut self) {
        self.cycle.clear();
        for (target, &legacy) in self.legacy.iter().enumerate() {
            if legacy {
                self.cycle.extend([
                    (target, PollKind::Temperature),
                    (target, PollKind::Voltage),
                    (target, PollKind::Errors),
                ]);
            } else {
                self.cycle.push((target, PollKind::Telemetry));
            }
        }
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
        let (target, kind) = self.cycle[self.cursor as usize % self.cycle.len()];
        self.cursor += 1;
        Some(PollStep::Poll { target, kind })
    }
}

/// Per-node data-age clock: stale is a self-clearing warning, lost
/// LATCHES until the user clear path.
///
/// `None` means "never seen", which only [`configure`](Self::configure)
/// produces: it is an absorbing state ([`latch_lost`](Self::latch_lost)
/// skips it and [`classify`](Self::classify) maps it to
/// [`Freshness::Unknown`]), so nothing that means "forget the fault" may
/// ever write it — see [`clear_latch`](Self::clear_latch).
#[derive(Debug)]
pub(crate) struct FreshnessClock {
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
    /// caller) and forget every observation. Boot is the one moment where
    /// "never seen" is the truth — the bus scan and the RT boot selfcheck
    /// are what catch a node that never appears at all.
    ///
    /// Both thresholds floor at one tick. `classify` and `latch_lost`
    /// test `age >= threshold` and `mark` tests the same for the
    /// stale→fresh edge, so a zero would read every node stale at age
    /// zero, make every frame a reconnect (a stored-config resend per
    /// node per tick), and latch `CAN_LOST` on the tick after a node's
    /// first frame. Config validation rejects a window shorter than the
    /// tick; this is the floor that keeps a rounding result from
    /// reintroducing it in any backend.
    pub(crate) fn configure(&mut self, stale_warn_ticks: u64, lost_ticks: u64) {
        self.stale_warn_ticks = stale_warn_ticks.max(1);
        self.lost_ticks = lost_ticks.max(1);
        self.last_rx_tick = [None; MAX_NODES];
        self.lost_latched = [false; MAX_NODES];
        self.last_gripper_rx_tick = None;
    }

    /// Latch every node whose age has reached the lost threshold. Called
    /// once per tick, before the drain.
    pub(crate) fn latch_lost(&mut self, tick: u64) {
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
    /// A node's FIRST-ever frame counts as an edge only when the boot
    /// scan never saw it (`booted == false`): one that boots after the
    /// last scheduled config shot has missed every push it will get,
    /// and this sighting is the only signal left to configure it. A
    /// node the paced boot pass already configured needs no extra shot
    /// for merely answering.
    pub(crate) fn mark(&mut self, node: NodeId, tick: u64, booted: bool) -> bool {
        let n = usize::from(node);
        let reconnected = match self.last_rx_tick[n] {
            None => !booted,
            Some(last) => tick.saturating_sub(last) >= self.stale_warn_ticks,
        };
        self.last_rx_tick[n] = Some(tick);
        reconnected
    }

    /// Record a firmware-gripper reply (cmd 60), which ages separately
    /// from the node's other traffic.
    pub(crate) fn mark_gripper(&mut self, tick: u64) {
        self.last_gripper_rx_tick = Some(tick);
    }

    /// Ticks since `node`'s last frame; `u64::MAX` = never seen.
    pub(crate) fn age(&self, node: NodeId, tick: u64) -> u64 {
        match self.last_rx_tick[usize::from(node)] {
            Some(last) => tick.saturating_sub(last),
            None => u64::MAX,
        }
    }

    /// Ticks since the last firmware-gripper reply.
    pub(crate) fn gripper_age(&self, tick: u64) -> u64 {
        match self.last_gripper_rx_tick {
            Some(last) => tick.saturating_sub(last),
            None => u64::MAX,
        }
    }

    /// Freshness classification of one node at `tick`.
    pub(crate) fn classify(&self, node: NodeId, tick: u64) -> Freshness {
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

    /// User clear-errors path for one node: drop the latch and stamp the
    /// node SEEN NOW.
    ///
    /// "Forget" must mean "seen now", never "never seen". Zeroing the
    /// observation would make a node that never speaks again permanently
    /// un-reportable and would cost it the stale→fresh edge that resends
    /// its stored config; stamping the current tick re-arms both — a
    /// still-silent node re-latches `lost_ticks` later on its own, and one
    /// that comes back is a reconnect.
    pub(crate) fn clear_latch(&mut self, node: NodeId, tick: u64) {
        let n = usize::from(node);
        self.lost_latched[n] = false;
        self.last_rx_tick[n] = Some(tick);
    }

    /// Stamp every node SEEN NOW and drop every latch (FLASHING exit):
    /// the deliberately silent window must not read as a mass disconnect,
    /// while a node that did not survive the flash still latches
    /// `lost_ticks` later.
    pub(crate) fn rebase(&mut self, tick: u64) {
        self.last_rx_tick = [Some(tick); MAX_NODES];
        self.lost_latched = [false; MAX_NODES];
        self.last_gripper_rx_tick = Some(tick);
    }
}

/// The seven configuration message types, in boot order.
/// One pass = these seven frames to one node; one paced batch = one
/// message type to every node.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigKind {
    Watchdog,
    Limits,
    VoltageLimit,
    PdGains,
    CurrentGains,
    VelocityGains,
    PositionGains,
}

impl ConfigKind {
    /// All configuration fields in the drive's boot order.
    pub const ALL: [Self; 7] = CONFIG_ORDER;

    /// This kind's position in [`Self::ALL`].
    pub fn index(self) -> usize {
        CONFIG_ORDER
            .iter()
            .position(|k| *k == self)
            .expect("every kind is in CONFIG_ORDER")
    }
}

/// The order the boot config load sends message types in.
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
pub(crate) fn config_frame(kind: ConfigKind, c: &NodeConfig) -> CanFrame {
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
    /// microseconds against a ~10 frames/ms drain.
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
        let extra: Vec<CanFrame> = configs.iter().flat_map(NodeConfig::extra_frames).collect();
        if !extra.is_empty() {
            out.extend(extra.into_iter().map(BootStep::Frame));
            out.push(BootStep::Pace);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spectral::codec::{unpack_can_id, CommandId};
    use par6_config::{Gains, WatchdogAction};

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
            ripple: [(0, 0, 0); 8],
            velocity_window: None,
        }
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

    /// Config frames carry each joint's configured values, every kind of
    /// them, so a reconnect resend restores exactly what boot installed.
    #[test]
    fn config_frames_carry_the_stored_values() {
        let path =
            std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
        let robot = par6_config::RobotConfig::load(&path).expect("PAR6.toml");
        let be = |v: f64| (v as f32).to_be_bytes();
        let pair = |a: f64, b: f64| [be(a), be(b)].concat();
        for j in &robot.joints {
            let c = NodeConfig::arm(j, WatchdogAction::Idle);
            let payload = |kind| config_frame(kind, &c).payload().to_vec();
            let g = &j.gains;
            assert_eq!(
                payload(ConfigKind::Watchdog),
                [j.watchdog_timeout_ms.to_be_bytes().as_slice(), &[0]].concat(),
                "{}: watchdog ms then the Idle action",
                j.name
            );
            assert_eq!(
                payload(ConfigKind::Limits),
                pair(j.velocity_limit_ticks_s, j.ilim_ma),
                "{}",
                j.name
            );
            assert_eq!(
                payload(ConfigKind::VoltageLimit),
                j.voltage_limit_mv.to_be_bytes(),
                "{}",
                j.name
            );
            assert_eq!(payload(ConfigKind::PdGains), pair(g.kp, g.kd), "{}", j.name);
            assert_eq!(
                payload(ConfigKind::CurrentGains),
                pair(g.kpiq, g.kiiq),
                "{}",
                j.name
            );
            assert_eq!(
                payload(ConfigKind::VelocityGains),
                pair(g.kpv, g.kiv),
                "{}",
                j.name
            );
            assert_eq!(payload(ConfigKind::PositionGains), be(g.kpp), "{}", j.name);
        }
    }
}
