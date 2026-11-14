//! One node's stored driver configuration: what the boot pass, the
//! scheduled re-push shots, a reconnect resend and a live retune all put
//! on the wire. Both backends keep the same struct so a field added to
//! [`DriveTune`] reaches the hardware and the simulator alike.

use par6_config::{
    Gains, GripperDriverConfig, JointConfig, RippleHarmonic, SpeedEstimator, WatchdogAction,
    WatchdogAfter, WatchdogFallback,
};

use crate::spectral::codec::{
    encode_param_set, encode_ripple, CanFrame, Param, COGGING_MODE_HARMONICS, RIPPLE_SLOTS,
};
use crate::types::{DriveTune, NodeId};

/// A drive's ripple slots: (harmonic, cosine mA, sine mA), harmonic 0 unused.
pub(crate) type RippleSlots = [(u8, i16, i16); RIPPLE_SLOTS as usize];

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct NodeConfig {
    pub(crate) node: NodeId,
    pub(crate) watchdog_ms: u32,
    pub(crate) watchdog_action: WatchdogAction,
    pub(crate) watchdog_fallback: WatchdogFallback,
    pub(crate) velocity_limit_ticks_s: f64,
    pub(crate) ilim_ma: f64,
    pub(crate) voltage_limit_mv: u32,
    pub(crate) gains: Gains,
    pub(crate) ripple: RippleSlots,
    pub(crate) speed_estimator: Option<SpeedEstimator>,
    pub(crate) speed_pll_bandwidth_hz: Option<u16>,
}

/// The watchdog settings every drive on the bus shares.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Watchdog {
    pub(crate) action: WatchdogAction,
    pub(crate) fallback: WatchdogFallback,
}

impl Watchdog {
    pub(crate) fn of(bus: &par6_config::BusConfig) -> Self {
        Self {
            action: bus.watchdog_action,
            fallback: bus.watchdog_fallback,
        }
    }
}

impl NodeConfig {
    /// An arm joint's driver, as configured.
    pub(crate) fn arm(j: &JointConfig, watchdog: Watchdog) -> Self {
        Self {
            node: j.node_id,
            watchdog_ms: j.watchdog_timeout_ms,
            watchdog_action: watchdog.action,
            watchdog_fallback: watchdog.fallback,
            velocity_limit_ticks_s: j.velocity_limit_ticks_s,
            ilim_ma: j.ilim_ma,
            voltage_limit_mv: j.voltage_limit_mv,
            gains: j.gains,
            ripple: ripple_slots(&j.ripple),
            speed_estimator: j.speed_estimator,
            speed_pll_bandwidth_hz: j.speed_pll_bandwidth_hz,
        }
    }

    /// The CAN gripper motor's driver, as configured.
    pub(crate) fn gripper(node: NodeId, d: &GripperDriverConfig, watchdog: Watchdog) -> Self {
        Self {
            node,
            watchdog_ms: d.watchdog_timeout_ms,
            watchdog_action: watchdog.action,
            watchdog_fallback: watchdog.fallback,
            velocity_limit_ticks_s: d.velocity_limit_ticks_s,
            ilim_ma: d.ilim_ma,
            voltage_limit_mv: d.voltage_limit_mv,
            gains: d.gains,
            ripple: [(0, 0, 0); RIPPLE_SLOTS as usize],
            speed_estimator: None,
            speed_pll_bandwidth_hz: None,
        }
    }

    /// Replace what `SET_PID_GAINS` retunes; the watchdog settings are
    /// deliberately untouched.
    pub(crate) fn apply_tune(&mut self, tune: &DriveTune) {
        self.gains = tune.gains;
        self.ilim_ma = tune.ilim_ma;
        self.velocity_limit_ticks_s = tune.velocity_limit_ticks_s;
        self.voltage_limit_mv = tune.voltage_limit_mv;
    }
}

/// Harmonics into slots, the rest cleared (config validation caps the count).
pub(crate) fn ripple_slots(harmonics: &[RippleHarmonic]) -> RippleSlots {
    let mut slots = [(0, 0, 0); RIPPLE_SLOTS as usize];
    for (slot, r) in slots.iter_mut().zip(harmonics) {
        *slot = (r.harmonic, r.a_ma, r.b_ma);
    }
    slots
}

impl NodeConfig {
    /// The parameter frames this drive is set up with beyond the seven
    /// configuration frames (firmware 109): how a watchdog hold or descend
    /// runs, its speed estimate where the config names one, and its ripple
    /// slots where it carries any.
    pub(crate) fn extra_frames(&self) -> impl Iterator<Item = CanFrame> + '_ {
        let set = |param, value| encode_param_set(self.node, param, value);
        let f = self.watchdog_fallback;
        let fallback = [
            set(
                Param::WatchdogFallbackMs,
                (f.time_s * 1000.0).round() as i32,
            ),
            set(Param::WatchdogFallbackCurrentMa, i32::from(f.current_ma)),
            set(Param::WatchdogFallbackSpeed, i32::from(f.speed_ticks_s)),
            set(
                Param::WatchdogAfter,
                match f.after {
                    WatchdogAfter::Idle => 0,
                    WatchdogAfter::Brake => 2,
                    WatchdogAfter::Coast => 3,
                },
            ),
        ];
        let estimator = self.speed_estimator.map(|e| {
            set(
                Param::SpeedEstimator,
                match e {
                    SpeedEstimator::Average => 0,
                    SpeedEstimator::Pll => 1,
                    SpeedEstimator::PllDirect => 2,
                },
            )
        });
        let bandwidth = self
            .speed_pll_bandwidth_hz
            .map(|hz| set(Param::SpeedPllBandwidthHz, i32::from(hz)));
        let ripple = self.has_ripple().then(|| self.ripple_frames());
        fallback
            .into_iter()
            .chain(estimator)
            .chain(bandwidth)
            .chain(ripple.into_iter().flatten())
    }

    /// Whether this drive carries ripple feedforward; one without sends no
    /// ripple frames, so its anti-cogging stays as the drive has it saved.
    pub(crate) fn has_ripple(&self) -> bool {
        self.ripple.iter().any(|(h, _, _)| *h != 0)
    }

    pub(crate) fn ripple_frames(&self) -> impl Iterator<Item = CanFrame> + '_ {
        ripple_frames(self.node, &self.ripple)
    }
}

/// Every slot, the unused ones cleared, then the switch that applies them:
/// on with any harmonic set, off with none.
pub(crate) fn ripple_frames(
    node: NodeId,
    slots: &RippleSlots,
) -> impl Iterator<Item = CanFrame> + '_ {
    let mode = if slots.iter().any(|(h, _, _)| *h != 0) {
        COGGING_MODE_HARMONICS
    } else {
        0
    };
    slots
        .iter()
        .enumerate()
        .map(move |(slot, (h, a, b))| encode_ripple(node, slot as u8, *h, *a, *b))
        .chain(std::iter::once(encode_param_set(
            node,
            Param::CoggingMode,
            mode,
        )))
}
