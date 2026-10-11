//! One node's stored driver configuration: what the boot pass, the
//! scheduled re-push shots, a reconnect resend and a live retune all put
//! on the wire. Both backends keep the same struct so a field added to
//! [`DriveTune`] reaches the hardware and the simulator alike.

use par6_config::{Gains, GripperDriverConfig, JointConfig, RippleHarmonic, WatchdogAction};

use crate::spectral::codec::{encode_ripple, encode_velocity_window, CanFrame, RIPPLE_SLOTS};
use crate::types::{DriveTune, NodeId};

/// A drive's ripple slots: (harmonic, cosine mA, sine mA), harmonic 0 unused.
pub(crate) type RippleSlots = [(u8, i16, i16); RIPPLE_SLOTS as usize];

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct NodeConfig {
    pub(crate) node: NodeId,
    pub(crate) watchdog_ms: u32,
    pub(crate) watchdog_action: WatchdogAction,
    pub(crate) velocity_limit_ticks_s: f64,
    pub(crate) ilim_ma: f64,
    pub(crate) voltage_limit_mv: u32,
    pub(crate) gains: Gains,
    pub(crate) ripple: RippleSlots,
    pub(crate) velocity_window: Option<u8>,
}

impl NodeConfig {
    /// An arm joint's driver, as configured.
    pub(crate) fn arm(j: &JointConfig, watchdog_action: WatchdogAction) -> Self {
        Self {
            node: j.node_id,
            watchdog_ms: j.watchdog_timeout_ms,
            watchdog_action,
            velocity_limit_ticks_s: j.velocity_limit_ticks_s,
            ilim_ma: j.ilim_ma,
            voltage_limit_mv: j.voltage_limit_mv,
            gains: j.gains,
            ripple: ripple_slots(&j.ripple),
            velocity_window: j.velocity_window,
        }
    }

    /// The CAN gripper motor's driver, as configured.
    pub(crate) fn gripper(
        node: NodeId,
        d: &GripperDriverConfig,
        watchdog_action: WatchdogAction,
    ) -> Self {
        Self {
            node,
            watchdog_ms: d.watchdog_timeout_ms,
            watchdog_action,
            velocity_limit_ticks_s: d.velocity_limit_ticks_s,
            ilim_ma: d.ilim_ma,
            voltage_limit_mv: d.voltage_limit_mv,
            gains: d.gains,
            ripple: [(0, 0, 0); RIPPLE_SLOTS as usize],
            velocity_window: None,
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
    /// The frames beyond the vendor's configuration this drive is set up with
    /// (par6 firmware): its ripple slots and its speed filter window, none
    /// for a drive configured like the vendor's.
    pub(crate) fn extra_frames(&self) -> impl Iterator<Item = CanFrame> + '_ {
        let ripple = self.has_ripple().then(|| self.ripple_frames());
        let window = self
            .velocity_window
            .map(|w| encode_velocity_window(self.node, w));
        ripple.into_iter().flatten().chain(window)
    }

    /// Whether this drive carries ripple feedforward; one without sends no
    /// ripple frames, so a drive that has never heard of cmd 40 never does.
    pub(crate) fn has_ripple(&self) -> bool {
        self.ripple.iter().any(|(h, _, _)| *h != 0)
    }

    /// Every slot, the unused ones cleared.
    pub(crate) fn ripple_frames(&self) -> impl Iterator<Item = CanFrame> + '_ {
        self.ripple
            .iter()
            .enumerate()
            .map(|(slot, (h, a, b))| encode_ripple(self.node, slot as u8, *h, *a, *b))
    }
}
