//! One node's stored driver configuration: what the boot pass, the
//! scheduled re-push shots, a reconnect resend and a live retune all put
//! on the wire. Both backends keep the same struct so a field added to
//! [`DriveTune`] reaches the hardware and the simulator alike.

use par6_config::{Gains, GripperDriverConfig, JointConfig, WatchdogAction};

use crate::types::{DriveTune, NodeId};

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct NodeConfig {
    pub(crate) node: NodeId,
    pub(crate) watchdog_ms: u32,
    pub(crate) watchdog_action: WatchdogAction,
    pub(crate) velocity_limit_ticks_s: f64,
    pub(crate) ilim_ma: f64,
    pub(crate) voltage_limit_mv: u32,
    pub(crate) gains: Gains,
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
