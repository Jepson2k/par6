//! Runtime backend selection: the one concrete [`DriverBus`] the daemon
//! instantiates, so `--sim` and hardware share a single monomorphized RT
//! core instead of duplicating the whole startup path per backend.

use par6_config::{GripperConfig, RobotConfig};

use crate::bus::DriverBus;
use crate::hw::SocketCanBus;
use crate::sim::SimBus;
use crate::types::{
    BusError, BusState, Freshness, GripperCommand, JointCommand, LinkHealth, NodeId, PollAction,
};

/// The bus backend a running daemon talks to.
///
/// Delegation only — every variant keeps its own semantics; this type
/// exists so the backend is a runtime choice rather than a type
/// parameter that fans out through the whole daemon.
pub enum RuntimeBus {
    /// Closed-loop simulator (runs anywhere, including CI).
    Sim(Box<SimBus>),
    /// SocketCAN hardware.
    SocketCan(Box<SocketCanBus>),
}

impl RuntimeBus {
    /// Whether this is the simulator backend.
    pub fn is_sim(&self) -> bool {
        matches!(self, Self::Sim(_))
    }

    /// The simulator backend's own surface (teleport re-seeding and the
    /// other sim-only hooks); `None` on hardware.
    pub fn sim_mut(&mut self) -> Option<&mut SimBus> {
        match self {
            Self::Sim(b) => Some(&mut **b),
            Self::SocketCan(_) => None,
        }
    }
}

impl From<SimBus> for RuntimeBus {
    fn from(b: SimBus) -> Self {
        Self::Sim(Box::new(b))
    }
}

impl From<SocketCanBus> for RuntimeBus {
    fn from(b: SocketCanBus) -> Self {
        Self::SocketCan(Box::new(b))
    }
}

/// Forward a `&mut self` method to the active backend.
macro_rules! dispatch {
    ($self:ident, $method:ident ( $($arg:expr),* )) => {
        match $self {
            RuntimeBus::Sim(b) => b.$method($($arg),*),
            RuntimeBus::SocketCan(b) => b.$method($($arg),*),
        }
    };
}

impl DriverBus for RuntimeBus {
    fn begin_tick(&mut self, tick: u64) {
        dispatch!(self, begin_tick(tick))
    }

    fn drain_rx(&mut self, state: &mut BusState) -> Result<usize, BusError> {
        dispatch!(self, drain_rx(state))
    }

    fn send_joint_commands(&mut self, commands: &[JointCommand]) -> Result<(), BusError> {
        dispatch!(self, send_joint_commands(commands))
    }

    fn send_gripper(&mut self, command: &GripperCommand) -> Result<(), BusError> {
        dispatch!(self, send_gripper(command))
    }

    fn poll_step(&mut self) -> Result<(), BusError> {
        dispatch!(self, poll_step())
    }

    fn queue_poll_override(&mut self, action: PollAction, repeats: u16) {
        dispatch!(self, queue_poll_override(action, repeats))
    }

    fn boot_configure(
        &mut self,
        robot: &RobotConfig,
        gripper: Option<&GripperConfig>,
        repeats: u8,
    ) -> Result<(), BusError> {
        dispatch!(self, boot_configure(robot, gripper, repeats))
    }

    fn resend_node_config(&mut self, node: NodeId, repeats: u8) -> Result<(), BusError> {
        dispatch!(self, resend_node_config(node, repeats))
    }

    fn send_limits(
        &mut self,
        node: NodeId,
        velocity_limit_ticks_s: f32,
        current_limit_ma: f32,
        repeats: u8,
    ) -> Result<(), BusError> {
        dispatch!(
            self,
            send_limits(node, velocity_limit_ticks_s, current_limit_ma, repeats)
        )
    }

    fn send_clear_error(&mut self, node: NodeId, repeats: u8) -> Result<(), BusError> {
        dispatch!(self, send_clear_error(node, repeats))
    }

    fn set_silent(&mut self, silent: bool) {
        dispatch!(self, set_silent(silent))
    }

    fn is_silent(&self) -> bool {
        dispatch!(self, is_silent())
    }

    fn freshness(&self, node: NodeId) -> Freshness {
        dispatch!(self, freshness(node))
    }

    fn clear_lost_latch(&mut self, node: NodeId) {
        dispatch!(self, clear_lost_latch(node))
    }

    fn rebase_freshness(&mut self) {
        dispatch!(self, rebase_freshness())
    }

    fn connected_nodes(&self) -> u16 {
        dispatch!(self, connected_nodes())
    }

    fn link_health(&self) -> LinkHealth {
        dispatch!(self, link_health())
    }
}
