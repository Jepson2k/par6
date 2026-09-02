//! The [`DriverBus`] trait — the complete surface the RT tick loop uses
//! to talk to motor drivers, backend-agnostic (SocketCAN, closed-loop
//! sim, loopback).

use par6_config::{GripperConfig, RobotConfig};

use crate::types::{
    BusError, BusState, DriveTune, Freshness, GripperCommand, JointCommand, LinkHealth, NodeId,
    PollAction,
};

/// What the RT tick loop needs from a motor bus.
///
/// # Time model
///
/// The bus is `Instant`-free: all timing is expressed in RT **ticks**
/// (u64), supplied by the caller through [`begin_tick`](Self::begin_tick).
/// This keeps sim backends bit-for-bit deterministic — sim time is the
/// tick counter, wall time never enters the contract. Data ages,
/// freshness thresholds, and frame ages are all tick counts; config gives
/// thresholds in seconds and the caller converts once with
/// `RobotConfig::ticks`.
///
/// # Per-tick call order (measure-then-command)
///
/// ```text
/// begin_tick(tick)            advance the time base
/// drain_rx(&mut state)        decode everything pending (capped)
/// ... RT computes setpoints from state ...
/// send_joint_commands(&cmds)  one frame per arm joint
/// send_gripper(&cmd)          exactly one gripper-slot frame
/// poll_step()                 this tick's telemetry poll slot
/// ```
///
/// # Allocation
///
/// Implementations MUST NOT allocate in the per-tick methods
/// (`begin_tick`, `drain_rx`, `send_*`, `poll_step`, accessors). All
/// buffers are preallocated at construction / `boot_configure`. The
/// boot/config methods may allocate.
///
/// # Errors
///
/// Send errors are PROPAGATED (never swallowed); the RT loop decides the
/// reaction. A `Result` here reports transport failure — protocol-level
/// faults arrive through [`BusState`] instead.
pub trait DriverBus {
    /// Advance the bus time base to `tick`. Called exactly once per RT
    /// tick, before any other per-tick method; `tick` is monotonically
    /// non-decreasing. Freshness classification (stale/lost transitions,
    /// lost-latching) is evaluated here.
    fn begin_tick(&mut self, tick: u64);

    /// Drain pending RX frames into `state`, up to the configured
    /// per-tick cap (`bus.rx_frames_per_tick_cap`), and refresh every
    /// node's `data_age_ticks`. Returns the number of frames consumed.
    ///
    /// Per frame: the arbitration id's err bit is harvested into
    /// `live_error_bit` BEFORE payload dispatch; wrong-DLC frames are
    /// discarded whole (never partially applied).
    fn drain_rx(&mut self, state: &mut BusState) -> Result<usize, BusError>;

    /// Send one motion frame per arm joint. `commands[i]` targets the
    /// node of joint `i` (config order) — the single-send-per-joint-
    /// per-tick invariant is the caller's responsibility; backends may
    /// reject a second call in the same tick.
    ///
    /// `commands.len()` must equal the configured joint count.
    /// While silent ([`set_silent`](Self::set_silent)), any send is a
    /// contract violation (`InvalidCommand`).
    fn send_joint_commands(&mut self, commands: &[JointCommand]) -> Result<(), BusError>;

    /// Send this tick's single gripper-slot frame (motor / firmware /
    /// empty-poll / calibrate / dummy-node ping when no gripper).
    fn send_gripper(&mut self, command: &GripperCommand) -> Result<(), BusError>;

    /// Execute this tick's telemetry poll slot: the queued override if
    /// one is pending, otherwise the round-robin schedule (each node gets
    /// temperature/voltage/errors every `3 × total_nodes` ticks; hardware
    /// backends additionally run the periodic device-info sweep). While
    /// silent this is a no-op `Ok` — polls are suppressed but the tick
    /// structure stays uniform.
    fn poll_step(&mut self) -> Result<(), BusError>;

    /// Queue a poll override into the single-slot queue: `action` will
    /// preempt the round-robin poll for the next `repeats` calls of
    /// [`poll_step`](Self::poll_step). A queued override that has not
    /// finished is REPLACED (single slot, last writer wins).
    fn queue_poll_override(&mut self, action: PollAction, repeats: u16);

    /// Run the boot configuration sequence: for each node, `repeats`
    /// paced passes of Watchdog → Limits → Voltage_Limit → PD_Gains →
    /// Current_Gains → Velocity_Gains → Position_Gains, same seven to the
    /// gripper when present; then kt fetch (when `kt_source = auto`) and
    /// the bus scan. Stores the configs for later per-node resends.
    /// `gripper = None` means no CAN gripper is fitted.
    fn boot_configure(
        &mut self,
        robot: &RobotConfig,
        gripper: Option<&GripperConfig>,
        repeats: u8,
    ) -> Result<(), BusError>;

    /// Re-send one node's full stored configuration (`repeats` passes) —
    /// the reconnect path for nodes reported in
    /// [`BusState::reconnected_mask`].
    fn resend_node_config(&mut self, node: NodeId, repeats: u8) -> Result<(), BusError>;

    /// Replace one node's stored drive tuning (gains + limits + voltage
    /// limit; the watchdog is untouched) and push it now, `repeats`
    /// passes — the live half of `SET_PID_GAINS`. Because the STORED
    /// config changes, every later resend (reconnect, FLASHING exit)
    /// carries the new tune too. Unknown nodes are refused.
    fn retune_node(&mut self, node: NodeId, tune: &DriveTune, repeats: u8) -> Result<(), BusError>;

    /// Send a Limits frame (cmd 20: velocity limit ticks/s + current
    /// limit mA), `repeats` times. Homing uses this to drop a node to its
    /// homing current on FSM start and restore the normal Ilim on
    /// completion — the only path that also applies to the gripper motor.
    fn send_limits(
        &mut self,
        node: NodeId,
        velocity_limit_ticks_s: f32,
        current_limit_ma: f32,
        repeats: u8,
    ) -> Result<(), BusError>;

    /// Send Clear_Error (cmd 1) to a node, `repeats` times (the vendor
    /// clear sequence sends ×3 to each faulted node + gripper).
    fn send_clear_error(&mut self, node: NodeId, repeats: u8) -> Result<(), BusError>;

    /// Enter/leave bus-silent operation (FLASHING mode): while silent the
    /// backend transmits NOTHING (polls included, freshness checks
    /// suspended) and [`drain_rx`](Self::drain_rx) discards frames
    /// undecoded (bootloader page frames alias application ids). On
    /// leaving silence, callers must [`rebase_freshness`](Self::rebase_freshness).
    fn set_silent(&mut self, silent: bool);

    /// Whether the bus is currently silent.
    fn is_silent(&self) -> bool;

    /// Freshness classification of one node at the current tick.
    /// `Lost` is latched (see [`Freshness`]).
    fn freshness(&self, node: NodeId) -> Freshness;

    /// Clear one node's latched `Lost` state (user clear-errors path).
    fn clear_lost_latch(&mut self, node: NodeId);

    /// Required on FLASHING exit so the silent period does not read as
    /// a mass disconnect.
    fn rebase_freshness(&mut self);

    /// Bitmask of nodes that answered the boot bus scan (bit n = node n).
    fn connected_nodes(&self) -> u16;

    /// Last known kernel link health (bus-off/error-passive/restarts,
    /// sampled off the RT thread at ~1 Hz on hardware backends).
    fn link_health(&self) -> LinkHealth;
}
