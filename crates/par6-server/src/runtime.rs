//! The boundary between the command plane and the rest of `par6d`.
//!
//! Two small traits: [`Planner`] executes queued commands (planning →
//! sample-ring feeding → completion reporting) and [`RtCommands`] applies
//! immediate effects (streaming setpoints, enable/disable, I/O, backend
//! switches). `par6d` wires them to `par6-motion` / `par6-rt`; tests drive
//! them with in-crate doubles. The server itself owns the motion queue,
//! the index allocator, and all protocol bookkeeping — these traits only
//! carry what must cross into the motion/RT layers.
//!
//! Units on this boundary are WIRE units (mm / degrees / fractions), as
//! decoded by `par6-proto`; the implementations convert to SI internally,
//! exactly like the protocol spec prescribes for the runtime.

use par6_proto::{CompletionPolicy, Shape, WireError, EN_SLOTS, NUM_JOINTS};
use par6_rt::{SnapshotReader, StateSnapshot};

/// Outcome of a queued command previously handed to [`Planner::start`].
#[derive(Debug, Clone, PartialEq)]
pub struct CommandOutcome {
    /// The command index passed to [`Planner::start`].
    pub index: u64,
    /// `None` = finished ok; `Some` = finished with this error. The server
    /// overwrites `command_index` with the queue index before it goes on
    /// the wire, so implementations may leave it unattributed.
    pub error: Option<WireError>,
}

/// Per-joint / per-axis enablement flags (freedom before hitting limits),
/// computed by the motion layer: it owns kinematics and the limit model.
///
/// Slot layout matches the wire: `[j1−, j1+, …, j6−, j6+]` for joints and
/// `[x−, x+, y−, y+, z−, z+, rx−, …]` per Cartesian frame; 1 = motion
/// allowed in that direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Enablement {
    /// Per-joint direction flags.
    pub joint_en: [u8; EN_SLOTS],
    /// Per-axis flags in the world reference frame.
    pub cart_en_wrf: [u8; EN_SLOTS],
    /// Per-axis flags in the tool reference frame.
    pub cart_en_trf: [u8; EN_SLOTS],
}

impl Default for Enablement {
    fn default() -> Self {
        Self {
            joint_en: [1; EN_SLOTS],
            cart_en_wrf: [1; EN_SLOTS],
            cart_en_trf: [1; EN_SLOTS],
        }
    }
}

/// Planning context the server pushes to the planner whenever one of its
/// pieces changes (profile / tool / TCP offset / shapes / completion
/// policy). The planner needs it to plan correctly; the server remains
/// the owner of the authoritative copy it reports in queries and STATUS.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanContext<'a> {
    /// Active motion profile name.
    pub profile: &'a str,
    /// Active tool registry key; empty = none selected.
    pub tool: &'a str,
    /// Active jaw/variant key; `None` = tool default.
    pub tool_variant: Option<&'a str>,
    /// TCP offset in the tool-local frame (mm).
    pub tcp_offset_mm: [f64; 3],
    /// Program-layer collision shapes (wire units).
    pub shapes: &'a [Shape],
    /// Controller-side completion policy for queued motion.
    pub completion_policy: CompletionPolicy,
}

/// Executes queued commands: plans them, feeds the RT sample ring, and
/// reports completion. Exactly one command is in flight at a time — the
/// server serializes the queue and calls [`Planner::start`] only after
/// the previous command's outcome arrived (or was cancelled).
pub trait Planner: Send {
    /// Begin executing queued command `index` (wire units). `Err` means
    /// the command never started; the server latches the error, clears
    /// the pending queue, and pushes `COMPLETE(ok=false)`.
    fn start(&mut self, index: u64, cmd: &par6_proto::Command) -> Result<(), WireError>;

    /// Poll the outcome of the in-flight command; `None` while it is
    /// still running. Outcomes for cancelled indexes are ignored by the
    /// server, so implementations may report them unconditionally.
    fn poll(&mut self) -> Option<CommandOutcome>;

    /// Cancel the in-flight command (if any) and discard its planned
    /// samples. Idempotent. No outcome is expected afterwards.
    fn cancel(&mut self);

    /// The planning context changed (profile / tool / TCP offset /
    /// shapes / completion policy). Also called once at server startup
    /// with the initial context.
    fn sync(&mut self, ctx: PlanContext<'_>);

    /// Current enablement flags for the REACHABLE query and the STATUS
    /// broadcast.
    fn enablement(&self) -> Enablement;
}

/// Immediate command effects forwarded to the RT core. All methods are
/// fire-and-forget from the server's point of view unless they return a
/// `Result`; failures surface to the client as ERROR replies.
pub trait RtCommands: Send {
    /// Forward a streaming setpoint (`servo_j` / `servo_j_pose` /
    /// `servo_l` / `jog_j` / `jog_l`, wire units). Called both to start
    /// a stream and to update the active one in place — the RT session
    /// watchdog owns termination.
    fn stream(&mut self, cmd: &par6_proto::Command);

    /// Stop the active streaming session (hold in place). Idempotent.
    fn cancel_stream(&mut self);

    /// Halt all motion now (stop/estop scope). Idempotent.
    fn halt(&mut self);

    /// Enable (`reset`) or disable (`estop` latch) motion.
    fn set_enabled(&mut self, enabled: bool);

    /// Instantly set joint angles (degrees) and optionally tool joint
    /// positions. Only called in simulator mode — the server gates
    /// `teleport` with `SYS_NOT_SIMULATOR` beforehand.
    fn teleport(&mut self, angles_deg: &[f64; NUM_JOINTS], tool_positions: Option<&[f64]>);

    /// Set one digital output (`port` 0..=7, `value` 0/1).
    fn write_io(&mut self, port: u8, value: u8);

    /// Switch the bus backend between hardware and simulator live
    /// (state re-seeded by the implementation).
    fn set_simulator(&mut self, on: bool) -> Result<(), WireError>;

    /// (Re)connect the hardware bus on `port`.
    fn connect_hardware(&mut self, port: &str) -> Result<(), WireError>;

    /// Full controller state reset follow-through on the RT side
    /// (`reset_state`): clear latched errors and re-sync.
    fn reset_state(&mut self);

    /// Reset loop timing statistics (truly unacked fire-and-forget).
    fn reset_loop_stats(&mut self);
}

/// Everything the server needs from the rest of `par6d`, bundled.
pub struct RuntimeHandle<P: Planner, R: RtCommands> {
    /// Queued-command execution.
    pub planner: P,
    /// Immediate effects.
    pub rt: R,
    /// Reader half of the RT snapshot channel; feeds queries, STATUS
    /// (including `link_ok` / `data_age_ms` freshness) and telemetry.
    pub snapshots: SnapshotReader<StateSnapshot>,
}
