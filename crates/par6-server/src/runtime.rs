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

use par6_proto::{CompletionPolicy, Layer, Shape, WireError, EN_SLOTS, NUM_JOINTS};
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
    /// Settle verdict on a successful tool move, straight off the
    /// gripper reply (1 = object while closing, 2 = object while
    /// opening, 3 = target reached, no object); `None` elsewhere. Rides
    /// the COMPLETE push so a pick verifies atomically instead of racing
    /// a TOOL_STATUS poll against the next queued motion.
    pub verdict: Option<u8>,
}

/// Per-joint / per-axis enablement flags (freedom before hitting limits),
/// computed by the motion layer: it owns kinematics and the limit model.
///
/// Slot layout matches the wire, POSITIVE DIRECTION FIRST:
/// `[j1+, j1−, …, j6+, j6−]` for joints and
/// `[x+, x−, y+, y−, z+, z−, rx+, rx−, ry+, ry−, rz+, rz−]` per Cartesian
/// frame; 1 = motion allowed in that direction. The order is the one the
/// waldoctl frontend unpacks (`can_jog_pos[i] = slot[2i]`) and the one the
/// parol6 runtime publishes — a backend that fills the pairs the other way
/// round greys out the opposite button.
///
/// The wire slots are 0/1 — there is no "unknown" — and clients gate jog
/// controls on them, so an implementation sets 1 only for a direction its
/// model says is free. The server narrows what it gets before it goes out:
/// a runtime configured without kinematics reports no Cartesian freedom,
/// whatever the planner claims.
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

/// A runtime payload: mass at a COM with optional rotational inertia,
/// in end-effector-frame coordinates. `mass == 0` = no payload.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PayloadSpec {
    /// Payload mass \[kg\].
    pub mass: f64,
    /// COM in end-effector-frame coordinates \[m\].
    pub com: [f64; 3],
    /// Rotational inertia about the COM `(Ixx, Ixy, Iyy, Ixz, Iyz,
    /// Izz)`; `None` = point mass.
    pub inertia: Option<[f64; 6]>,
}

/// Planning context the server pushes to the planner whenever one of its
/// pieces changes (profile / tool / TCP offset / completion policy). The
/// planner needs it to plan correctly; the server remains the owner of
/// the authoritative copy it reports in queries and STATUS.
///
/// Collision shapes do NOT ride here: applying them can fail, and a
/// refusal has to reach the client, so they go through
/// [`Planner::set_shapes`] instead.
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
    /// Controller-side completion policy for queued motion.
    pub completion_policy: CompletionPolicy,
    /// The runtime payload the torque feedforward must carry.
    pub payload: PayloadSpec,
}

/// The collision verdict STATUS carries: `collision_active` and, when it
/// is, the colliding geometry pairs by name.
///
/// It describes the configuration a motion was BLOCKED AT — the pairs the
/// refused move would have collided in — not a sample of wherever the arm
/// happens to be standing. That is what a client can act on and
/// highlight, and it is why an arm resting in a park pose whose own links
/// touch does not raise a permanent alarm.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct CollisionState {
    /// Whether a motion was blocked by collision.
    pub active: bool,
    /// The colliding pairs; empty unless `active`.
    pub pairs: Vec<(String, String)>,
}

/// One queued command offered to [`Planner::start`], with the index the
/// server allocated for it.
#[derive(Debug, Clone, Copy)]
pub struct QueuedCommand<'a> {
    /// Wire command index (the one the client was acked with).
    pub index: u64,
    /// The decoded command.
    pub cmd: &'a par6_proto::Command,
}

/// The blend radius \[mm\] a queued command carries, if its type has one
/// and the client set it. `None` and `Some(0.0)` both mean "stop at the
/// target"; only a positive radius asks for a rounded corner.
///
/// One rule for the server (which decides how long to hold a move open
/// for its successor) and for the planner (which decides how far a blend
/// chain reaches), so the two cannot disagree about what blends.
pub fn blend_radius_mm(cmd: &par6_proto::Command) -> Option<f64> {
    use par6_proto::Command as C;
    match cmd {
        C::MoveJ(p) => p.blend_radius,
        C::MoveJPose(p) => p.blend_radius,
        C::MoveL(p) => p.blend_radius,
        C::MoveC(p) => p.blend_radius,
        _ => None,
    }
}

/// Executes queued commands: plans them, feeds the RT sample ring, and
/// reports completion. Exactly one MOTION is in flight at a time — the
/// server serializes the queue and calls [`Planner::start`] only after
/// the previous outcome arrived (or was cancelled) — but one motion may
/// cover SEVERAL queued commands when they blend into one another.
pub trait Planner: Send {
    /// Begin executing `batch[0]` (wire units). The rest of `batch` is
    /// the queue standing behind it, in order, offered for blending: an
    /// implementation may fold as many of those as it can honour into
    /// the same motion.
    ///
    /// Returns how many commands from the front of `batch` the started
    /// motion covers — at least 1, never more than `batch.len()`. The
    /// server removes exactly that many from its queue and completes all
    /// of them together when [`Planner::poll`] reports the outcome for
    /// `batch[0].index`; a blended-away command has no outcome of its
    /// own.
    ///
    /// `Err` means nothing started; the server latches the error against
    /// `batch[0].index`, clears the pending queue, and pushes
    /// `COMPLETE(ok=false)`.
    fn start(&mut self, batch: &[QueuedCommand<'_>]) -> Result<usize, WireError>;

    /// Poll the outcome of the in-flight command; `None` while it is
    /// still running. Outcomes for cancelled indexes are ignored by the
    /// server, so implementations may report them unconditionally.
    fn poll(&mut self) -> Option<CommandOutcome>;

    /// Cancel the in-flight command (if any) and discard its planned
    /// samples. Idempotent. No outcome is expected afterwards.
    fn cancel(&mut self);

    /// The planning context changed (profile / tool / TCP offset /
    /// completion policy). Also called once at server startup with the
    /// initial context.
    fn sync(&mut self, ctx: PlanContext<'_>);

    /// Replace one collision-world layer (wire units: metres and radians),
    /// returning the `scene_epoch` of the world now applied.
    ///
    /// `Ok(None)` = this runtime enforces no collision world (a build
    /// without kinematics); the applied world's epoch is the server's to
    /// count either way, and STATUS keeps reporting no collision. When a
    /// world IS enforced the returned epoch must equal the server's — both
    /// count accepted replacements from zero. `Err` refuses the shape set
    /// WHOLE: the previously applied world stays enforced and its epoch
    /// does not move, so a malformed shape can never leave a half-built
    /// keep-out world in place.
    fn set_shapes(&mut self, layer: Layer, shapes: &[Shape]) -> Result<Option<u64>, WireError>;

    /// The latched collision verdict for the STATUS broadcast: the pairs
    /// the last blocked motion would have collided in. `None` = this
    /// runtime enforces no collision world.
    ///
    /// Called at the status cadence, so it is a read of what the gate
    /// already decided — never a fresh check.
    fn collision(&mut self) -> Option<CollisionState>;

    /// Planner-side warnings for the STATUS `warnings` slot (merged with
    /// the RT latch's own), e.g. a near-singular cartesian path. Read at
    /// the status cadence — a read of what planning already decided,
    /// never fresh work. Planners with nothing to report return empty.
    fn warnings(&self) -> Vec<WireError> {
        Vec::new()
    }

    /// Drop the latched collision verdict. The server calls this when it
    /// accepts a motion command, so a refusal's pairs never outlive the
    /// motion that produced them.
    fn clear_collision(&mut self);

    /// Current enablement flags for the REACHABLE query and the STATUS
    /// broadcast.
    fn enablement(&self) -> Enablement;

    /// Seconds of work the NOT-YET-STARTED `pending` commands represent,
    /// in queue order, as the planner's own timing model sees it.
    ///
    /// Planning is not free, so the server calls this only when the
    /// queue changes and caches the answer; an implementation times what
    /// its model can time and contributes NOTHING for a command it
    /// cannot time without executing it. The reported total therefore
    /// means "queued seconds that are known" — never a guess, and never
    /// to be read as "the queue is nearly done" when it is small.
    fn queued_duration(&mut self, pending: &[QueuedCommand<'_>]) -> f64;

    /// Seconds of motion the command IN FLIGHT still has to run, given
    /// the RT snapshot the server is reporting from. 0 when nothing is
    /// in flight, or when the running command has no duration model
    /// (homing, a gripper action).
    ///
    /// Read at the status cadence, so it must be arithmetic on state the
    /// planner already holds — never a re-plan.
    fn inflight_duration(&self, snap: &StateSnapshot) -> f64;
}

/// Immediate command effects forwarded to the RT core. All methods are
/// fire-and-forget from the server's point of view unless they return a
/// `Result`; failures surface to the client as ERROR replies.
pub trait RtCommands: Send {
    /// Forward a streaming setpoint (`servo_j` / `servo_j_pose` /
    /// `servo_l` / `jog_j` / `jog_l`, wire units). Called both to start
    /// a stream and to update the active one in place — the RT session
    /// watchdog owns termination.
    ///
    /// `Err` REFUSES the setpoint: nothing was forwarded, and the server
    /// answers the datagram with the error (and latches it as the
    /// standing error like any refused fire-and-forget). This is where
    /// the streaming collision gate lives — parol6 gates jog/servo
    /// against its collision world too (`collision_blocked` plus a
    /// velocity-scaled lookahead), and a jog the world blocks must be a
    /// spoken refusal, never a silent drop. Implementations without a
    /// collision world simply always return `Ok`.
    fn stream(&mut self, cmd: &par6_proto::Command) -> Result<(), WireError>;

    /// Stop the active streaming session (hold in place). Idempotent.
    fn cancel_stream(&mut self);

    /// Halt all motion now (stop/estop scope). Idempotent.
    fn halt(&mut self);

    /// Enable (`reset`) or disable (`estop` latch) motion.
    ///
    /// Enabling is a REQUEST, not a fact: the RT core refuses it while
    /// the e-stop is engaged or a hard error is latched, and it takes
    /// several ticks to answer. Implementations start the request here
    /// and publish its outcome through
    /// [`take_enable_outcome`](RtCommands::take_enable_outcome).
    /// Disabling is immediate, and cancels any outstanding enable.
    fn set_enabled(&mut self, enabled: bool);

    /// Apply (or stop applying) the gravity-compensation feedforward.
    ///
    /// G(q) is computed and published every tick regardless; this controls
    /// only whether it is fed forward, which cancels weight that must
    /// actually exist in the plant — true on hardware and on the torque
    /// plant, false on the kinematic one.
    fn set_gravity_comp(&mut self, on: bool);

    /// Replace the runtime payload in the gravity model (the planner's
    /// own dynamics follow through [`Planner::sync`]'s `payload`).
    /// Inputs are wire-validated (finite, mass >= 0, PSD inertia)
    /// before this is called.
    fn set_payload(&mut self, payload: PayloadSpec);

    /// Hold or resume the executing trajectory, leaving the sample ring
    /// intact so a resume continues rather than restarts.
    fn set_exec_paused(&mut self, paused: bool);

    /// Take the outcome of the last `set_enabled(true)` request, once the
    /// RT has actually answered it: `Some(Ok(()))` when the core came up
    /// ENABLED, `Some(Err(..))` when it refused or was superseded, `None`
    /// while the request is still outstanding or there is none.
    ///
    /// The server holds the `reset` reply until this resolves — waldoctl
    /// forbids reporting success for something the backend could not
    /// confirm, and "the enable was queued" is not "the arm will move".
    /// Each outcome is delivered exactly once.
    fn take_enable_outcome(&mut self) -> Option<Result<(), WireError>>;

    /// Instantly set joint angles (degrees) and optionally tool joint
    /// positions. Only called in simulator mode — the server gates
    /// `teleport` with `SYS_NOT_SIMULATOR` beforehand.
    fn teleport(&mut self, angles_deg: &[f64; NUM_JOINTS], tool_positions: Option<&[f64]>);

    /// Set one digital output (`port` 0..=7, `value` 0/1).
    fn write_io(&mut self, port: u8, value: u8);

    /// Ask the RT to enter FLASHING: assert the human park vouching and
    /// request the mode. Like an enable this is a REQUEST — the core
    /// refuses it outside IDLE/ACTIVE_ERROR, asynchronously — so
    /// implementations start it here and publish the verdict through
    /// [`take_flashing_outcome`](RtCommands::take_flashing_outcome).
    fn enter_flashing(&mut self);

    /// Ask the RT to leave FLASHING (mode back to IDLE; the bus wakes and
    /// the stored config is re-pushed). Same deferred-verdict discipline
    /// as [`enter_flashing`](RtCommands::enter_flashing). Only called
    /// when the reported mode IS `Flashing` — the server refuses an exit
    /// from any other mode, because `SetMode(Idle)` from a working mode
    /// would cancel motion the client never asked to stop.
    fn exit_flashing(&mut self);

    /// Take the outcome of the last `enter_flashing`/`exit_flashing`
    /// request once the published mode answers it: `Some(Ok(()))` when
    /// the mode reached the requested one, `Some(Err(..))` when the
    /// window closed without it. Each outcome is delivered exactly once.
    fn take_flashing_outcome(&mut self) -> Option<Result<(), WireError>>;

    /// Push one node's drive tuning through the stored boot-config path
    /// (`SET_PID_GAINS`). The server has already validated the values
    /// (codec) and the node id (config), so this only forwards.
    fn set_pid_gains(&mut self, gains: &par6_proto::command::SetPidGains);

    /// Halt the tool's jaws in place now, out-of-band of the queue.
    ///
    /// A queued `ToolAction("stop")` dispatches only after the very move
    /// it is meant to halt has settled, so the server fires the physical
    /// stop at admission; the queued instance re-applies it (idempotent
    /// at the RT — the re-target reads the same held jaw byte) and
    /// carries the ack/COMPLETE discipline. Like any command frame, the
    /// halt aborts a firmware calibration sweep in progress.
    fn tool_stop(&mut self);

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

    /// Mirror one collision-world layer into the RT side's own gate
    /// model (wire units, the same set just applied to the planner),
    /// returning that gate's `scene_epoch` after the apply.
    ///
    /// The planner's [`Planner::set_shapes`] is the authoritative apply
    /// — it validates and refuses first — so this is called only with a
    /// set the planner accepted, and conversion is the same deterministic
    /// path. `Ok(None)` = this runtime's streaming gate has no collision
    /// world, which is the default.
    fn set_shapes(&mut self, _layer: Layer, _shapes: &[Shape]) -> Result<Option<u64>, WireError> {
        Ok(None)
    }

    /// The streaming gate's latched collision verdict — the pairs a
    /// refused or stopped jog/servo would have collided in. `None` =
    /// this runtime gates no streams. Merged into STATUS alongside the
    /// planner's verdict; read at the status cadence, never a fresh
    /// check.
    fn collision(&mut self) -> Option<CollisionState> {
        None
    }

    /// Drop the streaming gate's latched verdict (the next accepted
    /// motion command supersedes it, exactly like the planner's).
    fn clear_collision(&mut self) {}
}

/// Everything the server needs from the rest of `par6d`, bundled.
pub struct RuntimeHandle<P: Planner, R: RtCommands> {
    /// Queued-command execution.
    pub planner: P,
    /// Immediate effects.
    pub rt: R,
    /// Reader half of the RT snapshot channel; feeds queries, STATUS and
    /// telemetry. `link_ok` / `data_age_ms` derive from the per-node
    /// `data_age_ticks` it carries (motor-bus freshness), aged further by
    /// the snapshot's own wall age.
    pub snapshots: SnapshotReader<StateSnapshot>,
}
