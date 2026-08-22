//! Client→server commands: param structs, validation, encode/decode.
//!
//! Wire form: `[cmd_tag, req_id, ...params]`, with QUEUED commands carrying an
//! idempotency key as the first param: `[cmd_tag, req_id, key, ...params]`.
//!
//! Conventions frozen here:
//! - Exactly one "unspecified" encoding: msgpack `nil` (`Option` in Rust).
//!   There are no `0.0` sentinels anywhere.
//! - Fixed arity per command. Params are always present (as `nil` when
//!   unspecified); a wrong element count is a decode error.
//! - Units at the wire are mm and degrees; `speed`/`accel` are fractions of
//!   the configured maxima in `(0, 1]`; jog speeds are signed fractions in
//!   `[-1, 1]`; durations are seconds.
//! - A TCP pose `[x, y, z, rx, ry, rz]` rotates intrinsic XYZ,
//!   `R = Rx(rx)·Ry(ry)·Rz(rz)`. A collision
//!   [`Shape`]'s pose is the other way round — waldoctl's `Shape.pose`
//!   contract is extrinsic XYZ, `R = Rz·Ry·Rx`, so that the shape the
//!   frontend draws is the shape the checker enforces.
//! - All floats must be finite; NaN/inf are rejected at decode.
//! - The codec validates shape and ranges only. Joint limits, tool names and
//!   recipe names are configuration, validated in the server layer (the codec
//!   performs no process-global lookups).

use crate::enums::{CmdType, CompletionPolicy, Frame};
use crate::wire::{w_array, w_bool, w_f64, w_int, w_nil, w_str, w_uint, Reader};
use crate::{DecodeError, NUM_JOINTS};

/// Longest duration any command may carry \[s\].
///
/// Every duration ends up in `Duration::from_secs_f64`, which PANICS above
/// ~1.8e19 s, and in `Instant + Duration`, which panics above ~9.2e18 s —
/// so an unbounded duration on the wire is a reachable abort of the whole
/// runtime. An hour is past anything a queued dwell or a timed move needs
/// and far below the arithmetic cliff.
pub const MAX_DURATION_S: f64 = 3600.0;

/// Longest duration a `jog_*` watchdog may be armed for \[s\].
///
/// The duration IS the watchdog: it is what stops the jog when the client
/// streaming it goes away, and UIs refresh it at 20–50 Hz (par6's own
/// client defaults to 0.1 s). A minute is two orders of magnitude above
/// that and still short enough that one datagram cannot leave the arm
/// jogging until it hits a soft limit — which is the whole reason jog
/// carries a duration. Long traverses are `move_j`'s job.
pub const MAX_JOG_DURATION_S: f64 = 60.0;

/// Largest waypoint list a `move_s` / `move_p` may carry.
///
/// The reassembler admits [`crate::MAX_TRANSFER_BYTES`] (4 MiB), which is
/// ~73 k float64 waypoints — minutes of planner work, and tens of MB of
/// allocation, out of one transfer. 10 k waypoints is ~540 kB on the
/// wire, well inside that budget and longer than any path a 250 Hz arm is
/// asked to spline through.
pub const MAX_WAYPOINTS: usize = 10_000;

/// Largest collision-shape list a `set_shapes` may carry. The world these
/// populate is installation keep-outs plus a program's own fixtures —
/// tens, not thousands — and every one is checked against every link on
/// every gate call.
pub const MAX_SHAPES: usize = 256;

/// Longest bare float vector any command may carry. The widest real user
/// is a `plane` shape's four params; `teleport.tool_positions` and
/// `tool_action.params` are capped at 16 by [`Command::validate`].
const MAX_VEC_ELEMS: usize = 16;

/// One workspace collision shape (mirrors waldoctl `Shape.to_wire()`).
///
/// Wire form: `[kind, params, pose, collision, margin|nil, name]`.
#[derive(Debug, Clone, PartialEq)]
pub struct Shape {
    /// Shape kind (`"box"`, `"sphere"`, …) — interpreted by the server layer.
    pub kind: String,
    /// Kind-specific dimensions (mm).
    pub params: Vec<f64>,
    /// Shape pose (mm / degrees), kind-specific length.
    pub pose: Vec<f64>,
    /// Whether the shape participates in collision checking.
    pub collision: bool,
    /// Optional safety margin (mm); `None` = server default.
    pub margin: Option<f64>,
    /// Display name.
    pub name: String,
}

/// One scalar parameter of a [`Command::ToolAction`].
#[derive(Debug, Clone, PartialEq)]
pub enum ToolParam {
    /// A float parameter (position, speed, current, …).
    Float(f64),
    /// An integer parameter.
    Int(i64),
    /// A flag parameter.
    Bool(bool),
    /// A symbolic parameter.
    Str(String),
}

// ---------------------------------------------------------------------------
// Param structs
// ---------------------------------------------------------------------------

/// STOP: halt motion with explicit cancel scope; controller stays ENABLED.
#[derive(Debug, Clone, PartialEq)]
pub struct Stop {
    /// Also clear the pending queue (not just the active motion).
    pub clear_queue: bool,
}

/// WRITE_IO: set one digital output.
#[derive(Debug, Clone, PartialEq)]
pub struct WriteIo {
    /// Output port index, `0..=7`.
    pub port: u8,
    /// Output level, `0` or `1`.
    pub value: u8,
}

/// SIMULATOR: switch the bus backend between hardware and simulator.
#[derive(Debug, Clone, PartialEq)]
pub struct Simulator {
    /// `true` = simulator backend.
    pub on: bool,
}

/// PAUSE: hold or resume the executing trajectory.
///
/// Distinct from STOP: the sample ring is left intact, so resuming
/// continues the move from where it paused instead of requiring the
/// caller to re-issue it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pause {
    /// `true` holds the executing trajectory; `false` resumes it.
    pub on: bool,
}

/// SET_GRAVITY_COMP: apply (or stop applying) the G(q) feedforward.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SetGravityComp {
    /// `true` = apply the feedforward.
    pub on: bool,
}

/// SELECT_PROFILE: select the motion planner profile.
#[derive(Debug, Clone, PartialEq)]
pub struct SelectProfile {
    /// Profile name (1–32 chars), validated against config server-side.
    pub profile: String,
}

/// CONNECT_HARDWARE: (re)connect the hardware bus.
#[derive(Debug, Clone, PartialEq)]
pub struct ConnectHardware {
    /// Bus/port identifier (1–256 chars), e.g. `"can0"`.
    pub port: String,
}

/// SET_TCP_OFFSET: offset the effective TCP in the tool-local frame (mm).
#[derive(Debug, Clone, PartialEq)]
pub struct SetTcpOffset {
    /// X offset (mm).
    pub x: f64,
    /// Y offset (mm).
    pub y: f64,
    /// Z offset (mm).
    pub z: f64,
}

/// SET_SHAPES: replace the workspace collision-world shapes.
#[derive(Debug, Clone, PartialEq)]
pub struct SetShapes {
    /// The new program-layer shape set.
    pub shapes: Vec<Shape>,
}

/// SET_COMPLETION_POLICY: select the controller-side completion policy.
#[derive(Debug, Clone, PartialEq)]
pub struct SetCompletionPolicy {
    /// The policy to apply to subsequent queued motion.
    pub policy: CompletionPolicy,
}

/// SET_RECIPE: select the telemetry recipe (unknown names are refused).
#[derive(Debug, Clone, PartialEq)]
pub struct SetRecipe {
    /// Recipe name (1–64 chars), validated against config server-side.
    pub name: String,
}

/// POSE query params.
#[derive(Debug, Clone, PartialEq)]
pub struct PoseQuery {
    /// Reference frame; `None` = server default (WRF).
    pub frame: Option<Frame>,
}

/// SERVO_J: streaming joint position target.
#[derive(Debug, Clone, PartialEq)]
pub struct ServoJ {
    /// Target joint angles (degrees).
    pub angles: [f64; NUM_JOINTS],
    /// Velocity fraction `(0, 1]`; `None` = server default.
    pub speed: Option<f64>,
    /// Acceleration fraction `(0, 1]`; `None` = server default.
    pub accel: Option<f64>,
}

/// SERVO_J_POSE: streaming joint position target via Cartesian pose (IK).
#[derive(Debug, Clone, PartialEq)]
pub struct ServoJPose {
    /// Target pose `[x, y, z, rx, ry, rz]` (mm / degrees).
    pub pose: [f64; 6],
    /// Velocity fraction `(0, 1]`; `None` = server default.
    pub speed: Option<f64>,
    /// Acceleration fraction `(0, 1]`; `None` = server default.
    pub accel: Option<f64>,
}

/// SERVO_L: streaming linear Cartesian position target.
#[derive(Debug, Clone, PartialEq)]
pub struct ServoL {
    /// Target pose `[x, y, z, rx, ry, rz]` (mm / degrees).
    pub pose: [f64; 6],
    /// Velocity fraction `(0, 1]`; `None` = server default.
    pub speed: Option<f64>,
    /// Acceleration fraction `(0, 1]`; `None` = server default.
    pub accel: Option<f64>,
}

/// JOG_J: streaming joint velocity with a self-terminating watchdog.
#[derive(Debug, Clone, PartialEq)]
pub struct JogJ {
    /// Signed velocity fractions per joint, each in `[-1, 1]`.
    pub speeds: [f64; NUM_JOINTS],
    /// Watchdog duration (seconds, > 0); UIs stream fresh jogs at 20–50 Hz.
    pub duration: f64,
    /// Acceleration fraction `(0, 1]`; `None` = server default.
    pub accel: Option<f64>,
}

/// JOG_L: streaming Cartesian velocity with a watchdog.
#[derive(Debug, Clone, PartialEq)]
pub struct JogL {
    /// Signed velocity fractions `[vx, vy, vz, wx, wy, wz]`, each in `[-1, 1]`.
    pub velocities: [f64; 6],
    /// Watchdog duration (seconds, > 0).
    pub duration: f64,
    /// Reference frame.
    pub frame: Frame,
    /// Acceleration fraction `(0, 1]`; `None` = server default.
    pub accel: Option<f64>,
}

/// TELEPORT: instantly set joint angles. Simulator only — a real error
/// (`SYS_NOT_SIMULATOR`) on hardware, never a silent no-op.
#[derive(Debug, Clone, PartialEq)]
pub struct Teleport {
    /// Joint angles (degrees).
    pub angles: [f64; NUM_JOINTS],
    /// Optional tool joint positions; `None` = leave the tool alone.
    pub tool_positions: Option<Vec<f64>>,
}

/// HOME: run the homing sequence.
#[derive(Debug, Clone, PartialEq)]
pub struct Home {
    /// Idempotency key (client-generated uuid64).
    pub key: u64,
}

/// MOVE_J: joint-space move to target angles.
#[derive(Debug, Clone, PartialEq)]
pub struct MoveJ {
    /// Idempotency key (client-generated uuid64).
    pub key: u64,
    /// Target joint angles (degrees); relative deltas when `rel`.
    pub angles: [f64; NUM_JOINTS],
    /// Move duration (seconds, > 0). Exactly one of `duration`/`speed`.
    pub duration: Option<f64>,
    /// Velocity fraction `(0, 1]`. Exactly one of `duration`/`speed`.
    pub speed: Option<f64>,
    /// Acceleration fraction `(0, 1]`; `None` = server default.
    pub accel: Option<f64>,
    /// Blend radius (mm, ≥ 0); `None` = no blending.
    pub blend_radius: Option<f64>,
    /// Interpret `angles` as deltas from the current position.
    pub rel: bool,
}

/// MOVE_J_POSE: joint-space move to a Cartesian pose (IK at target).
#[derive(Debug, Clone, PartialEq)]
pub struct MoveJPose {
    /// Idempotency key (client-generated uuid64).
    pub key: u64,
    /// Target pose `[x, y, z, rx, ry, rz]` (mm / degrees).
    pub pose: [f64; 6],
    /// Move duration (seconds, > 0). Exactly one of `duration`/`speed`.
    pub duration: Option<f64>,
    /// Velocity fraction `(0, 1]`. Exactly one of `duration`/`speed`.
    pub speed: Option<f64>,
    /// Acceleration fraction `(0, 1]`; `None` = server default.
    pub accel: Option<f64>,
    /// Blend radius (mm, ≥ 0); `None` = no blending.
    pub blend_radius: Option<f64>,
}

/// MOVE_L: linear Cartesian move.
#[derive(Debug, Clone, PartialEq)]
pub struct MoveL {
    /// Idempotency key (client-generated uuid64).
    pub key: u64,
    /// Target pose `[x, y, z, rx, ry, rz]` (mm / degrees).
    pub pose: [f64; 6],
    /// Reference frame.
    pub frame: Frame,
    /// Move duration (seconds, > 0). Exactly one of `duration`/`speed`.
    pub duration: Option<f64>,
    /// Velocity fraction `(0, 1]`. Exactly one of `duration`/`speed`.
    pub speed: Option<f64>,
    /// Acceleration fraction `(0, 1]`; `None` = server default.
    pub accel: Option<f64>,
    /// Blend radius (mm, ≥ 0); `None` = no blending.
    pub blend_radius: Option<f64>,
    /// Interpret `pose` as a delta from the current pose.
    pub rel: bool,
}

/// MOVE_C: circular arc through current → via → end.
#[derive(Debug, Clone, PartialEq)]
pub struct MoveC {
    /// Idempotency key (client-generated uuid64).
    pub key: u64,
    /// Via pose `[x, y, z, rx, ry, rz]` (mm / degrees).
    pub via: [f64; 6],
    /// End pose `[x, y, z, rx, ry, rz]` (mm / degrees).
    pub end: [f64; 6],
    /// Reference frame.
    pub frame: Frame,
    /// Move duration (seconds, > 0). Exactly one of `duration`/`speed`.
    pub duration: Option<f64>,
    /// Velocity fraction `(0, 1]`. Exactly one of `duration`/`speed`.
    pub speed: Option<f64>,
    /// Acceleration fraction `(0, 1]`; `None` = server default.
    pub accel: Option<f64>,
    /// Blend radius (mm, ≥ 0); `None` = no blending.
    pub blend_radius: Option<f64>,
}

/// MOVE_S: cubic spline through waypoints (bulk; may arrive chunked).
#[derive(Debug, Clone, PartialEq)]
pub struct MoveS {
    /// Idempotency key (client-generated uuid64).
    pub key: u64,
    /// Waypoint poses (≥ 2), each `[x, y, z, rx, ry, rz]` (mm / degrees).
    pub waypoints: Vec<[f64; 6]>,
    /// Reference frame.
    pub frame: Frame,
    /// Move duration (seconds, > 0). Exactly one of `duration`/`speed`.
    pub duration: Option<f64>,
    /// Velocity fraction `(0, 1]`. Exactly one of `duration`/`speed`.
    pub speed: Option<f64>,
    /// Acceleration fraction `(0, 1]`; `None` = server default.
    pub accel: Option<f64>,
}

/// MOVE_P: process move — constant TCP speed, auto-blended corners (bulk).
#[derive(Debug, Clone, PartialEq)]
pub struct MoveP {
    /// Idempotency key (client-generated uuid64).
    pub key: u64,
    /// Waypoint poses (≥ 2), each `[x, y, z, rx, ry, rz]` (mm / degrees).
    pub waypoints: Vec<[f64; 6]>,
    /// Reference frame.
    pub frame: Frame,
    /// Move duration (seconds, > 0). Exactly one of `duration`/`speed`.
    pub duration: Option<f64>,
    /// Velocity fraction `(0, 1]`. Exactly one of `duration`/`speed`.
    pub speed: Option<f64>,
    /// Acceleration fraction `(0, 1]`; `None` = server default.
    pub accel: Option<f64>,
}

/// SELECT_TOOL: select the active end-of-arm tool.
#[derive(Debug, Clone, PartialEq)]
pub struct SelectTool {
    /// Idempotency key (client-generated uuid64).
    pub key: u64,
    /// Tool name (1–64 chars), validated against the registry server-side.
    pub tool_name: String,
    /// Jaw/variant key (≤ 64 chars); `None` = tool default.
    pub variant_key: Option<String>,
}

/// DELAY: queued dwell.
#[derive(Debug, Clone, PartialEq)]
pub struct Delay {
    /// Idempotency key (client-generated uuid64).
    pub key: u64,
    /// Dwell time (seconds, > 0).
    pub seconds: f64,
}

/// CHECKPOINT: queue marker for progress tracking.
#[derive(Debug, Clone, PartialEq)]
pub struct Checkpoint {
    /// Idempotency key (client-generated uuid64).
    pub key: u64,
    /// Marker label (1–128 chars).
    pub label: String,
}

/// TOOL_ACTION: generic tool action, delegated server-side.
#[derive(Debug, Clone, PartialEq)]
pub struct ToolAction {
    /// Idempotency key (client-generated uuid64).
    pub key: u64,
    /// Tool key (1–64 chars), validated against the registry server-side.
    pub tool_key: String,
    /// Action name (1–64 chars).
    pub action: String,
    /// Action parameters (≤ 16 scalars).
    pub params: Vec<ToolParam>,
}

/// A decoded (validated) command. See the module docs for the wire form.
#[derive(Debug, Clone, PartialEq)]
#[allow(missing_docs)] // variants are documented via CmdType and the param structs
pub enum Command {
    // SYSTEM
    Reset,
    Estop,
    SafetyStop,
    SetGravityComp(SetGravityComp),
    /// Hold or resume the executing trajectory.
    Pause(Pause),
    Stop(Stop),
    WriteIo(WriteIo),
    Simulator(Simulator),
    SelectProfile(SelectProfile),
    ResetState,
    ConnectHardware(ConnectHardware),
    SetTcpOffset(SetTcpOffset),
    SetShapes(SetShapes),
    SetCompletionPolicy(SetCompletionPolicy),
    SetRecipe(SetRecipe),
    // QUERY
    Ping,
    Status,
    Angles,
    Pose(PoseQuery),
    Io,
    Speeds,
    Tools,
    Queue,
    Activity,
    LoopStats,
    Profile,
    Reachable,
    Error,
    TcpSpeed,
    TcpOffset,
    ToolStatus,
    IsSimulator,
    Shapes,
    ConfigInfo,
    // FIRE_AND_FORGET
    ServoJ(ServoJ),
    ServoJPose(ServoJPose),
    ServoL(ServoL),
    JogJ(JogJ),
    JogL(JogL),
    Teleport(Teleport),
    ResetLoopStats,
    // QUEUED
    Home(Home),
    MoveJ(MoveJ),
    MoveJPose(MoveJPose),
    MoveL(MoveL),
    MoveC(MoveC),
    MoveS(MoveS),
    MoveP(MoveP),
    SelectTool(SelectTool),
    Delay(Delay),
    Checkpoint(Checkpoint),
    ToolAction(ToolAction),
}

impl Command {
    /// The wire tag for this command.
    pub fn tag(&self) -> CmdType {
        use Command as C;
        match self {
            C::Reset => CmdType::Reset,
            C::Estop => CmdType::Estop,
            C::SafetyStop => CmdType::SafetyStop,
            C::SetGravityComp(_) => CmdType::SetGravityComp,
            C::Pause(_) => CmdType::Pause,
            C::Stop(_) => CmdType::Stop,
            C::WriteIo(_) => CmdType::WriteIo,
            C::Simulator(_) => CmdType::Simulator,
            C::SelectProfile(_) => CmdType::SelectProfile,
            C::ResetState => CmdType::ResetState,
            C::ConnectHardware(_) => CmdType::ConnectHardware,
            C::SetTcpOffset(_) => CmdType::SetTcpOffset,
            C::SetShapes(_) => CmdType::SetShapes,
            C::SetCompletionPolicy(_) => CmdType::SetCompletionPolicy,
            C::SetRecipe(_) => CmdType::SetRecipe,
            C::Ping => CmdType::Ping,
            C::Status => CmdType::Status,
            C::Angles => CmdType::Angles,
            C::Pose(_) => CmdType::Pose,
            C::Io => CmdType::Io,
            C::Speeds => CmdType::Speeds,
            C::Tools => CmdType::Tools,
            C::Queue => CmdType::Queue,
            C::Activity => CmdType::Activity,
            C::LoopStats => CmdType::LoopStats,
            C::Profile => CmdType::Profile,
            C::Reachable => CmdType::Reachable,
            C::Error => CmdType::Error,
            C::TcpSpeed => CmdType::TcpSpeed,
            C::TcpOffset => CmdType::TcpOffset,
            C::ToolStatus => CmdType::ToolStatus,
            C::IsSimulator => CmdType::IsSimulator,
            C::Shapes => CmdType::Shapes,
            C::ConfigInfo => CmdType::ConfigInfo,
            C::ServoJ(_) => CmdType::ServoJ,
            C::ServoJPose(_) => CmdType::ServoJPose,
            C::ServoL(_) => CmdType::ServoL,
            C::JogJ(_) => CmdType::JogJ,
            C::JogL(_) => CmdType::JogL,
            C::Teleport(_) => CmdType::Teleport,
            C::ResetLoopStats => CmdType::ResetLoopStats,
            C::Home(_) => CmdType::Home,
            C::MoveJ(_) => CmdType::MoveJ,
            C::MoveJPose(_) => CmdType::MoveJPose,
            C::MoveL(_) => CmdType::MoveL,
            C::MoveC(_) => CmdType::MoveC,
            C::MoveS(_) => CmdType::MoveS,
            C::MoveP(_) => CmdType::MoveP,
            C::SelectTool(_) => CmdType::SelectTool,
            C::Delay(_) => CmdType::Delay,
            C::Checkpoint(_) => CmdType::Checkpoint,
            C::ToolAction(_) => CmdType::ToolAction,
        }
    }

    /// The idempotency key, for QUEUED commands.
    pub fn idempotency_key(&self) -> Option<u64> {
        use Command as C;
        match self {
            C::Home(p) => Some(p.key),
            C::MoveJ(p) => Some(p.key),
            C::MoveJPose(p) => Some(p.key),
            C::MoveL(p) => Some(p.key),
            C::MoveC(p) => Some(p.key),
            C::MoveS(p) => Some(p.key),
            C::MoveP(p) => Some(p.key),
            C::SelectTool(p) => Some(p.key),
            C::Delay(p) => Some(p.key),
            C::Checkpoint(p) => Some(p.key),
            C::ToolAction(p) => Some(p.key),
            _ => None,
        }
    }

    /// Validate parameter ranges/shapes (called automatically by
    /// [`decode_command`] and [`encode_command`]).
    pub fn validate(&self) -> Result<(), DecodeError> {
        use Command as C;
        match self {
            C::Stop(_)
            | C::Simulator(_)
            | C::SetGravityComp(_)
            | C::Pause(_)
            | C::Reset
            | C::Estop
            | C::SafetyStop
            | C::ResetState
            | C::SetCompletionPolicy(_)
            | C::Ping
            | C::Status
            | C::Angles
            | C::Pose(_)
            | C::Io
            | C::Speeds
            | C::Tools
            | C::Queue
            | C::Activity
            | C::LoopStats
            | C::Profile
            | C::Reachable
            | C::Error
            | C::TcpSpeed
            | C::TcpOffset
            | C::ToolStatus
            | C::IsSimulator
            | C::Shapes
            | C::ConfigInfo
            | C::ResetLoopStats => Ok(()),
            C::WriteIo(p) => {
                check(p.port <= 7, "write_io.port", "must be 0..=7")?;
                check(p.value <= 1, "write_io.value", "must be 0 or 1")
            }
            C::SelectProfile(p) => str_len("select_profile.profile", &p.profile, 1, 32),
            C::ConnectHardware(p) => str_len("connect_hardware.port", &p.port, 1, 256),
            C::SetTcpOffset(p) => {
                finite("set_tcp_offset.x", p.x)?;
                finite("set_tcp_offset.y", p.y)?;
                finite("set_tcp_offset.z", p.z)
            }
            C::SetShapes(p) => {
                check(
                    p.shapes.len() <= MAX_SHAPES,
                    "set_shapes.shapes",
                    &format!("at most {MAX_SHAPES} shapes"),
                )?;
                for s in &p.shapes {
                    str_len("shape.kind", &s.kind, 1, 32)?;
                    str_len("shape.name", &s.name, 0, 128)?;
                    finite_all("shape.params", &s.params)?;
                    finite_all("shape.pose", &s.pose)?;
                    if let Some(m) = s.margin {
                        finite("shape.margin", m)?;
                        check(m >= 0.0, "shape.margin", "must be >= 0")?;
                    }
                }
                Ok(())
            }
            C::SetRecipe(p) => str_len("set_recipe.name", &p.name, 1, 64),
            C::ServoJ(p) => {
                finite_all("servo_j.angles", &p.angles)?;
                opt_frac("servo_j.speed", p.speed)?;
                opt_frac("servo_j.accel", p.accel)
            }
            C::ServoJPose(p) => {
                finite_all("servo_j_pose.pose", &p.pose)?;
                opt_frac("servo_j_pose.speed", p.speed)?;
                opt_frac("servo_j_pose.accel", p.accel)
            }
            C::ServoL(p) => {
                finite_all("servo_l.pose", &p.pose)?;
                opt_frac("servo_l.speed", p.speed)?;
                opt_frac("servo_l.accel", p.accel)
            }
            C::JogJ(p) => {
                signed_fracs("jog_j.speeds", &p.speeds)?;
                bounded_duration("jog_j.duration", p.duration, MAX_JOG_DURATION_S)?;
                opt_frac("jog_j.accel", p.accel)
            }
            C::JogL(p) => {
                signed_fracs("jog_l.velocities", &p.velocities)?;
                bounded_duration("jog_l.duration", p.duration, MAX_JOG_DURATION_S)?;
                opt_frac("jog_l.accel", p.accel)
            }
            C::Teleport(p) => {
                finite_all("teleport.angles", &p.angles)?;
                if let Some(tp) = &p.tool_positions {
                    check(
                        tp.len() <= 16,
                        "teleport.tool_positions",
                        "at most 16 values",
                    )?;
                    finite_all("teleport.tool_positions", tp)?;
                }
                Ok(())
            }
            C::Home(_) => Ok(()),
            C::MoveJ(p) => {
                finite_all("move_j.angles", &p.angles)?;
                motion_timing("move_j", p.duration, p.speed, p.accel)?;
                blend("move_j.r", p.blend_radius)
            }
            C::MoveJPose(p) => {
                finite_all("move_j_pose.pose", &p.pose)?;
                motion_timing("move_j_pose", p.duration, p.speed, p.accel)?;
                blend("move_j_pose.r", p.blend_radius)
            }
            C::MoveL(p) => {
                finite_all("move_l.pose", &p.pose)?;
                motion_timing("move_l", p.duration, p.speed, p.accel)?;
                blend("move_l.r", p.blend_radius)
            }
            C::MoveC(p) => {
                finite_all("move_c.via", &p.via)?;
                finite_all("move_c.end", &p.end)?;
                motion_timing("move_c", p.duration, p.speed, p.accel)?;
                blend("move_c.r", p.blend_radius)
            }
            C::MoveS(p) => {
                waypoints("move_s.waypoints", &p.waypoints)?;
                motion_timing("move_s", p.duration, p.speed, p.accel)
            }
            C::MoveP(p) => {
                waypoints("move_p.waypoints", &p.waypoints)?;
                motion_timing("move_p", p.duration, p.speed, p.accel)
            }
            C::SelectTool(p) => {
                str_len("select_tool.tool_name", &p.tool_name, 1, 64)?;
                match &p.variant_key {
                    Some(v) => str_len("select_tool.variant_key", v, 1, 64),
                    None => Ok(()),
                }
            }
            C::Delay(p) => duration("delay.seconds", p.seconds),
            C::Checkpoint(p) => str_len("checkpoint.label", &p.label, 1, 128),
            C::ToolAction(p) => {
                str_len("tool_action.tool_key", &p.tool_key, 1, 64)?;
                str_len("tool_action.action", &p.action, 1, 64)?;
                check(
                    p.params.len() <= 16,
                    "tool_action.params",
                    "at most 16 values",
                )?;
                for tp in &p.params {
                    if let ToolParam::Float(v) = tp {
                        finite("tool_action.params", *v)?;
                    }
                }
                Ok(())
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Validation helpers
// ---------------------------------------------------------------------------

fn check(ok: bool, what: &'static str, why: &str) -> Result<(), DecodeError> {
    if ok {
        Ok(())
    } else {
        Err(DecodeError::Validation {
            what,
            why: why.to_owned(),
        })
    }
}

fn finite(what: &'static str, v: f64) -> Result<(), DecodeError> {
    check(v.is_finite(), what, "must be finite")
}

fn finite_all(what: &'static str, vs: &[f64]) -> Result<(), DecodeError> {
    for v in vs {
        finite(what, *v)?;
    }
    Ok(())
}

fn frac(what: &'static str, v: f64) -> Result<(), DecodeError> {
    finite(what, v)?;
    check(v > 0.0 && v <= 1.0, what, "must be in (0, 1]")
}

fn opt_frac(what: &'static str, v: Option<f64>) -> Result<(), DecodeError> {
    match v {
        Some(v) => frac(what, v),
        None => Ok(()),
    }
}

fn signed_fracs(what: &'static str, vs: &[f64]) -> Result<(), DecodeError> {
    for v in vs {
        finite(what, *v)?;
        check((-1.0..=1.0).contains(v), what, "must be in [-1, 1]")?;
    }
    Ok(())
}

fn duration(what: &'static str, v: f64) -> Result<(), DecodeError> {
    bounded_duration(what, v, MAX_DURATION_S)
}

fn bounded_duration(what: &'static str, v: f64, max: f64) -> Result<(), DecodeError> {
    finite(what, v)?;
    check(
        v > 0.0 && v <= max,
        what,
        &format!("must be in (0, {max}] seconds"),
    )
}

fn blend(what: &'static str, v: Option<f64>) -> Result<(), DecodeError> {
    match v {
        Some(v) => {
            finite(what, v)?;
            check(v >= 0.0, what, "must be >= 0")
        }
        None => Ok(()),
    }
}

fn motion_timing(
    what: &'static str,
    dur: Option<f64>,
    speed: Option<f64>,
    accel: Option<f64>,
) -> Result<(), DecodeError> {
    match (dur, speed) {
        (Some(d), None) => duration(what, d)?,
        (None, Some(s)) => frac(what, s)?,
        (None, None) => {
            return Err(DecodeError::Validation {
                what,
                why: "requires one of duration or speed".into(),
            });
        }
        (Some(_), Some(_)) => {
            return Err(DecodeError::Validation {
                what,
                why: "duration and speed are mutually exclusive".into(),
            });
        }
    }
    opt_frac(what, accel)
}

fn str_len(what: &'static str, s: &str, min: usize, max: usize) -> Result<(), DecodeError> {
    check(
        s.len() >= min && s.len() <= max,
        what,
        &format!("length must be {min}..={max} bytes"),
    )
}

fn waypoints(what: &'static str, wps: &[[f64; 6]]) -> Result<(), DecodeError> {
    check(wps.len() >= 2, what, "requires at least 2 waypoints")?;
    check(
        wps.len() <= MAX_WAYPOINTS,
        what,
        &format!("at most {MAX_WAYPOINTS} waypoints"),
    )?;
    for wp in wps {
        finite_all(what, wp)?;
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Encode
// ---------------------------------------------------------------------------

/// Number of wire elements for each command (tag + req_id + params).
fn arity(tag: CmdType) -> usize {
    use CmdType as T;
    match tag {
        T::Reset | T::Estop | T::SafetyStop | T::ResetState | T::ResetLoopStats => 2,
        T::Ping
        | T::Status
        | T::Angles
        | T::Io
        | T::Speeds
        | T::Tools
        | T::Queue
        | T::Activity
        | T::LoopStats
        | T::Profile
        | T::Reachable
        | T::Error
        | T::TcpSpeed
        | T::TcpOffset
        | T::ToolStatus
        | T::IsSimulator
        | T::Shapes
        | T::ConfigInfo => 2,
        T::Stop
        | T::Simulator
        | T::SetGravityComp
        | T::Pause
        | T::SelectProfile
        | T::ConnectHardware
        | T::SetShapes
        | T::SetCompletionPolicy
        | T::SetRecipe
        | T::Pose => 3,
        T::WriteIo => 4,
        T::SetTcpOffset => 5,
        T::ServoJ | T::ServoJPose | T::ServoL => 5,
        T::JogJ => 5,
        T::JogL => 6,
        T::Teleport => 4,
        T::Home => 3,
        T::MoveJ => 9,
        T::MoveJPose => 8,
        T::MoveL => 10,
        T::MoveC => 10,
        T::MoveS | T::MoveP => 8,
        T::SelectTool => 5,
        T::Delay => 4,
        T::Checkpoint => 4,
        T::ToolAction => 6,
    }
}

fn w_fixed(buf: &mut Vec<u8>, vs: &[f64]) {
    w_array(buf, vs.len());
    for v in vs {
        w_f64(buf, *v);
    }
}

fn w_opt_f64(buf: &mut Vec<u8>, v: Option<f64>) {
    match v {
        Some(v) => w_f64(buf, v),
        None => w_nil(buf),
    }
}

pub(crate) fn w_shape(buf: &mut Vec<u8>, s: &Shape) {
    w_array(buf, 6);
    w_str(buf, &s.kind);
    w_fixed(buf, &s.params);
    w_fixed(buf, &s.pose);
    w_bool(buf, s.collision);
    w_opt_f64(buf, s.margin);
    w_str(buf, &s.name);
}

/// Encode `[cmd_tag, req_id, ...params]` into `buf` (cleared first).
///
/// Validates the command; the bytes of an invalid command never hit the wire.
pub fn encode_command(cmd: &Command, req_id: u32, buf: &mut Vec<u8>) -> Result<(), DecodeError> {
    cmd.validate()?;
    buf.clear();
    let tag = cmd.tag();
    w_array(buf, arity(tag));
    w_uint(buf, u64::from(tag as u16));
    w_uint(buf, u64::from(req_id));
    use Command as C;
    match cmd {
        C::Reset
        | C::Estop
        | C::SafetyStop
        | C::ResetState
        | C::ResetLoopStats
        | C::Ping
        | C::Status
        | C::Angles
        | C::Io
        | C::Speeds
        | C::Tools
        | C::Queue
        | C::Activity
        | C::LoopStats
        | C::Profile
        | C::Reachable
        | C::Error
        | C::TcpSpeed
        | C::TcpOffset
        | C::ToolStatus
        | C::IsSimulator
        | C::Shapes
        | C::ConfigInfo => {}
        C::Stop(p) => w_bool(buf, p.clear_queue),
        C::WriteIo(p) => {
            w_uint(buf, u64::from(p.port));
            w_uint(buf, u64::from(p.value));
        }
        C::Simulator(p) => w_bool(buf, p.on),
        C::SetGravityComp(p) => w_bool(buf, p.on),
        C::Pause(p) => w_bool(buf, p.on),
        C::SelectProfile(p) => w_str(buf, &p.profile),
        C::ConnectHardware(p) => w_str(buf, &p.port),
        C::SetTcpOffset(p) => {
            w_f64(buf, p.x);
            w_f64(buf, p.y);
            w_f64(buf, p.z);
        }
        C::SetShapes(p) => {
            w_array(buf, p.shapes.len());
            for s in &p.shapes {
                w_shape(buf, s);
            }
        }
        C::SetCompletionPolicy(p) => w_uint(buf, u64::from(p.policy as u8)),
        C::SetRecipe(p) => w_str(buf, &p.name),
        C::Pose(p) => match p.frame {
            Some(f) => w_uint(buf, u64::from(f as u8)),
            None => w_nil(buf),
        },
        C::ServoJ(p) => {
            w_fixed(buf, &p.angles);
            w_opt_f64(buf, p.speed);
            w_opt_f64(buf, p.accel);
        }
        C::ServoJPose(p) => {
            w_fixed(buf, &p.pose);
            w_opt_f64(buf, p.speed);
            w_opt_f64(buf, p.accel);
        }
        C::ServoL(p) => {
            w_fixed(buf, &p.pose);
            w_opt_f64(buf, p.speed);
            w_opt_f64(buf, p.accel);
        }
        C::JogJ(p) => {
            w_fixed(buf, &p.speeds);
            w_f64(buf, p.duration);
            w_opt_f64(buf, p.accel);
        }
        C::JogL(p) => {
            w_fixed(buf, &p.velocities);
            w_f64(buf, p.duration);
            w_uint(buf, u64::from(p.frame as u8));
            w_opt_f64(buf, p.accel);
        }
        C::Teleport(p) => {
            w_fixed(buf, &p.angles);
            match &p.tool_positions {
                Some(tp) => w_fixed(buf, tp),
                None => w_nil(buf),
            }
        }
        C::Home(p) => w_uint(buf, p.key),
        C::MoveJ(p) => {
            w_uint(buf, p.key);
            w_fixed(buf, &p.angles);
            w_opt_f64(buf, p.duration);
            w_opt_f64(buf, p.speed);
            w_opt_f64(buf, p.accel);
            w_opt_f64(buf, p.blend_radius);
            w_bool(buf, p.rel);
        }
        C::MoveJPose(p) => {
            w_uint(buf, p.key);
            w_fixed(buf, &p.pose);
            w_opt_f64(buf, p.duration);
            w_opt_f64(buf, p.speed);
            w_opt_f64(buf, p.accel);
            w_opt_f64(buf, p.blend_radius);
        }
        C::MoveL(p) => {
            w_uint(buf, p.key);
            w_fixed(buf, &p.pose);
            w_uint(buf, u64::from(p.frame as u8));
            w_opt_f64(buf, p.duration);
            w_opt_f64(buf, p.speed);
            w_opt_f64(buf, p.accel);
            w_opt_f64(buf, p.blend_radius);
            w_bool(buf, p.rel);
        }
        C::MoveC(p) => {
            w_uint(buf, p.key);
            w_fixed(buf, &p.via);
            w_fixed(buf, &p.end);
            w_uint(buf, u64::from(p.frame as u8));
            w_opt_f64(buf, p.duration);
            w_opt_f64(buf, p.speed);
            w_opt_f64(buf, p.accel);
            w_opt_f64(buf, p.blend_radius);
        }
        C::MoveS(p) => {
            w_uint(buf, p.key);
            w_array(buf, p.waypoints.len());
            for wp in &p.waypoints {
                w_fixed(buf, wp);
            }
            w_uint(buf, u64::from(p.frame as u8));
            w_opt_f64(buf, p.duration);
            w_opt_f64(buf, p.speed);
            w_opt_f64(buf, p.accel);
        }
        C::MoveP(p) => {
            w_uint(buf, p.key);
            w_array(buf, p.waypoints.len());
            for wp in &p.waypoints {
                w_fixed(buf, wp);
            }
            w_uint(buf, u64::from(p.frame as u8));
            w_opt_f64(buf, p.duration);
            w_opt_f64(buf, p.speed);
            w_opt_f64(buf, p.accel);
        }
        C::SelectTool(p) => {
            w_uint(buf, p.key);
            w_str(buf, &p.tool_name);
            match &p.variant_key {
                Some(v) => w_str(buf, v),
                None => w_nil(buf),
            }
        }
        C::Delay(p) => {
            w_uint(buf, p.key);
            w_f64(buf, p.seconds);
        }
        C::Checkpoint(p) => {
            w_uint(buf, p.key);
            w_str(buf, &p.label);
        }
        C::ToolAction(p) => {
            w_uint(buf, p.key);
            w_str(buf, &p.tool_key);
            w_str(buf, &p.action);
            w_array(buf, p.params.len());
            for tp in &p.params {
                match tp {
                    ToolParam::Float(v) => w_f64(buf, *v),
                    ToolParam::Int(v) => w_int(buf, *v),
                    ToolParam::Bool(v) => w_bool(buf, *v),
                    ToolParam::Str(v) => w_str(buf, v),
                }
            }
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Decode
// ---------------------------------------------------------------------------

fn r_fixed6(r: &mut Reader<'_>, what: &'static str) -> Result<[f64; 6], DecodeError> {
    let n = r.array_len()?;
    if n != 6 {
        return Err(DecodeError::Arity {
            what,
            expected: 6,
            got: n,
        });
    }
    let mut out = [0.0; 6];
    for v in &mut out {
        *v = r.f64()?;
    }
    Ok(out)
}

/// Read a length header and refuse it before anything is reserved on its
/// word. `array_len` accepts msgpack's 0xDD form up to 4 294 967 295 with
/// no cross-check against the bytes actually present, so a nine-byte
/// datagram can otherwise ask the allocator for hundreds of gigabytes and
/// abort the process on `handle_alloc_error`.
pub(crate) fn r_len(
    r: &mut Reader<'_>,
    what: &'static str,
    max: usize,
) -> Result<usize, DecodeError> {
    let n = r.array_len()?;
    check(n <= max, what, &format!("at most {max} elements"))?;
    Ok(n)
}

fn r_vec_f64(r: &mut Reader<'_>, what: &'static str) -> Result<Vec<f64>, DecodeError> {
    let n = r_len(r, what, MAX_VEC_ELEMS)?;
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        out.push(r.f64()?);
    }
    Ok(out)
}

fn r_frame(r: &mut Reader<'_>) -> Result<Frame, DecodeError> {
    let v = r.uint()?;
    Frame::from_wire(v as i64).ok_or(DecodeError::InvalidEnum {
        what: "frame",
        value: v as i64,
    })
}

fn r_waypoints(r: &mut Reader<'_>, what: &'static str) -> Result<Vec<[f64; 6]>, DecodeError> {
    let n = r_len(r, what, MAX_WAYPOINTS)?;
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        out.push(r_fixed6(r, "waypoint")?);
    }
    Ok(out)
}

pub(crate) fn r_shape(r: &mut Reader<'_>) -> Result<Shape, DecodeError> {
    let n = r.array_len()?;
    if n != 6 {
        return Err(DecodeError::Arity {
            what: "shape",
            expected: 6,
            got: n,
        });
    }
    Ok(Shape {
        kind: r.str()?.to_owned(),
        params: r_vec_f64(r, "shape.params")?,
        pose: r_vec_f64(r, "shape.pose")?,
        collision: r.bool()?,
        margin: r.opt_f64()?,
        name: r.str()?.to_owned(),
    })
}

fn r_tool_param(r: &mut Reader<'_>) -> Result<ToolParam, DecodeError> {
    match r.peek_marker()? {
        0xC2 | 0xC3 => Ok(ToolParam::Bool(r.bool()?)),
        0xCB => Ok(ToolParam::Float(r.f64()?)),
        0xA0..=0xBF | 0xD9..=0xDB => Ok(ToolParam::Str(r.str()?.to_owned())),
        0x00..=0x7F | 0xE0..=0xFF | 0xCC..=0xCF | 0xD0..=0xD3 => Ok(ToolParam::Int(r.int()?)),
        _ => Err(DecodeError::Validation {
            what: "tool_action.params",
            why: "parameters must be float, int, bool or str".into(),
        }),
    }
}

/// Decode `[cmd_tag, req_id, ...params]`, returning `(req_id, command)`.
///
/// Rejects unknown tags, wrong arity, wrong types, non-minimal payload shapes
/// and out-of-range values. Trailing bytes after the array are an error.
pub fn decode_command(data: &[u8]) -> Result<(u32, Command), DecodeError> {
    let mut r = Reader::new(data);
    let n = r.array_len()?;
    if n < 2 {
        return Err(DecodeError::Arity {
            what: "command envelope",
            expected: 2,
            got: n,
        });
    }
    let raw_tag = r.int()?;
    let tag = CmdType::from_wire(raw_tag).ok_or(DecodeError::UnknownTag(raw_tag))?;
    let expected = arity(tag);
    if n != expected {
        return Err(DecodeError::Arity {
            what: "command",
            expected,
            got: n,
        });
    }
    let req_id = u32::try_from(r.uint()?).map_err(|_| DecodeError::Validation {
        what: "req_id",
        why: "exceeds u32".into(),
    })?;

    use CmdType as T;
    let cmd = match tag {
        T::Reset => Command::Reset,
        T::Estop => Command::Estop,
        T::SafetyStop => Command::SafetyStop,
        T::Stop => Command::Stop(Stop {
            clear_queue: r.bool()?,
        }),
        T::WriteIo => {
            let port = r.uint()?;
            let value = r.uint()?;
            Command::WriteIo(WriteIo {
                port: u8::try_from(port).map_err(|_| DecodeError::Validation {
                    what: "write_io.port",
                    why: "must be 0..=7".into(),
                })?,
                value: u8::try_from(value).map_err(|_| DecodeError::Validation {
                    what: "write_io.value",
                    why: "must be 0 or 1".into(),
                })?,
            })
        }
        T::Simulator => Command::Simulator(Simulator { on: r.bool()? }),
        T::SetGravityComp => Command::SetGravityComp(SetGravityComp { on: r.bool()? }),
        T::Pause => Command::Pause(Pause { on: r.bool()? }),
        T::SelectProfile => Command::SelectProfile(SelectProfile {
            profile: r.str()?.to_owned(),
        }),
        T::ResetState => Command::ResetState,
        T::ConnectHardware => Command::ConnectHardware(ConnectHardware {
            port: r.str()?.to_owned(),
        }),
        T::SetTcpOffset => Command::SetTcpOffset(SetTcpOffset {
            x: r.f64()?,
            y: r.f64()?,
            z: r.f64()?,
        }),
        T::SetShapes => {
            let n = r_len(&mut r, "set_shapes.shapes", MAX_SHAPES)?;
            let mut shapes = Vec::with_capacity(n);
            for _ in 0..n {
                shapes.push(r_shape(&mut r)?);
            }
            Command::SetShapes(SetShapes { shapes })
        }
        T::SetCompletionPolicy => {
            let v = r.uint()?;
            let policy = CompletionPolicy::from_wire(v as i64).ok_or(DecodeError::InvalidEnum {
                what: "completion policy",
                value: v as i64,
            })?;
            Command::SetCompletionPolicy(SetCompletionPolicy { policy })
        }
        T::SetRecipe => Command::SetRecipe(SetRecipe {
            name: r.str()?.to_owned(),
        }),
        T::Ping => Command::Ping,
        T::Status => Command::Status,
        T::Angles => Command::Angles,
        T::Pose => {
            let frame = if r.peek_nil() {
                r.nil()?;
                None
            } else {
                Some(r_frame(&mut r)?)
            };
            Command::Pose(PoseQuery { frame })
        }
        T::Io => Command::Io,
        T::Speeds => Command::Speeds,
        T::Tools => Command::Tools,
        T::Queue => Command::Queue,
        T::Activity => Command::Activity,
        T::LoopStats => Command::LoopStats,
        T::Profile => Command::Profile,
        T::Reachable => Command::Reachable,
        T::Error => Command::Error,
        T::TcpSpeed => Command::TcpSpeed,
        T::TcpOffset => Command::TcpOffset,
        T::ToolStatus => Command::ToolStatus,
        T::IsSimulator => Command::IsSimulator,
        T::Shapes => Command::Shapes,
        T::ConfigInfo => Command::ConfigInfo,
        T::ServoJ => Command::ServoJ(ServoJ {
            angles: r_fixed6(&mut r, "servo_j.angles")?,
            speed: r.opt_f64()?,
            accel: r.opt_f64()?,
        }),
        T::ServoJPose => Command::ServoJPose(ServoJPose {
            pose: r_fixed6(&mut r, "servo_j_pose.pose")?,
            speed: r.opt_f64()?,
            accel: r.opt_f64()?,
        }),
        T::ServoL => Command::ServoL(ServoL {
            pose: r_fixed6(&mut r, "servo_l.pose")?,
            speed: r.opt_f64()?,
            accel: r.opt_f64()?,
        }),
        T::JogJ => Command::JogJ(JogJ {
            speeds: r_fixed6(&mut r, "jog_j.speeds")?,
            duration: r.f64()?,
            accel: r.opt_f64()?,
        }),
        T::JogL => Command::JogL(JogL {
            velocities: r_fixed6(&mut r, "jog_l.velocities")?,
            duration: r.f64()?,
            frame: r_frame(&mut r)?,
            accel: r.opt_f64()?,
        }),
        T::Teleport => Command::Teleport(Teleport {
            angles: r_fixed6(&mut r, "teleport.angles")?,
            tool_positions: if r.peek_nil() {
                r.nil()?;
                None
            } else {
                Some(r_vec_f64(&mut r, "teleport.tool_positions")?)
            },
        }),
        T::ResetLoopStats => Command::ResetLoopStats,
        T::Home => Command::Home(Home { key: r.uint()? }),
        T::MoveJ => Command::MoveJ(MoveJ {
            key: r.uint()?,
            angles: r_fixed6(&mut r, "move_j.angles")?,
            duration: r.opt_f64()?,
            speed: r.opt_f64()?,
            accel: r.opt_f64()?,
            blend_radius: r.opt_f64()?,
            rel: r.bool()?,
        }),
        T::MoveJPose => Command::MoveJPose(MoveJPose {
            key: r.uint()?,
            pose: r_fixed6(&mut r, "move_j_pose.pose")?,
            duration: r.opt_f64()?,
            speed: r.opt_f64()?,
            accel: r.opt_f64()?,
            blend_radius: r.opt_f64()?,
        }),
        T::MoveL => Command::MoveL(MoveL {
            key: r.uint()?,
            pose: r_fixed6(&mut r, "move_l.pose")?,
            frame: r_frame(&mut r)?,
            duration: r.opt_f64()?,
            speed: r.opt_f64()?,
            accel: r.opt_f64()?,
            blend_radius: r.opt_f64()?,
            rel: r.bool()?,
        }),
        T::MoveC => Command::MoveC(MoveC {
            key: r.uint()?,
            via: r_fixed6(&mut r, "move_c.via")?,
            end: r_fixed6(&mut r, "move_c.end")?,
            frame: r_frame(&mut r)?,
            duration: r.opt_f64()?,
            speed: r.opt_f64()?,
            accel: r.opt_f64()?,
            blend_radius: r.opt_f64()?,
        }),
        T::MoveS => Command::MoveS(MoveS {
            key: r.uint()?,
            waypoints: r_waypoints(&mut r, "move_s.waypoints")?,
            frame: r_frame(&mut r)?,
            duration: r.opt_f64()?,
            speed: r.opt_f64()?,
            accel: r.opt_f64()?,
        }),
        T::MoveP => Command::MoveP(MoveP {
            key: r.uint()?,
            waypoints: r_waypoints(&mut r, "move_p.waypoints")?,
            frame: r_frame(&mut r)?,
            duration: r.opt_f64()?,
            speed: r.opt_f64()?,
            accel: r.opt_f64()?,
        }),
        T::SelectTool => Command::SelectTool(SelectTool {
            key: r.uint()?,
            tool_name: r.str()?.to_owned(),
            variant_key: if r.peek_nil() {
                r.nil()?;
                None
            } else {
                Some(r.str()?.to_owned())
            },
        }),
        T::Delay => Command::Delay(Delay {
            key: r.uint()?,
            seconds: r.f64()?,
        }),
        T::Checkpoint => Command::Checkpoint(Checkpoint {
            key: r.uint()?,
            label: r.str()?.to_owned(),
        }),
        T::ToolAction => {
            let key = r.uint()?;
            let tool_key = r.str()?.to_owned();
            let action = r.str()?.to_owned();
            let n = r_len(&mut r, "tool_action.params", MAX_VEC_ELEMS)?;
            let mut params = Vec::with_capacity(n);
            for _ in 0..n {
                params.push(r_tool_param(&mut r)?);
            }
            Command::ToolAction(ToolAction {
                key,
                tool_key,
                action,
                params,
            })
        }
    };
    r.finish()?;
    cmd.validate()?;
    Ok((req_id, cmd))
}
