//! Server→client replies and pushes: OK, ERROR, RESPONSE, COMPLETE.
//!
//! Wire forms:
//! - `[OK, req_id]` / `[OK, req_id, index]` (queued commands echo the index)
//! - `[ERROR, req_id, [command_index, code, title, cause, effect, remedy]]`
//! - `[RESPONSE, req_id, [query_tag, ...fields]]`
//! - `[COMPLETE, 0, index, ok]` / `[COMPLETE, 0, index, ok, detail]`
//!
//! `req_id` is the client-generated u32 echoed verbatim; pushes use 0.

use crate::command::r_len;
use crate::enums::{ActionState, MsgType, QueryType, ToolState};
use crate::error::WireError;
use crate::wire::{w_array, w_bool, w_f64, w_int, w_nil, w_str, w_uint, Reader};
use crate::{DecodeError, EN_SLOTS, MAX_IO_SLOTS, NUM_JOINTS, POSE_ELEMS};

/// Full tool status as it travels on the wire (STATUS body, STATUS query and
/// TOOL_STATUS query). 8 slots:
/// `[key, state, engaged, part_detected, fault_code, positions, channels,
/// variant_key]` — `variant_key` is new in v2.
#[derive(Debug, Clone, PartialEq)]
pub struct ToolStatusWire {
    /// Tool registry key; empty when no tool is selected.
    pub key: String,
    /// Tool state.
    pub state: ToolState,
    /// Whether the tool is currently engaged (e.g. gripping).
    pub engaged: bool,
    /// Whether a part is detected.
    pub part_detected: bool,
    /// Tool-specific fault code; 0 = no fault.
    pub fault_code: i32,
    /// Tool joint positions.
    pub positions: Vec<f64>,
    /// Tool analog channels.
    pub channels: Vec<f64>,
    /// Active jaw/variant key; empty = tool default.
    pub variant_key: String,
}

impl ToolStatusWire {
    pub(crate) fn encode(&self, buf: &mut Vec<u8>) {
        w_array(buf, 8);
        w_str(buf, &self.key);
        w_uint(buf, u64::from(self.state as u8));
        w_bool(buf, self.engaged);
        w_bool(buf, self.part_detected);
        w_int(buf, i64::from(self.fault_code));
        w_array(buf, self.positions.len());
        for v in &self.positions {
            w_f64(buf, *v);
        }
        w_array(buf, self.channels.len());
        for v in &self.channels {
            w_f64(buf, *v);
        }
        w_str(buf, &self.variant_key);
    }

    pub(crate) fn decode(r: &mut Reader<'_>) -> Result<Self, DecodeError> {
        let n = r.array_len()?;
        if n != 8 {
            return Err(DecodeError::Arity {
                what: "tool_status",
                expected: 8,
                got: n,
            });
        }
        let key = r.str()?.to_owned();
        let state_raw = r.uint()?;
        let state = ToolState::from_wire(state_raw as i64).ok_or(DecodeError::InvalidEnum {
            what: "tool state",
            value: state_raw as i64,
        })?;
        let engaged = r.bool()?;
        let part_detected = r.bool()?;
        let fault_code = i32::try_from(r.int()?).map_err(|_| DecodeError::Validation {
            what: "tool_status.fault_code",
            why: "exceeds i32".into(),
        })?;
        let mut positions = Vec::new();
        for _ in 0..r.array_len()? {
            positions.push(r.f64()?);
        }
        let mut channels = Vec::new();
        for _ in 0..r.array_len()? {
            channels.push(r.f64()?);
        }
        Ok(ToolStatusWire {
            key,
            state,
            engaged,
            part_detected,
            fault_code,
            positions,
            channels,
            variant_key: r.str()?.to_owned(),
        })
    }
}

/// Control-loop timing statistics (LOOP_STATS query result fields).
#[derive(Debug, Clone, PartialEq)]
pub struct LoopStatsResult {
    /// Configured tick rate.
    pub target_hz: f64,
    /// Ticks since start/reset.
    pub loop_count: u64,
    /// Ticks that exceeded the period budget.
    pub overrun_count: u64,
    /// Mean loop period (seconds).
    pub mean_period_s: f64,
    /// Standard deviation of the loop period (seconds).
    pub std_period_s: f64,
    /// Minimum observed period (seconds).
    pub min_period_s: f64,
    /// Maximum observed period (seconds).
    pub max_period_s: f64,
    /// 95th-percentile period (seconds).
    pub p95_period_s: f64,
    /// 99th-percentile period (seconds).
    pub p99_period_s: f64,
    /// Mean achieved rate (Hz).
    pub mean_hz: f64,
    /// 50th-percentile period (seconds).
    pub p50_period_s: f64,
    /// 90th-percentile period (seconds).
    pub p90_period_s: f64,
    /// Min CAN frame age seen in the last bus drain \[ticks\].
    pub can_frame_age_min_ticks: u64,
    /// Max CAN frame age seen in the last bus drain \[ticks\].
    pub can_frame_age_max_ticks: u64,
    /// Whether the RT thread runs under SCHED_FIFO (setup succeeded).
    pub rt_fifo: bool,
    /// Whether the RT thread is pinned to its configured CPU.
    pub rt_pinned: bool,
}

/// A typed query result — the nested `[query_tag, ...fields]` payload of a
/// RESPONSE reply.
#[derive(Debug, Clone, PartialEq)]
// Decoded transiently per reply, never stored in bulk — variant size skew
// (Status/LoopStats vs Ping) is irrelevant here, so no boxing.
#[allow(clippy::large_enum_variant)]
pub enum QueryResult {
    /// PING result.
    Ping {
        /// Whether the hardware bus is connected.
        hardware_connected: bool,
    },
    /// STATUS query result (aggregate snapshot; the broadcast is richer).
    Status {
        /// Flattened 4×4 row-major TCP pose (mm).
        pose: [f64; POSE_ELEMS],
        /// Joint angles (degrees).
        angles: [f64; NUM_JOINTS],
        /// Joint speeds (rad/s).
        speeds: [f64; NUM_JOINTS],
        /// Digital line levels, e-stop last. Variable-length — see
        /// [`crate::IO_SLOTS`].
        io: Vec<u8>,
        /// Tool status, if a tool is selected.
        tool_status: Option<ToolStatusWire>,
    },
    /// ANGLES result (degrees).
    Angles {
        /// Joint angles (degrees).
        angles: [f64; NUM_JOINTS],
    },
    /// POSE result.
    Pose {
        /// Flattened 4×4 row-major TCP pose (mm).
        pose: [f64; POSE_ELEMS],
    },
    /// IO result.
    Io {
        /// Digital line levels, e-stop last. Variable-length — see
        /// [`crate::IO_SLOTS`].
        io: Vec<u8>,
    },
    /// SPEEDS result.
    Speeds {
        /// Joint speeds (rad/s).
        speeds: [f64; NUM_JOINTS],
    },
    /// TOOLS result.
    Tools {
        /// Currently selected tool key (empty = none).
        tool: String,
        /// All registered tool keys.
        available: Vec<String>,
    },
    /// QUEUE result.
    Queue {
        /// Names of queued (not yet executing) commands.
        queue: Vec<String>,
        /// Index currently executing; −1 = none.
        executing_index: i64,
        /// High-water completed index; −1 = none.
        completed_index: i64,
        /// Label of the last checkpoint passed.
        last_checkpoint: String,
        /// Estimated seconds of queued motion.
        queued_duration: f64,
    },
    /// ACTIVITY result.
    Activity {
        /// Name of the current action (empty = idle).
        current: String,
        /// Action state.
        state: ActionState,
        /// Name of the next queued action (empty = none).
        next: String,
        /// Parameter summary of the current action.
        params: String,
    },
    /// LOOP_STATS result.
    LoopStats(LoopStatsResult),
    /// PROFILE result.
    Profile {
        /// Active motion profile name.
        profile: String,
    },
    /// REACHABLE result: per-joint / per-axis enablement flags.
    Reachable {
        /// Per-joint flags `[j1−, j1+, …, j6−, j6+]` (1 = motion allowed).
        joint_en: [u8; EN_SLOTS],
        /// Per-axis WRF flags `[x−, x+, y−, y+, z−, z+, rx−, …]`.
        cart_en_wrf: [u8; EN_SLOTS],
        /// Per-axis TRF flags.
        cart_en_trf: [u8; EN_SLOTS],
    },
    /// ERROR result.
    Error {
        /// The standing error, if any.
        error: Option<WireError>,
    },
    /// TCP_SPEED result.
    TcpSpeed {
        /// TCP linear speed (mm/s).
        speed: f64,
    },
    /// TCP_OFFSET result (mm, tool-local frame).
    TcpOffset {
        /// X offset (mm).
        x: f64,
        /// Y offset (mm).
        y: f64,
        /// Z offset (mm).
        z: f64,
    },
    /// TOOL_STATUS result.
    ToolStatus {
        /// Tool status, if a tool is selected.
        tool_status: Option<ToolStatusWire>,
    },
    /// IS_SIMULATOR result.
    IsSimulator {
        /// Whether the simulator backend is active.
        active: bool,
    },
    /// CONFIG_INFO result: the runtime's effective configuration — the
    /// config-skew hook a UI compares against its packaged mirror.
    ConfigInfo {
        /// Config file path on the daemon host.
        path: String,
        /// Content fingerprint: sha256 hex over the robot TOML and each
        /// gripper TOML (sorted by filename), each hashed as
        /// `filename\n` then content bytes.
        fingerprint: String,
        /// RT tick period \[s\].
        tick_dt_s: f64,
        /// The `[motion]` feel constants, in declaration order:
        /// `jog_l_linear_max_m_s, jog_l_angular_max_rad_s, cart_step_m,
        /// cart_step_rad, move_l_max_joint_step_rad, dls_lambda,
        /// settle_tolerance_rad, settle_timeout_s`.
        motion: [f64; 8],
        /// Per-joint effective EXEC limits: `[soft_min_rad,
        /// soft_max_rad, velocity_rad_s, acceleration_rad_s2]`.
        joints: Vec<[f64; 4]>,
    },
    /// PAYLOAD result: the effective runtime payload (zeros = none).
    Payload {
        /// Payload mass \[kg\].
        mass: f64,
        /// Payload COM in end-effector-frame coordinates \[m\].
        com: [f64; 3],
        /// Rotational inertia about the COM, ee-frame axes,
        /// `(Ixx, Ixy, Iyy, Ixz, Iyz, Izz)` \[kg m²\].
        inertia: [f64; 6],
    },
    /// SHAPES result: the applied collision world by layer.
    Shapes {
        /// Installation-layer shapes (persistent keep-outs).
        installation: Vec<crate::command::Shape>,
        /// Program-layer shapes (set via SET_SHAPES).
        program: Vec<crate::command::Shape>,
        /// Scene epoch this readback represents.
        epoch: u64,
    },
    /// CONFIG_BUNDLE result: the loaded config files verbatim, so a
    /// client can run previews from exactly the daemon's numbers.
    ConfigBundle {
        /// Config file path on the daemon host.
        path: String,
        /// Same content fingerprint as CONFIG_INFO.
        fingerprint: String,
        /// Robot TOML file name (base name, e.g. `PAR6.toml`).
        robot_filename: String,
        /// Robot TOML content.
        robot_toml: String,
        /// Gripper TOMLs as `(file name, content)`, sorted by file name.
        grippers: Vec<(String, String)>,
    },
}

impl QueryResult {
    /// The nested payload tag for this result.
    pub fn tag(&self) -> QueryType {
        use QueryResult as Q;
        match self {
            Q::Ping { .. } => QueryType::Ping,
            Q::Status { .. } => QueryType::Status,
            Q::Angles { .. } => QueryType::Angles,
            Q::Pose { .. } => QueryType::Pose,
            Q::Io { .. } => QueryType::Io,
            Q::Speeds { .. } => QueryType::Speeds,
            Q::Tools { .. } => QueryType::Tools,
            Q::Queue { .. } => QueryType::Queue,
            Q::Activity { .. } => QueryType::Activity,
            Q::LoopStats(_) => QueryType::LoopStats,
            Q::Profile { .. } => QueryType::Profile,
            Q::Reachable { .. } => QueryType::Reachable,
            Q::Error { .. } => QueryType::Error,
            Q::TcpSpeed { .. } => QueryType::TcpSpeed,
            Q::TcpOffset { .. } => QueryType::TcpOffset,
            Q::ToolStatus { .. } => QueryType::ToolStatus,
            Q::IsSimulator { .. } => QueryType::IsSimulator,
            Q::ConfigInfo { .. } => QueryType::ConfigInfo,
            Q::ConfigBundle { .. } => QueryType::ConfigBundle,
            Q::Payload { .. } => QueryType::Payload,
            Q::Shapes { .. } => QueryType::Shapes,
        }
    }
}

/// A decoded server→client message.
#[derive(Debug, Clone, PartialEq)]
// Same rationale as QueryResult: transient decode product, no boxing.
#[allow(clippy::large_enum_variant)]
pub enum Reply {
    /// Ack for SYSTEM (no index) and QUEUED (with index) commands.
    Ok {
        /// Echo of the client's request id.
        req_id: u32,
        /// Queue index for QUEUED commands.
        index: Option<u64>,
    },
    /// Rejection with a structured error.
    Error {
        /// Echo of the client's request id.
        req_id: u32,
        /// The structured error.
        error: WireError,
    },
    /// Query response.
    Response {
        /// Echo of the client's request id.
        req_id: u32,
        /// The typed result payload.
        result: QueryResult,
    },
    /// Unsolicited completion push for a queued command (req_id 0 on the wire).
    Complete {
        /// Queue index of the finished command.
        index: u64,
        /// Whether it finished successfully.
        ok: bool,
        /// Failure detail; present when `ok` is false.
        detail: Option<WireError>,
    },
}

fn w_u8s(buf: &mut Vec<u8>, vs: &[u8]) {
    w_array(buf, vs.len());
    for v in vs {
        w_uint(buf, u64::from(*v));
    }
}

fn w_f64s(buf: &mut Vec<u8>, vs: &[f64]) {
    w_array(buf, vs.len());
    for v in vs {
        w_f64(buf, *v);
    }
}

fn w_opt_tool_status(buf: &mut Vec<u8>, ts: &Option<ToolStatusWire>) {
    match ts {
        Some(ts) => ts.encode(buf),
        None => w_nil(buf),
    }
}

fn w_shapes(buf: &mut Vec<u8>, shapes: &[crate::command::Shape]) {
    w_array(buf, shapes.len());
    for s in shapes {
        crate::command::w_shape(buf, s);
    }
}

fn encode_result(result: &QueryResult, buf: &mut Vec<u8>) {
    use QueryResult as Q;
    let tag = result.tag() as u8;
    match result {
        Q::Ping { hardware_connected } => {
            w_array(buf, 2);
            w_uint(buf, u64::from(tag));
            w_bool(buf, *hardware_connected);
        }
        Q::Status {
            pose,
            angles,
            speeds,
            io,
            tool_status,
        } => {
            w_array(buf, 6);
            w_uint(buf, u64::from(tag));
            w_f64s(buf, pose);
            w_f64s(buf, angles);
            w_f64s(buf, speeds);
            w_u8s(buf, io);
            w_opt_tool_status(buf, tool_status);
        }
        Q::Angles { angles } => {
            w_array(buf, 2);
            w_uint(buf, u64::from(tag));
            w_f64s(buf, angles);
        }
        Q::Pose { pose } => {
            w_array(buf, 2);
            w_uint(buf, u64::from(tag));
            w_f64s(buf, pose);
        }
        Q::Io { io } => {
            w_array(buf, 2);
            w_uint(buf, u64::from(tag));
            w_u8s(buf, io);
        }
        Q::Speeds { speeds } => {
            w_array(buf, 2);
            w_uint(buf, u64::from(tag));
            w_f64s(buf, speeds);
        }
        Q::Tools { tool, available } => {
            w_array(buf, 3);
            w_uint(buf, u64::from(tag));
            w_str(buf, tool);
            w_array(buf, available.len());
            for t in available {
                w_str(buf, t);
            }
        }
        Q::Queue {
            queue,
            executing_index,
            completed_index,
            last_checkpoint,
            queued_duration,
        } => {
            w_array(buf, 6);
            w_uint(buf, u64::from(tag));
            w_array(buf, queue.len());
            for q in queue {
                w_str(buf, q);
            }
            w_int(buf, *executing_index);
            w_int(buf, *completed_index);
            w_str(buf, last_checkpoint);
            w_f64(buf, *queued_duration);
        }
        Q::Activity {
            current,
            state,
            next,
            params,
        } => {
            w_array(buf, 5);
            w_uint(buf, u64::from(tag));
            w_str(buf, current);
            w_uint(buf, u64::from(*state as u8));
            w_str(buf, next);
            w_str(buf, params);
        }
        Q::LoopStats(s) => {
            w_array(buf, 17);
            w_uint(buf, u64::from(tag));
            w_f64(buf, s.target_hz);
            w_uint(buf, s.loop_count);
            w_uint(buf, s.overrun_count);
            w_f64(buf, s.mean_period_s);
            w_f64(buf, s.std_period_s);
            w_f64(buf, s.min_period_s);
            w_f64(buf, s.max_period_s);
            w_f64(buf, s.p95_period_s);
            w_f64(buf, s.p99_period_s);
            w_f64(buf, s.mean_hz);
            w_f64(buf, s.p50_period_s);
            w_f64(buf, s.p90_period_s);
            w_uint(buf, s.can_frame_age_min_ticks);
            w_uint(buf, s.can_frame_age_max_ticks);
            w_bool(buf, s.rt_fifo);
            w_bool(buf, s.rt_pinned);
        }
        Q::Profile { profile } => {
            w_array(buf, 2);
            w_uint(buf, u64::from(tag));
            w_str(buf, profile);
        }
        Q::Reachable {
            joint_en,
            cart_en_wrf,
            cart_en_trf,
        } => {
            w_array(buf, 4);
            w_uint(buf, u64::from(tag));
            w_u8s(buf, joint_en);
            w_u8s(buf, cart_en_wrf);
            w_u8s(buf, cart_en_trf);
        }
        Q::Error { error } => {
            w_array(buf, 2);
            w_uint(buf, u64::from(tag));
            match error {
                Some(e) => e.encode(buf),
                None => w_nil(buf),
            }
        }
        Q::TcpSpeed { speed } => {
            w_array(buf, 2);
            w_uint(buf, u64::from(tag));
            w_f64(buf, *speed);
        }
        Q::TcpOffset { x, y, z } => {
            w_array(buf, 4);
            w_uint(buf, u64::from(tag));
            w_f64(buf, *x);
            w_f64(buf, *y);
            w_f64(buf, *z);
        }
        Q::ToolStatus { tool_status } => {
            w_array(buf, 2);
            w_uint(buf, u64::from(tag));
            w_opt_tool_status(buf, tool_status);
        }
        Q::IsSimulator { active } => {
            w_array(buf, 2);
            w_uint(buf, u64::from(tag));
            w_bool(buf, *active);
        }
        Q::ConfigInfo {
            path,
            fingerprint,
            tick_dt_s,
            motion,
            joints,
        } => {
            w_array(buf, 6);
            w_uint(buf, u64::from(tag));
            w_str(buf, path);
            w_str(buf, fingerprint);
            w_f64(buf, *tick_dt_s);
            w_array(buf, motion.len());
            for v in motion {
                w_f64(buf, *v);
            }
            w_array(buf, joints.len());
            for j in joints {
                w_array(buf, j.len());
                for v in j {
                    w_f64(buf, *v);
                }
            }
        }
        Q::ConfigBundle {
            path,
            fingerprint,
            robot_filename,
            robot_toml,
            grippers,
        } => {
            w_array(buf, 6);
            w_uint(buf, u64::from(tag));
            w_str(buf, path);
            w_str(buf, fingerprint);
            w_str(buf, robot_filename);
            w_str(buf, robot_toml);
            w_array(buf, grippers.len());
            for (name, content) in grippers {
                w_array(buf, 2);
                w_str(buf, name);
                w_str(buf, content);
            }
        }
        Q::Payload { mass, com, inertia } => {
            w_array(buf, 4);
            w_uint(buf, u64::from(tag));
            w_f64(buf, *mass);
            w_array(buf, 3);
            for v in com {
                w_f64(buf, *v);
            }
            w_array(buf, 6);
            for v in inertia {
                w_f64(buf, *v);
            }
        }
        Q::Shapes {
            installation,
            program,
            epoch,
        } => {
            w_array(buf, 4);
            w_uint(buf, u64::from(tag));
            w_shapes(buf, installation);
            w_shapes(buf, program);
            w_uint(buf, *epoch);
        }
    }
}

/// Encode a reply into `buf` (cleared first).
pub fn encode_reply(reply: &Reply, buf: &mut Vec<u8>) {
    buf.clear();
    match reply {
        Reply::Ok { req_id, index } => {
            w_array(buf, if index.is_some() { 3 } else { 2 });
            w_uint(buf, u64::from(MsgType::Ok as u8));
            w_uint(buf, u64::from(*req_id));
            if let Some(i) = index {
                w_uint(buf, *i);
            }
        }
        Reply::Error { req_id, error } => {
            w_array(buf, 3);
            w_uint(buf, u64::from(MsgType::Error as u8));
            w_uint(buf, u64::from(*req_id));
            error.encode(buf);
        }
        Reply::Response { req_id, result } => {
            w_array(buf, 3);
            w_uint(buf, u64::from(MsgType::Response as u8));
            w_uint(buf, u64::from(*req_id));
            encode_result(result, buf);
        }
        Reply::Complete { index, ok, detail } => {
            w_array(buf, if detail.is_some() { 5 } else { 4 });
            w_uint(buf, u64::from(MsgType::Complete as u8));
            w_uint(buf, 0);
            w_uint(buf, *index);
            w_bool(buf, *ok);
            if let Some(d) = detail {
                d.encode(buf);
            }
        }
    }
}

fn r_f64_fixed<const N: usize>(
    r: &mut Reader<'_>,
    what: &'static str,
) -> Result<[f64; N], DecodeError> {
    let n = r.array_len()?;
    if n != N {
        return Err(DecodeError::Arity {
            what,
            expected: N,
            got: n,
        });
    }
    let mut out = [0.0; N];
    for v in &mut out {
        *v = r.f64()?;
    }
    Ok(out)
}

/// Read a variable-length `u8` array, refusing the length header before
/// anything is reserved on its word.
fn r_u8_dyn(r: &mut Reader<'_>, what: &'static str) -> Result<Vec<u8>, DecodeError> {
    let n = r_len(r, what, MAX_IO_SLOTS)?;
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        out.push(
            u8::try_from(r.uint()?).map_err(|_| DecodeError::Validation {
                what,
                why: "exceeds u8".into(),
            })?,
        );
    }
    Ok(out)
}

fn r_u8_fixed<const N: usize>(
    r: &mut Reader<'_>,
    what: &'static str,
) -> Result<[u8; N], DecodeError> {
    let n = r.array_len()?;
    if n != N {
        return Err(DecodeError::Arity {
            what,
            expected: N,
            got: n,
        });
    }
    let mut out = [0u8; N];
    for v in &mut out {
        *v = u8::try_from(r.uint()?).map_err(|_| DecodeError::Validation {
            what,
            why: "exceeds u8".into(),
        })?;
    }
    Ok(out)
}

/// Bound on a reply's string list (tool names, queue entries). The
/// command side has guarded its length headers since `r_len` was written;
/// the reply and status decoders reserved on the sender's word, so an
/// eleven-byte datagram could ask the allocator for a hundred gigabytes
/// and abort the client on `handle_alloc_error`.
const MAX_REPLY_STRINGS: usize = 512;

fn r_strings(r: &mut Reader<'_>) -> Result<Vec<String>, DecodeError> {
    let n = crate::command::r_len(r, "reply string list", MAX_REPLY_STRINGS)?;
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        out.push(r.str()?.to_owned());
    }
    Ok(out)
}

fn r_opt_tool_status(r: &mut Reader<'_>) -> Result<Option<ToolStatusWire>, DecodeError> {
    if r.peek_nil() {
        r.nil()?;
        Ok(None)
    } else {
        Ok(Some(ToolStatusWire::decode(r)?))
    }
}

fn r_shapes(r: &mut Reader<'_>) -> Result<Vec<crate::command::Shape>, DecodeError> {
    let n = crate::command::r_len(r, "reply shapes", crate::command::MAX_SHAPES)?;
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        out.push(crate::command::r_shape(r)?);
    }
    Ok(out)
}

fn expect_arity(what: &'static str, got: usize, expected: usize) -> Result<(), DecodeError> {
    if got != expected {
        return Err(DecodeError::Arity {
            what,
            expected,
            got,
        });
    }
    Ok(())
}

fn decode_result(r: &mut Reader<'_>) -> Result<QueryResult, DecodeError> {
    let n = r.array_len()?;
    if n < 1 {
        return Err(DecodeError::Arity {
            what: "query result",
            expected: 1,
            got: n,
        });
    }
    let raw = r.int()?;
    let tag = QueryType::from_wire(raw).ok_or(DecodeError::UnknownTag(raw))?;
    use QueryType as T;
    let result = match tag {
        T::Ping => {
            expect_arity("ping result", n, 2)?;
            QueryResult::Ping {
                hardware_connected: r.bool()?,
            }
        }
        T::Status => {
            expect_arity("status result", n, 6)?;
            QueryResult::Status {
                pose: r_f64_fixed(r, "status.pose")?,
                angles: r_f64_fixed(r, "status.angles")?,
                speeds: r_f64_fixed(r, "status.speeds")?,
                io: r_u8_dyn(r, "status.io")?,
                tool_status: r_opt_tool_status(r)?,
            }
        }
        T::Angles => {
            expect_arity("angles result", n, 2)?;
            QueryResult::Angles {
                angles: r_f64_fixed(r, "angles")?,
            }
        }
        T::Pose => {
            expect_arity("pose result", n, 2)?;
            QueryResult::Pose {
                pose: r_f64_fixed(r, "pose")?,
            }
        }
        T::Io => {
            expect_arity("io result", n, 2)?;
            QueryResult::Io {
                io: r_u8_dyn(r, "io")?,
            }
        }
        T::Speeds => {
            expect_arity("speeds result", n, 2)?;
            QueryResult::Speeds {
                speeds: r_f64_fixed(r, "speeds")?,
            }
        }
        T::Tools => {
            expect_arity("tools result", n, 3)?;
            QueryResult::Tools {
                tool: r.str()?.to_owned(),
                available: r_strings(r)?,
            }
        }
        T::Queue => {
            expect_arity("queue result", n, 6)?;
            QueryResult::Queue {
                queue: r_strings(r)?,
                executing_index: r.int()?,
                completed_index: r.int()?,
                last_checkpoint: r.str()?.to_owned(),
                queued_duration: r.f64()?,
            }
        }
        T::Activity => {
            expect_arity("activity result", n, 5)?;
            let current = r.str()?.to_owned();
            let raw = r.uint()?;
            let state = ActionState::from_wire(raw as i64).ok_or(DecodeError::InvalidEnum {
                what: "action state",
                value: raw as i64,
            })?;
            QueryResult::Activity {
                current,
                state,
                next: r.str()?.to_owned(),
                params: r.str()?.to_owned(),
            }
        }
        T::LoopStats => {
            expect_arity("loop_stats result", n, 17)?;
            QueryResult::LoopStats(LoopStatsResult {
                target_hz: r.f64()?,
                loop_count: r.uint()?,
                overrun_count: r.uint()?,
                mean_period_s: r.f64()?,
                std_period_s: r.f64()?,
                min_period_s: r.f64()?,
                max_period_s: r.f64()?,
                p95_period_s: r.f64()?,
                p99_period_s: r.f64()?,
                mean_hz: r.f64()?,
                p50_period_s: r.f64()?,
                p90_period_s: r.f64()?,
                can_frame_age_min_ticks: r.uint()?,
                can_frame_age_max_ticks: r.uint()?,
                rt_fifo: r.bool()?,
                rt_pinned: r.bool()?,
            })
        }
        T::Profile => {
            expect_arity("profile result", n, 2)?;
            QueryResult::Profile {
                profile: r.str()?.to_owned(),
            }
        }
        T::Reachable => {
            expect_arity("reachable result", n, 4)?;
            QueryResult::Reachable {
                joint_en: r_u8_fixed(r, "reachable.joint_en")?,
                cart_en_wrf: r_u8_fixed(r, "reachable.cart_en_wrf")?,
                cart_en_trf: r_u8_fixed(r, "reachable.cart_en_trf")?,
            }
        }
        T::Error => {
            expect_arity("error result", n, 2)?;
            QueryResult::Error {
                error: if r.peek_nil() {
                    r.nil()?;
                    None
                } else {
                    Some(WireError::decode(r)?)
                },
            }
        }
        T::TcpSpeed => {
            expect_arity("tcp_speed result", n, 2)?;
            QueryResult::TcpSpeed { speed: r.f64()? }
        }
        T::TcpOffset => {
            expect_arity("tcp_offset result", n, 4)?;
            QueryResult::TcpOffset {
                x: r.f64()?,
                y: r.f64()?,
                z: r.f64()?,
            }
        }
        T::ToolStatus => {
            expect_arity("tool_status result", n, 2)?;
            QueryResult::ToolStatus {
                tool_status: r_opt_tool_status(r)?,
            }
        }
        T::IsSimulator => {
            expect_arity("is_simulator result", n, 2)?;
            QueryResult::IsSimulator { active: r.bool()? }
        }
        T::ConfigInfo => {
            expect_arity("config_info result", n, 6)?;
            let path = r.str()?.to_owned();
            let fingerprint = r.str()?.to_owned();
            let tick_dt_s = r.f64()?;
            let motion = r_f64_fixed(r, "config_info.motion")?;
            let jn = r.array_len()?;
            // One entry per arm joint; anything past the node-slot cap
            // is a corrupt packet, not a big robot.
            if jn > 32 {
                return Err(DecodeError::Arity {
                    what: "config_info.joints",
                    expected: 32,
                    got: jn,
                });
            }
            let mut joints = Vec::with_capacity(jn);
            for _ in 0..jn {
                joints.push(r_f64_fixed(r, "config_info.joints[]")?);
            }
            QueryResult::ConfigInfo {
                path,
                fingerprint,
                tick_dt_s,
                motion,
                joints,
            }
        }
        T::ConfigBundle => {
            expect_arity("config_bundle result", n, 6)?;
            let path = r.str()?.to_owned();
            let fingerprint = r.str()?.to_owned();
            let robot_filename = r.str()?.to_owned();
            let robot_toml = r.str()?.to_owned();
            let gn = r.array_len()?;
            // A config dir holds a handful of gripper files; hundreds is
            // a corrupt packet, not a big deployment.
            if gn > 64 {
                return Err(DecodeError::Arity {
                    what: "config_bundle.grippers",
                    expected: 64,
                    got: gn,
                });
            }
            let mut grippers = Vec::with_capacity(gn);
            for _ in 0..gn {
                expect_arity("config_bundle.grippers[]", r.array_len()?, 2)?;
                grippers.push((r.str()?.to_owned(), r.str()?.to_owned()));
            }
            QueryResult::ConfigBundle {
                path,
                fingerprint,
                robot_filename,
                robot_toml,
                grippers,
            }
        }
        T::Payload => {
            expect_arity("payload result", n, 4)?;
            QueryResult::Payload {
                mass: r.f64()?,
                com: r_f64_fixed(r, "payload.com")?,
                inertia: r_f64_fixed(r, "payload.inertia")?,
            }
        }
        T::Shapes => {
            expect_arity("shapes result", n, 4)?;
            QueryResult::Shapes {
                installation: r_shapes(r)?,
                program: r_shapes(r)?,
                epoch: r.uint()?,
            }
        }
    };
    Ok(result)
}

/// Decode a server→client message (OK / ERROR / RESPONSE / COMPLETE).
///
/// STATUS broadcasts use [`crate::status::decode_status`] and CHUNK envelopes
/// [`crate::chunk::decode_chunk`]; both are rejected here.
pub fn decode_reply(data: &[u8]) -> Result<Reply, DecodeError> {
    let mut r = Reader::new(data);
    let n = r.array_len()?;
    if n < 2 {
        return Err(DecodeError::Arity {
            what: "reply envelope",
            expected: 2,
            got: n,
        });
    }
    let raw = r.int()?;
    let tag = MsgType::from_wire(raw).ok_or(DecodeError::UnknownTag(raw))?;
    let req_id = u32::try_from(r.uint()?).map_err(|_| DecodeError::Validation {
        what: "req_id",
        why: "exceeds u32".into(),
    })?;
    let reply = match tag {
        MsgType::Ok => {
            if n > 3 {
                return Err(DecodeError::Arity {
                    what: "OK reply",
                    expected: 3,
                    got: n,
                });
            }
            Reply::Ok {
                req_id,
                index: if n == 3 { Some(r.uint()?) } else { None },
            }
        }
        MsgType::Error => {
            expect_arity("ERROR reply", n, 3)?;
            Reply::Error {
                req_id,
                error: WireError::decode(&mut r)?,
            }
        }
        MsgType::Response => {
            expect_arity("RESPONSE reply", n, 3)?;
            Reply::Response {
                req_id,
                result: decode_result(&mut r)?,
            }
        }
        MsgType::Complete => {
            if req_id != 0 {
                return Err(DecodeError::Validation {
                    what: "COMPLETE push",
                    why: "req_id must be 0".into(),
                });
            }
            if n != 4 && n != 5 {
                return Err(DecodeError::Arity {
                    what: "COMPLETE push",
                    expected: 4,
                    got: n,
                });
            }
            Reply::Complete {
                index: r.uint()?,
                ok: r.bool()?,
                detail: if n == 5 {
                    if r.peek_nil() {
                        r.nil()?;
                        None
                    } else {
                        Some(WireError::decode(&mut r)?)
                    }
                } else {
                    None
                },
            }
        }
        MsgType::Status | MsgType::Chunk => {
            return Err(DecodeError::UnknownTag(raw));
        }
    };
    r.finish()?;
    Ok(reply)
}
