//! The broadcast STATUS packet: v2 header + parol6-inherited body.
//!
//! Wire form (one positional array, header-first, no `req_id`):
//!
//! ```text
//! [STATUS, proto_version u8, controller_id u32, seq u64, mono_time_ns u64,
//!  link_ok u8, data_age_ms u16,
//!  pose f64[16], angles f64[6], speeds f64[6], io u8[] (estop last),
//!  action_current str, action_state u8, joint_en u8[12], cart_en_wrf u8[12],
//!  cart_en_trf u8[12], executing_index i64, completed_index i64,
//!  last_checkpoint str, error nil|err6, queued_segments u32,
//!  queued_duration f64, action_params str, tool_status nil|ts8,
//!  tcp_speed f64, simulator_active bool, collision_active bool,
//!  collision_pairs [[str,str]], scene_epoch u64, accepted_index i64,
//!  homed bool, torques f64[6], mode u8, enabled bool, gravity_comp bool,
//!  warnings [err6], link_health [state u8, restarts u32, tx_errors u64,
//!  rx_frames u64], homing [active bool, step u8, [[status u8, phase u8]]],
//!  torques_ext f64[6],
//!  drive_health [temperatures_c f64[], currents_ma f64[], bus_voltage_v nil|f64],
//!  loop_health [p99_period_s f64, overruns u64]]
//! ```
//!
//! 41 elements total. STATUS is broadcast even when the bus link is down —
//! `link_ok`/`data_age_ms` report staleness instead of going silent. Decoders
//! must tolerate a LONGER array (future fields append at the tail) but never a
//! shorter one.

use crate::command::r_len;
use crate::enums::{ActionState, ControllerMode, MsgType};
use crate::error::WireError;
use crate::reply::ToolStatusWire;
use crate::wire::{w_array, w_bool, w_f64, w_int, w_nil, w_str, w_uint, Reader};
use crate::{DecodeError, EN_SLOTS, IO_SLOTS, MAX_IO_SLOTS, NUM_JOINTS, POSE_ELEMS, PROTO_VERSION};
use crate::{HomingJointState, HomingPhase, LinkState};

/// Total number of elements in a v2 STATUS array (including the tag).
pub const STATUS_LEN: usize = 41;
/// Decode cap on the `warnings` list (the RT latch holds at most 32
/// entries; a longer claim is hostile input).
const MAX_WARNINGS: usize = 64;
/// Decode cap on the per-node `homing.joints` list.
const MAX_NODE_SLOTS: usize = 32;
/// Number of header elements (tag through `data_age_ms`).
pub const STATUS_HEADER_LEN: usize = 7;

/// One decoded STATUS packet. Field order mirrors the wire exactly.
#[derive(Debug, Clone, PartialEq)]
pub struct Status {
    /// Protocol version of the sender (2 for this codec).
    pub proto_version: u8,
    /// Stable id of the sending controller (disambiguates multicast).
    pub controller_id: u32,
    /// Monotonic broadcast sequence number (detects loss/reorder).
    pub seq: u64,
    /// Sender monotonic clock at snapshot time (nanoseconds).
    pub mono_time_ns: u64,
    /// 1 while the motor bus link is healthy; 0 when the data is stale.
    pub link_ok: u8,
    /// Age of the underlying bus data (ms, saturating).
    pub data_age_ms: u16,
    /// Flattened 4×4 row-major TCP pose (mm).
    pub pose: [f64; POSE_ELEMS],
    /// Joint angles (degrees).
    pub angles: [f64; NUM_JOINTS],
    /// Joint speeds (rad/s).
    pub speeds: [f64; NUM_JOINTS],
    /// Digital line levels, in the order the runtime's config declares
    /// them, with the e-stop last. Variable-length: a control box with
    /// ten lines publishes ten of them plus the e-stop, and a consumer
    /// sizes from `io.len()` rather than from [`IO_SLOTS`].
    pub io: Vec<u8>,
    /// Name of the current action (empty = idle).
    pub action_current: String,
    /// Action state.
    pub action_state: ActionState,
    /// Per-joint enablement flags (1 = motion allowed in that direction).
    pub joint_en: [u8; EN_SLOTS],
    /// Per-axis WRF enablement flags.
    pub cart_en_wrf: [u8; EN_SLOTS],
    /// Per-axis TRF enablement flags.
    pub cart_en_trf: [u8; EN_SLOTS],
    /// Index currently executing; −1 = none.
    pub executing_index: i64,
    /// High-water completed index (blended-away commands report the max of
    /// consumed indexes); −1 = none.
    pub completed_index: i64,
    /// Label of the last checkpoint passed.
    pub last_checkpoint: String,
    /// Standing error, if any.
    pub error: Option<WireError>,
    /// Number of queued (not yet executing) segments.
    pub queued_segments: u32,
    /// Estimated seconds of queued motion.
    pub queued_duration: f64,
    /// Parameter summary of the current action.
    pub action_params: String,
    /// Tool status (includes `variant_key`, new in v2), if a tool is selected.
    pub tool_status: Option<ToolStatusWire>,
    /// TCP linear speed (mm/s).
    pub tcp_speed: f64,
    /// Whether the simulator backend is active.
    pub simulator_active: bool,
    /// Whether any collision pair is currently active.
    pub collision_active: bool,
    /// Names of the colliding body pairs.
    pub collision_pairs: Vec<(String, String)>,
    /// Epoch of the applied collision world.
    pub scene_epoch: u64,
    /// Last command index the server accepted; −1 = none yet. Lets waiters
    /// order a standing error against their own command's acceptance.
    pub accepted_index: i64,
    /// All joints homed.
    pub homed: bool,
    /// Measured joint torque \[Nm\], kt-calibrated and filtered.
    pub torques: [f64; NUM_JOINTS],
    /// Controller mode. `par6-server` maps the RT core's own `Mode` onto
    /// this with an exhaustive match, so the wire vocabulary is owned here.
    pub mode: ControllerMode,
    /// Drive authority is granted (the RT's `ArmState::Enabled`).
    pub enabled: bool,
    /// The gravity feedforward is being applied. With `mode` IDLE and the
    /// arm homed and enabled, this is exactly the freedrive condition:
    /// torque-only G(q) with no position hold. A configured `[freedrive]
    /// drift_lock` holds the pose with the drive's impedance frame plus a
    /// clamped integral once the arm has been still, released by any
    /// motion; the integral shows up as the difference between the
    /// commanded and gravity torques in telemetry.
    pub gravity_comp: bool,
    /// Warning-class latch entries (self-clearing conditions: stale CAN
    /// data, degraded loop, failed homing, …). The standing `error` slot
    /// carries only HARD latches; these are the rest of the RT's list.
    pub warnings: Vec<WireError>,
    /// Motor-bus link health.
    pub link_health: LinkHealthWire,
    /// Homing progress.
    pub homing: HomingWire,
    /// External joint torque estimate \[Nm\]: filtered measured torque
    /// minus the model's gravity torque.
    pub torques_ext: [f64; NUM_JOINTS],
    /// Per-drive analog readings.
    pub drive_health: DriveHealthWire,
    /// Control-loop health.
    pub loop_health: LoopHealthWire,
}

/// Per-drive analog readings as STATUS carries them (slot 39).
///
/// These are the trends that let an operator watch a joint climb toward a
/// limit before it faults; the faults themselves ride `warnings` and the
/// standing `error`, and are deliberately not duplicated here. Readings
/// are per node, arm joints first and the tool drive last, matching
/// `homing.joints`; `NaN` marks a node that has not answered yet.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct DriveHealthWire {
    /// Per-node driver temperature \[°C\].
    pub temperatures_c: Vec<f64>,
    /// Per-node motor current \[mA\].
    pub currents_ma: Vec<f64>,
    /// Lowest supply voltage any node reports \[V\] — sag under load shows
    /// up at the loaded drive, so the minimum is the diagnostic one.
    /// `None` when no node has reported one.
    pub bus_voltage_v: Option<f64>,
}

/// Control-loop health as STATUS carries it (slot 40).
///
/// The tail and the miss count, which are what say a loop is unwell; the
/// mean, the rest of the distribution and the boot constants (target rate,
/// scheduling policy) stay in the `LOOP_STATS` query, where a display reads
/// them once instead of every tick.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LoopHealthWire {
    /// 99th-percentile tick period \[s\] over the runtime's window.
    pub p99_period_s: f64,
    /// Ticks that missed their deadline, cumulative.
    pub overruns: u64,
}

/// Motor-bus link health as STATUS carries it (slot 36).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LinkHealthWire {
    /// Kernel link state — a [`LinkState`] value.
    pub state: u8,
    /// Interface restarts observed.
    pub restarts: u32,
    /// TX errors observed.
    pub tx_errors: u64,
    /// Total frames received.
    pub rx_frames: u64,
}

/// Homing progress as STATUS carries it (slot 37).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct HomingWire {
    /// Whether the homing sequence is running.
    pub active: bool,
    /// Current sequence step (0-based); meaningful while `active`.
    pub sequence_step: u8,
    /// Per-actuator `[status, phase]` — a [`HomingJointState`] and a
    /// [`HomingPhase`] value — arm joints first, gripper last.
    pub joints: Vec<(u8, u8)>,
}

impl Default for Status {
    fn default() -> Self {
        Status {
            proto_version: PROTO_VERSION,
            controller_id: 0,
            seq: 0,
            mono_time_ns: 0,
            link_ok: 0,
            data_age_ms: 0,
            pose: [0.0; POSE_ELEMS],
            angles: [0.0; NUM_JOINTS],
            speeds: [0.0; NUM_JOINTS],
            io: vec![0; IO_SLOTS],
            action_current: String::new(),
            action_state: ActionState::Idle,
            joint_en: [1; EN_SLOTS],
            cart_en_wrf: [1; EN_SLOTS],
            cart_en_trf: [1; EN_SLOTS],
            executing_index: -1,
            completed_index: -1,
            last_checkpoint: String::new(),
            error: None,
            queued_segments: 0,
            queued_duration: 0.0,
            action_params: String::new(),
            tool_status: None,
            tcp_speed: 0.0,
            simulator_active: false,
            collision_active: false,
            collision_pairs: Vec::new(),
            scene_epoch: 0,
            accepted_index: -1,
            homed: false,
            torques: [0.0; NUM_JOINTS],
            mode: ControllerMode::Booting,
            enabled: false,
            gravity_comp: false,
            warnings: Vec::new(),
            link_health: LinkHealthWire::default(),
            homing: HomingWire::default(),
            torques_ext: [0.0; NUM_JOINTS],
            drive_health: DriveHealthWire::default(),
            loop_health: LoopHealthWire::default(),
        }
    }
}

/// Encode a STATUS packet into `buf` (cleared first, capacity retained).
///
/// This is the broadcast hot path: the only allocation is growth of the
/// caller's reusable buffer.
pub fn encode_status_into(s: &Status, buf: &mut Vec<u8>) {
    buf.clear();
    w_array(buf, STATUS_LEN);
    w_uint(buf, u64::from(MsgType::Status as u8));
    w_uint(buf, u64::from(s.proto_version));
    w_uint(buf, u64::from(s.controller_id));
    w_uint(buf, s.seq);
    w_uint(buf, s.mono_time_ns);
    w_uint(buf, u64::from(s.link_ok));
    w_uint(buf, u64::from(s.data_age_ms));
    w_array(buf, POSE_ELEMS);
    for v in &s.pose {
        w_f64(buf, *v);
    }
    w_array(buf, NUM_JOINTS);
    for v in &s.angles {
        w_f64(buf, *v);
    }
    w_array(buf, NUM_JOINTS);
    for v in &s.speeds {
        w_f64(buf, *v);
    }
    w_array(buf, s.io.len());
    for v in &s.io {
        w_uint(buf, u64::from(*v));
    }
    w_str(buf, &s.action_current);
    w_uint(buf, u64::from(s.action_state as u8));
    for arr in [&s.joint_en, &s.cart_en_wrf, &s.cart_en_trf] {
        w_array(buf, EN_SLOTS);
        for v in arr.iter() {
            w_uint(buf, u64::from(*v));
        }
    }
    w_int(buf, s.executing_index);
    w_int(buf, s.completed_index);
    w_str(buf, &s.last_checkpoint);
    match &s.error {
        Some(e) => e.encode(buf),
        None => w_nil(buf),
    }
    w_uint(buf, u64::from(s.queued_segments));
    w_f64(buf, s.queued_duration);
    w_str(buf, &s.action_params);
    match &s.tool_status {
        Some(ts) => ts.encode(buf),
        None => w_nil(buf),
    }
    w_f64(buf, s.tcp_speed);
    w_bool(buf, s.simulator_active);
    w_bool(buf, s.collision_active);
    w_array(buf, s.collision_pairs.len());
    for (a, b) in &s.collision_pairs {
        w_array(buf, 2);
        w_str(buf, a);
        w_str(buf, b);
    }
    w_uint(buf, s.scene_epoch);
    w_int(buf, s.accepted_index);
    w_bool(buf, s.homed);
    w_array(buf, NUM_JOINTS);
    for v in s.torques {
        w_f64(buf, v);
    }
    w_uint(buf, u64::from(s.mode as u8));
    w_bool(buf, s.enabled);
    w_bool(buf, s.gravity_comp);
    w_array(buf, s.warnings.len());
    for w in &s.warnings {
        w.encode(buf);
    }
    w_array(buf, 4);
    w_uint(buf, u64::from(s.link_health.state));
    w_uint(buf, u64::from(s.link_health.restarts));
    w_uint(buf, s.link_health.tx_errors);
    w_uint(buf, s.link_health.rx_frames);
    w_array(buf, 3);
    w_bool(buf, s.homing.active);
    w_uint(buf, u64::from(s.homing.sequence_step));
    w_array(buf, s.homing.joints.len());
    for (status, phase) in &s.homing.joints {
        w_array(buf, 2);
        w_uint(buf, u64::from(*status));
        w_uint(buf, u64::from(*phase));
    }
    w_array(buf, NUM_JOINTS);
    for v in s.torques_ext {
        w_f64(buf, v);
    }
    w_array(buf, 3);
    w_array(buf, s.drive_health.temperatures_c.len());
    for v in &s.drive_health.temperatures_c {
        w_f64(buf, *v);
    }
    w_array(buf, s.drive_health.currents_ma.len());
    for v in &s.drive_health.currents_ma {
        w_f64(buf, *v);
    }
    match s.drive_health.bus_voltage_v {
        Some(v) => w_f64(buf, v),
        None => w_nil(buf),
    }
    w_array(buf, 2);
    w_f64(buf, s.loop_health.p99_period_s);
    w_uint(buf, s.loop_health.overruns);
}

/// Reusable STATUS encoder: owns the broadcast buffer so the hot path
/// allocates nothing after warm-up.
#[derive(Debug, Default)]
pub struct StatusEncoder {
    buf: Vec<u8>,
}

impl StatusEncoder {
    /// New encoder with a pre-sized buffer (a full packet is ~1 KiB).
    pub fn new() -> Self {
        StatusEncoder {
            buf: Vec::with_capacity(2048),
        }
    }

    /// Encode `s`, returning the packet bytes (valid until the next call).
    pub fn encode(&mut self, s: &Status) -> &[u8] {
        encode_status_into(s, &mut self.buf);
        &self.buf
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

/// Read the variable-length `io` array, bounded before anything is
/// reserved on the length header's word.
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

/// Decode a STATUS packet.
///
/// Requires all [`STATUS_LEN`] v2 elements; any elements beyond them are
/// skipped (forward compatibility with newer producers).
pub fn decode_status(data: &[u8]) -> Result<Status, DecodeError> {
    let mut r = Reader::new(data);
    let n = r.array_len()?;
    if n < STATUS_LEN {
        return Err(DecodeError::Arity {
            what: "STATUS packet",
            expected: STATUS_LEN,
            got: n,
        });
    }
    let raw = r.int()?;
    if raw != MsgType::Status as i64 {
        return Err(DecodeError::UnknownTag(raw));
    }
    let proto_version = u8::try_from(r.uint()?).map_err(|_| DecodeError::Validation {
        what: "status.proto_version",
        why: "exceeds u8".into(),
    })?;
    let controller_id = u32::try_from(r.uint()?).map_err(|_| DecodeError::Validation {
        what: "status.controller_id",
        why: "exceeds u32".into(),
    })?;
    let seq = r.uint()?;
    let mono_time_ns = r.uint()?;
    let link_ok = u8::try_from(r.uint()?).map_err(|_| DecodeError::Validation {
        what: "status.link_ok",
        why: "exceeds u8".into(),
    })?;
    let data_age_ms = u16::try_from(r.uint()?).map_err(|_| DecodeError::Validation {
        what: "status.data_age_ms",
        why: "exceeds u16".into(),
    })?;
    let pose = r_f64_fixed(&mut r, "status.pose")?;
    let angles = r_f64_fixed(&mut r, "status.angles")?;
    let speeds = r_f64_fixed(&mut r, "status.speeds")?;
    let io = r_u8_dyn(&mut r, "status.io")?;
    let action_current = r.str()?.to_owned();
    let raw_state = r.uint()?;
    let action_state =
        ActionState::from_wire(raw_state as i64).ok_or(DecodeError::InvalidEnum {
            what: "action state",
            value: raw_state as i64,
        })?;
    let joint_en = r_u8_fixed(&mut r, "status.joint_en")?;
    let cart_en_wrf = r_u8_fixed(&mut r, "status.cart_en_wrf")?;
    let cart_en_trf = r_u8_fixed(&mut r, "status.cart_en_trf")?;
    let executing_index = r.int()?;
    let completed_index = r.int()?;
    let last_checkpoint = r.str()?.to_owned();
    let error = if r.peek_nil() {
        r.nil()?;
        None
    } else {
        Some(WireError::decode(&mut r)?)
    };
    let queued_segments = u32::try_from(r.uint()?).map_err(|_| DecodeError::Validation {
        what: "status.queued_segments",
        why: "exceeds u32".into(),
    })?;
    let queued_duration = r.f64()?;
    let action_params = r.str()?.to_owned();
    let tool_status = if r.peek_nil() {
        r.nil()?;
        None
    } else {
        Some(ToolStatusWire::decode(&mut r)?)
    };
    let tcp_speed = r.f64()?;
    let simulator_active = r.bool()?;
    let collision_active = r.bool()?;
    // STATUS is a multicast broadcast any host on the segment can send,
    // so its length headers are as untrusted as a command's. The world is
    // capped at MAX_SHAPES geometries, which bounds the pairs they form.
    let n_pairs = crate::command::r_len(
        &mut r,
        "collision pairs",
        crate::command::MAX_SHAPES * crate::command::MAX_SHAPES,
    )?;
    let mut collision_pairs = Vec::with_capacity(n_pairs);
    for _ in 0..n_pairs {
        let pn = r.array_len()?;
        if pn != 2 {
            return Err(DecodeError::Arity {
                what: "collision pair",
                expected: 2,
                got: pn,
            });
        }
        collision_pairs.push((r.str()?.to_owned(), r.str()?.to_owned()));
    }
    let scene_epoch = r.uint()?;
    let accepted_index = r.int()?;
    let homed = r.bool()?;
    let torques: [f64; NUM_JOINTS] = r_f64_fixed(&mut r, "status.torques")?;
    let raw_mode = r.uint()? as i64;
    let mode = ControllerMode::from_wire(raw_mode).ok_or(DecodeError::InvalidEnum {
        what: "controller mode",
        value: raw_mode,
    })?;
    let enabled = r.bool()?;
    let gravity_comp = r.bool()?;
    let n_warnings = r_len(&mut r, "status.warnings", MAX_WARNINGS)?;
    let mut warnings = Vec::with_capacity(n_warnings);
    for _ in 0..n_warnings {
        warnings.push(WireError::decode(&mut r)?);
    }
    let lh = r.array_len()?;
    if lh != 4 {
        return Err(DecodeError::Arity {
            what: "status.link_health",
            expected: 4,
            got: lh,
        });
    }
    let link_state = u8::try_from(r.uint()?).map_err(|_| DecodeError::Validation {
        what: "status.link_health.state",
        why: "exceeds u8".into(),
    })?;
    LinkState::from_wire(i64::from(link_state)).ok_or(DecodeError::InvalidEnum {
        what: "link state",
        value: i64::from(link_state),
    })?;
    let link_health = LinkHealthWire {
        state: link_state,
        restarts: u32::try_from(r.uint()?).map_err(|_| DecodeError::Validation {
            what: "status.link_health.restarts",
            why: "exceeds u32".into(),
        })?,
        tx_errors: r.uint()?,
        rx_frames: r.uint()?,
    };
    let hn = r.array_len()?;
    if hn != 3 {
        return Err(DecodeError::Arity {
            what: "status.homing",
            expected: 3,
            got: hn,
        });
    }
    let homing_active = r.bool()?;
    let sequence_step = u8::try_from(r.uint()?).map_err(|_| DecodeError::Validation {
        what: "status.homing.sequence_step",
        why: "exceeds u8".into(),
    })?;
    let n_hj = r_len(&mut r, "status.homing.joints", MAX_NODE_SLOTS)?;
    let mut homing_joints = Vec::with_capacity(n_hj);
    for _ in 0..n_hj {
        let pn = r.array_len()?;
        if pn != 2 {
            return Err(DecodeError::Arity {
                what: "status.homing joint",
                expected: 2,
                got: pn,
            });
        }
        let status = u8::try_from(r.uint()?).map_err(|_| DecodeError::Validation {
            what: "status.homing status",
            why: "exceeds u8".into(),
        })?;
        HomingJointState::from_wire(i64::from(status)).ok_or(DecodeError::InvalidEnum {
            what: "homing status",
            value: i64::from(status),
        })?;
        let phase = u8::try_from(r.uint()?).map_err(|_| DecodeError::Validation {
            what: "status.homing phase",
            why: "exceeds u8".into(),
        })?;
        HomingPhase::from_wire(i64::from(phase)).ok_or(DecodeError::InvalidEnum {
            what: "homing phase",
            value: i64::from(phase),
        })?;
        homing_joints.push((status, phase));
    }
    let homing = HomingWire {
        active: homing_active,
        sequence_step,
        joints: homing_joints,
    };
    let torques_ext: [f64; NUM_JOINTS] = r_f64_fixed(&mut r, "status.torques_ext")?;
    let dn = r.array_len()?;
    if dn != 3 {
        return Err(DecodeError::Arity {
            what: "status.drive_health",
            expected: 3,
            got: dn,
        });
    }
    let n_temps = r_len(&mut r, "status.drive_health.temperatures_c", MAX_NODE_SLOTS)?;
    let mut temperatures_c = Vec::with_capacity(n_temps);
    for _ in 0..n_temps {
        temperatures_c.push(r.f64()?);
    }
    let n_curr = r_len(&mut r, "status.drive_health.currents_ma", MAX_NODE_SLOTS)?;
    let mut currents_ma = Vec::with_capacity(n_curr);
    for _ in 0..n_curr {
        currents_ma.push(r.f64()?);
    }
    let drive_health = DriveHealthWire {
        temperatures_c,
        currents_ma,
        bus_voltage_v: if r.peek_nil() {
            r.nil()?;
            None
        } else {
            Some(r.f64()?)
        },
    };
    let ln = r.array_len()?;
    if ln != 2 {
        return Err(DecodeError::Arity {
            what: "status.loop_health",
            expected: 2,
            got: ln,
        });
    }
    let loop_health = LoopHealthWire {
        p99_period_s: r.f64()?,
        overruns: r.uint()?,
    };

    // Forward compatibility: skip any fields a newer producer appended.
    for _ in STATUS_LEN..n {
        r.skip_value()?;
    }
    r.finish()?;

    Ok(Status {
        proto_version,
        controller_id,
        seq,
        mono_time_ns,
        link_ok,
        data_age_ms,
        pose,
        angles,
        speeds,
        io,
        action_current,
        action_state,
        joint_en,
        cart_en_wrf,
        cart_en_trf,
        executing_index,
        completed_index,
        last_checkpoint,
        error,
        queued_segments,
        queued_duration,
        action_params,
        tool_status,
        tcp_speed,
        simulator_active,
        collision_active,
        collision_pairs,
        scene_epoch,
        accepted_index,
        homed,
        torques,
        mode,
        enabled,
        gravity_comp,
        warnings,
        link_health,
        homing,
        torques_ext,
        drive_health,
        loop_health,
    })
}
