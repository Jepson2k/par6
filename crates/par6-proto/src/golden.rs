//! Golden-vector definitions: the cross-language conformance suite.
//!
//! Each [`Vector`] is one `.bin` msgpack file under `tests/golden/protocol/`
//! plus its manifest entry. The manifest's `wire` arrays are produced by
//! decoding the encoded bytes back to JSON, so manifest and bytes cannot
//! drift; the Python tests re-encode `wire` with an independent packer
//! and byte-compare against the Rust encoder's output.
//!
//! Regenerate the committed files with
//! `cargo run -p par6-proto --bin gen_golden`; `cargo test -p par6-proto`
//! fails if they are stale.

use serde_json::{json, Value};

use crate::chunk::{encode_chunk, split_into_chunks, Chunk};
use crate::command::{
    encode_command, Checkpoint, Command, ConnectHardware, Delay, EnterFlashing, Home, JogJ, JogL,
    MoveC, MoveJ, MoveJPose, MoveL, MoveP, MoveS, PoseQuery, SelectProfile, SelectTool, ServoJ,
    ServoJPose, ServoL, SetCompletionPolicy, SetPayload, SetPidGains, SetRecipe, SetShapes,
    SetTcpOffset, Shape, Simulator, Stop, Teleport, ToolAction, ToolParam, WriteIo,
};
use crate::enums::{
    command_class, ActionState, CompletionPolicy, ControllerMode, FlashingAssertion, Frame,
    ToolState,
};
use crate::error::{make_error, ErrorCode, UNATTRIBUTED};
use crate::pygen;
use crate::reply::{encode_reply, LoopStatsResult, QueryResult, Reply, ToolStatusWire};
use crate::status::{encode_status_into, Status, STATUS_LEN};
use crate::wire::{w_array, w_f64, w_nil, w_str, w_uint, Reader};
use crate::{DecodeError, PROTO_VERSION};

/// What a conformance test should do with a vector's bytes.
#[derive(Debug, Clone)]
pub enum Check {
    /// Encode must reproduce the bytes; decode must yield `(req_id, cmd)`.
    Command {
        /// Expected request id.
        req_id: u32,
        /// Expected decoded command.
        cmd: Command,
    },
    /// Encode must reproduce the bytes; decode must yield the reply.
    Reply(Reply),
    /// Decode must yield the status; encode must reproduce the bytes unless
    /// `decode_only` (used for the longer-tail forward-compat vector).
    Status {
        /// Expected decoded status.
        status: Box<Status>,
        /// Skip the encode comparison (bytes deliberately not v2-canonical).
        decode_only: bool,
    },
    /// Encode must reproduce the bytes; decode must yield the chunk.
    Chunk(Box<Chunk>),
    /// `decode_command` must fail.
    MalformedCommand,
    /// `decode_reply` must fail.
    MalformedReply,
    /// `decode_status` must fail.
    MalformedStatus,
}

/// One golden vector: a `.bin` file plus its manifest entry and check.
#[derive(Debug, Clone)]
pub struct Vector {
    /// File stem (`<name>.bin`).
    pub name: &'static str,
    /// The frozen bytes.
    pub bytes: Vec<u8>,
    /// The manifest entry (without the `file` key, added by [`manifest_json`]).
    pub manifest: Value,
    /// What conformance tests assert about the bytes.
    pub check: Check,
}

// ---------------------------------------------------------------------------
// msgpack → JSON (manifest `wire` arrays)
// ---------------------------------------------------------------------------

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

fn mp_value(r: &mut Reader<'_>) -> Result<Value, DecodeError> {
    let m = r.peek_marker()?;
    Ok(match m {
        0xC0 => {
            r.nil()?;
            Value::Null
        }
        0xC2 | 0xC3 => Value::Bool(r.bool()?),
        0xCB => {
            let f = r.f64()?;
            Value::Number(
                serde_json::Number::from_f64(f)
                    .expect("golden vectors contain no NaN in wire arrays"),
            )
        }
        0x00..=0x7F | 0xCC..=0xCF => Value::Number(r.uint()?.into()),
        0xE0..=0xFF | 0xD0..=0xD3 => Value::Number(r.int()?.into()),
        0xA0..=0xBF | 0xD9..=0xDB => Value::String(r.str()?.to_owned()),
        0xC4..=0xC6 => json!({ "__bin__": hex(r.bin()?) }),
        0x90..=0x9F | 0xDC | 0xDD => {
            let n = r.array_len()?;
            let mut out = Vec::with_capacity(n);
            for _ in 0..n {
                out.push(mp_value(r)?);
            }
            Value::Array(out)
        }
        other => {
            return Err(DecodeError::Type {
                expected: "json-representable value",
                found: other,
                pos: 0,
            });
        }
    })
}

/// Decode a whole msgpack payload into the manifest's JSON `wire` form.
fn wire_json(bytes: &[u8]) -> Value {
    let mut r = Reader::new(bytes);
    let v = mp_value(&mut r).expect("golden bytes decode to JSON");
    r.finish().expect("golden bytes have no trailing garbage");
    v
}

// ---------------------------------------------------------------------------
// Vector constructors
// ---------------------------------------------------------------------------

fn cmd_vec(name: &'static str, req_id: u32, cmd: Command) -> Vector {
    let mut bytes = Vec::new();
    encode_command(&cmd, req_id, &mut bytes).expect("golden commands are valid");
    let tag = cmd.tag();
    let class = command_class(tag);
    let manifest = json!({
        "name": name,
        "kind": "command",
        "cmd": upper_snake(&format!("{tag:?}")),
        "tag": tag as u16,
        "class": upper_snake(&format!("{class:?}")),
        "req_id": req_id,
        "idempotency_key": cmd.idempotency_key(),
        "wire": wire_json(&bytes),
    });
    Vector {
        name,
        bytes,
        manifest,
        check: Check::Command { req_id, cmd },
    }
}

fn reply_vec(name: &'static str, reply: Reply) -> Vector {
    let mut bytes = Vec::new();
    encode_reply(&reply, &mut bytes);
    let msg = match &reply {
        Reply::Ok { .. } => "OK".to_owned(),
        Reply::Error { .. } => "ERROR".to_owned(),
        Reply::Response { result, .. } => {
            format!("RESPONSE/{}", upper_snake(&format!("{:?}", result.tag())))
        }
        Reply::Complete { .. } => "COMPLETE".to_owned(),
    };
    let manifest = json!({
        "name": name,
        "kind": "reply",
        "msg": msg,
        "wire": wire_json(&bytes),
    });
    Vector {
        name,
        bytes,
        manifest,
        check: Check::Reply(reply),
    }
}

fn status_vec(name: &'static str, status: Status) -> Vector {
    let mut bytes = Vec::new();
    encode_status_into(&status, &mut bytes);
    let manifest = json!({
        "name": name,
        "kind": "status",
        "wire": wire_json(&bytes),
        "fields": status_fields(&status),
    });
    Vector {
        name,
        bytes,
        manifest,
        check: Check::Status {
            status: Box::new(status),
            decode_only: false,
        },
    }
}

fn malformed_vec(name: &'static str, target: &str, reason: &str, bytes: Vec<u8>) -> Vector {
    let check = match target {
        "command" => Check::MalformedCommand,
        "reply" => Check::MalformedReply,
        "status" => Check::MalformedStatus,
        _ => unreachable!("unknown malformed target"),
    };
    let manifest = json!({
        "name": name,
        "kind": "malformed",
        "target": target,
        "reason": reason,
    });
    Vector {
        name,
        bytes,
        manifest,
        check,
    }
}

fn upper_snake(name: &str) -> String {
    let mut out = String::new();
    for (i, c) in name.chars().enumerate() {
        if c.is_ascii_uppercase() && i > 0 {
            out.push('_');
        }
        out.push(c.to_ascii_uppercase());
    }
    out
}

fn error_json(e: &crate::error::WireError) -> Value {
    json!([
        e.command_index,
        e.code,
        e.title,
        e.cause,
        e.effect,
        e.remedy
    ])
}

fn tool_status_json(ts: &ToolStatusWire) -> Value {
    json!([
        ts.key,
        ts.state as u8,
        ts.engaged,
        ts.part_detected,
        ts.fault_code,
        ts.positions,
        ts.channels,
        ts.variant_key,
    ])
}

fn status_fields(s: &Status) -> Value {
    let warnings: Vec<Value> = s.warnings.iter().map(error_json).collect();
    let lh = &s.link_health;
    let link_health = json!([lh.state, lh.restarts, lh.tx_errors, lh.rx_frames]);
    let homing_joints: Vec<Vec<u8>> = s.homing.joints.iter().map(|(a, b)| vec![*a, *b]).collect();
    let homing = json!([s.homing.active, s.homing.sequence_step, homing_joints]);
    json!({
        "proto_version": s.proto_version,
        "controller_id": s.controller_id,
        "seq": s.seq,
        "mono_time_ns": s.mono_time_ns,
        "link_ok": s.link_ok,
        "data_age_ms": s.data_age_ms,
        "pose": s.pose.to_vec(),
        "torques": s.torques.to_vec(),
        "mode": s.mode as u8,
        "enabled": s.enabled,
        "gravity_comp": s.gravity_comp,
        "angles": s.angles.to_vec(),
        "speeds": s.speeds.to_vec(),
        "io": s.io.to_vec(),
        "action_current": s.action_current,
        "action_state": s.action_state as u8,
        "joint_en": s.joint_en.to_vec(),
        "cart_en_wrf": s.cart_en_wrf.to_vec(),
        "cart_en_trf": s.cart_en_trf.to_vec(),
        "executing_index": s.executing_index,
        "completed_index": s.completed_index,
        "last_checkpoint": s.last_checkpoint,
        "error": s.error.as_ref().map(error_json),
        "queued_segments": s.queued_segments,
        "queued_duration": s.queued_duration,
        "action_params": s.action_params,
        "tool_status": s.tool_status.as_ref().map(tool_status_json),
        "tcp_speed": s.tcp_speed,
        "simulator_active": s.simulator_active,
        "collision_active": s.collision_active,
        "collision_pairs": s.collision_pairs,
        "scene_epoch": s.scene_epoch,
        "accepted_index": s.accepted_index,
        "homed": s.homed,
        "warnings": warnings,
        "link_health": link_health,
        "homing": homing,
        "torques_ext": s.torques_ext.to_vec(),
    })
}

// ---------------------------------------------------------------------------
// Shared fixture values
// ---------------------------------------------------------------------------

const ANGLES: [f64; 6] = [10.0, -20.25, 30.5, 0.0, 45.0, -90.0];
const POSE6: [f64; 6] = [250.0, -10.5, 180.0, 0.0, 90.0, 179.5];

fn pose16() -> [f64; 16] {
    [
        1.0, 0.0, 0.0, 250.5, //
        0.0, 1.0, 0.0, -10.25, //
        0.0, 0.0, 1.0, 300.0, //
        0.0, 0.0, 0.0, 1.0,
    ]
}

fn shape_box() -> Shape {
    Shape {
        kind: "box".into(),
        params: vec![400.0, 600.0, 20.0],
        pose: vec![0.0, 0.0, -10.0, 0.0, 0.0, 0.0],
        collision: true,
        margin: Some(5.0),
        name: "table".into(),
    }
}

fn shape_sphere() -> Shape {
    Shape {
        kind: "sphere".into(),
        params: vec![50.0],
        pose: vec![120.0, 80.0, 200.0, 0.0, 0.0, 0.0],
        collision: false,
        margin: None,
        name: "camera".into(),
    }
}

fn tool_status_fixture() -> ToolStatusWire {
    ToolStatusWire {
        key: "SSG48".into(),
        state: ToolState::Active,
        engaged: true,
        part_detected: false,
        fault_code: 0,
        positions: vec![12.5],
        channels: vec![0.0, 3.3],
        variant_key: "fin_ray".into(),
    }
}

fn status_full_fixture() -> Status {
    Status {
        proto_version: PROTO_VERSION,
        controller_id: 0x00C0_FFEE,
        seq: 123_456_789,
        mono_time_ns: 3_600_000_000_123,
        link_ok: 1,
        data_age_ms: 4,
        pose: pose16(),
        angles: ANGLES,
        speeds: [0.1, -0.2, 0.0, 0.3, 0.0, 0.05],
        io: vec![1, 0, 1, 1, 0],
        action_current: "MOVE_L".into(),
        action_state: ActionState::Executing,
        joint_en: [1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1],
        cart_en_wrf: [1; 12],
        cart_en_trf: [1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1],
        executing_index: 12,
        completed_index: 11,
        last_checkpoint: "approach".into(),
        error: Some(make_error(
            ErrorCode::SysSelfCollision,
            12,
            &[
                ("sample", "140"),
                ("total", "500"),
                ("pairs", "[lower_arm, table]"),
            ],
        )),
        queued_segments: 3,
        queued_duration: 4.75,
        action_params: "pose=[250.0, -10.5, 180.0]".into(),
        tool_status: Some(tool_status_fixture()),
        tcp_speed: 33.25,
        simulator_active: true,
        collision_active: true,
        collision_pairs: vec![("lower_arm".into(), "table".into())],
        scene_epoch: 5,
        accepted_index: 14,
        homed: true,
        // Non-default on every new field, so a decoder that silently
        // defaults them cannot pass the cross-language vector.
        torques: [0.75, -1.5, 0.25, -0.125, 0.0625, -0.03125],
        mode: ControllerMode::Exec,
        enabled: true,
        gravity_comp: true,
        warnings: vec![
            make_error(ErrorCode::SysCanStale, UNATTRIBUTED, &[("joint", "2")]),
            make_error(ErrorCode::SysLoopDegraded, UNATTRIBUTED, &[]),
        ],
        link_health: crate::status::LinkHealthWire {
            state: crate::LinkState::ErrorPassive as u8,
            restarts: 2,
            tx_errors: 17,
            rx_frames: 987_654,
        },
        homing: crate::status::HomingWire {
            active: true,
            sequence_step: 3,
            joints: vec![
                (
                    crate::HomingJointState::Done as u8,
                    crate::HomingPhase::Finished as u8,
                ),
                (
                    crate::HomingJointState::Running as u8,
                    crate::HomingPhase::Settle as u8,
                ),
                (
                    crate::HomingJointState::Failed as u8,
                    crate::HomingPhase::Approach as u8,
                ),
                (
                    crate::HomingJointState::Idle as u8,
                    crate::HomingPhase::Idle as u8,
                ),
                (
                    crate::HomingJointState::Running as u8,
                    crate::HomingPhase::Backoff as u8,
                ),
                (
                    crate::HomingJointState::Done as u8,
                    crate::HomingPhase::Finished as u8,
                ),
                (
                    crate::HomingJointState::Idle as u8,
                    crate::HomingPhase::Idle as u8,
                ),
            ],
        },
        torques_ext: [0.5, -0.25, 0.125, -0.0625, 0.03125, -0.015625],
        drive_health: crate::status::DriveHealthWire {
            temperatures_c: vec![31.0, 32.5, 33.0, 34.5, 35.0, 36.5, 28.0],
            currents_ma: vec![120.0, 340.0, 275.0, 60.0, 45.0, 30.0, 15.0],
            bus_voltage_v: Some(23.7),
        },
        loop_health: crate::status::LoopHealthWire {
            p99_period_s: 0.004_51,
            overruns: 16,
        },
    }
}

// ---------------------------------------------------------------------------
// The vectors
// ---------------------------------------------------------------------------

/// Every golden vector, in manifest order.
#[allow(clippy::vec_init_then_push)] // one long declarative list reads best as pushes
pub fn vectors() -> Vec<Vector> {
    let mut v = Vec::new();

    // -- commands: SYSTEM --
    v.push(cmd_vec("cmd_reset", 1, Command::Reset));
    v.push(cmd_vec("cmd_estop", 2, Command::Estop));
    v.push(cmd_vec(
        "cmd_set_gravity_comp",
        2,
        Command::SetGravityComp(crate::command::SetGravityComp { on: true }),
    ));
    v.push(cmd_vec(
        "cmd_pause",
        2,
        Command::Pause(crate::command::Pause { on: true }),
    ));
    v.push(cmd_vec(
        "cmd_stop",
        3,
        Command::Stop(Stop { clear_queue: true }),
    ));
    v.push(cmd_vec(
        "cmd_write_io",
        4,
        Command::WriteIo(WriteIo { port: 3, value: 1 }),
    ));
    v.push(cmd_vec(
        "cmd_simulator",
        5,
        Command::Simulator(Simulator { on: true }),
    ));
    v.push(cmd_vec(
        "cmd_select_profile",
        6,
        Command::SelectProfile(SelectProfile {
            profile: "TOPPRA".into(),
        }),
    ));
    v.push(cmd_vec("cmd_reset_state", 7, Command::ResetState));
    v.push(cmd_vec(
        "cmd_connect_hardware",
        8,
        Command::ConnectHardware(ConnectHardware {
            port: "can0".into(),
        }),
    ));
    v.push(cmd_vec(
        "cmd_set_tcp_offset",
        9,
        Command::SetTcpOffset(SetTcpOffset {
            key: 113,
            x: 1.5,
            y: -2.0,
            z: 35.5,
        }),
    ));
    v.push(cmd_vec(
        "cmd_set_shapes",
        10,
        Command::SetShapes(SetShapes {
            shapes: vec![shape_box(), shape_sphere()],
        }),
    ));
    v.push(cmd_vec(
        "cmd_set_completion_policy",
        11,
        Command::SetCompletionPolicy(SetCompletionPolicy {
            policy: CompletionPolicy::Strict,
        }),
    ));
    v.push(cmd_vec(
        "cmd_set_recipe",
        12,
        Command::SetRecipe(SetRecipe {
            name: "diagnostics".into(),
        }),
    ));
    v.push(cmd_vec(
        "cmd_enter_flashing",
        13,
        Command::EnterFlashing(EnterFlashing {
            assertion: FlashingAssertion::Parked,
        }),
    ));
    v.push(cmd_vec(
        "cmd_enter_flashing_force",
        14,
        Command::EnterFlashing(EnterFlashing {
            assertion: FlashingAssertion::Force,
        }),
    ));
    v.push(cmd_vec("cmd_exit_flashing", 15, Command::ExitFlashing));
    v.push(cmd_vec(
        "cmd_set_pid_gains",
        16,
        Command::SetPidGains(SetPidGains {
            node: 2,
            kpp: 9.0,
            kpv: 0.05,
            kiv: 0.005,
            kpiq: 1.2,
            kiiq: 1.0,
            kp: 0.12,
            kd: 0.002,
            ilim_ma: 2200.0,
            velocity_limit_ticks_s: 150000.0,
            voltage_limit_mv: 0,
        }),
    ));

    // -- commands: QUERY --
    v.push(cmd_vec("cmd_ping", 20, Command::Ping));
    v.push(cmd_vec("cmd_status", 21, Command::Status));
    v.push(cmd_vec("cmd_angles", 22, Command::Angles));
    v.push(cmd_vec(
        "cmd_pose",
        23,
        Command::Pose(PoseQuery {
            frame: Some(Frame::Trf),
        }),
    ));
    v.push(cmd_vec(
        "cmd_pose_default_frame",
        24,
        Command::Pose(PoseQuery { frame: None }),
    ));
    v.push(cmd_vec("cmd_io", 25, Command::Io));
    v.push(cmd_vec("cmd_speeds", 26, Command::Speeds));
    v.push(cmd_vec("cmd_tools", 27, Command::Tools));
    v.push(cmd_vec("cmd_queue", 28, Command::Queue));
    v.push(cmd_vec("cmd_activity", 29, Command::Activity));
    v.push(cmd_vec("cmd_loop_stats", 30, Command::LoopStats));
    v.push(cmd_vec("cmd_profile", 31, Command::Profile));
    v.push(cmd_vec("cmd_reachable", 32, Command::Reachable));
    v.push(cmd_vec("cmd_error", 33, Command::Error));
    v.push(cmd_vec("cmd_tcp_speed", 34, Command::TcpSpeed));
    v.push(cmd_vec("cmd_tcp_offset", 35, Command::TcpOffset));
    v.push(cmd_vec("cmd_tool_status", 36, Command::ToolStatus));
    v.push(cmd_vec("cmd_is_simulator", 37, Command::IsSimulator));
    v.push(cmd_vec("cmd_shapes", 38, Command::Shapes));
    v.push(cmd_vec("cmd_config_info", 39, Command::ConfigInfo));
    v.push(cmd_vec(
        "cmd_set_payload",
        40,
        Command::SetPayload(SetPayload {
            mass: 1.25,
            com: [0.0, 0.01, 0.055],
            inertia: Some([0.002, 0.0, 0.003, 0.0001, 0.0, 0.004]),
        }),
    ));
    v.push(cmd_vec(
        "cmd_set_payload_point",
        41,
        Command::SetPayload(SetPayload {
            mass: 0.5,
            com: [0.0, 0.0, 0.04],
            inertia: None,
        }),
    ));
    v.push(cmd_vec("cmd_payload", 42, Command::Payload));
    v.push(cmd_vec("cmd_bus_scan", 43, Command::BusScan));
    v.push(cmd_vec(
        "cmd_set_can_id",
        17,
        Command::SetCanId(crate::command::SetCanId {
            node: 0,
            new_id: 9,
            force: true,
        }),
    ));
    v.push(cmd_vec(
        "cmd_save_config",
        18,
        Command::SaveConfig(crate::command::SaveConfig {
            node: 9,
            force: false,
        }),
    ));
    v.push(cmd_vec("cmd_config_bundle", 43, Command::ConfigBundle));

    // -- commands: FIRE_AND_FORGET --
    v.push(cmd_vec(
        "cmd_servo_j",
        50,
        Command::ServoJ(ServoJ {
            angles: ANGLES,
            speed: Some(0.5),
            accel: None,
        }),
    ));
    v.push(cmd_vec(
        "cmd_servo_j_pose",
        51,
        Command::ServoJPose(ServoJPose {
            pose: POSE6,
            speed: None,
            accel: Some(0.25),
        }),
    ));
    v.push(cmd_vec(
        "cmd_servo_l",
        52,
        Command::ServoL(ServoL {
            pose: POSE6,
            speed: Some(1.0),
            accel: Some(1.0),
        }),
    ));
    v.push(cmd_vec(
        "cmd_jog_j",
        53,
        Command::JogJ(JogJ {
            speeds: [0.2, -0.2, 0.0, 0.0, 1.0, -1.0],
            duration: 0.04,
            accel: Some(0.8),
        }),
    ));
    v.push(cmd_vec(
        "cmd_jog_l",
        54,
        Command::JogL(JogL {
            velocities: [0.0, 0.0, -0.5, 0.0, 0.0, 0.0],
            duration: 0.05,
            frame: Frame::Trf,
            accel: None,
        }),
    ));
    v.push(cmd_vec(
        "cmd_teleport",
        55,
        Command::Teleport(Teleport {
            angles: ANGLES,
            tool_positions: Some(vec![10.0]),
        }),
    ));
    v.push(cmd_vec(
        "cmd_teleport_no_tool",
        56,
        Command::Teleport(Teleport {
            angles: ANGLES,
            tool_positions: None,
        }),
    ));
    v.push(cmd_vec("cmd_reset_loop_stats", 57, Command::ResetLoopStats));

    // -- commands: QUEUED (idempotency key after req_id) --
    v.push(cmd_vec(
        "cmd_home",
        70,
        Command::Home(Home {
            key: 0x0123_4567_89AB_CDEF,
            calibrate: false,
        }),
    ));
    v.push(cmd_vec(
        "cmd_home_calibrate",
        301,
        Command::Home(Home {
            key: 0x0123_4567_89AB_CDF0,
            calibrate: true,
        }),
    ));
    v.push(cmd_vec(
        "cmd_move_j",
        71,
        Command::MoveJ(MoveJ {
            key: 101,
            angles: ANGLES,
            duration: None,
            speed: Some(0.5),
            accel: Some(0.75),
            blend_radius: None,
            rel: false,
        }),
    ));
    v.push(cmd_vec(
        "cmd_move_j_duration",
        72,
        Command::MoveJ(MoveJ {
            key: 102,
            angles: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            duration: Some(2.5),
            speed: None,
            accel: None,
            blend_radius: None,
            rel: true,
        }),
    ));
    v.push(cmd_vec(
        "cmd_move_j_pose",
        73,
        Command::MoveJPose(MoveJPose {
            key: 103,
            pose: POSE6,
            duration: None,
            speed: Some(0.9),
            accel: None,
            blend_radius: Some(10.0),
        }),
    ));
    v.push(cmd_vec(
        "cmd_move_l",
        74,
        Command::MoveL(MoveL {
            key: 104,
            pose: POSE6,
            frame: Frame::Wrf,
            duration: None,
            speed: Some(0.4),
            accel: Some(0.5),
            blend_radius: Some(5.0),
            rel: true,
        }),
    ));
    v.push(cmd_vec(
        "cmd_move_c",
        75,
        Command::MoveC(MoveC {
            key: 105,
            via: [200.0, 50.0, 150.0, 0.0, 90.0, 0.0],
            end: POSE6,
            frame: Frame::Wrf,
            duration: Some(3.0),
            speed: None,
            accel: None,
            blend_radius: None,
            rel: false,
        }),
    ));
    v.push(cmd_vec(
        "cmd_move_c_rel",
        79,
        Command::MoveC(MoveC {
            key: 109,
            via: [10.0, 0.0, 5.0, 0.0, 0.0, 0.0],
            end: [20.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            frame: Frame::Wrf,
            duration: None,
            speed: Some(0.5),
            accel: None,
            blend_radius: None,
            rel: true,
        }),
    ));
    v.push(cmd_vec(
        "cmd_move_s",
        76,
        Command::MoveS(MoveS {
            key: 106,
            waypoints: vec![
                [250.0, 0.0, 180.0, 0.0, 90.0, 0.0],
                [260.0, 10.0, 190.0, 0.0, 90.0, 0.0],
                [270.0, 0.0, 200.0, 0.0, 90.0, 0.0],
            ],
            frame: Frame::Wrf,
            duration: None,
            speed: Some(0.6),
            accel: None,
            rel: false,
        }),
    ));
    v.push(cmd_vec(
        "cmd_move_p",
        77,
        Command::MoveP(MoveP {
            key: 107,
            waypoints: vec![
                [250.0, 0.0, 180.0, 0.0, 90.0, 0.0],
                [250.0, 100.0, 180.0, 0.0, 90.0, 0.0],
            ],
            frame: Frame::Trf,
            duration: None,
            speed: Some(0.3),
            accel: Some(0.4),
            rel: false,
        }),
    ));
    v.push(cmd_vec(
        "cmd_select_tool",
        78,
        Command::SelectTool(SelectTool {
            key: 108,
            tool_name: "SSG48".into(),
            variant_key: Some("fin_ray".into()),
        }),
    ));
    v.push(cmd_vec(
        "cmd_select_tool_default_variant",
        79,
        Command::SelectTool(SelectTool {
            key: 109,
            tool_name: "MSG".into(),
            variant_key: None,
        }),
    ));
    v.push(cmd_vec(
        "cmd_delay",
        80,
        Command::Delay(Delay {
            key: 110,
            seconds: 0.5,
        }),
    ));
    v.push(cmd_vec(
        "cmd_checkpoint",
        81,
        Command::Checkpoint(Checkpoint {
            key: 111,
            label: "step-1".into(),
        }),
    ));
    v.push(cmd_vec(
        "cmd_tool_action",
        82,
        Command::ToolAction(ToolAction {
            key: 112,
            tool_key: "SSG48".into(),
            action: "move".into(),
            params: vec![
                ToolParam::Float(25.5),
                ToolParam::Int(80),
                ToolParam::Bool(true),
                ToolParam::Str("soft".into()),
            ],
        }),
    ));

    // -- replies --
    v.push(reply_vec(
        "reply_ok",
        Reply::Ok {
            req_id: 42,
            index: None,
        },
    ));
    v.push(reply_vec(
        "reply_ok_index",
        Reply::Ok {
            req_id: 43,
            index: Some(7),
        },
    ));
    v.push(reply_vec(
        "reply_error",
        Reply::Error {
            req_id: 44,
            error: make_error(
                ErrorCode::IkTargetUnreachable,
                5,
                &[("detail", "Target [520.0, 0.0, 130.0] mm.")],
            ),
        },
    ));
    v.push(reply_vec(
        "reply_complete_ok",
        Reply::Complete {
            index: 7,
            ok: true,
            detail: None,
            verdict: None,
        },
    ));
    v.push(reply_vec(
        "reply_complete_error",
        Reply::Complete {
            index: 8,
            ok: false,
            detail: Some(make_error(
                ErrorCode::MotnSettleTimeout,
                8,
                &[("residual", "0.02")],
            )),
            verdict: None,
        },
    ));
    v.push(reply_vec(
        "reply_complete_cancelled",
        Reply::Complete {
            index: 9,
            ok: false,
            detail: Some(make_error(
                ErrorCode::MotnCancelled,
                9,
                &[("scope", "stop")],
            )),
            verdict: None,
        },
    ));
    v.push(reply_vec(
        "reply_complete_tool_verdict",
        Reply::Complete {
            index: 10,
            ok: true,
            detail: None,
            verdict: Some(1),
        },
    ));
    v.push(reply_vec(
        "reply_error_near_singularity",
        Reply::Error {
            req_id: 47,
            error: make_error(
                ErrorCode::TrajNearSingularity,
                UNATTRIBUTED,
                &[("cond", "2400"), ("sigma", "0.00007")],
            ),
        },
    ));
    v.push(reply_vec(
        "reply_error_stream_fault",
        Reply::Error {
            req_id: 46,
            error: make_error(ErrorCode::SysStreamFault, UNATTRIBUTED, &[]),
        },
    ));

    // -- RESPONSE payloads, one per query type --
    v.push(reply_vec(
        "response_ping",
        Reply::Response {
            req_id: 100,
            result: QueryResult::Ping {
                hardware_connected: true,
            },
        },
    ));
    v.push(reply_vec(
        "response_status",
        Reply::Response {
            req_id: 101,
            result: QueryResult::Status {
                pose: pose16(),
                angles: ANGLES,
                speeds: [0.1, -0.2, 0.0, 0.3, 0.0, 0.05],
                io: vec![1, 0, 1, 0, 0],
                tool_status: Some(tool_status_fixture()),
            },
        },
    ));
    v.push(reply_vec(
        "response_angles",
        Reply::Response {
            req_id: 102,
            result: QueryResult::Angles { angles: ANGLES },
        },
    ));
    v.push(reply_vec(
        "response_pose",
        Reply::Response {
            req_id: 103,
            result: QueryResult::Pose { pose: pose16() },
        },
    ));
    v.push(reply_vec(
        "response_io",
        Reply::Response {
            req_id: 104,
            result: QueryResult::Io {
                io: vec![1, 1, 0, 0, 0],
            },
        },
    ));
    v.push(reply_vec(
        "response_speeds",
        Reply::Response {
            req_id: 105,
            result: QueryResult::Speeds {
                speeds: [0.0, 0.0, 0.1, 0.0, -0.1, 0.0],
            },
        },
    ));
    v.push(reply_vec(
        "response_tools",
        Reply::Response {
            req_id: 106,
            result: QueryResult::Tools {
                tool: "SSG48".into(),
                available: vec!["SSG48".into(), "MSG".into()],
            },
        },
    ));
    v.push(reply_vec(
        "response_queue",
        Reply::Response {
            req_id: 107,
            result: QueryResult::Queue {
                queue: vec!["MOVE_J".into(), "DELAY".into()],
                executing_index: 4,
                completed_index: 3,
                last_checkpoint: "pick".into(),
                queued_duration: 2.5,
            },
        },
    ));
    v.push(reply_vec(
        "response_activity",
        Reply::Response {
            req_id: 108,
            result: QueryResult::Activity {
                current: "MOVE_L".into(),
                state: ActionState::Executing,
                next: "DELAY".into(),
                params: "pose=[250.0, -10.5, 180.0]".into(),
            },
        },
    ));
    v.push(reply_vec(
        "response_loop_stats",
        Reply::Response {
            req_id: 109,
            result: QueryResult::LoopStats(LoopStatsResult {
                target_hz: 250.0,
                loop_count: 100_000,
                overrun_count: 3,
                mean_period_s: 0.004,
                std_period_s: 0.0001,
                min_period_s: 0.0038,
                max_period_s: 0.0061,
                p95_period_s: 0.0042,
                p99_period_s: 0.0044,
                mean_hz: 249.9,
                p50_period_s: 0.0039,
                p90_period_s: 0.0041,
                can_frame_age_min_ticks: 1,
                can_frame_age_max_ticks: 4,
                rt_fifo: true,
                rt_pinned: true,
            }),
        },
    ));
    v.push(reply_vec(
        "response_profile",
        Reply::Response {
            req_id: 110,
            result: QueryResult::Profile {
                profile: "TOPPRA".into(),
            },
        },
    ));
    v.push(reply_vec(
        "response_reachable",
        Reply::Response {
            req_id: 111,
            result: QueryResult::Reachable {
                joint_en: [1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1],
                cart_en_wrf: [1; 12],
                cart_en_trf: [1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1],
            },
        },
    ));
    v.push(reply_vec(
        "response_error_none",
        Reply::Response {
            req_id: 112,
            result: QueryResult::Error { error: None },
        },
    ));
    v.push(reply_vec(
        "response_error_active",
        Reply::Response {
            req_id: 113,
            result: QueryResult::Error {
                error: Some(make_error(ErrorCode::SysEstopActive, UNATTRIBUTED, &[])),
            },
        },
    ));
    v.push(reply_vec(
        "response_tcp_speed",
        Reply::Response {
            req_id: 114,
            result: QueryResult::TcpSpeed { speed: 12.5 },
        },
    ));
    v.push(reply_vec(
        "response_tcp_offset",
        Reply::Response {
            req_id: 115,
            result: QueryResult::TcpOffset {
                x: 0.0,
                y: 0.0,
                z: 35.5,
            },
        },
    ));
    v.push(reply_vec(
        "response_tool_status",
        Reply::Response {
            req_id: 116,
            result: QueryResult::ToolStatus {
                tool_status: Some(tool_status_fixture()),
            },
        },
    ));
    v.push(reply_vec(
        "response_is_simulator",
        Reply::Response {
            req_id: 117,
            result: QueryResult::IsSimulator { active: true },
        },
    ));
    v.push(reply_vec(
        "response_shapes",
        Reply::Response {
            req_id: 118,
            result: QueryResult::Shapes {
                installation: vec![shape_box()],
                program: vec![shape_sphere()],
                epoch: 3,
            },
        },
    ));
    v.push(reply_vec(
        "response_config_info",
        Reply::Response {
            req_id: 119,
            result: QueryResult::ConfigInfo {
                path: "/etc/par6/PAR6.toml".to_owned(),
                fingerprint: "9f86d081884c7d659a2feaa0c55ad015a3bf4f1b2b0b822cd15d6c15b0f00a08"
                    .to_owned(),
                tick_dt_s: 0.004,
                motion: [0.08, 0.6, 0.005, 0.05, 0.35, 0.05, 0.01, 2.0],
                joints: vec![
                    [-2.15, 2.15, 3.0, 12.0],
                    [-1.0, 1.9, 2.5, 10.0],
                    [-1.8, 1.2, 2.5, 10.0],
                    [-1.9, 1.9, 4.0, 16.0],
                    [-2.0, 2.0, 4.0, 16.0],
                    [-6.3, 6.3, 6.0, 20.0],
                ],
                active_recipe: Some("standard".to_owned()),
                recipes: vec![
                    "minimal".to_owned(),
                    "standard".to_owned(),
                    "full".to_owned(),
                    "diagnostics".to_owned(),
                ],
            },
        },
    ));
    v.push(reply_vec(
        "response_config_info_telemetry_off",
        Reply::Response {
            req_id: 127,
            result: QueryResult::ConfigInfo {
                path: "/etc/par6/PAR6.toml".to_owned(),
                fingerprint: "9f86d081884c7d659a2feaa0c55ad015a3bf4f1b2b0b822cd15d6c15b0f00a08"
                    .to_owned(),
                tick_dt_s: 0.004,
                motion: [0.08, 0.6, 0.005, 0.05, 0.35, 0.05, 0.01, 2.0],
                joints: vec![[-2.15, 2.15, 3.0, 12.0]],
                active_recipe: None,
                recipes: vec!["minimal".to_owned(), "standard".to_owned()],
            },
        },
    ));
    v.push(reply_vec(
        "response_payload",
        Reply::Response {
            req_id: 120,
            result: QueryResult::Payload {
                mass: 1.25,
                com: [0.0, 0.01, 0.055],
                inertia: [0.002, 0.0, 0.003, 0.0001, 0.0, 0.004],
            },
        },
    ));

    v.push(reply_vec(
        "response_bus_scan",
        Reply::Response {
            req_id: 128,
            result: QueryResult::BusScan {
                nodes: vec![
                    crate::BusNode {
                        node: 0,
                        configured: true,
                        present: true,
                        freshness: 1,
                        hw_ver: 2,
                        sw_ver: 7,
                        serial: 1234567,
                    },
                    crate::BusNode {
                        node: 9,
                        configured: false,
                        present: true,
                        freshness: 0,
                        hw_ver: 0,
                        sw_ver: 0,
                        serial: 0,
                    },
                ],
            },
        },
    ));

    v.push(reply_vec(
        "response_config_bundle",
        Reply::Response {
            req_id: 121,
            result: QueryResult::ConfigBundle {
                path: "/etc/par6/PAR6.toml".to_owned(),
                fingerprint: "9f86d081884c7d659a2feaa0c55ad015a3bf4f1b2b0b822cd15d6c15b0f00a08"
                    .to_owned(),
                robot_filename: "PAR6.toml".to_owned(),
                robot_toml: "[robot]\nname = \"PAR6\"\ntick_hz = 250\n".to_owned(),
                grippers: vec![
                    ("SSG-48.toml".to_owned(), "name = \"SSG-48\"\n".to_owned()),
                    ("SSG-66.toml".to_owned(), "name = \"SSG-66\"\n".to_owned()),
                ],
            },
        },
    ));

    // -- STATUS broadcasts --
    v.push(status_vec("status_full", status_full_fixture()));
    v.push(status_vec("status_idle", Status::default()));
    v.push(status_longer_tail_vec());
    // The vendor control box's ten lines plus the e-stop. `io` is
    // variable-length, so covering them is a longer array and NOT a
    // layout change; this vector is what says so across both languages.
    v.push(status_vec(
        "status_ten_io_lines",
        status_ten_io_lines_fixture(),
    ));

    // -- CHUNK sequence: a MOVE_S split into 3 envelopes --
    v.extend(chunk_vectors());

    // -- malformed --
    v.extend(malformed_vectors());

    v
}

/// The stock control box's lines, in the order the shipped `[io]` config
/// declares them: seven inputs (3 isolated, 4 general), three isolated
/// outputs, e-stop last. Nothing about the STATUS layout changes to
/// carry them — only the length of one array, which is why the suite
/// also keeps a five-slot status alongside this one.
fn status_ten_io_lines_fixture() -> Status {
    Status {
        io: vec![1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0],
        ..status_full_fixture()
    }
}

/// A v2 status packet with two extra tail elements, as a v3 producer would
/// send: v2 decoders must ignore the tail (forward compatibility).
fn status_longer_tail_vec() -> Vector {
    let status = status_full_fixture();
    let mut bytes = Vec::new();
    // Re-encode by hand with a larger arity and extra tail values.
    encode_status_into(&status, &mut bytes);
    // The 31-element header is `0xDC 0x00 0x1F` (array16); patch to 33.
    assert_eq!(&bytes[..3], &[0xDC, 0x00, STATUS_LEN as u8]);
    bytes[2] = (STATUS_LEN + 2) as u8;
    w_str(&mut bytes, "future-field");
    w_f64(&mut bytes, 1.25);
    let manifest = json!({
        "name": "status_longer_tail",
        "kind": "status",
        "wire": wire_json(&bytes),
        "fields": status_fields(&status),
    });
    Vector {
        name: "status_longer_tail",
        bytes,
        manifest,
        check: Check::Status {
            status: Box::new(status),
            decode_only: true,
        },
    }
}

fn chunk_vectors() -> Vec<Vector> {
    let inner = Command::MoveS(MoveS {
        key: 200,
        waypoints: (0..20)
            .map(|i| {
                let t = f64::from(i);
                [250.0 + t, t * 2.0, 180.0 + t, 0.0, 90.0, 0.0]
            })
            .collect(),
        frame: Frame::Wrf,
        duration: None,
        speed: Some(0.5),
        accel: None,
        rel: false,
    });
    let mut payload = Vec::new();
    encode_command(&inner, 300, &mut payload).expect("valid inner command");
    let chunks = split_into_chunks(300, 77, &payload, payload.len().div_ceil(3));
    assert_eq!(chunks.len(), 3, "fixture is sized for a 3-chunk sequence");
    let names: [&'static str; 3] = ["chunk_0", "chunk_1", "chunk_2"];
    let assembled = hex(&payload);
    chunks
        .into_iter()
        .zip(names)
        .map(|(c, name)| {
            let mut bytes = Vec::new();
            encode_chunk(&c, &mut bytes);
            let manifest = json!({
                "name": name,
                "kind": "chunk",
                "req_id": c.req_id,
                "transfer_id": c.transfer_id,
                "index": c.index,
                "total": c.total,
                "assembled_hex": assembled,
                "inner_cmd": "MOVE_S",
                "wire": wire_json(&bytes),
            });
            Vector {
                name,
                bytes,
                manifest,
                check: Check::Chunk(Box::new(c)),
            }
        })
        .collect()
}

fn malformed_vectors() -> Vec<Vector> {
    let mut out = Vec::new();

    // wrong tag
    let mut b = Vec::new();
    w_array(&mut b, 2);
    w_uint(&mut b, 999);
    w_uint(&mut b, 1);
    out.push(malformed_vec(
        "malformed_unknown_tag",
        "command",
        "tag 999 names no command",
        b,
    ));

    // short array (no req_id)
    let mut b = Vec::new();
    w_array(&mut b, 1);
    w_uint(&mut b, 81);
    out.push(malformed_vec(
        "malformed_short_array",
        "command",
        "envelope has no req_id",
        b,
    ));

    // req_id has the wrong type
    let mut b = Vec::new();
    w_array(&mut b, 2);
    w_uint(&mut b, 10);
    w_str(&mut b, "abc");
    out.push(malformed_vec(
        "malformed_reqid_type",
        "command",
        "req_id must be an unsigned int",
        b,
    ));

    // req_id negative
    let mut b = Vec::new();
    w_array(&mut b, 2);
    w_uint(&mut b, 10);
    b.push(0xFF); // -1 as negative fixint
    out.push(malformed_vec(
        "malformed_reqid_negative",
        "command",
        "req_id must be unsigned",
        b,
    ));

    // param with the wrong type (write_io port as str)
    let mut b = Vec::new();
    w_array(&mut b, 4);
    w_uint(&mut b, 13);
    w_uint(&mut b, 7);
    w_str(&mut b, "x");
    w_uint(&mut b, 1);
    out.push(malformed_vec(
        "malformed_param_type",
        "command",
        "write_io.port must be an unsigned int",
        b,
    ));

    // param out of range (write_io port 9)
    let mut b = Vec::new();
    w_array(&mut b, 4);
    w_uint(&mut b, 13);
    w_uint(&mut b, 7);
    w_uint(&mut b, 9);
    w_uint(&mut b, 0);
    out.push(malformed_vec(
        "malformed_param_range",
        "command",
        "write_io.port must be 0..=7",
        b,
    ));

    // NaN param (delay seconds)
    let mut b = Vec::new();
    w_array(&mut b, 4);
    w_uint(&mut b, 88);
    w_uint(&mut b, 7);
    w_uint(&mut b, 5);
    w_f64(&mut b, f64::NAN);
    out.push(malformed_vec(
        "malformed_nan_param",
        "command",
        "floats must be finite",
        b,
    ));

    // duration and speed both set (move_j)
    let mut b = Vec::new();
    w_array(&mut b, 9);
    w_uint(&mut b, 81);
    w_uint(&mut b, 7);
    w_uint(&mut b, 5);
    w_array(&mut b, 6);
    for a in ANGLES {
        w_f64(&mut b, a);
    }
    w_f64(&mut b, 1.0); // duration
    w_f64(&mut b, 0.5); // speed
    w_nil(&mut b);
    w_nil(&mut b);
    b.push(0xC2); // rel = false
    out.push(malformed_vec(
        "malformed_both_timing",
        "command",
        "duration and speed are mutually exclusive",
        b,
    ));

    // queued command missing its idempotency key (home with bare envelope)
    let mut b = Vec::new();
    w_array(&mut b, 2);
    w_uint(&mut b, 80);
    w_uint(&mut b, 7);
    out.push(malformed_vec(
        "malformed_missing_key",
        "command",
        "queued commands carry an idempotency key",
        b,
    ));

    // truncated datagram
    let mut full = Vec::new();
    encode_command(
        &Command::MoveJ(MoveJ {
            key: 101,
            angles: ANGLES,
            duration: None,
            speed: Some(0.5),
            accel: None,
            blend_radius: None,
            rel: false,
        }),
        71,
        &mut full,
    )
    .unwrap();
    full.truncate(full.len() / 2);
    out.push(malformed_vec(
        "malformed_truncated",
        "command",
        "datagram cut mid-value",
        full,
    ));

    // reply with an unknown tag
    let mut b = Vec::new();
    w_array(&mut b, 2);
    w_uint(&mut b, 99);
    w_uint(&mut b, 1);
    out.push(malformed_vec(
        "malformed_reply_unknown_tag",
        "reply",
        "tag 99 names no message",
        b,
    ));

    // COMPLETE with a nonzero req_id
    let mut b = Vec::new();
    w_array(&mut b, 4);
    w_uint(&mut b, 5);
    w_uint(&mut b, 3);
    w_uint(&mut b, 7);
    b.push(0xC3); // true
    out.push(malformed_vec(
        "malformed_complete_reqid",
        "reply",
        "COMPLETE pushes use req_id 0",
        b,
    ));

    // status with too few elements
    let mut b = Vec::new();
    w_array(&mut b, 2);
    w_uint(&mut b, 3);
    w_uint(&mut b, 2);
    out.push(malformed_vec(
        "malformed_status_short",
        "status",
        "v2 status has 31 elements",
        b,
    ));

    // status with a mistyped field (pose as str)
    let mut b = Vec::new();
    w_array(&mut b, STATUS_LEN);
    w_uint(&mut b, 3); // STATUS
    w_uint(&mut b, 2); // proto_version
    w_uint(&mut b, 1); // controller_id
    w_uint(&mut b, 1); // seq
    w_uint(&mut b, 1); // mono_time_ns
    w_uint(&mut b, 1); // link_ok
    w_uint(&mut b, 0); // data_age_ms
    w_str(&mut b, "nope"); // pose must be f64[16]
    for _ in 8..STATUS_LEN {
        w_nil(&mut b);
    }
    out.push(malformed_vec(
        "malformed_status_bad_field",
        "status",
        "pose must be a float array",
        b,
    ));

    // set_payload: negative mass
    let mut b = Vec::new();
    w_array(&mut b, 5);
    w_uint(&mut b, 25);
    w_uint(&mut b, 9);
    w_f64(&mut b, -0.5);
    w_array(&mut b, 3);
    for _ in 0..3 {
        w_f64(&mut b, 0.0);
    }
    w_nil(&mut b);
    out.push(malformed_vec(
        "malformed_payload_negative_mass",
        "command",
        "payload mass must be >= 0",
        b,
    ));

    // set_payload: negative-definite inertia (fails the PSD check)
    let mut b = Vec::new();
    w_array(&mut b, 5);
    w_uint(&mut b, 25);
    w_uint(&mut b, 9);
    w_f64(&mut b, 1.0);
    w_array(&mut b, 3);
    for _ in 0..3 {
        w_f64(&mut b, 0.0);
    }
    w_array(&mut b, 6);
    for v in [-1.0, 0.0, 1.0, 0.0, 0.0, 1.0] {
        w_f64(&mut b, v);
    }
    out.push(malformed_vec(
        "malformed_payload_indefinite_inertia",
        "command",
        "payload inertia must be positive semidefinite",
        b,
    ));

    out
}

/// The manifest document for a vector set.
pub fn manifest_json(vectors: &[Vector]) -> Value {
    let entries: Vec<Value> = vectors
        .iter()
        .map(|v| {
            let mut m = v.manifest.clone();
            m.as_object_mut()
                .expect("manifest entries are objects")
                .insert("file".into(), json!(format!("{}.bin", v.name)));
            m
        })
        .collect();
    json!({
        "format": 1,
        "proto_version": PROTO_VERSION,
        "generator": "cargo run -p par6-proto --bin gen_golden",
        "vectors": entries,
    })
}

/// The manifest serialized exactly as committed (pretty JSON + newline).
pub fn manifest_string(vectors: &[Vector]) -> String {
    let mut s = serde_json::to_string_pretty(&manifest_json(vectors)).expect("serializable");
    s.push('\n');
    s
}

/// The python constants file exactly as committed.
pub fn constants_py() -> String {
    pygen::generate()
}
