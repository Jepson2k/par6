//! Frame codec: arbitration ids, big-endian payload primitives, and the
//! full Spectral/STEPFOC command table — encoding for every host→driver
//! frame, decoding for every driver→host reply.
//!
//! Everything here is pure and allocation-free ([`CanFrame`] is a fixed
//! 8-byte buffer), so the RT tick path can call it directly.

use par6_config::WatchdogAction;

use crate::types::{
    DeviceInfo, ErrorFlags, FirmwareGripperCommand, GripperCommand, GripperReply, HallState,
    JointCommand, NodeId, ObjectDetection, Pack, PollKind,
};

/// Classic CAN 2.0A payload capacity.
pub const CAN_MAX_DATA: usize = 8;

/// One classic CAN 2.0A frame (11-bit id, ≤8 data bytes).
///
/// `dlc` is the valid prefix of `data`; RTR frames carry no data
/// (`dlc = 0` — the vendor stack requests replies with empty remote
/// frames).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanFrame {
    /// 11-bit arbitration id: `(node << 7) | (cmd << 1) | err_bit`.
    pub id: u16,
    /// Remote frame (telemetry request) — no payload.
    pub rtr: bool,
    /// Number of valid bytes in `data`.
    pub dlc: u8,
    /// Payload buffer; only `data[..dlc]` is meaningful.
    pub data: [u8; CAN_MAX_DATA],
}

impl CanFrame {
    /// A data frame with the given payload (≤8 bytes).
    pub fn data_frame(id: u16, payload: &[u8]) -> Self {
        debug_assert!(payload.len() <= CAN_MAX_DATA);
        let mut data = [0u8; CAN_MAX_DATA];
        data[..payload.len()].copy_from_slice(payload);
        Self {
            id,
            rtr: false,
            dlc: payload.len() as u8,
            data,
        }
    }

    /// An empty remote (RTR) frame.
    pub fn rtr_frame(id: u16) -> Self {
        Self {
            id,
            rtr: true,
            dlc: 0,
            data: [0u8; CAN_MAX_DATA],
        }
    }

    /// The valid payload bytes.
    pub fn payload(&self) -> &[u8] {
        &self.data[..usize::from(self.dlc.min(CAN_MAX_DATA as u8))]
    }
}

/// The Spectral/STEPFOC command table (6-bit command field).
///
/// Discriminants are the wire values. Cmds 5/7 are in the table but
/// reserved — [`decode_frame`] refuses them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum CommandId {
    /// 0 — ESTOP (data DLC 0).
    Estop = 0,
    /// 1 — Clear_Error (data DLC 0).
    ClearError = 1,
    /// 2 — data_pack_1, cascade PID motion (DLC 8/5/2).
    DataPack1 = 2,
    /// 3 — Respond_data_pack_1 (D→H DLC 8).
    RespondDataPack1 = 3,
    /// 4 — data_pack_PD, impedance (DLC 8).
    DataPackPd = 4,
    /// 5 — Respond_data_pack_2 (reserved — do not decode).
    RespondDataPack2 = 5,
    /// 7 — Respond_data_pack_3 (reserved — do not decode).
    RespondDataPack3 = 7,
    /// 9 — heartbeat reply (enabled by cmd 30; no payload).
    RespondHeartbeat = 9,
    /// 10 — Ping (RTR; the driver replies with a cmd-10 data frame).
    Ping = 10,
    /// 11 — Set_CAN_ID (DLC 1).
    SetCanId = 11,
    /// 12 — Idle (data DLC 0).
    Idle = 12,
    /// 13 — Save_config (data DLC 0).
    SaveConfig = 13,
    /// 14 — Reset (data DLC 0).
    Reset = 14,
    /// 15 — Watchdog (DLC 5: u32 ms + u8 action).
    Watchdog = 15,
    /// 16 — PD_Gains (DLC 8: f32 KP, f32 KD).
    PdGains = 16,
    /// 17 — Current_Gains (DLC 8: f32 KPIQ, f32 KIIQ).
    CurrentGains = 17,
    /// 18 — Velocity_Gains (DLC 8: f32 KPV, f32 KIV).
    VelocityGains = 18,
    /// 19 — Position_Gains (DLC 4: f32 KPP).
    PositionGains = 19,
    /// 20 — Limits (DLC 8: f32 vel ticks/s, f32 cur mA).
    Limits = 20,
    /// 22 — Kt set (DLC 4: f32 Nm/A).
    Kt = 22,
    /// 23 — Temperature (RTR; reply DLC 2: i16 °C).
    Temperature = 23,
    /// 24 — Voltage (RTR; reply DLC 2: i16 mV).
    Voltage = 24,
    /// 25 — Device_Info (RTR; reply DLC 7).
    DeviceInfo = 25,
    /// 26 — State_of_Errors (RTR; reply DLC 2 bitfields).
    StateOfErrors = 26,
    /// 27 — Iq_data (RTR; reply DLC 2: i16 mA).
    IqData = 27,
    /// 28 — Encoder_data (RTR; reply DLC 8: i32 pos, i32 spd).
    EncoderData = 28,
    /// 30 — Heartbeat_Setup (DLC 4: u32 ms).
    HeartbeatSetup = 30,
    /// 31 — data_pack_HALL (DLC 4: i24 speed + u8 trigger value).
    DataPackHall = 31,
    /// 32 — RESPOND_DATA_HALL (D→H DLC 4).
    RespondDataHall = 32,
    /// 33 — Respond_Kt (RTR; reply DLC 4: f32 Nm/A).
    RespondKt = 33,
    /// 34 — Voltage_Limit (DLC 4: u32 mV; old firmware ignores).
    VoltageLimit = 34,
    /// 60 — Respond_Gripper_data (D→H DLC 4).
    RespondGripperData = 60,
    /// 61 — Gripper_data_pack (DLC 5, or DLC 0 = empty watchdog poll).
    GripperDataPack = 61,
    /// 62 — Gripper_calibrate (data DLC 0).
    GripperCalibrate = 62,
}

impl CommandId {
    /// The 6-bit wire value.
    pub fn raw(self) -> u8 {
        self as u8
    }

    /// Look up a wire command value; `None` for ids outside the table.
    pub fn from_raw(cmd: u8) -> Option<Self> {
        use CommandId::*;
        Some(match cmd {
            0 => Estop,
            1 => ClearError,
            2 => DataPack1,
            3 => RespondDataPack1,
            4 => DataPackPd,
            5 => RespondDataPack2,
            7 => RespondDataPack3,
            9 => RespondHeartbeat,
            10 => Ping,
            11 => SetCanId,
            12 => Idle,
            13 => SaveConfig,
            14 => Reset,
            15 => Watchdog,
            16 => PdGains,
            17 => CurrentGains,
            18 => VelocityGains,
            19 => PositionGains,
            20 => Limits,
            22 => Kt,
            23 => Temperature,
            24 => Voltage,
            25 => DeviceInfo,
            26 => StateOfErrors,
            27 => IqData,
            28 => EncoderData,
            30 => HeartbeatSetup,
            31 => DataPackHall,
            32 => RespondDataHall,
            33 => RespondKt,
            34 => VoltageLimit,
            60 => RespondGripperData,
            61 => GripperDataPack,
            62 => GripperCalibrate,
            _ => return None,
        })
    }
}

/// Pack an 11-bit arbitration id: `(node << 7) | (cmd << 1) | err`.
pub fn pack_can_id(node: NodeId, cmd: CommandId, err_bit: bool) -> u16 {
    (u16::from(node & 0xF) << 7) | (u16::from(cmd.raw() & 0x3F) << 1) | u16::from(err_bit)
}

/// Unpack an 11-bit arbitration id into `(node, raw_cmd, err_bit)`.
///
/// The command comes back raw (it may be outside the table — bootloader
/// page frames alias application ids); classify with
/// [`CommandId::from_raw`].
pub fn unpack_can_id(id: u16) -> (NodeId, u8, bool) {
    let node = ((id >> 7) & 0xF) as u8;
    let cmd = ((id >> 1) & 0x3F) as u8;
    (node, cmd, id & 1 == 1)
}

// ---------------------------------------------------------------------------
// Big-endian payload primitives
// ---------------------------------------------------------------------------

/// Pack a signed 24-bit value, big-endian two's complement. Out-of-range
/// inputs wrap modulo 2^24 (vendor `v & 0xFFFFFF` semantics).
pub fn pack_i24(v: i32) -> [u8; 3] {
    let u = (v as u32) & 0x00FF_FFFF;
    [(u >> 16) as u8, (u >> 8) as u8, u as u8]
}

/// Unpack a big-endian two's-complement 24-bit value.
pub fn unpack_i24(b: [u8; 3]) -> i32 {
    let u = (u32::from(b[0]) << 16) | (u32::from(b[1]) << 8) | u32::from(b[2]);
    if u >= 1 << 23 {
        u as i32 - (1 << 24)
    } else {
        u as i32
    }
}

/// Pack a signed 16-bit value, big-endian.
pub fn pack_i16(v: i16) -> [u8; 2] {
    v.to_be_bytes()
}

/// Unpack a big-endian signed 16-bit value.
pub fn unpack_i16(b: [u8; 2]) -> i16 {
    i16::from_be_bytes(b)
}

/// Pack a signed 32-bit value, big-endian.
pub fn pack_i32(v: i32) -> [u8; 4] {
    v.to_be_bytes()
}

/// Unpack a big-endian signed 32-bit value.
pub fn unpack_i32(b: [u8; 4]) -> i32 {
    i32::from_be_bytes(b)
}

/// Pack an unsigned 32-bit value, big-endian.
pub fn pack_u32(v: u32) -> [u8; 4] {
    v.to_be_bytes()
}

/// Unpack a big-endian unsigned 32-bit value.
pub fn unpack_u32(b: [u8; 4]) -> u32 {
    u32::from_be_bytes(b)
}

/// Pack an IEEE-754 single, big-endian.
pub fn pack_f32(v: f32) -> [u8; 4] {
    v.to_be_bytes()
}

/// Unpack a big-endian IEEE-754 single.
pub fn unpack_f32(b: [u8; 4]) -> f32 {
    f32::from_be_bytes(b)
}

/// Fold a bit list into one byte, MSB first: `bits[0]` becomes bit 7.
/// This is the vendor list convention — the reason the docs' bit tables
/// look inverted.
pub fn fold_bits_msb_first(bits: [bool; 8]) -> u8 {
    bits.iter().fold(0u8, |n, &b| (n << 1) | u8::from(b))
}

/// Unfold one byte into a bit list, MSB first: index 0 = bit 7.
pub fn unfold_bits_msb_first(byte: u8) -> [bool; 8] {
    core::array::from_fn(|i| (byte >> (7 - i)) & 1 == 1)
}

// ---------------------------------------------------------------------------
// Encoding (host → driver)
// ---------------------------------------------------------------------------

/// A command that has no wire form.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum EncodeError {
    /// cmd 2 has DLC variants for pos+vel+cur / vel+cur / cur — a position
    /// without a velocity is not encodable (the vendor stack refuses it
    /// too, silently; we refuse loudly).
    #[error("cmd 2 has no wire form for position without velocity")]
    PositionWithoutVelocity,
    /// RTR polls exist only for the telemetry/reply commands
    /// (10/23/24/25/26/27/28/33).
    #[error("cmd {cmd} is not pollable via RTR")]
    NotPollable {
        /// The refused command.
        cmd: u8,
    },
}

/// Encode one joint motion frame per `command.pack`.
///
/// - [`Pack::Pid`] (cmd 2): DLC selects the mode — pos+vel+cur → 8,
///   vel+cur → 5, cur → 2. All three channels `None` = **no frame**
///   (`Ok(None)`). `cur_ma = None` substitutes 0 here, per the contract
///   on [`JointCommand`]; a position without a velocity is refused.
/// - [`Pack::Pd`] (cmd 4): always DLC 8; `None` channels pack as 0
///   (vendor defaults).
/// - [`Pack::Hall`] (cmd 31): DLC 4, i24 speed (`vel`, `None` → 0) + the
///   trigger byte; `pos`/`cur_ma` are not part of the frame.
/// - [`Pack::Idle`] (cmd 12): DLC-0 data frame, no channels.
/// - [`Pack::EncoderPoll`] (cmd 28): RTR request, no channels.
pub fn encode_joint_command(
    node: NodeId,
    command: &JointCommand,
) -> Result<Option<CanFrame>, EncodeError> {
    let frame = match command.pack {
        Pack::Pid => {
            let id = pack_can_id(node, CommandId::DataPack1, false);
            let cur = pack_i16(command.cur_ma.unwrap_or(0));
            match (command.pos, command.vel) {
                (Some(pos), Some(vel)) => {
                    let mut p = [0u8; 8];
                    p[0..3].copy_from_slice(&pack_i24(pos));
                    p[3..6].copy_from_slice(&pack_i24(vel));
                    p[6..8].copy_from_slice(&cur);
                    CanFrame::data_frame(id, &p)
                }
                (None, Some(vel)) => {
                    let mut p = [0u8; 5];
                    p[0..3].copy_from_slice(&pack_i24(vel));
                    p[3..5].copy_from_slice(&cur);
                    CanFrame::data_frame(id, &p)
                }
                (None, None) => {
                    if command.cur_ma.is_none() {
                        return Ok(None);
                    }
                    CanFrame::data_frame(id, &cur)
                }
                (Some(_), None) => return Err(EncodeError::PositionWithoutVelocity),
            }
        }
        Pack::Pd => {
            let id = pack_can_id(node, CommandId::DataPackPd, false);
            let mut p = [0u8; 8];
            p[0..3].copy_from_slice(&pack_i24(command.pos.unwrap_or(0)));
            p[3..6].copy_from_slice(&pack_i24(command.vel.unwrap_or(0)));
            p[6..8].copy_from_slice(&pack_i16(command.cur_ma.unwrap_or(0)));
            CanFrame::data_frame(id, &p)
        }
        Pack::Hall { trigger_value } => {
            let id = pack_can_id(node, CommandId::DataPackHall, false);
            let mut p = [0u8; 4];
            p[0..3].copy_from_slice(&pack_i24(command.vel.unwrap_or(0)));
            p[3] = trigger_value;
            CanFrame::data_frame(id, &p)
        }
        Pack::Idle => encode_idle(node),
        Pack::EncoderPoll => CanFrame::rtr_frame(pack_can_id(node, CommandId::EncoderData, false)),
    };
    Ok(Some(frame))
}

/// Encode the per-tick gripper-slot frame.
///
/// `NoGripper` becomes the RTR ping to the timing dummy node that keeps
/// the frame cadence constant; `Motor` encodes like a 7th joint on the
/// gripper node. `Ok(None)` only for a `Motor` command with every channel
/// `None`.
pub fn encode_gripper_command(
    gripper_node: NodeId,
    timing_dummy_node: NodeId,
    command: &GripperCommand,
) -> Result<Option<CanFrame>, EncodeError> {
    match command {
        GripperCommand::NoGripper => Ok(Some(CanFrame::rtr_frame(pack_can_id(
            timing_dummy_node,
            CommandId::Ping,
            false,
        )))),
        GripperCommand::Motor(jc) => encode_joint_command(gripper_node, jc),
        GripperCommand::Firmware(f) => Ok(Some(encode_firmware_gripper(gripper_node, f))),
        GripperCommand::FirmwarePoll => Ok(Some(CanFrame::data_frame(
            pack_can_id(gripper_node, CommandId::GripperDataPack, false),
            &[],
        ))),
        GripperCommand::Calibrate => Ok(Some(CanFrame::data_frame(
            pack_can_id(gripper_node, CommandId::GripperCalibrate, false),
            &[],
        ))),
    }
}

fn encode_firmware_gripper(node: NodeId, f: &FirmwareGripperCommand) -> CanFrame {
    let mut p = [0u8; 5];
    p[0] = f.position;
    p[1] = f.speed;
    p[2..4].copy_from_slice(&pack_i16(f.current_ma));
    p[4] = fold_bits_msb_first([
        f.activate,
        f.action,
        f.estop,
        f.release_dir,
        false,
        false,
        false,
        false,
    ]);
    CanFrame::data_frame(pack_can_id(node, CommandId::GripperDataPack, false), &p)
}

/// Watchdog config (cmd 15): u32 timeout ms + action byte.
pub fn encode_watchdog(node: NodeId, timeout_ms: u32, action: WatchdogAction) -> CanFrame {
    let action_byte = match action {
        WatchdogAction::Idle => 0u8,
    };
    let mut p = [0u8; 5];
    p[0..4].copy_from_slice(&pack_u32(timeout_ms));
    p[4] = action_byte;
    CanFrame::data_frame(pack_can_id(node, CommandId::Watchdog, false), &p)
}

fn two_f32_frame(node: NodeId, cmd: CommandId, a: f32, b: f32) -> CanFrame {
    let mut p = [0u8; 8];
    p[0..4].copy_from_slice(&pack_f32(a));
    p[4..8].copy_from_slice(&pack_f32(b));
    CanFrame::data_frame(pack_can_id(node, cmd, false), &p)
}

/// Limits (cmd 20): f32 velocity limit \[ticks/s\] + f32 current limit \[mA\].
pub fn encode_limits(node: NodeId, velocity_limit_ticks_s: f32, current_limit_ma: f32) -> CanFrame {
    two_f32_frame(
        node,
        CommandId::Limits,
        velocity_limit_ticks_s,
        current_limit_ma,
    )
}

/// Voltage limit (cmd 34): u32 mV, 0 = use VBUS.
pub fn encode_voltage_limit(node: NodeId, limit_mv: u32) -> CanFrame {
    CanFrame::data_frame(
        pack_can_id(node, CommandId::VoltageLimit, false),
        &pack_u32(limit_mv),
    )
}

/// Impedance PD gains (cmd 16): f32 KP + f32 KD.
pub fn encode_pd_gains(node: NodeId, kp: f32, kd: f32) -> CanFrame {
    two_f32_frame(node, CommandId::PdGains, kp, kd)
}

/// Current-loop gains (cmd 17): f32 KPIQ + f32 KIIQ.
pub fn encode_current_gains(node: NodeId, kpiq: f32, kiiq: f32) -> CanFrame {
    two_f32_frame(node, CommandId::CurrentGains, kpiq, kiiq)
}

/// Velocity-loop gains (cmd 18): f32 KPV + f32 KIV.
pub fn encode_velocity_gains(node: NodeId, kpv: f32, kiv: f32) -> CanFrame {
    two_f32_frame(node, CommandId::VelocityGains, kpv, kiv)
}

/// Position-loop gain (cmd 19): f32 KPP (DLC 4).
pub fn encode_position_gains(node: NodeId, kpp: f32) -> CanFrame {
    CanFrame::data_frame(
        pack_can_id(node, CommandId::PositionGains, false),
        &pack_f32(kpp),
    )
}

/// Torque constant set (cmd 22): f32 Nm/A.
pub fn encode_kt(node: NodeId, kt_nm_a: f32) -> CanFrame {
    CanFrame::data_frame(pack_can_id(node, CommandId::Kt, false), &pack_f32(kt_nm_a))
}

/// Heartbeat setup (cmd 30): u32 period ms.
pub fn encode_heartbeat_setup(node: NodeId, period_ms: u32) -> CanFrame {
    CanFrame::data_frame(
        pack_can_id(node, CommandId::HeartbeatSetup, false),
        &pack_u32(period_ms),
    )
}

/// Set CAN id (cmd 11): u8 new node id.
pub fn encode_set_can_id(node: NodeId, new_id: u8) -> CanFrame {
    CanFrame::data_frame(pack_can_id(node, CommandId::SetCanId, false), &[new_id])
}

/// ESTOP (cmd 0, data DLC 0).
pub fn encode_estop(node: NodeId) -> CanFrame {
    CanFrame::data_frame(pack_can_id(node, CommandId::Estop, false), &[])
}

/// Clear_Error (cmd 1, data DLC 0).
pub fn encode_clear_error(node: NodeId) -> CanFrame {
    CanFrame::data_frame(pack_can_id(node, CommandId::ClearError, false), &[])
}

/// Idle (cmd 12, data DLC 0).
pub fn encode_idle(node: NodeId) -> CanFrame {
    CanFrame::data_frame(pack_can_id(node, CommandId::Idle, false), &[])
}

/// Save_config (cmd 13, data DLC 0).
pub fn encode_save_config(node: NodeId) -> CanFrame {
    CanFrame::data_frame(pack_can_id(node, CommandId::SaveConfig, false), &[])
}

/// Reset (cmd 14, data DLC 0) — reboots the driver into its bootloader.
pub fn encode_reset(node: NodeId) -> CanFrame {
    CanFrame::data_frame(pack_can_id(node, CommandId::Reset, false), &[])
}

/// The command an RTR telemetry poll of `kind` targets.
pub fn poll_command(kind: PollKind) -> CommandId {
    match kind {
        PollKind::Temperature => CommandId::Temperature,
        PollKind::Voltage => CommandId::Voltage,
        PollKind::Errors => CommandId::StateOfErrors,
        PollKind::DeviceInfo => CommandId::DeviceInfo,
        PollKind::Encoder => CommandId::EncoderData,
        PollKind::Kt => CommandId::RespondKt,
        PollKind::Ping => CommandId::Ping,
    }
}

/// Encode the RTR telemetry poll for a [`PollKind`].
pub fn encode_poll(node: NodeId, kind: PollKind) -> CanFrame {
    CanFrame::rtr_frame(pack_can_id(node, poll_command(kind), false))
}

/// Encode an RTR poll for any pollable command
/// (10/23/24/25/26/27/28/33 — [`PollKind`] plus Iq data).
pub fn encode_rtr_poll(node: NodeId, cmd: CommandId) -> Result<CanFrame, EncodeError> {
    use CommandId::*;
    match cmd {
        Ping | Temperature | Voltage | DeviceInfo | StateOfErrors | IqData | EncoderData
        | RespondKt => Ok(CanFrame::rtr_frame(pack_can_id(node, cmd, false))),
        other => Err(EncodeError::NotPollable { cmd: other.raw() }),
    }
}

// ---------------------------------------------------------------------------
// Decoding (driver → host)
// ---------------------------------------------------------------------------

/// A frame that must be discarded whole. Every variant still carries the
/// arbitration-id fields, so the RX drain can harvest the live fault bit
/// (`err_bit`) and bookkeeping ids before dropping the frame — the spec
/// harvests them BEFORE payload dispatch.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum DecodeError {
    /// cmds 5/7: reserved reply packs — never decode.
    #[error("node {node} cmd {cmd}: reserved reply — do not decode")]
    Reserved {
        /// Source node.
        node: NodeId,
        /// Raw command id.
        cmd: u8,
        /// Arbitration-id err bit.
        err_bit: bool,
    },
    /// A known command that is not a driver→host reply (host commands
    /// echoed back, our own RTR requests, bootloader frames aliasing
    /// application ids).
    #[error("node {node} cmd {cmd}: not a driver reply")]
    NotAReply {
        /// Source node.
        node: NodeId,
        /// Raw command id.
        cmd: u8,
        /// Arbitration-id err bit.
        err_bit: bool,
    },
    /// A command id outside the table.
    #[error("node {node} cmd {cmd}: unknown command")]
    UnknownCommand {
        /// Source node.
        node: NodeId,
        /// Raw command id.
        cmd: u8,
        /// Arbitration-id err bit.
        err_bit: bool,
    },
    /// Known reply with the wrong payload length — the whole frame is
    /// discarded, never partially applied.
    #[error("node {node} cmd {cmd}: dlc {dlc}, expected {expected} — frame discarded")]
    WrongDlc {
        /// Source node.
        node: NodeId,
        /// Raw command id.
        cmd: u8,
        /// Arbitration-id err bit.
        err_bit: bool,
        /// Received payload length.
        dlc: u8,
        /// Required payload length.
        expected: u8,
    },
}

impl DecodeError {
    /// Source node of the refused frame.
    pub fn node(&self) -> NodeId {
        match *self {
            Self::Reserved { node, .. }
            | Self::NotAReply { node, .. }
            | Self::UnknownCommand { node, .. }
            | Self::WrongDlc { node, .. } => node,
        }
    }

    /// Arbitration-id err bit of the refused frame — still valid as the
    /// node's live fault signal even though the payload is discarded.
    pub fn err_bit(&self) -> bool {
        match *self {
            Self::Reserved { err_bit, .. }
            | Self::NotAReply { err_bit, .. }
            | Self::UnknownCommand { err_bit, .. }
            | Self::WrongDlc { err_bit, .. } => err_bit,
        }
    }
}

/// Decoded payload of one driver→host frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Payload {
    /// cmd 3: i24 position, i24 speed, i16 current.
    Motion {
        /// Motor position \[encoder ticks\].
        position_ticks: i32,
        /// Motor speed \[ticks/s\].
        speed_ticks_s: i32,
        /// Motor current \[mA\].
        current_ma: i16,
    },
    /// cmd 28: i32 position, i32 speed.
    Encoder {
        /// Motor position \[encoder ticks\].
        position_ticks: i32,
        /// Motor speed \[ticks/s\].
        speed_ticks_s: i32,
    },
    /// cmd 32: i24 position latched at trigger + hall bits.
    Hall {
        /// Latched motor position \[encoder ticks\].
        position_ticks: i32,
        /// Trigger/pin2/edge bits.
        state: HallState,
    },
    /// cmd 23 reply.
    Temperature {
        /// Driver temperature \[°C\].
        deg_c: i16,
    },
    /// cmd 24 reply.
    Voltage {
        /// Bus voltage \[mV\].
        mv: i16,
    },
    /// cmd 27 reply.
    IqCurrent {
        /// Iq current \[mA\].
        ma: i16,
    },
    /// cmd 26 reply.
    Errors(ErrorFlags),
    /// cmd 25 reply.
    DeviceInfo(DeviceInfo),
    /// cmd 33 reply.
    Kt {
        /// Torque constant \[Nm/A\].
        nm_per_a: f32,
    },
    /// cmd 60 reply (firmware gripper).
    Gripper(GripperReply),
    /// cmd 10 reply (liveness).
    Ping,
    /// cmd 9 (periodic heartbeat, if enabled via cmd 30).
    Heartbeat,
}

/// One successfully decoded driver→host frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DecodedFrame {
    /// Source node from the arbitration id.
    pub node: NodeId,
    /// Live fault bit from the arbitration id — set by the driver on
    /// EVERY reply while it has an active fault.
    pub err_bit: bool,
    /// Decoded payload.
    pub payload: Payload,
}

fn expect_dlc(
    frame: &CanFrame,
    node: NodeId,
    cmd: u8,
    err_bit: bool,
    expected: u8,
) -> Result<(), DecodeError> {
    if frame.dlc != expected {
        return Err(DecodeError::WrongDlc {
            node,
            cmd,
            err_bit,
            dlc: frame.dlc,
            expected,
        });
    }
    Ok(())
}

fn object_detection_from_bits(hi: bool, lo: bool) -> ObjectDetection {
    match (hi, lo) {
        (false, false) => ObjectDetection::Moving,
        (false, true) => ObjectDetection::DetectedClosing,
        (true, false) => ObjectDetection::DetectedOpening,
        (true, true) => ObjectDetection::ReachedNoObject,
    }
}

/// Decode one driver→host frame into the frozen [`crate::types`] model.
///
/// Refusals ([`DecodeError`]) always mean whole-frame discard; they still
/// carry `node`/`err_bit` so the drain can harvest the live fault bit
/// first, as the spec requires.
pub fn decode_frame(frame: &CanFrame) -> Result<DecodedFrame, DecodeError> {
    let (node, raw_cmd, err_bit) = unpack_can_id(frame.id);
    let Some(cmd) = CommandId::from_raw(raw_cmd) else {
        return Err(DecodeError::UnknownCommand {
            node,
            cmd: raw_cmd,
            err_bit,
        });
    };
    // A remote frame is a request (ours, looped back, or aliased) — no
    // driver reply is ever RTR.
    if frame.rtr {
        return Err(DecodeError::NotAReply {
            node,
            cmd: raw_cmd,
            err_bit,
        });
    }
    let d = frame.payload();
    let payload = match cmd {
        CommandId::RespondDataPack1 => {
            expect_dlc(frame, node, raw_cmd, err_bit, 8)?;
            Payload::Motion {
                position_ticks: unpack_i24([d[0], d[1], d[2]]),
                speed_ticks_s: unpack_i24([d[3], d[4], d[5]]),
                current_ma: unpack_i16([d[6], d[7]]),
            }
        }
        CommandId::EncoderData => {
            expect_dlc(frame, node, raw_cmd, err_bit, 8)?;
            Payload::Encoder {
                position_ticks: unpack_i32([d[0], d[1], d[2], d[3]]),
                speed_ticks_s: unpack_i32([d[4], d[5], d[6], d[7]]),
            }
        }
        CommandId::RespondDataHall => {
            expect_dlc(frame, node, raw_cmd, err_bit, 4)?;
            let bits = unfold_bits_msb_first(d[3]);
            Payload::Hall {
                position_ticks: unpack_i24([d[0], d[1], d[2]]),
                state: HallState {
                    trigger: bits[0],
                    pin2: bits[1],
                    edge: bits[2],
                },
            }
        }
        CommandId::Temperature => {
            expect_dlc(frame, node, raw_cmd, err_bit, 2)?;
            Payload::Temperature {
                deg_c: unpack_i16([d[0], d[1]]),
            }
        }
        CommandId::Voltage => {
            expect_dlc(frame, node, raw_cmd, err_bit, 2)?;
            Payload::Voltage {
                mv: unpack_i16([d[0], d[1]]),
            }
        }
        CommandId::IqData => {
            expect_dlc(frame, node, raw_cmd, err_bit, 2)?;
            Payload::IqCurrent {
                ma: unpack_i16([d[0], d[1]]),
            }
        }
        CommandId::StateOfErrors => {
            expect_dlc(frame, node, raw_cmd, err_bit, 2)?;
            let b0 = unfold_bits_msb_first(d[0]);
            let b1 = unfold_bits_msb_first(d[1]);
            Payload::Errors(ErrorFlags {
                error: b0[0],
                temperature: b0[1],
                encoder: b0[2],
                vbus: b0[3],
                driver: b0[4],
                velocity: b0[5],
                current: b0[6],
                estop: b0[7],
                calibrated: b1[0],
                activated: b1[1],
                watchdog: b1[2],
            })
        }
        CommandId::DeviceInfo => {
            expect_dlc(frame, node, raw_cmd, err_bit, 7)?;
            Payload::DeviceInfo(DeviceInfo {
                hw_ver: d[0],
                batch: d[1],
                sw_ver: d[2],
                serial: unpack_i32([d[3], d[4], d[5], d[6]]),
            })
        }
        CommandId::RespondKt => {
            expect_dlc(frame, node, raw_cmd, err_bit, 4)?;
            Payload::Kt {
                nm_per_a: unpack_f32([d[0], d[1], d[2], d[3]]),
            }
        }
        CommandId::RespondGripperData => {
            expect_dlc(frame, node, raw_cmd, err_bit, 4)?;
            let bits = unfold_bits_msb_first(d[3]);
            Payload::Gripper(GripperReply {
                position: d[0],
                current_ma: unpack_i16([d[1], d[2]]),
                activated: bits[0],
                action_status: bits[1],
                object_detection: object_detection_from_bits(bits[2], bits[3]),
                temperature_error: bits[4],
                timeout_error: bits[5],
                estop_error: bits[6],
                calibrated: bits[7],
            })
        }
        // No payload; the vendor stack accepts any DLC for these.
        CommandId::Ping => Payload::Ping,
        CommandId::RespondHeartbeat => Payload::Heartbeat,
        CommandId::RespondDataPack2 | CommandId::RespondDataPack3 => {
            return Err(DecodeError::Reserved {
                node,
                cmd: raw_cmd,
                err_bit,
            })
        }
        // Host→driver commands — receiving one is never a reply.
        CommandId::Estop
        | CommandId::ClearError
        | CommandId::DataPack1
        | CommandId::DataPackPd
        | CommandId::SetCanId
        | CommandId::Idle
        | CommandId::SaveConfig
        | CommandId::Reset
        | CommandId::Watchdog
        | CommandId::PdGains
        | CommandId::CurrentGains
        | CommandId::VelocityGains
        | CommandId::PositionGains
        | CommandId::Limits
        | CommandId::Kt
        | CommandId::HeartbeatSetup
        | CommandId::DataPackHall
        | CommandId::VoltageLimit
        | CommandId::GripperDataPack
        | CommandId::GripperCalibrate => {
            return Err(DecodeError::NotAReply {
                node,
                cmd: raw_cmd,
                err_bit,
            })
        }
    };
    Ok(DecodedFrame {
        node,
        err_bit,
        payload,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Tiny deterministic LCG so the roundtrip sweeps cover a spread of
    /// values without a proptest dependency.
    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self) -> u64 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            self.0
        }
    }

    #[test]
    fn primitive_roundtrips_cover_bounds_and_random_values() {
        let mut rng = Lcg(0x5EED);
        // i24: full-range boundaries + sweep. Out-of-range wraps mod 2^24
        // (vendor & 0xFFFFFF), so only in-range values roundtrip.
        for v in [-(1 << 23), -1, 0, 1, (1 << 23) - 1, -150, 12345] {
            assert_eq!(unpack_i24(pack_i24(v)), v, "i24 {v}");
        }
        for _ in 0..1000 {
            let v = (rng.next() as i32) % (1 << 23);
            assert_eq!(unpack_i24(pack_i24(v)), v, "i24 {v}");
        }
        // Wrap behavior at the i24 boundary matches two's complement.
        assert_eq!(unpack_i24(pack_i24(1 << 23)), -(1 << 23));
        assert_eq!(unpack_i24(pack_i24(-(1 << 23) - 1)), (1 << 23) - 1);
        for v in [i16::MIN, -1, 0, 1, i16::MAX] {
            assert_eq!(unpack_i16(pack_i16(v)), v);
        }
        for v in [i32::MIN, -1, 0, 1, i32::MAX] {
            assert_eq!(unpack_i32(pack_i32(v)), v);
        }
        for v in [0u32, 1, u32::MAX] {
            assert_eq!(unpack_u32(pack_u32(v)), v);
        }
        for _ in 0..1000 {
            let v = rng.next() as i32;
            assert_eq!(unpack_i32(pack_i32(v)), v);
            assert_eq!(unpack_i16(pack_i16(v as i16)), v as i16);
            let f = f32::from_bits(rng.next() as u32);
            if f.is_nan() {
                continue;
            }
            assert_eq!(unpack_f32(pack_f32(f)), f);
        }
        // Bitfield fold/unfold: index 0 = bit 7 (MSB-first).
        for _ in 0..256 {
            let b = rng.next() as u8;
            assert_eq!(fold_bits_msb_first(unfold_bits_msb_first(b)), b);
        }
        let mut bits = [false; 8];
        bits[0] = true;
        assert_eq!(fold_bits_msb_first(bits), 0x80, "index 0 must be bit 7");
    }

    #[test]
    fn can_id_roundtrips_over_full_field_ranges() {
        for node in 0u8..16 {
            for raw in 0u8..64 {
                let Some(cmd) = CommandId::from_raw(raw) else {
                    continue;
                };
                assert_eq!(cmd.raw(), raw);
                for err in [false, true] {
                    let id = pack_can_id(node, cmd, err);
                    assert!(id < 1 << 11, "11-bit id");
                    assert_eq!(unpack_can_id(id), (node, raw, err));
                }
            }
        }
    }

    #[test]
    fn cmd2_channel_semantics_select_dlc_and_refuse_the_rest() {
        // All-None: no frame at all (active idle is an explicit vel=0 cmd).
        assert_eq!(
            encode_joint_command(
                0,
                &JointCommand {
                    pos: None,
                    vel: None,
                    cur_ma: None,
                    pack: Pack::Pid
                }
            ),
            Ok(None)
        );
        // Position without velocity has no wire form.
        assert_eq!(
            encode_joint_command(
                0,
                &JointCommand {
                    pos: Some(10),
                    vel: None,
                    cur_ma: Some(5),
                    pack: Pack::Pid
                }
            ),
            Err(EncodeError::PositionWithoutVelocity)
        );
        // Current None substitutes 0 — the frame is still sent (DLC keeps
        // the mode), and the current bytes are zero.
        let f = encode_joint_command(
            1,
            &JointCommand {
                pos: None,
                vel: Some(-40000),
                cur_ma: None,
                pack: Pack::Pid,
            },
        )
        .unwrap()
        .unwrap();
        assert_eq!(f.dlc, 5);
        assert_eq!(&f.payload()[3..5], &[0, 0]);
        // DLC ladder 8/5/2 from the Option channels.
        for (cmd, dlc) in [
            (JointCommand::position(1, 2, 3), 8),
            (JointCommand::velocity(1, 2), 5),
            (JointCommand::current(3), 2),
        ] {
            let f = encode_joint_command(0, &cmd).unwrap().unwrap();
            assert_eq!(f.dlc, dlc);
            assert_eq!(unpack_can_id(f.id).1, 2);
        }
        // Firmware idle: cmd 12, DLC 0, a DATA frame (not RTR) — the
        // driver drops its loop and sends no reply.
        let f = encode_joint_command(3, &JointCommand::drop_to_idle())
            .unwrap()
            .unwrap();
        assert!(!f.rtr);
        assert_eq!((f.dlc, unpack_can_id(f.id)), (0, (3, 12, false)));
        // Encoder poll: cmd 28 as an RTR request.
        let f = encode_joint_command(3, &JointCommand::encoder_poll())
            .unwrap()
            .unwrap();
        assert!(f.rtr);
        assert_eq!((f.dlc, unpack_can_id(f.id)), (0, (3, 28, false)));
    }

    #[test]
    fn encode_then_decode_roundtrips_reply_shaped_values() {
        // The codec's own encoders are host→driver only, so reply
        // roundtrips go through hand-packed frames: pack primitives →
        // decode_frame → compare. Randomized in-range sweep.
        let mut rng = Lcg(0xCA5CADE);
        for _ in 0..500 {
            let pos = (rng.next() as i32) % (1 << 23);
            let spd = (rng.next() as i32) % (1 << 23);
            let cur = rng.next() as i16;
            let mut p = [0u8; 8];
            p[0..3].copy_from_slice(&pack_i24(pos));
            p[3..6].copy_from_slice(&pack_i24(spd));
            p[6..8].copy_from_slice(&pack_i16(cur));
            let node = (rng.next() % 16) as u8;
            let err = rng.next() % 2 == 1;
            let id = pack_can_id(node, CommandId::RespondDataPack1, err);
            let decoded = decode_frame(&CanFrame::data_frame(id, &p)).unwrap();
            assert_eq!(
                decoded,
                DecodedFrame {
                    node,
                    err_bit: err,
                    payload: Payload::Motion {
                        position_ticks: pos,
                        speed_ticks_s: spd,
                        current_ma: cur,
                    },
                }
            );
        }
    }

    #[test]
    fn gripper_slot_variants_produce_the_spec_frames() {
        // DLC-0 empty poll ≠ RTR: it is a DATA frame (feeds the watchdog
        // without overwriting the in-progress command).
        let poll = encode_gripper_command(6, 13, &GripperCommand::FirmwarePoll)
            .unwrap()
            .unwrap();
        assert!(!poll.rtr);
        assert_eq!((poll.dlc, unpack_can_id(poll.id)), (0, (6, 61, false)));
        let cal = encode_gripper_command(6, 13, &GripperCommand::Calibrate)
            .unwrap()
            .unwrap();
        assert_eq!((cal.dlc, unpack_can_id(cal.id).1), (0, 62));
        // No gripper: RTR ping to the timing dummy keeps the cadence.
        let ping = encode_gripper_command(6, 13, &GripperCommand::NoGripper)
            .unwrap()
            .unwrap();
        assert!(ping.rtr);
        assert_eq!(unpack_can_id(ping.id), (13, 10, false));
        // Motor mode targets the gripper node like a 7th joint.
        let m = encode_gripper_command(6, 13, &GripperCommand::Motor(JointCommand::idle()))
            .unwrap()
            .unwrap();
        assert_eq!(unpack_can_id(m.id), (6, 2, false));
        assert_eq!(m.dlc, 5);
    }

    #[test]
    fn rtr_poll_whitelist_refuses_non_telemetry_commands() {
        for cmd in [
            CommandId::Ping,
            CommandId::Temperature,
            CommandId::Voltage,
            CommandId::DeviceInfo,
            CommandId::StateOfErrors,
            CommandId::IqData,
            CommandId::EncoderData,
            CommandId::RespondKt,
        ] {
            let f = encode_rtr_poll(3, cmd).unwrap();
            assert!(f.rtr);
            assert_eq!(f.dlc, 0);
        }
        assert_eq!(
            encode_rtr_poll(3, CommandId::DataPack1),
            Err(EncodeError::NotPollable { cmd: 2 })
        );
        assert_eq!(
            encode_rtr_poll(3, CommandId::Reset),
            Err(EncodeError::NotPollable { cmd: 14 })
        );
    }

    #[test]
    fn decode_refusals_keep_the_err_bit_harvestable() {
        // Reserved cmd 5 with the fault bit set: the payload is refused
        // but the live fault signal must survive.
        let id = pack_can_id(4, CommandId::RespondDataPack2, true);
        let err = decode_frame(&CanFrame::data_frame(id, &[1, 2, 3])).unwrap_err();
        assert_eq!(err.node(), 4);
        assert!(err.err_bit());
        assert!(matches!(err, DecodeError::Reserved { cmd: 5, .. }));
        // Wrong DLC likewise.
        let id = pack_can_id(2, CommandId::RespondDataPack1, true);
        let err = decode_frame(&CanFrame::data_frame(id, &[0; 5])).unwrap_err();
        assert!(err.err_bit());
        assert!(matches!(
            err,
            DecodeError::WrongDlc {
                dlc: 5,
                expected: 8,
                ..
            }
        ));
    }
}
