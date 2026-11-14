//! Frame codec: arbitration ids, big-endian payload primitives, and the
//! full Spectral/STEPFOC command table — encoding for every host→driver
//! frame, decoding for every driver→host reply.
//!
//! Everything here is pure and allocation-free ([`CanFrame`] is a fixed
//! 8-byte buffer), so the RT tick path can call it directly.

use par6_config::WatchdogAction;

use crate::types::{
    DeviceInfo, DriveRange, ErrorFlags, FirmwareGripperCommand, GripperCommand, GripperReply,
    HallState, Identity, JointCommand, NodeId, ObjectDetection, Pack, PollKind,
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
    /// 36 — Param_Set (DLC 5: u8 parameter, four value bytes; firmware 109).
    /// No answer: an unknown parameter or a refused value changes nothing.
    ParamSet = 36,
    /// 37 — Param_Get (DLC 1: u8 parameter), answered on cmd 38.
    ParamGet = 37,
    /// 38 — Param_Value (D→H DLC 5: the parameter and its value in force;
    /// DLC 1: a parameter the drive does not have).
    ParamValue = 38,
    /// 39 — Identity (RTR; reply DLC 8: product, tool id, u16 firmware
    /// version, hardware version, pole pairs, calibrated, flags).
    Identity = 39,
    /// 40 — Telemetry (RTR; reply DLC 8: i16 °C, i16 mV, the two
    /// State_of_Errors bytes, i16 mA).
    Telemetry = 40,
    /// 41 — Buffer_Read (DLC 5: u8 kind, u16 first index, u16 count). The
    /// drive answers one cmd 42 frame per index from its main loop, as fast
    /// as the bus takes them; a new request replaces one in progress.
    BufferRead = 41,
    /// 42 — Buffer_Data (D→H DLC 8: u16 `kind << 13 | index`, then six
    /// bytes by kind).
    BufferData = 42,
    /// 43 — Buffer_Write (DLC 8, the cmd 42 layout; the anti-cogging kinds).
    BufferWrite = 43,
    /// 44 — Sound (H→D DLC 8: u8 operation, u8 channel, then three u16:
    /// pitch \[Hz\], loudness \[mA\], time \[ms\]). Played through the motor
    /// of a drive in closed loop; no reply, and it does not feed the
    /// watchdog.
    Sound = 44,
    /// 46 — Range (DLC 8: i32 min, i32 max drive position \[ticks\]; DLC 0
    /// disarms; read with an RTR, answered DLC 8 armed or DLC 0 not; par6
    /// firmware). Past either end while still moving out, the drive trips
    /// (error byte 1 b3) and brakes its motor.
    Range = 46,
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
            36 => ParamSet,
            37 => ParamGet,
            38 => ParamValue,
            39 => Identity,
            40 => Telemetry,
            41 => BufferRead,
            42 => BufferData,
            43 => BufferWrite,
            44 => Sound,
            46 => Range,
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

/// The two State_of_Errors bytes (cmd 26, and bytes 4–5 of cmd 40).
fn unpack_error_flags(b0: u8, b1: u8) -> ErrorFlags {
    let b0 = unfold_bits_msb_first(b0);
    let b1 = unfold_bits_msb_first(b1);
    ErrorFlags {
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
        wrong_product: b1[3],
        range: b1[4],
    }
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

/// The byte a watchdog action travels as (cmd 15 `data[4]`, parameter 7).
pub fn watchdog_action_byte(action: WatchdogAction) -> u8 {
    match action {
        WatchdogAction::Idle => 0,
        WatchdogAction::Hold => 1,
        WatchdogAction::Brake => 2,
        WatchdogAction::Coast => 3,
        WatchdogAction::Descend => 4,
    }
}

/// Watchdog config (cmd 15): u32 timeout ms + action byte.
pub fn encode_watchdog(node: NodeId, timeout_ms: u32, action: WatchdogAction) -> CanFrame {
    let mut p = [0u8; 5];
    p[0..4].copy_from_slice(&pack_u32(timeout_ms));
    p[4] = watchdog_action_byte(action);
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

/// Drive parameters (cmds 36–38), by the numbers the firmware fixes for
/// good. A value travels as four big-endian bytes: an i32, or for the gains
/// and limits a float's bit pattern.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum Param {
    /// The tool the drive is built into, 0 = none; kept by Save_config.
    ToolId = 1,
    /// How long a watchdog hold or descend may run \[ms\], 0..=60000.
    WatchdogFallbackMs = 2,
    /// The current a watchdog descend applies \[mA\], signed.
    WatchdogFallbackCurrentMa = 3,
    /// The speed a watchdog descend is capped at \[ticks/s\], 0..=65535.
    WatchdogFallbackSpeed = 4,
    /// What follows a hold or descend: 0 as before firmware 109, 2 brake,
    /// 3 coast.
    WatchdogAfter = 5,
    /// Watchdog time \[ms\] (what cmd 15 sets).
    WatchdogMs = 6,
    /// Watchdog action byte (what cmd 15 sets).
    WatchdogAction = 7,
    /// Position gain (float).
    Kpp = 8,
    /// Velocity P gain (float).
    Kpv = 9,
    /// Velocity I gain (float).
    Kiv = 10,
    /// Current P gain (float).
    Kpiq = 11,
    /// Current I gain (float).
    Kiiq = 12,
    /// Impedance stiffness (float).
    Kp = 13,
    /// Impedance damping (float).
    Kd = 14,
    /// Velocity limit \[ticks/s\] (float).
    VelocityLimit = 15,
    /// Current limit \[mA\] (float).
    CurrentLimit = 16,
    /// Voltage limit \[mV\], 0 = bus voltage.
    VoltageLimitMv = 17,
    /// The signal capture channel A records ([`CaptureSignal`]).
    CaptureSourceA = 24,
    /// Channel B's.
    CaptureSourceB = 25,
    /// Channel C's.
    CaptureSourceC = 26,
    /// Control loops per captured sample, 1..=65535.
    CaptureDivider = 27,
    /// Writing [`CAPTURE_TRIGGER_NOW`] records a full capture from the next
    /// loop.
    CaptureTrigger = 28,
    /// Samples kept from before the trigger.
    CapturePretrigger = 30,
    /// Samples a read-out holds, 0 to [`CAPTURE_LEN`] (read-only).
    CaptureCount = 32,
    /// Anti-cogging: 0 off, 1 the harmonics, 2 the table.
    CoggingMode = 37,
    /// The drive's own anti-cogging identification: writing
    /// [`COGCAL_RUN_TRIMMED`] starts it, 0 aborts; read as a
    /// [`DriveRun`]-coded state.
    Cogcal = 43,
    /// The cogging current that identification found, rms \[0.1 mA\]
    /// (read-only).
    CogcalRippleX10 = 45,
    /// The speed the loops act on: 0 the moving average, 1 the PLL, 2 the
    /// direct PLL.
    SpeedEstimator = 50,
    /// The speed PLL's natural frequency \[Hz\].
    SpeedPllBandwidthHz = 51,
    /// The drive's own encoder-table measurement: writing [`ENCAL_RUN`]
    /// starts it, 0 aborts; read as a [`DriveRun`]-coded state.
    Encal = 52,
    /// Whether the encoder table corrects the encoder, 1 or 0.
    EncalUse = 53,
    /// The encoder table's error, peak to peak \[0.01 deg of the motor
    /// shaft\] (read-only).
    EncalPeakToPeakX100 = 54,
    /// Its once-per-turn part, amplitude \[0.01 deg\] (read-only).
    EncalOncePerTurnX100 = 55,
    /// How far the last encoder-table run moved the electrical offset
    /// \[0.1 deg electrical\] (read-only).
    EncalOffsetMoveX10 = 57,
    /// The trim on current-sense zero 1 \[ADC counts\].
    ZeroTrim1 = 59,
    /// The trim on current-sense zero 2 \[ADC counts\].
    ZeroTrim2 = 60,
    /// The electrical offset in force \[0.1 deg electrical\] (read-only).
    ThetaOffsetX10 = 61,
    /// The cogging current's first harmonic before the last zero trim
    /// \[0.1 mA\] (read-only).
    TrimFirstHarmonicBeforeX10 = 62,
    /// And after it \[0.1 mA\] (read-only).
    TrimFirstHarmonicAfterX10 = 63,
    /// What is in force and what runs, as bits (read-only).
    Features = 79,
    /// What the drive's sound is doing, as bits; [`SOUND_STATE_PLAYING`]
    /// while a queued tune plays (read-only).
    SoundState = 80,
    /// The axis sound is added on: 1 the q axis, a torque ripple at the
    /// pitch and what a drive starts with; 0 the d axis, no torque.
    SoundAxis = 82,
    /// The git commit the firmware was built from, seven hex digits
    /// (read-only).
    BuildCommit = 84,
    /// The board: its chip's unique number folded into 32 bits (read-only).
    BoardId = 86,
    /// A number drawn anew at every successful calibration, 0 before the
    /// first on firmware 109 (read-only).
    CalibrationId = 87,
}

/// Writing this to [`Param::Encal`] measures the encoder table and puts it
/// in use, in RAM; a Save_config keeps it.
pub const ENCAL_RUN: i32 = 1;
/// Writing this to [`Param::Cogcal`] identifies the anti-cogging with the
/// current-sense zero trim and switches its harmonics on, in RAM.
pub const COGCAL_RUN_TRIMMED: i32 = 4;

/// Where one of the drive's own calibration runs stands, from the state it
/// answers on [`Param::Encal`] or [`Param::Cogcal`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveRun {
    /// Nothing has run since the drive started.
    Idle,
    /// The motor is turning for it.
    Running,
    /// It finished and its result is in use.
    Done,
    /// It finished; the anti-cogging is in use without the zero trim, which
    /// did not lower the first harmonic.
    DoneUntrimmed,
    /// It ended without a result; the state code says why.
    Failed(u8),
}

impl DriveRun {
    /// The state behind an answer to [`Param::Encal`].
    pub fn of_encal(state: i32) -> Self {
        match state {
            0 => Self::Idle,
            1 | 2 => Self::Running,
            3 => Self::Done,
            other => Self::Failed(other.clamp(0, 255) as u8),
        }
    }

    /// The state behind an answer to [`Param::Cogcal`].
    pub fn of_cogcal(state: i32) -> Self {
        match state {
            0 => Self::Idle,
            1 | 2 | 7 => Self::Running,
            3 => Self::Done,
            11 => Self::DoneUntrimmed,
            other => Self::Failed(other.clamp(0, 255) as u8),
        }
    }

    /// Why a failed run says it failed.
    pub fn failure(self) -> Option<&'static str> {
        match self {
            Self::Failed(4) => Some("a command reached the drive during the run"),
            Self::Failed(5) => Some("the drive was in error or is not calibrated"),
            Self::Failed(6) => Some("the rotor did not follow"),
            Self::Failed(7) => Some(
                "the encoder table came out implausible (over 4 deg, or the offset moved over 60 deg)",
            ),
            Self::Failed(8) => Some("the encoder table of a combined run failed"),
            Self::Failed(9) => Some("the zero trim did not lower the first harmonic"),
            Self::Failed(10) => Some("switched off: the encoder table was measured again since"),
            Self::Failed(_) => Some("the drive gave a state this runtime does not know"),
            _ => None,
        }
    }
}

/// One note of a tune a drive plays through its motor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Note {
    /// Pitch \[Hz\], [`SOUND_HZ`]; 0 is a rest.
    pub hz: u16,
    /// How long it sounds \[ms\].
    pub ms: u16,
}

/// The pitches a drive can play \[Hz\]: below half its 6250 Hz loop.
pub const SOUND_HZ: std::ops::RangeInclusive<u16> = 20..=3124;
/// The notes one tune may hold: what a drive's queue does.
pub const SOUND_NOTES: usize = 31;
/// The bit of [`Param::SoundState`] that is set while a queued tune plays.
pub const SOUND_STATE_PLAYING: i32 = 1 << 25;
/// Sound operations (cmd 44 byte 0): stop, queue a note, play the queue.
pub const SOUND_STOP: u8 = 0;
/// Queue one note.
pub const SOUND_QUEUE: u8 = 2;
/// Play what is queued.
pub const SOUND_PLAY: u8 = 3;

fn sound_frame(node: NodeId, op: u8, hz: u16, ma: u16, ms: u16) -> CanFrame {
    let (hz, ma, ms) = (hz.to_be_bytes(), ma.to_be_bytes(), ms.to_be_bytes());
    CanFrame::data_frame(
        pack_can_id(node, CommandId::Sound, false),
        &[op, 0, hz[0], hz[1], ma[0], ma[1], ms[0], ms[1]],
    )
}

/// Whether a drive can play `notes` as one tune.
pub fn check_tune(notes: &[Note]) -> Result<(), crate::BusError> {
    if notes.len() > SOUND_NOTES {
        return Err(crate::BusError::InvalidCommand {
            reason: "more notes than a drive queues",
        });
    }
    if notes.iter().any(|n| n.hz != 0 && !SOUND_HZ.contains(&n.hz)) {
        return Err(crate::BusError::InvalidCommand {
            reason: "a pitch outside what a drive can play (20 to 3124 Hz)",
        });
    }
    Ok(())
}

/// Sound (cmd 44): stop whatever the drive is playing and empty its queue.
pub fn encode_sound_stop(node: NodeId) -> CanFrame {
    sound_frame(node, SOUND_STOP, 0, 0, 0)
}

/// Sound (cmd 44): the frames that queue `notes` at `loudness_ma` and start
/// them, one after another on one voice, in place of whatever it was
/// playing. The drive times them itself, so the tune is as even as its
/// control loop however the frames arrive, and falls silent after the last.
/// They sound on the d axis, set with every tune because a drive forgets it
/// at a restart: on the q axis the tone is a current the host reads as load
/// on the joint.
pub fn encode_tune(
    node: NodeId,
    notes: &[Note],
    loudness_ma: u16,
) -> impl Iterator<Item = CanFrame> + '_ {
    [
        encode_param_set(node, Param::SoundAxis, 0),
        encode_sound_stop(node),
    ]
    .into_iter()
    .chain(
        notes
            .iter()
            .map(move |n| sound_frame(node, SOUND_QUEUE, n.hz, loudness_ma, n.ms)),
    )
    .chain(std::iter::once(sound_frame(node, SOUND_PLAY, 0, 0, 0)))
}

impl Param {
    /// The parameter number on the wire.
    pub fn raw(self) -> u8 {
        self as u8
    }
}

/// Param_Set (cmd 36): `param` takes `value`. Held in the drive's RAM; the
/// saved ones persist with a Save_config.
pub fn encode_param_set(node: NodeId, param: Param, value: i32) -> CanFrame {
    let v = pack_i32(value);
    CanFrame::data_frame(
        pack_can_id(node, CommandId::ParamSet, false),
        &[param.raw(), v[0], v[1], v[2], v[3]],
    )
}

/// Param_Get (cmd 37): ask for `param`'s value in force, answered on cmd 38.
pub fn encode_param_get(node: NodeId, param: Param) -> CanFrame {
    CanFrame::data_frame(
        pack_can_id(node, CommandId::ParamGet, false),
        &[param.raw()],
    )
}

/// What a drive runs for one configuration frame, put together from the
/// parameters that frame sets (firmware 109). Floats and integers arrive
/// exactly as written, so a read-back compares equal to what was sent when
/// the write landed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Readback {
    /// cmd 15: watchdog timeout \[ms\] and action byte.
    Watchdog {
        /// Timeout \[ms\].
        ms: u32,
        /// Action on expiry.
        action: u8,
    },
    /// cmd 20: velocity limit \[ticks/s\] and current limit \[mA\].
    Limits {
        /// Velocity limit.
        velocity_ticks_s: f32,
        /// Current limit.
        current_ma: f32,
    },
    /// cmd 34: voltage limit \[mV\], 0 = bus voltage.
    VoltageLimit {
        /// The limit.
        mv: u32,
    },
    /// cmd 16: impedance PD gains.
    PdGains {
        /// Stiffness.
        kp: f32,
        /// Damping.
        kd: f32,
    },
    /// cmd 17: current-loop gains.
    CurrentGains {
        /// Proportional.
        kpiq: f32,
        /// Integral.
        kiiq: f32,
    },
    /// cmd 18: velocity-loop gains.
    VelocityGains {
        /// Proportional.
        kpv: f32,
        /// Integral.
        kiv: f32,
    },
    /// cmd 19: position-loop gain.
    PositionGains {
        /// Proportional.
        kpp: f32,
    },
}

impl Readback {
    /// The configuration frame this answers for.
    pub fn kind(self) -> crate::ConfigKind {
        use crate::ConfigKind as K;
        match self {
            Self::Watchdog { .. } => K::Watchdog,
            Self::Limits { .. } => K::Limits,
            Self::VoltageLimit { .. } => K::VoltageLimit,
            Self::PdGains { .. } => K::PdGains,
            Self::CurrentGains { .. } => K::CurrentGains,
            Self::VelocityGains { .. } => K::VelocityGains,
            Self::PositionGains { .. } => K::PositionGains,
        }
    }
}

/// The parameters a configuration frame sets, in the order its payload
/// carries them.
pub fn readback_params(kind: crate::ConfigKind) -> &'static [Param] {
    use crate::ConfigKind as K;
    match kind {
        K::Watchdog => &[Param::WatchdogMs, Param::WatchdogAction],
        K::Limits => &[Param::VelocityLimit, Param::CurrentLimit],
        K::VoltageLimit => &[Param::VoltageLimitMv],
        K::PdGains => &[Param::Kp, Param::Kd],
        K::CurrentGains => &[Param::Kpiq, Param::Kiiq],
        K::VelocityGains => &[Param::Kpv, Param::Kiv],
        K::PositionGains => &[Param::Kpp],
    }
}

impl Readback {
    /// What the drive runs for `kind`, from the parameter values it has
    /// answered with; `None` until every one of the frame's parameters has
    /// been answered.
    pub fn from_params(
        kind: crate::ConfigKind,
        value: impl Fn(Param) -> Option<[u8; 4]>,
    ) -> Option<Self> {
        use crate::ConfigKind as K;
        let float = |p| value(p).map(unpack_f32);
        let int = |p| value(p).map(unpack_u32);
        Some(match kind {
            K::Watchdog => Self::Watchdog {
                ms: int(Param::WatchdogMs)?,
                action: int(Param::WatchdogAction)? as u8,
            },
            K::Limits => Self::Limits {
                velocity_ticks_s: float(Param::VelocityLimit)?,
                current_ma: float(Param::CurrentLimit)?,
            },
            K::VoltageLimit => Self::VoltageLimit {
                mv: int(Param::VoltageLimitMv)?,
            },
            K::PdGains => Self::PdGains {
                kp: float(Param::Kp)?,
                kd: float(Param::Kd)?,
            },
            K::CurrentGains => Self::CurrentGains {
                kpiq: float(Param::Kpiq)?,
                kiiq: float(Param::Kiiq)?,
            },
            K::VelocityGains => Self::VelocityGains {
                kpv: float(Param::Kpv)?,
                kiv: float(Param::Kiv)?,
            },
            K::PositionGains => Self::PositionGains {
                kpp: float(Param::Kpp)?,
            },
        })
    }
}

/// Arm `node`'s joint range over drive positions `[min, max]` \[ticks\]
/// (cmd 46), or disarm it with `None`.
pub fn encode_range(node: NodeId, range: Option<(i32, i32)>) -> CanFrame {
    let id = pack_can_id(node, CommandId::Range, false);
    match range {
        Some((min, max)) => {
            let mut d = [0u8; 8];
            d[..4].copy_from_slice(&min.to_be_bytes());
            d[4..].copy_from_slice(&max.to_be_bytes());
            CanFrame::data_frame(id, &d)
        }
        None => CanFrame::data_frame(id, &[]),
    }
}

/// Samples per channel a capture holds; divisor 1 records every control
/// loop (6250 Hz).
pub const CAPTURE_LEN: u16 = 1024;
/// The velocity channel's unit: ticks/s per count.
pub const CAPTURE_VEL_SCALE: i32 = 16;
/// The capture trigger value that records a full capture from the next loop.
pub const CAPTURE_TRIGGER_NOW: i32 = 1;
/// The buffer kind (cmds 41–43) of the capture: channels A, B, C per sample.
pub const BUFFER_CAPTURE: u8 = 0;
/// The buffer kind of the anti-cogging harmonics: one slot per index.
pub const BUFFER_HARMONICS: u8 = 2;
/// The anti-cogging mode that applies the harmonics.
pub const COGGING_MODE_HARMONICS: i32 = 1;

/// A signal a capture channel can record (parameters 24–26).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CaptureSignal {
    /// Measured Iq \[mA\].
    Iq = 1,
    /// The speed the loops act on, in ticks/s / [`CAPTURE_VEL_SCALE`].
    Velocity16 = 5,
    /// The electrical angle the anti-cogging harmonics turn with, 0..16383
    /// per cycle.
    ElectricalAngle = 8,
}

/// What a capture records, channel by channel: the speed the loop acts on,
/// Iq, and the electrical angle.
pub const CAPTURE_CHANNELS: [CaptureSignal; 3] = [
    CaptureSignal::Velocity16,
    CaptureSignal::Iq,
    CaptureSignal::ElectricalAngle,
];

/// The frames that start a capture of [`CAPTURE_CHANNELS`], one sample every
/// `divisor` control loops from the next loop, [`CAPTURE_LEN`] samples.
pub fn encode_capture(node: NodeId, divisor: u16) -> [CanFrame; 6] {
    let set = |param, value| encode_param_set(node, param, value);
    [
        set(Param::CaptureSourceA, CAPTURE_CHANNELS[0] as i32),
        set(Param::CaptureSourceB, CAPTURE_CHANNELS[1] as i32),
        set(Param::CaptureSourceC, CAPTURE_CHANNELS[2] as i32),
        set(Param::CaptureDivider, i32::from(divisor.max(1))),
        set(Param::CapturePretrigger, 0),
        set(Param::CaptureTrigger, CAPTURE_TRIGGER_NOW),
    ]
}

/// Buffer_Read (cmd 41): have `node` send `count` entries of buffer `kind`
/// from `first`, one cmd 42 frame each, paced by the drive.
pub fn encode_buffer_read(node: NodeId, kind: u8, first: u16, count: u16) -> CanFrame {
    let (f, c) = (first.to_be_bytes(), count.to_be_bytes());
    CanFrame::data_frame(
        pack_can_id(node, CommandId::BufferRead, false),
        &[kind, f[0], f[1], c[0], c[1]],
    )
}

/// Ripple feedforward slots a drive has (its anti-cogging harmonics).
pub const RIPPLE_SLOTS: u8 = 8;
/// The highest harmonic of the electrical angle a slot may hold.
pub const RIPPLE_MAX_HARMONIC: u8 = 32;

/// Buffer_Write of one anti-cogging harmonic (cmd 43): slot `slot` adds
/// `a cos(h angle) + b sin(h angle)` \[mA\] to the velocity and position
/// loops' current once the harmonics are switched on
/// ([`Param::CoggingMode`]); `harmonic` 0 clears the slot.
pub fn encode_ripple(node: NodeId, slot: u8, harmonic: u8, a_ma: i16, b_ma: i16) -> CanFrame {
    let header = (u16::from(BUFFER_HARMONICS) << 13 | u16::from(slot)).to_be_bytes();
    let (a, b) = (a_ma.to_be_bytes(), b_ma.to_be_bytes());
    CanFrame::data_frame(
        pack_can_id(node, CommandId::BufferWrite, false),
        &[header[0], header[1], 0, harmonic, a[0], a[1], b[0], b[1]],
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
        PollKind::Telemetry => CommandId::Telemetry,
        PollKind::Temperature => CommandId::Temperature,
        PollKind::Voltage => CommandId::Voltage,
        PollKind::Errors => CommandId::StateOfErrors,
        PollKind::DeviceInfo => CommandId::DeviceInfo,
        PollKind::Identity => CommandId::Identity,
        PollKind::Encoder => CommandId::EncoderData,
        PollKind::Kt => CommandId::RespondKt,
        PollKind::Ping => CommandId::Ping,
        PollKind::Range => CommandId::Range,
    }
}

/// Encode the RTR telemetry poll for a [`PollKind`].
pub fn encode_poll(node: NodeId, kind: PollKind) -> CanFrame {
    CanFrame::rtr_frame(pack_can_id(node, poll_command(kind), false))
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
    /// cmd 32: live i24 position + hall bits; firmware holds its target at the edge.
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
    /// cmd 38: a parameter's value in force, as its four wire bytes (an
    /// i32, or a float's bit pattern); `None` for a parameter the drive
    /// does not have.
    Param {
        /// The parameter number.
        param: u8,
        /// Its value.
        value: Option<[u8; 4]>,
    },
    /// cmd 39 reply.
    Identity(Identity),
    /// cmd 46 reply: the range the drive enforces.
    Range(DriveRange),
    /// cmd 42, the capture: sample `index` of every channel, oldest first,
    /// in [`CAPTURE_CHANNELS`] order.
    Capture {
        /// Which sample.
        index: u16,
        /// Channels A, B, C.
        samples: [i16; 3],
    },
    /// cmd 42, the anti-cogging harmonics: what slot `slot` holds.
    Harmonic {
        /// Which slot.
        slot: u8,
        /// Harmonic of the electrical angle, 0 = unused.
        harmonic: u8,
        /// Cosine amplitude \[mA\].
        a_ma: i16,
        /// Sine amplitude \[mA\].
        b_ma: i16,
    },
    /// cmd 40 reply: what cmds 23, 24, 26 and 27 answer, in one frame.
    Telemetry {
        /// Driver temperature \[°C\].
        deg_c: i16,
        /// Bus voltage \[mV\].
        mv: i16,
        /// Per-type fault flags.
        flags: ErrorFlags,
        /// Iq current \[mA\].
        ma: i16,
    },
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

/// The 2-bit object-detection status, from its high and low bits.
///
/// The firmware fills its bool array LSB-first and `bitsToByte` maps array
/// index `i` onto bit `7 - i`, so the status value's LOW bit lands on byte
/// bit 5 and its HIGH bit on bit 4 — the reverse of the byte's own
/// high-to-low order. Take the bits by role, never by position.
fn object_detection_from_bits(msb: bool, lsb: bool) -> ObjectDetection {
    match (msb, lsb) {
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
            Payload::Errors(unpack_error_flags(d[0], d[1]))
        }
        CommandId::Telemetry => {
            expect_dlc(frame, node, raw_cmd, err_bit, 8)?;
            Payload::Telemetry {
                deg_c: unpack_i16([d[0], d[1]]),
                mv: unpack_i16([d[2], d[3]]),
                flags: unpack_error_flags(d[4], d[5]),
                ma: unpack_i16([d[6], d[7]]),
            }
        }
        CommandId::ParamValue => {
            if frame.dlc != 1 {
                expect_dlc(frame, node, raw_cmd, err_bit, 5)?;
            }
            Payload::Param {
                param: d[0],
                value: (frame.dlc == 5).then(|| [d[1], d[2], d[3], d[4]]),
            }
        }
        CommandId::Identity => {
            expect_dlc(frame, node, raw_cmd, err_bit, 8)?;
            Payload::Identity(Identity {
                product: d[0],
                tool_id: d[1],
                firmware: u16::from_be_bytes([d[2], d[3]]),
                hw_ver: d[4],
                pole_pairs: d[5],
                calibrated: d[6] == 1,
                flags: d[7],
            })
        }
        CommandId::BufferData => {
            expect_dlc(frame, node, raw_cmd, err_bit, 8)?;
            let header = u16::from_be_bytes([d[0], d[1]]);
            let (kind, index) = ((header >> 13) as u8, header & 0x1FFF);
            let word = |i: usize| unpack_i16([d[i], d[i + 1]]);
            match kind {
                BUFFER_CAPTURE => Payload::Capture {
                    index,
                    samples: [word(2), word(4), word(6)],
                },
                BUFFER_HARMONICS if index < u16::from(RIPPLE_SLOTS) => Payload::Harmonic {
                    slot: index as u8,
                    harmonic: d[3],
                    a_ma: word(4),
                    b_ma: word(6),
                },
                // The histogram and the tables: buffers this runtime never
                // asks for.
                _ => {
                    return Err(DecodeError::NotAReply {
                        node,
                        cmd: raw_cmd,
                        err_bit,
                    })
                }
            }
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
                // bits[3] is byte bit 4 (the value's high bit), bits[2] is
                // bit 5 (its low bit) — see `object_detection_from_bits`.
                object_detection: object_detection_from_bits(bits[3], bits[2]),
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
        CommandId::Range => match frame.payload().len() {
            0 => Payload::Range(DriveRange::Disarmed),
            _ => {
                expect_dlc(frame, node, raw_cmd, err_bit, 8)?;
                Payload::Range(DriveRange::Armed {
                    min_ticks: unpack_i32([d[0], d[1], d[2], d[3]]),
                    max_ticks: unpack_i32([d[4], d[5], d[6], d[7]]),
                })
            }
        },
        // Host→driver commands — receiving one is never a reply.
        CommandId::Estop
        | CommandId::ClearError
        | CommandId::DataPack1
        | CommandId::DataPackPd
        | CommandId::SetCanId
        | CommandId::Idle
        | CommandId::SaveConfig
        | CommandId::Reset
        | CommandId::Kt
        | CommandId::HeartbeatSetup
        | CommandId::DataPackHall
        | CommandId::Watchdog
        | CommandId::PdGains
        | CommandId::CurrentGains
        | CommandId::VelocityGains
        | CommandId::PositionGains
        | CommandId::Limits
        | CommandId::VoltageLimit
        | CommandId::ParamSet
        | CommandId::ParamGet
        | CommandId::BufferRead
        | CommandId::BufferWrite
        | CommandId::Sound
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
        // Round-tripping pack against unpack passes for any pair of
        // matching shifts, so the field positions are pinned to literal
        // ids read off the vendor layout `(node << 7) | (cmd << 1) | err`
        // first. Get a shift wrong and every drive on the bus answers to
        // the wrong arbitration id.
        for (node, cmd, err, id) in [
            (0u8, CommandId::Estop, false, 0x000u16),
            (0, CommandId::DataPack1, false, 0x004),
            (1, CommandId::DataPack1, false, 0x084),
            (2, CommandId::RespondDataPack1, true, 0x107),
            (3, CommandId::Ping, false, 0x194),
            (5, CommandId::Limits, false, 0x2A8),
            (15, CommandId::Reset, true, 0x79D),
        ] {
            assert_eq!(
                pack_can_id(node, cmd, err),
                id,
                "node {node} {cmd:?} err={err}"
            );
            assert_eq!(unpack_can_id(id), (node, cmd.raw(), err), "id {id:#05X}");
        }

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

    #[test]
    fn capture_starts_by_parameter_and_reads_back_a_sample_per_frame() {
        let start = encode_capture(2, 0);
        for f in &start {
            assert_eq!(unpack_can_id(f.id), (2, 36, false));
        }
        // Velocity / 16, Iq, the electrical angle; divisor floored at 1; no
        // pre-trigger; trigger now, last, so the sources are in place.
        assert_eq!(start[0].payload(), &[24, 0, 0, 0, 5]);
        assert_eq!(start[1].payload(), &[25, 0, 0, 0, 1]);
        assert_eq!(start[2].payload(), &[26, 0, 0, 0, 8]);
        assert_eq!(start[3].payload(), &[27, 0, 0, 0, 1]);
        assert_eq!(start[4].payload(), &[30, 0, 0, 0, 0]);
        assert_eq!(start[5].payload(), &[28, 0, 0, 0, 1]);

        let read = encode_buffer_read(2, BUFFER_CAPTURE, 0x0123, 1024);
        assert_eq!(unpack_can_id(read.id), (2, 41, false));
        assert_eq!(read.payload(), &[0, 0x01, 0x23, 0x04, 0x00]);

        let id = pack_can_id(2, CommandId::BufferData, false);
        let sample = CanFrame::data_frame(id, &[0x00, 0x05, 0xFF, 0xF0, 0x00, 0x10, 0x3F, 0xFF]);
        let decoded = decode_frame(&sample).expect("a capture sample");
        assert_eq!(decoded.node, 2);
        assert_eq!(
            decoded.payload,
            Payload::Capture {
                index: 5,
                samples: [-16, 16, 16383]
            }
        );
        // Kind 1, the histogram: a buffer nobody here asks for.
        let other = CanFrame::data_frame(id, &[0x20, 0x05, 0, 0, 0, 0, 0, 0]);
        assert!(decode_frame(&other).is_err());
        let short = CanFrame::data_frame(id, &[0, 0, 0]);
        assert!(decode_frame(&short).is_err(), "a wrong length is refused");
        assert!(decode_frame(&read).is_err(), "cmd 41 is host-to-drive only");
    }

    #[test]
    fn configuration_reads_back_parameter_by_parameter() {
        use crate::ConfigKind;
        // The parameters a frame sets, in its payload's order.
        let params = readback_params(ConfigKind::VelocityGains);
        assert_eq!(params, &[Param::Kpv, Param::Kiv]);
        let request = encode_param_get(3, params[0]);
        assert!(!request.rtr);
        assert_eq!(unpack_can_id(request.id), (3, 37, false));
        assert_eq!(request.payload(), &[9]);

        // The drive answers a float parameter with the bit pattern the gain
        // frame carried, so a landed write reads back equal to what was sent.
        let written = encode_velocity_gains(3, 0.014186124, 0.000835031);
        let w = written.payload();
        let answer = |param: u8, v: &[u8], err| {
            CanFrame::data_frame(
                pack_can_id(3, CommandId::ParamValue, err),
                &[param, v[0], v[1], v[2], v[3]],
            )
        };
        let kpv = decode_frame(&answer(9, &w[0..4], true)).expect("an answer");
        assert!(kpv.err_bit, "the live fault bit rides the answer too");
        let Payload::Param {
            param: 9,
            value: Some(kpv),
        } = kpv.payload
        else {
            panic!("a parameter value");
        };
        let Payload::Param {
            value: Some(kiv), ..
        } = decode_frame(&answer(10, &w[4..8], false)).unwrap().payload
        else {
            panic!("a parameter value");
        };
        let have = |p| match p {
            Param::Kpv => Some(kpv),
            Param::Kiv => Some(kiv),
            _ => None,
        };
        assert_eq!(
            Readback::from_params(ConfigKind::VelocityGains, have),
            Some(Readback::VelocityGains {
                kpv: 0.014186124,
                kiv: 0.000835031
            })
        );
        assert_eq!(
            Readback::from_params(ConfigKind::VelocityGains, |p| have(p)
                .filter(|_| p == Param::Kpv)),
            None,
            "half a frame's parameters is not a read-back"
        );
        // The watchdog's action is the byte cmd 15 carries.
        let watchdog = encode_watchdog(3, 5000, WatchdogAction::Hold);
        assert_eq!(watchdog.payload(), &[0, 0, 0x13, 0x88, 1]);
        assert_eq!(
            Readback::from_params(ConfigKind::Watchdog, |p| Some(match p {
                Param::WatchdogMs => [0, 0, 0x13, 0x88],
                _ => [0, 0, 0, 1],
            })),
            Some(Readback::Watchdog {
                ms: 5000,
                action: 1
            })
        );

        let unknown = CanFrame::data_frame(pack_can_id(3, CommandId::ParamValue, false), &[200]);
        assert_eq!(
            decode_frame(&unknown).unwrap().payload,
            Payload::Param {
                param: 200,
                value: None
            },
            "a parameter the drive does not have answers with its number alone"
        );
        let short = CanFrame::data_frame(pack_can_id(3, CommandId::ParamValue, false), &[9, 0]);
        assert!(
            decode_frame(&short).is_err(),
            "a wrong length is refused, not guessed at"
        );
        assert!(
            decode_frame(&request).is_err(),
            "the request itself is not an answer"
        );
        assert!(
            decode_frame(&written).is_err(),
            "a gain frame is host-to-drive only"
        );
    }

    #[test]
    fn identity_says_what_the_drive_is_and_which_tool_it_carries() {
        let set = encode_param_set(6, Param::ToolId, 12);
        assert_eq!(unpack_can_id(set.id), (6, 36, false));
        assert_eq!(set.payload(), &[1, 0, 0, 0, 12]);
        let ask = encode_poll(6, PollKind::Identity);
        assert!(ask.rtr);
        assert_eq!(unpack_can_id(ask.id), (6, 39, false));
        let answer = CanFrame::data_frame(
            pack_can_id(6, CommandId::Identity, false),
            &[1, 12, 0x00, 0x6D, 3, 50, 1, 0b0000_1001],
        );
        assert_eq!(
            decode_frame(&answer).unwrap().payload,
            Payload::Identity(Identity {
                product: 1,
                tool_id: 12,
                firmware: 109,
                hw_ver: 3,
                pole_pairs: 50,
                calibrated: true,
                flags: 0b0000_1001,
            })
        );
        assert!(decode_frame(&ask).is_err());
    }

    #[test]
    fn range_frames_arm_disarm_and_read_back() {
        let arm = encode_range(4, Some((-123_456, 789_012)));
        assert_eq!(unpack_can_id(arm.id), (4, 46, false));
        // The drive answers a read in the layout the write used.
        let answer = CanFrame::data_frame(pack_can_id(4, CommandId::Range, true), arm.payload());
        let decoded = decode_frame(&answer).expect("an answer");
        assert!(decoded.err_bit);
        assert_eq!(
            decoded.payload,
            Payload::Range(DriveRange::Armed {
                min_ticks: -123_456,
                max_ticks: 789_012
            })
        );
        let disarm = encode_range(4, None);
        assert!(disarm.payload().is_empty() && !disarm.rtr);
        let answer = CanFrame::data_frame(pack_can_id(4, CommandId::Range, false), &[]);
        assert_eq!(
            decode_frame(&answer).unwrap().payload,
            Payload::Range(DriveRange::Disarmed)
        );
        let odd = CanFrame::data_frame(pack_can_id(4, CommandId::Range, false), &[0; 5]);
        assert!(decode_frame(&odd).is_err(), "a wrong length is refused");
        assert!(decode_frame(&encode_poll(4, PollKind::Range)).is_err());

        // A trip reports in error byte 1 bit 3; bit 4 is the firmware's
        // wrong-board flag and no fault of the joint's.
        let errors = CanFrame::data_frame(
            pack_can_id(4, CommandId::StateOfErrors, true),
            &[0b1000_0000, 0b1000_1000],
        );
        let Payload::Errors(flags) = decode_frame(&errors).unwrap().payload else {
            panic!("an error reply");
        };
        assert!(flags.error && flags.range && flags.calibrated && !flags.watchdog);
        assert!(!flags.wrong_product);
        assert_eq!(
            flags.faults().collect::<Vec<_>>(),
            vec![crate::Fault::Range]
        );
        let wrong_board = CanFrame::data_frame(
            pack_can_id(4, CommandId::StateOfErrors, true),
            &[0b1000_0000, 0b0001_0000],
        );
        let Payload::Errors(flags) = decode_frame(&wrong_board).unwrap().payload else {
            panic!("an error reply");
        };
        assert!(flags.wrong_product && !flags.range);
        assert_eq!(
            flags.faults().collect::<Vec<_>>(),
            vec![crate::Fault::WrongProduct]
        );
    }

    #[test]
    fn ripple_frame_carries_slot_harmonic_and_amplitudes_as_the_firmware_reads_them() {
        let f = encode_ripple(4, 2, 4, -300, 1234);
        assert_eq!(unpack_can_id(f.id), (4, 43, false));
        // Header: kind 2 in the top three bits, the slot below; then the
        // harmonic as a u16 and both amplitudes, big-endian.
        assert_eq!(f.payload(), &[0x40, 2, 0, 4, 0xFE, 0xD4, 0x04, 0xD2]);
        assert!(decode_frame(&f).is_err(), "cmd 43 is host-to-drive only");
        // The drive reads a slot back in the same layout on cmd 42.
        let back = CanFrame::data_frame(pack_can_id(4, CommandId::BufferData, false), f.payload());
        assert_eq!(
            decode_frame(&back).unwrap().payload,
            Payload::Harmonic {
                slot: 2,
                harmonic: 4,
                a_ma: -300,
                b_ma: 1234
            }
        );
    }

    /// A tune on the wire is the axis it sounds on, then what firmware 109
    /// reads on cmd 44: a stop, one
    /// queued event a note (operation, voice, pitch, loudness and time, big
    /// endian), then the play.
    #[test]
    fn a_tune_is_a_stop_its_notes_and_a_play() {
        let notes = [Note { hz: 880, ms: 150 }, Note { hz: 0, ms: 50 }];
        let frames: Vec<CanFrame> = encode_tune(3, &notes, 200).collect();
        assert_eq!(frames[0], encode_param_set(3, Param::SoundAxis, 0));
        let frames = &frames[1..];
        let payloads: Vec<&[u8]> = frames.iter().map(|f| f.payload()).collect();
        assert_eq!(
            payloads,
            [
                &[0u8, 0, 0, 0, 0, 0, 0, 0][..],
                &[2, 0, 0x03, 0x70, 0, 200, 0, 150],
                &[2, 0, 0, 0, 0, 200, 0, 50],
                &[3, 0, 0, 0, 0, 0, 0, 0],
            ]
        );
        for f in frames {
            assert_eq!(
                (f.id, f.rtr),
                (pack_can_id(3, CommandId::Sound, false), false)
            );
        }
        assert!(check_tune(&notes).is_ok());
        assert!(check_tune(&[Note { hz: 19, ms: 10 }]).is_err());
        assert!(check_tune(&[Note { hz: 3125, ms: 10 }]).is_err());
    }

    /// The drive's run states read as running, done or failed with the
    /// reason firmware 109 gives for the code.
    #[test]
    fn a_calibration_run_state_reads_as_running_done_or_why_it_failed() {
        assert_eq!(DriveRun::of_encal(0), DriveRun::Idle);
        assert_eq!(DriveRun::of_encal(2), DriveRun::Running);
        assert_eq!(DriveRun::of_encal(3), DriveRun::Done);
        assert!(DriveRun::of_encal(7)
            .failure()
            .is_some_and(|w| w.contains("implausible")));
        assert_eq!(DriveRun::of_cogcal(7), DriveRun::Running);
        assert_eq!(DriveRun::of_cogcal(11), DriveRun::DoneUntrimmed);
        assert!(DriveRun::of_cogcal(4)
            .failure()
            .is_some_and(|w| w.contains("command")));
        assert_eq!(DriveRun::of_cogcal(3).failure(), None);
    }
}
