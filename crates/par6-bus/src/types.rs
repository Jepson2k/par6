//! Driver-model types shared by every bus backend: per-tick commands, the
//! decoded per-node state the RX drain fills, and the health/freshness
//! surface. Wire semantics in `spec/CAN.md`.

/// CAN node id (low 4 bits of the 11-bit arbitration id).
pub type NodeId = u8;

/// Number of addressable nodes (4-bit node field).
pub const MAX_NODES: usize = 16;

/// Conventional gripper node on PAR6.
pub const NODE_GRIPPER: NodeId = 6;
/// Reserved timing-dummy node (pinged when no gripper is fitted).
pub const NODE_TIMING_DUMMY: NodeId = 13;
/// Host node id.
pub const NODE_HOST: NodeId = 14;
/// Bootloader node id.
pub const NODE_BOOTLOADER: NodeId = 15;

/// How a joint motion frame is packed on the wire.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Pack {
    /// Cascade PID motion frame (CAN cmd 2). DLC selects the mode:
    /// pos+vel+cur (8), vel+cur (5), cur only (2).
    #[default]
    Pid,
    /// Impedance PD frame (CAN cmd 4): always DLC 8; the driver computes
    /// torque from position+velocity error with onboard KP/KD, `cur_ma`
    /// acts as feedforward.
    Pd,
    /// HALL homing pack (CAN cmd 31): `vel` is the i24 speed;
    /// `pos`/`cur_ma` are ignored and should be `None`. Replies arrive as
    /// cmd 32 (`NodeState::hall`).
    Hall {
        /// Trigger-value byte sent with the speed (vendor homing uses 2).
        trigger_value: u8,
    },
}

/// Per-joint setpoint for one tick.
///
/// **Channel semantics are load-bearing** (spec/CAN.md): `pos = None` and
/// `vel = None` mean the channel is OMITTED on the wire (the frame DLC
/// shrinks) — the driver switches control mode accordingly. They do NOT
/// mean zero. `cur_ma = None` means "unspecified": the current channel is
/// never omitted from a cmd-2/4 frame, so the CODEC substitutes 0 for
/// `None` at pack time. `Option<i16>` is kept here (rather than a bare
/// `i16`) so the RT mode table's `(pos?, vel?, trq?)` output law maps
/// 1:1 onto this struct and "mode did not command torque" stays
/// distinguishable from "mode commanded exactly 0 mA" in telemetry; the
/// substitute-0 rule lives in the codec, where spec/CAN.md places it.
///
/// Values are truncated toward zero (vendor `int()`), not rounded, when
/// packed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct JointCommand {
    /// Position setpoint \[encoder ticks\]; `None` = channel omitted.
    pub pos: Option<i32>,
    /// Velocity setpoint \[encoder ticks/s\]; `None` = channel omitted.
    pub vel: Option<i32>,
    /// Current setpoint/limit/feedforward \[mA\]; `None` = codec packs 0.
    pub cur_ma: Option<i16>,
    /// Wire packing (control law) for this frame.
    pub pack: Pack,
}

impl JointCommand {
    /// Position-mode PID frame (DLC 8).
    pub fn position(pos: i32, vel: i32, cur_ma: i16) -> Self {
        Self {
            pos: Some(pos),
            vel: Some(vel),
            cur_ma: Some(cur_ma),
            pack: Pack::Pid,
        }
    }

    /// Velocity-mode PID frame (DLC 5).
    pub fn velocity(vel: i32, cur_ma: i16) -> Self {
        Self {
            pos: None,
            vel: Some(vel),
            cur_ma: Some(cur_ma),
            pack: Pack::Pid,
        }
    }

    /// Current-only PID frame (DLC 2).
    pub fn current(cur_ma: i16) -> Self {
        Self {
            pos: None,
            vel: None,
            cur_ma: Some(cur_ma),
            pack: Pack::Pid,
        }
    }

    /// Active idle: velocity 0, current 0 (keeps the driver watchdog fed
    /// and the freshness detector alive without moving).
    pub fn idle() -> Self {
        Self::velocity(0, 0)
    }

    /// Impedance PD frame (cmd 4, DLC 8) with current feedforward.
    pub fn pd(pos: i32, vel: i32, cur_ff_ma: i16) -> Self {
        Self {
            pos: Some(pos),
            vel: Some(vel),
            cur_ma: Some(cur_ff_ma),
            pack: Pack::Pd,
        }
    }

    /// HALL homing drive frame (cmd 31).
    pub fn hall(vel: i32, trigger_value: u8) -> Self {
        Self {
            pos: None,
            vel: Some(vel),
            cur_ma: None,
            pack: Pack::Hall { trigger_value },
        }
    }
}

/// Firmware-mode gripper command payload (CAN cmd 61, DLC 5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FirmwareGripperCommand {
    /// 0 = open … 255 = closed.
    pub position: u8,
    /// Speed byte.
    pub speed: u8,
    /// Current limit \[mA\].
    pub current_ma: i16,
    /// Activate bit (b7; always 1 in vendor practice).
    pub activate: bool,
    /// Action bit (b6; 1 = go to position).
    pub action: bool,
    /// E-stop bit (b5).
    pub estop: bool,
    /// Release-direction bit (b4).
    pub release_dir: bool,
}

/// The gripper frame the RT tick sends — exactly one per tick, always.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GripperCommand {
    /// No gripper fitted: the backend sends an RTR ping to the timing
    /// dummy node instead, keeping the per-tick frame cadence constant.
    NoGripper,
    /// Motor mode: the gripper driver acts as a 7th joint (cmd 2/4/31 to
    /// the gripper node).
    Motor(JointCommand),
    /// Firmware mode command (cmd 61, DLC 5).
    Firmware(FirmwareGripperCommand),
    /// DLC-0 empty poll (cmd 61, DLC 0): feeds the driver watchdog WITHOUT
    /// overwriting the in-progress firmware command. Required every tick
    /// during calibration and firmware homing.
    FirmwarePoll,
    /// Start firmware calibration (cmd 62). Send ONCE, then `FirmwarePoll`
    /// every tick until a new gripper command arrives or timeout.
    Calibrate,
}

/// Telemetry request kinds a poll slot can carry (RTR frames).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PollKind {
    /// cmd 23 → `NodeState::temperature_c`.
    Temperature,
    /// cmd 24 → `NodeState::voltage_mv`.
    Voltage,
    /// cmd 26 → `NodeState::error_flags`.
    Errors,
    /// cmd 25 → `NodeState::device_info`.
    DeviceInfo,
    /// cmd 28 → position/speed refresh.
    Encoder,
    /// cmd 33 → `NodeState::kt_nm_a`.
    Kt,
    /// cmd 10 → liveness ping.
    Ping,
}

/// A single-slot override that preempts the round-robin poll for
/// `repeats` ticks (vendor uses it for config resend and clear-error).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PollAction {
    /// One specific telemetry request.
    Poll {
        /// Target node.
        node: NodeId,
        /// Request kind.
        kind: PollKind,
    },
    /// Clear-error frame (cmd 1) to a node.
    ClearError {
        /// Target node.
        node: NodeId,
    },
    /// Re-send the node's full boot configuration (reconnect path).
    ResendConfig {
        /// Target node.
        node: NodeId,
    },
}

/// Per-type driver fault flags (cmd 26 reply, DLC 2; list index 0 = bit 7).
///
/// Only ~84 ms fresh at 250 Hz / 7 nodes — per RT.md these are trusted
/// only while the node's live fault bit ([`NodeState::live_error_bit`])
/// is set.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ErrorFlags {
    /// Aggregate error bit (byte 0 b7).
    pub error: bool,
    /// Over-temperature (b6).
    pub temperature: bool,
    /// Encoder fault (b5).
    pub encoder: bool,
    /// VBUS fault (b4).
    pub vbus: bool,
    /// Driver fault (b3).
    pub driver: bool,
    /// Velocity fault (b2).
    pub velocity: bool,
    /// Current fault (b1).
    pub current: bool,
    /// Motor-side e-stop (b0).
    pub estop: bool,
    /// Calibrated flag (byte 1 b7).
    pub calibrated: bool,
    /// Activated flag (byte 1 b6).
    pub activated: bool,
    /// Watchdog fired (byte 1 b5).
    pub watchdog: bool,
}

/// Device identity (cmd 25 reply, DLC 7).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DeviceInfo {
    /// Hardware version.
    pub hw_ver: u8,
    /// Production batch.
    pub batch: u8,
    /// Firmware version.
    pub sw_ver: u8,
    /// Serial number.
    pub serial: i32,
}

/// HALL homing reply bits (cmd 32, DLC 4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct HallState {
    /// HALL_trigger bit (b7). Vendor hit condition: trigger == 0 or
    /// `edge` set; position is latched AT trigger.
    pub trigger: bool,
    /// Pin-2 state (b6).
    pub pin2: bool,
    /// Hall index / edge bit (b5).
    pub edge: bool,
}

/// Everything known about one CAN node, updated by [`super::DriverBus::drain_rx`].
///
/// `None` fields have never been reported by the node (or were reset by a
/// freshness re-base). Position/speed/current refresh at frame rate;
/// temp/voltage/error-flags refresh at the round-robin poll rate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NodeState {
    /// Motor position \[encoder ticks\] (cmd 3 / 28 / 32 replies).
    pub position_ticks: Option<i32>,
    /// Motor speed \[encoder ticks/s\].
    pub speed_ticks_s: Option<i32>,
    /// Motor current \[mA\].
    pub current_ma: Option<i16>,
    /// Driver temperature \[°C\] (cmd 23).
    pub temperature_c: Option<i16>,
    /// Bus voltage \[mV\] (cmd 24).
    pub voltage_mv: Option<i16>,
    /// Per-type fault flags (cmd 26) — gate on `live_error_bit` per RT.md.
    pub error_flags: Option<ErrorFlags>,
    /// Torque constant reported by the driver \[Nm/A\] (cmd 33).
    pub kt_nm_a: Option<f32>,
    /// Device identity (cmd 25).
    pub device_info: Option<DeviceInfo>,
    /// HALL reply bits (cmd 32), present only while hall-driven.
    pub hall: Option<HallState>,
    /// Live fault bit: the err bit of the CAN id, set by the driver on
    /// EVERY reply while it has an active fault. Authoritative and
    /// per-frame fresh, unlike `error_flags`.
    pub live_error_bit: bool,
    /// Ticks since this node's last frame; `u64::MAX` = never seen.
    pub data_age_ticks: u64,
}

impl Default for NodeState {
    fn default() -> Self {
        Self {
            position_ticks: None,
            speed_ticks_s: None,
            current_ma: None,
            temperature_c: None,
            voltage_mv: None,
            error_flags: None,
            kt_nm_a: None,
            device_info: None,
            hall: None,
            live_error_bit: false,
            data_age_ticks: u64::MAX,
        }
    }
}

/// Object-detection field of the firmware gripper reply
/// (cmd 60, bits (b5<<1)|b4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ObjectDetection {
    /// 0 — jaws moving.
    #[default]
    Moving = 0,
    /// 1 — object detected while closing.
    DetectedClosing = 1,
    /// 2 — object detected while opening.
    DetectedOpening = 2,
    /// 3 — target reached, no object.
    ReachedNoObject = 3,
}

/// Decoded firmware-mode gripper reply (cmd 60, DLC 4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct GripperReply {
    /// Jaw position, 0 = open … 255 = closed. Known vendor defect: the
    /// SI-converted position reads as a constant in firmware mode —
    /// publish NaN / gate on control mode downstream instead of trusting
    /// a conversion of this byte.
    pub position: u8,
    /// Motor current \[mA\] (payload bytes 1..3).
    pub current_ma: i16,
    /// Activated bit (b7).
    pub activated: bool,
    /// Action-status bit (b6).
    pub action_status: bool,
    /// Object detection code (b5..b4).
    pub object_detection: ObjectDetection,
    /// Temperature error (b3).
    pub temperature_error: bool,
    /// Timeout error (b2).
    pub timeout_error: bool,
    /// E-stop error (b1).
    pub estop_error: bool,
    /// Calibrated (b0).
    pub calibrated: bool,
}

/// Firmware-mode gripper state. Motor-mode gripper telemetry (cmd 3
/// replies) lands in `nodes[gripper_node]` like any joint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GripperState {
    /// Last decoded cmd-60 reply; `None` before the first one.
    pub reply: Option<GripperReply>,
    /// Live fault bit from the gripper's reply CAN ids.
    pub live_error_bit: bool,
    /// Ticks since the last cmd-60 reply; `u64::MAX` = never seen.
    pub data_age_ticks: u64,
}

impl Default for GripperState {
    fn default() -> Self {
        Self {
            reply: None,
            live_error_bit: false,
            data_age_ticks: u64::MAX,
        }
    }
}

/// Everything [`super::DriverBus::drain_rx`] writes: decoded per-node
/// state plus per-drain bookkeeping. Preallocated by the RT loop and
/// reused every tick — filling it never allocates.
#[derive(Debug, Clone, PartialEq)]
pub struct BusState {
    /// Per-node decoded state, indexed by CAN node id.
    pub nodes: [NodeState; MAX_NODES],
    /// Firmware-mode gripper state.
    pub gripper: GripperState,
    /// Frames consumed by the last drain (≤ the per-tick cap).
    pub frames_last_drain: u32,
    /// Max frame age observed in the last drain \[ticks\]. min≈max large
    /// = genuine backlog; only-max large = one slow frame class.
    pub frame_age_max_ticks: u64,
    /// Min frame age observed in the last drain \[ticks\].
    pub frame_age_min_ticks: u64,
    /// Bitmask of nodes whose stale→fresh edge happened during the last
    /// drain — the RT loop re-sends those nodes' config
    /// ([`super::DriverBus::resend_node_config`]).
    pub reconnected_mask: u16,
}

impl Default for BusState {
    fn default() -> Self {
        Self {
            nodes: [NodeState::default(); MAX_NODES],
            gripper: GripperState::default(),
            frames_last_drain: 0,
            frame_age_max_ticks: 0,
            frame_age_min_ticks: 0,
            reconnected_mask: 0,
        }
    }
}

impl BusState {
    /// A fresh, never-seen-anything state.
    pub fn new() -> Self {
        Self::default()
    }
}

/// Data-age classification for one node (spec/CAN.md freshness layer 1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Freshness {
    /// No frame seen since boot / last re-base.
    Unknown,
    /// Data younger than the stale threshold.
    Fresh,
    /// Age ≥ stale threshold — live WARNING, self-clears on the next frame.
    Stale,
    /// Age reached the lost threshold — LATCHED: stays `Lost` even if
    /// frames resume, until [`super::DriverBus::clear_lost_latch`] /
    /// [`super::DriverBus::rebase_freshness`] (user clear-errors or
    /// FLASHING exit).
    Lost,
}

/// Kernel-level CAN link state (netlink, sampled off the RT thread).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LinkState {
    /// State not (yet) known — e.g. loopback/sim backends.
    #[default]
    Unknown,
    /// Link up and error-active.
    Up,
    /// Controller error-passive.
    ErrorPassive,
    /// Bus-off (kernel auto-restart pending).
    BusOff,
}

/// Aggregated link health surface.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LinkHealth {
    /// Last known kernel link state.
    pub state: LinkState,
    /// Interface restarts observed (a decreased kernel counter means the
    /// interface was re-based, not a negative delta).
    pub restarts: u32,
    /// TX errors observed (send failures are also PROPAGATED per call).
    pub tx_errors: u64,
    /// Total frames received.
    pub rx_frames: u64,
}

/// Bus operation failure. Send errors are propagated, never swallowed
/// (vendor swallowed them — documented production bug class).
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum BusError {
    /// A frame could not be transmitted to `node`.
    #[error("TX failed for node {node}")]
    Tx {
        /// Target node of the failed frame.
        node: NodeId,
    },
    /// The interface TX queue is full (it drops silently at the kernel —
    /// backends must detect and report).
    #[error("TX queue full")]
    TxQueueFull,
    /// The link is down / bus-off.
    #[error("bus link down")]
    LinkDown,
    /// The bus has not been boot-configured yet.
    #[error("bus not configured (boot_configure has not run)")]
    NotConfigured,
    /// A command violates the contract (e.g. wrong joint count, TX while
    /// silent).
    #[error("invalid command: {reason}")]
    InvalidCommand {
        /// What was violated.
        reason: &'static str,
    },
}
