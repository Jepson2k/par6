//! Driver-model types shared by every bus backend: per-tick commands, the
//! decoded per-node state the RX drain fills, and the health/freshness
//! surface. Wire semantics live in [`crate::spectral`].

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
    /// Firmware Idle (CAN cmd 12, DLC 0): the driver drops its control
    /// loop entirely — no drive, phases limp — until the next motion
    /// frame re-arms it. All channels must be `None`; the driver never
    /// replies to cmd 12, so a sender that needs feedback must follow
    /// with [`Pack::EncoderPoll`].
    Idle,
    /// Encoder telemetry request (CAN cmd 28, RTR): keeps position/speed
    /// feedback and the freshness detector alive on a joint that has been
    /// dropped to firmware Idle without re-arming its control loop. All
    /// channels must be `None`.
    EncoderPoll,
}

/// Per-joint setpoint for one tick.
///
/// **Channel semantics are load-bearing**: `pos = None` and
/// `vel = None` mean the channel is OMITTED on the wire (the frame DLC
/// shrinks) — the driver switches control mode accordingly. They do NOT
/// mean zero. `cur_ma = None` means "unspecified": the current channel is
/// never omitted from a cmd-2/4 frame, so the CODEC substitutes 0 for
/// `None` at pack time. `Option<i16>` is kept here (rather than a bare
/// `i16`) so the RT mode table's `(pos?, vel?, trq?)` output law maps
/// 1:1 onto this struct and "mode did not command torque" stays
/// distinguishable from "mode commanded exactly 0 mA" in telemetry; the
/// substitute-0 rule lives in the codec, at the wire boundary.
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
    /// and the freshness detector alive without moving). The driver's
    /// control loop stays armed — for a genuinely limp joint use
    /// [`drop_to_idle`](Self::drop_to_idle).
    pub fn idle() -> Self {
        Self::velocity(0, 0)
    }

    /// Firmware Idle frame (cmd 12, DLC 0): the driver goes limp. It
    /// sends no reply, so follow with [`encoder_poll`](Self::encoder_poll)
    /// to keep feedback and freshness alive for the idled joint.
    pub fn drop_to_idle() -> Self {
        Self {
            pos: None,
            vel: None,
            cur_ma: None,
            pack: Pack::Idle,
        }
    }

    /// Encoder telemetry request (cmd 28, RTR): position/speed feedback
    /// without re-arming an idled driver.
    pub fn encoder_poll() -> Self {
        Self {
            pos: None,
            vel: None,
            cur_ma: None,
            pack: Pack::EncoderPoll,
        }
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
    /// cmd 40 → temperature, voltage, error flags and current in one
    /// reply (firmware 109). An older firmware does not answer it, and
    /// gets the three polls below instead.
    Telemetry,
    /// cmd 23 → `NodeState::temperature_c`.
    Temperature,
    /// cmd 24 → `NodeState::voltage_mv`.
    Voltage,
    /// cmd 26 → `NodeState::error_flags`.
    Errors,
    /// cmd 25 → `NodeState::device_info`.
    DeviceInfo,
    /// cmd 39 → `NodeState::identity` (firmware 109).
    Identity,
    /// cmd 28 → position/speed refresh.
    Encoder,
    /// cmd 33 → `NodeState::kt_nm_a`.
    Kt,
    /// cmd 10 → liveness ping.
    Ping,
    /// cmd 46 → `NodeState::range` (par6 firmware).
    Range,
}

/// The joint range a drive enforces (cmd 46, par6 firmware): past either
/// end while still moving out, the drive trips and brakes its motor. Its
/// own multi-turn count, so it holds only until the drive restarts, which
/// drops it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveRange {
    /// No range: nothing stops the motor wherever it goes.
    Disarmed,
    /// Armed over these drive positions \[encoder ticks\].
    Armed {
        /// Lowest position allowed.
        min_ticks: i32,
        /// Highest position allowed.
        max_ticks: i32,
    },
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
    /// One stored configuration field, consuming exactly one poll slot.
    ConfigFrame {
        /// Target node.
        node: NodeId,
        /// Field to transmit.
        kind: crate::ConfigKind,
    },
    /// A read of the parameters one configuration frame sets (cmd 37 each),
    /// consuming one poll slot; the answers land in
    /// [`NodeState::readback`].
    ConfigRead {
        /// Target node.
        node: NodeId,
        /// Configuration frame to read.
        kind: crate::ConfigKind,
    },
    /// Ask for one parameter (cmd 37), one poll slot; the answer lands in
    /// [`NodeState::param`].
    ParamRead {
        /// Target node.
        node: NodeId,
        /// The parameter.
        param: crate::spectral::codec::Param,
    },
    /// Ask how many samples the capture holds (cmd 37), one poll slot; the
    /// answer lands in the backend's [`CaptureBuffer`] for the node.
    CaptureStatus {
        /// Target node.
        node: NodeId,
    },
    /// Have the drive send `count` capture samples from `first` (cmd 41),
    /// one poll slot; they land in the node's [`CaptureBuffer`] as the drive
    /// paces them out.
    CaptureRead {
        /// Target node.
        node: NodeId,
        /// First sample.
        first: u16,
        /// How many.
        count: u16,
    },
}

/// A loop-rate capture as read back so far: the velocity the drive's loop
/// acted on, its measured Iq and the electrical angle, one row per sample.
/// The backend keeps one per node, sized once at boot, so a read on the tick
/// path stores without allocating.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CaptureBuffer {
    /// Samples the drive says it holds (status answer); a capture is
    /// complete at `CAPTURE_LEN`.
    pub recorded: u16,
    /// Control loops per sample the capture was started with.
    pub divisor: u16,
    /// Channel A \[ticks/s / `CAPTURE_VEL_SCALE`\], indexed by sample.
    pub velocity: Vec<i16>,
    /// Channel B \[mA\], indexed by sample.
    pub current: Vec<i16>,
    /// Channel C: the electrical angle, 0..16383 per cycle.
    pub phase: Vec<i16>,
    /// Which samples have arrived.
    received: Vec<bool>,
}

impl Default for CaptureBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl CaptureBuffer {
    /// Room for a full capture, zeroed.
    pub fn new() -> Self {
        let len = usize::from(crate::spectral::codec::CAPTURE_LEN);
        Self {
            recorded: 0,
            divisor: 0,
            velocity: vec![0; len],
            current: vec![0; len],
            phase: vec![0; len],
            received: vec![false; len],
        }
    }

    /// Forget the previous capture: a new one is starting, one sample every
    /// `divisor` loops.
    pub fn start(&mut self, divisor: u16) {
        self.recorded = 0;
        self.divisor = divisor;
        self.velocity.fill(0);
        self.current.fill(0);
        self.phase.fill(0);
        self.received.fill(false);
    }

    /// One sample of every channel (a cmd 42 capture frame).
    pub fn store(&mut self, index: u16, samples: [i16; 3]) {
        let at = usize::from(index);
        if at >= self.received.len() {
            return;
        }
        self.velocity[at] = samples[0];
        self.current[at] = samples[1];
        self.phase[at] = samples[2];
        self.received[at] = true;
    }

    /// The span of the samples the drive holds that have not all arrived:
    /// where the first missing one is and how many to its last. A frame
    /// lost on the bus is asked for again inside it.
    pub fn missing(&self) -> Option<(u16, u16)> {
        let held = &self.received[..usize::from(self.recorded).min(self.received.len())];
        let first = held.iter().position(|seen| !seen)?;
        let last = held.iter().rposition(|seen| !seen)?;
        Some((first as u16, (last - first + 1) as u16))
    }

    /// The drive's answer to how many samples it holds.
    pub fn status(&mut self, recorded: u16) {
        self.recorded = recorded;
    }
}

/// Per-type driver fault flags (cmd 26 reply, DLC 2; list index 0 = bit 7).
///
/// Only ~84 ms fresh at 250 Hz / 7 nodes — these are trusted
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
    /// The firmware is on the other product's board and drives nothing
    /// (byte 1 b4, firmware 109).
    pub wrong_product: bool,
    /// The joint left its armed range (byte 1 b3, par6 firmware).
    pub range: bool,
}

/// One drive fault bit, named the way the whole stack reports it.
///
/// The names are the [`ErrorFlags`] register's own, in the spelling the
/// RT latch keys use, so a fault an operator sees on a joint error and
/// the same fault in a drive-health label cannot be two different words.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    /// Driver over-temperature.
    Temperature,
    /// Encoder fault.
    Encoder,
    /// VBUS fault.
    Vbus,
    /// Driver fault.
    Driver,
    /// Velocity fault.
    Velocity,
    /// Current fault.
    Current,
    /// Motor-side e-stop.
    EstopMotor,
    /// Driver watchdog fired.
    Watchdog,
    /// The joint left the range its drive enforces.
    Range,
    /// The drive runs the other product's firmware and drives nothing.
    WrongProduct,
}

impl Fault {
    /// The label this fault is reported under.
    pub const fn name(self) -> &'static str {
        match self {
            Self::Temperature => "temperature",
            Self::Encoder => "encoder",
            Self::Vbus => "vbus",
            Self::Driver => "driver",
            Self::Velocity => "velocity",
            Self::Current => "current",
            Self::EstopMotor => "estop_motor",
            Self::Watchdog => "watchdog",
            Self::Range => "range",
            Self::WrongProduct => "wrong_product",
        }
    }
}

impl ErrorFlags {
    /// The fault bits that are set, most significant first.
    ///
    /// The one table both consumers read: the RT core latches an error
    /// per entry and the server labels one per entry, so a new bit cannot
    /// be threaded through only one of them, and the aggregate `error`
    /// bit stays out of it — it says only that one of these is set.
    ///
    /// Allocation-free: the pairs live in a fixed array the iterator owns,
    /// so the RT tick path may call it.
    pub fn faults(self) -> impl Iterator<Item = Fault> {
        [
            (self.temperature, Fault::Temperature),
            (self.encoder, Fault::Encoder),
            (self.vbus, Fault::Vbus),
            (self.driver, Fault::Driver),
            (self.velocity, Fault::Velocity),
            (self.current, Fault::Current),
            (self.estop, Fault::EstopMotor),
            (self.watchdog, Fault::Watchdog),
            (self.range, Fault::Range),
            (self.wrong_product, Fault::WrongProduct),
        ]
        .into_iter()
        .filter_map(|(set, fault)| set.then_some(fault))
    }
}

/// Device information (cmd 25 reply, DLC 7).
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

/// What a drive is (cmd 39 reply, firmware 109).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Identity {
    /// 1 STEPFOC, 2 Spectral.
    pub product: u8,
    /// Which tool the drive is built into, as the drive's EEPROM says
    /// (`ToolConfig::can_tool_id`); 0 = not set.
    pub tool_id: u8,
    /// Firmware version, e.g. 109.
    pub firmware: u16,
    /// Hardware version.
    pub hw_ver: u8,
    /// Motor pole pairs.
    pub pole_pairs: u8,
    /// The drive holds a motor calibration.
    pub calibrated: bool,
    /// The firmware's own flag byte (bit 0 gripper, bit 1 brake on idle).
    pub flags: u8,
}

/// HALL homing reply bits (cmd 32, DLC 4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct HallState {
    /// HALL_trigger bit (b7): true while searching, false while holding the hit.
    pub trigger: bool,
    /// Pin-2 state (b6).
    pub pin2: bool,
    /// Hall index / edge bit (b5), retained until a new Hall-mode entry.
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
    /// Changes on every received position sample, including unchanged positions.
    /// Other telemetry does not advance this wrapping counter.
    pub position_generation: u64,
    /// Motor speed \[encoder ticks/s\].
    pub speed_ticks_s: Option<i32>,
    /// Motor current \[mA\].
    pub current_ma: Option<i16>,
    /// Driver temperature \[°C\] (cmd 23).
    pub temperature_c: Option<i16>,
    /// Bus voltage \[mV\] (cmd 24).
    pub voltage_mv: Option<i16>,
    /// Per-type fault flags (cmd 26) — gate on `live_error_bit`.
    pub error_flags: Option<ErrorFlags>,
    /// Torque constant reported by the driver \[Nm/A\] (cmd 33).
    pub kt_nm_a: Option<f32>,
    /// Device information (cmd 25).
    pub device_info: Option<DeviceInfo>,
    /// What the drive says it is (cmd 39); `None` until it answered, and
    /// for good on a firmware before 109.
    pub identity: Option<Identity>,
    /// The node has answered the combined telemetry poll (cmd 40): one
    /// poll refreshes what a firmware before 109 needs three for.
    pub combined_telemetry: bool,
    /// HALL reply bits (cmd 32), present only while hall-driven.
    pub hall: Option<HallState>,
    /// Live fault bit: the err bit of the CAN id, set by the driver on
    /// EVERY reply while it has an active fault. Authoritative and
    /// per-frame fresh, unlike `error_flags`.
    pub live_error_bit: bool,
    /// The value the drive last answered for each parameter, by its number,
    /// as wire bytes; `None` until answered, or once forgotten.
    pub params: [Option<[u8; 4]>; PARAM_SLOTS],
    /// The range the drive says it enforces (cmd 46); `None` until it
    /// answered.
    pub range: Option<DriveRange>,
    /// Ticks since this node's last frame; `u64::MAX` = never seen.
    pub data_age_ticks: u64,
}

impl Default for NodeState {
    fn default() -> Self {
        Self {
            position_ticks: None,
            position_generation: 0,
            speed_ticks_s: None,
            current_ma: None,
            temperature_c: None,
            voltage_mv: None,
            error_flags: None,
            kt_nm_a: None,
            device_info: None,
            identity: None,
            combined_telemetry: false,
            hall: None,
            live_error_bit: false,
            data_age_ticks: u64::MAX,
            params: [None; PARAM_SLOTS],
            range: None,
        }
    }
}

/// Parameter numbers a node's state keeps answers for: firmware 109 ends
/// at 87.
pub const PARAM_SLOTS: usize = 96;

impl NodeState {
    /// What the drive runs for `kind`, from its answers to the last read of
    /// that frame's parameters; `None` until all of them answered.
    pub fn readback(&self, kind: crate::ConfigKind) -> Option<crate::spectral::codec::Readback> {
        crate::spectral::codec::Readback::from_params(kind, |p| self.params[usize::from(p.raw())])
    }

    /// The integer the drive last answered for `param`; `None` until it
    /// has, or since [`Self::forget_param`].
    pub fn param(&self, param: crate::spectral::codec::Param) -> Option<i32> {
        self.params[usize::from(param.raw())].map(i32::from_be_bytes)
    }

    /// Drop the answer held for `param`, so the next one read is known to
    /// have come after this.
    pub fn forget_param(&mut self, param: crate::spectral::codec::Param) {
        self.params[usize::from(param.raw())] = None;
    }

    /// A parameter answer (cmd 38).
    pub(crate) fn record_param(&mut self, param: u8, value: Option<[u8; 4]>) {
        if let Some(slot) = self.params.get_mut(usize::from(param)) {
            *slot = value;
        }
    }

    pub(crate) fn record_position(&mut self, position_ticks: i32) {
        self.position_ticks = Some(position_ticks);
        self.position_generation = self.position_generation.wrapping_add(1);
    }
}

/// Object-detection field of the firmware gripper reply (cmd 60). The
/// firmware packs this value's low bit into byte bit 5 and its high bit
/// into bit 4, so the value is `(b4 << 1) | b5`.
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
    /// Nodes whose reconnect edge landed in the last drain; the RT loop
    /// re-sends their config.
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

/// Data-age classification for one node.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Freshness {
    /// No frame seen since boot.
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
    /// Interface restarts since the daemon started (a decreased kernel
    /// counter means the interface was re-based, not a negative delta).
    pub restarts: u32,
    /// Bus-off events since the daemon started, counted from the device
    /// counters: one the kernel auto-restart already recovered from
    /// still lost frames and still shows up here, while `state` reports
    /// the controller as it is now.
    pub bus_off_events: u32,
    /// TX errors observed (send failures are also PROPAGATED per call).
    pub tx_errors: u64,
    /// Total frames received.
    pub rx_frames: u64,
}

/// A live drive retune: the values `SET_PID_GAINS` replaces in one
/// node's stored configuration before the push. The watchdog settings
/// are deliberately absent — retuning must never touch the safety
/// timeout.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DriveTune {
    /// Cascade-PID and impedance-PD gains (cmds 16/22/23/24).
    pub gains: par6_config::Gains,
    /// Driver current limit \[mA\] (cmd 20).
    pub ilim_ma: f64,
    /// Motor velocity limit \[encoder ticks/s\] (cmd 20).
    pub velocity_limit_ticks_s: f64,
    /// Driver voltage limit \[mV\] (cmd 34); 0 = use VBUS.
    pub voltage_limit_mv: u32,
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
