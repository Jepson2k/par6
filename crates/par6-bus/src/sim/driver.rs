//! Virtual Spectral/STEPFOC driver: one per CAN node. Consumes real
//! host→driver frames (parsed by DLC exactly like firmware), runs the
//! command semantics from the REAL config gains — cascade position →
//! velocity PI → current with Ilim saturation, PD impedance, HALL drive —
//! and carries the driver-side watchdog, per-type fault flags, the live
//! err bit and the telemetry values the RTR polls report.

use crate::spectral::codec::{
    readback_params, unpack_f32, unpack_i16, unpack_i24, unpack_u32, CommandId, Param,
    BUFFER_HARMONICS, CAPTURE_LEN, CAPTURE_TRIGGER_NOW, CAPTURE_VEL_SCALE, COGGING_MODE_HARMONICS,
    RIPPLE_MAX_HARMONIC, RIPPLE_SLOTS, SOUND_PLAY, SOUND_QUEUE, SOUND_STATE_PLAYING, SOUND_STOP,
};
use crate::types::{DeviceInfo, DriveRange, ErrorFlags, Identity, NodeId};

/// Firmware cascade frequency [Hz], from STEPFOC constants.h at 32fb5b5.
/// The physics integrates between loop evaluations; scaling the integral
/// without updating plant feedback adds a phase lag the drive does not have.
const FW_LOOP_HZ: f64 = 6250.0;
/// The raw encoder's counts per motor revolution, and the NEMA 17 steppers' pole
/// pairs (1.8 degree step), which set the electrical phase the ripple
/// feedforward follows.
const ENCODER_COUNTS: f64 = 16384.0;
const POLE_PAIRS: u32 = 50;
pub(crate) const FW_LOOP_DT: f64 = 1.0 / FW_LOOP_HZ;
/// STEPFOC's measured-velocity moving average, sampled each drive
/// iteration. This plant's encoder is exact, so a shorter window looks
/// better here than it is: on the arm's base 8 samples hunted after every
/// move where 20 hold still.
const VELOCITY_WINDOW: usize = 20;
/// The firmware's outward speed past an armed range that trips it
/// \[ticks/s\].
const RANGE_OUTWARD_TICKS_S: f64 = 1000.0;

/// Bridge MOSFET on-resistance \[ohm\] (STEPFOC `Rdson`). Two of them
/// sit in the winding's current path.
const RDSON_OHM: f64 = 0.2;
/// Current-sense resistor \[ohm\] (STEPFOC `SENSE_RESISTOR`).
const SENSE_OHM: f64 = 0.025;
/// Back-EMF constant per unit torque constant.
///
/// The firmware's calibration derives `Kt = 8.2747 / KV` with KV in
/// RPM/V, while the back-EMF constant in V.s/rad is `9.5493 / KV`. The
/// ratio between them is the line-to-line against phase convention the
/// two are quoted in.
const KE_PER_KT: f64 = 9.5493 / 8.2747;
/// Bus voltage \[V\] the bridge switches, and the ceiling a configured
/// voltage limit is taken against (the firmware caps to whichever is
/// smaller).
const VBUS_V: f64 = 24.0;

/// A motor's electrical constants, without which a driver has no
/// current dynamics to model.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Electrical {
    /// Total circuit resistance \[ohm\]: the winding plus the bridge's
    /// own pair of [`RDSON_OHM`] and [`SENSE_OHM`], which is what the
    /// firmware keeps as its TOTAL_RESISTANCE beside the phase value.
    pub r_ohm: f64,
    /// Winding inductance \[H\].
    pub l_h: f64,
    /// Back-EMF constant at the motor shaft \[V.s/rad\].
    pub ke_v_s_rad: f64,
    /// Encoder counts per motor revolution.
    pub ticks_per_rev: f64,
}

impl Electrical {
    /// From a joint's datasheet phase values.
    pub(crate) fn new(phase_r_ohm: f64, phase_l_mh: f64, kt_nm_a: f64, encoder_bits: u8) -> Self {
        Self {
            r_ohm: phase_r_ohm + 2.0 * RDSON_OHM + SENSE_OHM,
            l_h: phase_l_mh * 1e-3,
            ke_v_s_rad: kt_nm_a * KE_PER_KT,
            ticks_per_rev: f64::from(1u32 << encoder_bits),
        }
    }
}

/// A per-type driver fault a test can inject ([`super::SimBus::inject_fault`]).
/// Maps 1:1 onto the cmd-26 flag bits; every injected fault also raises the
/// aggregate `error` flag and the per-frame live err bit until Clear_Error.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FaultKind {
    /// Over-temperature (byte 0 b6).
    Temperature,
    /// Encoder fault (b5).
    Encoder,
    /// VBUS fault (b4).
    Vbus,
    /// Driver fault (b3).
    Driver,
    /// Velocity fault (b2).
    Velocity,
    /// Current fault (b1).
    Current,
    /// Motor-side e-stop (b0).
    Estop,
    /// A watchdog trip still latched: what every drive carries after a
    /// silence past its timeout, the end of the previous run included.
    Watchdog,
}

/// What the driver's control loop asks of the plant for one step.
#[derive(Debug, Clone, Copy)]
pub(crate) struct PlantCmd {
    /// Loop output current \[mA\], already Ilim-saturated.
    pub current_ma: f64,
    /// The additive torque-feedforward share of `current_ma` (the loop
    /// modes' `cur_ff` channel). The jaw model subtracts it: its
    /// current→acceleration gain is a synthetic Ilim mapping, not a
    /// torque model, so current calibrated for the real drives' Kt and
    /// gearing would fabricate acceleration there. The scene integrates
    /// the full current for real.
    pub ff_ma: f64,
    /// Driver velocity limit \[ticks/s\] the plant must respect.
    pub vel_limit_ticks_s: f64,
    /// Driver is in Idle (no drive; shorted-phase-style damping).
    pub idle: bool,
}

/// Latched motion command (the driver's active control mode).
#[derive(Debug, Clone, Copy)]
enum Mode {
    Idle,
    Position { pos: f64, speed: f64, cur_ff: f64 },
    Velocity { vel: f64, cur_ff: f64 },
    Current { cur: f64 },
    Pd { pos: f64, vel: f64, cur_ff: f64 },
    Hall { vel: f64, trigger_value: u8 },
}

impl Mode {
    /// Position and velocity frames run the same cascade and carry its
    /// velocity integral on from each other.
    fn is_cascade(&self) -> bool {
        matches!(self, Mode::Position { .. } | Mode::Velocity { .. })
    }
}

/// What the bus must transmit back for a delivered data frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ReplyKind {
    /// No reply (config frames, clear-error, idle, …).
    None,
    /// cmd 3 motion reply (response to cmd 2 / cmd 4).
    Motion,
    /// cmd 32 HALL reply (response to cmd 31).
    Hall,
}

pub(crate) struct VirtualDriver {
    dt: f64,
    // -- pushed configuration (updated live by config frames) --
    kpp: f64,
    kpv: f64,
    kiv: f64,
    kp_pd: f64,
    kd_pd: f64,
    /// Current-loop PI (cmd 17), in volts per amp of Iq error and volts
    /// per amp per firmware iteration.
    kpiq: f64,
    kiiq: f64,
    /// Configured inverter voltage ceiling \[V\] (cmd 34); 0 = VBUS.
    v_limit_v: f64,
    pub vel_limit: f64,
    pub ilim_ma: f64,
    watchdog_ticks: u64,
    pub kt_nm_a: f32,
    // -- control state --
    mode: Mode,
    integral_ma: f64,
    loop_phase: f64,
    last_drive: PlantCmd,
    velocity_history: [f64; VELOCITY_WINDOW],
    /// The speed filter's length in loops: the firmware's moving average,
    /// or the shortest this model has for its PLL estimates, which do not
    /// lag the way an average does.
    velocity_window: usize,
    velocity_samples: usize,
    previous_encoder: Option<f64>,
    pub measured_velocity: f64,
    /// The motor's electrical model; `None` leaves the driver with the
    /// commanded current applied instantly, which is a drive that can
    /// change its torque infinitely fast.
    electrical: Option<Electrical>,
    /// Current actually flowing \[mA\], and the current loop's integral
    /// \[V\].
    iq_ma: f64,
    iq_err_sum: f64,
    armed: bool,
    ticks_since_data: u64,
    pub cur_out_ma: f64,
    capture_wanted: u16,
    capture_div: u16,
    capture_tick: u16,
    capture_pos: u16,
    capture_vel: Vec<i16>,
    capture_iq: Vec<i16>,
    capture_phase: Vec<i16>,
    /// What the watchdog does at a trip (cmd 15's action byte), how long a
    /// hold may run, and where and for how much longer one is running.
    watchdog_action: u8,
    fallback_ticks: u64,
    hold_pos: Option<f64>,
    hold_ticks_left: u64,
    /// The tune queued and not yet started, and how much of the one
    /// playing is left \[ticks\].
    sound_queued_ticks: u64,
    sound_ticks_left: u64,
    /// The anti-cogging harmonics are switched on (parameter 37).
    ripple_on: bool,
    /// Ripple feedforward slots (the anti-cogging harmonics): (harmonic,
    /// cosine mA, sine mA).
    ripple: [(u8, i16, i16); RIPPLE_SLOTS as usize],
    /// The joint range it enforces (cmd 46).
    range: DriveRange,
    /// The rotor's electrical phase this loop, 0..16383 per cycle, as the
    /// firmware derives it from the raw count.
    phase: u32,
    /// Each configuration frame as last written: what its parameters read
    /// back as, the way the firmware reports the values in force.
    written: [Option<([u8; 8], usize)>; 7],
    // -- faults --
    flags: ErrorFlags,
    // -- HALL sensor runtime (band logic evaluated by the bus) --
    pub hall_in_band: bool,
    pub hall_trigger: bool,
    pub hall_edge: bool,
    hall_needs_initial_sample: bool,
    pub hall_latched_ticks: Option<i32>,
    // -- telemetry constants --
    pub temperature_c: i16,
    pub voltage_mv: i16,
    pub device: DeviceInfo,
    pub identity: Identity,
}

/// The watchdog action byte that holds position at a trip.
const WATCHDOG_HOLD: u8 = 1;
/// The speed estimate the firmware's parameter 50 calls the moving average.
const SPEED_ESTIMATOR_AVERAGE: i32 = 0;
/// The shortest speed filter this model runs, standing in for the PLLs.
const VELOCITY_WINDOW_MIN: usize = 4;

impl VirtualDriver {
    pub fn new(
        dt: f64,
        node: NodeId,
        vel_limit: f64,
        ilim_ma: f64,
        kt_nm_a: f64,
        electrical: Option<Electrical>,
    ) -> Self {
        Self {
            dt,
            kpp: 0.0,
            kpv: 0.0,
            kiv: 0.0,
            kp_pd: 0.0,
            kd_pd: 0.0,
            kpiq: 0.0,
            kiiq: 0.0,
            v_limit_v: 0.0,
            vel_limit,
            ilim_ma,
            watchdog_ticks: u64::MAX,
            kt_nm_a: kt_nm_a as f32,
            mode: Mode::Idle,
            integral_ma: 0.0,
            loop_phase: 0.0,
            last_drive: PlantCmd {
                current_ma: 0.0,
                ff_ma: 0.0,
                vel_limit_ticks_s: vel_limit,
                idle: true,
            },
            velocity_history: [0.0; VELOCITY_WINDOW],
            velocity_window: VELOCITY_WINDOW,
            velocity_samples: 0,
            previous_encoder: None,
            measured_velocity: 0.0,
            electrical,
            iq_ma: 0.0,
            iq_err_sum: 0.0,
            armed: false,
            ticks_since_data: 0,
            cur_out_ma: 0.0,
            capture_wanted: 0,
            capture_div: 1,
            capture_tick: 0,
            capture_pos: 0,
            capture_vel: vec![0; usize::from(CAPTURE_LEN)],
            capture_iq: vec![0; usize::from(CAPTURE_LEN)],
            capture_phase: vec![0; usize::from(CAPTURE_LEN)],
            watchdog_action: 0,
            fallback_ticks: 0,
            sound_queued_ticks: 0,
            sound_ticks_left: 0,
            hold_pos: None,
            hold_ticks_left: 0,
            ripple_on: false,
            ripple: [(0, 0, 0); RIPPLE_SLOTS as usize],
            range: DriveRange::Disarmed,
            phase: 0,
            written: [None; 7],
            flags: ErrorFlags {
                calibrated: true,
                activated: true,
                ..ErrorFlags::default()
            },
            hall_in_band: false,
            hall_trigger: true,
            hall_edge: false,
            hall_needs_initial_sample: true,
            hall_latched_ticks: None,
            temperature_c: 32 + i16::from(node),
            voltage_mv: 24_000,
            device: DeviceInfo {
                hw_ver: 1,
                batch: 1,
                sw_ver: 109,
                serial: 1_000 + i32::from(node),
            },
            identity: Identity {
                product: 1,
                tool_id: 0,
                firmware: 109,
                hw_ver: 1,
                pole_pairs: POLE_PAIRS as u8,
                calibrated: true,
                flags: 0,
            },
        }
    }

    /// Record a full capture, one sample every `divisor` loops, from the
    /// next loop; the firmware floors the divisor at one.
    fn capture_start(&mut self, divisor: u16) {
        self.capture_wanted = CAPTURE_LEN;
        self.capture_div = divisor.max(1);
        self.capture_tick = 0;
        self.capture_pos = 0;
    }

    /// A parameter's value in force (the answer to cmd 37), for the
    /// parameters this model carries; a configuration frame's read back as
    /// last written, nothing if it never was.
    pub fn param_value(&self, param: u8) -> Option<[u8; 4]> {
        if param == Param::ToolId.raw() {
            return Some(i32::from(self.identity.tool_id).to_be_bytes());
        }
        if param == Param::CaptureCount.raw() {
            return Some(i32::from(self.capture_pos).to_be_bytes());
        }
        if param == Param::SoundState.raw() {
            let playing = if self.sound_ticks_left > 0 {
                SOUND_STATE_PLAYING
            } else {
                0
            };
            return Some(playing.to_be_bytes());
        }
        for kind in crate::ConfigKind::ALL {
            let Some(i) = readback_params(kind).iter().position(|p| p.raw() == param) else {
                continue;
            };
            let (bytes, _) = self.written[kind.index()]?;
            return Some(match (kind, i) {
                // The action is the frame's fifth byte, read back as an i32.
                (crate::ConfigKind::Watchdog, 1) => [0, 0, 0, bytes[4]],
                _ => [
                    bytes[4 * i],
                    bytes[4 * i + 1],
                    bytes[4 * i + 2],
                    bytes[4 * i + 3],
                ],
            });
        }
        None
    }

    /// Samples a capture read-out holds.
    pub fn capture_held(&self) -> u16 {
        self.capture_pos
    }

    /// cmd 42 payload for capture sample `index`: the header, then the
    /// velocity, Iq and electrical angle.
    pub fn capture_frame(&self, index: u16) -> [u8; 8] {
        let i = usize::from(index);
        let mut p = [0u8; 8];
        p[0..2].copy_from_slice(&index.to_be_bytes());
        p[2..4].copy_from_slice(&self.capture_vel[i].to_be_bytes());
        p[4..6].copy_from_slice(&self.capture_iq[i].to_be_bytes());
        p[6..8].copy_from_slice(&self.capture_phase[i].to_be_bytes());
        p
    }

    fn set_param(&mut self, param: u8, value: i32) {
        let is = |p: Param| p.raw() == param;
        if is(Param::ToolId) {
            self.identity.tool_id = u8::try_from(value).unwrap_or(0);
        } else if is(Param::WatchdogFallbackMs) {
            self.fallback_ticks =
                (f64::from(value.clamp(0, 60_000)) / 1000.0 / self.dt).round() as u64;
        } else if is(Param::CaptureDivider) {
            self.capture_div = value.clamp(1, 65_535) as u16;
        } else if is(Param::CaptureTrigger) && value == CAPTURE_TRIGGER_NOW {
            self.capture_start(self.capture_div);
        } else if is(Param::CoggingMode) {
            self.ripple_on = value == COGGING_MODE_HARMONICS;
        } else if is(Param::SpeedEstimator) {
            let window = if value == SPEED_ESTIMATOR_AVERAGE {
                VELOCITY_WINDOW
            } else {
                VELOCITY_WINDOW_MIN
            };
            if window != self.velocity_window {
                self.velocity_window = window;
                self.velocity_samples = 0;
            }
        }
    }

    /// Handle one host→driver DATA frame. Feeds the watchdog (any valid
    /// data frame counts as command traffic; RTR polls do not), updates
    /// config/mode, and names the reply the bus owes. Wrong-DLC frames are
    /// discarded whole — no state change, no watchdog feed.
    pub fn on_data_frame(&mut self, cmd: CommandId, d: &[u8]) -> ReplyKind {
        use CommandId::*;
        if let Some(kind) = config_kind(cmd).filter(|k| d.len() == config_dlc(*k)) {
            let mut bytes = [0u8; 8];
            bytes[..d.len()].copy_from_slice(d);
            self.written[kind.index()] = Some((bytes, d.len()));
        }
        // Firmware sets `watchdog_reset = 1` only in the data-pack cases
        // that install a Controller_mode, and only on a well-formed frame:
        // the wrong-DLC branch sets `Wrong_DL` instead and feeds nothing.
        let fed = matches!(
            (cmd, d.len()),
            (CommandId::DataPack1, 8 | 5 | 2)
                | (CommandId::DataPackPd, 8)
                | (CommandId::DataPackHall, 4)
        );
        let was_cascade = self.mode.is_cascade();
        let reply = match (cmd, d.len()) {
            (ParamSet, 5) => {
                self.set_param(d[0], i32::from_be_bytes([d[1], d[2], d[3], d[4]]));
                ReplyKind::None
            }
            (Range, 8) => {
                let min_ticks = i32::from_be_bytes([d[0], d[1], d[2], d[3]]);
                let max_ticks = i32::from_be_bytes([d[4], d[5], d[6], d[7]]);
                if min_ticks < max_ticks {
                    self.range = DriveRange::Armed {
                        min_ticks,
                        max_ticks,
                    };
                }
                ReplyKind::None
            }
            (Range, 0) => {
                self.range = DriveRange::Disarmed;
                ReplyKind::None
            }
            (Sound, 8) => {
                let ms = u16::from_be_bytes([d[6], d[7]]);
                match d[0] {
                    SOUND_STOP => (self.sound_queued_ticks, self.sound_ticks_left) = (0, 0),
                    SOUND_QUEUE => {
                        self.sound_queued_ticks += (f64::from(ms) / 1000.0 / self.dt).round() as u64
                    }
                    SOUND_PLAY => {
                        self.sound_ticks_left = std::mem::take(&mut self.sound_queued_ticks)
                    }
                    _ => {}
                }
                ReplyKind::None
            }
            (BufferWrite, 8) => {
                let header = u16::from_be_bytes([d[0], d[1]]);
                let (kind, slot) = ((header >> 13) as u8, header & 0x1FFF);
                if kind == BUFFER_HARMONICS && slot < u16::from(RIPPLE_SLOTS) {
                    let harmonic = u16::from_be_bytes([d[2], d[3]]);
                    self.ripple[usize::from(slot)] = (
                        // The firmware takes a harmonic past its highest as
                        // an unused slot.
                        u8::try_from(harmonic)
                            .ok()
                            .filter(|h| *h <= RIPPLE_MAX_HARMONIC)
                            .unwrap_or(0),
                        i16::from_be_bytes([d[4], d[5]]),
                        i16::from_be_bytes([d[6], d[7]]),
                    );
                }
                ReplyKind::None
            }
            (DataPack1, 8) => {
                self.mode = Mode::Position {
                    pos: f64::from(unpack_i24([d[0], d[1], d[2]])),
                    speed: f64::from(unpack_i24([d[3], d[4], d[5]])),
                    cur_ff: f64::from(unpack_i16([d[6], d[7]])),
                };
                self.armed = true;
                ReplyKind::Motion
            }
            (DataPack1, 5) => {
                self.mode = Mode::Velocity {
                    vel: f64::from(unpack_i24([d[0], d[1], d[2]])),
                    cur_ff: f64::from(unpack_i16([d[3], d[4]])),
                };
                self.armed = true;
                ReplyKind::Motion
            }
            (DataPack1, 2) => {
                self.mode = Mode::Current {
                    cur: f64::from(unpack_i16([d[0], d[1]])),
                };
                self.armed = true;
                ReplyKind::Motion
            }
            (DataPackPd, 8) => {
                self.mode = Mode::Pd {
                    pos: f64::from(unpack_i24([d[0], d[1], d[2]])),
                    vel: f64::from(unpack_i24([d[3], d[4], d[5]])),
                    cur_ff: f64::from(unpack_i16([d[6], d[7]])),
                };
                self.armed = true;
                ReplyKind::Motion
            }
            (DataPackHall, 4) => {
                if !matches!(self.mode, Mode::Hall { .. }) {
                    self.hall_trigger = true;
                    self.hall_edge = false;
                    self.hall_latched_ticks = None;
                    self.hall_needs_initial_sample = true;
                }
                self.mode = Mode::Hall {
                    vel: f64::from(unpack_i24([d[0], d[1], d[2]])),
                    trigger_value: d[3],
                };
                self.armed = true;
                ReplyKind::Hall
            }
            (Watchdog, 5) => {
                let ms = unpack_u32([d[0], d[1], d[2], d[3]]);
                self.watchdog_ticks = (f64::from(ms) / 1000.0 / self.dt).round() as u64;
                self.watchdog_action = d[4];
                ReplyKind::None
            }
            (Limits, 8) => {
                self.vel_limit = f64::from(unpack_f32([d[0], d[1], d[2], d[3]]));
                self.ilim_ma = f64::from(unpack_f32([d[4], d[5], d[6], d[7]]));
                ReplyKind::None
            }
            (PdGains, 8) => {
                self.kp_pd = f64::from(unpack_f32([d[0], d[1], d[2], d[3]]));
                self.kd_pd = f64::from(unpack_f32([d[4], d[5], d[6], d[7]]));
                ReplyKind::None
            }
            (VelocityGains, 8) => {
                self.kpv = f64::from(unpack_f32([d[0], d[1], d[2], d[3]]));
                self.kiv = f64::from(unpack_f32([d[4], d[5], d[6], d[7]]));
                ReplyKind::None
            }
            (PositionGains, 4) => {
                self.kpp = f64::from(unpack_f32([d[0], d[1], d[2], d[3]]));
                ReplyKind::None
            }
            (CurrentGains, 8) => {
                self.kpiq = f64::from(unpack_f32([d[0], d[1], d[2], d[3]]));
                self.kiiq = f64::from(unpack_f32([d[4], d[5], d[6], d[7]]));
                ReplyKind::None
            }
            (VoltageLimit, 4) => {
                self.v_limit_v = f64::from(unpack_u32([d[0], d[1], d[2], d[3]])) * 1e-3;
                ReplyKind::None
            }
            (HeartbeatSetup, 4) | (SaveConfig, 0) => ReplyKind::None,
            (Kt, 4) => {
                self.kt_nm_a = unpack_f32([d[0], d[1], d[2], d[3]]);
                ReplyKind::None
            }
            (ClearError, 0) => {
                self.clear_faults();
                ReplyKind::None
            }
            (Idle, 0) => {
                self.mode = Mode::Idle;
                ReplyKind::None
            }
            (Estop, 0) => {
                self.mode = Mode::Idle;
                self.flags.estop = true;
                self.flags.error = true;
                ReplyKind::None
            }
            // A wrong-DLC frame on a command that HAS a reply: firmware
            // sets `Wrong_DL = 1` and then answers anyway ("Always respond
            // with this", `Data_pack_1_CAN()` / `Gripper_pack_data()`).
            // State is untouched and the watchdog is not fed, but replies
            // keep flowing and the node stays Fresh — the hardware failure
            // signature is a stream of unchanging replies, not silence.
            (CommandId::DataPack1 | CommandId::DataPackPd, _) => ReplyKind::Motion,
            (CommandId::DataPackHall, _) => ReplyKind::Hall,
            // A non-driver command: nothing to answer.
            _ => return ReplyKind::None,
        };
        // Firmware sets `watchdog_reset = 1` only in the data-pack cases
        // that install a Controller_mode (and on RTR polls, fed by
        // `feed_watchdog`). Idle, Estop, Clear_Error, the gain/limit
        // config writes and the watchdog setup itself do NOT feed it — so
        // a config-only or idle-only traffic pattern keeps the arm's
        // watchdog running even though every frame was accepted. Those
        // arms are exactly the ones that arm the driver, which is what
        // Motion/Hall mark here.
        if fed {
            self.ticks_since_data = 0;
        }
        // The par6 firmware starts the velocity integral afresh when the
        // position/velocity cascade is entered from any other mode: its
        // charge is against the load the loop last drove, and after a
        // current-mode release it is the wind-up of the push before it.
        // The vendor firmware keeps the charge, and slams a joint that a
        // current-mode push left wound up; this plant is the par6 build,
        // which every drive is meant to run.
        if !was_cascade && self.mode.is_cascade() {
            self.integral_ma = 0.0;
        }
        reply
    }

    /// Edge mode latches either transition until the host leaves Hall mode.
    pub fn sample_hall(&mut self, in_band: bool, position_ticks: i32) {
        let changed = self.hall_in_band != in_band;
        self.hall_in_band = in_band;
        let Mode::Hall { trigger_value, .. } = self.mode else {
            return;
        };
        if self.hall_needs_initial_sample {
            self.hall_needs_initial_sample = false;
            if trigger_value == 2 {
                return;
            }
        }
        let hit = if trigger_value == 2 {
            changed
        } else {
            u8::from(in_band) == trigger_value
        };
        if hit && self.hall_trigger {
            self.hall_latched_ticks = Some(position_ticks);
            self.hall_trigger = false;
            self.hall_edge = true;
        } else if !hit && trigger_value != 2 {
            self.hall_trigger = true;
        }
    }

    /// Feed the watchdog for an answered RTR telemetry poll.
    ///
    /// Firmware feeds on every `REMOTE_FRAME` it answers (ping, encoder,
    /// kt, temperature, …), which is what keeps a driver alive through the
    /// RT's homing pattern of idle frames plus encoder polls. Unlike
    /// [`Self::feed_watchdog`] this does NOT arm the driver: a poll is not
    /// a command, and arming an uncommanded driver would start a watchdog
    /// that then fires on a node nobody is driving.
    pub fn feed_watchdog_poll(&mut self) {
        self.ticks_since_data = 0;
    }

    /// A physics step can be shorter than one drive iteration. Carry
    /// its fraction forward so host retiming cannot change the drive's
    /// sampling window or integral gain.
    pub fn loop_step(&mut self, pos_ticks: f64, vel_ticks_s: f64, fw_steps: f64) -> PlantCmd {
        self.loop_phase += fw_steps;
        if self.loop_phase >= 1.0 - 1e-10 {
            self.loop_phase = (self.loop_phase - 1.0).max(0.0);
            self.last_drive = self.control_iteration(pos_ticks, vel_ticks_s);
        }
        self.last_drive
    }

    fn control_iteration(&mut self, pos_ticks: f64, vel_ticks_s: f64) -> PlantCmd {
        // The shaft's true speed, which only the winding sees.
        let shaft_ticks_s = vel_ticks_s;
        let pos_ticks = pos_ticks.round();
        self.phase = ((pos_ticks.rem_euclid(ENCODER_COUNTS) as u32) * POLE_PAIRS) & 16383;
        let vel_ticks_s = self
            .previous_encoder
            .map_or(vel_ticks_s, |previous| (pos_ticks - previous) * FW_LOOP_HZ)
            .trunc();
        self.previous_encoder = Some(pos_ticks);
        let window = self.velocity_window;
        self.velocity_history.rotate_left(1);
        self.velocity_history[VELOCITY_WINDOW - 1] = vel_ticks_s;
        self.velocity_samples = (self.velocity_samples + 1).min(window);
        let vel_ticks_s = (self.velocity_history[VELOCITY_WINDOW - self.velocity_samples..]
            .iter()
            .sum::<f64>()
            / self.velocity_samples as f64)
            .trunc();
        self.measured_velocity = vel_ticks_s;
        if let DriveRange::Armed {
            min_ticks,
            max_ticks,
        } = self.range
        {
            if (pos_ticks > f64::from(max_ticks) && vel_ticks_s > RANGE_OUTWARD_TICKS_S)
                || (pos_ticks < f64::from(min_ticks) && vel_ticks_s < -RANGE_OUTWARD_TICKS_S)
            {
                self.flags.range = true;
                self.flags.error = true;
            }
        }
        let driven = self.cur_out_ma;
        let out = self.control_law(pos_ticks, vel_ticks_s, shaft_ticks_s);
        self.record_capture(vel_ticks_s, driven);
        out
    }

    /// Loop-rate capture, after the loop the way the firmware takes it: this
    /// loop's filtered velocity, the current the last loop drove, and the
    /// electrical phase.
    fn record_capture(&mut self, vel_ticks_s: f64, driven: f64) {
        if self.capture_pos >= self.capture_wanted {
            return;
        }
        self.capture_tick += 1;
        if self.capture_tick >= self.capture_div.max(1) {
            self.capture_tick = 0;
            let i = usize::from(self.capture_pos);
            self.capture_vel[i] = (vel_ticks_s / f64::from(CAPTURE_VEL_SCALE)) as i16;
            self.capture_iq[i] = driven as i16;
            self.capture_phase[i] = self.phase as i16;
            self.capture_pos += 1;
        }
    }

    fn control_law(&mut self, pos_ticks: f64, vel_ticks_s: f64, shaft_ticks_s: f64) -> PlantCmd {
        let fw_steps = 1.0;
        let hold = self
            .hold_pos
            .filter(|_| self.hold_ticks_left > 0 && self.only_watchdog());
        // Without this a test could fault a joint, keep commanding it, and
        // pass — against hardware where the arm simply freewheels.
        if self.flags.error && hold.is_none() {
            self.mode = Mode::Idle;
            self.integral_ma = 0.0;
            self.iq_ma = 0.0;
            self.iq_err_sum = 0.0;
            self.cur_out_ma = 0.0;
            return PlantCmd {
                current_ma: 0.0,
                ff_ma: 0.0,
                vel_limit_ticks_s: self.vel_limit,
                idle: true,
            };
        }
        // A watchdog hold is the position loop on the trip position, with
        // none of the feedforward the host was sending.
        let mode = match hold {
            Some(pos) => Mode::Position {
                pos,
                speed: 0.0,
                cur_ff: 0.0,
            },
            None => self.mode,
        };
        let ilim = self.ilim_ma;
        let ff = match mode {
            Mode::Position { cur_ff, .. }
            | Mode::Velocity { cur_ff, .. }
            | Mode::Pd { cur_ff, .. } => cur_ff,
            _ => 0.0,
        };
        let cur = match mode {
            Mode::Idle => {
                self.iq_ma = 0.0;
                self.iq_err_sum = 0.0;
                self.cur_out_ma = 0.0;
                return PlantCmd {
                    current_ma: 0.0,
                    ff_ma: 0.0,
                    vel_limit_ticks_s: self.vel_limit,
                    idle: true,
                };
            }
            Mode::Position { pos, speed, cur_ff } => {
                // Firmware `Position_mode()`: the frame's speed channel is
                // an ADDITIVE velocity feedforward on the position loop's
                // output, and only the configured velocity limit clamps
                // the combined target. It is not a per-command cap — a
                // hold frame with speed 0 still closes position error at
                // full authority.
                let vt =
                    (self.kpp * (pos - pos_ticks) + speed).clamp(-self.vel_limit, self.vel_limit);
                self.velocity_pi(vt, vel_ticks_s, cur_ff, fw_steps)
            }
            Mode::Velocity { vel, cur_ff } => {
                let vt = vel.clamp(-self.vel_limit, self.vel_limit);
                self.velocity_pi(vt, vel_ticks_s, cur_ff, fw_steps)
            }
            Mode::Hall { vel, .. } => {
                let target = if self.hall_trigger {
                    vel
                } else {
                    self.kpp
                        * (f64::from(self.hall_latched_ticks.unwrap_or(pos_ticks as i32))
                            - pos_ticks)
                };
                let vt = target.clamp(-self.vel_limit, self.vel_limit);
                self.velocity_pi(vt, vel_ticks_s, 0.0, fw_steps)
            }
            Mode::Current { cur } => cur,
            Mode::Pd { pos, vel, cur_ff } => {
                self.kp_pd * (pos - pos_ticks) + self.kd_pd * (vel - vel_ticks_s) + cur_ff
            }
        };
        let setpoint = cur.clamp(-ilim, ilim);
        self.cur_out_ma = self.current_loop(setpoint, shaft_ticks_s, fw_steps);
        PlantCmd {
            current_ma: self.cur_out_ma,
            ff_ma: ff.clamp(-ilim, ilim),
            vel_limit_ticks_s: self.vel_limit,
            idle: false,
        }
    }

    /// Re-aim the driver at a re-seeded pose (teleport).
    ///
    /// The velocity-loop integral is charge accumulated against the
    /// plant's PREVIOUS motion — after a jog-release brake it holds
    /// hundreds of mA. A teleport puts the plant at rest somewhere else;
    /// letting the stale integral discharge there shoves the arm off the
    /// teleported pose (about a thousand ticks after a fast jog) and
    /// rings, violating the teleport contract that the arm lands exactly
    /// where the client asked. The latched motion command goes with it,
    /// replaced by a position hold at the new wire reading `pos_ticks`
    /// already putting out `hold_ma`, the current the landed load takes:
    /// until the runtime's next frame re-commands the joint it stays
    /// held, as a drive that had been holding there would — a limp tick
    /// lets a wrist loaded past its gearbox's holding friction back-drive
    /// a degree before the feedforward arrives, and a cold one sags a
    /// milliradian onto its drivetrain while its current loop catches up.
    pub fn reseed_hold(&mut self, pos_ticks: f64, hold_ma: f64) {
        self.loop_phase = 0.0;
        self.last_drive = PlantCmd {
            current_ma: hold_ma,
            ff_ma: hold_ma,
            vel_limit_ticks_s: self.vel_limit,
            idle: false,
        };
        self.velocity_history.fill(0.0);
        self.velocity_samples = 0;
        self.previous_encoder = None;
        self.measured_velocity = 0.0;
        self.integral_ma = 0.0;
        self.iq_ma = hold_ma;
        self.iq_err_sum = 0.0;
        self.reset_velocity_filter();
        self.cur_out_ma = hold_ma;
        self.mode = Mode::Position {
            pos: pos_ticks,
            speed: 0.0,
            cur_ff: hold_ma,
        };
    }

    /// Reset the command-silence counter (a valid data frame arrived).
    /// The firmware gripper uses this for cmd 61/62 frames, whose payloads
    /// the gripper model parses itself.
    pub fn feed_watchdog(&mut self) {
        self.armed = true;
        self.ticks_since_data = 0;
    }

    /// A tune is playing.
    pub fn sounding(&self) -> bool {
        self.sound_ticks_left > 0
    }

    /// One tick of the tune: the firmware plays only while a loop runs
    /// with nothing wrong, and an idle or faulted drive drops it whole.
    pub fn age_sound(&mut self) {
        if matches!(self.mode, Mode::Idle) || self.flags.error {
            (self.sound_queued_ticks, self.sound_ticks_left) = (0, 0);
        } else {
            self.sound_ticks_left = self.sound_ticks_left.saturating_sub(1);
        }
    }

    /// Watchdog aging without a control law — the firmware-mode gripper
    /// path, where the cascade is bypassed but the CAN watchdog still runs.
    pub fn age_watchdog(&mut self) {
        if !self.armed {
            return;
        }
        self.ticks_since_data = self.ticks_since_data.saturating_add(1);
        if self.ticks_since_data == self.watchdog_ticks {
            // A hold starts only on a drive that was running a loop with
            // nothing else wrong; otherwise the trip is what it always was.
            let running = !matches!(self.mode, Mode::Idle) && !self.flags.error;
            if self.watchdog_action == WATCHDOG_HOLD && running {
                self.hold_pos = self.previous_encoder;
                self.hold_ticks_left = self.fallback_ticks;
            }
            self.mode = Mode::Idle;
            self.flags.watchdog = true;
            self.flags.error = true;
        } else {
            self.hold_ticks_left = self.hold_ticks_left.saturating_sub(1);
        }
    }

    /// The watchdog is the only fault: any other gets the full response,
    /// whatever the watchdog's action says.
    fn only_watchdog(&self) -> bool {
        let f = &self.flags;
        f.watchdog
            && !(f.temperature
                || f.encoder
                || f.vbus
                || f.driver
                || f.velocity
                || f.current
                || f.estop
                || f.range
                || f.wrong_product)
    }

    /// Whether the watchdog has dropped the driver to Idle (used by the
    /// firmware gripper to halt jaw motion on command silence).
    pub fn watchdog_fired(&self) -> bool {
        self.armed && self.ticks_since_data >= self.watchdog_ticks
    }

    fn reset_velocity_filter(&mut self) {
        self.velocity_history.fill(0.0);
        self.velocity_samples = 0;
        self.previous_encoder = None;
    }

    /// The firmware's `Torque_mode()` current loop and the winding it
    /// drives: PI on the Iq error into a voltage, capped by the
    /// configured limit, integrated through `L di/dt = U - R.i - Ke.w`.
    ///
    /// Without it the commanded current appears in the winding instantly,
    /// which is a drive with unlimited voltage — the one assumption that
    /// makes a position loop stiffer in the simulator than it can be on
    /// the bench. Slewing this motor's full current through its winding
    /// in one firmware iteration would want tens of volts; the bridge has
    /// six.
    fn current_loop(&mut self, iq_set_ma: f64, vel_ticks_s: f64, fw_steps: f64) -> f64 {
        let Some(e) = self.electrical else {
            return iq_set_ma;
        };
        let ceiling = if self.v_limit_v > 0.0 {
            self.v_limit_v.min(VBUS_V)
        } else {
            VBUS_V
        };
        // The back-EMF the winding sees, from the shaft speed this step.
        let omega = vel_ticks_s / e.ticks_per_rev * std::f64::consts::TAU;
        let bemf = e.ke_v_s_rad * omega;
        // The loop runs at the firmware's own iteration rate, and the
        // substep this call stands for is `fw_steps` of them.
        let n = fw_steps.round().max(1.0);
        let dt_e = fw_steps * FW_LOOP_DT / n;
        for _ in 0..n as u32 {
            let err_a = (iq_set_ma - self.iq_ma) * 1e-3;
            self.iq_err_sum = (self.iq_err_sum + self.kiiq * err_a).clamp(-ceiling, ceiling);
            let uq = (self.kpiq * err_a + self.iq_err_sum).clamp(-ceiling, ceiling);
            let di_a_s = (uq - e.r_ohm * self.iq_ma * 1e-3 - bemf) / e.l_h;
            self.iq_ma += di_a_s * dt_e * 1e3;
        }
        self.iq_ma
    }

    fn velocity_pi(&mut self, vel_target: f64, vel_meas: f64, cur_ff: f64, fw_steps: f64) -> f64 {
        let err = vel_target - vel_meas;
        self.integral_ma =
            (self.integral_ma + self.kiv * err * fw_steps).clamp(-self.ilim_ma, self.ilim_ma);
        self.kpv * err + self.integral_ma + cur_ff + self.ripple_ff()
    }

    /// The firmware's ripple feedforward at this loop's phase \[mA\].
    fn ripple_ff(&self) -> f64 {
        if !self.ripple_on {
            return 0.0;
        }
        let phase = std::f64::consts::TAU * f64::from(self.phase) / 16384.0;
        self.ripple
            .iter()
            .filter(|(h, _, _)| *h != 0)
            .map(|(h, a, b)| {
                let x = f64::from(*h) * phase;
                f64::from(*a) * x.cos() + f64::from(*b) * x.sin()
            })
            .sum()
    }

    pub fn set_fault(&mut self, kind: FaultKind) {
        match kind {
            FaultKind::Temperature => self.flags.temperature = true,
            FaultKind::Encoder => self.flags.encoder = true,
            FaultKind::Vbus => self.flags.vbus = true,
            FaultKind::Driver => self.flags.driver = true,
            FaultKind::Velocity => self.flags.velocity = true,
            FaultKind::Current => self.flags.current = true,
            FaultKind::Estop => self.flags.estop = true,
            FaultKind::Watchdog => {
                self.mode = Mode::Idle;
                self.flags.watchdog = true;
            }
        }
        self.flags.error = true;
    }

    pub fn clear_faults(&mut self) {
        self.mode = Mode::Idle;
        self.hold_pos = None;
        self.hold_ticks_left = 0;
        let (calibrated, activated) = (self.flags.calibrated, self.flags.activated);
        self.flags = ErrorFlags {
            calibrated,
            activated,
            ..ErrorFlags::default()
        };
    }

    /// The per-frame live fault bit: set on EVERY reply while any fault
    /// flag is active.
    pub fn err_bit(&self) -> bool {
        let f = &self.flags;
        f.error
            || f.temperature
            || f.encoder
            || f.vbus
            || f.driver
            || f.velocity
            || f.current
            || f.estop
            || f.watchdog
            || f.range
    }

    /// The joint range it enforces (cmd 46).
    pub fn range(&self) -> DriveRange {
        self.range
    }

    /// Drop the range, as a restart does.
    pub fn disarm_range(&mut self) {
        self.range = DriveRange::Disarmed;
    }

    /// Current cmd-26 flag state.
    pub fn flags(&self) -> ErrorFlags {
        self.flags
    }
}

/// The configuration frame a command writes, if it writes one.
pub(crate) fn config_kind(cmd: CommandId) -> Option<crate::ConfigKind> {
    use crate::ConfigKind as K;
    Some(match cmd {
        CommandId::Watchdog => K::Watchdog,
        CommandId::Limits => K::Limits,
        CommandId::VoltageLimit => K::VoltageLimit,
        CommandId::PdGains => K::PdGains,
        CommandId::CurrentGains => K::CurrentGains,
        CommandId::VelocityGains => K::VelocityGains,
        CommandId::PositionGains => K::PositionGains,
        _ => return None,
    })
}

fn config_dlc(kind: crate::ConfigKind) -> usize {
    use crate::ConfigKind as K;
    match kind {
        K::Watchdog => 5,
        K::VoltageLimit | K::PositionGains => 4,
        K::Limits | K::PdGains | K::CurrentGains | K::VelocityGains => 8,
    }
}
