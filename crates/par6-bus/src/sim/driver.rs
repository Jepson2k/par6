//! Virtual Spectral/STEPFOC driver: one per CAN node. Consumes real
//! host→driver frames (parsed by DLC exactly like firmware), runs the
//! command semantics from the REAL config gains — cascade position →
//! velocity PI → current with Ilim saturation, PD impedance, HALL drive —
//! and carries the driver-side watchdog, per-type fault flags, the live
//! err bit and the telemetry values the RTR polls report.

use crate::spectral::codec::{unpack_f32, unpack_i16, unpack_i24, unpack_u32, CommandId};
use crate::types::{DeviceInfo, ErrorFlags, NodeId};

/// Assumed firmware velocity-loop period \[s\]. The config `kiv` is a
/// per-loop-iteration gain; the firmware loop runs much faster than the
/// bus tick, so the sim integrates `kiv · err` once per firmware
/// iteration (`dt / FW_LOOP_DT` times per tick). Without this the
/// integral unwinds so slowly that a homing backoff cannot break the
/// endstop seat within the vendor-configured backoff window.
const FW_LOOP_DT: f64 = 0.001;

/// A per-type driver fault a test can inject ([`super::SimBus::inject_fault`]).
/// Maps 1:1 onto the cmd-26 flag bits; every injected fault also raises the
/// aggregate `error` flag and the per-frame live err bit until Clear_Error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
}

/// What the driver's control loop asks of the plant for one step.
#[derive(Debug, Clone, Copy)]
pub(crate) struct PlantCmd {
    /// Loop output current \[mA\], already Ilim-saturated.
    pub current_ma: f64,
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
    Hall { vel: f64 },
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
    /// Firmware velocity-loop iterations per bus tick (≥ 1).
    fw_steps: f64,
    // -- pushed configuration (updated live by config frames) --
    kpp: f64,
    kpv: f64,
    kiv: f64,
    kp_pd: f64,
    kd_pd: f64,
    pub vel_limit: f64,
    pub ilim_ma: f64,
    watchdog_ticks: u64,
    pub kt_nm_a: f32,
    // -- control state --
    mode: Mode,
    integral_ma: f64,
    armed: bool,
    ticks_since_data: u64,
    pub cur_out_ma: f64,
    // -- faults --
    flags: ErrorFlags,
    // -- HALL sensor runtime (band logic evaluated by the bus) --
    pub hall_in_band: bool,
    pub hall_edge_pending: bool,
    pub hall_latched_ticks: Option<i32>,
    // -- telemetry constants --
    pub temperature_c: i16,
    pub voltage_mv: i16,
    pub device: DeviceInfo,
}

impl VirtualDriver {
    pub fn new(dt: f64, node: NodeId, vel_limit: f64, ilim_ma: f64, kt_nm_a: f64) -> Self {
        Self {
            dt,
            fw_steps: (dt / FW_LOOP_DT).round().max(1.0),
            kpp: 0.0,
            kpv: 0.0,
            kiv: 0.0,
            kp_pd: 0.0,
            kd_pd: 0.0,
            vel_limit,
            ilim_ma,
            watchdog_ticks: u64::MAX,
            kt_nm_a: kt_nm_a as f32,
            mode: Mode::Idle,
            integral_ma: 0.0,
            armed: false,
            ticks_since_data: 0,
            cur_out_ma: 0.0,
            flags: ErrorFlags {
                calibrated: true,
                activated: true,
                ..ErrorFlags::default()
            },
            hall_in_band: false,
            hall_edge_pending: false,
            hall_latched_ticks: None,
            temperature_c: 32 + i16::from(node),
            voltage_mv: 24_000,
            device: DeviceInfo {
                hw_ver: 1,
                batch: 1,
                sw_ver: 3,
                serial: 1_000 + i32::from(node),
            },
        }
    }

    /// Handle one host→driver DATA frame. Feeds the watchdog (any valid
    /// data frame counts as command traffic; RTR polls do not), updates
    /// config/mode, and names the reply the bus owes. Wrong-DLC frames are
    /// discarded whole — no state change, no watchdog feed.
    pub fn on_data_frame(&mut self, cmd: CommandId, d: &[u8]) -> ReplyKind {
        use CommandId::*;
        // Firmware sets `watchdog_reset = 1` only in the data-pack cases
        // that install a Controller_mode, and only on a well-formed frame:
        // the wrong-DLC branch sets `Wrong_DL` instead and feeds nothing.
        let fed = matches!(
            (cmd, d.len()),
            (CommandId::DataPack1, 8 | 5 | 2)
                | (CommandId::DataPackPd, 8)
                | (CommandId::DataPackHall, 4)
        );
        let reply = match (cmd, d.len()) {
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
                self.integral_ma = 0.0;
                self.armed = true;
                ReplyKind::Motion
            }
            (DataPackPd, 8) => {
                self.mode = Mode::Pd {
                    pos: f64::from(unpack_i24([d[0], d[1], d[2]])),
                    vel: f64::from(unpack_i24([d[3], d[4], d[5]])),
                    cur_ff: f64::from(unpack_i16([d[6], d[7]])),
                };
                self.integral_ma = 0.0;
                self.armed = true;
                ReplyKind::Motion
            }
            (DataPackHall, 4) => {
                self.mode = Mode::Hall {
                    vel: f64::from(unpack_i24([d[0], d[1], d[2]])),
                };
                self.armed = true;
                ReplyKind::Hall
            }
            (Watchdog, 5) => {
                let ms = unpack_u32([d[0], d[1], d[2], d[3]]);
                self.watchdog_ticks = (f64::from(ms) / 1000.0 / self.dt).round() as u64;
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
            // Current-loop gains and the voltage limit shape the inner
            // current loop / bus voltage, which the plant abstracts away —
            // accepted (they feed the watchdog) but numerically unused.
            (CurrentGains, 8) | (VoltageLimit, 4) | (HeartbeatSetup, 4) | (SaveConfig, 0) => {
                ReplyKind::None
            }
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
                self.integral_ma = 0.0;
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
        reply
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

    /// One control-loop step at the measured plant state. Ages the
    /// watchdog first (a fire drops to Idle and latches the watchdog
    /// flag), then computes the mode's Ilim-saturated current output.
    ///
    /// A latched fault removes drive authority entirely, as it does on the
    /// arm: firmware runs its mode switch only while `Error == 0`, and the
    /// else branch forces `Controller_mode = 0` and drops SLEEP/RESET
    /// until `Clear_Error`.
    pub fn control_step(&mut self, pos_ticks: f64, vel_ticks_s: f64) -> PlantCmd {
        self.age_watchdog();
        // Without this a test could fault a joint, keep commanding it, and
        // pass — against hardware where the arm simply freewheels.
        if self.flags.error {
            self.mode = Mode::Idle;
            self.integral_ma = 0.0;
            self.cur_out_ma = 0.0;
            return PlantCmd {
                current_ma: 0.0,
                vel_limit_ticks_s: self.vel_limit,
                idle: true,
            };
        }
        let ilim = self.ilim_ma;
        let cur = match self.mode {
            Mode::Idle => {
                self.cur_out_ma = 0.0;
                return PlantCmd {
                    current_ma: 0.0,
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
                self.velocity_pi(vt, vel_ticks_s, cur_ff)
            }
            Mode::Velocity { vel, cur_ff } => {
                let vt = vel.clamp(-self.vel_limit, self.vel_limit);
                self.velocity_pi(vt, vel_ticks_s, cur_ff)
            }
            Mode::Hall { vel } => {
                let vt = vel.clamp(-self.vel_limit, self.vel_limit);
                self.velocity_pi(vt, vel_ticks_s, 0.0)
            }
            Mode::Current { cur } => cur,
            Mode::Pd { pos, vel, cur_ff } => {
                self.kp_pd * (pos - pos_ticks) + self.kd_pd * (vel - vel_ticks_s) + cur_ff
            }
        };
        self.cur_out_ma = cur.clamp(-ilim, ilim);
        PlantCmd {
            current_ma: self.cur_out_ma,
            vel_limit_ticks_s: self.vel_limit,
            idle: false,
        }
    }

    /// Discard motion-transient controller state (teleport re-seed).
    ///
    /// The velocity-loop integral is charge accumulated against the
    /// plant's PREVIOUS motion — after a jog-release brake it holds
    /// hundreds of mA. A teleport puts the plant at rest somewhere else;
    /// letting the stale integral discharge there shoves the arm off the
    /// teleported pose (about a thousand ticks after a fast jog) and
    /// rings, violating the teleport contract that the arm lands exactly
    /// where the client asked. The latched motion command goes with it:
    /// it names a target in the PRE-teleport report frame, and the
    /// position loop would drive toward it at full authority until the
    /// next host frame re-latches — the arm visibly lurches off the
    /// landing pose. One tick of Idle instead; config survives.
    pub fn reset_motion_transients(&mut self) {
        self.integral_ma = 0.0;
        self.cur_out_ma = 0.0;
        self.mode = Mode::Idle;
    }

    /// Reset the command-silence counter (a valid data frame arrived).
    /// The firmware gripper uses this for cmd 61/62 frames, whose payloads
    /// the gripper model parses itself.
    pub fn feed_watchdog(&mut self) {
        self.armed = true;
        self.ticks_since_data = 0;
    }

    /// Watchdog aging without a control law — the firmware-mode gripper
    /// path, where the cascade is bypassed but the CAN watchdog still runs.
    pub fn age_watchdog(&mut self) {
        if !self.armed {
            return;
        }
        self.ticks_since_data = self.ticks_since_data.saturating_add(1);
        if self.ticks_since_data == self.watchdog_ticks {
            self.mode = Mode::Idle;
            self.integral_ma = 0.0;
            self.flags.watchdog = true;
            self.flags.error = true;
        }
    }

    /// Whether the watchdog has dropped the driver to Idle (used by the
    /// firmware gripper to halt jaw motion on command silence).
    pub fn watchdog_fired(&self) -> bool {
        self.armed && self.ticks_since_data >= self.watchdog_ticks
    }

    fn velocity_pi(&mut self, vel_target: f64, vel_meas: f64, cur_ff: f64) -> f64 {
        let err = vel_target - vel_meas;
        self.integral_ma =
            (self.integral_ma + self.kiv * err * self.fw_steps).clamp(-self.ilim_ma, self.ilim_ma);
        self.kpv * err + self.integral_ma + cur_ff
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
        }
        self.flags.error = true;
    }

    pub fn clear_faults(&mut self) {
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
    }

    /// Current cmd-26 flag state.
    pub fn flags(&self) -> ErrorFlags {
        self.flags
    }
}
