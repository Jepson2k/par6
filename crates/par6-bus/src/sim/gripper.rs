//! Simulated CAN gripper: one physical jaw pair driven in one of two
//! exclusive modes. Motor mode runs the node's virtual driver + 1-DOF
//! plant like a 7th joint; firmware mode (cmd 61/62) runs the onboard
//! controller model — byte-position moves, the DLC-0 empty poll that
//! feeds the watchdog without overwriting the command, the cmd-62
//! calibration sequence and object-detection codes from jaw travel vs
//! the commanded position.

use par6_config::GripperConfig;

use crate::spectral::codec::CommandId;
use crate::types::{FirmwareGripperCommand, NodeId, ObjectDetection};

use super::driver::{ReplyKind, VirtualDriver};
use super::plant::KinJoint;

/// Firmware jaw speed \[position bytes per second per speed-byte unit\]
/// (the MuJoCo plant's jaw approach uses the same rate).
pub(crate) const BYTES_PER_S_PER_SPEED_UNIT: f64 = 4.0;
/// Firmware calibration sweep duration \[s\] (vendor waits ≥2 s, times
/// out at 10 s).
const CALIBRATION_S: f64 = 1.5;
/// Reported motor current while the jaws move freely \[mA\].
const MOVING_CUR_MA: f64 = 100.0;

/// Which drive mode owns the jaw (the two are exclusive on real hardware).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Ctrl {
    Motor,
    Firmware,
}

pub(crate) struct GripperSim {
    pub driver: VirtualDriver,
    /// Jaw plant in motor-tick space: 0 = fully closed, `stroke_ticks` =
    /// fully open (`ticks_per_meter = 2^14 / (4π · gear_r)`).
    pub joint: KinJoint,
    stroke_ticks: f64,
    ctrl: Ctrl,
    // -- firmware-mode state --
    /// Jaw position byte, 0 = open … 255 = closed.
    pos_byte: f64,
    cmd: FirmwareGripperCommand,
    calibrated: bool,
    calibration_ticks_left: u64,
    calibration_total_ticks: u64,
    detection: ObjectDetection,
    moving: bool,
    pressing: bool,
    estop_latched: bool,
    /// Obstruction while closing: the position byte the jaws jam at.
    pub object_close_at: Option<u8>,
    /// Obstruction while opening: the position byte the jaws jam at.
    pub object_open_at: Option<u8>,
    pub load_ma: f64,
}

impl GripperSim {
    pub fn new(dt: f64, node: NodeId, cfg: &GripperConfig) -> Self {
        let d = cfg
            .driver
            .as_ref()
            .expect("GripperSim requires a CAN gripper ([driver] table)");
        let ticks_per_meter = 16384.0 / (4.0 * std::f64::consts::PI * d.gear_r_m);
        let stroke_ticks = d.stroke_mm / 1000.0 * ticks_per_meter;
        // The jaw accel ceiling has no config field; full boot Ilim
        // reaching the commanded 80000-ticks/s class speeds in ~50 ms is
        // consistent with the arm joints.
        let accel_max = d.velocity_limit_ticks_s * 20.0;
        let cal_ticks = (CALIBRATION_S / dt).round() as u64;
        Self {
            driver: VirtualDriver::new(dt, node, d.velocity_limit_ticks_s, d.ilim_ma, d.kt_nm_a),
            joint: KinJoint::new(
                dt,
                stroke_ticks / 2.0,
                0.0,
                stroke_ticks,
                accel_max,
                d.ilim_ma,
            ),
            stroke_ticks,
            ctrl: Ctrl::Motor,
            pos_byte: 127.5,
            cmd: FirmwareGripperCommand::default(),
            calibrated: false,
            calibration_ticks_left: 0,
            calibration_total_ticks: cal_ticks,
            detection: ObjectDetection::Moving,
            moving: false,
            pressing: false,
            estop_latched: false,
            object_close_at: None,
            object_open_at: None,
            load_ma: 0.0,
        }
    }

    /// A motor-mode frame (cmd 2/4/31) arrived — the driver handles it and
    /// motor mode takes the jaw over.
    pub fn on_motor_frame(&mut self, cmd: CommandId, data: &[u8]) -> ReplyKind {
        let reply = self.driver.on_data_frame(cmd, data);
        if reply != ReplyKind::None {
            self.ctrl = Ctrl::Motor;
        }
        reply
    }

    /// Firmware command (cmd 61, DLC 5): overwrites the in-progress
    /// command (aborting a running calibration) and feeds the watchdog.
    /// `moving`/`pressing` are left to the next `step` — homing replays
    /// the same gripper_move every tick, and the status bits must keep
    /// reporting the motion in progress across replays.
    pub fn on_firmware_command(&mut self, cmd: FirmwareGripperCommand) {
        self.ctrl = Ctrl::Firmware;
        self.calibration_ticks_left = 0;
        self.cmd = cmd;
        self.estop_latched = cmd.estop;
        self.driver.feed_watchdog();
    }

    /// DLC-0 empty poll: feeds the watchdog WITHOUT overwriting the
    /// in-progress firmware command or calibration.
    pub fn on_empty_poll(&mut self) {
        self.ctrl = Ctrl::Firmware;
        self.driver.feed_watchdog();
    }

    /// cmd 62: start the calibration sequence.
    pub fn on_calibrate(&mut self) {
        self.ctrl = Ctrl::Firmware;
        self.calibrated = false;
        self.calibration_ticks_left = self.calibration_total_ticks;
        self.cmd = FirmwareGripperCommand::default();
        self.moving = false;
        self.pressing = false;
        self.driver.feed_watchdog();
    }

    /// Re-seed the jaw at `closed` (0 = fully open, 1 = fully closed) —
    /// the gripper half of the simulator's teleport. The in-progress
    /// firmware command follows the jaw, so a standing "go to position"
    /// does not drag it back off the teleported pose.
    pub fn teleport(&mut self, closed: f64) {
        self.pos_byte = (closed.clamp(0.0, 1.0) * 255.0).round();
        self.joint
            .reseed((1.0 - self.pos_byte / 255.0) * self.stroke_ticks);
        self.cmd.position = self.pos_byte as u8;
        self.calibration_ticks_left = 0;
        self.moving = false;
        self.pressing = false;
        self.detection = ObjectDetection::ReachedNoObject;
    }

    /// One fixed step of whichever controller owns the jaw.
    pub fn step(&mut self, dt: f64) {
        match self.ctrl {
            Ctrl::Motor => {
                let cmd = self
                    .driver
                    .control_step(self.joint.pos, self.joint.reported_vel);
                self.joint.step(dt, &cmd, self.load_ma);
                self.pos_byte =
                    255.0 * (1.0 - (self.joint.pos / self.stroke_ticks).clamp(0.0, 1.0));
            }
            Ctrl::Firmware => {
                self.driver.age_watchdog();
                self.firmware_step(dt);
                self.joint.pos = (1.0 - self.pos_byte / 255.0) * self.stroke_ticks;
            }
        }
    }

    fn firmware_step(&mut self, dt: f64) {
        if self.calibration_ticks_left > 0 {
            if self.driver.watchdog_fired() {
                // Bus went quiet mid-calibration: the sequence halts
                // uncalibrated (the empty poll every tick is mandatory).
                self.calibration_ticks_left = 0;
                return;
            }
            self.calibration_ticks_left -= 1;
            // Close-then-open sweep, ending fully open.
            let progress =
                1.0 - self.calibration_ticks_left as f64 / self.calibration_total_ticks as f64;
            self.pos_byte = if progress < 0.5 {
                510.0 * progress
            } else {
                510.0 * (1.0 - progress)
            };
            self.moving = true;
            self.detection = ObjectDetection::Moving;
            if self.calibration_ticks_left == 0 {
                self.calibrated = true;
                self.pos_byte = 0.0;
                self.moving = false;
            }
            return;
        }
        self.moving = false;
        if self.driver.watchdog_fired()
            || self.estop_latched
            || !self.cmd.activate
            || !self.cmd.action
        {
            self.pressing = false;
            return;
        }
        let target = f64::from(self.cmd.position);
        let closing = target > self.pos_byte;
        // The jaw stops early where an obstruction sits.
        let block = if closing {
            self.object_close_at.map(f64::from).filter(|b| *b < target)
        } else {
            self.object_open_at.map(f64::from).filter(|b| *b > target)
        };
        let stop_at = match block {
            Some(b) => b,
            None => target,
        };
        let rate = f64::from(self.cmd.speed).max(1.0) * BYTES_PER_S_PER_SPEED_UNIT * dt;
        let remaining = stop_at - self.pos_byte;
        if remaining.abs() > 1e-9 {
            let step = remaining.clamp(-rate, rate);
            self.pos_byte = (self.pos_byte + step).clamp(0.0, 255.0);
            self.moving = (stop_at - self.pos_byte).abs() > 1e-9;
        }
        if self.moving {
            self.detection = ObjectDetection::Moving;
            self.pressing = false;
        } else if block.is_some() {
            self.detection = if closing {
                ObjectDetection::DetectedClosing
            } else {
                ObjectDetection::DetectedOpening
            };
            self.pressing = true;
        } else if (target - self.pos_byte).abs() <= 1e-9 {
            self.detection = ObjectDetection::ReachedNoObject;
            self.pressing = false;
        }
    }

    /// Payload of the cmd-60 reply (position byte, i16 current, bit field
    /// [activated, action_status, det_lo, det_hi, temp, timeout, estop,
    /// calibrated] MSB-first).
    ///
    /// The detection bits go low-then-high, matching the firmware's own
    /// `Gripper_pack_data`: the status value's LOW bit lands on byte bit 5
    /// and its HIGH bit on bit 4. Emitting them the other way round is
    /// invisible against our own decoder and wrong against the arm.
    pub fn firmware_reply(&self) -> (u8, i16, [bool; 8]) {
        let cur = if self.pressing {
            f64::from(self.cmd.current_ma)
        } else if self.moving || self.calibration_ticks_left > 0 {
            MOVING_CUR_MA.min(f64::from(self.cmd.current_ma.max(100)))
        } else {
            0.0
        };
        let det = self.detection as u8;
        (
            self.pos_byte.round() as u8,
            cur.round() as i16,
            [
                self.cmd.activate || self.calibration_ticks_left > 0,
                self.moving || self.calibration_ticks_left > 0,
                det & 0b01 != 0,
                det & 0b10 != 0,
                self.driver.flags().temperature,
                false,
                self.estop_latched,
                self.calibrated,
            ],
        )
    }
}
