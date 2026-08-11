//! Closed-loop simulated [`DriverBus`] backend (Tier-1 sim, spec/CAN.md +
//! spec/HOMING.md "Sim requirements").
//!
//! One virtual Spectral driver per CAN node consumes the REAL host→driver
//! frames (encoded by the production codec, parsed by DLC exactly like
//! firmware) and runs the real command semantics from the config gains:
//! cascade position→velocity-PI→current with Ilim saturation, velocity
//! and current/torque modes, PD impedance, the HALL homing pack, the
//! driver watchdog (command silence → Idle), live config-frame updates,
//! telemetry replies on the RTR round-robin, per-type fault injection and
//! the per-frame live err bit.
//!
//! Behind the drivers sits a plant stepped at the fixed tick dt (time is
//! the caller's tick counter — wall clock never enters):
//!
//! - default: a rate-limited kinematic plant per joint
//!   ([`plant::KinJoint`]) with hard endstops, stall behavior (current
//!   winds to the loop's saturated output, displacement plateaus),
//!   gearbox-windup preload that relaxes during release-phase current
//!   commands, and hall-sensor emulation at configured trigger positions;
//! - feature `sim-dynamics`: torque-level dynamics through Pinocchio ABA
//!   ([`dynamics::DynamicsPlant`]) — motor torques + gravity + friction +
//!   endstop spring-dampers, semi-implicit Euler integration.
//!
//! Encoder output goes through the real spectral conversions: positions
//! report from the 14-bit-wrapped boot reading (sector semantics) and
//! replies are packed/decoded with the production codec (i24 wrap
//! included), so the RT side cannot tell sim from real above the
//! [`DriverBus`] line. Everything is deterministic: identical tick and
//! command streams produce bit-identical state streams.

mod driver;
#[cfg(feature = "sim-dynamics")]
mod dynamics;
mod gripper;
mod plant;

pub use driver::FaultKind;

use std::collections::VecDeque;
#[cfg(feature = "sim-dynamics")]
use std::path::PathBuf;

use par6_config::{Gains, GripperConfig, KtSource, RobotConfig, WatchdogAction};

use crate::bus::DriverBus;
use crate::spectral::codec::{
    decode_frame, encode_clear_error, encode_current_gains, encode_gripper_command, encode_limits,
    encode_pd_gains, encode_position_gains, encode_velocity_gains, encode_voltage_limit,
    encode_watchdog, fold_bits_msb_first, pack_can_id, pack_f32, pack_i16, pack_i24, pack_i32,
    unfold_bits_msb_first, unpack_can_id, unpack_i16, CanFrame, CommandId, Payload,
};
use crate::spectral::convert::JointConversion;
use crate::types::{
    BusError, BusState, DeviceInfo, ErrorFlags, FirmwareGripperCommand, Freshness, GripperCommand,
    HallState, JointCommand, LinkHealth, NodeId, PollAction, PollKind, MAX_NODES,
};

use driver::{PlantCmd, ReplyKind, VirtualDriver};
use gripper::GripperSim;
use plant::{JointMap, KinJoint};

/// RX queue capacity \[frames\]. Replies past it are dropped, mirroring
/// the silent kernel-queue drop of a saturated real interface.
const RX_QUEUE_CAP: usize = 512;
/// Poll slots between device-info sweeps (~4 s at 250 Hz, spec/CAN.md).
const DEVICE_INFO_PERIOD_SLOTS: u64 = 1006;
/// Default hall-sensor band half-width \[rad\].
const HALL_HALF_WIDTH_RAD: f64 = 0.02;

/// Config values re-sent to a node on the reconnect path.
struct NodeConfig {
    node: NodeId,
    watchdog_ms: u32,
    action: WatchdogAction,
    velocity_limit_ticks_s: f64,
    ilim_ma: f64,
    voltage_limit_mv: u32,
    gains: Gains,
}

enum ArmPlant {
    Kinematic(Vec<KinJoint>),
    #[cfg(feature = "sim-dynamics")]
    Dynamics(dynamics::DynamicsPlant),
}

/// The closed-loop sim bus. Construct, optionally set hooks
/// ([`set_initial_joint_rad`](Self::set_initial_joint_rad)), then
/// [`DriverBus::boot_configure`] with the real configs before any
/// per-tick call.
pub struct SimBus {
    tick: u64,
    dt: f64,
    silent: bool,
    configured: bool,
    joint_nodes: Vec<NodeId>,
    node_to_joint: [Option<usize>; MAX_NODES],
    gripper_node: NodeId,
    timing_dummy_node: NodeId,
    stale_warn_ticks: u64,
    lost_ticks: u64,
    rx_cap: usize,
    last_rx_tick: [Option<u64>; MAX_NODES],
    lost_latched: [bool; MAX_NODES],
    last_gripper_rx_tick: Option<u64>,
    connected: u16,
    drivers: Vec<VirtualDriver>,
    gripper: Option<GripperSim>,
    plant: ArmPlant,
    maps: Vec<JointMap>,
    loads_ma: Vec<f64>,
    cmd_buf: Vec<PlantCmd>,
    hall_bands: Vec<Option<(f64, f64)>>,
    rx: VecDeque<(u64, CanFrame)>,
    dropped_rx: u64,
    node_configs: Vec<NodeConfig>,
    poll_cursor: u64,
    slot_counter: u64,
    di_remaining: usize,
    override_slot: Option<(PollAction, u16)>,
    joints_sent_this_tick: bool,
    health: LinkHealth,
    initial_q: Option<Vec<f64>>,
    #[cfg(feature = "sim-dynamics")]
    urdf: Option<PathBuf>,
}

impl SimBus {
    /// A sim bus with the default rate-limited kinematic plant.
    pub fn new() -> Self {
        Self {
            tick: 0,
            dt: 0.004,
            silent: false,
            configured: false,
            joint_nodes: Vec::new(),
            node_to_joint: [None; MAX_NODES],
            gripper_node: 0,
            timing_dummy_node: 0,
            stale_warn_ticks: u64::MAX,
            lost_ticks: u64::MAX,
            rx_cap: 32,
            last_rx_tick: [None; MAX_NODES],
            lost_latched: [false; MAX_NODES],
            last_gripper_rx_tick: None,
            connected: 0,
            drivers: Vec::new(),
            gripper: None,
            plant: ArmPlant::Kinematic(Vec::new()),
            maps: Vec::new(),
            loads_ma: Vec::new(),
            cmd_buf: Vec::new(),
            hall_bands: Vec::new(),
            rx: VecDeque::new(),
            dropped_rx: 0,
            node_configs: Vec::new(),
            poll_cursor: 0,
            slot_counter: 0,
            di_remaining: 0,
            override_slot: None,
            joints_sent_this_tick: false,
            health: LinkHealth::default(),
            initial_q: None,
            #[cfg(feature = "sim-dynamics")]
            urdf: None,
        }
    }

    /// A sim bus whose arm plant is the torque-level Pinocchio dynamics
    /// model built from `urdf` at [`DriverBus::boot_configure`] time.
    /// Panics at boot if the URDF cannot be loaded or its joint count
    /// does not match the robot config.
    #[cfg(feature = "sim-dynamics")]
    pub fn with_dynamics(urdf: impl Into<PathBuf>) -> Self {
        let mut bus = Self::new();
        bus.urdf = Some(urdf.into());
        bus
    }

    /// Override the true boot pose \[rad\], one entry per joint (default:
    /// the config boot-calibration pose, where each joint reads its
    /// `sector_home_offset`). Call before `boot_configure`; values are
    /// clamped inside the hard limits.
    pub fn set_initial_joint_rad(&mut self, q0: &[f64]) {
        self.initial_q = Some(q0.to_vec());
    }

    /// Move a hall sensor: joint `joint`'s trigger band becomes
    /// `center_rad ± half_width_rad` (defaults come from the homing
    /// config: hall-strategy joints trigger at their `home_offset_rad`).
    /// Call after `boot_configure`.
    pub fn set_hall_trigger(&mut self, joint: usize, center_rad: f64, half_width_rad: f64) {
        self.hall_bands[joint] = Some((center_rad, half_width_rad));
    }

    /// Inject a driver fault on `node`: the per-type cmd-26 flag (plus the
    /// aggregate error bit) is raised and every reply from that node
    /// carries the live err bit until a Clear_Error (cmd 1) arrives.
    pub fn inject_fault(&mut self, node: NodeId, fault: FaultKind) {
        if let Some(d) = self.driver_mut(node) {
            d.set_fault(fault);
        }
    }

    /// Constant external load on `node`'s motor, in motor-current
    /// equivalent \[mA\] (positive opposes positive motion). The
    /// dynamics plant converts it to a joint torque through the config
    /// torque↔current factor.
    pub fn set_joint_load_ma(&mut self, node: NodeId, load_ma: f64) {
        if let Some(j) = self.node_to_joint[usize::from(node)] {
            self.loads_ma[j] = load_ma;
        } else if node == self.gripper_node {
            if let Some(g) = &mut self.gripper {
                g.load_ma = load_ma;
            }
        }
    }

    /// Put (or remove) an object between the jaws: closing jams at this
    /// position byte (`None` = free travel).
    pub fn set_gripper_object_closing(&mut self, at: Option<u8>) {
        if let Some(g) = &mut self.gripper {
            g.object_close_at = at;
        }
    }

    /// Jam (or free) the opening direction at this position byte.
    pub fn set_gripper_object_opening(&mut self, at: Option<u8>) {
        if let Some(g) = &mut self.gripper {
            g.object_open_at = at;
        }
    }

    /// Frames dropped because the RX queue was full.
    pub fn dropped_rx_frames(&self) -> u64 {
        self.dropped_rx
    }

    // ------------------------------------------------------------------
    // internals
    // ------------------------------------------------------------------

    fn driver_mut(&mut self, node: NodeId) -> Option<&mut VirtualDriver> {
        if let Some(j) = self.node_to_joint[usize::from(node)] {
            return Some(&mut self.drivers[j]);
        }
        if node == self.gripper_node {
            return self.gripper.as_mut().map(|g| &mut g.driver);
        }
        None
    }

    fn ensure_ready(&self) -> Result<(), BusError> {
        if !self.configured {
            return Err(BusError::NotConfigured);
        }
        if self.silent {
            return Err(BusError::InvalidCommand {
                reason: "TX while bus-silent (FLASHING)",
            });
        }
        Ok(())
    }

    fn enqueue(&mut self, frame: CanFrame) {
        if self.rx.len() >= RX_QUEUE_CAP {
            self.dropped_rx += 1;
            return;
        }
        self.rx.push_back((self.tick, frame));
    }

    fn motor_state(&self, j: usize) -> (f64, f64) {
        match &self.plant {
            ArmPlant::Kinematic(joints) => (joints[j].pos, joints[j].reported_vel),
            #[cfg(feature = "sim-dynamics")]
            ArmPlant::Dynamics(d) => d.motor_state(j, &self.maps[j]),
        }
    }

    /// One fixed-dt physics step: drivers close their loops on the
    /// measured state, the plant integrates, hall bands update.
    fn step_once(&mut self) {
        let dt = self.dt;
        for j in 0..self.drivers.len() {
            let (p, v) = self.motor_state(j);
            self.cmd_buf[j] = self.drivers[j].control_step(p, v);
        }
        match &mut self.plant {
            ArmPlant::Kinematic(joints) => {
                for (j, joint) in joints.iter_mut().enumerate() {
                    joint.step(dt, &self.cmd_buf[j], self.loads_ma[j]);
                }
            }
            #[cfg(feature = "sim-dynamics")]
            ArmPlant::Dynamics(d) => d.step(dt, &self.cmd_buf, &self.loads_ma, &self.maps),
        }
        if let Some(g) = &mut self.gripper {
            g.step(dt);
        }
        for j in 0..self.drivers.len() {
            let Some((center, half)) = self.hall_bands[j] else {
                continue;
            };
            let (pos, _) = self.motor_state(j);
            let in_band = (self.maps[j].joint_rad(pos) - center).abs() <= half;
            let d = &mut self.drivers[j];
            if in_band && !d.hall_in_band {
                d.hall_latched_ticks = Some(self.maps[j].report_pos(pos));
                d.hall_edge_pending = true;
            }
            d.hall_in_band = in_band;
        }
    }

    fn joint_reply_values(&self, j: usize) -> (i32, i32, i16) {
        let (pos, vel) = self.motor_state(j);
        let cur = self.drivers[j].cur_out_ma;
        (
            self.maps[j].report_pos(pos),
            vel.round() as i32,
            cur.round().clamp(f64::from(i16::MIN), f64::from(i16::MAX)) as i16,
        )
    }

    fn motion_reply(node: NodeId, err: bool, pos: i32, spd: i32, cur: i16) -> CanFrame {
        let mut p = [0u8; 8];
        p[0..3].copy_from_slice(&pack_i24(pos));
        p[3..6].copy_from_slice(&pack_i24(spd));
        p[6..8].copy_from_slice(&pack_i16(cur));
        CanFrame::data_frame(pack_can_id(node, CommandId::RespondDataPack1, err), &p)
    }

    fn hall_reply(node: NodeId, err: bool, pos: i32, state: HallState) -> CanFrame {
        let mut p = [0u8; 4];
        p[0..3].copy_from_slice(&pack_i24(pos));
        p[3] = fold_bits_msb_first([
            state.trigger,
            state.pin2,
            state.edge,
            false,
            false,
            false,
            false,
            false,
        ]);
        CanFrame::data_frame(pack_can_id(node, CommandId::RespondDataHall, err), &p)
    }

    fn errors_payload(flags: ErrorFlags) -> [u8; 2] {
        [
            fold_bits_msb_first([
                flags.error,
                flags.temperature,
                flags.encoder,
                flags.vbus,
                flags.driver,
                flags.velocity,
                flags.current,
                flags.estop,
            ]),
            fold_bits_msb_first([
                flags.calibrated,
                flags.activated,
                flags.watchdog,
                false,
                false,
                false,
                false,
                false,
            ]),
        ]
    }

    /// Deliver an RTR telemetry poll to `node` and enqueue its reply.
    /// Nodes without a driver (the timing dummy) stay silent.
    fn deliver_rtr(&mut self, node: NodeId, kind: PollKind) {
        let joint = self.node_to_joint[usize::from(node)];
        let motion = joint.map(|j| self.joint_reply_values(j)).or_else(|| {
            if node == self.gripper_node {
                self.gripper.as_ref().map(|g| {
                    (
                        g.joint.pos.round() as i32,
                        g.joint.reported_vel.round() as i32,
                        g.driver.cur_out_ma.round() as i16,
                    )
                })
            } else {
                None
            }
        });
        let Some(d) = self.driver_mut(node) else {
            return;
        };
        let err = d.err_bit();
        let frame = match kind {
            PollKind::Temperature => CanFrame::data_frame(
                pack_can_id(node, CommandId::Temperature, err),
                &pack_i16(d.temperature_c),
            ),
            PollKind::Voltage => CanFrame::data_frame(
                pack_can_id(node, CommandId::Voltage, err),
                &pack_i16(d.voltage_mv),
            ),
            PollKind::Errors => CanFrame::data_frame(
                pack_can_id(node, CommandId::StateOfErrors, err),
                &Self::errors_payload(d.flags()),
            ),
            PollKind::DeviceInfo => {
                let DeviceInfo {
                    hw_ver,
                    batch,
                    sw_ver,
                    serial,
                } = d.device;
                let mut p = [0u8; 7];
                p[0] = hw_ver;
                p[1] = batch;
                p[2] = sw_ver;
                p[3..7].copy_from_slice(&pack_i32(serial));
                CanFrame::data_frame(pack_can_id(node, CommandId::DeviceInfo, err), &p)
            }
            PollKind::Kt => CanFrame::data_frame(
                pack_can_id(node, CommandId::RespondKt, err),
                &pack_f32(d.kt_nm_a),
            ),
            PollKind::Ping => {
                CanFrame::data_frame(pack_can_id(node, CommandId::Ping, err), &[])
            }
            PollKind::Encoder => {
                let (pos, spd, _) = motion.unwrap_or((0, 0, 0));
                let mut p = [0u8; 8];
                p[0..4].copy_from_slice(&pack_i32(pos));
                p[4..8].copy_from_slice(&pack_i32(spd));
                CanFrame::data_frame(pack_can_id(node, CommandId::EncoderData, err), &p)
            }
        };
        self.enqueue(frame);
    }

    /// Deliver one host→driver DATA frame to its node and enqueue
    /// whatever the driver replies.
    fn deliver_data(&mut self, frame: &CanFrame) {
        let (node, raw_cmd, _) = unpack_can_id(frame.id);
        let Some(cmd) = CommandId::from_raw(raw_cmd) else {
            return;
        };
        if node == self.gripper_node && self.gripper.is_some() {
            self.deliver_gripper_data(cmd, frame);
            return;
        }
        let Some(j) = self.node_to_joint[usize::from(node)] else {
            return;
        };
        let reply = self.drivers[j].on_data_frame(cmd, frame.payload());
        match reply {
            ReplyKind::None => {}
            ReplyKind::Motion => {
                let err = self.drivers[j].err_bit();
                let (pos, spd, cur) = self.joint_reply_values(j);
                let f = Self::motion_reply(node, err, pos, spd, cur);
                self.enqueue(f);
            }
            ReplyKind::Hall => {
                let err = self.drivers[j].err_bit();
                let (live_pos, _, _) = self.joint_reply_values(j);
                let d = &mut self.drivers[j];
                let state = HallState {
                    trigger: !d.hall_in_band,
                    pin2: false,
                    edge: d.hall_edge_pending,
                };
                d.hall_edge_pending = false;
                let pos = if d.hall_in_band {
                    d.hall_latched_ticks.unwrap_or(live_pos)
                } else {
                    live_pos
                };
                let f = Self::hall_reply(node, err, pos, state);
                self.enqueue(f);
            }
        }
    }

    fn deliver_gripper_data(&mut self, cmd: CommandId, frame: &CanFrame) {
        let d = frame.payload();
        let motor_reply = {
            let g = self.gripper.as_mut().expect("checked by caller");
            match (cmd, d.len()) {
                (CommandId::GripperDataPack, 5) => {
                    let bits = unfold_bits_msb_first(d[4]);
                    g.on_firmware_command(FirmwareGripperCommand {
                        position: d[0],
                        speed: d[1],
                        current_ma: unpack_i16([d[2], d[3]]),
                        activate: bits[0],
                        action: bits[1],
                        estop: bits[2],
                        release_dir: bits[3],
                    });
                    None
                }
                (CommandId::GripperDataPack, 0) => {
                    g.on_empty_poll();
                    None
                }
                (CommandId::GripperCalibrate, 0) => {
                    g.on_calibrate();
                    None
                }
                _ => Some(g.on_motor_frame(cmd, d)),
            }
        };
        match motor_reply {
            None => self.enqueue_gripper_reply(),
            Some(ReplyKind::Motion) => {
                let g = self.gripper.as_ref().expect("checked by caller");
                let f = Self::motion_reply(
                    self.gripper_node,
                    g.driver.err_bit(),
                    g.joint.pos.round() as i32,
                    g.joint.reported_vel.round() as i32,
                    g.driver
                        .cur_out_ma
                        .round()
                        .clamp(f64::from(i16::MIN), f64::from(i16::MAX))
                        as i16,
                );
                self.enqueue(f);
            }
            Some(_) => {}
        }
    }

    /// cmd-60 firmware gripper reply.
    fn enqueue_gripper_reply(&mut self) {
        let Some(g) = self.gripper.as_ref() else {
            return;
        };
        let (pos, cur, bits) = g.firmware_reply();
        let mut p = [0u8; 4];
        p[0] = pos;
        p[1..3].copy_from_slice(&pack_i16(cur));
        p[3] = fold_bits_msb_first(bits);
        let f = CanFrame::data_frame(
            pack_can_id(self.gripper_node, CommandId::RespondGripperData, g.driver.err_bit()),
            &p,
        );
        self.enqueue(f);
    }

    /// Send one node's full stored configuration, `repeats` passes
    /// (Watchdog → Limits → Voltage_Limit → PD → Current → Velocity →
    /// Position gains — the spec/CAN.md boot order).
    fn apply_node_config(&mut self, node: NodeId, repeats: u8) {
        let Some(i) = self.node_configs.iter().position(|c| c.node == node) else {
            return;
        };
        for _ in 0..repeats {
            let c = &self.node_configs[i];
            let frames = [
                encode_watchdog(node, c.watchdog_ms, c.action),
                encode_limits(node, c.velocity_limit_ticks_s as f32, c.ilim_ma as f32),
                encode_voltage_limit(node, c.voltage_limit_mv),
                encode_pd_gains(node, c.gains.kp as f32, c.gains.kd as f32),
                encode_current_gains(node, c.gains.kpiq as f32, c.gains.kiiq as f32),
                encode_velocity_gains(node, c.gains.kpv as f32, c.gains.kiv as f32),
                encode_position_gains(node, c.gains.kpp as f32),
            ];
            for f in frames {
                self.deliver_data(&f);
            }
        }
    }

    fn poll_targets(&self) -> usize {
        self.joint_nodes.len() + 1
    }

    fn poll_target_node(&self, idx: usize) -> NodeId {
        if idx < self.joint_nodes.len() {
            self.joint_nodes[idx]
        } else {
            self.gripper_node
        }
    }

    fn apply(decoded: &crate::spectral::codec::DecodedFrame, state: &mut BusState) {
        let n = usize::from(decoded.node);
        match decoded.payload {
            Payload::Motion {
                position_ticks,
                speed_ticks_s,
                current_ma,
            } => {
                state.nodes[n].position_ticks = Some(position_ticks);
                state.nodes[n].speed_ticks_s = Some(speed_ticks_s);
                state.nodes[n].current_ma = Some(current_ma);
            }
            Payload::Encoder {
                position_ticks,
                speed_ticks_s,
            } => {
                state.nodes[n].position_ticks = Some(position_ticks);
                state.nodes[n].speed_ticks_s = Some(speed_ticks_s);
            }
            Payload::Hall {
                position_ticks,
                state: hall,
            } => {
                state.nodes[n].position_ticks = Some(position_ticks);
                state.nodes[n].hall = Some(hall);
            }
            Payload::Temperature { deg_c } => state.nodes[n].temperature_c = Some(deg_c),
            Payload::Voltage { mv } => state.nodes[n].voltage_mv = Some(mv),
            Payload::IqCurrent { ma } => state.nodes[n].current_ma = Some(ma),
            Payload::Errors(flags) => state.nodes[n].error_flags = Some(flags),
            Payload::DeviceInfo(info) => state.nodes[n].device_info = Some(info),
            Payload::Kt { nm_per_a } => state.nodes[n].kt_nm_a = Some(nm_per_a),
            Payload::Gripper(reply) => {
                state.gripper.reply = Some(reply);
                state.gripper.live_error_bit = decoded.err_bit;
            }
            Payload::Ping | Payload::Heartbeat => {}
        }
        state.nodes[n].live_error_bit = decoded.err_bit;
    }
}

impl Default for SimBus {
    fn default() -> Self {
        Self::new()
    }
}

impl DriverBus for SimBus {
    fn begin_tick(&mut self, tick: u64) {
        debug_assert!(tick >= self.tick, "tick must be non-decreasing");
        if self.configured {
            for _ in self.tick..tick {
                self.step_once();
            }
        }
        self.tick = tick;
        self.joints_sent_this_tick = false;
        if !self.silent {
            for n in 0..MAX_NODES {
                if let Some(last) = self.last_rx_tick[n] {
                    if tick.saturating_sub(last) >= self.lost_ticks {
                        self.lost_latched[n] = true;
                    }
                }
            }
        }
    }

    fn drain_rx(&mut self, state: &mut BusState) -> Result<usize, BusError> {
        if !self.configured {
            return Err(BusError::NotConfigured);
        }
        state.frames_last_drain = 0;
        state.frame_age_min_ticks = 0;
        state.frame_age_max_ticks = 0;
        state.reconnected_mask = 0;
        let cap = if self.silent { 64 } else { self.rx_cap };
        let mut count = 0usize;
        let mut age_min = u64::MAX;
        let mut age_max = 0u64;
        while count < cap {
            let Some((enqueued, frame)) = self.rx.pop_front() else {
                break;
            };
            count += 1;
            self.health.rx_frames += 1;
            if self.silent {
                // FLASHING: drain-and-discard, never decode.
                continue;
            }
            let age = self.tick.saturating_sub(enqueued);
            age_min = age_min.min(age);
            age_max = age_max.max(age);
            // Harvest node + err bit BEFORE payload dispatch; refused
            // frames still count for freshness and the live fault bit.
            let (node, err_bit) = match decode_frame(&frame) {
                Ok(d) => {
                    Self::apply(&d, state);
                    (d.node, d.err_bit)
                }
                Err(e) => (e.node(), e.err_bit()),
            };
            let n = usize::from(node);
            state.nodes[n].live_error_bit = err_bit;
            if let Some(last) = self.last_rx_tick[n] {
                if self.tick.saturating_sub(last) >= self.stale_warn_ticks {
                    state.reconnected_mask |= 1 << n;
                }
            }
            self.last_rx_tick[n] = Some(self.tick);
            let (_, raw_cmd, _) = unpack_can_id(frame.id);
            if raw_cmd == CommandId::RespondGripperData.raw() {
                self.last_gripper_rx_tick = Some(self.tick);
            }
        }
        state.frames_last_drain = count as u32;
        if count > 0 && !self.silent {
            state.frame_age_min_ticks = age_min;
            state.frame_age_max_ticks = age_max;
        }
        for n in 0..MAX_NODES {
            state.nodes[n].data_age_ticks = match self.last_rx_tick[n] {
                Some(last) => self.tick.saturating_sub(last),
                None => u64::MAX,
            };
        }
        state.gripper.data_age_ticks = match self.last_gripper_rx_tick {
            Some(last) => self.tick.saturating_sub(last),
            None => u64::MAX,
        };
        Ok(count)
    }

    fn send_joint_commands(&mut self, commands: &[JointCommand]) -> Result<(), BusError> {
        self.ensure_ready()?;
        if commands.len() != self.joint_nodes.len() {
            return Err(BusError::InvalidCommand {
                reason: "command slice length != configured joint count",
            });
        }
        if self.joints_sent_this_tick {
            return Err(BusError::InvalidCommand {
                reason: "second joint send in one tick (single-send invariant)",
            });
        }
        self.joints_sent_this_tick = true;
        for (i, cmd) in commands.iter().enumerate() {
            let node = self.joint_nodes[i];
            let frame =
                crate::spectral::codec::encode_joint_command(node, cmd).map_err(|_| {
                    BusError::InvalidCommand {
                        reason: "cmd 2 has no wire form for position without velocity",
                    }
                })?;
            if let Some(f) = frame {
                self.deliver_data(&f);
            }
        }
        Ok(())
    }

    fn send_gripper(&mut self, command: &GripperCommand) -> Result<(), BusError> {
        self.ensure_ready()?;
        let frame = encode_gripper_command(self.gripper_node, self.timing_dummy_node, command)
            .map_err(|_| BusError::InvalidCommand {
                reason: "cmd 2 has no wire form for position without velocity",
            })?;
        let Some(f) = frame else {
            return Ok(());
        };
        if f.rtr {
            // NoGripper: RTR ping to the timing dummy — nothing answers.
            return Ok(());
        }
        self.deliver_data(&f);
        Ok(())
    }

    fn poll_step(&mut self) -> Result<(), BusError> {
        if !self.configured {
            return Err(BusError::NotConfigured);
        }
        if self.silent {
            return Ok(());
        }
        if let Some((action, repeats)) = self.override_slot.take() {
            match action {
                PollAction::Poll { node, kind } => self.deliver_rtr(node, kind),
                PollAction::ClearError { node } => {
                    let f = encode_clear_error(node);
                    self.deliver_data(&f);
                }
                PollAction::ResendConfig { node } => self.apply_node_config(node, 1),
            }
            if repeats > 1 {
                self.override_slot = Some((action, repeats - 1));
            }
            return Ok(());
        }
        self.slot_counter += 1;
        if self.di_remaining > 0 {
            let idx = self.poll_targets() - self.di_remaining;
            self.di_remaining -= 1;
            let node = self.poll_target_node(idx);
            self.deliver_rtr(node, PollKind::DeviceInfo);
            return Ok(());
        }
        if self.slot_counter.is_multiple_of(DEVICE_INFO_PERIOD_SLOTS) {
            self.di_remaining = self.poll_targets();
        }
        let idx = (self.poll_cursor / 3) as usize % self.poll_targets();
        let node = self.poll_target_node(idx);
        let kind = match self.poll_cursor % 3 {
            0 => PollKind::Temperature,
            1 => PollKind::Voltage,
            _ => PollKind::Errors,
        };
        self.poll_cursor += 1;
        self.deliver_rtr(node, kind);
        Ok(())
    }

    fn queue_poll_override(&mut self, action: PollAction, repeats: u16) {
        if repeats == 0 {
            return;
        }
        self.override_slot = Some((action, repeats));
    }

    fn boot_configure(
        &mut self,
        robot: &RobotConfig,
        gripper: Option<&GripperConfig>,
        repeats: u8,
    ) -> Result<(), BusError> {
        let n = robot.joints.len();
        self.dt = robot.robot.tick_dt_s;
        self.joint_nodes = robot.joints.iter().map(|j| j.node_id).collect();
        self.node_to_joint = [None; MAX_NODES];
        for (j, node) in self.joint_nodes.iter().enumerate() {
            self.node_to_joint[usize::from(*node)] = Some(j);
        }
        self.gripper_node = robot.bus.gripper_node;
        self.timing_dummy_node = robot.bus.timing_dummy_node;
        self.stale_warn_ticks = u64::from(robot.ticks(robot.bus.stale_warn_s));
        self.lost_ticks = u64::from(robot.ticks(robot.bus.lost_s));
        self.rx_cap = robot.bus.rx_frames_per_tick_cap as usize;
        self.rx = VecDeque::with_capacity(RX_QUEUE_CAP);

        // True boot pose: config calibration pose unless overridden.
        let q0: Vec<f64> = match self.initial_q.take() {
            Some(q) => {
                assert_eq!(q.len(), n, "initial pose length != joint count");
                q.iter()
                    .zip(&robot.joints)
                    .map(|(q, j)| q.clamp(j.limits.hard_min_rad, j.limits.hard_max_rad))
                    .collect()
            }
            None => robot
                .joints
                .iter()
                .map(|j| JointConversion::from_config(j).joint_rad(j.sector_master_position_ticks))
                .collect(),
        };
        self.maps = robot
            .joints
            .iter()
            .zip(&q0)
            .map(|(j, q)| JointMap::from_config(j, *q))
            .collect();
        self.drivers = robot
            .joints
            .iter()
            .map(|j| {
                VirtualDriver::new(
                    self.dt,
                    j.node_id,
                    j.velocity_limit_ticks_s,
                    j.ilim_ma,
                    j.kt_nm_a,
                )
            })
            .collect();
        self.loads_ma = vec![0.0; n];
        self.cmd_buf = vec![
            PlantCmd {
                current_ma: 0.0,
                vel_limit_ticks_s: 0.0,
                idle: true,
            };
            n
        ];
        self.hall_bands = robot
            .homing
            .joints
            .iter()
            .map(|h| match h.strategy {
                par6_config::HomingStrategy::Hall => {
                    Some((h.home_offset_rad, HALL_HALF_WIDTH_RAD))
                }
                par6_config::HomingStrategy::Stall => None,
            })
            .collect();

        self.plant = {
            #[cfg(feature = "sim-dynamics")]
            if let Some(urdf) = &self.urdf {
                ArmPlant::Dynamics(dynamics::DynamicsPlant::new(urdf, &self.maps, &q0))
            } else {
                ArmPlant::Kinematic(Self::kinematic_joints(robot, &self.maps, &q0, self.dt))
            }
            #[cfg(not(feature = "sim-dynamics"))]
            ArmPlant::Kinematic(Self::kinematic_joints(robot, &self.maps, &q0, self.dt))
        };

        let has_can_gripper = gripper.is_some_and(|g| g.driver.is_some());
        self.gripper = if has_can_gripper {
            Some(GripperSim::new(
                self.dt,
                self.gripper_node,
                gripper.expect("has_can_gripper"),
            ))
        } else {
            None
        };

        // Stored per-node config for boot passes + reconnect resends.
        self.node_configs = robot
            .joints
            .iter()
            .map(|j| NodeConfig {
                node: j.node_id,
                watchdog_ms: j.watchdog_timeout_ms,
                action: robot.bus.watchdog_action,
                velocity_limit_ticks_s: j.velocity_limit_ticks_s,
                ilim_ma: j.ilim_ma,
                voltage_limit_mv: j.voltage_limit_mv,
                gains: j.gains,
            })
            .collect();
        if has_can_gripper {
            let d = gripper
                .and_then(|g| g.driver.as_ref())
                .expect("has_can_gripper");
            self.node_configs.push(NodeConfig {
                node: self.gripper_node,
                watchdog_ms: d.watchdog_timeout_ms,
                action: robot.bus.watchdog_action,
                velocity_limit_ticks_s: d.velocity_limit_ticks_s,
                ilim_ma: d.ilim_ma,
                voltage_limit_mv: d.voltage_limit_mv,
                gains: d.gains,
            });
        }

        self.configured = true;
        let nodes: Vec<NodeId> = self.node_configs.iter().map(|c| c.node).collect();
        for node in &nodes {
            self.apply_node_config(*node, repeats);
        }
        // Boot kt fetch (cmd 33 RTR per node) when kt comes from drivers.
        if robot.robot.kt_source == KtSource::Auto {
            for node in &nodes {
                self.deliver_rtr(*node, PollKind::Kt);
            }
        }
        // Bus scan: every simulated driver answers its ping.
        self.connected = nodes.iter().fold(0u16, |m, n| m | (1 << u16::from(*n)));
        Ok(())
    }

    fn resend_node_config(&mut self, node: NodeId, repeats: u8) -> Result<(), BusError> {
        self.ensure_ready()?;
        self.apply_node_config(node, repeats);
        Ok(())
    }

    fn send_limits(
        &mut self,
        node: NodeId,
        velocity_limit_ticks_s: f32,
        current_limit_ma: f32,
        repeats: u8,
    ) -> Result<(), BusError> {
        self.ensure_ready()?;
        for _ in 0..repeats {
            let f = encode_limits(node, velocity_limit_ticks_s, current_limit_ma);
            self.deliver_data(&f);
        }
        Ok(())
    }

    fn send_clear_error(&mut self, node: NodeId, repeats: u8) -> Result<(), BusError> {
        self.ensure_ready()?;
        for _ in 0..repeats {
            let f = encode_clear_error(node);
            self.deliver_data(&f);
        }
        Ok(())
    }

    fn set_silent(&mut self, silent: bool) {
        self.silent = silent;
    }

    fn is_silent(&self) -> bool {
        self.silent
    }

    fn freshness(&self, node: NodeId) -> Freshness {
        let n = usize::from(node);
        if self.lost_latched[n] {
            return Freshness::Lost;
        }
        match self.last_rx_tick[n] {
            None => Freshness::Unknown,
            Some(last) => {
                let age = self.tick.saturating_sub(last);
                if age >= self.lost_ticks {
                    Freshness::Lost
                } else if age >= self.stale_warn_ticks {
                    Freshness::Stale
                } else {
                    Freshness::Fresh
                }
            }
        }
    }

    fn clear_lost_latch(&mut self, node: NodeId) {
        let n = usize::from(node);
        self.lost_latched[n] = false;
        self.last_rx_tick[n] = None;
    }

    fn rebase_freshness(&mut self) {
        self.last_rx_tick = [None; MAX_NODES];
        self.lost_latched = [false; MAX_NODES];
        self.last_gripper_rx_tick = None;
    }

    fn connected_nodes(&self) -> u16 {
        self.connected
    }

    fn link_health(&self) -> LinkHealth {
        self.health
    }
}

impl SimBus {
    fn kinematic_joints(
        robot: &RobotConfig,
        maps: &[JointMap],
        q0: &[f64],
        dt: f64,
    ) -> Vec<KinJoint> {
        robot
            .joints
            .iter()
            .zip(maps)
            .zip(q0)
            .map(|((j, map), q)| {
                let pos0 = f64::from(map.conv.motor_ticks(*q));
                let accel_max = j.limits.acceleration_rad_s2 * map.tpr;
                KinJoint::new(dt, pos0, map.bound_lo, map.bound_hi, accel_max, j.ilim_ma)
            })
            .collect()
    }
}
