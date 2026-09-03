//! Robot-level schema: identity, tick timing, joints (limits/gains/driver
//! parameters), bus, protocol, and mode-default sections.

use serde::{Deserialize, Serialize};
use std::path::Path;

use crate::homing::HomingConfig;
use crate::io::IoConfig;
use crate::{invalid, read_to_string, ConfigError};

/// Strictly positive AND comparable — rejects NaN, unlike `v <= 0.0`.
fn is_positive(v: f64) -> bool {
    matches!(v.partial_cmp(&0.0), Some(std::cmp::Ordering::Greater))
}

/// Driver board firmware product on a CAN node. Used to refuse a
/// mismatched firmware image at flashing time — not a protocol difference.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum DriverType {
    /// STEPFOC stepper-FOC driver.
    Stepfoc,
    /// Spectral BLDC driver.
    SpectralBldc,
}

/// Where per-joint torque constants come from at boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum KtSource {
    /// Fetch kt from each driver over CAN at boot (cmd 33 RTR); fall back
    /// to the config value for nodes that do not respond.
    Auto,
    /// Always use the kt values from config, skip the CAN fetch.
    /// (Vendor system.xml calls this `xml`.)
    Config,
}

/// What the driver-side watchdog does when it fires.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WatchdogAction {
    /// Drop to Idle (cmd 12 behavior) — the vendor setting.
    Idle,
}

/// Cascade-PID and impedance-PD gains pushed to a driver at boot.
///
/// Field names match the vendor XML tags (KPP/KPV/KIV/KPIQ/KIIQ/KP/KD),
/// lowercased.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Gains {
    /// Position-loop proportional gain (cascade PID).
    pub kpp: f64,
    /// Velocity-loop proportional gain (cascade PID).
    pub kpv: f64,
    /// Velocity-loop integral gain (cascade PID).
    pub kiv: f64,
    /// Current-loop proportional gain (cascade PID).
    pub kpiq: f64,
    /// Current-loop integral gain (cascade PID).
    pub kiiq: f64,
    /// Impedance PD stiffness (cmd 16, used by the PD pack / cmd 4).
    pub kp: f64,
    /// Impedance PD damping (cmd 16).
    pub kd: f64,
}

/// Kinodynamic limits requested by one control mode. Any omitted field
/// falls back to the joint's hardware ceiling; no field may exceed it
/// (validated on load).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ModeLimits {
    /// Joint velocity limit \[rad/s\].
    pub velocity_rad_s: f64,
    /// Joint acceleration limit \[rad/s^2\].
    pub acceleration_rad_s2: f64,
    /// Joint jerk limit \[rad/s^3\]. `None` = ceiling.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub jerk_rad_s3: Option<f64>,
    /// Commanded-torque slew limit \[Nm/s\]. `None` = ceiling.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub torque_rate_nm_s: Option<f64>,
}

/// Control modes that carry their own kinodynamic limit set.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LimitMode {
    /// Queued/planned motion (EXEC).
    Exec,
    /// Manual jogging.
    Jog,
    /// Streamed external control (RTI).
    Stream,
}

/// Fully-resolved limits for one mode (every fallback applied).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResolvedLimits {
    /// Joint velocity limit \[rad/s\].
    pub velocity_rad_s: f64,
    /// Joint acceleration limit \[rad/s^2\].
    pub acceleration_rad_s2: f64,
    /// Joint jerk limit \[rad/s^3\]; `None` when the ceiling declares none.
    pub jerk_rad_s3: Option<f64>,
    /// Torque slew limit \[Nm/s\]; `None` when the ceiling declares none.
    pub torque_rate_nm_s: Option<f64>,
}

/// Position limits plus the hardware kinodynamic ceiling and optional
/// per-mode subsets.
///
/// The bare `velocity/acceleration/jerk/torque_rate` fields are the
/// HARDWARE CEILING — what the arm survives. A mode block may only ask
/// for less, never more (validated on load, mirroring the vendor
/// xml_parser clamp).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JointLimits {
    /// Mechanical endstop, negative side \[rad\].
    pub hard_min_rad: f64,
    /// Mechanical endstop, positive side \[rad\].
    pub hard_max_rad: f64,
    /// Software limit, negative side \[rad\]; must sit inside the hard limits.
    pub soft_min_rad: f64,
    /// Software limit, positive side \[rad\].
    pub soft_max_rad: f64,
    /// Ceiling joint velocity \[rad/s\].
    pub velocity_rad_s: f64,
    /// Ceiling joint acceleration \[rad/s^2\].
    pub acceleration_rad_s2: f64,
    /// Ceiling joint jerk \[rad/s^3\].
    pub jerk_rad_s3: f64,
    /// Ceiling commanded-torque slew \[Nm/s\].
    pub torque_rate_nm_s: f64,
    /// EXEC (queued motion) limits; omitted = ceiling.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exec: Option<ModeLimits>,
    /// JOG limits; omitted = ceiling (the vendor jog reads the shared field).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub jog: Option<ModeLimits>,
    /// Streaming (RTI) limits; omitted = ceiling.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stream: Option<ModeLimits>,
}

impl JointLimits {
    /// Resolve the limits one mode actually runs under, applying the
    /// fall-back-to-ceiling rule field by field.
    pub fn for_mode(&self, mode: LimitMode) -> ResolvedLimits {
        let block = match mode {
            LimitMode::Exec => self.exec.as_ref(),
            LimitMode::Jog => self.jog.as_ref(),
            LimitMode::Stream => self.stream.as_ref(),
        };
        match block {
            Some(m) => ResolvedLimits {
                velocity_rad_s: m.velocity_rad_s,
                acceleration_rad_s2: m.acceleration_rad_s2,
                jerk_rad_s3: Some(m.jerk_rad_s3.unwrap_or(self.jerk_rad_s3)),
                torque_rate_nm_s: Some(m.torque_rate_nm_s.unwrap_or(self.torque_rate_nm_s)),
            },
            None => ResolvedLimits {
                velocity_rad_s: self.velocity_rad_s,
                acceleration_rad_s2: self.acceleration_rad_s2,
                jerk_rad_s3: Some(self.jerk_rad_s3),
                torque_rate_nm_s: Some(self.torque_rate_nm_s),
            },
        }
    }
}

/// One arm joint: driver parameters, gains, limits, encoder geometry.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JointConfig {
    /// Human-readable joint name (`joint1`…).
    pub name: String,
    /// CAN node id (0–5 for J1–J6 on PAR6). Must be unique and must not
    /// collide with the gripper/timing/host nodes in `[bus]`.
    pub node_id: u8,
    /// Driver board firmware product on this node.
    pub driver_type: DriverType,
    /// Torque constant \[Nm/A\]. When `robot.kt_source = "auto"` this is
    /// the fallback for a driver that does not answer the boot kt fetch.
    pub kt_nm_a: f64,
    /// Driver current limit \[mA\] (cmd 20).
    pub ilim_ma: f64,
    /// Driver voltage limit \[mV\] (cmd 34); 0 = use VBUS. Old firmware
    /// ignores the frame.
    pub voltage_limit_mv: u32,
    /// Motor velocity limit \[encoder ticks/s\] (cmd 20).
    pub velocity_limit_ticks_s: f64,
    /// Driver watchdog timeout \[ms\] (cmd 15, wire unit is ms). Fires
    /// into [`WatchdogAction`] configured in `[bus]`.
    pub watchdog_timeout_ms: u32,
    /// Encoder resolution in bits (14 → 16384 counts).
    pub encoder_bits: u8,
    /// Gear ratio motor→joint.
    pub gear_ratio: f64,
    /// Gear ratio the SIM dynamics reflections use (rotor inertia,
    /// motor friction); omitted = `gear_ratio`. Exists because the
    /// vendor's dynamics table and motor config can disagree (J1: 20 in
    /// the dynamics table vs 25 in the motor config).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dynamics_gear_ratio: Option<f64>,
    /// Gear efficiency (0–1], used in the torque→current conversion.
    pub gear_efficiency: f64,
    /// 0 = joint positive rotation matches the kinematic model, 1 = inverted.
    pub dir: u8,
    /// Encoder ticks at the master position at boot (vendor
    /// `sector_home_master_position`) — input to boot sector selection.
    pub sector_master_position_ticks: i32,
    /// Joint-level offset from master position to the kinematic zero \[rad\]
    /// (vendor `sector_home_offset`).
    pub sector_home_offset_rad: f64,
    /// Controller gains pushed at boot.
    pub gains: Gains,
    /// Position limits + kinodynamic ceiling + per-mode blocks.
    pub limits: JointLimits,
}

/// Robot identity and global timing.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RobotSection {
    /// Robot name.
    pub name: String,
    /// RT tick period \[s\] (PAR6: 0.004 = 250 Hz).
    pub tick_dt_s: f64,
    /// Gravity for dynamics \[m/s^2\].
    pub gravity_m_s2: f64,
    /// Standby pose the arm is parked in before torque-losing maintenance
    /// (firmware flashing) \[rad\], one entry per joint.
    pub park_pose_rad: Vec<f64>,
    /// Name of the active gripper — must match a `grippers/*.toml` name.
    /// Drives kt/stroke/homing offsets AND the driver type that firmware
    /// flashing checks against.
    pub active_gripper: String,
    /// Where torque constants come from at boot.
    pub kt_source: KtSource,
}

/// Boot-time kt fetch parameters (cmd 33 RTR per node).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct KtFetchConfig {
    /// Per-request reply timeout \[s\].
    pub timeout_s: f64,
    /// Retries per node per round.
    pub retries: u8,
    /// Fetch rounds.
    pub rounds: u8,
}

/// Boot bus-scan parameters (RTR ping ids 0–15).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScanConfig {
    /// Scan rounds.
    pub rounds: u8,
    /// Per-ping reply wait \[s\].
    pub wait_s: f64,
}

/// CAN bus bring-up, node map, freshness thresholds, boot pacing.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BusConfig {
    /// SocketCAN interface name.
    pub interface: String,
    /// Bitrate \[bps\].
    pub bitrate: u32,
    /// Kernel auto-restart after bus-off \[ms\].
    pub restart_ms: u32,
    /// Interface TX queue length \[frames\].
    pub txqueuelen: u32,
    /// Requested SO_SNDBUF \[bytes\] (kernel caps at wmem_max).
    pub sndbuf_bytes: u32,
    /// CAN node id of the gripper (PAR6: 6). Joint node ids live on the
    /// joints themselves (`joints[i].node_id`).
    pub gripper_node: u8,
    /// Reserved timing-dummy node the backend pings when no gripper is
    /// fitted, keeping the per-tick frame cadence constant (PAR6: 13).
    pub timing_dummy_node: u8,
    /// Host node id (PAR6: 14).
    pub host_node: u8,
    /// Data age at which a node goes stale (live WARNING, self-clears) \[s\].
    /// 0.04 s = 10 ticks at 250 Hz.
    pub stale_warn_s: f64,
    /// Data age at which a node counts as disconnected (latched ERROR —
    /// only user clear-errors resets it) \[s\]. 0.2 s = 50 ticks at 250 Hz.
    pub lost_s: f64,
    /// Max RX frames drained per tick (surplus over the ~8 steady-state
    /// clears backlogs).
    pub rx_frames_per_tick_cap: u32,
    /// Config passes per node during boot configuration.
    pub boot_config_repeats: u8,
    /// How long daemon startup keeps retrying a failed bus open \[s\]
    /// (1 s pacing; 0 = fail on the first attempt). Covers the boot
    /// race where par6d starts before the CAN driver has enumerated
    /// the interface.
    #[serde(default = "default_open_retry_s")]
    pub open_retry_s: f64,
    /// Pacing between message-type batches during config load \[s\]
    /// (the TX queue silently drops on overflow).
    pub config_pace_s: f64,
    /// What the driver watchdog does when it fires.
    pub watchdog_action: WatchdogAction,
    /// Boot kt fetch parameters.
    pub kt_fetch: KtFetchConfig,
    /// Boot bus scan parameters.
    pub scan: ScanConfig,
}

/// UDP command/status/telemetry plane parameters (protocol v2).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProtocolConfig {
    /// Unicast UDP command port (msgpack envelopes).
    pub command_port: u16,
    /// Status broadcast port.
    pub status_port: u16,
    /// Status multicast group (IPv4); unicast fallback per the transport
    /// ladder.
    pub status_multicast_group: String,
    /// Telemetry stream port.
    pub telemetry_port: u16,
    /// Status broadcast rate \[Hz\]. Must divide the tick rate exactly
    /// (validated) so the broadcaster is a clean tick decimation.
    pub status_rate_hz: u32,
    /// Telemetry stream rate \[Hz\].
    #[serde(default = "default_telemetry_rate_hz")]
    pub telemetry_rate_hz: u32,
    /// Telemetry recipe active from boot. Omitted = the stream stays
    /// silent until a client's `set_recipe` — the shipped default. An
    /// unknown name refuses startup, like `set_recipe` refuses it live.
    #[serde(default)]
    pub initial_recipe: Option<String>,
}

fn default_telemetry_rate_hz() -> u32 {
    100
}

/// Jog profile shape.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JogProfile {
    /// Trapezoidal velocity ramp.
    Trapezoid,
    /// Jerk-limited s-curve ramp.
    Scurve,
}

/// Joint control packing selected by a mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlMode {
    /// Cascade PID (CAN cmd 2).
    Pid,
    /// Impedance PD (CAN cmd 4).
    Pd,
}

/// Jog startup defaults (vendor system.xml `jog_defaults`). Live state may
/// diverge as the user adjusts controls.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct JogDefaults {
    /// Fraction of max joint speed (0–1].
    pub speed_pct: f64,
    /// Ramp time from 0 to full speed \[s\] (runtime floor 0.05).
    pub accel_time_s: f64,
    /// Ramp shape.
    pub profile: JogProfile,
    /// jerk = accel × factor (s-curve only; runtime floor 0.5).
    pub jerk_factor: f64,
    /// pid or pd packing for jog frames.
    pub control_mode: ControlMode,
}

impl Default for JogDefaults {
    fn default() -> Self {
        Self {
            speed_pct: 0.20,
            accel_time_s: 0.55,
            profile: JogProfile::Scurve,
            jerk_factor: 3.0,
            control_mode: ControlMode::Pid,
        }
    }
}

/// Streaming (RTI) mode policy — robot-independent tuning (vendor
/// system.xml `rti_defaults`). Physical limits do NOT belong here; they
/// are per-joint properties.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct StreamDefaults {
    /// Silence on the stream link before the robot stops itself and
    /// latches `RTI_LINK_LOST` \[s\]. Also the minimum command rate.
    pub command_timeout_s: f64,
    /// Command low-pass cutoff \[Hz\]; 0 = off.
    pub lowpass_cutoff_hz: f64,
    /// Moving success-rate window \[s\] (0.4 s = 100 ticks at 250 Hz).
    pub success_window_s: f64,
    /// Consecutive rate-limiter step failures tolerated before the
    /// runtime hard-latches `STREAM_FAULT` \[s\] — a failing limiter
    /// holds in place, and a stream that silently holds instead of
    /// tracking must become an error the operator can see.
    pub fault_latch_s: f64,
}

impl Default for StreamDefaults {
    fn default() -> Self {
        Self {
            command_timeout_s: 0.040,
            lowpass_cutoff_hz: 0.0,
            success_window_s: 0.4,
            fault_latch_s: 0.5,
        }
    }
}

fn default_open_retry_s() -> f64 {
    10.0
}

/// Torque-level sim plant parameters (feature `sim-dynamics`): the
/// motor-referred rotor dynamics the vendor models
/// (robots/PAR6.py dynamics table — values only, no code). Reflected
/// through each joint's dynamics gear ratio G as G²·jm (inertia),
/// G²·b (viscous) and G·tc (Coulomb). Ignored by the kinematic plant
/// and by hardware.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct SimConfig {
    /// Motor rotor inertia per joint \[kg·m², motor side\].
    pub motor_jm_kg_m2: Vec<f64>,
    /// Motor viscous friction \[Nm·s/rad, motor side\], shared.
    pub motor_b_nm_s: f64,
    /// Motor Coulomb friction \[Nm, motor side\], shared.
    pub motor_tc_nm: f64,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            motor_jm_kg_m2: vec![1.02e-5, 1.02e-5, 5.7e-6, 5.7e-6, 5.7e-6, 1.5e-6],
            motor_b_nm_s: 1.0e-4,
            motor_tc_nm: 0.02,
        }
    }
}

/// Loop-period degradation bands.
///
/// The p99 of the measured loop period is compared against multiples of
/// the tick period: above `degraded_factor · dt` the runtime raises the
/// self-clearing `LOOP_DEGRADED` warning, and above `critical_factor ·
/// dt` held for `critical_sustain_s` it hard-latches `LOOP_CRITICAL`
/// (controller DISABLED).
///
/// Defaults are the vendor values, sized for a dedicated PREEMPT_RT host.
/// A host that cannot hold the tick deadline — a wall-clock simulator on
/// a shared CI runner — needs wider bands, or host jitter latches a
/// critical that no robot fault caused.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct TimingConfig {
    /// Warning band as a multiple of the tick period.
    pub degraded_factor: f64,
    /// Hard band as a multiple of the tick period.
    pub critical_factor: f64,
    /// How long the hard band must hold before latching \[s\].
    pub critical_sustain_s: f64,
    /// CPU the RT thread pins to on hardware; negative disables pinning.
    /// Ignored under `--sim`, which always runs unpinned and unprivileged.
    pub cpu: i64,
    /// SCHED_FIFO priority the RT thread requests on hardware (1..=99);
    /// 0 disables the request. Ignored under `--sim`.
    pub fifo_priority: u8,
}

impl Default for TimingConfig {
    fn default() -> Self {
        Self {
            degraded_factor: 1.05,
            critical_factor: 1.10,
            critical_sustain_s: 1.0,
            cpu: 3,
            fifo_priority: 99,
        }
    }
}

impl TimingConfig {
    /// Bands for a wall-clock simulator sharing its host with everything
    /// else on the box (CI runners, dev containers).
    ///
    /// Wide enough that ordinary scheduler starvation only raises the
    /// self-clearing warning, tight enough that a loop actually running
    /// several times slower than its period, continuously, still latches.
    /// The sustain also has to outlast the runtime's percentile recompute
    /// interval — a sustain shorter than one recompute reduces to "one
    /// bad percentile latches".
    pub const SIM: Self = Self {
        degraded_factor: 1.5,
        critical_factor: 4.0,
        critical_sustain_s: 5.0,
        // Scheduling knobs are hardware-only; the sim path never reads
        // them, so the vendor defaults ride along unchanged.
        cpu: 3,
        fifo_priority: 99,
    };
}

/// `[motion]`: the runtime's motion "feel" constants — sampling pitch,
/// solver damping, jog full-scale rates, completion settling. Defaults
/// are the values par6 shipped with as compiled constants; the section
/// makes them installation-tunable, and the offline preview reads the
/// same file, so a tuned deployment previews the way it runs.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct MotionConfig {
    /// Full-scale `jog_l` linear TCP speed \[m/s\] (a `velocities`
    /// fraction of ±1 maps to this).
    pub jog_l_linear_max_m_s: f64,
    /// Full-scale `jog_l` angular TCP speed \[rad/s\].
    pub jog_l_angular_max_rad_s: f64,
    /// MOVE_L sampling pitch: one IK waypoint per this much
    /// translation \[m\] …
    pub cart_step_m: f64,
    /// … or per this much rotation \[rad\], whichever yields more
    /// waypoints (vendor: 2°).
    pub cart_step_rad: f64,
    /// Multi-segment cartesian paths (arc, spline, process move, blend
    /// chains) sample much finer, on the combined length metric
    /// √(t² + (0.15·θ)²) \[m\] — the vendor's path pitch.
    pub path_step_m: f64,
    /// Joint-space blend-chain pitch \[rad\]; omitted = half the
    /// per-tick travel at the full EXEC velocity norm
    /// (0.5·‖v_exec‖·dt, floored at 0.01 — the vendor rule).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub joint_step_rad: Option<f64>,
    /// Largest joint change allowed between consecutive cartesian IK
    /// waypoints \[rad\]; a bigger jump means the solver hopped to
    /// another IK branch and the commanded path would whip the arm.
    pub move_l_max_joint_step_rad: f64,
    /// DLS damping λ for the jacobian velocity solve
    /// (`Jᵀ(JJᵀ+λ²I)⁻¹v`).
    pub dls_lambda: f64,
    /// All-joint settle tolerance \[rad\] for the `settled`/`strict`
    /// completion policies.
    pub settle_tolerance_rad: f64,
    /// Settle timeout \[s\].
    pub settle_timeout_s: f64,
}

impl Default for MotionConfig {
    fn default() -> Self {
        Self {
            jog_l_linear_max_m_s: 0.08,
            jog_l_angular_max_rad_s: 0.6,
            cart_step_m: 0.01,
            cart_step_rad: 0.034906585,
            path_step_m: 0.002,
            joint_step_rad: None,
            move_l_max_joint_step_rad: 0.35,
            dls_lambda: 0.05,
            settle_tolerance_rad: 0.01,
            settle_timeout_s: 2.0,
        }
    }
}

/// `[limits]`: safety-envelope enforcement thresholds.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct LimitsSection {
    /// External-torque envelope margin \[Nm\]: hard fault when a joint's
    /// external torque estimate (measured minus model, minus the active
    /// motion's feed-forward) stays beyond this for the window below.
    /// 0 disables the envelope — the shipped default, because the
    /// estimate is only as good as the torque constants and the gravity
    /// model; hardware opts in with a calibrated margin.
    pub tau_ext_margin_nm: f64,
    /// How long a joint must stay beyond the margin before the fault
    /// latches \[s\] — rides out measurement transients and filter lag.
    pub tau_ext_window_s: f64,
}

impl Default for LimitsSection {
    fn default() -> Self {
        Self {
            tau_ext_margin_nm: 0.0,
            tau_ext_window_s: 0.1,
        }
    }
}

/// The shutdown retreat: an opt-in slow drive to a rest pose before the
/// drives are idled, so an arm left mid-air by a process exit does not
/// drop from wherever it was when the terminal limp frame lands.
///
/// Off by default: a retreat is a motion, and a motion on shutdown must
/// be asked for. Durations are seconds; the runtime converts with
/// `round(s / dt)`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct ShutdownConfig {
    /// Drive to the rest pose on shutdown. Only a homed, enabled,
    /// error-free arm retreats; anything else goes straight to the
    /// existing halt-settle-limp exit.
    pub safe_park: bool,
    /// Per-joint distance to the rest pose that counts as arrived \[rad\].
    pub tolerance_rad: f64,
    /// Ceiling on the retreat \[s\]. Expiry is logged and the exit
    /// continues from wherever the arm is — it never blocks a shutdown.
    pub timeout_s: f64,
    /// Joint-velocity ceiling for the retreat \[rad/s\], applied to every
    /// joint as a fraction of the STREAM limits.
    pub velocity_limit_rad_s: f64,
    /// The rest pose \[rad\], one entry per joint. Omitted = the
    /// `robot.park_pose_rad` the homing return targets.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub safe_park_q: Option<Vec<f64>>,
}

impl Default for ShutdownConfig {
    fn default() -> Self {
        Self {
            safe_park: false,
            tolerance_rad: 0.03,
            timeout_s: 15.0,
            velocity_limit_rad_s: 0.25,
            safe_park_q: None,
        }
    }
}

/// Root of a robot TOML file.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RobotConfig {
    /// Identity and global timing.
    pub robot: RobotSection,
    /// Arm joints, in kinematic order.
    pub joints: Vec<JointConfig>,
    /// Homing parameters and sequence.
    pub homing: HomingConfig,
    /// CAN bus configuration.
    pub bus: BusConfig,
    /// UDP protocol plane configuration.
    pub protocol: ProtocolConfig,
    /// Digital I/O lines. Omitted = the stock control box's ten, which
    /// is the hardware par6 ships against; declare the section to
    /// describe a box wired differently.
    #[serde(default)]
    pub io: IoConfig,
    /// Jog startup defaults.
    #[serde(default)]
    pub jog: JogDefaults,
    /// Streaming mode policy defaults.
    #[serde(default)]
    pub stream: StreamDefaults,
    /// Loop-period degradation bands. Omitted = the vendor bands, so a
    /// config that says nothing keeps hardware behavior; `par6d --sim`
    /// fills this in with [`TimingConfig::SIM`] when it is absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub timing: Option<TimingConfig>,
    /// Safety-envelope enforcement thresholds. Omitted = everything
    /// disabled.
    #[serde(default)]
    pub limits: LimitsSection,
    /// Shutdown retreat. Omitted = no retreat.
    #[serde(default)]
    pub shutdown: ShutdownConfig,
    /// Motion feel constants. Omitted = the shipped defaults.
    #[serde(default)]
    pub motion: MotionConfig,
    /// Torque-level sim plant parameters. Omitted = the vendor table.
    #[serde(default)]
    pub sim: SimConfig,
}

impl RobotConfig {
    /// Parse and validate a robot config from a TOML string.
    pub fn from_toml_str(text: &str) -> Result<Self, ConfigError> {
        let cfg: Self = toml::from_str(text).map_err(|source| ConfigError::Parse {
            path: "<string>".into(),
            source: Box::new(source),
        })?;
        cfg.validate()?;
        Ok(cfg)
    }

    /// Load and validate a robot config file.
    pub fn load(path: &Path) -> Result<Self, ConfigError> {
        let text = read_to_string(path)?;
        let cfg: Self = toml::from_str(&text).map_err(|source| ConfigError::Parse {
            path: path.display().to_string(),
            source: Box::new(source),
        })?;
        cfg.validate()?;
        Ok(cfg)
    }

    /// Tick rate \[Hz\] derived from `robot.tick_dt_s`.
    pub fn tick_rate_hz(&self) -> f64 {
        1.0 / self.robot.tick_dt_s
    }

    /// The pose the shutdown retreat drives to \[rad\]: `shutdown.safe_park_q`
    /// when set, else `robot.park_pose_rad`.
    pub fn safe_park_q(&self) -> &[f64] {
        self.shutdown
            .safe_park_q
            .as_deref()
            .unwrap_or(&self.robot.park_pose_rad)
    }

    /// Convert a config time constant in seconds to ticks:
    /// `round(seconds / dt)`. The only sanctioned seconds→ticks path.
    pub fn ticks(&self, seconds: f64) -> u32 {
        (seconds / self.robot.tick_dt_s).round() as u32
    }

    /// The loop-period bands this config runs under, applying the
    /// fall-back-to-vendor rule.
    pub fn loop_timing(&self) -> TimingConfig {
        self.timing.unwrap_or_default()
    }

    /// Validate the whole tree; every error names its field.
    pub fn validate(&self) -> Result<(), ConfigError> {
        let r = &self.robot;
        if !(r.tick_dt_s > 0.0 && r.tick_dt_s < 1.0) {
            return Err(invalid("robot.tick_dt_s", "must be in (0, 1) seconds"));
        }
        if self.joints.is_empty() {
            return Err(invalid("joints", "at least one joint is required"));
        }
        if r.park_pose_rad.len() != self.joints.len() {
            return Err(invalid(
                "robot.park_pose_rad",
                format!(
                    "must have one entry per joint ({} joints, {} entries)",
                    self.joints.len(),
                    r.park_pose_rad.len()
                ),
            ));
        }
        let reserved = [
            (self.bus.gripper_node, "bus.gripper_node"),
            (self.bus.timing_dummy_node, "bus.timing_dummy_node"),
            (self.bus.host_node, "bus.host_node"),
        ];
        for (i, j) in self.joints.iter().enumerate() {
            self.validate_joint(i, j, &reserved)?;
        }
        self.homing.validate(self.joints.len())?;
        self.io.validate()?;
        self.validate_bus()?;
        self.validate_protocol()?;
        self.validate_defaults()?;
        self.validate_timing()?;
        self.validate_limits()?;
        self.validate_motion()?;
        self.validate_shutdown()?;
        self.validate_sim()?;
        Ok(())
    }

    fn validate_joint(
        &self,
        i: usize,
        j: &JointConfig,
        reserved: &[(u8, &str)],
    ) -> Result<(), ConfigError> {
        let f = |name: &str| format!("joints[{i}].{name}");
        if j.node_id >= 16 {
            return Err(invalid(
                f("node_id"),
                "11-bit CAN ids carry 4-bit node ids (0-15)",
            ));
        }
        for (other, name) in reserved {
            if j.node_id == *other {
                return Err(invalid(f("node_id"), format!("collides with {name}")));
            }
        }
        if self.joints[..i].iter().any(|o| o.node_id == j.node_id) {
            return Err(invalid(f("node_id"), "duplicate node id"));
        }
        if !(1..=24).contains(&j.encoder_bits) {
            return Err(invalid(f("encoder_bits"), "must be in 1..=24"));
        }
        if j.gear_ratio <= 0.0 {
            return Err(invalid(f("gear_ratio"), "must be > 0"));
        }
        if !(j.gear_efficiency > 0.0 && j.gear_efficiency <= 1.0) {
            return Err(invalid(f("gear_efficiency"), "must be in (0, 1]"));
        }
        if j.dir > 1 {
            return Err(invalid(f("dir"), "must be 0 or 1"));
        }
        if j.kt_nm_a <= 0.0 {
            return Err(invalid(f("kt_nm_a"), "must be > 0"));
        }
        if j.ilim_ma <= 0.0 {
            return Err(invalid(f("ilim_ma"), "must be > 0"));
        }
        if j.velocity_limit_ticks_s <= 0.0 {
            return Err(invalid(f("velocity_limit_ticks_s"), "must be > 0"));
        }
        if j.watchdog_timeout_ms == 0 {
            return Err(invalid(f("watchdog_timeout_ms"), "must be > 0"));
        }
        let l = &j.limits;
        if l.hard_min_rad >= l.hard_max_rad {
            return Err(invalid(
                f("limits.hard_min_rad"),
                "hard_min must be < hard_max",
            ));
        }
        if l.soft_min_rad >= l.soft_max_rad {
            return Err(invalid(
                f("limits.soft_min_rad"),
                "soft_min must be < soft_max",
            ));
        }
        // NOTE: soft ⊆ hard is deliberately NOT enforced. The soft window of a
        // wrapping joint lives in an unwrapped frame that can exceed the
        // endstop coordinates (PAR6 J6: hard ±2π, soft −0.85..7.14). Soft
        // limits are the authoritative motion bound; hard limits record the
        // mechanical endstop positions.
        for (v, name) in [
            (l.velocity_rad_s, "limits.velocity_rad_s"),
            (l.acceleration_rad_s2, "limits.acceleration_rad_s2"),
            (l.jerk_rad_s3, "limits.jerk_rad_s3"),
            (l.torque_rate_nm_s, "limits.torque_rate_nm_s"),
        ] {
            if v <= 0.0 {
                return Err(invalid(f(name), "must be > 0"));
            }
        }
        for (block, name) in [(l.exec, "exec"), (l.jog, "jog"), (l.stream, "stream")] {
            let Some(m) = block else { continue };
            let mf = |field: &str| f(&format!("limits.{name}.{field}"));
            if !(m.velocity_rad_s > 0.0 && m.velocity_rad_s <= l.velocity_rad_s) {
                return Err(invalid(
                    mf("velocity_rad_s"),
                    format!(
                        "must be in (0, {}] (the hardware ceiling)",
                        l.velocity_rad_s
                    ),
                ));
            }
            if !(m.acceleration_rad_s2 > 0.0 && m.acceleration_rad_s2 <= l.acceleration_rad_s2) {
                return Err(invalid(
                    mf("acceleration_rad_s2"),
                    format!(
                        "must be in (0, {}] (the hardware ceiling)",
                        l.acceleration_rad_s2
                    ),
                ));
            }
            if let Some(v) = m.jerk_rad_s3 {
                if !(v > 0.0 && v <= l.jerk_rad_s3) {
                    return Err(invalid(
                        mf("jerk_rad_s3"),
                        format!("must be in (0, {}] (the hardware ceiling)", l.jerk_rad_s3),
                    ));
                }
            }
            if let Some(v) = m.torque_rate_nm_s {
                if !(v > 0.0 && v <= l.torque_rate_nm_s) {
                    return Err(invalid(
                        mf("torque_rate_nm_s"),
                        format!(
                            "must be in (0, {}] (the hardware ceiling)",
                            l.torque_rate_nm_s
                        ),
                    ));
                }
            }
        }
        Ok(())
    }

    fn validate_bus(&self) -> Result<(), ConfigError> {
        let b = &self.bus;
        if b.bitrate == 0 {
            return Err(invalid("bus.bitrate", "must be > 0"));
        }
        for (v, name) in [
            (b.gripper_node, "bus.gripper_node"),
            (b.timing_dummy_node, "bus.timing_dummy_node"),
            (b.host_node, "bus.host_node"),
        ] {
            if v >= 16 {
                return Err(invalid(name, "11-bit CAN ids carry 4-bit node ids (0-15)"));
            }
        }
        if !is_positive(b.stale_warn_s) {
            return Err(invalid("bus.stale_warn_s", "must be > 0"));
        }
        if b.lost_s <= b.stale_warn_s {
            return Err(invalid(
                "bus.lost_s",
                "must be greater than bus.stale_warn_s",
            ));
        }
        if b.rx_frames_per_tick_cap == 0 {
            return Err(invalid("bus.rx_frames_per_tick_cap", "must be >= 1"));
        }
        if b.boot_config_repeats == 0 {
            return Err(invalid("bus.boot_config_repeats", "must be >= 1"));
        }
        if !(b.open_retry_s.is_finite() && b.open_retry_s >= 0.0) {
            return Err(invalid("bus.open_retry_s", "must be finite and >= 0"));
        }
        if b.config_pace_s < 0.0 {
            return Err(invalid("bus.config_pace_s", "must be >= 0"));
        }
        Ok(())
    }

    fn validate_protocol(&self) -> Result<(), ConfigError> {
        let p = &self.protocol;
        let ports = [
            (p.command_port, "protocol.command_port"),
            (p.status_port, "protocol.status_port"),
            (p.telemetry_port, "protocol.telemetry_port"),
        ];
        for (i, (port, name)) in ports.iter().enumerate() {
            if *port == 0 {
                return Err(invalid(*name, "must be nonzero"));
            }
            if ports[..i].iter().any(|(other, _)| other == port) {
                return Err(invalid(*name, "duplicate port"));
            }
        }
        let group: std::net::Ipv4Addr = p.status_multicast_group.parse().map_err(|_| {
            invalid(
                "protocol.status_multicast_group",
                "not a valid IPv4 address",
            )
        })?;
        if !group.is_multicast() {
            return Err(invalid(
                "protocol.status_multicast_group",
                "must be an IPv4 multicast address (224.0.0.0/4)",
            ));
        }
        if p.status_rate_hz == 0 {
            return Err(invalid("protocol.status_rate_hz", "must be > 0"));
        }
        if !(1..=1000).contains(&p.telemetry_rate_hz) {
            return Err(invalid("protocol.telemetry_rate_hz", "must be 1..=1000"));
        }
        let rate = self.tick_rate_hz();
        let per = rate / f64::from(p.status_rate_hz);
        if per < 1.0 || (per - per.round()).abs() > 1e-9 {
            return Err(invalid(
                "protocol.status_rate_hz",
                format!("must divide the tick rate ({rate} Hz) exactly"),
            ));
        }
        Ok(())
    }

    fn validate_defaults(&self) -> Result<(), ConfigError> {
        let j = &self.jog;
        if !(j.speed_pct > 0.0 && j.speed_pct <= 1.0) {
            return Err(invalid("jog.speed_pct", "must be in (0, 1]"));
        }
        if !is_positive(j.accel_time_s) {
            return Err(invalid("jog.accel_time_s", "must be > 0"));
        }
        if !is_positive(j.jerk_factor) {
            return Err(invalid("jog.jerk_factor", "must be > 0"));
        }
        let s = &self.stream;
        for (v, name) in [
            (s.command_timeout_s, "stream.command_timeout_s"),
            (s.success_window_s, "stream.success_window_s"),
            (s.fault_latch_s, "stream.fault_latch_s"),
        ] {
            if !is_positive(v) {
                return Err(invalid(name, "must be > 0"));
            }
        }
        if s.lowpass_cutoff_hz < 0.0 {
            return Err(invalid(
                "stream.lowpass_cutoff_hz",
                "must be >= 0 (0 = off)",
            ));
        }
        Ok(())
    }

    fn validate_sim(&self) -> Result<(), ConfigError> {
        let sim = &self.sim;
        if sim.motor_jm_kg_m2.len() != self.joints.len() {
            return Err(invalid(
                "sim.motor_jm_kg_m2",
                "must carry one entry per joint",
            ));
        }
        for (j, v) in sim.motor_jm_kg_m2.iter().enumerate() {
            if !(v.is_finite() && *v >= 0.0) {
                return Err(invalid(
                    "sim.motor_jm_kg_m2",
                    format!("entry {j} must be finite and >= 0"),
                ));
            }
        }
        for (v, name) in [
            (sim.motor_b_nm_s, "sim.motor_b_nm_s"),
            (sim.motor_tc_nm, "sim.motor_tc_nm"),
        ] {
            if !(v.is_finite() && v >= 0.0) {
                return Err(invalid(name, "must be finite and >= 0"));
            }
        }
        for (i, j) in self.joints.iter().enumerate() {
            if let Some(g) = j.dynamics_gear_ratio {
                if !is_positive(g) {
                    return Err(invalid(
                        "joints.dynamics_gear_ratio",
                        format!("joint {i}: must be > 0"),
                    ));
                }
            }
        }
        Ok(())
    }

    fn validate_timing(&self) -> Result<(), ConfigError> {
        let Some(t) = self.timing else {
            return Ok(());
        };
        // A band at or below the nominal period fires on a loop that is
        // meeting its deadline, so both factors must clear 1.0.
        for (v, name) in [
            (t.degraded_factor, "timing.degraded_factor"),
            (t.critical_factor, "timing.critical_factor"),
        ] {
            if !is_positive(v - 1.0) || !v.is_finite() {
                return Err(invalid(
                    name,
                    "must be > 1.0 (a multiple of the tick period)",
                ));
            }
        }
        if t.critical_factor < t.degraded_factor {
            return Err(invalid(
                "timing.critical_factor",
                "must be >= timing.degraded_factor",
            ));
        }
        if !is_positive(t.critical_sustain_s) || !t.critical_sustain_s.is_finite() {
            return Err(invalid("timing.critical_sustain_s", "must be > 0"));
        }
        if t.fifo_priority > 99 {
            return Err(invalid(
                "timing.fifo_priority",
                "must be 0..=99 (0 disables SCHED_FIFO)",
            ));
        }
        Ok(())
    }

    fn validate_shutdown(&self) -> Result<(), ConfigError> {
        let s = &self.shutdown;
        for (v, name) in [
            (s.tolerance_rad, "shutdown.tolerance_rad"),
            (s.timeout_s, "shutdown.timeout_s"),
            (s.velocity_limit_rad_s, "shutdown.velocity_limit_rad_s"),
        ] {
            if !is_positive(v) {
                return Err(invalid(name, "must be > 0"));
            }
        }
        let q = self.safe_park_q();
        if q.len() != self.joints.len() {
            return Err(invalid(
                "shutdown.safe_park_q",
                format!(
                    "must have one entry per joint ({} joints, {} entries)",
                    self.joints.len(),
                    q.len()
                ),
            ));
        }
        for (i, (v, j)) in q.iter().zip(&self.joints).enumerate() {
            if !v.is_finite() || *v < j.limits.soft_min_rad || *v > j.limits.soft_max_rad {
                return Err(invalid(
                    "shutdown.safe_park_q",
                    format!(
                        "joint {i}: {v} rad is outside the soft limits [{}, {}]",
                        j.limits.soft_min_rad, j.limits.soft_max_rad
                    ),
                ));
            }
        }
        Ok(())
    }

    fn validate_motion(&self) -> Result<(), ConfigError> {
        let m = &self.motion;
        for (v, name) in [
            (m.jog_l_linear_max_m_s, "motion.jog_l_linear_max_m_s"),
            (m.jog_l_angular_max_rad_s, "motion.jog_l_angular_max_rad_s"),
            (m.cart_step_m, "motion.cart_step_m"),
            (m.cart_step_rad, "motion.cart_step_rad"),
            (m.path_step_m, "motion.path_step_m"),
            (m.joint_step_rad.unwrap_or(0.01), "motion.joint_step_rad"),
            (
                m.move_l_max_joint_step_rad,
                "motion.move_l_max_joint_step_rad",
            ),
            (m.dls_lambda, "motion.dls_lambda"),
            (m.settle_tolerance_rad, "motion.settle_tolerance_rad"),
            (m.settle_timeout_s, "motion.settle_timeout_s"),
        ] {
            if !is_positive(v) || !v.is_finite() {
                return Err(invalid(name, "must be > 0 and finite"));
            }
        }
        Ok(())
    }

    fn validate_limits(&self) -> Result<(), ConfigError> {
        let l = &self.limits;
        if l.tau_ext_margin_nm < 0.0 || !l.tau_ext_margin_nm.is_finite() {
            return Err(invalid(
                "limits.tau_ext_margin_nm",
                "must be >= 0 (0 disables the envelope)",
            ));
        }
        if !is_positive(l.tau_ext_window_s) || !l.tau_ext_window_s.is_finite() {
            return Err(invalid("limits.tau_ext_window_s", "must be > 0"));
        }
        Ok(())
    }
}
