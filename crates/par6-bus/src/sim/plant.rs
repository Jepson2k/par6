//! Default plant: a per-joint 1-DOF rate-limited model in motor-tick
//! space, grounded in config values (the hardware acceleration ceiling and
//! the boot Ilim size the current→acceleration gain). It honors every
//! driver mode through the loop current and produces the endstop
//! signatures homing detection needs: displacement plateau, loop-current
//! rise to the saturated output, gearbox-windup preload that relaxes
//! during the release phase.

use par6_config::JointConfig;

use crate::spectral::convert::{ticks_per_radian, JointConversion};

use super::driver::PlantCmd;

/// Plant viscous drag \[1/s\]: `accel -= VISC · vel`. Sized so the cascade
/// velocity loop is well damped at 250 Hz while the drag current during a
/// homing approach stays well under the stall-current threshold.
const VISC: f64 = 8.0;
/// Seating threshold \[mA\]: leaving an endstop needs more away-drive than
/// this. Emulates the gravity/preload seating force that lets the release
/// phase (e.g. +150 mA) relax the gearbox without detaching the joint,
/// while homing backoff (≥250 mA homing currents) detaches normally.
const DETACH_MA: f64 = 200.0;
/// Gearbox windup \[ticks per mA of into-stop current\].
const WINDUP_TICKS_PER_MA: f64 = 0.5;
/// Windup ceiling \[ticks\].
const WINDUP_MAX_TICKS: f64 = 400.0;
/// Windup first-order time constant \[s\].
const WINDUP_TAU_S: f64 = 0.1;
/// Idle (watchdog fired / cmd 12) velocity decay time constant \[s\] —
/// the shorted-phase-brake behavior of an idled driver.
const IDLE_TAU_S: f64 = 0.05;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Seat {
    Free,
    Lo,
    Hi,
}

/// One 1-DOF joint in motor-tick space.
#[derive(Debug, Clone)]
pub(crate) struct KinJoint {
    /// True motor position \[ticks\] (may sit past a bound by the windup).
    pub pos: f64,
    /// Integrated velocity state \[ticks/s\].
    vel: f64,
    /// Velocity the encoder reports (creep-rate while seated) \[ticks/s\].
    pub reported_vel: f64,
    /// Current→acceleration gain \[ticks/s² per mA\].
    k_a: f64,
    /// Acceleration ceiling \[ticks/s²\] (config hardware limit).
    accel_max: f64,
    /// Mechanical endstops \[ticks\].
    lo: f64,
    hi: f64,
    seat: Seat,
    /// Windup magnitude past the seated bound \[ticks\], always ≥ 0.
    excess: f64,
    windup_keep: f64,
    idle_keep: f64,
}

impl KinJoint {
    pub fn new(dt: f64, pos0: f64, lo: f64, hi: f64, accel_max: f64, ilim_boot_ma: f64) -> Self {
        Self {
            pos: pos0,
            vel: 0.0,
            reported_vel: 0.0,
            k_a: accel_max / ilim_boot_ma,
            accel_max,
            lo,
            hi,
            seat: Seat::Free,
            excess: 0.0,
            windup_keep: (-dt / WINDUP_TAU_S).exp(),
            idle_keep: (-dt / IDLE_TAU_S).exp(),
        }
    }

    /// Integrate one fixed step. `load_ma` is a constant external load in
    /// motor-current equivalent (positive opposes positive motion).
    pub fn step(&mut self, dt: f64, cmd: &PlantCmd, load_ma: f64) {
        let net = cmd.current_ma - load_ma;
        if cmd.idle {
            self.vel *= self.idle_keep;
        }
        let acc = (self.k_a * net - VISC * self.vel).clamp(-self.accel_max, self.accel_max);
        let vlim = cmd.vel_limit_ticks_s.abs();
        self.vel = (self.vel + acc * dt).clamp(-vlim, vlim);
        let prev = self.pos;
        match self.seat {
            Seat::Free => {
                let tentative = self.pos + self.vel * dt;
                if tentative >= self.hi && self.vel > 0.0 {
                    self.seat = Seat::Hi;
                    self.excess = 0.0;
                    self.pos = self.hi;
                    self.vel = 0.0;
                } else if tentative <= self.lo && self.vel < 0.0 {
                    self.seat = Seat::Lo;
                    self.excess = 0.0;
                    self.pos = self.lo;
                    self.vel = 0.0;
                } else {
                    self.pos = tentative;
                }
            }
            Seat::Hi => self.seated_step(net, self.hi, 1.0),
            Seat::Lo => self.seated_step(net, self.lo, -1.0),
        }
        self.reported_vel = if self.seat == Seat::Free {
            self.vel
        } else {
            (self.pos - prev) / dt
        };
    }

    /// Seated at `bound`; `into_sign` is the drive sign that presses into
    /// the stop. Windup follows the into-current first-order; enough
    /// away-current breaks the seat.
    fn seated_step(&mut self, net_ma: f64, bound: f64, into_sign: f64) {
        let into = (net_ma * into_sign).max(0.0);
        let away = (-net_ma * into_sign).max(0.0);
        if away > DETACH_MA {
            self.seat = Seat::Free;
            self.vel = 0.0;
            return;
        }
        let target = (WINDUP_TICKS_PER_MA * into).min(WINDUP_MAX_TICKS);
        self.excess = target + (self.excess - target) * self.windup_keep;
        self.pos = bound + into_sign * self.excess;
        self.vel = 0.0;
    }
}

/// Per-joint bridge between the plant's ground truth and the wire:
/// tick↔radian conversion (the REAL spectral conversion state, sector
/// shift 0 — the sim's ground truth is unwrapped), the boot 14-bit wrap
/// offset for reported positions, endstop bounds and the torque↔current
/// factor.
#[derive(Debug, Clone)]
pub(crate) struct JointMap {
    pub conv: JointConversion,
    /// Added to the true motor position before reporting: at boot the
    /// encoder is absolute only within one revolution, so the first
    /// reported position is the boot position wrapped to 14 bits and
    /// everything after accumulates from there.
    pub report_offset: f64,
    /// Motor-tick endstop bounds (hard limits mapped through `conv`).
    pub bound_lo: f64,
    pub bound_hi: f64,
    /// `torque_to_ma_factor` \[mA per Nm\], sign included (dynamics
    /// plant: current→torque).
    #[cfg_attr(not(feature = "sim-dynamics"), allow(dead_code))]
    pub factor_ma_per_nm: f64,
    /// Hard limits \[rad\] (dynamics-plant endstops).
    #[cfg_attr(not(feature = "sim-dynamics"), allow(dead_code))]
    pub hard_lo_rad: f64,
    #[cfg_attr(not(feature = "sim-dynamics"), allow(dead_code))]
    pub hard_hi_rad: f64,
    /// Motor ticks per joint radian (unsigned magnitude).
    pub tpr: f64,
    #[cfg_attr(not(feature = "sim-dynamics"), allow(dead_code))]
    pub gear_ratio: f64,
    #[cfg_attr(not(feature = "sim-dynamics"), allow(dead_code))]
    pub encoder_max_counts: i32,
}

impl JointMap {
    /// Build from a joint's config entry with the sim's true boot pose
    /// `q0_rad` (joint frame).
    pub fn from_config(j: &JointConfig, q0_rad: f64) -> Self {
        let conv = JointConversion::from_config(j);
        let a = f64::from(conv.motor_ticks(j.limits.hard_min_rad));
        let b = f64::from(conv.motor_ticks(j.limits.hard_max_rad));
        let encoder_max_counts = 1i32 << j.encoder_bits;
        let true0 = conv.motor_ticks(q0_rad);
        let wrapped0 = true0.rem_euclid(encoder_max_counts);
        Self {
            conv,
            report_offset: f64::from(wrapped0 - true0),
            bound_lo: a.min(b),
            bound_hi: a.max(b),
            factor_ma_per_nm: crate::spectral::convert::torque_to_ma_factor(
                j.gear_ratio,
                j.gear_efficiency,
                j.kt_nm_a,
                j.dir,
            ),
            hard_lo_rad: j.limits.hard_min_rad,
            hard_hi_rad: j.limits.hard_max_rad,
            tpr: ticks_per_radian(encoder_max_counts, j.gear_ratio),
            gear_ratio: j.gear_ratio,
            encoder_max_counts,
        }
    }

    /// True motor position → reported encoder position \[ticks\].
    pub fn report_pos(&self, true_pos_ticks: f64) -> i32 {
        (true_pos_ticks + self.report_offset).round() as i32
    }

    /// Joint angle of a true motor position (band checks, hall logic).
    pub fn joint_rad(&self, true_pos_ticks: f64) -> f64 {
        self.conv.joint_rad(true_pos_ticks.round() as i32)
    }
}
