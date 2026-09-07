//! Per-joint bridge between the plant's ground truth and the wire.

use par6_config::JointConfig;

use crate::spectral::convert::JointConversion;

/// Tick↔radian conversion (the REAL spectral conversion state, sector
/// shift 0 — the sim's ground truth is unwrapped), the boot 14-bit wrap
/// offset for reported positions, the hard limits and the torque↔current
/// factor of one arm joint.
#[derive(Debug, Clone)]
pub(crate) struct JointMap {
    pub conv: JointConversion,
    /// Added to the true motor position before reporting: at boot the
    /// encoder is absolute only within one revolution, so the first
    /// reported position is the boot position wrapped to 14 bits and
    /// everything after accumulates from there.
    pub report_offset: f64,
    /// `torque_to_ma_factor` \[mA per Nm\], sign included.
    pub factor_ma_per_nm: f64,
    /// Hard limits \[rad\] (the scene's joint limits, the teleport clamp).
    pub hard_lo_rad: f64,
    pub hard_hi_rad: f64,
    pub gear_ratio: f64,
    /// Gear ratio the drivetrain reflections use (`dynamics_gear_ratio`,
    /// falling back to `gear_ratio` — the vendor's J1 tables disagree).
    pub dyn_gear: f64,
    pub encoder_max_counts: i32,
}

impl JointMap {
    /// Build from a joint's config entry with the sim's true boot pose
    /// `q0_rad` (joint frame).
    pub fn from_config(j: &JointConfig, q0_rad: f64) -> Self {
        let conv = JointConversion::from_config(j);
        let encoder_max_counts = 1i32 << j.encoder_bits;
        let true0 = conv.motor_ticks(q0_rad);
        let wrapped0 = true0.rem_euclid(encoder_max_counts);
        Self {
            conv,
            report_offset: f64::from(wrapped0 - true0),
            factor_ma_per_nm: crate::spectral::convert::torque_to_ma_factor(
                j.gear_ratio,
                j.gear_efficiency,
                j.kt_nm_a,
                j.dir,
            ),
            hard_lo_rad: j.limits.hard_min_rad,
            hard_hi_rad: j.limits.hard_max_rad,
            gear_ratio: j.gear_ratio,
            dyn_gear: j.dynamics_gear_ratio.unwrap_or(j.gear_ratio),
            encoder_max_counts,
        }
    }

    /// Re-base the reported-position wrap onto a new true pose
    /// (teleport): the next reported position reads exactly as a boot
    /// reading at `q0_rad` would, which is the contract the RT's
    /// re-reference call relies on.
    pub fn reseed(&mut self, q0_rad: f64) {
        let true0 = self.conv.motor_ticks(q0_rad);
        let wrapped0 = true0.rem_euclid(self.encoder_max_counts);
        self.report_offset = f64::from(wrapped0 - true0);
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
