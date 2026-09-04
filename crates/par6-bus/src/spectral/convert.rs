//! Wire↔SI unit conversions — the vendor `SourceRoboticsToolbox.Joint`
//! semantics, as pure functions plus a per-joint [`JointConversion`]
//! carrying the calibration state (master position, offset, direction,
//! sector wrap correction).
//!
//! Encoder positions on the wire are motor-shaft ticks (14-bit absolute
//! encoder, accumulated by the driver). At boot the reading is only
//! absolute within one motor revolution, so
//! [`JointConversion::determine_sector`] picks the ±one-revolution wrap
//! correction that places the boot pose nearest the master position.

use par6_config::JointConfig;

const TWO_PI: f64 = core::f64::consts::TAU;

/// Truncate toward zero at the encode boundary — vendor `int()`
/// semantics, NOT rounding and NOT floor (−1045.58 → −1045). Saturates at
/// the i32 bounds; narrow further with `as` where a field is i16 (wraps,
/// like the vendor's `& 0xFFFF`).
pub fn trunc_to_wire(v: f64) -> i32 {
    v.trunc() as i32
}

/// Motor ticks per joint radian: `(encoder_max_counts * gear_ratio) / 2π`.
pub fn ticks_per_radian(encoder_max_counts: i32, gear_ratio: f64) -> f64 {
    f64::from(encoder_max_counts) * gear_ratio / TWO_PI
}

/// Joint radians → motor ticks, rounded half-to-even (Python `round`,
/// which the vendor calibration math uses).
pub fn radians_to_ticks(radians: f64, encoder_max_counts: i32, gear_ratio: f64) -> i32 {
    (radians * ticks_per_radian(encoder_max_counts, gear_ratio)).round_ties_even() as i32
}

/// Motor ticks → joint radians.
pub fn ticks_to_radians(ticks: f64, encoder_max_counts: i32, gear_ratio: f64) -> f64 {
    ticks / ticks_per_radian(encoder_max_counts, gear_ratio)
}

/// Joint-torque→motor-current factor \[mA per Nm\]:
/// `sign * 1000 / (gear_ratio * gear_efficiency * kt)`, `sign = 1 − 2·dir`.
///
/// Usage: `current_ma = trunc_to_wire(torque_nm * factor)`; the inverse
/// (telemetry mA → joint Nm) is `ma / factor` (the sign is ±1, so the
/// factor is its own sign-inverse).
pub fn torque_to_ma_factor(gear_ratio: f64, gear_efficiency: f64, kt_nm_a: f64, dir: u8) -> f64 {
    let sign = f64::from(1 - 2 * i32::from(dir == 1));
    sign * 1000.0 / (gear_ratio * gear_efficiency * kt_nm_a)
}

/// Per-joint calibration state for tick↔radian conversion.
///
/// Mirrors the vendor `Joint`: `joint_ticks = motor − master + shift +
/// offset_ticks`, radians via [`ticks_to_radians`], mirrored (`2π − x`)
/// when `dir == 1`. `shift` is the boot sector wrap correction
/// (0 or ±one motor revolution).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointConversion {
    encoder_max_counts: i32,
    gear_ratio: f64,
    dir: u8,
    master_position_ticks: i32,
    offset_ticks: i32,
    sector_shift_ticks: i32,
}

impl JointConversion {
    /// Build from raw calibration values. `offset_rad` is the joint angle
    /// the master position maps to (vendor `sector_home_offset`); the
    /// sector shift starts at 0 ("middle") until
    /// [`determine_sector`](Self::determine_sector) runs at boot.
    pub fn new(
        encoder_bits: u8,
        gear_ratio: f64,
        dir: u8,
        master_position_ticks: i32,
        offset_rad: f64,
    ) -> Self {
        let encoder_max_counts = 1i32 << encoder_bits;
        let mut s = Self {
            encoder_max_counts,
            gear_ratio,
            dir,
            master_position_ticks,
            offset_ticks: 0,
            sector_shift_ticks: 0,
        };
        s.offset_ticks = s.offset_ticks_for(offset_rad);
        s
    }

    /// Build from a joint's config entry (boot calibration values).
    pub fn from_config(j: &JointConfig) -> Self {
        Self::new(
            j.encoder_bits,
            j.gear_ratio,
            j.dir,
            j.sector_master_position_ticks,
            j.sector_home_offset_rad,
        )
    }

    /// Vendor rule: the stored tick offset is mirrored for inverted
    /// joints (`dir == 1` stores `2π − offset`).
    fn offset_ticks_for(&self, offset_rad: f64) -> i32 {
        let rad = if self.dir == 0 {
            offset_rad
        } else {
            TWO_PI - offset_rad
        };
        radians_to_ticks(rad, self.encoder_max_counts, self.gear_ratio)
    }

    /// Boot-time sector selection from the first (wrapped) encoder
    /// reading: picks the ±one-revolution correction that places the boot
    /// pose nearest the master position. Call once at boot, before any
    /// position conversion (vendor `Joint.determine_sector`).
    ///
    /// Boundary readings exactly half a revolution from the master (which
    /// the vendor leaves undefined) resolve to the uncorrected sector.
    pub fn determine_sector(&mut self, initial_motor_ticks: i32) {
        let max = self.encoder_max_counts;
        let resolution = max - 1;
        let midpoint = resolution / 2; // max/2 − 1
        let initial = initial_motor_ticks.rem_euclid(max);
        let master = self.master_position_ticks;
        // Vendor discriminant `master + midpoint − resolution` = master − max/2.
        self.sector_shift_ticks = if master - max / 2 > 0 {
            // "left": master in the upper half — readings that wrapped past
            // the encoder top sit low and need +one revolution.
            if initial < master - max / 2 {
                max
            } else {
                0
            }
        } else if master - max / 2 < 0 {
            // "right": master in the lower half — readings that wrapped
            // below zero sit high and need −one revolution.
            if initial > master + midpoint {
                -max
            } else {
                0
            }
        } else {
            0 // "middle"
        };
    }

    /// Home reference update: after homing latches the endstop tick,
    /// re-base so `joint_rad(latched_ticks) == home_offset_rad`.
    ///
    /// The sector shift is cleared: the latched tick is a live
    /// accumulated position (not a wrapped boot reading), so no wrap
    /// correction applies. (The vendor keeps the stale boot shift, which
    /// violates the spec post-condition whenever it was nonzero; the spec
    /// is the contract here.)
    pub fn set_home(&mut self, latched_motor_ticks: i32, home_offset_rad: f64) {
        self.master_position_ticks = latched_motor_ticks;
        self.offset_ticks = self.offset_ticks_for(home_offset_rad);
        self.sector_shift_ticks = 0;
    }

    /// Motor position → joint angle \[rad\].
    pub fn joint_rad(&self, motor_ticks: i32) -> f64 {
        let joint_ticks =
            motor_ticks - self.master_position_ticks + self.sector_shift_ticks + self.offset_ticks;
        let rad = ticks_to_radians(
            f64::from(joint_ticks),
            self.encoder_max_counts,
            self.gear_ratio,
        );
        if self.dir == 1 {
            TWO_PI - rad
        } else {
            rad
        }
    }

    /// Joint angle \[rad\] → motor position ticks (inverse of
    /// [`joint_rad`](Self::joint_rad)).
    pub fn motor_ticks(&self, joint_rad: f64) -> i32 {
        let rad = if self.dir == 1 {
            TWO_PI - joint_rad
        } else {
            joint_rad
        };
        radians_to_ticks(rad, self.encoder_max_counts, self.gear_ratio) + self.master_position_ticks
            - self.offset_ticks
            - self.sector_shift_ticks
    }

    /// Motor speed \[ticks/s\] → joint speed \[rad/s\]
    /// (`dir == 1` flips the sign — the spec formula's sign is implicit
    /// in the vendor `get_joint_speed`).
    pub fn joint_speed_rad_s(&self, motor_ticks_s: f64) -> f64 {
        let signed = if self.dir == 1 {
            -motor_ticks_s
        } else {
            motor_ticks_s
        };
        signed * (TWO_PI / f64::from(self.encoder_max_counts)) / self.gear_ratio
    }

    /// Joint speed \[rad/s\] → motor speed \[ticks/s\]. Truncate with
    /// [`trunc_to_wire`] at the encode boundary.
    pub fn motor_speed_ticks_s(&self, joint_rad_s: f64) -> f64 {
        let signed = if self.dir == 1 {
            -joint_rad_s
        } else {
            joint_rad_s
        };
        signed * self.gear_ratio / (TWO_PI / f64::from(self.encoder_max_counts))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use par6_config::RobotConfig;
    use std::path::PathBuf;

    fn par6() -> RobotConfig {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
        RobotConfig::load(&path).expect("PAR6.toml")
    }

    #[test]
    fn truncation_is_toward_zero_not_floor_or_round() {
        assert_eq!(trunc_to_wire(3.9), 3);
        assert_eq!(trunc_to_wire(-3.9), -3, "floor would give -4");
        assert_eq!(trunc_to_wire(-0.99), 0);
        assert_eq!(trunc_to_wire(1045.999), 1045, "round would give 1046");
        assert_eq!(trunc_to_wire(-1045.58), -1045);
    }

    /// Spec invariant behind sector selection: the encoder is absolute
    /// only within one motor revolution, so the corrected boot position
    /// must be the representative of `initial − master (mod rev)` nearest
    /// zero — within ±half a motor revolution of the master.
    #[test]
    fn determine_sector_maps_boot_reading_to_nearest_wrap() {
        let max = 16384i32;
        for master in [0, 400, 3969, 8191, 8192, 8193, 12000, 16000, 16383] {
            for initial in [0, 1, 400, 3969, 8191, 8192, 12161, 16000, 16383] {
                // offset 0, dir 0: joint_rad * tpr is exactly the corrected
                // delta from the master.
                let mut c = JointConversion::new(14, 1.0, 0, master, 0.0);
                c.determine_sector(initial);
                let delta =
                    (c.joint_rad(initial) * ticks_per_radian(max, 1.0)).round_ties_even() as i32;
                assert_eq!(
                    delta.rem_euclid(max),
                    (initial - master).rem_euclid(max),
                    "master={master} initial={initial}: wrap must preserve the reading"
                );
                assert!(
                    (-max / 2 - 1..=max / 2).contains(&delta),
                    "master={master} initial={initial}: delta {delta} not within half a revolution"
                );
            }
        }
    }

    /// Spec post-condition of the boot calibration: at the master
    /// position the joint reads the configured home offset (dir mirroring
    /// included) — checked for every real PAR6 joint.
    #[test]
    fn par6_boot_calibration_reads_home_offset_at_master() {
        let robot = par6();
        for j in &robot.joints {
            let mut c = JointConversion::from_config(j);
            c.determine_sector(j.sector_master_position_ticks);
            let got = c.joint_rad(j.sector_master_position_ticks);
            let tick_rad = 1.0 / ticks_per_radian(1 << j.encoder_bits, j.gear_ratio);
            assert!(
                (got - j.sector_home_offset_rad).abs() <= tick_rad,
                "{}: joint_rad(master) = {got}, want {} (±{tick_rad})",
                j.name,
                j.sector_home_offset_rad
            );
        }
    }

    #[test]
    fn position_roundtrip_and_dir_mirroring() {
        let robot = par6();
        for j in &robot.joints {
            let mut c = JointConversion::from_config(j);
            // Boot from a wrapped reading so a nonzero shift is exercised
            // on joints whose master sits near the wrap boundary.
            c.determine_sector(j.sector_master_position_ticks + 100);
            for delta in [-40000, -1000, -1, 0, 1, 999, 40000] {
                let motor = j.sector_master_position_ticks + delta;
                let rad = c.joint_rad(motor);
                let back = c.motor_ticks(rad);
                assert!(
                    (back - motor).abs() <= 1,
                    "{}: motor {motor} -> {rad} rad -> {back}",
                    j.name
                );
            }
            // dir mirroring: a positive motor move decreases the joint
            // angle exactly when dir == 1.
            let a = c.joint_rad(j.sector_master_position_ticks);
            let b = c.joint_rad(j.sector_master_position_ticks + 500);
            if j.dir == 1 {
                assert!(b < a, "{}: dir=1 must mirror", j.name);
            } else {
                assert!(b > a, "{}: dir=0 must not mirror", j.name);
            }
        }
    }

    #[test]
    fn speed_conversion_matches_hand_derived_value_and_roundtrips() {
        let robot = par6();
        // Hand derivation from the spec formula, J2 (gear 25, dir 0), at
        // the configured 80000 ticks/s limit:
        // 80000 · (2π/16384) / 25 = 1.227184630308513 rad/s.
        let j2 = JointConversion::from_config(&robot.joints[1]);
        let v = j2.joint_speed_rad_s(80000.0);
        assert!((v - 1.227184630308513).abs() < 1e-12, "got {v}");
        // dir == 1 flips the sign (J3).
        let j3 = JointConversion::from_config(&robot.joints[2]);
        assert!(j3.joint_speed_rad_s(80000.0) < 0.0);
        // Roundtrip both directions for every joint.
        for j in &robot.joints {
            let c = JointConversion::from_config(j);
            for ticks in [-80000.0, -1.0, 0.0, 12345.0, 80000.0] {
                let back = c.motor_speed_ticks_s(c.joint_speed_rad_s(ticks));
                assert!((back - ticks).abs() < 1e-9, "{}: {ticks} -> {back}", j.name);
            }
        }
    }

    #[test]
    fn torque_factor_signs_scale_and_truncation() {
        let robot = par6();
        // J6: gear 10, eff 0.95, kt 0.151, dir 1 →
        // factor = −1000/(10·0.95·0.151) = −1000/1.4345 = −697.1070…
        let j6 = &robot.joints[5];
        let f = torque_to_ma_factor(j6.gear_ratio, j6.gear_efficiency, j6.kt_nm_a, j6.dir);
        assert!((f - -697.1070).abs() < 1e-3, "got {f}");
        // 1.5 Nm → −1045.66… mA, truncated TOWARD ZERO → −1045 (floor
        // would give −1046; this is the vendor int() behavior).
        assert_eq!(trunc_to_wire(1.5 * f), -1045);
        // dir = 0 keeps the sign (J1), and the factor inverts cleanly.
        let j1 = &robot.joints[0];
        let f1 = torque_to_ma_factor(j1.gear_ratio, j1.gear_efficiency, j1.kt_nm_a, j1.dir);
        assert!(f1 > 0.0);
    }

    #[test]
    fn set_home_rebases_reference_and_clears_wrap_shift() {
        let robot = par6();
        let j = &robot.joints[0]; // J1: dir 0, gear 6.4
        let mut c = JointConversion::from_config(j);
        // Boot from a reading that forces a nonzero sector shift:
        // master 3969 ("right"), wrapped reading above master + 8191.
        c.determine_sector(13000);
        // Homing latches an accumulated (multi-revolution) endstop tick.
        let latched = 42_000;
        let home_offset = robot.homing.joints[0].home_offset_rad;
        c.set_home(latched, home_offset);
        let tick_rad = 1.0 / ticks_per_radian(1 << j.encoder_bits, j.gear_ratio);
        let at_endstop = c.joint_rad(latched);
        assert!(
            (at_endstop - home_offset).abs() <= tick_rad,
            "joint_rad(latched) = {at_endstop}, want {home_offset}"
        );
        // Positions remain continuous around the new reference.
        let step = c.joint_rad(latched + 1000) - at_endstop;
        let want = ticks_to_radians(1000.0, 1 << j.encoder_bits, j.gear_ratio);
        assert!((step - want).abs() < 1e-12);
    }
}
