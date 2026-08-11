//! Gripper (tool) schema: one TOML file per tool variant. A tool always
//! carries kinematics (it hangs off the arm's last link) and may carry a
//! CAN driver + homing when it is an actuated gripper.
//! PAR6 values transcribed from the vendor `grippers/*.xml`.

use serde::{Deserialize, Serialize};
use std::path::Path;

use crate::homing::JointHoming;
use crate::robot::{DriverType, Gains};
use crate::{invalid, read_to_string, ConfigError};

/// CAN driver parameters for an actuated gripper. Absent for passive
/// tools (Flange): no CAN node, no boot config, no homing.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GripperDriverConfig {
    /// Driver board firmware product — a flashing-time guard, not a
    /// protocol difference (SSG48 = spectral-bldc, MSG = stepfoc).
    pub driver_type: DriverType,
    /// Jaw stroke \[mm\].
    pub stroke_mm: f64,
    /// Torque constant \[Nm/A\].
    pub kt_nm_a: f64,
    /// Current limit \[mA\].
    pub ilim_ma: f64,
    /// Voltage limit \[mV\]; 0 = use VBUS.
    pub voltage_limit_mv: u32,
    /// Motor velocity limit \[ticks/s\].
    pub velocity_limit_ticks_s: f64,
    /// Driver watchdog timeout \[ms\].
    pub watchdog_timeout_ms: u32,
    /// Pinion pitch radius \[m\] (vendor `Gear_r`). Converts between
    /// linear jaw motion and motor rotation:
    /// `ticks_per_meter = 2^14 / (4π · gear_r_m)` (one pinion, two jaws).
    pub gear_r_m: f64,
    /// Controller gains pushed at boot.
    pub gains: Gains,
}

/// Home-offset override for one ARM joint, applied when that joint is
/// flagged `home_offset_gripper_dependent` and this gripper is active.
/// Swapping grippers changes the arm joint's home reference — this makes
/// the dependency explicit.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ArmJointHomeOffset {
    /// Arm joint index (0-based).
    pub joint: u8,
    /// Replacement home offset \[rad\].
    pub home_offset_rad: f64,
}

/// DH tool link + dynamics appended to the arm's kinematic chain while
/// this tool is fitted. Feeds gravity compensation and TCP kinematics.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ToolKinematics {
    /// DH link offset \[m\].
    pub d_m: f64,
    /// DH link length \[m\].
    pub a_m: f64,
    /// DH twist angle \[rad\].
    pub alpha_rad: f64,
    /// Total tool mass \[kg\].
    pub mass_kg: f64,
    /// Center of mass x/y/z \[m\].
    pub com_m: [f64; 3],
    /// Inertia Ixx Iyy Izz Ixy Iyz Ixz \[kg·m^2\].
    pub inertia_kg_m2: [f64; 6],
    /// Motor rotor inertia \[kg·m^2\] (vendor `Jm`).
    pub motor_inertia_kg_m2: f64,
    /// Tool-joint gear ratio (vendor `G`).
    pub gear_ratio: f64,
    /// Viscous friction \[N·m·s/rad\] (vendor `B`).
    pub viscous_friction: f64,
    /// Coulomb friction \[+, −\] \[N·m\] (vendor `Tc`).
    pub coulomb_friction_nm: [f64; 2],
    /// Tool-joint limits \[deg\] (vendor `qlim_deg`, transcribed verbatim).
    pub qlim_deg: [f64; 2],
}

/// Root of a gripper TOML file.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GripperConfig {
    /// Tool name — what `robot.active_gripper` selects.
    pub name: String,
    /// CAN driver parameters; absent = passive tool (vendor
    /// `CAN_gripper = 0`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub driver: Option<GripperDriverConfig>,
    /// Motor homing parameters for the gripper's own actuator; absent for
    /// passive tools. `home_offset_gripper_dependent` is meaningless here
    /// and must stay false.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub homing: Option<JointHoming>,
    /// Home-offset overrides for gripper-dependent ARM joints.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub arm_joint_home_offsets: Vec<ArmJointHomeOffset>,
    /// Tool link kinematics/dynamics.
    pub kinematics: ToolKinematics,
}

impl GripperConfig {
    /// Parse and validate a gripper config from a TOML string.
    pub fn from_toml_str(text: &str) -> Result<Self, ConfigError> {
        let cfg: Self = toml::from_str(text).map_err(|source| ConfigError::Parse {
            path: "<string>".into(),
            source: Box::new(source),
        })?;
        cfg.validate()?;
        Ok(cfg)
    }

    /// Load and validate a gripper config file.
    pub fn load(path: &Path) -> Result<Self, ConfigError> {
        let text = read_to_string(path)?;
        let cfg: Self = toml::from_str(&text).map_err(|source| ConfigError::Parse {
            path: path.display().to_string(),
            source: Box::new(source),
        })?;
        cfg.validate()?;
        Ok(cfg)
    }

    /// Validate this gripper alone; every error names its field.
    pub fn validate(&self) -> Result<(), ConfigError> {
        if self.name.is_empty() {
            return Err(invalid("name", "must not be empty"));
        }
        if let Some(d) = &self.driver {
            for (v, name) in [
                (d.stroke_mm, "driver.stroke_mm"),
                (d.kt_nm_a, "driver.kt_nm_a"),
                (d.ilim_ma, "driver.ilim_ma"),
                (d.velocity_limit_ticks_s, "driver.velocity_limit_ticks_s"),
                (d.gear_r_m, "driver.gear_r_m"),
            ] {
                if v <= 0.0 {
                    return Err(invalid(name, "must be > 0"));
                }
            }
            if d.watchdog_timeout_ms == 0 {
                return Err(invalid("driver.watchdog_timeout_ms", "must be > 0"));
            }
        }
        if self.homing.is_some() && self.driver.is_none() {
            return Err(invalid(
                "homing",
                "a passive tool (no [driver]) cannot home",
            ));
        }
        if let Some(h) = &self.homing {
            h.validate("homing")?;
            if h.home_offset_gripper_dependent {
                return Err(invalid(
                    "homing.home_offset_gripper_dependent",
                    "gripper-dependent overrides apply to ARM joints, not the gripper itself",
                ));
            }
        }
        for (i, o) in self.arm_joint_home_offsets.iter().enumerate() {
            if o.joint >= 16 {
                return Err(invalid(
                    format!("arm_joint_home_offsets[{i}].joint"),
                    "not a plausible arm joint index",
                ));
            }
        }
        if self.kinematics.mass_kg < 0.0 {
            return Err(invalid("kinematics.mass_kg", "must be >= 0"));
        }
        if self.kinematics.gear_ratio <= 0.0 {
            return Err(invalid("kinematics.gear_ratio", "must be > 0"));
        }
        Ok(())
    }
}
