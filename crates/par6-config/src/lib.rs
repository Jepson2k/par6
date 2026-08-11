//! Robot, gripper, and homing configuration.
//!
//! TOML schema covering: joint limits (soft/hard, per-mode kinodynamic),
//! driver gains (KPP/KPV/KIV/KPIQ/KIIQ/KP/KD), current/velocity/voltage
//! limits, kt + gear ratios + directions, encoder geometry, homing
//! parameters (per-joint FSM settings, sequence steps, gripper-dependent
//! home offsets), bus node map, tick rate. Values for PAR6 are transcribed
//! from the vendor XML (see `spec/RT.md` and `spec/HOMING.md`).
//!
//! All time constants are seconds in config, converted with `round(s / dt)`
//! at construction — never hardcoded tick counts. Use
//! [`RobotConfig::ticks`] for the conversion.
//!
//! Layout on disk (repo `config/` directory):
//!
//! ```text
//! config/PAR6.toml            robot + homing + bus + protocol
//! config/grippers/*.toml      one file per tool (MSG…, SSG48, Flange, …)
//! ```
//!
//! Load a robot alone with [`RobotConfig::load`], a single gripper with
//! [`GripperConfig::load`], or everything (robot + every gripper next to
//! it, cross-validated) with [`ConfigBundle::load`].

mod gripper;
mod homing;
mod robot;

pub use gripper::{ArmJointHomeOffset, GripperConfig, GripperDriverConfig, ToolKinematics};
pub use homing::{
    GripperHomeMode, HomeGroup, HomingConfig, HomingStrategy, JointHoming, MoveTo, PostHomeConfig,
    PreMove, ReleaseConfig, SequenceStep,
};
pub use robot::{
    BusConfig, ControlMode, DriverType, Gains, JogDefaults, JogProfile, JointConfig, JointLimits,
    KtFetchConfig, KtSource, LimitMode, ModeLimits, ProtocolConfig, ResolvedLimits, RobotConfig,
    RobotSection, ScanConfig, StreamDefaults, WatchdogAction,
};

use std::path::Path;

/// Error produced by loading or validating configuration.
///
/// Every validation failure names the offending field with its full TOML
/// path (e.g. `joints[2].limits.soft_min_rad`) so a bad config is fixable
/// without reading loader source.
#[derive(Debug, thiserror::Error)]
pub enum ConfigError {
    /// The file could not be read.
    #[error("failed to read {path}: {source}")]
    Io {
        /// Path that failed to read.
        path: String,
        /// Underlying I/O error.
        #[source]
        source: std::io::Error,
    },
    /// The file is not valid TOML for the schema.
    #[error("failed to parse {path}: {source}")]
    Parse {
        /// Path (or `<string>`) that failed to parse.
        path: String,
        /// Underlying TOML error (includes line/column and field names).
        #[source]
        source: Box<toml::de::Error>,
    },
    /// A field parsed but holds a value the contract forbids.
    #[error("invalid value for `{field}`: {reason}")]
    Invalid {
        /// Full TOML path of the offending field.
        field: String,
        /// Human-readable constraint that was violated.
        reason: String,
    },
}

pub(crate) fn invalid(field: impl Into<String>, reason: impl Into<String>) -> ConfigError {
    ConfigError::Invalid {
        field: field.into(),
        reason: reason.into(),
    }
}

pub(crate) fn read_to_string(path: &Path) -> Result<String, ConfigError> {
    std::fs::read_to_string(path).map_err(|source| ConfigError::Io {
        path: path.display().to_string(),
        source,
    })
}

/// A robot plus every gripper config found beside it, cross-validated.
#[derive(Debug, Clone, PartialEq)]
pub struct ConfigBundle {
    /// The robot configuration.
    pub robot: RobotConfig,
    /// All gripper configurations from `<robot dir>/grippers/*.toml`,
    /// sorted by file name.
    pub grippers: Vec<GripperConfig>,
}

impl ConfigBundle {
    /// Load `robot_toml` plus every `grippers/*.toml` in the same
    /// directory, then cross-validate (active gripper exists; gripper
    /// homing steps in the sequence have a CAN gripper to run on).
    pub fn load(robot_toml: &Path) -> Result<Self, ConfigError> {
        let robot = RobotConfig::load(robot_toml)?;
        let dir = robot_toml
            .parent()
            .map(|p| p.join("grippers"))
            .unwrap_or_else(|| Path::new("grippers").to_path_buf());
        let mut paths: Vec<_> = std::fs::read_dir(&dir)
            .map_err(|source| ConfigError::Io {
                path: dir.display().to_string(),
                source,
            })?
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.extension().is_some_and(|e| e == "toml"))
            .collect();
        paths.sort();
        let grippers = paths
            .iter()
            .map(|p| GripperConfig::load(p))
            .collect::<Result<Vec<_>, _>>()?;
        let bundle = Self { robot, grippers };
        bundle.validate()?;
        Ok(bundle)
    }

    /// The gripper selected by `robot.active_gripper`.
    pub fn active_gripper(&self) -> Option<&GripperConfig> {
        self.grippers
            .iter()
            .find(|g| g.name == self.robot.robot.active_gripper)
    }

    /// Effective home offset for an arm joint under the ACTIVE gripper:
    /// the gripper's `arm_joint_home_offsets` override when the joint is
    /// flagged `home_offset_gripper_dependent` and the gripper provides
    /// one, else the joint's own `home_offset_rad` fallback.
    /// `None` when `joint` is out of range.
    pub fn effective_home_offset(&self, joint: usize) -> Option<f64> {
        let jh = self.robot.homing.joints.get(joint)?;
        if jh.home_offset_gripper_dependent {
            if let Some(g) = self.active_gripper() {
                if let Some(o) = g
                    .arm_joint_home_offsets
                    .iter()
                    .find(|o| usize::from(o.joint) == joint)
                {
                    return Some(o.home_offset_rad);
                }
            }
        }
        Some(jh.home_offset_rad)
    }

    fn validate(&self) -> Result<(), ConfigError> {
        let Some(active) = self.active_gripper() else {
            return Err(invalid(
                "robot.active_gripper",
                format!(
                    "no gripper named `{}` found in grippers/ directory",
                    self.robot.robot.active_gripper
                ),
            ));
        };
        let sequence_homes_gripper = self
            .robot
            .homing
            .sequence
            .iter()
            .any(|s| s.home.as_ref().is_some_and(|h| h.gripper.is_some()));
        if sequence_homes_gripper && active.driver.is_none() {
            return Err(invalid(
                "homing.sequence",
                format!(
                    "sequence homes the gripper but active gripper `{}` has no [driver] section",
                    active.name
                ),
            ));
        }
        if sequence_homes_gripper && active.homing.is_none() {
            return Err(invalid(
                "homing.sequence",
                format!(
                    "sequence homes the gripper but active gripper `{}` has no [homing] section",
                    active.name
                ),
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn config_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config")
    }

    #[test]
    fn par6_toml_loads_and_roundtrips() {
        let path = config_dir().join("PAR6.toml");
        let cfg = RobotConfig::load(&path).expect("PAR6.toml must load");

        // Spot-check transcribed vendor values (robots/PAR6.xml).
        assert_eq!(cfg.robot.name, "PAR6");
        assert_eq!(cfg.robot.tick_dt_s, 0.004);
        assert_eq!(cfg.tick_rate_hz(), 250.0);
        assert_eq!(cfg.joints.len(), 6);
        let ratios: Vec<f64> = cfg.joints.iter().map(|j| j.gear_ratio).collect();
        assert_eq!(ratios, vec![6.4, 25.0, 18.0952381, 4.0, 4.0, 10.0]);
        assert_eq!(cfg.joints[0].kt_nm_a, 0.28);
        assert_eq!(cfg.joints[1].ilim_ma, 2500.0);
        assert_eq!(cfg.joints[1].gains.kpp, 3.0);
        assert_eq!(cfg.joints[2].dir, 1);
        assert_eq!(cfg.joints[5].limits.soft_max_rad, 7.14);
        // Per-mode limits: exec is the pre-liberal set, stream the ceiling.
        let exec = cfg.joints[0].limits.for_mode(LimitMode::Exec);
        assert_eq!(exec.acceleration_rad_s2, 9.6);
        assert_eq!(exec.jerk_rad_s3, Some(28.8));
        let jog = cfg.joints[0].limits.for_mode(LimitMode::Jog);
        assert_eq!(jog.acceleration_rad_s2, 32.0); // falls back to ceiling
                                                   // Homing values (robots/PAR6.xml homing fields).
        assert_eq!(cfg.homing.joints[0].timeout_s, 13.0);
        assert_eq!(cfg.homing.joints[0].two_pass_max_diff_ticks, 3500);
        assert_eq!(
            cfg.homing.joints[1].release.as_ref().unwrap().current_ma,
            150.0
        );
        assert_eq!(
            cfg.homing.joints[2].release.as_ref().unwrap().current_ma,
            -150.0
        );
        assert!(cfg.homing.joints[3].release.is_none());
        assert_eq!(cfg.homing.joints[5].strategy, HomingStrategy::Hall);
        assert!(cfg.homing.joints[3].home_offset_gripper_dependent);
        // Seconds→ticks conversion helper.
        assert_eq!(cfg.ticks(0.08), 20);

        // Round-trip: serialize → reparse → identical.
        let text = toml::to_string(&cfg).expect("serialize");
        let back = RobotConfig::from_toml_str(&text).expect("reparse");
        assert_eq!(cfg, back);
    }

    #[test]
    fn bundle_resolves_gripper_dependent_offsets() {
        let bundle = ConfigBundle::load(&config_dir().join("PAR6.toml")).expect("bundle");
        assert_eq!(bundle.grippers.len(), 3);
        let active = bundle.active_gripper().expect("active gripper");
        assert_eq!(active.name, "MSG_small_motor_150mm_rail");
        assert_eq!(active.driver.as_ref().unwrap().stroke_mm, 106.0);
        // J4 (index 4) is gripper-dependent and overridden by the MSG gripper.
        assert_eq!(bundle.effective_home_offset(4), Some(-2.070));
        // J3 (index 3) is gripper-dependent but no gripper overrides it → fallback.
        assert_eq!(bundle.effective_home_offset(3), Some(-2.717));
        // J0 is not gripper-dependent.
        assert_eq!(bundle.effective_home_offset(0), Some(2.96279));
        // Flange is a passive tool: no driver, no homing, but kinematics + offsets.
        let flange = bundle.grippers.iter().find(|g| g.name == "Flange").unwrap();
        assert!(flange.driver.is_none());
        assert!(flange.homing.is_none());
        assert_eq!(flange.arm_joint_home_offsets[0].home_offset_rad, -2.258);
        // Gripper round-trip.
        let text = toml::to_string(active).expect("serialize gripper");
        let back = GripperConfig::from_toml_str(&text).expect("reparse gripper");
        assert_eq!(*active, back);
    }

    #[test]
    fn validation_errors_name_the_field() {
        let path = config_dir().join("PAR6.toml");
        let good = RobotConfig::load(&path).unwrap();

        // status rate must divide the tick rate
        let mut cfg = good.clone();
        cfg.protocol.status_rate_hz = 60; // 250 / 60 is not integral
        let err = cfg.validate().unwrap_err().to_string();
        assert!(err.contains("protocol.status_rate_hz"), "{err}");

        // soft window must be non-empty
        let mut cfg = good.clone();
        cfg.joints[2].limits.soft_min_rad = cfg.joints[2].limits.soft_max_rad;
        let err = cfg.validate().unwrap_err().to_string();
        assert!(err.contains("joints[2].limits.soft_min_rad"), "{err}");

        // a mode may only ask for LESS than the hardware ceiling
        let mut cfg = good.clone();
        cfg.joints[0].limits.exec.as_mut().unwrap().velocity_rad_s = 100.0;
        let err = cfg.validate().unwrap_err().to_string();
        assert!(
            err.contains("joints[0].limits.exec.velocity_rad_s"),
            "{err}"
        );

        // homing table must cover every joint
        let mut cfg = good.clone();
        cfg.homing.joints.pop();
        let err = cfg.validate().unwrap_err().to_string();
        assert!(err.contains("homing.joints"), "{err}");

        // release sample_pct is a fraction
        let mut cfg = good.clone();
        cfg.homing.joints[1].release.as_mut().unwrap().sample_pct = 1.5;
        let err = cfg.validate().unwrap_err().to_string();
        assert!(err.contains("homing.joints[1].release.sample_pct"), "{err}");

        // sequence steps may only reference real joints
        let mut cfg = good.clone();
        cfg.homing.sequence[1].home.as_mut().unwrap().joints = vec![9];
        let err = cfg.validate().unwrap_err().to_string();
        assert!(err.contains("homing.sequence[1].home.joints"), "{err}");

        // duplicate node ids collide on the bus
        let mut cfg = good.clone();
        cfg.joints[1].node_id = 0;
        let err = cfg.validate().unwrap_err().to_string();
        assert!(err.contains("joints[1].node_id"), "{err}");

        // unknown TOML keys are rejected at parse time (typo protection)
        let text = std::fs::read_to_string(&path).unwrap();
        let text = text.replacen("tick_dt_s", "tick_dt_sec", 1);
        assert!(RobotConfig::from_toml_str(&text).is_err());
    }
}
