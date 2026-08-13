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
    RobotSection, ScanConfig, StreamDefaults, TimingConfig, WatchdogAction,
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
    /// directory, drop the sequence steps the active tool cannot run,
    /// then cross-validate.
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
        let mut bundle = Self { robot, grippers };
        bundle.drop_gripper_homing_without_a_gripper();
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

    /// Strip the gripper work out of the homing sequence when the active
    /// tool has no CAN driver to run it on.
    ///
    /// The sequence in `PAR6.toml` is written for the shipped gripper and
    /// is shared by every tool, so selecting the bare flange (vendor
    /// `Flange.xml`, `CAN_gripper = 0`) leaves steps addressing a node
    /// that is not on the bus. Refusing the bundle instead would make the
    /// safest possible first power-on — arm bare, nothing on the flange —
    /// the one configuration that cannot boot, and would force the
    /// operator to hand-edit the shared sequence this file exists to stop
    /// them transcribing. The vendor resolves it the same way, skipping
    /// both gripper homing modes with a warning
    /// (`rcb-runtime/robotics/homing.py`).
    ///
    /// A home group left with no joints and no gripper is a no-op step
    /// the FSM walks straight through, so the surrounding sequence and
    /// its arm-joint references are untouched.
    fn drop_gripper_homing_without_a_gripper(&mut self) {
        if self.active_gripper().is_none_or(|g| g.driver.is_some()) {
            return;
        }
        let tool = self.robot.robot.active_gripper.clone();
        let strip = |where_: String, moves: &mut Vec<PreMove>| {
            let before = moves.len();
            moves.retain(|m| !matches!(m, PreMove::GripperMove { .. }));
            if moves.len() < before {
                log::warn!(
                    "{where_}: skipping {} gripper move(s) — tool `{tool}` has no CAN driver",
                    before - moves.len()
                );
            }
        };
        for (i, step) in self.robot.homing.sequence.iter_mut().enumerate() {
            if let Some(mode) = step.home.as_mut().and_then(|h| h.gripper.take()) {
                log::warn!(
                    "homing.sequence[{i}]: skipping {mode:?} gripper homing — \
                     tool `{tool}` has no CAN driver"
                );
            }
            strip(
                format!("homing.sequence[{i}].pre_moves"),
                &mut step.pre_moves,
            );
            strip(
                format!("homing.sequence[{i}].post_moves"),
                &mut step.post_moves,
            );
        }
        strip(
            "homing.post_moves".into(),
            &mut self.robot.homing.post_moves,
        );
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
        // Motor-mode gripper homing runs the joint FSM against the
        // gripper's own `[homing]` parameters; without them the step
        // would report Done on the tick it started and the jaws would
        // never be referenced. A tool WITHOUT a driver never gets here —
        // its gripper steps were dropped above.
        let motor_homed = self.robot.homing.sequence.iter().any(|s| {
            s.home
                .as_ref()
                .is_some_and(|h| h.gripper == Some(GripperHomeMode::Motor))
        });
        if motor_homed && active.homing.is_none() {
            return Err(invalid(
                "homing.sequence",
                format!(
                    "sequence homes the gripper motor but gripper `{}` has no [homing] section",
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

    /// A throwaway copy of `config/` a test may rewrite.
    /// [`ConfigBundle::load`] reads `grippers/` relative to the robot
    /// file, so selecting a different tool means moving the whole tree.
    struct TempConfig(PathBuf);

    impl TempConfig {
        /// Copy the shipped tree, then apply `edit` to each file's text
        /// (keyed by file name; `PAR6.toml` for the robot).
        fn new(edit: impl Fn(&str, &str) -> String) -> Self {
            use std::sync::atomic::{AtomicU32, Ordering};
            static N: AtomicU32 = AtomicU32::new(0);
            let root = std::env::temp_dir().join(format!(
                "par6-config-{}-{}",
                std::process::id(),
                N.fetch_add(1, Ordering::Relaxed)
            ));
            let grippers = root.join("grippers");
            std::fs::create_dir_all(&grippers).expect("temp config dir");
            let src = config_dir();
            let mut files = vec![src.join("PAR6.toml")];
            files.extend(
                std::fs::read_dir(src.join("grippers"))
                    .expect("grippers dir")
                    .map(|e| e.expect("dir entry").path()),
            );
            for path in files {
                let name = path.file_name().unwrap().to_str().unwrap().to_owned();
                let text = edit(&name, &std::fs::read_to_string(&path).expect("read config"));
                let dest = if name == "PAR6.toml" {
                    root.join(&name)
                } else {
                    grippers.join(&name)
                };
                std::fs::write(dest, text).expect("write config");
            }
            Self(root)
        }

        fn robot(&self) -> PathBuf {
            self.0.join("PAR6.toml")
        }
    }

    impl Drop for TempConfig {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    fn select_tool(name: &str) -> impl Fn(&str, &str) -> String + '_ {
        move |file, text| {
            if file == "PAR6.toml" {
                text.replace(
                    "active_gripper = \"MSG_small_motor_150mm_rail\"",
                    &format!("active_gripper = \"{name}\""),
                )
            } else {
                text.to_owned()
            }
        }
    }

    /// Drop one `[section]` and everything up to the next table header.
    fn without_section(text: &str, section: &str) -> String {
        let start = text.find(section).expect("section present");
        let tail = &text[start + section.len()..];
        let end = tail
            .find("\n[")
            .map(|i| start + section.len() + i + 1)
            .unwrap_or(text.len());
        format!("{}{}", &text[..start], &text[end..])
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

    /// The bare flange is a supported tool, and the shared sequence is
    /// what adapts to it: selecting `Flange` (no `[driver]`, vendor
    /// `CAN_gripper = 0`) must load, and must leave the runtime a
    /// sequence with nothing addressed to a gripper node that is not on
    /// the bus — otherwise the firmware-calibrate step runs into its
    /// 10 s timeout and fails the whole homing run.
    #[test]
    fn the_bare_flange_loads_and_takes_the_gripper_out_of_the_sequence() {
        let flanged = TempConfig::new(select_tool("Flange"));

        // The premise: the shipped sequence does home the gripper.
        let stock = ConfigBundle::load(&config_dir().join("PAR6.toml")).expect("stock bundle");
        let stock_modes: Vec<_> = stock
            .robot
            .homing
            .sequence
            .iter()
            .filter_map(|s| s.home.as_ref().and_then(|h| h.gripper))
            .collect();
        assert_eq!(
            stock_modes,
            vec![GripperHomeMode::Firmware, GripperHomeMode::Motor],
            "the shipped sequence must still exercise both gripper modes"
        );

        let bundle = ConfigBundle::load(&flanged.robot()).expect("the bare flange must load");
        assert_eq!(
            bundle.active_gripper().map(|g| g.name.as_str()),
            Some("Flange")
        );
        assert!(
            bundle
                .robot
                .homing
                .sequence
                .iter()
                .all(|s| s.home.as_ref().is_none_or(|h| h.gripper.is_none())),
            "no step may home a gripper that has no driver"
        );
        assert!(
            bundle
                .robot
                .homing
                .sequence
                .iter()
                .flat_map(|s| s.pre_moves.iter().chain(s.post_moves.iter()))
                .chain(bundle.robot.homing.post_moves.iter())
                .all(|m| !matches!(m, PreMove::GripperMove { .. })),
            "no move may command a gripper that has no driver"
        );

        // The arm's own homing work survives intact — this drops the
        // gripper, not the sequence.
        let arm_steps: Vec<Vec<u8>> = bundle
            .robot
            .homing
            .sequence
            .iter()
            .filter_map(|s| s.home.as_ref())
            .map(|h| h.joints.clone())
            .filter(|j| !j.is_empty())
            .collect();
        assert_eq!(arm_steps, vec![vec![0], vec![1, 2], vec![3, 5], vec![4]]);
        // ...and the flange's own J4 offset is what the runtime homes to.
        assert_eq!(bundle.effective_home_offset(4), Some(-2.258));
    }

    /// A gripper that IS on the bus but has no `[homing]` parameters
    /// cannot be motor-homed: the FSM would report Done on the tick it
    /// started and the jaws would never touch their endstop.
    #[test]
    fn a_can_gripper_without_homing_params_is_still_refused() {
        let select = select_tool("SSG48");
        let broken = TempConfig::new(|file, text| {
            let text = select(file, text);
            if file == "SSG48.toml" {
                without_section(&text, "[homing]")
            } else {
                text
            }
        });
        let err = ConfigBundle::load(&broken.robot())
            .expect_err("motor homing without [homing] must be refused")
            .to_string();
        assert!(err.contains("homing.sequence"), "{err}");
        assert!(err.contains("SSG48"), "{err}");

        // Intact, the same tool loads: it is the missing section that is
        // refused, not the tool selection.
        let ok = TempConfig::new(select_tool("SSG48"));
        ConfigBundle::load(&ok.robot()).expect("SSG48 with its [homing] section must load");
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

    #[test]
    fn loop_bands_default_to_the_vendor_values_and_reject_unusable_ones() {
        let text = std::fs::read_to_string(config_dir().join("PAR6.toml")).unwrap();

        // A config that says nothing about timing runs the vendor bands,
        // so hardware behavior does not depend on this section existing.
        let stock = RobotConfig::from_toml_str(&text).unwrap();
        let bands = stock.loop_timing();
        assert_eq!(bands.degraded_factor, 1.05);
        assert_eq!(bands.critical_factor, 1.10);
        assert_eq!(bands.critical_sustain_s, 1.0);

        // A declared section is what the runtime then uses.
        let declared = RobotConfig::from_toml_str(&format!(
            "{text}\n[timing]\ndegraded_factor = 1.5\n\
             critical_factor = 4.0\ncritical_sustain_s = 5.0\n"
        ))
        .expect("declared timing section must load");
        assert_eq!(declared.loop_timing(), TimingConfig::SIM);
        // …and survives a serialize/reparse round-trip.
        let back = RobotConfig::from_toml_str(&toml::to_string(&declared).unwrap()).unwrap();
        assert_eq!(back, declared);

        // Bands that would fire on a loop meeting its deadline, an
        // inverted pair, or a non-finite/zero sustain are refused by name.
        for (section, field) in [
            ("degraded_factor = 1.0", "timing.degraded_factor"),
            ("critical_factor = 0.9", "timing.critical_factor"),
            ("degraded_factor = nan", "timing.degraded_factor"),
            ("critical_factor = inf", "timing.critical_factor"),
            (
                "degraded_factor = 2.0\ncritical_factor = 1.5",
                "timing.critical_factor",
            ),
            ("critical_sustain_s = 0.0", "timing.critical_sustain_s"),
            ("critical_sustain_s = -1.0", "timing.critical_sustain_s"),
        ] {
            let err = RobotConfig::from_toml_str(&format!("{text}\n[timing]\n{section}\n"))
                .expect_err(&format!("`{section}` must be refused"))
                .to_string();
            assert!(err.contains(field), "{section}: {err}");
        }

        // Typos in the section are caught, not silently defaulted.
        assert!(RobotConfig::from_toml_str(&format!(
            "{text}\n[timing]\ncritical_factor_x = 4.0\n"
        ))
        .is_err());
    }
}
