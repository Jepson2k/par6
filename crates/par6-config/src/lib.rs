//! Robot, gripper, and homing configuration.
//!
//! TOML schema covering: joint limits (soft/hard, per-mode kinodynamic),
//! driver gains (KPP/KPV/KIV/KPIQ/KIIQ/KP/KD), current/velocity/voltage
//! limits, kt + gear ratios + directions, encoder geometry, homing
//! parameters (per-joint FSM settings, sequence steps, gripper-dependent
//! home offsets), bus node map, tick rate. Values for PAR6 are transcribed
//! from the vendor XML.
//!
//! All time constants are seconds in config, converted with `round(s / dt)`
//! at construction — never hardcoded tick counts. Use
//! [`RobotConfig::ticks`] for the conversion.
//!
//! Layout on disk (repo `config/`, a symlink to `python/par6/_data/config/`
//! so the pip package ships the same files rather than a copy):
//!
//! ```text
//! config/PAR6.toml            robot + homing + bus + protocol
//! config/grippers/*.toml      one file per tool (MSG…, SSG48, Flange, …)
//! ```
//!
//! Load a robot alone with [`RobotConfig::load`], a single gripper with
//! [`ToolConfig::load`], or everything (robot + every gripper next to
//! it, cross-validated) with [`ConfigBundle::load`].

mod gripper;
mod homing;
mod io;
mod robot;

pub use gripper::{
    ArmJointHomeOffset, GripperDriverConfig, SettleTimings, ToolConfig, ToolKinematics,
};
pub use homing::{
    GripperHomeMode, HomeGroup, HomingConfig, HomingStrategy, JointHoming, MoveTo, PostHomeConfig,
    PreMove, ReleaseConfig, SequenceStep,
};
pub use io::{IoConfig, IoLine, MAX_IO_LINES};
pub use robot::{
    BusConfig, ControlMode, DriverType, Gains, JogDefaults, JogProfile, JointConfig, JointLimits,
    KtFetchConfig, KtSource, LimitMode, LimitsSection, ModeLimits, MotionConfig, ProtocolConfig,
    ResolvedLimits, RippleHarmonic, RobotConfig, RobotSection, ScanConfig, SelfcalConfig,
    SimConfig, StreamDefaults, TimingConfig, WatchdogAction, MAX_OPEN_RETRY_S,
    MAX_RIPPLE_HARMONICS,
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
    pub tools: Vec<ToolConfig>,
    /// Installation-layer keep-out shapes from the robot TOML's
    /// `[[installation_shapes]]` array (empty when the section is
    /// absent).
    pub installation_shapes: Vec<par6_proto::Shape>,
}

impl ConfigBundle {
    /// Load `robot_toml` plus every `grippers/*.toml` in the same
    /// directory, drop the sequence steps the active tool cannot run,
    /// then cross-validate.
    pub fn load(robot_toml: &Path) -> Result<Self, ConfigError> {
        Self::load_inner(robot_toml, None)
    }

    /// [`load`](Self::load), fitted with the tool named `tool` rather than
    /// the one `active_tool` boots with — the tool on the arm is the
    /// operator's to say. Matched case-insensitively; an unknown name is
    /// refused. The homing sequence is trimmed for THIS tool, which is why
    /// the choice is made here rather than patched onto a loaded bundle.
    pub fn load_fitted(robot_toml: &Path, tool: &str) -> Result<Self, ConfigError> {
        Self::load_inner(robot_toml, Some(tool))
    }

    fn load_inner(robot_toml: &Path, fitted: Option<&str>) -> Result<Self, ConfigError> {
        let (mut robot, installation_shapes) = load_robot_with_shapes(robot_toml)?;
        // `tools/` is the name; `grippers/` is what it used to be called,
        // and a config on disk is the operator's, not ours to invalidate.
        // A tool is not necessarily a gripper — the bare flange is one.
        let beside = |name: &str| {
            robot_toml
                .parent()
                .map(|p| p.join(name))
                .unwrap_or_else(|| Path::new(name).to_path_buf())
        };
        let dir = match beside("tools") {
            d if d.is_dir() => d,
            _ => beside("grippers"),
        };
        let mut paths: Vec<_> = std::fs::read_dir(&dir)
            .map_err(|source| ConfigError::Io {
                path: dir.display().to_string(),
                source,
            })?
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.extension().is_some_and(|e| e == "toml"))
            .collect();
        paths.sort();
        let tools = paths
            .iter()
            .map(|p| ToolConfig::load(p))
            .collect::<Result<Vec<_>, _>>()?;
        if let Some(name) = fitted {
            let tool = tools
                .iter()
                .find(|t| t.name.eq_ignore_ascii_case(name.trim()))
                .ok_or_else(|| invalid("tool", format!("no tool named `{name}`")))?;
            robot.robot.active_tool.clone_from(&tool.name);
        }
        let mut bundle = Self {
            robot,
            tools,
            installation_shapes,
        };
        bundle.drop_gripper_homing_without_a_gripper();
        bundle.validate()?;
        Ok(bundle)
    }

    /// The tool selected by `robot.active_tool` — a gripper, or a passive
    /// attachment like the bare flange.
    pub fn active_tool(&self) -> Option<&ToolConfig> {
        self.tools
            .iter()
            .find(|g| g.name == self.robot.robot.active_tool)
    }

    /// The tool whose drive reports `tool_id` in its device info
    /// (`ToolConfig::can_tool_id`); `None` for 0 and for an id no tool
    /// carries.
    pub fn tool_by_can_id(&self, tool_id: u8) -> Option<&ToolConfig> {
        (tool_id != 0).then(|| self.tools.iter().find(|g| g.can_tool_id == Some(tool_id)))?
    }

    /// Effective home offset for an arm joint under the ACTIVE gripper:
    /// the gripper's `arm_joint_home_offsets` override when the joint is
    /// flagged `home_offset_gripper_dependent` and the gripper provides
    /// one, else the joint's own `home_offset_rad` fallback.
    /// `None` when `joint` is out of range.
    pub fn effective_home_offset(&self, joint: usize) -> Option<f64> {
        let jh = self.robot.homing.joints.get(joint)?;
        if jh.home_offset_gripper_dependent {
            if let Some(g) = self.active_tool() {
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
    /// Stripping can empty a step completely — the two gripper-homing
    /// steps do nothing else. An empty group, and an empty step, are both
    /// config errors when someone writes them by hand, and
    /// [`Self::validate`] says so, so the emptied ones are removed rather
    /// than left behind. Steps are addressed by order and never by index,
    /// and the joints they home are named inside them, so dropping one
    /// leaves the remaining sequence and its arm-joint references intact.
    fn drop_gripper_homing_without_a_gripper(&mut self) {
        if self.active_tool().is_none_or(|g| g.driver.is_some()) {
            return;
        }
        let tool = self.robot.robot.active_tool.clone();
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
            if step.home.as_ref().is_some_and(|h| h.joints.is_empty()) {
                step.home = None;
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
        self.robot.homing.sequence.retain(|step| {
            !(step.pre_moves.is_empty()
                && step.home.is_none()
                && step.move_to.is_empty()
                && step.post_moves.is_empty())
        });
    }

    fn validate(&self) -> Result<(), ConfigError> {
        for (i, s) in self.installation_shapes.iter().enumerate() {
            // The same contract SET_SHAPES enforces on the wire, written
            // once in par6-proto and reported here under this section's
            // path rather than the wire's.
            if let Err((what, why)) = par6_proto::validate_shape(s) {
                let leaf = what.strip_prefix("shape.").unwrap_or(what);
                return Err(invalid(
                    format!("installation_shapes[{i}].{leaf}"),
                    format!("shape `{}`: {why}", s.name),
                ));
            }
        }
        for (i, tool) in self.tools.iter().enumerate() {
            let Some(id) = tool.can_tool_id else {
                continue;
            };
            if let Some(other) = self.tools[..i].iter().find(|t| t.can_tool_id == Some(id)) {
                return Err(invalid(
                    "can_tool_id",
                    format!(
                        "tools `{}` and `{}` both claim id {id}; a drive reports one id \
                         and it must name one tool",
                        other.name, tool.name
                    ),
                ));
            }
        }
        let Some(active) = self.active_tool() else {
            return Err(invalid(
                "robot.active_tool",
                format!(
                    "no gripper named `{}` found in grippers/ directory",
                    self.robot.robot.active_tool
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

/// Parse the robot TOML, splitting the `[[installation_shapes]]` array
/// off before the strict `RobotConfig` schema sees the text.
///
/// The shapes ride in the robot file (the parol6 arrangement: keep-outs
/// are installation config, next to the other installation limits), but
/// they are a server-layer vocabulary, not a robot parameter — so
/// `RobotConfig` keeps its own schema and its `deny_unknown_fields` typo
/// protection, and the split hands it exactly the document minus this one
/// key. A file without the key takes the plain [`RobotConfig::load`]
/// path, byte for byte.
fn load_robot_with_shapes(
    path: &Path,
) -> Result<(RobotConfig, Vec<par6_proto::Shape>), ConfigError> {
    let text = read_to_string(path)?;
    let parse_err = |source: toml::de::Error| ConfigError::Parse {
        path: path.display().to_string(),
        source: Box::new(source),
    };
    let mut table: toml::Table = toml::from_str(&text).map_err(parse_err)?;
    let Some(value) = table.remove("installation_shapes") else {
        return Ok((RobotConfig::load(path)?, Vec::new()));
    };
    let shapes: Vec<par6_proto::Shape> = value.try_into().map_err(parse_err)?;
    let rest = toml::to_string(&table).map_err(|e| {
        invalid(
            "installation_shapes",
            format!("cannot re-serialize the remaining config: {e}"),
        )
    })?;
    let robot = RobotConfig::from_toml_str(&rest).map_err(|e| match e {
        // Re-attach the real path: the round-trip through a string names
        // `<string>` otherwise, which is useless in a startup error.
        ConfigError::Parse { source, .. } => ConfigError::Parse {
            path: path.display().to_string(),
            source,
        },
        other => other,
    })?;
    Ok((robot, shapes))
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
    /// The driven tool these cases exercise. They SELECT it rather than
    /// asserting the shipped config names it: which tool is bolted on is the
    /// operator's to change, and a test that pins `active_tool` turns a
    /// tool swap into a suite failure.
    const DRIVEN_TOOL: &str = "MSG_small_motor_200mm_rail";

    fn select_tool(name: &str) -> impl Fn(&str, &str) -> String + '_ {
        move |file, text| {
            if file == "PAR6.toml" {
                // By line, so the shipped tool can change (and carry a
                // trailing comment) without silently selecting nothing.
                text.split_inclusive('\n')
                    .map(|line| {
                        if line.trim_start().starts_with("active_tool") {
                            format!("active_tool = \"{name}\"\n")
                        } else {
                            line.to_owned()
                        }
                    })
                    .collect()
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
        assert_eq!(cfg.joints.len(), 6);
        assert_eq!(cfg.homing.joints.len(), cfg.joints.len());
        // Seconds become ticks by rounding, not truncating.
        let dt = cfg.robot.tick_dt_s;
        assert_eq!(cfg.tick_rate_hz(), 1.0 / dt);
        assert_eq!(cfg.ticks(2.4 * dt), 2);
        assert_eq!(cfg.ticks(2.6 * dt), 3);

        // Round-trip: serialize → reparse → identical.
        let text = toml::to_string(&cfg).expect("serialize");
        let back = RobotConfig::from_toml_str(&text).expect("reparse");
        assert_eq!(cfg, back);
    }

    /// A mode's limits fall back to the ceiling field by field: a mode
    /// table that leaves jerk or torque rate out runs those at the
    /// ceiling, never at zero.
    #[test]
    fn a_mode_table_falls_back_to_the_ceiling_field_by_field() {
        let path = config_dir().join("PAR6.toml");
        let mut limits = RobotConfig::load(&path)
            .expect("PAR6.toml must load")
            .joints[1]
            .limits;
        limits.jerk_rad_s3 = 30.0;
        limits.torque_rate_nm_s = 364.0;
        limits.exec = Some(ModeLimits {
            velocity_rad_s: 1.0,
            acceleration_rad_s2: 2.0,
            jerk_rad_s3: None,
            torque_rate_nm_s: None,
        });
        limits.jog = Some(ModeLimits {
            velocity_rad_s: 1.0,
            acceleration_rad_s2: 2.0,
            jerk_rad_s3: Some(5.0),
            torque_rate_nm_s: Some(50.0),
        });
        limits.stream = None;
        let exec = limits.for_mode(LimitMode::Exec);
        assert_eq!((exec.velocity_rad_s, exec.acceleration_rad_s2), (1.0, 2.0));
        assert_eq!(
            (exec.jerk_rad_s3, exec.torque_rate_nm_s),
            (Some(30.0), Some(364.0))
        );
        let jog = limits.for_mode(LimitMode::Jog);
        assert_eq!(
            (jog.jerk_rad_s3, jog.torque_rate_nm_s),
            (Some(5.0), Some(50.0))
        );
        let stream = limits.for_mode(LimitMode::Stream);
        assert_eq!(
            (
                stream.velocity_rad_s,
                stream.acceleration_rad_s2,
                stream.jerk_rad_s3,
                stream.torque_rate_nm_s
            ),
            (
                limits.velocity_rad_s,
                limits.acceleration_rad_s2,
                Some(30.0),
                Some(364.0)
            )
        );
    }

    /// A config written before the tool/gripper rename still loads.
    ///
    /// `active_gripper` became `active_tool` because not every tool is a
    /// gripper — the bare flange has no jaw and no driver — but a config
    /// already on disk belongs to whoever wrote it, and a rename that
    /// invalidates it is a rename that breaks a running arm. Same for the
    /// directory: `tools/` is preferred, `grippers/` still resolves.
    #[test]
    fn a_config_using_the_old_gripper_spelling_still_loads() {
        let old = TempConfig::new(|file, text| {
            if file == "PAR6.toml" {
                text.split_inclusive('\n')
                    .map(|line| {
                        if line.trim_start().starts_with("active_tool") {
                            format!("active_gripper = \"{DRIVEN_TOOL}\"\n")
                        } else {
                            line.to_owned()
                        }
                    })
                    .collect()
            } else {
                text.to_owned()
            }
        });
        let text = std::fs::read_to_string(old.robot()).expect("read");
        assert!(
            text.contains("active_gripper ="),
            "the fixture must actually use the old spelling"
        );
        let bundle = ConfigBundle::load(&old.robot()).expect("the old spelling must still load");
        assert_eq!(
            bundle.active_tool().map(|t| t.name.as_str()),
            Some(DRIVEN_TOOL),
            "the aliased key must select the tool it names"
        );
    }

    /// A config with no `[selfcal]` table loads, with exactly what an empty
    /// table would give it. The table only configures the standalone
    /// calibration binary, so its absence — any config written before it
    /// existed — is no reason to refuse to run the arm.
    #[test]
    fn a_config_without_a_selfcal_table_loads_with_its_defaults() {
        let with_table = |empty: bool| {
            TempConfig::new(move |file, text| {
                if file != "PAR6.toml" {
                    return text.to_owned();
                }
                let mut out = String::new();
                let mut in_selfcal = false;
                for line in text.split_inclusive('\n') {
                    let t = line.trim_start();
                    if t.starts_with('[') {
                        in_selfcal = t.starts_with("[selfcal]");
                        if in_selfcal && empty {
                            out.push_str("[selfcal]\n");
                        }
                    }
                    if !in_selfcal {
                        out.push_str(line);
                    }
                }
                out
            })
        };
        let absent = with_table(false);
        let empty = with_table(true);
        let text = std::fs::read_to_string(absent.robot()).expect("read");
        assert!(
            !text.contains("[selfcal]"),
            "the fixture must actually drop the table"
        );
        let absent =
            ConfigBundle::load(&absent.robot()).expect("a config without [selfcal] must load");
        let empty = ConfigBundle::load(&empty.robot()).expect("an empty [selfcal] must load");
        assert_eq!(absent.robot.selfcal, empty.robot.selfcal);
    }
    #[test]
    fn bundle_resolves_gripper_dependent_offsets() {
        // The tool also lists J0, which is not gripper-dependent: its own
        // offset has to win there.
        let tool_file = format!("{DRIVEN_TOOL}.toml");
        let driven = TempConfig::new(|file, text| {
            let text = select_tool(DRIVEN_TOOL)(file, text);
            if file == tool_file {
                format!("{text}\n[[arm_joint_home_offsets]]\njoint = 0\nhome_offset_rad = 1.234\n")
            } else {
                text
            }
        });
        let bundle = ConfigBundle::load(&driven.robot()).expect("bundle");
        // Every shipped TOML is loaded and cross-validated, under a name
        // no other file claims — a duplicate would make
        // `active_tool` pick by file order.
        let files = std::fs::read_dir(config_dir().join("grippers"))
            .expect("gripper dir")
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.extension().is_some_and(|e| e == "toml"))
            .count();
        assert_eq!(bundle.tools.len(), files, "one config per shipped file");
        let mut names: Vec<&str> = bundle.tools.iter().map(|g| g.name.as_str()).collect();
        names.sort_unstable();
        let distinct = names.len();
        names.dedup();
        assert_eq!(names.len(), distinct, "gripper names collide: {names:?}");

        let active = bundle.active_tool().expect("active gripper");
        assert_eq!(active.name, DRIVEN_TOOL);
        // A joint homes to the tool's offset only where it is flagged
        // gripper-dependent and the tool overrides it; everywhere else, to
        // its own — including a joint the tool lists but is not flagged.
        let mut cases = [false; 3];
        for (j, jh) in bundle.robot.homing.joints.iter().enumerate() {
            let tool = active
                .arm_joint_home_offsets
                .iter()
                .find(|o| usize::from(o.joint) == j)
                .map(|o| o.home_offset_rad);
            let want = match (jh.home_offset_gripper_dependent, tool) {
                (true, Some(v)) => {
                    cases[0] = true;
                    v
                }
                (true, None) => {
                    cases[1] = true;
                    jh.home_offset_rad
                }
                (false, Some(_)) => {
                    cases[2] = true;
                    jh.home_offset_rad
                }
                (false, None) => jh.home_offset_rad,
            };
            assert_eq!(bundle.effective_home_offset(j), Some(want), "J{j}");
        }
        assert_eq!(
            cases, [true; 3],
            "overridden, fallback and unflagged-but-listed joints must all occur"
        );
        // Flange is a passive tool: no driver, no homing.
        let flange = bundle.tools.iter().find(|g| g.name == "Flange").unwrap();
        assert!(flange.driver.is_none());
        assert!(flange.homing.is_none());
        // Gripper round-trip.
        let text = toml::to_string(active).expect("serialize gripper");
        let back = ToolConfig::from_toml_str(&text).expect("reparse gripper");
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

        // The premise: with a driven tool selected, the sequence homes the
        // gripper. Selected here, not read off the shipped config, so which
        // tool is actually bolted on stays the operator's choice.
        let driven = TempConfig::new(select_tool(DRIVEN_TOOL));
        let stock = ConfigBundle::load(&driven.robot()).expect("driven bundle");
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
            "the sequence for a driven tool must still exercise both gripper modes"
        );

        let bundle = ConfigBundle::load(&flanged.robot()).expect("the bare flange must load");
        assert_eq!(
            bundle.active_tool().map(|g| g.name.as_str()),
            Some("Flange")
        );
        // Loading is not enough: the daemon and par6-selfcal both re-check
        // the stripped robot config on startup, so whatever stripping
        // leaves behind has to satisfy that check too. Leaving an emptied
        // home group (or an emptied step) behind stopped both of them from
        // starting with nothing on the flange.
        bundle
            .robot
            .validate()
            .expect("the stripped sequence must still satisfy RobotConfig::validate");
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
        // gripper, not the sequence: every arm home group, nudge and
        // move_to the driven tool's sequence has, in order.
        let arm_work = |b: &ConfigBundle| {
            let arm = |moves: &[PreMove]| -> Vec<PreMove> {
                moves
                    .iter()
                    .filter(|m| !matches!(m, PreMove::GripperMove { .. }))
                    .copied()
                    .collect()
            };
            let mut steps: Vec<_> = b
                .robot
                .homing
                .sequence
                .iter()
                .map(|s| {
                    (
                        arm(&s.pre_moves),
                        s.home
                            .as_ref()
                            .map(|h| h.joints.clone())
                            .unwrap_or_default(),
                        s.move_to.clone(),
                        arm(&s.post_moves),
                    )
                })
                .filter(|(pre, home, to, post)| {
                    !(pre.is_empty() && home.is_empty() && to.is_empty() && post.is_empty())
                })
                .collect();
            steps.push((arm(&b.robot.homing.post_moves), vec![], vec![], vec![]));
            steps
        };
        let driven_work = arm_work(&stock);
        assert!(
            driven_work.iter().any(|(pre, ..)| !pre.is_empty())
                && driven_work.iter().any(|(_, _, to, _)| !to.is_empty()),
            "the premise: the sequence has arm nudges and move_to entries to keep"
        );
        assert_eq!(arm_work(&bundle), driven_work);
        // ...and the flange's own J4 offset is what the runtime homes to.
        let flange = bundle.active_tool().expect("the flange");
        let j4 = flange
            .arm_joint_home_offsets
            .iter()
            .find(|o| o.joint == 4)
            .expect("the flange sets J4's offset");
        assert_eq!(bundle.effective_home_offset(4), Some(j4.home_offset_rad));
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

    /// `[[installation_shapes]]` rides in the robot TOML and comes out of
    /// `ConfigBundle::load` as typed shapes, without costing `RobotConfig`
    /// its strict schema: the same file's robot half still validates, and
    /// a file WITHOUT the section still loads to an empty list.
    #[test]
    fn installation_shapes_load_from_the_robot_toml() {
        let stock = ConfigBundle::load(&config_dir().join("PAR6.toml")).expect("stock bundle");
        assert_eq!(
            stock
                .installation_shapes
                .iter()
                .map(|s| s.name.as_str())
                .collect::<Vec<_>>(),
            ["floor"],
            "the shipped config declares the ground the robot stands on"
        );

        // A robot that declares no ground at all still loads, with an
        // empty layer.
        let bare =
            TempConfig::new(|file, text| {
                if file == "PAR6.toml" {
                    let stripped = without_section(
                        &without_section(text, "[[installation_shapes]]"),
                        "[installation_shapes.physics]",
                    );
                    assert!(
                        stripped.lines().all(|l| l.trim_start().starts_with('#')
                            || !l.contains("installation_shapes")),
                        "the floor is the only declared shape"
                    );
                    stripped
                } else {
                    text.to_owned()
                }
            });
        let bundle = ConfigBundle::load(&bare.robot()).expect("no installation shapes loads");
        assert!(bundle.installation_shapes.is_empty());

        let with_shapes = TempConfig::new(|file, text| {
            if file == "PAR6.toml" {
                format!(
                    "{text}\n[[installation_shapes]]\n\
                     name = \"table\"\nkind = \"box\"\n\
                     params = [0.8, 0.8, 0.02]\n\
                     pose = [0.3, 0.0, -0.11, 0.0, 0.0, 0.0]\n\
                     \n[[installation_shapes]]\n\
                     name = \"marker\"\nkind = \"sphere\"\nparams = [0.05]\n\
                     pose = [0.0, 0.4, 0.2, 0.0, 0.0, 0.0]\n\
                     collision = false\nmargin = 0.01\n"
                )
            } else {
                text.to_owned()
            }
        });
        let bundle = ConfigBundle::load(&with_shapes.robot()).expect("shapes must load");
        assert_eq!(
            bundle.installation_shapes[1..],
            [
                par6_proto::Shape {
                    attachment: None,
                    name: "table".into(),
                    kind: "box".into(),
                    params: vec![0.8, 0.8, 0.02],
                    pose: vec![0.3, 0.0, -0.11, 0.0, 0.0, 0.0],
                    collision: true,
                    margin: None,
                    physics: None,
                },
                par6_proto::Shape {
                    attachment: None,
                    name: "marker".into(),
                    kind: "sphere".into(),
                    params: vec![0.05],
                    pose: vec![0.0, 0.4, 0.2, 0.0, 0.0, 0.0],
                    collision: false,
                    margin: Some(0.01),
                    physics: None,
                },
            ]
        );
        assert_eq!(bundle.installation_shapes[0].name, "floor");
        // The robot half of the same file went through its normal
        // parse-and-validate path.
        assert_eq!(bundle.robot, stock.robot);
    }

    /// Malformed `[[installation_shapes]]` entries fail the LOAD with a
    /// message pointing at the problem — a keep-out that does not parse
    /// must never become a keep-out that silently is not there.
    #[test]
    fn malformed_installation_shapes_are_refused_by_name() {
        let load_with = |entry: &str| {
            let cfg = TempConfig::new(|file, text| {
                if file == "PAR6.toml" {
                    format!("{text}\n[[installation_shapes]]\n{entry}\n")
                } else {
                    text.to_owned()
                }
            });
            ConfigBundle::load(&cfg.robot())
        };

        // A pose that is not [x, y, z, rx, ry, rz].
        let err = load_with(
            "name = \"wall\"\nkind = \"box\"\nparams = [1.0, 0.02, 1.0]\n\
             pose = [0.4, 0.0, 0.3]",
        )
        .expect_err("a 3-element pose must be refused")
        .to_string();
        assert!(err.to_lowercase().contains("length 6"), "{err}");

        // A typo'd key inside a shape entry (schema protection).
        let err = load_with(
            "name = \"wall\"\nkind = \"box\"\nparams = [1.0, 0.02, 1.0]\n\
             pose = [0.4, 0.0, 0.3, 0.0, 0.0, 0.0]\nmargins = 0.01",
        )
        .expect_err("an unknown shape key must be refused")
        .to_string();
        assert!(err.contains("margins"), "{err}");

        // A non-finite dimension, named by field and shape.
        let err = load_with(
            "name = \"wall\"\nkind = \"box\"\nparams = [1.0, nan, 1.0]\n\
             pose = [0.4, 0.0, 0.3, 0.0, 0.0, 0.0]",
        )
        .expect_err("a NaN dimension must be refused")
        .to_string();
        assert!(err.contains(".params"), "{err}");
        assert!(err.contains("wall"), "{err}");
    }

    /// A bus freshness window that rounds away to zero ticks is refused.
    ///
    /// Regression: `bus.stale_warn_s` and `bus.lost_s` convert with
    /// `round(s/dt)`, and the freshness clock tests `age >= threshold`.
    /// Under half a tick they round to zero, which reads every node
    /// Stale at age zero, turns every frame into a stale→fresh edge (a
    /// stored-config resend per node per tick), and latches `CAN_LOST`
    /// on the tick after a node's first frame — the arm cannot boot.
    /// `tick_dt_s` was validated only as `(0, 1) s`, so nothing caught
    /// the combination.
    #[test]
    fn a_freshness_window_that_rounds_away_is_refused() {
        let path = config_dir().join("PAR6.toml");
        let good = RobotConfig::load(&path).unwrap();
        // Shipped: 0.04 s over a 0.004 s tick is 10 ticks.
        good.validate().expect("the shipped config stands");

        // The re-ticked harnesses still stand: at the python rig's
        // 0.05 s tick, 0.04 s rounds UP to one tick. That is a real
        // window, so refusing it would break a rig that works.
        let mut cfg = good.clone();
        cfg.robot.tick_dt_s = 0.05;
        cfg.protocol.status_rate_hz = 20;
        cfg.validate()
            .expect("0.8 of a tick rounds up to a window that works");

        // Past half a tick it rounds away entirely.
        let mut cfg = good.clone();
        cfg.robot.tick_dt_s = 0.1; // 0.04 / 0.1 = 0.4 -> 0 ticks
        cfg.protocol.status_rate_hz = 10;
        let err = cfg.validate().unwrap_err().to_string();
        assert!(err.contains("bus.stale_warn_s"), "{err}");
        assert!(err.contains("rounds to zero ticks"), "{err}");
    }

    #[test]
    fn validation_errors_name_the_field() {
        for value in [0.0, -1.0, f64::NAN, f64::INFINITY, 2.01] {
            let mut cfg = RobotConfig::load(&config_dir().join("PAR6.toml")).unwrap();
            cfg.gravity_scale[2] = value;
            assert!(cfg
                .validate()
                .unwrap_err()
                .to_string()
                .contains("gravity_scale"));
        }
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
        assert!(stock.timing.is_none(), "the shipped config is silent");
        assert_eq!(stock.loop_timing(), TimingConfig::default());

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
            ("fifo_priority = 100", "timing.fifo_priority"),
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
