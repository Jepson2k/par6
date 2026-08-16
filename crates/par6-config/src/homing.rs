//! Homing schema: per-joint FSM parameters and the parallel step sequence.
//! PAR6 values from the vendor `robots/PAR6.xml` homing fields and
//! `config/PAR6_homing.xml`.

use serde::{Deserialize, Serialize};

use crate::{invalid, ConfigError};

/// Endstop detection strategy for one actuator.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HomingStrategy {
    /// Velocity drive into the endstop; hit = windowed stall gated with a
    /// current-ratio threshold.
    Stall,
    /// HALL pack drive (CAN cmd 31); hit = hall trigger/edge, position
    /// latched AT trigger. Hall joints skip two-pass and release.
    Hall,
}

/// Release phase: relieve gearbox preload after the endstop hit so the
/// latched position is the true resting endstop. Stall joints only;
/// omit the table to skip the phase (the vendor encodes "skip" as
/// `homing_release_duration_s = NaN`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReleaseConfig {
    /// Current-only command during release \[mA\]. Sign matters (drive
    /// direction); 0 = coast.
    pub current_ma: f64,
    /// How long to hold the release current \[s\].
    pub duration_s: f64,
    /// When to latch the encoder position: 0.0 = phase start, 1.0 = end.
    pub sample_pct: f64,
}

/// Optional per-joint position move after the home reference is applied.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PostHomeConfig {
    /// Target position \[rad\] (post-reference joint frame).
    pub position_rad: f64,
    /// Motor speed for the move \[ticks/s\].
    pub speed_ticks_s: f64,
}

/// Per-actuator homing FSM parameters. Used for arm joints
/// (`homing.joints`, index-aligned with `joints`) and for the gripper
/// motor (`[homing]` in a gripper TOML).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JointHoming {
    /// Endstop detection strategy.
    pub strategy: HomingStrategy,
    /// Motor speed toward the endstop \[ticks/s\] (magnitude; sign comes
    /// from `direction`).
    pub speed_ticks_s: f64,
    /// 0 = positive motor direction, 1 = negative.
    pub direction: u8,
    /// Current threshold \[mA\] — above this (with stall) = endstop hit.
    /// Also the per-node current limit applied for the duration of homing.
    pub current_ma: f64,
    /// Approach timeout \[s\]; exceeding it FAILS the joint (and the
    /// sequence).
    pub timeout_s: f64,
    /// Reverse-off-the-endstop time \[s\].
    pub backoff_s: f64,
    /// Two-pass homing: fast approach, back off, slow re-approach.
    /// Hall joints skip this regardless (the digital edge is the
    /// reference).
    pub two_pass: bool,
    /// Max |pass2 − pass1| \[encoder ticks\]; exceeding it fails homing.
    pub two_pass_max_diff_ticks: u32,
    /// Release phase; omit to skip.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub release: Option<ReleaseConfig>,
    /// Post-home position move; omit to skip.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub post_home: Option<PostHomeConfig>,
    /// Joint position at the endstop \[rad\] — the home reference
    /// post-condition. Fallback when `home_offset_gripper_dependent` and
    /// the active gripper does not override.
    pub home_offset_rad: f64,
    /// When true, the ACTIVE gripper's `arm_joint_home_offsets` may
    /// override `home_offset_rad` for this joint (a joint that homes
    /// against the gripper body).
    #[serde(default)]
    pub home_offset_gripper_dependent: bool,
}

/// Gripper homing flavor referenced by a sequence step.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GripperHomeMode {
    /// Stall-detect motor homing (gripper driver as a 7th joint).
    Motor,
    /// Firmware calibrate: CAN cmd 62 once, then DLC-0 empty polls every
    /// tick until done/timeout.
    Firmware,
}

/// One scripted move inside a sequence step (`pre_moves`/`post_moves`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum PreMove {
    /// Drop a joint to firmware idle (cmd 12 — limp, no active hold)
    /// for a duration, polling its encoder to keep freshness alive
    /// while other joints home.
    Idle {
        /// Arm joint index.
        joint: u8,
        /// Hold duration \[s\].
        duration_s: f64,
    },
    /// Velocity-mode push for a duration (used before the joint is
    /// calibrated, where a position move is meaningless).
    Nudge {
        /// Arm joint index.
        joint: u8,
        /// Signed motor speed \[ticks/s\].
        speed_ticks_s: f64,
        /// Push duration \[s\].
        duration_s: f64,
    },
    /// Position-mode move (joint must already be homed).
    Position {
        /// Arm joint index.
        joint: u8,
        /// Target position \[rad\].
        position_rad: f64,
        /// Move duration \[s\].
        duration_s: f64,
    },
    /// Firmware-mode gripper command (CAN cmd 61 fields).
    GripperMove {
        /// 0 = open … 255 = closed.
        position: u8,
        /// Speed byte.
        speed: u8,
        /// Current limit \[mA\].
        current_ma: i16,
        /// Activate bit (always 1 in practice).
        activate: bool,
        /// Action bit (1 = go to position).
        action: bool,
        /// E-stop bit.
        estop: bool,
        /// Release-direction bit.
        release_dir: bool,
        /// How long the move is given \[s\].
        duration_s: f64,
    },
}

/// Cubic-Hermite position move inside a step (zero start/end velocity).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MoveTo {
    /// Arm joint index.
    pub joint: u8,
    /// Target position \[rad\].
    pub position_rad: f64,
    /// Move duration \[s\] (timeout = duration + 2 s, warn-and-continue).
    pub duration_s: f64,
}

/// The set of actuators one step homes in parallel.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HomeGroup {
    /// Arm joint indices homed in parallel.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub joints: Vec<u8>,
    /// Home the gripper too/instead, in this mode.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gripper: Option<GripperHomeMode>,
}

/// One step of the homing sequence. Runs `pre_moves`, then the `home`
/// group in parallel, then `move_to` moves. Pre/post/move_to timeouts
/// warn and continue; home-phase timeouts FAIL the sequence.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SequenceStep {
    /// Moves executed before the home group.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub pre_moves: Vec<PreMove>,
    /// Actuators homed in parallel in this step.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub home: Option<HomeGroup>,
    /// Position moves executed after the home group.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub move_to: Vec<MoveTo>,
    /// Moves executed at the end of the step.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub post_moves: Vec<PreMove>,
}

/// Homing configuration: per-joint FSM parameters plus the sequence.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HomingConfig {
    /// Per-joint parameters, index-aligned with `joints`.
    pub joints: Vec<JointHoming>,
    /// The scripted parallel-step sequence.
    pub sequence: Vec<SequenceStep>,
    /// Global trailing moves after the last step.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub post_moves: Vec<PreMove>,
}

impl JointHoming {
    pub(crate) fn validate(&self, field_prefix: &str) -> Result<(), ConfigError> {
        let f = |name: &str| format!("{field_prefix}.{name}");
        if self.speed_ticks_s <= 0.0 {
            return Err(invalid(
                f("speed_ticks_s"),
                "must be > 0 (sign comes from `direction`)",
            ));
        }
        if self.direction > 1 {
            return Err(invalid(
                f("direction"),
                "must be 0 (positive) or 1 (negative)",
            ));
        }
        if self.current_ma <= 0.0 {
            return Err(invalid(f("current_ma"), "must be > 0"));
        }
        if self.timeout_s <= 0.0 {
            return Err(invalid(f("timeout_s"), "must be > 0"));
        }
        if self.backoff_s < 0.0 {
            return Err(invalid(f("backoff_s"), "must be >= 0"));
        }
        if let Some(r) = &self.release {
            if self.strategy == HomingStrategy::Hall {
                return Err(invalid(
                    f("release"),
                    "hall joints have no release phase — remove the table",
                ));
            }
            if r.duration_s <= 0.0 {
                return Err(invalid(
                    f("release.duration_s"),
                    "must be > 0 (omit the table to skip)",
                ));
            }
            if !(0.0..=1.0).contains(&r.sample_pct) {
                return Err(invalid(f("release.sample_pct"), "must be in [0, 1]"));
            }
        }
        if let Some(p) = &self.post_home {
            if p.speed_ticks_s <= 0.0 {
                return Err(invalid(f("post_home.speed_ticks_s"), "must be > 0"));
            }
        }
        Ok(())
    }
}

impl HomingConfig {
    pub(crate) fn validate(&self, num_joints: usize) -> Result<(), ConfigError> {
        if self.joints.len() != num_joints {
            return Err(invalid(
                "homing.joints",
                format!(
                    "must have one entry per joint ({} joints, {} entries)",
                    num_joints,
                    self.joints.len()
                ),
            ));
        }
        for (i, jh) in self.joints.iter().enumerate() {
            jh.validate(&format!("homing.joints[{i}]"))?;
        }
        for (i, step) in self.sequence.iter().enumerate() {
            let prefix = format!("homing.sequence[{i}]");
            if step.pre_moves.is_empty()
                && step.home.is_none()
                && step.move_to.is_empty()
                && step.post_moves.is_empty()
            {
                return Err(invalid(
                    prefix,
                    "step has no pre_moves, home, move_to, or post_moves",
                ));
            }
            validate_moves(&step.pre_moves, num_joints, &format!("{prefix}.pre_moves"))?;
            validate_moves(
                &step.post_moves,
                num_joints,
                &format!("{prefix}.post_moves"),
            )?;
            if let Some(home) = &step.home {
                if home.joints.is_empty() && home.gripper.is_none() {
                    return Err(invalid(
                        format!("{prefix}.home"),
                        "home group names no joints and no gripper",
                    ));
                }
                for j in &home.joints {
                    if usize::from(*j) >= num_joints {
                        return Err(invalid(
                            format!("{prefix}.home.joints"),
                            format!("joint {j} out of range (robot has {num_joints} joints)"),
                        ));
                    }
                }
            }
            for (k, m) in step.move_to.iter().enumerate() {
                let f = format!("{prefix}.move_to[{k}]");
                if usize::from(m.joint) >= num_joints {
                    return Err(invalid(
                        format!("{f}.joint"),
                        format!(
                            "joint {} out of range (robot has {num_joints} joints)",
                            m.joint
                        ),
                    ));
                }
                if m.duration_s <= 0.0 {
                    return Err(invalid(format!("{f}.duration_s"), "must be > 0"));
                }
            }
        }
        // The sequence must reference every joint. One that skips a joint
        // (or is empty) still runs to completion, leaving the runtime
        // nothing to distinguish "finished" from "referenced" — an arm
        // reporting homed with an axis still on its boot-sector guess,
        // free to jog, stream and execute.
        let mut referenced = vec![false; num_joints];
        for step in &self.sequence {
            if let Some(home) = &step.home {
                for j in &home.joints {
                    referenced[usize::from(*j)] = true;
                }
            }
        }
        let missing: Vec<usize> = (0..num_joints).filter(|&i| !referenced[i]).collect();
        if !missing.is_empty() {
            return Err(invalid(
                "homing.sequence",
                format!("no step homes joint(s) {missing:?}; every joint must be referenced"),
            ));
        }
        validate_moves(&self.post_moves, num_joints, "homing.post_moves")?;
        Ok(())
    }
}

fn validate_moves(moves: &[PreMove], num_joints: usize, prefix: &str) -> Result<(), ConfigError> {
    for (k, m) in moves.iter().enumerate() {
        let f = format!("{prefix}[{k}]");
        let (joint, duration) = match m {
            PreMove::Idle { joint, duration_s } => (Some(*joint), *duration_s),
            PreMove::Nudge {
                joint, duration_s, ..
            } => (Some(*joint), *duration_s),
            PreMove::Position {
                joint, duration_s, ..
            } => (Some(*joint), *duration_s),
            PreMove::GripperMove { duration_s, .. } => (None, *duration_s),
        };
        if let Some(j) = joint {
            if usize::from(j) >= num_joints {
                return Err(invalid(
                    format!("{f}.joint"),
                    format!("joint {j} out of range (robot has {num_joints} joints)"),
                ));
            }
        }
        if duration <= 0.0 {
            return Err(invalid(format!("{f}.duration_s"), "must be > 0"));
        }
    }
    Ok(())
}
