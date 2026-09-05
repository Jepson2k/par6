//! The simulator's MuJoCo model, built from the vendor MJCF at load time.
//!
//! The vendor file (`assets/par6_description/PAR6_<tool>_gripper.xml`,
//! Source Robotics) is the base; everything par6 needs differently is an
//! explicit edit on the parsed `MjSpec`, so there is no forked copy to keep
//! in step with it:
//!
//! - meshes: the decimated `<part>_simplified.stl` variant of every mesh
//!   that has one;
//! - inertials: overwritten from the URDF, the mass-property source of
//!   truth (the vendor MJCF carries stale shell-only inertias — see the
//!   assets CHANGELOG);
//! - timestep: the largest step ≤ 1 ms that divides the bus tick;
//! - actuators: deleted — the plant drives every DOF through `qfrc_applied`;
//! - arm joints: armature / damping / frictionloss / limits from the robot
//!   config, replacing the vendor's single eyeballed class-`Y` tuning;
//! - contacts: off by default, on for the jaw pads, the floor, the pedestal
//!   and the grasp object;
//! - `Tool::Flange`: derived from the MSG spec — jaws and their coupling
//!   deleted, the wrist mesh swapped for the flange plate.

use std::path::{Path, PathBuf};
use std::sync::Mutex;

use mujoco_rs::prelude::{MjModel, MjSpec, MjtGeom, MjtJoint, SpecItem, SpecObject};
use mujoco_rs::wrappers::mj_editing::MjsGeom;
use par6_config::SimConfig;

use super::plant::JointMap;

/// MJCF joint names of the six arm joints, in config order.
pub const ARM_JOINTS: [&str; 6] = [
    "shoulder_JOINT",
    "upper_arm_JOINT",
    "elbow_JOINT",
    "lower_arm_JOINT",
    "wrist_JOINT",
    "gripper_JOINT",
];

/// Upper bound on the integration step \[s\]; the actual step is the
/// largest value at or under this that divides the bus tick exactly.
pub const MAX_TIMESTEP_S: f64 = 0.001;

/// The tool the scene is fitted with — the config's `urdf_variant`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tool {
    /// Bare mounting flange.
    Flange,
    /// MSG gripper (prismatic jaws).
    Msg,
    /// SSG48 gripper (prismatic jaws, shorter stroke).
    Ssg48,
}

impl Tool {
    /// The config's `urdf_variant` string; `None` for a variant with no
    /// scene.
    pub fn from_urdf_variant(variant: &str) -> Option<Self> {
        Some(match variant {
            "flange" => Tool::Flange,
            "msg" => Tool::Msg,
            "ssg48" => Tool::Ssg48,
            _ => return None,
        })
    }

    fn has_jaws(self) -> bool {
        !matches!(self, Tool::Flange)
    }
}

/// Which description tree to build from.
#[derive(Debug, Clone)]
pub struct Scene {
    /// The fitted tool.
    pub tool: Tool,
    /// The `par6_description` tree (vendor MJCFs, `assets/`, `URDF/`).
    pub assets: PathBuf,
}

/// Per-joint physics reflected through the drivetrain — what the torque
/// plant used to compute for itself, now MJCF joint attributes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointTuning {
    /// Reflected rotor inertia `G² · Jm` \[kg·m²\].
    pub armature: f64,
    /// Reflected viscous friction `G² · b` \[N·m·s\].
    pub damping: f64,
    /// Reflected Coulomb friction `G · tc` \[N·m\].
    pub frictionloss: f64,
    /// Config hard limits \[rad\].
    pub range: [f64; 2],
}

impl JointTuning {
    /// From the config's `[sim]` motor constants and one joint's map.
    pub(crate) fn from_config(map: &JointMap, motor_jm_kg_m2: f64, sim: &SimConfig) -> Self {
        let g = map.dyn_gear;
        Self {
            armature: g * g * motor_jm_kg_m2,
            damping: g * g * sim.motor_b_nm_s,
            frictionloss: g * sim.motor_tc_nm,
            range: [map.hard_lo_rad, map.hard_hi_rad],
        }
    }
}

/// Why a scene could not be built.
#[derive(Debug, thiserror::Error)]
pub enum SceneError {
    /// The vendor MJCF failed to parse or the edited spec to compile.
    #[error("MJCF {path}: {detail}")]
    Mjcf {
        /// The vendor file.
        path: PathBuf,
        /// MuJoCo's message.
        detail: String,
    },
    /// The URDF failed to read or parse.
    #[error("URDF {path}: {detail}")]
    Urdf {
        /// The URDF file.
        path: PathBuf,
        /// What was wrong.
        detail: String,
    },
    /// The vendor file lacks an element the edits rely on.
    #[error("vendor MJCF has no {kind} named {name:?}")]
    Missing {
        /// Element kind.
        kind: &'static str,
        /// Element name.
        name: String,
    },
}

/// The XML parser keeps global "last XML" state and is not thread-safe;
/// compiled models and per-instance data are.
static LOAD_LOCK: Mutex<()> = Mutex::new(());

/// The largest step at or under [`MAX_TIMESTEP_S`] that divides `dt`.
pub fn timestep_for(dt: f64) -> f64 {
    dt / (dt / MAX_TIMESTEP_S).ceil()
}

impl Scene {
    /// The vendor MJCF the spec starts from.
    pub fn vendor_mjcf(&self) -> PathBuf {
        let file = match self.tool {
            Tool::Msg | Tool::Flange => "PAR6_MSG_gripper.xml",
            Tool::Ssg48 => "PAR6_SSG48_gripper.xml",
        };
        self.assets.join(file)
    }

    /// The URDF whose inertials the bodies take.
    pub fn urdf(&self) -> PathBuf {
        let rel = match self.tool {
            Tool::Flange => "URDF/par6_flange/urdf/par6_flange.urdf",
            Tool::Msg => "URDF/par6_msg_gripper/urdf/PAR6_MSG.urdf",
            Tool::Ssg48 => "URDF/par6_ssg48_gripper/urdf/par6_ssg48_urdf.urdf",
        };
        self.assets.join(rel)
    }

    /// The mesh directory (vendor STLs and their `_simplified` variants).
    pub fn meshes(&self) -> PathBuf {
        self.assets.join("assets")
    }

    /// The edited spec: vendor base plus every par6 delta.
    pub fn spec(&self, timestep: f64, joints: &[JointTuning]) -> Result<MjSpec, SceneError> {
        let path = self.vendor_mjcf();
        let mjcf = |detail: String| SceneError::Mjcf {
            path: path.clone(),
            detail,
        };
        let mut spec = {
            let _guard = LOAD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            MjSpec::from_xml(&path).map_err(|e| mjcf(e.to_string()))?
        };
        let meshdir = self.meshes();
        let meshdir = meshdir.canonicalize().unwrap_or(meshdir);
        spec.compiler_mut()
            .set_meshdir(meshdir.to_str().expect("mesh dir is UTF-8"));
        spec.option_mut().timestep = timestep;

        // Every DOF is driven through qfrc_applied.
        for actuator in spec.actuator_iter_mut() {
            // SAFETY: the iterator reads the next element before yielding
            // this one, so deleting the handle it holds is safe.
            unsafe { actuator.delete() }.map_err(|e| mjcf(e.to_string()))?;
        }

        for (name, tuning) in ARM_JOINTS.iter().zip(joints) {
            let joint = spec.joint_mut(name).ok_or_else(|| SceneError::Missing {
                kind: "joint",
                name: (*name).to_owned(),
            })?;
            joint.set_armature(tuning.armature);
            joint.set_frictionloss(tuning.frictionloss);
            let mut damping = *joint.damping();
            damping.fill(0.0);
            damping[0] = tuning.damping;
            joint.with_damping(damping);
            joint.with_range(tuning.range);
            // The config hard limits are the plant's endstops: a stiff,
            // critically damped limit constraint (reference time two
            // substeps) that admits sub-milliradian penetration at the
            // homing currents.
            *joint.solref_limit_mut() = [2.0 * timestep, 1.0];
            *joint.solimp_limit_mut() = [0.95, 0.99, 0.001, 0.5, 2.0];
            // The drivetrain friction (set per substep by the plant) must
            // hold, not creep: at MuJoCo's default impedance a held joint
            // leaks ~5 % of its free-fall acceleration, which is degrees
            // per second under the shoulder's load.
            *joint.solref_friction_mut() = [2.0 * timestep, 1.0];
            *joint.solimp_friction_mut() = [0.9999, 0.9999, 0.001, 0.5, 2.0];
        }

        if self.tool == Tool::Flange {
            // Derived from the MSG spec: no jaws, no jaw coupling, flange
            // plate in place of the gripper body's mesh.
            for jaw in ["jaw1", "jaw2"] {
                let body = spec.body_mut(jaw).ok_or_else(|| SceneError::Missing {
                    kind: "body",
                    name: jaw.to_owned(),
                })?;
                // SAFETY: a live body looked up by name, deleted once; its
                // subtree (joint, geom) goes with it.
                unsafe { body.delete() }.map_err(|e| mjcf(e.to_string()))?;
            }
            for equality in spec.equality_iter_mut() {
                // SAFETY: as for the actuators above.
                unsafe { equality.delete() }.map_err(|e| mjcf(e.to_string()))?;
            }
            spec.mesh_mut("gripper")
                .ok_or_else(|| SceneError::Missing {
                    kind: "mesh",
                    name: "gripper".to_owned(),
                })?
                .set_file("gripper_flange.STL");
        }

        // Contacts are opt-in: nothing collides unless named below.
        for geom in spec.geom_iter_mut() {
            let jaw = matches!(geom.meshname(), "jaw1" | "jaw2");
            geom.set_contype(i32::from(jaw));
            geom.set_conaffinity(i32::from(jaw));
            if jaw {
                pad_contact(geom);
            }
        }
        if self.tool.has_jaws() {
            let exclude = spec.add_exclude();
            exclude.with_name("jaw1_jaw2");
            exclude.set_bodyname1("jaw1");
            exclude.set_bodyname2("jaw2");
        }

        // The grasp scene: floor, pedestal, and a free object above it under
        // the reach-down test pose q = [0, -0.25, 4.35, 0, -1.28, 0].
        let world = spec.world_body_mut();
        world
            .add_geom()
            .with_name("floor")
            .with_type(MjtGeom::mjGEOM_PLANE)
            .with_size([0.0, 0.0, 0.05])
            .with_rgba([0.3, 0.35, 0.4, 1.0])
            .with_contype(1)
            .with_conaffinity(1);
        world
            .add_geom()
            .with_name("pedestal")
            .with_type(MjtGeom::mjGEOM_BOX)
            .with_pos([0.3713, 0.0, 0.005])
            .with_size([0.02, 0.02, 0.005])
            .with_rgba([0.4, 0.4, 0.45, 1.0])
            .with_contype(1)
            .with_conaffinity(1);
        let object = world
            .add_body()
            .with_name("grasp_object")
            .with_pos([0.3713, 0.0, 0.04]);
        object
            .add_joint()
            .with_name("grasp_object_JOINT")
            .with_type(MjtJoint::mjJNT_FREE);
        let geom = object
            .add_geom()
            .with_name("grasp_object")
            .with_type(MjtGeom::mjGEOM_BOX)
            .with_size([0.018, 0.018, 0.03])
            .with_rgba([0.85, 0.55, 0.2, 1.0])
            .with_contype(1)
            .with_conaffinity(1);
        geom.set_mass(0.05);
        pad_contact(geom);

        apply_urdf_inertials(&mut spec, &self.urdf())?;
        // The vendor's debug bodies — a camera frame and a 1 m TCP marker
        // capsule (alpha 0) — have no <inertial>, so MuJoCo would weigh
        // them by volume: ~0.3 kg hanging 0.22 m below the wrist, 0.4 Nm
        // of phantom shoulder torque. They are frames, not parts.
        for name in ["camera_center", "tcp_link"] {
            if let Some(body) = spec.body_mut(name) {
                body.set_mass(0.0);
                body.with_inertia([0.0; 3]);
                body.set_explicitinertial(true);
            }
        }
        prefer_simplified_meshes(&mut spec, &meshdir);
        Ok(spec)
    }

    /// The compiled model.
    pub fn model(&self, timestep: f64, joints: &[JointTuning]) -> Result<MjModel, SceneError> {
        let mut spec = self.spec(timestep, joints)?;
        let _guard = LOAD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        spec.compile().map_err(|e| SceneError::Mjcf {
            path: self.vendor_mjcf(),
            detail: e.to_string(),
        })
    }
}

/// The vendor's jaw-pad contact tuning, shared with the grasp object so a
/// grasp is symmetric.
fn pad_contact(geom: &mut MjsGeom) {
    geom.with_friction([1.2, 0.3, 0.001]);
    let mut solref = *geom.solref();
    solref[0] = 0.002;
    solref[1] = 1.0;
    geom.with_solref(solref);
    let mut solimp = *geom.solimp();
    solimp[0] = 0.9;
    solimp[1] = 0.95;
    solimp[2] = 0.001;
    geom.with_solimp(solimp);
}

/// Every mesh with a decimated `<stem>_simplified.stl` beside it uses that.
fn prefer_simplified_meshes(spec: &mut MjSpec, meshdir: &Path) {
    for mesh in spec.mesh_iter_mut() {
        let file = mesh.file().to_owned();
        let Some(stem) = Path::new(&file).file_stem().and_then(|s| s.to_str()) else {
            continue;
        };
        let simplified = format!("{stem}_simplified.stl");
        if meshdir.join(&simplified).is_file() {
            mesh.set_file(&simplified);
        }
    }
}

/// One URDF `<inertial>`: mass, centre of mass and the inertia tensor
/// expressed in the body frame.
#[derive(Debug, Clone, PartialEq)]
pub struct UrdfInertial {
    /// URDF link name (== MJCF body name).
    pub link: String,
    /// Mass \[kg\].
    pub mass: f64,
    /// Centre of mass in the link frame \[m\].
    pub com: [f64; 3],
    /// Inertia about the centre of mass, in the link frame:
    /// `[ixx, iyy, izz, ixy, ixz, iyz]` \[kg·m²\] (MJCF `fullinertia` order).
    pub inertia: [f64; 6],
}

/// Parse every `<link>` that declares an `<inertial>`.
pub fn urdf_inertials(urdf: &Path) -> Result<Vec<UrdfInertial>, SceneError> {
    let err = |detail: String| SceneError::Urdf {
        path: urdf.to_path_buf(),
        detail,
    };
    let text = std::fs::read_to_string(urdf).map_err(|e| err(e.to_string()))?;
    let doc = roxmltree::Document::parse(&text).map_err(|e| err(e.to_string()))?;
    let floats = |s: &str| -> Result<Vec<f64>, SceneError> {
        s.split_whitespace()
            .map(|v| v.parse::<f64>().map_err(|e| err(format!("{v:?}: {e}"))))
            .collect()
    };
    let mut out = Vec::new();
    for link in doc.descendants().filter(|n| n.has_tag_name("link")) {
        let Some(inertial) = link.children().find(|n| n.has_tag_name("inertial")) else {
            continue;
        };
        let name = link
            .attribute("name")
            .ok_or_else(|| err("link without name".into()))?;
        let child = |tag: &str| {
            inertial
                .children()
                .find(|n| n.has_tag_name(tag))
                .ok_or_else(|| err(format!("link {name:?}: inertial without <{tag}>")))
        };
        let mass_el = child("mass")?;
        let mass: f64 = mass_el
            .attribute("value")
            .ok_or_else(|| err(format!("link {name:?}: mass without value")))?
            .parse()
            .map_err(|e| err(format!("link {name:?}: mass: {e}")))?;
        let (com, rpy) = match inertial.children().find(|n| n.has_tag_name("origin")) {
            Some(o) => (
                floats(o.attribute("xyz").unwrap_or("0 0 0"))?,
                floats(o.attribute("rpy").unwrap_or("0 0 0"))?,
            ),
            None => (vec![0.0; 3], vec![0.0; 3]),
        };
        if com.len() != 3 || rpy.len() != 3 {
            return Err(err(format!("link {name:?}: origin needs 3 xyz and 3 rpy")));
        }
        let inertia_el = child("inertia")?;
        let mut i = [0.0; 6];
        for (slot, key) in i.iter_mut().zip(["ixx", "iyy", "izz", "ixy", "ixz", "iyz"]) {
            *slot = inertia_el
                .attribute(key)
                .ok_or_else(|| err(format!("link {name:?}: inertia without {key}")))?
                .parse()
                .map_err(|e| err(format!("link {name:?}: {key}: {e}")))?;
        }
        out.push(UrdfInertial {
            link: name.to_owned(),
            mass,
            com: [com[0], com[1], com[2]],
            inertia: rotate_inertia(i, [rpy[0], rpy[1], rpy[2]]),
        });
    }
    Ok(out)
}

/// A tensor given in a frame rotated by URDF `rpy` (fixed-axis roll, pitch,
/// yaw: `R = Rz·Ry·Rx`), re-expressed in the link frame: `R·I·Rᵀ`.
fn rotate_inertia(i: [f64; 6], rpy: [f64; 3]) -> [f64; 6] {
    if rpy == [0.0; 3] {
        return i;
    }
    let (sr, cr) = rpy[0].sin_cos();
    let (sp, cp) = rpy[1].sin_cos();
    let (sy, cy) = rpy[2].sin_cos();
    let r = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ];
    let m = [[i[0], i[3], i[4]], [i[3], i[1], i[5]], [i[4], i[5], i[2]]];
    let mut rm = [[0.0; 3]; 3];
    for a in 0..3 {
        for b in 0..3 {
            rm[a][b] = (0..3).map(|k| r[a][k] * m[k][b]).sum();
        }
    }
    let mut out = [[0.0; 3]; 3];
    for a in 0..3 {
        for b in 0..3 {
            out[a][b] = (0..3).map(|k| rm[a][k] * r[b][k]).sum();
        }
    }
    [
        out[0][0], out[1][1], out[2][2], out[0][1], out[0][2], out[1][2],
    ]
}

/// Overwrite every body's mass properties with the URDF's. Links without
/// a body (the static base, the massless `tcp` frame) are skipped.
fn apply_urdf_inertials(spec: &mut MjSpec, urdf: &Path) -> Result<(), SceneError> {
    for inertial in urdf_inertials(urdf)? {
        let Some(body) = spec.body_mut(&inertial.link) else {
            continue;
        };
        // MuJoCo stores an inertial frame plus principal moments; handing it
        // exactly that avoids its full-vs-diagonal exclusivity rule.
        let (moments, iquat) = principal_axes(inertial.inertia);
        body.set_mass(inertial.mass);
        body.with_ipos(inertial.com);
        body.with_iquat(iquat);
        body.with_inertia(moments);
        body.set_explicitinertial(true);
    }
    Ok(())
}

/// Principal moments and the inertial-frame quaternion `[w, x, y, z]` of a
/// symmetric tensor `[ixx, iyy, izz, ixy, ixz, iyz]`: cyclic Jacobi
/// rotations until the off-diagonals vanish, the eigenvectors forming a
/// right-handed frame.
// Each Jacobi update touches two columns (or rows) of the same matrix by
// the loop index; iterator forms would obscure the textbook step.
#[allow(clippy::needless_range_loop)]
fn principal_axes(i: [f64; 6]) -> ([f64; 3], [f64; 4]) {
    let mut a = [[i[0], i[3], i[4]], [i[3], i[1], i[5]], [i[4], i[5], i[2]]];
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..50 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        if off < 1e-18 {
            break;
        }
        for (p, q) in [(0usize, 1usize), (0, 2), (1, 2)] {
            if a[p][q].abs() < 1e-300 {
                continue;
            }
            let theta = 0.5 * (a[q][q] - a[p][p]) / a[p][q];
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for k in 0..3 {
                let (akp, akq) = (a[k][p], a[k][q]);
                a[k][p] = c * akp - s * akq;
                a[k][q] = s * akp + c * akq;
            }
            for k in 0..3 {
                let (apk, aqk) = (a[p][k], a[q][k]);
                a[p][k] = c * apk - s * aqk;
                a[q][k] = s * apk + c * aqk;
            }
            for k in 0..3 {
                let (vkp, vkq) = (v[k][p], v[k][q]);
                v[k][p] = c * vkp - s * vkq;
                v[k][q] = s * vkp + c * vkq;
            }
        }
    }
    // Columns of `v` are the principal axes; make the frame right-handed.
    let det = v[0][0] * (v[1][1] * v[2][2] - v[1][2] * v[2][1])
        - v[0][1] * (v[1][0] * v[2][2] - v[1][2] * v[2][0])
        + v[0][2] * (v[1][0] * v[2][1] - v[1][1] * v[2][0]);
    if det < 0.0 {
        for row in v.iter_mut() {
            row[2] = -row[2];
        }
    }
    let moments = [a[0][0], a[1][1], a[2][2]];
    // Rotation matrix (columns = axes) → quaternion, Shepperd's method.
    let (r00, r11, r22) = (v[0][0], v[1][1], v[2][2]);
    let trace = r00 + r11 + r22;
    let q = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        [
            0.25 * s,
            (v[2][1] - v[1][2]) / s,
            (v[0][2] - v[2][0]) / s,
            (v[1][0] - v[0][1]) / s,
        ]
    } else if r00 > r11 && r00 > r22 {
        let s = (1.0 + r00 - r11 - r22).sqrt() * 2.0;
        [
            (v[2][1] - v[1][2]) / s,
            0.25 * s,
            (v[0][1] + v[1][0]) / s,
            (v[0][2] + v[2][0]) / s,
        ]
    } else if r11 > r22 {
        let s = (1.0 + r11 - r00 - r22).sqrt() * 2.0;
        [
            (v[0][2] - v[2][0]) / s,
            (v[0][1] + v[1][0]) / s,
            0.25 * s,
            (v[1][2] + v[2][1]) / s,
        ]
    } else {
        let s = (1.0 + r22 - r00 - r11).sqrt() * 2.0;
        [
            (v[1][0] - v[0][1]) / s,
            (v[0][2] + v[2][0]) / s,
            (v[1][2] + v[2][1]) / s,
            0.25 * s,
        ]
    };
    let norm = q.iter().map(|x| x * x).sum::<f64>().sqrt();
    (moments, q.map(|x| x / norm))
}
