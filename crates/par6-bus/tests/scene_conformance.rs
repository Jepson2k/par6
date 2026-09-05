//! The MuJoCo scene is the vendor MJCF plus par6's deltas. This pins that
//! what comes out describes the arm the URDF describes (mass properties)
//! and the arm the vendor's own dynamics table describes (gravity torque),
//! for every tool variant — so a stale vendor inertial, a mis-rotated
//! tensor or a frame drift in the MJCF chain fails here and not in a grasp
//! test's tolerance.
use std::path::PathBuf;

use mujoco_rs::prelude::*;
use par6_bus::sim::scene::{
    inject_world, urdf_inertials, Build, JointTuning, Scene, Tool, ToolInertial, ARM_JOINTS,
    WORLD_PREFIX,
};
use par6_proto::{Physical, Shape};
use serde::Deserialize;

fn assets() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/par6_description")
        .canonicalize()
        .expect("assets tree")
}

/// Any plausible tuning: the config derivation is the bus's job and its
/// values do not enter what is checked here.
fn tuning() -> Vec<JointTuning> {
    vec![
        JointTuning {
            armature: 4e-3,
            damping: 0.04,
            frictionloss: 0.4,
            range: [-std::f64::consts::TAU, std::f64::consts::TAU],
        };
        6
    ]
}

fn scene(tool: Tool) -> Scene {
    Scene {
        tool,
        assets: assets(),
    }
}

/// A build with the shipped floor height and no config tool.
fn build(joints: &[JointTuning]) -> Build<'_> {
    Build {
        timestep: 0.001,
        joints,
        tool: None,
        floor_z_m: Some(0.0),
    }
}

fn model(tool: Tool) -> MjModel {
    let tuning = tuning();
    scene(tool)
        .model(&build(&tuning), &[&[], &[]])
        .unwrap_or_else(|e| panic!("{tool:?}: {e}"))
}

#[test]
fn every_variant_compiles_with_the_expected_joints_and_decimated_meshes() {
    for (tool, jaws) in [
        (Tool::Msg, true),
        (Tool::Ssg48, true),
        (Tool::Flange, false),
    ] {
        let m = model(tool);
        for (i, name) in ARM_JOINTS.iter().enumerate() {
            assert_eq!(
                m.name_to_id(MjtObj::mjOBJ_JOINT, name),
                Some(i),
                "{tool:?}: arm joint {name} must be scene joint {i}"
            );
        }
        assert_eq!(
            m.name_to_id(MjtObj::mjOBJ_JOINT, "jaw1_JOINT").is_some(),
            jaws,
            "{tool:?}: jaw joints"
        );
        assert_eq!(
            m.ffi().nu,
            0,
            "{tool:?}: the plant drives qfrc_applied, not actuators"
        );
        assert_eq!(
            m.ffi().nq as usize,
            ARM_JOINTS.len() + if jaws { 2 } else { 0 },
            "{tool:?}: the base scene carries no world objects"
        );

        let tuning = tuning();
        let spec = scene(tool).spec(&build(&tuning)).expect("spec");
        for mesh in spec.mesh_iter() {
            assert!(
                mesh.file().ends_with("_simplified.stl"),
                "{tool:?}: mesh {:?} is not the decimated variant",
                mesh.file()
            );
        }
    }
}

#[test]
fn bodies_carry_the_urdf_mass_properties() {
    for tool in [Tool::Msg, Tool::Ssg48, Tool::Flange] {
        let sc = scene(tool);
        let tuning = tuning();
        let m = sc.model(&build(&tuning), &[&[], &[]]).expect("model");
        let inertials = urdf_inertials(&sc.urdf()).expect("urdf");
        let mut matched = 0;
        for want in &inertials {
            let Some(id) = m.name_to_id(MjtObj::mjOBJ_BODY, &want.link) else {
                continue;
            };
            matched += 1;
            let got_mass = m.body_mass()[id];
            assert!(
                (got_mass - want.mass).abs() < 1e-9,
                "{tool:?} {}: mass {got_mass} vs URDF {}",
                want.link,
                want.mass
            );
            for k in 0..3 {
                assert!(
                    (m.body_ipos()[id][k] - want.com[k]).abs() < 1e-9,
                    "{tool:?} {}: com[{k}] {} vs URDF {}",
                    want.link,
                    m.body_ipos()[id][k],
                    want.com[k]
                );
            }
            // Principal moments are a rotation of the URDF tensor, so the
            // trace is invariant.
            let trace: f64 = m.body_inertia()[id].iter().sum();
            let want_trace = want.inertia[0] + want.inertia[1] + want.inertia[2];
            assert!(
                (trace - want_trace).abs() < 1e-9,
                "{tool:?} {}: inertia trace {trace} vs URDF {want_trace}",
                want.link
            );
        }
        assert!(
            matched >= 6,
            "{tool:?}: only {matched} URDF links matched a body"
        );
    }
}

#[derive(Deserialize)]
struct Fixture {
    cases: Vec<Case>,
}

#[derive(Deserialize)]
struct Case {
    q: [f64; 6],
    tau_flange_variant: [f64; 6],
}

/// The vendor table is independent of any URDF and of MuJoCo; par6-kin's
/// gravity_reference test holds the URDF to it at 1e-6 Nm. The MJCF
/// chain's quaternions carry ~8 digits, worth ~1e-6 Nm at the shoulder,
/// so 1e-4 leaves margin while still catching a tool-mass slip (~1e-2).
const GRAVITY_TOL_NM: f64 = 1e-4;

#[test]
fn flange_variant_gravity_matches_the_vendor_dynamics_table() {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../par6-kin/tests/golden/gravity/vendor_reference.json");
    let fx: Fixture = serde_json::from_str(&std::fs::read_to_string(&path).expect("fixture"))
        .expect("fixture schema");
    let m = model(Tool::Flange);
    let mut data = m.make_data();
    let mut worst = 0.0f64;
    for (i, case) in fx.cases.iter().enumerate() {
        data.reset();
        data.qpos_mut()[..6].copy_from_slice(&case.q);
        data.forward();
        let tau = &data.qfrc_bias()[..6];
        for (j, (g, w)) in tau.iter().zip(case.tau_flange_variant.iter()).enumerate() {
            let diff = (g - w).abs();
            worst = worst.max(diff);
            assert!(
                diff <= GRAVITY_TOL_NM,
                "case {i} joint {j}: MuJoCo G(q) = {g:.6} Nm, vendor table {w:.6} Nm (diff {diff:.3e})"
            );
        }
    }
    println!(
        "flange-variant gravity: worst |diff| = {worst:.3e} Nm over {} cases",
        fx.cases.len()
    );
}

/// The configured gearbox holding friction covers gravity everywhere the
/// arm reaches with the heaviest tool fitted, so an IDLE joint never
/// back-drives under its own weight. Sampled over the shoulder/elbow box
/// with the wrist at its extremes, where the tool's lever arm is longest.
#[test]
fn holding_friction_covers_gravity_across_the_reachable_envelope() {
    /// Required margin over the worst measured gravity torque.
    const MARGIN: f64 = 1.1;
    let robot = par6_config::RobotConfig::load(
        &PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml"),
    )
    .expect("PAR6.toml");
    let hold = &robot.sim.holding_friction_nm;
    let m = model(Tool::Msg);
    let mut data = m.make_data();
    let grid = |j: usize, n: usize| -> Vec<f64> {
        let (lo, hi) = (
            robot.joints[j].limits.hard_min_rad,
            robot.joints[j].limits.hard_max_rad,
        );
        (0..n)
            .map(|i| lo + (hi - lo) * i as f64 / (n - 1) as f64)
            .collect()
    };
    let mut worst = [0.0f64; 6];
    for &q1 in &grid(1, 25) {
        for &q2 in &grid(2, 25) {
            for &q3 in &grid(3, 3) {
                for &q4 in &grid(4, 3) {
                    data.qpos_mut()[..6].copy_from_slice(&[0.0, q1, q2, q3, q4, 0.0]);
                    data.forward();
                    for (w, g) in worst.iter_mut().zip(&data.qfrc_bias()[..6]) {
                        *w = w.max(g.abs());
                    }
                }
            }
        }
    }
    for (j, (w, h)) in worst.iter().zip(hold).enumerate() {
        eprintln!("J{j}: worst |G| {w:.3} Nm, holding friction {h:.3} Nm");
    }
    for (j, (w, h)) in worst.iter().zip(hold).enumerate() {
        assert!(
            *h >= MARGIN * w,
            "J{j}: holding friction {h} Nm does not cover {w:.3} Nm of gravity with a {MARGIN}x margin"
        );
    }
}

/// With the config tool applied, the tool the scene swings is the one the
/// runtime's gravity model carries: the tool subtree's mass and COM are
/// the config `[kinematics]` values (composed through the same DH tool
/// frame), jaws included. A scene weighing the URDF's gripper links
/// instead disagrees with G(q) by a tenth of a newton-metre at the
/// wrist, which a feedforward-held joint creeps on.
#[test]
fn the_tool_subtree_carries_the_config_inertials() {
    let cfg = par6_config::GripperConfig::load(
        &PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../config/grippers/MSG_small_motor_150mm_rail.toml"),
    )
    .expect("MSG gripper TOML");
    let k = &cfg.kinematics;
    let tool = ToolInertial {
        d_m: k.d_m,
        a_m: k.a_m,
        alpha_rad: k.alpha_rad,
        mass_kg: k.mass_kg,
        com_m: k.com_m,
        inertia_kg_m2: k.inertia_kg_m2,
    };
    let tuning = tuning();
    let m = scene(Tool::Msg)
        .model(
            &Build {
                tool: Some(&tool),
                ..build(&tuning)
            },
            &[&[], &[]],
        )
        .expect("MSG scene with the config tool");
    let gripper = m
        .name_to_id(MjtObj::mjOBJ_BODY, "gripper")
        .expect("gripper body");
    let mut data = m.make_data();
    data.forward();
    assert!(
        (m.body_subtreemass()[gripper] - k.mass_kg).abs() < 1e-9,
        "tool subtree mass {} kg vs config {} kg",
        m.body_subtreemass()[gripper],
        k.mass_kg
    );
    // The config COM, DH-composed into the flange frame, then into the
    // world through the gripper body's pose.
    let (s, c) = k.alpha_rad.sin_cos();
    let r = [
        k.com_m[0],
        c * k.com_m[1] - s * k.com_m[2],
        s * k.com_m[1] + c * k.com_m[2],
    ];
    let com_flange = [r[0] + k.a_m, r[1], r[2] + k.d_m];
    let xpos = data.xpos()[gripper];
    let xmat = data.xmat()[gripper];
    let want: [f64; 3] = std::array::from_fn(|i| {
        xpos[i] + (0..3).map(|j| xmat[3 * i + j] * com_flange[j]).sum::<f64>()
    });
    let got = data.subtree_com()[gripper];
    for i in 0..3 {
        assert!(
            (got[i] - want[i]).abs() < 1e-6,
            "tool subtree COM {got:?} vs config {want:?} (axis {i})"
        );
    }
}

/// Every world shape enters the scene as its declaration says: coal's
/// constructor sizes become MuJoCo's, the pose rotation is the extrinsic
/// XYZ one, contact is on only for `physics` shapes, and only a shape with
/// mass gets a free body.
#[test]
fn world_shapes_enter_the_scene_as_declared() {
    let physical = |mass: Option<f64>| {
        Some(Physical {
            mass,
            friction: [0.8, 0.01, 0.001],
        })
    };
    let shape =
        |name: &str, kind: &str, params: &[f64], pose: [f64; 6], physics, collision| Shape {
            kind: kind.to_owned(),
            params: params.to_vec(),
            pose: pose.to_vec(),
            collision,
            margin: None,
            name: name.to_owned(),
            physics,
        };
    let world = vec![
        shape(
            "crate",
            "box",
            &[0.2, 0.4, 0.6],
            [1.0, 2.0, 3.0, 0.0, 0.0, 0.0],
            physical(Some(2.0)),
            true,
        ),
        shape(
            "ball",
            "sphere",
            &[0.05],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            physical(None),
            true,
        ),
        shape(
            "post",
            "cylinder",
            &[0.02, 0.5],
            [0.5, 0.0, 0.25, 0.0, 0.0, 0.0],
            None,
            true,
        ),
        shape(
            "bar",
            "capsule",
            &[0.01, 0.3],
            [0.0, 0.5, 0.5, 0.0, 0.0, 0.0],
            None,
            false,
        ),
        shape(
            "egg",
            "ellipsoid",
            &[0.1, 0.2, 0.3],
            [0.0, -0.5, 0.5, 0.0, 0.0, 0.0],
            physical(None),
            true,
        ),
        // A wall: solid where x >= 0.8, i.e. normal -x, offset -0.8.
        shape(
            "wall",
            "plane",
            &[-1.0, 0.0, 0.0, -0.8],
            [0.0; 6],
            physical(None),
            true,
        ),
        // Rotated 90° about z: the box's x side ends up along world y.
        shape(
            "turned",
            "box",
            &[0.2, 0.4, 0.6],
            [0.0, 0.0, 0.0, 0.0, 0.0, std::f64::consts::FRAC_PI_2],
            None,
            true,
        ),
    ];
    let tuning = tuning();
    let mut spec = scene(Tool::Msg).spec(&build(&tuning)).expect("spec");
    inject_world(&mut spec, &[&world, &[]]).expect("inject");
    let m = spec.compile().expect("compile");
    let geom = |name: &str| {
        m.name_to_id(MjtObj::mjOBJ_GEOM, &format!("{WORLD_PREFIX}{name}"))
            .expect(name)
    };
    let expect = [
        ("crate", MjtGeom::mjGEOM_BOX, [0.1, 0.2, 0.3], 1),
        ("ball", MjtGeom::mjGEOM_SPHERE, [0.05, 0.0, 0.0], 1),
        ("post", MjtGeom::mjGEOM_CYLINDER, [0.02, 0.25, 0.0], 0),
        ("bar", MjtGeom::mjGEOM_CAPSULE, [0.01, 0.15, 0.0], 0),
        ("egg", MjtGeom::mjGEOM_ELLIPSOID, [0.1, 0.2, 0.3], 1),
        ("wall", MjtGeom::mjGEOM_PLANE, [0.0, 0.0, 0.05], 1),
    ];
    for (name, kind, size, contype) in expect {
        let g = geom(name);
        assert_eq!(m.geom_type()[g], kind, "{name}: geom type");
        for (k, (got, want)) in m.geom_size()[g].iter().zip(&size).enumerate() {
            assert!((got - want).abs() < 1e-12, "{name}: size {k} = {got}");
        }
        assert_eq!(m.geom_contype()[g], contype, "{name}: contype");
        assert_eq!(m.geom_conaffinity()[g], contype, "{name}: conaffinity");
    }
    // Only the massed shape is a free body, at its declared pose and mass.
    let free: Vec<usize> = (0..m.ffi().njnt as usize)
        .filter(|&j| m.jnt_type()[j] == MjtJoint::mjJNT_FREE)
        .collect();
    assert_eq!(free.len(), 1, "exactly one free body");
    let body = m
        .name_to_id(MjtObj::mjOBJ_BODY, &format!("{WORLD_PREFIX}crate"))
        .expect("crate body");
    assert!(
        (m.body_mass()[body] - 2.0).abs() < 1e-12,
        "crate mass {}",
        m.body_mass()[body]
    );
    let mut data = m.make_data();
    data.forward();
    let p = data.xpos()[body];
    assert!(
        (p[0] - 1.0).abs() < 1e-12 && (p[1] - 2.0).abs() < 1e-12 && (p[2] - 3.0).abs() < 1e-12,
        "crate at {p:?}"
    );
    // The wall's plane normal (local +z) points along -x, through x = 0.8.
    let wall = geom("wall");
    let n = data.geom_xmat()[wall];
    assert!(
        (n[2] + 1.0).abs() < 1e-9 && n[5].abs() < 1e-9 && n[8].abs() < 1e-9,
        "wall normal {:?}",
        [n[2], n[5], n[8]]
    );
    assert!(
        (data.geom_xpos()[wall][0] - 0.8).abs() < 1e-9,
        "wall at x {}",
        data.geom_xpos()[wall][0]
    );
    // The turned box: local x (0.1 half side) along world y.
    let turned = geom("turned");
    let r = data.geom_xmat()[turned];
    assert!(
        r[3].abs() > 0.999 && r[0].abs() < 1e-9,
        "turned box rotation {r:?}"
    );
}
