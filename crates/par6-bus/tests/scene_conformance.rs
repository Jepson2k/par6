//! The MuJoCo scene is the vendor MJCF plus par6's deltas. This pins that
//! what comes out describes the arm the URDF describes (mass properties)
//! and the arm the vendor's own dynamics table describes (gravity torque),
//! for every tool variant — so a stale vendor inertial, a mis-rotated
//! tensor or a frame drift in the MJCF chain fails here and not in a grasp
//! test's tolerance.
#![cfg(feature = "sim-mujoco")]

use std::path::PathBuf;

use mujoco_rs::prelude::*;
use par6_bus::sim::scene::{urdf_inertials, JointTuning, Scene, Tool, ARM_JOINTS};
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
            range: [-6.28, 6.28],
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

fn model(tool: Tool) -> MjModel {
    scene(tool)
        .model(0.001, &tuning())
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
        assert!(
            m.name_to_id(MjtObj::mjOBJ_BODY, "grasp_object").is_some(),
            "{tool:?}: grasp scene present"
        );

        let spec = scene(tool).spec(0.001, &tuning()).expect("spec");
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
        let m = sc.model(0.001, &tuning()).expect("model");
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
