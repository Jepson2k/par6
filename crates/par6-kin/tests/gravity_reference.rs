//! Gravity ground truth: G(q) against the VENDOR's dynamics table, not
//! against another reading of the same URDF.
//!
//! Every other gravity test in this repo is self-referential — Rust vs
//! Python over the same URDF, sim plant vs gravity model over the same
//! URDF — so a URDF whose inertials drift from the real arm passes all
//! of them (the SolidWorks export shipped 2.375 kg of moving mass
//! against the vendor's 5.114 kg, and nothing noticed). The fixture here
//! (`tests/golden/gravity/vendor_reference.json`) is derived from the
//! vendor runtime's own per-link mass/COM table by an independent
//! static-torque computation over the vendor DH chain, cross-checked
//! against torque values printed in the vendor's own model file. It
//! never touches a URDF, so these assertions fail loudly whenever the
//! shipped model's mass distribution stops describing the arm.
//!
//! Pre-fix failure: against the original SolidWorks inertials this test
//! fails with per-joint errors up to 2.7 Nm (~50% of the shoulder load).
#![cfg(feature = "ffi")]

use std::path::PathBuf;

use serde::Deserialize;

use par6_kin::{GripperVariant, Kin, NQ};

/// Fixture agreement measured at generation time is ~2e-12 Nm (fixture
/// rounding); the defects this guards against start at ~1e-2 Nm (a tool
/// mass slip) and reach Nm scale (a simplified URDF). 1e-6 keeps six
/// orders of margin on both sides.
const TOL: f64 = 1e-6;

#[derive(Deserialize)]
struct Fixture {
    #[allow(dead_code)]
    provenance: String,
    tools: Tools,
    cases: Vec<Case>,
}

#[derive(Deserialize)]
struct Tools {
    #[serde(rename = "MSG")]
    msg: ToolEntry,
    #[serde(rename = "SSG48")]
    ssg48: ToolEntry,
    #[allow(dead_code)]
    #[serde(rename = "Flange")]
    flange: ToolEntry,
}

/// The vendor DH tool description, spelled exactly like the
/// `[kinematics]` table of a gripper config.
#[derive(Deserialize)]
struct ToolEntry {
    d_m: f64,
    a_m: f64,
    alpha_rad: f64,
    mass_kg: f64,
    com_m: [f64; 3],
    inertia_kg_m2: [f64; 6],
}

#[derive(Deserialize)]
struct Case {
    q: [f64; NQ],
    /// Arm alone (massless tool stub).
    tau_arm: [f64; NQ],
    /// The flange VARIANT URDF — arm plus the vendor Flange plate the
    /// torque-level sim plant swings.
    tau_flange_variant: [f64; NQ],
    /// Arm plus the MSG tool attached through `dh_tool_params`.
    tau_arm_msg_tool: [f64; NQ],
    /// Arm plus the SSG48 tool attached through `dh_tool_params`.
    tau_arm_ssg48_tool: [f64; NQ],
}

fn assets_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/par6_description")
        .canonicalize()
        .unwrap()
}

fn fixture() -> Fixture {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/golden/gravity/vendor_reference.json");
    serde_json::from_str(&std::fs::read_to_string(&path).expect("fixture file"))
        .expect("fixture schema")
}

fn tool_params(t: &ToolEntry) -> pinokin_sys::ToolParams {
    Kin::dh_tool_params(
        t.d_m,
        t.a_m,
        t.alpha_rad,
        t.mass_kg,
        t.com_m,
        t.inertia_kg_m2,
    )
}

fn assert_gravity(label: &str, kin: &mut Kin, cases: &[Case], pick: impl Fn(&Case) -> [f64; NQ]) {
    let mut tau = [0.0; NQ];
    for (i, case) in cases.iter().enumerate() {
        kin.gravity(&case.q, &mut tau).expect("gravity");
        let want = pick(case);
        for (j, (g, w)) in tau.iter().zip(want.iter()).enumerate() {
            assert!(
                (g - w).abs() <= TOL,
                "{label}[case {i}] joint {j}: G(q) = {g:.9} Nm, vendor table says {w:.9} Nm \
                 (diff {:.3e}); the URDF's mass distribution no longer describes the arm",
                (g - w).abs()
            );
        }
    }
}

/// The arm-only chain and the flange variant (the body the torque-level
/// sim plant swings) both reproduce the vendor table.
#[test]
fn urdf_gravity_matches_the_vendor_dynamics_table() {
    let fx = fixture();
    let mut arm = Kin::load_arm(&assets_dir(), None).expect("arm-only model");
    assert_gravity("arm", &mut arm, &fx.cases, |c| c.tau_arm);

    let mut flange = Kin::load(&assets_dir(), GripperVariant::Flange).expect("flange variant");
    assert_gravity("flange-variant", &mut flange, &fx.cases, |c| {
        c.tau_flange_variant
    });
}

/// Config-sourced tool inertials through `dh_tool_params` reproduce the
/// vendor model with that tool link — the whole conversion chain (DH
/// tool frame -> ee frame, inertia reordering, attachment to the wrist
/// joint) against an external reference. MSG has a non-zero DH `a`, so a
/// wrong translation or rotation order in the conversion fails here.
#[test]
fn config_tool_inertials_reproduce_the_vendor_tool_link() {
    let fx = fixture();
    let dir = assets_dir();

    let msg = tool_params(&fx.tools.msg);
    let mut kin = Kin::load_arm(&dir, Some(&msg)).expect("arm + MSG tool");
    assert_gravity("arm+MSG", &mut kin, &fx.cases, |c| c.tau_arm_msg_tool);

    let ssg = tool_params(&fx.tools.ssg48);
    let mut kin = Kin::load_arm(&dir, Some(&ssg)).expect("arm + SSG48 tool");
    assert_gravity("arm+SSG48", &mut kin, &fx.cases, |c| c.tau_arm_ssg48_tool);

    // The tool is the difference between the two models: dropping it must
    // change the wrist load by the tool's own weight, so a ToolParams
    // that silently fails to attach cannot pass the assertions above by
    // matching the arm-only column.
    let case = &fx.cases[0];
    let (mut with_tool, mut without) = ([0.0; NQ], [0.0; NQ]);
    kin.gravity(&case.q, &mut with_tool).unwrap();
    Kin::load_arm(&dir, None)
        .unwrap()
        .gravity(&case.q, &mut without)
        .unwrap();
    let delta: f64 = with_tool
        .iter()
        .zip(without.iter())
        .map(|(a, b)| (a - b).abs())
        .sum();
    assert!(
        delta > 0.05,
        "attaching the SSG48 tool changed G(q) by only {delta:.4} Nm across all joints"
    );
}
