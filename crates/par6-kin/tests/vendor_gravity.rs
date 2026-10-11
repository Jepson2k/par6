//! Does the shipped model describe the real arm?
//!
//! Every other gravity test here is self-referential: the model against
//! another reading of the same URDF, so a URDF whose inertials drift
//! from the arm passes all of them — which is how a SolidWorks export
//! carrying 2.375 kg of moving mass shipped against the vendor's
//! 5.114 kg and nothing noticed.
//!
//! The fixture is per-joint `G(q)` derived from the vendor runtime's own
//! mass/COM table by a static-torque computation over the vendor DH
//! chain, touching no URDF at all. It is the authority for the arm's
//! link inertials, and it is the only thing here that can fail when the
//! model stops describing the arm.
//!
//! It cannot be replaced by measuring the arm. Gravity does not observe
//! every inertial parameter — nothing about the first body of a
//! vertical-axis arm, nor the component of a first moment along its own
//! joint axis — so an identification run would correct the observable
//! directions, leave the rest wrong, and report a good residual either
//! way. Anything that physically changes a link needs new nominal data,
//! not a measurement. What identification IS for is the load at the
//! tool, which no table can describe: `par6_kin::gravity::fit_payload`.

use std::path::PathBuf;

use par6_kin::gravity::{self, GravitySample};
use par6_kin::{GripperVariant, Kin, NQ};
use serde::Deserialize;

mod common;
use common::assets_dir;

#[derive(Deserialize)]
struct Fixture {
    tools: Tools,
    cases: Vec<Case>,
}

/// Keyed by the vendor gripper file each entry was read from, which is
/// also the name of the config file that must carry the same values.
#[derive(Deserialize)]
struct Tools {
    #[serde(rename = "MSG_small_motor_150mm_rail")]
    msg_small_150: ToolEntry,
    #[serde(rename = "SSG48")]
    ssg48: ToolEntry,
}

/// The vendor DH tool description, spelled as a gripper config's
/// `[kinematics]` table.
#[derive(Deserialize, Debug, PartialEq)]
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
    /// Arm alone, massless tool stub — what `par6_arm.urdf` models.
    tau_arm: [f64; NQ],
    /// The Flange VARIANT tree: arm plus the vendor flange plate.
    tau_flange_variant: [f64; NQ],
    /// The arm carrying each vendor gripper as a DH tool.
    tau_arm_msg_small_motor_150mm_rail_tool: [f64; NQ],
    tau_arm_ssg48_tool: [f64; NQ],
}

fn fixture() -> Fixture {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/golden/gravity/vendor_reference.json");
    let text = std::fs::read_to_string(&path).expect("vendor reference");
    serde_json::from_str(&text).expect("vendor reference parses")
}

#[test]
fn the_shipped_arm_model_is_the_vendors_arm() {
    let fx = fixture();

    let mut kin = Kin::from_urdf(
        &assets_dir().join(Kin::ARM_URDF_RELPATH),
        Some(Kin::ARM_EE_FRAME),
    )
    .expect("arm model");

    let samples: Vec<GravitySample> = fx
        .cases
        .iter()
        .map(|c| GravitySample {
            q: c.q,
            tau: c.tau_arm,
        })
        .collect();

    let theta = gravity::flatten(&gravity::model_params(&kin).expect("model parameters"));
    let mut worst = 0.0f64;
    for s in &samples {
        let tau = gravity::predict(&mut kin, &theta, &s.q).expect("predict");
        for (got, want) in tau.iter().zip(&s.tau) {
            worst = worst.max((got - want).abs());
        }
    }
    let residual = gravity::rms(&mut kin, &theta, &samples).expect("rms");
    println!(
        "shipped arm model vs the vendor over {} poses: {residual:.3e} Nm rms, \
         worst joint {worst:.3e} Nm",
        samples.len()
    );

    // Agreement at generation time is fixture rounding. The defects this
    // guards against start at ~1e-2 Nm (a tool mass slip) and reach Nm
    // scale (a simplified URDF), so this leaves orders of margin on both
    // sides while still failing the moment the model stops being the
    // vendor's arm.
    assert!(
        worst < 1e-6,
        "the shipped URDF no longer describes the vendor's arm: worst joint {worst:.4e} Nm \
         over {} poses. The link inertials are nominal data — fix them from CAD or the \
         vendor table, not by measuring the arm.",
        samples.len()
    );
}

/// The shipped gripper config `name`'s `[kinematics]`, as the vendor's
/// DH tool description.
fn shipped_tool(name: &str) -> ToolEntry {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join(format!("../../config/grippers/{name}.toml"));
    let k = par6_config::ToolConfig::load(&path)
        .unwrap_or_else(|e| panic!("{name}: {e}"))
        .kinematics;
    ToolEntry {
        d_m: k.d_m,
        a_m: k.a_m,
        alpha_rad: k.alpha_rad,
        mass_kg: k.mass_kg,
        com_m: k.com_m,
        inertia_kg_m2: k.inertia_kg_m2,
    }
}

/// The tool paths: a variant tree with its plate, and the shipped
/// gripper configs attached as DH tools at load. Each is a different
/// composition of the same arm, and the vendor computed the load for all
/// of them — a tool mass slip in a config lands here and nowhere else.
#[test]
fn the_shipped_tool_compositions_are_the_vendors_too() {
    let fx = fixture();
    let dh = |t: &ToolEntry| {
        Kin::dh_tool_params(
            t.d_m,
            t.a_m,
            t.alpha_rad,
            t.mass_kg,
            t.com_m,
            t.inertia_kg_m2,
        )
    };
    let msg_small_150 = shipped_tool("MSG_small_motor_150mm_rail");
    let ssg48 = shipped_tool("SSG48");
    assert_eq!(msg_small_150, fx.tools.msg_small_150);
    assert_eq!(ssg48, fx.tools.ssg48);
    let mut flange = Kin::load(&assets_dir(), GripperVariant::Flange).expect("flange variant");
    let mut msg = Kin::load_arm(&assets_dir(), Some(&dh(&msg_small_150))).expect("arm + MSG");
    let mut ssg = Kin::load_arm(&assets_dir(), Some(&dh(&ssg48))).expect("arm + SSG48");

    let mut worst = [
        ("flange variant", 0.0f64),
        ("arm + MSG_small_motor_150mm_rail tool", 0.0),
        ("arm + SSG48 tool", 0.0),
    ];
    for c in &fx.cases {
        for (slot, (kin, want)) in [
            (&mut flange, &c.tau_flange_variant),
            (&mut msg, &c.tau_arm_msg_small_motor_150mm_rail_tool),
            (&mut ssg, &c.tau_arm_ssg48_tool),
        ]
        .into_iter()
        .enumerate()
        {
            let mut got = [0.0; NQ];
            kin.gravity(&c.q, &mut got).expect("gravity");
            for (g, w) in got.iter().zip(want) {
                worst[slot].1 = worst[slot].1.max((g - w).abs());
            }
        }
    }
    for (name, err) in worst {
        println!("{name}: worst joint {err:.3e} Nm against the vendor");
        assert!(
            err < 1e-6,
            "{name} no longer matches the vendor's load: worst joint {err:.4e} Nm"
        );
    }
}
