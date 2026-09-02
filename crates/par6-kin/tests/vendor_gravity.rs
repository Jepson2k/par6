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
#![cfg(feature = "ffi")]

use std::path::PathBuf;

use par6_kin::gravity::{self, GravitySample};
use par6_kin::{Kin, NQ};
use serde::Deserialize;

#[derive(Deserialize)]
struct Fixture {
    provenance: String,
    cases: Vec<Case>,
}

#[derive(Deserialize)]
struct Case {
    q: [f64; NQ],
    /// Arm alone, massless tool stub — what `par6_arm.urdf` models.
    tau_arm: [f64; NQ],
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
    let text = std::fs::read_to_string(&path).expect("vendor reference");
    serde_json::from_str(&text).expect("vendor reference parses")
}

#[test]
fn the_shipped_arm_model_is_the_vendors_arm() {
    let fx = fixture();
    assert!(fx.provenance.contains("vendor"), "{}", fx.provenance);

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
