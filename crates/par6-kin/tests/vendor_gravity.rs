//! Does identification from measured torque recover the arm the VENDOR
//! describes?
//!
//! Every other gravity test here is self-referential: the model against
//! another reading of the same URDF. This one is not. The fixture is
//! per-joint `G(q)` derived from the vendor runtime's own mass/COM table
//! by a static-torque computation over the vendor DH chain, touching no
//! URDF at all — so it says what the real arm's gravity load is,
//! independently of what our model believes.
//!
//! Feeding those torques to `gravity::fit` as if they had been measured
//! on the arm answers the question the calibration exists to answer: run
//! on real hardware, does it land on the vendor's arm?
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

/// `par6-calibrate-gravity --prior-weight`'s default.
const PRIOR_WEIGHT: f64 = 0.01;

/// The arm's travel, as the calibration's own pose planner sees it.
const WINDOW: [(f64, f64); NQ] = [
    (-2.5, 2.5),
    (-1.5, 0.3),
    (1.5, 4.5),
    (-2.5, 2.5),
    (-1.4, 1.4),
    (-3.0, 3.0),
];

/// How far each centre of mass is moved to make a wrong arm \[m\].
const SHIFT_M: f64 = 0.02;

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
fn identification_from_measured_torque_lands_on_the_vendors_arm() {
    let fx = fixture();
    assert!(fx.provenance.contains("vendor"), "{}", fx.provenance);
    assert!(fx.cases.len() >= 6, "too few vendor poses to split");

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
    let holdout = samples.len() / 3;
    let (train, held) = samples.split_at(samples.len() - holdout);

    let truth = gravity::model_params(&kin).expect("model parameters");
    let theta_truth = gravity::flatten(&truth);
    let shipped = gravity::rms(&mut kin, &theta_truth, held).expect("rms");
    println!(
        "vendor poses: {} train, {} held out",
        train.len(),
        held.len()
    );
    println!("shipped URDF vs vendor, held-out poses: {shipped:.2e} Nm");

    // Start from an arm whose centres of mass are wrong by 2 cm — the
    // scale of a real modelling error, and far more than the shipped
    // model is out by. This is the arm a calibration would be run on.
    let wrong: Vec<gravity::BodyParams> = truth
        .iter()
        .enumerate()
        .map(|(i, b)| {
            let c = b.com();
            let s = if i % 2 == 0 { 1.0 } else { -1.0 };
            gravity::BodyParams {
                joint: b.joint.clone(),
                mass: b.mass,
                first_moment: [
                    b.mass * (c[0] + s * SHIFT_M),
                    b.mass * (c[1] - s * SHIFT_M),
                    b.mass * (c[2] + s * SHIFT_M),
                ],
            }
        })
        .collect();
    let theta_wrong = gravity::flatten(&wrong);
    let before = gravity::rms(&mut kin, &theta_wrong, held).expect("rms");

    // The ridge pulls toward the model the arm is CARRYING, which in a
    // real run is the wrong one — that is the honest starting point.
    let fit = gravity::fit(&mut kin, train, &wrong, PRIOR_WEIGHT).expect("fit");
    let theta = gravity::flatten(&fit.bodies);
    let after = gravity::rms(&mut kin, &theta, held).expect("rms");

    println!("held-out RMS vs vendor: wrong model {before:.4} Nm -> identified {after:.4} Nm");
    for ((w, f), t) in wrong.iter().zip(&fit.bodies).zip(&truth) {
        let (cw, cf, ct) = (w.com(), f.com(), t.com());
        let was = (0..3).map(|k| (cw[k] - ct[k]).abs()).fold(0.0, f64::max);
        let now = (0..3).map(|k| (cf[k] - ct[k]).abs()).fold(0.0, f64::max);
        println!("  {:<16} com error {was:.4} m -> {now:.4} m", t.joint);
    }
    for (i, s) in held.iter().enumerate() {
        let p = gravity::predict(&mut kin, &theta, &s.q).expect("predict");
        let worst = (0..NQ).map(|j| (p[j] - s.tau[j]).abs()).fold(0.0, f64::max);
        println!("  held-out pose {i}: worst joint error {worst:.4} Nm");
    }

    // The COMs it did not recover: do they cost anything? Gravity torque
    // is a linear map of the first moments with a null space — some
    // directions produce zero torque at EVERY configuration — so a
    // component the fit left alone is either one it corrected or one
    // that contributes nothing to compensate. Swept across the whole
    // joint window rather than the vendor's ten poses, which is the
    // difference between "predicts the training set" and "describes the
    // arm".
    let sweep = gravity::calibration_poses(&WINDOW, 500, 20260902);
    let mut worst_fit = 0.0f64;
    let mut worst_wrong = 0.0f64;
    for q in &sweep {
        let truth_tau = gravity::predict(&mut kin, &theta_truth, q).expect("predict");
        let fit_tau = gravity::predict(&mut kin, &theta, q).expect("predict");
        let wrong_tau = gravity::predict(&mut kin, &theta_wrong, q).expect("predict");
        for j in 0..NQ {
            worst_fit = worst_fit.max((fit_tau[j] - truth_tau[j]).abs());
            worst_wrong = worst_wrong.max((wrong_tau[j] - truth_tau[j]).abs());
        }
    }
    println!(
        "across {} poses spanning the joint window: wrong model is out by up to \
         {worst_wrong:.4} Nm, the identified one by {worst_fit:.4} Nm",
        sweep.len()
    );

    assert!(
        worst_fit < 0.05 * worst_wrong,
        "identification must hold across the workspace, not just the fitted poses: \
         {worst_fit:.4} Nm against {worst_wrong:.4} Nm"
    );
    assert!(
        shipped < 1e-3,
        "the shipped URDF should already agree with the vendor: {shipped:.4} Nm"
    );
    assert!(
        after < 0.1 * before,
        "identification from vendor torques must recover the vendor's arm: \
         {after:.4} Nm against {before:.4} Nm before"
    );
}
