//! The gravity identification contract: the regressor is the
//! linear-in-parameters form of the model's own G(q), a fit from static
//! torques predicts the torques at poses it never saw, and what the
//! writer puts in the URDF is what a model loaded from it computes.
#![cfg(feature = "ffi")]
// Joint values are spelled the way config/PAR6.toml spells them.
#![allow(clippy::approx_constant)]

use std::path::PathBuf;

use par6_kin::gravity::{self, GravitySample};
use par6_kin::{GripperVariant, Kin, NQ};

fn assets_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/par6_description")
        .canonicalize()
        .unwrap()
}

const CASES: [[f64; NQ]; 5] = [
    [0.0, -1.5708, 3.1416, 0.0, 0.0, 3.1416],
    [1.2, -1.2708, 3.7416, 0.0, 0.5, 0.0],
    [-2.007, -0.698, 3.491, 0.0, 1.047, 3.1416],
    [0.5, -1.0, 2.6, 0.3, 0.8, 2.5],
    [-0.8, -1.3, 3.3, -0.6, -0.7, 3.6],
];

/// A conservative slice of the arm's travel for sampling poses.
const WINDOW: [(f64, f64); NQ] = [
    (-2.5, 2.5),
    (-1.5, 0.3),
    (1.5, 4.5),
    (-2.5, 2.5),
    (-1.4, 1.4),
    (-3.0, 3.0),
];

/// A heavier tool than any shipped gripper, so the tool's share of the
/// payload body is far from zero in every check below.
fn heavy_tool() -> pinokin_sys::ToolParams {
    Kin::dh_tool_params(
        0.12,
        0.0,
        0.0,
        0.9,
        [0.02, -0.01, 0.05],
        [1e-3, 0.0, 1e-3, 0.0, 0.0, 1e-3],
    )
}

fn max_abs_diff(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

/// `Y(q) θ_model` is `G(q)` for the gripper variants and for the arm
/// chain with a tool attached — the identification form and RNEA agree.
#[test]
fn the_regressor_is_the_linear_form_of_the_gravity_model() {
    let tool = heavy_tool();
    let mut models: Vec<(String, Kin)> = GripperVariant::ALL
        .iter()
        .map(|v| (format!("{v:?}"), Kin::load(&assets_dir(), *v).unwrap()))
        .collect();
    models.push((
        "arm+tool".into(),
        Kin::load_arm(&assets_dir(), Some(&tool)).unwrap(),
    ));
    for (name, kin) in models.iter_mut() {
        let theta = gravity::flatten(&gravity::model_params(kin).unwrap());
        assert_eq!(theta.len(), 4 * kin.body_count(), "{name}");
        let mut tau = [0.0; NQ];
        for q in &CASES {
            kin.gravity(q, &mut tau).unwrap();
            let predicted = gravity::predict(kin, &theta, q).unwrap();
            assert!(
                max_abs_diff(&predicted, &tau) < 1e-9,
                "{name}: Y·θ = {predicted:?} vs G(q) = {tau:?}"
            );
        }
        // The last body carries the tool: its share is what a write-back
        // subtracts, and it must be a proper part of the composite.
        let tool_share = kin.tool_inertial().unwrap();
        let last = &gravity::model_params(kin).unwrap()[kin.body_count() - 1];
        assert!(
            tool_share[0] < last.mass + 1e-12,
            "{name}: tool mass {} exceeds the payload body {}",
            tool_share[0],
            last.mass
        );
    }
}

/// From exact static torques at a spread of poses, a fit started from
/// centres of mass that are all three centimetres out recovers them and
/// predicts the torques at held-out poses; the prior does not. Bodies the
/// pose set cannot excite are reported as such and keep the prior instead
/// of drifting.
#[test]
fn a_fit_from_static_torques_predicts_held_out_poses() {
    const SHIFT_M: f64 = 0.03;
    let tool = heavy_tool();
    let mut kin = Kin::load_arm(&assets_dir(), Some(&tool)).unwrap();
    let truth = gravity::model_params(&kin).unwrap();
    let theta_true = gravity::flatten(&truth);
    let poses = gravity::calibration_poses(&WINDOW, 40, 7);
    let samples: Vec<GravitySample> = poses
        .iter()
        .map(|q| GravitySample {
            q: *q,
            tau: gravity::predict(&mut kin, &theta_true, q).unwrap(),
        })
        .collect();
    let (train, held) = samples.split_at(30);

    // Masses are not fitted, so the prior keeps them and moves only where
    // each centre of mass sits.
    let prior: Vec<_> = truth
        .iter()
        .map(|b| gravity::BodyParams {
            joint: b.joint.clone(),
            mass: b.mass,
            first_moment: [
                b.first_moment[0] + b.mass * SHIFT_M,
                b.first_moment[1] - b.mass * SHIFT_M,
                b.first_moment[2] + b.mass * SHIFT_M,
            ],
        })
        .collect();
    let fit = gravity::fit(&mut kin, train, &prior, 1e-9).unwrap();
    let theta = gravity::flatten(&fit.bodies);
    let held_prior = gravity::rms(&mut kin, &gravity::flatten(&prior), held).unwrap();
    let held_fit = gravity::rms(&mut kin, &theta, held).unwrap();
    assert!(
        held_prior > 0.1,
        "centres of mass {SHIFT_M} m out must miss by more than {held_prior} Nm"
    );
    assert!(
        held_fit < 1e-7,
        "the fit must predict unseen poses: {held_fit} Nm (prior {held_prior} Nm)"
    );
    assert!(fit.rms_fit_nm < 1e-7 && fit.rms_prior_nm > 0.1);
    assert_eq!(fit.bodies.len(), truth.len());

    // Masses come back untouched. Whatever the DATA determined lands on
    // the truth; whatever it did not keeps the prior. Nothing is asserted
    // in between, because a partly-determined parameter is legitimately
    // somewhere between the two.
    for ((got, want), excite) in fit.bodies.iter().zip(&truth).zip(&fit.determined) {
        assert_eq!(got.joint, want.joint);
        assert_eq!(got.mass, want.mass, "{}: mass must be held", want.joint);
        let prior_h = prior.iter().find(|p| p.joint == want.joint).unwrap();
        for (axis, share) in excite.iter().enumerate() {
            if *share > 0.99 {
                assert!(
                    (got.first_moment[axis] - want.first_moment[axis]).abs() < 1e-9,
                    "{} axis {axis}: excited component must land on the truth, {} vs {}",
                    want.joint,
                    got.first_moment[axis],
                    want.first_moment[axis]
                );
            } else if *share < 0.01 {
                assert!(
                    (got.first_moment[axis] - prior_h.first_moment[axis]).abs() < 1e-9,
                    "{} axis {axis}: an undetermined component must keep the prior",
                    want.joint
                );
            }
        }
    }
    assert!(
        fit.determined[0].iter().all(|e| *e < 0.01),
        "the first body cannot be seen by gravity, got {:?}",
        fit.determined[0]
    );
}

/// The centre of mass written into the arm URDF is what a model loaded
/// from that file then computes gravity with, and nothing else in the
/// file moves.
#[test]
fn written_inertials_are_what_the_model_reads_back() {
    let urdf_path = assets_dir().join(Kin::ARM_URDF_RELPATH);
    let text = std::fs::read_to_string(&urdf_path).unwrap();
    let kin = Kin::load_arm(&assets_dir(), None).unwrap();
    let current = gravity::model_params(&kin).unwrap();

    // Every centre of mass shifted, so an unchanged file cannot pass. The
    // chain ends in a massless stub, which has no centre of mass to place
    // and is left out.
    let changed: Vec<_> = current
        .iter()
        .filter(|b| b.mass > 0.0)
        .enumerate()
        .map(|(i, b)| {
            let com = b.com();
            let shift = 0.005 * (i as f64 + 1.0);
            gravity::BodyParams {
                joint: b.joint.clone(),
                mass: b.mass,
                first_moment: [
                    b.mass * (com[0] + shift),
                    b.mass * (com[1] - shift),
                    b.mass * (com[2] + 2.0 * shift),
                ],
            }
        })
        .collect();
    assert!(
        changed.len() + 1 == current.len(),
        "one massless stub is skipped"
    );
    let rewritten = gravity::rewrite_inertials(&text, &changed).unwrap();
    assert_ne!(rewritten, text);
    assert_eq!(
        rewritten.matches("<link").count(),
        text.matches("<link").count(),
        "only inertial origins change"
    );
    assert_eq!(
        rewritten.matches("<mass").count(),
        text.matches("<mass").count()
    );
    for mass in text.split("<mass").skip(1) {
        assert!(
            rewritten.contains(&mass[..mass.find('>').unwrap()]),
            "a mass value changed: {}",
            &mass[..mass.find('>').unwrap()]
        );
    }

    let dir = std::env::temp_dir().join(format!("par6-gravity-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let tmp = dir.join("par6_arm.urdf");
    std::fs::write(&tmp, &rewritten).unwrap();
    let reloaded = Kin::from_urdf(&tmp, Some(Kin::ARM_EE_FRAME)).unwrap();
    let back = gravity::model_params(&reloaded).unwrap();
    for (w, r) in changed.iter().zip(&back) {
        assert_eq!(w.joint, r.joint);
        assert_eq!(w.mass, r.mass, "{}: mass must survive untouched", w.joint);
        assert!(
            max_abs_diff(&w.first_moment, &r.first_moment) < 1e-9,
            "{}: wrote {:?}, model read back {:?}",
            w.joint,
            w.first_moment,
            r.first_moment
        );
    }

    // Gravity follows what was written.
    let mut reloaded = reloaded;
    let theta = gravity::flatten(&back);
    let mut tau = [0.0; NQ];
    for q in &CASES {
        reloaded.gravity(q, &mut tau).unwrap();
        let predicted = gravity::predict(&mut reloaded, &theta, q).unwrap();
        assert!(
            max_abs_diff(&predicted, &tau) < 1e-9,
            "reloaded G(q) {tau:?} vs written parameters {predicted:?}"
        );
    }

    // The writer refuses what a model could not carry.
    let mut massless = changed.clone();
    massless[2].mass = 0.0;
    assert!(gravity::rewrite_inertials(&text, &massless).is_err());
    let mut unknown = changed.clone();
    unknown[0].joint = "no_such_joint".into();
    assert!(gravity::rewrite_inertials(&text, &unknown).is_err());
}
