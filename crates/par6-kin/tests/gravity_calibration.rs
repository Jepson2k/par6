//! The payload identification contract: the regressor is the
//! linear-in-parameters form of the model's own G(q), and a fit from
//! static torques recovers a load the model was not carrying.
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

/// A heavier tool than any shipped gripper, so the tool's share of the
/// payload body is far from zero in every check below.
fn heavy_tool() -> par6_kin::ToolParams {
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
/// The wrist poses a payload identification actually uses: the arm stays
/// where it stands and only the last three joints swing.
fn wrist_poses(start: [f64; NQ], spread: f64) -> Vec<[f64; NQ]> {
    let mut out = vec![start];
    for j in [3usize, 4, 5] {
        for dir in [1.0, -1.0] {
            let mut q = start;
            q[j] += dir * spread;
            out.push(q);
        }
    }
    out
}

#[test]
fn a_payload_is_recovered_from_the_torque_the_arm_cannot_explain() {
    // The model the runtime carries: the arm, and nothing in the hand.
    let mut unloaded = Kin::load_arm(&assets_dir(), None).unwrap();

    const MASS: f64 = 1.35;
    const COM: [f64; 3] = [0.012, -0.028, 0.061];

    for (name, start, spread) in [
        (
            "reaching out",
            [-2.007, -0.698, 3.491, 0.0, 1.047, 3.1416],
            0.5,
        ),
        ("folded up", [0.5, -1.0, 2.6, 0.3, 0.8, 2.5], 0.5),
    ] {
        // What the sensors report: the arm actually carrying the part,
        // through RNEA. The identification runs on the analytic
        // regressor, so measuring with it too would make the fit invert
        // the very matrix that produced its input.
        unloaded.set_tool(MASS, COM, None).unwrap();
        let samples: Vec<GravitySample> = wrist_poses(start, spread)
            .into_iter()
            .map(|q| {
                let mut tau = [0.0; NQ];
                unloaded.gravity(&q, &mut tau).unwrap();
                GravitySample { q, tau }
            })
            .collect();
        // The part is put down before the fit: what it must recover is
        // the difference between the torque it was handed and the torque
        // the model it holds can account for.
        unloaded.set_tool(0.0, [0.0; 3], None).unwrap();

        let fit = gravity::fit_payload(&mut unloaded, &samples, 1e-6).unwrap();
        assert!(
            (fit.mass - MASS).abs() < 0.01,
            "{name}: identified {:.4} kg against {MASS} kg carried",
            fit.mass
        );
        assert!(
            max_abs_diff(&fit.com, &COM) < 0.005,
            "{name}: identified com {:?} against {COM:?} carried",
            fit.com
        );
        // Not zero: the ridge biases the solution slightly even at
        // 1e-6, and what is left is a tenth of a milli-newton-metre.
        assert!(
            fit.rms_nm < 1e-3,
            "{name}: the fit must explain the torque it was given, {:.2e} Nm left",
            fit.rms_nm
        );
        assert!(
            fit.rms_unloaded_nm > 0.5,
            "{name}: a 1.35 kg payload must be visible in the torque at all, \
             only {:.4} Nm of it showed",
            fit.rms_unloaded_nm
        );
        assert!(
            fit.determined.iter().all(|d| *d > 0.5),
            "{name}: swinging the wrist must measure all four parameters, got {:?}",
            fit.determined
        );
    }
}

/// One-milli-newton-metre-scale torque error is what a current-sense
/// estimate carries; a fit that turns it into a payload would declare a
/// part in an empty gripper.
const TORQUE_NOISE_NM: f64 = 0.05;

/// A deterministic sign-varying sequence in [-1, 1] — the point is a
/// reproducible non-zero residual, not statistical realism.
fn noise_seq() -> impl FnMut() -> f64 {
    let mut n: u64 = 0x9E37_79B9_7F4A_7C15;
    move || {
        n ^= n << 13;
        n ^= n >> 7;
        n ^= n << 17;
        ((n >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
    }
}

#[test]
fn an_empty_hand_identifies_as_empty_and_a_still_wrist_says_so() {
    let tool = heavy_tool();
    let mut kin = Kin::load_arm(&assets_dir(), Some(&tool)).unwrap();
    let theta = gravity::flatten(&gravity::model_params(&kin).unwrap());
    let start = [-2.007, -0.698, 3.491, 0.0, 1.047, 3.1416];

    // Carrying nothing, measured the way the runtime measures: RNEA for
    // the true torque, plus the sensing error that never cancels. The
    // sweep is the same one that recovers 1.35 kg above, so the fit has
    // every chance to attribute the noise to a payload.
    let mut noise = noise_seq();
    let empty: Vec<GravitySample> = wrist_poses(start, 0.5)
        .into_iter()
        .map(|q| {
            let mut tau = [0.0; NQ];
            kin.gravity(&q, &mut tau).unwrap();
            for t in tau.iter_mut() {
                *t += TORQUE_NOISE_NM * noise();
            }
            GravitySample { q, tau }
        })
        .collect();
    let fit = gravity::fit_payload(&mut kin, &empty, 1e-6).unwrap();
    assert!(
        fit.mass.abs() < 0.05,
        "an empty hand must identify as empty, got {:.4} kg out of \
         {TORQUE_NOISE_NM} Nm of noise",
        fit.mass
    );
    // And it must be empty for the right reason. `calibrate::estimate`
    // refuses on `determined[0]`, so a sweep that DID separate the four
    // parameters and simply found no mass has to look different from one
    // that could not measure a mass at all — the still wrist below.
    assert!(
        fit.determined[0] > 0.9,
        "the swept poses must measure the mass they found to be zero, got {:?}",
        fit.determined
    );

    // A wrist that never moved gives the same lever arm every time, so
    // the parameters are not separable — and `determined` has to say so
    // rather than the fit inventing a split.
    let still: Vec<GravitySample> = std::iter::repeat_n(start, 5)
        .map(|q| GravitySample {
            q,
            tau: gravity::predict(&mut kin, &theta, &q).unwrap(),
        })
        .collect();
    let fit = gravity::fit_payload(&mut kin, &still, 1e-3).unwrap();
    assert!(
        fit.determined.iter().any(|d| *d < 0.5),
        "a wrist held still cannot measure four parameters, yet reported {:?}",
        fit.determined
    );
}

#[test]
fn the_payload_fit_refuses_what_it_cannot_use() {
    let mut kin = Kin::load_arm(&assets_dir(), None).unwrap();
    assert!(gravity::fit_payload(&mut kin, &[], 0.01).is_err());
    let one = [GravitySample {
        q: [0.0, -1.5708, 3.1416, 0.0, 0.0, 3.1416],
        tau: [0.0; NQ],
    }];
    assert!(gravity::fit_payload(&mut kin, &one, -1.0).is_err());
    assert!(gravity::fit_payload(&mut kin, &one, f64::NAN).is_err());
}

#[test]
fn a_declared_payload_changes_the_gravity_the_arm_holds() {
    // The wire's SET_PAYLOAD ends at `Kin::set_tool`, and everything
    // between is plumbing that has been tested by asserting the command
    // ARRIVED. Arriving is not the property: an arm told it is carrying
    // 1.35 kg and still compensating for an empty hand sags under the
    // load, with the command acked all the way back to the caller.
    let mut kin = Kin::load_arm(&assets_dir(), None).unwrap();
    let unloaded = gravity::flatten(&gravity::model_params(&kin).unwrap());

    const MASS: f64 = 1.35;
    const COM: [f64; 3] = [0.012, -0.028, 0.061];

    // What the model SHOULD compute once it carries the load: the same
    // parameters with the payload's mass and first moment added to the
    // body at the end of the chain.
    let mut loaded = unloaded.clone();
    let base = loaded.len() - 4;
    loaded[base] += MASS;
    for k in 0..3 {
        loaded[base + 1 + k] += MASS * COM[k];
    }

    for q in &CASES {
        let want_empty = gravity::predict(&mut kin, &unloaded, q).unwrap();
        let want_loaded = gravity::predict(&mut kin, &loaded, q).unwrap();

        let mut got = [0.0; NQ];
        kin.gravity(q, &mut got).unwrap();
        assert!(
            max_abs_diff(&got, &want_empty) < 1e-9,
            "empty hand: {got:?} vs {want_empty:?}"
        );

        kin.set_tool(MASS, COM, None).unwrap();
        kin.gravity(q, &mut got).unwrap();
        assert!(
            max_abs_diff(&got, &want_loaded) < 1e-9,
            "carrying {MASS} kg at {COM:?}: gravity {got:?} against the {want_loaded:?} \
             a model holding that load computes"
        );

        // And the load comes off again: a part put down must not keep
        // being compensated for.
        kin.set_tool(0.0, [0.0; 3], None).unwrap();
        kin.gravity(q, &mut got).unwrap();
        assert!(
            max_abs_diff(&got, &want_empty) < 1e-9,
            "payload cleared: {got:?} vs the empty-hand {want_empty:?}"
        );
    }
}
