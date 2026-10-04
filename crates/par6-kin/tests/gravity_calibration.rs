//! The payload identification contract: the regressor is the
//! linear-in-parameters form of the model's own G(q), and a fit from
//! static torques recovers a load the model was not carrying.
// Joint values are spelled the way config/PAR6.toml spells them.
#![allow(clippy::approx_constant)]

use par6_kin::gravity::{self, GravitySample};
use par6_kin::{GripperVariant, Kin, NQ};

mod common;
use common::assets_dir;

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
    let mut unit = common::xorshift(0x9E37_79B9_7F4A_7C15);
    move || unit() * 2.0 - 1.0
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

/// The fit refuses what it cannot use — no samples, a ridge that is not
/// a non-negative number, a torque that is not one — on a pose set it
/// otherwise identifies a payload from.
#[test]
fn the_payload_fit_refuses_what_it_cannot_use() {
    let mut kin = Kin::load_arm(&assets_dir(), None).unwrap();
    kin.set_tool(0.8, [0.01, -0.02, 0.05], None).unwrap();
    let samples: Vec<GravitySample> = wrist_poses([-2.007, -0.698, 3.491, 0.0, 1.047, 3.1416], 0.5)
        .into_iter()
        .map(|q| {
            let mut tau = [0.0; NQ];
            kin.gravity(&q, &mut tau).unwrap();
            GravitySample { q, tau }
        })
        .collect();
    kin.set_tool(0.0, [0.0; 3], None).unwrap();
    let fit = gravity::fit_payload(&mut kin, &samples, 1e-6).expect("the control fits");
    assert!(
        (fit.mass - 0.8).abs() < 0.01,
        "the control: {} kg",
        fit.mass
    );

    assert!(gravity::fit_payload(&mut kin, &[], 0.01).is_err());
    // A negative ridge too small to break the solve is still refused.
    for ridge in [-1e-12, -1.0, f64::NAN, f64::INFINITY] {
        assert!(
            gravity::fit_payload(&mut kin, &samples, ridge).is_err(),
            "ridge {ridge}"
        );
    }
    let mut torn = samples.clone();
    torn[3].tau[1] = f64::NAN;
    assert!(
        gravity::fit_payload(&mut kin, &torn, 1e-6).is_err(),
        "a NaN torque is no measurement"
    );
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

#[test]
fn arm_correction_changes_gravity_without_corrupting_payload_or_dynamics() {
    let tool = heavy_tool();
    let mut nominal = Kin::load_arm(&assets_dir(), Some(&tool)).unwrap();
    let mut fitted = Kin::load_arm(&assets_dir(), Some(&tool)).unwrap();
    let mut delta = vec![0.0; fitted.body_count() * 4];
    delta[9] = 0.02;
    fitted.set_gravity_correction(&delta).unwrap();
    for q in CASES {
        let mut a = [0.; NQ];
        let mut b = [0.; NQ];
        nominal.gravity(&q, &mut a).unwrap();
        fitted.gravity(&q, &mut b).unwrap();
        let change = gravity::predict(&mut nominal, &delta, &q).unwrap();
        assert!(max_abs_diff(&b, &std::array::from_fn::<_, NQ, _>(|j| a[j] + change[j])) < 1e-10);
        nominal
            .dyn_feedforward(&q, &[0.1; NQ], &[0.2; NQ], &mut a)
            .unwrap();
        fitted
            .dyn_feedforward(&q, &[0.1; NQ], &[0.2; NQ], &mut b)
            .unwrap();
        assert!(max_abs_diff(&a, &b) < 1e-10);
    }
    // A correction that is not one is refused and leaves the installed
    // one — and the payload — standing.
    let mut before = [0.; NQ];
    fitted.gravity(&CASES[0], &mut before).unwrap();
    let n = fitted.body_count() * 4;
    let mut nan = vec![0.0; n];
    nan[5] = f64::NAN;
    let mut huge = vec![0.0; n];
    huge[5] = 11.0;
    for (what, bad) in [("NaN", nan), ("huge", huge), ("short", vec![0.0; 4])] {
        assert!(fitted.set_gravity_correction(&bad).is_err(), "{what}");
        let mut after = [0.; NQ];
        fitted.gravity(&CASES[0], &mut after).unwrap();
        assert!(
            max_abs_diff(&before, &after) < 1e-12,
            "a refused {what} correction changed gravity"
        );
    }

    fitted.set_gravity_correction(&[]).unwrap();
    let mut a = [0.; NQ];
    let mut b = [0.; NQ];
    nominal.gravity(&CASES[0], &mut a).unwrap();
    fitted.gravity(&CASES[0], &mut b).unwrap();
    assert!(max_abs_diff(&a, &b) < 1e-10);
}

/// The two calibrations compose: a one-time fit of this arm's own
/// (printed, so off-table) links is installed as a correction, and a
/// later payload identification must charge the payload for the payload
/// only — not for the arm's modelling error as well.
#[test]
fn a_payload_fit_does_not_reabsorb_an_installed_arm_correction() {
    let mut kin = Kin::load_arm(&assets_dir(), None).unwrap();
    let bodies = kin.body_count();

    // The arm's links are heavier than the table says, the way a printed
    // arm is: every body up to the wrist off by a few percent of a kilo.
    let mut correction = vec![0.0; 4 * bodies];
    for b in 0..bodies - 1 {
        correction[4 * b] = 0.05;
        correction[4 * b + 3] = 0.05 * 0.04;
    }

    const MASS: f64 = 0.8;
    const COM: [f64; 3] = [0.01, -0.02, 0.05];
    let start = [-2.007, -0.698, 3.491, 0.0, 1.047, 3.1416];

    // Measured torque: the corrected arm, carrying the part. The
    // correction is real: it moves the arm's gravity on its own.
    let mut bare = [0.0; NQ];
    kin.gravity(&start, &mut bare).unwrap();
    kin.set_gravity_correction(&correction).unwrap();
    let mut corrected = [0.0; NQ];
    kin.gravity(&start, &mut corrected).unwrap();
    assert!(
        max_abs_diff(&bare, &corrected) > 1e-3,
        "the installed correction must change gravity"
    );
    kin.set_tool(MASS, COM, None).unwrap();
    let samples: Vec<GravitySample> = wrist_poses(start, 0.5)
        .into_iter()
        .map(|q| {
            let mut tau = [0.0; NQ];
            kin.gravity(&q, &mut tau).unwrap();
            GravitySample { q, tau }
        })
        .collect();

    // Put the part down. The correction stays installed, as it does in
    // service: it describes the arm, not the load.
    kin.set_tool(0.0, [0.0; 3], None).unwrap();
    let fit = gravity::fit_payload(&mut kin, &samples, 1e-6).unwrap();
    assert!(
        (fit.mass - MASS).abs() < 0.01,
        "identified {:.4} kg against {MASS} kg carried; the arm correction leaked into the payload",
        fit.mass
    );
    assert!(
        max_abs_diff(&fit.com, &COM) < 0.005,
        "identified com {:?} against {COM:?} carried",
        fit.com
    );
}

/// Poses spread across the joint limits, deterministic.
fn spread_poses(n: usize) -> Vec<[f64; NQ]> {
    const LO: [f64; NQ] = [-2.8647335, -2.4407335, 1.9912665, -2.6147335, -1.73, -0.85];
    const HI: [f64; NQ] = [2.8647335, -0.1122665, 6.5627335, 2.5547335, 1.6, 7.14];
    let mut seed = 0x2545_F491_4F6C_DD1Du64;
    let mut rnd = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    (0..n)
        .map(|_| std::array::from_fn(|j| LO[j] + rnd() * (HI[j] - LO[j])))
        .collect()
}

/// An arm that is not the table, built without the correction under
/// test: the arm URDF with every link's mass 10-20 % off, each by its own amount, and
/// its centre of mass shifted, and its gravity from the full rigid-body
/// model of that tree.
fn printed_arm() -> Kin {
    let src = assets_dir().join(Kin::ARM_URDF_RELPATH);
    let text = std::fs::read_to_string(&src).expect("arm URDF");
    let mut out = String::new();
    let mut rest = text.as_str();
    let mut link = 0usize;
    while let Some(at) = rest.find("<inertial>") {
        let end = rest[at..].find("</inertial>").expect("closed inertial") + at;
        out.push_str(&rest[..at]);
        let mut block = rest[at..end].to_string();
        let off = 0.1 + 0.1 * (link as f64 / 7.0);
        // Scale the mass.
        let m0 = block.find("value=\"").expect("mass value") + 7;
        let m1 = block[m0..].find('"').expect("quoted") + m0;
        let mass: f64 = block[m0..m1].trim().parse().expect("mass");
        block.replace_range(m0..m1, &format!("{}", mass * (1.0 + off)));
        // Shift the centre of mass.
        let c0 = block.find("xyz=\"").expect("com") + 5;
        let c1 = block[c0..].find('"').expect("quoted") + c0;
        let com: Vec<f64> = block[c0..c1]
            .split_whitespace()
            .map(|v| v.parse().expect("com value"))
            .collect();
        let shifted = format!("{} {} {}", com[0] + 0.01 * off, com[1], com[2] + 0.05 * off);
        block.replace_range(c0..c1, &shifted);
        out.push_str(&block);
        rest = &rest[end..];
        link += 1;
    }
    out.push_str(rest);
    let dir = std::env::temp_dir().join(format!("par6-printed-arm-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("temp dir");
    let urdf = dir.join("par6_arm.urdf");
    std::fs::write(&urdf, out).expect("perturbed URDF");
    Kin::from_urdf(&urdf, Some(Kin::ARM_EE_FRAME)).expect("perturbed arm")
}

/// The arm's own links are identified from static torque: a printed arm
/// weighs what it weighs, and this recovers the difference from the
/// table. Gravity fixes only part of the parameter set, so what has to
/// come back right is the TORQUE the corrected model predicts at poses
/// the fit never saw — with clean measurements, and with the friction
/// that averaging two approach directions leaves behind — and the
/// parameters the poses cannot fix must say so rather than drift.
#[test]
fn the_arms_own_links_are_identified_from_static_torque() {
    let mut truth = printed_arm();
    let measured = |truth: &mut Kin, residual_nm: f64| -> Vec<GravitySample> {
        spread_poses(24)
            .iter()
            .map(|q| {
                let mut tau = [0.0; NQ];
                truth.gravity(q, &mut tau).unwrap();
                // What friction leaves after averaging opposes the load.
                for t in &mut tau {
                    *t += residual_nm * t.signum();
                }
                GravitySample { q: *q, tau }
            })
            .collect()
    };
    let unseen = spread_poses(40);
    let worst_unseen = |truth: &mut Kin, model: &mut Kin| {
        unseen.iter().skip(24).fold(0.0f64, |worst, q| {
            let (mut want, mut got) = ([0.0; NQ], [0.0; NQ]);
            truth.gravity(q, &mut want).unwrap();
            model.gravity(q, &mut got).unwrap();
            worst.max(max_abs_diff(&want, &got))
        })
    };
    let mut plain = Kin::load_arm(&assets_dir(), None).unwrap();
    let before = worst_unseen(&mut truth, &mut plain);

    let mut model = Kin::load_arm(&assets_dir(), None).unwrap();
    let fit = gravity::fit_arm(&mut model, &measured(&mut truth, 0.0), 1e-9).unwrap();
    model.set_gravity_correction(&fit.correction).unwrap();
    let clean = worst_unseen(&mut truth, &mut model);
    assert!(
        clean < before / 20.0,
        "worst unseen-pose error {clean:.5} Nm against {before:.5} Nm uncorrected"
    );
    // The base link turns about gravity, so no pose can weigh it. That
    // has to be reported, not quietly guessed at.
    assert!(
        fit.determined[0] < 0.01,
        "the base link cannot be identified from gravity, got {}",
        fit.determined[0]
    );
    // Each joint after the base sees gravity through two combined first
    // moments — the components across its axis, lumped with everything
    // beyond it — so the five that tilt fix ten directions of the 24: the
    // trace of what the fit reports fixed, whatever basis it splits them
    // over. Not a count above a threshold: a pair it cannot tell apart
    // reads 0.5 each, which one platform's rounding counts and another's
    // does not.
    let fixed: f64 = fit.determined.iter().sum();
    assert!(
        (fixed - 10.0).abs() < 0.01,
        "the poses fixed {fixed:.3} directions, not the arm's ten: {:?}",
        fit.determined
    );

    // 0.05 Nm per joint: a twentieth of this arm's measured elbow
    // friction, what is left when the two directions cancel to a few
    // percent rather than exactly.
    let mut model = Kin::load_arm(&assets_dir(), None).unwrap();
    let fit = gravity::fit_arm(&mut model, &measured(&mut truth, 0.05), 1e-6).unwrap();
    model.set_gravity_correction(&fit.correction).unwrap();
    let rough = worst_unseen(&mut truth, &mut model);
    assert!(
        rough < before / 4.0,
        "with friction left over: worst unseen-pose error {rough:.4} Nm against \
         {before:.4} Nm uncorrected"
    );
}
