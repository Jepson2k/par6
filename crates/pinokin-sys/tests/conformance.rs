//! Conformance: the C++ shim must reproduce the pip `pin` (Pinocchio Python)
//! reference numerics on the PAR6 URDF to 1e-9 absolute.
//!
//! Fixtures come from `scripts/ffi/gen_fixtures.py`; regenerate with the
//! pinned `pin` version whenever the sampled set or tool params change.

use std::path::PathBuf;

use serde::Deserialize;

use pinokin_sys::{IkOptions, Model, ToolParams};

const TOL: f64 = 1e-9;

#[derive(Deserialize)]
struct Fixture {
    pin_version: String,
    urdf: String,
    ee_frame: String,
    tool: ToolFixture,
    cases_flange: Vec<Case>,
    cases_tool: Vec<Case>,
}

#[derive(Deserialize)]
struct ToolFixture {
    transform: [f64; 16],
    mass: f64,
    com: [f64; 3],
    inertia: [f64; 6],
}

#[derive(Deserialize)]
struct Case {
    q: Vec<f64>,
    fk: Vec<f64>,
    jac: Vec<f64>,
    tau: Vec<f64>,
}

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .unwrap()
}

fn load_fixture() -> Fixture {
    let path =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/par6_flange_pin.json");
    let json = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("cannot read {path:?}: {e}; run scripts/ffi/gen_fixtures.py"));
    serde_json::from_str(&json).expect("fixture JSON schema mismatch")
}

fn assert_close(label: &str, case_idx: usize, got: &[f64], want: &[f64]) {
    assert_eq!(got.len(), want.len(), "{label}[{case_idx}] length");
    for (i, (g, w)) in got.iter().zip(want).enumerate() {
        assert!(
            (g - w).abs() <= TOL,
            "{label}[case {case_idx}][{i}]: got {g:.15e}, want {w:.15e}, \
             diff {:.3e} > {TOL:.0e}",
            (g - w).abs()
        );
    }
}

fn check_cases(model: &mut Model, cases: &[Case], label: &str) {
    let nq = model.nq();
    let mut jac = vec![0.0; 6 * nq];
    let mut tau = vec![0.0; nq];
    for (i, case) in cases.iter().enumerate() {
        let pose = model.fk(&case.q).unwrap();
        assert_close(&format!("{label}.fk"), i, &pose, &case.fk);
        model.jacobian_into(&case.q, &mut jac).unwrap();
        assert_close(&format!("{label}.jac"), i, &jac, &case.jac);
        model.gravity_into(&case.q, &mut tau).unwrap();
        assert_close(&format!("{label}.tau"), i, &tau, &case.tau);
    }
}

#[test]
fn matches_pin_reference_on_par6_urdf() {
    let fx = load_fixture();
    let urdf = repo_root().join(&fx.urdf);
    assert!(urdf.exists(), "URDF missing: {urdf:?}");

    // Bare flange model.
    let mut model = Model::from_urdf(&urdf, Some(&fx.ee_frame), None)
        .unwrap_or_else(|e| panic!("create failed (pin fixture v{}): {e}", fx.pin_version));
    assert_eq!(model.nq(), 6);
    check_cases(&mut model, &fx.cases_flange, "flange");

    // Same model with the fixture's rigid tool (transform + inertia): fk/jac
    // shift to the tool frame, gravity picks up the tool.
    let tool = ToolParams {
        transform: fx.tool.transform,
        mass: fx.tool.mass,
        com: fx.tool.com,
        inertia: fx.tool.inertia,
    };
    let mut model = Model::from_urdf(&urdf, Some(&fx.ee_frame), Some(&tool)).unwrap();
    check_cases(&mut model, &fx.cases_tool, "tool");
}

#[test]
fn ik_recovers_fixture_poses_from_perturbed_seeds() {
    let fx = load_fixture();
    let urdf = repo_root().join(&fx.urdf);
    let mut model = Model::from_urdf(&urdf, Some(&fx.ee_frame), None).unwrap();
    let nq = model.nq();

    let mut solved = 0usize;
    for case in fx.cases_flange.iter().take(8) {
        let target: [f64; 16] = case.fk.clone().try_into().unwrap();
        // Perturb the known solution; DLS should walk back to the pose
        // (possibly via a different q — pose error is what matters).
        let seed: Vec<f64> = case
            .q
            .iter()
            .enumerate()
            .map(|(i, v)| v + 0.15 * if i % 2 == 0 { 1.0 } else { -1.0 })
            .collect();
        let mut q_out = vec![0.0; nq];
        let opts = IkOptions {
            max_iters: 200,
            tol: 1e-16, // |e|^2, so pose error ~1e-8 — tighter than the assert
            ..IkOptions::default()
        };
        let converged = model.ik_step(&seed, &target, &mut q_out, opts).unwrap();
        if !converged {
            continue;
        }
        solved += 1;
        let pose = model.fk(&q_out).unwrap();
        for (g, w) in pose.iter().zip(&target) {
            assert!(
                (g - w).abs() < 1e-6,
                "converged IK pose mismatch: {g} vs {w}"
            );
        }
    }
    assert!(
        solved >= 6,
        "DLS IK should recover most mildly-perturbed poses, solved {solved}/8"
    );
}

#[test]
fn create_reports_urdf_and_frame_errors() {
    let root = repo_root();
    let err = Model::from_urdf(&root.join("does-not-exist.urdf"), None, None).unwrap_err();
    match err {
        pinokin_sys::Error::Create(msg) => assert!(!msg.is_empty()),
        other => panic!("expected Create error, got {other:?}"),
    }

    let fx = load_fixture();
    let err = Model::from_urdf(&root.join(&fx.urdf), Some("no_such_frame"), None).unwrap_err();
    match err {
        pinokin_sys::Error::Create(msg) => {
            assert!(msg.contains("no_such_frame"), "unhelpful message: {msg}")
        }
        other => panic!("expected Create error, got {other:?}"),
    }
}

/// Inverse dynamics agrees with the gravity entry at rest, and the torque
/// it adds under acceleration is the mass matrix acting on that
/// acceleration.
///
/// The shim called `pinocchio::rnea` exactly once, always with its two zero
/// vectors, so the only dynamics it could express was G(q). `Sample::tau_ff`
/// was wired ring -> law -> wire and every planner wrote zeros into it,
/// because there was nothing to compute a feedforward WITH.
#[test]
fn inverse_dynamics_reduces_to_gravity_at_rest_and_adds_symmetric_inertia() {
    let fx = load_fixture();
    let urdf = repo_root().join(&fx.urdf);
    let mut model = Model::from_urdf(&urdf, Some(&fx.ee_frame), None).unwrap();
    let nq = model.nq();
    let zeros = vec![0.0; nq];

    for (i, case) in fx.cases_flange.iter().enumerate() {
        // Zero velocity and acceleration must reproduce G(q) exactly — not
        // approximately: it is literally the same RNEA call.
        let mut grav = vec![0.0; nq];
        model.gravity_into(&case.q, &mut grav).unwrap();
        let mut rest = vec![0.0; nq];
        model
            .inverse_dynamics_into(&case.q, &zeros, &zeros, &mut rest)
            .unwrap();
        for k in 0..nq {
            assert_eq!(
                rest[k], grav[k],
                "case {i}: tau[{k}] at rest must equal the gravity entry"
            );
        }

        // tau(q, 0, a) - G(q) is M(q)·a. Probing one axis at a time reads
        // out M(q) column by column, and physics requires it symmetric with
        // a positive diagonal.
        if i > 0 {
            continue;
        }
        let mut mass = vec![vec![0.0; nq]; nq];
        for j in 0..nq {
            let mut a = vec![0.0; nq];
            a[j] = 1.0;
            let mut tau = vec![0.0; nq];
            model
                .inverse_dynamics_into(&case.q, &zeros, &a, &mut tau)
                .unwrap();
            for k in 0..nq {
                mass[k][j] = tau[k] - grav[k];
            }
        }
        for (j, row) in mass.iter().enumerate() {
            assert!(row[j] > 0.0, "M[{j}][{j}] = {} must be positive", row[j]);
            for (k, column) in mass.iter().enumerate().take(j) {
                let (a, b) = (row[k], column[j]);
                let scale = a.abs().max(b.abs()).max(1.0);
                assert!(
                    (a - b).abs() < 1e-8 * scale,
                    "M[{j}][{k}] = {a:e} vs M[{k}][{j}] = {b:e}: not symmetric"
                );
            }
        }
    }
}

/// `ik_solve` never walks away from the target; `ik_step` can.
///
/// `par6_kin_ik_step` commits every damped-least-squares step
/// unconditionally with a fixed lambda, so near a singularity — or from a
/// seed far enough out that the linearisation is poor — a step can INCREASE
/// the residual and the solver still burns its whole budget getting worse.
/// `ik_solve` backtracks: a step is accepted only if it reduces the error,
/// and it refuses outright rather than committing when no probe does.
///
/// Asserted as a monotonicity property, which is the actual contract, not
/// as "converges more often" — that would be true by construction on any
/// seed close enough to work either way.
#[test]
fn ik_solve_never_increases_the_residual_where_ik_step_can() {
    /// The residual the solver actually minimises: squared translation
    /// error plus squared rotation angle. Measuring translation alone
    /// would be the wrong contract — a step may trade position against
    /// orientation and still reduce the combined error.
    fn pose_err(a: &[f64; 16], b: &[f64; 16]) -> f64 {
        let trans = (a[3] - b[3]).powi(2) + (a[7] - b[7]).powi(2) + (a[11] - b[11]).powi(2);
        // trace(R_a * R_b^T) for the two row-major 3x3 blocks.
        let rows = |m: &[f64; 16], r: usize| [m[4 * r], m[4 * r + 1], m[4 * r + 2]];
        let (a0, a1, a2) = (rows(a, 0), rows(a, 1), rows(a, 2));
        let (b0, b1, b2) = (rows(b, 0), rows(b, 1), rows(b, 2));
        let dot = |x: [f64; 3], y: [f64; 3]| x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
        let trace = dot(a0, b0) + dot(a1, b1) + dot(a2, b2);
        let angle = ((trace - 1.0) / 2.0).clamp(-1.0, 1.0).acos();
        trans + angle * angle
    }

    let fx = load_fixture();
    let urdf = repo_root().join(&fx.urdf);
    let mut model = Model::from_urdf(&urdf, Some(&fx.ee_frame), None).unwrap();
    let nq = model.nq();
    // Few iterations and a coarse tolerance: the point is what the solver
    // does on the way, not whether it eventually arrives.
    let opts = IkOptions {
        max_iters: 6,
        tol: 1e-12,
        damping: 1e-3,
    };

    let mut regressions_step = 0usize;
    for case in fx.cases_flange.iter() {
        let target = model.fk(&case.q).unwrap();
        // Seeds displaced hard enough that the first linearisation is poor.
        for bump in [0.9_f64, -1.2, 1.6] {
            let seed: Vec<f64> = case.q.iter().map(|v| v + bump).collect();
            let seed_err = pose_err(&model.fk(&seed).unwrap(), &target);

            let mut out = vec![0.0; nq];
            model.ik_solve(&seed, &target, &mut out, opts).unwrap();
            let solve_err = pose_err(&model.fk(&out).unwrap(), &target);
            assert!(
                solve_err <= seed_err + 1e-12,
                "ik_solve increased its own residual: {seed_err:e} -> {solve_err:e}"
            );

            let mut out_step = vec![0.0; nq];
            model.ik_step(&seed, &target, &mut out_step, opts).unwrap();
            if pose_err(&model.fk(&out_step).unwrap(), &target) > seed_err + 1e-12 {
                regressions_step += 1;
            }
        }
    }
    // Guard against the test silently becoming vacuous: if ik_step stops
    // regressing on every one of these seeds, the seeds are no longer
    // exercising the case the line search exists for.
    assert!(
        regressions_step > 0,
        "no seed made ik_step regress, so this proves nothing about the line search"
    );
}

#[test]
fn set_tool_reproduces_create_time_gravity_and_reverts() {
    let fx = load_fixture();
    let urdf = repo_root().join(&fx.urdf);
    // Built WITHOUT a tool; the runtime payload call must reproduce the
    // create-time tool's gravity exactly — same composition, same
    // numbers as the pin reference.
    let mut model = Model::from_urdf(&urdf, Some(&fx.ee_frame), None).unwrap();
    let nq = model.nq();
    let mut tau = vec![0.0; nq];

    model
        .set_tool(fx.tool.mass, fx.tool.com, Some(fx.tool.inertia))
        .unwrap();
    for (i, case) in fx.cases_tool.iter().enumerate() {
        model.gravity_into(&case.q, &mut tau).unwrap();
        assert_close("set_tool.tau", i, &tau, &case.tau);
    }

    // Clearing restores the create-time model exactly (reversible).
    model.set_tool(0.0, [0.0; 3], None).unwrap();
    for (i, case) in fx.cases_flange.iter().enumerate() {
        model.gravity_into(&case.q, &mut tau).unwrap();
        assert_close("set_tool.cleared.tau", i, &tau, &case.tau);
    }

    // Refused inputs: negative mass, NaN, non-PSD inertia — and a
    // refusal must leave the model untouched.
    assert!(model.set_tool(-1.0, [0.0; 3], None).is_err());
    assert!(model.set_tool(f64::NAN, [0.0; 3], None).is_err());
    assert!(model
        .set_tool(1.0, [0.0; 3], Some([-1.0, 0.0, 1.0, 0.0, 0.0, 1.0]))
        .is_err());
    model.gravity_into(&fx.cases_flange[0].q, &mut tau).unwrap();
    assert_close(
        "set_tool.after_refusal.tau",
        0,
        &tau,
        &fx.cases_flange[0].tau,
    );
}
