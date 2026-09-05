//! The kinematics contract on the shipped PAR6 URDF variants, checked
//! against requirements rather than against a recording: the Jacobian
//! must be the derivative of the forward kinematics, the analytic IK must
//! land back on any pose FK produced and say so when a pose is out of
//! reach, and non-finite input must come back as "pose unknown", never as
//! a fabricated answer. (FK itself is cross-checked against the
//! independent OPW closed form every time a model loads — `Opw::derive`
//! refuses a tree whose FK it cannot reproduce.)
// Joint values are spelled the way config/PAR6.toml spells them.
#![allow(clippy::approx_constant)]

use std::time::Instant;

use par6_kin::{GripperVariant, IkOutcome, Kin, IK_POSE_TOL, NQ};

mod common;
use common::assets_dir;

/// Configurations inside every joint's travel, spread over the workspace:
/// the park pose (a wrist singularity), the runtime's cartesian test
/// postures, and two general poses with every joint off its home value.
/// `CASES[0]` puts the wrist through its singularity, where q4 and q6
/// are only determined in combination and the closed form is least
/// conditioned.
const WRIST_SINGULAR_CASE: usize = 0;

const CASES: [[f64; NQ]; 6] = [
    [0.0, -1.5708, 3.1416, 0.0, 0.0, 3.1416],
    [1.2, -1.2708, 3.7416, 0.0, 0.5, 0.0],
    [-2.007, -0.698, 3.491, 0.0, 1.047, 3.1416],
    [0.0, -1.309, 5.323, 0.349, -0.524, 3.1416],
    [0.5, -1.0, 2.6, 0.3, 0.8, 2.5],
    [-0.8, -1.3, 3.3, -0.6, -0.7, 3.6],
];

fn load(variant: GripperVariant) -> Kin {
    Kin::load(&assets_dir(), variant).unwrap_or_else(|e| panic!("{variant:?} load failed: {e}"))
}

/// The angular velocity `ω` of a rotation-rate matrix `Ṙ Rᵀ` (world axes).
fn vee(w: [[f64; 3]; 3]) -> [f64; 3] {
    [w[2][1], w[0][2], w[1][0]]
}

fn rotation(pose: &[f64; 16]) -> [[f64; 3]; 3] {
    let mut r = [[0.0; 3]; 3];
    for (i, row) in r.iter_mut().enumerate() {
        row.copy_from_slice(&pose[4 * i..4 * i + 3]);
    }
    r
}

/// The Jacobian is the derivative of FK: its linear rows are the TCP
/// velocity per unit joint rate and its angular rows the world-frame
/// angular velocity, both measured here by central differences of FK.
#[test]
fn jacobian_is_the_derivative_of_forward_kinematics() {
    const H: f64 = 1e-6;
    const TOL: f64 = 1e-6;
    for variant in GripperVariant::ALL {
        let mut kin = load(variant);
        let mut jac = [0.0; 6 * NQ];
        let mut plus = [0.0; 16];
        let mut minus = [0.0; 16];
        for (c, q) in CASES.iter().enumerate() {
            kin.jacobian(q, &mut jac).unwrap();
            for j in 0..NQ {
                let mut qp = *q;
                let mut qm = *q;
                qp[j] += H;
                qm[j] -= H;
                kin.fk(&qp, &mut plus).unwrap();
                kin.fk(&qm, &mut minus).unwrap();
                for (axis, &slot) in [3usize, 7, 11].iter().enumerate() {
                    let numeric = (plus[slot] - minus[slot]) / (2.0 * H);
                    let analytic = jac[axis * NQ + j];
                    assert!(
                        (numeric - analytic).abs() < TOL,
                        "{variant:?} case {c} joint {j} linear axis {axis}: \
                         J = {analytic}, FK slope = {numeric}"
                    );
                }
                // ω̂ = Ṙ Rᵀ, with Ṙ the central difference and R at q.
                let mut at = [0.0; 16];
                kin.fk(q, &mut at).unwrap();
                let (rp, rm, r0) = (rotation(&plus), rotation(&minus), rotation(&at));
                let mut w = [[0.0; 3]; 3];
                for (a, row) in w.iter_mut().enumerate() {
                    for (b, cell) in row.iter_mut().enumerate() {
                        *cell = (0..3)
                            .map(|k| (rp[a][k] - rm[a][k]) / (2.0 * H) * r0[b][k])
                            .sum();
                    }
                }
                let omega = vee(w);
                for axis in 0..3 {
                    let analytic = jac[(3 + axis) * NQ + j];
                    assert!(
                        (omega[axis] - analytic).abs() < TOL,
                        "{variant:?} case {c} joint {j} angular axis {axis}: \
                         J = {analytic}, FK slope = {}",
                        omega[axis]
                    );
                }
            }
        }
    }
}

/// Every pose FK produces is reachable by construction, so the closed
/// form must land on it: exactly on the same branch when seeded with the
/// truth, and on a pose-equivalent branch from a perturbed seed. A pose
/// outside the workspace is an explicit `Unreachable`.
#[test]
fn ik_recovers_fk_poses_and_reports_unreachable() {
    // The gripper variants exercise the padded-jaw path; flange the plain one.
    for variant in [GripperVariant::Flange, GripperVariant::Ssg48] {
        let mut kin = load(variant);
        kin.opw().unwrap_or_else(|e| panic!("{variant:?}: {e}"));
        let mut target = [0.0; 16];
        let mut reached = [0.0; 16];
        let mut q_out = [0.0; NQ];
        for (c, q) in CASES.iter().enumerate() {
            kin.fk(q, &mut target).unwrap();

            // A perturbed seed only selects the branch; the closed form
            // lands on the pose regardless.
            let mut seed = *q;
            for (j, s) in seed.iter_mut().enumerate() {
                *s += 0.15 * if j % 2 == 0 { 1.0 } else { -1.0 };
            }
            assert_eq!(
                kin.ik(&seed, &target, &mut q_out).unwrap(),
                IkOutcome::Converged,
                "{variant:?} case {c}"
            );
            kin.fk(&q_out, &mut reached).unwrap();
            // `Converged` already promises the pose is within
            // `IK_POSE_TOL`, so re-checking that bound would assert what
            // the outcome means. What is worth pinning is that the
            // ANALYTIC solver lands at machine precision when the arm is
            // not singular — an implementation that started iterating,
            // or lost a factorisation's worth of digits, would still say
            // Converged while missing this by orders of magnitude.
            let worst = reached
                .iter()
                .zip(&target)
                .map(|(g, w)| (g - w).abs())
                .fold(0.0, f64::max);
            let bound = if c == WRIST_SINGULAR_CASE {
                // Here the branch is picked from a perturbed seed and
                // the closed form is least conditioned, so the contract
                // tolerance is all that holds.
                IK_POSE_TOL
            } else {
                1e-12
            };
            assert!(
                worst < bound,
                "{variant:?} case {c}: IK reached the pose to {worst:.3e}, \
                 further than the {bound:.0e} an analytic solve should"
            );

            // Seeded with the truth, the nearest branch IS the truth —
            // except at a wrist singularity, where q4 and q6 are only
            // determined as a sum.
            assert_eq!(
                kin.ik(q, &target, &mut q_out).unwrap(),
                IkOutcome::Converged
            );
            // At a wrist singularity the fourth and sixth axes line up,
            // so only their sum is determined, and the whole solve is
            // ill-conditioned: the closed form still lands on the pose
            // (checked above, exactly) but returns joints good to
            // microradians rather than nanoradians. A microradian is six
            // orders of magnitude below anything the arm can resolve.
            let singular = q[4].abs() < 1e-9;
            let tol = if singular { 1e-5 } else { 1e-9 };
            for (j, (g, w)) in q_out.iter().zip(q).enumerate() {
                if singular && (j == 3 || j == 5) {
                    continue;
                }
                assert!(
                    (g - w).abs() < tol,
                    "{variant:?} case {c} joint {j}: {g} vs {w}"
                );
            }
            if singular {
                assert!(
                    ((q_out[3] + q_out[5]) - (q[3] + q[5])).abs() < tol,
                    "{variant:?} case {c}: q4 + q6 must be preserved through the singularity"
                );
            }
        }

        // A pose far outside the workspace must come back as an explicit
        // Unreachable — no panic, no false convergence claim.
        let unreachable: [f64; 16] = [
            1.0, 0.0, 0.0, 10.0, //
            0.0, 1.0, 0.0, 0.0, //
            0.0, 0.0, 1.0, 10.0, //
            0.0, 0.0, 0.0, 1.0,
        ];
        let outcome = kin.ik(&[0.0; NQ], &unreachable, &mut q_out).unwrap();
        assert_eq!(
            outcome,
            IkOutcome::Unreachable,
            "{variant:?} unreachable target"
        );
    }
}

#[test]
fn nan_inputs_yield_nan_pose_not_panic() {
    // ForwardKin seam contract: NaN input channels may produce NaN outputs
    // ("pose unknown"), and nothing may panic or fabricate a pose.
    let mut kin = load(GripperVariant::Flange);
    let mut q = [0.0; NQ];
    q[2] = f64::NAN;
    let mut tcp = [0.0; 6];
    kin.tcp(&q, &mut tcp);
    assert!(
        tcp.iter().all(|v| v.is_nan()),
        "expected all-NaN tcp, got {tcp:?}"
    );

    // NaN seed through IK: must terminate and must not claim convergence.
    let target: [f64; 16] = {
        let mut pose = [0.0; 16];
        kin.fk(&[0.0; NQ], &mut pose).unwrap();
        pose
    };
    let mut q_out = [0.0; NQ];
    let outcome = kin.ik(&q, &target, &mut q_out).unwrap();
    assert_eq!(outcome, IkOutcome::Unreachable);
}

/// What the RT tick spends on kinematics, measured and printed.
///
/// The tick calls exactly two of these: forward kinematics for the pose
/// the snapshot publishes, and gravity for the feedforward. The rest run
/// on the planner and housekeeping threads. The guard is a catastrophe
/// check against the tick the runtime is aiming at, not a benchmark gate:
/// a lost preallocation or an accidental model rebuild costs orders of
/// magnitude, runner noise costs a factor of two.
#[test]
fn per_tick_kinematics_cost_is_reported() {
    /// The tick period the runtime targets past 250 Hz.
    const TARGET_TICK_US: f64 = 1_000.0;
    /// The share of it the RT kinematics may take.
    const BUDGET_FRACTION: f64 = 0.2;

    let n = 500;
    for variant in [GripperVariant::Flange, GripperVariant::Ssg48] {
        let mut kin = load(variant);
        let mut pose = [0.0; 16];
        let mut tau = [0.0; NQ];
        let mut jac = [0.0; 6 * NQ];
        let mut tcp = [0.0; 6];
        let mut q_out = [0.0; NQ];
        kin.fk(&CASES[0], &mut pose).unwrap();

        let mut rt_us = 0.0;
        for (name, mut call) in [
            (
                "fk",
                Box::new(|k: &mut Kin, q: &[f64; NQ]| {
                    k.fk(q, &mut pose).unwrap();
                }) as Box<dyn FnMut(&mut Kin, &[f64; NQ])>,
            ),
            (
                "gravity",
                Box::new(|k: &mut Kin, q: &[f64; NQ]| {
                    k.gravity(q, &mut tau).unwrap();
                }),
            ),
            (
                "tcp",
                Box::new(|k: &mut Kin, q: &[f64; NQ]| {
                    k.tcp(q, &mut tcp);
                }),
            ),
            (
                "jacobian",
                Box::new(|k: &mut Kin, q: &[f64; NQ]| {
                    k.jacobian(q, &mut jac).unwrap();
                }),
            ),
            (
                "ik",
                Box::new(|k: &mut Kin, q: &[f64; NQ]| {
                    let mut target = [0.0; 16];
                    k.fk(q, &mut target).unwrap();
                    k.ik(q, &target, &mut q_out).unwrap();
                }),
            ),
        ] {
            let t0 = Instant::now();
            for i in 0..n {
                call(&mut kin, &CASES[i % CASES.len()]);
            }
            let each = t0.elapsed().as_secs_f64() * 1e6 / n as f64;
            println!("{variant:?} {name:<9} {each:8.2} us");
            if name == "fk" || name == "gravity" {
                rt_us += each;
            }
        }
        assert!(
            rt_us < TARGET_TICK_US * BUDGET_FRACTION,
            "{variant:?}: the RT tick's kinematics take {rt_us:.1} us of a \
             {TARGET_TICK_US:.0} us tick"
        );
    }
}
