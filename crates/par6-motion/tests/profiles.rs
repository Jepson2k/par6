//! Planned-move profile tests against the real PAR6 exec limits: limit
//! adherence by finite differences, duration/speed parameterization,
//! and input validation.

mod common;

use common::{assert_within_limits, max_err, par6_config, positions_with_start};
use par6_config::LimitMode;
use par6_motion::{
    MotionError, MotionLimits, MoveParams, Plan, ProfileKind, ProgramBuilder, NUM_JOINTS,
};

const HOME: [f64; NUM_JOINTS] = [0.0, -1.5, 3.0, 0.0, 0.0, 3.1];
const TARGET: [f64; NUM_JOINTS] = [1.0, -0.5, 2.5, 1.0, 0.8, 1.0];

fn exec_limits() -> (MotionLimits, f64) {
    let cfg = par6_config();
    (
        MotionLimits::from_config(&cfg, LimitMode::Exec).unwrap(),
        cfg.robot.tick_dt_s,
    )
}

fn plan_one(profile: ProfileKind, params: MoveParams) -> (Plan, MotionLimits, f64) {
    let (limits, dt) = exec_limits();
    let mut b = ProgramBuilder::new(HOME, limits, dt).unwrap();
    b.move_j(TARGET, MoveParams { profile, ..params }).unwrap();
    (b.plan().unwrap(), limits, dt)
}

fn peak_velocity(plan: &Plan) -> [f64; NUM_JOINTS] {
    let mut peak = [0.0_f64; NUM_JOINTS];
    for s in plan.samples() {
        for (p, v) in peak.iter_mut().zip(s.qd.iter()) {
            *p = p.max(v.abs());
        }
    }
    peak
}

/// Peak |qdd| per joint over the stream.
fn peak_acceleration(plan: &Plan) -> [f64; NUM_JOINTS] {
    let mut peak = [0.0_f64; NUM_JOINTS];
    for s in plan.samples() {
        for (p, a) in peak.iter_mut().zip(s.qdd.iter()) {
            *p = p.max(a.abs());
        }
    }
    peak
}

/// Jerk per joint read off the emitted acceleration, the move starting from
/// rest. The final sample is forced to rest and would read as a spurious
/// step, so it is left out.
fn emitted_jerk(plan: &Plan, dt: f64) -> Vec<[f64; NUM_JOINTS]> {
    let s = &plan.samples()[..plan.len() - 1];
    let mut prev = [0.0; NUM_JOINTS];
    s.iter()
        .map(|x| {
            let j = std::array::from_fn(|i| (x.qdd[i] - prev[i]) / dt);
            prev = x.qdd;
            j
        })
        .collect()
}

/// Every planned profile keeps every limit it holds (jerk too where it
/// holds one), lands on the target at rest, honours a requested duration
/// and a speed fraction, and — on a move too short to cruise — is as
/// short as its binding limit allows. Trapezoid and polynomials run one
/// scalar profile scaled per joint; the polynomials also start and end
/// at rest in acceleration, and the septic in jerk as well.
#[test]
fn every_profile_respects_its_limits_and_parameterization() {
    use ProfileKind::{Quintic, Ruckig, Septic, Trapezoid};
    for profile in [Trapezoid, Ruckig, Quintic, Septic] {
        let name = format!("{profile:?}");
        let holds_jerk = matches!(profile, Ruckig | Septic);
        let polynomial = matches!(profile, Quintic | Septic);
        let jerk_bound = |l: &MotionLimits| holds_jerk.then_some(l.jerk);
        let (plan, limits, dt) = plan_one(profile, MoveParams::default());
        let qs = positions_with_start(HOME, plan.samples());
        assert_within_limits(
            &qs,
            dt,
            &limits.velocity,
            &limits.acceleration,
            jerk_bound(&limits).as_ref(),
            &name,
        );
        let last = plan.samples().last().unwrap();
        assert!(
            max_err(&last.q, &TARGET) < 1e-9,
            "{name}: must land on the target"
        );
        assert!(
            last.qd.iter().all(|&v| v.abs() < 1e-9),
            "{name}: must land at rest"
        );

        // Slowest-joint synchronization: one scalar profile scaled by each
        // joint's displacement, so qd_j / Δ_j matches across joints.
        if profile != Ruckig {
            let mid = &plan.samples()[plan.len() / 2];
            let ratios: Vec<f64> = (0..NUM_JOINTS)
                .map(|j| mid.qd[j] / (TARGET[j] - HOME[j]))
                .collect();
            for r in &ratios {
                assert!(
                    (r - ratios[0]).abs() <= 1e-9 * ratios[0].abs().max(1.0),
                    "{name}: joints must be synchronized on one path profile, ratios {ratios:?}"
                );
            }
        }

        if polynomial {
            // They START at rest in acceleration too: one tick in, a
            // trapezoid is already at its full ramp acceleration; these
            // have barely begun. The last sample is forced to rest and is
            // excluded, so the far end reads the second-to-last.
            let peak = peak_acceleration(&plan);
            let first = &plan.samples()[0];
            let penult = &plan.samples()[plan.len() - 2];
            for j in 0..NUM_JOINTS {
                if (TARGET[j] - HOME[j]).abs() < 1e-9 {
                    continue;
                }
                assert!(
                    first.qdd[j].abs() < 0.05 * peak[j],
                    "{name}: joint {j} starts at {} rad/s^2 against a peak of {}",
                    first.qdd[j],
                    peak[j]
                );
                assert!(
                    penult.qdd[j].abs() < 0.05 * peak[j],
                    "{name}: joint {j} ends at {} rad/s^2 against a peak of {}",
                    penult.qdd[j],
                    peak[j]
                );
            }
            // The septic's jerk starts and ends at rest as well; the
            // quintic's steps straight to its peak on the first tick — the
            // control that makes the septic's assertion discriminate.
            let jerk = emitted_jerk(&plan, dt);
            for j in 0..NUM_JOINTS {
                if (TARGET[j] - HOME[j]).abs() < 1e-9 {
                    continue;
                }
                let peak_j = jerk.iter().map(|x| x[j].abs()).fold(0.0, f64::max);
                let (start, end) = (jerk[0][j].abs(), jerk[jerk.len() - 1][j].abs());
                if profile == Septic {
                    assert!(
                        start < 0.1 * peak_j && end < 0.1 * peak_j,
                        "{name}: joint {j} jerk {start} at the start and {end} at the end \
                         against a peak of {peak_j}"
                    );
                } else {
                    assert!(
                        start > 0.9 * peak_j,
                        "{name}: joint {j} jerk {start} on the first tick against a peak of \
                         {peak_j} — the quintic should step to its peak"
                    );
                }
            }
        }

        // Duration-parameterized: stretching to 2× the minimum is honored.
        let t0 = plan.duration_s();
        let (stretched, limits, dt) = plan_one(
            profile,
            MoveParams {
                min_duration_s: Some(2.0 * t0),
                ..MoveParams::default()
            },
        );
        let stretch_tol = if profile == Ruckig { 3.0 } else { 2.0 } * dt;
        assert!(
            (stretched.duration_s() - 2.0 * t0).abs() <= stretch_tol,
            "{name}: requested {} s, planned {} s",
            2.0 * t0,
            stretched.duration_s()
        );
        let qs = positions_with_start(HOME, stretched.samples());
        assert_within_limits(
            &qs,
            dt,
            &limits.velocity,
            &limits.acceleration,
            jerk_bound(&limits).as_ref(),
            &format!("{name} stretched"),
        );
        assert!(max_err(&stretched.samples().last().unwrap().q, &TARGET) < 1e-9);

        // Speed-parameterized: half speed halves the velocity budget.
        let (half, limits, _) = plan_one(
            profile,
            MoveParams {
                speed_fraction: 0.5,
                ..MoveParams::default()
            },
        );
        let peak = peak_velocity(&half);
        for (j, (&p, &v)) in peak.iter().zip(limits.velocity.iter()).enumerate() {
            assert!(
                p <= 0.5 * v + 1e-9,
                "{name}: joint {j} peak {p} exceeds half budget {}",
                0.5 * v
            );
        }
        assert!(half.duration_s() > t0, "{name}: half speed takes longer");

        // Too short to cruise: acceleration (and jerk, where it is held)
        // sets the duration, and the plan is as short as that allows — the
        // binding joint reaches its limit.
        let short = HOME.map(|q| q + 0.02);
        let (limits, dt) = exec_limits();
        let mut b = ProgramBuilder::new(HOME, limits, dt).unwrap();
        b.move_j(
            short,
            MoveParams {
                profile,
                ..MoveParams::default()
            },
        )
        .unwrap();
        let brief = b.plan().unwrap();
        let qs = positions_with_start(HOME, brief.samples());
        assert_within_limits(
            &qs,
            dt,
            &limits.velocity,
            &limits.acceleration,
            jerk_bound(&limits).as_ref(),
            &format!("{name} short"),
        );
        let acc = peak_acceleration(&brief);
        let jerk = emitted_jerk(&brief, dt);
        let binding = (0..NUM_JOINTS)
            .map(|j| {
                let a = acc[j] / limits.acceleration[j];
                let jk = jerk.iter().map(|x| x[j].abs()).fold(0.0, f64::max) / limits.jerk[j];
                if holds_jerk {
                    a.max(jk)
                } else {
                    a
                }
            })
            .fold(0.0, f64::max);
        assert!(
            binding > 0.95,
            "{name}: a short move reaches only {binding:.3} of its binding limit — \
             it is slower than the limits require"
        );
    }

    // ...and a trapezoid does not start at rest in acceleration, which is
    // what makes the polynomials' assertion discriminate.
    let (trap, _, _) = plan_one(ProfileKind::Trapezoid, MoveParams::default());
    let trap_peak = peak_acceleration(&trap);
    let moving = (0..NUM_JOINTS)
        .find(|&j| (TARGET[j] - HOME[j]).abs() > 1e-9)
        .unwrap();
    assert!(
        trap.samples()[0].qdd[moving].abs() > 0.9 * trap_peak[moving],
        "the trapezoid should step straight to its ramp acceleration"
    );
}

/// `qdd` against the centered difference of the emitted `qd`. A
/// jerk-limited profile has continuous acceleration and the two must
/// agree everywhere; a trapezoid holds its acceleration constant within
/// each phase and steps it between them, and the difference smears each
/// step across two ticks, so there a mismatch is legal only where the
/// profile actually steps. The last
/// two samples are excluded: the final sample is forced to land at
/// rest, which the difference stencil reads as a spurious deceleration.
fn assert_qdd_is_the_derivative_of_qd(
    case: &str,
    profile: ProfileKind,
    plan: &Plan,
    limits: &MotionLimits,
    dt: f64,
) {
    let steps = matches!(profile, ProfileKind::Trapezoid);
    let s = plan.samples();
    // A quintic bounds its jerk by nothing but its duration, so the
    // slack the finite difference is allowed is its OWN peak jerk, read
    // off the emitted acceleration, rather than the configured limit.
    let jerk_scale: Vec<f64> = (0..NUM_JOINTS)
        .map(|j| match profile {
            ProfileKind::Quintic => s
                .windows(2)
                .map(|w| ((w[1].qdd[j] - w[0].qdd[j]) / dt).abs())
                .fold(0.0, f64::max),
            _ => limits.jerk[j],
        })
        .collect();
    assert!(
        s.iter().any(|x| x.qdd.iter().any(|a| a.abs() > 1e-3)),
        "a move that starts and ends at rest must accelerate somewhere"
    );
    if steps {
        // Constant acceleration within each phase: it changes only where
        // a phase begins or ends — four places at most.
        for j in 0..NUM_JOINTS {
            let changes = s[..s.len() - 1]
                .windows(2)
                .filter(|w| (w[1].qdd[j] - w[0].qdd[j]).abs() > 1e-9)
                .count();
            assert!(
                changes <= 4,
                "{case}: joint {j} acceleration changes {changes} times — it is not a trapezoid"
            );
        }
    }
    for (j, &jscale) in jerk_scale.iter().enumerate() {
        for k in 1..s.len().saturating_sub(2) {
            let fd = (s[k + 1].qd[j] - s[k - 1].qd[j]) / (2.0 * dt);
            let err = (fd - s[k].qdd[j]).abs();
            if !steps {
                let tol = jscale * dt + 1e-6;
                assert!(
                    err <= tol,
                    "{case}: joint {j} sample {k}: qdd {} vs finite-difference {fd} (tol {tol})",
                    s[k].qdd[j]
                );
            } else if err > 1e-6 {
                assert!(
                    (s[k + 1].qdd[j] - s[k - 1].qdd[j]).abs() > 1e-9,
                    "{case}: joint {j} sample {k}: qdd {} vs finite-difference {fd} \
                     away from any phase boundary",
                    s[k].qdd[j]
                );
            }
        }
    }
}

#[test]
fn emitted_acceleration_is_the_derivative_of_emitted_velocity() {
    for profile in [
        ProfileKind::Trapezoid,
        ProfileKind::Ruckig,
        ProfileKind::Quintic,
        ProfileKind::Septic,
    ] {
        let (plan, limits, dt) = plan_one(profile, MoveParams::default());
        assert_qdd_is_the_derivative_of_qd(&format!("{profile:?}"), profile, &plan, &limits, dt);
    }
}

#[test]
fn builder_rejects_invalid_programs() {
    let (limits, dt) = exec_limits();
    let mut b = ProgramBuilder::new(HOME, limits, dt).unwrap();

    let nan = {
        let mut t = TARGET;
        t[2] = f64::NAN;
        t
    };
    assert!(matches!(
        b.move_j(nan, MoveParams::default()),
        Err(MotionError::InvalidInput { what: "target", .. })
    ));
    let inf = {
        let mut t = TARGET;
        t[0] = f64::INFINITY;
        t
    };
    assert!(matches!(
        b.move_j(inf, MoveParams::default()),
        Err(MotionError::InvalidInput { what: "target", .. })
    ));
    let outside = {
        let mut t = TARGET;
        t[1] = 0.5; // J1 soft window is [-2.44, -0.11]
        t
    };
    assert!(matches!(
        b.move_j(outside, MoveParams::default()),
        Err(MotionError::TargetOutsideSoftLimits { joint: 1, .. })
    ));
    for bad_frac in [0.0, -0.5, 1.5, f64::NAN] {
        assert!(matches!(
            b.move_j(
                TARGET,
                MoveParams {
                    speed_fraction: bad_frac,
                    ..MoveParams::default()
                }
            ),
            Err(MotionError::InvalidInput {
                what: "speed_fraction",
                ..
            })
        ));
    }
    for bad_dur in [0.0, -1.0, f64::INFINITY] {
        assert!(matches!(
            b.move_j(
                TARGET,
                MoveParams {
                    min_duration_s: Some(bad_dur),
                    ..MoveParams::default()
                }
            ),
            Err(MotionError::InvalidInput {
                what: "min_duration_s",
                ..
            })
        ));
    }

    // Nothing was queued by the rejected moves.
    assert!(matches!(
        b.plan(),
        Err(MotionError::InvalidInput { what: "moves", .. })
    ));

    // The ruckig profile needs finite jerk limits.
    let mut no_jerk = limits;
    no_jerk.jerk = [f64::INFINITY; NUM_JOINTS];
    let mut b = ProgramBuilder::new(HOME, no_jerk, dt).unwrap();
    b.move_j(TARGET, MoveParams::default()).unwrap();
    assert!(matches!(
        b.plan(),
        Err(MotionError::MissingJerkLimit { joint: 0 })
    ));
    // ...but the trapezoid profile does not.
    let mut b = ProgramBuilder::new(HOME, no_jerk, dt).unwrap();
    b.move_j(
        TARGET,
        MoveParams {
            profile: ProfileKind::Trapezoid,
            ..MoveParams::default()
        },
    )
    .unwrap();
    assert!(b.plan().is_ok());
}
