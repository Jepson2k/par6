//! Planned-move profile tests against the real PAR6 exec limits: limit
//! adherence by finite differences, duration/speed parameterization,
//! corner blending continuity, and input validation.

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

#[test]
fn trapezoid_move_respects_limits_and_parameterization() {
    let (plan, limits, dt) = plan_one(ProfileKind::Trapezoid, MoveParams::default());
    let qs = positions_with_start(HOME, plan.samples());
    assert_within_limits(
        &qs,
        dt,
        &limits.velocity,
        &limits.acceleration,
        None,
        "trap",
    );
    let last = plan.samples().last().unwrap();
    assert!(max_err(&last.q, &TARGET) < 1e-9, "must land on the target");
    assert!(last.qd.iter().all(|&v| v == 0.0), "must land at rest");

    // Slowest-joint synchronization: every joint runs the same scalar
    // profile scaled by its displacement, so qd_j / Δ_j matches across
    // joints at every tick.
    let mid = &plan.samples()[plan.len() / 2];
    let ratios: Vec<f64> = (0..NUM_JOINTS)
        .map(|j| mid.qd[j] / (TARGET[j] - HOME[j]))
        .collect();
    for r in &ratios {
        assert!(
            (r - ratios[0]).abs() <= 1e-9 * ratios[0].abs().max(1.0),
            "joints must be synchronized on one path profile, ratios {ratios:?}"
        );
    }

    // Duration-parameterized: stretching to 2× the minimum is honored.
    let t0 = plan.duration_s();
    let (stretched, limits, dt) = plan_one(
        ProfileKind::Trapezoid,
        MoveParams {
            min_duration_s: Some(2.0 * t0),
            ..MoveParams::default()
        },
    );
    assert!(
        (stretched.duration_s() - 2.0 * t0).abs() <= 2.0 * dt,
        "requested {} s, planned {} s",
        2.0 * t0,
        stretched.duration_s()
    );
    let qs = positions_with_start(HOME, stretched.samples());
    assert_within_limits(
        &qs,
        dt,
        &limits.velocity,
        &limits.acceleration,
        None,
        "trap stretched",
    );
    assert!(max_err(&stretched.samples().last().unwrap().q, &TARGET) < 1e-9);

    // Speed-parameterized: half speed halves the velocity budget and takes
    // longer.
    let (half, limits, _) = plan_one(
        ProfileKind::Trapezoid,
        MoveParams {
            speed_fraction: 0.5,
            ..MoveParams::default()
        },
    );
    let peak = peak_velocity(&half);
    for (j, (&p, &v)) in peak.iter().zip(limits.velocity.iter()).enumerate() {
        assert!(
            p <= 0.5 * v + 1e-9,
            "joint {j} peak {p} exceeds half budget {}",
            0.5 * v
        );
    }
    assert!(half.duration_s() > t0);
}

#[test]
fn ruckig_move_respects_limits_including_jerk() {
    let (plan, limits, dt) = plan_one(ProfileKind::Ruckig, MoveParams::default());
    let qs = positions_with_start(HOME, plan.samples());
    assert_within_limits(
        &qs,
        dt,
        &limits.velocity,
        &limits.acceleration,
        Some(&limits.jerk),
        "ruckig",
    );
    let last = plan.samples().last().unwrap();
    assert!(max_err(&last.q, &TARGET) < 1e-9, "must land on the target");
    assert!(last.qd.iter().all(|&v| v.abs() < 1e-9), "must land at rest");

    // Duration-parameterized via ruckig's minimum duration.
    let t0 = plan.duration_s();
    let (stretched, _, dt) = plan_one(
        ProfileKind::Ruckig,
        MoveParams {
            min_duration_s: Some(2.0 * t0),
            ..MoveParams::default()
        },
    );
    assert!(
        (stretched.duration_s() - 2.0 * t0).abs() <= 3.0 * dt,
        "requested {} s, planned {} s",
        2.0 * t0,
        stretched.duration_s()
    );

    // Speed-parameterized.
    let (half, limits, _) = plan_one(
        ProfileKind::Ruckig,
        MoveParams {
            speed_fraction: 0.5,
            ..MoveParams::default()
        },
    );
    let peak = peak_velocity(&half);
    for (j, (&p, &v)) in peak.iter().zip(limits.velocity.iter()).enumerate() {
        assert!(
            p <= 0.5 * v + 1e-9,
            "joint {j} peak {p} exceeds half budget {}",
            0.5 * v
        );
    }
}

/// Second target continuing every joint in its HOME→TARGET direction
/// (clipped inside the soft windows) so a blended corner keeps cruising
/// instead of reversing.
fn second_target() -> [f64; NUM_JOINTS] {
    [2.0, -0.2, 2.0, 2.0, 1.5, -0.5]
}

fn plan_two(profile: ProfileKind, blend: bool) -> (Plan, MotionLimits, f64) {
    let (limits, dt) = exec_limits();
    let mut b = ProgramBuilder::new(HOME, limits, dt).unwrap();
    b.move_j(
        TARGET,
        MoveParams {
            profile,
            blend_with_next: blend,
            ..MoveParams::default()
        },
    )
    .unwrap()
    .move_j(
        second_target(),
        MoveParams {
            profile,
            ..MoveParams::default()
        },
    )
    .unwrap();
    (b.plan().unwrap(), limits, dt)
}

/// Largest velocity utilization max_j |qd_j| / v_limit_j at one sample.
fn vel_utilization(s: &par6_motion::Sample, limits: &MotionLimits) -> f64 {
    (0..NUM_JOINTS)
        .map(|j| s.qd[j].abs() / limits.velocity[j])
        .fold(0.0, f64::max)
}

fn boundary_index(plan: &Plan) -> usize {
    plan.samples()
        .iter()
        .position(|s| s.meta.command_index == 1)
        .expect("second command must appear in the stream")
}

fn assert_blend_behavior(profile: ProfileKind) {
    let (plan, limits, dt) = plan_two(profile, true);
    let jerk = match profile {
        ProfileKind::Ruckig => Some(&limits.jerk),
        _ => None,
    };
    let qs = positions_with_start(HOME, plan.samples());
    assert_within_limits(
        &qs,
        dt,
        &limits.velocity,
        &limits.acceleration,
        jerk,
        "blend",
    );

    // C1 continuity: the commanded velocity stream never slews faster than
    // the acceleration limit, splice included, and stays consistent with
    // the position stream.
    let mut prev_qd = [0.0; NUM_JOINTS];
    for (k, s) in plan.samples().iter().enumerate() {
        for j in 0..NUM_JOINTS {
            let dv = (s.qd[j] - prev_qd[j]).abs();
            assert!(
                dv <= limits.acceleration[j] * dt * (1.0 + 1e-6) + 1e-9,
                "qd slew {dv} on joint {j} at tick {k} exceeds a*dt"
            );
            let fd = (qs[k + 1][j] - qs[k][j]) / dt;
            let mid = 0.5 * (s.qd[j] + prev_qd[j]);
            assert!(
                (fd - mid).abs() <= limits.acceleration[j] * dt + 1e-6,
                "qd inconsistent with positions on joint {j} at tick {k}: fd {fd} vs {mid}"
            );
        }
        prev_qd = s.qd;
    }

    // The corner is taken at speed: the peak cruise utilization does not
    // collapse at the command boundary.
    let peak_util = plan
        .samples()
        .iter()
        .map(|s| vel_utilization(s, &limits))
        .fold(0.0, f64::max);
    let b = boundary_index(&plan);
    let boundary_util = vel_utilization(&plan.samples()[b], &limits);
    assert!(
        boundary_util > 0.25 * peak_util,
        "blended corner nearly stopped: {boundary_util} vs peak {peak_util}"
    );

    // Metadata semantics.
    for s in &plan.samples()[..b] {
        assert!(
            s.meta.blend_continues,
            "blending segment must carry the flag"
        );
        assert_eq!(s.meta.command_index, 0);
        assert_eq!(s.meta.checkpoint_id, 0);
    }
    for s in &plan.samples()[b..] {
        assert!(!s.meta.blend_continues);
        assert_eq!(s.meta.command_index, 1);
        assert_eq!(s.meta.checkpoint_id, 1);
    }
    let last = plan.samples().last().unwrap();
    assert!(last.meta.is_last);
    assert!(max_err(&last.q, &second_target()) < 1e-9);

    // Contrast: without blending the same program settles to rest at the
    // boundary and carries no blend flag.
    let (unblended, _, _) = plan_two(profile, false);
    let b = boundary_index(&unblended);
    let handoff = &unblended.samples()[b - 1];
    assert!(
        handoff.qd.iter().all(|&v| v.abs() < 1e-9),
        "unblended boundary must be at rest, got {:?}",
        handoff.qd
    );
    assert!(!handoff.meta.blend_continues);
}

#[test]
fn corner_blending_is_velocity_continuous_trapezoid() {
    assert_blend_behavior(ProfileKind::Trapezoid);
}

#[test]
fn corner_blending_is_velocity_continuous_ruckig() {
    assert_blend_behavior(ProfileKind::Ruckig);
}

/// `qdd` against the centered difference of the emitted `qd`. A
/// jerk-limited profile has continuous acceleration and the two must
/// agree everywhere; a trapezoid steps its acceleration between phases
/// and the difference smears each step across two ticks, so there a
/// mismatch is legal only where the profile actually steps. The last
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
    assert!(
        s.iter().any(|x| x.qdd.iter().any(|a| a.abs() > 1e-3)),
        "a move that starts and ends at rest must accelerate somewhere"
    );
    for j in 0..NUM_JOINTS {
        for k in 1..s.len().saturating_sub(2) {
            let fd = (s[k + 1].qd[j] - s[k - 1].qd[j]) / (2.0 * dt);
            let err = (fd - s[k].qdd[j]).abs();
            if !steps {
                let tol = limits.jerk[j] * dt + 1e-6;
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
    for profile in [ProfileKind::Trapezoid, ProfileKind::Ruckig] {
        let (plan, limits, dt) = plan_one(profile, MoveParams::default());
        assert_qdd_is_the_derivative_of_qd(
            &format!("{profile:?} single"),
            profile,
            &plan,
            &limits,
            dt,
        );
        let (plan, limits, dt) = plan_two(profile, true);
        assert_qdd_is_the_derivative_of_qd(
            &format!("{profile:?} blend"),
            profile,
            &plan,
            &limits,
            dt,
        );
    }
}

#[test]
fn ruckig_waypoint_chain_passes_through_waypoints() {
    let (limits, dt) = exec_limits();
    let way1 = TARGET;
    let way2: [f64; NUM_JOINTS] = [0.5, -1.0, 2.7, 0.5, 0.2, 2.0];
    let end: [f64; NUM_JOINTS] = [-0.5, -2.0, 3.5, -0.5, -0.4, 3.0];
    let mut b = ProgramBuilder::new(HOME, limits, dt).unwrap();
    let blend = MoveParams {
        profile: ProfileKind::Ruckig,
        blend_with_next: true,
        ..MoveParams::default()
    };
    b.move_j(way1, blend)
        .unwrap()
        .move_j(way2, blend)
        .unwrap()
        .move_j(
            end,
            MoveParams {
                profile: ProfileKind::Ruckig,
                ..MoveParams::default()
            },
        )
        .unwrap();
    let plan = b.plan().unwrap();
    let qs = positions_with_start(HOME, plan.samples());
    assert_within_limits(
        &qs,
        dt,
        &limits.velocity,
        &limits.acceleration,
        Some(&limits.jerk),
        "waypoint chain",
    );
    for (name, wp) in [("way1", way1), ("way2", way2)] {
        let closest = plan
            .samples()
            .iter()
            .map(|s| max_err(&s.q, &wp))
            .fold(f64::INFINITY, f64::min);
        assert!(
            closest < 0.05,
            "chain must pass through {name}, closest approach {closest} rad"
        );
    }
    assert!(max_err(&plan.samples().last().unwrap().q, &end) < 1e-9);
    // Three commands appear in order.
    let cmds: Vec<u32> = plan
        .samples()
        .iter()
        .map(|s| s.meta.command_index)
        .collect();
    assert!(cmds.windows(2).all(|w| w[0] <= w[1]));
    assert_eq!(*cmds.last().unwrap(), 2);
    assert_eq!(cmds[0], 0);
    assert!(cmds.contains(&1));
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

    // A blend chain must not mix profiles.
    let mut b = ProgramBuilder::new(HOME, limits, dt).unwrap();
    b.move_j(
        TARGET,
        MoveParams {
            profile: ProfileKind::Trapezoid,
            blend_with_next: true,
            ..MoveParams::default()
        },
    )
    .unwrap()
    .move_j(
        second_target(),
        MoveParams {
            profile: ProfileKind::Ruckig,
            ..MoveParams::default()
        },
    )
    .unwrap();
    assert!(matches!(
        b.plan(),
        Err(MotionError::MixedProfileBlend {
            first: 0,
            second: 1
        })
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
