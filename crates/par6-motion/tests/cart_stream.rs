//! Cartesian streaming executor against the real PAR6 motion config.
//!
//! The promise `servo_l` makes is a LINE, so that is what these check:
//! the TCP path between setpoints, the envelope it holds to, what
//! happens when the joint layer cannot keep up, and the stop.

mod common;

use common::par6_config;
use par6_motion::cart::{self, Pose};
use par6_motion::{CartLimits, CartesianStreamingExecutor};

/// Pose at a translation \[m\], axes unrotated.
fn at(x: f64, y: f64, z: f64) -> Pose {
    [
        1.0, 0.0, 0.0, x, //
        0.0, 1.0, 0.0, y, //
        0.0, 0.0, 1.0, z, //
        0.0, 0.0, 0.0, 1.0,
    ]
}

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

fn setup() -> (CartesianStreamingExecutor, CartLimits, f64) {
    let cfg = par6_config();
    let limits = CartLimits::from_config(&cfg);
    let dt = cfg.robot.tick_dt_s;
    (
        CartesianStreamingExecutor::new(dt, limits).unwrap(),
        limits,
        dt,
    )
}

/// The whole point of running the limiter on the SE(3) tangent instead
/// of in joint space: the TCP travels the straight line between the two
/// poses, holds the linear velocity ceiling doing it, turns nothing on
/// the way, and arrives.
#[test]
fn a_pure_translation_is_followed_along_a_straight_tcp_line() {
    let (mut exec, limits, dt) = setup();
    let start = at(0.35, 0.10, 0.20);
    let end = at(0.47, 0.01, 0.26);
    exec.activate(&start);
    exec.set_target(&end).unwrap();

    let a = cart::translation(&start);
    let b = cart::translation(&end);
    let len = dist(a, b);
    let dir = [
        (b[0] - a[0]) / len,
        (b[1] - a[1]) / len,
        (b[2] - a[2]) / len,
    ];

    let mut prev = a;
    let (mut worst_off_line, mut worst_speed, mut worst_rot) = (0.0f64, 0.0f64, 0.0f64);
    let mut finished = false;
    for _ in 0..20_000 {
        let s = exec.step().unwrap();
        let p = cart::translation(&s.pose);
        let rel = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
        let along = rel[0] * dir[0] + rel[1] * dir[1] + rel[2] * dir[2];
        let foot = [
            a[0] + along * dir[0],
            a[1] + along * dir[1],
            a[2] + along * dir[2],
        ];
        worst_off_line = worst_off_line.max(dist(p, foot));
        worst_speed = worst_speed.max(dist(p, prev) / dt);
        // Rotation block: a translation turns nothing.
        for k in (0..12).filter(|k| k % 4 != 3) {
            worst_rot = worst_rot.max((s.pose[k] - start[k]).abs());
        }
        prev = p;
        if s.finished {
            finished = true;
            break;
        }
    }
    assert!(finished, "the move never reported finished");
    assert!(
        worst_off_line < 1e-9,
        "TCP bowed {worst_off_line} m off the straight line"
    );
    assert!(
        worst_speed <= limits.linear_velocity * 1.01,
        "TCP ran at {worst_speed} m/s over a {} m/s ceiling",
        limits.linear_velocity
    );
    assert!(worst_rot < 1e-12, "orientation drifted by {worst_rot}");
    assert!(
        dist(prev, b) < 1e-9,
        "stopped {} m short of the target",
        dist(prev, b)
    );
}

/// With rotation in the move the path is the screw rather than a
/// straight line, and it still has to land on both endpoints and turn
/// one way. Ruckig phase-synchronizes the six tangent components when
/// it can and falls back to time synchronization when the linear and
/// angular ceilings bind differently, so the path is the exact geodesic
/// in the first case and a bounded approximation of it in the second —
/// which is the deviation this pins down.
#[test]
fn a_move_that_turns_follows_the_screw_and_lands_on_both_endpoints() {
    let (mut exec, _limits, _dt) = setup();
    let start = at(0.30, 0.05, 0.25);
    // A target reached by a known screw about the start pose.
    let tangent = [0.08, -0.05, 0.04, 0.25, -0.18, 0.40];
    let end = cart::se3_mul(&start, &cart::se3_exp(&tangent));
    exec.activate(&start);
    exec.set_target(&end).unwrap();

    let total_rot = (tangent[3].powi(2) + tangent[4].powi(2) + tangent[5].powi(2)).sqrt();
    let mut prev_rot = 0.0;
    let mut worst_off_screw = 0.0f64;
    let mut last = start;
    let mut finished = false;
    for _ in 0..20_000 {
        let s = exec.step().unwrap();
        let local = cart::se3_log(&cart::se3_mul(&cart::se3_inverse(&start), &s.pose));
        let rot = (local[3].powi(2) + local[4].powi(2) + local[5].powi(2)).sqrt();
        assert!(
            rot >= prev_rot - 1e-12,
            "the wrist reversed: {rot} after {prev_rot}"
        );
        prev_rot = rot;
        // Distance from the exact screw: the closest point on it is the
        // one at this tick's share of the total rotation.
        let s_frac = if total_rot > 0.0 {
            rot / total_rot
        } else {
            0.0
        };
        let on_screw: [f64; 6] = std::array::from_fn(|i| s_frac * tangent[i]);
        worst_off_screw = worst_off_screw.max(dist(
            cart::translation(&s.pose),
            cart::translation(&cart::se3_mul(&start, &cart::se3_exp(&on_screw))),
        ));
        last = s.pose;
        if s.finished {
            finished = true;
            break;
        }
    }
    assert!(finished, "the move never reported finished");
    let worst_end = last
        .iter()
        .zip(end.iter())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0, f64::max);
    assert!(worst_end < 1e-9, "missed the target pose by {worst_end}");
    assert!(
        (prev_rot - total_rot).abs() < 1e-9,
        "turned {prev_rot} rad of a {total_rot} rad move"
    );
    // Sub-millimetre: close enough that the joint layer's own limits
    // dominate, and a regression to joint interpolation would blow it.
    assert!(
        worst_off_screw < 1e-3,
        "path strayed {worst_off_screw} m from the screw"
    );
}

/// A stop sheds the velocity the TCP has; it does not go back for the
/// ground it covered.
#[test]
fn release_brakes_to_rest_without_reversing() {
    let (mut exec, _limits, dt) = setup();
    let start = at(0.35, 0.10, 0.20);
    exec.activate(&start);
    exec.set_target(&at(0.60, 0.10, 0.20)).unwrap();
    for _ in 0..40 {
        exec.step().unwrap();
    }
    let at_release = cart::translation(&exec.step().unwrap().pose);

    exec.release();
    let mut prev = at_release;
    let mut speed = f64::INFINITY;
    for _ in 0..20_000 {
        let p = cart::translation(&exec.step().unwrap().pose);
        // Travel is one-way: x only ever grows.
        assert!(
            p[0] >= prev[0] - 1e-12,
            "the TCP reversed: {} after {}",
            p[0],
            prev[0]
        );
        speed = dist(p, prev) / dt;
        prev = p;
        if speed < 1e-9 {
            break;
        }
    }
    assert!(speed < 1e-9, "never came to rest (last speed {speed} m/s)");
    assert!(
        prev[0] > at_release[0],
        "braking has to cover ground, not freeze"
    );
}
