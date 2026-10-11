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
    let limits = CartLimits::from_motion(&cfg.motion);
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

/// `R(φ)` about the unit axis `u` (Rodrigues), row-major.
fn rot_about(u: [f64; 3], phi: f64) -> [[f64; 3]; 3] {
    let k = [[0.0, -u[2], u[1]], [u[2], 0.0, -u[0]], [-u[1], u[0], 0.0]];
    let (s, c) = phi.sin_cos();
    std::array::from_fn(|r| {
        std::array::from_fn(|col| {
            let k2: f64 = (0..3).map(|m| k[r][m] * k[m][col]).sum();
            f64::from(u8::from(r == col)) + s * k[r][col] + (1.0 - c) * k2
        })
    })
}

/// A point carried by the screw about the line through `c` along `u`:
/// turned `φ` about it and advanced `pitch·φ` along it.
fn on_helix(p: [f64; 3], c: [f64; 3], u: [f64; 3], pitch: f64, phi: f64) -> [f64; 3] {
    let r = rot_about(u, phi);
    let d = [p[0] - c[0], p[1] - c[1], p[2] - c[2]];
    std::array::from_fn(|i| (0..3).map(|m| r[i][m] * d[m]).sum::<f64>() + c[i] + pitch * phi * u[i])
}

/// With rotation in the move the path is the screw rather than a
/// straight line, and it still has to land on both endpoints and turn
/// one way. The target is built as a known helix about a known axis
/// (Chasles: any rigid motion is one), and the path is checked against
/// that helix, written here without the library's own SE(3) maps.
/// Ruckig phase-synchronizes the six tangent components when it can and
/// falls back to time synchronization when the linear and angular
/// ceilings bind differently, so the path is the exact helix in the
/// first case and a bounded approximation of it in the second — which
/// is the deviation this pins down.
#[test]
fn a_move_that_turns_follows_the_screw_and_lands_on_both_endpoints() {
    let (mut exec, _limits, _dt) = setup();
    let p0 = [0.30, 0.05, 0.25];
    let start = at(p0[0], p0[1], p0[2]);
    let n = (0.3f64 * 0.3 + 0.5 * 0.5 + 0.8 * 0.8).sqrt();
    let u = [0.3 / n, -0.5 / n, 0.8 / n];
    let (c, pitch, theta) = ([0.25, 0.0, 0.20], 0.05, 0.6);
    let r_end = rot_about(u, theta);
    let p_end = on_helix(p0, c, u, pitch, theta);
    let end: Pose = std::array::from_fn(|k| {
        let (row, col) = (k / 4, k % 4);
        match (row, col) {
            (3, 3) => 1.0,
            (3, _) => 0.0,
            (_, 3) => p_end[row],
            _ => r_end[row][col],
        }
    });
    exec.activate(&start);
    exec.set_target(&end).unwrap();

    // The start is unrotated, so the angle turned is read off the trace.
    let turned = |m: &Pose| ((m[0] + m[5] + m[10] - 1.0) / 2.0).clamp(-1.0, 1.0).acos();
    let mut prev_rot = 0.0;
    let mut worst_off_screw = 0.0f64;
    let mut last = start;
    let mut finished = false;
    for _ in 0..20_000 {
        let s = exec.step().unwrap();
        let rot = turned(&s.pose);
        assert!(
            rot >= prev_rot - 1e-9,
            "the wrist reversed: {rot} after {prev_rot}"
        );
        prev_rot = rot;
        // The closest point of the helix is the one at this tick's share
        // of the rotation.
        worst_off_screw = worst_off_screw.max(dist(
            cart::translation(&s.pose),
            on_helix(p0, c, u, pitch, rot),
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
        (prev_rot - theta).abs() < 1e-9,
        "turned {prev_rot} rad of a {theta} rad move"
    );
    // Sub-millimetre: close enough that the joint layer's own limits
    // dominate, and a regression to joint interpolation would blow it.
    assert!(
        worst_off_screw < 1e-3,
        "path strayed {worst_off_screw} m from the screw"
    );
}

/// A stop sheds the velocity the TCP has, inside the distance its
/// acceleration and jerk ceilings need to, short of the target it was
/// heading for; it does not go back for the ground it covered.
#[test]
fn release_brakes_to_rest_without_reversing() {
    let (mut exec, limits, dt) = setup();
    let start = at(0.35, 0.10, 0.20);
    let target_x = 0.60;
    exec.activate(&start);
    exec.set_target(&at(target_x, 0.10, 0.20)).unwrap();
    for _ in 0..39 {
        exec.step().unwrap();
    }
    let before = cart::translation(&exec.step().unwrap().pose);
    let at_release = cart::translation(&exec.step().unwrap().pose);
    let v = dist(at_release, before) / dt;
    assert!(v > 0.01, "the TCP is moving when released: {v} m/s");

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
    // The worst case under the ceilings: swing the acceleration from
    // +a to -a at the jerk limit, carrying the speed that swing adds,
    // then shed the rest at -a.
    let (a, j) = (limits.linear_acceleration, limits.linear_jerk);
    let swing = 2.0 * a / j;
    let v_peak = v + a * a / (2.0 * j);
    let bound = v_peak * swing + v_peak * v_peak / (2.0 * a);
    let braked = prev[0] - at_release[0];
    assert!(
        braked <= bound,
        "braked over {braked} m from {v} m/s; the ceilings allow {bound} m"
    );
    assert!(
        prev[0] < target_x - 1e-3,
        "the release ran on to the target: rested at x = {}",
        prev[0]
    );
}

/// A cartesian jog is a TCP velocity command, so the tool has to
/// accelerate under the TCP acceleration ceiling and travel along the
/// axis it was given — not have the twist applied whole and the
/// smoothing happen somewhere that shapes the joints instead.
#[test]
fn a_jog_ramps_the_tool_under_its_ceilings_and_holds_its_axis() {
    let (mut exec, limits, dt) = setup();
    let start = at(0.35, 0.10, 0.20);
    exec.activate(&start);

    // Two thirds of the linear ceiling, on a diagonal.
    let axis = [0.6, -0.8, 0.0];
    let speed = limits.linear_velocity * (2.0 / 3.0);
    let twist = [
        axis[0] * speed,
        axis[1] * speed,
        axis[2] * speed,
        0.0,
        0.0,
        0.0,
    ];
    exec.set_twist(&twist).unwrap();

    let a = cart::translation(&start);
    let mut prev = a;
    let mut prev_speed = 0.0;
    let (mut worst_accel, mut peak, mut worst_off_axis) = (0.0f64, 0.0f64, 0.0f64);
    for _ in 0..4_000 {
        let p = cart::translation(&exec.step().unwrap().pose);
        let v = dist(p, prev) / dt;
        worst_accel = worst_accel.max((v - prev_speed).abs() / dt);
        peak = peak.max(v);
        // Travel has to stay on the commanded axis.
        let rel = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
        let along = rel[0] * axis[0] + rel[1] * axis[1] + rel[2] * axis[2];
        let foot = [
            a[0] + along * axis[0],
            a[1] + along * axis[1],
            a[2] + along * axis[2],
        ];
        worst_off_axis = worst_off_axis.max(dist(p, foot));
        prev_speed = v;
        prev = p;
        if (v - speed).abs() < 1e-9 {
            break;
        }
    }
    assert!(
        (prev_speed - speed).abs() < 1e-6,
        "the tool settled at {prev_speed} m/s, not the commanded {speed}"
    );
    assert!(
        worst_accel <= limits.linear_acceleration * 1.05,
        "the tool accelerated at {worst_accel} m/s² over a {} m/s² ceiling",
        limits.linear_acceleration
    );
    assert!(
        peak <= limits.linear_velocity * 1.01,
        "the tool ran at {peak} m/s over a {} m/s ceiling",
        limits.linear_velocity
    );
    assert!(
        worst_off_axis < 1e-9,
        "the tool wandered {worst_off_axis} m off its commanded axis"
    );

    // Releasing brings it to rest the same way, without reversing.
    exec.release();
    let mut last = prev;
    let mut at_rest = false;
    for _ in 0..4_000 {
        let p = cart::translation(&exec.step().unwrap().pose);
        let along = (p[0] - last[0]) * axis[0] + (p[1] - last[1]) * axis[1];
        assert!(along >= -1e-12, "the tool reversed while braking");
        let v = dist(p, last) / dt;
        last = p;
        if v < 1e-9 {
            at_rest = true;
            break;
        }
    }
    assert!(at_rest, "the jog never came to rest");
}
