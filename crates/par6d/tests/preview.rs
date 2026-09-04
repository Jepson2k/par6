//! Preview ↔ runtime parity: the offline preview plans with the daemon's
//! own planner, so the two must agree — same landing, same refusals,
//! same collision verdicts — with no mirrored math anywhere.

use std::path::PathBuf;
use std::time::Duration;

use par6_proto::command::{MoveJ, MoveL, Teleport};
use par6_proto::{Command, ErrorCode, Frame, Shape, NUM_JOINTS};
use par6_server::ShapeLayer;
use par6d::preview::Preview;

mod common;
use common::{Client, Rig};

const BUDGET: Duration = Duration::from_secs(20);

/// The shipped config re-ticked to 50 Hz, shared verbatim by the daemon
/// AND the preview so the parity below is over identical inputs.
fn test_config() -> PathBuf {
    common::retimed_config("preview", 0.02)
}

fn assets() -> PathBuf {
    common::assets_dir()
}

fn park_deg() -> [f64; NUM_JOINTS] {
    common::park_deg()
}

/// A pose with the wrist clear of its singularity: park folds J5 to 0,
/// where a rotation about world z has no IK branch and a damped
/// least-squares jog drifts off its axis.
fn wrist_clear_deg() -> [f64; NUM_JOINTS] {
    [0.0, -60.0, 150.0, 0.0, 45.0, 180.0]
}

fn to_rad(deg: &[f64; NUM_JOINTS]) -> [f64; NUM_JOINTS] {
    let mut out = [0.0; NUM_JOINTS];
    for (o, d) in out.iter_mut().zip(deg.iter()) {
        *o = d.to_radians();
    }
    out
}

fn teleport_home(rig: &Rig, c: &mut Client, angles: [f64; NUM_JOINTS]) {
    let deadline = std::time::Instant::now() + BUDGET;
    loop {
        c.send(&Command::Teleport(Teleport {
            angles,
            tool_positions: None,
        }));
        let window = std::time::Instant::now() + Duration::from_millis(400);
        while std::time::Instant::now() < window {
            if let Some(s) = rig.recv_status() {
                let close = s
                    .angles
                    .iter()
                    .zip(angles.iter())
                    .all(|(a, b)| (a - b).abs() < 1.0);
                if s.homed && close {
                    return;
                }
            }
        }
        assert!(
            std::time::Instant::now() < deadline,
            "teleport did not take effect within budget"
        );
    }
}

fn move_j_cmd(angles_deg: [f64; NUM_JOINTS]) -> Command {
    use std::sync::atomic::{AtomicU64, Ordering};
    static KEY: AtomicU64 = AtomicU64::new(4242);
    Command::MoveJ(MoveJ {
        key: KEY.fetch_add(1, Ordering::Relaxed),
        angles: angles_deg,
        duration: None,
        speed: Some(1.0),
        accel: None,
        blend_radius: None,
        rel: false,
    })
}

#[test]
fn the_preview_and_the_runtime_agree_on_moves_and_refusals() {
    let config = test_config();
    let rig = Rig::boot(config.clone());
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);

    let mut preview = Preview::new(Some(&config), Some(&assets())).expect("preview boots");
    let park = park_deg();
    teleport_home(&rig, &mut c, park);
    preview.teleport_rad(to_rad(&park));

    // -- a joint move lands in the same place on both sides ------------
    let mut target = park;
    target[0] += 20.0;
    let planned = preview.preview(move_j_cmd(target));
    assert!(planned.valid(), "preview refused a plain move: {planned:?}");
    assert!(
        planned.duration_s > 0.2,
        "a 20 degree move cannot be instant: {} s",
        planned.duration_s
    );
    let end_deg: Vec<f64> = planned
        .end_joints_rad
        .iter()
        .map(|r| r.to_degrees())
        .collect();
    assert!(
        end_deg
            .iter()
            .zip(target.iter())
            .all(|(a, b)| (a - b).abs() < 0.1),
        "the preview must land on the target: {end_deg:?} vs {target:?}"
    );
    assert_eq!(
        planned.joint_trajectory_rad.len(),
        planned.tcp_poses.len(),
        "every trajectory sample carries its FK pose"
    );

    let index = c.ok_index(&move_j_cmd(target));
    let (ok, detail) = c.wait_complete(index);
    assert!(ok, "the runtime must run the same move: {detail:?}");
    let landed = rig.wait_status("runtime landed", |s| {
        s.angles
            .iter()
            .zip(target.iter())
            .all(|(a, b)| (a - b).abs() < 1.5)
    });
    for (a, b) in landed.angles.iter().zip(end_deg.iter()) {
        assert!(
            (a - b).abs() < 1.5,
            "preview and runtime landings diverge: {:?} vs {end_deg:?}",
            landed.angles
        );
    }

    // -- a keep-out refuses the same move on both sides ----------------
    // Park a box on the TCP position at the middle of the return sweep,
    // read off the PREVIEW's own FK (which the runtime's must match).
    let mut mid = park;
    mid[0] += 10.0;
    preview.teleport_rad(to_rad(&mid));
    let pose = preview.pose().expect("preview FK");
    let center_m = [pose[3], pose[7], pose[11]];
    preview.teleport_rad(to_rad(&target));
    let keepout = vec![Shape {
        kind: "box".into(),
        params: vec![0.1, 0.1, 0.1],
        pose: vec![center_m[0], center_m[1], center_m[2], 0.0, 0.0, 0.0],
        collision: true,
        margin: None,
        name: "keepout".into(),
    }];

    preview
        .set_shapes(ShapeLayer::Program, &keepout)
        .expect("preview applies the keep-out");
    let refused = preview.preview(move_j_cmd(park));
    let err = refused
        .error
        .as_ref()
        .expect("the preview must refuse the move through the keep-out");
    assert_eq!(err.code, ErrorCode::SysSelfCollision as u16, "{err:?}");
    assert!(err.cause.contains("keepout"), "{err:?}");
    assert_eq!(
        refused.end_joints_rad,
        to_rad(&target),
        "a refused move must not advance the virtual arm"
    );

    c.ok(&Command::SetShapes(par6_proto::command::SetShapes {
        shapes: keepout,
    }));
    let index = c.ok_index(&move_j_cmd(park));
    let (ok, detail) = c.wait_complete(index);
    assert!(
        !ok,
        "the runtime must refuse exactly what the preview refused"
    );
    let err = detail.expect("a refused move carries its error");
    assert_eq!(err.code, ErrorCode::SysSelfCollision as u16, "{err:?}");
    assert!(err.cause.contains("keepout"), "{err:?}");

    // -- clearing the world un-refuses both sides ----------------------
    preview
        .set_shapes(ShapeLayer::Program, &[])
        .expect("preview clears the world");
    assert!(preview.preview(move_j_cmd(park)).valid());
    c.ok(&Command::SetShapes(par6_proto::command::SetShapes {
        shapes: vec![],
    }));
    let index = c.ok_index(&move_j_cmd(park));
    let (ok, detail) = c.wait_complete(index);
    assert!(ok, "{detail:?}");

    rig.shutdown();
}

#[test]
fn the_preview_runs_the_cartesian_pipeline() {
    let config = test_config();
    let mut preview = Preview::new(Some(&config), Some(&assets())).expect("preview boots");
    let park = park_deg();
    preview.teleport_rad(to_rad(&park));

    // A small straight -Z move through the real segment → IK → TOPPRA
    // pipeline: the trajectory must end 30 mm below where it started.
    let start = preview.pose().expect("FK");
    let planned = preview.preview(Command::MoveL(MoveL {
        key: 7,
        pose: [0.0, 0.0, -30.0, 0.0, 0.0, 0.0],
        frame: Frame::Wrf,
        duration: None,
        speed: Some(0.5),
        accel: None,
        blend_radius: None,
        rel: true,
    }));
    assert!(planned.valid(), "{planned:?}");
    let end = preview.pose().expect("FK");
    let dz = end[11] - start[11];
    assert!(
        (dz + 0.030).abs() < 0.002,
        "the straight move must descend 30 mm, got {:.1} mm",
        dz * 1e3
    );
    assert!(
        planned.joint_trajectory_rad.len() > 5,
        "a 30 mm cartesian move is many ticks long"
    );

    // An unreachable pose is the runtime's structured refusal.
    let refused = preview.preview(Command::MoveL(MoveL {
        key: 8,
        pose: [5000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        frame: Frame::Wrf,
        duration: None,
        speed: Some(0.5),
        accel: None,
        blend_radius: None,
        rel: false,
    }));
    let err = refused.error.expect("an unreachable pose must be refused");
    assert!(
        err.code == ErrorCode::IkTargetUnreachable as u16
            || err.code == ErrorCode::IkPartialPath as u16,
        "{err:?}"
    );
}

/// The jog preview runs the runtime's own ramp: a diagonal jog moves
/// both commanded joints in their commanded directions, and the wire's
/// validation refusals (over-unity speed, an over-long watchdog) come
/// back as the runtime's structured errors without a daemon anywhere.
#[test]
fn the_preview_jogs_and_refuses_what_the_wire_refuses() {
    let config = test_config();
    let mut preview = Preview::new(Some(&config), Some(&assets())).expect("preview");
    let start = preview.angles_rad();

    let jogged = preview.preview_jog([0.4, 0.0, 0.0, -0.4, 0.0, 0.0], 0.4, None);
    assert!(jogged.valid(), "{jogged:?}");
    let end = jogged.end_joints_rad;
    assert!(end[0] > start[0] + 0.01, "J0 must jog forward");
    assert!(end[3] < start[3] - 0.01, "J3 must jog back");
    assert!(jogged.duration_s > 0.3, "{}", jogged.duration_s);
    assert_eq!(
        preview.angles_rad(),
        end,
        "the virtual arm advances to where the jog ends"
    );

    for (speeds, duration) in [
        ([1.5, 0.0, 0.0, 0.0, 0.0, 0.0], 0.4),
        ([0.2, 0.0, 0.0, 0.0, 0.0, 0.0], 100.0),
    ] {
        let refused = preview.preview_jog(speeds, duration, None);
        let err = refused.error.expect("the wire refuses this jog");
        assert_eq!(err.code, ErrorCode::CommValidationError as u16, "{err:?}");
        assert_eq!(refused.end_joints_rad, end, "a refused jog moves nothing");
    }
}

/// The servo preview is the runtime's limiter offline: a step target
/// from the middle of a joint's range is reached without overshoot,
/// the commanded velocity never exceeds the STREAM ceiling, a fraction
/// scales that ceiling, and a target past the soft limit is clamped to
/// it — measured on the same executor and clamp the RT core ticks.
#[test]
fn the_servo_preview_runs_the_limiter_from_the_virtual_pose() {
    let config = test_config();
    let robot = par6_config::RobotConfig::load(&config).expect("config");
    let mut preview = Preview::new(Some(&config), Some(&assets())).expect("preview");
    let lim = &robot.joints[0].limits;
    let mid = [
        (lim.soft_min_rad + lim.soft_max_rad) / 2.0,
        preview.angles_rad()[1],
        preview.angles_rad()[2],
        preview.angles_rad()[3],
        preview.angles_rad()[4],
        preview.angles_rad()[5],
    ];
    preview.teleport_rad(mid);
    let v_stream = lim.for_mode(par6_config::LimitMode::Stream).velocity_rad_s;

    let mut target = mid;
    target[0] += 0.5;
    let r = preview.preview_servo(&[target], 400, None, None);
    assert_eq!(r.q.len(), 400);
    let finished = r.finished_tick.expect("a 0.5 rad step settles inside 8 s");
    assert!(
        finished > 5,
        "a jerk-limited step takes more than a few ticks"
    );
    let peak_v = r.qd.iter().map(|v| v[0].abs()).fold(0.0, f64::max);
    assert!(
        peak_v <= v_stream * 1.001 && peak_v > 0.2 * v_stream,
        "peak {peak_v} rad/s must use the STREAM ceiling {v_stream} without exceeding it"
    );
    let overshoot = r.q.iter().map(|q| q[0] - target[0]).fold(0.0, f64::max);
    assert!(overshoot < 1e-6, "overshoot {overshoot} rad");
    assert!(
        (r.q[399][0] - target[0]).abs() < 1e-6,
        "settled at the target"
    );
    for q in &r.q {
        assert_eq!(q[1..], mid[1..], "an untargeted joint never moves");
    }

    let slow = preview.preview_servo(&[target], 400, Some(0.25), None);
    let slow_peak = slow.qd.iter().map(|v| v[0].abs()).fold(0.0, f64::max);
    assert!(
        slow_peak <= 0.25 * v_stream * 1.001 && slow_peak > 0.1 * v_stream,
        "the speed fraction scales the ceiling: {slow_peak} vs {v_stream}"
    );

    let mut beyond = mid;
    beyond[0] = lim.soft_max_rad + 1.0;
    let r = preview.preview_servo(&[beyond], 600, None, None);
    let end = r.q.last().unwrap()[0];
    assert!(
        (end - lim.soft_max_rad).abs() < 1e-6 && end < beyond[0],
        "a target past the soft limit is clamped to it: {end} vs {}",
        lim.soft_max_rad
    );
    assert_eq!(preview.angles_rad(), mid, "the virtual arm does not move");
}

/// The cartesian jog preview integrates the runtime's own twist: a +x
/// jog in the world frame moves the TCP along +x and nothing else, and a
/// wire-invalid request comes back as the result's error.
#[test]
fn the_preview_jogs_cartesian_through_the_runtime_kinematics() {
    let config = test_config();
    let mut preview = Preview::new(Some(&config), Some(&assets())).expect("preview boots");
    preview.teleport_rad(to_rad(&wrist_clear_deg()));

    let r = preview.preview_jog_l(
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        par6_proto::Frame::Wrf,
        0.5,
        None,
    );
    assert!(r.valid(), "a plain +x jog previews: {:?}", r.error);
    assert_eq!(r.joint_trajectory_rad.len(), r.tcp_poses.len());
    let (first, last) = (r.tcp_poses[0], r.tcp_poses[r.tcp_poses.len() - 1]);
    let dx = last[3] - first[3];
    let dy = (last[7] - first[7]).abs();
    let dz = (last[11] - first[11]).abs();
    assert!(
        dx > 0.02,
        "half a second of full-scale +x jog must travel: {dx} m"
    );
    assert!(
        dy < 0.003 && dz < 0.003,
        "and only along x: dy {dy} dz {dz}"
    );

    let refused = preview.preview_jog_l([f64::NAN; 6], par6_proto::Frame::Wrf, 0.5, None);
    assert!(!refused.valid(), "a NaN twist is refused at the wire");
}

/// A first waypoint that only reorients the tool is a real segment: the
/// path snaps it to the start only when it is within the path metric of
/// the start, rotation weighted, not when its translation alone is small.
#[test]
fn a_pure_reorientation_first_waypoint_is_not_dropped() {
    let config = test_config();
    let mut preview = Preview::new(Some(&config), Some(&assets())).expect("preview boots");
    preview.teleport_rad(to_rad(&wrist_clear_deg()));

    // Relative to the current pose: first turn 20 deg about z in place,
    // then translate 50 mm along x.
    let r = preview.preview(Command::MoveP(par6_proto::command::MoveP {
        key: 9901,
        waypoints: vec![
            [0.0, 0.0, 0.0, 0.0, 0.0, 20.0],
            [50.0, 0.0, 0.0, 0.0, 0.0, 20.0],
        ],
        frame: par6_proto::Frame::Wrf,
        duration: None,
        speed: Some(0.5),
        accel: None,
        rel: true,
    }));
    assert!(
        r.valid(),
        "a rotate-then-translate path previews: {:?}",
        r.error
    );
    let start = r.tcp_poses[0];
    let translation = |p: &[f64; 16]| {
        ((p[3] - start[3]).powi(2) + (p[7] - start[7]).powi(2) + (p[11] - start[11]).powi(2)).sqrt()
    };
    // Rotation angle between the start orientation and `p`.
    let rotation = |p: &[f64; 16]| {
        let mut trace = 0.0;
        for r in 0..3 {
            for k in 0..3 {
                trace += start[r * 4 + k] * p[r * 4 + k];
            }
        }
        ((trace - 1.0) / 2.0).clamp(-1.0, 1.0).acos().to_degrees()
    };
    let moving = r
        .tcp_poses
        .iter()
        .find(|p| translation(p) > 0.005)
        .expect("the path translates 50 mm");
    assert!(
        rotation(moving) > 15.0,
        "the reorientation must complete before the translation starts: {:.1} deg turned \
         when the TCP first left the start",
        rotation(moving)
    );
}
