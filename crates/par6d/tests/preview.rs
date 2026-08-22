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
use common::{repo_root, Client, Rig};

const BUDGET: Duration = Duration::from_secs(20);

/// The shipped config re-ticked to 50 Hz, shared verbatim by the daemon
/// AND the preview so the parity below is over identical inputs.
fn test_config() -> PathBuf {
    let src = repo_root().join("config/PAR6.toml");
    let dir = std::env::temp_dir().join(format!("par6d-preview-{}", std::process::id()));
    let grippers = dir.join("grippers");
    std::fs::create_dir_all(&grippers).expect("test config dir");
    let text = std::fs::read_to_string(&src).expect("read PAR6.toml");
    let patched = text.replace("tick_dt_s = 0.004", "tick_dt_s = 0.02");
    assert_ne!(patched, text, "tick_dt_s patch point must exist");
    let dst = dir.join("PAR6.toml");
    std::fs::write(&dst, patched).expect("write test config");
    for entry in std::fs::read_dir(src.parent().unwrap().join("grippers")).expect("grippers dir") {
        let e = entry.expect("dir entry");
        std::fs::copy(e.path(), grippers.join(e.file_name())).expect("copy gripper toml");
    }
    dst
}

fn assets() -> PathBuf {
    repo_root().join("assets/par6_description")
}

fn park_deg() -> [f64; NUM_JOINTS] {
    let cfg = par6_config::RobotConfig::load(&repo_root().join("config/PAR6.toml")).expect("cfg");
    let mut a = [0.0; NUM_JOINTS];
    for (out, rad) in a.iter_mut().zip(cfg.robot.park_pose_rad.iter()) {
        *out = rad.to_degrees();
    }
    a
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
