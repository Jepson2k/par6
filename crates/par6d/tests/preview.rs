//! Preview ↔ runtime parity: the offline preview plans with the daemon's
//! own planner and refuses with the server's own rules, so the two must
//! agree — same landing, same refusal text, same collision verdicts —
//! with no mirrored math anywhere.

use std::path::PathBuf;
use std::time::Duration;

use par6_proto::command::{Home, JogL, MoveJ, MoveL, SelectProfile, Stop, Teleport, WriteIo};
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

fn to_deg(rad: &[f64; NUM_JOINTS]) -> [f64; NUM_JOINTS] {
    let mut out = [0.0; NUM_JOINTS];
    for (o, r) in out.iter_mut().zip(rad.iter()) {
        *o = r.to_degrees();
    }
    out
}

fn max_deg_error(a: &[f64; NUM_JOINTS], b: &[f64; NUM_JOINTS]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

fn teleport_cmd(angles: [f64; NUM_JOINTS]) -> Command {
    Command::Teleport(Teleport {
        angles,
        tool_positions: None,
    })
}

fn teleport_home(rig: &Rig, c: &mut Client, angles: [f64; NUM_JOINTS]) {
    let deadline = std::time::Instant::now() + BUDGET;
    loop {
        c.send(&teleport_cmd(angles));
        let window = std::time::Instant::now() + Duration::from_millis(400);
        while std::time::Instant::now() < window {
            if let Some(s) = rig.recv_status() {
                if s.homed && max_deg_error(&s.angles, &angles) < 1.0 {
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

fn move_j_cmd(angles_deg: [f64; NUM_JOINTS], blend_mm: Option<f64>) -> Command {
    use std::sync::atomic::{AtomicU64, Ordering};
    static KEY: AtomicU64 = AtomicU64::new(4242);
    Command::MoveJ(MoveJ {
        key: KEY.fetch_add(1, Ordering::Relaxed),
        angles: angles_deg,
        duration: None,
        speed: Some(1.0),
        accel: None,
        blend_radius: blend_mm,
        rel: false,
    })
}

fn jog_l_cmd(velocities: [f64; 6], duration: f64) -> Command {
    Command::JogL(JogL {
        velocities,
        duration,
        frame: Frame::Wrf,
        accel: None,
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
    preview.place_rad(to_rad(&park));

    // -- a joint move lands in the same place on both sides ------------
    let mut target = park;
    target[0] += 20.0;
    let planned = preview.submit(move_j_cmd(target, None));
    assert!(planned.valid(), "preview refused a plain move: {planned:?}");
    assert!(!planned.pending, "an unblended move runs at once");
    assert!(
        planned.duration_s > 0.2,
        "a 20 degree move cannot be instant: {} s",
        planned.duration_s
    );
    let end_deg = to_deg(&planned.end_joints_rad);
    assert!(
        max_deg_error(&end_deg, &target) < 0.1,
        "the preview must land on the target: {end_deg:?} vs {target:?}"
    );
    assert_eq!(
        planned.joint_trajectory_rad.len(),
        planned.tcp_poses.len(),
        "every trajectory sample carries its FK pose"
    );

    let index = c.ok_index(&move_j_cmd(target, None));
    let (ok, detail) = c.wait_complete(index);
    assert!(ok, "the runtime must run the same move: {detail:?}");
    let landed = rig.wait_status("runtime landed", |s| {
        max_deg_error(&s.angles, &target) < 1.5
    });
    assert!(
        max_deg_error(&landed.angles, &end_deg) < 1.5,
        "preview and runtime landings diverge: {:?} vs {end_deg:?}",
        landed.angles
    );

    // -- server-side refusals are the runtime's own text ---------------
    // A teleport outside a joint's travel, a write to an output the box
    // does not declare, and an unknown profile: each refused here with
    // the identical code and cause the daemon answers with.
    let mut beyond = park;
    beyond[0] += 400.0;
    for cmd in [
        teleport_cmd(beyond),
        Command::WriteIo(WriteIo { port: 7, value: 1 }),
        Command::SelectProfile(SelectProfile {
            profile: "BANG_BANG".into(),
        }),
    ] {
        let mine = preview.submit(cmd.clone());
        let theirs = c.expect_error(&cmd);
        let err = mine
            .error
            .as_ref()
            .unwrap_or_else(|| panic!("the preview must refuse {cmd:?}"));
        assert_eq!(err.code, theirs.code, "{cmd:?}: {err:?} vs {theirs:?}");
        assert_eq!(err.cause, theirs.cause, "{cmd:?}");
    }
    assert!(
        max_deg_error(&to_deg(&preview.angles_rad()), &target) < 1e-9,
        "a refused teleport moves nothing"
    );

    // -- a keep-out refuses the same move on both sides ----------------
    // Park a box on the TCP position at the middle of the return sweep,
    // read off the PREVIEW's own FK (which the runtime's must match).
    let mut mid = park;
    mid[0] += 10.0;
    preview.place_rad(to_rad(&mid));
    let pose = preview.pose().expect("preview FK");
    let center_m = [pose[3], pose[7], pose[11]];
    preview.place_rad(to_rad(&target));
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
    let refused = preview.submit(move_j_cmd(park, None));
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
    let index = c.ok_index(&move_j_cmd(park, None));
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
    assert!(preview.submit(move_j_cmd(park, None)).valid());
    c.ok(&Command::SetShapes(par6_proto::command::SetShapes {
        shapes: vec![],
    }));
    let index = c.ok_index(&move_j_cmd(park, None));
    let (ok, detail) = c.wait_complete(index);
    assert!(ok, "{detail:?}");

    // -- a cartesian jog integrates to where the runtime's does --------
    // One `jog_l` datagram straight down for its whole watchdog window:
    // the preview steps the housekeeping integrator, the runtime streams
    // it, and both must lower the TCP by the same amount.
    let before = preview.pose().expect("FK")[11];
    let jogged = preview.submit(jog_l_cmd([0.0, 0.0, -1.0, 0.0, 0.0, 0.0], 0.5));
    assert!(jogged.valid(), "{jogged:?}");
    let drop_m = before - preview.pose().expect("FK")[11];
    assert!(
        drop_m > 0.02,
        "a half-second full-scale -Z jog must lower the TCP, got {drop_m} m"
    );
    let start = rig.wait_status("runtime at park", |s| max_deg_error(&s.angles, &park) < 1.5);
    c.send(&jog_l_cmd([0.0, 0.0, -1.0, 0.0, 0.0, 0.0], 0.5));
    assert!(
        c.try_recv().is_none(),
        "the runtime must admit the jog the preview admitted"
    );
    let moving = rig.wait_status("jog moving", |s| s.pose[11] < start.pose[11] - 5.0);
    std::thread::sleep(Duration::from_millis(900));
    // Frames from before the watchdog expired are still queued on the
    // socket; the rest pose is read off a fresh one.
    rig.drain_status();
    let rest = rig.wait_status("jog watchdog expired", |s| {
        s.seq > moving.seq && s.speeds.iter().all(|v| v.abs() < 0.05)
    });
    let runtime_drop_m = (start.pose[11] - rest.pose[11]) / 1000.0;
    assert!(
        (runtime_drop_m - drop_m).abs() < 0.005,
        "preview and runtime jog_l diverge: {drop_m} m vs {runtime_drop_m} m"
    );

    rig.shutdown();
}

#[test]
fn the_preview_runs_the_cartesian_pipeline() {
    let config = test_config();
    let mut preview = Preview::new(Some(&config), Some(&assets())).expect("preview boots");
    let park = park_deg();
    preview.place_rad(to_rad(&park));

    // A small straight -Z move through the real segment → IK → TOPPRA
    // pipeline: the trajectory must end 30 mm below where it started.
    let start = preview.pose().expect("FK");
    let planned = preview.submit(Command::MoveL(MoveL {
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
    let refused = preview.submit(Command::MoveL(MoveL {
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

    // A cartesian jog's setpoints are TRACKED by the streaming executor,
    // and `accel` scales how hard it tracks. A preview that stopped at
    // the setpoints reported the same travel whatever the caller asked
    // for, so a jog told to accelerate gently must cover less ground.
    let mut drop_at = |accel: Option<f64>| {
        preview.place_rad(to_rad(&park));
        let before = preview.pose().expect("FK")[11];
        let jogged = preview.submit(Command::JogL(JogL {
            velocities: [0.0, 0.0, -1.0, 0.0, 0.0, 0.0],
            duration: 0.4,
            frame: Frame::Wrf,
            accel,
        }));
        assert!(jogged.valid(), "{jogged:?}");
        before - preview.pose().expect("FK")[11]
    };
    let full = drop_at(Some(1.0));
    let gentle = drop_at(Some(0.01));
    assert!(full > 0.01, "a -Z jog must lower the TCP: {full} m");
    // The STREAM ceilings are loose next to jog_l's rates, so the
    // executor tracks the setpoints closely and the gap is a few per
    // cent rather than a fraction. The point is that there is one at
    // all: a preview that stopped at the setpoints reported identical
    // travel for every accel a caller could ask for.
    assert!(
        full - gentle > 0.01 * full,
        "a gently accelerated jog must cover less ground: {gentle} m vs {full} m"
    );
}

/// The server's blend hold, offline: a move asking to round its corner
/// waits for the successor, a stopping successor closes the chain into
/// one motion, a queue-clearing stop drops what waits, and a flush runs
/// what the end of a program leaves behind.
#[test]
fn blended_moves_hold_fold_and_flush_as_the_queue_does() {
    let config = test_config();
    let mut preview = Preview::new(Some(&config), Some(&assets())).expect("preview boots");
    let park = park_deg();
    preview.place_rad(to_rad(&park));
    let mut a = park;
    a[0] += 15.0;
    let mut b = a;
    b[1] += 10.0;

    let held = preview.submit(move_j_cmd(a, Some(20.0)));
    assert!(held.pending && held.valid(), "{held:?}");
    assert!(held.joint_trajectory_rad.is_empty());
    assert_eq!(preview.held_names(), vec!["move_j"]);
    assert_eq!(preview.angles_rad(), to_rad(&park), "nothing planned yet");

    let chain = preview.submit(move_j_cmd(b, None));
    assert!(!chain.pending && chain.valid(), "{chain:?}");
    assert!(preview.held_names().is_empty());
    assert!(
        max_deg_error(&to_deg(&chain.end_joints_rad), &b) < 0.1,
        "the chain ends at the stopping move's target"
    );
    let alone = {
        let mut p = Preview::new(Some(&config), Some(&assets())).expect("second preview");
        p.place_rad(to_rad(&park));
        let first = p.submit(move_j_cmd(a, None));
        let second = p.submit(move_j_cmd(b, None));
        first.duration_s + second.duration_s
    };
    assert!(
        chain.duration_s < alone,
        "a rounded corner is faster than stopping at it: {} vs {}",
        chain.duration_s,
        alone
    );
    // The corner is cut: the chain never passes through A itself.
    let nearest = chain
        .joint_trajectory_rad
        .iter()
        .map(|q| max_deg_error(&to_deg(q), &a))
        .fold(f64::INFINITY, f64::min);
    assert!(
        nearest > 0.5,
        "a blended chain must round the corner, came within {nearest} deg"
    );

    // A queue-clearing stop drops the hold; a plain stop keeps it.
    preview.place_rad(to_rad(&park));
    assert!(preview.submit(move_j_cmd(a, Some(20.0))).pending);
    assert!(preview
        .submit(Command::Stop(Stop { clear_queue: false }))
        .valid());
    assert_eq!(preview.held_names(), vec!["move_j"]);
    assert!(preview
        .submit(Command::Stop(Stop { clear_queue: true }))
        .valid());
    assert!(preview.held_names().is_empty());
    assert!(preview.flush().is_none(), "nothing left to plan");
    assert_eq!(
        preview.angles_rad(),
        to_rad(&park),
        "a dropped move never ran"
    );

    // A streamable preempts the hold too, and the flush at the end of a
    // program runs the remainder as a move that stops at its target.
    assert!(preview.submit(move_j_cmd(a, Some(20.0))).pending);
    assert!(preview.submit(teleport_cmd(park)).valid());
    assert!(preview.held_names().is_empty());
    assert!(preview.submit(move_j_cmd(a, Some(20.0))).pending);
    let flushed = preview.flush().expect("the flush plans the held move");
    assert!(flushed.valid() && !flushed.pending, "{flushed:?}");
    assert!(max_deg_error(&to_deg(&flushed.end_joints_rad), &a) < 0.1);
    assert!(flushed.duration_s > 0.1);
}

/// HOME is two commands wearing one name: the referencing seek on an
/// un-referenced arm, which ends where the configured sequence's
/// `move_to` steps leave it and has no planned duration, and a planned
/// return to the park pose once referenced.
#[test]
fn home_previews_as_a_seek_until_referenced_and_a_return_afterwards() {
    let config = test_config();
    let mut preview = Preview::new(Some(&config), Some(&assets())).expect("preview boots");
    let park = park_deg();
    let ready = preview.homing_ready_pose_rad();
    assert!(
        max_deg_error(&to_deg(&ready), &park) > 5.0,
        "the seek and the park pose must differ for this test to mean anything"
    );

    preview.place_rad(to_rad(&park));
    preview.set_homed(false);
    let gated = preview.submit(move_j_cmd(park, None));
    assert_eq!(
        gated.error.as_ref().map(|e| e.code),
        Some(ErrorCode::MotnNotHomed as u16),
        "planned motion is refused un-referenced: {gated:?}"
    );

    let seek = preview.submit(Command::Home(Home {
        key: 1,
        calibrate: false,
    }));
    assert!(seek.valid(), "{seek:?}");
    assert_eq!(seek.end_joints_rad, ready);
    assert_eq!(seek.duration_s, 0.0);
    assert!(preview.homed());

    let ret = preview.submit(Command::Home(Home {
        key: 2,
        calibrate: false,
    }));
    assert!(ret.valid(), "{ret:?}");
    assert!(
        ret.duration_s > 0.1,
        "a referenced HOME is a planned move, not a jump"
    );
    assert!(max_deg_error(&to_deg(&ret.end_joints_rad), &park) < 0.1);
    assert!(ret.tcp_poses.len() > 1, "a planned move draws a path");
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

fn jog_j_cmd(speeds: [f64; NUM_JOINTS], duration: f64) -> Command {
    Command::JogJ(par6_proto::command::JogJ {
        speeds,
        duration,
        accel: None,
    })
}

/// Two things the preview used to get wrong on its own, both checked
/// against the runtime: an e-stop is a latch that outlives the command
/// that set it, and a stream of jog datagrams is one acceleration rather
/// than one per frame.
#[test]
fn the_preview_latches_an_estop_and_streams_a_jog_the_way_the_runtime_does() {
    let config = test_config();
    let rig = Rig::boot(config.clone());
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);

    let mut preview = Preview::new(Some(&config), Some(&assets())).expect("preview boots");
    let park = park_deg();
    teleport_home(&rig, &mut c, park);
    preview.place_rad(to_rad(&park));

    // -- an e-stop latches until a reset -------------------------------
    let mut target = park;
    target[0] += 15.0;
    preview.submit(Command::Estop);
    c.ok(&Command::Estop);
    assert_eq!(
        preview.error().map(|e| e.code),
        Some(ErrorCode::SysEstopActive as u16),
        "an e-stop must stand as the preview's error"
    );

    let mine = preview.submit(move_j_cmd(target, None));
    let theirs = c.expect_error(&move_j_cmd(target, None));
    let err = mine
        .error
        .as_ref()
        .expect("a latched e-stop must refuse a move");
    assert_eq!(err.code, theirs.code, "{err:?} vs {theirs:?}");
    assert_eq!(err.cause, theirs.cause, "{err:?} vs {theirs:?}");
    assert_eq!(
        mine.end_joints_rad,
        to_rad(&park),
        "a refused move must not advance the virtual arm"
    );

    preview.submit(Command::Reset);
    c.ok(&Command::Reset);
    assert!(
        preview.error().is_none(),
        "a reset must clear the standing e-stop: {:?}",
        preview.error()
    );
    let after = preview.submit(move_j_cmd(target, None));
    assert!(after.valid(), "a reset must un-refuse the move: {after:?}");
    let index = c.ok_index(&move_j_cmd(target, None));
    let (ok, detail) = c.wait_complete(index);
    assert!(
        ok,
        "the runtime must run the move after a reset: {detail:?}"
    );
    rig.wait_status("runtime landed", |s| {
        max_deg_error(&s.angles, &target) < 1.5
    });

    // -- a streamed jog ramps once, not once per datagram ---------------
    // The RT accelerates on JOG mode ENTRY, so a UI sending frames at
    // 20 Hz gets one acceleration. Six frames of 0.15 s must therefore
    // travel what a single 0.9 s jog travels, not six ramps' worth.
    const FRAMES: usize = 6;
    // A whole number of ticks per frame, so the two runs integrate the
    // same number of them and only the ramp can separate them.
    let frame_s = 8.0 * preview.tick_dt_s();
    let mut speeds = [0.0; NUM_JOINTS];
    speeds[0] = 0.5;

    preview.place_rad(to_rad(&park));
    let one = preview.submit(jog_j_cmd(speeds, FRAMES as f64 * frame_s));
    assert!(one.valid(), "{one:?}");
    let single_deg = to_deg(&one.end_joints_rad)[0] - park[0];

    preview.place_rad(to_rad(&park));
    for _ in 0..FRAMES {
        let step = preview.submit(jog_j_cmd(speeds, frame_s));
        assert!(step.valid(), "{step:?}");
    }
    let streamed_deg = to_deg(&preview.angles_rad())[0] - park[0];
    assert!(
        single_deg > 1.0,
        "the jog must actually move joint 0: {single_deg} deg"
    );
    assert!(
        (streamed_deg - single_deg).abs() < 0.01 * single_deg,
        "a stream of jogs must travel what one long jog travels: \
         {streamed_deg} deg streamed vs {single_deg} deg in one"
    );

    // Against the runtime, one datagram of the same total duration: the
    // stream is already known to travel the same distance, and a single
    // send keeps wall-clock jitter out of the comparison. The ramp down
    // is included on both sides — releasing the jog leaves the RT in JOG
    // until the velocity reaches rest.
    preview.place_rad(to_rad(&park));
    let whole = preview.submit(jog_j_cmd(speeds, FRAMES as f64 * frame_s));
    assert!(whole.valid(), "{whole:?}");
    preview.submit(Command::Reset);
    let settled_deg = to_deg(&preview.angles_rad())[0] - park[0];
    assert!(
        settled_deg > single_deg,
        "the ramp down must cover ground: {settled_deg} vs {single_deg} deg"
    );

    teleport_home(&rig, &mut c, park);
    let start = rig.wait_status("runtime at rest", |s| {
        s.speeds.iter().all(|v| v.abs() < 0.05)
    });
    c.send(&jog_j_cmd(speeds, FRAMES as f64 * frame_s));
    std::thread::sleep(Duration::from_millis(600));
    rig.drain_status();
    let rest = rig.wait_status("jog watchdog expired", |s| {
        s.seq > start.seq && s.speeds.iter().all(|v| v.abs() < 0.05)
    });
    // The jog engine integrates from the MEASURED pose, so whatever the
    // servo loop fails to track in a tick is ground the jog never gets
    // back. The runtime therefore lands a little short of the preview,
    // which integrates its own output and so tracks perfectly; it never
    // lands beyond it.
    let runtime_deg = rest.angles[0] - start.angles[0];
    assert!(
        runtime_deg <= settled_deg,
        "the runtime cannot outrun a perfect-tracking preview: \
         {runtime_deg} deg vs {settled_deg} deg"
    );
    assert!(
        settled_deg - runtime_deg < 0.06 * settled_deg,
        "preview and runtime jogs diverge by more than the servo can lag: \
         {settled_deg} deg vs {runtime_deg} deg"
    );

    rig.shutdown();
}

/// `home(calibrate=True)` asks for the referencing seek on an arm that
/// already holds its references — a different motion from the planned
/// park return HOME otherwise means. The flag crosses the Python dict,
/// the wire conversion and the planner before anything can act on it, so
/// what is checked here is that it survives the trip.
#[test]
fn a_calibrating_home_seeks_even_when_the_arm_is_already_referenced() {
    let config = test_config();
    let mut preview = Preview::new(Some(&config), Some(&assets())).expect("preview boots");
    let park = park_deg();
    preview.place_rad(to_rad(&park));
    preview.set_homed(true);

    // Referenced: a plain HOME is a planned move back to park, which
    // takes time and traces a path.
    let plain = preview.submit(Command::Home(Home {
        key: 10,
        calibrate: false,
    }));
    assert!(plain.valid(), "{plain:?}");

    // Referenced, but asked to calibrate: the seek runs instead, ending
    // where the homing sequence leaves the arm rather than at park.
    let mut off = park;
    off[0] += 20.0;
    preview.place_rad(to_rad(&off));
    let seek = preview.submit(Command::Home(Home {
        key: 11,
        calibrate: true,
    }));
    assert!(seek.valid(), "{seek:?}");
    let ready = to_deg(&preview.homing_ready_pose_rad());
    assert!(
        max_deg_error(&to_deg(&seek.end_joints_rad), &ready) < 1e-6,
        "a calibrating home must end where the seek ends: {:?} vs {ready:?}",
        to_deg(&seek.end_joints_rad)
    );
    assert!(
        seek.duration_s == 0.0,
        "the seek's duration belongs to the physical sequence, not to a plan: {}",
        seek.duration_s
    );
}
