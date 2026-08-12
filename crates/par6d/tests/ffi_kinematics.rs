//! Kinematics-backed runtime (feature `ffi`), driven end-to-end through
//! the real protocol-v2 encoding over UDP against `par6d --sim`:
//!
//! - the gravity hook does physical work: on the torque-level sim plant
//!   an IDLE arm holds its pose only while G(q) is fed forward,
//! - the FK hook publishes the true TCP pose: STATUS reproduces the
//!   golden kinematics fixture's FK matrix for a known q,
//! - `move_l` runs the cartesian pipeline (segment → seeded IK → TOPPRA
//!   → ring) to COMPLETE, and the measured TCP stays on the line,
//! - an out-of-workspace pose is a real IK error reply, never a no-op.
#![cfg(feature = "ffi")]

use std::net::{SocketAddr, UdpSocket};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use par6_proto::command::{JogL, MoveJPose, MoveL, Teleport};
use par6_proto::{
    decode_reply, decode_status, encode_command, Command, ErrorCode, Frame, Reply, Status,
    WireError, NUM_JOINTS,
};
use par6d::options::StatusTransport;
use par6d::{Daemon, Options};

const BUDGET: Duration = Duration::from_secs(30);
const READ_TIMEOUT: Duration = Duration::from_millis(100);

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("repo root")
}

fn assets_dir() -> PathBuf {
    repo_root().join("assets/par6_description")
}

fn config_path() -> PathBuf {
    repo_root().join("config/PAR6.toml")
}

/// The PAR6 config re-ticked to 50 Hz, like the sim-session test: loaded
/// CI machines without RT scheduling miss 4 ms deadlines and would latch
/// LOOP_CRITICAL. Every RT time constant derives from config seconds, so
/// the wiring under test is identical.
fn test_config(tag: &str) -> PathBuf {
    let src = config_path();
    let dir = std::env::temp_dir().join(format!("par6d-ffi-{tag}-{}", std::process::id()));
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

// ---- in-process rig --------------------------------------------------------

struct Rig {
    daemon: Option<Daemon>,
    status_rx: UdpSocket,
    _telemetry_rx: UdpSocket,
}

impl Rig {
    fn boot(tag: &str, sim_dynamics: bool) -> Rig {
        let _ = env_logger::builder().is_test(true).try_init();
        let status_rx = UdpSocket::bind("127.0.0.1:0").expect("status socket");
        status_rx
            .set_read_timeout(Some(READ_TIMEOUT))
            .expect("timeout");
        let telemetry_rx = UdpSocket::bind("127.0.0.1:0").expect("telemetry socket");
        let opts = Options {
            sim: true,
            sim_dynamics,
            config: Some(test_config(tag)),
            assets: Some(assets_dir()),
            command_port: Some(0),
            bind: Some("127.0.0.1".parse().unwrap()),
            status_host: Some("127.0.0.1".parse().unwrap()),
            status_port: Some(status_rx.local_addr().unwrap().port()),
            telemetry_port: Some(telemetry_rx.local_addr().unwrap().port()),
            status_transport: Some(StatusTransport::Unicast),
            ..Options::default()
        };
        Rig {
            daemon: Some(Daemon::start(&opts).expect("daemon boots in sim mode")),
            status_rx,
            _telemetry_rx: telemetry_rx,
        }
    }

    fn addr(&self) -> SocketAddr {
        self.daemon.as_ref().expect("running").command_addr()
    }

    fn recv_status(&self) -> Option<Status> {
        let mut buf = [0u8; 65535];
        match self.status_rx.recv_from(&mut buf) {
            Ok((n, _)) => Some(decode_status(&buf[..n]).expect("decodable status")),
            Err(e) if is_timeout(&e) => None,
            Err(e) => panic!("status recv failed: {e}"),
        }
    }

    fn wait_status(&self, what: &str, pred: impl Fn(&Status) -> bool) -> Status {
        let deadline = Instant::now() + BUDGET;
        let mut last: Option<Status> = None;
        loop {
            if let Some(s) = self.recv_status() {
                if pred(&s) {
                    return s;
                }
                last = Some(s);
            }
            assert!(
                Instant::now() < deadline,
                "status condition `{what}` not met within budget; last: {last:?}"
            );
        }
    }

    /// Collect every STATUS arriving in `window`.
    fn collect_status(&self, window: Duration) -> Vec<Status> {
        let until = Instant::now() + window;
        let mut out = Vec::new();
        while Instant::now() < until {
            if let Some(s) = self.recv_status() {
                out.push(s);
            }
        }
        out
    }

    fn shutdown(mut self) {
        self.daemon.take().expect("running").shutdown();
    }
}

fn is_timeout(e: &std::io::Error) -> bool {
    matches!(
        e.kind(),
        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
    )
}

// ---- protocol client -------------------------------------------------------

struct Client {
    sock: UdpSocket,
    server: SocketAddr,
    next_req: u32,
    completes: Vec<(u64, bool, Option<WireError>)>,
}

impl Client {
    fn new(server: SocketAddr) -> Client {
        let sock = UdpSocket::bind("127.0.0.1:0").expect("client socket");
        sock.set_read_timeout(Some(READ_TIMEOUT)).expect("timeout");
        Client {
            sock,
            server,
            next_req: 1,
            completes: Vec::new(),
        }
    }

    fn send(&mut self, cmd: &Command) -> u32 {
        let req_id = self.next_req;
        self.next_req += 1;
        let mut buf = Vec::new();
        encode_command(cmd, req_id, &mut buf).expect("encodable command");
        self.sock.send_to(&buf, self.server).expect("send");
        req_id
    }

    fn try_recv(&mut self) -> Option<Reply> {
        let mut buf = [0u8; 65535];
        match self.sock.recv_from(&mut buf) {
            Ok((n, _)) => Some(decode_reply(&buf[..n]).expect("decodable reply")),
            Err(e) if is_timeout(&e) => None,
            Err(e) => panic!("client recv failed: {e}"),
        }
    }

    fn request(&mut self, cmd: &Command) -> Reply {
        let req_id = self.send(cmd);
        let deadline = Instant::now() + BUDGET;
        loop {
            if let Some(r) = self.try_recv() {
                match &r {
                    Reply::Ok { req_id: id, .. }
                    | Reply::Error { req_id: id, .. }
                    | Reply::Response { req_id: id, .. }
                        if *id == req_id =>
                    {
                        return r;
                    }
                    Reply::Complete { index, ok, detail } => {
                        self.completes.push((*index, *ok, detail.clone()));
                    }
                    _ => {}
                }
            }
            assert!(
                Instant::now() < deadline,
                "no reply to {cmd:?} within budget"
            );
        }
    }

    fn ok(&mut self, cmd: &Command) {
        match self.request(cmd) {
            Reply::Ok { .. } => {}
            other => panic!("expected OK, got {other:?}"),
        }
    }

    fn ok_index(&mut self, cmd: &Command) -> u64 {
        match self.request(cmd) {
            Reply::Ok { index: Some(i), .. } => i,
            other => panic!("expected OK with index, got {other:?}"),
        }
    }

    fn wait_complete(&mut self, index: u64) -> (bool, Option<WireError>) {
        if let Some(pos) = self.completes.iter().position(|c| c.0 == index) {
            let (_, ok, detail) = self.completes.remove(pos);
            return (ok, detail);
        }
        let deadline = Instant::now() + BUDGET;
        loop {
            if let Some(Reply::Complete {
                index: i,
                ok,
                detail,
            }) = self.try_recv()
            {
                if i == index {
                    return (ok, detail);
                }
                self.completes.push((i, ok, detail));
            }
            assert!(
                Instant::now() < deadline,
                "no COMPLETE for index {index} within budget"
            );
        }
    }
}

/// Put the sim arm at `angles_deg` (teleport is the sim's fast homing)
/// and wait for the broadcast to show it there. One datagram per
/// attempt with a generous confirmation window: a duplicate teleport
/// applies later on the RT thread and would jerk the arm mid-test.
fn enable_and_teleport(rig: &Rig, c: &mut Client, angles_deg: [f64; NUM_JOINTS]) {
    let deadline = Instant::now() + BUDGET;
    loop {
        c.send(&Command::Teleport(Teleport {
            angles: angles_deg,
            tool_positions: None,
        }));
        let window = Instant::now() + Duration::from_secs(3);
        while Instant::now() < window {
            if let Some(s) = rig.recv_status() {
                let close = s
                    .angles
                    .iter()
                    .zip(angles_deg.iter())
                    .all(|(a, b)| (a - b).abs() < 1.0);
                if s.homed && close {
                    return;
                }
            }
        }
        assert!(
            Instant::now() < deadline,
            "teleport did not take effect within budget"
        );
    }
}

// ---- golden kinematics fixture ---------------------------------------------

#[derive(serde::Deserialize)]
struct Fixture {
    cases: Vec<FixtureCase>,
}

#[derive(serde::Deserialize)]
struct FixtureCase {
    q: [f64; NUM_JOINTS],
    /// Row-major 4x4 TCP pose \[m\] from the Python/Pinocchio reference.
    fk: Vec<f64>,
}

/// The golden fixture for the URDF variant the configured gripper
/// selects — the same file `par6-kin`'s conformance test uses.
fn golden_fixture() -> Fixture {
    let gripper = par6_config::RobotConfig::load(&config_path())
        .expect("PAR6 config")
        .robot
        .active_gripper;
    let name = if gripper.eq_ignore_ascii_case("flange") {
        "par6_flange"
    } else if gripper.starts_with("MSG") {
        "par6_msg"
    } else if gripper.starts_with("SSG48") {
        "par6_ssg48"
    } else {
        panic!("no golden fixture for configured gripper {gripper}");
    };
    let path = repo_root().join(format!("tests/golden/kinematics/{name}.json"));
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    serde_json::from_str(&text).expect("golden kinematics fixture")
}

/// The first golden case the arm can actually be teleported to: the
/// fixtures sample the whole joint space, teleport clamps to the hard
/// window, and a clamped pose would no longer match the golden FK.
fn reachable_golden_case() -> FixtureCase {
    let cfg = par6_config::RobotConfig::load(&config_path()).expect("PAR6 config");
    golden_fixture()
        .cases
        .into_iter()
        .find(|c| {
            c.q.iter()
                .zip(cfg.joints.iter())
                .all(|(q, j)| *q >= j.limits.hard_min_rad && *q <= j.limits.hard_max_rad)
        })
        .expect("at least one golden case inside the hard joint window")
}

// ---- cartesian geometry helpers --------------------------------------------

fn tcp_mm(s: &Status) -> [f64; 3] {
    [s.pose[3], s.pose[7], s.pose[11]]
}

/// Distance \[mm\] from `p` to the segment `a`→`b`.
fn distance_to_segment(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let len2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    let w = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let t = ((w[0] * d[0] + w[1] * d[1] + w[2] * d[2]) / len2).clamp(0.0, 1.0);
    let e = [w[0] - t * d[0], w[1] - t * d[1], w[2] - t * d[2]];
    (e[0] * e[0] + e[1] * e[1] + e[2] * e[2]).sqrt()
}

/// Euclidean distance \[mm\] between two TCP positions.
fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

/// Fraction of the segment `a`→`b` covered by `p`'s projection.
fn progress_along(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let len2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    let w = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    (w[0] * d[0] + w[1] * d[1] + w[2] * d[2]) / len2
}

/// Wire pose `[x y z mm, rx ry rz deg]` from a STATUS pose matrix
/// (row-major 4x4, mm, `R = Rz·Ry·Rx`) with the translation replaced.
fn wire_pose_at(pose: &[f64; 16], xyz_mm: [f64; 3]) -> [f64; 6] {
    let (r00, r10, r20, r21, r22) = (pose[0], pose[4], pose[8], pose[9], pose[10]);
    [
        xyz_mm[0],
        xyz_mm[1],
        xyz_mm[2],
        r21.atan2(r22).to_degrees(),
        (-r20).atan2(r00.hypot(r10)).to_degrees(),
        r10.atan2(r00).to_degrees(),
    ]
}

/// A well-conditioned start posture for cartesian moves: away from the
/// wrist-aligned park singularity, comfortably inside every soft window.
const CART_START_DEG: [f64; NUM_JOINTS] = [0.0, -70.0, 150.0, 20.0, 45.0, 180.0];
/// Cartesian move duration \[s\]. Long enough that the sim's cascade
/// tracking lag stays small next to the path tolerances.
const MOVE_S: f64 = 6.0;

// ---- tests -----------------------------------------------------------------

/// The whole cartesian surface over one session: the FK hook publishes
/// the golden TCP pose, `move_l` holds the straight line where a
/// joint-space `move_j_pose` to the same target bows far off it,
/// `jog_l` drives the TCP through the jacobian, and an out-of-workspace
/// target fails both cartesian moves with IK_TARGET_UNREACHABLE instead
/// of moving the arm.
#[test]
fn cartesian_surface_over_protocol_v2() {
    let rig = Rig::boot("cart", false);
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    // --- FK hook: STATUS carries the golden TCP pose for a known q.
    let case = reachable_golden_case();
    let mut case_deg = [0.0; NUM_JOINTS];
    for (out, rad) in case_deg.iter_mut().zip(case.q.iter()) {
        *out = rad.to_degrees();
    }
    enable_and_teleport(&rig, &mut c, case_deg);
    // The arm reports through 14-bit encoders, so it lands within a
    // quantum (~2e-5 rad) of the commanded configuration, not on it.
    let s = rig.wait_status("pose for the golden configuration", |s| {
        s.angles
            .iter()
            .zip(case_deg.iter())
            .all(|(a, b)| (a - b).abs() < 0.01)
    });
    for (k, golden) in case.fk.iter().enumerate() {
        // Tolerances leave ~100x margin over that quantum, and are still
        // orders of magnitude below what any convention slip (frame, row
        // order, rpy composition) would cost.
        // Columns 3/7/11 are the translation (golden in m, wire in mm).
        let (want, tol) = if k % 4 == 3 && k < 12 {
            (golden * 1000.0, 0.05)
        } else {
            (*golden, 5e-4)
        };
        assert!(
            (s.pose[k] - want).abs() < tol,
            "STATUS pose element {k} = {} != golden FK {want} (whole matrix {:?})",
            s.pose[k],
            s.pose
        );
    }

    // --- move_l: the measured TCP stays on the commanded line.
    enable_and_teleport(&rig, &mut c, CART_START_DEG);
    let s = rig.wait_status("start pose", |_| true);
    let start = tcp_mm(&s);
    let target = [start[0] + 80.0, start[1], start[2] + 40.0];
    let wire_target = wire_pose_at(&s.pose, target);
    let move_l = Command::MoveL(MoveL {
        key: 1001,
        pose: wire_target,
        frame: Frame::Wrf,
        duration: Some(MOVE_S),
        speed: None,
        accel: None,
        blend_radius: None,
        rel: false,
    });
    let i = c.ok_index(&move_l);
    let path: Vec<[f64; 3]> = rig
        .collect_status(Duration::from_secs_f64(MOVE_S + 1.0))
        .iter()
        .map(tcp_mm)
        .collect();
    let (ok, detail) = c.wait_complete(i);
    assert!(ok, "move_l must complete ok, got {detail:?}");

    let moving: Vec<[f64; 3]> = path
        .into_iter()
        .filter(|p| progress_along(*p, start, target) > 0.05)
        .collect();
    assert!(
        moving.len() > 20,
        "expected a sampled trajectory, got {} moving samples",
        moving.len()
    );
    let line_dev = moving
        .iter()
        .map(|p| distance_to_segment(*p, start, target))
        .fold(0.0f64, f64::max);
    let reach = moving
        .iter()
        .map(|p| progress_along(*p, start, target))
        .fold(0.0f64, f64::max);
    assert!(
        line_dev < 8.0,
        "move_l left the commanded line by {line_dev:.2} mm"
    );
    assert!(
        reach > 0.8,
        "move_l covered only {:.0}% of the segment",
        reach * 100.0
    );

    // --- move_j_pose: same target through IK + the joint-space profile.
    // Its TCP path bows far off the line — which is what makes the
    // collinearity bound above a real measurement and not a truism.
    enable_and_teleport(&rig, &mut c, CART_START_DEG);
    let i = c.ok_index(&Command::MoveJPose(MoveJPose {
        key: 1002,
        pose: wire_target,
        duration: Some(MOVE_S),
        speed: None,
        accel: None,
        blend_radius: None,
    }));
    let joint_path: Vec<[f64; 3]> = rig
        .collect_status(Duration::from_secs_f64(MOVE_S + 1.0))
        .iter()
        .map(tcp_mm)
        .collect();
    let (ok, detail) = c.wait_complete(i);
    assert!(ok, "move_j_pose must complete ok, got {detail:?}");
    let joint_dev = joint_path
        .iter()
        .map(|p| distance_to_segment(*p, start, target))
        .fold(0.0f64, f64::max);
    assert!(
        joint_dev > 12.0,
        "the joint-space route to the same pose bowed only {joint_dev:.2} mm — \
         the move_l line tolerance proves nothing at this scale"
    );
    let end = rig.wait_status("settled after move_j_pose", |_| true);
    let reached = tcp_mm(&end);
    let miss = distance(reached, target);
    assert!(
        miss < 15.0,
        "move_j_pose IK target missed by {miss:.1} mm (reached {reached:?}, target {target:?})"
    );

    // --- jog_l: cartesian velocity streaming through the jacobian.
    enable_and_teleport(&rig, &mut c, CART_START_DEG);
    let before = tcp_mm(&rig.wait_status("pose before jog_l", |_| true));
    for _ in 0..6 {
        c.send(&Command::JogL(JogL {
            velocities: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            duration: 0.4,
            frame: Frame::Wrf,
            accel: None,
        }));
        std::thread::sleep(Duration::from_millis(50)); // client-side stream pacing
    }
    let jogged = rig.wait_status("jog_l drives the TCP along +x", |s| {
        tcp_mm(s)[0] > before[0] + 5.0
    });
    let drift = tcp_mm(&jogged);
    assert!(
        (drift[1] - before[1]).abs() < 8.0 && (drift[2] - before[2]).abs() < 8.0,
        "jog_l on +x alone moved the TCP off-axis: {before:?} -> {drift:?}"
    );
    rig.wait_status("jog_l self-terminates", |s| {
        s.speeds.iter().all(|v| v.abs() < 0.05)
    });

    // --- unreachable target: a real IK error on both cartesian moves,
    // and the arm does not move.
    enable_and_teleport(&rig, &mut c, CART_START_DEG);
    let unreachable = [2000.0, 0.0, 200.0, 0.0, 0.0, 0.0];
    let before = tcp_mm(&rig.wait_status("pose before the unreachable target", |_| true));
    for (label, cmd) in [
        (
            "move_j_pose",
            Command::MoveJPose(MoveJPose {
                key: 1003,
                pose: unreachable,
                duration: Some(1.0),
                speed: None,
                accel: None,
                blend_radius: None,
            }),
        ),
        (
            "move_l",
            Command::MoveL(MoveL {
                key: 1004,
                pose: unreachable,
                frame: Frame::Wrf,
                duration: Some(1.0),
                speed: None,
                accel: None,
                blend_radius: None,
                rel: false,
            }),
        ),
    ] {
        let i = c.ok_index(&cmd);
        let (ok, detail) = c.wait_complete(i);
        assert!(!ok, "{label} to an unreachable pose must fail");
        let e = detail.expect("a failed COMPLETE carries the error");
        assert_eq!(
            e.code,
            ErrorCode::IkTargetUnreachable as u16,
            "{label} must report IK_TARGET_UNREACHABLE, got {e:?}"
        );
    }
    let after = tcp_mm(&rig.wait_status("pose after the rejected targets", |_| true));
    for k in 0..3 {
        assert!(
            (after[k] - before[k]).abs() < 1.0,
            "a rejected cartesian target moved the arm: {before:?} -> {after:?}"
        );
    }

    rig.shutdown();
}

/// The gravity hook does physical work. On the torque-level plant
/// (`--sim-dynamics`) an IDLE arm is held by nothing but the G(q)
/// feedforward: the shoulder and elbow, which carry essentially the
/// whole weight of the arm, stay where the sim placed them. With the
/// `ZeroGravity` placeholder the same rig collapses — measured here,
/// the shoulder is 69° down and the elbow 108° over inside one second,
/// both against their endstops by the second — so this bound cannot
/// pass without the hook.
#[test]
fn gravity_hook_holds_the_arm_on_the_torque_plant() {
    /// Hold tolerance \[deg\] for the load-bearing joints.
    const HOLD_TOL: f64 = 2.5;
    /// Shoulder and elbow. The wrist joints are not asserted: their
    /// gravity load is a small residual that the sim driver's
    /// torque↔current path does not reproduce faithfully (J3 keeps
    /// creeping even perfectly compensated), which says nothing about
    /// the hook under test.
    const LOADED: [usize; 2] = [1, 2];

    let rig = Rig::boot("gravity", true);
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);

    // Enable, let the RT clear sequence settle, then enable again right
    // before the teleport: re-seeding the plant reboots the sim drivers,
    // and only the retry window a fresh reset opens brings them back up
    // enabled — otherwise the arm is limp and just falls.
    let placed = CART_START_DEG;
    c.ok(&Command::Reset);
    rig.collect_status(Duration::from_secs(2));
    c.ok(&Command::Reset);
    enable_and_teleport(&rig, &mut c, placed);

    // The loaded joints give way slightly before the feedforward
    // catches them; the hold is what happens after that.
    let settle = rig.collect_status(Duration::from_secs(1));
    let held = settle.last().expect("status while settling").clone();
    let give = LOADED
        .iter()
        .map(|&j| (held.angles[j] - placed[j]).abs())
        .fold(0.0f64, f64::max);
    assert!(
        give > 0.05,
        "the plant never loaded the arm ({give:.3}°) — gravity is not being simulated"
    );
    assert!(
        give < 10.0,
        "the arm left the pose it was placed at ({give:.2}°): {placed:?} -> {:?}",
        held.angles
    );

    let watch = rig.collect_status(Duration::from_secs(4));
    assert!(
        watch.len() > 40,
        "expected a stream of status during the hold, got {}",
        watch.len()
    );
    for s in &watch {
        assert!(
            s.error.is_none(),
            "unexpected error while holding: {:?}",
            s.error
        );
        for &j in &LOADED {
            assert!(
                (s.angles[j] - held.angles[j]).abs() < HOLD_TOL,
                "joint {j} moved {:.2}° off the held pose while IDLE ({:?} -> {:?})",
                s.angles[j] - held.angles[j],
                held.angles,
                s.angles
            );
        }
    }

    rig.shutdown();
}
