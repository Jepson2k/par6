//! Kinematics-backed runtime (feature `ffi`), driven end-to-end through
//! the real protocol-v2 encoding over UDP against `par6d --sim`:
//!
//! - the gravity hook does physical work: on the torque-level sim plant
//!   an IDLE arm holds its pose only while G(q) is fed forward,
//! - the FK hook publishes the true TCP pose: STATUS reproduces the
//!   golden kinematics fixture's FK matrix for a known q,
//! - `move_l` runs the cartesian pipeline (segment → seeded IK → TOPPRA
//!   → ring) to COMPLETE, and the measured TCP stays on the line,
//! - an out-of-workspace pose is a real IK error reply, never a no-op,
//! - the collision world is enforced: a planned move through a keep-out
//!   is refused before dispatch, STATUS reports the live verdict, and a
//!   malformed shape set changes neither the epoch nor the enforced
//!   world.
#![cfg(feature = "ffi")]

use std::net::{SocketAddr, UdpSocket};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use par6_proto::command::{
    JogL, MoveC, MoveJ, MoveJPose, MoveL, MoveP, MoveS, SetShapes, SetTcpOffset, Shape, Teleport,
};
use par6_proto::{
    decode_reply, decode_status, encode_command, Command, ErrorCode, Frame, QueryResult, Reply,
    Status, WireError, NUM_JOINTS,
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

    /// Drop the STATUS frames the socket has already buffered.
    ///
    /// The broadcast runs at 50 Hz whatever the test is doing, so by the
    /// time a multi-second move ends there is a queue of history waiting:
    /// a stale frame can satisfy a wait for a condition that has not
    /// happened yet. Draining first makes the next frame a live one.
    fn drain_status(&self) {
        self.status_rx
            .set_nonblocking(true)
            .expect("nonblocking status socket");
        let mut buf = [0u8; 65535];
        while self.status_rx.recv_from(&mut buf).is_ok() {}
        self.status_rx
            .set_nonblocking(false)
            .expect("blocking status socket");
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

    fn query(&mut self, cmd: &Command) -> QueryResult {
        match self.request(cmd) {
            Reply::Response { result, .. } => result,
            other => panic!("expected RESPONSE, got {other:?}"),
        }
    }

    fn expect_error(&mut self, cmd: &Command) -> WireError {
        match self.request(cmd) {
            Reply::Error { error, .. } => error,
            other => panic!("expected ERROR, got {other:?}"),
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
/// (row-major 4x4, mm) with the translation replaced.
///
/// Decoded the way a client decodes it — `pinokin.so3_rpy`, the wire's
/// intrinsic-XYZ convention (`spec/PROTOCOL-V2.md`), written out here
/// rather than borrowed from the runtime so the two halves of the
/// round trip cannot agree on the wrong thing.
fn wire_pose_at(pose: &[f64; 16], xyz_mm: [f64; 3]) -> [f64; 6] {
    let (r00, r01, r02) = (pose[0], pose[1], pose[2]);
    let (r12, r22) = (pose[6], pose[10]);
    let cp = r12.hypot(r22);
    [
        xyz_mm[0],
        xyz_mm[1],
        xyz_mm[2],
        (-r12).atan2(r22).to_degrees(),
        r02.atan2(cp).to_degrees(),
        (-r01).atan2(r00).to_degrees(),
    ]
}

/// Largest absolute difference between the rotation blocks of two STATUS
/// pose matrices — the orientation held (or not) across a move.
fn rotation_drift(a: &[f64; 16], b: &[f64; 16]) -> f64 {
    (0..12)
        .filter(|i| i % 4 != 3)
        .map(|i| (a[i] - b[i]).abs())
        .fold(0.0f64, f64::max)
}

/// A well-conditioned start posture for cartesian moves: away from the
/// wrist-aligned park singularity, comfortably inside every soft window,
/// and extended clear of the arm's own collision meshes. The posture this
/// test used before enforcement landed had the arm wrapped down onto its
/// own base (TCP 76 mm BELOW the base plane, six link pairs in contact),
/// where the collision gate rightly refuses a move that tightens the fold
/// further.
const CART_START_DEG: [f64; NUM_JOINTS] = [0.0, -75.0, 305.0, 20.0, -30.0, 180.0];
/// Cartesian move duration \[s\]. Long enough that the sim's cascade
/// tracking lag stays small next to the path tolerances: the lag is
/// proportional to speed, and the line below is held to 8 mm over a
/// 180 mm move.
const MOVE_S: f64 = 15.0;

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
    // Out of the arm's plane in all three axes: the joint-space route to
    // the same pose then bows tens of millimetres off the line, which is
    // what makes the collinearity bound below a measurement instead of a
    // truism.
    let target = [start[0] + 120.0, start[1] + 60.0, start[2] + 120.0];
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
    // The commanded target carries the start orientation, decoded out of
    // STATUS the way a client decodes it and handed straight back, so the
    // arm has to finish pointing where it started: a runtime that
    // rebuilds those three numbers in the other order (`Rz·Ry·Rx`, the
    // URDF `rpy` reading) turns the wrist 36.7° at this posture on its
    // way to a target the operator never asked for. The 0.1 bound is ~6°
    // of rotation-block error — room for the cascade's settle lag, an
    // order of magnitude under the 0.56 the swapped order costs.
    let landed = settled_tcp(&rig, "the pose move_l finished at");
    let rot_drift = rotation_drift(&s.pose, &landed.pose);
    assert!(
        rot_drift < 0.1,
        "move_l changed the orientation it was told to hold (rotation \
         block off by {rot_drift:.3}): commanded {wire_target:?}, started \
         {:?}, finished {:?}",
        s.pose,
        landed.pose
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
    // The seeded-IK entry point reads the target's rotation through the
    // same decode as move_l's, so it holds the orientation too.
    let after = settled_tcp(&rig, "settled after move_j_pose");
    let rot_drift = rotation_drift(&s.pose, &after.pose);
    assert!(
        rot_drift < 0.1,
        "move_j_pose solved for a different orientation than it was given \
         (rotation block off by {rot_drift:.3})"
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
/// feedforward: every loaded joint, wrist included, stays where the sim
/// placed it. With the `ZeroGravity` placeholder the same rig collapses
/// — measured here, the shoulder is 69° down and the elbow 108° over
/// inside one second, both against their endstops by the second — so
/// this bound cannot pass without the hook.
#[test]
fn gravity_hook_holds_the_arm_on_the_torque_plant() {
    /// Hold tolerance \[deg\] for every joint.
    const HOLD_TOL: f64 = 2.5;
    /// Every joint with a gravity load: the shoulder and elbow carry
    /// essentially the whole arm, the wrist joints carry a residual
    /// small enough that only a faithful torque↔current path holds
    /// them (J0 is on the vertical axis and carries nothing).
    const LOADED: [usize; 5] = [1, 2, 3, 4, 5];

    let rig = Rig::boot("gravity", true);
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);

    let placed = CART_START_DEG;
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

// ---- collision enforcement -------------------------------------------------

/// Start of the base sweep the keep-out tests drive: the arm extended
/// (its own meshes clear of each other, unlike the folded park pose),
/// rotated back around J0 so the sweep's midpoint sits in open workspace
/// where a keep-out can be parked.
const SWEEP_START_DEG: [f64; NUM_JOINTS] = [-40.0, -15.0, 365.0, 0.0, 0.0, 180.0];
/// J0 travel of the sweep \[deg\]; its midpoint is where the box goes.
const SWEEP_DEG: f64 = 80.0;
/// Sweep duration \[s\].
const SWEEP_S: f64 = 3.0;
/// Keep-out edge length \[m\]. Wide enough that the gripper cannot slip
/// past it between two checked configurations, small enough that the
/// sweep's endpoints stay well clear.
const KEEPOUT_M: f64 = 0.1;

fn with_j0(base: [f64; NUM_JOINTS], delta_deg: f64) -> [f64; NUM_JOINTS] {
    let mut a = base;
    a[0] += delta_deg;
    a
}

fn move_j(key: u64, angles_deg: [f64; NUM_JOINTS], duration_s: f64) -> Command {
    Command::MoveJ(MoveJ {
        key,
        angles: angles_deg,
        duration: Some(duration_s),
        speed: None,
        accel: None,
        blend_radius: None,
        rel: false,
    })
}

/// An axis-aligned cube keep-out centred on a TCP position read from
/// STATUS. Shapes are metres/radians on the wire (what waldoctl sends);
/// STATUS translations are mm.
fn keepout_at(name: &str, tcp_mm: [f64; 3]) -> Shape {
    Shape {
        kind: "box".to_owned(),
        params: vec![KEEPOUT_M, KEEPOUT_M, KEEPOUT_M],
        pose: vec![
            tcp_mm[0] / 1000.0,
            tcp_mm[1] / 1000.0,
            tcp_mm[2] / 1000.0,
            0.0,
            0.0,
            0.0,
        ],
        collision: true,
        margin: None,
        name: name.to_owned(),
    }
}

fn set_shapes(shapes: Vec<Shape>) -> Command {
    Command::SetShapes(SetShapes { shapes })
}

/// The applied collision world as the SHAPES query reports it.
fn shapes_readback(c: &mut Client) -> (Vec<Shape>, u64) {
    match c.query(&Command::Shapes) {
        QueryResult::Shapes { program, epoch, .. } => (program, epoch),
        other => panic!("unexpected SHAPES result {other:?}"),
    }
}

/// The configured park pose in wire units — where every program ends.
fn park_deg() -> [f64; NUM_JOINTS] {
    let cfg = par6_config::RobotConfig::load(&config_path()).expect("PAR6 config");
    let mut a = [0.0; NUM_JOINTS];
    for (out, rad) in a.iter_mut().zip(cfg.robot.park_pose_rad.iter()) {
        *out = rad.to_degrees();
    }
    a
}

fn angles_close(a: &[f64; NUM_JOINTS], b: &[f64; NUM_JOINTS], tol_deg: f64) -> bool {
    a.iter()
        .zip(b.iter())
        .all(|(x, y)| (x - y).abs() <= tol_deg)
}

/// Collision enforcement end to end, over the real protocol against the
/// real coal world:
///
/// - a `move_j` whose ENDPOINTS are both clear but whose interior sweeps
///   the gripper through a keep-out is refused before a sample reaches
///   the RT ring, with `SYS_SELF_COLLISION` and the colliding pair in the
///   payload — and the arm does not move;
/// - the same move runs to COMPLETE once the box is gone;
/// - STATUS carries that verdict (`collision_active` / `collision_pairs`)
///   until a motion is accepted, in the URDF's reporting vocabulary;
/// - a malformed shape refuses the WHOLE set: the epoch does not move,
///   the readback does not change, and the previous world stays
///   ENFORCED (not merely echoed);
/// - a keep-out dropped onto a RUNNING move stops it;
/// - the same move runs once the box is gone, and `reset_state` clears
///   the program layer.
#[test]
fn collision_world_is_enforced_over_protocol_v2() {
    let rig = Rig::boot("collision", false);
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    let mid_deg = with_j0(SWEEP_START_DEG, SWEEP_DEG / 2.0);
    let end_deg = with_j0(SWEEP_START_DEG, SWEEP_DEG);

    // Where the gripper passes halfway through the sweep: the keep-out
    // goes there, so both endpoints stay clear and only the interior of
    // the move is blocked.
    enable_and_teleport(&rig, &mut c, mid_deg);
    let mid_tcp =
        tcp_mm(&rig.wait_status("midpoint pose", |s| angles_close(&s.angles, &mid_deg, 0.5)));

    // Baseline: with an empty world the sweep runs to COMPLETE.
    enable_and_teleport(&rig, &mut c, SWEEP_START_DEG);
    let i = c.ok_index(&move_j(7001, end_deg, SWEEP_S));
    let (ok, detail) = c.wait_complete(i);
    assert!(ok, "the sweep must run with an empty world, got {detail:?}");

    // The keep-out, straddling the middle of that same sweep.
    enable_and_teleport(&rig, &mut c, SWEEP_START_DEG);
    let keepout = keepout_at("keepout", mid_tcp);
    c.ok(&set_shapes(vec![keepout.clone()]));
    let (program, epoch) = shapes_readback(&mut c);
    assert_eq!(program, vec![keepout.clone()]);
    assert!(epoch > 0, "an applied world must carry a non-zero epoch");

    // Both endpoints are clear — proven by STATUS at each of them — so
    // only checking the interior of the path can catch this move.
    rig.drain_status();
    let s = rig.wait_status("start of the sweep is clear", |s| {
        angles_close(&s.angles, &SWEEP_START_DEG, 0.5)
    });
    assert!(
        !s.collision_active,
        "the sweep start must be outside the keep-out: {:?}",
        s.collision_pairs
    );
    let i = c.ok_index(&move_j(7002, end_deg, SWEEP_S));
    let (ok, detail) = c.wait_complete(i);
    assert!(!ok, "a move sweeping through the keep-out must be refused");
    let e = detail.expect("a failed COMPLETE carries the error");
    assert_eq!(
        e.code,
        ErrorCode::SysSelfCollision as u16,
        "the refusal must be SYS_SELF_COLLISION, got {e:?}"
    );
    assert!(
        e.cause.contains("keepout"),
        "the error payload must name the colliding pair: {e:?}"
    );
    rig.drain_status();
    let s = rig.wait_status("pose after the refusal", |s| {
        s.action_state != par6_proto::ActionState::Executing
    });
    assert!(
        angles_close(&s.angles, &SWEEP_START_DEG, 1.0),
        "a refused move must not drive the arm: {:?}",
        s.angles
    );

    // STATUS carries the verdict of the refusal: the pairs the blocked
    // move would have collided in, in the URDF's reporting vocabulary
    // (link names and the client's own shape names, never the solver's
    // per-link geometry identifiers).
    rig.drain_status();
    let s = rig.wait_status("the refusal reaches STATUS", |s| s.collision_active);
    let pair = s
        .collision_pairs
        .iter()
        .find(|(a, b)| a == "keepout" || b == "keepout")
        .unwrap_or_else(|| {
            panic!(
                "collision_pairs must name the keep-out: {:?}",
                s.collision_pairs
            )
        });
    let link = if pair.0 == "keepout" {
        &pair.1
    } else {
        &pair.0
    };
    assert!(
        !link.ends_with("_0"),
        "the pair must name a URDF link, not a solver geometry id: {link}"
    );

    // A malformed set is refused WHOLE. Every flavour of malformed: a
    // kind waldoctl does not define, an arity that does not match the
    // kind, a dimension coal cannot build, and a name already taken.
    let mut unknown_kind = keepout.clone();
    unknown_kind.kind = "pyramid".to_owned();
    unknown_kind.name = "bad".to_owned();
    let mut short_params = keepout.clone();
    short_params.params = vec![KEEPOUT_M, KEEPOUT_M];
    short_params.name = "bad".to_owned();
    let mut negative = keepout.clone();
    negative.kind = "sphere".to_owned();
    negative.params = vec![-1.0];
    negative.name = "bad".to_owned();
    let duplicate = keepout.clone();
    for (label, bad) in [
        ("unknown kind", unknown_kind),
        ("wrong arity", short_params),
        ("negative radius", negative),
        ("duplicate name", duplicate),
    ] {
        let err = c.expect_error(&set_shapes(vec![keepout.clone(), bad]));
        assert_eq!(
            err.code,
            ErrorCode::CommValidationError as u16,
            "a {label} shape must be refused, got {err:?}"
        );
        let (program, refused_epoch) = shapes_readback(&mut c);
        assert_eq!(
            refused_epoch, epoch,
            "a refused world must not advance scene_epoch ({label})"
        );
        assert_eq!(
            program,
            vec![keepout.clone()],
            "a refused set must not change the readback ({label})"
        );
    }
    // …and the previous world is still ENFORCED, not merely echoed: the
    // same move is still refused.
    let i = c.ok_index(&move_j(7003, end_deg, SWEEP_S));
    let (ok, detail) = c.wait_complete(i);
    assert!(!ok, "a refused SET_SHAPES dropped the enforced keep-out");
    assert_eq!(
        detail.expect("a failed COMPLETE carries the error").code,
        ErrorCode::SysSelfCollision as u16
    );

    // A world change does not spare motion already committed: drop the
    // keep-out onto the path of a move that is already running and it
    // stops, instead of being enforced only from the next command on.
    c.ok(&set_shapes(Vec::new()));
    enable_and_teleport(&rig, &mut c, SWEEP_START_DEG);
    let i = c.ok_index(&move_j(7004, end_deg, SWEEP_S));
    rig.drain_status();
    rig.wait_status("the sweep is under way but short of the keep-out", |s| {
        s.executing_index == i as i64
            && s.angles[0] > SWEEP_START_DEG[0] + 3.0
            && s.angles[0] < -10.0
    });
    c.ok(&set_shapes(vec![keepout.clone()]));
    let (ok, detail) = c.wait_complete(i);
    assert!(!ok, "a keep-out dropped on a running move must stop it");
    let e = detail.expect("a failed COMPLETE carries the error");
    assert_eq!(
        e.code,
        ErrorCode::SysSelfCollision as u16,
        "the invalidated move must report SYS_SELF_COLLISION, got {e:?}"
    );
    rig.drain_status();
    let s = rig.wait_status("the arm stops", |s| s.speeds.iter().all(|v| v.abs() < 0.05));
    assert!(
        s.angles[0] < mid_deg[0],
        "the arm drove into the keep-out it was stopped for: {:?}",
        s.angles
    );

    // Removing the keep-out advances the epoch and lets the very same
    // move through — and accepting it clears the latched verdict.
    c.ok(&set_shapes(Vec::new()));
    let (program, cleared_epoch) = shapes_readback(&mut c);
    assert!(program.is_empty());
    assert!(
        cleared_epoch > epoch,
        "clearing the world must advance the epoch: {cleared_epoch} vs {epoch}"
    );
    enable_and_teleport(&rig, &mut c, SWEEP_START_DEG);
    let i = c.ok_index(&move_j(7005, end_deg, SWEEP_S));
    let (ok, detail) = c.wait_complete(i);
    assert!(
        ok,
        "the sweep must run once the keep-out is removed, got {detail:?}"
    );
    rig.drain_status();
    let s = rig.wait_status("a clean move clears the verdict", |_| true);
    assert!(
        !s.collision_active && s.collision_pairs.is_empty(),
        "the refusal's pairs outlived the motion that caused them: {:?}",
        s.collision_pairs
    );

    // reset_state clears the program layer: the readback empties and the
    // keep-out stops being enforced.
    c.ok(&set_shapes(vec![keepout]));
    enable_and_teleport(&rig, &mut c, SWEEP_START_DEG);
    let i = c.ok_index(&move_j(7006, end_deg, SWEEP_S));
    assert!(!c.wait_complete(i).0, "the keep-out must be back in force");
    c.ok(&Command::ResetState);
    let (program, _) = shapes_readback(&mut c);
    assert!(
        program.is_empty(),
        "reset_state must clear the program readback: {program:?}"
    );
    enable_and_teleport(&rig, &mut c, SWEEP_START_DEG);
    let i = c.ok_index(&move_j(7007, end_deg, SWEEP_S));
    let (ok, detail) = c.wait_complete(i);
    assert!(
        ok,
        "reset_state must stop enforcing the program layer, got {detail:?}"
    );

    // The arm must be able to return to its OWN park pose. PAR6 parks
    // folded, forearm back and resting against the base, which the vendor
    // collision meshes report as contact; if that counted as a collision
    // the last step of every program would be refused.
    let i = c.ok_index(&move_j(7008, park_deg(), SWEEP_S));
    let (ok, detail) = c.wait_complete(i);
    assert!(
        ok,
        "a move to the configured park pose must not be refused: {detail:?}"
    );

    rig.shutdown();
}

// ---- TCP offset -------------------------------------------------------------

/// Length of the commanded TCP offset \[mm\] — a tool standing this far
/// off the gripper's own TCP, the case `set_tcp_offset` exists for.
const TOOL_OFFSET_MM: f64 = 100.0;
/// World displacement of the commanded target from the start pose \[mm\].
/// The travel `cartesian_surface_over_protocol_v2` already proves the arm
/// covers from [`CART_START_DEG`], so both runs below land well inside the
/// workspace; the offset points along the same ray.
const OFFSET_TARGET_MM: [f64; 3] = [120.0, 60.0, 120.0];
/// Settle time for the cartesian moves below \[s\].
const OFFSET_MOVE_S: f64 = 8.0;

fn move_j_pose(key: u64, pose: [f64; 6], duration_s: f64) -> Command {
    Command::MoveJPose(MoveJPose {
        key,
        pose,
        duration: Some(duration_s),
        speed: None,
        accel: None,
        blend_radius: None,
    })
}

fn set_tcp_offset(x: f64, y: f64, z: f64) -> Command {
    Command::SetTcpOffset(SetTcpOffset { x, y, z })
}

fn tcp_offset_readback(c: &mut Client) -> [f64; 3] {
    match c.query(&Command::TcpOffset) {
        QueryResult::TcpOffset { x, y, z } => [x, y, z],
        other => panic!("unexpected TCP_OFFSET result {other:?}"),
    }
}

/// Settled TCP position after motion stops.
fn settled_tcp(rig: &Rig, what: &str) -> Status {
    rig.drain_status();
    rig.wait_status(what, |s| s.speeds.iter().all(|v| v.abs() < 0.05))
}

/// `set_tcp_offset` retargets the whole cartesian surface, not just the
/// readback.
///
/// The offset composes AFTER the URDF variant's own TCP frame and in the
/// TOOL-LOCAL frame — `T_flange→TCP = T_tool · T_offset`, the composition
/// the Python client already applies for preview FK/IK. The commanded
/// translation here is `Rᵀ·v`, so it is neither the world displacement it
/// must produce nor axis-aligned in any frame: a runtime that read it as
/// a world offset, or dropped it, lands somewhere else. With it set:
///
/// - STATUS reports the offset point, immediately, without the arm moving,
///   and it sits exactly `v` from the flange;
/// - the same commanded `move_j_pose` target puts THAT point on the
///   target, which parks the flange 100 mm from where the identical
///   command parks it with no offset;
/// - the `TCP_OFFSET` query still answers the COMMANDED translation, not
///   the composed transform.
///
/// Before the offset reached the models this failed on the first bound
/// already: STATUS kept reporting the flange, and the two `move_j_pose`
/// runs parked the arm in the same configuration.
#[test]
fn tcp_offset_retargets_the_cartesian_surface_over_protocol_v2() {
    let rig = Rig::boot("tcpoffset", false);
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    enable_and_teleport(&rig, &mut c, CART_START_DEG);
    rig.drain_status();
    let flange = rig.wait_status("start pose", |_| true);
    let p_flange = tcp_mm(&flange);

    // The world displacement the offset must produce, and the tool-local
    // translation that produces it: `d = Rᵀ·v` off the start orientation.
    let norm = (OFFSET_TARGET_MM[0] * OFFSET_TARGET_MM[0]
        + OFFSET_TARGET_MM[1] * OFFSET_TARGET_MM[1]
        + OFFSET_TARGET_MM[2] * OFFSET_TARGET_MM[2])
        .sqrt();
    let v: [f64; 3] = std::array::from_fn(|i| TOOL_OFFSET_MM * OFFSET_TARGET_MM[i] / norm);
    let d: [f64; 3] = std::array::from_fn(|j| (0..3).map(|i| flange.pose[4 * i + j] * v[i]).sum());

    // --- The reported point moves; the arm does not.
    c.ok(&set_tcp_offset(d[0], d[1], d[2]));
    rig.drain_status();
    let offset = rig.wait_status("STATUS follows the offset TCP", |s| {
        distance(tcp_mm(s), p_flange) > 1.0
    });
    let p_tcp = tcp_mm(&offset);
    for k in 0..3 {
        let want = p_flange[k] + v[k];
        assert!(
            (p_tcp[k] - want).abs() < 0.5,
            "the tool-local offset {d:?} must displace the reported TCP by {v:?}: \
             axis {k} is {}, expected {want} ({p_flange:?} -> {p_tcp:?})",
            p_tcp[k]
        );
    }
    assert!(
        angles_close(&offset.angles, &flange.angles, 0.1),
        "setting a TCP offset must not move the arm: {:?} -> {:?}",
        flange.angles,
        offset.angles
    );
    // A pure translation in the tool frame: the orientation block is
    // untouched, so only the point the runtime resolves at has changed.
    for k in [0, 1, 2, 4, 5, 6, 8, 9, 10] {
        assert!(
            (offset.pose[k] - flange.pose[k]).abs() < 1e-6,
            "the offset rotated the reported pose at element {k}"
        );
    }
    let readback = tcp_offset_readback(&mut c);
    for k in 0..3 {
        assert!(
            (readback[k] - d[k]).abs() < 1e-9,
            "TCP_OFFSET answers the commanded translation {d:?}, not the composed \
             transform: got {readback:?}"
        );
    }

    // --- The same commanded target, with and without the offset. Both
    // runs park the flange along the travel
    // `cartesian_surface_over_protocol_v2` already proves is clear, a full
    // TOOL_OFFSET_MM apart from each other.
    let target: [f64; 3] = std::array::from_fn(|k| p_flange[k] + OFFSET_TARGET_MM[k]);
    let wire_target = wire_pose_at(&flange.pose, target);

    let i = c.ok_index(&move_j_pose(2001, wire_target, OFFSET_MOVE_S));
    let (ok, detail) = c.wait_complete(i);
    assert!(
        ok,
        "move_j_pose with a TCP offset must complete, got {detail:?}"
    );
    let landed = settled_tcp(&rig, "settled with the offset");
    let reached = tcp_mm(&landed);
    assert!(
        distance(reached, target) < 10.0,
        "STATUS must report the OFFSET point on the commanded target: \
         {reached:?} vs {target:?}"
    );

    // Where the flange actually ended up: same configuration, offset off.
    c.ok(&set_tcp_offset(0.0, 0.0, 0.0));
    rig.drain_status();
    let f_with = tcp_mm(&rig.wait_status("flange of the offset run", |s| {
        distance(tcp_mm(s), reached) > 1.0
    }));

    rig.drain_status();
    enable_and_teleport(&rig, &mut c, CART_START_DEG);
    let i = c.ok_index(&move_j_pose(2002, wire_target, OFFSET_MOVE_S));
    let (ok, detail) = c.wait_complete(i);
    assert!(
        ok,
        "move_j_pose without an offset must complete, got {detail:?}"
    );
    let plain = settled_tcp(&rig, "settled without the offset");
    let f_without = tcp_mm(&plain);
    assert!(
        distance(f_without, target) < 10.0,
        "without an offset the FLANGE lands on the target: {f_without:?} vs {target:?}"
    );

    let moved = distance(f_with, f_without);
    assert!(
        moved > 80.0,
        "the offset must land the flange somewhere else for the same commanded \
         target: {f_with:?} vs {f_without:?} ({moved:.1} mm apart, expected \
         about {TOOL_OFFSET_MM})"
    );
    let joint_delta = landed
        .angles
        .iter()
        .zip(plain.angles.iter())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0f64, f64::max);
    assert!(
        joint_delta > 2.0,
        "the two runs parked in the same configuration ({joint_delta:.3} deg apart): \
         the offset never reached the planner's IK"
    );

    rig.shutdown();
}

// ---- cartesian enablement ---------------------------------------------------

/// A configuration at the edge of the reachable workspace: the arm is
/// stretched out along −x holding the wrist fixed, so a step further out
/// has no IK solution while a step back in does. Measured, not assumed —
/// the test drives a real 20 mm `move_l` each way and the runtime agrees
/// with the flags.
const BOUNDARY_DEG: [f64; NUM_JOINTS] = [0.0, -100.0, 285.0, 0.0, -30.0, 180.0];
/// World-frame enablement slots, positive direction first.
const X_POS: usize = 0;
const X_NEG: usize = 1;

fn move_l_to(key: u64, pose: [f64; 6], duration_s: f64) -> Command {
    Command::MoveL(MoveL {
        key,
        pose,
        frame: Frame::Wrf,
        duration: Some(duration_s),
        speed: None,
        accel: None,
        blend_radius: None,
        rel: false,
    })
}

/// The cartesian enablement flags describe the real workspace.
///
/// Parked against the outer edge of its reach, the arm may still move
/// inward and may not move further out — and STATUS says exactly that,
/// per direction, in both frames. The flags are then corroborated against
/// what the runtime actually does with the same two directions: the
/// blocked one is an `IK_TARGET_UNREACHABLE` refusal, the free one runs to
/// COMPLETE.
///
/// Before the probe landed, an `ffi` runtime published
/// `Enablement::default()` — all twelve directions free, in both frames,
/// in every pose — so the blocked direction read 1 and a frontend offered
/// a jog button for a motion the runtime would refuse.
#[test]
fn cartesian_enablement_measures_the_real_workspace() {
    let rig = Rig::boot("enablement", false);
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    enable_and_teleport(&rig, &mut c, BOUNDARY_DEG);
    rig.drain_status();
    let s = rig.wait_status("parked at the edge of reach", |s| {
        angles_close(&s.angles, &BOUNDARY_DEG, 0.5)
    });
    let edge = tcp_mm(&s);
    assert!(
        edge[0] < -400.0,
        "the boundary pose must reach out along -x, got {edge:?}"
    );
    assert_eq!(
        (s.cart_en_wrf[X_POS], s.cart_en_wrf[X_NEG]),
        (1, 0),
        "at the edge of reach the arm may move in (+x) and not out (-x); \
         world-frame flags were {:?}",
        s.cart_en_wrf
    );
    assert!(
        s.cart_en_trf.contains(&0),
        "the tool-frame flags are a real measurement too, not a default: {:?}",
        s.cart_en_trf
    );
    // The REACHABLE query answers from the same measurement.
    match c.query(&Command::Reachable) {
        QueryResult::Reachable { cart_en_wrf, .. } => assert_eq!(
            (cart_en_wrf[X_POS], cart_en_wrf[X_NEG]),
            (1, 0),
            "REACHABLE disagrees with STATUS: {cart_en_wrf:?}"
        ),
        other => panic!("unexpected REACHABLE result {other:?}"),
    }

    // Corroboration: the runtime really does refuse the direction it
    // greyed, and really does run the one it kept.
    let inward = wire_pose_at(&s.pose, [edge[0] + 20.0, edge[1], edge[2]]);
    let i = c.ok_index(&move_l_to(3001, inward, 3.0));
    let (ok, detail) = c.wait_complete(i);
    assert!(
        ok,
        "the direction reported free must actually run, got {detail:?}"
    );

    enable_and_teleport(&rig, &mut c, BOUNDARY_DEG);
    let outward = wire_pose_at(&s.pose, [edge[0] - 20.0, edge[1], edge[2]]);
    let i = c.ok_index(&move_l_to(3002, outward, 3.0));
    let (ok, detail) = c.wait_complete(i);
    assert!(
        !ok,
        "the direction reported blocked must actually be refused"
    );
    // Either IK verdict says the same thing — the line leaves the
    // reachable workspace; which one depends on whether the endpoint or
    // an interior sample loses convergence first.
    let code = detail.expect("a failed COMPLETE carries the error").code;
    assert!(
        code == ErrorCode::IkTargetUnreachable as u16 || code == ErrorCode::IkPartialPath as u16,
        "the blocked direction must be blocked for the reason the flag claims, \
         got error code {code}"
    );

    rig.shutdown();
}

// ---- curved and blended moves ----------------------------------------------

/// Start posture for the curved and blended moves: the same kind of
/// well-conditioned pose as [`CART_START_DEG`], chosen (by sweeping the
/// IK + soft-window envelope) for room around it — a 120 mm arc and two
/// 120 mm legs fit inside every joint's soft window from here, which
/// they do not from [`CART_START_DEG`], where the elbow is 15° from its
/// soft stop.
const CURVE_START_DEG: [f64; NUM_JOINTS] = [0.0, -70.0, 260.0, 0.0, -20.0, 180.0];
/// Duration of the spline move \[s\]. Slower than [`MOVE_S`] because the
/// sim's tracking lag is proportional to speed AND to path curvature,
/// and a wave has far more of the second than a straight line does.
const SPLINE_S: f64 = 25.0;

/// Speed of the TCP \[mm/s\] over a sliding window of STATUS frames,
/// paired with the position at the middle of each window.
///
/// Measured from the broadcast itself (pose delta over the header's
/// monotonic clock) rather than read out of `STATUS.tcp_speed`, so the
/// numbers below stand on the same evidence a client has. The window
/// spans [`SPEED_WINDOW`] frames because the status rate and the RT tick
/// rate are the same here: consecutive frames sometimes carry the same
/// snapshot, and a frame pair that straddles one would read as a
/// standstill.
fn tcp_speeds(path: &[Status]) -> Vec<([f64; 3], f64)> {
    path.windows(SPEED_WINDOW)
        .filter_map(|w| {
            let (first, last) = (&w[0], &w[SPEED_WINDOW - 1]);
            let dt = last.mono_time_ns.checked_sub(first.mono_time_ns)? as f64 * 1e-9;
            (dt > 0.0).then(|| {
                (
                    tcp_mm(&w[SPEED_WINDOW / 2]),
                    distance(tcp_mm(first), tcp_mm(last)) / dt,
                )
            })
        })
        .collect()
}

/// STATUS frames per speed measurement (~0.1 s at the 50 Hz broadcast).
const SPEED_WINDOW: usize = 5;

/// Teleport to the curved-move start posture and return the pose the arm
/// actually came to rest in.
///
/// The wait is on the ANGLES, not just on the broadcast: `teleport`
/// confirms within a degree, and a degree of J1 is centimetres of TCP —
/// enough to move every target derived from this pose by more than the
/// path tolerances below.
fn curve_start(rig: &Rig, c: &mut Client) -> Status {
    enable_and_teleport(rig, c, CURVE_START_DEG);
    rig.drain_status();
    rig.wait_status("the arm at rest in the curved-move start posture", |s| {
        s.angles
            .iter()
            .zip(CURVE_START_DEG.iter())
            .all(|(a, b)| (a - b).abs() < 0.02)
            && s.speeds.iter().all(|v| v.abs() < 0.01)
    })
}

/// The slowest the TCP ever got while it was within `radius_mm` of
/// `corner`, and the mean speed over the whole move.
fn corner_and_mean_speed(path: &[Status], corner: [f64; 3], radius_mm: f64) -> (f64, f64) {
    let speeds = tcp_speeds(path);
    let moving: Vec<f64> = speeds
        .iter()
        .map(|(_, v)| *v)
        .filter(|v| *v > 0.5)
        .collect();
    let mean = if moving.is_empty() {
        0.0
    } else {
        moving.iter().sum::<f64>() / moving.len() as f64
    };
    let at_corner = speeds
        .iter()
        .filter(|(p, _)| distance(*p, corner) < radius_mm)
        .map(|(_, v)| *v)
        .fold(f64::INFINITY, f64::min);
    (at_corner, mean)
}

/// Closest the measured TCP path came to `p` \[mm\], measured against the
/// path (its frame-to-frame segments), not just the sampled points.
fn path_misses(path: &[[f64; 3]], p: [f64; 3]) -> f64 {
    path.windows(2)
        .map(|w| distance_to_segment(p, w[0], w[1]))
        .fold(f64::INFINITY, f64::min)
}

/// Arc, spline and process moves trace the geometry they name.
///
/// All three were `MOTN_SETUP_FAILED "arc/spline/process moves are not
/// implemented yet"` before this landed, so completion alone would be a
/// result — but completion is not the claim. The claims are geometric
/// and are measured off the STATUS broadcast:
///
/// - `move_c` puts every point of the path on the circle through
///   start / via / end, in that circle's plane, and passes through the
///   via point — while bowing far off the straight chord a `move_l`
///   would have taken;
/// - `move_s` passes through every waypoint it was given, and curves
///   between them (a polyline through the same waypoints would not);
/// - `move_p` rounds its interior corner instead of stopping in it: the
///   path stays inside the auto-blend zone and the TCP is still moving
///   when it goes through.
#[test]
fn curved_moves_trace_their_geometry() {
    let rig = Rig::boot("curved", false);
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    // --- move_c: a half circle of radius R in the world XZ plane,
    // from the start pose, up over the top, and down to +2R in x.
    const R: f64 = 60.0;
    let s = curve_start(&rig, &mut c);
    let start = tcp_mm(&s);
    let center = [start[0] + R, start[1], start[2]];
    let via = [center[0], center[1], center[2] - R];
    let end = [center[0] + R, center[1], center[2]];
    let i = c.ok_index(&Command::MoveC(MoveC {
        key: 4001,
        via: wire_pose_at(&s.pose, via),
        end: wire_pose_at(&s.pose, end),
        frame: Frame::Wrf,
        duration: Some(MOVE_S),
        speed: None,
        accel: None,
        blend_radius: None,
    }));
    let arc: Vec<[f64; 3]> = rig
        .collect_status(Duration::from_secs_f64(MOVE_S + 1.0))
        .iter()
        .map(tcp_mm)
        .collect();
    let (ok, detail) = c.wait_complete(i);
    assert!(ok, "move_c must complete ok, got {detail:?}");

    let moving: Vec<[f64; 3]> = arc
        .iter()
        .copied()
        .filter(|p| distance(*p, start) > 3.0)
        .collect();
    assert!(
        moving.len() > 50,
        "expected a sampled arc, got {} moving samples",
        moving.len()
    );
    let radial = moving
        .iter()
        .map(|p| (distance(*p, center) - R).abs())
        .fold(0.0f64, f64::max);
    let out_of_plane = moving
        .iter()
        .map(|p| (p[1] - center[1]).abs())
        .fold(0.0f64, f64::max);
    assert!(
        radial < 8.0,
        "move_c left its circle by {radial:.2} mm (radius {R} mm about {center:?})"
    );
    assert!(
        out_of_plane < 8.0,
        "move_c left the arc plane by {out_of_plane:.2} mm"
    );
    let via_miss = path_misses(&arc, via);
    assert!(
        via_miss < 8.0,
        "move_c missed its via point by {via_miss:.2} mm"
    );
    let chord_dev = moving
        .iter()
        .map(|p| distance_to_segment(*p, start, end))
        .fold(0.0f64, f64::max);
    assert!(
        chord_dev > R / 2.0,
        "move_c hugged the straight chord ({chord_dev:.2} mm off it): that is a move_l, not an arc"
    );
    // Where the arm comes to rest, within the rig's own steady-state
    // tracking error — the same ~10 mm the joint-space cartesian move
    // above is held to.
    let end_miss = distance(tcp_mm(&settled_tcp(&rig, "settled after move_c")), end);
    assert!(
        end_miss < 15.0,
        "move_c ended {end_miss:.1} mm off its end pose"
    );

    // --- move_s: a wave through four waypoints.
    let s = curve_start(&rig, &mut c);
    let start = tcp_mm(&s);
    let waypoints: Vec<[f64; 3]> = [[45.0, 0.0, 45.0], [90.0, 0.0, -45.0], [135.0, 0.0, 30.0]]
        .iter()
        .map(|d| [start[0] + d[0], start[1] + d[1], start[2] + d[2]])
        .collect();
    let i = c.ok_index(&Command::MoveS(MoveS {
        key: 4002,
        waypoints: waypoints
            .iter()
            .map(|p| wire_pose_at(&s.pose, *p))
            .collect(),
        frame: Frame::Wrf,
        duration: Some(SPLINE_S),
        speed: None,
        accel: None,
    }));
    let spline: Vec<[f64; 3]> = rig
        .collect_status(Duration::from_secs_f64(SPLINE_S + 1.0))
        .iter()
        .map(tcp_mm)
        .collect();
    let (ok, detail) = c.wait_complete(i);
    assert!(ok, "move_s must complete ok, got {detail:?}");
    // The spline geometry itself passes within 0.1 mm of every
    // waypoint (`par6-motion`'s own tests measure that); what is
    // measured here is the arm ON it, so the bound is the sim's
    // tracking lag at a curvature peak. It is meaningful because the
    // straight line joining the same endpoints misses these waypoints
    // by more than twice as much — asserted right below.
    let last = *waypoints.last().expect("waypoints");
    for (k, w) in waypoints.iter().enumerate() {
        let miss = path_misses(&spline, *w);
        assert!(
            miss < 12.0,
            "move_s missed waypoint {k} ({w:?}) by {miss:.2} mm"
        );
        // The bound above is only worth something because the straight
        // route between the same endpoints comes nowhere near these
        // waypoints.
        if k + 1 < waypoints.len() {
            let straight = distance_to_segment(*w, start, last);
            assert!(
                straight > 25.0,
                "waypoint {k} sits {straight:.1} mm off the straight route: \
                 passing near it proves nothing"
            );
        }
    }
    let end_miss = distance(tcp_mm(&settled_tcp(&rig, "settled after move_s")), last);
    assert!(
        end_miss < 15.0,
        "move_s ended {end_miss:.1} mm off its last waypoint"
    );
    // A spline is not a polyline: between the second and third waypoints
    // it leaves the chord that joins them.
    let bow = spline
        .iter()
        .filter(|p| {
            let t = progress_along(**p, waypoints[0], waypoints[1]);
            (0.1..0.9).contains(&t)
        })
        .map(|p| distance_to_segment(*p, waypoints[0], waypoints[1]))
        .fold(0.0f64, f64::max);
    assert!(
        bow > 3.0,
        "move_s ran straight between its waypoints ({bow:.2} mm of bow)"
    );

    // --- move_p: a right-angle corner, auto-blended.
    let s = curve_start(&rig, &mut c);
    let start = tcp_mm(&s);
    let corner = [start[0] + 100.0, start[1], start[2]];
    let finish = [corner[0], corner[1], corner[2] - 100.0];
    let i = c.ok_index(&Command::MoveP(MoveP {
        key: 4003,
        waypoints: vec![wire_pose_at(&s.pose, corner), wire_pose_at(&s.pose, finish)],
        frame: Frame::Wrf,
        duration: Some(MOVE_S),
        speed: None,
        accel: None,
    }));
    let process = rig.collect_status(Duration::from_secs_f64(MOVE_S + 1.0));
    let (ok, detail) = c.wait_complete(i);
    assert!(ok, "move_p must complete ok, got {detail:?}");
    let points: Vec<[f64; 3]> = process.iter().map(tcp_mm).collect();
    // 25 mm of auto-blend on 100 mm segments, so the corner is cut by
    // something under that and by more than the tracking error.
    let corner_miss = path_misses(&points, corner);
    assert!(
        (2.0..25.0).contains(&corner_miss),
        "move_p's corner was not rounded into its blend zone: closest approach {corner_miss:.2} mm"
    );
    let (at_corner, mean) = corner_and_mean_speed(&process, corner, 30.0);
    assert!(
        at_corner > 0.25 * mean,
        "move_p slowed to {at_corner:.2} mm/s at the corner against a mean of {mean:.2} mm/s: \
         a blend that stops is not a blend"
    );
    let end_miss = distance(tcp_mm(&settled_tcp(&rig, "settled after move_p")), finish);
    assert!(
        end_miss < 15.0,
        "move_p ended {end_miss:.1} mm off its last waypoint"
    );

    rig.shutdown();
}

/// A blend radius on a queued `move_l` really rounds the corner into the
/// NEXT queued `move_l` — measured against the same corner run without
/// one.
///
/// Before this landed the runtime refused any non-nil `r` outright
/// (`COMM_VALIDATION_ERROR`), because it started exactly one queued
/// command at a time. The claim now is a comparison, not a completion:
/// with `r = 0` the arm stops dead in the corner and passes through it;
/// with `r = 25` it cuts the corner, never stops, and gets there sooner.
/// Both commands still report their own COMPLETE, and the high-water
/// `completed_index` ends on the second of them.
#[test]
fn a_blend_radius_rounds_the_corner_into_the_next_queued_move() {
    const BLEND_MM: f64 = 60.0;
    const LEG_MM: f64 = 150.0;
    /// Duration of each leg \[s\]. Slow, for the same reason the other
    /// cartesian measurements here are: the rig's tracking error scales
    /// with speed and the corner geometry is what is being measured.
    const LEG_S: f64 = 8.0;

    let rig = Rig::boot("blend", false);
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    // Two legs meeting at a right angle: +x, then +z.
    let leg = |s: &Status, key: u64, r: Option<f64>| -> (Command, [f64; 3], [f64; 3]) {
        let start = tcp_mm(s);
        let corner = [start[0] + LEG_MM, start[1], start[2]];
        let finish = [corner[0], corner[1], corner[2] - LEG_MM];
        (
            Command::MoveL(MoveL {
                key,
                pose: wire_pose_at(&s.pose, corner),
                frame: Frame::Wrf,
                duration: Some(LEG_S),
                speed: None,
                accel: None,
                blend_radius: r,
                rel: false,
            }),
            corner,
            finish,
        )
    };
    let second = |s: &Status, key: u64, finish: [f64; 3]| {
        Command::MoveL(MoveL {
            key,
            pose: wire_pose_at(&s.pose, finish),
            frame: Frame::Wrf,
            duration: Some(LEG_S),
            speed: None,
            accel: None,
            blend_radius: None,
            rel: false,
        })
    };

    // --- control: the same corner with no blend radius.
    let s = curve_start(&rig, &mut c);
    let (first, corner, finish) = leg(&s, 5001, None);
    let started = Instant::now();
    let i1 = c.ok_index(&first);
    let i2 = c.ok_index(&second(&s, 5002, finish));
    let sharp = rig.collect_status(Duration::from_secs_f64(2.0 * LEG_S + 2.0));
    let (ok, detail) = c.wait_complete(i1);
    assert!(
        ok,
        "the unblended first leg must complete ok, got {detail:?}"
    );
    let (ok, detail) = c.wait_complete(i2);
    assert!(
        ok,
        "the unblended second leg must complete ok, got {detail:?}"
    );
    let sharp_time = started.elapsed().as_secs_f64();
    let sharp_points: Vec<[f64; 3]> = sharp.iter().map(tcp_mm).collect();
    let sharp_miss = path_misses(&sharp_points, corner);
    let (sharp_corner_speed, sharp_mean) = corner_and_mean_speed(&sharp, corner, 20.0);
    assert!(
        sharp_miss < 16.0,
        "without a blend radius the arm must go INTO the corner, missed by {sharp_miss:.2} mm \
         (the rig's own stopping error is about 10 mm)"
    );
    assert!(
        sharp_corner_speed < 1.0,
        "without a blend radius the arm must STOP in the corner, \
         slowest it got was {sharp_corner_speed:.2} mm/s"
    );

    // --- the same corner, blended.
    let s = curve_start(&rig, &mut c);
    let (first, corner, finish) = leg(&s, 5003, Some(BLEND_MM));
    let started = Instant::now();
    let i1 = c.ok_index(&first);
    let i2 = c.ok_index(&second(&s, 5004, finish));
    let blended = rig.collect_status(Duration::from_secs_f64(2.0 * LEG_S + 2.0));
    let (ok, detail) = c.wait_complete(i1);
    assert!(ok, "the blended first leg must complete ok, got {detail:?}");
    let (ok, detail) = c.wait_complete(i2);
    assert!(
        ok,
        "the blended second leg must complete ok, got {detail:?}"
    );
    let blend_time = started.elapsed().as_secs_f64();
    let blended_points: Vec<[f64; 3]> = blended.iter().map(tcp_mm).collect();

    // The corner is rounded: cut by more than the tracking error, and
    // by no more than the radius that was asked for.
    // Measured against the SAME corner driven without a radius, so the
    // rig's stopping error cancels out of the comparison: the geometry
    // cuts this corner by 0.35 r (21 mm here), the rest is tracking.
    let blend_miss = path_misses(&blended_points, corner);
    assert!(
        blend_miss > sharp_miss + 6.0,
        "r = {BLEND_MM} mm cut the corner by {blend_miss:.2} mm and the same corner without \
         a radius by {sharp_miss:.2} mm: that is not a rounded corner"
    );
    assert!(
        blend_miss < BLEND_MM,
        "the rounded corner strayed {blend_miss:.2} mm from the waypoint, further than the \
         {BLEND_MM} mm zone that was asked for"
    );
    // And it is a fly-by, not a stop-and-go.
    let (blend_corner_speed, blend_mean) = corner_and_mean_speed(&blended, corner, 40.0);
    assert!(
        blend_corner_speed > 0.3 * blend_mean && blend_corner_speed > 5.0,
        "the blended corner slowed to {blend_corner_speed:.2} mm/s against a mean of \
         {blend_mean:.2} mm/s (unblended: {sharp_corner_speed:.2} of {sharp_mean:.2}) — \
         a blend that stops is not a blend"
    );
    assert!(
        blend_time < sharp_time,
        "the blended corner took {blend_time:.2} s and the sharp one {sharp_time:.2} s: \
         not stopping is supposed to be faster"
    );
    let end_miss = distance(
        tcp_mm(&settled_tcp(&rig, "settled after the blended chain")),
        finish,
    );
    assert!(
        end_miss < 15.0,
        "the blended chain ended {end_miss:.1} mm off its last target"
    );
    // Completion-index semantics for a blended pair: both indexes are
    // completed, and the high-water mark is the LAST of them.
    let s = rig.wait_status("completed index reaches the blended pair", |s| {
        s.completed_index >= i2 as i64
    });
    assert_eq!(
        s.executing_index, -1,
        "nothing may still be executing once both blended commands completed"
    );

    rig.shutdown();
}

/// A blend radius on a queued `move_j` rounds the corner between two
/// JOINT moves, and is measured where a joint move lives: in joint
/// space.
///
/// Without a radius the arm has to arrive at the intermediate
/// configuration exactly — that is what "the move completed" means — and
/// then set off again. With one, the runtime plans both moves as a
/// single joint path whose corner is a Bézier zone sized from the TCP
/// distance the radius describes, so the intermediate configuration is
/// passed BY, not landed on, and the arm never comes to rest in it.
#[test]
fn a_blend_radius_rounds_a_joint_chain_too() {
    const BLEND_MM: f64 = 30.0;
    const LEG_S: f64 = 4.0;

    let rig = Rig::boot("jointblend", false);
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    let mut corner_deg = CURVE_START_DEG;
    corner_deg[0] += 15.0;
    let mut finish_deg = corner_deg;
    finish_deg[1] -= 12.0;
    let move_j = |key: u64, angles: [f64; NUM_JOINTS], r: Option<f64>| {
        Command::MoveJ(MoveJ {
            key,
            angles,
            duration: Some(LEG_S),
            speed: None,
            accel: None,
            blend_radius: r,
            rel: false,
        })
    };
    /// Largest per-joint distance from `q` to the corner configuration.
    fn from_corner(s: &Status, corner: [f64; NUM_JOINTS]) -> f64 {
        s.angles
            .iter()
            .zip(corner.iter())
            .map(|(a, b)| (a - b).abs())
            .fold(0.0f64, f64::max)
    }

    // --- control: the same two moves, no radius. The arm lands on the
    // intermediate configuration and stops there.
    curve_start(&rig, &mut c);
    let i1 = c.ok_index(&move_j(6001, corner_deg, None));
    let i2 = c.ok_index(&move_j(6002, finish_deg, None));
    let sharp = rig.collect_status(Duration::from_secs_f64(2.0 * LEG_S + 2.0));
    for i in [i1, i2] {
        let (ok, detail) = c.wait_complete(i);
        assert!(
            ok,
            "the unblended joint legs must complete ok, got {detail:?}"
        );
    }
    let sharp_miss = sharp
        .iter()
        .map(|s| from_corner(s, corner_deg))
        .fold(f64::INFINITY, f64::min);
    assert!(
        sharp_miss < 1.5,
        "without a radius the arm must reach the intermediate configuration, \
         closest it got was {sharp_miss:.2}° (the rig's own stopping error is a few tenths)"
    );

    // --- the same pair, blended.
    curve_start(&rig, &mut c);
    let i1 = c.ok_index(&move_j(6003, corner_deg, Some(BLEND_MM)));
    let i2 = c.ok_index(&move_j(6004, finish_deg, None));
    let blended = rig.collect_status(Duration::from_secs_f64(2.0 * LEG_S + 2.0));
    for i in [i1, i2] {
        let (ok, detail) = c.wait_complete(i);
        assert!(
            ok,
            "the blended joint legs must complete ok, got {detail:?}"
        );
    }
    let blend_miss = blended
        .iter()
        .map(|s| from_corner(s, corner_deg))
        .fold(f64::INFINITY, f64::min);
    assert!(
        blend_miss > sharp_miss + 0.5,
        "r = {BLEND_MM} mm passed within {blend_miss:.2}° of the intermediate configuration \
         and the unblended pair within {sharp_miss:.2}°: the corner was not rounded"
    );

    // And it never stopped there: between leaving the start and reaching
    // the end, the joints keep moving.
    let mid: Vec<&Status> = blended
        .iter()
        .filter(|s| from_corner(s, corner_deg) < 4.0)
        .collect();
    assert!(
        mid.len() > 5,
        "expected the corner region to be sampled, got {} frames",
        mid.len()
    );
    let slowest = mid
        .iter()
        .map(|s| s.speeds.iter().fold(0.0f64, |m, v| m.max(v.abs())))
        .fold(f64::INFINITY, f64::min);
    assert!(
        slowest > 0.02,
        "the blended joint corner slowed to {slowest:.4} rad/s: a blend that stops is not a blend"
    );
    rig.wait_status(
        "the blended joint chain reports both commands complete",
        |s| s.completed_index >= i2 as i64,
    );
    // Where it came to rest, within the rig's own stopping error (the
    // unblended pair above lands no closer).
    let settled = settled_tcp(&rig, "the blended joint chain at rest");
    assert!(
        settled
            .angles
            .iter()
            .zip(finish_deg.iter())
            .all(|(a, b)| (a - b).abs() < 2.0),
        "the blended chain ended at {:?}, not at {finish_deg:?}",
        settled.angles
    );

    rig.shutdown();
}

/// The configured soft window, in radians, per joint.
fn soft_window_rad() -> [(f64, f64); NUM_JOINTS] {
    let cfg = par6_config::RobotConfig::load(&config_path()).expect("PAR6 config");
    let mut out = [(0.0, 0.0); NUM_JOINTS];
    for (slot, joint) in out.iter_mut().zip(cfg.joints.iter()) {
        *slot = (joint.limits.soft_min_rad, joint.limits.soft_max_rad);
    }
    out
}

fn to_deg(rad: [f64; NUM_JOINTS]) -> [f64; NUM_JOINTS] {
    rad.map(f64::to_degrees)
}

/// A posture the seeded solve reaches only by turning: from the park
/// pose, DLS converges on this configuration with J1 and J4 each a full
/// revolution out (`+2π` / `−2π`), which is the same arm and a pair of
/// numbers the soft-limit check rejects.
const TURNED_POSTURE_RAD: [f64; NUM_JOINTS] = [
    1.151_079_285_338_248,
    -2.149_645_469_653_074,
    3.980_431_385_749_838,
    0.623_123_050_584_976,
    1.021_342_994_420_38,
    0.990_794_316_439_534,
];

/// J5 held past its SOFT window (1.9 rad) but inside its hard one: a
/// posture the arm can be teleported into and whose pose no turn of any
/// joint brings back in range.
const BEYOND_SOFT_J5_RAD: f64 = 1.9;

/// A converged IK solution is judged as a configuration, not as a turn
/// count.
///
/// Damped least squares integrates joint increments without bound, so a
/// solve routinely lands on `q + 2πk`: the same arm posture, carried by
/// a number the soft-limit check refuses verbatim — measured on this
/// rig as `move target for joint 3 (5.366112773926797 rad) is outside
/// soft limits [-2.6147335, 2.5547335]`, for a solution 0.917 rad
/// inside that window.
///
/// Both halves are the fix: the turned solution has to run and land on
/// the commanded pose, and a target that is out of range at every turn
/// count has to stay refused — wrapping is branch selection, never a
/// way past the limits.
#[test]
fn ik_solutions_are_wrapped_into_their_soft_window() {
    let rig = Rig::boot("ikwrap", false);
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    let soft = soft_window_rad();
    let park = park_deg();

    // The pose of the turned posture, as the runtime itself reports it.
    let turned_deg = to_deg(TURNED_POSTURE_RAD);
    enable_and_teleport(&rig, &mut c, turned_deg);
    rig.drain_status();
    let at_posture = rig.wait_status("parked at the turned posture", |s| {
        angles_close(&s.angles, &turned_deg, 0.5)
    });
    let target = wire_pose_at(&at_posture.pose, tcp_mm(&at_posture));

    // Commanded from the park pose, the seeded solve turns J1 and J4 a
    // revolution past their windows to reach it.
    enable_and_teleport(&rig, &mut c, park);
    let i = c.ok_index(&move_j_pose(9101, target, 8.0));
    let (ok, detail) = c.wait_complete(i);
    assert!(
        ok,
        "a solution that is inside every soft window after wrapping must run: {detail:?}"
    );
    let settled = settled_tcp(&rig, "the wrapped solution at rest");
    assert!(
        distance(tcp_mm(&settled), [target[0], target[1], target[2]]) < 5.0,
        "the wrapped solution must land on the commanded pose: {:?} vs {target:?}",
        tcp_mm(&settled)
    );
    for (j, angle_deg) in settled.angles.iter().enumerate() {
        let rad = angle_deg.to_radians();
        assert!(
            rad >= soft[j].0 - 1e-6 && rad <= soft[j].1 + 1e-6,
            "the arm parked outside joint {j}'s soft window: {rad} rad in {:?}",
            soft[j]
        );
    }

    // Out of range at every turn count: J5 beyond its soft window, which
    // the wrist flip mirrors to the far side of the same window.
    let mut beyond_deg = park;
    beyond_deg[4] = BEYOND_SOFT_J5_RAD.to_degrees();
    enable_and_teleport(&rig, &mut c, beyond_deg);
    rig.drain_status();
    let at_beyond = rig.wait_status("parked past J5's soft window", |s| {
        angles_close(&s.angles, &beyond_deg, 0.5)
    });
    let refused_target = wire_pose_at(&at_beyond.pose, tcp_mm(&at_beyond));

    enable_and_teleport(&rig, &mut c, park);
    let before = tcp_mm(&rig.wait_status("pose before the out-of-range target", |_| true));
    let i = c.ok_index(&move_j_pose(9102, refused_target, 8.0));
    let (ok, detail) = c.wait_complete(i);
    assert!(!ok, "a target outside the soft window must stay refused");
    let e = detail.expect("a failed COMPLETE carries the error");
    assert_eq!(
        e.code,
        ErrorCode::CommValidationError as u16,
        "the refusal must name the soft-limit violation, got {e:?}"
    );
    let after = tcp_mm(&rig.wait_status("pose after the out-of-range target", |_| true));
    assert!(
        distance(before, after) < 1.0,
        "a refused target moved the arm: {before:?} -> {after:?}"
    );

    rig.shutdown();
}
