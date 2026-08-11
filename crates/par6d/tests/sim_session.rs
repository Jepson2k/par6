//! Full-stack integration: the complete par6d runtime booted with the
//! sim backend on ephemeral ports, driven exclusively through the REAL
//! protocol-v2 encoding over UDP (par6-proto client side). One test
//! boots in-process for the protocol workflow; one spawns the actual
//! binary to prove the CLI/ready-line/signal path. All waiting is
//! deadline-bounded polling — no blind sleeps stand in for conditions.

use std::io::BufRead;
use std::net::{SocketAddr, UdpSocket};
use std::path::PathBuf;
use std::process::Stdio;
use std::time::{Duration, Instant};

use par6_proto::command::{JogJ, MoveJ, Stop, Teleport};
use par6_proto::{
    decode_reply, decode_status, encode_command, ActionState, Command, ErrorCode, QueryResult,
    Reply, Status, WireError, NUM_JOINTS,
};
use par6d::options::StatusTransport;
use par6d::{Daemon, Options};

const BUDGET: Duration = Duration::from_secs(20);
const READ_TIMEOUT: Duration = Duration::from_millis(100);

fn config_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml")
}

/// The PAR6 config re-ticked to 50 Hz for the in-process session test.
/// Loaded CI machines without RT scheduling miss 4 ms deadlines and
/// would latch LOOP_CRITICAL mid-test; every RT time constant derives
/// from config seconds (`round(s/dt)`), so the runtime is rate-agnostic
/// by contract and the wiring under test is identical.
fn test_config() -> PathBuf {
    let src = config_path();
    let dir = std::env::temp_dir().join(format!("par6d-sim-session-{}", std::process::id()));
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

/// The config park pose in wire units (degrees) — inside every joint's
/// soft window, so it works as a teleport target and move base.
fn park_deg() -> [f64; NUM_JOINTS] {
    let cfg = par6_config::RobotConfig::load(&config_path()).expect("PAR6 config");
    let mut a = [0.0; NUM_JOINTS];
    for (out, rad) in a.iter_mut().zip(cfg.robot.park_pose_rad.iter()) {
        *out = rad.to_degrees();
    }
    a
}

fn with_j0(base: [f64; NUM_JOINTS], delta_deg: f64) -> [f64; NUM_JOINTS] {
    let mut a = base;
    a[0] += delta_deg;
    a
}

fn angles_close(a: &[f64; NUM_JOINTS], b: &[f64; NUM_JOINTS], tol_deg: f64) -> bool {
    a.iter().zip(b).all(|(x, y)| (x - y).abs() <= tol_deg)
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

fn jog_j(j0_speed: f64, duration_s: f64) -> Command {
    Command::JogJ(JogJ {
        speeds: [j0_speed, 0.0, 0.0, 0.0, 0.0, 0.0],
        duration: duration_s,
        accel: None,
    })
}

fn teleport(angles_deg: [f64; NUM_JOINTS]) -> Command {
    Command::Teleport(Teleport {
        angles: angles_deg,
        tool_positions: None,
    })
}

// ---- in-process rig --------------------------------------------------------

struct Rig {
    daemon: Option<Daemon>,
    status_rx: UdpSocket,
    _telemetry_rx: UdpSocket,
}

impl Rig {
    fn boot() -> Rig {
        let _ = env_logger::builder().is_test(true).try_init();
        let status_rx = UdpSocket::bind("127.0.0.1:0").expect("status socket");
        status_rx
            .set_read_timeout(Some(READ_TIMEOUT))
            .expect("timeout");
        let telemetry_rx = UdpSocket::bind("127.0.0.1:0").expect("telemetry socket");
        let opts = Options {
            sim: true,
            config: Some(test_config()),
            command_port: Some(0),
            bind: Some("127.0.0.1".parse().unwrap()),
            status_host: Some("127.0.0.1".parse().unwrap()),
            status_port: Some(status_rx.local_addr().unwrap().port()),
            telemetry_port: Some(telemetry_rx.local_addr().unwrap().port()),
            status_transport: Some(StatusTransport::Unicast),
            help: false,
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

    /// One STATUS datagram, or `None` on a quiet 100 ms window.
    fn recv_status(&self) -> Option<Status> {
        let mut buf = [0u8; 65535];
        match self.status_rx.recv_from(&mut buf) {
            Ok((n, _)) => Some(decode_status(&buf[..n]).expect("decodable status")),
            Err(e) if is_timeout(&e) => None,
            Err(e) => panic!("status recv failed: {e}"),
        }
    }

    /// Poll the broadcast until `pred` holds; panics after the budget.
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

    /// Send and wait for the direct reply with the matching `req_id`,
    /// stashing COMPLETE pushes and dropping stale replies.
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

    fn query(&mut self, cmd: &Command) -> QueryResult {
        match self.request(cmd) {
            Reply::Response { result, .. } => result,
            other => panic!("expected RESPONSE, got {other:?}"),
        }
    }

    fn ok(&mut self, cmd: &Command) {
        match self.request(cmd) {
            Reply::Ok { index: None, .. } => {}
            other => panic!("expected plain OK, got {other:?}"),
        }
    }

    fn ok_index(&mut self, cmd: &Command) -> u64 {
        match self.request(cmd) {
            Reply::Ok { index: Some(i), .. } => i,
            other => panic!("expected OK with index, got {other:?}"),
        }
    }

    fn expect_error(&mut self, cmd: &Command) -> WireError {
        match self.request(cmd) {
            Reply::Error { error, .. } => error,
            other => panic!("expected ERROR, got {other:?}"),
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

    fn saw_complete(&self, index: u64) -> bool {
        self.completes.iter().any(|c| c.0 == index)
    }
}

// ---- tests -----------------------------------------------------------------

/// One protocol session against the fully wired sim runtime: ping →
/// status header/broadcast, enable, teleport (sim-only fast homing),
/// queued move_j to COMPLETE + completed_index high-water, jog
/// preemption of a queued move (with real sim motion and jog
/// self-termination), stop {clear_queue}, e-stop latch + reset, and the
/// never-reset index allocator. Cancelled commands must never push
/// COMPLETE.
#[test]
fn full_sim_session_over_protocol_v2() {
    let rig = Rig::boot();
    let mut c = Client::new(rig.addr());

    // Ping answers over the real codec; sim mode reports no hardware.
    match c.query(&Command::Ping) {
        QueryResult::Ping { hardware_connected } => assert!(!hardware_connected),
        other => panic!("unexpected ping result {other:?}"),
    }

    // STATUS broadcast: sane v2 header, fresh RT link, un-homed at boot.
    let s1 = rig.wait_status("link_ok", |s| s.link_ok == 1);
    assert_eq!(s1.proto_version, 2);
    assert!(s1.simulator_active);
    assert!(!s1.homed);
    assert_eq!(s1.executing_index, -1);
    assert!(s1.error.is_none());
    let s2 = rig.wait_status("seq advances", |s| s.seq > s1.seq);
    assert!(s2.mono_time_ns >= s1.mono_time_ns);

    // Un-homed planned motion is refused before dispatch.
    let err = c.expect_error(&move_j(900, park_deg(), 0.5));
    assert!(
        err.code == ErrorCode::MotnNotHomed as u16
            || err.code == ErrorCode::SysControllerDisabled as u16,
        "un-homed/disabled move must be rejected, got {err:?}"
    );

    // reset enables the controller (the RT clear sequence settles first,
    // so the effect is polled, not assumed).
    c.ok(&Command::Reset);

    // Teleport: accepted in sim once ENABLED; homed=true plus the pose
    // land in the broadcast. Re-sent until the gate passes.
    let park = park_deg();
    let deadline = Instant::now() + BUDGET;
    'teleport: loop {
        c.send(&teleport(park));
        let window = Instant::now() + Duration::from_millis(400);
        while Instant::now() < window {
            if let Some(s) = rig.recv_status() {
                if s.homed && angles_close(&s.angles, &park, 1.0) {
                    break 'teleport;
                }
            }
        }
        assert!(
            Instant::now() < deadline,
            "teleport did not take effect within budget"
        );
    }

    // Queued move_j: ack carries the index, COMPLETE pushes, and the
    // completed_index high-water mark + real sim motion land in STATUS.
    // Position tolerance is coarse on purpose: the sim's Spectral
    // cascade treats the commanded velocity as a cap, so fast moves
    // carry a few degrees of tracking lag and `settled` completes via
    // its bounded timeout — the assertion proves real closed-loop
    // motion toward the target, not servo-grade tracking.
    let target1 = with_j0(park, 10.0);
    let i1 = c.ok_index(&move_j(1001, target1, 0.5));
    let (ok, detail) = c.wait_complete(i1);
    assert!(ok, "move_j must complete ok, got {detail:?}");
    let s = rig.wait_status("completed_index reaches the move", |s| {
        s.completed_index >= i1 as i64
    });
    assert!(
        s.angles[0] > park[0] + 5.0 && (s.angles[0] - target1[0]).abs() < 6.0,
        "J0 must have driven toward the target; got {:?}",
        s.angles
    );

    // Jog preempts a queued move: the planned motion vanishes without a
    // COMPLETE, the jog physically drives the sim, then self-terminates
    // when its duration watchdog expires.
    let i2 = c.ok_index(&move_j(1002, park, 3.0));
    rig.wait_status("long move starts executing", |s| {
        s.executing_index == i2 as i64
    });
    let before = rig.wait_status("pose sample before jog", |_| true).angles[0];
    for _ in 0..3 {
        c.send(&jog_j(0.3, 0.5));
        std::thread::sleep(Duration::from_millis(40)); // UI-style jog stream pacing
    }
    rig.wait_status("jog cancelled the planned motion", |s| {
        s.executing_index == -1 && s.queued_segments == 0
    });
    rig.wait_status("jog drives J0 positive", |s| s.angles[0] > before + 0.3);
    rig.wait_status("jog self-terminates after its duration", |s| {
        s.action_state == ActionState::Idle && s.speeds[0].abs() < 0.05
    });

    // stop {clear_queue: true}: nothing executing, nothing pending, and
    // the controller stays usable afterwards.
    let i3 = c.ok_index(&move_j(1003, park, 3.0));
    let i4 = c.ok_index(&move_j(1004, target1, 0.5));
    c.ok(&Command::Stop(Stop { clear_queue: true }));
    match c.query(&Command::Queue) {
        QueryResult::Queue {
            queue,
            executing_index,
            ..
        } => {
            assert!(queue.is_empty(), "stop must clear the queue: {queue:?}");
            assert_eq!(executing_index, -1);
        }
        other => panic!("unexpected {other:?}"),
    }

    // E-stop: latches DISABLED with a standing error until reset.
    c.ok(&Command::Estop);
    let err = c.expect_error(&move_j(1005, park, 0.5));
    assert_eq!(err.code, ErrorCode::SysEstopActive as u16);
    match c.query(&Command::Error) {
        QueryResult::Error { error: Some(e) } => {
            assert_eq!(e.code, ErrorCode::SysEstopActive as u16);
        }
        other => panic!("standing e-stop error expected, got {other:?}"),
    }
    c.ok(&Command::Reset);
    // Re-enable propagates through the RT clear-sequence settle window,
    // so early attempts are either rejected (DISABLED) or accepted and
    // then truthfully failed via COMPLETE(ok=false) with the e-stop
    // error — acceptance is not success. Retry with fresh keys until a
    // move runs to a clean COMPLETE, like a real client would.
    let retriable = |code: u16| {
        code == ErrorCode::SysControllerDisabled as u16
            || code == ErrorCode::SysEstopActive as u16
            || code == ErrorCode::MotnSetupFailed as u16
    };
    let deadline = Instant::now() + BUDGET;
    let mut attempt = 0u64;
    let i5 = loop {
        attempt += 1;
        match c.request(&move_j(2000 + attempt, park, 0.5)) {
            Reply::Ok { index: Some(i), .. } => {
                let (ok, detail) = c.wait_complete(i);
                if ok {
                    break i;
                }
                let detail = detail.expect("failed COMPLETE carries detail");
                assert!(
                    retriable(detail.code),
                    "unexpected failure while re-enabling: {detail:?}"
                );
            }
            Reply::Error { error, .. } => {
                assert!(
                    retriable(error.code),
                    "unexpected rejection while re-enabling: {error:?}"
                );
                std::thread::sleep(Duration::from_millis(50));
            }
            other => panic!("unexpected reply {other:?}"),
        }
        assert!(
            Instant::now() < deadline,
            "controller did not re-enable after reset"
        );
    };
    assert!(
        i5 > i4,
        "the index allocator is never reset ({i5} must follow {i4})"
    );

    // Cancellation drops commands WITHOUT a COMPLETE push (jog-preempted
    // i2, stopped i3, queue-cleared i4).
    for i in [i2, i3, i4] {
        assert!(!c.saw_complete(i), "cancelled command {i} pushed COMPLETE");
    }

    rig.shutdown();
}

/// The real binary: `--sim --port 0` boots on an ephemeral port,
/// announces it in the machine-readable ready line, answers PING there,
/// and exits cleanly (code 0, no abort) on SIGTERM — the exact flow the
/// python `Robot.start()` bootstrap relies on.
#[test]
fn daemon_binary_ephemeral_port_ready_line_and_sigterm() {
    let status_rx = UdpSocket::bind("127.0.0.1:0").expect("status sink");
    let telemetry_rx = UdpSocket::bind("127.0.0.1:0").expect("telemetry sink");
    let mut child = std::process::Command::new(env!("CARGO_BIN_EXE_par6d"))
        .args([
            "--sim",
            "--config",
            config_path().to_str().expect("utf-8 path"),
            "--port",
            "0",
            "--bind",
            "127.0.0.1",
            "--status-transport",
            "unicast",
            "--status-host",
            "127.0.0.1",
            "--status-port",
            &status_rx.local_addr().unwrap().port().to_string(),
            "--telemetry-port",
            &telemetry_rx.local_addr().unwrap().port().to_string(),
        ])
        .stdout(Stdio::piped())
        .spawn()
        .expect("spawn par6d");

    let stdout = child.stdout.take().expect("piped stdout");
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let mut line = String::new();
        let _ = std::io::BufReader::new(stdout).read_line(&mut line);
        let _ = tx.send(line);
    });
    let line = rx.recv_timeout(BUDGET).expect("ready line within budget");
    assert!(
        line.starts_with("PAR6D_READY "),
        "machine-readable ready line, got: {line:?}"
    );
    let port: u16 = line
        .split_whitespace()
        .find_map(|kv| kv.strip_prefix("command_port="))
        .expect("command_port key in ready line")
        .parse()
        .expect("numeric port");
    assert_ne!(port, 0, "ephemeral port must be resolved");

    let mut c = Client::new(SocketAddr::from(([127, 0, 0, 1], port)));
    match c.query(&Command::Ping) {
        QueryResult::Ping { .. } => {}
        other => panic!("unexpected ping result {other:?}"),
    }

    // SAFETY: plain kill(2) on our own child with a standard signal.
    unsafe { libc::kill(child.id() as i32, libc::SIGTERM) };
    let deadline = Instant::now() + BUDGET;
    let status = loop {
        if let Some(st) = child.try_wait().expect("try_wait") {
            break st;
        }
        assert!(
            Instant::now() < deadline,
            "par6d did not exit after SIGTERM"
        );
        std::thread::sleep(Duration::from_millis(50));
    };
    assert!(status.success(), "clean exit expected, got {status:?}");
}

/// Startup failure paths are clear errors, never panics: hardware mode
/// without a CAN interface names the interface and points at `--sim`;
/// a missing config file names the path it tried.
#[test]
fn hardware_mode_and_bad_config_fail_with_clear_errors() {
    let opts = Options {
        sim: false,
        config: Some(config_path()),
        ..Options::default()
    };
    let err = Daemon::start(&opts)
        .err()
        .expect("hardware mode must fail cleanly in CI");
    let msg = err.to_string();
    assert!(msg.contains("can0"), "names the CAN interface: {msg}");
    assert!(msg.contains("--sim"), "points at the simulator: {msg}");

    let opts = Options {
        sim: true,
        config: Some(PathBuf::from("/nonexistent/par6.toml")),
        ..Options::default()
    };
    let err = Daemon::start(&opts)
        .err()
        .expect("missing config must fail cleanly");
    assert!(
        err.to_string().contains("/nonexistent/par6.toml"),
        "names the missing path: {err}"
    );
}
