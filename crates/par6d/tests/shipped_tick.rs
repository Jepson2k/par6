//! The SHIPPED 250 Hz configuration, end to end — the one test that runs
//! `config/PAR6.toml` with its `tick_dt_s = 0.004` UNPATCHED.
//!
//! Every other harness re-ticks the config for CI headroom
//! (`sim_session.rs` 0.02, `ffi_kinematics.rs` 0.02, the python e2e rig
//! 0.05), so before this test nothing ever exercised the ~20 derived
//! `round(seconds/dt)` counts, the 32-frame RX cap, the 7-node poll
//! cadence or the 5:1 status decimation at the values that meet the arm —
//! and the repo has already been bitten by exactly that class of bug
//! (`core_errors.rs` documents the stream watchdog rounding to a single
//! unsatisfiable tick at a non-default dt).
//!
//! Run it in RELEASE — it skips itself in debug builds, where the RT loop
//! cannot hold a 4 ms period anywhere. CI runs it as the
//! `shipped 250 Hz soak` step; the same invocation doubles as the
//! standard PRE-DEPLOY command on the control box, where the printed p99
//! must additionally clear the tight on-box band:
//!
//! ```bash
//! source .ffi/env.sh
//! PAR6_P99_FACTOR=1.05 cargo test -p par6d --release --features ffi \
//!     --test shipped_tick -- --nocapture
//! ```
//!
//! The hard gate here is that `LOOP_CRITICAL` never latches over a
//! representative session. The p99 assertion is deliberately generous
//! (see `p99_factor`) because shared CI runners have no RT scheduling.
#![cfg(feature = "ffi")]

use std::net::{SocketAddr, UdpSocket};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use par6_proto::command::{JogJ, MoveJ, Teleport};
use par6_proto::{
    decode_reply, decode_status, encode_command, Command, ErrorCode, QueryResult, Reply, Status,
    WireError, NUM_JOINTS,
};
use par6d::options::StatusTransport;
use par6d::{Daemon, Options};

const BUDGET: Duration = Duration::from_secs(20);
const READ_TIMEOUT: Duration = Duration::from_millis(100);

/// CI gate on the loop p99, as a multiple of the tick period
/// (`PAR6_P99_FACTOR` overrides; the pre-deploy run on the box sets 1.05).
///
/// Why 3.0: a GitHub runner is a shared, non-RT host — the sim loop
/// paces itself with ordinary sleeps and eats whatever scheduler jitter
/// its neighbours cause, so the tight 1.05·dt production band would turn
/// host load into red CI. 3.0·dt is still strictly inside the 4.0·dt
/// sim LOOP_CRITICAL band with margin, and a "250 Hz" runtime whose p99
/// cannot even do 12 ms is not a 250 Hz runtime on any host. The number
/// that matters for deployment is PRINTED, and judged at 1.05 on the box.
fn p99_factor() -> f64 {
    std::env::var("PAR6_P99_FACTOR")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(3.0)
}

fn config_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml")
}

fn park_deg() -> [f64; NUM_JOINTS] {
    let cfg = par6_config::RobotConfig::load(&config_path()).expect("PAR6 config");
    let mut a = [0.0; NUM_JOINTS];
    for (out, rad) in a.iter_mut().zip(cfg.robot.park_pose_rad.iter()) {
        *out = rad.to_degrees();
    }
    a
}

fn is_timeout(e: &std::io::Error) -> bool {
    matches!(
        e.kind(),
        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
    )
}

struct Rig {
    daemon: Option<Daemon>,
    status_rx: UdpSocket,
    _telemetry_rx: UdpSocket,
}

impl Rig {
    /// Boot the full runtime on the UNPATCHED shipped config.
    fn boot() -> Rig {
        let _ = env_logger::builder().is_test(true).try_init();
        let status_rx = UdpSocket::bind("127.0.0.1:0").expect("status socket");
        status_rx
            .set_read_timeout(Some(READ_TIMEOUT))
            .expect("timeout");
        let telemetry_rx = UdpSocket::bind("127.0.0.1:0").expect("telemetry socket");
        let opts = Options {
            sim: true,
            config: Some(config_path()),
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
                assert_never_loop_critical(&s);
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

/// The hard gate, applied to every STATUS this test ever decodes.
fn assert_never_loop_critical(s: &Status) {
    if let Some(WireError { code, .. }) = &s.error {
        assert_ne!(
            *code,
            ErrorCode::SysLoopCritical as u16,
            "LOOP_CRITICAL latched at the shipped 250 Hz configuration"
        );
    }
}

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

fn teleport_home(rig: &Rig, c: &mut Client, angles: [f64; NUM_JOINTS]) {
    let deadline = Instant::now() + BUDGET;
    loop {
        c.send(&Command::Teleport(Teleport {
            angles,
            tool_positions: None,
        }));
        let window = Instant::now() + Duration::from_millis(400);
        while Instant::now() < window {
            if let Some(s) = rig.recv_status() {
                assert_never_loop_critical(&s);
                if s.homed
                    && s.angles
                        .iter()
                        .zip(&angles)
                        .all(|(a, b)| (a - b).abs() <= 1.0)
                {
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

/// One representative session at the unpatched 4 ms tick: enable,
/// teleport-home, a queued move to COMPLETE, a UI-style jog burst with
/// its watchdog self-termination, and STATUS streamed at the real 5:1
/// decimation — then the loop's own account of itself. `LOOP_CRITICAL`
/// must never latch, and the measured p99 is printed and gated (see
/// `p99_factor` for the CI-vs-box split).
#[test]
fn shipped_250hz_configuration_holds_a_full_session() {
    if cfg!(debug_assertions) {
        eprintln!(
            "skipping: the shipped-rate soak needs --release (a debug RT \
             loop cannot hold 4 ms anywhere)"
        );
        return;
    }

    // Guard the premise: this test exists to run the SHIPPED numbers, so
    // it fails loudly if the config it points at stops being them.
    let cfg = par6_config::RobotConfig::load(&config_path()).expect("PAR6 config");
    let dt = cfg.robot.tick_dt_s;
    assert_eq!(dt, 0.004, "this soak must run the shipped tick period");
    assert_eq!(
        cfg.protocol.status_rate_hz, 50,
        "this soak must run the shipped status decimation"
    );

    let rig = Rig::boot();
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);

    // -- enable + sim-home ------------------------------------------------
    c.ok(&Command::Reset);
    let park = park_deg();
    teleport_home(&rig, &mut c, park);

    // -- a queued move runs to COMPLETE (LOOP_CRITICAL would DISABLE the
    //    controller mid-flight and fail it).
    let mut target = park;
    target[0] += 12.0;
    let index = c.ok_index(&Command::MoveJ(MoveJ {
        key: 4001,
        angles: target,
        duration: Some(1.0),
        speed: None,
        accel: None,
        blend_radius: None,
        rel: false,
    }));
    let (ok, detail) = c.wait_complete(index);
    assert!(ok, "move_j must complete ok at 250 Hz, got {detail:?}");
    rig.wait_status("move landed", |s| s.completed_index >= index as i64);

    // -- a UI-style jog burst: fresh watchdogged jogs at ~10 Hz ----------
    let before = rig.wait_status("angles", |_| true).angles[0];
    for _ in 0..12 {
        c.send(&Command::JogJ(JogJ {
            speeds: [-0.3, 0.0, 0.0, 0.0, 0.0, 0.0],
            duration: 0.3,
            accel: None,
        }));
        std::thread::sleep(Duration::from_millis(100));
    }
    rig.wait_status("jog moved the arm", |s| s.angles[0] < before - 1.0);
    rig.wait_status("jog watchdog self-terminated", |s| s.speeds[0].abs() < 0.05);

    // -- STATUS at the real decimation: ~50 Hz on the wire, not 250 -------
    let window = Duration::from_secs(3);
    let end = Instant::now() + window;
    let mut frames = 0u32;
    let mut last_seq = None;
    while Instant::now() < end {
        if let Some(s) = rig.recv_status() {
            assert_never_loop_critical(&s);
            if let Some(prev) = last_seq {
                assert!(s.seq > prev, "seq must advance");
            }
            last_seq = Some(s.seq);
            frames += 1;
        }
    }
    // Generous bounds around 150: enough to prove the broadcaster is
    // alive at ~50 Hz, far below the 750 an undecimated tick would emit.
    assert!(
        (90..=300).contains(&frames),
        "expected ~150 STATUS frames in {window:?} at 50 Hz, saw {frames}"
    );

    // -- the loop's own account -------------------------------------------
    let deadline = Instant::now() + BUDGET;
    let stats = loop {
        match c.query(&Command::LoopStats) {
            QueryResult::LoopStats(ls) if ls.p99_period_s > 0.0 => break ls,
            QueryResult::LoopStats(_) => {}
            other => panic!("unexpected loop_stats result {other:?}"),
        }
        assert!(Instant::now() < deadline, "loop stats never populated");
    };
    println!(
        "shipped-tick soak: target {} Hz, {} ticks, {} overruns, mean {:.6} s, \
         p95 {:.6} s, p99 {:.6} s, max {:.6} s",
        stats.target_hz,
        stats.loop_count,
        stats.overrun_count,
        stats.mean_period_s,
        stats.p95_period_s,
        stats.p99_period_s,
        stats.max_period_s,
    );
    assert_eq!(
        stats.target_hz, 250.0,
        "the RT loop must run the shipped rate"
    );
    assert!(
        stats.loop_count > 2000,
        "the soak must have covered thousands of real ticks, got {}",
        stats.loop_count
    );
    let bound = p99_factor() * dt;
    assert!(
        stats.p99_period_s <= bound,
        "loop p99 {:.6} s exceeds {:.6} s ({}x dt) — the shipped 250 Hz \
         configuration does not hold on this host",
        stats.p99_period_s,
        bound,
        p99_factor(),
    );

    // No standing error of any kind may survive the session.
    match c.query(&Command::Error) {
        QueryResult::Error { error } => {
            assert!(error.is_none(), "session left a standing error: {error:?}")
        }
        other => panic!("unexpected error result {other:?}"),
    }

    rig.shutdown();
}
