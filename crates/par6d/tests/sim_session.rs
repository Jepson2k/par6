//! Full-stack integration: the complete par6d runtime booted with the
//! sim backend on ephemeral ports, driven exclusively through the REAL
//! protocol-v2 encoding over UDP (par6-proto client side). Most tests
//! boot in-process for a protocol workflow; one spawns the actual binary
//! to prove the CLI/ready-line/signal path. All waiting is
//! deadline-bounded polling — no blind sleeps stand in for conditions.
//!
//! These tests exercise the protocol plane of exactly the binary that
//! deploys.

use std::io::BufRead;
use std::net::{SocketAddr, UdpSocket};
use std::path::PathBuf;
use std::process::Stdio;
use std::time::{Duration, Instant};

use par6_proto::command::{
    JogJ, MoveC, MoveJ, SelectProfile, SelectTool, SetCompletionPolicy, Stop, Teleport, ToolAction,
    ToolParam,
};
use par6_proto::{
    ActionState, Command, CompletionPolicy, ErrorCode, Frame, QueryResult, Reply, Status,
    ToolState, NUM_JOINTS,
};
use par6d::{Daemon, Options};

mod common;
use common::{Client, Rig, BUDGET};

/// The PAR6 config re-ticked to 50 Hz for the in-process session test.
/// Loaded CI machines without RT scheduling miss 4 ms deadlines and
/// would latch LOOP_CRITICAL mid-test; every RT time constant derives
/// from config seconds (`round(s/dt)`), so the runtime is rate-agnostic
/// by contract and the wiring under test is identical.
fn test_config() -> PathBuf {
    let src = common::shipped_config();
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
    let cfg = par6_config::RobotConfig::load(&common::shipped_config()).expect("PAR6 config");
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

fn max_deg_error(a: &[f64; NUM_JOINTS], b: &[f64; NUM_JOINTS]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

/// Where the configured homing sequence leaves the arm: its `move_to`
/// steps replayed in order, last write per joint. Derived from the same
/// config the runtime executes, never a transcribed constant.
fn ready_pose_deg() -> [f64; NUM_JOINTS] {
    let cfg = par6_config::RobotConfig::load(&common::shipped_config()).expect("PAR6 config");
    let mut a = [f64::NAN; NUM_JOINTS];
    for step in &cfg.homing.sequence {
        for m in &step.move_to {
            a[usize::from(m.joint)] = m.position_rad.to_degrees();
        }
    }
    assert!(
        a.iter().all(|v| v.is_finite()),
        "the homing sequence must place every joint"
    );
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

/// The gripper `par6d` is actually fitted with, in the canonical
/// (upper-case) spelling the python client sends.
fn fitted_tool() -> String {
    par6_config::RobotConfig::load(&common::shipped_config())
        .expect("PAR6 config")
        .robot
        .active_gripper
        .to_uppercase()
}

fn select_tool(key: u64, tool: &str, variant: Option<&str>) -> Command {
    Command::SelectTool(SelectTool {
        key,
        tool_name: tool.to_owned(),
        variant_key: variant.map(str::to_owned),
    })
}

fn tool_action(key: u64, tool: &str, action: &str, params: &[f64]) -> Command {
    Command::ToolAction(ToolAction {
        key,
        tool_key: tool.to_owned(),
        action: action.to_owned(),
        params: params.iter().map(|v| ToolParam::Float(*v)).collect(),
    })
}

fn select_profile(name: &str) -> Command {
    Command::SelectProfile(SelectProfile {
        profile: name.to_owned(),
    })
}

// ---- in-process rig --------------------------------------------------------

/// Sim-only fast homing: re-send `teleport` until the broadcast shows
/// the pose landed and the arm reads homed (the gate needs ENABLED,
/// which the RT clear sequence reaches asynchronously).
fn teleport_home(rig: &Rig, c: &mut Client, angles: [f64; NUM_JOINTS]) {
    let deadline = Instant::now() + BUDGET;
    loop {
        c.send(&teleport(angles));
        let window = Instant::now() + Duration::from_millis(400);
        while Instant::now() < window {
            if let Some(s) = rig.recv_status() {
                if s.homed && angles_close(&s.angles, &angles, 1.0) {
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

#[test]
fn full_sim_session_over_protocol_v2() {
    let rig = Rig::boot(test_config());
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
    teleport_home(&rig, &mut c, park);

    // Queued move_j: ack carries the index, COMPLETE pushes, and the
    // completed_index high-water mark + real sim motion land in STATUS.
    // The post-profile hold closes the tracking residual on position
    // error alone, so `settled` completion leaves the arm on the target
    // for real.
    let target1 = with_j0(park, 10.0);
    let i1 = c.ok_index(&move_j(1001, target1, 0.5));
    let (ok, detail) = c.wait_complete(i1);
    assert!(ok, "move_j must complete ok, got {detail:?}");
    let s = rig.wait_status("completed_index reaches the move", |s| {
        s.completed_index >= i1 as i64
    });
    assert!(
        s.angles[0] > park[0] + 5.0 && (s.angles[0] - target1[0]).abs() < 1.0,
        "J0 must have settled on the target; got {:?}",
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
            common::shipped_config().to_str().expect("utf-8 path"),
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
        config: Some(common::shipped_config()),
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

/// Issue #15 regression: a `stop` immediately followed by a queued move.
/// The stop's EXEC flush rides the RT command queue (one command per
/// tick) while the new move's samples ride the SPSC ring (immediate), so
/// an unbounded flush lands AFTER those samples and erases them — EXEC
/// then holds forever and the move never completes. The move must run to
/// COMPLETE and the arm must actually be at the target.
#[test]
fn stop_then_move_completes_without_losing_samples() {
    let rig = Rig::boot(test_config());
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    let park = park_deg();
    teleport_home(&rig, &mut c, park);

    // A long move to stop in the middle of, then — back to back with the
    // stop — a fresh short move.
    let i_long = c.ok_index(&move_j(3001, with_j0(park, 40.0), 6.0));
    rig.wait_status("the long move is executing", |s| {
        s.executing_index == i_long as i64
    });
    c.ok(&Command::Stop(Stop { clear_queue: true }));
    let target = with_j0(park, 10.0);
    let i_next = c.ok_index(&move_j(3002, target, 0.5));
    let (ok, detail) = c.wait_complete(i_next);
    assert!(
        ok,
        "the move queued right after a stop must complete, got {detail:?}"
    );
    let s = rig.wait_status("completed_index reaches the move", |s| {
        s.completed_index >= i_next as i64
    });
    assert!(
        (s.angles[0] - target[0]).abs() < 6.0,
        "the move after the stop never drove J0 to the target: {:?}",
        s.angles
    );
    assert!(!c.saw_complete(i_long), "the stopped move pushed COMPLETE");

    rig.shutdown();
}

/// Move size for the profile probe: short enough that the whole move is
/// ramping, where a jerk limit costs the most against a profile without
/// one (long moves are cruise-dominated and converge).
const PROFILE_PROBE_DEG: f64 = 4.0;

/// Time one `move_j` under `profile`, from a fixed start pose, measured
/// ack → COMPLETE. The profile is what shapes the trajectory, so its
/// duration is the observable that proves the selection reached the
/// planner rather than being stored and ignored.
fn timed_move_under(rig: &Rig, c: &mut Client, profile: &str, key: u64) -> Duration {
    let park = park_deg();
    teleport_home(rig, c, park);
    c.ok(&select_profile(profile));
    match c.query(&Command::Profile) {
        QueryResult::Profile { profile: p } => assert_eq!(p, profile),
        other => panic!("unexpected profile result {other:?}"),
    }
    let cmd = Command::MoveJ(MoveJ {
        key,
        angles: with_j0(park, PROFILE_PROBE_DEG),
        duration: None,
        speed: Some(1.0),
        accel: None,
        blend_radius: None,
        rel: false,
    });
    let started = Instant::now();
    let index = c.ok_index(&cmd);
    let (ok, detail) = c.wait_complete(index);
    assert!(ok, "{profile} move must complete, got {detail:?}");
    started.elapsed()
}

fn tool_status(s: &Status) -> par6_proto::ToolStatusWire {
    s.tool_status.clone().expect("tool_status in STATUS")
}

fn jaw(s: &Status) -> f64 {
    tool_status(s)
        .positions
        .first()
        .copied()
        .unwrap_or(f64::NAN)
}

/// The tool / profile / parameter surface, end to end over protocol v2.
///
/// Everything asserted here was previously either hard-rejected
/// (`tool_action`), silently ignored (`blend_radius`, `teleport`'s
/// `tool_positions`, the non-dominant axes of a multi-axis jog) or
/// impossible (`select_profile`, whose registry was never populated):
///
/// - unsupported parameters answer with a real ERROR instead of being
///   dropped on the floor,
/// - the motion-profile registry holds the profiles the planner really
///   implements, and selecting one changes the planned trajectory,
/// - `select_tool` is constrained to the tool the runtime is fitted
///   with, and `tool_action` drives the simulated gripper's jaw — the
///   physical outcome arrives through the real cmd-60 status path in
///   the STATUS broadcast.
#[test]
fn tool_actions_profiles_and_unsupported_parameters() {
    let rig = Rig::boot(test_config());
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);
    let park = park_deg();
    teleport_home(&rig, &mut c, park);

    // ---- parameters that cannot be honoured are refused, never ignored.
    // A corner radius on an ARC is one of them: par6d rounds corners
    // between straight cartesian moves and between joint moves, but an
    // arc ends at its end pose, and a radius that quietly did nothing
    // would be the silent alteration this surface exists to prevent.
    let err = c.expect_error(&Command::MoveC(MoveC {
        key: 5001,
        via: [0.0; 6],
        end: [0.0; 6],
        frame: Frame::Wrf,
        duration: Some(0.5),
        speed: None,
        accel: None,
        blend_radius: Some(5.0),
    }));
    assert_eq!(
        err.code,
        ErrorCode::CommValidationError as u16,
        "a blend radius par6d cannot honour must be refused, got {err:?}"
    );
    let err = c.expect_error(&Command::JogJ(JogJ {
        speeds: [0.2, 0.2, 0.0, 0.0, 0.0, 0.0],
        duration: 0.2,
        accel: None,
    }));
    assert_eq!(
        err.code,
        ErrorCode::CommValidationError as u16,
        "a multi-axis jog must be refused rather than collapsed, got {err:?}"
    );
    let err = c.expect_error(&Command::Teleport(Teleport {
        angles: park,
        tool_positions: Some(vec![0.5, 0.5]),
    }));
    assert_eq!(
        err.code,
        ErrorCode::CommValidationError as u16,
        "a tool_positions length the tool does not have must be refused, got {err:?}"
    );
    let err = c.expect_error(&Command::Teleport(Teleport {
        angles: park,
        tool_positions: Some(vec![1.5]),
    }));
    assert_eq!(
        err.code,
        ErrorCode::CommValidationError as u16,
        "an out-of-range tool position must be refused, got {err:?}"
    );
    // …and so is an ANGLE outside the joint's travel. It used to be
    // clamped into range with an OK reply, which landed the arm tens of
    // degrees from the request and told the client it had arrived.
    let cfg = par6_config::RobotConfig::load(&common::shipped_config()).expect("PAR6 config");
    let mut past_hard = park;
    past_hard[2] = cfg.joints[2].limits.hard_max_rad.to_degrees() + 10.0;
    let err = c.expect_error(&teleport(past_hard));
    assert_eq!(
        err.code,
        ErrorCode::CommValidationError as u16,
        "an unreachable teleport angle must be refused, got {err:?}"
    );
    assert!(
        err.cause.contains("angles[2]"),
        "the refusal must name the joint that could not be placed: {}",
        err.cause
    );
    let s = rig.wait_status("a refused teleport moved nothing", |_| true);
    assert!(
        (s.angles[2] - park[2]).abs() < 1.0,
        "a refused teleport must leave the arm where it was, not clamp it: {:?}",
        s.angles
    );
    // ---- the profile registry is real: unknown names are refused, the
    // advertised ones are selectable, and the choice reaches the planner
    // (jerk-limited ruckig cannot beat the unlimited-jerk trapezoid over
    // the same move under the same limits).
    let err = c.expect_error(&select_profile("BOGUS"));
    assert_eq!(err.code, ErrorCode::SysProfileInvalid as u16);
    // Time the trajectory, not the servo: under `commanded` a move
    // finishes when its last sample has been issued, so the measurement
    // is the planned duration plus a datagram.
    c.ok(&Command::SetCompletionPolicy(SetCompletionPolicy {
        policy: CompletionPolicy::Commanded,
    }));
    let trapezoid = timed_move_under(&rig, &mut c, "TRAPEZOID", 5100);
    let ruckig = timed_move_under(&rig, &mut c, "RUCKIG", 5101);
    assert!(
        ruckig > trapezoid.mul_f64(1.4),
        "the selected profile did not change the trajectory: \
         TRAPEZOID {trapezoid:?} vs RUCKIG {ruckig:?}"
    );
    let toppra = timed_move_under(&rig, &mut c, "TOPPRA", 5102);
    assert!(
        toppra < ruckig,
        "TOPPRA (time-optimal, no jerk limit) must not be slower than \
         jerk-limited RUCKIG: {toppra:?} vs {ruckig:?}"
    );

    // ---- tools. The fitted tool reports from boot — a client does not
    // have to ask for the tool the runtime is physically wearing — and
    // no other tool can be selected.
    let tool = fitted_tool();
    let s = rig.wait_status("tool status reaches STATUS", |s| s.tool_status.is_some());
    let ts = tool_status(&s);
    assert_eq!(ts.key.to_uppercase(), tool);
    assert_eq!(ts.fault_code, 0, "a healthy gripper must report no fault");
    let err = c.expect_error(&select_tool(6001, "FLANGE", None));
    assert_eq!(
        err.code,
        ErrorCode::CommValidationError as u16,
        "selecting a tool the runtime is not fitted with must be refused, got {err:?}"
    );
    // The key is matched case-insensitively (clients canonicalise it),
    // and the variant does reach STATUS.
    let i = c.ok_index(&select_tool(6002, &tool, Some("wide")));
    let (ok, detail) = c.wait_complete(i);
    assert!(ok, "select_tool must complete, got {detail:?}");
    rig.wait_status("the selected variant reaches STATUS", |s| {
        s.tool_status
            .as_ref()
            .is_some_and(|t| t.variant_key == "wide")
    });

    // ---- tool_action drives the jaw. Closing runs it to the commanded
    // position with nothing between the jaws (detection: reached, no
    // object), opening runs it back.
    let before = jaw(&s);
    let i = c.ok_index(&tool_action(6003, &tool, "move", &[1.0, 0.5, 500.0]));
    let (ok, detail) = c.wait_complete(i);
    assert!(ok, "gripper close must complete, got {detail:?}");
    let s = rig.wait_status("the jaw reaches the closed command and stops", |s| {
        jaw(s) > 0.99 && tool_status(s).state == ToolState::Idle
    });
    assert!(
        before < 0.95,
        "the jaw was already closed before the command; travel proves nothing"
    );
    assert!(
        !tool_status(&s).part_detected,
        "nothing is between the jaws; no part may be reported"
    );

    let i = c.ok_index(&tool_action(6004, &tool, "move", &[0.0, 0.5, 500.0]));
    let (ok, detail) = c.wait_complete(i);
    assert!(ok, "gripper open must complete, got {detail:?}");
    rig.wait_status("the jaw reaches the open command", |s| jaw(s) < 0.05);

    // ---- calibrate runs the firmware sweep and leaves the jaws open.
    let i = c.ok_index(&tool_action(6005, &tool, "calibrate", &[]));
    let (ok, detail) = c.wait_complete(i);
    assert!(ok, "gripper calibrate must complete, got {detail:?}");
    rig.wait_status("calibration leaves the jaws open", |s| jaw(s) < 0.05);

    // ---- an action the tool does not implement fails the command; it is
    // never reported as done.
    let i = c.ok_index(&tool_action(6006, &tool, "spin", &[]));
    let (ok, detail) = c.wait_complete(i);
    assert!(!ok, "an unknown tool action must fail");
    assert_eq!(
        detail.expect("failed COMPLETE carries detail").code,
        ErrorCode::CommValidationError as u16
    );
    // …and out-of-range move parameters are refused the same way.
    let i = c.ok_index(&tool_action(6007, &tool, "move", &[2.0, 0.5, 500.0]));
    assert!(!c.wait_complete(i).0, "position 2.0 must fail");

    // ---- teleport places the tool as well as the arm.
    let deadline = Instant::now() + BUDGET;
    loop {
        c.send(&Command::Teleport(Teleport {
            angles: park,
            tool_positions: Some(vec![0.6]),
        }));
        let window = Instant::now() + Duration::from_millis(400);
        let mut landed = false;
        while Instant::now() < window && !landed {
            if let Some(s) = rig.recv_status() {
                landed = (jaw(&s) - 0.6).abs() < 0.02;
            }
        }
        if landed {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "teleport did not place the tool within budget"
        );
    }

    rig.shutdown();
}

/// The enablement pair is POSITIVE slot first: `[j1+, j1−, …]`.
///
/// Parked past a joint's upper soft limit, the positive slot must read 0
/// and the negative slot 1. Nothing on the wire pins the order, so a
/// backend that fills the pair the other way round quietly greys out the
/// opposite jog button in a frontend that unpacks it as
/// `can_jog_pos[i] = slot[2i]` (which is what waldoctl and parol6 do).
#[test]
fn joint_enablement_slots_are_positive_direction_first() {
    let rig = Rig::boot(test_config());
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);
    teleport_home(&rig, &mut c, park_deg());

    // Between J0's soft and hard windows: teleport clamps to the hard
    // window, so the arm really lands outside the soft one.
    let cfg = par6_config::RobotConfig::load(&common::shipped_config()).expect("PAR6 config");
    let (soft_max, hard_max) = (
        cfg.joints[0].limits.soft_max_rad.to_degrees(),
        cfg.joints[0].limits.hard_max_rad.to_degrees(),
    );
    assert!(hard_max > soft_max, "the config must leave a soft margin");
    let mut past_max = park_deg();
    past_max[0] = 0.5 * (soft_max + hard_max);
    teleport_home(&rig, &mut c, past_max);

    // The enablement probe runs at the status cadence, so the frame that
    // first carries a new pose may still carry the previous answer: poll
    // for the settled one rather than reading the first arrival.
    let s = rig.wait_status("past the upper soft limit J0 may only move negative", |s| {
        (s.angles[0] - past_max[0]).abs() < 1.0 && (s.joint_en[0], s.joint_en[1]) == (0, 1)
    });
    // A joint inside its window keeps both directions — asserted on that
    // same frame, so it is this pose's answer and not a stale one.
    assert_eq!(
        (s.joint_en[2], s.joint_en[3]),
        (1, 1),
        "J1 sits inside its soft window: both directions stay free"
    );

    // The window is reported with parol6's margin, not as a bare
    // inequality: a joint a tenth of a degree short of its soft stop has
    // no usable freedom left, and a frontend that offers the button gets
    // a jog refused the moment it is pressed.
    let mut at_max = park_deg();
    at_max[0] = soft_max - 0.1;
    teleport_home(&rig, &mut c, at_max);
    let s = rig.wait_status("0.1 deg of travel is not freedom", |s| {
        (s.angles[0] - at_max[0]).abs() < 0.02 && (s.joint_en[0], s.joint_en[1]) == (0, 1)
    });
    assert_eq!(
        (s.joint_en[2], s.joint_en[3]),
        (1, 1),
        "only J0 is at its stop; J1 keeps both directions"
    );

    rig.shutdown();
}

/// Gap 14: `home` on an arm that is already referenced is a planned
/// return to the configured park pose, not another full referencing
/// seek (parol6 `server/motion_planner.py:239-241` routes `HomeCmd` to
/// a `MoveJCmd(HOME_ANGLES_DEG, HOME_RETURN_SPEED_FRAC)` when
/// `Homed_in[:6].all()`).
///
/// The shipped PAR6 sequence takes ~60 s and leaves the arm on the ready
/// pose its `move_to` steps command, so a re-seek satisfies neither
/// half of this: the return has to finish inside the test budget AND
/// leave the arm where `Robot.joints.home` says home is.
#[test]
fn home_on_a_referenced_arm_returns_to_the_park_pose_without_reseeking() {
    let rig = Rig::boot(test_config());
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    // Referenced (teleport applies the home references) but standing
    // away from the park pose on every joint the ready pose differs on.
    let park = park_deg();
    let ready = ready_pose_deg();
    let mut away = park;
    for (a, r) in away.iter_mut().zip(ready.iter()) {
        *a += 0.25 * (r - *a);
    }
    teleport_home(&rig, &mut c, away);

    let started = Instant::now();
    let index = c.ok_index(&Command::Home(par6_proto::command::Home { key: 7401 }));
    let (ok, detail) = c.wait_complete(index);
    let elapsed = started.elapsed();
    assert!(ok, "home on a referenced arm must complete, got {detail:?}");
    assert!(
        elapsed < Duration::from_secs(15),
        "an already-referenced home must not re-run the seek (took {elapsed:?})"
    );

    let s = rig.wait_status("the return move is reported complete", |s| {
        s.completed_index >= index as i64
    });
    // The post-profile hold settles the arm onto the return target, so
    // home must land ON the park pose — and nowhere near where a
    // referencing seek would have left it.
    assert!(
        max_deg_error(&s.angles, &park) < max_deg_error(&s.angles, &ready),
        "home must return to the park pose {park:?}, not the referencing \
         sequence's ready pose {ready:?}; got {:?}",
        s.angles
    );
    assert!(
        max_deg_error(&s.angles, &park) < 1.0,
        "home must settle on the park pose: from {away:?} it reached {:?} \
         (home is {park:?})",
        s.angles
    );
    // A referencing seek drops `homed` on its way through; a planned
    // return never does.
    assert!(s.homed, "the return move must not drop the home reference");

    rig.shutdown();
}

/// Gap 24: the queue ETA covers moves parameterised by SPEED, which is
/// how nearly every client queues them — `duration=` is the exception.
/// parol6 accumulates the planned duration of every buffered segment
/// (`server/segment_player.py:94,257`), so its ETA is real; a runtime
/// that only counted an explicit `duration=` reported 0 for a queue
/// full of work.
///
/// The ETA is PLANNED time, so the plan is what it is checked against:
/// the same travel at half the speed takes proportionally longer, a
/// second queued move adds its own time, and neither may exceed the
/// wall-clock the moves really take (which also carries the settle the
/// plan does not describe). Every move is RELATIVE, so each one covers
/// the same travel wherever the sim's tracking lag left the arm.
#[test]
fn queue_eta_counts_speed_parameterised_moves() {
    let rig = Rig::boot(test_config());
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);
    teleport_home(&rig, &mut c, park_deg());

    let sweep = |key: u64, deg: f64, speed: f64| {
        Command::MoveJ(MoveJ {
            key,
            angles: [deg, 0.0, 0.0, 0.0, 0.0, 0.0],
            duration: None,
            speed: Some(speed),
            accel: None,
            blend_radius: None,
            rel: true,
        })
    };

    // One move in flight: the ETA is that move's planned duration.
    let index = c.ok_index(&sweep(7411, 20.0, 0.10));
    let started = Instant::now();
    let fast = queued_duration(&mut c);
    let (ok, detail) = c.wait_complete(index);
    assert!(ok, "the move must complete, got {detail:?}");
    let wall = started.elapsed().as_secs_f64();
    assert!(
        fast > 0.5,
        "a 20 deg move at a tenth of full speed is real work, ETA said {fast}"
    );
    assert!(
        fast <= wall,
        "planned time ({fast:.2} s) cannot exceed the time the move really \
         took ({wall:.2} s)"
    );

    // Drained queue: nothing left to wait for.
    let idle = rig.wait_status("queue drains", |s| {
        s.queued_segments == 0 && s.executing_index == -1
    });
    assert!(
        idle.queued_duration < 0.5,
        "an empty queue has no ETA, got {}",
        idle.queued_duration
    );

    // The same travel at half the speed takes proportionally longer —
    // up to twice as long, less the ramps, which the acceleration limit
    // fixes independently of the speed fraction.
    let index = c.ok_index(&sweep(7412, -20.0, 0.05));
    let slow = queued_duration(&mut c);
    let ratio = slow / fast;
    assert!(
        (1.4..=2.05).contains(&ratio),
        "halving the speed must nearly double the ETA \
         ({fast:.2} s -> {slow:.2} s)"
    );
    assert!(c.wait_complete(index).0);

    // A second queued move counts too: the ETA covers the whole queue,
    // not just the motion in flight.
    let a = c.ok_index(&sweep(7413, 20.0, 0.05));
    let b = c.ok_index(&sweep(7414, -20.0, 0.05));
    let two = queued_duration(&mut c);
    let ratio = two / slow;
    assert!(
        (1.8..2.2).contains(&ratio),
        "a second identical move must double the ETA ({slow:.2} s -> {two:.2} s)"
    );
    assert!(c.wait_complete(a).0 && c.wait_complete(b).0);

    rig.shutdown();
}

/// The QUEUE query's `queued_duration`.
fn queued_duration(c: &mut Client) -> f64 {
    match c.query(&Command::Queue) {
        QueryResult::Queue {
            queued_duration, ..
        } => queued_duration,
        other => panic!("unexpected queue result {other:?}"),
    }
}

/// Gap 25: LOOP_STATS publishes ten numbers and every one of them has
/// to come from the loop. `std`, `min` and `p95` were hardcoded 0.0
/// while the same rolling window already produced p50/p90/p99/max —
/// three of ten metrics that could not be told apart from a stopped
/// loop.
#[test]
fn loop_stats_reports_the_whole_window_not_three_zeros() {
    let rig = Rig::boot(test_config());
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);

    // The percentiles are recomputed on a periodic tick boundary; poll
    // until the window has been summarised at least once.
    let deadline = Instant::now() + BUDGET;
    let stats = loop {
        match c.query(&Command::LoopStats) {
            QueryResult::LoopStats(ls) if ls.max_period_s > 0.0 => break ls,
            QueryResult::LoopStats(_) => {}
            other => panic!("unexpected loop_stats result {other:?}"),
        }
        assert!(
            Instant::now() < deadline,
            "the loop never published its window"
        );
        std::thread::sleep(Duration::from_millis(50));
    };

    assert!(
        stats.min_period_s > 0.0,
        "a running loop has a real fastest tick, got {stats:?}"
    );
    assert!(
        stats.p95_period_s > 0.0,
        "a running loop has a real p95, got {stats:?}"
    );
    assert!(
        stats.std_period_s > 0.0,
        "a wall-clock loop has real jitter, got {stats:?}"
    );
    assert!(
        stats.min_period_s <= stats.p95_period_s
            && stats.p95_period_s <= stats.p99_period_s
            && stats.p99_period_s <= stats.max_period_s,
        "the window statistics must be ordered: {stats:?}"
    );

    rig.shutdown();
}

/// Gap 26: `pose(frame="TRF")` is the WORLD expressed in the tool frame
/// — parol6 returns `inv(T_fk)` (`commands/query_commands.py:71-78`).
/// Answering the identity is definitionally true and carries no
/// information at all.
#[test]
fn pose_in_the_tool_frame_is_the_world_in_tool_transform() {
    let rig = Rig::boot(test_config());
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);
    // Off the park pose, so neither matrix is near-identity by accident.
    teleport_home(&rig, &mut c, with_j0(park_deg(), 30.0));

    let pose = |c: &mut Client, frame: Option<Frame>| -> [f64; 16] {
        match c.query(&Command::Pose(par6_proto::command::PoseQuery { frame })) {
            QueryResult::Pose { pose } => pose,
            other => panic!("unexpected pose result {other:?}"),
        }
    };
    let wrf = pose(&mut c, Some(Frame::Wrf));
    let trf = pose(&mut c, Some(Frame::Trf));

    assert!(
        wrf.iter().all(|v| v.is_finite()) && trf.iter().all(|v| v.is_finite()),
        "both frames must answer with a real pose: wrf {wrf:?} trf {trf:?}"
    );
    assert!(
        !is_identity(&trf),
        "the tool frame's answer must not be the identity: {trf:?}"
    );
    // The two describe the same transform in opposite directions, so
    // composing them (in metres — the wire carries mm translations)
    // must land on the identity.
    let product = mat_mul(&to_metres(&trf), &to_metres(&wrf));
    assert!(
        is_identity(&product),
        "TRF must be the inverse of WRF; their product is {product:?}"
    );

    rig.shutdown();
}

/// Row-major 4x4 with the translation converted mm → m.
fn to_metres(m: &[f64; 16]) -> [f64; 16] {
    let mut out = *m;
    for row in 0..3 {
        out[row * 4 + 3] /= 1000.0;
    }
    out
}

fn mat_mul(a: &[f64; 16], b: &[f64; 16]) -> [f64; 16] {
    let mut out = [0.0; 16];
    for r in 0..4 {
        for col in 0..4 {
            out[r * 4 + col] = (0..4).map(|k| a[r * 4 + k] * b[k * 4 + col]).sum();
        }
    }
    out
}

fn is_identity(m: &[f64; 16]) -> bool {
    (0..4).all(|r| {
        (0..4).all(|c| {
            let want = if r == c { 1.0 } else { 0.0 };
            (m[r * 4 + c] - want).abs() < 1e-6
        })
    })
}

/// The RT jog latch reaches the enablement flags.
/// A client streaming jog faster than the RT ticks must not be able to
/// build a command backlog that the release then queues behind.
///
/// The RT drains one command per tick. A UI holding a jog control sends the
/// same speed every frame, so a path that enqueues per datagram puts the
/// release at the tail of a queue hundreds deep: the arm keeps jogging for
/// queue-depth ticks after the operator let go. A repeated setpoint carries
/// no new instruction, so it must cost nothing.
#[test]
fn a_flood_of_identical_jog_setpoints_does_not_delay_the_release() {
    let rig = Rig::boot(test_config());
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);
    teleport_home(&rig, &mut c, park_deg());
    let start = park_deg()[0];

    // Hold the control: many datagrams, one instruction.
    for _ in 0..300 {
        c.send(&jog_j(0.5, 5.0));
    }
    rig.wait_status("the arm is jogging", |s| s.angles[0] > start + 1.0);

    // Let go, and count STATUS frames until the arm is at rest. Frames are
    // published once per tick, so this measures the backlog in the RT's own
    // units rather than in wall-clock time — the arm's ramp-down costs a
    // handful of ticks, while a per-datagram queue costs one tick per
    // datagram before the release is even read.
    let released_at = rig
        .wait_status("a status frame to anchor the release", |_| true)
        .seq;
    c.send(&jog_j(0.0, 5.0));
    let stopped = rig.wait_status("the jog ramps to rest after the release", |s| {
        s.speeds.iter().all(|v| v.abs() < 1e-3)
    });
    let ticks = stopped.seq.saturating_sub(released_at);
    assert!(
        ticks < 150,
        "the release waited {ticks} ticks behind a backlog of 300 datagrams"
    );

    rig.shutdown();
}

/// The jog engine blocks a direction at its jerk-aware brake-at-limits
/// bound, which at full jog speed latches with the joint still far from
/// the soft wall — while the planner's static margin (a fraction of a
/// degree) would keep reporting the direction free. A frontend greying
/// its jog buttons off `joint_en` must see the direction the RT actually
/// stopped honoring, not the wall the arm never got to.
#[test]
fn the_rt_jog_latch_greys_the_enablement_flag() {
    let rig = Rig::boot(test_config());
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);
    teleport_home(&rig, &mut c, park_deg());

    let cfg = par6_config::RobotConfig::load(&common::shipped_config()).expect("PAR6 config");
    let soft_max_deg = cfg.joints[0].limits.soft_max_rad.to_degrees();

    // Park J0 well inside its window but near enough the wall that a
    // full-speed jog's braking distance covers the gap: the engine
    // latches the positive direction while the joint is still tens of
    // degrees from the soft limit.
    let mut near = park_deg();
    near[0] = soft_max_deg - 25.0;
    teleport_home(&rig, &mut c, near);
    rig.wait_status("both directions free before the jog", |s| {
        (s.joint_en[0], s.joint_en[1]) == (1, 1)
    });

    c.send(&jog_j(1.0, 2.0));
    let s = rig.wait_status("the latched direction greys while far from the wall", |s| {
        s.joint_en[0] == 0
    });
    assert!(
        soft_max_deg - s.angles[0] > 2.0,
        "the latch must fire from the brake bound, not the static margin: \
         J0 at {:.2} deg with the soft wall at {soft_max_deg:.2} deg",
        s.angles[0]
    );
    assert_eq!(
        s.joint_en[1], 1,
        "only the latched direction greys; negative stays free"
    );

    // Jogging away releases the latch: the flag is live state, not a
    // one-way trip.
    c.send(&jog_j(-0.3, 1.0));
    rig.wait_status("the direction frees once the arm moves away", |s| {
        s.joint_en[0] == 1
    });

    rig.shutdown();
}
