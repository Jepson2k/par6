//! e2e: the Rust client against an in-process `par6d --sim` — real UDP,
//! real protocol, no fakes. Mirrors the workflows the Python suite drives
//! through the same daemon.

use std::net::UdpSocket;
use std::path::PathBuf;
use std::time::Duration;

use par6_client::{Ack, Client, ClientConfig, ClientError, Frame, StatusTransport, NUM_JOINTS};
use par6_proto::Shape;
use par6d::options::StatusTransport as DaemonStatusTransport;
use par6d::{Daemon, Options};

const BUDGET: Duration = Duration::from_secs(20);

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .unwrap()
}

/// The shipped config re-ticked to 50 Hz: loaded test machines without RT
/// scheduling miss 4 ms deadlines and would latch LOOP_CRITICAL mid-test;
/// every RT time constant derives from config seconds, so the wiring under
/// test is identical.
fn test_config() -> PathBuf {
    let src = repo_root().join("config/PAR6.toml");
    let dir = std::env::temp_dir().join(format!("par6-client-sim-{}", std::process::id()));
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

/// Point the bus-grant segments at a scratch directory, once per test
/// binary — a test rig must never claim the machine's real bus.
fn redirect_bus_grant() {
    static ONCE: std::sync::OnceLock<PathBuf> = std::sync::OnceLock::new();
    ONCE.get_or_init(|| {
        let dir = std::env::temp_dir().join(format!("par6-client-shm-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("scratch shm dir");
        std::env::set_var("PAR6_SHM_DIR", &dir);
        dir
    });
}

/// Boot an in-process sim daemon on ephemeral ports (the daemon owns its
/// own tokio runtime, so this must run OUTSIDE the client's) and return
/// the client config wired to it over unicast STATUS.
fn boot_daemon() -> (Daemon, ClientConfig) {
    let _ = env_logger::builder().is_test(true).try_init();
    redirect_bus_grant();
    // Probe a free port for the STATUS stream; the client binds it.
    let status_port = {
        let probe = UdpSocket::bind("127.0.0.1:0").expect("probe socket");
        probe.local_addr().unwrap().port()
    };
    let opts = Options {
        sim: true,
        config: Some(test_config()),
        assets: Some(repo_root().join("assets/par6_description")),
        command_port: Some(0),
        bind: Some("127.0.0.1".parse().unwrap()),
        status_host: Some("127.0.0.1".parse().unwrap()),
        status_port: Some(status_port),
        telemetry_port: Some(0),
        status_transport: Some(DaemonStatusTransport::Unicast),
        ..Options::default()
    };
    let daemon = Daemon::start(&opts).expect("daemon boots in sim mode");
    let cfg = ClientConfig {
        host: "127.0.0.1".into(),
        port: daemon.command_addr().port(),
        timeout: Duration::from_secs(1),
        retries: 2,
        status: StatusTransport::Unicast {
            host: "127.0.0.1".parse().unwrap(),
        },
        status_port,
        mtu: 1400,
    };
    (daemon, cfg)
}

/// Drive one async session against a fresh daemon on a private runtime.
fn run_session<Fut>(body: impl FnOnce(Client) -> Fut)
where
    Fut: std::future::Future<Output = ()>,
{
    let (daemon, cfg) = boot_daemon();
    let rt = tokio::runtime::Runtime::new().expect("client runtime");
    rt.block_on(async move {
        let client = Client::connect(cfg).await.expect("client connects");
        body(client.clone()).await;
        client.close();
    });
    drop(rt);
    daemon.shutdown();
}

fn park_deg() -> [f64; NUM_JOINTS] {
    let cfg =
        par6_config::RobotConfig::load(&repo_root().join("config/PAR6.toml")).expect("config");
    let mut a = [0.0; NUM_JOINTS];
    for (out, rad) in a.iter_mut().zip(cfg.robot.park_pose_rad.iter()) {
        *out = rad.to_degrees();
    }
    a
}

fn close_deg(a: &[f64; NUM_JOINTS], b: &[f64; NUM_JOINTS], tol: f64) -> bool {
    a.iter().zip(b.iter()).all(|(x, y)| (x - y).abs() < tol)
}

/// Sim-only fast homing: re-send `teleport` until the broadcast shows the
/// pose landed and the arm reads homed (the enable gate is reached
/// asynchronously by the RT clear sequence).
async fn settle_at(client: &Client, target: [f64; NUM_JOINTS]) {
    let deadline = tokio::time::Instant::now() + BUDGET;
    loop {
        client.teleport(target, None).await.expect("teleport sends");
        let landed = client
            .wait_status(
                move |s| s.homed && close_deg(&s.angles, &target, 1.0),
                Duration::from_millis(400),
            )
            .await;
        if landed {
            return;
        }
        assert!(
            tokio::time::Instant::now() < deadline,
            "teleport did not take effect within budget"
        );
    }
}

#[test]
fn a_full_session_over_the_rust_client() {
    run_session(|client| async move {
        assert!(client.wait_ready(Duration::from_secs(15)).await);
        assert!(client.is_simulator().await.expect("is_simulator"));

        let park = park_deg();
        settle_at(&client, park).await;
        assert!(client.error().await.expect("error query").is_none());

        // A queued joint move runs to completion and lands where it said.
        let mut target = park;
        target[0] += 10.0;
        let index = client
            .move_j(target, None, Some(1.0), None, None, false)
            .await
            .expect("move_j accepted")
            .expect("move_j acked with an index");
        assert!(client
            .wait_command(index, BUDGET)
            .await
            .expect("move completes"));
        let angles = client.angles().await.expect("angles");
        assert!(
            close_deg(&angles, &target, 1.5),
            "the move must land on its target: {angles:?} vs {target:?}"
        );

        // A jog stream drives the arm; stop brings it to rest.
        let before = client.angles().await.expect("angles")[0];
        for _ in 0..20 {
            client
                .jog_j([0.4, 0.0, 0.0, 0.0, 0.0, 0.0], 0.4, None)
                .await
                .expect("jog sends");
            tokio::time::sleep(Duration::from_millis(40)).await;
        }
        assert!(
            client
                .wait_status(move |s| s.angles[0] > before + 1.0, BUDGET)
                .await,
            "the jog stream never drove the arm"
        );
        client.stop(true).await.expect("stop");
        assert!(
            client
                .wait_status(|s| s.speeds.iter().all(|v| v.abs() < 0.05), BUDGET)
                .await,
            "the arm never came to rest after stop"
        );

        // A refused fire-and-forget surfaces through the standing error
        // (issue #23): out-of-range teleport, then acceptance clears it.
        let mut bad = park;
        bad[0] = 1.0e5;
        client.teleport(bad, None).await.expect("send succeeds");
        assert!(
            client.wait_status(|s| s.error.is_some(), BUDGET).await,
            "the refused teleport never latched a standing error"
        );
        settle_at(&client, park).await;
        assert!(
            client.wait_status(|s| s.error.is_none(), BUDGET).await,
            "acceptance must clear the refusal"
        );

        // Chunked transfer: a shape world too large for one datagram.
        let shapes: Vec<Shape> = (0..64)
            .map(|i| Shape {
                kind: "box".into(),
                params: vec![0.02, 0.02, 0.02],
                pose: vec![2.0 + (i as f64) * 0.05, 2.0, 2.0, 0.0, 0.0, 0.0],
                collision: true,
                margin: None,
                name: format!("far-box-{i}"),
                physics: None,
            })
            .collect();
        assert_eq!(
            client.set_shapes(shapes).await.expect("set_shapes"),
            Ack::Confirmed,
            "the chunked shape world must be acked"
        );

        // Queries round-trip.
        let stats = client.loop_stats().await.expect("loop_stats");
        assert!(stats.loop_count > 0, "the loop must be ticking: {stats:?}");
        assert!(!client.profile().await.expect("profile").is_empty());
    })
}

#[test]
fn a_refusal_is_a_structured_robot_error() {
    run_session(|client| async move {
        assert!(client.wait_ready(Duration::from_secs(15)).await);
        match client.select_profile("BOGUS").await {
            Err(ClientError::Robot(e)) => {
                assert!(
                    e.cause.contains("BOGUS") || !e.title.is_empty(),
                    "the refusal must be structured: {e:?}"
                );
            }
            other => panic!("an unknown profile must be refused, got {other:?}"),
        }
        // A pose query in an explicit frame decodes.
        let pose = client.pose(Frame::Wrf).await.expect("pose");
        assert!(pose.iter().all(|v| v.is_finite()));
    })
}
