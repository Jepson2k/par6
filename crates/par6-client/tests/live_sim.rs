//! e2e: the Rust client against an in-process `par6d --sim` — real UDP,
//! real protocol, no fakes. Mirrors the workflows the Python suite drives
//! through the same daemon, plus the transport invariants only this
//! client can prove: reply correlation, the idempotent re-ack, the
//! COMPLETE contract and the STATUS fallback.

use std::net::{IpAddr, Ipv4Addr, UdpSocket};
use std::time::Duration;

use par6_client::{
    Ack, Client, ClientConfig, ClientError, Frame, StatusTransport, MIN_MTU, NUM_JOINTS,
};
use par6_proto::command as cmd;
use par6_proto::{Command, ErrorCode, Shape};
use par6d::Daemon;

#[path = "../../par6d/tests/common/mod.rs"]
mod common;

const BUDGET: Duration = Duration::from_secs(20);

/// A free loopback UDP port for the STATUS stream; the client binds it.
fn free_port() -> u16 {
    UdpSocket::bind("127.0.0.1:0")
        .expect("probe socket")
        .local_addr()
        .unwrap()
        .port()
}

/// Boot an in-process sim daemon (it owns its own tokio runtime, so this
/// runs OUTSIDE the client's) on a config tree private to `tag`, STATUS
/// unicast to `status_host`; the client config is wired to it.
fn boot_daemon(tag: &str, status_host: Ipv4Addr) -> (Daemon, ClientConfig) {
    let _ = env_logger::builder().is_test(true).try_init();
    common::redirect_bus_grant();
    let status_port = free_port();
    let config = common::retimed_config(&format!("client-{tag}"), 0.02);
    let mut opts = common::sim_options(config, status_port);
    opts.status_host = Some(IpAddr::V4(status_host));
    let daemon = Daemon::start(&opts).expect("daemon boots in sim mode");
    let cfg = ClientConfig {
        host: "127.0.0.1".into(),
        port: daemon.command_addr().port(),
        timeout: Duration::from_secs(1),
        retries: 2,
        status: StatusTransport::Unicast { host: status_host },
        status_port,
        mtu: 1400,
    };
    (daemon, cfg)
}

/// Drive one async session against `daemon` on a private runtime.
fn run_with<Fut>(daemon: Daemon, cfg: ClientConfig, body: impl FnOnce(Client) -> Fut)
where
    Fut: std::future::Future<Output = ()>,
{
    let rt = tokio::runtime::Runtime::new().expect("client runtime");
    rt.block_on(async move {
        let client = Client::connect(cfg).await.expect("client connects");
        body(client.clone()).await;
        client.close();
    });
    drop(rt);
    daemon.shutdown();
}

/// A fresh daemon for `tag`, and one session against it.
fn run_session<Fut>(tag: &str, body: impl FnOnce(Client) -> Fut)
where
    Fut: std::future::Future<Output = ()>,
{
    let (daemon, cfg) = boot_daemon(tag, Ipv4Addr::LOCALHOST);
    run_with(daemon, cfg, body);
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
    run_session("session", |client| async move {
        assert!(client.wait_ready(Duration::from_secs(15)).await);
        assert!(client.is_simulator().await.expect("is_simulator"));

        let park = common::park_deg();
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
    run_session("refusal", |client| async move {
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

/// Replies are matched to their request by the echoed req_id, never by
/// arrival order: six different queries in flight on one socket at once
/// each get their own typed answer, round after round. A reply landing
/// on the wrong waiter would decode as the wrong result variant and
/// surface as `Unreachable`.
#[test]
fn replies_correlate_by_request_id_under_concurrent_queries() {
    run_session("correlate", |client| async move {
        assert!(client.wait_ready(Duration::from_secs(15)).await);
        for round in 0..25 {
            let (angles, profile, sim, stats, pose, ping) = tokio::join!(
                client.angles(),
                client.profile(),
                client.is_simulator(),
                client.loop_stats(),
                client.pose(Frame::Wrf),
                client.ping(),
            );
            let angles = angles.unwrap_or_else(|e| panic!("round {round}: angles: {e}"));
            assert!(angles.iter().all(|v| v.is_finite()));
            let profile = profile.unwrap_or_else(|e| panic!("round {round}: profile: {e}"));
            assert!(!profile.is_empty());
            assert!(sim.unwrap_or_else(|e| panic!("round {round}: is_simulator: {e}")));
            let stats = stats.unwrap_or_else(|e| panic!("round {round}: loop_stats: {e}"));
            assert!(stats.target_hz > 0.0);
            let pose = pose.unwrap_or_else(|e| panic!("round {round}: pose: {e}"));
            assert!(pose.iter().all(|v| v.is_finite()));
            ping.unwrap_or_else(|e| panic!("round {round}: ping: {e}"));
        }
    })
}

/// The idempotency contract: a QUEUED command sent again under the same
/// key (what the client's retry does when an ack is lost) is re-acked
/// with its ORIGINAL index and not queued twice — the next fresh command
/// takes the very next index.
#[test]
fn a_retransmitted_queued_command_is_re_acked_with_its_original_index() {
    run_session("dedup", |client| async move {
        assert!(client.wait_ready(Duration::from_secs(15)).await);
        let park = common::park_deg();
        settle_at(&client, park).await;

        let mut target = park;
        target[0] += 5.0;
        let keyed = Command::MoveJ(cmd::MoveJ {
            key: client.fresh_key(),
            angles: target,
            duration: None,
            speed: Some(0.5),
            accel: None,
            blend_radius: None,
            rel: false,
        });
        let index = client
            .queued(keyed.clone())
            .await
            .expect("accepted")
            .expect("acked");
        let again = client
            .queued(keyed)
            .await
            .expect("accepted")
            .expect("acked");
        assert_eq!(again, index, "the retransmit re-acks the original index");
        assert!(client.wait_command(index, BUDGET).await.expect("completes"));

        let next = client
            .move_j(park, None, Some(0.5), None, None, false)
            .await
            .expect("accepted")
            .expect("acked");
        assert_eq!(
            next,
            index + 1,
            "a fresh key is the next command; the retransmit took no slot"
        );
        assert!(client.wait_command(next, BUDGET).await.expect("completes"));
    })
}

/// A move cancelled mid-flight completes in error: `wait_command`
/// surfaces the runtime's MOTN_CANCELLED as a structured refusal (never
/// `Ok(true)`), and there is no settle verdict to read off it.
#[test]
fn a_cancelled_move_completes_in_error_with_no_verdict() {
    run_session("cancel", |client| async move {
        assert!(client.wait_ready(Duration::from_secs(15)).await);
        let park = common::park_deg();
        settle_at(&client, park).await;

        let mut far = park;
        far[0] += 60.0;
        let index = client
            .move_j(far, Some(6.0), None, None, None, false)
            .await
            .expect("accepted")
            .expect("acked");
        assert!(
            client
                .wait_status(
                    move |s| s.executing_index == index as i64
                        || s.speeds.iter().any(|v| v.abs() > 0.01),
                    BUDGET
                )
                .await,
            "the move must start before it is stopped"
        );
        client.stop(true).await.expect("stop");
        match client.wait_command(index, BUDGET).await {
            Err(ClientError::Robot(e)) => {
                assert_eq!(e.code, ErrorCode::MotnCancelled as u16, "{e:?}")
            }
            other => panic!("a cancelled move must complete in error, got {other:?}"),
        }
        assert_eq!(client.command_verdict(index), None);
    })
}

/// When no interface can join the multicast group, the STATUS
/// subscription falls back to unicast on the CONFIGURED fallback host —
/// the one the daemon is told to send to — not to localhost.
#[test]
fn the_status_stream_falls_back_to_the_configured_unicast_host() {
    let host = Ipv4Addr::new(127, 0, 0, 2);
    let (daemon, mut cfg) = boot_daemon("fallback", host);
    cfg.status = StatusTransport::Multicast {
        // A unicast address is not a group any interface can join, so
        // every rung of the multicast ladder fails and the fallback runs.
        group: Ipv4Addr::LOCALHOST,
        iface: Ipv4Addr::LOCALHOST,
        fallback: host,
    };
    run_with(daemon, cfg, |client| async move {
        assert!(client.wait_ready(Duration::from_secs(15)).await);
        assert!(
            client.wait_status(|_| true, BUDGET).await,
            "STATUS must reach the fallback socket bound on the configured host"
        );
    });
}

/// An MTU too small to carry a chunk envelope is refused at connect,
/// never wrapped into an oversized datagram.
#[test]
fn a_too_small_mtu_is_refused_at_connect() {
    let rt = tokio::runtime::Runtime::new().expect("client runtime");
    for mtu in [0, 1, MIN_MTU - 1] {
        let cfg = ClientConfig {
            mtu,
            ..ClientConfig::default()
        };
        match rt.block_on(Client::connect(cfg)) {
            Err(ClientError::Invalid(msg)) => assert!(msg.contains("mtu"), "{msg}"),
            Err(other) => panic!("mtu {mtu} must be refused as invalid, got {other}"),
            Ok(_) => panic!("mtu {mtu} must be refused, but the client connected"),
        }
    }
}

/// A program keep-out on the wire, metres/radians.
fn program_box(name: &str) -> Shape {
    Shape {
        kind: "box".to_owned(),
        params: vec![0.6, 0.4, 0.02],
        pose: vec![0.9, 0.9, -0.01, 0.0, 0.0, 0.0],
        collision: true,
        margin: None,
        name: name.to_owned(),
    }
}

async fn program_layer(client: &Client) -> Vec<Shape> {
    match client.query(Command::Shapes).await.expect("SHAPES answers") {
        par6_proto::QueryResult::Shapes { program, .. } => program,
        other => panic!("expected SHAPES, got {other:?}"),
    }
}

/// `set_shapes` answers one of three ways and never a fake success:
/// confirmed when the runtime applied the world, a structured refusal
/// when it would not, and UNCONFIRMED when nothing answered at all.
///
/// The third is the one a client gets wrong: a send with no reply is
/// not "applied", and a program that took it for one would run against
/// a world its keep-outs never reached.
#[test]
fn set_shapes_is_confirmed_refused_or_unconfirmed_never_a_fake_success() {
    let (daemon, cfg) = boot_daemon("shapes-ack", Ipv4Addr::LOCALHOST);
    // Nothing listens here: a port the kernel just handed out and released.
    let mut dead = cfg.clone();
    dead.port = free_port();
    dead.status_port = free_port();
    dead.timeout = Duration::from_millis(300);
    dead.retries = 0;

    run_with(daemon, cfg, |client| async move {
        let table = program_box("table");
        let ack = client
            .system(Command::SetShapes(cmd::SetShapes {
                shapes: vec![table.clone()],
            }))
            .await
            .expect("a valid world is not refused");
        assert_eq!(ack, Ack::Confirmed);
        assert_eq!(program_layer(&client).await, vec![table.clone()]);

        // Refused: two shapes with one name. The applied world survives it.
        match client
            .system(Command::SetShapes(cmd::SetShapes {
                shapes: vec![table.clone(), program_box("table")],
            }))
            .await
        {
            Err(ClientError::Robot(e)) => assert_eq!(
                e.code,
                ErrorCode::CommValidationError as u16,
                "a duplicate name is a validation refusal: {e:?}"
            ),
            other => panic!("a duplicate name must be refused, got {other:?}"),
        }
        assert_eq!(
            program_layer(&client).await,
            vec![table.clone()],
            "a refused set must leave the applied world standing"
        );

        // Unreachable: no reply is no confirmation, and the readback is
        // unreachable too rather than an empty world.
        let silent = Client::connect(dead).await.expect("sockets bind");
        let ack = silent
            .system(Command::SetShapes(cmd::SetShapes {
                shapes: vec![table.clone()],
            }))
            .await
            .expect("no reply is not an error, it is an unconfirmed send");
        assert_eq!(ack, Ack::Unconfirmed);
        assert!(
            matches!(
                silent.query(Command::Shapes).await,
                Err(ClientError::Unreachable)
            ),
            "a readback from nowhere must say so, not answer with an empty world"
        );
        silent.close();
    });
}
