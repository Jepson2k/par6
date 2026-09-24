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

/// Park with the wrist turned off its singularity: at park J5 sits at
/// zero, where the pose's roll/pitch/yaw form degenerates and a target
/// built by round-tripping through it carries a rotation nobody asked for.
fn clear_of_the_wrist_deg() -> [f64; NUM_JOINTS] {
    let mut q = common::park_deg();
    q[3] += 25.0;
    q[4] += 35.0;
    q
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
        // Refused until the boot enable lands; the loop re-sends.
        let _ = client.teleport(target, None).await;
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

/// A COMPLETE push that never arrives does not leave the wait guessing:
/// STATUS shows the command finished, and the runtime is asked what it
/// finished as — a landing as much as a cancellation.
#[test]
fn a_missing_complete_push_is_recovered_from_the_runtime() {
    run_session("recover", |client| async move {
        assert!(client.wait_ready(Duration::from_secs(15)).await);
        let park = common::park_deg();
        settle_at(&client, park).await;
        client.drop_complete_pushes_for_test(true);

        let mut target = park;
        target[0] += 8.0;
        let landed = client
            .move_j(target, None, Some(1.0), None, None, false)
            .await
            .expect("move_j accepted")
            .expect("move_j acked with an index");
        assert!(
            client
                .wait_command(landed, BUDGET)
                .await
                .expect("recovered"),
            "the landing must be recovered without its push"
        );
        assert_eq!(
            client.command_completion(landed).await.expect("query"),
            (true, true, None, None)
        );

        let mut far = park;
        far[0] -= 30.0;
        let cancelled = client
            .move_j(far, Some(6.0), None, None, None, false)
            .await
            .expect("move_j accepted")
            .expect("move_j acked with an index");
        assert!(
            client
                .wait_status(|s| s.speeds.iter().any(|v| v.abs() > 0.02), BUDGET)
                .await,
            "the long move never started"
        );
        client.stop(true).await.expect("stop");
        match client.wait_command(cancelled, BUDGET).await {
            Err(ClientError::Robot(e)) => {
                assert_eq!(e.code, ErrorCode::MotnCancelled as u16, "{e:?}")
            }
            other => panic!("the cancellation must be recovered without its push: {other:?}"),
        }
        client.drop_complete_pushes_for_test(false);
    });
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

        // A teleport is acked: an out-of-range one is refused in the
        // reply, and the arm stays where the stop left it.
        let resting = client.angles().await.expect("angles");
        let mut bad = park;
        bad[0] = 1.0e5;
        let refused = client
            .teleport(bad, None)
            .await
            .expect_err("an out-of-range teleport is refused");
        assert!(
            matches!(&refused, ClientError::Robot(e) if e.code == ErrorCode::CommValidationError as u16),
            "{refused:?}"
        );
        assert!(
            client
                .wait_status(move |s| close_deg(&s.angles, &resting, 0.5), BUDGET)
                .await,
            "the refused teleport moved the arm"
        );
        settle_at(&client, park).await;

        // Chunked transfer: a shape world too large for one datagram.
        let shapes: Vec<Shape> = (0..64)
            .map(|i| Shape {
                attachment: None,
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
        let before = client.latest_status().expect("received status");
        assert_ne!(before.session_id, 0);
        assert!(
            client
                .wait_status(
                    |next| {
                        next.session_id == before.session_id
                            && next.seq > before.seq
                            && next.mono_time_ns > before.mono_time_ns
                    },
                    BUDGET,
                )
                .await
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
        attachment: None,
        kind: "box".to_owned(),
        params: vec![0.6, 0.4, 0.02],
        pose: vec![0.9, 0.9, -0.01, 0.0, 0.0, 0.0],
        collision: true,
        margin: None,
        physics: None,
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

/// A keep-out sphere centred on the TCP the arm would reach at `there`,
/// so a servo toward `there` is refused by the collision gate.
async fn keep_out_at(
    client: &Client,
    park: [f64; NUM_JOINTS],
    there: [f64; NUM_JOINTS],
) -> (Shape, [f64; 3]) {
    settle_at(client, there).await;
    let pose = client.pose(Frame::Wrf).await.expect("pose at the target");
    settle_at(client, park).await;
    // Status poses are in mm; shapes are declared in metres.
    let centre = [pose[3] / 1000.0, pose[7] / 1000.0, pose[11] / 1000.0];
    let shape = Shape {
        name: "wall".into(),
        kind: "sphere".into(),
        params: vec![0.06],
        pose: vec![centre[0], centre[1], centre[2], 0.0, 0.0, 0.0],
        collision: true,
        margin: None,
        physics: None,
        attachment: None,
    };
    (shape, centre)
}

/// A servo target refused while the arm is at rest is refused and nothing
/// more: the standoff exists to shed momentum, and an arm with none must
/// not be driven toward the keep-out it was just refused.
#[test]
fn a_servo_target_refused_at_rest_leaves_the_arm_where_it_is() {
    run_session("servo-rest", |client| async move {
        assert!(client.wait_ready(Duration::from_secs(15)).await);
        let park = common::park_deg();
        let mut there = park;
        there[0] += 12.0;
        let (wall, _) = keep_out_at(&client, park, there).await;
        client
            .set_shapes(vec![wall])
            .await
            .expect("keep-out applies");
        let rest = client.angles().await.expect("angles at rest");
        client
            .servo_j(there, Some(0.2), None)
            .await
            .expect("fire-and-forget sends");
        // Longer than the standoff's whole travel budget.
        let moved = client
            .wait_status(
                move |s| !close_deg(&s.angles, &rest, 0.3),
                Duration::from_millis(3500),
            )
            .await;
        assert!(!moved, "a refused target at rest moved the arm");
        client.set_shapes(vec![]).await.expect("keep-out clears");
    })
}

/// The wire's [x, y, z, roll, pitch, yaw] (mm / degrees) for a status pose
/// matrix — the inverse of `par6_proto::pose_matrix`.
fn wire_pose(m: &[f64; 16]) -> [f64; 6] {
    let pitch = m[2].clamp(-1.0, 1.0).asin();
    let roll = (-m[6]).atan2(m[10]);
    let yaw = (-m[1]).atan2(m[0]);
    [
        m[3],
        m[7],
        m[11],
        roll.to_degrees(),
        pitch.to_degrees(),
        yaw.to_degrees(),
    ]
}

/// A Cartesian servo stream that is already moving when its next setpoint
/// is refused is braked and placed short of the keep-out, like the joint
/// servo stream, rather than dropped into IDLE with its momentum — which
/// coasts it through the keep-out the refusal was about.
#[test]
fn a_moving_cartesian_servo_stream_stops_outside_the_keep_out() {
    run_session("servo-l-brake", |client| async move {
        assert!(client.wait_ready(Duration::from_secs(15)).await);
        let park = common::park_deg();
        let mut there = park;
        there[0] += 25.0;
        settle_at(&client, there).await;
        let end = wire_pose(&client.pose(Frame::Wrf).await.expect("pose at the target"));
        let (wall, centre) = keep_out_at(&client, park, there).await;
        let start = wire_pose(&client.pose(Frame::Wrf).await.expect("pose at park"));
        client
            .set_shapes(vec![wall])
            .await
            .expect("keep-out applies");
        for step in 1..=40u32 {
            let t = f64::from(step) / 40.0;
            let mut pose = [0.0; 6];
            for (i, p) in pose.iter_mut().enumerate() {
                *p = start[i] + (end[i] - start[i]) * t;
            }
            client
                .servo_l(pose, Some(1.0), Some(1.0))
                .await
                .expect("fire-and-forget sends");
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        let stopped = client
            .wait_status(
                |s| s.speeds.iter().all(|v| v.abs() < 0.01),
                Duration::from_secs(5),
            )
            .await;
        assert!(stopped, "the refused stream never came to rest");
        tokio::time::sleep(Duration::from_millis(500)).await;
        let pose = client.pose(Frame::Wrf).await.expect("pose");
        let tcp = [pose[3] / 1000.0, pose[7] / 1000.0, pose[11] / 1000.0];
        let distance = ((tcp[0] - centre[0]).powi(2)
            + (tcp[1] - centre[1]).powi(2)
            + (tcp[2] - centre[2]).powi(2))
        .sqrt();
        assert!(
            distance > 0.06,
            "the arm coasted into the keep-out ({:.1} mm from its centre)",
            distance * 1000.0
        );
        client.set_shapes(vec![]).await.expect("keep-out clears");
    })
}

/// Perpendicular distance of `p` from the line `a`→`b`, in the wire's
/// millimetres (translation components only).
fn off_line(a: &[f64; 6], b: &[f64; 6], p: &[f64; 6]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    let u = [d[0] / len, d[1] / len, d[2] / len];
    let r = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let along = r[0] * u[0] + r[1] * u[1] + r[2] * u[2];
    let perp = [
        r[0] - along * u[0],
        r[1] - along * u[1],
        r[2] - along * u[2],
    ];
    (perp[0] * perp[0] + perp[1] * perp[1] + perp[2] * perp[2]).sqrt()
}

/// `servo_l` means the TOOL travels the straight line to the target.
///
/// That is its whole difference from `servo_j_pose`, which also ends in
/// the right place — by interpolating in JOINT space, which bows the
/// tool off the line on the way. Limiting in joint space does the same
/// thing, so this pins down both: a `servo_l` aliased onto
/// `servo_j_pose`, and one whose cartesian profile is re-planned by the
/// joint limiter downstream.
///
/// The residual is the drives following the command, not the command
/// bending: it scales with the speed fraction (about 1 mm here, ~8 mm
/// at full speed and acceleration), while the joint-interpolated path
/// leaves the line by tens of millimetres whatever the speed.
#[test]
fn servo_l_holds_the_line_where_servo_j_pose_does_not() {
    run_session("servo-l-line", |client| async move {
        assert!(client.wait_ready(Duration::from_secs(15)).await);

        // Off the wrist singularity: at park the tool points straight
        // down, pitch sits at -90 deg and the [x, y, z, r, p, y] wire
        // form degenerates — roll and yaw stop being separable, so a
        // target built by round-tripping through it carries a rotation
        // nobody asked for and the move is a screw, not a line.
        let from = clear_of_the_wrist_deg();

        // A diagonal in all three axes: a move along one axis alone
        // cannot tell a straight path from a bowed one.
        let offsets = [60.0, -45.0, 30.0];

        let mut worst = [0.0f64; 2];
        for (mode, out) in worst.iter_mut().enumerate() {
            settle_at(&client, from).await;
            let start = wire_pose(&client.pose(Frame::Wrf).await.expect("pose at start"));
            let mut target = start;
            for (axis, d) in offsets.iter().enumerate() {
                target[axis] += d;
            }

            let mut samples = 0u32;
            let mut arrived = false;
            for _ in 0..250 {
                if mode == 0 {
                    client.servo_l(target, Some(0.3), Some(0.3)).await
                } else {
                    client.servo_j_pose(target, Some(0.3), Some(0.3)).await
                }
                .expect("fire-and-forget sends");
                tokio::time::sleep(Duration::from_millis(20)).await;
                let here = wire_pose(&client.pose(Frame::Wrf).await.expect("pose"));
                let travelled = ((here[0] - start[0]).powi(2)
                    + (here[1] - start[1]).powi(2)
                    + (here[2] - start[2]).powi(2))
                .sqrt();
                let remaining = ((target[0] - here[0]).powi(2)
                    + (target[1] - here[1]).powi(2)
                    + (target[2] - here[2]).powi(2))
                .sqrt();
                // Judge only the part of the path actually under way: at
                // the very start every point is trivially on the line.
                if travelled > 2.0 {
                    *out = out.max(off_line(&start, &target, &here));
                    samples += 1;
                }
                if remaining < 1.0 {
                    arrived = true;
                    break;
                }
            }
            assert!(arrived, "mode {mode} never reached its target");
            assert!(
                samples > 20,
                "mode {mode}: only {samples} samples along the path"
            );
        }
        let (cartesian, joint) = (worst[0], worst[1]);
        println!("off the line — servo_l {cartesian:.2} mm, servo_j_pose {joint:.2} mm");
        assert!(
            cartesian < 2.0,
            "servo_l left the line by {cartesian:.2} mm"
        );
        assert!(
            joint > 5.0 * cartesian,
            "servo_j_pose ({joint:.2} mm) tracked the line as well as servo_l \
             ({cartesian:.2} mm) — servo_l is not running the cartesian limiter"
        );
    })
}

/// `jog_l` is a TCP velocity command, so the tool travels along the axis
/// it was given and accelerates onto it rather than having the twist
/// applied whole.
///
/// The axis is what pins this down: smoothing the twist in joint space
/// shapes each joint's own ramp, and the tool wanders off the commanded
/// direction while they are out of step with each other.
///
/// The residual is the drives following the command rather than the
/// command leaving the axis — it scales with the commanded rate, about
/// 1 mm at these fractions and 2.5 mm at double them. The executor's own
/// output holds the axis to 1e-9 m (`par6-motion`, `cart_stream`).
#[test]
fn jog_l_drives_the_tool_along_the_axis_it_was_given() {
    run_session("jog-l-axis", |client| async move {
        assert!(client.wait_ready(Duration::from_secs(15)).await);
        // Off the wrist singularity, as the servo_l line test is.
        let from = clear_of_the_wrist_deg();
        settle_at(&client, from).await;
        let start = wire_pose(&client.pose(Frame::Wrf).await.expect("pose at start"));

        // A diagonal in world axes: a single-axis jog cannot tell a
        // straight travel from a wandering one. The fractions set the
        // rate; the unit vector they point along is what the travel is
        // judged against.
        let fractions = [0.3f64, -0.4, 0.0];
        let norm = (fractions[0] * fractions[0]
            + fractions[1] * fractions[1]
            + fractions[2] * fractions[2])
            .sqrt();
        let axis = [
            fractions[0] / norm,
            fractions[1] / norm,
            fractions[2] / norm,
        ];
        let mut worst_off = 0.0f64;
        let mut samples = 0u32;
        let mut travelled = 0.0f64;
        for _ in 0..60 {
            client
                .jog_l(
                    [fractions[0], fractions[1], fractions[2], 0.0, 0.0, 0.0],
                    0.3,
                    Frame::Wrf,
                    Some(1.0),
                )
                .await
                .expect("fire-and-forget sends");
            tokio::time::sleep(Duration::from_millis(20)).await;
            let here = wire_pose(&client.pose(Frame::Wrf).await.expect("pose"));
            let rel = [here[0] - start[0], here[1] - start[1], here[2] - start[2]];
            travelled = (rel[0] * rel[0] + rel[1] * rel[1] + rel[2] * rel[2]).sqrt();
            if travelled > 2.0 {
                let along = rel[0] * axis[0] + rel[1] * axis[1] + rel[2] * axis[2];
                let perp = [
                    rel[0] - along * axis[0],
                    rel[1] - along * axis[1],
                    rel[2] - along * axis[2],
                ];
                worst_off = worst_off
                    .max((perp[0] * perp[0] + perp[1] * perp[1] + perp[2] * perp[2]).sqrt());
                samples += 1;
            }
        }
        assert!(travelled > 10.0, "the jog only moved {travelled:.2} mm");
        assert!(samples > 10, "only {samples} samples along the travel");
        println!(
            "jog_l off the commanded axis: {worst_off:.2} mm over {travelled:.1} mm travelled"
        );
        assert!(
            worst_off < 2.0,
            "the tool wandered {worst_off:.2} mm off the axis it was jogged along"
        );
    })
}

fn distance_mm(a: &[f64; 6], b: &[f64; 6]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// Poll the WRF pose until it holds still for ten readings in a row.
async fn pose_at_rest(client: &Client) -> [f64; 6] {
    let deadline = tokio::time::Instant::now() + Duration::from_secs(8);
    let mut last = wire_pose(&client.pose(Frame::Wrf).await.expect("pose"));
    let mut still = 0;
    while still < 10 {
        assert!(
            tokio::time::Instant::now() < deadline,
            "the tool never came to rest"
        );
        tokio::time::sleep(Duration::from_millis(50)).await;
        let here = wire_pose(&client.pose(Frame::Wrf).await.expect("pose"));
        still = if distance_mm(&here, &last) < 0.05 {
            still + 1
        } else {
            0
        };
        last = here;
    }
    last
}

/// A STATUS in another protocol version that still decodes — a newer
/// daemon's frame, or a second daemon on a shared group — does not stop
/// the waits: the client warns and goes on reading the runtime it talks
/// to. Only a frame that cannot be read at all latches the mismatch.
#[test]
fn a_readable_status_in_another_version_does_not_stop_the_waits() {
    let (daemon, cfg) = boot_daemon("skew", Ipv4Addr::LOCALHOST);
    let status_port = cfg.status_port;
    run_with(daemon, cfg, |client| async move {
        assert!(client.wait_ready(Duration::from_secs(15)).await);
        assert!(
            client.wait_status(|_| true, BUDGET).await,
            "no STATUS arrived"
        );
        let mut foreign = (*client.latest_status().expect("status")).clone();
        foreign.proto_version = foreign.proto_version.wrapping_add(1);
        let mut frame = Vec::new();
        par6_proto::encode_status_into(&foreign, &mut frame);
        let sender = UdpSocket::bind("127.0.0.1:0").expect("sender socket");
        sender
            .send_to(&frame, ("127.0.0.1", status_port))
            .expect("the foreign frame is sent");
        tokio::time::sleep(Duration::from_millis(100)).await;

        assert_eq!(client.protocol_mismatch(), None);
        let seen = client.latest_status().expect("status").seq;
        assert!(
            client.wait_status(|s| s.seq > seen, BUDGET).await,
            "one readable foreign frame stopped every wait"
        );
    });
}

/// A runtime that restarts while a command is awaited numbers its queue
/// from the start again, so the index waited on names nothing: the wait
/// says the session changed, every time, never a plain `false` that reads
/// as a timeout. Several restarts, because which of the two signals the
/// client sees first is a race it must win whichever way it goes.
#[test]
fn a_restart_mid_wait_is_reported_as_the_session_changing() {
    let (first, cfg) = boot_daemon("restart", Ipv4Addr::LOCALHOST);
    let status_port = cfg.status_port;
    let command_port = cfg.port;
    let rt = tokio::runtime::Runtime::new().expect("client runtime");
    let client = rt.block_on(Client::connect(cfg)).expect("client connects");
    let mut daemon = first;
    for round in 0..5 {
        let index = rt.block_on(async {
            assert!(client.wait_ready(Duration::from_secs(15)).await);
            let park = common::park_deg();
            settle_at(&client, park).await;
            let mut far = park;
            far[0] -= 30.0;
            client
                .move_j(far, Some(6.0), None, None, None, false)
                .await
                .expect("move_j accepted")
                .expect("move_j acked with an index")
        });
        let waiting = {
            let client = client.clone();
            rt.spawn(async move { client.wait_command(index, BUDGET).await })
        };
        daemon.shutdown();
        let mut opts =
            common::sim_options(common::retimed_config("client-restart", 0.02), status_port);
        opts.command_port = Some(command_port);
        daemon = Daemon::start(&opts).expect("the daemon restarts");
        match rt.block_on(waiting).expect("the wait ran") {
            Err(ClientError::SessionChanged { index: i }) => assert_eq!(i, index),
            other => panic!("round {round}: a restart mid-wait must read as one: {other:?}"),
        }
    }
    client.close();
    drop(rt);
    daemon.shutdown();
}

/// A `servo_l` stream that goes silent brakes the tool ALONG its line and
/// stops it well short of the pose it was last sent. The brake owns the
/// limiter until a new target arrives, so the tool never drives on toward
/// the goal of a stream nobody is sending.
#[test]
fn a_servo_l_stream_that_goes_silent_brakes_short_of_its_target() {
    run_session("servo-l-silence", |client| async move {
        assert!(client.wait_ready(Duration::from_secs(15)).await);
        let from = clear_of_the_wrist_deg();
        settle_at(&client, from).await;
        let start = wire_pose(&client.pose(Frame::Wrf).await.expect("pose at start"));
        let mut target = start;
        for (axis, d) in [60.0, -45.0, 30.0].iter().enumerate() {
            target[axis] += d;
        }
        let length = distance_mm(&start, &target);

        // Under way, then silent.
        for _ in 0..15 {
            client
                .servo_l(target, Some(0.6), Some(1.0))
                .await
                .expect("fire-and-forget sends");
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        let rest = pose_at_rest(&client).await;
        let remaining = distance_mm(&rest, &target);
        assert!(
            remaining > 0.4 * length,
            "the silent stream drove on toward its target: {remaining:.1} mm left of {length:.1}"
        );
        assert!(
            distance_mm(&rest, &start) > 2.0,
            "the stream never got under way"
        );
        assert!(
            off_line(&start, &target, &rest) < 2.0,
            "the brake left the line by {:.2} mm",
            off_line(&start, &target, &rest)
        );
    })
}

/// A servo target nothing can reach is REFUSED — a standing error the
/// client can read — never dropped as if it had been accepted. An
/// accepted datagram wipes the standing error and the collision verdict,
/// so a dropped target would read as a stream running normally while the
/// arm sat still.
#[test]
fn an_unreachable_servo_target_is_refused_not_dropped() {
    run_session("servo-unreachable", |client| async move {
        assert!(client.wait_ready(Duration::from_secs(15)).await);
        let park = common::park_deg();
        settle_at(&client, park).await;
        let rest = client.angles().await.expect("angles at rest");
        // Two metres out: past any reach of this arm.
        client
            .servo_j_pose([2000.0, 0.0, 300.0, 180.0, 0.0, 180.0], Some(0.2), None)
            .await
            .expect("fire-and-forget sends");
        let deadline = tokio::time::Instant::now() + Duration::from_secs(3);
        let error = loop {
            if let Some(e) = client.error().await.expect("error query") {
                break e;
            }
            assert!(
                tokio::time::Instant::now() < deadline,
                "an unreachable servo target left no standing error"
            );
            tokio::time::sleep(Duration::from_millis(20)).await;
        };
        assert_eq!(
            error.code,
            par6_proto::ErrorCode::IkTargetUnreachable as u16
        );
        let moved = client
            .wait_status(
                move |s| !close_deg(&s.angles, &rest, 0.3),
                Duration::from_millis(1000),
            )
            .await;
        assert!(!moved, "the arm moved on a refused target");
    })
}

/// The fastest joint's speed \[rad/s\] as the runtime measures it, averaged
/// over the last `SMOOTH` readings. The runtime's own velocities need no
/// wall-clock timing; the average rides over the one tick a housekeeping
/// pass that wakes late leaves without a fresh setpoint.
struct JointSpeedTrace {
    recent: std::collections::VecDeque<f64>,
}

impl JointSpeedTrace {
    const SMOOTH: usize = 6;

    fn new() -> Self {
        Self {
            recent: std::collections::VecDeque::with_capacity(Self::SMOOTH + 1),
        }
    }

    async fn sample(&mut self, client: &Client) -> f64 {
        let fastest = client
            .joint_speeds()
            .await
            .expect("joint speeds")
            .iter()
            .fold(0.0_f64, |m, v| m.max(v.abs()));
        self.recent.push_back(fastest);
        if self.recent.len() > Self::SMOOTH {
            self.recent.pop_front();
        }
        self.recent.iter().sum::<f64>() / self.recent.len() as f64
    }
}

/// `stop()` during a `jog_l` brakes the tool ALONG the axis it was
/// travelling and then holds it, as a stop does for every other motion:
/// the cartesian limiter ramps it down, instead of the arm stopping dead
/// from jog speed wherever the stream was cut.
#[test]
fn a_stop_mid_jog_l_brakes_along_the_axis_and_then_holds() {
    run_session("jog-l-stop", |client| async move {
        assert!(client.wait_ready(Duration::from_secs(15)).await);
        let from = clear_of_the_wrist_deg();
        settle_at(&client, from).await;
        let start = wire_pose(&client.pose(Frame::Wrf).await.expect("pose at start"));
        // A quarter of the acceleration: a ramp of a few tenths of a
        // second, long against the 20 ms between samples.
        let press = || client.jog_l([0.3, -0.4, 0.0, 0.0, 0.0, 0.0], 0.3, Frame::Wrf, Some(0.25));
        let mut trace = JointSpeedTrace::new();
        let mut cruise = 0.0;
        for _ in 0..50 {
            press().await.expect("fire-and-forget sends");
            tokio::time::sleep(Duration::from_millis(20)).await;
            cruise = trace.sample(&client).await;
        }
        assert!(
            cruise > 0.01,
            "the jog never got up to speed: {cruise:.4} rad/s"
        );
        let at_stop = wire_pose(&client.pose(Frame::Wrf).await.expect("pose at the stop"));

        client.stop(true).await.expect("the stop is acked");
        // Stopped dead, the plant is at rest well inside 100 ms of the
        // stop; braked at a quarter of the acceleration it is still
        // travelling at most of its cruise speed at 150 ms.
        tokio::time::sleep(Duration::from_millis(150)).await;
        let braking = client
            .joint_speeds()
            .await
            .expect("joint speeds")
            .iter()
            .fold(0.0_f64, |m, v| m.max(v.abs()));
        assert!(
            braking > 0.3 * cruise,
            "the stop cut the jog dead: {braking:.4} rad/s 150 ms in, against a \
             {cruise:.4} rad/s cruise"
        );

        let rest = pose_at_rest(&client).await;
        assert!(
            distance_mm(&at_stop, &rest) > 1.0,
            "the brake covered no ground past the stop"
        );
        let off = off_line(&start, &at_stop, &rest);
        assert!(off < 2.0, "the brake left the jog axis by {off:.2} mm");
    })
}

/// A `jog_l` pressed again while the ramp from the previous press is still
/// running resumes from the speed the tool still carries, as parol6's
/// does.
#[test]
fn a_jog_l_pressed_again_mid_ramp_carries_on_without_stopping() {
    run_session("jog-l-repress", |client| async move {
        assert!(client.wait_ready(Duration::from_secs(15)).await);
        // The pose and diagonal the axis test jogs along: clear of the
        // wrist singularity, and short enough that the joint speeds a
        // given tool speed needs stay put.
        let from = clear_of_the_wrist_deg();
        settle_at(&client, from).await;
        // A quarter of the rates: a ramp long enough to press into, with
        // ticks to spare either side of the moment the ramp begins.
        let press = || {
            client.jog_l(
                [0.15, -0.2, 0.0, 0.0, 0.0, 0.0],
                0.1,
                Frame::Wrf,
                Some(0.25),
            )
        };
        let mut trace = JointSpeedTrace::new();

        let mut cruise = 0.0;
        let mut last_press = tokio::time::Instant::now();
        for _ in 0..50 {
            press().await.expect("fire-and-forget sends");
            last_press = tokio::time::Instant::now();
            tokio::time::sleep(Duration::from_millis(20)).await;
            cruise = trace.sample(&client).await;
        }
        assert!(
            cruise > 0.01,
            "the jog never got up to speed: {cruise:.4} rad/s"
        );
        // The ramp down starts 0.1 s (the press's duration) after the last
        // press lands and runs ~0.36 s at this acceleration: re-press a
        // quarter of the way into it.
        tokio::time::sleep_until(last_press + Duration::from_millis(200)).await;
        let mut slowest = f64::MAX;
        for _ in 0..25 {
            press().await.expect("fire-and-forget sends");
            tokio::time::sleep(Duration::from_millis(20)).await;
            slowest = slowest.min(trace.sample(&client).await);
        }
        assert!(
            slowest > 0.3 * cruise,
            "the re-press stopped the arm: {slowest:.4} rad/s against a {cruise:.4} rad/s cruise"
        );
    })
}
