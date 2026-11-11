//! Contact calibration against a live `par6d --sim`, through the real
//! client: the thresholds it fits come from the arm's own motion, the
//! motion they were fitted on runs clean under them, something in the way
//! still stops the arm, and a run that fails leaves the session's contact
//! settings as it found them.

use std::sync::Mutex;
use std::time::Duration;

use par6_client::{Client, ClientConfig, StatusTransport};
use par6_proto::{ContactReaction, ErrorCode, NUM_JOINTS};
use par6d::calibrate::{self, ContactProtocol};

mod common;
use common::{assets_dir, boot_for_client_in, free_udp_port};

const BUDGET: Duration = Duration::from_secs(30);

/// Reaching down in front of the base, tool about 0.37 m out.
const REACH_RAD: [f64; NUM_JOINTS] = [0.0, -0.25, 4.35, 0.0, -1.28, 0.0];

/// One real-time daemon at a time: two of them share the cores a tick
/// needs.
static ONE_DAEMON: Mutex<()> = Mutex::new(());

async fn connect(cmd: std::net::SocketAddr, status_port: u16) -> Client {
    let client = Client::connect(ClientConfig {
        host: cmd.ip().to_string(),
        port: cmd.port(),
        status: StatusTransport::Unicast {
            host: "127.0.0.1".parse().unwrap(),
        },
        status_port,
        ..ClientConfig::default()
    })
    .await
    .expect("client connects");
    assert!(
        client.wait_status(|s| s.link_ok == 1, BUDGET).await,
        "the sim bus never came up"
    );
    client
}

/// Reset, then stand the arm at the reach pose, referenced.
async fn stand_at_reach(client: &Client) -> [f64; NUM_JOINTS] {
    let reach = common::to_deg(&REACH_RAD);
    client.reset().await.expect("reset");
    let deadline = tokio::time::Instant::now() + BUDGET;
    loop {
        let _ = client.teleport(reach, None).await;
        if client
            .wait_status(
                |s| s.homed && s.enabled && common::max_deg_error(&s.angles, &reach) < 0.5,
                Duration::from_millis(300),
            )
            .await
        {
            return reach;
        }
        assert!(
            tokio::time::Instant::now() < deadline,
            "the arm never stood at the reach pose"
        );
    }
}

/// Drive to `angles` and say whether the move completed.
async fn run_to(client: &Client, angles: [f64; NUM_JOINTS], speed: f64) -> bool {
    let index = client
        .move_j(angles, None, Some(speed), None, None, false)
        .await
        .expect("move_j is admitted")
        .expect("move_j is confirmed");
    matches!(client.wait_command(index, BUDGET).await, Ok(true))
}

#[test]
fn fitted_thresholds_pass_the_arms_own_motion_and_still_stop_for_a_post() {
    let _one = ONE_DAEMON.lock().unwrap_or_else(|e| e.into_inner());
    let config = common::retimed_config("contact-calibration", 0.004);
    // A post on the tool's circle, 30 degrees round from where it starts:
    // beyond the swings, in the way of a longer move.
    let scenario = config.with_file_name("post.toml");
    std::fs::write(
        &scenario,
        "[obstacle]\nsize_m = [0.06, 0.06, 0.4]\npose = [0.3204, 0.185, 0.2, 0.0, 0.0, 0.0]\n",
    )
    .expect("write the scenario");
    let model_config = config.clone();
    let status_port = free_udp_port();
    let daemon = boot_for_client_in(config, status_port, Some(scenario)).expect("daemon boots");
    let cmd = daemon.command_addr();
    let rt = tokio::runtime::Runtime::new().expect("runtime");

    rt.block_on(async {
        let client = connect(cmd, status_port).await;
        let reach = stand_at_reach(&client).await;
        let shipped = client.contact().await.expect("contact reads");
        assert_eq!(shipped.reaction, ContactReaction::Stop);
        let mut model =
            par6d::kin::estimation_model(Some(&model_config), Some(&assets_dir()), None, None)
                .expect("model");

        let protocol = ContactProtocol {
            spread: 0.2,
            ..ContactProtocol::default()
        };
        let report = calibrate::contact(&client, &mut model, &protocol, true)
            .await
            .expect("the calibration runs");

        assert!(report.applied && report.reaction == ContactReaction::Stop);
        assert!(
            report.swings >= NUM_JOINTS,
            "every joint is swung at least once: {} swings",
            report.swings
        );
        assert!(
            report.peak_signal.iter().any(|p| *p > 0.0)
                && report.peak_impact.iter().any(|p| *p > 0.0),
            "the swings must have been watched: {:?} / {:?}",
            report.peak_signal,
            report.peak_impact
        );
        assert_eq!(
            report.sustained_nm,
            calibrate::thresholds_from(&report.peak_signal, protocol.margin)
        );
        assert_eq!(
            report.impact_nm,
            calibrate::thresholds_from(&report.peak_impact, protocol.margin)
        );
        for j in 0..NUM_JOINTS {
            assert!(
                report.sustained_nm[j] >= protocol.margin * report.peak_signal[j]
                    && report.sustained_nm[j] >= calibrate::CONTACT_FLOOR_NM,
                "J{}: a {:.3} Nm threshold over a {:.3} Nm peak",
                j + 1,
                report.sustained_nm[j],
                report.peak_signal[j]
            );
        }
        assert_ne!(
            report.sustained_nm, shipped.sustained_nm,
            "the thresholds are this arm's now, not the reference arm's"
        );
        let fitted = client.contact().await.expect("contact reads");
        assert_eq!(
            fitted.reaction,
            ContactReaction::Stop,
            "the reaction is back"
        );
        assert_eq!(fitted.sustained_nm, report.sustained_nm);
        assert_eq!(fitted.impact_nm, report.impact_nm);
        let ended = client.angles().await.expect("angles");
        assert!(
            common::max_deg_error(&ended, &reach) < 0.5,
            "the arm is back where it stood: {ended:?}"
        );

        // The motion the thresholds were fitted on runs clean under them.
        for swing in [0.2_f64, -0.2] {
            let mut to = reach;
            to[0] += swing.to_degrees();
            assert!(
                run_to(&client, to, 1.0).await && run_to(&client, reach, 1.0).await,
                "a swing inside the calibrated range was stopped: {:?}",
                client.error().await
            );
        }
        let clean = client.contact().await.expect("contact reads");
        assert_eq!(clean.trips, 0, "free motion must not read as a contact");

        // And what is in the way still stops the arm.
        let mut through = reach;
        through[0] = 60.0;
        assert!(
            !run_to(&client, through, 0.5).await,
            "a move through the post must not complete"
        );
        let standing = client
            .error()
            .await
            .expect("error reads")
            .expect("the stop stands");
        assert_eq!(standing.code, ErrorCode::SysContact as u16);
        assert_eq!(client.contact().await.expect("contact reads").trips, 1);
        client.close_joined().await;
    });
    daemon
        .shutdown()
        .expect("the daemon's threads exit cleanly");
}

/// Keep-outs the program declared stand either side of the arm, and the
/// calibration's own model has not heard of them: the runtime refuses the
/// base both ways, the run ends naming the joint, and the session's
/// reaction and thresholds are as they were — not left reporting-only.
#[test]
fn a_calibration_the_runtime_leaves_no_swing_for_puts_the_contact_settings_back() {
    let _one = ONE_DAEMON.lock().unwrap_or_else(|e| e.into_inner());
    let config = common::retimed_config("contact-calibration-restore", 0.004);
    let model_config = config.clone();
    let status_port = free_udp_port();
    let daemon = boot_for_client_in(config, status_port, None).expect("daemon boots");
    let cmd = daemon.command_addr();
    let rt = tokio::runtime::Runtime::new().expect("runtime");

    rt.block_on(async {
        let client = connect(cmd, status_port).await;
        let reach = stand_at_reach(&client).await;
        let before = client.contact().await.expect("contact reads");
        assert_eq!(before.reaction, ContactReaction::Stop);
        let post = |name: &str, y: f64| par6_proto::Shape {
            kind: "box".into(),
            params: vec![0.06, 0.06, 0.4],
            pose: vec![0.3204, y, 0.2, 0.0, 0.0, 0.0],
            collision: true,
            margin: None,
            name: name.into(),
            physics: None,
            attachment: None,
        };
        client
            .set_shapes(vec![post("left", 0.185), post("right", -0.185)])
            .await
            .expect("the keep-outs are accepted");
        let mut model =
            par6d::kin::estimation_model(Some(&model_config), Some(&assets_dir()), None, None)
                .expect("model");

        let protocol = ContactProtocol {
            spread: 0.6,
            ..ContactProtocol::default()
        };
        let error = calibrate::contact(&client, &mut model, &protocol, true)
            .await
            .expect_err("a joint the runtime will not swing must end the run");
        assert!(
            error.contains("joint 1 has no room"),
            "the joint is named: {error}"
        );

        let after = client.contact().await.expect("contact reads");
        assert_eq!(
            after.reaction,
            ContactReaction::Stop,
            "a failed calibration must not leave contacts reported only"
        );
        assert_eq!(after.sustained_nm, before.sustained_nm);
        assert_eq!(after.impact_nm, before.impact_nm);
        let ended = client.angles().await.expect("angles");
        assert!(
            common::max_deg_error(&ended, &reach) < 0.5,
            "a refused swing never starts: the arm is where it stood, {ended:?}"
        );
        client.close_joined().await;
    });
    daemon
        .shutdown()
        .expect("the daemon's threads exit cleanly");
}

/// A joint with no room to swing either way is said, by name, before
/// anything moves; with room one way that way is used; and no swing
/// leaves the joint window.
#[test]
fn swings_stay_inside_the_window_and_a_joint_without_room_is_named() {
    let config = common::shipped_config();
    let mut model = par6d::kin::estimation_model(Some(&config), Some(&assets_dir()), None, None)
        .expect("model");
    let window = model.window;
    let start = REACH_RAD;

    let swings =
        calibrate::plan_swings(&mut model.collision, &start, &window, 0.2).expect("swings");
    for j in 0..NUM_JOINTS {
        assert!(
            swings.iter().any(|q| (q[j] - start[j]).abs() > 0.19),
            "J{} is never swung",
            j + 1
        );
    }
    for q in &swings {
        for j in 0..NUM_JOINTS {
            assert!(
                q[j] >= window[j].0 && q[j] <= window[j].1,
                "J{} leaves its window at {:.3} rad",
                j + 1,
                q[j]
            );
        }
    }

    // Against its upper limit the base can only swing down.
    let mut cornered = start;
    cornered[0] = window[0].1 - 0.05;
    let one_way = calibrate::plan_swings(&mut model.collision, &cornered, &window, 0.2)
        .expect("one way is enough");
    assert!(one_way.iter().all(|q| q[0] <= cornered[0] + 1e-12));
    assert!(one_way.iter().any(|q| q[0] < cornered[0] - 0.19));

    // No joint window is ten radians wide.
    let error = calibrate::plan_swings(&mut model.collision, &start, &window, 10.0)
        .expect_err("a spread no joint has room for");
    assert!(error.contains("joint 1"), "the joint is named: {error}");
}
