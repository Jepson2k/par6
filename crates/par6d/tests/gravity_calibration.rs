//! Payload identification end to end on the torque-level sim plant.
//!
//! The daemon runs with its fitted gripper, so the plant physically
//! swings that gripper's mass. The identification model is loaded with
//! NO tool, so the gripper is exactly what a payload is: a load at the
//! end of the chain the model has never heard of. Swinging the wrist
//! through the real client has to recover its mass from the torques the
//! plant holds — the property an identification on the real arm is for,
//! with the gripper's own configured mass as the truth.

use std::path::PathBuf;
use std::time::Duration;

use par6_client::{Client, ClientConfig, StatusTransport};
use par6_kin::gravity;
use par6_kin::{Collision, GripperVariant, Kin, NQ};
use par6_proto::NUM_JOINTS;

mod common;
use common::{assets_dir, boot_for_client, free_udp_port, shipped_config};

const BUDGET: Duration = Duration::from_secs(30);

/// Every pose is approached from both sides by this much, so joint
/// friction cancels in the average (see `Protocol::approach_rad`).
const APPROACH_RAD: f64 = 0.05;

/// How far each wrist joint swings either way, giving the load a
/// different lever arm at each pose.
const SPREAD_RAD: f64 = 0.5;

/// The shipped config, at its real tick rate.
///
/// Every other daemon test re-ticks to 50 Hz so a loaded CI box can hold
/// the deadline, but the torque plant cannot be slowed down like that and
/// still be measured: at 20 ms per bus tick the drivers' 1 kHz loops are
/// integrated so coarsely that the joints limit-cycle, and the mean
/// current a pose is held with is chatter and friction rather than
/// gravity. Measured that way the wrist reads about a newton-metre where
/// gravity is zero, which is the whole quantity under test.
fn test_config() -> PathBuf {
    shipped_config()
}

#[test]
fn a_fit_from_the_plants_held_torques_predicts_poses_it_never_rested_in() {
    let config = test_config();
    let bundle = par6_config::ConfigBundle::load(&config).expect("config");
    let robot = &bundle.robot;
    let gripper = bundle.active_gripper();
    // No tool: the gripper the plant swings is the unknown load.
    let mut kin = Kin::load_arm(&assets_dir(), None).expect("gravity model");
    let carried_kg = gripper
        .map(|g| g.kinematics.mass_kg)
        .expect("a fitted gripper");
    assert!(
        carried_kg > 0.05,
        "the fitted gripper must have mass to find"
    );
    let variant = GripperVariant::resolve(
        &robot.robot.active_gripper.to_ascii_uppercase(),
        gripper.and_then(|g| g.urdf_variant.as_deref()),
    );
    let mut collision = Collision::load(&assets_dir(), variant, 0.0).expect("collision world");
    let mut window = [(0.0, 0.0); NQ];
    for (w, j) in window.iter_mut().zip(robot.joints.iter()) {
        *w = (j.limits.soft_min_rad, j.limits.soft_max_rad);
    }
    // The park pose is where the arm stands when a program would ask;
    // the wrist swings from there and nothing below it moves.
    let mut start = [0.0; NQ];
    for (out, rad) in start.iter_mut().zip(robot.robot.park_pose_rad.iter()) {
        *out = *rad;
    }
    let poses =
        par6_calibrate::plan_poses(&mut collision, &start, &window, SPREAD_RAD, APPROACH_RAD)
            .expect("poses");
    let mut park = [0.0; NUM_JOINTS];
    for (out, rad) in park.iter_mut().zip(robot.robot.park_pose_rad.iter()) {
        *out = rad.to_degrees();
    }

    let status_port = free_udp_port();
    let (daemon, _telemetry) =
        boot_for_client(config, true, status_port).expect("daemon boots on the torque plant");
    let cmd = daemon.command_addr();
    let rt = tokio::runtime::Runtime::new().expect("runtime");
    let samples = rt.block_on(async {
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
        client.reset().await.expect("reset");
        // Teleport is streamable-class: keep sending until the arm reads
        // referenced at the park pose (the boot enable can still be settling).
        let deadline = tokio::time::Instant::now() + BUDGET;
        loop {
            let _ = client.teleport(park, None).await;
            if client
                .wait_status(|s| s.homed && s.angles[1] < 0.0, Duration::from_millis(300))
                .await
            {
                break;
            }
            assert!(
                tokio::time::Instant::now() < deadline,
                "teleport never took"
            );
        }

        let protocol = par6_calibrate::Protocol {
            speed: 1.0,
            approach_rad: APPROACH_RAD,
            settle: Duration::from_millis(200),
            frames: 25,
            ..par6_calibrate::Protocol::default()
        };
        let samples = par6_calibrate::measure(&client, &poses, &protocol)
            .await
            .expect("every pose measured");
        client.close_joined().await;
        samples
    });
    daemon.shutdown();

    let fit = gravity::fit_payload(&mut kin, &samples, 1e-4).expect("fit");
    println!(
        "carried {carried_kg:.4} kg, identified {:.4} kg at com {:?}\n\
         residual {:.4} Nm, against {:.4} Nm with an empty model\n\
         determined {:?}",
        fit.mass, fit.com, fit.rms_nm, fit.rms_unloaded_nm, fit.determined
    );

    assert!(
        fit.rms_unloaded_nm > 0.05,
        "a model that does not know about the gripper must miss the plant by more \
         than {} Nm, or there is nothing to identify",
        fit.rms_unloaded_nm
    );
    assert!(
        fit.rms_nm < 0.5 * fit.rms_unloaded_nm,
        "identifying the load must explain the torque better than ignoring it: \
         {} vs {} Nm",
        fit.rms_nm,
        fit.rms_unloaded_nm
    );
    // Mass is frame-independent, so it is the honest thing to compare
    // against the gripper's configured value. Measured off a torque
    // plant through the real wire, so this is a per-cent-scale check,
    // not an exact one.
    assert!(
        (fit.mass - carried_kg).abs() < 0.15 * carried_kg,
        "identified {:.4} kg against the {carried_kg:.4} kg the plant swings",
        fit.mass
    );
    assert!(
        fit.determined[0] > par6_calibrate::MEASURED,
        "swinging the wrist must measure the mass, determined {:?}",
        fit.determined
    );
}
