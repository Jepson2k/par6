//! Gravity calibration end to end on the torque-level sim plant: the
//! routine rests the arm in a pose set through the real client, reads
//! the torques the plant holds each pose with, and a fit started from a
//! model that is wrong by half must predict the plant's torques at poses
//! it never rested in — the property a calibration on the real arm is
//! for, with the plant's own inertials as the truth.

use std::path::PathBuf;
use std::time::Duration;

use par6_client::{Client, ClientConfig, StatusTransport};
use par6_kin::gravity::{self, BodyParams};
use par6_kin::{Collision, GripperVariant, Kin, NQ};
use par6_proto::NUM_JOINTS;

mod common;
use common::{assets_dir, boot_for_client, free_udp_port, shipped_config};

const BUDGET: Duration = Duration::from_secs(30);

/// Every pose is approached from both sides by this much, so joint
/// friction cancels in the average (see `Protocol::approach_rad`).
const APPROACH_RAD: f64 = 0.05;

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
    let tool = gripper.map(|g| {
        let k = &g.kinematics;
        Kin::dh_tool_params(
            k.d_m,
            k.a_m,
            k.alpha_rad,
            k.mass_kg,
            k.com_m,
            k.inertia_kg_m2,
        )
    });
    let mut kin = Kin::load_arm(&assets_dir(), tool.as_ref()).expect("gravity model");
    let variant = GripperVariant::resolve(
        &robot.robot.active_gripper.to_ascii_uppercase(),
        gripper.and_then(|g| g.urdf_variant.as_deref()),
    );
    let mut collision = Collision::load(&assets_dir(), variant, 0.0).expect("collision world");
    let mut window = [(0.0, 0.0); NQ];
    for (w, j) in window.iter_mut().zip(robot.joints.iter()) {
        *w = (j.limits.soft_min_rad, j.limits.soft_max_rad);
    }
    let poses =
        par6_calibrate::plan_poses(&mut collision, &window, 12, 3, APPROACH_RAD).expect("poses");
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

    // A prior whose centres of mass are all two centimetres out. Masses
    // are not fitted and stay as the plant has them, which is the point:
    // statics pins where the mass sits, not how much there is.
    const SHIFT_M: f64 = 0.02;
    let truth = gravity::model_params(&kin).unwrap();
    let prior: Vec<BodyParams> = truth
        .iter()
        .map(|b| BodyParams {
            joint: b.joint.clone(),
            mass: b.mass,
            first_moment: [
                b.first_moment[0] + b.mass * SHIFT_M,
                b.first_moment[1] - b.mass * SHIFT_M,
                b.first_moment[2] + b.mass * SHIFT_M,
            ],
        })
        .collect();
    let report = par6_calibrate::evaluate(&mut kin, samples, 4, prior, 1e-3).expect("fit");
    println!("{}", par6_calibrate::describe(&report));
    assert!(
        report.holdout_rms_prior_nm > 0.05,
        "centres of mass {SHIFT_M} m out must miss the plant by more than {} Nm",
        report.holdout_rms_prior_nm
    );
    assert!(
        report.holdout_rms_fit_nm < 0.25 * report.holdout_rms_prior_nm,
        "the fit must beat the prior on unseen poses: {} vs {} Nm",
        report.holdout_rms_fit_nm,
        report.holdout_rms_prior_nm
    );

    // The measured axes must land near where the plant's mass actually
    // is, not merely produce the same torques: the prior put them 20 mm
    // out, and what comes back has to be much closer than that.
    let mut checked = 0;
    for ((got, want), axes) in report
        .fit
        .bodies
        .iter()
        .zip(&truth)
        .zip(&report.fit.determined)
    {
        for (axis, share) in axes.iter().enumerate() {
            if *share > 0.9 {
                checked += 1;
                let err = (got.com()[axis] - want.com()[axis]).abs();
                assert!(
                    err < 0.005,
                    "{} axis {axis}: measured centre of mass is {err:.4} m out, \
                     the prior was {SHIFT_M} m out",
                    want.joint
                );
            }
        }
    }
    assert!(checked > 0, "the run measured nothing to check");
}
