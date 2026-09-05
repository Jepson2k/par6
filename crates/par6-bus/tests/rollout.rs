//! The preview rollout: the runtime plant with the arm pinned, the jaws
//! and the world objects live — a preview grasp jams on the same object,
//! at the same jaw byte, that the simulator's status path would report.

use std::path::PathBuf;

use par6_bus::sim::rollout::Rollout;
use par6_bus::sim::scene::{Scene, Tool};
use par6_config::{GripperConfig, RobotConfig};
use par6_proto::{Physical, Shape};

fn par6() -> RobotConfig {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
    RobotConfig::load(&path).expect("PAR6.toml")
}

fn msg_gripper() -> GripperConfig {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../config/grippers/MSG_small_motor_150mm_rail.toml");
    GripperConfig::load(&path).expect("MSG gripper TOML")
}

fn scene() -> Scene {
    Scene {
        tool: Tool::Msg,
        assets: PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/par6_description"),
    }
}

fn shape(name: &str, params: &[f64], pose: [f64; 6], mass: Option<f64>) -> Shape {
    Shape {
        kind: "box".into(),
        params: params.to_vec(),
        pose: pose.to_vec(),
        collision: true,
        margin: None,
        name: name.into(),
        physics: Some(Physical {
            mass,
            friction: [1.0, 0.005, 0.0001],
        }),
    }
}

/// Reach-down pose over the stand (config frame), and the stand and block
/// the vendor scene used to hard-code.
const GRASP_POSE: [f64; 6] = [0.0, -0.25, 4.35, 0.0, -1.28, 0.0];

fn grasp_world() -> Vec<Shape> {
    vec![
        shape(
            "stand",
            &[0.04, 0.04, 0.01],
            [0.3713, 0.0, 0.005, 0.0, 0.0, 0.0],
            None,
        ),
        shape(
            "block",
            &[0.036, 0.036, 0.06],
            [0.3713, 0.0, 0.04, 0.0, 0.0, 0.0],
            Some(0.05),
        ),
    ]
}

#[test]
fn a_rollout_grasps_releases_and_drops_what_the_simulator_would() {
    let robot = par6();
    let gripper = msg_gripper();
    let world = grasp_world();
    let mut roll = Rollout::new(
        &scene(),
        &robot,
        Some(&gripper),
        &[&[], &world],
        &GRASP_POSE,
    )
    .expect("rollout scene");
    assert_eq!(roll.object_names(), vec!["block".to_owned()]);
    assert_eq!(
        Rollout::free_object_names(&[&[], &world]),
        vec!["block".to_owned()]
    );
    let dt = roll.dt();
    let ticks = |s: f64| (s / dt).round() as usize;
    roll.place_jaw(20.0);
    // Let the scene ring down with the jaws open.
    for _ in 0..ticks(0.5) {
        roll.step(None);
    }
    let resting = roll.object_pose("block").expect("block");
    assert!(
        (resting[2] - 0.04).abs() < 0.005,
        "block rests on the stand, z {}",
        resting[2]
    );

    // Close: the jaws jam on the block mid-travel and the block stays put.
    let mut jam = None;
    for _ in 0..ticks(2.0) {
        roll.step(Some(Rollout::jaw_drive(252.0)));
        if let (Some(at), _) = roll.jaw_obstruction() {
            jam = Some(at);
            break;
        }
    }
    let jam = jam.expect("closing on the block must jam the jaws");
    assert!(
        (100..240).contains(&jam),
        "jam byte {jam} not in mid-travel"
    );
    let held = roll.object_pose("block").unwrap();
    assert!(
        (held[0] - resting[0]).abs() < 0.01 && (held[2] - resting[2]).abs() < 0.01,
        "the grasp knocked the block away: {resting:?} -> {held:?}"
    );

    // Open: free travel, no obstruction.
    for _ in 0..ticks(1.5) {
        roll.step(Some(Rollout::jaw_drive(20.0)));
    }
    assert_eq!(
        roll.jaw_obstruction(),
        (None, None),
        "opening away is free travel"
    );
    assert!(
        (roll.jaw_byte().unwrap() - 20.0).abs() < 2.0,
        "jaws opened to the target"
    );

    // A released object falls to the floor and settles.
    let mut pose = held;
    pose[0] = 0.2;
    pose[1] = 0.6;
    pose[2] = 0.3;
    assert!(roll.place_object("block", pose));
    for _ in 0..ticks(1.5) {
        roll.step(None);
    }
    let dropped = roll.object_pose("block").unwrap();
    assert!(
        (dropped[2] - 0.03).abs() < 0.005,
        "block rests on the floor, z {}",
        dropped[2]
    );
    assert!(roll.object_speed("block").unwrap() < 1e-3, "settled");

    // The arm is pinned where it is placed, drivers idle.
    roll.place_arm(&[0.0, -1.85, 2.85, 0.0, 0.0, 0.0]);
    for _ in 0..ticks(0.5) {
        roll.step(None);
    }
    // No API reads the arm back on purpose (the trajectory is the
    // planner's); the pin is exercised by the object staying put under it.
    assert!((roll.object_pose("block").unwrap()[2] - 0.03).abs() < 0.005);
}
