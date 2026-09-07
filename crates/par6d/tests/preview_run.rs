//! The simulated dry run: the daemon's own planner driving a real
//! control loop against the plant, ticked flat out instead of paced.
//!
//! The parity suite next door checks that the preview PLANS what the
//! runtime plans. These check the other half — that a run reports what
//! the arm DID, which a plan cannot: the servo lag, the sag, and a block
//! that rises because friction against the jaw pads carries it.

use std::path::PathBuf;

use par6_proto::command::{MoveJ, ToolAction};
use par6_proto::{Command, Shape, ToolParam, NUM_JOINTS};
use par6_server::ShapeLayer;
use par6d::preview::record::StopReason;
use par6d::preview::{Preview, RunLimits};

mod common;
use common::{park_deg, to_rad};

/// The shipped config re-ticked to 50 Hz, as the parity suite uses.
fn test_config() -> PathBuf {
    common::retimed_config("preview-run", 0.02)
}

fn assets() -> PathBuf {
    common::assets_dir()
}

fn move_j_cmd(angles_deg: [f64; NUM_JOINTS], key: u64, speed: f64) -> Command {
    Command::MoveJ(MoveJ {
        key,
        angles: angles_deg,
        duration: None,
        speed: Some(speed),
        accel: None,
        blend_radius: None,
        rel: false,
    })
}

/// The simulated run against the planner it replaces.
///
/// The planner says where the arm is *told* to go; the run says where it
/// *went*, because the same commands were queued to the same planner
/// driving a real control loop against the plant. The two must agree
/// inside the settle tolerance — that is what the `settled` completion
/// policy promises — while the run additionally shows the tracking error
/// the plan cannot: `q_commanded` is what went on the motor bus and `q`
/// is what the joints did with it.
#[test]
fn the_simulated_run_lands_where_the_plan_says_and_shows_the_tracking_error() {
    let config = test_config();
    let park = to_rad(&park_deg());
    let mut first = park_deg();
    first[0] += 20.0;
    let mut second = first;
    second[1] -= 15.0;
    let cmds = [move_j_cmd(first, 9101, 1.0), move_j_cmd(second, 9102, 1.0)];

    let mut planned_session =
        Preview::new(Some(&config), Some(&assets()), None).expect("preview boots");
    planned_session.teleport_rad(park);
    let planned: Vec<_> = cmds
        .iter()
        .cloned()
        .map(|c| planned_session.submit(c))
        .collect();
    assert!(planned.iter().all(|r| r.valid()), "{planned:?}");
    let planned_end = planned.last().expect("two results").end_joints_rad;

    let mut session = Preview::new(Some(&config), Some(&assets()), None).expect("preview boots");
    session.teleport_rad(park);
    let batch = session
        .run(&cmds, RunLimits::default())
        .expect("the run completes");

    assert_eq!(batch.stop, StopReason::Completed);
    assert_eq!(batch.commands.len(), cmds.len(), "one span per command");
    assert!(
        batch.commands.iter().all(|c| c.error.is_none()),
        "the run refused what the planner accepted: {:?}",
        batch.commands
    );
    assert!(batch.rows > 10, "a two-move program takes ticks");
    assert_eq!(batch.q_rad.len(), batch.rows * batch.joints);
    assert_eq!(batch.tcp.len(), batch.rows * 6);

    // Spans tile the record in order, so a consumer can map any row back
    // to the line of the program that produced it.
    let mut expected_start = 0;
    for span in &batch.commands {
        assert_eq!(
            span.start_row, expected_start,
            "command spans must tile the record: {:?}",
            batch.commands
        );
        expected_start += span.rows;
    }
    assert_eq!(expected_start, batch.rows, "spans must cover every row");

    // Where it ended up, against where the plan said.
    let last = (batch.rows - 1) * batch.joints;
    let achieved = &batch.q_rad[last..last + batch.joints];
    for (j, (got, want)) in achieved.iter().zip(&planned_end).enumerate() {
        assert!(
            (f64::from(*got) - want).abs() < 0.01,
            "joint {j} landed {:+.5} rad off the plan (settle tolerance is 0.01)",
            f64::from(*got) - want
        );
    }

    // The tracking error the plan cannot show. It must be real — a
    // record where the two columns are identical is not measuring a
    // servo, it is copying the command — and it must stay small.
    // Only over the rows that commanded a position: an idle arm holds
    // itself with a torque and no target, and reports NaN.
    let worst = (0..batch.q_rad.len())
        .filter(|i| batch.q_commanded_rad[*i].is_finite())
        .map(|i| (batch.q_rad[i] - batch.q_commanded_rad[i]).abs())
        .fold(0.0f32, f32::max);
    assert!(
        worst > 1e-5,
        "q and q_commanded are the same column: the plant is not being simulated"
    );
    assert!(
        worst < 0.05,
        "the arm is not following its commands: worst tracking error {worst} rad"
    );
}

/// A grasp, through the whole machine.
///
/// Nothing here is arranged: the jaws close because a tool action was
/// queued, they stop on the block because the contact solver says so, the
/// block rises because friction against the pads carries it, and it falls
/// when they open. There is no "carried" flag and no rigid transform
/// welding it to the TCP — both would be claims physics could contradict.
#[test]
fn a_run_grasps_lifts_and_drops_a_world_object() {
    let config = test_config();
    let mut session = Preview::new(Some(&config), Some(&assets()), None).expect("preview boots");
    // Reach-down pose over the stand (config frame), as in the bus tests.
    let grasp_pose = [0.0, -0.25, 4.35, 0.0, -1.28, 0.0];
    session.teleport_rad(grasp_pose);
    let shape = |name: &str, params: [f64; 3], z: f64, mass: Option<f64>| Shape {
        kind: "box".into(),
        params: params.to_vec(),
        pose: vec![0.3713, 0.0, z, 0.0, 0.0, 0.0],
        collision: true,
        margin: None,
        name: name.into(),
        physics: Some(par6_proto::Physical {
            mass,
            friction: [1.0, 0.005, 0.0001],
        }),
    };
    session
        .set_shapes(
            ShapeLayer::Program,
            &[
                shape("stand", [0.04, 0.04, 0.01], 0.005, None),
                shape("block", [0.036, 0.036, 0.06], 0.04, Some(0.05)),
            ],
        )
        .expect("world applied");

    let tool = par6_config::RobotConfig::load(&config)
        .expect("cfg")
        .robot
        .active_gripper;
    let tool_move = |key: u64, closed: f64| {
        Command::ToolAction(ToolAction {
            key,
            tool_key: tool.clone(),
            action: "move".into(),
            params: vec![
                ToolParam::Float(closed),
                ToolParam::Float(0.5),
                ToolParam::Float(500.0),
            ],
        })
    };
    // Close on the block, swing the shoulder back to raise the TCP, open.
    let mut lifted = grasp_pose;
    lifted[1] -= 0.3;
    let cmds = [
        tool_move(9001, 1.0),
        move_j_cmd(std::array::from_fn(|i| lifted[i].to_degrees()), 9002, 0.5),
        tool_move(9003, 0.0),
    ];
    let batch = session
        .run(&cmds, RunLimits::default())
        .expect("the run completes");
    assert!(
        batch.commands.iter().all(|c| c.error.is_none()),
        "the grasp program was refused: {:?}",
        batch.commands
    );

    let block = batch
        .objects
        .iter()
        .find(|t| t.name == "block")
        .expect("the block has a track");
    assert!(
        block.poses.len() > 1,
        "a block that gets picked up and dropped is not a still object"
    );
    assert!(
        !batch.objects.iter().any(|t| t.name == "stand"),
        "a massless shape is a fixture welded into the world, not a body \
         with a pose to track: {:?}",
        batch.objects.iter().map(|t| &t.name).collect::<Vec<_>>()
    );

    let z = |row: usize| f64::from(block.poses[row][2]);
    let close_end = batch.commands[0].start_row + batch.commands[0].rows;
    let lift_end = batch.commands[1].start_row + batch.commands[1].rows;
    let held = z(close_end - 1);
    let raised = z(lift_end - 1);
    let dropped = z(block.poses.len() - 1);
    assert!(
        (held - 0.04).abs() < 0.01,
        "the block stayed on its stand while the jaws closed: z {held}"
    );
    assert!(
        raised > held + 0.05,
        "friction against the closed jaws must carry the block up: {held} -> {raised}"
    );
    assert!(
        dropped < raised - 0.05,
        "opening the jaws must drop it: held at {raised}, ended at {dropped}"
    );

    // The jaws report the hold themselves, and the contact solver has
    // something to say for the whole time they do.
    let gripping = batch.commands[1].start_row..lift_end;
    assert!(
        gripping.clone().any(|r| batch.tool_gripping[r]),
        "the gripper never reported an object between its jaws"
    );
    assert!(
        gripping
            .clone()
            .any(|r| batch.contact_starts[r + 1] > batch.contact_starts[r]),
        "a block held between two pads has contacts"
    );
}
