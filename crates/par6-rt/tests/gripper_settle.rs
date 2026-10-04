//! When a jaw action is done, decided from the firmware's reply stream:
//! commands go through the core, replies arrive the way the drain hands
//! them over, and the verdict is read where the planner reads it. Every
//! window comes from the fitted tool's `[driver.settle]` seconds.

mod common;

use common::{bundle, Rig};
use par6_bus::{FirmwareGripperCommand, ObjectDetection};
use par6_rt::gripper_settle::{ToolSettle, ToolWait};
use par6_rt::RtCommand;

struct Windows {
    grace: u32,
    debounce: u32,
    move_timeout: u32,
}

fn windows(rig: &Rig) -> Windows {
    let b = bundle();
    let s = b
        .active_tool()
        .and_then(|t| t.driver.as_ref())
        .expect("the shipped tool is driven")
        .settle;
    let ticks = |secs: f64| (secs / rig.dt).round() as u32;
    Windows {
        grace: ticks(s.command_grace_s),
        debounce: ticks(s.detect_debounce_s),
        move_timeout: ticks(s.move_timeout_s),
    }
}

/// A rig with the jaws at `at`, a close to 200 just commanded.
fn closing_from(at: u8) -> Rig {
    let mut rig = Rig::new();
    rig.ready();
    rig.gripper_reply.position = at;
    rig.gripper_reply.action_status = true;
    rig.gripper_reply.object_detection = ObjectDetection::ReachedNoObject;
    rig.tick();
    rig.cmd(RtCommand::Gripper(FirmwareGripperCommand {
        position: 200,
        speed: 40,
        current_ma: 400,
        activate: true,
        action: true,
        estop: false,
        release_dir: false,
    }));
    rig
}

fn feed(rig: &mut Rig, n: u32, detection: ObjectDetection) -> ToolSettle {
    rig.gripper_reply.object_detection = detection;
    for _ in 0..n {
        rig.tick();
    }
    rig.snap().tool.verdict
}

/// A close settles on a SUSTAINED contact in the commanded direction, and
/// on nothing else: not the latched code left by the previous action,
/// not contact on the far side, not a code that chatters at the contact
/// threshold. It settles on exactly the debounce window's last tick.
#[test]
fn a_grasp_settles_only_on_sustained_contact_in_the_commanded_direction() {
    let mut rig = closing_from(100);
    let w = windows(&rig);
    assert!(w.debounce > 1 && w.grace > 1, "the windows span ticks");
    use ObjectDetection::*;

    // The previous action's "at position" latch, inside the grace.
    assert_eq!(
        feed(&mut rig, w.grace - 2, ReachedNoObject),
        ToolSettle::Running
    );
    assert_eq!(feed(&mut rig, 2, Moving), ToolSettle::Running);

    // Contact on the opening side says nothing about a close.
    assert_eq!(
        feed(&mut rig, 4 * w.debounce, DetectedOpening),
        ToolSettle::Running
    );
    // A run of closing contact broken by the far side starts over.
    feed(&mut rig, w.debounce - 1, DetectedClosing);
    feed(&mut rig, 1, DetectedOpening);
    assert_eq!(feed(&mut rig, 1, DetectedClosing), ToolSettle::Running);
    feed(&mut rig, 1, Moving);
    // Chatter: never a full window in a row.
    for _ in 0..10 {
        assert_eq!(
            feed(&mut rig, w.debounce - 1, DetectedClosing),
            ToolSettle::Running
        );
        assert_eq!(feed(&mut rig, 1, Moving), ToolSettle::Running);
    }

    assert_eq!(
        feed(&mut rig, w.debounce - 1, DetectedClosing),
        ToolSettle::Running
    );
    assert_eq!(
        feed(&mut rig, 1, DetectedClosing),
        ToolSettle::Settled(DetectedClosing),
        "a held object is a completed grasp"
    );
}

/// A stop holds the jaws where they are, so the standing action stays
/// asserted; the stop is done when the jaws stop travelling.
#[test]
fn a_stop_completes_when_travel_ends_with_the_action_still_asserted() {
    let mut rig = closing_from(100);
    let w = windows(&rig);
    feed(&mut rig, w.grace + 2, ObjectDetection::Moving);
    rig.cmd(RtCommand::GripperStop);
    assert!(rig.gripper_reply.action_status, "the hold keeps the action");
    assert_eq!(
        feed(&mut rig, w.grace + 2, ObjectDetection::Moving),
        ToolSettle::Running
    );
    assert_eq!(
        feed(&mut rig, 1, ObjectDetection::ReachedNoObject),
        ToolSettle::Done,
        "the jaws stopped travelling, which is all a stop promises"
    );
}

/// A move with no verdict fails on its own window rather than hanging the
/// queue.
#[test]
fn a_move_that_never_arrives_times_out_on_its_window() {
    let mut rig = closing_from(100);
    let w = windows(&rig);
    assert_eq!(
        feed(&mut rig, w.move_timeout - 2, ObjectDetection::Moving),
        ToolSettle::Running
    );
    assert_eq!(
        feed(&mut rig, 3, ObjectDetection::Moving),
        ToolSettle::Timeout(ToolWait::Move)
    );
}

/// A fault outranks arrival, whether the gripper reports it in its flags
/// or only on the frame's live error bit.
#[test]
fn a_fault_outranks_arrival_from_the_flags_or_the_live_error_bit() {
    let mut rig = closing_from(100);
    let w = windows(&rig);
    rig.gripper_reply.temperature_error = true;
    assert_eq!(
        feed(
            &mut rig,
            w.grace + w.debounce,
            ObjectDetection::ReachedNoObject
        ),
        ToolSettle::Fault(0b0001)
    );

    let mut rig = closing_from(100);
    rig.fault_nodes = 1 << rig.gripper_node;
    assert_eq!(
        feed(
            &mut rig,
            w.grace + w.debounce,
            ObjectDetection::ReachedNoObject
        ),
        ToolSettle::Fault(0b1000)
    );
}
