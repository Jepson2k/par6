//! Every mode change runs the transition hooks, and SAFETY_STOP outranks
//! the hard-error reaction: a hard error latched during a protective stop
//! must keep the limp law, and one latched mid-homing must hand the
//! gripper back idle instead of leaving the jaws clamped.

mod common;

use common::Rig;
use par6_rt::{ErrorCode, Mode, RtCommand, MAX_JOINTS};

fn has_error(rig: &mut Rig, code: ErrorCode) -> bool {
    rig.snap().errors.as_slice().iter().any(|e| e.code == code)
}

/// Silence node 3 until the RT hard-latches it as lost.
fn latch_can_lost(rig: &mut Rig) {
    let robot = common::bundle().robot;
    let lost_ticks = robot.ticks(robot.bus.lost_s);
    rig.skip_nodes = 1 << 3;
    rig.tick_n(lost_ticks + 2);
    assert!(
        has_error(rig, ErrorCode::CanLost),
        "node 3 must be latched lost"
    );
}

#[test]
fn a_hard_error_during_safety_stop_keeps_the_limp_law() {
    let mut rig = Rig::new();
    rig.ready();
    rig.cmd(RtCommand::SetMode(Mode::SafetyStop));
    assert_eq!(rig.snap().mode, Mode::SafetyStop);

    latch_can_lost(&mut rig);
    let s = rig.snap();
    assert_eq!(
        s.mode,
        Mode::SafetyStop,
        "the protective stop must not be rewritten into ACTIVE_ERROR's hold"
    );
    let last = rig.last_joints();
    assert_eq!(last.len(), MAX_JOINTS);
    for (i, c) in last.iter().enumerate() {
        assert_eq!(c.pos, None, "J{i}: limp frames hold no position");
        assert_eq!(c.vel, None, "J{i}: limp frames command no velocity");
        assert_eq!(
            c.cur_ma.unwrap_or(0),
            0,
            "J{i}: limp frames carry no current"
        );
    }
}
