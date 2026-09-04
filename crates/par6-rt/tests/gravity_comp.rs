//! STATUS `gravity_comp` means the feedforward is being applied this
//! tick, not that it was requested: a Waldo Commander reading the field
//! per its contract concludes the arm is back-driveable, and an arm held
//! under a position law is not.

mod common;

use common::Rig;
use par6_rt::{ArmState, RtCommand};

#[test]
fn status_reports_the_gravity_feedforward_actually_applied() {
    let mut rig = Rig::new();
    rig.boot_to_idle();
    rig.cmd(RtCommand::SetGravityComp(true));
    let s = rig.snap_after_tick();
    assert!(
        !s.gravity_comp,
        "un-referenced, disabled arm: the request stands but nothing is applied"
    );

    rig.core.set_homed(true);
    rig.cmd(RtCommand::Enable);
    let s = rig.tick_until(50, |s| s.gravity_comp);
    assert_eq!(
        s.state,
        ArmState::Enabled,
        "the flag turns on only once the arm is enabled"
    );

    rig.cmd(RtCommand::SetGravityComp(false));
    let s = rig.snap_after_tick();
    assert!(!s.gravity_comp, "the request is withdrawn, so is the flag");
}
