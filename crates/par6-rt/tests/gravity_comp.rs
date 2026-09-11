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

#[test]
fn gravity_trim_changes_only_requested_drive_feedforward() {
    use common::{bundle, ConstGravity};
    use par6_bus::spectral::{torque_to_ma_factor, trunc_to_wire};
    use par6_rt::CompletionPolicy;

    let mut b = bundle();
    let original = b.robot.joints.clone();
    b.robot.gravity_scale[2] = 1.3;
    b.robot.validate().unwrap();
    let g = [0.2, -1.5, 2.8, 0.05, -0.12, 0.01];
    let mut rig = Rig::build_bundle(
        b,
        CompletionPolicy::Settled,
        Box::new(ConstGravity(g)),
        true,
    );
    rig.ready();
    rig.tick_n(25);
    let s = rig.snap_after_tick();
    let frames = rig.last_joints();
    for (i, joint) in original.iter().enumerate() {
        let expected = g[i] * if i == 2 { 1.3 } else { 1.0 };
        assert!((s.gravity_torque_nm[i] - expected).abs() < 1e-12);
        let factor = torque_to_ma_factor(
            joint.gear_ratio,
            joint.gear_efficiency,
            joint.kt_nm_a,
            joint.dir,
        );
        assert_eq!(
            frames[i].cur_ma,
            Some(trunc_to_wire(expected * factor) as i16)
        );
        assert_eq!(frames[i].pos, None);
    }
    rig.cmd(RtCommand::SetGravityComp(false));
    rig.snap_after_tick();
    assert!(rig.last_joints().iter().all(|f| f.cur_ma == Some(0)));
}
