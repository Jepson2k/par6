//! A non-finite jog entry is a stop for that joint, never "keep doing
//! what it was doing": `[0, 0, NaN, 0, 0, 0]` sent as a stop must bring
//! joint 2 to rest like a zero would.

mod common;

use common::Rig;
use par6_rt::{Mode, RtCommand, MAX_JOINTS};

fn jog(rig: &mut Rig, speeds: [f64; MAX_JOINTS]) {
    rig.cmd(RtCommand::Jog { speeds, accel: 1.0 });
}

#[test]
fn a_nan_entry_stops_the_joint_instead_of_keeping_its_request() {
    let mut speeds = [0.0; MAX_JOINTS];
    speeds[2] = 0.5;

    // Control: without the stop the joint is still moving at the end.
    let mut running = Rig::new();
    running.ready();
    running.cmd(RtCommand::SetMode(Mode::Jog));
    jog(&mut running, speeds);
    running.tick_n(20);
    assert!(running.snap().qd_commanded[2].abs() > 1e-6, "joint 2 jogs");
    running.tick_n(60);
    assert!(
        running.snap().qd_commanded[2].abs() > 1e-6,
        "the control run keeps joint 2 moving for the whole window"
    );

    let mut stopped = Rig::new();
    stopped.ready();
    stopped.cmd(RtCommand::SetMode(Mode::Jog));
    jog(&mut stopped, speeds);
    stopped.tick_n(20);
    let mut nan_stop = [0.0; MAX_JOINTS];
    nan_stop[2] = f64::NAN;
    jog(&mut stopped, nan_stop);
    stopped.tick_n(60);
    assert!(
        stopped.snap().qd_commanded[2].abs() < 1e-9,
        "a NaN entry must stop joint 2: qd = {}",
        stopped.snap().qd_commanded[2]
    );
}
