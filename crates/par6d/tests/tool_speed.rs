//! The tool-speed ceiling, met over the wire on a live `par6d --sim`: the
//! motion a client streams is slowed to it, not stopped for it.

use par6_proto::command::{JogJ, ServoJ};
use par6_proto::{Command, NUM_JOINTS};

mod common;
use common::{Client, Rig};

/// Arm out in front: the base swings the tool on about half a metre.
const REACH_DEG: [f64; NUM_JOINTS] = [0.0, -60.0, 220.0, 0.0, 30.0, 180.0];

/// Feed `next` on every status frame for `frames` of them and return the
/// fastest the tool was reported moving \[m/s\] and the last frame.
fn drive(
    rig: &Rig,
    c: &mut Client,
    frames: usize,
    next: impl Fn() -> Command,
) -> (f64, par6_proto::Status) {
    let mut fastest: f64 = 0.0;
    let mut last = rig.wait_status("a frame to start from", |_| true);
    for _ in 0..frames {
        c.send(&next());
        if let Some(s) = rig.recv_status() {
            fastest = fastest.max(s.tcp_speed / 1000.0);
            last = s;
        }
    }
    c.drain();
    (fastest, last)
}

/// A joint jog at full fraction and a servo target a long way round both
/// ask the base for its joint limit, which at this reach is several times
/// the ceiling. Both run — no stop, no error — with the tool at the
/// ceiling instead.
#[test]
fn jogs_and_streamed_joint_targets_are_slowed_to_the_tool_speed_ceiling() {
    let ceiling = 0.25;
    let config = common::retimed_config("tool-speed-live", 0.004);
    let rig = Rig::boot_with(config);
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);
    rig.wait_status("enabled", |s| s.enabled);
    common::teleport_home(&rig, &mut c, REACH_DEG);

    let (jogged, after) = drive(&rig, &mut c, 100, || {
        Command::JogJ(JogJ {
            speeds: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            duration: 0.2,
            accel: None,
        })
    });
    assert!(
        after.error.is_none() && after.angles[0] > 10.0,
        "the jog must have run: base at {:.1} deg, error {:?}",
        after.angles[0],
        after.error
    );
    assert!(
        jogged > 0.6 * ceiling && jogged < 1.1 * ceiling,
        "a full jog at this reach runs at the ceiling, not past it: {jogged:.3} m/s"
    );
    rig.wait_status("the jog to end", |s| {
        s.speeds.iter().all(|v| v.abs() < 0.01)
    });

    let mut target = REACH_DEG;
    target[0] = -80.0;
    let (served, after) = drive(&rig, &mut c, 100, || {
        Command::ServoJ(ServoJ {
            angles: target,
            speed: None,
            accel: None,
        })
    });
    assert!(
        after.error.is_none(),
        "the servo must not be stopped for its speed: {:?}",
        after.error
    );
    assert!(
        served > 0.6 * ceiling && served < 1.1 * ceiling,
        "a servo a long way round runs at the ceiling, not past it: {served:.3} m/s"
    );
    rig.shutdown();
}
