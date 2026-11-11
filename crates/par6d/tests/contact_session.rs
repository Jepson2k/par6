//! A contact, met the way a client meets it: a live `par6d --sim` whose
//! physics holds a post the runtime was never told about, and an arm sent
//! through where it stands.

use par6_proto::command::MoveJ;
use par6_proto::{Command, ContactChannel, ErrorCode, QueryResult, NUM_JOINTS};

mod common;
use common::{Client, Rig, BUDGET};

/// Reaching down in front of the base, tool about 0.37 m out.
const REACH_RAD: [f64; NUM_JOINTS] = [0.0, -0.25, 4.35, 0.0, -1.28, 0.0];

fn move_j(key: u64, angles: [f64; NUM_JOINTS]) -> Command {
    Command::MoveJ(MoveJ {
        key,
        angles,
        duration: None,
        speed: Some(0.5),
        accel: None,
        blend_radius: None,
        rel: false,
    })
}

fn contact(c: &mut Client) -> par6_proto::ContactResult {
    match c.query(&Command::Contact) {
        QueryResult::Contact(r) => r,
        other => panic!("CONTACT answered {other:?}"),
    }
}

/// The whole event over the wire: the move ends in a contact error with the
/// arm short of the post and backed off it, the controller still enabled
/// and referenced, the error standing in STATUS and in front of every
/// further move; `reset` clears it and the arm moves away.
#[test]
fn a_move_into_something_undeclared_stops_backs_off_and_waits_for_reset() {
    let config = common::retimed_config("contact-session", 0.004);
    // A post on the tool's circle, 30 degrees round from where it starts.
    let scenario = config.with_file_name("post.toml");
    std::fs::write(
        &scenario,
        "[obstacle]\nsize_m = [0.06, 0.06, 0.4]\npose = [0.3204, 0.185, 0.2, 0.0, 0.0, 0.0]\n",
    )
    .expect("write the scenario");
    let rig = Rig::boot_in_scenario(config, scenario);
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);
    rig.wait_status("enabled", |s| s.enabled);
    let start = common::to_deg(&REACH_RAD);
    common::teleport_home(&rig, &mut c, start);

    let mut through = start;
    through[0] = 60.0;
    let index = c.ok_index(&move_j(1, through));
    let run = rig.collect_through(index, BUDGET);
    let (ok, detail) = c.wait_complete(index);
    let detail = detail.expect("a move stopped by a contact says why");
    assert!(
        !ok && detail.code == ErrorCode::SysContact as u16,
        "the move into the post must end as a contact, got ok={ok} {detail:?}"
    );
    let furthest = run.iter().map(|s| s.angles[0]).fold(f64::MIN, f64::max);
    assert!(
        (5.0..30.0).contains(&furthest),
        "the arm stopped at the post, not before it and not through it: {furthest:.1} deg"
    );

    // The arm may have been pressed against the post, at rest, for a
    // moment before it read the contact: what is waited for is the hold
    // it ends in, off the post.
    let held = rig.wait_status("the arm to back off the post and hold", |s| {
        s.angles[0] < furthest - 0.5 && s.speeds.iter().all(|v| v.abs() < 0.05)
    });
    assert!(
        held.enabled && held.homed,
        "a contact is not a fault: the arm stays enabled and referenced"
    );
    assert_eq!(
        held.error.as_ref().map(|e| e.code),
        Some(ErrorCode::SysContact as u16),
        "the contact stands in STATUS until it is reset"
    );

    let readout = contact(&mut c);
    assert_eq!(readout.trips, 1, "one contact, counted once");
    assert!(
        matches!(
            readout.last_trip,
            Some((_, ContactChannel::Sustained | ContactChannel::Impact))
        ),
        "the readout names the joint and channel that read it: {:?}",
        readout.last_trip
    );

    // Away from the post is still a move, and still refused.
    let mut away = start;
    away[0] = -20.0;
    let refused = c.expect_error(&move_j(2, away));
    assert_eq!(
        refused.code,
        ErrorCode::SysContact as u16,
        "motion is refused with the contact until it is reset: {refused:?}"
    );

    c.ok(&Command::Reset);
    let cleared = rig.wait_status("the contact cleared", |s| s.error.is_none());
    assert!(
        cleared.enabled && cleared.homed,
        "reset re-arms the arm without disabling or un-referencing it"
    );
    let index = c.ok_index(&move_j(3, away));
    let (ok, detail) = c.wait_complete(index);
    assert!(ok, "the arm moves away after the reset, got {detail:?}");
    let there = rig.wait_status("the arm away from the post", |s| {
        (s.angles[0] - away[0]).abs() < 1.0
    });
    assert!(there.error.is_none());
    rig.shutdown();
}
