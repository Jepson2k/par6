//! Contact detection and the stop that follows, against the simulated arm
//! on the daemon's own dynamics model, advanced in virtual time.
use crate::sim_rig::SimRig;
use par6_config::ContactReaction;
use par6_rt::contact::ContactChannel;
use par6_rt::{ArmState, ErrorCode, ErrorEntry, Mode, RtCommand, StateSnapshot, MAX_JOINTS};

fn reach() -> [f64; MAX_JOINTS] {
    [0.0_f64, -60.0, 220.0, 0.0, 30.0, 180.0].map(f64::to_radians)
}

fn latched(s: &StateSnapshot, code: ErrorCode, joint: usize) -> bool {
    s.errors.contains(ErrorEntry {
        code,
        joint: Some(joint as u8),
    })
}

/// Swing the base toward `target` until it is travelling, returning the
/// snapshot it was last seen in.
fn swing(rig: &mut SimRig, target: [f64; MAX_JOINTS]) -> StateSnapshot {
    for _ in 0..rig.ticks(2.0) {
        let s = rig.stream(target);
        if s.qd[0].abs() > 0.4 {
            return s;
        }
    }
    panic!("the base never got up to speed");
}

/// The whole stop, the way an operator meets it: the arm runs into
/// something, stops, backs off the way it came and holds there; it stays
/// enabled and referenced; it will not move again until it is reset; and
/// the reset is immediate, because nothing on the drives needs clearing.
#[test]
fn a_moving_arm_stops_for_what_it_runs_into_backs_off_and_holds() {
    for dt in [0.004, 0.02] {
        for direction in [1.0, -1.0] {
            let start = reach();
            let mut rig = SimRig::boot(dt, start);
            rig.hold();
            let mut target = start;
            target[0] += direction * 1.2;
            swing(&mut rig, target);

            // Something in the way: a torque against the base's travel.
            rig.push(0, -direction * 2.0);
            let mut hit = None;
            for _ in 0..rig.ticks(0.3) {
                let s = rig.stream(target);
                if s.protective_active {
                    hit = Some(s);
                    break;
                }
            }
            let hit = hit.unwrap_or_else(|| panic!("dt {dt}: the contact was never read"));
            assert!(
                latched(&hit, ErrorCode::Contact, 0),
                "dt {dt}: the base is the joint that read it: {:?}",
                hit.errors.as_slice()
            );
            assert!(
                !hit.error_active && hit.state == ArmState::Enabled && hit.homed,
                "dt {dt}: a contact is not a fault: the arm stays enabled and referenced"
            );

            // The stream keeps asking for the far side, as a client that
            // has not heard yet would; the arm retreats regardless.
            let mut held = hit;
            for _ in 0..rig.ticks(2.0) {
                held = rig.stream(target);
                if held.mode == Mode::Exec {
                    break;
                }
            }
            assert_eq!(held.mode, Mode::Exec, "dt {dt}: the retreat ends in a hold");
            rig.push(0, 0.0);
            for _ in 0..rig.ticks(0.5) {
                held = rig.stream(target);
            }
            let backed_off = direction * (hit.q[0] - held.q[0]);
            assert!(
                backed_off > 0.02,
                "dt {dt}: the arm backed off {backed_off:+.4} rad from where it read the contact"
            );
            let mut later = held;
            for _ in 0..rig.ticks(0.3) {
                later = rig.stream(target);
            }
            let drift = (0..MAX_JOINTS)
                .map(|j| (later.q[j] - held.q[j]).abs())
                .fold(0.0, f64::max);
            assert!(
                later.mode == Mode::Exec && drift < 0.002,
                "dt {dt}: and holds there: it moved {drift:.5} rad in the next 0.3 s"
            );

            // Latched: neither moved nor let go until the operator has
            // dealt with it. IDLE is where the arm would float.
            rig.commands.send(RtCommand::SetMode(Mode::Idle)).unwrap();
            let kept = rig.tick();
            assert_eq!(
                kept.mode,
                Mode::Exec,
                "dt {dt}: a latched contact keeps its hold"
            );
            rig.commands.send(RtCommand::SetMode(Mode::Stream)).unwrap();
            let refused = rig.stream(target);
            assert_ne!(
                refused.mode,
                Mode::Stream,
                "dt {dt}: a latched contact refuses motion"
            );
            assert!(refused.protective_active);

            // Reset: gone on the next tick, with nothing to wait out.
            rig.commands.send(RtCommand::ClearErrors).unwrap();
            let cleared = rig.tick();
            assert!(
                !cleared.protective_active && cleared.errors.is_empty(),
                "dt {dt}: a contact clears at once: {:?}",
                cleared.errors.as_slice()
            );
            rig.commands.send(RtCommand::SetMode(Mode::Idle)).unwrap();
            let at = rig.hold();
            let mut away = at;
            away[0] -= direction * 0.3;
            let mut moved = rig.stream(away);
            for _ in 0..rig.ticks(1.5) {
                moved = rig.stream(away);
            }
            assert!(
                (moved.q[0] - away[0]).abs() < 0.02 && !moved.protective_active,
                "dt {dt}: and the arm moves away again"
            );
        }
    }
}

/// Every joint, flat out, there and back with nothing in the way: no
/// stop, and the peaks the run leaves behind are under half the thresholds
/// the arm ships with — at the shipped tick, and at the coarse one the
/// daemon's own test rigs run at.
#[test]
fn nothing_in_the_way_is_never_a_contact() {
    let from = reach();
    let to = [50.0_f64, -95.0, 250.0, 60.0, -30.0, 250.0].map(f64::to_radians);
    for dt in [0.004, 0.02] {
        let mut rig = SimRig::boot(dt, from);
        assert_eq!(
            rig.bundle.robot.contact.reaction,
            ContactReaction::Stop,
            "the shipped arm stops for a contact"
        );
        let at = rig.hold();
        let mut last = rig.stream(at);
        for target in [to, from, to, from] {
            for _ in 0..rig.ticks(3.0) {
                last = rig.stream(target);
                assert!(
                    !last.protective_active && last.mode == Mode::Stream,
                    "dt {dt}: a free move stopped: {:?}",
                    last.errors.as_slice()
                );
            }
        }
        let c = last.contact;
        assert_eq!(c.trips, 0);
        for j in 0..MAX_JOINTS {
            assert!(
                c.peak_signal[j] > 0.0,
                "dt {dt}, J{j}: the run was watched (peak {:.4} Nm)",
                c.peak_signal[j]
            );
            assert!(
                c.peak_signal[j] < 0.5 * c.sustained_nm[j],
                "dt {dt}, J{j}: free motion peaked at {:.3} Nm against a {:.3} Nm threshold",
                c.peak_signal[j],
                c.sustained_nm[j]
            );
            assert!(
                c.peak_impact[j] < 0.5 * c.impact_nm[j],
                "dt {dt}, J{j}: free motion bumped {:.3} Nm against a {:.3} Nm threshold",
                c.peak_impact[j],
                c.impact_nm[j]
            );
        }
    }
}

/// A load the model was never told about shifts the whole estimate on
/// the joints that carry it. The arm still moves freely under it, and
/// still catches a bump: the impact channel reads the change, not the
/// level.
#[test]
fn a_bump_is_caught_through_a_load_the_model_does_not_know() {
    let start = reach();
    let mut target = start;
    target[0] += 1.2;
    let undeclared = 1.2;

    // Free under the load: no stop on the way there.
    let mut rig = SimRig::boot(0.004, start);
    rig.push(1, undeclared);
    rig.hold();
    let mut last = rig.tick();
    for _ in 0..rig.ticks(3.0) {
        last = rig.stream(target);
    }
    assert!(
        !last.protective_active && (last.q[0] - target[0]).abs() < 0.02,
        "the loaded arm reached its target: {:?}",
        last.errors.as_slice()
    );
    assert!(
        last.contact.signal[1].abs() > 0.2,
        "the load is in the shoulder's signal: {:+.3} Nm",
        last.contact.signal[1]
    );

    // A bump on the way.
    let mut rig = SimRig::boot(0.004, start);
    rig.push(1, undeclared);
    rig.hold();
    swing(&mut rig, target);
    rig.push(0, -2.0);
    let mut hit = None;
    for _ in 0..rig.ticks(0.3) {
        let s = rig.stream(target);
        if s.protective_active {
            hit = Some(s);
            break;
        }
    }
    let hit = hit.expect("the bump was read through the load");
    assert_eq!(
        hit.contact.last_trip.map(|t| (t.joint, t.channel)),
        Some((0, ContactChannel::Impact)),
        "a bump is the impact channel's to read"
    );
}

/// An arm configured to report rather than stop keeps moving through the
/// same contact and says so; the warning goes when the contact does.
#[test]
fn a_reporting_arm_warns_and_keeps_moving() {
    let start = reach();
    let mut rig = SimRig::boot_with(0.004, start, |robot| {
        robot.contact.reaction = ContactReaction::Warn;
        // The rig streams at full rate, underneath what slows motion to
        // the tool-speed ceiling.
        robot.limits.tcp_speed_max_m_s = 0.0;
    });
    rig.hold();
    let mut target = start;
    target[0] += 1.2;
    let at_speed = swing(&mut rig, target);

    rig.push(0, -2.0);
    let mut warned = false;
    let mut last = at_speed;
    for _ in 0..rig.ticks(0.4) {
        last = rig.stream(target);
        warned |= latched(&last, ErrorCode::ContactWarning, 0);
        assert!(
            !last.protective_active && last.mode == Mode::Stream,
            "a reporting arm does not stop"
        );
    }
    assert!(warned, "the contact was reported");
    assert!(
        last.q[0] - at_speed.q[0] > 0.05,
        "and the arm kept going through it"
    );

    rig.push(0, 0.0);
    let hold = rig.bundle.robot.contact.warning_hold_s;
    for _ in 0..rig.ticks(3.0 + hold) {
        last = rig.stream(target);
    }
    assert!(
        !latched(&last, ErrorCode::ContactWarning, 0),
        "the warning goes with the contact"
    );
}

/// Leaning on an arm that is holding still is not something it can stop
/// for — it is not moving. It reports the push and stays where it is.
#[test]
fn a_push_on_a_resting_arm_is_reported_not_stopped_for() {
    let mut rig = SimRig::boot(0.004, reach());
    let at = rig.hold();
    for _ in 0..rig.ticks(0.5) {
        rig.stream(at);
    }
    rig.push(1, 4.5);
    let mut warned = false;
    let mut last = rig.tick();
    for _ in 0..rig.ticks(1.0) {
        last = rig.stream(at);
        warned |= latched(&last, ErrorCode::ContactWarning, 1);
    }
    assert!(warned, "the push was reported");
    assert!(
        !last.protective_active && last.mode == Mode::Stream,
        "a resting arm has nothing to stop: {:?}",
        last.errors.as_slice()
    );
    assert!(
        (last.q[1] - at[1]).abs() < 0.05,
        "and it holds its pose against the push"
    );
}

/// A joint standing outside its soft window — jogged there before the arm
/// was referenced, or guided there by hand — stays where it is when
/// another joint's contact stops the arm. The retreat goes back along the
/// arm's own track; it is not a move into the window.
#[test]
fn a_retreat_leaves_a_joint_outside_its_soft_window_where_it_is() {
    let mut rig = SimRig::boot(0.004, reach());
    let wrist = rig.bundle.robot.joints[4].limits;
    let (_, hard_max) = wrist.travel_rad();
    let outside = wrist.soft_max_rad + 0.5 * (hard_max - wrist.soft_max_rad);
    assert!(
        outside - wrist.soft_max_rad > 0.1,
        "the wrist has room past its soft window"
    );
    let mut start = reach();
    start[4] = outside;
    rig.land(start);

    // Jog the base: a jog asks for a direction and a speed, so the wrist
    // is left where it stands.
    rig.commands.send(RtCommand::SetMode(Mode::Jog)).unwrap();
    rig.tick();
    let mut speeds = [0.0; MAX_JOINTS];
    speeds[0] = 0.5;
    rig.commands
        .send(RtCommand::Jog { speeds, accel: 1.0 })
        .unwrap();
    let mut moving = rig.tick();
    for _ in 0..rig.ticks(2.0) {
        moving = rig.tick();
        if moving.qd[0] > 0.4 {
            break;
        }
    }
    assert!(moving.qd[0] > 0.4, "the base jogs: {:?}", moving.qd);
    assert!(
        (moving.q[4] - outside).abs() < 0.01,
        "the jog left the wrist outside its window"
    );

    rig.push(0, -2.0);
    let mut held = moving;
    for _ in 0..rig.ticks(2.0) {
        held = rig.tick();
        if held.protective_active && held.mode == Mode::Exec {
            break;
        }
    }
    rig.push(0, 0.0);
    assert!(
        held.protective_active && held.mode == Mode::Exec,
        "the contact stopped the jog into a hold: {:?} in {:?}",
        held.errors.as_slice(),
        held.mode
    );
    for _ in 0..rig.ticks(0.5) {
        held = rig.tick();
    }
    assert!(
        (held.q[4] - outside).abs() < 0.01,
        "the retreat moved the wrist {:+.4} rad toward its soft window",
        held.q[4] - outside
    );
}

/// What an arm read before it was referenced is not a level to judge its
/// first move against. Unreferenced, the model's angles are a guess and
/// the estimate carries whatever the guess leaves over; the moment the
/// arm is referenced that is gone, and a move begun right then runs.
#[test]
fn a_move_begun_as_the_arm_is_referenced_is_not_a_contact() {
    let start = reach();
    let mut rig = SimRig::boot(0.004, start);
    // Held on its drives rather than floating, so the arm is being read
    // from the tick it is referenced in.
    rig.commands.send(RtCommand::SetGravityComp(false)).unwrap();
    // An unreferenced arm with a standing torque in its estimate: the
    // elbow's, here, from something leaning on it.
    rig.core.set_homed(false);
    rig.push(2, 2.5);
    let mut leaning = rig.tick();
    for _ in 0..rig.ticks(1.5) {
        leaning = rig.tick();
    }
    assert!(
        leaning.tau_ext[2].abs() > 1.0 && !leaning.homed,
        "the unreferenced elbow's estimate stands at {:+.3} Nm",
        leaning.tau_ext[2]
    );

    rig.push(2, 0.0);
    rig.land(start);
    rig.hold();
    let mut target = start;
    target[0] += 0.6;
    let mut last = rig.tick();
    for _ in 0..rig.ticks(2.0) {
        last = rig.stream(target);
        assert!(
            !last.protective_active,
            "the first move of a freshly referenced arm stopped: {:?} ({:?})",
            last.errors.as_slice(),
            last.contact.last_trip
        );
    }
    assert!(
        (last.q[0] - target[0]).abs() < 0.02,
        "and it arrived: {:+.4} rad short",
        target[0] - last.q[0]
    );
}

/// A payload is declared before the arm has it and stays declared until
/// after it has let go, so for part of every pick the arm compensates a
/// weight it does not hold and the estimate reads that weight missing.
/// Anywhere between not holding it and holding it is not a contact. A load
/// past the far side of that — more than was ever declared — still is.
#[test]
fn a_declared_payload_the_arm_is_not_holding_is_not_a_contact() {
    let from = reach();
    let to = [0.0_f64, -95.0, 250.0, 0.0, 30.0, 180.0].map(f64::to_radians);
    let mut rig = SimRig::boot(0.004, from);
    rig.hold();
    rig.commands
        .send(RtCommand::SetPayload {
            mass: 0.8,
            com: [0.0, 0.0, 0.05],
            inertia: None,
        })
        .unwrap();

    let limit = rig.bundle.robot.contact.sustained_nm[1];
    let mut missing: f64 = 0.0;
    for target in [to, from] {
        for _ in 0..rig.ticks(3.0) {
            let s = rig.stream(target);
            assert!(
                !s.protective_active,
                "the missing payload was read as a contact: {:?}",
                s.errors.as_slice()
            );
            if s.tau_ext[1].abs() > missing.abs() {
                missing = s.tau_ext[1];
            }
        }
    }
    assert!(
        missing.abs() > limit,
        "0.8 kg at this reach is more than the shoulder's threshold, so the run above was \
         only quiet because the payload was allowed for: {missing:+.2} Nm against {limit}"
    );

    // Half its weight carried — the block coming off the table — is
    // still inside what was declared.
    rig.push(1, 0.5 * missing);
    for target in [to, from] {
        for _ in 0..rig.ticks(3.0) {
            let s = rig.stream(target);
            assert!(
                !s.protective_active,
                "a payload half carried was read as a contact: {:?}",
                s.errors.as_slice()
            );
        }
    }

    // A push the payload cannot account for: past carrying all of it.
    rig.push(1, 0.0);
    let mut moving = rig.stream(to);
    for _ in 0..rig.ticks(1.0) {
        moving = rig.stream(to);
        if moving.qd[1].abs() > 0.2 {
            break;
        }
    }
    assert!(moving.qd[1].abs() > 0.2, "the shoulder never got moving");
    rig.push(1, missing + missing.signum() * (limit + 1.5));
    let mut hit = None;
    for _ in 0..rig.ticks(0.5) {
        let s = rig.stream(to);
        if s.protective_active {
            hit = Some(s);
            break;
        }
    }
    let hit = hit.expect("a load beyond the declared payload is a contact");
    assert!(
        latched(&hit, ErrorCode::Contact, 1),
        "the shoulder is the joint that read it: {:?}",
        hit.errors.as_slice()
    );
}
