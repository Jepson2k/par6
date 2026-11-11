//! The floating arm, pushed around on the simulated plant in virtual time:
//! what a hand feels, where the arm stops by itself, and what it does when
//! it is not a hand that is moving it.
use crate::bridge::{FloatGuard, StreamGate};
use crate::sim_rig::SimRig;
use par6_rt::{ArmState, ErrorCode, ErrorEntry, Mode, RtCommand, StateSnapshot, MAX_JOINTS};
use par6_server::ShapeLayer;

/// Speed under which a joint counts as at rest \[rad/s\]: a little over
/// the step one encoder count makes in the drives' reported velocity.
const AT_REST: f64 = 0.05;

fn reach() -> [f64; MAX_JOINTS] {
    [0.0_f64, -60.0, 220.0, 0.0, 30.0, 180.0].map(f64::to_radians)
}

fn floating(s: &StateSnapshot) -> bool {
    s.mode == Mode::Idle && s.gravity_comp
}

/// Push the base with `nm` for `seconds` and return how far it went.
fn shove(rig: &mut SimRig, nm: f64, seconds: f64) -> f64 {
    let from = rig.tick().q[0];
    rig.push(0, nm);
    let mut s = rig.tick();
    for _ in 0..rig.ticks(seconds) {
        s = rig.tick();
    }
    rig.push(0, 0.0);
    s.q[0] - from
}

/// Left alone the arm stays where it is; pushed, it goes, and it stops
/// when the push does. The assist is what the hand notices: the same push
/// carries a joint further when part of its friction is paid for.
#[test]
fn a_floating_arm_stays_put_goes_where_it_is_pushed_and_is_lighter_with_assist() {
    let mut travelled = [0.0; 2];
    for (run, assist) in [0.6, 0.0].into_iter().enumerate() {
        let mut rig = SimRig::boot_with(0.004, reach(), |robot| {
            robot.freedrive.push_assist = assist;
        });
        let start = rig.tick();
        assert!(floating(&start), "a referenced, enabled idle arm floats");
        let mut rested = start;
        for _ in 0..rig.ticks(2.0) {
            rested = rig.tick();
        }
        let drift = (0..MAX_JOINTS)
            .map(|j| (rested.q[j] - start.q[j]).abs())
            .fold(0.0, f64::max);
        assert!(
            drift < 0.02 && floating(&rested),
            "assist {assist}: the arm wandered {drift:.4} rad with nothing touching it"
        );

        travelled[run] = shove(&mut rig, 0.45, 0.8);
        assert!(
            travelled[run] > 0.03,
            "assist {assist}: a 0.45 Nm push moved the base {:.4} rad",
            travelled[run]
        );
        let mut after = rig.tick();
        for _ in 0..rig.ticks(1.5) {
            after = rig.tick();
        }
        assert!(
            after.qd[0].abs() < AT_REST && floating(&after),
            "assist {assist}: the base kept going at {:.3} rad/s after the push ended",
            after.qd[0]
        );
    }
    assert!(
        travelled[0] > 1.15 * travelled[1],
        "the assist must make the same push go further: {:.3} rad with, {:.3} without",
        travelled[0],
        travelled[1]
    );
}

/// Pushed into a soft limit the joint is held there for as long as the push
/// lasts, while the rest of the arm goes on floating; a push back the other
/// way lets it go.
#[test]
fn a_joint_pushed_to_its_soft_limit_is_held_there_until_it_is_pushed_back() {
    let mut rig = SimRig::boot(0.004, reach());
    let limit = rig.bundle.robot.joints[0].limits.soft_max_rad;
    let mut near = reach();
    near[0] = limit - 0.1;
    rig.land(near);
    assert!(floating(&rig.tick()));

    rig.push(0, 0.8);
    let mut furthest = f64::MIN;
    let mut s = rig.tick();
    for _ in 0..rig.ticks(4.0) {
        s = rig.tick();
        furthest = furthest.max(s.q[0]);
    }
    assert!(
        furthest > limit - 0.02,
        "the push must have carried the base to its limit: reached {furthest:.4} of {limit:.4}"
    );
    assert!(
        furthest < limit + 0.05 && s.qd[0].abs() < AT_REST,
        "the base went {:.4} rad past its soft limit and is doing {:.3} rad/s",
        furthest - limit,
        s.qd[0]
    );
    assert!(
        floating(&s) && !s.protective_active,
        "a joint at its limit is held alone: the arm is still floating"
    );

    rig.push(0, -1.2);
    for _ in 0..rig.ticks(0.3) {
        s = rig.tick();
    }
    rig.push(0, 0.0);
    for _ in 0..rig.ticks(1.0) {
        s = rig.tick();
    }
    assert!(
        floating(&s) && s.q[0] < limit - 0.05,
        "pushed back in, the base must come off its limit: still at {:.4} of {limit:.4}",
        s.q[0]
    );
}

/// An arm moving faster than a hand moves it is caught: stopped where it
/// is and held, enabled and referenced, until it is reset. Floating is
/// then asked for again, not resumed.
#[test]
fn an_arm_thrown_while_floating_is_caught_and_held() {
    let mut rig = SimRig::boot(0.004, reach());
    assert!(floating(&rig.tick()));
    rig.push(0, 8.0);
    let mut caught = None;
    for _ in 0..rig.ticks(3.0) {
        let s = rig.tick();
        if s.protective_active {
            caught = Some(s);
            break;
        }
    }
    rig.push(0, 0.0);
    let caught = caught.expect("an 8 Nm shove on a floating base is not hand guiding");
    assert!(
        caught.errors.contains(ErrorEntry {
            code: ErrorCode::FreedriveCatch,
            joint: Some(0),
        }),
        "the base is the joint that ran away: {:?}",
        caught.errors.as_slice()
    );
    let mut held = caught;
    for _ in 0..rig.ticks(2.0) {
        held = rig.tick();
    }
    assert!(
        held.mode == Mode::Exec && held.qd.iter().all(|v| v.abs() < AT_REST),
        "the catch ends in a hold: mode {:?}, speeds {:?}",
        held.mode,
        held.qd
    );
    assert!(
        held.state == ArmState::Enabled && held.homed && !held.error_active,
        "a catch is not a fault"
    );

    rig.commands.send(RtCommand::SetGravityComp(true)).unwrap();
    let still = rig.tick();
    assert!(
        still.mode == Mode::Exec && still.protective_active,
        "a caught arm does not float again until it is reset"
    );
    rig.commands.send(RtCommand::ClearErrors).unwrap();
    let cleared = rig.tick();
    assert!(!cleared.protective_active && cleared.mode == Mode::Exec);
    rig.commands.send(RtCommand::SetGravityComp(true)).unwrap();
    rig.tick();
    assert!(
        floating(&rig.tick()),
        "asked again after the reset, it floats"
    );
}

/// A payload declared and not carried pushes a floating arm upward for as
/// long as it floats. It may not run away with it: the arm ends at rest
/// inside its soft window, caught or held at a limit.
#[test]
fn a_floating_arm_compensating_a_load_it_does_not_hold_cannot_run_away() {
    let mut rig = SimRig::boot(0.004, reach());
    rig.commands
        .send(RtCommand::SetPayload {
            mass: 0.8,
            com: [0.0, 0.0, 0.05],
            inertia: None,
        })
        .unwrap();
    let start = rig.tick();
    let mut s = start;
    for _ in 0..rig.ticks(8.0) {
        s = rig.tick();
    }
    let moved = (0..MAX_JOINTS)
        .map(|j| (s.q[j] - start.q[j]).abs())
        .fold(0.0, f64::max);
    assert!(
        moved > 0.05,
        "0.8 kg that is not there must have set the arm moving: it went {moved:.4} rad"
    );
    for (j, joint) in rig.bundle.robot.joints.iter().enumerate() {
        assert!(
            s.q[j] > joint.limits.soft_min_rad - 0.05 && s.q[j] < joint.limits.soft_max_rad + 0.05,
            "J{j} ended outside its soft window at {:.4} rad",
            s.q[j]
        );
        assert!(
            s.qd[j].abs() < AT_REST,
            "J{j} is still moving at {:.3} rad/s",
            s.qd[j]
        );
    }
}

/// Reaching down in front of the base, tool about 0.37 m out.
fn in_front() -> [f64; MAX_JOINTS] {
    [0.0, -0.25, 4.35, 0.0, -1.28, 0.0]
}

/// A gate whose world holds one keep-out, a post on the circle the tool
/// sweeps as the base turns, and the base angle at which the arm first
/// comes inside its clearance turning toward it from `in_front`.
fn gate_with_a_post(rig: &SimRig) -> (StreamGate, f64) {
    let mut gate = rig.gate();
    gate.set_layer(
        ShapeLayer::Program,
        &[par6_proto::Shape {
            kind: "box".into(),
            params: vec![0.06, 0.06, 0.4],
            pose: vec![0.3204, 0.185, 0.2, 0.0, 0.0, 0.0],
            collision: true,
            margin: None,
            name: "post".into(),
            physics: None,
            attachment: None,
        }],
    )
    .expect("the keep-out is accepted");
    let edge = (0..600)
        .map(|k| f64::from(k) * 0.002)
        .find(|&base| {
            let mut q = in_front();
            q[0] = base;
            gate.inside_world(&q).expect("collision query")
        })
        .expect("turning the base brings the arm to the post");
    assert!(edge > 0.1, "the post must stand clear of the start pose");
    (gate, edge)
}

/// Push the base with `nm` until the guard reports the arm inside a
/// keep-out, and send the RT what the daemon sends it then.
fn guide_until_caught(
    rig: &mut SimRig,
    gate: &mut StreamGate,
    guard: &mut FloatGuard,
    nm: f64,
    seconds: f64,
) -> Option<(StateSnapshot, Vec<(String, String)>)> {
    rig.push(0, nm);
    for _ in 0..rig.ticks(seconds) {
        let s = rig.tick();
        if let Some(pairs) = guard.entered(gate, &s) {
            rig.commands.send(RtCommand::KeepOutCatch).unwrap();
            return Some((s, pairs));
        }
    }
    None
}

/// Guided into a keep-out, the arm goes back the way it came — against the
/// hand still pushing it — and is held clear of the keep-out, enabled and
/// referenced, until it is reset. Floating again from there is not a catch.
#[test]
fn a_floating_arm_guided_into_a_keep_out_backs_out_the_way_it_came_and_is_held() {
    let mut rig = SimRig::boot(0.004, in_front());
    let (mut gate, edge) = gate_with_a_post(&rig);
    let mut near = in_front();
    near[0] = edge - 0.08;
    rig.land(near);
    let mut guard = FloatGuard::new();
    let start = rig.tick();
    assert!(floating(&start));
    assert!(
        guard.entered(&mut gate, &start).is_none(),
        "an arm clear of every keep-out is left alone"
    );

    let (caught, pairs) = guide_until_caught(&mut rig, &mut gate, &mut guard, 0.8, 8.0)
        .expect("a push toward the post must carry the arm into its clearance");
    assert!(
        pairs
            .iter()
            .any(|(a, b)| a == "shape:post" || b == "shape:post"),
        "the keep-out it entered is the one named: {pairs:?}"
    );
    assert!(
        caught.q[0] - edge < 0.02,
        "caught {:.4} rad inside the clearance",
        caught.q[0] - edge
    );

    let mut held = caught;
    for _ in 0..rig.ticks(2.0) {
        held = rig.tick();
    }
    rig.push(0, 0.0);
    assert!(
        held.errors.contains(ErrorEntry {
            code: ErrorCode::KeepOut,
            joint: None,
        }) && held.protective_active,
        "the catch stands: {:?}",
        held.errors.as_slice()
    );
    let backed_off = edge - held.q[0];
    assert!(
        !gate.inside_world(&held.q).unwrap()
            && backed_off > rig.bundle.robot.contact.retreat_max_rad / 4.0,
        "the arm must end clear of the keep-out, back along the way it came: \
         {backed_off:.4} rad short of the clearance"
    );
    assert!(
        held.mode == Mode::Exec && held.qd.iter().all(|v| v.abs() < AT_REST),
        "the catch ends in a hold: mode {:?}, speeds {:?}",
        held.mode,
        held.qd
    );
    assert!(
        held.state == ArmState::Enabled && held.homed && !held.error_active,
        "a catch is not a fault"
    );

    rig.commands.send(RtCommand::SetGravityComp(true)).unwrap();
    assert!(
        !floating(&rig.tick()),
        "a caught arm does not float again until it is reset"
    );
    rig.commands.send(RtCommand::ClearErrors).unwrap();
    rig.tick();
    rig.commands.send(RtCommand::SetGravityComp(true)).unwrap();
    rig.tick();
    let again = rig.tick();
    assert!(floating(&again), "asked again after the reset, it floats");
    assert!(
        guard.entered(&mut gate, &again).is_none(),
        "floating from where it was backed out to is not another catch"
    );
    assert!(
        guide_until_caught(&mut rig, &mut gate, &mut guard, -0.8, 1.0).is_none(),
        "guided away from the keep-out, the arm is left to float"
    );
    rig.push(0, 0.0);
    assert!(floating(&rig.tick()));
}

/// An arm that begins to float inside a keep-out has no way it came: it is
/// left to be guided out, and caught if it is then taken back in.
#[test]
fn an_arm_floated_inside_a_keep_out_is_let_out_and_caught_going_back_in() {
    let mut rig = SimRig::boot(0.004, in_front());
    let (mut gate, edge) = gate_with_a_post(&rig);
    let mut inside = in_front();
    inside[0] = edge + 0.04;
    rig.land(inside);
    let mut guard = FloatGuard::new();
    let start = rig.tick();
    assert!(floating(&start) && gate.inside_world(&start.q).unwrap());

    rig.push(0, -0.8);
    let mut s = start;
    for _ in 0..rig.ticks(8.0) {
        s = rig.tick();
        assert!(
            guard.entered(&mut gate, &s).is_none(),
            "caught on the way out, at {:.4} rad from the clearance",
            s.q[0] - edge
        );
        if s.q[0] < edge - 0.06 {
            break;
        }
    }
    rig.push(0, 0.0);
    assert!(
        floating(&s) && !gate.inside_world(&s.q).unwrap(),
        "the arm must have been guided clear, still floating: {:.4} rad from the clearance",
        s.q[0] - edge
    );

    let (caught, _) = guide_until_caught(&mut rig, &mut gate, &mut guard, 0.8, 8.0)
        .expect("taken back in, the arm is caught");
    rig.push(0, 0.0);
    assert!(caught.q[0] >= edge - 0.002);
    let mut held = caught;
    for _ in 0..rig.ticks(2.0) {
        held = rig.tick();
    }
    assert!(held.protective_active && !gate.inside_world(&held.q).unwrap());
}
