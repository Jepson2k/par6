//! The external-torque estimate against the simulated arm, on the
//! daemon's own dynamics model and advanced in virtual time.
use crate::sim_rig::SimRig;
use par6_rt::{Mode, StateSnapshot, MAX_JOINTS};

/// Arm out in front, elbow bent: every gravity-loaded joint carries load
/// and nothing is near a limit.
fn reach() -> [f64; MAX_JOINTS] {
    [0.0_f64, -60.0, 220.0, 0.0, 30.0, 180.0].map(f64::to_radians)
}

fn coulomb(rig: &SimRig) -> [f64; MAX_JOINTS] {
    std::array::from_fn(|i| rig.bundle.robot.dynamics.friction_coulomb_nm[i])
}

/// Mean external-torque estimate over `seconds` of a hold at `at`.
fn mean_estimate(rig: &mut SimRig, at: [f64; MAX_JOINTS], seconds: f64) -> [f64; MAX_JOINTS] {
    let n = rig.ticks(seconds);
    let mut sum = [0.0; MAX_JOINTS];
    for _ in 0..n {
        let s = rig.stream(at);
        for (sum, estimate) in sum.iter_mut().zip(s.tau_ext) {
            *sum += estimate;
        }
    }
    sum.map(|v| v / f64::from(n))
}

/// A torque applied to one joint of a held arm comes back in that joint's
/// estimate, with the sign of the effort the motor spends against it, and
/// the estimate lets go when the push does. What stiction can carry on the
/// motor's behalf is the one thing the estimate cannot see, so that is
/// the tolerance.
#[test]
fn a_push_on_a_held_joint_is_read_back() {
    for dt in [0.004, 0.02] {
        for (joint, push) in [(0, 1.5), (1, -4.0), (2, 3.0), (4, 0.5)] {
            let mut rig = SimRig::boot(dt, reach());
            let at = rig.hold();
            let friction = coulomb(&rig);
            mean_estimate(&mut rig, at, 0.5);
            let rest = mean_estimate(&mut rig, at, 0.2);

            rig.push(joint, push);
            mean_estimate(&mut rig, at, 0.5);
            let pushed = mean_estimate(&mut rig, at, 0.2);
            let read = pushed[joint] - rest[joint];
            assert!(
                (read + push).abs() <= friction[joint] + 0.15 * push.abs(),
                "dt {dt}: a {push:+.2} Nm push on J{joint} read as {read:+.3} Nm"
            );

            rig.push(joint, 0.0);
            mean_estimate(&mut rig, at, 0.5);
            let released = mean_estimate(&mut rig, at, 0.2);
            assert!(
                (released[joint] - rest[joint]).abs() <= 2.0 * friction[joint] + 0.1,
                "dt {dt}: J{joint} still reads {:+.3} Nm after the push ended",
                released[joint] - rest[joint]
            );
        }
    }
}

/// Peak of `value` per joint over a run of snapshots.
fn peaks(run: &[StateSnapshot], value: impl Fn(&StateSnapshot, usize) -> f64) -> [f64; MAX_JOINTS] {
    std::array::from_fn(|j| run.iter().map(|s| value(s, j).abs()).fold(0.0, f64::max))
}

/// Stream every joint from `from` to `to` and back at the stream's own
/// limits, returning every snapshot on the way.
fn sweep(rig: &mut SimRig, from: [f64; MAX_JOINTS], to: [f64; MAX_JOINTS]) -> Vec<StateSnapshot> {
    let mut run = Vec::new();
    for target in [to, from] {
        for _ in 0..rig.ticks(3.0) {
            run.push(rig.stream(target));
        }
    }
    run
}

/// A fast move with nothing in its way reads as nothing in its way: the
/// friction and the acceleration the arm spends on itself are the
/// model's, not an external torque. Measured against what the estimate
/// was before it knew either — the motor torque left over after gravity.
#[test]
fn the_arms_own_motion_is_not_read_as_contact() {
    let from = reach();
    let to = [50.0_f64, -95.0, 250.0, 60.0, -30.0, 250.0].map(f64::to_radians);
    let mut rig = SimRig::boot(0.004, from);
    rig.hold();
    let run = sweep(&mut rig, from, to);
    assert!(
        run.iter()
            .all(|s| s.mode == Mode::Stream && !s.error_active),
        "the sweep ran clean"
    );
    let travelled = peaks(&run, |s, j| s.q[j] - from[j]);
    assert!(
        travelled.iter().all(|d| *d > 0.3),
        "every joint moved: {travelled:.2?}"
    );

    let estimate = peaks(&run, |s, j| s.tau_ext[j]);
    let gravity_only = peaks(&run, |s, j| s.tau_filtered[j] - s.gravity_torque_nm[j]);
    for j in 0..MAX_JOINTS {
        assert!(
            estimate[j] < 0.6 * gravity_only[j],
            "J{j}: the estimate peaked at {:.3} Nm where gravity alone leaves {:.3} Nm",
            estimate[j],
            gravity_only[j]
        );
    }
}

/// A floating arm that is pushed accelerates, and its motors do nothing
/// about it: the push is visible only through the arm's own inertia. The
/// estimate reads it while the arm is still gathering speed.
#[test]
fn a_push_on_a_floating_arm_is_read_back() {
    let mut rig = SimRig::boot(0.004, reach());
    let friction = coulomb(&rig);
    let settled = rig.ticks(0.5);
    for _ in 0..settled {
        rig.tick();
    }
    let floating = rig.tick();
    assert!(
        floating.mode == Mode::Idle && floating.gravity_comp,
        "the arm floats under gravity compensation"
    );

    let push = 1.5;
    rig.push(0, push);
    let window = rig.ticks(0.25);
    let mut sum = 0.0;
    let mut samples = 0.0;
    for i in 0..window {
        let s = rig.tick();
        if i >= window / 2 {
            sum += s.tau_ext[0];
            samples += 1.0;
        }
    }
    let moved = rig.tick();
    assert!(
        (moved.q[0] - floating.q[0]) > 0.02,
        "the push moved the base: {:+.4} rad",
        moved.q[0] - floating.q[0]
    );
    let read = sum / samples;
    assert!(
        (read + push).abs() <= friction[0] + 0.25 * push,
        "a {push:+.2} Nm push on the floating base read as {read:+.3} Nm"
    );
}
