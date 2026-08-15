//! EXEC ring playback through the full core: all three completion
//! policies, the blend-continues bypass, the strict-timeout error, pause
//! semantics, and the exec link watchdog.

mod common;

use common::Rig;
use par6_rt::{
    ArmState, CompletionPolicy, ErrorCode, Mode, RtCommand, Sample, SampleMeta, MAX_JOINTS,
};

/// Push `n` samples for command `index` ramping J0 from `from` by `step`
/// per sample; `blend` marks every sample's segment as blending onward;
/// `last` marks the final sample as the program end.
fn push_cmd(rig: &mut Rig, index: u32, from: f64, step: f64, n: usize, blend: bool, last: bool) {
    let mut q = rig.pose;
    for k in 0..n {
        q[0] = from + step * (k + 1) as f64;
        let s = Sample {
            q,
            qd: [0.0; MAX_JOINTS],
            tau_ff: [0.0; MAX_JOINTS],
            meta: SampleMeta {
                command_index: index,
                checkpoint_id: index,
                blend_continues: blend,
                is_last: last && k == n - 1,
            },
        };
        assert!(rig.producer.try_push(&s), "ring capacity");
    }
}

fn enter_exec(rig: &mut Rig) {
    rig.ready();
    rig.cmd(RtCommand::SetMode(Mode::Exec));
    rig.tick();
    assert_eq!(rig.snap().mode, Mode::Exec);
}

/// J0 position commands sent on the bus since `tick`, in motor ticks.
fn j0_positions(rig: &mut Rig, since: u64) -> Vec<i32> {
    rig.joints_since(since)
        .iter()
        .map(|(_, f)| f[0].pos.expect("EXEC frames carry positions"))
        .collect()
}

#[test]
fn playback_pops_one_sample_per_tick_and_holds_when_starved() {
    let mut rig = Rig::new();
    enter_exec(&mut rig);
    let q0 = rig.pose[0];
    push_cmd(&mut rig, 1, q0, 0.001, 10, false, false);
    let start = rig.snap().tick;
    rig.tick_n(15);

    let pos = j0_positions(&mut rig, start + 1);
    let expected: Vec<i32> = (1..=10)
        .map(|k| rig.conv[0].motor_ticks(q0 + 0.001 * k as f64))
        .collect();
    assert_eq!(&pos[..10], &expected[..], "one sample per tick, in order");
    // Starved: holds the last target (no boundary was signalled — the
    // command has no is_last and no successor yet).
    assert!(
        pos[10..].iter().all(|p| *p == expected[9]),
        "hold on starve"
    );
    assert_eq!(rig.snap().exec.samples_remaining, 0);
    assert_eq!(
        rig.snap().exec.completed_index,
        0,
        "not complete: no boundary"
    );
}

#[test]
fn commanded_policy_completes_at_the_last_sample_without_holding() {
    let mut rig = Rig::with_policy(CompletionPolicy::Commanded);
    enter_exec(&mut rig);
    let q0 = rig.pose[0];
    // Target far from the measured pose: a settling policy would hold
    // here; commanded must not.
    push_cmd(&mut rig, 1, q0, 0.02, 5, false, false);
    push_cmd(&mut rig, 2, q0 + 0.1, 0.02, 5, false, true);
    let start = rig.snap().tick;
    rig.tick_n(13);

    let pos = j0_positions(&mut rig, start + 1);
    // 5 + 5 samples play back-to-back with NO hold tick at the 1→2
    // boundary — commanded completes at the last sample even though the
    // measured pose is far away.
    let expected: Vec<i32> = (1..=10)
        .map(|k| rig.conv[0].motor_ticks(q0 + 0.02 * k as f64))
        .collect();
    assert_eq!(&pos[..10], &expected[..], "continuous playback");
    let s = rig.snap();
    assert_eq!(s.exec.completed_index, 2);
    assert!(!s.error_active);
}

#[test]
fn settled_policy_holds_until_tracking_then_resumes() {
    let mut rig = Rig::new(); // Settled is the default policy
    enter_exec(&mut rig);
    let q0 = rig.pose[0];
    let target = q0 + 0.05;
    push_cmd(&mut rig, 1, q0, 0.01, 5, false, false);
    // Command 2 holds at the same target so its own final settle can
    // complete once the measured pose tracks.
    push_cmd(&mut rig, 2, target, 0.0, 5, false, true);
    rig.tick_n(8);

    // Boundary reached with the measured pose 0.05 rad off: settling.
    let s = rig.snap();
    assert!(s.exec.settling, "holding at the boundary");
    assert_eq!(s.exec.completed_index, 0);
    let hold = rig.last_joints()[0].pos.unwrap();
    assert_eq!(
        hold,
        rig.conv[0].motor_ticks(target),
        "holds the boundary target"
    );
    rig.tick_n(20);
    assert_eq!(
        rig.last_joints()[0].pos.unwrap(),
        hold,
        "still holding while off-target"
    );

    // The measured pose converges within the 0.01 rad tolerance:
    // completion, then command 2 plays.
    rig.pose[0] = target;
    rig.tick_n(10);
    let s = rig.snap();
    assert!(!s.exec.settling);
    assert_eq!(s.exec.completed_index, 2, "cmd 2 finished (tracking pose)");
    assert!(!s.error_active);
}

#[test]
fn settled_timeout_completes_without_error_strict_timeout_latches() {
    // Settled: 500-tick timeout then complete anyway.
    let mut rig = Rig::new();
    enter_exec(&mut rig);
    let q0 = rig.pose[0];
    push_cmd(&mut rig, 1, q0 + 0.05, 0.01, 3, false, true);
    rig.tick_n(4 + 500 + 5);
    let s = rig.snap();
    assert_eq!(s.exec.completed_index, 1, "timeout completes under settled");
    assert!(!s.error_active, "settled timeout is NOT an error");
    assert_eq!(s.mode, Mode::Exec);

    // Strict: the same timeout is a hard error.
    let mut rig = Rig::with_policy(CompletionPolicy::Strict);
    enter_exec(&mut rig);
    let q0 = rig.pose[0];
    push_cmd(&mut rig, 1, q0 + 0.05, 0.01, 3, false, true);
    rig.tick_n(4 + 500 + 5);
    let s = rig.snap();
    assert!(
        s.errors
            .as_slice()
            .iter()
            .any(|e| e.code == ErrorCode::ExecSettleTimeout),
        "strict timeout latches"
    );
    assert!(s.error_active);
    assert_eq!(s.mode, Mode::ActiveError);
    assert_eq!(s.state, ArmState::Disabled);
    assert_eq!(s.exec.completed_index, 0, "the command did NOT complete");
    // The reaction output law took over: active zero-velocity frames.
    let f = rig.last_joints();
    assert!(f.iter().all(|c| c.pos.is_none() && c.vel == Some(0)));
}

#[test]
fn blend_continues_bypasses_settling_across_the_boundary() {
    let mut rig = Rig::new(); // Settled policy — the bypass must win
    enter_exec(&mut rig);
    let q0 = rig.pose[0];
    // Command 1 blends into command 2; targets far from measured, so
    // only the bypass can keep motion continuous.
    push_cmd(&mut rig, 1, q0, 0.02, 5, true, false);
    push_cmd(&mut rig, 2, q0 + 0.1, 0.02, 5, false, true);
    let start = rig.snap().tick;
    rig.tick_n(12);

    let pos = j0_positions(&mut rig, start + 1);
    let expected: Vec<i32> = (1..=10)
        .map(|k| rig.conv[0].motor_ticks(q0 + 0.02 * k as f64))
        .collect();
    // No hold tick at the 1→2 boundary: blended corners stay
    // velocity-continuous even under the settled policy.
    assert_eq!(&pos[..10], &expected[..], "continuous through the blend");
    let s = rig.snap();
    assert_eq!(
        s.exec.completed_index, 1,
        "cmd 1 completed via blend bypass"
    );
    // The final (non-blended) boundary settles normally — measured is
    // far off, so playback is holding there.
    assert!(s.exec.settling, "final boundary settles under the policy");
}

#[test]
fn pause_holds_in_place_with_the_ring_untouched() {
    let mut rig = Rig::new();
    enter_exec(&mut rig);
    let q0 = rig.pose[0];
    push_cmd(&mut rig, 1, q0, 0.001, 100, false, false);
    rig.tick_n(10);
    let before = rig.snap().exec.samples_remaining;
    let held = rig.last_joints()[0].pos.unwrap();

    rig.cmd(RtCommand::ExecSetPaused(true));
    rig.tick_n(30);
    let s = rig.snap();
    assert!(s.exec.paused);
    assert_eq!(
        s.exec.samples_remaining, before,
        "ring untouched while paused"
    );
    assert_eq!(rig.last_joints()[0].pos.unwrap(), held, "holds in place");
    assert_eq!(rig.last_joints()[0].vel, Some(0), "zero velocity hold");

    rig.cmd(RtCommand::ExecSetPaused(false));
    rig.tick_n(5);
    assert!(
        rig.snap().exec.samples_remaining < before,
        "playback resumed"
    );

    // Flush (stop path) discards the queue — unlike pause. The bound
    // rides the ring: an unmarked flush leaves the backlog alone (the
    // tick still plays its own sample), a marked one drops all of it.
    let queued = rig.snap().exec.samples_remaining;
    assert!(queued > 1, "backlog needed to tell the two apart");
    rig.cmd(RtCommand::ExecFlush); // one tick: the command, and one sample played
    assert_eq!(
        rig.snap().exec.samples_remaining,
        queued - 1,
        "an unmarked flush must not discard the backlog"
    );
    rig.producer.flush_marker().mark();
    rig.cmd(RtCommand::ExecFlush);
    assert_eq!(rig.snap().exec.samples_remaining, 0);
}

#[test]
fn exec_link_watchdog_latches_after_heartbeat_silence_with_samples_pending() {
    let mut rig = Rig::new();
    enter_exec(&mut rig);
    let q0 = rig.pose[0];
    push_cmd(&mut rig, 1, q0, 0.0001, 2000, false, false);

    // Fed heartbeat: no error while playing.
    for _ in 0..200 {
        rig.handles.heartbeat.feed();
        rig.tick();
    }
    assert!(!rig.snap().error_active, "heartbeat keeps the link alive");

    // 0.5 s of silence while samples are pending: EXEC_LINK_LOST latches.
    rig.tick_n(130);
    let s = rig.snap();
    assert!(
        s.errors
            .as_slice()
            .iter()
            .any(|e| e.code == ErrorCode::ExecLinkLost),
        "link watchdog latched"
    );
    assert!(s.error_active);
    assert_eq!(s.mode, Mode::ActiveError);
}
