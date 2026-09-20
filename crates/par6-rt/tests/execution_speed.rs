//! Execution-rate changes through the real core and driver output path,
//! driven by virtual ticks. A single-axis inertia is the torque oracle.

mod common;

use common::{bundle, Rig};
use par6_config::LimitMode;
use par6_rt::{
    CompletionPolicy, Mode, RtCommand, Sample, SampleMeta, SampleStart, StateSnapshot, ZeroGravity,
};

const INERTIA: f64 = 0.05;

fn push_path(rig: &mut Rig, index: u32, distance: f64, duration: f64) -> usize {
    let intervals = (duration / rig.dt).round() as usize;
    let duration = intervals as f64 * rig.dt;
    let start = rig.snap().q_target;
    for k in 0..=intervals {
        let u = k as f64 / intervals as f64;
        let mut sample = Sample {
            q: start,
            meta: SampleMeta {
                command_index: index,
                is_last: k == intervals,
                ..SampleMeta::default()
            },
            ..Sample::default()
        };
        sample.q[0] += distance * (10.0 * u.powi(3) - 15.0 * u.powi(4) + 6.0 * u.powi(5));
        sample.qd[0] =
            distance * (30.0 * u.powi(2) - 60.0 * u.powi(3) + 30.0 * u.powi(4)) / duration;
        let acceleration =
            distance * (60.0 * u - 180.0 * u.powi(2) + 120.0 * u.powi(3)) / duration.powi(2);
        sample.tau_ff[0] = (INERTIA * acceleration) as f32;
        sample.inertia_velocity[0] = (INERTIA * sample.qd[0]) as f32;
        assert!(rig.producer.try_push(&sample));
    }
    intervals + 1
}

fn tick(rig: &mut Rig) -> StateSnapshot {
    rig.handles.heartbeat.feed();
    rig.tick();
    let state = rig.snap();
    assert!(
        !state.error_active,
        "unexpected runtime error: {:?}",
        state.errors
    );
    state
}

fn inertia_torque(position: f64, scale: f64, rate: f64) -> f64 {
    // Invert the known path from the observed target, independently of the
    // playback cursor, then apply the chain rule to the inertia model.
    let mut low: f64 = 0.0;
    let mut high: f64 = 1.0;
    for _ in 0..40 {
        let u = 0.5 * (low + high);
        let q = 0.25 * (10.0 * u.powi(3) - 15.0 * u.powi(4) + 6.0 * u.powi(5));
        if q < position {
            low = u;
        } else {
            high = u;
        }
    }
    let u = 0.5 * (low + high);
    let velocity = 0.25 * (30.0 * u.powi(2) - 60.0 * u.powi(3) + 30.0 * u.powi(4)) / 0.4;
    let acceleration = 0.25 * (60.0 * u - 180.0 * u.powi(2) + 120.0 * u.powi(3)) / 0.4_f64.powi(2);
    INERTIA * (scale * scale * acceleration + rate * velocity)
}

#[test]
fn fixed_execution_rates_retime_the_same_path_and_scale_feedforward() {
    for scale in [1.0, 0.6, 0.5, 0.1] {
        let mut rig = Rig::with_policy(CompletionPolicy::Commanded);
        rig.ready();
        rig.cmd(RtCommand::ExecSetSpeedScale(scale));
        rig.cmd(RtCommand::SetMode(Mode::Exec));
        let start = rig.snap().q_target[0];
        let samples = push_path(&mut rig, 1, 0.1, 0.4);
        let expected_ticks = (samples as f64 / scale).ceil() as usize;
        let mut previous_velocity = 0.0;
        let mut complete_at = None;
        for k in 1..=expected_ticks + 3 {
            let state = tick(&mut rig);
            assert_eq!(state.exec.applied_scale, scale);
            let acceleration = (state.qd_target[0] - previous_velocity) / rig.dt;
            assert!(
                (state.tau_commanded[0] - INERTIA * acceleration).abs() < 0.012,
                "scale {scale}, tick {k}: torque {} inconsistent with acceleration {acceleration}",
                state.tau_commanded[0]
            );
            previous_velocity = state.qd_target[0];
            if state.exec.completed_index == 1 {
                complete_at = Some(k);
                assert!((state.q_target[0] - start - 0.1).abs() < 1e-10);
                break;
            }
        }
        let actual_ticks = complete_at.expect("retimed command completes");
        assert!(
            actual_ticks.abs_diff(expected_ticks) <= 2,
            "{scale}: {actual_ticks} vs {expected_ticks}"
        );
    }
}

#[test]
fn changing_speed_does_not_introduce_a_jerk_spike_on_a_smooth_path() {
    let config = bundle();
    let jerk_limit = config.robot.joints[0]
        .limits
        .for_mode(LimitMode::Exec)
        .jerk_rad_s3
        .expect("configured jerk limit");
    let mut rig = Rig::with_policy(CompletionPolicy::Commanded);
    rig.ready();
    rig.cmd(RtCommand::SetMode(Mode::Exec));
    // This quintic's maximum nominal jerk is 6 rad/s^3, below the
    // configured limit. A speed change must not add an acceleration jump.
    push_path(&mut rig, 1, 0.1, 1.0);
    let mut velocity = 0.0;
    let mut acceleration = 0.0;
    for k in 0..(3.0 / rig.dt).round() as usize {
        if k == (0.3 / rig.dt).round() as usize {
            rig.send(RtCommand::ExecSetSpeedScale(0.1));
        }
        let state = tick(&mut rig);
        let next_acceleration = (state.qd_target[0] - velocity) / rig.dt;
        let jerk = (next_acceleration - acceleration) / rig.dt;
        assert!(
            jerk.abs() <= jerk_limit * 1.01,
            "tick {k}: execution override introduced jerk {jerk}, limit {jerk_limit}"
        );
        velocity = state.qd_target[0];
        acceleration = next_acceleration;
        if state.exec.completed_index == 1 {
            break;
        }
    }
}

#[test]
fn rate_transitions_pause_resume_and_flush_preserve_motion_derivatives() {
    let mut config = bundle();
    config.robot.motion.execution_override_transition_s = 0.03;
    let acceleration_limit = config.robot.joints[0]
        .limits
        .for_mode(LimitMode::Exec)
        .acceleration_rad_s2;
    let mut rig = Rig::build_bundle(
        config,
        CompletionPolicy::Commanded,
        Box::new(ZeroGravity),
        true,
    );
    rig.ready();
    rig.cmd(RtCommand::SetMode(Mode::Exec));
    let start = rig.snap().q_target[0];
    push_path(&mut rig, 1, 0.25, 0.4);
    let mut previous = rig.snap();
    let mut paused_at = None;
    let mut held_state = None;
    let mut complete = false;
    for k in 0..2000 {
        if k == 40 {
            rig.send(RtCommand::ExecSetSpeedScale(0.1));
        }
        if k == 120 {
            rig.send(RtCommand::ExecSetPaused(true));
        }
        if paused_at.is_some_and(|at| k == at + 50) {
            rig.send(RtCommand::ExecSetSpeedScale(0.6));
        }
        if paused_at.is_some_and(|at| k == at + 100) {
            rig.send(RtCommand::ExecSetPaused(false));
        }
        let state = tick(&mut rig);
        if paused_at.is_some_and(|at| k >= at + 50 && k < at + 100) {
            assert!(state.exec.paused, "selecting a speed must preserve pause");
            assert_eq!(state.exec.resume_scale, 0.6);
        }
        let acceleration = (state.qd_target[0] - previous.qd_target[0]) / rig.dt;
        assert!(
            acceleration.abs() <= acceleration_limit * 1.01,
            "tick {k}: acceleration {acceleration} > {acceleration_limit}, scale {} -> {}",
            previous.exec.applied_scale,
            state.exec.applied_scale
        );
        let displacement = state.q_target[0] - previous.q_target[0];
        let integrated_velocity = 0.5 * (state.qd_target[0] + previous.qd_target[0]) * rig.dt;
        assert!(
            (displacement - integrated_velocity).abs() < 2e-5,
            "tick {k}: position/velocity disagreement {displacement} vs {integrated_velocity}"
        );
        let expected_torque = inertia_torque(
            state.q_target[0] - start,
            state.exec.applied_scale,
            (state.exec.applied_scale - previous.exec.applied_scale) / rig.dt,
        );
        assert!(
            (state.tau_commanded[0] - expected_torque).abs() < 0.0002,
            "tick {k}: torque {} vs inertia oracle {expected_torque}",
            state.tau_commanded[0]
        );
        if state.exec.paused {
            if let Some(held) = &held_state {
                let held: &StateSnapshot = held;
                assert_eq!(state.q_target, held.q_target);
                assert_eq!(state.exec.samples_remaining, held.exec.samples_remaining);
                assert_eq!(state.exec.completed_index, 0);
            } else {
                paused_at = Some(k);
                held_state = Some(state);
            }
        }
        if state.exec.completed_index == 1 {
            assert!(paused_at.is_some(), "completed without exercising pause");
            assert!((state.q_target[0] - start - 0.25).abs() < 1e-10);
            complete = true;
            break;
        }
        previous = state;
    }
    assert!(complete, "resumed command must finish");

    rig.pose[0] = start + 0.25;
    push_path(&mut rig, 2, -0.05, 0.4);
    for _ in 0..40 {
        tick(&mut rig);
    }
    let before = rig.snap();
    rig.send(RtCommand::ExecFlush);
    let after = tick(&mut rig);
    assert!(
        before.exec.samples_remaining - after.exec.samples_remaining <= 1,
        "unmarked flush does not discard queued motion"
    );
    rig.producer.flush_marker().mark();
    rig.send(RtCommand::ExecFlush);
    let stopped = tick(&mut rig);
    assert_eq!(stopped.exec.samples_remaining, 0);
    assert_ne!(
        stopped.exec.completed_index, 2,
        "Stop cannot report completion"
    );
    for _ in 0..100 {
        assert_eq!(tick(&mut rig).q_target, stopped.q_target);
    }
}

#[test]
fn measured_plan_start_does_not_create_an_unplanned_velocity_bridge() {
    let mut rig = Rig::with_policy(CompletionPolicy::Commanded);
    rig.ready();
    rig.cmd(RtCommand::ExecSetSpeedScale(0.5));
    rig.cmd(RtCommand::SetMode(Mode::Exec));
    let mut planned_start = rig.snap().q_target;
    // A planner may start from measured feedback after a previous command's
    // completion tolerance leaves a small residual from its held target.
    planned_start[0] += 0.00005;
    for k in 0..3 {
        assert!(rig.producer.try_push(&Sample {
            q: planned_start,
            meta: SampleMeta {
                command_index: 1,
                is_last: k == 2,
                ..SampleMeta::default()
            },
            ..Sample::default()
        }));
    }
    let state = tick(&mut rig);
    assert_eq!(state.q_target, planned_start);
    assert_eq!(state.qd_target, [0.0; 6]);
    assert_eq!(state.tau_commanded, [0.0; 6]);
}

#[test]
fn the_first_moving_sample_interpolates_from_the_planned_start() {
    let mut rig = Rig::with_policy(CompletionPolicy::Commanded);
    rig.ready();
    rig.cmd(RtCommand::ExecSetSpeedScale(0.5));
    rig.cmd(RtCommand::SetMode(Mode::Exec));
    let start = rig.snap().q_target;
    for k in 1..=5 {
        let t = k as f64 * rig.dt;
        let mut sample = Sample {
            q: start,
            meta: SampleMeta {
                command_index: 1,
                is_last: k == 5,
                ..SampleMeta::default()
            },
            start: (k == 1).then_some(SampleStart {
                q: start,
                tau_ff: [INERTIA as f32, 0.0, 0.0, 0.0, 0.0, 0.0],
            }),
            ..Sample::default()
        };
        sample.q[0] += 0.5 * t * t;
        sample.qd[0] = t;
        sample.tau_ff[0] = INERTIA as f32;
        sample.inertia_velocity[0] = (INERTIA * t) as f32;
        assert!(rig.producer.try_push(&sample));
    }
    let state = tick(&mut rig);
    let t = 0.5 * rig.dt;
    assert!((state.q_target[0] - start[0] - 0.5 * t * t).abs() < 1e-12);
    assert!((state.qd_target[0] - t * 0.5).abs() < 1e-10);
    assert!((state.tau_commanded[0] - INERTIA * 0.25).abs() < 1e-8);
}
