//! `stream.lowpass_cutoff_hz`: the command smoothing a streaming session
//! runs its setpoints through.
//!
//! It was a config field with no consumer — declared, validated, and
//! never read, so every cutoff smoothed exactly nothing.

mod common;

use common::{bundle_at, Rig};
use par6_rt::{CompletionPolicy, Mode, RtCommand, StreamSetpoint, ZeroGravity};

const DT: f64 = 0.05;

fn rig_with_cutoff(cutoff_hz: f64) -> Rig {
    let mut bundle = bundle_at(DT);
    bundle.robot.stream.lowpass_cutoff_hz = cutoff_hz;
    let mut rig = Rig::build_bundle(
        bundle,
        CompletionPolicy::Settled,
        Box::new(ZeroGravity),
        true,
    );
    rig.ready();
    rig
}

/// The cutoff must actually filter the command, and must leave `q_target`
/// carrying the raw request so the smoothing stays visible.
///
/// The rig's [`ClampStream`] does no rate limiting of its own, so
/// `q_commanded` is the filter output directly — a step response with
/// nothing else in the way to explain a lag.
#[test]
fn the_command_lowpass_smooths_a_step_and_zero_leaves_it_alone() {
    let step = 0.01;
    let cutoff = 1.0;
    // alpha = dt / (dt + 1/(2*pi*fc)) for a first-order lag.
    let alpha = DT / (DT + 1.0 / (2.0 * std::f64::consts::PI * cutoff));

    let mut filtered = rig_with_cutoff(cutoff);
    let mut plain = rig_with_cutoff(0.0);
    for rig in [&mut filtered, &mut plain] {
        rig.cmd(RtCommand::SetMode(Mode::Stream));
        let mut q = rig.pose;
        q[0] += step;
        rig.handles.stream.send(&StreamSetpoint {
            q,
            ..Default::default()
        });
        rig.tick();
    }

    let (f, p) = (filtered.snap(), plain.snap());
    // Measured q, not the injected pose: the encoder round trip quantises,
    // and the filter is seeded at what the RT actually measured.
    let want = f.q[0] + alpha * (filtered.pose[0] + step - f.q[0]);
    assert!(
        (f.q_commanded[0] - want).abs() < 1e-9,
        "a {cutoff} Hz cutoff must move the command {alpha:.3} of the way \
         ({want:.6} rad), not {:.6}",
        f.q_commanded[0]
    );
    assert!(
        (f.q_target[0] - (filtered.pose[0] + step)).abs() < 1e-12,
        "q_target must carry the RAW request so the filtering is visible"
    );
    assert!(
        (p.q_commanded[0] - (plain.pose[0] + step)).abs() < 1e-6,
        "cutoff 0 means off: the command must follow the request exactly"
    );

    // A filtered stream still converges — the lag is a lag, not an offset.
    let mut q = filtered.pose;
    q[0] += step;
    for _ in 0..200 {
        filtered.handles.stream.send(&StreamSetpoint {
            q,
            ..Default::default()
        });
        filtered.tick();
    }
    assert!(
        (filtered.snap().q_commanded[0] - q[0]).abs() < 1e-6,
        "a held target must be reached, not approached forever"
    );
}

/// A cutoff the tick rate cannot represent must read as OFF, not as a
/// coefficient near 1 that pretends to filter.
#[test]
fn a_cutoff_at_or_above_nyquist_is_reported_as_off() {
    let step = 0.01;
    let mut rig = rig_with_cutoff(1.0 / DT);
    rig.cmd(RtCommand::SetMode(Mode::Stream));
    let mut q = rig.pose;
    q[0] += step;
    rig.handles.stream.send(&StreamSetpoint {
        q,
        ..Default::default()
    });
    rig.tick();
    let s = rig.snap();
    assert!(
        (s.q_commanded[0] - q[0]).abs() < 1e-6,
        "a cutoff above the tick Nyquist filters nothing, so the command \
         must follow the request exactly; it landed {:.6} rad short",
        q[0] - s.q_commanded[0]
    );
}

/// The start-pose gate: a session's FIRST setpoint farther than
/// `stream.start_pose_tol_rad` from the measured pose (worst joint) is
/// dropped un-applied and latches the hard `StreamStartPose` key — the
/// executor must never ramp the arm to wherever a client happened to
/// start publishing. Within tolerance the session admits, and once
/// admitted later setpoints range freely (the watchdog and limiter own
/// them); every NEW session re-arms the check.
#[test]
fn a_first_setpoint_beyond_the_start_tolerance_never_moves_the_arm() {
    use par6_rt::ErrorCode;

    let mut rig = rig_with_cutoff(0.0);
    rig.cmd(RtCommand::SetMode(Mode::Stream));
    // Baseline is the MEASURED pose — the session holds at what the RT
    // decoded, and the encoder round trip quantises `rig.pose`.
    let start = rig.snap().q;
    let mut q = start;
    q[0] += 0.2; // double the shipped 0.1 rad tolerance
    rig.handles.stream.send(&StreamSetpoint {
        q,
        ..Default::default()
    });
    // The tick that consumes the refused setpoint must not move the
    // command at all — an applied one would step the OTG toward the
    // target this very tick.
    let s = rig.snap_after_tick();
    assert_eq!(
        s.mode,
        Mode::Stream,
        "the reaction lands on the NEXT error pass; this tick still \
         publishes the streaming law's command"
    );
    assert!(
        (s.q_commanded[0] - start[0]).abs() < 1e-9,
        "the refused setpoint must never move the command: {} vs {}",
        s.q_commanded[0],
        start[0]
    );
    for _ in 0..20 {
        rig.tick();
    }
    let s = rig.snap();
    assert!(
        s.errors
            .as_slice()
            .iter()
            .any(|e| e.code == ErrorCode::StreamStartPose && e.joint == Some(0)),
        "the refusal must latch the dedicated key on the worst joint: {:?}",
        s.errors.as_slice()
    );
    assert_eq!(s.mode, Mode::ActiveError, "a hard latch reacts");

    // Within tolerance the session admits, and the gate is first-only:
    // an in-session jump twice the tolerance is the limiter's business.
    let mut rig = rig_with_cutoff(0.0);
    let start = rig.pose;
    rig.cmd(RtCommand::SetMode(Mode::Stream));
    let mut q = start;
    q[0] += 0.05;
    rig.handles.stream.send(&StreamSetpoint {
        q,
        ..Default::default()
    });
    rig.tick();
    assert!(
        (rig.snap().q_target[0] - q[0]).abs() < 1e-12,
        "within tolerance the first setpoint applies"
    );
    q[0] += 0.2;
    rig.handles.stream.send(&StreamSetpoint {
        q,
        ..Default::default()
    });
    rig.tick();
    let s = rig.snap();
    assert!(
        (s.q_target[0] - q[0]).abs() < 1e-12,
        "in-session setpoints are not start-gated"
    );
    assert!(!s.error_active, "no latch from an admitted session");

    // A NEW session re-arms the gate.
    rig.cmd(RtCommand::SetMode(Mode::Idle));
    rig.cmd(RtCommand::SetMode(Mode::Stream));
    let mut q = rig.snap().q;
    q[1] += 0.2;
    rig.handles.stream.send(&StreamSetpoint {
        q,
        ..Default::default()
    });
    for _ in 0..20 {
        rig.tick();
    }
    assert!(
        rig.snap()
            .errors
            .as_slice()
            .iter()
            .any(|e| e.code == ErrorCode::StreamStartPose && e.joint == Some(1)),
        "re-entering STREAM re-arms the first-setpoint check"
    );
}
