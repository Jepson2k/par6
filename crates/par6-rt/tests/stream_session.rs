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
