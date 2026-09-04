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

/// The core half of the limiter-fault contract: while the tracker
/// reports `faulted()`, STREAM mode hard-latches `StreamFault` and the
/// reaction lands. The counting side (round(fault_latch_s / dt)
/// consecutive failures, one log per streak) is pinned from below by
/// the MotionStream adapter's own tests.
#[test]
fn a_faulted_tracker_hard_latches_stream_fault() {
    use par6_rt::hooks::{ClampStream, StreamTracker};
    use par6_rt::{ErrorCode, MAX_JOINTS};

    /// The real clamp tracker, reporting its limiter dead — the one-line
    /// oracle for the core's reaction.
    struct FaultyClamp(ClampStream);
    impl StreamTracker for FaultyClamp {
        fn activate(&mut self, q_meas: &[f64; MAX_JOINTS]) {
            self.0.activate(q_meas);
        }
        fn set_target(&mut self, q_target: &[f64; MAX_JOINTS]) {
            self.0.set_target(q_target);
        }
        fn set_scale(&mut self, speed: f64, accel: f64) {
            self.0.set_scale(speed, accel);
        }
        fn step(&mut self, q_out: &mut [f64; MAX_JOINTS], qd_out: &mut [f64; MAX_JOINTS]) {
            self.0.step(q_out, qd_out);
        }
        fn faulted(&self) -> bool {
            true
        }
    }

    let bundle = bundle_at(DT);
    let tracker = FaultyClamp(ClampStream::new(&bundle.robot));
    let mut rig = Rig::build_bundle_with_stream(
        bundle,
        CompletionPolicy::Settled,
        Box::new(ZeroGravity),
        true,
        Some(Box::new(tracker)),
    );
    rig.ready();
    rig.cmd(RtCommand::SetMode(Mode::Stream));
    rig.tick_n(10);
    let s = rig.snap();
    assert!(
        s.errors
            .as_slice()
            .iter()
            .any(|e| e.code == ErrorCode::StreamFault),
        "a dead limiter must latch the dedicated hard key: {:?}",
        s.errors.as_slice()
    );
    assert_eq!(s.mode, Mode::ActiveError, "the hard latch reacts");
}

/// The low-pass is a per-tick coefficient, so the command keeps
/// converging between setpoints: a publisher slower than the tick rate
/// gets the configured cutoff, not one scaled down by its own cadence.
#[test]
fn the_command_lowpass_converges_between_setpoints() {
    let step = 0.01;
    let cutoff = 1.0;
    let alpha = DT / (DT + 1.0 / (2.0 * std::f64::consts::PI * cutoff));

    let mut bundle = bundle_at(DT);
    bundle.robot.stream.lowpass_cutoff_hz = cutoff;
    // Long enough that publishing every fourth tick never trips the
    // stream watchdog.
    bundle.robot.stream.command_timeout_s = 1.0;
    let mut rig = Rig::build_bundle(
        bundle,
        CompletionPolicy::Settled,
        Box::new(ZeroGravity),
        true,
    );
    rig.ready();
    rig.cmd(RtCommand::SetMode(Mode::Stream));
    let mut q = rig.pose;
    q[0] += step;
    let ticks: i32 = 40;
    for t in 0..ticks {
        if t % 4 == 0 {
            rig.handles.stream.send(&StreamSetpoint {
                q,
                ..Default::default()
            });
        }
        rig.tick();
    }
    assert_eq!(
        rig.snap().mode,
        Mode::Stream,
        "the session must still be alive"
    );

    let residual = (q[0] - rig.snap().q_commanded[0]).abs();
    // Stepping only on receipt would leave (1-α)^10 of the step; stepping
    // every tick leaves (1-α)^40.
    let per_tick = step * (1.0 - alpha).powi(ticks - 1);
    let per_receipt = step * (1.0 - alpha).powi(ticks / 4);
    assert!(
        residual < per_tick * 2.0,
        "residual {residual:.3e} — per-tick stepping would leave ~{per_tick:.3e}, \
         per-receipt stepping ~{per_receipt:.3e}"
    );
}
