//! `stream.lowpass_cutoff_hz`: the command smoothing a streaming session
//! runs its setpoints through.
//!
//! It was a config field with no consumer — declared, validated, and
//! never read, so every cutoff smoothed exactly nothing.

mod common;

use common::{bundle_at, Rig};
use par6_rt::hooks::ClampStream;
use par6_rt::{CompletionPolicy, Mode, RtCommand, StreamSetpoint, ZeroGravity, MAX_JOINTS};

const DT: f64 = 0.05;

/// A rig whose free stream runs through [`ClampStream`] — the
/// passthrough tracker the core uses for shaped streams — so
/// `q_commanded` is the filter output itself, with no rate limiter's lag
/// in the way.
fn rig_with_cutoff(cutoff_hz: f64, command_timeout_s: f64) -> Rig {
    let mut bundle = bundle_at(DT);
    bundle.robot.stream.lowpass_cutoff_hz = cutoff_hz;
    bundle.robot.stream.command_timeout_s = command_timeout_s;
    let clamp = ClampStream::new(&bundle.robot);
    let mut rig = Rig::build_bundle_with_stream(
        bundle,
        CompletionPolicy::Settled,
        Box::new(ZeroGravity),
        true,
        Some(Box::new(clamp)),
    );
    rig.ready();
    rig.cmd(RtCommand::SetMode(Mode::Stream));
    rig
}

fn send(rig: &mut Rig, q: [f64; MAX_JOINTS]) {
    rig.handles.stream.send(&StreamSetpoint {
        q,
        ..Default::default()
    });
}

/// The cutoff filters the command as a per-tick first-order lag, leaves
/// `q_target` carrying the raw request so the smoothing stays visible,
/// keeps converging between setpoints, and reads as OFF at zero and at
/// any cutoff the tick rate cannot represent.
#[test]
fn the_command_lowpass_filters_every_tick_and_is_off_where_it_cannot_filter() {
    let step = 0.01;
    let cutoff = 1.0;
    // alpha = dt / (dt + 1/(2*pi*fc)) for a first-order lag.
    let alpha = DT / (DT + 1.0 / (2.0 * std::f64::consts::PI * cutoff));

    let mut rig = rig_with_cutoff(cutoff, 1.0);
    let mut q = rig.pose;
    q[0] += step;
    send(&mut rig, q);
    rig.tick();
    let s = rig.snap();
    // Measured q, not the injected pose: the encoder round trip quantises,
    // and the filter is seeded at what the RT actually measured.
    let want = s.q[0] + alpha * (q[0] - s.q[0]);
    assert!(
        (s.q_commanded[0] - want).abs() < 1e-9,
        "a {cutoff} Hz cutoff must move the command {alpha:.3} of the way \
         ({want:.6} rad), not {:.6}",
        s.q_commanded[0]
    );
    assert!(
        (s.q_target[0] - q[0]).abs() < 1e-12,
        "q_target must carry the RAW request so the filtering is visible"
    );

    // Publishing every fourth tick: the filter steps every tick, so the
    // residual is (1-α)^ticks of the step, not (1-α)^receipts.
    let ticks: i32 = 40;
    for t in 1..ticks {
        if t % 4 == 0 {
            send(&mut rig, q);
        }
        rig.tick();
    }
    assert_eq!(rig.snap().mode, Mode::Stream, "the session stays alive");
    let residual = (q[0] - rig.snap().q_commanded[0]).abs();
    let per_tick = step * (1.0 - alpha).powi(ticks - 1);
    let per_receipt = step * (1.0 - alpha).powi(ticks / 4);
    assert!(
        residual < per_tick * 2.0,
        "residual {residual:.3e} — per-tick stepping would leave ~{per_tick:.3e}, \
         per-receipt stepping ~{per_receipt:.3e}"
    );
    for _ in 0..200 {
        send(&mut rig, q);
        rig.tick();
    }
    assert!(
        (rig.snap().q_commanded[0] - q[0]).abs() < 1e-6,
        "a held target is reached, not approached forever"
    );

    // Off: zero, exactly Nyquist, and above it.
    for off in [0.0, 0.5 / DT, 1.0 / DT] {
        let mut rig = rig_with_cutoff(off, 1.0);
        let mut q = rig.pose;
        q[0] += step;
        send(&mut rig, q);
        rig.tick();
        let s = rig.snap();
        assert!(
            (s.q_commanded[0] - q[0]).abs() < 1e-6,
            "a {off} Hz cutoff filters nothing, so the command follows the request; \
             it landed {:.6} rad short",
            q[0] - s.q_commanded[0]
        );
    }
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
        fn set_scale_per_joint(&mut self, speed: &[f64; MAX_JOINTS], accel: f64) {
            self.0.set_scale_per_joint(speed, accel);
        }
        fn set_bounds(&mut self, min: &[f64; MAX_JOINTS], max: &[f64; MAX_JOINTS]) {
            self.0.set_bounds(min, max);
        }
        fn step(&mut self, q_out: &mut [f64; MAX_JOINTS], qd_out: &mut [f64; MAX_JOINTS]) {
            self.0.step(q_out, qd_out);
        }
        fn release(&mut self) {
            self.0.release();
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

/// Liveness telemetry is not position feedback. Losing just one joint's
/// encoder updates must not turn a cached pose into a completed release.
#[test]
fn telemetry_without_encoder_updates_cannot_complete_a_stream_release() {
    for dt in [0.004, 0.02] {
        let mut rig = Rig::at_tick_dt(dt);
        rig.ready();
        rig.cmd(RtCommand::SetMode(Mode::Stream));
        let node = rig.node_of[0];
        rig.skip_nodes = 1 << node;
        rig.send(RtCommand::StreamRelease);
        for _ in 0..(0.24_f64 / dt).round() as u32 {
            rig.core
                .bus_mut()
                .inject(false, par6_bus::Reply::Voltage { node, mv: 24000 });
            rig.tick();
            let s = rig.snap();
            assert_eq!(
                s.node_freshness[0],
                par6_bus::Freshness::Fresh,
                "the drive still answers telemetry"
            );
            assert_eq!(s.mode, Mode::Stream,
                "dt {dt}: cached encoder positions completed a release without new position feedback");
            assert!(
                !s.error_active,
                "live telemetry must keep this distinct from CAN loss"
            );
        }
        rig.skip_nodes = 0;
        for _ in 0..(0.4_f64 / dt).round() as u32 {
            rig.tick();
            if rig.snap().mode == Mode::Exec {
                break;
            }
        }
        assert_eq!(
            rig.snap().mode,
            Mode::Exec,
            "fresh, stationary encoder replies must let the release complete into the hold"
        );
    }
}
