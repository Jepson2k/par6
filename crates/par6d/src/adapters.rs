//! The real `par6-motion` engines behind the `par6-rt` per-tick hook
//! traits — thin lifecycle mappings, no behavior of their own.

use par6_motion::{JogDirection, MotionLimits, StreamStep, StreamingExecutor};
use par6_rt::{JogEngine as RtJogEngine, StreamTracker, MAX_JOINTS};

/// `par6_motion::JogEngine` (jerk-aware lookahead, direction-block
/// latching) behind the RT jog hook.
pub struct MotionJog {
    engine: par6_motion::JogEngine,
    /// Configured ramp time, so a fraction always scales the config value
    /// rather than compounding on the last scaled one.
    base_accel_time_s: f64,
}

impl MotionJog {
    /// Wrap a configured jog engine running the config's `accel_time_s`.
    pub fn new(engine: par6_motion::JogEngine, base_accel_time_s: f64) -> Self {
        Self {
            engine,
            base_accel_time_s,
        }
    }
}

impl RtJogEngine for MotionJog {
    fn activate(&mut self, q_meas: &[f64; MAX_JOINTS]) {
        self.engine.activate(q_meas);
    }

    fn command(&mut self, speeds: &[f64; MAX_JOINTS]) {
        let mut clean = [0.0; MAX_JOINTS];
        for (out, v) in clean.iter_mut().zip(speeds.iter()) {
            *out = if v.is_finite() {
                v.clamp(-1.0, 1.0)
            } else {
                0.0
            };
        }
        if clean.iter().all(|v| *v == 0.0) {
            self.engine.release();
            return;
        }
        if let Err(e) = self.engine.command(&clean) {
            log::warn!("jog command refused: {e}");
        }
    }

    /// A jog asked to accelerate at a fraction of the configured rate
    /// takes proportionally longer to ramp: the engine's ramp
    /// acceleration is `v_full / accel_time_s`, so dividing the time by
    /// the fraction scales the acceleration by it.
    fn set_accel_scale(&mut self, accel: f64) {
        if let Err(e) = self.engine.set_accel_time_s(self.base_accel_time_s / accel) {
            log::warn!("jog accel scale {accel} refused: {e}");
        }
    }

    fn release(&mut self) {
        self.engine.release();
    }

    fn tick(
        &mut self,
        q_meas: &[f64; MAX_JOINTS],
        q_out: &mut [f64; MAX_JOINTS],
        qd_out: &mut [f64; MAX_JOINTS],
    ) -> u16 {
        let t = self.engine.tick(q_meas);
        *q_out = t.q;
        *qd_out = t.qd;
        let mut mask = 0u16;
        for j in 0..MAX_JOINTS {
            match self.engine.blocked_direction(j) {
                Some(JogDirection::Negative) => mask |= 1 << (2 * j),
                Some(JogDirection::Positive) => mask |= 2 << (2 * j),
                None => {}
            }
        }
        mask
    }
}

/// `par6_motion::StreamingExecutor` (jerk-limited OTG) behind the RT
/// stream hook, with the vendor's unconditional soft-limit clamp on the
/// way in and out.
pub struct MotionStream {
    executor: StreamingExecutor,
    dt: f64,
    soft_min: [f64; MAX_JOINTS],
    soft_max: [f64; MAX_JOINTS],
    hold_q: [f64; MAX_JOINTS],
    /// Unscaled STREAM ceilings, kept so a fraction always scales the
    /// configured limit rather than the last scaled one.
    base: MotionLimits,
    /// Consecutive `step()` failures; at `fault_latch_ticks` the core
    /// hard-latches `StreamFault` — a limiter that holds in place
    /// instead of tracking must not stay silent.
    fail_streak: u32,
    fault_latch_ticks: u32,
    /// Log-throttle edges: each failure site speaks once per streak,
    /// never once per 250 Hz tick.
    target_refused: bool,
    scale_refused: bool,
}

impl MotionStream {
    /// Wrap a configured streaming executor running at tick period `dt`
    /// \[s\]; `soft_min`/`soft_max` are the per-joint soft position
    /// limits \[rad\]. `fault_latch_s` is how long `step()` may keep
    /// failing before [`StreamTracker::faulted`] reads true.
    pub fn new(
        executor: StreamingExecutor,
        dt: f64,
        base: MotionLimits,
        fault_latch_s: f64,
    ) -> Self {
        Self {
            executor,
            dt,
            soft_min: base.soft_min,
            soft_max: base.soft_max,
            hold_q: [0.0; MAX_JOINTS],
            base,
            fail_streak: 0,
            fault_latch_ticks: (fault_latch_s / dt).round().max(1.0) as u32,
            target_refused: false,
            scale_refused: false,
        }
    }

    fn clamp(&self, q: &mut [f64; MAX_JOINTS]) {
        for (j, v) in q.iter_mut().enumerate() {
            *v = v.clamp(self.soft_min[j], self.soft_max[j]);
        }
    }
}

impl StreamTracker for MotionStream {
    fn activate(&mut self, q_meas: &[f64; MAX_JOINTS]) {
        self.hold_q = *q_meas;
        self.executor.activate(q_meas);
        // A new session starts its fault window from zero: the streak a
        // latched session left behind must not re-latch on tick one.
        self.fail_streak = 0;
        self.target_refused = false;
        self.scale_refused = false;
    }

    fn set_target(&mut self, q_target: &[f64; MAX_JOINTS]) {
        let mut clamped = *q_target;
        self.clamp(&mut clamped);
        match self.executor.set_target(&clamped) {
            Ok(()) => self.target_refused = false,
            Err(e) => {
                if !self.target_refused {
                    log::warn!("stream target refused: {e} (repeats suppressed)");
                }
                self.target_refused = true;
            }
        }
    }

    fn set_scale(&mut self, speed: f64, accel: f64) {
        let mut scaled = self.base;
        for j in 0..MAX_JOINTS {
            scaled.velocity[j] = self.base.velocity[j] * speed;
            scaled.acceleration[j] = self.base.acceleration[j] * accel;
            // Jerk rides the acceleration fraction: a stream asked to
            // accelerate gently that kept the full jerk ceiling would
            // reach the lower acceleration just as abruptly, which is the
            // jolt the fraction is asking to avoid.
            scaled.jerk[j] = self.base.jerk[j] * accel;
        }
        match self.executor.set_limits(&scaled) {
            Ok(()) => self.scale_refused = false,
            Err(e) => {
                if !self.scale_refused {
                    log::warn!(
                        "stream limit scale ({speed}, {accel}) refused: {e} (repeats suppressed)"
                    );
                }
                self.scale_refused = true;
            }
        }
    }

    fn step(&mut self, q_out: &mut [f64; MAX_JOINTS], qd_out: &mut [f64; MAX_JOINTS]) {
        match self.executor.step() {
            Ok(StreamStep { q, qd, .. }) => {
                *q_out = q;
                self.clamp(q_out);
                // The velocity channel of a cmd-2 position frame is an
                // additive feedforward on the driver's position loop
                // (vendor firmware). The OTG reports the velocity it ends the
                // tick AT, which is zero on every tick that lands on the
                // current target — a stepped stream advancing a reachable
                // target every cycle would get no feedforward at all and
                // track only on position error. Send the larger of the
                // OTG's profile velocity and the rate the position
                // channel actually advanced this tick, so the driver is
                // fed the true rate of the commanded motion.
                for j in 0..MAX_JOINTS {
                    let advance = (q_out[j] - self.hold_q[j]) / self.dt;
                    qd_out[j] = if advance.abs() > qd[j].abs() {
                        advance
                    } else {
                        qd[j]
                    };
                }
                self.hold_q = *q_out;
                if self.fail_streak > 0 {
                    log::info!(
                        "stream limiter recovered after {} failed tick(s)",
                        self.fail_streak
                    );
                    self.fail_streak = 0;
                }
            }
            Err(e) => {
                // Hold in place rather than emit garbage; the RT loop
                // must keep ticking. One record per streak — this site
                // runs at the tick rate.
                if self.fail_streak == 0 {
                    log::warn!("stream step failed ({e}); holding (repeats suppressed)");
                }
                self.fail_streak = self.fail_streak.saturating_add(1);
                *q_out = self.hold_q;
                qd_out.fill(0.0);
            }
        }
    }

    fn faulted(&self) -> bool {
        self.fail_streak >= self.fault_latch_ticks
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use par6_config::LimitMode;
    use par6_motion::MotionLimits;
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicUsize, Ordering};

    /// Counts "stream step failed" warn records (throttle assertion).
    static STEP_FAIL_RECORDS: AtomicUsize = AtomicUsize::new(0);

    struct CountingLogger;
    static LOGGER: CountingLogger = CountingLogger;

    impl log::Log for CountingLogger {
        fn enabled(&self, _meta: &log::Metadata) -> bool {
            true
        }
        fn log(&self, record: &log::Record) {
            if format!("{}", record.args()).contains("stream step failed") {
                STEP_FAIL_RECORDS.fetch_add(1, Ordering::Relaxed);
            }
        }
        fn flush(&self) {}
    }

    fn stream_limits() -> MotionLimits {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
        let cfg = par6_config::RobotConfig::load(&path).expect("PAR6 config");
        MotionLimits::from_config(&cfg, LimitMode::Stream).expect("stream limits")
    }

    /// A failing rate limiter must hold in place, speak ONCE per streak
    /// (its failure site runs at the tick rate), flip `faulted()` after
    /// exactly `round(fault_latch_s / dt)` consecutive failures, and
    /// recover cleanly the moment a step succeeds again. The failure is
    /// the real one: a zeroed velocity ceiling that Ruckig refuses on
    /// every update.
    #[test]
    fn a_failing_limiter_throttles_its_log_and_faults_after_the_window() {
        let _ = log::set_logger(&LOGGER).map(|()| log::set_max_level(log::LevelFilter::Warn));
        let limits = stream_limits();
        let dt = 0.05;
        let fault_latch_s = 0.5; // 10 ticks at this dt
        let mut stream = MotionStream::new(
            par6_motion::StreamingExecutor::new(dt, &limits).expect("executor"),
            dt,
            limits,
            fault_latch_s,
        );
        let start = [0.0; MAX_JOINTS];
        stream.activate(&start);
        let mut target = start;
        target[0] = 0.3;
        stream.set_target(&target);
        stream.set_scale(0.0, 0.0);

        let mut q = [f64::NAN; MAX_JOINTS];
        let mut qd = [f64::NAN; MAX_JOINTS];
        STEP_FAIL_RECORDS.store(0, Ordering::Relaxed);
        for tick in 1..10 {
            stream.step(&mut q, &mut qd);
            assert!(
                !stream.faulted(),
                "tick {tick}: the latch window is {fault_latch_s} s = 10 ticks"
            );
            assert_eq!(q[0], start[0], "a failing step holds, never emits garbage");
            assert_eq!(qd[0], 0.0);
        }
        stream.step(&mut q, &mut qd);
        assert!(
            stream.faulted(),
            "10 consecutive failures = round(fault_latch_s / dt) must fault"
        );
        assert_eq!(
            STEP_FAIL_RECORDS.load(Ordering::Relaxed),
            1,
            "one warn per streak, not one per 250 Hz tick"
        );

        // Recovery: a healthy scale makes the next step succeed and the
        // fault reads clear again.
        stream.set_scale(1.0, 1.0);
        stream.step(&mut q, &mut qd);
        assert!(!stream.faulted(), "a recovered limiter is healthy");
        assert!(q[0] > start[0], "and it is tracking the target again");
    }

    /// A servo source nudging its target a little further every cycle —
    /// the shape a UI slider or a teleoperation feed emits — must be
    /// commanded a velocity that covers the setpoint's advance.
    ///
    /// The wire velocity is the driver's feedforward: without it the
    /// plant tracks a moving setpoint on position error alone and lags
    /// by the full `v/kpp`. Ruckig's terminal velocity is exactly 0 on
    /// every tick whose minimum-time move lands on the target — which is
    /// every tick of a stream whose steps are reachable within one — so
    /// forwarding it verbatim starves the stream of its feedforward.
    #[test]
    fn stepped_stream_targets_are_commanded_a_velocity_that_covers_their_advance() {
        let limits = stream_limits();
        let dt = 0.05;
        let mut stream = MotionStream::new(
            par6_motion::StreamingExecutor::new(dt, &limits).expect("executor"),
            dt,
            limits,
            0.5,
        );

        let start = [0.0; MAX_JOINTS];
        stream.activate(&start);

        let step_rad = 0.25_f64.to_radians();
        let mut target = start;
        let mut q = [0.0; MAX_JOINTS];
        let mut qd = [0.0; MAX_JOINTS];
        let mut ticks_advancing = 0usize;

        for _ in 0..160 {
            let previous = q[0];
            target[0] += step_rad;
            stream.set_target(&target);
            stream.step(&mut q, &mut qd);
            let advance = (q[0] - previous) / dt;
            if advance.abs() > 0.0 {
                ticks_advancing += 1;
                assert!(
                    qd[0].abs() + 1e-12 >= advance.abs(),
                    "commanded velocity {:.6} rad/s under-feeds a position \
                     channel advancing at {:.6} rad/s",
                    qd[0],
                    advance,
                );
            }
        }

        assert!(
            ticks_advancing > 150,
            "the position channel should track the stepped target on \
             essentially every tick, advanced on {ticks_advancing}/160"
        );
        assert!(
            q[0] > 0.9 * target[0],
            "the tracker fell behind the stepped target: {:.4} of {:.4} rad",
            q[0],
            target[0]
        );
    }
}
