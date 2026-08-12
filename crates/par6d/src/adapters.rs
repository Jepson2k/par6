//! The real `par6-motion` engines behind the `par6-rt` per-tick hook
//! traits — thin lifecycle mappings, no behavior of their own.

use par6_motion::{JogDirection, StreamStep, StreamingExecutor};
use par6_rt::{JogEngine as RtJogEngine, StreamTracker, MAX_JOINTS};

/// `par6_motion::JogEngine` (jerk-aware lookahead, direction-block
/// latching) behind the RT jog hook.
pub struct MotionJog {
    engine: par6_motion::JogEngine,
}

impl MotionJog {
    /// Wrap a configured jog engine.
    pub fn new(engine: par6_motion::JogEngine) -> Self {
        Self { engine }
    }
}

impl RtJogEngine for MotionJog {
    fn activate(&mut self, q_meas: &[f64; MAX_JOINTS]) {
        self.engine.activate(q_meas);
    }

    fn command(&mut self, joint: usize, signed_pct: f64) {
        if signed_pct == 0.0 || !signed_pct.is_finite() {
            self.engine.release();
            return;
        }
        let dir = if signed_pct > 0.0 {
            JogDirection::Positive
        } else {
            JogDirection::Negative
        };
        if let Err(e) = self.engine.command(joint, dir, signed_pct.abs().min(1.0)) {
            log::warn!("jog command refused: {e}");
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
/// stream hook, with the spec's unconditional soft-limit clamp on the
/// way in and out.
pub struct MotionStream {
    executor: StreamingExecutor,
    dt: f64,
    soft_min: [f64; MAX_JOINTS],
    soft_max: [f64; MAX_JOINTS],
    hold_q: [f64; MAX_JOINTS],
}

impl MotionStream {
    /// Wrap a configured streaming executor running at tick period `dt`
    /// \[s\]; `soft_min`/`soft_max` are the per-joint soft position
    /// limits \[rad\].
    pub fn new(
        executor: StreamingExecutor,
        dt: f64,
        soft_min: [f64; MAX_JOINTS],
        soft_max: [f64; MAX_JOINTS],
    ) -> Self {
        Self {
            executor,
            dt,
            soft_min,
            soft_max,
            hold_q: [0.0; MAX_JOINTS],
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
    }

    fn set_target(&mut self, q_target: &[f64; MAX_JOINTS]) {
        let mut clamped = *q_target;
        self.clamp(&mut clamped);
        if let Err(e) = self.executor.set_target(&clamped) {
            log::warn!("stream target refused: {e}");
        }
    }

    fn step(&mut self, q_out: &mut [f64; MAX_JOINTS], qd_out: &mut [f64; MAX_JOINTS]) {
        match self.executor.step() {
            Ok(StreamStep { q, qd, .. }) => {
                *q_out = q;
                self.clamp(q_out);
                // The velocity channel of a cmd-2 position frame is a
                // CAP the driver clamps its own position-loop output to
                // (spec/CAN.md), not a setpoint. The OTG reports the
                // velocity it ends the tick AT, which is zero on every
                // tick that lands on the current target — so forwarding
                // it caps a still-advancing position channel at a
                // standstill. Command the larger of the two: the OTG's
                // own profile velocity, or the rate the position channel
                // is advancing at this tick.
                for j in 0..MAX_JOINTS {
                    let advance = (q_out[j] - self.hold_q[j]) / self.dt;
                    qd_out[j] = if advance.abs() > qd[j].abs() {
                        advance
                    } else {
                        qd[j]
                    };
                }
                self.hold_q = *q_out;
            }
            Err(e) => {
                // Hold in place rather than emit garbage; the RT loop
                // must keep ticking.
                log::warn!("stream step failed ({e}); holding");
                *q_out = self.hold_q;
                qd_out.fill(0.0);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use par6_config::LimitMode;
    use par6_motion::MotionLimits;
    use std::path::PathBuf;

    fn stream_limits() -> MotionLimits {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
        let cfg = par6_config::RobotConfig::load(&path).expect("PAR6 config");
        MotionLimits::from_config(&cfg, LimitMode::Stream).expect("stream limits")
    }

    /// A servo source nudging its target a little further every cycle —
    /// the shape a UI slider or a teleoperation feed emits — must be
    /// commanded a velocity that covers the setpoint's advance.
    ///
    /// The wire velocity is a cap, so a commanded velocity below the rate
    /// the position channel advances at throttles the arm to a standstill
    /// while the position channel keeps moving: the failure reports
    /// nothing anywhere. Ruckig's terminal velocity is exactly 0 on every
    /// tick whose minimum-time move lands on the target, which is every
    /// tick of a stream whose steps are reachable within one.
    #[test]
    fn stepped_stream_targets_are_commanded_a_velocity_that_covers_their_advance() {
        let limits = stream_limits();
        let dt = 0.05;
        let mut stream = MotionStream::new(
            par6_motion::StreamingExecutor::new(dt, &limits).expect("executor"),
            dt,
            limits.soft_min,
            limits.soft_max,
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
                    "commanded velocity {:.6} rad/s caps a position channel \
                     advancing at {:.6} rad/s",
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
