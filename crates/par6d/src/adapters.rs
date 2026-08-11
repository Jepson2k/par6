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
    soft_min: [f64; MAX_JOINTS],
    soft_max: [f64; MAX_JOINTS],
    hold_q: [f64; MAX_JOINTS],
}

impl MotionStream {
    /// Wrap a configured streaming executor; `soft_min`/`soft_max` are
    /// the per-joint soft position limits \[rad\].
    pub fn new(
        executor: StreamingExecutor,
        soft_min: [f64; MAX_JOINTS],
        soft_max: [f64; MAX_JOINTS],
    ) -> Self {
        Self {
            executor,
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
                self.hold_q = *q_out;
                *qd_out = qd;
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
