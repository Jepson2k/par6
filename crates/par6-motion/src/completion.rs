//! Controller-side completion policies (spec/RT.md, EXEC section).
//!
//! A queued command finishes under one of three policies: `commanded`
//! completes at its last sample; `settled` (default) holds until every
//! joint tracks within tolerance or a timeout elapses; `strict` is
//! `settled` with the timeout escalated to an error. A `blend_continues`
//! boundary bypasses settling entirely so blended corners stay
//! velocity-continuous.

use crate::{MotionError, NUM_JOINTS};

/// When a queued command counts as complete.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CompletionPolicy {
    /// Complete as soon as the last sample is consumed.
    Commanded,
    /// Hold after the last sample until all joints track within tolerance,
    /// or the timeout elapses (then complete anyway).
    #[default]
    Settled,
    /// Like `Settled`, but a timeout is an error.
    Strict,
}

/// Settling parameters. Times are seconds, converted to ticks at
/// construction (`round(s / dt)`), per the config time-constant rule.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SettleParams {
    /// All-joint position error tolerance \[rad\] (spec default 0.01).
    pub tolerance_rad: f64,
    /// Settle timeout \[s\] (spec default 2.0 = 500 ticks at 4 ms).
    pub timeout_s: f64,
}

impl Default for SettleParams {
    fn default() -> Self {
        Self {
            tolerance_rad: 0.01,
            timeout_s: 2.0,
        }
    }
}

/// Progress reported by the monitor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CompletionEvent {
    /// Still settling (or nothing armed).
    Pending,
    /// The command is complete.
    Complete,
}

/// Per-command completion state machine.
///
/// Drive it from the EXEC consumer: [`arm`] at a segment boundary (passing
/// the boundary sample's `blend_continues`), then [`tick`] each RT tick
/// with the measured and target joint positions until it reports
/// [`CompletionEvent::Complete`] — or, under `strict`, fails with
/// [`MotionError::SettleTimeout`].
///
/// [`arm`]: CompletionMonitor::arm
/// [`tick`]: CompletionMonitor::tick
#[derive(Debug, Clone)]
pub struct CompletionMonitor {
    policy: CompletionPolicy,
    tolerance_rad: f64,
    timeout_ticks: u32,
    elapsed: u32,
    armed: bool,
}

impl CompletionMonitor {
    /// Build a monitor for `policy` with `params` converted at tick period
    /// `dt` \[s\].
    pub fn new(
        policy: CompletionPolicy,
        params: SettleParams,
        dt: f64,
    ) -> Result<Self, MotionError> {
        if !(dt.is_finite() && dt > 0.0 && dt < 1.0) {
            return Err(MotionError::InvalidInput {
                what: "dt",
                reason: format!("must be a finite tick period in (0, 1) s, got {dt}"),
            });
        }
        if !(params.tolerance_rad.is_finite() && params.tolerance_rad > 0.0) {
            return Err(MotionError::InvalidInput {
                what: "tolerance_rad",
                reason: format!("must be finite and > 0, got {}", params.tolerance_rad),
            });
        }
        if !(params.timeout_s.is_finite() && params.timeout_s > 0.0) {
            return Err(MotionError::InvalidInput {
                what: "timeout_s",
                reason: format!("must be finite and > 0, got {}", params.timeout_s),
            });
        }
        Ok(Self {
            policy,
            tolerance_rad: params.tolerance_rad,
            timeout_ticks: (params.timeout_s / dt).round() as u32,
            elapsed: 0,
            armed: false,
        })
    }

    /// Arm at a segment boundary. Returns [`CompletionEvent::Complete`]
    /// immediately when no settling applies — `commanded` policy, or a
    /// `blend_continues` boundary (any policy).
    pub fn arm(&mut self, blend_continues: bool) -> CompletionEvent {
        if blend_continues || self.policy == CompletionPolicy::Commanded {
            self.armed = false;
            return CompletionEvent::Complete;
        }
        self.armed = true;
        self.elapsed = 0;
        CompletionEvent::Pending
    }

    /// True while a settle is in progress.
    pub fn is_armed(&self) -> bool {
        self.armed
    }

    /// Feed one tick of measured vs. target joint positions.
    ///
    /// Completes when the largest per-joint error is within tolerance;
    /// at timeout `settled` completes anyway while `strict` fails with
    /// [`MotionError::SettleTimeout`]. Unarmed ticks report `Pending`.
    pub fn tick(
        &mut self,
        q_meas: &[f64; NUM_JOINTS],
        q_target: &[f64; NUM_JOINTS],
    ) -> Result<CompletionEvent, MotionError> {
        if !self.armed {
            return Ok(CompletionEvent::Pending);
        }
        let mut worst_joint = 0;
        let mut worst = 0.0_f64;
        for (j, (m, t)) in q_meas.iter().zip(q_target.iter()).enumerate() {
            let err = (m - t).abs();
            if err > worst {
                worst = err;
                worst_joint = j;
            }
        }
        if worst <= self.tolerance_rad {
            self.armed = false;
            return Ok(CompletionEvent::Complete);
        }
        self.elapsed += 1;
        if self.elapsed >= self.timeout_ticks {
            self.armed = false;
            return match self.policy {
                CompletionPolicy::Strict => Err(MotionError::SettleTimeout {
                    worst_joint,
                    error_rad: worst,
                    tolerance_rad: self.tolerance_rad,
                }),
                _ => Ok(CompletionEvent::Complete),
            };
        }
        Ok(CompletionEvent::Pending)
    }
}
