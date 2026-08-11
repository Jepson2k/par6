//! Loop-period statistics and the one-sided degradation bands
//! (spec/RT.md "Rate & timing").
//!
//! Fed one measured loop period per tick; keeps a rolling window whose
//! percentiles are recomputed periodically. Bands are one-sided (the
//! vendor loop can only run SLOW; with absolute deadlines an early wake
//! is absorbed by the next `clock_nanosleep`): `p99 > 1.05·dt` is a
//! self-clearing `LOOP_DEGRADED` warning, `p99 > 1.10·dt` sustained for
//! 1.0 s is the `LOOP_CRITICAL` hard latch. Nothing is evaluated during
//! the warmup, so boot jitter cannot latch a false critical.
//!
//! Everything is preallocated at construction; [`LoopTiming::record`] is
//! allocation-free.

use crate::state::LoopStats;

/// Rolling-window size in samples (vendor constant).
const WINDOW: usize = 500;
/// Percentiles are recomputed every this many ticks (vendor constant).
const RECOMPUTE_EVERY: u64 = 50;
/// Ticks before the bands are evaluated at all (vendor constant; covers
/// filling the window plus scheduler settling at boot).
const WARMUP_TICKS: u64 = 850;
/// `p99 > DEGRADED_FACTOR · dt` = warning band.
const DEGRADED_FACTOR: f64 = 1.05;
/// `p99 > CRITICAL_FACTOR · dt` sustained = hard band.
const CRITICAL_FACTOR: f64 = 1.10;
/// How long the critical band must hold before latching \[s\].
const CRITICAL_SUSTAIN_S: f64 = 1.0;
/// EMA smoothing factor for the published mean period.
const EMA_ALPHA: f64 = 0.05;

/// Degradation verdict for the current tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoopHealth {
    /// Within bands (or still warming up).
    Ok,
    /// p99 above the warning band — self-clearing condition.
    Degraded,
    /// p99 above the critical band for the sustain time — the caller
    /// hard-latches `LOOP_CRITICAL`.
    Critical,
}

/// Loop-period tracker. One instance per RT core, fed once per tick.
#[derive(Debug)]
pub struct LoopTiming {
    dt: f64,
    window: Vec<f64>,
    scratch: Vec<f64>,
    next: usize,
    filled: bool,
    ticks: u64,
    critical_streak: u32,
    critical_sustain_ticks: u32,
    stats: LoopStats,
}

impl LoopTiming {
    /// Tracker for tick period `dt` \[s\]. Allocates its buffers here.
    pub fn new(dt: f64) -> Self {
        Self {
            dt,
            window: Vec::with_capacity(WINDOW),
            scratch: vec![0.0; WINDOW],
            next: 0,
            filled: false,
            ticks: 0,
            critical_streak: 0,
            critical_sustain_ticks: ((CRITICAL_SUSTAIN_S / dt).round() as u32).max(1),
            stats: LoopStats::default(),
        }
    }

    /// Feed one measured loop period; `overrun` marks a missed deadline.
    /// Returns the band verdict for this tick.
    pub fn record(&mut self, period_s: f64, overrun: bool) -> LoopHealth {
        self.ticks += 1;
        if overrun {
            self.stats.overruns = self.stats.overruns.saturating_add(1);
        }
        self.stats.period_ema_s = if self.stats.period_ema_s == 0.0 {
            period_s
        } else {
            self.stats.period_ema_s + EMA_ALPHA * (period_s - self.stats.period_ema_s)
        };
        if self.window.len() < WINDOW {
            self.window.push(period_s);
        } else {
            self.window[self.next] = period_s;
        }
        self.next = (self.next + 1) % WINDOW;
        if self.window.len() == WINDOW {
            self.filled = true;
        }
        if self.ticks % RECOMPUTE_EVERY == 0 && !self.window.is_empty() {
            self.recompute();
        }
        if self.ticks < WARMUP_TICKS || !self.filled {
            self.critical_streak = 0;
            return LoopHealth::Ok;
        }
        if self.stats.p99_s > CRITICAL_FACTOR * self.dt {
            self.critical_streak = self.critical_streak.saturating_add(1);
        } else {
            self.critical_streak = 0;
        }
        if self.critical_streak >= self.critical_sustain_ticks {
            LoopHealth::Critical
        } else if self.stats.p99_s > DEGRADED_FACTOR * self.dt {
            LoopHealth::Degraded
        } else {
            LoopHealth::Ok
        }
    }

    fn recompute(&mut self) {
        let n = self.window.len();
        self.scratch[..n].copy_from_slice(&self.window);
        let s = &mut self.scratch[..n];
        s.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let rank = |p: f64| -> f64 {
            let idx = ((n as f64 * p).ceil() as usize).clamp(1, n) - 1;
            s[idx]
        };
        self.stats.p50_s = rank(0.50);
        self.stats.p90_s = rank(0.90);
        self.stats.p99_s = rank(0.99);
        self.stats.max_s = s[n - 1];
    }

    /// Current statistics for the snapshot (frame ages are filled by the
    /// caller from the bus drain).
    pub fn stats(&self) -> LoopStats {
        self.stats
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bands_need_warmup_then_degrade_then_latch_critical() {
        let dt = 0.004;
        let mut t = LoopTiming::new(dt);
        // Slow periods from the very first tick: warmup must still gate.
        for _ in 0..(WARMUP_TICKS - 1) {
            assert_eq!(t.record(dt * 1.2, false), LoopHealth::Ok, "warmup gates");
        }
        // Past warmup with a fully bad window the sustain counter runs;
        // 1.0 s of sustained critical-band p99 latches.
        let sustain = (1.0f64 / dt).round() as u64;
        let mut verdicts = Vec::new();
        for _ in 0..sustain + RECOMPUTE_EVERY {
            verdicts.push(t.record(dt * 1.2, false));
        }
        assert!(verdicts.contains(&LoopHealth::Critical), "sustained latch");
        // Degraded band: periods slightly high (7% over) — degraded but
        // never critical.
        let mut t = LoopTiming::new(dt);
        let mut saw_degraded = false;
        for _ in 0..(WARMUP_TICKS + 3 * sustain) {
            match t.record(dt * 1.07, false) {
                LoopHealth::Critical => panic!("1.07·dt must not be critical"),
                LoopHealth::Degraded => saw_degraded = true,
                LoopHealth::Ok => {}
            }
        }
        assert!(saw_degraded);
        // Self-clears when periods recover.
        for _ in 0..(WINDOW as u64 + RECOMPUTE_EVERY) {
            t.record(dt, false);
        }
        assert_eq!(t.record(dt, false), LoopHealth::Ok);
        assert!(t.stats().p99_s <= dt * 1.0001);
    }
}
