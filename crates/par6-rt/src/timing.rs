//! Loop-period statistics and the one-sided degradation bands.
//!
//! Fed one measured loop period per tick; keeps a rolling window whose
//! percentiles are recomputed periodically. Bands are one-sided (the
//! vendor loop can only run SLOW; with absolute deadlines an early wake
//! is absorbed by the next `clock_nanosleep`): `p99 > degraded·dt` is a
//! self-clearing `LOOP_DEGRADED` warning, `p99 > critical·dt` sustained
//! for the configured interval is the `LOOP_CRITICAL` hard latch. The
//! three band parameters come from [`TimingConfig`] and default to the
//! vendor values. Nothing is evaluated during the warmup, so boot jitter
//! cannot latch a false critical.
//!
//! The sustain is a floor, not a resolution: `p99` only moves every
//! `RECOMPUTE_EVERY` ticks, so a sustain shorter than that interval
//! latches on the first bad percentile. That interval is a wall-clock
//! one — `RECOMPUTE_EVERY · dt`, which [`sustain_resolution_s`] reports
//! — so the tick rate decides whether a given sustain is resolvable at
//! all, and `par6d` refuses a config that pairs the two badly.
//!
//! Everything is preallocated at construction; [`LoopTiming::record`] is
//! allocation-free.

use par6_config::TimingConfig;

use crate::state::LoopStats;

/// Rolling-window size in samples (vendor constant).
///
/// A COUNT, not a duration, and deliberately so: `p99` is the 495th of
/// these 500 order statistics, and a percentile needs its samples to
/// mean anything. Held to a wall-clock span instead, a 50 ms tick would
/// leave 40 samples and a "p99" that is really the window maximum.
const WINDOW: usize = 500;
/// Percentiles are recomputed every this many ticks (vendor constant).
///
/// Also a count — it buys the sort back over that many ticks. What it
/// costs is resolution: `p99` cannot move faster than this, which is
/// what [`sustain_resolution_s`] converts into the shortest critical
/// sustain worth configuring.
const RECOMPUTE_EVERY: u64 = 50;
/// Ticks before the bands are evaluated at all (vendor constant; covers
/// filling the window plus scheduler settling at boot).
///
/// Coupled to [`WINDOW`]: the bands cannot be judged before the window
/// holds samples, so this must outrun it and is a count for the same
/// reason. Longer at a slow tick is the conservative direction — it
/// delays judgement, it does not weaken it.
const WARMUP_TICKS: u64 = 850;
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

/// The shortest critical-band sustain that means anything at tick `dt`
/// \[s\].
///
/// `p99` only moves once every `RECOMPUTE_EVERY` ticks, so a sustain
/// under that reduces `LOOP_CRITICAL` — a hard latch that disables the
/// controller — to "the first bad percentile latches". The vendor
/// default sustain of 1 s clears this only while the tick stays under
/// 20 ms; past that the guard becomes a hair trigger with nothing to
/// say so. `par6d` refuses a config that pairs the two badly; the
/// number lives here because `RECOMPUTE_EVERY` does.
pub fn sustain_resolution_s(dt: f64) -> f64 {
    RECOMPUTE_EVERY as f64 * dt
}

/// Loop-period tracker. One instance per RT core, fed once per tick.
#[derive(Debug)]
pub struct LoopTiming {
    degraded_period_s: f64,
    critical_period_s: f64,
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
    /// Tracker for tick period `dt` \[s\] with the config's degradation
    /// bands. Allocates its buffers here.
    pub fn new(dt: f64, bands: TimingConfig) -> Self {
        Self {
            degraded_period_s: bands.degraded_factor * dt,
            critical_period_s: bands.critical_factor * dt,
            window: Vec::with_capacity(WINDOW),
            scratch: vec![0.0; WINDOW],
            next: 0,
            filled: false,
            ticks: 0,
            critical_streak: 0,
            critical_sustain_ticks: ((bands.critical_sustain_s / dt).round() as u32).max(1),
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
        if self.ticks.is_multiple_of(RECOMPUTE_EVERY) && !self.window.is_empty() {
            self.recompute();
        }
        if self.ticks < WARMUP_TICKS || !self.filled {
            self.critical_streak = 0;
            return LoopHealth::Ok;
        }
        if self.stats.p99_s > self.critical_period_s {
            self.critical_streak = self.critical_streak.saturating_add(1);
        } else {
            self.critical_streak = 0;
        }
        if self.critical_streak >= self.critical_sustain_ticks {
            LoopHealth::Critical
        } else if self.stats.p99_s > self.degraded_period_s {
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
        self.stats.p95_s = rank(0.95);
        self.stats.p99_s = rank(0.99);
        self.stats.min_s = s[0];
        self.stats.max_s = s[n - 1];
        // Dispersion over the same window the percentiles come from,
        // as the population sigma the vendor's Welford pass computes.
        let mean = s.iter().sum::<f64>() / n as f64;
        let var = s.iter().map(|v| (v - mean) * (v - mean)).sum::<f64>() / n as f64;
        self.stats.std_s = var.sqrt();
    }

    /// Current statistics for the snapshot (frame ages are filled by the
    /// caller from the bus drain).
    pub fn stats(&self) -> LoopStats {
        self.stats
    }

    /// Discard all statistics and re-enter the warmup gate (the
    /// `reset_loop_stats` command). Allocation-free: the window keeps its
    /// capacity.
    pub fn reset(&mut self) {
        self.window.clear();
        self.next = 0;
        self.filled = false;
        self.ticks = 0;
        self.critical_streak = 0;
        self.stats = LoopStats::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Run `ticks` periods from `period` through a fresh tracker and
    /// report whether it ever reached each band.
    fn run(dt: f64, bands: TimingConfig, ticks: u64, period: impl Fn(u64) -> f64) -> (bool, bool) {
        let mut t = LoopTiming::new(dt, bands);
        let (mut degraded, mut critical) = (false, false);
        for i in 0..ticks {
            match t.record(period(i), false) {
                LoopHealth::Critical => critical = true,
                LoopHealth::Degraded => degraded = true,
                LoopHealth::Ok => {}
            }
        }
        (degraded, critical)
    }

    /// The CI-flake trace: a wall-clock sim at a 50 ms tick on a shared
    /// host, where a small fraction of ticks wake very late. p99 is the
    /// 495th of 500 samples, so ~1.6% late ticks put it at the outlier
    /// value — 2.8·dt, far above the vendor 1.10·dt hard band but nowhere
    /// near a loop that is actually failing to run.
    #[test]
    fn sim_bands_ride_out_host_jitter_that_latches_the_vendor_bands() {
        let dt = 0.05;
        let jitter = |i: u64| if i.is_multiple_of(60) { dt * 2.8 } else { dt };
        // Long enough to clear warmup and then hold the trace for several
        // multiples of the sim sustain window.
        let ticks = WARMUP_TICKS + 10 * (TimingConfig::SIM.critical_sustain_s / dt) as u64;

        let (_, vendor_critical) = run(dt, TimingConfig::default(), ticks, jitter);
        assert!(
            vendor_critical,
            "the vendor bands are what makes this trace latch"
        );

        let (sim_degraded, sim_critical) = run(dt, TimingConfig::SIM, ticks, jitter);
        assert!(
            !sim_critical,
            "host jitter must not disable the controller under the sim bands"
        );
        assert!(
            sim_degraded,
            "the jitter must still be reported as degradation"
        );

        // A loop genuinely running an order of magnitude slow is still
        // caught — the sim bands widen the guard, they do not remove it.
        let (_, runaway_critical) = run(dt, TimingConfig::SIM, ticks, |_| dt * 10.0);
        assert!(runaway_critical, "a runaway loop must still hard-latch");
    }

    /// Every statistic the LOOP_STATS query publishes comes off this
    /// window — `std`, `min` and `p95` included. They were reported as
    /// 0.0, which is indistinguishable from a loop that never ran.
    #[test]
    fn the_window_yields_min_std_and_p95_not_zeros() {
        let dt = 0.004;
        let mut t = LoopTiming::new(dt, TimingConfig::default());
        // A window with known order statistics: 100 periods spread
        // evenly over [dt, 2·dt], fed until the window holds only these.
        let n = 100;
        let period = |i: u64| dt * (1.0 + (i % n) as f64 / n as f64);
        for i in 0..(WINDOW as u64 + RECOMPUTE_EVERY) {
            t.record(period(i), false);
        }
        let s = t.stats();
        assert!(
            (s.min_s - dt).abs() < 1e-12,
            "the fastest tick in the window is dt, got {}",
            s.min_s
        );
        assert!(
            (s.max_s - dt * 1.99).abs() < 1e-12,
            "the slowest is just under 2·dt, got {}",
            s.max_s
        );
        assert!(
            s.min_s < s.p50_s && s.p50_s < s.p95_s && s.p95_s < s.p99_s && s.p99_s <= s.max_s,
            "the percentiles must be ordered and distinct: {s:?}"
        );
        // Uniform over [dt, 2·dt): sigma = span / sqrt(12).
        let expected = dt / 12f64.sqrt();
        assert!(
            (s.std_s - expected).abs() < 0.02 * expected,
            "std must measure the window's spread ({expected}), got {}",
            s.std_s
        );
        // A perfectly steady loop has no spread at all.
        for _ in 0..(WINDOW as u64 + RECOMPUTE_EVERY) {
            t.record(dt, false);
        }
        let s = t.stats();
        assert!(s.std_s < 1e-15, "a steady loop has no spread: {}", s.std_s);
        assert_eq!((s.min_s, s.p95_s, s.max_s), (dt, dt, dt));
    }

    #[test]
    fn bands_need_warmup_then_degrade_then_latch_critical() {
        let dt = 0.004;
        let mut t = LoopTiming::new(dt, TimingConfig::default());
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
        let mut t = LoopTiming::new(dt, TimingConfig::default());
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
