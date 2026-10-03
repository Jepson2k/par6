//! Deterministic, explicitly assumed perturbations of the simulated bus.

use serde::{Deserialize, Serialize};

use super::FaultKind;

/// A time window relative to scenario activation, in simulated seconds.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Window {
    pub start_s: f64,
    pub duration_s: f64,
}

/// A latched virtual-driver fault. Normal clear-error commands still apply.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DriverFault {
    pub at_s: f64,
    pub node: u8,
    pub kind: FaultKind,
}

/// Supply availability falls linearly to zero, then stays off. This is an
/// assumed envelope, not a capacitor or motor electrical model.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SupplyLoss {
    pub at_s: f64,
    pub decay_s: f64,
}

/// Offline scenario inputs. No perturbation is active unless requested.
/// Encoder noise affects host telemetry, never the driver's local feedback.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
pub struct SimulationScenario {
    pub seed: u64,
    pub observation_delay_s: f64,
    pub encoder_noise_ticks: u32,
    pub dropout: Option<Window>,
    pub driver_fault: Option<DriverFault>,
    pub supply_loss: Option<SupplyLoss>,
}

impl SimulationScenario {
    pub fn validate(&self) -> Result<(), String> {
        let time = |v: f64| v.is_finite() && (0.0..=3600.0).contains(&v);
        if !self.observation_delay_s.is_finite() || !(0.0..=1.0).contains(&self.observation_delay_s)
        {
            return Err("observation_delay_s must be finite and in [0, 1]".into());
        }
        if self.encoder_noise_ticks > 16384 {
            return Err("encoder_noise_ticks must be at most 16384 motor ticks".into());
        }
        if self
            .dropout
            .is_some_and(|w| !time(w.start_s) || !time(w.duration_s) || w.duration_s == 0.0)
        {
            return Err(
                "dropout requires start_s >= 0 and duration_s > 0, each at most 3600".into(),
            );
        }
        if self
            .driver_fault
            .is_some_and(|f| !time(f.at_s) || f.node >= 16)
        {
            return Err(
                "driver_fault requires a configured node in [0, 15] and at_s in [0, 3600]".into(),
            );
        }
        if self
            .supply_loss
            .is_some_and(|f| !time(f.at_s) || !time(f.decay_s))
        {
            return Err("supply_loss times must be finite and in [0, 3600]".into());
        }
        Ok(())
    }
}

/// Fixed-size state used by the tick path; setup validates and converts times.
#[derive(Default)]
pub(super) struct ActiveScenario {
    pub delay_ticks: u64,
    start_tick: u64,
    noise: u32,
    rng: u64,
    dropout: Option<(u64, u64)>,
    fault: Option<(u64, u8, FaultKind)>,
    supply: Option<(u64, u64)>,
}

impl ActiveScenario {
    pub fn new(s: &SimulationScenario, tick: u64, dt: f64) -> Self {
        let ticks = |t: f64| (t / dt).round() as u64;
        Self {
            delay_ticks: ticks(s.observation_delay_s),
            start_tick: tick,
            noise: s.encoder_noise_ticks,
            rng: s.seed,
            dropout: s
                .dropout
                .map(|w| (ticks(w.start_s), ticks(w.duration_s).max(1))),
            fault: s.driver_fault.map(|f| (ticks(f.at_s), f.node, f.kind)),
            supply: s.supply_loss.map(|f| (ticks(f.at_s), ticks(f.decay_s))),
        }
    }

    pub fn supply_scale(&self, tick: u64) -> f64 {
        let Some((at, decay)) = self.supply else {
            return 1.0;
        };
        let elapsed = tick.saturating_sub(self.start_tick);
        if elapsed < at {
            return 1.0;
        }
        if decay == 0 {
            return 0.0;
        }
        (1.0 - elapsed.saturating_sub(at) as f64 / decay as f64).max(0.0)
    }

    pub fn drop_reply(&self, tick: u64) -> bool {
        let elapsed = tick.saturating_sub(self.start_tick);
        self.supply_scale(tick) == 0.0
            || self
                .dropout
                .is_some_and(|(start, duration)| elapsed >= start && elapsed - start < duration)
    }

    pub fn take_fault(&mut self, tick: u64) -> Option<(u8, FaultKind)> {
        let (at, node, kind) = self.fault?;
        if tick.saturating_sub(self.start_tick) < at {
            return None;
        }
        self.fault = None;
        Some((node, kind))
    }

    pub fn encoder_noise(&mut self) -> i32 {
        if self.noise == 0 {
            return 0;
        }
        self.rng = self.rng.wrapping_add(0x9e3779b97f4a7c15);
        let mut z = self.rng;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d049bb133111eb);
        z ^= z >> 31;
        (z % (2 * u64::from(self.noise) + 1)) as i32 - self.noise as i32
    }
}
