//! Input shaping for the arm's vibration modes on what it stands on.
//!
//! The arm and its support swing at a few hertz with almost no damping
//! (J2 rang at 5 Hz for seconds after each move, damping ratio ~0.01,
//! 2026-11-08), and the motors' encoders barely see it, so no gain damps
//! it. A shaper sends each EXEC move as a few delayed, weighted copies of
//! itself whose vibrations cancel. Each copy's weight is positive and the
//! weights sum to one, so every sample of a shaped move is a weighted mean
//! of the unshaped move's and stays inside the same limits.
//!
//! The modes' frequencies shift with the pose and the support, so the
//! shaper is an extra-insensitive one (Singhose), leaving at most
//! `RESIDUAL` of a mode's vibration anywhere in a band of frequencies:
//! the three-impulse EI, one period of the band's centre long, covers
//! 0.8 to 1.2 times its centre; the two-hump EI, half a period longer,
//! 0.638 to 1.362.

use par6_rt::MAX_JOINTS;

/// One tick-rate sample: `[q, q̇, q̈]`.
pub(crate) type Sample = [f64; 3 * MAX_JOINTS];

/// The share of a mode's vibration a shaped move may leave.
const RESIDUAL: f64 = 0.05;

/// The widest spread of frequencies, highest over lowest, each shaper
/// leaves within `RESIDUAL`: the EI's and the two-hump EI's.
const EI_SPREAD: f64 = 1.2 / 0.8;
const EI2_SPREAD: f64 = 1.362 / 0.638;

/// A sequence of `(delay [s], weight)` impulses.
#[derive(Debug, Clone, PartialEq)]
pub struct Shaper(Vec<(f64, f64)>);

impl Shaper {
    /// The shortest shaper that leaves at most `RESIDUAL` of a mode at
    /// any of `frequencies_hz`, or why none here does.
    pub fn covering(frequencies_hz: &[f64]) -> Result<Self, String> {
        let lo = frequencies_hz.iter().copied().fold(f64::INFINITY, f64::min);
        let hi = frequencies_hz.iter().copied().fold(0.0, f64::max);
        if frequencies_hz.is_empty() || !frequencies_hz.iter().all(|f| f.is_finite() && *f > 0.0) {
            return Err(format!(
                "vibration frequencies must be > 0 and finite, got {frequencies_hz:?}"
            ));
        }
        let period = 2.0 / (lo + hi);
        let v = RESIDUAL;
        if hi / lo <= EI_SPREAD {
            let side = (1.0 + v) / 4.0;
            return Ok(Self(vec![
                (0.0, side),
                (period / 2.0, 1.0 - 2.0 * side),
                (period, side),
            ]));
        }
        if hi / lo <= EI2_SPREAD {
            let x = (v * v * ((1.0 - v * v).sqrt() + 1.0)).cbrt();
            let outer = (3.0 * x * x + 2.0 * x + 3.0 * v * v) / (16.0 * x);
            return Ok(Self(vec![
                (0.0, outer),
                (period / 2.0, 0.5 - outer),
                (period, 0.5 - outer),
                (1.5 * period, outer),
            ]));
        }
        Err(format!(
            "vibration frequencies {lo:.2} to {hi:.2} Hz spread wider than one shaper covers \
             ({EI2_SPREAD:.2}x)"
        ))
    }

    /// How much later a shaped move ends \[s\].
    pub fn duration(&self) -> f64 {
        self.0.last().map_or(0.0, |&(delay, _)| delay)
    }

    /// `samples`, a move from rest at `start` at one sample per `dt` from
    /// `dt` on, shaped. Each delay is rounded to whole samples, so every
    /// shaped sample mixes whole samples and stays self-consistent; the
    /// rounding is within half a sample of a period of tens of them.
    pub(crate) fn shape(
        &self,
        start: &[f64; MAX_JOINTS],
        samples: &[Sample],
        dt: f64,
    ) -> Vec<Sample> {
        let Some(last) = samples.last() else {
            return Vec::new();
        };
        let mut rest = [0.0; 3 * MAX_JOINTS];
        rest[..MAX_JOINTS].copy_from_slice(start);
        let delays: Vec<(usize, f64)> = self
            .0
            .iter()
            .map(|&(delay, weight)| ((delay / dt).round() as usize, weight))
            .collect();
        let n = samples.len() + delays.last().map_or(0, |&(d, _)| d);
        // Sample `k` of the unshaped move (1-based), at rest on either side.
        let at = |k: usize| -> &Sample {
            match k {
                0 => &rest,
                k if k > samples.len() => last,
                k => &samples[k - 1],
            }
        };
        (1..=n)
            .map(|k| {
                let mut out = [0.0; 3 * MAX_JOINTS];
                for &(delay, weight) in &delays {
                    let s = at(k.saturating_sub(delay));
                    for (o, v) in out.iter_mut().zip(s) {
                        *o += weight * v;
                    }
                }
                out
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The vibration a shaper leaves of an undamped mode at `f` \[Hz\],
    /// as a share of the unshaped move's.
    fn residual(shaper: &Shaper, f: f64) -> f64 {
        let w = 2.0 * std::f64::consts::PI * f;
        let (c, s) = shaper.0.iter().fold((0.0, 0.0), |(c, s), &(t, a)| {
            (c + a * (w * t).cos(), s + a * (w * t).sin())
        });
        c.hypot(s)
    }

    #[test]
    fn a_shaper_cancels_every_mode_in_its_band_and_the_shortest_is_chosen() {
        // The arm's modes on the desk (2026-11-08): J1 4.3 Hz arm out to
        // 6.5 Hz folded, J2 and J3 5 Hz.
        for (modes, duration) in [
            (vec![4.9, 5.2], 1.0 / 5.05),
            (vec![4.3, 5.0, 6.4], 1.0 / 5.35),
            (vec![3.0, 6.3], 1.5 / 4.65),
        ] {
            let shaper = Shaper::covering(&modes).unwrap();
            assert!((shaper.duration() - duration).abs() < 1e-9, "{modes:?}");
            let (lo, hi) = (modes[0], modes[modes.len() - 1]);
            for i in 0..=50 {
                let f = lo + (hi - lo) * f64::from(i) / 50.0;
                let r = residual(&shaper, f);
                assert!(
                    r <= RESIDUAL + 1e-3,
                    "{modes:?}: {r} of the mode left at {f} Hz"
                );
            }
            let sum: f64 = shaper.0.iter().map(|&(_, a)| a).sum();
            assert!((sum - 1.0).abs() < 1e-12 && shaper.0.iter().all(|&(_, a)| a > 0.0));
        }
        for wrong in [vec![], vec![0.0, 5.0], vec![f64::NAN], vec![2.0, 6.0]] {
            assert!(Shaper::covering(&wrong).is_err(), "{wrong:?}");
        }
    }

    #[test]
    fn a_shaped_move_ends_where_the_move_does_at_rest_and_never_faster() {
        let dt = 0.004;
        // A J1 rest-to-rest move: a sine-squared velocity pulse.
        let n = 150;
        let span = 0.5;
        let samples: Vec<Sample> = (1..=n)
            .map(|k| {
                let u = k as f64 / n as f64;
                let tf = n as f64 * dt;
                let w = std::f64::consts::PI / tf;
                let mut s = [0.0; 3 * MAX_JOINTS];
                s[0] = span
                    * (u - (2.0 * std::f64::consts::PI * u).sin() / (2.0 * std::f64::consts::PI));
                s[MAX_JOINTS] = span / tf * 2.0 * (w * u * tf).sin().powi(2);
                s[2 * MAX_JOINTS] = span / tf * 2.0 * w * (2.0 * w * u * tf).sin();
                s
            })
            .collect();
        let shaper = Shaper::covering(&[4.3, 6.4]).unwrap();
        let shaped = shaper.shape(&[0.0; MAX_JOINTS], &samples, dt);
        assert_eq!(shaped.len(), n + (shaper.duration() / dt).round() as usize);
        let end = shaped.last().unwrap();
        assert!((end[0] - span).abs() < 1e-12 && end[MAX_JOINTS].abs() < 1e-12);
        let peak = |s: &[Sample], c: usize| s.iter().map(|x| x[c].abs()).fold(0.0, f64::max);
        for c in [MAX_JOINTS, 2 * MAX_JOINTS] {
            assert!(peak(&shaped, c) <= peak(&samples, c) + 1e-12);
        }
    }
}
