//! Ripple that repeats with a stepper's electrical angle, from a loop-rate
//! capture.
//!
//! Cogging (the detent, 4x the electrical frequency on a two-phase stepper)
//! and commutation error (1x and 2x) are torques fixed to the rotor's
//! electrical angle. At a slow, constant speed the drive's velocity loop
//! cancels them, so the current it spends, set against the electrical phase
//! the capture records beside it, is the feedforward that would do the same
//! job without waiting for the error. At speed the same fit, applied to the
//! loop's speed instead, measures how much ripple is left.

use std::f64::consts::TAU;

use par6_config::RippleHarmonic;

/// Counts per electrical cycle in the captured phase channel.
const PHASE_COUNTS: f64 = 16384.0;
/// Least feedforward a harmonic must have been sent \[mA\] for [`refine`] to
/// read how the ripple answers it.
const REFINE_MIN_SENT_MA: f64 = 2.0;

/// One harmonic of a signal against the electrical phase: `a cos(h phase) +
/// b sin(h phase)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Harmonic {
    /// Harmonic of the electrical phase.
    pub harmonic: u8,
    /// Cosine amplitude.
    pub a: f64,
    /// Sine amplitude.
    pub b: f64,
}

impl Harmonic {
    /// Peak amplitude.
    pub fn amplitude(&self) -> f64 {
        self.a.hypot(self.b)
    }
}

/// Least squares of `values` against `harmonics` of the electrical `phase`
/// (0..16383 per cycle, sample by sample), from sample `skip` on. A
/// quadratic in time rides along and is thrown away: gravity and friction
/// drift slowly over a sweep, and would otherwise leak into the harmonics.
/// `None` when the samples do not pin the harmonics down (too few, or the
/// phase barely moved).
pub fn fit(values: &[f64], phase: &[f64], skip: usize, harmonics: &[u8]) -> Option<Vec<Harmonic>> {
    let n = values.len().min(phase.len());
    let columns = 3 + 2 * harmonics.len();
    if n <= skip + 4 * columns {
        return None;
    }
    let span = (n - skip - 1).max(1) as f64;
    let mut normal = vec![0.0; columns * columns];
    let mut rhs = vec![0.0; columns];
    let mut row = vec![0.0; columns];
    for k in skip..n {
        let t = 2.0 * (k - skip) as f64 / span - 1.0;
        row[0] = 1.0;
        row[1] = t;
        row[2] = t * t;
        let angle = TAU * phase[k] / PHASE_COUNTS;
        for (i, h) in harmonics.iter().enumerate() {
            let x = f64::from(*h) * angle;
            row[3 + 2 * i] = x.cos();
            row[4 + 2 * i] = x.sin();
        }
        for r in 0..columns {
            rhs[r] += row[r] * values[k];
            for c in 0..columns {
                normal[r * columns + c] += row[r] * row[c];
            }
        }
    }
    let solution = solve(&mut normal, &mut rhs, columns)?;
    Some(
        harmonics
            .iter()
            .enumerate()
            .map(|(i, h)| Harmonic {
                harmonic: *h,
                a: solution[3 + 2 * i],
                b: solution[4 + 2 * i],
            })
            .collect(),
    )
}

/// Root-sum-square amplitude of a fit: one number for how much ripple there
/// is across the harmonics.
pub fn total(harmonics: &[Harmonic]) -> f64 {
    harmonics
        .iter()
        .map(|h| h.a * h.a + h.b * h.b)
        .sum::<f64>()
        .sqrt()
}

/// One secant step per harmonic, at running speed. `v0` is the speed ripple
/// without feedforward and `v1` with `sent`, both against the electrical
/// phase; as phasors they give how the ripple answers the feedforward at
/// that harmonic, and the step returns the feedforward that would null `v0`,
/// each harmonic's amplitude capped at `cap` \[mA\]. A harmonic whose answer
/// cannot be told (too little was sent, or nothing moved) keeps what was
/// sent. `sent`, `v0` and `v1` list the same harmonics in the same order.
pub fn refine(
    sent: &[RippleHarmonic],
    v0: &[Harmonic],
    v1: &[Harmonic],
    cap: f64,
) -> Vec<RippleHarmonic> {
    // a cos + b sin as the phasor a - jb, so a linear response multiplies it.
    let phasor = |a: f64, b: f64| (a, -b);
    let mul = |(p, q): (f64, f64), (r, s): (f64, f64)| (p * r - q * s, p * s + q * r);
    let div = |x: (f64, f64), (r, s): (f64, f64)| {
        let d = r * r + s * s;
        mul(x, (r / d, -s / d))
    };
    sent.iter()
        .zip(v0.iter().zip(v1))
        .map(|(f, (h0, h1))| {
            let zf = phasor(f64::from(f.a_ma), f64::from(f.b_ma));
            let z0 = phasor(h0.a, h0.b);
            let moved = (h1.a - h0.a, -(h1.b - h0.b));
            if zf.0.hypot(zf.1) < REFINE_MIN_SENT_MA || moved.0.hypot(moved.1) <= f64::EPSILON {
                return *f;
            }
            let (mut re, mut im) = div(mul((-z0.0, -z0.1), zf), moved);
            let size = re.hypot(im);
            if size > cap {
                re *= cap / size;
                im *= cap / size;
            }
            RippleHarmonic {
                harmonic: f.harmonic,
                a_ma: re.round() as i16,
                b_ma: (-im).round() as i16,
            }
        })
        .collect()
}

/// Gaussian elimination with partial pivoting on an `n` x `n` system;
/// `None` if it is singular.
pub(crate) fn solve(a: &mut [f64], b: &mut [f64], n: usize) -> Option<Vec<f64>> {
    for col in 0..n {
        let pivot =
            (col..n).max_by(|x, y| a[x * n + col].abs().total_cmp(&a[y * n + col].abs()))?;
        let scale = (0..n)
            .map(|c| a[col * n + c].abs())
            .fold(0.0, f64::max)
            .max(1.0);
        if a[pivot * n + col].abs() <= 1e-12 * scale {
            return None;
        }
        if pivot != col {
            for c in 0..n {
                a.swap(pivot * n + c, col * n + c);
            }
            b.swap(pivot, col);
        }
        for r in col + 1..n {
            let f = a[r * n + col] / a[col * n + col];
            for c in col..n {
                a[r * n + c] -= f * a[col * n + c];
            }
            b[r] -= f * b[col];
        }
    }
    let mut x = vec![0.0; n];
    for r in (0..n).rev() {
        let sum: f64 = (r + 1..n).map(|c| a[r * n + c] * x[c]).sum();
        x[r] = (b[r] - sum) / a[r * n + r];
    }
    Some(x)
}
