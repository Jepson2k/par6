//! The velocity loop's frequency response, measured at the drive's own rate,
//! and the PI gains it supports.
//!
//! The periodic capture path adds a known multisine to the drive's velocity
//! loop setpoint (cmd 43) and records the setpoint it
//! commanded beside the speed the loop acted on. Against that excitation the
//! ratio of two cross spectra is the plant as the loop sees it -- current
//! loop, motor, arm and belt, speed filter and the loop's own delay -- free
//! of the feedback around it and of anything the excitation did not cause.
//! The drive's PI is known exactly, so the loop any pair of gains would
//! close can be assessed on the measured frequencies before it runs: [`design`] takes the gains with
//! the most integral action every measured loop allows at a bounded
//! sensitivity peak (Åström and Hägglund's MIGO rule).

use std::f64::consts::TAU;
use std::ops::{Add, Div, Mul, Sub};

use par6_bus::spectral::codec::inject_sequence;
use par6_bus::spectral::periodic;

/// The drive's control loop rate \[Hz\].
pub const LOOP_HZ: f64 = 6250.0;
/// The band the response is estimated over: from the capture's third
/// frequency step, since the Hann window's main lobe spans two steps either
/// side and below that what the detrend leaves of the slide leaks in, up to
/// where the speed filter's nulls have swallowed the plant \[Hz\].
const FIRST_BIN: usize = 3;
const F_HI_HZ: f64 = 800.0;
/// Neighbouring frequencies pool into bands at most this wide, as the ratio
/// of their ends, unless that leaves fewer than `MIN_AVERAGES` frequency-
/// captures in a band.
const BAND_RATIO: f64 = 1.1;
const MIN_AVERAGES: usize = 6;
/// A band whose coherence with the injection is under this is not
/// measured: too much of what the capture shows there the injection did not
/// cause. Frequency bins under a Hann window are correlated: their count is
/// not a count of independent experiments or a statistical error bound.
pub const COHERENCE_MIN: f64 = 0.6;
/// The largest sensitivity peak designed gains may give any measured loop:
/// at least 9.5 dB of gain margin and 39° of phase margin. Åström and
/// Hägglund put reasonable values between 1.2 and 2 (*Advanced PID
/// Control*, 2006, §4.2). This bound applies to the measured response, not
/// unmeasured dynamics or a confidence interval on the real arm.
pub const MS_MAX: f64 = 1.5;
/// A loop is judged only where the measurement covers its crossover: the
/// lowest measured band at an open-loop gain of at least `COVERED_ABOVE`,
/// the highest at most `COVERED_BELOW`, and no unmeasured band where it
/// crosses between them.
const COVERED_ABOVE: f64 = 2.0;
const COVERED_BELOW: f64 = 0.5;
/// Where the open loop's principal phase says it is past −180° \[rad\].
/// This loop -- a PI (−90° to 0°) on an inertia (−90°) behind the speed
/// filter's and the drive's delays -- lies between −270° and 0°, except
/// where a structural mode lifts it a little above 0° (J1's anti-resonance
/// near 28 Hz: +9°). A principal phase between this and 180° is therefore
/// the loop past −180°, and at a gain above one there its Nyquist curve
/// goes around −1: the loop is unstable however far the sampled points sit
/// from −1.
const PAST_HALF_TURN: f64 = std::f64::consts::FRAC_PI_2;
/// The alignment check's shifts \[samples\]: the injection ahead of the
/// setpoint, which cannot have moved it yet, and behind it, where the loop's
/// answer to it lies. The loop answers within a few samples at every capture
/// set's rate; beyond that the injection's bits are independent of the one
/// being weighed.
const ALIGN_LEAD: usize = 3;
const ALIGN_LAG: usize = 8;
/// Half a pass-through: the least the unshifted weight may be, and more than
/// any weight ahead may. A sample taken some loops into its injection bit
/// already carries the loop's answer to that bit, so its weight is under
/// one (J1: 1.0 three and seven loops in, 0.7 eleven and twenty-three in),
/// and a loop ringing through the capture puts some tenths on every shift
/// (J1 at 150 Hz: 0.16 ahead). A capture out of step by a sample moves the
/// pass-through ahead or takes it off the unshifted injection.
const ALIGN_HALF: f64 = 0.5;
/// Where the PI the capture saw act is compared with the drive's own \[Hz\].
const CHECK_HZ: (f64, f64) = (20.0, 300.0);
/// How far the median of that comparison may stray before the capture is
/// not taken to show the loop its gains describe.
pub const CONTROLLER_TOLERANCE: f64 = 0.25;
/// The design grid: factors of the configured gains either way, and points
/// across that span.
const KPV_SPAN: f64 = 8.0;
const KPV_POINTS: usize = 49;
const KIV_SPAN: f64 = 30.0;
const KIV_POINTS: usize = 121;
/// A step's tracking is scored from where its speed first reaches this
/// share of the step.
const SCORE_FROM: f64 = 0.1;

/// A complex number, as much of one as this module needs.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Complex {
    /// Real part.
    pub re: f64,
    /// Imaginary part.
    pub im: f64,
}

impl Complex {
    /// `re + j im`.
    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }
    fn cis(angle: f64) -> Self {
        Self::new(angle.cos(), angle.sin())
    }
    fn conj(self) -> Self {
        Self::new(self.re, -self.im)
    }
    fn norm_sqr(self) -> f64 {
        self.re * self.re + self.im * self.im
    }
    /// Magnitude.
    pub fn abs(self) -> f64 {
        self.re.hypot(self.im)
    }
    /// Phase \[rad\].
    pub fn arg(self) -> f64 {
        self.im.atan2(self.re)
    }
}

impl Add for Complex {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.re + o.re, self.im + o.im)
    }
}

impl Sub for Complex {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.re - o.re, self.im - o.im)
    }
}

impl Mul for Complex {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        Self::new(
            self.re * o.re - self.im * o.im,
            self.re * o.im + self.im * o.re,
        )
    }
}

impl Div for Complex {
    type Output = Self;
    fn div(self, o: Self) -> Self {
        let d = o.norm_sqr();
        let n = self * o.conj();
        Self::new(n.re / d, n.im / d)
    }
}

/// One capture taken with an injection, every channel over the same
/// samples.
#[derive(Clone, Debug)]
pub struct Excited {
    /// The current the injection added at each sample \[mA\].
    pub injection: Vec<f64>,
    /// The current setpoint the loop commanded, injection included \[mA\].
    pub setpoint: Vec<f64>,
    /// The speed the loop acted on \[ticks/s\].
    pub speed: Vec<f64>,
}

/// The current an injection of `amplitude_ma`, `seed` and `hold` added at
/// each of `samples` samples recorded one every `divisor` loops: the drive
/// takes sample k on loop (k + 1) * divisor - 1 of the capture.
pub fn injected(
    amplitude_ma: f64,
    seed: u16,
    hold: u8,
    divisor: usize,
    samples: usize,
) -> Vec<f64> {
    let divisor = divisor.max(1);
    let signs = inject_sequence(seed, hold, samples * divisor);
    (0..samples)
        .map(|k| amplitude_ma * signs[(k + 1) * divisor - 1])
        .collect()
}

/// Whether the setpoint carries the injection sample for sample. The drive
/// adds each loop's injection straight onto the setpoint it records, while
/// the rest of the setpoint -- the PI acting on a speed measured before the
/// injection could move it -- only answers injections already applied. So a
/// least-squares fit of the setpoint on the injection at every shift from
/// `ALIGN_LEAD` samples ahead to `ALIGN_LAG` behind weighs the unshifted
/// injection at most of one and those ahead at next to nothing. A drive
/// that ran no injection, or ran it out of step with its capture, fails;
/// `Err` carries the shift weighed most.
pub fn check_alignment(e: &Excited) -> Result<(), i32> {
    if !e.valid() {
        return Err(0);
    }
    let n = e.setpoint.len().min(e.injection.len());
    let columns = ALIGN_LEAD + 1 + ALIGN_LAG + 1;
    if n < ALIGN_LEAD + ALIGN_LAG + 4 * columns {
        return Err(0);
    }
    let setpoint = detrended(&e.setpoint[..n]);
    let mut normal = vec![0.0; columns * columns];
    let mut rhs = vec![0.0; columns];
    let mut row = vec![0.0; columns];
    for (k, target) in setpoint
        .iter()
        .enumerate()
        .take(n - ALIGN_LEAD)
        .skip(ALIGN_LAG)
    {
        // Injection shifted from ALIGN_LEAD ahead down to ALIGN_LAG behind,
        // then a constant.
        for (c, slot) in row.iter_mut().take(columns - 1).enumerate() {
            *slot = e.injection[k + ALIGN_LEAD - c];
        }
        row[columns - 1] = 1.0;
        for r in 0..columns {
            rhs[r] += row[r] * target;
            for c in 0..columns {
                normal[r * columns + c] += row[r] * row[c];
            }
        }
    }
    let Some(weights) = crate::ripple::solve(&mut normal, &mut rhs, columns) else {
        return Err(0);
    };
    let own = weights[ALIGN_LEAD];
    let ahead = weights[..ALIGN_LEAD]
        .iter()
        .fold(0.0_f64, |m, w| m.max(w.abs()));
    if own >= ALIGN_HALF && ahead < ALIGN_HALF {
        Ok(())
    } else {
        let most = (0..columns - 1)
            .max_by(|a, b| weights[*a].total_cmp(&weights[*b]))
            .unwrap_or(ALIGN_LEAD);
        Err(ALIGN_LEAD as i32 - most as i32)
    }
}

/// The loop's response at one pose, band by band.
#[derive(Clone, Debug, Default)]
pub struct Response {
    /// Band centres \[Hz\].
    pub hz: Vec<f64>,
    /// Speed per current setpoint \[ticks/s per mA\]: everything the loop
    /// drives, as it sees it.
    pub plant: Vec<Complex>,
    /// The PI as the capture saw it act \[mA per ticks/s\].
    pub controller: Vec<Complex>,
    /// Coherence with the injection of the weaker of speed and setpoint.
    pub coherence: Vec<f64>,
    /// Absolute plant uncertainty radius from period scatter (two standard
    /// errors, with a quantization floor). Not a stability confidence proof.
    pub uncertainty: Vec<f64>,
}

impl Excited {
    fn valid(&self) -> bool {
        let n = self.injection.len();
        n >= 64
            && self.setpoint.len() == n
            && self.speed.len() == n
            && self
                .injection
                .iter()
                .chain(&self.setpoint)
                .chain(&self.speed)
                .all(|v| v.is_finite())
    }
}

impl Response {
    fn valid(&self) -> bool {
        let n = self.hz.len();
        n > 0
            && self.plant.len() == n
            && self.controller.len() == n
            && self.coherence.len() == n
            && self.uncertainty.len() == n
            && self.uncertainty.iter().all(|v| v.is_finite() && *v >= 0.0)
            && self.hz.iter().all(|f| f.is_finite() && *f > 0.0)
            && self.hz.windows(2).all(|w| w[0] < w[1])
            && self
                .plant
                .iter()
                .chain(&self.controller)
                .all(|p| p.re.is_finite() && p.im.is_finite())
            && self
                .coherence
                .iter()
                .all(|c| c.is_finite() && (0.0..=1.0).contains(c))
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct Pool {
    hz: f64,
    dd: f64,
    uu: f64,
    ww: f64,
    du: Complex,
    dw: Complex,
    count: usize,
}

impl Pool {
    fn add(&mut self, d: Complex, u: Complex, w: Complex) {
        self.dd += d.norm_sqr();
        self.uu += u.norm_sqr();
        self.ww += w.norm_sqr();
        self.du = self.du + d.conj() * u;
        self.dw = self.dw + d.conj() * w;
        self.count += 1;
    }
    fn merge(&mut self, o: &Pool) {
        self.dd += o.dd;
        self.uu += o.uu;
        self.ww += o.ww;
        self.du = self.du + o.du;
        self.dw = self.dw + o.dw;
        self.count += o.count;
    }
}

fn detrended(x: &[f64]) -> Vec<f64> {
    let n = x.len() as f64;
    let mid = (n - 1.0) / 2.0;
    let mean = x.iter().sum::<f64>() / n;
    let (mut sxy, mut sxx) = (0.0, 0.0);
    for (k, v) in x.iter().enumerate() {
        let t = k as f64 - mid;
        sxy += t * (v - mean);
        sxx += t * t;
    }
    let slope = if sxx > 0.0 { sxy / sxx } else { 0.0 };
    x.iter()
        .enumerate()
        .map(|(k, v)| v - mean - slope * (k as f64 - mid))
        .collect()
}

/// The response from the captures made at one pose, recorded one sample
/// every `divisor` loops: Hann-windowed spectra of each capture, the cross
/// spectra with the injection pooled over captures and neighbouring
/// frequencies. `None` when the captures are too short to cover the band.
pub fn measure(captures: &[Excited], divisor: usize) -> Option<Response> {
    if divisor == 0 || captures.iter().any(|c| !c.valid()) {
        return None;
    }
    let n = captures
        .iter()
        .map(|c| c.injection.len().min(c.setpoint.len()).min(c.speed.len()))
        .min()?;
    if n < 64 {
        return None;
    }
    let rate = LOOP_HZ / divisor.max(1) as f64;
    let step_hz = rate / n as f64;
    let first = FIRST_BIN;
    let last = (F_HI_HZ.min(0.45 * rate) / step_hz).floor() as usize;
    if last < first {
        return None;
    }
    let window: Vec<f64> = (0..n)
        .map(|k| 0.5 - 0.5 * (TAU * k as f64 / (n - 1) as f64).cos())
        .collect();
    let turns: Vec<Complex> = (0..n)
        .map(|i| Complex::cis(-TAU * i as f64 / n as f64))
        .collect();
    let dft = |x: &[f64], k: usize| {
        x.iter()
            .enumerate()
            .fold(Complex::default(), |sum, (i, v)| {
                let t = turns[(k * i) % n];
                Complex::new(sum.re + v * t.re, sum.im + v * t.im)
            })
    };
    let mut bins: Vec<Pool> = (first..=last)
        .map(|k| Pool {
            hz: k as f64 * step_hz,
            ..Pool::default()
        })
        .collect();
    for c in captures {
        let [d, u, w] = [&c.injection, &c.setpoint, &c.speed].map(|x| {
            detrended(&x[..n])
                .iter()
                .zip(&window)
                .map(|(v, h)| v * h)
                .collect::<Vec<f64>>()
        });
        for (b, pool) in bins.iter_mut().enumerate() {
            let k = first + b;
            pool.add(dft(&d, k), dft(&u, k), dft(&w, k));
        }
    }

    let mut response = Response::default();
    // A band pools bins: their summed frequencies, the first bin's, and how
    // many.
    let mut band: Option<(Pool, f64, usize)> = None;
    for bin in &bins {
        match &mut band {
            Some((pool, from, count))
                if pool.count < MIN_AVERAGES || bin.hz / *from <= BAND_RATIO =>
            {
                pool.merge(bin);
                pool.hz += bin.hz;
                *count += 1;
            }
            _ => {
                if let Some((pool, _, count)) = band.take() {
                    push_band(&mut response, pool, count);
                }
                band = Some((*bin, bin.hz, 1));
            }
        }
    }
    if let Some((pool, _, count)) = band {
        push_band(&mut response, pool, count);
    }
    (!response.hz.is_empty()).then_some(response)
}

/// One band's plant, PI and coherence from its pooled spectra; a band too
/// shallow to average is left out.
fn push_band(response: &mut Response, pool: Pool, bins: usize) {
    if pool.count < MIN_AVERAGES || pool.dd <= 0.0 {
        return;
    }
    let (du, dw) = (pool.du, pool.dw);
    let coherence = |cross: Complex, own: f64| {
        if own > 0.0 {
            (cross.norm_sqr() / (pool.dd * own)).clamp(0.0, 1.0)
        } else {
            0.0
        }
    };
    let usable = du.norm_sqr() > 0.0 && dw.norm_sqr() > 0.0;
    response.hz.push(pool.hz / bins as f64);
    // Legacy PRBS replay has no independent period-scatter estimate.
    response.uncertainty.push(0.0);
    if usable {
        // The PI's output is the setpoint less the injection, c = u - d, and
        // acts on the speed with the sign the loop feeds it back.
        response.plant.push(dw / du);
        response
            .controller
            .push(Complex::default() - (du - Complex::new(pool.dd, 0.0)) / dw);
        response
            .coherence
            .push(coherence(dw, pool.ww).min(coherence(du, pool.uu)));
    } else {
        response.plant.push(Complex::default());
        response.controller.push(Complex::default());
        response.coherence.push(0.0);
    }
}

/// Synchronous DFT of four complete periods, averaging only the SAME frequency.
/// Schoukens/Godfrey/Schoukens, IEEE Control Systems 2018,
/// doi:10.1109/MCS.2018.2830080: periodic excitation, reference-channel
/// cross spectra for closed-loop identification, and period-to-period noise.
/// Two standard errors is our design guard, not a claimed confidence level;
/// repeated periods cannot detect every systematic nonlinear distortion.
pub fn measure_periodic(e: &Excited, spec: periodic::Spec) -> Option<Response> {
    if !spec.valid()
        || spec.peak_ma == 0
        || !e.valid()
        || e.speed.len() != periodic::SAMPLES * periodic::PERIODS
        || e.injection != spec.samples()
    {
        return None;
    }
    let n = periodic::SAMPLES;
    let m = periodic::PERIODS as f64;
    // The joint traverses an arc, so gravity/current bias may drift. Fit
    // that linear background from whole-period means: a periodic component
    // has the same mean each period and cannot bias this slope estimate.
    let background_removed = |x: &[f64]| {
        let mid = (m - 1.0) / 2.0;
        let mut numerator = 0.0;
        let mut denominator = 0.0;
        for (p, period) in x.chunks_exact(n).enumerate() {
            let offset = p as f64 - mid;
            numerator += offset * period.iter().sum::<f64>() / n as f64;
            denominator += offset * offset;
        }
        let slope = numerator / (denominator * n as f64);
        x.iter()
            .enumerate()
            .map(|(i, x)| x - slope * i as f64)
            .collect::<Vec<_>>()
    };
    let setpoint = background_removed(&e.setpoint);
    let speed = background_removed(&e.speed);
    let dft = |x: &[f64], k: usize| {
        x.iter()
            .enumerate()
            .fold(Complex::default(), |sum, (t, x)| {
                sum + Complex::cis(-TAU * (k * t) as f64 / n as f64)
                    * Complex::new(*x / n as f64, 0.0)
            })
    };
    let mut result = Response::default();
    for k in spec.bins() {
        let mut pool = Pool::default();
        let mut values = Vec::with_capacity(periodic::PERIODS);
        for p in 0..periodic::PERIODS {
            let range = p * n..(p + 1) * n;
            let d = dft(&e.injection[range.clone()], k);
            let u = dft(&setpoint[range.clone()], k);
            let w = dft(&speed[range], k);
            pool.add(d, u, w);
            values.push((u, w));
        }
        let hz = k as f64 * LOOP_HZ / (n * usize::from(spec.divisor())) as f64;
        result.hz.push(hz);
        let zero = Complex::default();
        let coherent = pool.dd > 0.0
            && pool.uu > 0.0
            && pool.ww > 0.0
            && pool.du.norm_sqr() > 0.0
            && pool.dw.norm_sqr() > 0.0;
        if !coherent {
            result.plant.push(zero);
            result.controller.push(zero);
            result.coherence.push(0.0);
            result.uncertainty.push(0.0);
            continue;
        }
        let plant = pool.dw / pool.du;
        let controller = zero - (pool.du - Complex::new(pool.dd, 0.0)) / pool.dw;
        // A repeatable quantized zero is not a high-coherence measurement.
        let u_floor = m / (12.0 * n as f64);
        let w_floor = 16.0_f64.powi(2) * u_floor;
        let coherence = (pool.du.norm_sqr() / (pool.dd * (pool.uu + u_floor)))
            .min(pool.dw.norm_sqr() / (pool.dd * (pool.ww + w_floor)))
            .clamp(0.0, 1.0);
        let mean_u = values.iter().fold(zero, |sum, (u, _)| sum + *u) / Complex::new(m, 0.0);
        let scatter = values
            .iter()
            .map(|(u, w)| (*w - plant * *u).norm_sqr())
            .sum::<f64>()
            / (m * (m - 1.0));
        // Uniform quantization model, deliberately NOT reduced by the number
        // of repeated periods: identical quantization errors can recur.
        let quantization = (16.0_f64.powi(2) + plant.norm_sqr()) / (12.0 * n as f64);
        let radius = 2.0 * ((scatter + quantization) / mean_u.norm_sqr()).sqrt();
        result.plant.push(plant);
        result.controller.push(controller);
        result.coherence.push(coherence);
        result.uncertainty.push(radius);
    }
    result.valid().then_some(result)
}

/// One response from two measured over overlapping bands: `low`'s bands
/// under `split_hz`, `high`'s from there up. Each set is used where its
/// excitation is strongest.
pub fn merge(low: &Response, high: &Response, split_hz: f64) -> Response {
    let mut out = Response::default();
    let bands = |r: &Response, keep: &dyn Fn(f64) -> bool| {
        (0..r.hz.len())
            .filter(|i| keep(r.hz[*i]))
            .map(|i| {
                (
                    r.hz[i],
                    r.plant[i],
                    r.controller[i],
                    r.coherence[i],
                    r.uncertainty[i],
                )
            })
            .collect::<Vec<_>>()
    };
    for (hz, plant, controller, coherence, uncertainty) in bands(low, &|hz| hz < split_hz)
        .into_iter()
        .chain(bands(high, &|hz| hz >= split_hz))
    {
        out.hz.push(hz);
        out.plant.push(plant);
        out.controller.push(controller);
        out.coherence.push(coherence);
        out.uncertainty.push(uncertainty);
    }
    out
}

/// The drive's PI at `hz` \[mA per ticks/s\]: `kpv + kiv / (1 - z^-1)`,
/// the integral stepped once a loop with that loop's error, as
/// `Velocity_mode()` runs it.
pub fn pi(kpv: f64, kiv: f64, hz: f64) -> Complex {
    let one = Complex::new(1.0, 0.0);
    Complex::new(kpv, 0.0) + Complex::new(kiv, 0.0) / (one - Complex::cis(-TAU * hz / LOOP_HZ))
}

/// How far the PI the capture saw act strays from the drive's PI at the
/// gains in force: the median of |seen / expected - 1| over the coherent
/// bands in `CHECK_HZ`. `None` without any.
pub fn controller_error(r: &Response, kpv: f64, kiv: f64) -> Option<f64> {
    if !r.valid() || !kpv.is_finite() || !kiv.is_finite() || kpv <= 0.0 || kiv <= 0.0 {
        return None;
    }
    let one = Complex::new(1.0, 0.0);
    let mut errors: Vec<f64> =
        r.hz.iter()
            .zip(&r.controller)
            .zip(&r.coherence)
            .filter(|((hz, _), c)| **c >= COHERENCE_MIN && (CHECK_HZ.0..=CHECK_HZ.1).contains(*hz))
            .map(|((hz, seen), _)| (*seen / pi(kpv, kiv, *hz) - one).abs())
            .collect();
    if errors.is_empty() || errors.iter().any(|e| !e.is_finite()) {
        return None;
    }
    errors.sort_by(f64::total_cmp);
    Some(errors[errors.len() / 2])
}

/// Diagnostic coverage of 10–40 Hz, the unresolved band in the J1 recordings.
/// A usable line has coherence >= 0.6 and a two-standard-error radius <= 35%
/// of its plant magnitude. This is an experiment gate, not a stability proof.
pub fn focused_quality(r: &Response) -> (usize, usize) {
    if !r.valid() {
        return (0, 0);
    }
    let mut usable = 0;
    let mut total = 0;
    for (i, hz) in r.hz.iter().enumerate() {
        if (10.0..=40.0).contains(hz) {
            total += 1;
            if r.coherence[i] >= COHERENCE_MIN
                && r.plant[i].abs() > 0.0
                && r.uncertainty[i] <= 0.35 * r.plant[i].abs()
            {
                usable += 1;
            }
        }
    }
    (usable, total)
}

/// Largest separation of two amplitude measurements relative to the sum of
/// their uncertainty radii. <= 1 means their disks overlap at every focused
/// line. Weak measurements cannot qualify merely by having large radii.
pub fn focused_agreement(a: &Response, b: &Response) -> Option<f64> {
    for r in [a, b] {
        let (usable, total) = focused_quality(r);
        if total == 0 || usable != total {
            return None;
        }
    }
    if a.hz != b.hz {
        return None;
    }
    let mut worst: f64 = 0.0;
    for (i, hz) in a.hz.iter().enumerate() {
        if (10.0..=40.0).contains(hz) {
            let distance = (a.plant[i] - b.plant[i]).abs();
            let radius = a.uncertainty[i] + b.uncertainty[i];
            let separation = if radius > 0.0 {
                distance / radius
            } else if distance == 0.0 {
                0.0
            } else {
                f64::INFINITY
            };
            worst = worst.max(separation);
        }
    }
    Some(worst)
}

/// How the loop a pair of gains would close on a measured response fares.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Judged {
    /// Sensitivity peak, the largest `1 / |1 + L|` over the measured bands.
    pub ms: f64,
    /// The first measured band \[Hz\] where the open loop falls under unity.
    pub crossover_hz: f64,
}

/// The loop `kpv`/`kiv` would close on `r`, its sensitivity peak infinite
/// when it would be unstable (`PAST_HALF_TURN`). `None` when the measurement
/// cannot vouch for that loop: its crossover lies beyond the coherent
/// bands, or runs through bands the capture could not resolve.
pub fn judge(r: &Response, kpv: f64, kiv: f64) -> Option<Judged> {
    if !r.valid() || !kpv.is_finite() || !kiv.is_finite() || kpv <= 0.0 || kiv <= 0.0 {
        return None;
    }
    let one = Complex::new(1.0, 0.0);
    let bands: Vec<(f64, Complex, bool, f64)> =
        r.hz.iter()
            .zip(&r.plant)
            .zip(&r.coherence)
            .zip(&r.uncertainty)
            .map(|(((hz, p), c), uncertainty)| {
                let controller = pi(kpv, kiv, *hz);
                (
                    *hz,
                    controller * *p,
                    *c >= COHERENCE_MIN,
                    controller.abs() * uncertainty,
                )
            })
            .collect();
    if bands
        .iter()
        .any(|(_, l, _, radius)| !l.abs().is_finite() || !radius.is_finite())
    {
        return None;
    }
    let first = bands.iter().position(|b| b.2)?;
    let last = bands.iter().rposition(|b| b.2)?;
    if bands[first].1.abs() - bands[first].3 < COVERED_ABOVE
        || bands[last].1.abs() + bands[last].3 > COVERED_BELOW
    {
        return None;
    }
    let below =
        (first..=last).find(|&i| bands[i].2 && bands[i].1.abs() - bands[i].3 < COVERED_ABOVE)?;
    let above = (first..=last)
        .rev()
        .find(|&i| bands[i].2 && bands[i].1.abs() + bands[i].3 > COVERED_BELOW)?;
    let from = below.min(above).saturating_sub(1).max(first);
    let to = (below.max(above) + 1).min(last);
    if (from..=to).any(|i| !bands[i].2) {
        return None;
    }
    let measured = || bands[first..=last].iter().filter(|b| b.2);
    let encircles = measured().any(|b| b.1.abs() >= 1.0 && b.1.arg() > PAST_HALF_TURN);
    let ms = if encircles {
        f64::INFINITY
    } else {
        measured()
            .map(|b| {
                let distance = (one + b.1).abs() - b.3;
                if distance > 0.0 {
                    1.0 / distance
                } else {
                    f64::INFINITY
                }
            })
            .fold(0.0, f64::max)
    };
    let crossover_hz = measured().find(|b| b.1.abs() < 1.0)?.0;
    Some(Judged { ms, crossover_hz })
}

/// Gains the design chose, and how the worst measured loop fares with them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Design {
    /// Proportional gain \[mA per ticks/s\].
    pub kpv: f64,
    /// Integral gain \[mA per ticks/s, per loop\].
    pub kiv: f64,
    /// The highest sensitivity peak across the responses.
    pub ms: f64,
    /// The lowest crossover across the responses \[Hz\].
    pub crossover_hz: f64,
}

fn worst(responses: &[Response], kpv: f64, kiv: f64) -> Option<Judged> {
    responses.iter().try_fold(
        Judged {
            ms: 0.0,
            crossover_hz: f64::INFINITY,
        },
        |w, r| {
            let j = judge(r, kpv, kiv)?;
            Some(Judged {
                ms: w.ms.max(j.ms),
                crossover_hz: w.crossover_hz.min(j.crossover_hz),
            })
        },
    )
}

/// How every response fares under `kpv`/`kiv`: the worst sensitivity peak
/// and lowest crossover, or `None` if any response cannot vouch for them.
pub fn judge_all(responses: &[Response], kpv: f64, kiv: f64) -> Option<Judged> {
    if responses.is_empty() {
        return None;
    }
    worst(responses, kpv, kiv)
}

/// The gains with the most integral action every response allows at a
/// sensitivity peak of at most [`MS_MAX`], kpv at most `kpv_max`: MIGO
/// (Åström, Panagopoulos and Hägglund, "Design of PI controllers based on
/// non-convex optimization", Automatica 34(5), 1998), whose objective, the
/// integral gain, is what sets a PI loop's rejection of load disturbances.
/// Searched
/// on a log grid `KPV_SPAN` and `KIV_SPAN` either side of `configured`
/// (kpv, kiv): for each kpv, the highest kiv of the passing run the grid
/// climbs into. `None` when nothing passes, or when the best sits on the
/// grid's top edge in kiv -- the measurement did not bound it there.
pub fn design(responses: &[Response], configured: (f64, f64), kpv_max: f64) -> Option<Design> {
    match search(responses, configured, kpv_max) {
        Some((d, false)) => Some(d),
        _ => None,
    }
}

/// Whether any gain pair is supported. Unlike final design, a feasible pair
/// at the grid edge may justify measuring the next pose: that pose can bound
/// the common optimum even when the first pose alone cannot.
pub fn supports_design(responses: &[Response], configured: (f64, f64), kpv_max: f64) -> bool {
    search(responses, configured, kpv_max).is_some()
}

fn search(responses: &[Response], configured: (f64, f64), kpv_max: f64) -> Option<(Design, bool)> {
    if responses.is_empty()
        || responses.iter().any(|r| !r.valid())
        || [configured.0, configured.1, kpv_max]
            .iter()
            .any(|v| !v.is_finite() || *v <= 0.0)
    {
        return None;
    }
    let grid = |centre: f64, span: f64, points: usize, i: usize| {
        centre * span.powf(2.0 * i as f64 / (points - 1) as f64 - 1.0)
    };
    let mut best: Option<(Design, bool)> = None;
    for p in 0..KPV_POINTS {
        let kpv = grid(configured.0, KPV_SPAN, KPV_POINTS, p);
        if kpv > kpv_max {
            break;
        }
        let mut found = None;
        for m in 0..KIV_POINTS {
            let kiv = grid(configured.1, KIV_SPAN, KIV_POINTS, m);
            match worst(responses, kpv, kiv) {
                Some(w) if w.ms <= MS_MAX => {
                    found = Some((
                        Design {
                            kpv,
                            kiv,
                            ms: w.ms,
                            crossover_hz: w.crossover_hz,
                        },
                        m == KIV_POINTS - 1,
                    ))
                }
                _ if found.is_some() => break,
                _ => {}
            }
        }
        if let Some((d, edge)) = found {
            let better = best.is_none_or(|(b, _)| {
                d.kiv > b.kiv * (1.0 + 1e-9) || (d.kiv >= b.kiv * (1.0 - 1e-9) && d.ms < b.ms)
            });
            if better {
                best = Some((d, edge));
            }
        }
    }
    best
}

/// A capture turned into one positive step: the loop's speed \[ticks/s\]
/// per sample, sign flipped when the step was negative.
#[derive(Clone, Debug, PartialEq)]
pub struct Record {
    /// The loop's filtered speed.
    pub speed: Vec<f64>,
    /// The step's size \[ticks/s\], positive.
    pub step: f64,
}

impl Record {
    /// A capture of a step of `step` ticks/s, `speed` as the drive reported
    /// it.
    pub fn new(speed: &[f64], step: f64) -> Self {
        let sign = step.signum();
        Self {
            speed: speed.iter().map(|v| v * sign).collect(),
            step: step.abs(),
        }
    }

    /// Tracking error per unit step: RMS of speed minus the step from where
    /// the speed first reaches `SCORE_FROM` of it. Slow rise and ringing
    /// both cost; how long the joint took to break away does not, which is
    /// its friction and preload, not its gains.
    pub fn score(&self) -> f64 {
        if !self.step.is_finite() || self.step <= 0.0 || self.speed.iter().any(|v| !v.is_finite()) {
            return f64::INFINITY;
        }
        let from = self
            .speed
            .iter()
            .position(|v| *v >= SCORE_FROM * self.step)
            .unwrap_or(self.speed.len());
        let tail = &self.speed[from..];
        if tail.is_empty() || self.step == 0.0 {
            return f64::INFINITY;
        }
        let sum: f64 = tail.iter().map(|v| (v - self.step).powi(2)).sum();
        (sum / tail.len() as f64).sqrt() / self.step
    }
}

/// A trial must meet the application's relative RMS limit and must not
/// worsen the matched baseline. Non-finite scores include runaway and
/// captures where the motor never reached the scoring window.
pub fn accept_step(candidate: f64, baseline: f64, limit: f64) -> bool {
    candidate.is_finite()
        && baseline.is_finite()
        && limit.is_finite()
        && candidate >= 0.0
        && baseline >= 0.0
        && limit > 0.0
        && candidate <= baseline
        && candidate <= limit
}
