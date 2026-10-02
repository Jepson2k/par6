//! Offline replay of physical J1 captures; no bus, daemon or simulated plant.

use par6d::loop_response::{self as response, Excited, Record};

const FAST: &str = include_str!("data/loop-response/j1-fast.csv");
const SLOW: &str = include_str!("data/loop-response/j1-slow.csv");

fn excited(csv: &str) -> Excited {
    let mut capture = Excited {
        injection: Vec::new(),
        setpoint: Vec::new(),
        speed: Vec::new(),
    };
    for line in csv.lines().skip(2) {
        let row: Vec<f64> = line.split(',').map(|v| v.parse().unwrap()).collect();
        capture.injection.push(row[2]);
        capture.setpoint.push(row[3]);
        capture.speed.push(row[4]);
    }
    capture
}

#[test]
fn a_short_hardware_pair_is_identified_but_does_not_authorize_unsupported_gains() {
    let fast = excited(FAST);
    let slow = excited(SLOW);
    assert!(response::check_alignment(&fast).is_ok());
    assert!(response::check_alignment(&slow).is_ok());
    let mut shifted = fast.clone();
    shifted.injection.rotate_left(3);
    assert!(response::check_alignment(&shifted).is_err());
    let high = response::measure(&[fast], 4).unwrap();
    let low = response::measure(&[slow], 12).unwrap();
    let measured = response::merge(&low, &high, 40.0);
    let error = response::controller_error(&measured, 0.02, 0.003).unwrap();
    assert!(error < response::CONTROLLER_TOLERANCE, "PI error {error}");
    // The actual low band has gaps. A short experiment must report that,
    // rather than infer that a correctly captured PI proves plant coverage.
    assert!(response::judge(&measured, 0.02, 0.003).is_none());
    assert!(!response::supports_design(
        std::slice::from_ref(&measured),
        (0.02, 0.003),
        0.0256
    ));
    assert!(response::design(std::slice::from_ref(&measured), (0.02, 0.003), 0.0256).is_none());

    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut capture = excited(FAST);
        capture.speed[200] = invalid;
        assert!(response::measure(std::slice::from_ref(&capture), 4).is_none());
        assert!(response::check_alignment(&capture).is_err());
        let mut broken = measured.clone();
        broken.plant[0].re = invalid;
        assert!(response::controller_error(&broken, 0.02, 0.003).is_none());
        assert!(response::design(&[broken], (0.02, 0.003), 0.0256).is_none());
    }
    let mut partial = excited(FAST);
    partial.setpoint.pop();
    assert!(response::measure(&[partial], 4).is_none());
    assert!(response::measure(&[excited(FAST)], 0).is_none());
}

fn step(csv: &str) -> Record {
    let command = csv
        .lines()
        .next()
        .unwrap()
        .split_whitespace()
        .nth(1)
        .unwrap()
        .strip_prefix("step_ticks_s=")
        .unwrap()
        .parse()
        .unwrap();
    let speed: Vec<f64> = csv
        .lines()
        .skip(2)
        .map(|line| line.split(',').nth(2).unwrap().parse().unwrap())
        .collect();
    Record::new(&speed, command)
}

#[test]
fn physical_trials_must_meet_tracking_and_must_not_regress() {
    let baseline = step(include_str!("data/loop-response/j1-baseline.csv")).score();
    let worse = step(include_str!("data/loop-response/j1-worse.csv")).score();
    let better = step(include_str!("data/loop-response/j1-better.csv")).score();
    // Hardware: Kiv .003 -> .0018 worsened the step; .0045 improved it.
    // These are step decisions only, not approval to deploy either gain.
    assert!(better < baseline && baseline < worse);
    let limit = 5.0 / 20.0;
    assert!(!response::accept_step(worse, baseline, limit));
    assert!(response::accept_step(better, baseline, limit));
    assert!(!response::accept_step(better, baseline, better / 2.0));
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1.0] {
        assert!(!response::accept_step(invalid, baseline, limit));
        assert!(!response::accept_step(better, invalid, limit));
        assert!(!response::accept_step(better, baseline, invalid));
    }
}

#[test]
fn periodic_fourier_analysis_keeps_adjacent_lines_and_reports_repeat_scatter() {
    use par6_bus::spectral::periodic::{Spec, PERIODS, SAMPLES};
    use std::f64::consts::TAU;
    let spec = Spec {
        profile: 0,
        token: 1,
        peak_ma: 90,
    };
    // Exact Fourier algebra, not a motor/robot model: adjacent orthogonal
    // lines have equal gain and different phase. Pooling them corrupts both.
    let mut e = Excited {
        injection: spec.samples(),
        setpoint: Vec::new(),
        speed: Vec::new(),
    };
    for i in 0..SAMPLES * PERIODS {
        let t = TAU * i as f64 / SAMPLES as f64;
        e.setpoint
            .push(100.0 + 20.0 * (10.0 * t).cos() + 20.0 * (11.0 * t).cos());
        e.speed
            .push(5000.0 + 120.0 * (10.0 * t).cos() + 120.0 * (11.0 * t).sin());
    }
    // A changing gravity/current background is fitted from period means,
    // without fitting away the excited Fourier components.
    for i in 0..e.speed.len() {
        e.setpoint[i] += 0.2 * i as f64;
        e.speed[i] += 0.1 * i as f64;
    }
    let measured = response::measure_periodic(&e, spec).unwrap();
    let line = |k| {
        measured
            .hz
            .iter()
            .position(|f| (*f - k as f64 * 6250.0 / (SAMPLES as f64 * 16.0)).abs() < 1e-9)
            .unwrap()
    };
    let a = line(10);
    let b = line(11);
    assert!((measured.plant[a].re - 6.0).abs() < 1e-9);
    assert!(measured.plant[a].im.abs() < 1e-9);
    assert!(measured.plant[b].re.abs() < 1e-9);
    assert!((measured.plant[b].im + 6.0).abs() < 1e-9);
    assert!(measured.coherence[a] > 0.9999 && measured.coherence[b] > 0.9999);
    assert!(measured.uncertainty[a] > 0.0); // Quantization floor even with no scatter.
                                            // Agreement with itself cannot turn a two-line record into full coverage.
    assert!(response::focused_agreement(&measured, &measured).is_none());
    let mut disturbed = e.clone();
    for (i, value) in disturbed.speed.iter_mut().enumerate() {
        let sign = if (i / SAMPLES).is_multiple_of(2) {
            1.0
        } else {
            -1.0
        };
        *value += sign * 160.0 * (TAU * 10.0 * i as f64 / SAMPLES as f64).cos();
    }
    let noisy = response::measure_periodic(&disturbed, spec).unwrap();
    assert!(noisy.coherence[a] < response::COHERENCE_MIN);
    assert!(noisy.uncertainty[a] > 10.0 * measured.uncertainty[a]);
    assert!((noisy.plant[b].im - measured.plant[b].im).abs() < 1e-9);
    // Recorded PRBS hardware data cannot be relabelled as periodic evidence.
    assert!(response::measure_periodic(&excited(SLOW), spec).is_none());
    let mut wrong_phase = e.clone();
    wrong_phase.injection.rotate_left(1);
    assert!(response::measure_periodic(&wrong_phase, spec).is_none());
    e.speed.pop();
    assert!(response::measure_periodic(&e, spec).is_none());

    // Exact signal scaling at two excitation levels: no dynamic plant model.
    // Recover the same ratio, and distinguish an amplitude-dependent ratio.
    let scaled = |peak_ma, ratio| {
        let spec = Spec { peak_ma, ..spec };
        let injection = spec.samples();
        let e = Excited {
            setpoint: injection.iter().map(|v| 100.0 + v).collect(),
            speed: injection.iter().map(|v| 4000.0 + ratio * v).collect(),
            injection,
        };
        response::measure_periodic(&e, spec).unwrap()
    };
    let first = scaled(40, 6.0);
    let second = scaled(60, 6.0);
    let different = scaled(60, 9.0);
    let (usable, total) = response::focused_quality(&first);
    assert!(total > 0 && usable == total);
    assert!(response::focused_agreement(&first, &second).unwrap() < 1e-8);
    assert!(response::focused_agreement(&first, &different).unwrap() > 1.0);
    let mut uncertain = different;
    uncertain.uncertainty.fill(10000.0);
    assert!(response::focused_agreement(&first, &uncertain).is_none());
}
