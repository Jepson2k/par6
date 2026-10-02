//! Wire and firmware-sequencer compatibility; no bus or robot simulation.
use par6_bus::spectral::{
    codec::*,
    periodic::{self, Spec},
};

#[test]
fn firmware_waveforms_and_completion_identity_match_the_host() {
    let csv = include_str!("data/periodic-current-v2.csv");
    for line in csv.lines().skip(1) {
        let v: Vec<i32> = line.split(',').map(|v| v.parse().unwrap()).collect();
        let spec = Spec {
            profile: v[0] as u8,
            token: 0x1234,
            peak_ma: 90,
        };
        let tick = (v[1] as usize + 1) * usize::from(spec.divisor()) - 1;
        assert_eq!(i32::from(spec.current(tick)), v[2], "{line}");
    }
    let spec = Spec {
        profile: 0,
        token: 0x1234,
        peak_ma: 90,
    };
    let request = periodic::encode(3, spec);
    assert_eq!(request.id, 0x1d6);
    assert_eq!(request.payload(), [2, 0, 16, 0, 0x12, 0x34, 0, 90]);
    let frame = CanFrame::data_frame(0x1d6, &[2, 0, 16, 4, 0x12, 0x34, 0, 90]);
    let Payload::PeriodicStatus(s) = decode_frame(&frame).unwrap().payload else {
        panic!("wrong reply");
    };
    assert!(s.completed(spec));
    // A partial, clipped, stale, or differently configured record cannot
    // authorize analysis even when its three sample channels have full length.
    for (i, value) in [
        (0, 1),
        (1, 1),
        (2, 4),
        (3, 1),
        (3, 2),
        (3, 8),
        (3, 20),
        (4, 0),
        (7, 91),
    ] {
        let mut bad = frame;
        bad.data[i] = value;
        let Payload::PeriodicStatus(s) = decode_frame(&bad).unwrap().payload else {
            panic!();
        };
        assert!(!s.completed(spec));
    }
    for dlc in 0..8 {
        let mut short = frame;
        short.dlc = dlc;
        assert!(decode_frame(&short).is_err());
    }
    assert!(decode_frame(&periodic::request(3)).is_err());
    for spec in [
        Spec { profile: 2, ..spec },
        Spec { token: 0, ..spec },
        Spec {
            peak_ma: 1001,
            ..spec
        },
    ] {
        assert!(!spec.valid());
    }

    // At least 85% of low-profile line power must reach the unresolved
    // 10–40 Hz band, with nonzero anchors retained on both sides.
    let samples = spec.samples();
    let mut focused = 0.0;
    let mut total = 0.0;
    for k in spec.bins() {
        let (re, im) =
            samples[..periodic::SAMPLES]
                .iter()
                .enumerate()
                .fold((0.0, 0.0), |(re, im), (n, x)| {
                    let phase = std::f64::consts::TAU * (k * n) as f64 / periodic::SAMPLES as f64;
                    (re + x * phase.cos(), im - x * phase.sin())
                });
        let power = re * re + im * im;
        assert!(power > 1.0, "missing anchor at bin {k}");
        total += power;
        let hz = k as f64 * 6250.0 / (periodic::SAMPLES as f64 * f64::from(spec.divisor()));
        if (10.0..=40.0).contains(&hz) {
            focused += power;
        }
    }
    assert!(
        focused / total >= 0.85,
        "focused fraction {}",
        focused / total
    );
}
