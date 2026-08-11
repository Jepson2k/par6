//! Shared helpers for par6-motion integration tests.
// Each test binary compiles this module and uses a subset of it.
#![allow(dead_code)]

use par6_config::RobotConfig;
use par6_motion::{Sample, NUM_JOINTS};

/// Load the real PAR6 robot config from the repo `config/` directory.
pub fn par6_config() -> RobotConfig {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../config/PAR6.toml");
    RobotConfig::load(std::path::Path::new(path)).expect("load config/PAR6.toml")
}

/// Position sequence of a plan including the start pose, for finite
/// differencing at tick resolution.
pub fn positions_with_start(
    start: [f64; NUM_JOINTS],
    samples: &[Sample],
) -> Vec<[f64; NUM_JOINTS]> {
    let mut qs = Vec::with_capacity(samples.len() + 1);
    qs.push(start);
    qs.extend(samples.iter().map(|s| s.q));
    qs
}

/// Assert that finite differences of `qs` at `dt` never exceed the given
/// per-joint velocity/acceleration (and optionally jerk) limits.
pub fn assert_within_limits(
    qs: &[[f64; NUM_JOINTS]],
    dt: f64,
    vel: &[f64; NUM_JOINTS],
    acc: &[f64; NUM_JOINTS],
    jerk: Option<&[f64; NUM_JOINTS]>,
    ctx: &str,
) {
    let tol = 1.0 + 1e-6;
    let abs = 1e-9;
    let v: Vec<[f64; NUM_JOINTS]> = qs
        .windows(2)
        .map(|w| std::array::from_fn(|j| (w[1][j] - w[0][j]) / dt))
        .collect();
    for (k, vk) in v.iter().enumerate() {
        for j in 0..NUM_JOINTS {
            assert!(
                vk[j].abs() <= vel[j] * tol + abs,
                "{ctx}: joint {j} velocity {} exceeds limit {} at tick {k}",
                vk[j],
                vel[j]
            );
        }
    }
    let a: Vec<[f64; NUM_JOINTS]> = v
        .windows(2)
        .map(|w| std::array::from_fn(|j| (w[1][j] - w[0][j]) / dt))
        .collect();
    for (k, ak) in a.iter().enumerate() {
        for j in 0..NUM_JOINTS {
            assert!(
                ak[j].abs() <= acc[j] * tol + abs,
                "{ctx}: joint {j} acceleration {} exceeds limit {} at tick {k}",
                ak[j],
                acc[j]
            );
        }
    }
    if let Some(jerk) = jerk {
        for (k, w) in a.windows(2).enumerate() {
            for j in 0..NUM_JOINTS {
                let jj = (w[1][j] - w[0][j]) / dt;
                assert!(
                    jj.abs() <= jerk[j] * tol + 1e-6,
                    "{ctx}: joint {j} jerk {} exceeds limit {} at tick {k}",
                    jj,
                    jerk[j]
                );
            }
        }
    }
}

/// Largest |q[j] - target[j]| over all joints.
pub fn max_err(q: &[f64; NUM_JOINTS], target: &[f64; NUM_JOINTS]) -> f64 {
    q.iter()
        .zip(target.iter())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0, f64::max)
}
