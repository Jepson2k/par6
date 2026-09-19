//! Shared by par6-kin's integration tests.
#![allow(dead_code)]

use std::path::PathBuf;

/// The repo checkout this crate lives in.
pub fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .unwrap()
}

/// The vendor description tree: URDFs, SRDFs and meshes.
pub fn assets_dir() -> PathBuf {
    repo_root().join("assets/par6_description")
}

/// A deterministic xorshift64 sequence of unit values in `[0, 1)` — a
/// reproducible spread of samples, not statistical realism.
pub fn xorshift(seed: u64) -> impl FnMut() -> f64 {
    let mut x = seed;
    move || {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        (x >> 11) as f64 / (1u64 << 53) as f64
    }
}
