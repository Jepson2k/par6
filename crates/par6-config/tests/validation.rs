//! Config validation rejects what the runtime cannot survive: a NaN that
//! slips past a `v <= 0.0` check reaches `f64::clamp` in the torque slew
//! and aborts the RT thread, a NaN cutoff poisons the stream filter, and
//! an unbounded retry window wraps the daemon's attempt count to zero.

use std::path::PathBuf;

use par6_config::{ConfigError, RobotConfig};

fn shipped() -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
    std::fs::read_to_string(&path).expect("shipped PAR6.toml")
}

/// Load the shipped config with one line rewritten.
fn load_with(from: &str, to: &str) -> Result<RobotConfig, ConfigError> {
    let text = shipped();
    assert!(text.contains(from), "the shipped config carries `{from}`");
    let text = text.replacen(from, to, 1);
    let dir = std::env::temp_dir().join(format!(
        "par6-config-validation-{}-{}",
        std::process::id(),
        from.split_whitespace().next().unwrap_or("x")
    ));
    std::fs::create_dir_all(&dir).expect("scratch dir");
    let path = dir.join("PAR6.toml");
    std::fs::write(&path, text).expect("write patched config");
    let result = RobotConfig::load(&path);
    let _ = std::fs::remove_dir_all(&dir);
    result
}

fn refused_field(result: Result<RobotConfig, ConfigError>, what: &str) -> String {
    match result {
        Err(ConfigError::Invalid { field, .. }) => field,
        Err(other) => panic!("{what}: refused for the wrong reason: {other}"),
        Ok(_) => panic!("{what}: loaded"),
    }
}

#[test]
fn nan_and_unbounded_values_are_refused_by_name() {
    let field = refused_field(
        load_with("torque_rate_nm_s = 364.0", "torque_rate_nm_s = nan"),
        "NaN torque rate",
    );
    assert!(field.ends_with("limits.torque_rate_nm_s"), "{field}");

    let field = refused_field(
        load_with("lowpass_cutoff_hz = 0.0", "lowpass_cutoff_hz = nan"),
        "NaN low-pass cutoff",
    );
    assert_eq!(field, "stream.lowpass_cutoff_hz");

    let field = refused_field(
        load_with("open_retry_s = 10.0", "open_retry_s = 1e12"),
        "unbounded open retry",
    );
    assert_eq!(field, "bus.open_retry_s");

    let field = refused_field(
        load_with("servo_grace_s = 0.25", "servo_grace_s = 0.0"),
        "zero servo grace",
    );
    assert_eq!(field, "stream.servo_grace_s");

    assert!(
        load_with("torque_rate_nm_s = 364.0", "torque_rate_nm_s = 364.0").is_ok(),
        "the shipped config loads"
    );
}
