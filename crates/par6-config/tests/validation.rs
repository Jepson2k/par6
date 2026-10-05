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
fn powered_support_requires_one_finite_nonnegative_value_per_joint() {
    for values in [
        "[]",
        "[1.0]",
        "[1.0, 8.0, 3.0, 0.5, 0.5, 0.3, 0.1]",
        "[nan, 8.0, 3.0, 0.5, 0.5, 0.3]",
        "[inf, 8.0, 3.0, 0.5, 0.5, 0.3]",
        "[-1.0, 8.0, 3.0, 0.5, 0.5, 0.3]",
    ] {
        let field = refused_field(
            load_with(
                "powered_support_nm = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
                &format!("powered_support_nm = {values}"),
            ),
            values,
        );
        assert_eq!(field, "sim.powered_support_nm");
    }
    load_with(
        "powered_support_nm = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
        "powered_support_nm = [1.0, 8.0, 3.0, 0.5, 0.5, 0.3]",
    )
    .expect("a positive support is a valid model assumption");
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

/// Homing must not depend on where the arm was last parked. Every joint
/// gets a seek budget long enough to cross its whole mechanical range at
/// the seek speed, so a joint left at the far end still reaches the
/// endstop. The shipped `timeout_s` values do not: on 2026-09-20 J1 swept
/// 196 deg of its 338 deg range inside the configured 13 s and stopped
/// short of the switch, and J2, J3 and J6 carry the same shortfall.
#[test]
fn every_joint_can_seek_across_its_whole_range() {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
    let robot = RobotConfig::load(&path).expect("shipped PAR6.toml");
    for (i, (joint, homing)) in robot.joints.iter().zip(&robot.homing.joints).enumerate() {
        let ticks_per_rad =
            f64::from(1u32 << joint.encoder_bits) / std::f64::consts::TAU * joint.gear_ratio;
        let span_ticks = (joint.limits.hard_max_rad - joint.limits.hard_min_rad) * ticks_per_rad;
        let crossing_s = span_ticks / homing.speed_ticks_s;
        let budget_s = homing.seek_timeout_s(joint);
        // A bare crossing leaves nothing for the ramp up to seek speed or
        // the stall confirmation at the stop: a fifth more, at least.
        assert!(
            budget_s >= 1.2 * crossing_s,
            "J{}: seek budget {budget_s:.1} s covers only {:.0}% of the {crossing_s:.1} s \
             needed to cross its range at {} ticks/s",
            i + 1,
            budget_s / crossing_s * 100.0,
            homing.speed_ticks_s,
        );
    }
}

/// The two-pass check compares where the second pass stalled with the
/// first, and the second pass starts a backoff away. A tolerance that
/// reaches the backoff passes a second pass that stalled where it began
/// — J0 shipped the vendor's 3500 ticks against a 1350-tick backoff.
#[test]
fn a_two_pass_tolerance_must_be_below_the_backoff_travel() {
    let shipped = "two_pass_max_diff_ticks = 500";
    // J0: 4500 ticks/s for 0.3 s.
    let field = refused_field(
        load_with(shipped, "two_pass_max_diff_ticks = 1350"),
        "a tolerance equal to the backoff",
    );
    assert_eq!(field, "homing.joints[0].two_pass_max_diff_ticks");
    load_with(shipped, "two_pass_max_diff_ticks = 1349").expect("a tolerance inside the backoff");
}

/// A joint's name is how an overlay addresses it, so two joints may not
/// share one.
#[test]
fn joint_names_are_unique() {
    let field = refused_field(
        load_with("name = \"joint2\"", "name = \"joint1\""),
        "two joints named joint1",
    );
    assert_eq!(field, "joints[1].name");
}

/// One installation's values layer over the shipped file: a local overlay
/// sets one joint's gain and stands the arm on its bench, everything it
/// does not name stays shipped, and a key the schema does not know is
/// refused with the overlay named, since the mistake is there.
#[test]
fn a_local_overlay_layers_one_installations_values_over_the_shipped_file() {
    use par6_config::ConfigBundle;
    let shipped = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
    let dir = std::env::temp_dir().join(format!("par6-config-overlay-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("scratch dir");
    let local = dir.join("local.toml");
    std::fs::write(
        &local,
        "[[joints]]\nname = \"joint1\"\n\
         [[joints]]\nname = \"joint2\"\n[joints.gains]\nkiv = 0.00123\n\
         [[installation_shapes]]\nname = \"bench\"\nkind = \"box\"\n\
         params = [1.0, 1.0, 0.1]\npose = [0.0, 0.0, -0.06, 0.0, 0.0, 0.0]\n",
    )
    .expect("overlay");
    let plain = ConfigBundle::load(&shipped).expect("shipped");
    let layered = ConfigBundle::load_with(&shipped, Some(&local), None).expect("layered");
    assert_eq!(layered.robot.joints[1].gains.kiv, 0.00123);
    assert_eq!(
        layered.robot.joints[1].gains.kpv,
        plain.robot.joints[1].gains.kpv
    );
    assert_eq!(layered.robot.joints[0], plain.robot.joints[0]);
    assert_eq!(layered.robot.sim, plain.robot.sim);
    assert!(
        layered
            .installation_shapes
            .iter()
            .any(|s| s.name == "bench"),
        "the overlay's bench shape: {:?}",
        layered.installation_shapes
    );

    std::fs::write(&local, "[sim]\nviscous = [0.0]\n").expect("typo");
    let err = ConfigBundle::load_with(&shipped, Some(&local), None)
        .expect_err("an unknown key must be refused");
    assert!(
        err.to_string().contains("local.toml"),
        "the refusal must name the overlay: {err}"
    );
    let _ = std::fs::remove_dir_all(&dir);
}
