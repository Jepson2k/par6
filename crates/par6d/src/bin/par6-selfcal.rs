// CALIBRATION RULE: No assistant-invented heuristics. Cite a primary source beside
// every metric, tuning method, and numerical decision rule. State what it supports.
// Vendor constants and explicit user requirements must identify their provenance.
// A citation for a formula does not justify an arbitrary threshold or certify hardware.
// User requirement: no blind waits. Advance on feedback or completion of a measured experiment.
include!("selfcal/mod.rs");
use std::fmt::Write as _;
use std::{
    fs,
    io::{BufRead, Write},
    path::PathBuf,
};

// https://docs.rs/clap/latest/clap/_derive/_tutorial/index.html
#[derive(clap::Parser)]
#[command(
    name = "par6-selfcal",
    about = "Measure and tune PAR6 from vendor references"
)]
struct Args {
    #[arg(default_value = "config/PAR6.toml")]
    config: PathBuf,
    #[arg(long, default_value = "calibration-runs")]
    output_dir: PathBuf,
    /// Maximum gain experiments per search stage.
    #[arg(long, value_parser = clap::value_parser!(u32).range(3..),
        required_unless_present_any = ["baseline", "check_runtime", "check_gravity", "probe_endstop", "replay"])]
    trials: Option<u32>,
    /// Preview the fixed gravity poses and predicted loads without motor commands.
    #[arg(long, conflicts_with_all = ["apply", "home_only", "joint", "baseline", "check_runtime"])]
    check_gravity: bool,
    /// Home all joints, permitting gain changes only on this joint; skip gravity.
    #[arg(long, value_parser = clap::value_parser!(u8).range(1..=6), conflicts_with = "apply")]
    joint: Option<u8>,
    /// Measure vendor gains without searching.
    #[arg(long, requires = "joint", conflicts_with = "apply")]
    baseline: bool,
    /// Seek only this endstop joint; stop on detected stall or the requested duration.
    #[arg(long, value_parser = clap::value_parser!(u8).range(1..=5),
        requires = "probe_seconds",
        conflicts_with_all = ["apply", "joint", "baseline", "home_only", "check_runtime", "check_gravity", "trials"])]
    probe_endstop: Option<u8>,
    /// Maximum duration of the single endstop probe, in seconds.
    #[arg(long, value_parser = clap::value_parser!(u32).range(1..), requires = "probe_endstop")]
    probe_seconds: Option<u32>,
    /// Replay a recorded run's samples.csv without opening CAN or moving motors.
    #[arg(long, conflicts_with_all = ["apply", "joint", "baseline", "home_only", "check_runtime", "check_gravity", "probe_endstop", "trials", "sim"])]
    replay: Option<PathBuf>,
    // User requirement, 2026-09-18: movement 5 deg/s, holding 1 deg/s RMS.
    // These are application acceptance limits, not values derived from a paper.
    /// Maximum RMS speed error during movement, in joint degrees/second.
    #[arg(long, default_value = "5", value_parser = parse_rms_limit)]
    moving_rms_deg_s: f64,
    // Measured on this arm with quiet holds, i.e. the drive's own speed-report
    // ripple (STEPFOC V108: one 6250 Hz encoder delta averaged over 20 samples)
    // rather than loop error: J4 8.1% of a 63 deg/s peak command (run
    // 1789703194339052897), J5 10.8-11.2% of 74 deg/s at every Kpv/Kiv within
    // bounds (run 1789776117327509284). User decision 2026-09-18: the moving
    // limit scales with the command so that floor is not the requirement on
    // the fast wrist joints; 0.15 clears the measured J5 floor with margin
    // while an oscillating J3 measured 21% and above. The absolute value
    // stays as a floor for slow legs.
    /// Moving limit as a fraction of the leg's peak commanded speed; the larger
    /// of this and --moving-rms-deg-s applies.
    #[arg(long, default_value = "0.15", value_parser = parse_fraction)]
    moving_rms_fraction: f64,
    /// Start from the vendor velocity gains and gravity_scale = 1 instead of
    /// the configuration's current values.
    #[arg(long)]
    from_vendor: bool,
    // STEPFOC's tuning guide raises a gain until oscillation and backs off 20%,
    // i.e. one 1.25x step: https://source-robotics.github.io/STEPFOC-docs/PID_tuning/
    /// Ratio between successive gain scales while walking toward a boundary.
    #[arg(long, default_value = "1.25", value_parser = parse_ratio)]
    gain_step: f64,
    // Bound carried over from the previous par6-selfcal (gain_ceiling = 2.5),
    // applied to the run's starting gains in both directions.
    /// Largest factor by which any gain may differ from its starting value.
    #[arg(long, default_value = "2.5", value_parser = parse_ratio)]
    gain_ceiling: f64,
    // Search resolution, not a physical threshold: bisection of each boundary
    // stops when the passing and failing scales are within this ratio.
    /// Ratio within which a band boundary is considered located.
    #[arg(long, default_value = "1.1", value_parser = parse_ratio)]
    gain_resolution: f64,
    // The previous par6-selfcal allowed at most four homing attempts per joint.
    /// Additional slow approaches allowed when the two homing contacts disagree.
    #[arg(long, default_value = "3", value_parser = clap::value_parser!(u32).range(1..))]
    home_retries: u32,
    /// Maximum RMS speed while holding, in joint degrees/second.
    #[arg(long, default_value = "1", value_parser = parse_rms_limit)]
    holding_rms_deg_s: f64,
    // Kollmorgen's simple velocity tuning procedure recommends 1 s for an
    // initial service interval when expected settling time is unknown:
    // https://www.kollmorgen.com/en-us/developer-network/akd-online-tuning-guide
    // This is a finite measurement interval, not proof of indefinite stability.
    /// Holding observation interval, in seconds; independent of the settling timeout.
    #[arg(long, default_value = "1", value_parser = parse_observation)]
    hold_observation_s: f64,
    // `[bus].stale_warn_s` is a self-clearing warning (its own config comment
    // says so) and doubles as this binary's control-deadline margin, so it is
    // the wrong knob for "this drive has stopped answering". Measured over
    // four hardware runs (467k samples each): every joint is answered within
    // 1 tick almost always and 3 ticks at worst, against a 10-tick abort. On
    // 2026-09-19 J1 alone went unanswered for 10 consecutive ticks while the
    // other five answered every tick; that abort, through a shutdown that
    // could then not park, dropped the arm. The cause of that dropout is not
    // established, so this default buys time for one to clear rather than
    // claiming a specific fault; it stays far under the drives' 5 s watchdog.
    /// How long a drive may go without fresh motion feedback before the run
    /// treats it as lost, in seconds.
    #[arg(long, default_value = "0.2", value_parser = parse_observation)]
    feedback_timeout_s: f64,
    #[arg(long)]
    home_only: bool,
    /// Check real-time setup without sending motor commands.
    #[arg(long, conflicts_with = "apply")]
    check_runtime: bool,
    #[arg(long)]
    sim: bool,
    /// Apply completed results to the input configuration.
    #[arg(long)]
    apply: bool,
}
fn parse_rms_limit(value: &str) -> std::result::Result<f64, String> {
    let degrees = value.parse::<f64>().map_err(|e| e.to_string())?;
    if !degrees.is_finite() || degrees.to_radians() <= 0.0 {
        return Err("RMS speed limit must be finite and positive".into());
    }
    Ok(degrees.to_radians())
}
fn parse_ratio(value: &str) -> std::result::Result<f64, String> {
    let ratio = value.parse::<f64>().map_err(|e| e.to_string())?;
    if !ratio.is_finite() || ratio <= 1.0 {
        return Err("ratio must be finite and greater than one".into());
    }
    Ok(ratio)
}
fn parse_fraction(value: &str) -> std::result::Result<f64, String> {
    let fraction = value.parse::<f64>().map_err(|e| e.to_string())?;
    if !fraction.is_finite() || fraction < 0.0 {
        return Err("fraction must be finite and not negative".into());
    }
    Ok(fraction)
}
fn parse_observation(value: &str) -> std::result::Result<f64, String> {
    let seconds = value.parse::<f64>().map_err(|e| e.to_string())?;
    if !seconds.is_finite() || seconds <= 0.0 {
        return Err("holding observation must be finite and positive".into());
    }
    Ok(seconds)
}
fn replay_quality(directory: &Path, bundle: &ConfigBundle, limits: QualityLimits) -> Result<()> {
    // Replay actual recorded hardware samples through the live hold accumulator.
    // The CSV has eight scalar fields followed by a quoted Debug command; splitn
    // preserves that command verbatim, including its embedded commas.
    let log = fs::read_to_string(directory.join("console.log"))?;
    // Align measured holds with their recorded tick boundaries, including the
    // final observation immediately before shutdown. Wall-clock jitter must not
    // cause that last window to disappear from replay.
    let hold_ticks = (limits.hold_observation_s / bundle.robot.robot.tick_dt_s)
        .round()
        .max(1.0) as u64;
    let hold_ends: std::collections::BTreeSet<u64> = log
        .lines()
        .filter(|line| line.contains(" HOLD_QUALITY "))
        .filter_map(|line| {
            line.split_whitespace()
                .next()?
                .strip_prefix("tick=")?
                .parse()
                .ok()
        })
        .collect();
    let hold_starts: std::collections::BTreeSet<u64> = hold_ends
        .iter()
        .map(|end| end.saturating_sub(hold_ticks))
        .collect();
    let csv = std::io::BufReader::new(fs::File::open(directory.join("samples.csv"))?);
    let per_tick: [f64; N] = std::array::from_fn(|j| {
        let conversion = JointConversion::from_config(&bundle.robot.joints[j]);
        conversion.joint_rad(1) - conversion.joint_rad(0)
    });
    let window_ns = (limits.hold_observation_s * 1e9) as u64;
    let mut target = [None; N];
    let mut generation = [0; N];
    let mut sample_ns = [0; N];
    let mut start_ns = [0; N];
    let mut start_tick = [0; N];
    let mut quality = [HoldQuality::default(); N];
    let mut estimators = std::array::from_fn::<_, N, _>(|_| HoldVelocity::default());
    for estimator in &mut estimators {
        *estimator = HoldVelocity::new(limits.hold_observation_s, bundle.robot.robot.tick_dt_s)?;
    }
    let mut passes = [0; N];
    let mut rejected = [0; N];
    let mut worst_speed = [0.0_f64; N];
    let mut previous_pass = [None; N];
    fn commanded(command: &str, field: &str) -> Option<i32> {
        command.split_once(field)?.1.split_once(')')?.0.parse().ok()
    }
    for (index, line) in csv.lines().enumerate() {
        let line = line?;
        if index == 0 {
            continue;
        }
        // Runs recorded before the drive-fault column have nine fields; the
        // quoted command is always last.
        let fields: Vec<_> = line.splitn(10, ',').collect();
        if !(9..=10).contains(&fields.len()) {
            return Err(format!("invalid samples.csv row {}", index + 1).into());
        }
        let command = fields[fields.len() - 1];
        let tick = fields[0].parse::<u64>()?;
        let time_ns = fields[1].parse::<u64>()?;
        let j = fields[3]
            .parse::<usize>()?
            .checked_sub(1)
            .filter(|j| *j < N)
            .ok_or("invalid recorded joint")?;
        let fresh = fields[4].parse::<u64>()?;
        let position = fields[5].parse::<i32>()?;
        let speed = fields[6].parse::<i32>()?;
        let current = fields[7].parse::<i16>()?;
        if fresh != generation[j] {
            sample_ns[j] = time_ns;
        }
        let held = commanded(command, "pos: Some(")
            .filter(|_| commanded(command, "vel: Some(") == Some(0));
        if target[j] != held || held.is_none() || hold_starts.contains(&tick) {
            target[j] = held;
            quality[j] = HoldQuality::default();
            start_ns[j] = time_ns;
            start_tick[j] = tick;
            generation[j] = fresh;
            estimators[j].reset(sample_ns[j], position);
            continue;
        }
        if fresh == generation[j] {
            continue;
        }
        generation[j] = fresh;
        quality[j].push(
            (f64::from(position) - f64::from(held.ok_or("missing replay target")?)) * per_tick[j],
            estimators[j]
                .push(time_ns, position)?
                .map(|speed| speed * per_tick[j]),
            f64::from(speed) * per_tick[j],
            f64::from(current),
        );
        if !hold_ends.contains(&tick) && time_ns.saturating_sub(start_ns[j]) < window_ns {
            continue;
        }
        let [_, velocity, current_ac] = quality[j].rms();
        let pass = quality[j]
            .score(
                limits.holding_limit(per_tick[j], bundle.robot.robot.tick_dt_s),
                bundle.robot.motion.settle_tolerance_rad,
            )
            .accepted();
        worst_speed[j] = worst_speed[j].max(velocity);
        if pass {
            passes[j] += 1;
        } else {
            rejected[j] += 1;
        }
        // Report changes and summarize all windows, rather than flooding output.
        if previous_pass[j] != Some(pass) || hold_ends.contains(&tick) {
            println!("REPLAY J{} hold ticks={}-{} speed_estimator=end_fit_foaw speed_rms={:.6}deg/s raw_speed_rms={:.6}deg/s speed_limit={:.6}deg/s peak_position_error={:.6}deg current_ac_rms={:.1}mA pass={pass}",
                j + 1, start_tick[j], tick, velocity.to_degrees(), quality[j].raw_speed_rms().to_degrees(),
                limits.holding_limit(per_tick[j], bundle.robot.robot.tick_dt_s).to_degrees(),
                quality[j].peak_error_rad.to_degrees(), current_ac);
        }
        if previous_pass[j].is_none() {
            println!(
                "REPLAY J{} first_hold_action={}",
                j + 1,
                if pass { "accept" } else { "tune_before_homing" }
            );
        }
        previous_pass[j] = Some(pass);
        quality[j] = HoldQuality::default();
        estimators[j].reset(time_ns, position);
        start_ns[j] = time_ns;
        start_tick[j] = tick;
    }
    // Reassess completed legs' recorded RMS values with the live holding-stage
    // ranking. This verifies score handling, not the response to untried gains.
    // Contact and interrupted legs do not supply a completed free-motion trial.
    for line in log.lines().filter(|line| {
        (line.contains(" END Complete ") || line.contains(" END Tracking "))
            && line.contains("stop=Some(")
    }) {
        // The holding limit is per joint, so the replay needs the joint the
        // line belongs to: "tick=N J<n> END ...".
        let joint = line
            .split_once(" J")
            .and_then(|(_, rest)| rest.split_once(' '))
            .and_then(|(index, _)| index.parse::<usize>().ok())
            .filter(|n| (1..=N).contains(n))
            .ok_or("replay line names no joint")?
            - 1;
        let Some((_, speed)) = line.split_once(" velocity RMS=") else {
            continue;
        };
        let speed = speed
            .split_once("deg/s")
            .ok_or("invalid movement RMS")?
            .0
            .parse::<f64>()?;
        let holding = line
            .split_once(" hold velocity RMS=")
            .and_then(|(_, value)| value.split_once("deg/s"))
            .ok_or("invalid holding RMS")?
            .0
            .parse::<f64>()?;
        // Runs recorded before the limit was logged used the absolute value alone.
        let moving_limit = match line.split_once(" moving_limit=") {
            Some((_, value)) => value
                .split_once("deg/s")
                .ok_or("invalid moving limit")?
                .0
                .parse::<f64>()?
                .to_radians(),
            None => limits.moving_rad_s,
        };
        let score = GainStage::Velocity.score(
            Measurement {
                worst_velocity_rms_rad_s: speed.to_radians(),
                hold_velocity_rms_rad_s: holding.to_radians(),
                moving_limit_rad_s: moving_limit,
                holding_limit_rad_s: limits
                    .holding_limit(per_tick[joint], bundle.robot.robot.tick_dt_s),
                ..Measurement::default()
            },
            bundle.robot.motion.settle_tolerance_rad,
            true,
        );
        if !score.accepted() {
            let label = line.split(" elapsed=").next().unwrap_or(line);
            println!(
                "REPLAY {label} moving_speed_error_rms={speed:.6}deg/s limit={:.6}deg/s holding_speed_rms={holding:.6}deg/s velocity_leg_rank={score:?} pass=false",
                moving_limit.to_degrees()
            );
        }
    }
    for j in 0..N {
        println!("REPLAY J{} hold_windows_pass={} hold_windows_reject={} worst_hold_speed_rms={:.6}deg/s", j + 1, passes[j], rejected[j], worst_speed[j].to_degrees());
    }
    println!("Recorded-data replay completed; no motor commands sent.");
    Ok(())
}
/// Replace the value of `key = ...` on the first matching line of `block`,
/// keeping everything after the value (a trailing comment) intact.
fn patch_value(block: &str, key: &str, value: &str) -> Result<String> {
    let mut out = String::with_capacity(block.len() + 16);
    let mut done = false;
    for line in block.split_inclusive('\n') {
        let trimmed = line.trim_start();
        if !done && trimmed.starts_with(key) && trimmed[key.len()..].trim_start().starts_with('=') {
            let indent = &line[..line.len() - trimmed.len()];
            let after_eq = trimmed[key.len()..].trim_start()[1..].trim_start();
            let end = after_eq
                .find(|c: char| c.is_whitespace() || c == '#')
                .unwrap_or(after_eq.len());
            out.push_str(indent);
            out.push_str(key);
            out.push_str(" = ");
            out.push_str(value);
            out.push_str(&after_eq[end..]);
            done = true;
        } else {
            out.push_str(line);
        }
    }
    if done {
        Ok(out)
    } else {
        Err(format!("configuration has no `{key}` line to patch").into())
    }
}
/// Write measured gains and gravity factors into the configuration text.
/// Replace an existing top-level array in place, or add the whole line
/// ahead of the first one already there when the file does not carry it.
fn patch_array(
    text: &mut String,
    key: &str,
    values: &[f64],
    render: impl Fn(f64) -> String,
) -> Result<()> {
    let rendered = values
        .iter()
        .map(|v| render(*v))
        .collect::<Vec<_>>()
        .join(", ");
    let Some(start) = text.find(&format!("{key} =")) else {
        text.insert_str(0, &format!("{key} = [{rendered}]  # selfcal: measured\n"));
        return Ok(());
    };
    let open = start
        + text[start..]
            .find('[')
            .ok_or_else(|| format!("{key} is not an array"))?;
    let close = open
        + text[open..]
            .find(']')
            .ok_or_else(|| format!("{key} array is not closed"))?;
    text.replace_range(open..=close, &format!("[{rendered}]"));
    Ok(())
}
fn patch_config(
    original: &str,
    joints: &[par6_config::JointConfig],
    gravity_scale: Option<&[f64]>,
    gravity_correction: Option<&[f64]>,
) -> Result<String> {
    let mut text = original.to_owned();
    for (j, joint) in joints.iter().enumerate() {
        let name = text
            .find(&format!("name = \"{}\"", joint.name))
            .ok_or_else(|| format!("configuration has no joint named {}", joint.name))?;
        let gains = name
            + text[name..]
                .find("[joints.gains]")
                .ok_or_else(|| format!("{} has no [joints.gains] table", joint.name))?;
        let body = gains + "[joints.gains]".len();
        let end = body
            + text[body..]
                .find("\n[")
                .map_or(text.len() - body, |i| i + 1);
        let mut block = text[body..end].to_owned();
        for (key, value) in [
            ("kpv", joint.gains.kpv),
            ("kiv", joint.gains.kiv),
            ("kpp", joint.gains.kpp),
        ] {
            block = patch_value(&block, key, &format!("{value:?}"))
                .map_err(|e| format!("joint{}: {e}", j + 1))?;
        }
        text.replace_range(body..end, &block);
    }
    if let Some(scale) = gravity_scale {
        patch_array(&mut text, "gravity_scale", scale, |v| format!("{v:.4}"))?;
    }
    // The 24 identified link parameters. A file that never carried the key
    // gets the line: this is the first run that has one to write. They run
    // from grams-of-first-moment down to the solver's own noise floor, so
    // they are written at full precision rather than to a fixed decimal
    // place, which would quietly round the small ones to zero.
    if let Some(correction) = gravity_correction {
        patch_array(&mut text, "gravity_correction", correction, |v| {
            format!("{v:?}")
        })?;
    }
    Ok(text)
}
#[cfg(test)]
mod apply_tests {
    use super::*;
    #[test]
    fn measured_values_are_patched_in_place_and_everything_else_survives() {
        let original = fs::read_to_string(
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml"),
        )
        .unwrap();
        let mut robot = toml::from_str::<par6_config::RobotConfig>(&original).unwrap();
        for (j, joint) in robot.joints.iter_mut().enumerate() {
            joint.gains.kpv = 0.0101 + j as f64 * 0.001;
            joint.gains.kiv = 0.00051 + j as f64 * 0.0001;
            joint.gains.kpp = 4.5 + j as f64;
        }
        robot.gravity_scale = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5];
        // Spanning what a real fit produces: first moments in the tens of
        // milli-kg-m next to parameters the poses never fixed, down at the
        // solver's noise floor. Written to a fixed decimal place the small
        // ones round to zero and the file no longer says what was measured.
        robot.gravity_correction = (0..24)
            .map(|k| 6.010_996_101_271_37e-17 * f64::from(k + 1) - 0.002_5 * f64::from(k % 5))
            .collect();
        let patched = patch_config(
            &original,
            &robot.joints,
            Some(&robot.gravity_scale),
            Some(&robot.gravity_correction),
        )
        .unwrap();
        let reloaded = toml::from_str::<par6_config::RobotConfig>(&patched).unwrap();
        for (j, joint) in reloaded.joints.iter().enumerate() {
            assert_eq!(joint.gains, robot.joints[j].gains, "joint{}", j + 1);
        }
        assert_eq!(reloaded.gravity_scale, robot.gravity_scale);
        assert_eq!(reloaded.gravity_correction, robot.gravity_correction);
        // The shipped file carries no correction yet, so that one line is
        // added; everything else is patched where it stands, and comments,
        // spacing and every other key survive.
        assert_eq!(
            patched.lines().count(),
            original.lines().count() + 1,
            "the correction line is the only addition"
        );
        let kept: Vec<_> = patched
            .lines()
            .filter(|l| !l.starts_with("gravity_correction"))
            .collect();
        let changed = original.lines().zip(&kept).filter(|(a, b)| a != *b).count();
        assert_eq!(changed, 3 * 6 + 1, "exactly the measured lines change");
        assert!(patched.contains("# selfcal: measured per joint"));
        let gains_only = patch_config(&original, &robot.joints, None, None).unwrap();
        assert_eq!(
            toml::from_str::<par6_config::RobotConfig>(&gains_only)
                .unwrap()
                .gravity_correction,
            toml::from_str::<par6_config::RobotConfig>(&original)
                .unwrap()
                .gravity_correction,
            "a run without the identification stage leaves the correction alone"
        );
    }
}
fn main() -> std::process::ExitCode {
    use clap::Parser;
    match run(Args::parse()) {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("{e}");
            std::process::ExitCode::FAILURE
        }
    }
}
fn run(args: Args) -> Result<()> {
    let Args {
        config,
        output_dir: output,
        trials,
        check_gravity,
        joint,
        baseline: baseline_only,
        probe_endstop,
        probe_seconds,
        replay,
        moving_rms_deg_s: moving_rad_s,
        moving_rms_fraction: moving_fraction,
        from_vendor,
        gain_step,
        gain_ceiling,
        gain_resolution,
        home_retries,
        holding_rms_deg_s: holding_rad_s,
        hold_observation_s,
        feedback_timeout_s,
        home_only,
        check_runtime: check,
        sim,
        apply,
    } = args;
    let quality_limits = probe_endstop.is_none().then_some(QualityLimits {
        moving_rad_s,
        moving_fraction,
        holding_rad_s,
        hold_observation_s,
    });
    let focused_joint = joint.map(|j| usize::from(j - 1));
    let probe_joint = probe_endstop.map(|j| usize::from(j - 1));
    let home_only = home_only || focused_joint.is_some() || probe_joint.is_some();
    if !sim && !check_gravity && replay.is_none() {
        for entry in fs::read_dir("/proc")?.flatten() {
            let Ok(pid) = entry.file_name().to_string_lossy().parse::<u32>() else {
                continue;
            };
            if pid == std::process::id() {
                continue;
            }
            let comm = fs::read_to_string(entry.path().join("comm")).unwrap_or_default();
            if matches!(comm.trim(), "par6d" | "par6-selfcal") {
                return Err(format!(
                    "another controller is running: {}",
                    entry.file_name().to_string_lossy()
                )
                .into());
            }
        }
    }
    let config = config.canonicalize()?;
    let mut bundle = ConfigBundle::load(&config)?;
    if bundle.robot.joints.len() != N {
        return Err("calibration requires six joints".into());
    }
    // Vendor references, independent of saved calibration; constants only, not GPL code:
    // https://github.com/Source-Robotics/RCB-Runtime/blob/main/robots/PAR6.xml
    // User decision 2026-09-18: the configuration's values are the default
    // start; --from-vendor resets gains and gravity_scale for repeatability runs.
    let vendor_pi = [
        [0.015, 0.0015],
        [0.01, 0.001],
        [0.015, 0.0015],
        [0.015, 0.0005],
        [0.015, 0.0005],
        [0.009, 0.0006],
    ];
    // Homing currents come from the configuration ([homing.joints].current_ma);
    // backoff and holding use the operating current.
    if from_vendor {
        for (j, joint) in bundle.robot.joints.iter_mut().enumerate() {
            joint.gains.kpv = vendor_pi[j][0];
            joint.gains.kiv = vendor_pi[j][1];
        }
        bundle.robot.gravity_scale = [1.0; N];
    }
    // User-requested isolated endstop experiment: use the existing homing speed,
    // direction, current and stall detector, with an explicitly supplied duration.
    if let Some(j) = probe_joint {
        bundle.robot.homing.joints[j].timeout_s =
            f64::from(probe_seconds.ok_or("endstop probe needs a duration")?);
    }
    bundle.robot.validate()?;
    if let Some(recording) = replay {
        return replay_quality(
            &recording,
            &bundle,
            quality_limits.ok_or("replay needs quality limits")?,
        );
    }
    let assets = config
        .parent()
        .and_then(Path::parent)
        .ok_or("config needs a parent")?
        .join("assets/par6_description");
    let gravity_plan = if !home_only && !check {
        Some(identification_poses(
            &bundle,
            &assets,
            planned_ready(&bundle),
            bundle.robot.selfcal.identification_poses,
        )?)
    } else {
        None
    };
    let planned_ready = gravity_plan.as_ref().map(|poses: &Vec<[f64; N]>| {
        let ready = planned_ready(&bundle);
        println!(
            "identification plan: {} poses, each approached from below and above, \
             every leg clear of the collision world",
            poses.len()
        );
        ready
    });
    let mut pose_report = String::from("pose,q_rad\n");
    if let Some(poses) = &gravity_plan {
        for (i, q) in poses.iter().enumerate() {
            use std::fmt::Write;
            writeln!(pose_report, "{},\"{:?}\"", i + 1, q)?;
        }
        print!("{pose_report}");
    }
    if check_gravity {
        println!("Gravity pose preview completed; no motor commands sent.");
        return Ok(());
    }
    let timing = bundle.robot.timing.unwrap_or_default();
    let runtime = if sim {
        None
    } else {
        Some(runtime::Runtime::prepare(timing.cpu)?)
    };
    if check {
        if !sim {
            runtime::realtime(timing.cpu, timing.fifo_priority)?;
        }
        drop(runtime);
        println!("Runtime check passed; no motor commands sent.");
        return Ok(());
    }
    let id = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)?
        .as_nanos();
    let directory = output.join(format!(
        "{}-selfcal-{id}",
        if sim { "sim" } else { "hardware" }
    ));
    fs::create_dir_all(&directory)?;
    if gravity_plan.is_some() {
        fs::write(directory.join("identification-poses.csv"), &pose_report)?;
    }
    let original = fs::read(&config)?;
    fs::write(directory.join("config.before.toml"), &original)?;
    fs::write(
        directory.join("starting-config.toml"),
        toml::to_string_pretty(&bundle.robot)?,
    )?;
    println!("RUN_DIRECTORY: {}", directory.display());
    // The run directory must identify what produced it: the command line,
    // the executable and the configuration actually loaded.
    let executable = std::env::current_exe()?;
    let metadata = fs::metadata(&executable)?;
    let modified = metadata
        .modified()?
        .duration_since(std::time::UNIX_EPOCH)?
        .as_secs();
    fs::write(
        directory.join("invocation.txt"),
        format!(
            "argv: {}\nexecutable: {} ({} bytes, modified {modified} s since epoch)\nconfig: {}\ntrials per stage: {:?}\ngain_step: {gain_step}\ngain_ceiling: {gain_ceiling}\ngain_resolution: {gain_resolution}\nhome_retries: {home_retries}\njoint: {:?}\nfrom_vendor: {from_vendor}\nbaseline: {baseline_only}\nhome_only: {home_only}\nprobe_endstop: {:?}\nsim: {sim}\napply: {apply}\n",
            std::env::args().collect::<Vec<_>>().join(" "),
            executable.display(),
            metadata.len(),
            config.display(),
            trials,
            focused_joint.map(|j| j + 1),
            probe_joint.map(|j| j + 1),
        ),
    )?;
    if let Some(limits) = quality_limits {
        let specification = format!(
            "Explicit acceptance requirements: movement RMS speed error <= max({:.6} deg/s, {:.3} x the leg's peak commanded speed); holding RMS speed <= {:.6} deg/s; position-controlled moves: position error after the commanded motion <= {:.6} deg (tracking lag while moving is diagnostic); hold observation = {:.3} s; feedback timeout = {:.3} s. All values are joint-side; velocity-mode position lag is diagnostic only; every startup hold must pass before homing; limits are never increased automatically. Starting gains: {}.\n",
            limits.moving_rad_s.to_degrees(), limits.moving_fraction, limits.holding_rad_s.to_degrees(),
            bundle.robot.motion.settle_tolerance_rad.to_degrees(), limits.hold_observation_s,
            feedback_timeout_s,
            if from_vendor { "vendor" } else { "configuration" }
        );
        print!("{specification}");
        fs::write(directory.join("acceptance.txt"), specification)?;
    }
    let mut csv = std::io::BufWriter::new(fs::File::create(directory.join("samples.csv"))?);
    let mut log = fs::File::create(directory.join("console.log"))?;
    // Each joint resolves a different speed per encoder count, so the holding
    // limit that applied to a measurement is per joint; resolve it here, where
    // the config is still in scope, for the writer thread to report.
    let holding_limits: [f64; N] = std::array::from_fn(|j| {
        let conversion = JointConversion::from_config(&bundle.robot.joints[j]);
        quality_limits.map_or(f64::INFINITY, |l| {
            l.holding_limit(
                conversion.joint_rad(1) - conversion.joint_rad(0),
                bundle.robot.robot.tick_dt_s,
            )
        })
    });
    let (tx, rx) = std::sync::mpsc::sync_channel::<Event>(16384);
    let accepted_path = directory.join("accepted-gains.toml");

    let mut status_csv =
        std::io::BufWriter::new(fs::File::create(directory.join("node-status.csv"))?);
    // Filesystem and console latency must never delay the 250 Hz command thread.
    let writer = std::thread::spawn(move || -> std::io::Result<()> {
        let mut accepted = std::collections::BTreeMap::<String, par6_config::Gains>::new();

        writeln!(csv, "tick,rx_drain_ns,tx_begin_ns,joint,generation,position_ticks,speed_ticks_s,current_ma,drive_fault,command")?;
        writeln!(
            status_csv,
            "tick,joint,voltage_mv,temperature_c,live_error_bit,error,temperature,encoder,vbus,driver,velocity,current,estop,calibrated,watchdog"
        )?;
        let legend = "Diagnostics: ticks match samples.csv; positions are motor encoder ticks. Detector windows use the existing vendor rules: stopped when encoder range < max(10, 25% of requested ticks); loaded when >=60% of fresh samples meet the current threshold. decision=None means continue. Encoder-derived travel does not confirm output-joint movement.";
        println!("{legend}");
        writeln!(log, "{legend}")?;
        for event in rx {
            let line = match event {
                Event::Sample(tick, received, sent, commands, pos, vel, cur, generation, fault) => {
                    for j in 0..N {
                        writeln!(
                            csv,
                            "{tick},{received},{sent},{},{},{},{},{},{},\"{:?}\"",
                            j + 1,
                            generation[j],
                            pos[j],
                            vel[j],
                            cur[j],
                            u8::from(fault[j]),
                            commands[j]
                        )?;
                    }
                    continue;
                }
                Event::Phase(name, j) => format!("J{}: {name}", j + 1),
                Event::MotionStart(tick, j, motion, start, limit, guard, approach) => format!(
                    "tick={tick} J{} START {motion:?} encoder={start} limit={limit:.0}mA current_policy={} detector_start_after={guard:.3}s",
                    j + 1, if approach { "homing" } else { "operating" }
                ),
                Event::Motion(tick, j, start, end, radians_per_tick, m) => {
                    let [position, velocity, current] = m.rms();
                    let delta = i64::from(end) - i64::from(start);
                    format!("tick={tick} J{} END {:?} elapsed={:.3}s encoder={start}->{end} delta={delta}ticks motor-derived-travel={:.5}deg position RMS={:.5}deg peak error={:.5}deg settled error={:.5}deg velocity RMS={:.5}deg/s moving_limit={:.5}deg/s peak_command={:.5}deg/s hold velocity RMS={:.5}deg/s current RMS={:.0}mA peak={:.0}mA samples={} stop={:?}", j + 1, m.outcome, m.elapsed_s, (delta as f64 * radians_per_tick).to_degrees(), position.to_degrees(), m.peak_error_rad.to_degrees(), m.settled_error_rad.to_degrees(), velocity.to_degrees(), m.moving_limit_rad_s.to_degrees(), m.peak_command_rad_s.to_degrees(), m.hold_velocity_rms_rad_s.to_degrees(), current, m.peak_current_ma, m.samples, m.stop)
                }
                Event::HoldQuality(tick, j, purpose, quality) => {
                    let [position, velocity, current] = quality.rms();
                    let limit = holding_limits[j];
                    format!("tick={tick} J{} HOLD_QUALITY purpose={purpose:?} samples={} position_rms={:.6}deg peak_error={:.6}deg speed_estimator=end_fit_foaw residual_bound={FOAW_RESIDUAL_COUNTS}ticks speed_rms={:.6}deg/s raw_speed_rms={:.6}deg/s speed_limit={:.6}deg/s current_ac_rms={current:.1}mA speed_pass={}",
                        j + 1, quality.samples, position.to_degrees(), quality.peak_error_rad.to_degrees(), velocity.to_degrees(), quality.raw_speed_rms().to_degrees(), limit.to_degrees(), velocity <= limit)
                }
                Event::Detection(j, d) => format!(
                    "tick={} J{} DETECT elapsed={:.3}s window={:.3}s encoder={} range={}ticks requested={:.2}ticks stall_below={:.2}ticks stopped={} current={:.0}mA threshold={:.0}mA limit={:.0}mA high_current={}/{} loaded={} commanded_speed={}ticks/s encoder_speed={}ticks/s advancing={} tracking_error={:.5}deg at_target={} position_tracking_enabled={} decision={:?}",
                    d.tick, j + 1, d.elapsed_s, d.window_s, d.position_ticks, d.encoder_range_ticks,
                    d.requested_ticks, d.stall_below_ticks, d.stopped, d.current_ma, d.current_threshold_ma,
                    d.limit_ma, d.high_current_samples, d.fresh_samples, d.loaded, d.commanded_ticks_s,
                    d.encoder_ticks_s, d.advancing, d.tracking_error_deg, d.at_target,
                    d.position_tracking_enabled, d.decision
                ),
                Event::Configure(tick, j, limit, position) => format!(
                    "tick={tick} J{} CONFIGURE requested_limit={limit:.0}mA encoder={position}", j + 1
                ),
                Event::ReverseCheck(tick, j, contact, position, travel, minimum) => format!(
                    "tick={tick} J{} REVERSE contact_encoder={contact} encoder={position} reverse_travel={travel}ticks required={minimum}ticks moved_back={}",
                    j + 1, travel >= minimum
                ),
                Event::HomeRepeatability(tick, j, first, second, maximum, difference) => format!(
                    "tick={tick} J{} HOME first_encoder={first} second_encoder={second} difference={}ticks ({:.5}deg) allowed={maximum}ticks repeatable={}",
                    j + 1,
                    (i64::from(first) - i64::from(second)).abs(), difference.to_degrees(),
                    (i64::from(first) - i64::from(second)).abs() <= i64::from(maximum)
                ),
                Event::FeedbackGap(tick, j, silent) => format!(
                    "tick={tick} J{} answered again after {silent:.3} s of silence",
                    j + 1
                ),
                Event::IdentificationPose(i, total, q) => {
                    format!("IDENT pose {i}/{total} q={q:?}")
                }
                Event::IdentificationTorque(i, tau) => {
                    format!("IDENT pose {i} holding torque (both approaches averaged) {tau:?} Nm")
                }
                Event::ArmFit(before, after) => format!(
                    "IDENT arm fit: torque residual {before:.5} Nm -> {after:.5} Nm"
                ),
                Event::Tune(j, stage, gains, limit) => format!(
                    "J{} automatic trial: stage={stage:?} Kpv={:?} Kiv={:?} Kpp={:?} limit={limit:.0} mA",
                    j + 1, gains.kpv, gains.kiv, gains.kpp
                ),
                Event::TuneScore(j, trial, stage, score, accepted) => format!(
                    "J{} trial={trial} stage={stage:?} rank={score:?} accepted={accepted}", j + 1
                ),
                Event::NodeStatus(tick, j, status) => {
                    let f = status.flags.unwrap_or_default();
                    writeln!(
                        status_csv,
                        "{tick},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                        j + 1,
                        status.voltage_mv.map_or(-1, i32::from),
                        status.temperature_c.map_or(-1000, i32::from),
                        u8::from(status.live_error_bit),
                        u8::from(f.error),
                        u8::from(f.temperature),
                        u8::from(f.encoder),
                        u8::from(f.vbus),
                        u8::from(f.driver),
                        u8::from(f.velocity),
                        u8::from(f.current),
                        u8::from(f.estop),
                        u8::from(f.calibrated),
                        u8::from(f.watchdog),
                    )?;
                    continue;
                }
                Event::ToolDetected(node, info) => match info {
                    Some(i) => format!(
                        "TOOL gripper driver answered on node {node}: hw {}, batch {}, fw {}, serial {}",
                        i.hw_ver, i.batch, i.sw_ver, i.serial
                    ),
                    None => format!("TOOL no gripper driver answered on node {node}"),
                },
                Event::GainsAccepted(j, gains) => {
                    accepted.insert(format!("joint{}", j + 1), gains);
                    let rendered = toml::to_string_pretty(&accepted).map_err(std::io::Error::other)?;
                    fs::write(&accepted_path, rendered)?;
                    format!(
                        "J{} accepted gains Kpv={:?} Kiv={:?} Kpp={:?} recorded in accepted-gains.toml",
                        j + 1, gains.kpv, gains.kiv, gains.kpp
                    )
                }
                Event::Band(j, stage, band) => format!(
                    "J{} BAND stage={stage:?} lowest_pass={:?} highest_pass={:?} lower_fail={:?} upper_fail={:?} operating={:?} verified={} experiments={}",
                    j + 1, band.lowest_pass, band.highest_pass, band.lower_fail, band.upper_fail,
                    band.operating, band.verified, band.experiments
                ),
                Event::TuneStop(tick, j, reference, position, tolerance) => format!(
                    "tick={tick} J{} TUNING_RETURN reference={reference} encoder={position} difference={}ticks allowed={tolerance}ticks expected_stop=true action=score_trial",
                    j + 1, (i64::from(reference) - i64::from(position)).abs()
                ),
            };
            println!("{line}");
            writeln!(log, "{line}")?;
        }
        status_csv.flush()?;
        csv.flush()
    });
    unsafe {
        libc::signal(libc::SIGINT, cancel as *const () as libc::sighandler_t);
        libc::signal(libc::SIGTERM, cancel as *const () as libc::sighandler_t);
    }
    let mut arm = Arm::open(bundle, &assets, sim, Some(tx), quality_limits)?;
    arm.focused_joint = focused_joint;
    arm.planned_ready = planned_ready;
    arm.baseline_only = baseline_only;
    arm.trials = trials.unwrap_or(0) as usize;
    arm.home_retries = home_retries as usize;
    arm.feedback_timeout_s = feedback_timeout_s;
    arm.search = SearchLimits {
        step: gain_step,
        ceiling: gain_ceiling,
        resolution: gain_resolution,
    };
    if !sim {
        runtime::realtime(timing.cpu, timing.fifo_priority)?;
    }
    let mut identified: Option<par6_kin::gravity::ArmFit> = None;
    let outcome = arm.initialize().and_then(|()| {
        if quality_limits.is_some() {
            arm.prepare_holding()?;
        }
        if let Some(j) = probe_joint {
            arm.adaptive = true;
            let h = &arm.bundle.robot.homing.joints[j];
            let speed = (h.speed_ticks_s * if h.direction == 1 { -1.0 } else { 1.0 }) as i32;
            arm.emit(Event::Phase("isolated endstop probe", j))?;
            arm.motion(j, Motion::Seek(speed, false), false)?;
            return Ok(());
        }
        arm.home()?;
        if home_only {
            Ok(())
        } else {
            let poses = gravity_plan
                .as_ref()
                .ok_or("missing identification pose plan")?;
            let ready = planned_ready.ok_or("missing planned ready pose")?;
            let fit = arm.identify(poses, ready)?;
            arm.correction = fit.correction.clone();
            identified = Some(fit);
            Ok(())
        }
    });
    let shutdown = arm.shutdown();
    arm.events.take();
    drop(runtime);
    let recording = writer.join().map_err(|_| "recording thread failed")?;
    println!(
        "shutdown: {}",
        if shutdown.is_ok() {
            "completed"
        } else {
            "failed"
        }
    );
    let result = outcome.and(shutdown).and(recording.map_err(Into::into));
    fs::write(directory.join("result.txt"), format!("{result:?}\n"))?;
    writeln!(
        fs::OpenOptions::new()
            .append(true)
            .open(directory.join("console.log"))?,
        "RUN_RESULT: {result:?}"
    )?;
    result?;
    if probe_joint.is_some() {
        println!("Endstop probe completed; motors released. No calibrated configuration written.");
        return Ok(());
    }
    if baseline_only {
        println!("Baseline measurements completed; gains unchanged, motors released. No calibrated configuration written.");
        return Ok(());
    }
    for j in 0..N {
        arm.bundle.robot.joints[j].gains = arm.gains[j];
    }
    arm.bundle.robot.gravity_correction = arm.correction.clone();
    if let Some(fit) = &identified {
        // What the data fixed, beside what it left at the model's own
        // value — a low `determined` is the normal answer for a parameter
        // gravity cannot see, not a failure.
        let mut report = format!(
            "# Arm links identified from static torque, with the base attachment fitted.\n\
             # Torque residual {:.5} Nm, against {:.5} Nm for the model as it stood.\n\
             # `determined` is the share of each parameter the poses fixed, 0..1.\n",
            fit.rms_nm, fit.rms_before_nm
        );
        for (b, chunk) in fit.correction.chunks(4).enumerate() {
            let d = &fit.determined[b * 4..b * 4 + 4];
            writeln!(
                report,
                "\n[body{}]\nd_mass_kg = {:?}\nd_first_moment_kg_m = {:?}\ndetermined = {:?}",
                b + 1,
                chunk[0],
                &chunk[1..4],
                d
            )?;
        }
        fs::write(directory.join("identified-arm.toml"), report)?;
        println!(
            "identification: torque residual {:.5} Nm -> {:.5} Nm; {} of {} parameters fixed by the data",
            fit.rms_before_nm,
            fit.rms_nm,
            fit.determined.iter().filter(|d| **d > 0.5).count(),
            fit.determined.len()
        );
    }
    arm.bundle.robot.validate()?;
    let calibrated = toml::to_string_pretty(&arm.bundle.robot)?;
    let filename = if focused_joint.is_some() {
        "candidate.toml"
    } else {
        "calibrated.toml"
    };
    fs::write(directory.join(filename), &calibrated)?;
    if apply && !sim {
        let backup = config.with_extension("toml.before-selfcal");
        if !backup.exists() {
            fs::write(backup, &original)?;
        }
        // Patch the measured values into the file as written, so its comments
        // and layout survive; a full re-serialisation would discard them.
        // Only what this run measured: gravity factors stay as they were
        // when the gravity stage did not run.
        let patched = patch_config(
            std::str::from_utf8(&original)?,
            &arm.bundle.robot.joints,
            (!home_only).then_some(&arm.bundle.robot.gravity_scale[..]),
            (!home_only).then_some(&arm.bundle.robot.gravity_correction[..]),
        )?;
        if let Err(error) = toml::from_str::<par6_config::RobotConfig>(&patched)
            .map_err(|e| e.to_string())
            .and_then(|robot| robot.validate().map_err(|e| e.to_string()))
        {
            fs::write(directory.join("patched-config.toml"), &patched)?;
            return Err(format!(
                "patched configuration does not load ({error}); written to the run directory instead"
            )
            .into());
        }
        let temp = config.with_extension("toml.selfcal-tmp");
        fs::write(&temp, patched)?;
        fs::rename(temp, config)?;
    }
    println!(
        "Run completed. Results: {}",
        directory.join(filename).display()
    );
    Ok(())
}
