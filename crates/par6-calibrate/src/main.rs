//! `par6-calibrate-gravity`: identify the arm's gravity model against a
//! running `par6d` and, on request, write it into the arm URDF.

use std::path::PathBuf;
use std::time::Duration;

use par6_client::{Client, ClientConfig};
use par6_config::ConfigBundle;
use par6_kin::{Collision, GripperVariant, Kin, NQ};

const USAGE: &str = "\
par6-calibrate-gravity — fit link masses and centres of mass to the torques the arm holds

  --config PATH       robot TOML (default: the daemon's own search)
  --assets PATH       assets tree with URDF/ (default: assets/par6_description next to the config)
  --host HOST         command endpoint (default: PAR6_HOST or 127.0.0.1)
  --port PORT         command port (default: PAR6_COMMAND_PORT or 6001)
  --poses N           poses to rest in (default 24)
  --holdout N         of those, poses kept back to score the fit (default 6)
  --seed N            pose draw seed (default 1)
  --speed F           joint-move speed fraction between poses (default 0.5)
  --approach RAD      offset each pose is approached from, both ways (default 0.05)
  --settle S          rest before reading each pose (default 0.5)
  --frames N          STATUS frames averaged per pose (default 25)
  --prior-weight W    pull toward the current URDF, 0 = data only (default 0.01)
  --write             rewrite URDF/par6_flange/urdf/par6_arm.urdf in the assets tree
";

struct Args {
    config: Option<PathBuf>,
    assets: Option<PathBuf>,
    host: Option<String>,
    port: Option<u16>,
    poses: usize,
    holdout: usize,
    seed: u64,
    protocol: par6_calibrate::Protocol,
    prior_weight: f64,
    write: bool,
}

fn parse(mut argv: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut a = Args {
        config: None,
        assets: None,
        host: None,
        port: None,
        poses: 24,
        holdout: 6,
        seed: 1,
        protocol: par6_calibrate::Protocol::default(),
        prior_weight: 0.01,
        write: false,
    };
    let value = |flag: &str, argv: &mut dyn Iterator<Item = String>| {
        argv.next().ok_or_else(|| format!("{flag} needs a value"))
    };
    while let Some(flag) = argv.next() {
        match flag.as_str() {
            "--config" => a.config = Some(PathBuf::from(value(&flag, &mut argv)?)),
            "--assets" => a.assets = Some(PathBuf::from(value(&flag, &mut argv)?)),
            "--host" => a.host = Some(value(&flag, &mut argv)?),
            "--port" => a.port = Some(num(&value(&flag, &mut argv)?, &flag)?),
            "--poses" => a.poses = num(&value(&flag, &mut argv)?, &flag)?,
            "--holdout" => a.holdout = num(&value(&flag, &mut argv)?, &flag)?,
            "--seed" => a.seed = num(&value(&flag, &mut argv)?, &flag)?,
            "--speed" => a.protocol.speed = num(&value(&flag, &mut argv)?, &flag)?,
            "--approach" => a.protocol.approach_rad = num(&value(&flag, &mut argv)?, &flag)?,
            "--settle" => {
                let s: f64 = num(&value(&flag, &mut argv)?, &flag)?;
                // `from_secs_f64` panics on a negative or a NaN.
                if !(s.is_finite() && s >= 0.0) {
                    return Err("--settle must be finite and >= 0".into());
                }
                a.protocol.settle = Duration::from_secs_f64(s);
            }
            "--frames" => a.protocol.frames = num(&value(&flag, &mut argv)?, &flag)?,
            "--prior-weight" => a.prior_weight = num(&value(&flag, &mut argv)?, &flag)?,
            "--write" => a.write = true,
            "-h" | "--help" => return Err(USAGE.to_owned()),
            other => return Err(format!("unknown argument {other}\n\n{USAGE}")),
        }
    }
    if a.poses < 2 || a.holdout >= a.poses {
        return Err("--poses must exceed --holdout, with at least two poses".into());
    }
    // Every one of these divides, scales or times something. A zero or a
    // negative turns a measurement into a NaN or a panic several minutes
    // into a run on real hardware, so they are refused up front.
    if a.protocol.frames == 0 {
        return Err("--frames must be at least 1".into());
    }
    if !(a.protocol.speed.is_finite() && a.protocol.speed > 0.0 && a.protocol.speed <= 1.0) {
        return Err("--speed must be in (0, 1]".into());
    }
    if !(a.protocol.approach_rad.is_finite() && a.protocol.approach_rad > 0.0) {
        return Err("--approach must be finite and > 0".into());
    }
    if !(a.prior_weight.is_finite() && a.prior_weight >= 0.0) {
        return Err("--prior-weight must be finite and >= 0".into());
    }
    Ok(a)
}

fn num<T: std::str::FromStr>(v: &str, flag: &str) -> Result<T, String> {
    v.parse().map_err(|_| format!("{flag}: cannot parse {v:?}"))
}

fn run(a: Args) -> Result<(), String> {
    let config_path = match a.config {
        Some(p) => p,
        None => par6d_config_search()?,
    };
    let bundle = ConfigBundle::load(&config_path).map_err(|e| e.to_string())?;
    let robot = &bundle.robot;
    let assets = a.assets.unwrap_or_else(|| {
        config_path
            .parent()
            .map(|d| d.join("../assets/par6_description"))
            .unwrap_or_else(|| PathBuf::from("assets/par6_description"))
    });
    let gripper = bundle.active_gripper();
    let tool = gripper.map(|g| {
        let k = &g.kinematics;
        Kin::dh_tool_params(
            k.d_m,
            k.a_m,
            k.alpha_rad,
            k.mass_kg,
            k.com_m,
            k.inertia_kg_m2,
        )
    });
    let mut kin = Kin::load_arm(&assets, tool.as_ref()).map_err(|e| e.to_string())?;
    let variant = GripperVariant::resolve(
        &robot.robot.active_gripper.to_ascii_uppercase(),
        gripper.and_then(|g| g.urdf_variant.as_deref()),
    );
    let mut collision = Collision::load(&assets, variant, 0.0).map_err(|e| e.to_string())?;
    let mut window = [(0.0, 0.0); NQ];
    for (w, j) in window.iter_mut().zip(robot.joints.iter()) {
        *w = (j.limits.soft_min_rad, j.limits.soft_max_rad);
    }
    let poses = par6_calibrate::plan_poses(
        &mut collision,
        &window,
        a.poses,
        a.seed,
        a.protocol.approach_rad,
    )?;

    let mut cfg = ClientConfig::default();
    if let Some(h) = a.host {
        cfg.host = h;
    }
    if let Some(p) = a.port {
        cfg.port = p;
    }
    let rt = tokio::runtime::Runtime::new().map_err(|e| e.to_string())?;
    let report = rt.block_on(async {
        let client = Client::connect(cfg).await.map_err(|e| e.to_string())?;
        let report = par6_calibrate::calibrate(
            &client,
            &mut kin,
            &poses,
            a.holdout,
            &a.protocol,
            a.prior_weight,
        )
        .await;
        client.close_joined().await;
        report
    })?;
    print!("{}", par6_calibrate::describe(&report));

    if a.write {
        // A fit that predicts held-out poses WORSE than the URDF already
        // does is not an improvement, and this rewrites the model the
        // daemon loads next boot. Refuse rather than make the arm worse.
        let improved = report.holdout_rms_fit_nm.is_finite()
            && report.holdout_rms_fit_nm < report.holdout_rms_prior_nm;
        if !improved {
            return Err(format!(
                "refusing to write: the fit predicts held-out poses no better than the \
                 current model ({:.4} Nm vs {:.4} Nm). Measure more poses, or spread them \
                 wider, before writing.",
                report.holdout_rms_fit_nm, report.holdout_rms_prior_nm
            ));
        }
        let arm = par6_calibrate::arm_params(&kin, &report.fit.bodies)?;
        let urdf = assets.join(Kin::ARM_URDF_RELPATH);
        let backup = write_inertials_with_backup(&urdf, &arm)?;
        println!(
            "wrote {} (previous model kept at {})",
            urdf.display(),
            backup.display()
        );
    }
    Ok(())
}

/// Rewrite `urdf` in place, keeping the previous model beside it and
/// swapping the new one in by rename — so an interrupted write cannot
/// leave the daemon with half a URDF to load.
fn write_inertials_with_backup(
    urdf: &std::path::Path,
    bodies: &[par6_kin::gravity::BodyParams],
) -> Result<PathBuf, String> {
    let text = std::fs::read_to_string(urdf).map_err(|e| format!("{}: {e}", urdf.display()))?;
    let rewritten = par6_kin::gravity::rewrite_inertials(&text, bodies)?;
    let backup = urdf.with_extension("urdf.bak");
    std::fs::write(&backup, &text).map_err(|e| format!("{}: {e}", backup.display()))?;
    let staged = urdf.with_extension("urdf.new");
    std::fs::write(&staged, &rewritten).map_err(|e| format!("{}: {e}", staged.display()))?;
    std::fs::rename(&staged, urdf).map_err(|e| format!("{}: {e}", urdf.display()))?;
    Ok(backup)
}

/// The daemon's config search, so a bare run on the control box finds
/// what `par6d` runs with.
fn par6d_config_search() -> Result<PathBuf, String> {
    if let Ok(p) = std::env::var("PAR6_CONFIG") {
        return Ok(PathBuf::from(p));
    }
    for candidate in ["config/PAR6.toml", "/etc/par6/PAR6.toml"] {
        let p = PathBuf::from(candidate);
        if p.is_file() {
            return Ok(p);
        }
    }
    Err("no robot config: pass --config or set PAR6_CONFIG".into())
}

fn main() {
    env_logger::init();
    let args = match parse(std::env::args().skip(1)) {
        Ok(a) => a,
        Err(msg) => {
            eprintln!("{msg}");
            std::process::exit(2);
        }
    };
    if let Err(msg) = run(args) {
        eprintln!("par6-calibrate-gravity: {msg}");
        std::process::exit(1);
    }
}
