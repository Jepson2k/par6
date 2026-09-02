//! `par6-identify-payload`: swing the wrist and work out what the arm is
//! carrying, then declare it to the running `par6d`.

use std::path::PathBuf;
use std::time::Duration;

use par6_client::{Client, ClientConfig};
use par6_config::ConfigBundle;
use par6_kin::{Collision, GripperVariant, Kin, NQ};

const USAGE: &str = "\
par6-identify-payload — measure the load at the tool from the torques the arm holds

Swings the WRIST where the arm already stands, so the arm below is not
disturbed and the whole run takes seconds. The arm's own link inertials
are never touched: those are the vendor's and stay that way.

  --config PATH       robot TOML (default: the daemon's own search)
  --assets PATH       assets tree with URDF/ (default: assets/par6_description next to the config)
  --host HOST         command endpoint (default: PAR6_HOST or 127.0.0.1)
  --port PORT         command port (default: PAR6_COMMAND_PORT or 6001)
  --spread RAD        how far each wrist joint swings either way (default 0.5)
  --speed F           joint-move speed fraction (default 1.0)
  --approach RAD      offset each pose is approached from, both ways (default 0.05)
  --settle S          rest before reading each pose (default 0.25)
  --frames N          STATUS frames averaged per pose (default 20)
  --ridge W           hold back parameters the poses do not measure (default 0.01)
  --declare           send the result to the runtime as its payload
";

struct Args {
    config: Option<PathBuf>,
    assets: Option<PathBuf>,
    host: Option<String>,
    port: Option<u16>,
    spread: f64,
    protocol: par6_calibrate::Protocol,
    ridge: f64,
    declare: bool,
}

fn parse(mut argv: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut a = Args {
        config: None,
        assets: None,
        host: None,
        port: None,
        spread: 0.5,
        protocol: par6_calibrate::Protocol::default(),
        ridge: 0.01,
        declare: false,
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
            "--spread" => a.spread = num(&value(&flag, &mut argv)?, &flag)?,
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
            "--ridge" => a.ridge = num(&value(&flag, &mut argv)?, &flag)?,
            "--declare" => a.declare = true,
            "-h" | "--help" => return Err(USAGE.to_owned()),
            other => return Err(format!("unknown argument {other}\n\n{USAGE}")),
        }
    }
    if !(a.spread.is_finite() && a.spread > 0.0) {
        return Err("--spread must be finite and > 0".into());
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
    if !(a.ridge.is_finite() && a.ridge >= 0.0) {
        return Err("--ridge must be finite and >= 0".into());
    }
    Ok(a)
}

fn num<T: std::str::FromStr>(v: &str, flag: &str) -> Result<T, String> {
    v.parse().map_err(|_| format!("{flag}: cannot parse {v:?}"))
}

fn run(a: Args) -> Result<(), String> {
    let (host, port) = (a.host.clone(), a.port);
    let config_path = match a.config.clone() {
        Some(p) => p,
        None => par6d_config_search()?,
    };
    let bundle = ConfigBundle::load(&config_path).map_err(|e| e.to_string())?;
    let robot = &bundle.robot;
    let assets = a.assets.clone().unwrap_or_else(|| {
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
    let mut cfg = ClientConfig::default();
    if let Some(h) = host {
        cfg.host = h;
    }
    if let Some(p) = port {
        cfg.port = p;
    }

    let rt = tokio::runtime::Runtime::new().map_err(|e| e.to_string())?;
    rt.block_on(async {
        let client = Client::connect(cfg).await.map_err(|e| e.to_string())?;
        let out = run_identification(&client, &mut kin, &mut collision, &window, &a).await;
        client.close_joined().await;
        out
    })
}

/// Measure from where the arm stands, then say what was found.
async fn run_identification(
    client: &Client,
    kin: &mut Kin,
    collision: &mut Collision,
    window: &[(f64, f64); NQ],
    a: &Args,
) -> Result<(), String> {
    // The residual the fit explains is torque the UNLOADED model cannot
    // account for, so whatever the runtime was carrying has to go first —
    // otherwise the arm is asked to identify a load it is already
    // compensating for and reports nothing.
    client
        .set_payload(0.0, [0.0; 3], None)
        .await
        .map_err(|e| format!("clearing the payload: {e}"))?;

    let angles = client
        .angles()
        .await
        .map_err(|e| format!("reading the pose: {e}"))?;
    let mut start = [0.0; NQ];
    for (out, deg) in start.iter_mut().zip(angles.iter()) {
        *out = deg.to_radians();
    }

    let poses =
        par6_calibrate::plan_poses(collision, &start, window, a.spread, a.protocol.approach_rad)?;
    println!("swinging the wrist through {} poses", poses.len());

    let report = par6_calibrate::identify(client, kin, &poses, &a.protocol, a.ridge).await?;
    print!("{}", par6_calibrate::describe(&report));

    if a.declare {
        // Nothing measurable means nothing to declare: a wrist that
        // could not move, or an arm carrying nothing, both land here, and
        // pushing a noise-level mass would make the gravity model worse
        // than leaving it empty.
        if report.fit.determined[0] <= par6_calibrate::MEASURED {
            return Err(format!(
                "refusing to declare: the poses did not measure the mass                  (determined {:.2}). Give the wrist more room, or a wider --spread.",
                report.fit.determined[0]
            ));
        }
        if !(report.fit.mass.is_finite() && report.fit.mass > 0.0) {
            return Err(format!(
                "refusing to declare a mass of {:.4} kg",
                report.fit.mass
            ));
        }
        client
            .set_payload(report.fit.mass, report.fit.com, None)
            .await
            .map_err(|e| format!("declaring the payload: {e}"))?;
        println!("declared {:.3} kg to the runtime", report.fit.mass);
    }
    Ok(())
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
