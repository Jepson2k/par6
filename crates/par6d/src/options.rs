//! CLI/env surface of the `par6d` binary.
//!
//! Precedence: CLI flag > `PAR6_*` environment variable > robot TOML
//! `[protocol]` defaults. Only the knobs a deployment actually needs are
//! exposed; everything else lives in the config file.

use std::net::IpAddr;
use std::path::{Path, PathBuf};

pub use par6_server::StatusTransport;

/// Usage text printed for `--help` and argument errors.
pub const USAGE: &str = "\
par6d — PAR6 runtime daemon (protocol v2 command plane + RT core)

USAGE:
    par6d [--sim] [OPTIONS]

MODES:
    (default)                  Hardware mode: the SocketCAN backend on the
                               configured interface (brought up at the config
                               bitrate when it is down — needs CAP_NET_ADMIN).
    --sim                      Closed-loop simulator backend (runs anywhere).

OPTIONS:
    --config <PATH>            Robot TOML (default: $PAR6_CONFIG, then
                               ./config/PAR6.toml, then <exe>/../../config/PAR6.toml)
    --assets <DIR>             assets/par6_description tree with the PAR6 URDFs
                               (default: $PAR6_ASSETS, then the tree next to the
                               config directory). Used by the kinematics stack.
    --sim-dynamics             With --sim: torque-level physics plant (Pinocchio
                               forward dynamics) instead of the kinematic plant.
                               [env: PAR6_SIM_DYNAMICS=1]
    --port <PORT>              Command UDP port; 0 = ephemeral. The bound port is
                               printed on stdout as `PAR6D_READY command_port=...`.
                               [env: PAR6_COMMAND_PORT] [config: protocol.command_port]
    --bind <IP>                Command-socket bind address [env: PAR6_BIND] [default: 0.0.0.0]
    --status-host <IP>         Unicast status/telemetry destination
                               [env: PAR6_STATUS_HOST] [default: 127.0.0.1]
    --status-port <PORT>       Status broadcast port [env: PAR6_STATUS_PORT]
    --telemetry-port <PORT>    Telemetry stream port [env: PAR6_TELEMETRY_PORT]
    --status-transport <MODE>  auto | multicast | unicast [env: PAR6_STATUS_TRANSPORT]
    --status-rate <HZ>         STATUS broadcast rate; must divide the tick rate
                               exactly [env: PAR6_STATUS_RATE_HZ]
                               [config: protocol.status_rate_hz]
    -h, --help                 Print this help
";

/// Parsed command-line + environment options.
#[derive(Debug, Clone, Default)]
pub struct Options {
    /// Run the closed-loop simulator backend instead of hardware.
    pub sim: bool,
    /// Explicit robot TOML path (`--config` / `PAR6_CONFIG`).
    pub config: Option<PathBuf>,
    /// Explicit `assets/par6_description` tree (`--assets` / `PAR6_ASSETS`).
    pub assets: Option<PathBuf>,
    /// Run the sim on the torque-level dynamics plant (`--sim-dynamics` /
    /// `PAR6_SIM_DYNAMICS`); requires feature `ffi`.
    pub sim_dynamics: bool,
    /// Command UDP port override (0 = ephemeral).
    pub command_port: Option<u16>,
    /// Command-socket bind address override.
    pub bind: Option<IpAddr>,
    /// Unicast status/telemetry destination override.
    pub status_host: Option<IpAddr>,
    /// Status broadcast port override.
    pub status_port: Option<u16>,
    /// Telemetry stream port override.
    pub telemetry_port: Option<u16>,
    /// Status transport ladder override.
    pub status_transport: Option<StatusTransport>,
    /// STATUS broadcast rate override \[Hz\].
    pub status_rate_hz: Option<u32>,
    /// `--help` was requested.
    pub help: bool,
}

impl Options {
    /// Parse CLI arguments (without the program name), then fill unset
    /// fields from the `PAR6_*` environment.
    pub fn parse(args: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut o = Options::default();
        let mut args = args.peekable();
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--sim" => o.sim = true,
                "--config" => o.config = Some(PathBuf::from(value(&mut args, "--config")?)),
                "--assets" => o.assets = Some(PathBuf::from(value(&mut args, "--assets")?)),
                "--sim-dynamics" => o.sim_dynamics = true,
                "--port" | "--command-port" => {
                    o.command_port = Some(parse_num(&value(&mut args, &arg)?, &arg)?);
                }
                "--bind" => o.bind = Some(parse_ip(&value(&mut args, "--bind")?, "--bind")?),
                "--status-host" => {
                    o.status_host = Some(parse_ip(&value(&mut args, "--status-host")?, &arg)?);
                }
                "--status-port" => {
                    o.status_port = Some(parse_num(&value(&mut args, &arg)?, &arg)?);
                }
                "--telemetry-port" => {
                    o.telemetry_port = Some(parse_num(&value(&mut args, &arg)?, &arg)?);
                }
                "--status-transport" => {
                    o.status_transport = Some(parse_transport(&value(&mut args, &arg)?)?);
                }
                "--status-rate" => {
                    o.status_rate_hz = Some(parse_rate(&value(&mut args, &arg)?, &arg)?);
                }
                "-h" | "--help" => o.help = true,
                other => return Err(format!("unknown argument `{other}`\n\n{USAGE}")),
            }
        }
        o.fill_from_env()?;
        Ok(o)
    }

    fn fill_from_env(&mut self) -> Result<(), String> {
        if self.config.is_none() {
            if let Some(v) = env_var("PAR6_CONFIG") {
                self.config = Some(PathBuf::from(v));
            }
        }
        if self.assets.is_none() {
            if let Some(v) = env_var("PAR6_ASSETS") {
                self.assets = Some(PathBuf::from(v));
            }
        }
        if !self.sim_dynamics {
            if let Some(v) = env_var("PAR6_SIM_DYNAMICS") {
                self.sim_dynamics = v == "1" || v.eq_ignore_ascii_case("true");
            }
        }
        if self.command_port.is_none() {
            if let Some(v) = env_var("PAR6_COMMAND_PORT") {
                self.command_port = Some(parse_num(&v, "PAR6_COMMAND_PORT")?);
            }
        }
        if self.bind.is_none() {
            if let Some(v) = env_var("PAR6_BIND") {
                self.bind = Some(parse_ip(&v, "PAR6_BIND")?);
            }
        }
        if self.status_host.is_none() {
            if let Some(v) = env_var("PAR6_STATUS_HOST") {
                self.status_host = Some(parse_ip(&v, "PAR6_STATUS_HOST")?);
            }
        }
        if self.status_port.is_none() {
            if let Some(v) = env_var("PAR6_STATUS_PORT") {
                self.status_port = Some(parse_num(&v, "PAR6_STATUS_PORT")?);
            }
        }
        if self.telemetry_port.is_none() {
            if let Some(v) = env_var("PAR6_TELEMETRY_PORT") {
                self.telemetry_port = Some(parse_num(&v, "PAR6_TELEMETRY_PORT")?);
            }
        }
        if self.status_transport.is_none() {
            if let Some(v) = env_var("PAR6_STATUS_TRANSPORT") {
                self.status_transport = Some(parse_transport(&v)?);
            }
        }
        if self.status_rate_hz.is_none() {
            if let Some(v) = env_var("PAR6_STATUS_RATE_HZ") {
                self.status_rate_hz = Some(parse_rate(&v, "PAR6_STATUS_RATE_HZ")?);
            }
        }
        Ok(())
    }
}

/// Resolve the robot TOML path: the explicit choice when given, else the
/// first existing default location. The error names every path tried.
pub fn resolve_config_path(explicit: Option<&Path>) -> Result<PathBuf, String> {
    if let Some(p) = explicit {
        return if p.is_file() {
            Ok(p.to_path_buf())
        } else {
            Err(format!("config file not found: {}", p.display()))
        };
    }
    let mut candidates = vec![PathBuf::from("config/PAR6.toml")];
    if let Ok(exe) = std::env::current_exe() {
        // target/{debug,release}/par6d → repo config/ two levels up.
        if let Some(dir) = exe.parent() {
            candidates.push(dir.join("../../config/PAR6.toml"));
            candidates.push(dir.join("../../../config/PAR6.toml"));
        }
    }
    for c in &candidates {
        if c.is_file() {
            return Ok(c.clone());
        }
    }
    Err(format!(
        "no robot config found; set --config or PAR6_CONFIG (tried: {})",
        candidates
            .iter()
            .map(|c| c.display().to_string())
            .collect::<Vec<_>>()
            .join(", ")
    ))
}

fn env_var(name: &str) -> Option<String> {
    std::env::var(name).ok().filter(|v| !v.is_empty())
}

fn value(
    args: &mut std::iter::Peekable<impl Iterator<Item = String>>,
    flag: &str,
) -> Result<String, String> {
    args.next()
        .ok_or_else(|| format!("{flag} requires a value"))
}

fn parse_num(v: &str, what: &str) -> Result<u16, String> {
    v.parse::<u16>()
        .map_err(|_| format!("{what}: invalid port `{v}`"))
}

fn parse_rate(v: &str, what: &str) -> Result<u32, String> {
    v.parse::<u32>()
        .map_err(|_| format!("{what}: invalid rate `{v}`"))
}

fn parse_ip(v: &str, what: &str) -> Result<IpAddr, String> {
    v.parse::<IpAddr>()
        .map_err(|_| format!("{what}: invalid IP address `{v}`"))
}

fn parse_transport(v: &str) -> Result<StatusTransport, String> {
    match v {
        "auto" => Ok(StatusTransport::Auto),
        "multicast" => Ok(StatusTransport::Multicast),
        "unicast" => Ok(StatusTransport::Unicast),
        other => Err(format!(
            "--status-transport: `{other}` is not auto|multicast|unicast"
        )),
    }
}
