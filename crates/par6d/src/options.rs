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
    --status-host <IP>         Unicast status destination
                               [env: PAR6_STATUS_HOST] [default: 127.0.0.1]
    --status-port <PORT>       Status broadcast port [env: PAR6_STATUS_PORT]
    --status-transport <MODE>  auto | multicast | unicast [env: PAR6_STATUS_TRANSPORT]
    --status-rate <HZ>         STATUS broadcast rate; must divide the tick rate
                               exactly [env: PAR6_STATUS_RATE_HZ]
                               [config: protocol.status_rate_hz]
    --tick-profile             Profile the RT tick per phase (one clock read per
                               phase) and log the running maxima and the last
                               overrun's phase times once a second.
                               [env: PAR6_TICK_PROFILE=1]
    --log-dir <DIR>            Also write the activity logs there: rt.log (the RT
                               thread, 2 MiB x5) and commands.log (command plane,
                               daemon, host vitals, 20 MiB x5). stderr is unchanged.
                               [env: PAR6_LOG_DIR]
    --check-config             Validate the config bundle (robot TOML + grippers)
                               and exit: 0 = valid, 1 = invalid.
    --parent-pid <PID>         Exit when this process is no longer the parent
                               (the spawner's own pid; a parent that dies has
                               its children reparented), so a runtime a client
                               spawned never outlives it.
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
    /// Where `package://` mesh URIs resolve. Set when the assets tree is
    /// an installed package whose URDFs reference their meshes by package
    /// URI rather than a repo checkout's `<assets>/URDF` layout.
    pub package_dir: Option<PathBuf>,
    /// Run the sim on the torque-level dynamics plant (`--sim-dynamics` /
    /// `PAR6_SIM_DYNAMICS`); requires feature `ffi`.
    pub sim_dynamics: bool,
    /// Run the RT tick's per-phase profiler (`--tick-profile` /
    /// `PAR6_TICK_PROFILE`); the profile is logged once a second.
    pub tick_profile: bool,
    /// Command UDP port override (0 = ephemeral).
    pub command_port: Option<u16>,
    /// Command-socket bind address override.
    pub bind: Option<IpAddr>,
    /// Unicast status destination override.
    pub status_host: Option<IpAddr>,
    /// Status broadcast port override.
    pub status_port: Option<u16>,
    /// Status transport ladder override.
    pub status_transport: Option<StatusTransport>,
    /// STATUS broadcast rate override \[Hz\].
    pub status_rate_hz: Option<u32>,
    /// Directory for the rotating activity logs (`--log-dir` /
    /// `PAR6_LOG_DIR`); `None` = stderr only.
    pub log_dir: Option<PathBuf>,
    /// `--check-config` was requested: validate the bundle and exit.
    pub check_config: bool,
    /// Die with this process (`--parent-pid`): the spawner's pid, compared
    /// against `getppid` before boot and from the main loop after it.
    pub parent_pid: Option<u32>,
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
                "--tick-profile" => o.tick_profile = true,
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
                "--status-transport" => {
                    o.status_transport = Some(parse_transport(&value(&mut args, &arg)?)?);
                }
                "--status-rate" => {
                    o.status_rate_hz = Some(parse_rate(&value(&mut args, &arg)?, &arg)?);
                }
                "--log-dir" => o.log_dir = Some(PathBuf::from(value(&mut args, "--log-dir")?)),
                "--check-config" => o.check_config = true,
                "--parent-pid" => {
                    let raw = value(&mut args, &arg)?;
                    let pid: u32 = raw
                        .parse()
                        .ok()
                        .filter(|p| *p > 0)
                        .ok_or_else(|| format!("--parent-pid: `{raw}` is not a process id"))?;
                    o.parent_pid = Some(pid);
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
        if !self.tick_profile {
            if let Some(v) = env_var("PAR6_TICK_PROFILE") {
                self.tick_profile = v == "1" || v.eq_ignore_ascii_case("true");
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
        if self.log_dir.is_none() {
            if let Some(v) = env_var("PAR6_LOG_DIR") {
                self.log_dir = Some(PathBuf::from(v));
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
    // The one search order for everything that loads a config — the
    // daemon, the Python binding, the preview: the environment, a repo
    // checkout around the binary, then the deploy bundle's install
    // location on a control box.
    let mut candidates = Vec::new();
    if let Ok(p) = std::env::var("PAR6_CONFIG") {
        candidates.push(PathBuf::from(p));
    }
    candidates.push(PathBuf::from("config/PAR6.toml"));
    if let Ok(exe) = std::env::current_exe() {
        // target/{debug,release}/par6d → repo config/ two levels up.
        if let Some(dir) = exe.parent() {
            candidates.push(dir.join("../../config/PAR6.toml"));
            candidates.push(dir.join("../../../config/PAR6.toml"));
        }
    }
    candidates.push(PathBuf::from("/etc/par6/PAR6.toml"));
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn log_dir_comes_from_the_flag_or_the_environment() {
        let o = Options::parse(
            ["--sim", "--log-dir", "/var/log/par6"]
                .map(String::from)
                .into_iter(),
        )
        .unwrap();
        assert_eq!(
            o.log_dir.as_deref(),
            Some(std::path::Path::new("/var/log/par6"))
        );
        assert!(
            Options::parse(["--log-dir"].map(String::from).into_iter()).is_err(),
            "a bare --log-dir is a usage error"
        );
        assert!(
            Options::parse(["--sim", "--tick-profile"].map(String::from).into_iter())
                .unwrap()
                .tick_profile
        );
    }
}
