//! CLI/env surface of the `par6d` binary.
//!
//! Precedence: CLI flag > `PAR6_*` environment variable > robot TOML
//! `[protocol]` defaults. Every override is an `Option`, so a flag that
//! is not passed leaves the config file's value in place — clap
//! `default_value`s would silently outrank the TOML. Only the knobs a
//! deployment actually needs are exposed; everything else lives in the
//! config file.

use std::net::IpAddr;
use std::path::{Path, PathBuf};

use clap::builder::{BoolishValueParser, PossibleValuesParser, TypedValueParser};
use clap::{ArgAction, Parser};

pub use par6_server::StatusTransport;

/// PAR6 runtime daemon (protocol v2 command plane + RT core).
///
/// The default mode drives the SocketCAN backend on the configured
/// interface, brought up at the config bitrate when it is down (which
/// needs CAP_NET_ADMIN). `--sim` runs the closed-loop simulator backend
/// instead, which runs anywhere, CI included.
///
/// A flag beats its `PAR6_*` environment variable, which beats the
/// `[protocol]` section of the robot TOML.
#[derive(Parser, Debug, Clone, Default)]
#[command(name = "par6d", version)]
pub struct Options {
    /// Run the closed-loop simulator backend instead of hardware.
    #[arg(long)]
    pub sim: bool,

    /// Robot TOML (default: ./config/PAR6.toml, then <exe>/../../config/PAR6.toml).
    #[arg(long, value_name = "PATH", env = "PAR6_CONFIG")]
    pub config: Option<PathBuf>,

    /// assets/par6_description tree with the PAR6 URDFs, used by the
    /// kinematics stack (default: the tree next to the config directory).
    #[arg(long, value_name = "DIR", env = "PAR6_ASSETS")]
    pub assets: Option<PathBuf>,

    /// Where `package://` mesh URIs resolve.
    ///
    /// Needed when the assets tree is an installed package whose URDFs
    /// name their meshes by package URI — a pip-installed `par6` points
    /// this at its site-packages directory.
    #[arg(long, value_name = "DIR", env = "PAR6_PACKAGE_DIR")]
    pub package_dir: Option<PathBuf>,

    /// Profile the RT tick per phase (one clock read per phase) and log
    /// the running maxima and the last overrun's phase times once a second.
    #[arg(
        long,
        env = "PAR6_TICK_PROFILE",
        action = ArgAction::SetTrue,
        value_parser = BoolishValueParser::new(),
    )]
    pub tick_profile: bool,

    /// Command UDP port; 0 = ephemeral [config: protocol.command_port].
    ///
    /// The bound port is printed on stdout as `PAR6D_READY command_port=...`.
    #[arg(
        long = "port",
        alias = "command-port",
        value_name = "PORT",
        env = "PAR6_COMMAND_PORT"
    )]
    pub command_port: Option<u16>,

    /// Command-socket bind address [default: 0.0.0.0].
    #[arg(long, value_name = "IP", env = "PAR6_BIND")]
    pub bind: Option<IpAddr>,

    /// Unicast status destination [default: 127.0.0.1].
    #[arg(long, value_name = "IP", env = "PAR6_STATUS_HOST")]
    pub status_host: Option<IpAddr>,

    /// Status broadcast port [config: protocol.status_port].
    #[arg(long, value_name = "PORT", env = "PAR6_STATUS_PORT")]
    pub status_port: Option<u16>,

    /// Status transport ladder.
    #[arg(
        long,
        value_name = "MODE",
        env = "PAR6_STATUS_TRANSPORT",
        value_parser = PossibleValuesParser::new(["auto", "multicast", "unicast"])
            .map(|s| match s.as_str() {
                "auto" => StatusTransport::Auto,
                "multicast" => StatusTransport::Multicast,
                _ => StatusTransport::Unicast,
            }),
    )]
    pub status_transport: Option<StatusTransport>,

    /// STATUS broadcast rate; must divide the tick rate exactly
    /// [config: protocol.status_rate_hz].
    #[arg(long = "status-rate", value_name = "HZ", env = "PAR6_STATUS_RATE_HZ")]
    pub status_rate_hz: Option<u32>,

    /// Also write the activity logs there; stderr is unchanged.
    ///
    /// rt.log carries the RT thread (2 MiB x5) and commands.log the
    /// command plane, daemon and host vitals (20 MiB x5).
    #[arg(long, value_name = "DIR", env = "PAR6_LOG_DIR")]
    pub log_dir: Option<PathBuf>,

    /// Validate the config bundle (robot TOML + grippers) and exit:
    /// 0 = valid, 1 = invalid.
    #[arg(long)]
    pub check_config: bool,

    /// Exit when this process is no longer the parent, so a runtime a
    /// client spawned never outlives it.
    ///
    /// Pass the spawner's own pid; a parent that dies has its children
    /// reparented.
    #[arg(long, value_name = "PID", value_parser = clap::value_parser!(u32).range(1..))]
    pub parent_pid: Option<u32>,
}

/// Drop `PAR6_*` variables that are set but empty, so they read as unset.
///
/// A systemd unit's `Environment=PAR6_BIND=` leaves the name present and
/// empty, which would otherwise fail the parse and stop the daemon
/// booting. Call before the parse, while still single-threaded.
pub fn clear_empty_env() {
    let empty: Vec<String> = std::env::vars()
        .filter(|(k, v)| k.starts_with("PAR6_") && v.is_empty())
        .map(|(k, _)| k)
        .collect();
    for key in empty {
        std::env::remove_var(key);
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flags_parse_and_a_bare_value_flag_is_a_usage_error() {
        let o = Options::try_parse_from([
            "par6d",
            "--sim",
            "--log-dir",
            "/var/log/par6",
            "--tick-profile",
            "--port",
            "0",
            "--status-transport",
            "unicast",
            "--parent-pid",
            "42",
        ])
        .unwrap();
        assert_eq!(o.log_dir.as_deref(), Some(Path::new("/var/log/par6")));
        assert!(o.sim && o.tick_profile);
        assert_eq!(o.command_port, Some(0));
        assert_eq!(o.status_transport, Some(StatusTransport::Unicast));
        assert_eq!(o.parent_pid, Some(42));
        // `--command-port` is the long-form alias `--port` kept for the
        // deploy scripts that spell it out.
        assert_eq!(
            Options::try_parse_from(["par6d", "--command-port", "6001"])
                .unwrap()
                .command_port,
            Some(6001)
        );
        assert!(
            Options::try_parse_from(["par6d", "--log-dir"]).is_err(),
            "a bare --log-dir is a usage error"
        );
        assert!(
            Options::try_parse_from(["par6d", "--status-transport", "carrier-pigeon"]).is_err(),
            "the transport ladder has exactly three modes"
        );
        assert!(
            Options::try_parse_from(["par6d", "--parent-pid", "0"]).is_err(),
            "pid 0 is not a process to follow"
        );
    }
}
