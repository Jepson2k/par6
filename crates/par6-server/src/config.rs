//! Server configuration: sockets, rates, transport ladder knobs, queue
//! sizing, and the server-layer registries (tools / profiles / telemetry
//! recipes) the codec deliberately does not know about.

use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::time::Duration;

use par6_config::ProtocolConfig;
use par6_proto::Shape;

use crate::telemetry::TelemetryRecipe;

/// Status/telemetry transport selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StatusTransport {
    /// Multicast with a startup reachability probe; permanent unicast
    /// failover on probe failure or 3 consecutive send errors (the spec
    /// ladder, the default).
    Auto,
    /// Multicast without a probe (still fails over on send errors).
    Multicast,
    /// Unicast only.
    Unicast,
}

/// Complete command-plane configuration. Every knob has a sensible
/// default; [`ServerConfig::from_protocol`] fills the wire-facing fields
/// from the `[protocol]` section of the robot TOML.
#[derive(Debug, Clone)]
pub struct ServerConfig {
    /// Command socket bind address (`0.0.0.0:command_port` in prod;
    /// `127.0.0.1:0` in tests).
    pub bind: SocketAddr,
    /// Stable controller id carried in the STATUS header.
    pub controller_id: u32,
    /// Transport ladder mode.
    pub status_transport: StatusTransport,
    /// Status multicast group.
    pub multicast_group: Ipv4Addr,
    /// Multicast TTL.
    pub multicast_ttl: u32,
    /// Interface address used when joining the group for the probe.
    pub multicast_iface: Ipv4Addr,
    /// Unicast destination host (probe-failure / send-error fallback,
    /// or the sole destination in [`StatusTransport::Unicast`] mode).
    pub status_dest_host: IpAddr,
    /// Status broadcast port.
    pub status_port: u16,
    /// Status broadcast rate \[Hz\].
    pub status_rate_hz: u32,
    /// Telemetry stream port.
    pub telemetry_port: u16,
    /// Telemetry stream rate \[Hz\].
    pub telemetry_rate_hz: u32,
    /// Telemetry recipe registry; `set_recipe` refuses names not listed
    /// here.
    pub recipes: Vec<TelemetryRecipe>,
    /// Recipe active at startup; `None` = telemetry off until
    /// `set_recipe`.
    pub initial_recipe: Option<String>,
    /// Reachability-probe reply timeout.
    pub probe_timeout: Duration,
    /// RT tick rate \[Hz\] (loop-stats reporting).
    pub rt_tick_rate_hz: f64,
    /// Snapshot age beyond which STATUS reports `link_ok = 0`.
    pub link_stale: Duration,
    /// Pending-queue capacity; enqueueing beyond it is `COMM_QUEUE_FULL`.
    pub queue_capacity: usize,
    /// Idempotency dedup window (last N keys → index).
    pub dedup_window: usize,
    /// Chunked-transfer inactivity timeout (`COMM_CHUNK_TIMEOUT`).
    pub chunk_timeout: Duration,
    /// Cadence of the internal housekeeping tick (planner outcome
    /// polling, queue pumping, chunk expiry).
    pub poll_interval: Duration,
    /// Whether the simulator backend is active at startup.
    pub simulator: bool,
    /// Tool registry keys (`select_tool` / `tool_action` validation and
    /// the TOOLS query). Matched case-insensitively on the wire.
    pub tools: Vec<String>,
    /// The tool the runtime is actually fitted with — active from startup
    /// (and after `reset_state`), and the only key `select_tool` accepts:
    /// swapping a tool changes the kinematic model, which is a restart.
    /// Empty = no tool.
    pub fitted_tool: String,
    /// Controllable degrees of freedom of the fitted tool. 0 = passive:
    /// `tool_action` and `teleport`'s `tool_positions` are refused.
    pub tool_dof: usize,
    /// Whether this runtime has kinematics. `false` refuses the cartesian
    /// STREAMING commands (`servo_j_pose` / `servo_l` / `jog_l`), which
    /// would otherwise be dropped without a word — a fire-and-forget
    /// command the runtime cannot execute must still say so.
    pub cartesian: bool,
    /// Motion profile names (`select_profile` validation).
    pub profiles: Vec<String>,
    /// Profile active at startup (and after `reset_state`).
    pub initial_profile: String,
    /// Installation-layer collision shapes (persistent keep-outs,
    /// reported by the SHAPES query alongside the program layer).
    pub installation_shapes: Vec<Shape>,
}

impl Default for ServerConfig {
    fn default() -> Self {
        Self {
            bind: SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 6001),
            controller_id: 1,
            status_transport: StatusTransport::Auto,
            multicast_group: Ipv4Addr::new(239, 255, 0, 71),
            multicast_ttl: 1,
            multicast_iface: Ipv4Addr::LOCALHOST,
            status_dest_host: IpAddr::V4(Ipv4Addr::LOCALHOST),
            status_port: 6002,
            status_rate_hz: 50,
            telemetry_port: 6003,
            telemetry_rate_hz: 100,
            recipes: TelemetryRecipe::defaults(),
            initial_recipe: None,
            probe_timeout: Duration::from_millis(200),
            rt_tick_rate_hz: 250.0,
            link_stale: Duration::from_millis(200),
            queue_capacity: 128,
            dedup_window: 256,
            chunk_timeout: Duration::from_secs(2),
            poll_interval: Duration::from_millis(2),
            simulator: false,
            tools: Vec::new(),
            fitted_tool: String::new(),
            tool_dof: 0,
            cartesian: true,
            profiles: vec!["default".to_owned()],
            initial_profile: "default".to_owned(),
            installation_shapes: Vec::new(),
        }
    }
}

impl ServerConfig {
    /// Build a config from the robot TOML `[protocol]` section, leaving
    /// every other knob at its default.
    pub fn from_protocol(p: &ProtocolConfig) -> Self {
        let group = p
            .status_multicast_group
            .parse::<Ipv4Addr>()
            .unwrap_or(Ipv4Addr::new(239, 255, 0, 71));
        Self {
            bind: SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), p.command_port),
            multicast_group: group,
            status_port: p.status_port,
            status_rate_hz: p.status_rate_hz,
            telemetry_port: p.telemetry_port,
            ..Self::default()
        }
    }
}
