//! Server configuration: sockets, rates, transport ladder knobs, queue
//! sizing, and the server-layer registries (tools / profiles) the codec
//! deliberately does not know about.

use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::time::Duration;

use par6_config::ProtocolConfig;
use par6_proto::{Shape, NUM_JOINTS};

/// Status transport selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StatusTransport {
    /// Multicast with a startup reachability probe; permanent unicast
    /// failover on probe failure or 3 consecutive send errors (the protocol
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
    /// Reachability-probe reply timeout.
    pub probe_timeout: Duration,
    /// RT tick rate \[Hz\] (loop-stats reporting).
    pub rt_tick_rate_hz: f64,
    /// Motor-bus freshness window: STATUS reports `link_ok = 0` when no
    /// node's data (per-node `data_age_ticks`, aged by the snapshot's
    /// wall age) is younger than this.
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
    /// How many queued commands the planner may see ahead of the one it
    /// is about to start, so it can blend them into a single motion.
    /// 1 = no lookahead: every move runs alone and stops at its target.
    pub blend_lookahead: usize,
    /// How long a move whose blend radius is positive waits at the head
    /// of the queue for the successor it is meant to blend into.
    ///
    /// A corner cannot be rounded before both of its segments are known,
    /// so the first move of a chain has to be held briefly. It costs
    /// nothing once a program is streaming commands — the successor is
    /// then already queued behind a move that is still running — and it
    /// bounds how long a blended move can sit still if the successor
    /// never comes.
    pub blend_hold: Duration,
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
    /// Names of the declared digital output lines, in `write_io` port
    /// order — `write_io(port, …)` is refused past the end of this list,
    /// naming the line count the box actually has. Empty = a box that
    /// drives no outputs, and every `write_io` is refused.
    pub digital_outputs: Vec<String>,
    /// CAN node ids `set_pid_gains` may target (the configured joint
    /// nodes plus the CAN gripper node when one is fitted). Empty = a
    /// runtime with no tunable drives, and every `set_pid_gains` is
    /// refused.
    pub tunable_nodes: Vec<u8>,
    /// Motion profile names (`select_profile` validation).
    pub profiles: Vec<String>,
    /// Profile active at startup (and after `reset_state`).
    pub initial_profile: String,
    /// Per-joint hard travel window \[degrees\], `(min, max)` in wire
    /// units and kinematic order. `teleport` is refused outside it: the
    /// runtime cannot place a joint there, and clamping into range put
    /// the arm somewhere the client never asked for and reported
    /// success. Unbounded by default so a config that declares no limits
    /// constrains nothing.
    pub joint_hard_limits_deg: [(f64, f64); NUM_JOINTS],
    /// Installation-layer collision shapes (persistent keep-outs,
    /// reported by the SHAPES query alongside the program layer).
    /// `par6d` fills this from the robot TOML's `[[installation_shapes]]`
    /// array; the server pushes it into both collision gates at spawn and
    /// refuses to start on a shape the world cannot apply.
    pub installation_shapes: Vec<Shape>,
    /// Effective-configuration readback served for the CONFIG_INFO
    /// query. The daemon fills it from the loaded bundle at startup.
    pub config_info: ConfigInfoData,
}

/// The CONFIG_INFO payload: where the runtime's config came from, what
/// its content hashes to, and the effective values a client would want
/// to compare or display.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ConfigInfoData {
    /// Config file path on the daemon host.
    pub path: String,
    /// Content fingerprint: sha256 hex over the robot TOML and each
    /// gripper TOML (sorted by filename), each hashed as `filename\n`
    /// then content bytes.
    pub fingerprint: String,
    /// RT tick period \[s\].
    pub tick_dt_s: f64,
    /// The `[motion]` feel constants, in declaration order.
    pub motion: [f64; 8],
    /// Per-joint effective EXEC limits: `[soft_min_rad, soft_max_rad,
    /// velocity_rad_s, acceleration_rad_s2]`.
    pub joints: Vec<[f64; 4]>,
    /// Robot TOML file name (base name), served by CONFIG_BUNDLE.
    pub robot_filename: String,
    /// Robot TOML content verbatim, served by CONFIG_BUNDLE.
    pub robot_toml: String,
    /// Gripper TOMLs as `(file name, content)`, sorted by file name,
    /// served by CONFIG_BUNDLE.
    pub grippers: Vec<(String, String)>,
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
            probe_timeout: Duration::from_millis(200),
            rt_tick_rate_hz: 250.0,
            link_stale: Duration::from_millis(200),
            // Vendor parity: rcb-runtime's queue admits 256 before FULL.
            queue_capacity: 256,
            dedup_window: 256,
            chunk_timeout: Duration::from_secs(2),
            poll_interval: Duration::from_millis(2),
            // parol6's blend buffer holds up to PAROL6_MAX_BLEND_LOOKAHEAD
            // (100) commands and flushes after 100 ms of queue silence
            // (`server/motion_planner.py`); the same numbers here mean a
            // program written against parol6 blends the same way.
            blend_lookahead: 100,
            blend_hold: Duration::from_millis(100),
            simulator: false,
            tools: Vec::new(),
            fitted_tool: String::new(),
            tool_dof: 0,
            cartesian: true,
            digital_outputs: Vec::new(),
            tunable_nodes: Vec::new(),
            profiles: vec!["default".to_owned()],
            initial_profile: "default".to_owned(),
            joint_hard_limits_deg: [(f64::NEG_INFINITY, f64::INFINITY); NUM_JOINTS],
            installation_shapes: Vec::new(),
            config_info: ConfigInfoData::default(),
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
            ..Self::default()
        }
    }
}
