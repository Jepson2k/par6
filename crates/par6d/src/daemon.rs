//! Runtime assembly: config load, thread spawn/wiring, clean shutdown.
//!
//! Threads owned by one [`Daemon`]:
//!
//! 1. **RT thread** — `RtCore<SimBus>::run()` (absolute-deadline pacing;
//!    SCHED_FIFO/pinning skipped in sim mode). Between `run()` sessions
//!    it applies queued [`CoreOp`](crate::bridge) closures (teleport
//!    re-seed, settle-policy swap, loop-stats reset).
//! 2. **Snapshot tee** — fans the single RT snapshot reader out to the
//!    server, the planner, and housekeeping (the snapshot channel is
//!    single-reader by construction).
//! 3. **Housekeeping** — jog/servo watchdogs and enable retries
//!    (see [`crate::bridge`]).
//! 4. **Tokio runtime** — the `par6-server` command-plane task.
//!
//! Shutdown (SIGINT/SIGTERM → [`Daemon::shutdown`]): notify the server
//! task and give it a grace period, then flag the worker threads and
//! join them all, then drain the tokio runtime. No thread is aborted.

use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

use par6_bus::sim::SimBus;
use par6_config::{ConfigBundle, ConfigError, LimitMode};
use par6_motion::{JogEngine, MotionError, MotionLimits, StreamingExecutor};
use par6_rt::{
    sample_ring, snapshot_channel, CompletionPolicy, NoFk, RtCore, RtHooks, RunOptions,
    SharedFlashMarker, SharedLineGpio, SnapshotReader, SnapshotWriter, SpecSettle, StateSnapshot,
    ZeroGravity,
};
use par6_server::{ServerConfig, ServerHandle};

use crate::adapters::{MotionJog, MotionStream};
use crate::bridge::{housekeeping_loop, CoreLink, CoreOp, RtBridge, SharedState};
use crate::options::{resolve_config_path, Options};
use crate::planner::Par6Planner;

/// Planner→RT sample ring capacity \[samples\] (~16 s at 4 ms; longer
/// plans stream in under backpressure from the planner's poll loop).
const RING_CAPACITY: usize = 4096;
/// Grace period for the server task to exit after the shutdown notify.
const SERVER_GRACE: Duration = Duration::from_millis(100);

/// Startup failure. Every variant renders as a one-line, actionable
/// message — a missing CAN interface or config file must never panic.
#[derive(Debug, thiserror::Error)]
pub enum DaemonError {
    /// The robot TOML could not be located.
    #[error("{0}")]
    ConfigPath(String),
    /// The robot TOML failed to load or validate.
    #[error("config: {0}")]
    Config(#[from] ConfigError),
    /// Hardware mode is unavailable (missing interface or backend).
    #[error("{0}")]
    Hardware(String),
    /// The RT core could not be constructed.
    #[error("RT core: {0}")]
    Core(#[from] par6_rt::CoreError),
    /// The motion engines could not be constructed from the config.
    #[error("motion: {0}")]
    Motion(#[from] MotionError),
    /// The command plane could not bind or start.
    #[error("command plane: {0}")]
    Io(#[from] std::io::Error),
}

/// A running par6d instance (all threads + the command-plane server).
pub struct Daemon {
    command_addr: SocketAddr,
    status_port: u16,
    telemetry_port: u16,
    server: Option<ServerHandle>,
    runtime: Option<tokio::runtime::Runtime>,
    shutdown: Arc<AtomicBool>,
    rt_break: Arc<AtomicBool>,
    threads: Vec<JoinHandle<()>>,
}

impl Daemon {
    /// Bound command-socket address (the ephemeral port lives here when
    /// started with `--port 0`).
    pub fn command_addr(&self) -> SocketAddr {
        self.command_addr
    }

    /// Status broadcast destination port.
    pub fn status_port(&self) -> u16 {
        self.status_port
    }

    /// Telemetry stream destination port.
    pub fn telemetry_port(&self) -> u16 {
        self.telemetry_port
    }

    /// Boot the full runtime: load config, build the RT core over the
    /// selected bus backend, wire the planner and bridge, and spawn the
    /// command plane.
    pub fn start(opts: &Options) -> Result<Self, DaemonError> {
        let config_path =
            resolve_config_path(opts.config.as_deref()).map_err(DaemonError::ConfigPath)?;
        let bundle = Arc::new(ConfigBundle::load(&config_path)?);
        let robot = &bundle.robot;
        log::info!(
            "loaded {} ({} joints, tick {} Hz) from {}",
            robot.robot.name,
            robot.joints.len(),
            robot.tick_rate_hz(),
            config_path.display()
        );

        if !opts.sim {
            return Err(hardware_unavailable(&robot.bus.interface));
        }

        let dt = robot.robot.tick_dt_s;
        let stream_limits = MotionLimits::from_config(robot, LimitMode::Stream)?;
        let jog = MotionJog::new(JogEngine::new(robot)?);
        let stream = MotionStream::new(
            StreamingExecutor::new(dt, &stream_limits)?,
            stream_limits.soft_min,
            stream_limits.soft_max,
        );

        let (cmds_tx, cmds_rx) = mpsc::channel();
        let (ops_tx, ops_rx) = mpsc::channel::<CoreOp>();
        let shutdown = Arc::new(AtomicBool::new(false));
        let rt_break = Arc::new(AtomicBool::new(false));
        let (producer, consumer) = sample_ring(RING_CAPACITY);
        let (gpio, _estop_line) = SharedLineGpio::new(true);
        let (flash, _flash_flag) = SharedFlashMarker::new();

        // Gravity and TCP FK run the built-in defaults: ZeroGravity and
        // NoFk (all-NaN TCP pose). Follow-up: adapt the pinokin gravity
        // model and par6-kin FK here once the kinematics workstream
        // lands its adapter — do NOT wire pinokin from this crate yet.
        let hooks = RtHooks {
            gravity: Box::new(ZeroGravity),
            jog: Box::new(jog),
            stream: Box::new(stream),
            settle: Box::new(SpecSettle::new(CompletionPolicy::Settled, dt)),
            estop: Box::new(gpio),
            flash: Box::new(flash),
            commands: Box::new(cmds_rx),
            fk: Box::new(NoFk),
            samples: consumer,
        };
        let (core, handles) = RtCore::new(&bundle, SimBus::new(), hooks)?;

        // The RT snapshot channel is single-reader; the tee fans it out.
        let (srv_w, srv_r) = snapshot_channel::<StateSnapshot>();
        let (plan_w, plan_r) = snapshot_channel::<StateSnapshot>();
        let (hk_w, hk_r) = snapshot_channel::<StateSnapshot>();

        let link = CoreLink::new(cmds_tx, ops_tx, rt_break.clone());
        let planner = Par6Planner::new(
            link.clone(),
            producer,
            handles.heartbeat.clone(),
            plan_r,
            &bundle,
        )?;
        let stream_input = Arc::new(Mutex::new(handles.stream));
        let shared = Arc::new(Mutex::new(SharedState::default()));
        let bridge = RtBridge::new(
            link.clone(),
            stream_input.clone(),
            shared.clone(),
            bundle.clone(),
            opts.sim,
        );

        let cfg = server_config(opts, &bundle);
        let (status_port, telemetry_port) = (cfg.status_port, cfg.telemetry_port);
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .enable_all()
            .build()?;
        let server = runtime.block_on(par6_server::spawn(
            cfg,
            par6_server::RuntimeHandle {
                planner,
                rt: bridge,
                snapshots: srv_r,
            },
        ))?;
        let command_addr = server.addr;
        log::info!("command plane on {command_addr} (sim backend)");

        let run_opts = RunOptions {
            // Sim runs unprivileged and in CI: no pinning, no SCHED_FIFO.
            cpu: None,
            fifo_priority: None,
        };
        let mut threads = Vec::new();
        {
            let (rt_break, shutdown) = (rt_break.clone(), shutdown.clone());
            threads.push(
                std::thread::Builder::new()
                    .name("par6d-rt".into())
                    .spawn(move || rt_loop(core, ops_rx, rt_break, shutdown, run_opts))?,
            );
        }
        {
            let shutdown = shutdown.clone();
            let rt_reader = handles.snapshots;
            threads.push(
                std::thread::Builder::new()
                    .name("par6d-tee".into())
                    .spawn(move || tee_loop(rt_reader, vec![srv_w, plan_w, hk_w], shutdown))?,
            );
        }
        {
            let (link, shutdown) = (link, shutdown.clone());
            threads.push(
                std::thread::Builder::new()
                    .name("par6d-housekeeping".into())
                    .spawn(move || {
                        housekeeping_loop(link, stream_input, shared, hk_r, dt, shutdown)
                    })?,
            );
        }

        Ok(Self {
            command_addr,
            status_port,
            telemetry_port,
            server: Some(server),
            runtime: Some(runtime),
            shutdown,
            rt_break,
            threads,
        })
    }

    /// Stop everything: server task first, then the worker threads (all
    /// joined), then the tokio runtime.
    pub fn shutdown(mut self) {
        self.stop();
    }

    fn stop(&mut self) {
        if let Some(server) = self.server.take() {
            server.shutdown();
            if let Some(rt) = &self.runtime {
                // Built inside block_on: the sleep needs the reactor.
                rt.block_on(async { tokio::time::sleep(SERVER_GRACE).await });
            }
            drop(server);
        }
        self.shutdown.store(true, Ordering::SeqCst);
        self.rt_break.store(true, Ordering::SeqCst);
        for t in self.threads.drain(..) {
            if t.join().is_err() {
                log::error!("worker thread panicked during shutdown");
            }
        }
        if let Some(rt) = self.runtime.take() {
            rt.shutdown_timeout(Duration::from_secs(1));
        }
    }
}

impl Drop for Daemon {
    fn drop(&mut self) {
        self.stop();
    }
}

/// The RT thread body: `run()` until an op or shutdown breaks the loop,
/// apply pending ops with `&mut RtCore`, repeat.
fn rt_loop(
    mut core: RtCore<SimBus>,
    ops: mpsc::Receiver<CoreOp>,
    rt_break: Arc<AtomicBool>,
    shutdown: Arc<AtomicBool>,
    run_opts: RunOptions,
) {
    loop {
        if shutdown.load(Ordering::SeqCst) {
            break;
        }
        rt_break.store(false, Ordering::SeqCst);
        while let Ok(op) = ops.try_recv() {
            op(&mut core);
        }
        // An op that raced the drain re-set the flag; drain again before
        // re-entering the pacing loop.
        if rt_break.load(Ordering::SeqCst) || shutdown.load(Ordering::SeqCst) {
            continue;
        }
        core.run(&run_opts, &rt_break);
    }
    log::info!("RT thread stopped");
}

/// Fan one snapshot stream out to N single-reader channels.
fn tee_loop(
    mut reader: SnapshotReader<StateSnapshot>,
    mut writers: Vec<SnapshotWriter<StateSnapshot>>,
    shutdown: Arc<AtomicBool>,
) {
    while !shutdown.load(Ordering::SeqCst) {
        match reader.take() {
            Some(s) => {
                for w in &mut writers {
                    w.publish(&s);
                }
            }
            None => std::thread::sleep(Duration::from_millis(1)),
        }
    }
}

fn server_config(opts: &Options, bundle: &ConfigBundle) -> ServerConfig {
    let robot = &bundle.robot;
    let mut cfg = ServerConfig::from_protocol(&robot.protocol);
    cfg.rt_tick_rate_hz = robot.tick_rate_hz();
    cfg.simulator = opts.sim;
    cfg.tools = bundle.grippers.iter().map(|g| g.name.clone()).collect();
    if let Some(ip) = opts.bind {
        cfg.bind.set_ip(ip);
    }
    if let Some(port) = opts.command_port {
        cfg.bind.set_port(port);
    }
    if let Some(host) = opts.status_host {
        cfg.status_dest_host = host;
    }
    if let Some(port) = opts.status_port {
        cfg.status_port = port;
    }
    if let Some(port) = opts.telemetry_port {
        cfg.telemetry_port = port;
    }
    if let Some(t) = opts.status_transport {
        cfg.status_transport = t;
    }
    cfg
}

/// Hardware mode diagnosis: distinguish "no such interface" from "the
/// backend is missing" so the operator knows which problem to fix.
fn hardware_unavailable(iface: &str) -> DaemonError {
    use socketcan::Socket;
    match socketcan::CanSocket::open(iface) {
        Ok(_) => DaemonError::Hardware(format!(
            "CAN interface '{iface}' is present, but the SocketCAN DriverBus backend \
             has not landed in par6-bus yet; run with --sim"
        )),
        Err(e) => DaemonError::Hardware(format!(
            "CAN interface '{iface}' is not available ({e}); hardware mode needs a \
             configured SocketCAN interface — run with --sim for the simulator"
        )),
    }
}
