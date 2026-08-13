//! Runtime assembly: config load, thread spawn/wiring, clean shutdown.
//!
//! Threads owned by one [`Daemon`]:
//!
//! 1. **RT thread** — `RtCore<RuntimeBus>::run()` (absolute-deadline pacing;
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
use par6_bus::{RuntimeBus, SocketCanBus};
use par6_config::{ConfigBundle, ConfigError, LimitMode, TimingConfig};
use par6_motion::{JogEngine, MotionError, MotionLimits, StreamingExecutor};
#[cfg(not(feature = "ffi"))]
use par6_rt::NoFk;
use par6_rt::{
    sample_ring, snapshot_channel, CompletionPolicy, EstopGpio, FlashMarker, ForwardKin,
    GravityModel, RtCore, RtHooks, RunOptions, SharedLineGpio, SnapshotReader, SnapshotWriter,
    SpecSettle, StateSnapshot, ZeroGravity,
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

/// Why a par6d compiled without feature `ffi` does not boot. Kinematics
/// is load-bearing for the advertised surface, and its absence is
/// invisible from the client side, so it is a startup failure rather
/// than a degraded mode.
const NO_FFI_REFUSAL: &str = "this par6d was built without feature `ffi`, so it has no \
     kinematics: the TCP pose broadcast would read NaN, `move_l` / `move_j_pose` / `servo_l` / \
     `servo_j_pose` / `jog_l` would all be refused, the TOPPRA profile the registry advertises \
     would be unavailable, and the collision world would be empty while `set_shapes` still \
     reported success. Refusing to start rather than serve that. Build the runtime with \
     `--features ffi` (scripts/ffi/setup.sh, then `source .ffi/env.sh`); \
     see scripts/deploy/README.md for the aarch64 control box.";

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
    /// The kinematics stack could not start (missing assets tree, URDF
    /// load failure, or a build without feature `ffi`).
    #[error("kinematics: {0}")]
    Kinematics(String),
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
    ///
    /// Refuses to boot a binary built without feature `ffi`: that build
    /// has no kinematics at all, and every one of its degradations is
    /// silent to a client (NaN TCP pose, zero cartesian freedom, an
    /// empty collision world that still answers `set_shapes` with
    /// success). Kinematics is not an optional part of the runtime's
    /// advertised surface, so the build that lacks it does not run.
    pub fn start(opts: &Options) -> Result<Self, DaemonError> {
        if !cfg!(feature = "ffi") {
            return Err(DaemonError::Kinematics(NO_FFI_REFUSAL.into()));
        }
        Self::start_inner(opts)
    }

    fn start_inner(opts: &Options) -> Result<Self, DaemonError> {
        let config_path =
            resolve_config_path(opts.config.as_deref()).map_err(DaemonError::ConfigPath)?;
        let mut loaded = ConfigBundle::load(&config_path)?;
        loaded.robot.timing = Some(resolve_loop_bands(opts.sim, loaded.robot.timing));
        loaded.robot.stream.command_timeout_s =
            resolve_stream_timeout(opts.sim, loaded.robot.stream.command_timeout_s);
        let bundle = Arc::new(loaded);
        let robot = &bundle.robot;
        let bands = robot.loop_timing();
        log::info!(
            "loaded {} ({} joints, tick {} Hz) from {}",
            robot.robot.name,
            robot.joints.len(),
            robot.tick_rate_hz(),
            config_path.display()
        );
        log::info!(
            "loop bands: degraded > {:.2}x dt, critical > {:.2}x dt sustained {} s",
            bands.degraded_factor,
            bands.critical_factor,
            bands.critical_sustain_s
        );
        log::info!(
            "streaming watchdog: {:.3} s of client silence",
            robot.stream.command_timeout_s
        );

        if opts.sim_dynamics && !opts.sim {
            return Err(DaemonError::Hardware(
                "--sim-dynamics is a simulator plant; add --sim or drop it".into(),
            ));
        }
        #[cfg(not(feature = "ffi"))]
        if opts.sim_dynamics {
            return Err(DaemonError::Kinematics(
                "--sim-dynamics needs a par6d build with feature `ffi`".into(),
            ));
        }
        #[cfg(not(feature = "ffi"))]
        if opts.assets.is_some() {
            log::warn!("--assets has no effect: this par6d was built without feature `ffi`");
        }

        #[cfg(feature = "ffi")]
        let KinStack {
            fk: kin_fk,
            gravity: kin_gravity,
            planner: kin_planner,
            bridge: kin_bridge,
            housekeeping: kin_hk,
            collision,
            tool_offset,
            assets_dir,
        } = load_kin_stack(opts, &config_path, robot)?;

        let dt = robot.robot.tick_dt_s;
        let stream_limits = MotionLimits::from_config(robot, LimitMode::Stream)?;
        let jog = MotionJog::new(JogEngine::new(robot)?);
        let stream = MotionStream::new(
            StreamingExecutor::new(dt, &stream_limits)?,
            dt,
            stream_limits.soft_min,
            stream_limits.soft_max,
        );

        let (cmds_tx, cmds_rx) = mpsc::channel();
        let (ops_tx, ops_rx) = mpsc::channel::<CoreOp>();
        let shutdown = Arc::new(AtomicBool::new(false));
        let rt_break = Arc::new(AtomicBool::new(false));
        let (producer, consumer) = sample_ring(RING_CAPACITY);
        // The bridge's `halt` flushes the same ring the planner fills.
        let flush_marker = producer.flush_marker();

        // Hardware prerequisites, in the order an operator fixes them:
        // the CAN interface, then the e-stop line. Both are startup
        // refusals — nothing has been spawned yet.
        #[cfg(feature = "ffi")]
        let sim_bus = if opts.sim_dynamics {
            // The torque-level plant models the ARM (its URDF must carry
            // exactly the configured joint count, so the bare-flange
            // model is the only fit); the active tool's mass rides the
            // gravity model, not the plant.
            let urdf = assets_dir.join(par6_kin::GripperVariant::Flange.urdf_relpath());
            if !urdf.is_file() {
                return Err(DaemonError::Kinematics(format!(
                    "sim-dynamics URDF missing: {}",
                    urdf.display()
                )));
            }
            log::info!("sim plant: torque-level dynamics ({})", urdf.display());
            SimBus::with_dynamics(urdf)
        } else {
            SimBus::new()
        };
        #[cfg(not(feature = "ffi"))]
        let sim_bus = SimBus::new();
        let bus = if opts.sim {
            RuntimeBus::from(sim_bus)
        } else {
            RuntimeBus::from(open_hardware_bus(&robot.bus)?)
        };
        let estop = estop_source(opts)?;

        // Gravity and TCP FK: Kin-backed (G(q) + real TCP pose) with
        // feature `ffi`, the built-in defaults (ZeroGravity, NoFk =
        // all-NaN TCP pose) otherwise.
        //
        // G(q) is a feedforward that cancels the arm's OWN weight, so it
        // belongs only where that weight exists: the torque-level plant
        // (and, later, hardware). The kinematic plant integrates
        // commanded current directly and models no gravity, so feeding
        // it G(q) would accelerate an IDLE arm off its pose.
        #[cfg(feature = "ffi")]
        let gravity_hook: Box<dyn GravityModel> = if opts.sim && !opts.sim_dynamics {
            Box::new(ZeroGravity)
        } else {
            Box::new(kin_gravity)
        };
        #[cfg(feature = "ffi")]
        let fk_hook: Box<dyn ForwardKin> = Box::new(kin_fk);
        #[cfg(not(feature = "ffi"))]
        let (gravity_hook, fk_hook): (Box<dyn GravityModel>, Box<dyn ForwardKin>) =
            (Box::new(ZeroGravity), Box::new(NoFk));
        let hooks = RtHooks {
            gravity: gravity_hook,
            jog: Box::new(jog),
            stream: Box::new(stream),
            settle: Box::new(SpecSettle::new(CompletionPolicy::Settled, dt)),
            estop,
            flash: flash_marker(),
            commands: Box::new(cmds_rx),
            fk: fk_hook,
            samples: consumer,
        };
        let (core, handles) = RtCore::new(&bundle, bus, hooks)?;

        // The RT snapshot channel is single-reader; the tee fans it out.
        let (srv_w, srv_r) = snapshot_channel::<StateSnapshot>();
        let (plan_w, plan_r) = snapshot_channel::<StateSnapshot>();
        let (hk_w, hk_r) = snapshot_channel::<StateSnapshot>();
        // The bridge only needs its own tap with feature `ffi` (seeding
        // cartesian streams from the measured pose).
        #[cfg_attr(not(feature = "ffi"), allow(unused_mut))]
        let mut tee_writers = vec![srv_w, plan_w, hk_w];
        #[cfg(feature = "ffi")]
        let bridge_snapshots = {
            let (br_w, br_r) = snapshot_channel::<StateSnapshot>();
            tee_writers.push(br_w);
            br_r
        };

        let link = CoreLink::new(cmds_tx, ops_tx, rt_break.clone());
        #[cfg(feature = "ffi")]
        let planner = Par6Planner::new(
            link.clone(),
            producer,
            handles.heartbeat.clone(),
            plan_r,
            &bundle,
            crate::planner::PlannerKin {
                kin: kin_planner,
                collision,
                tool_offset,
            },
        )?;
        #[cfg(not(feature = "ffi"))]
        let planner = Par6Planner::new(
            link.clone(),
            producer,
            handles.heartbeat.clone(),
            plan_r,
            &bundle,
        )?;
        let stream_input = Arc::new(Mutex::new(handles.stream));
        let shared = Arc::new(Mutex::new(SharedState::default()));
        #[cfg(feature = "ffi")]
        let bridge = RtBridge::new(
            link.clone(),
            stream_input.clone(),
            shared.clone(),
            flush_marker,
            bundle.clone(),
            opts.sim,
            crate::bridge::CartStream {
                kin: kin_bridge,
                snapshots: bridge_snapshots,
                soft_min: stream_limits.soft_min,
                soft_max: stream_limits.soft_max,
            },
        );
        #[cfg(not(feature = "ffi"))]
        let bridge = RtBridge::new(
            link.clone(),
            stream_input.clone(),
            shared.clone(),
            flush_marker,
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
        let backend = if opts.sim { "sim" } else { "SocketCAN" };
        log::info!("command plane on {command_addr} ({backend} backend)");

        let run_opts = if opts.sim {
            // Sim runs unprivileged and in CI: no pinning, no SCHED_FIFO.
            RunOptions {
                cpu: None,
                fifo_priority: None,
            }
        } else {
            // Hardware: SCHED_FIFO on the isolated core (spec/RT.md
            // scheduling; setup failure is logged DEGRADED, not fatal).
            RunOptions::default()
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
                    .spawn(move || tee_loop(rt_reader, tee_writers, shutdown))?,
            );
        }
        {
            let (link, shutdown) = (link, shutdown.clone());
            threads.push(
                std::thread::Builder::new()
                    .name("par6d-housekeeping".into())
                    .spawn(move || {
                        #[cfg(feature = "ffi")]
                        housekeeping_loop(link, stream_input, shared, hk_r, shutdown, kin_hk);
                        #[cfg(not(feature = "ffi"))]
                        housekeeping_loop(link, stream_input, shared, hk_r, shutdown);
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
    mut core: RtCore<RuntimeBus>,
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

/// The loop-period bands the RT core runs under.
///
/// The simulator paces itself off the wall clock on whatever host it is
/// given, where the vendor bands (sized for a dedicated PREEMPT_RT box)
/// turn ordinary scheduler starvation into a latched `LOOP_CRITICAL` that
/// no robot fault caused. `--sim` therefore falls back to the wider
/// [`TimingConfig::SIM`] bands — but only as a default: a config that
/// declares `[timing]` always wins, on hardware and in sim alike, so the
/// guard can still be tightened for a deliberate test.
fn resolve_loop_bands(sim: bool, declared: Option<TimingConfig>) -> TimingConfig {
    match declared {
        Some(t) => t,
        None if sim => TimingConfig::SIM,
        None => TimingConfig::default(),
    }
}

/// The streaming watchdog window this runtime runs under \[s\].
///
/// The watchdog stops a real arm when the PC streaming to it goes quiet,
/// and the configured 40 ms is right for an arm whose RT thread owns an
/// isolated core. Under `--sim` the same loop paces itself off the wall
/// clock on whatever host it is given, and the keep-alive feeding the
/// watchdog is an ordinary thread: one scheduler hiccup then latches
/// `RTI_LINK_LOST` that no client fault caused, and takes the controller
/// DISABLED with it. The sim floor is sized past the bridge's own servo
/// grace period, so the property [`crate::bridge`] states — housekeeping
/// ends a silent stream before the RT watchdog fires — actually holds
/// there. A config asking for a LONGER window always wins, in sim and on
/// hardware alike.
fn resolve_stream_timeout(sim: bool, declared: f64) -> f64 {
    if sim {
        declared.max(2.0 * crate::bridge::SERVO_GRACE.as_secs_f64())
    } else {
        declared
    }
}

fn server_config(opts: &Options, bundle: &ConfigBundle) -> ServerConfig {
    let robot = &bundle.robot;
    let mut cfg = ServerConfig::from_protocol(&robot.protocol);
    cfg.rt_tick_rate_hz = robot.tick_rate_hz();
    cfg.simulator = opts.sim;
    cfg.tools = bundle.grippers.iter().map(|g| g.name.clone()).collect();
    // The fitted tool is the one the kinematics, gravity model and bus
    // were built around at startup; a passive tool (no CAN driver) has no
    // controllable DOF.
    cfg.fitted_tool = robot.robot.active_gripper.clone();
    cfg.tool_dof = usize::from(bundle.active_gripper().is_some_and(|g| g.driver.is_some()));
    cfg.cartesian = cfg!(feature = "ffi");
    // The window `teleport` may place a joint in. Refusing outside it is
    // the server's job: the bridge is fire-and-forget and has no reply
    // channel to refuse on.
    for (slot, joint) in cfg.joint_hard_limits_deg.iter_mut().zip(&robot.joints) {
        *slot = (
            joint.limits.hard_min_rad.to_degrees(),
            joint.limits.hard_max_rad.to_degrees(),
        );
    }
    cfg.profiles = crate::planner::profile_names();
    cfg.initial_profile = crate::planner::DEFAULT_PROFILE.to_owned();
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

/// The kinematics models loaded at startup (feature `ffi`): one
/// [`par6_kin::Kin`] per consumer — pinocchio's `Data` is mutated by
/// every call, so instances are never shared across threads.
#[cfg(feature = "ffi")]
struct KinStack {
    fk: crate::kin::KinFk,
    gravity: crate::kin::KinGravity,
    planner: crate::kin::CartKin,
    bridge: crate::kin::CartKin,
    housekeeping: crate::kin::CartKin,
    collision: par6_kin::Collision,
    /// The one TCP-offset cell all of the above read.
    tool_offset: crate::kin::ToolOffset,
    assets_dir: std::path::PathBuf,
}

/// Standoff \[m\] every collision pair is checked with: geometry within
/// this distance counts as colliding, so the arm keeps a near-miss buffer
/// from itself and from keep-outs that absorbs model and calibration
/// error. The value parol6 runs the same arm with; a shape that wants a
/// wider berth carries its own `margin`.
#[cfg(feature = "ffi")]
const COLLISION_CLEARANCE_M: f64 = 0.005;

/// Resolve the assets tree and load every model instance. Any failure
/// (missing tree, bad URDF) is a clean startup error.
#[cfg(feature = "ffi")]
fn load_kin_stack(
    opts: &Options,
    config_path: &std::path::Path,
    robot: &par6_config::RobotConfig,
) -> Result<KinStack, DaemonError> {
    use crate::kin::{
        load_kin, resolve_assets_dir, variant_for, CartKin, KinFk, KinGravity, SoftWindow,
        ToolOffset,
    };
    let assets_dir =
        resolve_assets_dir(opts.assets.as_deref(), config_path).map_err(DaemonError::Kinematics)?;
    let variant = variant_for(&robot.robot.active_gripper);
    log::info!(
        "kinematics: {} from {}",
        variant.urdf_relpath(),
        assets_dir.display()
    );
    let load = || load_kin(&assets_dir, variant).map_err(DaemonError::Kinematics);
    // G(q) must describe the body that actually swings: the torque-level
    // sim plant is built from the arm-only URDF (the gripper variants
    // carry jaw joints the plant cannot take), so compensating a tool it
    // is not carrying would push an IDLE arm upward.
    let gravity_variant = if opts.sim_dynamics {
        par6_kin::GripperVariant::Flange
    } else {
        variant
    };
    // The collision world models the same body the planner plans for,
    // tool included — a keep-out the gripper enters is a collision even
    // when the flange clears it. Loaded once: the vendor collision meshes
    // cost hundreds of milliseconds to read.
    let collision = par6_kin::Collision::load(&assets_dir, variant, COLLISION_CLEARANCE_M)
        .map_err(|e| {
            DaemonError::Kinematics(format!(
                "cannot load collision model {} from {}: {e}",
                variant.urdf_relpath(),
                assets_dir.display()
            ))
        })?;
    // Gravity is the only model that does not carry it: the offset is a
    // massless point, not a load.
    let tool_offset = ToolOffset::new();
    let window = SoftWindow::from_config(robot);
    Ok(KinStack {
        fk: KinFk::new(load()?, tool_offset.clone()),
        gravity: KinGravity::new(
            load_kin(&assets_dir, gravity_variant).map_err(DaemonError::Kinematics)?,
        ),
        planner: CartKin::new(load()?, tool_offset.clone(), window),
        bridge: CartKin::new(load()?, tool_offset.clone(), window),
        housekeeping: CartKin::new(load()?, tool_offset.clone(), window),
        collision,
        tool_offset,
        assets_dir,
    })
}

/// The e-stop input this runtime reads once per tick.
///
/// Hardware gets the control box's physical ESTOP_1 line and refuses to
/// start without it. There is no degraded mode here: an always-released
/// stub is indistinguishable from a working line in every field the
/// runtime publishes — the ESTOP latch, the mode, the state and
/// `io()[4]` all read exactly as they do with an intact chain — so a
/// par6d that cannot read the button must not present as one that can.
///
/// `--sim` has no button. Its line sits released for the session and a
/// simulated stop goes through the software e-stop instead, which is a
/// separate key (`SW_ESTOP`) with the same reaction.
fn estop_source(opts: &Options) -> Result<Box<dyn EstopGpio>, DaemonError> {
    if opts.sim {
        let (gpio, _released) = SharedLineGpio::new(true);
        return Ok(Box::new(gpio));
    }
    par6_rt::gpio::open_estop1().map_err(|e| {
        DaemonError::Hardware(format!(
            "{e} — the physical e-stop must be readable before the arm moves; \
             run with --sim for the simulator"
        ))
    })
}

/// Whether firmware was flashed during a FLASHING window, consulted once
/// on the way out of it.
///
/// A flash reboots the driver and the encoder is absolute only within one
/// motor revolution, so the flashed joint's home reference dies with it:
/// the vendor consumes a marker file its flasher writes and clears homing
/// robot-wide. par6 ships no flasher — `spec/CAN.md` leaves the bootloader
/// protocol to the vendor tools — so nothing here can tell a flash from a
/// scan, and the only answer that is never unrecoverable is "yes". Every
/// FLASHING exit therefore costs a re-home; a marker-writing flasher is
/// what would buy the scan-only case back.
fn flash_marker() -> Box<dyn FlashMarker> {
    struct AssumeFlashed;
    impl FlashMarker for AssumeFlashed {
        fn flashed(&mut self) -> bool {
            true
        }
    }
    Box::new(AssumeFlashed)
}

/// Open the hardware bus, turning the backend's bring-up diagnosis into a
/// clean startup error (the operator needs to know which problem to fix:
/// missing interface, no `CAP_NET_ADMIN`, wrong bitrate).
fn open_hardware_bus(cfg: &par6_config::BusConfig) -> Result<SocketCanBus, DaemonError> {
    log::info!("bus backend: SocketCAN on '{}'", cfg.interface);
    SocketCanBus::open(cfg)
        .map_err(|e| DaemonError::Hardware(format!("{e} — run with --sim for the simulator")))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `--sim` supplies the relaxed bands only where the config is silent;
    /// a declared section is authoritative in both directions.
    #[test]
    fn sim_relaxes_the_loop_bands_but_never_overrides_a_declared_section() {
        assert_eq!(resolve_loop_bands(false, None), TimingConfig::default());
        assert_eq!(resolve_loop_bands(true, None), TimingConfig::SIM);
        assert!(TimingConfig::SIM.critical_factor > TimingConfig::default().critical_factor);

        // A config asking for a tight guard keeps it under --sim, so a
        // test can still prove the critical latch fires on the simulator.
        let tight = TimingConfig {
            degraded_factor: 1.01,
            critical_factor: 1.02,
            critical_sustain_s: 0.1,
        };
        assert_eq!(resolve_loop_bands(true, Some(tight)), tight);
        assert_eq!(resolve_loop_bands(false, Some(tight)), tight);
    }

    /// Hardware mode reads a physical line or does not start; `--sim`
    /// never wants one.
    ///
    /// The RT core's debounce → latch → `ACTIVE_ERROR` path is covered
    /// thoroughly elsewhere, over a shared-flag line the tests flip
    /// themselves — which says nothing about whether the shipped runtime
    /// reads anything at all, and that question has no answer from
    /// outside: an unread line publishes the same latch, the same mode
    /// and the same `io()[4]` as an intact chain. So the check has to sit
    /// here, at the selection, and the falling-back-to-a-stub branch has
    /// to not exist. Pressing the actual button is HIL step 2.
    #[test]
    fn hardware_needs_a_real_estop_line_and_sim_never_asks_for_one() {
        // A chip that cannot be one, so the outcome is the same on a bare
        // CI container and on the control box. No other test in this
        // binary reads the variable.
        std::env::set_var("PAR6_GPIO_CHIP", "/dev/par6-no-such-gpiochip");

        let gpio = estop_source(&Options {
            sim: true,
            ..Default::default()
        })
        .expect("sim has no button and must not need a chardev");
        let mut monitor = par6_rt::EstopMonitor::new(gpio);
        for _ in 0..(par6_rt::DEBOUNCE_READS * 2) {
            assert!(!monitor.pressed(), "the simulated line reads released");
        }

        let refusal = estop_source(&Options {
            sim: false,
            ..Default::default()
        });
        std::env::remove_var("PAR6_GPIO_CHIP");
        let Err(DaemonError::Hardware(msg)) = refusal else {
            panic!("hardware mode must refuse an unreadable ESTOP_1");
        };
        assert!(msg.contains("ESTOP_1"), "the refusal names the line: {msg}");
    }

    /// Every FLASHING exit invalidates homing.
    ///
    /// The wiring this pins was a dropped write handle: a marker nothing
    /// could ever set answered "no flash happened" for the life of the
    /// process, so `RtCore::leave_mode` kept a home reference that the
    /// driver reboot had already destroyed.
    #[test]
    fn the_flash_marker_reports_a_flash_on_every_flashing_exit() {
        let mut marker = flash_marker();
        assert!(marker.flashed(), "par6d cannot tell a flash from a scan");
        assert!(
            marker.flashed(),
            "consulted once per window — a second window must invalidate too"
        );
    }
}
