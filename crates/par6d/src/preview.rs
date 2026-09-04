//! Offline dry run: the daemon's OWN planner, server rules and streaming
//! integrator, driven against a virtual arm instead of a running RT
//! core. A submitted command is validated, gated, held for blending,
//! planned and collision-checked by exactly the code that would drive
//! the arm — then discarded instead of dispatched — so a preview can
//! never drift from the runtime. Nothing here re-implements a rule;
//! every refusal is the server's own text.

use std::path::{Path, PathBuf};
use std::sync::mpsc;
use std::sync::{atomic::AtomicBool, Arc};

use par6_kin::NQ;
use par6_motion::{JogEngine, MotionLimits};
use par6_proto::command::{self as cmd, JogJ, ToolParam};
use par6_proto::{
    command_class, make_error, Command, CommandClass, CompletionPolicy, ErrorCode, WireError,
    NUM_JOINTS, UNATTRIBUTED,
};
use par6_rt::{
    hooks::StreamTracker, sample_ring, snapshot_channel, ExecHeartbeat, JogEngine as RtJogEngine,
    Mode, SampleConsumer, SnapshotWriter, StateSnapshot, MAX_JOINTS,
};
use par6_server::{
    check_gate, cmd_name, decode_error_to_wire, pid_gains_fault, session, validate_registries,
    validate_supported, write_io_fault, GateContext, PayloadSpec, PlanContext, Planner,
    QueuedCommand, ServerConfig, ShapeLayer,
};

use crate::adapters::{MotionJog, MotionStream};
use crate::bridge::{
    step_cart_jog, CartJogState, CoreLink, CoreOp, StreamGate, HOUSEKEEPING_PERIOD,
    STREAM_LOOKAHEAD_S,
};
use crate::daemon::{load_preview_kin, DaemonError};
use crate::kin::CartKin;
use crate::options::{resolve_config_path, Options};
use crate::planner::{profile_names, Par6Planner, PlannedMotion, PlannerKin};

/// One submitted command's outcome: the trajectory the runtime would
/// drive, the exact refusal it would answer with, or `pending` while
/// the command sits in the blend hold waiting for its successor.
#[derive(Debug, Clone)]
pub struct PreviewResult {
    /// Sampled joint trajectory \[rad\] at tick dt (empty for a command
    /// that moves nothing, or a refused one).
    pub joint_trajectory_rad: Vec<[f64; MAX_JOINTS]>,
    /// FK pose (flattened row-major 4×4, translation in metres) per
    /// trajectory sample.
    pub tcp_poses: Vec<[f64; 16]>,
    /// Where the arm ends \[rad\].
    pub end_joints_rad: [f64; MAX_JOINTS],
    /// Trajectory duration \[s\].
    pub duration_s: f64,
    /// The runtime's refusal, when the command would be refused.
    pub error: Option<WireError>,
    /// The command is queued behind the blend hold: nothing has been
    /// planned yet, and its motion arrives with the command that closes
    /// the chain (or with [`Preview::flush`]).
    pub pending: bool,
}

impl PreviewResult {
    /// Whether the command would be accepted.
    pub fn valid(&self) -> bool {
        self.error.is_none()
    }

    fn still(q: [f64; MAX_JOINTS]) -> Self {
        Self {
            joint_trajectory_rad: Vec::new(),
            tcp_poses: Vec::new(),
            end_joints_rad: q,
            duration_s: 0.0,
            error: None,
            pending: false,
        }
    }

    fn refusal(q: [f64; MAX_JOINTS], error: WireError) -> Self {
        Self {
            error: Some(error),
            ..Self::still(q)
        }
    }

    fn pending(q: [f64; MAX_JOINTS]) -> Self {
        Self {
            pending: true,
            ..Self::still(q)
        }
    }

    /// Several results as one, in the order they run: trajectories and
    /// poses concatenated, durations summed, the end pose the last one's.
    /// The first refusal ends the motion and is carried as the error —
    /// what a program sees when a held chain closes and the runtime
    /// refuses one of its legs.
    pub fn concat(results: Vec<PreviewResult>) -> Option<PreviewResult> {
        let mut out: Option<PreviewResult> = None;
        for r in results {
            let acc = out.get_or_insert_with(|| PreviewResult::still(r.end_joints_rad));
            acc.joint_trajectory_rad.extend(r.joint_trajectory_rad);
            acc.tcp_poses.extend(r.tcp_poses);
            acc.duration_s += r.duration_s;
            acc.end_joints_rad = r.end_joints_rad;
            if r.error.is_some() {
                acc.error = r.error;
                break;
            }
        }
        out
    }
}

/// What the runtime's streaming limiter would command, tick by tick,
/// for a sequence of servo targets — the offline half of a bring-up
/// limiter check, run through the same jerk-limited executor and
/// soft-limit clamp the RT ticks.
#[derive(Debug, Clone)]
pub struct ServoPreview {
    /// Commanded joint positions per tick \[rad\] (post-clamp).
    pub q: Vec<[f64; MAX_JOINTS]>,
    /// Commanded joint velocities per tick \[rad/s\].
    pub qd: Vec<[f64; MAX_JOINTS]>,
    /// The tick the limiter first reported the LAST target reached, if
    /// it did inside the window.
    pub finished_tick: Option<usize>,
}

/// The offline session: a virtual arm plus the runtime's planner,
/// server-side validation and state (profile, TCP offset, completion
/// policy, IO levels, tool state) — everything a program can observe.
pub struct Preview {
    planner: Par6Planner,
    jog: MotionJog,
    /// The housekeeping loop's own cartesian solver, so a `jog_l` preview
    /// integrates through the identical damped jacobian.
    cart: CartKin,
    soft_min: [f64; MAX_JOINTS],
    soft_max: [f64; MAX_JOINTS],
    snap: StateSnapshot,
    snap_w: SnapshotWriter<StateSnapshot>,
    next_index: u64,
    dt: f64,
    motion: par6_config::MotionConfig,
    /// The server config the live daemon would run with this bundle —
    /// what every server-side refusal here is checked against.
    cfg: ServerConfig,
    /// Where the configured homing seek leaves the arm.
    ready_pose: [f64; MAX_JOINTS],
    /// Queued moves waiting for the successor they blend into.
    held: session::BlendQueue<Command>,
    profile: String,
    tool: String,
    tool_variant: Option<String>,
    tcp_offset_mm: [f64; 3],
    policy: CompletionPolicy,
    payload: PayloadSpec,
    io_levels: Vec<u8>,
    io_inputs: usize,
    /// Commanded jaw position 0 = open … 1 = closed.
    tool_position: f64,
    flashing: bool,
    /// Whether the simulator backend is selected: `teleport` is gated on
    /// it, exactly as the server gates it.
    simulator: bool,
    /// The e-stop latch and the standing error, with the server's own
    /// transitions for a stop, an e-stop, a reset and an accepted motion.
    latches: session::Latches,
    /// Whether a jog stream is open. The RT ramps from rest on JOG mode
    /// ENTRY, not per datagram, so a stream of jogs must keep ramping
    /// rather than restart each time.
    jog_streaming: bool,
    /// The configured 0-to-full ramp time, and the fraction the open
    /// stream asked for: together they bound how long the ramp down can
    /// take when the stream ends.
    jog_accel_time_s: f64,
    jog_accel_scale: f64,
    /// Whether a cartesian jog stream is open: the RT enters STREAM once
    /// per session and its executor carries velocity across datagrams,
    /// so a preview must not restart the executor from rest per frame.
    cart_streaming: bool,
    /// The RT's own streaming executor. A cartesian jog is integrated
    /// into joint setpoints and then TRACKED by this, so a preview that
    /// stopped at the setpoints would report a jog that starts and stops
    /// instantly and would have nothing for `accel` to scale.
    stream: MotionStream,
    /// The streaming collision gate, the same one the housekeeping loop
    /// runs: a jog is admitted only if its projected lookahead clears.
    gate: StreamGate,
    // Keep the stub channel/ring ends alive so the planner's control
    // sends stay silent no-ops instead of logged errors.
    _cmds_rx: mpsc::Receiver<par6_rt::RtCommand>,
    _ops_rx: mpsc::Receiver<CoreOp>,
    _ring: SampleConsumer,
}

impl Preview {
    /// Build a session from the robot config (default search when
    /// `None`), assets tree and `package://` search dir, starting
    /// referenced at the park pose
    /// with the runtime's startup context.
    pub fn new(
        config: Option<&Path>,
        assets: Option<&Path>,
        package_dir: Option<&Path>,
    ) -> Result<Self, DaemonError> {
        let opts = Options {
            sim: true,
            config: config.map(Path::to_path_buf),
            assets: assets.map(Path::to_path_buf),
            package_dir: package_dir.map(Path::to_path_buf),
            ..Options::default()
        };
        let config_path =
            resolve_config_path(opts.config.as_deref()).map_err(DaemonError::ConfigPath)?;
        let bundle = par6_config::ConfigBundle::load(&config_path)?;
        let robot = &bundle.robot;
        let stack = load_preview_kin(&opts, &config_path, robot, bundle.active_gripper())?;

        let (cmds_tx, cmds_rx) = mpsc::channel();
        let (ops_tx, ops_rx) = mpsc::channel();
        let link = CoreLink::new(cmds_tx, ops_tx, Arc::new(AtomicBool::new(false)));
        let (producer, ring) = sample_ring(64);
        let (snap_w, snap_r) = snapshot_channel::<StateSnapshot>();
        let planner = Par6Planner::new(
            link,
            producer,
            ExecHeartbeat::unmonitored(),
            snap_r,
            &bundle,
            PlannerKin {
                kin: stack.planner,
                collision: stack.collision,
                tool_offset: stack.tool_offset,
            },
        )?;

        let mut snap = StateSnapshot::default();
        for (out, rad) in snap.q.iter_mut().zip(robot.robot.park_pose_rad.iter()) {
            *out = *rad;
        }
        snap.homed = true;
        snap.mode = Mode::Idle;
        let ready = robot
            .homing
            .ready_pose_rad(robot.joints.len())
            .map_err(DaemonError::ConfigPath)?;
        let mut ready_pose = snap.q;
        for (out, rad) in ready_pose.iter_mut().zip(ready.iter()) {
            *out = *rad;
        }
        let stream_limits = MotionLimits::from_config(robot, par6_config::LimitMode::Stream)?;
        let jog_limits = MotionLimits::from_config(robot, par6_config::LimitMode::Jog)?;
        let jog = MotionJog::new(JogEngine::new(robot)?, robot.jog.accel_time_s);
        let cfg = crate::daemon::server_config(&opts, &bundle);
        let mut preview = Self {
            planner,
            jog,
            cart: stack.cart,
            soft_min: stream_limits.soft_min,
            soft_max: stream_limits.soft_max,
            snap,
            snap_w,
            next_index: 0,
            dt: robot.robot.tick_dt_s,
            motion: robot.motion,
            ready_pose,
            held: session::BlendQueue::default(),
            profile: cfg.initial_profile.clone(),
            tool: cfg.fitted_tool.clone(),
            tool_variant: None,
            tcp_offset_mm: [0.0; 3],
            policy: CompletionPolicy::Settled,
            payload: PayloadSpec::default(),
            io_levels: vec![0; cfg.digital_outputs.len()],
            io_inputs: robot.io.inputs.len(),
            tool_position: 0.0,
            flashing: false,
            simulator: cfg.simulator,
            latches: session::Latches::default(),
            jog_streaming: false,
            jog_accel_time_s: robot.jog.accel_time_s,
            jog_accel_scale: 1.0,
            cart_streaming: false,
            stream: MotionStream::new(
                par6_motion::StreamingExecutor::new(robot.robot.tick_dt_s, &stream_limits)?,
                robot.robot.tick_dt_s,
                stream_limits,
                robot.stream.fault_latch_s,
            ),
            gate: StreamGate::new(stack.gate_collision, &jog_limits),
            cfg,
            _cmds_rx: cmds_rx,
            _ops_rx: ops_rx,
            _ring: ring,
        };
        let installation = preview.cfg.installation_shapes.clone();
        preview
            .planner
            .set_shapes(ShapeLayer::Installation, &installation)
            .map_err(|e| DaemonError::Kinematics(e.cause))?;
        // The streaming gate keeps its own world; the server mirrors
        // every accepted layer into it, and so must the preview or a
        // jog is admitted through a keep-out the planner refuses.
        preview
            .gate
            .set_layer(ShapeLayer::Installation, &installation)
            .map_err(|e| DaemonError::Kinematics(e.cause))?;
        preview.sync_planner();
        preview.publish();
        Ok(preview)
    }

    fn publish(&mut self) {
        self.snap_w.publish(&self.snap);
    }

    fn sync_planner(&mut self) {
        self.planner.sync(PlanContext {
            profile: &self.profile,
            tool: &self.tool,
            tool_variant: self.tool_variant.as_deref(),
            tcp_offset_mm: self.tcp_offset_mm,
            completion_policy: self.policy,
            payload: self.payload,
        });
    }

    // ------------------------------------------------------------ state

    /// The virtual arm pose \[rad\].
    pub fn angles_rad(&self) -> [f64; MAX_JOINTS] {
        self.snap.q
    }

    /// Whether the virtual arm holds its position references.
    pub fn homed(&self) -> bool {
        self.snap.homed
    }

    /// Set the virtual arm's homed state. While unhomed, commands the
    /// server gates on homing are refused with the server's own refusal,
    /// and HOME previews as the referencing seek instead of a planned
    /// park return.
    pub fn set_homed(&mut self, homed: bool) {
        self.snap.homed = homed;
        self.publish();
    }
    /// Move the virtual arm instantly — the preview's teleport, under
    /// the name the wire command carries.
    pub fn teleport_rad(&mut self, q: [f64; MAX_JOINTS]) {
        self.place_rad(q);
    }

    /// Move the virtual arm instantly without the wire's checks — the
    /// host seeding a session at the live arm's pose. Any open jog
    /// stream ends: an arm that was teleported is no longer the arm the
    /// stream was ramping, so the next jog starts from rest.
    pub fn place_rad(&mut self, q: [f64; MAX_JOINTS]) {
        self.end_jog_stream();
        self.cart_streaming = false;
        self.snap.q = q;
        self.publish();
    }

    /// FK at the virtual pose (flattened row-major 4×4, translation in
    /// metres — the engine's SI frame).
    pub fn pose(&mut self) -> Result<[f64; 16], WireError> {
        let q = self.snap.q;
        self.planner.current_pose(&q)
    }

    /// Registered motion profile names.
    pub fn profiles() -> Vec<String> {
        profile_names()
    }

    /// The active motion profile, in the registry's spelling.
    pub fn profile(&self) -> &str {
        &self.profile
    }

    /// The TCP offset \[mm\] on top of the tool transform.
    pub fn tcp_offset_mm(&self) -> [f64; 3] {
        self.tcp_offset_mm
    }

    /// The active tool key and variant.
    pub fn tool(&self) -> (&str, Option<&str>) {
        (&self.tool, self.tool_variant.as_deref())
    }

    /// Commanded jaw position, 0 = open … 1 = closed.
    pub fn tool_position(&self) -> f64 {
        self.tool_position
    }

    /// Whether the gripper has run its calibrate action.
    pub fn tool_calibrated(&self) -> bool {
        self.snap.gripper.reply.is_some_and(|r| r.calibrated)
    }

    /// `inputs ++ outputs ++ [estop]`, the STATUS layout: a preview reads
    /// no lines (every input low, the e-stop clear) and the outputs are
    /// whatever `write_io` last set.
    pub fn io(&self) -> Vec<u8> {
        let mut io = vec![0u8; self.io_inputs];
        io.extend_from_slice(&self.io_levels);
        io.push(1);
        io
    }

    /// Wire names of the commands waiting in the blend hold.
    pub fn held_names(&self) -> Vec<&'static str> {
        self.held.iter().map(|c| cmd_name(c.tag())).collect()
    }

    /// The effective `[motion]` feel constants the preview plans with.
    pub fn motion(&self) -> par6_config::MotionConfig {
        self.motion
    }

    /// The tick period \[s\] trajectories are sampled at.
    pub fn tick_dt_s(&self) -> f64 {
        self.dt
    }

    /// Where the configured homing seek leaves the arm \[rad\].
    pub fn homing_ready_pose_rad(&self) -> [f64; MAX_JOINTS] {
        self.ready_pose
    }

    /// The config path search used when `Preview::new` gets `None`.
    pub fn default_config_path() -> Result<PathBuf, String> {
        resolve_config_path(None)
    }

    // ---------------------------------------------------------- submit

    /// Submit one command exactly as the runtime would receive it: the
    /// wire's validation, the server's gates and registries, the blend
    /// hold, the planner, the collision world. Queued moves may come
    /// back `pending`; streamables run at once and drop the hold, as a
    /// streaming preemption does live; system commands change state and
    /// move nothing.
    pub fn submit(&mut self, command: Command) -> PreviewResult {
        let result = self.submit_inner(command);
        self.drain_stubs();
        result
    }

    /// The planner's RT-bound sends land in stub channels nothing reads;
    /// left alone they grow with the program.
    fn drain_stubs(&mut self) {
        self._cmds_rx.try_iter().for_each(drop);
        self._ops_rx.try_iter().for_each(drop);
    }

    fn submit_inner(&mut self, command: Command) -> PreviewResult {
        if let Err(e) = command.validate() {
            return self.refuse(decode_error_to_wire(&e));
        }
        if !matches!(command, Command::JogJ(_)) {
            self.end_jog_stream();
        }
        if !matches!(command, Command::JogL(_)) {
            self.cart_streaming = false;
        }
        match command_class(command.tag()) {
            CommandClass::Queued => self.submit_queued(command),
            CommandClass::FireAndForget => self.submit_stream(command),
            CommandClass::System => self.submit_system(command),
            CommandClass::Query => self.refuse(make_error(
                ErrorCode::CommValidationError,
                UNATTRIBUTED,
                &[(
                    "detail",
                    &format!("{:?} is a query, not a previewable command", command.tag()),
                )],
            )),
        }
    }

    /// Plan whatever the blend hold still holds, as the runtime's hold
    /// expiry would at the end of a program. `None` when nothing waits.
    pub fn flush(&mut self) -> Option<PreviewResult> {
        self.run_held()
    }

    fn refuse(&self, error: WireError) -> PreviewResult {
        PreviewResult::refusal(self.snap.q, error)
    }

    /// The server's gate table, through the server's own check. A
    /// FLASHING window is what the RT reports it as — disabled — rather
    /// than a refusal of the preview's own wording.
    fn check_gate(&self, command: &Command) -> Option<WireError> {
        check_gate(
            command.tag(),
            &GateContext {
                estop_latched: self.latches.estop_latched,
                enabled: !self.flashing,
                homed: self.snap.homed,
                simulator: self.simulator,
            },
        )
    }

    /// Preview a servo stream: each target in `targets` is held for
    /// `hold_ticks` ticks (the first setpoint sits at the virtual pose,
    /// as the RT's start-pose gate demands of a real stream), through the
    /// same jerk-limited executor, limit fractions and soft-limit clamp
    /// the RT core drives. The virtual arm does not move: this is a
    /// limiter measurement, not a motion.
    pub fn preview_servo(
        &mut self,
        targets: &[[f64; MAX_JOINTS]],
        hold_ticks: usize,
        speed: Option<f64>,
        accel: Option<f64>,
    ) -> ServoPreview {
        let hold = hold_ticks.max(1);
        self.stream
            .set_scale(speed.unwrap_or(1.0), accel.unwrap_or(1.0));
        self.stream.activate(&self.snap.q);
        let mut out = ServoPreview {
            q: Vec::with_capacity(targets.len() * hold),
            qd: Vec::with_capacity(targets.len() * hold),
            finished_tick: None,
        };
        let last = targets.len().saturating_sub(1);
        for (i, target) in targets.iter().enumerate() {
            self.stream.set_target(target);
            for _ in 0..hold {
                let mut q = [0.0; MAX_JOINTS];
                let mut qd = [0.0; MAX_JOINTS];
                self.stream.step(&mut q, &mut qd);
                if i == last && out.finished_tick.is_none() && self.stream.at_target() {
                    out.finished_tick = Some(out.q.len());
                }
                out.q.push(q);
                out.qd.push(qd);
            }
        }
        self.stream.set_scale(1.0, 1.0);
        out
    }

    /// Whether the virtual gripper holds a calibration: the runtime
    /// refuses a jaw move on an uncalibrated gripper, and a previewed
    /// `calibrate` action establishes one.
    pub fn set_gripper_calibrated(&mut self, calibrated: bool) {
        self.snap.gripper.reply = Some(par6_bus::GripperReply {
            calibrated,
            ..par6_bus::GripperReply::default()
        });
        self.snap.gripper.data_age_ticks = 0;
        self.publish();
    }

    /// The payload the virtual arm carries.
    pub fn payload(&self) -> PayloadSpec {
        self.payload
    }

    /// The motion a payload estimation makes from the virtual arm's
    /// pose: the wrist poses `par6_calibrate` would plan, swept at its
    /// protocol's speed and ending back where the arm stood, planned and
    /// gated like any other move. Measures nothing — a preview has no
    /// torque — so what comes back is the swing.
    pub fn preview_estimation(&mut self, spread: f64) -> Result<(usize, PreviewResult), String> {
        let mut start = [0.0; NQ];
        start.copy_from_slice(&self.snap.q[..NQ]);
        let window: [(f64, f64); NQ] =
            std::array::from_fn(|j| (self.soft_min[j], self.soft_max[j]));
        let protocol = par6_calibrate::Protocol::default();
        let poses = par6_calibrate::plan_poses(
            self.gate.collision_mut(),
            &start,
            &window,
            spread,
            protocol.approach_rad,
        )?;
        let to_deg = |q: &[f64; NQ]| -> [f64; par6_proto::NUM_JOINTS] {
            std::array::from_fn(|j| q[j].to_degrees())
        };
        let moves: Vec<Command> = poses
            .iter()
            .chain(std::iter::once(&start))
            .map(|q| {
                Command::MoveJ(cmd::MoveJ {
                    key: 0,
                    angles: to_deg(q),
                    duration: None,
                    speed: Some(protocol.speed),
                    accel: None,
                    blend_radius: None,
                    rel: false,
                })
            })
            .collect();
        let results = self.plan_batch(&moves);
        if let Some(refused) = results.iter().find(|r| r.error.is_some()) {
            return Err(refused
                .error
                .as_ref()
                .map(|e| e.cause.clone())
                .unwrap_or_default());
        }
        PreviewResult::concat(results)
            .map(|r| (poses.len(), r))
            .ok_or_else(|| "nothing to swing".to_owned())
    }

    /// The refusal the runtime would leave standing, or `None`.
    pub fn error(&self) -> Option<&WireError> {
        self.latches.standing_error.as_ref()
    }

    /// End an open jog stream the way the watchdog does: housekeeping
    /// sends `JogRelease`, which zeroes the engine's target fractions but
    /// not its velocity, so the ramp runs down over the following ticks.
    /// That is ground the arm actually covers, so the virtual arm covers
    /// it too and the next command starts from where the jog came to
    /// rest.
    fn end_jog_stream(&mut self) {
        if !self.jog_streaming {
            return;
        }
        self.jog.release();
        // The ramp runs at the scaled acceleration, and the s-curve
        // profile adds jerk phases on top of the linear time; four times
        // the scaled constant is beyond either. Reaching the cap means the
        // engine never reported rest, which is worth saying, because the
        // pose taken here is where every later preview starts from.
        let ramp_s = self.jog_accel_time_s / self.jog_accel_scale.max(f64::EPSILON);
        let cap = ((4.0 * ramp_s / self.dt).ceil() as usize).max(1);
        let mut q = self.snap.q;
        let mut at_rest = false;
        for _ in 0..cap {
            let mut q_out = [0.0; MAX_JOINTS];
            let mut qd_out = [0.0; MAX_JOINTS];
            self.jog.tick(&q, &mut q_out, &mut qd_out);
            q = q_out;
            if qd_out.iter().all(|v| *v == 0.0) {
                at_rest = true;
                break;
            }
        }
        if !at_rest {
            log::warn!(
                "preview jog ramp did not reach rest within {cap} ticks; the virtual arm is \
                 placed where the ramp was cut"
            );
        }
        self.snap.q = q;
        self.jog_streaming = false;
        self.publish();
    }

    fn submit_queued(&mut self, command: Command) -> PreviewResult {
        if let Some(error) = self
            .check_gate(&command)
            .or_else(|| validate_registries(&self.cfg, &command))
            .or_else(|| validate_supported(&self.cfg, &command))
        {
            return self.refuse(error);
        }
        self.latches.motion_accepted();
        self.held.push_back(command);
        if self.holding_for_blend() {
            return PreviewResult::pending(self.snap.q);
        }
        self.run_held()
            .unwrap_or_else(|| PreviewResult::still(self.snap.q))
    }

    /// The server's hold rule, with no expiry: offline there is no clock,
    /// so the chain closes with the next stopping command or
    /// [`Self::flush`].
    fn holding_for_blend(&mut self) -> bool {
        let lookahead = self.cfg.blend_lookahead;
        self.held.holding_for_blend(lookahead, None, |c| c)
    }

    fn run_held(&mut self) -> Option<PreviewResult> {
        if self.held.is_empty() {
            return None;
        }
        let batch: Vec<Command> = self.held.drain(..).collect();
        let results = self.plan_batch(&batch);
        PreviewResult::concat(results)
    }

    fn submit_stream(&mut self, command: Command) -> PreviewResult {
        if let Some(error) = self
            .check_gate(&command)
            .or_else(|| validate_supported(&self.cfg, &command))
        {
            return self.refuse(error);
        }
        // A streamable preempts planned motion, pending queue included.
        self.held.clear();
        match command {
            Command::Teleport(p) => {
                // The travel check already ran: `validate_supported` calls
                // `teleport_angle_fault` above.
                let mut q = self.snap.q;
                for (out, deg) in q.iter_mut().zip(p.angles.iter()) {
                    *out = deg.to_radians();
                }
                self.snap.q = q;
                self.snap.homed = true;
                if let Some(pos) = p.tool_positions.as_ref().and_then(|v| v.first()) {
                    self.tool_position = *pos;
                }
                self.publish();
                self.standing()
            }
            Command::JogJ(p) => self.preview_jog(p.speeds, p.duration, p.accel),
            Command::JogL(p) => self.preview_jog_l(p.velocities, p.frame, p.duration, p.accel),
            // A streamed target is tracked by the RT's own OTG at the
            // cadence targets arrive; offline there is no cadence, so the
            // settle onto the newest target is the planner's joint move.
            Command::ServoJ(p) => self.plan_one(Command::MoveJ(cmd::MoveJ {
                key: 0,
                angles: p.angles,
                duration: None,
                speed: Some(p.speed.unwrap_or(1.0)),
                accel: p.accel,
                blend_radius: None,
                rel: false,
            })),
            Command::ServoJPose(p) => self.settle_on_pose(p.pose, p.speed, p.accel),
            Command::ServoL(p) => self.settle_on_pose(p.pose, p.speed, p.accel),
            other => self.refuse(make_error(
                ErrorCode::CommValidationError,
                UNATTRIBUTED,
                &[("detail", &format!("{:?} cannot be previewed", other.tag()))],
            )),
        }
    }

    /// A streamed cartesian target settles as the planner's joint move
    /// onto it — the same rule for every servo family.
    fn settle_on_pose(
        &mut self,
        pose: [f64; 6],
        speed: Option<f64>,
        accel: Option<f64>,
    ) -> PreviewResult {
        self.plan_one(Command::MoveJPose(cmd::MoveJPose {
            key: 0,
            pose,
            duration: None,
            speed: Some(speed.unwrap_or(1.0)),
            accel,
            blend_radius: None,
        }))
    }

    fn submit_system(&mut self, command: Command) -> PreviewResult {
        let detail = |d: String| {
            make_error(
                ErrorCode::CommValidationError,
                UNATTRIBUTED,
                &[("detail", &d)],
            )
        };
        match command {
            Command::Stop(p) => {
                let cleared = p.clear_queue && !self.held.is_empty();
                if p.clear_queue {
                    self.held.clear();
                }
                self.latches.stop(cleared);
            }
            Command::Estop => {
                self.held.clear();
                self.latches.estop();
            }
            Command::Reset => self.latches.reset(),
            Command::Pause(_) | Command::SetGravityComp(_) => {}
            Command::ResetState => {
                self.held.clear();
                self.latches.reset();
                self.tool.clone_from(&self.cfg.fitted_tool);
                self.tool_variant = None;
                self.tcp_offset_mm = [0.0; 3];
                self.policy = CompletionPolicy::Settled;
                self.profile = self.cfg.initial_profile.clone();
                self.sync_planner();
                if let Err(e) = self.planner.set_shapes(ShapeLayer::Program, &[]) {
                    return self.refuse(e);
                }
                if let Err(e) = self.gate.set_layer(ShapeLayer::Program, &[]) {
                    return self.refuse(e);
                }
            }
            Command::WriteIo(p) => match write_io_fault(p.port, &self.cfg) {
                None => self.io_levels[usize::from(p.port)] = p.value,
                Some(e) => return self.refuse(e),
            },
            // A bus swap cancels every motion in flight (`Server`'s
            // `cancel_all_motion`), the held blend chain included — and
            // decides the references: the simulator is born referenced
            // (`swap_to_sim` sets homed), while real hardware knows
            // nothing until it seeks (`swap_to_hardware` un-homes).
            Command::Simulator(p) => {
                self.held.clear();
                self.simulator = p.on;
                self.snap.homed = p.on;
                self.publish();
            }
            Command::ConnectHardware(_) => {
                self.held.clear();
                self.simulator = false;
                self.snap.homed = false;
                self.publish();
            }
            Command::SelectProfile(p) => {
                let known = self
                    .cfg
                    .profiles
                    .iter()
                    .find(|x| x.eq_ignore_ascii_case(&p.profile))
                    .cloned();
                match known {
                    Some(name) => {
                        self.profile = name;
                        self.sync_planner();
                    }
                    None => {
                        return self.refuse(make_error(
                            ErrorCode::SysProfileInvalid,
                            UNATTRIBUTED,
                            &[("detail", &p.profile)],
                        ))
                    }
                }
            }
            Command::SetTcpOffset(p) => {
                self.tcp_offset_mm = [p.x, p.y, p.z];
                self.sync_planner();
            }
            Command::SetPayload(p) => {
                self.payload = PayloadSpec {
                    mass: p.mass,
                    com: p.com,
                    inertia: p.inertia,
                };
                self.sync_planner();
            }
            Command::SetShapes(p) => {
                if let Err(e) = self.planner.set_shapes(ShapeLayer::Program, &p.shapes) {
                    return self.refuse(e);
                }
                if let Err(e) = self.gate.set_layer(ShapeLayer::Program, &p.shapes) {
                    return self.refuse(e);
                }
            }
            Command::SetPidGains(p) => {
                if let Some(error) = pid_gains_fault(&p, &self.cfg) {
                    return self.refuse(error);
                }
            }
            Command::SetCompletionPolicy(p) => {
                self.policy = p.policy;
                self.sync_planner();
            }
            Command::EnterFlashing(_) => {
                self.held.clear();
                self.flashing = true;
            }
            // The runtime invalidates homing only when firmware was
            // actually flashed (`RtCore::leave_mode`), which a preview
            // never does, so the reference survives the window.
            Command::ExitFlashing => {
                if !self.flashing {
                    return self.refuse(detail(format!(
                        "exit_flashing while the controller mode is {:?}, not FLASHING",
                        self.snap.mode
                    )));
                }
                self.flashing = false;
            }
            other => return self.refuse(detail(format!("{:?} cannot be previewed", other.tag()))),
        }
        self.standing()
    }

    /// An accepted command that moves nothing: one sample at the pose
    /// the arm holds, so a timeline drawn from results never has a gap.
    fn standing(&mut self) -> PreviewResult {
        let q = self.snap.q;
        let mut r = PreviewResult::still(q);
        if let Ok(pose) = self.planner.current_pose(&q) {
            r.joint_trajectory_rad.push(q);
            r.tcp_poses.push(pose);
        }
        r
    }

    /// Plan one command outside the queue discipline (a streamed target).
    fn plan_one(&mut self, command: Command) -> PreviewResult {
        self.plan_batch(&[command])
            .pop()
            .expect("one command in, one result out")
    }

    // -------------------------------------------------------------- jogs

    /// A joint velocity jog held for `duration_s`: the same `par6-motion`
    /// jog engine the RT core ticks, integrated from the virtual pose
    /// (per-joint ramps, soft-limit direction blocking).
    pub fn preview_jog(
        &mut self,
        speeds: [f64; NUM_JOINTS],
        duration_s: f64,
        accel: Option<f64>,
    ) -> PreviewResult {
        let command = Command::JogJ(JogJ {
            speeds,
            duration: duration_s,
            accel,
        });
        if let Err(e) = command.validate() {
            return self.refuse(decode_error_to_wire(&e));
        }
        let mut fractions = [0.0; MAX_JOINTS];
        fractions[..NUM_JOINTS].copy_from_slice(&speeds);
        // The runtime admits a jog only if where it will be one lookahead
        // horizon ahead clears the world (`RtBridge`'s jog admission).
        if let Some(error) = self.jog_blocked(&fractions) {
            return self.refuse(error);
        }
        let scale = accel.unwrap_or(1.0);
        self.jog.set_accel_scale(scale);
        self.jog_accel_scale = scale;
        // The RT ramps from rest on JOG mode ENTRY, not per datagram: a
        // UI streaming jogs at 20 Hz gets one acceleration, not one per
        // frame. Re-activating here would preview a fraction of the real
        // travel and clear the soft-limit latches between frames.
        if !self.jog_streaming {
            self.jog.activate(&self.snap.q);
            self.jog_streaming = true;
        }
        self.jog.command(&fractions);
        let ticks = ((duration_s / self.dt).round() as usize).max(1);
        let mut q = self.snap.q;
        let mut trajectory = Vec::with_capacity(ticks);
        for _ in 0..ticks {
            let mut q_out = [0.0; MAX_JOINTS];
            let mut qd_out = [0.0; MAX_JOINTS];
            self.jog.tick(&q, &mut q_out, &mut qd_out);
            q = q_out;
            trajectory.push(q);
        }
        self.finish_stream(trajectory, trajectory_duration(ticks, self.dt))
    }

    /// A cartesian velocity jog held for `duration_s`: the housekeeping
    /// loop's own integrator (`step_cart_jog`) at its own period — twist
    /// from the velocity fractions and the `[motion]` full-scale rates,
    /// a TRF twist rotated by the current orientation, joint rates
    /// through the damped jacobian, the target clamped to the soft
    /// window.
    ///
    /// What comes back is the setpoint stream the runtime feeds its
    /// streaming executor, which is what housekeeping computes. `accel`
    /// scales how hard that executor tracks the stream, so it is
    /// validated here but does not move the setpoints.
    pub fn preview_jog_l(
        &mut self,
        velocities: [f64; 6],
        frame: par6_proto::Frame,
        duration_s: f64,
        accel: Option<f64>,
    ) -> PreviewResult {
        let command = Command::JogL(cmd::JogL {
            velocities,
            duration: duration_s,
            frame,
            accel,
        });
        if let Err(e) = command.validate() {
            return self.refuse(decode_error_to_wire(&e));
        }
        let mut twist = [0.0; 6];
        for (i, (out, frac)) in twist.iter_mut().zip(velocities.iter()).enumerate() {
            let full = if i < 3 {
                self.motion.jog_l_linear_max_m_s
            } else {
                self.motion.jog_l_angular_max_rad_s
            };
            *out = frac * full;
        }
        let mut state = CartJogState {
            twist,
            frame,
            q: self.snap.q,
            soft_min: self.soft_min,
            soft_max: self.soft_max,
        };
        if let Some(error) = self.cart_jog_blocked(&mut state.clone()) {
            return self.refuse(error);
        }
        // Housekeeping emits a setpoint every period and the RT tracks it
        // at the tick — so that is what runs here, on the runtime's own
        // executor rather than on the raw setpoints.
        let period = HOUSEKEEPING_PERIOD.as_secs_f64();
        let steps = ((duration_s / period).round() as usize).max(1);
        let ticks_per_step = (period / self.dt).round().max(1.0) as usize;
        if !self.cart_streaming {
            self.stream.activate(&self.snap.q);
            self.cart_streaming = true;
        }
        self.stream.set_scale(1.0, accel.unwrap_or(1.0));
        let mut trajectory = Vec::with_capacity(steps * ticks_per_step);
        let mut q = self.snap.q;
        for _ in 0..steps {
            let target = match step_cart_jog(&mut self.cart, &mut state, period) {
                Ok((target, _)) => target,
                // A twist the jacobian cannot resolve holds in place, as
                // housekeeping holds on every failed solve.
                Err(_) => state.q,
            };
            self.stream.set_target(&target);
            for _ in 0..ticks_per_step {
                let mut qd_out = [0.0; MAX_JOINTS];
                self.stream.step(&mut q, &mut qd_out);
                trajectory.push(q);
            }
        }
        self.finish_stream(
            trajectory,
            trajectory_duration(steps * ticks_per_step, self.dt),
        )
    }

    /// The runtime's jog admission check: where the commanded speeds put
    /// the arm one lookahead horizon from here must not collide, or from
    /// inside a keep-out must not deepen it.
    fn jog_blocked(&mut self, fractions: &[f64; MAX_JOINTS]) -> Option<WireError> {
        let q = self.snap.q;
        let la = self.gate.jog_lookahead(&q, fractions);
        match self.gate.blocked(&q, &la) {
            Ok(Some(pairs)) => Some(self.gate.refuse(pairs)),
            Ok(None) => None,
            Err(e) => Some(e),
        }
    }

    /// The same admission check for a cartesian jog, projected through
    /// the jacobian exactly as the bridge projects it.
    fn cart_jog_blocked(&mut self, probe: &mut CartJogState) -> Option<WireError> {
        let q = self.snap.q;
        // A twist the jacobian cannot resolve is admitted: housekeeping
        // holds in place on a failed solve, so nothing unchecked streams.
        let la = match step_cart_jog(&mut self.cart, probe, STREAM_LOOKAHEAD_S) {
            Ok((la, _)) => la,
            Err(_) => return None,
        };
        match self.gate.blocked(&q, &la) {
            Ok(Some(pairs)) => Some(self.gate.refuse(pairs)),
            Ok(None) => None,
            Err(e) => Some(e),
        }
    }

    fn finish_stream(
        &mut self,
        trajectory: Vec<[f64; MAX_JOINTS]>,
        duration_s: f64,
    ) -> PreviewResult {
        self.latches.motion_accepted();
        self.advance(trajectory, duration_s)
    }

    /// Move the virtual arm along `trajectory` and report the motion.
    fn advance(&mut self, trajectory: Vec<[f64; MAX_JOINTS]>, duration_s: f64) -> PreviewResult {
        let end = trajectory.last().copied().unwrap_or(self.snap.q);
        let tcp_poses = self.poses_along(&trajectory);
        self.snap.q = end;
        self.publish();
        PreviewResult {
            joint_trajectory_rad: trajectory,
            tcp_poses,
            end_joints_rad: end,
            duration_s,
            error: None,
            pending: false,
        }
    }

    fn poses_along(&mut self, trajectory: &[[f64; MAX_JOINTS]]) -> Vec<[f64; 16]> {
        let mut poses = Vec::with_capacity(trajectory.len());
        for q in trajectory {
            match self.planner.current_pose(q) {
                Ok(pose) => poses.push(pose),
                Err(_) => break,
            }
        }
        poses
    }

    // ------------------------------------------------------------ planner

    /// Offer `cmds` to the planner in server order, so blend chains fold
    /// exactly as they would live. One result per command; commands
    /// folded into a predecessor's chain return an empty trajectory with
    /// the chain's end pose.
    fn plan_batch(&mut self, cmds: &[Command]) -> Vec<PreviewResult> {
        let mut results = Vec::with_capacity(cmds.len());
        let mut rest = cmds;
        while !rest.is_empty() {
            if let Err(e) = rest[0].validate() {
                results.push(self.refuse(decode_error_to_wire(&e)));
                rest = &rest[1..];
                continue;
            }
            if let Some(error) = self
                .check_gate(&rest[0])
                .or_else(|| validate_registries(&self.cfg, &rest[0]))
                .or_else(|| validate_supported(&self.cfg, &rest[0]))
            {
                results.push(self.refuse(error));
                rest = &rest[1..];
                continue;
            }
            // Only offer the leading run of wire-valid commands: a later
            // invalid one would have been refused at its own datagram, so
            // the planner must not fold it into this chain.
            let valid = rest
                .iter()
                .take(self.cfg.blend_lookahead.max(1))
                .take_while(|c| c.validate().is_ok() && validate_supported(&self.cfg, c).is_none())
                .count();
            self.publish();
            let batch: Vec<QueuedCommand<'_>> = rest[..valid]
                .iter()
                .enumerate()
                .map(|(k, cmd)| QueuedCommand {
                    index: self.next_index + k as u64,
                    cmd,
                })
                .collect();
            let (result, consumed) = match self.planner.start(&batch) {
                Err(error) => (self.refuse(error), 1),
                Ok(consumed) => {
                    let result = self.collect_plan(&rest[0]);
                    (result, consumed.clamp(1, valid))
                }
            };
            self.next_index += consumed as u64;
            let folded = consumed - 1;
            let end = result.end_joints_rad;
            let refused = result.error.is_some();
            results.push(result);
            for _ in 0..folded {
                results.push(PreviewResult::still(end));
            }
            rest = &rest[consumed..];
            if refused {
                // The server drops every pending command when one fails
                // (`fail_command` -> `drop_pending`), so nothing behind a
                // refusal runs, and the virtual arm must not advance past
                // it either.
                break;
            }
        }
        results
    }

    /// Read the in-flight plan off the planner, advance the virtual arm
    /// to where it ends, and cancel it (nothing executes here).
    fn collect_plan(&mut self, head: &Command) -> PreviewResult {
        let (trajectory, duration_s): (Vec<[f64; MAX_JOINTS]>, f64) =
            match self.planner.planned_motion(self.snap.tick) {
                PlannedMotion::Exec(samples) => {
                    let q: Vec<_> = samples.iter().map(|s| s.q).collect();
                    let duration = q.len() as f64 * self.dt;
                    (q, duration)
                }
                // The seek establishes the references and ends where the
                // configured sequence's last move_to steps leave the arm;
                // its wall-clock duration belongs to the physical seek,
                // not to a plan.
                PlannedMotion::Home => {
                    self.snap.homed = true;
                    (vec![self.ready_pose], 0.0)
                }
                PlannedMotion::Hold(ticks) => (Vec::new(), ticks as f64 * self.dt),
                PlannedMotion::Still => (Vec::new(), 0.0),
            };
        self.planner.cancel();
        self.note_effects(head);
        let mut result = self.advance(trajectory, duration_s);
        if result.joint_trajectory_rad.is_empty() {
            result = PreviewResult {
                duration_s,
                ..self.standing()
            };
        }
        result
    }

    /// What an accepted queued command changes besides the arm's pose —
    /// the server's post-effects and the tool state a program reads back.
    fn note_effects(&mut self, head: &Command) {
        match head {
            Command::SelectTool(p) => {
                // A variant carries its own TCP frame: a real change clears
                // the offset, a re-selection leaves it alone.
                if p.variant_key != self.tool_variant {
                    self.tcp_offset_mm = [0.0; 3];
                }
                self.tool_variant = p.variant_key.clone();
                self.sync_planner();
            }
            Command::ToolAction(p) => match p.action.as_str() {
                "move" => {
                    // The planner takes an integer position as readily as
                    // a float (a script's `move(1)` is a full close).
                    let position = match p.params.first() {
                        Some(ToolParam::Float(v)) => Some(*v),
                        Some(ToolParam::Int(v)) => Some(*v as f64),
                        _ => None,
                    };
                    if let Some(position) = position {
                        self.tool_position = position.clamp(0.0, 1.0);
                    }
                }
                "calibrate" => {
                    let mut reply = self.snap.gripper.reply.unwrap_or_default();
                    reply.calibrated = true;
                    reply.activated = true;
                    self.snap.gripper.reply = Some(reply);
                    self.tool_position = 0.0;
                }
                "idle" => self.tool_position = 0.0,
                _ => {}
            },
            _ => {}
        }
    }

    /// Replace one collision-world layer (wire units), exactly as the
    /// runtime would; a refused set leaves the enforced world unchanged.
    pub fn set_shapes(
        &mut self,
        layer: ShapeLayer,
        shapes: &[par6_proto::Shape],
    ) -> Result<Option<u64>, WireError> {
        let epoch = self.planner.set_shapes(layer, shapes)?;
        // The streaming gate keeps its own world, and only a set the
        // planner accepted reaches it — the same order the server uses.
        if epoch.is_some() {
            self.gate.set_layer(layer, shapes)?;
        }
        Ok(epoch)
    }
}

fn trajectory_duration(samples: usize, period_s: f64) -> f64 {
    samples as f64 * period_s
}

impl std::fmt::Debug for Preview {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Preview")
            .field("q", &self.snap.q)
            .field("homed", &self.snap.homed)
            .field("held", &self.held.len())
            .field("profile", &self.profile)
            .finish()
    }
}
