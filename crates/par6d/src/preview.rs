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

use par6_motion::{JogEngine, MotionLimits};
use par6_proto::command::{self as cmd, JogJ, ToolParam};
use par6_proto::{
    command_class, make_error, Command, CommandClass, CompletionPolicy, ErrorCode, WireError,
    NUM_JOINTS, UNATTRIBUTED,
};
use par6_rt::{
    sample_ring, snapshot_channel, ExecHeartbeat, JogEngine as RtJogEngine, Mode, SampleConsumer,
    SnapshotWriter, StateSnapshot, MAX_JOINTS,
};
use par6_server::{
    blend_radius_mm, cmd_name, decode_error_to_wire, gate, registry_fault, teleport_angle_fault,
    validate_supported, write_io_fault, PayloadSpec, PlanContext, Planner, QueuedCommand,
    ServerConfig, ShapeLayer,
};

use crate::adapters::MotionJog;
use crate::bridge::{step_cart_jog, CartJogState, CoreLink, CoreOp, HOUSEKEEPING_PERIOD};
use crate::daemon::{load_kin_stack, DaemonError};
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
    held: Vec<Command>,
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
    // Keep the stub channel/ring ends alive so the planner's control
    // sends stay silent no-ops instead of logged errors.
    _cmds_rx: mpsc::Receiver<par6_rt::RtCommand>,
    _ops_rx: mpsc::Receiver<CoreOp>,
    _ring: SampleConsumer,
}

impl Preview {
    /// Build a session from the robot config (default search when
    /// `None`) and assets tree, starting referenced at the park pose
    /// with the runtime's startup context.
    pub fn new(config: Option<&Path>, assets: Option<&Path>) -> Result<Self, DaemonError> {
        let opts = Options {
            sim: true,
            config: config.map(Path::to_path_buf),
            assets: assets.map(Path::to_path_buf),
            ..Options::default()
        };
        let config_path =
            resolve_config_path(opts.config.as_deref()).map_err(DaemonError::ConfigPath)?;
        let bundle = par6_config::ConfigBundle::load(&config_path)?;
        let robot = &bundle.robot;
        let stack = load_kin_stack(&opts, &config_path, robot, bundle.active_gripper())?;

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
        let jog = MotionJog::new(JogEngine::new(robot)?, robot.jog.accel_time_s);
        let cfg = crate::daemon::server_config(&opts, &bundle);
        let mut preview = Self {
            planner,
            jog,
            cart: stack.housekeeping,
            soft_min: stream_limits.soft_min,
            soft_max: stream_limits.soft_max,
            snap,
            snap_w,
            next_index: 0,
            dt: robot.robot.tick_dt_s,
            motion: robot.motion,
            ready_pose,
            held: Vec::new(),
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

    /// Move the virtual arm instantly without the wire's checks — the
    /// host seeding a session at the live arm's pose.
    pub fn place_rad(&mut self, q: [f64; MAX_JOINTS]) {
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
        if let Err(e) = command.validate() {
            return self.refuse(decode_error_to_wire(&e));
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

    fn check_gate(&self, command: &Command) -> Option<WireError> {
        let g = gate(command.tag());
        if g.needs_homed && !self.snap.homed {
            return Some(make_error(ErrorCode::MotnNotHomed, UNATTRIBUTED, &[]));
        }
        None
    }

    fn submit_queued(&mut self, command: Command) -> PreviewResult {
        if let Some(error) = self
            .check_gate(&command)
            .or_else(|| registry_fault(&command, &self.cfg))
            .or_else(|| validate_supported(&self.cfg, &command))
        {
            return self.refuse(error);
        }
        self.held.push(command);
        if self.holding_for_blend() {
            return PreviewResult::pending(self.snap.q);
        }
        self.run_held()
            .unwrap_or_else(|| PreviewResult::still(self.snap.q))
    }

    /// The server's hold rule: the LAST queued command asks to blend into
    /// a successor that has not arrived, and the lookahead is not full.
    /// Offline there is no hold expiry — the chain closes with the next
    /// stopping command or [`Self::flush`].
    fn holding_for_blend(&self) -> bool {
        self.held.len() < self.cfg.blend_lookahead
            && self
                .held
                .last()
                .and_then(blend_radius_mm)
                .is_some_and(|r| r > 0.0)
    }

    fn run_held(&mut self) -> Option<PreviewResult> {
        if self.held.is_empty() {
            return None;
        }
        let batch = std::mem::take(&mut self.held);
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
                if let Some(detail) = teleport_angle_fault(&p.angles, &self.cfg) {
                    return self.refuse(make_error(
                        ErrorCode::CommValidationError,
                        UNATTRIBUTED,
                        &[("detail", &detail)],
                    ));
                }
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
            Command::JogL(p) => self.preview_jog_l(p.velocities, p.frame, p.duration),
            // A streamed target is tracked by the RT's own OTG at the
            // cadence targets arrive; offline there is no cadence, so the
            // settle onto the newest target is the planner's joint move.
            Command::ServoJ(p) => {
                let speed = Some(p.speed.unwrap_or(1.0));
                self.plan_one(Command::MoveJ(cmd::MoveJ {
                    key: 0,
                    angles: p.angles,
                    duration: None,
                    speed,
                    accel: p.accel,
                    blend_radius: None,
                    rel: false,
                }))
            }
            Command::ServoJPose(p) => self.plan_one(Command::MoveJPose(cmd::MoveJPose {
                key: 0,
                pose: p.pose,
                duration: None,
                speed: Some(p.speed.unwrap_or(1.0)),
                accel: p.accel,
                blend_radius: None,
            })),
            Command::ServoL(p) => self.plan_one(Command::MoveJPose(cmd::MoveJPose {
                key: 0,
                pose: p.pose,
                duration: None,
                speed: Some(p.speed.unwrap_or(1.0)),
                accel: p.accel,
                blend_radius: None,
            })),
            other => self.refuse(make_error(
                ErrorCode::CommValidationError,
                UNATTRIBUTED,
                &[("detail", &format!("{:?} cannot be previewed", other.tag()))],
            )),
        }
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
                if p.clear_queue {
                    self.held.clear();
                }
            }
            Command::Estop => self.held.clear(),
            Command::Reset | Command::Pause(_) | Command::SetGravityComp(_) => {}
            Command::ResetState => {
                self.held.clear();
                self.tool.clone_from(&self.cfg.fitted_tool);
                self.tool_variant = None;
                self.tcp_offset_mm = [0.0; 3];
                self.policy = CompletionPolicy::Settled;
                self.profile = self.cfg.initial_profile.clone();
                self.sync_planner();
                if let Err(e) = self.planner.set_shapes(ShapeLayer::Program, &[]) {
                    return self.refuse(e);
                }
            }
            Command::WriteIo(p) => match write_io_fault(p.port, &self.cfg) {
                None => self.io_levels[usize::from(p.port)] = p.value,
                Some(e) => return self.refuse(e),
            },
            Command::Simulator(_) | Command::ConnectHardware(_) => {}
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
            }
            Command::SetPidGains(p) => {
                if !self.cfg.tunable_nodes.contains(&p.node) {
                    return self.refuse(detail(format!(
                        "set_pid_gains node {} is not a configured drive (tunable nodes: {:?})",
                        p.node, self.cfg.tunable_nodes
                    )));
                }
            }
            Command::SetCompletionPolicy(p) => {
                self.policy = p.policy;
                self.sync_planner();
            }
            Command::SetRecipe(p) => {
                if !self.cfg.recipes.iter().any(|r| r.name == p.name) {
                    return self.refuse(make_error(
                        ErrorCode::CommUnknownRecipe,
                        UNATTRIBUTED,
                        &[("name", &p.name)],
                    ));
                }
            }
            Command::EnterFlashing(_) => {
                self.held.clear();
                self.flashing = true;
            }
            // Leaving the window costs the references: the runtime
            // cannot tell a flash from a scan, so every exit re-homes.
            Command::ExitFlashing => {
                if !self.flashing {
                    return self.refuse(detail(
                        "exit_flashing: the controller is not in FLASHING".into(),
                    ));
                }
                self.flashing = false;
                self.snap.homed = false;
                self.publish();
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
        self.jog.set_accel_scale(accel.unwrap_or(1.0));
        self.jog.activate(&self.snap.q);
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
        self.jog.release();
        self.finish_stream(trajectory, trajectory_duration(ticks, self.dt))
    }

    /// A cartesian velocity jog held for `duration_s`: the housekeeping
    /// loop's own integrator (`step_cart_jog`) at its own period — twist
    /// from the velocity fractions and the `[motion]` full-scale rates,
    /// a TRF twist rotated by the current orientation, joint rates
    /// through the damped jacobian, the target clamped to the soft
    /// window.
    pub fn preview_jog_l(
        &mut self,
        velocities: [f64; 6],
        frame: par6_proto::Frame,
        duration_s: f64,
    ) -> PreviewResult {
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
        let period = HOUSEKEEPING_PERIOD.as_secs_f64();
        let steps = ((duration_s / period).round() as usize).max(1);
        let mut trajectory = Vec::with_capacity(steps);
        for _ in 0..steps {
            match step_cart_jog(&mut self.cart, &mut state, period) {
                // A twist the jacobian cannot resolve holds in place, as
                // housekeeping holds on every failed solve.
                Ok((q, _)) => trajectory.push(q),
                Err(_) => trajectory.push(state.q),
            }
        }
        self.finish_stream(trajectory, trajectory_duration(steps, period))
    }

    fn finish_stream(
        &mut self,
        trajectory: Vec<[f64; MAX_JOINTS]>,
        duration_s: f64,
    ) -> PreviewResult {
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
                .or_else(|| registry_fault(&rest[0], &self.cfg))
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
                .take_while(|c| c.validate().is_ok() && validate_supported(&self.cfg, c).is_none())
                .count()
                .min(self.cfg.blend_lookahead.max(1));
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
            results.push(result);
            for _ in 0..folded {
                results.push(PreviewResult::still(end));
            }
            rest = &rest[consumed..];
        }
        results
    }

    /// Read the in-flight plan off the planner, advance the virtual arm
    /// to where it ends, and cancel it (nothing executes here).
    fn collect_plan(&mut self, head: &Command) -> PreviewResult {
        let (trajectory, duration_s): (Vec<[f64; MAX_JOINTS]>, f64) =
            match self.planner.planned_motion() {
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
                PlannedMotion::Wait { ticks } => (Vec::new(), ticks as f64 * self.dt),
                PlannedMotion::Still => (Vec::new(), 0.0),
            };
        self.planner.cancel();
        self.note_effects(head);
        let end = trajectory.last().copied().unwrap_or(self.snap.q);
        let tcp_poses = self.poses_along(&trajectory);
        self.snap.q = end;
        self.publish();
        let mut result = PreviewResult {
            joint_trajectory_rad: trajectory,
            tcp_poses,
            end_joints_rad: end,
            duration_s,
            error: None,
            pending: false,
        };
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
                    if let Some(ToolParam::Float(position)) = p.params.first() {
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
        self.planner.set_shapes(layer, shapes)
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
