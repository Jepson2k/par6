//! Offline dry-run preview: the daemon's OWN planner, driven through the
//! server's `Planner` trait against a fabricated harness instead of a
//! running RT core. A previewed command is planned by exactly the code
//! that would drive the arm — same profiles, same IK, same TOPPRA
//! timing, same collision gate — and then discarded instead of
//! dispatched, so a preview can never drift from the runtime.

use std::path::Path;
use std::sync::mpsc;
use std::sync::{atomic::AtomicBool, Arc};

use par6_config::LimitMode;
use par6_motion::{JogEngine, MotionLimits};
use par6_proto::{
    command::{JogJ, JogL},
    Command, CompletionPolicy, Frame, WireError, NUM_JOINTS,
};
use par6_proto::{make_error, ErrorCode, UNATTRIBUTED};
use par6_rt::{
    sample_ring, snapshot_channel, ExecHeartbeat, JogEngine as RtJogEngine, Mode, SampleConsumer,
    SnapshotWriter, StateSnapshot, MAX_JOINTS,
};
use par6_server::{
    check_gate, decode_error_to_wire, validate_registries, validate_supported, GateContext,
    PayloadSpec, PlanContext, Planner, QueuedCommand, ShapeLayer,
};

use crate::adapters::MotionJog;
use crate::bridge::{step_cart_jog, CartJogState, CoreLink, CoreOp};
use crate::daemon::{load_preview_kin, DaemonError};
use crate::options::{resolve_config_path, Options};
use crate::planner::{profile_names, Par6Planner, PlannedMotion, PlannerKin};

/// One previewed command's outcome: the trajectory the runtime would
/// drive, or the exact refusal it would answer with.
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
}

impl PreviewResult {
    /// Whether the command would be accepted.
    pub fn valid(&self) -> bool {
        self.error.is_none()
    }
}

/// The offline preview session: a virtual arm pose plus the real planner.
pub struct Preview {
    planner: Par6Planner,
    jog: MotionJog,
    snap: StateSnapshot,
    snap_w: SnapshotWriter<StateSnapshot>,
    next_index: u64,
    dt: f64,
    motion: par6_config::MotionConfig,
    /// Cartesian-jog integration (`preview_jog_l`): the same solver the
    /// live bridge steps a `jog_l` with.
    cart: crate::kin::CartKin,
    /// STREAM-mode soft window the cartesian jog is clamped to.
    soft_min: [f64; MAX_JOINTS],
    soft_max: [f64; MAX_JOINTS],
    /// The server config the live daemon would run with this bundle —
    /// what `validate_supported` refuses against, so a parameter the
    /// runtime cannot honour previews as the same refusal.
    cfg: par6_server::ServerConfig,
    /// The planning context as last synced — what a payload change
    /// re-syncs the planner with.
    profile: String,
    tcp_offset_mm: [f64; 3],
    completion_policy: CompletionPolicy,
    /// The payload the preview plans with, as the live `set_payload`.
    payload: PayloadSpec,
    /// The config file this session was built from.
    config_path: std::path::PathBuf,
    // Keep the stub channel/ring ends alive so the planner's control
    // sends stay silent no-ops instead of logged errors.
    _cmds_rx: mpsc::Receiver<par6_rt::RtCommand>,
    _ops_rx: mpsc::Receiver<CoreOp>,
    _ring: SampleConsumer,
}

impl Preview {
    /// Build a preview session from the robot config (default search when
    /// `None`) and assets tree, starting at the configured park pose.
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
        let kin = load_preview_kin(&opts, &config_path, robot, bundle.active_gripper())?;
        let stream_limits = MotionLimits::from_config(robot, LimitMode::Stream)?;

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
                kin: kin.planner,
                collision: kin.collision,
                tool_offset: kin.tool_offset,
            },
        )?;

        let mut snap = StateSnapshot::default();
        for (out, rad) in snap.q.iter_mut().zip(robot.robot.park_pose_rad.iter()) {
            *out = *rad;
        }
        snap.homed = true;
        snap.mode = Mode::Idle;
        let jog = MotionJog::new(JogEngine::new(robot)?, robot.jog.accel_time_s);
        let dt = robot.robot.tick_dt_s;
        let motion = robot.motion;
        let cfg = crate::daemon::server_config(&opts, &bundle);
        let profile = cfg.initial_profile.clone();
        let mut preview = Self {
            config_path,
            planner,
            jog,
            snap,
            snap_w,
            next_index: 0,
            dt,
            motion,
            cart: kin.cart,
            soft_min: stream_limits.soft_min,
            soft_max: stream_limits.soft_max,
            cfg,
            profile,
            tcp_offset_mm: [0.0; 3],
            completion_policy: CompletionPolicy::Settled,
            payload: PayloadSpec::default(),
            _cmds_rx: cmds_rx,
            _ops_rx: ops_rx,
            _ring: ring,
        };
        preview.publish();
        Ok(preview)
    }

    fn publish(&mut self) {
        self.snap_w.publish(&self.snap);
    }

    /// The virtual arm pose \[rad\].
    pub fn angles_rad(&self) -> [f64; MAX_JOINTS] {
        self.snap.q
    }

    /// Move the virtual arm instantly (the preview's teleport).
    pub fn teleport_rad(&mut self, q: [f64; MAX_JOINTS]) {
        self.snap.q = q;
        self.publish();
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

    /// FK at the virtual pose (flattened row-major 4×4, translation in
    /// metres — the engine's SI frame; wire conversions live at the
    /// command boundary).
    pub fn pose(&mut self) -> Result<[f64; 16], WireError> {
        let q = self.snap.q;
        self.planner.current_pose(&q)
    }

    /// Registered motion profile names.
    pub fn profiles() -> Vec<String> {
        profile_names()
    }

    /// Apply planning context (profile, TCP offset, completion policy) —
    /// the same sync the server pushes to the live planner.
    pub fn set_context(
        &mut self,
        profile: &str,
        tcp_offset_mm: [f64; 3],
        policy: CompletionPolicy,
    ) {
        self.profile = profile.to_owned();
        self.tcp_offset_mm = tcp_offset_mm;
        self.completion_policy = policy;
        self.sync_context();
    }

    /// Replace the payload the preview plans with — the same spec the
    /// live `set_payload` pushes, refused by the same wire validation, so
    /// a program's gravity-dependent refusals preview the way they run.
    pub fn set_payload(&mut self, payload: PayloadSpec) -> Result<(), WireError> {
        Command::SetPayload(par6_proto::command::SetPayload {
            mass: payload.mass,
            com: payload.com,
            inertia: payload.inertia,
        })
        .validate()
        .map_err(|e| decode_error_to_wire(&e))?;
        self.payload = payload;
        self.sync_context();
        Ok(())
    }

    /// The payload the preview plans with.
    pub fn payload(&self) -> PayloadSpec {
        self.payload
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

    /// The planning context as last synced: profile, TCP offset \[mm\],
    /// completion policy — the runtime's own startup context until a
    /// program changes it.
    pub fn context(&self) -> (&str, [f64; 3], CompletionPolicy) {
        (&self.profile, self.tcp_offset_mm, self.completion_policy)
    }

    /// How many blended moves the live queue holds before it plans the
    /// chain as it stands.
    pub fn blend_lookahead(&self) -> usize {
        self.cfg.blend_lookahead
    }

    /// The config file this session was built from.
    pub fn config_path(&self) -> &Path {
        &self.config_path
    }

    fn sync_context(&mut self) {
        self.planner.sync(PlanContext {
            profile: &self.profile,
            tool: "",
            tool_variant: None,
            tcp_offset_mm: self.tcp_offset_mm,
            completion_policy: self.completion_policy,
            payload: self.payload,
        });
    }

    /// FK over a trajectory: every sample's pose, or the first failure
    /// as the structured error it is — never a list shorter than the
    /// trajectory.
    fn tcp_poses(&mut self, trajectory: &[[f64; MAX_JOINTS]]) -> Result<Vec<[f64; 16]>, WireError> {
        let mut out = Vec::with_capacity(trajectory.len());
        for q in trajectory {
            out.push(self.planner.current_pose(q)?);
        }
        Ok(out)
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

    /// Preview one command: plan it with the runtime's planner from the
    /// virtual pose, advance the virtual pose to where it ends, and
    /// return the trajectory (or the refusal). Never executes anything.
    pub fn preview(&mut self, cmd: Command) -> PreviewResult {
        self.preview_batch(&[cmd]).pop().expect("one command in")
    }

    fn refusal(&self, error: WireError) -> PreviewResult {
        PreviewResult {
            joint_trajectory_rad: Vec::new(),
            tcp_poses: Vec::new(),
            end_joints_rad: self.snap.q,
            duration_s: 0.0,
            error: Some(error),
        }
    }

    /// Preview a velocity jog held for `duration_s`: the same
    /// `par6-motion` jog engine the RT core ticks, integrated from the
    /// virtual pose (per-joint ramps, soft-limit direction blocking).
    /// Wire-invalid parameters are refused exactly as the runtime
    /// refuses the datagram; the virtual arm advances to where the jog
    /// ends.
    pub fn preview_jog(
        &mut self,
        speeds: [f64; NUM_JOINTS],
        duration_s: f64,
        accel: Option<f64>,
    ) -> PreviewResult {
        let cmd = Command::JogJ(JogJ {
            speeds,
            duration: duration_s,
            accel,
        });
        if let Err(e) = cmd.validate() {
            return self.refusal(decode_error_to_wire(&e));
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
        self.finish_jog(trajectory)
    }

    /// Gate an integrated jog trajectory on the collision world and FK
    /// it; the virtual arm advances only for a trajectory that passes.
    fn finish_jog(&mut self, trajectory: Vec<[f64; MAX_JOINTS]>) -> PreviewResult {
        if let Err(error) = self.planner.gate_samples(trajectory.iter().copied()) {
            return self.refusal(error);
        }
        let tcp_poses = match self.tcp_poses(&trajectory) {
            Ok(poses) => poses,
            Err(error) => return self.refusal(error),
        };
        let end = trajectory.last().copied().unwrap_or(self.snap.q);
        self.snap.q = end;
        self.publish();
        PreviewResult {
            duration_s: trajectory.len() as f64 * self.dt,
            joint_trajectory_rad: trajectory,
            tcp_poses,
            end_joints_rad: end,
            error: None,
        }
    }

    /// Preview a cartesian velocity jog: `velocities` are signed fractions
    /// of the configured full-scale linear (xyz) and angular (rotation
    /// about xyz) jog rates, held for `duration_s` in `frame` — the
    /// runtime's own twist integration through the same kinematics and
    /// soft window, gated on the collision world.
    pub fn preview_jog_l(
        &mut self,
        velocities: [f64; 6],
        frame: Frame,
        duration_s: f64,
        accel: Option<f64>,
    ) -> PreviewResult {
        let cmd = Command::JogL(JogL {
            velocities,
            duration: duration_s,
            frame,
            accel,
        });
        if let Err(e) = cmd.validate() {
            return self.refusal(decode_error_to_wire(&e));
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
        let ticks = ((duration_s / self.dt).round() as usize).max(1);
        let mut trajectory = Vec::with_capacity(ticks);
        for _ in 0..ticks {
            match step_cart_jog(&mut self.cart, &mut state, self.dt) {
                Ok((q, _)) => trajectory.push(q),
                Err(e) => {
                    return self.refusal(make_error(
                        ErrorCode::MotnSetupFailed,
                        UNATTRIBUTED,
                        &[("detail", &e)],
                    ))
                }
            }
        }
        self.finish_jog(trajectory)
    }

    /// Preview a queued program: commands are offered to the planner in
    /// server order, so blend chains fold exactly as they would live.
    /// One result per command; commands folded into a predecessor's
    /// chain return an empty trajectory with the chain's end pose.
    pub fn preview_batch(&mut self, cmds: &[Command]) -> Vec<PreviewResult> {
        let mut results = Vec::with_capacity(cmds.len());
        // The runtime refuses at admission — decode validation, the gate
        // table, the tool registries and the unsupported-parameter check,
        // in that order — so the preview answers exactly what the live
        // ack would, and a refused command never reaches the planner.
        let ctx = GateContext {
            estop_latched: false,
            enabled: true,
            homed: self.snap.homed,
            simulator: true,
        };
        let admitted: Vec<Option<WireError>> = cmds
            .iter()
            .map(|c| {
                c.validate()
                    .err()
                    .map(|e| decode_error_to_wire(&e))
                    .or_else(|| check_gate(c.tag(), &ctx))
                    .or_else(|| validate_registries(&self.cfg, c))
                    .or_else(|| validate_supported(&self.cfg, c))
            })
            .collect();
        let mut i = 0;
        while i < cmds.len() {
            if let Some(error) = &admitted[i] {
                results.push(self.refusal(error.clone()));
                i += 1;
                continue;
            }
            // Only offer the leading run of admissible commands: a later
            // refused one would have been refused at its own datagram, so
            // the planner must not fold it into this chain.
            let valid = admitted[i..].iter().take_while(|e| e.is_none()).count();
            let rest = &cmds[i..];
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
                Err(error) => (
                    PreviewResult {
                        joint_trajectory_rad: Vec::new(),
                        tcp_poses: Vec::new(),
                        end_joints_rad: self.snap.q,
                        duration_s: 0.0,
                        error: Some(error),
                    },
                    1,
                ),
                Ok(consumed) => {
                    let mut homing_seek = false;
                    let mut hold_ticks = 0u64;
                    let trajectory: Vec<[f64; MAX_JOINTS]> =
                        match self.planner.planned_motion(self.snap.tick) {
                            PlannedMotion::Exec(samples) => samples.iter().map(|s| s.q).collect(),
                            PlannedMotion::Home => {
                                homing_seek = true;
                                vec![self.planner.home_pose()]
                            }
                            PlannedMotion::Hold(ticks) => {
                                hold_ticks = ticks;
                                Vec::new()
                            }
                            PlannedMotion::Still => Vec::new(),
                        };
                    if homing_seek {
                        // The seek establishes the references; where it
                        // ends is the configured homing-ready pose.
                        self.snap.homed = true;
                    }
                    if let Command::ToolAction(action) = &cmds[i] {
                        if action.action == "calibrate" {
                            self.set_gripper_calibrated(true);
                        }
                    }
                    self.planner.cancel();
                    let end = trajectory.last().copied().unwrap_or(self.snap.q);
                    let duration_s = (trajectory.len() as f64 + hold_ticks as f64) * self.dt;
                    let (tcp_poses, error) = match self.tcp_poses(&trajectory) {
                        Ok(poses) => (poses, None),
                        Err(error) => (Vec::new(), Some(error)),
                    };
                    self.snap.q = end;
                    (
                        PreviewResult {
                            joint_trajectory_rad: trajectory,
                            tcp_poses,
                            end_joints_rad: end,
                            duration_s,
                            error,
                        },
                        consumed,
                    )
                }
            };
            self.next_index += consumed as u64;
            let folded = consumed.saturating_sub(1);
            let end = result.end_joints_rad;
            results.push(result);
            // Commands folded into the chain share its outcome; their own
            // slots report the chain's end with no trajectory of their own.
            for _ in 0..folded {
                results.push(PreviewResult {
                    joint_trajectory_rad: Vec::new(),
                    tcp_poses: Vec::new(),
                    end_joints_rad: end,
                    duration_s: 0.0,
                    error: None,
                });
            }
            i += consumed;
        }
        results
    }

    /// The effective `[motion]` feel constants the preview plans with —
    /// the same file the daemon reads, so a consumer that integrates a
    /// motion itself (the dry-run client's `jog_l`) uses the runtime's
    /// own values.
    pub fn motion(&self) -> par6_config::MotionConfig {
        self.motion
    }

    /// The tick period \[s\] trajectories are sampled at.
    pub fn tick_dt_s(&self) -> f64 {
        self.dt
    }
}
