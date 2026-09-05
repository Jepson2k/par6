//! Offline dry-run preview: the daemon's OWN planner, driven through the
//! server's `Planner` trait against a fabricated harness instead of a
//! running RT core. A previewed command is planned by exactly the code
//! that would drive the arm — same profiles, same IK, same TOPPRA
//! timing, same collision gate — and then discarded instead of
//! dispatched, so a preview can never drift from the runtime.

use std::path::{Path, PathBuf};
use std::sync::mpsc;
use std::sync::{atomic::AtomicBool, Arc};

use par6_motion::JogEngine;
use par6_proto::Layer;
use par6_proto::{command::JogJ, Command, CompletionPolicy, WireError, NUM_JOINTS};
use par6_proto::{make_error, ErrorCode, UNATTRIBUTED};
use par6_rt::{
    sample_ring, snapshot_channel, ExecHeartbeat, JogEngine as RtJogEngine, Mode, SampleConsumer,
    SnapshotWriter, StateSnapshot, MAX_JOINTS,
};
use par6_server::{decode_error_to_wire, gate, PlanContext, Planner, QueuedCommand};

use crate::adapters::MotionJog;
use crate::bridge::{CoreLink, CoreOp};
use crate::daemon::{load_kin_stack, DaemonError};
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
    /// The server config the live daemon would run with this bundle —
    /// what `validate_supported` refuses against, so a parameter the
    /// runtime cannot honour previews as the same refusal.
    cfg: par6_server::ServerConfig,
    /// The applied world — the config's installation layer plus the
    /// program layer this session set — read back exactly as the runtime's.
    world: par6_server::WorldState,
    /// The preview's runtime payload — none today; the field keeps the
    /// planner sync honest if a payload surface is added offline.
    payload: par6_server::PayloadSpec,
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
        let jog = MotionJog::new(JogEngine::new(robot)?, robot.jog.accel_time_s);
        let dt = robot.robot.tick_dt_s;
        let motion = robot.motion;
        let cfg = crate::daemon::server_config(&opts, &bundle);
        let mut preview = Self {
            planner,
            jog,
            snap,
            snap_w,
            next_index: 0,
            dt,
            motion,
            cfg,
            world: par6_server::WorldState::default(),
            payload: par6_server::PayloadSpec::default(),
            _cmds_rx: cmds_rx,
            _ops_rx: ops_rx,
            _ring: ring,
        };
        preview.publish();
        // The installation layer is config, and the runtime applies it at
        // startup — so does the preview, or a dry run plans through a
        // keep-out the arm will refuse.
        preview
            .world
            .install(&mut preview.planner, |_, _| Ok(None), &preview.cfg)
            .map_err(|e| {
                DaemonError::Config(par6_config::ConfigError::Invalid {
                    field: "installation_shapes".into(),
                    reason: e.cause,
                })
            })?;
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
        self.planner.sync(PlanContext {
            profile,
            tool: "",
            tool_variant: None,
            tcp_offset_mm,
            completion_policy: policy,
            payload: self.payload,
        });
    }

    /// Replace the program layer (wire units), exactly as the runtime
    /// would; a refused set leaves the enforced world unchanged. Returns
    /// the epoch of the applied world. The installation layer is config,
    /// applied when the preview boots — it cannot be set from here, exactly
    /// as it cannot from the wire.
    pub fn set_shapes(&mut self, shapes: &[par6_proto::Shape]) -> Result<u64, WireError> {
        self.world.apply(
            &mut self.planner,
            |_, _| Ok(None),
            Layer::Program,
            shapes.to_vec(),
        )?;
        Ok(self.world.epoch())
    }

    /// The applied world: the config's installation layer, the program
    /// layer this session set, and the epoch — the runtime's SHAPES
    /// readback, for the same config.
    pub fn world(&self) -> &par6_server::WorldState {
        &self.world
    }

    /// Colliding pairs at `q`, in the runtime's reporting vocabulary.
    pub fn colliding_pairs(
        &mut self,
        q: &[f64; MAX_JOINTS],
    ) -> Result<Vec<(String, String)>, WireError> {
        self.planner.colliding_pairs(q)
    }

    /// Whether `q` collides, in any pair.
    pub fn in_collision(&mut self, q: &[f64; MAX_JOINTS]) -> Result<bool, WireError> {
        self.planner.in_collision(q)
    }

    /// Minimum signed distance over every pair at `q` \[m\]; negative =
    /// penetrating.
    pub fn min_distance(&mut self, q: &[f64; MAX_JOINTS]) -> Result<f64, WireError> {
        self.planner.min_distance(q)
    }

    /// Index of the first colliding sample along `path`.
    pub fn first_collision(
        &mut self,
        path: &[[f64; MAX_JOINTS]],
    ) -> Result<Option<usize>, WireError> {
        self.planner.first_collision(path)
    }

    /// Default standoff \[m\] applied to pairs without a shape override.
    pub fn clearance(&self) -> f64 {
        self.planner.clearance()
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
        let mut tcp_poses = Vec::with_capacity(trajectory.len());
        for q in &trajectory {
            match self.planner.current_pose(q) {
                Ok(pose) => tcp_poses.push(pose),
                Err(_) => break,
            }
        }
        self.snap.q = q;
        self.publish();
        PreviewResult {
            duration_s: trajectory.len() as f64 * self.dt,
            joint_trajectory_rad: trajectory,
            tcp_poses,
            end_joints_rad: q,
            error: None,
        }
    }

    /// Preview a queued program: commands are offered to the planner in
    /// server order, so blend chains fold exactly as they would live.
    /// One result per command; commands folded into a predecessor's
    /// chain return an empty trajectory with the chain's end pose.
    pub fn preview_batch(&mut self, cmds: &[Command]) -> Vec<PreviewResult> {
        let mut results = Vec::with_capacity(cmds.len());
        let mut rest = cmds;
        while !rest.is_empty() {
            // The runtime validates every datagram at decode; an invalid
            // command is refused there and never reaches the queue, so it
            // never enters the planner here either.
            if let Err(e) = rest[0].validate() {
                results.push(self.refusal(decode_error_to_wire(&e)));
                rest = &rest[1..];
                continue;
            }
            // The server refuses planned motion while unreferenced —
            // its own gate table, applied here so the preview answers
            // exactly what the runtime would.
            if gate(rest[0].tag()).needs_homed && !self.snap.homed {
                results.push(self.refusal(make_error(ErrorCode::MotnNotHomed, UNATTRIBUTED, &[])));
                rest = &rest[1..];
                continue;
            }
            // ...and parameters the runtime cannot honour (a blend
            // radius on move_c, say) — the server's own check over the
            // same config, so the preview never accepts a command the
            // live ack refuses.
            if let Some(error) = par6_server::validate_supported(&self.cfg, &rest[0]) {
                results.push(self.refusal(error));
                rest = &rest[1..];
                continue;
            }
            // Only offer the leading run of wire-valid commands: a later
            // invalid one would have been refused at its own datagram, so
            // the planner must not fold it into this chain.
            let valid = rest
                .iter()
                .take_while(|c| {
                    c.validate().is_ok() && par6_server::validate_supported(&self.cfg, c).is_none()
                })
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
                    let trajectory: Vec<[f64; MAX_JOINTS]> = match self.planner.planned_motion() {
                        PlannedMotion::Exec(samples) => samples.iter().map(|s| s.q).collect(),
                        PlannedMotion::Home => {
                            homing_seek = true;
                            vec![self.planner.home_pose()]
                        }
                        PlannedMotion::Still => Vec::new(),
                    };
                    if homing_seek {
                        // The seek establishes the references; where it
                        // ends is the configured homing-ready pose.
                        self.snap.homed = true;
                    }
                    self.planner.cancel();
                    let end = trajectory.last().copied().unwrap_or(self.snap.q);
                    let duration_s = trajectory.len() as f64 * self.dt;
                    let mut tcp_poses = Vec::with_capacity(trajectory.len());
                    for q in &trajectory {
                        match self.planner.current_pose(q) {
                            Ok(pose) => tcp_poses.push(pose),
                            Err(_) => break,
                        }
                    }
                    self.snap.q = end;
                    (
                        PreviewResult {
                            joint_trajectory_rad: trajectory,
                            tcp_poses,
                            end_joints_rad: end,
                            duration_s,
                            error: None,
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
            rest = &rest[consumed..];
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

    /// The config path search used when `Preview::new` gets `None`.
    pub fn default_config_path() -> Result<PathBuf, String> {
        resolve_config_path(None)
    }
}
