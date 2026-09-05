//! Offline dry-run preview: the daemon's OWN planner, driven through the
//! server's `Planner` trait against a fabricated harness instead of a
//! running RT core. A previewed command is planned by exactly the code
//! that would drive the arm — same profiles, same IK, same TOPPRA
//! timing, same collision gate — and then discarded instead of
//! dispatched, so a preview can never drift from the runtime.

use std::path::{Path, PathBuf};
use std::sync::mpsc;
use std::sync::{atomic::AtomicBool, Arc};

use par6_bus::sim::rollout::Rollout;
use par6_bus::sim::scene::Scene;
use par6_config::{GripperConfig, RobotConfig};
use par6_motion::JogEngine;
use par6_proto::Layer;
use par6_proto::Shape;
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

/// Where one free world object went during a previewed command.
#[derive(Debug, Clone, PartialEq)]
pub struct ObjectTrack {
    /// The shape's name — the same identity it has in the collision world
    /// and the readback.
    pub name: String,
    /// `[x, y, z, qw, qx, qy, qz]` per trajectory sample; a stationary
    /// object carries one row.
    pub poses: Vec<[f64; 7]>,
    /// Riding the TCP rather than free.
    pub carried: bool,
    /// The track was stepped by the simulator; `false` when the rollout
    /// hit its step budget and the poses are where things stood.
    pub physics: bool,
}

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
    /// The free world objects' motion over this command (empty when the
    /// world has none).
    pub object_tracks: Vec<ObjectTrack>,
}

impl PreviewResult {
    /// Whether the command would be accepted.
    pub fn valid(&self) -> bool {
        self.error.is_none()
    }
}

/// A grasped object: its pose in the TCP frame at the grasp.
struct Carried {
    name: String,
    grasp: [f64; 16],
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
    /// The simulator scene the rollouts step, for this config's tool.
    scene: Scene,
    robot: RobotConfig,
    gripper: Option<GripperConfig>,
    /// The rollout, built on the first command that needs physics.
    rollout: Option<Rollout>,
    /// Objects the jaws hold, with the object pose in the TCP frame the
    /// grasp closed at.
    carried: Vec<Carried>,
    /// The previewed jaw position (0 = open, 1 = closed).
    jaw_closed: f64,
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
    /// `gripper` names the bundle's gripper to model instead of the
    /// config's active one — the tool a client is displaying — matched
    /// case-insensitively; an unknown name is a config error.
    pub fn new(
        config: Option<&Path>,
        assets: Option<&Path>,
        gripper: Option<&str>,
    ) -> Result<Self, DaemonError> {
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
        let gripper = match gripper {
            Some(name) => Some(
                bundle
                    .grippers
                    .iter()
                    .find(|g| g.name.eq_ignore_ascii_case(name))
                    .ok_or_else(|| {
                        DaemonError::Config(par6_config::ConfigError::Invalid {
                            field: "gripper".into(),
                            reason: format!("no gripper `{name}` in the config bundle"),
                        })
                    })?,
            ),
            None => bundle.active_gripper(),
        };
        let stack = load_kin_stack(&opts, &config_path, robot, gripper)?;

        let (cmds_tx, cmds_rx) = mpsc::channel();
        let (ops_tx, ops_rx) = mpsc::channel();
        let link = CoreLink::new(cmds_tx, ops_tx, Arc::new(AtomicBool::new(false)));
        let (producer, ring) = sample_ring(64);
        let (snap_w, snap_r) = snapshot_channel::<StateSnapshot>();
        let scene = Scene {
            tool: crate::daemon::scene_tool(stack.variant),
            assets: stack.assets_dir.clone(),
        };
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
            scene,
            robot: robot.clone(),
            gripper: gripper.cloned(),
            rollout: None,
            carried: Vec::new(),
            jaw_closed: 0.5,
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
            .install(&mut preview.planner, (), &preview.cfg)
            .map_err(|e| {
                DaemonError::Config(par6_config::ConfigError::Invalid {
                    field: "installation_shapes".into(),
                    reason: e.cause,
                })
            })?;
        Ok(preview)
    }

    /// Preview a tool action: the jaws move to `closed` (0 = open,
    /// 1 = closed) at the tool actions' default speed while the arm holds,
    /// stepped in the simulator scene. Closing on an object jams the jaws
    /// and the object becomes carried — it rides the TCP through the moves
    /// that follow — and opening releases whatever is held to fall and
    /// settle. With no free object in the world nothing is stepped and the
    /// action takes no time, exactly as the runtime's tool actions
    /// contribute none to a plan. The arm's pose never changes here.
    pub fn preview_tool(&mut self, closed: f64) -> PreviewResult {
        /// Step budget \[s\]; past it the track is reported as where
        /// things stood, not physics.
        const BUDGET_S: f64 = 2.0;
        /// Objects slower than this are settled \[m/s, rad/s\] — above
        /// the jitter of a body held between pressing pads.
        const SETTLED: f64 = 2e-2;
        /// Dwell after a jam before the grasp counts as settled \[s\].
        const JAM_DWELL_S: f64 = 0.2;
        /// A grasped object sits within this of the TCP \[m\].
        const GRASP_REACH_M: f64 = 0.15;
        let closed = closed.clamp(0.0, 1.0);
        let q = self.snap.q;
        let still = |this: &mut Self, tracks: Vec<ObjectTrack>| PreviewResult {
            joint_trajectory_rad: Vec::new(),
            tcp_poses: Vec::new(),
            end_joints_rad: this.snap.q,
            duration_s: 0.0,
            error: None,
            object_tracks: tracks,
        };
        let names = Rollout::free_object_names(&self.world_refs());
        if names.is_empty() {
            self.jaw_closed = closed;
            return still(self, Vec::new());
        }
        let Ok(tcp) = self.planner.current_pose(&q) else {
            self.jaw_closed = closed;
            return still(self, self.standing_tracks(&names));
        };
        let n = self.robot.joints.len();
        let target = closed * 255.0;
        let closing = closed > self.jaw_closed;
        if !closing {
            // Opening releases whatever the jaws held.
            self.carried.clear();
        }
        let Some(roll) = self.ensure_rollout() else {
            self.jaw_closed = closed;
            return still(self, self.standing_tracks(&names));
        };
        roll.place_arm(&q[..n]);
        let dt = roll.dt();
        let budget = (BUDGET_S / dt).round() as usize;
        let mut poses: Vec<Vec<[f64; 7]>> = vec![Vec::new(); names.len()];
        let mut jam = None;
        let mut jam_tick = 0;
        let mut settled = false;
        let mut ticks = 0;
        while ticks < budget {
            roll.step(Some(Rollout::jaw_drive(target)));
            ticks += 1;
            for (name, track) in names.iter().zip(poses.iter_mut()) {
                if let Some(p) = roll.object_pose(name) {
                    track.push(p);
                }
            }
            if closing && jam.is_none() {
                jam = roll.jaw_obstruction().0;
                jam_tick = ticks;
            }
            let arrived = roll.jaw_byte().is_some_and(|b| (b - target).abs() < 1.5);
            let moving = names
                .iter()
                .any(|name| roll.object_speed(name).is_some_and(|v| v > SETTLED));
            let jam_dwelt = jam.is_some() && (ticks - jam_tick) as f64 * dt >= JAM_DWELL_S;
            if (jam_dwelt || (arrived && !moving)) && ticks as f64 * dt >= 0.1 {
                settled = true;
                break;
            }
        }
        if closing && jam.is_some() {
            // The object between the pads: the free object nearest the TCP.
            let tcp_p = [tcp[3], tcp[7], tcp[11]];
            let nearest = names
                .iter()
                .filter_map(|name| roll.object_pose(name).map(|p| (name, p)))
                .map(|(name, p)| {
                    let d = (0..3)
                        .map(|k| (p[k] - tcp_p[k]).powi(2))
                        .sum::<f64>()
                        .sqrt();
                    (name, p, d)
                })
                .filter(|(_, _, d)| *d < GRASP_REACH_M)
                .min_by(|a, b| a.2.total_cmp(&b.2));
            if let Some((name, p, _)) = nearest {
                let grasp = mat_mul(&mat_inv_rigid(&tcp), &pose7_to_mat(&p));
                self.carried.retain(|c| c.name != *name);
                self.carried.push(Carried {
                    name: name.clone(),
                    grasp,
                });
            }
        }
        self.jaw_closed = closed;
        let carried = &self.carried;
        let object_tracks = names
            .iter()
            .zip(poses)
            .map(|(name, poses)| ObjectTrack {
                name: name.clone(),
                carried: carried.iter().any(|c| c.name == *name),
                physics: settled,
                poses,
            })
            .collect();
        PreviewResult {
            joint_trajectory_rad: vec![q; ticks],
            tcp_poses: vec![tcp; ticks],
            end_joints_rad: q,
            duration_s: ticks as f64 * dt,
            error: None,
            object_tracks,
        }
    }

    /// The previewed jaw position (0 = open, 1 = closed).
    pub fn jaw_closed(&self) -> f64 {
        self.jaw_closed
    }

    fn world_refs(&self) -> [&[Shape]; 2] {
        [self.world.installation(), self.world.program()]
    }

    /// The rollout for the current world, built on first use; `None` when
    /// the scene cannot be built (logged once, tracks then report standing
    /// poses without physics).
    fn ensure_rollout(&mut self) -> Option<&mut Rollout> {
        if self.rollout.is_none() {
            let n = self.robot.joints.len();
            let q: Vec<f64> = self.snap.q[..n].to_vec();
            let built = Rollout::new(
                &self.scene,
                &self.robot,
                self.gripper.as_ref(),
                &self.world_refs(),
                &q,
            );
            match built {
                Ok(mut roll) => {
                    roll.place_jaw(self.jaw_closed * 255.0);
                    self.rollout = Some(roll);
                }
                Err(e) => {
                    log::warn!("preview physics unavailable: {e}");
                    return None;
                }
            }
        }
        self.rollout.as_mut()
    }

    /// Rebuild the rollout's world after a layer change; objects that no
    /// longer exist are no longer carried.
    fn sync_rollout_world(&mut self) {
        let names = Rollout::free_object_names(&self.world_refs());
        self.carried.retain(|c| names.contains(&c.name));
        let world = [
            self.world.installation().to_vec(),
            self.world.program().to_vec(),
        ];
        if let Some(roll) = self.rollout.as_mut() {
            if let Err(e) = roll.set_world(&[&world[0], &world[1]]) {
                log::warn!("preview scene rebuild refused, rebuilding from scratch: {e}");
                self.rollout = None;
            }
        }
    }

    /// Where the free objects stand right now: the rollout's poses when it
    /// exists, else where the world declares them.
    fn standing_tracks(&self, names: &[String]) -> Vec<ObjectTrack> {
        names
            .iter()
            .map(|name| ObjectTrack {
                name: name.clone(),
                poses: vec![self.standing_pose(name)],
                carried: self.carried.iter().any(|c| c.name == *name),
                physics: true,
            })
            .collect()
    }

    fn standing_pose(&self, name: &str) -> [f64; 7] {
        if let Some(p) = self.rollout.as_ref().and_then(|r| r.object_pose(name)) {
            return p;
        }
        self.world_refs()
            .iter()
            .flat_map(|layer| layer.iter())
            .find(|s| s.name == name)
            .map(|s| pose6_to_pose7(&s.pose))
            .unwrap_or([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    }

    /// The objects' tracks over a planned trajectory ending at `end`:
    /// carried objects ride the TCP by their grasp transform, free ones
    /// stand where they are (one row). The rollout follows the arm and
    /// the carried objects to the end pose.
    fn tracks_along(
        &mut self,
        tcp_poses: &[[f64; 16]],
        end: &[f64; MAX_JOINTS],
    ) -> Vec<ObjectTrack> {
        let names = Rollout::free_object_names(&self.world_refs());
        if names.is_empty() {
            return Vec::new();
        }
        let tracks: Vec<ObjectTrack> = names
            .iter()
            .map(|name| match self.carried.iter().find(|c| c.name == *name) {
                Some(c) if !tcp_poses.is_empty() => ObjectTrack {
                    name: name.clone(),
                    poses: tcp_poses
                        .iter()
                        .map(|tcp| mat_to_pose7(&mat_mul(tcp, &c.grasp)))
                        .collect(),
                    carried: true,
                    physics: true,
                },
                carried => ObjectTrack {
                    name: name.clone(),
                    poses: vec![self.standing_pose(name)],
                    carried: carried.is_some(),
                    physics: true,
                },
            })
            .collect();
        self.settle_rollout(end);
        for t in &tracks {
            if t.carried {
                if let (Some(roll), Some(last)) = (self.rollout.as_mut(), t.poses.last()) {
                    roll.place_object(&t.name, *last);
                }
            }
        }
        tracks
    }

    /// Follow the arm to `q` in the rollout, if one exists.
    fn settle_rollout(&mut self, q: &[f64; MAX_JOINTS]) {
        let n = self.robot.joints.len();
        if let Some(roll) = self.rollout.as_mut() {
            roll.place_arm(&q[..n]);
        }
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
        self.settle_rollout(&q);
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
        self.world
            .apply(&mut self.planner, (), Layer::Program, shapes.to_vec())?;
        self.sync_rollout_world();
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
            object_tracks: Vec::new(),
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
        let object_tracks = self.tracks_along(&tcp_poses, &q);
        self.snap.q = q;
        self.publish();
        PreviewResult {
            duration_s: trajectory.len() as f64 * self.dt,
            joint_trajectory_rad: trajectory,
            tcp_poses,
            end_joints_rad: q,
            error: None,
            object_tracks,
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
                        object_tracks: Vec::new(),
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
                    let object_tracks = self.tracks_along(&tcp_poses, &end);
                    self.snap.q = end;
                    (
                        PreviewResult {
                            joint_trajectory_rad: trajectory,
                            tcp_poses,
                            end_joints_rad: end,
                            duration_s,
                            error: None,
                            object_tracks,
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
                    object_tracks: Vec::new(),
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

/// Row-major 4×4 product `a · b`.
fn mat_mul(a: &[f64; 16], b: &[f64; 16]) -> [f64; 16] {
    let mut out = [0.0; 16];
    for r in 0..4 {
        for c in 0..4 {
            out[4 * r + c] = (0..4).map(|k| a[4 * r + k] * b[4 * k + c]).sum();
        }
    }
    out
}

/// Inverse of a rigid transform: `Rᵀ`, `−Rᵀ·t`.
fn mat_inv_rigid(m: &[f64; 16]) -> [f64; 16] {
    let mut out = [0.0; 16];
    for r in 0..3 {
        for c in 0..3 {
            out[4 * r + c] = m[4 * c + r];
        }
        out[4 * r + 3] = -(0..3).map(|k| m[4 * k + r] * m[4 * k + 3]).sum::<f64>();
    }
    out[15] = 1.0;
    out
}

/// `[x, y, z, qw, qx, qy, qz]` of a rigid transform (Shepperd's method).
fn mat_to_pose7(m: &[f64; 16]) -> [f64; 7] {
    let (r00, r01, r02) = (m[0], m[1], m[2]);
    let (r10, r11, r12) = (m[4], m[5], m[6]);
    let (r20, r21, r22) = (m[8], m[9], m[10]);
    let trace = r00 + r11 + r22;
    let q = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        [s / 4.0, (r21 - r12) / s, (r02 - r20) / s, (r10 - r01) / s]
    } else if r00 > r11 && r00 > r22 {
        let s = (1.0 + r00 - r11 - r22).sqrt() * 2.0;
        [(r21 - r12) / s, s / 4.0, (r01 + r10) / s, (r02 + r20) / s]
    } else if r11 > r22 {
        let s = (1.0 + r11 - r00 - r22).sqrt() * 2.0;
        [(r02 - r20) / s, (r01 + r10) / s, s / 4.0, (r12 + r21) / s]
    } else {
        let s = (1.0 + r22 - r00 - r11).sqrt() * 2.0;
        [(r10 - r01) / s, (r02 + r20) / s, (r12 + r21) / s, s / 4.0]
    };
    [m[3], m[7], m[11], q[0], q[1], q[2], q[3]]
}

/// Rigid transform of `[x, y, z, qw, qx, qy, qz]`.
fn pose7_to_mat(p: &[f64; 7]) -> [f64; 16] {
    let (w, x, y, z) = (p[3], p[4], p[5], p[6]);
    [
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y - w * z),
        2.0 * (x * z + w * y),
        p[0],
        2.0 * (x * y + w * z),
        1.0 - 2.0 * (x * x + z * z),
        2.0 * (y * z - w * x),
        p[1],
        2.0 * (x * z - w * y),
        2.0 * (y * z + w * x),
        1.0 - 2.0 * (x * x + y * y),
        p[2],
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

/// `[x, y, z, qw, qx, qy, qz]` of a wire pose `[x, y, z, rx, ry, rz]`
/// (`R = Rz·Ry·Rx`).
fn pose6_to_pose7(pose: &[f64]) -> [f64; 7] {
    let (sx, cx) = (pose[3] / 2.0).sin_cos();
    let (sy, cy) = (pose[4] / 2.0).sin_cos();
    let (sz, cz) = (pose[5] / 2.0).sin_cos();
    let mul = |a: [f64; 4], b: [f64; 4]| {
        [
            a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
        ]
    };
    let q = mul(
        mul([cz, 0.0, 0.0, sz], [cy, 0.0, sy, 0.0]),
        [cx, sx, 0.0, 0.0],
    );
    [pose[0], pose[1], pose[2], q[0], q[1], q[2], q[3]]
}
