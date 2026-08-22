//! Offline dry-run preview: the daemon's OWN planner, driven through the
//! server's `Planner` trait against a fabricated harness instead of a
//! running RT core. A previewed command is planned by exactly the code
//! that would drive the arm — same profiles, same IK, same TOPPRA
//! timing, same collision gate — and then discarded instead of
//! dispatched, so a preview can never drift from the runtime.

use std::path::{Path, PathBuf};
use std::sync::mpsc;
use std::sync::{atomic::AtomicBool, Arc};

use par6_proto::{Command, CompletionPolicy, WireError};
use par6_rt::{
    sample_ring, snapshot_channel, ExecHeartbeat, Mode, SampleConsumer, SnapshotWriter,
    StateSnapshot, MAX_JOINTS,
};
use par6_server::{PlanContext, Planner, QueuedCommand, ShapeLayer};

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
    snap: StateSnapshot,
    snap_w: SnapshotWriter<StateSnapshot>,
    next_index: u64,
    dt: f64,
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
        let dt = robot.robot.tick_dt_s;
        let mut preview = Self {
            planner,
            snap,
            snap_w,
            next_index: 0,
            dt,
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
        });
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

    /// Preview a queued program: commands are offered to the planner in
    /// server order, so blend chains fold exactly as they would live.
    /// One result per command; commands folded into a predecessor's
    /// chain return an empty trajectory with the chain's end pose.
    pub fn preview_batch(&mut self, cmds: &[Command]) -> Vec<PreviewResult> {
        let mut results = Vec::with_capacity(cmds.len());
        let mut rest = cmds;
        while !rest.is_empty() {
            self.publish();
            let batch: Vec<QueuedCommand<'_>> = rest
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
                    let trajectory: Vec<[f64; MAX_JOINTS]> = match self.planner.planned_motion() {
                        PlannedMotion::Exec(samples) => samples.iter().map(|s| s.q).collect(),
                        PlannedMotion::Home => vec![self.planner.home_pose()],
                        PlannedMotion::Still => Vec::new(),
                    };
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

    /// The tick period \[s\] trajectories are sampled at.
    pub fn tick_dt_s(&self) -> f64 {
        self.dt
    }

    /// The config path search used when `Preview::new` gets `None`.
    pub fn default_config_path() -> Result<PathBuf, String> {
        resolve_config_path(None)
    }
}
