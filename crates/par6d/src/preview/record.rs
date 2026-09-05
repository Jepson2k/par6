//! What a dry run brings back: a columnar record of the ticks the
//! engine ran.
//!
//! A tick record is not a plan. Every number in here was read off the
//! same snapshot the daemon publishes, after the same control laws ran
//! against the same plant, so a consumer replaying it is replaying what
//! the arm did rather than what it was asked to do. The two are stored
//! side by side — [`TickBatch::q_rad`] against
//! [`TickBatch::q_commanded_rad`] — because the gap between them is the
//! servo error and the gravity sag, and it is the thing worth looking at.
//!
//! # Resolution
//!
//! The simulation always runs at the robot's full tick rate. `stride`
//! is the resolution of the *record*, not of the physics: with the
//! engine at 250 Hz and a consumer that paints at 50 Hz, four rows in
//! five would never be drawn, so only every fifth is kept. Anything
//! transient enough to fall between two rows is carried as an event
//! against the row it landed on instead of as a column.
//!
//! `f32` throughout the sampled columns. It resolves about 1e-7 rad at
//! joint scale, two orders finer than the 14-bit encoder the real arm
//! reports through, so the record is limited by the robot and not by
//! its own storage.

use par6_bus::ObjectDetection;
use par6_proto::WireError;
use par6_rt::{Mode, StateSnapshot};

/// Rows are kept at roughly this rate \[Hz\] — the fastest any consumer
/// paints. Faster storage would be discarded on the way to the screen.
const ROW_RATE_HZ: f64 = 50.0;

/// Why the run stopped.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StopReason {
    /// Every command finished.
    Completed,
    /// A command was refused or failed; the span carries the error.
    Failed,
    /// The tick budget ran out with commands still queued. The record is
    /// what happened up to there, and it is not the whole program.
    BudgetExhausted,
}

/// One program command's rows.
#[derive(Debug, Clone)]
pub struct CommandSpan {
    /// Index into the command list the run was given.
    pub command: usize,
    /// First row this command owns.
    pub start_row: usize,
    /// How many rows it owns. Zero for a command the planner folded into
    /// a predecessor's blend chain, or one refused before it ran.
    pub rows: usize,
    /// The refusal the runtime would answer with, when there is one.
    pub error: Option<WireError>,
}

/// Where one world object went. Named by its shape name — the identity
/// it has in the collision world and in readback.
#[derive(Debug, Clone, PartialEq)]
pub struct ObjectTrack {
    /// The shape's name.
    pub name: String,
    /// `[x, y, z, qw, qx, qy, qz]` per row. An object that never moved
    /// carries a single row, which consumers broadcast.
    pub poses: Vec<[f32; 7]>,
}

/// A run-length span of one discrete column.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Span<T> {
    /// The first row the value holds from; it holds until the next
    /// span, or the end of the record.
    pub start_row: usize,
    /// What it is.
    pub value: T,
}

/// The record of a run.
///
/// Sampled columns are flat and row-major, `rows` long in their outer
/// dimension; `joints` is the arm's joint count, so `q_rad[r * joints
/// + j]`.
#[derive(Debug, Clone)]
pub struct TickBatch {
    /// Simulated seconds between rows.
    pub row_dt_s: f64,
    /// Simulated seconds per engine tick.
    pub tick_dt_s: f64,
    /// Engine ticks per row.
    pub stride: usize,
    /// Arm joints, the inner dimension of the joint columns.
    pub joints: usize,
    /// Recorded rows, the outer dimension of every sampled column.
    pub rows: usize,
    /// Achieved joint positions \[rad\].
    pub q_rad: Vec<f32>,
    /// Commanded joint positions \[rad\], post-limiter — what went on the
    /// motor bus. The difference from `q_rad` is the tracking error.
    ///
    /// NaN until the first tick that commands a position. An idle arm
    /// holds itself with a gravity-compensating torque and no position
    /// target at all, so there is nothing to report and nothing for the
    /// achieved column to diverge from; a consumer drawing the gap
    /// should draw none over those rows rather than invent one.
    pub q_commanded_rad: Vec<f32>,
    /// Achieved TCP `[x y z (m), roll pitch yaw (rad)]`, `rows × 6`,
    /// in the wire's intrinsic-XYZ convention.
    pub tcp: Vec<f32>,
    /// Jaw closure, 0 = open … 1 = closed, one per row.
    pub tool_closed: Vec<f32>,
    /// Whether the jaws stopped on an object while closing, one per row.
    /// The gripper firmware's own detection bit, so a grip indicator
    /// cannot claim a hold the contacts contradict — and it reads the
    /// same on hardware, where there is no plant to ask.
    pub tool_gripping: Vec<bool>,
    /// Scene centre of mass \[m\], `rows × 3`. Empty without a plant.
    pub com: Vec<f32>,
    /// Operating mode, as spans.
    pub modes: Vec<Span<Mode>>,
    /// One entry per command the run was given, in order.
    pub commands: Vec<CommandSpan>,
    /// Every free world object's track, in scene order.
    pub objects: Vec<ObjectTrack>,
    /// Contact positions \[m\] for every row concatenated, 3 per contact;
    /// row `r` owns `contact_starts[r]..contact_starts[r + 1]`.
    pub contact_pos: Vec<f32>,
    /// World-frame contact forces \[N\], aligned with `contact_pos`.
    pub contact_force: Vec<f32>,
    /// Prefix offsets into the contact arrays, `rows + 1` long, counted
    /// in contacts rather than floats.
    pub contact_starts: Vec<u32>,
    /// Why the run ended.
    pub stop: StopReason,
}

impl TickBatch {
    /// Total simulated time the record covers \[s\].
    pub fn duration_s(&self) -> f64 {
        self.rows as f64 * self.row_dt_s
    }
}

impl StopReason {
    /// The name a consumer outside Rust identifies this by.
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Completed => "completed",
            Self::Failed => "failed",
            Self::BudgetExhausted => "budget_exhausted",
        }
    }
}

/// The RT mode's name, for a consumer outside Rust.
pub fn mode_name(mode: Mode) -> &'static str {
    match mode {
        Mode::Booting => "booting",
        Mode::Idle => "idle",
        Mode::ActiveError => "active_error",
        Mode::Homing => "homing",
        Mode::Jog => "jog",
        Mode::Stream => "stream",
        Mode::Exec => "exec",
        Mode::HandGuiding => "hand_guiding",
        Mode::Impedance => "impedance",
        Mode::SafetyStop => "safety_stop",
        Mode::Flashing => "flashing",
    }
}

/// Builds a [`TickBatch`] as the engine ticks.
///
/// The recorder decides *when* a row is taken and the caller decides
/// *what* the tick means: [`Recorder::tick`] is offered every tick and
/// keeps every `stride`-th, while command spans are opened and closed
/// around it by the run loop.
pub(crate) struct Recorder {
    stride: usize,
    since_row: usize,
    joints: usize,
    object_names: Vec<String>,
    /// Per object, the rows recorded so far. Collapsed to a single row
    /// at the end when nothing moved.
    tracks: Vec<Vec<[f32; 7]>>,
    /// Scratch the object read fills, sized once.
    poses: Vec<[f64; 7]>,
    contact_pos: Vec<[f64; 3]>,
    contact_force: Vec<[f64; 3]>,
    out: TickBatch,
}

/// Below this an object's motion is storage, not movement: a body at
/// rest in MuJoCo still jitters at solver tolerance.
const MOVED_M: f32 = 1e-4;

impl Recorder {
    pub(crate) fn new(tick_dt_s: f64, joints: usize, object_names: Vec<String>) -> Self {
        let stride = ((1.0 / (ROW_RATE_HZ * tick_dt_s)).round() as usize).max(1);
        let n = object_names.len();
        Self {
            stride,
            // The first tick offered is recorded: a run's opening pose is
            // where the arm actually is.
            since_row: stride - 1,
            joints,
            object_names,
            tracks: vec![Vec::new(); n],
            poses: vec![[0.0; 7]; n],
            contact_pos: Vec::new(),
            contact_force: Vec::new(),
            out: TickBatch {
                row_dt_s: tick_dt_s * stride as f64,
                tick_dt_s,
                stride,
                joints,
                rows: 0,
                q_rad: Vec::new(),
                q_commanded_rad: Vec::new(),
                tcp: Vec::new(),
                tool_closed: Vec::new(),
                tool_gripping: Vec::new(),
                com: Vec::new(),
                modes: Vec::new(),
                commands: Vec::new(),
                objects: Vec::new(),
                contact_pos: Vec::new(),
                contact_force: Vec::new(),
                contact_starts: vec![0],
                stop: StopReason::Completed,
            },
        }
    }

    /// How many rows have been recorded.
    pub(crate) fn rows(&self) -> usize {
        self.out.rows
    }

    /// Offer one tick. Returns whether it became a row.
    pub(crate) fn tick(&mut self, snap: &StateSnapshot, bus: &mut par6_bus::RuntimeBus) -> bool {
        self.since_row += 1;
        if self.since_row < self.stride {
            return false;
        }
        self.since_row = 0;
        self.out
            .q_rad
            .extend(snap.q[..self.joints].iter().map(|v| *v as f32));
        self.out
            .q_commanded_rad
            .extend(snap.q_commanded[..self.joints].iter().map(|v| *v as f32));
        self.out.tcp.extend(snap.tcp.iter().map(|v| *v as f32));
        self.out.tool_closed.push(
            snap.gripper
                .reply
                .map_or(f32::NAN, |r| f32::from(r.position) / 255.0),
        );
        // The firmware's own verdict, not ours: the jaws stopped short of
        // their target because something was between them. It reads the
        // same on hardware, where there is no plant to ask.
        self.out.tool_gripping.push(
            snap.gripper
                .reply
                .is_some_and(|r| r.object_detection == ObjectDetection::DetectedClosing),
        );
        if let Some(sim) = bus.sim_mut() {
            let n = sim.object_poses_into(&mut self.poses);
            for (track, pose) in self.tracks.iter_mut().zip(&self.poses[..n]) {
                let mut row = [0.0f32; 7];
                for (o, v) in row.iter_mut().zip(pose) {
                    *o = *v as f32;
                }
                track.push(row);
            }
            if let Some(com) = sim.center_of_mass() {
                self.out.com.extend(com.iter().map(|v| *v as f32));
            }
            self.contact_pos.clear();
            self.contact_force.clear();
            sim.contacts_into(&mut self.contact_pos, &mut self.contact_force);
            for (p, q) in self.contact_pos.iter().zip(&self.contact_force) {
                self.out.contact_pos.extend(p.iter().map(|v| *v as f32));
                self.out.contact_force.extend(q.iter().map(|v| *v as f32));
            }
        }
        self.out.rows += 1;
        self.out
            .contact_starts
            .push((self.out.contact_pos.len() / 3) as u32);
        if self.out.modes.last().map(|s| s.value) != Some(snap.mode) {
            self.out.modes.push(Span {
                start_row: self.out.rows - 1,
                value: snap.mode,
            });
        }
        true
    }

    /// Record one command's outcome.
    pub(crate) fn command_span(
        &mut self,
        command: usize,
        start_row: usize,
        rows: usize,
        error: Option<WireError>,
    ) {
        self.out.commands.push(CommandSpan {
            command,
            start_row,
            rows,
            error,
        });
    }

    pub(crate) fn finish(mut self, stop: StopReason) -> TickBatch {
        self.out.stop = stop;
        self.out.objects = self
            .object_names
            .into_iter()
            .zip(self.tracks)
            .map(|(name, mut poses)| {
                // A stand or a table holds one row for the whole run;
                // consumers broadcast it. Rotation is left out of the
                // test on purpose — a body that has not translated by a
                // tenth of a millimetre has not visibly turned either,
                // and a quaternion sign flip is not motion.
                let still = poses.first().is_some_and(|first| {
                    poses.iter().all(|p| {
                        p[..3]
                            .iter()
                            .zip(&first[..3])
                            .all(|(a, b)| (a - b).abs() < MOVED_M)
                    })
                });
                if still {
                    poses.truncate(1);
                }
                ObjectTrack { name, poses }
            })
            .collect();
        self.out
    }
}
