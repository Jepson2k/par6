// CALIBRATION RULE: No assistant-invented heuristics. Cite a primary source beside
// every metric, tuning method, and numerical decision rule. State what it supports.
// Vendor constants and explicit user requirements must identify their provenance.
// A citation for a formula does not justify an arbitrary threshold or certify hardware.
// User requirement: no blind waits. Advance on feedback or completion of a measured experiment.
use par6_bus::{
    hw::SocketCanBus,
    sim::{
        scene::{Scene, Tool},
        SimBus,
    },
    spectral::JointConversion,
    BusState, ConfigKind, DriveTune, DriverBus, ErrorFlags, GripperCommand, JointCommand,
    PollAction, PollKind,
};
use par6_config::{ConfigBundle, HomingStrategy, LimitMode, PreMove};
use std::{
    path::Path,
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc::SyncSender,
    },
    time::Duration,
};
static CANCEL: AtomicBool = AtomicBool::new(false);
extern "C" fn cancel(_: libc::c_int) {
    CANCEL.store(true, Ordering::Relaxed);
}
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
const N: usize = 6;
// Vendor stall-current threshold, not an automatic current-increase rule:
// https://github.com/Source-Robotics/RCB-Runtime/blob/main/robotics/homing.py
const STALL_CURRENT_FRACTION: f64 = 0.7;
/// Waiting out one silent drive is normal; a run that spends its time doing
/// nothing else has a bus problem no amount of waiting will fix.
const MAX_FEEDBACK_RECOVERIES: u32 = 24;
/// Whether a drive's error register names an actual fault.
///
/// `calibrated` and `activated` are status, not faults. The frame's own
/// error bit is deliberately not consulted: this arm's J1 raises it with
/// every register flag clear, normal bus voltage and calibration intact,
/// then goes quiet for ten seconds and comes back — a transient, not a
/// fault, and the register is what tells the two apart.
fn reported_fault(flags: &par6_bus::ErrorFlags) -> bool {
    flags.error
        || flags.temperature
        || flags.encoder
        || flags.vbus
        || flags.driver
        || flags.velocity
        || flags.current
        || flags.estop
        || flags.watchdog
}
#[derive(Clone, Copy, Debug)]
#[allow(clippy::large_enum_variant)] // Fixed-size queue entries avoid allocation on the RT thread.
enum Event {
    Phase(&'static str, usize),
    MotionStart(u64, usize, Motion, i32, f64, f64, bool),
    Motion(u64, usize, i32, i32, f64, Measurement),
    Detection(usize, Detection),
    Configure(u64, usize, f64, i32),
    HoldQuality(u64, usize, &'static str, HoldQuality),
    ReverseCheck(u64, usize, i32, i32, i64, i64),
    Tune(usize, GainStage, par6_config::Gains, f64),
    TuneScore(usize, usize, GainStage, search::Score, bool),
    TuneStop(u64, usize, i32, i32, u32),
    GainsAccepted(usize, par6_config::Gains),
    ToolDetected(u8, Option<par6_bus::DeviceInfo>),
    /// A drive stopped answering and then came back: tick, joint, seconds silent.
    FeedbackGap(u64, usize, f64),
    IdentificationPose(usize, usize, [f64; N]),
    IdentificationTorque(usize, [f64; N]),
    ArmFit(f64, f64),
    /// A drive's polled health changed. The poll rotates temperature, voltage
    /// and error flags per node (~84 ms each at this tick rate), and a run
    /// that loses a drive needs these to tell a reset (`calibrated` clears)
    /// from a supply dip (`vbus` sets) from a plain gap in communication.
    NodeStatus(u64, usize, NodeStatus),
    Band(usize, GainStage, search::Band),
    HomeRepeatability(u64, usize, i32, i32, u32, f64),
    Sample(
        u64,
        u64,
        u64,
        [JointCommand; N],
        [i32; N],
        [i32; N],
        [i16; N],
        [u64; N],
        [bool; N],
    ),
}
#[derive(Clone, Copy, Debug)]
enum Motion {
    Position(i32, f64),
    VelocityProfile(i32, f64),
    Seek(i32, bool),
    Velocity(i32, f64),
}
#[derive(Clone, Copy)]
struct VelocityTrial {
    speed: i32,
    seconds: f64,
    return_stop: Option<i32>,
}
// STEPFOC's cascade is velocity PI inside position P. Moving-response tuning
// follows the inner-to-outer order; StartupHold only recovers a quiet hold.
// https://source-robotics.github.io/STEPFOC-docs/PID_tuning/
// The separate tracking/holding requirements and ordering are user-approved.
#[derive(Clone, Copy, Debug, PartialEq)]
enum GainStage {
    StartupHold,
    Velocity,
    VelocityI,
    Position,
}
impl GainStage {
    fn score(
        self,
        measurement: Measurement,
        position_tolerance: f64,
        complete: bool,
    ) -> search::Score {
        let moving = measurement.moving_ratio();
        let holding = measurement.holding_ratio();
        // The settling tolerance applies once the commanded motion has ended;
        // the peak error while moving is lag, logged as a diagnostic.
        // https://www.kollmorgen.com/en-us/developer-network/akd-online-tuning-guide
        let position = measurement.settled_error_rad / position_tolerance;
        if !complete || !moving.is_finite() || !holding.is_finite() || !position.is_finite() {
            return search::Score::Invalid;
        }
        // Feasibility first: compare violated constraints by their magnitude,
        // and compare the holding objective only when constraints pass.
        // https://pymoo.org/constraints/feas_first.html
        // Sum positive constraint violations after normalizing each by its
        // existing acceptance limit; no penalty weight or relaxed limit:
        // https://pymoo.org/getting_started/part_2.html
        // https://github.com/anyoptimization/pymoo/blob/main/pymoo/core/individual.py
        let violation = (moving - 1.0).max(0.0)
            + if matches!(self, Self::Position | Self::StartupHold) {
                (position - 1.0).max(0.0)
            } else {
                0.0
            };
        if violation > 0.0 {
            search::Score::Infeasible { violation }
        } else {
            search::Score::Feasible { objective: holding }
        }
    }
}
#[derive(Clone, Copy, Debug, Default, PartialEq)]
enum Outcome {
    Complete,
    Blocked,
    Tracking,
    #[default]
    Timeout,
}
// User-requested diagnostics: capture the inputs and actual branch result of the
// existing vendor-derived detector. Formatting stays on the recording thread.
#[derive(Clone, Copy, Debug)]
struct Detection {
    tick: u64,
    elapsed_s: f64,
    window_s: f64,
    position_ticks: i32,
    encoder_range_ticks: i64,
    requested_ticks: f64,
    stall_below_ticks: f64,
    current_ma: f64,
    current_threshold_ma: f64,
    limit_ma: f64,
    high_current_samples: usize,
    fresh_samples: usize,
    commanded_ticks_s: i32,
    encoder_ticks_s: i32,
    tracking_error_deg: f64,
    stopped: bool,
    loaded: bool,
    advancing: bool,
    at_target: bool,
    position_tracking_enabled: bool,
    decision: Option<Outcome>,
}
#[derive(Clone, Copy, Debug, Default)]
struct Measurement {
    // Kissling et al. (2009), §§3.1, 5.2: RMS speed and position tracking error.
    // https://doi.org/10.1016/j.conengprac.2009.02.005
    // Accumulate the movement separately from its following position hold.
    position_squared_rad: f64,
    velocity_squared_rad_s: f64,
    current_squared_ma: f64,
    samples: u64,
    stop: Option<i32>,
    peak_current_ma: f64,
    peak_error_rad: f64,
    settled_error_rad: f64,
    outcome: Outcome,
    elapsed_s: f64,
    direction: i32,
    worst_velocity_rms_rad_s: f64,
    hold_velocity_rms_rad_s: f64,
    // The moving limit that applied to the worst leg, and that leg's peak
    // commanded joint speed; the limit may scale with the command.
    moving_limit_rad_s: f64,
    peak_command_rad_s: f64,
    // The holding limit that applied to this joint. Like the moving limit it
    // is per-joint, because each joint resolves a different speed per encoder
    // count (see `Arm::holding_limit`).
    holding_limit_rad_s: f64,
}
impl Measurement {
    fn holding_ratio(self) -> f64 {
        if self.holding_limit_rad_s > 0.0 {
            self.hold_velocity_rms_rad_s / self.holding_limit_rad_s
        } else if self.hold_velocity_rms_rad_s == 0.0 {
            0.0
        } else {
            f64::INFINITY
        }
    }
    fn moving_ratio(self) -> f64 {
        if self.moving_limit_rad_s > 0.0 {
            self.worst_velocity_rms_rad_s / self.moving_limit_rad_s
        } else if self.worst_velocity_rms_rad_s == 0.0 {
            0.0
        } else {
            f64::INFINITY
        }
    }
    fn combine(self, other: Self) -> Self {
        let worst = if self.moving_ratio() >= other.moving_ratio() {
            self
        } else {
            other
        };
        Self {
            position_squared_rad: self.position_squared_rad + other.position_squared_rad,
            velocity_squared_rad_s: self.velocity_squared_rad_s + other.velocity_squared_rad_s,
            current_squared_ma: self.current_squared_ma + other.current_squared_ma,
            samples: self.samples + other.samples,
            peak_current_ma: self.peak_current_ma.max(other.peak_current_ma),
            peak_error_rad: self.peak_error_rad.max(other.peak_error_rad),
            settled_error_rad: self.settled_error_rad.max(other.settled_error_rad),
            outcome: other.outcome,
            elapsed_s: self.elapsed_s + other.elapsed_s,
            direction: other.direction,
            stop: other.stop,
            // Each leg and hold must pass; averaging cannot hide a bad leg.
            worst_velocity_rms_rad_s: worst.worst_velocity_rms_rad_s,
            moving_limit_rad_s: worst.moving_limit_rad_s,
            peak_command_rad_s: worst.peak_command_rad_s,
            holding_limit_rad_s: self.holding_limit_rad_s.max(other.holding_limit_rad_s),
            hold_velocity_rms_rad_s: self
                .hold_velocity_rms_rad_s
                .max(other.hold_velocity_rms_rad_s),
        }
    }
    fn rms(self) -> [f64; 3] {
        [
            self.position_squared_rad,
            self.velocity_squared_rad_s,
            self.current_squared_ma,
        ]
        .map(|sum| {
            if self.samples == 0 {
                f64::INFINITY
            } else {
                (sum / self.samples as f64).sqrt()
            }
        })
    }
}
// User-approved move-and-hold acceptance requirements. Kollmorgen specifies
// separate position, settling and velocity criteria, but supplies no PAR6 limits:
// https://www.kollmorgen.com/en-us/developer-network/akd-online-tuning-guide
// Limits must be supplied explicitly in joint-side units, never inferred from
// a possibly vibrating initial hold or silently widened by the gain search.
#[derive(Clone, Copy)]
struct QualityLimits {
    moving_rad_s: f64,
    // Fraction of the leg's peak commanded speed; the drive's reported speed
    // carries a ripple that scales with speed, so a fixed floor alone is a
    // different requirement on every joint (par6-selfcal.rs documents the value).
    moving_fraction: f64,
    holding_rad_s: f64,
    hold_observation_s: f64,
}
// Gain search geometry (par6-selfcal.rs documents each value's provenance).
#[derive(Clone, Copy, Debug, Default)]
struct SearchLimits {
    step: f64,
    ceiling: f64,
    resolution: f64,
}
impl QualityLimits {
    fn moving_limit(self, peak_command_rad_s: f64) -> f64 {
        self.moving_rad_s
            .max(self.moving_fraction * peak_command_rad_s)
    }
    /// Holding speed limit \[rad/s\] for a joint resolving `radians_per_tick`
    /// per motor count.
    ///
    /// The FOAW note below records that a still joint resting on a count
    /// boundary reads as a full count per tick, so speeds under one count per
    /// tick carry no information. A single scalar limit therefore asks
    /// something different of every joint: on this arm one count per tick is
    /// 0.220 deg/s on J2 but 1.373 deg/s on J4 and J5, so the configured
    /// 1.0 deg/s is unsatisfiable on those two — a one-count dither already
    /// exceeds it. Floor the limit at the joint's own resolution, exactly as
    /// `moving_limit` floors the moving limit at a fraction of the command.
    fn holding_limit(self, radians_per_tick: f64, dt: f64) -> f64 {
        // `radians_per_tick` carries the joint's direction sign.
        self.holding_rad_s.max(radians_per_tick.abs() / dt)
    }
}

// End-fit first-order adaptive windowing (FOAW), §III-A, equations (13–14):
// https://cim.mcgill.ca/~haptic/pub/FS-VH-CSC-TCST-00.pdf
// Position uncertainty is only ideal encoder quantization: ±0.5 motor count.
// The end-fit line passes through two quantized endpoints, each up to 0.5
// count from the true position, so a sample on the same true line can sit a
// full count from that line. The residual bound is therefore one count. At
// 0.5 a joint resting on a count boundary (readings alternating p, p+1) never
// extends its window past one sample and reads as one count per tick.
// No additional sensor-noise allowance is inferred from a vibrating hold.
// Use actual receive timestamps in place of n*T. History is bounded by the
// existing holding observation interval, not a new smoothing-time parameter.
const FOAW_RESIDUAL_COUNTS: f64 = 1.0;
#[derive(Default)]
struct HoldVelocity {
    history: Vec<(u64, i32)>,
    next: usize,
    len: usize,
    horizon_ns: u64,
}
impl HoldVelocity {
    fn new(seconds: f64, dt: f64) -> Result<Self> {
        let capacity = ((seconds / dt).ceil().max(1.0) as usize)
            .checked_add(1)
            .ok_or("holding observation is too large")?;
        let mut history = Vec::new();
        history.try_reserve_exact(capacity)?;
        history.resize(capacity, (0, 0));
        Ok(Self {
            history,
            horizon_ns: (seconds.max(dt) * 1e9) as u64,
            ..Self::default()
        })
    }
    fn reset(&mut self, time_ns: u64, position: i32) {
        self.history[0] = (time_ns, position);
        self.next = 1;
        self.len = 1;
    }
    fn push(&mut self, time_ns: u64, position: i32) -> Result<Option<f64>> {
        let capacity = self.history.len();
        let previous = self.history[(self.next + capacity - 1) % capacity];
        if time_ns <= previous.0 {
            return Err("holding encoder timestamps are not increasing".into());
        }
        self.history[self.next] = (time_ns, position);
        self.next = (self.next + 1) % capacity;
        self.len = (self.len + 1).min(capacity);
        let (mut lower, mut upper) = (f64::NEG_INFINITY, f64::INFINITY);
        let mut estimate = None;
        for lag in 1..self.len {
            let (past_ns, past_position) =
                self.history[(self.next + capacity - 1 - lag) % capacity];
            let elapsed_ns = time_ns - past_ns;
            if elapsed_ns > self.horizon_ns {
                break;
            }
            let elapsed = elapsed_ns as f64 * 1e-9;
            let delta = f64::from(position) - f64::from(past_position);
            let slope = delta / elapsed;
            // Equation (13)'s intermediate-point residual bounds rearranged
            // as slope intervals. Their intersection makes window growth O(n),
            // without rescanning every intermediate point at each extension.
            if slope < lower || slope > upper {
                break;
            }
            estimate = Some(slope);
            lower = lower.max((delta - FOAW_RESIDUAL_COUNTS) / elapsed);
            upper = upper.min((delta + FOAW_RESIDUAL_COUNTS) / elapsed);
        }
        Ok(estimate)
    }
}

// RMS tracking measurement: Kissling et al. (2009), §§3.1, 5.2:
// https://doi.org/10.1016/j.conengprac.2009.02.005
// The current AC RMS (sqrt(mean(I²) - mean(I)²)) is diagnostic only.
#[derive(Clone, Copy, Default, Debug)]
struct HoldQuality {
    samples: u64,
    position_squared_rad: f64,
    velocity_squared_rad_s: f64,
    velocity_samples: u64,
    raw_velocity_squared_rad_s: f64,
    current_sum_ma: f64,
    current_squared_ma: f64,
    peak_error_rad: f64,
    peak_current_ma: f64,
}
/// Whether a joint that this experiment was not tuning failed to hold.
///
/// A passive joint is asked to stay where it was put — its position
/// excursion — not to be quiet, which is the objective the gain search
/// minimises for the joint it is tuning. The two differ: on 2026-09-20 J5
/// rang at 2.2 deg/s after a J4 move while moving 0.038 deg in total, 7% of
/// the settling tolerance. Failing there aborts the run before the stage that
/// retunes J5, so a joint the run has not reached yet vetoes its own fix.
fn passive_hold_failed(
    quality: HoldQuality,
    holding_limit_rad_s: f64,
    position_tolerance: f64,
) -> bool {
    !quality
        .score(holding_limit_rad_s, position_tolerance)
        .feasible()
}
impl HoldQuality {
    fn score(self, holding_limit_rad_s: f64, position_tolerance: f64) -> search::Score {
        GainStage::StartupHold.score(
            Measurement {
                hold_velocity_rms_rad_s: self.rms()[1],
                peak_error_rad: self.peak_error_rad,
                settled_error_rad: self.peak_error_rad,
                holding_limit_rad_s,
                ..Measurement::default()
            },
            position_tolerance,
            self.samples > 0,
        )
    }
    fn push(
        &mut self,
        error_rad: f64,
        speed_rad_s: Option<f64>,
        raw_speed_rad_s: f64,
        current_ma: f64,
    ) {
        self.samples += 1;
        self.position_squared_rad += error_rad * error_rad;
        if let Some(speed) = speed_rad_s {
            self.velocity_samples += 1;
            self.velocity_squared_rad_s += speed * speed;
        }
        self.raw_velocity_squared_rad_s += raw_speed_rad_s * raw_speed_rad_s;
        self.current_sum_ma += current_ma;
        self.current_squared_ma += current_ma * current_ma;
        self.peak_error_rad = self.peak_error_rad.max(error_rad.abs());
        self.peak_current_ma = self.peak_current_ma.max(current_ma.abs());
    }
    fn raw_speed_rms(self) -> f64 {
        if self.samples == 0 {
            f64::INFINITY
        } else {
            (self.raw_velocity_squared_rad_s / self.samples as f64).sqrt()
        }
    }
    fn rms(self) -> [f64; 3] {
        if self.samples == 0 {
            return [f64::INFINITY; 3];
        }
        let n = self.samples as f64;
        [
            (self.position_squared_rad / n).sqrt(),
            if self.velocity_samples == 0 {
                f64::INFINITY
            } else {
                (self.velocity_squared_rad_s / self.velocity_samples as f64).sqrt()
            },
            (self.current_squared_ma / n - (self.current_sum_ma / n).powi(2))
                .max(0.0)
                .sqrt(),
        ]
    }
}
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct NodeStatus {
    voltage_mv: Option<i16>,
    temperature_c: Option<i16>,
    flags: Option<ErrorFlags>,
    live_error_bit: bool,
}
fn gravity_backoff(bundle: &ConfigBundle) -> f64 {
    bundle.robot.selfcal.approach_rad
}
/// Poses for identifying the arm's own links, spread over the joint
/// limits rather than chosen by hand.
///
/// Gravity fixes only about half a six-axis arm's inertial parameters,
/// and reaching that much takes roughly twenty well-spread poses; three
/// hand-picked ones reach two thirds of it (measured against this arm's
/// model, 2026-09-19). The spread is deterministic, so a run is
/// repeatable and a pose set can be replayed.
///
/// Each candidate is kept only if it, and the two approach offsets the
/// measurement uses, are inside the soft limits and clear of the
/// collision world, and if the straight joint-space path from the
/// previous kept pose is clear too.
fn identification_poses(
    bundle: &ConfigBundle,
    assets: &Path,
    ready: [f64; N],
    count: usize,
) -> Result<Vec<[f64; N]>> {
    let mut world = collision_world(bundle, assets)?;
    let backoff = [gravity_backoff(bundle); N];
    let mut seed = 0x2545_F491_4F6C_DD1Du64;
    let mut unit = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    let mut poses = Vec::with_capacity(count);
    let mut previous = ready;
    // Bounded: a pose set that cannot be filled says so rather than
    // spinning on a scene that refuses everything.
    for _ in 0..count * 200 {
        if poses.len() == count {
            break;
        }
        // Both limits: this arm's J6 declares a soft range wider than its
        // hard one, and a pose picked from the soft range alone drives it
        // into the mechanical stop at full current.
        //
        // A band around the ready pose rather than the whole range: at
        // ±20% of each joint's travel the parameter set is already fully
        // determined, with the same conditioning as the full sweep
        // (measured on this arm's model, 2026-09-19), and the arm spends
        // far less of the run travelling and stays clear of its stops.
        const BAND: f64 = 0.3;
        let bounds = |j: usize| {
            let l = &bundle.robot.joints[j].limits;
            let (lo, hi) = (
                l.soft_min_rad.max(l.hard_min_rad) + backoff[j],
                l.soft_max_rad.min(l.hard_max_rad) - backoff[j],
            );
            let span = (hi - lo) * BAND / 2.0;
            (
                (ready[j] - span).clamp(lo, hi),
                (ready[j] + span).clamp(lo, hi),
            )
        };
        if (0..N).any(|j| {
            let (lo, hi) = bounds(j);
            lo >= hi
        }) {
            return Err("a joint has no travel left once its approach offsets are allowed for".into());
        }
        let q: [f64; N] = std::array::from_fn(|j| {
            let (lo, hi) = bounds(j);
            lo + unit() * (hi - lo)
        });
        let approaches: [[f64; N]; 2] = [
            std::array::from_fn(|j| q[j] - backoff[j]),
            std::array::from_fn(|j| q[j] + backoff[j]),
        ];
        if [&q].into_iter().chain(&approaches).any(|p| {
            world
                .check(p, true)
                .map(|r| r.active())
                .unwrap_or(true)
        }) {
            continue;
        }
        // Every leg the measurement will actually drive.
        let legs = [
            (previous, approaches[0]),
            (approaches[0], q),
            (q, approaches[1]),
            (approaches[1], q),
        ];
        if legs
            .iter()
            .any(|(a, b)| world.check_segment(a, b, 40).map(|c| c.is_some()).unwrap_or(true))
        {
            continue;
        }
        poses.push(q);
        previous = q;
    }
    if poses.len() < count {
        return Err(format!(
            "only {} of {count} identification poses cleared the joint limits and the collision world",
            poses.len()
        )
        .into());
    }
    // Visit them nearest-first: a random order spends most of the run
    // travelling, and the poses are equally informative in any order.
    let mut ordered = Vec::with_capacity(poses.len());
    let mut at = ready;
    while !poses.is_empty() {
        let (i, _) = poses
            .iter()
            .enumerate()
            .map(|(i, q)| {
                let d: f64 = (0..N).map(|j| (q[j] - at[j]).abs()).sum();
                (i, d)
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .ok_or("no poses")?;
        at = poses.remove(i);
        ordered.push(at);
    }
    let poses = ordered;
    let previous = at;
    // Home again at the end, so the run finishes where parking expects.
    if world
        .check_segment(&previous, &ready, 40)?
        .is_some()
    {
        return Err("the return to the ready pose collides".into());
    }
    Ok(poses)
}

fn planned_ready(bundle: &ConfigBundle) -> [f64; N] {
    let mut q: [f64; N] = std::array::from_fn(|j| bundle.robot.robot.park_pose_rad[j]);
    for step in &bundle.robot.homing.sequence {
        if let Some(group) = &step.home {
            for j in &group.joints {
                if let Some(post) = bundle.robot.homing.joints[usize::from(*j)].post_home {
                    q[usize::from(*j)] = post.position_rad;
                }
            }
        }
        for m in &step.move_to {
            q[usize::from(m.joint)] = m.position_rad;
        }
    }
    q
}
/// The daemon's collision world for the fitted tool, with the configured
/// installation keep-outs applied.
fn collision_world(bundle: &ConfigBundle, assets: &Path) -> Result<par6_kin::Collision> {
    let variant = par6_kin::GripperVariant::resolve(
        &bundle.robot.robot.active_gripper.to_ascii_uppercase(),
        bundle
            .active_gripper()
            .and_then(|g| g.urdf_variant.as_deref()),
    );
    let mut world = par6_kin::Collision::load(assets, variant, par6_kin::COLLISION_CLEARANCE_M)?;
    let shapes = bundle
        .installation_shapes
        .iter()
        .map(par6_kin::Shape::from_proto)
        .collect::<std::result::Result<Vec<_>, _>>()?;
    world.set_layer(par6_kin::Layer::Installation, &shapes)?;
    Ok(world)
}

struct Arm {
    bundle: ConfigBundle,
    bus: Box<dyn DriverBus>,
    state: BusState,
    conv: [JointConversion; N],
    kin: par6_kin::Kin,
    // The ready pose the gravity path was checked from at planning time.
    planned_ready: Option<[f64; N]>,
    status: [NodeStatus; N],
    hold: [i32; N],
    hold_velocity: [HoldVelocity; N],
    feedback_ns: [u64; N],
    homed: [bool; N],
    // Encoder counts at the first hold, so joints the run never references
    // can be returned to where the run found them.
    found_at: [i32; N],
    gains: [par6_config::Gains; N],
    start_gains: [par6_config::Gains; N],
    search: SearchLimits,
    home_retries: usize,
    correction: Vec<f64>,
    generation: [u64; N],
    seen: [u64; N],
    tick: u64,
    dt: f64,
    simulated: bool,
    stopping: bool,
    // Fault/staleness scope. A joint whose feedback has gone stale must not
    // veto another joint's motion, above all not the shutdown park that puts
    // the shoulder and elbow back on their stops.
    check_only: Option<usize>,
    recoveries: u32,
    // Last-resort park: command the trajectory without requiring fresh
    // feedback. The drives close their own position loop on their own
    // encoders, so a blind park still lands; it just cannot be verified.
    blind: bool,
    /// How long a drive may go without fresh motion feedback before the run
    /// treats it as lost. Set by the CLI; see its documentation there.
    feedback_timeout_s: f64,
    focused_joint: Option<usize>,
    baseline_only: bool,
    trials: usize,
    used_trials: [usize; N],
    adaptive: bool,
    quality_limits: Option<QualityLimits>,
    startup_verified: bool,
    gain_stage: Option<GainStage>,
    // Set by a stage that found no passing scale: the least-violating scale.
    no_band: Option<f64>,
    // Absolute gain edges of the band each stage has already verified on this
    // joint; a later episode searches inside them instead of driving the joint
    // back to the oscillation edge.
    band_memory: [[Option<(f64, f64)>; 3]; N],
    deadline: Duration,
    events: Option<SyncSender<Event>>,
}
impl Arm {
    fn open(
        bundle: ConfigBundle,
        assets: &Path,
        simulated: bool,
        events: Option<SyncSender<Event>>,
        quality_limits: Option<QualityLimits>,
    ) -> Result<Self> {
        let tool = bundle.active_gripper();
        let bus: Box<dyn DriverBus> = if simulated {
            Box::new(SimBus::new(Scene {
                assets: assets.to_owned(),
                tool: tool
                    .and_then(|g| g.urdf_variant.as_deref())
                    .and_then(Tool::from_urdf_variant)
                    .unwrap_or(Tool::Flange),
            }))
        } else {
            Box::new(SocketCanBus::open(&bundle.robot.bus)?)
        };
        Self::adopt(bundle, assets, bus, simulated, events, quality_limits)
    }
    /// Take an already-built bus, so a test can install one that misbehaves
    /// in a way hardware does.
    fn adopt(
        bundle: ConfigBundle,
        assets: &Path,
        mut bus: Box<dyn DriverBus>,
        simulated: bool,
        events: Option<SyncSender<Event>>,
        quality_limits: Option<QualityLimits>,
    ) -> Result<Self> {
        // Allocate the estimator history before booting the bus or entering RT.
        let mut hold_velocity = std::array::from_fn(|_| HoldVelocity::default());
        if let Some(limits) = quality_limits {
            for estimator in &mut hold_velocity {
                *estimator =
                    HoldVelocity::new(limits.hold_observation_s, bundle.robot.robot.tick_dt_s)?;
            }
        }
        let tool = bundle.active_gripper();
        let kin = {
            let params = bundle.active_gripper().map(|g| {
                let k = &g.kinematics;
                par6_kin::Kin::dh_tool_params(
                    k.d_m,
                    k.a_m,
                    k.alpha_rad,
                    k.mass_kg,
                    k.com_m,
                    k.inertia_kg_m2,
                )
            });
            let mut kin = par6_kin::Kin::load_arm(assets, params.as_ref())?;
            // The daemon installs this (daemon.rs), so identifying against a
            // model without it would refit what the runtime already carries.
            kin.set_gravity_correction(&bundle.robot.gravity_correction)?;
            kin
        };
        // Begin position holding with a zero current cap, then raise it in the
        // paced startup ramp. Keep the operating limits in the actual bundle.
        // STEPFOC IN_LIMITS sets Iq_current_limit without changing the gains:
        // https://github.com/Source-Robotics/STEPFOC-stepper-controller/blob/V108/STEPFOC%20firmware/src/communication_CAN.cpp
        let mut boot = bundle.robot.clone();
        for joint in &mut boot.joints {
            joint.ilim_ma = 0.0;
        }
        bus.boot_configure(&boot, tool, bundle.robot.bus.boot_config_repeats)?;
        for j in &bundle.robot.joints {
            bus.send_clear_error(j.node_id, 3)?;
        }
        let robot = &bundle.robot;
        let default_feedback_timeout_s = robot.bus.stale_warn_s;
        Ok(Self {
            conv: std::array::from_fn(|j| JointConversion::from_config(&robot.joints[j])),
            gains: std::array::from_fn(|j| robot.joints[j].gains),
            start_gains: std::array::from_fn(|j| robot.joints[j].gains),
            search: SearchLimits::default(),
            home_retries: 1,
            correction: robot.gravity_correction.clone(),
            dt: robot.robot.tick_dt_s,
            bundle,
            bus,
            kin,
            planned_ready: None,
            status: [NodeStatus::default(); N],
            state: BusState::new(),
            hold: [0; N],
            hold_velocity,
            feedback_ns: [0; N],
            homed: [false; N],
            found_at: [0; N],
            generation: [0; N],
            seen: [0; N],
            tick: 0,
            simulated,
            stopping: false,
            check_only: None,
            recoveries: 0,
            blind: false,
            feedback_timeout_s: default_feedback_timeout_s,
            focused_joint: None,
            baseline_only: false,
            trials: 0, // Set explicitly by the CLI before any experiment.
            used_trials: [0; N],
            adaptive: false,
            quality_limits,
            startup_verified: false,
            gain_stage: None,
            no_band: None,
            band_memory: [[None; 3]; N],
            deadline: Duration::ZERO,
            events,
        })
    }
    fn emit(&self, event: Event) -> Result<()> {
        if let Some(tx) = &self.events {
            tx.try_send(event)
                .map_err(|_| "recording queue unavailable")?;
        }
        Ok(())
    }
    fn ticks(&self, seconds: f64) -> u64 {
        (seconds / self.dt).round().max(1.0) as u64
    }
    /// This joint's holding speed limit \[rad/s\], floored at its own encoder
    /// resolution (`QualityLimits::holding_limit`).
    fn holding_limit(&self, j: usize) -> f64 {
        let radians_per_tick = self.conv[j].joint_rad(1) - self.conv[j].joint_rad(0);
        self.quality_limits
            .map_or(f64::INFINITY, |limits| {
                limits.holding_limit(radians_per_tick, self.dt)
            })
    }
    fn node(&self, j: usize) -> usize {
        self.bundle.robot.joints[j].node_id as usize
    }
    fn pos(&self, j: usize) -> Result<i32> {
        self.state.nodes[self.node(j)]
            .position_ticks
            .ok_or_else(|| format!("J{} missing position", j + 1).into())
    }
    fn angles(&self) -> Result<[f64; N]> {
        let mut q = [0.0; N];
        for (j, v) in q.iter_mut().enumerate() {
            *v = self.conv[j].joint_rad(self.pos(j)?);
        }
        Ok(q)
    }
    /// Hold position and wait out a drive that has stopped answering.
    ///
    /// A STEPFOC drive answers CAN from `loop()` while its current loop
    /// runs in a 6250 Hz timer interrupt that also feeds the watchdog, so
    /// a starved `loop()` goes silent with the motor still controlled and
    /// nothing ever reporting a fault. On this arm J1 went quiet for
    /// 10.5 s mid-run with no error flag, normal bus voltage and all five
    /// other drives answering every tick; calling that a fault threw away
    /// eleven measured poses. Silence is therefore something to wait out,
    /// and only a drive that never comes back is a fault.
    fn recover_feedback(&mut self, j: usize) -> Result<()> {
        self.recoveries += 1;
        if self.recoveries > MAX_FEEDBACK_RECOVERIES {
            return Err(format!(
                "drives stopped answering {} times in one run; the bus is not healthy",
                self.recoveries
            )
            .into());
        }
        let stop: [i32; N] = std::array::from_fn(|k| self.pos(k).unwrap_or(self.hold[k]));
        let started = self.tick;
        self.emit(Event::Phase("silent drive: holding position until it answers", j))?;
        // The round robin reaches one node's error register every ~84 ms;
        // waiting on a drive is exactly when that reading cannot wait.
        let node = self.bundle.robot.joints[j].node_id;
        self.bus.queue_poll_override(
            par6_bus::PollAction::Poll {
                node,
                kind: par6_bus::PollKind::Errors,
            },
            4,
        );
        let blind = self.blind;
        self.blind = true;
        let timeout = self.ticks(self.feedback_timeout_s);
        let mut answered = false;
        let mut fault = None;
        for _ in 0..self.ticks(self.bundle.robot.selfcal.stale_recovery_s) {
            self.exchange(stop.map(|p| JointCommand::position(p, 0, 0)), false)?;
            let state = &self.state.nodes[usize::from(node)];
            if state.error_flags.as_ref().is_some_and(reported_fault) {
                fault = state.error_flags;
                break;
            }
            if !state.live_error_bit
                && (0..N).all(|k| self.tick.saturating_sub(self.seen[k]) <= timeout)
            {
                answered = true;
                break;
            }
        }
        self.blind = blind;
        if let Some(flags) = fault {
            return Err(format!("J{} reported a drive fault: {flags:?}", j + 1).into());
        }
        let silent = self.tick.saturating_sub(started);
        if !answered {
            return Err(format!(
                "J{} did not come back in {:.1} s of silence",
                j + 1,
                silent as f64 * self.dt
            )
            .into());
        }
        self.emit(Event::FeedbackGap(self.tick, j, silent as f64 * self.dt))?;
        Ok(())
    }
    fn exchange(&mut self, commands: [JointCommand; N], check: bool) -> Result<()> {
        if !self.stopping && CANCEL.load(Ordering::Relaxed) {
            return Err("cancelled".into());
        }
        // Judge the drives on what the last tick's drain showed, before this
        // one begins: recovery runs whole ticks of its own, and starting
        // those halfway through a tick would send this joint twice in it.
        if check && !self.blind {
            let only = self.check_only;
            let timeout = self.ticks(self.feedback_timeout_s);
            let mut faulted = None;
            let mut silent = None;
            for j in 0..N {
                if only.is_some_and(|watched| watched != j) {
                    continue;
                }
                let node = &self.state.nodes[self.node(j)];
                if node.error_flags.as_ref().is_some_and(reported_fault) {
                    faulted = Some((j, node.error_flags));
                    break;
                }
                if silent.is_none()
                    && (node.live_error_bit
                        || self.tick.saturating_sub(self.seen[j]) > timeout)
                {
                    silent = Some(j);
                }
            }
            if let Some((j, flags)) = faulted {
                return Err(format!("J{} reported a drive fault: {flags:?}", j + 1).into());
            }
            if let Some(j) = silent {
                self.recover_feedback(j)?;
            }
        }
        if !self.simulated {
            let now = runtime::now();
            if self.deadline.is_zero() {
                self.deadline = now;
            }
            if check
                && now > self.deadline + Duration::from_secs_f64(self.bundle.robot.bus.stale_warn_s)
            {
                return Err("control deadline missed".into());
            }
            runtime::sleep(self.deadline);
            self.deadline += Duration::from_secs_f64(self.dt);
        }
        self.tick += 1;
        self.bus.begin_tick(self.tick);
        self.bus.drain_rx(&mut self.state)?;
        let rx_ns = if self.simulated {
            (self.tick as f64 * self.dt * 1e9) as u64
        } else {
            runtime::now().as_nanos() as u64
        };
        for j in 0..N {
            let state = &self.state.nodes[self.node(j)];
            if state.position_generation != self.generation[j] {
                self.generation[j] = state.position_generation;
                self.feedback_ns[j] = rx_ns;
                self.seen[j] = self.tick;
            }
        }
        for j in 0..N {
            let node = &self.state.nodes[self.node(j)];
            let status = NodeStatus {
                voltage_mv: node.voltage_mv,
                temperature_c: node.temperature_c,
                flags: node.error_flags,
                live_error_bit: node.live_error_bit,
            };
            if status != self.status[j] {
                self.status[j] = status;
                self.emit(Event::NodeStatus(self.tick, j, status))?;
            }
        }
        let tx_ns = runtime::now().as_nanos() as u64;
        // A queued current-limit change precedes this tick's motion command.
        // The bus still sends one poll/configuration frame per tick.
        self.bus.poll_step()?;
        self.bus.send_joint_commands(&commands)?;
        self.bus.send_gripper(&GripperCommand::NoGripper)?;
        self.emit(Event::Sample(
            self.tick,
            rx_ns,
            tx_ns,
            commands,
            std::array::from_fn(|j| self.state.nodes[self.node(j)].position_ticks.unwrap_or(0)),
            std::array::from_fn(|j| self.state.nodes[self.node(j)].speed_ticks_s.unwrap_or(0)),
            std::array::from_fn(|j| self.state.nodes[self.node(j)].current_ma.unwrap_or(0)),
            self.generation,
            // Distinguishes "the drive reported a fault" from "the drive did
            // not answer": on 2026-09-19 only the second was recorded, and the
            // two could not be told apart afterwards.
            std::array::from_fn(|j| self.state.nodes[self.node(j)].live_error_bit),
        ))
    }
    fn initialize(&mut self) -> Result<()> {
        let mut holding = false;
        for _ in 0..self.ticks(2.0) {
            if holding {
                self.frame(None)?;
            } else {
                self.exchange([JointCommand::encoder_poll(); N], false)?;
            }
            if !holding && (0..N).all(|j| self.generation[j] != 0) {
                for j in 0..N {
                    self.hold[j] = self.pos(j)?;
                }
                self.found_at = self.hold;
                holding = true;
            }
            // crates/par6-bus/src/hw/mod.rs::apply_payload: encoder replies
            // omit current; position-hold replies supply all motion feedback.
            if holding
                && (0..N).all(|j| {
                    let state = &self.state.nodes[self.node(j)];
                    state.current_ma.is_some() && state.speed_ticks_s.is_some()
                })
            {
                self.ramp_startup_current()?;
                return self.check_fitted_tool();
            }
        }
        Err("not all six drives supplied position, velocity and current feedback".into())
    }
    /// Does the bus agree with the config about what is on the flange?
    ///
    /// Full gripper identification is not available: the drive's device
    /// info carries hardware/firmware version and a serial number that
    /// reads back as 1 on every node here, and nothing naming the model.
    /// What the bus does settle is whether a gripper driver answers at
    /// all, which separates a driven tool from a passive one. Getting
    /// that wrong silently is expensive — the tool's mass is most of the
    /// wrist's gravity load, so calibrating with the config describing a
    /// tool that is not fitted (or missing one that is) produces a model
    /// that is wrong everywhere.
    fn check_fitted_tool(&mut self) -> Result<()> {
        if self.simulated {
            return Ok(());
        }
        let node = self.bundle.robot.bus.gripper_node;
        let declared = self.bundle.active_gripper().is_some_and(|g| g.driver.is_some());
        // The round-robin reaches device info only every few seconds, so
        // ask directly rather than waiting for its turn.
        for _ in 0..3 {
            self.bus.queue_poll_override(
                PollAction::Poll {
                    node,
                    kind: PollKind::DeviceInfo,
                },
                1,
            );
            for _ in 0..self.ticks(0.05) {
                self.frame(None)?;
            }
        }
        let found = self.state.nodes[usize::from(node)].device_info;
        self.emit(Event::ToolDetected(node, found))?;
        // Only the mismatch paths format, and they end the run.
        let tool = &self.bundle.robot.robot.active_gripper;
        match (declared, found) {
            (true, Some(_)) | (false, None) => Ok(()),
            (true, None) => Err(format!(
                "config names `{tool}`, a tool with a CAN driver, but no gripper driver answered on node {node}.                  Fit it, or select a passive tool such as `Flange`."
            )
            .into()),
            (false, Some(info)) => Err(format!(
                "config names `{tool}`, a passive tool, but a gripper driver answered on node {node}                  (hw {}, fw {}). Select the tool that is fitted: its mass is most of the wrist's gravity load.",
                info.hw_ver, info.sw_ver
            )
            .into()),
        }
    }
    fn ramp_startup_current(&mut self) -> Result<()> {
        // Apply cubic interpolation to the current-cap envelope (not a claim
        // that measured current follows that envelope). Polynomial source:
        // https://modernrobotics.northwestern.edu/nu-gm-book-resource/9-1-and-9-2-point-to-point-trajectories-part-2-of-2/
        // Reuse the existing vendor-derived jog ramp duration; the paper does
        // not prescribe a PAR6 startup duration. Duration provenance:
        // https://github.com/Source-Robotics/RCB-Runtime/blob/main/config/system.xml
        // One limit frame per tick,
        // round-robin across six joints, preserves the existing CAN budget.
        let sweeps = self
            .ticks(self.bundle.robot.jog.accel_time_s)
            .div_ceil(N as u64);
        for j in 0..N {
            self.emit(Event::Phase("ramp startup holding current", j))?;
        }
        for step in 1..=sweeps {
            let u = step as f64 / sweeps as f64;
            let fraction = u * u * (3.0 - 2.0 * u);
            for j in 0..N {
                self.cache_tune(j, self.bundle.robot.joints[j].ilim_ma * fraction)?;
                self.bus.queue_poll_override(
                    PollAction::ConfigFrame {
                        node: self.bundle.robot.joints[j].node_id,
                        kind: ConfigKind::Limits,
                    },
                    1,
                );
                self.frame(None)?;
            }
        }
        Ok(())
    }
    fn prepare_holding(&mut self) -> Result<()> {
        self.quality_limits
            .ok_or("startup holding needs quality limits")?;
        loop {
            let measured = self.measure_hold("startup acceptance", None)?;
            let holding: [f64; N] = std::array::from_fn(|j| self.holding_limit(j));
            let tolerance = self.bundle.robot.motion.settle_tolerance_rad;
            let failed = measured.iter().enumerate().position(|(j, quality)| {
                !quality.score(holding[j], tolerance).accepted()
            });
            let Some(j) = failed else {
                self.startup_verified = true;
                return Ok(());
            };
            // User requirement: correct a failed startup hold before homing.
            // Reuse the constrained Nelder-Mead search on the actual position
            // hold, with all three cascade gains as variables. This only
            // establishes holding quality; homing still tests moving response.
            // https://pymoo.org/constraints/feas_first.html
            // https://www.mathworks.com/help/optim/ug/fminsearch-algorithm.html
            self.gain_stage = Some(GainStage::StartupHold);
            let origin = self.hold[j];
            let result = self.tune_stage(j, origin, origin, self.dt, None);
            self.gain_stage = None;
            result?;
        }
    }
    fn frame(&mut self, active: Option<(usize, JointCommand)>) -> Result<()> {
        let mut commands = self.hold.map(|p| JointCommand::position(p, 0, 0));
        if let Some((j, cmd)) = active {
            commands[j] = cmd;
        }
        self.exchange(commands, true)
    }
    fn measure_hold(
        &mut self,
        purpose: &'static str,
        velocity_joint: Option<usize>,
    ) -> Result<[HoldQuality; N]> {
        let mut measured = [HoldQuality::default(); N];
        let mut generation = self.generation;
        // This is an RMS observation window, not a settling delay. Every fresh
        // sample contributes; a single zero-speed crossing cannot establish quiet.
        // The explicit CLI observation interval is separate from the settling timeout.
        let limits = self
            .quality_limits
            .ok_or("holding measurement needs quality limits")?;
        for j in 0..N {
            let position = self.pos(j)?;
            self.hold_velocity[j].reset(self.feedback_ns[j], position);
        }
        for _ in 0..self.ticks(limits.hold_observation_s) {
            self.frame(velocity_joint.map(|j| (j, JointCommand::velocity(0, 0))))?;
            for j in 0..N {
                if generation[j] == self.generation[j] {
                    continue;
                }
                generation[j] = self.generation[j];
                let position = self.pos(j)?;
                let speed = self.hold_velocity[j].push(self.feedback_ns[j], position)?;
                let state = &self.state.nodes[self.node(j)];
                let per_tick = self.conv[j].joint_rad(1) - self.conv[j].joint_rad(0);
                measured[j].push(
                    (f64::from(position) - f64::from(self.hold[j])) * per_tick,
                    speed.map(|speed| speed * per_tick),
                    f64::from(state.speed_ticks_s.ok_or("missing hold velocity")?) * per_tick,
                    f64::from(state.current_ma.ok_or("missing hold current")?),
                );
            }
        }
        for (j, quality) in measured.iter().enumerate() {
            self.emit(Event::HoldQuality(self.tick, j, purpose, *quality))?;
        }
        Ok(measured)
    }
    fn cache_tune(&mut self, j: usize, ilim: f64) -> Result<()> {
        let cfg = &self.bundle.robot.joints[j];
        let node = cfg.node_id;
        let tune = DriveTune {
            gains: self.gains[j],
            ilim_ma: ilim,
            velocity_limit_ticks_s: cfg.velocity_limit_ticks_s,
            voltage_limit_mv: cfg.voltage_limit_mv,
        };
        self.bus.retune_node(node, &tune, 0)?;
        self.emit(Event::Configure(self.tick, j, ilim, self.pos(j)?))
    }
    fn configure(&mut self, j: usize, ilim: f64) -> Result<()> {
        self.cache_tune(j, ilim)?;
        let node = self.bundle.robot.joints[j].node_id;
        for _ in 0..3 {
            for kind in [
                ConfigKind::Limits,
                ConfigKind::VelocityGains,
                ConfigKind::PositionGains,
            ] {
                self.bus
                    .queue_poll_override(PollAction::ConfigFrame { node, kind }, 1);
                self.frame(None)?;
            }
        }
        Ok(())
    }
    fn queue_current_limit(&mut self, j: usize, limit: f64) -> Result<()> {
        // DriverBus::retune_node(repeats=0) + queue_poll_override in
        // crates/par6-bus/src/bus.rs: update the cache, then use paced CAN slots.
        self.cache_tune(j, limit)?;
        self.bus.queue_poll_override(
            PollAction::ConfigFrame {
                node: self.bundle.robot.joints[j].node_id,
                kind: ConfigKind::Limits,
            },
            3,
        );
        Ok(())
    }
    fn position_motion(&self, to: i32, seconds: f64) -> Motion {
        // User requirement: use requested slow motion timing; determining velocity,
        // acceleration and jerk limits belongs to a later optional calibration stage.
        Motion::Position(to, seconds.max(self.dt))
    }
    fn acceptable(&self, m: Measurement, motion: Motion) -> bool {
        // User correction: position lag is diagnostic in velocity mode, not
        // its acceptance criterion. STEPFOC's velocity loop tracks speed;
        // position regulation belongs to the outer loop:
        // https://source-robotics.github.io/STEPFOC-docs/PID_tuning/
        (!matches!(motion, Motion::Position(..))
            || m.settled_error_rad <= self.bundle.robot.motion.settle_tolerance_rad)
            && self.quality_limits.is_none_or(|limits| {
                let _ = limits;
                m.moving_ratio() <= 1.0 && m.holding_ratio() <= 1.0
            })
    }
    fn setup_motion(&mut self, j: usize, motion: Motion) -> Result<Measurement> {
        // Returning to the experiment origin is setup, not another scored trial.
        // Same initial conditions: Hjalmarsson et al., p.31:
        // https://perso.uclouvain.be/michel.gevers/PublisMig/IFT-CSMagazine98.pdf
        let limits = self.quality_limits.take();
        let result = self.motion(j, motion, false);
        self.quality_limits = limits;
        result
    }
    fn motion(&mut self, j: usize, motion: Motion, watch_tracking: bool) -> Result<Measurement> {
        let start = self.pos(j)?;
        let radians_per_tick = self.conv[j].joint_rad(1) - self.conv[j].joint_rad(0);
        // User requirement: no blind waits for an already achieved endpoint.
        // Same position tolerance as normal completion; observe the hold instead
        // of running an effectively stationary trajectory for its entire duration.
        let already_at_target = matches!(motion, Motion::Position(target, _) | Motion::VelocityProfile(target, _)
            if ((f64::from(target) - f64::from(start)) * radians_per_tick).abs()
                <= self.bundle.robot.motion.settle_tolerance_rad);
        let motion = if already_at_target {
            self.emit(Event::Phase("target already reached; measure holding", j))?;
            if matches!(motion, Motion::VelocityProfile(..)) {
                Motion::VelocityProfile(start, self.dt)
            } else {
                Motion::Position(start, self.dt)
            }
        } else {
            motion
        };
        self.emit(Event::Phase(
            match motion {
                Motion::Position(..) => "position move",
                Motion::VelocityProfile(..) => "velocity replay of position trajectory",
                Motion::Velocity(..) => "velocity move",
                Motion::Seek(_, true) => "Hall seek",
                Motion::Seek(_, false) => "endstop seek",
            },
            j,
        ))?;
        let (mut duration, speed, hall) = match motion {
            Motion::Position(_, seconds) | Motion::VelocityProfile(_, seconds) => (
                seconds + self.bundle.robot.motion.settle_timeout_s,
                0,
                false,
            ),
            Motion::Seek(v, hall) => (
                self.bundle.robot.homing.joints[j].seek_timeout_s(&self.bundle.robot.joints[j]),
                v,
                hall,
            ),
            Motion::Velocity(v, seconds) => (seconds, v, false),
        };
        // Vendor jog startup duration (config/PAR6.toml [jog]), used as a ramp time
        // at the requested homing speed, not an acceleration acceptance threshold.
        // https://github.com/Source-Robotics/RCB-Runtime/blob/main/config/system.xml
        let ramp = match motion {
            // A bounded velocity move has area speed * seconds under its profile;
            // shortening the ramp preserves that distance when seconds < ramp time.
            Motion::Velocity(_, seconds) => self.bundle.robot.jog.accel_time_s.min(seconds),
            _ => self.bundle.robot.jog.accel_time_s,
        }
        .max(self.dt);
        let moving_until = match motion {
            Motion::Position(_, seconds) | Motion::VelocityProfile(_, seconds) => seconds,
            Motion::Velocity(_, seconds) => seconds.max(ramp) + ramp,
            Motion::Seek(..) => duration,
        };
        if matches!(motion, Motion::Velocity(..)) {
            duration = moving_until + self.bundle.robot.motion.settle_timeout_s;
        }
        let tolerance = self.bundle.robot.motion.settle_tolerance_rad;
        let direction = match motion {
            Motion::Position(target, _) | Motion::VelocityProfile(target, _) => {
                (i64::from(target) - i64::from(start)).signum() as i32
            }
            Motion::Seek(speed, _) | Motion::Velocity(speed, _) => speed.signum(),
        };
        // User requirement: every unreferenced homing/tuning leg toward the stop
        // uses the fixed homing current; away and holding use full operating current.
        // This includes gain-test resets and both velocity-test directions.
        let approach = self.adaptive
            && !self.homed[j]
            && direction
                == if self.bundle.robot.homing.joints[j].direction == 1 {
                    -1
                } else {
                    1
                };
        let operating_limit = self.bundle.robot.joints[j].ilim_ma;
        let limit = if approach {
            self.bundle.robot.homing.joints[j].current_ma
        } else {
            operating_limit
        };
        // Peak commanded joint speed of this leg: quintic peak is 1.875 x
        // distance / duration (reference above); ramped legs peak at the request.
        let peak_command_rad_s = match motion {
            Motion::Position(target, seconds) | Motion::VelocityProfile(target, seconds) => {
                1.875 * ((f64::from(target) - f64::from(start)) * radians_per_tick).abs() / seconds
            }
            Motion::Seek(speed, _) | Motion::Velocity(speed, _) => {
                (f64::from(speed) * radians_per_tick).abs()
            }
        };
        let mut out = Measurement {
            direction,
            peak_command_rad_s,
            moving_limit_rad_s: self
                .quality_limits
                .map_or(f64::INFINITY, |limits| limits.moving_limit(peak_command_rad_s)),
            holding_limit_rad_s: self.holding_limit(j),
            ..Measurement::default()
        };
        let mut expected = f64::from(start);
        let mut previous_velocity = 0;
        let mut generation = self.generation[j];
        let mut window_start = 0;
        let mut window_min = start;
        let mut window_max = start;
        let mut window_expected = expected;
        let mut high_current = 0;
        let mut observations = 0;
        // Vendor detection window/startup guard/current fraction/occupancy/travel floor:
        // https://github.com/Source-Robotics/RCB-Runtime/blob/main/robotics/homing.py
        // Constants and behavior only; these detect contact, not acceptable tracking.
        let window = self.ticks(0.08);
        // The vendor startup guard excludes acceleration transients. Our velocity
        // profile has an explicit ramp: begin the observation window after it ends.
        // This also gives a reversal time to unwind the existing velocity integral.
        // V108 retains that integral across motion commands:
        // https://github.com/Source-Robotics/STEPFOC-stepper-controller/blob/V108/STEPFOC%20firmware/src/communication_CAN.cpp
        let observe_after = self.ticks(match motion {
            Motion::Position(..) | Motion::VelocityProfile(..) => 0.15,
            _ => ramp.max(0.15),
        });
        if hall {
            self.frame(Some((j, JointCommand::position(start, 0, 0))))?;
        }
        self.queue_current_limit(j, limit)?;
        self.emit(Event::MotionStart(
            self.tick + 1,
            j,
            motion,
            start,
            limit,
            observe_after as f64 * self.dt,
            approach,
        ))?;
        for t in 0..self.ticks(duration) {
            let feedback_expected = expected;
            let feedback_velocity = previous_velocity;
            let cmd = match motion {
                Motion::Position(target, seconds) | Motion::VelocityProfile(target, seconds) => {
                    // Quintic time scaling, with zero endpoint velocity/acceleration;
                    // peak speed is 1.875 times distance/duration.
                    // https://modernrobotics.northwestern.edu/nu-gm-book-resource/9-1-and-9-2-point-to-point-trajectories-part-2-of-2/
                    let u = ((t + 1) as f64 * self.dt / seconds).min(1.0);
                    let distance = f64::from(target) - f64::from(start);
                    expected =
                        f64::from(start) + distance * u.powi(3) * (10.0 - 15.0 * u + 6.0 * u * u);
                    let velocity = (distance * 30.0 * u * u * (1.0 - u).powi(2) / seconds) as i32;
                    // Tune the inner loop with the actual transfer's velocity
                    // profile, including its speed and acceleration. Realistic
                    // service motion: https://www.kollmorgen.com/en-us/developer-network/akd-online-tuning-guide
                    if matches!(motion, Motion::VelocityProfile(..)) {
                        JointCommand::velocity(velocity, 0)
                    } else {
                        JointCommand::position(expected.round() as i32, velocity, 0)
                    }
                }
                Motion::Seek(..) | Motion::Velocity(..) => {
                    // Cubic time scaling s=3u²-2u³ from the same reference above,
                    // here interpolating velocity with zero slope at both endpoints.
                    let time = (t + 1) as f64 * self.dt;
                    let mut u = (time / ramp).min(1.0);
                    if matches!(motion, Motion::Velocity(..)) {
                        u = u.min(((moving_until - time) / ramp).clamp(0.0, 1.0));
                    }
                    let v = (f64::from(speed) * u * u * (3.0 - 2.0 * u)) as i32;
                    expected += f64::from(v) * self.dt;
                    if hall {
                        JointCommand::hall(v, 2)
                    } else {
                        JointCommand::velocity(v, 0)
                    }
                }
            };
            let node = self.node(j);
            self.state.nodes[node].hall = None;
            self.frame(Some((j, cmd)))?;
            previous_velocity = cmd.vel.unwrap_or(0);
            if self.generation[j] == generation {
                continue;
            }
            generation = self.generation[j];
            let p = self.pos(j)?;
            out.elapsed_s = (t + 1) as f64 * self.dt;
            let current = f64::from(
                self.state.nodes[node]
                    .current_ma
                    .ok_or("missing current feedback")?,
            );
            let actual_speed = self.state.nodes[node]
                .speed_ticks_s
                .ok_or("missing velocity feedback")?;
            let error = (f64::from(p) - feedback_expected) * radians_per_tick;
            // STEPFOC feedback is measured Iq, motor ticks and filtered motor ticks/s.
            // https://github.com/Source-Robotics/STEPFOC-stepper-controller/blob/32fb5b5594e47930df7e23309043468ed5a81ed4/STEPFOC%20firmware/src/communication_CAN.cpp
            // Kissling et al. §3.1: RMS tracking error, used to rank repeat experiments.
            // https://doi.org/10.1016/j.conengprac.2009.02.005
            // An already-reached endpoint is held from the first frame, so that
            // first fresh zero-command-speed sample is valid for its measurement.
            if t > 0 || already_at_target {
                out.peak_current_ma = out.peak_current_ma.max(current.abs());
                out.peak_error_rad = out.peak_error_rad.max(error.abs());
                // The RMS starts after the same guard window the detector
                // uses: the ramp and, after a stall, the integrator unwind are
                // transients of the previous command, not of these gains.
                if (t >= observe_after && t < self.ticks(moving_until)) || already_at_target {
                    let speed_error =
                        (f64::from(actual_speed) - f64::from(feedback_velocity)) * radians_per_tick;
                    out.samples += 1;
                    out.position_squared_rad += error * error;
                    out.velocity_squared_rad_s += speed_error * speed_error;
                    out.current_squared_ma += current * current;
                }
            }
            if hall
                && self.state.nodes[node]
                    .hall
                    .is_some_and(|s| !s.trigger && s.edge)
            {
                out.outcome = Outcome::Complete;
                out.stop = Some(p);
                break;
            }
            if let Motion::Position(target, seconds) = motion {
                if t >= self.ticks(seconds)
                    && ((f64::from(p) - f64::from(target)) * radians_per_tick).abs() <= tolerance
                {
                    out.outcome = Outcome::Complete;
                    out.stop = Some(p);
                    break;
                }
            }
            if matches!(motion, Motion::Velocity(..) | Motion::VelocityProfile(..))
                && t >= self.ticks(moving_until)
            {
                // The velocity experiment ends after its commanded ramp to zero.
                // Its measured holding response is checked below; velocity mode
                // cannot correct an accumulated position offset after that ramp.
                out.outcome = Outcome::Complete;
                out.stop = Some(p);
                break;
            }
            if t < observe_after {
                window_start = t;
                window_min = p;
                window_max = p;
                window_expected = feedback_expected;
                continue;
            }
            observations += 1;
            high_current += usize::from(current.abs() >= STALL_CURRENT_FRACTION * limit);
            window_min = window_min.min(p);
            window_max = window_max.max(p);
            if t.saturating_sub(window_start) < window {
                continue;
            }
            let requested = (feedback_expected - window_expected).abs();
            // Observe the whole position range so back-and-forth movement is not
            // mistaken for standstill merely because its net displacement is zero.
            let encoder_range = i64::from(window_max) - i64::from(window_min);
            let stall_below = (requested * 0.25).max(10.0);
            let stopped = (encoder_range as f64) < stall_below;
            let loaded = high_current as f64 >= 0.6 * observations as f64;
            // Contact takes priority over reserve and tracking. No minimum prior travel.
            let at_target = matches!(motion, Motion::Position(target, _)
                if ((f64::from(p) - f64::from(target)) * radians_per_tick).abs() <= tolerance)
                || matches!(motion, Motion::Velocity(..) | Motion::VelocityProfile(..))
                    && t >= self.ticks(moving_until);
            // Require motion now, not just earlier in a window ending at contact.
            // Use the same vendor 25%-of-commanded-speed stall criterion above.
            let advancing = f64::from(actual_speed) * f64::from(feedback_velocity.signum())
                > f64::from(feedback_velocity).abs() * 0.25;
            let position_tracking_enabled = watch_tracking
                && self.adaptive
                && !self.baseline_only
                && matches!(motion, Motion::Position(..));
            let decision = if loaded && stopped && requested >= 10.0 && !at_target {
                Some(Outcome::Blocked)
            // Position lag is commanded minus measured position:
            // https://infosys.beckhoff.com/content/1033/axispos/15310823051.html
            // Apply the configured position tolerance only to position commands,
            // and only when the joint is not following at all (below the vendor
            // 25%-of-command speed criterion): lag while advancing is measured
            // by the settled error and the speed RMS, not aborted.
            // User correction: velocity-mode homing must not be interrupted by
            // accumulated position lag; stall/Hall detection supplies its reference.
            } else if position_tracking_enabled && !loaded && !advancing && error.abs() > tolerance
            {
                Some(Outcome::Tracking)
            } else {
                None
            };
            self.emit(Event::Detection(
                j,
                Detection {
                    tick: self.tick,
                    elapsed_s: out.elapsed_s,
                    window_s: t.saturating_sub(window_start) as f64 * self.dt,
                    position_ticks: p,
                    encoder_range_ticks: encoder_range,
                    requested_ticks: requested,
                    stall_below_ticks: stall_below,
                    current_ma: current,
                    current_threshold_ma: STALL_CURRENT_FRACTION * limit,
                    limit_ma: limit,
                    high_current_samples: high_current,
                    fresh_samples: observations,
                    commanded_ticks_s: feedback_velocity,
                    encoder_ticks_s: actual_speed,
                    tracking_error_deg: error.to_degrees(),
                    stopped,
                    loaded,
                    advancing,
                    at_target,
                    position_tracking_enabled,
                    decision,
                },
            ))?;
            window_start = t;
            window_min = p;
            window_max = p;
            window_expected = feedback_expected;
            observations = 0;
            high_current = 0;
            if let Some(decision) = decision {
                out.outcome = decision;
                if decision == Outcome::Blocked {
                    out.stop = Some(p);
                }
                break;
            }
        }
        self.hold[j] = self.pos(j)?;
        if approach {
            self.queue_current_limit(j, operating_limit)?;
        }
        let velocity_hold = matches!(motion, Motion::Velocity(..) | Motion::VelocityProfile(..))
            && matches!(
                self.gain_stage,
                Some(GainStage::Velocity | GainStage::VelocityI)
            );
        self.frame(velocity_hold.then_some((j, JointCommand::velocity(0, 0))))?;
        out.worst_velocity_rms_rad_s = out.rms()[1];
        let mut passive_failure: Option<(usize, HoldQuality)> = None;
        // A referenced endpoint alone says nothing about settling. Observe this
        // joint at zero speed in the loop being tuned before accepting its move.
        // https://webhelp.kollmorgen.com/akd2g/english/content/AKD2G_User_Manual/Autotuner_Advanced.htm
        // Seeks retain their stall/Hall semantics; endstop current is expected.
        if self.quality_limits.is_some()
            && !self.stopping
            && out.outcome == Outcome::Complete
            && !matches!(motion, Motion::Seek(..))
        {
            let holds = self.measure_hold(
                if velocity_hold {
                    "velocity-loop zero-speed response"
                } else {
                    "position holding response"
                },
                velocity_hold.then_some(j),
            )?;
            let hold = holds[j];
            out.hold_velocity_rms_rad_s = hold.rms()[1];
            out.peak_error_rad = out.peak_error_rad.max(hold.peak_error_rad);
            out.settled_error_rad = hold.peak_error_rad;
            out.peak_current_ma = out.peak_current_ma.max(hold.peak_current_ma);
            if !self.acceptable(out, motion) {
                out.outcome = Outcome::Tracking;
            }
            // A passive joint's failed measurement must not be discarded.
            // User requirement: never start tuning an unrelated joint during
            // this joint's experiment. Stop with its measured failed criterion,
            // naming the worst passive joint, after this joint's own record has
            // been written. While the active joint itself fails its hold, a
            // passive joint's flicker is that shaking seen through the arm, not
            // independent evidence: the active joint's failure is what to act on.
            if self.startup_verified && out.outcome != Outcome::Tracking {
                let holding: [f64; N] = std::array::from_fn(|other| self.holding_limit(other));
                passive_failure = holds
                    .iter()
                    .enumerate()
                    .filter(|(other, quality)| {
                        *other != j
                            && passive_hold_failed(**quality, holding[*other], tolerance)
                    })
                    .max_by(|a, b| a.1.peak_error_rad.total_cmp(&b.1.peak_error_rad))
                    .map(|(other, quality)| (other, *quality));
            }
            // Capture the actual endpoint after a velocity-mode observation;
            // switching back to position mode must not snap to its old start.
            self.hold[j] = self.pos(j)?;
            self.frame(None)?;
        }
        self.emit(Event::Motion(
            self.tick,
            j,
            start,
            self.hold[j],
            radians_per_tick,
            out,
        ))?;
        if let Some((other, quality)) = passive_failure {
            return Err(format!(
                "J{} did not hold position after J{} move: peak position error={:.6}deg (limit {:.6}); speed RMS={:.6}deg/s (quiet below {:.6}); J{} own hold speed RMS={:.6}deg/s outcome={:?}",
                other + 1, j + 1, quality.peak_error_rad.to_degrees(), tolerance.to_degrees(),
                quality.rms()[1].to_degrees(), self.holding_limit(other).to_degrees(),
                j + 1, out.hold_velocity_rms_rad_s.to_degrees(), out.outcome
            ).into());
        }
        Ok(out)
    }
    fn adaptive_motion(
        &mut self,
        j: usize,
        request: Motion,
        reference: Option<i32>,
    ) -> Result<Measurement> {
        let origin = self.pos(j)?;
        let h = self.bundle.robot.homing.joints[j].clone();
        let (target, seconds, direction) = match request {
            Motion::Position(target, seconds) => (
                target,
                seconds,
                (i64::from(target) - i64::from(origin)).signum() as i32,
            ),
            Motion::Seek(speed, _) => (
                origin.saturating_add((f64::from(speed) * h.backoff_s) as i32),
                h.backoff_s * 1.875,
                speed.signum(),
            ),
            Motion::Velocity(..) | Motion::VelocityProfile(..) => {
                unreachable!("velocity experiments are bounded inside gain tuning")
            }
        };
        let confirming_stop = reference.is_some() && matches!(request, Motion::Seek(_, false));
        let mut pending = None;
        let mut seek_ticks = 0;
        loop {
            if matches!(request, Motion::Seek(..))
                && seek_ticks >= self.ticks(h.seek_timeout_s(&self.bundle.robot.joints[j]))
            {
                return Err(format!(
                    "J{} home reference not found within configured seek time",
                    j + 1
                )
                .into());
            }
            let motion = match request {
                Motion::Position(..) => {
                    let remaining = (f64::from(target) - f64::from(self.pos(j)?)).abs();
                    let distance = (f64::from(target) - f64::from(origin)).abs();
                    self.position_motion(target, seconds * remaining / distance.max(1.0))
                }
                other => other,
            };
            let measured = match pending.take() {
                Some(m) => m,
                None => {
                    let began = self.tick;
                    // A return to an observed stop checks that reference at its seek
                    // current. User requirement: endstop load must not trigger reserve tuning.
                    let measured = self.motion(j, motion, !confirming_stop)?;
                    seek_ticks += self.tick - began;
                    measured
                }
            };
            match measured.outcome {
                Outcome::Complete
                    if self.acceptable(measured, request)
                        || self.baseline_only
                        || matches!(request, Motion::Seek(..)) =>
                {
                    return Ok(measured);
                }
                Outcome::Blocked => {
                    let contact = self.pos(j)?;
                    let blocked_direction = measured.direction;
                    if matches!(request, Motion::Seek(_, false))
                        && blocked_direction == direction
                        && reference.is_some_and(|p| {
                            (i64::from(p) - i64::from(contact)).abs()
                                <= i64::from(h.two_pass_max_diff_ticks)
                        })
                    {
                        return Ok(measured);
                    }
                    let reverse_target = contact.saturating_sub(
                        (f64::from(blocked_direction) * h.speed_ticks_s * h.backoff_s) as i32,
                    );
                    self.emit(Event::Phase(
                        "check opposite direction using its configured current",
                        j,
                    ))?;
                    let reverse = self.position_motion(reverse_target, h.backoff_s * 1.875);
                    // Vendor homing.py _do_backoff uses velocity mode. A slow
                    // position ramp can report a stall before reverse torque builds.
                    // https://github.com/Source-Robotics/RCB-Runtime/blob/main/robotics/homing.py
                    let reverse_speed = (-f64::from(blocked_direction) * h.speed_ticks_s) as i32;
                    let backoff = Motion::Velocity(reverse_speed, h.backoff_s);
                    let back = self.motion(j, backoff, false)?;
                    let reverse_travel = (i64::from(contact) - i64::from(self.pos(j)?))
                        * i64::from(blocked_direction);
                    let reverse_minimum = (h.speed_ticks_s * 0.08 * 0.25).max(10.0) as i64;
                    let moved_back = reverse_travel >= reverse_minimum;
                    self.emit(Event::ReverseCheck(
                        self.tick,
                        j,
                        contact,
                        self.pos(j)?,
                        reverse_travel,
                        reverse_minimum,
                    ))?;
                    if back.outcome == Outcome::Blocked && !moved_back {
                        return Err(format!(
                            "J{} cannot move either direction at the configured homing/operating currents",
                            j + 1
                        ).into());
                    }
                    if moved_back {
                        // User's diagnostic: moving away makes this a stop candidate.
                        // The second slow approach checks the reference before it is latched.
                        if back.outcome == Outcome::Blocked {
                            return Err(format!(
                                "J{} reverse move blocked at its configured current",
                                j + 1
                            )
                            .into());
                        }
                        if !self.acceptable(back, backoff) {
                            let Motion::Position(target, seconds) = reverse else {
                                unreachable!()
                            };
                            if let Some(blocked) = self.tune_motion(
                                j,
                                contact,
                                target,
                                seconds,
                                Some(VelocityTrial {
                                    speed: reverse_speed,
                                    seconds: h.backoff_s,
                                    return_stop: (h.strategy != HomingStrategy::Hall
                                        && blocked_direction
                                            == if h.direction == 1 { -1 } else { 1 })
                                    .then_some(contact),
                                }),
                            )? {
                                pending = Some(blocked);
                                continue;
                            }
                        }
                        if back.outcome != Outcome::Complete
                            || (self.conv[j].joint_rad(self.pos(j)?)
                                - self.conv[j].joint_rad(reverse_target))
                            .abs()
                                > self.bundle.robot.motion.settle_tolerance_rad
                        {
                            self.adaptive_motion(j, reverse, None)?;
                        }
                        if matches!(request, Motion::Seek(_, false))
                            && blocked_direction == direction
                        {
                            return Ok(measured);
                        }
                        if matches!(request, Motion::Position(..)) && blocked_direction == direction
                        {
                            return Err(format!(
                                "J{} requested transfer encounters a stop; reverse motion works",
                                j + 1
                            )
                            .into());
                        }
                        // J6 must still reach its Hall event; a stall never supplies its home.
                        continue;
                    }
                    let seek = if let Motion::Seek(speed, _) = request {
                        Some(VelocityTrial {
                            speed,
                            seconds: measured.elapsed_s,
                            return_stop: None,
                        })
                    } else {
                        None
                    };
                    pending = self.tune_motion(j, origin, target, seconds, seek)?;
                }
                Outcome::Timeout
                    if matches!(request, Motion::Seek(..))
                        && (confirming_stop || self.acceptable(measured, request)) =>
                {
                    return Err(format!(
                        "J{} no home event arrived within configured seek time",
                        j + 1
                    )
                    .into());
                }
                _ => {
                    let seek = if let Motion::Seek(speed, _) = request {
                        Some(VelocityTrial {
                            speed,
                            seconds: measured.elapsed_s,
                            return_stop: None,
                        })
                    } else {
                        None
                    };
                    pending = self.tune_motion(j, origin, target, seconds, seek)?;
                }
            }
        }
    }
    fn tune_motion(
        &mut self,
        j: usize,
        origin: i32,
        target: i32,
        seconds: f64,
        seek: Option<VelocityTrial>,
    ) -> Result<Option<Measurement>> {
        // Cascade order from STEPFOC's guide: velocity P alone, then velocity
        // I alone with P fixed (both judged on the movement and the zero-speed
        // holding requirement of every velocity-mode leg), then position P
        // with those PI gains fixed.
        // https://source-robotics.github.io/STEPFOC-docs/PID_tuning/
        // When no Kpv passes at the current Kiv, move to the least-violating
        // Kpv, search Ki there, then search Kpv once more (one round of
        // coordinate descent).
        let result = (|| {
            let stage = |arm: &mut Self, stage| -> Result<Option<Option<Measurement>>> {
                arm.gain_stage = Some(stage);
                Ok(match arm.tune_stage(j, origin, target, seconds, seek)? {
                    Some(blocked) => Some(Some(blocked)),
                    None if arm.no_band.is_some() => None,
                    None => Some(None),
                })
            };
            let no_band = |arm: &mut Self, stage: GainStage| -> Result<()> {
                Err(format!(
                    "J{} {stage:?}: no gain scale within the bounds met the requirements; least violation at Kpv={:?} Kiv={:?} Kpp={:?}",
                    j + 1, arm.gains[j].kpv, arm.gains[j].kiv, arm.gains[j].kpp
                )
                .into())
            };
            let mut ki_searched = false;
            let mut velocity = stage(self, GainStage::Velocity)?;
            if velocity.is_none() {
                let best = self.no_band.ok_or("missing least-violating scale")?;
                self.scale_gains(j, GainStage::Velocity, best);
                self.emit(Event::Phase(
                    "no passing Kpv; search Ki at the least-violating Kpv",
                    j,
                ))?;
                match stage(self, GainStage::VelocityI)? {
                    Some(Some(blocked)) => return Ok(Some(blocked)),
                    Some(None) => {}
                    None => {
                        let best = self.no_band.ok_or("missing least-violating scale")?;
                        self.scale_gains(j, GainStage::VelocityI, best);
                    }
                }
                ki_searched = true;
                velocity = stage(self, GainStage::Velocity)?;
            }
            match velocity {
                Some(Some(blocked)) => return Ok(Some(blocked)),
                Some(None) => {}
                None => return no_band(self, GainStage::Velocity).map(|()| None),
            }
            if !ki_searched {
                match stage(self, GainStage::VelocityI)? {
                    Some(Some(blocked)) => return Ok(Some(blocked)),
                    Some(None) => {}
                    None => return no_band(self, GainStage::VelocityI).map(|()| None),
                }
            }
            match stage(self, GainStage::Position)? {
                Some(Some(blocked)) => Ok(Some(blocked)),
                Some(None) => Ok(None),
                None => no_band(self, GainStage::Position).map(|()| None),
            }
        })();
        self.gain_stage = None;
        result
    }
    fn tune_stage(
        &mut self,
        j: usize,
        origin: i32,
        target: i32,
        seconds: f64,
        seek: Option<VelocityTrial>,
    ) -> Result<Option<Measurement>> {
        let stage = self.gain_stage.ok_or("missing gain tuning stage")?;
        let limits = self
            .quality_limits
            .ok_or("gain tuning needs explicit quality limits")?;
        if self.baseline_only || self.focused_joint.is_some_and(|selected| selected != j) {
            return Err(format!(
                "J{} needs gain tuning; this diagnostic run disables it",
                j + 1
            )
            .into());
        }
        // The budget is per stage: a joint that is tuned again later in the
        // sequence gets a full search, not the remainder of an earlier one.
        let budget = self.trials;
        self.no_band = None;
        self.emit(Event::Phase(
            if stage == GainStage::StartupHold {
                "tune startup position hold"
            } else {
                "tune gains during homing"
            },
            j,
        ))?;
        let initial = self.gains[j];
        let start = self.start_gains[j];
        let search = self.search;
        let tolerance = self.bundle.robot.motion.settle_tolerance_rad;
        // Every gain stays within the ceiling of the run's starting value,
        // however many episodes tune this joint. The bounds are expressed as
        // scales of this episode's starting gains.
        let bound = |current: f64, origin: f64| -> (f64, f64) {
            (origin / (search.ceiling * current), search.ceiling * origin / current)
        };
        let (mut floor, mut ceiling) = if stage == GainStage::Position {
            bound(initial.kpp, start.kpp)
        } else if stage == GainStage::VelocityI {
            bound(initial.kiv, start.kiv)
        } else {
            let p = bound(initial.kpv, start.kpv);
            if stage == GainStage::Velocity {
                p
            } else {
                let i = bound(initial.kiv, start.kiv);
                (p.0.max(i.0), p.1.min(i.1))
            }
        };
        let staged_gain = |gains: par6_config::Gains| match stage {
            GainStage::Position => gains.kpp,
            GainStage::VelocityI => gains.kiv,
            GainStage::Velocity | GainStage::StartupHold => gains.kpv,
        };
        let memory_slot = match stage {
            GainStage::Velocity => Some(0),
            GainStage::VelocityI => Some(1),
            GainStage::Position => Some(2),
            GainStage::StartupHold => None,
        };
        if let Some((low, high)) = memory_slot.and_then(|slot| self.band_memory[j][slot]) {
            let (remembered_floor, remembered_ceiling) =
                (low / staged_gain(initial), high / staged_gain(initial));
            if remembered_floor < remembered_ceiling {
                floor = floor.max(remembered_floor);
                ceiling = ceiling.min(remembered_ceiling);
            }
        }
        let mut interrupted = None;
        let return_stop = seek.and_then(|trial| trial.return_stop);
        let h = &self.bundle.robot.homing.joints[j];
        let home_direction = if h.direction == 1 { -1 } else { 1 };
        let stop_tolerance = h.two_pass_max_diff_ticks;
        // This reference exists only after contact followed by successful backoff.
        // Use the configured vendor two-pass repeatability criterion, not a new
        // proximity threshold: https://github.com/Source-Robotics/RCB-Runtime/blob/main/robotics/homing.py
        let at_return_stop = |m: Measurement| {
            return_stop
                .zip(m.stop)
                .is_some_and(|(reference, position)| {
                    (i64::from(reference) - i64::from(position)).abs() <= i64::from(stop_tolerance)
                        && (m.outcome != Outcome::Blocked || m.direction == home_direction)
                })
        };
        // A failed hold is the oscillation side of the band; a failure with a
        // quiet hold is the sluggish side. The search walks away from whichever
        // it saw. STEPFOC's guide: raise gain until oscillation, then back off.
        // https://source-robotics.github.io/STEPFOC-docs/PID_tuning/
        let holding_limit = self.holding_limit(j);
        let verdict = |score: search::Score, hold_rms_rad_s: f64| {
            if score.accepted() {
                search::Verdict::Pass
            } else {
                search::Verdict::Fail {
                    oscillating: hold_rms_rad_s > holding_limit,
                    violation: match score {
                        search::Score::Feasible { objective } => objective - 1.0,
                        search::Score::Infeasible { violation } => 1.0 + violation,
                        search::Score::Invalid => f64::INFINITY,
                    },
                }
            }
        };
        // Same start and commanded trajectory on every experiment; the firmware
        // integrator cannot be reset over CAN, so every experiment also holds at
        // its start for one observation interval before moving.
        // Hjalmarsson et al., Initial Conditions, p.31:
        // https://perso.uclouvain.be/michel.gevers/PublisMig/IFT-CSMagazine98.pdf
        let mut evaluate = |scale: f64| -> Result<search::Verdict> {
            self.used_trials[j] += 1;
            let mut gains = initial;
            match stage {
                GainStage::Position => gains.kpp *= scale,
                GainStage::Velocity => gains.kpv *= scale,
                GainStage::VelocityI => gains.kiv *= scale,
                GainStage::StartupHold => {
                    gains.kpv *= scale;
                    gains.kiv *= scale;
                }
            }
            if [gains.kpv, gains.kiv, gains.kpp]
                .iter()
                .any(|g| !(*g as f32).is_finite() || *g <= 0.0)
            {
                return Ok(search::Verdict::Fail {
                    oscillating: false,
                    violation: f64::INFINITY,
                });
            }
            self.gains[j] = gains;
            let operating_limit = self.bundle.robot.joints[j].ilim_ma;
            self.configure(j, operating_limit)?;
            self.emit(Event::Tune(j, stage, gains, operating_limit))?;
            if stage == GainStage::StartupHold {
                // Fixed position target throughout; no velocity-mode release,
                // reset move, or fabricated movement measurement at startup.
                let quality = self.measure_hold("startup gain response", None)?[j];
                let score = quality.score(self.holding_limit(j), tolerance);
                self.emit(Event::TuneScore(
                    j,
                    self.used_trials[j],
                    stage,
                    score,
                    score.accepted(),
                ))?;
                return Ok(verdict(score, quality.rms()[1]));
            }
            // The reset must cover its actual distance at the same homing
            // speed. Quintic time scaling has peak speed 1.875 * distance/T:
            // https://modernrobotics.northwestern.edu/nu-gm-book-resource/9-1-and-9-2-point-to-point-trajectories-part-2-of-2/
            let distance = (f64::from(origin) - f64::from(self.pos(j)?)).abs();
            let reset_seconds =
                seconds.max(1.875 * distance / self.bundle.robot.homing.joints[j].speed_ticks_s);
            let reset = self.position_motion(origin, reset_seconds);
            let reset = self.setup_motion(j, reset)?;
            let reset_at_stop = at_return_stop(reset);
            if reset.outcome == Outcome::Blocked && !reset_at_stop {
                interrupted = Some(reset);
                return Ok(search::Verdict::Interrupted);
            }
            if !reset_at_stop
                && (!matches!(reset.outcome, Outcome::Complete | Outcome::Tracking)
                    || reset.stop.is_none()
                    || return_stop.is_some())
            {
                return Ok(search::Verdict::Fail {
                    oscillating: false,
                    violation: f64::INFINITY,
                });
            }
            for _ in 0..self.ticks(limits.hold_observation_s) {
                self.frame(None)?;
            }
            // User requirement: reaching the endstop under homing current is
            // expected. A known-contact reset is experimental setup; score the
            // free backoff and its loaded hold, not tracking into the hard stop.
            let mut measurement = Measurement::default();
            let mut complete = reset_at_stop || reset.outcome == Outcome::Complete;
            for (leg, endpoint) in [target, origin].into_iter().enumerate() {
                // A seek must be tuned in velocity mode: the position loop would
                // otherwise hide its velocity tracking error. Replay at least the
                // failed seek's exposure, then ramp down before reversing.
                let probe = if stage == GainStage::Position {
                    self.position_motion(endpoint, seconds)
                } else if let Some(seek) = seek {
                    Motion::Velocity(
                        if leg == 0 { seek.speed } else { -seek.speed },
                        seek.seconds,
                    )
                } else {
                    Motion::VelocityProfile(endpoint, seconds.max(self.dt))
                };
                let result = if leg == 1 && return_stop.is_some() {
                    self.setup_motion(j, probe)?
                } else {
                    self.motion(j, probe, false)?
                };
                if leg == 1 && at_return_stop(result) {
                    self.emit(Event::TuneStop(
                        self.tick,
                        j,
                        return_stop.ok_or("missing tuning stop reference")?,
                        result.stop.ok_or("missing tuning return position")?,
                        stop_tolerance,
                    ))?;
                    continue;
                }
                if result.outcome == Outcome::Blocked {
                    interrupted = Some(result);
                    return Ok(search::Verdict::Interrupted);
                }
                if leg == 1 && return_stop.is_some() {
                    return Ok(search::Verdict::Fail {
                    oscillating: false,
                    violation: f64::INFINITY,
                });
                }
                complete &= result.stop.is_some()
                    && matches!(result.outcome, Outcome::Complete | Outcome::Tracking);
                measurement = measurement.combine(result);
            }
            let score = stage.score(measurement, tolerance, complete);
            self.emit(Event::TuneScore(
                j,
                self.used_trials[j],
                stage,
                score,
                score.accepted(),
            ))?;
            Ok(verdict(score, measurement.hold_velocity_rms_rad_s))
        };
        let searched = search::bracket(
            search.step,
            floor,
            ceiling,
            search.resolution,
            budget,
            &mut evaluate,
        );
        // An interrupted experiment resumes the common current/contact diagnosis
        // with the same gains that produced its measurement.
        if let Some(measurement) = interrupted {
            self.configure(j, self.bundle.robot.joints[j].ilim_ma)?;
            searched?;
            return Ok(Some(measurement));
        }
        let outcome = searched?;
        self.emit(Event::Band(j, stage, outcome.band))?;
        if let Some(operating) = outcome.operating() {
            if let (Some(slot), Some(low), Some(high)) = (
                memory_slot,
                outcome.band.lowest_pass,
                outcome.band.highest_pass,
            ) {
                self.band_memory[j][slot] =
                    Some((low * staged_gain(initial), high * staged_gain(initial)));
            }
            self.gains[j] = initial;
            self.scale_gains(j, stage, operating);
            self.configure(j, self.bundle.robot.joints[j].ilim_ma)?;
            // Accepted gains are evidence even when a later joint fails the run.
            self.emit(Event::GainsAccepted(j, self.gains[j]))?;
            return Ok(None);
        }
        self.gains[j] = initial;
        self.configure(j, self.bundle.robot.joints[j].ilim_ma)?;
        if let (search::Stop::NoPass, Some((best, _))) = (outcome.stop, outcome.band.best_fail) {
            self.no_band = Some(best);
            return Ok(None);
        }
        Err(format!(
            "J{} {stage:?}: {} (scales x{floor:.3}..x{ceiling:.3} of this episode's start, within x{} of the run's starting gains; {} experiments per stage)",
            j + 1,
            outcome.reason(),
            search.ceiling,
            budget
        )
        .into())
    }
    fn scale_gains(&mut self, j: usize, stage: GainStage, scale: f64) {
        match stage {
            GainStage::Position => self.gains[j].kpp *= scale,
            GainStage::Velocity => self.gains[j].kpv *= scale,
            GainStage::VelocityI => self.gains[j].kiv *= scale,
            GainStage::StartupHold => {
                self.gains[j].kpv *= scale;
                self.gains[j].kiv *= scale;
            }
        }
    }
    fn home_joint(&mut self, j: usize) -> Result<i32> {
        let h = self.bundle.robot.homing.joints[j].clone();
        let speed = (h.speed_ticks_s * if h.direction == 1 { -1.0 } else { 1.0 }) as i32;
        let hall = h.strategy == HomingStrategy::Hall;
        let first = self.adaptive_motion(j, Motion::Seek(speed, hall), None)?;
        let mut reference = first.stop.ok_or("home reference unavailable")?;
        if !hall && h.two_pass {
            // adaptive_motion has already performed the reverse test/backoff.
            // Vendor two-pass speed ratio and repeatability tolerance:
            // https://github.com/Source-Robotics/RCB-Runtime/blob/main/robotics/homing.py
            let mut checks = self.home_retries.max(1);
            loop {
                let second = self.adaptive_motion(
                    j,
                    Motion::Seek((f64::from(speed) * 0.3) as i32, false),
                    Some(reference),
                )?;
                let second = second.stop.ok_or("second home reference unavailable")?;
                let difference = (i64::from(reference) - i64::from(second)).abs();
                self.emit(Event::HomeRepeatability(
                    self.tick,
                    j,
                    reference,
                    second,
                    h.two_pass_max_diff_ticks,
                    (self.conv[j].joint_rad(reference) - self.conv[j].joint_rad(second)).abs(),
                ))?;
                reference = second;
                if difference <= i64::from(h.two_pass_max_diff_ticks) {
                    break;
                }
                checks -= 1;
                if checks == 0 {
                    return Err(format!(
                        "J{} home reference did not repeat within requested experiment budget",
                        j + 1
                    )
                    .into());
                }
                self.emit(Event::Phase(
                    "repeat slow approach using the new stop candidate",
                    j,
                ))?;
            }
        }
        if !hall && !h.two_pass {
            // Even single-pass stall homing must return from the reverse diagnostic.
            let approach = self.adaptive_motion(j, Motion::Seek(speed, false), Some(reference))?;
            reference = approach.stop.ok_or("missing final contact")?;
        }
        self.configure(j, self.bundle.robot.joints[j].ilim_ma)?;
        if let Some(release) = h.release {
            // Vendor current-mode release is an excitation with a specified
            // reference-sampling time, not an idle settling delay. frame() records
            // encoder/current feedback and checks drive freshness every tick.
            // https://github.com/Source-Robotics/RCB-Runtime/blob/main/robotics/homing.py
            for t in 0..self.ticks(release.duration_s) {
                self.frame(Some((j, JointCommand::current(release.current_ma as i16))))?;
                if t == (self.ticks(release.duration_s) as f64 * release.sample_pct).round() as u64
                {
                    reference = self.pos(j)?;
                }
            }
            self.hold[j] = self.pos(j)?;
            self.frame(None)?;
        }
        Ok(reference)
    }
    fn premove(&mut self, spec: PreMove) -> Result<()> {
        match spec {
            PreMove::Nudge {
                joint,
                speed_ticks_s,
                duration_s,
            } => {
                let j = joint as usize;
                let target = self
                    .pos(j)?
                    .saturating_add((speed_ticks_s * duration_s) as i32);
                let motion = self.position_motion(target, duration_s * 1.875);
                self.adaptive_motion(j, motion, None)?;
            }
            PreMove::Position {
                joint,
                position_rad,
                duration_s,
            } => {
                self.move_joint(joint as usize, position_rad, duration_s)?;
            }
            PreMove::Idle { joint, duration_s } => {
                let j = joint as usize;
                let mut generation = self.generation[j];
                // User requirement: proceed on fresh feedback after releasing,
                // with the configured duration serving only as a timeout.
                for t in 0..self.ticks(duration_s) {
                    self.frame(Some((
                        joint as usize,
                        if t < 2 {
                            JointCommand::drop_to_idle()
                        } else {
                            JointCommand::encoder_poll()
                        },
                    )))?;
                    if t < 2 {
                        generation = self.generation[j];
                    } else if self.generation[j] != generation {
                        self.hold[j] = self.pos(j)?;
                        return Ok(());
                    }
                }
                return Err(format!("J{} no fresh encoder feedback after release", j + 1).into());
            }
            PreMove::GripperMove { .. } => {}
        }
        Ok(())
    }
    fn move_joint(&mut self, j: usize, radians: f64, seconds: f64) -> Result<()> {
        if !self.homed[j] {
            return Err(format!("J{} is not referenced", j + 1).into());
        }
        let motion = self.position_motion(self.conv[j].motor_ticks(radians), seconds);
        let result = if self.adaptive {
            self.adaptive_motion(j, motion, None)?
        } else {
            self.motion(j, motion, false)?
        };
        if result.outcome != Outcome::Complete {
            return Err(format!("J{} did not reach the requested position", j + 1).into());
        }
        Ok(())
    }
    fn home(&mut self) -> Result<()> {
        self.adaptive = true;
        let sequence = self.bundle.robot.homing.sequence.clone();
        for step in sequence {
            for m in step.pre_moves {
                self.premove(m)?;
            }
            if let Some(group) = step.home {
                for j in group.joints {
                    let j = j as usize;
                    self.emit(Event::Phase("home", j))?;
                    let found = self.home_joint(j)?;
                    let offset = self
                        .bundle
                        .effective_home_offset(j)
                        .ok_or("missing home offset")?;
                    self.conv[j].set_home(found, offset);
                    self.homed[j] = true;
                    if let Some(post) = self.bundle.robot.homing.joints[j].post_home {
                        let seconds =
                            (f64::from(self.conv[j].motor_ticks(post.position_rad) - self.pos(j)?)
                                .abs()
                                / post.speed_ticks_s)
                                .max(0.3);
                        self.move_joint(j, post.position_rad, seconds)?;
                    }
                }
            }
            for m in step.move_to {
                self.move_joint(m.joint as usize, m.position_rad, m.duration_s)?;
            }
            for m in step.post_moves {
                self.premove(m)?;
            }
        }
        for m in self.bundle.robot.homing.post_moves.clone() {
            self.premove(m)?;
        }
        self.adaptive = false;
        for j in 0..N {
            self.configure(j, self.bundle.robot.joints[j].ilim_ma)?;
        }
        Ok(())
    }
    /// Move every joint to `q` together along one quintic in joint space.
    /// Joint-at-a-time transitions swing the gripper through the table
    /// between the ready and forward-reach poses; the straight joint-space
    /// line between the same endpoints clears it (checked offline against the
    /// collision world, 2026-09-19, and again by the preflight before moving).
    fn pose(&mut self, q: [f64; N], _returning: bool) -> Result<()> {
        let tolerance = self.bundle.robot.motion.settle_tolerance_rad;
        let current = self.angles()?;
        if (0..N).all(|j| (q[j] - current[j]).abs() <= tolerance) {
            return Ok(());
        }
        if !self.homed.iter().all(|h| *h) {
            return Err("a synchronized move needs every joint referenced".into());
        }
        let start: [i32; N] = std::array::from_fn(|j| self.hold[j]);
        let target: [i32; N] = std::array::from_fn(|j| self.conv[j].motor_ticks(q[j]));
        // Every joint runs the same quintic, so the duration is whatever
        // the most demanding joint needs under its own EXEC limits: peak
        // speed is 1.875·d/T and peak acceleration 5.7735·d/T².
        // https://modernrobotics.northwestern.edu/nu-gm-book-resource/9-1-and-9-2-point-to-point-trajectories-part-2-of-2/
        let fraction = self.bundle.robot.selfcal.move_speed_fraction;
        let mut duration = 0.3_f64;
        let mut widest = 0;
        for j in 0..N {
            let travel = (q[j] - current[j]).abs();
            let limits = self.bundle.robot.joints[j].limits.for_mode(LimitMode::Exec);
            let by_speed = 1.875 * travel / (fraction * limits.velocity_rad_s);
            let by_accel = (5.7735 * travel / (fraction * limits.acceleration_rad_s2)).sqrt();
            let seconds = by_speed.max(by_accel);
            if seconds > duration {
                duration = seconds;
                widest = j;
            }
        }
        self.emit(Event::Phase("synchronized move of all joints", widest))?;
        let move_ticks = self.ticks(duration);
        let total = self.ticks(duration + self.bundle.robot.motion.settle_timeout_s);
        for t in 0..total {
            // Quintic time scaling shared by every joint, so all arrive together.
            // https://modernrobotics.northwestern.edu/nu-gm-book-resource/9-1-and-9-2-point-to-point-trajectories-part-2-of-2/
            let u = ((t + 1) as f64 * self.dt / duration).min(1.0);
            let position_scale = u.powi(3) * (10.0 - 15.0 * u + 6.0 * u * u);
            let velocity_scale = 30.0 * u * u * (1.0 - u).powi(2) / duration;
            let commands: [JointCommand; N] = std::array::from_fn(|j| {
                let distance = f64::from(target[j]) - f64::from(start[j]);
                JointCommand::position(
                    (f64::from(start[j]) + distance * position_scale).round() as i32,
                    (distance * velocity_scale) as i32,
                    0,
                )
            });
            self.exchange(commands, true)?;
            if t + 1 >= move_ticks {
                let settled = (0..N).all(|j| {
                    self.pos(j).is_ok_and(|p| {
                        ((f64::from(p) - f64::from(target[j]))
                            * (self.conv[j].joint_rad(1) - self.conv[j].joint_rad(0)))
                        .abs()
                            <= tolerance
                    })
                });
                if settled {
                    self.hold = target;
                    return Ok(());
                }
            }
        }
        let worst = (0..N)
            .map(|j| {
                let error = (f64::from(self.pos(j).unwrap_or(target[j])) - f64::from(target[j]))
                    * (self.conv[j].joint_rad(1) - self.conv[j].joint_rad(0));
                (j, error)
            })
            .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
            .ok_or("no joints")?;
        Err(format!(
            "synchronized move did not settle in {:.1} s: J{} is {:.4} rad from its target (limit {tolerance:.4})",
            duration + self.bundle.robot.motion.settle_timeout_s,
            worst.0 + 1,
            worst.1
        )
        .into())
    }
    /// Every joint's holding current at the pose it is already at,
    /// averaged over the observation window.
    ///
    /// Encoder-only replies do not refresh current, so a value must come
    /// from this drain rather than a cached earlier reply, and the arm
    /// must not drift while measuring — a joint that wanders is holding
    /// something other than what the pose says.
    fn holding_current(&mut self) -> Result<[f64; N]> {
        let tolerance = self.bundle.robot.motion.settle_tolerance_rad;
        let held: [f64; N] = std::array::from_fn(|k| self.conv[k].joint_rad(self.hold[k]));
        let mut sum = [0.0; N];
        let mut count = [0u32; N];
        let mut generation = self.generation;
        for j in 0..N {
            self.state.nodes[self.node(j)].current_ma = None;
        }
        for _ in 0..self.ticks(self.bundle.robot.selfcal.hold_s) {
            self.frame(None)?;
            let q = self.angles()?;
            for k in 0..N {
                if (q[k] - held[k]).abs() > tolerance {
                    return Err(format!(
                        "J{} drifted {:.4} rad from the measured pose (limit {tolerance:.4})",
                        k + 1,
                        q[k] - held[k]
                    )
                    .into());
                }
            }
            for j in 0..N {
                let node = self.node(j);
                let Some(current) = self.state.nodes[node].current_ma else {
                    continue;
                };
                if generation[j] == self.generation[j] {
                    continue;
                }
                generation[j] = self.generation[j];
                let current = f64::from(current);
                if current.abs() >= self.bundle.robot.joints[j].ilim_ma {
                    return Err(format!(
                        "J{} holding current is saturated at this pose",
                        j + 1
                    )
                    .into());
                }
                sum[j] += current;
                count[j] += 1;
                self.state.nodes[node].current_ma = None;
            }
        }
        if let Some(j) = (0..N).find(|j| count[*j] == 0) {
            return Err(format!("J{} reported no holding current at this pose", j + 1).into());
        }
        Ok(std::array::from_fn(|j| sum[j] / f64::from(count[j])))
    }

    /// One identification sample: the torque the arm holds at `q`.
    ///
    /// The pose is approached from below and then from above on every
    /// joint, and the two holding currents averaged. Coulomb friction
    /// enters with the sign of the approach, so averaging opposite
    /// approaches cancels its symmetric part, which on this arm is most
    /// of it — the residual of that fit matched the measured friction to
    /// within 1% (run 1789789330681603048). Lin et al. §IV collect both
    /// directions for the same reason: https://arxiv.org/pdf/2001.06156
    fn identification_sample(&mut self, q: [f64; N]) -> Result<[f64; N]> {
        let mut measured = [[0.0; N]; 2];
        for (k, direction) in [-1.0_f64, 1.0].into_iter().enumerate() {
            let approach: [f64; N] =
                std::array::from_fn(|j| q[j] + direction * gravity_backoff(&self.bundle));
            self.pose(approach, false)?;
            self.pose(q, false)?;
            self.emit(Event::Phase(
                if direction < 0.0 {
                    "hold after approaching from below"
                } else {
                    "hold after approaching from above"
                },
                0,
            ))?;
            measured[k] = self.holding_current()?;
        }
        // Motor mA to joint Nm, through the same factor the model uses.
        Ok(std::array::from_fn(|j| {
            let cfg = &self.bundle.robot.joints[j];
            let factor = par6_bus::spectral::torque_to_ma_factor(
                cfg.gear_ratio,
                cfg.gear_efficiency,
                cfg.kt_nm_a,
                cfg.dir,
            );
            (measured[0][j] + measured[1][j]) / 2.0 / factor
        }))
    }

    /// Identify this arm's own links from static torque.
    ///
    /// What comes out describes the arm with nothing on the flange but
    /// the base attachment, so it stays true when the end effector
    /// changes. A tool is rigidly joined to the last link and shares its
    /// regressor columns, so anything fitted with a gripper on describes
    /// that gripper as much as the arm.
    fn identify(&mut self, poses: &[[f64; N]], ready: [f64; N]) -> Result<par6_kin::gravity::ArmFit> {
        let mut samples = Vec::with_capacity(poses.len());
        for (i, q) in poses.iter().enumerate() {
            self.emit(Event::IdentificationPose(i + 1, poses.len(), *q))?;
            let tau = self.identification_sample(*q)?;
            self.emit(Event::IdentificationTorque(i + 1, tau))?;
            samples.push(par6_kin::gravity::GravitySample { q: *q, tau });
        }
        self.pose(ready, true)?;
        let ridge = self.bundle.robot.selfcal.ridge;
        let fit = par6_kin::gravity::fit_arm(&mut self.kin, &samples, ridge)?;
        self.emit(Event::ArmFit(fit.rms_before_nm, fit.rms_nm))?;
        Ok(fit)
    }

    /// Drive one homed joint to `target` at the configured shutdown speed.
    fn park_joint(&mut self, j: usize, target: f64) -> Result<()> {
        let seconds = (1.5 * (target - self.angles()?[j]).abs()
            / self.bundle.robot.shutdown.velocity_limit_rad_s)
            .max(0.3);
        self.configure(j, self.bundle.robot.joints[j].ilim_ma)?;
        self.move_joint(j, target, seconds)
    }
    fn shutdown(&mut self) -> Result<()> {
        self.adaptive = false;
        self.stopping = true;
        self.deadline = Duration::ZERO;
        // A joint may have stopped answering for a reason that clears on its
        // own; the run reaches here without knowing. Hold position and give
        // every drive a chance to answer again before deciding anything.
        self.blind = true;
        for _ in 0..self.ticks(self.bundle.robot.selfcal.stale_recovery_s) {
            if self
                .exchange(self.hold.map(|p| JointCommand::position(p, 0, 0)), false)
                .is_err()
            {
                break;
            }
            if (0..N).all(|j| {
                self.tick.saturating_sub(self.seen[j]) <= self.ticks(self.feedback_timeout_s)
            }) {
                break;
            }
        }
        self.blind = false;
        for j in 0..N {
            if let Ok(p) = self.pos(j) {
                self.hold[j] = p;
            }
        }
        let mut park = Ok(());
        for j in (0..N).filter(|j| ![1, 2].contains(j)).chain([1, 2]) {
            if !self.homed[j] {
                // A joint the run never referenced goes back to the encoder
                // count the run found it at, so the next run starts from the
                // same posture: the vendor's pre-homing wrist nudge is a
                // relative move and assumes that posture.
                let ticks_per_s = (self.bundle.robot.shutdown.velocity_limit_rad_s
                    / (self.conv[j].joint_rad(1) - self.conv[j].joint_rad(0)).abs())
                .max(1.0);
                let distance = f64::from(self.found_at[j] - self.pos(j)?).abs();
                if distance <= 1.0 {
                    continue;
                }
                let motion = self
                    .position_motion(self.found_at[j], (1.5 * distance / ticks_per_s).max(0.3));
                self.emit(Event::Phase("return unreferenced joint to where the run found it", j))?;
                self.check_only = Some(j);
                let outcome = self
                    .configure(j, self.bundle.robot.joints[j].ilim_ma)
                    .and_then(|()| self.motion(j, motion, false))
                    .and_then(|m| {
                        if m.outcome == Outcome::Complete {
                            Ok(())
                        } else {
                            Err(format!("J{} did not return to its starting position", j + 1).into())
                        }
                    });
                self.check_only = None;
                if let Err(error) = outcome {
                    let _ = self.emit(Event::Phase("parking failed", j));
                    park = Err(error);
                }
                continue;
            }
            // User requirement (2026-09-18): the shoulder and elbow return to
            // their homing endstops before current is released, so releasing
            // them lets them rest on the stops instead of dropping. The other
            // joints park at the rest pose; shoulder and elbow move last.
            let target = if [1, 2].contains(&j) {
                self.bundle
                    .effective_home_offset(j)
                    .ok_or("missing home offset")?
            } else {
                self.bundle.robot.robot.park_pose_rad[j]
            };
            // Each joint is parked on its own feedback. One stale joint used to
            // fail every park, including the loaded shoulder and elbow, and the
            // release that followed dropped the arm (2026-09-19).
            self.check_only = Some(j);
            let mut outcome = self.park_joint(j, target);
            if outcome.is_err() && [1, 2].contains(&j) {
                // The shoulder and elbow hold the arm up: try once more without
                // requiring feedback rather than release them where they are.
                let _ = self.emit(Event::Phase("park blind: feedback unavailable", j));
                self.blind = true;
                outcome = self.park_joint(j, target);
                self.blind = false;
            }
            self.check_only = None;
            if let Err(error) = outcome {
                // A recording error must not bypass the configured motor release.
                let _ = self.emit(Event::Phase("parking failed", j));
                park = Err(error);
            }
        }
        if park.is_err() && !self.bundle.robot.selfcal.release_on_failure {
            return Err("parking failed and release_on_failure is false".into());
        }
        let mut release = Ok(());
        for _ in 0..3 {
            if let Err(e) = self.exchange([JointCommand::drop_to_idle(); N], false) {
                release = Err(e);
            }
        }
        park.and(release)
    }
}

mod search {
    //! Geometric bracketing of the passing band of a single gain scale.
    //!
    //! A velocity PI (or position P) loop passes the requirements on an
    //! interval of gain: sluggish below it, oscillating above it. STEPFOC's
    //! tuning guide walks up until oscillation and backs off 20%; this search
    //! walks in both directions from the start in equal ratio steps, bisects
    //! each boundary to the requested resolution, and operates at the
    //! geometric midpoint of the measured passing band, which is the point
    //! farthest in ratio from both failure modes. That point is then measured
    //! once more and must pass. https://source-robotics.github.io/STEPFOC-docs/PID_tuning/

    // Feasible trials outrank infeasible trials; infeasible trials compare only
    // constraint violation. The objective never compensates for a violation.
    // https://pymoo.org/constraints/feas_first.html
    // Invalid represents an incomplete experiment or unusable measurement.
    #[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
    pub(super) enum Score {
        Feasible { objective: f64 },
        Infeasible { violation: f64 },
        Invalid,
    }
    impl Score {
        pub(super) fn accepted(self) -> bool {
            matches!(self, Self::Feasible { objective } if objective <= 1.0)
        }
        /// Constraints satisfied, whatever the objective. For a hold this is
        /// "the joint stayed where it was put"; `accepted` additionally
        /// requires the hold to be quiet.
        pub(super) fn feasible(self) -> bool {
            matches!(self, Self::Feasible { .. })
        }
    }
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub(super) enum Verdict {
        Pass,
        /// `violation` orders failures: larger is further from passing.
        Fail { oscillating: bool, violation: f64 },
        Interrupted,
    }
    /// Everything measured about the passing band, as scales of the start.
    #[derive(Clone, Copy, Debug, Default)]
    pub(super) struct Band {
        pub lowest_pass: Option<f64>,
        pub highest_pass: Option<f64>,
        pub lower_fail: Option<f64>,
        pub upper_fail: Option<f64>,
        pub operating: Option<f64>,
        pub verified: bool,
        pub experiments: usize,
        /// The failing scale with the smallest violation, for a retry along
        /// another gain axis when no scale on this one passes.
        pub best_fail: Option<(f64, f64)>,
    }
    #[derive(Clone, Copy, Debug)]
    pub(super) enum Stop {
        Verified,
        Interrupted,
        NoPass,
        Budget,
        VerificationFailed,
    }
    #[derive(Clone, Copy, Debug)]
    pub(super) struct Outcome {
        pub band: Band,
        pub stop: Stop,
    }
    impl Outcome {
        pub(super) fn operating(self) -> Option<f64> {
            self.band.operating.filter(|_| self.band.verified)
        }
        pub(super) fn reason(self) -> &'static str {
            match self.stop {
                Stop::Verified => "verified",
                Stop::Interrupted => "interrupted by contact",
                Stop::NoPass => "no gain scale within the bounds met the requirements",
                Stop::Budget => "the experiment budget ran out before the band was bracketed",
                Stop::VerificationFailed => {
                    "the midpoint of the measured band failed its confirming experiment"
                }
            }
        }
    }

    pub(super) fn bracket<E>(
        step: f64,
        floor: f64,
        ceiling: f64,
        resolution: f64,
        budget: usize,
        evaluate: &mut impl FnMut(f64) -> Result<Verdict, E>,
    ) -> Result<Outcome, E> {
        assert!(step > 1.0 && resolution > 1.0, "ratios must exceed one");
        let mut band = Band::default();
        let inside = |scale: f64| scale >= floor * (1.0 - 1e-9) && scale <= ceiling * (1.0 + 1e-9);
        macro_rules! sample {
            ($scale:expr) => {{
                if band.experiments == budget {
                    return Ok(Outcome { band, stop: Stop::Budget });
                }
                let scale: f64 = $scale;
                band.experiments += 1;
                match evaluate(scale)? {
                    Verdict::Interrupted => return Ok(Outcome { band, stop: Stop::Interrupted }),
                    Verdict::Pass => {
                        band.lowest_pass = Some(band.lowest_pass.map_or(scale, |v| v.min(scale)));
                        band.highest_pass = Some(band.highest_pass.map_or(scale, |v| v.max(scale)));
                        true
                    }
                    Verdict::Fail { violation, .. } => {
                        note_fail(&mut band, scale, violation);
                        false
                    }
                }
            }};
        }
        fn note_fail(band: &mut Band, scale: f64, violation: f64) {
            if band.best_fail.is_none_or(|(_, best)| violation < best) {
                band.best_fail = Some((scale, violation));
            }
        }
        let start = 1.0_f64.clamp(floor, ceiling);
        band.experiments += 1;
        let first = match evaluate(start)? {
            Verdict::Interrupted => return Ok(Outcome { band, stop: Stop::Interrupted }),
            other => other,
        };
        let mut start_passed = false;
        match first {
            Verdict::Pass => {
                start_passed = true;
                band.lowest_pass = Some(start);
                band.highest_pass = Some(start);
            }
            Verdict::Fail {
                oscillating,
                violation,
            } => {
                note_fail(&mut band, start, violation);
                // Walk away from the observed failure. A failed hold says
                // oscillation (walk down), a quiet failure says sluggish (walk
                // up), but motion can oscillate while the hold stays quiet, so
                // the violation must shrink along the walk: if the first step
                // is worse, walk the other way from the start; if a later step
                // is worse, the line has no passing scale.
                let preferred = if oscillating { 1.0 / step } else { step };
                let mut found = false;
                for ratio in [preferred, 1.0 / preferred] {
                    let mut scale = start;
                    let mut previous = violation;
                    let mut worse_at_first_step = false;
                    loop {
                        let next = scale * ratio;
                        if !inside(next) {
                            break;
                        }
                        if band.experiments == budget {
                            return Ok(Outcome { band, stop: Stop::Budget });
                        }
                        band.experiments += 1;
                        match evaluate(next)? {
                            Verdict::Interrupted => {
                                return Ok(Outcome { band, stop: Stop::Interrupted })
                            }
                            Verdict::Pass => {
                                band.lowest_pass = Some(next);
                                band.highest_pass = Some(next);
                                if ratio > 1.0 {
                                    band.lower_fail = Some(scale);
                                } else {
                                    band.upper_fail = Some(scale);
                                }
                                found = true;
                                break;
                            }
                            Verdict::Fail { violation, .. } => {
                                note_fail(&mut band, next, violation);
                                if violation > previous {
                                    if ratio > 1.0 {
                                        band.upper_fail = Some(next);
                                    } else {
                                        band.lower_fail = Some(next);
                                    }
                                    worse_at_first_step = scale == start;
                                    break;
                                }
                                previous = violation;
                                scale = next;
                            }
                        }
                    }
                    if found || !worse_at_first_step {
                        break;
                    }
                }
                if !found {
                    return Ok(Outcome { band, stop: Stop::NoPass });
                }
            }
            Verdict::Interrupted => unreachable!(),
        }
        // A start that passes with one step of margin on each side (STEPFOC's
        // 20% back-off, both ways) is kept as it is: walking further out only
        // drives the joint to its oscillation edge to learn a midpoint that
        // cannot be more than a step away.
        if start_passed {
            let up = start * step;
            let down = start / step;
            let up_ok = !inside(up) || sample!(up);
            if !up_ok {
                band.upper_fail = Some(up);
            }
            let down_ok = !inside(down) || sample!(down);
            if !down_ok {
                band.lower_fail = Some(down);
            }
            if up_ok && down_ok {
                band.operating = Some(start);
                band.verified = true;
                return Ok(Outcome { band, stop: Stop::Verified });
            }
        }
        // Expand toward each boundary not yet seen, then bisect both.
        if band.upper_fail.is_none() {
            loop {
                let next = band.highest_pass.unwrap_or(start) * step;
                if !inside(next) {
                    break;
                }
                if !sample!(next) {
                    band.upper_fail = Some(next);
                    break;
                }
            }
        }
        if band.lower_fail.is_none() {
            loop {
                let next = band.lowest_pass.unwrap_or(start) / step;
                if !inside(next) {
                    break;
                }
                if !sample!(next) {
                    band.lower_fail = Some(next);
                    break;
                }
            }
        }
        while let (Some(pass), Some(fail)) = (band.highest_pass, band.upper_fail) {
            if fail / pass <= resolution {
                break;
            }
            let middle = (pass * fail).sqrt();
            if !sample!(middle) {
                band.upper_fail = Some(middle);
            }
        }
        while let (Some(pass), Some(fail)) = (band.lowest_pass, band.lower_fail) {
            if pass / fail <= resolution {
                break;
            }
            let middle = (pass * fail).sqrt();
            if !sample!(middle) {
                band.lower_fail = Some(middle);
            }
        }
        let (Some(low), Some(high)) = (band.lowest_pass, band.highest_pass) else {
            return Ok(Outcome { band, stop: Stop::NoPass });
        };
        let operating = (low * high).sqrt();
        band.operating = Some(operating);
        // A single measured point is its own confirmation only when the band
        // could not be widened at all; otherwise the midpoint is a new point.
        if low == high {
            band.verified = true;
            return Ok(Outcome { band, stop: Stop::Verified });
        }
        if sample!(operating) {
            band.verified = true;
            Ok(Outcome { band, stop: Stop::Verified })
        } else {
            Ok(Outcome { band, stop: Stop::VerificationFailed })
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        /// A passing band [low, high]; outside it the violation grows with the
        /// ratio distance from the band, and the hold fails above `high`.
        fn band(low: f64, high: f64) -> impl FnMut(f64) -> Result<Verdict, ()> {
            move |scale| {
                Ok(if scale >= low && scale <= high {
                    Verdict::Pass
                } else {
                    Verdict::Fail {
                        oscillating: scale > high,
                        violation: (scale / high).ln().max((low / scale).ln()),
                    }
                })
            }
        }
        #[test]
        fn brackets_a_band_from_inside_and_from_both_failing_sides() {
            for start_ratio in [1.0, 0.5, 3.0] {
                // The passing band is [0.9, 1.6] of the true gains; the search
                // starts at a gain start_ratio times the true one.
                let edges = (0.9 / start_ratio, 1.6 / start_ratio);
                let mut count = 0;
                let outcome = bracket(1.25, 0.4, 2.5, 1.1, 30, &mut |scale| {
                    count += 1;
                    band(edges.0, edges.1)(scale)
                })
                .unwrap();
                let operating = outcome.operating().expect("band found");
                // The reachable band is clipped to the search bounds.
                let expected = (edges.0.max(0.4) * edges.1.min(2.5)).sqrt();
                assert!(
                    (operating / expected - 1.0).abs() < 0.12,
                    "start x{start_ratio}: operating {operating} vs midpoint {expected}"
                );
                assert!(count <= 16, "start x{start_ratio} used {count} experiments");
            }
        }
        #[test]
        fn a_passing_start_with_a_step_of_margin_each_way_is_kept() {
            let mut count = 0;
            let outcome = bracket(1.25, 0.4, 2.5, 1.1, 30, &mut |scale| {
                count += 1;
                band(0.7, 1.4)(scale)
            })
            .unwrap();
            assert_eq!(outcome.operating(), Some(1.0));
            assert_eq!(count, 3, "start, one step up, one step down");
        }
        #[test]
        fn motion_oscillation_with_a_quiet_hold_turns_the_walk_around() {
            // Below 0.7 the hold is quiet but motion fails; the heuristic says
            // "sluggish, walk up" at the start, and up is worse.
            let mut scales = Vec::new();
            let outcome = bracket(1.25, 0.4, 2.5, 1.1, 30, &mut |scale: f64| {
                scales.push(scale);
                Ok::<_, ()>(if (0.45..=0.7).contains(&scale) {
                    Verdict::Pass
                } else {
                    Verdict::Fail {
                        oscillating: false,
                        violation: (scale / 0.7).ln().abs(),
                    }
                })
            })
            .unwrap();
            assert!(outcome.operating().is_some(), "{outcome:?}");
            assert_eq!(scales[..3], [1.0, 1.25, 0.8], "one step up, then reverse");
        }
        #[test]
        fn a_line_without_a_band_stops_at_its_minimum() {
            let mut count = 0;
            let outcome = bracket(1.25, 0.4, 2.5, 1.1, 30, &mut |scale: f64| {
                count += 1;
                Ok::<_, ()>(Verdict::Fail {
                    oscillating: scale > 0.6,
                    violation: 0.5 + (scale / 0.6).ln().abs(),
                })
            })
            .unwrap();
            assert!(matches!(outcome.stop, Stop::NoPass), "{outcome:?}");
            assert!(count <= 5, "used {count} experiments");
        }
        #[test]
        fn reports_no_pass_and_budget_exhaustion_without_accepting() {
            let outcome = bracket(1.25, 0.4, 2.5, 1.1, 2, &mut band(0.5, 3.0)).unwrap();
            assert!(outcome.operating().is_none());
            assert!(matches!(outcome.stop, Stop::Budget));
        }
        #[test]
        fn a_noisy_midpoint_is_not_accepted() {
            // The start fails (sluggish); the band is [1.3, 2.4] except that
            // its midpoint, where the confirming experiment lands, fails.
            let outcome = bracket(1.25, 0.4, 2.5, 1.1, 30, &mut |scale: f64| {
                let inside = (1.3..=2.4).contains(&scale) && !(1.70..1.80).contains(&scale);
                Ok::<_, ()>(if inside {
                    Verdict::Pass
                } else {
                    Verdict::Fail {
                        oscillating: scale > 2.4,
                        violation: (scale / 2.4).ln().max((1.3 / scale).ln()).max(0.1),
                    }
                })
            })
            .unwrap();
            assert!(outcome.operating().is_none(), "{outcome:?}");
            assert!(matches!(outcome.stop, Stop::VerificationFailed), "{outcome:?}");
        }
    }
}

mod runtime {
    // Linux scheduling and absolute-deadline sleep semantics:
    // https://man7.org/linux/man-pages/man2/sched_setscheduler.2.html
    // https://man7.org/linux/man-pages/man2/clock_nanosleep.2.html
    use std::{io, time::Duration};

    pub struct Runtime(libc::cpu_set_t, i32, libc::sched_param);
    impl Runtime {
        pub fn prepare(cpu: i64) -> io::Result<Self> {
            if !(0..i64::from(libc::CPU_SETSIZE)).contains(&cpu) {
                return Err(io::Error::other("invalid control CPU"));
            }
            // These calls affect this thread; workers inherit its background CPU mask.
            let mut before = unsafe { std::mem::zeroed() };
            let mut param = libc::sched_param { sched_priority: 0 };
            unsafe {
                if libc::sched_getaffinity(0, std::mem::size_of_val(&before), &mut before) != 0
                    || libc::sched_getparam(0, &mut param) != 0
                {
                    return Err(io::Error::last_os_error());
                }
            }
            let saved = Self(before, unsafe { libc::sched_getscheduler(0) }, param);
            let mut background = before;
            unsafe { libc::CPU_CLR(cpu as usize, &mut background) };
            affinity(&background)?;
            Ok(saved)
        }
    }
    impl Drop for Runtime {
        fn drop(&mut self) {
            unsafe {
                libc::sched_setscheduler(0, self.1, &self.2);
                libc::munlockall();
            }
            if let Err(e) = affinity(&self.0) {
                eprintln!("restore CPU affinity: {e}");
            }
        }
    }
    fn affinity(mask: &libc::cpu_set_t) -> io::Result<()> {
        if unsafe { libc::sched_setaffinity(0, std::mem::size_of_val(mask), mask) } != 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }
    pub fn realtime(cpu: i64, priority: u8) -> io::Result<()> {
        let mut mask = unsafe { std::mem::zeroed() };
        unsafe { libc::CPU_SET(cpu as usize, &mut mask) };
        affinity(&mask)?;
        let param = libc::sched_param {
            sched_priority: i32::from(priority),
        };
        unsafe {
            if libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE) != 0
                || libc::sched_setscheduler(0, libc::SCHED_FIFO, &param) != 0
            {
                return Err(io::Error::last_os_error());
            }
        }
        Ok(())
    }
    pub fn now() -> Duration {
        let mut t = libc::timespec {
            tv_sec: 0,
            tv_nsec: 0,
        };
        unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut t) };
        Duration::new(t.tv_sec as u64, t.tv_nsec as u32)
    }
    pub fn sleep(deadline: Duration) {
        let t = libc::timespec {
            tv_sec: deadline.as_secs() as _,
            tv_nsec: deadline.subsec_nanos() as _,
        };
        unsafe {
            while libc::clock_nanosleep(
                libc::CLOCK_MONOTONIC,
                libc::TIMER_ABSTIME,
                &t,
                std::ptr::null_mut(),
            ) == libc::EINTR
            {}
        }
    }
}

#[cfg(test)]
mod hold_velocity_tests {
    use super::*;
    const TICK_NS: u64 = 4_000_000;

    #[test]
    fn a_joint_resting_on_a_count_boundary_reads_as_stationary() {
        // Readings alternate p, p+1 every tick: the true position sits on the
        // boundary and the joint is not moving.
        let mut estimator = HoldVelocity::new(1.0, 0.004).unwrap();
        estimator.reset(0, 100);
        let mut quality = HoldQuality::default();
        for k in 1..=250u64 {
            let speed = estimator.push(k * TICK_NS, 100 + (k % 2) as i32).unwrap();
            quality.push(0.0, speed, 0.0, 0.0);
        }
        let rms_counts_s = quality.rms()[1];
        assert!(
            rms_counts_s < 30.0,
            "RMS {rms_counts_s} counts/s reads a stationary joint as moving"
        );
    }

    #[test]
    fn a_steady_ramp_is_estimated_at_its_true_speed() {
        let mut estimator = HoldVelocity::new(1.0, 0.004).unwrap();
        estimator.reset(0, 0);
        let mut last = None;
        for k in 1..=250u64 {
            last = estimator.push(k * TICK_NS, (k as i32) * 24).unwrap();
        }
        let speed = last.expect("a linear history yields an estimate");
        assert!((speed - 6000.0).abs() < 1e-6, "estimated {speed} counts/s");
    }
}

#[cfg(test)]
mod gravity_path_tests {
    use super::*;
    /// The daemon's collision world for the fitted MSG gripper with this
    /// arm's configured floor.
    fn world() -> par6_kin::Collision {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let bundle = ConfigBundle::load(&root.join("config/PAR6.toml")).unwrap();
        let mut world = par6_kin::Collision::load(
            &root.join("assets/par6_description"),
            par6_kin::GripperVariant::Msg,
            par6_kin::COLLISION_CLEARANCE_M,
        )
        .unwrap();
        let shapes = bundle
            .installation_shapes
            .iter()
            .map(par6_kin::Shape::from_proto)
            .collect::<std::result::Result<Vec<_>, _>>()
            .unwrap();
        world
            .set_layer(par6_kin::Layer::Installation, &shapes)
            .unwrap();
        world
    }
    #[test]
    fn the_ready_to_forward_reach_transition_is_clear_only_when_synchronized() {
        let mut world = world();
        let ready = [1.57, -1.85, 2.85, 0.0, -0.5, std::f64::consts::PI];
        let forward = [0.0, -0.250_324_770_911_400_93, 4.462_064_209_473_288, 0.0, 0.0, std::f64::consts::PI];
        assert_eq!(world.check_segment(&ready, &forward, 40).unwrap(), None);
        assert_eq!(world.check_segment(&forward, &ready, 40).unwrap(), None);
        // Joint at a time, index order: J2 swings down with the forearm folded.
        let mut step = ready;
        step[0] = forward[0];
        let mut after_j2 = step;
        after_j2[1] = forward[1];
        assert!(
            world.check_segment(&step, &after_j2, 40).unwrap().is_some(),
            "moving J2 alone before J3 must be reported as a collision"
        );
    }
}

#[cfg(test)]
mod holding_limit_tests {
    use super::*;
    const DT: f64 = 0.004;
    fn limits() -> QualityLimits {
        // The shipped CLI defaults (par6-selfcal.rs).
        QualityLimits {
            moving_rad_s: 5f64.to_radians(),
            moving_fraction: 0.15,
            holding_rad_s: 1f64.to_radians(),
            hold_observation_s: 1.0,
        }
    }
    /// Every joint's holding limit has to be reachable on that joint's own
    /// encoder. A still joint resting on a count boundary already reads as a
    /// full count per tick, so a limit below that asks for a quiet that the
    /// estimator cannot report. On this arm one count per tick is 0.220 deg/s
    /// on J2 but 1.373 deg/s on J4 and J5, against a configured 1.0 deg/s.
    #[test]
    fn no_joint_is_asked_to_hold_quieter_than_one_encoder_count_per_tick() {
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
        let robot = par6_config::RobotConfig::load(&path).expect("shipped PAR6.toml");
        for (j, joint) in robot.joints.iter().enumerate() {
            let conversion = JointConversion::from_config(joint);
            let per_tick = conversion.joint_rad(1) - conversion.joint_rad(0);
            let quantum = per_tick.abs() / DT;
            let limit = limits().holding_limit(per_tick, DT);
            assert!(
                limit >= quantum,
                "J{}: holding limit {:.3} deg/s is below its {:.3} deg/s encoder quantum",
                j + 1,
                limit.to_degrees(),
                quantum.to_degrees(),
            );
        }
    }
    /// A passive joint is asked to stay where it was put, not to be quiet.
    /// J5 rang at 2.2 deg/s after a J4 move on 2026-09-20 while moving 0.038
    /// deg in total — 7% of the settling tolerance — and that aborted the run
    /// before the stage that would have retuned J5.
    #[test]
    fn a_passive_joint_that_rings_in_place_still_counts_as_holding() {
        let tolerance = 0.572958f64.to_radians();
        let ringing = HoldQuality {
            samples: 250,
            velocity_samples: 250,
            velocity_squared_rad_s: 250.0 * 2.207f64.to_radians().powi(2),
            peak_error_rad: 0.038f64.to_radians(),
            ..HoldQuality::default()
        };
        // J5 counts negatively; the limit must not depend on that sign.
        let limit = limits().holding_limit(-0.005493f64.to_radians(), DT);
        assert!(
            !passive_hold_failed(ringing, limit, tolerance),
            "a joint 7% of the way to the settling tolerance is holding"
        );
        assert!(
            !ringing.score(limit, tolerance).accepted(),
            "it is still not quiet, and the gain search must see that"
        );
        // A joint that has actually left its commanded position has not held.
        let drifting = HoldQuality {
            peak_error_rad: tolerance * 1.5,
            ..ringing
        };
        assert!(passive_hold_failed(drifting, limit, tolerance));
    }
}

#[cfg(test)]
mod feedback_recovery_tests {
    use super::*;
    /// A drive that stops answering and then comes back must not end the run.
    ///
    /// On 2026-09-21 J1 went quiet for 10.5 s mid-identification with no
    /// error flag, normal bus voltage and all five other drives replying
    /// every tick; the run called it a fault and threw away eleven measured
    /// poses. The vendor firmware answers CAN from `loop()` while the
    /// current loop runs in a timer interrupt, so silence alone says
    /// nothing about whether the motor is still controlled.
    #[test]
    fn a_drive_that_goes_quiet_and_returns_does_not_end_the_run() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let bundle = ConfigBundle::load(&root.join("config/PAR6.toml")).unwrap();
        let assets = root.join("assets/par6_description");
        let tool = bundle
            .active_gripper()
            .and_then(|g| g.urdf_variant.as_deref())
            .and_then(Tool::from_urdf_variant)
            .unwrap_or(Tool::Flange);
        let node = bundle.robot.joints[0].node_id;
        let timeout_s = bundle.robot.bus.stale_warn_s;
        let dt = bundle.robot.robot.tick_dt_s;
        // Ten times the timeout, and well inside the recovery window.
        let silence = (10.0 * timeout_s / dt).round() as u64;
        // Clear of the startup poll, ramp and tool check, so the drive goes
        // quiet during ordinary holding.
        let start = 600;
        let mut sim = SimBus::new(Scene {
            assets: assets.clone(),
            tool,
        });
        sim.silence_node(node, start, silence);
        let (tx, rx) = std::sync::mpsc::sync_channel(8192);
        let mut arm = Arm::adopt(bundle, &assets, Box::new(sim), true, Some(tx), None).unwrap();
        arm.initialize().unwrap();
        let commands = arm.hold.map(|p| JointCommand::position(p, 0, 0));
        while arm.tick < start + silence + 200 {
            let at = arm.tick;
            arm.exchange(commands, true)
                .unwrap_or_else(|e| panic!("tick {at}: {e}"));
        }
        let gaps: Vec<_> = rx
            .try_iter()
            .filter_map(|event| match event {
                Event::FeedbackGap(_, j, seconds) => Some((j, seconds)),
                _ => None,
            })
            .collect();
        assert_eq!(gaps.len(), 1, "expected one recovery, got {gaps:?}");
        assert_eq!(gaps[0].0, 0, "the silent joint was J1");
        // Recovery starts only once the drive has been quiet for the
        // feedback timeout, so the reported gap is the silence less that
        // detection delay.
        let muted_s = silence as f64 * dt;
        assert!(
            (gaps[0].1 - (muted_s - timeout_s)).abs() <= 2.0 * dt,
            "J1 was muted {muted_s:.2} s and detected after {timeout_s:.2} s; \
             the run reported {:.2} s",
            gaps[0].1
        );
    }
}
