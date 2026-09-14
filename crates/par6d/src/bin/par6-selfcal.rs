//! `par6-selfcal` — calibration with nothing between it and the arm.
//!
//! One binary, one CAN socket, and its own copy of the homing sequence. No
//! daemon, no command protocol, no client, and deliberately not the
//! runtime's `HomingSystem`: that state machine fails a whole sequence when
//! a joint misses its position, and what calibration needs is the opposite
//! — stay on that joint, give it more authority, try it again, and only
//! move on once it arrives.
//!
//! STEP 1 — home on the configured (vendor) gains, making every joint
//! arrive rather than restarting the sequence:
//!
//!   * `nudge` pre-moves run as velocity pushes, as the config describes.
//!   * a `home` group seeks each joint to its endstop by stall: drive at
//!     the configured seek speed under the configured current limit, watch
//!     for the encoder to stop advancing while current is up, dwell, back
//!     off, and latch the reference through [`JointConversion::set_home`].
//!   * a `move_to` drives a Hermite position profile and requires the
//!     joint to sit inside the arrival window for a run of ticks.
//!   * when a seek times out or a move_to never arrives, that joint's
//!     velocity-loop gains are raised a step and the SAME operation runs
//!     again, up to the ceiling. Nothing earlier in the sequence repeats.
//!
//! Only `kpv` and `kiv` move. Position gain, the current loop and every
//! limit stay where the vendor put them, and nothing is written to the
//! drives' stored config, so a power cycle undoes all of it.
//!
//! The runtime must not be running: two writers on one bus is two
//! controllers fighting over the same drives.

use std::time::{Duration, Instant};

use par6_bus::spectral::JointConversion;
use par6_bus::sim::{scene::Scene, SimBus};
use par6_bus::{
    hw::SocketCanBus, BusError, BusState, DriveTune, DriverBus, GripperCommand, JointCommand,
    NodeId,
};

/// Which bus the tool is driving.
///
/// The simulated backend exists so the SCRIPT can be proven with no arm
/// attached: whether any operation sits still while commanded to move,
/// whether a seek stops when the joint stops, whether the monitor reports.
/// It says nothing about real vibration or real gain values, because the
/// simulated drive model is softer than the arm.
enum Backend {
    Can(SocketCanBus),
    Sim(SimBus),
}

impl Backend {
    fn as_bus(&mut self) -> &mut dyn DriverBus {
        match self {
            Backend::Can(b) => b,
            Backend::Sim(b) => b,
        }
    }
}
use par6_config::{
    ConfigBundle, GripperConfig, HomingStrategy, JointHoming, PreMove, RobotConfig,
};

/// Each retry multiplies the joint's velocity-loop gains by this.
const GAIN_STEP: f64 = 1.25;

/// The most the gains may be raised before the answer is "this is not a
/// tuning problem".
const GAIN_CEILING: f64 = 2.5;

/// The velocity-gain scales a ringing joint is measured at, as multiples of
/// the config's value.
///
/// Walking down one rung at a time cannot find the best of them: the quietest
/// is usually in the middle, and a walk that passes it and then hits the
/// bottom has nowhere left to go — and a joint left at the bottom is too
/// slack to hold its own weight. So every rung is held, timed, and the
/// quietest wins.
const RING_LADDER: [f64; 5] = [1.0, 0.8, 0.64, 0.51, 0.41];

/// Samples the mid-move oscillation detector looks back over, and what it
/// takes to call a joint ringing: direction reversals that keep coming, with
/// an amplitude worth caring about. A limit cycle reverses every half
/// period — around 3 Hz on this arm, so a dozen ticks — where tracking a
/// profile does not reverse at all.
const RING_WINDOW_TICKS: usize = 500;
const RING_REVERSALS: usize = 8;

/// The slowest reversal that still counts as shaking \[ticks between turning
/// points\]. Fifty ticks is 0.2 s, so anything from 2.5 Hz up.
const RING_HALF_PERIOD_TICKS: usize = 50;

/// How far above a joint's own measured dither a swing has to be before it is
/// a ring rather than that joint working.
const RING_OVER_FLOOR: f64 = 1.5;

/// How long a ring has to persist before the run is over \[ticks\]: one whole
/// further window of it, continuously.
const RING_PERSIST_TICKS: usize = RING_WINDOW_TICKS;

/// How long a joint is left alone after a move before anything judges it
/// \[s\].
///
/// Holding a load with no load term means the velocity integrator has to wind
/// up to carry it, and that takes seconds: the joint creeps, overshoots, and
/// rings down. All three are one transient. Judging inside it reads the creep
/// as droop and the overshoot as ringing, changes the gain, and restarts the
/// transient — which is audible as a staircase that never converges.
const SETTLE_BEFORE_JUDGING_S: f64 = 0.25;

/// The excursion that counts as ringing, as an ANGLE \[deg\].
///
/// Not a tick count: one encoder tick is a different angle on every joint,
/// so a fixed count is 0.10 deg of slop on the base and 0.026 deg on the
/// shoulder. The same shake would be flagged on one joint and invisible on
/// the next, which is exactly what happened to the base.
const RING_AMPLITUDE_DEG: f64 = 0.20;
// The only externally calibrated figure available: the ring that is audible
// from this arm, standing next to it, measured 0.204 deg. Below that the
// numbers are the drives working — the elbow swings 0.05-0.13 deg holding 2 A
// at full extension, varying between windows, and every tighter bar I tried
// failed healthy runs on it. 0.02 deg was below the arrival window itself
// (0.044 deg), which failed the elbow for moving less than the tolerance it
// was being held to.

/// The most two seek passes may disagree before the first stop is called an
/// obstruction rather than the endstop \[deg\].
///
/// The config states this per joint in encoder ticks, and the same count is
/// 12 deg on the base and 1.8 deg on the shoulder — so the looser joints can
/// accept a disagreement of degrees and still report a reference. This is
/// the angular ceiling that applies whatever the tick count says.
const TWO_PASS_MAX_DEG: f64 = 0.5;

/// How one-directional a joint's motion has to be to call it a droop rather
/// than a wobble \[percent of the steps that moved\].
const DRIFT_AGREEMENT_PCT: usize = 85;

/// Arrival window for a `move_to` \[encoder ticks\], and how many
/// consecutive ticks inside it count as arrived. Both are the runtime's.
const POS_TOL_TICKS: i64 = 50;
const IN_POS_STREAK: u32 = 10;

/// Stall detection: the joint is on its stop when the speed it is making
/// collapses against the speed it was told to make.
///
/// Relative, not an absolute tick count. A joint leaning on its stop under a
/// seek current deflects and creeps — it does not stop dead — so an absolute
/// threshold is either met before the stop is reached or never met at all,
/// and "never met" means pushing against the stop for the whole timeout.
/// Measured speed under this fraction of commanded is the stop, whatever the
/// numbers are.
const STALL_WINDOW_TICKS: u32 = 40;
const STALL_SPEED_FRACTION: f64 = 0.15;
const STALL_CURRENT_FRACTION: f64 = 0.85;

/// Ticks to ignore at the start of a seek, while the drive spins up.
const SEEK_STARTUP_TICKS: u32 = 50;

/// The trigger value a HALL approach asks the driver to report on, and the
/// window in which an immediate trigger means the joint STARTED on the
/// sensor rather than just found it \[ticks\].
const HALL_TRIGGER_VALUE: u8 = 2;
const HALL_PRECLEAR_TICKS: u32 = 125;

/// Ticks to let pushed config land before believing a reading.
///
/// The config frames repeat three times at the bus's own pacing, so this is
/// the time those take to go out, not a margin for luck.
const CONFIG_SETTLE_TICKS: u32 = 40;

/// The longest the arm may be commanded to move while not moving \[s\].
///
/// A hard rule, enforced rather than documented: if a joint is being driven
/// and its encoder has not changed for this long, the operation is over. Every
/// way this tool has wasted time — leaning on a stop until a twenty second
/// timeout, running out a settle budget after arriving, waiting on a profile
/// that already ended — is commanded motion with nothing moving, and this
/// catches all of them in one place.
const NO_MOTION_ABORT_S: f64 = 1.0;

/// How far a joint has to turn for the arm to count as having moved \[deg\].
/// Above encoder noise on every joint, and far below anything an operator
/// would call movement.
const STILL_DEG: f64 = 0.01;

/// A seek abandons a joint that has not moved for this long \[s\].
///
/// Deliberately shorter than [`NO_MOTION_ABORT_S`]: a joint too weak to start
/// is the case the gain bump exists for, so it has to be recognised and acted
/// on before the arm-wide rule calls the whole run a failure.
const SEEK_STILL_S: f64 = 0.4;

/// How far a joint must travel before a stall is believed to be its endstop
/// \[deg\].
///
/// A loaded joint that never leaves the pose it started in is saturating its
/// current against the arm's own weight, and its speed collapses exactly the
/// way arriving at a stop does. Without this the shoulder "stalled" 0.1 deg
/// into a 38 deg approach and referenced the arm there.
const SEEK_MIN_TRAVEL_DEG: f64 = 0.5;

/// A seek has to reach this fraction of its commanded speed before anything
/// about the way it slows down means "endstop".
///
/// A joint that never gets up to speed is not approaching a stop, it is losing
/// to the arm's own weight, and its speed collapse looks exactly like arriving.
/// The shoulder made 850 of 6000 ticks/s for a whole approach and was latched
/// as referenced; this is what tells those apart.
const SEEK_LAUNCH_FRACTION: f64 = 0.5;

/// How long a seek is given to get up to speed \[s\].
const SEEK_LAUNCH_S: f64 = 1.0;

/// How much of the secant's suggested jump to take, and the most the gravity
/// scale may move in one attempt. Damped because each drift reading is half a
/// second of a real arm, so one noisy pair must not throw the search.
/// How far the joint under test may move before the watch is cut short
/// \[rad\]. It is on torque only, so this is the fall a wrong scale is allowed
/// to produce — enough travel to fit a clean rate to, far less than enough to
/// matter.
const GRAVITY_ABORT_RAD: f64 = 3.5e-3;

/// How much of the torque-only window is thrown away before the rate is fitted
/// \[s\].
///
/// The moment the position pack is dropped, the drive's integrated current
/// goes with it, and the joint moves through that release whatever the
/// feedforward is. Fitting across it is what made the elbow read 0.016 deg/s
/// and 0.045 deg/s at the same scale. Total stillness stays under the second
/// the arm is allowed: 0.25 settling + 0.65 watching.
const GRAVITY_SETTLE_IN_S: f64 = 0.15;

/// How much wider the arrival window is for a joint whose gravity feedforward
/// has not been measured yet.
const POSE_TOL_FACTOR: i64 = 3;

const SECANT_DAMPING: f64 = 0.7;
const GRAVITY_MAX_STEP: f64 = 0.15;

/// How much a saturated seek raises its current limit per attempt, up to the
/// joint's own `ilim_ma`.
const SEEK_CURRENT_STEP: f64 = 1.25;

/// A position change bigger than this multiple of what the joint's own
/// velocity limit allows in one tick is a counter glitch, not motion.
///
/// The shoulder's reading stepped by a third of a million ticks mid-seek once,
/// and the travel bound read that as 145 deg of travel past a 143 deg range —
/// so it abandoned three consecutive passes and the run took 91 s instead of
/// 55. A reference is relative to the stop it latches, so a counter that moves
/// once and stays put is harmless; what is not harmless is treating the step as
/// distance covered.
const IMPLAUSIBLE_STEP_FACTOR: f64 = 3.0;

/// A reference spread beyond this is a counter that re-zeroed between runs,
/// not a joint that found a different stop \[deg\].
const COUNTER_EPOCH_DEG: f64 = 10.0;

/// How many ticks a limit is repeated over. The vendor repeats a config frame
/// three times; spreading them one per tick keeps the cadence clean.
const LIMIT_REPEATS: usize = 3;

/// How many times a limit push waits for room in the transmit queue. At one
/// tick each this is a second of patience, against a drive watchdog of five.
const LIMIT_ATTEMPTS: usize = 250;

/// Current this close to a joint's limit counts as saturated: the drive is
/// giving everything it has, so nothing about the controller will help.
const SATURATED_FRACTION: f64 = 0.9;

/// Encoder movement below this is noise, not motion \[ticks\].
///
/// Generous enough that a joint creeping into its stop under a seek current
/// counts as stopped: the point of the rule is that the operator sees no
/// motion, and a fraction of a degree per second is no motion.
const STALL_IDLE_TICKS: i64 = 120;

/// Encoder movement below this counts as settled when deciding the approach
/// has finished \[ticks\]. Tighter than the no-motion rule, because this is
/// about a measurement being clean rather than about the operator's patience.
const SETTLED_TICKS: i64 = 20;

/// Slack on the travel bound, so a seek that legitimately starts at one limit
/// and ends at the other is not cut short \[ticks\].
const SEEK_RANGE_SLACK_TICKS: i64 = 2000;

/// Every joint is watched for oscillation on every tick, not just the one
/// being worked on: ringing anywhere after stage 1 means stage 1 did not
/// finish its job, wherever it shows up.
// Tied to the ring detector's own window deliberately. Held at 250 while
// `ringing_now` wanted 500, the continuous check could never fire: every joint's
// history was too short to judge, so `oscillating` always came back empty and
// the "nothing is ringing" it produced meant nothing at all.
const WATCH_WINDOW_TICKS: usize = RING_WINDOW_TICKS;
const _: () = assert!(
    WATCH_WINDOW_TICKS >= RING_WINDOW_TICKS,
    "the per-joint history must be long enough for the ring detector to judge, \
     or the continuous check silently passes everything"
);

/// How long the arm is held, on its configured gains, while the tool hands
/// it back \[s\]. Long enough for a runtime to be started against it.
const HANDOVER_HOLD_S: f64 = 0.3;

/// The rate a repositioning move is planned at \[rad/s\], the ramp it adds,
/// and the bounds on the result. These moves are not measurements, so they run
/// at a sensible rate rather than a cautious one.
const MOVE_RAD_S: f64 = 0.6;
const MOVE_RAMP_S: f64 = 0.35;
const MOVE_MIN_S: f64 = 0.3;
const MOVE_MAX_S: f64 = 4.0;

/// The longest the boot wait will look for a first reading from every joint
/// \[s\].
const BOOT_READING_CAP_S: f64 = 2.0;

// ----------------------------------------------------------------- step 2

/// How long each joint is watched while compensation carries it \[s\].
///
/// Long enough for a slow creep to show: falling is obvious, but a joint that
/// gives up a tenth of a degree over several seconds is the case that matters,
/// and a short look cannot tell that from noise.
const GRAVITY_WATCH_S: f64 = 0.65;

/// Drift that counts as holding \[rad\]. About 0.06 deg — the same order as
/// homing's own arrival window.
const GRAVITY_HOLD_RAD_S: f64 = 1.0e-3;
// 0.057 deg/s. Chosen from the measured scatter, not from taste: repeated
// readings of the same scale on the elbow land 0.01-0.05 deg/s apart, so a
// tighter bound rejects correct answers as often as wrong ones. On a
// torque-only hold this is 0.06 deg of give per second, against an arm whose
// arrival window is 0.044 deg.

/// How strongly the scale reacts to the drift it just saw. The correction is
/// proportional to the error, so a large miss is not walked off in constant
/// steps.
const GRAVITY_GAIN: f64 = 0.08;

/// How far inside the hold tolerance a scale has to land to be accepted.
///
/// A quarter, from what the joints actually do: the wrist and wrist pitch
/// settle at 0.005-0.013 deg/s on their right scale, while the elbow was
/// accepted at 0.025 — inside the tolerance, on the edge of this margin — and
/// then drifted 0.63 deg/s when verification looked again. Verification
/// recovers from that, but it costs a refinement round the search should not
/// have needed: a reading five times worse than a joint's own best is a signal
/// to keep looking, not an answer.
const ACCEPT_MARGIN: f64 = 0.25;

/// Attempts per joint before the answer is "this is not a gravity-scale
/// problem".
const GRAVITY_ATTEMPTS: usize = 12;

/// The range a per-arm difference can explain. The vendor's masses are close;
/// a scale outside this is a different fault wearing gravity's clothes.
const GRAVITY_SCALE_MIN: f64 = 0.5;
const GRAVITY_SCALE_MAX: f64 = 1.5;

/// How hard a correction to the vendor parameters is penalised, relative to
/// the observation scale.
///
/// How long the brake pack is commanded before the tool exits \[s\].
const RELEASE_S: f64 = 0.3;

/// Ask for the same real-time treatment the runtime's control loop gets.
///
/// The drives close their own loops at 6.25 kHz and expect a setpoint every
/// tick; a command stream that slips is a disturbance injected into the
/// position loop, and no gain value answers that. A plain userspace loop on a
/// loaded board slips, so this asks for SCHED_FIFO, locks memory, and paces
/// on absolute deadlines — measuring the period either way, because a request
/// that was refused must not look like one that was granted.
fn request_realtime(priority: u8) -> bool {
    #[cfg(target_os = "linux")]
    {
        use thread_priority::{
            set_thread_priority_and_policy, thread_native_id, RealtimeThreadSchedulePolicy,
            ThreadPriority, ThreadPriorityValue, ThreadSchedulePolicy,
        };
        // SAFETY: mlockall takes only flags and touches no caller memory.
        unsafe {
            libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE);
        }
        let Ok(value) = ThreadPriorityValue::try_from(priority) else {
            return false;
        };
        set_thread_priority_and_policy(
            thread_native_id(),
            ThreadPriority::Crossplatform(value),
            ThreadSchedulePolicy::Realtime(RealtimeThreadSchedulePolicy::Fifo),
        )
        .is_ok()
    }
    #[cfg(not(target_os = "linux"))]
    false
}

/// Sleep until `deadline` on the monotonic clock, absolute rather than
/// relative so an early wake is absorbed instead of drifting.
#[cfg(target_os = "linux")]
fn sleep_until(deadline: Duration) {
    let ts = libc::timespec {
        tv_sec: deadline.as_secs() as libc::time_t,
        tv_nsec: deadline.subsec_nanos() as libc::c_long,
    };
    // SAFETY: TIMER_ABSTIME sleep against a fully initialized timespec;
    // EINTR retries are handled by looping on the return value.
    unsafe {
        while libc::clock_nanosleep(
            libc::CLOCK_MONOTONIC,
            libc::TIMER_ABSTIME,
            &ts,
            std::ptr::null_mut(),
        ) == libc::EINTR
        {}
    }
}

/// The monotonic clock as the kernel sees it, for absolute deadlines.
#[cfg(target_os = "linux")]
fn monotonic_now() -> Duration {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: writes only the timespec it is given.
    unsafe {
        libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts);
    }
    Duration::new(ts.tv_sec as u64, ts.tv_nsec as u32)
}

/// What the command cadence actually did, in microseconds per tick.
struct Cadence {
    /// Tick periods as a fixed histogram rather than a sample per tick: the
    /// tick path must not allocate, and a Vec that grows once every couple of
    /// hundred ticks still allocates. A run can be minutes long, so the sample
    /// list was unbounded too.
    hist: [u32; CADENCE_BUCKETS],
    count: usize,
    sum_us: u64,
    worst_us: u32,
    worst_at: usize,
    worst_phase: String,
    phase: String,
}

/// Histogram resolution and extent for the cadence record: 50 us buckets up to
/// 10 ms, with everything beyond in the last one.
const CADENCE_BUCKET_US: u32 = 50;
const CADENCE_BUCKETS: usize = 200;

impl Default for Cadence {
    fn default() -> Self {
        Self {
            hist: [0; CADENCE_BUCKETS],
            count: 0,
            sum_us: 0,
            worst_us: 0,
            worst_at: 0,
            worst_phase: String::new(),
            phase: String::new(),
        }
    }
}

impl Cadence {
    fn record(&mut self, d: Duration) {
        let us = d.as_micros() as u32;
        if us > self.worst_us {
            self.worst_us = us;
            self.worst_at = self.count;
            // The one allocation in here, and only when a new worst appears:
            // a handful of times per run, never per tick.
            self.worst_phase.clear();
            self.worst_phase.push_str(&self.phase);
        }
        let bucket = (us / CADENCE_BUCKET_US) as usize;
        self.hist[bucket.min(CADENCE_BUCKETS - 1)] += 1;
        self.count += 1;
        self.sum_us += u64::from(us);
    }

    /// Name what the tool is doing, so a late tick can be attributed instead
    /// of guessed at.
    fn phase(&mut self, what: &str) {
        self.phase = what.to_string();
    }

    /// Mean, p99, worst \[us\] and the sample count.
    fn summary(&self) -> Option<(f64, u32, u32, usize)> {
        if self.count == 0 {
            return None;
        }
        let mean = self.sum_us as f64 / self.count as f64;
        // The bucket the 99th percentile falls in, reported as its upper edge.
        let target = (self.count as f64 * 0.99).ceil() as usize;
        let mut seen = 0usize;
        let mut p99 = self.worst_us;
        for (b, n) in self.hist.iter().enumerate() {
            seen += *n as usize;
            if seen >= target {
                p99 = ((b + 1) as u32 * CADENCE_BUCKET_US).min(self.worst_us);
                break;
            }
        }
        Some((mean, p99, self.worst_us, self.count))
    }
}

/// The comment written beside a gain this run raised.
///
/// A second run starts from the config a first run wrote, so `scale` is
/// relative to a value that may itself already be scaled. Reading the earlier
/// comment back keeps the total honest — otherwise the third apply still
/// claims "x1.25 of the vendor value" while the drive is on x1.95 of it.
fn gain_note(existing: &str, scale: f64) -> String {
    let prior = existing
        .split("selfcal: x")
        .nth(1)
        .and_then(|rest| {
            rest.split(|c: char| !(c.is_ascii_digit() || c == '.'))
                .next()
                .and_then(|n| n.parse::<f64>().ok())
        })
        .filter(|p| *p > 0.0);
    match prior {
        Some(p) => format!(
            "# selfcal: x{:.3} of the vendor value (x{scale:.3} again this run)",
            p * scale
        ),
        None => format!("# selfcal: x{scale:.3} of the vendor value"),
    }
}

/// The loudest thing a joint did, in both channels, with the phase it did it
/// in.
///
/// Recorded on every joint from the first tick and never used to judge
/// anything: the thresholds in this tool were set from one audible datum, and
/// a run that PASSES while somebody in the room can hear the arm shaking is a
/// run whose measurements have to be read before its bars are moved again.
struct Noise {
    /// Worst swing between reversals while the joint was travelling \[ticks\].
    moving: f64,
    moving_phase: String,
    /// Worst swing between reversals while it was holding \[ticks\].
    holding: f64,
    holding_phase: String,
    /// Worst swing between reversals in the drive current \[mA\].
    ripple: f64,
    ripple_phase: String,
}

impl Default for Noise {
    fn default() -> Self {
        Self {
            moving: 0.0,
            // Preallocated: these are written from the tick path, where
            // growing a String is an allocation like any other.
            moving_phase: String::with_capacity(PHASE_NAME_BYTES),
            holding: 0.0,
            holding_phase: String::with_capacity(PHASE_NAME_BYTES),
            ripple: 0.0,
            ripple_phase: String::with_capacity(PHASE_NAME_BYTES),
        }
    }
}

/// Room for the longest phase name, so recording one never reallocates.
const PHASE_NAME_BYTES: usize = 64;

/// What the run measured, accumulated as it goes so a later failure cannot
/// take the earlier answers down with it.
#[derive(Default)]
struct Results {
    /// Velocity-gain scale per joint that homing needed.
    gain_scale: Vec<f64>,
    /// Gravity scale per joint that held it at its most loaded pose, where
    /// this run measured one.
    ///
    /// `None` is not "one": it is "not measured", and the two have to stay
    /// apart. A run that fails before it reaches the elbow must leave the
    /// elbow's scale — very likely an earlier run's measurement — exactly
    /// where it is, instead of writing a default over it.
    gravity_scale: Vec<Option<f64>>,
    /// Home reference per joint \[rad\], as latched.
    home_rad: Vec<Option<f64>>,
    /// Seek current per joint the arm actually needed \[mA\], where it is not
    /// the configured value.
    seek_ma: Vec<Option<f64>>,
}

impl Results {
    fn new(n: usize) -> Self {
        Self {
            gain_scale: vec![1.0; n],
            gravity_scale: vec![None; n],
            home_rad: vec![None; n],
            seek_ma: vec![None; n],
        }
    }

    /// Write the measurements into the robot config itself, keeping a backup.
    ///
    /// A calibration that only prints is a calibration nobody has. The backup
    /// is written first and named in the output, so the change is reversible by
    /// copying one file back.
    fn apply(&self, config: &std::path::Path, robot: &RobotConfig) -> Result<(), String> {
        let text = std::fs::read_to_string(config)
            .map_err(|e| format!("reading {}: {e}", config.display()))?;
        let backup = config.with_extension("toml.before-selfcal");
        std::fs::write(&backup, &text).map_err(|e| format!("writing {}: {e}", backup.display()))?;

        // Edited as text, not reserialised: the config is full of comments that
        // explain why its numbers are what they are, and a round trip through a
        // serialiser would throw every one of them away.
        let mut out = String::new();
        let mut joint = 0usize;
        let mut in_gains = false;
        let mut homing_joint = 0usize;
        let mut in_homing = false;
        for line in text.lines() {
            let trimmed = line.trim();
            if trimmed.starts_with("[[homing.joints]]") {
                homing_joint += 1;
                in_homing = true;
                in_gains = false;
            } else if trimmed.starts_with("[[joints]]") {
                joint += 1;
                in_gains = false;
                in_homing = false;
            } else if trimmed.starts_with("[joints.gains]") {
                in_gains = true;
                in_homing = false;
            } else if trimmed.starts_with('[') {
                in_gains = false;
                in_homing = false;
            }
            if in_homing && trimmed.starts_with("current_ma = ") {
                if let Some(ma) = self
                    .seek_ma
                    .get(homing_joint.saturating_sub(1))
                    .copied()
                    .flatten()
                {
                    out.push_str(&format!(
                        "current_ma = {ma:.1}  # selfcal: the drive saturated below this\n"
                    ));
                    continue;
                }
            }
            let j = joint.saturating_sub(1);
            let scale = self.gain_scale.get(j).copied().unwrap_or(1.0);
            if in_gains && scale != 1.0 && j < robot.joints.len() {
                if trimmed.starts_with("kpv = ") {
                    out.push_str(&format!(
                        "kpv = {:.6}  {}\n",
                        robot.joints[j].gains.kpv * scale,
                        gain_note(line, scale)
                    ));
                    continue;
                }
                if trimmed.starts_with("kiv = ") {
                    out.push_str(&format!(
                        "kiv = {:.6}  {}\n",
                        robot.joints[j].gains.kiv * scale,
                        gain_note(line, scale)
                    ));
                    continue;
                }
            }
            out.push_str(line);
            out.push('\n');
        }
        // The gravity scales are step 2's whole output, and the config has a
        // field for exactly this: a per-joint feedforward trim that changes no
        // torque constant. Written at the top level, above the first section,
        // where a bare key has to live in TOML.
        // What each joint ends up on: this run's measurement where there is
        // one, and otherwise whatever the config already carries — which on a
        // second run is the previous run's answer.
        let effective: Vec<f64> = (0..6).map(|j| self.gravity_for(j, robot)).collect();
        if effective
            .iter()
            .enumerate()
            .any(|(j, s)| *s != 1.0 || *s != robot.gravity_scale.get(j).copied().unwrap_or(1.0))
        {
            let values: Vec<String> =
                effective.iter().map(|s| format!("{s:.4}")).collect();
            let line = format!("gravity_scale = [{}]", values.join(", "));
            let mut replaced = false;
            let mut with_scale = String::new();
            for l in out.lines() {
                if l.trim_start().starts_with("gravity_scale") {
                    with_scale.push_str(&format!("{line}  # selfcal: measured per joint\n"));
                    replaced = true;
                } else {
                    with_scale.push_str(l);
                    with_scale.push('\n');
                }
            }
            out = if replaced {
                with_scale
            } else {
                format!("{line}  # selfcal: measured per joint\n{out}")
            };
        }
        std::fs::write(config, &out).map_err(|e| format!("writing {}: {e}", config.display()))?;
        println!(
            "  applied to {}; the config as it was before this run is {}",
            config.display(),
            backup.display()
        );
        Ok(())
    }

    /// The scale a joint ends a run on: measured here, or carried from the
    /// config, which is where a previous run's answer lives.
    fn gravity_for(&self, joint: usize, robot: &RobotConfig) -> f64 {
        self.gravity_scale
            .get(joint)
            .copied()
            .flatten()
            .unwrap_or_else(|| robot.gravity_scale.get(joint).copied().unwrap_or(1.0))
    }

    /// Write what was measured as a TOML patch.
    ///
    /// Written on EVERY exit, including a failure, because a run that homed
    /// four joints and then failed on the fifth has still measured four
    /// joints, and throwing that away means doing it again for nothing. The
    /// patch is a patch, not the config: applying it stays a decision.
    fn write(&self, path: &std::path::Path, robot: &RobotConfig) {
        let mut s = String::new();
        s.push_str("# par6-selfcal measurements. Apply the values you want; this\n");
        s.push_str("# file is never read back by anything.\n");
        for (j, joint) in robot.joints.iter().enumerate() {
            let gain = self.gain_scale.get(j).copied().unwrap_or(1.0);
            let configured = robot.gravity_scale.get(j).copied().unwrap_or(1.0);
            let grav = self
                .gravity_scale
                .get(j)
                .copied()
                .flatten()
                .filter(|g| *g != configured);
            let home = self.home_rad.get(j).copied().flatten();
            let seek = self.seek_ma.get(j).copied().flatten();
            if gain == 1.0 && grav.is_none() && home.is_none() && seek.is_none() {
                continue;
            }
            s.push_str(&format!("\n# --- J{} ---\n", j + 1));
            if let Some(h) = home {
                s.push_str(&format!("# homed at {h:.6} rad\n"));
            }
            if gain != 1.0 {
                s.push_str(&format!(
                    "# velocity gains x{gain:.3} of the configured values\n\
                     # [[joints]] #{}\n# [joints.gains]\n# kpv = {:.6}\n# kiv = {:.6}\n",
                    j + 1,
                    joint.gains.kpv * gain,
                    joint.gains.kiv * gain
                ));
            }
            if let Some(g) = grav {
                s.push_str(&format!(
                    "# gravity_scale = {g:.4}  (the config has {configured:.4})\n"
                ));
            }
            if let Some(ma) = seek {
                s.push_str(&format!(
                    "# [[homing.joints]] #{}\n# current_ma = {ma:.1}\n",
                    j + 1
                ));
            }
        }
        match std::fs::write(path, s) {
            Ok(()) => println!("  measurements written to {}", path.display()),
            Err(e) => eprintln!("  could not write {}: {e}", path.display()),
        }
    }
}

struct Arm {
    bus: Backend,
    state: BusState,
    robot: RobotConfig,
    homing: Vec<JointHoming>,
    conv: Vec<JointConversion>,
    /// Home offset actually in force per joint \[rad\].
    ///
    /// Not the joint's own `home_offset_rad`: a joint that homes against the
    /// GRIPPER body takes its offset from the active gripper, and using the
    /// fallback puts the reference degrees away from where the arm really is.
    /// The wrist pitch is 2.07 rad out with this tool fitted — it homes
    /// perfectly and then drives at a target that means nothing.
    home_offset: Vec<f64>,
    tick: u64,
    dt: f64,
    /// Velocity-gain scale in force per joint; 1.0 = the config's value.
    scales: Vec<f64>,
    /// Seek current in force per joint \[mA\], starting at the config's.
    ///
    /// Raised when a seek saturates: a drive that is asking for more current
    /// than the approach limit allows is not short of gain, it is short of
    /// current, and no velocity gain can add torque the current loop is not
    /// permitted to produce.
    seek_ma: Vec<f64>,
    /// Whether the last seek ended with the drive against its approach limit.
    seek_saturated: bool,
    /// Joints that shook during homing, when the gravity model was still the
    /// config's guess. Carried into step 3, which judges them with the
    /// measured feedforward in force.
    rang_while_homing: Vec<bool>,
    /// The joint currently held on torque alone, if any.
    ///
    /// It has been deliberately released from position control so its gravity
    /// feedforward can be measured, so whatever it does in that window is the
    /// measurement — not a verdict on its gains. The oscillation rule judges
    /// joints the tool is holding.
    torque_only: Option<usize>,
    /// Set for step 3 only: the arrival window tightens to the runtime's.
    verifying: bool,
    /// Set while the arm is being handed back. The park drives two joints onto
    /// their own endstops, where drooping into the stop is the POINT — and the
    /// gains have already been restored, so a move that "helpfully" raises
    /// them again leaves the arm on gains nobody measured.
    handing_over: bool,
    /// Joints whose gravity feedforward has not been measured yet, while step
    /// 2 is working through them. A joint cannot be held to "no oscillation"
    /// on a feedforward that is still the config's guess — but the moment its
    /// own scale is measured, it can, and is.
    awaiting_gravity: Vec<bool>,
    /// The lowest velocity-gain scale each joint has been SHOWN to move at.
    ///
    /// A quiet joint that cannot move is not tuned, it is switched off. The
    /// ring ladder may only choose from rungs at or above this, and it is
    /// raised whenever a move discovers that the current rung cannot carry
    /// the joint.
    scale_floor: Vec<f64>,
    /// Joints that have latched a reference.
    referenced: Vec<bool>,
    /// The encoder tick each joint's reference was latched AT, which is the
    /// thing whose repeatability matters. The joint angle afterwards is
    /// re-anchored to it every run, so comparing those would compare a
    /// number to itself.
    home_ticks: Vec<Option<i32>>,
    cadence: Cadence,
    last_tick_at: Option<Instant>,
    /// Where the watched joint was when it last actually moved, and when.
    motion_mark: Option<(i32, Instant)>,
    /// Where EVERY joint was the last time any of them moved, and when.
    /// The arm-wide rule is enforced from this. A fixed array, because it is
    /// rebuilt on every tick and the tick path does not allocate.
    still_mark: Option<([Option<i32>; par6_kin::NQ], Instant)>,
    /// Set only while stillness is the measurement: a hold proof and a
    /// gravity watch are asking whether the arm stays put, so the rule that
    /// staying put is a failure cannot apply to them. Both are bounded by
    /// their own watch length.
    measuring: bool,
    /// Armed once step 1 has finished. Step 1 IS the tuning, so a joint is
    /// allowed to ring while its gains are being found; past that point any
    /// oscillation means step 1 shipped a gain set that does not hold the
    /// arm, and the run is over.
    vibration_fatal: bool,
    /// The swing each joint showed while POSITION-held at its loaded pose
    /// \[ticks\], captured just before it is released for measurement.
    ///
    /// That is the state the oscillation rule judges, so it is the state the
    /// bar has to come from. Sampling it after the torque-only window instead
    /// measured the release, and clearing that window's history left nothing to
    /// measure at all.
    hold_swing: Vec<f64>,
    /// How long each joint has been continuously ringing \[ticks\].
    ///
    /// A verdict on one window is a verdict on a burst: the elbow's swings vary
    /// between windows, and a run of two minutes should not end because one of
    /// them was large. The rule fires on oscillation that is still there a
    /// window later.
    ringing_for: Vec<u32>,
    /// Each joint's own dither while it is holding a pose it has been
    /// calibrated for \[ticks\], measured rather than assumed.
    ///
    /// A threshold for "ringing" that sits below what the hardware does when it
    /// is working correctly fails every run on a healthy arm. The elbow swings
    /// 0.13 deg holding 2 A at full extension; the wrist swings a fraction of
    /// that. One constant cannot describe both, so each joint's own figure is
    /// taken at the moment its gravity scale is accepted and reported.
    ring_floor: Vec<f64>,
    /// Recent encoder history for EVERY joint, refreshed on every tick. The
    /// point of monitoring all of them is that a joint left ringing by stage 1
    /// shows up wherever it is, not only while it is the one being worked on.
    history: Vec<Vec<i32>>,
    /// The same window of TRACKING ERROR per joint \[ticks\]: measured
    /// position minus the position it was commanded to be at.
    ///
    /// Raw position cannot say whether a joint is shaking while it travels —
    /// a nudge that reverses, or a seek that stalls and backs off, swings the
    /// encoder by degrees on purpose. Against its own setpoint a joint doing
    /// what it was told is flat, however far or however fast it goes, so
    /// anything left is the joint disagreeing with the command.
    error_history: Vec<Vec<i32>>,
    /// The same window of drive current per joint \[mA\].
    ///
    /// A drive can chatter loudly while moving the encoder less than a tick,
    /// and at 250 Hz the position channel cannot see it at all. The current it
    /// takes to do that is visible here.
    current_history: Vec<Vec<i16>>,
    /// Scratch for the swing measurement, so the per-tick oscillation check
    /// does not allocate.
    swing_scratch: Vec<i64>,
    /// The loudest thing each joint has done so far, for the report.
    noise: Vec<Noise>,
    /// The next tick's absolute wake target on the monotonic clock.
    next_deadline: Duration,
    /// Where homing left the arm \[rad\]: every sweep pose is an offset from
    /// it, so the poses are reachable by construction.
    ready_pose: [f64; par6_kin::NQ],
    /// The gravity model, used on every tick of every move.
    ///
    /// Not only by step 2. A joint commanded with no feedforward has to
    /// generate its holding torque out of tracking error, so a loaded one
    /// CANNOT sit on its target — it hunts around it, and no gain fixes that
    /// because the error is the only thing producing the torque. The elbow
    /// hunting 0.06 deg at every gain was this. The runtime streams
    /// feedforward during normal motion; so does this.
    kin: par6_kin::Kin,
    /// Gravity feedforward per joint from the last tick \[mA\].
    last_ff: Vec<i16>,
    /// The frame being built, allocated once.
    ///
    /// This tool's tick path is an RT tick path — it paces the drives itself —
    /// and the house rule for those is that they allocate nothing after init. A
    /// six-element Vec per tick is cheap until the page it wants is not
    /// resident, and one run showed a 24 ms tick.
    cmds: Vec<JointCommand>,
    /// Gravity scale in force per joint; the config's until step 2 measures.
    grav_scale: Vec<f64>,
    /// Everything measured so far.
    results: Results,
}

impl Arm {
    fn open(
        bundle: &ConfigBundle,
        assets_dir: &std::path::Path,
        simulated: bool,
    ) -> Result<Self, String> {
        let robot = bundle.robot.clone();
        let mut bus = if simulated {
            let tool = bundle
                .active_gripper()
                .and_then(|g| g.urdf_variant.as_deref())
                .and_then(par6_bus::sim::scene::Tool::from_urdf_variant)
                .unwrap_or(par6_bus::sim::scene::Tool::Flange);
            Backend::Sim(SimBus::new(Scene {
                tool,
                assets: assets_dir.to_path_buf(),
            }))
        } else {
            Backend::Can(SocketCanBus::open(&robot.bus).map_err(|e| format!("CAN open: {e}"))?)
        };
        let gripper = bundle.grippers.iter().find(|g| g.driver.is_some());
        // A bus left error-passive by an earlier run cannot take a single
        // frame, so the first push fails and every run after it fails the same
        // way until somebody cycles the interface by hand. The bus layer
        // already knows how to recover itself; this asks it to, once, rather
        // than making a wedged link the operator's problem.
        if let Err(e) = bus
            .as_bus()
            .boot_configure(&robot, gripper, robot.bus.boot_config_repeats)
        {
            println!("  the bus would not take the config ({e}); cycling the interface");
            if !bus.as_bus().recover_link() {
                return Err(format!("config push: {e}, and the interface would not cycle"));
            }
            bus.as_bus()
                .boot_configure(&robot, gripper, robot.bus.boot_config_repeats)
                .map_err(|e| format!("config push after cycling the interface: {e}"))?;
        }
        // A drive with a latched fault ignores motion until it is cleared,
        // and a power cycle or a runtime that died holding leaves exactly
        // that. Without this the arm never moves and a joint gets blamed
        // for a fault nobody cleared.
        for j in &robot.joints {
            bus.as_bus()
                .send_clear_error(j.node_id, 3)
                .map_err(|e| format!("clear error on node {}: {e}", j.node_id))?;
        }
        let conv = robot.joints.iter().map(JointConversion::from_config).collect();
        let n = robot.joints.len();
        let home_offset: Vec<f64> = (0..n)
            .map(|i| {
                bundle
                    .effective_home_offset(i)
                    .unwrap_or(robot.homing.joints[i].home_offset_rad)
            })
            .collect();
        for (i, o) in home_offset.iter().enumerate() {
            let fallback = robot.homing.joints[i].home_offset_rad;
            if (o - fallback).abs() > 1e-9 {
                println!(
                    "  J{}: the active gripper sets its home offset to {o:.4} rad \
                     (joint default {fallback:.4})",
                    i + 1
                );
            }
        }
        let seek_ma: Vec<f64> = robot.homing.joints.iter().map(|h| h.current_ma).collect();
        let grav_scale: Vec<f64> = (0..n)
            .map(|j| robot.gravity_scale.get(j).copied().unwrap_or(1.0))
            .collect();
        Ok(Self {
            dt: robot.robot.tick_dt_s,
            homing: robot.homing.joints.clone(),
            robot,
            bus,
            state: BusState::new(),
            conv,
            home_offset,
            tick: 0,
            scales: vec![1.0; n],
            referenced: vec![false; n],
            home_ticks: vec![None; n],
            ring_floor: vec![0.0; n],
            ringing_for: vec![0; n],
            hold_swing: vec![0.0; n],
            cadence: Cadence::default(),
            last_tick_at: None,
            motion_mark: None,
            still_mark: None,
            seek_ma,
            seek_saturated: false,
            rang_while_homing: vec![false; n],
            awaiting_gravity: vec![false; n],
            torque_only: None,
            verifying: false,
            handing_over: false,
            scale_floor: vec![0.0; n],
            measuring: false,
            vibration_fatal: false,
            history: vec![Vec::with_capacity(WATCH_WINDOW_TICKS + 1); n],
            error_history: vec![Vec::with_capacity(WATCH_WINDOW_TICKS + 1); n],
            current_history: vec![Vec::with_capacity(WATCH_WINDOW_TICKS + 1); n],
            swing_scratch: Vec::with_capacity(WATCH_WINDOW_TICKS + 1),
            noise: (0..n).map(|_| Noise::default()).collect(),
            next_deadline: monotonic_now(),
            ready_pose: [0.0; par6_kin::NQ],
            kin: gravity_model(assets_dir, gripper)?,
            last_ff: vec![0; n],
            cmds: vec![JointCommand::idle(); n],
            grav_scale: grav_scale.to_vec(),
            results: Results::new(n),
        })
    }

    fn n(&self) -> usize {
        self.robot.joints.len()
    }

    fn node(&self, joint: usize) -> NodeId {
        self.robot.joints[joint].node_id
    }

    fn position(&self, joint: usize) -> Option<i32> {
        self.state.nodes[usize::from(self.node(joint))].position_ticks
    }

    fn current_ma(&self, joint: usize) -> Option<i16> {
        self.state.nodes[usize::from(self.node(joint))].current_ma
    }

    /// Push a node's limits until the queue takes them.
    ///
    /// Unlike a setpoint, a limit is sent once rather than every tick, so a
    /// dropped one stays dropped — and the one that matters is the RESTORE
    /// after a seek. A joint left on its seek current cannot hold a load at
    /// any gain, which looks exactly like a tuning problem and is not one.
    fn insist_limits(&mut self, node: NodeId, vel: f32, ilim: f32) -> Result<(), String> {
        self.cadence.phase("limit push");
        // A full queue is drained by NOT sending. Earlier versions retried
        // through the tick loop, which adds six joint frames, a gripper frame
        // and a poll on every attempt — so the wait made the queue it was
        // waiting on worse, and forty attempts later the limit still had not
        // gone out. Here the tool goes quiet: no frames at all until the
        // kernel has room. The drives tolerate that for far longer than this
        // takes; their watchdog is seconds, not milliseconds.
        let mut quiet_ticks = 0u32;
        for attempt in 0..LIMIT_ATTEMPTS {
            match self.bus.as_bus().send_limits(node, vel, ilim, LIMIT_REPEATS as u8) {
                Ok(()) => return Ok(()),
                Err(BusError::TxQueueFull) if attempt + 1 < LIMIT_ATTEMPTS => {
                    quiet_ticks += 1;
                    let mut deadline = monotonic_now();
                    deadline += Duration::from_secs_f64(self.dt);
                    sleep_until(deadline);
                    // Keep the tick base honest: the loop was paused, so the
                    // next tick's deadline is from now, not from a schedule
                    // that kept running while nothing was sent.
                    self.next_deadline = monotonic_now();
                }
                Err(e) => return Err(format!("node {node} limits ({ilim:.0} mA): {e}")),
            }
        }
        Err(format!(
            "node {node}: the transmit queue stayed full through {quiet_ticks} quiet ticks, \
             so the {ilim:.0} mA limit never went out"
        ))
    }

    /// Put every joint back on its configured gains and keep holding.
    ///
    /// Run on every exit, including a failure. A joint left on a reduced gain
    /// is a joint that cannot carry its own weight, and a process that simply
    /// returns stops feeding the drives, so their watchdogs time out and the
    /// arm goes limp wherever it happens to be — which is how the elbow came
    /// down. Restoring first and holding afterwards hands the arm over in a
    /// state somebody can take.
    fn restore(&mut self, hold_s: f64) {
        self.handing_over = true;
        // Only the two gains this tool ever moved. Resending a node's whole
        // stored config reinstalls its mode as well, which is a disturbance
        // the arm does not need at the moment it is handed over.
        for j in 0..self.n() {
            if self.scales[j] == 1.0 {
                continue;
            }
            println!("  J{}: velocity gains back to the configured values", j + 1);
            if let Err(e) = self.push_scale(j, 1.0) {
                eprintln!("  J{}: could not restore its gains: {e}", j + 1);
            }
        }
        // Park the shoulder and elbow back on their own endstops. The
        // driver's watchdog action is the vendor's Idle (cmd 12), which
        // de-energises the motor, and the firmware offers no brake — so once
        // this process stops sending, whatever is loaded falls. On their home
        // references those two rest against their stops, where the mechanism
        // holds them and the drives do not have to. The elbow folds first, so
        // the upper arm comes down over a folded forearm.
        for joint in [2usize, 1usize] {
            if !self.referenced[joint] {
                println!("  J{}: no reference, so it cannot be parked", joint + 1);
                continue;
            }
            let home = self.home_offset[joint];
            println!("  J{}: back to its home reference at {home:.4} rad", joint + 1);
            let s = self.travel_time(joint, home);
            if let Err(e) = self.move_to(joint, home, s) {
                eprintln!("  J{}: could not park on its stop: {e}", joint + 1);
                break;
            }
        }
        if let Err(e) = self.hold_all(hold_s) {
            eprintln!("  could not hold the arm while handing over: {e}");
        }
        // The vendor's own brake: a current-only pack carrying zero leaves
        // the phases energised under the current loop, where cmd 12 would
        // drop them. It lasts only while this tool is still sending, and the
        // driver's watchdog offers nothing but cmd 12, so a loaded joint will
        // fall once this process is gone. Said out loud rather than hidden.
        println!("  releasing on the brake pack");
        self.measuring = true;
        let ticks = (RELEASE_S / self.dt).round().max(1.0) as u64;
        for _ in 0..ticks {
            if let Err(e) = self.tick_each(|_, _| JointCommand::current(0)) {
                eprintln!("  release: {e}");
                break;
            }
            let mut unused = Instant::now();
            self.sleep_to(&mut unused);
        }
    }

    /// Hold every joint that has a reading, for `seconds`.
    fn hold_all(&mut self, seconds: f64) -> Result<(), String> {
        self.measuring = true;
        let ticks = (seconds / self.dt).round().max(1.0) as u64;
        let mut next_tick = Instant::now();
        let targets: Vec<Option<i32>> = (0..self.n()).map(|j| self.position(j)).collect();
        for _ in 0..ticks {
            self.tick_each(|_, j| match targets[j] {
                Some(p) => JointCommand::position(p, 0, 0),
                None => JointCommand::idle(),
            })?;
            self.sleep_to(&mut next_tick);
        }
        self.measuring = false;
        Ok(())
    }

    /// Whether gravity can pull this joint off a held target.
    ///
    /// The base turns about the vertical, so gravity has no moment about its
    /// axis: it can be under-driven or it can ring, but it cannot droop.
    /// Reading its slow tracking as droop and raising its gains is what set
    /// it ringing.
    fn gravity_can_load(&self, joint: usize) -> bool {
        joint != 0
    }

    /// Encoder ticks per radian of joint motion.
    fn ticks_per_rad(&self, joint: usize) -> f64 {
        let j = &self.robot.joints[joint];
        f64::from(1i32 << j.encoder_bits) * j.gear_ratio / std::f64::consts::TAU
    }

    fn ticks_for_deg(&self, joint: usize, deg: f64) -> i64 {
        (deg.to_radians() * self.ticks_per_rad(joint)).round() as i64
    }

    fn deg_for_ticks(&self, joint: usize, ticks: i64) -> f64 {
        (ticks as f64 / self.ticks_per_rad(joint)).to_degrees()
    }

    /// A full TX queue is this tick's frames not fitting, not a failed run:
    /// commands are re-sent every tick and the telemetry poll is a round
    /// robin that loses nothing by skipping a turn.
    fn tolerate(e: BusError) -> Result<(), BusError> {
        if matches!(e, BusError::TxQueueFull) {
            Ok(())
        } else {
            Err(e)
        }
    }

    /// What counts as a ring on this joint \[ticks\]: the configured bar, or
    /// comfortably more than this joint's own measured dither, whichever is
    /// larger.
    fn ring_threshold(&self, joint: usize) -> i64 {
        let absolute = self.ticks_for_deg(joint, RING_AMPLITUDE_DEG);
        let measured = (self.ring_floor[joint] * RING_OVER_FLOOR) as i64;
        absolute.max(measured)
    }

    /// Every joint that is oscillating right now, with its excursion \[deg\].
    ///
    /// Checked continuously rather than at checkpoints: a joint that rings
    /// only while another joint is being driven would never be caught by a
    /// test that looks at one joint at a time.
    fn oscillating(&self) -> Vec<(usize, f64)> {
        (0..self.history.len())
            .filter_map(|j| {
                let h = &self.history[j];
                if h.len() < WATCH_WINDOW_TICKS {
                    return None;
                }
                if !ringing_now(h, self.ring_threshold(j)) {
                    return None;
                }
                Some((j, self.deg_for_ticks(j, ring_swings(h).1 as i64)))
            })
            .collect()
    }

    /// Watch a joint while it is being driven, and give up on it the moment
    /// it stops moving for [`NO_MOTION_ABORT_S`].
    ///
    /// Returns true while there is still motion to wait for.
    fn still_moving(&mut self, joint: usize, tolerance_ticks: i64) -> bool {
        self.moving_within(joint, tolerance_ticks, NO_MOTION_ABORT_S)
    }

    /// Forget where the last watched motion was, so the next operation starts
    /// its own watch.
    fn reset_motion_watch(&mut self) {
        self.motion_mark = None;
        self.still_mark = None;
        // Seeks and moves all start here, so an error that skipped a
        // measurement window's cleanup cannot leave either rule switched off.
        self.measuring = false;
        self.torque_only = None;
    }

    /// Whether `joint` has moved within the last `window_s`.
    fn moving_within(&mut self, joint: usize, tolerance_ticks: i64, window_s: f64) -> bool {
        let now = self.position(joint);
        match (now, self.motion_mark) {
            (Some(p), Some((marked, at))) => {
                if (i64::from(p) - i64::from(marked)).abs() > tolerance_ticks {
                    self.motion_mark = Some((p, Instant::now()));
                    true
                } else {
                    at.elapsed().as_secs_f64() < window_s
                }
            }
            (Some(p), None) => {
                self.motion_mark = Some((p, Instant::now()));
                true
            }
            (None, _) => true,
        }
    }

    /// The rule: the arm does not stand still.
    ///
    /// If no joint has changed angle for [`NO_MOTION_ABORT_S`], the run has
    /// failed, and it fails here — on the one path every command goes through
    /// — rather than at each place that might have decided its own wait was
    /// reasonable. Those per-site judgements are what let a seek lean on a
    /// joint that was never going to move, and a profile run out its span
    /// after the arm had already stopped.
    fn enforce_rules(&mut self) -> Result<(), String> {
        self.enforce_quiet()?;
        self.enforce_motion()
    }

    /// Measure one joint's noise, in both channels, and keep the worst.
    ///
    /// One joint per tick, round-robin: every joint is measured every six
    /// ticks, which is twenty-four milliseconds and far finer than anything
    /// the ear is being asked to match, at a sixth of the cost of doing all
    /// of them every tick.
    fn sample_noise(&mut self) {
        let n = self.history.len();
        if n == 0 {
            return;
        }
        let j = self.tick as usize % n;
        if self.error_history[j].len() < WATCH_WINDOW_TICKS {
            return;
        }
        let mut turns = std::mem::take(&mut self.swing_scratch);
        let pos = swings_into(&self.error_history[j], &mut turns);
        let cur = swings_into(&self.current_history[j], &mut turns);
        self.swing_scratch = turns;
        // Travelling or holding, decided from the joint itself rather than
        // from what the tool believes it asked for: an approach that has ended
        // and a joint that was never commanded look the same from here, and
        // both are holds.
        let moving = span_ticks(&self.history[j]) > self.arrival_tol(j);
        // Disjoint fields, so the phase can be read while the record is
        // written.
        let phase = &self.cadence.phase;
        let noise = &mut self.noise[j];
        if pos.reversals >= RING_REVERSALS {
            let (worst, seen_in) = if moving {
                (&mut noise.moving, &mut noise.moving_phase)
            } else {
                (&mut noise.holding, &mut noise.holding_phase)
            };
            if pos.worst > *worst {
                *worst = pos.worst;
                seen_in.clear();
                seen_in.push_str(phase);
            }
        }
        if cur.reversals >= RING_REVERSALS && cur.worst > noise.ripple {
            noise.ripple = cur.worst;
            noise.ripple_phase.clear();
            noise.ripple_phase.push_str(phase);
        }
    }

    /// The two loudest joints right now, as one line.
    ///
    /// Printed where the arm is standing still and somebody is listening, so
    /// what is heard and what is measured can be put side by side.
    fn noise_line(&mut self) -> String {
        let mut turns = std::mem::take(&mut self.swing_scratch);
        let mut rows: Vec<(usize, f64, f64)> = Vec::new();
        for j in 0..self.history.len() {
            if self.error_history[j].len() < WATCH_WINDOW_TICKS {
                continue;
            }
            let pos = swings_into(&self.error_history[j], &mut turns);
            let cur = swings_into(&self.current_history[j], &mut turns);
            rows.push((j, pos.median, cur.median));
        }
        self.swing_scratch = turns;
        rows.sort_by(|a, b| b.1.total_cmp(&a.1));
        rows.truncate(2);
        rows.iter()
            .map(|(j, swing, ripple)| {
                format!(
                    "J{} {:.3} deg / {ripple:.0} mA",
                    j + 1,
                    self.deg_for_ticks(*j, *swing as i64)
                )
            })
            .collect::<Vec<_>>()
            .join(", ")
    }

    /// The second rule: past step 1, the arm does not shake.
    ///
    /// Every tick, every joint — not at checkpoints. A joint that only rings
    /// while a different joint is being driven is invisible to a check that
    /// looks at the joint under test, and that is the ring that was reported
    /// from the room while the tool reported success.
    fn enforce_quiet(&mut self) -> Result<(), String> {
        if !self.vibration_fatal || self.handing_over {
            // Handing back is parking two joints onto their stops and letting
            // go; a verdict there stops the park and leaves the arm worse.
            return Ok(());
        }
        // Asked as a question first, because this runs on every tick: the list
        // is only built once something is actually wrong.
        let mut sustained = false;
        let mut turns = std::mem::take(&mut self.swing_scratch);
        for j in 0..self.history.len() {
            let ringing = Some(j) != self.torque_only
                && !self.awaiting_gravity[j]
                && self.history[j].len() >= WATCH_WINDOW_TICKS
                && ringing_now_into(&self.history[j], self.ring_threshold(j), &mut turns);
            self.ringing_for[j] = if ringing { self.ringing_for[j] + 1 } else { 0 };
            sustained |= self.ringing_for[j] as usize >= RING_PERSIST_TICKS;
        }
        self.swing_scratch = turns;
        if !sustained {
            return Ok(());
        }
        let ringing: Vec<(usize, f64)> = self
            .oscillating()
            .into_iter()
            .filter(|(j, _)| {
                self.ringing_for[*j] as usize >= RING_PERSIST_TICKS
                    && Some(*j) != self.torque_only
                    && !self.awaiting_gravity[*j]
            })
            .collect();
        if ringing.is_empty() {
            return Ok(());
        }
        let named: Vec<String> = ringing
            .iter()
            .map(|(j, deg)| format!("J{} at {deg:.3} deg", j + 1))
            .collect();
        Err(format!(
            "oscillation after step 1 during '{}': {} — step 1's gains do not \
             hold this arm",
            self.cadence.phase,
            named.join(", ")
        ))
    }

    fn enforce_motion(&mut self) -> Result<(), String> {
        if self.measuring {
            self.still_mark = None;
            return Ok(());
        }
        let mut now = [None; par6_kin::NQ];
        for (j, slot) in now.iter_mut().enumerate().take(self.n()) {
            *slot = self.position(j);
        }
        let Some((marked, since)) = self.still_mark.take() else {
            self.still_mark = Some((now, Instant::now()));
            return Ok(());
        };
        let moved = (0..self.n().min(par6_kin::NQ)).any(|j| {
            match (now.get(j).copied().flatten(), marked.get(j).copied().flatten()) {
                (Some(a), Some(b)) => {
                    (i64::from(a) - i64::from(b)).abs() > self.ticks_for_deg(j, STILL_DEG).max(1)
                }
                // A joint that has only just started reporting counts as
                // movement: the arm is not the thing that is stuck.
                (Some(_), None) => true,
                _ => false,
            }
        });
        if moved {
            self.still_mark = Some((now, Instant::now()));
            return Ok(());
        }
        let held = since.elapsed().as_secs_f64();
        if held >= NO_MOTION_ABORT_S {
            // Only on the way out, where one allocation costs nothing.
            let where_at: Vec<String> = (0..self.n())
                .filter_map(|j| {
                    self.position(j)
                        .map(|p| format!("J{}={:.2} deg", j + 1, self.conv[j].joint_rad(p).to_degrees()))
                })
                .collect();
            return Err(format!(
                "no joint has moved for {held:.1} s during '{}' — the arm stood                  still at {}",
                self.cadence.phase,
                where_at.join(", ")
            ));
        }
        self.still_mark = Some((marked, since));
        Ok(())
    }

    /// One bus tick: read, send `cmds`, feed the gripper and poll slots.
    fn exchange(&mut self, cmds: &[JointCommand]) -> Result<(), String> {
        let now = Instant::now();
        if let Some(previous) = self.last_tick_at {
            self.cadence.record(now - previous);
        }
        self.last_tick_at = Some(now);
        self.tick += 1;
        self.bus.as_bus().begin_tick(self.tick);
        self.bus
            .as_bus()
            .drain_rx(&mut self.state)
            .map_err(|e| format!("rx: {e}"))?;
        for j in 0..self.history.len() {
            if let Some(p) = self.state.nodes[usize::from(self.robot.joints[j].node_id)].position_ticks
            {
                self.history[j].push(p);
                if self.history[j].len() > WATCH_WINDOW_TICKS {
                    self.history[j].remove(0);
                }
                match cmds.get(j).and_then(|c| c.pos) {
                    Some(target) => {
                        self.error_history[j].push(p.saturating_sub(target));
                        if self.error_history[j].len() > WATCH_WINDOW_TICKS {
                            self.error_history[j].remove(0);
                        }
                    }
                    // No setpoint this tick — a velocity push, or a joint let
                    // go on torque only. There is no error to measure, and
                    // stitching the next position-held window onto this one
                    // would read the change of regime as a swing.
                    None => self.error_history[j].clear(),
                }
            }
            if let Some(c) = self.state.nodes[usize::from(self.robot.joints[j].node_id)].current_ma {
                self.current_history[j].push(c);
                if self.current_history[j].len() > WATCH_WINDOW_TICKS {
                    self.current_history[j].remove(0);
                }
            }
        }
        self.sample_noise();
        self.tick_gravity();
        self.enforce_rules()?;
        self.bus
            .as_bus()
            .send_joint_commands(cmds)
            .or_else(Self::tolerate)
            .map_err(|e| format!("tx: {e}"))?;
        self.bus
            .as_bus()
            .send_gripper(&GripperCommand::NoGripper)
            .or_else(Self::tolerate)
            .map_err(|e| format!("gripper slot: {e}"))?;
        self.bus
            .as_bus()
            .poll_step()
            .or_else(Self::tolerate)
            .map_err(|e| format!("poll: {e}"))?;
        Ok(())
    }

    /// The arrival window for `joint` \[ticks\].
    ///
    /// Every move here is getting the arm into position so something can be
    /// measured there, and a tenth of a degree of elbow changes the wrist's
    /// gravity load by nothing. What this tool claims is measured, not
    /// positional: the drift rate on a torque-only hold, and that nothing
    /// oscillates. Holding the APPROACH to the runtime's own 0.06 deg window
    /// instead failed the elbow twice for dithering 0.12 deg at a pose where
    /// it carries 2 A — which no gain fixes, and which says nothing about
    /// whether the pose was reached.
    fn arrival_tol(&self, _joint: usize) -> i64 {
        POS_TOL_TICKS * POSE_TOL_FACTOR
    }

    /// Recompute the gravity feedforward from where the arm IS.
    ///
    /// Only referenced joints contribute and only they receive it: an
    /// unreferenced joint's angle is an arbitrary encoder count, and feeding
    /// the model arbitrary angles produces arbitrary torques.
    fn tick_gravity(&mut self) {
        if !self.referenced.iter().any(|r| *r) {
            self.last_ff.iter_mut().for_each(|f| *f = 0);
            return;
        }
        let mut q = [0.0_f64; par6_kin::NQ];
        for (j, angle) in q.iter_mut().enumerate().take(self.n()) {
            if let Some(p) = self.position(j) {
                *angle = self.conv[j].joint_rad(p);
            }
        }
        let mut g = [0.0_f64; par6_kin::NQ];
        if self.kin.gravity(&q, &mut g).is_err() {
            self.last_ff.iter_mut().for_each(|f| *f = 0);
            return;
        }
        // A fixed array, not a Vec: this runs on every tick of a loop that is
        // pacing drives, and the heap is not something to touch 250 times a
        // second for six numbers.
        let mut ff = [0_i16; par6_kin::NQ];
        for (j, out) in ff.iter_mut().enumerate().take(self.n()) {
            if self.referenced[j] {
                *out = self.torque_to_ma(j, self.grav_scale[j] * g[j]);
            }
        }
        let n = self.n().min(par6_kin::NQ);
        self.last_ff[..n].copy_from_slice(&ff[..n]);
    }

    /// Every joint idle except `active`, which holds whatever the caller
    /// built. A joint with a reference holds position so it does not sag
    /// while another joint works.
    fn fill_frame(&self, cmds: &mut [JointCommand], active: Option<(usize, JointCommand)>) {
        for (j, cmd) in cmds.iter_mut().enumerate() {
            *cmd = if self.referenced[j] {
                match self.position(j) {
                    Some(p) => JointCommand::position(p, 0, self.last_ff[j]),
                    None => JointCommand::idle(),
                }
            } else {
                JointCommand::idle()
            };
        }
        if let Some((j, cmd)) = active {
            cmds[j] = cmd;
        }
    }

    /// One tick: build the standard frame into the owned buffer and send it.
    fn tick_frame(&mut self, active: Option<(usize, JointCommand)>) -> Result<(), String> {
        let mut buf = std::mem::take(&mut self.cmds);
        buf.resize(self.n(), JointCommand::idle());
        self.fill_frame(&mut buf, active);
        let sent = self.exchange(&buf);
        self.cmds = buf;
        sent
    }

    /// One tick, with every joint's command decided by `build`.
    fn tick_each(
        &mut self,
        mut build: impl FnMut(&Self, usize) -> JointCommand,
    ) -> Result<(), String> {
        let mut buf = std::mem::take(&mut self.cmds);
        buf.resize(self.n(), JointCommand::idle());
        for (j, cmd) in buf.iter_mut().enumerate() {
            *cmd = build(self, j);
        }
        let sent = self.exchange(&buf);
        self.cmds = buf;
        sent
    }

    /// Wait for the next tick boundary. Absolute deadlines, so an early wake
    /// is absorbed rather than accumulating drift; a missed one re-bases
    /// instead of firing a catch-up burst at the drives.
    fn sleep_to(&mut self, _unused: &mut Instant) {
        let period = Duration::from_secs_f64(self.dt);
        self.next_deadline += period;
        let now = monotonic_now();
        if self.next_deadline <= now {
            self.next_deadline = now + period;
        }
        sleep_until(self.next_deadline);
    }

    /// How long a move of this distance should take \[s\].
    ///
    /// From the distance and a comfortable rate, not a flat allowance: a one
    /// degree move given three seconds is nearly three seconds of an arm
    /// going nowhere, repeated once per joint per pose.
    fn travel_time(&self, joint: usize, target_rad: f64) -> f64 {
        let now = self
            .position(joint)
            .map(|p| self.conv[joint].joint_rad(p))
            .unwrap_or(target_rad);
        let distance = (target_rad - now).abs();
        (distance / MOVE_RAD_S + MOVE_RAMP_S).clamp(MOVE_MIN_S, MOVE_MAX_S)
    }

    /// Wait until every joint has reported a position, and no longer.
    ///
    /// The thing being waited for is a reading, so that is what is waited on.
    /// A flat second spent hoping is a second of a still arm.
    fn wait_for_readings(&mut self) -> Result<(), String> {
        // Nothing is being driven yet, so nothing can be expected to move.
        self.measuring = true;
        let cap = (BOOT_READING_CAP_S / self.dt).round().max(1.0) as u64;
        for k in 0..cap {
            self.cadence.phase("waiting for first readings");
            self.tick_frame(None)?;
            let mut unused = Instant::now();
            self.sleep_to(&mut unused);
            if (0..self.n()).all(|j| self.position(j).is_some()) {
                self.measuring = false;
                return Ok(());
            }
            // Halfway through, try clearing again. A drive left latched by a
            // previous run answers nothing until it is cleared, and one clear
            // at startup can land while the bus is still coming up.
            if k == cap / 2 {
                for j in 0..self.n() {
                    let node = self.node(j);
                    let _ = self.bus.as_bus().send_clear_error(node, 1);
                }
            }
        }
        // Name them. "Something did not report" is not a diagnosis, and the
        // silent node is the whole content of the failure.
        let silent: Vec<String> = (0..self.n())
            .filter(|j| self.position(*j).is_none())
            .map(|j| {
                let node = self.node(j);
                let s = &self.state.nodes[usize::from(node)];
                format!(
                    "J{} (node {node}, fault bit {}, flags {:?}, age {} ticks)",
                    j + 1,
                    s.live_error_bit,
                    s.error_flags,
                    s.data_age_ticks
                )
            })
            .collect();
        Err(format!("no position reported by {}", silent.join(", ")))
    }

    /// Keep the bus alive for `seconds` with nothing new commanded.
    fn coast(&mut self, seconds: f64) -> Result<(), String> {
        self.cadence.phase("coasting");
        // Coasting is commanded stillness, so the rule would be right to fire
        // on it and wrong to. It is capped below the rule's second instead,
        // which is also all any of the vendor's dwells need.
        let seconds = seconds.min(NO_MOTION_ABORT_S * 0.8);
        let ticks = (seconds / self.dt).round().max(1.0) as u64;
        let mut next = Instant::now();
        for _ in 0..ticks {
            self.tick_frame(None)?;
            self.sleep_to(&mut next);
        }
        Ok(())
    }

    /// Push one joint's velocity-loop gains, scaled; nothing else moves.
    fn push_scale(&mut self, joint: usize, scale: f64) -> Result<(), String> {
        self.cadence.phase("gain push");
        let j = &self.robot.joints[joint];
        let mut gains = j.gains;
        gains.kpv = j.gains.kpv * scale;
        gains.kiv = j.gains.kiv * scale;
        let tune = DriveTune {
            gains,
            ilim_ma: j.ilim_ma,
            velocity_limit_ticks_s: j.velocity_limit_ticks_s,
            voltage_limit_mv: j.voltage_limit_mv,
        };
        let node = j.node_id;
        // A full queue here is the same transient it is anywhere else, and
        // the repeats mean the push is not lost; aborting the run for it
        // throws away everything homed so far.
        self.bus
            .as_bus()
            .retune_node(node, &tune, 3)
            .or_else(Self::tolerate)
            .map_err(|e| format!("retune J{}: {e}", joint + 1))?;
        self.scales[joint] = scale;
        // Deliberately not recorded here. Pushing a gain is a trial, not a
        // measurement: the scale the search happened to be on when a run died
        // was being written into the config as though the arm had been tuned
        // to it — J3 went out at x2.44, the value that had just failed. A
        // scale becomes a result in `move_to`, when a joint arrives on it.
        self.coast(f64::from(CONFIG_SETTLE_TICKS) * self.dt)
    }

    /// Move one joint's gains toward making it arrive.
    ///
    /// The two failures are opposites. A joint that never came near its
    /// target is under-driven: raise a step. A joint that reached the
    /// neighbourhood and would not stay still is ringing, and for that the
    /// answer is not a direction but a measurement.
    fn retune_toward_arrival(
        &mut self,
        joint: usize,
        ringing: bool,
        saturated: bool,
    ) -> Result<f64, String> {
        if ringing {
            return self.quieter_scale(joint);
        }
        // A joint pulling its full current limit and still not arriving is not
        // short of gain: the loop is already asking for everything the drive
        // can give. More gain asks harder for a current that is already capped,
        // so it changes nothing and the arm sits there being pushed.
        if saturated {
            return Err(format!(
                "J{} is at its current limit and still has not arrived — it is \
                 blocked or loaded beyond what the drive can hold, not under-driven",
                joint + 1
            ));
        }
        let current = self.scales[joint];
        let next = current * GAIN_STEP;
        if next > GAIN_CEILING {
            return Err(format!(
                "J{} will not reach its position with velocity gains at x{current:.2}; \
                 raising them further is not the answer",
                joint + 1
            ));
        }
        println!("    J{} is under-driven: velocity gains -> x{next:.2}", joint + 1);
        self.push_scale(joint, next)?;
        Ok(next)
    }

    /// Step down one rung of [`RING_LADDER`], never below what the joint has
    /// been shown to move on.
    ///
    /// This used to hold the joint at every rung and keep the quietest — which
    /// measured stillness by standing still, three times over, and stillness
    /// is the one thing this tool is not allowed to do. The ring is judged
    /// while the joint MOVES instead (see `move_to`), so the response to one
    /// is a single step down and straight back into motion.
    fn quieter_scale(&mut self, joint: usize) -> Result<f64, String> {
        let current = self.scales[joint];
        let floor = self.scale_floor[joint];
        let next = RING_LADDER
            .iter()
            .copied()
            .find(|s| *s < current - 1e-9 && *s >= floor);
        match next {
            Some(s) => {
                println!(
                    "    J{} is ringing at x{current:.2}: velocity gains -> x{s:.2}",
                    joint + 1
                );
                self.push_scale(joint, s)?;
                Ok(s)
            }
            None => Err(format!(
                "J{} rings at x{current:.2} and every rung below it is one this joint \
                 cannot move on (floor x{floor:.2}) — its gains are not the problem",
                joint + 1
            )),
        }
    }

    /// Velocity push for a duration, as a `nudge` pre-move.
    fn nudge(&mut self, joint: usize, speed_ticks_s: f64, duration_s: f64) -> Result<(), String> {
        let ticks = (duration_s / self.dt).round().max(1.0) as u64;
        let cur = self.homing[joint].current_ma as i16;
        let mut next = Instant::now();
        self.reset_motion_watch();
        for _ in 0..ticks {
            let cmd = JointCommand::velocity(speed_ticks_s as i32, cur);
            self.tick_frame(Some((joint, cmd)))?;
            self.sleep_to(&mut next);
            // A nudge unloads a joint; it is not a measurement and it has no
            // target. Once the joint has stopped moving it has done everything
            // it is going to do, and the rest of its configured duration is an
            // arm standing still — which is how a joint already against its
            // limit spent 1.8 s going nowhere.
            if !self.moving_within(joint, STALL_IDLE_TICKS, SEEK_STILL_S) {
                println!("    J{} nudge: it has stopped moving, so that is done", joint + 1);
                break;
            }
        }
        Ok(())
    }

    /// Seek one joint to its endstop by stall, once. `Ok(ticks)` is the
    /// latched endstop position; `Ok(None)` means the approach timed out.
    fn seek_once(&mut self, joint: usize) -> Result<Option<i32>, String> {
        match self.homing[joint].strategy {
            HomingStrategy::Hall => self.seek_hall(joint),
            HomingStrategy::Stall => self.seek_stall(joint),
        }
    }

    /// Find a joint's reference on its HALL sensor.
    ///
    /// This joint has no endstop: its reference is a sensor edge, and the
    /// position is latched AT the trigger. Pushing it until it stalls — which
    /// is what a stall seek does — drives it into a hard limit it was never
    /// meant to reach, and the harder it is pushed the less repeatable that
    /// stop becomes. The approach also guards the case where the joint starts
    /// already on the sensor: an immediate trigger is not a find, so it backs
    /// off until the band reads clear and approaches again.
    fn seek_hall(&mut self, joint: usize) -> Result<Option<i32>, String> {
        let h = self.homing[joint].clone();
        let sign = if h.direction == 1 { -1.0 } else { 1.0 };
        let speed = (sign * h.speed_ticks_s) as i32;
        let node = self.node(joint);
        let ilim = self.robot.joints[joint].ilim_ma as f32;
        let vel_limit = self.robot.joints[joint].velocity_limit_ticks_s as f32;
        self.cadence.phase("seek");
        self.insist_limits(node, vel_limit, h.current_ma as f32)?;
        let ticks = (h.timeout_s / self.dt).round().max(1.0) as u64;
        let mut started_clear = false;
        let mut outcome = None;
        let mut next_tick = Instant::now();
        for t in 0..ticks {
            let cmd = JointCommand::hall(speed, HALL_TRIGGER_VALUE);
            self.tick_frame(Some((joint, cmd)))?;
            self.sleep_to(&mut next_tick);
            let hall = self.state.nodes[usize::from(node)].hall;
            let Some(hall) = hall else { continue };
            if hall.trigger && !hall.edge {
                // Off the sensor: whatever the trigger says later, this
                // approach began clear and its find can be trusted.
                started_clear = true;
                continue;
            }
            let found = !hall.trigger || hall.edge;
            if !found {
                continue;
            }
            if t <= u64::from(HALL_PRECLEAR_TICKS) && !started_clear {
                println!("    J{} began on its sensor; backing off first", joint + 1);
                let back = (-sign * h.speed_ticks_s) as i32;
                let backoff = (h.backoff_s / self.dt).round().max(1.0) as u64;
                for _ in 0..backoff {
                    let cmd = JointCommand::hall(back, HALL_TRIGGER_VALUE);
                    self.tick_frame(Some((joint, cmd)))?;
                    self.sleep_to(&mut next_tick);
                }
                started_clear = true;
                continue;
            }
            let latched = self.position(joint);
            println!(
                "    J{} found its hall edge at {:?} ticks",
                joint + 1,
                latched
            );
            outcome = latched;
            break;
        }
        self.insist_limits(node, vel_limit, ilim)?;
        if outcome.is_none() {
            println!("    J{} saw no hall edge within its timeout", joint + 1);
        }
        Ok(outcome)
    }

    /// Find a joint's endstop by driving into it until the encoder stops.
    fn seek_stall(&mut self, joint: usize) -> Result<Option<i32>, String> {
        let h = self.homing[joint].clone();
        let sign = if h.direction == 1 { -1.0 } else { 1.0 };
        let speed = (sign * h.speed_ticks_s) as i32;
        let seek_ma = self.seek_ma[joint];
        let threshold = seek_ma * STALL_CURRENT_FRACTION;
        let ticks = (h.timeout_s / self.dt).round().max(1.0) as u64;
        // The seek current is the drive's LIMIT for the approach, not a
        // feedforward: the joint is meant to push until the endstop stops
        // it, with the limit deciding how hard. Injecting it on the wire
        // instead would drive a joint that is already on its stop.
        let node = self.node(joint);
        let normal_ilim = self.robot.joints[joint].ilim_ma as f32;
        let vel_limit = self.robot.joints[joint].velocity_limit_ticks_s as f32;
        self.insist_limits(node, vel_limit, seek_ma as f32)?;
        let mut ring: Vec<i32> = Vec::with_capacity(STALL_WINDOW_TICKS as usize + 1);
        let mut peak_current = 0.0_f64;
        let mut next = Instant::now();
        let mut outcome = None;
        self.reset_motion_watch();
        // A seek that has travelled further than the joint's own range has not
        // found its stop and will not: either it is turning the wrong way or
        // there is nothing to find. Grinding on for the rest of a twenty second
        // timeout is the arm standing still as far as anybody watching is
        // concerned.
        let range_rad = (self.robot.joints[joint].limits.hard_max_rad
            - self.robot.joints[joint].limits.hard_min_rad)
            .abs();
        let range_ticks = self.ticks_for_deg(joint, range_rad.to_degrees()) + SEEK_RANGE_SLACK_TICKS;
        let mut began_at = self.position(joint);
        let mut last_seen = began_at;
        let most_per_tick =
            (vel_limit as f64 * self.dt * IMPLAUSIBLE_STEP_FACTOR).max(1.0) as i64;
        let min_travel = self.ticks_for_deg(joint, SEEK_MIN_TRAVEL_DEG);
        let mut weak = false;
        // The fastest this approach was ever seen to go. A stop is something a
        // joint arrives at, so it has to have been going somewhere first.
        let mut best_making = 0.0_f64;
        let launch_deadline = (SEEK_LAUNCH_S / self.dt).round().max(1.0) as u64;
        self.seek_saturated = false;
        for t in 0..ticks {
            // Re-base on a counter glitch, so travel stays a measure of how far
            // the joint went rather than of what its counter did.
            if let (Some(prev), Some(now)) = (last_seen, self.position(joint)) {
                let step = i64::from(now) - i64::from(prev);
                if step.abs() > most_per_tick {
                    println!(
                        "    J{} reported a {:.1} deg step in one tick, which it cannot \
                         do: treating it as a counter glitch and measuring travel from \
                         here",
                        joint + 1,
                        self.deg_for_ticks(joint, step.abs())
                    );
                    began_at = began_at.map(|b| b.saturating_add(step as i32));
                }
            }
            last_seen = self.position(joint);
            let travelled = match (began_at, self.position(joint)) {
                (Some(from), Some(now)) => (i64::from(now) - i64::from(from)).abs(),
                _ => 0,
            };
            if travelled > range_ticks {
                println!(
                    "    J{} has travelled {:.1} deg without stalling, past its {:.1} deg \
                     range — it is not going to find a stop this way",
                    joint + 1,
                    self.deg_for_ticks(joint, travelled),
                    range_rad.to_degrees()
                );
                break;
            }
            // A joint that is no longer moving is either leaning on its stop
            // or too weak to leave where it started, and what tells those
            // apart is whether it went anywhere. Calling the second case a
            // stop is how the shoulder got referenced 0.1 deg into a 38 deg
            // approach; waiting out the timeout instead is the arm standing
            // still. Neither: decide now, on the travel.
            if !self.moving_within(joint, STALL_IDLE_TICKS, SEEK_STILL_S) {
                if travelled >= min_travel {
                    let latched = self.position(joint);
                    println!(
                        "    J{} stopped {:.2} deg in under {:.0} mA: that is its stop",
                        joint + 1,
                        self.deg_for_ticks(joint, travelled),
                        seek_ma
                    );
                    outcome = latched;
                } else {
                    println!(
                        "    J{} has not moved ({:.3} deg in {:.1} s) with {:.0} mA to \
                         spend — it is too weak to seek, not on a stop",
                        joint + 1,
                        self.deg_for_ticks(joint, travelled),
                        f64::from(t as u32) * self.dt,
                        seek_ma
                    );
                    weak = true;
                }
                break;
            }
            let cmd = JointCommand::velocity(speed, 0);
            self.tick_frame(Some((joint, cmd)))?;
            self.sleep_to(&mut next);
            let Some(pos) = self.position(joint) else {
                continue;
            };
            ring.push(pos);
            if ring.len() > STALL_WINDOW_TICKS as usize {
                ring.remove(0);
            }
            if let Some(c) = self.current_ma(joint) {
                peak_current = peak_current.max(f64::from(c.abs()));
            }
            if t < u64::from(SEEK_STARTUP_TICKS) || ring.len() < STALL_WINDOW_TICKS as usize {
                continue;
            }
            let moved = (i64::from(*ring.last().unwrap()) - i64::from(ring[0])).abs();
            // Measured speed over the window against what was commanded. The
            // stop is where the joint stops keeping up, and that reads the
            // same whether it halts dead or creeps under load.
            let window_s = f64::from(STALL_WINDOW_TICKS) * self.dt;
            let making = moved as f64 / window_s;
            best_making = best_making.max(making);
            // Given a second to get going and never past half the commanded
            // speed: this joint is not going to find anything. Whether more
            // gain or more current is the answer is decided by the caller
            // from the peak current, and either way waiting is not.
            if t > launch_deadline && best_making < SEEK_LAUNCH_FRACTION * h.speed_ticks_s {
                println!(
                    "    J{} never got above {:.0} of {:.0} ticks/s in {:.1} s with \
                     {:.0} mA to spend (peak draw {peak_current:.0} mA, {:.2} deg \
                     travelled) — it is not approaching a stop, it is losing to \
                     the load",
                    joint + 1,
                    best_making,
                    h.speed_ticks_s,
                    f64::from(t as u32) * self.dt,
                    seek_ma,
                    self.deg_for_ticks(joint, travelled)
                );
                weak = true;
                break;
            }
            if making < STALL_SPEED_FRACTION * h.speed_ticks_s {
                // Speed collapsing within the first half degree is a joint
                // saturating its current against the arm's own weight, which
                // reads identically to arriving at a stop. A stop is somewhere
                // the joint got to, so make it prove it got there.
                if travelled < min_travel || best_making < SEEK_LAUNCH_FRACTION * h.speed_ticks_s {
                    println!(
                        "    J{} is creeping at {making:.0} of {:.0} ticks/s only {:.3} deg \
                         from where it started (peak current {peak_current:.0} mA) — that \
                         is load, not an endstop",
                        joint + 1,
                        h.speed_ticks_s,
                        self.deg_for_ticks(joint, travelled)
                    );
                    weak = true;
                    break;
                }
                // Dwell on the stop, then reverse off it, exactly as the
                // sequence's backoff describes.
                self.coast(0.1)?;
                let latched = self.position(joint).unwrap_or(pos);
                let back = (-sign * h.speed_ticks_s) as i32;
                let backoff = (h.backoff_s / self.dt).round().max(1.0) as u64;
                for _ in 0..backoff {
                    let cmd = JointCommand::velocity(back, 0);
                    self.tick_frame(Some((joint, cmd)))?;
                    self.sleep_to(&mut next);
                }
                println!(
                    "    J{} stalled at {latched} ticks: making {making:.0} of \
                     {:.0} ticks/s, peak current {peak_current:.0} mA (threshold \
                     {threshold:.0})",
                    joint + 1,
                    h.speed_ticks_s
                );
                outcome = Some(latched);
                break;
            }
        }
        self.seek_saturated = weak && peak_current >= seek_ma * SATURATED_FRACTION;
        if weak {
            // Leave it off whatever it was pushing into, so the next attempt
            // starts where this one did instead of already leaning.
            let back = (-sign * h.speed_ticks_s) as i32;
            let backoff = (h.backoff_s / self.dt).round().max(1.0) as u64;
            self.reset_motion_watch();
            for _ in 0..backoff {
                let cmd = JointCommand::velocity(back, 0);
                self.tick_frame(Some((joint, cmd)))?;
                self.sleep_to(&mut next);
                // If it cannot reverse either, there is nothing to unload and
                // nothing to wait for.
                if !self.moving_within(joint, STALL_IDLE_TICKS, SEEK_STILL_S) {
                    break;
                }
            }
        }
        // The approach limit is the seek's, not the arm's: put the joint's
        // own current limit back whichever way the seek ended.
        self.insist_limits(node, vel_limit, normal_ilim)?;
        if outcome.is_none() && !weak {
            println!(
                "    J{} never plateaued (peak current {peak_current:.0} mA)",
                joint + 1
            );
        }
        Ok(outcome)
    }

    /// Seek one joint until two passes AGREE on where its endstop is.
    ///
    /// One stall is not a reference. A joint can stop against the arm's own
    /// body, a cable, or a tight spot in its gearbox, and the encoder
    /// reports that just as honestly as it reports the endstop — so a single
    /// pass cannot tell a stop from an obstruction, and the arm ends up
    /// referenced somewhere it has never been. Two passes can: approach,
    /// back off, approach again, and require the two to land within the
    /// config's `two_pass_max_diff_ticks`. A false stall does not repeat to
    /// that tolerance.
    fn seek_verified(&mut self, joint: usize) -> Result<Option<i32>, String> {
        let h = self.homing[joint].clone();
        let Some(first) = self.seek_once(joint)? else {
            return Ok(None);
        };
        // Every joint is verified, including the ones the config marks
        // single-pass. An unverified reference does not just mis-place its
        // own joint: the sequence moves the arm using it, so a joint homed
        // later seeks from somewhere the sequence did not intend and stalls
        // against something that is not its endstop. A wrist roll accepted
        // 98 deg out moved the joint homed after it by 45 deg.
        // A hall reference is a sensor edge, not a mechanical stop, so a
        // second pass adds nothing to verify against — the edge IS the
        // measurement. Only stall seeks get the agreement check.
        if h.strategy == HomingStrategy::Hall {
            return Ok(Some(first));
        }
        if !h.two_pass {
            println!(
                "    J{} is marked single-pass in the config; verifying anyway",
                joint + 1
            );
        }
        let Some(second) = self.seek_once(joint)? else {
            return Ok(None);
        };
        let diff = (i64::from(second) - i64::from(first)).abs();
        let allowed = i64::from(h.two_pass_max_diff_ticks)
            .min(self.ticks_for_deg(joint, TWO_PASS_MAX_DEG));
        if diff > allowed {
            println!(
                "    J{} passes disagree by {diff} ticks ({:.3} deg, allowed {:.3} deg): \
                 the first stop was not the endstop",
                joint + 1,
                self.deg_for_ticks(joint, diff),
                self.deg_for_ticks(joint, allowed)
            );
            return Ok(None);
        }
        println!(
            "    J{} passes agree within {diff} ticks ({:.4} deg)",
            joint + 1,
            self.deg_for_ticks(joint, diff)
        );
        // The slow pass is the better measurement of the two.
        Ok(Some(second))
    }

    /// Seek one joint until it finds its endstop, raising its gains when it
    /// does not. The reference is latched here.
    fn home_joint(&mut self, joint: usize) -> Result<(), String> {
        // A joint that is ALREADY on its stop cannot move in the seek
        // direction, which reads exactly like a joint too weak to move. It is
        // also the normal state: the handover parks the shoulder and elbow on
        // their own stops, so every run after the first starts there. The
        // seek's own backoff has already reversed off whatever it was leaning
        // on, so the first failure buys one plain retry before any value is
        // called into question — otherwise the shoulder escalates its seek
        // current by 25 % on every run for a condition that resolves itself.
        let mut retried_off_the_stop = false;
        loop {
            println!("  J{}: seeking its endstop", joint + 1);
            match self.seek_verified(joint)? {
                Some(latched) => {
                    let offset = self.home_offset[joint];
                    self.conv[joint].set_home(latched, offset);
                    self.referenced[joint] = true;
                    self.home_ticks[joint] = Some(latched);
                    self.results.home_rad[joint] = Some(offset);
                    println!(
                        "  J{}: referenced at {latched} ticks ({:.4} rad)",
                        joint + 1,
                        offset
                    );
                    return Ok(());
                }
                None => {
                    println!("  J{}: no verified endstop this pass", joint + 1);
                    if !retried_off_the_stop {
                        retried_off_the_stop = true;
                        println!(
                            "    J{} may have started on its stop; it has been backed \
                             off, so trying again as it is",
                            joint + 1
                        );
                        continue;
                    }
                    // Which knob depends on WHY it failed. A drive already
                    // against its approach limit is short of current, and a
                    // velocity gain cannot conjure torque the current loop is
                    // not allowed to produce — raising gain there only makes
                    // the loop shout louder at a limit it is already on.
                    let ilim = self.robot.joints[joint].ilim_ma;
                    let now_ma = self.seek_ma[joint];
                    if self.seek_saturated && now_ma < ilim {
                        let next = (now_ma * SEEK_CURRENT_STEP).min(ilim);
                        println!(
                            "    J{} was against its {now_ma:.0} mA approach limit: \
                             seek current -> {next:.0} mA (the joint allows {ilim:.0})",
                            joint + 1
                        );
                        self.seek_ma[joint] = next;
                        self.results.seek_ma[joint] = Some(next);
                        continue;
                    }
                    let current = self.scales[joint];
                    let next = current * GAIN_STEP;
                    if next > GAIN_CEILING {
                        return Err(format!(
                            "J{} found no verified endstop with velocity gains at \
                             x{current:.2} and {now_ma:.0} mA of seek current \
                             (the joint allows {ilim:.0} mA)",
                            joint + 1
                        ));
                    }
                    println!("    J{} seeks harder: velocity gains -> x{next:.2}", joint + 1);
                    self.push_scale(joint, next)?;
                }
            }
        }
    }

    /// Drive a joint to `position_rad` over `duration_s`, and keep working
    /// that joint until it arrives.
    fn move_to(&mut self, joint: usize, position_rad: f64, duration_s: f64) -> Result<(), String> {
        // The arm's own limits, not a seek's: a restore dropped earlier would
        // otherwise hold this move at a seek current for its whole span.
        let node = self.node(joint);
        let ilim = self.robot.joints[joint].ilim_ma as f32;
        let vel_limit = self.robot.joints[joint].velocity_limit_ticks_s as f32;
        self.insist_limits(node, vel_limit, ilim)?;
        loop {
            // Label AFTER the limit push, so a late tick in the move is not
            // attributed to the push that preceded it.
            self.cadence.phase("move");
            let target = self.conv[joint].motor_ticks(position_rad);
            let Some(start) = self.position(joint) else {
                return Err(format!("J{} reports no position", joint + 1));
            };
            let span = (duration_s / self.dt).round().max(1.0) as u32;
            let tol = self.arrival_tol(joint);
            let ring_amplitude = self.ticks_for_deg(joint, RING_AMPLITUDE_DEG);
            let settle_ticks = (SETTLE_BEFORE_JUDGING_S / self.dt).round().max(1.0) as u64;
            // The profile's own span, time for the drive to close the last
            // ticks, and time for the wind-up to finish before anything is
            // called a failure.
            let limit = u64::from(span) * 2 + settle_ticks;
            let mut streak = 0_u32;
            let mut next_tick = Instant::now();
            let mut arrived = false;
            let mut closest = i64::MAX;
            let mut history: Vec<i32> = Vec::with_capacity(RING_WINDOW_TICKS + 1);
            let mut saw_ringing = false;
            let mut too_weak = false;
            let mut peak_current = 0.0_f64;
            self.reset_motion_watch();
            for t in 1..=limit {
                let (pos, vel) = hermite(
                    f64::from(start),
                    f64::from(target),
                    t.min(u64::from(span)) as u32,
                    span,
                    self.dt,
                );
                let cmd = JointCommand::position(pos as i32, vel as i32, self.last_ff[joint]);
                self.tick_frame(Some((joint, cmd)))?;
                self.sleep_to(&mut next_tick);
                if let Some(c) = self.current_ma(joint) {
                    peak_current = peak_current.max(f64::from(c.abs()));
                }
                // Not moving, and not because it is there: the rung this
                // joint is on cannot carry it. That is the ring ladder's
                // other half — a scale chosen for being quiet is worthless if
                // the joint cannot move on it, and lowering gains until the
                // elbow went silent is exactly how it ended up unable to
                // leave its pose.
                if !self.moving_within(joint, STALL_IDLE_TICKS, SEEK_STILL_S) {
                    let far = self
                        .position(joint)
                        .is_some_and(|p| (i64::from(p) - i64::from(target)).abs() > tol);
                    if far && !self.handing_over && self.scales[joint] * GAIN_STEP <= GAIN_CEILING {
                        let next = self.scales[joint] * GAIN_STEP;
                        println!(
                            "    J{} cannot move at x{:.2}: velocity gains -> x{next:.2} \
                             (and x{:.2} is now this joint's floor)",
                            joint + 1,
                            self.scales[joint],
                            next
                        );
                        self.scale_floor[joint] = next;
                        self.push_scale(joint, next)?;
                        too_weak = true;
                        break;
                    }
                    // Stopped, and inside the arrival window: that IS arrival.
                    // Requiring a streak of in-window ticks on top of it asks
                    // a joint that has already stopped to keep proving it,
                    // and the elbow was failed for settling 0.04 deg from a
                    // target with a 0.044 deg window.
                    arrived = !far;
                    break;
                }
                if let Some(p) = self.position(joint) {
                    closest = closest.min((i64::from(p) - i64::from(target)).abs());
                    history.push(p);
                    if history.len() > RING_WINDOW_TICKS {
                        history.remove(0);
                    }
                }
                // Ringing is caught WHILE the move runs, not after it gives
                // up. Waiting for the move to fail means the joint shakes
                // for its whole span and the only reading left is where it
                // happened to stop, which says nothing; worse, a retry that
                // reads that as "not arrived" raises the gains and makes the
                // shaking louder. Lower it the moment the reversals appear
                // and let the same move carry on from where it is.
                // Oscillating while the move runs: stop the move and settle
                // the question by measurement rather than shaking the joint
                // for the rest of its span.
                if t >= u64::from(span) + settle_ticks
                    && ringing_now(&history, ring_amplitude)
                {
                    // Before gravity is measured, stopping the move here is
                    // what created the deadlock: the joint ends up short of
                    // its target, that reads as "did not settle", and the
                    // ladder is asked to quiet a joint whose feedforward is
                    // still a guess. Note it and let the move finish.
                    if !self.vibration_fatal {
                        if !self.rang_while_homing[joint] {
                            println!(
                                "    J{} is shaking on the config's gravity model; \
                                 noted for step 3",
                                joint + 1
                            );
                        }
                        self.rang_while_homing[joint] = true;
                        // Deliberately NOT `continue`: the arrival check lives
                        // below, and skipping it meant a joint that was
                        // dithering 0.02 deg inside a 0.044 deg window could
                        // never finish its move at all.
                    } else {
                        println!("    J{} is still oscillating after settling", joint + 1);
                        saw_ringing = true;
                        break;
                    }
                }
                // Past the profile, a joint that keeps sliding one way is
                // not tracking: it is being carried by gravity because its
                // loop was left too slack — the over-correction a ringing
                // joint invites. Put the authority back.
                if t >= u64::from(span) + settle_ticks
                    && !self.handing_over
                    && self.gravity_can_load(joint)
                    && drifting_now(&history, ring_amplitude)
                    && self.scales[joint] * GAIN_STEP <= GAIN_CEILING
                {
                    let next = self.scales[joint] * GAIN_STEP;
                    println!(
                        "    J{} is drooping mid-move: velocity gains -> x{next:.2}",
                        joint + 1
                    );
                    self.push_scale(joint, next)?;
                    history.clear();
                    streak = 0;
                    next_tick = Instant::now();
                    continue;
                }
                let inside = self
                    .position(joint)
                    .is_some_and(|p| (i64::from(p) - i64::from(target)).abs() <= tol);
                streak = if inside { streak + 1 } else { 0 };
                if t >= u64::from(span) && streak >= IN_POS_STREAK {
                    arrived = true;
                    break;
                }
            }
            // The gains changed under it, so the profile it was following no
            // longer describes where it is. Start the move again from here.
            if too_weak {
                continue;
            }
            if arrived && !saw_ringing && closest <= tol {
                // Arrived clean and never shook: there is nothing for a hold
                // proof to discover, and paying one on every move is seconds
                // per joint spent confirming what the arrival streak already
                // showed.
                println!(
                    "  J{}: at {:.4} rad (peak {peak_current:.0} of {ilim:.0} mA)",
                    joint + 1,
                    position_rad
                );
                self.results.gain_scale[joint] = self.scales[joint];
                return Ok(());
            }
            if arrived {
                println!(
                    "  J{}: at {:.4} rad after shaking on the way in (peak \
                     {peak_current:.0} of {ilim:.0} mA)",
                    joint + 1,
                    position_rad
                );
                self.results.gain_scale[joint] = self.scales[joint];
                // Before gravity is known, a ring is not evidence about the
                // gains. A joint carrying a load its feedforward under-states
                // has to generate the difference out of tracking error, so it
                // CANNOT sit on its target — and lowering its gains to quiet
                // that leaves it unable to move, which is the deadlock the
                // elbow kept landing in. Homing only needs it to arrive. The
                // ladder runs in step 3, where the feedforward is measured and
                // a ring means what it is supposed to mean.
                if !self.vibration_fatal {
                    self.rang_while_homing[joint] = true;
                    return Ok(());
                }
                self.retune_toward_arrival(joint, true, false)?;
                continue;
            }
            let off = self
                .position(joint)
                .map(|p| i64::from(p) - i64::from(target))
                .unwrap_or_default();
            // Ending inside the window IS arrival. The streak exists to stop a
            // joint being called arrived while it is still flying through the
            // target, which the `t >= span` guard already covers; demanding ten
            // CONSECUTIVE in-window ticks on top of it failed the elbow while
            // it sat one tick from its target, because a joint that dithers
            // never strings ten together.
            if !arrived && off.abs() <= tol {
                println!(
                    "  J{}: at {:.4} rad, {:.4} deg off and dithering (peak \
                     {peak_current:.0} of {ilim:.0} mA)",
                    joint + 1,
                    position_rad,
                    self.deg_for_ticks(joint, off.abs())
                );
                self.results.gain_scale[joint] = self.scales[joint];
                return Ok(());
            }
            // Ring and droop are opposite failures and they are told apart by
            // the SHAPE of the tail, not by how close the move once came.
            // "It reached the target at some point, so it must be ringing"
            // called a joint sitting 0.117 deg below its target under 2 A a
            // ringing joint, and then lowered the gains of something that was
            // plainly under-driven.
            if self.handing_over {
                println!(
                    "  J{}: parked {:.4} deg from its stop; it rests there",
                    joint + 1,
                    self.deg_for_ticks(joint, off.abs())
                );
                return Ok(());
            }
            let ringing = saw_ringing || ringing_now(&history, ring_amplitude);
            println!(
                "  J{}: did not settle at {:.4} rad ({off} ticks off, closest {closest}, \
                 peak {peak_current:.0} of {ilim:.0} mA{})",
                joint + 1,
                position_rad,
                if peak_current >= f64::from(ilim) * 0.9 {
                    " — AT ITS CURRENT LIMIT"
                } else {
                    ""
                }
            );
            let saturated = peak_current >= f64::from(ilim) * SATURATED_FRACTION;
            self.retune_toward_arrival(joint, ringing, saturated)?;
        }
    }
}

/// The gravity model for the arm AS FITTED: the URDF chain plus the active
/// gripper's inertials.
///
/// Loading the arm alone leaves the tool's mass out of G(q) entirely, and the
/// tool IS the load at the wrist: with it missing, the model asked for 0.018 Nm
/// at the wrist pitch while the drive was pulling 0.52 Nm to hold the pose, and
/// no per-joint scale can make up a body the model does not have. The vendor's
/// rule is that the tool REPLACES the sixth link, so even a bare flange plate
/// is a config entry rather than part of the URDF chain.
fn gravity_model(
    assets_dir: &std::path::Path,
    gripper: Option<&GripperConfig>,
) -> Result<par6_kin::Kin, String> {
    let tool = gripper.map(|g| {
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
    par6_kin::Kin::load_arm(assets_dir, tool.as_ref()).map_err(|e| format!("gravity model: {e}"))
}

/// The rate `joint` is FALLING at \[rad/s\], from its own angle history and
/// the model's torque for it. Positive is a sag, whichever way gravity pulls.
///
/// The sign convention of the whole gravity search, in one place so that one
/// test can pin it. G(q) is the torque the motor must apply to HOLD, so
/// gravity accelerates the joint the other way: falling is motion in the
/// direction `-sign(G)`. Getting this backwards is not a compile error — it
/// labelled a lifting wrist "sagging", stepped the scale the wrong way, and
/// made the slope test conclude the feedforward was not helping.
fn sag_rate(samples: &[(f64, f64)], torque_nm: f64) -> Option<f64> {
    let along_gravity: Vec<(f64, f64)> = samples
        .iter()
        .map(|(t, angle)| (*t, angle * -torque_nm.signum()))
        .collect();
    slope_per_s(&along_gravity)
}

/// Least-squares slope of `(t, y)` \[y per second\], or None with too few
/// samples to fit one.
fn slope_per_s(samples: &[(f64, f64)]) -> Option<f64> {
    if samples.len() < 8 {
        return None;
    }
    let n = samples.len() as f64;
    let mean_t = samples.iter().map(|(t, _)| t).sum::<f64>() / n;
    let mean_y = samples.iter().map(|(_, y)| y).sum::<f64>() / n;
    let mut num = 0.0;
    let mut den = 0.0;
    for (t, y) in samples {
        num += (t - mean_t) * (y - mean_y);
        den += (t - mean_t) * (t - mean_t);
    }
    if den <= 0.0 {
        return None;
    }
    Some(num / den)
}

/// The reversals in this window, and the typical swing between them \[ticks\].
///
/// Amplitude measured BETWEEN reversals, which is what makes this independent
/// of whatever the joint was commanded to do. Measuring the window's raw span
/// counted a commanded 1.7 deg move as ring amplitude; measuring it about a
/// straight-line trend still counted 1.25 deg, because the profile is a quintic
/// and a line cannot describe its curvature. The distance a joint travels
/// between turning points, though, is the ring itself — a profile has no
/// turning points at all, however far or however unevenly it travels.
fn ring_swings(history: &[i32]) -> (usize, f64) {
    let mut turns: Vec<i64> = Vec::new();
    let s = swings_into(history, &mut turns);
    (s.reversals, s.median)
}

/// What a run of samples does between its turning points.
#[derive(Clone, Copy, Debug, Default)]
struct Swings {
    reversals: usize,
    /// The typical swing, which is what a sustained oscillation shows.
    median: f64,
    /// The largest swing, which is what a BURST shows — a tenth of a second
    /// of buzz inside a two-second window leaves the median at the quiet
    /// value, and that is a shake somebody in the room can hear.
    worst: f64,
}

/// The swings in `samples`, using `turns` as scratch so nothing allocates.
///
/// Generic over the sample type because the same question is asked of encoder
/// position and of drive current: chatter that barely moves the encoder still
/// reverses the current hard, and that is the channel a 250 Hz position
/// sampler cannot see.
fn swings_into<T: Copy + Into<i64>>(samples: &[T], turns: &mut Vec<i64>) -> Swings {
    turns.clear();
    let mut last_dir = 0_i64;
    let mut last_turn = samples.first().map_or(0, |p| (*p).into());
    let mut last_turn_at = 0_usize;
    for (k, pair) in samples.windows(2).enumerate() {
        let d: i64 = pair[1].into() - pair[0].into();
        if d == 0 {
            continue;
        }
        let dir = d.signum();
        if last_dir != 0 && dir != last_dir {
            let at: i64 = pair[0].into();
            // Only a FAST reversal is a shake. A seek that stalls and backs
            // off, or a move that comes back, reverses too — seconds apart —
            // and the distance it covers between those turns is the travel
            // itself. Unbounded, the largest swing on the base came out at 60
            // deg, which is the base doing its job.
            if k - last_turn_at <= RING_HALF_PERIOD_TICKS {
                turns.push((at - last_turn).abs());
            }
            last_turn = at;
            last_turn_at = k;
        }
        last_dir = dir;
    }
    if turns.is_empty() {
        return Swings::default();
    }
    let reversals = turns.len();
    turns.sort_unstable();
    Swings {
        reversals,
        // The median swing, so one large excursion on the way into a pose
        // cannot stand in for a sustained oscillation.
        median: turns[reversals / 2] as f64,
        worst: turns[reversals - 1] as f64,
    }
}

/// How far a window of samples spans end to end, which separates a joint that
/// is travelling from one that is sitting still and shaking.
fn span_ticks(history: &[i32]) -> i64 {
    let mut lo = i64::MAX;
    let mut hi = i64::MIN;
    for p in history {
        let v = i64::from(*p);
        lo = lo.min(v);
        hi = hi.max(v);
    }
    if lo > hi {
        0
    } else {
        hi - lo
    }
}

/// Whether these samples are a joint shaking rather than moving.
///
/// Both halves are needed. Reversals alone are satisfied by encoder noise on a
/// joint standing still; amplitude alone is satisfied by any move.
fn ringing_now(history: &[i32], amplitude_ticks: i64) -> bool {
    let mut turns: Vec<i64> = Vec::new();
    ringing_now_into(history, amplitude_ticks, &mut turns)
}

/// The same question asked from the tick path, where allocating is not
/// allowed: this runs on every joint on every tick once the window is full.
fn ringing_now_into(history: &[i32], amplitude_ticks: i64, turns: &mut Vec<i64>) -> bool {
    if history.len() < RING_WINDOW_TICKS {
        return false;
    }
    let s = swings_into(history, turns);
    s.reversals >= RING_REVERSALS && s.median >= amplitude_ticks as f64
}

/// Whether these samples are a joint sliding one way rather than shaking.
///
/// Drooping under gravity and ringing are opposite failures with the same
/// symptom — the joint is not where it was told to be — and they want
/// opposite corrections, so they have to be told apart. A droop is
/// monotonic: nearly every step goes the same way.
fn drifting_now(history: &[i32], amplitude_ticks: i64) -> bool {
    if history.len() < RING_WINDOW_TICKS {
        return false;
    }
    let span = i64::from(*history.last().unwrap()) - i64::from(history[0]);
    if span.abs() < amplitude_ticks {
        return false;
    }
    let with = history
        .windows(2)
        .filter(|p| {
            let d = i64::from(p[1]) - i64::from(p[0]);
            d != 0 && d.signum() == span.signum()
        })
        .count();
    let moved = history
        .windows(2)
        .filter(|p| i64::from(p[1]) != i64::from(p[0]))
        .count();
    moved > 0 && with * 100 / moved >= DRIFT_AGREEMENT_PCT
}

/// Cubic Hermite from `start` to `target` over `span` ticks: position and
/// the tangent as a velocity feedforward, both in motor ticks.
fn hermite(start: f64, target: f64, elapsed: u32, span: u32, dt: f64) -> (f64, f64) {
    let u = f64::from(elapsed) / f64::from(span.max(1));
    let u = u.clamp(0.0, 1.0);
    // Quintic, not cubic. A cubic's acceleration jumps at both ends, which is
    // unbounded jerk and is what the arm is felt to do; the quintic leaves
    // acceleration at zero where the move starts and stops.
    let h = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
    let dh = 30.0 * u * u * (1.0 - 2.0 * u + u * u);
    let d = target - start;
    (start + h * d, dh * d / (f64::from(span.max(1)) * dt))
}

const USAGE: &str = "\
par6-selfcal [CONFIG] [--sim] [--apply] [--home-only]

Homes one PAR6 arm, measures what this arm needs, and proves the result. Talks
to the CAN bus directly, and refuses to start while par6d is running.

  CONFIG       robot config to read (default config/PAR6.toml)
  --sim        run against the simulated bus, with no arm attached
  --apply      write the measurements into CONFIG, keeping a .before-selfcal
               backup. Only ever written when the run verified.
  --home-only  stop after step 1 (homing), for repeatability runs
  --repeat N   home N more times afterwards and report how far each joint's
               reference moves between runs

Step 1 homes on the configured gains, raising a joint's seek current or its
velocity gains when it will not reach its endstop. Step 2 measures each loaded
joint's gravity feedforward scale on a torque-only hold, distal first. Step 3
returns every joint to its most loaded pose and checks it holds on what was
measured; a joint that misses is measured again and re-verified.

Every exit writes selfcal-measurements.toml, including a failed run: a run that
measured four joints and failed on the fifth has still measured four joints.

Needs CAP_SYS_NICE (or root) for SCHED_FIFO; without it the command cadence
slips and the drives feel that as a disturbance. Takes about two minutes.
";

/// The PID of a running `par6d`, if there is one.
///
/// Two processes commanding the same drives is not a race this tool can win or
/// should try to: the runtime holds position while this one is seeking
/// endstops, and the drives obey whichever frame arrived last. Checked by name
/// in /proc rather than by port, because the daemon's command socket may be on
/// an ephemeral one.
fn running_daemon() -> Option<u32> {
    let mine = std::process::id();
    let entries = std::fs::read_dir("/proc").ok()?;
    for entry in entries.flatten() {
        let pid: u32 = match entry.file_name().to_str().and_then(|n| n.parse().ok()) {
            Some(pid) if pid != mine => pid,
            _ => continue,
        };
        let comm = std::fs::read_to_string(entry.path().join("comm")).unwrap_or_default();
        if comm.trim() == "par6d" {
            return Some(pid);
        }
    }
    None
}

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).init();
    if std::env::args().any(|a| a == "--help" || a == "-h") {
        print!("{USAGE}");
        return;
    }
    if !std::env::args().any(|a| a == "--sim") {
        if let Some(pid) = running_daemon() {
            eprintln!(
                "par6d is running as pid {pid}. This tool drives the bus itself, and \
                 two processes commanding the same drives means the arm obeys whichever \
                 frame arrived last. Stop par6d and run this again."
            );
            std::process::exit(2);
        }
    }
    let path = std::env::args()
        .nth(1)
        .filter(|a| !a.starts_with('-'))
        .unwrap_or_else(|| "config/PAR6.toml".to_string());
    let bundle = match ConfigBundle::load(std::path::Path::new(&path)) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("config {path}: {e}");
            std::process::exit(2);
        }
    };
    // The assets tree sits beside the config, as it does for the runtime.
    let assets_dir = std::path::Path::new(&path)
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("assets/par6_description"))
        .unwrap_or_else(|| std::path::PathBuf::from("assets/par6_description"));
    let repeats = std::env::args()
        .skip_while(|a| a != "--repeat")
        .nth(1)
        .and_then(|n| n.parse::<usize>().ok());
    let simulated = std::env::args().any(|a| a == "--sim");
    if simulated {
        println!("--sim: the simulated bus, to prove the script with no arm attached");
    }
    let mut arm = match Arm::open(&bundle, &assets_dir, simulated) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}");
            eprintln!("is par6d still running? this tool needs the bus to itself");
            std::process::exit(2);
        }
    };
    let priority = arm.robot.timing.as_ref().map_or(98, |t| t.fifo_priority);
    if request_realtime(priority) {
        println!("real-time: SCHED_FIFO {priority}, memory locked");
    } else {
        println!(
            "real-time: SCHED_FIFO {priority} REFUSED — the command cadence will slip, \
             which the drives feel as a disturbance. Run with CAP_SYS_NICE or raise \
             RLIMIT_RTPRIO."
        );
    }

    // Whatever happens inside, the arm is handed back on its configured
    // gains and holding: the failure path is where a joint is most likely to
    // be left slack, and that is the path that dropped the elbow.
    // Step 2 only runs if step 1 homed: every pose it sweeps is an offset
    // from the pose homing ended in, and without a reference those are
    // arbitrary encoder counts.
    let outcome = step1(&mut arm).and_then(|()| {
        // Step 1 is the tuning, so it is the last place a joint is allowed to
        // shake. From here the rule is enforced on every tick.
        arm.vibration_fatal = true;
        arm.ready_pose = std::array::from_fn(|i| {
            if i < arm.n() {
                arm.position(i)
                    .map(|p| arm.conv[i].joint_rad(p))
                    .unwrap_or(0.0)
            } else {
                0.0
            }
        });
        if std::env::args().any(|a| a == "--home-only") {
            println!("\n--home-only: stopping after step 1");
            return Ok(());
        }
        step2(&mut arm)
    });
    let outcome = outcome.and_then(|()| match repeats {
        Some(n) => repeatability(&mut arm, n),
        None => Ok(()),
    });
    let scales = arm.scales.clone();
    match &outcome {
        Ok(()) => {
            // One table, because the answer to "what did it find" should not
            // have to be assembled out of a hundred lines of progress.
            println!("\nwhat this arm needed, per joint:");
            println!(
                "  {:<4}{:>10}{:>10}{:>10}{:>10}{:>10}",
                "", "home", "gain", "seek mA", "gravity", "dither"
            );
            for j in 0..arm.n() {
                let gain = scales.get(j).copied().unwrap_or(1.0);
                let seek = arm.results.seek_ma[j];
                let measured = arm.results.gravity_scale.get(j).copied().flatten();
                let grav = measured.unwrap_or_else(|| arm.grav_scale[j]);
                let dither = arm.deg_for_ticks(j, arm.ring_floor[j] as i64);
                println!(
                    "  J{:<3}{:>10}{:>10}{:>10}{:>10}{:>10}",
                    j + 1,
                    arm.home_ticks[j]
                        .map(|p| p.to_string())
                        .unwrap_or_else(|| "-".into()),
                    if gain == 1.0 {
                        "vendor".to_string()
                    } else {
                        format!("x{gain:.2}")
                    },
                    seek
                        .map(|ma| format!("{ma:.0}"))
                        .unwrap_or_else(|| "vendor".into()),
                    if grav == 1.0 {
                        "vendor".to_string()
                    } else if measured.is_some() {
                        format!("x{grav:.3}")
                    } else {
                        // Carried from the config, not measured this run.
                        format!("x{grav:.3} cfg")
                    },
                    if dither > 0.0 {
                        format!("{dither:.3} deg")
                    } else {
                        "-".into()
                    },
                );
            }
            if scales.iter().all(|s| *s == 1.0) {
                println!("  the configured gains homed the arm as they are");
            }
        }
        Err(e) => eprintln!("\n{e}"),
    }
    // Printed whatever the outcome: this is evidence about what the arm
    // sounded like, and a failed run is exactly when it is wanted.
    println!("\nwhat it sounded like, worst swing between reversals:");
    println!(
        "  {:<4}{:>12}{:>12}{:>12}  loudest during",
        "", "moving", "holding", "current"
    );
    for j in 0..arm.n() {
        let n = &arm.noise[j];
        if n.moving == 0.0 && n.holding == 0.0 && n.ripple == 0.0 {
            continue;
        }
        let loudest = if n.moving >= n.holding {
            &n.moving_phase
        } else {
            &n.holding_phase
        };
        println!(
            "  J{:<3}{:>12}{:>12}{:>12}  {}",
            j + 1,
            format!("{:.3} deg", arm.deg_for_ticks(j, n.moving as i64)),
            format!("{:.3} deg", arm.deg_for_ticks(j, n.holding as i64)),
            format!("{:.0} mA", n.ripple),
            if loudest.is_empty() { "-" } else { loudest.as_str() },
        );
    }
    println!(
        "  reported, not judged: the bar is {:.3} deg and was set from one \
         audible datum",
        RING_AMPLITUDE_DEG
    );
    if let Some((mean, p99, worst, n)) = arm.cadence.summary() {
        println!(
            "\ncommand cadence over {n} ticks: target {:.0} us, mean {mean:.0}, p99 under {p99}, \
             worst {worst} at tick {} during {}",
            arm.dt * 1e6,
            arm.cadence.worst_at,
            if arm.cadence.worst_phase.is_empty() {
                "an unnamed phase"
            } else {
                &arm.cadence.worst_phase
            }
        );
    }
    let patch = std::path::Path::new("selfcal-measurements.toml");
    arm.results.write(patch, &arm.robot);
    let asked_to_apply = std::env::args().any(|a| a == "--apply");
    if asked_to_apply && outcome.is_err() {
        println!(
            "  NOT applied: the run did not verify. A value the search was still \
             holding when it failed is not a measurement, and writing one into the \
             config is how J3 ended up at x2.44 of its vendor gain."
        );
    } else if asked_to_apply {
        if let Err(e) = arm.results.apply(std::path::Path::new(&path), &arm.robot) {
            eprintln!("  could not apply the measurements: {e}");
        }
    } else {
        println!("  --apply writes these into the config (a backup is kept)");
    }
    println!("\nhanding the arm back:");
    arm.restore(HANDOVER_HOLD_S);
    if outcome.is_err() {
        std::process::exit(1);
    }
}

/// Step 2: check the vendor's gravity model against this arm, one joint at a
/// time, and correct it where it is wrong.
///
/// The vendor's masses get close; what differs between two of these arms is
/// small. So this does not identify anything from scratch. For each joint, in
/// the order the load stacks — wrist pitch, wrist, elbow, shoulder — the arm
/// is placed where that joint carries the most it ever will, compensation is
/// switched on for it, and the encoder says whether the joint holds, sags or
/// lifts. The scale that holds it is that joint's answer.
///
/// Sag and lift are both failures and they are distinguishable by sign, so a
/// scale that overshoots cannot pass as success the way "did not fall" would.
fn step2(arm: &mut Arm) -> Result<(), String> {
    // A ring HERE is still a failure, but the joints that shook during homing
    // are named rather than failed on: their feedforward is about to be
    // measured, and that is the thing most likely to have caused it.
    let noted: Vec<String> = (0..arm.n())
        .filter(|j| arm.rang_while_homing[*j])
        .map(|j| format!("J{}", j + 1))
        .collect();
    if !noted.is_empty() {
        println!(
            "  {} shook while homing on the config's gravity model; step 3 judges \
             them again once it is measured",
            noted.join(", ")
        );
    }
    for joint in [4usize, 3, 2, 1] {
        arm.awaiting_gravity[joint] = true;
    }
    let ringing: Vec<(usize, f64)> = arm
        .oscillating()
        .into_iter()
        .filter(|(j, _)| !arm.awaiting_gravity[*j])
        .collect();
    if !ringing.is_empty() {
        return Err(format!(
            "stage 1 left {} oscillating; stage 1 has to fix that before gravity \
             can be measured",
            ringing
                .iter()
                .map(|(j, d)| format!("J{} at {d:.4} deg", j + 1))
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }
    println!("\nstep 2: gravity compensation, joint by joint");
    // Re-push what step 1 measured, and say it out loud. The gains are
    // volatile and nothing since has touched them, but "should still be
    // there" is not the same as knowing, and step 2 holding a joint on
    // untuned gains is what made step 1 pointless once already.
    let tuned: Vec<(usize, f64)> = (0..arm.n())
        .filter(|j| arm.scales[*j] != 1.0)
        .map(|j| (j, arm.scales[j]))
        .collect();
    if tuned.is_empty() {
        println!("  step 1 needed no gain changes; the configured gains are in force");
    } else {
        for (j, s) in tuned {
            println!("  J{}: holding on step 1's velocity gains x{s:.2}", j + 1);
            arm.push_scale(j, s)?;
        }
    }
    // Distal first: every joint carries what is beyond it, so the wrist has to
    // be right before the elbow's reading means anything.
    let order: [usize; 4] = [4, 3, 2, 1];
    // Start from what the config carries, which after an --apply is the last
    // run's answer. The pass/fail test is unchanged and absolute — a joint
    // still has to HOLD — so starting nearer cannot make a wrong scale pass;
    // it only spends the attempts refining instead of rediscovering.
    let mut scales = [1.0_f64; par6_kin::NQ];
    for (j, s) in scales.iter_mut().enumerate().take(arm.n()) {
        *s = arm.grav_scale[j];
    }
    let carried: Vec<String> = (0..arm.n())
        .filter(|j| scales[*j] != 1.0)
        .map(|j| format!("J{} x{:.4}", j + 1, scales[j]))
        .collect();
    if carried.is_empty() {
        println!("  starting from the vendor's model on every joint");
    } else {
        println!(
            "  starting from the scales already in the config ({}); this run \
             refines them",
            carried.join(", ")
        );
    }
    for joint in order {
        let pose = arm.loaded_pose(joint)?;
        println!(
            "  J{}: to its most loaded pose {:?}",
            joint + 1,
            pose.iter().map(|v| (v * 100.0).round() / 100.0).collect::<Vec<_>>()
        );
        for (j, target) in pose.iter().enumerate().take(arm.n()) {
            let s = arm.travel_time(j, *target);
            arm.move_to(j, *target, s)?;
        }
        let from = scales[joint];
        let held = search_gravity(arm, joint, &pose, &scales, from)?;
        if from != 1.0 {
            println!(
                "  J{}: the config's x{from:.4} refines to x{held:.4} ({:+.2}%)",
                joint + 1,
                (held / from - 1.0) * 100.0
            );
        }
        scales[joint] = held;
        // Every later approach move holds this joint on the scale just
        // measured, not on the config's guess — and from here it is held to
        // the no-oscillation rule like everything else.
        arm.grav_scale[joint] = held;
        arm.awaiting_gravity[joint] = false;
        arm.results.gravity_scale[joint] = Some(held);
    }

    println!("\n# measured gravity scale per joint");
    for (j, scale) in scales.iter().enumerate().take(arm.n()).skip(1) {
        println!("# J{}  gravity_scale = {scale:.4}", j + 1);
    }
    step3(arm, &scales)
}

impl Arm {
    /// The pose that puts `joint` under the most gravity load it will see,
    /// with everything outboard of it in a known configuration.
    ///
    /// Built from the pose homing ended in, so it is reachable: only the
    /// joints that change the lever arm are moved, and the wrist is taken to
    /// level or vertical rather than to an angle chosen for no reason.
    fn loaded_pose(&self, joint: usize) -> Result<[f64; par6_kin::NQ], String> {
        let mut q = self.ready_pose;
        match joint {
            // Wrist pitch level: its own load is greatest across the axis.
            4 => q[4] = 0.0,
            // Wrist pitch pointing down, wrist roll across: J4 carries the
            // hand at its longest lever.
            3 => {
                q[4] = -std::f64::consts::FRAC_PI_2;
                q[3] = std::f64::consts::FRAC_PI_2;
            }
            // Forearm out, wrist level: the elbow carries everything beyond.
            2 => {
                q[3] = 0.0;
                q[4] = 0.0;
            }
            // The shoulder carries everything regardless of how the elbow is
            // folded, so the elbow stays where homing left it — holding at its
            // own limit to extend the arm further only tests the elbow, and it
            // failed doing exactly that.
            1 => {
                q[3] = 0.0;
                q[4] = 0.0;
            }
            _ => return Err(format!("J{} is not gravity-loaded", joint + 1)),
        }
        Ok(q)
    }

    /// Hold `pose` with the drive's impedance frame plus `scale` times the
    /// model's gravity torque, and return how far `joint` drifted \[rad\].
    ///
    /// Signed in the direction gravity pulls, so a negative number is a sag
    /// and a positive one is a lift.
    fn hold_on_gravity(
        &mut self,
        joint: usize,
        pose: &[f64; par6_kin::NQ],
        scale: f64,
        settled: &[f64; par6_kin::NQ],
    ) -> Result<f64, String> {
        self.cadence.phase("gravity watch");
        let ticks = (GRAVITY_WATCH_S / self.dt).round().max(1.0) as u64;
        // Evaluate the model once BEFORE the timed loop. The first call into
        // the kinematics does its lazy setup, and paying that inside the loop
        // is a tick three times late — which the drives feel as a disturbance
        // and the cadence report shows as a single bad outlier.
        let mut warm = [0.0_f64; par6_kin::NQ];
        self.kin.gravity(pose, &mut warm)
            .map_err(|e| format!("gravity warm-up: {e}"))?;
        let targets: Vec<i32> = (0..self.n())
            .map(|j| self.conv[j].motor_ticks(pose[j]))
            .collect();
        // Let the approach finish before the reference is taken. Measuring
        // from the instant the move ends folds the tail of that move into the
        // reading, and that tail does not care what the gravity scale is — it
        // shows up as the same drift at every scale, which is exactly how it
        // was found.
        let mut next_tick = Instant::now();
        self.reset_motion_watch();
        self.measuring = true;
        let settle_cap = (SETTLE_BEFORE_JUDGING_S / self.dt).round().max(1.0) as u64;
        for _ in 0..settle_cap {
            self.tick_each(|_, j| JointCommand::position(targets[j], 0, 0))?;
            self.sleep_to(&mut next_tick);
            if !self.still_moving(joint, SETTLED_TICKS) {
                break;
            }
        }
        let start = self.position(joint).ok_or("no position")?;
        // Settled, still on position: this is the joint working, which is what
        // the oscillation rule has to be measured against.
        let (reversals, swing) = ring_swings(&self.history[joint]);
        if reversals >= RING_REVERSALS {
            self.hold_swing[joint] = swing;
        }
        self.reset_motion_watch();
        self.torque_only = Some(joint);
        let mut g = [0.0_f64; par6_kin::NQ];
        let mut drift = 0.0_f64;
        // (elapsed, joint angle) through the watch. The verdict is the SLOPE
        // of these, not the endpoint difference: a least-squares rate over a
        // hundred samples resolves a sag far finer than the encoder resolves
        // a single displacement, which is what lets the watch be short enough
        // that the arm is never standing still for a second.
        let mut samples: Vec<(f64, f64)> = Vec::with_capacity(ticks as usize + 1);
        let watch_began = Instant::now();
        for _ in 0..ticks {
            // G(q) at where the arm IS, not where it was asked to be: a
            // sagging joint's load grows as it falls.
            let mut q = *pose;
            for (j, angle) in q.iter_mut().enumerate().take(self.n()) {
                if let Some(p) = self.position(j) {
                    *angle = self.conv[j].joint_rad(p);
                }
            }
            self.kin
                .gravity(&q, &mut g)
                .map_err(|e| format!("gravity: {e}"))?;
            self.tick_each(|arm, j| {
                // The joint under test gets the scale being tried; the ones
                // already settled keep theirs; the rest are held on position
                // so they cannot contribute motion of their own.
                let s = if j == joint { scale } else { settled[j] };
                let ff = if j < par6_kin::NQ {
                    arm.torque_to_ma(j, s * g[j])
                } else {
                    0
                };
                if j == joint {
                    // TORQUE ONLY for the joint being measured — the runtime's
                    // own gravity hold (`law_idle`), and the only frame in
                    // which this question has an answer. Held on the position
                    // pack instead, the drive's integrating loop supplies
                    // whatever the feedforward does not, so the joint holds at
                    // any scale and the feedforward on top of it merely
                    // over-drives: the wrist read -0.99 deg/s of LIFT at the
                    // model's own value while that value matched the current
                    // the drive was really pulling. Nothing about gravity was
                    // being measured.
                    JointCommand::current(ff)
                } else {
                    // Everything else stays clamped on position so it cannot
                    // contribute motion of its own.
                    JointCommand::position(targets[j], 0, ff)
                }
            })?;
            self.sleep_to(&mut next_tick);
            if let Some(p) = self.position(joint) {
                let here = self.conv[joint].joint_rad(p);
                let moved = here - self.conv[joint].joint_rad(start);
                // Signed so that falling is negative whichever way gravity
                // pulls this joint at this pose.
                drift = moved * -g[joint].signum();
                let at = watch_began.elapsed().as_secs_f64();
                if at >= GRAVITY_SETTLE_IN_S {
                    samples.push((at, here));
                }
            }
            // Verdict reached: it has clearly moved off, or it has settled and
            // stopped. Either way the rest of the watch is a still arm
            // teaching nobody anything.
            // The joint is on torque only here, so a wrong scale means it is
            // falling. This both ends a decided measurement early and bounds
            // how far a bad scale gets to take it.
            if drift.abs() > GRAVITY_ABORT_RAD && samples.len() >= 8 {
                break;
            }
            if !self.still_moving(joint, STALL_IDLE_TICKS) {
                break;
            }
        }
        self.measuring = false;
        self.torque_only = None;
        // Said out loud while the arm is still standing in the pose it was
        // measured in, so what the room hears and what the encoder saw can be
        // compared at the moment it happens.
        let heard = self.noise_line();
        if !heard.is_empty() {
            println!("    loudest through that hold: {heard}");
        }
        // What this joint did while it was let go is a measurement, not
        // evidence about its gains — and the window outlives the release, so
        // leaving it in place would have the rule judge the next move on it.
        self.history[joint].clear();
        self.error_history[joint].clear();
        self.current_history[joint].clear();
        Ok(sag_rate(&samples, g[joint]).unwrap_or(drift / GRAVITY_WATCH_S))
    }

    /// Joint torque to the drive's current feedforward \[mA\].
    fn torque_to_ma(&self, joint: usize, tau_nm: f64) -> i16 {
        let j = &self.robot.joints[joint];
        let sign = if j.dir == 1 { -1.0 } else { 1.0 };
        let motor_nm = tau_nm / (j.gear_ratio * j.gear_efficiency);
        let ma = sign * motor_nm / j.kt_nm_a * 1000.0;
        ma.clamp(-j.ilim_ma, j.ilim_ma) as i16
    }
}

/// Find the gravity scale that holds `joint` at `pose`, starting from `start`.
///
/// Used by step 2 to measure it and by step 3 to refine it when verification
/// disagrees: the same search either way, so a verification miss feeds back
/// into the number instead of discarding it.
/// The scale where the drift crosses zero, from a sagging probe and a lifting
/// one.
///
/// `None` when the probes do not straddle zero, when the two nearest ones are
/// the same scale, or when the crossing is close enough to `settled` that
/// testing it would only measure the noise.
fn bracket(probes: &[(f64, f64)], settled: f64) -> Option<f64> {
    let sag = probes
        .iter()
        .filter(|(_, d)| *d > 0.0)
        .min_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))?;
    let lift = probes
        .iter()
        .filter(|(_, d)| *d < 0.0)
        .min_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))?;
    let ds = lift.0 - sag.0;
    if ds.abs() < 1.0e-6 {
        return None;
    }
    let zero = sag.0 + (0.0 - sag.1) * ds / (lift.1 - sag.1);
    let (lo, hi) = if sag.0 < lift.0 {
        (sag.0, lift.0)
    } else {
        (lift.0, sag.0)
    };
    if zero <= lo || zero >= hi || (zero - settled).abs() < 0.005 {
        return None;
    }
    Some(zero)
}

fn search_gravity(
    arm: &mut Arm,
    joint: usize,
    pose: &[f64; par6_kin::NQ],
    settled: &[f64; par6_kin::NQ],
    start: f64,
) -> Result<f64, String> {
        let mut scale = start;
        let mut best: Option<(f64, f64)> = None;
    // Every (scale, drift) this search measured. The loop stops at the first
    // scale that holds, and which one that is depends on where the search
    // STARTED — so two runs of the same arm answer differently by as much as
    // the accept band is wide. The pairs are what fixes that: taken together
    // they locate the scale where the drift is zero, independent of the path
    // taken to them.
    let mut probes: Vec<(f64, f64)> = Vec::new();
    // The previous (scale, drift) pair, which is what makes the next step a
    // measurement rather than a guess.
        let mut last: Option<(f64, f64)> = None;
        for attempt in 0..GRAVITY_ATTEMPTS {
            // Back to the pose first. A joint that sagged 20 degrees on the
            // last attempt is somewhere much lighter now, and holding THERE
            // says nothing about holding here.
            if attempt > 0 {
                for (j, target) in pose.iter().enumerate().take(arm.n()) {
                    let s = arm.travel_time(j, *target);
                    arm.move_to(j, *target, s)?;
                }
            }
            if attempt == 0 {
                // Signed, both of them. A feedforward that is helping and one
                // that is fighting look identical in magnitude, and the whole
                // search assumes it is helping.
                let mut g = [0.0_f64; par6_kin::NQ];
                arm.kin
                    .gravity(pose, &mut g)
                    .map_err(|e| format!("gravity: {e}"))?;
                println!(
                    "    J{}: the model asks for {:+.3} Nm ({:+} mA); the drive is \
                     pulling {:+} mA to hold this pose",
                    joint + 1,
                    g[joint],
                    arm.torque_to_ma(joint, g[joint]),
                    arm.current_ma(joint).unwrap_or(0)
                );
            }
            let drift = arm.hold_on_gravity(joint, pose, scale, settled)?;
            let deg = arm.deg_for_ticks(joint, 0) + drift.to_degrees();
            println!(
                "    J{} at gravity x{scale:.3}: drifting {:+.4} deg/s{}",
                joint + 1,
                deg,
                if drift.abs() <= GRAVITY_HOLD_RAD_S {
                    " — holds"
                } else if drift > 0.0 {
                    // Positive IS falling: the rate is measured along the
                    // direction gravity pulls, which is -sign(G(q)).
                    " — sagging"
                } else {
                    " — lifting"
                }
            );
            probes.push((scale, drift));
            if best.is_none_or(|(_, d): (f64, f64)| drift.abs() < d.abs()) {
                best = Some((scale, drift));
            }
            // Comfortably inside, not merely inside. A scale accepted at the
            // edge of tolerance passes once and fails when it is checked,
            // which is what verification caught: the number was lucky, not
            // right.
            if drift.abs() <= GRAVITY_HOLD_RAD_S * ACCEPT_MARGIN {
                // And it has to do it twice, from a fresh approach. One hold
                // can be the tail of the last move rather than a property of
                // the gain.
                for (j, target) in pose.iter().enumerate().take(arm.n()) {
                    let s = arm.travel_time(j, *target);
                    arm.move_to(j, *target, s)?;
                }
                let again = arm.hold_on_gravity(joint, pose, scale, settled)?;
                println!(
                    "    J{} at gravity x{scale:.3}: {:+.4} deg/s on a second approach",
                    joint + 1,
                    again.to_degrees()
                );
                if again.abs() <= GRAVITY_HOLD_RAD_S {
                    best = Some((scale, again.abs().max(drift.abs()) * drift.signum()));
                    break;
                }
                // It did not repeat: the scale is not trusted, so the search
                // carries on from the pair it has rather than from a reading
                // that just failed to reproduce.
            }
            // Where to go next comes from how this joint ACTUALLY responded to
            // the last change, not from an assumption about which way
            // feedforward helps. Two attempts give a secant, and the secant
            // also exposes the case that matters: a joint whose drift does not
            // improve as its feedforward grows is not mis-compensated, and no
            // scale will fix it.
            let here = (scale, drift);
            if let Some((prev_scale, prev_drift)) = last {
                let ds = scale - prev_scale;
                if ds.abs() > 1.0e-9 {
                    let slope = (drift - prev_drift) / ds;
                    // More feedforward has to push a sagging joint back up, so
                    // the sag rate must FALL as the scale rises. Anything else
                    // means the sag is not the model's error.
                    // And the worsening has to be bigger than the scatter the
                    // reading itself has, or the search reacts to noise: two
                    // measurements of the SAME scale differ by about the
                    // tolerance, which is enough to fake any slope.
                    if slope >= 0.0 && (drift - prev_drift).abs() > GRAVITY_HOLD_RAD_S {
                        let gain = arm.scales[joint];
                        let next_gain = gain * GAIN_STEP;
                        if next_gain > GAIN_CEILING {
                            return Err(format!(
                                "J{} sags {:+.4} deg/s and sags no better as its gravity \
                                 feedforward is raised (x{prev_scale:.3} -> x{scale:.3}), \
                                 so the model is not what is wrong — and its velocity \
                                 gains are already at x{gain:.2}. This joint cannot hold \
                                 this pose.",
                                joint + 1,
                                drift.to_degrees()
                            ));
                        }
                        // A sag feedforward cannot fix is a feedback problem.
                        println!(
                            "    J{} sags worse as feedforward rises, so this is \
                             feedback, not gravity: velocity gains -> x{next_gain:.2}, \
                             restarting its search",
                            joint + 1
                        );
                        arm.scale_floor[joint] = next_gain;
                        arm.push_scale(joint, next_gain)?;
                        scale = 1.0;
                        last = None;
                        continue;
                    }
                    // Secant step straight at zero drift, damped so one noisy
                    // pair cannot throw the search across the range.
                    let target = scale - drift / slope;
                    scale += ((target - scale) * SECANT_DAMPING)
                        .clamp(-GRAVITY_MAX_STEP, GRAVITY_MAX_STEP);
                    last = Some(here);
                    if !(GRAVITY_SCALE_MIN..=GRAVITY_SCALE_MAX).contains(&scale) {
                        return Err(format!(
                            "J{} needs a gravity scale of x{scale:.2}, outside the range \
                             a per-arm difference explains — something else is wrong",
                            joint + 1
                        ));
                    }
                    continue;
                }
            }
            last = Some(here);
            // First move of the search, with no slope to go on yet: one
            // proportional step. Sagging (positive) means the model is
            // under-stating this joint's load, so the scale goes up.
            let step = 1.0 + GRAVITY_GAIN * (drift / GRAVITY_HOLD_RAD_S).clamp(-1.0, 1.0);
            scale *= step;
            if !(GRAVITY_SCALE_MIN..=GRAVITY_SCALE_MAX).contains(&scale) {
                return Err(format!(
                    "J{} needs a gravity scale of x{scale:.2}, outside the range a \
                     per-arm difference explains — something else is wrong",
                    joint + 1
                ));
            }
            if attempt + 1 == GRAVITY_ATTEMPTS {
                return Err(format!(
                    "J{} never held: best was {:+.4} deg at x{:.3}",
                    joint + 1,
                    best.map(|(_, d)| d.to_degrees()).unwrap_or_default(),
                    best.map(|(s, _)| s).unwrap_or(1.0)
                ));
            }
        }
    let (held, drift) = best.expect("at least one attempt");
    // Banked before anything optional runs. This scale has already held twice
    // from fresh approaches, and the refinement below can fail on the bus like
    // any other move — a failure there must not turn a joint that measured into
    // a joint that did not, which is how the caller would read it and what the
    // patch file would then leave out.
    arm.grav_scale[joint] = held;
    arm.results.gravity_scale[joint] = Some(held);
    // If the search saw the joint sag at one scale and lift at another, the
    // answer is between them, and interpolating is a measurement rather than a
    // guess. It is only adopted if it actually holds better than the scale the
    // search stopped on, so a bad interpolation costs one hold and nothing
    // else.
    let (held, drift) = match bracket(&probes, held) {
        Some(candidate) => {
            for (j, target) in pose.iter().enumerate().take(arm.n()) {
                let s = arm.travel_time(j, *target);
                arm.move_to(j, *target, s)?;
            }
            let d = arm.hold_on_gravity(joint, pose, candidate, settled)?;
            println!(
                "    J{}: the sag and the lift put zero at x{candidate:.4}; it drifts \
                 {:+.4} deg/s there",
                joint + 1,
                d.to_degrees()
            );
            if d.abs() < drift.abs() {
                arm.grav_scale[joint] = candidate;
                arm.results.gravity_scale[joint] = Some(candidate);
                (candidate, d)
            } else {
                (held, drift)
            }
        }
        None => (held, drift),
    };
    // This joint is holding the pose it was just calibrated for, so whatever it
    // is doing now is what it does when it is working. That is the figure the
    // oscillation rule has to beat.
    let swing = arm.hold_swing[joint];
    if swing > 0.0 {
        arm.ring_floor[joint] = arm.ring_floor[joint].max(swing);
        println!(
            "  J{}: it dithers {:.4} deg holding this pose; a ring has to beat \
             {:.4} deg",
            joint + 1,
            arm.deg_for_ticks(joint, swing as i64),
            arm.deg_for_ticks(joint, arm.ring_threshold(joint))
        );
    }
    println!(
        "  J{}: gravity scale x{held:.3} holds it to {:+.4} deg/s",
        joint + 1,
        drift.to_degrees()
    );
    Ok(held)
}

/// Step 3: prove the calibration, with the values it just measured in force.
///
/// A calibration is not finished when it produces numbers; it is finished when
/// the arm holds with those numbers in it. Every loaded joint goes back to the
/// pose that loads it most and is asked to hold on the scale that was chosen.
/// A joint that misses is measured again from there and re-verified, up to
/// [`VERIFY_ROUNDS`]; a pass is the claim the tool is allowed to make.
fn step3(arm: &mut Arm, scales: &[f64; par6_kin::NQ]) -> Result<(), String> {
    // A joint that misses here has not invalidated the run: it has produced one
    // more measurement of itself, at the same pose, with everything in force.
    // Throwing the whole calibration away over it — and with it the joints that
    // passed — is what made every failure cost a fresh start.
    let mut scales = *scales;
    for round in 0..VERIFY_ROUNDS {
        match verify_once(arm, &scales) {
            Ok(()) => return Ok(()),
            Err(failed) if round + 1 < VERIFY_ROUNDS && !failed.joints.is_empty() => {
                println!(
                    "  {} missed; refining {} and verifying again",
                    failed.detail,
                    if failed.joints.len() == 1 {
                        "it"
                    } else {
                        "them"
                    }
                );
                for joint in failed.joints {
                    let pose = arm.loaded_pose(joint)?;
                    // From the number verification just disagreed with, not
                    // from scratch: the previous search is evidence too.
                    let refined =
                        search_gravity(arm, joint, &pose, &scales, scales[joint])?;
                    scales[joint] = refined;
                    arm.grav_scale[joint] = refined;
                    arm.results.gravity_scale[joint] = Some(refined);
                }
            }
            Err(failed) => return Err(failed.detail),
        }
    }
    Ok(())
}

/// How many times verification may refine and retry before the run is a
/// failure.
const VERIFY_ROUNDS: usize = 3;

/// A failed verification: what to say, and which joints to go back to.
struct VerifyFailure {
    detail: String,
    joints: Vec<usize>,
}

impl VerifyFailure {
    /// Something went wrong that refining a scale cannot address.
    fn fatal(detail: String) -> Self {
        Self {
            detail,
            joints: Vec::new(),
        }
    }
}

fn verify_once(arm: &mut Arm, scales: &[f64; par6_kin::NQ]) -> Result<(), VerifyFailure> {
    println!("\nstep 3: verification, with the measured values in force");
    arm.verifying = true;
    let mut failures = Vec::new();
    let mut missed = Vec::new();
    for joint in [4usize, 3, 2, 1] {
        let pose = arm.loaded_pose(joint).map_err(VerifyFailure::fatal)?;
        for (j, target) in pose.iter().enumerate().take(arm.n()) {
            let s = arm.travel_time(j, *target);
            arm.move_to(j, *target, s).map_err(VerifyFailure::fatal)?;
        }
        let drift = arm
            .hold_on_gravity(joint, &pose, scales[joint], scales)
            .map_err(VerifyFailure::fatal)?;
        let held = drift.abs() <= GRAVITY_HOLD_RAD_S;
        println!(
            "  J{} at x{:.3}: {:+.4} deg/s — {}",
            joint + 1,
            scales[joint],
            drift.to_degrees(),
            if held { "holds" } else { "FAILS" }
        );
        if !held {
            missed.push(joint);
            failures.push(format!(
                "J{} drifting {:+.4} deg/s",
                joint + 1,
                drift.to_degrees()
            ));
        }
    }
    // Nothing may be ringing at the end either: an arm that holds its pose
    // while shaking is not calibrated.
    let ringing = arm.oscillating();
    if !ringing.is_empty() {
        failures.push(format!(
            "{} still oscillating",
            ringing
                .iter()
                .map(|(j, d)| format!("J{} at {d:.4} deg", j + 1))
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }
    if failures.is_empty() {
        println!("  verified: every loaded joint holds on its measured values, nothing ringing");
        return Ok(());
    }
    Err(VerifyFailure {
        detail: format!("verification failed: {}", failures.join("; ")),
        joints: missed,
    })
}

/// Home again `extra` times and report how far the references move.
///
/// A reference is only as good as its repeatability, and the number that
/// matters is not whether one seek found a stop but whether the next one finds
/// the same stop. Reported per joint in degrees, because a tick is a different
/// angle on every joint — fifty of them is 0.044 deg at the shoulder and
/// 0.172 deg at the base.
fn repeatability(arm: &mut Arm, extra: usize) -> Result<(), String> {
    let n = arm.n();
    let mut seen: Vec<Vec<i32>> = (0..n)
        .map(|j| arm.home_ticks[j].map(|p| vec![p]).unwrap_or_default())
        .collect();
    let mut took = Vec::new();
    for round in 0..extra {
        println!("\nrepeat {} of {extra}: homing again", round + 1);
        let began = Instant::now();
        // Forget every reference: a run that kept them would be measuring the
        // conversion it already had, not a fresh seek.
        arm.referenced.iter_mut().for_each(|r| *r = false);
        arm.vibration_fatal = false;
        step1(arm)?;
        arm.vibration_fatal = true;
        took.push(began.elapsed().as_secs_f64());
        for (j, ticks) in seen.iter_mut().enumerate() {
            if let Some(p) = arm.home_ticks[j] {
                ticks.push(p);
            }
        }
    }
    println!("\nhoming repeatability over {} runs:", extra + 1);
    for (j, ticks) in seen.iter().enumerate() {
        if ticks.len() < 2 {
            continue;
        }
        let lo = ticks.iter().copied().min().unwrap_or(0);
        let hi = ticks.iter().copied().max().unwrap_or(0);
        let spread = arm.deg_for_ticks(j, i64::from(hi) - i64::from(lo));
        let kind = match arm.homing[j].strategy {
            HomingStrategy::Hall => "hall edge",
            HomingStrategy::Stall => "endstop",
        };
        // A reference is relative to the stop it latches, so a drive whose
        // counter re-zeroes between runs is harmless — but it is not a spread,
        // and reporting it as one would hide a real one.
        if spread > COUNTER_EPOCH_DEG {
            println!(
                "  J{}  its counter moved between runs ({spread:.1} deg apart), so \
                 these references cannot be compared — the references themselves \
                 are still sound, each being relative to the stop it found",
                j + 1
            );
            continue;
        }
        println!("  J{}  {spread:.4} deg across {} runs  ({kind})", j + 1, ticks.len());
    }
    let slowest = took.iter().copied().fold(0.0_f64, f64::max);
    println!(
        "  slowest repeat {slowest:.1} s; every run must home inside the time the \
         operator is willing to wait"
    );
    Ok(())
}

/// Step 1: home the arm, making every joint arrive.
fn step1(arm: &mut Arm) -> Result<(), String> {
    // Let the boot config land and every node report before the first move.
    arm.wait_for_readings()?;
    // Nothing has a measured feedforward yet, so every loaded joint gets the
    // wider pose window from the first move rather than only in step 2.
    for joint in [4usize, 3, 2, 1] {
        arm.awaiting_gravity[joint] = true;
    }
    println!("step 1: home on the configured (vendor) gains");
    let steps = arm.robot.homing.sequence.clone();
    for (i, step) in steps.iter().enumerate() {
        println!("sequence step {}/{}", i + 1, steps.len());
        for m in &step.pre_moves {
            match *m {
                PreMove::Nudge {
                    joint,
                    speed_ticks_s,
                    duration_s,
                } => arm.nudge(usize::from(joint), speed_ticks_s, duration_s)?,
                PreMove::Idle { duration_s, .. } => arm.coast(duration_s)?,
                PreMove::Position {
                    joint,
                    position_rad,
                    duration_s,
                } => arm.move_to(usize::from(joint), position_rad, duration_s)?,
                // The gripper is not part of arm calibration and homing it
                // needs its own driver; skipped, and said so.
                PreMove::GripperMove { .. } => println!("  gripper pre-move skipped"),
            }
        }
        if let Some(group) = &step.home {
            for j in &group.joints {
                arm.home_joint(usize::from(*j))?;
            }
            if group.gripper.is_some() {
                println!("  gripper homing skipped");
            }
        }
        for m in &step.move_to {
            arm.move_to(usize::from(m.joint), m.position_rad, m.duration_s)?;
        }
    }
    Ok(())
}

/// Counting allocator, test builds only, so the tick path's own rule can be
/// asserted rather than read.
#[cfg(test)]
mod counting {
    use std::alloc::{GlobalAlloc, Layout, System};
    use std::sync::atomic::{AtomicU64, Ordering};

    pub static ALLOCS: AtomicU64 = AtomicU64::new(0);

    pub struct CountingAlloc;

    // SAFETY: every operation is delegated to the system allocator unchanged;
    // the counter is a relaxed atomic side effect.
    unsafe impl GlobalAlloc for CountingAlloc {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            ALLOCS.fetch_add(1, Ordering::Relaxed);
            unsafe { System.alloc(layout) }
        }

        unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
            ALLOCS.fetch_add(1, Ordering::Relaxed);
            unsafe { System.alloc_zeroed(layout) }
        }

        unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
            ALLOCS.fetch_add(1, Ordering::Relaxed);
            unsafe { System.realloc(ptr, layout, new_size) }
        }

        unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
            unsafe { System.dealloc(ptr, layout) }
        }
    }
}

#[cfg(test)]
#[global_allocator]
static COUNTING: counting::CountingAlloc = counting::CountingAlloc;

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::Ordering;

    /// A joint that is FALLING reads as a positive drift rate.
    ///
    /// This is the convention the whole gravity search rests on, and getting
    /// it backwards is not a compile error: the label said "sagging" while the
    /// joint lifted, the first step moved the scale the wrong way, and the
    /// slope test concluded the feedforward was not helping. Those three read
    /// the same number, so one test pins all three.
    #[test]
    fn falling_is_a_positive_drift_rate() {
        // G(q) > 0 means the motor must pull positive to hold, so gravity
        // accelerates the joint NEGATIVE: falling is a decreasing angle.
        // Raw encoder angles, exactly as the watch records them.
        let ramp = |per_s: f64| -> Vec<(f64, f64)> {
            (0..100)
                .map(|k| {
                    let t = f64::from(k) * 0.004;
                    (t, 0.5 + per_s * t)
                })
                .collect()
        };
        // G(q) > 0: the motor pulls positive to hold, so falling decreases the
        // angle.
        let rate = sag_rate(&ramp(-0.02), 1.7).expect("enough samples to fit");
        assert!(rate > 0.0, "a falling joint must read as a sag, got {rate}");
        assert!((rate - 0.02).abs() < 1e-6, "and at its real rate: {rate}");
        let rate = sag_rate(&ramp(0.02), 1.7).expect("enough samples to fit");
        assert!(rate < 0.0, "a lifting joint must read negative, got {rate}");

        // G(q) < 0: falling is now an INCREASING angle, and still a sag.
        let rate = sag_rate(&ramp(0.02), -1.7).expect("enough samples to fit");
        assert!(rate > 0.0, "sign of G(q) must not change the verdict: {rate}");
        let rate = sag_rate(&ramp(-0.02), -1.7).expect("enough samples to fit");
        assert!(rate < 0.0, "nor the other verdict: {rate}");
    }

    #[test]
    fn a_fit_needs_more_than_a_couple_of_samples() {
        assert!(sag_rate(&[(0.0, 0.0), (0.004, 0.1)], 1.0).is_none());
        // All at one instant: no slope exists, and dividing by zero spread
        // would invent one.
        let stacked: Vec<(f64, f64)> = (0..20).map(|_| (0.5, 1.0)).collect();
        assert!(slope_per_s(&stacked).is_none());
    }

    /// Ring and droop are opposite failures with the same symptom, and the
    /// tool does opposite things about them: lower the gains, or raise them.
    #[test]
    fn ringing_and_drifting_are_told_apart() {
        let amplitude = 20_i64;
        // A ring: reversals that keep coming, no net travel.
        let ring: Vec<i32> = (0..RING_WINDOW_TICKS)
            .map(|k| (60.0 * ((k as f64) * 0.35).sin()) as i32)
            .collect();
        assert!(ringing_now(&ring, amplitude), "a shaking joint is ringing");
        assert!(!drifting_now(&ring, amplitude), "and it is not drifting");

        // A droop: one way, no reversals.
        let droop: Vec<i32> = (0..RING_WINDOW_TICKS).map(|k| -(k as i32) / 4).collect();
        assert!(drifting_now(&droop, amplitude), "a sliding joint is drifting");
        assert!(!ringing_now(&droop, amplitude), "and it is not ringing");

        // A COMMANDED MOVE is neither, however far it travels: the trend
        // explains all of it. This is the case that failed a run for the wrist
        // doing exactly what it was told.
        let commanded: Vec<i32> = (0..RING_WINDOW_TICKS)
            .map(|k| (k as i32) * 9 + if k % 3 == 0 { 1 } else { 0 })
            .collect();
        assert!(
            !ringing_now(&commanded, amplitude),
            "a move is not a ring, however large its span: {} ticks about trend",
            ring_swings(&commanded).1
        );
        // A ring RIDING a move still is one: that is the case the continuous
        // check exists for.
        let both: Vec<i32> = (0..RING_WINDOW_TICKS)
            .map(|k| (k as i32) * 9 + (60.0 * ((k as f64) * 0.35).sin()) as i32)
            .collect();
        assert!(ringing_now(&both, amplitude), "a ring on top of a move is a ring");

        // A joint sitting still is neither, however long it is watched.
        let still = vec![1234_i32; RING_WINDOW_TICKS];
        assert!(!ringing_now(&still, amplitude));
        assert!(!drifting_now(&still, amplitude));

        // Dither below the amplitude that counts is neither: the elbow was
        // failed at every gain for moving less than its own arrival window.
        let dither: Vec<i32> = (0..RING_WINDOW_TICKS)
            .map(|k| if k % 2 == 0 { 0 } else { amplitude as i32 / 4 })
            .collect();
        assert!(!ringing_now(&dither, amplitude));
    }

    /// The continuous check must be able to see a ring at all.
    ///
    /// It could not: the per-joint history was capped at 250 samples while the
    /// detector refused to judge fewer than 500, so every joint came back
    /// "quiet" and the rule the tool reports on was inert. A length mismatch
    /// like that is invisible — the output says the right thing either way.
    #[test]
    fn the_watch_window_is_long_enough_to_judge_a_ring() {
        // The length relationship itself is a compile-time assertion beside the
        // constants. What is left to check is that a full window of real
        // shaking comes back as a ring.
        let ring: Vec<i32> = (0..WATCH_WINDOW_TICKS)
            .map(|k| (60.0 * ((k as f64) * 0.35).sin()) as i32)
            .collect();
        assert!(ringing_now(&ring, 20));
    }

    /// The burst figure has to survive a COMMANDED reversal.
    ///
    /// The report's "worst swing" exists to catch a shake too short to move a
    /// median. Measured without a bound on how fast a reversal has to be, a
    /// seek that drives one way and backs off the other counts the whole of
    /// that travel as one swing: the simulated run reported 60 deg on the base
    /// and 69 deg on the wrist, which is both joints doing exactly what they
    /// were told.
    #[test]
    fn a_slow_reversal_is_travel_and_a_fast_one_is_a_shake() {
        let mut turns: Vec<i64> = Vec::new();
        let out_and_back: Vec<i32> = (0..RING_WINDOW_TICKS)
            .map(|k| {
                let half = RING_WINDOW_TICKS as i32 / 2;
                let k = k as i32;
                if k < half { k * 20 } else { (RING_WINDOW_TICKS as i32 - k) * 20 }
            })
            .collect();
        let travel = swings_into(&out_and_back, &mut turns);
        assert_eq!(
            travel.worst, 0.0,
            "a single slow reversal is travel, not a swing"
        );

        let ring: Vec<i32> = (0..RING_WINDOW_TICKS)
            .map(|k| (60.0 * ((k as f64) * 0.35).sin()) as i32)
            .collect();
        let shaking = swings_into(&ring, &mut turns);
        assert!(
            shaking.worst >= 100.0 && shaking.reversals >= RING_REVERSALS,
            "a 3 Hz shake is measured: {shaking:?} swings"
        );
        // And the burst the median hides: a short buzz on top of the single
        // encoder tick every joint dithers by. The median is the dither,
        // because that is most of the window; the buzz is only in the worst.
        let burst: Vec<i32> = (0..RING_WINDOW_TICKS)
            .map(|k| {
                let dither = (k % 2) as i32;
                if (100..140).contains(&k) {
                    dither + (k % 2) as i32 * 40
                } else {
                    dither
                }
            })
            .collect();
        let bursting = swings_into(&burst, &mut turns);
        assert!(
            bursting.worst >= 40.0 && bursting.median < bursting.worst,
            "the burst is in the worst figure and not in the median: {bursting:?}"
        );
    }

    /// The tick path allocates nothing once it is running.
    ///
    /// The house rule for an RT tick path, and this tool is one: it paces the
    /// drives itself. A six-element Vec per tick is cheap right up until the
    /// page it wants is not resident, and one run on the arm showed a 24 ms
    /// tick — against a 4 ms period — while the feedforward was being built
    /// into a fresh Vec every time.
    #[test]
    fn the_tick_path_allocates_nothing_once_it_is_running() {
        let assets = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/par6_description");
        let config = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
        let bundle = ConfigBundle::load(&config).expect("config");
        let mut arm = Arm::open(&bundle, &assets, true).expect("simulated arm");
        // With the rule ARMED. Left off, the oscillation check returns on its
        // first line and the measurement behind it — which runs on every joint
        // on every tick — is never reached, so the test passed while that path
        // allocated. The same shape of gap made the rule itself inert once.
        arm.vibration_fatal = true;

        // Warm everything the first tick would touch: the bus's own buffers,
        // the gravity model's lazy setup, the history vectors.
        for _ in 0..WATCH_WINDOW_TICKS + 10 {
            arm.tick_frame(None).expect("warm-up tick");
        }

        let before = counting::ALLOCS.load(Ordering::Relaxed);
        for _ in 0..200 {
            arm.tick_frame(None).expect("tick");
        }
        let plain = counting::ALLOCS.load(Ordering::Relaxed) - before;

        let before = counting::ALLOCS.load(Ordering::Relaxed);
        for _ in 0..200 {
            arm.tick_each(|_, _| JointCommand::current(0)).expect("tick");
        }
        let each = counting::ALLOCS.load(Ordering::Relaxed) - before;

        assert_eq!(plain, 0, "tick_frame allocated {plain} times in 200 ticks");
        assert_eq!(each, 0, "tick_each allocated {each} times in 200 ticks");
    }

    #[test]
    fn a_profile_starts_and_ends_at_rest() {
        let (pos, vel) = hermite(100.0, 900.0, 0, 250, 0.004);
        assert!((pos - 100.0).abs() < 1e-9, "starts where the joint is");
        assert!(vel.abs() < 1e-9, "and at rest");
        let (pos, vel) = hermite(100.0, 900.0, 250, 250, 0.004);
        assert!((pos - 900.0).abs() < 1e-9, "ends on target");
        assert!(vel.abs() < 1e-9, "and at rest");
        // Halfway through a symmetric profile is halfway there, moving.
        let (pos, vel) = hermite(100.0, 900.0, 125, 250, 0.004);
        assert!((pos - 500.0).abs() < 1e-6, "symmetric at the midpoint");
        assert!(vel > 0.0, "and moving toward the target");
    }

    /// `--apply` edits the config as text, so this is the test that it edits
    /// the RIGHT text — on the config this arm actually ships, because that is
    /// the file whose shape matters: comments explaining why its numbers are
    /// what they are, and a gains section that repeats once per joint.
    #[test]
    fn apply_patches_the_named_joint_and_keeps_the_comments() {
        let shipped = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../config/PAR6.toml")
            .canonicalize()
            .expect("the shipped config");
        let original = std::fs::read_to_string(&shipped).expect("read shipped config");
        let dir = std::env::temp_dir().join(format!("selfcal-apply-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("temp dir");
        let path = dir.join("PAR6.toml");
        std::fs::write(&path, &original).expect("copy config");
        // The bundle reads the gripper library beside the config, and the
        // patched file is re-validated below, so the copy needs one too.
        let grippers = dir.join("grippers");
        std::fs::create_dir_all(&grippers).expect("gripper dir");
        for entry in std::fs::read_dir(shipped.parent().unwrap().join("grippers")).expect("grippers")
        {
            let entry = entry.expect("gripper entry");
            std::fs::copy(entry.path(), grippers.join(entry.file_name())).expect("copy gripper");
        }
        let bundle = ConfigBundle::load(&path).expect("load the copy");
        let robot = bundle.robot.clone();

        let mut results = Results::new(robot.joints.len());
        // The ELBOW's gains, the SHOULDER's seek current, one scale: three
        // different sections, each of which repeats per joint.
        results.gain_scale[2] = 1.25;
        results.seek_ma[1] = Some(1125.0);
        results.gravity_scale[2] = Some(1.08);
        results.apply(&path, &robot).expect("apply");
        let patched = std::fs::read_to_string(&path).expect("read back");

        let kpv_expected = format!("kpv = {:.6}", robot.joints[2].gains.kpv * 1.25);
        assert!(
            patched.contains(&kpv_expected),
            "the elbow gets x1.25 of its own vendor value ({kpv_expected})"
        );
        assert!(
            patched.contains("current_ma = 1125.0"),
            "the shoulder's seek current is written"
        );
        assert!(
            patched.contains("gravity_scale = [1.0000, 1.0000, 1.0800"),
            "the scale lands at the top level as a bare key"
        );
        // Untouched joints keep their own values, not the patched joint's.
        let j0 = format!("kpv = {}", robot.joints[0].gains.kpv);
        assert!(
            patched.contains(&j0) || patched.contains(&format!("kpv = {:.3}", robot.joints[0].gains.kpv)),
            "joint 1 is left alone"
        );
        let backup = std::fs::read_to_string(dir.join("PAR6.toml.before-selfcal")).expect("backup");
        assert_eq!(backup, original, "the backup is the file as it was");

        // A SECOND run reads the config the first one wrote. Its gains are
        // relative to that, its gravity scales carry where it measured none,
        // and the comment has to describe the drive rather than the last edit.
        let again = ConfigBundle::load(&path).expect("load the patched config");
        let mut second = Results::new(again.robot.joints.len());
        second.gain_scale[2] = 1.25;
        second.apply(&path, &again.robot).expect("apply again");
        let twice = std::fs::read_to_string(&path).expect("read back");
        let compounded = format!("kpv = {:.6}", again.robot.joints[2].gains.kpv * 1.25);
        assert!(
            twice.contains(&compounded),
            "the second run raises the gain the first run wrote ({compounded})"
        );
        // The shipped config already carries x1.250 from an earlier session,
        // so two more steps of x1.25 put the drive on x1.953 of the vendor
        // value — which is what the comment has to say.
        assert!(
            twice.contains("x1.953 of the vendor value"),
            "and says what the drive is on, not what this run changed:\n{twice}"
        );
        assert!(
            twice.contains("gravity_scale = [1.0000, 1.0000, 1.0800"),
            "a run that measured no scale leaves the measured one alone"
        );

        // Comments are the reason this is a text edit rather than a round trip
        // through a serialiser.
        let comments_before = original.lines().filter(|l| l.trim_start().starts_with('#')).count();
        let comments_after = patched.lines().filter(|l| l.trim_start().starts_with('#')).count();
        assert!(
            comments_after >= comments_before,
            "every comment survives: {comments_before} before, {comments_after} after"
        );
        // And the patched file is still a config the runtime can load.
        ConfigBundle::load(&path).expect("the patched config still validates");

        // The backup is one step of undo, so after the second apply it is the
        // file the second run started from — copying it back returns the arm
        // to the config it was just running, not to the vendor's.
        let undo = std::fs::read_to_string(dir.join("PAR6.toml.before-selfcal")).expect("backup");
        assert_eq!(undo, patched, "the backup undoes the run that wrote it");
        std::fs::remove_dir_all(&dir).ok();
    }
}
