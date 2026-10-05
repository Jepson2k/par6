//! Calibrate a PAR6: everything a new arm needs measured, in one run.
//!
//! In this order, each stage building on what came before:
//!
//! 1. HOME: find every joint's reference against its endstop or hall.
//! 2. RIPPLE: the current that cancels each joint's cogging and commutation
//!    ripple, from slow sweeps against the rotor's electrical angle. First,
//!    because every verdict after it reads speed: uncompensated, J4's first
//!    harmonic at 47 deg/s is a 25 Hz, 10 deg/s RMS swing that reads as a
//!    hunting loop (2026-10-01).
//! 3. GAINS: tune Kpv, Kiv and Kpp, qualify normal motion and representative
//!    calibration poses, then keep the qualified gains for measurements.
//! 4. STICTION: ramp each joint's current both ways from its hold until it
//!    slides, at ready and with the arm out.
//! 5. BELT: a current chirp on the base, for the arm's compliance.
//! 6. MECHANICS: each joint's friction from constant-velocity sweeps, then
//!    held poses to fit this arm's own link masses, which are 3D printed and
//!    so are not the vendor's.
//! 7. LIMITS, only with `--limits`: each joint's velocity, acceleration and
//!    jerk limits.
//! 8. PARK: return the shoulder and elbow to their stops, then release.
//!
//! `--only <stage>` (repeatable) runs homing and just those stages, for
//! development.
//!
//! The gains procedure follows the StepFOC guide supplied by the user.
//! Numeric observation thresholds and gain increments are automation choices;
//! the guide prescribes the sequence and Kpv's 20% backoff.
//!
//! A number that is the same on every run is a constant here, not a flag. The
//! justification still gets written down; it costs one comment instead of a
//! flag, a parser, a struct field and a log line to keep in sync.

use par6_bus::spectral::codec::{CAPTURE_LEN, CAPTURE_STATUS_CHANNEL, CAPTURE_VEL_SCALE};
use par6_bus::{
    hw::SocketCanBus,
    sim::{
        scene::{Scene, Tool},
        SimBus,
    },
    spectral::{torque_to_ma_factor, JointConversion},
    BusError, BusState, ConfigKind, DriveTune, DriverBus, ErrorFlags, GripperCommand, JointCommand,
    PollAction, PollKind, RuntimeBus,
};
use par6_config::{ConfigBundle, Gains, LimitMode, PreMove, RippleHarmonic};
use par6_motion::{SSeptic, SEPTIC_PEAK_ACC, SEPTIC_PEAK_VEL};
use par6_rt::homing::{Homer, HomerEvent, HomerParams};
use par6d::ripple;
use socketcan::{CanSocket, EmbeddedFrame, Frame as _, Socket, SocketOptions};
use std::{
    collections::VecDeque,
    fmt::Write as _,
    fs,
    io::Write as _,
    path::{Path, PathBuf},
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc::{sync_channel, Receiver, SyncSender},
    },
    time::Duration,
};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
const N: usize = 6;
const LOOP_HZ: f64 = 6250.0;

static CANCEL: AtomicBool = AtomicBool::new(false);
extern "C" fn cancel(_: libc::c_int) {
    CANCEL.store(true, Ordering::Relaxed);
}

// ---------------------------------------------------------------- constants

/// User requirement, 2026-09-18: movement 5 deg/s, holding 1 deg/s RMS.
/// Application acceptance limits, not values derived from a paper.
const MOVING_RMS_RAD_S: f64 = 5.0 * std::f64::consts::PI / 180.0;
const HOLDING_RMS_RAD_S: f64 = std::f64::consts::PI / 180.0;

/// Following error a position move may carry per unit of commanded speed, in
/// rad per rad/s -- the position loop's own time constant.
///
/// Peak against peak, never sample by sample: a septic's commanded speed
/// passes through zero at both ends while the error does not, so an
/// instantaneous ratio diverges there (a J5 leg 1.29x over on peaks read 833x
/// on its quietest sample, 2026-09-21). Measured per unit speed over run
/// 1789992107916503931: J2 0.0016, J6 0.0024, J3 0.0029, J1 0.0036, against
/// J4 0.0122 and J5 0.0204 -- the two 4:1 wrist joints the arm is audibly
/// rough on. User decision 2026-09-21: 0.006 clears every healthy joint by
/// 1.7x and fails both wrist joints.
const FOLLOWING_S: f64 = 0.006;

/// Kollmorgen's velocity tuning guide recommends 1 s as an initial service
/// interval when settling time is unknown. A finite measurement interval, not
/// proof of indefinite stability.
/// https://www.kollmorgen.com/en-us/developer-network/akd-online-tuning-guide
const HOLD_OBSERVATION_S: f64 = 1.0;

/// How long a drive may go without fresh motion feedback before the run treats
/// it as lost. `[bus].stale_warn_s` is a self-clearing warning and the wrong
/// knob for this. Measured over four runs (467k samples each): every joint is
/// answered within 1 tick almost always, 3 at worst. On 2026-09-19 J1 alone
/// went unanswered for 10 consecutive ticks while the other five answered
/// every tick.
const FEEDBACK_TIMEOUT_S: f64 = 0.2;

/// Velocity is differenced from the encoder count over this window, never read
/// from the drive's own speed field: that field is one count difference at
/// 6250 Hz through a 20-sample moving average (STEPFOC motor_control.cpp
/// `Velocity`/`Velocity_Filter`), and on this arm it disagreed with the
/// encoder by 10% of peak on J4 and reported 3.2 deg/s on a J1 that was
/// commanded still and had not moved. Two ticks at 250 Hz: the count floor
/// stays under 0.7 deg/s on the coarsest joint, while anything longer averages
/// away the chatter the acceptance is there to catch.
const SPEED_WINDOW_S: f64 = 0.008;

/// Vendor stall-current threshold, detection window, occupancy and travel
/// floor -- behaviour and constants only. These detect contact, not acceptable
/// tracking.
/// https://github.com/Source-Robotics/RCB-Runtime/blob/main/robotics/homing.py
const STALL_CURRENT_FRACTION: f64 = 0.7;
const STALL_WINDOW_S: f64 = 0.08;
const STALL_OCCUPANCY: f64 = 0.6;
const STALL_TRAVEL_FRACTION: f64 = 0.25;
const STALL_TRAVEL_FLOOR: f64 = 10.0;
/// The vendor startup guard excludes acceleration transients; our profile has
/// an explicit ramp, so the window opens after it. This also gives the drive's
/// velocity integral, which V108 keeps across motion commands, time to unwind.
const DETECT_GUARD_S: f64 = 0.15;

/// Constant-velocity friction sweep: how many speeds, and as what fraction of
/// the fastest leg the travel budget allows.
///
/// Four speeds give the two-parameter line two degrees of freedom to spare,
/// and a 4x span between the slowest and fastest is what separates the
/// viscous slope from the Coulomb intercept -- at one speed they are one
/// number.
const DRAG_LEVELS: usize = 4;
const DRAG_MIN_FRACTION: f64 = 0.15;
const DRAG_MAX_FRACTION: f64 = 0.60;
/// Most arc one leg traverses. Both directions run over it, so the joint
/// returns to where it started; a joint with less room than that, by its
/// checked span, gets a share of the room it has.
const DRAG_TRAVEL_RAD: f64 = 0.5;
/// Time after the ramp for the velocity loop to settle at the commanded
/// speed, then the window the current is averaged over.
const DRAG_SETTLE_S: f64 = 0.35;
const DRAG_AVERAGE_S: f64 = 0.35;
/// Hold still between legs so the next one starts from rest.
const DRAG_RECOVER_S: f64 = 0.3;
/// How far the held speed may sit from the commanded one before the leg is
/// discarded: a joint still accelerating is paying current for inertia, which
/// would be read as friction.
const DRAG_SPEED_TOLERANCE: f64 = 0.15;
/// Below this the joint is dithering in its own backlash, not running.
const COAST_MIN_RAD_S: f64 = 0.02;

/// Limits stage (`--limits`): each step scales a joint's EXEC caps by this.
/// A quarter at a time leaves the last pass within 25% of where the joint
/// first fails, and finds three-fold headroom in five steps.
const LIMITS_STEP: f64 = 1.25;
/// Below this fraction of its configured limits a joint that still cannot
/// meet the requirements has a fault, not a limit, and is not moved further.
const LIMITS_MIN_FACTOR: f64 = 0.4;
/// Most travel a probe move covers on each side of the ready pose \[rad\].
const LIMITS_SPAN_RAD: f64 = 1.0;
/// The short probe move's length, as a fraction of the length at which speed
/// would start to bind: short enough that acceleration and jerk set its
/// duration, long enough to reach 70% of the cap on the way.
const LIMITS_SHORT_FRACTION: f64 = 0.5;
/// Joint-space step the probe span is collision-checked at \[rad\].
const LIMITS_SPAN_STEP_RAD: f64 = 0.02;
/// Speed ripple, as a fraction of the commanded peak speed, above which a
/// step fails whatever its following error. Ripple cannot rank steps, but
/// J6 passed the following-error rule at 16.8% and audibly rumbled (user,
/// 2026-09-23); the joints that run quietly sit under 8%.
const LIMITS_RIPPLE_CEILING: f64 = 0.10;
/// The gains step: a velocity step this fast, joint side \[rad/s\], or
/// slower when the capture window's travel would not fit
/// `GAINS_ROOM_SHARE` of the joint's clear span. Fast enough that the
/// speed quantum (312 ticks/s through the drive's filter) is a small part
/// of it on a 4:1 wrist; slow enough to be quiet.
const GAINS_STEP_RAD_S: f64 = 20.0 * std::f64::consts::PI / 180.0;
const GAINS_ROOM_SHARE: f64 = 0.6;
/// Every third control loop: 1024 samples cover 0.49 s, several periods of
/// the 12-15 Hz ring the arm's joints show, at 2083 Hz.
const GAINS_CAPTURE_DIVISOR: u8 = 3;
/// Status requests before a drive counts as recording no capture.
const GAINS_STATUS_TRIES: u32 = 3;
/// Read passes over the chunks a lossy bus did not answer.
const GAINS_READ_PASSES: u32 = 3;
/// How close to an endstop a sample stops counting as tracking. User
/// requirement, 2026-09-21: within this much of a stop a joint is nudging
/// into, or breaking away from, a mechanical limit, and what it does there
/// says nothing about its gains.
const ENDSTOP_EXCLUSION_RAD: f64 = 5.0 * std::f64::consts::PI / 180.0;
/// How long a streamed capture is given to land: 1536 frames at the drive's
/// 320 µs pace is half a second, and the host's own traffic shares the bus.
/// Pairs still missing after it are read one at a time.
const CAPTURE_STREAM_WAIT_S: f64 = 1.0;
/// The arm may not stand still longer than this between motions, anywhere in
/// a run: every second of it is the operator's. A longer stop is reported
/// when motion resumes, with what the run was doing, and the run's total is
/// printed at the end.
const IDLE_LIMIT_S: f64 = 1.0;
/// A joint has moved when its encoder left the count it rested at by more
/// than this; the count of noise a still joint reads is not motion.
const IDLE_MOTION_TICKS: i64 = 2;
/// After a trial runs away: how long the joint is held, and how much of the
/// start of that a spinning-down joint may still read loud.
const CALM_S: f64 = 0.5;
const CALM_QUIET_S: f64 = 0.2;
/// How far a joint may move with its loop open while it is calmed.
const CALM_OPEN_TRAVEL_RAD: f64 = 5.0 * std::f64::consts::PI / 180.0;
/// The ripple sweep's speed \[motor ticks/s\]: about two electrical cycles a
/// second on the arm's 50-pole-pair steppers, slow enough that the velocity
/// loop has the ripple in hand and the current it spends is what cancels it.
const RIPPLE_SWEEP_TICKS_S: f64 = 655.0;
/// Every twentieth loop: 1024 samples cover 3.3 s, six electrical cycles.
const RIPPLE_CAPTURE_DIVISOR: u8 = 20;
/// Harmonics of the electrical phase fitted and fed forward: the 1x and 2x
/// commutation error and the 4x detent the arm's captures showed, and the
/// 3x, 6x and 8x still left on J3, J4 and J6 once those were cancelled.
const RIPPLE_HARMONICS: [u8; 6] = [1, 2, 3, 4, 6, 8];
/// Each sweep's start, while the loop takes up the speed, left out of the
/// fit \[s\].
const RIPPLE_SETTLE_S: f64 = 0.3;
/// The gains step's rise, left out when its speed ripple is measured \[s\].
const RIPPLE_STEP_SKIP_S: f64 = 0.15;
/// The feedforward stays only if it cuts the speed ripple at those
/// harmonics, measured on the gains step, by this share.
const RIPPLE_MIN_IMPROVEMENT: f64 = 0.2;
/// The refinement step extrapolates from two captures; no harmonic of it
/// may exceed the current limit over this.
const RIPPLE_REFINE_ILIM_SHARE: f64 = 8.0;
/// A speed error this large for `RUNAWAY_TICKS` in a row is a loop that
/// has gone unstable, not a rough joint: J2's backlash chatter spikes to
/// 77 deg/s for single samples, the J1 buzz sat at +-240 deg/s.
const RUNAWAY_RAD_S: f64 = 90.0 * std::f64::consts::PI / 180.0;
const RUNAWAY_TICKS: u32 = 3;
/// The breakaway ramp: current climbs from the gravity balance at this
/// share of the joint's current limit per second, slow enough that the
/// encoder reports the first movement within a few mA of the current
/// that caused it, and gives up at `STICTION_MAX_ILIM` of the limit
/// beyond the balance.
const STICTION_RAMP_ILIM_PER_S: f64 = 0.05;
const STICTION_MAX_ILIM: f64 = 0.6;
/// Sustained sliding: the encoder advancing at this rate \[motor
/// ticks/s\] over `STICTION_WINDOW_S` is the link moving. The ramp winds a
/// transmission at only a few ticks per second on every joint (its torque
/// rate over a stiffness of hundreds of Nm/rad), and a joint that has
/// broken away accelerates past this within a fraction of a second.
const STICTION_SLIDE_TICKS_PER_S: f64 = 100.0;
const STICTION_WINDOW_S: f64 = 0.2;
/// Total encoder travel a breakaway must also show, past position noise.
const STICTION_BREAK_TICKS: i32 = 40;
/// A joint is still when its encoder stays within this over the window;
/// each ramp waits for that, up to the timeout, before it starts.
const STICTION_STILL_TICKS: i32 = 2;
const STICTION_STILL_TIMEOUT_S: f64 = 5.0;
/// No breakaway counts before the ramp has advanced this share of the
/// current limit: whatever moves at the balance current is not friction.
const STICTION_MIN_RAMP_ILIM: f64 = 0.01;
/// The belt chirp: J1 in current mode at the ready pose, a sine of this
/// torque \[Nm, joint side\] about the held current sweeping
/// `BELT_F_LO_HZ` to `BELT_F_HI_HZ` over `BELT_SECONDS`. The rotor's
/// answer against a rigid inertia shows the arm's compliance about the
/// base axis, which `[sim] arm_lateral_stiffness_nm_rad` carries. Bounded
/// by `BELT_ABORT_RAD` of travel, which stops the chirp and holds, and
/// only run with `BELT_CLEARANCE` times that clear on both sides.
const BELT_CHIRP_NM: f64 = 0.35;
const BELT_F_LO_HZ: f64 = 2.0;
const BELT_F_HI_HZ: f64 = 60.0;
const BELT_SECONDS: f64 = 6.0;
const BELT_ABORT_RAD: f64 = 5.0 * std::f64::consts::PI / 180.0;
const BELT_CLEARANCE: f64 = 2.0;
/// The ease back from a current-mode excursion takes at least this long
/// \[s\]; the EXEC caps stretch it further when the distance needs it.
const RETURN_S: f64 = 1.0;

/// Velocity, acceleration and jerk caps for one joint's moves.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Caps {
    velocity: f64,
    acceleration: f64,
    jerk: f64,
}

impl Caps {
    /// The septic a move of `distance_rad` runs under these caps.
    fn profile(&self, distance_rad: f64, min_seconds: f64, floor: f64) -> SSeptic {
        SSeptic::new(
            self.velocity / distance_rad,
            self.acceleration / distance_rad,
            self.jerk / distance_rad,
            Some(min_seconds),
            floor,
        )
    }

    /// Each cap scaled by `k`, none above `ceiling`.
    fn scaled(&self, k: f64, ceiling: &Caps) -> Caps {
        Caps {
            velocity: (k * self.velocity).min(ceiling.velocity),
            acceleration: (k * self.acceleration).min(ceiling.acceleration),
            jerk: (k * self.jerk).min(ceiling.jerk),
        }
    }
}

/// What the limits stage found for one joint.
#[derive(Clone, Copy, Debug)]
struct Found {
    caps: Caps,
    /// The probe span was long enough for the velocity cap to bind; when it
    /// was not, the velocity is a lower bound.
    velocity_reached: bool,
    /// The jerk is a limit the joint reached: the search ended on a failing
    /// step or at the ceiling. When it ended instead because no probe move
    /// was limited by the jerk any more, the value is only a lower bound, and
    /// one high enough to switch jerk limiting off -- so it is not written.
    jerk_measured: bool,
    /// Worst speed ripple of the step's probe moves, as a fraction of each
    /// move's peak commanded speed.
    ripple: f64,
    /// The EXEC caps the search started from, so `--apply` writes only what
    /// it moved.
    configured: Caps,
}

/// A loop-rate capture as read back, one row per sample: the drive's speed
/// \[ticks/s\], its measured Iq \[mA\] and, when asked for, the rotor's
/// electrical phase (0..16383 per cycle).
#[derive(Clone, Debug)]
struct Captured {
    speed: Vec<f64>,
    current: Vec<f64>,
    phase: Vec<f64>,
    divisor: usize,
}

#[derive(Clone, Copy)]
struct DragSample {
    position: f64,
    speed: f64,
    current: f64,
}

impl DragSample {
    fn at(samples: &[Self], position: f64) -> Option<Self> {
        samples.windows(2).find_map(|pair| {
            let [a, b] = [pair[0], pair[1]];
            if a.position == b.position
                || position < a.position.min(b.position)
                || position > a.position.max(b.position)
            {
                return None;
            }
            let fraction = (position - a.position) / (b.position - a.position);
            Some(Self {
                position,
                speed: a.speed + fraction * (b.speed - a.speed),
                current: a.current + fraction * (b.current - a.current),
            })
        })
    }
}

/// What the gains stage settled on for one joint.
#[derive(Clone, Copy, Debug)]
struct Tuned {
    before: Gains,
    after: Gains,
    observations: usize,
}

// The StepFOC guide supplied by the user fixes the order (Kpv, Kiv, Kpp) and
// Kpv's 20% backoff. Everything else here makes that manual procedure
// finite, start-independent and repeatable: fixed lattices to walk, a bound
// on observations, and the motions each verdict is read from.
const GAIN_OBSERVATIONS: usize = 40;
const GAIN_DIVISOR: u8 = 6;
const GAIN_VELOCITY_RAD_S: f64 = 40.0 * std::f64::consts::PI / 180.0;
const GAIN_SETTLE_S: f64 = 0.3;
/// The pulse: a cosine ramp sized to the joint's EXEC acceleration within
/// these bounds, a steady stretch, the same ramp down, then the stop the
/// standstill verdict is read from. A velocity step instead put J3 on its
/// current limit at the second Kpv point, which says nothing about its loop.
const GAIN_RAMP_MIN_S: f64 = 0.1;
const GAIN_RAMP_MAX_S: f64 = 0.5;
const GAIN_STEADY_S: f64 = 0.35;
/// How far into the steady stretch and into the stop the verdicts begin.
const GAIN_RAMP_SETTLE_S: f64 = 0.1;
const GAIN_TAIL_SETTLE_S: f64 = 0.25;
/// The guide's 20% Kpv backoff is one step of its lattice: the result stays
/// a lattice point, so the next run snaps straight back onto it.
const KPV_BACKOFF_STEPS: i32 = 1;
/// A position step may overshoot by two encoder counts or this share of the
/// step, whichever is larger. The share is the guide's "overshoot appears":
/// a few counts past a 1 deg step is encoder noise and stiction release
/// (J1 read 3 counts one run and 4 the next at the same gains, 2026-10-02),
/// and a Kpp verdict must not turn on that. Whether the chosen Kpp also
/// holds still at the arm's inertia extremes is the posture check's call.
const GAIN_OVERSHOOT_FRACTION: f64 = 0.05;
/// Qualification steps a gain down its lattice at most this many times.
const GAIN_QUALIFY_STEPS: usize = 3;
/// Oscillation on the normal profile: the speed error crossing from beyond
/// one side of this band to beyond the other `SWEEP_REVERSALS` times. Lag
/// crosses it once each way; a loop hunting at tens of hertz crosses it
/// every half cycle (J4 at 55 Hz, 2026-10-01).
const SWEEP_BAND_RAD_S: f64 = 3.0 * MOVING_RMS_RAD_S;
const SWEEP_REVERSALS: u32 = 6;
/// A candidate that misbehaves on a calibration pose steps its Kpv down this
/// many lattice points before its configured gains are retained instead.
const POSE_BACKOFF_STEPS: u8 = 2;
/// Passes over the calibration poses before the qualification is given up.
const POSE_ATTEMPTS: usize = 8;
/// How long the CAN transmit queue may stay full before the bus is given up.
const TX_FULL_S: f64 = 1.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum GainAxis {
    Kpv,
    Kiv,
    Kpp,
}

impl GainAxis {
    fn value(self, g: Gains) -> f64 {
        match self {
            Self::Kpv => g.kpv,
            Self::Kiv => g.kiv,
            Self::Kpp => g.kpp,
        }
    }
    fn set(self, g: &mut Gains, value: f64) {
        let value = f64::from(value as f32);
        match self {
            Self::Kpv => {
                // The integral corner (Kiv/Kpv) belongs to the Kiv search; a Kpv
                // step keeps it so the loop's shape survives the walk. With Kiv
                // left at the config's value, J3 hunted harder at every step
                // down (2026-10-01).
                let ratio = if g.kpv > 0.0 { g.kiv / g.kpv } else { 0.0 };
                g.kpv = value;
                g.kiv = f64::from((ratio * value) as f32);
            }
            Self::Kiv => g.kiv = value,
            Self::Kpp => g.kpp = value,
        }
    }
    fn label(self) -> &'static str {
        match self {
            Self::Kpv => "Kpv",
            Self::Kiv => "Kiv",
            Self::Kpp => "Kpp",
        }
    }
    /// The lattice this axis is searched on: base, ratio and the highest
    /// index. Anchored to fixed numbers, not to the configured value, so
    /// two runs from different configs walk the same points and can agree.
    /// Kpp stops at 20: the drive's position loop closes on a 250 Hz target
    /// stream, and above a few hertz it amplifies that staircase.
    fn lattice(self) -> (f64, f64, i32) {
        match self {
            Self::Kpv => (0.001, 1.25, 21),
            Self::Kiv => (0.0001, 1.5, 10),
            Self::Kpp => (2.5, 2.0, 3),
        }
    }
    fn top(self) -> i32 {
        self.lattice().2
    }
    /// Lattice point `index`, as the drive will hold it.
    fn at(self, index: i32) -> f64 {
        let (base, ratio, _) = self.lattice();
        f64::from((base * ratio.powi(index)) as f32)
    }
    /// The lattice point nearest `value` by ratio, within the lattice.
    fn snap(self, value: f64) -> i32 {
        let (base, ratio, top) = self.lattice();
        if !(value.is_finite() && value > 0.0) {
            return 0;
        }
        ((value / base).ln() / ratio.ln())
            .round()
            .clamp(0.0, f64::from(top)) as i32
    }
}

#[derive(Clone, Copy)]
enum GainCommand {
    /// Constant velocity \[ticks/s\] for the whole capture.
    Velocity(f64),
    /// Ramped to `ticks_s` over `ramp_s`, held `GAIN_STEADY_S`, ramped back
    /// to rest, then still for the rest of the capture.
    Pulse { ticks_s: f64, ramp_s: f64 },
    /// A P-only position step \[ticks\], without velocity feedforward.
    Position(i32),
}

impl GainCommand {
    fn divisor(self) -> u8 {
        match self {
            Self::Velocity(_) => GAIN_DIVISOR,
            Self::Pulse { .. } | Self::Position(_) => 12,
        }
    }
}

/// The pulse's velocity profile at `t` seconds, as a fraction of its speed.
fn pulse_shape(t: f64, ramp_s: f64) -> f64 {
    let up = |x: f64| 0.5 * (1.0 - (std::f64::consts::PI * x).cos());
    if t < ramp_s {
        up(t / ramp_s)
    } else if t < ramp_s + GAIN_STEADY_S {
        1.0
    } else if t < 2.0 * ramp_s + GAIN_STEADY_S {
        1.0 - up((t - ramp_s - GAIN_STEADY_S) / ramp_s)
    } else {
        0.0
    }
}

/// How a candidate misbehaved on a calibration pose: on the way, or at it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PoseFault {
    /// Oscillated on a leg: the velocity loop's doing, so Kpv steps down.
    Motion,
    /// Would not hold still at the pose, where the measurements need two
    /// counts of stillness: hunting, so Kpp steps down, then Kiv.
    Hold,
}

/// What one leg of the normal profile said about trial gains.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Sweep {
    Passed,
    /// Oscillated, hunted at the end, or did not land: the gains' doing.
    Failed,
    /// Could not be judged: the budget, or a contact on the way.
    Stopped,
}

impl Sweep {
    fn label(self) -> &'static str {
        match self {
            Self::Passed => "passed",
            Self::Failed => "failed",
            Self::Stopped => "stopped",
        }
    }
}

/// The verdict on one leg: a runaway or an oscillation fails it, as does a
/// joint that will not hold still afterwards or did not land; a contact
/// says nothing about the gains.
fn sweep_verdict(m: &Measure, holding_limit: f64, tolerance: f64) -> Sweep {
    match m.outcome {
        Outcome::Unstable | Outcome::Timeout => Sweep::Failed,
        Outcome::Blocked => Sweep::Stopped,
        Outcome::Complete => {
            if m.reversals >= SWEEP_REVERSALS
                || m.hold_rms_rad_s > holding_limit
                || m.settled_error_rad > tolerance
            {
                Sweep::Failed
            } else {
                Sweep::Passed
            }
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct GainObservation {
    stable: bool,
    ripple: f64,
    reversals: usize,
    overshoot: f64,
    error: f64,
    /// Mean velocity error without removing angle-dependent ripple.
    mean_error: f64,
}

/// RMS about the mean measures ripple. Repeated crossings of the requested
/// speed distinguish oscillation about the target from motion below it.
fn gain_ripple(values: &[f64], reference: f64, per_tick: f64, limit: f64) -> (f64, usize) {
    let mean = values.iter().sum::<f64>() / values.len() as f64;
    let ripple = (values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / values.len() as f64)
        .sqrt()
        * per_tick.abs();
    let mut sign = 0;
    let mut reversals = 0;
    for v in values {
        let velocity = (v - reference) * per_tick;
        let next = if velocity > limit / 2.0 {
            1
        } else if velocity < -limit / 2.0 {
            -1
        } else {
            0
        };
        if next != 0 {
            if sign != 0 && sign != next {
                reversals += 1;
            }
            sign = next;
        }
    }
    (ripple, reversals)
}

/// Separate the ripple fixed to the rotor's electrical angle (the harmonics
/// the ripple stage cancels) from free oscillation while tuning. Tracking
/// error is still judged on the raw motion.
fn gain_moving_residual(speed: &[f64], phase: &[f64]) -> Vec<f64> {
    let mut residual = speed.to_vec();
    let travel = phase
        .windows(2)
        .map(|p| (p[1] - p[0] + 8192.0).rem_euclid(16384.0) - 8192.0)
        .sum::<f64>()
        .abs();
    // A stalled P-only trial cannot identify an angle-dependent disturbance.
    if travel >= 2.0 * 16384.0 {
        if let Some(fit) = ripple::fit(speed, phase, 0, &RIPPLE_HARMONICS) {
            for (v, p) in residual.iter_mut().zip(phase) {
                let angle = std::f64::consts::TAU * p / 16384.0;
                for harmonic in &fit {
                    let x = f64::from(harmonic.harmonic) * angle;
                    *v -= harmonic.a * x.cos() + harmonic.b * x.sin();
                }
            }
        }
    }
    residual
}

struct GainSession {
    joint: usize,
    span: (f64, f64),
    /// The gains every observation restores before anything else moves.
    original: Gains,
}

/// One coordinated approach and hold, observed on all selected joints.
struct GainPose {
    from_ready: bool,
    approach: [f64; N],
    target: [f64; N],
}

/// Waiting out one silent drive is normal; a run that spends its time doing
/// nothing else has a bus problem no amount of waiting will fix.
const MAX_RECOVERIES: u32 = 24;
/// Clear-and-read-back rounds a drive gets at startup before its fault is
/// taken as real.
const CLEAR_ROUNDS: u32 = 3;
/// Candidate poses drawn over each joint's whole window, from which the
/// identification plan keeps the ones that pin the gravity parameters best.
/// How many it keeps is `selfcal.identification_poses`.
const IDENT_CANDIDATES: usize = 300;
/// The share of a joint's current limit a candidate pose may need just to
/// hold itself against gravity: the hold has to leave room for friction and
/// the two approaches. The ready pose needs about a tenth on this arm.
const IDENT_HOLD_FRACTION: f64 = 0.6;

// ---------------------------------------------------------------- CLI

#[derive(clap::Parser)]
#[command(
    name = "par6-selfcal",
    about = "Home a PAR6, measure its mechanics and identify its link masses"
)]
struct Args {
    #[arg(default_value = "config/PAR6.toml")]
    config: PathBuf,
    /// This arm's local overlay: layered over the config, and where
    /// `--apply` writes what was measured (default: `local.toml` beside the
    /// config).
    #[arg(long, value_name = "PATH", env = par6_config::LOCAL_CONFIG_ENV)]
    local_config: Option<PathBuf>,
    #[arg(long, default_value = "calibration-runs")]
    output_dir: PathBuf,
    /// Print the history table for this existing run directory -- its
    /// candidate beside the earlier runs in the same output directory -- and
    /// exit. No arm, no bus.
    #[arg(long, value_name = "RUN_DIR")]
    history: Option<PathBuf>,
    /// Run against the simulator instead of the arm.
    #[arg(long)]
    sim: bool,
    /// Write what was measured into the local overlay.
    #[arg(long)]
    apply: bool,
    /// Also find each joint's velocity, acceleration and jerk limits, and
    /// with `--apply` write them as its EXEC limits. Off by default: it
    /// drives every joint to the edge of what it can do, and what it finds
    /// changes little from arm to arm.
    #[arg(long)]
    limits: bool,
    /// Development: home, then run only these stages (repeatable), in the
    /// order a full calibration runs them. A full calibration runs every
    /// stage but the limits.
    #[arg(long, value_enum)]
    only: Vec<Stage>,
    /// Development, with `--only` ripple and/or gains: run those stages on
    /// only these joints (1-6, repeatable).
    #[arg(long = "joint", value_parser = clap::value_parser!(u8).range(1..=6))]
    joints: Vec<u8>,
    /// The tool fitted on the arm now, by its config name (e.g. `Flange`).
    /// Defaults to the config's `active_tool`; the file is not changed.
    #[arg(long)]
    tool: Option<String>,
    /// With --only gains, verify gains from this TOML without a new search.
    /// Selected joints use the candidate gains together; startup, recovery
    /// and parking use the primary config.
    #[arg(long, conflicts_with = "apply")]
    verify_gains: Option<PathBuf>,
    /// Also verify the candidate file's ripple compensation, installed after
    /// homing and restored to the primary config before parking.
    #[arg(long, requires = "verify_gains")]
    verify_ripple: bool,
}

/// The stages a calibration runs, in order.
#[derive(Clone, Copy, Debug, PartialEq, Eq, clap::ValueEnum)]
enum Stage {
    /// Ripple feedforward: cogging and commutation error, cancelled in the drive.
    Ripple,
    /// StepFOC velocity PI followed by position P tuning and final verification.
    Gains,
    /// Static friction: breakaway at the ready pose and with the arm out.
    Stiction,
    /// J1's current chirp: the arm's compliance about the base axis.
    Belt,
    /// Friction per joint and the gravity identification sweep.
    Mechanics,
    /// Each joint's EXEC limits (opt-in with `--limits` in a full run).
    Limits,
}

// ---------------------------------------------------------------- events

#[derive(Clone, Debug)]
#[allow(clippy::large_enum_variant)] // Tick events stay inline, without allocation.
enum Event {
    Phase(&'static str, usize),
    Tool(u8, bool),
    Configure(u64, usize, f64, i32),
    Hold(u64, usize, &'static str, f64, f64),
    Move(u64, usize, Outcome, f64, i32, i32, Measure),
    Contact(u64, usize, i64, f64, f64, bool, bool),
    FeedbackGap(u64, usize, f64),
    /// The arm moved again after standing still longer than `IDLE_LIMIT_S`:
    /// for how long \[s\], during which announced activity on which joint.
    Idle(u64, f64, &'static str, usize),
    Drag(u64, usize, f64, f64, f64),
    Mechanics(u64, usize, f64, f64, f64),
    FrictionQuality(u64, usize, f64, f64, f64, f64, usize),
    /// Breakaway at a labelled pose: the currents up and down \[mA\], the
    /// static friction they straddle \[Nm\], the gravity current the model
    /// predicted against the one the pair measured \[mA\], the transmission
    /// wind-up before the link moved \[motor ticks\] and the stiffness that
    /// implies \[Nm/rad, joint side\].
    Stiction(u64, usize, &'static str, f64, f64, f64, f64, f64, f64, f64),
    /// The belt chirp's outcome: the rotor's swing and drift over it
    /// \[deg\], and whether the travel bound stopped it.
    Belt(u64, usize, f64, f64, bool),
    LimitsStep(u64, usize, Caps, Option<&'static str>),
    Limits(u64, usize, Found),
    GainObservation(u64, usize, Gains, &'static str, GainObservation),
    GainPose(u64, usize, usize, [f64; N]),
    /// One joint on a coordinated qualification move: speed RMS \[rad/s\],
    /// peak position error \[rad\] and speed-error reversals across the
    /// oscillation band.
    GainPoseMotion(u64, usize, f64, f64, u32),
    /// One leg of the normal profile under trial gains: speed-error
    /// reversals across the oscillation band, the hold RMS after it
    /// \[rad/s\], the lag \[ms\] and the verdict.
    Sweep(u64, usize, Gains, u32, f64, f64, &'static str),
    /// Why a joint keeps its configured gains.
    GainsNote(u64, usize, &'static str),
    Gains(u64, usize, Tuned),
    /// The ripple feedforward fitted from the sweeps: per harmonic, the
    /// harmonic and its cosine and sine current \[mA\].
    RippleFit(u64, usize, [(u8, i16, i16); 6]),
    /// The at-speed refinement: speed ripple on the step with the slow-sweep
    /// feedforward and with the refined one \[ticks/s\].
    RippleRefine(u64, usize, f64, f64),
    /// The speed ripple at those harmonics on the gains step, without and
    /// with the feedforward \[ticks/s\], and whether it stayed.
    RippleCheck(u64, usize, f64, f64, bool),
    /// Why a joint gets no ripple feedforward.
    RippleNote(u64, usize, &'static str),
    IdentPose(usize, usize, [f64; N]),
    IdentTorque(usize, [f64; N]),
    ArmFit(f64, f64),
    Capture(usize, u32, f64, Gains, Captured, Option<f64>),
    Sample(
        u64,
        [JointCommand; N],
        [i32; N],
        [i32; N],
        [i32; N],
        [bool; N],
        [u64; N],
    ),
}

/// `None` means the event belongs in a csv, not the console.
fn describe(event: &Event) -> Option<String> {
    Some(match *event {
        Event::Sample(..) | Event::Capture(..) => return None,
        Event::Phase(name, j) => format!("J{}: {name}", j + 1),
        Event::Tool(node, found) => format!(
            "TOOL node {node}: {}",
            if found {
                "a gripper driver answered"
            } else {
                "no gripper driver answered"
            }
        ),
        Event::Configure(tick, j, ilim, at) => {
            format!(
                "tick={tick} J{} CONFIGURE limit={ilim:.0}mA encoder={at}",
                j + 1
            )
        }
        Event::Hold(tick, j, why, position_rms, speed_rms) => format!(
            "tick={tick} J{} HOLD {why} position_rms={:.5}deg speed_rms={:.5}deg/s",
            j + 1,
            position_rms.to_degrees(),
            speed_rms.to_degrees()
        ),
        Event::Move(tick, j, outcome, elapsed, from, to, m) => format!(
            "tick={tick} J{} MOVE {outcome:?} {elapsed:.3}s {from}->{to} \
             position_rms={:.5}deg peak_error={:.5}deg settled={:.5}deg \
             speed_rms={:.5}deg/s peak_command={:.5}deg/s excess={:.4}",
            j + 1,
            m.rms_position().to_degrees(),
            m.peak_error_rad.to_degrees(),
            m.settled_error_rad.to_degrees(),
            m.rms_speed().to_degrees(),
            m.peak_command_rad_s.to_degrees(),
            m.excess
        ),
        Event::Contact(tick, j, range, requested, below, stopped, loaded) => format!(
            "tick={tick} J{} DETECT range={range}ticks requested={requested:.0}ticks \
             stall_below={below:.0}ticks stopped={stopped} loaded={loaded}",
            j + 1
        ),
        Event::Idle(tick, idle, name, j) => format!(
            "tick={tick} IDLE {idle:.1}s: the arm stood still longer than {IDLE_LIMIT_S}s during J{} {name}",
            j + 1
        ),
        Event::FeedbackGap(tick, j, silent) => {
            format!(
                "tick={tick} J{} answered again after {silent:.3}s of silence",
                j + 1
            )
        }
        Event::Drag(tick, j, speed, forward, reverse) => format!(
            "tick={tick} J{} DRAG {speed:.4}rad/s forward={forward:.1}mA reverse={reverse:.1}mA",
            j + 1
        ),
        Event::Mechanics(tick, j, inertia, b, tc) => format!(
            "tick={tick} J{} MECHANICS J={inertia:.6}kg.m2 b={b:.6}Nm.s tc={tc:.4}Nm",
            j + 1
        ),
        Event::FrictionQuality(tick, j, b, tc, b_se, tc_se, speeds) => format!(
            "tick={tick} J{} FRICTION FIT speeds={speeds} b={b:.6} +/- {b_se:.6}Nm.s tc={tc:.6} +/- {tc_se:.6}Nm (one standard error; excludes systematic bias)",
            j + 1
        ),
        Event::Belt(tick, j, swing, drift, aborted) => format!(
            "tick={tick} J{} BELT chirp={BELT_CHIRP_NM:.2}Nm swing={swing:.2}deg drift={drift:+.2}deg{}",
            j + 1,
            if aborted { " ABORTED on travel" } else { "" }
        ),
        Event::Stiction(tick, j, label, up, down, s, model, measured, windup, k) => {
            let stiffness = if k.is_finite() { format!("{k:.0}Nm/rad") } else {
                "unresolved (windup below encoder resolution)".to_owned()
            };
            format!(
                "tick={tick} J{} STICTION {label} up={up:.0}mA down={down:.0}mA static={s:.4}Nm \
                 gravity model={model:.0}mA measured={measured:.0}mA windup={windup:.1}ticks \
                 stiffness={stiffness}", j + 1
            )
        },
        Event::LimitsStep(tick, j, c, failed) => format!(
            "tick={tick} J{} LIMITS step velocity={:.4}rad/s acceleration={:.4}rad/s2 \
             jerk={:.4}rad/s3 {}",
            j + 1,
            c.velocity,
            c.acceleration,
            c.jerk,
            failed.map_or("passed".to_owned(), |why| format!("failed: {why}"))
        ),
        Event::GainObservation(tick, j, g, stage, o) => format!(
            "tick={tick} J{} GAINS {stage} kpv={:.6} kiv={:.8} kpp={:.5} encoder_ripple={:.3}deg/s reversals={} overshoot={:.4}deg error={:.4} mean_error={:.4} stable={}",
            j + 1, g.kpv, g.kiv, g.kpp, o.ripple.to_degrees(), o.reversals,
            o.overshoot.to_degrees(), o.error, o.mean_error, o.stable
        ),
        Event::GainsNote(tick, j, note) => format!("tick={tick} J{} GAINS {note}", j + 1),
        Event::GainPose(tick, at, total, q) => format!(
            "tick={tick} GAINS qualify calibration pose {at}/{total} q={q:?}"
        ),
        Event::GainPoseMotion(tick, j, speed, position, reversals) => format!(
            "tick={tick} J{} GAINS pose motion speed_rms={:.3}deg/s peak_position_error={:.3}deg reversals={reversals}",
            j + 1, speed.to_degrees(), position.to_degrees()
        ),
        Event::Sweep(tick, j, g, reversals, hold, lag_ms, verdict) => format!(
            "tick={tick} J{} GAINS sweep kpv={:.6} kiv={:.8} kpp={:.5} reversals={reversals} hold_rms={:.3}deg/s lag={lag_ms:.1}ms {verdict}",
            j + 1, g.kpv, g.kiv, g.kpp, hold.to_degrees()
        ),
        Event::RippleFit(tick, j, h) => format!(
            "tick={tick} J{} RIPPLE fit {}",
            j + 1,
            h.iter()
                .map(|(n, a, b)| format!("h{n}: {a}/{b}mA"))
                .collect::<Vec<_>>()
                .join(" ")
        ),
        Event::RippleCheck(tick, j, before, after, kept) => format!(
            "tick={tick} J{} RIPPLE speed ripple about the 20 deg/s step {before:.0}->{after:.0}ticks/s {}",
            j + 1,
            if kept { "kept" } else { "no better; cleared" }
        ),
        Event::RippleRefine(tick, j, first, refined) => format!(
            "tick={tick} J{} RIPPLE refined at speed {first:.0}->{refined:.0}ticks/s",
            j + 1
        ),
        Event::RippleNote(tick, j, note) => format!("tick={tick} J{} RIPPLE {note}", j + 1),
        Event::Gains(tick, j, t) => format!(
            "tick={tick} J{} GAINS StepFOC kpv={:.6}->{:.6} kiv={:.8}->{:.8} kpp={:.5}->{:.5} observations={}",
            j + 1, t.before.kpv, t.after.kpv, t.before.kiv, t.after.kiv,
            t.before.kpp, t.after.kpp, t.observations
        ),
        Event::Limits(tick, j, f) => format!(
            "tick={tick} J{} LIMITS velocity={:.4}rad/s{} acceleration={:.4}rad/s2 \
             jerk={:.4}rad/s3{} ripple={:.1}%",
            j + 1,
            f.caps.velocity,
            if f.velocity_reached {
                ""
            } else {
                " (lower bound)"
            },
            f.caps.acceleration,
            f.caps.jerk,
            if f.jerk_measured {
                ""
            } else {
                " (lower bound, not written)"
            },
            100.0 * f.ripple
        ),
        Event::IdentPose(i, total, q) => format!("IDENT pose {i}/{total} q={q:?}"),
        Event::IdentTorque(i, tau) => format!("IDENT pose {i} torque {tau:?} Nm"),
        Event::ArmFit(before, after) => format!("IDENT residual {before:.5} Nm -> {after:.5} Nm"),
    })
}

/// Drains events to `console.log` and `samples.csv` off the control thread.
fn writer(rx: Receiver<Event>, directory: PathBuf) -> std::io::Result<()> {
    let mut console = fs::File::create(directory.join("console.log"))?;
    let mut samples = fs::File::create(directory.join("samples.csv"))?;
    writeln!(
        samples,
        "tick,joint,command,position_ticks,speed_ticks_s,current_ma,drive_fault,position_rx_ns"
    )?;
    let mut line = String::new();
    while let Ok(event) = rx.recv() {
        if let Event::Capture(j, index, step, gains, captured, stop_after) = event {
            if let Err(error) =
                save_capture(&directory, j, index, step, gains, &captured, stop_after)
            {
                writeln!(
                    console,
                    "J{} capture {index} could not be saved: {error}",
                    j + 1
                )?;
            }
            continue;
        }
        if let Event::Sample(tick, cmd, pos, speed, current, fault, received) = event {
            line.clear();
            for j in 0..N {
                let _ = writeln!(
                    line,
                    "{tick},{},\"{:?}\",{},{},{},{},{}",
                    j + 1,
                    cmd[j],
                    pos[j],
                    speed[j],
                    current[j],
                    u8::from(fault[j]),
                    received[j]
                );
            }
            samples.write_all(line.as_bytes())?;
            continue;
        }
        if let Some(text) = describe(&event) {
            println!("{text}");
            writeln!(console, "{text}")?;
        }
    }
    Ok(())
}

fn save_capture(
    directory: &Path,
    j: usize,
    index: u32,
    step: f64,
    tried: Gains,
    captured: &Captured,
    stop_after: Option<f64>,
) -> std::io::Result<()> {
    let divisor = captured.divisor;
    let path = directory.join(format!("capture-J{}-{index}.csv", j + 1));
    let stop = stop_after.map_or_else(|| "none".to_owned(), |s| s.to_string());
    let mut text = format!(
        "# step_ticks_s={step:.0} divisor={divisor} stop_after_s={stop} gains kpv={} kiv={} kpp={}\nsample,t_s,speed_ticks_s,iq_ma,phase\n",
        tried.kpv, tried.kiv, tried.kpp
    );
    for (k, (v, i)) in captured.speed.iter().zip(&captured.current).enumerate() {
        let t = (k * divisor) as f64 / LOOP_HZ;
        let phase = captured
            .phase
            .get(k)
            .map_or(String::new(), |p| format!("{p:.0}"));
        let _ = writeln!(text, "{k},{t:.6},{v:.0},{i:.0},{phase}");
    }
    fs::write(path, text)
}

// ---------------------------------------------------------------- measurement

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum Outcome {
    #[default]
    Timeout,
    Complete,
    /// The joint stopped while loaded: a mechanical stop.
    Blocked,
    /// The loop oscillated under trial gains; the move stopped under the
    /// original gains.
    Unstable,
}

#[derive(Clone, Copy, Debug, Default)]
struct Measure {
    samples: u32,
    position_sq: f64,
    speed_sq: f64,
    peak_error_rad: f64,
    peak_command_rad_s: f64,
    /// Largest current the move drew beyond what gravity alone needs at the
    /// measured pose \[mA\]: what accelerating the joint and its friction cost.
    peak_dynamic_ma: f64,
    /// Times the speed error crossed from beyond one side of
    /// `SWEEP_BAND_RAD_S` to beyond the other.
    reversals: u32,
    settled_error_rad: f64,
    hold_rms_rad_s: f64,
    /// How far this move missed the acceptance limits, as a fraction of them.
    /// Zero passes. One scale, no offset: an earlier design carried
    /// `1.0 + violation` in one place and the raw value in another, and a
    /// threshold comparing against the wrong one silently never fired.
    excess: f64,
    elapsed: f64,
    outcome: Outcome,
}

impl Measure {
    fn outcome_complete(&mut self) {
        self.outcome = Outcome::Complete;
    }
    fn rms_position(&self) -> f64 {
        (self.position_sq / f64::from(self.samples.max(1))).sqrt()
    }
    fn rms_speed(&self) -> f64 {
        (self.speed_sq / f64::from(self.samples.max(1))).sqrt()
    }
    /// A position move is regulated by the position loop, so how far the joint
    /// runs behind its own commanded position IS the criterion there, not a
    /// diagnostic: on 2026-09-21 J4 carried 1.19 deg of following error at
    /// 97 deg/s, four times what a healthy joint carries per unit speed, and
    /// passed on speed RMS alone.
    fn score(&mut self, positional: bool, moving_limit: f64, holding_limit: f64, tolerance: f64) {
        let over = |value: f64, limit: f64| {
            if limit > 0.0 {
                value / limit - 1.0
            } else {
                0.0
            }
        };
        let mut excess = over(self.rms_speed(), moving_limit);
        excess = excess.max(over(self.hold_rms_rad_s, holding_limit));
        if positional {
            excess = excess.max(over(
                self.peak_error_rad,
                FOLLOWING_S * self.peak_command_rad_s,
            ));
            excess = excess.max(over(self.settled_error_rad, tolerance));
        }
        self.excess = excess.max(0.0);
    }
}

/// Whether a drive's error register names an actual fault.
///
/// `calibrated` and `activated` are status, not faults. The frame's own error
/// bit is deliberately not consulted: this arm's J1 raises it with every
/// register flag clear, normal bus voltage and calibration intact, then goes
/// quiet for ten seconds and comes back -- a transient, and the register is
/// what tells the two apart.
///
/// `ErrorFlags::faults` is that register, and the one table the RT core and
/// the server also read, so a new bit cannot reach only one of them.
fn reported_fault(flags: &ErrorFlags) -> bool {
    flags.faults().next().is_some()
}
/// Encoder-difference speed estimate over a fixed window.
#[derive(Clone, Copy, Default)]
struct Ring {
    samples: [(u64, i32, u64); 8],
    len: usize,
}
impl Ring {
    /// Push a fresh sample; return counts/second measured back to the newest
    /// sample that is at least `window` ticks old, once there is one.
    fn push(&mut self, tick: u64, position: i32, window: u64, dt: f64) -> Option<f64> {
        self.push_at(tick, position, window, (tick as f64 * dt * 1e9) as u64)
    }

    fn push_at(&mut self, tick: u64, position: i32, window: u64, received_ns: u64) -> Option<f64> {
        let mut speed = None;
        for k in 0..self.len {
            let (then, was, received_then) = self.samples[k];
            if tick.saturating_sub(then) >= window {
                speed = received_ns
                    .checked_sub(received_then)
                    .filter(|dt| *dt > 0)
                    .map(|dt| (f64::from(position) - f64::from(was)) / (dt as f64 * 1e-9));
            }
        }
        if self.len < self.samples.len() {
            self.samples[self.len] = (tick, position, received_ns);
            self.len += 1;
        } else {
            self.samples.rotate_left(1);
            self.samples[self.len - 1] = (tick, position, received_ns);
        }
        speed
    }
}

/// The bus state exposes positions but not their receive timestamps. A
/// passive socket supplies the timestamp of the identical motion reply;
/// four replies per joint cover a drain crossing into the next CAN frame.
struct EncoderClock {
    socket: CanSocket,
    nodes: [u8; N],
    replies: [[(i32, i32, u64); 4]; N],
}

impl EncoderClock {
    fn open(robot: &par6_config::RobotConfig) -> Result<Self> {
        let socket = CanSocket::open(&robot.bus.interface)?;
        socket.set_nonblocking(true)?;
        socket.set_recv_timestamp(true)?;
        let nodes = std::array::from_fn(|j| robot.joints[j].node_id);
        let filters: [(u32, u32); N * 2] = std::array::from_fn(|k| {
            let command = if k % 2 == 0 { 3 } else { 28 };
            ((u32::from(nodes[k / 2]) << 7) | (command << 1), 0xc000_07fe)
        });
        socket.set_filters(&filters)?;
        Ok(Self {
            socket,
            nodes,
            replies: [[(0, 0, 0); 4]; N],
        })
    }

    fn drain(&mut self) -> Result<()> {
        use par6_bus::spectral::codec::{unpack_i24, unpack_i32};
        for _ in 0..128 {
            let (frame, stamp) = match self.socket.read_frame_with_timestamps() {
                Ok(frame) => frame,
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => return Ok(()),
                Err(e) => return Err(e.into()),
            };
            let data = frame.data();
            let Some(j) = self
                .nodes
                .iter()
                .position(|n| u32::from(*n) == frame.raw_id() >> 7)
            else {
                continue;
            };
            if data.len() != 8 {
                continue;
            }
            let ns = stamp
                .socket
                .ok_or("motion reply has no receive timestamp")?
                .duration_since(std::time::SystemTime::UNIX_EPOCH)?
                .as_nanos() as u64;
            self.replies[j].rotate_left(1);
            self.replies[j][3] = if (frame.raw_id() >> 1) & 0x3f == 28 {
                (
                    unpack_i32([data[0], data[1], data[2], data[3]]),
                    unpack_i32([data[4], data[5], data[6], data[7]]),
                    ns,
                )
            } else {
                (
                    unpack_i24([data[0], data[1], data[2]]),
                    unpack_i24([data[3], data[4], data[5]]),
                    ns,
                )
            };
        }
        Err("encoder timestamp socket backlog exceeded its drain bound".into())
    }

    fn received(&self, j: usize, position: i32, speed: i32) -> u64 {
        self.replies[j]
            .iter()
            .rev()
            .find(|r| (r.0, r.1) == (position, speed))
            .map_or(0, |r| r.2)
    }
}

// ---------------------------------------------------------------- the arm

struct Arm {
    bus: RuntimeBus,
    state: BusState,
    bundle: ConfigBundle,
    kin: par6_kin::Kin,
    conv: [JointConversion; N],
    events: Option<SyncSender<Event>>,
    gains: [Gains; N],
    hold: [i32; N],
    homed: [bool; N],
    /// Where each joint was when the run first held it; `None` before then.
    found_at: [Option<i32>; N],
    generation: [u64; N],
    seen: [u64; N],
    encoder_clock: Option<EncoderClock>,
    position_rx_ns: [u64; N],
    /// Positive point estimates can still be dominated by fit uncertainty.
    friction_fit_uncertain: [bool; N],
    /// The endstop this joint last touched: the one homing referenced it
    /// against, or the one a move stopped on. Samples within
    /// `ENDSTOP_EXCLUSION_RAD` of it are not scored.
    endstop_guard: [Option<i32>; N],
    /// While the gains stage trials a joint: the last gains that tracked,
    /// restored within a tick if the trial runs away.
    sane_gains: [Option<Gains>; N],
    /// Observations spent on each joint, every part of the gains work included.
    gain_used: [usize; N],
    gain_watch: bool,
    gain_joint: Option<usize>,
    held_runaway: [u32; N],
    /// Fresh-feedback guard shared by synchronized moves, sweeps and holds.
    tracking_runaway: [u32; N],
    /// The joint that guard last fired on, so the posture qualification can
    /// tell a candidate's fault from a bus fault.
    runaway_joint: Option<usize>,
    /// Consecutive ticks the CAN transmit queue refused the frames.
    tx_full: u64,
    /// What the run last announced, and how long each announcement held the
    /// arm still during the current stop, so the idle report names what the
    /// stop was spent on rather than what ended it.
    activity: std::cell::Cell<(&'static str, usize)>,
    idle_spans: Vec<((&'static str, usize), u64)>,
    /// The idle rule's state: the count each joint rested at when the arm
    /// last stopped, the tick it stopped, and the tally of stops longer than
    /// `IDLE_LIMIT_S`: how many, their total \[s\], and the longest \[s\]
    /// with what the run was doing.
    rest: [Option<i32>; N],
    rest_since: u64,
    idle_stops: u32,
    idle_total_s: f64,
    idle_longest: (f64, &'static str, usize),
    /// The pose homing leaves the arm in, once planned.
    ready: Option<[f64; N]>,
    /// Every coordinated pose reached since the arm was last at ready, in
    /// order: the legs between them were collision-checked before they were
    /// driven, so driven backwards they are the one known-clear way home.
    visited: Vec<[f64; N]>,
    recoveries: u32,
    /// Homing drives unreferenced joints toward the stop at the configured
    /// homing current, not the operating one.
    homing: bool,
    /// Shutdown: watch only this joint, and tolerate silence.
    only: Option<usize>,
    /// Where the gains stage leaves each capture as a CSV.
    run_directory: Option<PathBuf>,
    captures_written: u32,
    blind: bool,
    stopping: bool,
    /// A gain restore is on the drive's frames: the motion guard, which
    /// restores gains itself, stands aside so it cannot recurse.
    restoring: bool,
    tick: u64,
    dt: f64,
    deadline: Duration,
    simulated: bool,
}

impl Arm {
    fn open(
        bundle: ConfigBundle,
        assets: &Path,
        simulated: bool,
        events: SyncSender<Event>,
    ) -> Result<Self> {
        let tool = bundle.active_tool();
        let mut bus: RuntimeBus = if simulated {
            RuntimeBus::from(SimBus::new(Scene {
                assets: assets.to_owned(),
                tool: tool
                    .and_then(|g| g.urdf_variant.as_deref())
                    .and_then(Tool::from_urdf_variant)
                    .unwrap_or(Tool::Flange),
            }))
        } else {
            RuntimeBus::from(SocketCanBus::open(&bundle.robot.bus)?)
        };
        let kin = arm_kin(&bundle, assets)?;
        // Begin holding with a zero current cap, then raise it in the paced
        // startup ramp. STEPFOC IN_LIMITS sets Iq_current_limit without
        // touching the gains.
        let mut boot = bundle.robot.clone();
        for joint in &mut boot.joints {
            joint.ilim_ma = 0.0;
        }
        bus.boot_configure(&boot, tool, bundle.robot.bus.boot_config_repeats)?;
        for j in &bundle.robot.joints {
            bus.send_clear_error(j.node_id, 3)?;
        }
        let robot = &bundle.robot;
        Ok(Self {
            conv: std::array::from_fn(|j| JointConversion::from_config(&robot.joints[j])),
            gains: std::array::from_fn(|j| robot.joints[j].gains),
            dt: robot.robot.tick_dt_s,
            bundle,
            bus,
            kin,
            state: BusState::new(),
            events: Some(events),
            hold: [0; N],
            homed: [false; N],
            found_at: [None; N],
            generation: [0; N],
            seen: [0; N],
            encoder_clock: None,
            position_rx_ns: [0; N],
            friction_fit_uncertain: [false; N],
            endstop_guard: [None; N],
            sane_gains: [None; N],
            gain_used: [0; N],
            gain_watch: false,
            gain_joint: None,
            held_runaway: [0; N],
            tracking_runaway: [0; N],
            runaway_joint: None,
            activity: std::cell::Cell::new(("startup", 0)),
            idle_spans: Vec::new(),
            rest: [None; N],
            rest_since: 0,
            idle_stops: 0,
            idle_total_s: 0.0,
            idle_longest: (0.0, "startup", 0),
            tx_full: 0,
            ready: None,
            visited: Vec::new(),
            recoveries: 0,
            homing: false,
            only: None,
            run_directory: None,
            captures_written: 0,
            blind: false,
            stopping: false,
            restoring: false,
            tick: 0,
            deadline: Duration::ZERO,
            simulated,
        })
    }

    /// Recording is best effort. It must never fail a motion or a park: a
    /// full disk used to error every `emit`, which failed every park and
    /// then released a loaded arm anyway.
    fn emit(&self, event: Event) {
        if let Event::Phase(name, j) = &event {
            self.activity.set((name, *j));
        }
        if let Some(tx) = &self.events {
            let _ = tx.try_send(event);
        }
    }
    fn ticks(&self, seconds: f64) -> u64 {
        (seconds / self.dt).round().max(1.0) as u64
    }
    fn node(&self, j: usize) -> usize {
        self.bundle.robot.joints[j].node_id as usize
    }
    fn pos(&self, j: usize) -> Result<i32> {
        self.state.nodes[self.node(j)]
            .position_ticks
            .ok_or_else(|| format!("J{} missing position", j + 1).into())
    }
    /// Signed joint radians per motor count.
    fn per_tick(&self, j: usize) -> f64 {
        self.conv[j].joint_rad(1) - self.conv[j].joint_rad(0)
    }
    fn angles(&self) -> Result<[f64; N]> {
        let mut q = [0.0; N];
        for (j, v) in q.iter_mut().enumerate() {
            *v = self.conv[j].joint_rad(self.pos(j)?);
        }
        Ok(q)
    }
    /// A still joint resting on a count boundary reads as a full count per
    /// tick, so one scalar limit asks something different of every joint: one
    /// count per tick is 0.220 deg/s on J2 but 1.373 on J4 and J5, where the
    /// configured 1 deg/s is unsatisfiable. Floor at the joint's resolution.
    fn holding_limit(&self, j: usize) -> f64 {
        HOLDING_RMS_RAD_S.max(self.per_tick(j).abs() / self.dt)
    }
    fn tolerance(&self) -> f64 {
        self.bundle.robot.motion.settle_tolerance_rad
    }

    /// One control tick: judge the drives, pace, drain, send.
    fn exchange(&mut self, commands: [JointCommand; N], check: bool) -> Result<()> {
        if !self.stopping && CANCEL.load(Ordering::Relaxed) {
            return Err("cancelled".into());
        }
        // Judge on what the last tick's drain showed, before this one begins:
        // recovery runs whole ticks of its own, and starting those halfway
        // through a tick would send this joint twice in it.
        if check && !self.blind {
            let timeout = self.ticks(FEEDBACK_TIMEOUT_S);
            let mut faulted = None;
            let mut silent = None;
            for j in 0..N {
                if self.only.is_some_and(|watched| watched != j) {
                    continue;
                }
                let node = &self.state.nodes[self.node(j)];
                if node.error_flags.as_ref().is_some_and(reported_fault) {
                    faulted = Some((j, node.error_flags));
                    break;
                }
                if silent.is_none()
                    && (node.live_error_bit || self.tick.saturating_sub(self.seen[j]) > timeout)
                {
                    silent = Some(j);
                }
            }
            if let Some((j, flags)) = faulted {
                return Err(format!("J{} reported a drive fault: {flags:?}", j + 1).into());
            }
            if let Some(j) = silent {
                self.recover(j)?;
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
        if let Some(clock) = &mut self.encoder_clock {
            clock.drain()?;
            for j in 0..N {
                let node = &self.state.nodes[usize::from(self.bundle.robot.joints[j].node_id)];
                self.position_rx_ns[j] = match (node.position_ticks, node.speed_ticks_s) {
                    (Some(p), Some(v)) => clock.received(j, p, v),
                    _ => 0,
                };
            }
        }
        for (j, command) in commands.iter().enumerate() {
            let node = &self.state.nodes[self.node(j)];
            let guarded = check
                && !self.blind
                && !self.restoring
                && !self.homing
                && self.homed[j]
                // Gain trials have their own guard and bounded backoff path.
                && !(self.gain_joint == Some(j) && self.sane_gains[j].is_some())
                && command.vel.is_some();
            if !guarded {
                self.tracking_runaway[j] = 0;
            }
            if node.position_generation != self.generation[j] {
                self.generation[j] = node.position_generation;
                self.seen[j] = self.tick;
                if guarded {
                    let position = node
                        .position_ticks
                        .ok_or("missing motion position feedback")?;
                    let speed = f64::from(
                        node.speed_ticks_s
                            .ok_or("missing motion velocity feedback")?,
                    );
                    let correction = command.pos.map_or(0.0, |target| {
                        (f64::from(target) - f64::from(position)) * self.gains[j].kpp
                    });
                    let limit = self.bundle.robot.joints[j].velocity_limit_ticks_s;
                    let expected =
                        (f64::from(command.vel.unwrap_or(0)) + correction).clamp(-limit, limit);
                    let error = (speed - expected) * self.per_tick(j);
                    self.tracking_runaway[j] = if error.abs() > RUNAWAY_RAD_S {
                        self.tracking_runaway[j] + 1
                    } else {
                        0
                    };
                    if self.tracking_runaway[j] >= RUNAWAY_TICKS {
                        // A synchronized move keeps `hold` at its start until
                        // arrival. Recovery must catch here, not return there.
                        for joint in 0..N {
                            self.adopt(joint)?;
                        }
                        self.runaway_joint = Some(j);
                        self.emit(Event::GainsNote(self.tick, j, "motion or hold runaway; restoring configured gains and aborting calibration"));
                        let restored = self.gain_restore(j, self.bundle.robot.joints[j].gains);
                        return Err(format!("J{} motion/hold speed error {:.1}deg/s exceeded the runaway guard; gain restore: {}", j + 1, error.to_degrees(), if restored.is_ok() { "completed" } else { "failed" }).into());
                    }
                }
            }
        }
        self.watch_idle();
        // A transmit queue that fills is a bus that stopped carrying frames
        // for a moment -- on 2026-10-01 for long enough to end a run and fail
        // its parking. The drives hold their last target meanwhile, so the
        // tick is skipped and the next one tries again, up to `TX_FULL_S`.
        let sent = self
            .bus
            .poll_step()
            .and_then(|()| self.bus.send_joint_commands(&commands))
            .and_then(|()| self.bus.send_gripper(&GripperCommand::NoGripper));
        match sent {
            Ok(()) => self.tx_full = 0,
            Err(BusError::TxQueueFull) => {
                self.tx_full += 1;
                if self.tx_full == 1 {
                    self.emit(Event::Phase(
                        "transmit queue full; holding until it drains",
                        0,
                    ));
                }
                if self.tx_full > self.ticks(TX_FULL_S) {
                    return Err(format!(
                        "the CAN transmit queue stayed full for {TX_FULL_S} s; the bus is not \
                         carrying frames"
                    )
                    .into());
                }
            }
            Err(error) => return Err(error.into()),
        }
        self.emit(Event::Sample(
            self.tick,
            commands,
            std::array::from_fn(|j| self.state.nodes[self.node(j)].position_ticks.unwrap_or(0)),
            std::array::from_fn(|j| self.state.nodes[self.node(j)].speed_ticks_s.unwrap_or(0)),
            std::array::from_fn(|j| {
                self.state.nodes[self.node(j)]
                    .current_ma
                    .map_or(0, i32::from)
            }),
            // Distinguishes "the drive reported a fault" from "the drive did
            // not answer": on 2026-09-19 only the second was recorded and the
            // two could not be told apart afterwards.
            std::array::from_fn(|j| self.state.nodes[self.node(j)].live_error_bit),
            self.position_rx_ns,
        ));
        Ok(())
    }

    /// The idle rule: note where the arm came to rest, and when any joint
    /// leaves that rest report a stop longer than `IDLE_LIMIT_S` with what
    /// the run was doing at the time.
    fn watch_idle(&mut self) {
        let positions: [Option<i32>; N] =
            std::array::from_fn(|j| self.state.nodes[self.node(j)].position_ticks);
        let moved = (0..N).any(|j| match (positions[j], self.rest[j]) {
            (Some(position), Some(rest)) => {
                (i64::from(position) - i64::from(rest)).abs() > IDLE_MOTION_TICKS
            }
            _ => false,
        });
        let current = self.activity.get();
        match self.idle_spans.last_mut() {
            Some((label, ticks)) if *label == current => *ticks += 1,
            _ => self.idle_spans.push((current, 1)),
        }
        if moved {
            let idle = self.tick.saturating_sub(self.rest_since) as f64 * self.dt;
            if idle > IDLE_LIMIT_S {
                let (name, j) = self
                    .idle_spans
                    .iter()
                    .max_by_key(|(_, ticks)| *ticks)
                    .map_or(current, |(label, _)| *label);
                self.idle_stops += 1;
                self.idle_total_s += idle;
                if idle > self.idle_longest.0 {
                    self.idle_longest = (idle, name, j);
                }
                self.emit(Event::Idle(self.tick, idle, name, j));
            }
        }
        if moved || self.rest.iter().any(Option::is_none) {
            self.rest = positions;
            self.rest_since = self.tick;
            self.idle_spans.clear();
        }
    }

    /// Hold position and wait out a drive that has stopped answering.
    ///
    /// A STEPFOC answers CAN from `loop()` while its current loop runs in a
    /// 6250 Hz timer interrupt that also feeds the watchdog, so a starved
    /// `loop()` goes silent with the motor still controlled and nothing
    /// reporting a fault. On this arm J1 went quiet for 10.2 to 14.7 s across
    /// five runs with no error flag, normal bus voltage and all five other
    /// drives answering every tick; UART on J1 confirmed its telemetry never
    /// gapped by more than 200 ms throughout, so `loop()` was alive the whole
    /// time. Silence is therefore something to wait out, and only a drive that
    /// never comes back is a fault.
    fn recover(&mut self, j: usize) -> Result<()> {
        self.recoveries += 1;
        if self.recoveries > MAX_RECOVERIES {
            return Err(format!(
                "drives stopped answering {} times in one run; the bus is not healthy",
                self.recoveries
            )
            .into());
        }
        let stop: [i32; N] = std::array::from_fn(|k| self.pos(k).unwrap_or(self.hold[k]));
        let started = self.tick;
        self.emit(Event::Phase("silent drive: holding until it answers", j));
        // The round robin reaches one node's error register every ~84 ms;
        // waiting on a drive is exactly when that reading cannot wait.
        let node = self.bundle.robot.joints[j].node_id;
        self.bus.queue_poll_override(
            PollAction::Poll {
                node,
                kind: PollKind::Errors,
            },
            4,
        );
        let blind = std::mem::replace(&mut self.blind, true);
        let timeout = self.ticks(FEEDBACK_TIMEOUT_S);
        let deadline = self.ticks(self.bundle.robot.selfcal.stale_recovery_s);
        // Wait only on the joints being watched. Requiring all six brings
        // back the drop this exists to prevent: during shutdown one
        // permanently silent joint would keep every other joint's recovery
        // from ever succeeding, failing the park of the loaded shoulder.
        let watched = self.only;
        let mut outcome = Ok(false);
        for _ in 0..deadline {
            if let Err(e) = self.exchange(stop.map(|p| JointCommand::position(p, 0, 0)), false) {
                outcome = Err(e);
                break;
            }
            let state = &self.state.nodes[usize::from(node)];
            if state.error_flags.as_ref().is_some_and(reported_fault) {
                outcome =
                    Err(
                        format!("J{} reported a drive fault: {:?}", j + 1, state.error_flags)
                            .into(),
                    );
                break;
            }
            let answering = (0..N)
                .filter(|k| watched.is_none_or(|w| w == *k))
                .all(|k| self.tick.saturating_sub(self.seen[k]) <= timeout);
            if !state.live_error_bit && answering {
                outcome = Ok(true);
                break;
            }
        }
        // Restored on every path, including the error one: leaving it set
        // switched fault detection off for the rest of shutdown.
        self.blind = blind;
        let answered = outcome?;
        let silent = self.tick.saturating_sub(started);
        if !answered {
            return Err(format!(
                "J{} did not come back in {:.1}s of silence",
                j + 1,
                silent as f64 * self.dt
            )
            .into());
        }
        self.emit(Event::FeedbackGap(self.tick, j, silent as f64 * self.dt));
        Ok(())
    }

    /// Hold every joint, optionally overriding one. A hold carries the same
    /// gravity feedforward a move does: a move whose frames fed gravity
    /// forward and whose hold then did not stepped J4 by 78 mA at every
    /// landing, and the position loop took a second to make it up.
    fn frame(&mut self, active: Option<(usize, JointCommand)>) -> Result<()> {
        let generation = self.generation;
        let feedforward = self.gravity_feedforward();
        let mut commands: [JointCommand; N] =
            std::array::from_fn(|j| JointCommand::position(self.hold[j], 0, feedforward[j]));
        if let Some((j, cmd)) = active {
            commands[j] = cmd;
        }
        self.exchange(commands, true)?;
        if self.gain_watch && !self.stopping {
            for (j, previous) in generation.iter().enumerate() {
                if active.is_some_and(|(moving, _)| moving == j) || self.gain_joint == Some(j) {
                    self.held_runaway[j] = 0;
                    continue;
                }
                if *previous == self.generation[j] {
                    continue;
                }
                let speed = f64::from(
                    self.state.nodes[self.node(j)]
                        .speed_ticks_s
                        .ok_or("missing held-joint velocity feedback")?,
                ) * self.per_tick(j);
                self.held_runaway[j] = if speed.abs() > RUNAWAY_RAD_S {
                    self.held_runaway[j] + 1
                } else {
                    0
                };
                if self.held_runaway[j] >= RUNAWAY_TICKS {
                    self.gain_watch = false;
                    self.calm(j, self.bundle.robot.joints[j].gains)?;
                    return Err(
                        format!("J{} ran away while holding during gains tuning", j + 1).into(),
                    );
                }
            }
        }
        Ok(())
    }

    /// The current that balances gravity at the measured pose, per joint
    /// \[mA\]; zero while a pose is unknown.
    fn gravity_feedforward(&mut self) -> [i16; N] {
        let mut tau = [0.0; N];
        let known = self
            .angles()
            .and_then(|q| self.kin.gravity(&q, &mut tau).map_err(Into::into));
        if known.is_err() {
            return [0; N];
        }
        std::array::from_fn(|j| {
            let cfg = &self.bundle.robot.joints[j];
            let ma = tau[j]
                * torque_to_ma_factor(cfg.gear_ratio, cfg.gear_efficiency, cfg.kt_nm_a, cfg.dir);
            ma.clamp(-cfg.ilim_ma, cfg.ilim_ma).round() as i16
        })
    }
    fn settle(&mut self, seconds: f64) -> Result<()> {
        for _ in 0..self.ticks(seconds) {
            self.frame(None)?;
        }
        Ok(())
    }
    fn adopt(&mut self, j: usize) -> Result<()> {
        self.hold[j] = self.pos(j)?;
        Ok(())
    }

    fn retune(&mut self, j: usize, ilim: f64) -> Result<()> {
        let cfg = &self.bundle.robot.joints[j];
        let tune = DriveTune {
            gains: self.gains[j],
            ilim_ma: ilim,
            velocity_limit_ticks_s: cfg.velocity_limit_ticks_s,
            voltage_limit_mv: cfg.voltage_limit_mv,
        };
        let node = cfg.node_id;
        self.bus.retune_node(node, &tune, 0)?;
        self.emit(Event::Configure(self.tick, j, ilim, self.pos(j)?));
        Ok(())
    }
    /// Push gains and limits to one drive and give the frames time to land.
    fn configure(&mut self, j: usize, ilim: f64) -> Result<()> {
        self.retune(j, ilim)?;
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
        // `frame` skips a tick whose frames the transmit queue refused; a
        // queue still full at the end carried none of the last pass, and a
        // configuration that did not reach the drive must not count as done.
        if self.tx_full > 0 {
            return Err(format!(
                "J{} configuration frames were not carried: the transmit queue is full",
                j + 1
            )
            .into());
        }
        Ok(())
    }
    fn operating(&mut self, j: usize) -> Result<()> {
        self.configure(j, self.bundle.robot.joints[j].ilim_ma)
    }

    /// Poll until every drive reports position, speed and current, ramp the
    /// holding current up, check the bus agrees about the fitted tool, and
    /// confirm every joint holds still before anything moves.
    fn initialize(&mut self) -> Result<()> {
        // Clear_Error is one frame into a 3-deep FIFO that nothing
        // acknowledges, and a drive's startup watchdog fault survives a
        // missed one: read every drive's error register back, and clear
        // again while it still reports a fault.
        let mut clean = [false; N];
        for _round in 0..CLEAR_ROUNDS {
            for (j, clean) in clean.iter_mut().enumerate() {
                if *clean {
                    continue;
                }
                let node = self.bundle.robot.joints[j].node_id;
                self.bus.queue_poll_override(
                    PollAction::Poll {
                        node,
                        kind: PollKind::Errors,
                    },
                    1,
                );
                // The poll leaves with this tick; its answer is in the
                // next tick's drain.
                self.exchange([JointCommand::encoder_poll(); N], false)?;
                self.exchange([JointCommand::encoder_poll(); N], false)?;
                let flags = self.state.nodes[self.node(j)].error_flags;
                *clean = flags.is_some_and(|f| !reported_fault(&f));
                if !*clean {
                    self.emit(Event::Phase(
                        "still faulted after the clear; clearing again",
                        j,
                    ));
                    self.bus.send_clear_error(node, 1)?;
                }
            }
            if clean.iter().all(|c| *c) {
                break;
            }
        }
        if let Some(j) = clean.iter().position(|c| !c) {
            return Err(format!(
                "J{} still reports {:?} after {CLEAR_ROUNDS} clears",
                j + 1,
                self.state.nodes[self.node(j)].error_flags
            )
            .into());
        }
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
                self.found_at = self.hold.map(Some);
                holding = true;
            }
            // Encoder replies omit current; position-hold replies supply all
            // motion feedback.
            if holding
                && (0..N).all(|j| {
                    let s = &self.state.nodes[self.node(j)];
                    s.current_ma.is_some() && s.speed_ticks_s.is_some()
                })
            {
                self.ramp_current()?;
                self.check_tool()?;
                return self.check_startup_hold();
            }
        }
        Err("not all six drives supplied position, velocity and current feedback".into())
    }

    /// Raise the holding-current cap along a cubic, one limit frame per tick
    /// round-robin so the CAN budget is unchanged. Duration is the vendor jog
    /// ramp; the polynomial is Modern Robotics 9.2. This shapes a current cap,
    /// not a claim that measured current follows it.
    fn ramp_current(&mut self) -> Result<()> {
        let sweeps = self
            .ticks(self.bundle.robot.jog.accel_time_s)
            .div_ceil(N as u64);
        for j in 0..N {
            self.emit(Event::Phase("ramp startup holding current", j));
        }
        for step in 1..=sweeps {
            let u = step as f64 / sweeps as f64;
            let fraction = u * u * (3.0 - 2.0 * u);
            for j in 0..N {
                self.retune(j, self.bundle.robot.joints[j].ilim_ma * fraction)?;
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

    /// Does the bus agree with the config about what is on the flange?
    ///
    /// Full gripper identification is not available -- the drive's device info
    /// carries no model. What the bus does settle is whether a gripper driver
    /// answers at all, which separates a driven tool from a passive one.
    /// Getting that wrong silently is expensive: the tool's mass is most of
    /// the wrist's gravity load, so the whole identification comes out wrong.
    fn check_tool(&mut self) -> Result<()> {
        if self.simulated {
            return Ok(());
        }
        let node = self.bundle.robot.bus.gripper_node;
        let declared = self
            .bundle
            .active_tool()
            .is_some_and(|g| g.driver.is_some());
        for _ in 0..3 {
            self.bus.queue_poll_override(
                PollAction::Poll {
                    node,
                    kind: PollKind::DeviceInfo,
                },
                1,
            );
            self.settle(0.05)?;
        }
        let found = self.state.nodes[usize::from(node)].device_info.is_some();
        self.emit(Event::Tool(node, found));
        let tool = &self.bundle.robot.robot.active_tool;
        match (declared, found) {
            (true, true) | (false, false) => Ok(()),
            (true, false) => Err(format!(
                "config names `{tool}`, a tool with a CAN driver, but none answered on node \
                 {node}. Fit it, or select a passive tool such as `Flange`."
            )
            .into()),
            (false, true) => Err(format!(
                "config names `{tool}`, a passive tool, but a gripper driver answered on node \
                 {node}. Select the tool that is fitted: its mass is most of the wrist's load."
            )
            .into()),
        }
    }

    fn check_startup_hold(&mut self) -> Result<()> {
        self.check_hold("startup")
    }

    fn check_hold(&mut self, why: &'static str) -> Result<()> {
        let held = self.measure_hold(why)?;
        for (j, (offset, speed)) in held.iter().enumerate() {
            if *offset > self.tolerance() || *speed > self.holding_limit(j) {
                return Err(format!(
                    "J{} will not hold still at {why}: {:.4}deg, {:.4}deg/s",
                    j + 1,
                    offset.to_degrees(),
                    speed.to_degrees()
                )
                .into());
            }
        }
        Ok(())
    }

    /// Position RMS and speed RMS of every joint over the observation window.
    fn measure_hold(&mut self, why: &'static str) -> Result<[(f64, f64); N]> {
        let mut rings: [Ring; N] = std::array::from_fn(|_| Ring::default());
        let mut sums = [(0.0_f64, 0.0_f64, 0u32); N];
        let mut generation = self.generation;
        let window = self.ticks(SPEED_WINDOW_S);
        // A joint holding on trial gains has no guard but this one: the
        // shared guard stands aside for it and the motion's own guard ended
        // with the motion. A runaway here ends the hold at once as a failed
        // hold, and the trial's own backoff takes it from there.
        let trial = self.gain_joint.filter(|&j| self.sane_gains[j].is_some());
        let mut loud = 0u32;
        let mut ran_away = None;
        'hold: for _ in 0..self.ticks(HOLD_OBSERVATION_S) {
            self.frame(None)?;
            for j in 0..N {
                if generation[j] == self.generation[j] {
                    continue;
                }
                generation[j] = self.generation[j];
                if trial == Some(j) {
                    let reported = self.state.nodes[self.node(j)].speed_ticks_s.unwrap_or(0);
                    loud = if (f64::from(reported) * self.per_tick(j)).abs() > RUNAWAY_RAD_S {
                        loud + 1
                    } else {
                        0
                    };
                    if loud >= RUNAWAY_TICKS {
                        self.emit(Event::GainsNote(
                            self.tick,
                            j,
                            "ran away while holding on trial gains; the hold is failed",
                        ));
                        ran_away = Some(j);
                        break 'hold;
                    }
                }
                let p = self.pos(j)?;
                let per_tick = self.per_tick(j);
                let offset = (f64::from(p) - f64::from(self.hold[j])) * per_tick;
                let Some(speed) = rings[j].push(self.tick, p, window, self.dt) else {
                    continue;
                };
                let speed = speed * per_tick;
                sums[j].0 += offset * offset;
                sums[j].1 += speed * speed;
                sums[j].2 += 1;
            }
        }
        if ran_away.is_none() {
            if let Some(j) = sums.iter().position(|(_, _, n)| *n == 0) {
                return Err(
                    format!("J{} supplied no complete encoder window at {why}", j + 1).into(),
                );
            }
        }
        let out: [(f64, f64); N] = std::array::from_fn(|j| {
            if ran_away == Some(j) {
                return (f64::INFINITY, f64::INFINITY);
            }
            let n = f64::from(sums[j].2.max(1));
            ((sums[j].0 / n).sqrt(), (sums[j].1 / n).sqrt())
        });
        for (j, (offset, speed)) in out.iter().enumerate() {
            self.emit(Event::Hold(self.tick, j, why, *offset, *speed));
        }
        Ok(out)
    }

    // ------------------------------------------------------------ motion
    /// The joint's EXEC limits as move caps.
    fn exec_caps(&self, j: usize) -> Caps {
        let exec = self.bundle.robot.joints[j].limits.for_mode(LimitMode::Exec);
        Caps {
            velocity: exec.velocity_rad_s,
            acceleration: exec.acceleration_rad_s2,
            jerk: exec.jerk_rad_s3.unwrap_or(f64::INFINITY),
        }
    }

    /// The joint's hardware ceiling as move caps: what no move may exceed.
    fn ceiling_caps(&self, j: usize) -> Caps {
        let limits = &self.bundle.robot.joints[j].limits;
        Caps {
            velocity: limits.velocity_rad_s,
            acceleration: limits.acceleration_rad_s2,
            jerk: limits.jerk_rad_s3,
        }
    }

    /// Run one commanded position move on one joint under its EXEC limits.
    fn run_motion(&mut self, j: usize, target: i32, seconds: f64, score: bool) -> Result<Measure> {
        self.run_motion_capped(j, target, seconds, score, self.exec_caps(j))
    }

    /// Run one commanded position move on one joint and measure how it went.
    ///
    /// Septic time scaling over at least `seconds`, stretched further when
    /// `caps` needs it: peak speed is `SEPTIC_PEAK_VEL x distance /
    /// duration`, and jerk starts and ends at zero. Seeking is not here --
    /// `par6-rt`'s homing FSM owns the approach, the stall detection and the
    /// backoff, so this only ever runs a bounded position move.
    fn run_motion_capped(
        &mut self,
        j: usize,
        target: i32,
        seconds: f64,
        score: bool,
        caps: Caps,
    ) -> Result<Measure> {
        let start = self.pos(j)?;
        let per_tick = self.per_tick(j);
        let tolerance = self.tolerance();
        let dist_rad = ((f64::from(target) - f64::from(start)) * per_tick).abs();
        let profile = caps.profile(dist_rad, seconds, self.dt);
        let joint = &self.bundle.robot.joints[j];
        let ma_per_nm = torque_to_ma_factor(
            joint.gear_ratio,
            joint.gear_efficiency,
            joint.kt_nm_a,
            joint.dir,
        );
        let mut gravity = [0.0; N];
        let seconds = profile.duration();
        let home = &self.bundle.robot.homing.joints[j];
        let window_s = seconds + self.bundle.robot.motion.settle_timeout_s;
        let ramp = self.bundle.robot.jog.accel_time_s.max(self.dt);
        let duration = window_s;
        let direction = (i64::from(target) - i64::from(start)).signum() as i32;
        // Every unreferenced leg toward the stop uses the fixed homing
        // current; away and holding use full operating current.
        let toward_stop =
            self.homing && !self.homed[j] && direction == if home.direction == 1 { -1 } else { 1 };
        let operating = self.bundle.robot.joints[j].ilim_ma;
        let limit = if toward_stop {
            home.current_ma
        } else {
            operating
        };
        let peak_command_rad_s = SEPTIC_PEAK_VEL * dist_rad / seconds;
        self.emit(Event::Phase("position move", j));
        self.retune(j, limit)?;
        self.bus.queue_poll_override(
            PollAction::ConfigFrame {
                node: self.bundle.robot.joints[j].node_id,
                kind: ConfigKind::Limits,
            },
            3,
        );

        let mut out = Measure {
            peak_command_rad_s,
            ..Measure::default()
        };
        // Current feedforward for the frame's own channel: gravity at the
        // commanded pose plus the inertial torque of the profile, so the
        // position loop only has to close friction and model error. Only
        // this joint moves, and its own diagonal does not depend on its own
        // angle, so one inertia evaluation covers the move.
        let mut q = self.angles()?;
        let m_jj = self.inertia(q)?[j];
        let mut expected = f64::from(start);
        let mut commanded_speed = 0;
        let mut ring = Ring::default();
        let mut reference_ring = Ring::default();
        let mut guard_contact = ContactGuard::new(self.ticks(STALL_WINDOW_S), start, expected);
        let gain_trial = self.sane_gains[j].is_some();
        // Gain trials reset the integrator. The jog ramp must not exclude
        // the entire acceleration and peak speed of a short profile.
        let guard = self.ticks(if gain_trial {
            DETECT_GUARD_S
        } else {
            DETECT_GUARD_S.max(ramp)
        });
        let speed_window = self.ticks(SPEED_WINDOW_S);
        let mut generation = self.generation[j];
        let started = self.tick;
        let mut runaway_ticks = 0u32;
        let mut sign = 0i8;
        for t in 0..self.ticks(duration) {
            let reference = expected;
            let reference_speed = commanded_speed;
            let (s, s_dot, s_ddot) = profile.sample((t + 1) as f64 * self.dt);
            let distance = f64::from(target) - f64::from(start);
            expected = f64::from(start) + distance * s;
            commanded_speed = (distance * s_dot) as i32;
            q[j] = self.conv[j].joint_rad(expected.round() as i32);
            self.kin.gravity(&q, &mut gravity)?;
            let feedforward = (gravity[j] + m_jj * distance * s_ddot * per_tick) * ma_per_nm;
            let cmd = JointCommand::position(
                expected.round() as i32,
                commanded_speed,
                feedforward.clamp(-limit, limit).round() as i16,
            );
            self.frame(Some((j, cmd)))?;
            if self.generation[j] == generation {
                continue;
            }
            generation = self.generation[j];
            let p = self.pos(j)?;
            let current = f64::from(
                self.state.nodes[self.node(j)]
                    .current_ma
                    .ok_or("missing current feedback")?,
            );
            let measured = if gain_trial && !self.simulated {
                let received = self.position_rx_ns[j];
                if received == 0 {
                    return Err(format!("J{} motion reply has no matching timestamp", j + 1).into());
                }
                if ring.len > 0 && received <= ring.samples[ring.len - 1].2 {
                    return Err("motion receive timestamps did not advance".into());
                }
                ring.push_at(self.tick, p, speed_window, received)
            } else {
                ring.push(self.tick, p, speed_window, self.dt)
            };
            let reference_measured =
                reference_ring.push(self.tick, reference.round() as i32, speed_window, self.dt);
            let scored_speed = if gain_trial {
                reference_measured.unwrap_or(f64::from(reference_speed))
            } else {
                f64::from(reference_speed)
            };
            let error = (f64::from(p) - reference) * per_tick;
            if let Some(sane) = self.sane_gains[j] {
                let speed = f64::from(
                    self.state.nodes[self.node(j)]
                        .speed_ticks_s
                        .ok_or("missing velocity feedback")?,
                );
                let speed_error = (speed - f64::from(reference_speed)) * per_tick;
                runaway_ticks = if speed_error.abs() > RUNAWAY_RAD_S {
                    runaway_ticks + 1
                } else {
                    0
                };
                if runaway_ticks >= RUNAWAY_TICKS {
                    self.emit(Event::GainsNote(
                        self.tick,
                        j,
                        "runaway speed during motion; stopping with original gains",
                    ));
                    self.gain_configure(j, sane)?;
                    out.outcome = Outcome::Unstable;
                    break;
                }
            }
            // Two exclusions on the scoring window. The guard drops the ramp
            // and the drive's velocity integral unwinding from the previous
            // command -- a spike there used to fail a gain that tracked. The
            // endstop band drops samples within ENDSTOP_EXCLUSION_RAD of the
            // stop this joint last touched.
            let near_stop = self.endstop_guard[j].is_some_and(|contact| {
                ((f64::from(p) - f64::from(contact)) * per_tick).abs() < ENDSTOP_EXCLUSION_RAD
            });
            if t >= guard && !near_stop {
                out.peak_error_rad = out.peak_error_rad.max(error.abs());
                out.samples += 1;
                out.position_sq += error * error;
                if let Some(v) = measured {
                    let speed_error = (v - scored_speed) * per_tick;
                    out.speed_sq += speed_error * speed_error;
                    let next = if speed_error > SWEEP_BAND_RAD_S {
                        1
                    } else if speed_error < -SWEEP_BAND_RAD_S {
                        -1
                    } else {
                        0
                    };
                    if next != 0 {
                        if sign != 0 && sign != next {
                            out.reversals += 1;
                        }
                        sign = next;
                    }
                }
                let q = self.angles()?;
                self.kin.gravity(&q, &mut gravity)?;
                let dynamic = current - gravity[j] * ma_per_nm;
                out.peak_dynamic_ma = out.peak_dynamic_ma.max(dynamic.abs());
            }

            if t >= self.ticks(seconds)
                && ((f64::from(p) - f64::from(target)) * per_tick).abs() <= tolerance
            {
                out.outcome_complete();
                break;
            }
            if t < guard {
                guard_contact.restart(t, p, reference);
                continue;
            }
            if let Some(contact) = guard_contact.observe(t, p, reference, current, limit) {
                self.emit(Event::Contact(
                    self.tick,
                    j,
                    contact.range,
                    contact.requested,
                    contact.below,
                    contact.stopped,
                    contact.loaded,
                ));
                if contact.blocked {
                    self.endstop_guard[j] = Some(p);
                    out.outcome = Outcome::Blocked;
                    break;
                }
            }
        }
        out.elapsed = (self.tick - started) as f64 * self.dt;
        self.adopt(j)?;
        if toward_stop {
            self.retune(j, operating)?;
        }
        let held = self.measure_hold("after move")?;
        out.hold_rms_rad_s = held[j].1;
        // Where the joint is once it has held, not where it landed: a joint
        // that drifts through the hold has not settled.
        out.settled_error_rad = ((f64::from(self.pos(j)?) - f64::from(target)) * per_tick).abs();
        if score {
            out.score(true, MOVING_RMS_RAD_S, self.holding_limit(j), tolerance);
        }
        self.emit(Event::Move(
            self.tick,
            j,
            out.outcome,
            out.elapsed,
            start,
            self.pos(j)?,
            out,
        ));
        Ok(out)
    }
}

/// Contact detection for an ordinary move over a sliding window: a joint
/// drawing current while its encoder range stays under a quarter of the
/// commanded travel is against something — a displacement plateau while the
/// drive is pulling current, which is how a move that jams reports itself.
///
/// Not the homing stall detector -- `par6-rt`'s `Homer` owns that, with the
/// vendor window and current-ratio rules, and this is the weaker predicate a
/// move that is not approaching a stop needs.
struct ContactGuard {
    window: u64,
    start: u64,
    min: i32,
    max: i32,
    reference: f64,
    loaded_samples: u32,
    samples: u32,
}

struct Contact {
    range: i64,
    requested: f64,
    below: f64,
    stopped: bool,
    loaded: bool,
    blocked: bool,
}

impl ContactGuard {
    fn new(window: u64, at: i32, reference: f64) -> Self {
        Self {
            window,
            start: 0,
            min: at,
            max: at,
            reference,
            loaded_samples: 0,
            samples: 0,
        }
    }
    fn restart(&mut self, t: u64, at: i32, reference: f64) {
        self.start = t;
        self.min = at;
        self.max = at;
        self.reference = reference;
        self.loaded_samples = 0;
        self.samples = 0;
    }
    fn observe(
        &mut self,
        t: u64,
        at: i32,
        reference: f64,
        current: f64,
        limit: f64,
    ) -> Option<Contact> {
        self.samples += 1;
        self.loaded_samples += u32::from(current.abs() >= STALL_CURRENT_FRACTION * limit);
        self.min = self.min.min(at);
        self.max = self.max.max(at);
        if t.saturating_sub(self.start) < self.window {
            return None;
        }
        let requested = (reference - self.reference).abs();
        // The whole range, so back-and-forth movement is not mistaken for
        // standstill because its net displacement is zero.
        let range = i64::from(self.max) - i64::from(self.min);
        let below = (requested * STALL_TRAVEL_FRACTION).max(STALL_TRAVEL_FLOOR);
        let stopped = (range as f64) < below;
        let loaded = f64::from(self.loaded_samples) >= STALL_OCCUPANCY * f64::from(self.samples);
        let blocked = loaded && stopped && requested >= STALL_TRAVEL_FLOOR;
        self.restart(t, at, reference);
        Some(Contact {
            range,
            requested,
            below,
            stopped,
            loaded,
            blocked,
        })
    }
}

impl Arm {
    // ------------------------------------------------------------ homing

    /// Walk the configured homing sequence.
    fn home(&mut self) -> Result<()> {
        self.homing = true;
        let plan = self.bundle.robot.homing.for_tool(self.bundle.active_tool());
        for step in plan.sequence {
            for m in step.pre_moves {
                self.premove(m)?;
            }
            if let Some(group) = step.home {
                for j in group.joints {
                    let j = usize::from(j);
                    self.emit(Event::Phase("home", j));
                    // Reference, limits and the post-home move are the FSM's;
                    // this records that the joint is referenced and where the
                    // stop it was referenced against sits.
                    let contact = self.home_joint(j)?;
                    self.endstop_guard[j] = Some(contact);
                    self.homed[j] = true;
                }
            }
            for m in step.move_to {
                self.move_joint(usize::from(m.joint), m.position_rad, m.duration_s)?;
            }
            for m in step.post_moves {
                self.premove(m)?;
            }
        }
        for m in plan.post_moves {
            self.premove(m)?;
        }
        self.homing = false;
        for j in 0..N {
            self.operating(j)?;
        }
        Ok(())
    }

    /// Find one joint's reference by driving the runtime's own homing FSM.
    ///
    /// `par6-rt` owns the approach, stall detection, backoff, the two-pass
    /// compare and the release phase, with the vendor constants. Selfcal
    /// writes a config whose home reference the daemon has to reproduce, so a
    /// second implementation of this would be a second reference; `Homer::tick`
    /// takes no bus and no generics, so the same FSM drives from here.
    ///
    /// The orchestrator's protocol, minus the sequence: tick while it runs,
    /// apply the reference when it latches, and hand back the post-home target
    /// in motor ticks, which the FSM has no joint conversion to compute.
    fn home_joint(&mut self, j: usize) -> Result<i32> {
        let robot = &self.bundle.robot;
        let cfg = &robot.joints[j];
        let jh = &robot.homing.joints[j];
        let params = HomerParams::from_config(
            cfg.node_id,
            jh,
            jh.seek_timeout_s(cfg),
            cfg.velocity_limit_ticks_s,
            cfg.ilim_ma,
            self.dt,
        );
        let offset = self
            .bundle
            .effective_home_offset(j)
            .ok_or("missing home offset")?;
        let mut homer = Homer::new(&params);
        homer.start();
        // The seeking current limit, restored to the operating one the moment
        // the reference latches -- the orchestrator's `apply_phase_limits`
        // rule, which is this much when there is no step machinery above it.
        self.configure(j, params.current_ma)?;
        let mut reference = None;
        while homer.running() {
            let (cmd, event) = homer.tick(&params, &mut self.state.nodes[self.node(j)]);
            self.frame(Some((j, cmd)))?;
            match event {
                Some(HomerEvent::Reference { latched_ticks }) => {
                    self.conv[j].set_home(latched_ticks, offset);
                    // The frames that carry the limit change hold every
                    // joint at `hold`; for this one that is still where the
                    // seek began, tens of thousands of ticks away.
                    self.adopt(j)?;
                    self.operating(j)?;
                    if let Some(post) = self.bundle.robot.homing.joints[j].post_home {
                        homer.post_target_ticks = self.conv[j].motor_ticks(post.position_rad);
                    }
                    reference = Some(latched_ticks);
                }
                Some(HomerEvent::Failed) => {
                    return Err(format!("J{} homing failed", j + 1).into());
                }
                None => {}
            }
        }
        self.adopt(j)?;
        reference.ok_or_else(|| format!("J{} finished homing without a reference", j + 1).into())
    }

    fn premove(&mut self, spec: PreMove) -> Result<()> {
        match spec {
            PreMove::Nudge {
                joint,
                speed_ticks_s,
                duration_s,
            } => {
                let j = usize::from(joint);
                let target = self
                    .pos(j)?
                    .saturating_add((speed_ticks_s * duration_s) as i32);
                self.run_motion(j, target, duration_s * SEPTIC_PEAK_VEL, false)?;
            }
            PreMove::Position {
                joint,
                position_rad,
                duration_s,
            } => {
                self.move_joint(usize::from(joint), position_rad, duration_s)?;
            }
            PreMove::Idle { joint, duration_s } => {
                let j = usize::from(joint);
                let mut generation = self.generation[j];
                // Proceed on fresh feedback after releasing; the configured
                // duration is only a timeout.
                for t in 0..self.ticks(duration_s) {
                    let cmd = if t < 2 {
                        JointCommand::drop_to_idle()
                    } else {
                        JointCommand::encoder_poll()
                    };
                    self.frame(Some((j, cmd)))?;
                    if t < 2 {
                        generation = self.generation[j];
                    } else if self.generation[j] != generation {
                        return self.adopt(j);
                    }
                }
                return Err(format!("J{} gave no feedback after release", j + 1).into());
            }
            // Selfcal drives the six arm joints only; a sequence that needs the
            // gripper moved cannot be run here, and silently skipping it would
            // home the arm from a pose the config did not ask for.
            PreMove::GripperMove { .. } => {
                return Err(
                    "the homing sequence moves the gripper, which selfcal does not drive".into(),
                );
            }
        }
        Ok(())
    }

    fn move_joint(&mut self, j: usize, radians: f64, seconds: f64) -> Result<()> {
        if !self.homed[j] {
            return Err(format!("J{} is not referenced", j + 1).into());
        }
        let target = self.conv[j].motor_ticks(radians);
        let result = self.run_motion(j, target, seconds.max(self.dt), false)?;
        if result.outcome != Outcome::Complete {
            return Err(format!("J{} did not reach the requested position", j + 1).into());
        }
        Ok(())
    }

    // ------------------------------------------------------------ tuning

    /// Joint-side inertia at `q`, as [`joint_inertia`] models it.
    fn inertia(&mut self, q: [f64; N]) -> Result<[f64; N]> {
        joint_inertia(&mut self.kin, &self.bundle, q)
    }

    /// The septic that takes joint `j` between rest and `speed_rad_s`.
    ///
    /// The profiled quantity is the velocity itself, so its first and second
    /// derivatives are the acceleration and jerk: per unit of speed change the
    /// septic's "velocity" cap is `a / v` and its "acceleration" cap `j / v`,
    /// both from the joint's EXEC limits. Acceleration and jerk start and end
    /// at zero, so a leg neither jolts in nor snaps to a stop.
    fn speed_ramp(&self, j: usize, speed_rad_s: f64) -> SSeptic {
        let exec = self.bundle.robot.joints[j].limits.for_mode(LimitMode::Exec);
        let v = speed_rad_s.abs();
        SSeptic::new(
            exec.acceleration_rad_s2 / v,
            exec.jerk_rad_s3.unwrap_or(f64::INFINITY) / v,
            f64::INFINITY,
            None,
            self.dt,
        )
    }

    /// One constant-velocity leg: ramp to `ticks_s`, hold it until the speed
    /// settles, average the current it costs \[mA, signed\], then ramp back to
    /// rest. `None` when the joint never reached the commanded speed, so the
    /// leg says nothing about friction.
    ///
    /// Velocity rather than torque, because a joint whose viscous friction is
    /// near zero has no terminal velocity: under constant torque it simply
    /// accelerates until it runs out of travel, and the steady state the fit
    /// needs never arrives. Held at a speed, the balance is the same equation
    /// read the other way round, and the travel per leg is known in advance
    /// rather than discovered by hitting a limit.
    fn drag(&mut self, j: usize, ticks_s: f64) -> Result<Option<Vec<DragSample>>> {
        let per_tick = self.per_tick(j).abs();
        let profile = self.speed_ramp(j, ticks_s * per_tick);
        let ramp = self.ticks(profile.duration());
        let settle = ramp + self.ticks(DRAG_SETTLE_S);
        let measured_until = settle + self.ticks(DRAG_AVERAGE_S);
        // Centre the measured arc so the reverse leg covers the same poses.
        let hold = measured_until + self.ticks(DRAG_SETTLE_S);
        let mut ring = Ring::default();
        let window = self.ticks(SPEED_WINDOW_S);
        let mut generation = self.generation[j];
        let mut speed = 0.0;
        let mut n = 0.0;
        let mut samples = Vec::with_capacity(self.ticks(DRAG_AVERAGE_S) as usize + 1);
        let node = self.node(j);
        let mut saturated = false;
        for t in 0..hold + ramp {
            let fraction = if t < ramp {
                profile.sample((t + 1) as f64 * self.dt).0
            } else if t < hold {
                1.0
            } else {
                1.0 - profile.sample((t - hold + 1) as f64 * self.dt).0
            };
            let cmd = JointCommand::velocity((ticks_s * fraction) as i32, 0);
            self.state.nodes[node].current_ma = None;
            self.frame(Some((j, cmd)))?;
            if self.generation[j] == generation {
                continue;
            }
            generation = self.generation[j];
            let p = self.pos(j)?;
            let received = if self.encoder_clock.is_some() {
                self.position_rx_ns[j]
            } else {
                (self.tick as f64 * self.dt * 1e9) as u64
            };
            if received == 0 {
                continue;
            }
            if let (Some(v), Some(ma)) = (
                ring.push_at(self.tick, p, window, received),
                self.state.nodes[node].current_ma,
            ) {
                if (settle..measured_until).contains(&t) {
                    saturated |= f64::from(ma).abs() >= self.bundle.robot.joints[j].ilim_ma;
                    speed += v;
                    n += 1.0;
                    samples.push(DragSample {
                        position: f64::from(p),
                        speed: v,
                        current: f64::from(ma),
                    });
                }
            }
        }
        // The ramp has already brought it to rest; hold where it stopped.
        self.adopt(j)?;
        self.settle(DRAG_RECOVER_S)?;
        if n < 2.0 || saturated {
            return Ok(None);
        }
        let held = speed / n;
        // The drive has to have actually reached the commanded speed, or the
        // current is paying for acceleration rather than for friction.
        if (held - ticks_s).abs() > DRAG_SPEED_TOLERANCE * ticks_s.abs()
            || (held * per_tick).abs() < COAST_MIN_RAD_S
        {
            return Ok(None);
        }
        Ok(Some(samples))
    }

    /// Viscous and Coulomb friction for one joint \[Nm.s/rad, Nm\], joint side.
    ///
    /// Each speed is held both ways over the same arc. Gravity is
    /// position-dependent but direction-independent, so their signed
    /// half-difference cancels the gravity shared by the two directions.
    /// No gravity model enters the friction fit:
    ///
    ///   forward:  I_f/k = +b v + tc + tau_g
    ///   reverse:  I_r/k = -b v - tc + tau_g
    ///   half diff: (I_f - I_r)/2k = b v + tc
    ///
    /// Two parameters, ordinary least squares over the speeds that held.
    fn friction(&mut self, j: usize, span: (f64, f64)) -> Result<Option<(f64, f64)>> {
        let home = self.angles()?[j];
        let (up, down) = (span.1 - home, home - span.0);
        // Out toward the wider side of the checked span, never past a share
        // of its room: the legs have no contact abort of their own.
        let budget = DRAG_TRAVEL_RAD.min(GAINS_ROOM_SHARE * up.max(down));
        let outward = if up >= down { 1.0 } else { -1.0 } * self.per_tick(j).signum();
        let cfg = &self.bundle.robot.joints[j];
        let factor =
            torque_to_ma_factor(cfg.gear_ratio, cfg.gear_efficiency, cfg.kt_nm_a, cfg.dir).abs();
        let per_tick = self.per_tick(j).abs();
        // The fastest leg that still fits the travel budget, so a joint near
        // its limits is bounded by geometry rather than by luck. A leg at `v`
        // covers `v * span` holding and `v * T` across its two ramps (each
        // septic averages half its end value), and `T` grows with `v`, so the
        // speed is found by bisection on that travel.
        let span = 2.0 * DRAG_SETTLE_S + DRAG_AVERAGE_S;
        let travel = |v: f64| v * (span + self.speed_ramp(j, v).duration());
        let (mut lo, mut hi) = (0.0, cfg.limits.for_mode(LimitMode::Exec).velocity_rad_s);
        if travel(hi) > budget {
            for _ in 0..50 {
                let mid = 0.5 * (lo + hi);
                if travel(mid) > budget {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            hi = lo;
        }
        let fastest = hi / per_tick;
        let mut rows: Vec<(f64, f64)> = Vec::new();
        for step in 0..DRAG_LEVELS {
            let fraction = DRAG_MIN_FRACTION
                + (DRAG_MAX_FRACTION - DRAG_MIN_FRACTION) * step as f64
                    / (DRAG_LEVELS - 1).max(1) as f64;
            let ticks_s = fraction * fastest;
            let start = self.pos(j)?;
            let out = self.drag(j, outward * ticks_s)?;
            let back = if out.is_some() {
                self.drag(j, -outward * ticks_s)?
            } else {
                None
            };
            let (forward, reverse) = if outward > 0.0 {
                (out, back)
            } else {
                (back, out)
            };
            // Every speed starts at the same pose, including rejected legs.
            self.return_to(j, start)?;
            let (Some(forward), Some(reverse)) = (forward, reverse) else {
                continue;
            };
            let extent = |samples: &[DragSample]| {
                samples
                    .iter()
                    .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), s| {
                        (lo.min(s.position), hi.max(s.position))
                    })
            };
            let (flo, fhi) = extent(&forward);
            let (rlo, rhi) = extent(&reverse);
            let (lo, hi) = (flo.max(rlo), fhi.min(rhi));
            // Insufficient common travel does not identify direction-independent gravity.
            if hi - lo < 0.5 * ticks_s.abs() * DRAG_AVERAGE_S {
                self.emit(Event::Phase(
                    "friction: insufficient shared measurement arc",
                    j,
                ));
                continue;
            }
            let mut sum = [0.0; 3];
            let mut matched = 0;
            for bin in 0..16 {
                let at = lo + (hi - lo) * (f64::from(bin) + 0.5) / 16.0;
                if let (Some(f), Some(r)) =
                    (DragSample::at(&forward, at), DragSample::at(&reverse, at))
                {
                    if f.speed > 0.0 && r.speed < 0.0 {
                        sum[0] += f.current;
                        sum[1] += r.current;
                        sum[2] += (f.speed - r.speed) * 0.5;
                        matched += 1;
                    }
                }
            }
            if matched < 8 {
                self.emit(Event::Phase("friction: too few matching moving samples", j));
                continue;
            }
            let forward = sum[0] / f64::from(matched);
            let reverse = sum[1] / f64::from(matched);
            let torque = (forward - reverse) / 2.0 / factor;
            let speed = sum[2] / f64::from(matched) * per_tick;
            self.emit(Event::Drag(self.tick, j, speed, forward, reverse));
            rows.push((speed, torque));
        }
        if rows.len() < 3 {
            self.emit(Event::Phase(
                "friction: fewer than three valid speeds; fit unresolved",
                j,
            ));
            return Ok(None);
        }
        // tau = b v + tc: the normal equations for a straight line.
        let n = rows.len() as f64;
        let sv: f64 = rows.iter().map(|r| r.0).sum();
        let st: f64 = rows.iter().map(|r| r.1).sum();
        let svv: f64 = rows.iter().map(|r| r.0 * r.0).sum();
        let svt: f64 = rows.iter().map(|r| r.0 * r.1).sum();
        let denominator = n * svv - sv * sv;
        if denominator.abs() < f64::EPSILON {
            return Err(
                format!("J{} held one speed only; friction is not separable", j + 1).into(),
            );
        }
        let b = (n * svt - sv * st) / denominator;
        let tc = (st - b * sv) / n;
        let variance = rows
            .iter()
            .map(|(v, t)| (t - b * v - tc).powi(2))
            .sum::<f64>()
            / (n - 2.0);
        let b_se = (variance * n / denominator).sqrt();
        let tc_se = (variance * svv / denominator).sqrt();
        // This flags unresolved coefficients, not a confidence interval or
        // a substitute for comparing independent hardware measurements.
        self.friction_fit_uncertain[j] = b_se >= b.abs() || tc_se >= tc.abs();
        self.emit(Event::FrictionQuality(
            self.tick,
            j,
            b,
            tc,
            b_se,
            tc_se,
            rows.len(),
        ));
        if !b.is_finite() || !tc.is_finite() || b < 0.0 || tc < 0.0 {
            self.emit(Event::Phase(
                "friction: invalid fit; configured values retained",
                j,
            ));
            return Ok(None);
        }
        Ok(Some((b, tc)))
    }

    /// Measure every joint's inertia and friction.
    ///
    /// The gains are not touched. Setting `Kpv` needs a design crossover and
    /// the arm has none to offer: on 2026-09-22 the shipped gains implied
    /// crossovers spanning two orders of magnitude (J1 11 rad/s against J6
    /// 2244), because what a joint can run is set by its own inertia and its
    /// drive's inner lag -- and that lag is inside the drive, where a 250 Hz
    /// link cannot see it. Nothing measurable from out here decides a value,
    /// so nothing here writes one.
    ///
    /// What this replaces is a relay that measured the 250 Hz CAN round trip
    /// rather than the joint (identical `Tu` on joints an order of magnitude
    /// apart in inertia) and handed the result to a loop closed at 6250 Hz.
    fn measure_mechanics(&mut self, spans: &[(f64, f64); N]) -> Result<[Option<(f64, f64)>; N]> {
        let q = self.angles()?;
        let inertia = self.inertia(q)?;
        let mut friction = [None; N];
        for j in 0..N {
            self.emit(Event::Phase("friction", j));
            let Some((b, tc)) = self.friction(j, spans[j])? else {
                continue;
            };
            self.emit(Event::Mechanics(self.tick, j, inertia[j], b, tc));
            friction[j] = Some((b, tc));
        }
        Ok(friction)
    }

    /// The current at which joint `j` breaks away from rest at the pose the
    /// arm holds, ramped from the model's gravity balance in `sign`'s
    /// direction, and the transmission's wind-up when it did \[mA, motor
    /// ticks\]; `None` when the ramp reached its ceiling with the joint
    /// still.
    ///
    /// The encoder is on the motor, and a belt or gear train winds up
    /// before the link moves: on the first arm the geared joints "broke
    /// away" a few ticks in with nothing but the gearbox flexing. Breakaway
    /// is therefore sustained sliding -- the encoder advancing faster over
    /// the last `STICTION_WINDOW_S` than the ramp can wind a transmission --
    /// and the current is the one at the start of that window. The joint
    /// is caught there, held, then eased back to where it started.
    fn breakaway(&mut self, j: usize, sign: f64) -> Result<Option<(f64, i64)>> {
        // Still first: the previous breakaway's return was still settling
        // when J3's ramp began, and the sliding test read that as a
        // breakaway at the balance current.
        self.wait_still(std::array::from_fn(|k| k == j))?;
        let window = self.ticks(STICTION_WINDOW_S);
        let start = self.pos(j)?;
        // The ramp starts from the current the drive is holding with, not
        // the model's balance: the position loop parks a joint against its
        // own stiction with a push of its own (J3 held with 100 mA past the
        // model), and dropping that at the switch to current mode sprang the
        // joint most of a degree before any ramp had begun.
        let balance = self.holding_current_ma(j);
        let ilim = self.bundle.robot.joints[j].ilim_ma;
        let rate = STICTION_RAMP_ILIM_PER_S * ilim;
        let ceiling = STICTION_MAX_ILIM * ilim;
        let mut generation = self.generation[j];
        let mut recent: VecDeque<(u64, i64, f64)> = VecDeque::with_capacity(window as usize + 2);
        let mut found = None;
        for t in 0.. {
            let ramp = sign * rate * t as f64 * self.dt;
            if ramp.abs() > ceiling {
                break;
            }
            let current = (balance + ramp).clamp(-ilim, ilim);
            self.frame(Some((j, JointCommand::current(current.round() as i16))))?;
            if self.generation[j] == generation {
                continue;
            }
            generation = self.generation[j];
            let moved = i64::from(self.pos(j)?) - i64::from(start);
            recent.push_back((self.tick, moved, current));
            while recent
                .front()
                .is_some_and(|(tick, _, _)| self.tick.saturating_sub(*tick) > window)
            {
                recent.pop_front();
            }
            let Some(&(oldest_tick, oldest_moved, oldest_current)) = recent.front() else {
                continue;
            };
            let span_s = (self.tick - oldest_tick) as f64 * self.dt;
            if span_s < 0.5 * STICTION_WINDOW_S {
                continue;
            }
            let ticks_per_s = (moved - oldest_moved) as f64 / span_s;
            if ticks_per_s.abs() >= STICTION_SLIDE_TICKS_PER_S
                && moved.abs() >= i64::from(STICTION_BREAK_TICKS)
                && ramp.abs() >= STICTION_MIN_RAMP_ILIM * ilim
            {
                found = Some((oldest_current, oldest_moved));
                break;
            }
        }
        self.return_to(j, start)?;
        Ok(found)
    }

    /// The current the drive holds joint `j` with now \[mA\]: what it last
    /// reported, or the gravity model's balance before it has reported.
    fn holding_current_ma(&mut self, j: usize) -> f64 {
        let node = self.node(j);
        match self.state.nodes[node].current_ma {
            Some(ma) => f64::from(ma),
            None => f64::from(self.gravity_feedforward()[j]),
        }
    }

    /// Catch joint `j` where a current-mode excursion left it and ease it
    /// back to `start` on the septic under its EXEC caps: a chirp can leave
    /// the base degrees away, which is no distance to jump in one frame.
    fn return_to(&mut self, j: usize, start: i32) -> Result<()> {
        self.adopt(j)?;
        let result = self.run_motion(j, start, RETURN_S, false)?;
        if result.outcome != Outcome::Complete {
            return Err(format!("J{} did not return to where it started", j + 1).into());
        }
        Ok(())
    }

    /// Static friction per joint at the pose the arm holds \[Nm\]. The two
    /// ramps, up and down from the balance, meet static friction on
    /// opposite sides of gravity, so half their difference is the friction
    /// and their mean is the gravity current the joint really carries --
    /// reported beside the model's, which is what the ramps started from.
    fn stiction(&mut self, label: &'static str) -> Result<[Option<f64>; N]> {
        let mut out = [None; N];
        for (j, slot) in out.iter_mut().enumerate() {
            self.emit(Event::Phase("stiction", j));
            let cfg = &self.bundle.robot.joints[j];
            let factor =
                torque_to_ma_factor(cfg.gear_ratio, cfg.gear_efficiency, cfg.kt_nm_a, cfg.dir)
                    .abs();
            let model = f64::from(self.gravity_feedforward()[j]);
            let up = self.breakaway(j, 1.0)?;
            let down = self.breakaway(j, -1.0)?;
            let (Some((up, up_windup)), Some((down, down_windup))) = (up, down) else {
                self.emit(Event::Phase("no breakaway within the ramp", j));
                continue;
            };
            let static_nm = (up - down).abs() / 2.0 / factor;
            let measured = (up + down) / 2.0;
            // The motor turned this far, against the static friction, before
            // the link moved: the transmission's wind-up, and the torque over
            // it is the transmission's stiffness, joint side.
            let windup = (up_windup.abs() + down_windup.abs()) as f64 / 2.0;
            let stiffness = if windup >= 1.0 {
                static_nm / (windup * self.per_tick(j).abs())
            } else {
                f64::NAN
            };
            self.emit(Event::Stiction(
                self.tick, j, label, up, down, static_nm, model, measured, windup, stiffness,
            ));
            *slot = Some(static_nm);
        }
        Ok(out)
    }

    /// Chirp joint `j`'s drive in current mode about its gravity current:
    /// the sine of `BELT_CHIRP_NM` sweeping `BELT_F_LO_HZ` to
    /// `BELT_F_HI_HZ` over `BELT_SECONDS`. Every tick's command and the
    /// encoder's answer go to the run's samples, which is what the belt
    /// fit reads; nothing is fitted here. Travel past `BELT_ABORT_RAD`
    /// stops it, and the joint is held where it is either way, then eased
    /// back to where it started. Refused unless `span`, the interval the
    /// joint may sweep clear of its limits and the collision world, gives
    /// the cutoff `BELT_CLEARANCE` on both sides: current mode has no
    /// bounds of its own, and a stop inside the cutoff would take the
    /// whole chirp.
    fn belt(&mut self, j: usize, span: (f64, f64)) -> Result<bool> {
        self.emit(Event::Phase("belt chirp", j));
        self.wait_still(std::array::from_fn(|k| k == j))?;
        let start = self.pos(j)?;
        let at = self.conv[j].joint_rad(start);
        let clear = (at - span.0).min(span.1 - at);
        if clear < BELT_CLEARANCE * BELT_ABORT_RAD {
            return Err(format!(
                "belt chirp: J{} has {:.1} deg clear of its limits and the collision world, needs {:.1}",
                j + 1,
                clear.to_degrees(),
                (BELT_CLEARANCE * BELT_ABORT_RAD).to_degrees()
            )
            .into());
        }
        // The base's gravity torque is zero. Its position-loop holding
        // current includes transmission preload: carrying that into the
        // chirp added 97 mA of DC and drove the base into its travel guard.
        let balance = f64::from(self.gravity_feedforward()[j]);
        let cfg = &self.bundle.robot.joints[j];
        let ilim = cfg.ilim_ma;
        let amplitude = BELT_CHIRP_NM
            * torque_to_ma_factor(cfg.gear_ratio, cfg.gear_efficiency, cfg.kt_nm_a, cfg.dir).abs();
        let per_tick = self.per_tick(j).abs();
        let (mut lo, mut hi) = (start, start);
        let mut aborted = false;
        for t in 0..self.ticks(BELT_SECONDS) {
            let s = t as f64 * self.dt;
            let phase = std::f64::consts::TAU
                * (BELT_F_LO_HZ * s + (BELT_F_HI_HZ - BELT_F_LO_HZ) * s * s / (2.0 * BELT_SECONDS));
            let current = (balance + amplitude * phase.sin()).clamp(-ilim, ilim);
            self.frame(Some((j, JointCommand::current(current.round() as i16))))?;
            let p = self.pos(j)?;
            lo = lo.min(p);
            hi = hi.max(p);
            if ((f64::from(p) - f64::from(start)) * per_tick).abs() > BELT_ABORT_RAD {
                aborted = true;
                break;
            }
        }
        let last = self.pos(j)?;
        self.return_to(j, start)?;
        self.emit(Event::Belt(
            self.tick,
            j,
            (f64::from(hi - lo) * per_tick).to_degrees(),
            (f64::from(last - start) * per_tick).to_degrees(),
            aborted,
        ));
        Ok(!aborted)
    }
}

impl Arm {
    // ------------------------------------------------------------ limits

    /// Measure and cancel each joint's ripple: cogging and commutation
    /// error, torques fixed to the rotor's electrical angle. A slow sweep
    /// each way at `RIPPLE_SWEEP_TICKS_S`, captured with the electrical
    /// phase, gives the current the loop spends at each angle; its
    /// harmonics, averaged over the two directions so friction and the
    /// loop's lag cancel, go to the drive as feedforward (cmd 40). The gains
    /// step is captured without and with it, and the feedforward stays only
    /// if the speed ripple at those harmonics falls by
    /// `RIPPLE_MIN_IMPROVEMENT`. A joint left without any says why.
    fn ripple(
        &mut self,
        ready: [f64; N],
        spans: &[(f64, f64); N],
        chosen: [bool; N],
    ) -> Result<[Option<Vec<RippleHarmonic>>; N]> {
        let mut found: [Option<Vec<RippleHarmonic>>; N] = Default::default();
        for (j, slot) in found.iter_mut().enumerate() {
            if !chosen[j] {
                continue;
            }
            self.pose(ready)?;
            self.emit(Event::Phase("ripple", j));
            // `None` leaves the file's coefficients as they are: nothing was
            // measured that says otherwise.
            *slot = self.joint_ripple(j, ready[j], spans[j])?;
        }
        self.pose(ready)?;
        Ok(found)
    }

    fn joint_ripple(
        &mut self,
        j: usize,
        home: f64,
        span: (f64, f64),
    ) -> Result<Option<Vec<RippleHarmonic>>> {
        let node = self.bundle.robot.joints[j].node_id;
        let ilim = self.bundle.robot.joints[j].ilim_ma;
        let per_tick = self.per_tick(j);
        let (up, down) = (span.1 - home, home - span.0);
        let direction = if up >= down { 1.0 } else { -1.0 };
        let sweep_s = f64::from(CAPTURE_LEN) * f64::from(RIPPLE_CAPTURE_DIVISOR) / LOOP_HZ;
        let travel = RIPPLE_SWEEP_TICKS_S * sweep_s;
        if travel * per_tick.abs() > GAINS_ROOM_SHARE * up.max(down) {
            self.emit(Event::RippleNote(
                self.tick,
                j,
                "no room for the sweep at the ready pose",
            ));
            return Ok(None);
        }
        // Out along the wider side, then back from where that ended. The
        // drive runs uncompensated from here, so every way out of this stage
        // but a kept fit leaves it — and the overlay — with no ripple.
        let out = direction * per_tick.signum() * RIPPLE_SWEEP_TICKS_S;
        self.bus.set_ripple(node, &[])?;
        let start = self.pos(j)?;
        let Some(there) = self.capture_step(j, out, RIPPLE_CAPTURE_DIVISOR, span)? else {
            self.emit(Event::RippleNote(
                self.tick,
                j,
                "the drive records no capture (its firmware predates cmd 38)",
            ));
            return Ok(Some(Vec::new()));
        };
        let far = start + (out * sweep_s).round() as i32;
        if self.run_motion(j, far, RETURN_S, false)?.outcome != Outcome::Complete {
            return Err(format!("J{} did not reach the ripple sweep's far end", j + 1).into());
        }
        let back = self
            .capture_step(j, -out, RIPPLE_CAPTURE_DIVISOR, span)?
            .ok_or_else(|| format!("J{} stopped answering captures", j + 1))?;
        if self.run_motion(j, start, RETURN_S, false)?.outcome != Outcome::Complete {
            return Err(format!("J{} did not return from the ripple sweep", j + 1).into());
        }
        let skip = (RIPPLE_SETTLE_S * LOOP_HZ / f64::from(RIPPLE_CAPTURE_DIVISOR)) as usize;
        let fits = (
            ripple::fit(&there.current, &there.phase, skip, &RIPPLE_HARMONICS),
            ripple::fit(&back.current, &back.phase, skip, &RIPPLE_HARMONICS),
        );
        let (Some(there), Some(back)) = fits else {
            self.emit(Event::RippleNote(
                self.tick,
                j,
                "the sweeps do not pin the ripple down",
            ));
            return Ok(Some(Vec::new()));
        };
        let mean = |x: f64, y: f64| ((x + y) / 2.0).round().clamp(-ilim, ilim) as i16;
        let harmonics: Vec<RippleHarmonic> = there
            .iter()
            .zip(&back)
            .map(|(x, y)| RippleHarmonic {
                harmonic: x.harmonic,
                a_ma: mean(x.a, y.a),
                b_ma: mean(x.b, y.b),
            })
            .collect();
        let mut logged = [(0u8, 0i16, 0i16); 6];
        for (slot, h) in logged.iter_mut().zip(&harmonics) {
            *slot = (h.harmonic, h.a_ma, h.b_ma);
        }
        self.emit(Event::RippleFit(self.tick, j, logged));

        // Judged where the arm's captures showed the ripple: the gains step.
        let window = f64::from(CAPTURE_LEN) * f64::from(GAINS_CAPTURE_DIVISOR) / LOOP_HZ;
        let speed = GAINS_STEP_RAD_S.min(GAINS_ROOM_SHARE * up.max(down) / window);
        let step = direction * speed / per_tick;
        let skip = (RIPPLE_STEP_SKIP_S * LOOP_HZ / f64::from(GAINS_CAPTURE_DIVISOR)) as usize;
        let speed_ripple = |c: &Captured| ripple::fit(&c.speed, &c.phase, skip, &RIPPLE_HARMONICS);
        let without = self
            .capture_step(j, step, GAINS_CAPTURE_DIVISOR, span)?
            .ok_or_else(|| format!("J{} stopped answering captures", j + 1))?;
        self.bus.set_ripple(node, &harmonics)?;
        let with = self
            .capture_step(j, step, GAINS_CAPTURE_DIVISOR, span)?
            .ok_or_else(|| format!("J{} stopped answering captures", j + 1))?;
        let (Some(v0), Some(v1)) = (speed_ripple(&without), speed_ripple(&with)) else {
            self.bus.set_ripple(node, &[])?;
            self.emit(Event::RippleNote(
                self.tick,
                j,
                "the gains step does not pin the speed ripple down; cleared",
            ));
            return Ok(Some(Vec::new()));
        };
        let before = ripple::total(&v0);
        let mut best = (ripple::total(&v1), harmonics.clone());

        // At speed the ripple is not quite what the slow sweep saw. One
        // secant step per harmonic from the two step captures: how the speed
        // ripple moved for the feedforward sent, and the feedforward that
        // would null it, tried on the joint like the first.
        let refined = ripple::refine(&harmonics, &v0, &v1, ilim / RIPPLE_REFINE_ILIM_SHARE);
        if refined != harmonics {
            self.bus.set_ripple(node, &refined)?;
            let again = self
                .capture_step(j, step, GAINS_CAPTURE_DIVISOR, span)?
                .ok_or_else(|| format!("J{} stopped answering captures", j + 1))?;
            let after = speed_ripple(&again).map_or(f64::INFINITY, |v| ripple::total(&v));
            self.emit(Event::RippleRefine(self.tick, j, best.0, after));
            if after < best.0 {
                best = (after, refined);
            }
        }
        let (after, chosen) = best;
        let kept = after < before * (1.0 - RIPPLE_MIN_IMPROVEMENT);
        self.emit(Event::RippleCheck(self.tick, j, before, after, kept));
        if !kept {
            self.bus.set_ripple(node, &[])?;
            return Ok(Some(Vec::new()));
        }
        self.bus.set_ripple(node, &chosen)?;
        Ok(Some(chosen))
    }

    // ------------------------------------------------------------ gains

    /// Tune every chosen joint in turn at the ready pose and leave each
    /// accepted candidate active, so the joints that follow are tuned with
    /// the arm held the way the measurements will hold it.
    fn gains(
        &mut self,
        ready: [f64; N],
        spans: &[(f64, f64); N],
        chosen: [bool; N],
    ) -> Result<[Option<Tuned>; N]> {
        self.gain_watch = true;
        self.held_runaway = [0; N];
        self.check_startup_hold()?;
        let mut tuned = [None; N];
        for j in 0..N {
            if !chosen[j] {
                continue;
            }
            self.pose(ready)?;
            self.emit(Event::Phase("StepFOC gains", j));
            let session = GainSession {
                joint: j,
                span: spans[j],
                original: self.gains[j],
            };
            self.gain_joint = Some(j);
            let result = self.stepfoc_gains(&session);
            self.gain_joint = None;
            tuned[j] = result?;
            if let Some(result) = tuned[j] {
                self.gain_configure(j, result.after)?;
                self.check_startup_hold()?;
                self.emit(Event::Gains(self.tick, j, result));
            }
        }
        self.pose(ready)?;
        self.gain_watch = false;
        Ok(tuned)
    }

    /// Qualify `candidates` from a file on the chosen joints, without a
    /// search: the normal profile over the checked travel, the stage's speed
    /// each way and a position step each way, as a searched candidate is
    /// qualified. Nothing steps down; a candidate passes or it does not.
    fn verify_gains(
        &mut self,
        ready: [f64; N],
        spans: &[(f64, f64); N],
        chosen: [bool; N],
        candidates: [Gains; N],
    ) -> Result<[Option<Tuned>; N]> {
        self.gain_watch = true;
        self.held_runaway = [0; N];
        self.check_startup_hold()?;
        let mut tuned = [None; N];
        for j in 0..N {
            if !chosen[j] {
                continue;
            }
            self.pose(ready)?;
            self.emit(Event::Phase("verify candidate gains", j));
            let session = GainSession {
                joint: j,
                span: spans[j],
                original: self.gains[j],
            };
            let Some((speed, offset, _)) = self.gain_motion_plan(&session) else {
                self.emit(Event::GainsNote(
                    self.tick,
                    j,
                    "insufficient clear travel for gain verification; candidate not qualified",
                ));
                continue;
            };
            let candidate = candidates[j];
            self.gain_joint = Some(j);
            let outcome = (|| -> Result<bool> {
                if self.gain_verify_motion(&session, candidate)? != Sweep::Passed {
                    return Ok(false);
                }
                if self.gain_verify_speeds(&session, candidate, speed)? != Outcome::Complete {
                    return Ok(false);
                }
                self.gain_verify_positions(&session, candidate, offset)
            })();
            self.gain_joint = None;
            if outcome? {
                let result = Tuned {
                    before: session.original,
                    after: candidate,
                    observations: self.gain_used[j],
                };
                tuned[j] = Some(result);
                self.gain_configure(j, candidate)?;
                self.check_startup_hold()?;
                self.emit(Event::Gains(self.tick, j, result));
            }
        }
        self.pose(ready)?;
        self.gain_watch = false;
        Ok(tuned)
    }

    /// Every selected candidate active together on the coordinated moves the
    /// measurements make. `Some(j)` names the first `judged` joint that
    /// oscillated on a leg or would not hold at a pose, left on its
    /// configured gains; the arm is back at ready either way, retracing the
    /// checked legs it came by. A joint not judged is one the run has
    /// nothing better to offer: its faults are reported, not acted on.
    fn verify_gain_poses(
        &mut self,
        ready: [f64; N],
        poses: &[GainPose],
        judged: [bool; N],
    ) -> Result<Option<(usize, PoseFault)>> {
        let mut trail: Vec<[f64; N]> = vec![ready];
        let mut fault = None;
        'poses: for (i, pose) in poses.iter().enumerate() {
            self.emit(Event::GainPose(self.tick, i + 1, poses.len(), pose.target));
            let legs: [Option<[f64; N]>; 3] = [
                pose.from_ready.then_some(ready),
                Some(pose.approach),
                Some(pose.target),
            ];
            for to in legs.into_iter().flatten() {
                if let Some(j) = self.pose_leg(to, &mut trail, judged)? {
                    fault = Some((j, PoseFault::Motion));
                    break 'poses;
                }
            }
            // The measurements' own stillness rule, so a joint that hunts at
            // the pose is caught here and not at a gravity hold minutes in.
            if let Some(j) = self.stillness(judged)? {
                self.emit(Event::GainsNote(
                    self.tick,
                    j,
                    "did not reach stillness at a calibration pose",
                ));
                fault = Some((j, PoseFault::Hold));
                break 'poses;
            }
            if let Some(j) = self.hold_fault("calibration-pose qualification", judged)? {
                fault = Some((j, PoseFault::Hold));
                break 'poses;
            }
        }
        if let Some((j, _)) = fault {
            self.emit(Event::GainsNote(
                self.tick,
                j,
                "misbehaved on a calibration pose; retracing to ready on its configured gains",
            ));
            let original = self.bundle.robot.joints[j].gains;
            self.gain_configure(j, original)?;
            for waypoint in trail.iter().rev().skip(1) {
                self.pose(*waypoint)?;
            }
            return Ok(fault);
        }
        self.pose(ready)?;
        let fault = self.hold_fault("return from calibration-pose qualification", judged)?;
        if let Some(j) = fault {
            self.gain_configure(j, self.bundle.robot.joints[j].gains)?;
        }
        Ok(fault.map(|j| (j, PoseFault::Hold)))
    }

    /// One checked leg of the posture qualification. A joint that runs away
    /// on the way is brought back to the leg's start along the same line,
    /// the only route from there that was checked.
    fn pose_leg(
        &mut self,
        to: [f64; N],
        trail: &mut Vec<[f64; N]>,
        judged: [bool; N],
    ) -> Result<Option<usize>> {
        self.runaway_joint = None;
        match self.pose_checked(to, true) {
            Ok(reversals) => {
                trail.push(to);
                Ok(reversals.and_then(|r| (0..N).find(|&j| judged[j] && r[j] >= SWEEP_REVERSALS)))
            }
            Err(error) => match self.runaway_joint.take() {
                Some(j) if judged[j] => {
                    // Settled on its configured gains before anything moves
                    // again: a joint left in its limit cycle trips the guard
                    // on the very next leg.
                    self.calm(j, self.bundle.robot.joints[j].gains)?;
                    let back = *trail.last().ok_or("empty posture trail")?;
                    self.pose(back)?;
                    Ok(Some(j))
                }
                _ => Err(error),
            },
        }
    }

    /// The first joint that will not hold still now, by the same rule
    /// `check_hold` fails on. A joint not being judged has nothing to fall
    /// back to, so its failure to hold is the run's, not the candidate's.
    fn hold_fault(&mut self, why: &'static str, judged: [bool; N]) -> Result<Option<usize>> {
        let held = self.measure_hold(why)?;
        let fault =
            (0..N).find(|&j| held[j].0 > self.tolerance() || held[j].1 > self.holding_limit(j));
        match fault {
            Some(j) if !judged[j] => Err(format!(
                "J{} will not hold still at {why}: {:.4}deg, {:.4}deg/s",
                j + 1,
                held[j].0.to_degrees(),
                held[j].1.to_degrees()
            )
            .into()),
            other => Ok(other),
        }
    }

    /// The stage's test motion for the session's joint at the pose it holds:
    /// the speed of its pulses and constant-velocity checks \[rad/s\], the
    /// position step \[motor ticks\] and the pulse's ramp \[s\]. `None` when
    /// the checked travel leaves no room for them.
    fn gain_motion_plan(&self, session: &GainSession) -> Option<(f64, i32, f64)> {
        let j = session.joint;
        let home = self.conv[j].joint_rad(self.hold[j]);
        let room = (home - session.span.0).min(session.span.1 - home);
        let exec = self.bundle.robot.joints[j].limits.for_mode(LimitMode::Exec);
        let capture = f64::from(CAPTURE_LEN) * f64::from(GAIN_DIVISOR) / LOOP_HZ;
        let travel_s = capture.max(GAIN_RAMP_MAX_S + GAIN_STEADY_S) + RETURN_S / 10.0;
        // A cosine ramp of `ramp` seconds to `speed` peaks at pi/2 · speed/ramp;
        // the ramp is sized so that peak is the EXEC acceleration, and the
        // speed comes down when the longest ramp cannot hold it there.
        let accel = exec.acceleration_rad_s2;
        let speed = GAIN_VELOCITY_RAD_S
            .min(exec.velocity_rad_s)
            .min(GAINS_ROOM_SHARE * room / travel_s)
            .min(accel * GAIN_RAMP_MAX_S / std::f64::consts::FRAC_PI_2);
        let ramp =
            (std::f64::consts::FRAC_PI_2 * speed / accel).clamp(GAIN_RAMP_MIN_S, GAIN_RAMP_MAX_S);
        // The step is sized so that even the lattice's highest Kpp asks no
        // more than the stage's speed of it.
        let kpp_top = GainAxis::Kpp.at(GainAxis::Kpp.top());
        let distance = 2.0_f64.to_radians().min(room * 0.25).min(speed / kpp_top);
        let offset = (distance / self.per_tick(j)).round() as i32;
        if speed <= self.holding_limit(j) || room <= self.tolerance() || offset.abs() < 4 {
            return None;
        }
        Some((speed, offset, ramp))
    }

    /// StepFOC's order on one joint: Kpv, Kiv, then Kpp, each searched along
    /// its lattice with the stage's pulse or step, confirmed over the checked
    /// travel at the normal profile before the next axis is searched, and
    /// the complete candidate qualified last. Kpv keeps the guide's 20%
    /// backoff from its last stable lattice point.
    fn stepfoc_gains(&mut self, session: &GainSession) -> Result<Option<Tuned>> {
        let j = session.joint;
        let Some((speed, offset, ramp)) = self.gain_motion_plan(session) else {
            self.emit(Event::GainsNote(
                self.tick,
                j,
                "insufficient clear travel for gains tuning; gains kept",
            ));
            return Ok(None);
        };
        let pulse = GainCommand::Pulse {
            ticks_s: speed / self.per_tick(j),
            ramp_s: ramp,
        };
        self.emit(Event::GainsNote(
            self.tick,
            j,
            "velocity gain trials observe a ramped pulse and the stop after it; each axis is confirmed on the normal profile",
        ));
        let before = session.original;
        let step = GainCommand::Position(offset);
        let axes = [
            (GainAxis::Kpv, pulse, KPV_BACKOFF_STEPS),
            (GainAxis::Kiv, pulse, 0),
            (GainAxis::Kpp, step, 0),
        ];
        let mut candidate = before;
        let mut indices = [0; 3];
        for (k, (axis, command, backoff)) in axes.into_iter().enumerate() {
            let Some((found, index)) = self.gain_search(session, candidate, axis, command)? else {
                self.emit(Event::GainsNote(
                    self.tick,
                    j,
                    match axis {
                        GainAxis::Kpv => "no stable Kpv on its lattice; gains kept",
                        GainAxis::Kiv => "no stable Kiv on its lattice; gains kept",
                        GainAxis::Kpp => "no stable Kpp on its lattice; gains kept",
                    },
                ));
                return Ok(None);
            };
            let Some((confirmed, index)) =
                self.gain_sweep_down(session, found, axis, (index - backoff).max(0))?
            else {
                self.emit(Event::GainsNote(
                    self.tick,
                    j,
                    match axis {
                        GainAxis::Kpv => {
                            "Kpv failed the normal profile down to its lattice floor; gains kept"
                        }
                        GainAxis::Kiv => {
                            "Kiv failed the normal profile down to its lattice floor; gains kept"
                        }
                        GainAxis::Kpp => {
                            "Kpp failed the normal profile down to its lattice floor; gains kept"
                        }
                    },
                ));
                return Ok(None);
            };
            candidate = confirmed;
            indices[k] = index;
        }
        let Some(qualified) =
            self.gain_qualify(session, candidate, indices[0], indices[2], speed, offset)?
        else {
            return Ok(None);
        };
        Ok(Some(Tuned {
            before,
            after: qualified,
            observations: self.gain_used[j],
        }))
    }

    /// Walk `axis` along its lattice from the point nearest the candidate's
    /// value: up while the observation stays stable, or down from an
    /// unstable start until one is. The last stable lattice point and its
    /// index, or `None` when none was found before the floor or the budget.
    fn gain_search(
        &mut self,
        session: &GainSession,
        mut gains: Gains,
        axis: GainAxis,
        command: GainCommand,
    ) -> Result<Option<(Gains, i32)>> {
        let j = session.joint;
        let mut index = axis.snap(axis.value(gains));
        axis.set(&mut gains, axis.at(index));
        let Some(first) = self.gain_observe(session, gains, command, axis.label())? else {
            return Ok(None);
        };
        if first.stable {
            let mut best = (gains, index);
            while index < axis.top() {
                index += 1;
                let mut next = gains;
                axis.set(&mut next, axis.at(index));
                // A ceiling (the velocity limit, the budget) ends the walk
                // at the last point that passed.
                let Some(observed) = self.gain_observe(session, next, command, axis.label())?
                else {
                    break;
                };
                if !observed.stable {
                    break;
                }
                best = (next, index);
            }
            return Ok(Some(best));
        }
        while index > 0 {
            index -= 1;
            axis.set(&mut gains, axis.at(index));
            let Some(observed) = self.gain_observe(session, gains, command, axis.label())? else {
                return Ok(None);
            };
            if observed.stable {
                return Ok(Some((gains, index)));
            }
        }
        self.emit(Event::GainsNote(
            self.tick,
            j,
            "unstable down to the lattice floor",
        ));
        Ok(None)
    }

    /// `axis` at its lattice point `index`, over the checked travel at the
    /// normal profile both ways; while that misbehaves the axis steps one
    /// lattice point down. The gains that passed and their index, or `None`
    /// at the floor or the budget.
    fn gain_sweep_down(
        &mut self,
        session: &GainSession,
        mut gains: Gains,
        axis: GainAxis,
        mut index: i32,
    ) -> Result<Option<(Gains, i32)>> {
        loop {
            axis.set(&mut gains, axis.at(index));
            match self.gain_verify_motion(session, gains)? {
                Sweep::Passed => return Ok(Some((gains, index))),
                Sweep::Stopped => return Ok(None),
                Sweep::Failed => {
                    if index == 0 {
                        return Ok(None);
                    }
                    index -= 1;
                    self.emit(Event::GainsNote(
                        self.tick,
                        session.joint,
                        "normal motion misbehaved; one lattice step down",
                    ));
                }
            }
        }
    }

    /// The complete candidate at the stage's speed both ways and a position
    /// step both ways. An unstable speed check steps Kpv down its lattice
    /// and re-confirms the profile; a failed step check steps Kpp down.
    /// What passes within the step allowance is the joint's result.
    fn gain_qualify(
        &mut self,
        session: &GainSession,
        mut gains: Gains,
        mut kpv_index: i32,
        mut kpp_index: i32,
        speed: f64,
        offset: i32,
    ) -> Result<Option<Gains>> {
        let j = session.joint;
        for _ in 0..=GAIN_QUALIFY_STEPS {
            match self.gain_verify_speeds(session, gains, speed)? {
                Outcome::Complete => {}
                Outcome::Unstable => {
                    if kpv_index == 0 {
                        self.emit(Event::GainsNote(
                            self.tick,
                            j,
                            "unstable at the stage's speed down to the Kpv lattice floor; gains kept",
                        ));
                        return Ok(None);
                    }
                    kpv_index -= 1;
                    GainAxis::Kpv.set(&mut gains, GainAxis::Kpv.at(kpv_index));
                    self.emit(Event::GainsNote(
                        self.tick,
                        j,
                        "speed check unstable; Kpv one lattice step down, normal profile re-confirmed",
                    ));
                    if self.gain_verify_motion(session, gains)? != Sweep::Passed {
                        return Ok(None);
                    }
                    continue;
                }
                // Tracking too poor for the requirement, or nothing more to
                // observe: no lower gain fixes either.
                _ => return Ok(None),
            }
            if self.gain_verify_positions(session, gains, offset)? {
                return Ok(Some(gains));
            }
            if kpp_index == 0 {
                self.emit(Event::GainsNote(
                    self.tick,
                    j,
                    "position step failed down to the Kpp lattice floor; gains kept",
                ));
                return Ok(None);
            }
            kpp_index -= 1;
            GainAxis::Kpp.set(&mut gains, GainAxis::Kpp.at(kpp_index));
            self.emit(Event::GainsNote(
                self.tick,
                j,
                "position check failed; Kpp one lattice step down, normal profile re-confirmed",
            ));
            if self.gain_verify_motion(session, gains)? != Sweep::Passed {
                return Ok(None);
            }
        }
        self.emit(Event::GainsNote(
            self.tick,
            j,
            "qualification did not settle within its step allowance; gains kept",
        ));
        Ok(None)
    }

    /// The stage's speed each way at `gains`: stable, and tracking within
    /// the moving requirement once the ripple the drive has not yet been
    /// told about is set aside.
    fn gain_verify_speeds(
        &mut self,
        session: &GainSession,
        gains: Gains,
        speed: f64,
    ) -> Result<Outcome> {
        let j = session.joint;
        for fraction in [0.5, -0.5] {
            let Some(observed) = self.gain_observe(
                session,
                gains,
                GainCommand::Velocity(fraction * speed / self.per_tick(j)),
                "verify speed",
            )?
            else {
                return Ok(Outcome::Timeout);
            };
            if !observed.stable {
                return Ok(Outcome::Unstable);
            }
            if observed.mean_error.hypot(observed.ripple) > MOVING_RMS_RAD_S {
                self.emit(Event::GainsNote(
                    self.tick,
                    j,
                    "velocity gains miss the speed tracking requirement; gains kept",
                ));
                return Ok(Outcome::Timeout);
            }
        }
        Ok(Outcome::Complete)
    }

    /// A position step each way at `gains`, each settling without overshoot
    /// or hunting.
    fn gain_verify_positions(
        &mut self,
        session: &GainSession,
        gains: Gains,
        offset: i32,
    ) -> Result<bool> {
        for direction in [1, -1] {
            let observed = self.gain_observe(
                session,
                gains,
                GainCommand::Position(direction * offset),
                "verify position and hold",
            )?;
            if !observed.is_some_and(|o| o.stable) {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// Sweep the checked travel both ways at the normal profile under
    /// `gains`. A quiet low-speed pulse does not establish stability at
    /// speed: J4's loop held at 40 deg/s and hunted at 55 Hz above 150.
    fn gain_verify_motion(&mut self, session: &GainSession, gains: Gains) -> Result<Sweep> {
        let j = session.joint;
        if self.gain_used[j] + 2 > GAIN_OBSERVATIONS {
            self.emit(Event::GainsNote(
                self.tick,
                j,
                "observation budget exhausted before motion verification",
            ));
            return Ok(Sweep::Stopped);
        }
        let start = self.pos(j)?;
        let low = self.conv[j].motor_ticks(session.span.0);
        let high = self.conv[j].motor_ticks(session.span.1);
        self.return_to(j, low)?;
        self.sane_gains[j] = Some(session.original);
        let result: Result<Sweep> = (|| {
            self.gain_configure(j, gains)?;
            for target in [high, low] {
                self.gain_used[j] += 1;
                let measured = self.run_motion(j, target, self.dt, false)?;
                let verdict = sweep_verdict(&measured, self.holding_limit(j), self.tolerance());
                let lag_ms = if measured.peak_command_rad_s > 0.0 {
                    1000.0 * measured.peak_error_rad / measured.peak_command_rad_s
                } else {
                    0.0
                };
                self.emit(Event::Sweep(
                    self.tick,
                    j,
                    gains,
                    measured.reversals,
                    measured.hold_rms_rad_s,
                    lag_ms,
                    verdict.label(),
                ));
                if verdict != Sweep::Passed {
                    return Ok(verdict);
                }
            }
            Ok(Sweep::Passed)
        })();
        let restored = self.gain_restore(j, session.original);
        self.sane_gains[j] = None;
        restored?;
        let verdict = result?;
        self.calm(j, session.original)?;
        self.return_to(j, start)?;
        Ok(verdict)
    }

    /// Every observation restores the original gains before downloading or
    /// returning. This includes failed captures and control-loop errors.
    /// `None` when the joint could not be observed at these gains for a
    /// reason that is not its stability: the budget, or a step the velocity
    /// limit would clip.
    fn gain_observe(
        &mut self,
        session: &GainSession,
        gains: Gains,
        command: GainCommand,
        label: &'static str,
    ) -> Result<Option<GainObservation>> {
        let j = session.joint;
        if self.gain_used[j] >= GAIN_OBSERVATIONS {
            self.emit(Event::GainsNote(
                self.tick,
                j,
                "observation budget exhausted",
            ));
            return Ok(None);
        }
        let start = self.pos(j)?;
        let per_tick = self.per_tick(j);
        let limits = self.bundle.robot.joints[j].limits.for_mode(LimitMode::Exec);
        let initial_speed = match command {
            GainCommand::Velocity(v) | GainCommand::Pulse { ticks_s: v, .. } => {
                (v * per_tick).abs()
            }
            GainCommand::Position(offset) => (f64::from(offset) * per_tick * gains.kpp).abs(),
        };
        if initial_speed > limits.velocity_rad_s {
            self.emit(Event::GainsNote(
                self.tick,
                j,
                "the step would exceed the velocity limit; lattice ceiling",
            ));
            return Ok(None);
        }
        self.gain_used[j] += 1;
        self.adopt(j)?;
        self.sane_gains[j] = Some(session.original);
        let result = (|| {
            self.gain_configure(j, gains)?;
            self.gain_record(session, command, start)
        })();
        let restored = self.gain_restore(j, session.original);
        self.sane_gains[j] = None;
        restored?;
        let (overshoot, position_error, stopped) = result?;
        if stopped {
            self.calm(j, session.original)?;
            self.return_to(j, start)?;
            let observed = GainObservation {
                stable: false,
                ripple: f64::INFINITY,
                reversals: 0,
                overshoot,
                error: f64::INFINITY,
                mean_error: f64::INFINITY,
            };
            self.emit(Event::GainObservation(self.tick, j, gains, label, observed));
            return Ok(Some(observed));
        }
        let node = self.bundle.robot.joints[j].node_id;
        let divisor = command.divisor();
        let phase_capture = !matches!(command, GainCommand::Position(_));
        let channels: &[u8] = if phase_capture { &[0, 1, 2] } else { &[0, 1] };
        if !self.fetch_capture(j, channels, divisor)? {
            return Err(format!("J{} did not supply encoder capture", j + 1).into());
        }
        let record = self
            .bus
            .capture(node)
            .ok_or("encoder capture disappeared")?;
        let captured = Captured {
            speed: record
                .velocity
                .iter()
                .map(|&v| f64::from(i32::from(v) * CAPTURE_VEL_SCALE))
                .collect(),
            current: record.current.iter().map(|&i| f64::from(i)).collect(),
            phase: if phase_capture {
                record.phase.iter().map(|&p| f64::from(p)).collect()
            } else {
                Vec::new()
            },
            divisor: usize::from(divisor),
        };
        let step = match command {
            GainCommand::Velocity(v) | GainCommand::Pulse { ticks_s: v, .. } => v,
            GainCommand::Position(_) => 0.0,
        };
        let rate = LOOP_HZ / f64::from(divisor);
        // Where each verdict is read: the stop after a pulse or a step, the
        // steady stretch of a pulse, the whole of a constant-velocity run
        // once it has settled.
        let (tail_from, moving) = match command {
            GainCommand::Pulse { ramp_s, .. } => (
                2.0 * ramp_s + GAIN_STEADY_S + GAIN_TAIL_SETTLE_S,
                Some((ramp_s + GAIN_RAMP_SETTLE_S, ramp_s + GAIN_STEADY_S)),
            ),
            _ => (GAIN_SETTLE_S, None),
        };
        self.write_capture(
            j,
            step,
            gains,
            &captured,
            match command {
                GainCommand::Pulse { ramp_s, .. } => Some(2.0 * ramp_s + GAIN_STEADY_S),
                _ => None,
            },
        );
        let n = captured.speed.len();
        let skip = ((tail_from * rate) as usize).min(n.saturating_sub(8));
        let tail = &captured.speed[skip..];
        // Hysteresis rejects sign changes caused by one quantized near-zero
        // sample. Four reversals require repeated motion, not step overshoot.
        // Moving ripple is judged against the movement requirement; applying
        // the stationary limit there rejects the normal commutation ripple.
        let limit = match command {
            GainCommand::Velocity(_) => MOVING_RMS_RAD_S,
            GainCommand::Pulse { .. } | GainCommand::Position(_) => self.holding_limit(j),
        };
        let reference = if matches!(command, GainCommand::Velocity(_)) {
            step
        } else {
            0.0
        };
        let residual = matches!(command, GainCommand::Velocity(_))
            .then(|| gain_moving_residual(tail, &captured.phase[skip..]));
        let (mut ripple, mut reversals) = gain_ripple(
            residual.as_deref().unwrap_or(tail),
            reference,
            per_tick,
            limit,
        );
        // Variation beyond the limit is instability on whichever side of the
        // reference it sits; the crossings are reported, not judged.
        let mut stable = ripple <= limit;
        if let Some((from_s, to_s)) = moving {
            let from = ((from_s * rate).ceil() as usize).min(n);
            let to = ((to_s * rate).floor() as usize).min(n);
            if to > from + 8 {
                let residual =
                    gain_moving_residual(&captured.speed[from..to], &captured.phase[from..to]);
                let (moving, crossings) = gain_ripple(&residual, step, per_tick, MOVING_RMS_RAD_S);
                let moving_stable = moving <= MOVING_RMS_RAD_S;
                // Report the failing window, or the larger variation if both pass.
                if stable && (!moving_stable || moving > ripple) {
                    ripple = moving;
                    reversals = crossings;
                }
                stable &= moving_stable;
            }
        }
        let error = match command {
            GainCommand::Velocity(_) | GainCommand::Pulse { .. } => {
                (tail.iter().map(|x| (x - reference).powi(2)).sum::<f64>() / tail.len() as f64)
                    .sqrt()
                    * per_tick.abs()
            }
            GainCommand::Position(_) => position_error,
        };
        let mean_error = match command {
            GainCommand::Velocity(_) | GainCommand::Pulse { .. } => {
                (tail.iter().sum::<f64>() / tail.len() as f64 - reference).abs() * per_tick.abs()
            }
            GainCommand::Position(_) => position_error,
        };
        if let GainCommand::Position(offset) = command {
            let allowed = (2.0 * per_tick.abs())
                .max(GAIN_OVERSHOOT_FRACTION * (f64::from(offset) * per_tick).abs());
            stable = stable && overshoot <= allowed && position_error <= self.tolerance();
        }
        let observed = GainObservation {
            stable,
            ripple,
            reversals,
            overshoot,
            error,
            mean_error,
        };
        self.return_to(j, start)?;
        self.emit(Event::GainObservation(self.tick, j, gains, label, observed));
        Ok(Some(observed))
    }

    fn gain_record(
        &mut self,
        session: &GainSession,
        command: GainCommand,
        start: i32,
    ) -> Result<(f64, f64, bool)> {
        let j = session.joint;
        let node = self.bundle.robot.joints[j].node_id;
        let per_tick = self.per_tick(j);
        let target = match command {
            GainCommand::Position(offset) => {
                start.checked_add(offset).ok_or("position step overflow")?
            }
            _ => start,
        };
        let direction = (i64::from(target) - i64::from(start)).signum() as f64;
        let divisor = command.divisor();
        let duration = f64::from(CAPTURE_LEN) * f64::from(divisor) / LOOP_HZ;
        self.bus.capture_start(node, divisor, CAPTURE_LEN)?;
        let mut overshoot: f64 = 0.0;
        let mut runaway = 0;
        let mut generation = self.generation[j];
        for t in 0..self.ticks(duration) + 2 {
            let velocity = match command {
                GainCommand::Velocity(v) => v,
                GainCommand::Pulse { ticks_s, ramp_s } => {
                    ticks_s * pulse_shape(t as f64 * self.dt, ramp_s)
                }
                GainCommand::Position(_) => 0.0,
            };
            let frame = match command {
                GainCommand::Velocity(_) | GainCommand::Pulse { .. } => {
                    JointCommand::velocity(velocity.round() as i32, self.gravity_feedforward()[j])
                }
                GainCommand::Position(_) => {
                    JointCommand::position(target, 0, self.gravity_feedforward()[j])
                }
            };
            self.frame(Some((j, frame)))?;
            let actual = self.pos(j)?;
            overshoot =
                overshoot.max((f64::from(actual) - f64::from(target)) * direction * per_tick.abs());
            let angle = self.conv[j].joint_rad(actual);
            if !(session.span.0..=session.span.1).contains(&angle) {
                return Err(format!("J{} reached its tuning travel bound", j + 1).into());
            }
            if self.generation[j] == generation {
                continue;
            }
            generation = self.generation[j];
            let speed = f64::from(
                self.state.nodes[self.node(j)]
                    .speed_ticks_s
                    .ok_or("missing velocity feedback")?,
            );
            let expected = match command {
                GainCommand::Velocity(_) | GainCommand::Pulse { .. } => velocity,
                GainCommand::Position(_) => {
                    ((f64::from(target) - f64::from(actual)) * self.gains[j].kpp).clamp(
                        -self.bundle.robot.joints[j].velocity_limit_ticks_s,
                        self.bundle.robot.joints[j].velocity_limit_ticks_s,
                    )
                }
            };
            let velocity_error = (speed - expected) * per_tick;
            runaway = if velocity_error.abs() > RUNAWAY_RAD_S {
                runaway + 1
            } else {
                0
            };
            if runaway >= RUNAWAY_TICKS {
                self.emit(Event::GainsNote(
                    self.tick,
                    j,
                    "runaway speed guard; restoring original gains",
                ));
                return Ok((overshoot, f64::INFINITY, true));
            }
        }
        let error = ((f64::from(self.pos(j)?) - f64::from(target)) * per_tick).abs();
        Ok((overshoot, error, false))
    }

    fn gain_configure(&mut self, j: usize, gains: Gains) -> Result<()> {
        self.gains[j] = gains;
        self.retune(j, self.bundle.robot.joints[j].ilim_ma)?;
        // Changing gains does not clear the velocity accumulator. Returning
        // from current mode to position mode does; catch at the actual pose.
        let gravity = self.gravity_feedforward()[j];
        self.frame(Some((j, JointCommand::current(gravity))))?;
        self.adopt(j)?;
        self.configure(j, self.bundle.robot.joints[j].ilim_ma)
    }

    /// Finish transmitting recovery gains even when a stop was requested.
    /// The cancellation flag stays set so calibration cannot resume afterward.
    fn gain_restore(&mut self, j: usize, gains: Gains) -> Result<()> {
        let stopping = std::mem::replace(&mut self.stopping, true);
        let restoring = std::mem::replace(&mut self.restoring, true);
        // A trial that ended on a missed deadline would otherwise fail every
        // restore frame on that same deadline and leave its gains installed.
        if !self.simulated {
            self.deadline = runtime::now();
        }
        let restored = self.gain_configure(j, gains);
        self.stopping = stopping;
        self.restoring = restoring;
        restored
    }

    /// The current feedforward for joint `j`: gravity at the measured pose,
    /// within the joint's current limit \[mA\].
    fn feedforward(&mut self, j: usize) -> i16 {
        let gravity = f64::from(self.gravity_feedforward()[j]);
        let ilim = self.bundle.robot.joints[j].ilim_ma;
        gravity.clamp(-ilim, ilim).round() as i16
    }

    /// A trial ran away: the known-good gains go back on the drive and the
    /// joint holds where it is. If it stays loud, its loop is opened -- the
    /// gravity current alone, with no feedback, cannot oscillate -- then
    /// closed again from rest. A drive can sit in a large-signal limit cycle
    /// at gains that hold and sweep cleanly from rest (J1 at ~100 Hz,
    /// ±0.5 deg, ±1 A for as long as it was held, 2026-10-02), so settling
    /// that way is a recovery, not a verdict on the gains, and the run goes
    /// on. Staying loud after the loop was reopened stops the run: something
    /// the stage relied on did not hold.
    fn calm(&mut self, j: usize, sane: Gains) -> Result<()> {
        let ilim = self.bundle.robot.joints[j].ilim_ma;
        self.adopt(j)?;
        self.gains[j] = sane;
        // The joint may still be spinning down from the trial just withdrawn
        // (J1's base read 117 deg/s 72 ms after one, 2026-10-01), which the
        // tracking guard would call a second runaway and end the run on.
        // These holds are the judge of that decay, so the guard stands down
        // for them.
        let guard = self.sane_gains[j].replace(sane);
        let settled = (|| {
            self.configure(j, ilim)?;
            self.quiet(j)
        })();
        self.sane_gains[j] = guard;
        if settled? {
            return Ok(());
        }
        self.emit(Event::GainsNote(
            self.tick,
            j,
            "still oscillating on its known-good gains; opening its loop",
        ));
        let feedforward = self.gravity_feedforward()[j];
        let per_tick = self.per_tick(j);
        let opened_at = self.pos(j)?;
        for _ in 0..self.ticks(CALM_S) {
            self.frame(Some((j, JointCommand::current(feedforward))))?;
            // No loop bounds the joint now; the gravity current is a model,
            // and a sag just under the runaway speed would cover a lot of
            // ground in the time it is open.
            let reported = self.state.nodes[self.node(j)].speed_ticks_s.unwrap_or(0);
            let drifted = f64::from(self.pos(j)? - opened_at) * per_tick;
            if (f64::from(reported) * per_tick).abs() > RUNAWAY_RAD_S
                || drifted.abs() > CALM_OPEN_TRAVEL_RAD
            {
                self.adopt(j)?;
                return Err(format!(
                    "J{} ran away with its loop open after its trial gains were withdrawn",
                    j + 1
                )
                .into());
            }
        }
        self.adopt(j)?;
        let guard = self.sane_gains[j].replace(sane);
        let settled = self.quiet(j);
        self.sane_gains[j] = guard;
        if settled? {
            self.emit(Event::GainsNote(
                self.tick,
                j,
                "settled once its loop was opened and closed again; continuing",
            ));
            return Ok(());
        }
        Err(format!(
            "J{} kept oscillating after its trial gains were withdrawn and its loop was opened",
            j + 1
        )
        .into())
    }

    /// Hold joint `j` for up to `CALM_S`: whether its reported speed stayed
    /// under `RUNAWAY_RAD_S` once the first `CALM_QUIET_S`, which a withdrawn
    /// trial may still be spinning down through, had passed. A loud reading
    /// after that ends the hold at once: a limit cycle does not decay on its
    /// own, and every tick of it hammers the joint.
    fn quiet(&mut self, j: usize) -> Result<bool> {
        let per_tick = self.per_tick(j);
        let (total, grace) = (self.ticks(CALM_S), self.ticks(CALM_QUIET_S));
        for t in 0..total {
            self.frame(None)?;
            let reported = self.state.nodes[self.node(j)].speed_ticks_s.unwrap_or(0);
            if (f64::from(reported) * per_tick).abs() > RUNAWAY_RAD_S && t >= grace {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// Capture a ripple trial inside the collision-checked travel. Failure
    /// withdraws compensation before any download or return motion resumes.
    fn capture_step(
        &mut self,
        j: usize,
        step: f64,
        divisor: u8,
        span: (f64, f64),
    ) -> Result<Option<Captured>> {
        let node = self.bundle.robot.joints[j].node_id;
        let per_tick = self.per_tick(j);
        let tried = self.gains[j];
        let result = (|| {
            let start = self.pos(j)?;
            let window = f64::from(CAPTURE_LEN) * f64::from(divisor) / LOOP_HZ;
            self.bus.capture_start(node, divisor, CAPTURE_LEN)?;
            let mut runaway_ticks = 0u32;
            let mut generation = self.generation[j];
            let stepping = self.ticks(window) + 2;
            for t in 0..stepping + self.ticks(RETURN_S / 10.0) {
                let command = if t < stepping { step } else { 0.0 };
                let feedforward = self.feedforward(j);
                self.frame(Some((
                    j,
                    JointCommand::velocity(command.round() as i32, feedforward),
                )))?;
                let angle = self.conv[j].joint_rad(self.pos(j)?);
                if !(span.0..=span.1).contains(&angle) {
                    return Err(format!("J{} reached its ripple travel bound", j + 1).into());
                }
                if self.generation[j] == generation {
                    continue;
                }
                generation = self.generation[j];
                let reported = self.state.nodes[self.node(j)]
                    .speed_ticks_s
                    .ok_or("missing ripple velocity feedback")?;
                let error = (f64::from(reported) - command) * per_tick;
                runaway_ticks = if error.abs() > RUNAWAY_RAD_S {
                    runaway_ticks + 1
                } else {
                    0
                };
                if runaway_ticks >= RUNAWAY_TICKS {
                    return Err(format!("J{} ripple capture ran away", j + 1).into());
                }
            }
            self.adopt(j)?;
            if !self.fetch_capture(j, &[0, 1, 2], divisor)? {
                self.return_to(j, start)?;
                return Ok(None);
            }
            let capture = self
                .bus
                .capture(node)
                .ok_or_else(|| format!("J{} has no capture buffer", j + 1))?;
            let n = usize::from(capture.recorded);
            let captured = Captured {
                divisor: usize::from(capture.divisor),
                current: capture.current[..n].iter().map(|&i| f64::from(i)).collect(),
                phase: capture.phase[..n].iter().map(|&p| f64::from(p)).collect(),
                speed: capture.velocity[..n]
                    .iter()
                    .map(|&v| f64::from(i32::from(v) * CAPTURE_VEL_SCALE))
                    .collect(),
            };
            self.write_capture(j, step, tried, &captured, None);
            self.return_to(j, start)?;
            Ok(Some(captured))
        })();
        if result.is_err() {
            let joint = &self.bundle.robot.joints[j];
            let gains = joint.gains;
            let restored_ripple = self.bus.set_ripple(node, &joint.ripple);
            let restored_gains = self.gain_restore(j, gains);
            restored_ripple?;
            restored_gains?;
        }
        result
    }

    /// Read joint `j`'s capture back over the poll slot: its status until it
    /// answers, then every chunk of `channels`, again for any a lossy bus
    /// dropped. `false` when the drive answers no status.
    fn fetch_capture(&mut self, j: usize, channels: &[u8], divisor: u8) -> Result<bool> {
        let node = self.bundle.robot.joints[j].node_id;
        self.emit(Event::Phase("capture download", j));
        let mut answered = false;
        for _ in 0..GAINS_STATUS_TRIES {
            self.read_capture(node, CAPTURE_STATUS_CHANNEL, 0)?;
            self.settle(self.dt)?;
            if self.bus.capture(node).is_some_and(|c| c.wanted > 0) {
                answered = true;
                break;
            }
        }
        if !answered {
            return Ok(false);
        }
        let capture = self.bus.capture(node).ok_or("capture status disappeared")?;
        if capture.recorded != CAPTURE_LEN
            || capture.wanted != CAPTURE_LEN
            || capture.divisor != u16::from(divisor)
        {
            return Err(format!("J{} capture has the wrong length or sample rate", j + 1).into());
        }
        // The whole buffer on one request, paced by the drive; the reads
        // below then fill only what the bus dropped. A drive without the
        // stream command sends nothing and the reads fetch everything.
        self.bus.capture_stream(node)?;
        for _ in 0..self.ticks(CAPTURE_STREAM_WAIT_S) {
            self.frame(None)?;
            let complete = self.bus.capture(node).is_some_and(|capture| {
                channels
                    .iter()
                    .all(|channel| capture.missing(*channel).next().is_none())
            });
            if complete {
                break;
            }
        }
        for _ in 0..GAINS_READ_PASSES {
            let missing: Vec<(u8, u16)> = channels
                .iter()
                .copied()
                .flat_map(|channel| {
                    let capture = self.bus.capture(node);
                    capture
                        .map(|c| {
                            c.missing(channel)
                                .map(|chunk| (channel, chunk))
                                .collect::<Vec<_>>()
                        })
                        .unwrap_or_default()
                })
                .collect();
            if missing.is_empty() {
                break;
            }
            for (channel, chunk) in missing {
                self.read_capture(node, channel, chunk)?;
            }
            self.settle(2.0 * self.dt)?;
        }
        let capture = self
            .bus
            .capture(node)
            .ok_or_else(|| format!("J{} has no capture buffer", j + 1))?;
        if channels
            .iter()
            .any(|c| capture.missing(*c).next().is_some())
        {
            return Err(format!(
                "J{} capture incomplete after {GAINS_READ_PASSES} reads",
                j + 1
            )
            .into());
        }
        Ok(true)
    }

    /// Leave a capture in the run directory as `capture-J<j>-<n>.csv`:
    /// sample, time \[s\], speed \[ticks/s\], Iq \[mA\], electrical phase
    /// (0..16383, when captured), with the step
    /// \[ticks/s\] and divisor in the header. Best effort, like every other
    /// record.
    fn write_capture(
        &mut self,
        j: usize,
        step: f64,
        tried: Gains,
        captured: &Captured,
        stop_after: Option<f64>,
    ) {
        if self.run_directory.is_none() {
            return;
        }
        self.captures_written += 1;
        self.emit(Event::Capture(
            j,
            self.captures_written,
            step,
            tried,
            captured.clone(),
            stop_after,
        ));
    }

    /// One capture read in this tick's poll slot.
    fn read_capture(&mut self, node: u8, channel: u8, chunk: u16) -> Result<()> {
        self.bus.queue_poll_override(
            PollAction::CaptureRead {
                node,
                channel,
                chunk,
            },
            1,
        );
        self.frame(None)
    }

    /// The fastest each joint moves while still meeting the tracking
    /// requirements, found joint by joint: each carries its own inertia and
    /// gets its own limits. A joint's EXEC caps are scaled together by
    /// `LIMITS_STEP` towards its hardware ceiling until a step fails; when
    /// EXEC itself fails, the factor steps down instead. Opt-in
    /// (`--limits`): it drives every joint to the edge of what it can do,
    /// which an ordinary run has no reason to.
    ///
    /// A step passes when every probe move keeps its following error, landing
    /// and hold within the recorded requirements (`Measure::score`) and its
    /// peak dynamic current, plus the most gravity current the joint carries
    /// at any identification pose, fits under the current limit -- so the
    /// limit holds anywhere in the workspace, not just around the ready pose
    /// it was measured at.
    ///
    /// Speed ripple is reported rather than judged against a requirement
    /// (user decision 2026-09-23): it does not grow with the caps, so it
    /// cannot tell one step from the next. On that day's run J4 carried
    /// 6.1-8.4 deg/s at every step from 1.0 down to 0.4 of EXEC while its
    /// following error passed at each. Only `LIMITS_RIPPLE_CEILING` bounds
    /// it, as a sanity cap on a joint that is rumbling rather than tracking.
    fn limits(
        &mut self,
        poses: &[[f64; N]],
        ready: [f64; N],
        spans: &[(f64, f64); N],
        chosen: [bool; N],
    ) -> Result<[Option<Found>; N]> {
        let worst = self.worst_gravity_ma(poses, ready)?;
        let mut found = [None; N];
        for (j, slot) in found.iter_mut().enumerate() {
            if !chosen[j] {
                continue;
            }
            self.pose(ready)?;
            self.emit(Event::Phase("limits", j));
            *slot = self.joint_limits(j, ready[j], spans[j], worst[j])?;
            match *slot {
                Some(f) => self.emit(Event::Limits(self.tick, j, f)),
                None => self.emit(Event::Phase(
                    "limits: misses the requirements even at the lowest factor",
                    j,
                )),
            }
        }
        self.pose(ready)?;
        Ok(found)
    }

    /// Per joint, the most gravity current it carries at the ready pose or
    /// any identification pose \[mA\].
    fn worst_gravity_ma(&mut self, poses: &[[f64; N]], ready: [f64; N]) -> Result<[f64; N]> {
        let mut worst = [0.0_f64; N];
        let mut tau = [0.0; N];
        for q in poses.iter().chain(std::iter::once(&ready)) {
            self.kin.gravity(q, &mut tau)?;
            for (j, w) in worst.iter_mut().enumerate() {
                let joint = &self.bundle.robot.joints[j];
                let ma_per_nm = torque_to_ma_factor(
                    joint.gear_ratio,
                    joint.gear_efficiency,
                    joint.kt_nm_a,
                    joint.dir,
                );
                *w = f64::max(*w, (tau[j] * ma_per_nm).abs());
            }
        }
        Ok(worst)
    }

    /// The search for one joint, between `span` \[rad\] around `home`.
    fn joint_limits(
        &mut self,
        j: usize,
        home: f64,
        span: (f64, f64),
        worst_ma: f64,
    ) -> Result<Option<Found>> {
        let exec = self.exec_caps(j);
        let ceiling = self.ceiling_caps(j);
        let mut k = 1.0;
        let mut caps = exec.scaled(k, &ceiling);
        let Some(mut best) = self.probe(j, home, span, caps, worst_ma)? else {
            loop {
                k /= LIMITS_STEP;
                if k < LIMITS_MIN_FACTOR {
                    return Ok(None);
                }
                caps = exec.scaled(k, &ceiling);
                if let Some(found) = self.probe(j, home, span, caps, worst_ma)? {
                    return Ok(Some(Found {
                        jerk_measured: true,
                        ..found
                    }));
                }
            }
        };
        loop {
            let next = exec.scaled(k * LIMITS_STEP, &ceiling);
            // Nothing left to raise: every cap is at its ceiling, or the ones
            // that are not no longer bind either probe move.
            if self.probe_ticks(home, span, next) == self.probe_ticks(home, span, caps) {
                best.jerk_measured = best.caps.jerk >= ceiling.jerk;
                return Ok(Some(best));
            }
            k *= LIMITS_STEP;
            caps = next;
            match self.probe(j, home, span, caps, worst_ma)? {
                Some(found) => best = found,
                None => {
                    best.jerk_measured = true;
                    return Ok(Some(best));
                }
            }
        }
    }

    /// Where the short probe move goes, and the distance at which speed
    /// starts to bind a move under `caps` \[rad\].
    fn probe_short(&self, home: f64, span: (f64, f64), caps: Caps) -> (f64, f64) {
        let binds_at =
            SEPTIC_PEAK_ACC * caps.velocity.powi(2) / (SEPTIC_PEAK_VEL.powi(2) * caps.acceleration);
        let (up, down) = (span.1 - home, home - span.0);
        let short = (LIMITS_SHORT_FRACTION * binds_at).min(up.max(down));
        (
            if up >= down {
                home + short
            } else {
                home - short
            },
            binds_at,
        )
    }

    /// The two probe moves' durations under `caps` \[ticks\].
    fn probe_ticks(&self, home: f64, span: (f64, f64), caps: Caps) -> (u64, u64) {
        let (short_target, _) = self.probe_short(home, span, caps);
        let short = caps
            .profile((short_target - home).abs(), self.dt, self.dt)
            .duration();
        let long = caps.profile(span.1 - span.0, self.dt, self.dt).duration();
        (self.ticks(short), self.ticks(long))
    }

    /// One step: a short move out and back, where acceleration and jerk set
    /// the duration, then the full span both ways, where speed does when
    /// the span is long enough. `Some` when every move passed, with the jerk
    /// not yet counted as measured; the joint is back at `home` either way.
    fn probe(
        &mut self,
        j: usize,
        home: f64,
        span: (f64, f64),
        caps: Caps,
        worst_ma: f64,
    ) -> Result<Option<Found>> {
        let (short_target, binds_at) = self.probe_short(home, span, caps);
        // Getting to the start of the long move is positioning, not probing.
        let moves = [
            (short_target, true),
            (home, true),
            (span.0, false),
            (span.1, true),
            (span.0, true),
        ];
        let mut ripple = 0.0_f64;
        for (target, scored) in moves {
            let ticks = self.conv[j].motor_ticks(target);
            if !scored {
                self.run_motion(j, ticks, self.dt, false)?;
                continue;
            }
            let mut m = self.run_motion_capped(j, ticks, self.dt, false, caps)?;
            m.score(true, f64::INFINITY, self.holding_limit(j), self.tolerance());
            if m.peak_command_rad_s > 0.0 {
                ripple = ripple.max(m.rms_speed() / m.peak_command_rad_s);
            }
            let ilim = self.bundle.robot.joints[j].ilim_ma;
            let why = if m.outcome != Outcome::Complete {
                Some("did not complete")
            } else if m.excess > 0.0 {
                Some("tracking")
            } else if m.peak_dynamic_ma + worst_ma > ilim {
                Some("current")
            } else if ripple > LIMITS_RIPPLE_CEILING {
                Some("ripple")
            } else {
                None
            };
            if let Some(why) = why {
                self.emit(Event::LimitsStep(self.tick, j, caps, Some(why)));
                self.run_motion(j, self.conv[j].motor_ticks(home), self.dt, false)?;
                return Ok(None);
            }
        }
        self.run_motion(j, self.conv[j].motor_ticks(home), self.dt, false)?;
        self.emit(Event::LimitsStep(self.tick, j, caps, None));
        let long = span.1 - span.0;
        let t_speed = SEPTIC_PEAK_VEL * long / caps.velocity;
        let velocity_reached = long >= binds_at
            && t_speed + self.dt >= caps.profile(long, self.dt, self.dt).duration();
        Ok(Some(Found {
            caps,
            velocity_reached,
            jerk_measured: false,
            ripple,
            configured: self.exec_caps(j),
        }))
    }
}

/// Ready is covered by the bidirectional joint probes. Add the extended
/// stiction pose and the first identification transition: a ready-only
/// qualification missed J4's instability at the second identification pose.
/// This is representative coverage, not certification of every arm pose.
fn gain_pose_plan(
    bundle: &ConfigBundle,
    assets: &Path,
    ready: [f64; N],
    identification: &[[f64; N]],
    out: Option<[f64; N]>,
) -> Result<Vec<GainPose>> {
    let mut poses = Vec::with_capacity(3);
    if let Some(target) = out {
        poses.push(GainPose {
            from_ready: true,
            approach: ready,
            target,
        });
    }
    let backoff = bundle.robot.selfcal.approach_rad;
    // For each joint, the plan poses where it carries the least and the most
    // inertia: a loop tuned at ready runs hot where its load falls away and
    // slack where it grows, and J1 hunted at pose 9 with the arm stacked
    // over the base, 3 cm off its axis, while the two nearest poses had
    // passed it (2026-10-01).
    let mut kin = arm_kin(bundle, assets)?;
    let inertias: Vec<[f64; N]> = identification
        .iter()
        .map(|q| joint_inertia(&mut kin, bundle, *q))
        .collect::<Result<_>>()?;
    let mut heaviest: Vec<usize> = Vec::with_capacity(2 * N);
    let columns: [Vec<f64>; N] =
        std::array::from_fn(|j| inertias.iter().map(|row| row[j]).collect());
    for column in &columns {
        let extremes = [
            (0..identification.len()).min_by(|a, b| column[*a].total_cmp(&column[*b])),
            (0..identification.len()).max_by(|a, b| column[*a].total_cmp(&column[*b])),
        ];
        for i in extremes.into_iter().flatten() {
            if !heaviest.contains(&i) {
                heaviest.push(i);
            }
        }
    }
    heaviest.sort_unstable();
    let mut world = collision_world(bundle, assets)?;
    let mut at = ready;
    if let Some(pose) = poses.first() {
        if world.check_segment(&at, &pose.target, 40)?.is_some() {
            return Err("the arm-out pose is not reachable from ready".into());
        }
        at = pose.target;
    }
    for i in heaviest {
        let target = identification[i];
        let approach: [f64; N] = std::array::from_fn(|j| target[j] - backoff);
        // Straight from where the last pose left the arm when that is clear,
        // else through ready; a pose clear neither way is left to the
        // identification, whose own legs reach it.
        let direct = world.check_segment(&at, &approach, 40)?.is_none();
        let via_ready = !direct
            && world.check_segment(&at, &ready, 40)?.is_none()
            && world.check_segment(&ready, &approach, 40)?.is_none();
        if !(direct || via_ready) || world.check_segment(&approach, &target, 40)?.is_some() {
            println!(
                "gains: identification pose {} is not reachable for the posture check; skipped",
                i + 1
            );
            continue;
        }
        poses.push(GainPose {
            from_ready: via_ready,
            approach,
            target,
        });
        at = target;
    }
    if world.check_segment(&at, &ready, 40)?.is_some() {
        return Err("gain qualification cannot return safely to ready".into());
    }
    Ok(poses)
}

/// Every joint's probe span, planned before anything moves: loading the
/// collision world and sweeping each span takes far longer than a control
/// tick, and on the arm the loop would miss its deadline doing it.
fn limit_spans(bundle: &ConfigBundle, assets: &Path, ready: [f64; N]) -> Result<[(f64, f64); N]> {
    let mut world = collision_world(bundle, assets)?;
    let mut spans = [(0.0, 0.0); N];
    for (j, span) in spans.iter_mut().enumerate() {
        *span = probe_span(bundle, &mut world, j, ready)?;
    }
    Ok(spans)
}

/// The largest interval \[rad\] joint `j` can sweep around the ready pose,
/// the others holding it: inside both limit sets by the approach margin, at
/// most `LIMITS_SPAN_RAD` either way, and clear of the collision world at
/// every `LIMITS_SPAN_STEP_RAD` along it.
fn probe_span(
    bundle: &ConfigBundle,
    world: &mut par6_kin::Collision,
    j: usize,
    ready: [f64; N],
) -> Result<(f64, f64)> {
    let limits = &bundle.robot.joints[j].limits;
    let margin = bundle.robot.selfcal.approach_rad;
    let low = limits.soft_min_rad.max(limits.hard_min_rad) + margin;
    let high = limits.soft_max_rad.min(limits.hard_max_rad) - margin;
    let mut reach = |direction: f64| -> Result<f64> {
        let mut at = ready;
        loop {
            let next = at[j] + direction * LIMITS_SPAN_STEP_RAD;
            if (next - ready[j]).abs() > LIMITS_SPAN_RAD || next < low || next > high {
                return Ok(at[j]);
            }
            let mut to = at;
            to[j] = next;
            if world.check_segment(&at, &to, 4)?.is_some() {
                return Ok(at[j]);
            }
            at = to;
        }
    };
    Ok((reach(-1.0)?, reach(1.0)?))
}

impl Arm {
    // ------------------------------------------------ gravity identification

    /// Move every joint to `q` together along one septic in joint space.
    ///
    /// Joint at a time swings the tool through the table between the ready and
    /// forward-reach poses; the straight joint-space line between the same
    /// endpoints clears it, which is what the pose planner checks.
    fn pose(&mut self, q: [f64; N]) -> Result<()> {
        self.pose_checked(q, false).map(|_| ())
    }

    /// With `check`, count every joint's speed-error reversals across the
    /// oscillation band on the way, for the caller to judge; `None` when
    /// nothing was checked.
    fn pose_checked(&mut self, q: [f64; N], check: bool) -> Result<Option<[u32; N]>> {
        let tolerance = self.tolerance();
        let current = self.angles()?;
        if (0..N).all(|j| (q[j] - current[j]).abs() <= tolerance) {
            return Ok(None);
        }
        if !self.homed.iter().all(|h| *h) {
            return Err("a synchronized move needs every joint referenced".into());
        }
        let start: [i32; N] = self.hold;
        let target: [i32; N] = std::array::from_fn(|j| self.conv[j].motor_ticks(q[j]));
        // One septic shared by every joint, so they arrive together on the
        // straight joint-space line the pose planner checked; its duration is
        // whatever the most demanding joint needs under its own EXEC limits.
        let fraction = self.bundle.robot.selfcal.move_speed_fraction;
        let mut profile = SSeptic::new(f64::INFINITY, f64::INFINITY, f64::INFINITY, None, 0.3);
        let mut widest = 0;
        for j in 0..N {
            let travel = (q[j] - current[j]).abs();
            let limits = self.bundle.robot.joints[j].limits.for_mode(LimitMode::Exec);
            let needs = SSeptic::new(
                fraction * limits.velocity_rad_s / travel,
                fraction * limits.acceleration_rad_s2 / travel,
                fraction * limits.jerk_rad_s3.unwrap_or(f64::INFINITY) / travel,
                None,
                0.3,
            );
            if needs.duration() > profile.duration() {
                profile = needs;
                widest = j;
            }
        }
        let duration = profile.duration();
        self.emit(Event::Phase("synchronized move of all joints", widest));
        let moving = self.ticks(duration);
        let total = self.ticks(duration + self.bundle.robot.motion.settle_timeout_s);
        let mut rings: [Ring; N] = std::array::from_fn(|_| Ring::default());
        let mut reference_rings: [Ring; N] = std::array::from_fn(|_| Ring::default());
        let mut expected = start.map(f64::from);
        let mut speed_sq = [0.0; N];
        let mut peak_error = [0.0_f64; N];
        let mut count = [0u32; N];
        let mut reversals = [0u32; N];
        let mut signs = [0i8; N];
        let window = self.ticks(SPEED_WINDOW_S);
        let guard = self.ticks(DETECT_GUARD_S);
        for t in 0..total {
            let generation = self.generation;
            let reference = expected;
            let (position, velocity, _) = profile.sample((t + 1) as f64 * self.dt);
            let feedforward = self.gravity_feedforward();
            let commands: [JointCommand; N] = std::array::from_fn(|j| {
                let distance = f64::from(target[j]) - f64::from(start[j]);
                expected[j] = f64::from(start[j]) + distance * position;
                JointCommand::position(
                    expected[j].round() as i32,
                    (distance * velocity) as i32,
                    feedforward[j],
                )
            });
            self.exchange(commands, true)?;
            if check {
                for j in 0..N {
                    if generation[j] == self.generation[j] {
                        continue;
                    }
                    let p = self.pos(j)?;
                    let measured = if self.simulated {
                        rings[j].push(self.tick, p, window, self.dt)
                    } else {
                        let received = self.position_rx_ns[j];
                        if received == 0
                            || (rings[j].len > 0
                                && received <= rings[j].samples[rings[j].len - 1].2)
                        {
                            return Err(format!(
                                "J{} pose feedback timestamps did not advance",
                                j + 1
                            )
                            .into());
                        }
                        rings[j].push_at(self.tick, p, window, received)
                    };
                    let commanded = reference_rings[j].push(
                        self.tick,
                        reference[j].round() as i32,
                        window,
                        self.dt,
                    );
                    if let (Some(measured), Some(commanded)) = (measured, commanded) {
                        if t >= guard {
                            let speed_error = (measured - commanded) * self.per_tick(j);
                            speed_sq[j] += speed_error.powi(2);
                            let next = if speed_error > SWEEP_BAND_RAD_S {
                                1
                            } else if speed_error < -SWEEP_BAND_RAD_S {
                                -1
                            } else {
                                0
                            };
                            if next != 0 {
                                if signs[j] != 0 && signs[j] != next {
                                    reversals[j] += 1;
                                }
                                signs[j] = next;
                            }
                            peak_error[j] = peak_error[j]
                                .max(((f64::from(p) - reference[j]) * self.per_tick(j)).abs());
                            count[j] += 1;
                        }
                    }
                }
            }
            if t + 1 >= moving
                && (0..N).all(|j| {
                    self.pos(j).is_ok_and(|p| {
                        ((f64::from(p) - f64::from(target[j])) * self.per_tick(j)).abs()
                            <= tolerance
                    })
                })
            {
                self.hold = target;
                match self.ready {
                    Some(ready) if (0..N).all(|j| (q[j] - ready[j]).abs() <= tolerance) => {
                        self.visited.clear();
                    }
                    _ => self.visited.push(q),
                }
                if check {
                    for j in 0..N {
                        if count[j] == 0 {
                            return Err(
                                format!("J{} supplied no scored pose feedback", j + 1).into()
                            );
                        }
                        let speed = (speed_sq[j] / f64::from(count[j])).sqrt();
                        self.emit(Event::GainPoseMotion(
                            self.tick,
                            j,
                            speed,
                            peak_error[j],
                            reversals[j],
                        ));
                    }
                    // Lag is reported, not judged: J4 carries 13 ms of it at
                    // its configured gains where the others carry 2 to 3,
                    // and no gain this stage may choose closes that.
                    // Oscillation is what a candidate answers for.
                    for j in 0..N {
                        if !speed_sq[j].is_finite() {
                            reversals[j] = u32::MAX;
                        }
                    }
                    return Ok(Some(reversals));
                }
                return Ok(None);
            }
        }
        let worst = (0..N)
            .map(|j| {
                let off = (f64::from(self.pos(j).unwrap_or(target[j])) - f64::from(target[j]))
                    * self.per_tick(j);
                (j, off)
            })
            .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
            .ok_or("no joints")?;
        Err(format!(
            "synchronized move did not settle: J{} is {:.4} rad from its target (limit \
             {tolerance:.4})",
            worst.0 + 1,
            worst.1
        )
        .into())
    }

    /// Require a complete stillness window of fresh encoder observations.
    /// A timeout invalidates the measurement; elapsed time is not settling.
    fn wait_still(&mut self, chosen: [bool; N]) -> Result<()> {
        match self.stillness(chosen)? {
            None => Ok(()),
            Some(joint) => Err(format!(
                "J{} did not settle within {STICTION_STILL_TIMEOUT_S}s; measurement rejected",
                joint + 1
            )
            .into()),
        }
    }

    /// The first chosen joint that did not reach a full window of stillness
    /// within `STICTION_STILL_TIMEOUT_S`, or `None` when all did.
    fn stillness(&mut self, chosen: [bool; N]) -> Result<Option<usize>> {
        let window = self.ticks(STICTION_WINDOW_S);
        let mut seen: [VecDeque<(u64, i32)>; N] =
            std::array::from_fn(|_| VecDeque::with_capacity(window as usize + 2));
        let mut generation = self.generation;
        let mut still = chosen.map(|selected| !selected);
        for _ in 0..self.ticks(STICTION_STILL_TIMEOUT_S) {
            self.frame(None)?;
            for j in 0..N {
                if !chosen[j] || generation[j] == self.generation[j] {
                    continue;
                }
                generation[j] = self.generation[j];
                seen[j].push_back((self.tick, self.pos(j)?));
                while seen[j]
                    .get(1)
                    .is_some_and(|(tick, _)| self.tick - tick >= window)
                {
                    seen[j].pop_front();
                }
                let (lo, hi) = seen[j]
                    .iter()
                    .fold((i32::MAX, i32::MIN), |(lo, hi), &(_, p)| {
                        (lo.min(p), hi.max(p))
                    });
                still[j] = seen[j]
                    .front()
                    .is_some_and(|(tick, _)| self.tick - tick >= window)
                    && i64::from(hi) - i64::from(lo) <= i64::from(STICTION_STILL_TICKS);
            }
            if still.iter().all(|s| *s) {
                return Ok(None);
            }
        }
        Ok(still.iter().position(|s| !s))
    }

    /// Every joint's holding current at the pose it is already at.
    ///
    /// Encoder-only replies do not refresh current, so each value must come
    /// from this drain rather than a cached earlier reply, and the arm must
    /// not drift while measuring: a joint that wanders is holding something
    /// other than what the pose says.
    fn holding_current(&mut self) -> Result<[f64; N]> {
        self.wait_still([true; N])?;
        let tolerance = self.tolerance();
        let held: [f64; N] = std::array::from_fn(|k| self.conv[k].joint_rad(self.hold[k]));
        let mut sum = [0.0; N];
        let mut count = [0u32; N];
        let mut generation = self.generation;
        let mut rings: [Ring; N] = std::array::from_fn(|_| Ring::default());
        let mut speed_squared = [0.0; N];
        let mut speed_count = [0u32; N];
        let window = self.ticks(SPEED_WINDOW_S);
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
                let position = self.pos(j)?;
                if let Some(speed) = rings[j].push(self.tick, position, window, self.dt) {
                    speed_squared[j] += (speed * self.per_tick(j)).powi(2);
                    speed_count[j] += 1;
                }
                let current = f64::from(current);
                if current.abs() >= self.bundle.robot.joints[j].ilim_ma {
                    return Err(
                        format!("J{} holding current is saturated at this pose", j + 1).into(),
                    );
                }
                sum[j] += current;
                count[j] += 1;
                self.state.nodes[node].current_ma = None;
            }
        }
        if let Some(j) = (0..N).find(|j| count[*j] == 0) {
            return Err(format!("J{} reported no holding current at this pose", j + 1).into());
        }
        if let Some(j) = (0..N).find(|j| {
            speed_count[*j] == 0
                || (speed_squared[*j] / f64::from(speed_count[*j])).sqrt() > self.holding_limit(*j)
        }) {
            return Err(format!(
                "J{} did not remain still during the gravity measurement",
                j + 1
            )
            .into());
        }
        Ok(std::array::from_fn(|j| sum[j] / f64::from(count[j])))
    }

    /// One identification sample: the torque the arm holds at `q`.
    ///
    /// The pose is approached from below and then from above on every joint,
    /// and the two holding currents averaged. Coulomb friction enters with the
    /// sign of the approach, so averaging opposite approaches cancels its
    /// symmetric part, which on this arm is most of it -- the residual of that
    /// fit matched the measured friction to within 1%. Lin et al. section IV
    /// collect both directions for the same reason:
    /// https://arxiv.org/pdf/2001.06156
    fn sample(&mut self, q: [f64; N]) -> Result<[f64; N]> {
        let backoff = self.bundle.robot.selfcal.approach_rad;
        let mut measured = [[0.0; N]; 2];
        for (k, direction) in [-1.0_f64, 1.0].into_iter().enumerate() {
            self.pose(std::array::from_fn(|j| q[j] + direction * backoff))?;
            self.pose(q)?;
            self.emit(Event::Phase(
                if direction < 0.0 {
                    "hold, approached from below"
                } else {
                    "hold, approached from above"
                },
                0,
            ));
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
    /// What comes out describes the arm with nothing on the flange but the
    /// base attachment, so it stays true when the end effector changes. A tool
    /// is rigidly joined to the last link and shares its regressor columns, so
    /// anything fitted with a gripper on describes that gripper as much as the
    /// arm -- which is why this runs bare.
    fn identify(
        &mut self,
        poses: &[[f64; N]],
        ready: [f64; N],
    ) -> Result<par6_kin::gravity::ArmFit> {
        let mut samples = Vec::with_capacity(poses.len());
        for (i, q) in poses.iter().enumerate() {
            self.emit(Event::IdentPose(i + 1, poses.len(), *q));
            let tau = self.sample(*q)?;
            self.emit(Event::IdentTorque(i + 1, tau));
            samples.push(par6_kin::gravity::GravitySample { q: *q, tau });
        }
        self.pose(ready)?;
        let fit =
            par6_kin::gravity::fit_arm(&mut self.kin, &samples, self.bundle.robot.selfcal.ridge)?;
        self.emit(Event::ArmFit(fit.rms_before_nm, fit.rms_nm));
        if !fit.rms_nm.is_finite() || fit.rms_nm > fit.rms_before_nm {
            return Err("gravity fit did not improve the measured torque residual".into());
        }
        let correction: Vec<f64> = fit
            .correction
            .iter()
            .enumerate()
            .map(|(i, delta)| delta + self.kin.gravity_correction().get(i).copied().unwrap_or(0.0))
            .collect();
        self.kin.set_gravity_correction(&correction)?;
        Ok(fit)
    }

    // ------------------------------------------------------------ parking

    /// Park every joint, then release: on the configured gains, with the
    /// motion guard on, a failed park retried once.
    ///
    /// The shoulder and elbow go back to their homing endstops so releasing
    /// them lets them rest on the stops instead of dropping; everything else
    /// parks at the rest pose. Each joint is parked on its own feedback: one
    /// stale joint used to fail every park, including the loaded shoulder and
    /// elbow, and the release that followed dropped the arm (2026-09-19).
    fn shutdown(&mut self) -> Result<()> {
        self.homing = false;
        self.stopping = true;
        self.deadline = Duration::ZERO;
        // A joint may have stopped answering for a reason that clears on its
        // own. Hold and give every drive a chance before deciding anything.
        self.blind = true;
        for _ in 0..self.ticks(self.bundle.robot.selfcal.stale_recovery_s) {
            if self
                .exchange(self.hold.map(|p| JointCommand::position(p, 0, 0)), false)
                .is_err()
            {
                break;
            }
            if (0..N)
                .all(|j| self.tick.saturating_sub(self.seen[j]) <= self.ticks(FEEDBACK_TIMEOUT_S))
            {
                break;
            }
        }
        self.blind = false;
        for j in 0..N {
            if let Ok(p) = self.pos(j) {
                self.hold[j] = p;
            }
        }
        // Parking runs on the configured gains, whatever a stage left on a
        // drive; the motion guard stays on, and a joint that runs away is
        // held on them and parked again.
        for j in 0..N {
            let configured = self.bundle.robot.joints[j].gains;
            if self.gains[j] != configured && self.gain_restore(j, configured).is_err() {
                self.emit(Event::Phase(
                    "configured gains not confirmed before parking",
                    j,
                ));
            }
        }
        // Back along the legs the run came by, each checked before it was
        // driven, before anything parks joint by joint: that path is not
        // checked, and from identification pose 9 on 2026-10-01 it drove the
        // folded arm into an obstacle, forced the base at its current limit
        // into a 270 deg/s swing and took the bus down with it.
        if let Some(ready) = self.ready {
            let away = self
                .angles()
                .map(|q| (0..N).any(|j| (q[j] - ready[j]).abs() > self.tolerance()))
                .unwrap_or(false);
            // Mid-leg counts: the leg back to the last checked pose, or to
            // ready itself, runs along the segment that was checked.
            if away && self.homed.iter().all(|h| *h) {
                // A run ended by the runaway guard leaves that joint on its
                // configured gains but maybe still in its limit cycle; the
                // retrace would trip the guard on its first leg and park
                // joint by joint from here instead.
                if let Some(j) = self.runaway_joint.take() {
                    if let Err(error) = self.calm(j, self.bundle.robot.joints[j].gains) {
                        self.emit(Event::Phase("could not calm the joint that ran away", j));
                        println!("calm J{}: {error}", j + 1);
                    }
                }
                self.emit(Event::Phase(
                    "retrace the checked legs to ready before parking",
                    0,
                ));
                let mut trail = self.visited.clone();
                trail.reverse();
                trail.push(ready);
                for waypoint in trail {
                    if self.pose(waypoint).is_err() {
                        self.emit(Event::Phase("retrace failed; parking from here", 0));
                        break;
                    }
                }
                for j in 0..N {
                    if let Ok(p) = self.pos(j) {
                        self.hold[j] = p;
                    }
                }
            }
        }
        let mut parked = Ok(());
        // The joints parked on their endstops last: they hold the arm up.
        let rests = |j: usize| self.bundle.robot.parks_on_endstop(j);
        let order: Vec<usize> = (0..N)
            .filter(|&j| !rests(j))
            .chain((0..N).filter(|&j| rests(j)))
            .collect();
        for j in order {
            self.only = Some(j);
            let mut outcome = self.park_one(j);
            if outcome.is_err() {
                self.emit(Event::Phase(
                    "parking failed; once more on its configured gains",
                    j,
                ));
                let _ = self.gain_restore(j, self.bundle.robot.joints[j].gains);
                outcome = self.park_one(j);
            }
            self.only = None;
            if let Err(error) = outcome {
                self.emit(Event::Phase("parking failed", j));
                parked = Err(error);
            }
        }
        if parked.is_err() && !self.bundle.robot.selfcal.release_on_failure {
            return Err("parking failed and release_on_failure is false".into());
        }
        let mut release = Ok(());
        for _ in 0..3 {
            if let Err(e) = self.exchange([JointCommand::drop_to_idle(); N], false) {
                release = Err(e);
            }
        }
        parked.and(release)
    }

    fn park_one(&mut self, j: usize) -> Result<()> {
        if !self.homed[j] {
            // A joint the run never referenced goes back to the count the run
            // found it at, so the next run starts from the same posture: the
            // vendor's pre-homing wrist nudge is a relative move. A joint the
            // run never held was never driven: it is where it was found.
            let Some(found) = self.found_at[j] else {
                return Ok(());
            };
            let distance = (i64::from(found) - i64::from(self.pos(j)?)).abs() as f64;
            if distance <= 1.0 {
                return Ok(());
            }
            let per_s =
                (self.bundle.robot.shutdown.velocity_limit_rad_s / self.per_tick(j).abs()).max(1.0);
            self.emit(Event::Phase("return to where the run found it", j));
            self.operating(j)?;
            let seconds = (SEPTIC_PEAK_VEL * distance / per_s).max(0.3);
            let m = self.run_motion(j, found, seconds, false)?;
            return if m.outcome == Outcome::Complete {
                Ok(())
            } else {
                Err(format!("J{} did not return to its starting position", j + 1).into())
            };
        }
        let target = if self.bundle.robot.parks_on_endstop(j) {
            self.bundle
                .effective_home_offset(j)
                .ok_or("missing home offset")?
        } else {
            self.bundle.robot.robot.park_pose_rad[j]
        };
        let mut outcome = self.park_to(j, target);
        if outcome.is_err() && self.bundle.robot.parks_on_endstop(j) {
            // These hold the arm up: try once more without requiring feedback
            // rather than release them where they are.
            self.emit(Event::Phase("park blind: feedback unavailable", j));
            self.blind = true;
            outcome = self.park_to(j, target);
            self.blind = false;
        }
        outcome
    }

    fn park_to(&mut self, j: usize, target: f64) -> Result<()> {
        let travel = (target - self.angles()?[j]).abs();
        let seconds =
            (SEPTIC_PEAK_VEL * travel / self.bundle.robot.shutdown.velocity_limit_rad_s).max(0.3);
        self.operating(j)?;
        self.move_joint(j, target, seconds)
    }
}

// ---------------------------------------------------------------- poses

/// The pose homing leaves the arm in, exactly as the daemon computes it: the
/// last `move_to` per joint in sequence order. A joint no step places is an
/// error there, so it is one here -- the identification poses are drawn around
/// this pose, and a different answer would sweep a different arm.
fn planned_ready(bundle: &ConfigBundle) -> Result<[f64; N]> {
    let ready = bundle
        .robot
        .homing
        .ready_pose_rad(bundle.robot.joints.len())
        .map_err(|e| format!("ready pose: {e}"))?;
    Ok(std::array::from_fn(|j| ready[j]))
}

/// The daemon's collision world for the fitted tool, with the installation
/// keep-outs applied.
fn collision_world(bundle: &ConfigBundle, assets: &Path) -> Result<par6_kin::Collision> {
    let variant = par6_kin::GripperVariant::resolve(
        &bundle.robot.robot.active_tool.to_ascii_uppercase(),
        bundle.active_tool().and_then(|g| g.urdf_variant.as_deref()),
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

/// The identification plan: the poses in visiting order, and the share of
/// each parameter they would pin.
struct IdentPlan {
    poses: Vec<[f64; N]>,
    determined: Vec<f64>,
}
/// Joint-side inertia at `q`: the mass-matrix diagonal, plus the rotor
/// reflected through the reduction.
///
/// `Kin::dyn_feedforward` is the inverse dynamics with the gravity term
/// subtracted back out, so zero velocity and a unit acceleration on one
/// joint alone leave exactly `M_jj(q)` in that slot. The rotor table is
/// motor-side and reflects as `G^2 jm` through the dynamics ratio -- the
/// vendor's J1 reduction disagrees with its kinematic one, so this uses the
/// same fallback `par6-bus` does.
fn joint_inertia(kin: &mut par6_kin::Kin, bundle: &ConfigBundle, q: [f64; N]) -> Result<[f64; N]> {
    let zero = [0.0; N];
    let mut out = [0.0; N];
    for j in 0..N {
        let mut qdd = zero;
        qdd[j] = 1.0;
        let mut tau = zero;
        kin.dyn_feedforward(&q, &zero, &qdd, &mut tau)
            .map_err(|e| format!("J{} inertia: {e}", j + 1))?;
        let cfg = &bundle.robot.joints[j];
        let g = cfg.dynamics_gear_ratio.unwrap_or(cfg.gear_ratio);
        out[j] = tau[j] + g * g * bundle.robot.sim.motor_jm_kg_m2[j];
    }
    Ok(out)
}

/// The model the run identifies against: the arm with the fitted tool,
/// carrying the correction the daemon already installs, so a fit refines
/// rather than refits.
fn arm_kin(bundle: &ConfigBundle, assets: &Path) -> Result<par6_kin::Kin> {
    let params = bundle.active_tool().map(|g| {
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
    kin.set_gravity_correction(&bundle.robot.gravity_correction)?;
    Ok(kin)
}

/// Poses for identifying the arm's own links, chosen for what they pin.
///
/// Candidates are drawn deterministically over each joint's whole window,
/// so a run is repeatable, and kept only if the pose and the two approach
/// offsets the measurement uses are inside both limit sets, clear of the
/// collision world, and holdable with `IDENT_HOLD_FRACTION` of every
/// joint's current limit. From those the plan takes, one at a time, the
/// pose that raises the log-determinant of the set's ridged normal matrix
/// the most -- the D-optimal pick, scored exactly as `fit_arm` will score
/// the result -- among the candidates whose legs from the previous pick
/// are clear. A band around the ready pose used to bound the draw; on the
/// arm it pinned 7 of 24 parameters, and the model sagged with the arm
/// horizontal, where nothing had been measured.
///
/// Both limit sets because this arm's J6 declares a soft range wider than
/// its hard one, and a pose drawn from the soft range alone drove it into
/// the mechanical stop at full current.
fn identification_poses(
    bundle: &ConfigBundle,
    assets: &Path,
    ready: [f64; N],
) -> Result<IdentPlan> {
    let mut world = collision_world(bundle, assets)?;
    let mut kin = arm_kin(bundle, assets)?;
    let backoff = bundle.robot.selfcal.approach_rad;
    let ridge = bundle.robot.selfcal.ridge;
    let mut seed = 0x2545_F491_4F6C_DD1Du64;
    let mut unit = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    let window = |j: usize| {
        let l = &bundle.robot.joints[j].limits;
        let lo = l.soft_min_rad.max(l.hard_min_rad) + backoff;
        let hi = l.soft_max_rad.min(l.hard_max_rad) - backoff;
        (lo, hi)
    };
    if let Some(j) = (0..N).find(|j| {
        let (lo, hi) = window(*j);
        lo >= hi
    }) {
        return Err(format!(
            "J{} has no travel left once its {backoff} rad approach offsets are allowed for",
            j + 1
        )
        .into());
    }
    // Gravity torque a pose may ask of each joint, from its current limit.
    let budget: [f64; N] = std::array::from_fn(|j| {
        let joint = &bundle.robot.joints[j];
        let ma_per_nm = par6_bus::spectral::torque_to_ma_factor(
            joint.gear_ratio,
            joint.gear_efficiency,
            joint.kt_nm_a,
            joint.dir,
        )
        .abs();
        IDENT_HOLD_FRACTION * joint.ilim_ma / ma_per_nm
    });
    let approaches = |q: &[f64; N]| -> [[f64; N]; 2] {
        [
            std::array::from_fn(|j| q[j] - backoff),
            std::array::from_fn(|j| q[j] + backoff),
        ]
    };
    let clear = |world: &mut par6_kin::Collision, p: &[f64; N]| -> bool {
        !world.check(p, true).map(|r| r.active()).unwrap_or(true)
    };
    // Every leg the measurement will actually drive from `from` to `q`.
    let legs_clear = |world: &mut par6_kin::Collision, from: &[f64; N], q: &[f64; N]| -> bool {
        let [below, above] = approaches(q);
        [(*from, below), (below, *q), (*q, above), (above, *q)]
            .iter()
            .all(|(a, b)| {
                world
                    .check_segment(a, b, 40)
                    .map(|c| c.is_none())
                    .unwrap_or(false)
            })
    };

    let wanted = bundle.robot.selfcal.identification_poses;
    let mut candidates: Vec<([f64; N], Vec<f64>)> = Vec::with_capacity(IDENT_CANDIDATES);
    let mut tau = [0.0; N];
    // Bounded: a scene that refuses everything says so rather than spinning.
    for _ in 0..IDENT_CANDIDATES * 20 {
        if candidates.len() == IDENT_CANDIDATES {
            break;
        }
        // Gravity does not see the base joint, so the plan does not swing
        // it: every pose keeps J1 where the ready pose has it.
        let q: [f64; N] = std::array::from_fn(|j| {
            let (lo, hi) = window(j);
            if j == 0 {
                ready[j].clamp(lo, hi)
            } else {
                lo + unit() * (hi - lo)
            }
        });
        let [below, above] = approaches(&q);
        if !(clear(&mut world, &q) && clear(&mut world, &below) && clear(&mut world, &above)) {
            continue;
        }
        kin.gravity(&q, &mut tau)?;
        if (0..N).any(|j| tau[j].abs() > budget[j]) {
            continue;
        }
        candidates.push((q, par6_kin::gravity::PoseDesign::regressor(&mut kin, &q)?));
    }
    if candidates.len() < wanted {
        return Err(format!(
            "only {} of the {wanted} identification poses wanted cleared the limits, the \
             collision world and the holding budget",
            candidates.len()
        )
        .into());
    }

    let mut design = par6_kin::gravity::PoseDesign::new(&kin);
    let mut picked: Vec<[f64; N]> = Vec::with_capacity(wanted);
    let mut previous = ready;
    while picked.len() < wanted {
        // The score is cheap and the leg sweep is not: rank every candidate,
        // then sweep down the ranking only until one clears.
        let mut ranked: Vec<(usize, f64)> = candidates
            .iter()
            .enumerate()
            .filter_map(|(i, (_, y))| design.gain(y, ridge).map(|g| (i, g)))
            .collect();
        ranked.sort_by(|a, b| b.1.total_cmp(&a.1));
        let Some(pick) = ranked
            .iter()
            .map(|(i, _)| *i)
            .find(|&i| legs_clear(&mut world, &previous, &candidates[i].0))
        else {
            return Err(format!(
                "no candidate pose can be reached from identification pose {}",
                picked.len()
            )
            .into());
        };
        let (q, y) = candidates.swap_remove(pick);
        design.add(&y);
        previous = q;
        picked.push(q);
    }
    let determined = design.determined(ridge);

    // Visit them nearest-first: the pick order spends most of the run
    // travelling, and the poses are equally informative in any order.
    let mut remaining = picked.clone();
    let mut ordered = Vec::with_capacity(remaining.len());
    let mut at = ready;
    while !remaining.is_empty() {
        let (i, _) = remaining
            .iter()
            .enumerate()
            .map(|(i, q)| (i, (0..N).map(|j| (q[j] - at[j]).abs()).sum::<f64>()))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .ok_or("no poses")?;
        at = remaining.remove(i);
        ordered.push(at);
    }
    // Sweep the legs the run will actually drive, in the order it drives
    // them -- `Arm::pose` makes no check of its own -- and the return to
    // the ready pose, so the run finishes where parking expects. Nearest-
    // first permuted the list, so a leg it introduced may collide where
    // the pick order, swept above, did not; that order is the fallback.
    let mut sweep = |order: &[[f64; N]]| -> Result<bool> {
        let mut at = ready;
        for q in order {
            let first = std::array::from_fn::<f64, N, _>(|j| q[j] - backoff);
            for (a, b) in [(at, first), (first, *q)] {
                if world.check_segment(&a, &b, 40)?.is_some() {
                    return Ok(false);
                }
            }
            at = *q;
        }
        Ok(world.check_segment(&at, &ready, 40)?.is_none())
    };
    let poses = if sweep(&ordered)? {
        ordered
    } else if sweep(&picked)? {
        picked
    } else {
        return Err(
            "no visiting order of the identification poses is clear of the collision \
                    world, the return to the ready pose included"
                .into(),
        );
    };
    Ok(IdentPlan { poses, determined })
}

// ---------------------------------------------------------------- applying

/// Earlier runs the end-of-run history shows beside this one: the two that
/// must agree for repeatability, and one more for the trend.
const HISTORY_RUNS: usize = 3;

/// One run as its directory records it: the candidate it wrote, which stages
/// passed for which joint (`stages.tsv`), and whether it ran in the
/// simulator (`run.toml`).
struct Run {
    label: String,
    config: par6_config::RobotConfig,
    passed: Vec<(String, Option<usize>)>,
    sim: bool,
}

impl Run {
    const FILES: [&'static str; 3] = ["calibrated.toml", "stages.tsv", "run.toml"];

    fn recorded(dir: &Path) -> bool {
        Self::FILES.iter().all(|file| dir.join(file).exists())
    }

    fn load(dir: &Path) -> Result<Self> {
        let config = par6_config::RobotConfig::load(&dir.join("calibrated.toml"))?;
        let passed = fs::read_to_string(dir.join("stages.tsv"))?
            .lines()
            .skip(1)
            .filter_map(|line| {
                let mut cells = line.split('\t');
                let (stage, joint, status) = (cells.next()?, cells.next()?, cells.next()?);
                status.starts_with("passed").then(|| {
                    (
                        stage.to_owned(),
                        joint.parse::<usize>().ok().map(|j| j.saturating_sub(1)),
                    )
                })
            })
            .collect();
        let sim = fs::read_to_string(dir.join("run.toml"))?
            .lines()
            .any(|line| line.trim() == "sim = true");
        Ok(Self {
            label: run_label(dir),
            config,
            passed,
            sim,
        })
    }

    /// Whether this run measured `stage` for `joint` (`None`: the arm).
    fn measured(&self, stage: &str, joint: Option<usize>) -> bool {
        self.passed.iter().any(|(s, j)| s == stage && *j == joint)
    }

    fn joint<T>(
        &self,
        stage: &str,
        j: usize,
        pick: impl Fn(&par6_config::JointConfig) -> T,
    ) -> Option<T> {
        self.measured(stage, Some(j))
            .then(|| self.config.joints.get(j).map(pick))
            .flatten()
    }
}

/// The run directory's name carries nanoseconds since the epoch, which
/// nobody reads by eye: the label is its minute, `MM-DD HH:MMZ`.
fn run_label(dir: &Path) -> String {
    let name = dir
        .file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_default();
    match run_stamp(dir) {
        Some(nanos) => utc_minute((nanos / 1_000_000_000) as u64),
        None => name,
    }
}

/// The nanosecond stamp in a `selfcal-<nanos>` directory name.
fn run_stamp(dir: &Path) -> Option<u128> {
    dir.file_name()?
        .to_str()?
        .strip_prefix("selfcal-")?
        .parse()
        .ok()
}

/// `MM-DD HH:MMZ` of a Unix time (Hinnant's civil-from-days).
fn utc_minute(secs: u64) -> String {
    let z = (secs / 86_400) as i64 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let rem = secs % 86_400;
    format!(
        "{month:02}-{day:02} {:02}:{:02}Z",
        rem / 3600,
        (rem % 3600) / 60
    )
}

/// One line of the history table: the quantity, its value in each run shown
/// (`-` where that run did not measure it), and how this run's compares with
/// the newest earlier one.
struct HistoryRow {
    name: String,
    values: Vec<String>,
    change: String,
}

impl HistoryRow {
    /// `same` when the two displayed values agree, otherwise the change as a
    /// share of the earlier value; `n/a` when either run did not measure it.
    fn numeric(name: String, values: &[Option<f64>], decimals: usize) -> Self {
        let text =
            |value: Option<f64>| value.map_or_else(|| "-".to_owned(), |v| compact(v, decimals));
        let change = match (values[0], values[1]) {
            (Some(now), Some(before)) if text(Some(now)) == text(Some(before)) => "same".to_owned(),
            (Some(now), Some(before)) if before != 0.0 => {
                format!("{:+.1}%", 100.0 * (now - before) / before)
            }
            (Some(_), Some(_)) => "from zero".to_owned(),
            _ => "n/a".to_owned(),
        };
        Self {
            name,
            values: values.iter().map(|value| text(*value)).collect(),
            change,
        }
    }
}

/// A per-joint friction value out of the simulator section.
type SimPick = fn(&par6_config::SimConfig, usize) -> Option<f64>;
/// One of a joint's EXEC limits.
type LimitPick = fn(&par6_config::ResolvedLimits) -> Option<f64>;

/// How the calibration is moving: this run's candidate beside the most
/// recent earlier runs in the same output directory with the same simulator
/// flag, newest first, one row per quantity and joint. A value a run did not
/// measure shows as `-` and is not compared. The console gets the rows that
/// moved; the whole table is left as `history.tsv`. `None` when no earlier
/// run can be compared; what was skipped is said.
fn history(directory: &Path) -> Result<Option<(String, String)>> {
    let current = Run::load(directory)?;
    let Some(parent) = directory.parent() else {
        return Ok(None);
    };
    let mut siblings: Vec<(u128, PathBuf)> = fs::read_dir(parent)?
        .filter_map(|entry| entry.ok().map(|entry| entry.path()))
        .filter(|path| path != directory && path.is_dir())
        .filter_map(|path| run_stamp(&path).map(|stamp| (stamp, path)))
        .collect();
    siblings.sort_by_key(|(stamp, _)| std::cmp::Reverse(*stamp));
    let mut earlier: Vec<Run> = Vec::new();
    let mut skipped = 0usize;
    for (_, path) in siblings {
        if !Run::recorded(&path) {
            skipped += 1;
            continue;
        }
        match Run::load(&path) {
            Ok(run) if run.sim == current.sim => {
                earlier.push(run);
                if earlier.len() == HISTORY_RUNS {
                    break;
                }
            }
            Ok(_) => skipped += 1,
            Err(error) => {
                println!("history: {}: {error}", path.display());
                skipped += 1;
            }
        }
    }
    if earlier.is_empty() {
        if skipped > 0 {
            println!(
                "history: no comparable earlier run in {} ({skipped} skipped: no record, \
                 other simulator flag, or unreadable)",
                parent.display()
            );
        }
        return Ok(None);
    }
    let runs: Vec<&Run> = std::iter::once(&current).chain(earlier.iter()).collect();
    let joints = runs
        .iter()
        .map(|run| run.config.joints.len())
        .min()
        .unwrap_or(0);
    let mut cells: Vec<HistoryRow> = Vec::new();
    for j in 0..joints {
        for axis in [GainAxis::Kpv, GainAxis::Kiv, GainAxis::Kpp] {
            let values: Vec<Option<f64>> = runs
                .iter()
                .map(|run| run.joint("gains", j, |joint| axis.value(joint.gains)))
                .collect();
            let decimals = if axis == GainAxis::Kiv { 8 } else { 6 };
            cells.push(HistoryRow::numeric(
                format!("J{} {}", j + 1, axis.label()),
                &values,
                decimals,
            ));
        }
    }
    for j in 0..joints {
        let picks: [(&str, SimPick); 2] = [
            ("viscous Nm.s/rad", |sim, j| {
                sim.viscous_nm_s.get(j).copied()
            }),
            ("coulomb Nm", |sim, j| sim.coulomb_nm.get(j).copied()),
        ];
        for (name, pick) in picks {
            let values: Vec<Option<f64>> = runs
                .iter()
                .map(|run| {
                    run.measured("friction", Some(j))
                        .then(|| pick(&run.config.sim, j))
                        .flatten()
                })
                .collect();
            cells.push(HistoryRow::numeric(
                format!("J{} {name}", j + 1),
                &values,
                6,
            ));
        }
    }
    for j in 0..joints {
        let terms = |run: &Run| {
            run.joint("ripple", j, |joint| {
                let mut terms: Vec<(u8, i16, i16)> = joint
                    .ripple
                    .iter()
                    .map(|h| (h.harmonic, h.a_ma, h.b_ma))
                    .collect();
                terms.sort_unstable();
                terms
            })
        };
        let size = |terms: &[(u8, i16, i16)]| {
            ripple::total(
                &terms
                    .iter()
                    .map(|&(harmonic, a, b)| ripple::Harmonic {
                        harmonic,
                        a: f64::from(a),
                        b: f64::from(b),
                    })
                    .collect::<Vec<_>>(),
            )
        };
        let totals: Vec<Option<f64>> = runs
            .iter()
            .map(|run| terms(run).map(|terms| size(&terms)))
            .collect();
        let mut row = HistoryRow::numeric(format!("J{} ripple mA", j + 1), &totals, 1);
        // The size can stay while the coefficients move (a harmonic changing
        // sign, amplitude moving between harmonics): "same" is their call.
        if let (Some(now), Some(before)) = (terms(runs[0]), terms(runs[1])) {
            if now == before {
                row.change = "same".to_owned();
            } else if row.change == "same" {
                row.change = "coefficients moved".to_owned();
            }
        }
        cells.push(row);
    }
    for j in 0..joints {
        let exec = |run: &Run, pick: LimitPick| {
            run.joint("limits", j, |joint| {
                pick(&joint.limits.for_mode(LimitMode::Exec))
            })
            .flatten()
        };
        let picks: [(&str, LimitPick); 3] = [
            ("exec velocity rad/s", |l| Some(l.velocity_rad_s)),
            ("exec acceleration rad/s2", |l| Some(l.acceleration_rad_s2)),
            ("exec jerk rad/s3", |l| l.jerk_rad_s3),
        ];
        for (name, pick) in picks {
            let values: Vec<Option<f64>> = runs.iter().map(|run| exec(run, pick)).collect();
            cells.push(HistoryRow::numeric(
                format!("J{} {name}", j + 1),
                &values,
                4,
            ));
        }
    }
    let gravity = |run: &Run| {
        run.measured("gravity", None)
            .then(|| run.config.gravity_correction.clone())
    };
    let peaks: Vec<Option<f64>> = runs
        .iter()
        .map(|run| gravity(run).map(|terms| terms.iter().fold(0.0_f64, |max, t| max.max(t.abs()))))
        .collect();
    let mut row = HistoryRow::numeric("gravity max |term|".to_owned(), &peaks, 5);
    if let (Some(now), Some(before)) = (gravity(runs[0]), gravity(runs[1])) {
        row.change = if now == before {
            "same".to_owned()
        } else if now.len() == before.len() {
            let delta = now
                .iter()
                .zip(&before)
                .fold(0.0_f64, |max, (a, b)| max.max((a - b).abs()));
            format!("max |diff| {delta:.2e}")
        } else {
            "terms differ".to_owned()
        };
    }
    cells.push(row);

    let header: Vec<String> = runs.iter().map(|run| run.label.clone()).collect();
    let aligned = |name: &str, values: &[String], change: &str| {
        let mut text = format!("{name:<26}");
        for value in values {
            let _ = write!(text, "{value:>14}");
        }
        let _ = writeln!(text, "{change:>20}");
        text
    };
    let tabbed = |name: &str, values: &[String], change: &str| {
        format!("{name}\t{}\t{change}\n", values.join("\t"))
    };
    let header_labels: Vec<String> = std::iter::once("this run".to_owned())
        .chain(header.iter().skip(1).cloned())
        .collect();
    let mut table = tabbed("quantity", &header_labels, "change vs newest");
    let mut moved = String::new();
    let (mut unchanged, mut compared) = (0usize, 0usize);
    for row in &cells {
        table.push_str(&tabbed(&row.name, &row.values, &row.change));
        if row.change == "n/a" {
            continue;
        }
        compared += 1;
        if row.change == "same" {
            unchanged += 1;
        } else {
            moved.push_str(&aligned(&row.name, &row.values, &row.change));
        }
    }
    let mut console = format!(
        "history: {} earlier run{} in {} (newest first); {unchanged} of {compared} compared values unchanged since {}{}\n",
        earlier.len(),
        if earlier.len() == 1 { "" } else { "s" },
        parent.display(),
        earlier[0].label,
        if skipped > 0 {
            format!("; {skipped} run director{} skipped", if skipped == 1 { "y" } else { "ies" })
        } else {
            String::new()
        },
    );
    if moved.is_empty() {
        console.push_str("history: no compared value moved\n");
    } else {
        console.push_str(&aligned("quantity", &header_labels, "change vs newest"));
        console.push_str(&moved);
    }
    Ok(Some((console, table)))
}

/// `decimals` places with the trailing zeros dropped; a zero of either sign
/// is `0`.
fn compact(value: f64, decimals: usize) -> String {
    if value == 0.0 {
        return "0".to_owned();
    }
    let text = format!("{value:.decimals$}");
    text.trim_end_matches('0').trim_end_matches('.').to_owned()
}

/// The measured values as the installation's local overlay: `original`
/// (the overlay as it stands, empty when there is none) with what this run
/// moved written over it, its comments and every value it did not touch
/// kept as they were.
fn patch_config(
    original: &str,
    correction: Option<&[f64]>,
    friction: Option<&[Option<(f64, f64)>; N]>,
    sim: &par6_config::SimConfig,
    ripples: Option<&[Option<Vec<RippleHarmonic>>; N]>,
    tuned: Option<&[Option<Tuned>; N]>,
    limits: Option<&[Option<Found>; N]>,
) -> Result<String> {
    let mut overlay = par6_config::LocalOverlay::parse(original)?;
    let joint = |j: usize| format!("joint{}", j + 1);
    if let Some(correction) = correction {
        overlay.set_array(&[], "gravity_correction", correction)?;
        // Identification measures the true torque, so any per-joint trim from
        // an older calibration is superseded and would otherwise multiply it.
        overlay.set_array(&[], "gravity_scale", &[1.0; N])?;
    }
    if let Some(friction) = friction {
        // The friction the simulator's joints show their drives: measured
        // joint by joint, a joint the run skipped keeps its current value.
        let measured = |pick: fn(&(f64, f64)) -> f64, current: &[f64]| -> Vec<f64> {
            friction
                .iter()
                .zip(current)
                .map(|(m, c)| m.as_ref().map_or(*c, pick))
                .collect()
        };
        overlay.set_array(
            &["sim"],
            "viscous_nm_s",
            &measured(|m| m.0, &sim.viscous_nm_s),
        )?;
        overlay.set_array(&["sim"], "coulomb_nm", &measured(|m| m.1, &sim.coulomb_nm))?;
    }
    for (j, r) in ripples.into_iter().flatten().enumerate() {
        // What the stage found for a joint it visited replaces what the
        // overlay had, and a visited joint it found nothing for loses its
        // line, so the shipped value stands again.
        match r {
            Some(r) if r.is_empty() => overlay.remove_joint_key(&joint(j), "ripple")?,
            Some(r) => {
                let entries: toml_edit::Array = r
                    .iter()
                    .map(|h| {
                        let mut entry = toml_edit::InlineTable::new();
                        entry.insert("harmonic", i64::from(h.harmonic).into());
                        entry.insert("a_ma", i64::from(h.a_ma).into());
                        entry.insert("b_ma", i64::from(h.b_ma).into());
                        toml_edit::Value::InlineTable(entry)
                    })
                    .collect();
                overlay.set_joint(&joint(j), &[], "ripple", entries)?;
            }
            None => {}
        }
    }
    for (j, t) in tuned.into_iter().flatten().enumerate() {
        // Only what the design moved: a gain it left alone is not written.
        if let Some(t) = t {
            for (key, before, after) in [
                ("kpv", t.before.kpv, t.after.kpv),
                ("kiv", t.before.kiv, t.after.kiv),
                ("kpp", t.before.kpp, t.after.kpp),
            ] {
                if after != before {
                    overlay.set_joint(&joint(j), &["gains"], key, after)?;
                }
            }
        }
    }
    for (j, found) in limits.into_iter().flatten().enumerate() {
        // A cap the search left where it was is not written, and a jerk it
        // only bounded keeps its configured value.
        if let Some(found) = found {
            let (caps, was) = (&found.caps, &found.configured);
            let round = |v: f64| (v * 1e5).round() / 1e5;
            for (key, cap, configured, write) in [
                ("velocity_rad_s", caps.velocity, was.velocity, true),
                (
                    "acceleration_rad_s2",
                    caps.acceleration,
                    was.acceleration,
                    true,
                ),
                ("jerk_rad_s3", caps.jerk, was.jerk, found.jerk_measured),
            ] {
                if write && cap != configured {
                    overlay.set_joint(&joint(j), &["limits", "exec"], key, round(cap))?;
                }
            }
        }
    }
    Ok(overlay.to_string())
}

fn main() -> std::process::ExitCode {
    use clap::Parser;
    match run(Args::parse()) {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("{e}");
            std::process::ExitCode::FAILURE
        }
    }
}

fn stage_status(
    statuses: &mut [(&'static str, Option<usize>, &'static str)],
    stage: &'static str,
    joint: Option<usize>,
    status: &'static str,
) {
    if let Some(row) = statuses
        .iter_mut()
        .find(|(s, j, _)| *s == stage && *j == joint)
    {
        row.2 = status;
    }
    match joint {
        Some(j) => println!("STAGE {stage} J{}: {status}", j + 1),
        None => println!("STAGE {stage}: {status}"),
    }
}

fn run(args: Args) -> Result<()> {
    if let Some(dir) = &args.history {
        match history(dir)? {
            Some((console, _)) => print!("{console}"),
            None => println!(
                "history: no comparable earlier run beside {}",
                dir.display()
            ),
        }
        return Ok(());
    }
    let runs = |stage: Stage| args.only.is_empty() || args.only.contains(&stage);
    if args.verify_gains.is_some() && args.only != [Stage::Gains] {
        return Err("--verify-gains requires --only gains".into());
    }
    if !args.joints.is_empty()
        && (args.only.is_empty()
            || args
                .only
                .iter()
                .any(|s| !matches!(s, Stage::Ripple | Stage::Gains)))
    {
        return Err("--joint applies to --only ripple and --only gains".into());
    }
    let chosen: [bool; N] =
        std::array::from_fn(|j| args.joints.is_empty() || args.joints.contains(&(j as u8 + 1)));
    let limits_stage = args.limits && args.only.is_empty() || args.only.contains(&Stage::Limits);
    let ripple_stage = runs(Stage::Ripple);
    let gains_stage = runs(Stage::Gains);
    let stiction_stage = runs(Stage::Stiction);
    let belt_stage = runs(Stage::Belt);
    let mechanics_stage = runs(Stage::Mechanics);
    // The shipped config describes the PAR6; this arm's own values, the
    // ones a run measures, live in its overlay.
    let overlay = args
        .local_config
        .clone()
        .unwrap_or_else(|| args.config.with_file_name(par6_config::LOCAL_CONFIG_NAME));
    let existing = overlay.is_file().then(|| overlay.clone());
    let bundle = ConfigBundle::load_with(&args.config, existing.as_deref(), args.tool.as_deref())?;
    bundle.robot.validate()?;
    // The arm's gravity is fitted with nothing on the flange: a tool shares
    // the last link's regressor columns, so a fit with one on writes that
    // tool into a correction every other tool then carries.
    if mechanics_stage
        && !bundle
            .active_tool()
            .is_some_and(|t| t.name.eq_ignore_ascii_case("Flange"))
    {
        return Err(format!(
            "the mechanics stage identifies the bare arm, but `{}` is fitted: take the \
             tool off and run with --tool Flange, or leave the stage out with --only",
            bundle.robot.robot.active_tool
        )
        .into());
    }
    if bundle.robot.joints.len() != N {
        return Err(format!(
            "par6-selfcal calibrates a {N}-joint arm; this configuration names {}",
            bundle.robot.joints.len()
        )
        .into());
    }
    let load_candidate = |path: &PathBuf| -> Result<par6_config::RobotConfig> {
        let candidate = par6_config::RobotConfig::from_toml_str(&fs::read_to_string(path)?)?;
        candidate.validate()?;
        if candidate.joints.len() != N
            || candidate
                .joints
                .iter()
                .zip(&bundle.robot.joints)
                .any(|(a, b)| a.node_id != b.node_id || a.name != b.name)
        {
            return Err("candidate gains must name the same six joints and node IDs".into());
        }
        Ok(candidate)
    };
    let candidates = args.verify_gains.as_ref().map(load_candidate).transpose()?;
    let sim = bundle.robot.sim.clone();
    // Resolved the way the daemon resolves it: a lexical step up from the
    // config directory, never `config/..` through the filesystem, which
    // follows the `config` symlink into the package and lands beside it.
    let assets = par6d::kin::resolve_assets_dir(None, &args.config)?;
    let ready = planned_ready(&bundle)?;
    let poses = if mechanics_stage || limits_stage || gains_stage {
        // Plan the poses before anything moves: a scene that cannot be covered
        // should say so with the arm still parked.
        let plan = identification_poses(&bundle, &assets, ready)?;
        let poses = plan.poses;
        // What the fit will report is a rank, the same for any non-degenerate
        // set; what the plan buys is coverage, so that is what it prints.
        let coverage: Vec<String> = (1..N)
            .map(|j| {
                let lo = poses.iter().map(|q| q[j]).fold(f64::INFINITY, f64::min);
                let hi = poses.iter().map(|q| q[j]).fold(f64::NEG_INFINITY, f64::max);
                format!("J{} {:.0}..{:.0}", j + 1, lo.to_degrees(), hi.to_degrees())
            })
            .collect();
        println!(
            "identification plan: {} poses, J1 held at ready, {} deg; {:.1} of {} parameters \
         observable",
            poses.len(),
            coverage.join(", "),
            plan.determined.iter().sum::<f64>(),
            plan.determined.len()
        );

        poses
    } else {
        Vec::new()
    };

    let directory = args.output_dir.join(format!(
        "selfcal-{}",
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)?
            .as_nanos()
    ));
    fs::create_dir_all(&directory)?;
    println!("RUN DIRECTORY: {}", directory.display());
    fs::write(
        directory.join("identification-poses.csv"),
        poses
            .iter()
            .enumerate()
            .map(|(i, q)| format!("{},\"{q:?}\"\n", i + 1))
            .collect::<String>(),
    )?;

    // The queue is bounded: recording must not let the control loop run ahead
    // of what can be written, and must not allocate on the tick path.
    let (tx, rx) = sync_channel(1 << 16);
    let sink = directory.clone();
    let recorder = std::thread::spawn(move || writer(rx, sink));

    unsafe {
        libc::signal(libc::SIGINT, cancel as *const () as libc::sighandler_t);
        libc::signal(libc::SIGTERM, cancel as *const () as libc::sighandler_t);
    }
    let timing = bundle.robot.timing.unwrap_or_default();
    let _runtime = (!args.sim)
        .then(|| runtime::Runtime::prepare(timing.cpu))
        .transpose()?;

    let original = match &existing {
        Some(path) => fs::read_to_string(path)?,
        None => String::new(),
    };
    let config_correction = bundle.robot.gravity_correction.clone();
    // The runtime multiplies the gravity feedforward by this
    // (par6-rt/src/core.rs), but the model identified against here does not
    // carry it. A trim left by an older calibration would therefore keep
    // scaling the freshly measured torque -- J3 shipped 8.15% high that way.
    // The identification supersedes it, so it is written back as unity.
    let stale_scale: Vec<usize> = bundle
        .robot
        .gravity_scale
        .iter()
        .enumerate()
        .filter(|(_, s)| (**s - 1.0).abs() > 1e-9)
        .map(|(j, _)| j + 1)
        .collect();
    // The stiction stage's second pose, the arm out level, where the base's
    // bearings carry the most overturning moment: taken only if the
    // straight path there and back is clear.
    let out_pose = if stiction_stage {
        let mut out = ready;
        out[1] = (-40.0_f64).to_radians();
        out[2] = 185.0_f64.to_radians();
        out[4] = (-60.0_f64).to_radians();
        let mut world = collision_world(&bundle, &assets)?;
        if world.check_segment(&ready, &out, 40)?.is_some() {
            println!(
                "stiction: the arm-out pose is not reachable from ready; measuring at ready only"
            );
            None
        } else {
            Some(out)
        }
    } else {
        None
    };
    let spans = if limits_stage || gains_stage || belt_stage || ripple_stage || mechanics_stage {
        Some(limit_spans(&bundle, &assets, ready)?)
    } else {
        None
    };
    let gain_poses = if gains_stage {
        gain_pose_plan(&bundle, &assets, ready, &poses, out_pose)?
    } else {
        Vec::new()
    };
    if gains_stage {
        if candidates.is_some() {
            println!("gains: verify selected candidates together, without a gain search");
        } else {
            println!("gains: StepFOC Kpv -> Kiv -> Kpp on fixed lattices, at most {GAIN_OBSERVATIONS} observations per joint including verification");
        }
        println!(
            "gains: {} calibration poses qualify the accepted candidates together",
            gain_poses.len()
        );
        fs::write(
            directory.join("gain-qualification-poses.csv"),
            gain_poses
                .iter()
                .enumerate()
                .map(|(i, p)| {
                    format!(
                        "{},from_ready={},\"{:?}\",\"{:?}\"\n",
                        i + 1,
                        p.from_ready,
                        p.approach,
                        p.target,
                    )
                })
                .collect::<String>(),
        )?;
    }
    let active_tool = bundle.robot.robot.active_tool.clone();
    let mut arm = Arm::open(bundle, &assets, args.sim, tx)?;
    if (gains_stage || mechanics_stage) && !args.sim {
        arm.encoder_clock = Some(EncoderClock::open(&arm.bundle.robot)?);
    }
    arm.run_directory = Some(directory.clone());
    arm.ready = Some(ready);
    if !args.sim {
        runtime::realtime(timing.cpu, timing.fifo_priority)?;
    }

    let mut fit = None;
    let mut friction = None;
    let mut ripples = None;
    let mut tuned = None;
    let mut limits = None;
    let mut stiction_ready = None;
    let mut stiction_out = None;
    let mut statuses = vec![
        ("homing", None, "not reached"),
        ("parking", None, "not reached"),
    ];
    if gains_stage {
        statuses.push(("gain poses", None, "not reached"));
    }
    if belt_stage {
        statuses.push(("belt", Some(0), "not reached"));
    }
    if mechanics_stage {
        statuses.push(("gravity", None, "not reached"));
    }
    for (j, selected) in chosen.iter().enumerate() {
        if stiction_stage {
            statuses.push(("stiction ready", Some(j), "not reached"));
            if out_pose.is_some() {
                statuses.push(("stiction arm out", Some(j), "not reached"));
            }
        }
        if mechanics_stage {
            statuses.push(("friction", Some(j), "not reached"));
        }
        if ripple_stage && *selected {
            statuses.push(("ripple", Some(j), "not reached"));
        }
        if gains_stage && *selected {
            statuses.push(("gains", Some(j), "not reached"));
        }
        if limits_stage {
            statuses.push(("limits", Some(j), "not reached"));
        }
    }
    let outcome = (|| {
        arm.initialize()?;
        arm.home()?;
        stage_status(&mut statuses, "homing", None, "passed");
        if let (true, Some(spans)) = (ripple_stage, &spans) {
            ripples = Some(arm.ripple(ready, spans, chosen)?);
            for (j, found) in ripples.as_ref().unwrap().iter().enumerate() {
                if chosen[j] {
                    stage_status(
                        &mut statuses,
                        "ripple",
                        Some(j),
                        if found.as_ref().is_some_and(|r| !r.is_empty()) {
                            "passed"
                        } else {
                            "passed: no compensation improved the ripple"
                        },
                    );
                }
            }
        }
        if let (true, Some(spans)) = (gains_stage, &spans) {
            if let (true, Some(candidate)) = (args.verify_ripple, &candidates) {
                for (j, selected) in chosen.iter().enumerate() {
                    if *selected {
                        let joint = &candidate.joints[j];
                        arm.bus.set_ripple(joint.node_id, &joint.ripple)?;
                    }
                }
            }
            let mut found = match &candidates {
                Some(candidate) => arm.verify_gains(
                    ready,
                    spans,
                    chosen,
                    std::array::from_fn(|j| candidate.joints[j].gains),
                )?,
                None => arm.gains(ready, spans, chosen)?,
            };
            // Every accepted candidate together on the measurement moves. A
            // joint that misbehaves there steps its Kpv one lattice point
            // down (Kiv with it) and the poses are taken again; past
            // POSE_BACKOFF_STEPS it keeps its configured gains. A joint whose
            // configured gains misbehave too is reported and judged no
            // further: the run has nothing better to give it.
            stage_status(&mut statuses, "gain poses", None, "running");
            let mut reverted = [false; N];
            let mut unjudged = [false; N];
            let mut judged = chosen;
            let mut stepped = [0u8; N];
            let mut settled = false;
            for _ in 0..POSE_ATTEMPTS {
                match arm.verify_gain_poses(ready, &gain_poses, judged)? {
                    None => {
                        settled = true;
                        break;
                    }
                    Some((j, kind)) => match found[j] {
                        Some(candidate) if stepped[j] < POSE_BACKOFF_STEPS => {
                            // Oscillation on the way is the velocity loop's;
                            // hunting at the pose is the position loop's,
                            // then the integral's.
                            let mut next = candidate.after;
                            let axis = match kind {
                                PoseFault::Motion => GainAxis::Kpv,
                                PoseFault::Hold if GainAxis::Kpp.snap(next.kpp) > 0 => {
                                    GainAxis::Kpp
                                }
                                PoseFault::Hold => GainAxis::Kiv,
                            };
                            let index = axis.snap(axis.value(next));
                            if index == 0 {
                                found[j] = None;
                                reverted[j] = true;
                                println!(
                                    "gains J{}: misbehaved on a calibration pose at the {} lattice \
                                     floor; configured gains retained",
                                    j + 1,
                                    axis.label()
                                );
                                continue;
                            }
                            axis.set(&mut next, axis.at(index - 1));
                            stepped[j] += 1;
                            found[j] = Some(Tuned {
                                after: next,
                                ..candidate
                            });
                            arm.gain_configure(j, next)?;
                            println!(
                                "gains J{}: misbehaved on a calibration pose ({kind:?}); {} one \
                                 lattice step down to {:.6}, poses taken again",
                                j + 1,
                                axis.label(),
                                axis.value(next)
                            );
                        }
                        Some(_) => {
                            found[j] = None;
                            reverted[j] = true;
                            println!(
                                "gains J{}: misbehaved on a calibration pose after \
                                 {POSE_BACKOFF_STEPS} gain steps; configured gains retained",
                                j + 1
                            );
                        }
                        None => {
                            judged[j] = false;
                            unjudged[j] = true;
                            println!(
                                "gains J{}: misbehaves on a calibration pose with its configured \
                                 gains; reported, not judged further",
                                j + 1
                            );
                        }
                    },
                }
            }
            if !settled {
                return Err(format!(
                    "the calibration poses did not qualify in {POSE_ATTEMPTS} passes"
                )
                .into());
            }
            stage_status(&mut statuses, "gain poses", None, "passed");
            for j in 0..N {
                if chosen[j] {
                    stage_status(
                        &mut statuses,
                        "gains",
                        Some(j),
                        if found[j].is_some() {
                            "passed"
                        } else if unjudged[j] {
                            "retained: configured gains misbehave on the calibration poses"
                        } else if reverted[j] {
                            "retained: posture qualification failed"
                        } else {
                            "retained: no qualified candidate"
                        },
                    );
                }
            }
            for (j, accepted) in found.iter_mut().enumerate() {
                if let Some(accepted) = accepted {
                    accepted.observations = arm.gain_used[j];
                }
            }
            if let (true, Some(candidate)) = (args.verify_ripple, &candidates) {
                if (0..N).all(|j| !chosen[j] || found[j].is_some()) {
                    ripples = Some(std::array::from_fn(|j| {
                        chosen[j].then(|| candidate.joints[j].ripple.clone())
                    }));
                }
            }
            tuned = Some(found);
        }
        if stiction_stage {
            stiction_ready = Some(arm.stiction("ready")?);
            for (j, found) in stiction_ready.as_ref().unwrap().iter().enumerate() {
                stage_status(
                    &mut statuses,
                    "stiction ready",
                    Some(j),
                    if found.is_some() {
                        "passed"
                    } else {
                        // Stiction is reported, never written.
                        "retained: unresolved"
                    },
                );
            }
            if let Some(out) = out_pose {
                arm.pose(out)?;
                stiction_out = Some(arm.stiction("arm out")?);
                for (j, found) in stiction_out.as_ref().unwrap().iter().enumerate() {
                    stage_status(
                        &mut statuses,
                        "stiction arm out",
                        Some(j),
                        if found.is_some() {
                            "passed"
                        } else {
                            "retained: unresolved"
                        },
                    );
                }
                arm.pose(ready)?;
            }
        }
        if let (true, Some(spans)) = (belt_stage, &spans) {
            let captured = arm.belt(0, spans[0])?;
            stage_status(
                &mut statuses,
                "belt",
                Some(0),
                if captured {
                    "passed"
                } else {
                    // The belt is reported, never written.
                    "retained: travel guard"
                },
            );
        }
        if let (true, Some(spans)) = (mechanics_stage, &spans) {
            let mut measured = arm.measure_mechanics(spans)?;
            for (j, found) in measured.iter_mut().enumerate() {
                // A coefficient its own standard error swamps is not a
                // measurement; the file keeps what it had.
                if arm.friction_fit_uncertain[j] {
                    *found = None;
                }
                stage_status(
                    &mut statuses,
                    "friction",
                    Some(j),
                    if found.is_some() {
                        "passed"
                    } else {
                        "retained: fit unresolved"
                    },
                );
            }
            friction = Some(measured);
            fit = Some(arm.identify(&poses, ready)?);
            stage_status(&mut statuses, "gravity", None, "passed");
        }
        if let (true, Some(spans)) = (limits_stage, &spans) {
            if let Some(tuned) = &tuned {
                for (j, accepted) in tuned.iter().enumerate() {
                    if let Some(accepted) = accepted {
                        arm.gain_configure(j, accepted.after)?;
                    }
                }
                arm.check_startup_hold()?;
            }
            let qualified = std::array::from_fn(|j| {
                !gains_stage || tuned.as_ref().is_some_and(|t| t[j].is_some())
            });
            limits = Some(arm.limits(&poses, ready, spans, qualified)?);
            for (j, found) in limits.as_ref().unwrap().iter().enumerate() {
                let status = if !qualified[j] {
                    "skipped: gains not verified"
                } else {
                    match found {
                        Some(f) if f.velocity_reached && f.jerk_measured => "passed",
                        // The caps it reached are written; a jerk it only
                        // bounded is not.
                        Some(_) => "passed: lower bound only",
                        None => "unresolved: no passing limits",
                    }
                };
                stage_status(&mut statuses, "limits", Some(j), status);
            }
        }
        Ok(())
    })();
    arm.encoder_clock = None;
    arm.position_rx_ns = [0; N];
    if outcome.is_err() {
        for (_, _, status) in &mut statuses {
            if *status == "running" {
                *status = "failed";
            }
        }
        // A failed coordinated move may not have reached its saved hold.
        for j in 0..N {
            if let Ok(position) = arm.pos(j) {
                arm.hold[j] = position;
            }
        }
    }
    let mut restored_gains: Result<()> = Ok(());
    if gains_stage {
        for j in 0..N {
            let original = arm.bundle.robot.joints[j].gains;
            restored_gains = restored_gains.and(arm.gain_restore(j, original));
        }
    }
    let mut restored_ripple: Result<()> = Ok(());
    if ripple_stage || args.verify_ripple {
        for (j, selected) in chosen.iter().enumerate() {
            if *selected {
                let joint = &arm.bundle.robot.joints[j];
                let restored = arm
                    .bus
                    .set_ripple(joint.node_id, &joint.ripple)
                    .map_err(Into::into);
                restored_ripple = restored_ripple.and(restored);
            }
        }
        println!(
            "ripple: original settings {} before parking",
            if restored_ripple.is_ok() {
                "restored"
            } else {
                "RESTORE FAILED"
            }
        );
    }
    let restored_gravity = arm
        .kin
        .set_gravity_correction(&config_correction)
        .map_err(Into::into);
    let parked = arm.shutdown();
    stage_status(
        &mut statuses,
        "parking",
        None,
        if parked.is_ok() { "passed" } else { "failed" },
    );
    let gain_used = arm.gain_used;
    let idle = (arm.idle_stops, arm.idle_total_s, arm.idle_longest);
    let correction = fit.as_ref().map(|f| f.correction.clone());
    arm.events.take();
    drop(arm);
    let recorded = recorder.join().map_err(|_| "recording thread failed")?;

    println!(
        "parking: {}",
        if parked.is_ok() {
            "completed"
        } else {
            "FAILED"
        }
    );
    let idle_report = format!(
        "idle: {} stops longer than {IDLE_LIMIT_S}s totalling {:.0}s; longest {:.1}s during J{} {}",
        idle.0,
        idle.1,
        idle.2 .0,
        idle.2 .2 + 1,
        idle.2 .1
    );
    println!("{idle_report}");
    let result = outcome
        .and(restored_gains)
        .and(restored_ripple)
        .and(restored_gravity)
        .and(parked)
        .and(recorded.map_err(Into::into));
    // Retaining the file's value for a joint is a complete outcome: nothing
    // is written for it, so applying the rest is sound.
    let complete = statuses
        .iter()
        .all(|(_, _, status)| status.starts_with("passed") || status.starts_with("retained"));
    let report = statuses.iter().fold(
        String::from("stage\tjoint\tstatus\n"),
        |mut text, (stage, joint, status)| {
            let joint = joint.map_or_else(|| "all".to_owned(), |j| (j + 1).to_string());
            let _ = writeln!(text, "{stage}\t{joint}\t{status}");
            text
        },
    );
    fs::write(directory.join("stages.tsv"), report)?;
    fs::write(
        directory.join("gain-observations.txt"),
        format!("{gain_used:?}\n"),
    )?;
    fs::write(
        directory.join("result.txt"),
        format!(
            "{}\n{}\n{idle_report}\n",
            if complete && result.is_ok() {
                "COMPLETE"
            } else {
                "INCOMPLETE"
            },
            match &result {
                Err(error) => error.to_string(),
                Ok(()) if !complete =>
                    "one or more required stages remain unresolved; see stages.tsv".to_owned(),
                Ok(()) => "all requested stages passed".to_owned(),
            },
        ),
    )?;
    // What was measured is the record, whatever ended the run: the candidate
    // and the run's marker are written here, and the history read from them,
    // before the verdict can return. `--apply` stays gated below on both.
    let candidate = patch_config(
        &original,
        correction.as_deref(),
        friction.as_ref(),
        &sim,
        ripples.as_ref(),
        tuned.as_ref(),
        limits.as_ref(),
    )
    .and_then(|patched| {
        let written = directory.join(par6_config::LOCAL_CONFIG_NAME);
        fs::write(&written, &patched)?;
        ConfigBundle::load_with(&args.config, Some(&written), args.tool.as_deref())?;
        Ok(patched)
    });
    match &candidate {
        Ok(_) => {
            let effective = par6_config::effective_robot_toml(
                &args.config,
                Some(&directory.join(par6_config::LOCAL_CONFIG_NAME)),
            )?;
            fs::write(directory.join("calibrated.toml"), run_record(&effective)?)?;
            fs::write(
                directory.join("run.toml"),
                format!("sim = {}\ntool = {active_tool:?}\n", args.sim),
            )?;
            match history(&directory) {
                Ok(Some((console, table))) => {
                    print!("{console}");
                    if let Err(error) = fs::write(directory.join("history.tsv"), table) {
                        println!("history: not saved: {error}");
                    }
                }
                Ok(None) => {}
                Err(error) => println!("history: unavailable: {error}"),
            }
        }
        Err(error) => println!("candidate: not written: {error}"),
    }
    result?;

    if let Some(fit) = &fit {
        let mut report = format!(
            "# Arm links identified from static torque, with the base attachment fitted.\n\
             # Torque residual {:.5} Nm, against {:.5} Nm for the model as it stood.\n\
             # `determined` is the share of each parameter the poses fixed, 0..1.\n",
            fit.rms_nm, fit.rms_before_nm
        );
        for (body, chunk) in fit.correction.chunks(4).enumerate() {
            writeln!(
                report,
                "\n[body{}]\nd_mass_kg = {:?}\nd_first_moment_kg_m = [{:?}, {:?}, {:?}]\n\
                 determined = {:?}",
                body + 1,
                chunk[0],
                chunk[1],
                chunk[2],
                chunk[3],
                &fit.determined[body * 4..body * 4 + 4]
            )?;
        }
        fs::write(directory.join("identified-arm.toml"), report)?;
        println!(
            "identification: torque residual {:.5} Nm -> {:.5} Nm; {} of {} parameters fixed",
            fit.rms_before_nm,
            fit.rms_nm,
            fit.determined.iter().filter(|d| **d > 0.5).count(),
            fit.determined.len()
        );
    }

    // `fit_arm` solves for a correction ON TOP OF everything the model
    // already carries, the installed `gravity_correction` included, so the
    // value belonging in the file is installed + fitted. Writing the delta
    // alone made a second --apply throw away the first run's masses while
    // reporting an improved residual.
    let installed = &config_correction;
    let correction: Option<Vec<f64>> = correction.map(|delta| {
        delta
            .iter()
            .enumerate()
            .map(|(i, d)| d + installed.get(i).copied().unwrap_or(0.0))
            .collect()
    });
    if correction.is_some() && !stale_scale.is_empty() {
        println!(
            "gravity_scale was not unity on {stale_scale:?}; the identification supersedes it \
             and it is written back as 1.0"
        );
    }
    if let Some(limits) = &limits {
        for (j, found) in limits.iter().enumerate() {
            match found {
                Some(f) => println!(
                    "limits J{}: velocity {:.4} rad/s{}, acceleration {:.4} rad/s2, jerk {:.4} rad/s3{}, \
                     speed ripple {:.1}%",
                    j + 1,
                    f.caps.velocity,
                    if f.velocity_reached { "" } else { " (lower bound)" },
                    f.caps.acceleration,
                    f.caps.jerk,
                    if f.jerk_measured {
                        ""
                    } else {
                        " (lower bound: no probe move was limited by it; not written)"
                    },
                    100.0 * f.ripple
                ),
                None => println!(
                    "limits J{}: misses the requirements even at {LIMITS_MIN_FACTOR} of its EXEC \
                     limits; left unchanged",
                    j + 1
                ),
            }
        }
    }
    for (label, table) in [("ready", &stiction_ready), ("arm out", &stiction_out)] {
        if let Some(table) = table {
            let joints: Vec<String> = table
                .iter()
                .enumerate()
                .filter_map(|(j, s)| s.map(|s| format!("J{} {s:.3}", j + 1)))
                .collect();
            println!("stiction at {label}: {} Nm", joints.join(", "));
        }
    }
    if let Some(ripples) = &ripples {
        for (j, r) in ripples.iter().enumerate() {
            let Some(r) = r else {
                continue;
            };
            let harmonics: Vec<String> = r
                .iter()
                .map(|h| format!("h{} {}/{} mA", h.harmonic, h.a_ma, h.b_ma))
                .collect();
            println!(
                "ripple J{}: {}",
                j + 1,
                if harmonics.is_empty() {
                    "none".to_owned()
                } else {
                    harmonics.join(", ")
                }
            );
        }
    }
    if let Some(tuned) = &tuned {
        for (j, t) in tuned.iter().enumerate() {
            if chosen[j] && t.is_none() {
                println!("gains J{}: unchanged (no gain change accepted)", j + 1);
            }
            if let Some(t) = t {
                println!(
                    "gains J{}: kpv {:.6} -> {:.6}, kiv {:.8} -> {:.8}, kpp {:.5} -> {:.5}; {} observations",
                    j + 1, t.before.kpv, t.after.kpv, t.before.kiv, t.after.kiv,
                    t.before.kpp, t.after.kpp, t.observations
                );
            }
        }
    }
    let patched = candidate?;
    if !complete {
        return Err(format!(
            "calibration incomplete; see {}/stages.tsv; candidate saved but not applied",
            directory.display()
        )
        .into());
    }
    if args.apply && args.sim {
        return Err("--apply writes masses fitted to the simulator; refusing".into());
    }
    if args.apply {
        let backup = overlay.with_extension("toml.before-selfcal");
        if existing.is_some() && !backup.exists() {
            fs::write(backup, &original)?;
        }
        let temp = overlay.with_extension("toml.selfcal-tmp");
        fs::write(&temp, &patched)?;
        fs::rename(temp, &overlay)?;
        println!("applied to {}", overlay.display());
    } else {
        println!(
            "results: {}",
            directory.join(par6_config::LOCAL_CONFIG_NAME).display()
        );
    }
    Ok(())
}

/// A run's record of the config it would leave the arm with, for the
/// history: the robot document without its installation shapes, which the
/// robot schema on its own does not read.
fn run_record(effective: &str) -> Result<String> {
    let mut table: toml::Table = toml::from_str(effective)?;
    table.remove("installation_shapes");
    Ok(toml::to_string(&table)?)
}

/// Save/restore around the real-time transition. The syscalls themselves are
/// `par6_rt::rt`'s -- including its `RLIMIT_RTPRIO` check, which this file's
/// own copy lacked, so a priority the box refuses now lowers to the ceiling
/// instead of silently leaving the loop at ordinary priority. What stays here
/// is the part the daemon has no use for: the daemon owns its process for its
/// whole life, while selfcal has to hand the terminal back.
mod runtime {
    use std::{io, time::Duration};

    /// The prior affinity mask, scheduler and parameters, restored on drop.
    pub struct Runtime(libc::cpu_set_t, i32, libc::sched_param);

    impl Runtime {
        pub fn prepare(cpu: i64) -> io::Result<Self> {
            if !(0..i64::from(libc::CPU_SETSIZE)).contains(&cpu) {
                return Err(io::Error::other("invalid control CPU"));
            }
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
            // Keep everything else off the control CPU for the run.
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

    /// Pin, lock and raise this thread, at the highest priority the box
    /// actually permits.
    pub fn realtime(cpu: i64, priority: u8) -> io::Result<()> {
        use par6_rt::rt::{
            lock_memory, permitted_priority, pin_to_cpu, rtprio_ceiling, set_fifo_priority,
        };
        let map = io::Error::other;
        pin_to_cpu(cpu as usize).map_err(map)?;
        lock_memory().map_err(map)?;
        let prio = permitted_priority(priority, rtprio_ceiling());
        if prio == 0 {
            return Err(io::Error::other(
                "this process may not use SCHED_FIFO at all (RLIMIT_RTPRIO is 0); the tick \
                 loop would run at ordinary priority",
            ));
        }
        set_fifo_priority(prio).map_err(map)
    }

    pub fn now() -> Duration {
        Duration::from_nanos(par6_rt::rt::monotonic_ns())
    }

    pub fn sleep(deadline: Duration) {
        par6_rt::rt::sleep_until(deadline.as_nanos() as u64);
    }
}
