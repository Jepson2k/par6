//! Measure a PAR6: its mechanics and its link masses.
//!
//! Four phases, in this order and for this reason:
//!
//! 1. HOME: find every joint's reference against its endstop or hall.
//! 2. MEASURE: each joint's inertia from the model and its friction from a
//!    constant-velocity sweep. Nothing is tuned: the drive loops run at
//!    6250 Hz, which a 250 Hz link cannot observe, so their gains stand.
//! 3. IDENTIFY: hold twenty poses and fit this arm's own link masses, which
//!    are 3D printed and so are not the vendor's.
//! 4. PARK: return the shoulder and elbow to their stops, then release.
//!
//! CALIBRATION RULE: no invented heuristics. Every metric, tuning method and
//! numerical decision rule cites a primary source beside it, and says what that
//! source supports. A citation for a formula does not justify a threshold.
//!
//! A number that is the same on every run is a constant here, not a flag. The
//! justification still gets written down; it costs one comment instead of a
//! flag, a parser, a struct field and a log line to keep in sync.

use par6_bus::{
    hw::SocketCanBus,
    sim::{
        scene::{Scene, Tool},
        SimBus,
    },
    spectral::{torque_to_ma_factor, JointConversion},
    BusState, ConfigKind, DriveTune, DriverBus, ErrorFlags, GripperCommand, JointCommand,
    PollAction, PollKind, RuntimeBus,
};
use par6_config::{ConfigBundle, Gains, LimitMode, PreMove};
use par6_motion::{SSeptic, SEPTIC_PEAK_ACC, SEPTIC_PEAK_VEL};
use par6_rt::homing::{Homer, HomerEvent, HomerParams};
use std::{
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
/// Arc one leg traverses. Both directions run over it, so the joint returns
/// to where it started and six joints stay well inside their soft limits.
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

/// How close to a known endstop a sample stops counting as tracking. User
/// requirement, 2026-09-21: within this much of a stop a joint is breaking
/// away from, or settling into, a mechanical limit, and what it does there
/// says nothing about its gains.
const ENDSTOP_EXCLUSION_RAD: f64 = 5.0 * std::f64::consts::PI / 180.0;

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
/// Fraction of a joint's EXEC caps the gains probe moves at: fast enough
/// for a loop that hunts to show it, slow enough that a joint at its
/// limits is not what is being scored.
const GAINS_PROBE_FRACTION: f64 = 0.5;
/// Integral gain steps: halve going down, one and a half going up. Every
/// gain that was hand-tuned on 2026-09-23 moved by a factor in that range
/// (J1 kiv x2, J6 kiv /3); a doubling of kpv with a quadrupling of kiv in
/// one step is what buzzed J1.
const GAINS_STEP_DOWN: f64 = 0.5;
const GAINS_STEP_UP: f64 = 1.5;
/// Proportional gain steps, gentler: kpv changes the loop's crossover.
const GAINS_KPV_STEP_DOWN: f64 = 0.75;
const GAINS_KPV_STEP_UP: f64 = 1.25;
/// How far a search may walk from the configured value, either way.
const GAINS_MIN_FACTOR: f64 = 0.1;
const GAINS_MAX_FACTOR: f64 = 3.0;
/// Least a gains probe move lasts \[s\]: at half caps a wrist's short probe
/// is under 0.5 s, inside the scoring guard, and scores nothing.
const GAINS_PROBE_MIN_S: f64 = 1.5;
/// A step must beat the best score by this fraction to count: on the
/// simulator J3's trials differ by under 1%, which is repeat noise, and a
/// search that follows noise walks the gains for nothing.
const GAINS_MIN_IMPROVEMENT: f64 = 0.05;
/// A speed error this large for `RUNAWAY_TICKS` in a row is a loop that
/// has gone unstable, not a rough joint: J2's backlash chatter spikes to
/// 77 deg/s for single samples, the J1 buzz sat at +-240 deg/s.
const RUNAWAY_RAD_S: f64 = 90.0 * std::f64::consts::PI / 180.0;
const RUNAWAY_TICKS: u32 = 3;

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

/// What the gains stage settled on for one joint.
#[derive(Clone, Copy, Debug)]
struct Tuned {
    before: Gains,
    after: Gains,
    /// Speed RMS off the profile on the probe move, before and after
    /// \[rad/s\].
    score_before: f64,
    score_after: f64,
}

/// Waiting out one silent drive is normal; a run that spends its time doing
/// nothing else has a bus problem no amount of waiting will fix.
const MAX_RECOVERIES: u32 = 24;
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
    #[arg(long, default_value = "calibration-runs")]
    output_dir: PathBuf,
    /// Run against the simulator instead of the arm.
    #[arg(long)]
    sim: bool,
    /// Write the identified gravity correction into the config.
    #[arg(long)]
    apply: bool,
    /// Also find each joint's velocity, acceleration and jerk limits, and
    /// with `--apply` write them as its EXEC limits. Off by default: it
    /// drives every joint to the edge of what it can do.
    #[arg(long)]
    limits: bool,
    /// Home, then run only the limits stage: skip the friction sweep and the
    /// identification, and leave the gravity correction as configured.
    #[arg(long)]
    limits_only: bool,
    /// Also tune each joint's velocity-loop gains (kiv, then kpv) on a probe
    /// move, and with `--apply` write them. Runs before the limits stage,
    /// which depends on them. Off by default.
    #[arg(long)]
    gains: bool,
    /// Home, then run only the gains stage.
    #[arg(long)]
    gains_only: bool,
    /// The tool fitted on the arm now, by its config name (e.g. `Flange`).
    /// Defaults to the config's `active_tool`; the file is not changed.
    #[arg(long)]
    tool: Option<String>,
}

// ---------------------------------------------------------------- events

#[derive(Clone, Copy, Debug)]
#[allow(clippy::large_enum_variant)] // Fixed-size entries: no allocation on the tick path.
enum Event {
    Phase(&'static str, usize),
    Tool(u8, bool),
    Configure(u64, usize, f64, i32),
    Hold(u64, usize, &'static str, f64, f64),
    Move(u64, usize, Outcome, f64, i32, i32, Measure),
    Contact(u64, usize, i64, f64, f64, bool, bool),
    FeedbackGap(u64, usize, f64),
    Drag(u64, usize, f64, f64, f64),
    Mechanics(u64, usize, f64, f64, f64),
    LimitsStep(u64, usize, Caps, Option<&'static str>),
    Limits(u64, usize, Found),
    GainsStep(u64, usize, Gains, Option<f64>, &'static str),
    Gains(u64, usize, Tuned),
    IdentPose(usize, usize, [f64; N]),
    IdentTorque(usize, [f64; N]),
    ArmFit(f64, f64),
    Sample(
        u64,
        [JointCommand; N],
        [i32; N],
        [i32; N],
        [i32; N],
        [bool; N],
    ),
}

/// `None` means the event belongs in a csv, not the console.
fn describe(event: &Event) -> Option<String> {
    Some(match *event {
        Event::Sample(..) => return None,
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
        Event::LimitsStep(tick, j, c, failed) => format!(
            "tick={tick} J{} LIMITS step velocity={:.4}rad/s acceleration={:.4}rad/s2 \
             jerk={:.4}rad/s3 {}",
            j + 1,
            c.velocity,
            c.acceleration,
            c.jerk,
            failed.map_or("passed".to_owned(), |why| format!("failed: {why}"))
        ),
        Event::GainsStep(tick, j, g, score, verdict) => format!(
            "tick={tick} J{} GAINS step kpv={:.5} kiv={:.5} {} {verdict}",
            j + 1,
            g.kpv,
            g.kiv,
            score.map_or("unstable".to_owned(), |s| format!(
                "score={:.3}deg/s",
                s.to_degrees()
            ))
        ),
        Event::Gains(tick, j, t) => format!(
            "tick={tick} J{} GAINS kpv={:.5}->{:.5} kiv={:.5}->{:.5} score={:.3}->{:.3}deg/s",
            j + 1,
            t.before.kpv,
            t.after.kpv,
            t.before.kiv,
            t.after.kiv,
            t.score_before.to_degrees(),
            t.score_after.to_degrees()
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
        "tick,joint,command,position_ticks,speed_ticks_s,current_ma,drive_fault"
    )?;
    let mut line = String::new();
    while let Ok(event) = rx.recv() {
        if let Event::Sample(tick, cmd, pos, speed, current, fault) = event {
            line.clear();
            for j in 0..N {
                let _ = writeln!(
                    line,
                    "{tick},{},\"{:?}\",{},{},{},{}",
                    j + 1,
                    cmd[j],
                    pos[j],
                    speed[j],
                    current[j],
                    u8::from(fault[j])
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

// ---------------------------------------------------------------- measurement

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum Outcome {
    #[default]
    Timeout,
    Complete,
    /// The joint stopped while loaded: a mechanical stop.
    Blocked,
    /// The loop ran away under trial gains; the move finished under the
    /// last sane ones.
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
    settled_error_rad: f64,
    hold_rms_rad_s: f64,
    /// How far this move missed the acceptance limits, as a fraction of them.
    /// Zero passes. One scale, no offset: an earlier design carried
    /// `1.0 + violation` in one place and the raw value in another, and a
    /// threshold comparing against the wrong one silently never fired.
    excess: f64,
    elapsed: f64,
    outcome: Outcome,
    /// Where the joint stopped, when it stopped against something.
    stop: Option<i32>,
}

impl Measure {
    fn outcome_complete(&mut self, at: i32) {
        self.outcome = Outcome::Complete;
        self.stop = Some(at);
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
    samples: [(u64, i32); 8],
    len: usize,
}
impl Ring {
    /// Push a fresh sample; return counts/second measured back to the newest
    /// sample that is at least `window` ticks old, once there is one.
    fn push(&mut self, tick: u64, position: i32, window: u64, dt: f64) -> Option<f64> {
        let mut speed = None;
        for k in 0..self.len {
            let (then, was) = self.samples[k];
            if tick.saturating_sub(then) >= window {
                speed = Some((f64::from(position) - f64::from(was)) / ((tick - then) as f64 * dt));
            }
        }
        if self.len < self.samples.len() {
            self.samples[self.len] = (tick, position);
            self.len += 1;
        } else {
            self.samples.rotate_left(1);
            self.samples[self.len - 1] = (tick, position);
        }
        speed
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
    found_at: [i32; N],
    generation: [u64; N],
    seen: [u64; N],
    /// A contact this joint is at. Samples within `ENDSTOP_EXCLUSION_RAD` of
    /// it are not scored.
    endstop_guard: [Option<i32>; N],
    /// While the gains stage trials a joint: the last gains that tracked,
    /// restored within a tick if the trial runs away.
    sane_gains: [Option<Gains>; N],
    recoveries: u32,
    /// Homing drives unreferenced joints toward the stop at the configured
    /// homing current, not the operating one.
    homing: bool,
    /// Shutdown: watch only this joint, and tolerate silence.
    only: Option<usize>,
    blind: bool,
    stopping: bool,
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
            found_at: [0; N],
            generation: [0; N],
            seen: [0; N],
            endstop_guard: [None; N],
            sane_gains: [None; N],
            recoveries: 0,
            homing: false,
            only: None,
            blind: false,
            stopping: false,
            tick: 0,
            deadline: Duration::ZERO,
            simulated,
        })
    }

    /// Recording is best effort. It must never fail a motion or a park: a
    /// full disk used to error every `emit`, which failed every park and
    /// then released a loaded arm anyway.
    fn emit(&self, event: Event) {
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
        for j in 0..N {
            let node = &self.state.nodes[self.node(j)];
            if node.position_generation != self.generation[j] {
                self.generation[j] = node.position_generation;
                self.seen[j] = self.tick;
            }
        }
        self.bus.poll_step()?;
        self.bus.send_joint_commands(&commands)?;
        self.bus.send_gripper(&GripperCommand::NoGripper)?;
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
        ));
        Ok(())
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
        let feedforward = self.gravity_feedforward();
        let mut commands: [JointCommand; N] =
            std::array::from_fn(|j| JointCommand::position(self.hold[j], 0, feedforward[j]));
        if let Some((j, cmd)) = active {
            commands[j] = cmd;
        }
        self.exchange(commands, true)
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
        Ok(())
    }
    fn operating(&mut self, j: usize) -> Result<()> {
        self.configure(j, self.bundle.robot.joints[j].ilim_ma)
    }

    /// Poll until every drive reports position, speed and current, ramp the
    /// holding current up, check the bus agrees about the fitted tool, and
    /// confirm every joint holds still before anything moves.
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
        let held = self.measure_hold("startup", None)?;
        for (j, (offset, speed)) in held.iter().enumerate() {
            if *offset > self.tolerance() || *speed > self.holding_limit(j) {
                return Err(format!(
                    "J{} will not hold still at startup: {:.4}deg, {:.4}deg/s",
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
    fn measure_hold(
        &mut self,
        why: &'static str,
        velocity_joint: Option<usize>,
    ) -> Result<[(f64, f64); N]> {
        let mut rings: [Ring; N] = std::array::from_fn(|_| Ring::default());
        let mut sums = [(0.0_f64, 0.0_f64, 0u32); N];
        let mut generation = self.generation;
        let window = self.ticks(SPEED_WINDOW_S);
        for _ in 0..self.ticks(HOLD_OBSERVATION_S) {
            self.frame(velocity_joint.map(|j| (j, JointCommand::velocity(0, 0))))?;
            for j in 0..N {
                if generation[j] == self.generation[j] {
                    continue;
                }
                generation[j] = self.generation[j];
                let p = self.pos(j)?;
                let per_tick = self.per_tick(j);
                let offset = (f64::from(p) - f64::from(self.hold[j])) * per_tick;
                let speed = rings[j].push(self.tick, p, window, self.dt).unwrap_or(0.0) * per_tick;
                sums[j].0 += offset * offset;
                sums[j].1 += speed * speed;
                sums[j].2 += 1;
            }
        }
        let out: [(f64, f64); N] = std::array::from_fn(|j| {
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
        let mut guard_contact = ContactGuard::new(self.ticks(STALL_WINDOW_S), start, expected);
        let guard = self.ticks(DETECT_GUARD_S).max(self.ticks(ramp));
        let speed_window = self.ticks(SPEED_WINDOW_S);
        let mut generation = self.generation[j];
        let started = self.tick;
        let mut runaway_ticks = 0u32;
        let mut unstable = false;
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
            let measured = ring.push(self.tick, p, speed_window, self.dt);
            let error = (f64::from(p) - reference) * per_tick;
            // Two exclusions on the scoring window. The guard drops the ramp
            // and the drive's velocity integral unwinding from the previous
            // command -- a spike there used to fail a gain that tracked. The
            // endstop band drops samples taken within ENDSTOP_EXCLUSION_RAD of
            // a contact this joint is breaking away from or settling into,
            // where what it does says nothing about its gains (user
            // requirement, 2026-09-21).
            let near_stop = self.endstop_guard[j].is_some_and(|contact| {
                ((f64::from(p) - f64::from(contact)) * per_tick).abs() < ENDSTOP_EXCLUSION_RAD
            });
            if t >= guard && !near_stop {
                out.peak_error_rad = out.peak_error_rad.max(error.abs());
                out.samples += 1;
                out.position_sq += error * error;
                if let Some(v) = measured {
                    let speed_error = (v - f64::from(reference_speed)) * per_tick;
                    out.speed_sq += speed_error * speed_error;
                    // A trial gain that runs away is caught here, and the
                    // last sane gains are back on the drive before the
                    // next frame; the move goes on to its landing under
                    // them, scored as unstable.
                    runaway_ticks = if speed_error.abs() > RUNAWAY_RAD_S {
                        runaway_ticks + 1
                    } else {
                        0
                    };
                    if runaway_ticks >= RUNAWAY_TICKS && !unstable {
                        if let Some(sane) = self.sane_gains[j] {
                            unstable = true;
                            self.gains[j] = sane;
                            self.retune(j, limit)?;
                        }
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
                out.outcome_complete(p);
                if unstable {
                    out.outcome = Outcome::Unstable;
                }
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
                    out.stop = Some(p);
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
        out.settled_error_rad = ((f64::from(self.pos(j)?) - f64::from(target)) * per_tick).abs();
        let held = self.measure_hold("after move", None)?;
        out.hold_rms_rad_s = held[j].1;
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

/// Contact detection over a sliding window: the vendor's rule, which is that a
/// joint drawing current while its encoder range stays under a quarter of the
/// Contact detection for an ordinary move: a displacement plateau while the
/// drive is pulling current, which is how a move that jams reports itself.
///
/// Not the homing stall detector -- `par6-rt`'s `Homer` owns that, with the
/// vendor window and current-ratio rules, and this is the weaker predicate a
/// move that is not approaching a stop needs.
/// commanded travel is against something.
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
        for step in self.bundle.robot.homing.sequence.clone() {
            for m in step.pre_moves {
                self.premove(m)?;
            }
            if let Some(group) = step.home {
                for j in group.joints {
                    let j = usize::from(j);
                    self.emit(Event::Phase("home", j));
                    // Reference, limits and the post-home move are the FSM's;
                    // this only records that the joint is referenced. The
                    // contact it just left is no longer where it is, so
                    // nothing after this may be excluded by it.
                    self.home_joint(j)?;
                    self.endstop_guard[j] = None;
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
        for m in self.bundle.robot.homing.post_moves.clone() {
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
        self.endstop_guard[j] = None;
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

    /// Joint-side inertia at `q`: the mass-matrix diagonal, plus the rotor
    /// reflected through the reduction.
    ///
    /// `Kin::dyn_feedforward` is the inverse dynamics with the gravity term
    /// subtracted back out, so zero velocity and a unit acceleration on one
    /// joint alone leave exactly `M_jj(q)` in that slot. The rotor table is
    /// motor-side and reflects as `G^2 jm` through the dynamics ratio -- the
    /// vendor's J1 reduction disagrees with its kinematic one, so this uses
    /// the same fallback `par6-bus` does.
    fn inertia(&mut self, q: [f64; N]) -> Result<[f64; N]> {
        let zero = [0.0; N];
        let mut out = [0.0; N];
        for j in 0..N {
            let mut qdd = zero;
            qdd[j] = 1.0;
            let mut tau = zero;
            self.kin
                .dyn_feedforward(&q, &zero, &qdd, &mut tau)
                .map_err(|e| format!("J{} inertia: {e}", j + 1))?;
            let cfg = &self.bundle.robot.joints[j];
            let g = cfg.dynamics_gear_ratio.unwrap_or(cfg.gear_ratio);
            out[j] = tau[j] + g * g * self.bundle.robot.sim.motor_jm_kg_m2[j];
        }
        Ok(out)
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
    fn drag(&mut self, j: usize, ticks_s: f64) -> Result<Option<f64>> {
        let per_tick = self.per_tick(j).abs();
        let profile = self.speed_ramp(j, ticks_s * per_tick);
        let ramp = self.ticks(profile.duration());
        let settle = ramp + self.ticks(DRAG_SETTLE_S);
        let hold = settle + self.ticks(DRAG_AVERAGE_S);
        let mut ring = Ring::default();
        let window = self.ticks(SPEED_WINDOW_S);
        let mut generation = self.generation[j];
        let mut current = 0.0;
        let mut speed = 0.0;
        let mut n = 0.0;
        for t in 0..hold + ramp {
            let fraction = if t < ramp {
                profile.sample((t + 1) as f64 * self.dt).0
            } else if t < hold {
                1.0
            } else {
                1.0 - profile.sample((t - hold + 1) as f64 * self.dt).0
            };
            let cmd = JointCommand::velocity((ticks_s * fraction) as i32, 0);
            self.frame(Some((j, cmd)))?;
            if self.generation[j] == generation {
                continue;
            }
            generation = self.generation[j];
            let p = self.pos(j)?;
            let ma = self.state.nodes[self.node(j)]
                .current_ma
                .ok_or("missing current feedback")?;
            if let Some(v) = ring.push(self.tick, p, window, self.dt) {
                if (settle..hold).contains(&t) {
                    current += f64::from(ma);
                    speed += v;
                    n += 1.0;
                }
            }
        }
        // The ramp has already brought it to rest; hold where it stopped.
        self.adopt(j)?;
        self.settle(DRAG_RECOVER_S)?;
        if n < 1.0 {
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
        Ok(Some(current / n))
    }

    /// Viscous and Coulomb friction for one joint \[Nm.s/rad, Nm\], joint side.
    ///
    /// Each speed is held both ways over the same arc. Gravity is
    /// position-dependent but direction-independent, so averaging the two
    /// currents cancels it exactly -- the same trick the gravity sweep uses on
    /// its approach directions, and it means no gravity model enters the
    /// friction fit at all:
    ///
    ///   forward:  +I_f/k = +b v + tc + tau_g
    ///   reverse:  -I_r/k = -b v - tc + tau_g
    ///   half diff: (I_f + I_r)/2k = b v + tc
    ///
    /// Two parameters, ordinary least squares over the speeds that held.
    fn friction(&mut self, j: usize) -> Result<(f64, f64)> {
        let cfg = &self.bundle.robot.joints[j];
        let factor =
            torque_to_ma_factor(cfg.gear_ratio, cfg.gear_efficiency, cfg.kt_nm_a, cfg.dir).abs();
        let per_tick = self.per_tick(j).abs();
        // The fastest leg that still fits the travel budget, so a joint near
        // its limits is bounded by geometry rather than by luck. A leg at `v`
        // covers `v * span` holding and `v * T` across its two ramps (each
        // septic averages half its end value), and `T` grows with `v`, so the
        // speed is found by bisection on that travel.
        let span = DRAG_SETTLE_S + DRAG_AVERAGE_S;
        let travel = |v: f64| v * (span + self.speed_ramp(j, v).duration());
        let (mut lo, mut hi) = (0.0, cfg.limits.for_mode(LimitMode::Exec).velocity_rad_s);
        if travel(hi) > DRAG_TRAVEL_RAD {
            for _ in 0..50 {
                let mid = 0.5 * (lo + hi);
                if travel(mid) > DRAG_TRAVEL_RAD {
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
            let Some(forward) = self.drag(j, ticks_s)? else {
                continue;
            };
            let Some(reverse) = self.drag(j, -ticks_s)? else {
                continue;
            };
            let torque = (forward.abs() + reverse.abs()) / 2.0 / factor;
            let speed = ticks_s * per_tick;
            self.emit(Event::Drag(self.tick, j, speed, forward, reverse));
            rows.push((speed, torque));
        }
        if rows.len() < 2 {
            return Err(format!(
                "J{} held {} of {DRAG_LEVELS} speeds; friction needs two",
                j + 1,
                rows.len()
            )
            .into());
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
        Ok((b.max(0.0), tc.max(0.0)))
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
    fn measure_mechanics(&mut self) -> Result<[Option<(f64, f64)>; N]> {
        let legs: Vec<usize> = (0..N)
            .filter(|j| self.only.is_none_or(|o| o == *j))
            .collect();
        let q = self.angles()?;
        let inertia = self.inertia(q)?;
        let mut friction = [None; N];
        for &j in &legs {
            self.emit(Event::Phase("friction", j));
            let (b, tc) = self.friction(j)?;
            self.emit(Event::Mechanics(self.tick, j, inertia[j], b, tc));
            friction[j] = Some((b, tc));
        }
        Ok(friction)
    }
}

impl Arm {
    // ------------------------------------------------------------ limits

    /// Tune each joint's velocity loop on one probe move: the septic at
    /// `GAINS_PROBE_FRACTION` of its EXEC caps, out and back, scored by how
    /// far the drive's own speed strays from the profile (speed RMS off the
    /// commanded speed, the same number the limits stage reports as ripple).
    ///
    /// This is a search, not a placement: a 250 Hz bus cannot see the
    /// 6250 Hz loop it is tuning, but it can see whether a move got
    /// smoother. Hand-tuned on 2026-09-23 this way, J6 went from 37 to
    /// 2.6 deg/s off the profile (kiv /3) and J1 halved its 12 Hz surge (kiv
    /// x2, kpv x1.33). kiv first, then kpv; each walks down from the
    /// configured value while the score improves, up only when down did
    /// not help, never past `GAINS_MIN_FACTOR`/`GAINS_MAX_FACTOR`. A trial
    /// that runs away is caught inside the move (`RUNAWAY_RAD_S`), the last
    /// sane gains go back on the drive within a tick, and that direction
    /// ends.
    fn gains(&mut self, ready: [f64; N], spans: &[(f64, f64); N]) -> Result<[Option<Tuned>; N]> {
        let mut tuned = [None; N];
        for (j, slot) in tuned.iter_mut().enumerate() {
            self.pose(ready)?;
            self.emit(Event::Phase("gains", j));
            *slot = Some(self.joint_gains(j, ready[j], spans[j])?);
            if let Some(t) = *slot {
                self.emit(Event::Gains(self.tick, j, t));
            }
        }
        self.pose(ready)?;
        Ok(tuned)
    }

    fn joint_gains(&mut self, j: usize, home: f64, span: (f64, f64)) -> Result<Tuned> {
        let caps = self
            .exec_caps(j)
            .scaled(GAINS_PROBE_FRACTION, &self.ceiling_caps(j));
        let (target, _) = self.probe_short(home, span, caps);
        let before = self.gains[j];
        let score_before = self
            .gains_trial(j, home, target, caps, before)?
            .ok_or_else(|| format!("J{} runs away on its configured gains", j + 1))?;
        let mut best = (before, score_before);
        for field in [GainField::Kiv, GainField::Kpv] {
            let base = field.get(&before);
            let (down, up) = field.steps();
            // Down first: the safe direction. Up only when down did not help.
            let mut improved = false;
            loop {
                let value = field.get(&best.0) * down;
                if value < base * GAINS_MIN_FACTOR {
                    break;
                }
                let g = field.with(best.0, value);
                match self.gains_trial(j, home, target, caps, g)? {
                    Some(score) if score < best.1 * (1.0 - GAINS_MIN_IMPROVEMENT) => {
                        best = (g, score);
                        improved = true;
                    }
                    _ => break,
                }
            }
            if improved {
                continue;
            }
            loop {
                let value = field.get(&best.0) * up;
                if value > base * GAINS_MAX_FACTOR {
                    break;
                }
                let g = field.with(best.0, value);
                match self.gains_trial(j, home, target, caps, g)? {
                    Some(score) if score < best.1 * (1.0 - GAINS_MIN_IMPROVEMENT) => {
                        best = (g, score);
                    }
                    _ => break,
                }
            }
        }
        self.gains[j] = best.0;
        self.retune(j, self.bundle.robot.joints[j].ilim_ma)?;
        Ok(Tuned {
            before,
            after: best.0,
            score_before,
            score_after: best.1,
        })
    }

    /// One trial: the probe out and back under `gains`, the joint left at
    /// `home`. `None` when the loop ran away, in which case the drive is
    /// already back on the last sane gains.
    fn gains_trial(
        &mut self,
        j: usize,
        home: f64,
        target: f64,
        caps: Caps,
        gains: Gains,
    ) -> Result<Option<f64>> {
        let sane = self.sane_gains[j].unwrap_or(self.gains[j]);
        self.sane_gains[j] = Some(sane);
        self.gains[j] = gains;
        let mut score = 0.0_f64;
        let mut stable = true;
        for point in [target, home] {
            let ticks = self.conv[j].motor_ticks(point);
            let m = self.run_motion_capped(j, ticks, GAINS_PROBE_MIN_S, false, caps)?;
            if m.outcome != Outcome::Complete {
                stable = false;
            }
            score = score.max(m.rms_speed());
        }
        if stable {
            self.sane_gains[j] = Some(gains);
        } else {
            // Whatever ran away, the drive holds the sane gains now; make
            // the bookkeeping say the same.
            self.gains[j] = sane;
            self.run_motion_capped(j, self.conv[j].motor_ticks(home), self.dt, false, caps)?;
        }
        let verdict = if !stable {
            "runaway; last sane gains restored"
        } else {
            "scored"
        };
        self.emit(Event::GainsStep(
            self.tick,
            j,
            gains,
            stable.then_some(score),
            verdict,
        ));
        Ok(stable.then_some(score))
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
    ) -> Result<[Option<Found>; N]> {
        let worst = self.worst_gravity_ma(poses, ready)?;
        let mut found = [None; N];
        for (j, slot) in found.iter_mut().enumerate() {
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

/// Every joint's probe span, planned before anything moves: loading the
/// collision world and sweeping each span takes far longer than a control
/// tick, and on the arm the loop would miss its deadline doing it.
/// The two velocity-loop gains the gains stage searches, in search order.
#[derive(Clone, Copy)]
enum GainField {
    Kiv,
    Kpv,
}

impl GainField {
    fn get(self, g: &Gains) -> f64 {
        match self {
            GainField::Kiv => g.kiv,
            GainField::Kpv => g.kpv,
        }
    }
    fn with(self, mut g: Gains, value: f64) -> Gains {
        match self {
            GainField::Kiv => g.kiv = value,
            GainField::Kpv => g.kpv = value,
        }
        g
    }
    /// (down, up) step factors.
    fn steps(self) -> (f64, f64) {
        match self {
            GainField::Kiv => (GAINS_STEP_DOWN, GAINS_STEP_UP),
            GainField::Kpv => (GAINS_KPV_STEP_DOWN, GAINS_KPV_STEP_UP),
        }
    }
}

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
        let tolerance = self.tolerance();
        let current = self.angles()?;
        if (0..N).all(|j| (q[j] - current[j]).abs() <= tolerance) {
            return Ok(());
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
        for t in 0..total {
            let (position, velocity, _) = profile.sample((t + 1) as f64 * self.dt);
            let commands: [JointCommand; N] = std::array::from_fn(|j| {
                let distance = f64::from(target[j]) - f64::from(start[j]);
                JointCommand::position(
                    (f64::from(start[j]) + distance * position).round() as i32,
                    (distance * velocity) as i32,
                    0,
                )
            });
            self.exchange(commands, true)?;
            if t + 1 >= moving
                && (0..N).all(|j| {
                    self.pos(j).is_ok_and(|p| {
                        ((f64::from(p) - f64::from(target[j])) * self.per_tick(j)).abs()
                            <= tolerance
                    })
                })
            {
                self.hold = target;
                return Ok(());
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

    /// Every joint's holding current at the pose it is already at.
    ///
    /// Encoder-only replies do not refresh current, so each value must come
    /// from this drain rather than a cached earlier reply, and the arm must
    /// not drift while measuring: a joint that wanders is holding something
    /// other than what the pose says.
    fn holding_current(&mut self) -> Result<[f64; N]> {
        let tolerance = self.tolerance();
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
        Ok(fit)
    }

    // ------------------------------------------------------------ parking

    /// Return every joint somewhere safe, then release.
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
        let mut parked = Ok(());
        // The joints parked on their endstops last: they hold the arm up.
        let rests = |j: usize| self.bundle.robot.parks_on_endstop(j);
        let order: Vec<usize> = (0..N)
            .filter(|&j| !rests(j))
            .chain((0..N).filter(|&j| rests(j)))
            .collect();
        for j in order {
            self.only = Some(j);
            let outcome = self.park_one(j);
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
            // vendor's pre-homing wrist nudge is a relative move.
            let distance = (i64::from(self.found_at[j]) - i64::from(self.pos(j)?)).abs() as f64;
            if distance <= 1.0 {
                return Ok(());
            }
            let per_s =
                (self.bundle.robot.shutdown.velocity_limit_rad_s / self.per_tick(j).abs()).max(1.0);
            self.emit(Event::Phase("return to where the run found it", j));
            self.operating(j)?;
            let seconds = (SEPTIC_PEAK_VEL * distance / per_s).max(0.3);
            let m = self.run_motion(j, self.found_at[j], seconds, false)?;
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

/// Replace a top-level array in place, or add the whole line when the file
/// does not carry it yet.
fn patch_array(text: &mut String, key: &str, values: &[f64]) -> Result<()> {
    // Full precision, not a fixed decimal place: these run from tens of
    // milli-kg-m down to the solver's own noise floor, and rounding would
    // quietly zero the small ones.
    let rendered = values
        .iter()
        .map(|v| format!("{v:?}"))
        .collect::<Vec<_>>()
        .join(", ");
    let Some(start) = text.find(&format!("{key} =")) else {
        text.insert_str(0, &format!("{key} = [{rendered}]  # selfcal: measured\n"));
        return Ok(());
    };
    let open = start
        + text[start..]
            .find('[')
            .ok_or_else(|| format!("{key} is not an array"))?;
    let close = open
        + text[open..]
            .find(']')
            .ok_or_else(|| format!("{key} is not closed"))?;
    text.replace_range(open..=close, &format!("[{rendered}]"));
    Ok(())
}

/// Patch the measured values into the file as written, so its comments and
/// layout survive; a full re-serialisation would discard them.
fn patch_config(
    original: &str,
    correction: Option<&[f64]>,
    friction: Option<&[Option<(f64, f64)>; N]>,
    sim: &par6_config::SimConfig,
    tuned: Option<&[Option<Tuned>; N]>,
    limits: Option<&[Option<Found>; N]>,
) -> Result<String> {
    let mut text = original.to_owned();
    if let Some(correction) = correction {
        patch_array(&mut text, "gravity_correction", correction)?;
        // Identification measures the true torque, so any per-joint trim from
        // an older calibration is superseded and would otherwise multiply it.
        patch_array(&mut text, "gravity_scale", &[1.0; N])?;
    }
    if let Some(friction) = friction {
        // The friction the simulator's joints show their drives: measured
        // joint by joint, a joint the run skipped keeps the file's value.
        let measured = |pick: fn(&(f64, f64)) -> f64, current: &[f64]| -> Vec<f64> {
            friction
                .iter()
                .zip(current)
                .map(|(m, c)| m.as_ref().map_or(*c, pick))
                .collect()
        };
        let viscous = measured(|m| m.0, &sim.viscous_nm_s);
        let coulomb = measured(|m| m.1, &sim.coulomb_nm);
        patch_array(&mut text, "viscous_nm_s", &viscous)?;
        patch_array(&mut text, "coulomb_nm", &coulomb)?;
    }
    for (j, t) in tuned.into_iter().flatten().enumerate() {
        // Only what the search moved: a joint it left alone keeps its lines
        // byte for byte, comments and all.
        if let Some(t) = t {
            let mut values = Vec::new();
            if t.after.kpv != t.before.kpv {
                values.push(("kpv", t.after.kpv));
            }
            if t.after.kiv != t.before.kiv {
                values.push(("kiv", t.after.kiv));
            }
            if !values.is_empty() {
                patch_joint_table(&mut text, j, "[joints.gains]", &values)?;
            }
        }
    }
    for (j, found) in limits.into_iter().flatten().enumerate() {
        if let Some(found) = found {
            patch_exec_limits(&mut text, j, found)?;
        }
    }
    Ok(text)
}

/// Write what the limits stage moved in joint `j`'s `[joints.limits.exec]`
/// table: a cap the search left where the file had it keeps its line byte
/// for byte, and a jerk it only bounded keeps its configured value.
fn patch_exec_limits(text: &mut String, j: usize, found: &Found) -> Result<()> {
    let caps = &found.caps;
    let was = &found.configured;
    let mut values = Vec::new();
    if caps.velocity != was.velocity {
        values.push(("velocity_rad_s", caps.velocity));
    }
    if caps.acceleration != was.acceleration {
        values.push(("acceleration_rad_s2", caps.acceleration));
    }
    if found.jerk_measured && caps.jerk != was.jerk {
        values.push(("jerk_rad_s3", caps.jerk));
    }
    if values.is_empty() {
        return Ok(());
    }
    patch_joint_table(text, j, "[joints.limits.exec]", &values)
}

/// Set `values` inside joint `j`'s `header` table, keeping everything else
/// in the file byte for byte.
fn patch_joint_table(
    text: &mut String,
    j: usize,
    header: &str,
    values: &[(&str, f64)],
) -> Result<()> {
    let name = text
        .find(&format!("name = \"joint{}\"", j + 1))
        .ok_or_else(|| format!("configuration has no joint{}", j + 1))?;
    let next_joint = text[name..]
        .find("[[joints]]")
        .map_or(text.len(), |i| name + i);
    let table = text[name..next_joint]
        .find(header)
        .map(|i| name + i)
        .ok_or_else(|| format!("joint{} has no {header} table", j + 1))?;
    let body = table + header.len();
    let end = body
        + text[body..]
            .find("\n[")
            .map_or(text.len() - body, |i| i + 1);
    let mut block = text[body..end].to_owned();
    for (key, value) in values {
        block = set_value(&block, key, &format!("{value:.5}"));
    }
    text.replace_range(body..end, &block);
    Ok(())
}

/// Set one `key = value` inside a table body, keeping the rest byte for byte
/// and a trailing comment on the line; appended when the table lacks it.
fn set_value(block: &str, key: &str, value: &str) -> String {
    let mut out = String::with_capacity(block.len() + key.len() + value.len() + 4);
    let mut replaced = false;
    for line in block.split_inclusive('\n') {
        let trimmed = line.trim_start();
        let is_key = trimmed
            .strip_prefix(key)
            .is_some_and(|rest| rest.trim_start().starts_with('='));
        if !replaced && is_key {
            let indent = &line[..line.len() - trimmed.len()];
            let comment = line.find('#').map_or("", |i| line[i..].trim_end());
            out.push_str(indent);
            out.push_str(key);
            out.push_str(" = ");
            out.push_str(value);
            if !comment.is_empty() {
                out.push(' ');
                out.push_str(comment);
            }
            out.push('\n');
            replaced = true;
        } else {
            out.push_str(line);
        }
    }
    if !replaced {
        if !out.ends_with('\n') {
            out.push('\n');
        }
        out.push_str(&format!("{key} = {value}\n"));
    }
    out
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

fn run(args: Args) -> Result<()> {
    let bundle = match &args.tool {
        Some(tool) => ConfigBundle::load_fitted(&args.config, tool)?,
        None => ConfigBundle::load(&args.config)?,
    };
    bundle.robot.validate()?;
    // The file's own friction, for joints a partial run leaves unmeasured.
    let sim = bundle.robot.sim.clone();
    // Resolved the way the daemon resolves it: a lexical step up from the
    // config directory, never `config/..` through the filesystem, which
    // follows the `config` symlink into the package and lands beside it.
    let assets = par6d::kin::resolve_assets_dir(None, &args.config)?;
    let ready = planned_ready(&bundle)?;
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

    let original = fs::read_to_string(&args.config)?;
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
    let limits_stage = args.limits || args.limits_only;
    let gains_stage = args.gains || args.gains_only;
    let only = args.limits_only || args.gains_only;
    let spans = if limits_stage || gains_stage {
        Some(limit_spans(&bundle, &assets, ready)?)
    } else {
        None
    };
    let mut arm = Arm::open(bundle, &assets, args.sim, tx)?;
    if !args.sim {
        runtime::realtime(timing.cpu, timing.fifo_priority)?;
    }

    let mut fit = None;
    let mut friction = None;
    let mut tuned = None;
    let mut limits = None;
    let outcome = arm.initialize().and_then(|()| {
        arm.home()?;
        if !only {
            friction = Some(arm.measure_mechanics()?);
            fit = Some(arm.identify(&poses, ready)?);
        }
        if let (true, Some(spans)) = (gains_stage, &spans) {
            tuned = Some(arm.gains(ready, spans)?);
        }
        if let (true, Some(spans)) = (limits_stage, &spans) {
            limits = Some(arm.limits(&poses, ready, spans)?);
        }
        Ok(())
    });
    let parked = arm.shutdown();
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
    let result = outcome.and(parked).and(recorded.map_err(Into::into));
    fs::write(directory.join("result.txt"), format!("{result:?}\n"))?;
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
    if let Some(tuned) = &tuned {
        for (j, t) in tuned.iter().enumerate() {
            if let Some(t) = t {
                println!(
                    "gains J{}: kpv {:.5} -> {:.5}, kiv {:.5} -> {:.5}, off the profile {:.2} -> {:.2} deg/s",
                    j + 1,
                    t.before.kpv,
                    t.after.kpv,
                    t.before.kiv,
                    t.after.kiv,
                    t.score_before.to_degrees(),
                    t.score_after.to_degrees()
                );
            }
        }
    }
    let patched = patch_config(
        &original,
        correction.as_deref(),
        friction.as_ref(),
        &sim,
        tuned.as_ref(),
        limits.as_ref(),
    )?;
    // Refuse to write something that will not load.
    par6_config::RobotConfig::from_toml_str(&patched)?.validate()?;
    fs::write(directory.join("calibrated.toml"), &patched)?;
    if args.apply && args.sim {
        return Err("--apply writes masses fitted to the simulator; refusing".into());
    }
    if args.apply {
        let backup = args.config.with_extension("toml.before-selfcal");
        if !backup.exists() {
            fs::write(backup, &original)?;
        }
        let temp = args.config.with_extension("toml.selfcal-tmp");
        fs::write(&temp, &patched)?;
        fs::rename(temp, &args.config)?;
        println!("applied to {}", args.config.display());
    } else {
        println!("results: {}", directory.join("calibrated.toml").display());
    }
    Ok(())
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
