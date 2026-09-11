//! Immediate command effects (`RtCommands`) bridged onto the RT core,
//! plus the housekeeping loop that owns the timed follow-throughs.
//!
//! Two paths into the core:
//!
//! - `RtCommand` mpsc: the tick loop consumes AT MOST ONE per tick, so
//!   multi-step effects (mode dances, e-stop clear) are ordered queues,
//!   never synchronous calls.
//! - [`CoreOp`] closures: applied with `&mut RtCore` between `run()`
//!   sessions on the RT thread (the RT loop breaks out of `run()` when
//!   an op is queued). Used for teleport re-seeding, settle-policy
//!   swaps, and loop-stats reset — things the command vocabulary does
//!   not carry.
//!
//! The protocol is modeless, but the RT core is not: streamables drive
//! the JOG/STREAM mode transitions here, and the housekeeping thread
//! self-terminates them (jog duration watchdog, servo silence timeout)
//! so the RT watchdog never latches a link-lost error on a client that
//! simply stopped streaming.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::time::{Duration, Instant};

use par6_bus::sim::scene::Scene;
use par6_bus::sim::SimBus;
use par6_bus::sim::WorldMailbox;
use par6_bus::{RuntimeBus, SocketCanBus};
use par6_config::ConfigBundle;
use par6_proto::command::MAX_JOG_DURATION_S;
use par6_proto::{make_error, Command, ErrorCode, WireError, NUM_JOINTS, UNATTRIBUTED};
use par6_rt::{
    ArmState, FlushMarker, Mode, RtCommand, RtCore, SnapshotReader, StateSnapshot, StreamInput,
    StreamSetpoint, MAX_JOINTS,
};
use par6_server::RtCommands;
use par6_server::{CollisionState, ShapeLayer};

use crate::collision_world::{is_world_name, kin_layer, ShapeNames};

/// A closure applied to the core on the RT thread, between `run()`
/// sessions.
pub(crate) type CoreOp = Box<dyn FnOnce(&mut RtCore<RuntimeBus>) + Send>;

/// How long the enable retry keeps trying after `reset` (covers the RT
/// clear-sequence settle window with margin, even on a loaded host).
const ENABLE_RETRY_WINDOW: Duration = Duration::from_secs(5);
/// Housekeeping loop period: one RT tick.
///
/// Housekeeping is the soft-real-time plane — it has to keep up with the
/// tick on average, and a miss costs one late cart-jog ramp step. The
/// period is also the dt it integrates the cartesian jog twist over, so
/// pinning it to anything but the tick would integrate the twist on a
/// different clock than the loop consuming the result, and publish
/// setpoints the RT's latest-wins slot then throws away.
pub(crate) fn housekeeping_period(dt: f64) -> Duration {
    Duration::from_secs_f64(dt)
}

/// Spacing between enable retries, in RT ticks.
///
/// The budget it must not saturate is the RT's one-command-per-tick
/// drain, so the spacing is counted in ticks and converted at the tick
/// rate in force — as a fixed 60 ms it was "a few ticks" only at 250 Hz
/// and 1.2 ticks at the rate the python rig runs.
const ENABLE_RETRY_TICKS: u32 = 15;
/// How long a FLASHING enter/exit waits for the published mode to
/// answer. The RT decides on the next tick, so this only has to cover
/// command-queue and snapshot latency on a loaded host.
const FLASHING_WINDOW: Duration = Duration::from_secs(2);

/// Watchdog deadline for a jog carrying `duration_s` seconds.
///
/// The codec already bounds `duration` to [`MAX_JOG_DURATION_S`], but
/// this is the arithmetic that takes the process down if that bound is
/// ever wrong — `Duration::from_secs_f64` panics above ~1.8e19 s, and it
/// runs here with the shared-state lock held, in a build compiled
/// `panic = "abort"`. So it clamps rather than trusts: nothing reachable
/// from the wire may abort the daemon.
fn jog_deadline(duration_s: f64) -> Instant {
    let bounded = duration_s.clamp(0.0, MAX_JOG_DURATION_S);
    // `clamp` propagates NaN, and the deadline is what STOPS the jog, so
    // an unusable duration expires it at once instead of arming an
    // undefined watchdog.
    let bounded = if bounded.is_nan() { 0.0 } else { bounded };
    Instant::now() + Duration::from_secs_f64(bounded)
}

/// Ticks of PIPELINE the streaming stopping projection allows for,
/// before the arm's own settling is added.
///
/// This is the part of the reaction that is bookkeeping rather than
/// physics: a snapshot at most one tick old, a housekeeping pass every
/// tick, and an RT that drains one command per tick. Counted in ticks
/// because that is what those stages are made of — a flat wall-clock
/// horizon was a different number of pipeline stages at every tick rate.
const STOP_PIPELINE_TICKS: f64 = 24.0;

/// First-order lags the settling term counts.
///
/// Two, not one: the DRIVE closing its position error is one lag, and
/// the streaming executor re-planning every setpoint to be reached at
/// rest is a second one of comparable size in front of it. Measured on
/// the sim rig, a single lag under-projects a 1 mm/50 ms approach by
/// about half its coast.
const STOP_SETTLE_LAGS: f64 = 2.0;

/// Signed joint travel between "moving at `v_rad_s`" and "stopped"
/// \[rad\]: [`STOP_PIPELINE_TICKS`] of `tick_dt_s` before anything
/// happens, then the arm settling out of its own tracking error.
///
/// The settling term is `v / position_loop_hz`, not `v^2 / 2a`. In this
/// regime the arm is not deceleration-limited: the streaming executor's
/// command drops inside a handful of ticks and what takes the time is
/// the DRIVE closing the position error it was already carrying, which
/// is a first-order lag with time constant `1 / kpp`. Pricing the coast
/// at the configured acceleration ceiling instead was optimistic by
/// more than an order of magnitude — measured on the sim rig at ~0.7
/// rad/s^2 achieved against a 32 rad/s^2 stream limit — and the arm
/// coasted through the whole standoff every time.
///
/// This is what the streaming collision gate projects, so a caller
/// choosing a jog speed that must be refused (or admitted) at a known
/// distance from an obstacle inverts this function rather than guessing.
///
/// A non-finite or non-positive gain contributes no settling term,
/// rather than an infinite one that would refuse every motion.
pub fn stream_stopping_travel(v_rad_s: f64, position_loop_hz: f64, tick_dt_s: f64) -> f64 {
    let pipeline = v_rad_s * STOP_PIPELINE_TICKS * tick_dt_s;
    if !position_loop_hz.is_finite() || position_loop_hz <= 0.0 {
        return pipeline;
    }
    pipeline + STOP_SETTLE_LAGS * v_rad_s / position_loop_hz
}

/// Joint speed above which a stream counts as MOVING and is re-tested
/// against its projection every period \[rad/s\]. Below it the arm is
/// settling on a held target and the projection would be noise.
const STREAM_MOVING_RAD_S: f64 = 0.01;

/// How near a joint has to be to a gate-imposed standoff to count as
/// arrived \[rad\]. Half a millimetre at the arm's reach, an order below
/// the standoff itself, so "arrived" is a statement about the geometry
/// rather than about the executor's last increment.
const STANDOFF_ARRIVED_RAD: f64 = 2.0e-3;

/// How far short of the solved boundary the placement is commanded
/// \[rad\], on the fastest joint.
///
/// `stop_point` returns the configuration exactly ON the clearance, and
/// an arm commanded exactly there settles either side of it — half the
/// settle distribution lands inside the standoff, which is the one place
/// it must not. Commanding the boundary less this margin puts the whole
/// distribution outside. Half a millimetre at the arm's reach: enough to
/// cover the settle, an order below the standoff it is protecting.
const STANDOFF_SETTLE_MARGIN_RAD: f64 = 1.0e-3;

/// How far ahead of the MEASURED pose a placement setpoint may sit
/// \[rad\].
///
/// The placement ends ON a keep-out's clearance, so it is the one move
/// that must not overshoot — and a position command the plant is
/// chasing is exactly a move that does: measured on the sim rig, a 5.3
/// degree placement commanded outright ran 5.7 mm past the standoff
/// before settling back onto it, straight through the keep-out. Walking
/// the setpoint on from the measurement means the command never leads
/// the arm by more than this, so there is no stored tracking error to
/// carry it past.
const STANDOFF_CREEP_RAD: f64 = 4.0e-3;

/// Speed and acceleration fractions the placement move onto a standoff
/// runs at.
///
/// The placement is a short, precise move onto a surface the arm was
/// just refused at, made from rest — there is nothing to gain by taking
/// it at streaming speed.
const STANDOFF_PLACEMENT_SCALE: (f64, f64) = (0.05, 0.05);

/// Consecutive FRESH snapshots below [`STREAM_MOVING_RAD_S`] that count
/// as the arm having stopped.
///
/// One frame is not rest — a snapshot's velocity passes through zero
/// whenever the executor re-plans — and housekeeping runs faster than
/// the RT publishes, so the count only advances on a new tick.
const STANDOFF_STILL_TICKS: u8 = 8;

/// How long the arm is given to reach a standoff before the stream is
/// ended anyway.
///
/// The refeed exists because the client has stopped sending — its
/// motion was refused — so nothing else would keep the stream alive
/// across the travel. That makes it the one place a stuck executor
/// could hold a stream open forever, and this is the backstop.
const STANDOFF_TRAVEL_BUDGET: Duration = Duration::from_secs(3);
/// Escape-depth tolerance \[m\]: a min-distance drop smaller than this
/// counts as "no deeper" (absorbs signed-distance jitter between two
/// nearby configurations; parol6's escape tolerance). Used by the
/// planner's per-sample check against the START depth, matching
/// parol6's `guard_joint_path`.
pub(crate) const ESCAPE_TOL_M: f64 = 1e-4;
/// The streaming gate's escape-depth tolerance \[m\]. parol6 applies
/// [`ESCAPE_TOL_M`] per integrator step (~10 ms of travel); par6's gate
/// compares across the whole stopping projection in one
/// step, so the same per-step slack scales with the horizon — otherwise
/// an escaping arc whose link transiently dips ~1 mm deeper inside the
/// window is refused, and the arm is trapped in the keep-out the rule
/// exists to let it leave. Sustained grinding is still caught: the
/// housekeeping re-check advances `current` every period, so a deepening
/// beyond this slack cannot accumulate unrefused.
const STREAM_ESCAPE_TOL_M: f64 = 1.5e-3;

/// Joint-space pitch of the coarse pass in [`StreamGate::stop_point`],
/// matching the planner's own sweep pitch: at PAR6's reach a 0.02 rad
/// step moves the wrist under 10 mm, so no keep-out thick enough to
/// matter can be stepped over.
const GATE_STEP_RAD: f64 = 0.02;

/// Joint-space pitch of the gate's SEGMENT check, coarser than
/// [`GATE_STEP_RAD`] because it runs on every admitted datagram rather
/// than once per refusal: 0.05 rad moves the wrist about 25 mm at PAR6's
/// reach, still well under any keep-out worth declaring, and it keeps a
/// stopping projection at streaming speed to single-digit collision
/// queries instead of tens.
const GATE_SEGMENT_STEP_RAD: f64 = 0.05;

/// Samples one segment check will take before coarsening further. The
/// bound is what keeps the command plane's admission cost flat when a
/// client streams targets across the whole soft window.
const GATE_SEGMENT_MAX_STEPS: usize = 32;

/// Interior samples every gate sweep takes even when the pitch would
/// ask for fewer. A pitch bounds the joint travel one sample can hide,
/// not the obstacle it can step over, and the shortest sweeps are the
/// ones that graze a keep-out rather than drive through it.
const GATE_MIN_SAMPLES: usize = 8;

/// Coarse samples [`StreamGate::stop_point`] will take before giving up.
/// The bound matters because the span comes off the wire: a target at
/// the far end of the soft window must not turn into an unbounded sweep.
const GATE_MAX_STEPS: usize = 512;

/// Bisection rounds refining the coarse pass. The pitch above resolves
/// the stopping point to ~10 mm, which is twice the standoff it is
/// supposed to land on; 16 halvings take that under 1 um, and each
/// round is one collision check.
const GATE_BISECT_ROUNDS: u32 = 16;

/// A firmware "go to position" gripper frame from wire units: `closed`
/// and `speed` are fractions in \[0, 1\] (0 = fully open / slowest byte,
/// 1 = fully closed / fastest), `current_ma` the press-force limit.
pub(crate) fn gripper_move_command(
    closed: f64,
    speed: f64,
    current_ma: f64,
) -> par6_bus::FirmwareGripperCommand {
    let byte = |v: f64| (v.clamp(0.0, 1.0) * 255.0).round() as u8;
    par6_bus::FirmwareGripperCommand {
        position: byte(closed),
        speed: byte(speed),
        current_ma: current_ma.clamp(0.0, f64::from(i16::MAX)).round() as i16,
        activate: true,
        action: true,
        estop: false,
        release_dir: false,
    }
}

/// The streaming collision gate: the collision world as enforced against
/// `jog_j` / `jog_l` / `servo_*` setpoints.
///
/// The RT thread integrates the jog/servo ramp, and a coal check cannot
/// run there — so the gate runs on the two places that CAN see the
/// stream: the bridge (admission, one check per accepted datagram) and
/// housekeeping (a re-check every period while a stream is live, which
/// is the moving-jog analogue of parol6 re-checking every controller
/// tick). It holds its OWN model instance (pinocchio's `GeometryData` is
/// mutated by every query), mirrored layer-for-layer with the planner's
/// world by [`RtCommands::set_shapes`].
///
/// The verdict rule is parol6's `collision_blocked`: approaching, a
/// motion is blocked when its lookahead configuration collides; already
/// colliding (a keep-out placed over the arm), it is blocked when it
/// contacts anything NEW or goes DEEPER (`min_distance` comparison) —
/// escaping stays allowed, because streaming is the only way OUT of a
/// keep-out the arm is already inside. Self pairs the arm may rest in
/// are excluded model-side by the variant's SRDF.
pub(crate) struct StreamGate {
    collision: par6_kin::Collision,
    /// Reporting names for the applied keep-out shapes.
    shape_names: ShapeNames,
    /// Per-joint JOG-mode velocity limits \[rad/s\] — what a `speeds`
    /// fraction of ±1 commands, and therefore what the lookahead
    /// projects with.
    jog_vel: [f64; MAX_JOINTS],
    /// Per-joint drive position-loop gains \[1/s\] (the config `kpp`
    /// pushed to each driver) — the reciprocal of the time constant the
    /// arm takes to settle out of its tracking error, and so most of how
    /// far it travels stopping.
    position_loop_hz: [f64; MAX_JOINTS],
    /// RT period \[s\]; the reaction half of the stopping distance is
    /// counted in ticks of it.
    tick_dt_s: f64,
    soft_min: [f64; MAX_JOINTS],
    soft_max: [f64; MAX_JOINTS],
    /// The pairs the last refused or stopped stream would have collided
    /// in — the streaming half of the STATUS `collision_active` fields.
    latch: CollisionState,
}

impl StreamGate {
    /// The gate's own collision world, for planning poses against the
    /// same shapes the gate admits jogs against.
    pub(crate) fn collision_mut(&mut self) -> &mut par6_kin::Collision {
        &mut self.collision
    }

    pub(crate) fn new(
        collision: par6_kin::Collision,
        jog_limits: &par6_motion::MotionLimits,
        position_loop_hz: [f64; MAX_JOINTS],
        tick_dt_s: f64,
    ) -> Self {
        Self {
            collision,
            shape_names: ShapeNames::default(),
            jog_vel: jog_limits.velocity,
            position_loop_hz,
            tick_dt_s,
            soft_min: jog_limits.soft_min,
            soft_max: jog_limits.soft_max,
            latch: CollisionState::default(),
        }
    }

    /// Mirror one layer of the planner-accepted world. The conversion is
    /// the identical `Shape::from_proto` path the planner ran, so on a
    /// set the server hands over it cannot disagree.
    pub(crate) fn set_layer(
        &mut self,
        layer: ShapeLayer,
        shapes: &[par6_proto::Shape],
    ) -> Result<(), WireError> {
        let converted = shapes
            .iter()
            .map(par6_kin::Shape::from_proto)
            .collect::<Result<Vec<_>, _>>()
            .map_err(|e| {
                make_error(
                    ErrorCode::CommValidationError,
                    UNATTRIBUTED,
                    &[("detail", &e.to_string())],
                )
            })?;
        self.collision
            .set_layer(kin_layer(layer), &converted)
            .map_err(|e| {
                make_error(
                    ErrorCode::CommValidationError,
                    UNATTRIBUTED,
                    &[("detail", &format!("stream gate collision world: {e}"))],
                )
            })?;
        self.shape_names.set_layer(layer, &converted);
        Ok(())
    }

    /// Interior samples for a segment check between `from` and `to`:
    /// [`GATE_SEGMENT_STEP_RAD`] apart on the fastest joint, bounded so a
    /// target off the far end of the soft window cannot turn into an
    /// unbounded sweep.
    fn segment_steps(&self, from: &[f64; MAX_JOINTS], to: &[f64; MAX_JOINTS]) -> usize {
        let span = from[..par6_kin::NQ]
            .iter()
            .zip(to[..par6_kin::NQ].iter())
            .map(|(a, b)| (b - a).abs())
            .fold(0.0, f64::max);
        if !span.is_finite() {
            return GATE_SEGMENT_MAX_STEPS;
        }
        ((span / GATE_SEGMENT_STEP_RAD).ceil() as usize)
            .clamp(GATE_MIN_SAMPLES, GATE_SEGMENT_MAX_STEPS)
    }

    /// The colliding pairs at the first sample along `from -> to` that
    /// collides, or `None` when the whole segment clears.
    fn hit_along(
        &mut self,
        from: &[f64; MAX_JOINTS],
        to: &[f64; MAX_JOINTS],
        steps: usize,
    ) -> Result<Option<Vec<(String, String)>>, WireError> {
        let mut a = [0.0; par6_kin::NQ];
        let mut b = [0.0; par6_kin::NQ];
        a.copy_from_slice(&from[..par6_kin::NQ]);
        b.copy_from_slice(&to[..par6_kin::NQ]);
        let Some(i) = self
            .collision
            .check_segment(&a, &b, steps)
            .map_err(gate_error)?
        else {
            return Ok(None);
        };
        let mut hit = *from;
        let t = i as f64 / (steps + 1) as f64;
        for j in 0..par6_kin::NQ {
            hit[j] = from[j] + t * (to[j] - from[j]);
        }
        Ok(Some(self.offending(&hit)?))
    }

    /// Whether `q` itself collides, clearance included — the point half
    /// of [`blocked`]'s verdict, without the pair names or the segment
    /// walk. What a sweep that has already established its start is
    /// clear needs per sample.
    ///
    /// [`blocked`]: StreamGate::blocked
    fn collides(&mut self, q: &[f64; MAX_JOINTS]) -> Result<bool, WireError> {
        let mut nq = [0.0; par6_kin::NQ];
        nq.copy_from_slice(&q[..par6_kin::NQ]);
        Ok(self
            .collision
            .check(&nq, true)
            .map_err(gate_error)?
            .active())
    }

    /// Colliding pairs at `q`, in reporting names.
    fn offending(&mut self, q: &[f64; MAX_JOINTS]) -> Result<Vec<(String, String)>, WireError> {
        let mut nq = [0.0; par6_kin::NQ];
        nq.copy_from_slice(&q[..par6_kin::NQ]);
        let names = &self.shape_names;
        let report = self.collision.check(&nq, false).map_err(gate_error)?;
        Ok(names.render(&report))
    }

    /// The offending pairs at `q` that involve a world shape.
    ///
    /// Self pairs are excluded because a standing self contact is not a
    /// thing the arm can back out of along a commanded path — the SRDF
    /// leaves the park pose resting in some — while a world contact is
    /// exactly what the standoff exists to hold.
    fn world_offenders(
        &mut self,
        q: &[f64; MAX_JOINTS],
    ) -> Result<Vec<(String, String)>, WireError> {
        Ok(self
            .offending(q)?
            .into_iter()
            .filter(|p| is_world_name(&p.0) || is_world_name(&p.1))
            .collect())
    }

    /// Where a refused motion should come to rest: the configuration on
    /// the `current -> target` line that sits ON the standoff.
    ///
    /// Refusing a motion says only that the arm must not finish where it
    /// was asked to; it does not say where it should finish. Left at
    /// that, the resting place is whatever the executor's tracking lag
    /// and the datagram timing happened to leave — measured on the sim
    /// rig at anywhere from 2.7 mm to 24 mm from a keep-out whose
    /// standoff is 5 mm, varying run to run at one speed. Commanding
    /// this point makes the stop a property of the geometry instead: the
    /// arm lands ON the clearance, whatever speed it approached at.
    ///
    /// The model's clearance is already inside the collision verdict, so
    /// the boundary between world-clear and world-touching IS the
    /// standoff — nothing here adds a margin of its own.
    ///
    /// Two directions, because a refusal can arrive either side of the
    /// boundary:
    ///
    /// - Clear of the world: walk FORWARD and stop at the last admitted
    ///   sample. [`blocked`] is the predicate, not a bare collision
    ///   test, so a tolerated standing self contact does not read as
    ///   "nowhere to go".
    /// - Already touching the world: walk BACKWARD down the same line
    ///   until the arm is off the shape, and stop there. This is the
    ///   case the escape-depth half of [`blocked`] cannot help with —
    ///   once a mesh-vs-box pair is in contact its reported depth is
    ///   nearly flat, so `blocked` admits deeper motion and the
    ///   forward walk would hand back a configuration inside the
    ///   standoff.
    ///
    /// Each direction takes a coarse pass at [`GATE_STEP_RAD`] and then
    /// bisects, so a non-convex clear region cannot be skipped over.
    ///
    /// [`blocked`]: StreamGate::blocked
    pub(crate) fn stop_point(
        &mut self,
        current: &[f64; MAX_JOINTS],
        target: &[f64; MAX_JOINTS],
    ) -> Result<[f64; MAX_JOINTS], WireError> {
        let mut dir = [0.0; MAX_JOINTS];
        for j in 0..par6_kin::NQ {
            dir[j] = target[j] - current[j];
        }
        let span = dir[..par6_kin::NQ]
            .iter()
            .map(|v| v.abs())
            .fold(0.0, f64::max);
        if !span.is_finite() || span == 0.0 {
            return Ok(*current);
        }
        let at = |t: f64| {
            let mut q = *current;
            for j in 0..par6_kin::NQ {
                q[j] = current[j] + t * dir[j];
            }
            q
        };
        // One coarse step is GATE_STEP_RAD on the fastest joint.
        let dt = GATE_STEP_RAD / span;

        if !self.world_offenders(current)?.is_empty() {
            // BOTH directions along the line, nearest escape first. The
            // refused target is not reliably the way back out: a refusal
            // is answered by braking, and the arm can coast past the
            // target it was refused at, which leaves `goal` BEHIND it and
            // "away from goal" pointing straight through the keep-out.
            // Which side the arm entered from is not recoverable at rest,
            // so the shortest way out is the honest answer.
            let mut escaped = None;
            'coarse: for i in 1..=GATE_MAX_STEPS {
                let step = i as f64 * dt;
                for t in [-step, step] {
                    if self.world_offenders(&at(t))?.is_empty() {
                        escaped = Some(t);
                        break 'coarse;
                    }
                }
            }
            // Nothing on this line clears the world. Hold rather than
            // drive the arm somewhere nobody asked for.
            let Some(mut clear) = escaped else {
                return Ok(*current);
            };
            // Bisect against `current` itself: it is the one sample known
            // to be inside on the same side as the escape.
            let mut inside = 0.0;
            for _ in 0..GATE_BISECT_ROUNDS {
                let mid = 0.5 * (clear + inside);
                if self.world_offenders(&at(mid))?.is_empty() {
                    clear = mid;
                } else {
                    inside = mid;
                }
            }
            return Ok(at(clear));
        }

        // Per-sample point test, not `blocked`: the start is known clear
        // here, so any collision at a sample is a new one, and walking
        // the segment once is what keeps this linear — re-asking
        // `blocked`, which walks the whole segment itself, at every
        // sample squares the cost and stalls the command plane.
        // At least GATE_MIN_SAMPLES whatever the span. The pitch alone is
        // a bound on how much JOINT travel a sample can hide, not on how
        // much of an obstacle it can step over: a short projection that
        // only grazes a keep-out gets two or three samples at that pitch
        // and can miss it entirely — while the segment check that refused
        // the motion, walking the same line, did not.
        let steps =
            ((span / GATE_STEP_RAD).ceil() as usize).clamp(GATE_MIN_SAMPLES, GATE_MAX_STEPS);
        let mut clear = 0.0;
        let mut hit = None;
        for i in 1..=steps {
            let t = i as f64 / steps as f64;
            if self.collides(&at(t))? {
                hit = Some(t);
                break;
            }
            clear = t;
        }
        let Some(mut refused) = hit else {
            return Ok(*target);
        };
        for _ in 0..GATE_BISECT_ROUNDS {
            let mid = 0.5 * (clear + refused);
            if self.collides(&at(mid))? {
                refused = mid;
            } else {
                clear = mid;
            }
        }
        let stop = at(clear);
        Ok(stop)
    }

    fn world_distance(&mut self, q: &[f64; MAX_JOINTS]) -> Result<f64, WireError> {
        let mut nq = [0.0; par6_kin::NQ];
        nq.copy_from_slice(&q[..par6_kin::NQ]);
        self.collision.world_distance(&nq).map_err(gate_error)
    }

    /// Whether streaming from `current` toward `target` must stop, and
    /// the pairs to report if so.
    ///
    /// parol6's `collision_blocked` rule. Approaching: blocked when the
    /// target configuration collides. Already colliding: blocked when
    /// the target contacts a pair the arm is not already in, or when the
    /// deepest world penetration grows (the hull-vs-world
    /// `world_distance` drops by more than [`ESCAPE_TOL_M`]) — a
    /// pair-set check alone cannot tell an escaping move from one
    /// grinding deeper through the same pair, and the depth check alone
    /// cannot tell an improving start-collision from a new shallower
    /// one, so both run.
    pub(crate) fn blocked(
        &mut self,
        current: &[f64; MAX_JOINTS],
        target: &[f64; MAX_JOINTS],
    ) -> Result<Option<Vec<(String, String)>>, WireError> {
        let cur = self.offending(current)?;
        let tgt = self.offending(target)?;
        if cur.is_empty() {
            if !tgt.is_empty() {
                return Ok(Some(tgt));
            }
            // The SEGMENT, not just its end. A stopping projection is a
            // long reach at speed, and a keep-out is a solid: an
            // endpoint test lets the projection tunnel clean through one
            // and report clear, which is why raising the horizon stopped
            // making the arm stop any earlier. The pairs reported are
            // the ones at the first colliding sample.
            let steps = self.segment_steps(current, target);
            if let Some(hit) = self.hit_along(current, target, steps)? {
                return Ok(Some(hit));
            }
            return Ok(None);
        }
        let new: Vec<(String, String)> = tgt.iter().filter(|p| !cur.contains(p)).cloned().collect();
        if !new.is_empty() {
            return Ok(Some(new));
        }
        // The depth half runs only when the standing collision involves a
        // WORLD shape — the keep-out case escape exists for, and the
        // only case the signal speaks about: `world_distance` covers
        // world pairs only, so an arm-arm contact has no depth here and
        // remains guarded by the pair half above.
        let world_pair = cur
            .iter()
            .any(|p| is_world_name(&p.0) || is_world_name(&p.1));
        if world_pair {
            // Still touching a keep-out at the target, so the move has to
            // EARN its way out: measurably farther, not merely "no
            // deeper". The weaker rule cannot be enforced with the signal
            // available — coal's penetration depth for a mesh-vs-box pair
            // is a local contact-patch estimate, nearly flat in the true
            // depth (~5 mm of reported deepening across 40 mm of
            // face-to-centre travel on the sim rig), so "deeper" is
            // indistinguishable from "lateral" and a stream that once
            // reached contact was admitted straight through the box.
            //
            // Nothing is trapped by this: a refusal is answered by
            // placing the arm ON the standoff (see `stop_point`), which
            // leaves it clear of the world and free to move again.
            // `world_distance >= clearance` IS "no world shape offends":
            // that is the threshold `check` flags a world pair at. Read
            // off the distance already needed rather than running a
            // second full check for the pair list — this whole branch
            // runs on the command plane, per datagram.
            let reach = self.collision.clearance();
            let (d_target, d_current) =
                (self.world_distance(target)?, self.world_distance(current)?);
            if d_target < reach && d_target <= d_current + STREAM_ESCAPE_TOL_M {
                return Ok(Some(if tgt.is_empty() { cur } else { tgt }));
            }
        }
        Ok(None)
    }

    /// Epoch of the applied collision world (the model's `scene_epoch`);
    /// moves only on an accepted layer replacement.
    fn epoch(&self) -> u64 {
        self.collision.scene_epoch()
    }

    /// Where a `jog_j` on `joint` at `signed_pct` will be one lookahead
    /// horizon from `q`, clamped into the soft window so a pose at the
    /// stop cannot phantom-trip the gate.
    pub(crate) fn jog_lookahead(
        &self,
        q: &[f64; MAX_JOINTS],
        speeds: &[f64; MAX_JOINTS],
    ) -> [f64; MAX_JOINTS] {
        let mut la = *q;
        for (j, pct) in speeds.iter().enumerate() {
            let v = pct * self.jog_vel[j];
            la[j] = (la[j] + self.stopping_travel(j, v)).clamp(self.soft_min[j], self.soft_max[j]);
        }
        la
    }

    /// Where the arm will be one lookahead horizon from `q` if it keeps
    /// moving at `qd`, clamped into the soft window so a pose at the stop
    /// cannot phantom-trip the gate.
    ///
    /// Callers pass the COMMANDED pose, not the measured one. The plant
    /// trails what it has been told by roughly a position-loop time
    /// constant, and that trailing distance is ground the arm will cover
    /// whatever happens next — projecting from the measurement leaves it
    /// out and the arm coasts that much further than the gate priced
    /// for. Measured on the sim rig: projecting from the measurement,
    /// both a 5 mm/50 ms and a 1 mm/50 ms approach ended up inside the
    /// keep-out; from the commanded pose, both stop 1.9 mm clear of it.
    ///
    /// The jog projection asks the same question of a COMMANDED velocity
    /// fraction; this one asks it of the measured motion, which is what a
    /// position stream leaves to read: its datagrams carry a target, not
    /// a rate.
    pub(crate) fn motion_lookahead(
        &self,
        q: &[f64; MAX_JOINTS],
        qd: &[f64; MAX_JOINTS],
    ) -> [f64; MAX_JOINTS] {
        let mut la = *q;
        for (j, v) in qd.iter().enumerate() {
            la[j] = (la[j] + self.stopping_travel(j, *v)).clamp(self.soft_min[j], self.soft_max[j]);
        }
        la
    }

    /// Where joint `joint` comes to rest from `v` \[rad\]; see
    /// [`stream_stopping_travel`].
    pub(crate) fn stopping_travel(&self, joint: usize, v: f64) -> f64 {
        stream_stopping_travel(v, self.position_loop_hz[joint], self.tick_dt_s)
    }

    /// How long the arm holds its speed before a refusal takes effect \[s\].
    pub(crate) fn reaction_s(&self) -> f64 {
        STOP_PIPELINE_TICKS * self.tick_dt_s
    }

    /// The braking half of [`stream_stopping_travel`] \[rad\] — the
    /// cartesian paths integrate the reaction half through the jacobian
    /// instead, so they add only this.
    pub(crate) fn braking_travel(&self, joint: usize, v: f64) -> f64 {
        stream_stopping_travel(v, self.position_loop_hz[joint], self.tick_dt_s)
            - v * self.reaction_s()
    }

    /// Re-report the standing verdict without re-testing anything.
    ///
    /// A stream being walked onto a standoff is under the gate's control,
    /// not the client's, and the client has already been told. Its next
    /// datagram must still be answered with the refusal rather than
    /// accepted, because the server clears the `collision_active` latch
    /// on any command it accepts.
    pub(crate) fn standing_refusal(&self) -> WireError {
        let rendered = self
            .latch
            .pairs
            .iter()
            .take(4)
            .map(|(a, b)| format!("[{a}, {b}]"))
            .collect::<Vec<_>>()
            .join(", ");
        make_error(
            ErrorCode::SysSelfCollision,
            UNATTRIBUTED,
            &[("sample", "0"), ("total", "1"), ("pairs", &rendered)],
        )
    }

    /// Latch `pairs` as the streaming collision verdict and build the
    /// refusal the client reads. One checked configuration, so the error
    /// template's path slots read `0` of `1`.
    pub(crate) fn refuse(&mut self, pairs: Vec<(String, String)>) -> WireError {
        let rendered = pairs
            .iter()
            .take(4)
            .map(|(a, b)| format!("[{a}, {b}]"))
            .collect::<Vec<_>>()
            .join(", ");
        self.latch = CollisionState {
            active: true,
            pairs,
        };
        make_error(
            ErrorCode::SysSelfCollision,
            UNATTRIBUTED,
            &[("sample", "0"), ("total", "1"), ("pairs", &rendered)],
        )
    }
}

/// A collision-world query the shim refused (a broken model, never a
/// well-formed configuration).
fn gate_error(e: par6_kin::KinError) -> WireError {
    make_error(
        ErrorCode::CommValidationError,
        UNATTRIBUTED,
        &[("detail", &format!("stream gate collision world: {e}"))],
    )
}

/// Both channels into the RT thread, bundled (cloneable).
#[derive(Clone)]
pub(crate) struct CoreLink {
    cmds: mpsc::Sender<RtCommand>,
    ops: mpsc::Sender<CoreOp>,
    rt_break: Arc<AtomicBool>,
}

impl CoreLink {
    pub(crate) fn new(
        cmds: mpsc::Sender<RtCommand>,
        ops: mpsc::Sender<CoreOp>,
        rt_break: Arc<AtomicBool>,
    ) -> Self {
        Self {
            cmds,
            ops,
            rt_break,
        }
    }

    /// Queue a tick-loop command (consumed one per tick, in order).
    pub(crate) fn send(&self, cmd: RtCommand) {
        if self.cmds.send(cmd).is_err() {
            log::error!("RT command channel closed; command dropped");
        }
    }

    /// Queue a core op and break the RT loop out of `run()` to apply it.
    pub(crate) fn op(&self, op: CoreOp) {
        if self.ops.send(op).is_ok() {
            self.rt_break.store(true, Ordering::SeqCst);
        } else {
            log::error!("RT op channel closed; op dropped");
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StreamKind {
    Jog,
    Servo,
    /// Cartesian velocity jog (`jog_l`): housekeeping integrates the
    /// twist through the jacobian and streams the joint targets.
    CartJog,
}

/// Live state of a cartesian jog, advanced by housekeeping each period.
#[derive(Clone, Copy)]
pub(crate) struct CartJogState {
    /// Commanded TCP twist `[vx vy vz (m/s), wx wy wz (rad/s)]` in the
    /// commanded frame's axes.
    pub(crate) twist: [f64; 6],
    pub(crate) frame: par6_proto::Frame,
    /// Integrated joint target \[rad\] (the stream setpoint source).
    pub(crate) q: [f64; MAX_JOINTS],
    pub(crate) soft_min: [f64; MAX_JOINTS],
    pub(crate) soft_max: [f64; MAX_JOINTS],
}

struct ActiveStream {
    kind: StreamKind,
    deadline: Instant,
    /// A jog whose watchdog expired and whose engine is ramping to rest.
    /// The session is not over — parol6's executor stays `active` until
    /// its velocity is zero, and a datagram arriving meanwhile just
    /// becomes the ramp's new target — so it stays open here too, until
    /// the RT reports it left JOG, or `deadline` (the ramp cap) passes.
    releasing: bool,
    servo_target: Option<[f64; MAX_JOINTS]>,
    /// The gate refusal this stream is working through, if any.
    ///
    /// A refused stream is no longer following the client — it is being
    /// brought to rest and then placed on the standoff — so it outlives
    /// the client's grace: the client has been told the motion was
    /// refused and has stopped sending, and expiring the stream
    /// mid-travel would abandon the arm wherever it had got to rather
    /// than on the standoff.
    standoff: Option<Standoff>,
    /// The live `jog_j` command: per-joint signed speed fraction, all
    /// zero once the button released. What housekeeping's periodic
    /// collision re-check projects the lookahead from.
    jog: [f64; MAX_JOINTS],
    /// `scene_epoch` of the collision world a held SERVO target was last
    /// checked against. A held target cannot move, so it only needs
    /// re-testing when the WORLD does — this is what housekeeping's
    /// re-check keys on, so the steady state costs no collision queries.
    world_epoch: u64,
    cart: Option<CartJogState>,
    /// Consecutive FRESH snapshots the arm has measured stopped in.
    still: u8,
    /// The tick `still` last counted, so housekeeping running faster than
    /// the RT publishes cannot count one frame eight times.
    still_tick: u64,
    /// The stream's `(speed, accel)` fractions, carried so housekeeping's
    /// keep-alive refeeds the setpoint the client asked for rather than
    /// silently restoring full-speed limits between datagrams.
    scale: (f64, f64),
}

/// The pose a stopping projection starts from.
///
/// The COMMANDED pose when the RT has one: the plant trails what it has
/// been told by roughly a position-loop time constant, and that trailing
/// distance is ground the arm will cover whatever happens next.
///
/// Modes that command no position (IDLE, BOOTING) publish NaN there, and
/// a projection seeded from NaN reaches nowhere and everywhere at once —
/// so the measured pose stands in, which is what it was before the arm
/// was under a position law anyway.
fn projection_seed(snap: &StateSnapshot) -> [f64; MAX_JOINTS] {
    if snap.q_commanded.iter().all(|v| v.is_finite()) {
        snap.q_commanded
    } else {
        snap.q
    }
}

/// A gate refusal being worked through, in two steps.
///
/// The arm cannot simply be commanded onto the standoff: a refusal
/// arrives while it is moving, and at streaming speed the standoff is
/// usually already inside its own stopping distance — commanding it
/// asks the plant for a deceleration it cannot make, and it overshoots
/// and rings around the setpoint instead of landing on it. So the arm is
/// stopped first, under its own limits, and only then placed.
#[derive(Clone, Copy)]
enum Standoff {
    /// Shedding velocity under the executor's release. `goal` is the
    /// refused target, kept because the standoff has to be solved from
    /// wherever the arm actually stops: short of it after a gentle
    /// refusal, past it and inside the keep-out after a fast one.
    Braking { goal: [f64; MAX_JOINTS] },
    /// Travelling the last stretch onto the solved standoff.
    Placing { stop: [f64; MAX_JOINTS] },
}

/// An enable request in flight, retried by housekeeping until the RT
/// answers it or the window closes.
struct EnableRequest {
    /// When to give up and report the controller still DISABLED.
    deadline: Instant,
    /// When the last `Enable` went out (retry spacing).
    last_sent: Option<Instant>,
    /// The core's `enable_seq` sampled just BEFORE that send. The first
    /// snapshot whose `enable_seq` is past it has processed an Enable
    /// belonging to this request, so its `state` is the request's answer
    /// — not a leftover reading from before it.
    sent_at_seq: Option<u64>,
}

/// A FLASHING enter/exit in flight: resolved when the published mode
/// reaches `want`, failed when the window closes first. No retry — the
/// RT processes `SetMode` on the next tick, so the window only covers
/// command-queue latency.
struct FlashingRequest {
    /// The mode the ack is waiting for (`Flashing` on enter, `Idle` on
    /// exit).
    want: Mode,
    /// When to give up and report the mode unchanged.
    deadline: Instant,
}

/// State shared between the bridge (server task) and housekeeping.
#[derive(Default)]
pub(crate) struct SharedState {
    stream: Option<ActiveStream>,
    enable: Option<EnableRequest>,
    /// Resolved enable outcome, waiting to be collected by the server.
    enable_outcome: Option<Result<(), WireError>>,
    flashing: Option<FlashingRequest>,
    /// Resolved FLASHING outcome, waiting to be collected by the server.
    flashing_outcome: Option<Result<(), WireError>>,
}

/// The bridge's kinematics kit (feature `ffi`): its own model instance,
/// the snapshot reader that seeds IK from the measured pose, and the
/// streaming collision gate it shares with housekeeping.
pub(crate) struct CartStream {
    pub(crate) kin: crate::kin::CartKin,
    pub(crate) snapshots: SnapshotReader<StateSnapshot>,
    pub(crate) soft_min: [f64; MAX_JOINTS],
    pub(crate) soft_max: [f64; MAX_JOINTS],
    pub(crate) gate: Arc<Mutex<StreamGate>>,
}

/// The `RtCommands` implementation `par6d` hands to the server.
pub(crate) struct RtBridge {
    link: CoreLink,
    stream_input: Arc<Mutex<StreamInput>>,
    shared: Arc<Mutex<SharedState>>,
    /// Bound for the EXEC flushes `halt` queues (see [`RtBridge::halt`]).
    flush: FlushMarker,
    bundle: Arc<ConfigBundle>,
    sim: bool,
    /// The scene a simulator swap boots on.
    scene: Scene,
    /// Where the running simulator takes world layers from; `None` on
    /// hardware.
    sim_world: Option<WorldMailbox>,
    cart: CartStream,
}

impl RtBridge {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        link: CoreLink,
        stream_input: Arc<Mutex<StreamInput>>,
        shared: Arc<Mutex<SharedState>>,
        flush: FlushMarker,
        bundle: Arc<ConfigBundle>,
        sim: bool,
        scene: Scene,
        sim_world: Option<WorldMailbox>,
        cart: CartStream,
    ) -> Self {
        Self {
            link,
            stream_input,
            shared,
            flush,
            bundle,
            sim,
            scene,
            sim_world,
            cart,
        }
    }

    /// Client silence after which a servo stream ends itself; housekeeping
    /// keeps the RT stream watchdog fed until then.
    fn servo_grace(&self) -> Duration {
        Duration::from_secs_f64(self.bundle.robot.stream.servo_grace_s)
    }

    /// Mode dance into a stream mode. The RT transition table only
    /// allows working-mode changes through IDLE, and `SetMode` to the
    /// current mode is a no-op, so the pair is always safe to queue.
    fn enter_stream_mode(&self, target: Mode) {
        self.link.send(RtCommand::SetMode(Mode::Idle));
        self.link.send(RtCommand::SetMode(target));
    }

    /// Admit a servo target, the way every servo stream is gated: the
    /// target and the arm's own stopping travel must both be clear. A
    /// refusal on a moving arm installs the standoff housekeeping works
    /// through; an arm already at rest is simply refused. Returns the
    /// gate epoch the datagram was admitted under.
    fn admit_servo_target(
        cart: &mut CartStream,
        link: &CoreLink,
        sh: &mut SharedState,
        target: &[f64; MAX_JOINTS],
        scale: (f64, f64),
    ) -> Result<u64, WireError> {
        let target = *target;
        let (refusal, world_epoch) = {
            let snap = cart.snapshots.latest();
            let mut gate = cart.gate.lock().unwrap();
            let epoch = gate.epoch();
            // Two questions, both of which must pass. The target
            // is where the client asked the arm to finish; the
            // lookahead is where the arm's own momentum would
            // carry it if this datagram were the last. Checking
            // only the target admits motion right up to the
            // keep-out and then discovers, one braking distance
            // later, that the arm cannot stop outside it — the
            // same re-check housekeeping already runs on a
            // moving stream, moved to where the datagram is
            // still refusable.
            let la = gate.motion_lookahead(&projection_seed(&snap), &snap.qd);
            // Which question failed decides where the arm is then
            // put. The TARGET failing means the arm must stop
            // before the configuration it was sent to, so the
            // standoff is solved toward that target. The
            // PROJECTION failing means the target is fine and the
            // arm's own momentum is the problem, so it is solved
            // toward the projection instead — solving toward a
            // target that is already clear just parks the arm
            // back on it.
            // The target carries the arm's stopping travel with
            // it: arriving AT a configuration and stopping there
            // are different places, and a target admitted right
            // on the clearance is entered anyway on the way to
            // rest. Measured on the sim rig, that is the whole
            // of what a 1 mm/50 ms approach still had left over
            // once the projection covered everything else.
            let target_stop = gate.motion_lookahead(&target, &snap.qd);
            let verdict = match gate.blocked(&snap.q, &target_stop)? {
                Some(pairs) => Some((pairs, target_stop)),
                None => gate.blocked(&snap.q, &la)?.map(|pairs| (pairs, la)),
            };
            let moving = snap.qd.iter().any(|v| v.abs() > STREAM_MOVING_RAD_S);
            match verdict {
                None => (None, epoch),
                Some((pairs, goal)) => (Some((gate.refuse(pairs), goal, moving)), epoch),
            }
        };
        if let Some((refusal, goal, moving)) = refusal {
            // A standoff sheds momentum. An arm already at rest has
            // none: the refusal is the whole answer, and driving it
            // toward the keep-out it was refused would be motion
            // the client was just told it did not get.
            if !moving {
                return Err(refusal);
            }
            log::warn!("servo: collision predicted; stopping on the standoff");
            // A release, not a position hold. The drive closes a
            // position error against the arm's own momentum, and
            // handing it a hold while the arm still carries
            // velocity is what makes it ring: measured on the sim
            // rig at 10.4 Hz, growing ~8% a cycle, from a 0.14
            // rad/s residual. The release sheds the velocity
            // first; the standoff is commanded from rest.
            link.send(RtCommand::JogRelease);
            link.send(RtCommand::StreamRelease);
            sh.stream = Some(ActiveStream {
                releasing: false,
                kind: StreamKind::Servo,
                // A refused stream outlives the client's
                // silence: the refusal is why it stopped
                // sending, and the arm still has to be stopped
                // and placed.
                deadline: Instant::now() + STANDOFF_TRAVEL_BUDGET,
                servo_target: None,
                standoff: Some(Standoff::Braking { goal }),
                jog: [0.0; MAX_JOINTS],
                world_epoch,
                cart: None,
                still: 0,
                still_tick: 0,
                scale,
            });
            // Reported, not swallowed. A command the server
            // treats as accepted clears the standing collision
            // verdict — so a refusal that returned Ok would
            // wipe, on its own datagram, the STATUS latch that
            // tells the operator the stream was stopped at all.
            return Err(refusal);
        }
        Ok(world_epoch)
    }

    fn stop_stream_commands(&self) {
        self.link.send(RtCommand::JogRelease);
        self.link.send(RtCommand::SetMode(Mode::Idle));
    }
}

impl RtCommands for RtBridge {
    fn stream(&mut self, cmd: &Command) -> Result<(), WireError> {
        match cmd {
            Command::JogJ(p) => {
                let mut speeds = [0.0; MAX_JOINTS];
                for (out, v) in speeds.iter_mut().zip(p.speeds.iter()) {
                    *out = *v;
                }
                let moving = speeds.iter().any(|v| *v != 0.0);
                let mut sh = self.shared.lock().unwrap();
                // Admission gate: where this jog will be one lookahead
                // horizon ahead must not collide (or, from inside a
                // keep-out, must not deepen it). The commanded velocity
                // bounds the RT integrator's ramp from above, so a jog
                // this projection clears cannot outrun it. Every driven
                // joint is projected at once, so the configuration under
                // test is the one the arm will actually be in — a
                // per-joint check would clear two axes that only collide
                // together.
                if moving {
                    let q = self.cart.snapshots.latest().q;
                    let mut gate = self.cart.gate.lock().unwrap();
                    let la = gate.jog_lookahead(&q, &speeds);
                    if let Some(pairs) = gate.blocked(&q, &la)? {
                        return Err(gate.refuse(pairs));
                    }
                }
                let active = match sh.stream {
                    Some(ActiveStream {
                        kind: StreamKind::Jog,
                        jog,
                        ..
                    }) => jog,
                    _ => {
                        self.enter_stream_mode(Mode::Jog);
                        [0.0; MAX_JOINTS]
                    }
                };
                // The RT drains one command per tick, so a client
                // streaming jog faster than the tick rate would grow the
                // queue without bound and leave the release sitting behind
                // its own backlog — the arm keeps jogging after the
                // operator let go. A repeated setpoint carries no new
                // instruction (the jog engine is already ramping to it),
                // so only a CHANGE is worth a command. Holding a control
                // steady therefore costs one command, not one per
                // datagram, and the release is never more than a couple of
                // entries deep. The datagram still refreshes the watchdog
                // deadline below either way.
                let accel_changed = sh
                    .stream
                    .as_ref()
                    .is_some_and(|a| a.scale.1 != p.accel.unwrap_or(1.0));
                if speeds != active || (moving && accel_changed) {
                    if moving {
                        self.link.send(RtCommand::Jog {
                            speeds,
                            accel: p.accel.unwrap_or(1.0),
                        });
                    } else {
                        self.link.send(RtCommand::JogRelease);
                    }
                }
                sh.stream = Some(ActiveStream {
                    releasing: false,
                    kind: StreamKind::Jog,
                    deadline: jog_deadline(p.duration),
                    servo_target: None,
                    standoff: None,
                    jog: speeds,
                    world_epoch: 0,
                    cart: None,
                    still: 0,
                    still_tick: 0,
                    // JOG runs on the RT jog engine, not the streaming
                    // executor; its accel rides `RtCommand::Jog`. Kept
                    // here so a change of accel alone still resends.
                    scale: (1.0, p.accel.unwrap_or(1.0)),
                });
            }
            Command::ServoJ(p) => {
                let mut target = [0.0; MAX_JOINTS];
                for (t, a) in target.iter_mut().zip(p.angles.iter()) {
                    *t = a.to_radians();
                }
                let mut sh = self.shared.lock().unwrap();
                // A refusal already in progress owns the arm until it is
                // on the standoff. Accepting a datagram here would throw
                // that away mid-placement — and, because the server
                // clears the standing collision verdict on any command it
                // accepts, would also wipe the latch that says the stream
                // was stopped.
                if matches!(
                    sh.stream,
                    Some(ActiveStream {
                        standoff: Some(_),
                        ..
                    })
                ) {
                    return Err(self.cart.gate.lock().unwrap().standing_refusal());
                }
                // Servo targets are explicit configurations, so the gate
                // checks the target itself — each datagram is its own
                // admission check, which is the streaming cadence parol6
                // gates at.
                // A refused target is not simply dropped. Dropping it
                // leaves the arm wherever its tracking lag put it, which
                // is not a distance anyone chose — measured on the sim
                // rig at anywhere from 2.7 mm to 24 mm from a keep-out
                // whose standoff is 5 mm. Instead the arm is stopped and
                // then placed ON the standoff; housekeeping runs that.
                let scale = (p.speed.unwrap_or(1.0), p.accel.unwrap_or(1.0));
                let world_epoch =
                    Self::admit_servo_target(&mut self.cart, &self.link, &mut sh, &target, scale)?;
                if !matches!(
                    sh.stream,
                    Some(ActiveStream {
                        kind: StreamKind::Servo,
                        ..
                    })
                ) {
                    self.enter_stream_mode(Mode::Stream);
                }
                self.stream_input.lock().unwrap().send(&StreamSetpoint {
                    q: target,
                    speed: scale.0,
                    accel: scale.1,
                });
                sh.stream = Some(ActiveStream {
                    releasing: false,
                    kind: StreamKind::Servo,
                    deadline: Instant::now() + self.servo_grace(),
                    servo_target: Some(target),
                    standoff: None,
                    jog: [0.0; MAX_JOINTS],
                    world_epoch,
                    cart: None,
                    still: 0,
                    still_tick: 0,
                    scale,
                });
            }
            // Cartesian position streams: seeded IK, then the exact
            // servo_j path. An unreachable target drops the datagram
            // (fire-and-forget has no reply channel) — the arm must not
            // move on a pose the solver cannot reach.
            Command::ServoJPose(par6_proto::command::ServoJPose {
                pose, speed, accel, ..
            })
            | Command::ServoL(par6_proto::command::ServoL {
                pose, speed, accel, ..
            }) => {
                let mut sh = self.shared.lock().unwrap();
                let seed = match &sh.stream {
                    Some(ActiveStream {
                        kind: StreamKind::Servo,
                        servo_target: Some(t),
                        standoff: None,
                        ..
                    }) => *t,
                    _ => self.cart.snapshots.latest().q,
                };
                let target_pose = crate::kin::wire_pose_to_matrix(pose);
                let mut target = match self.cart.kin.ik(&seed, &target_pose) {
                    crate::kin::IkResult::Solved(q) => q,
                    crate::kin::IkResult::Unreachable => {
                        log::warn!("{:?}: target pose unreachable; dropped", cmd.tag());
                        return Ok(());
                    }
                    crate::kin::IkResult::Failed(e) => {
                        log::warn!("{:?}: IK failed ({e}); dropped", cmd.tag());
                        return Ok(());
                    }
                };
                for (j, v) in target.iter_mut().enumerate() {
                    *v = v.clamp(self.cart.soft_min[j], self.cart.soft_max[j]);
                }
                let scale = (speed.unwrap_or(1.0), accel.unwrap_or(1.0));
                let world_epoch =
                    Self::admit_servo_target(&mut self.cart, &self.link, &mut sh, &target, scale)?;
                if !matches!(
                    sh.stream,
                    Some(ActiveStream {
                        kind: StreamKind::Servo,
                        ..
                    })
                ) {
                    self.enter_stream_mode(Mode::Stream);
                }
                self.stream_input.lock().unwrap().send(&StreamSetpoint {
                    q: target,
                    speed: scale.0,
                    accel: scale.1,
                });
                sh.stream = Some(ActiveStream {
                    releasing: false,
                    kind: StreamKind::Servo,
                    deadline: Instant::now() + self.servo_grace(),
                    servo_target: Some(target),
                    standoff: None,
                    jog: [0.0; MAX_JOINTS],
                    world_epoch,
                    cart: None,
                    still: 0,
                    still_tick: 0,
                    scale,
                });
            }
            // Cartesian velocity jog: housekeeping steps the twist
            // through the jacobian each period until the watchdog
            // duration elapses.
            Command::JogL(p) => {
                let mut sh = self.shared.lock().unwrap();
                let q = match &sh.stream {
                    Some(ActiveStream {
                        kind: StreamKind::CartJog,
                        cart: Some(state),
                        still: 0,
                        still_tick: 0,
                        ..
                    }) => state.q,
                    _ => self.cart.snapshots.latest().q,
                };
                let mut twist = [0.0; 6];
                let motion = &self.bundle.robot.motion;
                for (i, (out, frac)) in twist.iter_mut().zip(p.velocities.iter()).enumerate() {
                    let full = if i < 3 {
                        motion.jog_l_linear_max_m_s
                    } else {
                        motion.jog_l_angular_max_rad_s
                    };
                    *out = frac * full;
                }
                // Admission gate on the projected lookahead. A twist the
                // jacobian cannot resolve is admitted — housekeeping
                // holds in place on every failed solve, so nothing
                // unchecked ever streams.
                let mut probe = CartJogState {
                    twist,
                    frame: p.frame,
                    q,
                    soft_min: self.cart.soft_min,
                    soft_max: self.cart.soft_max,
                };
                let reaction_s = self.cart.gate.lock().unwrap().reaction_s();
                if let Ok((mut la, qd)) = step_cart_jog(&mut self.cart.kin, &mut probe, reaction_s)
                {
                    let mut gate = self.cart.gate.lock().unwrap();
                    for (j, v) in la.iter_mut().enumerate() {
                        *v = (*v + gate.braking_travel(j, qd[j]))
                            .clamp(probe.soft_min[j], probe.soft_max[j]);
                    }
                    if let Some(pairs) = gate.blocked(&q, &la)? {
                        return Err(gate.refuse(pairs));
                    }
                }
                if !matches!(
                    sh.stream,
                    Some(ActiveStream {
                        kind: StreamKind::CartJog,
                        ..
                    })
                ) {
                    self.enter_stream_mode(Mode::Stream);
                }
                sh.stream = Some(ActiveStream {
                    releasing: false,
                    kind: StreamKind::CartJog,
                    deadline: jog_deadline(p.duration),
                    servo_target: None,
                    standoff: None,
                    jog: [0.0; MAX_JOINTS],
                    world_epoch: 0,
                    still: 0,
                    still_tick: 0,
                    cart: Some(CartJogState {
                        twist,
                        frame: p.frame,
                        q,
                        soft_min: self.cart.soft_min,
                        soft_max: self.cart.soft_max,
                    }),
                    // A cartesian jog is integrated into joint targets and
                    // then tracked by the streaming executor, so its accel
                    // fraction is the stream's.
                    scale: (1.0, p.accel.unwrap_or(1.0)),
                });
            }
            other => log::warn!("unexpected stream command {:?}", other.tag()),
        }
        Ok(())
    }

    fn cancel_stream(&mut self) {
        self.shared.lock().unwrap().stream = None;
        self.stop_stream_commands();
    }

    fn stop_refused_stream(&mut self) -> bool {
        let mut shared = self.shared.lock().unwrap();
        if shared.stream.as_ref().is_some_and(|a| a.standoff.is_some()) {
            // The collision gate owns braking and placement. Cancelling it
            // on a late datagram would abandon the configured standoff.
            return true;
        }
        shared.stream = None;
        drop(shared);
        self.stop_stream_commands();
        false
    }

    fn discard_exec(&mut self) {
        // Marked before it is queued, for the reason `halt` gives: the
        // mark is pinned to what is in the ring now, so a move accepted
        // behind this keeps its own fill.
        self.flush.mark();
        self.link.send(RtCommand::ExecFlush);
        self.link.send(RtCommand::SetMode(Mode::Idle));
    }

    fn halt(&mut self) {
        self.shared.lock().unwrap().stream = None;
        self.link.send(RtCommand::JogRelease);
        // Marked before it is queued, so the flush is pinned to the
        // samples in the ring right now: a move accepted while this
        // stop is still working its way through the RT command queue
        // keeps its own fill.
        self.flush.mark();
        self.link.send(RtCommand::ExecFlush);
        self.link.send(RtCommand::SetMode(Mode::Idle));
    }

    fn set_gravity_comp(&mut self, on: bool) {
        self.link.send(RtCommand::SetGravityComp(on));
    }

    fn set_payload(&mut self, payload: par6_server::PayloadSpec) {
        self.link.send(RtCommand::SetPayload {
            mass: payload.mass,
            com: payload.com,
            inertia: payload.inertia,
        });
    }

    fn set_exec_paused(&mut self, paused: bool) {
        self.link.send(RtCommand::ExecSetPaused(paused));
    }

    fn set_enabled(&mut self, enabled: bool) {
        let mut sh = self.shared.lock().unwrap();
        if enabled {
            // Clear the soft e-stop flag and run the RT clear sequence;
            // Enable only succeeds after the clear settle window, so
            // housekeeping retries it until the core answers.
            self.link.send(RtCommand::SetSoftEstop(false));
            self.link.send(RtCommand::ClearErrors);
            // A verdict nobody collected belongs to the request it came
            // from; this one gets its own answer, never an inherited one.
            sh.enable_outcome = None;
            sh.enable = Some(EnableRequest {
                deadline: Instant::now() + ENABLE_RETRY_WINDOW,
                last_sent: None,
                sent_at_seq: None,
            });
        } else {
            self.link.send(RtCommand::SetSoftEstop(true));
            if sh.enable.take().is_some() {
                // An enable that an e-stop overtook did not happen, and
                // whoever is waiting on it must be told so.
                sh.enable_outcome = Some(Err(make_error(
                    ErrorCode::SysEstopActive,
                    UNATTRIBUTED,
                    &[],
                )));
            }
        }
    }

    fn take_enable_outcome(&mut self) -> Option<Result<(), WireError>> {
        self.shared.lock().unwrap().enable_outcome.take()
    }

    fn enter_flashing(&mut self) {
        let mut sh = self.shared.lock().unwrap();
        // The assertion rides the same queue as the mode request, so the
        // core consumes them in order; any transition in between drops
        // the one-shot assertion, which is the safety property intended.
        self.link.send(RtCommand::AssertParked);
        self.link.send(RtCommand::SetMode(Mode::Flashing));
        sh.flashing_outcome = None;
        sh.flashing = Some(FlashingRequest {
            want: Mode::Flashing,
            deadline: Instant::now() + FLASHING_WINDOW,
        });
    }

    fn exit_flashing(&mut self) {
        let mut sh = self.shared.lock().unwrap();
        self.link.send(RtCommand::SetMode(Mode::Idle));
        sh.flashing_outcome = None;
        sh.flashing = Some(FlashingRequest {
            want: Mode::Idle,
            deadline: Instant::now() + FLASHING_WINDOW,
        });
    }

    fn take_flashing_outcome(&mut self) -> Option<Result<(), WireError>> {
        self.shared.lock().unwrap().flashing_outcome.take()
    }

    fn set_pid_gains(&mut self, p: &par6_proto::command::SetPidGains) {
        self.link.send(RtCommand::RetuneNode {
            node: p.node,
            tune: par6_bus::DriveTune {
                gains: par6_config::Gains {
                    kpp: p.kpp,
                    kpv: p.kpv,
                    kiv: p.kiv,
                    kpiq: p.kpiq,
                    kiiq: p.kiiq,
                    kp: p.kp,
                    kd: p.kd,
                },
                ilim_ma: p.ilim_ma,
                velocity_limit_ticks_s: p.velocity_limit_ticks_s,
                voltage_limit_mv: p.voltage_limit_mv,
            },
        });
    }

    fn set_can_id(&mut self, node: u8, new_id: u8) {
        self.link.send(RtCommand::SetCanId { node, new_id });
    }

    fn save_config(&mut self, node: u8) {
        self.link.send(RtCommand::SaveConfig { node });
    }

    fn rescan_bus(&mut self) {
        self.link.send(RtCommand::RescanBus);
    }

    fn teleport(&mut self, angles_deg: &[f64; NUM_JOINTS], tool_positions: Option<&[f64]>) {
        if !self.sim {
            // The server gates teleport with SYS_NOT_SIMULATOR; this is
            // pure defense in depth.
            log::error!("teleport outside simulator mode reached the bridge; dropped");
            return;
        }
        // The tool DOF is re-seeded like a joint: the jaw jumps, and the
        // standing firmware frame is re-aimed at where it landed so the
        // onboard controller holds it there instead of driving back to
        // the previous target. The server validates count and range.
        let tool_closed = tool_positions.and_then(|p| p.first().copied());
        if let Some(closed) = tool_closed {
            let hold_ma = self
                .bundle
                .active_gripper()
                .and_then(|g| g.driver.as_ref())
                .map(|d| d.ilim_ma)
                .unwrap_or(0.0);
            self.link.send(RtCommand::Gripper(gripper_move_command(
                closed, 1.0, hold_ma,
            )));
        }
        let bundle = self.bundle.clone();
        // Taken as given: the server refuses any angle outside the joint's
        // hard window before it reaches here, so the arm lands exactly
        // where the client asked or the command never runs.
        let mut q = [0.0; MAX_JOINTS];
        for (out, deg) in q.iter_mut().zip(angles_deg.iter()) {
            *out = deg.to_radians();
        }
        self.link.op(Box::new(move |core| {
            let robot = &bundle.robot;
            let Some(bus) = core.bus_mut().sim_mut() else {
                log::error!("teleport reached a hardware bus; dropped");
                return;
            };
            // Re-seed, not a bus reboot: the drivers keep running, so the
            // arm is still held the tick after it lands.
            if let Err(e) = bus.teleport_joint_rad(&q[..robot.joints.len()]) {
                log::error!("teleport: sim re-seed failed: {e}");
                return;
            }
            if let Some(closed) = tool_closed {
                if let Err(e) = bus.teleport_gripper(closed) {
                    log::error!("teleport: sim tool re-seed failed: {e}");
                }
            }
            for (i, joint) in robot.joints.iter().enumerate() {
                // The re-seeded sim reports the wrapped boot reading
                // first; re-base the core's conversion so that reading
                // maps exactly to the teleported angle.
                let conv = par6_bus::spectral::JointConversion::from_config(joint);
                let true0 = conv.motor_ticks(q[i]);
                let wrapped0 = true0.rem_euclid(1i32 << joint.encoder_bits);
                core.set_joint_reference(i, wrapped0, q[i]);
            }
            core.reseed_motion_targets();
            core.set_homed(true);
            log::info!("teleport applied: {q:?} rad, homed=true");
        }));
    }

    fn write_io(&mut self, port: u8, value: u8) {
        // The server has already checked `port` against the declared
        // outputs, so this only forwards; the RT thread owns the pins
        // and drives them on the tick that consumes the command.
        self.link.send(RtCommand::WriteIo { port, value });
    }

    fn set_simulator(&mut self, on: bool) -> Result<(), WireError> {
        if on == self.sim {
            return Ok(());
        }
        if on {
            self.swap_to_sim()
        } else {
            self.swap_to_hardware(&self.bundle.robot.bus.interface.clone())
        }
    }

    fn connect_hardware(&mut self, port: &str) -> Result<(), WireError> {
        self.swap_to_hardware(port)
    }

    fn reset_state(&mut self) {
        // Clear latched errors; the soft e-stop FLAG is untouched, so an
        // active e-stop re-latches — reset_state must not clear the
        // e-stop latch (protocol contract).
        self.link.send(RtCommand::ClearErrors);
    }

    fn reset_loop_stats(&mut self) {
        self.link.op(Box::new(|core| core.reset_loop_stats()));
    }

    fn set_shapes(
        &mut self,
        layer: ShapeLayer,
        shapes: &[par6_proto::Shape],
    ) -> Result<(), WireError> {
        self.cart.gate.lock().unwrap().set_layer(layer, shapes)
    }

    fn collision(&mut self) -> Option<CollisionState> {
        Some(self.cart.gate.lock().unwrap().latch.clone())
    }

    fn clear_collision(&mut self) {
        self.cart.gate.lock().unwrap().latch = CollisionState::default();
    }
}

impl RtBridge {
    /// Swap the running bus for a fresh simulator, seeded at the pose
    /// the arm was last measured at.
    ///
    /// Seeding is what makes this a mode change rather than a teleport:
    /// a simulator started at its own default would jump the model to
    /// the park pose the instant an operator flipped the toggle, and
    /// every client watching STATUS would see the arm move. The home
    /// reference survives with it — the pose it refers to is the pose
    /// the sim now holds — which is the one direction where keeping it
    /// is true.
    ///
    /// What this does NOT do is stop the physical arm. The drivers keep
    /// whatever they were last commanded until their own watchdogs fire
    /// (`bus.watchdog_action`), so on hardware this is a way to stop
    /// LOOKING at the arm, not a way to park it.
    fn swap_to_sim(&mut self) -> Result<(), WireError> {
        let sim = SimBus::new(self.scene.clone());
        self.sim_world = Some(sim.mailbox());
        let bundle = self.bundle.clone();
        self.sim = true;
        self.link.op(Box::new(move |core| {
            let q = core.measured_q();
            if let Err(e) = core.replace_bus(RuntimeBus::from(sim)) {
                log::error!("simulator swap refused: {e}");
                return;
            }
            let robot = &bundle.robot;
            let n = robot.joints.len();
            let Some(bus) = core.bus_mut().sim_mut() else {
                log::error!("the simulator swap did not install a simulator");
                return;
            };
            if let Err(e) = bus.teleport_joint_rad(&q[..n]) {
                log::error!("simulator swap: plant re-seed failed: {e}");
                return;
            }
            for (i, joint) in robot.joints.iter().enumerate() {
                // Same re-basing the teleport path uses: the re-seeded
                // sim reports the WRAPPED boot reading first, so the
                // conversion has to be told which revolution it is on
                // before that reading is interpreted.
                let conv = par6_bus::spectral::JointConversion::from_config(joint);
                let true0 = conv.motor_ticks(q[i]);
                let wrapped0 = true0.rem_euclid(1i32 << joint.encoder_bits);
                core.set_joint_reference(i, wrapped0, q[i]);
            }
            core.reseed_motion_targets();
            core.set_homed(true);
            log::info!("bus backend: simulator, seeded at {q:?} rad");
        }));
        Ok(())
    }

    /// Swap the running bus for SocketCAN on `interface`.
    ///
    /// The interface is opened HERE, on the command plane, because that
    /// is the only place a failure has a client to answer: a missing
    /// interface, a missing `CAP_NET_ADMIN` or a wrong bitrate becomes
    /// the reply to this command instead of a line in the journal.
    ///
    /// Homing does not survive: the arm's real joints are wherever they
    /// are, and a home reference carried over from a simulator refers to
    /// a pose the physical arm was never in. The core drops it, so the
    /// first motion command afterwards is refused as un-homed rather
    /// than run against a fiction.
    fn swap_to_hardware(&mut self, interface: &str) -> Result<(), WireError> {
        let mut cfg = self.bundle.robot.bus.clone();
        interface.clone_into(&mut cfg.interface);
        // The same opener as boot: a driver power-cycle leaves the
        // interface enumerating for a moment, and the retry window absorbs
        // it here too. The server task blocks for at most the window.
        let hw = crate::daemon::open_with_retry(
            cfg.open_retry_s,
            || SocketCanBus::open(&cfg),
            std::thread::sleep,
        )
        .map_err(|e| {
            make_error(
                ErrorCode::MotnSetupFailed,
                UNATTRIBUTED,
                &[("detail", &format!("cannot open '{}': {e}", cfg.interface))],
            )
        })?;
        self.sim = false;
        let name = cfg.interface.clone();
        self.link.op(Box::new(move |core| {
            if let Err(e) = core.replace_bus(RuntimeBus::from(hw)) {
                log::error!("hardware swap refused: {e}");
                return;
            }
            log::info!("bus backend: SocketCAN on '{name}' (un-homed)");
        }));
        Ok(())
    }
}

/// Timed follow-throughs that the datagram-driven bridge cannot run
/// itself: jog duration watchdog, servo keep-alive + silence timeout,
/// and the enable retry that resolves a `reset` into a real answer.
#[allow(clippy::too_many_arguments)]
pub(crate) fn housekeeping_loop(
    dt: f64,
    servo_grace: Duration,
    jog_accel_time_s: f64,
    link: CoreLink,
    stream_input: Arc<Mutex<StreamInput>>,
    shared: Arc<Mutex<SharedState>>,
    mut snapshots: SnapshotReader<StateSnapshot>,
    shutdown: Arc<AtomicBool>,
    mut kin: crate::kin::CartKin,
    gate: Arc<Mutex<StreamGate>>,
) {
    // Stops the live stream because its next step is collision-blocked:
    // latch the verdict for STATUS and put the RT back to IDLE. The
    // abrupt stop is deliberate — the alternative is driving on toward
    // contact (parol6 halts its joint jog on the same prediction).
    let collision_stop = |link: &CoreLink,
                          gate: &Arc<Mutex<StreamGate>>,
                          what: &str,
                          pairs: Vec<(String, String)>| {
        log::warn!("{what}: collision predicted; stopping the stream");
        gate.lock().unwrap().refuse(pairs);
        // Both releases, because either mode may be the one running and
        // each ignores the release that is not its own. They ramp the
        // arm to rest under its limits and hand the mode to IDLE
        // themselves once it is there.
        //
        // Not `SetMode(Idle)`: IDLE holds against gravity and has no
        // velocity authority, so a moving arm dropped into it keeps its
        // momentum. A refused servo stream coasted ~164 mm that way —
        // straight through the keep-out the refusal was about, however
        // early the gate saw it coming.
        link.send(RtCommand::JogRelease);
        link.send(RtCommand::StreamRelease);
    };
    // Longer than any ramp the config can ask for (the s-curve profile
    // adds jerk phases to the linear time); only reached if the RT never
    // reports rest.
    let jog_ramp_cap = Duration::from_secs_f64(4.0 * jog_accel_time_s);
    let mut profile_logged = Instant::now();
    'housekeeping: while !shutdown.load(Ordering::SeqCst) {
        let now = Instant::now();
        let snap = snapshots.latest();
        if now.duration_since(profile_logged) >= Duration::from_secs(1) {
            profile_logged = now;
            let p = &snap.tick_profile;
            if p.phase_max_ns.iter().any(|&n| n > 0) {
                log::info!(
                    target: "par6d::profile",
                    "tick phases max [us] {:?} last overrun [us] {:?} overruns traced {}",
                    p.phase_max_ns.map(|n| n / 1000),
                    p.overrun_ns.map(|n| n / 1000),
                    p.overruns_traced
                );
            }
        }
        {
            let mut sh = shared.lock().unwrap();
            // The standoff arm leaves through the block label, not
            // `continue`: the enable/flashing bookkeeping below and the
            // period sleep must run on every iteration, however long a
            // brake-and-place takes.
            'stream: {
                match &mut sh.stream {
                    // The ramp reached rest and the RT left JOG on its own:
                    // the session is over.
                    Some(a) if a.releasing && snap.mode != Mode::Jog => {
                        sh.stream = None;
                    }
                    // Working through a gate refusal: brake to rest, then
                    // place the arm on the standoff. The client is not
                    // sending any more — it was told the motion was refused
                    // — so the grace below would otherwise end the stream in
                    // transit and leave the arm wherever the brake happened
                    // to stop it.
                    Some(a) if a.standoff.is_some() => {
                        let phase = a.standoff.expect("checked by the guard");
                        let expired = now >= a.deadline;
                        match phase {
                            Standoff::Braking { goal } => {
                                if snap.tick != a.still_tick {
                                    a.still_tick = snap.tick;
                                    a.still =
                                        if snap.qd.iter().all(|v| v.abs() <= STREAM_MOVING_RAD_S) {
                                            a.still.saturating_add(1)
                                        } else {
                                            0
                                        };
                                }
                                if a.still < STANDOFF_STILL_TICKS && !expired {
                                    break 'stream;
                                }
                                if a.still < STANDOFF_STILL_TICKS {
                                    log::warn!(
                                        "the arm did not come to rest after a refusal; idling"
                                    );
                                    link.send(RtCommand::SetMode(Mode::Idle));
                                    sh.stream = None;
                                    break 'stream;
                                }
                                // Solved from where the arm ACTUALLY stopped:
                                // short of the standoff after a gentle
                                // refusal, past it and inside the keep-out
                                // after a fast one. `stop_point` answers
                                // both, forwards and backwards.
                                let stop = gate.lock().unwrap().stop_point(&snap.q, &goal);
                                let Ok(stop) = stop else {
                                    log::error!("the gate could not solve a standoff; idling");
                                    link.send(RtCommand::SetMode(Mode::Idle));
                                    sh.stream = None;
                                    break 'stream;
                                };
                                // Backed off the boundary by the settle
                                // margin, along the line the standoff was
                                // solved on and away from what refused it.
                                let stop = {
                                    let mut back = stop;
                                    let span = (0..par6_kin::NQ)
                                        .map(|j| (goal[j] - stop[j]).abs())
                                        .fold(0.0, f64::max);
                                    if span > 0.0 {
                                        let k = STANDOFF_SETTLE_MARGIN_RAD / span;
                                        for j in 0..par6_kin::NQ {
                                            back[j] = stop[j] - k * (goal[j] - stop[j]);
                                        }
                                    }
                                    back
                                };
                                if snap
                                    .q
                                    .iter()
                                    .zip(stop.iter())
                                    .all(|(q, s)| (q - s).abs() <= STANDOFF_ARRIVED_RAD)
                                {
                                    link.send(RtCommand::SetMode(Mode::Idle));
                                    sh.stream = None;
                                    break 'stream;
                                }
                                link.send(RtCommand::SetMode(Mode::Idle));
                                link.send(RtCommand::SetMode(Mode::Stream));
                                stream_input.lock().unwrap().send(&StreamSetpoint {
                                    q: stop,
                                    speed: STANDOFF_PLACEMENT_SCALE.0,
                                    accel: STANDOFF_PLACEMENT_SCALE.1,
                                });
                                a.standoff = Some(Standoff::Placing { stop });
                                a.deadline = now + STANDOFF_TRAVEL_BUDGET;
                                a.still = 0;
                                a.still_tick = 0;
                                break 'stream;
                            }
                            Standoff::Placing { stop } => {
                                // Arrival is measured in POSITION, not in
                                // speed: a snapshot's velocity passes through
                                // zero whenever the executor re-plans, and
                                // ending the stream on that reading abandons
                                // the arm mid-travel.
                                // At the standoff AND stopped on it. IDLE has
                                // no velocity authority — it holds against
                                // gravity and nothing else — so an arm handed
                                // over while it still carries speed coasts off
                                // the standoff it was just placed on: measured
                                // on the sim rig at 10.5 degrees of drift-back
                                // after an otherwise exact placement.
                                let arrived = snap
                                    .q
                                    .iter()
                                    .zip(stop.iter())
                                    .all(|(q, s)| (q - s).abs() <= STANDOFF_ARRIVED_RAD)
                                    && snap.qd.iter().all(|v| v.abs() <= STREAM_MOVING_RAD_S);
                                if arrived {
                                    // Held, not idled. IDLE has no position
                                    // authority — it holds against gravity and
                                    // nothing else — so an arm handed to it
                                    // settles back off the standoff under
                                    // drivetrain friction. Handing the stream
                                    // its own target instead leaves the arm
                                    // exactly where a client commanding the
                                    // standoff would have left it, and the
                                    // normal servo lifecycle ends it.
                                    a.standoff = None;
                                    a.servo_target = Some(stop);
                                    a.deadline = now + servo_grace;
                                    break 'stream;
                                }
                                if expired {
                                    log::warn!(
                                    "the standoff was not reached within the travel budget; idling"
                                );
                                    // The RT is still in STREAM, and a stream
                                    // left unfed trips the streaming watchdog
                                    // and latches a fault on an arm that did
                                    // exactly what the gate told it to.
                                    link.send(RtCommand::SetMode(Mode::Idle));
                                    sh.stream = None;
                                    break 'stream;
                                }
                                let mut next = snap.q;
                                for j in 0..par6_kin::NQ {
                                    next[j] = snap.q[j]
                                        + (stop[j] - snap.q[j])
                                            .clamp(-STANDOFF_CREEP_RAD, STANDOFF_CREEP_RAD);
                                }
                                stream_input.lock().unwrap().send(&StreamSetpoint {
                                    q: next,
                                    speed: STANDOFF_PLACEMENT_SCALE.0,
                                    accel: STANDOFF_PLACEMENT_SCALE.1,
                                });
                                break 'stream;
                            }
                        }
                    }
                    Some(a) if now >= a.deadline => {
                        match a.kind {
                            // Released rather than idled: `JogRelease` zeroes
                            // the engine's target but not its velocity, and
                            // the RT only ticks the engine in JOG, so cutting
                            // to IDLE here would stop the arm dead from full
                            // jog speed. The session stays open while the
                            // ramp runs, so a re-press joins it instead of
                            // bouncing the RT through IDLE.
                            StreamKind::Jog if !a.releasing => {
                                log::debug!("jog duration elapsed; releasing");
                                link.send(RtCommand::JogRelease);
                                a.releasing = true;
                                a.jog = [0.0; MAX_JOINTS];
                                a.deadline = now + jog_ramp_cap;
                                continue 'housekeeping;
                            }
                            StreamKind::Jog => {
                                log::warn!("jog ramp never reported rest; idling");
                                link.send(RtCommand::SetMode(Mode::Idle));
                            }
                            StreamKind::Servo => {
                                log::debug!("servo stream went silent; stopping");
                                link.send(RtCommand::SetMode(Mode::Idle));
                            }
                            StreamKind::CartJog => {
                                log::debug!("jog_l duration elapsed; stopping");
                                link.send(RtCommand::SetMode(Mode::Idle));
                            }
                        }
                        sh.stream = None;
                    }
                    // The moving-jog re-check: the admission gate saw the
                    // configuration the jog STARTED at, and the arm has
                    // moved since. Every period the lookahead is projected
                    // from the measured pose and re-tested — against the
                    // world as it is NOW, so a keep-out dropped onto a
                    // running jog stops it too.
                    Some(a) if a.kind == StreamKind::Jog && a.jog.iter().any(|v| *v != 0.0) => {
                        let speeds = a.jog;
                        let mut g = gate.lock().unwrap();
                        let la = g.jog_lookahead(&snap.q, &speeds);
                        match g.blocked(&snap.q, &la) {
                            Ok(None) => {}
                            Ok(Some(pairs)) => {
                                drop(g);
                                collision_stop(&link, &gate, "jog_j", pairs);
                                sh.stream = None;
                            }
                            Err(e) => {
                                // A world the gate cannot query gates
                                // nothing it can prove; stop rather than
                                // stream unchecked.
                                drop(g);
                                log::error!("jog_j gate check failed: {}", e.cause);
                                link.send(RtCommand::JogRelease);
                                link.send(RtCommand::SetMode(Mode::Idle));
                                sh.stream = None;
                            }
                        }
                    }
                    // A servo stream that is MOVING gets the jog treatment:
                    // its next datagram is admitted on the target it carries,
                    // which says nothing about the ground the arm covers
                    // getting there — and it cannot stop dead, so a target
                    // admitted just outside a keep-out is entered anyway on
                    // the braking distance. The measured motion is projected
                    // to where it would come to rest and re-tested every
                    // period, which is the same promise the jog path makes.
                    Some(a)
                        if a.kind == StreamKind::Servo
                            && snap.qd.iter().any(|v| v.abs() > STREAM_MOVING_RAD_S) =>
                    {
                        let mut g = gate.lock().unwrap();
                        let la = g.motion_lookahead(&projection_seed(&snap), &snap.qd);
                        match g.blocked(&snap.q, &la) {
                            Ok(None) => {}
                            Ok(Some(pairs)) => {
                                // Brake, then place — see `Standoff`. The
                                // goal is `la`, not the servo target: the
                                // target is what the arm is still permitted
                                // to reach, so solving a standoff toward it
                                // is a no-op. `la` is the configuration that
                                // just failed the check, so the last
                                // admitted point on the way to it is the
                                // standoff itself.
                                drop(g);
                                collision_stop(&link, &gate, "servo", pairs);
                                a.standoff = Some(Standoff::Braking { goal: la });
                                a.servo_target = None;
                                a.deadline = now + STANDOFF_TRAVEL_BUDGET;
                                continue 'housekeeping;
                            }
                            Err(e) => {
                                drop(g);
                                log::error!("servo gate check failed: {}", e.cause);
                                link.send(RtCommand::SetMode(Mode::Idle));
                                sh.stream = None;
                                continue 'housekeeping;
                            }
                        }
                    }
                    Some(a) if a.kind == StreamKind::Servo => {
                        // A held servo target was admitted against the world
                        // of its datagram, and a target the arm has settled on
                        // cannot move — so it is re-tested exactly when the
                        // WORLD changes (the analogue of the planner's
                        // in-flight revalidation), never per period: a resting
                        // stream costs no collision queries, so the keep-alive
                        // below is never starved past the RT stream watchdog.
                        if let Some(t) = a.servo_target {
                            let epoch = gate.lock().unwrap().epoch();
                            if epoch != a.world_epoch {
                                a.world_epoch = epoch;
                                let verdict = gate.lock().unwrap().blocked(&snap.q, &t);
                                match verdict {
                                    Ok(None) => {}
                                    Ok(Some(pairs)) => {
                                        collision_stop(&link, &gate, "servo", pairs);
                                        sh.stream = None;
                                        continue 'housekeeping;
                                    }
                                    Err(e) => {
                                        // A world the gate cannot query gates
                                        // nothing it can prove; stop (without
                                        // a collision verdict — this is a
                                        // model failure) rather than stream
                                        // unchecked.
                                        log::error!("servo gate check failed: {}", e.cause);
                                        link.send(RtCommand::SetMode(Mode::Idle));
                                        sh.stream = None;
                                        continue 'housekeeping;
                                    }
                                }
                            }
                        }
                        // Keep the RT stream watchdog fed between client
                        // datagrams (its timeout is shorter than the grace).
                        if let Some(t) = a.servo_target {
                            stream_input.lock().unwrap().send(&StreamSetpoint {
                                q: t,
                                speed: a.scale.0,
                                accel: a.scale.1,
                            });
                        }
                    }
                    Some(a) if a.kind == StreamKind::CartJog => {
                        if let Some(state) = &mut a.cart {
                            let before = state.q;
                            match step_cart_jog(&mut kin, state, dt) {
                                Ok((target, qd)) => {
                                    // Where the arm comes to rest if this
                                    // step turns out to be the last one
                                    // admitted.
                                    let mut la = target;
                                    let verdict = {
                                        let mut g = gate.lock().unwrap();
                                        for (j, v) in la.iter_mut().enumerate() {
                                            *v = (*v + g.stopping_travel(j, qd[j]))
                                                .clamp(state.soft_min[j], state.soft_max[j]);
                                        }
                                        g.blocked(&before, &la)
                                    };
                                    match verdict {
                                        Ok(None) => {
                                            stream_input.lock().unwrap().send(&StreamSetpoint {
                                                q: target,
                                                speed: a.scale.0,
                                                accel: a.scale.1,
                                            })
                                        }
                                        Ok(Some(pairs)) => {
                                            collision_stop(&link, &gate, "jog_l", pairs);
                                            sh.stream = None;
                                            continue 'housekeeping;
                                        }
                                        Err(e) => {
                                            // Stop without a collision verdict:
                                            // this is a model failure, not a
                                            // predicted contact.
                                            log::error!("jog_l gate check failed: {}", e.cause);
                                            link.send(RtCommand::SetMode(Mode::Idle));
                                            sh.stream = None;
                                            continue 'housekeeping;
                                        }
                                    }
                                }
                                Err(e) => {
                                    // Hold in place rather than integrate on a
                                    // failed solve; the stream watchdog still
                                    // needs feeding.
                                    log::warn!("jog_l step failed ({e}); holding");
                                    stream_input.lock().unwrap().send(&StreamSetpoint {
                                        q: state.q,
                                        speed: a.scale.0,
                                        accel: a.scale.1,
                                    });
                                }
                            }
                        }
                    }
                    _ => {}
                }
            }
            if let Some(req) = &mut sh.enable {
                // `enable_seq` counts every Enable the core PROCESSED,
                // granted or refused, so a reading past our baseline
                // makes the `state` in the same snapshot this request's
                // answer rather than whatever an earlier one left behind.
                let answered = req.sent_at_seq.is_some_and(|s| snap.enable_seq > s);
                if answered && snap.state == ArmState::Enabled {
                    sh.enable = None;
                    sh.enable_outcome = Some(Ok(()));
                } else if now >= req.deadline {
                    log::warn!("enable retry window expired; controller still DISABLED");
                    sh.enable = None;
                    sh.enable_outcome = Some(Err(make_error(
                        ErrorCode::SysControllerDisabled,
                        UNATTRIBUTED,
                        &[(
                            "detail",
                            "The RT core refused to enable: the e-stop line is engaged \
                             or a hard error is latched.",
                        )],
                    )));
                } else if req.last_sent.is_none_or(|t| {
                    now.duration_since(t) >= housekeeping_period(dt) * ENABLE_RETRY_TICKS
                }) {
                    req.sent_at_seq = Some(snap.enable_seq);
                    req.last_sent = Some(now);
                    link.send(RtCommand::Enable);
                }
            }
            if let Some(req) = &sh.flashing {
                // An exit lands wherever the RT settles: a still-latched
                // hard error re-drives a FLASHING exit to ACTIVE_ERROR, and
                // that is a successful exit, not a timeout.
                let arrived = snap.mode == req.want
                    || (req.want == Mode::Idle && snap.mode != Mode::Flashing);
                if arrived {
                    sh.flashing = None;
                    sh.flashing_outcome = Some(Ok(()));
                } else if now >= req.deadline {
                    let detail = match req.want {
                        Mode::Flashing => format!(
                            "the controller mode stayed {:?}: FLASHING is reachable only \
                             from IDLE and ACTIVE_ERROR",
                            snap.mode
                        ),
                        _ => format!(
                            "the controller mode stayed {:?} instead of returning to IDLE",
                            snap.mode
                        ),
                    };
                    log::warn!("FLASHING request expired: {detail}");
                    sh.flashing = None;
                    sh.flashing_outcome = Some(Err(make_error(
                        ErrorCode::CommValidationError,
                        UNATTRIBUTED,
                        &[("detail", &detail)],
                    )));
                }
            }
        }
        std::thread::sleep(housekeeping_period(dt));
    }
}

/// One cartesian-jog integration step: resolve the twist into world
/// axes, solve joint velocities through the damped jacobian, integrate
/// the joint target and clamp it inside the soft window. Returns the
/// integrated target and the joint velocity it moved at — what the
/// collision gate projects its lookahead with.
pub(crate) fn step_cart_jog(
    kin: &mut crate::kin::CartKin,
    state: &mut CartJogState,
    dt_s: f64,
) -> Result<([f64; MAX_JOINTS], [f64; MAX_JOINTS]), String> {
    let mut v = state.twist;
    if state.frame == par6_proto::Frame::Trf {
        let pose = kin.fk(&state.q)?;
        let rot = |vec: [f64; 3]| {
            [
                pose[0] * vec[0] + pose[1] * vec[1] + pose[2] * vec[2],
                pose[4] * vec[0] + pose[5] * vec[1] + pose[6] * vec[2],
                pose[8] * vec[0] + pose[9] * vec[1] + pose[10] * vec[2],
            ]
        };
        let lin = rot([v[0], v[1], v[2]]);
        let ang = rot([v[3], v[4], v[5]]);
        v = [lin[0], lin[1], lin[2], ang[0], ang[1], ang[2]];
    }
    let qd = kin.twist_to_qd(&state.q, &v)?;
    for (j, q) in state.q.iter_mut().enumerate() {
        *q = (*q + qd[j] * dt_s).clamp(state.soft_min[j], state.soft_max[j]);
    }
    Ok((state.q, qd))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The jog watchdog deadline is what STOPS a jog, and it is computed
    /// from a wire float with `Duration`/`Instant` arithmetic that panics
    /// near f64's range — in a build compiled `panic = "abort"`, under
    /// the shared-state lock. The codec refuses these values; this is the
    /// second wall, so no reachable `duration` can either abort the
    /// daemon or arm a watchdog that outlives the shift.
    #[test]
    fn no_wire_duration_can_produce_an_unusable_jog_deadline() {
        let ceiling = Duration::from_secs_f64(MAX_JOG_DURATION_S);
        for hostile in [1e30, 1e19, f64::MAX, f64::INFINITY, f64::NAN, -1.0] {
            // `jog_deadline` reads its own `Instant::now()`, so the
            // ceiling has to be measured from an instant at or after
            // that read — bracketing the call is what makes the bound
            // exact rather than off by the clock tick between them.
            let before = Instant::now();
            let deadline = jog_deadline(hostile);
            let after = Instant::now();
            assert!(
                deadline <= after + ceiling,
                "duration {hostile} armed the watchdog past the ceiling"
            );
            assert!(
                deadline >= before,
                "duration {hostile} armed it in the past"
            );
        }
        // A duration a UI actually streams is honoured, not clamped away.
        let before = Instant::now();
        let deadline = jog_deadline(0.1);
        assert!(deadline >= before + Duration::from_millis(100));
        assert!(deadline < before + Duration::from_millis(200));
    }
}
