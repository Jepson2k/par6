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

use par6_bus::sim::SimBus;
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

/// Servo streams self-terminate after this much client silence (the RT
/// stream watchdog is fed by housekeeping keep-alives until then).
pub(crate) const SERVO_GRACE: Duration = Duration::from_millis(250);
/// How long the enable retry keeps trying after `reset` (covers the RT
/// clear-sequence settle window with margin, even on a loaded host).
const ENABLE_RETRY_WINDOW: Duration = Duration::from_secs(5);
/// Spacing between enable retries (a few RT ticks at any supported
/// rate, so retries never saturate the one-command-per-tick budget).
const ENABLE_RETRY_PERIOD: Duration = Duration::from_millis(60);
/// Housekeeping loop period.
pub(crate) const HOUSEKEEPING_PERIOD: Duration = Duration::from_millis(4);
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

/// Velocity-scaled streaming lookahead horizon \[s\]: a jog is refused or
/// stopped when the configuration this far ahead AT THE COMMANDED
/// VELOCITY would collide, so faster jogs stop further from contact.
/// parol6 runs the same horizon (`COLLISION_JOG_LOOKAHEAD_S`) on its
/// server-side jog integrator; par6's integrator ramps on the RT thread,
/// so the projection here uses the commanded target velocity — an upper
/// bound on the ramping integrator's, which errs on the stopping side.
pub(crate) const STREAM_LOOKAHEAD_S: f64 = 0.15;
/// Escape-depth tolerance \[m\]: a min-distance drop smaller than this
/// counts as "no deeper" (absorbs signed-distance jitter between two
/// nearby configurations; parol6's escape tolerance). Used by the
/// planner's per-sample check against the START depth, matching
/// parol6's `guard_joint_path`.
pub(crate) const ESCAPE_TOL_M: f64 = 1e-4;
/// The streaming gate's escape-depth tolerance \[m\]. parol6 applies
/// [`ESCAPE_TOL_M`] per integrator step (~10 ms of travel); par6's gate
/// compares across the whole [`STREAM_LOOKAHEAD_S`] projection in one
/// step, so the same per-step slack scales with the horizon — otherwise
/// an escaping arc whose link transiently dips ~1 mm deeper inside the
/// window is refused, and the arm is trapped in the keep-out the rule
/// exists to let it leave. Sustained grinding is still caught: the
/// housekeeping re-check advances `current` every period, so a deepening
/// beyond this slack cannot accumulate unrefused.
const STREAM_ESCAPE_TOL_M: f64 = 1.5e-3;

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
    soft_min: [f64; MAX_JOINTS],
    soft_max: [f64; MAX_JOINTS],
    /// The pairs the last refused or stopped stream would have collided
    /// in — the streaming half of the STATUS `collision_active` fields.
    latch: CollisionState,
}

impl StreamGate {
    pub(crate) fn new(
        collision: par6_kin::Collision,
        jog_limits: &par6_motion::MotionLimits,
    ) -> Self {
        Self {
            collision,
            shape_names: ShapeNames::default(),
            jog_vel: jog_limits.velocity,
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

    /// Colliding pairs at `q`, in reporting names.
    fn offending(&mut self, q: &[f64; MAX_JOINTS]) -> Result<Vec<(String, String)>, WireError> {
        let mut nq = [0.0; par6_kin::NQ];
        nq.copy_from_slice(&q[..par6_kin::NQ]);
        let names = &self.shape_names;
        let report = self.collision.check(&nq, false).map_err(gate_error)?;
        Ok(names.render(&report))
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
            return Ok((!tgt.is_empty()).then_some(tgt));
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
        if world_pair
            && self.world_distance(target)? < self.world_distance(current)? - STREAM_ESCAPE_TOL_M
        {
            return Ok(Some(if tgt.is_empty() { cur } else { tgt }));
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
            la[j] = (la[j] + pct * self.jog_vel[j] * STREAM_LOOKAHEAD_S)
                .clamp(self.soft_min[j], self.soft_max[j]);
        }
        la
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
    servo_target: Option<[f64; MAX_JOINTS]>,
    /// The live `jog_j` command: per-joint signed speed fraction, all
    /// zero once the button released. What housekeeping's periodic
    /// collision re-check projects the lookahead from.
    jog: [f64; MAX_JOINTS],
    /// `scene_epoch` of the collision world a held SERVO target was last
    /// checked against. A held target cannot move, so it only needs
    /// re-testing when the WORLD does — this is what housekeeping's
    /// re-check keys on, so the steady state costs no collision queries
    /// (and a pathological Plane keep-out cannot starve the keep-alive).
    world_epoch: u64,
    cart: Option<CartJogState>,
    /// The stream's `(speed, accel)` fractions, carried so housekeeping's
    /// keep-alive refeeds the setpoint the client asked for rather than
    /// silently restoring full-speed limits between datagrams.
    scale: (f64, f64),
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
    cart: CartStream,
}

impl RtBridge {
    pub(crate) fn new(
        link: CoreLink,
        stream_input: Arc<Mutex<StreamInput>>,
        shared: Arc<Mutex<SharedState>>,
        flush: FlushMarker,
        bundle: Arc<ConfigBundle>,
        sim: bool,
        cart: CartStream,
    ) -> Self {
        Self {
            link,
            stream_input,
            shared,
            flush,
            bundle,
            sim,
            cart,
        }
    }

    /// Mode dance into a stream mode. The RT transition table only
    /// allows working-mode changes through IDLE, and `SetMode` to the
    /// current mode is a no-op, so the pair is always safe to queue.
    fn enter_stream_mode(&self, target: Mode) {
        self.link.send(RtCommand::SetMode(Mode::Idle));
        self.link.send(RtCommand::SetMode(target));
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
                    kind: StreamKind::Jog,
                    deadline: jog_deadline(p.duration),
                    servo_target: None,
                    jog: speeds,
                    world_epoch: 0,
                    cart: None,
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
                // Servo targets are explicit configurations, so the gate
                // checks the target itself — each datagram is its own
                // admission check, which is the streaming cadence parol6
                // gates at.
                let world_epoch = {
                    let q = self.cart.snapshots.latest().q;
                    let mut gate = self.cart.gate.lock().unwrap();
                    if let Some(pairs) = gate.blocked(&q, &target)? {
                        return Err(gate.refuse(pairs));
                    }
                    gate.epoch()
                };
                if !matches!(
                    sh.stream,
                    Some(ActiveStream {
                        kind: StreamKind::Servo,
                        ..
                    })
                ) {
                    self.enter_stream_mode(Mode::Stream);
                }
                let scale = (p.speed.unwrap_or(1.0), p.accel.unwrap_or(1.0));
                self.stream_input.lock().unwrap().send(&StreamSetpoint {
                    q: target,
                    speed: scale.0,
                    accel: scale.1,
                });
                sh.stream = Some(ActiveStream {
                    kind: StreamKind::Servo,
                    deadline: Instant::now() + SERVO_GRACE,
                    servo_target: Some(target),
                    jog: [0.0; MAX_JOINTS],
                    world_epoch,
                    cart: None,
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
                let world_epoch = {
                    let q = self.cart.snapshots.latest().q;
                    let mut gate = self.cart.gate.lock().unwrap();
                    if let Some(pairs) = gate.blocked(&q, &target)? {
                        return Err(gate.refuse(pairs));
                    }
                    gate.epoch()
                };
                if !matches!(
                    sh.stream,
                    Some(ActiveStream {
                        kind: StreamKind::Servo,
                        ..
                    })
                ) {
                    self.enter_stream_mode(Mode::Stream);
                }
                let scale = (speed.unwrap_or(1.0), accel.unwrap_or(1.0));
                self.stream_input.lock().unwrap().send(&StreamSetpoint {
                    q: target,
                    speed: scale.0,
                    accel: scale.1,
                });
                sh.stream = Some(ActiveStream {
                    kind: StreamKind::Servo,
                    deadline: Instant::now() + SERVO_GRACE,
                    servo_target: Some(target),
                    jog: [0.0; MAX_JOINTS],
                    world_epoch,
                    cart: None,
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
                if let Ok((la, _)) =
                    step_cart_jog(&mut self.cart.kin, &mut probe, STREAM_LOOKAHEAD_S)
                {
                    let mut gate = self.cart.gate.lock().unwrap();
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
                    kind: StreamKind::CartJog,
                    deadline: jog_deadline(p.duration),
                    servo_target: None,
                    jog: [0.0; MAX_JOINTS],
                    world_epoch: 0,
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

    fn tool_stop(&mut self) {
        self.link.send(RtCommand::GripperStop);
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
        let sim = SimBus::new();
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
        let hw = SocketCanBus::open(&cfg).map_err(|e| {
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
pub(crate) fn housekeeping_loop(
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
        link.send(RtCommand::JogRelease);
        link.send(RtCommand::SetMode(Mode::Idle));
    };
    while !shutdown.load(Ordering::SeqCst) {
        let now = Instant::now();
        let snap = snapshots.latest();
        {
            let mut sh = shared.lock().unwrap();
            match &mut sh.stream {
                Some(a) if now >= a.deadline => {
                    match a.kind {
                        // Released rather than idled: `JogRelease` zeroes
                        // the engine's target but not its velocity, and
                        // the RT only ticks the engine in JOG, so cutting
                        // to IDLE here would stop the arm dead from full
                        // jog speed. The RT leaves JOG itself once the
                        // ramp reaches rest.
                        StreamKind::Jog => {
                            log::debug!("jog duration elapsed; releasing");
                            link.send(RtCommand::JogRelease);
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
                Some(a) if a.kind == StreamKind::Servo => {
                    // A held servo target was admitted against the world
                    // of its datagram, and a held target cannot move —
                    // so it is re-tested exactly when the WORLD changes
                    // (the analogue of the planner's in-flight
                    // revalidation), never per period: the steady state
                    // costs no collision queries, which is what keeps a
                    // pathological Plane keep-out from starving the
                    // keep-alive below past the RT stream watchdog.
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
                                    continue;
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
                                    continue;
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
                        match step_cart_jog(&mut kin, state, HOUSEKEEPING_PERIOD.as_secs_f64()) {
                            Ok((target, qd)) => {
                                // The velocity-scaled horizon, projected
                                // past the step just integrated.
                                let mut la = target;
                                for (j, v) in la.iter_mut().enumerate() {
                                    *v = (*v + qd[j] * STREAM_LOOKAHEAD_S)
                                        .clamp(state.soft_min[j], state.soft_max[j]);
                                }
                                let verdict = gate.lock().unwrap().blocked(&before, &la);
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
                                        continue;
                                    }
                                    Err(e) => {
                                        // Stop without a collision verdict:
                                        // this is a model failure, not a
                                        // predicted contact.
                                        log::error!("jog_l gate check failed: {}", e.cause);
                                        link.send(RtCommand::SetMode(Mode::Idle));
                                        sh.stream = None;
                                        continue;
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
                } else if req
                    .last_sent
                    .is_none_or(|t| now.duration_since(t) >= ENABLE_RETRY_PERIOD)
                {
                    req.sent_at_seq = Some(snap.enable_seq);
                    req.last_sent = Some(now);
                    link.send(RtCommand::Enable);
                }
            }
            if let Some(req) = &sh.flashing {
                if snap.mode == req.want {
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
        std::thread::sleep(HOUSEKEEPING_PERIOD);
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
