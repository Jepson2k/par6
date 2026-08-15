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

use par6_bus::RuntimeBus;
use par6_config::ConfigBundle;
use par6_proto::command::MAX_JOG_DURATION_S;
use par6_proto::{make_error, Command, ErrorCode, WireError, NUM_JOINTS, UNATTRIBUTED};
use par6_rt::{
    ArmState, FlushMarker, Mode, RtCommand, RtCore, SnapshotReader, StateSnapshot, StreamInput,
    MAX_JOINTS,
};
use par6_server::RtCommands;
#[cfg(feature = "ffi")]
use par6_server::{CollisionState, ShapeLayer};

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
const HOUSEKEEPING_PERIOD: Duration = Duration::from_millis(4);

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

/// Full-scale `jog_l` linear TCP speed \[m/s\] (a `velocities` fraction
/// of ±1 maps to this; conservative — the RT stream limiter still owns
/// the joint-space envelope).
#[cfg(feature = "ffi")]
const JOG_L_LINEAR_MAX_M_S: f64 = 0.08;
/// Full-scale `jog_l` angular TCP speed \[rad/s\].
#[cfg(feature = "ffi")]
const JOG_L_ANGULAR_MAX_RAD_S: f64 = 0.6;

/// Velocity-scaled streaming lookahead horizon \[s\]: a jog is refused or
/// stopped when the configuration this far ahead AT THE COMMANDED
/// VELOCITY would collide, so faster jogs stop further from contact.
/// parol6 runs the same horizon (`COLLISION_JOG_LOOKAHEAD_S`) on its
/// server-side jog integrator; par6's integrator ramps on the RT thread,
/// so the projection here uses the commanded target velocity — an upper
/// bound on the ramping integrator's, which errs on the stopping side.
#[cfg(feature = "ffi")]
const STREAM_LOOKAHEAD_S: f64 = 0.15;
/// Escape-depth tolerance \[m\]: a min-distance drop smaller than this
/// counts as "no deeper" (absorbs signed-distance jitter between two
/// nearby configurations; parol6's escape tolerance). Used by the
/// planner's per-sample check against the START depth, matching
/// parol6's `guard_joint_path`.
#[cfg(feature = "ffi")]
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
#[cfg(feature = "ffi")]
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
#[cfg(feature = "ffi")]
pub(crate) struct StreamGate {
    collision: par6_kin::Collision,
    /// Applied world-shape names per layer (reporting vocabulary).
    layer_names: [Vec<String>; 2],
    world_names: Vec<String>,
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

#[cfg(feature = "ffi")]
impl StreamGate {
    pub(crate) fn new(
        collision: par6_kin::Collision,
        jog_limits: &par6_motion::MotionLimits,
    ) -> Self {
        Self {
            collision,
            layer_names: [Vec::new(), Vec::new()],
            world_names: Vec::new(),
            jog_vel: jog_limits.velocity,
            soft_min: jog_limits.soft_min,
            soft_max: jog_limits.soft_max,
            latch: CollisionState::default(),
        }
    }

    /// Mirror one layer of the planner-accepted world. The conversion is
    /// the identical `Shape::from_proto` path the planner ran, so on a
    /// set the server hands over it cannot disagree.
    fn set_layer(
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
        let (slot, kin_layer) = match layer {
            ShapeLayer::Installation => (0, par6_kin::Layer::Installation),
            ShapeLayer::Program => (1, par6_kin::Layer::Program),
        };
        self.collision
            .set_layer(kin_layer, &converted)
            .map_err(|e| {
                make_error(
                    ErrorCode::CommValidationError,
                    UNATTRIBUTED,
                    &[("detail", &format!("stream gate collision world: {e}"))],
                )
            })?;
        self.layer_names[slot] = converted
            .iter()
            .filter(|s| s.collision)
            .map(|s| s.name.clone())
            .collect();
        self.world_names = self.layer_names.concat();
        Ok(())
    }

    /// The reporting name of one colliding geometry: world shapes keep
    /// the name they were applied with, robot geometry drops the model's
    /// per-link geometry suffix (`upper_arm_0` → `upper_arm`).
    fn display(&self, geom: &str) -> String {
        if self.world_names.iter().any(|n| n == geom) {
            return geom.to_owned();
        }
        trim_geom(geom).to_owned()
    }

    /// Colliding pairs at `q`, in reporting names.
    fn offending(&mut self, q: &[f64; MAX_JOINTS]) -> Result<Vec<(String, String)>, WireError> {
        let mut nq = [0.0; par6_kin::NQ];
        nq.copy_from_slice(&q[..par6_kin::NQ]);
        let report = self.collision.check(&nq, false).map_err(gate_error)?;
        let raw: Vec<(String, String)> = report
            .pairs()
            .map(|(a, b)| (a.to_owned(), b.to_owned()))
            .collect();
        Ok(raw
            .into_iter()
            .map(|(a, b)| (self.display(&a), self.display(&b)))
            .collect())
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
    fn blocked(
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
            .any(|p| self.world_names.contains(&p.0) || self.world_names.contains(&p.1));
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
    fn jog_lookahead(
        &self,
        q: &[f64; MAX_JOINTS],
        joint: usize,
        signed_pct: f64,
    ) -> [f64; MAX_JOINTS] {
        let mut la = *q;
        la[joint] = (la[joint] + signed_pct * self.jog_vel[joint] * STREAM_LOOKAHEAD_S)
            .clamp(self.soft_min[joint], self.soft_max[joint]);
        la
    }

    /// Latch `pairs` as the streaming collision verdict and build the
    /// refusal the client reads. One checked configuration, so the error
    /// template's path slots read `0` of `1`.
    fn refuse(&mut self, pairs: Vec<(String, String)>) -> WireError {
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

/// Robot geometry names carry the model's per-link geometry index
/// (`upper_arm_0`); reports name URDF links.
#[cfg(feature = "ffi")]
fn trim_geom(geom: &str) -> &str {
    match geom.rsplit_once('_') {
        Some((link, idx)) if !idx.is_empty() && idx.bytes().all(|b| b.is_ascii_digit()) => link,
        _ => geom,
    }
}

/// A collision-world query the shim refused (a broken model, never a
/// well-formed configuration).
#[cfg(feature = "ffi")]
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
    #[cfg(feature = "ffi")]
    CartJog,
}

/// Live state of a cartesian jog, advanced by housekeeping each period.
#[cfg(feature = "ffi")]
struct CartJogState {
    /// Commanded TCP twist `[vx vy vz (m/s), wx wy wz (rad/s)]` in the
    /// commanded frame's axes.
    twist: [f64; 6],
    frame: par6_proto::Frame,
    /// Integrated joint target \[rad\] (the stream setpoint source).
    q: [f64; MAX_JOINTS],
    soft_min: [f64; MAX_JOINTS],
    soft_max: [f64; MAX_JOINTS],
}

struct ActiveStream {
    kind: StreamKind,
    deadline: Instant,
    servo_target: Option<[f64; MAX_JOINTS]>,
    /// The live `jog_j` command `(joint, signed speed fraction)`; `None`
    /// once the button released. What housekeeping's periodic collision
    /// re-check projects the lookahead from (feature `ffi` — a build
    /// without kinematics has no gate to read it).
    #[cfg_attr(not(feature = "ffi"), allow(dead_code))]
    jog: Option<(usize, f64)>,
    /// `scene_epoch` of the collision world a held SERVO target was last
    /// checked against. A held target cannot move, so it only needs
    /// re-testing when the WORLD does — this is what housekeeping's
    /// re-check keys on, so the steady state costs no collision queries
    /// (and a pathological Plane keep-out cannot starve the keep-alive).
    #[cfg_attr(not(feature = "ffi"), allow(dead_code))]
    world_epoch: u64,
    #[cfg(feature = "ffi")]
    cart: Option<CartJogState>,
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

/// State shared between the bridge (server task) and housekeeping.
#[derive(Default)]
pub(crate) struct SharedState {
    stream: Option<ActiveStream>,
    enable: Option<EnableRequest>,
    /// Resolved enable outcome, waiting to be collected by the server.
    enable_outcome: Option<Result<(), WireError>>,
}

/// The bridge's kinematics kit (feature `ffi`): its own model instance,
/// the snapshot reader that seeds IK from the measured pose, and the
/// streaming collision gate it shares with housekeeping.
#[cfg(feature = "ffi")]
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
    #[cfg(feature = "ffi")]
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
        #[cfg(feature = "ffi")] cart: CartStream,
    ) -> Self {
        Self {
            link,
            stream_input,
            shared,
            flush,
            bundle,
            sim,
            #[cfg(feature = "ffi")]
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
                // Single-axis by contract: the server refuses a jog with
                // more than one non-zero speed, because the RT jog engine
                // ramps one joint at a time.
                let (joint, pct) = p
                    .speeds
                    .iter()
                    .copied()
                    .enumerate()
                    .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
                    .expect("NUM_JOINTS > 0");
                let mut sh = self.shared.lock().unwrap();
                // Admission gate: where this jog will be one lookahead
                // horizon ahead must not collide (or, from inside a
                // keep-out, must not deepen it). The commanded velocity
                // bounds the RT integrator's ramp from above, so a jog
                // this projection clears cannot outrun it.
                #[cfg(feature = "ffi")]
                if pct != 0.0 {
                    let q = self.cart.snapshots.latest().q;
                    let mut gate = self.cart.gate.lock().unwrap();
                    let la = gate.jog_lookahead(&q, joint, pct);
                    if let Some(pairs) = gate.blocked(&q, &la)? {
                        return Err(gate.refuse(pairs));
                    }
                }
                if !matches!(
                    sh.stream,
                    Some(ActiveStream {
                        kind: StreamKind::Jog,
                        ..
                    })
                ) {
                    self.enter_stream_mode(Mode::Jog);
                }
                if pct == 0.0 {
                    self.link.send(RtCommand::JogRelease);
                } else {
                    self.link.send(RtCommand::Jog {
                        joint: joint as u8,
                        signed_pct: pct,
                    });
                }
                sh.stream = Some(ActiveStream {
                    kind: StreamKind::Jog,
                    deadline: jog_deadline(p.duration),
                    servo_target: None,
                    jog: (pct != 0.0).then_some((joint, pct)),
                    world_epoch: 0,
                    #[cfg(feature = "ffi")]
                    cart: None,
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
                #[cfg(feature = "ffi")]
                let world_epoch = {
                    let q = self.cart.snapshots.latest().q;
                    let mut gate = self.cart.gate.lock().unwrap();
                    if let Some(pairs) = gate.blocked(&q, &target)? {
                        return Err(gate.refuse(pairs));
                    }
                    gate.epoch()
                };
                #[cfg(not(feature = "ffi"))]
                let world_epoch = 0;
                if !matches!(
                    sh.stream,
                    Some(ActiveStream {
                        kind: StreamKind::Servo,
                        ..
                    })
                ) {
                    self.enter_stream_mode(Mode::Stream);
                }
                self.stream_input.lock().unwrap().send(&target);
                sh.stream = Some(ActiveStream {
                    kind: StreamKind::Servo,
                    deadline: Instant::now() + SERVO_GRACE,
                    servo_target: Some(target),
                    jog: None,
                    world_epoch,
                    #[cfg(feature = "ffi")]
                    cart: None,
                });
            }
            // Cartesian position streams: seeded IK, then the exact
            // servo_j path. An unreachable target drops the datagram
            // (fire-and-forget has no reply channel) — the arm must not
            // move on a pose the solver cannot reach.
            #[cfg(feature = "ffi")]
            Command::ServoJPose(par6_proto::command::ServoJPose { pose, .. })
            | Command::ServoL(par6_proto::command::ServoL { pose, .. }) => {
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
                self.stream_input.lock().unwrap().send(&target);
                sh.stream = Some(ActiveStream {
                    kind: StreamKind::Servo,
                    deadline: Instant::now() + SERVO_GRACE,
                    servo_target: Some(target),
                    jog: None,
                    world_epoch,
                    cart: None,
                });
            }
            // Cartesian velocity jog: housekeeping steps the twist
            // through the jacobian each period until the watchdog
            // duration elapses.
            #[cfg(feature = "ffi")]
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
                for (i, (out, frac)) in twist.iter_mut().zip(p.velocities.iter()).enumerate() {
                    let full = if i < 3 {
                        JOG_L_LINEAR_MAX_M_S
                    } else {
                        JOG_L_ANGULAR_MAX_RAD_S
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
                    jog: None,
                    world_epoch: 0,
                    cart: Some(CartJogState {
                        twist,
                        frame: p.frame,
                        q,
                        soft_min: self.cart.soft_min,
                        soft_max: self.cart.soft_max,
                    }),
                });
            }
            #[cfg(not(feature = "ffi"))]
            Command::JogL(_) | Command::ServoJPose(_) | Command::ServoL(_) => {
                // The server refuses cartesian commands when it is told
                // the runtime has no kinematics; this is defense in depth.
                log::error!(
                    "{:?} reached a par6d built without feature `ffi`; dropped",
                    cmd.tag()
                );
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
        // The sim backend has no digital output pins; the server mirrors
        // outputs into STATUS. Physical GPIO lands with hardware mode.
        log::info!("write_io port={port} value={value} (no physical outputs in sim)");
    }

    fn set_simulator(&mut self, on: bool) -> Result<(), WireError> {
        if on == self.sim {
            return Ok(());
        }
        Err(make_error(
            ErrorCode::MotnSetupFailed,
            UNATTRIBUTED,
            &[(
                "detail",
                "live backend switching is not wired yet; restart par6d with/without --sim",
            )],
        ))
    }

    fn connect_hardware(&mut self, port: &str) -> Result<(), WireError> {
        Err(make_error(
            ErrorCode::MotnSetupFailed,
            UNATTRIBUTED,
            &[(
                "detail",
                &format!(
                    "cannot switch to hardware bus '{port}' while running; \
                     restart par6d without --sim to open the configured interface"
                ),
            )],
        ))
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

    #[cfg(feature = "ffi")]
    fn set_shapes(
        &mut self,
        layer: ShapeLayer,
        shapes: &[par6_proto::Shape],
    ) -> Result<(), WireError> {
        self.cart.gate.lock().unwrap().set_layer(layer, shapes)
    }

    #[cfg(feature = "ffi")]
    fn collision(&mut self) -> Option<CollisionState> {
        Some(self.cart.gate.lock().unwrap().latch.clone())
    }

    #[cfg(feature = "ffi")]
    fn clear_collision(&mut self) {
        self.cart.gate.lock().unwrap().latch = CollisionState::default();
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
    #[cfg(feature = "ffi")] mut kin: crate::kin::CartKin,
    #[cfg(feature = "ffi")] gate: Arc<Mutex<StreamGate>>,
) {
    // Stops the live stream because its next step is collision-blocked:
    // latch the verdict for STATUS and put the RT back to IDLE. The
    // abrupt stop is deliberate — the alternative is driving on toward
    // contact (parol6 halts its joint jog on the same prediction).
    #[cfg(feature = "ffi")]
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
                        StreamKind::Jog => {
                            log::debug!("jog duration elapsed; releasing");
                            link.send(RtCommand::JogRelease);
                        }
                        StreamKind::Servo => log::debug!("servo stream went silent; stopping"),
                        #[cfg(feature = "ffi")]
                        StreamKind::CartJog => log::debug!("jog_l duration elapsed; stopping"),
                    }
                    link.send(RtCommand::SetMode(Mode::Idle));
                    sh.stream = None;
                }
                // The moving-jog re-check: the admission gate saw the
                // configuration the jog STARTED at, and the arm has
                // moved since. Every period the lookahead is projected
                // from the measured pose and re-tested — against the
                // world as it is NOW, so a keep-out dropped onto a
                // running jog stops it too.
                #[cfg(feature = "ffi")]
                Some(a) if a.kind == StreamKind::Jog && a.jog.is_some() => {
                    let (joint, pct) = a.jog.expect("guarded");
                    let mut g = gate.lock().unwrap();
                    let la = g.jog_lookahead(&snap.q, joint, pct);
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
                    #[cfg(feature = "ffi")]
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
                        stream_input.lock().unwrap().send(&t);
                    }
                }
                #[cfg(feature = "ffi")]
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
                                    Ok(None) => stream_input.lock().unwrap().send(&target),
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
                                stream_input.lock().unwrap().send(&state.q);
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
        }
        std::thread::sleep(HOUSEKEEPING_PERIOD);
    }
}

/// One cartesian-jog integration step: resolve the twist into world
/// axes, solve joint velocities through the damped jacobian, integrate
/// the joint target and clamp it inside the soft window. Returns the
/// integrated target and the joint velocity it moved at — what the
/// collision gate projects its lookahead with.
#[cfg(feature = "ffi")]
fn step_cart_jog(
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
