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
use par6_proto::{make_error, Command, ErrorCode, WireError, NUM_JOINTS, UNATTRIBUTED};
use par6_rt::{
    ArmState, FlushMarker, Mode, RtCommand, RtCore, SnapshotReader, StateSnapshot, StreamInput,
    MAX_JOINTS,
};
use par6_server::RtCommands;

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
/// Full-scale `jog_l` linear TCP speed \[m/s\] (a `velocities` fraction
/// of ±1 maps to this; conservative — the RT stream limiter still owns
/// the joint-space envelope).
#[cfg(feature = "ffi")]
const JOG_L_LINEAR_MAX_M_S: f64 = 0.08;
/// Full-scale `jog_l` angular TCP speed \[rad/s\].
#[cfg(feature = "ffi")]
const JOG_L_ANGULAR_MAX_RAD_S: f64 = 0.6;

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

/// The bridge's kinematics kit (feature `ffi`): its own model instance
/// plus the snapshot reader that seeds IK from the measured pose.
#[cfg(feature = "ffi")]
pub(crate) struct CartStream {
    pub(crate) kin: crate::kin::CartKin,
    pub(crate) snapshots: SnapshotReader<StateSnapshot>,
    pub(crate) soft_min: [f64; MAX_JOINTS],
    pub(crate) soft_max: [f64; MAX_JOINTS],
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
    fn stream(&mut self, cmd: &Command) {
        match cmd {
            Command::JogJ(p) => {
                // Single-axis by contract: the server refuses a jog with
                // more than one non-zero speed, because the RT jog engine
                // ramps one joint at a time (spec/RT.md, Jog).
                let (joint, pct) = p
                    .speeds
                    .iter()
                    .copied()
                    .enumerate()
                    .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
                    .expect("NUM_JOINTS > 0");
                let mut sh = self.shared.lock().unwrap();
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
                    deadline: Instant::now() + Duration::from_secs_f64(p.duration),
                    servo_target: None,
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
                        return;
                    }
                    crate::kin::IkResult::Failed(e) => {
                        log::warn!("{:?}: IK failed ({e}); dropped", cmd.tag());
                        return;
                    }
                };
                for (j, v) in target.iter_mut().enumerate() {
                    *v = v.clamp(self.cart.soft_min[j], self.cart.soft_max[j]);
                }
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
                    _ => {
                        self.enter_stream_mode(Mode::Stream);
                        self.cart.snapshots.latest().q
                    }
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
                sh.stream = Some(ActiveStream {
                    kind: StreamKind::CartJog,
                    deadline: Instant::now() + Duration::from_secs_f64(p.duration),
                    servo_target: None,
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
) {
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
                Some(a) if a.kind == StreamKind::Servo => {
                    // Keep the RT stream watchdog fed between client
                    // datagrams (its timeout is shorter than the grace).
                    if let Some(t) = a.servo_target {
                        stream_input.lock().unwrap().send(&t);
                    }
                }
                #[cfg(feature = "ffi")]
                Some(a) if a.kind == StreamKind::CartJog => {
                    if let Some(state) = &mut a.cart {
                        match step_cart_jog(&mut kin, state, HOUSEKEEPING_PERIOD.as_secs_f64()) {
                            Ok(target) => stream_input.lock().unwrap().send(&target),
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
/// the joint target and clamp it inside the soft window.
#[cfg(feature = "ffi")]
fn step_cart_jog(
    kin: &mut crate::kin::CartKin,
    state: &mut CartJogState,
    dt_s: f64,
) -> Result<[f64; MAX_JOINTS], String> {
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
    Ok(state.q)
}
