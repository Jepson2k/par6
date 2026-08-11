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
use par6_bus::DriverBus;
use par6_config::ConfigBundle;
use par6_proto::{make_error, Command, ErrorCode, WireError, NUM_JOINTS, UNATTRIBUTED};
use par6_rt::{
    ArmState, Mode, RtCommand, RtCore, SnapshotReader, StateSnapshot, StreamInput, MAX_JOINTS,
};
use par6_server::RtCommands;

/// A closure applied to the core on the RT thread, between `run()`
/// sessions.
pub(crate) type CoreOp = Box<dyn FnOnce(&mut RtCore<SimBus>) + Send>;

/// Servo streams self-terminate after this much client silence (the RT
/// stream watchdog is fed by housekeeping keep-alives until then).
const SERVO_GRACE: Duration = Duration::from_millis(250);
/// How long the enable retry keeps trying after `reset` (covers the RT
/// clear-sequence settle window with margin, even on a loaded host).
const ENABLE_RETRY_WINDOW: Duration = Duration::from_secs(5);
/// Spacing between enable retries (a few RT ticks at any supported
/// rate, so retries never saturate the one-command-per-tick budget).
const ENABLE_RETRY_PERIOD: Duration = Duration::from_millis(60);
/// Extra RT ticks on top of the clear-settle countdown before an
/// ENABLED snapshot is trusted as the OUTCOME of this enable request —
/// covers commands already queued ahead of ours (an e-stop still in
/// flight must not read as "already enabled").
const ENABLE_TRUST_SLACK_TICKS: u64 = 16;
/// Housekeeping loop period.
const HOUSEKEEPING_PERIOD: Duration = Duration::from_millis(4);

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
}

struct ActiveStream {
    kind: StreamKind,
    deadline: Instant,
    servo_target: Option<[f64; MAX_JOINTS]>,
}

/// State shared between the bridge (server task) and housekeeping.
#[derive(Default)]
pub(crate) struct SharedState {
    stream: Option<ActiveStream>,
    enable_deadline: Option<Instant>,
}

/// The `RtCommands` implementation `par6d` hands to the server.
pub(crate) struct RtBridge {
    link: CoreLink,
    stream_input: Arc<Mutex<StreamInput>>,
    shared: Arc<Mutex<SharedState>>,
    bundle: Arc<ConfigBundle>,
    sim: bool,
}

impl RtBridge {
    pub(crate) fn new(
        link: CoreLink,
        stream_input: Arc<Mutex<StreamInput>>,
        shared: Arc<Mutex<SharedState>>,
        bundle: Arc<ConfigBundle>,
        sim: bool,
    ) -> Self {
        Self {
            link,
            stream_input,
            shared,
            bundle,
            sim,
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
                // The RT jog engine drives one joint at a time; collapse
                // to the dominant axis (UIs send single-axis jogs).
                let (joint, pct) = p
                    .speeds
                    .iter()
                    .copied()
                    .enumerate()
                    .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
                    .expect("NUM_JOINTS > 0");
                if p.speeds.iter().filter(|s| **s != 0.0).count() > 1 {
                    log::debug!("multi-axis jog_j collapsed to joint {joint}");
                }
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
                });
            }
            Command::JogL(_) | Command::ServoJPose(_) | Command::ServoL(_) => {
                // Cartesian streaming needs IK; the par6-kin adapter is a
                // follow-up owned by the kinematics workstream.
                log::warn!(
                    "{:?} requires kinematics (par6-kin adapter not wired yet); ignored",
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
        self.link.send(RtCommand::ExecFlush);
        self.link.send(RtCommand::SetMode(Mode::Idle));
    }

    fn set_enabled(&mut self, enabled: bool) {
        let mut sh = self.shared.lock().unwrap();
        if enabled {
            // Clear the soft e-stop flag and run the RT clear sequence;
            // Enable only succeeds after the clear settle window, so
            // housekeeping retries it until the snapshot reads ENABLED.
            self.link.send(RtCommand::SetSoftEstop(false));
            self.link.send(RtCommand::ClearErrors);
            sh.enable_deadline = Some(Instant::now() + ENABLE_RETRY_WINDOW);
        } else {
            self.link.send(RtCommand::SetSoftEstop(true));
            sh.enable_deadline = None;
        }
    }

    fn teleport(&mut self, angles_deg: &[f64; NUM_JOINTS], tool_positions: Option<&[f64]>) {
        if !self.sim {
            // The server gates teleport with SYS_NOT_SIMULATOR; this is
            // pure defense in depth.
            log::error!("teleport outside simulator mode reached the bridge; dropped");
            return;
        }
        if tool_positions.is_some() {
            log::warn!("teleport tool_positions are not supported by the sim backend; ignored");
        }
        let bundle = self.bundle.clone();
        let mut q = [0.0; MAX_JOINTS];
        for (i, (out, deg)) in q.iter_mut().zip(angles_deg.iter()).enumerate() {
            let l = &bundle.robot.joints[i].limits;
            *out = deg.to_radians().clamp(l.hard_min_rad, l.hard_max_rad);
        }
        self.link.op(Box::new(move |core| {
            let robot = &bundle.robot;
            let gripper = bundle.active_gripper().filter(|g| g.driver.is_some());
            let bus = core.bus_mut();
            bus.set_initial_joint_rad(&q);
            if let Err(e) = bus.boot_configure(robot, gripper, 1) {
                log::error!("teleport: sim re-seed failed: {e}");
                return;
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
                &format!("hardware bus '{port}' unavailable: the SocketCAN DriverBus backend has not landed"),
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
/// and the post-`reset` enable retry. `tick_dt_s` sizes the enable
/// trust window in RT ticks (all time constants are config seconds).
pub(crate) fn housekeeping_loop(
    link: CoreLink,
    stream_input: Arc<Mutex<StreamInput>>,
    shared: Arc<Mutex<SharedState>>,
    mut snapshots: SnapshotReader<StateSnapshot>,
    tick_dt_s: f64,
    shutdown: Arc<AtomicBool>,
) {
    // ENABLED may only be trusted as the outcome of THIS request once
    // the RT has had time to drain the queue ahead of it and run the
    // clear-settle countdown; before that it can be the stale pre-e-stop
    // state (the reset usually races the e-stop through the queue).
    let trust_delay_ticks =
        (par6_rt::errors::CLEAR_SETTLE_S / tick_dt_s).ceil() as u64 + ENABLE_TRUST_SLACK_TICKS;
    let mut window: Option<(Instant, u64)> = None; // (deadline, trust_after_tick)
    let mut last_enable: Option<Instant> = None;
    while !shutdown.load(Ordering::SeqCst) {
        let now = Instant::now();
        let snap = snapshots.latest();
        {
            let mut sh = shared.lock().unwrap();
            match &sh.stream {
                Some(a) if now >= a.deadline => {
                    match a.kind {
                        StreamKind::Jog => {
                            log::debug!("jog duration elapsed; releasing");
                            link.send(RtCommand::JogRelease);
                        }
                        StreamKind::Servo => log::debug!("servo stream went silent; stopping"),
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
                _ => {}
            }
            match sh.enable_deadline {
                Some(deadline) => {
                    if window.map(|(d, _)| d) != Some(deadline) {
                        window = Some((deadline, snap.tick + trust_delay_ticks));
                        last_enable = None;
                    }
                    let (_, trust_after) = window.expect("set above");
                    if snap.state == ArmState::Enabled && snap.tick >= trust_after {
                        sh.enable_deadline = None;
                        window = None;
                    } else if now >= deadline {
                        sh.enable_deadline = None;
                        window = None;
                        log::warn!("enable retry window expired; controller still DISABLED");
                    } else if last_enable
                        .is_none_or(|t| now.duration_since(t) >= ENABLE_RETRY_PERIOD)
                    {
                        link.send(RtCommand::Enable);
                        last_enable = Some(now);
                    }
                }
                None => window = None,
            }
        }
        std::thread::sleep(HOUSEKEEPING_PERIOD);
    }
}
