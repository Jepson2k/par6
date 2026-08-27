//! The command-plane actor: one tokio task owning the UDP command
//! socket, the motion queue, the SINGLE command-index allocator, and the
//! status/telemetry broadcast schedule.
//!
//! Decisions resolved here:
//!
//! - The index allocator is monotonic and NEVER reset — not even by
//!   `reset_state` — so a stale pre-reset status frame can never satisfy
//!   a post-reset wait.
//! - Gating rejections always answer with ERROR (echoed `req_id`),
//!   including FIRE_AND_FORGET commands whose success stays unacked. A
//!   refused fire-and-forget additionally latches as the standing error
//!   while the pipeline is idle, so the refusal reaches STATUS and the
//!   ERROR query — no caller awaits the reply datagram itself.
//! - A parameter this runtime cannot honour is REFUSED the same way
//!   (`validate_supported`), never dropped: a command that half-executes
//!   moves the arm somewhere the client did not ask for, and the client
//!   has no way to find out.
//! - Cancellation (stop/estop/reset/simulator toggle/stream preemption)
//!   drops commands WITHOUT a COMPLETE push; clients observe it through
//!   the status stream (queue emptied, `executing_index` = −1, and for
//!   estop a standing `SYS_ESTOP_ACTIVE` error whose ordering fails
//!   their waits).
//! - A failed queued command latches its error (attributed to its
//!   index), pushes `COMPLETE(ok=false)`, and clears the pending queue —
//!   later commands must not run from an unexpected position. Acceptance
//!   of the next motion command clears the standing error.
//! - `stop {clear_queue: false}` halts the active motion only; the next
//!   queued command then starts. `{clear_queue: true}` also empties the
//!   queue. `estop` = stop + clear + DISABLED latch until `reset`;
//!   `reset_state` resets world/tool/errors but NOT the e-stop latch and
//!   NOT the index allocator.

use std::collections::{HashMap, VecDeque};
use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Instant;

use par6_proto::{
    command_class, decode_chunk, decode_command, encode_reply, make_error, peek_tag, ActionState,
    CmdType, Command, CommandClass, CompletionPolicy, ControllerMode, DecodeError, ErrorCode,
    LoopStatsResult, MsgType, QueryResult, Reassembler, Reply, Status, StatusEncoder, ToolState,
    ToolStatusWire, WireError, EN_SLOTS, NUM_JOINTS, POSE_ELEMS, PROTO_VERSION, UNATTRIBUTED,
};
use par6_rt::{ArmState, Mode, StateSnapshot};
use tokio::net::UdpSocket;
use tokio::sync::Notify;
use tokio::time::MissedTickBehavior;

use crate::config::ServerConfig;
use crate::faults::{gripper_fault_code, rt_standing_error};
use crate::gating::{gate, is_stream};
use crate::link::BroadcastLink;
use crate::runtime::{
    blend_radius_mm, CollisionState, Enablement, PayloadSpec, PlanContext, Planner, QueuedCommand,
    RtCommands, RuntimeHandle, ShapeLayer,
};
use crate::telemetry;

/// Cap on the `action_params` summary string.
const MAX_PARAMS_LEN: usize = 100;

/// How many un-answered `reset` requests may pile up while the RT is
/// still deciding. A client that keeps re-sending past this is not
/// waiting for an answer, and the queue must stay bounded.
const MAX_RESET_WAITERS: usize = 16;

/// How many datagrams one stream preemption may lift off the socket.
/// Two clients streaming at 50 Hz put a handful in the queue; the cap is
/// what stops a flooding peer from keeping the server inside the drain
/// instead of back in its event loop.
const DRAIN_LIMIT: usize = 64;

/// Handle to a running server. Dropping it aborts the task.
pub struct ServerHandle {
    /// Bound command-socket address (useful with an ephemeral bind).
    pub addr: SocketAddr,
    shutdown: Arc<Notify>,
    task: tokio::task::JoinHandle<()>,
}

impl ServerHandle {
    /// Ask the server task to stop after its current event.
    pub fn shutdown(&self) {
        self.shutdown.notify_one();
    }
}

impl Drop for ServerHandle {
    fn drop(&mut self) {
        self.task.abort();
    }
}

/// Bind the command socket, run the broadcast-transport ladder, and
/// spawn the server task.
pub async fn spawn<P, R>(
    cfg: ServerConfig,
    runtime: RuntimeHandle<P, R>,
) -> std::io::Result<ServerHandle>
where
    P: Planner + 'static,
    R: RtCommands + 'static,
{
    // An unknown startup recipe is a startup failure, exactly as
    // `set_recipe` refuses it live — a silent fallback looks like a
    // dead robot.
    if let Some(name) = &cfg.initial_recipe {
        if !cfg.recipes.iter().any(|r| r.name == *name) {
            return Err(std::io::Error::other(format!(
                "unknown initial telemetry recipe {name:?}"
            )));
        }
    }
    let socket = UdpSocket::bind(cfg.bind).await?;
    let addr = socket.local_addr()?;
    let link = BroadcastLink::open(&cfg).await?;
    let shutdown = Arc::new(Notify::new());
    let stop = shutdown.clone();
    let mut core = Core::new(cfg, runtime, socket, link);
    // A configured keep-out the runtime cannot apply is a startup
    // failure: coming up anyway would enforce a world the operator did
    // not configure, and nobody is listening yet to be told.
    core.install_shapes()
        .map_err(|e| std::io::Error::other(format!("installation shapes refused: {}", e.cause)))?;
    let task = tokio::spawn(core.run(stop));
    Ok(ServerHandle {
        addr,
        shutdown,
        task,
    })
}

struct Pending {
    index: u64,
    cmd: Command,
    addr: SocketAddr,
}

enum PostEffect {
    None,
    Checkpoint(String),
    /// `select_tool` can only ever name the fitted tool (validated at
    /// accept time), so the variant is the part that actually changes.
    SelectVariant(Option<String>),
}

struct Executing {
    index: u64,
    addr: SocketAddr,
    name: &'static str,
    params: String,
    effect: PostEffect,
    /// Commands the planner blended into this motion, in queue order
    /// (motion commands only — a blend chain never reaches past one).
    /// They finish when it finishes: each gets its own COMPLETE push and
    /// the high-water `completed_index` jumps to the last of them, which
    /// is what protocol v2 prescribes for blended-away
    /// commands. There is no earlier honest moment to call one of them
    /// done — the arm never stops at their targets, and their samples
    /// are interleaved with the head's in one trajectory.
    blended: Vec<(u64, SocketAddr)>,
}

/// Idempotency dedup window: last N keys → original index.
struct Dedup {
    cap: usize,
    order: VecDeque<u64>,
    map: HashMap<u64, u64>,
}

impl Dedup {
    fn new(cap: usize) -> Self {
        Self {
            cap: cap.max(1),
            order: VecDeque::new(),
            map: HashMap::new(),
        }
    }

    fn get(&self, key: u64) -> Option<u64> {
        self.map.get(&key).copied()
    }

    fn insert(&mut self, key: u64, index: u64) {
        if self.map.insert(key, index).is_none() {
            self.order.push_back(key);
            if self.order.len() > self.cap {
                if let Some(old) = self.order.pop_front() {
                    self.map.remove(&old);
                }
            }
        }
    }
}

struct Core<P: Planner, R: RtCommands> {
    cfg: ServerConfig,
    runtime: RuntimeHandle<P, R>,
    socket: UdpSocket,
    link: BroadcastLink,
    txbuf: Vec<u8>,
    encoder: StatusEncoder,
    start: Instant,

    reassembler: Reassembler,
    transfer_addrs: HashMap<u32, SocketAddr>,

    next_index: u64,
    dedup: Dedup,
    pending: VecDeque<Pending>,
    executing: Option<Executing>,
    /// Deadline the head of the queue is being held to, while it waits
    /// for the successor its blend radius asks to round a corner into.
    blend_hold: Option<Instant>,
    accepted_index: i64,
    completed_index: i64,
    last_checkpoint: String,
    standing_error: Option<WireError>,
    action_state: ActionState,
    estop_latched: bool,
    active_stream: Option<CmdType>,
    /// Datagrams a preemption drain took off the socket without being
    /// entitled to discard them; dispatched by the run loop, in order.
    deferred: VecDeque<(Vec<u8>, SocketAddr)>,
    /// Scratch buffer for that drain (full datagram size — a short one
    /// would truncate whatever it rescued).
    drainbuf: Vec<u8>,
    /// Clients whose `reset` is still waiting for the RT's answer.
    reset_waiters: Vec<(u32, SocketAddr)>,
    /// Whether the boot-time enable has been requested yet.
    booted: bool,

    profile: String,
    tool: String,
    tool_variant: Option<String>,
    tcp_offset_mm: [f64; 3],
    /// The commanded runtime payload — served back by the PAYLOAD query.
    payload: PayloadSpec,
    shapes: Vec<par6_proto::Shape>,
    scene_epoch: u64,
    collision: CollisionState,
    completion_policy: CompletionPolicy,
    recipe: Option<String>,
    simulator: bool,

    /// Planner estimate of the pending queue, and the `(front, len)` of
    /// the queue it describes.
    queue_estimate: f64,
    queue_estimate_for: (u64, usize),

    snap: StateSnapshot,
    last_fresh: Option<Instant>,
    status_seq: u64,
    telemetry_seq: u64,
    tcp_speed: f64,
    prev_tcp: Option<([f64; 3], Instant)>,
}

enum Event {
    Datagram(usize, SocketAddr),
    Poll,
    Status,
    Telemetry,
    Shutdown,
}

impl<P: Planner, R: RtCommands> Core<P, R> {
    fn new(
        cfg: ServerConfig,
        runtime: RuntimeHandle<P, R>,
        socket: UdpSocket,
        link: BroadcastLink,
    ) -> Self {
        let reassembler = Reassembler::new(cfg.chunk_timeout);
        Self {
            dedup: Dedup::new(cfg.dedup_window),
            profile: cfg.initial_profile.clone(),
            recipe: cfg.initial_recipe.clone(),
            simulator: cfg.simulator,
            tool: cfg.fitted_tool.clone(),
            cfg,
            runtime,
            socket,
            link,
            txbuf: Vec::with_capacity(2048),
            encoder: StatusEncoder::new(),
            start: Instant::now(),
            reassembler,
            transfer_addrs: HashMap::new(),
            next_index: 1,
            pending: VecDeque::new(),
            executing: None,
            blend_hold: None,
            accepted_index: -1,
            completed_index: -1,
            last_checkpoint: String::new(),
            standing_error: None,
            action_state: ActionState::Idle,
            estop_latched: false,
            active_stream: None,
            deferred: VecDeque::new(),
            drainbuf: vec![0u8; 65535],
            reset_waiters: Vec::new(),
            booted: false,
            tool_variant: None,
            tcp_offset_mm: [0.0; 3],
            payload: PayloadSpec::default(),
            shapes: Vec::new(),
            scene_epoch: 0,
            collision: CollisionState::default(),
            completion_policy: CompletionPolicy::Settled,
            queue_estimate: 0.0,
            queue_estimate_for: (0, 0),
            snap: StateSnapshot::default(),
            last_fresh: None,
            status_seq: 0,
            telemetry_seq: 0,
            tcp_speed: 0.0,
            prev_tcp: None,
        }
    }

    async fn run(mut self, shutdown: Arc<Notify>) {
        let mut rxbuf = vec![0u8; 65535];
        let mut poll_iv = tokio::time::interval(self.cfg.poll_interval);
        let mut status_iv = tokio::time::interval(rate_period(self.cfg.status_rate_hz));
        let mut telem_iv = tokio::time::interval(rate_period(self.cfg.telemetry_rate_hz));
        for iv in [&mut poll_iv, &mut status_iv, &mut telem_iv] {
            iv.set_missed_tick_behavior(MissedTickBehavior::Skip);
        }
        self.sync_planner();
        loop {
            let ev = tokio::select! {
                r = self.socket.recv_from(&mut rxbuf) => match r {
                    Ok((n, addr)) => Event::Datagram(n, addr),
                    Err(e) => {
                        log::debug!("command socket recv error: {e}");
                        Event::Poll
                    }
                },
                _ = poll_iv.tick() => Event::Poll,
                _ = status_iv.tick() => Event::Status,
                _ = telem_iv.tick() => Event::Telemetry,
                _ = shutdown.notified() => Event::Shutdown,
            };
            match ev {
                Event::Datagram(n, addr) => {
                    self.on_datagram(&rxbuf[..n], addr).await;
                    // A stream preemption may have lifted other clients'
                    // traffic off the socket to get at the stale
                    // setpoints behind it. Those datagrams are dispatched
                    // here, in arrival order, before the next event.
                    while let Some((data, from)) = self.deferred.pop_front() {
                        self.on_datagram(&data, from).await;
                    }
                }
                Event::Poll => self.on_poll().await,
                Event::Status => self.on_status().await,
                Event::Telemetry => self.on_telemetry().await,
                Event::Shutdown => break,
            }
        }
    }

    // ---- datagram dispatch -------------------------------------------------

    async fn on_datagram(&mut self, data: &[u8], addr: SocketAddr) {
        self.refresh_snapshot();
        match peek_tag(data) {
            Ok(t) if t == MsgType::Chunk as u8 as i64 => self.on_chunk(data, addr).await,
            _ => self.on_command_bytes(data, addr).await,
        }
    }

    async fn on_command_bytes(&mut self, data: &[u8], addr: SocketAddr) {
        match decode_command(data) {
            Ok((req_id, cmd)) => self.dispatch(req_id, cmd, addr).await,
            Err(e) => {
                let req_id = par6_proto::peek_req_id(data).unwrap_or(0);
                let error = decode_error_to_wire(&e);
                self.reply(addr, &Reply::Error { req_id, error }).await;
            }
        }
    }

    async fn dispatch(&mut self, req_id: u32, cmd: Command, addr: SocketAddr) {
        match command_class(cmd.tag()) {
            CommandClass::Query => self.on_query(req_id, &cmd, addr).await,
            CommandClass::System => self.on_system(req_id, &cmd, addr).await,
            CommandClass::FireAndForget => self.on_faf(req_id, cmd, addr).await,
            CommandClass::Queued => self.on_queued(req_id, cmd, addr).await,
        }
    }

    async fn on_chunk(&mut self, data: &[u8], addr: SocketAddr) {
        let chunk = match decode_chunk(data) {
            Ok(c) => c,
            Err(e) => {
                let req_id = par6_proto::peek_req_id(data).unwrap_or(0);
                let error = decode_error_to_wire(&e);
                self.reply(addr, &Reply::Error { req_id, error }).await;
                return;
            }
        };
        let req_id = chunk.req_id;
        let transfer_id = chunk.transfer_id;
        self.transfer_addrs.insert(transfer_id, addr);
        match self.reassembler.push(chunk, Instant::now()) {
            Ok(None) => {}
            Ok(Some(assembled)) => {
                self.transfer_addrs.remove(&transfer_id);
                self.on_command_bytes(&assembled.payload, addr).await;
            }
            Err(e) => {
                self.transfer_addrs.remove(&transfer_id);
                let error = make_error(
                    ErrorCode::CommDecodeError,
                    UNATTRIBUTED,
                    &[("detail", &e.to_string())],
                );
                self.reply(addr, &Reply::Error { req_id, error }).await;
            }
        }
    }

    async fn expire_chunks(&mut self) {
        for exp in self.reassembler.expire(Instant::now()) {
            let Some(addr) = self.transfer_addrs.remove(&exp.transfer_id) else {
                continue;
            };
            let error = make_error(
                ErrorCode::CommChunkTimeout,
                UNATTRIBUTED,
                &[
                    ("transfer_id", &exp.transfer_id.to_string()),
                    ("received", &exp.received.to_string()),
                    ("total", &exp.total.to_string()),
                ],
            );
            self.reply(
                addr,
                &Reply::Error {
                    req_id: exp.req_id,
                    error,
                },
            )
            .await;
        }
    }

    // ---- ticks -------------------------------------------------------------

    async fn on_poll(&mut self) {
        self.refresh_snapshot();
        self.request_boot_enable();
        self.settle_enable().await;
        self.expire_chunks().await;
        self.collect_outcomes().await;
        self.pump().await;
    }

    async fn on_status(&mut self) {
        self.refresh_snapshot();
        self.update_tcp_speed();
        self.update_collision();
        self.refresh_queue_estimate();
        let status = self.build_status();
        self.status_seq += 1;
        let bytes = self.encoder.encode(&status);
        self.link.send(self.cfg.status_port, bytes).await;
    }

    async fn on_telemetry(&mut self) {
        self.refresh_snapshot();
        let Some(name) = self.recipe.as_deref() else {
            return;
        };
        let Some(recipe) = self.cfg.recipes.iter().find(|r| r.name == name) else {
            return;
        };
        let pkt = telemetry::encode_packet(recipe, self.telemetry_seq, self.mono_ns(), &self.snap);
        self.telemetry_seq += 1;
        self.link.send(self.cfg.telemetry_port, &pkt).await;
    }

    // ---- command classes ---------------------------------------------------

    async fn on_query(&mut self, req_id: u32, cmd: &Command, addr: SocketAddr) {
        self.refresh_queue_estimate();
        let result = self.query_result(cmd);
        self.reply(addr, &Reply::Response { req_id, result }).await;
    }

    async fn on_system(&mut self, req_id: u32, cmd: &Command, addr: SocketAddr) {
        use Command as C;
        if matches!(cmd, C::Reset) {
            self.on_reset(req_id, addr).await;
            return;
        }
        let result: Result<(), WireError> = match cmd {
            C::Estop => {
                self.cancel_all_motion();
                self.estop_latched = true;
                self.runtime.rt.set_enabled(false);
                self.standing_error =
                    Some(make_error(ErrorCode::SysEstopActive, UNATTRIBUTED, &[]));
                Ok(())
            }
            C::Pause(p) => {
                self.runtime.rt.set_exec_paused(p.on);
                Ok(())
            }
            C::SetGravityComp(p) => {
                self.runtime.rt.set_gravity_comp(p.on);
                Ok(())
            }
            C::Stop(p) => {
                self.cancel_active_motion();
                if p.clear_queue {
                    self.pending.clear();
                    self.blend_hold = None;
                }
                Ok(())
            }
            // `port` indexes the box's DECLARED outputs, so the wire's
            // own 0..=7 bound is not the answer here: a port past the
            // end names no line, and acking it would report a level the
            // arm never drove.
            C::WriteIo(p) => match self.cfg.digital_outputs.get(usize::from(p.port)) {
                Some(name) => {
                    log::debug!("write_io {name} (port {}) = {}", p.port, p.value);
                    self.runtime.rt.write_io(p.port, p.value);
                    Ok(())
                }
                None => Err(make_error(
                    ErrorCode::CommValidationError,
                    UNATTRIBUTED,
                    &[(
                        "detail",
                        &format!(
                            "write_io port {} does not exist: this box declares {} digital \
                             output(s)",
                            p.port,
                            self.cfg.digital_outputs.len()
                        ),
                    )],
                )),
            },
            C::Simulator(p) => {
                self.cancel_all_motion();
                self.runtime.rt.set_simulator(p.on).map(|()| {
                    self.simulator = p.on;
                })
            }
            C::SelectProfile(p) => {
                // Matched case-insensitively, stored in the registry's
                // spelling — the planner keys its behaviour off that.
                let known = self
                    .cfg
                    .profiles
                    .iter()
                    .find(|x| x.eq_ignore_ascii_case(&p.profile))
                    .cloned();
                match known {
                    Some(name) => {
                        self.profile = name;
                        self.sync_planner();
                        Ok(())
                    }
                    None => Err(make_error(
                        ErrorCode::SysProfileInvalid,
                        UNATTRIBUTED,
                        &[("detail", &p.profile)],
                    )),
                }
            }
            C::ResetState => {
                self.cancel_all_motion();
                self.standing_error = None;
                self.action_state = ActionState::Idle;
                self.last_checkpoint.clear();
                self.tool.clone_from(&self.cfg.fitted_tool);
                self.tool_variant = None;
                self.tcp_offset_mm = [0.0; 3];
                self.completion_policy = CompletionPolicy::Settled;
                self.profile = self.cfg.initial_profile.clone();
                self.recipe = self.cfg.initial_recipe.clone();
                self.runtime.rt.reset_state();
                self.sync_planner();
                // The program layer only: installation keep-outs are the
                // deployment's, not the program's, and survive.
                self.apply_program_shapes(Vec::new())
            }
            // Same reason `simulator` cancels first: the bus under a
            // running move is about to become a different arm, and a
            // move resumed against one whose position is not yet known
            // is a move to somewhere nobody asked for.
            C::ConnectHardware(p) => {
                self.cancel_all_motion();
                self.runtime.rt.connect_hardware(&p.port).inspect(|()| {
                    self.simulator = false;
                })
            }
            C::SetTcpOffset(p) => {
                self.tcp_offset_mm = [p.x, p.y, p.z];
                self.sync_planner();
                Ok(())
            }
            C::SetPayload(p) => {
                let payload = PayloadSpec {
                    mass: p.mass,
                    com: p.com,
                    inertia: p.inertia,
                };
                self.payload = payload;
                self.runtime.rt.set_payload(payload);
                self.sync_planner();
                Ok(())
            }
            C::SetShapes(p) => self.apply_program_shapes(p.shapes.clone()),
            C::SetCompletionPolicy(p) => {
                self.completion_policy = p.policy;
                self.sync_planner();
                Ok(())
            }
            C::SetRecipe(p) => {
                if self.cfg.recipes.iter().any(|r| r.name == p.name) {
                    self.recipe = Some(p.name.clone());
                    Ok(())
                } else {
                    Err(make_error(
                        ErrorCode::CommUnknownRecipe,
                        UNATTRIBUTED,
                        &[("name", &p.name)],
                    ))
                }
            }
            _ => unreachable!("dispatch routes only SYSTEM commands here"),
        };
        let reply = match result {
            Ok(()) => Reply::Ok {
                req_id,
                index: None,
            },
            Err(error) => Reply::Error { req_id, error },
        };
        self.reply(addr, &reply).await;
    }

    /// `reset` asks the RT to enable; only the RT can say whether it did.
    /// The reply therefore waits for [`RtCommands::take_enable_outcome`]
    /// — reporting OK on the strength of having sent the request is
    /// exactly the "success without confirmation" the waldoctl client
    /// contract forbids, and it is what let a `reset()` answer 1 while
    /// the very next command was still refused as DISABLED.
    async fn on_reset(&mut self, req_id: u32, addr: SocketAddr) {
        if self.reset_waiters.len() >= MAX_RESET_WAITERS {
            // Same code as a full motion queue, so the `{detail}` slot is
            // what tells an operator retrying `reset` on a latched arm
            // apart from one flooding moves — without it the template
            // pointed at an empty motion queue.
            let error = make_error(
                ErrorCode::CommQueueFull,
                UNATTRIBUTED,
                &[(
                    "detail",
                    "Too many reset requests are already awaiting the \
                     controller's verdict; stop retrying until the \
                     outstanding reset is answered.",
                )],
            );
            self.reply(addr, &Reply::Error { req_id, error }).await;
            return;
        }
        self.estop_latched = false;
        self.standing_error = None;
        self.action_state = ActionState::Idle;
        self.runtime.rt.set_enabled(true);
        self.reset_waiters.push((req_id, addr));
    }

    /// Hand the RT's enable verdict to whoever is waiting on it.
    async fn settle_enable(&mut self) {
        let Some(outcome) = self.runtime.rt.take_enable_outcome() else {
            return;
        };
        if self.reset_waiters.is_empty() {
            // The boot-time enable: nobody asked, so nobody is answered.
            // A refusal is not lost — it comes from an RT latch, and that
            // latch is what STATUS and the ERROR query now report.
            if let Err(e) = &outcome {
                log::warn!("startup enable refused: {e}");
            }
            return;
        }
        for (req_id, addr) in std::mem::take(&mut self.reset_waiters) {
            let reply = match &outcome {
                Ok(()) => Reply::Ok {
                    req_id,
                    index: None,
                },
                Err(error) => Reply::Error {
                    req_id,
                    error: error.clone(),
                },
            };
            self.reply(addr, &reply).await;
        }
    }

    /// Come up ready to accept motion, the way parol6's controller does
    /// (`server/state.py`, `enabled = True`): its `enabled` flag is a
    /// PROTECTIVE-STOP latch, and nothing is latched on a clean boot.
    /// par6's `ArmState` means the same thing — motors stay energized and
    /// holding either way — so booting DISABLED
    /// only meant that nothing moved until a client sent `reset`, which
    /// no frontend does at startup.
    ///
    /// It is safe because the enable runs through the identical gate a
    /// client `reset` does: the RT refuses it while the e-stop line is
    /// engaged or any hard error is latched, and every motion MODE
    /// additionally requires a home reference that only an explicit
    /// `home` can grant. Deferred until the core leaves BOOTING so a
    /// command can never be accepted for a mode the core cannot enter.
    fn request_boot_enable(&mut self) {
        if self.booted || self.snap.mode == Mode::Booting {
            return;
        }
        self.booted = true;
        log::info!("RT core out of BOOTING: enabling the controller");
        self.runtime.rt.set_enabled(true);
    }

    async fn on_faf(&mut self, req_id: u32, cmd: Command, addr: SocketAddr) {
        let tag = cmd.tag();
        if tag == CmdType::ResetLoopStats {
            self.runtime.rt.reset_loop_stats();
            return;
        }
        if let Some(error) = self
            .check_gate(tag)
            .or_else(|| self.validate_supported(&cmd))
        {
            // Rejection gets a real ERROR even though success is unacked —
            // and, when nothing else stands, it latches as the standing
            // error: no caller awaits a fire-and-forget reply, so the
            // datagram alone leaves `error()` = None over an arm that
            // silently refuses to move (issue #23).
            self.latch_faf_refusal(&error);
            self.reply(addr, &Reply::Error { req_id, error }).await;
            return;
        }
        if let Command::Teleport(p) = &cmd {
            // Streamable-class: preempts the active stream and planned
            // motion, but is not itself a continuing stream.
            if let Some(superseded) = self.active_stream.take() {
                self.runtime.rt.cancel_stream();
                self.drain_stream_backlog(superseded);
            }
            self.cancel_planned();
            self.runtime
                .rt
                .teleport(&p.angles, p.tool_positions.as_deref());
            self.on_motion_accepted();
            return;
        }
        debug_assert!(is_stream(tag));
        let outcome = match self.active_stream {
            Some(active) if active == tag => {
                // Same type: update the active command in place — no new
                // index, no cancel, no drain.
                let outcome = self.runtime.rt.stream(&cmd);
                if outcome.is_err() {
                    // A refused update stops the stream it was updating:
                    // the client asked for a direction the gate blocks,
                    // and letting the PREVIOUS setpoint keep driving
                    // would carry the arm on while the refusal is read.
                    self.active_stream = None;
                    self.runtime.rt.cancel_stream();
                }
                outcome
            }
            _ => {
                if let Some(superseded) = self.active_stream.take() {
                    // Type change: cancel and flush the stale backlog of
                    // the previous stream before starting fresh.
                    self.runtime.rt.cancel_stream();
                    self.drain_stream_backlog(superseded);
                }
                self.cancel_planned();
                let outcome = self.runtime.rt.stream(&cmd);
                if outcome.is_ok() {
                    self.active_stream = Some(tag);
                }
                outcome
            }
        };
        match outcome {
            Ok(()) => self.on_motion_accepted(),
            Err(error) => {
                // A refused streamable is a refused fire-and-forget:
                // ERROR to the sender, latched while nothing truer
                // stands. The gate's own collision latch (if the refusal
                // was a collision) reaches STATUS through
                // `update_collision`.
                self.latch_faf_refusal(&error);
                self.reply(addr, &Reply::Error { req_id, error }).await;
            }
        }
    }

    async fn on_queued(&mut self, req_id: u32, cmd: Command, addr: SocketAddr) {
        let key = cmd
            .idempotency_key()
            .expect("queued commands carry an idempotency key");
        if let Some(index) = self.dedup.get(key) {
            // Retry of an already-accepted command: re-ack the ORIGINAL
            // index instead of double-queueing.
            self.reply(
                addr,
                &Reply::Ok {
                    req_id,
                    index: Some(index),
                },
            )
            .await;
            return;
        }
        if let Some(error) = self.check_gate(cmd.tag()) {
            self.reply(addr, &Reply::Error { req_id, error }).await;
            return;
        }
        if let Some(error) = self
            .validate_registries(&cmd)
            .or_else(|| self.validate_supported(&cmd))
        {
            self.reply(addr, &Reply::Error { req_id, error }).await;
            return;
        }
        if self.pending.len() >= self.cfg.queue_capacity {
            let error = make_error(
                ErrorCode::CommQueueFull,
                UNATTRIBUTED,
                &[(
                    "detail",
                    "The motion queue is full; wait for queued motions to \
                     finish before enqueueing more.",
                )],
            );
            self.reply(addr, &Reply::Error { req_id, error }).await;
            return;
        }
        let index = self.next_index;
        self.next_index += 1;
        self.dedup.insert(key, index);
        self.accepted_index = index as i64;
        self.on_motion_accepted();
        if self.active_stream.take().is_some() {
            // A planned move cancels streaming.
            self.runtime.rt.cancel_stream();
        }
        self.pending.push_back(Pending { index, cmd, addr });
        self.reply(
            addr,
            &Reply::Ok {
                req_id,
                index: Some(index),
            },
        )
        .await;
        self.pump().await;
    }

    // ---- gating & validation ----------------------------------------------

    fn check_gate(&self, tag: CmdType) -> Option<WireError> {
        let g = gate(tag);
        if g.needs_enabled {
            if self.estop_latched {
                return Some(make_error(ErrorCode::SysEstopActive, UNATTRIBUTED, &[]));
            }
            if self.snap.state != ArmState::Enabled {
                return Some(make_error(
                    ErrorCode::SysControllerDisabled,
                    UNATTRIBUTED,
                    &[("detail", "The RT core reports DISABLED.")],
                ));
            }
        }
        if g.needs_homed && !self.snap.homed {
            return Some(make_error(ErrorCode::MotnNotHomed, UNATTRIBUTED, &[]));
        }
        if g.needs_simulator && !self.simulator {
            return Some(make_error(ErrorCode::SysNotSimulator, UNATTRIBUTED, &[]));
        }
        None
    }

    /// Server-layer name checks the codec deliberately leaves to config.
    /// Tool keys are matched case-insensitively: the registry spells them
    /// as the config does, clients as their own tool tables do.
    fn validate_registries(&self, cmd: &Command) -> Option<WireError> {
        let name = match cmd {
            Command::SelectTool(p) => &p.tool_name,
            Command::ToolAction(p) => &p.tool_key,
            _ => return None,
        };
        let detail = if !self
            .cfg
            .tools
            .iter()
            .any(|t| t.eq_ignore_ascii_case(name.as_str()))
        {
            format!(
                "unknown tool '{name}'; this runtime knows {:?}",
                self.cfg.tools
            )
        } else if !self.cfg.fitted_tool.eq_ignore_ascii_case(name.as_str()) {
            // Selecting a tool swaps the kinematic and gravity models,
            // which are built at startup from the configured tool.
            format!(
                "tool '{name}' is not fitted; this runtime is running '{}' \
                 (change robot.active_gripper and restart par6d)",
                self.cfg.fitted_tool
            )
        } else {
            return None;
        };
        Some(make_error(
            ErrorCode::CommValidationError,
            UNATTRIBUTED,
            &[("detail", &detail)],
        ))
    }

    /// Parameters the runtime cannot honour. Refusing them is the whole
    /// point: a silently dropped parameter makes the arm do something
    /// other than what the client asked for, with no way to tell.
    fn validate_supported(&self, cmd: &Command) -> Option<WireError> {
        let refuse = |detail: String| {
            Some(make_error(
                ErrorCode::CommValidationError,
                UNATTRIBUTED,
                &[("detail", &detail)],
            ))
        };
        // A corner is rounded by re-planning both of its segments as one
        // path, which needs kinematics (IK along the rounded corner, and
        // TOPPRA to time it). A runtime without them can only stop at
        // every waypoint, and saying so beats doing it silently.
        let blend = |r: Option<f64>| {
            r.filter(|r| *r > 0.0 && !self.cfg.cartesian).map(|r| {
                format!(
                    "blend radius {r} mm needs kinematics: this runtime has none, \
                     so every move stops at its target; send r = nil"
                )
            })
        };
        let unsupported = match cmd {
            Command::MoveJ(p) => blend(p.blend_radius),
            Command::MoveJPose(p) => blend(p.blend_radius),
            Command::MoveL(p) => blend(p.blend_radius),
            // An arc ends where its `end` pose is: par6d rounds corners
            // between straight segments and between joint moves, but has
            // no arc-to-successor blend, and a radius that quietly did
            // nothing would be the silent alteration this function
            // exists to prevent.
            Command::MoveC(p) => p.blend_radius.filter(|r| *r > 0.0).map(|r| {
                format!(
                    "blend radius {r} mm is not supported on move_c: an arc stops at \
                     its end pose; send r = nil"
                )
            }),
            // A pose the runtime cannot place the arm at is refused, not
            // clamped: clamping landed the arm tens of degrees from where
            // the client asked and answered success, which is the silent
            // alteration this whole function exists to prevent.
            Command::Teleport(p) => {
                teleport_angle_fault(&p.angles, &self.cfg).or(match p.tool_positions.as_deref() {
                    None => None,
                    Some(_) if self.cfg.tool_dof == 0 => Some(format!(
                        "tool '{}' has no controllable position",
                        self.cfg.fitted_tool
                    )),
                    Some(pos) if pos.len() != self.cfg.tool_dof => Some(format!(
                        "tool '{}' has {} position(s), {} given",
                        self.cfg.fitted_tool,
                        self.cfg.tool_dof,
                        pos.len()
                    )),
                    Some(pos) => pos
                        .iter()
                        .position(|v| !v.is_finite() || !(0.0..=1.0).contains(v))
                        .map(|i| format!("tool_positions[{i}] = {} is outside [0, 1]", pos[i])),
                })
            }
            // A passive tool (no driver) has nothing to actuate.
            Command::ToolAction(p) if self.cfg.tool_dof == 0 => Some(format!(
                "tool '{}' is passive: it has no actions",
                p.tool_key
            )),
            // Cartesian streamables need IK every tick; a runtime without
            // kinematics can only drop them.
            Command::ServoJPose(_) | Command::ServoL(_) | Command::JogL(_)
                if !self.cfg.cartesian =>
            {
                Some("this runtime has no kinematics: cartesian commands are unavailable".into())
            }
            _ => None,
        };
        unsupported.and_then(refuse)
    }

    // ---- queue engine ------------------------------------------------------

    async fn pump(&mut self) {
        while self.executing.is_none() {
            if self.estop_latched
                || self.snap.state != ArmState::Enabled
                || self.active_stream.is_some()
            {
                break;
            }
            if self.pending.is_empty() {
                self.blend_hold = None;
                break;
            }
            if self.holding_for_blend() {
                break;
            }
            self.blend_hold = None;
            // Disjoint field borrows: the queue is read to build the
            // lookahead while the planner is driven.
            let batch: Vec<QueuedCommand<'_>> = self
                .pending
                .iter()
                .take(self.cfg.blend_lookahead.max(1))
                .map(|p| QueuedCommand {
                    index: p.index,
                    cmd: &p.cmd,
                })
                .collect();
            let started = self.runtime.planner.start(&batch);
            let taken = match started {
                Ok(n) => n.clamp(1, batch.len()),
                Err(_) => 1,
            };
            drop(batch);
            let pc = self.pending.pop_front().expect("checked non-empty");
            let blended: Vec<(u64, SocketAddr)> = (1..taken)
                .map(|_| {
                    let p = self.pending.pop_front().expect("planner took what it saw");
                    (p.index, p.addr)
                })
                .collect();
            match started {
                Ok(_) => {
                    let name = cmd_name(pc.cmd.tag());
                    if !blended.is_empty() {
                        log::debug!(
                            "command {} blends {} following command(s) into one motion",
                            pc.index,
                            blended.len()
                        );
                    }
                    self.action_state = ActionState::Executing;
                    self.executing = Some(Executing {
                        index: pc.index,
                        addr: pc.addr,
                        name,
                        params: params_summary(&pc.cmd),
                        effect: post_effect(&pc.cmd),
                        blended,
                    });
                }
                Err(e) => {
                    self.fail_command(pc.index, pc.addr, e).await;
                    break;
                }
            }
        }
    }

    /// Whether the head of the queue is a blended move still waiting for
    /// the successor it is supposed to round a corner into.
    ///
    /// The wait is on the LAST queued command: while it asks to blend
    /// into something that has not been queued yet, the chain the
    /// planner would build is still growing, and starting now would cost
    /// the corner. It ends when a command that stops at its target
    /// arrives, when the lookahead is full, or when the hold expires.
    fn holding_for_blend(&mut self) -> bool {
        let wants_more = self.pending.len() < self.cfg.blend_lookahead
            && self
                .pending
                .back()
                .and_then(|p| blend_radius_mm(&p.cmd))
                .is_some_and(|r| r > 0.0);
        if !wants_more {
            return false;
        }
        let deadline = *self
            .blend_hold
            .get_or_insert_with(|| Instant::now() + self.cfg.blend_hold);
        Instant::now() < deadline
    }

    async fn collect_outcomes(&mut self) {
        while let Some(out) = self.runtime.planner.poll() {
            let Some(ex) = &self.executing else {
                continue; // outcome of a cancelled command
            };
            if ex.index != out.index {
                continue; // stale outcome (superseded by cancellation)
            }
            let ex = self.executing.take().expect("checked above");
            match out.error {
                None => {
                    self.completed_index = self.completed_index.max(ex.index as i64);
                    self.action_state = ActionState::Idle;
                    match ex.effect {
                        PostEffect::None => {}
                        PostEffect::Checkpoint(label) => self.last_checkpoint = label,
                        PostEffect::SelectVariant(variant) => {
                            // A variant carries its own TCP frame, so an
                            // offset measured against the old one describes
                            // nothing once it changes — a real change clears
                            // it, a re-selection of the same variant leaves
                            // it alone (the client API documents the reset,
                            // and it is what the parol6 runtime does).
                            if variant != self.tool_variant {
                                self.tcp_offset_mm = [0.0; 3];
                            }
                            self.tool_variant = variant;
                            self.sync_planner();
                        }
                    }
                    self.push_complete(ex.addr, ex.index, None).await;
                    // Blended-away commands finished in the same motion:
                    // each is completed in queue order, and the
                    // high-water mark ends on the last of them.
                    for (index, addr) in ex.blended {
                        self.completed_index = self.completed_index.max(index as i64);
                        self.push_complete(addr, index, None).await;
                    }
                }
                Some(e) => {
                    // The whole blended motion failed. The error is
                    // attributed to the command that started it, and the
                    // ones folded into it are dropped exactly like the
                    // pending queue behind them.
                    self.fail_command(ex.index, ex.addr, e).await;
                }
            }
        }
    }

    /// A queued command finished with an error: latch it (attributed),
    /// clear the pending queue (later commands must not run from an
    /// unexpected position), and push COMPLETE(ok=false).
    async fn fail_command(&mut self, index: u64, addr: SocketAddr, mut e: WireError) {
        e.command_index = index as i64;
        self.completed_index = self.completed_index.max(index as i64);
        self.standing_error = Some(e.clone());
        self.action_state = ActionState::Error;
        self.pending.clear();
        self.blend_hold = None;
        self.push_complete(addr, index, Some(e)).await;
    }

    /// stop scope: active motion (planned + streaming) halts; the
    /// pending queue is untouched.
    fn cancel_active_motion(&mut self) {
        self.runtime.planner.cancel();
        if self.executing.take().is_some() {
            self.action_state = ActionState::Idle;
        }
        if self.active_stream.take().is_some() {
            self.runtime.rt.cancel_stream();
        }
        self.runtime.rt.halt();
    }

    /// estop / reset_state / simulator-toggle scope: everything.
    fn cancel_all_motion(&mut self) {
        self.cancel_active_motion();
        self.pending.clear();
        self.blend_hold = None;
    }

    /// A streamable arrived: planned motion (active AND pending) is
    /// cancelled — the queued program must not resume from wherever a
    /// manual jog left the arm.
    fn cancel_planned(&mut self) {
        self.runtime.planner.cancel();
        if self.executing.take().is_some() {
            self.action_state = ActionState::Idle;
        }
        self.pending.clear();
        self.blend_hold = None;
    }

    /// A refused fire-and-forget command answered its sender with ERROR,
    /// but nothing awaits that datagram — so the refusal is ALSO latched
    /// as the standing error, reaching STATUS, `activity` and the ERROR
    /// query, the way parol6 surfaces pipeline failures through
    /// `state.error`. Without this, an idle arm refusing every jog or
    /// teleport reads healthy while nothing moves.
    ///
    /// Latched only while the pipeline is idle and nothing truer stands:
    /// - an ATTRIBUTED standing error (a queued command's failure) and
    ///   the RT's own latch describe the arm and outrank a refusal;
    /// - with motion in flight (executing / pending / streaming) a stray
    ///   refusal must not fail the running program's `wait_command`
    ///   through the stale-error ordering rule — the ERROR reply alone
    ///   serves there.
    ///
    /// Cleared like every standing error: by the next ACCEPTED motion
    /// command ([`Self::on_motion_accepted`]) or by `reset`/`reset_state`.
    fn latch_faf_refusal(&mut self, error: &WireError) {
        let busy =
            self.executing.is_some() || !self.pending.is_empty() || self.active_stream.is_some();
        let attributed = self
            .standing_error
            .as_ref()
            .is_some_and(|e| e.command_index != UNATTRIBUTED);
        if busy || attributed || rt_standing_error(&self.snap).is_some() {
            return;
        }
        self.standing_error = Some(error.clone());
    }

    /// A motion command was accepted: the previous attempt's verdicts stop
    /// being the current state of the world. The standing error clears,
    /// and so does the latched collision report — its pairs described the
    /// configuration a superseded motion was blocked at.
    fn on_motion_accepted(&mut self) {
        if self.standing_error.take().is_some() && self.action_state == ActionState::Error {
            self.action_state = ActionState::Idle;
        }
        self.runtime.planner.clear_collision();
        self.runtime.rt.clear_collision();
        self.collision = CollisionState::default();
    }

    /// Preemption drain: pull the datagrams already queued behind the
    /// streamable that just preempted `superseded`, and discard the ones
    /// belonging to that stream — they are setpoints the client has
    /// itself replaced, and replaying them would flip the active stream
    /// type straight back.
    ///
    /// Everything else is put on [`Self::deferred`] and dispatched
    /// normally. This socket is the ONE command socket: every client and
    /// every command class arrives on it, so a blind drain also destroys
    /// a buffered `estop` — with no reply, no effect, and a `_system`
    /// send that does not retry. Preemption discards only what it is
    /// entitled to discard (the wire contract: "cancels the active
    /// streamable, drains the socket backlog" — the previous stream's
    /// backlog).
    ///
    /// Bounded per call: what stays in the kernel queue is read by the
    /// normal event loop on the next turn, so a peer that streams faster
    /// than the server drains cannot hold it in here.
    fn drain_stream_backlog(&mut self, superseded: CmdType) {
        let (mut dropped, mut kept) = (0usize, 0usize);
        for _ in 0..DRAIN_LIMIT {
            let Ok((n, from)) = self.socket.try_recv_from(&mut self.drainbuf) else {
                break;
            };
            let data = &self.drainbuf[..n];
            if peek_tag(data).is_ok_and(|t| CmdType::from_wire(t) == Some(superseded)) {
                dropped += 1;
                continue;
            }
            self.deferred.push_back((data.to_vec(), from));
            kept += 1;
        }
        if dropped > 0 || kept > 0 {
            log::debug!(
                "stream preemption: dropped {dropped} stale {superseded:?}, \
                 deferred {kept} other datagram(s)"
            );
        }
    }

    // ---- state assembly ----------------------------------------------------

    fn refresh_snapshot(&mut self) {
        if let Some(s) = self.runtime.snapshots.take() {
            self.snap = s;
            self.last_fresh = Some(Instant::now());
        }
    }

    /// Age of the freshest MOTOR-BUS data \[ms, saturating\]: the youngest
    /// node age the RT snapshot carries (ticks → ms) plus the wall age of
    /// the snapshot itself. `u16::MAX` = no node has ever answered — the
    /// bus analogue of parol6's `first_frame_received == false`, so a
    /// runtime that has never heard a driver cannot report a healthy link.
    ///
    /// The RT publishes a snapshot every tick whether or not the bus
    /// spoke, so the snapshot's own wall age alone reads fresh over a
    /// silent bus; the per-node `data_age_ticks` in the same snapshot is
    /// the signal the wire doc actually promises ("motor bus link",
    /// `par6-proto/src/status.rs`). The snapshot wall age still
    /// contributes so a dead RT thread degrades the reading too.
    fn data_age_ms(&self) -> u16 {
        let Some(fresh) = self.last_fresh else {
            return u16::MAX;
        };
        let bus_ticks = self
            .snap
            .nodes
            .iter()
            .map(|n| n.data_age_ticks)
            .min()
            .unwrap_or(u64::MAX);
        if bus_ticks == u64::MAX {
            return u16::MAX;
        }
        let tick_ms = 1000.0 / self.cfg.rt_tick_rate_hz.max(1.0);
        let bus_ms = (bus_ticks as f64 * tick_ms).min(f64::from(u16::MAX)) as u128;
        (bus_ms + fresh.elapsed().as_millis()).min(u128::from(u16::MAX)) as u16
    }

    /// Whether the motor bus is live: some node answered within the
    /// staleness window. Feeds STATUS `link_ok` and (with `!simulator`)
    /// the PING query's `hardware_connected`.
    fn link_ok(&self) -> bool {
        u128::from(self.data_age_ms()) <= self.cfg.link_stale.as_millis()
    }

    fn mono_ns(&self) -> u64 {
        self.start.elapsed().as_nanos().min(u128::from(u64::MAX)) as u64
    }

    fn sync_planner(&mut self) {
        let ctx = PlanContext {
            profile: &self.profile,
            tool: &self.tool,
            tool_variant: self.tool_variant.as_deref(),
            tcp_offset_mm: self.tcp_offset_mm,
            completion_policy: self.completion_policy,
            payload: self.payload,
        };
        self.runtime.planner.sync(ctx);
    }

    /// Push the configured installation keep-outs into the planner's
    /// collision world. Called once, before the server task starts.
    fn install_shapes(&mut self) -> Result<(), WireError> {
        if self.cfg.installation_shapes.is_empty() {
            return Ok(());
        }
        let shapes = self.cfg.installation_shapes.clone();
        if let Some(epoch) = self
            .runtime
            .planner
            .set_shapes(ShapeLayer::Installation, &shapes)?
        {
            self.scene_epoch = epoch;
        }
        // The streaming gate enforces the same world the planner does.
        self.runtime
            .rt
            .set_shapes(ShapeLayer::Installation, &shapes)?;
        Ok(())
    }

    /// Replace the program layer: the planner enforces it, the server
    /// stores the copy the SHAPES query reads back, and the epoch of the
    /// APPLIED world becomes the reported one. A refusal changes
    /// nothing — neither the enforced world, nor the readback, nor the
    /// epoch — so a client that sees the epoch move knows the shapes it
    /// sent are the shapes being enforced.
    fn apply_program_shapes(&mut self, shapes: Vec<par6_proto::Shape>) -> Result<(), WireError> {
        match self
            .runtime
            .planner
            .set_shapes(ShapeLayer::Program, &shapes)?
        {
            Some(epoch) => self.scene_epoch = epoch,
            // No collision world to adopt an epoch from: the server's own
            // counter still has to move, or a readback cannot be tied to
            // the world it describes.
            None => self.scene_epoch += 1,
        }
        // Mirror into the streaming gate. The planner accepted this set,
        // and the gate converts through the identical path, so a refusal
        // here is a wiring defect — surfaced, because a jog gated against
        // a STALE world is worse than a loud error.
        self.runtime.rt.set_shapes(ShapeLayer::Program, &shapes)?;
        self.shapes = shapes;
        Ok(())
    }

    /// Refresh the STATUS collision fields from the latched verdicts.
    ///
    /// Two gates latch one: the planner's (a refused or invalidated
    /// planned move) and the streaming gate's (a refused or stopped
    /// jog/servo). At most one motion pipeline is active at a time and
    /// accepting a motion clears both, so they never disagree — the
    /// merge simply reports whichever is active.
    fn update_collision(&mut self) {
        let stream = self.runtime.rt.collision().filter(|s| s.active);
        if let Some(state) = self.runtime.planner.collision() {
            self.collision = if state.active {
                state
            } else {
                stream.unwrap_or(state)
            };
        } else if let Some(state) = stream {
            self.collision = state;
        }
    }

    fn update_tcp_speed(&mut self) {
        let now = Instant::now();
        let pos = [self.snap.tcp[0], self.snap.tcp[1], self.snap.tcp[2]];
        if let Some((prev, t)) = self.prev_tcp {
            let dt = now.duration_since(t).as_secs_f64();
            if dt > 0.0 {
                let d = ((pos[0] - prev[0]).powi(2)
                    + (pos[1] - prev[1]).powi(2)
                    + (pos[2] - prev[2]).powi(2))
                .sqrt();
                self.tcp_speed = d * 1000.0 / dt;
            }
        }
        self.prev_tcp = Some((pos, now));
    }

    fn estop_pressed(&self) -> bool {
        use par6_rt::ErrorCode as RtErr;
        self.estop_latched
            || self
                .snap
                .errors
                .as_slice()
                .iter()
                .any(|e| matches!(e.code, RtErr::Estop | RtErr::SwEstop))
    }

    /// `inputs ++ outputs ++ [estop]`, in `[io]` config order — the
    /// declared lines as the RT thread last read and drove them, with
    /// the e-stop always last.
    ///
    /// The e-stop slot is not a line level: it is the whole e-stop
    /// CONDITION, hardware chain and software flag together, which is
    /// what an operator watching this array needs to see.
    fn io(&self) -> Vec<u8> {
        let inputs = self.snap.io_input_levels();
        let outputs = self.snap.io_output_levels();
        let mut io = Vec::with_capacity(inputs.len() + outputs.len() + 1);
        io.extend_from_slice(inputs);
        io.extend_from_slice(outputs);
        io.push(u8::from(!self.estop_pressed()));
        io
    }

    fn angles_deg(&self) -> [f64; NUM_JOINTS] {
        let mut a = [0.0; NUM_JOINTS];
        for (out, q) in a.iter_mut().zip(self.snap.q.iter()) {
            *out = q.to_degrees();
        }
        a
    }

    fn stream_shown(&self) -> Option<&'static str> {
        match self.active_stream {
            Some(tag) if matches!(self.snap.mode, Mode::Jog | Mode::Stream) => Some(cmd_name(tag)),
            _ => None,
        }
    }

    /// The standing error as a client sees it: the one attributed to a
    /// queued command if there is one, otherwise whatever the RT has
    /// latched on its own. The RT latch is DERIVED, never stored — it
    /// stops being reported exactly when the RT stops latching it, so
    /// accepting a new motion command cannot clear an error the arm is
    /// still bricked by.
    fn effective_error(&self) -> Option<WireError> {
        self.standing_error
            .clone()
            .or_else(|| rt_standing_error(&self.snap))
    }

    fn action_fields(&self) -> (String, ActionState, String) {
        if let Some(ex) = &self.executing {
            (
                ex.name.to_owned(),
                ActionState::Executing,
                ex.params.clone(),
            )
        } else if let Some(name) = self.stream_shown() {
            (name.to_owned(), ActionState::Executing, String::new())
        } else if self.effective_error().is_some() {
            // Nothing is running and an error stands: the action state is
            // the error, whether a command earned it or the RT latched it.
            (String::new(), ActionState::Error, String::new())
        } else {
            (String::new(), self.action_state, String::new())
        }
    }

    fn build_tool_status(&self) -> Option<ToolStatusWire> {
        if self.tool.is_empty() {
            return None;
        }
        let g = &self.snap.gripper;
        let (state, engaged, part_detected, fault_code, positions, channels) = match g.reply {
            Some(r) => {
                let fault_code = gripper_fault_code(&self.snap);
                let od = r.object_detection as u8;
                let state = if fault_code != 0 {
                    ToolState::Error
                } else if r.action_status {
                    ToolState::Active
                } else if r.activated {
                    ToolState::Idle
                } else {
                    ToolState::Off
                };
                (
                    state,
                    r.activated && od == 1,
                    od == 1 || od == 2,
                    fault_code,
                    // Wire units for a tool DOF are 0 (open) … 1 (closed);
                    // the firmware reports a 0..255 jaw byte.
                    vec![f64::from(r.position) / 255.0],
                    vec![f64::from(r.current_ma)],
                )
            }
            None => (ToolState::Off, false, false, 0, Vec::new(), Vec::new()),
        };
        Some(ToolStatusWire {
            key: self.tool.clone(),
            state,
            engaged,
            part_detected,
            fault_code,
            positions,
            channels,
            variant_key: self.tool_variant.clone().unwrap_or_default(),
        })
    }

    /// The queue ETA a client reads: what the motion in flight has left
    /// to run plus the planner's estimate of everything still queued.
    fn queued_duration(&self) -> f64 {
        self.runtime.planner.inflight_duration(&self.snap) + self.queue_estimate
    }

    /// Re-estimate the pending queue when its contents changed.
    ///
    /// Estimating means planning, so it must not run at the status
    /// cadence. Queue indices are allocated in order and commands only
    /// ever leave from the front, so the pending queue is a contiguous
    /// index range and `(front, len)` names it exactly.
    fn refresh_queue_estimate(&mut self) {
        let key = (
            self.pending.front().map_or(0, |p| p.index),
            self.pending.len(),
        );
        if key == self.queue_estimate_for {
            return;
        }
        let batch: Vec<QueuedCommand<'_>> = self
            .pending
            .iter()
            .map(|p| QueuedCommand {
                index: p.index,
                cmd: &p.cmd,
            })
            .collect();
        self.queue_estimate = self.runtime.planner.queued_duration(&batch);
        self.queue_estimate_for = key;
    }

    /// Enablement as clients see it. The planner owns the model; a runtime
    /// built without kinematics has none to own for the Cartesian axes, and
    /// it already refuses every Cartesian command — so it reports no
    /// Cartesian freedom instead of full freedom. The wire slots are 0/1
    /// with no "unknown" spelling, and 1 means "you may move that way",
    /// which is the one thing such a runtime knows to be false.
    ///
    /// The RT's live jog latch is folded in on every read: the jog engine
    /// blocks a direction at its jerk-aware brake-at-limits bound, which
    /// at jog speed latches well before the static soft-limit margin the
    /// planner's probe applies — so mid-jog, the flags grey the direction
    /// the tick the RT actually stops honoring it.
    fn enablement(&self) -> Enablement {
        let mut en = self.runtime.planner.enablement();
        if !self.cfg.cartesian {
            en.cart_en_wrf = [0; EN_SLOTS];
            en.cart_en_trf = [0; EN_SLOTS];
        }
        // Only while the jog is live: the latch is the jog engine's
        // state, and once the mode ends it describes a jog that no
        // longer exists — the static probe rules again.
        if self.snap.mode == Mode::Jog {
            let mask = self.snap.jog.blocked_mask;
            for j in 0..EN_SLOTS / 2 {
                // Wire order is [j+, j−, …]; the RT mask carries a blocked
                // negative direction in bit 2j and positive in bit 2j+1.
                if mask & (2 << (2 * j)) != 0 {
                    en.joint_en[2 * j] = 0;
                }
                if mask & (1 << (2 * j)) != 0 {
                    en.joint_en[2 * j + 1] = 0;
                }
            }
        }
        en
    }

    fn build_status(&self) -> Status {
        let en = self.enablement();
        let (action_current, action_state, action_params) = self.action_fields();
        Status {
            proto_version: PROTO_VERSION,
            controller_id: self.cfg.controller_id,
            seq: self.status_seq,
            mono_time_ns: self.mono_ns(),
            link_ok: u8::from(self.link_ok()),
            data_age_ms: self.data_age_ms(),
            pose: pose_matrix_mm(&self.snap.tcp),
            angles: self.angles_deg(),
            speeds: self.snap.qd,
            io: self.io(),
            action_current,
            action_state,
            joint_en: en.joint_en,
            cart_en_wrf: en.cart_en_wrf,
            cart_en_trf: en.cart_en_trf,
            executing_index: self.executing.as_ref().map_or(-1, |e| e.index as i64),
            completed_index: self.completed_index,
            last_checkpoint: self.last_checkpoint.clone(),
            error: self.effective_error(),
            queued_segments: self.pending.len() as u32,
            queued_duration: self.queued_duration(),
            action_params,
            tool_status: self.build_tool_status(),
            tcp_speed: self.tcp_speed,
            simulator_active: self.simulator,
            collision_active: self.collision.active,
            collision_pairs: self.collision.pairs.clone(),
            scene_epoch: self.scene_epoch,
            accepted_index: self.accepted_index,
            homed: self.snap.homed,
            // Filtered, not raw: this is an operator readout, and the raw
            // per-tick current estimate is noisy.
            torques: {
                let mut out = [0.0; par6_proto::NUM_JOINTS];
                out.copy_from_slice(&self.snap.tau_filtered[..par6_proto::NUM_JOINTS]);
                out
            },
            mode: Self::wire_mode(self.snap.mode),
            enabled: self.snap.state == ArmState::Enabled,
            gravity_comp: self.snap.gravity_comp,
            warnings: crate::faults::rt_warnings(&self.snap),
            link_health: Self::wire_link_health(&self.snap.link),
            homing: Self::wire_homing(&self.snap.homing),
            torques_ext: {
                let mut out = [0.0; par6_proto::NUM_JOINTS];
                out.copy_from_slice(&self.snap.tau_ext[..par6_proto::NUM_JOINTS]);
                out
            },
        }
    }

    /// The bus link health in the wire's own vocabulary (exhaustive for
    /// the same reason as [`Self::wire_mode`]).
    fn wire_link_health(l: &par6_rt::LinkHealth) -> par6_proto::LinkHealthWire {
        use par6_proto::LinkState as W;
        use par6_rt::LinkState as R;
        let state = match l.state {
            R::Unknown => W::Unknown,
            R::Up => W::Up,
            R::ErrorPassive => W::ErrorPassive,
            R::BusOff => W::BusOff,
        };
        par6_proto::LinkHealthWire {
            state: state as u8,
            restarts: l.restarts,
            tx_errors: l.tx_errors,
            rx_frames: l.rx_frames,
        }
    }

    fn wire_homing(h: &par6_rt::HomingStatus) -> par6_proto::HomingWire {
        use par6_proto::HomingJointState as WS;
        use par6_proto::HomingPhase as WP;
        use par6_rt::{HomingJointStatus as RS, HomingPhase as RP};
        let joints = h
            .per_joint
            .iter()
            .zip(h.phase.iter())
            .map(|(status, phase)| {
                let ws = match status {
                    RS::Idle => WS::Idle,
                    RS::Running => WS::Running,
                    RS::Done => WS::Done,
                    RS::Failed => WS::Failed,
                };
                let wp = match phase {
                    RP::Idle => WP::Idle,
                    RP::Approach => WP::Approach,
                    RP::Dwell => WP::Dwell,
                    RP::Backoff => WP::Backoff,
                    RP::Pause => WP::Pause,
                    RP::Release => WP::Release,
                    RP::Settle => WP::Settle,
                    RP::PostMove => WP::PostMove,
                    RP::Finished => WP::Finished,
                };
                (ws as u8, wp as u8)
            })
            .collect();
        par6_proto::HomingWire {
            active: h.active,
            sequence_step: h.sequence_step,
            joints,
        }
    }

    /// The RT core's mode in the wire's own vocabulary.
    ///
    /// Exhaustive on purpose: `par6-rt` does not depend on `par6-proto`, so a
    /// new RT mode has to be given a wire meaning here rather than silently
    /// riding a discriminant.
    fn wire_mode(mode: Mode) -> ControllerMode {
        match mode {
            Mode::Booting => ControllerMode::Booting,
            Mode::Idle => ControllerMode::Idle,
            Mode::ActiveError => ControllerMode::ActiveError,
            Mode::Homing => ControllerMode::Homing,
            Mode::Jog => ControllerMode::Jog,
            Mode::Stream => ControllerMode::Stream,
            Mode::Exec => ControllerMode::Exec,
            Mode::HandGuiding => ControllerMode::HandGuiding,
            Mode::Impedance => ControllerMode::Impedance,
            Mode::SafetyStop => ControllerMode::SafetyStop,
            Mode::Flashing => ControllerMode::Flashing,
        }
    }

    fn query_result(&self, cmd: &Command) -> QueryResult {
        use Command as C;
        match cmd {
            C::Ping => QueryResult::Ping {
                hardware_connected: !self.simulator && self.link_ok(),
            },
            C::Status => QueryResult::Status {
                pose: pose_matrix_mm(&self.snap.tcp),
                angles: self.angles_deg(),
                speeds: self.snap.qd,
                io: self.io(),
                tool_status: self.build_tool_status(),
            },
            C::Angles => QueryResult::Angles {
                angles: self.angles_deg(),
            },
            C::Pose(q) => QueryResult::Pose {
                pose: match q.frame {
                    // The WORLD expressed in the tool frame — the tool
                    // pose read the other way round. (The TCP in its own
                    // frame is the identity, which is true of every arm
                    // in every configuration and tells a client nothing.)
                    Some(par6_proto::Frame::Trf) => world_in_tool_mm(&self.snap.tcp),
                    _ => pose_matrix_mm(&self.snap.tcp),
                },
            },
            C::Io => QueryResult::Io { io: self.io() },
            C::Speeds => QueryResult::Speeds {
                speeds: self.snap.qd,
            },
            C::Tools => QueryResult::Tools {
                tool: self.tool.clone(),
                available: self.cfg.tools.clone(),
            },
            C::Queue => QueryResult::Queue {
                queue: self
                    .pending
                    .iter()
                    .map(|p| cmd_name(p.cmd.tag()).to_owned())
                    .collect(),
                executing_index: self.executing.as_ref().map_or(-1, |e| e.index as i64),
                completed_index: self.completed_index,
                last_checkpoint: self.last_checkpoint.clone(),
                queued_duration: self.queued_duration(),
            },
            C::Activity => {
                let (current, state, params) = self.action_fields();
                QueryResult::Activity {
                    current,
                    state,
                    next: self
                        .pending
                        .front()
                        .map(|p| cmd_name(p.cmd.tag()).to_owned())
                        .unwrap_or_default(),
                    params,
                }
            }
            C::LoopStats => {
                let ls = &self.snap.loop_stats;
                // Every published number comes off the RT's rolling
                // window, except the mean: that one is the EMA the RT
                // updates every tick, which tracks the live loop rather
                // than the window's last summary.
                QueryResult::LoopStats(LoopStatsResult {
                    target_hz: self.cfg.rt_tick_rate_hz,
                    loop_count: self.snap.tick,
                    overrun_count: u64::from(ls.overruns),
                    mean_period_s: ls.period_ema_s,
                    std_period_s: ls.std_s,
                    min_period_s: ls.min_s,
                    max_period_s: ls.max_s,
                    p95_period_s: ls.p95_s,
                    p99_period_s: ls.p99_s,
                    mean_hz: if ls.period_ema_s > 0.0 {
                        1.0 / ls.period_ema_s
                    } else {
                        0.0
                    },
                    p50_period_s: ls.p50_s,
                    p90_period_s: ls.p90_s,
                    can_frame_age_min_ticks: ls.can_frame_age_min_ticks,
                    can_frame_age_max_ticks: ls.can_frame_age_max_ticks,
                    rt_fifo: ls.rt_fifo,
                    rt_pinned: ls.rt_pinned,
                })
            }
            C::Profile => QueryResult::Profile {
                profile: self.profile.clone(),
            },
            C::Reachable => {
                let en = self.enablement();
                QueryResult::Reachable {
                    joint_en: en.joint_en,
                    cart_en_wrf: en.cart_en_wrf,
                    cart_en_trf: en.cart_en_trf,
                }
            }
            C::Error => QueryResult::Error {
                error: self.effective_error(),
            },
            C::TcpSpeed => QueryResult::TcpSpeed {
                speed: self.tcp_speed,
            },
            C::TcpOffset => QueryResult::TcpOffset {
                x: self.tcp_offset_mm[0],
                y: self.tcp_offset_mm[1],
                z: self.tcp_offset_mm[2],
            },
            C::ToolStatus => QueryResult::ToolStatus {
                tool_status: self.build_tool_status(),
            },
            C::IsSimulator => QueryResult::IsSimulator {
                active: self.simulator,
            },
            C::Payload => QueryResult::Payload {
                mass: self.payload.mass,
                com: self.payload.com,
                inertia: self.payload.inertia.unwrap_or_default(),
            },
            C::ConfigInfo => {
                let ci = &self.cfg.config_info;
                QueryResult::ConfigInfo {
                    path: ci.path.clone(),
                    fingerprint: ci.fingerprint.clone(),
                    tick_dt_s: ci.tick_dt_s,
                    motion: ci.motion,
                    joints: ci.joints.clone(),
                }
            }
            C::Shapes => QueryResult::Shapes {
                installation: self.cfg.installation_shapes.clone(),
                program: self.shapes.clone(),
                epoch: self.scene_epoch,
            },
            _ => unreachable!("dispatch routes only QUERY commands here"),
        }
    }

    // ---- wire I/O ----------------------------------------------------------

    async fn reply(&mut self, addr: SocketAddr, reply: &Reply) {
        encode_reply(reply, &mut self.txbuf);
        if let Err(e) = self.socket.send_to(&self.txbuf, addr).await {
            log::debug!("reply send to {addr} failed: {e}");
        }
    }

    async fn push_complete(&mut self, addr: SocketAddr, index: u64, detail: Option<WireError>) {
        let reply = Reply::Complete {
            index,
            ok: detail.is_none(),
            detail,
        };
        self.reply(addr, &reply).await;
    }
}

// ---- free helpers ----------------------------------------------------------

fn rate_period(hz: u32) -> std::time::Duration {
    std::time::Duration::from_secs_f64(1.0 / f64::from(hz.max(1)))
}

/// 4×4 row-major pose matrix (translation in mm) from the snapshot's
/// `[x, y, z (m), roll, pitch, yaw (rad)]` TCP, with `R = Rx·Ry·Rz` —
/// the wire's intrinsic-XYZ rotation convention,
/// the exact composition the runtime's rpy extraction inverts.
fn pose_matrix_mm(tcp: &[f64; 6]) -> [f64; POSE_ELEMS] {
    par6_proto::pose_matrix(
        [tcp[0] * 1000.0, tcp[1] * 1000.0, tcp[2] * 1000.0],
        [tcp[3], tcp[4], tcp[5]],
    )
}

/// The world origin expressed in the tool frame: the inverse of
/// [`pose_matrix_mm`]'s transform, translation in mm.
///
/// Inverting a rigid transform is `Rᵀ` and `−Rᵀ·t`; the inversion runs in
/// metres and the result is scaled back to mm, so it composes with the
/// WRF matrix exactly (parol6 inverts the SE(3) and scales after, same
/// order).
fn world_in_tool_mm(tcp: &[f64; 6]) -> [f64; POSE_ELEMS] {
    let m = pose_matrix_mm(tcp);
    let t_m = [tcp[0], tcp[1], tcp[2]];
    let mut out = [0.0; POSE_ELEMS];
    for r in 0..3 {
        for c in 0..3 {
            out[r * 4 + c] = m[c * 4 + r];
        }
        let t = -(0..3).map(|k| m[k * 4 + r] * t_m[k]).sum::<f64>();
        out[r * 4 + 3] = t * 1000.0;
    }
    out[15] = 1.0;
    out
}

/// The first `teleport` angle the runtime cannot honour, described in
/// the terms a client can act on: which joint, what it asked for, and
/// the window it has. `None` = every angle is placeable.
fn teleport_angle_fault(angles: &[f64; NUM_JOINTS], cfg: &ServerConfig) -> Option<String> {
    // Finiteness belongs to the codec (`par6-proto` rejects NaN/inf at
    // decode), so only the travel window is left to check here.
    for (i, (&a, &(lo, hi))) in angles
        .iter()
        .zip(cfg.joint_hard_limits_deg.iter())
        .enumerate()
    {
        if a < lo || a > hi {
            return Some(format!(
                "angles[{i}] = {a:.3} deg is outside joint {i}'s travel \
                 [{lo:.3}, {hi:.3}] deg; teleport places the arm exactly \
                 where it is told or not at all"
            ));
        }
    }
    None
}

fn post_effect(cmd: &Command) -> PostEffect {
    match cmd {
        Command::Checkpoint(p) => PostEffect::Checkpoint(p.label.clone()),
        Command::SelectTool(p) => PostEffect::SelectVariant(p.variant_key.clone()),
        _ => PostEffect::None,
    }
}

fn params_summary(cmd: &Command) -> String {
    let mut s = format!("{cmd:?}");
    if s.len() > MAX_PARAMS_LEN {
        let cut = (0..=MAX_PARAMS_LEN)
            .rev()
            .find(|i| s.is_char_boundary(*i))
            .unwrap_or(0);
        s.truncate(cut);
    }
    s
}

/// Wire name of a command (STATUS `action_current`, QUEUE listing).
fn cmd_name(tag: CmdType) -> &'static str {
    use CmdType as T;
    match tag {
        T::Reset => "reset",
        T::Estop => "estop",
        T::SetGravityComp => "set_gravity_comp",
        T::Pause => "pause",
        T::Stop => "stop",
        T::WriteIo => "write_io",
        T::Simulator => "simulator",
        T::SelectProfile => "select_profile",
        T::ResetState => "reset_state",
        T::ConnectHardware => "connect_hardware",
        T::SetTcpOffset => "set_tcp_offset",
        T::SetPayload => "set_payload",
        T::SetShapes => "set_shapes",
        T::SetCompletionPolicy => "set_completion_policy",
        T::SetRecipe => "set_recipe",
        T::Ping => "ping",
        T::Status => "status",
        T::Angles => "angles",
        T::Pose => "pose",
        T::Io => "io",
        T::Speeds => "speeds",
        T::Tools => "tools",
        T::Queue => "queue",
        T::Activity => "activity",
        T::LoopStats => "loop_stats",
        T::Profile => "profile",
        T::Reachable => "reachable",
        T::Error => "error",
        T::TcpSpeed => "tcp_speed",
        T::TcpOffset => "tcp_offset",
        T::ToolStatus => "tool_status",
        T::IsSimulator => "is_simulator",
        T::Shapes => "shapes",
        T::ConfigInfo => "config_info",
        T::Payload => "payload",
        T::ServoJ => "servo_j",
        T::ServoJPose => "servo_j_pose",
        T::ServoL => "servo_l",
        T::JogJ => "jog_j",
        T::JogL => "jog_l",
        T::Teleport => "teleport",
        T::ResetLoopStats => "reset_loop_stats",
        T::Home => "home",
        T::MoveJ => "move_j",
        T::MoveJPose => "move_j_pose",
        T::MoveL => "move_l",
        T::MoveC => "move_c",
        T::MoveS => "move_s",
        T::MoveP => "move_p",
        T::SelectTool => "select_tool",
        T::Delay => "delay",
        T::Checkpoint => "checkpoint",
        T::ToolAction => "tool_action",
    }
}

/// The server's decode-failure answer: validation failures map to
/// `CommValidationError`, unknown tags to `CommUnknownCommand`, the rest
/// to `CommDecodeError`. Public so an offline preview refuses exactly
/// what the wire refuses.
pub fn decode_error_to_wire(e: &DecodeError) -> WireError {
    let code = match e {
        DecodeError::Validation { .. } => ErrorCode::CommValidationError,
        DecodeError::UnknownTag(_) => ErrorCode::CommUnknownCommand,
        _ => ErrorCode::CommDecodeError,
    };
    make_error(code, UNATTRIBUTED, &[("detail", &e.to_string())])
}
