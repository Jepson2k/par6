//! The command-plane actor: one tokio task owning the UDP command
//! socket, the motion queue, the SINGLE command-index allocator, and the
//! status/telemetry broadcast schedule.
//!
//! Decisions resolved here (see `spec/PROTOCOL-V2.md`):
//!
//! - The index allocator is monotonic and NEVER reset — not even by
//!   `reset_state` — so a stale pre-reset status frame can never satisfy
//!   a post-reset wait.
//! - Gating rejections always answer with ERROR (echoed `req_id`),
//!   including FIRE_AND_FORGET commands whose success stays unacked.
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
    CmdType, Command, CommandClass, CompletionPolicy, DecodeError, ErrorCode, LoopStatsResult,
    MsgType, QueryResult, Reassembler, Reply, Status, StatusEncoder, ToolState, ToolStatusWire,
    WireError, IO_SLOTS, NUM_JOINTS, POSE_ELEMS, PROTO_VERSION, UNATTRIBUTED,
};
use par6_rt::{ArmState, Mode, StateSnapshot};
use tokio::net::UdpSocket;
use tokio::sync::Notify;
use tokio::time::MissedTickBehavior;

use crate::config::ServerConfig;
use crate::gating::{gate, is_stream};
use crate::link::BroadcastLink;
use crate::runtime::{PlanContext, Planner, RtCommands, RuntimeHandle};
use crate::telemetry;

/// Cap on the `action_params` summary string.
const MAX_PARAMS_LEN: usize = 100;

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
    let socket = UdpSocket::bind(cfg.bind).await?;
    let addr = socket.local_addr()?;
    let link = BroadcastLink::open(&cfg).await?;
    let shutdown = Arc::new(Notify::new());
    let stop = shutdown.clone();
    let core = Core::new(cfg, runtime, socket, link);
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
    SelectTool {
        tool: String,
        variant: Option<String>,
    },
}

struct Executing {
    index: u64,
    addr: SocketAddr,
    name: &'static str,
    params: String,
    effect: PostEffect,
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
    accepted_index: i64,
    completed_index: i64,
    last_checkpoint: String,
    standing_error: Option<WireError>,
    action_state: ActionState,
    estop_latched: bool,
    active_stream: Option<CmdType>,

    profile: String,
    tool: String,
    tool_variant: Option<String>,
    tcp_offset_mm: [f64; 3],
    shapes: Vec<par6_proto::Shape>,
    scene_epoch: u64,
    completion_policy: CompletionPolicy,
    recipe: Option<String>,
    simulator: bool,
    io_out: [u8; 2],

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
            accepted_index: -1,
            completed_index: -1,
            last_checkpoint: String::new(),
            standing_error: None,
            action_state: ActionState::Idle,
            estop_latched: false,
            active_stream: None,
            tool: String::new(),
            tool_variant: None,
            tcp_offset_mm: [0.0; 3],
            shapes: Vec::new(),
            scene_epoch: 0,
            completion_policy: CompletionPolicy::Settled,
            io_out: [0; 2],
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
                Event::Datagram(n, addr) => self.on_datagram(&rxbuf[..n], addr).await,
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
                let req_id = peek_req_id(data).unwrap_or(0);
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
                let req_id = peek_req_id(data).unwrap_or(0);
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
        self.expire_chunks().await;
        self.collect_outcomes().await;
        self.pump().await;
    }

    async fn on_status(&mut self) {
        self.refresh_snapshot();
        self.update_tcp_speed();
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
        let result = self.query_result(cmd);
        self.reply(addr, &Reply::Response { req_id, result }).await;
    }

    async fn on_system(&mut self, req_id: u32, cmd: &Command, addr: SocketAddr) {
        use Command as C;
        let result: Result<(), WireError> = match cmd {
            C::Reset => {
                self.estop_latched = false;
                self.standing_error = None;
                self.action_state = ActionState::Idle;
                self.runtime.rt.set_enabled(true);
                Ok(())
            }
            C::Estop => {
                self.cancel_all_motion();
                self.estop_latched = true;
                self.runtime.rt.set_enabled(false);
                self.standing_error =
                    Some(make_error(ErrorCode::SysEstopActive, UNATTRIBUTED, &[]));
                Ok(())
            }
            C::Stop(p) => {
                self.cancel_active_motion();
                if p.clear_queue {
                    self.pending.clear();
                }
                Ok(())
            }
            C::WriteIo(p) => {
                self.runtime.rt.write_io(p.port, p.value);
                if usize::from(p.port) < self.io_out.len() {
                    self.io_out[usize::from(p.port)] = p.value;
                }
                Ok(())
            }
            C::Simulator(p) => {
                self.cancel_all_motion();
                self.runtime.rt.set_simulator(p.on).map(|()| {
                    self.simulator = p.on;
                })
            }
            C::SelectProfile(p) => {
                if self.cfg.profiles.iter().any(|x| x == &p.profile) {
                    self.profile = p.profile.clone();
                    self.sync_planner();
                    Ok(())
                } else {
                    Err(make_error(
                        ErrorCode::SysProfileInvalid,
                        UNATTRIBUTED,
                        &[("detail", &p.profile)],
                    ))
                }
            }
            C::ResetState => {
                self.cancel_all_motion();
                self.standing_error = None;
                self.action_state = ActionState::Idle;
                self.last_checkpoint.clear();
                self.tool.clear();
                self.tool_variant = None;
                self.tcp_offset_mm = [0.0; 3];
                self.shapes.clear();
                self.scene_epoch += 1;
                self.completion_policy = CompletionPolicy::Settled;
                self.profile = self.cfg.initial_profile.clone();
                self.recipe = self.cfg.initial_recipe.clone();
                self.runtime.rt.reset_state();
                self.sync_planner();
                Ok(())
            }
            C::ConnectHardware(p) => self.runtime.rt.connect_hardware(&p.port),
            C::SetTcpOffset(p) => {
                self.tcp_offset_mm = [p.x, p.y, p.z];
                self.sync_planner();
                Ok(())
            }
            C::SetShapes(p) => {
                self.shapes = p.shapes.clone();
                self.scene_epoch += 1;
                self.sync_planner();
                Ok(())
            }
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

    async fn on_faf(&mut self, req_id: u32, cmd: Command, addr: SocketAddr) {
        let tag = cmd.tag();
        if tag == CmdType::ResetLoopStats {
            self.runtime.rt.reset_loop_stats();
            return;
        }
        if let Some(error) = self.check_gate(tag) {
            // Rejection gets a real ERROR even though success is unacked.
            self.reply(addr, &Reply::Error { req_id, error }).await;
            return;
        }
        if let Command::Teleport(p) = &cmd {
            // Streamable-class: preempts the active stream and planned
            // motion, but is not itself a continuing stream.
            if self.active_stream.take().is_some() {
                self.runtime.rt.cancel_stream();
                self.drain_backlog();
            }
            self.cancel_planned();
            self.runtime
                .rt
                .teleport(&p.angles, p.tool_positions.as_deref());
            self.clear_standing_error();
            return;
        }
        debug_assert!(is_stream(tag));
        match self.active_stream {
            Some(active) if active == tag => {
                // Same type: update the active command in place — no new
                // index, no cancel, no drain.
                self.runtime.rt.stream(&cmd);
            }
            _ => {
                if self.active_stream.take().is_some() {
                    // Type change: cancel and flush the stale backlog of
                    // the previous stream before starting fresh.
                    self.runtime.rt.cancel_stream();
                    self.drain_backlog();
                }
                self.cancel_planned();
                self.active_stream = Some(tag);
                self.runtime.rt.stream(&cmd);
            }
        }
        self.clear_standing_error();
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
        if let Some(error) = self.validate_registries(&cmd) {
            self.reply(addr, &Reply::Error { req_id, error }).await;
            return;
        }
        if self.pending.len() >= self.cfg.queue_capacity {
            let error = make_error(ErrorCode::CommQueueFull, UNATTRIBUTED, &[]);
            self.reply(addr, &Reply::Error { req_id, error }).await;
            return;
        }
        let index = self.next_index;
        self.next_index += 1;
        self.dedup.insert(key, index);
        self.accepted_index = index as i64;
        self.clear_standing_error();
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
    fn validate_registries(&self, cmd: &Command) -> Option<WireError> {
        let unknown_tool = |name: &str| {
            make_error(
                ErrorCode::CommValidationError,
                UNATTRIBUTED,
                &[("detail", &format!("unknown tool '{name}'"))],
            )
        };
        match cmd {
            Command::SelectTool(p) if !self.cfg.tools.iter().any(|t| t == &p.tool_name) => {
                Some(unknown_tool(&p.tool_name))
            }
            Command::ToolAction(p) if !self.cfg.tools.iter().any(|t| t == &p.tool_key) => {
                Some(unknown_tool(&p.tool_key))
            }
            _ => None,
        }
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
            let Some(pc) = self.pending.pop_front() else {
                break;
            };
            let name = cmd_name(pc.cmd.tag());
            match self.runtime.planner.start(pc.index, &pc.cmd) {
                Ok(()) => {
                    self.action_state = ActionState::Executing;
                    self.executing = Some(Executing {
                        index: pc.index,
                        addr: pc.addr,
                        name,
                        params: params_summary(&pc.cmd),
                        effect: post_effect(&pc.cmd),
                    });
                }
                Err(e) => {
                    self.fail_command(pc.index, pc.addr, e).await;
                    break;
                }
            }
        }
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
                        PostEffect::SelectTool { tool, variant } => {
                            self.tool = tool;
                            self.tool_variant = variant;
                            self.sync_planner();
                        }
                    }
                    self.push_complete(ex.addr, ex.index, None).await;
                }
                Some(e) => {
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
    }

    fn clear_standing_error(&mut self) {
        if self.standing_error.take().is_some() && self.action_state == ActionState::Error {
            self.action_state = ActionState::Idle;
        }
    }

    fn drain_backlog(&self) {
        let mut buf = [0u8; 2048];
        let mut n = 0usize;
        while self.socket.try_recv_from(&mut buf).is_ok() {
            n += 1;
        }
        if n > 0 {
            log::debug!("stream type change: drained {n} backlogged datagrams");
        }
    }

    // ---- state assembly ----------------------------------------------------

    fn refresh_snapshot(&mut self) {
        if let Some(s) = self.runtime.snapshots.take() {
            self.snap = s;
            self.last_fresh = Some(Instant::now());
        }
    }

    fn data_age_ms(&self) -> u16 {
        match self.last_fresh {
            Some(t) => t.elapsed().as_millis().min(u128::from(u16::MAX)) as u16,
            None => u16::MAX,
        }
    }

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
            shapes: &self.shapes,
            completion_policy: self.completion_policy,
        };
        self.runtime.planner.sync(ctx);
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

    fn io(&self) -> [u8; IO_SLOTS] {
        [
            0,
            0,
            self.io_out[0],
            self.io_out[1],
            u8::from(!self.estop_pressed()),
        ]
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

    fn action_fields(&self) -> (String, ActionState, String) {
        if let Some(ex) = &self.executing {
            (
                ex.name.to_owned(),
                ActionState::Executing,
                ex.params.clone(),
            )
        } else if let Some(name) = self.stream_shown() {
            (name.to_owned(), ActionState::Executing, String::new())
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
                let fault_code = i32::from(r.temperature_error)
                    | (i32::from(r.timeout_error) << 1)
                    | (i32::from(r.estop_error) << 2)
                    | (i32::from(g.live_error_bit) << 3);
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
                    vec![f64::from(r.position)],
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

    fn queued_duration(&self) -> f64 {
        self.pending.iter().map(|p| duration_estimate(&p.cmd)).sum()
    }

    fn build_status(&self) -> Status {
        let en = self.runtime.planner.enablement();
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
            error: self.standing_error.clone(),
            queued_segments: self.pending.len() as u32,
            queued_duration: self.queued_duration(),
            action_params,
            tool_status: self.build_tool_status(),
            tcp_speed: self.tcp_speed,
            simulator_active: self.simulator,
            collision_active: false,
            collision_pairs: Vec::new(),
            scene_epoch: self.scene_epoch,
            accepted_index: self.accepted_index,
            homed: self.snap.homed,
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
                    // The TCP pose expressed in its own tool frame is the
                    // identity by definition.
                    Some(par6_proto::Frame::Trf) => identity_pose(),
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
                // The RT snapshot supplies EMA / p50 / p90 / p99 / max;
                // stats it does not carry (std, min, p95) report 0.0.
                QueryResult::LoopStats(LoopStatsResult {
                    target_hz: self.cfg.rt_tick_rate_hz,
                    loop_count: self.snap.tick,
                    overrun_count: u64::from(ls.overruns),
                    mean_period_s: ls.period_ema_s,
                    std_period_s: 0.0,
                    min_period_s: 0.0,
                    max_period_s: ls.max_s,
                    p95_period_s: 0.0,
                    p99_period_s: ls.p99_s,
                    mean_hz: if ls.period_ema_s > 0.0 {
                        1.0 / ls.period_ema_s
                    } else {
                        0.0
                    },
                })
            }
            C::Profile => QueryResult::Profile {
                profile: self.profile.clone(),
            },
            C::Reachable => {
                let en = self.runtime.planner.enablement();
                QueryResult::Reachable {
                    joint_en: en.joint_en,
                    cart_en_wrf: en.cart_en_wrf,
                    cart_en_trf: en.cart_en_trf,
                }
            }
            C::Error => QueryResult::Error {
                error: self.standing_error.clone(),
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
/// `[x, y, z (m), roll, pitch, yaw (rad)]` TCP, with R = Rz·Ry·Rx.
fn pose_matrix_mm(tcp: &[f64; 6]) -> [f64; POSE_ELEMS] {
    let (x, y, z) = (tcp[0] * 1000.0, tcp[1] * 1000.0, tcp[2] * 1000.0);
    let (sr, cr) = tcp[3].sin_cos();
    let (sp, cp) = tcp[4].sin_cos();
    let (sy, cy) = tcp[5].sin_cos();
    [
        cy * cp,
        cy * sp * sr - sy * cr,
        cy * sp * cr + sy * sr,
        x,
        sy * cp,
        sy * sp * sr + cy * cr,
        sy * sp * cr - cy * sr,
        y,
        -sp,
        cp * sr,
        cp * cr,
        z,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

fn identity_pose() -> [f64; POSE_ELEMS] {
    let mut m = [0.0; POSE_ELEMS];
    m[0] = 1.0;
    m[5] = 1.0;
    m[10] = 1.0;
    m[15] = 1.0;
    m
}

fn post_effect(cmd: &Command) -> PostEffect {
    match cmd {
        Command::Checkpoint(p) => PostEffect::Checkpoint(p.label.clone()),
        Command::SelectTool(p) => PostEffect::SelectTool {
            tool: p.tool_name.clone(),
            variant: p.variant_key.clone(),
        },
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

/// Explicitly specified time a queued command will take; speed-based
/// moves contribute 0 (the planner learns their duration only after
/// parameterization).
fn duration_estimate(cmd: &Command) -> f64 {
    use Command as C;
    match cmd {
        C::MoveJ(p) => p.duration.unwrap_or(0.0),
        C::MoveJPose(p) => p.duration.unwrap_or(0.0),
        C::MoveL(p) => p.duration.unwrap_or(0.0),
        C::MoveC(p) => p.duration.unwrap_or(0.0),
        C::MoveS(p) => p.duration.unwrap_or(0.0),
        C::MoveP(p) => p.duration.unwrap_or(0.0),
        C::Delay(p) => p.seconds,
        _ => 0.0,
    }
}

/// Wire name of a command (STATUS `action_current`, QUEUE listing).
fn cmd_name(tag: CmdType) -> &'static str {
    use CmdType as T;
    match tag {
        T::Reset => "reset",
        T::Estop => "estop",
        T::Stop => "stop",
        T::WriteIo => "write_io",
        T::Simulator => "simulator",
        T::SelectProfile => "select_profile",
        T::ResetState => "reset_state",
        T::ConnectHardware => "connect_hardware",
        T::SetTcpOffset => "set_tcp_offset",
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

fn decode_error_to_wire(e: &DecodeError) -> WireError {
    let code = match e {
        DecodeError::Validation { .. } => ErrorCode::CommValidationError,
        DecodeError::UnknownTag(_) => ErrorCode::CommUnknownCommand,
        _ => ErrorCode::CommDecodeError,
    };
    make_error(code, UNATTRIBUTED, &[("detail", &e.to_string())])
}

/// Best-effort `req_id` salvage from a malformed datagram so the decode
/// ERROR still correlates; 0 (the push convention) when unreadable.
fn peek_req_id(data: &[u8]) -> Option<u32> {
    fn uint(data: &[u8], pos: &mut usize) -> Option<u64> {
        let b = *data.get(*pos)?;
        *pos += 1;
        match b {
            0x00..=0x7f => Some(u64::from(b)),
            0xcc => {
                let v = *data.get(*pos)?;
                *pos += 1;
                Some(u64::from(v))
            }
            0xcd => {
                let v = u16::from_be_bytes(data.get(*pos..*pos + 2)?.try_into().ok()?);
                *pos += 2;
                Some(u64::from(v))
            }
            0xce => {
                let v = u32::from_be_bytes(data.get(*pos..*pos + 4)?.try_into().ok()?);
                *pos += 4;
                Some(u64::from(v))
            }
            0xcf => {
                let v = u64::from_be_bytes(data.get(*pos..*pos + 8)?.try_into().ok()?);
                *pos += 8;
                Some(v)
            }
            _ => None,
        }
    }
    let mut pos = match *data.first()? {
        0x90..=0x9f => 1usize,
        0xdc => 3,
        0xdd => 5,
        _ => return None,
    };
    uint(data, &mut pos)?; // tag
    u32::try_from(uint(data, &mut pos)?).ok()
}
