//! Protocol-surface integration tests: real UDP sockets on ephemeral
//! ports, real codec bytes in both directions, budget-bounded polling
//! (no blind sleeps). The `Planner` / `RtCommands` boundary is driven by
//! in-crate doubles; assertions go through replies, COMPLETE pushes,
//! queries and STATUS broadcasts wherever the protocol can express them.

use std::collections::VecDeque;
use std::net::SocketAddr;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use par6_proto::command::{
    JogJ, JogL, MoveJ, MoveS, SetRecipe, SetShapes, Shape, Simulator, Stop, Teleport, WriteIo,
};
use par6_proto::{
    decode_reply, decode_status, encode_chunk, encode_command, make_error, split_into_chunks,
    ActionState, CmdType, Command, ErrorCode, Frame, QueryResult, Reply, WireError, UNATTRIBUTED,
};
use par6_rt::{
    snapshot_channel, ArmState, ErrorCode as RtCode, ErrorEntry, Mode, SnapshotWriter,
    StateSnapshot,
};
use par6_server::{
    spawn, CollisionState, CommandOutcome, Enablement, PlanContext, Planner, QueuedCommand,
    RtCommands, RuntimeHandle, ServerConfig, ServerHandle, ShapeLayer, StatusTransport,
};
use tokio::net::UdpSocket;
use tokio::time::timeout;

const BUDGET: Duration = Duration::from_secs(2);

// ---- runtime doubles -------------------------------------------------------

#[derive(Debug, Clone, PartialEq)]
enum RtEvent {
    Stream(CmdType),
    CancelStream,
    Halt,
    SafetyStop,
    SetGravityComp(bool),
    SetEnabled(bool),
    Teleport([f64; 6]),
    WriteIo(u8, u8),
    Simulator(bool),
    ConnectHardware(String),
    ResetState,
    ResetLoopStats,
}

#[derive(Default)]
struct RtLog {
    events: Vec<RtEvent>,
    /// What the RT answers the NEXT streaming setpoint with. `None` = it
    /// is forwarded; `Some` = refused, the way the real bridge refuses a
    /// jog its collision gate blocks.
    stream_verdict: Option<WireError>,
    /// What the RT answers the NEXT enable request with. `None` = it came
    /// up ENABLED; `Some` = it refused, the way the real core does while
    /// the e-stop line is engaged or a hard error is latched.
    enable_verdict: Option<WireError>,
    /// The pending answer, collected exactly once by the server.
    enable_outcome: Option<Result<(), WireError>>,
    /// While true the RT is "still deciding": `take_enable_outcome`
    /// answers `None`, the way the real core does for several ticks —
    /// which is what lets `reset` waiters pile up.
    hold_enable_outcome: bool,
}

#[derive(Clone)]
struct TestRt(Arc<Mutex<RtLog>>);

impl TestRt {
    fn push(&self, e: RtEvent) {
        self.0.lock().unwrap().events.push(e);
    }
}

impl RtCommands for TestRt {
    fn stream(&mut self, cmd: &Command) -> Result<(), WireError> {
        if let Some(e) = self.0.lock().unwrap().stream_verdict.take() {
            return Err(e);
        }
        self.push(RtEvent::Stream(cmd.tag()));
        Ok(())
    }
    fn cancel_stream(&mut self) {
        self.push(RtEvent::CancelStream);
    }
    fn halt(&mut self) {
        self.push(RtEvent::Halt);
    }
    fn safety_stop(&mut self) {
        self.push(RtEvent::SafetyStop);
    }
    fn set_gravity_comp(&mut self, on: bool) {
        self.push(RtEvent::SetGravityComp(on));
    }
    fn set_enabled(&mut self, enabled: bool) {
        self.push(RtEvent::SetEnabled(enabled));
        let mut log = self.0.lock().unwrap();
        if enabled {
            log.enable_outcome = Some(match log.enable_verdict.clone() {
                None => Ok(()),
                Some(e) => Err(e),
            });
        } else if log.enable_outcome.take().is_some() {
            // A disable supersedes an enable nobody has collected yet.
            log.enable_outcome = Some(Err(make_error(
                ErrorCode::SysEstopActive,
                UNATTRIBUTED,
                &[],
            )));
        }
    }
    fn take_enable_outcome(&mut self) -> Option<Result<(), WireError>> {
        let mut log = self.0.lock().unwrap();
        if log.hold_enable_outcome {
            return None;
        }
        log.enable_outcome.take()
    }
    fn teleport(&mut self, angles_deg: &[f64; 6], _tool_positions: Option<&[f64]>) {
        self.push(RtEvent::Teleport(*angles_deg));
    }
    fn write_io(&mut self, port: u8, value: u8) {
        self.push(RtEvent::WriteIo(port, value));
    }
    fn set_simulator(&mut self, on: bool) -> Result<(), WireError> {
        self.push(RtEvent::Simulator(on));
        Ok(())
    }
    fn connect_hardware(&mut self, port: &str) -> Result<(), WireError> {
        self.push(RtEvent::ConnectHardware(port.to_owned()));
        Ok(())
    }
    fn reset_state(&mut self) {
        self.push(RtEvent::ResetState);
    }
    fn reset_loop_stats(&mut self) {
        self.push(RtEvent::ResetLoopStats);
    }
}

#[derive(Default)]
struct PlannerState {
    started: Vec<(u64, Command)>,
    /// Every batch the server offered to `start`, as the indexes in it.
    batches: Vec<Vec<u64>>,
    /// How many commands of each offered batch this planner reports it
    /// folded into one motion. 0 = the default (one command at a time).
    consume: usize,
    outcomes: VecDeque<CommandOutcome>,
    fail_next_start: Option<WireError>,
    cancels: usize,
    enablement: Enablement,
    /// Every accepted layer replacement, in order.
    layers: Vec<(ShapeLayer, Vec<Shape>)>,
    /// Epoch of the applied world, moved only by an accepted replacement
    /// (`par6-kin`'s `Collision::set_layer` contract).
    epoch: u64,
    fail_next_shapes: Option<WireError>,
    collision: Option<CollisionState>,
    /// How many times the server asked for a queue estimate.
    estimates: usize,
    /// Seconds the in-flight motion reports it has left.
    inflight_duration: f64,
}

#[derive(Clone)]
struct TestPlanner(Arc<Mutex<PlannerState>>);

impl Planner for TestPlanner {
    fn start(&mut self, batch: &[QueuedCommand<'_>]) -> Result<usize, WireError> {
        let mut s = self.0.lock().unwrap();
        s.batches.push(batch.iter().map(|q| q.index).collect());
        if let Some(e) = s.fail_next_start.take() {
            return Err(e);
        }
        let head = batch.first().expect("the server never starts nothing");
        s.started.push((head.index, head.cmd.clone()));
        Ok(s.consume.clamp(1, batch.len()))
    }
    fn poll(&mut self) -> Option<CommandOutcome> {
        self.0.lock().unwrap().outcomes.pop_front()
    }
    fn cancel(&mut self) {
        self.0.lock().unwrap().cancels += 1;
    }
    fn sync(&mut self, _ctx: PlanContext<'_>) {}
    fn set_shapes(
        &mut self,
        layer: ShapeLayer,
        shapes: &[Shape],
    ) -> Result<Option<u64>, WireError> {
        let mut s = self.0.lock().unwrap();
        if let Some(e) = s.fail_next_shapes.take() {
            return Err(e);
        }
        s.layers.push((layer, shapes.to_vec()));
        s.epoch += 1;
        Ok(Some(s.epoch))
    }
    fn collision(&mut self) -> Option<CollisionState> {
        self.0.lock().unwrap().collision.clone()
    }
    fn clear_collision(&mut self) {
        self.0.lock().unwrap().collision = Some(CollisionState::default());
    }
    fn enablement(&self) -> Enablement {
        self.0.lock().unwrap().enablement
    }
    /// Times what it is told — an explicit `duration=` on a move, a
    /// delay's seconds — and nothing else, which is the floor the trait
    /// asks for. Counts its calls: the server must not re-estimate
    /// (i.e. re-plan) a queue that has not changed.
    fn queued_duration(&mut self, pending: &[QueuedCommand<'_>]) -> f64 {
        let mut s = self.0.lock().unwrap();
        s.estimates += 1;
        pending
            .iter()
            .map(|q| match q.cmd {
                Command::MoveJ(p) => p.duration.unwrap_or(0.0),
                Command::MoveS(p) => p.duration.unwrap_or(0.0),
                Command::Delay(p) => p.seconds,
                _ => 0.0,
            })
            .sum()
    }
    fn inflight_duration(&self, _snap: &StateSnapshot) -> f64 {
        self.0.lock().unwrap().inflight_duration
    }
}

// ---- harness ---------------------------------------------------------------

struct Harness {
    server: ServerHandle,
    rt: Arc<Mutex<RtLog>>,
    planner: Arc<Mutex<PlannerState>>,
    writer: SnapshotWriter<StateSnapshot>,
    status_rx: UdpSocket,
    telemetry_rx: UdpSocket,
    tick: u64,
}

async fn start(tweak: impl FnOnce(&mut ServerConfig)) -> Harness {
    let status_rx = UdpSocket::bind("127.0.0.1:0").await.unwrap();
    let telemetry_rx = UdpSocket::bind("127.0.0.1:0").await.unwrap();
    let mut cfg = ServerConfig {
        bind: "127.0.0.1:0".parse().unwrap(),
        status_transport: StatusTransport::Unicast,
        status_dest_host: "127.0.0.1".parse().unwrap(),
        status_port: status_rx.local_addr().unwrap().port(),
        telemetry_port: telemetry_rx.local_addr().unwrap().port(),
        status_rate_hz: 100,
        telemetry_rate_hz: 200,
        poll_interval: Duration::from_millis(1),
        link_stale: Duration::from_millis(100),
        chunk_timeout: Duration::from_millis(100),
        controller_id: 42,
        tools: vec!["gripper".to_owned()],
        ..ServerConfig::default()
    };
    tweak(&mut cfg);
    let (writer, snapshots) = snapshot_channel::<StateSnapshot>();
    let rt = Arc::new(Mutex::new(RtLog::default()));
    let planner = Arc::new(Mutex::new(PlannerState::default()));
    let server = spawn(
        cfg,
        RuntimeHandle {
            planner: TestPlanner(planner.clone()),
            rt: TestRt(rt.clone()),
            snapshots,
        },
    )
    .await
    .expect("server spawns");
    Harness {
        server,
        rt,
        planner,
        writer,
        status_rx,
        telemetry_rx,
        tick: 0,
    }
}

impl Harness {
    /// Publish a fresh snapshot (enabled + homed + a live motor bus
    /// unless overridden — the default `NodeState` means "never heard",
    /// which reads as a dead link). The server re-reads the snapshot
    /// channel on every datagram, so a command sent after this sees the
    /// new state.
    fn publish(&mut self, f: impl FnOnce(&mut StateSnapshot)) {
        self.tick += 1;
        let mut s = StateSnapshot {
            tick: self.tick,
            state: ArmState::Enabled,
            homed: true,
            ..StateSnapshot::default()
        };
        for node in &mut s.nodes {
            node.data_age_ticks = 0;
        }
        f(&mut s);
        self.writer.publish(&s);
    }

    fn complete_ok(&self, index: u64) {
        self.planner
            .lock()
            .unwrap()
            .outcomes
            .push_back(CommandOutcome { index, error: None });
    }

    fn complete_err(&self, index: u64, error: WireError) {
        self.planner
            .lock()
            .unwrap()
            .outcomes
            .push_back(CommandOutcome {
                index,
                error: Some(error),
            });
    }

    fn rt_events(&self) -> Vec<RtEvent> {
        self.rt.lock().unwrap().events.clone()
    }

    async fn wait_rt(&self, pred: impl Fn(&[RtEvent]) -> bool) -> Vec<RtEvent> {
        let deadline = tokio::time::Instant::now() + BUDGET;
        loop {
            let ev = self.rt_events();
            if pred(&ev) {
                return ev;
            }
            assert!(
                tokio::time::Instant::now() < deadline,
                "rt condition not met within budget; events: {ev:?}"
            );
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
    }
}

// ---- client ----------------------------------------------------------------

struct Client {
    sock: UdpSocket,
    server: SocketAddr,
    next_req: u32,
    stash: Vec<Reply>,
}

impl Client {
    async fn new(h: &Harness) -> Client {
        Client::with_first_req(h, 1).await
    }

    async fn with_first_req(h: &Harness, first: u32) -> Client {
        Client {
            sock: UdpSocket::bind("127.0.0.1:0").await.unwrap(),
            server: h.server.addr,
            next_req: first,
            stash: Vec::new(),
        }
    }

    async fn send(&mut self, cmd: &Command) -> u32 {
        let req_id = self.next_req;
        self.next_req += 1;
        let mut buf = Vec::new();
        encode_command(cmd, req_id, &mut buf).expect("encodable command");
        self.sock.send_to(&buf, self.server).await.expect("send");
        req_id
    }

    async fn recv(&self) -> Reply {
        let mut buf = [0u8; 4096];
        let (n, _) = timeout(BUDGET, self.sock.recv_from(&mut buf))
            .await
            .expect("reply within budget")
            .expect("recv");
        decode_reply(&buf[..n]).expect("decodable reply")
    }

    /// Send and wait for the direct reply with the matching `req_id`,
    /// stashing any COMPLETE pushes that arrive in between.
    async fn request(&mut self, cmd: &Command) -> Reply {
        let req_id = self.send(cmd).await;
        loop {
            let r = self.recv().await;
            match &r {
                Reply::Ok { req_id: id, .. }
                | Reply::Error { req_id: id, .. }
                | Reply::Response { req_id: id, .. }
                    if *id == req_id =>
                {
                    return r;
                }
                Reply::Complete { .. } => self.stash.push(r),
                _ => {}
            }
        }
    }

    async fn query(&mut self, cmd: &Command) -> QueryResult {
        match self.request(cmd).await {
            Reply::Response { result, .. } => result,
            other => panic!("expected RESPONSE, got {other:?}"),
        }
    }

    async fn ok_index(&mut self, cmd: &Command) -> u64 {
        match self.request(cmd).await {
            Reply::Ok { index: Some(i), .. } => i,
            other => panic!("expected OK with index, got {other:?}"),
        }
    }

    async fn expect_error(&mut self, cmd: &Command) -> WireError {
        match self.request(cmd).await {
            Reply::Error { error, .. } => error,
            other => panic!("expected ERROR, got {other:?}"),
        }
    }

    async fn wait_complete(&mut self, index: u64) -> (bool, Option<WireError>) {
        if let Some(pos) = self
            .stash
            .iter()
            .position(|r| matches!(r, Reply::Complete { index: i, .. } if *i == index))
        {
            if let Reply::Complete { ok, detail, .. } = self.stash.remove(pos) {
                return (ok, detail);
            }
        }
        loop {
            if let Reply::Complete {
                index: i,
                ok,
                detail,
            } = self.recv().await
            {
                if i == index {
                    return (ok, detail);
                }
            }
        }
    }
}

async fn recv_status(sock: &UdpSocket) -> par6_proto::Status {
    let mut buf = [0u8; 4096];
    let (n, _) = timeout(BUDGET, sock.recv_from(&mut buf))
        .await
        .expect("status within budget")
        .expect("recv");
    decode_status(&buf[..n]).expect("decodable status")
}

async fn recv_telemetry(sock: &UdpSocket) -> (String, u64, u64, u64, Vec<f64>) {
    let mut buf = [0u8; 4096];
    let (n, _) = timeout(BUDGET, sock.recv_from(&mut buf))
        .await
        .expect("telemetry within budget")
        .expect("recv");
    rmp_serde::from_slice(&buf[..n]).expect("minimal recipe layout: [name, seq, mono_ns, tick, q]")
}

/// A TCP rotation with three substantial components \[rad\] — the only
/// kind that tells the wire's rotation convention apart from the
/// fixed-axis reading of the same three numbers.
const TILTED_RPY: [f64; 3] = [0.7, -0.4, 1.1];

/// `R = Rx(r)·Ry(p)·Rz(y)` as a row-major 3x3: the wire's intrinsic-XYZ
/// convention composed the long way round, so it
/// shares no code with the STATUS builder it judges.
fn intrinsic_xyz(rpy: [f64; 3]) -> [[f64; 3]; 3] {
    let (sr, cr) = rpy[0].sin_cos();
    let (sp, cp) = rpy[1].sin_cos();
    let (sy, cy) = rpy[2].sin_cos();
    let rx = [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]];
    let ry = [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]];
    let rz = [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]];
    let mul = |a: [[f64; 3]; 3], b: [[f64; 3]; 3]| {
        std::array::from_fn(|r| std::array::from_fn(|c| (0..3).map(|k| a[r][k] * b[k][c]).sum()))
    };
    mul(mul(rx, ry), rz)
}

// ---- command builders ------------------------------------------------------

fn move_j(key: u64) -> Command {
    Command::MoveJ(MoveJ {
        key,
        angles: [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        duration: Some(0.5),
        speed: None,
        accel: None,
        blend_radius: None,
        rel: false,
    })
}

fn jog_j() -> Command {
    Command::JogJ(JogJ {
        speeds: [0.2, 0.0, 0.0, 0.0, 0.0, 0.0],
        duration: 0.2,
        accel: None,
    })
}

fn jog_l() -> Command {
    Command::JogL(JogL {
        velocities: [0.0, 0.1, 0.0, 0.0, 0.0, 0.0],
        duration: 0.2,
        frame: Frame::Wrf,
        accel: None,
    })
}

fn move_s(key: u64, n: usize) -> Command {
    Command::MoveS(MoveS {
        key,
        waypoints: (0..n)
            .map(|i| {
                let v = i as f64;
                [v, v * 0.5, -v, 1.0, 2.0, 3.0]
            })
            .collect(),
        frame: Frame::Wrf,
        duration: Some(2.0),
        speed: None,
        accel: None,
    })
}

// ---- tests -----------------------------------------------------------------

/// req_id correlation across interleaved requests and clients, plus
/// query payload fidelity from the published snapshot.
#[tokio::test]
async fn request_reply_correlation_with_interleaved_clients() {
    let mut h = start(|_| {}).await;
    h.publish(|s| {
        s.q = [0.1, 0.2, 0.3, -0.1, -0.2, -0.3];
        s.qd = [1.0, 0.0, 0.0, 0.0, 0.0, 2.0];
    });

    let mut a = Client::with_first_req(&h, 100).await;
    let mut b = Client::with_first_req(&h, 9000).await;

    // Three requests in flight on one socket; replies matched by req_id.
    let ra = a.send(&Command::Angles).await;
    let rs = a.send(&Command::Speeds).await;
    let ri = a.send(&Command::IsSimulator).await;
    let rb = b.send(&Command::Angles).await;

    let mut got = Vec::new();
    for _ in 0..3 {
        got.push(a.recv().await);
    }
    for r in got {
        match r {
            Reply::Response { req_id, result } => match result {
                QueryResult::Angles { angles } => {
                    assert_eq!(req_id, ra);
                    assert!((angles[0] - 0.1f64.to_degrees()).abs() < 1e-9);
                    assert!((angles[5] - (-0.3f64).to_degrees()).abs() < 1e-9);
                }
                QueryResult::Speeds { speeds } => {
                    assert_eq!(req_id, rs);
                    assert_eq!(speeds, [1.0, 0.0, 0.0, 0.0, 0.0, 2.0]);
                }
                QueryResult::IsSimulator { active } => {
                    assert_eq!(req_id, ri);
                    assert!(!active);
                }
                other => panic!("unexpected result {other:?}"),
            },
            other => panic!("expected RESPONSE, got {other:?}"),
        }
    }
    // The other client's reply went to its own socket with its own id.
    match b.recv().await {
        Reply::Response { req_id, result } => {
            assert_eq!(req_id, rb);
            assert!(matches!(result, QueryResult::Angles { .. }));
        }
        other => panic!("expected RESPONSE, got {other:?}"),
    }

    // A malformed datagram still correlates: unknown tag 999, req_id 5.
    let raw = [0x92, 0xcd, 0x03, 0xe7, 0x05];
    a.sock.send_to(&raw, a.server).await.unwrap();
    match a.recv().await {
        Reply::Error { req_id, error } => {
            assert_eq!(req_id, 5);
            assert_eq!(error.code, ErrorCode::CommUnknownCommand as u16);
        }
        other => panic!("expected ERROR, got {other:?}"),
    }
}

/// Queued lifecycle: ack carries the index, idempotent retry re-acks the
/// ORIGINAL index without re-queueing, COMPLETE pushes on ok and error,
/// error latches attributed and acceptance clears it.
#[tokio::test]
async fn queued_ack_dedup_complete_and_error_latch() {
    let mut h = start(|_| {}).await;
    h.publish(|_| {});
    let mut c = Client::new(&h).await;

    let i1 = c.ok_index(&move_j(101)).await;
    assert_eq!(i1, 1, "first allocated index");
    h.complete_ok(i1);
    let (ok, detail) = c.wait_complete(i1).await;
    assert!(ok && detail.is_none());

    // Retry with the same idempotency key: same index, nothing re-runs.
    let retry = c.ok_index(&move_j(101)).await;
    assert_eq!(retry, i1);
    match c.query(&Command::Queue).await {
        QueryResult::Queue {
            queue,
            executing_index,
            completed_index,
            ..
        } => {
            assert!(queue.is_empty(), "retry must not re-queue");
            assert_eq!(executing_index, -1);
            assert_eq!(completed_index, i1 as i64);
        }
        other => panic!("unexpected {other:?}"),
    }

    // Failure: COMPLETE(ok=false) with the attributed 6-tuple, latched.
    let i2 = c.ok_index(&move_j(102)).await;
    h.complete_err(i2, make_error(ErrorCode::MotnTickFailed, UNATTRIBUTED, &[]));
    let (ok, detail) = c.wait_complete(i2).await;
    assert!(!ok);
    let detail = detail.expect("failure detail present");
    assert_eq!(detail.code, ErrorCode::MotnTickFailed as u16);
    assert_eq!(detail.command_index, i2 as i64);
    match c.query(&Command::Error).await {
        QueryResult::Error { error: Some(e) } => {
            assert_eq!(e.command_index, i2 as i64);
        }
        other => panic!("standing error expected, got {other:?}"),
    }

    // Acceptance of the next command clears the standing error.
    let i3 = c.ok_index(&move_j(103)).await;
    assert_eq!(i3, i2 + 1);
    match c.query(&Command::Error).await {
        QueryResult::Error { error } => assert!(error.is_none()),
        other => panic!("unexpected {other:?}"),
    }

    // A start-time rejection also completes with an error.
    h.planner.lock().unwrap().fail_next_start = Some(make_error(
        ErrorCode::IkTargetUnreachable,
        UNATTRIBUTED,
        &[],
    ));
    h.complete_ok(i3); // let i3 finish first so the queue advances
    let (ok, _) = c.wait_complete(i3).await;
    assert!(ok);
    let i4 = c.ok_index(&move_j(104)).await;
    let (ok, detail) = c.wait_complete(i4).await;
    assert!(!ok);
    assert_eq!(detail.unwrap().code, ErrorCode::IkTargetUnreachable as u16);
}

#[tokio::test]
async fn queue_full_rejects_with_comm_queue_full() {
    let mut h = start(|cfg| cfg.queue_capacity = 2).await;
    h.publish(|_| {});
    let mut c = Client::new(&h).await;

    // First one starts executing; the next two fill the pending queue.
    let waited = c.ok_index(&move_j(201)).await;
    let deadline = tokio::time::Instant::now() + BUDGET;
    loop {
        // Wait until it left the pending queue for the executing slot.
        if let QueryResult::Queue {
            executing_index, ..
        } = c.query(&Command::Queue).await
        {
            if executing_index == waited as i64 {
                break;
            }
        }
        assert!(tokio::time::Instant::now() < deadline);
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
    c.ok_index(&move_j(202)).await;
    c.ok_index(&move_j(203)).await;
    let err = c.expect_error(&move_j(204)).await;
    assert_eq!(err.code, ErrorCode::CommQueueFull as u16);
    assert!(
        err.cause.contains("motion queue"),
        "the refusal must name the MOTION queue: {}",
        err.cause
    );
}

/// A `reset` refused for the waiter cap shares COMM_QUEUE_FULL with the
/// motion queue, so its detail must name the RESET pile-up — pointing a
/// retrying operator at the outstanding reset, not at an empty motion
/// queue. The held waiters are all answered once the RT decides.
#[tokio::test]
async fn reset_waiter_overflow_names_itself_in_the_refusal() {
    let mut h = start(|_| {}).await;
    h.publish(|_| {});
    h.rt.lock().unwrap().hold_enable_outcome = true;
    let mut c = Client::new(&h).await;

    // 16 resets pile up unanswered while the RT is "still deciding".
    for _ in 0..16 {
        c.send(&Command::Reset).await;
    }
    let err = c.expect_error(&Command::Reset).await;
    assert_eq!(err.code, ErrorCode::CommQueueFull as u16);
    assert!(
        err.cause.contains("reset"),
        "the refusal must name the reset pile-up: {}",
        err.cause
    );
    assert!(
        !err.cause.contains("motion queue"),
        "the refusal must not point at the motion queue: {}",
        err.cause
    );

    // The RT decides: every held waiter gets the verdict.
    h.rt.lock().unwrap().hold_enable_outcome = false;
    for _ in 0..16 {
        match c.recv().await {
            Reply::Ok { index: None, .. } => {}
            other => panic!("held reset waiters must be answered, got {other:?}"),
        }
    }
}

/// The gating table over the wire: un-homed, disabled, e-stop latch and
/// simulator-only rejections carry their specific error codes; `home`
/// is the one motion command that stays available un-homed.
#[tokio::test]
async fn gating_rejections_carry_specific_codes() {
    let mut h = start(|_| {}).await;
    let mut c = Client::new(&h).await;

    // Un-homed: planned motion refused, home accepted.
    h.publish(|s| s.homed = false);
    let err = c.expect_error(&move_j(301)).await;
    assert_eq!(err.code, ErrorCode::MotnNotHomed as u16);
    let home_idx = c
        .ok_index(&Command::Home(par6_proto::command::Home { key: 302 }))
        .await;
    assert!(home_idx >= 1);

    // ...but a jog is NOT refused. An arm can need jogging clear of an
    // obstruction before it can be homed at all, so jog is gated on the
    // collision world and the soft-limit brake rather than on a home
    // reference — the RT's mode gate agrees, so nothing is dropped
    // silently downstream.
    c.send(&jog_j()).await; // fire-and-forget: success is unacked
    h.wait_rt(|ev| ev.contains(&RtEvent::Stream(CmdType::JogJ)))
        .await;

    // Homed: still accepted, unchanged.
    h.publish(|_| {});
    c.send(&jog_j()).await;
    h.wait_rt(|ev| ev.contains(&RtEvent::Stream(CmdType::JogJ)))
        .await;

    // Disabled (RT core reports DISABLED).
    h.publish(|s| s.state = ArmState::Disabled);
    let err = c.expect_error(&move_j(303)).await;
    assert_eq!(err.code, ErrorCode::SysControllerDisabled as u16);
    let err = c.expect_error(&jog_j()).await;
    assert_eq!(err.code, ErrorCode::SysControllerDisabled as u16);

    // Teleport outside simulator mode is a real error, not a no-op.
    h.publish(|_| {});
    let err = c
        .expect_error(&Command::Teleport(Teleport {
            angles: [0.0; 6],
            tool_positions: None,
        }))
        .await;
    assert_eq!(err.code, ErrorCode::SysNotSimulator as u16);
    // ...and works once the simulator backend is on.
    match c.request(&Command::Simulator(Simulator { on: true })).await {
        Reply::Ok { index: None, .. } => {}
        other => panic!("expected plain OK, got {other:?}"),
    }
    c.send(&Command::Teleport(Teleport {
        angles: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        tool_positions: None,
    }))
    .await;
    h.wait_rt(|ev| ev.contains(&RtEvent::Teleport([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])))
        .await;
}

/// stop/estop/reset/reset_state cancel scopes, the e-stop DISABLED
/// latch, and the never-reset index allocator.
#[tokio::test]
async fn stop_estop_reset_semantics() {
    let mut h = start(|_| {}).await;
    h.publish(|_| {});
    let mut c = Client::new(&h).await;

    let i1 = c.ok_index(&move_j(401)).await;
    let i2 = c.ok_index(&move_j(402)).await;
    let i3 = c.ok_index(&move_j(403)).await;
    assert_eq!((i1, i2, i3), (1, 2, 3));

    // stop {clear_queue: false}: the active command halts, the queue
    // continues with the next one.
    match c.request(&Command::Stop(Stop { clear_queue: false })).await {
        Reply::Ok { index: None, .. } => {}
        other => panic!("expected OK, got {other:?}"),
    }
    let deadline = tokio::time::Instant::now() + BUDGET;
    loop {
        if let QueryResult::Queue {
            executing_index, ..
        } = c.query(&Command::Queue).await
        {
            if executing_index == i2 as i64 {
                break;
            }
            assert_ne!(
                executing_index, i1 as i64,
                "stopped command must not resume"
            );
        }
        assert!(tokio::time::Instant::now() < deadline);
        tokio::time::sleep(Duration::from_millis(2)).await;
    }

    // stop {clear_queue: true}: nothing executing, nothing pending.
    c.request(&Command::Stop(Stop { clear_queue: true })).await;
    match c.query(&Command::Queue).await {
        QueryResult::Queue {
            queue,
            executing_index,
            ..
        } => {
            assert!(queue.is_empty());
            assert_eq!(executing_index, -1);
        }
        other => panic!("unexpected {other:?}"),
    }
    // Still enabled: a new command is accepted (and the allocator moved on).
    let i4 = c.ok_index(&move_j(404)).await;
    assert_eq!(i4, 4);

    // estop: halts + clears + latches DISABLED until reset.
    c.request(&Command::Estop).await;
    let ev = h.rt_events();
    assert!(ev.contains(&RtEvent::SetEnabled(false)), "{ev:?}");
    let err = c.expect_error(&move_j(405)).await;
    assert_eq!(err.code, ErrorCode::SysEstopActive as u16);
    match c.query(&Command::Error).await {
        QueryResult::Error { error: Some(e) } => {
            assert_eq!(e.code, ErrorCode::SysEstopActive as u16);
            assert_eq!(e.command_index, UNATTRIBUTED);
        }
        other => panic!("standing estop error expected, got {other:?}"),
    }
    // reset_state does NOT clear the e-stop latch...
    c.request(&Command::ResetState).await;
    let err = c.expect_error(&move_j(406)).await;
    assert_eq!(err.code, ErrorCode::SysEstopActive as u16);
    // ...reset does.
    c.request(&Command::Reset).await;
    let ev = h.rt_events();
    assert!(ev.contains(&RtEvent::SetEnabled(true)), "{ev:?}");
    match c.query(&Command::Error).await {
        QueryResult::Error { error } => assert!(error.is_none()),
        other => panic!("unexpected {other:?}"),
    }
    // The allocator was never reset — not by estop, not by reset_state.
    let i5 = c.ok_index(&move_j(407)).await;
    assert_eq!(i5, 5);
}

/// Streaming preemption: same-type updates in place, a different type
/// cancels + drains the socket backlog, a planned move cancels the
/// stream, and a stream cancels planned motion.
#[tokio::test]
async fn streaming_preemption_semantics() {
    let mut h = start(|_| {}).await;
    h.publish(|_| {});
    let mut c = Client::new(&h).await;

    // Start a jog stream, then update it in place.
    c.send(&jog_j()).await;
    h.wait_rt(|ev| {
        ev.iter()
            .filter(|e| **e == RtEvent::Stream(CmdType::JogJ))
            .count()
            == 1
    })
    .await;
    c.send(&jog_j()).await;
    let ev = h
        .wait_rt(|ev| {
            ev.iter()
                .filter(|e| **e == RtEvent::Stream(CmdType::JogJ))
                .count()
                == 2
        })
        .await;
    assert!(
        !ev.contains(&RtEvent::CancelStream),
        "same-type update must not cancel: {ev:?}"
    );

    // Type change with a backlog: jog_l followed immediately by two
    // stale jog_j datagrams (all buffered before the server reads —
    // current-thread runtime, ready sends). The switch must cancel the
    // jog stream and drain the backlog, so those jog_j never replay.
    c.send(&jog_l()).await;
    c.send(&jog_j()).await;
    c.send(&jog_j()).await;
    let ev = h
        .wait_rt(|ev| ev.contains(&RtEvent::Stream(CmdType::JogL)))
        .await;
    let jog_l_pos = ev
        .iter()
        .position(|e| *e == RtEvent::Stream(CmdType::JogL))
        .unwrap();
    assert_eq!(
        ev[jog_l_pos - 1],
        RtEvent::CancelStream,
        "type change cancels the previous stream: {ev:?}"
    );
    // Liveness after the drain, and proof the drained jog_j are gone.
    match c.query(&Command::Ping).await {
        QueryResult::Ping { .. } => {}
        other => panic!("unexpected {other:?}"),
    }
    let ev = h.rt_events();
    assert!(
        !ev[jog_l_pos + 1..].contains(&RtEvent::Stream(CmdType::JogJ)),
        "drained backlog must not replay: {ev:?}"
    );

    // A planned move cancels the (jog_l) stream.
    let cancels_before = h
        .rt_events()
        .iter()
        .filter(|e| **e == RtEvent::CancelStream)
        .count();
    let i = c.ok_index(&move_j(501)).await;
    let ev = h
        .wait_rt(|ev| ev.iter().filter(|e| **e == RtEvent::CancelStream).count() > cancels_before)
        .await;
    assert!(!ev.is_empty());

    // ...and a fresh stream cancels planned motion: the queued command
    // vanishes without resuming.
    c.send(&jog_j()).await;
    h.wait_rt(|ev| {
        ev.iter()
            .filter(|e| **e == RtEvent::Stream(CmdType::JogJ))
            .count()
            == 3
    })
    .await;
    match c.query(&Command::Queue).await {
        QueryResult::Queue {
            queue,
            executing_index,
            ..
        } => {
            assert!(queue.is_empty());
            assert_eq!(executing_index, -1, "jog cancelled planned move {i}");
        }
        other => panic!("unexpected {other:?}"),
    }
}

/// What a stream preemption is allowed to throw away.
///
/// The command socket carries every client and every command class, so
/// the drain that follows a stream type change must discard the stale
/// setpoints of the stream it replaced and NOTHING else. The case that
/// makes this a safety defect rather than a nicety: the operator jogs
/// joints, drags a Cartesian axis, and hits the software E-STOP — all
/// three datagrams queued before the server reads any of them. Draining
/// the socket destroys the `estop` with no reply and no effect, and the
/// client's SYSTEM send does not retry, so the UI shows E-STOP ACTIVE
/// over an arm that was never disabled.
#[tokio::test]
async fn stream_preemption_never_destroys_buffered_system_commands() {
    let mut h = start(|_| {}).await;
    h.publish(|_| {});
    let mut c = Client::new(&h).await;

    // An established jog_j stream — so the jog_l below is a type change,
    // the arm that drains.
    c.send(&jog_j()).await;
    h.wait_rt(|ev| ev.contains(&RtEvent::Stream(CmdType::JogJ)))
        .await;

    // Three datagrams, all buffered before the server reads (ready sends
    // on a current-thread runtime): the preempting jog_l, a stale jog_j
    // behind it, and the operator's `estop` behind that. The stale jog_j
    // sits between them deliberately — it is what the drain is entitled
    // to discard, and the assertion that it never replayed is also the
    // proof that the drain saw the `estop` and chose to keep it.
    c.send(&jog_l()).await;
    c.send(&jog_j()).await;
    let estop_req = c.send(&Command::Estop).await;

    // The estop is answered — it was neither dropped nor left unread.
    // (Against the blind drain there is no reply at all, and this waits
    // out its budget.)
    loop {
        match c.recv().await {
            Reply::Ok { req_id, .. } if req_id == estop_req => break,
            Reply::Error { req_id, error } if req_id == estop_req => {
                panic!("estop refused: {error:?}")
            }
            _ => {}
        }
    }

    // ...and it took effect: the RT was disabled and halted, and the
    // latch is standing.
    let ev = h
        .wait_rt(|ev| ev.contains(&RtEvent::SetEnabled(false)))
        .await;
    assert!(
        ev.contains(&RtEvent::Halt),
        "estop must halt motion: {ev:?}"
    );
    let err = c.expect_error(&move_j(511)).await;
    assert_eq!(
        err.code,
        ErrorCode::SysEstopActive as u16,
        "the e-stop latch must be standing after a preemption drain"
    );

    // The stale jog_j WAS discarded — that is what the drain is for, and
    // it only reads as discarded if the drain got to it before the estop
    // did, i.e. all three were queued together.
    let ev = h.rt_events();
    let jog_l_pos = ev
        .iter()
        .position(|e| *e == RtEvent::Stream(CmdType::JogL))
        .expect("the preempting jog_l reached the RT");
    assert!(
        !ev[jog_l_pos + 1..].contains(&RtEvent::Stream(CmdType::JogJ)),
        "a superseded jog_j must not replay (if it did, the three \
         datagrams were not buffered together and this test proved \
         nothing): {ev:?}"
    );
}

/// Chunked move_s: out-of-order reassembly hands the planner the exact
/// original command; a stalled transfer times out with
/// COMM_CHUNK_TIMEOUT on the transfer's req_id.
#[tokio::test]
async fn chunked_move_s_roundtrip_and_timeout() {
    let mut h = start(|_| {}).await;
    h.publish(|_| {});
    let mut c = Client::new(&h).await;

    let cmd = move_s(601, 300);
    let req_id = 77u32;
    let mut payload = Vec::new();
    encode_command(&cmd, req_id, &mut payload).unwrap();
    assert!(payload.len() > 8000, "bulk enough to need chunking");
    let chunks = split_into_chunks(req_id, 424242, &payload, 1024);
    assert!(chunks.len() > 4);
    let mut buf = Vec::new();
    for chunk in chunks.iter().rev() {
        encode_chunk(chunk, &mut buf);
        c.sock.send_to(&buf, c.server).await.unwrap();
    }
    let index = loop {
        match c.recv().await {
            Reply::Ok {
                req_id: id,
                index: Some(i),
            } => {
                assert_eq!(id, req_id);
                break i;
            }
            Reply::Complete { .. } => continue,
            other => panic!("expected chunked ack, got {other:?}"),
        }
    };
    // The planner received the byte-identical inner command.
    let deadline = tokio::time::Instant::now() + BUDGET;
    loop {
        let started = h.planner.lock().unwrap().started.clone();
        if let Some((i, got)) = started.last() {
            assert_eq!(*i, index);
            assert_eq!(*got, cmd, "reassembled command must round-trip exactly");
            break;
        }
        assert!(tokio::time::Instant::now() < deadline);
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
    h.complete_ok(index);
    let (ok, _) = c.wait_complete(index).await;
    assert!(ok);

    // Timeout: one chunk of three, then silence.
    let stalled = split_into_chunks(78, 555, &payload, (payload.len() / 3) + 1);
    assert_eq!(stalled.len(), 3);
    encode_chunk(&stalled[0], &mut buf);
    c.sock.send_to(&buf, c.server).await.unwrap();
    match c.recv().await {
        Reply::Error { req_id, error } => {
            assert_eq!(req_id, 78);
            assert_eq!(error.code, ErrorCode::CommChunkTimeout as u16);
            assert!(error.cause.contains("1/3"), "{}", error.cause);
        }
        other => panic!("expected chunk timeout, got {other:?}"),
    }
}

/// STATUS broadcast: v2 header content, seq/time monotonicity, snapshot
/// pass-through, planner enablement — and ALWAYS broadcasting with
/// link_ok=0 / growing data_age once snapshots stop.
#[tokio::test]
async fn status_broadcast_content_and_staleness() {
    let mut h = start(|_| {}).await;
    h.planner.lock().unwrap().enablement.joint_en = [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0];
    h.publish(|s| {
        s.q = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0];
        s.tcp = [0.1, 0.2, 0.3, TILTED_RPY[0], TILTED_RPY[1], TILTED_RPY[2]];
    });

    // Wait for a fresh-linked packet (the first may predate the publish).
    let deadline = tokio::time::Instant::now() + BUDGET;
    let s1 = loop {
        let s = recv_status(&h.status_rx).await;
        if s.link_ok == 1 {
            break s;
        }
        assert!(tokio::time::Instant::now() < deadline);
    };
    assert_eq!(s1.proto_version, 2);
    assert_eq!(s1.controller_id, 42);
    assert!((s1.angles[0] - 0.5f64.to_degrees()).abs() < 1e-9);
    assert!((s1.pose[3] - 100.0).abs() < 1e-9, "x translation in mm");
    assert!((s1.pose[7] - 200.0).abs() < 1e-9);
    assert!((s1.pose[11] - 300.0).abs() < 1e-9);
    // The rotation the snapshot's rpy names, composed independently: a
    // STATUS matrix built in the fixed-axis order instead would hand every
    // client an orientation 49° from this snapshot's.
    for (r, row) in intrinsic_xyz(TILTED_RPY).iter().enumerate() {
        for (c, want) in row.iter().enumerate() {
            let got = s1.pose[r * 4 + c];
            assert!(
                (got - want).abs() < 1e-12,
                "STATUS rotation [{r}][{c}] = {got}, want {want} (whole matrix {:?})",
                s1.pose
            );
        }
    }
    assert_eq!(s1.joint_en, [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0]);
    assert!(s1.homed);
    assert_eq!(s1.executing_index, -1);
    assert_eq!(s1.accepted_index, -1);
    assert!(s1.error.is_none());
    assert_eq!(s1.io[4], 1, "estop slot reads OK");

    let s2 = recv_status(&h.status_rx).await;
    assert!(s2.seq > s1.seq, "seq must increase");
    assert!(s2.mono_time_ns >= s1.mono_time_ns);

    // Stop publishing: broadcasts must keep coming, now flagged stale.
    let deadline = tokio::time::Instant::now() + BUDGET;
    let stale = loop {
        let s = recv_status(&h.status_rx).await;
        if s.link_ok == 0 {
            break s;
        }
        assert!(
            tokio::time::Instant::now() < deadline,
            "still link_ok after budget"
        );
    };
    assert!(
        stale.data_age_ms >= 100,
        "data_age reports the staleness: {}",
        stale.data_age_ms
    );
    assert!(stale.seq > s2.seq, "broadcast never went silent");
}

/// `link_ok` / `data_age_ms` / PING `hardware_connected` mean the MOTOR
/// BUS, not the RT snapshot channel: the RT publishes a snapshot every
/// tick whether or not any driver answered, so a silent bus behind a
/// healthy RT must read link-down — while STATUS keeps flowing.
///
/// (parol6's `hardware_connected` requires `first_frame_received` for the
/// same reason: a connected transport that has never produced a frame is
/// not a connected robot.)
#[tokio::test]
async fn silent_motor_bus_reads_link_down_while_status_keeps_flowing() {
    let mut h = start(|_| {}).await;
    let mut c = Client::new(&h).await;

    // Boot: the RT is alive and publishing, but no node has ever answered.
    h.publish(|s| {
        for node in &mut s.nodes {
            node.data_age_ticks = u64::MAX;
        }
    });
    let deadline = tokio::time::Instant::now() + BUDGET;
    let dead = loop {
        let s = recv_status(&h.status_rx).await;
        // Skip any frame that predates the publish (no snapshot yet also
        // reads link-down, so wait for the saturated age specifically).
        if s.data_age_ms == u16::MAX {
            break s;
        }
        assert!(
            tokio::time::Instant::now() < deadline,
            "never-seen bus never read as saturated data_age"
        );
    };
    assert_eq!(dead.link_ok, 0, "a bus that never spoke is not a link");
    match c.query(&Command::Ping).await {
        QueryResult::Ping { hardware_connected } => {
            assert!(!hardware_connected, "no frame ever arrived from a driver")
        }
        other => panic!("expected PING result, got {other:?}"),
    }

    // The bus comes up: fresh node data flips the link healthy. (Each
    // iteration re-publishes so the snapshot's own wall age never decides
    // the outcome on a slow runner.)
    let deadline = tokio::time::Instant::now() + BUDGET;
    loop {
        h.publish(|_| {});
        let s = recv_status(&h.status_rx).await;
        if s.link_ok == 1 {
            break;
        }
        assert!(
            tokio::time::Instant::now() < deadline,
            "fresh bus data never read as link_ok"
        );
    }
    h.publish(|_| {});
    match c.query(&Command::Ping).await {
        QueryResult::Ping { hardware_connected } => assert!(hardware_connected),
        other => panic!("expected PING result, got {other:?}"),
    }

    // Mid-session silence: the RT keeps publishing every tick (each
    // publish here is younger than `link_stale`), only the node ages
    // grow — 100 ticks at the default 250 Hz is 400 ms against the
    // harness's 100 ms staleness window. Before the fix this read
    // link_ok = 1 forever, because only the snapshot's wall age was
    // measured.
    let deadline = tokio::time::Instant::now() + BUDGET;
    let silent = loop {
        h.publish(|s| {
            for node in &mut s.nodes {
                node.data_age_ticks = 100;
            }
        });
        let s = recv_status(&h.status_rx).await;
        // The target frame carries the BUS age (100 ticks @ 250 Hz =
        // 400 ms), not merely a snapshot that aged in the rx backlog.
        if s.link_ok == 0 && s.data_age_ms >= 400 && s.data_age_ms != u16::MAX {
            break s;
        }
        assert!(
            tokio::time::Instant::now() < deadline,
            "a silent bus behind a live RT still read link_ok = 1 \
             (or data_age never carried the bus age)"
        );
    };
    assert!(silent.seq > dead.seq, "STATUS must keep flowing throughout");
    h.publish(|s| {
        for node in &mut s.nodes {
            node.data_age_ticks = 100;
        }
    });
    match c.query(&Command::Ping).await {
        QueryResult::Ping { hardware_connected } => {
            assert!(
                !hardware_connected,
                "a silent bus is not connected hardware"
            )
        }
        other => panic!("expected PING result, got {other:?}"),
    }
}

/// A refused fire-and-forget command must surface where a caller can see
/// it: the refusal latches as the standing error (STATUS + the ERROR
/// query), because nothing awaits the ERROR reply datagram itself
/// (issue #23). The next ACCEPTED motion command clears it, and a
/// refusal never latches over a running program.
#[tokio::test]
async fn refused_fire_and_forget_latches_the_standing_error_until_motion_is_accepted() {
    let mut h = start(|_| {}).await;
    h.publish(|_| {});
    let mut c = Client::new(&h).await;

    // Teleport outside sim mode: refused with a real ERROR reply...
    let err = c
        .expect_error(&Command::Teleport(Teleport {
            angles: [0.0; 6],
            tool_positions: None,
        }))
        .await;
    assert_eq!(err.code, ErrorCode::SysNotSimulator as u16);

    // ...and the refusal stands where a client that never awaited the
    // reply looks: the ERROR query and the broadcast.
    match c.query(&Command::Error).await {
        QueryResult::Error { error: Some(e) } => {
            assert_eq!(e.code, ErrorCode::SysNotSimulator as u16)
        }
        other => panic!("the refusal must stand in the ERROR query, got {other:?}"),
    }
    let deadline = tokio::time::Instant::now() + BUDGET;
    loop {
        let s = recv_status(&h.status_rx).await;
        if s.error
            .as_ref()
            .is_some_and(|e| e.code == ErrorCode::SysNotSimulator as u16)
        {
            break;
        }
        assert!(
            tokio::time::Instant::now() < deadline,
            "the refusal never reached the STATUS broadcast"
        );
    }

    // An accepted stream clears it, like every standing error.
    c.send(&jog_j()).await;
    h.wait_rt(|ev| ev.contains(&RtEvent::Stream(CmdType::JogJ)))
        .await;
    let deadline = tokio::time::Instant::now() + BUDGET;
    loop {
        match c.query(&Command::Error).await {
            QueryResult::Error { error: None } => break,
            QueryResult::Error { error: Some(_) } => {
                assert!(
                    tokio::time::Instant::now() < deadline,
                    "acceptance must clear the refusal latch"
                );
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            other => panic!("expected ERROR result, got {other:?}"),
        }
    }

    // While a stream is live, a stray refusal answers ERROR but does NOT
    // latch — it must not poison the running session's error surface.
    let err = c
        .expect_error(&Command::Teleport(Teleport {
            angles: [0.0; 6],
            tool_positions: None,
        }))
        .await;
    assert_eq!(err.code, ErrorCode::SysNotSimulator as u16);
    match c.query(&Command::Error).await {
        QueryResult::Error { error: None } => {}
        other => panic!("a refusal over live motion must not latch, got {other:?}"),
    }
}

/// A streaming setpoint the RUNTIME refuses (the bridge's collision gate
/// blocking a jog, issue #19) is a spoken refusal, not a silent drop:
/// the sender gets the runtime's ERROR, no stream session starts, and
/// the refusal latches as the standing error. A refusal of an in-place
/// UPDATE stops the stream it was updating — the previous setpoint must
/// not keep driving the arm while the client reads why its correction
/// was refused.
#[tokio::test]
async fn a_stream_the_runtime_refuses_answers_error_and_stops_the_session() {
    let mut h = start(|_| {}).await;
    h.publish(|_| {});
    let mut c = Client::new(&h).await;
    let collision = || {
        make_error(
            ErrorCode::SysSelfCollision,
            UNATTRIBUTED,
            &[("sample", "0"), ("total", "1"), ("pairs", "[j3, keepout]")],
        )
    };

    // Refused at admission: the ERROR reaches the sender, nothing was
    // forwarded to the RT, and the refusal stands in the ERROR query.
    h.rt.lock().unwrap().stream_verdict = Some(collision());
    let err = c.expect_error(&jog_j()).await;
    assert_eq!(err.code, ErrorCode::SysSelfCollision as u16);
    assert!(err.cause.contains("keepout"), "{err:?}");
    assert!(
        !h.rt_events().contains(&RtEvent::Stream(CmdType::JogJ)),
        "a refused setpoint must not reach the RT: {:?}",
        h.rt_events()
    );
    match c.query(&Command::Error).await {
        QueryResult::Error { error: Some(e) } => {
            assert_eq!(e.code, ErrorCode::SysSelfCollision as u16)
        }
        other => panic!("the refusal must stand in the ERROR query, got {other:?}"),
    }

    // The next ACCEPTED jog starts a session normally and clears the
    // latch — a refusal is a verdict about one setpoint, not a lockout.
    c.send(&jog_j()).await;
    h.wait_rt(|ev| ev.contains(&RtEvent::Stream(CmdType::JogJ)))
        .await;
    let deadline = tokio::time::Instant::now() + BUDGET;
    loop {
        match c.query(&Command::Error).await {
            QueryResult::Error { error: None } => break,
            QueryResult::Error { error: Some(_) } => {
                assert!(
                    tokio::time::Instant::now() < deadline,
                    "acceptance must clear the refusal latch"
                );
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            other => panic!("expected ERROR result, got {other:?}"),
        }
    }

    // A refused UPDATE of the live session cancels it: the arm stops
    // instead of continuing on the setpoint the client just replaced.
    let before = h.rt_events();
    assert!(
        !before.contains(&RtEvent::CancelStream),
        "premise: the session is live and uncancelled: {before:?}"
    );
    h.rt.lock().unwrap().stream_verdict = Some(collision());
    let err = c.expect_error(&jog_j()).await;
    assert_eq!(err.code, ErrorCode::SysSelfCollision as u16);
    h.wait_rt(|ev| ev.contains(&RtEvent::CancelStream)).await;
    // The session really ended: the next same-type jog is a fresh start,
    // which the double sees as a new Stream event after the cancel.
    c.send(&jog_j()).await;
    h.wait_rt(|ev| {
        let cancel = ev.iter().position(|e| *e == RtEvent::CancelStream);
        cancel.is_some_and(|i| ev[i..].contains(&RtEvent::Stream(CmdType::JogJ)))
    })
    .await;
}

/// Telemetry: unknown recipes are refused with COMM_UNKNOWN_RECIPE;
/// a selected recipe streams binary msgpack with seq + timestamp and
/// snapshot content.
#[tokio::test]
async fn telemetry_recipes_refusal_and_stream() {
    let mut h = start(|_| {}).await;
    h.publish(|s| {
        s.tick = 33;
        s.q = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
    });
    let mut c = Client::new(&h).await;

    let err = c
        .expect_error(&Command::SetRecipe(SetRecipe {
            name: "definitely-not-a-recipe".to_owned(),
        }))
        .await;
    assert_eq!(err.code, ErrorCode::CommUnknownRecipe as u16);
    assert!(err.cause.contains("definitely-not-a-recipe"));

    match c
        .request(&Command::SetRecipe(SetRecipe {
            name: "minimal".to_owned(),
        }))
        .await
    {
        Reply::Ok { index: None, .. } => {}
        other => panic!("expected OK, got {other:?}"),
    }

    let (name, seq1, ns1, tick, q) = recv_telemetry(&h.telemetry_rx).await;
    assert_eq!(name, "minimal");
    assert_eq!(tick, 33);
    assert_eq!(q, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    let (_, seq2, ns2, _, _) = recv_telemetry(&h.telemetry_rx).await;
    assert!(seq2 > seq1, "telemetry seq must increase");
    assert!(ns2 >= ns1);
}

fn wire_shape(name: &str, kind: &str) -> Shape {
    Shape {
        kind: kind.to_owned(),
        params: vec![0.2, 0.2, 0.2],
        pose: vec![0.3, 0.0, 0.1, 0.0, 0.0, 0.0],
        collision: true,
        margin: None,
        name: name.to_owned(),
    }
}

/// The collision-world seam the server owns: which layer goes to the
/// planner when, whose epoch STATUS reports, and what a refusal leaves
/// behind.
///
/// - the configured installation keep-outs are pushed ONCE, at startup,
///   and neither `set_shapes` nor `reset_state` ever touches that layer
///   again — a program cannot clear the deployment's keep-outs;
/// - `set_shapes` replaces the program layer and adopts the epoch of the
///   world the planner actually applied, instead of counting its own;
/// - a refused set changes nothing: not the enforced world, not the
///   SHAPES readback, not the epoch — so a client that sees the epoch
///   move knows its shapes are the ones being enforced;
/// - `reset_state` clears the program layer only;
/// - STATUS carries the planner's verdict rather than a hardcoded false.
#[tokio::test]
async fn shape_layers_epoch_adoption_and_collision_status() {
    let install = wire_shape("cage", "box");
    let mut h = start(|cfg| cfg.installation_shapes = vec![install.clone()]).await;
    h.publish(|_| {});
    let mut c = Client::new(&h).await;

    // Startup: installation only, and STATUS reports the applied epoch.
    assert_eq!(
        h.planner.lock().unwrap().layers,
        vec![(ShapeLayer::Installation, vec![install.clone()])],
        "installation keep-outs are pushed once, at startup"
    );
    let epoch_after_install = 1;

    let program = vec![wire_shape("table", "box"), wire_shape("post", "cylinder")];
    match c
        .request(&Command::SetShapes(SetShapes {
            shapes: program.clone(),
        }))
        .await
    {
        Reply::Ok { index: None, .. } => {}
        other => panic!("expected OK, got {other:?}"),
    }
    assert_eq!(
        h.planner.lock().unwrap().layers.last(),
        Some(&(ShapeLayer::Program, program.clone())),
        "set_shapes replaces the program layer"
    );
    match c.query(&Command::Shapes).await {
        QueryResult::Shapes {
            installation,
            program: p,
            epoch,
        } => {
            assert_eq!(installation, vec![install.clone()]);
            assert_eq!(p, program);
            assert_eq!(
                epoch,
                epoch_after_install + 1,
                "the reported epoch is the applied world's"
            );
        }
        other => panic!("unexpected {other:?}"),
    }

    // A refused set: the world, the readback and the epoch all stand.
    h.planner.lock().unwrap().fail_next_shapes = Some(make_error(
        ErrorCode::CommValidationError,
        UNATTRIBUTED,
        &[("detail", "shape \"bad\": unknown kind \"pyramid\"")],
    ));
    let err = c
        .expect_error(&Command::SetShapes(SetShapes {
            shapes: vec![wire_shape("bad", "pyramid")],
        }))
        .await;
    assert_eq!(err.code, ErrorCode::CommValidationError as u16);
    let applied = h.planner.lock().unwrap().layers.clone();
    assert_eq!(
        applied.last(),
        Some(&(ShapeLayer::Program, program.clone())),
        "a refused set must not reach the collision world"
    );
    match c.query(&Command::Shapes).await {
        QueryResult::Shapes {
            program: p, epoch, ..
        } => {
            assert_eq!(p, program, "a refused set must not change the readback");
            assert_eq!(
                epoch,
                epoch_after_install + 1,
                "a refused world must not advance scene_epoch"
            );
        }
        other => panic!("unexpected {other:?}"),
    }

    // reset_state clears the PROGRAM layer only.
    match c.request(&Command::ResetState).await {
        Reply::Ok { index: None, .. } => {}
        other => panic!("expected OK, got {other:?}"),
    }
    let applied = h.planner.lock().unwrap().layers.clone();
    assert_eq!(
        applied.last(),
        Some(&(ShapeLayer::Program, Vec::new())),
        "reset_state clears the program layer"
    );
    assert_eq!(
        applied
            .iter()
            .filter(|(l, _)| *l == ShapeLayer::Installation)
            .count(),
        1,
        "installation shapes are never re-pushed or cleared: {applied:?}"
    );
    match c.query(&Command::Shapes).await {
        QueryResult::Shapes {
            installation,
            program: p,
            ..
        } => {
            assert!(p.is_empty(), "reset_state clears the program readback");
            assert_eq!(
                installation,
                vec![install],
                "reset_state keeps the installation keep-outs"
            );
        }
        other => panic!("unexpected {other:?}"),
    }

    // STATUS reports what the planner says, not a hardcoded false.
    let s = recv_status(&h.status_rx).await;
    assert!(!s.collision_active);
    assert!(s.collision_pairs.is_empty());
    h.planner.lock().unwrap().collision = Some(CollisionState {
        active: true,
        pairs: vec![("forearm_0".to_owned(), "cage".to_owned())],
    });
    let deadline = tokio::time::Instant::now() + BUDGET;
    loop {
        let s = recv_status(&h.status_rx).await;
        if s.collision_active {
            assert_eq!(
                s.collision_pairs,
                vec![("forearm_0".to_owned(), "cage".to_owned())]
            );
            break;
        }
        assert!(
            tokio::time::Instant::now() < deadline,
            "the planner's collision verdict never reached STATUS"
        );
    }
}

/// A configured installation keep-out the runtime cannot apply is a
/// startup failure: coming up with an unenforceable world would silently
/// under-enforce, and no client is connected yet to be told.
#[tokio::test]
async fn unappliable_installation_shapes_fail_startup() {
    let status_rx = UdpSocket::bind("127.0.0.1:0").await.unwrap();
    let telemetry_rx = UdpSocket::bind("127.0.0.1:0").await.unwrap();
    let cfg = ServerConfig {
        bind: "127.0.0.1:0".parse().unwrap(),
        status_transport: StatusTransport::Unicast,
        status_dest_host: "127.0.0.1".parse().unwrap(),
        status_port: status_rx.local_addr().unwrap().port(),
        telemetry_port: telemetry_rx.local_addr().unwrap().port(),
        installation_shapes: vec![wire_shape("bad", "pyramid")],
        ..ServerConfig::default()
    };
    let (_writer, snapshots) = snapshot_channel::<StateSnapshot>();
    let planner = Arc::new(Mutex::new(PlannerState {
        fail_next_shapes: Some(make_error(
            ErrorCode::CommValidationError,
            UNATTRIBUTED,
            &[("detail", "shape \"bad\": unknown kind \"pyramid\"")],
        )),
        ..PlannerState::default()
    }));
    let started = spawn(
        cfg,
        RuntimeHandle {
            planner: TestPlanner(planner.clone()),
            rt: TestRt(Arc::new(Mutex::new(RtLog::default()))),
            snapshots,
        },
    )
    .await;
    let err = match started {
        Err(e) => e,
        Ok(_) => panic!("a refused installation layer must fail startup"),
    };
    assert!(
        err.to_string().contains("pyramid"),
        "the startup error names the offending shape: {err}"
    );
}

/// `write_io` is refused rather than acked into a fabricated readback.
///
/// Nothing in this runtime drives a digital output, so the only honest
/// answers are an ERROR for the command and an un-asserted level for the
/// two output slots — whichever port a client picks (the shipped Python
/// client sends the STATUS slot index, 2, for its logical output 0; the
/// wire's own numbering starts at 0).
#[tokio::test]
async fn write_io_is_refused_and_never_reported_as_a_driven_output() {
    let mut h = start(|_| {}).await;
    h.publish(|_| {});
    let mut c = Client::new(&h).await;

    for port in [0u8, 2] {
        let err = c
            .expect_error(&Command::WriteIo(WriteIo { port, value: 1 }))
            .await;
        assert_eq!(err.code, ErrorCode::CommValidationError as u16);
        assert!(
            err.cause.contains("digital output"),
            "the refusal says why: {}",
            err.cause
        );
    }

    match c.query(&Command::Io).await {
        QueryResult::Io { io } => {
            assert_eq!(io[2..4], [0, 0], "no output was driven, none is reported");
            assert_eq!(io[4], 1, "the e-stop slot is the one backed by a real line");
        }
        other => panic!("unexpected {other:?}"),
    }
    let status = recv_status(&h.status_rx).await;
    assert_eq!(status.io[2..4], [0, 0], "STATUS agrees with the IO query");

    assert!(
        !h.rt_events()
            .iter()
            .any(|e| matches!(e, RtEvent::WriteIo(..))),
        "a refused command reaches no backend: {:?}",
        h.rt_events()
    );
}

/// Cartesian freedom is reported only where a model backs it.
///
/// A runtime built without kinematics refuses every Cartesian command, so
/// claiming freedom in all twelve Cartesian directions is the one thing it
/// knows to be false — and the wire slot has no "unknown" to report
/// instead. Joint flags, which the planner really computes, pass through
/// either way, and so do the Cartesian ones once kinematics exist.
#[tokio::test]
async fn cartesian_freedom_is_reported_only_where_kinematics_exist() {
    let joints = [1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1];
    let mut h = start(|cfg| cfg.cartesian = false).await;
    h.planner.lock().unwrap().enablement.joint_en = joints;
    h.publish(|_| {});
    let mut c = Client::new(&h).await;

    // The premise: this runtime cannot execute a Cartesian command.
    let err = c.expect_error(&jog_l()).await;
    assert_eq!(err.code, ErrorCode::CommValidationError as u16);

    match c.query(&Command::Reachable).await {
        QueryResult::Reachable {
            joint_en,
            cart_en_wrf,
            cart_en_trf,
        } => {
            assert_eq!(
                joint_en, joints,
                "the joint model is real and passes through"
            );
            assert_eq!(cart_en_wrf, [0; 12], "no world-frame freedom is claimed");
            assert_eq!(cart_en_trf, [0; 12], "no tool-frame freedom is claimed");
        }
        other => panic!("unexpected {other:?}"),
    }
    let status = recv_status(&h.status_rx).await;
    assert_eq!(status.cart_en_wrf, [0; 12], "STATUS agrees with REACHABLE");
    assert_eq!(status.cart_en_trf, [0; 12]);
    assert_eq!(status.joint_en, joints);

    // With kinematics the planner's verdict is what goes on the wire —
    // the narrowing is conditional, not a blanket zero.
    let wrf = [1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0];
    let trf = [0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1];
    let mut h = start(|cfg| cfg.cartesian = true).await;
    {
        let mut p = h.planner.lock().unwrap();
        p.enablement.cart_en_wrf = wrf;
        p.enablement.cart_en_trf = trf;
    }
    h.publish(|_| {});
    let mut c = Client::new(&h).await;
    match c.query(&Command::Reachable).await {
        QueryResult::Reachable {
            cart_en_wrf,
            cart_en_trf,
            ..
        } => {
            assert_eq!(cart_en_wrf, wrf);
            assert_eq!(cart_en_trf, trf);
        }
        other => panic!("unexpected {other:?}"),
    }
}

/// Selecting a tool variant drops the TCP offset.
///
/// The offset is measured in the tool's own frame, so it means nothing once
/// a different variant moves that frame — the client API says a tool change
/// resets it, and the runtime has to make that true. Re-selecting the same
/// variant is not a change and leaves it alone.
#[tokio::test]
async fn selecting_a_different_variant_clears_the_tcp_offset() {
    let mut h = start(|cfg| {
        cfg.tools = vec!["gripper".to_owned()];
        cfg.fitted_tool = "gripper".to_owned();
        cfg.tool_dof = 1;
    })
    .await;
    h.publish(|_| {});
    let mut c = Client::new(&h).await;

    let select = |key: u64, variant: Option<&str>| {
        Command::SelectTool(par6_proto::command::SelectTool {
            key,
            tool_name: "GRIPPER".to_owned(),
            variant_key: variant.map(str::to_owned),
        })
    };
    let offset = |x: f64, y: f64, z: f64| {
        Command::SetTcpOffset(par6_proto::command::SetTcpOffset { x, y, z })
    };
    async fn read_offset(c: &mut Client) -> [f64; 3] {
        match c.query(&Command::TcpOffset).await {
            QueryResult::TcpOffset { x, y, z } => [x, y, z],
            other => panic!("unexpected {other:?}"),
        }
    }

    let i1 = c.ok_index(&select(701, Some("fin_ray"))).await;
    h.complete_ok(i1);
    c.wait_complete(i1).await;
    c.request(&offset(0.0, 0.0, -190.0)).await;
    assert_eq!(read_offset(&mut c).await, [0.0, 0.0, -190.0]);

    // Same variant again: nothing about the tool frame moved.
    let i2 = c.ok_index(&select(702, Some("fin_ray"))).await;
    h.complete_ok(i2);
    c.wait_complete(i2).await;
    assert_eq!(read_offset(&mut c).await, [0.0, 0.0, -190.0]);

    // A different variant moves the TCP the offset was measured from.
    let i3 = c.ok_index(&select(703, Some("wide_jaw"))).await;
    h.complete_ok(i3);
    c.wait_complete(i3).await;
    assert_eq!(read_offset(&mut c).await, [0.0, 0.0, 0.0]);
}

/// Gaps 2 + 3, together: the controller comes up ready to accept motion,
/// `reset` reports what the RT actually did with the enable, and a hard
/// error the RT latched on its own reaches the client.
///
/// Measured against the runtime before this landed: par6d booted DISABLED
/// and refused every queued command with `SYS_CONTROLLER_DISABLED` until
/// someone sent `reset` — which no frontend does at startup; `reset`
/// answered OK whether or not the RT accepted the enable; and with a hard
/// RT latch standing the client was told `error() -> None`,
/// `activity() -> IDLE`, `action_state = IDLE` while every command was
/// being refused. An arm that is bricked and says it is idle is the one
/// state an operator cannot diagnose.
#[tokio::test]
async fn boot_enables_reset_reports_the_rt_verdict_and_rt_latches_reach_the_client() {
    let mut h = start(|_| {}).await;
    let mut c = Client::new(&h).await;

    // ---- boot: leaving BOOTING enables the controller, unasked.
    h.publish(|s| s.mode = Mode::Idle);
    let ev = h
        .wait_rt(|ev| ev.contains(&RtEvent::SetEnabled(true)))
        .await;
    assert_eq!(
        ev.iter()
            .filter(|e| matches!(e, RtEvent::SetEnabled(_)))
            .count(),
        1,
        "exactly one enable, and no client asked for it: {ev:?}"
    );
    // Motion is accepted straight away — no client had to press Reset.
    let i = c.ok_index(&move_j(700)).await;
    h.complete_ok(i);
    assert!(c.wait_complete(i).await.0);

    // ---- a hard error the RT latched on its own: no command earned it,
    // so nothing in the command plane knows about it but the snapshot.
    h.publish(|s| {
        s.mode = Mode::ActiveError;
        s.state = ArmState::Disabled;
        s.errors.insert(ErrorEntry {
            code: RtCode::RtiLinkLost,
            joint: None,
        });
        s.error_active = true;
    });

    let err = c.expect_error(&move_j(701)).await;
    assert_eq!(err.code, ErrorCode::SysControllerDisabled as u16);
    match c.query(&Command::Error).await {
        QueryResult::Error { error: Some(e) } => assert_eq!(
            e.code,
            ErrorCode::SysRtiLinkLost as u16,
            "the ERROR query must name the latch, got {e:?}"
        ),
        other => panic!("a latched RT error must reach the ERROR query, got {other:?}"),
    }
    match c.query(&Command::Activity).await {
        QueryResult::Activity { state, .. } => assert_eq!(
            state,
            ActionState::Error,
            "activity must not read IDLE while the arm is latched"
        ),
        other => panic!("unexpected {other:?}"),
    }
    let deadline = tokio::time::Instant::now() + BUDGET;
    loop {
        let s = recv_status(&h.status_rx).await;
        if s.error
            .as_ref()
            .is_some_and(|e| e.code == ErrorCode::SysRtiLinkLost as u16)
            && s.action_state == ActionState::Error
        {
            break;
        }
        assert!(
            tokio::time::Instant::now() < deadline,
            "STATUS never carried the latched RT error; last: {:?}",
            s.error
        );
    }

    // ---- reset while the RT still refuses: the refusal is the answer.
    h.rt.lock().unwrap().enable_verdict = Some(make_error(
        ErrorCode::SysControllerDisabled,
        UNATTRIBUTED,
        &[("detail", "still latched")],
    ));
    match c.request(&Command::Reset).await {
        Reply::Error { error, .. } => assert_eq!(
            error.code,
            ErrorCode::SysControllerDisabled as u16,
            "a refused enable must not be reported as a successful reset"
        ),
        other => panic!("expected the RT's refusal, got {other:?}"),
    }
    // The refusal did not paper over the latch either.
    match c.query(&Command::Error).await {
        QueryResult::Error { error: Some(e) } => {
            assert_eq!(e.code, ErrorCode::SysRtiLinkLost as u16)
        }
        other => panic!("the latch outlives a failed reset, got {other:?}"),
    }

    // ---- reset once the RT really enables: OK, and the error is gone
    // because the LATCH is gone, not because the reset cleared a field.
    h.rt.lock().unwrap().enable_verdict = None;
    h.publish(|s| s.mode = Mode::Idle);
    match c.request(&Command::Reset).await {
        Reply::Ok { index: None, .. } => {}
        other => panic!("expected a confirmed reset, got {other:?}"),
    }
    match c.query(&Command::Error).await {
        QueryResult::Error { error } => assert!(error.is_none(), "{error:?}"),
        other => panic!("unexpected {other:?}"),
    }
    c.ok_index(&move_j(702)).await;
}

/// Blend lookahead: a move that asks to round a corner is held at the
/// head of the queue until its successor is there to round it INTO, the
/// planner is offered both, and the pair it folds into one motion
/// completes together.
///
/// The queue engine used to hand the planner exactly one command at a
/// time, so a corner radius had nowhere to reach and was refused at
/// accept time. What is asserted here is the contract that replaced it,
/// including the completion-index semantics protocol v2
/// prescribes for blended-away commands: one COMPLETE per queued
/// command, all of them at the end of the motion that carried them, and
/// a high-water `completed_index` on the LAST of them.
#[tokio::test]
async fn blend_lookahead_holds_the_head_and_completes_the_chain_together() {
    let mut h = start(|cfg| {
        cfg.blend_hold = Duration::from_millis(150);
        cfg.blend_lookahead = 4;
    })
    .await;
    h.publish(|_| {});
    let mut c = Client::new(&h).await;
    // This planner double folds two commands into every motion.
    h.planner.lock().unwrap().consume = 2;

    // A move that wants to blend is NOT started on its own: there is no
    // corner until the next move arrives.
    let i1 = c.ok_index(&move_j_blended(201, Some(15.0))).await;
    tokio::time::sleep(Duration::from_millis(40)).await;
    assert!(
        h.planner.lock().unwrap().started.is_empty(),
        "a blended move must wait for the successor it is meant to blend into"
    );

    // Its successor arrives, and the pair starts as one batch.
    let i2 = c.ok_index(&move_j_blended(202, None)).await;
    let deadline = tokio::time::Instant::now() + BUDGET;
    loop {
        let seen = {
            let s = h.planner.lock().unwrap();
            s.batches
                .last()
                .map(|batch| (batch.clone(), s.started.len()))
        };
        if let Some((batch, started)) = seen {
            assert_eq!(
                batch,
                vec![i1, i2],
                "the planner must see the whole runnable chain, not just its head"
            );
            assert_eq!(started, 1, "one motion covers both commands");
            break;
        }
        assert!(
            tokio::time::Instant::now() < deadline,
            "chain never started"
        );
        tokio::time::sleep(Duration::from_millis(2)).await;
    }

    // Nothing completes until the motion does — and then both do.
    match c.query(&Command::Queue).await {
        QueryResult::Queue {
            queue,
            executing_index,
            completed_index,
            ..
        } => {
            assert!(queue.is_empty(), "both commands left the pending queue");
            assert_eq!(executing_index, i1 as i64);
            assert_eq!(completed_index, -1);
        }
        other => panic!("unexpected {other:?}"),
    }
    h.complete_ok(i1);
    let (ok, _) = c.wait_complete(i1).await;
    assert!(ok);
    let (ok, _) = c.wait_complete(i2).await;
    assert!(ok, "the blended-away command reports its own COMPLETE");
    match c.query(&Command::Queue).await {
        QueryResult::Queue {
            executing_index,
            completed_index,
            ..
        } => {
            assert_eq!(executing_index, -1);
            assert_eq!(
                completed_index, i2 as i64,
                "the high-water completed index is the LAST command of the blend"
            );
        }
        other => panic!("unexpected {other:?}"),
    }

    // The hold is bounded: a blended move with no successor coming runs
    // on its own rather than sitting in the queue forever.
    let i3 = c.ok_index(&move_j_blended(203, Some(15.0))).await;
    let deadline = tokio::time::Instant::now() + BUDGET;
    loop {
        let ran = {
            let s = h.planner.lock().unwrap();
            s.started.iter().any(|(i, _)| *i == i3)
        };
        if ran {
            break;
        }
        assert!(
            tokio::time::Instant::now() < deadline,
            "a blended move whose successor never came must still run"
        );
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    h.complete_ok(i3);
    let (ok, _) = c.wait_complete(i3).await;
    assert!(ok);

    h.server.shutdown();
}

fn move_j_blended(key: u64, r: Option<f64>) -> Command {
    Command::MoveJ(MoveJ {
        key,
        angles: [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        duration: Some(0.5),
        speed: None,
        accel: None,
        blend_radius: r,
        rel: false,
    })
}

/// The queue ETA on the wire is the motion in flight plus the queue
/// standing behind it, and it is re-estimated only when the queue
/// actually changes — estimating means planning, which must not run at
/// the status cadence.
#[tokio::test]
async fn queue_eta_adds_the_inflight_motion_to_the_pending_estimate() {
    let mut h = start(|_| {}).await;
    h.publish(|_| {});
    h.planner.lock().unwrap().inflight_duration = 2.0;
    let mut c = Client::new(&h).await;

    // Nothing queued: the ETA is whatever is still running.
    let idle = match c.query(&Command::Queue).await {
        QueryResult::Queue {
            queued_duration, ..
        } => queued_duration,
        other => panic!("unexpected queue result {other:?}"),
    };
    assert!((idle - 2.0).abs() < 1e-9, "in-flight only, got {idle}");

    // Three moves: the first starts (leaving the planner's in-flight
    // duration to describe it), two stay pending at 0.5 s each.
    for key in 1..=3u64 {
        c.ok_index(&move_j(key)).await;
    }
    let queued = match c.query(&Command::Queue).await {
        QueryResult::Queue {
            queued_duration,
            queue,
            ..
        } => {
            assert_eq!(queue.len(), 2, "one move started, two are pending");
            queued_duration
        }
        other => panic!("unexpected queue result {other:?}"),
    };
    assert!(
        (queued - 3.0).abs() < 1e-9,
        "2 s in flight + 2 × 0.5 s queued, got {queued}"
    );

    // The STATUS broadcast carries the same number...
    let deadline = tokio::time::Instant::now() + BUDGET;
    loop {
        let s = recv_status(&h.status_rx).await;
        if s.queued_segments == 2 {
            assert!((s.queued_duration - 3.0).abs() < 1e-9);
            break;
        }
        assert!(
            tokio::time::Instant::now() < deadline,
            "no status frame with the queued moves"
        );
    }
    // ... without asking the planner to re-plan for every frame.
    let before = h.planner.lock().unwrap().estimates;
    for _ in 0..3 {
        recv_status(&h.status_rx).await;
    }
    assert_eq!(
        h.planner.lock().unwrap().estimates,
        before,
        "an unchanged queue must not be re-estimated"
    );
}

/// The arm's fully-limp state, reachable from the wire.
///
/// The RT has had the control law (`law_safety_stop` fills every joint
/// with torque-only zero) and an unconditional transition into it since
/// the core was written — but no command tag reached it, so an operator
/// who needed the arm limp to free a trapped person or a jammed joint had
/// nothing to send. Unlike the protective stop, which holds position under
/// power, this drops drive authority.
#[tokio::test]
async fn safety_stop_is_reachable_and_halts_motion() {
    let mut h = start(|_| {}).await;
    h.publish(|_| {});
    let mut c = Client::new(&h).await;

    assert!(
        matches!(c.request(&Command::SafetyStop).await, Reply::Ok { .. }),
        "SAFETY_STOP must be acked as a SYSTEM command"
    );
    let ev = h.wait_rt(|ev| ev.contains(&RtEvent::SafetyStop)).await;
    assert!(
        ev.contains(&RtEvent::Halt),
        "going limp must halt motion first: {ev:?}"
    );
}

/// The gravity-comp feedforward is switchable from the wire.
///
/// `SetGravityComp` existed as an internal RT command with no client-facing
/// sender, so once plain `--sim` turned the feedforward off at boot,
/// nothing could turn it back on — par6d's own comment said as much.
#[tokio::test]
async fn gravity_compensation_can_be_switched_from_the_wire() {
    let mut h = start(|_| {}).await;
    h.publish(|_| {});
    let mut c = Client::new(&h).await;

    for on in [true, false] {
        assert!(
            matches!(
                c.request(&Command::SetGravityComp(
                    par6_proto::command::SetGravityComp { on }
                ))
                .await,
                Reply::Ok { .. }
            ),
            "SET_GRAVITY_COMP({on}) must be acked"
        );
        h.wait_rt(|ev| ev.contains(&RtEvent::SetGravityComp(on)))
            .await;
    }
}
