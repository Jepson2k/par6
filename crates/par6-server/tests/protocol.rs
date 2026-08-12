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
    CmdType, Command, ErrorCode, Frame, QueryResult, Reply, WireError, UNATTRIBUTED,
};
use par6_rt::{snapshot_channel, ArmState, SnapshotWriter, StateSnapshot};
use par6_server::{
    spawn, CollisionState, CommandOutcome, Enablement, PlanContext, Planner, RtCommands,
    RuntimeHandle, ServerConfig, ServerHandle, ShapeLayer, StatusTransport,
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
}

#[derive(Clone)]
struct TestRt(Arc<Mutex<RtLog>>);

impl TestRt {
    fn push(&self, e: RtEvent) {
        self.0.lock().unwrap().events.push(e);
    }
}

impl RtCommands for TestRt {
    fn stream(&mut self, cmd: &Command) {
        self.push(RtEvent::Stream(cmd.tag()));
    }
    fn cancel_stream(&mut self) {
        self.push(RtEvent::CancelStream);
    }
    fn halt(&mut self) {
        self.push(RtEvent::Halt);
    }
    fn set_enabled(&mut self, enabled: bool) {
        self.push(RtEvent::SetEnabled(enabled));
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
}

#[derive(Clone)]
struct TestPlanner(Arc<Mutex<PlannerState>>);

impl Planner for TestPlanner {
    fn start(&mut self, index: u64, cmd: &Command) -> Result<(), WireError> {
        let mut s = self.0.lock().unwrap();
        if let Some(e) = s.fail_next_start.take() {
            return Err(e);
        }
        s.started.push((index, cmd.clone()));
        Ok(())
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
    /// Publish a fresh snapshot (enabled + homed unless overridden).
    /// The server re-reads the snapshot channel on every datagram, so a
    /// command sent after this sees the new state.
    fn publish(&mut self, f: impl FnOnce(&mut StateSnapshot)) {
        self.tick += 1;
        let mut s = StateSnapshot {
            tick: self.tick,
            state: ArmState::Enabled,
            homed: true,
            ..StateSnapshot::default()
        };
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
}

/// The gating table over the wire: un-homed, disabled, e-stop latch and
/// simulator-only rejections carry their specific error codes; homing
/// and jogging stay available un-homed.
#[tokio::test]
async fn gating_rejections_carry_specific_codes() {
    let mut h = start(|_| {}).await;
    let mut c = Client::new(&h).await;

    // Un-homed: planned motion refused, home + jog accepted.
    h.publish(|s| s.homed = false);
    let err = c.expect_error(&move_j(301)).await;
    assert_eq!(err.code, ErrorCode::MotnNotHomed as u16);
    let home_idx = c
        .ok_index(&Command::Home(par6_proto::command::Home { key: 302 }))
        .await;
    assert!(home_idx >= 1);
    c.send(&jog_j()).await; // fire-and-forget: success is unacked
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
        s.tcp = [0.1, 0.2, 0.3, 0.0, 0.0, 0.0];
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
    assert_eq!(s1.pose[0], 1.0, "identity rotation at zero rpy");
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
