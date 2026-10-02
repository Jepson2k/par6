//! Transport core: req-id correlation, retries, chunking, the status
//! subscription, and completion bookkeeping. The public method surface
//! lives in `api.rs`; everything here is the machinery under it.
//!
//! Contract ported from the reference client (`python/par6/client/
//! async_client.py`): replies are matched by echoed req_id, never arrival
//! order; queries retry under the same req_id; QUEUED commands retry under
//! the same idempotency key (the runtime's dedup window re-acks the
//! original index); SYSTEM commands are one send + wait; fire-and-forget
//! commands are validated locally and sent without a wait.

use std::collections::HashMap;
use std::net::Ipv4Addr;
use std::sync::atomic::{AtomicBool, AtomicI64, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use par6_proto::{
    decode_reply, decode_status, encode_chunk, encode_command, split_into_chunks, Command, Reply,
    Status, WireError, COMPLETIONS_KEPT,
};
use tokio::net::UdpSocket;
use tokio::sync::{oneshot, watch};
use tokio::task::JoinHandle;

use crate::error::ClientError;
use crate::sockets;

/// How often one error code may be logged for a reply nobody awaits.
const UNCLAIMED_ERROR_PERIOD: Duration = Duration::from_secs(1);

/// The smallest MTU the client accepts: the chunk envelope plus room for
/// a payload.
pub const MIN_MTU: usize = 64;

/// Bytes of every chunk datagram reserved for the chunk envelope.
const CHUNK_OVERHEAD: usize = 32;

/// After STATUS reports a command finished, how long its COMPLETE push
/// is still awaited before the verdict is declared unknown. The push is
/// sent before the STATUS frame that reflects it, so this only covers
/// reordering on the wire.
const COMPLETE_GRACE: Duration = Duration::from_millis(100);

/// How the STATUS broadcast is subscribed.
#[derive(Debug, Clone)]
pub enum StatusTransport {
    /// Multicast join with unicast fallback (the default ladder).
    Multicast {
        /// Multicast group address.
        group: Ipv4Addr,
        /// Interface address to join on first.
        iface: Ipv4Addr,
        /// Local address bound when no interface can join the group.
        fallback: Ipv4Addr,
    },
    /// Plain unicast bind.
    Unicast {
        /// Local address to bind.
        host: Ipv4Addr,
    },
}

/// Client configuration. `default()` reads the same `PAR6_*` environment
/// variables the reference client honors.
#[derive(Debug, Clone)]
pub struct ClientConfig {
    /// Runtime host.
    pub host: String,
    /// Command UDP port.
    pub port: u16,
    /// Per-attempt reply timeout.
    pub timeout: Duration,
    /// Extra attempts for queries and queued commands.
    pub retries: u32,
    /// STATUS subscription transport.
    pub status: StatusTransport,
    /// STATUS broadcast port.
    pub status_port: u16,
    /// Datagrams above this size are chunked.
    pub mtu: usize,
}

fn env_str(name: &str, default: &str) -> String {
    std::env::var(name).unwrap_or_else(|_| default.to_string())
}

fn env_parse<T: std::str::FromStr>(name: &str, default: T) -> T {
    std::env::var(name)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

impl Default for ClientConfig {
    fn default() -> Self {
        let kind = env_str("PAR6_STATUS_TRANSPORT", "MULTICAST").to_uppercase();
        let unicast_host = env_parse("PAR6_STATUS_UNICAST_HOST", Ipv4Addr::LOCALHOST);
        let status = if kind == "UNICAST" {
            StatusTransport::Unicast { host: unicast_host }
        } else {
            StatusTransport::Multicast {
                group: env_parse("PAR6_STATUS_MCAST_GROUP", Ipv4Addr::new(239, 255, 0, 71)),
                iface: env_parse("PAR6_STATUS_MCAST_IF", Ipv4Addr::LOCALHOST),
                fallback: unicast_host,
            }
        };
        Self {
            host: env_str("PAR6_HOST", "127.0.0.1"),
            port: env_parse("PAR6_COMMAND_PORT", 6001),
            timeout: Duration::from_secs_f64(1.0),
            retries: 1,
            status,
            status_port: env_parse("PAR6_STATUS_PORT", 6002),
            mtu: env_parse("PAR6_MTU", 1400),
        }
    }
}

/// Outcome of a SYSTEM command whose success reply may be lost on the wire.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ack {
    /// The runtime acked the command.
    Confirmed,
    /// No reply arrived; the command may or may not have been applied.
    Unconfirmed,
}

/// A finished queued command: success flag, failure detail, and the tool
/// settle verdict a successful gripper move completes with (1 = object
/// while closing, 2 = object while opening, 3 = target reached, no
/// object).
pub type Completion = (bool, Option<WireError>, Option<u8>);

struct Completions {
    /// Finished commands by index, insertion-ordered for eviction.
    log: HashMap<u64, Completion>,
    order: std::collections::VecDeque<u64>,
    waiters: HashMap<u64, Vec<oneshot::Sender<Completion>>>,
}

pub(crate) struct Inner {
    pub(crate) cfg: ClientConfig,
    sock: UdpSocket,
    pending: Mutex<HashMap<u32, oneshot::Sender<Reply>>>,
    completions: Mutex<Completions>,
    req_id: AtomicU32,
    transfer_id: AtomicU32,
    key_state: AtomicU64,
    pub(crate) status_tx: watch::Sender<Option<Arc<Status>>>,
    last_seq: Mutex<Option<(u64, u64)>>,
    seq_gaps: AtomicU64,
    unclaimed: Mutex<HashMap<u16, std::time::Instant>>,
    pub(crate) last_command_index: AtomicI64,
    /// The protocol version a STATUS frame declared when it was not this
    /// client's; latched, because nothing later can be read either.
    skew: Mutex<Option<u8>>,
    /// Fault injection for the client's own tests: discard COMPLETE
    /// pushes as they arrive, so a wait has to finish the other way.
    drop_complete_pushes: AtomicBool,
    /// The completion policy this client last set on its session, if
    /// any — the wire has no query for it, and a routine that changes it
    /// for its own run has to know what to put back.
    pub(crate) completion_policy: Mutex<Option<par6_proto::CompletionPolicy>>,
    closed: AtomicBool,
}

/// The async par6 client. Cheap to clone; all clones share one transport.
#[derive(Clone)]
pub struct Client {
    pub(crate) inner: Arc<Inner>,
    tasks: Arc<Mutex<Vec<JoinHandle<()>>>>,
}

impl Client {
    /// Connect with [`ClientConfig::default`] (environment-driven).
    pub async fn connect_default() -> Result<Self, ClientError> {
        Self::connect(ClientConfig::default()).await
    }

    /// Bind the command endpoint and start the reply + status listeners.
    /// `Invalid` for an MTU below [`MIN_MTU`].
    pub async fn connect(cfg: ClientConfig) -> Result<Self, ClientError> {
        if cfg.mtu < MIN_MTU {
            return Err(ClientError::Invalid(format!(
                "mtu {} is below the {MIN_MTU}-byte minimum",
                cfg.mtu
            )));
        }
        let sock = UdpSocket::bind("0.0.0.0:0").await?;
        sock.connect((cfg.host.as_str(), cfg.port)).await?;

        let status_std = match cfg.status {
            StatusTransport::Multicast {
                group,
                iface,
                fallback,
            } => sockets::multicast_socket(group, cfg.status_port, iface).or_else(|e| {
                log::warn!(
                    "multicast status subscription failed ({e}); falling back to unicast on {fallback}"
                );
                sockets::unicast_socket(fallback, cfg.status_port)
            })?,
            StatusTransport::Unicast { host } => sockets::unicast_socket(host, cfg.status_port)?,
        };
        let status_sock = UdpSocket::from_std(status_std)?;

        let seed = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(1);
        let (status_tx, _) = watch::channel(None);
        let inner = Arc::new(Inner {
            cfg,
            sock,
            pending: Mutex::new(HashMap::new()),
            completions: Mutex::new(Completions {
                log: HashMap::new(),
                order: std::collections::VecDeque::new(),
                waiters: HashMap::new(),
            }),
            req_id: AtomicU32::new((seed as u32) | 1),
            transfer_id: AtomicU32::new(seed as u32),
            key_state: AtomicU64::new(seed | 1),
            status_tx,
            last_seq: Mutex::new(None),
            seq_gaps: AtomicU64::new(0),
            unclaimed: Mutex::new(HashMap::new()),
            last_command_index: AtomicI64::new(-1),
            skew: Mutex::new(None),
            drop_complete_pushes: AtomicBool::new(false),
            completion_policy: Mutex::new(None),
            closed: AtomicBool::new(false),
        });

        let reply_task = tokio::spawn(reply_rx(inner.clone()));
        let status_task = tokio::spawn(status_rx(inner.clone(), status_sock));
        Ok(Client {
            inner,
            tasks: Arc::new(Mutex::new(vec![reply_task, status_task])),
        })
    }

    /// Stop the listeners and wake every waiter. Safe to call repeatedly.
    pub fn close(&self) {
        if self.inner.closed.swap(true, Ordering::SeqCst) {
            return;
        }
        for task in self.tasks.lock().unwrap().iter() {
            task.abort();
        }
        self.inner.pending.lock().unwrap().clear();
        self.inner.completions.lock().unwrap().waiters.clear();
        // Wake status waiters so they observe the closed flag.
        self.inner.status_tx.send_modify(|_| {});
    }

    /// Wait for the aborted listener tasks to wind down.
    ///
    /// `close` only requests the abort; the tasks' teardown still runs on
    /// a runtime worker afterwards. A caller that is about to let the
    /// process exit must wait it out — a worker mid-teardown while the C
    /// runtime tears the process down dies with a non-unwinding panic.
    pub async fn close_joined(&self) {
        self.close();
        let tasks: Vec<_> = self.tasks.lock().unwrap().drain(..).collect();
        for task in tasks {
            let _ = task.await;
        }
    }

    /// The configuration this client connected with.
    pub fn config(&self) -> &ClientConfig {
        &self.inner.cfg
    }

    /// Whether [`Client::close`] has been called.
    pub fn is_closed(&self) -> bool {
        self.inner.closed.load(Ordering::SeqCst)
    }

    /// Total STATUS packets lost so far, from header `seq` gaps.
    pub fn status_seq_gaps(&self) -> u64 {
        self.inner.seq_gaps.load(Ordering::Relaxed)
    }

    /// The queue index of the most recently acked queued command.
    pub fn last_command_index(&self) -> Option<u64> {
        let v = self.inner.last_command_index.load(Ordering::Relaxed);
        u64::try_from(v).ok()
    }

    fn next_req_id(&self) -> u32 {
        loop {
            let id = self.inner.req_id.fetch_add(1, Ordering::Relaxed);
            // 0 is the unsolicited-push id; never issue it.
            if id != 0 && !self.inner.pending.lock().unwrap().contains_key(&id) {
                return id;
            }
        }
    }

    /// A fresh 64-bit idempotency key (xorshift over a time-seeded
    /// state) — what the named queued-command methods stamp; callers
    /// building [`Command`] values directly use it the same way.
    pub fn fresh_key(&self) -> u64 {
        let mut x = self.inner.key_state.load(Ordering::Relaxed);
        loop {
            let mut n = x;
            n ^= n << 13;
            n ^= n >> 7;
            n ^= n << 17;
            match self.inner.key_state.compare_exchange_weak(
                x,
                n,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => return n,
                Err(cur) => x = cur,
            }
        }
    }

    fn datagrams(&self, data: Vec<u8>, req_id: u32) -> Vec<Vec<u8>> {
        if data.len() <= self.inner.cfg.mtu {
            return vec![data];
        }
        let transfer_id = self.inner.transfer_id.fetch_add(1, Ordering::Relaxed);
        let chunk = self.inner.cfg.mtu.saturating_sub(CHUNK_OVERHEAD).max(1);
        split_into_chunks(req_id, transfer_id, &data, chunk)
            .iter()
            .map(|c| {
                let mut buf = Vec::new();
                encode_chunk(c, &mut buf);
                buf
            })
            .collect()
    }

    async fn roundtrip(
        &self,
        datagrams: &[Vec<u8>],
        req_id: u32,
        attempts: u32,
    ) -> Result<Option<Reply>, ClientError> {
        if self.is_closed() {
            return Err(ClientError::Closed);
        }
        let (tx, mut rx) = oneshot::channel();
        self.inner.pending.lock().unwrap().insert(req_id, tx);
        let result = async {
            for attempt in 0..attempts {
                for datagram in datagrams {
                    self.inner.sock.send(datagram).await?;
                }
                match tokio::time::timeout(self.inner.cfg.timeout, &mut rx).await {
                    Ok(Ok(reply)) => return Ok(Some(reply)),
                    Ok(Err(_)) => return Err(ClientError::Closed),
                    Err(_) if attempt + 1 < attempts => {
                        // Deterministic backoff with a key-derived jitter.
                        let base = (0.05 * 2f64.powi(attempt as i32)).min(0.5);
                        let jitter = (self.fresh_key() % 50) as f64 / 1000.0;
                        tokio::time::sleep(Duration::from_secs_f64(base + jitter)).await;
                        if let Ok(reply) = rx.try_recv() {
                            return Ok(Some(reply));
                        }
                    }
                    Err(_) => {}
                }
            }
            Ok(None)
        }
        .await;
        self.inner.pending.lock().unwrap().remove(&req_id);
        result
    }

    fn encode(&self, cmd: &Command, req_id: u32) -> Result<Vec<u8>, ClientError> {
        let mut buf = Vec::new();
        encode_command(cmd, req_id, &mut buf).map_err(|e| match e {
            // The codec validates on encode with the same table the
            // runtime decodes with, so a locally refused command carries
            // the exact structured error the runtime would answer.
            par6_proto::DecodeError::Validation { .. } => {
                ClientError::Robot(par6_proto::make_error(
                    par6_proto::ErrorCode::CommValidationError,
                    par6_proto::UNATTRIBUTED,
                    &[("detail", &e.to_string())],
                ))
            }
            other => ClientError::Decode(other),
        })?;
        Ok(buf)
    }

    /// QUERY roundtrip with retries. `Unreachable` when no reply arrives;
    /// `Robot` on an ERROR reply.
    pub async fn query(&self, cmd: Command) -> Result<par6_proto::QueryResult, ClientError> {
        let req_id = self.next_req_id();
        let data = self.encode(&cmd, req_id)?;
        let datagrams = self.datagrams(data, req_id);
        match self
            .roundtrip(&datagrams, req_id, 1 + self.inner.cfg.retries)
            .await?
        {
            None => Err(ClientError::Unreachable),
            Some(Reply::Error { error, .. }) => Err(ClientError::Robot(error)),
            Some(Reply::Response { result, .. }) => Ok(result),
            Some(other) => {
                log::debug!("query got unexpected reply {other:?}");
                Err(ClientError::Unreachable)
            }
        }
    }

    /// SYSTEM roundtrip: one send + wait. `Robot` on rejection.
    pub async fn system(&self, cmd: Command) -> Result<Ack, ClientError> {
        let req_id = self.next_req_id();
        let data = self.encode(&cmd, req_id)?;
        let datagrams = self.datagrams(data, req_id);
        match self.roundtrip(&datagrams, req_id, 1).await? {
            None => Ok(Ack::Unconfirmed),
            Some(Reply::Error { error, .. }) => Err(ClientError::Robot(error)),
            Some(_) => Ok(Ack::Confirmed),
        }
    }

    /// QUEUED roundtrip: idempotency-keyed, retried. `Ok(Some(index))` on
    /// ack, `Ok(None)` when unconfirmed, `Robot` on rejection. The caller
    /// stamps the key with [`Client::fresh_key`] before building `cmd`.
    pub async fn queued(&self, cmd: Command) -> Result<Option<u64>, ClientError> {
        let req_id = self.next_req_id();
        let data = self.encode(&cmd, req_id)?;
        let datagrams = self.datagrams(data, req_id);
        match self
            .roundtrip(&datagrams, req_id, 1 + self.inner.cfg.retries)
            .await?
        {
            None => Ok(None),
            Some(Reply::Error { error, .. }) => Err(ClientError::Robot(error)),
            Some(Reply::Ok {
                index: Some(index), ..
            }) => {
                self.inner
                    .last_command_index
                    .store(index as i64, Ordering::Relaxed);
                Ok(Some(index))
            }
            Some(_) => {
                log::debug!("queued ack carried no index");
                Ok(None)
            }
        }
    }

    /// Fire-and-forget send: validated locally, no wait. A runtime refusal
    /// surfaces through the standing error and STATUS (issue #23), not here.
    pub async fn fire(&self, cmd: Command) -> Result<(), ClientError> {
        if self.is_closed() {
            return Err(ClientError::Closed);
        }
        let data = self.encode(&cmd, self.next_req_id())?;
        self.inner.sock.send(&data).await?;
        Ok(())
    }

    /// A watch receiver over the latest STATUS frame (`None` until the
    /// first packet). The basis for `stream_status` and `wait_status`.
    pub fn subscribe_status(&self) -> watch::Receiver<Option<Arc<Status>>> {
        self.inner.status_tx.subscribe()
    }

    /// The latest STATUS frame, if any has arrived.
    pub fn latest_status(&self) -> Option<Arc<Status>> {
        self.inner.status_tx.borrow().clone()
    }

    /// The protocol skew the status stream has latched, as `(daemon,
    /// client)` versions: STATUS from that runtime cannot be read, so
    /// every wait on it answers false at once rather than at its timeout.
    pub fn protocol_mismatch(&self) -> Option<(u8, u8)> {
        self.inner
            .skew
            .lock()
            .unwrap()
            .map(|daemon| (daemon, par6_proto::PROTO_VERSION))
    }

    /// Fault injection for tests: discard COMPLETE pushes as they arrive.
    #[doc(hidden)]
    pub fn drop_complete_pushes_for_test(&self, drop: bool) {
        self.inner
            .drop_complete_pushes
            .store(drop, Ordering::SeqCst);
    }

    /// Block until `pred` holds for a STATUS frame, or `timeout` expires.
    /// False at once under a latched protocol mismatch: no frame from
    /// that runtime will ever be read.
    pub async fn wait_status(
        &self,
        mut pred: impl FnMut(&Status) -> bool,
        timeout: Duration,
    ) -> bool {
        let mut rx = self.subscribe_status();
        let deadline = tokio::time::Instant::now() + timeout;
        loop {
            if self.protocol_mismatch().is_some() {
                return false;
            }
            if let Some(s) = rx.borrow_and_update().clone() {
                if pred(&s) {
                    return true;
                }
            }
            if self.is_closed() {
                return false;
            }
            match tokio::time::timeout_at(deadline, rx.changed()).await {
                Ok(Ok(())) => {}
                _ => return false,
            }
        }
    }

    /// The protocol's stale-error ordering rule: a standing error fails a
    /// wait on `index` only when the frame proves it postdates that
    /// command's acceptance.
    fn blocking_error(status: &Status, index: u64) -> Option<WireError> {
        let err = status.error.as_ref()?;
        if err.command_index > index as i64 {
            return None;
        }
        if status.accepted_index >= index as i64 {
            return Some(err.clone());
        }
        None
    }

    /// Block until queued command `index` completes. Satisfied by the
    /// COMPLETE push, with the status stream as fallback (completed_index
    /// high-water, or a blocking error under the stale-error rule).
    /// `Ok(true)` on success, `Robot` when the command finished in error,
    /// `Ok(false)` on timeout, or when STATUS showed the command finished
    /// and neither its COMPLETE push nor the runtime's COMMAND_COMPLETION
    /// record could be had (logged); `SessionChanged` if the runtime
    /// restarted mid-wait, `ProtocolMismatch` if its STATUS cannot be read.
    pub async fn wait_command(&self, index: u64, timeout: Duration) -> Result<bool, ClientError> {
        let logged = self
            .inner
            .completions
            .lock()
            .unwrap()
            .log
            .get(&index)
            .cloned();
        let done = match logged {
            Some(done) => done,
            None => match self.await_completion(index, timeout).await? {
                Some(done) => done,
                None => return Ok(false),
            },
        };
        match done {
            (true, _, _) => Ok(true),
            (false, Some(detail), _) => Err(ClientError::Robot(detail)),
            (false, None, _) => Err(ClientError::Robot(WireError {
                command_index: index as i64,
                code: 0,
                title: "Command failed".into(),
                cause: String::new(),
                effect: String::new(),
                remedy: String::new(),
            })),
        }
    }

    /// Register a waiter for `index` and race the COMPLETE push against
    /// the status stream. `None` = timed out, or finished per STATUS
    /// without the push. The waiter entry never outlives the wait.
    async fn await_completion(
        &self,
        index: u64,
        timeout: Duration,
    ) -> Result<Option<Completion>, ClientError> {
        let (tx, mut rx) = oneshot::channel();
        {
            let mut comp = self.inner.completions.lock().unwrap();
            if let Some(done) = comp.log.get(&index) {
                return Ok(Some(done.clone()));
            }
            comp.waiters.entry(index).or_default().push(tx);
        }
        let outcome = self.race_completion(index, &mut rx, timeout).await;
        drop(rx);
        let mut comp = self.inner.completions.lock().unwrap();
        if let Some(list) = comp.waiters.get_mut(&index) {
            list.retain(|tx| !tx.is_closed());
            if list.is_empty() {
                comp.waiters.remove(&index);
            }
        }
        outcome
    }

    async fn race_completion(
        &self,
        index: u64,
        rx: &mut oneshot::Receiver<Completion>,
        timeout: Duration,
    ) -> Result<Option<Completion>, ClientError> {
        // The session the wait began in: a runtime that restarts mid-wait
        // starts its indexes over, and the one awaited names nothing then.
        let session = self.latest_status().map(|s| s.session_id);
        let restarted = move |s: &Status| session.is_some_and(|s0| s.session_id != s0);
        let hit = {
            let via_status = self.wait_status(
                move |s| {
                    restarted(s)
                        || s.completed_index >= index as i64
                        || Self::blocking_error(s, index).is_some()
                },
                timeout,
            );
            tokio::pin!(via_status);
            tokio::select! {
                got = &mut *rx => {
                    return match got {
                        Ok(done) => Ok(Some(done)),
                        // The channel closes when a restart clears the
                        // waiters, after the new session's STATUS is out.
                        Err(_) => match self.latest_status() {
                            Some(s) if restarted(&s) => Err(ClientError::SessionChanged { index }),
                            _ => Ok(None),
                        },
                    };
                }
                hit = &mut via_status => hit,
            }
        };
        if !hit {
            if let Some((daemon, client)) = self.protocol_mismatch() {
                return Err(ClientError::ProtocolMismatch { daemon, client });
            }
            return Ok(None);
        }
        if let Some(s) = self.latest_status() {
            if restarted(&s) {
                return Err(ClientError::SessionChanged { index });
            }
            if let Some(err) = Self::blocking_error(&s, index) {
                return Err(ClientError::Robot(err));
            }
        }
        match tokio::time::timeout(COMPLETE_GRACE, rx).await {
            Ok(Ok(done)) => Ok(Some(done)),
            _ => self.recover_completion(index).await,
        }
    }

    /// STATUS says `index` finished but its COMPLETE push never came: the
    /// runtime keeps what the push carried, so ask for it rather than
    /// declare the verdict unknown. `None` only when the runtime has no
    /// record of it either.
    async fn recover_completion(&self, index: u64) -> Result<Option<Completion>, ClientError> {
        match self.query(Command::CommandCompletion { index }).await {
            Ok(par6_proto::QueryResult::CommandCompletion {
                finished: true,
                ok,
                detail,
                verdict,
                ..
            }) => {
                let done: Completion = (ok, detail, verdict);
                record_completion(&self.inner, index, &done);
                Ok(Some(done))
            }
            Ok(_) => {
                log::warn!(
                    "command {index} finished per STATUS but the runtime has no record of \
                     its completion; verdict unknown"
                );
                Ok(None)
            }
            Err(e) => {
                log::warn!(
                    "command {index} finished per STATUS, its COMPLETE push never arrived \
                     and the runtime could not be asked ({e}); verdict unknown"
                );
                Ok(None)
            }
        }
    }

    /// Settle verdict off command `index`'s COMPLETE push: 1 = object
    /// while closing, 2 = object while opening, 3 = target reached with
    /// no object. `None` for non-tool commands, unfinished ones, ones
    /// whose completion could not be recovered, and completions that fell
    /// out of the log (the last [`COMPLETIONS_KEPT`]) — call after [`Self::wait_command`]
    /// returns `Ok(true)`.
    pub fn command_verdict(&self, index: u64) -> Option<u8> {
        self.inner
            .completions
            .lock()
            .unwrap()
            .log
            .get(&index)
            .and_then(|(_, _, v)| *v)
    }
}

async fn reply_rx(inner: Arc<Inner>) {
    let mut buf = vec![0u8; 65536];
    loop {
        let n = match inner.sock.recv(&mut buf).await {
            Ok(n) => n,
            Err(e) => {
                if inner.closed.load(Ordering::SeqCst) {
                    return;
                }
                log::debug!("reply socket recv error: {e}");
                continue;
            }
        };
        let reply = match decode_reply(&buf[..n]) {
            Ok(reply) => reply,
            Err(e) => {
                log::debug!("ignoring undecodable reply datagram: {e}");
                continue;
            }
        };
        match reply {
            Reply::Complete {
                index,
                ok,
                detail,
                verdict,
            } => {
                if inner.drop_complete_pushes.load(Ordering::SeqCst) {
                    continue;
                }
                record_completion(&inner, index, &(ok, detail, verdict));
            }
            other => {
                let req_id = match &other {
                    Reply::Ok { req_id, .. }
                    | Reply::Error { req_id, .. }
                    | Reply::Response { req_id, .. } => *req_id,
                    Reply::Complete { .. } => unreachable!(),
                };
                let tx = inner.pending.lock().unwrap().remove(&req_id);
                match (tx, other) {
                    (Some(tx), reply) => {
                        let _ = tx.send(reply);
                    }
                    (None, Reply::Error { error, .. }) => log_unclaimed(&inner, &error),
                    (None, _) => {}
                }
            }
        }
    }
}

/// An ERROR nobody is waiting on — a rejected fire-and-forget, or a reply
/// that arrived after its request timed out. Throttled per code so a UI
/// streaming refused jogs gets a readable line, not a scroll. The
/// authoritative surface is the runtime's standing error (issue #23);
/// this log is corroboration.
fn log_unclaimed(inner: &Inner, error: &WireError) {
    let now = std::time::Instant::now();
    let mut seen = inner.unclaimed.lock().unwrap();
    if let Some(last) = seen.get(&error.code) {
        if now.duration_since(*last) < UNCLAIMED_ERROR_PERIOD {
            return;
        }
    }
    seen.insert(error.code, now);
    log::warn!(
        "runtime reported an error nothing is waiting on: [{}] {}: {}",
        error.code,
        error.title,
        error.cause
    );
}

/// A finished command, logged and handed to whoever is waiting on it.
fn record_completion(inner: &Inner, index: u64, done: &Completion) {
    let mut comp = inner.completions.lock().unwrap();
    if comp.log.insert(index, done.clone()).is_none() {
        comp.order.push_back(index);
    }
    while comp.order.len() > COMPLETIONS_KEPT {
        if let Some(old) = comp.order.pop_front() {
            comp.log.remove(&old);
        }
    }
    for tx in comp.waiters.remove(&index).unwrap_or_default() {
        let _ = tx.send(done.clone());
    }
}

/// The protocol-skew warning fires once per process, not per frame.
static VERSION_SKEW_WARNED: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

/// Report a daemon whose protocol version is not this client's, once, and
/// latch it for the waits when its STATUS could not be read (`readable`
/// false): a frame that cannot be read now will not be readable later
/// either, and a wait that ran out its timeout instead would report the
/// runtime silent rather than foreign. A STATUS that did decode clears the
/// latch — the runtime this client talks to is readable after all.
///
/// `None` is a datagram we could not even identify as a par6 STATUS, which
/// says nothing about versions and is left to the caller's debug line.
fn note_skew(inner: &Inner, daemon: Option<u8>, readable: bool) {
    if readable {
        let mut latched = inner.skew.lock().unwrap();
        if latched.is_some() {
            *latched = None;
        }
    }
    let Some(daemon) = daemon.filter(|v| *v != par6_proto::PROTO_VERSION) else {
        return;
    };
    if !readable {
        *inner.skew.lock().unwrap() = Some(daemon);
    }
    if VERSION_SKEW_WARNED.swap(true, Ordering::Relaxed) {
        return;
    }
    log::warn!(
        "daemon speaks protocol v{daemon} but this client was built for v{}: \
         messages whose layout differs will fail to decode, and STATUS may \
         stop arriving entirely",
        par6_proto::PROTO_VERSION
    );
}

async fn status_rx(inner: Arc<Inner>, sock: UdpSocket) {
    let mut buf = vec![0u8; 65536];
    loop {
        let n = match sock.recv(&mut buf).await {
            Ok(n) => n,
            Err(e) => {
                if inner.closed.load(Ordering::SeqCst) {
                    return;
                }
                log::debug!("status socket recv error: {e}");
                continue;
            }
        };
        let status = match decode_status(&buf[..n]) {
            Ok(status) => status,
            Err(e) => {
                // Skew is checked HERE too, not only on the success path.
                // A version that adds fields makes the array longer, so an
                // older daemon's STATUS fails on arity before `decode_status`
                // ever reads the version — which is to say the check below
                // could never fire in the one case it exists for, and the
                // client went silent with a debug line as its only account.
                note_skew(
                    &inner,
                    par6_proto::peek_status_proto_version(&buf[..n]),
                    false,
                );
                log::debug!("ignoring undecodable status datagram: {e}");
                continue;
            }
        };
        note_skew(&inner, Some(status.proto_version), true);
        let restarted = {
            let mut last = inner.last_seq.lock().unwrap();
            let mut restarted = false;
            if let Some((session, prev)) = *last {
                if session == status.session_id && status.seq <= prev {
                    continue;
                }
                restarted = session != status.session_id;
                if !restarted && status.seq > prev.saturating_add(1) {
                    inner
                        .seq_gaps
                        .fetch_add(status.seq - prev - 1, Ordering::Relaxed);
                }
            }
            *last = Some((status.session_id, status.seq));
            restarted
        };
        inner.status_tx.send_replace(Some(Arc::new(status)));
        if restarted {
            // A restarted runtime numbers its queue from the start again:
            // what was logged names other commands now. The new session's
            // STATUS is published first, so a waiter whose channel closes
            // here finds the restart in it rather than a lost answer.
            let mut comp = inner.completions.lock().unwrap();
            comp.log.clear();
            comp.order.clear();
            comp.waiters.clear();
        }
    }
}
