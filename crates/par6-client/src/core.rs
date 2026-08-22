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
    Status, WireError,
};
use tokio::net::UdpSocket;
use tokio::sync::{oneshot, watch};
use tokio::task::JoinHandle;

use crate::error::ClientError;
use crate::sockets;

/// How many completion results are kept for late `wait_command` callers.
const COMPLETIONS_KEPT: usize = 1024;

/// How often one error code may be logged for a reply nobody awaits.
const UNCLAIMED_ERROR_PERIOD: Duration = Duration::from_secs(1);

/// How the STATUS broadcast is subscribed.
#[derive(Debug, Clone)]
pub enum StatusTransport {
    /// Multicast join with unicast fallback (the default ladder).
    Multicast {
        /// Multicast group address.
        group: Ipv4Addr,
        /// Interface address to join on first.
        iface: Ipv4Addr,
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
        let status = if kind == "UNICAST" {
            StatusTransport::Unicast {
                host: env_parse("PAR6_STATUS_UNICAST_HOST", Ipv4Addr::LOCALHOST),
            }
        } else {
            StatusTransport::Multicast {
                group: env_parse("PAR6_STATUS_MCAST_GROUP", Ipv4Addr::new(239, 255, 0, 71)),
                iface: env_parse("PAR6_STATUS_MCAST_IF", Ipv4Addr::LOCALHOST),
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

/// A finished queued command: success flag plus failure detail.
pub type Completion = (bool, Option<WireError>);

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
    last_seq: Mutex<Option<u64>>,
    seq_gaps: AtomicU64,
    unclaimed: Mutex<HashMap<u16, std::time::Instant>>,
    pub(crate) last_command_index: AtomicI64,
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
    pub async fn connect(cfg: ClientConfig) -> Result<Self, ClientError> {
        let sock = UdpSocket::bind("0.0.0.0:0").await?;
        sock.connect((cfg.host.as_str(), cfg.port)).await?;

        let status_std = match cfg.status {
            StatusTransport::Multicast { group, iface } => {
                sockets::multicast_socket(group, cfg.status_port, iface).or_else(|e| {
                    log::warn!(
                        "multicast status subscription failed ({e}); falling back to unicast"
                    );
                    sockets::unicast_socket(Ipv4Addr::LOCALHOST, cfg.status_port)
                })?
            }
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
        for task in self.tasks.lock().unwrap().drain(..) {
            task.abort();
        }
        self.inner.pending.lock().unwrap().clear();
        self.inner.completions.lock().unwrap().waiters.clear();
        // Wake status waiters so they observe the closed flag.
        self.inner.status_tx.send_modify(|_| {});
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
        split_into_chunks(req_id, transfer_id, &data, self.inner.cfg.mtu - 32)
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

    /// Block until `pred` holds for a STATUS frame, or `timeout` expires.
    pub async fn wait_status(
        &self,
        mut pred: impl FnMut(&Status) -> bool,
        timeout: Duration,
    ) -> bool {
        let mut rx = self.subscribe_status();
        let deadline = tokio::time::Instant::now() + timeout;
        loop {
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
    /// `Ok(true)` on success, `Ok(false)` on timeout, `Robot` when the
    /// command finished in error.
    pub async fn wait_command(&self, index: u64, timeout: Duration) -> Result<bool, ClientError> {
        let done = self
            .inner
            .completions
            .lock()
            .unwrap()
            .log
            .get(&index)
            .cloned();
        let done = match done {
            Some(done) => Some(done),
            None => {
                let (tx, rx) = oneshot::channel();
                self.inner
                    .completions
                    .lock()
                    .unwrap()
                    .waiters
                    .entry(index)
                    .or_default()
                    .push(tx);
                let via_status = self.wait_status(
                    move |s| {
                        s.completed_index >= index as i64
                            || Self::blocking_error(s, index).is_some()
                    },
                    timeout,
                );
                tokio::select! {
                    got = rx => got.ok(),
                    hit = via_status => {
                        if hit {
                            if let Some(s) = self.latest_status() {
                                if let Some(err) = Self::blocking_error(&s, index) {
                                    return Err(ClientError::Robot(err));
                                }
                            }
                            Some((true, None))
                        } else {
                            return Ok(false);
                        }
                    }
                }
            }
        };
        match done {
            Some((true, _)) => Ok(true),
            Some((false, Some(detail))) => Err(ClientError::Robot(detail)),
            Some((false, None)) => Err(ClientError::Robot(WireError {
                command_index: index as i64,
                code: 0,
                title: "Command failed".into(),
                cause: String::new(),
                effect: String::new(),
                remedy: String::new(),
            })),
            None => Ok(false),
        }
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
            Reply::Complete { index, ok, detail } => {
                let mut comp = inner.completions.lock().unwrap();
                comp.log.insert(index, (ok, detail.clone()));
                comp.order.push_back(index);
                while comp.order.len() > COMPLETIONS_KEPT {
                    if let Some(old) = comp.order.pop_front() {
                        comp.log.remove(&old);
                    }
                }
                for tx in comp.waiters.remove(&index).unwrap_or_default() {
                    let _ = tx.send((ok, detail.clone()));
                }
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
                log::debug!("ignoring undecodable status datagram: {e}");
                continue;
            }
        };
        {
            let mut last = inner.last_seq.lock().unwrap();
            if let Some(prev) = *last {
                if status.seq > prev + 1 {
                    inner
                        .seq_gaps
                        .fetch_add(status.seq - prev - 1, Ordering::Relaxed);
                }
            }
            *last = Some(status.seq);
        }
        inner.status_tx.send_replace(Some(Arc::new(status)));
    }
}
