//! In-process daemon harness for the `par6d` integration tests.
//!
//! [`Rig`] boots a real [`Daemon`] in the test process on ephemeral
//! loopback ports and hands back the STATUS broadcast; [`Client`] speaks
//! protocol v2 to it over a real socket. Nothing here fakes anything — the
//! daemon under test is the one that ships, and every assertion a test
//! makes is on a datagram it actually sent.

#![allow(dead_code)]

use std::net::{SocketAddr, UdpSocket};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use par6_proto::{
    decode_reply, decode_status, encode_command, Command, QueryResult, Reply, Status, WireError,
};
use par6d::options::StatusTransport;
use par6d::{Daemon, Options};

/// Ceiling on any single wait. Generous: these run a real 20-50 Hz
/// control loop in wall-clock time on a shared CI box.
pub const BUDGET: Duration = Duration::from_secs(30);
/// Socket read timeout — short enough that a wait loop notices its
/// deadline, long enough not to spin.
pub const READ_TIMEOUT: Duration = Duration::from_millis(100);

pub fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("repo root")
}

/// The `assets/par6_description` tree the kinematics stack loads.
///
/// Passed explicitly because a test config lives in a temp dir, where the
/// daemon's default "beside the config" search finds nothing.
pub fn assets_dir() -> PathBuf {
    repo_root().join("assets/par6_description")
}

/// The shipped `config/PAR6.toml`, unpatched.
pub fn shipped_config() -> PathBuf {
    repo_root().join("config/PAR6.toml")
}

pub fn is_timeout(e: &std::io::Error) -> bool {
    matches!(
        e.kind(),
        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
    )
}

/// A running daemon plus the sockets its broadcasts land on.
pub struct Rig {
    daemon: Option<Daemon>,
    status_rx: UdpSocket,
    _telemetry_rx: UdpSocket,
}

impl Rig {
    /// Boot the simulator on `config`, with the kinematic plant.
    pub fn boot(config: PathBuf) -> Rig {
        Rig::boot_with(config, false)
    }

    /// Boot the simulator on `config`; `sim_dynamics` selects the
    /// torque-level plant over the kinematic one.
    pub fn boot_with(config: PathBuf, sim_dynamics: bool) -> Rig {
        Rig::boot_opts(config, sim_dynamics, None)
    }

    /// Boot the simulator with the STATUS broadcast rate overridden, as
    /// `--status-rate` / `PAR6_STATUS_RATE_HZ` do.
    pub fn boot_at_status_rate(config: PathBuf, hz: u32) -> Rig {
        Rig::try_boot_at_status_rate(config, hz).expect("daemon boots in sim mode")
    }

    /// The same, surfacing the startup error instead of panicking.
    pub fn try_boot_at_status_rate(config: PathBuf, hz: u32) -> Result<Rig, String> {
        Rig::try_boot_opts(config, false, Some(hz))
    }

    fn boot_opts(config: PathBuf, sim_dynamics: bool, status_rate_hz: Option<u32>) -> Rig {
        Rig::try_boot_opts(config, sim_dynamics, status_rate_hz).expect("daemon boots in sim mode")
    }

    fn try_boot_opts(
        config: PathBuf,
        sim_dynamics: bool,
        status_rate_hz: Option<u32>,
    ) -> Result<Rig, String> {
        let _ = env_logger::builder().is_test(true).try_init();
        let status_rx = UdpSocket::bind("127.0.0.1:0").expect("status socket");
        status_rx
            .set_read_timeout(Some(READ_TIMEOUT))
            .expect("timeout");
        let telemetry_rx = UdpSocket::bind("127.0.0.1:0").expect("telemetry socket");
        let opts = Options {
            sim: true,
            sim_dynamics,
            config: Some(config),
            assets: Some(assets_dir()),
            command_port: Some(0),
            bind: Some("127.0.0.1".parse().unwrap()),
            status_host: Some("127.0.0.1".parse().unwrap()),
            status_port: Some(status_rx.local_addr().unwrap().port()),
            telemetry_port: Some(telemetry_rx.local_addr().unwrap().port()),
            status_transport: Some(StatusTransport::Unicast),
            status_rate_hz,
            ..Options::default()
        };
        let daemon = Daemon::start(&opts).map_err(|e| e.to_string())?;
        Ok(Rig {
            daemon: Some(daemon),
            status_rx,
            _telemetry_rx: telemetry_rx,
        })
    }

    /// The command plane's bound address.
    pub fn addr(&self) -> SocketAddr {
        self.daemon.as_ref().expect("running").command_addr()
    }

    /// The telemetry socket, for tests that decode the stream.
    pub fn telemetry(&self) -> &UdpSocket {
        &self._telemetry_rx
    }

    /// Widen the STATUS socket's read timeout past the default
    /// [`READ_TIMEOUT`], for tests that run the broadcast slower than
    /// one frame per 100 ms.
    pub fn set_status_timeout(&self, t: Duration) {
        self.status_rx
            .set_read_timeout(Some(t))
            .expect("status timeout");
    }

    /// One STATUS datagram, or `None` on a quiet [`READ_TIMEOUT`] window.
    pub fn recv_status(&self) -> Option<Status> {
        let mut buf = [0u8; 65535];
        match self.status_rx.recv_from(&mut buf) {
            Ok((n, _)) => Some(decode_status(&buf[..n]).expect("decodable status")),
            Err(e) if is_timeout(&e) => None,
            Err(e) => panic!("status recv failed: {e}"),
        }
    }

    /// Poll the broadcast until `pred` holds; panics after [`BUDGET`].
    pub fn wait_status(&self, what: &str, pred: impl Fn(&Status) -> bool) -> Status {
        let deadline = Instant::now() + BUDGET;
        let mut last: Option<Status> = None;
        loop {
            if let Some(s) = self.recv_status() {
                if pred(&s) {
                    return s;
                }
                last = Some(s);
            }
            assert!(
                Instant::now() < deadline,
                "status condition `{what}` not met within budget; last: {last:?}"
            );
        }
    }

    /// Drop the STATUS frames already buffered on the socket.
    ///
    /// A wait that starts on a stale frame can be satisfied by state from
    /// before the command under test, so anything asserting a TRANSITION
    /// drains first.
    ///
    /// Non-blocking, not [`recv_status`](Self::recv_status)-until-empty:
    /// the broadcast keeps arriving at 50 Hz whatever the test is doing,
    /// so a timeout-bounded read always has another live frame waiting
    /// and the drain would run until the broadcaster happened to stall
    /// for longer than [`READ_TIMEOUT`]. Draining to `WouldBlock` empties
    /// exactly the backlog and returns.
    pub fn drain_status(&self) {
        self.status_rx
            .set_nonblocking(true)
            .expect("nonblocking status socket");
        let mut buf = [0u8; 65535];
        while self.status_rx.recv_from(&mut buf).is_ok() {}
        self.status_rx
            .set_nonblocking(false)
            .expect("blocking status socket");
    }

    /// Every STATUS arriving in `window` — for tests that measure a whole
    /// motion rather than waiting on one condition.
    pub fn collect_status(&self, window: Duration) -> Vec<Status> {
        let until = Instant::now() + window;
        let mut out = Vec::new();
        while Instant::now() < until {
            if let Some(s) = self.recv_status() {
                out.push(s);
            }
        }
        out
    }

    pub fn shutdown(mut self) {
        self.daemon.take().expect("running").shutdown();
    }
}

/// A protocol-v2 client on a real socket.
pub struct Client {
    sock: UdpSocket,
    server: SocketAddr,
    next_req: u32,
    completes: Vec<(u64, bool, Option<WireError>)>,
}

impl Client {
    pub fn new(server: SocketAddr) -> Client {
        let sock = UdpSocket::bind("127.0.0.1:0").expect("client socket");
        sock.set_read_timeout(Some(READ_TIMEOUT)).expect("timeout");
        Client {
            sock,
            server,
            next_req: 1,
            completes: Vec::new(),
        }
    }

    pub fn send(&mut self, cmd: &Command) -> u32 {
        let req_id = self.next_req;
        self.next_req += 1;
        let mut buf = Vec::new();
        encode_command(cmd, req_id, &mut buf).expect("encodable command");
        self.sock.send_to(&buf, self.server).expect("send");
        req_id
    }

    pub fn try_recv(&mut self) -> Option<Reply> {
        let mut buf = [0u8; 65535];
        match self.sock.recv_from(&mut buf) {
            Ok((n, _)) => Some(decode_reply(&buf[..n]).expect("decodable reply")),
            Err(e) if is_timeout(&e) => None,
            Err(e) => panic!("client recv failed: {e}"),
        }
    }

    /// Send and wait for the direct reply with the matching `req_id`,
    /// stashing COMPLETE pushes and dropping stale replies.
    pub fn request(&mut self, cmd: &Command) -> Reply {
        let req_id = self.send(cmd);
        let deadline = Instant::now() + BUDGET;
        loop {
            if let Some(r) = self.try_recv() {
                match &r {
                    Reply::Ok { req_id: id, .. }
                    | Reply::Error { req_id: id, .. }
                    | Reply::Response { req_id: id, .. }
                        if *id == req_id =>
                    {
                        return r;
                    }
                    Reply::Complete { index, ok, detail } => {
                        self.completes.push((*index, *ok, detail.clone()));
                    }
                    _ => {}
                }
            }
            assert!(
                Instant::now() < deadline,
                "no reply to {cmd:?} within budget"
            );
        }
    }

    pub fn query(&mut self, cmd: &Command) -> QueryResult {
        match self.request(cmd) {
            Reply::Response { result, .. } => result,
            other => panic!("expected RESPONSE, got {other:?}"),
        }
    }

    /// Expect a plain OK — no queue index. A command that allocates one is
    /// a different class, so accepting either here would hide a
    /// misclassified tag.
    pub fn ok(&mut self, cmd: &Command) {
        match self.request(cmd) {
            Reply::Ok { index: None, .. } => {}
            other => panic!("expected plain OK, got {other:?}"),
        }
    }

    /// Expect an OK carrying the queue index the command was allocated.
    pub fn ok_index(&mut self, cmd: &Command) -> u64 {
        match self.request(cmd) {
            Reply::Ok { index: Some(i), .. } => i,
            other => panic!("expected OK with index, got {other:?}"),
        }
    }

    pub fn expect_error(&mut self, cmd: &Command) -> WireError {
        match self.request(cmd) {
            Reply::Error { error, .. } => error,
            other => panic!("expected ERROR, got {other:?}"),
        }
    }

    /// The COMPLETE push for `index`, from the stash or off the wire.
    pub fn wait_complete(&mut self, index: u64) -> (bool, Option<WireError>) {
        if let Some(pos) = self.completes.iter().position(|c| c.0 == index) {
            let (_, ok, detail) = self.completes.remove(pos);
            return (ok, detail);
        }
        let deadline = Instant::now() + BUDGET;
        loop {
            if let Some(Reply::Complete {
                index: i,
                ok,
                detail,
            }) = self.try_recv()
            {
                if i == index {
                    return (ok, detail);
                }
                self.completes.push((i, ok, detail));
            }
            assert!(
                Instant::now() < deadline,
                "no COMPLETE for index {index} within budget"
            );
        }
    }

    /// Whether a COMPLETE for `index` has been seen — without waiting, so
    /// a test can assert one never arrives.
    pub fn saw_complete(&self, index: u64) -> bool {
        self.completes.iter().any(|c| c.0 == index)
    }

    /// Drain whatever replies are already buffered into the stash.
    pub fn drain(&mut self) {
        while let Some(r) = self.try_recv() {
            if let Reply::Complete { index, ok, detail } = r {
                self.completes.push((index, ok, detail));
            }
        }
    }
}
