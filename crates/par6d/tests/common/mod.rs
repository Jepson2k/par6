//! In-process daemon harness for the `par6d` integration tests.
//!
//! [`Rig`] boots a real [`Daemon`] in the test process on ephemeral
//! loopback ports and hands back the STATUS broadcast; [`Client`] speaks
//! protocol v2 to it over a real socket. Nothing here fakes anything — the
//! daemon under test is the one that ships, and every assertion a test
//! makes is on a datagram it actually sent.

#![allow(dead_code)]

use std::collections::BTreeSet;
use std::net::{SocketAddr, UdpSocket};
use std::path::PathBuf;
use std::sync::Mutex;
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

/// The shipped config with `[bus].interface` renamed to `iface`, in a
/// scratch directory with the gripper TOMLs beside it (they resolve
/// relative to the robot file). Hardware-mode failure tests use a name
/// no machine has, so they fail the same way on a control box with a
/// live `can0` as in a container with no CAN support at all.
pub fn config_with_interface(iface: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("par6-test-cfg-{}-{}", std::process::id(), iface));
    let grippers = dir.join("grippers");
    std::fs::create_dir_all(&grippers).expect("scratch config dir");
    for entry in std::fs::read_dir(repo_root().join("config/grippers")).expect("grippers dir") {
        let entry = entry.expect("gripper entry");
        std::fs::copy(entry.path(), grippers.join(entry.file_name())).expect("copy gripper");
    }
    let toml = std::fs::read_to_string(shipped_config()).expect("shipped config");
    let needle = "interface = \"can0\"";
    assert!(toml.contains(needle), "shipped config no longer names can0");
    let path = dir.join("PAR6.toml");
    std::fs::write(
        &path,
        toml.replace(needle, &format!("interface = \"{iface}\"")),
    )
    .expect("write config");
    path
}

/// Rewrite the value of a top-level `key = value` line, anchored on the
/// KEY.
///
/// Anchoring on the shipped VALUE — `text.replace("tick_dt_s = 0.004",
/// ...)` — makes every re-ticking harness in the repo depend on a
/// literal it does not own: change the shipped tick and they all stop
/// re-ticking. The assert is what turns that from a silent
/// run-at-the-wrong-rate into a failure.
fn set_scalar(text: &str, key: &str, value: &str) -> String {
    let mut hit = false;
    let mut out = String::with_capacity(text.len());
    for line in text.lines() {
        let is_key = line
            .split_once('=')
            .is_some_and(|(lhs, _)| lhs.trim() == key);
        if is_key && !hit {
            hit = true;
            out.push_str(&format!("{key} = {value}"));
        } else {
            out.push_str(line);
        }
        out.push('\n');
    }
    assert!(hit, "shipped config no longer declares `{key}`");
    out
}

/// Write `bytes` to `dst` so a concurrent reader never sees a torn file.
///
/// `fs::write` and `fs::copy` truncate and then fill, and a test that
/// boots a daemon while another is in that window reads an empty config
/// and fails with "missing field `robot`". Writing beside the target and
/// renaming leaves a reader with either the whole old file or the whole
/// new one — and the content is a pure function of `(tag, dt)`, so which
/// one it gets does not matter.
fn write_atomic(dst: &std::path::Path, bytes: &[u8]) {
    static SEQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let n = SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let tmp = dst.with_extension(format!("tmp.{n}"));
    std::fs::write(&tmp, bytes).expect("write test config");
    std::fs::rename(&tmp, dst).expect("publish test config");
}

/// The shipped config re-timed to `dt` seconds per tick, written with its
/// gripper files into a scratch directory named for `tag`.
///
/// Tests sharing a tag share the tree, which is why every file lands
/// through [`write_atomic`]: the content is identical for a given
/// `(tag, dt)`, so concurrent writers are harmless as long as no reader
/// can catch one mid-write.
pub fn retimed_config(tag: &str, dt: f64) -> PathBuf {
    let src = shipped_config();
    let dir = std::env::temp_dir().join(format!("par6d-{tag}-{}", std::process::id()));
    let grippers = dir.join("grippers");
    std::fs::create_dir_all(&grippers).expect("test config dir");
    let text = std::fs::read_to_string(&src).expect("read PAR6.toml");
    let patched = set_scalar(&text, "tick_dt_s", &dt.to_string());
    let dst = dir.join("PAR6.toml");
    write_atomic(&dst, patched.as_bytes());
    for entry in std::fs::read_dir(src.parent().unwrap().join("grippers")).expect("grippers dir") {
        let e = entry.expect("dir entry");
        let body = std::fs::read(e.path()).expect("read gripper toml");
        write_atomic(&grippers.join(e.file_name()), &body);
    }
    dst
}

/// The shipped config re-timed to `dt` AND pointed at `iface`, for the
/// hardware-mode startup checks that must refuse before any interface is
/// opened.
pub fn retimed_config_with_interface(tag: &str, dt: f64, iface: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("par6d-{tag}-{}", std::process::id()));
    let grippers = dir.join("grippers");
    std::fs::create_dir_all(&grippers).expect("test config dir");
    let src = shipped_config();
    let text = std::fs::read_to_string(&src).expect("read PAR6.toml");
    let patched = set_scalar(&text, "tick_dt_s", &dt.to_string());
    let patched = set_scalar(&patched, "interface", &format!("\"{iface}\""));
    let dst = dir.join("PAR6.toml");
    write_atomic(&dst, patched.as_bytes());
    for entry in std::fs::read_dir(src.parent().unwrap().join("grippers")).expect("grippers dir") {
        let e = entry.expect("dir entry");
        let body = std::fs::read(e.path()).expect("read gripper toml");
        write_atomic(&grippers.join(e.file_name()), &body);
    }
    dst
}

/// The shipped park pose in degrees, the way the wire carries angles.
pub fn park_deg() -> [f64; par6_proto::NUM_JOINTS] {
    let cfg = par6_config::RobotConfig::load(&shipped_config()).expect("PAR6 config");
    let mut a = [0.0; par6_proto::NUM_JOINTS];
    for (out, rad) in a.iter_mut().zip(cfg.robot.park_pose_rad.iter()) {
        *out = rad.to_degrees();
    }
    a
}

pub fn is_timeout(e: &std::io::Error) -> bool {
    matches!(
        e.kind(),
        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
    )
}

/// Point the bus-grant segments at a scratch directory, once per test
/// binary.
///
/// `/dev/shm/loop_tick` names the claim on the ONE `can0` a box has. A
/// test rig that wrote it would tell every CAN tool on the machine that
/// a runtime it cannot see owns the bus — and its teardown would then
/// take a real runtime's claim away.
pub fn redirect_bus_grant() {
    static ONCE: std::sync::OnceLock<PathBuf> = std::sync::OnceLock::new();
    let dir = ONCE.get_or_init(|| {
        let dir = std::env::temp_dir().join(format!("par6-test-shm-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("scratch shm dir");
        // SAFETY: set before any daemon in this binary reads it, and
        // always to the same value.
        std::env::set_var("PAR6_SHM_DIR", &dir);
        dir
    });
    debug_assert!(dir.is_dir());
}

/// Daemon options for a simulator boot on loopback: an ephemeral command
/// port, STATUS unicast to `127.0.0.1:status_port`, the repo assets tree.
pub fn sim_options(config: PathBuf, status_port: u16) -> Options {
    Options {
        sim: true,
        config: Some(config),
        assets: Some(assets_dir()),
        command_port: Some(0),
        bind: Some("127.0.0.1".parse().unwrap()),
        status_host: Some("127.0.0.1".parse().unwrap()),
        status_port: Some(status_port),
        status_transport: Some(StatusTransport::Unicast),
        ..Options::default()
    }
}

/// A daemon whose STATUS broadcast is aimed at `status_port` on
/// loopback, for a test that listens with a real `par6_client::Client`
/// instead of the rig's own socket.
pub fn boot_for_client(
    config: PathBuf,
    sim_dynamics: bool,
    status_port: u16,
) -> Result<Daemon, String> {
    let _ = env_logger::builder().is_test(true).try_init();
    redirect_bus_grant();
    let opts = Options {
        sim_dynamics,
        ..sim_options(config, status_port)
    };
    Daemon::start(&opts).map_err(|e| e.to_string())
}

/// A loopback UDP port nothing is bound to right now, and that this
/// process has not already handed out.
///
/// The probe socket has to be released before the caller can bind the
/// port, which leaves a window. Within one test binary — where cargo
/// runs tests in parallel threads and the window is widest — the handed
/// out ports are remembered, so the OS re-offering one it just freed
/// cannot give two tests the same port. Across processes the window
/// remains, and is why a daemon that fails to bind says so rather than
/// carrying on.
pub fn free_udp_port() -> u16 {
    static HANDED_OUT: Mutex<BTreeSet<u16>> = Mutex::new(BTreeSet::new());
    for _ in 0..64 {
        let port = UdpSocket::bind("127.0.0.1:0")
            .expect("probe socket")
            .local_addr()
            .unwrap()
            .port();
        if HANDED_OUT.lock().unwrap().insert(port) {
            return port;
        }
    }
    panic!("could not find a loopback port this process has not already used");
}

/// A running daemon plus the sockets its broadcasts land on.
pub struct Rig {
    daemon: Option<Daemon>,
    status_rx: UdpSocket,
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
        redirect_bus_grant();
        let status_rx = UdpSocket::bind("127.0.0.1:0").expect("status socket");
        status_rx
            .set_read_timeout(Some(READ_TIMEOUT))
            .expect("timeout");
        let opts = Options {
            sim_dynamics,
            status_rate_hz,
            ..sim_options(config, status_rx.local_addr().unwrap().port())
        };
        let daemon = Daemon::start(&opts).map_err(|e| e.to_string())?;
        Ok(Rig {
            daemon: Some(daemon),
            status_rx,
        })
    }

    /// The command plane's bound address.
    pub fn addr(&self) -> SocketAddr {
        self.daemon.as_ref().expect("running").command_addr()
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
    completes: Vec<(u64, bool, Option<WireError>, Option<u8>)>,
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
                    Reply::Complete {
                        index,
                        ok,
                        detail,
                        verdict,
                    } => {
                        self.completes.push((*index, *ok, detail.clone(), *verdict));
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

    /// Send several queueing commands back to back, THEN collect their
    /// indices. `ok_index` per command waits for each reply before the
    /// next datagram leaves, and that round trip is time the server
    /// spends holding a blended move at the head of the queue — on a
    /// loaded host it outlives the blend hold and the pair never blends.
    /// A real program queues moves back to back; so does this.
    pub fn ok_indices(&mut self, cmds: &[Command]) -> Vec<u64> {
        let ids: Vec<u32> = cmds.iter().map(|c| self.send(c)).collect();
        let deadline = Instant::now() + BUDGET;
        let mut out: Vec<Option<u64>> = vec![None; ids.len()];
        while out.iter().any(Option::is_none) {
            if let Some(r) = self.try_recv() {
                match &r {
                    Reply::Ok {
                        req_id,
                        index: Some(i),
                    } => {
                        if let Some(slot) = ids.iter().position(|id| id == req_id) {
                            out[slot] = Some(*i);
                        }
                    }
                    Reply::Error { req_id, error } => {
                        if ids.contains(req_id) {
                            panic!("expected OK with index, got {error:?}");
                        }
                    }
                    Reply::Complete {
                        index,
                        ok,
                        detail,
                        verdict,
                    } => {
                        self.completes.push((*index, *ok, detail.clone(), *verdict));
                    }
                    _ => {}
                }
            }
            assert!(
                Instant::now() < deadline,
                "not every queued command was acknowledged within budget"
            );
        }
        out.into_iter().map(Option::unwrap).collect()
    }

    pub fn expect_error(&mut self, cmd: &Command) -> WireError {
        match self.request(cmd) {
            Reply::Error { error, .. } => error,
            other => panic!("expected ERROR, got {other:?}"),
        }
    }

    /// The COMPLETE push for `index`, from the stash or off the wire.
    pub fn wait_complete(&mut self, index: u64) -> (bool, Option<WireError>) {
        let (ok, detail, _) = self.wait_complete_full(index);
        (ok, detail)
    }

    /// As [`Self::wait_complete`], but also the success verdict element.
    pub fn wait_complete_full(&mut self, index: u64) -> (bool, Option<WireError>, Option<u8>) {
        if let Some(pos) = self.completes.iter().position(|c| c.0 == index) {
            let (_, ok, detail, verdict) = self.completes.remove(pos);
            return (ok, detail, verdict);
        }
        let deadline = Instant::now() + BUDGET;
        loop {
            if let Some(Reply::Complete {
                index: i,
                ok,
                detail,
                verdict,
            }) = self.try_recv()
            {
                if i == index {
                    return (ok, detail, verdict);
                }
                self.completes.push((i, ok, detail, verdict));
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
            if let Reply::Complete {
                index,
                ok,
                detail,
                verdict,
            } = r
            {
                self.completes.push((index, ok, detail, verdict));
            }
        }
    }
}

pub fn to_rad(deg: &[f64; par6_proto::NUM_JOINTS]) -> [f64; par6_proto::NUM_JOINTS] {
    std::array::from_fn(|j| deg[j].to_radians())
}

pub fn to_deg(rad: &[f64; par6_proto::NUM_JOINTS]) -> [f64; par6_proto::NUM_JOINTS] {
    std::array::from_fn(|j| rad[j].to_degrees())
}

pub fn max_deg_error(a: &[f64; par6_proto::NUM_JOINTS], b: &[f64; par6_proto::NUM_JOINTS]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

pub fn teleport_cmd(angles: [f64; par6_proto::NUM_JOINTS]) -> par6_proto::Command {
    par6_proto::Command::Teleport(par6_proto::command::Teleport {
        angles,
        tool_positions: None,
    })
}

/// Teleport until the arm reads referenced at `angles` — the boot enable
/// can still be settling when the first one lands.
pub fn teleport_home(rig: &Rig, c: &mut Client, angles: [f64; par6_proto::NUM_JOINTS]) {
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        c.send(&teleport_cmd(angles));
        let window = Instant::now() + Duration::from_millis(400);
        while Instant::now() < window {
            if let Some(s) = rig.recv_status() {
                if s.homed && max_deg_error(&s.angles, &angles) < 1.0 {
                    return;
                }
            }
        }
        assert!(
            Instant::now() < deadline,
            "teleport did not take effect within budget"
        );
    }
}
