//! End-to-end coverage of [`SocketCanBus`] over a real SocketCAN
//! interface — the transport half the socket-free unit tests cannot
//! reach: kernel frames, kernel RX timestamps, non-blocking drain,
//! `EWOULDBLOCK` handling, the paced boot load as it actually lands on
//! the wire.
//!
//! **Requires a `vcan` interface** (default `vcan0`, override with
//! `PAR6_VCAN_IFACE`):
//!
//! ```bash
//! sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up
//! ```
//!
//! Every test SKIPS cleanly when the interface is absent, so a developer
//! checkout without vcan stays green. Set `PAR6_REQUIRE_VCAN=1` to turn
//! absence into a hard failure instead — the `socketcan (vcan)` CI job
//! sets it after creating vcan0, so the job can never silently degrade
//! to a no-op. Run with `--test-threads=1` when the interface exists:
//! every test observes the same wire and asserts exact frame sequences.
//!
//! There is no simulated driver here on purpose: a fake that answered
//! frames would be re-implementing the protocol. RX comes from the
//! cross-language golden vectors in `tests/golden/can/manifest.json`,
//! written onto the bus by a second plain socket — real wire bytes from
//! the frozen contract, no invented device behavior.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use par6_bus::spectral::{pack_can_id, pack_f32, CommandId};
use par6_bus::{
    BusState, DriverBus, Freshness, GripperCommand, JointCommand, NodeId, PollKind, SocketCanBus,
};
use par6_config::{ConfigBundle, GripperConfig, KtSource, RobotConfig};
use serde_json::Value;
use socketcan::{CanSocket, EmbeddedFrame, Frame, Socket};

// Per-thread so concurrently running tests in this binary cannot pollute
// the measured window.
thread_local! {
    static ALLOCS: Cell<u64> = const { Cell::new(0) };
}

struct CountingAlloc;

// SAFETY: every operation is delegated to the system allocator unchanged;
// the counter is a side effect on a `Cell<u64>`, which has no destructor
// and so never re-enters the allocator.
unsafe impl GlobalAlloc for CountingAlloc {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        bump();
        unsafe { System.alloc(layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        bump();
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        bump();
        unsafe { System.realloc(ptr, layout, new_size) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

fn bump() {
    let _ = ALLOCS.try_with(|c| c.set(c.get() + 1));
}

fn allocs() -> u64 {
    ALLOCS.with(Cell::get)
}

#[global_allocator]
static ALLOC: CountingAlloc = CountingAlloc;

/// The vcan interface to test on, or `None` when there is none.
fn vcan() -> Option<String> {
    let name = std::env::var("PAR6_VCAN_IFACE").unwrap_or_else(|_| "vcan0".to_string());
    PathBuf::from(format!("/sys/class/net/{name}"))
        .exists()
        .then_some(name)
}

macro_rules! require_vcan {
    () => {
        match vcan() {
            Some(name) => name,
            None => {
                // PAR6_REQUIRE_VCAN=1 is the CI job's setting: there the
                // interface is supposed to exist, so its absence means the
                // job is broken and must fail loudly, never no-op green.
                assert!(
                    std::env::var("PAR6_REQUIRE_VCAN").map_or(true, |v| v != "1"),
                    "PAR6_REQUIRE_VCAN=1 but no vcan interface is up \
                     (`sudo modprobe vcan && sudo ip link add dev vcan0 type vcan \
                     && sudo ip link set vcan0 up`)"
                );
                eprintln!(
                    "skipping: no vcan interface (set PAR6_VCAN_IFACE, or \
                     `sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up`)"
                );
                return;
            }
        }
    };
}

/// One frame as observed on the wire.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Seen {
    node: NodeId,
    cmd: u8,
    err: bool,
    rtr: bool,
    data: Vec<u8>,
}

/// A second socket on the same interface: sees everything the bus
/// transmits, and injects the golden reply vectors.
struct Wire(CanSocket);

impl Wire {
    fn open(iface: &str) -> Self {
        let sock = CanSocket::open(iface).expect("open observer socket");
        sock.set_nonblocking(true).expect("nonblocking");
        // The boot load is ~190 frames before anything drains this side.
        let _ = sock.as_raw_socket().set_recv_buffer_size(4 * 1024 * 1024);
        Self(sock)
    }

    /// Everything transmitted since the last call.
    fn drain(&self) -> Vec<Seen> {
        let mut out = Vec::new();
        while let Ok(f) = self.0.read_frame() {
            let id = f.raw_id() as u16;
            out.push(Seen {
                node: ((id >> 7) & 0xF) as u8,
                cmd: ((id >> 1) & 0x3F) as u8,
                err: id & 1 == 1,
                rtr: f.is_remote_frame(),
                data: f.data().to_vec(),
            });
        }
        out
    }

    fn send(&self, id: u16, data: &[u8]) {
        let frame = socketcan::CanFrame::from_raw_id(u32::from(id), data).expect("classic frame");
        self.0.write_frame(&frame).expect("inject reply");
    }
}

fn configs(iface: &str) -> (RobotConfig, GripperConfig) {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let bundle = ConfigBundle::load(&root.join("config/PAR6.toml")).expect("PAR6 config bundle");
    let gripper = bundle
        .active_gripper()
        .filter(|g| g.driver.is_some())
        .expect("the active gripper has a CAN driver")
        .clone();
    let mut robot = bundle.robot;
    robot.bus.interface = iface.to_string();
    (robot, gripper)
}

/// Reply frames from the cross-language golden manifest, by vector name.
fn golden(name: &str) -> (u16, Vec<u8>) {
    let path =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/golden/can/manifest.json");
    let text = std::fs::read_to_string(&path).expect("golden manifest");
    let m: Value = serde_json::from_str(&text).expect("manifest parses");
    let v = m["vectors"]
        .as_array()
        .expect("vectors")
        .iter()
        .find(|v| v["name"] == name)
        .unwrap_or_else(|| panic!("golden vector {name} missing"));
    let id = u16::from_str_radix(
        v["id_hex"]
            .as_str()
            .expect("id_hex")
            .trim_start_matches("0x"),
        16,
    )
    .expect("hex id");
    let hex = v["data_hex"].as_str().expect("data_hex");
    let data = (0..hex.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).expect("hex byte"))
        .collect();
    (id, data)
}

/// Bring a bus up with the boot phases that need a responder switched
/// off, so tests that care about the tick can start from a silent wire.
fn quiet_bus(iface: &str) -> (SocketCanBus, RobotConfig, GripperConfig) {
    let (mut robot, gripper) = configs(iface);
    robot.robot.kt_source = KtSource::Config;
    robot.bus.scan.rounds = 0;
    let mut bus = SocketCanBus::open(&robot.bus).expect("open SocketCanBus");
    bus.boot_configure(&robot, Some(&gripper), 0)
        .expect("boot_configure");
    (bus, robot, gripper)
}

/// The boot config load must reach the wire batched BY MESSAGE TYPE with
/// a real pause between batches: ~190 frames enqueue in microseconds
/// against a ~10 frames/ms drain, and the interface TX queue drops the
/// overflow silently (spec/CAN.md boot step 2).
#[test]
fn boot_config_load_is_paced_and_ordered_on_the_wire() {
    let iface = require_vcan!();
    let (mut robot, gripper) = configs(&iface);
    robot.robot.kt_source = KtSource::Config;
    robot.bus.scan.rounds = 0;
    let wire = Wire::open(&iface);
    let mut bus = SocketCanBus::open(&robot.bus).expect("open SocketCanBus");
    let _ = wire.drain();

    let repeats = robot.bus.boot_config_repeats;
    let nodes: Vec<NodeId> = robot
        .joints
        .iter()
        .map(|j| j.node_id)
        .chain(std::iter::once(robot.bus.gripper_node))
        .collect();
    let started = Instant::now();
    bus.boot_configure(&robot, Some(&gripper), repeats)
        .expect("boot_configure");
    let elapsed = started.elapsed();

    let seen = wire.drain();
    let batches = usize::from(repeats) * 7;
    // The encoder seed sweep follows the config load.
    let config_frames = batches * nodes.len();
    assert!(
        seen.len() >= config_frames,
        "expected at least {config_frames} config frames, saw {}",
        seen.len()
    );
    let order = [
        CommandId::Watchdog,
        CommandId::Limits,
        CommandId::VoltageLimit,
        CommandId::PdGains,
        CommandId::CurrentGains,
        CommandId::VelocityGains,
        CommandId::PositionGains,
    ];
    for (b, chunk) in seen[..config_frames].chunks(nodes.len()).enumerate() {
        let want = order[b % order.len()];
        assert_eq!(
            chunk.iter().map(|s| (s.node, s.cmd)).collect::<Vec<_>>(),
            nodes.iter().map(|n| (*n, want.raw())).collect::<Vec<_>>(),
            "batch {b} must carry {want:?} to every node in configuration order"
        );
        assert!(chunk.iter().all(|s| !s.rtr && !s.err));
    }

    // Pacing is real time on the wire, not a comment: one pause per
    // batch, so the whole load cannot outrun the TX queue.
    let want_pace = Duration::from_secs_f64(robot.bus.config_pace_s) * batches as u32;
    assert!(
        elapsed >= want_pace,
        "boot took {elapsed:?}, less than the {want_pace:?} of batch pacing"
    );

    // The seed sweep asks every node for its accumulated encoder reading,
    // so the RT loop's boot sector selection starts from a real 14-bit
    // wrapped position instead of the first motion reply it happens upon.
    let seed = &seen[config_frames..];
    assert_eq!(
        seed.iter()
            .map(|s| (s.node, s.cmd, s.rtr))
            .collect::<Vec<_>>(),
        nodes
            .iter()
            .map(|n| (*n, CommandId::EncoderData.raw(), true))
            .collect::<Vec<_>>()
    );
}

/// Boot scan + kt fetch: every node id is pinged, driver kt replies that
/// arrive during boot survive into the first `drain_rx`, and the
/// connected map reflects who actually answered.
#[test]
fn boot_scan_and_kt_fetch_reach_the_first_bus_state() {
    let iface = require_vcan!();
    let (mut robot, gripper) = configs(&iface);
    robot.robot.kt_source = KtSource::Auto;
    // Keep the no-reply retry ladder short: this test is about the
    // frames, not about waiting out 0.35 s × retries × rounds.
    robot.bus.kt_fetch.timeout_s = 0.02;
    let wire = Wire::open(&iface);
    let mut bus = SocketCanBus::open(&robot.bus).expect("open SocketCanBus");
    let _ = wire.drain();

    // Queue the golden kt reply for J6 as if node 5's driver had answered
    // (0.151 Nm/A). It waits in the bus socket until the fetch drains it.
    let (kt_id, kt_data) = golden("rx_cmd33_kt");
    assert_eq!(kt_id, pack_can_id(5, CommandId::RespondKt, false));
    wire.send(kt_id, &kt_data);

    bus.boot_configure(&robot, Some(&gripper), 0)
        .expect("boot_configure");

    let seen = wire.drain();
    let pings: Vec<NodeId> = seen
        .iter()
        .filter(|s| s.cmd == CommandId::Ping.raw() && s.rtr)
        .map(|s| s.node)
        .collect();
    assert_eq!(
        pings.len(),
        16 * usize::from(robot.bus.scan.rounds),
        "the scan pings every node id, every round"
    );
    assert_eq!(
        pings
            .iter()
            .copied()
            .collect::<std::collections::BTreeSet<_>>(),
        (0..16u8).collect::<std::collections::BTreeSet<_>>()
    );
    // The queued reply is drained during the ladder's FIRST wait window
    // (node 0's), so by the time the fetch reaches node 5 it is already
    // answered — an answered node is never asked. The silent nodes are
    // where the RTRs go.
    assert!(
        seen.iter()
            .any(|s| s.cmd == CommandId::RespondKt.raw() && s.rtr),
        "kt is fetched with RTRs to cmd 33"
    );
    assert!(
        !seen
            .iter()
            .any(|s| s.node == 5 && s.cmd == CommandId::RespondKt.raw() && s.rtr),
        "a node whose kt is already known must not be re-asked"
    );
    assert_eq!(
        bus.connected_nodes() & (1 << 5),
        1 << 5,
        "the node that answered is connected"
    );

    // The boot reply is not lost between boot and the first tick.
    let mut state = BusState::new();
    bus.begin_tick(0);
    bus.drain_rx(&mut state).expect("drain");
    assert_eq!(state.nodes[5].kt_nm_a, Some(0.151));
    assert_eq!(state.nodes[4].kt_nm_a, None, "silent nodes stay unknown");
}

/// One steady-state tick end to end: the frame budget on the wire, the
/// drain decoding golden replies into state, and the freshness ladder
/// (warn → latched disconnect → user clear) driven by real traffic.
#[test]
fn tick_exchange_frame_budget_and_freshness_ladder() {
    let iface = require_vcan!();
    let wire = Wire::open(&iface);
    let (mut bus, robot, _gripper) = quiet_bus(&iface);
    let _ = wire.drain();
    let stale = u64::from(robot.ticks(robot.bus.stale_warn_s));
    let lost = u64::from(robot.ticks(robot.bus.lost_s));
    let joints = [JointCommand::idle(); 6];
    let mut state = BusState::new();

    bus.begin_tick(1);
    assert_eq!(bus.drain_rx(&mut state).expect("drain"), 0);
    bus.send_joint_commands(&joints).expect("joint frames");
    bus.send_gripper(&GripperCommand::FirmwarePoll)
        .expect("gripper frame");
    bus.poll_step().expect("poll");
    assert_eq!(
        bus.tx_frames_this_tick(),
        8,
        "6 joints + gripper slot + one poll — inside the classic-CAN budget"
    );
    let seen = wire.drain();
    assert_eq!(
        seen.iter().map(|s| (s.node, s.cmd)).collect::<Vec<_>>(),
        vec![
            (0, CommandId::DataPack1.raw()),
            (1, CommandId::DataPack1.raw()),
            (2, CommandId::DataPack1.raw()),
            (3, CommandId::DataPack1.raw()),
            (4, CommandId::DataPack1.raw()),
            (5, CommandId::DataPack1.raw()),
            (robot.bus.gripper_node, CommandId::GripperDataPack.raw()),
            (0, CommandId::Temperature.raw()),
        ]
    );
    // Velocity-mode motion frames omit the position channel (DLC 5), the
    // empty gripper poll is DLC 0, and the telemetry poll is remote.
    assert_eq!(seen[0].data.len(), 5);
    assert!(seen[6].data.is_empty() && !seen[6].rtr);
    assert!(seen[7].rtr);

    // Golden replies on the wire become decoded state.
    let (motion_id, motion) = golden("rx_cmd3_motion_err_bit");
    wire.send(motion_id, &motion);
    let (temp_id, temp) = golden("rx_cmd23_temperature_negative");
    wire.send(temp_id, &temp);
    let motion_node = ((motion_id >> 7) & 0xF) as NodeId;
    bus.begin_tick(2);
    let n = bus.drain_rx(&mut state).expect("drain");
    assert_eq!(n, 2);
    assert_eq!(state.frames_last_drain, 2);
    assert_eq!(
        state.nodes[usize::from(motion_node)].position_ticks,
        Some(100_000)
    );
    assert!(
        state.nodes[usize::from(motion_node)].live_error_bit,
        "the arbitration-id err bit is harvested per frame"
    );
    assert_eq!(state.nodes[usize::from(motion_node)].data_age_ticks, 0);
    assert_eq!(bus.freshness(motion_node), Freshness::Fresh);
    assert_eq!(state.reconnected_mask, 0);
    assert!(bus.link_health().rx_frames >= 2);

    // Age past the stale threshold: a live, self-clearing warning.
    bus.begin_tick(2 + stale);
    bus.drain_rx(&mut state).expect("drain");
    assert_eq!(bus.freshness(motion_node), Freshness::Stale);
    assert_eq!(state.nodes[usize::from(motion_node)].data_age_ticks, stale);
    // A frame while stale clears it and reports the reconnect edge, which
    // is what drives the config resend.
    wire.send(motion_id, &motion);
    std::thread::sleep(Duration::from_millis(2));
    bus.drain_rx(&mut state).expect("drain");
    assert_eq!(state.reconnected_mask, 1 << motion_node);
    assert_eq!(bus.freshness(motion_node), Freshness::Fresh);

    // Reaching the lost threshold LATCHES: resumed traffic does not clear it.
    bus.begin_tick(2 + stale + lost);
    bus.drain_rx(&mut state).expect("drain");
    assert_eq!(bus.freshness(motion_node), Freshness::Lost);
    wire.send(motion_id, &motion);
    std::thread::sleep(Duration::from_millis(2));
    bus.drain_rx(&mut state).expect("drain");
    assert_eq!(bus.freshness(motion_node), Freshness::Lost);
    // The clear drops the latch and re-arms the clock at "seen now": a
    // node that is still off the bus re-latches on its own rather than
    // going permanently un-reportable.
    bus.clear_lost_latch(motion_node);
    assert_eq!(bus.freshness(motion_node), Freshness::Fresh);
    bus.begin_tick(2 + stale + 2 * lost);
    assert_eq!(bus.freshness(motion_node), Freshness::Lost);
    bus.clear_lost_latch(motion_node);

    // FLASHING: bus-silent. Nothing may be transmitted, and RX is drained
    // but never decoded (bootloader page frames alias application ids).
    let _ = wire.drain();
    bus.set_silent(true);
    bus.begin_tick(3 + stale + 2 * lost);
    assert!(bus.send_joint_commands(&joints).is_err());
    assert!(bus.send_gripper(&GripperCommand::FirmwarePoll).is_err());
    bus.poll_step().expect("polls are suppressed, not an error");
    assert_eq!(bus.tx_frames_this_tick(), 0);
    assert!(wire.drain().is_empty(), "a silent bus transmits nothing");
    let (enc_id, enc) = golden("rx_cmd28_encoder");
    wire.send(enc_id, &enc);
    std::thread::sleep(Duration::from_millis(2));
    let enc_node = ((enc_id >> 7) & 0xF) as usize;
    let before = state.nodes[enc_node].position_ticks;
    assert_eq!(bus.drain_rx(&mut state).expect("drain"), 1);
    assert_eq!(
        state.nodes[enc_node].position_ticks, before,
        "silent drains discard undecoded"
    );
    bus.set_silent(false);
    bus.rebase_freshness();
    for j in &robot.joints {
        assert_eq!(bus.freshness(j.node_id), Freshness::Fresh);
    }
    // Re-base stamps "seen now", not "never seen", so a driver that did
    // not come back from the flash still latches after the lost window.
    bus.begin_tick(3 + stale + 3 * lost);
    for j in &robot.joints {
        assert_eq!(bus.freshness(j.node_id), Freshness::Lost);
    }
}

/// How many cmd-33 (kt) RTR asks `seen` carries, per node id.
fn kt_asks(seen: &[Seen]) -> [usize; 16] {
    let mut n = [0usize; 16];
    for s in seen {
        if s.cmd == CommandId::RespondKt.raw() && s.rtr {
            n[usize::from(s.node)] += 1;
        }
    }
    n
}

/// Every configured node, joints then gripper — the order and population
/// the boot kt fetch walks.
fn configured_nodes(robot: &RobotConfig) -> Vec<NodeId> {
    robot
        .joints
        .iter()
        .map(|j| j.node_id)
        .chain(std::iter::once(robot.bus.gripper_node))
        .collect()
}

/// Boot kt fetch, failure shape 1: NO node answers (spec/CAN.md boot
/// step 3 — config is the fallback for a driver that does not answer).
///
/// The retry ladder must actually go out on the wire — every configured
/// node asked retries×rounds times, each unanswered ask waiting out its
/// reply timeout — boot must still terminate, and the first drained state
/// must publish kt UNKNOWN for every node. `None` is the provenance
/// `RtCore::adopt_driver_kt` keys its per-joint config fallback off
/// (that adoption, and the fallback torque factor it builds, are asserted
/// at the RT layer in par6-rt's core_modes suite).
#[test]
fn kt_fetch_with_no_replies_exhausts_the_ladder_and_publishes_unknown() {
    let iface = require_vcan!();
    let (mut robot, gripper) = configs(&iface);
    robot.robot.kt_source = KtSource::Auto;
    robot.bus.scan.rounds = 0;
    robot.bus.kt_fetch.timeout_s = 0.02;
    robot.bus.kt_fetch.retries = 2;
    robot.bus.kt_fetch.rounds = 2;
    let wire = Wire::open(&iface);
    let mut bus = SocketCanBus::open(&robot.bus).expect("open SocketCanBus");
    let _ = wire.drain();

    let started = Instant::now();
    bus.boot_configure(&robot, Some(&gripper), 0)
        .expect("a bus with no drivers answering must still boot");
    let elapsed = started.elapsed();

    let nodes = configured_nodes(&robot);
    let asks = kt_asks(&wire.drain());
    let per_node = usize::from(robot.bus.kt_fetch.retries) * usize::from(robot.bus.kt_fetch.rounds);
    for n in &nodes {
        assert_eq!(
            asks[usize::from(*n)],
            per_node,
            "node {n}: the full retry ladder must reach the wire"
        );
    }
    // Each unanswered ask waits out its reply timeout: the ladder is a
    // real wall-clock budget, not a burst — and it is bounded.
    let floor = Duration::from_secs_f64(robot.bus.kt_fetch.timeout_s)
        .mul_f64((per_node * nodes.len()) as f64 * 0.9);
    assert!(
        elapsed >= floor,
        "ladder finished in {elapsed:?}, under its {floor:?} wait budget"
    );
    assert!(
        elapsed < Duration::from_secs(5),
        "the no-reply ladder must stay bounded, took {elapsed:?}"
    );

    let mut state = BusState::new();
    bus.begin_tick(0);
    bus.drain_rx(&mut state).expect("drain");
    for n in &nodes {
        assert_eq!(
            state.nodes[usize::from(*n)].kt_nm_a,
            None,
            "node {n}: no reply must publish UNKNOWN, never a default"
        );
    }
}

/// Boot kt fetch, failure shapes 2 and 3: a node that answers GARBAGE,
/// and a reply that arrives after the fetch window is over.
///
/// The bus is the provenance transport, not the policy. It must publish
/// exactly what each driver said — a non-positive kt included — and stop
/// asking any node that answered; the adopt-or-reject policy (a
/// non-finite or non-positive reply is REJECTED and the config factor
/// stays in effect) is `RtCore::adopt_driver_kt` in par6-rt. Likewise
/// "an answer after the timeout is not applied": the transport keeps
/// publishing kt whenever the frame lands, so the snapshot provenance
/// stays truthful, while the torque factor is rebuilt exactly once at
/// the RT core's boot resolve tick and never re-adopted afterwards.
#[test]
fn kt_fetch_garbage_and_late_replies_are_provenance_not_re_asks() {
    let iface = require_vcan!();
    let (mut robot, gripper) = configs(&iface);
    robot.robot.kt_source = KtSource::Auto;
    robot.bus.scan.rounds = 0;
    robot.bus.kt_fetch.timeout_s = 0.02;
    robot.bus.kt_fetch.retries = 2;
    robot.bus.kt_fetch.rounds = 2;
    let wire = Wire::open(&iface);
    let mut bus = SocketCanBus::open(&robot.bus).expect("open SocketCanBus");
    let _ = wire.drain();

    // Queued before boot, so both replies land in the ladder's first wait
    // window: the golden kt for node 5, and a garbage (negative) kt for
    // node 1 — the out-of-family shape a mis-flashed driver produces.
    let (kt_id, kt_data) = golden("rx_cmd33_kt");
    wire.send(kt_id, &kt_data);
    wire.send(pack_can_id(1, CommandId::RespondKt, false), &pack_f32(-0.5));

    bus.boot_configure(&robot, Some(&gripper), 0)
        .expect("boot_configure");

    let asks = kt_asks(&wire.drain());
    let per_node = usize::from(robot.bus.kt_fetch.retries) * usize::from(robot.bus.kt_fetch.rounds);
    assert_eq!(asks[5], 0, "an answered node is never asked again");
    assert_eq!(
        asks[1], 0,
        "even a garbage answer ends the asking — rejection is the RT \
         core's job, re-asking would just re-fetch the same garbage"
    );
    for n in configured_nodes(&robot) {
        if n != 1 && n != 5 {
            assert_eq!(
                asks[usize::from(n)],
                per_node,
                "node {n}: silent ⇒ full ladder"
            );
        }
    }

    // The first drain publishes the verbatim answers next to the silence.
    let mut state = BusState::new();
    bus.begin_tick(0);
    bus.drain_rx(&mut state).expect("drain");
    assert_eq!(state.nodes[5].kt_nm_a, Some(0.151));
    assert_eq!(
        state.nodes[1].kt_nm_a,
        Some(-0.5),
        "garbage is published as said, so the RT layer can see and reject it"
    );
    assert_eq!(state.nodes[0].kt_nm_a, None);

    // Shape 3: an answer arriving after the whole fetch window. The
    // transport still decodes and publishes it on the next tick drain —
    // late kt is telemetry provenance; only the RT core's one-shot boot
    // resolution decides what the torque factor was built from.
    wire.send(pack_can_id(0, CommandId::RespondKt, false), &pack_f32(0.2));
    std::thread::sleep(Duration::from_millis(2));
    bus.begin_tick(1);
    bus.drain_rx(&mut state).expect("drain");
    assert_eq!(state.nodes[0].kt_nm_a, Some(0.2));
}

/// The RT tick path allocates NOTHING after init (CLAUDE.md Rust rules),
/// asserted over the full backend — transport syscalls included — with a
/// counting allocator.
#[test]
fn steady_state_ticks_allocate_nothing() {
    let iface = require_vcan!();
    let wire = Wire::open(&iface);
    let (mut bus, _robot, _gripper) = quiet_bus(&iface);
    let joints = [JointCommand::idle(); 6];
    let mut state = BusState::new();
    let (motion_id, motion) = golden("rx_cmd3_motion_negative");
    let reply = socketcan::CanFrame::from_raw_id(u32::from(motion_id), &motion).expect("frame");

    let tick = |bus: &mut SocketCanBus, state: &mut BusState, t: u64| {
        wire.0.write_frame(&reply).expect("inject");
        bus.begin_tick(t);
        bus.drain_rx(state).expect("drain");
        bus.send_joint_commands(&joints).expect("joints");
        bus.send_gripper(&GripperCommand::NoGripper)
            .expect("gripper");
        bus.poll_step().expect("poll");
    };

    // Warm up: first touch of every buffer, and the poll override path.
    bus.queue_poll_override(
        par6_bus::PollAction::Poll {
            node: 0,
            kind: PollKind::Errors,
        },
        4,
    );
    for t in 1..20 {
        tick(&mut bus, &mut state, t);
    }

    let before = allocs();
    for t in 20..320 {
        tick(&mut bus, &mut state, t);
    }
    let after = allocs();
    assert_eq!(after - before, 0, "the tick path must not allocate");
    assert!(
        state.frames_last_drain > 0,
        "the measured window actually decoded frames"
    );
}
