//! Socket-free coverage of the hardware backend: the RX decode→state
//! mapping, and the no-allocation contract of everything the RT tick does
//! between syscalls.
//!
//! The transport itself needs a real interface — `tests/socketcan_vcan.rs`
//! drives the full [`super::SocketCanBus`] over `vcan0` where one exists.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use par6_config::{Gains, WatchdogAction};

use super::sched::{config_frame, ConfigKind, FreshnessClock, NodeConfig, PollScheduler};
use super::*;
use crate::spectral::codec::{
    encode_gripper_command, encode_joint_command, pack_can_id, pack_f32, pack_i16, pack_i24,
    CanFrame,
};
use crate::types::{
    FirmwareGripperCommand, GripperCommand, JointCommand, ObjectDetection, Pack, MAX_NODES,
};

// Per-THREAD, because the lib test binary runs its tests concurrently and
// a process-wide counter would fold every other test's allocations into
// the measured window.
thread_local! {
    static ALLOCS: Cell<u64> = const { Cell::new(0) };
}

fn allocs() -> u64 {
    ALLOCS.with(Cell::get)
}

struct CountingAlloc;

// SAFETY: every operation is delegated to the system allocator unchanged;
// the counter is a side effect on a `Cell<u64>`, which has no destructor
// and so never re-enters the allocator.
unsafe impl GlobalAlloc for CountingAlloc {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        count();
        unsafe { System.alloc(layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        count();
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        count();
        unsafe { System.realloc(ptr, layout, new_size) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

fn count() {
    let _ = ALLOCS.try_with(|c| c.set(c.get() + 1));
}

#[global_allocator]
static ALLOC: CountingAlloc = CountingAlloc;

fn node_config(node: NodeId) -> NodeConfig {
    NodeConfig {
        node,
        watchdog_ms: 5000,
        watchdog_action: WatchdogAction::Idle,
        velocity_limit_ticks_s: 80000.0,
        ilim_ma: 1200.0,
        voltage_limit_mv: 6000,
        gains: Gains {
            kpp: 1.0,
            kpv: 2.0,
            kiv: 3.0,
            kpiq: 4.0,
            kiiq: 5.0,
            kp: 6.0,
            kd: 7.0,
        },
    }
}

fn decode_into(frame: &CanFrame, state: &mut BusState) {
    let d = decode_frame(frame).expect("decodable reply");
    apply_payload(&d, state);
}

/// The drain's decode→state mapping, driven with hand-packed reply
/// frames: each reply class must land in its own field, the live fault
/// bit rides every frame, and the gripper reply reaches the gripper slot.
#[test]
fn replies_map_onto_their_own_bus_state_fields() {
    let mut state = BusState::new();

    // rx_cmd3_motion_negative: node 0, pos -150, spd -187, cur 3047 mA.
    let mut motion = [0u8; 8];
    motion[0..3].copy_from_slice(&pack_i24(-150));
    motion[3..6].copy_from_slice(&pack_i24(-187));
    motion[6..8].copy_from_slice(&pack_i16(3047));
    decode_into(
        &CanFrame::data_frame(pack_can_id(0, CommandId::RespondDataPack1, false), &motion),
        &mut state,
    );
    assert_eq!(state.nodes[0].position_ticks, Some(-150));
    assert_eq!(state.nodes[0].speed_ticks_s, Some(-187));
    assert_eq!(state.nodes[0].current_ma, Some(3047));
    assert!(!state.nodes[0].live_error_bit);

    // The same node, err bit set: the live fault signal is per frame.
    decode_into(
        &CanFrame::data_frame(pack_can_id(0, CommandId::RespondDataPack1, true), &motion),
        &mut state,
    );
    assert!(state.nodes[0].live_error_bit);

    // Telemetry replies must not disturb the motion fields.
    decode_into(
        &CanFrame::data_frame(pack_can_id(0, CommandId::Temperature, false), &pack_i16(-5)),
        &mut state,
    );
    decode_into(
        &CanFrame::data_frame(pack_can_id(0, CommandId::Voltage, false), &pack_i16(24123)),
        &mut state,
    );
    decode_into(
        &CanFrame::data_frame(
            pack_can_id(0, CommandId::StateOfErrors, false),
            &[0xa1, 0xe0],
        ),
        &mut state,
    );
    decode_into(
        &CanFrame::data_frame(
            pack_can_id(0, CommandId::RespondKt, false),
            &pack_f32(0.151),
        ),
        &mut state,
    );
    assert_eq!(state.nodes[0].temperature_c, Some(-5));
    assert_eq!(state.nodes[0].voltage_mv, Some(24123));
    let flags = state.nodes[0].error_flags.expect("cmd 26 decoded");
    assert!(flags.error && flags.encoder && flags.estop);
    assert!(flags.calibrated && flags.activated);
    assert_eq!(state.nodes[0].kt_nm_a, Some(0.151));
    assert_eq!(state.nodes[0].position_ticks, Some(-150));
    assert!(
        !state.nodes[0].live_error_bit,
        "a clean reply clears the live bit"
    );

    // cmd 27 Iq is a current refresh, not a separate channel.
    decode_into(
        &CanFrame::data_frame(pack_can_id(0, CommandId::IqData, false), &pack_i16(-1200)),
        &mut state,
    );
    assert_eq!(state.nodes[0].current_ma, Some(-1200));

    // Firmware gripper reply lands in the gripper slot, not in nodes[].
    decode_into(
        &CanFrame::data_frame(
            pack_can_id(6, CommandId::RespondGripperData, true),
            &[0xfc, 0xff, 0x88, 0xa1],
        ),
        &mut state,
    );
    let g = state.gripper.reply.expect("cmd 60 decoded");
    assert_eq!(g.position, 252);
    assert_eq!(g.current_ma, -120);
    // 0xa1 = 0b1010_0001: bit 5 set, bit 4 clear. Firmware puts the
    // status value's LOW bit at 5 and its HIGH bit at 4, so that is
    // value 1 — detected while closing.
    assert_eq!(g.object_detection, ObjectDetection::DetectedClosing);
    assert!(g.activated && g.calibrated);
    assert!(state.gripper.live_error_bit);
    assert_eq!(state.nodes[6].position_ticks, None);
}

/// Every gripper object-detection code, packed the way the firmware packs
/// it rather than the way our decoder happens to read it.
///
/// `Gripper_pack_data` builds a bool array `{activated, action_status,
/// detection_bit_1, detection_bit_2, …}` where `detection_bit_1` is the
/// status value's LSB and `detection_bit_2` its MSB, then `bitsToByte`
/// maps array index `i` onto bit `7 - i`. So the LSB lands on bit 5 and
/// the MSB on bit 4 — the opposite order from reading the byte's own bits
/// high-to-low, which is what made codes 1 and 2 decode transposed.
#[test]
fn gripper_object_detection_matches_the_firmware_bit_order() {
    /// Pack a status byte exactly as the firmware does, for status `v`.
    fn firmware_status_byte(v: u8) -> u8 {
        let lsb = v & 1;
        let msb = (v >> 1) & 1;
        (lsb << 5) | (msb << 4)
    }

    for (value, expected) in [
        (0u8, ObjectDetection::Moving),
        (1, ObjectDetection::DetectedClosing),
        (2, ObjectDetection::DetectedOpening),
        (3, ObjectDetection::ReachedNoObject),
    ] {
        let mut state = BusState::default();
        decode_into(
            &CanFrame::data_frame(
                pack_can_id(6, CommandId::RespondGripperData, false),
                &[0, 0, 0, firmware_status_byte(value)],
            ),
            &mut state,
        );
        let g = state.gripper.reply.expect("cmd 60 decoded");
        assert_eq!(
            g.object_detection, expected,
            "firmware status {value} must decode as {expected:?}"
        );
    }
}

/// Everything the RT tick does between syscalls — schedule the poll slot,
/// age the freshness clock, encode the joint/gripper/config frames — must
/// allocate nothing (CLAUDE.md Rust rules). `tests/socketcan_vcan.rs`
/// makes the same assertion over the full `DriverBus` path, transport
/// included, where a CAN interface exists.
#[test]
fn tick_path_work_allocates_nothing() {
    let mut poll = PollScheduler::default();
    poll.configure(7);
    let mut fresh = FreshnessClock::default();
    fresh.configure(10, 50);
    let configs: Vec<NodeConfig> = (0..7).map(node_config).collect();
    let commands = [
        JointCommand::position(1000, 2000, 300),
        JointCommand::velocity(-500, 250),
        JointCommand::current(-150),
        JointCommand::hall(4500, 2),
        JointCommand::pd(10, 0, 50),
        JointCommand::idle(),
    ];
    let gripper = GripperCommand::Firmware(FirmwareGripperCommand {
        position: 128,
        speed: 40,
        current_ma: 600,
        activate: true,
        action: true,
        estop: false,
        release_dir: false,
    });
    let mut state = BusState::new();
    let reply = CanFrame::data_frame(
        pack_can_id(3, CommandId::RespondDataPack1, false),
        &[0, 0, 1, 0, 0, 2, 0, 3],
    );
    // Warm up: first touches of every buffer happen before measuring.
    for tick in 0..4u64 {
        drive(tick, &mut poll, &mut fresh, &configs, &commands, &gripper);
        decode_into(&reply, &mut state);
    }

    let before = allocs();
    for tick in 4..504u64 {
        drive(tick, &mut poll, &mut fresh, &configs, &commands, &gripper);
        let d = decode_frame(&reply).expect("decodable");
        apply_payload(&d, &mut state);
        fresh.mark(d.node, tick);
        for n in 0..MAX_NODES {
            state.nodes[n].data_age_ticks = fresh.age(n as NodeId, tick);
        }
    }
    let after = allocs();
    assert_eq!(after - before, 0, "the tick path must not allocate");
}

fn drive(
    tick: u64,
    poll: &mut PollScheduler,
    fresh: &mut FreshnessClock,
    configs: &[NodeConfig],
    commands: &[JointCommand; 6],
    gripper: &GripperCommand,
) {
    fresh.latch_lost(tick);
    for (i, cmd) in commands.iter().enumerate() {
        let _ = encode_joint_command(i as NodeId, cmd).expect("encodable");
    }
    let _ = encode_gripper_command(6, 13, gripper).expect("encodable");
    let _ = encode_gripper_command(6, 13, &GripperCommand::NoGripper).expect("encodable");
    let _ = poll.step().expect("configured");
    let _ = config_frame(ConfigKind::Limits, &configs[tick as usize % configs.len()]);
    let _ = fresh.classify(0, tick);
}

/// Frames the RT loop can hand the backend that have no wire form must be
/// refused at the encode boundary, not silently dropped on the bus.
#[test]
fn position_without_velocity_has_no_wire_form() {
    let cmd = JointCommand {
        pos: Some(100),
        vel: None,
        cur_ma: Some(0),
        pack: Pack::Pid,
    };
    assert!(encode_joint_command(0, &cmd).is_err());
    // All channels omitted: nothing goes on the wire at all.
    assert_eq!(
        encode_joint_command(
            0,
            &JointCommand {
                pos: None,
                vel: None,
                cur_ma: None,
                pack: Pack::Pid,
            }
        ),
        Ok(None)
    );
}
