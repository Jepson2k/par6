//! Socket-free coverage of the hardware backend's RX decode→state
//! mapping.
//!
//! The transport itself needs a real interface — `tests/socketcan_vcan.rs`
//! drives the full [`super::SocketCanBus`] over `vcan0` where one exists.

use super::*;
use crate::spectral::codec::{pack_can_id, pack_f32, pack_i16, pack_i24, CanFrame};
use crate::types::ObjectDetection;

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
