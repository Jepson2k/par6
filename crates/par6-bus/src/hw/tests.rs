//! Socket-free coverage of the hardware backend's RX decode→state
//! mapping.
//!
//! The transport itself needs a real interface — `tests/socketcan_vcan.rs`
//! drives the full [`super::SocketCanBus`] over `vcan0` where one exists.

use super::*;
use crate::spectral::codec::{pack_can_id, CanFrame};
use crate::types::ObjectDetection;

fn decode_into(frame: &CanFrame, state: &mut BusState) {
    let d = decode_frame(frame).expect("decodable reply");
    apply_payload(&d, state);
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
