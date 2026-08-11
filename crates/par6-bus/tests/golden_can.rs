//! Golden-vector conformance for the Spectral/STEPFOC CAN codec against
//! `tests/golden/can/manifest.json` (frames inline as hex — see the
//! manifest's `encoding` note).
//!
//! - `dir = "tx"`: build the typed command here, encode, byte-compare
//!   against the manifest frame.
//! - `dir = "rx"`: decode the manifest frame, value-compare against the
//!   typed expectation built here.
//! - `malformed`: decode must refuse with exactly the named error —
//!   whole-frame discard, err bit still harvestable.
//!
//! Coverage is two-way: every manifest vector must have a typed check
//! here, and every typed check must have a manifest vector.

use std::collections::BTreeSet;
use std::path::PathBuf;

use par6_bus::spectral::{
    decode_frame, encode_clear_error, encode_current_gains, encode_estop, encode_gripper_command,
    encode_heartbeat_setup, encode_idle, encode_joint_command, encode_kt, encode_limits,
    encode_pd_gains, encode_poll, encode_position_gains, encode_reset, encode_rtr_poll,
    encode_save_config, encode_set_can_id, encode_velocity_gains, encode_voltage_limit,
    encode_watchdog, pack_can_id, CanFrame, CommandId, DecodeError, DecodedFrame, Payload,
};
use par6_bus::{
    DeviceInfo, ErrorFlags, FirmwareGripperCommand, GripperCommand, GripperReply, HallState,
    JointCommand, ObjectDetection, Pack, PollKind,
};
use par6_config::WatchdogAction;
use serde_json::Value;

fn manifest() -> Value {
    let path =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/golden/can/manifest.json");
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("{}: {e} — golden manifest missing", path.display()));
    serde_json::from_str(&text).expect("manifest.json parses")
}

fn hex_bytes(s: &str) -> Vec<u8> {
    assert!(s.len().is_multiple_of(2), "odd hex length: {s:?}");
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).expect("hex"))
        .collect()
}

fn entry_frame(v: &Value) -> CanFrame {
    let id =
        u16::from_str_radix(v["id_hex"].as_str().unwrap().trim_start_matches("0x"), 16).unwrap();
    let data = hex_bytes(v["data_hex"].as_str().unwrap());
    if v["rtr"].as_bool().unwrap() {
        assert!(data.is_empty(), "RTR frames carry no data");
        CanFrame::rtr_frame(id)
    } else {
        CanFrame::data_frame(id, &data)
    }
}

fn entries<'a>(m: &'a Value, list: &str) -> Vec<&'a Value> {
    m[list].as_array().expect(list).iter().collect()
}

fn find<'a>(list: &[&'a Value], name: &str) -> &'a Value {
    list.iter()
        .find(|v| v["name"].as_str() == Some(name))
        .unwrap_or_else(|| panic!("vector {name:?} missing from manifest"))
}

fn assert_covered(list: &[&Value], dir: Option<&str>, checked: &BTreeSet<&str>) {
    let manifest_names: BTreeSet<&str> = list
        .iter()
        .filter(|v| dir.is_none_or(|d| v["dir"].as_str() == Some(d)))
        .map(|v| v["name"].as_str().unwrap())
        .collect();
    assert_eq!(
        &manifest_names, checked,
        "manifest vectors and typed checks must cover each other exactly"
    );
}

/// The manifest's node/cmd/err fields, its id_hex, and the codec's id
/// packing must all agree, for every vector including the malformed ones.
#[test]
fn manifest_ids_are_consistent_and_names_unique() {
    let m = manifest();
    let mut names = BTreeSet::new();
    for v in entries(&m, "vectors")
        .into_iter()
        .chain(entries(&m, "malformed"))
    {
        let name = v["name"].as_str().unwrap();
        assert!(names.insert(name.to_owned()), "duplicate vector {name:?}");
        let id = u16::from_str_radix(v["id_hex"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();
        let node = v["node"].as_u64().unwrap() as u16;
        let cmd = v["cmd"].as_u64().unwrap() as u16;
        let err = v["err"].as_u64().unwrap() as u16;
        assert_eq!(
            id,
            (node << 7) | (cmd << 1) | err,
            "{name}: id_hex does not match (node<<7)|(cmd<<1)|err"
        );
        if let Some(c) = CommandId::from_raw(cmd as u8) {
            assert_eq!(
                pack_can_id(node as u8, c, err == 1),
                id,
                "{name}: codec id packing disagrees"
            );
        }
        assert!(
            hex_bytes(v["data_hex"].as_str().unwrap()).len() <= 8,
            "{name}: CAN payload over 8 bytes"
        );
    }
}

#[test]
fn tx_vectors_encode_byte_exact() {
    let m = manifest();
    let list = entries(&m, "vectors");
    let joint = |n, jc: &JointCommand| encode_joint_command(n, jc).unwrap().unwrap();
    let grip = |gc: &GripperCommand| encode_gripper_command(6, 13, gc).unwrap().unwrap();
    let encoded: Vec<(&str, CanFrame)> = vec![
        (
            "tx_cmd2_pos_j0",
            joint(0, &JointCommand::position(1000, -2000, 300)),
        ),
        (
            "tx_cmd2_pos_negative_i24",
            joint(2, &JointCommand::position(-150, -187, -3047)),
        ),
        (
            "tx_cmd2_vel_j1",
            joint(1, &JointCommand::velocity(-40000, 250)),
        ),
        (
            "tx_cmd2_vel_cur_none_substitutes_zero",
            joint(
                1,
                &JointCommand {
                    pos: None,
                    vel: Some(-40000),
                    cur_ma: None,
                    pack: Pack::Pid,
                },
            ),
        ),
        ("tx_cmd2_cur_only", joint(5, &JointCommand::current(-1))),
        ("tx_cmd2_idle", joint(3, &JointCommand::idle())),
        ("tx_cmd4_pd", joint(0, &JointCommand::pd(1000, -2000, 300))),
        ("tx_cmd31_hall", joint(5, &JointCommand::hall(12000, 2))),
        (
            "tx_cmd61_gripper",
            grip(&GripperCommand::Firmware(FirmwareGripperCommand {
                position: 252,
                speed: 150,
                current_ma: 500,
                activate: true,
                action: true,
                estop: false,
                release_dir: false,
            })),
        ),
        ("tx_cmd61_empty_poll", grip(&GripperCommand::FirmwarePoll)),
        ("tx_cmd62_calibrate", grip(&GripperCommand::Calibrate)),
        ("tx_no_gripper_dummy_ping", grip(&GripperCommand::NoGripper)),
        (
            "tx_cmd15_watchdog",
            encode_watchdog(0, 5000, WatchdogAction::Idle),
        ),
        ("tx_cmd20_limits", encode_limits(0, 80000.0, 1800.0)),
        ("tx_cmd34_voltage_limit", encode_voltage_limit(0, 6000)),
        ("tx_cmd16_pd_gains", encode_pd_gains(0, 0.12, 0.002)),
        ("tx_cmd17_current_gains", encode_current_gains(0, 7.0, 0.9)),
        (
            "tx_cmd18_velocity_gains",
            encode_velocity_gains(0, 0.015, 0.0015),
        ),
        ("tx_cmd19_position_gains", encode_position_gains(0, 5.0)),
        ("tx_cmd22_kt", encode_kt(0, 0.28)),
        ("tx_cmd30_heartbeat_setup", encode_heartbeat_setup(4, 1000)),
        ("tx_cmd11_set_can_id", encode_set_can_id(0, 5)),
        ("tx_cmd0_estop", encode_estop(2)),
        ("tx_cmd1_clear_error", encode_clear_error(2)),
        ("tx_cmd12_idle", encode_idle(2)),
        ("tx_cmd13_save_config", encode_save_config(2)),
        ("tx_cmd14_reset", encode_reset(2)),
        ("tx_rtr_ping", encode_poll(3, PollKind::Ping)),
        ("tx_rtr_temperature", encode_poll(3, PollKind::Temperature)),
        ("tx_rtr_voltage", encode_poll(3, PollKind::Voltage)),
        ("tx_rtr_device_info", encode_poll(3, PollKind::DeviceInfo)),
        ("tx_rtr_errors", encode_poll(3, PollKind::Errors)),
        ("tx_rtr_iq", encode_rtr_poll(3, CommandId::IqData).unwrap()),
        ("tx_rtr_encoder", encode_poll(3, PollKind::Encoder)),
        ("tx_rtr_kt", encode_poll(3, PollKind::Kt)),
    ];
    for (name, frame) in &encoded {
        let want = entry_frame(find(&list, name));
        assert_eq!(frame, &want, "{name}: encoded frame differs from golden");
    }
    assert_covered(
        &list,
        Some("tx"),
        &encoded.iter().map(|(n, _)| *n).collect(),
    );
}

#[test]
fn rx_vectors_decode_value_exact() {
    let m = manifest();
    let list = entries(&m, "vectors");
    let expected: Vec<(&str, DecodedFrame)> = vec![
        (
            "rx_cmd3_motion_negative",
            DecodedFrame {
                node: 0,
                err_bit: false,
                payload: Payload::Motion {
                    position_ticks: -150,
                    speed_ticks_s: -187,
                    current_ma: 3047,
                },
            },
        ),
        (
            "rx_cmd3_motion_err_bit",
            DecodedFrame {
                node: 1,
                err_bit: true,
                payload: Payload::Motion {
                    position_ticks: 100_000,
                    speed_ticks_s: -50,
                    current_ma: -300,
                },
            },
        ),
        (
            "rx_cmd28_encoder",
            DecodedFrame {
                node: 2,
                err_bit: false,
                payload: Payload::Encoder {
                    position_ticks: -123_456,
                    speed_ticks_s: 2_000_000,
                },
            },
        ),
        (
            "rx_cmd32_hall",
            DecodedFrame {
                node: 5,
                err_bit: false,
                payload: Payload::Hall {
                    position_ticks: 3969,
                    state: HallState {
                        trigger: true,
                        pin2: false,
                        edge: true,
                    },
                },
            },
        ),
        (
            "rx_cmd23_temperature_negative",
            DecodedFrame {
                node: 4,
                err_bit: false,
                payload: Payload::Temperature { deg_c: -5 },
            },
        ),
        (
            "rx_cmd24_voltage",
            DecodedFrame {
                node: 4,
                err_bit: false,
                payload: Payload::Voltage { mv: 24_123 },
            },
        ),
        (
            "rx_cmd27_iq",
            DecodedFrame {
                node: 4,
                err_bit: false,
                payload: Payload::IqCurrent { ma: -1200 },
            },
        ),
        (
            "rx_cmd25_device_info",
            DecodedFrame {
                node: 0,
                err_bit: false,
                payload: Payload::DeviceInfo(DeviceInfo {
                    hw_ver: 2,
                    batch: 7,
                    sw_ver: 14,
                    serial: 305_419_896,
                }),
            },
        ),
        (
            "rx_cmd26_errors",
            DecodedFrame {
                node: 3,
                err_bit: true,
                payload: Payload::Errors(ErrorFlags {
                    error: true,
                    temperature: false,
                    encoder: true,
                    vbus: false,
                    driver: false,
                    velocity: false,
                    current: false,
                    estop: true,
                    calibrated: true,
                    activated: true,
                    watchdog: true,
                }),
            },
        ),
        (
            "rx_cmd33_kt",
            DecodedFrame {
                node: 5,
                err_bit: false,
                payload: Payload::Kt { nm_per_a: 0.151 },
            },
        ),
        (
            "rx_cmd60_gripper",
            DecodedFrame {
                node: 6,
                err_bit: false,
                payload: Payload::Gripper(GripperReply {
                    position: 252,
                    current_ma: -120,
                    activated: true,
                    action_status: false,
                    object_detection: ObjectDetection::DetectedOpening,
                    temperature_error: false,
                    timeout_error: false,
                    estop_error: false,
                    calibrated: true,
                }),
            },
        ),
        (
            "rx_cmd10_ping_reply",
            DecodedFrame {
                node: 2,
                err_bit: false,
                payload: Payload::Ping,
            },
        ),
        (
            "rx_cmd9_heartbeat",
            DecodedFrame {
                node: 2,
                err_bit: false,
                payload: Payload::Heartbeat,
            },
        ),
    ];
    for (name, want) in &expected {
        let frame = entry_frame(find(&list, name));
        let got = decode_frame(&frame)
            .unwrap_or_else(|e| panic!("{name}: golden frame failed to decode: {e}"));
        assert_eq!(&got, want, "{name}: decoded value differs from golden");
    }
    assert_covered(
        &list,
        Some("rx"),
        &expected.iter().map(|(n, _)| *n).collect(),
    );
}

#[test]
fn malformed_vectors_are_refused_whole() {
    let m = manifest();
    let list = entries(&m, "malformed");
    use DecodeError::*;
    let expected: Vec<(&str, DecodeError)> = vec![
        (
            "bad_cmd3_short_dlc5",
            WrongDlc {
                node: 0,
                cmd: 3,
                err_bit: false,
                dlc: 5,
                expected: 8,
            },
        ),
        (
            "bad_cmd3_empty",
            WrongDlc {
                node: 0,
                cmd: 3,
                err_bit: false,
                dlc: 0,
                expected: 8,
            },
        ),
        (
            "bad_cmd32_dlc3",
            WrongDlc {
                node: 5,
                cmd: 32,
                err_bit: false,
                dlc: 3,
                expected: 4,
            },
        ),
        (
            "bad_cmd60_dlc5",
            WrongDlc {
                node: 6,
                cmd: 60,
                err_bit: false,
                dlc: 5,
                expected: 4,
            },
        ),
        (
            "bad_cmd26_dlc1",
            WrongDlc {
                node: 3,
                cmd: 26,
                err_bit: false,
                dlc: 1,
                expected: 2,
            },
        ),
        (
            "bad_cmd25_dlc8",
            WrongDlc {
                node: 0,
                cmd: 25,
                err_bit: false,
                dlc: 8,
                expected: 7,
            },
        ),
        (
            "bad_cmd33_dlc2",
            WrongDlc {
                node: 5,
                cmd: 33,
                err_bit: false,
                dlc: 2,
                expected: 4,
            },
        ),
        (
            "bad_cmd5_reserved",
            Reserved {
                node: 4,
                cmd: 5,
                err_bit: true,
            },
        ),
        (
            "bad_cmd7_reserved",
            Reserved {
                node: 4,
                cmd: 7,
                err_bit: false,
            },
        ),
        (
            "bad_cmd2_echo_not_reply",
            NotAReply {
                node: 0,
                cmd: 2,
                err_bit: false,
            },
        ),
        (
            "bad_rtr_not_reply",
            NotAReply {
                node: 3,
                cmd: 23,
                err_bit: false,
            },
        ),
        (
            "bad_bootloader_alias",
            NotAReply {
                node: 3,
                cmd: 20,
                err_bit: false,
            },
        ),
        (
            "bad_unknown_cmd21",
            UnknownCommand {
                node: 1,
                cmd: 21,
                err_bit: false,
            },
        ),
        (
            "bad_unknown_cmd63_bootloader_node",
            UnknownCommand {
                node: 15,
                cmd: 63,
                err_bit: true,
            },
        ),
    ];
    for (name, want) in &expected {
        let frame = entry_frame(find(&list, name));
        let got =
            decode_frame(&frame).expect_err(&format!("{name}: malformed frame must be refused"));
        assert_eq!(&got, want, "{name}: refusal differs from golden");
        // The live fault bit and node id survive every refusal — the RX
        // drain harvests them before discarding the payload.
        assert_eq!(got.node(), want.node());
        assert_eq!(got.err_bit(), want.err_bit());
    }
    assert_covered(&list, None, &expected.iter().map(|(n, _)| *n).collect());
}
