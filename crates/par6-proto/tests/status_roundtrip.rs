//! STATUS survives its own codec.
//!
//! The wire vectors that used to pin this were removed with the vendored
//! golden suite, which left encode and decode free to disagree about a
//! slot's shape without anything noticing: the encoder is the only writer
//! and the decoder the only reader, so a mismatch shows up as a broadcast
//! that silently never arrives rather than as a failure anyone can see.

use par6_proto::{
    decode_status, make_error, ActionState, ControllerMode, DriveHealthWire, ErrorCode, HomingWire,
    LinkHealthWire, LoopHealthWire, Status, StatusEncoder, ToolState, ToolStatusWire, UNATTRIBUTED,
};

/// A status with every slot set to something other than its default and
/// different from every neighbour of the same type, so a slot whose
/// encoded arity drifts from what the decoder expects, or whose value
/// lands in the wrong slot, is caught here rather than by an arm that has
/// stopped reporting in the field.
fn populated() -> Status {
    Status {
        proto_version: par6_proto::PROTO_VERSION,
        controller_id: 77,
        seq: 4242,
        mono_time_ns: 9_000_000_123,
        link_ok: 1,
        data_age_ms: 17,
        pose: std::array::from_fn(|i| 0.5 + i as f64),
        angles: [1.0, -2.0, 3.5, -4.25, 5.125, -6.0625],
        speeds: [0.01, -0.02, 0.03, -0.04, 0.05, -0.06],
        io: vec![0, 1, 0, 1, 1],
        action_current: "move_l".to_owned(),
        action_state: ActionState::Executing,
        joint_en: std::array::from_fn(|i| (i % 2) as u8),
        cart_en_wrf: std::array::from_fn(|i| ((i + 1) % 2) as u8),
        cart_en_trf: std::array::from_fn(|i| u8::from(i % 3 == 0)),
        executing_index: 12,
        completed_index: 11,
        last_checkpoint: "pick".to_owned(),
        error: Some(make_error(
            ErrorCode::MotnCancelled,
            11,
            &[("scope", "stop")],
        )),
        queued_segments: 3,
        queued_duration: 4.5,
        action_params: "[0.1]".to_owned(),
        tool_status: Some(ToolStatusWire {
            key: "GRIPPER".to_owned(),
            state: ToolState::Active,
            engaged: true,
            part_detected: true,
            fault_code: -3,
            positions: vec![0.25],
            channels: vec![0.5, 0.75],
            variant_key: "wide".to_owned(),
        }),
        tcp_speed: 12.5,
        simulator_active: true,
        collision_active: false,
        collision_pairs: vec![("forearm".to_owned(), "cage".to_owned())],
        scene_epoch: 9,
        accepted_index: 13,
        homed: true,
        torques: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        mode: ControllerMode::Homing,
        enabled: true,
        gravity_comp: false,
        warnings: vec![make_error(
            ErrorCode::TrajNearSingularity,
            UNATTRIBUTED,
            &[],
        )],
        link_health: LinkHealthWire {
            state: 2,
            restarts: 4,
            tx_errors: 5,
            rx_frames: 6,
        },
        homing: HomingWire {
            active: true,
            sequence_step: 2,
            joints: vec![(1, 2), (3, 4)],
        },
        torques_ext: [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6],
        paused: true,
        drive_health: DriveHealthWire {
            temperatures_c: vec![41.0, 42.0, 43.0, 44.0],
            currents_ma: vec![100.0, 200.0, 300.0, 400.0],
            bus_voltage_v: Some(23.8),
            faults: vec![
                vec![],
                vec!["overtemperature".to_owned()],
                vec![],
                vec!["encoder".to_owned(), "overcurrent".to_owned()],
            ],
        },
        loop_health: LoopHealthWire {
            p99_period_s: 0.0041,
            overruns: 8,
        },
        session_id: u64::MAX - 1,
    }
}

fn round_trip(s: &Status) -> Status {
    let mut encoder = StatusEncoder::new();
    decode_status(encoder.encode(s)).expect("the encoder's own output must decode")
}

#[test]
fn every_status_slot_survives_encode_and_decode() {
    let sent = populated();
    assert_eq!(round_trip(&sent), sent);
    // Neighbouring flags differ above; flipped, each one is read for
    // itself rather than for a value it shares with its slot's default.
    let mut flipped = populated();
    for flag in [
        &mut flipped.simulator_active,
        &mut flipped.collision_active,
        &mut flipped.homed,
        &mut flipped.enabled,
        &mut flipped.gravity_comp,
        &mut flipped.paused,
        &mut flipped.homing.active,
    ] {
        *flag = !*flag;
    }
    assert_eq!(round_trip(&flipped), flipped);

    // NaN marks a register a drive has not answered; it has to stay NaN
    // rather than arriving as a plausible zero.
    let mut unanswered = populated();
    unanswered.drive_health.temperatures_c[2] = f64::NAN;
    unanswered.drive_health.currents_ma[3] = f64::NAN;
    let got = round_trip(&unanswered);
    assert!(got.drive_health.temperatures_c[2].is_nan());
    assert!(got.drive_health.currents_ma[3].is_nan());

    // A newer daemon's STATUS, with fields appended past the ones this
    // codec knows (a nested one among them), still decodes to them.
    let mut newer = Vec::new();
    par6_proto::encode_status_into(&sent, &mut newer);
    assert_eq!(newer[0], 0xDC, "STATUS_LEN no longer encodes as array16");
    let longer = (par6_proto::STATUS_LEN as u16) + 2;
    newer[1..3].copy_from_slice(&longer.to_be_bytes());
    newer.extend_from_slice(&[0x92, 0x01, 0x92, 0xa1, b'x', 0xc3]); // [1, ["x", true]]
    newer.push(0x07);
    assert_eq!(
        decode_status(&newer).expect("appended fields are skipped"),
        sent
    );

    // A bus with no drives reports empty lists, not missing ones.
    let bare = round_trip(&Status {
        seq: 7,
        ..Status::default()
    });
    assert!(bare.drive_health.faults.is_empty());
    assert!(bare.drive_health.temperatures_c.is_empty());
}

#[test]
fn session_metadata_requires_an_unsigned_integer_and_complete_field() {
    let mut encoder = StatusEncoder::new();
    let mut prefix = encoder.encode(&Status::default()).to_vec();
    prefix.pop();
    assert!(decode_status(&prefix).is_err());
    for invalid in [vec![0xc0], vec![0xc3], vec![0xff], vec![0xa1, b'1']] {
        let mut packet = prefix.clone();
        packet.extend_from_slice(&invalid);
        assert!(decode_status(&packet).is_err());
    }
}

/// A STATUS from an OLDER daemon must still yield its protocol version.
///
/// This is the whole point of `peek_status_proto_version`. Adding fields
/// grows the array, so a v4 producer sends fewer elements than a v5 client
/// requires and `decode_status` refuses it on ARITY — before it has read the
/// version. A client holding only `decode_status` therefore cannot tell a
/// version skew from a corrupt datagram and reports neither: it drops the
/// frame at debug level and goes quiet, which is the failure this exists to
/// explain.
///
/// The fixture is a real encoding with its array header shortened rather
/// than hand-written bytes, so it stays honest if the header layout moves.
#[test]
fn an_older_daemons_status_still_reports_its_version() {
    let mut buf = Vec::new();
    par6_proto::encode_status_into(
        &Status {
            proto_version: 4,
            ..populated()
        },
        &mut buf,
    );
    assert_eq!(
        par6_proto::peek_status_proto_version(&buf),
        Some(4),
        "the version must be readable from a STATUS that DOES decode"
    );

    // Now the skew itself: an older producer sends fewer elements. The
    // header is msgpack array16 — 0xDC then a big-endian count.
    assert_eq!(buf[0], 0xDC, "STATUS_LEN no longer encodes as array16");
    let short = (par6_proto::STATUS_LEN as u16) - 1;
    buf[1..3].copy_from_slice(&short.to_be_bytes());

    assert!(
        decode_status(&buf).is_err(),
        "a short STATUS must not decode; if it did, this test is not \
         exercising the skew case at all"
    );
    assert_eq!(
        par6_proto::peek_status_proto_version(&buf),
        Some(4),
        "the version must be readable from the datagram decode_status refused"
    );

    // ...and it claims nothing for a datagram that is not a STATUS.
    assert_eq!(par6_proto::peek_status_proto_version(&[]), None);
    assert_eq!(
        par6_proto::peek_status_proto_version(&[0x90]),
        None,
        "an empty array carries no tag"
    );
    assert_eq!(
        par6_proto::peek_status_proto_version(&[0x92, 0x7F, 0x04]),
        None,
        "a two-element array whose tag is not Status"
    );
}
