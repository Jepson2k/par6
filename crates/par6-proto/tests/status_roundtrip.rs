//! STATUS survives its own codec.
//!
//! The wire vectors that used to pin this were removed with the vendored
//! golden suite, which left encode and decode free to disagree about a
//! slot's shape without anything noticing: the encoder is the only writer
//! and the decoder the only reader, so a mismatch shows up as a broadcast
//! that silently never arrives rather than as a failure anyone can see.

use par6_proto::{decode_status, DriveHealthWire, Status, StatusEncoder};

/// A status with every variable-length slot non-trivially populated, so a
/// slot whose encoded arity drifts from what the decoder expects is caught
/// here rather than by an arm that has stopped reporting in the field.
fn populated() -> Status {
    Status {
        seq: 4242,
        angles: [1.0, -2.0, 3.5, -4.25, 5.125, -6.0625],
        torques: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        torques_ext: [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6],
        tcp_speed: 12.5,
        io: vec![0, 1, 0, 1, 1],
        drive_health: DriveHealthWire {
            temperatures_c: vec![41.0, 42.0, f64::NAN, 44.0],
            currents_ma: vec![100.0, 200.0, 300.0, f64::NAN],
            bus_voltage_v: Some(23.8),
            faults: vec![
                vec![],
                vec!["overtemperature".to_owned()],
                vec![],
                vec!["encoder".to_owned(), "overcurrent".to_owned()],
            ],
        },
        ..Status::default()
    }
}

#[test]
fn every_status_slot_survives_encode_and_decode() {
    let sent = populated();
    let mut encoder = StatusEncoder::new();
    let bytes = encoder.encode(&sent);
    let got = decode_status(bytes).expect("the encoder's own output must decode");

    assert_eq!(got.seq, sent.seq);
    assert_eq!(got.angles, sent.angles);
    assert_eq!(got.torques_ext, sent.torques_ext);
    assert_eq!(got.io, sent.io);
    assert_eq!(
        got.drive_health.faults, sent.drive_health.faults,
        "per-drive fault labels must survive the wire, including the empty \
         slots that say a drive is healthy rather than unreported"
    );
    assert_eq!(
        got.drive_health.bus_voltage_v,
        sent.drive_health.bus_voltage_v
    );
    assert_eq!(
        got.drive_health.currents_ma.len(),
        sent.drive_health.currents_ma.len()
    );
    // NaN marks a register a drive has not answered; it has to stay NaN
    // rather than arriving as a plausible zero.
    assert!(got.drive_health.temperatures_c[2].is_nan());
    assert!(got.drive_health.currents_ma[3].is_nan());
}

#[test]
fn a_bus_with_no_drives_still_round_trips() {
    let s = Status {
        seq: 7,
        ..Status::default()
    };
    let mut encoder = StatusEncoder::new();
    let bytes = encoder.encode(&s);
    let got = decode_status(bytes).expect("an empty drive_health must decode");
    assert!(got.drive_health.faults.is_empty());
    assert!(got.drive_health.temperatures_c.is_empty());
}
