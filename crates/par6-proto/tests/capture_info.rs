//! Capture source identity must survive the wire exactly, including absence.
//! These are codec tests; no recorder, server, simulator, or drive is started.

use par6_proto::{
    command_class, decode_command, decode_reply, encode_command, encode_reply, CaptureIdentity,
    CmdType, Command, CommandClass, DecodeError, QueryResult, Reply,
};

const REQUEST_ID: u32 = 17;
const FINGERPRINT: &str = "51e5e041ec2ae87bfa4c656e7d91f7891455bcf450da94294c369503970ec18a";

fn identity() -> CaptureIdentity {
    CaptureIdentity {
        // Exercise the full u64 wire field rather than silently narrowing it.
        pid: 0x1_0000_002a,
        started: 1_789_118_610.25,
        dt: 0.004,
        fingerprint: FINGERPRINT.to_owned(),
    }
}

fn f64_bytes(value: f64) -> Vec<u8> {
    let mut bytes = vec![0xcb];
    bytes.extend_from_slice(&value.to_be_bytes());
    bytes
}

fn identity_fields() -> Vec<Vec<u8>> {
    let id = identity();
    let mut pid = vec![0xcf];
    pid.extend_from_slice(&id.pid.to_be_bytes());
    let mut fingerprint = vec![0xd9, 64];
    fingerprint.extend_from_slice(FINGERPRINT.as_bytes());
    vec![pid, f64_bytes(id.started), f64_bytes(id.dt), fingerprint]
}

fn array(fields: &[Vec<u8>]) -> Vec<u8> {
    assert!(fields.len() < 16);
    let mut bytes = vec![0x90 | fields.len() as u8];
    for field in fields {
        bytes.extend_from_slice(field);
    }
    bytes
}

fn response(value: Vec<u8>) -> Vec<u8> {
    // [RESPONSE=4, req_id=17, [CAPTURE_INFO=24, identity-or-nil]].
    let mut bytes = vec![0x93, 4, 17, 0x92, 24];
    bytes.extend(value);
    bytes
}

#[test]
fn capture_query_and_optional_identity_round_trip_without_field_loss() {
    let mut command = Vec::new();
    encode_command(&Command::CaptureInfo, REQUEST_ID, &mut command).unwrap();
    // Pin the additive slot and its no-argument shape independently of enums.
    assert_eq!(command, [0x92, 63, 17]);
    assert_eq!(command_class(CmdType::CaptureInfo), CommandClass::Query);
    assert_eq!(
        decode_command(&command).unwrap(),
        (REQUEST_ID, Command::CaptureInfo)
    );

    for recording in [None, Some(identity())] {
        let expected = response(if recording.is_some() {
            array(&identity_fields())
        } else {
            vec![0xc0]
        });
        let reply = Reply::Response {
            req_id: REQUEST_ID,
            result: QueryResult::CaptureInfo {
                identity: recording,
            },
        };
        let mut encoded = Vec::new();
        encode_reply(&reply, &mut encoded);
        assert_eq!(encoded, expected);
        assert_eq!(decode_reply(&expected).unwrap(), reply);
        assert_eq!(decode_reply(&encoded).unwrap(), reply);
    }
}

#[test]
fn capture_query_and_response_reject_truncation_and_trailing_bytes() {
    let command = vec![0x92, 63, 17];
    for end in 0..command.len() {
        assert!(decode_command(&command[..end]).is_err(), "prefix {end}");
    }
    let mut trailing_command = command;
    trailing_command.push(0xc0);
    assert!(matches!(
        decode_command(&trailing_command),
        Err(DecodeError::TrailingBytes)
    ));

    for reply in [response(vec![0xc0]), response(array(&identity_fields()))] {
        for end in 0..reply.len() {
            assert!(decode_reply(&reply[..end]).is_err(), "prefix {end}");
        }
        let mut trailing_reply = reply;
        trailing_reply.push(0xc0);
        assert!(matches!(
            decode_reply(&trailing_reply),
            Err(DecodeError::TrailingBytes)
        ));
    }
}

#[test]
fn capture_identity_rejects_malformed_arrays_and_field_types() {
    for command in [
        vec![0x91, 63],             // Missing request id.
        vec![0x93, 63, 17, 0xc0],   // Unexpected command argument.
        vec![0x92, 63, 0xa1, b'x'], // String request id.
        vec![0x92, 63, 0xff],       // Negative request id.
    ] {
        assert!(decode_command(&command).is_err());
    }
    for reply in [
        vec![0x93, 4, 17, 0x91, 24],             // Missing identity-or-nil field.
        vec![0x93, 4, 17, 0x93, 24, 0xc0, 0xc0], // Extra query field.
        response(vec![0xc3]),                    // Bool is not nil or an array.
        response(vec![0x80]),                    // Map is not an identity array.
        response(vec![0x90]),                    // Empty array is not absence.
        response(vec![0xdd, 0xff, 0xff, 0xff, 0xff]), // Impossible identity arity.
    ] {
        assert!(decode_reply(&reply).is_err());
    }
    let fields = identity_fields();
    assert!(decode_reply(&response(array(&fields[..3]))).is_err());
    let mut extra = fields.clone();
    extra.push(vec![0xc0]);
    assert!(decode_reply(&response(array(&extra))).is_err());

    let mut binary_fingerprint = vec![0xc4, 64];
    binary_fingerprint.extend_from_slice(FINGERPRINT.as_bytes());
    for (field, replacement) in [
        (0, vec![0xff]),         // PID must be unsigned.
        (0, f64_bytes(42.0)),    // A float is not a PID.
        (1, vec![1]),            // Start time requires float64.
        (1, vec![0xa1, b'x']),   // String start time.
        (2, vec![0xc0]),         // Missing period inside a present identity.
        (2, vec![0xc3]),         // Bool period.
        (3, vec![1]),            // Integer fingerprint.
        (3, binary_fingerprint), // Bytes are not a UTF-8 string.
        (3, vec![0xa1, 0xff]),   // Invalid UTF-8 fingerprint.
    ] {
        let mut bad = fields.clone();
        bad[field] = replacement;
        assert!(
            decode_reply(&response(array(&bad))).is_err(),
            "accepted invalid identity field {field}"
        );
    }
}
