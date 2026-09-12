//! A full TCP correction must survive the wire without accepting partial or nonfinite data.
use par6_proto::command::SetTcpTransform;
use par6_proto::{
    decode_command, decode_reply, encode_command, encode_reply, CmdType, Command, QueryResult,
    Reply,
};

fn packet(values: [f64; 6]) -> Vec<u8> {
    let mut out = vec![0x99, CmdType::SetTcpTransform as u8, 7, 3];
    for value in values {
        out.push(0xcb);
        out.extend_from_slice(&value.to_be_bytes());
    }
    out
}

#[test]
fn full_transform_codec_rejects_corrupt_corrections_and_readbacks() {
    let values = [5.0, -3.0, 20.0, 20.0, 25.0, -10.0];
    let cmd = Command::SetTcpTransform(SetTcpTransform {
        key: 3,
        x: values[0],
        y: values[1],
        z: values[2],
        roll: values[3],
        pitch: values[4],
        yaw: values[5],
    });
    let wire = packet(values);
    assert_eq!(decode_command(&wire).unwrap(), (7, cmd.clone()));
    let mut encoded = Vec::new();
    encode_command(&cmd, 7, &mut encoded).unwrap();
    assert_eq!(decode_command(&encoded).unwrap(), (7, cmd));
    encode_command(&Command::TcpTransform, 8, &mut encoded).unwrap();
    assert_eq!(
        decode_command(&encoded).unwrap(),
        (8, Command::TcpTransform)
    );
    for end in 0..wire.len() {
        assert!(decode_command(&wire[..end]).is_err());
    }
    let mut extra = wire.clone();
    extra.push(0xc0);
    assert!(decode_command(&extra).is_err());
    let mut wrong_arity = wire.clone();
    wrong_arity[0] = 0x98;
    assert!(decode_command(&wrong_arity).is_err());
    for slot in 0..6 {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut v = values;
            v[slot] = bad;
            assert!(decode_command(&packet(v)).is_err());
            let response = Reply::Response {
                req_id: 7,
                result: QueryResult::TcpTransform { values: v },
            };
            encode_reply(&response, &mut encoded);
            assert!(decode_reply(&encoded).is_err());
        }
        for marker in [0xc3, 0xc0, 0xa0] {
            let mut malformed = wire.clone();
            malformed.splice(4 + slot * 9..4 + (slot + 1) * 9, [marker]);
            assert!(decode_command(&malformed).is_err());
        }
    }
    let response = Reply::Response {
        req_id: 7,
        result: QueryResult::TcpTransform { values },
    };
    encode_reply(&response, &mut encoded);
    assert_eq!(decode_reply(&encoded).unwrap(), response);
    for end in 0..encoded.len() {
        assert!(decode_reply(&encoded[..end]).is_err());
    }
}
