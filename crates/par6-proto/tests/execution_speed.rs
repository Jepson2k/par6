//! Speed selection and pause must remain distinct across the wire.
use par6_proto::command::{Pause, SetExecutionSpeed};
use par6_proto::{
    decode_command, decode_reply, encode_command, encode_reply, CmdType, Command, QueryResult,
    Reply,
};

fn request(scale: f64) -> Vec<u8> {
    let mut bytes = vec![0x93, CmdType::SetExecutionSpeed as u8, 7, 0xcb];
    bytes.extend_from_slice(&scale.to_be_bytes());
    bytes
}

#[test]
fn execution_speed_codec_rejects_pause_as_speed_and_corrupt_readback() {
    let mut bytes = Vec::new();
    for scale in [0.1, 0.5, 1.0] {
        let command = Command::SetExecutionSpeed(SetExecutionSpeed { scale });
        assert_eq!(
            decode_command(&request(scale)).unwrap(),
            (7, command.clone())
        );
        encode_command(&command, 7, &mut bytes).unwrap();
        assert_eq!(decode_command(&bytes).unwrap(), (7, command));
    }
    for command in [
        Command::Pause(Pause { on: true }),
        Command::Pause(Pause { on: false }),
        Command::ExecutionSpeed,
    ] {
        encode_command(&command, 8, &mut bytes).unwrap();
        assert_eq!(decode_command(&bytes).unwrap(), (8, command));
    }
    for bad in [
        0.0,
        -0.1,
        0.099,
        1.001,
        2.0,
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ] {
        assert!(decode_command(&request(bad)).is_err());
        assert!(encode_command(
            &Command::SetExecutionSpeed(SetExecutionSpeed { scale: bad }),
            7,
            &mut bytes
        )
        .is_err());
    }
    let good = request(0.5);
    for end in 0..good.len() {
        assert!(decode_command(&good[..end]).is_err());
    }
    for marker in [0xc0, 0xc2, 0xc3, 0xa0, 0x90] {
        assert!(decode_command(&[0x93, CmdType::SetExecutionSpeed as u8, 7, marker]).is_err());
    }
    let mut extra = good.clone();
    extra.push(0xc0);
    assert!(decode_command(&extra).is_err());
    extra[0] = 0x94;
    assert!(decode_command(&extra).is_err());

    for (target_scale, applied_scale, resume_scale, valid) in [
        (1.0, 0.5, 1.0, true),
        (0.0, 0.05, 0.1, true),
        (0.0, 0.0, 0.6, true),
        (0.5, 0.5, 1.0, false),
        (0.0, 0.0, 0.0, false),
        (0.0, -0.1, 0.5, false),
        (0.0, 1.1, 0.5, false),
        (f64::NAN, 0.0, 0.5, false),
        (0.0, f64::NAN, 0.5, false),
        (0.0, 0.0, f64::INFINITY, false),
    ] {
        let response = Reply::Response {
            req_id: 7,
            result: QueryResult::ExecutionSpeed {
                target_scale,
                applied_scale,
                resume_scale,
            },
        };
        encode_reply(&response, &mut bytes);
        if valid {
            assert_eq!(decode_reply(&bytes).unwrap(), response);
            for end in 0..bytes.len() {
                assert!(decode_reply(&bytes[..end]).is_err());
            }
        } else {
            assert!(decode_reply(&bytes).is_err());
        }
    }
}
