//! COMMAND_COMPLETION: the query names a queue index, its answer carries
//! what that index's COMPLETE push did (or that nothing finished), and
//! neither survives a truncated or corrupt datagram.
use par6_proto::{
    decode_command, decode_reply, encode_command, encode_reply, Command, QueryResult, Reply,
    WireError,
};

#[test]
fn a_completion_query_and_its_answers_round_trip_and_refuse_corruption() {
    let cmd = Command::CommandCompletion { index: 42 };
    let mut wire = Vec::new();
    encode_command(&cmd, 9, &mut wire).unwrap();
    assert_eq!(decode_command(&wire).unwrap(), (9, cmd));
    for end in 0..wire.len() {
        assert!(decode_command(&wire[..end]).is_err());
    }

    let cancelled = WireError {
        command_index: 42,
        code: 38,
        title: "Command cancelled".into(),
        cause: "a stop discarded it".into(),
        effect: "it did not run".into(),
        remedy: "resend it".into(),
    };
    let answers = [
        QueryResult::CommandCompletion {
            index: 42,
            finished: true,
            ok: true,
            detail: None,
            verdict: Some(3),
        },
        QueryResult::CommandCompletion {
            index: 42,
            finished: true,
            ok: true,
            detail: None,
            verdict: None,
        },
        QueryResult::CommandCompletion {
            index: 42,
            finished: true,
            ok: false,
            detail: Some(cancelled),
            verdict: None,
        },
        QueryResult::CommandCompletion {
            index: 7,
            finished: false,
            ok: false,
            detail: None,
            verdict: None,
        },
    ];
    for result in answers {
        let reply = Reply::Response { req_id: 9, result };
        let mut bytes = Vec::new();
        encode_reply(&reply, &mut bytes);
        assert_eq!(decode_reply(&bytes).unwrap(), reply);
        for end in 0..bytes.len() {
            assert!(decode_reply(&bytes[..end]).is_err());
        }
    }

    // A settle verdict outside 1..=3 is refused, as it is on COMPLETE.
    let mut bytes = Vec::new();
    encode_reply(
        &Reply::Response {
            req_id: 9,
            result: QueryResult::CommandCompletion {
                index: 42,
                finished: true,
                ok: true,
                detail: None,
                verdict: Some(3),
            },
        },
        &mut bytes,
    );
    let last = bytes.len() - 1;
    assert_eq!(bytes[last], 3, "the verdict is the last byte");
    bytes[last] = 4;
    assert!(decode_reply(&bytes).is_err());
}
