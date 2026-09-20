//! The STATUS_RATE reply: what the runtime answers a caller asking which
//! broadcast rates it will accept.

/// The servable-rate set travels on the wire, not as a rule the client
/// re-derives: `SET_STATUS_RATE` is checked against the runtime's divisors, and
/// a caller that recomputed them would disagree with it for any tick rate that
/// is not a whole number of Hz.
#[test]
fn a_status_rate_reply_carries_the_rates_the_runtime_serves() {
    use par6_proto::{decode_reply, encode_reply, QueryResult, Reply};

    let answered = QueryResult::StatusRate {
        hz: 50.0,
        tick_hz: 250.0,
        servable: vec![250.0, 125.0, 50.0, 25.0, 10.0, 5.0, 2.0, 1.0],
    };
    let mut buf = Vec::new();
    encode_reply(
        &Reply::Response {
            req_id: 9,
            result: answered.clone(),
        },
        &mut buf,
    );
    match decode_reply(&buf).expect("a status rate reply decodes") {
        Reply::Response {
            req_id: 9,
            result:
                QueryResult::StatusRate {
                    hz,
                    tick_hz,
                    servable,
                },
        } => {
            assert_eq!((hz, tick_hz), (50.0, 250.0));
            assert!(servable.contains(&hz), "the current rate is servable");
            assert_eq!(servable.first().copied(), Some(tick_hz));
            assert_eq!(servable.len(), 8);
        }
        other => panic!("decoded as {other:?}"),
    }
}
