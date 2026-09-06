//! What the codec does with bytes nobody's client would send.
//!
//! The command port binds `0.0.0.0` with no authentication, so every
//! decode path here is reachable by anything that can route a UDP
//! datagram to it. Two classes are covered:
//!
//! - **Length headers.** msgpack's 0xDD array form declares up to
//!   4 294 967 295 elements in five bytes. A decoder that reserves on
//!   that word before reading an element turns nine bytes into an
//!   allocation failure, and `handle_alloc_error` aborts the process —
//!   taking the RT thread and all CAN traffic with it.
//! - **Durations.** Every one ends up in `Duration::from_secs_f64` /
//!   `Instant + Duration`, both of which PANIC near f64's range, and the
//!   shipped runtime is built `panic = "abort"`.
//!
//! Datagrams are hand-assembled rather than encoded: `encode_command`
//! validates, so the only way to present the codec with what an attacker
//! presents it with is to write the bytes.

use par6_proto::command::{MAX_DURATION_S, MAX_JOG_DURATION_S, MAX_SHAPES, MAX_WAYPOINTS};
use par6_proto::{
    decode_command, decode_reply, decode_status, encode_command, CmdType, Command, DecodeError,
};

// ---- a minimal msgpack writer (the wire, not the codec) --------------------

fn fixarray(n: usize) -> Vec<u8> {
    assert!(n < 16);
    vec![0x90 | n as u8]
}

fn array32(n: u32) -> Vec<u8> {
    let mut v = vec![0xDD];
    v.extend_from_slice(&n.to_be_bytes());
    v
}

fn uint(v: u64) -> Vec<u8> {
    assert!(v < 0x80, "test only needs single-byte uints");
    vec![v as u8]
}

fn f64be(v: f64) -> Vec<u8> {
    let mut out = vec![0xCB];
    out.extend_from_slice(&v.to_be_bytes());
    out
}

fn nil() -> Vec<u8> {
    vec![0xC0]
}

fn cat(parts: &[Vec<u8>]) -> Vec<u8> {
    parts.concat()
}

/// `[JOG_J, req_id, speeds[6], duration, nil]`.
fn jog_j_bytes(duration: f64) -> Vec<u8> {
    let mut speeds = fixarray(6);
    for i in 0..6 {
        speeds.extend(f64be(if i == 0 { 0.2 } else { 0.0 }));
    }
    cat(&[
        fixarray(5),
        uint(CmdType::JogJ as u64),
        uint(7),
        speeds,
        f64be(duration),
        nil(),
    ])
}

/// `[JOG_L, req_id, velocities[6], duration, frame, nil]`.
fn jog_l_bytes(duration: f64) -> Vec<u8> {
    let mut vels = fixarray(6);
    for i in 0..6 {
        vels.extend(f64be(if i == 0 { 0.2 } else { 0.0 }));
    }
    cat(&[
        fixarray(6),
        uint(CmdType::JogL as u64),
        uint(7),
        vels,
        f64be(duration),
        uint(0), // WRF
        nil(),
    ])
}

/// `[DELAY, req_id, key, seconds]`.
fn delay_bytes(seconds: f64) -> Vec<u8> {
    cat(&[
        fixarray(4),
        uint(CmdType::Delay as u64),
        uint(7),
        uint(1),
        f64be(seconds),
    ])
}

/// `[MOVE_S, req_id, key, <waypoint array header>, …]` — truncated
/// straight after the header, which is all an allocation attack needs.
fn move_s_header(declared: u32) -> Vec<u8> {
    cat(&[
        fixarray(9),
        uint(CmdType::MoveS as u64),
        uint(0),
        uint(0),
        array32(declared),
    ])
}

fn refusal(data: &[u8], what: &str) -> String {
    match decode_command(data) {
        Err(e) => e.to_string(),
        Ok((_, cmd)) => panic!("{what} was accepted: {cmd:?}"),
    }
}

// ---- length headers --------------------------------------------------------

/// The nine bytes from the review: a well-formed MOVE_S envelope whose
/// waypoint array claims every element msgpack can count. Reserving on
/// that word asks the allocator for ~206 GB; the process does not come
/// back from that, so this test's failure mode against the bug is a
/// crashed test binary, not a failed assertion.
#[test]
fn a_nine_byte_datagram_cannot_ask_for_a_gigabyte() {
    let data = move_s_header(u32::MAX);
    assert_eq!(data.len(), 9, "the whole attack is nine bytes");
    let why = refusal(&data, "a 4-billion-waypoint header");
    assert!(
        why.contains("move_s.waypoints"),
        "the refusal must name the field: {why}"
    );

    // Same shape on every other length-prefixed field.
    for (what, data) in [
        (
            "set_shapes.shapes",
            cat(&[
                fixarray(3),
                uint(CmdType::SetShapes as u64),
                uint(0),
                array32(u32::MAX),
            ]),
        ),
        (
            "tool_action.params",
            cat(&[
                fixarray(6),
                uint(CmdType::ToolAction as u64),
                uint(0),
                uint(1),
                vec![0xA1, b'g'], // tool_key
                vec![0xA1, b'o'], // action
                array32(u32::MAX),
            ]),
        ),
        ("teleport.tool_positions", {
            let mut angles = fixarray(6);
            for _ in 0..6 {
                angles.extend(f64be(0.0));
            }
            cat(&[
                fixarray(4),
                uint(CmdType::Teleport as u64),
                uint(0),
                angles,
                array32(u32::MAX),
            ])
        }),
    ] {
        let why = refusal(&data, what);
        assert!(why.contains(what), "the refusal must name {what}: {why}");
    }
}

/// The cap is what bounds the reservation, so a header UNDER it that the
/// datagram cannot back up must still fail — as truncation, having
/// reserved only what the cap permits.
#[test]
fn a_header_the_datagram_cannot_back_up_is_truncation_not_a_reservation() {
    let data = move_s_header(MAX_WAYPOINTS as u32);
    assert!(
        matches!(decode_command(&data), Err(DecodeError::Truncated)),
        "a plausible header with no elements behind it must read as truncated"
    );
}

/// The caps admit everything a real program sends, and refuse the count
/// above them — the planner-work bound, not just the allocator one.
#[test]
fn the_counts_a_real_program_sends_still_decode() {
    let mut buf = Vec::new();
    let waypoints = |n: usize| {
        Command::MoveS(par6_proto::command::MoveS {
            key: 1,
            waypoints: (0..n)
                .map(|i| [i as f64, 0.0, 0.0, 0.0, 0.0, 0.0])
                .collect(),
            frame: par6_proto::Frame::Wrf,
            duration: Some(1.0),
            speed: None,
            accel: None,
            rel: false,
        })
    };
    encode_command(&waypoints(MAX_WAYPOINTS), 1, &mut buf).expect("the cap itself must encode");
    let (_, decoded) = decode_command(&buf).expect("and decode");
    assert_eq!(decoded, waypoints(MAX_WAYPOINTS));

    let too_many = encode_command(&waypoints(MAX_WAYPOINTS + 1), 1, &mut buf)
        .expect_err("one past the cap must be refused")
        .to_string();
    assert!(too_many.contains("move_s.waypoints"), "{too_many}");

    let shapes = |n: usize| {
        Command::SetShapes(par6_proto::command::SetShapes {
            shapes: (0..n)
                .map(|i| par6_proto::Shape {
                    kind: "sphere".into(),
                    params: vec![0.05],
                    pose: vec![0.0; 6],
                    collision: true,
                    margin: None,
                    name: format!("s{i}"),
                    physics: None,
                })
                .collect(),
        })
    };
    encode_command(&shapes(MAX_SHAPES), 1, &mut buf).expect("a full shape world must encode");
    decode_command(&buf).expect("and decode");
    let too_many = encode_command(&shapes(MAX_SHAPES + 1), 1, &mut buf)
        .expect_err("one shape past the cap must be refused")
        .to_string();
    assert!(too_many.contains("set_shapes.shapes"), "{too_many}");
}

// ---- durations -------------------------------------------------------------

/// A `jog_j` duration is a watchdog, and it is bounded on both sides:
/// large enough to panic the runtime's own arithmetic, and merely large
/// enough to outlive the operator, are both refused at the codec — the
/// layer every other range check lives in.
#[test]
fn a_jog_duration_can_neither_abort_the_runtime_nor_outlive_the_operator() {
    // Reproduces the abort: `Duration::from_secs_f64` panics above
    // ~1.8e19 s and `Instant + Duration` above ~9.2e18 s.
    for hostile in [1e30, 1e19, f64::MAX] {
        for (what, data) in [
            ("jog_j.duration", jog_j_bytes(hostile)),
            ("jog_l.duration", jog_l_bytes(hostile)),
        ] {
            let why = refusal(&data, &format!("{what} = {hostile}"));
            assert!(why.contains(what), "{why}");
        }
    }

    // ...and the non-panicking sibling: one datagram arming the watchdog
    // for a year defeats the watchdog just as thoroughly.
    let why = refusal(&jog_j_bytes(1e9), "a 31-year jog watchdog");
    assert!(why.contains("jog_j.duration"), "{why}");

    // The boundary is inclusive, and what a UI actually streams is well
    // inside it.
    for ok in [0.05, 0.1, 1.0, MAX_JOG_DURATION_S] {
        decode_command(&jog_j_bytes(ok)).unwrap_or_else(|e| panic!("jog_j({ok}) refused: {e}"));
        decode_command(&jog_l_bytes(ok)).unwrap_or_else(|e| panic!("jog_l({ok}) refused: {e}"));
    }
    let why = refusal(
        &jog_j_bytes(MAX_JOG_DURATION_S + 1.0),
        "one second past the jog ceiling",
    );
    assert!(why.contains("jog_j.duration"), "{why}");
}

/// Every other duration reaches the same panicking arithmetic through
/// the exec path, so the same bound applies — just a looser one, since
/// an hour-long dwell is a real program step.
#[test]
fn a_queued_dwell_is_bounded_by_the_same_arithmetic() {
    for hostile in [1e30, f64::MAX] {
        let why = refusal(&delay_bytes(hostile), "an unbounded delay");
        assert!(why.contains("delay.seconds"), "{why}");
    }
    decode_command(&delay_bytes(MAX_DURATION_S)).expect("an hour-long dwell is a real program");
    let why = refusal(
        &delay_bytes(MAX_DURATION_S + 1.0),
        "one second past the duration ceiling",
    );
    assert!(why.contains("delay.seconds"), "{why}");
}

// ---------------------------------------------------------------------------
// The REPLY and STATUS decoders, which the command-side guards never covered
// ---------------------------------------------------------------------------

/// A length header must not size an allocation before it is checked.
///
/// `command::r_len` has guarded this on the command side since it was
/// written, with a comment explaining the hazard. The reply and status
/// decoders reserved on the sender's word instead, so an eleven-byte
/// datagram could ask the allocator for a hundred gigabytes and abort the
/// process on `handle_alloc_error`. STATUS is a multicast broadcast, so
/// any host on the segment can send one.
#[test]
fn reply_and_status_length_headers_are_bounded_before_reserving() {
    // [RESPONSE, req_id, [TOOLS, "", <array32 4294967295>]]
    let hostile_reply: &[u8] = &[
        0x93, 0x04, 0x01, 0x93, 0x07, 0xA0, 0xDD, 0xFF, 0xFF, 0xFF, 0xFF,
    ];
    let err = decode_reply(hostile_reply).expect_err("a 4-billion string list must be refused");
    assert!(
        matches!(err, DecodeError::Validation { .. }),
        "the header must be refused on its own terms, not by running out \
         of bytes after reserving: {err:?}"
    );

    // The same shape reaching the status decoder's collision-pair list.
    let mut hostile_status = vec![0xDD, 0xFF, 0xFF, 0xFF, 0xFF];
    hostile_status.insert(0, 0x91);
    // The status decoder refuses this well before the pair list — the
    // point is that it refuses at all rather than reserving on the word.
    decode_status(&hostile_status).expect_err("a malformed status must be refused");
}
