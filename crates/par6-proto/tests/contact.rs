//! The contact readout across the wire: what the arm stops for, what it
//! has seen, and nothing a corrupt reply could pass off as either.
use par6_proto::{
    decode_command, decode_reply, encode_command, encode_reply, CmdType, Command, ContactChannel,
    ContactReaction, ContactResult, QueryResult, Reply,
};

fn readout() -> ContactResult {
    ContactResult {
        reaction: ContactReaction::Stop,
        sustained_nm: [1.2, 2.0, 1.7, 0.5, 0.5, 0.3],
        impact_nm: [1.1, 1.0, 0.9, 0.3, 0.3, 0.2],
        signal: [0.1, -0.4, 0.0, 0.02, -0.01, 0.0],
        impact: [0.05, -0.1, 0.0, 0.0, 0.01, 0.0],
        peak_signal: [0.5, 0.9, 0.3, 0.1, 0.05, 0.04],
        peak_impact: [0.4, 0.3, 0.2, 0.08, 0.05, 0.03],
        trips: 3,
        last_trip: Some((2, ContactChannel::Impact)),
        angles: [0.0, -60.0, 220.0, 0.0, 30.0, 180.0],
        gravity_nm: [0.0, -6.3, 2.1, 0.0, 0.15, 0.0],
    }
}

fn reply(result: ContactResult) -> Reply {
    Reply::Response {
        req_id: 9,
        result: QueryResult::Contact(result),
    }
}

#[test]
fn the_contact_query_and_its_readout_survive_the_wire() {
    let mut bytes = Vec::new();
    encode_command(&Command::Contact, 9, &mut bytes).unwrap();
    assert_eq!(bytes, [0x92, CmdType::Contact as u8, 9]);
    assert_eq!(decode_command(&bytes).unwrap(), (9, Command::Contact));
    // A query carries nothing but its tag and request id.
    assert!(decode_command(&[0x93, CmdType::Contact as u8, 9, 0xc0]).is_err());
    assert!(decode_command(&[0x91, CmdType::Contact as u8]).is_err());

    for result in [
        readout(),
        ContactResult {
            reaction: ContactReaction::Warn,
            trips: 0,
            last_trip: None,
            ..readout()
        },
        ContactResult {
            reaction: ContactReaction::Off,
            last_trip: Some((5, ContactChannel::Sustained)),
            ..readout()
        },
    ] {
        let response = reply(result);
        encode_reply(&response, &mut bytes);
        assert_eq!(decode_reply(&bytes).unwrap(), response);
        for end in 0..bytes.len() {
            assert!(
                decode_reply(&bytes[..end]).is_err(),
                "a reply cut at {end} bytes decoded"
            );
        }
        let mut extra = bytes.clone();
        extra.push(0xc0);
        assert!(decode_reply(&extra).is_err(), "trailing bytes decoded");
    }
}

type Corrupt = Box<dyn Fn(&mut ContactResult)>;

#[test]
fn a_contact_readout_that_cannot_be_true_is_refused() {
    let mut bytes = Vec::new();
    let cases: Vec<(&str, Corrupt)> = vec![
        ("a zero threshold", Box::new(|r| r.sustained_nm[1] = 0.0)),
        ("a negative threshold", Box::new(|r| r.impact_nm[4] = -0.3)),
        (
            "a NaN threshold",
            Box::new(|r| r.sustained_nm[0] = f64::NAN),
        ),
        (
            "an infinite threshold",
            Box::new(|r| r.impact_nm[0] = f64::INFINITY),
        ),
        ("a NaN signal", Box::new(|r| r.signal[3] = f64::NAN)),
        (
            "an infinite impact",
            Box::new(|r| r.impact[2] = f64::NEG_INFINITY),
        ),
        ("a negative peak", Box::new(|r| r.peak_signal[2] = -0.1)),
        ("a NaN peak", Box::new(|r| r.peak_impact[5] = f64::NAN)),
        ("a NaN angle", Box::new(|r| r.angles[1] = f64::NAN)),
        (
            "an infinite gravity torque",
            Box::new(|r| r.gravity_nm[1] = f64::INFINITY),
        ),
        (
            "a trip on a joint the arm does not have",
            Box::new(|r| r.last_trip = Some((6, ContactChannel::Sustained))),
        ),
        ("a last trip with none counted", Box::new(|r| r.trips = 0)),
        (
            "trips counted with no last trip",
            Box::new(|r| r.last_trip = None),
        ),
    ];
    for (what, corrupt) in cases {
        let mut result = readout();
        corrupt(&mut result);
        encode_reply(&reply(result), &mut bytes);
        assert!(decode_reply(&bytes).is_err(), "{what} decoded");
    }
}

/// The limits a client sets cross the wire whole, and nothing that could
/// not be a threshold — a negative one, a zero, a NaN, a reaction no arm
/// has, a truncated frame — decodes into a command.
#[test]
fn contact_limits_survive_the_wire_and_hostile_ones_are_refused() {
    use par6_proto::command::SetContactLimits;
    let limits = |reaction| SetContactLimits {
        reaction,
        sustained_nm: [1.2, 2.0, 1.7, 0.5, 0.5, 0.3],
        impact_nm: [1.1, 1.0, 0.9, 0.3, 0.3, 0.2],
    };
    let mut bytes = Vec::new();
    for reaction in [
        ContactReaction::Stop,
        ContactReaction::Warn,
        ContactReaction::Off,
    ] {
        let command = Command::SetContactLimits(limits(reaction));
        encode_command(&command, 41, &mut bytes).unwrap();
        assert_eq!(decode_command(&bytes).unwrap(), (41, command));
        for end in 0..bytes.len() {
            assert!(
                decode_command(&bytes[..end]).is_err(),
                "a frame cut at {end} of {} decoded",
                bytes.len()
            );
        }
    }

    for bad in [0.0, -0.5, f64::NAN, f64::INFINITY] {
        for channel in 0..2 {
            let mut hostile = limits(ContactReaction::Stop);
            let thresholds = if channel == 0 {
                &mut hostile.sustained_nm
            } else {
                &mut hostile.impact_nm
            };
            thresholds[3] = bad;
            assert!(
                encode_command(&Command::SetContactLimits(hostile), 1, &mut bytes).is_err(),
                "a threshold of {bad} on channel {channel} was encoded"
            );
        }
    }

    // The same refusals on the way in: a frame built by hand, so the
    // decoder is what says no.
    encode_command(
        &Command::SetContactLimits(limits(ContactReaction::Stop)),
        1,
        &mut bytes,
    )
    .unwrap();
    let reaction_at = 3;
    assert_eq!(bytes[reaction_at], ContactReaction::Stop as u8);
    let mut unknown = bytes.clone();
    unknown[reaction_at] = 9;
    assert!(decode_command(&unknown).is_err(), "reaction 9 decoded");
    // The first sustained threshold: an f64 marker and eight bytes, after
    // the reaction and the array header.
    let first = reaction_at + 2;
    assert_eq!(bytes[first], 0xcb, "a float64 where the first threshold is");
    for bad in [-1.0_f64, 0.0, f64::NAN] {
        let mut hostile = bytes.clone();
        hostile[first + 1..first + 9].copy_from_slice(&bad.to_be_bytes());
        assert!(
            decode_command(&hostile).is_err(),
            "a decoded threshold of {bad} was accepted"
        );
    }
}
