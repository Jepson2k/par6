use par6_proto::command::SetShapes;
use par6_proto::{
    decode_command, decode_reply, encode_command, encode_reply, Attachment, Command, QueryResult,
    Reply, Shape,
};

fn part() -> Shape {
    Shape {
        kind: "sphere".into(),
        params: vec![0.025],
        pose: vec![0.0; 6],
        collision: true,
        margin: None,
        name: "part".into(),
        physics: None,
        attachment: Some(Attachment {
            epoch: u64::MAX,
            allowed_contacts: vec![],
        }),
    }
}

#[test]
fn attachment_context_and_contacts_survive_the_wire_and_reject_corruption() {
    let shape = part();
    let command = Command::SetShapes(SetShapes {
        shapes: vec![shape.clone()],
    });
    let mut bytes = Vec::new();
    encode_command(&command, 7, &mut bytes).unwrap();
    assert_eq!(decode_command(&bytes).unwrap(), (7, command));
    for end in 0..bytes.len() {
        assert!(decode_command(&bytes[..end]).is_err());
    }
    let start = bytes.len() - 10;
    assert_eq!(bytes[start], 0xcf);
    for epoch in [
        vec![0],
        vec![0xc0],
        vec![0xc2],
        vec![0xc3],
        vec![0xff],
        vec![0xa0],
        vec![0x90],
        [vec![0xcb], 1.0_f64.to_be_bytes().to_vec()].concat(),
    ] {
        let corrupt = [bytes[..start].to_vec(), epoch, vec![0x90]].concat();
        assert!(decode_command(&corrupt).is_err(), "{corrupt:?}");
    }
    for contacts in [
        vec!["*".into()],
        vec!["gripper".into(), "gripper".into()],
        vec!["shape:part".into()],
        vec!["".into()],
        vec!["x".repeat(129)],
        (0..33).map(|i| format!("shape:{i}")).collect(),
    ] {
        let mut invalid = part();
        invalid.attachment.as_mut().unwrap().allowed_contacts = contacts;
        assert!(encode_command(
            &Command::SetShapes(SetShapes {
                shapes: vec![invalid.clone()]
            }),
            7,
            &mut bytes
        )
        .is_err());
        let reply = Reply::Response {
            req_id: 7,
            result: QueryResult::Shapes {
                installation: vec![],
                program: vec![invalid],
                epoch: 3,
                attachment_epoch: u64::MAX,
            },
        };
        encode_reply(&reply, &mut bytes);
        assert!(decode_reply(&bytes).is_err());
    }
    let reply = Reply::Response {
        req_id: 7,
        result: QueryResult::Shapes {
            installation: vec![],
            program: vec![shape],
            epoch: 3,
            attachment_epoch: u64::MAX,
        },
    };
    encode_reply(&reply, &mut bytes);
    assert_eq!(decode_reply(&bytes).unwrap(), reply);
}
