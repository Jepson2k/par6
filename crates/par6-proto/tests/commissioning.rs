//! The two wire additions a provisioned gripper drive needs — SET_TOOL_ID
//! and the bus-scan row's tool id — encode and decode back whole, and a
//! tool id that does not fit the drive's byte is refused at decode.

use par6_proto::command::SetToolId;
use par6_proto::{
    decode_command, decode_reply, encode_command, encode_reply, BusNode, CmdType, Command,
    QueryResult, Reply,
};

#[test]
fn set_tool_id_and_the_bus_scan_tool_id_survive_the_wire() {
    let cmd = Command::SetToolId(SetToolId {
        node: 6,
        tool_id: 13,
        force: true,
    });
    let mut buf = Vec::new();
    encode_command(&cmd, 7, &mut buf).unwrap();
    assert_eq!(decode_command(&buf).unwrap(), (7, cmd));

    let reply = Reply::Response {
        req_id: 7,
        result: QueryResult::BusScan {
            nodes: vec![BusNode {
                node: 6,
                configured: true,
                present: true,
                freshness: 1,
                hw_ver: 1,
                sw_ver: 8,
                serial: 1,
                tool_id: 13,
            }],
        },
    };
    let mut buf = Vec::new();
    encode_reply(&reply, &mut buf);
    assert_eq!(decode_reply(&buf).unwrap(), reply);
}

#[test]
fn a_tool_id_past_a_byte_is_refused() {
    // [SET_TOOL_ID, req_id 1, node 6, tool_id 256 (uint16), force false]
    let buf = [
        0x95,
        CmdType::SetToolId as u8,
        0x01,
        0x06,
        0xcd,
        0x01,
        0x00,
        0xc2,
    ];
    let err = decode_command(&buf).expect_err("256 is no tool id");
    assert!(err.to_string().contains("set_tool_id.tool_id"), "{err}");
}
