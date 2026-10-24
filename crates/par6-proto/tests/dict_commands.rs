//! Commands arriving as dicts, the way the Python preview submits them.
//!
//! The preview builds a dict per command and hands it to `Preview::submit`,
//! which deserializes it through the same `Deserialize` the runtime uses.
//! An idempotency key is the client's retry token on the wire and means
//! nothing to a preview, so every queued command must deserialize without
//! one — a single command that demands it refuses a program that previews
//! fine everywhere else.

use par6_proto::{command_class, CmdType, Command, CommandClass};

/// The wire name of a command type: its variant in snake case.
fn wire_name(c: CmdType) -> String {
    let mut out = String::new();
    for ch in format!("{c:?}").chars() {
        if ch.is_ascii_uppercase() && !out.is_empty() {
            out.push('_');
        }
        out.push(ch.to_ascii_lowercase());
    }
    out
}

/// The minimum dict a preview sends for a queued command: no `key`.
fn keyless_dict(name: &str) -> Option<serde_json::Value> {
    let pose = serde_json::json!([250.0, 0.0, 180.0, 0.0, 90.0, 0.0]);
    let angles = serde_json::json!([0.0, -90.0, 170.0, 0.0, -20.0, 180.0]);
    Some(match name {
        "home" => serde_json::json!({"type": "home"}),
        "move_j" => serde_json::json!({"type": "move_j", "angles": angles, "speed": 0.3}),
        "move_j_pose" => serde_json::json!({"type": "move_j_pose", "pose": pose, "speed": 0.3}),
        "move_l" => {
            serde_json::json!({"type": "move_l", "pose": pose, "frame": 0, "speed": 0.3})
        }
        "move_c" => serde_json::json!({
            "type": "move_c", "via": pose, "end": pose, "frame": 0, "speed": 0.3
        }),
        "move_s" => serde_json::json!({
            "type": "move_s", "waypoints": [pose], "frame": 0, "speed": 0.3
        }),
        "move_p" => serde_json::json!({
            "type": "move_p", "waypoints": [pose], "frame": 0, "speed": 0.3
        }),
        "select_tool" => serde_json::json!({"type": "select_tool", "tool_name": "Flange"}),
        "set_tcp_offset" => {
            serde_json::json!({"type": "set_tcp_offset", "x": 0.0, "y": 0.0, "z": 10.0})
        }
        "set_tcp_transform" => serde_json::json!({
            "type": "set_tcp_transform", "x": 0.0, "y": 0.0, "z": 10.0,
            "roll": 0.0, "pitch": 0.0, "yaw": 5.0
        }),
        "write_io" => serde_json::json!({"type": "write_io", "port": 0, "value": 1}),
        "delay" => serde_json::json!({"type": "delay", "seconds": 0.5}),
        "checkpoint" => serde_json::json!({"type": "checkpoint", "label": "mark"}),
        "tool_action" => serde_json::json!({
            "type": "tool_action", "tool_key": "Flange", "action": "open", "params": []
        }),
        _ => return None,
    })
}

/// Every queued command — read from the command classes, so a new one
/// cannot be left out — deserializes without a key, the absent key
/// reading as 0 (the value that means "none"), and carries one through
/// when the dict has it.
#[test]
fn a_queued_command_deserializes_without_an_idempotency_key() {
    let queued: Vec<CmdType> = CmdType::ALL
        .iter()
        .copied()
        .filter(|c| command_class(*c) == CommandClass::Queued)
        .collect();
    assert!(queued.len() > 10, "the queued class: {queued:?}");
    for c in queued {
        let name = wire_name(c);
        let dict = keyless_dict(&name)
            .unwrap_or_else(|| panic!("no keyless dict for the queued command {name}"));
        let command: Command = serde_json::from_value(dict.clone())
            .unwrap_or_else(|e| panic!("{name} without a key: {e}"));
        assert_eq!(command.tag(), c, "{name} parsed as another command");
        assert_eq!(command.idempotency_key(), Some(0), "{name}");
        let mut keyed = dict;
        keyed["key"] = serde_json::json!(7);
        let command: Command =
            serde_json::from_value(keyed).unwrap_or_else(|e| panic!("{name} with a key: {e}"));
        assert_eq!(command.idempotency_key(), Some(7), "{name}");
    }
}
