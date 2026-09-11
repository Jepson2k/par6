//! Commands arriving as dicts, the way the Python preview submits them.
//!
//! The preview builds a dict per command and hands it to `Preview::submit`,
//! which deserializes it through the same `Deserialize` the runtime uses.
//! An idempotency key is the client's retry token on the wire and means
//! nothing to a preview, so every queued command must deserialize without
//! one — a single command that demands it refuses a program that previews
//! fine everywhere else.

use par6_proto::Command;

/// Every queued command, as the minimum dict a preview sends: no `key`.
fn keyless_dicts() -> Vec<(&'static str, serde_json::Value)> {
    let pose = serde_json::json!([250.0, 0.0, 180.0, 0.0, 90.0, 0.0]);
    let angles = serde_json::json!([0.0, -90.0, 170.0, 0.0, -20.0, 180.0]);
    vec![
        ("home", serde_json::json!({"type": "home"})),
        (
            "move_j",
            serde_json::json!({"type": "move_j", "angles": angles, "speed": 0.3}),
        ),
        (
            "move_j_pose",
            serde_json::json!({"type": "move_j_pose", "pose": pose, "speed": 0.3}),
        ),
        (
            "move_l",
            serde_json::json!({"type": "move_l", "pose": pose, "frame": 0, "speed": 0.3}),
        ),
        (
            "move_c",
            serde_json::json!({
                "type": "move_c", "via": pose, "end": pose, "frame": 0, "speed": 0.3
            }),
        ),
        (
            "move_s",
            serde_json::json!({
                "type": "move_s", "waypoints": [pose], "frame": 0, "speed": 0.3
            }),
        ),
        (
            "move_p",
            serde_json::json!({
                "type": "move_p", "waypoints": [pose], "frame": 0, "speed": 0.3
            }),
        ),
        (
            "select_tool",
            serde_json::json!({"type": "select_tool", "tool_name": "Flange"}),
        ),
        (
            "set_tcp_offset",
            serde_json::json!({"type": "set_tcp_offset", "x": 0.0, "y": 0.0, "z": 10.0}),
        ),
        (
            "delay",
            serde_json::json!({"type": "delay", "seconds": 0.5}),
        ),
        (
            "checkpoint",
            serde_json::json!({"type": "checkpoint", "label": "mark"}),
        ),
        (
            "tool_action",
            serde_json::json!({
                "type": "tool_action", "tool_key": "Flange", "action": "open", "params": []
            }),
        ),
    ]
}

#[test]
fn a_queued_command_deserializes_without_an_idempotency_key() {
    for (name, dict) in keyless_dicts() {
        let parsed: Result<Command, _> = serde_json::from_value(dict.clone());
        let command = parsed.unwrap_or_else(|e| panic!("{name} without a key: {e}"));
        assert_eq!(
            command.idempotency_key(),
            Some(0),
            "{name}: the absent key reads as 0, the value that means 'none'"
        );
    }
}

#[test]
fn an_idempotency_key_in_the_dict_is_carried_through() {
    let dict =
        serde_json::json!({"type": "set_tcp_offset", "key": 7, "x": 0.0, "y": 0.0, "z": 10.0});
    let command: Command = serde_json::from_value(dict).expect("a keyed dict deserializes");
    assert_eq!(command.idempotency_key(), Some(7));
}
