//! Generator for the Python constants mirror
//! (`python/par6/protocol/constants.py`).
//!
//! Run `cargo run -p par6-proto --bin gen_python` to regenerate; a test in
//! this crate diffs the generator output against the committed file, so a
//! contract change that forgets to regenerate fails `cargo test -p par6-proto`.

use std::fmt::Write;

use crate::enums::{
    command_class, ActionState, CmdType, CommandClass, CompletionPolicy, Frame, MsgType, QueryType,
    ToolState,
};
use crate::error::ErrorCode;
use crate::status::{STATUS_HEADER_LEN, STATUS_LEN};
use crate::{EN_SLOTS, IO_SLOTS, MAX_IO_SLOTS, NUM_JOINTS, POSE_ELEMS, PROTO_VERSION};

/// `MoveJPose` → `MOVE_J_POSE`.
fn upper_snake(name: &str) -> String {
    let mut out = String::with_capacity(name.len() + 4);
    for (i, c) in name.chars().enumerate() {
        if c.is_ascii_uppercase() && i > 0 {
            out.push('_');
        }
        out.push(c.to_ascii_uppercase());
    }
    out
}

fn emit_enum(out: &mut String, name: &str, doc: &str, variants: &[(&str, i64)]) {
    writeln!(out, "\n\nclass {name}(IntEnum):").unwrap();
    writeln!(out, "    \"\"\"{doc}\"\"\"\n").unwrap();
    for (vname, value) in variants {
        writeln!(out, "    {} = {}", upper_snake(vname), value).unwrap();
    }
}

/// Render the full `constants.py` source.
pub fn generate() -> String {
    let mut out = String::with_capacity(8 * 1024);
    out.push_str(
        "\"\"\"Protocol v2 constants — GENERATED, DO NOT EDIT.\n\
         \n\
         Source of truth: the Rust `par6-proto` crate. Regenerate with\n\
         `cargo run -p par6-proto --bin gen_python > python/par6/protocol/constants.py`;\n\
         `cargo test -p par6-proto` fails if this file is stale.\n\
         \"\"\"\n\
         \n\
         from enum import IntEnum\n\
         \n\
         PROTO_VERSION = ",
    );
    writeln!(out, "{PROTO_VERSION}").unwrap();
    writeln!(out, "NUM_JOINTS = {NUM_JOINTS}").unwrap();
    writeln!(out, "POSE_ELEMS = {POSE_ELEMS}").unwrap();
    writeln!(out, "IO_SLOTS = {IO_SLOTS}").unwrap();
    writeln!(out, "MAX_IO_SLOTS = {MAX_IO_SLOTS}").unwrap();
    writeln!(out, "EN_SLOTS = {EN_SLOTS}").unwrap();
    writeln!(out, "STATUS_LEN = {STATUS_LEN}").unwrap();
    writeln!(out, "STATUS_HEADER_LEN = {STATUS_HEADER_LEN}").unwrap();

    emit_enum(
        &mut out,
        "MsgType",
        "Server->client message tags (slot 0).",
        MsgType::variants(),
    );
    emit_enum(
        &mut out,
        "CmdType",
        "Client->server command tags (slot 0).",
        CmdType::variants(),
    );
    emit_enum(
        &mut out,
        "QueryType",
        "Query result tags (slot 0 of the nested RESPONSE payload).",
        QueryType::variants(),
    );
    emit_enum(
        &mut out,
        "CommandClass",
        "Ack classes; see COMMAND_CLASS for the per-command table.",
        CommandClass::variants(),
    );
    emit_enum(
        &mut out,
        "ActionState",
        "State of the currently executing action.",
        ActionState::variants(),
    );
    emit_enum(
        &mut out,
        "CompletionPolicy",
        "Controller-side completion policy for queued motion.",
        CompletionPolicy::variants(),
    );
    emit_enum(
        &mut out,
        "Frame",
        "Cartesian reference frame.",
        Frame::variants(),
    );
    emit_enum(
        &mut out,
        "ToolState",
        "State of an end-of-arm tool.",
        ToolState::variants(),
    );
    emit_enum(
        &mut out,
        "ErrorCode",
        "Error codes in subsystem ranges of 10: IK 10-19, TRAJ 20-29, \
         MOTN 30-39, COMM 40-49, SYS 50-59.",
        ErrorCode::variants(),
    );

    out.push_str(
        "\n\n# The ack taxonomy: one table, both sides consult it.\n\
         COMMAND_CLASS: dict[CmdType, CommandClass] = {\n",
    );
    // ALL and variants() share declaration order by construction (wire_enum!).
    for (cmd, (name, _)) in CmdType::ALL.iter().zip(CmdType::variants()) {
        let class = command_class(*cmd);
        let class_name = CommandClass::variants()
            .iter()
            .find(|(_, v)| *v == class as i64)
            .expect("class present")
            .0;
        writeln!(
            out,
            "    CmdType.{}: CommandClass.{},",
            upper_snake(name),
            upper_snake(class_name)
        )
        .unwrap();
    }
    out.push_str("}\n");
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn upper_snake_splits_camel_case() {
        assert_eq!(upper_snake("MoveJPose"), "MOVE_J_POSE");
        assert_eq!(upper_snake("Ok"), "OK");
        assert_eq!(upper_snake("ResetLoopStats"), "RESET_LOOP_STATS");
        assert_eq!(upper_snake("IkTargetUnreachable"), "IK_TARGET_UNREACHABLE");
    }
}
