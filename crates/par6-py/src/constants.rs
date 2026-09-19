//! Protocol constants, exposed to Python straight off `par6_proto`.
//!
//! These were previously transcribed into a generated `constants.py` that a
//! byte-for-byte test policed for staleness. The extension is a hard
//! dependency of the Python package — nothing can import `par6` without it —
//! so the copy bought nothing that this does not, and cost a file that had to
//! be regenerated and a test whose only job was to catch you forgetting.
//!
//! The enums are real Python `IntEnum`s, built at module init from each
//! wire enum's `variants()`. That is the same reflection the generator used,
//! so members still cannot drift from the Rust definitions — there is simply
//! no intermediate artifact to drift *in*.
//!
//! `ActionState` and `ToolState` are deliberately absent. A filled
//! `StatusBuffer` is handed to waldoctl consumers that compare those fields
//! by identity against `waldoctl.ActionState` / `waldoctl.ToolState`, and two
//! `IntEnum`s with equal values are still different classes — `is` would be
//! false for every member. The Python package re-exports waldoctl's.

use pyo3::prelude::*;
use pyo3::types::{PyList, PyModule};

use par6_proto::{
    CompletionPolicy, ControllerMode, ErrorCode, Frame, HomingJointState, HomingPhase, LinkState,
    EN_SLOTS, IO_SLOTS, MAX_IO_SLOTS, NUM_JOINTS, POSE_ELEMS, PROTO_VERSION, STATUS_HEADER_LEN,
    STATUS_LEN,
};

/// `MoveJPose` -> `MOVE_J_POSE`: the Rust variant name in Python's casing.
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

/// Build `enum.IntEnum(name, [(MEMBER, value), ...])` and add it to the module.
///
/// `__module__` is set so the class is findable at `par6._par6.<name>`, which
/// is what lets it pickle — a previewed script runs in a subprocess and its
/// arguments cross a process boundary.
fn add_int_enum(
    py: Python<'_>,
    m: &Bound<'_, PyModule>,
    name: &str,
    doc: &str,
    variants: &[(&str, i64)],
) -> PyResult<()> {
    let members = PyList::new(
        py,
        variants
            .iter()
            .map(|(vname, value)| (upper_snake(vname), *value)),
    )?;
    let cls = py
        .import("enum")?
        .getattr("IntEnum")?
        .call1((name, members))?;
    cls.setattr("__module__", "par6._par6")?;
    cls.setattr("__doc__", doc)?;
    m.add(name, cls)
}

/// Register the wire scalars and enums on the extension module.
pub fn register(py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("PROTO_VERSION", PROTO_VERSION)?;
    m.add("NUM_JOINTS", NUM_JOINTS)?;
    m.add("POSE_ELEMS", POSE_ELEMS)?;
    m.add("IO_SLOTS", IO_SLOTS)?;
    m.add("MAX_IO_SLOTS", MAX_IO_SLOTS)?;
    m.add("EN_SLOTS", EN_SLOTS)?;
    m.add("STATUS_LEN", STATUS_LEN)?;
    m.add("STATUS_HEADER_LEN", STATUS_HEADER_LEN)?;

    // Only the enums Python actually reads. `MsgType`, `CmdType`, `QueryType`,
    // `CommandClass` and `FlashingAssertion` were generated and exported for
    // nobody: the Rust codec owns framing, so Python never sees a message tag,
    // and `enter_flashing` takes a plain string.
    add_int_enum(
        py,
        m,
        "ErrorCode",
        "Runtime refusal codes. Backend vocabulary: waldoctl's RobotError \
         carries `code` as a plain int because a par6 code and another \
         backend's identically-numbered code are unrelated conditions.",
        ErrorCode::variants(),
    )?;
    add_int_enum(
        py,
        m,
        "Frame",
        "Reference frame a Cartesian target is expressed in.",
        Frame::variants(),
    )?;
    add_int_enum(
        py,
        m,
        "ControllerMode",
        "Runtime mode reported on STATUS.",
        ControllerMode::variants(),
    )?;
    add_int_enum(
        py,
        m,
        "CompletionPolicy",
        "When a queued motion is reported complete.",
        CompletionPolicy::variants(),
    )?;
    add_int_enum(
        py,
        m,
        "LinkState",
        "Motor-bus link state.",
        LinkState::variants(),
    )?;
    add_int_enum(
        py,
        m,
        "HomingJointState",
        "Per-joint homing reference state.",
        HomingJointState::variants(),
    )?;
    add_int_enum(
        py,
        m,
        "HomingPhase",
        "Phase of the homing sequence.",
        HomingPhase::variants(),
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::upper_snake;

    #[test]
    fn upper_snake_splits_camel_case() {
        assert_eq!(upper_snake("MoveJPose"), "MOVE_J_POSE");
        assert_eq!(upper_snake("Ok"), "OK");
        assert_eq!(upper_snake("ResetLoopStats"), "RESET_LOOP_STATS");
        assert_eq!(upper_snake("IkTargetUnreachable"), "IK_TARGET_UNREACHABLE");
    }
}
