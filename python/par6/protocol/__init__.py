"""Protocol v2 layer — constants and the shared status buffer.

The wire codec lives in the Rust `par6-proto` crate, and so do the constants:
both are reached through the `par6._par6` extension module, which is a hard
dependency of this package. Nothing here restates a wire value.

`ActionState` and `ToolState` are the exceptions to "the name comes from the
engine": a filled :class:`StatusBuffer` is handed to waldoctl consumers, which
compare those fields by identity against `waldoctl.ActionState` /
`waldoctl.ToolState`, and two `IntEnum`s with equal values are still different
classes — `is` would be false for every member. So the public exports are
waldoctl's.
"""

from waldoctl import ActionState, ToolState

from par6._par6 import (
    EN_SLOTS,
    IO_SLOTS,
    MAX_IO_SLOTS,
    NUM_JOINTS,
    POSE_ELEMS,
    PROTO_VERSION,
    STATUS_HEADER_LEN,
    STATUS_LEN,
    CompletionPolicy,
    ControllerMode,
    ErrorCode,
    Frame,
    HomingJointState,
    HomingPhase,
    LinkState,
)

from . import wire
from .wire import (
    MAX_JOG_DURATION_S,
    StatusBuffer,
    ToolStatusWire,
    update_status_from_dict,
)

__all__ = [
    "wire",
    # constants, off the extension
    "EN_SLOTS",
    "IO_SLOTS",
    "MAX_IO_SLOTS",
    "MAX_JOG_DURATION_S",
    "NUM_JOINTS",
    "POSE_ELEMS",
    "PROTO_VERSION",
    "STATUS_HEADER_LEN",
    "STATUS_LEN",
    "ActionState",
    "CompletionPolicy",
    "ControllerMode",
    "ErrorCode",
    "Frame",
    "HomingJointState",
    "HomingPhase",
    "LinkState",
    "ToolState",
    # wire
    "StatusBuffer",
    "ToolStatusWire",
    "update_status_from_dict",
]
