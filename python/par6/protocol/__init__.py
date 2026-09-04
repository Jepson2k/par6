"""Protocol v2 layer — constants and the shared status buffer.

The wire codec lives in the Rust `par6-proto` crate, reached through the
`par6._par6` extension module. `constants.py` is GENERATED from that
crate (`cargo run -p par6-proto --bin gen_python`) — do not edit by hand;
`cargo test -p par6-proto` fails if it is stale.

`ActionState` and `ToolState` are the exceptions to "the public name is
the generated one": a filled :class:`StatusBuffer` is handed to waldoctl
consumers, which compare those fields by identity against
`waldoctl.ActionState` / `waldoctl.ToolState`, and two `IntEnum`s with
equal values are still different classes — `is` would be false for every
member. So the public exports are waldoctl's.
"""

from waldoctl import ActionState, ToolState

from . import constants, wire
from .constants import (
    COMMAND_CLASS,
    EN_SLOTS,
    IO_SLOTS,
    NUM_JOINTS,
    POSE_ELEMS,
    PROTO_VERSION,
    STATUS_HEADER_LEN,
    STATUS_LEN,
    CmdType,
    CommandClass,
    CompletionPolicy,
    ErrorCode,
    Frame,
    MsgType,
    QueryType,
)
from .wire import (
    MAX_JOG_DURATION_S,
    StatusBuffer,
    ToolStatusWire,
    update_status_from_dict,
)

__all__ = [
    "constants",
    "wire",
    # constants
    "COMMAND_CLASS",
    "EN_SLOTS",
    "IO_SLOTS",
    "MAX_JOG_DURATION_S",
    "NUM_JOINTS",
    "POSE_ELEMS",
    "PROTO_VERSION",
    "STATUS_HEADER_LEN",
    "STATUS_LEN",
    "ActionState",
    "CmdType",
    "CommandClass",
    "CompletionPolicy",
    "ErrorCode",
    "Frame",
    "MsgType",
    "QueryType",
    "ToolState",
    # wire
    "StatusBuffer",
    "ToolStatusWire",
    "update_status_from_dict",
]
