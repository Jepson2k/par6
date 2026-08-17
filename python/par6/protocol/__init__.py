"""Protocol v2 wire layer.

`constants.py` is GENERATED from the Rust `par6-proto` crate
(`cargo run -p par6-proto --bin gen_python`) — do not edit by hand;
`cargo test -p par6-proto` fails if it is stale. Golden vectors under
`tests/golden/protocol` are the cross-language conformance suite.

`ActionState` and `ToolState` are the exceptions to "the public name is
the generated one": a decoded :class:`StatusBuffer` is handed to waldoctl
consumers, which compare those fields by identity against
`waldoctl.ActionState` / `waldoctl.ToolState`, and two `IntEnum`s with
equal values are still different classes — `is` would be false for every
member. So the public exports are waldoctl's, and `test_protocol_golden`
pins the generated members to them.
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
    ProtocolError,
    Reassembler,
    StatusBuffer,
    ToolStatusWire,
    decode_chunk,
    decode_command,
    decode_reply,
    decode_status_bin_into,
    encode_chunk,
    encode_command,
    encode_wire,
    split_into_chunks,
)

__all__ = [
    "constants",
    "wire",
    # constants
    "COMMAND_CLASS",
    "EN_SLOTS",
    "IO_SLOTS",
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
    "ProtocolError",
    "Reassembler",
    "StatusBuffer",
    "ToolStatusWire",
    "decode_chunk",
    "decode_command",
    "decode_reply",
    "decode_status_bin_into",
    "encode_chunk",
    "encode_command",
    "encode_wire",
    "split_into_chunks",
]
