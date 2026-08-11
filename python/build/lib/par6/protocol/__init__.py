"""Protocol v2 wire layer.

`constants.py` is GENERATED from the Rust `par6-proto` crate
(`cargo run -p par6-proto --bin gen_python`) — do not edit by hand;
`cargo test -p par6-proto` fails if it is stale. Golden vectors under
`tests/golden/protocol` are the cross-language conformance suite.
"""

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
    ActionState,
    CmdType,
    CommandClass,
    CompletionPolicy,
    ErrorCode,
    Frame,
    MsgType,
    QueryType,
    ToolState,
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
