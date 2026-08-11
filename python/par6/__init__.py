"""par6 — waldoctl backend for the PAR6 arm, client of the par6d Rust runtime.

Public surface: :class:`AsyncRobotClient` (async UDP client),
:class:`RobotClient` (sync facade), :class:`RobotError`, and the protocol v2
wire layer re-exported from :mod:`par6.protocol`.  The ``Robot`` factory
lands with workstream P1.H.
"""

from importlib.metadata import PackageNotFoundError, version

from .client import (
    AsyncRobotClient,
    LoopStatsResult,
    QueueResult,
    ReachableResult,
    RobotClient,
    RobotError,
    StatusResult,
    copy_status,
)
from .protocol import (
    ActionState,
    CmdType,
    CommandClass,
    CompletionPolicy,
    ErrorCode,
    Frame,
    MsgType,
    ProtocolError,
    QueryType,
    StatusBuffer,
    ToolState,
    ToolStatusWire,
    decode_status_bin_into,
)

try:
    __version__ = version("par6")
except PackageNotFoundError:  # running from a source tree
    __version__ = "0.0.0.dev0"

__all__ = [
    # client
    "AsyncRobotClient",
    "RobotClient",
    "RobotError",
    "LoopStatsResult",
    "QueueResult",
    "ReachableResult",
    "StatusResult",
    "copy_status",
    # protocol
    "ActionState",
    "CmdType",
    "CommandClass",
    "CompletionPolicy",
    "ErrorCode",
    "Frame",
    "MsgType",
    "ProtocolError",
    "QueryType",
    "StatusBuffer",
    "ToolState",
    "ToolStatusWire",
    "decode_status_bin_into",
    "__version__",
]
