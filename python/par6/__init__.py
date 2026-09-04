"""par6 — waldoctl backend for the PAR6 arm, client of the par6d Rust runtime.

Public surface: :class:`Robot` (the waldoctl backend entry point),
:class:`AsyncRobotClient` (async UDP client), :class:`RobotClient` (sync
facade), :class:`RobotError`, and the protocol v2 wire layer re-exported
from :mod:`par6.protocol`.
"""

from importlib.metadata import PackageNotFoundError, version
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .robot import Robot

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
    QueryType,
    StatusBuffer,
    ToolState,
    ToolStatusWire,
)

try:
    __version__ = version("par6")
except PackageNotFoundError:  # running from a source tree
    __version__ = "0.0.0.dev0"


def __getattr__(name: str) -> object:
    # Lazy: Robot loads the kinematic model and the tool registry, which
    # pure protocol users of this package never need at import time.
    if name == "Robot":
        from .robot import Robot

        return Robot
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "Robot",
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
    "QueryType",
    "StatusBuffer",
    "ToolState",
    "ToolStatusWire",
    "__version__",
]
