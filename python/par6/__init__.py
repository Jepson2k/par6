"""par6 — waldoctl backend for the PAR6 arm, client of the par6d Rust runtime.

Scaffold: the public surface (Robot, AsyncRobotClient, RobotClient) lands
with workstreams G/H — see the repository README workstream board.
"""

from importlib.metadata import PackageNotFoundError, version

try:
    __version__ = version("par6")
except PackageNotFoundError:  # running from a source tree
    __version__ = "0.0.0.dev0"
