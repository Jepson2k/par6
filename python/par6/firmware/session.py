"""Holding the bus while a drive is flashed.

Flashing is the one operation that needs the CAN bus to itself: par6d
talks to every drive every tick, and a bootloader that hears a torque
command mid-page discards the page. ``enter_flashing`` exists for exactly
this — it takes the runtime bus-silent without shutting it down — and this
module is the bracket that pairs it with the socket, so neither can be
left held.

The lock is a file lock rather than a process-local one because the two
things that flash — the CLI and the panel — are different processes on
the same control box.
"""

from __future__ import annotations

import errno
import fcntl
import logging
import os
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path
from typing import Any

from par6.config import DEFAULT_CAN_INTERFACE, can_interface

logger = logging.getLogger(__name__)

#: Somewhere every process on the control box can reach, cleared by a
#: reboot — a stale lock outliving a crashed flasher is worse than no lock.
LOCK_PATH = Path(os.environ.get("PAR6_FLASH_LOCK", "/dev/shm/par6-flash.lock"))


class FlashBusy(RuntimeError):
    """Another flasher holds the bus."""


@contextmanager
def flash_lock(path: Path = LOCK_PATH) -> Iterator[None]:
    """Exclusive right to flash on this machine, or refuse.

    Two flashers on one bus interleave their page streams and both fail,
    with the second one's failure looking like a hardware fault. Refusing
    up front is the difference between a clear message and a bench
    session.
    """
    try:
        handle = open(path, "w")
    except OSError as err:
        raise FlashBusy(f"cannot take the flash lock at {path}: {err}") from err
    try:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as err:
            if err.errno not in (errno.EACCES, errno.EAGAIN):
                raise
            raise FlashBusy(
                "another flash is already running on this machine "
                f"(lock held at {path})"
            ) from err
        handle.write(f"{os.getpid()}\n")
        handle.flush()
        yield
    finally:
        handle.close()


def bus_interface(client: Any = None, override: str | None = None) -> str:
    """The SocketCAN interface the arm is on.

    Asking the runtime beats guessing: it is the process actually bound
    to the interface, and a control box with two of them is exactly where
    a guess flashes the wrong arm. A caller that already holds the
    runtime's config passes it as *override* rather than paying for the
    bundle a second time.
    """
    if override:
        return override
    if client is not None:
        try:
            bundle = client.config_bundle()
        except (OSError, RuntimeError, ValueError):
            logger.debug("the runtime did not name its bus; using the default")
        else:
            if isinstance(bundle, dict):
                return can_interface(bundle.get("robot_toml"))
    return DEFAULT_CAN_INTERFACE


@contextmanager
def granted_bus(
    client: Any,
    assertion: str = "parked",
    *,
    channel: str | None = None,
    bitrate: int | None = None,
) -> Iterator[Any]:
    """Take the bus from a live par6d, yield an open socket, give it back.

    The runtime stays up throughout; it simply stops transmitting. On the
    way out the bus wakes, stored drive config is re-pushed and homing is
    invalidated if firmware actually changed — which is why exiting
    matters even when the flash failed, and why it runs in ``finally``.

    That wake-up is also why a flash must not return the moment it
    commits: a freshly committed drive boots its image only after the bus
    falls silent, and par6d's resumed traffic restarts that timer. The
    silence is held inside the block, by
    :func:`par6.firmware.flasher.wait_for_application`.
    """
    import can

    interface = bus_interface(client, channel)
    entered = False
    bus = None
    try:
        client.enter_flashing(assertion)
        entered = True
        kwargs: dict[str, Any] = {"interface": "socketcan", "channel": interface}
        if bitrate is not None:
            kwargs["bitrate"] = bitrate
        bus = can.Bus(**kwargs)
        yield bus
    finally:
        if bus is not None:
            try:
                bus.shutdown()
            except Exception:
                logger.exception("closing the CAN socket failed")
        if entered:
            try:
                client.exit_flashing()
            except Exception:
                logger.exception(
                    "exit_flashing() failed; the runtime is still bus-silent "
                    "and must be released before the arm can move"
                )
