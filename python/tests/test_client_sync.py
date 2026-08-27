"""Sync facade smoke: the background-loop RobotClient over a live runtime.

The facade runs its coroutines on the par6 module loop in a helper thread;
the runtime is a real ``par6d --sim`` over real UDP — no fakes, no
scripted peer.  The engine client underneath is the same one the async
tests drive; what is under test here is the synchronous surface.
"""

from __future__ import annotations

import asyncio
import math
import socket
import time

import pytest
from live_daemon import LiveDaemon, requires_par6d

from par6 import config as _cfg
from par6.client import RobotClient

pytestmark = [pytest.mark.e2e, requires_par6d]


@pytest.fixture
def daemon(tmp_path):
    live = LiveDaemon.start(tmp_path)
    yield live
    live.stop()


def park_deg() -> list[float]:
    return [math.degrees(v) for v in _cfg.load_robot_config()["robot"]["park_pose_rad"]]


def settle_at(
    client: RobotClient, angles_deg: list[float], budget_s: float = 20.0
) -> None:
    """Reset and re-send teleport until the broadcast shows the arm there —
    the sync twin of ``live_daemon.settle_at`` (teleport is unacked and
    gated on ENABLED, so one send can land before the RT clear settles)."""
    deadline = time.monotonic() + budget_s
    client.reset()
    while time.monotonic() < deadline:
        client.teleport(angles_deg)
        if client.wait_status(
            lambda s: (
                s.homed and all(abs(a - b) < 0.5 for a, b in zip(s.angles, angles_deg))
            ),
            timeout=0.5,
        ):
            return
        time.sleep(0.05)
    raise AssertionError(f"the sim arm never reached {angles_deg}")


def sync_client(daemon: LiveDaemon, **kwargs) -> RobotClient:
    kwargs.setdefault("timeout", 2.0)
    kwargs.setdefault("retries", 2)
    return RobotClient(
        host="127.0.0.1",
        port=daemon.command_port,
        status_transport="UNICAST",
        status_port=daemon.status_port,
        status_unicast_host="127.0.0.1",
        **kwargs,
    )


@pytest.mark.timeout(120)
def test_sync_facade_smoke(daemon):
    """Every leg a synchronous script stands on, in one session: query,
    blocking motion, jog, the two make-it-safe controls, the status stream,
    and a loud failure after close."""
    with sync_client(daemon) as client:
        assert client.wait_ready(timeout=10.0) is True
        ping = client.ping()
        assert ping is not None and ping.hardware_connected is False

        park = park_deg()
        settle_at(client, park)

        target = list(park)
        target[0] += 10.0
        index = client.move_j(target, speed=0.5, wait=True, timeout=30.0)
        assert index >= 0
        angles = client.angles()
        assert angles is not None and angles[0] == pytest.approx(target[0], abs=0.5)

        assert client.jog_j(1, 0.4, 0.2) == 1
        assert client.stop() == 1

        # Clearing a protective stop and floating the arm under G(q)
        # alone — the control pair a synchronous script needs.
        assert client.reset() == 1
        assert client.set_gravity_comp(True) == 1
        assert client.set_gravity_comp(False) == 1

        assert client.status_seq_gaps == 0

    # Context-manager exit closed the client; further use must fail loudly.
    with pytest.raises(RuntimeError):
        client.ping()


def test_sync_facade_refuses_use_inside_a_running_loop():
    client = RobotClient(host="127.0.0.1", port=1, timeout=0.5, retries=0)
    try:

        async def misuse():
            client.ping()

        with pytest.raises(RuntimeError, match="event loop is running"):
            asyncio.run(misuse())
    finally:
        client.close()


@pytest.mark.timeout(120)
def test_cli_reads_a_live_runtime_and_reports_an_unreachable_one(daemon, capsys):
    """The console script is the shell view of the sync client.

    Drive the real entry point against the live runtime, then against an
    address nothing answers — a CLI that exits 0 on an unreachable runtime
    is worse than no CLI.
    """
    from par6.cli import EXIT_UNREACHABLE, main

    with sync_client(daemon) as client:
        assert client.wait_ready(timeout=10.0) is True
        settle_at(client, park_deg())

    assert (
        main(["--host", "127.0.0.1", "--port", str(daemon.command_port), "angles"]) == 0
    )
    reported = [float(v) for v in capsys.readouterr().out.split()]
    assert reported == pytest.approx(park_deg(), abs=0.5)

    # Nothing is listening on a port we know is free.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        dead = int(probe.getsockname()[1])
    assert (
        main(["--host", "127.0.0.1", "--port", str(dead), "--timeout", "0.2", "angles"])
        == EXIT_UNREACHABLE
    )
    assert "did not answer" in capsys.readouterr().err


@pytest.mark.timeout(120)
def test_freedrive_reads_the_broadcast_not_the_last_command(daemon):
    """Freedrive is a state of the arm, not a flag the client remembers.

    par6 has no freedrive MODE: with the gravity feedforward applied, IDLE
    emits a torque-only G(q) hold with no position term, so the arm floats.
    That means the honest answer to "is it floating?" is the runtime's own
    gravity_applied() condition, read off STATUS — IDLE, homed, enabled,
    gravity on.  Gravity comp off, or an un-homed arm, is not floating no
    matter what was last requested.
    """
    with sync_client(daemon) as client:
        assert client.wait_ready(timeout=10.0) is True

        # Homed + IDLE + enabled + gravity on: floating.
        settle_at(client, park_deg())
        assert client.set_gravity_comp(True) == 1
        assert client.wait_status(lambda s: s.homed and s.gravity_comp, timeout=10.0)
        assert client.is_freedrive() is True

        # Gravity comp off: same arm, no longer back-driveable — the
        # broadcast, not the last command, is what answers.
        assert client.set_gravity_comp(False) == 1
        assert client.wait_status(lambda s: not s.gravity_comp, timeout=10.0)
        assert client.is_freedrive() is False
