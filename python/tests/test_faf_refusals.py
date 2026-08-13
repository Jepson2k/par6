"""End-to-end: refused fire-and-forget commands reach the caller (issue #23).

Before the fix, an out-of-range ``teleport`` and a multi-axis ``jog_j`` both
"succeeded" at the client while the arm stood still: the runtime answered a
real ERROR datagram, but nothing awaits a fire-and-forget reply, so the
refusal evaporated — ``error()`` stayed ``None`` and STATUS carried nothing.
The runtime now latches such a refusal as the standing error (while the
pipeline is idle), so it surfaces through the ERROR query and the STATUS
broadcast, and the next accepted motion command clears it.

Everything here drives a real ``par6d --sim`` over real UDP with the real
client — no fakes, no scripted peer.  These tests fail against the pre-fix
runtime: ``error()`` then answers ``None`` after both refusals.
"""

from __future__ import annotations

import asyncio
import math
import time

import pytest
from live_daemon import LiveDaemon, requires_par6d, settle_at

from par6 import config as _cfg
from par6.client import AsyncRobotClient, RobotError
from par6.protocol.constants import ErrorCode

pytestmark = [pytest.mark.e2e, requires_par6d]

#: Wall-clock ceiling for one session step (boot, settle, a refusal landing).
STEP_BUDGET_S = 20.0


@pytest.fixture
def daemon(tmp_path):
    """A fresh ``par6d --sim`` process on ephemeral ports."""
    live = LiveDaemon.start(tmp_path)
    yield live
    live.stop()


def park_deg() -> list[float]:
    """The config park pose in wire units — inside every travel window."""
    return [
        math.degrees(v) for v in _cfg.load_robot_config()["robot"]["park_pose_rad"]
    ]


def max_abs_delta(actual, expected) -> float:
    return max(abs(a - b) for a, b in zip(actual, expected))


async def standing_error(
    client: AsyncRobotClient, budget_s: float = STEP_BUDGET_S
) -> RobotError | None:
    """Poll ``error()`` until a standing error appears, or the budget ends."""
    deadline = time.monotonic() + budget_s
    while time.monotonic() < deadline:
        err = await client.error()
        if err is not None:
            return err
        await asyncio.sleep(0.05)
    return None


async def error_clears(
    client: AsyncRobotClient, budget_s: float = STEP_BUDGET_S
) -> bool:
    """Poll ``error()`` until it answers ``None``, or the budget ends."""
    deadline = time.monotonic() + budget_s
    while time.monotonic() < deadline:
        if await client.error() is None:
            return True
        await asyncio.sleep(0.05)
    return False


@pytest.mark.timeout(120)
async def test_rejected_teleport_and_jog_surface_as_errors(daemon: LiveDaemon):
    """The issue's two reproductions, against the live runtime.

    A teleport outside the joint travel window and a multi-axis ``jog_j``
    are both refused server-side; the refusal must reach a caller that
    never awaits a reply — through ``error()`` AND through the STATUS
    broadcast (the surface Waldo Commander renders) — while the arm stays
    exactly where it was.  An accepted motion command then clears it.
    """
    park = park_deg()
    async with daemon.client() as client:
        # A healthy, idle, homed, enabled arm with no standing error.
        await settle_at(client, park)
        assert await client.error() is None

        # -- out-of-range teleport ----------------------------------------
        bad = list(park)
        bad[0] = 1.0e5  # outside any joint's travel window
        assert await client.teleport(bad) == 1  # fire-and-forget send "succeeds"

        err = await standing_error(client)
        assert err is not None, (
            "a refused teleport must surface through error(); "
            f"daemon log:\n{daemon.log()}"
        )
        assert err.code == ErrorCode.COMM_VALIDATION_ERROR, str(err)
        assert "angles[0]" in err.cause, str(err)
        assert err.command_index == -1, "a refusal is not attributable to a queue index"

        # The broadcast carries the same refusal — what a UI banner shows.
        assert await client.wait_status(
            lambda s: s.error is not None
            and s.error[1] == ErrorCode.COMM_VALIDATION_ERROR,
            timeout=STEP_BUDGET_S,
        ), "the refusal never reached the STATUS broadcast"

        # The arm did not move.
        angles = await client.angles()
        assert angles is not None
        assert max_abs_delta(angles, park) < 1.0, (
            f"a REFUSED teleport must not move the arm: {angles} vs {park}"
        )

        # An accepted motion command clears the refusal, like any
        # standing error.
        await client.teleport(park)
        assert await error_clears(client), "acceptance must clear the refusal"

        # -- multi-axis jog_j ---------------------------------------------
        before = await client.angles()
        assert before is not None
        assert await client.jog_j(joints=[0, 1], speeds=[0.2, -0.2], duration=0.4) == 1

        err = await standing_error(client)
        assert err is not None, (
            "a refused multi-axis jog must surface through error(); "
            f"daemon log:\n{daemon.log()}"
        )
        assert err.code == ErrorCode.COMM_VALIDATION_ERROR, str(err)
        assert "jog_j" in err.cause, str(err)

        # Still unmoved.
        angles = await client.angles()
        assert angles is not None
        assert max_abs_delta(angles, before) < 1.0, (
            f"a REFUSED jog must not move the arm: {angles} vs {before}"
        )

        await client.teleport(park)
        assert await error_clears(client), "acceptance must clear the refusal"


@pytest.mark.timeout(120)
async def test_healthy_jog_stream_is_not_serialized(daemon: LiveDaemon):
    """The fix must not buy visibility with round-trips: an accepted jog
    stream stays fire-and-forget.

    A burst of 100 ``jog_j`` calls must return in well under a second —
    a client that awaited a reply (or a reply timeout) per datagram would
    take at least 100 round-trips.  The same burst is also a real stream:
    it physically drives the sim arm, and leaves no standing error behind.
    """
    park = park_deg()
    async with daemon.client() as client:
        await settle_at(client, park)
        assert await client.error() is None
        start = (await client.angles())[0]

        t0 = time.monotonic()
        for _ in range(100):
            assert await client.jog_j(0, 0.3, duration=0.5) == 1
        elapsed = time.monotonic() - t0
        assert elapsed < 1.0, (
            f"100 fire-and-forget jogs took {elapsed:.2f}s — the stream is "
            f"being serialized on replies"
        )

        # The stream was accepted and drives the arm for real.
        assert await client.wait_status(
            lambda s: s.angles[0] > start + 1.0, timeout=STEP_BUDGET_S
        ), f"the jog stream never drove the arm; daemon log:\n{daemon.log()}"

        # The duration watchdog self-terminates the jog...
        assert await client.wait_status(
            lambda s: max(abs(v) for v in s.speeds) < 0.05, timeout=STEP_BUDGET_S
        ), "the jog never settled after its watchdog window"

        # ...and a healthy stream leaves no standing error behind.
        assert await client.error() is None, f"daemon log:\n{daemon.log()}"
