"""End-to-end against a real ``par6d --sim``: a refused teleport answers in
its own reply, and an accepted jog stream stays fire-and-forget.

Everything here drives a real ``par6d --sim`` over real UDP with the real
client — no fakes, no scripted peer.
"""

from __future__ import annotations

import math
import time

import pytest
from live_daemon import LiveDaemon, angles_now, requires_par6d, settle_at

from par6 import config as _cfg
from par6.client import RobotError
from par6.protocol import ErrorCode

pytestmark = [pytest.mark.e2e, requires_par6d]

#: Wall-clock ceiling for one session step (boot, settle, a refusal landing).
STEP_BUDGET_S = 20.0


def park_deg() -> list[float]:
    """The config park pose in wire units — inside every travel window."""
    return [math.degrees(v) for v in _cfg.config().park_pose_rad()]


def max_abs_delta(actual, expected) -> float:
    return max(abs(a - b) for a, b in zip(actual, expected))


@pytest.mark.timeout(120)
async def test_rejected_teleport_is_refused_in_its_reply(daemon: LiveDaemon):
    """A teleport is acked: a refusal reaches the caller as the reply.

    A teleport outside the joint travel window is refused server-side, and
    the caller hears it as the structured error of the call itself — not
    as a standing error it would have to go and read — while the arm
    stays exactly where it was and the session's error surface stays
    clean.
    """
    park = park_deg()
    async with daemon.client() as client:
        # A healthy, idle, homed, enabled arm with no standing error.
        await settle_at(client, park)
        assert await client.error() is None

        bad = list(park)
        bad[0] = 1.0e5  # outside any joint's travel window
        with pytest.raises(RobotError) as refused:
            await client.teleport(bad)
        assert refused.value.code == ErrorCode.COMM_VALIDATION_ERROR, str(refused.value)
        assert "angles[0]" in refused.value.cause, str(refused.value)

        # The arm did not move, and nothing is left standing.
        angles = await client.angles()
        assert angles is not None
        assert max_abs_delta(angles, park) < 1.0, (
            f"a REFUSED teleport must not move the arm: {angles} vs {park}"
        )
        assert await client.error() is None
        assert await client.teleport(park) == 1


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
        start = (await angles_now(client))[0]

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
            lambda s: max(abs(v) for v in s.speeds) < 3.0, timeout=STEP_BUDGET_S
        ), "the jog never settled after its watchdog window"

        # ...and a healthy stream leaves no standing error behind.
        assert await client.error() is None, f"daemon log:\n{daemon.log()}"
