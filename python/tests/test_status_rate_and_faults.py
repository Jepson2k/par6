"""STATUS rate and per-drive faults, against a live ``par6d --sim``.

The rate is a session knob: raising it is how a capture or a tuning run
gets resolution the default cannot, and the runtime is the only thing that
knows which rates it can serve. The faults are the other half of drive
health — the readings say a joint is climbing toward a limit, the labels
say which drive actually tripped and what its driver calls it.
"""

from __future__ import annotations

import asyncio
import time

import pytest
from live_daemon import LiveDaemon, requires_par6d

from par6.client import AsyncRobotClient, RobotError

pytestmark = requires_par6d


@pytest.fixture
def daemon(tmp_path):
    live = LiveDaemon.start(tmp_path)
    yield live
    live.stop()


async def _observed_hz(client: AsyncRobotClient, frames: int = 30) -> float:
    """The arrival rate over *frames* whole intervals.

    Timed from the first arrival, not from the call: the first frame lands
    somewhere inside a period, so counting it as an interval would report
    a rate that depends on when the measurement started.
    """
    seen = 0
    start = 0.0
    async for _ in client.stream_status():
        if seen == 0:
            start = time.perf_counter()
        seen += 1
        if seen > frames:
            break
    return frames / max(time.perf_counter() - start, 1e-9)


async def _rate_in_force(client: AsyncRobotClient, hz: float) -> None:
    """Set *hz* and wait for the runtime to say it is broadcasting at it.

    The readback is the barrier: it is a round trip through the same loop
    that owns the broadcast interval, so once it answers with the new rate
    the next frame is emitted at it. Sleeping instead would race the loop.
    """
    assert await client.set_status_rate(hz) > 0
    back = await client.status_rate()
    assert back is not None and back.hz == hz, (
        f"asked for {hz} Hz and the runtime reports {back and back.hz}"
    )


@pytest.mark.asyncio
async def test_the_runtime_serves_every_rate_it_says_it_can(daemon: LiveDaemon):
    """``control_hz`` is the whole contract: a caller derives the legal set
    from it instead of probing, so every rate it implies must be accepted
    and the current one must itself be legal."""
    async with daemon.client() as client:
        assert await client.wait_ready(timeout=10.0)

        rate = await client.status_rate()
        assert rate is not None
        assert rate.control_hz > 0.0 and rate.hz > 0.0
        assert rate.hz in rate.achievable(), (
            f"broadcasting at {rate.hz} Hz, which its own {rate.control_hz} Hz "
            f"tick rate cannot divide into"
        )

        for candidate in rate.achievable():
            assert await client.set_status_rate(candidate) > 0, (
                f"{candidate} Hz divides {rate.control_hz} Hz but was refused"
            )
        assert await client.set_status_rate(rate.hz) > 0


@pytest.mark.asyncio
async def test_raising_the_rate_actually_delivers_more_frames(daemon: LiveDaemon):
    """The knob exists for resolution, so the change has to show up in
    arrivals rather than only in the readback."""
    async with daemon.client() as client:
        assert await client.wait_ready(timeout=10.0)
        original = await client.status_rate()
        assert original is not None

        achievable = original.achievable()
        high, low = achievable[0], achievable[min(3, len(achievable) - 1)]
        assert high > low

        try:
            await _rate_in_force(client, low)
            slow = await _observed_hz(client)

            await _rate_in_force(client, high)
            fast = await _observed_hz(client)
        finally:
            await client.set_status_rate(original.hz)

        assert fast > slow * 1.5, (
            f"{low} Hz gave {slow:.1f}/s and {high} Hz gave {fast:.1f}/s"
        )


@pytest.mark.asyncio
async def test_an_unachievable_rate_is_refused_and_says_what_would_work(
    daemon: LiveDaemon,
):
    """Refused rather than rounded: a capture taken at a rate nobody asked
    for is wrong in a way nothing reports. The refusal has to carry the
    rates that would have worked, since that is the whole recovery.

    Both shapes of "not servable" go through it. The second is the one
    that hides: a rate the tick clock divides into a whole number of ticks
    but that is not itself a rate the broadcast can hold, so accepting it
    would serve — and report — a neighbouring one.
    """
    async with daemon.client() as client:
        assert await client.wait_ready(timeout=10.0)
        before = await client.status_rate()
        assert before is not None

        whole_ticks = next(
            before.control_hz / d
            for d in range(2, 10)
            if before.control_hz / d not in before.achievable()
        )
        for bogus in (before.control_hz / 3 + 0.5, whole_ticks):
            with pytest.raises(RobotError) as caught:
                await client.set_status_rate(bogus)
            message = str(caught.value).lower()
            assert "divide" in message, message
            assert str(int(before.achievable()[0])) in message, message

        after = await client.status_rate()
        assert after is not None and after.hz == before.hz, (
            "a refused rate must leave the broadcast alone"
        )


@pytest.mark.asyncio
async def test_drive_health_reports_one_fault_slot_per_node(daemon: LiveDaemon):
    """A healthy bus still reports a slot per drive, so "no faults" is
    visible as such rather than indistinguishable from a backend that does
    not report faults at all — which is what an empty list means."""
    async with daemon.client() as client:
        assert await client.wait_ready(timeout=10.0)

        seen: dict = {}

        async def collect() -> None:
            async for status in client.stream_status_shared():
                health = dict(getattr(status, "drive_health", {}) or {})
                if health.get("faults") is not None:
                    seen.update(health)
                    return

        try:
            await asyncio.wait_for(collect(), timeout=15.0)
        except asyncio.TimeoutError:
            pytest.fail("no drive health ever arrived on STATUS")

        faults = seen["faults"]
        assert len(faults) == len(seen["temperatures_c"]), (
            "faults and readings must line up node for node, or a display "
            f"cannot say which drive tripped: {len(faults)} vs "
            f"{len(seen['temperatures_c'])}"
        )
        assert all(isinstance(f, (list, tuple)) for f in faults), faults
        assert all(not f for f in faults), (
            f"the sim bus reports no faults, so every slot should be clear: {faults}"
        )
