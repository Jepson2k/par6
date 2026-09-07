"""Execution control through the native client and a real simulated daemon."""

import asyncio
import math
import time

import numpy as np
import pytest
from live_daemon import LiveDaemon, requires_par6d, settle_at

from par6 import config

pytestmark = [pytest.mark.e2e, requires_par6d]


@pytest.mark.timeout(90)
async def test_explicit_pause_preserves_queue_speed_and_standalone_deadlines(
    daemon: LiveDaemon,
):
    start = np.degrees(config.homing_ready_pose_rad()).tolist()
    async with daemon.client() as client:
        await settle_at(client, start)
        for invalid in [0, -1, 0.09, 1.01, 2, True, math.nan, math.inf, -math.inf]:
            with pytest.raises(ValueError):
                await client.set_execution_speed(invalid)

        assert await client.pause() == 1
        target = list(start)
        target[0] += 4
        index = await client.move_j(target, duration=1.0, wait=False)
        before = time.monotonic()
        assert not await client.wait_command(index, timeout=0.3)
        assert time.monotonic() - before < 1.0
        assert await client.set_execution_speed(0.5) == 1
        state = await client.execution_speed()
        assert state.paused and state.resume_scale == 0.5
        assert not await client.wait_command(index, timeout=0.3)
        assert await client.resume() == 1
        assert await client.wait_command(index, timeout=10)

        target[0] += 8
        index = await client.move_j(target, duration=2.0, wait=False)
        assert await client.wait_status(
            lambda s: s.angles[0] > start[0] + 5, timeout=10
        )
        assert await client.pause() == 1
        async with asyncio.timeout(5):
            while not (await client.execution_speed()).paused:
                await asyncio.sleep(0.02)
        assert not await client.wait_command(index, timeout=0.3)
        assert await client.set_execution_speed(0.6) == 1
        assert (await client.execution_speed()).paused
        assert not await client.wait_command(index, timeout=0.3)
        assert await client.resume() == 1
        assert await client.wait_command(index, timeout=10)
        assert await client.wait_status(
            lambda s: abs(s.angles[0] - target[0]) < 0.5, timeout=5
        )

        # Dwell time runs at ordinary speed, but an explicit pause retains
        # its remaining budget even while status and query traffic continue.
        index = await client.delay(1.0)
        async with asyncio.timeout(5):
            while (
                queue := await client.queue_state()
            ) is None or queue.executing_index != index:
                await asyncio.sleep(0.02)
        assert await client.pause() == 1
        assert not await client.wait_command(index, timeout=1.3)
        assert await client.ping() is not None
        assert await client.resume() == 1
        assert await client.wait_command(index, timeout=3)

        assert await client.pause() == 1
        cancelled = await client.move_j(start, duration=2.0, wait=False)
        with pytest.raises(TimeoutError):
            await client.move_j(start, duration=2.0, wait=True, timeout=0.2)
        assert await client.stop() == 1
        assert (await client.execution_speed()).paused
        assert await client.resume() == 1
        queue = await client.queue_state()
        assert (
            queue is not None and queue.executing_index != cancelled and not queue.queue
        )
