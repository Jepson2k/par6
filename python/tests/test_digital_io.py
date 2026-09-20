"""Native digital I/O readback and bounded missing-peer requests."""

import json
import socket
import time

import pytest
from live_daemon import requires_par6d
from waldoctl.skills import skill

from par6.client import AsyncRobotClient


async def test_missing_peer_deadlines_preserve_skill_timeout_outcomes():
    @skill(id="test.io_deadline", version="1.0.0")
    async def query(rbt):
        return await rbt.io(timeout=0.05)

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as silent:
        silent.bind(("127.0.0.1", 0))
        async with AsyncRobotClient(
            port=silent.getsockname()[1], timeout=5, retries=3
        ) as absent:
            start = time.monotonic()
            assert await query.async_call(absent) is None
            assert time.monotonic() - start < 1.0, "query ignored its per-call deadline"
            start = time.monotonic()
            with pytest.raises(TimeoutError):
                await absent.write_io(0, 1, timeout=0.05)
            assert time.monotonic() - start < 1.0, "write ignored its per-call deadline"
            for invalid in (0, -1, float("nan"), float("inf"), True):
                with pytest.raises(ValueError):
                    await absent.io(timeout=invalid)
                with pytest.raises(ValueError):
                    await absent.write_io(0, 1, timeout=invalid)


@pytest.mark.e2e
@requires_par6d
async def test_output_write_readback_and_native_preview(daemon):
    from par6 import config
    from par6.client.dry_run_client import DryRunRobotClient

    offset = len(config.io_line_names()[0])
    preview = DryRunRobotClient(config_path=str(daemon.config))
    async with daemon.client() as client:
        before = await client.io(timeout=2)
        assert before is not None
        assert json.loads(json.dumps(before)) == preview.io()
        try:
            assert await client.write_io(0, 1, timeout=2) >= 0
            assert await client.wait_status(lambda s: s.io[offset] == 1, timeout=2)
            actual = await client.io(timeout=2)
            assert actual is not None and actual[offset] == 1
            preview.write_io(0, 1, timeout=2)
            assert preview.io(timeout=2)[offset] == actual[offset]
        finally:
            await client.write_io(0, before[offset], timeout=2)
