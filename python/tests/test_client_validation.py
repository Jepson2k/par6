"""Client-side checks that need no runtime: the calls a script gets wrong
are refused with the exception the rest of the API raises, and the API's
own sentinels never escape as extension errors."""

from __future__ import annotations

import asyncio
from typing import Any, cast

import pytest

from par6 import config as _cfg
from par6.client import AsyncRobotClient, RobotClient


def _dead_client() -> AsyncRobotClient:
    """A client bound to a port nothing answers, with the shortest retry."""
    return AsyncRobotClient(host="127.0.0.1", port=1, timeout=0.2, retries=0)


def test_jog_l_refuses_mismatched_axes_and_unknown_axes_with_value_error():
    async def run():
        client = _dead_client()
        try:
            with pytest.raises(ValueError, match="axes and"):
                await client.jog_l("WRF", axes=["X", "Y"], speeds_list=[0.5])
            # Typed as the axis literals; a script passing a string is
            # what the runtime check exists for.
            with pytest.raises(ValueError, match="unknown axis"):
                await client.jog_l("WRF", cast(Any, "Q"), 0.5, 0.2)
            with pytest.raises(ValueError, match="unknown axis"):
                await client.jog_l(
                    "WRF", axes=cast(Any, ["X", "W"]), speeds_list=[0.5, 0.5]
                )
        finally:
            await client.close()

    asyncio.run(run())


def test_the_unconfirmed_sentinel_is_a_plain_no_for_the_waits():
    """``-1`` is what the queued verbs return when no ack arrived; waiting
    on it must answer False / None, never overflow the wire's index."""

    async def run():
        client = _dead_client()
        try:
            assert await client.wait_command(-1, timeout=0.1) is False
            assert await client.command_verdict(-1) is None
        finally:
            await client.close()

    asyncio.run(run())


def test_a_passive_tools_status_on_the_sync_facade_is_a_query_not_a_recursion():
    """The bare flange has no action verbs, so waldoctl hands it back
    unwrapped; its ``status()`` still has to be synchronous — and reach the
    runtime rather than call itself."""
    flange = _cfg.canonical_tool_key("Flange")
    client = RobotClient(host="127.0.0.1", port=1, timeout=0.2, retries=0)
    try:
        tool = client._bound_tools[flange]
        # Nothing answers port 1: the query comes back None, not a coroutine
        # and not a RecursionError.
        assert tool.status() is None
    finally:
        client.close()
