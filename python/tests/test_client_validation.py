"""Client-side checks that need no runtime: the calls a script gets wrong
are refused with the exception the rest of the API raises, and the API's
own sentinels never escape as extension errors."""

from __future__ import annotations

import asyncio
import math
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


def test_a_keyword_no_planned_move_declares_is_a_type_error():
    """``rel`` belongs to ``move_j`` and ``move_l`` alone, and a planned
    move's ``**wait_kwargs`` carry only what ``wait_command`` takes: anything
    else is refused before a byte is sent, as parol6's client refuses it,
    rather than planned as if it had not been written."""

    async def run():
        client = _dead_client()
        pose = [200.0, 0.0, 300.0, 180.0, 0.0, 0.0]
        try:
            with pytest.raises(TypeError, match="rel"):
                await client.move_c(pose, pose, rel=True, speed=0.5)
            with pytest.raises(TypeError, match="rel"):
                await client.move_s([pose, pose], rel=True, speed=0.5)
            with pytest.raises(TypeError, match="rel"):
                await client.move_p([pose, pose], rel=True, speed=0.5)
            with pytest.raises(TypeError, match="bogus"):
                await client.move_l(pose, speed=0.5, bogus=1)
            with pytest.raises(TypeError, match="bogus"):
                await client.move_j([0.0] * 6, speed=0.5, wait=False, bogus=1)
            with pytest.raises(TypeError, match="bogus"):
                await client.home(bogus=1)
        finally:
            await client.close()

    asyncio.run(run())


def test_planned_move_timing_out_of_range_is_a_value_error():
    """``speed`` and ``accel`` are fractions in (0, 1] and ``duration`` a
    finite time, 0 meaning "not given": every planned move refuses anything
    else before a byte is sent, NaN and inf included."""

    async def run():
        client = _dead_client()
        pose = [200.0, 0.0, 300.0, 180.0, 0.0, 0.0]
        moves = (
            lambda **t: client.move_j([0.0] * 6, **t),
            lambda **t: client.move_l(pose, **t),
            lambda **t: client.move_c(pose, pose, **t),
            lambda **t: client.move_s([pose, pose], **t),
            lambda **t: client.move_p([pose, pose], **t),
        )
        try:
            for move in moves:
                for bad in (0.0, -0.5, 1.5, math.nan, math.inf):
                    with pytest.raises(ValueError, match="speed"):
                        await move(speed=bad)
                    with pytest.raises(ValueError, match="accel"):
                        await move(accel=bad)
                for bad in (-1.0, math.nan, math.inf):
                    with pytest.raises(ValueError, match="duration"):
                        await move(duration=bad)
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


def test_an_execution_state_request_the_runtime_stops_answering_is_a_zero(
    monkeypatch: pytest.MonkeyPatch,
):
    """``set_execution_speed`` and ``pause`` answer 0 when no confirmation
    arrives — a runtime that goes away between the request and its
    readback answers the same 0 a timeout does, never an exception."""

    async def run():
        client = _dead_client()
        try:

            async def accepted(_request):
                return True

            async def gone(**_kwargs):
                raise ConnectionError("Controller execution speed is unavailable")

            monkeypatch.setattr(client, "_call", accepted)
            monkeypatch.setattr(client, "execution_speed", gone)
            assert await client.set_execution_speed(0.5) == 0
            assert await client.pause() == 0
        finally:
            await client.close()

    asyncio.run(run())
