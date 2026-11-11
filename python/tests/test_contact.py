"""Contact detection, end to end: a live ``par6d --sim`` whose physics holds
a post the runtime was never told about, and a program that drives the arm
through where it stands.

Skipped, not failed, when no ``par6d`` binary is reachable — see
:mod:`live_daemon`.
"""

from __future__ import annotations

import math

import pytest
from live_daemon import LiveDaemon, requires_par6d, settle_at

from par6.client import RobotError
from par6.protocol import ErrorCode

pytestmark = [pytest.mark.e2e, requires_par6d]

STEP_BUDGET_S = 20.0

#: Reaching down in front of the base, tool about 0.37 m out.
REACH_DEG = [math.degrees(q) for q in (0.0, -0.25, 4.35, 0.0, -1.28, 0.0)]

#: A post on the tool's circle, 30 degrees round from where it starts.
POST = """
[obstacle]
size_m = [0.06, 0.06, 0.4]
pose = [0.3204, 0.185, 0.2, 0.0, 0.0, 0.0]
"""


@pytest.mark.timeout(120)
async def test_a_move_into_an_undeclared_obstacle_stops_and_reset_rearms(tmp_path):
    live = LiveDaemon.start(tmp_path, scenario=POST)
    try:
        async with live.client() as client:
            assert await client.wait_ready(timeout=STEP_BUDGET_S)
            await settle_at(client, REACH_DEG)
            quiet = await client.contact()
            assert quiet is not None and quiet.reaction == "stop" and quiet.trips == 0

            through = list(REACH_DEG)
            through[0] = 60.0
            with pytest.raises(RobotError) as stopped:
                await client.move_j(
                    through, speed=0.5, wait=True, timeout=STEP_BUDGET_S
                )
            assert stopped.value.code == ErrorCode.SYS_CONTACT

            # Backed off and holding: still enabled, still referenced, short
            # of the post, with the contact standing.
            assert await client.wait_status(
                lambda s: (
                    s.enabled
                    and s.homed
                    and all(abs(v) < 0.5 for v in s.speeds)
                    and 0.0 < s.angles[0] < 30.0
                ),
                timeout=STEP_BUDGET_S,
            ), "the arm must end up held short of the post, enabled and referenced"
            standing = await client.error()
            assert standing is not None and standing.code == ErrorCode.SYS_CONTACT

            readout = await client.contact()
            assert readout is not None and readout.trips == 1
            assert readout.last_trip_joint is not None
            assert readout.last_trip_channel in ("sustained", "impact")

            # Away from the post is still a move, and still refused.
            away = list(REACH_DEG)
            away[0] = -20.0
            with pytest.raises(RobotError) as refused:
                await client.move_j(away, speed=0.5, wait=True, timeout=STEP_BUDGET_S)
            assert refused.value.code == ErrorCode.SYS_CONTACT

            assert await client.reset() == 1
            assert await client.wait_status(
                lambda s: s.enabled and s.homed, timeout=STEP_BUDGET_S
            )
            assert await client.error() is None
            await client.move_j(away, speed=0.5, wait=True, timeout=STEP_BUDGET_S)
            assert await client.wait_status(
                lambda s: abs(s.angles[0] - away[0]) < 1.0, timeout=STEP_BUDGET_S
            )
    finally:
        live.stop()


@pytest.mark.timeout(120)
async def test_a_floating_arm_that_runs_away_is_caught_and_refuses_to_float_until_reset(
    daemon: LiveDaemon,
):
    """Freedrive with a payload declared that the arm is not carrying: the
    drives push up against two kilos that are not there, the arm takes off
    faster than a hand moves it, and the catch stops it and holds."""
    async with daemon.client() as client:
        assert await client.wait_ready(timeout=STEP_BUDGET_S)
        await settle_at(client, REACH_DEG)
        assert await client.freedrive(True) == 1
        assert await client.wait_status(lambda s: s.freedrive, timeout=STEP_BUDGET_S)

        assert await client.set_payload(2.0, com=(0.0, 0.0, 0.05)) == 1
        assert await client.wait_status(
            lambda s: s.protective_stop, timeout=STEP_BUDGET_S
        ), "the arm was never caught"
        standing = await client.error()
        assert standing is not None and standing.code == ErrorCode.SYS_FREEDRIVE_CATCH
        assert await client.wait_status(
            lambda s: (
                s.enabled
                and s.homed
                and not s.freedrive
                and all(abs(v) < 3.0 for v in s.speeds)
            ),
            timeout=STEP_BUDGET_S,
        ), "a caught arm is held: enabled, referenced, at rest and not floating"

        with pytest.raises(RobotError) as refused:
            await client.freedrive(True)
        assert refused.value.code == ErrorCode.SYS_FREEDRIVE_CATCH

        assert await client.set_payload(0.0) == 1
        assert await client.reset() == 1
        assert await client.wait_status(
            lambda s: not s.protective_stop and s.error is None, timeout=STEP_BUDGET_S
        )
        assert await client.freedrive(True) == 1
        assert await client.wait_status(lambda s: s.freedrive, timeout=STEP_BUDGET_S)
        assert not await client.wait_status(lambda s: s.protective_stop, timeout=1.0), (
            "with the payload cleared the arm floats where it is"
        )
