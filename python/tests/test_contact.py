"""Contact detection, end to end: a live ``par6d --sim`` whose physics holds
a post the runtime was never told about, and a program that drives the arm
through where it stands.

Skipped, not failed, when no ``par6d`` binary is reachable — see
:mod:`live_daemon`.
"""

from __future__ import annotations

import asyncio
import json
import math

import pytest
from live_daemon import LiveDaemon, requires_par6d, settle_at
from waldoctl.shapes import Box

from par6.client import ContactReadout, RobotClient, RobotError
from par6.client.dry_run_client import DryRunRobotClient
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


#: A hand on the base (node 0) that never lets go: turning it toward the
#: post from a few seconds after boot. An arm held in position does not
#: give to it; a floating one goes where it pushes.
HAND_ON_THE_BASE = """
[joint_push]
at_s = 5.0
duration_s = 3000.0
node = 0
nm = 0.8
"""


@pytest.mark.timeout(180)
async def test_a_floating_arm_guided_into_a_keep_out_backs_out_and_waits_for_reset(
    tmp_path,
):
    """The post is a keep-out the program declared, and the hand takes the
    floating arm into it: the arm backs out the way it came, against the
    hand, and holds clear of it — enabled, referenced, the keep-out named —
    until it is reset.  Floated again with the hand still on it, it is
    caught again."""
    live = LiveDaemon.start(tmp_path, scenario=HAND_ON_THE_BASE)
    try:
        async with live.client() as client:
            assert await client.wait_ready(timeout=STEP_BUDGET_S)
            await settle_at(client, REACH_DEG)
            post = Box(
                name="post",
                x=0.06,
                y=0.06,
                z=0.4,
                pose=(0.3204, 0.185, 0.2, 0.0, 0.0, 0.0),
            )
            assert await client.set_shapes([post]) == 1

            for _ in range(2):
                assert await client.freedrive(True) == 1
                assert await client.wait_status(
                    lambda s: s.protective_stop, timeout=60.0
                ), "the hand never took the floating arm into the keep-out"
                standing = await client.error()
                assert standing is not None and standing.code == ErrorCode.SYS_KEEP_OUT
                base_deg: list[float] = []
                named: set[str] = set()

                def at_rest_and_held(s) -> bool:
                    resting = (
                        s.enabled
                        and s.homed
                        and not s.freedrive
                        and all(abs(v) < 0.5 for v in s.speeds)
                    )
                    if resting:
                        base_deg.append(float(s.angles[0]))
                        named.update(n for pair in s.collision_pairs for n in pair)
                    return resting

                assert await client.wait_status(
                    at_rest_and_held, timeout=STEP_BUDGET_S
                ), "a caught arm is held: enabled, referenced, at rest, not floating"
                assert 0.0 < base_deg[-1] < 30.0, (
                    f"the arm rests at {base_deg[-1]:.1f} deg, short of the post"
                )
                assert "shape:post" in named, f"the keep-out is named: {named}"

                with pytest.raises(RobotError) as refused:
                    await client.freedrive(True)
                assert refused.value.code == ErrorCode.SYS_KEEP_OUT
                assert await client.reset() == 1
                assert await client.wait_status(
                    lambda s: not s.protective_stop and s.error is None,
                    timeout=STEP_BUDGET_S,
                )
    finally:
        live.stop()


@pytest.mark.timeout(240)
def test_calibration_fits_the_thresholds_and_saves_them_in_the_overlay(
    daemon: LiveDaemon, tmp_path, capsys
):
    """``par6 calibrate-contact --save``: the arm swings, the runtime ends
    up running on thresholds read from those swings with its reaction as it
    was, and the overlay holds them beside what it already held."""
    from par6 import _par6
    from par6.cli import main

    overlay = tmp_path / "local.toml"
    overlay.write_text("# bench 3\n[limits]\ntcp_speed_max_m_s = 0.2\n")

    async def stand_at_reach() -> ContactReadout:
        async with daemon.client() as client:
            assert await client.wait_ready(timeout=STEP_BUDGET_S)
            await settle_at(client, REACH_DEG)
            readout = await client.contact()
            assert readout is not None
            return readout

    shipped = asyncio.run(stand_at_reach())

    capsys.readouterr()
    code = main(
        [
            "--host",
            "127.0.0.1",
            "--port",
            str(daemon.command_port),
            "--json",
            "calibrate-contact",
            "--spread-deg",
            "12",
            "--save",
            str(overlay),
        ]
    )
    assert code == 0
    fitted = json.loads(capsys.readouterr().out)
    assert fitted["applied"] and fitted["reaction"] == shipped.reaction
    assert any(peak > 0.0 for peak in fitted["peak_signal"]), fitted
    for peak, limit in zip(fitted["peak_signal"], fitted["sustained_nm"]):
        assert limit >= 2.0 * peak and limit >= 0.05
    assert fitted["sustained_nm"] != pytest.approx(shipped.sustained_nm)

    with RobotClient(host="127.0.0.1", port=daemon.command_port) as client:
        now = client.contact()
        assert now is not None and now.reaction == shipped.reaction
        assert now.sustained_nm == pytest.approx(fitted["sustained_nm"])
        assert now.impact_nm == pytest.approx(fitted["impact_nm"])

    saved = overlay.read_text()
    assert "# bench 3" in saved and "tcp_speed_max_m_s = 0.2" in saved
    # Read back the way the runtime reads it: as an overlay's [contact].
    again = _par6.overlay_with_contact(
        saved, fitted["reaction"], fitted["sustained_nm"], fitted["impact_nm"]
    )
    assert again == saved, "the saved table is exactly the fitted thresholds"


def test_a_dry_run_shows_the_calibration_swings_and_measures_nothing():
    """The preview of a program that calibrates: the swings are moves of
    the program, out and back, ending where the arm stood, and nothing is
    measured or set."""
    dry_run = DryRunRobotClient(initial_joints_deg=REACH_DEG)
    assert dry_run.calibrate_contact(spread=0.2) is None
    plan = dry_run.plan()
    assert len(plan.blocks) >= 12 and len(plan.blocks) % 2 == 0, (
        f"each swing is a move out and a move back: {len(plan.blocks)} blocks"
    )
    assert all(b.error is None and b.rows > 0 for b in plan.blocks)
    assert plan.stop == "completed", plan.stop
    assert dry_run.angles() == pytest.approx(REACH_DEG, abs=0.05)
    with pytest.raises(RuntimeError, match="no room"):
        DryRunRobotClient(initial_joints_deg=REACH_DEG).calibrate_contact(spread=10.0)


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


@pytest.mark.timeout(120)
async def test_the_reaction_set_live_decides_whether_the_arm_stops(tmp_path):
    """The tuning loop's write, against the post: told to report, the arm
    says it has met something and goes on pushing; told to stop, the same
    move ends as a contact.  What was set is what the readout answers, at
    once, and thresholds that could not be thresholds are refused."""
    live = LiveDaemon.start(tmp_path, scenario=POST)
    try:
        async with live.client() as client:
            assert await client.wait_ready(timeout=STEP_BUDGET_S)
            await settle_at(client, REACH_DEG)
            shipped = await client.contact()
            assert shipped is not None and shipped.reaction == "stop"
            through = list(REACH_DEG)
            through[0] = 60.0

            raised = [limit + 0.25 for limit in shipped.impact_nm]
            assert (
                await client.set_contact_limits(
                    reaction="warn",
                    sustained_nm=shipped.sustained_nm,
                    impact_nm=raised,
                )
                == 1
            )
            reporting = await client.contact()
            assert reporting is not None and reporting.reaction == "warn"
            assert reporting.impact_nm == pytest.approx(raised)
            assert reporting.sustained_nm == pytest.approx(shipped.sustained_nm)

            pressing = await client.move_j(through, speed=0.5)
            assert await client.wait_status(
                lambda s: any(w[1] == ErrorCode.SYS_CONTACT for w in s.warnings),
                timeout=STEP_BUDGET_S,
            ), "an arm that reports must say it has met the post"
            assert await client.wait_status(
                lambda s: not s.protective_stop and s.error is None, timeout=1.0
            ), "reporting is not stopping"
            assert await client.stop() == 1
            with pytest.raises(RobotError) as cancelled:
                await client.wait_command(pressing, timeout=STEP_BUDGET_S)
            assert cancelled.value.cancelled
            unstopped = await client.contact()
            assert unstopped is not None and unstopped.trips == 0

            await client.move_j(REACH_DEG, speed=0.5, wait=True, timeout=STEP_BUDGET_S)
            assert (
                await client.set_contact_limits(
                    reaction="stop",
                    sustained_nm=shipped.sustained_nm,
                    impact_nm=shipped.impact_nm,
                )
                == 1
            )
            with pytest.raises(RobotError) as stopped:
                await client.move_j(
                    through, speed=0.5, wait=True, timeout=STEP_BUDGET_S
                )
            assert stopped.value.code == ErrorCode.SYS_CONTACT

            for hostile in (-1.0, 0.0, math.nan):
                with pytest.raises((RobotError, ValueError)):
                    await client.set_contact_limits(
                        reaction="stop",
                        sustained_nm=[hostile] * len(raised),
                        impact_nm=shipped.impact_nm,
                    )
            with pytest.raises(ValueError):
                await client.set_contact_limits(
                    reaction="sometimes",
                    sustained_nm=shipped.sustained_nm,
                    impact_nm=shipped.impact_nm,
                )
    finally:
        live.stop()
