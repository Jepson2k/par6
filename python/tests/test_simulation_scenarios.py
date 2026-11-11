"""Program replay preserves its initial model and command boundaries."""

import numpy as np
import pytest

from par6.client import RobotError
from par6.client.dry_run_client import DryRunRobotClient
from par6.protocol import ErrorCode
from par6.robot import Robot


def test_physics_replays_the_programs_initial_tcp():
    client = DryRunRobotClient()
    client.delay(0.2)
    client.set_tcp_offset(x=100)
    client.delay(0.2)
    before = (client.angles(), client.tcp_offset())
    ticks = client.simulate(2)
    assert ticks.stop == "completed", ticks.blocks
    nominal = Robot().fk_batch(np.asarray(ticks.joints_rad, dtype=np.float64))
    offsets = np.linalg.norm(ticks.tcp[:, :3] - nominal[:, :3], axis=1)
    assert offsets[0] == pytest.approx(0, abs=1e-5)
    assert offsets[-1] == pytest.approx(0.1, abs=1e-5)
    assert (client.angles(), client.tcp_offset()) == before
    repeated = client.simulate(2)
    assert repeated.digest == ticks.digest


def test_a_push_in_the_scenario_ends_the_replayed_move_as_a_contact():
    """The replay runs the runtime's own contact detection: a torque against
    a joint in motion ends that move the way it would on the arm, short of
    its target, and the same program with nothing in the way runs clean."""
    client = DryRunRobotClient()
    start = client.angles()
    target = list(start)
    target[0] += 60.0
    client.move_j(target, speed=0.5)

    clean = client.simulate(10)
    assert clean.stop == "completed" and clean.blocks[-1].error is None
    assert np.degrees(clean.joints_rad[-1, 0]) == pytest.approx(target[0], abs=1.0)

    pushed = client.simulate(
        10,
        scenario={
            "joint_push": {"at_s": 0.4, "duration_s": 2.0, "node": 0, "nm": -3.0}
        },
    )
    error = pushed.blocks[-1].error
    assert isinstance(error, RobotError) and error.code == ErrorCode.SYS_CONTACT
    assert np.degrees(pushed.joints_rad[-1, 0]) < target[0] - 5.0


def test_a_replay_reads_contacts_with_the_limits_the_program_set():
    """A program that raises its thresholds before a move it expects to be
    nudged in is replayed with them in force: the same nudge that ends the
    move as a contact under the shipped limits is ridden through."""
    nudge = {"joint_push": {"at_s": 0.4, "duration_s": 0.3, "node": 0, "nm": -2.0}}

    def swing(client: DryRunRobotClient) -> list[float]:
        target = client.angles()
        target[0] += 60.0
        client.move_j(target, speed=0.5)
        return target

    shipped = DryRunRobotClient()
    swing(shipped)
    error = shipped.simulate(10, scenario=nudge).blocks[-1].error
    assert isinstance(error, RobotError) and error.code == ErrorCode.SYS_CONTACT

    raised = DryRunRobotClient()
    raised.set_contact_limits(
        reaction="stop", sustained_nm=[6.0] * 6, impact_nm=[6.0] * 6
    )
    target = swing(raised)
    ran = raised.simulate(10, scenario=nudge)
    assert ran.stop == "completed" and ran.blocks[-1].error is None
    assert np.degrees(ran.joints_rad[-1, 0]) == pytest.approx(target[0], abs=1.0)
