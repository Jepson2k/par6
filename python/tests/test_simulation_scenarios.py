"""Program replay preserves its initial model and command boundaries."""

import numpy as np
import pytest

from par6.client.dry_run_client import DryRunRobotClient
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
