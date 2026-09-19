"""TCP calibration readback and execution through the native client."""

import socket
from typing import cast

import pytest
from live_daemon import requires_par6d

from par6.client import AsyncRobotClient


async def test_unanswered_tcp_readback_cannot_clear_saved_calibration():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as silent:
        silent.bind(("127.0.0.1", 0))
        client = AsyncRobotClient(
            host="127.0.0.1", port=silent.getsockname()[1], timeout=0.05, retries=0
        )
        try:
            with pytest.raises(TimeoutError):
                await client.tcp_offset()
            with pytest.raises(TimeoutError):
                await client.tcp_transform()
        finally:
            await client.close()


def test_legacy_offset_remains_available_in_native_preview():
    import numpy as np

    from par6.client.dry_run_client import DryRunRobotClient

    preview = DryRunRobotClient(initial_joints_deg=[-115, -40, 200, 0, 60, 180])
    before = np.asarray(preview.pose())[:3]
    preview.set_tcp_offset(0, 0, 20)
    after = np.asarray(preview.pose())[:3]
    assert np.linalg.norm(after - before) == pytest.approx(20, abs=0.001)


@pytest.mark.e2e
@requires_par6d
@pytest.mark.timeout(90)
async def test_full_tcp_transform_agrees_across_wire_fk_preview_and_motion(daemon):
    import numpy as np
    from live_daemon import angles_now, pose_now, settle_at
    from waldoctl.setup import Pose, PoseValues

    from par6.client.dry_run_client import DryRunRobotClient
    from par6.robot import Robot

    values = (5.0, -3.0, 20.0, 20.0, 25.0, -10.0)
    async with daemon.client() as client:
        await settle_at(client, [-115, -40, 200, 0, 60, 180])
        angles = await angles_now(client)
        preview = DryRunRobotClient(
            initial_joints_deg=angles, config_path=str(daemon.config)
        )
        before = Pose(cast(PoseValues, tuple(preview.pose()))).matrix()
        expected = before @ Pose(values).matrix()
        index = await client.set_tcp_transform(*values)
        assert index >= 0 and await client.wait_command(index, timeout=10)
        assert await client.tcp_transform() == pytest.approx(values)
        assert await client.tcp_offset() == pytest.approx(values[:3])
        assert await client.wait_status(
            lambda s: np.allclose(np.asarray(s.pose).reshape(4, 4), expected, atol=0.1),
            timeout=10,
        ), "stationary STATUS did not adopt the full transform"

        local = Robot()
        local.set_active_tool(
            preview.active_tool_key,
            tcp_offset_m=(values[0] / 1000, values[1] / 1000, values[2] / 1000),
            tcp_rotation_rad=tuple(np.radians(values[3:])),
        )
        local_pose = local.fk(np.radians(angles), np.empty(6))
        local_matrix = Pose(
            tuple([*(local_pose[:3] * 1000), *np.degrees(local_pose[3:])])
        ).matrix()
        assert local_matrix == pytest.approx(expected, abs=0.001)
        preview.set_tcp_transform(*values)
        assert preview.tcp_transform() == pytest.approx(values)
        predicted = preview.move_l([0, 0, 5, 0, 0, 0], frame="TRF", rel=True, speed=0.2)
        assert predicted is not None and predicted.error is None
        target = expected @ Pose((0, 0, 5, 0, 0, 0)).matrix()
        predicted_pose = Pose(cast(PoseValues, tuple(preview.pose()))).matrix()
        assert predicted_pose == pytest.approx(target, abs=0.01)

        index = await client.move_l(
            [0, 0, 5, 0, 0, 0], frame="TRF", rel=True, speed=0.2
        )
        assert index >= 0 and await client.wait_command(index, timeout=20)
        assert await client.wait_status(
            lambda s: np.allclose(
                np.asarray(s.pose).reshape(4, 4)[:3, 3], target[:3, 3], atol=1.0
            ),
            timeout=10,
        )
        actual = await pose_now(client)
        assert Pose(cast(PoseValues, tuple(actual))).matrix()[:3, :3] == pytest.approx(
            target[:3, :3], abs=0.02
        )

        delay = await client.delay(5)
        assert await client.wait_status(lambda s: s.executing_index == delay, timeout=5)
        pending = await client.set_tcp_transform(0, 0, 40, 0, 90, 0)
        assert pending > delay
        assert await client.tcp_transform() == pytest.approx(values)
        assert await client.stop() > 0
        assert await client.tcp_transform() == pytest.approx(values)
        index = await client.move_l(actual, speed=0.2)
        assert index >= 0 and await client.wait_command(index, timeout=20)

        index = await client.select_tool(preview.active_tool_key)
        assert await client.wait_command(index, timeout=10)
        assert await client.tcp_transform() == pytest.approx(values)
        index = await client.set_tcp_offset(1, 2, 3)
        assert await client.wait_command(index, timeout=10)
        assert await client.tcp_transform() == pytest.approx([1, 2, 3, 0, 0, 0])
        preview.set_tcp_offset(1, 2, 3)
        assert preview.tcp_transform() == pytest.approx([1, 2, 3, 0, 0, 0])
        assert await client.reset_state() > 0
        assert await client.tcp_transform() == pytest.approx([0] * 6)
