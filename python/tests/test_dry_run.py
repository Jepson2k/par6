"""The offline dry-run client, checked against the runtime it predicts.

The offline-only tests assert properties the runtime enforces (limits, path
geometry, the refusals ``par6d`` answers with).  The e2e test closes the loop:
the same command is planned offline and queued on a live ``par6d --sim``, and
the prediction has to match what the runtime actually executed — a dry run
that only agrees with itself proves nothing.
"""

from __future__ import annotations

import math
import time

import numpy as np
import pytest
from live_daemon import LiveDaemon, requires_par6d

from par6 import config as _cfg
from par6 import motion as _motion
from par6.client import RobotError
from par6.client.dry_run_client import DryRunRobotClient
from par6.protocol.constants import CompletionPolicy, ErrorCode
from par6.robot import Robot

try:
    import pinokin
except ImportError:  # binary wheel unavailable on this platform
    pinokin = None

pytestmark = pytest.mark.skipif(
    pinokin is None, reason="pinokin binary wheel not installed"
)


def park_deg() -> list[float]:
    return np.degrees(_cfg.homing_ready_pose_rad()).tolist()


@pytest.fixture(scope="module")
def dry_run() -> DryRunRobotClient:
    return Robot().create_dry_run_client(initial_joints_deg=park_deg())


class TestPlannedMotion:
    def test_plan_obeys_the_config_limits_under_every_profile(self) -> None:
        """Each advertised profile must produce a plan the runtime's own limits
        admit: no tick may step a joint faster than its EXEC velocity ceiling
        (scaled by the requested speed), and every plan must land on the target.
        A slower speed must take longer."""
        limits = _motion.MotionLimits.from_config("exec")
        dt = _motion.tick_dt_s()
        start = _cfg.homing_ready_pose_rad()
        target = start + np.radians([25.0, -10.0, 15.0, 0.0, 20.0, 0.0])

        for profile in _motion.PROFILES:
            durations: list[float] = []
            for speed in (1.0, 0.25):
                path = _motion.plan_joint_move(
                    start, target, limits, dt, profile=profile, speed_fraction=speed
                )
                np.testing.assert_allclose(path[-1], target, atol=1e-6)
                step = np.abs(np.diff(np.vstack([start, path]), axis=0)) / dt
                ceiling = limits.velocity * speed
                assert np.all(step <= ceiling * 1.02 + 1e-9), (
                    f"{profile} at speed {speed} exceeds the velocity ceiling: "
                    f"{step.max(axis=0)} vs {ceiling}"
                )
                durations.append(len(path) * dt)
            full_speed, quarter_speed = durations
            assert quarter_speed > full_speed, (
                f"{profile}: a quarter-speed move must take longer"
            )

    def test_duration_request_stretches_the_plan(self, dry_run) -> None:
        """``duration=`` is a minimum the plan is stretched to meet, and it is
        not silently ignored: the same move at full speed is much shorter."""
        target = park_deg()
        target[0] += 20.0
        fast = dry_run.move_j(target, speed=1.0)
        dry_run.teleport(park_deg())
        slow = dry_run.move_j(target, duration=4.0)
        assert fast.duration < 1.5
        assert slow.duration == pytest.approx(4.0, abs=2 * _motion.tick_dt_s())
        np.testing.assert_allclose(
            slow.end_joints_rad, np.radians(target), atol=1e-6
        )


class TestCartesianMotion:
    def test_move_l_previews_a_straight_line_and_reports_where_it_fails(
        self, dry_run
    ) -> None:
        """A linear move must preview as a straight TCP line that ends on the
        requested pose; a line that leaves the workspace must report per-pose
        validity so a preview can draw how far it gets."""
        dry_run.teleport(park_deg())
        start = np.asarray(dry_run.pose())
        target = start.copy()
        target[2] += 40.0

        result = dry_run.move_l(target.tolist(), speed=0.5)
        assert result.error is None
        assert result.duration > 0.0
        points = result.tcp_poses[:, :3] * 1000.0
        assert np.allclose(points[-1], target[:3], atol=0.5)
        # Straightness: every sampled point lies on the start->end line.
        line = points[-1] - points[0]
        offsets = points - points[0]
        deviation = np.linalg.norm(
            offsets - np.outer(offsets @ line / (line @ line), line), axis=1
        )
        assert deviation.max() < 0.1, f"TCP path bows by {deviation.max():.3f} mm"

        before = list(dry_run.angles())
        unreachable = np.asarray(dry_run.pose())
        unreachable[0] += 5000.0
        blocked = dry_run.move_l(unreachable.tolist(), speed=0.5)
        assert blocked.error is not None
        assert blocked.error.code == ErrorCode.IK_PARTIAL_PATH
        assert blocked.valid is not None and not blocked.valid.all()
        assert len(blocked.valid) == blocked.tcp_poses.shape[0]
        # The arm must not have moved: the runtime rejects the whole command.
        np.testing.assert_allclose(dry_run.angles(), before, atol=1e-9)

    def test_refuses_what_the_runtime_refuses(self, dry_run) -> None:
        """Parameters and commands ``par6d`` rejects must be rejected here with
        the same code, so a preview never promises motion the arm will refuse."""
        dry_run.teleport(park_deg())
        with pytest.raises(RobotError) as blend:
            dry_run.move_l(dry_run.pose(), r=5.0)
        assert blend.value.code == ErrorCode.COMM_VALIDATION_ERROR

        with pytest.raises(RobotError) as tool:
            dry_run.select_tool("SSG48")
        assert tool.value.code == ErrorCode.COMM_VALIDATION_ERROR

        with pytest.raises(RobotError) as profile:
            dry_run.select_profile("BANG_BANG")
        assert profile.value.code == ErrorCode.SYS_PROFILE_INVALID

        curved = dry_run.move_c([0.0] * 6, [0.0] * 6)
        assert curved.error is not None
        assert curved.error.code == ErrorCode.MOTN_SETUP_FAILED
        assert curved.tcp_poses.shape[0] == 0

        far = list(dry_run.angles())
        far[1] = math.degrees(_cfg.soft_limits_rad()[1, 1]) + 20.0
        with pytest.raises(RobotError) as outside:
            dry_run.move_j(far)
        assert outside.value.code == ErrorCode.MOTN_SETUP_FAILED

        unhomed = Robot().create_dry_run_client(
            initial_joints_deg=park_deg(), initial_homed=False
        )
        with pytest.raises(RobotError) as gate:
            unhomed.move_j(park_deg())
        assert gate.value.code == ErrorCode.MOTN_NOT_HOMED
        # Jogging stays available while un-homed, as it does on the runtime.
        assert unhomed.jog_j(0, 0.2, 0.2).duration > 0.0


class TestProgramWorkflow:
    def test_a_program_previews_as_one_continuous_timeline(self) -> None:
        """Drive a whole program the way the editor does and check the results
        chain: every segment starts where the previous one ended, the tool
        action holds position, and home lands on the configured ready pose."""
        client = Robot().create_dry_run_client(initial_joints_deg=[0.0] * 6)
        results = []
        results.append(client.home())
        np.testing.assert_allclose(
            results[-1].end_joints_rad, _cfg.homing_ready_pose_rad(), atol=1e-9
        )

        above = np.asarray(client.pose())
        above[2] += 30.0
        results.append(client.move_l(above.tolist(), speed=0.4))
        results.append(client.tool.close())
        joints = list(client.angles())
        joints[0] -= 15.0
        results.append(client.move_j(joints, speed=0.6))
        assert client.flush() == []

        assert all(r.error is None for r in results)
        for previous, following in zip(results, results[1:]):
            np.testing.assert_allclose(
                following.tcp_poses[0][:3], previous.tcp_poses[-1][:3], atol=2e-3
            )
        # The tool action holds the arm still and carries no plan of its own.
        held = results[2]
        assert held.tcp_poses.shape[0] == 1
        np.testing.assert_allclose(
            held.end_joints_rad, results[1].end_joints_rad, atol=1e-12
        )
        assert client.angles() == pytest.approx(np.degrees(results[-1].end_joints_rad))
        assert sum(r.duration for r in results) > 0.0


@pytest.mark.e2e
@requires_par6d
@pytest.mark.timeout(300)
async def test_prediction_matches_what_the_runtime_executes(tmp_path) -> None:
    """The offline plan must match the motion a live ``par6d --sim`` runs.

    The same ``move_j`` is planned offline and queued on the runtime, with the
    completion policy set to ``commanded`` so the observed window is the plan's
    own sample stream and not a settle wait.  The window is timed from the
    enqueue to the COMPLETE the runtime pushes, which is the plan's execution
    plus dispatch — it can run long on a loaded box but never short, so the
    lower bound is tight and the upper one carries the overhead budget.  Two
    timings 5 s apart are run: a preview that reported a constant, ignored
    ``duration=``, or planned against the wrong limits fails the bounds and
    the shrink between them.
    """
    daemon = LiveDaemon.start(tmp_path)
    try:
        async with daemon.client() as client:
            assert await client.wait_status(lambda s: s.link_ok == 1, timeout=20.0)
            assert await client.reset() == 1
            park = park_deg()
            await _teleport_to(client, park)
            assert await client.set_completion_policy(CompletionPolicy.COMMANDED) == 1

            live_start = await client.angles()
            assert live_start is not None
            preview = Robot().create_dry_run_client(initial_joints_deg=live_start)

            observed: dict[float, tuple[float, float]] = {}
            for requested in (8.0, 3.0):
                target = list(live_start)
                target[0] += 25.0

                predicted = preview.move_j(target, duration=requested)
                preview.teleport(live_start)  # re-seed for the next comparison

                started = time.monotonic()
                index = await client.move_j(target, duration=requested)
                assert index >= 0
                assert await client.wait_command(index, timeout=60.0) is True
                measured = time.monotonic() - started
                observed[requested] = (predicted.duration, measured)

                assert predicted.duration - 0.3 <= measured <= predicted.duration + 1.5, (
                    f"duration={requested}: runtime took {measured:.3f}s, "
                    f"preview predicted {predicted.duration:.3f}s"
                )

                # The plan stops commanding at its last sample; the closed-loop
                # sim converges on it a little later, so let the arm settle
                # before comparing where it ended up.
                assert await client.wait_status(
                    lambda s: float(np.abs(np.asarray(s.speeds)).max()) < 0.02,
                    timeout=20.0,
                )
                landed = await client.angles()
                assert landed is not None
                predicted_deg = np.degrees(predicted.end_joints_rad)
                assert float(np.abs(np.asarray(landed) - predicted_deg).max()) < 2.5, (
                    f"duration={requested}: runtime landed at {landed}, "
                    f"preview predicted {predicted_deg.tolist()}"
                )
                await _teleport_to(client, live_start)

            # The measured window has to shrink by what the preview said it
            # would — the part of the comparison the dispatch overhead cancels
            # out of.
            slow_predicted, slow_measured = observed[8.0]
            fast_predicted, fast_measured = observed[3.0]
            assert (slow_measured - fast_measured) == pytest.approx(
                slow_predicted - fast_predicted, abs=1.0
            ), f"execution windows {observed} do not track the prediction"
    finally:
        daemon.stop()


async def _teleport_to(client, angles: list[float]) -> None:
    """Drive the sim to *angles*; teleport is unacked, so re-send until it lands."""
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline:
        await client.teleport(angles)
        if await client.wait_status(
            lambda s: s.homed
            and float(np.abs(np.asarray(s.angles) - angles).max()) < 1.0,
            timeout=0.5,
        ):
            return
    raise AssertionError("teleport never took effect")
