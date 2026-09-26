"""The offline dry-run client, checked against the runtime it predicts.

The offline-only tests assert properties the runtime enforces (limits, path
geometry, blend semantics, the refusals ``par6d`` answers with).  The e2e
tests close the loop: the same commands are planned offline and queued on a
live ``par6d --sim``, and the prediction has to match the joint path the
runtime actually drove, read back off the STATUS broadcast — because a dry
run that only agrees with itself proves nothing.
"""

from __future__ import annotations

import asyncio
import contextlib
import math
import time

import numpy as np
import pytest
from live_daemon import (
    STATUS_RATE_HZ,
    TICK_DT_S,
    LiveDaemon,
    requires_par6d,
    sim_config,
    teleport_to,
)
from waldoctl import CommandKind, command_table
from waldoctl.skills import UnresolvedPreview
from waldoctl.ticks import following_error

from par6 import config as _cfg
from par6._par6 import Preview as DryRunProfiles
from par6.client import RobotError
from par6.client.dry_run_client import DryRunRobotClient
from par6.protocol import IO_SLOTS, NUM_JOINTS, CompletionPolicy, ErrorCode
from par6.protocol.wire import MAX_JOG_DURATION_S
from par6.robot import Robot


def park_deg() -> list[float]:
    return np.degrees(_cfg.homing_ready_pose_rad()).tolist()


class _Block:
    """One command's motion, read off the commanded record.

    ``tcp_poses`` and ``joint_trajectory_rad`` are the block's rows (metres
    and radians); ``duration`` is the time they cover; ``end_joints_rad`` is
    where the arm stands when the block ends — its last row, or the row
    before it began for a command that moved nothing (empty when there is
    no such row).
    """

    def __init__(self, record, index: int) -> None:
        block = record.blocks[index]
        first, last = block.start_row, block.start_row + block.rows
        self.index = index
        self.rows = block.rows
        self.error = block.error
        self.move_type = block.move_type
        self.duration = block.rows * record.row_dt_s
        self.tcp_poses: np.ndarray = record.tcp[first:last].astype(np.float64)
        self.joint_trajectory_rad: np.ndarray = record.joints_rad[first:last].astype(
            np.float64
        )
        standing = last - 1 if block.rows else first - 1
        self.end_joints_rad: np.ndarray = (
            record.joints_rad[standing].astype(np.float64)
            if standing >= 0
            else np.empty(0)
        )


def _planned(client: DryRunRobotClient, index: int) -> _Block:
    """The block *index* owns in the commanded record — closing the blend
    hold, as reading the record does."""
    return _Block(client.plan(), index)


def _last(client: DryRunRobotClient) -> _Block:
    """The block of the command submitted last."""
    return _planned(client, client.program_length - 1)


def _offset(pose: np.ndarray, delta: tuple[float, float, float]) -> np.ndarray:
    """*pose* (mm + degrees) moved by *delta* mm, orientation untouched."""
    out = np.asarray(pose, dtype=np.float64).copy()
    out[:3] += delta
    return out


def _closest(points: np.ndarray, target: np.ndarray) -> float:
    """Closest approach of a sampled path to *target* [mm], measured against
    the path itself — a path passes through a point even when no sample lands
    exactly on it."""
    a, b = points[:-1], points[1:]
    d = b - a
    length2 = np.einsum("ij,ij->i", d, d)
    t = np.clip(
        np.einsum("ij,ij->i", target - a, d) / np.where(length2 > 0.0, length2, 1.0),
        0.0,
        1.0,
    )
    return float(np.linalg.norm(target - (a + t[:, None] * d), axis=1).min())


def _polyline_gap(points: np.ndarray, corners: list[np.ndarray]) -> float:
    """How far *points* strays from the polyline through *corners* [mm]."""
    polyline = np.stack(corners)
    return max(_closest(polyline, p) for p in points)


def _line_deviation_mm(points: np.ndarray) -> float:
    r"""How far the sampled path strays from the start->end line \[mm\]."""
    line = points[-1] - points[0]
    offsets = points - points[0]
    deviation = np.linalg.norm(
        offsets - np.outer(offsets @ line / (line @ line), line), axis=1
    )
    return float(deviation.max())


def _length(points: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def _tcp_speeds(result) -> np.ndarray:
    """TCP speed at every sample of a previewed motion [mm/s]."""
    points = result.tcp_poses[:, :3] * 1000.0
    return np.linalg.norm(np.diff(points, axis=0), axis=1) / (
        result.duration / len(points)
    )


def _ramp(speeds: np.ndarray) -> int:
    """Samples to ignore at each end: every motion starts and ends at rest."""
    return max(len(speeds) // 10, 1)


def _circle_through(
    p1: np.ndarray, p2: np.ndarray, p3: np.ndarray
) -> tuple[np.ndarray, float]:
    """Centre and radius of the circle through three points — derived here,
    independently of the client, so the arc is checked against geometry
    rather than against itself."""
    a, b = p2 - p1, p3 - p1
    aa, bb, ab = a @ a, b @ b, a @ b
    det = aa * bb - ab * ab
    centre = (
        p1 + a * (bb * (aa - ab)) / (2.0 * det) + b * (aa * (bb - ab)) / (2.0 * det)
    )
    return centre, float(np.linalg.norm(centre - p1))


@pytest.fixture(scope="module")
def dry_run() -> DryRunRobotClient:
    return Robot().create_dry_run_client(initial_joints_deg=park_deg())


def test_execution_override_retimes_preview_and_preserves_pause():
    start = park_deg()
    target = list(start)
    target[0] += 8
    normal = DryRunRobotClient(initial_joints_deg=start)
    slow = DryRunRobotClient(initial_joints_deg=start)
    nominal = _planned(normal, normal.move_j(target, duration=2))
    assert slow.pause() == 1
    assert slow.set_execution_speed(0.5) == 1
    assert slow.execution_speed().paused
    with pytest.raises(UnresolvedPreview, match="paused"):
        slow.delay(1)
    np.testing.assert_allclose(slow.angles(), start)
    assert slow.resume() == 1
    # The accepted dwell stays pending until the explicit resume, then
    # fills its block: the third command, after the pause and the speed.
    slow.flush()
    dwell = _planned(slow, 2)
    assert dwell.duration == pytest.approx(1, abs=2 * slow.plan().row_dt_s)
    retimed = _planned(slow, slow.move_j(target, duration=2))
    assert retimed.duration == pytest.approx(
        nominal.duration * 2, abs=2 * slow.plan().row_dt_s
    )
    # The same plan, stretched: every other row of the retimed motion is a
    # row of the nominal one, and both end on the target.
    np.testing.assert_allclose(
        retimed.joint_trajectory_rad[::2], nominal.joint_trajectory_rad, atol=1e-9
    )
    np.testing.assert_allclose(
        retimed.end_joints_rad, nominal.end_joints_rad, atol=1e-4
    )
    assert slow.execution_speed().applied_scale == 0.5
    for value in (0, True, 2, math.nan):
        with pytest.raises(ValueError):
            slow.set_execution_speed(value)

    # A clearing stop discards the queue the pause held, and the pause with
    # it: the next move plans without a resume.
    assert slow.pause() == 1
    assert slow.stop() == 1
    assert not slow.execution_speed().paused
    after_stop = _planned(slow, slow.move_j(start, duration=1))
    assert after_stop.duration == pytest.approx(2, abs=2 * slow.plan().row_dt_s)
    # A resume runs what the pause held: nothing stays queued behind it, and
    # the released move fills its block.
    assert slow.pause() == 1
    with pytest.raises(UnresolvedPreview, match="paused"):
        slow.move_j(target, duration=1)
    held_index = slow.program_length - 1
    assert len(slow.queue()) == 1
    assert slow.resume() == 1
    assert slow.queue() == []
    released = _planned(slow, held_index)
    assert released.duration == pytest.approx(2, abs=2 * slow.plan().row_dt_s)


class TestPlannedMotion:
    def test_plan_obeys_the_config_limits_under_every_profile(self, tmp_path) -> None:
        """Each advertised profile must produce a plan the runtime's own limits
        admit: no tick may step a joint faster than its EXEC velocity ceiling
        (scaled by the requested speed), and every plan must land on the target.
        A slower speed must take longer.

        Driven through the dry-run client — the plan under test is the one
        the runtime's own planner produces — with the config as the oracle.
        """
        cfg = _cfg.config()
        velocity = np.array(cfg.limits("exec")["velocity"])
        start = _cfg.homing_ready_pose_rad()
        target = start + np.radians([25.0, -10.0, 15.0, 0.0, 20.0, 0.0])
        # The CI tick is slower than the record's row rate, so every tick is
        # a row and the velocity check reads per-tick steps rather than a
        # stride's average, which would smear a fast tick across several.
        client = Robot().create_dry_run_client(
            initial_joints_deg=np.degrees(start).tolist(),
            config_path=str(sim_config(tmp_path / "config")),
        )
        dt = client._dt
        assert dt == pytest.approx(TICK_DT_S)

        for profile in DryRunProfiles.profiles():
            client.select_profile(profile)
            durations: list[float] = []
            for speed in (1.0, 0.25):
                client.teleport(np.degrees(start).tolist())
                planned = _planned(
                    client, client.move_j(np.degrees(target).tolist(), speed=speed)
                )
                assert planned.duration == pytest.approx(planned.rows * dt)
                path = planned.joint_trajectory_rad
                np.testing.assert_allclose(path[-1], target, atol=1e-6)
                step = np.abs(np.diff(np.vstack([start, path]), axis=0)) / dt
                ceiling = velocity * speed
                assert np.all(step <= ceiling * 1.02 + 1e-9), (
                    f"{profile} at speed {speed} exceeds the velocity ceiling: "
                    f"{step.max(axis=0)} vs {ceiling}"
                )
                durations.append(planned.duration)
            full_speed, quarter_speed = durations
            assert quarter_speed > full_speed, (
                f"{profile}: a quarter-speed move must take longer"
            )

    def test_duration_request_stretches_the_plan(self, dry_run) -> None:
        """``duration=`` is a minimum the plan is stretched to meet, and it is
        not silently ignored: the same move at full speed is much shorter."""
        target = park_deg()
        target[0] += 20.0
        fast = _planned(dry_run, dry_run.move_j(target, speed=1.0))
        dry_run.teleport(park_deg())
        slow = _planned(dry_run, dry_run.move_j(target, duration=4.0))
        assert fast.duration < 1.5
        assert slow.duration == pytest.approx(4.0, abs=2 * dry_run.plan().row_dt_s)
        np.testing.assert_allclose(slow.end_joints_rad, np.radians(target), atol=1e-6)

    def test_an_untimed_move_runs_at_half_speed_and_a_duration_sets_the_timing(
        self, dry_run
    ) -> None:
        """A planned move that names no timing runs at ``speed=0.5`` — not at
        full speed, and not refused — and a positive ``duration`` times the
        move even beside an explicit ``speed``."""
        dry_run.teleport(park_deg())
        target = np.asarray(dry_run.pose())
        target[1] += 50.0
        target[2] += 30.0

        def line(**timing: float) -> _Block:
            dry_run.teleport(park_deg())
            return _planned(dry_run, dry_run.move_l(target.tolist(), **timing))

        row = dry_run.plan().row_dt_s
        half, full, untimed = line(speed=0.5), line(speed=1.0), line()
        assert half.duration > full.duration + 2 * row, "speed must bind on this line"
        assert untimed.duration == pytest.approx(half.duration, abs=row)
        timed = line(duration=3.0, speed=1.0)
        assert timed.duration == pytest.approx(3.0, abs=2 * row)


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

        result = _planned(dry_run, dry_run.move_l(target.tolist(), speed=0.5))
        assert result.error is None
        assert result.duration > 0.0
        points = result.tcp_poses[:, :3] * 1000.0
        assert np.allclose(points[-1], target[:3], atol=0.5)
        bow = _line_deviation_mm(points)
        assert bow < 0.1, f"TCP path bows by {bow:.3f} mm"

        before = list(dry_run.angles())
        unreachable = np.asarray(dry_run.pose())
        unreachable[0] += 5000.0
        # The endpoint decides reachable-at-all before the path decides
        # reachable-along-the-way (the planner's own precheck), so a
        # target outside the workspace refuses the whole command.
        with pytest.raises(RobotError) as blocked:
            dry_run.move_l(unreachable.tolist(), speed=0.5)
        assert blocked.value.code == ErrorCode.IK_TARGET_UNREACHABLE
        # The arm must not have moved: the runtime rejects the whole command.
        np.testing.assert_allclose(dry_run.angles(), before, atol=1e-9)

    @pytest.mark.parametrize(
        "profile", ["RUCKIG", "TRAPEZOID", "QUINTIC", "TOPPRA", "LINEAR"]
    )
    def test_move_l_is_straight_under_every_profile(self, dry_run, profile) -> None:
        """The profile decides how a linear move is timed, not where it goes:
        every profile must keep the TCP on the start->end line. A profile
        that plans the move in joint space and only times it would bow."""
        try:
            assert dry_run.select_profile(profile) == 1
            assert dry_run.profile() == profile
            dry_run.teleport(park_deg())
            start = np.asarray(dry_run.pose())
            target = start.copy()
            target[1] += 50.0
            target[2] += 30.0

            result = _planned(dry_run, dry_run.move_l(target.tolist(), speed=0.5))
            assert result.error is None, f"{profile}: {result.error}"
            points = result.tcp_poses[:, :3] * 1000.0
            assert np.allclose(points[-1], target[:3], atol=0.5), profile
            bow = _line_deviation_mm(points)
            assert bow < 0.1, f"{profile}: TCP path bows by {bow:.3f} mm"
        finally:
            dry_run.select_profile("RUCKIG")

    def test_curved_moves_preview_the_shape_they_trace(self, dry_run) -> None:
        """``move_c`` must preview the arc through its via point, ``move_s`` the
        spline through every waypoint, and ``move_p`` the same waypoints with
        the corners rounded away — each ending on the last pose it was given."""
        dry_run.teleport(park_deg())
        base = np.asarray(dry_run.pose())
        via, end = _offset(base, (30.0, 0.0, 25.0)), _offset(base, (60.0, 0.0, 0.0))

        curve = _planned(dry_run, dry_run.move_c(via.tolist(), end.tolist(), speed=0.4))
        assert curve.error is None
        points = curve.tcp_poses[:, :3] * 1000.0
        centre, radius = _circle_through(base[:3], via[:3], end[:3])
        off_circle = np.abs(np.linalg.norm(points - centre, axis=1) - radius)
        assert off_circle.max() < 0.5, (
            f"arc leaves its circle by {off_circle.max():.3f} mm"
        )
        assert _closest(points, via[:3]) < 1.0, "the arc missed its via point"
        assert np.allclose(points[-1], end[:3], atol=0.5)

        dry_run.teleport(park_deg())
        waypoints = [
            _offset(base, delta).tolist()
            for delta in ((20.0, 0.0, 25.0), (40.0, 0.0, -15.0), (60.0, 0.0, 25.0))
        ]
        curved = _planned(dry_run, dry_run.move_s(waypoints, speed=0.4))
        assert curved.error is None
        spline = curved.tcp_poses[:, :3] * 1000.0
        for w in waypoints:
            assert _closest(spline, np.asarray(w[:3])) < 1.0, f"spline missed {w[:3]}"
        # A spline is not the polyline it interpolates: it bows off the chords.
        assert (
            _polyline_gap(spline, [base[:3]] + [np.asarray(w[:3]) for w in waypoints])
            > 2.0
        )

        dry_run.teleport(park_deg())
        process = _planned(dry_run, dry_run.move_p(waypoints, speed=0.4))
        assert process.error is None
        swept = process.tcp_poses[:, :3] * 1000.0
        # Auto-blended corners: the interior waypoints are rounded off (the
        # path passes near them, not through them), the ends are kept, and
        # cutting the corners makes the path shorter than the polyline.
        for w in waypoints[:-1]:
            miss = _closest(swept, np.asarray(w[:3]))
            assert 0.5 < miss < 25.0, f"corner {w[:3]} missed by {miss:.2f} mm"
        assert np.allclose(swept[-1], waypoints[-1][:3], atol=0.5)
        assert _length(swept) < _length(
            np.stack([base[:3]] + [np.asarray(w[:3]) for w in waypoints])
        )

    def test_blend_radius_folds_the_queue_into_one_motion(self, dry_run) -> None:
        """A move with ``r`` is held for the move behind it, exactly as the
        runtime's queue holds it: the two become ONE motion with a rounded
        corner that the arm never stops in. The head of the chain owns that
        motion in the record; the move that closed it (or ``flush()``) is
        folded into it and owns no rows."""
        dry_run.teleport(park_deg())
        base = np.asarray(dry_run.pose())
        corner, finish = (
            _offset(base, (50.0, 0.0, 0.0)),
            _offset(base, (50.0, 0.0, 40.0)),
        )

        sharp = [
            _planned(dry_run, dry_run.move_l(corner.tolist(), speed=0.4)),
            _planned(dry_run, dry_run.move_l(finish.tolist(), speed=0.4)),
        ]
        stopped = np.vstack([r.tcp_poses[:, :3] for r in sharp]) * 1000.0

        dry_run.teleport(park_deg())
        head = dry_run.move_l(corner.tolist(), speed=0.4, r=15.0)
        assert len(dry_run.queue()) == 1, "a corner move waits for its successor"
        tail = dry_run.move_l(finish.tolist(), speed=0.4)
        assert dry_run.queue() == [], "the move behind it closed the chain"
        blended = _planned(dry_run, head)
        assert blended.error is None
        assert _planned(dry_run, tail).rows == 0, "the head of a chain owns its motion"
        rounded = blended.tcp_poses[:, :3] * 1000.0

        miss = _closest(rounded, corner[:3])
        assert 1.0 < miss < 15.0, f"corner rounded by {miss:.2f} mm, radius was 15 mm"
        assert _closest(stopped, corner[:3]) < 0.5, (
            "the sharp pair must reach the corner"
        )
        assert np.allclose(rounded[-1], finish[:3], atol=0.5)
        assert _length(rounded) < _length(stopped) - 1.0
        # One motion, so the TCP sweeps through the corner instead of coming
        # to rest in it — which is exactly what the sharp pair does.  Compared
        # away from the start and stop ramps every motion has.
        blended_speeds = _tcp_speeds(blended)
        sharp_speeds = [_tcp_speeds(r) for r in sharp]
        edge = _ramp(blended_speeds)
        cruising = blended_speeds[edge:-edge].min()
        at_the_corner = min(
            sharp_speeds[0][_ramp(sharp_speeds[0]) :].min(),
            sharp_speeds[1][: -_ramp(sharp_speeds[1])].min(),
        )
        assert cruising > 0.1 * blended_speeds.max(), (
            f"the blended motion crawled to {cruising:.2f} mm/s mid-path"
        )
        assert at_the_corner < cruising, (
            "the un-blended pair is supposed to stop at the corner"
        )
        assert np.allclose(dry_run.angles(), np.degrees(blended.end_joints_rad))

        # A chain the program never closes is planned by flush(), which is
        # where the runtime's blend hold expires.
        dry_run.teleport(park_deg())
        head = dry_run.move_l(corner.tolist(), speed=0.4, r=15.0)
        assert len(dry_run.queue()) == 1
        dry_run.flush()
        trailing = _planned(dry_run, head)
        assert trailing.error is None
        assert np.allclose(trailing.tcp_poses[-1, :3] * 1000.0, corner[:3], atol=0.5)

    def test_blended_joint_moves_run_as_one_motion(self, dry_run) -> None:
        """Joint moves blend too: the corner zone is sized from the TCP distance
        the radius names, and the chain runs as one motion that never stops at
        the interior target — so it is quicker than the same moves run apart."""
        dry_run.teleport(park_deg())
        first = list(dry_run.angles())
        first[0] += 20.0
        second = list(first)
        second[1] -= 15.0

        apart = [
            _planned(dry_run, dry_run.move_j(first, speed=0.5)),
            _planned(dry_run, dry_run.move_j(second, speed=0.5)),
        ]
        separate = sum(r.duration for r in apart)

        dry_run.teleport(park_deg())
        head = dry_run.move_j(first, speed=0.5, r=25.0)
        assert len(dry_run.queue()) == 1
        assert _planned(dry_run, dry_run.move_j(second, speed=0.5)).rows == 0
        chain = _planned(dry_run, head)
        assert chain.error is None
        assert chain.duration < separate
        # The block's last row is the last kept sample, up to a stride short
        # of the target.
        np.testing.assert_allclose(np.degrees(chain.end_joints_rad), second, atol=0.05)
        # The corner is rounded in joint space: the chain passes close by the
        # interior target without ever reaching it.
        interior = np.abs(np.degrees(chain.joint_trajectory_rad) - first).max(axis=1)
        assert 0.1 < interior.min() < 5.0, (
            f"the chain came within {interior.min():.3f} deg of the interior target"
        )

    def test_a_wrist_roll_corner_stays_inside_the_commanded_envelope(
        self, dry_run
    ) -> None:
        """A blended chain must never drive a joint past the waypoints it was
        given.  The corner zone is sized from two independent TCP distances
        and a wrist roll moves no TCP at all — the TCP sits on J6's axis — so
        this chain's incoming fraction comes out zero while its outgoing one
        trims half the segment.  Anything that leaves that trimmed head
        unsampled hands TOPPRA a single interval as long as the trim, and its
        spline swings outside the envelope.  The runtime's own geometry
        carries the same guard (``crates/par6-motion/src/cart.rs``,
        ``blended_polyline_joint``)."""
        dry_run.teleport(park_deg())
        start = list(dry_run.angles())
        roll = list(start)
        roll[5] += 35.0
        swing = list(roll)
        swing[0] += 30.0

        head = dry_run.move_j(roll, speed=0.5, r=25.0)
        dry_run.move_j(swing, speed=0.5)
        chain = _planned(dry_run, head)
        assert chain.error is None

        corners = np.radians(np.stack([start, roll, swing]))
        low, high = corners.min(axis=0), corners.max(axis=0)
        traj = chain.joint_trajectory_rad
        excursion = max(
            float((low - traj.min(axis=0)).max()),
            float((traj.max(axis=0) - high).max()),
        )
        # Rounding a corner cuts inside the waypoints, never outside them;
        # what is left is the timing spline's own bow through them.
        assert excursion < 0.01, (
            f"the chain left the commanded envelope by {np.degrees(excursion):.2f} deg"
        )

    def test_refuses_what_the_runtime_refuses(self, dry_run) -> None:
        """Parameters and commands ``par6d`` rejects must be rejected here with
        the same code, so a preview never promises motion the arm will refuse."""
        dry_run.teleport(park_deg())
        # Geometry the runtime cannot turn into a path is a validation error,
        # never silently straightened into a line.
        base = np.asarray(dry_run.pose())
        with pytest.raises(RobotError) as collinear:
            dry_run.move_c(
                _offset(base, (20.0, 0.0, 0.0)).tolist(),
                _offset(base, (40.0, 0.0, 0.0)).tolist(),
                speed=1.0,
            )
        assert collinear.value.code == ErrorCode.COMM_VALIDATION_ERROR
        with pytest.raises(RobotError) as empty:
            dry_run.move_s([], speed=1.0)
        assert empty.value.code == ErrorCode.COMM_VALIDATION_ERROR

        with pytest.raises(RobotError) as tool:
            dry_run.select_tool("SSG48")
        assert tool.value.code == ErrorCode.COMM_VALIDATION_ERROR

        with pytest.raises(RobotError) as profile:
            dry_run.select_profile("BANG_BANG")
        assert profile.value.code == ErrorCode.SYS_PROFILE_INVALID

        # The live client refuses rel on a pose-target joint move
        # (MOVE_J_POSE is absolute on the wire) — a preview that quietly
        # planned the absolute move would validate a program the arm
        # then refuses with this very ValueError.
        with pytest.raises(ValueError, match="rel=True"):
            dry_run.move_j(pose=dry_run.pose(), rel=True, speed=1.0)
        # ``rel`` belongs to move_j/move_l, and a move takes only the
        # keywords its wait does: the live client refuses the rest with a
        # TypeError, and so must the preview of the same program.
        here = dry_run.pose()
        with pytest.raises(TypeError, match="rel"):
            dry_run.move_c(here, here, rel=True, speed=0.5)
        with pytest.raises(TypeError, match="rel"):
            dry_run.move_p([here, here], rel=True, speed=0.5)
        with pytest.raises(TypeError, match="bogus"):
            dry_run.move_l(here, speed=0.5, bogus=1)

        far = list(dry_run.angles())
        far[1] = math.degrees(_cfg.soft_limits_rad()[1, 1]) + 20.0
        with pytest.raises(RobotError) as outside:
            dry_run.move_j(far, speed=1.0)
        # A target outside the soft window is invalid input to the planner
        # (``planning_error``), the same class the runtime answers with.
        assert outside.value.code == ErrorCode.COMM_VALIDATION_ERROR

        unhomed = Robot().create_dry_run_client(
            initial_joints_deg=park_deg(), initial_homed=False
        )
        with pytest.raises(RobotError) as gate:
            unhomed.move_j(park_deg(), speed=1.0)
        assert gate.value.code == ErrorCode.MOTN_NOT_HOMED
        # Jogging stays available while un-homed, as it does on the runtime.
        unhomed.jog_j(0, 0.2, 0.2)
        assert _last(unhomed).duration > 0.0

    def test_the_preview_jogs_several_joints_at_once(self, dry_run) -> None:
        """A diagonal jog must preview as a diagonal.

        The preview refused any jog with more than one non-zero speed, so
        the two-axis gesture a pendant makes had no preview at all — while
        the engine underneath had been per-joint the whole time.
        """
        dry_run.teleport(park_deg())
        start = [math.radians(a) for a in dry_run.angles()]
        dry_run.jog_j(joints=[0, 3], speeds=[0.4, -0.4], duration=0.4)
        end = np.radians(dry_run.angles())
        assert end[0] > start[0] + 0.01, "J0 must have jogged forward"
        assert end[3] < start[3] - 0.01, "J3 must have jogged back"
        for j in (1, 2, 4, 5):
            assert abs(end[j] - start[j]) < 1e-9, (
                f"J{j} was never commanded and must not move"
            )

    def test_the_preview_refuses_the_inputs_the_wire_refuses(self, dry_run) -> None:
        """Values the codec rejects must be rejected before they are drawn.

        Each of these previously produced a confident trajectory the arm
        would never make: a jog past full scale, a watchdog longer than the
        runtime's ceiling, a non-finite speed, a short angle list numpy pads
        silently, and a teleport clamped into range instead of refused.
        """
        dry_run.teleport(park_deg())
        for kwargs in (
            {"joint": 0, "speed": 5.0, "duration": 0.5},
            {"joint": 0, "speed": 0.5, "duration": MAX_JOG_DURATION_S + 1.0},
            {"joint": 0, "speed": float("nan"), "duration": 0.5},
            {"joint": 0, "speed": float("inf"), "duration": 0.5},
        ):
            with pytest.raises(RobotError) as jog:
                dry_run.jog_j(**kwargs)
            assert jog.value.code == ErrorCode.COMM_VALIDATION_ERROR, kwargs

        # A wrong-length list is refused by the live client itself, before
        # any datagram, with ValueError — the preview raises the same.
        with pytest.raises(ValueError, match="requires"):
            dry_run.move_j([0.0, 0.0, 0.0], speed=1.0)
        # Timing out of range is refused as the live client refuses it, and
        # nothing is submitted.
        submitted = dry_run.program_length
        for timing in ({"speed": 0.0}, {"accel": 1.5}, {"duration": -1.0}):
            with pytest.raises(ValueError, match=next(iter(timing))):
                dry_run.move_j(dry_run.angles(), **timing)
        assert dry_run.program_length == submitted
        with pytest.raises(ValueError, match="requires"):
            dry_run.teleport([0.0, 0.0, 0.0])

        # Outside a joint's travel the runtime refuses rather than clamping,
        # because clamping lands the arm somewhere else and reports success.
        beyond = list(park_deg())
        beyond[0] = math.degrees(_cfg.config().hard_limits_rad()[0][1]) + 10.0
        before = dry_run.angles()
        with pytest.raises(RobotError) as clamped:
            dry_run.teleport(beyond)
        assert clamped.value.code == ErrorCode.COMM_VALIDATION_ERROR
        assert dry_run.angles() == pytest.approx(before, abs=1e-9)

        # The preview owns the readback even though it owns no pins: a
        # level set here has to show up where the runtime would put it,
        # or a program that reads its own outputs back behaves
        # differently against the preview than against the arm.
        n_in, n_out = (len(g) for g in _cfg.io_line_names())
        assert dry_run.io() == [0] * (n_in + n_out) + [1]
        dry_run.write_io(n_out - 1, 1)
        assert dry_run.io() == [0] * (n_in + n_out - 1) + [1, 1]
        dry_run.write_io(n_out - 1, 0)
        assert dry_run.io() == [0] * (n_in + n_out) + [1]

        # The live client bounds the port itself, with ValueError.
        with pytest.raises(ValueError, match="Output index"):
            dry_run.write_io(n_out, 1)
        with pytest.raises(ValueError, match="0 or 1"):
            dry_run.write_io(0, 2)

    def test_home_returns_to_park_once_the_arm_is_referenced(self) -> None:
        """HOME is two commands wearing one name, and the preview has to know
        which one it is drawing.

        Un-referenced it is the seek, whose own time is the arm's: the
        record shows it landing wherever the configured sequence's
        ``move_to`` steps leave the arm, then the planned return to the park
        pose the runtime runs once the references are established.
        Referenced it is only that return — which is what makes a Home
        button cost seconds rather than a full seek.
        """
        robot = Robot()
        cold = robot.create_dry_run_client(
            initial_joints_deg=park_deg(), initial_homed=False
        )
        seek = _planned(cold, cold.home())
        assert seek.rows > 1, "the seek's landing, then the return to park"
        np.testing.assert_allclose(
            seek.joint_trajectory_rad[0], _cfg.homing_ready_pose_rad(), atol=1e-6
        )
        assert seek.end_joints_rad == pytest.approx(robot.joints.home.rad, abs=1e-3)

        warm = robot.create_dry_run_client(
            initial_joints_deg=np.degrees(_cfg.homing_ready_pose_rad()).tolist()
        )
        ret = _planned(warm, warm.home())
        assert ret.duration > 0.0, "a referenced HOME is a planned move, not a jump"
        assert ret.end_joints_rad == pytest.approx(robot.joints.home.rad, abs=1e-3)
        assert ret.tcp_poses.shape[0] > 1, "a planned move draws a path"

    def test_the_preview_answers_the_queries_a_program_reads_back(self) -> None:
        """A script that reads state between moves must not hit AttributeError,
        and the tool's jaw state must follow what the program told it to do."""
        client = Robot().create_dry_run_client(initial_joints_deg=park_deg())

        assert len(client.io()) == IO_SLOTS
        assert client.queue() == []
        # The mirror must report what the ENGINE plans with from the
        # first preview: the runtime's own startup profile
        # (par6d::planner::DEFAULT_PROFILE). A mirror that disagreed with
        # the engine timed every pre-sync preview with the wrong profile.
        assert client.profile() == "TOPPRA"

        # STATUS builds its pose from the 4x4 the engine returns; pose()
        # reads the engine's own xyzrpy. The two must describe one arm.
        assert client.status().pose[3:12:4] == pytest.approx(
            client.pose()[:3], abs=1e-6
        )

        # The runtime refuses a jaw move on an uncalibrated gripper, and so
        # does the preview; after the calibrate the jaw state follows the
        # program.
        with pytest.raises(RobotError) as uncalibrated:
            client.tool.close()
        assert uncalibrated.value.code == ErrorCode.COMM_VALIDATION_ERROR
        assert "calibrat" in str(uncalibrated.value).lower()
        calibrated = _planned(client, client.tool.calibrate())
        assert calibrated.duration > 0.0, (
            "a calibrate holds the arm for the driver's settle"
        )
        assert client.tool.is_open(client.tool.status().positions[0])
        client.tool.close()
        assert not client.tool.is_open(client.tool.status().positions[0])
        assert client.tool.status().engaged
        client.tool.open()
        assert client.tool.is_open(client.tool.status().positions[0])
        assert client.tool.status().key == client.active_tool_key
        assert client.tool.key == client.active_tool_key
        with pytest.raises(AttributeError):
            client.tool.bogus_verb


class TestLiveParity:
    """What a program sees offline is what the arm would do: the dry run
    answers with the live client's methods, exception classes and
    refusals, and its timeline carries every command's time."""

    def test_a_state_only_command_keeps_a_held_chains_motion(self, dry_run) -> None:
        """A checkpoint (or any command with no path) closes the blend hold
        as the runtime's queue does: the held move keeps its motion under
        its own block, runs to its target with nothing to round into, and
        the next move starts where it ended."""
        dry_run.teleport(park_deg())
        base = np.asarray(dry_run.pose())
        corner = _offset(base, (50.0, 0.0, 0.0))
        finish = _offset(base, (50.0, 0.0, 40.0))
        head = dry_run.move_l(corner.tolist(), speed=0.4, r=15.0)
        mark = dry_run.checkpoint("corner")
        assert mark == head + 1 and dry_run.queue() == []
        onward = dry_run.move_l(finish.tolist(), speed=0.4)
        record = dry_run.plan()
        first, second = _Block(record, head), _Block(record, onward)
        assert first.error is None and second.error is None
        assert _Block(record, mark).rows == 0, "a checkpoint moves nothing"
        path = first.tcp_poses[:, :3] * 1000.0
        assert np.allclose(path[0], base[:3], atol=2.0), (
            f"the held move keeps its motion: {path[0]}"
        )
        assert np.allclose(path[-1], corner[:3], atol=0.5)
        assert np.allclose(second.tcp_poses[-1, :3] * 1000.0, finish[:3], atol=0.5)

    def test_the_blend_hold_fills_at_the_runtimes_lookahead(self, dry_run) -> None:
        """The runtime's queue plans a chain once the blend lookahead is
        full; a hold that grew without bound would fold a whole program
        into one motion the arm runs in several."""
        dry_run.teleport(park_deg())
        cap = dry_run._preview.blend_lookahead()
        start = list(dry_run.angles())
        indices = []
        for i in range(cap):
            target = list(start)
            target[0] += 2.0 * ((i % 2) + 1)
            indices.append(dry_run.move_j(target, speed=0.5, r=5.0))
            assert len(dry_run.queue()) == (i + 1) % cap, (
                "held until the lookahead fills"
            )
        record = dry_run.plan()
        assert _Block(record, indices[0]).rows > 0, (
            "the move that fills the hold runs the chain"
        )
        assert any(_Block(record, i).rows == 0 for i in indices[1:]), (
            "a chain folds the moves behind its head"
        )

    def test_delays_and_tool_actions_carry_their_duration(self, dry_run) -> None:
        """A delay holds the arm for its seconds and a calibration for the
        runtime's minimum wait; both are time on the program's timeline."""
        dry_run.teleport(park_deg())
        with pytest.raises(ValueError, match="positive"):
            dry_run.delay(0.0)
        held = _planned(dry_run, dry_run.delay(1.5))
        assert held.duration == pytest.approx(1.5, abs=2 * dry_run.plan().row_dt_s)
        assert held.rows > 1 and np.ptp(held.tcp_poses, axis=0).max() == 0, (
            "a delay holds one pose for its rows"
        )

        calibration = _planned(dry_run, dry_run.tool.calibrate())
        assert calibration.duration >= 2.0, "the runtime holds a calibration"
        assert _planned(dry_run, dry_run.tool.stop()).duration == 0.0

    def test_tool_verbs_send_the_live_wire_actions(self) -> None:
        """``release`` is the wire's ``idle`` and ``stop`` is ``stop`` — a
        preview that sent the method name would refuse what the arm
        accepts; and a jaw move on an uncalibrated gripper is refused
        exactly as the runtime refuses it."""
        client = Robot().create_dry_run_client(
            initial_joints_deg=park_deg(), initial_gripper_calibrated=False
        )
        with pytest.raises(RobotError) as uncalibrated:
            client.tool.close()
        assert uncalibrated.value.code == ErrorCode.COMM_VALIDATION_ERROR
        assert "calibrat" in uncalibrated.value.cause.lower()
        assert client.tool.is_open(), "a refused move leaves the jaws where they were"

        assert _planned(client, client.tool.calibrate()).duration >= 2.0
        close = client.tool.close()
        assert _planned(client, close).duration > 0.0, (
            "a jaw move holds the arm for the jaws' travel"
        )
        assert not client.tool.is_open()
        # The wire carries current as a fraction of the tool's range, like
        # speed; a move that names none grips at half the range.
        assert client._program[close]["params"] == [1.0, 0.5, 0.5]
        firm = client.tool.set_position(0.3, speed=0.8, current=0.25)
        assert client._program[firm]["params"] == [0.3, 0.8, 0.25]
        for current in (-0.1, 1.5, math.nan, math.inf):
            with pytest.raises(RobotError, match="current") as bad_current:
                client.tool.close(current=current)
            assert bad_current.value.code == ErrorCode.COMM_VALIDATION_ERROR
        assert _planned(client, client.tool.stop()).duration == 0.0
        assert _planned(client, client.tool.release()).duration == 0.0
        with pytest.raises(RobotError) as past_stroke:
            client.tool.set_position(1.5)
        assert past_stroke.value.code == ErrorCode.COMM_VALIDATION_ERROR
        with pytest.raises(RobotError) as unknown:
            client.tool_action(client.active_tool_key, "grab")
        assert unknown.value.code == ErrorCode.COMM_VALIDATION_ERROR

    def test_servo_speed_is_refused_where_the_wire_refuses_it(self, dry_run) -> None:
        """The live client passes the fraction through and the wire refuses
        0 and anything past 1; a preview that rewrote them validated a
        stream the arm rejects."""
        dry_run.teleport(park_deg())
        target = list(dry_run.angles())
        target[0] += 5.0
        for speed in (0.0, 1.5):
            with pytest.raises(RobotError) as refused:
                dry_run.servo_j(target, speed=speed)
            assert refused.value.code == ErrorCode.COMM_VALIDATION_ERROR, speed
        dry_run.servo_j(target, speed=0.5)
        assert _last(dry_run).duration > 0.0

    def test_payload_is_validated_and_read_back(self, dry_run) -> None:
        with pytest.raises(RobotError) as negative:
            dry_run.set_payload(-1.0)
        assert negative.value.code == ErrorCode.COMM_VALIDATION_ERROR
        assert dry_run.set_payload(0.5, com=(0.0, 0.0, 0.05)) == 1
        payload = dry_run.payload()
        assert payload.mass == pytest.approx(0.5)
        assert payload.com == pytest.approx((0.0, 0.0, 0.05))
        assert len(payload.inertia) == 6
        assert dry_run.set_payload(0.0) == 1

    def test_jog_l_previews_through_the_runtime_kinematics(self, dry_run) -> None:
        """A +X world jog moves the TCP along +X through the runtime's own
        twist integration; the axis vocabulary is refused like the live
        client refuses it."""
        # Clear of the wrist singularity park folds J5 into.
        dry_run.teleport([0.0, -60.0, 150.0, 0.0, 45.0, 180.0])
        start = np.asarray(dry_run.pose())
        dry_run.jog_l("WRF", "X", speed=1.0, duration=0.5)
        jog = _last(dry_run)
        end = np.asarray(dry_run.pose())
        assert end[0] - start[0] > 20.0, "half a second of full-scale +X must travel"
        assert abs(end[1] - start[1]) < 3.0 and abs(end[2] - start[2]) < 3.0
        # The watchdog releases the tool rather than stopping it dead, so
        # the jog occupies the arm for its window PLUS the ramp down —
        # which covers real ground, and a preview that stopped at the
        # window would under-predict where the runtime leaves the arm.
        # What is pinned here is the shape of that: longer than the
        # window, bounded, and finished at rest.
        assert jog.duration > 0.5, f"the window alone is 0.5 s, got {jog.duration}"
        assert jog.duration < 1.5, f"the ramp should be short, got {jog.duration}"
        traj = jog.joint_trajectory_rad
        assert traj.shape[1] == NUM_JOINTS
        # At rest means the arm has stopped, not that two rows match to
        # machine precision: the limiter settles to its own tolerance.
        # The runtime calls a joint stopped under 0.05 rad/s, so hold the
        # tail of the ramp well inside that.
        rest_rad_s = np.abs(traj[-1] - traj[-2]).max() / dry_run.plan().row_dt_s
        assert rest_rad_s < 0.01, (
            f"the previewed jog must end at rest, got {rest_rad_s} rad/s"
        )
        with pytest.raises(ValueError, match="unknown axis"):
            dry_run.jog_l("WRF", "Q", speed=0.5, duration=0.2)
        with pytest.raises(ValueError, match="axes and"):
            dry_run.jog_l("WRF", axes=["X", "Y"], speeds_list=[0.5], duration=0.2)

    def test_the_queries_a_live_program_reads_have_preview_answers(self) -> None:
        client = Robot().create_dry_run_client(initial_joints_deg=park_deg())
        assert client.ping().hardware_connected is False
        assert client.tools().tool == client.active_tool_key
        assert client.active_tool_key in client.tools().available
        assert client.activity().state is client.activity().state
        assert client.reachable().joint_en == [1] * NUM_JOINTS
        assert client.queue_state().queue == []
        assert client.loop_stats() is None
        assert client.reset_loop_stats() == 1
        assert client.wait_status(lambda s: s.homed) is True
        assert client.wait_status(lambda s: s.last_checkpoint == "x") is False
        client.checkpoint("x")
        assert client.wait_status(lambda s: s.last_checkpoint == "x") is True
        assert [s.homed for s in client.stream_status()] == [True]

        info = client.config_info()
        assert info["tick_dt_s"] == pytest.approx(client._dt)
        assert len(info["joints"]) == NUM_JOINTS
        bundle = client.config_bundle()
        assert bundle["robot_filename"].endswith(".toml")
        assert bundle["fingerprint"] == info["fingerprint"]
        assert "[robot]" in bundle["robot_toml"]

        # TRF answers as the runtime does: the world seen from the tool,
        # the inverse of the TCP pose.
        T = np.asarray(client.status().pose, dtype=np.float64).reshape(4, 4)
        world_in_tool = np.linalg.inv(T)
        assert client.pose(frame="TRF")[:3] == pytest.approx(
            world_in_tool[:3, 3].tolist(), abs=1e-6
        )
        assert client.pose(frame="WRF") == pytest.approx(client.pose(), abs=1e-9)


class TestProgramWorkflow:
    def test_a_program_previews_as_one_continuous_timeline(self) -> None:
        """Drive a whole program the way the editor does and check the results
        chain: every segment starts where the previous one ended, the tool
        action holds position, and home ends at the park pose.

        Un-referenced to start with, because that is the state an editor
        opens on and it is why a program's first line is ``home()``. Seeding
        a configuration instead would have to seed a REACHABLE one: all-zeros
        is outside joints 1 and 2's travel, so no plan out of it is meaningful.
        """
        client = Robot().create_dry_run_client(
            initial_joints_deg=[0.0] * 6, initial_homed=False
        )
        program = [client.home()]
        np.testing.assert_allclose(
            _planned(client, program[-1]).end_joints_rad,
            _cfg.config().park_pose_rad(),
            atol=1e-4,
        )

        above = np.asarray(client.pose())
        above[2] += 30.0
        program.append(client.move_l(above.tolist(), speed=0.4))
        program.append(client.tool.calibrate())
        program.append(client.tool.close())
        joints = list(client.angles())
        joints[0] -= 15.0
        program.append(client.move_j(joints, speed=0.6))
        record = client.plan()
        assert client.queue() == []
        results = [_Block(record, index) for index in program]

        assert all(r.error is None for r in results)
        for previous, following in zip(results, results[1:]):
            np.testing.assert_allclose(
                following.tcp_poses[0][:3], previous.tcp_poses[-1][:3], atol=2e-3
            )
        # The tool actions hold the arm still for their time.
        for held in (results[2], results[3]):
            assert held.rows > 0 and np.ptp(held.tcp_poses, axis=0).max() == 0
            np.testing.assert_allclose(
                held.end_joints_rad, results[1].end_joints_rad, atol=1e-3
            )
        # The record keeps every stride-th sample, so its last row is up to
        # one row short of the landing the virtual arm is placed on — a
        # profile's last stride, at rest by then.
        assert client.angles() == pytest.approx(
            np.degrees(results[-1].end_joints_rad), abs=0.05
        )
        assert sum(r.duration for r in results) > 0.0


#: An open posture the shapes below fit in: extended, clear of the collision
#: gate the runtime enforces, and away from the wrist singularity the seeded
#: IK chain cannot be driven through. Same posture as the runtime's own
#: curved-move tests (ffi_kinematics CURVE_START_DEG): straight-line room is
#: IK-verified in every axis direction and along the diagonals from here.
_OPEN_POSE_DEG = [-125.0, -80.0, 175.0, 0.0, -40.0, 180.0]

#: The shapes the comparison traces, as millimetre offsets from wherever the
#: arm is standing.
_ARC = ((25.0, 0.0, 20.0), (50.0, 0.0, 0.0))
#: A full circle: the via is diametrically opposite and the end comes back to
#: the start — a fraction of a millimetre off it, which is what a client that
#: hands its own measured pose back as the end actually sends.  Both sides
#: must read that as one lap and not as the nudge between the two points.
_CIRCLE = ((50.0, 0.0, 0.0), (0.0, 0.3, 0.0))
_CURVE = ((20.0, 0.0, 20.0), (40.0, 0.0, -15.0), (60.0, 0.0, 20.0))
_CHAIN = ((35.0, 0.0, 0.0), (35.0, 0.0, 30.0))
_CHAIN_R_MM = 15.0
_CASE_SPEED = 0.05

#: The RT tick and STATUS rate this capture runs at.  The rest of the suite
#: ticks at 20 Hz to keep CI light, which samples one of these paths a dozen
#: times — too coarse for a millimetre comparison, since the polyline
#: through those samples cuts every corner it spans.  The packaged config
#: documents ``status_rate_hz`` as the knob to raise for capture work, so
#: this test raises the tick and the broadcast together and reads one frame
#: per tick.
_CAPTURE_DT_S = 0.008
_CAPTURE_STATUS_HZ = 125


def _capture_rates(toml: str) -> str:
    """Re-tick the daemon's config for capture, checking the patch points."""
    patched = toml.replace(
        f"tick_dt_s = {TICK_DT_S}", f"tick_dt_s = {_CAPTURE_DT_S}"
    ).replace(
        f"status_rate_hz = {STATUS_RATE_HZ}", f"status_rate_hz = {_CAPTURE_STATUS_HZ}"
    )
    if patched == toml:
        raise RuntimeError("PAR6.toml capture patch points missing")
    return patched


#: How far the path the runtime DROVE may sit from the previewed one [mm].
#: The preview predicts the COMMANDED trajectory while STATUS reports where
#: the arm actually went, so this budget covers the sim plant's own
#: departure from its command as well as the chord error of comparing two
#: sampled paths.
#:
#: Most of that departure is tracking lag, which ``_CASE_SPEED`` keeps
#: small and which moves a sample ALONG the path rather than off it — the
#: comparison is geometric for exactly that reason.  What is left is a
#: transient where acceleration is highest, at the start and the stop:
#: measured through the dry run's own ``commanded`` column, the plant is
#: 4.6 mm off its command at the worst row of an arc at ``_CASE_SPEED``
#: and 0.6 mm on average, and slowing to a fortieth of full rate only
#: takes the worst row to 4.6 from 6.0 — it is the servo's response to a
#: corner, not something a slower move removes.  The geometric gap this
#: budgets comes out at 4.1–5.2 mm across these five cases.
#:
#: A geometry, sampling or corner-rounding difference is far larger: see
#: ``_MIN_BOW_MM``, the scale of the shapes themselves.
_PATH_GAP_MM = 8.0

#: How far a previewed path must depart from the straight line between its
#: own endpoints [mm], so that matching it means something.  Absolute
#: rather than a multiple of ``_PATH_GAP_MM``: the budget above is the
#: plant's, and widening it for a heavier plant must not quietly weaken
#: what counts as a shape.
_MIN_BOW_MM = 15.0

#: How far the endpoint may sit from the predicted one [mm].  The arm
#: settles onto its last commanded pose, so this is tighter than the
#: path budget above by the size of the transient it excludes.
_END_GAP_MM = 2.5

#: How far the captured motion's duration may sit from the predicted one
#: [s].  The window's ends are where MEASURED motion becomes detectable, and
#: the plant leaves the start and reaches the end asymptotically, so a
#: handful of ticks at each end fall under that threshold — about 1.5 % of
#: these cases' durations, which a timing difference would dwarf.
_DURATION_GAP_S = 0.25


class _ExecutedPath:
    """The joint path the runtime drove, captured off the STATUS broadcast.

    The CI config ticks the RT and the broadcast at the same rate, so this
    is one row per RT tick without raising anything — the packaged config
    documents ``status_rate_hz`` as the knob to raise for capture work.

    What arrives is the MEASURED position: the sim plant's response to the
    planner's stream, which carries its tracking lag. Lag moves a sample
    ALONG the path rather than off it, which is why the comparison this
    feeds is geometric.
    """

    def __init__(self, client) -> None:
        self._client = client
        self._rows: dict[int, np.ndarray] = {}
        self._task: asyncio.Task | None = None

    async def __aenter__(self) -> "_ExecutedPath":
        self._task = asyncio.create_task(self._collect())
        return self

    async def __aexit__(self, *exc: object) -> None:
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._task

    async def _collect(self) -> None:
        async for status in self._client.stream_status_shared():
            self._rows[int(status.seq)] = np.radians(
                np.asarray(status.angles[:NUM_JOINTS], dtype=np.float64)
            )

    def drain(self) -> None:
        """Discard everything captured so far (case isolation)."""
        self._rows.clear()

    def executed(self) -> np.ndarray:
        """The joint path since the last drain, ``(N, 6)`` radians.

        Trimmed to the motion's own frames: between two commands the arm
        rests where the last plan left it, so the first frame that differs
        is the point the motion starts FROM, not a sample of it. The
        preview's trajectory starts one sample in for the same reason.
        """
        path = np.stack([self._rows[seq] for seq in sorted(self._rows)])
        moving = np.flatnonzero(np.abs(np.diff(path, axis=0)).max(axis=1) > 1e-9)
        assert moving.size > 1, "the runtime drove no motion"
        return path[moving[0] + 1 : moving[-1] + 2]


def _shape_from(pose: list[float], deltas) -> list[list[float]]:
    """The shape's waypoints, anchored on *pose* and holding its orientation.

    Each side anchors on the pose IT reports, so the comparison is of the
    motion, not of two TCP frames: a pure-translation shape with a held
    orientation moves the whole arm the same way whatever point on the tool
    the frame is measured at.
    """
    base = np.asarray(pose, dtype=np.float64)
    return [_offset(base, delta).tolist() for delta in deltas]


def _preview_case(preview, case: str) -> list[_Block]:
    """Plan one case offline; returns the block of every command it queued."""
    if case in ("arc", "circle"):
        via, end = _shape_from(preview.pose(), _ARC if case == "arc" else _CIRCLE)
        queued = [preview.move_c(via, end, speed=_CASE_SPEED)]
    elif case == "spline":
        queued = [
            preview.move_s(_shape_from(preview.pose(), _CURVE), speed=_CASE_SPEED)
        ]
    elif case == "process":
        queued = [
            preview.move_p(_shape_from(preview.pose(), _CURVE), speed=_CASE_SPEED)
        ]
    else:
        corner, finish = _shape_from(preview.pose(), _CHAIN)
        queued = [preview.move_l(corner, speed=_CASE_SPEED, r=_CHAIN_R_MM)]
        assert len(preview.queue()) == 1, "a blended move is held for the one behind it"
        queued.append(preview.move_l(finish, speed=_CASE_SPEED))
    record = preview.plan()
    return [_Block(record, index) for index in queued]


async def _queue_case(client, case: str) -> list[int]:
    """Queue one case on the runtime; returns the command indexes."""
    pose = await client.pose()
    assert pose is not None
    if case in ("arc", "circle"):
        via, end = _shape_from(pose, _ARC if case == "arc" else _CIRCLE)
        return [await client.move_c(via, end, speed=_CASE_SPEED)]
    if case == "spline":
        return [await client.move_s(_shape_from(pose, _CURVE), speed=_CASE_SPEED)]
    if case == "process":
        return [await client.move_p(_shape_from(pose, _CURVE), speed=_CASE_SPEED)]
    corner, finish = _shape_from(pose, _CHAIN)
    return [
        await client.move_l(corner, speed=_CASE_SPEED, r=_CHAIN_R_MM),
        await client.move_l(finish, speed=_CASE_SPEED),
    ]


async def _run_case(client, case: str) -> tuple[list[int], list[float]]:
    """Queue one case on the runtime and wait it out.

    Returns the command indexes and when each of them completed, in seconds
    after the first was queued — the window a client observes, which is the
    plan's execution plus dispatch.
    """
    started = time.monotonic()
    indexes = await _queue_case(client, case)
    assert all(index >= 0 for index in indexes)
    waits = [
        asyncio.create_task(client.wait_command(index, timeout=90.0))
        for index in indexes
    ]
    finished = []
    for wait in waits:
        assert await wait is True
        finished.append(time.monotonic() - started)
    return indexes, finished


@pytest.mark.e2e
@requires_par6d
@pytest.mark.timeout(600)
async def test_curved_and_blended_previews_match_the_runtime(tmp_path) -> None:
    """Every curved and blended move must preview the motion par6d runs.

    An arc, a full circle, a spline, a process move and a blended pair of
    straight moves are each planned offline and queued on a live
    ``par6d --sim``, and the joint path the runtime drove is read back off
    its STATUS broadcast.  A preview whose geometry, sampling, corner
    rounding or timing differed from the runtime's would trace a different
    path or fill a different number of ticks with it.

    Both sides anchor their shape on the pose they themselves report and hold
    that orientation, so the two describe the same rigid motion whatever point
    on the tool each measures it at, and both paths are compared through the
    same kinematics.

    What STATUS reports is where the arm WENT rather than what the planner
    commanded, so the cases run slowly enough that the sim plant's tracking
    lag stays small, and the comparison is geometric so that what lag remains
    moves a sample along the path rather than off it — see
    :data:`_PATH_GAP_MM`.

    The blended pair also pins the completion semantics: two commands, ONE
    motion, both completing at the same instant, with the high-water mark
    ending on the last of them.
    """
    daemon = LiveDaemon.start(tmp_path, config_patch=_capture_rates)
    robot = Robot()
    try:
        async with daemon.client() as client, _ExecutedPath(client) as stream:
            assert await client.wait_status(lambda s: s.link_ok == 1, timeout=20.0)
            assert await client.reset() == 1
            assert await client.set_completion_policy(CompletionPolicy.COMMANDED) == 1

            for case in ("arc", "circle", "spline", "process", "chain"):
                # Both sides plan from the configuration the arm is measured
                # in, so the shapes are anchored on the same place.
                await teleport_to(client, _OPEN_POSE_DEG)
                assert await client.wait_status(
                    lambda s: float(np.abs(np.asarray(s.speeds)).max()) < 1.0,
                    timeout=20.0,
                )
                live_start = await client.angles()
                assert live_start is not None
                # The same `Robot` the anchor FK comes from: it caches the
                # daemon's config bundle, so a fresh one per case re-pings
                # and re-materialises it five times over.
                preview = robot.create_dry_run_client(initial_joints_deg=live_start)
                results = _preview_case(preview, case)
                assert all(r.error is None for r in results), (
                    f"{case}: the preview refused it: {[r.error for r in results]}"
                )
                drawn = [r for r in results if r.rows]
                anchor = robot.fk_batch(np.radians([live_start]))[:, :3] * 1000.0
                predicted = np.vstack(
                    [anchor, np.vstack([r.tcp_poses[:, :3] for r in drawn]) * 1000.0]
                )
                predicted_duration = sum(r.duration for r in results)

                stream.drain()
                indexes, finished = await _run_case(client, case)
                driven = stream.executed()
                executed = np.vstack([anchor, robot.fk_batch(driven)[:, :3] * 1000.0])

                # The shape has to be a shape: a straight line between the
                # same endpoints would sit far outside the gap budget, so the
                # comparison below has teeth.
                bow = max(
                    _closest(np.stack([predicted[0], predicted[-1]]), p)
                    for p in predicted
                )
                assert bow > _MIN_BOW_MM, (
                    f"{case}: previewed path is nearly straight ({bow:.2f} mm "
                    "from the chord between its own endpoints)"
                )

                gap = max(
                    max(_closest(predicted, p) for p in executed),
                    max(_closest(executed, p) for p in predicted),
                )
                assert gap < _PATH_GAP_MM, (
                    f"{case}: the runtime drove a path {gap:.2f} mm off the "
                    "previewed one"
                )
                assert np.allclose(executed[-1], predicted[-1], atol=_END_GAP_MM), (
                    f"{case}: the runtime finished at {executed[-1]}, preview "
                    f"predicted {predicted[-1]}"
                )
                # The runtime fills whole RT ticks with the motion the
                # preview timed, and the capture reads one frame per tick.
                driven_s = driven.shape[0] * _CAPTURE_DT_S
                assert abs(driven_s - predicted_duration) <= _DURATION_GAP_S, (
                    f"{case}: the runtime executed {driven_s:.3f}s of motion, "
                    f"preview predicted {predicted_duration:.3f}s"
                )
                assert (
                    predicted_duration - 0.5 <= finished[-1] <= predicted_duration + 1.5
                ), (
                    f"{case}: runtime took {finished[-1]:.3f}s end to end, preview "
                    f"predicted {predicted_duration:.3f}s"
                )
                if case == "chain":
                    assert len(drawn) == 1, "the chain is one motion, not two"
                    assert finished[1] - finished[0] < 0.3, (
                        "a blended motion completes every command it consumed at "
                        f"the same instant, not {finished[1] - finished[0]:.3f}s apart"
                    )
                    assert await client.wait_status(
                        lambda s: s.completed_index == indexes[-1], timeout=5.0
                    ), "the high-water mark must end on the last command consumed"

    finally:
        daemon.stop()


def test_a_retune_previews_from_the_same_call_that_runs_live(
    dry_run: DryRunRobotClient,
) -> None:
    """The point of a dry run is that the script it accepts is the script
    the arm accepts.

    `voltage_limit_mv` has a default on the live client, so a caller who
    leaves it out sends a complete frame there. The preview took
    `**gains` and forwarded only what it was handed, so the same call
    reached the wire converter a field short and died — the preview
    refusing a program that runs.
    """
    j = _cfg.config().joints()[2]
    tune = dict(
        kpp=j["gains"]["kpp"],
        kpv=j["gains"]["kpv"],
        kiv=j["gains"]["kiv"],
        kpiq=j["gains"]["kpiq"],
        kiiq=j["gains"]["kiiq"],
        kp=j["gains"]["kp"],
        kd=j["gains"]["kd"],
        ilim_ma=j["ilim_ma"],
        velocity_limit_ticks_s=j["velocity_limit_ticks_s"],
    )
    # No voltage_limit_mv: exactly the call the live client completes.
    assert dry_run.set_pid_gains(j["node_id"], **tune) >= 0

    # And a node the config does not declare is refused here too, so the
    # preview catches the typo before the arm does.
    with pytest.raises(RobotError):
        dry_run.set_pid_gains(15, **tune)


def test_a_payload_estimate_previews_the_wrist_swing_and_measures_nothing(
    dry_run: DryRunRobotClient,
) -> None:
    """The ABC's own example — `found = rbt.estimate_payload()` — is a
    valid program, and a valid program must preview. A dry run has no
    torque to read, so the estimate is empty; what it previews is the
    motion: the wrist swing, planned against the same keep-outs, ending
    back where it started.
    """
    dry_run.set_payload(1.2, com=(0.0, 0.01, 0.05))
    start = list(dry_run.angles())
    assert dry_run.payload().mass == pytest.approx(1.2)

    found = dry_run.estimate_payload()
    assert found.poses >= 3, "the wrist must have somewhere to swing from park"
    assert found.mass == 0.0 and found.determined == (0.0, 0.0, 0.0, 0.0)
    assert dry_run.payload().mass == pytest.approx(1.2), (
        "an estimate that measured nothing must leave the declared payload standing"
    )
    assert dry_run.angles() == pytest.approx(start, abs=1e-6), (
        "the swing must end where the pick left the arm"
    )


def test_plan_and_simulate_describe_the_same_program() -> None:
    """The commanded record and the predicted record are one program on one
    row axis: block ``i`` of either is command ``i``, drawn the same way,
    and the gap between them is the following error the plan cannot know."""
    client = Robot().create_dry_run_client(initial_joints_deg=park_deg())
    target = park_deg()
    target[0] += 12.0
    above = np.asarray(client.pose())
    above[2] += 20.0
    program = [
        client.move_j(target, speed=0.5),
        client.delay(0.5),
        client.move_l(above.tolist(), speed=0.4),
        client.checkpoint("there"),
    ]
    assert program == [0, 1, 2, 3]
    assert all(client.wait_command(index) for index in program)

    commanded = client.plan()
    predicted = client.simulate()
    for record in (commanded, predicted):
        assert [b.command for b in record.blocks] == program
        assert [b.move_type for b in record.blocks] == [
            command_table()["move_j"].move_type,
            None,
            command_table()["move_l"].move_type,
            None,
        ]
        assert record.stop == "completed"
        # The delay is on both timelines, holding.
        assert record.blocks[1].rows == pytest.approx(0.5 / record.row_dt_s, abs=3)
    assert commanded.row_dt_s == predicted.row_dt_s
    assert not commanded.channels, "a plan has no plant to read a setpoint off"
    assert "setpoint_rad" in predicted.channels
    assert commanded.digest != predicted.digest

    assert float(np.max(following_error(commanded, commanded))) == 0.0
    gap = following_error(commanded, predicted)
    assert gap.shape == (predicted.rows,)
    worst = float(np.max(gap))
    assert 0.0 < worst < 0.1, f"the arm is not following its commands: {worst}"

    # A later command extends the commanded record; a refused one keeps its
    # place in it, with the refusal, and never counts as complete.
    client.delay(0.25)
    longer = client.plan()
    assert len(longer.blocks) == 5 and longer.rows > commanded.rows
    far = park_deg()
    far[1] = math.degrees(_cfg.soft_limits_rad()[1, 1]) + 20.0
    with pytest.raises(RobotError):
        client.move_j(far, speed=1.0)
    assert client.wait_command(5) is False
    failed = client.plan()
    assert failed.stop == "failed" and failed.blocks[5].error is not None
    assert failed.blocks[5].rows == 0


_TABLE_ARGS: dict[str, tuple] = {
    "move_j": ([10.0, -80.0, 160.0, 5.0, -10.0, 170.0],),
    "move_l": (None,),
    "move_c": (None, None),
    "move_s": (None,),
    "move_p": (None,),
    "servo_j": ([10.0, -80.0, 160.0, 5.0, -10.0, 170.0],),
    "servo_l": (None,),
    "jog_j": (0, 0.5, 0.1),
    "jog_l": ("WRF", "X", 0.5, 0.1),
    "estimate_payload": (),
    "home": (),
    "checkpoint": ("mark",),
    "delay": (0.1,),
    "write_io": (0, 1),
    "tool_action": ("<fitted>", "calibrate"),
    "reset": (),
    "reset_state": (),
    "simulator": (True,),
    "teleport": (None,),
    "freedrive": (False,),
    "set_shapes": ([],),
    "select_profile": ("RUCKIG",),
    "select_tool": ("<fitted>",),
    "set_tcp_offset": (0.0, 0.0, 0.0),
    "set_tcp_transform": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "set_payload": (0.1,),
    "stop": (),
    "estop": (),
    "pause": (),
    "resume": (),
    "set_execution_speed": (0.5,),
}
_TABLE_KWARGS: dict[str, dict] = {
    n: {"speed": 0.3} for n in ("move_j", "move_l", "move_c", "move_s", "move_p")
}


def test_every_table_command_answers_with_the_kind_it_declares() -> None:
    """The command table on ``RobotClient`` classifies every command; this
    preview answers each kind the way a program can rely on.

    Queued work answers with its index in the program — the block it owns
    in the commanded and predicted records, the index the ABC promises a
    program may ``wait_command`` — and the block is drawn the way the
    table says. A system or control command has no block of its own, so it
    answers with the live client's own ``1``/``0``/negative code:
    ``if rbt.stop() < 0`` reads the same offline and on the arm.
    """
    client = Robot().create_dry_run_client(initial_joints_deg=park_deg())
    park = park_deg()
    base = np.asarray(client.pose())
    pose = _offset(base, (20.0, 0.0, 0.0)).tolist()
    problems: list[str] = []
    exercised = 0
    for name, spec in command_table().items():
        if spec.kind not in (
            CommandKind.MOTION,
            CommandKind.QUEUED,
            CommandKind.SYSTEM,
            CommandKind.CONTROL,
        ):
            continue
        method = getattr(client, name, None)
        if method is None or name == "connect_hardware":
            # An optional command par6 does not implement, or the one that
            # puts the preview on hardware, where teleport is refused.
            continue
        assert name in _TABLE_ARGS, f"add sample arguments for {name}"
        args = tuple(
            park
            if name == "teleport"
            else pose
            if a is None
            else client.active_tool_key
            if a == "<fitted>"
            else a
            for a in _TABLE_ARGS[name]
        )
        if name in ("move_s", "move_p"):
            args = ([pose, _offset(base, (20.0, 0.0, 20.0)).tolist()],)
        if name == "move_c":
            args = (_offset(base, (10.0, 10.0, 0.0)).tolist(), pose)
        # An e-stop latches and a move leaves the arm where it finished;
        # clear both so the next command answers for itself.
        client.reset()
        client.teleport(park)
        result = getattr(client, name)(*args, **_TABLE_KWARGS.get(name, {}))
        exercised += 1
        if name == "pause":
            # Nothing queued answers while paused; the next command must not
            # inherit this one's hold.
            client.resume()
        if name == "estimate_payload":
            if result is None or not hasattr(result, "mass"):
                problems.append(
                    f"estimate_payload answers the estimate, got {result!r}"
                )
        elif isinstance(result, bool) or not isinstance(result, int):
            problems.append(f"{name}: {spec.kind.value} answers an int, got {result!r}")
        elif result < 0:
            problems.append(f"{name}: refused with {result}")
        elif spec.mints_index:
            block = client.plan().blocks[result]
            if block.error is not None:
                problems.append(f"{name}: refused: {block.error}")
            if block.move_type != spec.move_type:
                problems.append(
                    f"{name}: drawn as {block.move_type!r}, table says "
                    f"{spec.move_type!r}"
                )
    assert not problems, "\n".join(problems)
    assert exercised >= 20


def test_reset_state_keeps_a_latched_estop_and_reset_clears_it() -> None:
    """``reset_state`` resets the program, not the controller: a latched
    e-stop still refuses motion after it, and only ``reset`` re-enables."""
    client = Robot().create_dry_run_client(initial_joints_deg=park_deg())
    target = park_deg()
    target[0] += 10.0
    assert client.estop() == 1
    assert client.reset_state() == 1
    with pytest.raises(RobotError) as refused:
        client.move_j(target, speed=0.5)
    assert refused.value.code == ErrorCode.SYS_ESTOP_ACTIVE
    assert client.angles() == pytest.approx(park_deg())
    assert client.reset() == 1
    assert client.move_j(target, speed=0.5) >= 0
    assert client.angles() == pytest.approx(target, abs=1e-6)
