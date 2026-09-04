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
from live_daemon import STATUS_RATE_HZ, TICK_DT_S, LiveDaemon, requires_par6d

from par6 import config as _cfg
from par6._par6 import Preview as DryRunProfiles
from par6.client import RobotError
from par6.client.dry_run_client import DryRunResultData, DryRunRobotClient
from par6.protocol.constants import IO_SLOTS, NUM_JOINTS, CompletionPolicy, ErrorCode
from par6.protocol.wire import MAX_JOG_DURATION_S
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


def _planned(result: DryRunResultData | None) -> DryRunResultData:
    """The plan a move returned, refusing the blended-away case.

    A move with ``r > 0`` is held for the one behind it and returns
    ``None``; a test that then reads ``.duration`` off it would fail with
    an ``AttributeError`` pointing at the read, not at the move.
    """
    assert result is not None, "the move was blended into the next one, not planned"
    return result


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


class TestPlannedMotion:
    def test_plan_obeys_the_config_limits_under_every_profile(self) -> None:
        """Each advertised profile must produce a plan the runtime's own limits
        admit: no tick may step a joint faster than its EXEC velocity ceiling
        (scaled by the requested speed), and every plan must land on the target.
        A slower speed must take longer.

        Driven through the dry-run client — the plan under test is the one
        the runtime's own planner produces — with the config as the oracle.
        """
        config = _cfg.load_robot_config()
        dt = float(config["robot"]["tick_dt_s"])
        velocity = np.array(
            [_cfg.resolve_mode_limits(j["limits"], "exec")[0] for j in config["joints"]]
        )
        start = _cfg.homing_ready_pose_rad()
        target = start + np.radians([25.0, -10.0, 15.0, 0.0, 20.0, 0.0])
        # Full-rate trajectories: the velocity check reads per-tick steps,
        # which downsampling would smear across several ticks.
        client = Robot().create_dry_run_client(
            initial_joints_deg=np.degrees(start).tolist(),
            max_snapshot_points=1_000_000,
        )

        for profile in DryRunProfiles.profiles():
            client.select_profile(profile)
            durations: list[float] = []
            for speed in (1.0, 0.25):
                client.teleport(np.degrees(start).tolist())
                planned = client.move_j(np.degrees(target).tolist(), speed=speed)
                assert planned is not None
                path = planned.joint_trajectory_rad
                assert path is not None
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
        fast = dry_run.move_j(target, speed=1.0)
        dry_run.teleport(park_deg())
        slow = dry_run.move_j(target, duration=4.0)
        assert fast.duration < 1.5
        assert slow.duration == pytest.approx(
            4.0, abs=2 * float(_cfg.load_robot_config()["robot"]["tick_dt_s"])
        )
        np.testing.assert_allclose(slow.end_joints_rad, np.radians(target), atol=1e-6)


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
        # The endpoint decides reachable-at-all before the path decides
        # reachable-along-the-way (the planner's own precheck), so a
        # target outside the workspace refuses the whole command.
        with pytest.raises(RobotError) as blocked:
            dry_run.move_l(unreachable.tolist(), speed=0.5)
        assert blocked.value.code == ErrorCode.IK_TARGET_UNREACHABLE
        # The arm must not have moved: the runtime rejects the whole command.
        np.testing.assert_allclose(dry_run.angles(), before, atol=1e-9)

    def test_curved_moves_preview_the_shape_they_trace(self, dry_run) -> None:
        """``move_c`` must preview the arc through its via point, ``move_s`` the
        spline through every waypoint, and ``move_p`` the same waypoints with
        the corners rounded away — each ending on the last pose it was given."""
        dry_run.teleport(park_deg())
        base = np.asarray(dry_run.pose())
        via, end = _offset(base, (30.0, 0.0, 25.0)), _offset(base, (60.0, 0.0, 0.0))

        curve = dry_run.move_c(via.tolist(), end.tolist(), speed=0.4)
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
        curved = dry_run.move_s(waypoints, speed=0.4)
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
        process = dry_run.move_p(waypoints, speed=0.4)
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
        corner that the arm never stops in, and it is the move that closes the
        chain (or ``flush()``) that reports it."""
        dry_run.teleport(park_deg())
        base = np.asarray(dry_run.pose())
        corner, finish = (
            _offset(base, (50.0, 0.0, 0.0)),
            _offset(base, (50.0, 0.0, 40.0)),
        )

        sharp = [
            dry_run.move_l(corner.tolist(), speed=0.4),
            dry_run.move_l(finish.tolist(), speed=0.4),
        ]
        assert all(r is not None for r in sharp)
        stopped = np.vstack([r.tcp_poses[:, :3] for r in sharp]) * 1000.0

        dry_run.teleport(park_deg())
        held = dry_run.move_l(corner.tolist(), speed=0.4, r=15.0)
        assert held is None, "a move that rounds a corner has no motion of its own"
        blended = dry_run.move_l(finish.tolist(), speed=0.4)
        assert blended is not None and blended.error is None
        assert dry_run.flush() == [], "the chain was already closed"
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
        assert at_the_corner < 0.01 * max(s.max() for s in sharp_speeds), (
            "the un-blended pair is supposed to stop at the corner"
        )
        assert np.allclose(dry_run.angles(), np.degrees(blended.end_joints_rad))

        # A chain the program never closes is planned by flush(), which is
        # where the runtime's blend hold expires.
        dry_run.teleport(park_deg())
        assert dry_run.move_l(corner.tolist(), speed=0.4, r=15.0) is None
        trailing = dry_run.flush()
        assert len(trailing) == 1 and trailing[0].error is None
        assert np.allclose(trailing[0].tcp_poses[-1, :3] * 1000.0, corner[:3], atol=0.5)

    def test_blended_joint_moves_run_as_one_motion(self, dry_run) -> None:
        """Joint moves blend too: the corner zone is sized from the TCP distance
        the radius names, and the chain runs as one motion that never stops at
        the interior target — so it is quicker than the same moves run apart."""
        dry_run.teleport(park_deg())
        first = list(dry_run.angles())
        first[0] += 20.0
        second = list(first)
        second[1] -= 15.0

        apart = [dry_run.move_j(first, speed=0.5), dry_run.move_j(second, speed=0.5)]
        assert all(r is not None for r in apart)
        separate = sum(r.duration for r in apart)

        dry_run.teleport(park_deg())
        assert dry_run.move_j(first, speed=0.5, r=25.0) is None
        chain = dry_run.move_j(second, speed=0.5)
        assert chain is not None and chain.error is None
        assert chain.duration < separate
        np.testing.assert_allclose(np.degrees(chain.end_joints_rad), second, atol=1e-6)
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

        assert dry_run.move_j(roll, speed=0.5, r=25.0) is None
        chain = dry_run.move_j(swing, speed=0.5)
        assert chain is not None and chain.error is None

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
        # An arc ends where its end pose is: par6d rounds corners between
        # straight moves and between joint moves, but has no arc-to-successor
        # blend, so a radius on move_c is refused rather than ignored.
        with pytest.raises(RobotError) as arc_blend:
            dry_run.move_c(dry_run.pose(), dry_run.pose(), r=5.0)
        assert arc_blend.value.code == ErrorCode.COMM_VALIDATION_ERROR

        # Geometry the runtime cannot turn into a path is a validation error,
        # never silently straightened into a line.
        base = np.asarray(dry_run.pose())
        with pytest.raises(RobotError) as collinear:
            dry_run.move_c(
                _offset(base, (20.0, 0.0, 0.0)).tolist(),
                _offset(base, (40.0, 0.0, 0.0)).tolist(),
            )
        assert collinear.value.code == ErrorCode.COMM_VALIDATION_ERROR
        with pytest.raises(RobotError) as empty:
            dry_run.move_s([])
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
            dry_run.move_j(pose=dry_run.pose(), rel=True)

        far = list(dry_run.angles())
        far[1] = math.degrees(_cfg.soft_limits_rad()[1, 1]) + 20.0
        with pytest.raises(RobotError) as outside:
            dry_run.move_j(far)
        # A target outside the soft window is invalid input to the planner
        # (``planning_error``), the same class the runtime answers with.
        assert outside.value.code == ErrorCode.COMM_VALIDATION_ERROR

        unhomed = Robot().create_dry_run_client(
            initial_joints_deg=park_deg(), initial_homed=False
        )
        with pytest.raises(RobotError) as gate:
            unhomed.move_j(park_deg())
        assert gate.value.code == ErrorCode.MOTN_NOT_HOMED
        # Jogging stays available while un-homed, as it does on the runtime.
        assert unhomed.jog_j(0, 0.2, 0.2).duration > 0.0

    def test_the_preview_jogs_several_joints_at_once(self, dry_run) -> None:
        """A diagonal jog must preview as a diagonal.

        The preview refused any jog with more than one non-zero speed, so
        the two-axis gesture a pendant makes had no preview at all — while
        the engine underneath had been per-joint the whole time.
        """
        dry_run.teleport(park_deg())
        start = [math.radians(a) for a in dry_run.angles()]
        end = dry_run.jog_j(
            joints=[0, 3], speeds=[0.4, -0.4], duration=0.4
        ).end_joints_rad
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
            dry_run.move_j([0.0, 0.0, 0.0])
        with pytest.raises(ValueError, match="requires"):
            dry_run.teleport([0.0, 0.0, 0.0])

        # Outside a joint's travel the runtime refuses rather than clamping,
        # because clamping lands the arm somewhere else and reports success.
        hard = _cfg.load_robot_config()["joints"][0]["limits"]
        beyond = list(park_deg())
        beyond[0] = math.degrees(hard["hard_max_rad"]) + 10.0
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

        Un-referenced it is the seek, which ends wherever the configured
        sequence's ``move_to`` steps leave the arm and reports no duration.
        Referenced it is an ordinary planned move to the park pose — which is
        what makes a Home button cost seconds rather than a full seek.
        """
        robot = Robot()
        cold = robot.create_dry_run_client(
            initial_joints_deg=park_deg(), initial_homed=False
        )
        seek = cold.home()
        assert seek.duration == 0.0
        assert seek.end_joints_rad == pytest.approx(
            _cfg.homing_ready_pose_rad(), abs=1e-6
        )

        warm = robot.create_dry_run_client(
            initial_joints_deg=np.degrees(_cfg.homing_ready_pose_rad()).tolist()
        )
        ret = warm.home()
        assert ret.duration > 0.0, "a referenced HOME is a planned move, not a jump"
        assert ret.end_joints_rad == pytest.approx(robot.joints.home.rad, abs=1e-6)
        assert ret.tcp_poses.shape[0] > 1, "a planned move draws a path"

    def test_the_preview_answers_the_queries_a_program_reads_back(self) -> None:
        """A script that reads state between moves must not hit AttributeError,
        and the tool's jaw state must follow what the program told it to do."""
        client = Robot().create_dry_run_client(initial_joints_deg=park_deg())

        assert len(client.io()) == IO_SLOTS
        assert client.error() is None
        assert client.queue() == []
        # The mirror must report what the ENGINE plans with from the
        # first preview: the runtime's own startup profile
        # (par6d::planner::DEFAULT_PROFILE). A mirror that said TOPPRA
        # while the engine ran RUCKIG timed every pre-sync preview with
        # the wrong profile.
        assert client.profile() == "RUCKIG"
        assert client.is_robot_stopped()
        assert not client.is_estop_pressed()
        assert client.joint_speeds() == [0.0] * NUM_JOINTS
        assert client.tcp_speed() == 0.0

        status = client.status()
        assert status.angles == pytest.approx(client.angles(), abs=1e-9)
        assert status.pose[3:12:4] == pytest.approx(client.pose()[:3], abs=1e-6)

        assert client.tool.is_open()
        client.tool.close()
        assert not client.tool.is_open()
        assert client.tool.status().engaged
        client.tool.open()
        assert client.tool.is_open()
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
        as the runtime's queue does; the motion it released is the head of
        the next result, never dropped."""
        dry_run.teleport(park_deg())
        base = np.asarray(dry_run.pose())
        corner = _offset(base, (50.0, 0.0, 0.0))
        finish = _offset(base, (50.0, 0.0, 40.0))
        assert dry_run.move_l(corner.tolist(), speed=0.4, r=15.0) is None
        assert dry_run.checkpoint("corner") == 0
        result = _planned(dry_run.move_l(finish.tolist(), speed=0.4))
        path = result.tcp_poses[:, :3] * 1000.0
        assert np.allclose(path[0], base[:3], atol=2.0), (
            f"the chain the checkpoint closed must lead the result: {path[0]}"
        )
        assert _closest(path, corner[:3]) < 15.0
        assert np.allclose(path[-1], finish[:3], atol=0.5)
        assert dry_run.flush() == []

    def test_the_blend_hold_fills_at_the_runtimes_lookahead(self, dry_run) -> None:
        """The runtime's queue plans a chain once the blend lookahead is
        full; a hold that grew without bound would fold a whole program
        into one motion the arm runs in several."""
        dry_run.teleport(park_deg())
        cap = dry_run._preview.blend_lookahead()
        start = list(dry_run.angles())
        results = []
        for i in range(cap):
            target = list(start)
            target[0] += 2.0 * ((i % 2) + 1)
            results.append(dry_run.move_j(target, speed=0.5, r=5.0))
        assert all(r is None for r in results[:-1]), "held until the lookahead fills"
        assert results[-1] is not None, "the move that fills the hold runs the chain"
        assert dry_run.flush() == []

    def test_delays_and_tool_actions_carry_their_duration(self, dry_run) -> None:
        """A delay holds the arm for its seconds and a calibration for the
        runtime's minimum wait; both are time on the program's timeline."""
        dry_run.teleport(park_deg())
        with pytest.raises(ValueError, match="positive"):
            dry_run.delay(0.0)
        assert dry_run.delay(1.5) == 0
        held = dry_run.flush()
        assert len(held) == 1
        assert held[0].duration == pytest.approx(1.5, abs=2 * dry_run._dt)
        assert held[0].tcp_poses.shape[0] == 1, "a delay draws no path"

        calibration = _planned(dry_run.tool.calibrate())
        assert calibration.duration >= 2.0, "the runtime holds a calibration"
        assert _planned(dry_run.tool.stop()).duration == 0.0

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

        assert client.tool.calibrate().duration >= 2.0
        assert client.tool.close().duration == 0.0
        assert not client.tool.is_open()
        assert client.tool.stop().duration == 0.0
        assert client.tool.release().duration == 0.0
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
        assert dry_run.servo_j(target, speed=0.5).duration > 0.0

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
        jog = dry_run.jog_l("WRF", "X", speed=1.0, duration=0.5)
        end = np.asarray(dry_run.pose())
        assert end[0] - start[0] > 20.0, "half a second of full-scale +X must travel"
        assert abs(end[1] - start[1]) < 3.0 and abs(end[2] - start[2]) < 3.0
        assert jog.duration == pytest.approx(0.5, abs=2 * dry_run._dt)
        assert jog.joint_trajectory_rad.shape[1] == NUM_JOINTS
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
        action holds position, and home lands on the configured ready pose.

        Un-referenced to start with, because that is the state an editor
        opens on and it is why a program's first line is ``home()``. Seeding
        a configuration instead would have to seed a REACHABLE one: all-zeros
        is outside joints 1 and 2's travel, so no plan out of it is meaningful.
        """
        client = Robot().create_dry_run_client(
            initial_joints_deg=[0.0] * 6, initial_homed=False
        )
        results = []
        results.append(_planned(client.home()))
        np.testing.assert_allclose(
            results[-1].end_joints_rad, _cfg.homing_ready_pose_rad(), atol=1e-9
        )

        above = np.asarray(client.pose())
        above[2] += 30.0
        results.append(_planned(client.move_l(above.tolist(), speed=0.4)))
        results.append(_planned(client.tool.close()))
        joints = list(client.angles())
        joints[0] -= 15.0
        results.append(_planned(client.move_j(joints, speed=0.6)))
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

                predicted = _planned(preview.move_j(target, duration=requested))
                preview.teleport(live_start)  # re-seed for the next comparison

                started = time.monotonic()
                index = await client.move_j(target, duration=requested)
                assert index >= 0
                assert await client.wait_command(index, timeout=60.0) is True
                measured = time.monotonic() - started
                observed[requested] = (predicted.duration, measured)

                assert (
                    predicted.duration - 0.3 <= measured <= predicted.duration + 1.5
                ), (
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
#: The preview predicts the commanded trajectory while STATUS reports where
#: the arm actually went, so this budget covers the sim plant's tracking lag
#: as well as the chord error of comparing two sampled paths.  The lag
#: scales with commanded velocity and NOT with the tick rate — it is loop
#: bandwidth, not sampling — which is why these cases run at a slow
#: ``_CASE_SPEED``: at 0.4 it reaches 11 mm and swamps the planner
#: difference being measured; here it stays inside 3 mm, and a geometry,
#: sampling or corner-rounding difference is far larger than that.
_PATH_GAP_MM = 3.0

#: How far the endpoint may sit from the predicted one [mm].
_END_GAP_MM = 1.5

#: How far the captured motion's duration may sit from the predicted one
#: [s].  The window's ends are where MEASURED motion becomes detectable, and
#: the plant leaves the start and reaches the end asymptotically, so a
#: handful of ticks at each end fall under that threshold — about 1 % of
#: these cases' durations, which a timing difference would dwarf.
_DURATION_GAP_S = 0.08


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


def _preview_case(preview, case: str) -> list:
    """Plan one case offline; returns every result the preview produced."""
    if case in ("arc", "circle"):
        via, end = _shape_from(preview.pose(), _ARC if case == "arc" else _CIRCLE)
        return [preview.move_c(via, end, speed=_CASE_SPEED)]
    if case == "spline":
        return [preview.move_s(_shape_from(preview.pose(), _CURVE), speed=_CASE_SPEED)]
    if case == "process":
        return [preview.move_p(_shape_from(preview.pose(), _CURVE), speed=_CASE_SPEED)]
    corner, finish = _shape_from(preview.pose(), _CHAIN)
    held = preview.move_l(corner, speed=_CASE_SPEED, r=_CHAIN_R_MM)
    assert held is None, "a blended move must be held for the one behind it"
    return [preview.move_l(finish, speed=_CASE_SPEED)]


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
    lag stays well inside the gap budget — see :data:`_PATH_GAP_MM`.

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
                await _teleport_to(client, _OPEN_POSE_DEG)
                assert await client.wait_status(
                    lambda s: float(np.abs(np.asarray(s.speeds)).max()) < 0.02,
                    timeout=20.0,
                )
                live_start = await client.angles()
                assert live_start is not None
                preview = Robot().create_dry_run_client(initial_joints_deg=live_start)
                results = _preview_case(preview, case)
                assert all(r is not None and r.error is None for r in results), (
                    f"{case}: the preview refused it: "
                    f"{[r.error for r in results if r is not None]}"
                )
                anchor = robot.fk_batch(np.radians([live_start]))[:, :3] * 1000.0
                predicted = np.vstack(
                    [anchor, np.vstack([r.tcp_poses[:, :3] for r in results]) * 1000.0]
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
                assert bow > 5 * _PATH_GAP_MM, (
                    f"{case}: previewed path is nearly straight"
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
                    assert len(results) == 1, "the chain is one motion, not two"
                    assert finished[1] - finished[0] < 0.3, (
                        "a blended motion completes every command it consumed at "
                        f"the same instant, not {finished[1] - finished[0]:.3f}s apart"
                    )
                    assert await client.wait_status(
                        lambda s: s.completed_index == indexes[-1], timeout=5.0
                    ), "the high-water mark must end on the last command consumed"

    finally:
        daemon.stop()


async def _teleport_to(client, angles: list[float]) -> None:
    """Drive the sim to *angles*; teleport is unacked, so re-send until it lands."""
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline:
        await client.teleport(angles)
        if await client.wait_status(
            lambda s: (
                s.homed and float(np.abs(np.asarray(s.angles) - angles).max()) < 1.0
            ),
            timeout=0.5,
        ):
            return
    raise AssertionError("teleport never took effect")
