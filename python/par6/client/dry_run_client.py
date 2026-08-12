"""Offline dry-run client — the command stream simulated without a runtime.

Answers the ``waldoctl.DryRunClient`` protocol so a host application can
preview a program (path segments, move targets, timing feasibility, playback
timeline) with no ``par6d`` and no hardware.

It is not a second implementation of par6: kinematics are this package's own
:class:`par6.robot.Robot` (pinokin on the packaged URDF, with the active
tool's TCP applied), trajectories come from :mod:`par6.motion` (the port of
the runtime's own profiles and path geometry), and limits, tick period, home
pose and tool set are read from the same packaged config the runtime runs.

Every cartesian move rides one pipeline, as it does in the runtime
(``Par6Planner::start_cart_path``): the geometry produces a pose list, seeded
IK turns each pose into a joint waypoint, and TOPPRA times the chain.  Only
the shape differs — a line for ``move_l``, the circle through the via point
for ``move_c``, a cubic spline for ``move_s``, an auto-rounded polyline for
``move_p``.

**Blending.**  A move whose blend radius is positive is HELD, exactly as the
runtime's queue holds it, until the command that follows it decides what the
corner looks like.  Consecutive same-family moves fold into one motion —
cartesian chains get Bézier corners, joint chains get zones sized from the FK
TCP distances — and that one motion completes every command it consumed at
the same instant.  A held move therefore returns ``None`` (it has no motion of
its own yet); the whole chain's motion is returned by the command that closes
it, or by :meth:`~DryRunRobotClient.flush` at the end of the program, which is
where the runtime's blend hold expires.  A streamable or a teleport cancels
planned motion on the runtime, so it discards a held chain here too.

Refusals are modelled too.  A command ``par6d`` rejects — an unreachable
target, a line that leaves a soft window, a blend radius on ``move_c``,
degenerate arc or spline geometry, a planned move before homing, a tool that
is not fitted — is refused here with the same error code, so the editor shows
the failure before the arm does.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
from numpy.typing import NDArray
from pinokin import se3_from_rpy, so3_rpy
from waldoctl.results import DryRunResultData
from waldoctl.shapes import Shape, ShapeWorld

from par6 import config as _cfg
from par6 import motion as _motion
from par6.client.errors import RobotError
from par6.motion import JogEngine, LineSegment, MotionLimits, PlanningError
from par6.protocol.constants import NUM_JOINTS, ErrorCode

#: Wire axis order for ``jog_l`` / Cartesian velocity vectors.
_AXIS_INDEX: dict[str, int] = {"X": 0, "Y": 1, "Z": 2, "RX": 3, "RY": 4, "RZ": 5}

#: Minimum time the runtime holds a gripper calibration
#: (``crates/par6d/src/planner.rs::TOOL_CALIBRATE_MIN_WAIT_S``).
_TOOL_CALIBRATE_MIN_WAIT_S = 2.0

# Error templates, mirroring ``crates/par6-proto/src/error.rs`` for the codes
# an offline plan can produce. The runtime formats these server-side; a
# preview has to say the same thing about the same refusal.
_TEMPLATES: dict[ErrorCode, tuple[str, str, str, str]] = {
    ErrorCode.IK_TARGET_UNREACHABLE: (
        "IK: target unreachable",
        "No valid IK solution exists for the target pose. {detail}",
        "Motion command rejected; pipeline halted.",
        "Verify the target lies inside the workspace, or try a different orientation.",
    ),
    ErrorCode.IK_PARTIAL_PATH: (
        "IK: partial path failure",
        "Only {valid}/{total} poses along the path are reachable.",
        "Motion command rejected; pipeline halted.",
        "Shorten the move, add intermediate waypoints, or adjust the orientation.",
    ),
    ErrorCode.TRAJ_NO_STEPS: (
        "Trajectory: no steps",
        "Trajectory timing produced zero samples. {detail}",
        "Motion command rejected.",
        "Increase the duration or reduce the speed fraction.",
    ),
    ErrorCode.MOTN_SETUP_FAILED: (
        "Command setup failed",
        "The command could not be initialized. {detail}",
        "Command rejected; pipeline halted.",
        "Check the command parameters and robot state.",
    ),
    ErrorCode.MOTN_NOT_HOMED: (
        "Robot not homed",
        "Planned motion requested while joint positions are unreferenced.",
        "Motion command rejected before dispatch.",
        "Run home first; jogging remains available.",
    ),
    ErrorCode.COMM_VALIDATION_ERROR: (
        "Command validation error",
        "Invalid parameters. {detail}",
        "Command rejected.",
        "Check parameter ranges and types.",
    ),
    ErrorCode.SYS_PROFILE_INVALID: (
        "Invalid motion profile",
        "Unrecognised motion profile: {detail}",
        "Profile unchanged.",
        "Use a profile name supported by the runtime.",
    ),
}


def make_error(code: ErrorCode, **fields: object) -> RobotError:
    """A :class:`RobotError` with the runtime's own text for *code*."""
    title, cause, effect, remedy = _TEMPLATES[code]
    for name, value in fields.items():
        cause = cause.replace("{" + name + "}", str(value))
    return RobotError(-1, int(code), title, cause.strip(), effect, remedy)


def _pose_to_matrix(pose: NDArray[np.float64]) -> NDArray[np.float64]:
    """``[x, y, z, rx, ry, rz]`` (m + rad) as a 4x4 homogeneous transform."""
    T = np.zeros((4, 4), dtype=np.float64)
    se3_from_rpy(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], T)
    return T


def _wire_pose(pose_mm_deg: list[float]) -> NDArray[np.float64]:
    """A wire pose (mm + degrees) in SI."""
    if len(pose_mm_deg) != 6:
        raise make_error(
            ErrorCode.COMM_VALIDATION_ERROR, detail="pose requires 6 values"
        )
    p = np.asarray(pose_mm_deg, dtype=np.float64)
    return np.concatenate([p[:3] / 1000.0, np.radians(p[3:])])


def _blend_radius_m(r: float | None) -> float:
    """A wire blend radius (mm, 0/None = unset) in metres."""
    return max(float(r), 0.0) / 1000.0 if r else 0.0


def _checked_frame(frame: str) -> str:
    if frame not in ("WRF", "TRF"):
        raise ValueError(f"unknown frame {frame!r} (par6 supports WRF and TRF)")
    return frame


def _timing(duration: float | None, speed: float | None) -> tuple[float | None, float]:
    """The client's duration/speed pair as ``(min_duration_s, speed_fraction)``.

    Same mapping as :func:`par6.client.async_client._timing`: 0 / None means
    unset, the two are mutually exclusive, and neither set means full speed.
    """
    d = float(duration) if duration else None
    s = float(speed) if speed else None
    if d is not None and s is not None:
        raise ValueError("duration and speed are mutually exclusive")
    return d, s if s is not None else 1.0


def _target_pose(
    start: NDArray[np.float64],
    wire: NDArray[np.float64],
    frame: str,
    rel: bool,
) -> NDArray[np.float64]:
    """Where a cartesian move's wire pose puts the TCP, resolved against the
    pose the move starts from (``planner.rs::target_pose``)."""
    if frame == "TRF":
        # A tool-frame pose is inherently relative to the tool frame the
        # move starts in.
        return start @ wire
    if rel:
        # World-frame delta: translation adds, rotation applies about the
        # world axes.
        target = wire @ start
        target[:3, 3] = start[:3, 3] + wire[:3, 3]
        return target
    return wire


def _waypoint_poses(
    start: NDArray[np.float64], waypoints: list[list[float]], frame: str
) -> list[NDArray[np.float64]]:
    """A wire waypoint list as poses, starting at where the arm is.

    TRF waypoints all resolve against the STARTING tool frame — the list
    describes one shape in one frame, not a chain of successive tool-relative
    hops (parol6's ``_transform_waypoints_trf_to_wrf``).  The first waypoint
    is replaced by the measured pose when it is within
    :data:`par6.motion.WAYPOINT_SNAP_M` of it, so a client that starts its
    list where it believes the arm is gets its shape and not that shape plus
    a millimetre-long lead-in segment.
    """
    poses = [start]
    wire = [
        _target_pose(start, _pose_to_matrix(_wire_pose(w)), frame, False)
        for w in waypoints
    ]
    if wire:
        lead = float(np.linalg.norm(wire[0][:3, 3] - start[:3, 3]))
        if lead <= _motion.WAYPOINT_SNAP_M:
            wire = wire[1:]
    return poses + wire


@dataclass
class _CartMove:
    """A queued straight cartesian move waiting to be planned."""

    wire: NDArray[np.float64]
    frame: str
    rel: bool
    radius_m: float
    speed: float | None
    accel: float | None
    duration: float | None


@dataclass
class _JointMove:
    """A queued joint-space move (``move_j`` / ``move_j`` with ``pose=``)."""

    angles_rad: NDArray[np.float64] | None
    rel: bool
    pose: NDArray[np.float64] | None
    radius_m: float
    speed: float | None
    accel: float | None
    duration: float | None


@dataclass
class _Chain:
    """Moves the queue is holding because the last of them rounds a corner."""

    family: str = ""
    moves: list[Any] = field(default_factory=list)

    def wants_more(self) -> bool:
        """Whether the chain is still waiting for a successor to round into.

        The runtime holds the head while its radius is positive and the
        planner has not yet seen its whole lookahead window
        (``par6-server``'s ``blend_lookahead``).
        """
        return bool(
            self.moves
            and self.moves[-1].radius_m > 0.0
            and len(self.moves) < _motion.BLEND_LOOKAHEAD
        )

    def fold_speed(self) -> float:
        speeds = [m.speed for m in self.moves if m.speed is not None]
        return min(speeds) if speeds else 1.0

    def fold_accel(self) -> float:
        accels = [m.accel for m in self.moves if m.accel is not None]
        return min(accels) if accels else 1.0

    def fold_duration(self) -> float | None:
        """Durations add up, but only when every move in the chain has one."""
        if not all(m.duration is not None for m in self.moves):
            return None
        return sum(m.duration for m in self.moves)


class _DryRunTool:
    """``client.tool`` for the preview — routes actions through the plan."""

    def __init__(self, client: "DryRunRobotClient") -> None:
        self._client = client

    def __getattr__(self, name: str) -> Any:
        def method(*args: Any, **kwargs: Any) -> DryRunResultData:
            return self._client.tool_action(
                self._client.active_tool_key, name, list(args), **kwargs
            )

        return method


class DryRunRobotClient:
    """Simulates the par6 command stream offline, one result per command.

    Constructed by :meth:`par6.robot.Robot.create_dry_run_client`; a host
    running previews in a worker process constructs it directly with the
    robot's live joint angles and homed state.
    """

    def __init__(
        self,
        initial_joints_deg: list[float] | None = None,
        max_snapshot_points: int = 200,
        initial_homed: bool = True,
    ) -> None:
        from par6.robot import Robot

        self._robot = Robot()
        self._dt = _motion.tick_dt_s()
        self._exec_limits = MotionLimits.from_config("exec")
        self._stream_limits = MotionLimits.from_config("stream")
        self._max_points = max(2, int(max_snapshot_points))
        self._profile = _motion.DEFAULT_PROFILE

        home_deg = self._robot.joints.home.deg
        self._q = (
            np.radians(np.asarray(initial_joints_deg, dtype=np.float64))
            if initial_joints_deg is not None
            else np.radians(np.asarray(home_deg, dtype=np.float64))
        )
        self._homed = bool(initial_homed)
        self._tool_key = self._robot.tools.default.key
        self._variant_key = ""
        self._tcp_offset_m = (0.0, 0.0, 0.0)
        self._shapes: tuple[Shape, ...] = ()
        self._tool = _DryRunTool(self)
        self._robot.set_active_tool(self._tool_key)
        self._chain = _Chain()
        self._pending: list[DryRunResultData] = []

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    @property
    def active_tool_key(self) -> str:
        return self._tool_key

    @property
    def tool(self) -> _DryRunTool:
        return self._tool

    def angles(self) -> list[float]:
        """Simulated joint angles in degrees."""
        return np.degrees(self._q).tolist()

    def pose(self) -> list[float]:
        """Simulated TCP pose ``[x, y, z, rx, ry, rz]`` in mm + degrees."""
        out = np.zeros(6, dtype=np.float64)
        self._robot.fk(self._q, out)
        return [*(out[:3] * 1000.0), *np.degrees(out[3:])]

    def tcp_offset(self) -> list[float]:
        """The TCP offset applied on top of the tool transform, in mm."""
        return [v * 1000.0 for v in self._tcp_offset_m]

    def shapes(self) -> ShapeWorld:
        """The preview's collision world (what this run has submitted)."""
        return ShapeWorld(installation=(), program=self._shapes)

    def profile(self) -> str:
        return self._profile

    def is_simulator(self) -> bool:
        return True

    def flush(self) -> list[DryRunResultData]:
        """Plan whatever the blend hold is still holding.

        A move whose radius is positive waits for the successor it rounds a
        corner into; at the end of a program that successor never comes, and
        the runtime's blend hold expires and runs the chain as it stands (the
        last move simply stops at its target).  Call this after the last
        command to collect that motion.
        """
        self._close_chain()
        pending, self._pending = self._pending, []
        return pending

    # ------------------------------------------------------------------
    # Result construction
    # ------------------------------------------------------------------

    def _result(self, joint_path: NDArray[np.float64]) -> DryRunResultData:
        """Commit *joint_path* (tick-rate, radians) and describe it."""
        duration = len(joint_path) * self._dt
        stride = max(1, len(joint_path) // self._max_points)
        sampled = joint_path[::stride]
        if not np.array_equal(sampled[-1], joint_path[-1]):
            sampled = np.vstack([sampled, joint_path[-1:]])
        self._q = joint_path[-1].copy()
        return DryRunResultData(
            tcp_poses=self._robot.fk_batch(sampled),
            end_joints_rad=self._q.copy(),
            duration=duration,
            joint_trajectory_rad=sampled.copy(),
        )

    def _instant(self) -> DryRunResultData:
        """A command that changes nothing: one sample at the current pose."""
        return self._result(self._q[np.newaxis, :].copy())

    def _snap_to(self, q_rad: NDArray[np.float64]) -> DryRunResultData:
        """Jump to *q_rad* with no trajectory (home / teleport)."""
        self._q = np.asarray(q_rad, dtype=np.float64).copy()
        self._homed = True
        result = self._result(self._q[np.newaxis, :].copy())
        result.duration = 0.0
        return result

    def _require_homed(self) -> None:
        if not self._homed:
            raise make_error(ErrorCode.MOTN_NOT_HOMED)

    def _plan(self, target: NDArray[np.float64], **kwargs: Any) -> DryRunResultData:
        try:
            path = _motion.plan_joint_move(
                self._q, target, self._exec_limits, self._dt, **kwargs
            )
        except PlanningError as e:
            raise make_error(ErrorCode.MOTN_SETUP_FAILED, detail=str(e)) from None
        return self._result(path)

    # ------------------------------------------------------------------
    # Blend chain
    # ------------------------------------------------------------------

    def _queue_move(self, family: str, move: Any) -> DryRunResultData | None:
        """Hold *move* for blending, or plan the chain it completes.

        Mirrors the runtime's queue: a chain grows while the move already in
        it asks for a rounded corner AND the next command is a move of the
        same family (``Par6Planner::blend_chain_len``).  Anything else ends
        it — the arm stops at that target, which is what "no blend radius"
        asks for.
        """
        if self._chain.moves and self._chain.family != family:
            self._close_chain()
        self._chain.family = family
        self._chain.moves.append(move)
        if not self._chain.wants_more():
            self._close_chain()
        return self._take_pending()

    def _close_chain(self) -> None:
        """Plan what the chain holds, as ONE motion, and queue its result."""
        chain, self._chain = self._chain, _Chain()
        if not chain.moves:
            return
        if chain.family == "cart":
            self._pending.append(self._plan_cart_chain(chain))
        else:
            self._pending.append(self._plan_joint_chain(chain))

    def _discard_chain(self) -> None:
        """Drop a held chain unplanned.

        A streamable or a teleport cancels planned motion on the runtime
        (``cancel_planned``), pending queue included, so a move still waiting
        for the successor it would blend into never runs.
        """
        self._chain = _Chain()

    def _take_pending(self) -> DryRunResultData | None:
        """The motions closed so far: one result, or all of them merged."""
        pending, self._pending = self._pending, []
        if not pending:
            return None
        return pending[0] if len(pending) == 1 else _merge(pending)

    def _emit(self, result: DryRunResultData) -> DryRunResultData:
        """*result*, behind whatever motion a chain closed ahead of it."""
        pending = self._take_pending()
        return result if pending is None else _merge([pending, result])

    def _plan_cart_chain(self, chain: _Chain) -> DryRunResultData:
        """One cartesian motion covering every move the chain holds.

        Each move's target resolves against its PREDECESSOR's target, not
        against the live pose: a relative or tool-frame move in the middle of
        a chain means "from where the move before it ends", which is where
        the arm will be (``Par6Planner::start_move_l_chain``, and parol6's
        ``do_setup_with_blend``).
        """
        moves: list[_CartMove] = chain.moves
        start = self._current_pose_matrix()
        waypoints = [start]
        for move in moves:
            waypoints.append(_target_pose(waypoints[-1], move.wire, move.frame, move.rel))
        if len(waypoints) == 2:
            poses = _motion.line(waypoints[0], waypoints[1], _motion.line_sampling())
        else:
            radii = [max(m.radius_m, 0.0) for m in moves[:-1]]
            poses = self._geometry(
                _motion.blended_polyline, waypoints, radii, _motion.path_sampling()
            )
        return self._cart_path(
            poses,
            speed_fraction=chain.fold_speed(),
            accel_fraction=chain.fold_accel(),
            min_duration_s=chain.fold_duration(),
        )

    def _plan_joint_chain(self, chain: _Chain) -> DryRunResultData:
        """One joint-space motion covering every move the chain holds.

        A single move rides the selected profile.  A chain's corners are
        rounded in joint space, and the radius is a CARTESIAN quantity that a
        joint segment has no length for — so each zone is sized by the FK TCP
        distance between the waypoints it joins, and the rounded path is
        timed by TOPPRA (``Par6Planner::start_joint_chain``; parol6 converts
        the radius the same way in ``commands/joint_commands.py``).
        """
        moves: list[_JointMove] = chain.moves
        waypoints = [self._q.copy()]
        for move in moves:
            previous = waypoints[-1]
            if move.pose is not None:
                solution = self._robot.ik(move.pose, previous)
                if not solution.success:
                    raise make_error(
                        ErrorCode.IK_TARGET_UNREACHABLE,
                        detail=solution.violations
                        or "The solver did not converge from the current configuration.",
                    )
                target = solution.q.copy()
            else:
                assert move.angles_rad is not None
                target = (
                    previous + move.angles_rad if move.rel else move.angles_rad.copy()
                )
            waypoints.append(target)

        if len(moves) == 1:
            return self._plan(
                waypoints[1],
                profile=self._profile,
                speed_fraction=chain.fold_speed(),
                accel_fraction=chain.fold_accel(),
                min_duration_s=chain.fold_duration(),
            )

        for target in waypoints[1:]:
            try:
                self._exec_limits.require_inside_soft(target)
            except PlanningError as e:
                raise make_error(ErrorCode.MOTN_SETUP_FAILED, detail=str(e)) from None
        tcp = self._robot.fk_batch(np.stack(waypoints))[:, :3]
        fracs: list[tuple[float, float]] = []
        for i in range(1, len(waypoints) - 1):
            r = max(moves[i - 1].radius_m, 0.0)
            before = float(np.linalg.norm(tcp[i] - tcp[i - 1]))
            after = float(np.linalg.norm(tcp[i + 1] - tcp[i]))
            fracs.append(
                (
                    r / before if before > 1e-9 else 0.0,
                    r / after if after > 1e-9 else 0.0,
                )
            )
        path = self._geometry(
            _motion.blended_polyline_joint,
            np.stack(waypoints),
            fracs,
            _motion.CART_STEP_RAD,
            _motion.CART_PATH_MAX_STEPS,
        )
        return self._time_path(
            path,
            speed_fraction=chain.fold_speed(),
            accel_fraction=chain.fold_accel(),
            min_duration_s=chain.fold_duration(),
        )

    @staticmethod
    def _geometry(generator: Any, *args: Any) -> Any:
        """Run a path generator, answering bad geometry the way the runtime
        does (``planning_error``: ``MotionError::InvalidInput`` is a command
        validation error, not a setup failure)."""
        try:
            return generator(*args)
        except PlanningError as e:
            raise make_error(ErrorCode.COMM_VALIDATION_ERROR, detail=str(e)) from None

    def _current_pose_matrix(self) -> NDArray[np.float64]:
        """Where the TCP is standing, as a 4x4 transform."""
        pose = np.zeros(6, dtype=np.float64)
        self._robot.fk(self._q, pose)
        return _pose_to_matrix(pose)

    def _time_path(
        self,
        waypoints: NDArray[np.float64],
        *,
        speed_fraction: float,
        accel_fraction: float,
        min_duration_s: float | None,
    ) -> DryRunResultData:
        """TOPPRA-time a joint waypoint path and commit it."""
        try:
            path = _motion.plan_toppra_path(
                waypoints,
                self._exec_limits,
                self._dt,
                speed_fraction=speed_fraction,
                accel_fraction=accel_fraction,
                min_duration_s=min_duration_s,
            )
        except PlanningError as e:
            raise make_error(ErrorCode.TRAJ_NO_STEPS, detail=str(e)) from None
        return self._result(path)

    def _cart_path(
        self,
        poses: list[NDArray[np.float64]],
        *,
        speed_fraction: float,
        accel_fraction: float,
        min_duration_s: float | None,
    ) -> DryRunResultData:
        """The cartesian pipeline every cartesian move rides.

        Port of ``Par6Planner::start_cart_path``: ``poses[0]`` is where the
        arm already is, every other pose is solved from the previous
        solution, and a pose that is unreachable or a configuration flip away
        from its seed fails the move.  A failed move still reports the
        requested path with per-pose validity, so a preview can show how far
        it gets.
        """
        segments = [LineSegment(a, b) for a, b in zip(poses, poses[1:])]
        moved = any(
            seg.length_m >= _motion.MOVE_L_NULL_M
            or seg.angle_rad >= _motion.MOVE_L_NULL_M
            for seg in segments
        )
        if not moved:
            return self._instant()

        total = len(poses)
        waypoints = [self._q.copy()]
        valid = np.ones(total - 1, dtype=np.bool_)
        failure: RobotError | None = None
        seed = self._q.copy()
        for k, T in enumerate(poses[1:]):
            solution = self._robot.ik(_matrix_to_pose(T), seed)
            flipped = (
                np.abs(solution.q - seed).max() > _motion.MOVE_L_MAX_JOINT_STEP_RAD
            )
            if not solution.success or flipped:
                valid[k:] = False
                failure = make_error(
                    ErrorCode.IK_PARTIAL_PATH, valid=k + 1, total=total
                )
                break
            waypoints.append(solution.q.copy())
            seed = solution.q
        if failure is not None:
            return DryRunResultData(
                tcp_poses=np.stack([_matrix_to_pose(T) for T in poses[1:]]),
                end_joints_rad=self._q.copy(),
                duration=0.0,
                error=failure,
                valid=valid,
            )
        return self._time_path(
            np.stack(waypoints),
            speed_fraction=speed_fraction,
            accel_fraction=accel_fraction,
            min_duration_s=min_duration_s,
        )

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    def home(self, **kwargs: Any) -> DryRunResultData:
        """Reference the arm and park it where the homing sequence ends.

        The runtime runs its full referencing sequence on every ``home()``
        (it does not short-circuit when already homed), and that sequence's
        ``move_to`` steps decide the final pose — so the preview lands there.
        Its wall-clock duration is a property of the physical seek, not of a
        plan, and is reported as 0.
        """
        self._close_chain()
        return self._emit(self._snap_to(_cfg.homing_ready_pose_rad()))

    def teleport(
        self, angles_deg: list[float], tool_positions: list[float] | None = None
    ) -> DryRunResultData:
        """Sim-only jump to *angles_deg*, clamped to the hard limits as the
        runtime clamps them, and establishing the position reference.

        ``tool_positions`` places the jaws on the runtime's simulated tool;
        the preview carries no tool geometry, so it changes nothing here.

        Teleport is streamable-class on the runtime: it cancels planned
        motion, so a move still held for blending never runs.
        """
        self._discard_chain()
        config = _cfg.load_robot_config()
        hard = np.array(
            [
                [j["limits"]["hard_min_rad"], j["limits"]["hard_max_rad"]]
                for j in config["joints"]
            ]
        )
        q = np.radians(np.asarray(angles_deg, dtype=np.float64))
        return self._snap_to(np.clip(q, hard[:, 0], hard[:, 1]))

    def move_j(
        self,
        angles: list[float] | None = None,
        *,
        pose: list[float] | None = None,
        duration: float = 0.0,
        speed: float = 0.0,
        accel: float = 1.0,
        r: float = 0.0,
        rel: bool = False,
        **kwargs: Any,
    ) -> DryRunResultData | None:
        """Joint-space move; ``pose=`` reaches a Cartesian target by IK.

        With ``r`` positive the move is held for the joint move behind it and
        returns ``None`` — the two are one motion, and it is the move that
        closes the chain (or :meth:`flush`) that returns it.
        """
        self._require_homed()
        min_duration, speed_fraction = _timing(duration, speed)
        if pose is not None:
            target = _wire_pose(pose)
            angles_rad = None
        else:
            if angles is None:
                raise ValueError("move_j requires angles or pose=")
            target = None
            angles_rad = np.radians(np.asarray(angles, dtype=np.float64))
        return self._queue_move(
            "joint",
            _JointMove(
                angles_rad=angles_rad,
                rel=bool(rel),
                pose=target,
                radius_m=_blend_radius_m(r),
                speed=speed_fraction if min_duration is None else None,
                accel=float(accel),
                duration=min_duration,
            ),
        )

    def move_l(
        self,
        pose: list[float],
        *,
        frame: str = "WRF",
        duration: float = 0.0,
        speed: float = 0.0,
        accel: float = 1.0,
        r: float = 0.0,
        rel: bool = False,
        **kwargs: Any,
    ) -> DryRunResultData | None:
        """Straight Cartesian move — IK along the line, then TOPPRA timing.

        The same pipeline the runtime runs (``Par6Planner::start_move_l``):
        the segment is discretized at the runtime's step size, every waypoint
        is solved from the previous solution, and a waypoint that is
        unreachable, outside a soft window or a configuration flip away from
        its seed fails the move.  A failed move still reports the requested
        line with per-pose validity, so a preview can show how far it gets.

        With ``r`` positive the move is held for the straight move behind it
        and returns ``None``: the corner between them is rounded and the two
        run as one motion.
        """
        self._require_homed()
        min_duration, speed_fraction = _timing(duration, speed)
        return self._queue_move(
            "cart",
            _CartMove(
                wire=_pose_to_matrix(_wire_pose(pose)),
                frame=_checked_frame(frame),
                rel=bool(rel),
                radius_m=_blend_radius_m(r),
                speed=speed_fraction if min_duration is None else None,
                accel=float(accel),
                duration=min_duration,
            ),
        )

    def move_c(
        self,
        via: list[float],
        end: list[float],
        *,
        frame: str = "WRF",
        duration: float = 0.0,
        speed: float = 0.0,
        accel: float = 1.0,
        r: float = 0.0,
        **kwargs: Any,
    ) -> DryRunResultData:
        """Circular arc through *via* to *end*, on the circle the three poses
        define; an *end* that repeats the start sweeps the whole circle.

        A blend radius is refused: an arc stops at its end pose, and the
        runtime has no arc-to-successor blend to offer instead.
        """
        self._require_homed()
        if r:
            raise make_error(
                ErrorCode.COMM_VALIDATION_ERROR,
                detail=(
                    f"blend radius {float(r)} mm is not supported on move_c: an "
                    "arc stops at its end pose; send r = nil"
                ),
            )
        min_duration, speed_fraction = _timing(duration, speed)
        checked = _checked_frame(frame)
        self._close_chain()
        start = self._current_pose_matrix()
        poses = self._geometry(
            _motion.arc,
            start,
            _target_pose(start, _pose_to_matrix(_wire_pose(via)), checked, False),
            _target_pose(start, _pose_to_matrix(_wire_pose(end)), checked, False),
            _motion.path_sampling(),
        )
        return self._emit(
            self._cart_path(
                poses,
                speed_fraction=speed_fraction,
                accel_fraction=float(accel),
                min_duration_s=min_duration,
            )
        )

    def move_s(
        self,
        waypoints: list[list[float]],
        *,
        frame: str = "WRF",
        duration: float = 0.0,
        speed: float = 0.0,
        accel: float = 1.0,
        **kwargs: Any,
    ) -> DryRunResultData:
        """Cubic spline through the waypoint list (every one is passed
        through), starting from where the arm stands."""
        self._require_homed()
        min_duration, speed_fraction = _timing(duration, speed)
        checked = _checked_frame(frame)
        self._close_chain()
        poses = self._geometry(
            _motion.spline,
            _waypoint_poses(self._current_pose_matrix(), waypoints, checked),
            _motion.path_sampling(),
        )
        return self._emit(
            self._cart_path(
                poses,
                speed_fraction=speed_fraction,
                accel_fraction=float(accel),
                min_duration_s=min_duration,
            )
        )

    def move_p(
        self,
        waypoints: list[list[float]],
        *,
        frame: str = "WRF",
        duration: float = 0.0,
        speed: float = 0.0,
        accel: float = 1.0,
        **kwargs: Any,
    ) -> DryRunResultData:
        """Process move: the waypoints as straight segments with every
        interior corner rounded, so the TCP sweeps the path without stopping.

        No client names those radii — ``move_p`` promises auto-blended
        corners — so each is a quarter of the shorter adjacent segment
        (``planner.rs::MOVE_P_AUTO_BLEND_FRAC``), which is the largest radius
        the zone clamp would leave that corner anyway.
        """
        self._require_homed()
        min_duration, speed_fraction = _timing(duration, speed)
        checked = _checked_frame(frame)
        self._close_chain()
        wps = _waypoint_poses(self._current_pose_matrix(), waypoints, checked)
        lengths = [LineSegment(a, b).length_m for a, b in zip(wps, wps[1:])]
        radii = [
            _motion.MOVE_P_AUTO_BLEND_FRAC * min(a, b)
            for a, b in zip(lengths, lengths[1:])
        ]
        poses = self._geometry(
            _motion.blended_polyline, wps, radii, _motion.path_sampling()
        )
        return self._emit(
            self._cart_path(
                poses,
                speed_fraction=speed_fraction,
                accel_fraction=float(accel),
                min_duration_s=min_duration,
            )
        )

    # ------------------------------------------------------------------
    # Streaming and jog
    # ------------------------------------------------------------------

    def servo_j(
        self,
        angles: list[float] | None = None,
        *,
        pose: list[float] | None = None,
        speed: float = 1.0,
        accel: float = 1.0,
        **kwargs: Any,
    ) -> DryRunResultData:
        """One streamed target, evaluated as if it were the last one.

        The runtime tracks the newest target with a jerk-limited OTG under
        the STREAM limits; offline there is no arrival cadence to model, so
        each target is run to completion under those same limits.

        A streamable cancels planned motion on the runtime, so a move held
        for blending is dropped rather than run behind this one.
        """
        self._discard_chain()
        if pose is not None:
            target_pose = _wire_pose(pose)
            solution = self._robot.ik(target_pose, self._q)
            if not solution.success:
                raise make_error(
                    ErrorCode.IK_TARGET_UNREACHABLE,
                    detail=solution.violations or "no solution from this configuration",
                )
            target = solution.q
        else:
            if angles is None:
                raise ValueError("servo_j requires angles or pose=")
            target = np.radians(np.asarray(angles, dtype=np.float64))
        try:
            path = _motion.plan_joint_move(
                self._q,
                target,
                self._stream_limits,
                self._dt,
                profile="RUCKIG",
                speed_fraction=float(speed),
                accel_fraction=float(accel),
            )
        except PlanningError as e:
            raise make_error(ErrorCode.MOTN_SETUP_FAILED, detail=str(e)) from None
        return self._result(path)

    def servo_l(
        self, pose: list[float], *, speed: float = 1.0, accel: float = 1.0, **kwargs: Any
    ) -> DryRunResultData:
        """A streamed Cartesian target — IK'd, then tracked like ``servo_j``.

        Both land the arm on the same configuration; the streamed path between
        two targets is the RT limiter's, which only exists once targets arrive
        at a cadence.
        """
        return self.servo_j(pose=pose, speed=speed, accel=accel)

    def jog_j(
        self,
        joint: int = -1,
        speed: float = 0.0,
        duration: float = 0.1,
        *,
        joints: list[int] | None = None,
        speeds: list[float] | None = None,
        accel: float = 1.0,
        **kwargs: Any,
    ) -> DryRunResultData:
        """Velocity jog held for *duration*, integrated by the jog ramp."""
        self._discard_chain()
        fractions = np.zeros(NUM_JOINTS, dtype=np.float64)
        if joints is not None and speeds is not None:
            for j, s in zip(joints, speeds):
                fractions[j] = float(s)
        elif joint >= 0:
            fractions[joint] = float(speed)
        else:
            raise ValueError("jog_j requires either joint= or joints=/speeds=")
        if np.count_nonzero(fractions) > 1:
            raise make_error(
                ErrorCode.COMM_VALIDATION_ERROR,
                detail="jog_j drives one joint at a time",
            )
        engine = JogEngine(self._q, self._dt)
        return self._result(engine.run(fractions, float(duration)))

    def jog_l(
        self,
        frame: str,
        axis: str | None = None,
        speed: float = 0.0,
        duration: float = 0.1,
        *,
        axes: list[str] | None = None,
        speeds_list: list[float] | None = None,
        accel: float = 1.0,
        **kwargs: Any,
    ) -> DryRunResultData:
        """Cartesian velocity jog, integrated the way the runtime integrates it.

        Port of ``step_cart_jog`` (``crates/par6d/src/bridge.rs``): the
        velocity fractions scale the runtime's full-scale TCP rates, a
        tool-frame twist is rotated into the world by the current
        orientation, joint rates come from the Jacobian, and the integrated
        target is clamped to the soft window.
        """
        self._discard_chain()
        velocities = np.zeros(6, dtype=np.float64)
        if axes is not None and speeds_list is not None:
            for a, s in zip(axes, speeds_list):
                velocities[_AXIS_INDEX[a]] = float(s)
        elif axis is not None:
            velocities[_AXIS_INDEX[axis]] = float(speed)
        else:
            raise ValueError("jog_l requires either axis= or axes=/speeds_list=")
        if frame not in ("WRF", "TRF"):
            raise ValueError(f"unknown frame {frame!r} (par6 supports WRF and TRF)")

        twist = np.concatenate(
            [
                velocities[:3] * _motion.JOG_L_LINEAR_MAX_M_S,
                velocities[3:] * _motion.JOG_L_ANGULAR_MAX_RAD_S,
            ]
        )
        ticks = max(int(round(float(duration) / self._dt)), 1)
        q = self._q.copy()
        path = []
        for _ in range(ticks):
            v = twist.copy()
            if frame == "TRF":
                pose = np.zeros(6, dtype=np.float64)
                self._robot.fk(q, pose)
                rot = _pose_to_matrix(pose)[:3, :3]
                v[:3] = rot @ v[:3]
                v[3:] = rot @ v[3:]
            q_dot = np.linalg.pinv(self._robot.jacobian(q)) @ v
            q = np.clip(
                q + q_dot * self._dt,
                self._exec_limits.soft_min,
                self._exec_limits.soft_max,
            )
            path.append(q.copy())
        return self._result(np.stack(path))

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    def select_tool(self, tool_name: str, variant_key: str = "", **kwargs: Any) -> int:
        """Select a tool — refused for any tool the runtime is not fitted with."""
        self._close_chain()
        key = _cfg.canonical_tool_key(tool_name)
        if key != _cfg.fitted_tool_key():
            raise make_error(
                ErrorCode.COMM_VALIDATION_ERROR,
                detail=(
                    f"tool '{tool_name}' is not fitted: this runtime is built "
                    f"around '{_cfg.fitted_tool_key()}'"
                ),
            )
        self._tool_key = key
        self._variant_key = variant_key
        self._tcp_offset_m = (0.0, 0.0, 0.0)
        self._robot.set_active_tool(key, variant_key=variant_key or None)
        return 0

    def set_tcp_offset(self, x: float = 0, y: float = 0, z: float = 0, **kwargs: Any) -> int:
        """TCP offset in mm, composed on top of the tool transform."""
        self._tcp_offset_m = (x / 1000.0, y / 1000.0, z / 1000.0)
        self._robot.set_active_tool(
            self._tool_key,
            tcp_offset_m=self._tcp_offset_m,
            variant_key=self._variant_key or None,
        )
        return 1

    def select_profile(self, profile: str, **kwargs: Any) -> int:
        name = profile.strip().upper()
        if name not in _motion.PROFILES:
            raise make_error(ErrorCode.SYS_PROFILE_INVALID, detail=profile)
        self._profile = name
        return 1

    def set_shapes(self, shapes: list[Shape], **kwargs: Any) -> int:
        self._shapes = tuple(shapes)
        return 1

    def tool_action(
        self, tool_key: str, action: str, params: list | None = None, **kwargs: Any
    ) -> DryRunResultData:
        """A gripper action: the arm holds still while the tool works.

        Duration is only what the config supports.  A ``move`` finishes when
        the driver reports it did — the config carries no jaw speed model to
        predict that from, so it contributes no time.  A ``calibrate`` runs
        the driver's homing sequence, which the runtime holds for at least
        ``TOOL_CALIBRATE_MIN_WAIT_S`` (``crates/par6d/src/planner.rs``).
        """
        verb = action.strip().lower()
        if verb in ("open", "close", "set_position"):
            verb = "move"
        if verb not in ("move", "calibrate"):
            raise make_error(
                ErrorCode.COMM_VALIDATION_ERROR,
                detail=f"unknown tool action {action!r} (move, calibrate)",
            )
        self._close_chain()
        result = self._instant()
        result.duration = _TOOL_CALIBRATE_MIN_WAIT_S if verb == "calibrate" else 0.0
        return self._emit(result)

    # ------------------------------------------------------------------
    # Commands with no effect on an offline plan
    # ------------------------------------------------------------------

    def checkpoint(self, label: str, **kwargs: Any) -> int:
        self._close_chain()
        return 0

    def delay(self, seconds: float = 0.0, **kwargs: Any) -> int:
        self._close_chain()
        return 0

    def stop(self, clear_queue: bool = True, **kwargs: Any) -> int:
        """Halt motion.  A held blend chain is queued motion: it is dropped
        when the queue is cleared, and kept when it is not."""
        if clear_queue:
            self._discard_chain()
        return 1

    def estop(self, **kwargs: Any) -> int:
        self._discard_chain()
        return 1

    def reset(self, **kwargs: Any) -> int:
        return 1

    def reset_state(self, **kwargs: Any) -> int:
        self._discard_chain()
        return 1

    def wait_motion(self, **kwargs: Any) -> bool:
        return True

    def wait_command(self, command_index: int = -1, **kwargs: Any) -> bool:
        return True

    def wait_checkpoint(self, label: str = "", **kwargs: Any) -> bool:
        return True

    def wait_ready(self, **kwargs: Any) -> bool:
        return True

    def close(self) -> None:
        return None


def _merge(results: list[DryRunResultData]) -> DryRunResultData:
    """Several motions as one result, in the order they run.

    A single call can close a held chain AND run a motion of its own; the
    caller still gets one result, so the two are concatenated — same shape as
    parol6's ``_merge_results``.
    """
    drawn = [r for r in results if r.tcp_poses.shape[0] > 0]
    error = next((r.error for r in results if r.error is not None), None)
    if not drawn:
        return results[-1]
    valid = (
        np.concatenate(
            [
                r.valid
                if r.valid is not None
                else np.ones(r.tcp_poses.shape[0], dtype=np.bool_)
                for r in drawn
            ]
        )
        if any(r.valid is not None for r in drawn)
        else None
    )
    trajectories = [r.joint_trajectory_rad for r in drawn]
    return DryRunResultData(
        tcp_poses=np.vstack([r.tcp_poses for r in drawn]),
        end_joints_rad=drawn[-1].end_joints_rad,
        duration=sum(r.duration for r in results),
        error=error,
        valid=valid,
        joint_trajectory_rad=(
            np.vstack(trajectories) if all(t is not None for t in trajectories) else None
        ),
    )


def _matrix_to_pose(T: NDArray[np.float64]) -> NDArray[np.float64]:
    """4x4 transform as ``[x, y, z, rx, ry, rz]`` (m + rad).

    ``so3_rpy`` is the exact inverse of the ``se3_from_rpy`` that
    :class:`par6.robot.Robot` builds its IK targets with; any other
    decomposition of the same rotation would hand IK a different target.
    """
    rpy = np.zeros(3, dtype=np.float64)
    so3_rpy(np.ascontiguousarray(T[:3, :3]), rpy)
    return np.array([T[0, 3], T[1, 3], T[2, 3], *rpy], dtype=np.float64)


__all__ = ["DryRunRobotClient", "make_error"]
