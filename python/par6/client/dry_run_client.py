"""Offline dry-run client — the command stream simulated without a runtime.

Answers the ``waldoctl.DryRunClient`` protocol so a host application can
preview a program (path segments, move targets, timing feasibility, playback
timeline) with no ``par6d`` and no hardware.

It is not a second implementation of par6: kinematics are this package's own
:class:`par6.robot.Robot` (pinokin on the packaged URDF, with the active
tool's TCP applied), trajectories come from :mod:`par6.motion` (the port of
the runtime's own profiles and path geometry), and limits, tick period, home
pose and tool set are read from the same packaged config the runtime runs.

Refusals are modelled too.  A command ``par6d`` rejects — an unreachable
target, a line that leaves a soft window, a blend radius, an arc/spline move,
a planned move before homing, a tool that is not fitted — is refused here with
the same error code, so the editor shows the failure before the arm does.
"""

from __future__ import annotations

from typing import Any

import numpy as np
from numpy.typing import NDArray
from pinokin import se3_from_rpy, so3_rpy
from waldoctl.results import DryRunResultData
from waldoctl.shapes import Shape, ShapeWorld

from par6 import config as _cfg
from par6 import motion as _motion
from par6.client.errors import RobotError
from par6.motion import CartSegment, JogEngine, MotionLimits, PlanningError
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
        """No pending work: par6d plans and runs one queued command at a time."""
        return []

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

    @staticmethod
    def _failed(error: RobotError) -> DryRunResultData:
        return DryRunResultData(
            tcp_poses=np.empty((0, 6), dtype=np.float64),
            end_joints_rad=np.empty(0, dtype=np.float64),
            duration=0.0,
            error=error,
        )

    def _require_homed(self) -> None:
        if not self._homed:
            raise make_error(ErrorCode.MOTN_NOT_HOMED)

    @staticmethod
    def _reject_blend(r: float | None) -> None:
        if r:
            raise make_error(
                ErrorCode.COMM_VALIDATION_ERROR,
                detail=(
                    f"blend radius {float(r)} is not supported: this runtime plans "
                    "and runs exactly one queued command at a time"
                ),
            )

    def _plan(self, target: NDArray[np.float64], **kwargs: Any) -> DryRunResultData:
        try:
            path = _motion.plan_joint_move(
                self._q, target, self._exec_limits, self._dt, **kwargs
            )
        except PlanningError as e:
            raise make_error(ErrorCode.MOTN_SETUP_FAILED, detail=str(e)) from None
        return self._result(path)

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
        return self._snap_to(_cfg.homing_ready_pose_rad())

    def teleport(
        self, angles_deg: list[float], tool_positions: list[float] | None = None
    ) -> DryRunResultData:
        """Sim-only jump to *angles_deg*, clamped to the hard limits as the
        runtime clamps them, and establishing the position reference.

        ``tool_positions`` places the jaws on the runtime's simulated tool;
        the preview carries no tool geometry, so it changes nothing here.
        """
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
    ) -> DryRunResultData:
        self._require_homed()
        self._reject_blend(r)
        min_duration, speed_fraction = _timing(duration, speed)
        if pose is not None:
            target_pose = _wire_pose(pose)
            solution = self._robot.ik(target_pose, self._q)
            if not solution.success:
                raise make_error(
                    ErrorCode.IK_TARGET_UNREACHABLE,
                    detail=solution.violations
                    or "The solver did not converge from the current configuration.",
                )
            target = solution.q
        else:
            if angles is None:
                raise ValueError("move_j requires angles or pose=")
            requested = np.radians(np.asarray(angles, dtype=np.float64))
            target = self._q + requested if rel else requested
        return self._plan(
            target,
            profile=self._profile,
            speed_fraction=speed_fraction,
            accel_fraction=float(accel),
            min_duration_s=min_duration,
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
    ) -> DryRunResultData:
        """Straight Cartesian move — IK along the line, then TOPPRA timing.

        The same pipeline the runtime runs (``Par6Planner::start_move_l``):
        the segment is discretized at the runtime's step size, every waypoint
        is solved from the previous solution, and a waypoint that is
        unreachable, outside a soft window or a configuration flip away from
        its seed fails the move.  A failed move still reports the requested
        line with per-pose validity, so a preview can show how far it gets.
        """
        self._require_homed()
        self._reject_blend(r)
        min_duration, speed_fraction = _timing(duration, speed)
        if frame not in ("WRF", "TRF"):
            raise ValueError(f"unknown frame {frame!r} (par6 supports WRF and TRF)")

        start_pose = np.zeros(6, dtype=np.float64)
        self._robot.fk(self._q, start_pose)
        T_start = _pose_to_matrix(start_pose)
        T_wire = _pose_to_matrix(_wire_pose(pose))
        if frame == "TRF":
            T_target = T_start @ T_wire
        elif rel:
            T_target = T_wire @ T_start
            T_target[:3, 3] = T_start[:3, 3] + T_wire[:3, 3]
        else:
            T_target = T_wire

        segment = CartSegment(T_start, T_target)
        if (
            segment.length_m < _motion.MOVE_L_NULL_M
            and segment.angle_rad < _motion.MOVE_L_NULL_M
        ):
            return self._instant()

        steps = segment.steps()
        poses = np.stack([segment.sample(k / steps) for k in range(1, steps + 1)])
        waypoints = [self._q.copy()]
        valid = np.ones(steps, dtype=np.bool_)
        failure: RobotError | None = None
        seed = self._q.copy()
        for k, T in enumerate(poses):
            solution = self._robot.ik(_matrix_to_pose(T), seed)
            flipped = (
                np.abs(solution.q - seed).max() > _motion.MOVE_L_MAX_JOINT_STEP_RAD
            )
            if not solution.success or flipped:
                valid[k:] = False
                failure = make_error(
                    ErrorCode.IK_PARTIAL_PATH, valid=k + 1, total=steps + 1
                )
                break
            waypoints.append(solution.q.copy())
            seed = solution.q
        if failure is not None:
            return DryRunResultData(
                tcp_poses=np.stack([_matrix_to_pose(T) for T in poses]),
                end_joints_rad=self._q.copy(),
                duration=0.0,
                error=failure,
                valid=valid,
            )
        try:
            path = _motion.plan_toppra_path(
                np.stack(waypoints),
                self._exec_limits,
                self._dt,
                speed_fraction=speed_fraction,
                accel_fraction=float(accel),
                min_duration_s=min_duration,
            )
        except PlanningError as e:
            raise make_error(ErrorCode.TRAJ_NO_STEPS, detail=str(e)) from None
        return self._result(path)

    def _curved_move_refused(self) -> DryRunResultData:
        """The refusal ``par6d`` answers arc / spline / process moves with."""
        return self._failed(
            make_error(
                ErrorCode.MOTN_SETUP_FAILED,
                detail=(
                    "arc/spline/process moves are not implemented yet "
                    "(par6d follow-up)"
                ),
            )
        )

    def move_c(self, via: list[float], end: list[float], **kwargs: Any) -> DryRunResultData:
        return self._curved_move_refused()

    def move_s(self, waypoints: list[list[float]], **kwargs: Any) -> DryRunResultData:
        return self._curved_move_refused()

    def move_p(self, waypoints: list[list[float]], **kwargs: Any) -> DryRunResultData:
        return self._curved_move_refused()

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
        """
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
        result = self._instant()
        result.duration = _TOOL_CALIBRATE_MIN_WAIT_S if verb == "calibrate" else 0.0
        return result

    # ------------------------------------------------------------------
    # Commands with no effect on an offline plan
    # ------------------------------------------------------------------

    def checkpoint(self, label: str, **kwargs: Any) -> int:
        return 0

    def delay(self, seconds: float = 0.0, **kwargs: Any) -> int:
        return 0

    def stop(self, clear_queue: bool = True, **kwargs: Any) -> int:
        return 1

    def estop(self, **kwargs: Any) -> int:
        return 1

    def reset(self, **kwargs: Any) -> int:
        return 1

    def reset_state(self, **kwargs: Any) -> int:
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
