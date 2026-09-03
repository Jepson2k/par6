"""Offline dry-run client — the command stream previewed by the engine.

Answers the ``waldoctl.DryRunClient`` protocol so a host application can
preview a program (path segments, move targets, timing feasibility, playback
timeline) with no ``par6d`` and no hardware.

It is not a second implementation of par6: the preview engine
(`par6._par6.Preview`, over ``par6d::preview``) drives the daemon's OWN
planner against a virtual arm — same profiles, same IK, same TOPPRA
timing, same collision gate, same wire validation, same tool-action
checks — so a previewed command is planned by exactly the code that would
drive the arm.  This module owns only the Python-facing surface: waldoctl
types, wire-unit conversions, the blend hold, and the previewed tool/IO
readback state.

**Blending.**  A move whose blend radius is positive is HELD, exactly as the
runtime's queue holds it, until the command that follows it decides what the
corner looks like — or until the hold fills to the runtime's blend
lookahead, where the queue plans the chain as it stands.  The held commands
are offered to the planner as one batch, which folds them exactly as the
live queue would; the whole chain's motion is returned by the command that
closes it, or by :meth:`~DryRunRobotClient.flush` at the end of the program,
which is where the runtime's blend hold expires.  A held move therefore
returns ``None``.  A command that only reconfigures state (a checkpoint, a
delay, a tool selection) closes the hold too; the motion it releases rides
at the head of the next result rather than being lost.  A streamable or a
teleport cancels planned motion on the runtime, so it discards a held chain
here too.

Refusals are the runtime's own: wire validation, the not-homed gate, the
collision gate, the tool checks and the planner's refusals are all answered
by the same code with the same error templates, so the editor shows the
failure before the arm does.  Where the live client refuses a call itself
(a wrong-length angle list, an out-of-range output index), the dry run
raises the same ``ValueError``.
"""

from __future__ import annotations

import logging
from collections.abc import Callable, Iterable, Iterator
from typing import Any

import numpy as np
from numpy.typing import NDArray
from pinokin import so3_rpy
from waldoctl import ElectricGripperTool, ToolSpec, ToolState, ToolStatus
from waldoctl.results import DryRunResultData
from waldoctl.shapes import Shape, ShapeWorld
from waldoctl.status import ActionState as WActionState
from waldoctl.status import ActivityResult, LoopStatsResult, PingResult, ToolResult

from par6 import config as _cfg
from par6._par6 import Preview, RobotWireError, make_wire_error
from par6.client.async_client import (
    QueueResult,
    ReachableResult,
    StatusResult,
    _axis_index,
    _blend,
    _f6,
    _inertia6,
    _matrix_to_pose,
    _timing,
    _wire_frame,
)
from par6.client.errors import RobotError
from par6.protocol.constants import (
    NUM_JOINTS,
    CompletionPolicy,
    ControllerMode,
    ErrorCode,
    Frame,
)
from par6.protocol.wire import StatusBuffer

logger = logging.getLogger(__name__)


def make_error(code: ErrorCode, **params: Any) -> RobotError:
    """The runtime's structured refusal for *code* — rendered by the
    engine's own error templates, so a preview-side refusal says exactly
    what the runtime would say."""
    rendered = make_wire_error(int(code), {k: str(v) for k, v in params.items()})
    return RobotError.from_wire(rendered)


def _matrix_to_si_pose(m: list[float] | NDArray[np.float64]) -> NDArray[np.float64]:
    """Flattened row-major 4x4 (metres) → ``[x, y, z, rx, ry, rz]`` (m + rad).

    ``so3_rpy`` is the exact inverse of the ``se3_from_rpy`` that
    :class:`par6.robot.Robot` builds its IK targets with; any other
    decomposition of the same rotation would describe a different pose.
    """
    T = np.asarray(m, dtype=np.float64).reshape(4, 4)
    rpy = np.zeros(3, dtype=np.float64)
    so3_rpy(np.ascontiguousarray(T[:3, :3]), rpy)
    return np.array([T[0, 3], T[1, 3], T[2, 3], *rpy], dtype=np.float64)


def _resolve_engine_paths(config: str | None = None) -> tuple[str | None, str | None]:
    """The config file + assets tree the preview engine loads.

    An explicit *config* (a daemon-fetched bundle materialized by
    :func:`par6.config.materialize_bundle`) wins; then ``PAR6_CONFIG``.
    ``PAR6_ASSETS`` takes precedence for the assets tree; otherwise the
    repo tree around an editable install, then the deploy bundle's
    install locations (``/etc/par6`` + ``/usr/share/par6``, what
    ``scripts/deploy/install.sh`` stages on a control box) — so a wheel
    installed next to a deployed daemon previews with the exact files
    the daemon runs. The engine's own resolver knows only the repo
    checkout, so the deploy locations are searched here. The packaged
    ``_data`` URDFs carry rewritten mesh URIs the engine's loader cannot
    resolve, so they cannot feed it yet; a wheel with no daemon, repo, or
    env vars raises with that remedy.
    """
    import os
    from pathlib import Path

    config = config or os.environ.get("PAR6_CONFIG")
    assets = os.environ.get("PAR6_ASSETS")
    if config and assets:
        return config, assets
    root = Path(__file__).resolve().parents[3]
    for cfg_probe, assets_probe in (
        (root / "config" / "PAR6.toml", root / "assets" / "par6_description"),
        (
            Path("/etc/par6/PAR6.toml"),
            Path("/usr/share/par6/par6_description"),
        ),
    ):
        if (assets or assets_probe.is_dir()) and (config or cfg_probe.is_file()):
            return (config or str(cfg_probe)), (assets or str(assets_probe))
    raise RuntimeError(
        "the dry-run engine needs the runtime config and assets tree; set "
        "PAR6_CONFIG and PAR6_ASSETS (no repo checkout, and no deployed "
        f"bundle under /etc/par6 + /usr/share/par6, found near "
        f"{Path(__file__).resolve()})"
    )


class _DryRunTool:
    """``client.tool`` for the preview: the live tool's verbs, routed through
    the plan, over the spec's own attributes.

    Each verb sends the wire action the live tool sends (``open`` /
    ``close`` / ``set_position`` are ``move``, ``release`` is ``idle``),
    so the engine refuses exactly what the runtime would — an
    uncalibrated jaw, a position past the stroke, a verb the driver does
    not have.  Jaw position is tracked here rather than left at a
    default: a program that closes the gripper and asks ``is_open()`` two
    lines later gets the answer the arm would give.
    """

    def __init__(self, client: DryRunRobotClient) -> None:
        self._client = client
        self._position = 0.0

    @property
    def _spec(self) -> ToolSpec:
        return self._client._tools[self._client.active_tool_key]

    def __getattr__(self, name: str) -> Any:
        if name.startswith("_"):
            raise AttributeError(name)
        return getattr(self._spec, name)

    def status(self) -> ToolStatus:
        """The previewed tool state. Not a query — the preview knows what it
        was told to do and nothing else, so nothing is faulted or detected."""
        spec = self._spec
        return ToolStatus(
            key=spec.key,
            variant_key=self._client._variant_key,
            state=ToolState.IDLE,
            engaged=self._position > 0.5,
            positions=(self._position,) * len(getattr(spec, "motions", ()) or (1,)),
        )

    def is_open(self, position: float | None = None) -> bool:
        """True when the jaws are open; defaults to the previewed position."""
        return (self._position if position is None else position) < 0.5

    def _act(
        self, verb: str, params: list[Any], position: float | None = None
    ) -> DryRunResultData:
        result = self._client.tool_action(self._client.active_tool_key, verb, params)
        if position is not None:
            self._position = float(position)
        return result

    def set_position(self, position: float, **kwargs: Any) -> DryRunResultData:
        spec = self._spec
        if not isinstance(spec, ElectricGripperTool):
            raise AttributeError(f"tool {spec.key!r} has no jaws to position")
        speed = float(kwargs.get("speed", 0.5))
        current = int(kwargs.get("current", spec.current_range[1]))
        return self._act("move", [float(position), speed, current], position)

    def open(self, **kwargs: Any) -> DryRunResultData:
        return self.set_position(0.0, **kwargs)

    def close(self, **kwargs: Any) -> DryRunResultData:
        return self.set_position(1.0, **kwargs)

    def calibrate(self, **kwargs: Any) -> DryRunResultData:
        return self._act("calibrate", [])

    def stop(self, **kwargs: Any) -> DryRunResultData:
        return self._act("stop", [])

    def release(self, **kwargs: Any) -> DryRunResultData:
        return self._act("idle", [])


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
        config_path: str | None = None,
        initial_gripper_calibrated: bool = True,
    ) -> None:
        from par6.tools import build_tools

        config, assets = _resolve_engine_paths(config_path)
        self._preview = Preview(config=config, assets=assets)
        # The file the engine loaded is the one every config read here uses,
        # so a tuned deployment previews with its numbers throughout.
        self._config_path = self._preview.config_path()
        self._config = _cfg.load_robot_config(self._config_path)
        self._dt = self._preview.tick_dt_s()
        self._max_points = max(2, int(max_snapshot_points))
        self._blend_lookahead = max(1, int(self._preview.blend_lookahead()))
        # The runtime's own startup context, read off the engine rather
        # than mirrored here.
        context = self._preview.context()
        self._profile = str(context["profile"])
        self._policy = int(context["policy"])
        self._tcp_offset_mm = tuple(float(v) for v in context["tcp_offset_mm"])

        if initial_joints_deg is not None:
            q = np.radians(np.asarray(initial_joints_deg, dtype=np.float64))
            self._preview.teleport_rad(q.tolist())
        self._preview.set_homed(bool(initial_homed))
        self._preview.set_gripper_calibrated(bool(initial_gripper_calibrated))

        self._tools = {spec.key: spec for spec in build_tools().available}
        self._tool_key = _cfg.fitted_tool_key(self._config)
        self._variant_key = ""
        self._shapes: tuple[Shape, ...] = ()
        self._tool = _DryRunTool(self)
        self._held: list[dict] = []
        self._pending: list[DryRunResultData] = []
        self._io_inputs, self._io_outputs = _cfg.io_line_names(self._config)
        self._io_levels = [0] * len(self._io_outputs)
        self._last_checkpoint = ""
        self._completed = -1

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    @property
    def active_tool_key(self) -> str:
        return self._tool_key

    @property
    def tool(self) -> _DryRunTool:
        return self._tool

    def _q(self) -> NDArray[np.float64]:
        return np.asarray(self._preview.angles_rad(), dtype=np.float64)

    def _matrix_m(self) -> NDArray[np.float64]:
        """The TCP pose as a 4x4 matrix, translation in metres."""
        return np.asarray(self._call(self._preview.pose), dtype=np.float64).reshape(
            4, 4
        )

    def angles(self) -> list[float]:
        """Simulated joint angles in degrees."""
        return np.degrees(self._q()).tolist()

    def pose(self, frame: str = "WRF") -> list[float]:
        """Simulated TCP pose ``[x, y, z, rx, ry, rz]`` in mm + degrees, in
        the world frame or — as the runtime answers ``TRF`` — the world
        seen from the tool."""
        T = self._matrix_m()
        if _wire_frame(frame) == int(Frame.TRF):
            T = np.linalg.inv(T)
        T = T.copy()
        T[:3, 3] *= 1000.0
        return _matrix_to_pose(T.flatten().tolist())

    def tcp_offset(self) -> list[float]:
        """The TCP offset applied on top of the tool transform, in mm."""
        return list(self._tcp_offset_mm)

    def shapes(self) -> ShapeWorld:
        """The preview's collision world (what this run has submitted)."""
        return ShapeWorld(installation=(), program=self._shapes)

    def profile(self) -> str:
        return self._profile

    def is_simulator(self) -> bool:
        return True

    # ------------------------------------------------------------------
    # Engine plumbing
    # ------------------------------------------------------------------

    @staticmethod
    def _call(fn: Any, *args: Any) -> Any:
        """Run one engine call, translating its structured refusals."""
        try:
            return fn(*args)
        except RobotWireError as e:
            raise RobotError.from_wire(e.args) from None

    def _si_pose_now(self) -> NDArray[np.float64]:
        return _matrix_to_si_pose(self._call(self._preview.pose))

    def _convert(self, r: dict) -> DryRunResultData:
        """One engine preview result as :class:`DryRunResultData`.

        The engine already thinned the trajectory to ``max_snapshot_points``
        (endpoints kept); an empty trajectory (a command that moves
        nothing, or holds still for its duration) reports one sample at the
        pose the arm holds.
        """
        traj = np.asarray(r["joint_trajectory_rad"], dtype=np.float64)
        if traj.size == 0:
            return DryRunResultData(
                tcp_poses=self._si_pose_now()[np.newaxis, :],
                end_joints_rad=np.asarray(r["end_joints_rad"], dtype=np.float64),
                duration=float(r["duration_s"]),
                joint_trajectory_rad=self._q()[np.newaxis, :].copy(),
            )
        return DryRunResultData(
            tcp_poses=np.stack([_matrix_to_si_pose(p) for p in r["tcp_poses"]]),
            end_joints_rad=np.asarray(r["end_joints_rad"], dtype=np.float64),
            duration=float(r["duration_s"]),
            joint_trajectory_rad=traj,
        )

    def _error_result(self, err: RobotError) -> DryRunResultData:
        return DryRunResultData(
            tcp_poses=np.empty((0, 6), dtype=np.float64),
            end_joints_rad=self._q().copy(),
            duration=0.0,
            error=err,
        )

    def _run_batch(self, cmds: list[dict]) -> list[DryRunResultData]:
        """Preview *cmds* as one planner batch; raise the first refusal.

        The one refusal that is returned rather than raised is
        ``IK_PARTIAL_PATH`` — a preview wants to show how far a line gets,
        and the caller reads that off the result's ``error``.
        """
        results = self._preview.preview_program(cmds, self._max_points)
        out: list[DryRunResultData] = []
        for r in results:
            err = r["error"]
            if err is not None:
                robot_err = RobotError.from_wire(err)
                if robot_err.code == ErrorCode.IK_PARTIAL_PATH:
                    out.append(self._error_result(robot_err))
                    continue
                raise robot_err
            out.append(self._convert(r))
        self._completed += len(cmds)
        return out

    # ------------------------------------------------------------------
    # Blend chain
    # ------------------------------------------------------------------

    def _queue_move(self, cmd: dict, held: bool) -> DryRunResultData | None:
        """Hold *cmd* for blending, or run the batch it completes.

        The runtime's queue holds a move whose radius is positive until its
        successor arrives, or until the hold fills to the blend lookahead;
        the planner folds the batch exactly as the live queue would, so
        nothing here decides what folds — only when the batch is offered.
        """
        self._held.append(cmd)
        if held and len(self._held) < self._blend_lookahead:
            return None
        results = self._close_chain()
        return _merge(results) if results else None

    def _discard_chain(self) -> None:
        """Drop a held chain unplanned.

        A streamable or a teleport cancels planned motion on the runtime
        (``cancel_planned``), pending queue included, so a move still waiting
        for the successor it would blend into never runs.
        """
        self._held = []

    def _close_chain(self) -> list[DryRunResultData]:
        """Everything queued motion still owes the caller: a chain a
        state-only command closed quietly, then whatever the blend hold
        still holds, run as the runtime's hold expiry would."""
        owed, self._pending = self._pending, []
        if self._held:
            batch, self._held = self._held, []
            owed.extend(self._run_batch(batch))
        return owed

    def _emit(self, result: DryRunResultData) -> DryRunResultData:
        """*result*, behind whatever motion a chain closed ahead of it."""
        pending = self._close_chain()
        return result if not pending else _merge([*pending, result])

    def flush(self) -> list[DryRunResultData]:
        """Plan whatever the blend hold is still holding.

        A move whose radius is positive waits for the successor it rounds a
        corner into; at the end of a program that successor never comes, and
        the runtime's blend hold expires and runs the chain as it stands (the
        last move simply stops at its target).  Call this after the last
        command to collect that motion.
        """
        results = self._close_chain()
        return [_merge(results)] if results else []

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    def home(self, **kwargs: Any) -> DryRunResultData:
        """Reference the arm, or return it to the park pose when it already
        holds its references; ``calibrate=True`` previews the seek either way.

        Two different commands wear one name on the runtime
        (``Par6Planner``'s ``Command::Home``). Un-referenced, HOME runs the
        configured seek, and where it ends is decided by that sequence's
        ``move_to`` steps; its wall-clock duration is a property of the
        physical seek rather than of a plan, so it is reported as 0.
        Already referenced, HOME is an ordinary planned joint move to
        ``[robot].park_pose_rad`` at half speed — which is what makes a Home
        button press cost seconds instead of a full seek, and it is planned
        and collision-gated by the engine like any other move.
        """
        pending = self._close_chain()
        if kwargs.get("calibrate") or not self._preview.homed():
            ready = _cfg.homing_ready_pose_rad(self._config)
            self._preview.teleport_rad(ready.tolist())
            self._preview.set_homed(True)
            result = DryRunResultData(
                tcp_poses=self._si_pose_now()[np.newaxis, :],
                end_joints_rad=ready.copy(),
                duration=0.0,
                joint_trajectory_rad=ready[np.newaxis, :].copy(),
            )
            return result if not pending else _merge([*pending, result])
        results = self._run_batch([{"type": "home"}])
        return _merge([*pending, *results])

    def teleport(
        self, angles_deg: list[float], tool_positions: list[float] | None = None
    ) -> DryRunResultData:
        """Sim-only jump to *angles_deg*, establishing the position reference.

        A pose outside a joint's travel is REFUSED, not clamped — the
        runtime's rule (``teleport_angle_fault``), because clamping lands the
        arm tens of degrees from where the caller asked and reports success.

        ``tool_positions`` places the jaws on the runtime's simulated tool;
        the preview carries no tool geometry, so it changes nothing here.

        Teleport is streamable-class on the runtime: it cancels planned
        motion, so a move still held for blending never runs.
        """
        self._discard_chain()
        deg = np.asarray(_f6(angles_deg, "angles_deg"), dtype=np.float64)
        hard_deg = np.degrees(
            np.array(
                [
                    [j["limits"]["hard_min_rad"], j["limits"]["hard_max_rad"]]
                    for j in self._config["joints"]
                ]
            )
        )
        outside = np.flatnonzero((deg < hard_deg[:, 0]) | (deg > hard_deg[:, 1]))
        if outside.size:
            i = int(outside[0])
            raise make_error(
                ErrorCode.COMM_VALIDATION_ERROR,
                detail=(
                    f"angles[{i}] = {deg[i]:.3f} deg is outside joint {i}'s travel "
                    f"[{hard_deg[i, 0]:.3f}, {hard_deg[i, 1]:.3f}] deg; teleport "
                    "places the arm exactly where it is told or not at all"
                ),
            )
        q = np.radians(deg)
        self._preview.teleport_rad(q.tolist())
        self._preview.set_homed(True)
        return DryRunResultData(
            tcp_poses=self._si_pose_now()[np.newaxis, :],
            end_joints_rad=q.copy(),
            duration=0.0,
            joint_trajectory_rad=q[np.newaxis, :].copy(),
        )

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
        min_duration, speed_fraction = _timing(duration, speed)
        if pose is not None:
            if rel:
                # The live client raises exactly this (MOVE_J_POSE is
                # absolute on the wire); a preview that quietly planned
                # the absolute move would validate a program the arm
                # then refuses.
                raise ValueError(
                    "move_j(pose=..., rel=True) is not supported: MOVE_J_POSE is "
                    "absolute. Compose the offset with the current TCP pose, or "
                    "use move_j(angles=..., rel=True) for a relative joint move."
                )
            cmd = {
                "type": "move_j_pose",
                "pose": _f6(pose, "pose"),
                "duration": min_duration,
                "speed": speed_fraction,
                "accel": float(accel),
                "blend_radius": _blend(r),
            }
        else:
            if angles is None:
                raise ValueError("move_j requires angles or pose=")
            cmd = {
                "type": "move_j",
                "angles": _f6(angles, "angles"),
                "duration": min_duration,
                "speed": speed_fraction,
                "accel": float(accel),
                "blend_radius": _blend(r),
                "rel": bool(rel),
            }
        return self._queue_move(cmd, held=bool(_blend(r)))

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
        """Straight Cartesian move — the runtime's own line pipeline: the
        segment is discretized at the planner's step size, every waypoint is
        solved from the previous solution, and the chain is TOPPRA-timed.

        With ``r`` positive the move is held for the straight move behind it
        and returns ``None``: the corner between them is rounded and the two
        run as one motion.
        """
        min_duration, speed_fraction = _timing(duration, speed)
        cmd = {
            "type": "move_l",
            "pose": _f6(pose, "pose"),
            "frame": _wire_frame(frame),
            "duration": min_duration,
            "speed": speed_fraction,
            "accel": float(accel),
            "blend_radius": _blend(r),
            "rel": bool(rel),
        }
        return self._queue_move(cmd, held=bool(_blend(r)))

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
        rel: bool = False,
        **kwargs: Any,
    ) -> DryRunResultData:
        """Circular arc through *via* to *end*, on the circle the three poses
        define; an *end* that repeats the start sweeps the whole circle.
        With ``rel=True``, *via* and *end* are deltas from the start pose.

        A blend radius is refused: an arc stops at its end pose, and the
        runtime has no arc-to-successor blend to offer instead.
        """
        min_duration, speed_fraction = _timing(duration, speed)
        cmd = {
            "type": "move_c",
            "via": _f6(via, "via"),
            "end": _f6(end, "end"),
            "frame": _wire_frame(frame),
            "duration": min_duration,
            "speed": speed_fraction,
            "accel": float(accel),
            "blend_radius": _blend(r),
            "rel": bool(rel),
        }
        return self._emit_batch(cmd)

    def move_s(
        self,
        waypoints: list[list[float]],
        *,
        frame: str = "WRF",
        duration: float = 0.0,
        speed: float = 0.0,
        accel: float = 1.0,
        rel: bool = False,
        **kwargs: Any,
    ) -> DryRunResultData:
        """Cubic spline through the waypoint list (every one is passed
        through), starting from where the arm stands. With ``rel=True``,
        every waypoint is a delta from the start pose."""
        min_duration, speed_fraction = _timing(duration, speed)
        cmd = {
            "type": "move_s",
            "waypoints": [_f6(wp, "waypoint") for wp in waypoints] if waypoints else [],
            "frame": _wire_frame(frame),
            "duration": min_duration,
            "speed": speed_fraction,
            "accel": float(accel),
            "rel": bool(rel),
        }
        return self._emit_batch(cmd)

    def move_p(
        self,
        waypoints: list[list[float]],
        *,
        frame: str = "WRF",
        duration: float = 0.0,
        speed: float = 0.0,
        accel: float = 1.0,
        rel: bool = False,
        **kwargs: Any,
    ) -> DryRunResultData:
        """Process move: the waypoints as straight segments with every
        interior corner rounded by the planner's auto-blend rule, so the TCP
        sweeps the path without stopping. With ``rel=True``, every waypoint
        is a delta from the start pose."""
        min_duration, speed_fraction = _timing(duration, speed)
        cmd = {
            "type": "move_p",
            "waypoints": [_f6(wp, "waypoint") for wp in waypoints] if waypoints else [],
            "frame": _wire_frame(frame),
            "duration": min_duration,
            "speed": speed_fraction,
            "accel": float(accel),
            "rel": bool(rel),
        }
        return self._emit_batch(cmd)

    def _emit_batch(self, cmd: dict) -> DryRunResultData:
        """Close the hold, run *cmd*, and answer with everything that moved."""
        pending = self._close_chain()
        results = self._run_batch([cmd])
        return _merge([*pending, *results])

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

        The runtime tracks the newest target with a jerk-limited OTG;
        offline there is no arrival cadence to model, so the settle to the
        target is previewed with the planner's own joint move.  The speed
        fraction rides through as sent: the wire refuses what it refuses.
        A streamable cancels planned motion on the runtime, so a move held
        for blending is dropped rather than run behind this one.
        """
        self._discard_chain()
        if pose is not None:
            cmd: dict = {
                "type": "move_j_pose",
                "pose": _f6(pose, "pose"),
                "duration": None,
                "speed": float(speed),
                "accel": float(accel),
                "blend_radius": None,
            }
        else:
            if angles is None:
                raise ValueError("servo_j requires angles or pose=")
            cmd = {
                "type": "move_j",
                "angles": _f6(angles, "angles"),
                "duration": None,
                "speed": float(speed),
                "accel": float(accel),
                "blend_radius": None,
                "rel": False,
            }
        return _merge(self._run_batch([cmd]))

    def servo_l(
        self,
        pose: list[float],
        *,
        speed: float = 1.0,
        accel: float = 1.0,
        **kwargs: Any,
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
        """Velocity jog held for *duration*, integrated by the runtime's own
        jog ramp (the engine drives the same ``par6-motion`` engine the RT
        core ticks, soft-limit direction blocking included)."""
        self._discard_chain()
        fractions = [0.0] * NUM_JOINTS
        if joints is not None and speeds is not None:
            if len(joints) != len(speeds):
                raise ValueError(
                    f"jog_j got {len(joints)} joints and {len(speeds)} speeds"
                )
            for j, s in zip(joints, speeds):
                # Same guard as the live client: a negative index wraps
                # onto another joint and previews the wrong axis.
                if not 0 <= j < NUM_JOINTS:
                    raise ValueError(
                        f"jog_j joint {j} out of range 0..{NUM_JOINTS - 1}"
                    )
                fractions[j] = float(s)
        elif joint >= 0:
            if joint >= NUM_JOINTS:
                raise ValueError(
                    f"jog_j joint {joint} out of range 0..{NUM_JOINTS - 1}"
                )
            fractions[joint] = float(speed)
        else:
            raise ValueError("jog_j requires either joint= or joints=/speeds=")
        r = self._preview.preview_jog(
            fractions, float(duration), float(accel), self._max_points
        )
        if r["error"] is not None:
            raise RobotError.from_wire(r["error"])
        return self._convert(r)

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
        """Cartesian velocity jog, integrated by the runtime's own twist
        solver (``step_cart_jog``) through the same kinematics, TCP offset
        and soft window the arm jogs with, and gated on the collision
        world."""
        self._discard_chain()
        velocities = [0.0] * 6
        if axes is not None and speeds_list is not None:
            if len(axes) != len(speeds_list):
                raise ValueError(
                    f"jog_l got {len(axes)} axes and {len(speeds_list)} speeds"
                )
            for a, s in zip(axes, speeds_list):
                velocities[_axis_index(a)] = float(s)
        elif axis is not None:
            velocities[_axis_index(axis)] = float(speed)
        else:
            raise ValueError("jog_l requires either axis= or axes=/speeds_list=")
        r = self._preview.preview_jog_l(
            velocities,
            float(duration),
            _wire_frame(frame),
            float(accel),
            self._max_points,
        )
        if r["error"] is not None:
            raise RobotError.from_wire(r["error"])
        return self._convert(r)

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    def select_tool(self, tool_name: str, variant_key: str = "", **kwargs: Any) -> int:
        """Select a tool — refused for any tool the runtime is not fitted with,
        matching the runtime's own rule (``server.rs``: par6d is built around
        one fitted gripper and rejects SELECT_TOOL for any other).

        No par6 tool declares variants: the vendor CAD fuses the gripper body
        into the arm's final link mesh, so there are no per-variant mesh sets
        to swap. ``variant_key`` is carried anyway because the runtime carries
        it — it reappears in STATUS's ``tool_status.variant_key`` and clears
        the TCP offset when it changes — but it selects no geometry here, and
        a key naming a variant that does not exist is warned about rather than
        silently absorbed.
        """
        self._flush_quietly()
        key = _cfg.canonical_tool_key(tool_name)
        if variant_key and not getattr(self._tools.get(key), "variants", ()):
            logger.warning(
                "tool %r declares no variants; variant_key=%r selects no "
                "geometry and only rides through to STATUS",
                tool_name,
                variant_key,
            )
        fitted = _cfg.fitted_tool_key(self._config)
        if key != fitted:
            raise make_error(
                ErrorCode.COMM_VALIDATION_ERROR,
                detail=(
                    f"tool '{tool_name}' is not fitted: this runtime is built "
                    f"around '{fitted}'"
                ),
            )
        self._tool_key = key
        self._variant_key = variant_key
        self._tcp_offset_mm = (0.0, 0.0, 0.0)
        self._sync_context()
        self._completed += 1
        return 0

    def set_tcp_offset(
        self, x: float = 0, y: float = 0, z: float = 0, **kwargs: Any
    ) -> int:
        """TCP offset in mm, composed on top of the tool transform."""
        self._tcp_offset_mm = (float(x), float(y), float(z))
        self._sync_context()
        return 1

    def select_profile(self, profile: str, **kwargs: Any) -> int:
        name = profile.strip().upper()
        if name not in Preview.profiles():
            raise make_error(ErrorCode.SYS_PROFILE_INVALID, detail=profile)
        self._profile = name
        self._sync_context()
        return 1

    def set_shapes(self, shapes: list[Shape], **kwargs: Any) -> int:
        """Replace the preview's keep-outs — and enforce them, as the
        runtime enforces the set the live client sends."""
        wire = []
        for shape in shapes:
            kind, params, pose, collision, margin, name = shape.to_wire()
            wire.append(
                {
                    "kind": kind,
                    "params": [float(p) for p in params],
                    "pose": [float(p) for p in pose],
                    "collision": bool(collision),
                    "margin": float(margin) if margin is not None else None,
                    "name": name,
                }
            )
        self._call(self._preview.set_shapes, "program", wire)
        self._shapes = tuple(shapes)
        return 1

    def set_payload(
        self,
        mass: float,
        com: tuple[float, float, float] = (0.0, 0.0, 0.0),
        inertia: tuple[float, float, float, float, float, float] | None = None,
        **kwargs: Any,
    ) -> int:
        """Declare the payload the preview plans with — refused (as the
        runtime refuses it) for a negative mass or a non-PSD inertia."""
        self._call(
            self._preview.set_payload,
            float(mass),
            [float(v) for v in com],
            _inertia6(inertia),
        )
        return 1

    def payload(self, **kwargs: Any) -> dict:
        """The payload the preview plans with: ``mass``, ``com``,
        ``inertia`` (zeros = none), the live query's shape."""
        return dict(self._preview.payload())

    def _sync_context(self) -> None:
        self._call(
            self._preview.set_context,
            self._profile,
            list(self._tcp_offset_mm),
            self._policy,
        )

    def tool_action(
        self, tool_key: str, action: str, params: list | None = None, **kwargs: Any
    ) -> DryRunResultData:
        """A tool action, admitted by the engine exactly as the runtime
        admits it: the verb must be one the driver has, the parameters
        must fit it, and a jaw move needs a calibrated gripper.  The arm
        holds still while the tool works; a ``calibrate`` carries the
        runtime's minimum calibration wait, the other verbs no time (the
        config carries no jaw speed model to predict travel from)."""
        cmd = {
            "type": "tool_action",
            "tool_key": _cfg.canonical_tool_key(tool_key),
            "action": action.strip().lower(),
            "params": list(params or []),
        }
        return self._emit_batch(cmd)

    # ------------------------------------------------------------------
    # Commands with no path of their own
    # ------------------------------------------------------------------

    def _flush_quietly(self) -> None:
        """Close the hold for a command that only reconfigures state; the
        motion it releases rides at the head of the next result."""
        owed = self._close_chain()
        self._pending.extend(owed)

    def checkpoint(self, label: str, **kwargs: Any) -> int:
        self._flush_quietly()
        self._last_checkpoint = label
        self._completed += 1
        return 0

    def delay(self, seconds: float = 0.0, **kwargs: Any) -> int:
        """A queued wait: the arm holds still for *seconds*, which the
        preview's timeline carries at the head of the next result."""
        if seconds <= 0:
            raise ValueError("Delay must be positive")
        self._flush_quietly()
        self._pending.extend(
            self._run_batch([{"type": "delay", "seconds": float(seconds)}])
        )
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

    def command_verdict(self, command_index: int = -1, **kwargs: Any) -> int | None:
        """Always None: a dry run has no jaw physics to produce a settle
        verdict, so a preview never claims an object was (or wasn't)
        caught."""
        return None

    def wait_checkpoint(self, label: str = "", **kwargs: Any) -> bool:
        return True

    def wait_ready(self, **kwargs: Any) -> bool:
        return True

    def wait_status(
        self, predicate: Callable[[StatusBuffer], bool], timeout: float = 5.0
    ) -> bool:
        """*predicate* over the previewed state, in the buffer shape the live
        stream delivers — answered at once, since nothing here changes
        between commands."""
        return bool(predicate(self._status_buffer()))

    def stream_status(self) -> Iterator[StatusBuffer]:
        """One snapshot of the previewed state; the stream a program iterates
        live has no cadence here."""
        yield self._status_buffer()

    def pause(self, **kwargs: Any) -> int:
        """A preview has no executing trajectory to hold."""
        return 1

    def resume(self, **kwargs: Any) -> int:
        """A preview has no held trajectory to continue."""
        return 1

    def set_gravity_comp(self, on: bool = True, **kwargs: Any) -> int:
        """Gravity feed-forward changes what the drives are asked for, not
        where the planner sends the arm, so a preview plans the same path."""
        return 1

    def freedrive(self, enabled: bool = True, **kwargs: Any) -> int:
        """Hand guiding changes what the drives are asked for, not where
        the planner sends the arm, so a preview plans the same path."""
        return 1

    def is_freedrive(self, **kwargs: Any) -> bool:
        """A preview has no arm to push around."""
        return False

    def simulator(self, enabled: bool = True, **kwargs: Any) -> int:
        """A preview has no bus to switch; it is always the simulator."""
        return 1

    def connect_hardware(self, port_str: str = "", **kwargs: Any) -> int:
        """A preview has no bus to connect."""
        return 1

    def enter_flashing(self, assertion: str = "", **kwargs: Any) -> int:
        """A preview has no bus to silence."""
        return 1

    def exit_flashing(self, **kwargs: Any) -> int:
        """A preview has no bus to wake."""
        return 1

    def set_pid_gains(self, node: int = 0, **kwargs: Any) -> int:
        """Drive tuning changes how the real arm tracks, not where the
        planner sends it, so a preview plans the same path."""
        return 1

    def set_completion_policy(self, policy: Any = 0, **kwargs: Any) -> int:
        """When a queued command reports COMPLETE — a timing choice about
        acknowledgement, not about the path, so the preview only checks that
        the policy exists."""
        try:
            self._policy = int(CompletionPolicy(int(policy)))
        except ValueError:
            raise make_error(
                ErrorCode.COMM_VALIDATION_ERROR,
                detail=f"unknown completion policy {policy!r}",
            ) from None
        self._sync_context()
        return 1

    def set_recipe(self, name: str = "", **kwargs: Any) -> int:
        """Recipes name a settings bundle the runtime holds; a preview has
        none, so it validates the name and carries on."""
        if not 1 <= len(name) <= 64:
            raise make_error(
                ErrorCode.COMM_VALIDATION_ERROR,
                detail="set_recipe.name: length must be in 1..=64",
            )
        return 1

    def reset_loop_stats(self, **kwargs: Any) -> int:
        """A preview runs no control loop."""
        return 1

    def bind_tools(self, specs: Iterable[ToolSpec]) -> None:
        """Replace the tool specs ``tool`` answers from."""
        self._tools = {spec.key: spec for spec in specs}

    def write_io(self, index: int = 0, value: int = 0, **kwargs: Any) -> int:
        """Drive one declared output, and remember the level.

        A preview owns no pins, but it does own the readback: the level
        shows up in :meth:`io` exactly where the runtime would put it, so a
        program that sets an output and then reads it back behaves the same
        against both.  The bound is the live client's own check, with its
        exception."""
        outputs = len(self._io_outputs)
        if not 0 <= int(index) < outputs:
            raise ValueError(f"Output index must be in 0..{outputs - 1}")
        if int(value) not in (0, 1):
            raise ValueError("I/O value must be 0 or 1")
        self._io_levels[int(index)] = 1 if value else 0
        return 1

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    def io(self) -> list[int]:
        """``inputs ++ outputs ++ [estop]``, the runtime's own layout.

        A preview reads no lines, so every input is low — which is what an
        unwired input reads on the box too — and it never latches an e-stop,
        so the last slot reads clear. The outputs are whatever
        :meth:`write_io` last set."""
        return [0] * len(self._io_inputs) + list(self._io_levels) + [1]

    def queue(self) -> list[str]:
        """A preview runs each command as it arrives, so nothing is ever
        waiting; a held blend chain is the one exception."""
        return [cmd["type"] for cmd in self._held]

    def queue_state(self) -> QueueResult:
        return QueueResult(
            queue=self.queue(),
            executing_index=-1,
            completed_index=self._completed,
            last_checkpoint=self._last_checkpoint,
            queued_duration=0.0,
        )

    def error(self) -> RobotError | None:
        """A preview RAISES its refusals rather than latching them, so there
        is never a standing error to read back."""
        return None

    def ping(self, **kwargs: Any) -> PingResult:
        """A preview always answers, and never has hardware behind it."""
        return PingResult(hardware_connected=False)

    def tools(self) -> ToolResult:
        return ToolResult(tool=self._tool_key, available=sorted(self._tools))

    def activity(self) -> ActivityResult:
        """Between commands a preview is always idle."""
        return ActivityResult(state=WActionState.IDLE, command="", params="", error="")

    def reachable(self) -> ReachableResult:
        """Every joint and axis is enabled: a preview enforces the soft
        window on the path, not through a per-axis inhibit."""
        ones = [1] * NUM_JOINTS
        return ReachableResult(
            joint_en=ones, cart_en_wrf=list(ones), cart_en_trf=list(ones)
        )

    def loop_stats(self) -> LoopStatsResult | None:
        """None: a preview runs no control loop to report on."""
        return None

    def config_info(self) -> dict:
        """The effective configuration in the live query's shape, from the
        file the engine loaded."""
        joints = []
        for j in self._config["joints"]:
            velocity, acceleration, _ = _cfg.resolve_mode_limits(j["limits"], "exec")
            joints.append(
                {
                    "soft_min_rad": j["limits"]["soft_min_rad"],
                    "soft_max_rad": j["limits"]["soft_max_rad"],
                    "velocity_rad_s": velocity,
                    "acceleration_rad_s2": acceleration,
                }
            )
        files = _cfg.config_files(self._config_path)
        return {
            "path": files["path"],
            "fingerprint": files["fingerprint"],
            "tick_dt_s": self._dt,
            "motion": self._preview.motion(),
            "joints": joints,
            "active_recipe": None,
            "recipes": [],
        }

    def config_bundle(self) -> dict:
        """The config files the engine loaded, verbatim, in the live
        query's shape."""
        return _cfg.config_files(self._config_path)

    def status(self) -> StatusResult:
        """The previewed state in the shape the STATUS query answers."""
        T = self._matrix_m().copy()
        T[:3, 3] *= 1000.0
        return StatusResult(
            pose=T.flatten().tolist(),
            angles=self.angles(),
            speeds=[0.0] * NUM_JOINTS,
            io=self.io(),
            tool_status=self._tool.status(),
        )

    def _status_buffer(self) -> StatusBuffer:
        """The previewed state in the shape the STATUS stream delivers."""
        buf = StatusBuffer()
        status = self.status()
        buf.pose[:] = status.pose
        buf.angles[:] = status.angles
        buf.io = np.asarray(status.io, dtype=np.int32)
        buf.homed = self._preview.homed()
        buf.mode = ControllerMode.IDLE
        buf.enabled = True
        buf.simulator_active = True
        buf.last_checkpoint = self._last_checkpoint
        buf.completed_index = self._completed
        buf.accepted_index = self._completed
        if status.tool_status is not None:
            buf.tool_status = status.tool_status
            buf.tool_status_present = True
        return buf

    def is_estop_pressed(self) -> bool:
        return False

    def is_robot_stopped(self, threshold_speed: float = 0.01) -> bool:
        """A preview holds still between commands: every motion it reports
        has already run to its end."""
        return True

    def joint_speeds(self) -> list[float]:
        return [0.0] * NUM_JOINTS

    def tcp_speed(self) -> float:
        return 0.0

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
    # A joint trajectory is stitched only when every leg carries one: a gap
    # would splice non-adjacent configurations into one continuous path.
    trajectories = [r.joint_trajectory_rad for r in drawn]
    present = [t for t in trajectories if t is not None]
    return DryRunResultData(
        tcp_poses=np.vstack([r.tcp_poses for r in drawn]),
        end_joints_rad=drawn[-1].end_joints_rad,
        duration=sum(r.duration for r in results),
        error=error,
        valid=valid,
        joint_trajectory_rad=(
            np.vstack(present) if len(present) == len(trajectories) else None
        ),
    )


__all__ = ["DryRunRobotClient", "make_error"]
