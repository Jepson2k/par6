"""Offline dry-run client — the command stream previewed by the engine.

Answers the ``waldoctl.DryRunClient`` protocol so a host application can
preview a program (path segments, move targets, timing feasibility, playback
timeline) with no ``par6d`` and no hardware.

It is not a second implementation of par6: the preview engine
(`par6._par6.Preview`, over ``par6d::preview``) drives the daemon's OWN
planner against a virtual arm — same profiles, same IK, same TOPPRA
timing, same collision gate, same wire validation — so a previewed
command is planned by exactly the code that would drive the arm.  This
module owns only the Python-facing surface: waldoctl types, wire-unit
conversions, the blend hold, and the previewed tool/IO readback state.

**Blending.**  A move whose blend radius is positive is HELD, exactly as the
runtime's queue holds it, until the command that follows it decides what the
corner looks like.  The held commands are offered to the planner as one
batch, which folds them exactly as the live queue would; the whole chain's
motion is returned by the command that closes it, or by
:meth:`~DryRunRobotClient.flush` at the end of the program, which is where
the runtime's blend hold expires.  A held move therefore returns ``None``.
A streamable or a teleport cancels planned motion on the runtime, so it
discards a held chain here too.

Refusals are the runtime's own: wire validation, the not-homed gate, the
collision gate and the planner's refusals are all answered by the same
code with the same error templates, so the editor shows the failure before
the arm does.
"""

from __future__ import annotations

import logging
from typing import Any

import numpy as np
from numpy.typing import NDArray
from pinokin import so3_rpy
from waldoctl import ToolState, ToolStatus
from waldoctl.results import DryRunResultData
from waldoctl.shapes import Shape, ShapeWorld

from par6 import config as _cfg
from par6._par6 import Preview, RobotWireError, make_wire_error
from par6.client.async_client import StatusResult
from par6.client.errors import RobotError
from par6.protocol.constants import NUM_JOINTS, CompletionPolicy, ErrorCode

logger = logging.getLogger(__name__)

#: Wire axis order for ``jog_l`` / Cartesian velocity vectors.
_AXIS_INDEX: dict[str, int] = {"X": 0, "Y": 1, "Z": 2, "RX": 3, "RY": 4, "RZ": 5}

#: Minimum time the runtime holds a gripper calibration
#: (``crates/par6d/src/planner.rs::TOOL_CALIBRATE_MIN_WAIT_S``).
_TOOL_CALIBRATE_MIN_WAIT_S = 2.0


def make_error(code: ErrorCode, **params: Any) -> RobotError:
    """The runtime's structured refusal for *code* — rendered by the
    engine's own error templates, so a preview-side refusal says exactly
    what the runtime would say."""
    rendered = make_wire_error(int(code), {k: str(v) for k, v in params.items()})
    return RobotError.from_wire(rendered)


def _wire_frame(frame: str) -> int:
    if frame == "WRF":
        return 0
    if frame == "TRF":
        return 1
    raise ValueError(f"unknown frame {frame!r} (par6 supports WRF and TRF)")


def _f6(values: Any, name: str) -> list[float]:
    out = [float(v) for v in values]
    if len(out) != NUM_JOINTS:
        raise make_error(
            ErrorCode.COMM_VALIDATION_ERROR,
            detail=f"{name} requires {NUM_JOINTS} values, got {len(out)}",
        )
    return out


def _timing(
    duration: float | None, speed: float | None
) -> tuple[float | None, float | None]:
    """The waldoctl duration/speed pair (0/None = unset) as the wire's
    exactly-one-of convention.  Neither set means full profile speed."""
    d = float(duration) if duration else None
    s = float(speed) if speed else None
    if d is not None and s is not None:
        raise ValueError("duration and speed are mutually exclusive")
    if d is None and s is None:
        s = 1.0
    return d, s


def _blend(r: float | None) -> float | None:
    return float(r) if r else None


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


def _resolve_engine_paths(config: str | None = None) -> tuple[str, str]:
    """The config file + assets tree the preview engine loads.

    An explicit *config* (a daemon-fetched bundle materialized by
    :func:`par6.config.materialize_bundle`) wins; then ``PAR6_CONFIG``.
    ``PAR6_ASSETS`` takes precedence for the assets tree; otherwise the
    repo tree around an editable install, then the deploy bundle's
    install locations (``/etc/par6`` + ``/usr/share/par6``, what
    ``scripts/deploy/install.sh`` stages on a control box) — so a wheel
    installed next to a deployed daemon previews with the exact files
    the daemon runs. The packaged ``_data`` URDFs carry rewritten mesh
    URIs the engine's loader cannot resolve, so they cannot feed it yet;
    a wheel with no daemon, repo, or env vars raises with that remedy.
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
    """``client.tool`` for the preview — routes actions through the plan.

    Jaw position is tracked here rather than left at a default: a program
    that closes the gripper and asks ``is_open()`` two lines later gets the
    answer the arm would give.
    """

    def __init__(self, client: "DryRunRobotClient") -> None:
        self._client = client
        self._position = 0.0

    def status(self) -> ToolStatus:
        """The previewed tool state. Not a query — the preview knows what it
        was told to do and nothing else, so nothing is faulted or detected."""
        spec = self._client._tools[self._client.active_tool_key]
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

    def set_position(self, position: float, **kwargs: Any) -> DryRunResultData:
        return self._act("set_position", [position], position)

    def open(self, **kwargs: Any) -> DryRunResultData:
        return self._act("open", [], 0.0)

    def close(self, **kwargs: Any) -> DryRunResultData:
        return self._act("close", [], 1.0)

    def _act(self, verb: str, params: list[Any], position: float) -> DryRunResultData:
        result = self._client.tool_action(self._client.active_tool_key, verb, params)
        self._position = min(max(float(position), 0.0), 1.0)
        return result

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
        config_path: str | None = None,
    ) -> None:
        from par6.tools import build_tools

        config, assets = _resolve_engine_paths(config_path)
        self._preview = Preview(config=config, assets=assets)
        self._dt = self._preview.tick_dt_s()
        self._max_points = max(2, int(max_snapshot_points))
        # The runtime's own startup context (planner DEFAULT_PROFILE +
        # the server's boot completion policy), pushed to the engine NOW:
        # a mirror the engine never heard of would report one profile
        # while planning with another until the first context-changing
        # call happened to sync them.
        self._profile = "RUCKIG"
        self._policy = int(CompletionPolicy.SETTLED)

        if initial_joints_deg is not None:
            q = np.radians(np.asarray(initial_joints_deg, dtype=np.float64))
            self._preview.teleport_rad(q.tolist())
        self._preview.set_homed(bool(initial_homed))

        self._tools = {spec.key: spec for spec in build_tools().available}
        self._tool_key = _cfg.fitted_tool_key()
        self._variant_key = ""
        self._tcp_offset_mm = (0.0, 0.0, 0.0)
        self._shapes: tuple[Shape, ...] = ()
        self._tool = _DryRunTool(self)
        self._held: list[dict] = []
        self._io_inputs, self._io_outputs = _cfg.io_line_names()
        self._io_levels = [0] * len(self._io_outputs)
        self._sync_context()

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

    def angles(self) -> list[float]:
        """Simulated joint angles in degrees."""
        return np.degrees(self._q()).tolist()

    def pose(self) -> list[float]:
        """Simulated TCP pose ``[x, y, z, rx, ry, rz]`` in mm + degrees."""
        si = _matrix_to_si_pose(self._call(self._preview.pose))
        return [*(si[:3] * 1000.0), *np.degrees(si[3:])]

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

        Trajectories are downsampled to ``max_snapshot_points`` (endpoints
        kept); an empty trajectory (a command that moves nothing) reports
        one sample at the pose the arm holds.
        """
        traj = np.asarray(r["joint_trajectory_rad"], dtype=np.float64)
        poses = r["tcp_poses"]
        if traj.size == 0:
            return DryRunResultData(
                tcp_poses=self._si_pose_now()[np.newaxis, :],
                end_joints_rad=np.asarray(r["end_joints_rad"], dtype=np.float64),
                duration=float(r["duration_s"]),
                joint_trajectory_rad=self._q()[np.newaxis, :].copy(),
            )
        stride = max(1, len(traj) // self._max_points)
        idx = list(range(0, len(traj), stride))
        if idx[-1] != len(traj) - 1:
            idx.append(len(traj) - 1)
        sampled = traj[idx]
        sampled_poses = np.stack(
            [_matrix_to_si_pose(poses[i]) for i in idx if i < len(poses)]
        )
        return DryRunResultData(
            tcp_poses=sampled_poses,
            end_joints_rad=np.asarray(r["end_joints_rad"], dtype=np.float64),
            duration=float(r["duration_s"]),
            joint_trajectory_rad=sampled,
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
        results = self._preview.preview_program(cmds)
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
        return out

    # ------------------------------------------------------------------
    # Blend chain
    # ------------------------------------------------------------------

    def _queue_move(self, cmd: dict, held: bool) -> DryRunResultData | None:
        """Hold *cmd* for blending, or run the batch it completes.

        The runtime's queue holds a move whose radius is positive until its
        successor arrives; the planner folds the batch exactly as the live
        queue would, so nothing here decides what folds — only when the
        batch is offered.
        """
        self._held.append(cmd)
        if held:
            return None
        batch, self._held = self._held, []
        results = self._run_batch(batch)
        return _merge(results) if results else None

    def _discard_chain(self) -> None:
        """Drop a held chain unplanned.

        A streamable or a teleport cancels planned motion on the runtime
        (``cancel_planned``), pending queue included, so a move still waiting
        for the successor it would blend into never runs.
        """
        self._held = []

    def _close_chain(self) -> list[DryRunResultData]:
        """Run whatever the blend hold still holds, as the runtime's hold
        expiry would, and return its results."""
        if not self._held:
            return []
        batch, self._held = self._held, []
        return self._run_batch(batch)

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
            ready = _cfg.homing_ready_pose_rad()
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
        if len(angles_deg) != NUM_JOINTS:
            raise make_error(
                ErrorCode.COMM_VALIDATION_ERROR,
                detail=f"teleport requires {NUM_JOINTS} angles, got {len(angles_deg)}",
            )
        config = _cfg.load_robot_config()
        hard_deg = np.degrees(
            np.array(
                [
                    [j["limits"]["hard_min_rad"], j["limits"]["hard_max_rad"]]
                    for j in config["joints"]
                ]
            )
        )
        deg = np.asarray(angles_deg, dtype=np.float64)
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
        sweeps the path at ONE speed without stopping. ``speed`` is a
        fraction of the fastest constant speed the joints allow on this
        path. With ``rel=True``, every waypoint is a delta from the start
        pose."""
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
        target is previewed with the planner's own joint move.  A streamable
        cancels planned motion on the runtime, so a move held for blending
        is dropped rather than run behind this one.
        """
        self._discard_chain()
        if pose is not None:
            cmd: dict = {
                "type": "move_j_pose",
                "pose": _f6(pose, "pose"),
                "duration": None,
                "speed": min(float(speed), 1.0) or 1.0,
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
                "speed": min(float(speed), 1.0) or 1.0,
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
        r = self._preview.preview_jog(fractions, float(duration), float(accel))
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
        if not np.isfinite(velocities).all() or np.abs(velocities).max() > 1.0:
            raise make_error(
                ErrorCode.COMM_VALIDATION_ERROR,
                detail="jog_l.velocities: each entry must be a finite fraction in [-1, 1]",
            )
        from par6.protocol.wire import MAX_JOG_DURATION_S

        if not 0.0 < float(duration) <= MAX_JOG_DURATION_S:
            raise make_error(
                ErrorCode.COMM_VALIDATION_ERROR,
                detail=f"jog_l.duration: must be in (0, {MAX_JOG_DURATION_S}] s",
            )

        robot = self._kin_robot()
        soft = _cfg.soft_limits_rad()
        motion = self._preview.motion()
        twist = np.concatenate(
            [
                velocities[:3] * motion["jog_l_linear_max_m_s"],
                velocities[3:] * motion["jog_l_angular_max_rad_s"],
            ]
        )
        ticks = max(int(round(float(duration) / self._dt)), 1)
        q = self._q()
        path = []
        for _ in range(ticks):
            v = twist.copy()
            if frame == "TRF":
                T = np.asarray(self._call(self._preview.pose), dtype=np.float64)
                rot = T.reshape(4, 4)[:3, :3]
                v[:3] = rot @ v[:3]
                v[3:] = rot @ v[3:]
            q_dot = np.linalg.pinv(robot.jacobian(q)) @ v
            q = np.clip(q + q_dot * self._dt, soft[:, 0], soft[:, 1])
            path.append(q.copy())
        end = path[-1]
        self._preview.teleport_rad(end.tolist())
        stride = max(1, len(path) // self._max_points)
        sampled = np.stack(path)[::stride]
        return DryRunResultData(
            tcp_poses=np.stack([robot.fk_vec(qk) for qk in sampled]),
            end_joints_rad=end.copy(),
            duration=len(path) * self._dt,
            joint_trajectory_rad=sampled,
        )

    def _kin_robot(self):
        """The package's pinokin model, for the two streamed-jog helpers that
        need a Jacobian (the engine previews queued motion and joint jogs)."""
        if not hasattr(self, "_robot_kin"):
            from par6.robot import Robot

            self._robot_kin = _KinFacade(Robot())
        return self._robot_kin

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
        self._tcp_offset_mm = (0.0, 0.0, 0.0)
        self._sync_context()
        return 0

    def set_tcp_offset(
        self, x: float = 0, y: float = 0, z: float = 0, **kwargs: Any
    ) -> int:
        """TCP offset in mm, composed on top of the tool transform. Queued
        on the runtime, so it closes the chain before it: the moves ahead
        of it keep the old frame, the ones after it get the new one."""
        self._flush_quietly()
        self._tcp_offset_mm = (float(x), float(y), float(z))
        self._sync_context()
        return 0

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
        """A gripper action: the arm holds still while the tool works.

        Duration is only what the config supports.  A ``move`` finishes when
        the driver reports it did — the config carries no jaw speed model to
        predict that from, so it contributes no time; ``stop`` and ``idle``
        settle the same way.  A ``calibrate`` runs the driver's homing
        sequence, which the runtime holds for at least
        ``TOOL_CALIBRATE_MIN_WAIT_S`` (``crates/par6d/src/planner.rs``).
        """
        verb = action.strip().lower()
        if verb in ("open", "close", "set_position"):
            verb = "move"
        if verb not in ("move", "calibrate", "stop", "idle"):
            raise make_error(
                ErrorCode.COMM_VALIDATION_ERROR,
                detail=(
                    f"unknown tool action {action!r} (move, calibrate, stop, idle)"
                ),
            )
        pending = self._close_chain()
        result = DryRunResultData(
            tcp_poses=self._si_pose_now()[np.newaxis, :],
            end_joints_rad=self._q().copy(),
            duration=_TOOL_CALIBRATE_MIN_WAIT_S if verb == "calibrate" else 0.0,
            joint_trajectory_rad=self._q()[np.newaxis, :].copy(),
        )
        return result if not pending else _merge([*pending, result])

    # ------------------------------------------------------------------
    # Commands with no effect on an offline plan
    # ------------------------------------------------------------------

    def _flush_quietly(self) -> None:
        """Close the hold for a command that only reconfigures state; the
        motion it releases is dropped (the caller asked no path back)."""
        self._close_chain()

    def checkpoint(self, label: str, **kwargs: Any) -> int:
        self._flush_quietly()
        return 0

    def delay(self, seconds: float = 0.0, **kwargs: Any) -> int:
        self._flush_quietly()
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

    def write_io(self, index: int = 0, value: int = 0, **kwargs: Any) -> int:
        """Drive one declared output, and remember the level.

        A preview owns no pins, but it does own the readback: the level
        shows up in :meth:`io` exactly where the runtime would put it, so a
        program that sets an output and then reads it back behaves the same
        against both."""
        if not 0 <= int(index) <= 7 or int(value) not in (0, 1):
            raise make_error(
                ErrorCode.COMM_VALIDATION_ERROR,
                detail="write_io: port must be 0..=7 and value 0 or 1",
            )
        if not 0 <= index < len(self._io_outputs):
            raise make_error(
                ErrorCode.COMM_VALIDATION_ERROR,
                detail=(
                    f"write_io port {index} does not exist: this box declares "
                    f"{len(self._io_outputs)} digital output(s)"
                ),
            )
        self._io_levels[index] = 1 if value else 0
        return 1

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

    def error(self) -> RobotError | None:
        """A preview RAISES its refusals rather than latching them, so there
        is never a standing error to read back."""
        return None

    def status(self) -> StatusResult:
        """The previewed state in the shape the STATUS query answers."""
        T = np.asarray(self._call(self._preview.pose), dtype=np.float64).reshape(4, 4)
        T = T.copy()
        T[:3, 3] *= 1000.0
        return StatusResult(
            pose=T.flatten().tolist(),
            angles=self.angles(),
            speeds=[0.0] * NUM_JOINTS,
            io=self.io(),
            tool_status=self._tool.status(),
        )

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


class _KinFacade:
    """The slice of :class:`par6.robot.Robot` the cartesian jog needs."""

    def __init__(self, robot: Any) -> None:
        self._robot = robot

    def jacobian(self, q: NDArray[np.float64]) -> NDArray[np.float64]:
        return self._robot.jacobian(q)

    def fk_vec(self, q: NDArray[np.float64]) -> NDArray[np.float64]:
        out = np.zeros(6, dtype=np.float64)
        self._robot.fk(q, out)
        return out


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
