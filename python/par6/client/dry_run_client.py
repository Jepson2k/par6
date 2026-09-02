"""Offline preview client — the waldoctl ``DryRunClient`` over the engine's
dry run (``par6._par6.Preview``, the ``par6d::preview`` engine).

Every method builds the same wire command the live client would send and
hands it to the engine, which validates, gates, holds for blending, plans
and collision-checks it with the daemon's own code.  This layer only
adapts arguments (the waldoctl conventions: mm, degrees, duration-or-speed)
and results (numpy arrays for ``DryRunResultData``).  A refusal is raised
as :class:`RobotError` with the runtime's own text; the one refusal that is
returned rather than raised is ``IK_PARTIAL_PATH`` — a preview wants to
show how far a line gets, and the caller reads that off the result's
``error``.
"""

from __future__ import annotations

import copy
import logging
from collections.abc import Coroutine
from pathlib import Path
from typing import Any

import numpy as np
from waldoctl.results import DryRunResultData
from waldoctl.shapes import Shape, ShapeWorld
from waldoctl.sync_tools import make_sync_tool
from waldoctl.tools import ToolState as WToolState
from waldoctl.tools import ToolStatus

from par6 import config as _cfg
from par6._par6 import Preview, RobotWireError
from par6.protocol.constants import NUM_JOINTS, CompletionPolicy, ErrorCode

from ._wire import (
    blend,
    f6,
    jog_j_speeds,
    jog_l_velocities,
    shape_to_wire,
    timing,
    wire_frame,
)
from .async_client import StatusResult
from .errors import RobotError

logger = logging.getLogger(__name__)


def _resolve_engine_paths(config: str | None = None) -> tuple[str, str]:
    """``(robot TOML, assets tree)`` for the preview engine.

    An explicit *config* (a daemon-fetched bundle materialized by
    :func:`par6.config.materialize_bundle`) wins; then ``PAR6_CONFIG``.
    ``PAR6_ASSETS`` takes precedence for the assets tree; otherwise the
    repo tree around an editable install, then the deploy bundle's
    install locations (``/etc/par6`` + ``/usr/share/par6``, what
    ``scripts/deploy/install.sh`` stages on a control box) — so a wheel
    installed next to a deployed daemon previews with the exact files
    the daemon runs. The packaged ``_data`` tree names its meshes under
    the ``par6`` package, which the engine's assets loader does not
    search, so a wheel with no daemon, repo, or env vars raises with
    that remedy.
    """
    import os

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


def _drive(coro: Coroutine[Any, Any, Any]) -> Any:
    """Run a tool verb's coroutine to completion.

    The preview answers every call at once, so the coroutine finishes on
    its first step — no event loop is involved, which keeps the dry run
    usable from a worker process and from inside a host's running loop.
    """
    try:
        coro.send(None)
    except StopIteration as done:
        return done.value
    coro.close()
    raise RuntimeError("a dry-run tool verb suspended; the preview never awaits")


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
        self._preview = Preview(
            config=config, assets=assets, max_points=max(2, int(max_snapshot_points))
        )
        if initial_joints_deg is not None:
            self._preview.place_rad(
                np.radians(np.asarray(initial_joints_deg, dtype=np.float64)).tolist()
            )
        self._preview.set_homed(bool(initial_homed))
        self._shapes: tuple[Shape, ...] = ()
        # The packaged tool specs, bound to this preview: ``tool.close()``
        # sends the same ``move`` verb and parameters the live gripper gets.
        # Typed Any: waldoctl's sync wrapper narrows every async verb to a
        # plain call, which the ToolSpec annotation cannot express.
        self._tools: dict[str, Any] = {}
        for spec in build_tools().available:
            bound: Any = copy.copy(spec)
            bound._execute = self._tool_execute
            bound._get_status = self._tool_status
            sync: Any = make_sync_tool(bound, _drive)
            sync.status = lambda b=bound: _drive(b.status())
            self._tools[spec.key] = sync

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    @property
    def active_tool_key(self) -> str:
        return _cfg.canonical_tool_key(self._preview.tool()[0])

    @property
    def tool(self) -> Any:
        """The active tool, sync-wrapped: ``tool.close()`` returns the
        previewed result."""
        return self._tools[self.active_tool_key]

    def angles(self) -> list[float]:
        """Simulated joint angles in degrees."""
        return self._preview.angles_deg()

    def pose(self) -> list[float]:
        """Simulated TCP pose ``[x, y, z, rx, ry, rz]`` in mm + degrees."""
        return self._call(self._preview.pose_xyzrpy)

    def tcp_offset(self) -> list[float]:
        """The TCP offset applied on top of the tool transform, in mm."""
        return self._preview.tcp_offset_mm()

    def shapes(self) -> ShapeWorld:
        """The preview's collision world (what this run has submitted)."""
        return ShapeWorld(installation=(), program=self._shapes)

    def profile(self) -> str:
        return self._preview.profile()

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

    def _submit(self, cmd: dict[str, Any]) -> DryRunResultData | None:
        """Submit one wire command; ``None`` while it waits in the blend
        hold, else its result.  Raises the runtime's refusal."""
        return self._result(self._preview.submit(cmd))

    def _result(self, r: dict[str, Any] | None) -> DryRunResultData | None:
        if r is None:
            return None
        error: RobotError | None = None
        if r["error"] is not None:
            error = RobotError.from_wire(r["error"])
            if error.code != ErrorCode.IK_PARTIAL_PATH:
                raise error
        traj = np.asarray(r["joint_trajectory_rad"], dtype=np.float64).reshape(
            -1, NUM_JOINTS
        )
        return DryRunResultData(
            tcp_poses=np.asarray(r["tcp_xyzrpy"], dtype=np.float64).reshape(-1, 6),
            end_joints_rad=np.asarray(r["end_joints_rad"], dtype=np.float64),
            duration=float(r["duration_s"]),
            error=error,
            joint_trajectory_rad=traj if traj.shape[0] else None,
        )

    def _system(self, cmd: dict[str, Any]) -> int:
        """A state-changing command: refused → raises, else 1."""
        self._submit(cmd)
        return 1

    def flush(self) -> list[DryRunResultData]:
        """Plan whatever the blend hold is still holding.

        A move whose radius is positive waits for the successor it rounds a
        corner into; at the end of a program that successor never comes, and
        the runtime's blend hold expires and runs the chain as it stands (the
        last move simply stops at its target).  Call this after the last
        command to collect that motion.
        """
        result = self._result(self._preview.flush())
        return [result] if result is not None else []

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    def home(self, **kwargs: Any) -> DryRunResultData | None:
        """Reference the arm (the configured seek, un-referenced) or return
        it to the park pose (already referenced) — the runtime's two
        meanings of HOME, decided by the engine.  ``calibrate=True`` raises
        NotImplementedError, as the live client does."""
        if kwargs.get("calibrate"):
            raise NotImplementedError(
                "par6d cannot re-reference a referenced arm yet; "
                "home(calibrate=True) needs a forced-homing command in the runtime"
            )
        return self._submit({"type": "home"})

    def teleport(
        self, angles_deg: list[float], tool_positions: list[float] | None = None
    ) -> DryRunResultData | None:
        """Sim-only jump to *angles_deg*, establishing the position reference.
        Refused outside a joint's travel, exactly as the runtime refuses."""
        return self._submit(
            {
                "type": "teleport",
                "angles": f6(angles_deg, "angles_deg"),
                "tool_positions": (
                    [float(p) for p in tool_positions]
                    if tool_positions is not None
                    else None
                ),
            }
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
        d, s = timing(duration, speed)
        if pose is not None:
            if rel:
                raise ValueError(
                    "move_j(pose=..., rel=True) is not supported: MOVE_J_POSE is "
                    "absolute. Compose the offset with the current TCP pose, or "
                    "use move_j(angles=..., rel=True) for a relative joint move."
                )
            return self._submit(
                {
                    "type": "move_j_pose",
                    "pose": f6(pose, "pose"),
                    "duration": d,
                    "speed": s,
                    "accel": float(accel),
                    "blend_radius": blend(r),
                }
            )
        if angles is None:
            raise ValueError("move_j requires angles or pose=")
        return self._submit(
            {
                "type": "move_j",
                "angles": f6(angles, "angles"),
                "duration": d,
                "speed": s,
                "accel": float(accel),
                "blend_radius": blend(r),
                "rel": bool(rel),
            }
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
        d, s = timing(duration, speed)
        return self._submit(
            {
                "type": "move_l",
                "pose": f6(pose, "pose"),
                "frame": wire_frame(frame),
                "duration": d,
                "speed": s,
                "accel": float(accel),
                "blend_radius": blend(r),
                "rel": bool(rel),
            }
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
        rel: bool = False,
        **kwargs: Any,
    ) -> DryRunResultData | None:
        d, s = timing(duration, speed)
        return self._submit(
            {
                "type": "move_c",
                "via": f6(via, "via"),
                "end": f6(end, "end"),
                "frame": wire_frame(frame),
                "duration": d,
                "speed": s,
                "accel": float(accel),
                "blend_radius": blend(r),
                "rel": bool(rel),
            }
        )

    def _move_multi(
        self,
        kind: str,
        waypoints: list[list[float]],
        frame: str,
        duration: float,
        speed: float,
        accel: float,
        rel: bool,
    ) -> DryRunResultData | None:
        d, s = timing(duration, speed)
        return self._submit(
            {
                "type": kind,
                "waypoints": [f6(wp, "waypoint") for wp in waypoints],
                "frame": wire_frame(frame),
                "duration": d,
                "speed": s,
                "accel": float(accel),
                "rel": bool(rel),
            }
        )

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
    ) -> DryRunResultData | None:
        return self._move_multi("move_s", waypoints, frame, duration, speed, accel, rel)

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
    ) -> DryRunResultData | None:
        return self._move_multi("move_p", waypoints, frame, duration, speed, accel, rel)

    # ------------------------------------------------------------------
    # Streaming
    # ------------------------------------------------------------------

    def servo_j(
        self,
        angles: list[float] | None = None,
        *,
        pose: list[float] | None = None,
        speed: float = 1.0,
        accel: float = 1.0,
        **kwargs: Any,
    ) -> DryRunResultData | None:
        """One streamed target, evaluated as if it were the last one: the
        settle onto it is previewed with the planner's own move."""
        if pose is not None:
            return self._submit(
                {
                    "type": "servo_j_pose",
                    "pose": f6(pose, "pose"),
                    "speed": float(speed),
                    "accel": float(accel),
                }
            )
        if angles is None:
            raise ValueError("servo_j requires angles or pose=")
        return self._submit(
            {
                "type": "servo_j",
                "angles": f6(angles, "angles"),
                "speed": float(speed),
                "accel": float(accel),
            }
        )

    def servo_l(
        self,
        pose: list[float],
        *,
        speed: float = 1.0,
        accel: float = 1.0,
        **kwargs: Any,
    ) -> DryRunResultData | None:
        return self._submit(
            {
                "type": "servo_l",
                "pose": f6(pose, "pose"),
                "speed": float(speed),
                "accel": float(accel),
            }
        )

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
    ) -> DryRunResultData | None:
        return self._submit(
            {
                "type": "jog_j",
                "speeds": jog_j_speeds(joint, speed, joints, speeds),
                "duration": float(duration),
                "accel": float(accel),
            }
        )

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
    ) -> DryRunResultData | None:
        return self._submit(
            {
                "type": "jog_l",
                "velocities": jog_l_velocities(axis, speed, axes, speeds_list),
                "duration": float(duration),
                "frame": wire_frame(frame),
                "accel": float(accel),
            }
        )

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    def select_tool(self, tool_name: str, variant_key: str = "", **kwargs: Any) -> int:
        """Select a tool — refused for any tool the runtime is not fitted
        with, matching the runtime's own rule."""
        self._submit(
            {
                "type": "select_tool",
                "tool_name": _cfg.canonical_tool_key(tool_name),
                "variant_key": variant_key or None,
            }
        )
        return 0

    def set_tcp_offset(
        self, x: float = 0, y: float = 0, z: float = 0, **kwargs: Any
    ) -> int:
        return self._system(
            {"type": "set_tcp_offset", "x": float(x), "y": float(y), "z": float(z)}
        )

    def select_profile(self, profile: str, **kwargs: Any) -> int:
        return self._system({"type": "select_profile", "profile": profile.strip()})

    def set_shapes(self, shapes: list[Shape], **kwargs: Any) -> int:
        """Replace the preview's keep-outs — and enforce them, as the
        runtime enforces the set the live client sends."""
        self._system(
            {"type": "set_shapes", "shapes": [shape_to_wire(s) for s in shapes]}
        )
        self._shapes = tuple(shapes)
        return 1

    def set_completion_policy(self, policy: Any = 0, **kwargs: Any) -> int:
        return self._system(
            {
                "type": "set_completion_policy",
                "policy": int(CompletionPolicy(int(policy))),
            }
        )

    def set_recipe(self, name: str = "", **kwargs: Any) -> int:
        return self._system({"type": "set_recipe", "name": name})

    def set_payload(
        self,
        mass: float,
        com: tuple[float, float, float] = (0.0, 0.0, 0.0),
        inertia: tuple[float, ...] | None = None,
        **kwargs: Any,
    ) -> int:
        return self._system(
            {
                "type": "set_payload",
                "mass": float(mass),
                "com": [float(v) for v in com],
                "inertia": [float(v) for v in inertia] if inertia is not None else None,
            }
        )

    def write_io(self, index: int = 0, value: int = 0, **kwargs: Any) -> int:
        """Drive one declared output; the level shows up in :meth:`io`
        exactly where the runtime would put it."""
        return self._system(
            {"type": "write_io", "port": int(index), "value": int(value)}
        )

    def tool_action(
        self, tool_key: str, action: str, params: list | None = None, **kwargs: Any
    ) -> DryRunResultData | None:
        """A gripper action, admitted by the runtime's own rules (a ``move``
        needs a calibrated gripper; ``calibrate`` holds the arm for the
        driver's minimum settle)."""
        return self._submit(
            {
                "type": "tool_action",
                "tool_key": _cfg.canonical_tool_key(tool_key),
                "action": action.strip().lower(),
                "params": list(params or []),
            }
        )

    async def _tool_execute(
        self, tool_key: str, action: str, params: list[Any], **kwargs: Any
    ) -> DryRunResultData | None:
        return self.tool_action(tool_key, action, params)

    async def _tool_status(self) -> ToolStatus:
        """The previewed tool state: what the program told it to do."""
        key, variant = self._preview.tool()
        key = _cfg.canonical_tool_key(key)
        position = self._preview.tool_position()
        spec = self._tools[key]
        return ToolStatus(
            key=key,
            variant_key=variant or "",
            state=WToolState.IDLE,
            engaged=position > 0.5,
            positions=(position,) * len(getattr(spec, "motions", ()) or (1,)),
        )

    # ------------------------------------------------------------------
    # Commands with no effect on an offline plan
    # ------------------------------------------------------------------

    def checkpoint(self, label: str, **kwargs: Any) -> int:
        self._submit({"type": "checkpoint", "label": label})
        return 0

    def delay(self, seconds: float = 0.0, **kwargs: Any) -> int:
        self._submit({"type": "delay", "seconds": float(seconds)})
        return 0

    def stop(self, clear_queue: bool = True, **kwargs: Any) -> int:
        """Halt motion.  A held blend chain is queued motion: it is dropped
        when the queue is cleared, and kept when it is not."""
        return self._system({"type": "stop", "clear_queue": bool(clear_queue)})

    def estop(self, **kwargs: Any) -> int:
        return self._system({"type": "estop"})

    def reset(self, **kwargs: Any) -> int:
        return self._system({"type": "reset"})

    def reset_state(self, **kwargs: Any) -> int:
        return self._system({"type": "reset_state"})

    def pause(self, **kwargs: Any) -> int:
        return self._system({"type": "pause", "on": True})

    def resume(self, **kwargs: Any) -> int:
        return self._system({"type": "pause", "on": False})

    def set_gravity_comp(self, on: bool = True, **kwargs: Any) -> int:
        return self._system({"type": "set_gravity_comp", "on": bool(on)})

    def freedrive(self, enabled: bool = True, **kwargs: Any) -> int:
        return self.set_gravity_comp(enabled)

    def is_freedrive(self, **kwargs: Any) -> bool:
        """A preview has no arm to push around."""
        return False

    def simulator(self, enabled: bool = True, **kwargs: Any) -> int:
        return self._system({"type": "simulator", "on": bool(enabled)})

    def connect_hardware(self, port_str: str = "", **kwargs: Any) -> int:
        return self._system({"type": "connect_hardware", "port": port_str})

    def enter_flashing(self, assertion: str, **kwargs: Any) -> int:
        return self._system({"type": "enter_flashing", "assertion": assertion})

    def exit_flashing(self, **kwargs: Any) -> int:
        return self._system({"type": "exit_flashing"})

    def set_pid_gains(self, node: int, **gains: Any) -> int:
        return self._system({"type": "set_pid_gains", "node": int(node), **gains})

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

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    def io(self) -> list[int]:
        """``inputs ++ outputs ++ [estop]``, the runtime's own layout."""
        return list(self._preview.io())

    def queue(self) -> list[str]:
        """The commands waiting in the blend hold — the only queue a
        preview ever has."""
        return self._preview.queue()

    def error(self) -> RobotError | None:
        """A preview RAISES its refusals rather than latching them, so there
        is never a standing error to read back."""
        return None

    def status(self) -> StatusResult:
        """The previewed state in the shape the STATUS query answers."""
        matrix = self._call(self._preview.pose)
        pose = list(matrix)
        for i in (3, 7, 11):
            pose[i] *= 1000.0
        return StatusResult(
            pose=pose,
            angles=self.angles(),
            speeds=[0.0] * NUM_JOINTS,
            io=self.io(),
            tool_status=_drive(self._tool_status()),
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


__all__ = ["DryRunRobotClient"]
