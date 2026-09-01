"""Synchronous facade over :class:`AsyncRobotClient`.

Coroutines run on a module-level background event loop in a daemon thread —
the standard sync-facade pattern.  In async code construct an
``AsyncRobotClient`` directly and ``await`` it instead; calling this facade
from inside a running event loop raises to prevent deadlocks.
"""

from __future__ import annotations

import asyncio
import atexit
import threading
from collections.abc import Callable, Coroutine, Iterable
from typing import Any, TypeVar

from waldoctl.shapes import Shape, ShapeWorld
from waldoctl.status import ActivityResult, PingResult, ToolResult
from waldoctl.sync_tools import make_sync_tool
from waldoctl.tools import ToolSpec, ToolStatus
from waldoctl.types import Axis, Frame

from ..protocol.constants import CompletionPolicy
from ..protocol.wire import StatusBuffer
from .async_client import (
    AsyncRobotClient,
    LoopStatsResult,
    QueueResult,
    ReachableResult,
    StatusResult,
)
from .errors import RobotError

T = TypeVar("T")

_LOOP: asyncio.AbstractEventLoop | None = None
_THREAD: threading.Thread | None = None
_LOOP_READY = threading.Event()


def _loop_worker(loop: asyncio.AbstractEventLoop) -> None:
    asyncio.set_event_loop(loop)
    _LOOP_READY.set()
    loop.run_forever()


def _stop_loop() -> None:
    global _LOOP, _THREAD
    loop, thread = _LOOP, _THREAD
    if loop is None:
        return

    async def _shutdown() -> None:
        tasks = [t for t in asyncio.all_tasks(loop) if t is not asyncio.current_task()]
        for task in tasks:
            task.cancel()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        loop.stop()

    try:
        asyncio.run_coroutine_threadsafe(_shutdown(), loop)
        if thread is not None:
            thread.join(timeout=2.0)
    except (RuntimeError, asyncio.InvalidStateError):
        pass  # loop already stopped or thread not joinable
    _LOOP = None
    _THREAD = None


def _ensure_loop() -> asyncio.AbstractEventLoop:
    global _LOOP, _THREAD
    if _LOOP is None:
        _LOOP = asyncio.new_event_loop()
        _THREAD = threading.Thread(
            target=_loop_worker, args=(_LOOP,), name="par6-sync-loop", daemon=True
        )
        _THREAD.start()
        _LOOP_READY.wait(timeout=1.0)
        atexit.register(_stop_loop)
    return _LOOP


def _run(coro: Coroutine[Any, Any, T]) -> T:
    """Run *coro* to completion on the background loop and return its result."""
    try:
        asyncio.get_running_loop()
    except RuntimeError:
        loop = _ensure_loop()
        return asyncio.run_coroutine_threadsafe(coro, loop).result()
    coro.close()
    raise RuntimeError(
        "RobotClient was used while an event loop is running.\n"
        "Construct an AsyncRobotClient in this loop and `await` it instead."
    )


def _sync_tool(tool: ToolSpec) -> ToolSpec:
    """waldoctl's sync wrapper for *tool*, with ``status()`` made synchronous.

    The wrapper overrides the action verbs but not ``status()``, and a tool
    with no action verbs is not wrapped at all — either way ``status()``
    reaches the async implementation and hands the caller an un-awaited
    coroutine, so ``rbt.tool.status().key`` raises on a facade whose whole
    contract is that nothing is a coroutine.
    """
    sync = make_sync_tool(tool, _run)
    # Deliberately narrowing an async method to a sync one, which is what
    # waldoctl's own sync wrappers do to every action verb.
    sync.status = lambda: _run(tool.status())  # ty: ignore[invalid-assignment]
    return sync


class RobotClient:
    """Synchronous wrapper around :class:`AsyncRobotClient` — every method
    returns a concrete result, never a coroutine.

    Usable as a context manager::

        with RobotClient() as rbt:
            rbt.home(wait=True)
    """

    def __init__(
        self,
        host: str | None = None,
        port: int | None = None,
        timeout: float = 2.0,
        retries: int = 1,
        **kwargs: Any,
    ) -> None:
        self._inner = AsyncRobotClient(
            host=host, port=port, timeout=timeout, retries=retries, **kwargs
        )
        self._bound_tools: dict[str, ToolSpec] = {
            key: _sync_tool(tool) for key, tool in self._inner._bound_tools.items()
        }

    # ---------- lifecycle ----------

    def close(self) -> None:
        """Close the underlying async client and release resources."""
        _run(self._inner.close())

    def __enter__(self) -> "RobotClient":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    @property
    def host(self) -> str:
        return self._inner.host

    @property
    def port(self) -> int:
        return self._inner.port

    @property
    def status_seq_gaps(self) -> int:
        """Total STATUS packets lost so far (header ``seq`` gap detection)."""
        return self._inner.status_seq_gaps

    # ---------- tools ----------

    def bind_tools(self, specs: Iterable[ToolSpec]) -> None:
        """Bind tool specs; actions run through this facade's background loop."""
        self._inner.bind_tools(specs)
        self._bound_tools = {
            key: _sync_tool(tool) for key, tool in self._inner._bound_tools.items()
        }

    @property
    def tool(self) -> ToolSpec:
        """The active bound tool.  Raises ``RuntimeError`` if no tool is set."""
        key = (self._inner._active_tool_key or "").upper()
        if not key:
            raise RuntimeError("No tool set. Call select_tool() first.")
        return self._bound_tools[key]

    # ---------- motion ----------

    def home(self, wait: bool = False, timeout: float = 60.0) -> int:
        """Move to the home position.  Returns the command index, -1 on failure."""
        return _run(self._inner.home(wait=wait, timeout=timeout))

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
        wait: bool = True,
        timeout: float = 10.0,
    ) -> int:
        """Joint-space move (blocking by default).  See the async client."""
        return _run(
            self._inner.move_j(
                angles,
                pose=pose,
                duration=duration,
                speed=speed,
                accel=accel,
                r=r,
                rel=rel,
                wait=wait,
                timeout=timeout,
            )
        )

    def move_l(
        self,
        pose: list[float],
        *,
        frame: Frame = "WRF",
        duration: float = 0.0,
        speed: float = 0.0,
        accel: float = 1.0,
        r: float = 0.0,
        rel: bool = False,
        wait: bool = True,
        timeout: float = 10.0,
    ) -> int:
        """Linear Cartesian move (blocking by default)."""
        return _run(
            self._inner.move_l(
                pose,
                frame=frame,
                duration=duration,
                speed=speed,
                accel=accel,
                r=r,
                rel=rel,
                wait=wait,
                timeout=timeout,
            )
        )

    def move_c(
        self,
        via: list[float],
        end: list[float],
        *,
        frame: Frame = "WRF",
        duration: float | None = None,
        speed: float | None = None,
        accel: float = 1.0,
        r: float = 0.0,
        rel: bool = False,
        wait: bool = True,
        timeout: float = 10.0,
    ) -> int:
        """Circular arc through *via* to *end* (blocking by default)."""
        return _run(
            self._inner.move_c(
                via,
                end,
                frame=frame,
                duration=duration,
                speed=speed,
                accel=accel,
                r=r,
                rel=rel,
                wait=wait,
                timeout=timeout,
            )
        )

    def move_s(
        self,
        waypoints: list[list[float]],
        *,
        frame: Frame = "WRF",
        duration: float | None = None,
        speed: float | None = None,
        accel: float = 1.0,
        rel: bool = False,
        wait: bool = True,
        timeout: float = 10.0,
    ) -> int:
        """Cubic spline through waypoints (blocking by default)."""
        return _run(
            self._inner.move_s(
                waypoints,
                frame=frame,
                duration=duration,
                speed=speed,
                accel=accel,
                rel=rel,
                wait=wait,
                timeout=timeout,
            )
        )

    def move_p(
        self,
        waypoints: list[list[float]],
        *,
        frame: Frame = "WRF",
        duration: float | None = None,
        speed: float | None = None,
        accel: float = 1.0,
        rel: bool = False,
        wait: bool = True,
        timeout: float = 10.0,
    ) -> int:
        """Process move with auto-blending (blocking by default)."""
        return _run(
            self._inner.move_p(
                waypoints,
                frame=frame,
                duration=duration,
                speed=speed,
                accel=accel,
                rel=rel,
                wait=wait,
                timeout=timeout,
            )
        )

    # ---------- streaming ----------

    def servo_j(
        self,
        angles: list[float] | None = None,
        *,
        pose: list[float] | None = None,
        speed: float = 1.0,
        accel: float = 1.0,
    ) -> int:
        """Streaming joint position target (fire-and-forget)."""
        return _run(self._inner.servo_j(angles, pose=pose, speed=speed, accel=accel))

    def servo_l(
        self, pose: list[float], *, speed: float = 1.0, accel: float = 1.0
    ) -> int:
        """Streaming linear Cartesian target (fire-and-forget)."""
        return _run(self._inner.servo_l(pose, speed=speed, accel=accel))

    def jog_j(
        self,
        joint: int = -1,
        speed: float = 0.0,
        duration: float = 0.1,
        *,
        joints: list[int] | None = None,
        speeds: list[float] | None = None,
        accel: float = 1.0,
    ) -> int:
        """Joint velocity jog (duration-watchdogged, fire-and-forget)."""
        return _run(
            self._inner.jog_j(
                joint, speed, duration, joints=joints, speeds=speeds, accel=accel
            )
        )

    def jog_l(
        self,
        frame: Frame,
        axis: Axis | None = None,
        speed: float = 0.0,
        duration: float = 0.1,
        *,
        axes: list[Axis] | None = None,
        speeds_list: list[float] | None = None,
        accel: float = 1.0,
    ) -> int:
        """Cartesian velocity jog (duration-watchdogged, fire-and-forget)."""
        return _run(
            self._inner.jog_l(
                frame,
                axis,
                speed,
                duration,
                axes=axes,
                speeds_list=speeds_list,
                accel=accel,
            )
        )

    def teleport(
        self,
        angles_deg: list[float],
        tool_positions: list[float] | None = None,
    ) -> int:
        """Instantly set joint angles (simulator only)."""
        return _run(self._inner.teleport(angles_deg, tool_positions=tool_positions))

    # ---------- control / configuration ----------

    def stop(self, clear_queue: bool = True) -> int:
        """Stop all motion; with *clear_queue* also clear the queue."""
        return _run(self._inner.stop(clear_queue=clear_queue))

    def estop(self) -> int:
        """Protective stop: latch the controller disabled until ``reset()``."""
        return _run(self._inner.estop())

    def pause(self) -> int:
        """Hold the executing trajectory; the queue survives."""
        return _run(self._inner.pause())

    def resume(self) -> int:
        """Continue a trajectory held by :meth:`pause`."""
        return _run(self._inner.resume())

    def freedrive(self, enabled: bool) -> int:
        """Enter or leave freedrive: IDLE under G(q) with no position hold."""
        return _run(self._inner.freedrive(enabled))

    def is_freedrive(self, timeout: float = 1.0) -> bool:
        """Whether the arm is floating right now, read from the broadcast."""
        return _run(self._inner.is_freedrive(timeout=timeout))

    def set_payload(
        self,
        mass: float,
        com: tuple[float, float, float] = (0.0, 0.0, 0.0),
        inertia: tuple[float, float, float, float, float, float] | None = None,
    ) -> int:
        """Declare the payload the arm is carrying at the TCP."""
        return _run(self._inner.set_payload(mass, com, inertia))

    def payload(self) -> dict | None:
        """The effective runtime payload (zeros = none)."""
        return _run(self._inner.payload())

    def set_gravity_comp(self, on: bool) -> int:
        """Apply (or stop applying) the gravity-compensation feedforward.

        With it on in IDLE the arm floats under G(q) alone with no position
        hold, which is what makes hand-guiding reachable.
        """
        return _run(self._inner.set_gravity_comp(on))

    def reset(self) -> int:
        """Clear a latched protective stop."""
        return _run(self._inner.reset())

    def reset_state(self) -> int:
        """Full controller state reset (world, tool, errors) + re-sync."""
        return _run(self._inner.reset_state())

    def simulator(self, enabled: bool) -> int:
        """Enable or disable simulator mode."""
        return _run(self._inner.simulator(enabled))

    def connect_hardware(self, port_str: str) -> int:
        """Connect to robot hardware via serial port."""
        return _run(self._inner.connect_hardware(port_str))

    def select_profile(self, profile: str) -> int:
        """Set the motion profile (e.g. ``"TOPPRA"``)."""
        return _run(self._inner.select_profile(profile))

    def select_tool(self, tool_name: str, variant_key: str = "") -> int:
        """Set the active end-effector tool on the controller."""
        return _run(self._inner.select_tool(tool_name, variant_key=variant_key))

    def set_tcp_offset(self, x: float = 0, y: float = 0, z: float = 0) -> int:
        """Set TCP offset in mm on top of the current tool transform."""
        return _run(self._inner.set_tcp_offset(x=x, y=y, z=z))

    def set_shapes(self, shapes: list[Shape]) -> int:
        """Replace the program-layer keep-out / marker shapes."""
        return _run(self._inner.set_shapes(shapes))

    def set_completion_policy(self, policy: CompletionPolicy | int) -> int:
        """Set the controller-side completion policy for queued motion."""
        return _run(self._inner.set_completion_policy(policy))

    def set_recipe(self, name: str) -> int:
        """Select the telemetry recipe (unknown names are refused)."""
        return _run(self._inner.set_recipe(name))

    def write_io(self, index: int, value: int) -> int:
        """Set digital output by logical index (0 = first output pin)."""
        return _run(self._inner.write_io(index, value))

    def tool_action(
        self,
        tool_key: str,
        action: str,
        params: list[Any] | None = None,
        *,
        wait: bool = True,
        timeout: float = 10.0,
    ) -> int:
        """Invoke a tool-specific action by key (blocking by default)."""
        return _run(
            self._inner.tool_action(
                tool_key, action, params, wait=wait, timeout=timeout
            )
        )

    def checkpoint(self, label: str) -> int:
        """Insert a checkpoint marker in the command queue."""
        return _run(self._inner.checkpoint(label))

    def delay(self, seconds: float) -> int:
        """Insert a non-blocking delay in the command queue."""
        return _run(self._inner.delay(seconds))

    def reset_loop_stats(self) -> int:
        """Reset control-loop metrics (unacked)."""
        return _run(self._inner.reset_loop_stats())

    # ---------- queries ----------

    def ping(self) -> PingResult | None:
        """Check connectivity.  None if unreachable."""
        return _run(self._inner.ping())

    def angles(self) -> list[float] | None:
        """Current joint angles in degrees."""
        return _run(self._inner.angles())

    def pose(self, frame: Frame = "WRF") -> list[float] | None:
        """Current TCP pose [x, y, z, rx, ry, rz] in mm and degrees."""
        return _run(self._inner.pose(frame=frame))

    def io(self) -> list[int] | None:
        """Digital I/O state [in1, in2, out1, out2, estop]."""
        return _run(self._inner.io())

    def joint_speeds(self) -> list[float] | None:
        """Current joint velocities in rad/s."""
        return _run(self._inner.joint_speeds())

    def status(self) -> StatusResult | None:
        """Aggregate status snapshot."""
        return _run(self._inner.status())

    def queue(self) -> list[str] | None:
        """Queued command list."""
        return _run(self._inner.queue())

    def queue_state(self) -> QueueResult | None:
        """Full QUEUE snapshot (names, indices, checkpoint, duration)."""
        return _run(self._inner.queue_state())

    def tools(self) -> ToolResult | None:
        """Current tool and available tools."""
        return _run(self._inner.tools())

    def activity(self) -> ActivityResult | None:
        """What the robot is currently doing."""
        return _run(self._inner.activity())

    def reachable(self) -> ReachableResult | None:
        """Per-joint / per-axis enablement flags."""
        return _run(self._inner.reachable())

    def error(self) -> RobotError | None:
        """Current standing error, or None."""
        return _run(self._inner.error())

    def profile(self) -> str | None:
        """Current motion profile name."""
        return _run(self._inner.profile())

    def tcp_speed(self) -> float | None:
        """TCP linear velocity in mm/s."""
        return _run(self._inner.tcp_speed())

    def is_estop_pressed(self) -> bool:
        """Whether the e-stop is engaged."""
        return _run(self._inner.is_estop_pressed())

    def is_robot_stopped(self, threshold_speed: float = 0.01) -> bool:
        """Whether every joint is below *threshold_speed* (rad/s)."""
        return _run(self._inner.is_robot_stopped(threshold_speed))

    def tcp_offset(self) -> list[float]:
        """Current TCP offset in mm [x, y, z]."""
        return _run(self._inner.tcp_offset())

    def is_simulator(self) -> bool:
        """Whether simulator mode is active."""
        return _run(self._inner.is_simulator())

    def loop_stats(self) -> LoopStatsResult | None:
        """Control-loop runtime metrics."""
        return _run(self._inner.loop_stats())

    def shapes(self) -> ShapeWorld | None:
        """The collision world the runtime is enforcing, by layer."""
        return _run(self._inner.shapes())

    def config_info(self) -> dict | None:
        """The runtime's effective configuration (path, fingerprint,
        limits, motion constants)."""
        return _run(self._inner.config_info())

    def config_bundle(self) -> dict | None:
        """The config files the runtime loaded, verbatim (robot +
        gripper TOMLs) — see ``AsyncRobotClient.config_bundle``."""
        return _run(self._inner.config_bundle())

    def _tool_status(self) -> ToolStatus | None:
        """Query tool status (internal — use ``rbt.tool.status()``)."""
        return _run(self._inner._tool_status())

    # ---------- waits ----------

    def wait_ready(self, timeout: float = 5.0, interval: float = 0.05) -> bool:
        """Poll ping() until the runtime responds or *timeout* expires."""
        return _run(self._inner.wait_ready(timeout=timeout, interval=interval))

    def wait_status(
        self, predicate: Callable[[StatusBuffer], bool], timeout: float = 5.0
    ) -> bool:
        """Wait until a status broadcast satisfies *predicate*.

        The predicate runs on the facade's background event-loop thread.
        """
        return _run(self._inner.wait_status(predicate, timeout=timeout))

    def wait_command(self, command_index: int, timeout: float = 10.0) -> bool:
        """Wait until *command_index* completes (COMPLETE push or status
        fallback).  Raises :class:`RobotError` on failure."""
        return _run(self._inner.wait_command(command_index, timeout=timeout))

    def wait_checkpoint(self, label: str, timeout: float = 30.0) -> bool:
        """Wait until the checkpoint *label* is reached."""
        return _run(self._inner.wait_checkpoint(label, timeout=timeout))

    def wait_motion(
        self,
        timeout: float = 10.0,
        settle_window: float = 0.25,
        speed_threshold: float = 0.01,
        angle_threshold: float = 0.5,
        motion_start_timeout: float = 1.0,
    ) -> bool:
        """Wait for the robot to stop moving (start-then-settle heuristic)."""
        return _run(
            self._inner.wait_motion(
                timeout=timeout,
                settle_window=settle_window,
                speed_threshold=speed_threshold,
                angle_threshold=angle_threshold,
                motion_start_timeout=motion_start_timeout,
            )
        )


__all__ = ["RobotClient"]
