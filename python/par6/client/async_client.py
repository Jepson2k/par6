"""Async client for the par6d runtime — protocol v2.

Implements the waldoctl ``RobotClient`` ABC as a thin shim over the Rust
engine's client (`par6._par6.CoreClient`, the `par6-client` crate). The
transport — req_id correlation, retries with the idempotency-key dedup
contract, the multicast status ladder, chunking, COMPLETE tracking — all
lives in Rust; this layer owns the Python-facing API: waldoctl types and
result containers, unit conventions, tool binding, and the ONE shared
:class:`StatusBuffer` consumers iterate with :meth:`stream_status_shared`.

Units at this API are mm and degrees (the waldoctl convention), which is
also what the v2 wire carries — the runtime converts to SI internally.
"""

from __future__ import annotations

import asyncio
import atexit
import contextlib
import copy
import logging
import math
import os
import time
import weakref
from collections.abc import AsyncGenerator, Callable, Iterable, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np
from waldoctl import RobotClient as _RobotClientABC
from waldoctl.shapes import Shape, ShapeWorld, shape_from_wire
from waldoctl.status import (
    ActionState as WActionState,
)
from waldoctl.status import (
    ActivityResult,
    LoopStatsResult,
    PingResult,
    ToolResult,
)
from waldoctl.tools import ToolSpec, ToolStatus
from waldoctl.tools import ToolState as WToolState
from waldoctl.types import Axis
from waldoctl.types import Frame as WFrame

from par6._par6 import CoreClient, RobotWireError

from ..config import canonical_tool_key, io_line_names
from ..protocol.constants import (
    NUM_JOINTS,
    CompletionPolicy,
    Frame,
)
from ..protocol.wire import StatusBuffer, update_status_from_dict
from .errors import RobotError

logger = logging.getLogger(__name__)

_AXIS_INDEX: dict[str, int] = {"X": 0, "Y": 1, "Z": 2, "RX": 3, "RY": 4, "RZ": 5}
_FRAMES: dict[str, Frame] = {"WRF": Frame.WRF, "TRF": Frame.TRF}

#: How long one status_after poll blocks in the pump before re-checking
#: for shutdown. Frames arrive far faster than this whenever a runtime is
#: broadcasting; the value only bounds close() latency on a silent link.
_STATUS_POLL_S = 0.5


def _env_str(name: str, default: str) -> str:
    return os.environ.get(name, default)


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    return int(raw) if raw else default


def _wire_frame(frame: WFrame) -> int:
    try:
        return int(_FRAMES[frame])
    except KeyError:
        raise ValueError(
            f"unknown frame {frame!r} (par6 supports WRF and TRF)"
        ) from None


def _f6(values: Sequence[float], name: str) -> list[float]:
    if len(values) != NUM_JOINTS:
        raise ValueError(f"{name} requires {NUM_JOINTS} values, got {len(values)}")
    return [float(v) for v in values]


def _timing(
    duration: float | None, speed: float | None
) -> tuple[float | None, float | None]:
    """Map the waldoctl duration/speed pair (0/None = unset) onto the wire's
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


def _matrix_to_pose(m: Sequence[float]) -> list[float]:
    """Flattened row-major 4x4 (translation in mm) -> [x, y, z, rx, ry, rz] deg.

    RPY convention: R = Rx(rx) @ Ry(ry) @ Rz(rz) (intrinsic XYZ), the
    decomposition ``pinokin.so3_rpy`` performs -- so a pose read here is the
    pose :meth:`par6.robot.Robot.fk` reports, the pose :meth:`move_l` re-encodes
    and the pose a frontend decoding the STATUS matrix itself sees.
    """
    x, y, z = m[3], m[7], m[11]
    r00, r01, r02 = m[0], m[1], m[2]
    r10, r11, r12 = m[4], m[5], m[6]
    r22 = m[10]
    cp = math.hypot(r12, r22)
    if cp > 1e-9:
        roll = math.atan2(-r12, r22)
        yaw = math.atan2(-r01, r00)
    else:  # gimbal lock (ry = +/-90 deg): only roll -/+ yaw is observable
        roll = math.atan2(math.copysign(1.0, r02) * r10, r11)
        yaw = 0.0
    pitch = math.atan2(r02, cp)
    return [x, y, z, math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def _tool_status_from_dict(raw: dict | None) -> ToolStatus | None:
    if raw is None:
        return None
    return ToolStatus(
        key=canonical_tool_key(raw["key"]),
        variant_key=raw["variant_key"],
        state=WToolState(raw["state"]),
        engaged=raw["engaged"],
        part_detected=raw["part_detected"],
        fault_code=raw["fault_code"],
        positions=tuple(raw["positions"]),
        channels=tuple(raw["channels"]),
    )


def copy_status(buf: StatusBuffer) -> StatusBuffer:
    """Deep copy of a status buffer, safe to store (``cart_en`` aliasing kept)."""
    return copy.deepcopy(buf)


# ---------------------------------------------------------------------------
# Query result containers (queries whose payload has no waldoctl type)
# ---------------------------------------------------------------------------


@dataclass
class StatusResult:
    """Aggregate STATUS query snapshot (the broadcast stream is richer)."""

    pose: list[float]
    """Flattened 4x4 row-major TCP pose (mm)."""
    angles: list[float]
    """Joint angles (degrees)."""
    speeds: list[float]
    """Joint speeds (rad/s)."""
    io: list[int]
    """Digital I/O: the configured inputs, then the outputs, then the
    e-stop — which is ALWAYS the last element. The width follows the
    `[io]` config block, so index by role, never by a fixed position."""
    tool_status: ToolStatus | None
    """Tool status, if a tool is selected."""


@dataclass
class ReachableResult:
    """Per-joint / per-axis enablement flags (1 = motion allowed)."""

    joint_en: list[int]
    cart_en_wrf: list[int]
    cart_en_trf: list[int]


@dataclass
class QueueResult:
    """QUEUE query snapshot."""

    queue: list[str]
    executing_index: int
    completed_index: int
    last_checkpoint: str
    queued_duration: float


def _inertia6(
    inertia: tuple[float, float, float, float, float, float] | None,
) -> list[float] | None:
    return None if inertia is None else [float(v) for v in inertia]


# Engine clients whose tokio tasks are still live. Interpreter finalization
# tears the process down under those tasks and a worker then dies with a
# non-unwinding panic (SIGABRT); atexit runs while everything is still
# alive, so every leftover engine client is stopped here. Clients closed
# properly have already left the set.
_LIVE_CORES: "weakref.WeakSet[AsyncRobotClient]" = weakref.WeakSet()


@atexit.register
def _close_leftover_cores() -> None:
    for client in list(_LIVE_CORES):
        core = client._core
        client._core = None
        client._closed = True
        if core is not None:
            core.close()


class AsyncRobotClient(_RobotClientABC):
    """Async client for the par6d runtime.

    All network knobs default from the ``PAR6_*`` env namespace, then to the
    config defaults (command port 6001, status port 6002, multicast
    group 239.255.0.71).
    """

    def __init__(
        self,
        host: str | None = None,
        port: int | None = None,
        timeout: float = 1.0,
        retries: int = 1,
        *,
        status_transport: str | None = None,
        status_port: int | None = None,
        mcast_group: str | None = None,
        mcast_iface: str | None = None,
        status_unicast_host: str | None = None,
        mtu: int | None = None,
        tool_specs: Iterable[ToolSpec] | None = None,
    ) -> None:
        self._host = host or _env_str("PAR6_HOST", "127.0.0.1")
        self._port = port if port is not None else _env_int("PAR6_COMMAND_PORT", 6001)
        self.timeout = timeout
        self.retries = retries
        self._status_transport_kind = (
            status_transport or _env_str("PAR6_STATUS_TRANSPORT", "MULTICAST")
        ).upper()
        self._status_port = (
            status_port
            if status_port is not None
            else _env_int("PAR6_STATUS_PORT", 6002)
        )
        self._mcast_group = mcast_group or _env_str(
            "PAR6_STATUS_MCAST_GROUP", "239.255.0.71"
        )
        self._mcast_iface = mcast_iface or _env_str("PAR6_STATUS_MCAST_IF", "127.0.0.1")
        self._status_unicast_host = status_unicast_host or _env_str(
            "PAR6_STATUS_UNICAST_HOST", "127.0.0.1"
        )
        self.mtu = mtu if mtu is not None else _env_int("PAR6_MTU", 1400)

        self._core: CoreClient | None = None
        self._core_lock = asyncio.Lock()
        self._closed = False
        _LIVE_CORES.add(self)

        # Shared status buffer + generation/event notification, filled by
        # the pump task from the engine's STATUS stream.
        self._shared_status = StatusBuffer()
        self._status_generation = 0
        self._status_event = asyncio.Event()
        self._status_task: asyncio.Task | None = None

        self._last_command_index: int | None = None
        self._active_tool_key: str | None = None
        self._active_variant_key = ""
        self._bound_tools: dict[str, ToolSpec] = {}
        if tool_specs is None:
            # A bare client (what a user script constructs) binds the packaged
            # tools itself, so ``rbt.select_tool(...); rbt.tool.close()`` works
            # without going through the Robot factory. Imported here: par6.tools
            # reaches back into the package, which is mid-import at class
            # definition time. The factory passes its own composed set, which
            # additionally carries plugin tools.
            from par6.tools import build_tools

            tool_specs = build_tools().available
        self.bind_tools(tool_specs)

    # ------------------------------------------------------------------
    # Configuration / lifecycle
    # ------------------------------------------------------------------

    @property
    def host(self) -> str:
        return self._host

    @property
    def port(self) -> int:
        return self._port

    @property
    def status_seq_gaps(self) -> int:
        """Total STATUS packets lost so far, detected via header ``seq`` gaps."""
        return self._core.status_seq_gaps() if self._core is not None else 0

    async def _ensure_core(self) -> CoreClient:
        if self._closed:
            raise RuntimeError("AsyncRobotClient is closed")
        core = self._core
        if core is not None:
            return core
        async with self._core_lock:
            if self._closed:
                raise RuntimeError("AsyncRobotClient is closed")
            if self._core is not None:
                return self._core
            core = await CoreClient.connect(
                self._host,
                self._port,
                self.timeout,
                self.retries,
                self._status_transport_kind,
                self._status_port,
                self._mcast_group,
                self._mcast_iface,
                self._status_unicast_host,
                self.mtu,
            )
            self._core = core
            self._status_task = asyncio.create_task(self._status_pump())
            logger.info(
                "par6 client endpoint: %s:%s (status %s @ port %s)",
                self._host,
                self._port,
                self._status_transport_kind,
                self._status_port,
            )
            return core

    async def _status_pump(self) -> None:
        """Fill the ONE shared buffer from the engine's STATUS stream and
        wake every waiter, preserving the generation/event contract."""
        core = self._core
        assert core is not None
        last_seq = -1
        while not self._closed:
            try:
                frame = await core.status_after(last_seq, _STATUS_POLL_S)
            except asyncio.CancelledError:
                raise
            except Exception:
                if self._closed:
                    return
                raise
            if frame is None:
                continue
            last_seq = frame["seq"]
            update_status_from_dict(self._shared_status, frame)
            # The wire carries the config spelling of the tool key; every
            # consumer (and ``waldoctl.ToolSpec``, which upper-cases what it
            # is given) indexes tools by the canonical one.
            tool_status = self._shared_status.tool_status
            tool_status.key = canonical_tool_key(tool_status.key)
            self._status_generation += 1
            self._status_event.set()

    async def _call(self, awaitable: Any) -> Any:
        """Await an engine call, translating its structured refusals into
        :class:`RobotError` (the exception this API raises)."""
        try:
            return await awaitable
        except RobotWireError as e:
            raise RobotError.from_wire(e.args) from None

    async def close(self) -> None:
        """Release the engine client and wake all waiters.  Safe to call
        repeatedly."""
        if self._closed:
            return
        self._closed = True
        self._status_event.set()
        if self._status_task is not None:
            self._status_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._status_task
            self._status_task = None
        if self._core is not None:
            self._core.close()
            self._core = None

    async def __aenter__(self) -> "AsyncRobotClient":
        if self._closed:
            raise RuntimeError("AsyncRobotClient is closed")
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:
        await self.close()

    # ------------------------------------------------------------------
    # Tool binding (specs are injected — par6.robot wires the registry)
    # ------------------------------------------------------------------

    def bind_tools(self, specs: Iterable[ToolSpec]) -> None:
        """Bind tool specs to this client's action/status transport.

        Each spec is shallow-copied and given ``_execute``/``_get_status``
        hooks pointing at this client, so ``client.tool.open()`` etc. drive
        ``tool_action`` on the runtime.  The ``par6.robot`` factory calls this
        with the registry it builds from config.

        A spec without the hooks is refused here rather than at first use:
        setting them on a plain :class:`ToolSpec` would succeed and leave a
        tool whose verbs raise only once someone drives it.
        """
        from par6.tools import _ClientBound

        bound: dict[str, ToolSpec] = {}
        for spec in specs:
            if not isinstance(spec, _ClientBound):
                raise TypeError(
                    f"tool spec {spec.key!r} carries no dispatch hooks; par6 "
                    "specs come from par6.tools.build_tools"
                )
            bound_spec = copy.copy(spec)
            bound_spec._execute = self.tool_action
            bound_spec._get_status = self._tool_status
            bound[bound_spec.key] = bound_spec
        self._bound_tools = bound

    @property
    def tool(self) -> ToolSpec:
        """The active bound tool.  Raises ``RuntimeError`` if no tool is set."""
        key = (self._active_tool_key or "").upper()
        if not key:
            raise RuntimeError("No tool set. Call select_tool() first.")
        return self._bound_tools[key]

    # ------------------------------------------------------------------
    # Status streaming
    # ------------------------------------------------------------------

    async def stream_status_shared(self) -> AsyncGenerator[StatusBuffer, None]:
        """Async iterator over the ONE shared status buffer (zero-copy).

        The same instance is yielded every iteration and is overwritten by
        the next packet — process immediately, never store it.  Slow
        consumers skip to the latest state.  Terminates when the client is
        closed.
        """
        await self._ensure_core()
        last_gen = 0
        while not self._closed:
            self._status_event.clear()
            if self._status_generation != last_gen:
                last_gen = self._status_generation
                yield self._shared_status
                continue
            await self._status_event.wait()
            if self._closed:
                break
            if self._status_generation != last_gen:
                last_gen = self._status_generation
                yield self._shared_status

    async def stream_status(self) -> AsyncGenerator[StatusBuffer, None]:
        """Async iterator of status snapshots — yields copies, safe to store.

        For zero-copy hot paths use :meth:`stream_status_shared`.
        """
        async for status in self.stream_status_shared():
            yield copy_status(status)

    async def wait_status(
        self, predicate: Callable[[StatusBuffer], bool], timeout: float = 5.0
    ) -> bool:
        """Block until *predicate* is True for a status snapshot."""
        await self._ensure_core()
        last_gen = 0
        deadline = time.monotonic() + timeout
        while not self._closed:
            self._status_event.clear()
            if self._status_generation != last_gen:
                last_gen = self._status_generation
                try:
                    if predicate(self._shared_status):
                        return True
                except Exception:
                    logger.debug("status predicate raised", exc_info=True)
                continue
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            try:
                await asyncio.wait_for(self._status_event.wait(), timeout=remaining)
            except TimeoutError:
                return False
        return False

    # ------------------------------------------------------------------
    # Completion / synchronization
    # ------------------------------------------------------------------

    async def wait_command(self, command_index: int, timeout: float = 10.0) -> bool:
        """Block until *command_index* has completed.

        Satisfied by the COMPLETE push for that index, with the status stream
        as fallback (``completed_index`` high-water mark, or a blocking error
        under the stale-error ordering rule) — the engine client implements
        both sides.

        Returns True on completion, False on timeout.  Raises
        :class:`RobotError` if the command finished in error or the pipeline
        reports a blocking error.

        Category: Synchronization

        Example:
            rbt.wait_command(<index>)
        """
        core = await self._ensure_core()
        return await self._call(core.wait_command(command_index, timeout))

    async def wait_checkpoint(self, label: str, timeout: float = 30.0) -> bool:
        """Block until the checkpoint *label* is reached.

        Category: Synchronization

        Example:
            rbt.wait_checkpoint("pick_done")
        """
        return await self.wait_status(
            lambda s: s.last_checkpoint == label, timeout=timeout
        )

    async def wait_motion(
        self,
        timeout: float = 10.0,
        settle_window: float = 0.25,
        speed_threshold: float = 0.01,
        angle_threshold: float = 0.5,
        motion_start_timeout: float = 1.0,
        **kwargs: Any,
    ) -> bool:
        """Block until the robot has stopped moving (start-then-settle).

        Waits for motion to START (joint speed or angle delta above the
        thresholds, bounded by *motion_start_timeout*), then for it to stay
        below them for *settle_window* seconds.  Returns False on timeout.

        Category: Synchronization

        Example:
            rbt.wait_motion()
        """
        await self._ensure_core()
        last_angles: np.ndarray | None = None
        settle_start: float | None = None
        motion_started = False
        start = time.monotonic()
        try:
            async with asyncio.timeout(timeout):
                async for status in self.stream_status_shared():
                    max_speed = float(np.abs(status.speeds).max())
                    if last_angles is None:
                        last_angles = status.angles.copy()
                        max_delta = 0.0
                    else:
                        max_delta = float(np.abs(status.angles - last_angles).max())
                        last_angles[:] = status.angles
                    now = time.monotonic()
                    moving = (
                        max_speed >= speed_threshold or max_delta >= angle_threshold
                    )
                    if not motion_started:
                        if moving or now - start > motion_start_timeout:
                            motion_started = True
                            settle_start = None
                        if moving:
                            continue
                    if motion_started:
                        if moving:
                            settle_start = None
                        elif settle_start is None:
                            settle_start = now
                        elif now - settle_start > settle_window:
                            return True
        except TimeoutError:
            return False
        return False

    async def wait_ready(self, timeout: float = 5.0, interval: float = 0.05) -> bool:
        """Poll ping() until the runtime responds or *timeout* expires."""
        deadline = time.monotonic() + timeout
        while True:
            if await self.ping() is not None:
                return True
            if time.monotonic() >= deadline:
                return False
            await asyncio.sleep(interval)

    # ------------------------------------------------------------------
    # Motion (queued)
    # ------------------------------------------------------------------

    async def _finish_queued(self, index: int, wait: bool, timeout: float) -> int:
        if index >= 0:
            self._last_command_index = index
            if wait:
                await self.wait_command(index, timeout=timeout)
        return index

    async def home(
        self, wait: bool = False, timeout: float = 60.0, **wait_kwargs: Any
    ) -> int:
        """Move to the robot's home position (full referencing sequence when
        unhomed).  Returns the command index (>= 0), -1 when unconfirmed.

        Category: Motion

        Example:
            rbt.home()
        """
        core = await self._ensure_core()
        index = await self._call(core.home())
        if index >= 0:
            self._last_command_index = index
        if wait and index >= 0 and not await self.wait_command(index, timeout=timeout):
            raise TimeoutError(f"home() timed out after {timeout}s")
        return index

    async def move_j(
        self,
        angles: list[float] | None = None,
        *,
        pose: list[float] | None = None,
        duration: float = 0.0,
        speed: float = 0.0,
        accel: float = 1.0,
        r: float = 0.0,
        rel: bool = False,
        wait: bool = False,
        timeout: float = 10.0,
        **wait_kwargs: Any,
    ) -> int:
        """Joint-space move.  *angles* in degrees; ``pose=`` dispatches a
        joint-interpolated move to a Cartesian target instead.

        Returns the command index (>= 0), -1 when unconfirmed.

        Category: Motion

        Example:
            rbt.move_j(<joint_angles_deg>, speed=0.5)
        """
        core = await self._ensure_core()
        d, s = _timing(duration, speed)
        if pose is not None:
            if rel:
                # MOVE_J_POSE carries no rel flag on the wire, so honouring
                # this silently would send an absolute base-frame move —
                # near the origin that differs from the intended nudge by
                # the arm's whole reach.
                raise ValueError(
                    "move_j(pose=..., rel=True) is not supported: MOVE_J_POSE is "
                    "absolute. Compose the offset with the current TCP pose, or "
                    "use move_j(angles=..., rel=True) for a relative joint move."
                )
            index = await self._call(
                core.move_j_pose(_f6(pose, "pose"), d, s, float(accel), _blend(r))
            )
        else:
            if angles is None:
                raise ValueError("move_j requires angles or pose=")
            index = await self._call(
                core.move_j(
                    _f6(angles, "angles"), d, s, float(accel), _blend(r), bool(rel)
                )
            )
        return await self._finish_queued(index, wait, timeout)

    async def move_l(
        self,
        pose: list[float],
        *,
        frame: WFrame = "WRF",
        duration: float = 0.0,
        speed: float = 0.0,
        accel: float = 1.0,
        r: float = 0.0,
        rel: bool = False,
        wait: bool = False,
        timeout: float = 10.0,
        **wait_kwargs: Any,
    ) -> int:
        """Linear Cartesian move to [x, y, z, rx, ry, rz] (mm/deg).

        Returns the command index (>= 0), -1 when unconfirmed.

        Category: Motion

        Example:
            rbt.move_l(<tcp_pose_mm_deg>, speed=0.5)
        """
        core = await self._ensure_core()
        d, s = _timing(duration, speed)
        index = await self._call(
            core.move_l(
                _f6(pose, "pose"),
                _wire_frame(frame),
                d,
                s,
                float(accel),
                _blend(r),
                bool(rel),
            )
        )
        return await self._finish_queued(index, wait, timeout)

    async def move_c(
        self,
        via: list[float],
        end: list[float],
        *,
        frame: WFrame = "WRF",
        duration: float | None = None,
        speed: float | None = None,
        accel: float = 1.0,
        r: float = 0.0,
        wait: bool = False,
        timeout: float = 10.0,
        **wait_kwargs: Any,
    ) -> int:
        """Circular arc through *via* to *end*.

        Category: Motion

        Example:
            rbt.move_c(<via_pose>, <end_pose>, speed=0.5)
        """
        core = await self._ensure_core()
        d, s = _timing(duration, speed)
        index = await self._call(
            core.move_c(
                _f6(via, "via"),
                _f6(end, "end"),
                _wire_frame(frame),
                d,
                s,
                float(accel),
                _blend(r),
            )
        )
        return await self._finish_queued(index, wait, timeout)

    async def _move_multi(
        self,
        method: str,
        waypoints: list[list[float]],
        frame: WFrame,
        duration: float | None,
        speed: float | None,
        accel: float,
        wait: bool,
        timeout: float,
    ) -> int:
        core = await self._ensure_core()
        d, s = _timing(duration, speed)
        wps = [_f6(wp, "waypoint") for wp in waypoints]
        index = await self._call(
            getattr(core, method)(wps, _wire_frame(frame), d, s, float(accel))
        )
        return await self._finish_queued(index, wait, timeout)

    async def move_s(
        self,
        waypoints: list[list[float]],
        *,
        frame: WFrame = "WRF",
        duration: float | None = None,
        speed: float | None = None,
        accel: float = 1.0,
        wait: bool = False,
        timeout: float = 10.0,
        **wait_kwargs: Any,
    ) -> int:
        """Cubic spline move through waypoints (auto-chunked when large).

        Category: Motion

        Example:
            rbt.move_s(<waypoints>, speed=0.5)
        """
        return await self._move_multi(
            "move_s", waypoints, frame, duration, speed, accel, wait, timeout
        )

    async def move_p(
        self,
        waypoints: list[list[float]],
        *,
        frame: WFrame = "WRF",
        duration: float | None = None,
        speed: float | None = None,
        accel: float = 1.0,
        wait: bool = False,
        timeout: float = 10.0,
        **wait_kwargs: Any,
    ) -> int:
        """Process move with auto-blending through waypoints (auto-chunked).

        Category: Motion

        Example:
            rbt.move_p(<waypoints>, speed=0.5)
        """
        return await self._move_multi(
            "move_p", waypoints, frame, duration, speed, accel, wait, timeout
        )

    # ------------------------------------------------------------------
    # Streaming (fire-and-forget)
    # ------------------------------------------------------------------

    async def servo_j(
        self,
        angles: list[float] | None = None,
        *,
        pose: list[float] | None = None,
        speed: float = 1.0,
        accel: float = 1.0,
    ) -> int:
        """Streaming joint position target (fire-and-forget).  *angles* in
        degrees; ``pose=`` dispatches a Cartesian target via IK.

        Category: Streaming

        Example:
            rbt.servo_j(<joint_angles_deg>)
        """
        core = await self._ensure_core()
        if pose is not None:
            return await self._call(
                core.servo_j_pose(_f6(pose, "pose"), float(speed), float(accel))
            )
        if angles is None:
            raise ValueError("servo_j requires angles or pose=")
        return await self._call(
            core.servo_j(_f6(angles, "angles"), float(speed), float(accel))
        )

    async def servo_l(
        self,
        pose: list[float],
        *,
        speed: float = 1.0,
        accel: float = 1.0,
    ) -> int:
        """Streaming linear Cartesian target (fire-and-forget), mm/deg.

        Category: Streaming

        Example:
            rbt.servo_l(<tcp_pose_mm_deg>)
        """
        core = await self._ensure_core()
        return await self._call(
            core.servo_l(_f6(pose, "pose"), float(speed), float(accel))
        )

    async def jog_j(
        self,
        joint: int = -1,
        speed: float = 0.0,
        duration: float = 0.1,
        *,
        joints: list[int] | None = None,
        speeds: list[float] | None = None,
        accel: float = 1.0,
    ) -> int:
        """Joint velocity jog (fire-and-forget).  *duration* is the
        self-terminating watchdog — UIs stream fresh jogs at 20-50 Hz.
        The runtime refuses a duration above 60 s: a watchdog that long is
        not a watchdog.  Long traverses are :meth:`move_j`'s job.

        Single joint: ``jog_j(0, 0.5, 1.0)``

        The ``joints=``/``speeds=`` form drives several joints at once,
        each on its own ramp: ``jog_j(joints=[0, 3], speeds=[0.5, -0.2])``.

        Category: Jog

        Example:
            rbt.jog_j(<joint_index>, speed=0.5, duration=1.0)
        """
        speed_arr = [0.0] * NUM_JOINTS
        if joints is not None and speeds is not None:
            if len(joints) != len(speeds):
                raise ValueError(
                    f"jog_j got {len(joints)} joints and {len(speeds)} speeds"
                )
            for j, s in zip(joints, speeds):
                # An out-of-range index must not reach the array: a
                # negative one lands on a different physical joint through
                # Python's wrap-around, and the arm moves the wrong axis
                # with nothing raised.
                if not 0 <= j < NUM_JOINTS:
                    raise ValueError(
                        f"jog_j joint {j} out of range 0..{NUM_JOINTS - 1}"
                    )
                speed_arr[j] = float(s)
        elif joint >= 0:
            if joint >= NUM_JOINTS:
                raise ValueError(
                    f"jog_j joint {joint} out of range 0..{NUM_JOINTS - 1}"
                )
            speed_arr[joint] = float(speed)
        else:
            raise ValueError("jog_j requires either joint= or joints=/speeds=")
        core = await self._ensure_core()
        return await self._call(core.jog_j(speed_arr, float(duration), float(accel)))

    async def jog_l(
        self,
        frame: WFrame,
        axis: Axis | None = None,
        speed: float = 0.0,
        duration: float = 0.1,
        *,
        axes: list[Axis] | None = None,
        speeds_list: list[float] | None = None,
        accel: float = 1.0,
    ) -> int:
        """Cartesian velocity jog (fire-and-forget), duration-watchdogged.
        Same 60 s ceiling on *duration* as :meth:`jog_j`.

        Single axis: ``jog_l("WRF", "X", 0.5, 1.0)``
        Multi axis:  ``jog_l("WRF", axes=["X", "Y"], speeds_list=[0.5, -0.3])``

        Category: Jog

        Example:
            rbt.jog_l("WRF", "X", speed=0.5, duration=1.0)
        """
        velocities = [0.0] * NUM_JOINTS
        if axes is not None and speeds_list is not None:
            for a, s in zip(axes, speeds_list):
                velocities[_AXIS_INDEX[a]] = float(s)
        elif axis is not None:
            velocities[_AXIS_INDEX[axis]] = float(speed)
        else:
            raise ValueError("jog_l requires either axis= or axes=/speeds_list=")
        core = await self._ensure_core()
        return await self._call(
            core.jog_l(velocities, float(duration), _wire_frame(frame), float(accel))
        )

    async def teleport(
        self,
        angles_deg: list[float],
        tool_positions: list[float] | None = None,
    ) -> int:
        """Instantly set joint angles (simulator only; the runtime rejects it
        outside sim mode with a real error).

        Category: Control

        Example:
            rbt.teleport([0, -90, 0, 0, 0, 0])
        """
        positions = (
            [float(p) for p in tool_positions] if tool_positions is not None else None
        )
        core = await self._ensure_core()
        return await self._call(core.teleport(_f6(angles_deg, "angles_deg"), positions))

    async def reset_loop_stats(self) -> int:
        """Reset control-loop min/max metrics and overrun count (unacked).

        Category: Query

        Example:
            rbt.reset_loop_stats()
        """
        core = await self._ensure_core()
        return await self._call(core.reset_loop_stats())

    # ------------------------------------------------------------------
    # Control (SYSTEM)
    # ------------------------------------------------------------------

    async def stop(self, clear_queue: bool = True) -> int:
        """Stop all motion; with *clear_queue* (the default) also clear the
        queue.  The controller stays enabled and holding position.

        Category: Control

        Example:
            rbt.stop()
        """
        core = await self._ensure_core()
        return await self._call(core.stop(bool(clear_queue)))

    async def estop(self) -> int:
        """Protective stop: halt motion and latch the controller disabled
        until ``reset()``.

        Category: Control

        Example:
            rbt.estop()
        """
        core = await self._ensure_core()
        return await self._call(core.estop())

    async def set_gravity_comp(self, on: bool) -> int:
        """Apply (or stop applying) the gravity-compensation feedforward.

        G(q) is computed and published in every mode regardless; this is
        only about whether it is fed forward. Applying it cancels weight
        that must actually exist in the plant, which is true on hardware
        and on the torque-level sim and false on the kinematic one — which
        is why plain ``--sim`` boots with it off.

        Category: Control

        Example:
            rbt.set_gravity_comp(True)
        """
        core = await self._ensure_core()
        return await self._call(core.set_gravity_comp(bool(on)))

    async def set_payload(
        self,
        mass: float,
        com: tuple[float, float, float] = (0.0, 0.0, 0.0),
        inertia: tuple[float, float, float, float, float, float] | None = None,
    ) -> int:
        """Declare the payload the arm is carrying at the TCP.

        The gravity feedforward and the torque planning carry it from the
        next tick; the collision geometry is unchanged.  *mass* is in kg
        (0 clears the payload), *com* the centre of mass in
        end-effector-frame metres, *inertia* the rotational inertia about
        the COM ``(Ixx, Ixy, Iyy, Ixz, Iyz, Izz)`` — omitted = a point
        mass.  Refused (COMM_VALIDATION_ERROR) for negative mass or a
        non-positive-semidefinite inertia.

        Category: Control

        Example:
            rbt.set_payload(1.2, com=(0.0, 0.0, 0.05))
        """
        core = await self._ensure_core()
        return await self._call(
            core.set_payload(float(mass), [float(v) for v in com], _inertia6(inertia))
        )

    async def payload(self) -> dict | None:
        """The effective runtime payload: ``mass``, ``com``, ``inertia``
        (zeros = none).  Returns None if unreachable.

        Category: Query

        Example:
            info = rbt.payload()
        """
        core = await self._ensure_core()
        return await self._call(core.payload())

    async def pause(self) -> int:
        """Hold the executing trajectory where it is.

        Unlike :meth:`stop`, the queued samples are left intact, so
        :meth:`resume` continues the move rather than requiring the caller
        to re-issue it.

        Category: Control

        Example:
            rbt.pause()
        """
        core = await self._ensure_core()
        return await self._call(core.pause(True))

    async def resume(self) -> int:
        """Continue a trajectory held by :meth:`pause`.

        Category: Control

        Example:
            rbt.resume()
        """
        core = await self._ensure_core()
        return await self._call(core.pause(False))

    async def freedrive(self, enabled: bool) -> int:
        """Enter or leave freedrive (hand-guiding).

        par6 has no separate freedrive mode and does not need one: with the
        gravity feedforward applied, IDLE emits a torque-only G(q) hold with
        no position term, so a homed and enabled arm floats and can be
        pushed by hand. Freedrive here is therefore exactly that switch.

        Category: Control

        Example:
            rbt.freedrive(True)
        """
        return await self.set_gravity_comp(enabled)

    async def is_freedrive(self, timeout: float = 1.0) -> bool:
        """Whether the arm is floating right now.

        Reads the broadcast rather than trusting the last command: the
        condition is the runtime's own ``gravity_applied()`` — IDLE, homed,
        enabled, gravity on. Anything else has a position term holding the
        arm, so it is not back-driveable no matter what was last requested.

        Reads the LATEST received broadcast rather than waiting for the
        next one — waiting would make the answer depend on which frame
        happened to land first. Blocks only when no status has arrived at
        all, and reports False if none ever does: an arm whose state is
        unknown is not one to declare safe to grab.

        Category: Control
        """
        if self._status_generation == 0:
            await self.wait_status(lambda _s: True, timeout=timeout)
        return self._shared_status.freedrive

    async def reset(self) -> int:
        """Clear a latched protective stop, re-enabling motion.

        Category: Control

        Example:
            rbt.reset()
        """
        core = await self._ensure_core()
        return await self._call(core.reset())

    async def reset_state(self) -> int:
        """Full controller state reset (world, tool, errors) + re-sync.

        Category: Control

        Example:
            rbt.reset_state()
        """
        core = await self._ensure_core()
        return await self._call(core.reset_state())

    async def simulator(self, enabled: bool) -> int:
        """Enable or disable simulator mode (live bus-backend switch).

        Category: Control

        Example:
            rbt.simulator(True)
        """
        core = await self._ensure_core()
        return await self._call(core.simulator(bool(enabled)))

    async def connect_hardware(self, port_str: str) -> int:
        """Connect to robot hardware via serial port.

        Category: Configuration

        Example:
            rbt.connect_hardware("/dev/ttyUSB0")
        """
        if not port_str:
            raise ValueError("No port provided")
        core = await self._ensure_core()
        return await self._call(core.connect_hardware(port_str))

    async def select_profile(self, profile: str) -> int:
        """Set the motion profile (e.g. ``"TOPPRA"``).

        Category: Configuration

        Example:
            rbt.select_profile("TOPPRA")
        """
        core = await self._ensure_core()
        return await self._call(core.select_profile(profile.upper()))

    async def set_tcp_offset(self, x: float = 0, y: float = 0, z: float = 0) -> int:
        """Set TCP offset in mm, composed on top of the current tool
        transform.  (0, 0, 0) resets; changing tools resets it too.

        Category: Configuration

        Example:
            rbt.set_tcp_offset(0, 0, -190)
        """
        core = await self._ensure_core()
        return await self._call(core.set_tcp_offset(float(x), float(y), float(z)))

    async def set_shapes(self, shapes: list[Shape]) -> int:
        """Replace the program-layer keep-out / marker shapes.

        Returns 1 only after the runtime confirms the world was applied, 0
        when unconfirmed; raises :class:`RobotError` on rejection.

        Category: Configuration

        Example:
            rbt.set_shapes([Box(name="table", x=0.6, y=0.4, z=0.02,
                                pose=(0.3, 0, -0.01, 0, 0, 0))])
        """
        wire_shapes = []
        for shape in shapes:
            kind, params, pose, collision, margin, name = shape.to_wire()
            wire_shapes.append(
                {
                    "kind": kind,
                    "params": [float(p) for p in params],
                    "pose": [float(p) for p in pose],
                    "collision": bool(collision),
                    "margin": float(margin) if margin is not None else None,
                    "name": name,
                }
            )
        core = await self._ensure_core()
        return await self._call(core.set_shapes(wire_shapes))

    async def set_completion_policy(self, policy: CompletionPolicy | int) -> int:
        """Set the controller-side completion policy for queued motion
        (commanded | settled | strict)."""
        core = await self._ensure_core()
        return await self._call(
            core.set_completion_policy(int(CompletionPolicy(policy)))
        )

    async def set_recipe(self, name: str) -> int:
        """Select the telemetry recipe.  Unknown names are refused by the
        runtime (raises :class:`RobotError`)."""
        core = await self._ensure_core()
        return await self._call(core.set_recipe(name))

    async def write_io(self, index: int, value: int) -> int:
        """Set digital output by logical index (0 = first output pin).

        *index* addresses the ``[io].outputs`` list, which is also where the
        STATUS ``io`` array carries the level back — at
        ``io[digital_inputs + index]``. The level persists until the next
        write, through mode changes and e-stops alike.

        The bound checked here is the packaged config's; the runtime checks
        its own and refuses a port it does not have, so a box wired
        differently is caught either way.

        Category: I/O

        Example:
            rbt.write_io(0, 1)   # Set first output HIGH
        """
        outputs = len(io_line_names()[1])
        if not 0 <= index < outputs:
            raise ValueError(f"Output index must be in 0..{outputs - 1}")
        if value not in (0, 1):
            raise ValueError("I/O value must be 0 or 1")
        core = await self._ensure_core()
        return await self._call(core.write_io(index, value))

    # ------------------------------------------------------------------
    # Queued non-motion commands
    # ------------------------------------------------------------------

    async def select_tool(self, tool_name: str, variant_key: str = "") -> int:
        """Set the active end-effector tool on the controller.

        A runtime is built around ONE fitted gripper and refuses any other
        key, so this selects the tool the box is already wearing — read the
        available one from ``robot.tools`` rather than naming it literally.
        No par6 tool declares variants, so ``variant_key`` selects no
        geometry; it rides through to STATUS and clears the TCP offset when
        it changes.

        Category: Configuration

        Example:
            rbt.select_tool(robot.tools.default.key)
        """
        key = canonical_tool_key(tool_name)
        core = await self._ensure_core()
        index = await self._call(
            core.select_tool(key, variant_key if variant_key else None)
        )
        # Only a tool the runtime accepted is the active one: a refused
        # selection (the runtime is fitted with a different tool) would
        # otherwise leave ``client.tool`` and the tool_action key pointing
        # at hardware that is not on the arm.
        self._active_tool_key = key
        self._active_variant_key = variant_key
        return await self._finish_queued(index, False, 0.0)

    async def checkpoint(self, label: str) -> int:
        """Insert a checkpoint marker in the command queue.

        Category: Synchronization

        Example:
            rbt.checkpoint("pick_done")
        """
        core = await self._ensure_core()
        index = await self._call(core.checkpoint(label))
        return await self._finish_queued(index, False, 0.0)

    async def delay(self, seconds: float) -> int:
        """Insert a non-blocking delay in the command queue.

        Category: Synchronization

        Example:
            rbt.delay(1.0)
        """
        if seconds <= 0:
            raise ValueError("Delay must be positive")
        core = await self._ensure_core()
        index = await self._call(core.delay(float(seconds)))
        return await self._finish_queued(index, False, 0.0)

    async def tool_action(
        self,
        tool_key: str,
        action: str,
        params: list[Any] | None = None,
        *,
        wait: bool = False,
        timeout: float = 10.0,
    ) -> int:
        """Invoke a tool-specific action by key.

        Category: I/O

        Example:
            rbt.tool_action("ELECTRIC", "calibrate")
        """
        core = await self._ensure_core()
        index = await self._call(
            core.tool_action(
                canonical_tool_key(tool_key), action.strip().lower(), list(params or [])
            )
        )
        return await self._finish_queued(index, wait, timeout)

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    async def ping(self) -> PingResult | None:
        """Check connectivity.  Returns None if unreachable.

        Category: Query

        Example:
            rbt.ping()
        """
        core = await self._ensure_core()
        hardware = await self._call(core.ping())
        if hardware is None:
            return None
        return PingResult(hardware_connected=bool(hardware))

    async def angles(self) -> list[float] | None:
        """Current joint angles in degrees.

        Category: Query

        Example:
            angles = rbt.angles()
        """
        core = await self._ensure_core()
        return await self._call(core.angles())

    async def pose(self, frame: WFrame = "WRF") -> list[float] | None:
        """Current TCP pose as [x, y, z, rx, ry, rz] in mm and degrees.

        Category: Query

        Example:
            pose = rbt.pose()
        """
        core = await self._ensure_core()
        matrix = await self._call(core.pose(_wire_frame(frame)))
        if matrix is None:
            return None
        return _matrix_to_pose(matrix)

    async def io(self) -> list[int] | None:
        """Digital I/O state [in1, in2, out1, out2, estop].

        Category: Query

        Example:
            io = rbt.io()
        """
        core = await self._ensure_core()
        return await self._call(core.io())

    async def joint_speeds(self) -> list[float] | None:
        """Current joint velocities in rad/s.

        Category: Query

        Example:
            speeds = rbt.joint_speeds()
        """
        core = await self._ensure_core()
        return await self._call(core.joint_speeds())

    async def status(self) -> StatusResult | None:
        """Aggregate status snapshot.

        Category: Query

        Example:
            status = rbt.status()
        """
        core = await self._ensure_core()
        result = await self._call(core.status_query())
        if result is None:
            return None
        return StatusResult(
            pose=result["pose"],
            angles=result["angles"],
            speeds=result["speeds"],
            io=result["io"],
            tool_status=_tool_status_from_dict(result["tool_status"]),
        )

    async def queue(self) -> list[str] | None:
        """Queued command list.

        Category: Query

        Example:
            queue = rbt.queue()
        """
        result = await self.queue_state()
        return result.queue if result is not None else None

    async def queue_state(self) -> QueueResult | None:
        """Full QUEUE snapshot (names, indices, checkpoint, duration)."""
        core = await self._ensure_core()
        result = await self._call(core.queue())
        if result is None:
            return None
        return QueueResult(
            queue=result["queue"],
            executing_index=result["executing_index"],
            completed_index=result["completed_index"],
            last_checkpoint=result["last_checkpoint"],
            queued_duration=result["queued_duration"],
        )

    async def tools(self) -> ToolResult | None:
        """Current tool and available tools.

        Category: Query

        Example:
            tools = rbt.tools()
        """
        core = await self._ensure_core()
        result = await self._call(core.tools())
        if result is None:
            return None
        return ToolResult(
            tool=canonical_tool_key(result["tool"]),
            available=[canonical_tool_key(k) for k in result["available"]],
        )

    async def activity(self) -> ActivityResult | None:
        """What the robot is currently doing.

        Category: Query

        Example:
            act = rbt.activity()
        """
        core = await self._ensure_core()
        result = await self._call(core.activity())
        if result is None:
            return None
        action_state = WActionState(result["state"])
        current = result["current"]
        return ActivityResult(
            state=action_state,
            command=current,
            params=result["params"],
            error=current if action_state is WActionState.ERROR else "",
        )

    async def reachable(self) -> ReachableResult | None:
        """Remaining freedom of movement per joint/axis before hitting limits.

        Category: Query

        Example:
            en = rbt.reachable()
        """
        core = await self._ensure_core()
        result = await self._call(core.reachable())
        if result is None:
            return None
        return ReachableResult(
            joint_en=result["joint_en"],
            cart_en_wrf=result["cart_en_wrf"],
            cart_en_trf=result["cart_en_trf"],
        )

    async def error(self) -> RobotError | None:
        """Current standing error, or None if no error.

        Category: Query

        Example:
            err = rbt.error()
        """
        core = await self._ensure_core()
        result = await self._call(core.error())
        if result is None:
            return None
        return RobotError.from_wire(result)

    async def profile(self) -> str | None:
        """Current motion profile name.

        Category: Query

        Example:
            profile = rbt.profile()
        """
        core = await self._ensure_core()
        result = await self._call(core.profile())
        return result.upper() if result is not None else None

    async def tcp_speed(self) -> float | None:
        """TCP linear velocity in mm/s.

        Category: Query

        Example:
            speed = rbt.tcp_speed()
        """
        core = await self._ensure_core()
        return await self._call(core.tcp_speed())

    async def is_estop_pressed(self) -> bool:
        """Whether the e-stop is engaged.

        Category: Query

        Example:
            pressed = rbt.is_estop_pressed()
        """
        io_status = await self.io()
        if not io_status:
            return False
        # The e-stop is the LAST slot whatever the box declares, and it
        # carries the LINE, which reads low while pressed.
        return io_status[-1] == 0

    async def is_robot_stopped(self, threshold_speed: float = 0.01) -> bool:
        """Whether every joint is below *threshold_speed* (rad/s).

        Polls the live joint speeds.  Prefer ``wait_command()`` to wait
        for a specific command and ``wait_motion()`` to wait for a
        settle; this is for diagnostics and manual stop logic.

        Category: Query

        Example:
            stopped = rbt.is_robot_stopped()
        """
        speeds = await self.joint_speeds()
        if not speeds:
            return False
        return max(abs(s) for s in speeds) < threshold_speed

    async def tcp_offset(self) -> list[float]:
        """Current TCP offset in mm [x, y, z].

        Category: Configuration

        Example:
            offset = rbt.tcp_offset()
        """
        core = await self._ensure_core()
        result = await self._call(core.tcp_offset())
        if result is None:
            return [0.0, 0.0, 0.0]
        return list(result)

    async def is_simulator(self) -> bool:
        """Query whether simulator mode is active.

        Category: Query

        Example:
            active = rbt.is_simulator()
        """
        core = await self._ensure_core()
        result = await self._call(core.is_simulator())
        return bool(result) if result is not None else False

    async def loop_stats(self) -> LoopStatsResult | None:
        """Control-loop runtime metrics.

        Category: Query

        Example:
            stats = rbt.loop_stats()
        """
        core = await self._ensure_core()
        result = await self._call(core.loop_stats())
        if result is None:
            return None
        return LoopStatsResult(**result)

    async def shapes(self) -> ShapeWorld | None:
        """The collision world the runtime is currently enforcing, by layer.

        Readback truth: displays should re-query when
        ``StatusBuffer.scene_epoch`` changes.  Returns None if unreachable.

        Category: Query

        Example:
            world = rbt.shapes()
        """
        core = await self._ensure_core()
        result = await self._call(core.shapes())
        if result is None:
            return None

        def _shape(w: dict) -> Shape:
            return shape_from_wire(
                w["kind"],
                w["params"],
                w["pose"],
                w["collision"],
                w["margin"],
                w["name"],
            )

        return ShapeWorld(
            installation=tuple(_shape(w) for w in result["installation"]),
            program=tuple(_shape(w) for w in result["program"]),
        )

    async def config_info(self) -> dict | None:
        """The runtime's effective configuration.

        A dict with ``path``, ``fingerprint`` (sha256 hex over the config
        bundle's files — compare against a local mirror to detect skew),
        ``tick_dt_s``, ``motion`` (the ``[motion]`` feel constants by
        name), and ``joints`` (per-joint soft limits + EXEC
        velocity/acceleration).  Returns None if unreachable.

        Category: Query

        Example:
            info = rbt.config_info()
        """
        core = await self._ensure_core()
        return await self._call(core.config_info())

    async def config_bundle(self) -> dict | None:
        """The config files the runtime loaded, verbatim.

        A dict with ``path``, ``fingerprint`` (as ``config_info``),
        ``robot_filename``, ``robot_toml`` (the robot TOML's content) and
        ``grippers`` (list of ``{filename, content}``).  This is how a
        client previews with exactly the numbers the arm enforces —
        materialize it with :func:`par6.config.materialize_bundle` and
        hand the path to the preview engine.  Returns None if
        unreachable.

        Category: Query

        Example:
            bundle = rbt.config_bundle()
        """
        core = await self._ensure_core()
        return await self._call(core.config_bundle())

    async def _tool_status(self) -> ToolStatus | None:
        """Query tool status (internal — use ``rbt.tool.status()``)."""
        core = await self._ensure_core()
        result = await self._call(core.tool_status())
        return _tool_status_from_dict(result)


# Re-exported for callers that type against the concrete buffer.
__all__ = [
    "AsyncRobotClient",
    "LoopStatsResult",
    "QueueResult",
    "ReachableResult",
    "RobotError",
    "StatusBuffer",
    "StatusResult",
    "copy_status",
]
