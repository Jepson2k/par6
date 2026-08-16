"""Async UDP client for the par6d runtime — protocol v2.

Implements the waldoctl ``RobotClient`` ABC on top of the frozen wire layer in
:mod:`par6.protocol`:

- req_id correlation: every in-flight request owns a future in a pending map;
  replies are matched by the echoed req_id, never by arrival order.
- Retries: queries retry with backoff under the same req_id; QUEUED commands
  retry with the same idempotency key, which the runtime's dedup window turns
  into an effectively-once enqueue (a retried enqueue re-acks the original
  index).  SYSTEM commands are one send + wait.
- Fire-and-forget commands (servo/jog/teleport) encode into a reused buffer.
- Status subscription: multicast join with the protocol's failover ladder
  (configured iface, then the primary NIC, then INADDR_ANY, then unicast),
  decoding into ONE shared :class:`StatusBuffer` guarded by a generation
  counter + ``asyncio.Event``.
- Completion: COMPLETE pushes resolve per-index waiters; ``wait_command``
  falls back to the status stream (``completed_index`` high-water plus the
  protocol's stale-error ordering rule).

Units at this API are mm and degrees (the waldoctl convention), which is
also what the v2 wire carries — the runtime converts to SI internally.
"""

from __future__ import annotations

import asyncio
import contextlib
import copy
import logging
import math
import os
import random
import socket
import struct
import time
from collections.abc import AsyncIterator, Callable, Iterable, Sequence
from dataclasses import dataclass
from typing import Any

import msgspec
import numpy as np
from waldoctl import RobotClient as _RobotClientABC
from waldoctl.shapes import Shape, ShapeWorld, shape_from_wire
from waldoctl.status import (
    ActionState as WActionState,
)
from waldoctl.status import (
    ActivityResult,
    PingResult,
    ToolResult,
)
from waldoctl.tools import ToolSpec, ToolStatus
from waldoctl.tools import ToolState as WToolState
from waldoctl.types import Axis
from waldoctl.types import Frame as WFrame

from ..config import canonical_tool_key
from ..protocol.constants import (
    NUM_JOINTS,
    CmdType,
    CompletionPolicy,
    Frame,
    MsgType,
)
from ..protocol.wire import (
    ProtocolError,
    StatusBuffer,
    _validate_command,
    decode_reply,
    decode_status_bin_into,
    encode_command,
    split_into_chunks,
)
from .errors import RobotError

logger = logging.getLogger(__name__)

_AXIS_INDEX: dict[str, int] = {"X": 0, "Y": 1, "Z": 2, "RX": 3, "RY": 4, "RZ": 5}
_FRAMES: dict[str, Frame] = {"WRF": Frame.WRF, "TRF": Frame.TRF}

_COMPLETIONS_KEPT = 1024

# How often one error code may be logged for a reply nobody awaits.
_UNCLAIMED_ERROR_PERIOD_S = 1.0


def _env_str(name: str, default: str) -> str:
    return os.environ.get(name, default)


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    return int(raw) if raw else default


def _wire_frame(frame: WFrame) -> int:
    try:
        return int(_FRAMES[frame])
    except KeyError:
        raise ValueError(f"unknown frame {frame!r} (par6 supports WRF and TRF)") from None


def _f6(values: Sequence[float], name: str) -> list[float]:
    if len(values) != NUM_JOINTS:
        raise ValueError(f"{name} requires {NUM_JOINTS} values, got {len(values)}")
    return [float(v) for v in values]


def _timing(duration: float | None, speed: float | None) -> tuple[float | None, float | None]:
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


def _wire_tool_status(raw: Sequence | None) -> ToolStatus | None:
    if raw is None:
        return None
    key, state, engaged, part_detected, fault_code, positions, channels, variant = raw
    return ToolStatus(
        key=canonical_tool_key(key),
        variant_key=variant,
        state=WToolState(state),
        engaged=engaged,
        part_detected=part_detected,
        fault_code=fault_code,
        positions=tuple(positions),
        channels=tuple(channels),
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
    """Digital I/O [in1, in2, out1, out2, estop]."""
    tool_status: ToolStatus | None
    """Tool status, if a tool is selected."""


@dataclass
class ReachableResult:
    """Per-joint / per-axis enablement flags (1 = motion allowed)."""

    joint_en: list[int]
    cart_en_wrf: list[int]
    cart_en_trf: list[int]


@dataclass
class LoopStatsResult:
    """Control-loop runtime metrics."""

    target_hz: float
    loop_count: int
    overrun_count: int
    mean_period_s: float
    std_period_s: float
    min_period_s: float
    max_period_s: float
    p95_period_s: float
    p99_period_s: float
    mean_hz: float


@dataclass
class QueueResult:
    """QUEUE query snapshot."""

    queue: list[str]
    executing_index: int
    completed_index: int
    last_checkpoint: str
    queued_duration: float


# ---------------------------------------------------------------------------
# Sockets for the status subscription (spec failover ladder)
# ---------------------------------------------------------------------------


def _primary_iface_ip() -> str:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("1.1.1.1", 80))
        return probe.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        probe.close()


def _multicast_socket(group: str, port: int, iface: str) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        with contextlib.suppress(AttributeError, OSError):
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        try:
            sock.bind(("", port))
        except OSError:
            sock.bind((iface, port))
        group_raw = socket.inet_aton(group)
        for member in (iface, _primary_iface_ip(), None):
            try:
                if member is None:
                    mreq = struct.pack("=4sl", group_raw, socket.INADDR_ANY)
                else:
                    mreq = struct.pack("=4s4s", group_raw, socket.inet_aton(member))
                sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
                break
            except OSError:
                continue
        else:
            raise OSError(f"could not join multicast group {group}")
        sock.setblocking(False)
        return sock
    except OSError:
        sock.close()
        raise


def _unicast_socket(host: str, port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        with contextlib.suppress(AttributeError, OSError):
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        try:
            sock.bind((host, port))
        except OSError:
            sock.bind(("", port))
        sock.setblocking(False)
        return sock
    except OSError:
        sock.close()
        raise


class _ReplyProtocol(asyncio.DatagramProtocol):
    """Command-endpoint protocol: forwards every datagram to the client."""

    def __init__(self, on_datagram: Callable[[bytes], None]) -> None:
        self._on_datagram = on_datagram

    def datagram_received(self, data: bytes, addr: tuple[str, int]) -> None:
        self._on_datagram(data)

    def error_received(self, exc: Exception) -> None:
        logger.debug("command endpoint error: %s", exc)


class _StatusProtocol(asyncio.DatagramProtocol):
    """Status-endpoint protocol: decodes into the client's shared buffer."""

    def __init__(self, on_datagram: Callable[[bytes], None]) -> None:
        self._on_datagram = on_datagram

    def datagram_received(self, data: bytes, addr: tuple[str, int]) -> None:
        self._on_datagram(data)

    def error_received(self, exc: Exception) -> None:
        logger.debug("status endpoint error: %s", exc)


class AsyncRobotClient(_RobotClientABC):
    """Async UDP client for the par6d runtime.

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
            status_port if status_port is not None else _env_int("PAR6_STATUS_PORT", 6002)
        )
        self._mcast_group = mcast_group or _env_str("PAR6_STATUS_MCAST_GROUP", "239.255.0.71")
        self._mcast_iface = mcast_iface or _env_str("PAR6_STATUS_MCAST_IF", "127.0.0.1")
        self._status_unicast_host = status_unicast_host or _env_str(
            "PAR6_STATUS_UNICAST_HOST", "127.0.0.1"
        )
        self.mtu = mtu if mtu is not None else _env_int("PAR6_MTU", 1400)

        self._transport: asyncio.DatagramTransport | None = None
        self._status_transport: asyncio.DatagramTransport | None = None
        self._status_sock: socket.socket | None = None
        self._ep_lock = asyncio.Lock()
        self._closed = False

        # req_id correlation: one future per in-flight request.
        self._pending: dict[int, asyncio.Future[tuple[MsgType, Any]]] = {}
        # Last time each error code was logged for a reply nobody awaits.
        self._unclaimed_errors: dict[int, float] = {}
        self._req_id = random.randrange(1, 1 << 32)
        self._transfer_id = random.randrange(0, 1 << 32)

        # Reused TX buffer for fire-and-forget encodes.
        self._tx_buf = bytearray(256)
        self._encoder = msgspec.msgpack.Encoder()

        # Shared status buffer + generation/event notification.
        self._shared_status = StatusBuffer()
        self._status_generation = 0
        self._status_event = asyncio.Event()
        self._last_status_seq: int | None = None
        self._status_seq_gaps = 0

        # COMPLETE push bookkeeping.
        self._completions: dict[int, tuple[bool, tuple | None]] = {}
        self._complete_waiters: dict[int, list[asyncio.Future[tuple[bool, tuple | None]]]] = {}

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
    def status_address(self) -> tuple[str, int] | None:
        """Local (host, port) the status listener is bound to, once created."""
        if self._status_sock is None:
            return None
        return self._status_sock.getsockname()

    @property
    def status_seq_gaps(self) -> int:
        """Total STATUS packets lost so far, detected via header ``seq`` gaps."""
        return self._status_seq_gaps

    async def _ensure_endpoint(self) -> None:
        if self._closed:
            raise RuntimeError("AsyncRobotClient is closed")
        if self._transport is not None:
            return
        async with self._ep_lock:
            if self._closed:
                raise RuntimeError("AsyncRobotClient is closed")
            if self._transport is not None:
                return
            loop = asyncio.get_running_loop()
            transport, _ = await loop.create_datagram_endpoint(
                lambda: _ReplyProtocol(self._handle_reply),
                remote_addr=(self._host, self._port),
            )
            self._transport = transport
            await self._start_status_listener()
            logger.info(
                "par6 client endpoint: %s:%s (status %s @ %s)",
                self._host,
                self._port,
                self._status_transport_kind,
                self.status_address,
            )

    async def _start_status_listener(self) -> None:
        if self._status_transport is not None:
            return
        if self._status_transport_kind == "UNICAST":
            sock = _unicast_socket(self._status_unicast_host, self._status_port)
        else:
            try:
                sock = _multicast_socket(
                    self._mcast_group, self._status_port, self._mcast_iface
                )
            except OSError:
                logger.warning(
                    "multicast status subscription failed; falling back to unicast"
                )
                sock = _unicast_socket(self._status_unicast_host, self._status_port)
        self._status_sock = sock
        loop = asyncio.get_running_loop()
        self._status_transport, _ = await loop.create_datagram_endpoint(
            lambda: _StatusProtocol(self._handle_status), sock=sock
        )

    async def close(self) -> None:
        """Release transports and wake all waiters.  Safe to call repeatedly."""
        if self._closed:
            return
        self._closed = True
        self._status_event.set()
        for fut in list(self._pending.values()):
            if not fut.done():
                fut.cancel()
        self._pending.clear()
        for waiters in list(self._complete_waiters.values()):
            for fut in waiters:
                if not fut.done():
                    fut.cancel()
        self._complete_waiters.clear()
        await asyncio.sleep(0)  # let in-flight datagram callbacks drain
        if self._status_transport is not None:
            with contextlib.suppress(OSError):
                self._status_transport.close()
            self._status_transport = None
            self._status_sock = None
        if self._transport is not None:
            with contextlib.suppress(OSError):
                self._transport.close()
            self._transport = None

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
        """
        bound: dict[str, ToolSpec] = {}
        for spec in specs:
            bound_spec = copy.copy(spec)
            bound_spec._execute = self.tool_action  # type: ignore[attr-defined]
            bound_spec._get_status = self._tool_status  # type: ignore[attr-defined]
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
    # Wire plumbing
    # ------------------------------------------------------------------

    def _handle_reply(self, data: bytes) -> None:
        if self._closed:
            return
        try:
            msg_type, req_id, payload = decode_reply(data)
        except ProtocolError as e:
            logger.debug("ignoring undecodable reply datagram: %s", e)
            return
        if msg_type is MsgType.COMPLETE:
            index, ok, detail = payload
            self._record_complete(index, ok, detail)
            return
        fut = self._pending.get(req_id)
        if fut is None:
            if msg_type is MsgType.ERROR:
                self._log_unclaimed_error(payload)
            return
        if not fut.done():
            fut.set_result((msg_type, payload))

    def _log_unclaimed_error(self, payload: Sequence) -> None:
        """An ERROR no caller is waiting on — a rejected fire-and-forget
        command, whose SUCCESS is unacked but whose REJECTION is a real
        ERROR, or a reply that arrived after
        its request timed out.  Dropping these silently is how a gated
        jog becomes a button that does nothing.

        Throttled per error code, because a UI streaming jogs at 20-50 Hz
        gets one refusal per datagram and the operator needs to read the
        reason, not scroll past it.  Returning it to the caller is a
        wider change than a log line: ``jog_j`` returns before the reply
        could exist.  The authoritative surface is the runtime's standing
        error — a refused fire-and-forget latches there (issue #23), so
        :meth:`error` and the STATUS broadcast carry the reason; this log
        line is corroboration, not the delivery path.
        """
        try:
            err = RobotError.from_wire(payload)
        except (TypeError, ValueError):
            logger.debug("ignoring malformed unclaimed ERROR payload")
            return
        now = time.monotonic()
        last = self._unclaimed_errors.get(err.code)
        if last is not None and now - last < _UNCLAIMED_ERROR_PERIOD_S:
            return
        self._unclaimed_errors[err.code] = now
        logger.warning("runtime reported an error nothing is waiting on: %s", err)

    def _record_complete(self, index: int, ok: bool, detail: tuple | None) -> None:
        self._completions[index] = (ok, detail)
        while len(self._completions) > _COMPLETIONS_KEPT:
            self._completions.pop(next(iter(self._completions)))
        for fut in self._complete_waiters.pop(index, []):
            if not fut.done():
                fut.set_result((ok, detail))

    def _handle_status(self, data: bytes) -> None:
        if self._closed:
            return
        if not decode_status_bin_into(data, self._shared_status):
            return
        # The wire carries the config spelling of the tool key; every consumer
        # (and ``waldoctl.ToolSpec``, which upper-cases what it is given)
        # indexes tools by the canonical one.
        tool_status = self._shared_status.tool_status
        tool_status.key = canonical_tool_key(tool_status.key)
        seq = self._shared_status.seq
        last = self._last_status_seq
        if last is not None and seq > last + 1:
            self._status_seq_gaps += seq - last - 1
        self._last_status_seq = seq
        self._status_generation += 1
        self._status_event.set()

    def _next_req_id(self) -> int:
        while True:
            req_id = self._req_id
            self._req_id = self._req_id % 0xFFFF_FFFF + 1  # wraps to 1, skipping 0
            if req_id not in self._pending:
                return req_id

    def _next_transfer_id(self) -> int:
        self._transfer_id = (self._transfer_id + 1) & 0xFFFF_FFFF
        return self._transfer_id

    def _datagrams(self, data: bytes, req_id: int) -> list[bytes]:
        """Auto-chunk encodings that exceed the MTU threshold."""
        if len(data) <= self.mtu:
            return [data]
        return split_into_chunks(req_id, self._next_transfer_id(), data, self.mtu - 32)

    async def _roundtrip(
        self, datagrams: list[bytes], req_id: int, attempts: int
    ) -> tuple[MsgType, Any] | None:
        await self._ensure_endpoint()
        transport = self._transport
        assert transport is not None
        loop = asyncio.get_running_loop()
        fut: asyncio.Future[tuple[MsgType, Any]] = loop.create_future()
        self._pending[req_id] = fut
        try:
            for attempt in range(attempts):
                if not fut.done():
                    for datagram in datagrams:
                        transport.sendto(datagram)
                try:
                    return await asyncio.wait_for(asyncio.shield(fut), self.timeout)
                except TimeoutError:
                    if attempt + 1 < attempts:
                        backoff = min(0.5, 0.05 * 2**attempt) + random.uniform(0, 0.05)
                        await asyncio.sleep(backoff)
                        if fut.done():  # a late reply landed during the backoff
                            return fut.result()
            return None
        finally:
            self._pending.pop(req_id, None)

    async def _query(self, cmd: CmdType, params: Sequence[object] = ()) -> list | None:
        """QUERY roundtrip with retries.  Returns the ``[query_tag, ...fields]``
        payload, or None when the runtime is unreachable.  Raises
        :class:`RobotError` on an ERROR reply."""
        req_id = self._next_req_id()
        data = encode_command(cmd, req_id, params)
        out = await self._roundtrip(self._datagrams(data, req_id), req_id, 1 + self.retries)
        if out is None:
            return None
        msg_type, payload = out
        if msg_type is MsgType.ERROR:
            raise RobotError.from_wire(payload)
        if msg_type is MsgType.RESPONSE:
            return payload
        logger.debug("query %s got unexpected %s reply", cmd.name, msg_type.name)
        return None

    async def _system(self, cmd: CmdType, params: Sequence[object] = ()) -> int:
        """SYSTEM roundtrip: one send + wait.  1 confirmed, 0 unconfirmed;
        raises :class:`RobotError` on rejection."""
        req_id = self._next_req_id()
        data = encode_command(cmd, req_id, params)
        out = await self._roundtrip(self._datagrams(data, req_id), req_id, 1)
        if out is None:
            return 0
        msg_type, payload = out
        if msg_type is MsgType.ERROR:
            raise RobotError.from_wire(payload)
        return 1

    async def _queued(self, cmd: CmdType, params: Sequence[object]) -> int:
        """QUEUED roundtrip: idempotency-keyed, retried.  Returns the queue
        index (>= 0), -1 when unconfirmed; raises :class:`RobotError` on
        rejection."""
        key = random.getrandbits(64)
        req_id = self._next_req_id()
        data = encode_command(cmd, req_id, [key, *params])
        out = await self._roundtrip(self._datagrams(data, req_id), req_id, 1 + self.retries)
        if out is None:
            return -1
        msg_type, payload = out
        if msg_type is MsgType.ERROR:
            raise RobotError.from_wire(payload)
        if msg_type is MsgType.OK and payload is not None:
            self._last_command_index = payload
            return payload
        logger.debug("queued %s ack carried no index", cmd.name)
        return -1

    async def _fire(self, cmd: CmdType, params: Sequence[object]) -> int:
        """Fire-and-forget send, encoding into the reused TX buffer."""
        await self._ensure_endpoint()
        transport = self._transport
        assert transport is not None
        params = list(params)
        _validate_command(cmd, params)
        self._encoder.encode_into([int(cmd), self._next_req_id(), *params], self._tx_buf)
        transport.sendto(self._tx_buf)
        return 1

    # ------------------------------------------------------------------
    # Status streaming
    # ------------------------------------------------------------------

    async def stream_status_shared(self) -> AsyncIterator[StatusBuffer]:
        """Async iterator over the ONE shared status buffer (zero-copy).

        The same instance is yielded every iteration and is overwritten by
        the next packet — process immediately, never store it.  Slow
        consumers skip to the latest state.  Terminates when the client is
        closed.
        """
        await self._ensure_endpoint()
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

    async def stream_status(self) -> AsyncIterator[StatusBuffer]:
        """Async iterator of status snapshots — yields copies, safe to store.

        For zero-copy hot paths use :meth:`stream_status_shared`.
        """
        async for status in self.stream_status_shared():
            yield copy_status(status)

    async def wait_status(
        self, predicate: Callable[[StatusBuffer], bool], timeout: float = 5.0
    ) -> bool:
        """Block until *predicate* is True for a status snapshot."""
        await self._ensure_endpoint()
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

    @staticmethod
    def _blocking_error(status: StatusBuffer, command_index: int) -> tuple | None:
        """The protocol's stale-error ordering rule: a standing error fails a wait
        on *command_index* only when the frame proves it postdates that
        command's acceptance (acceptance clears stale errors server-side and
        ``accepted_index`` is monotonic)."""
        err = status.error
        if err is None or err[0] > command_index:
            return None
        if status.accepted_index >= command_index:
            return err
        return None

    async def wait_command(self, command_index: int, timeout: float = 10.0) -> bool:
        """Block until *command_index* has completed.

        Satisfied by the COMPLETE push for that index, with the status stream
        as fallback (``completed_index`` high-water mark, or a blocking error
        under the stale-error ordering rule).

        Returns True on completion, False on timeout.  Raises
        :class:`RobotError` if the command finished in error or the pipeline
        reports a blocking error.

        Category: Synchronization

        Example:
            rbt.wait_command(<index>)
        """
        await self._ensure_endpoint()
        done = self._completions.get(command_index)
        if done is None:
            loop = asyncio.get_running_loop()
            fut: asyncio.Future[tuple[bool, tuple | None]] = loop.create_future()
            self._complete_waiters.setdefault(command_index, []).append(fut)
            status_task = asyncio.create_task(
                self.wait_status(
                    lambda s: s.completed_index >= command_index
                    or self._blocking_error(s, command_index) is not None,
                    timeout=timeout,
                )
            )
            try:
                await asyncio.wait(
                    {fut, status_task},
                    timeout=timeout,
                    return_when=asyncio.FIRST_COMPLETED,
                )
                if fut.done() and not fut.cancelled():
                    done = fut.result()
                elif status_task.done() and status_task.result():
                    err = self._blocking_error(self._shared_status, command_index)
                    if err is not None:
                        raise RobotError.from_wire(err)
                    return True
                else:
                    return False
            finally:
                status_task.cancel()
                waiters = self._complete_waiters.get(command_index)
                if waiters is not None:
                    with contextlib.suppress(ValueError):
                        waiters.remove(fut)
                    if not waiters:
                        self._complete_waiters.pop(command_index, None)
        ok, detail = done
        if not ok:
            if detail is not None:
                raise RobotError.from_wire(detail)
            raise RobotError(command_index, 0, "Command failed", "", "", "")
        return True

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
        await self._ensure_endpoint()
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
                    moving = max_speed >= speed_threshold or max_delta >= angle_threshold
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

    async def home(
        self, wait: bool = False, timeout: float = 60.0, **wait_kwargs: Any
    ) -> int:
        """Move to the robot's home position (full referencing sequence when
        unhomed).  Returns the command index (>= 0), -1 when unconfirmed.

        Category: Motion

        Example:
            rbt.home()
        """
        index = await self._queued(CmdType.HOME, [])
        if wait and index >= 0:
            if not await self.wait_command(index, timeout=timeout):
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
            index = await self._queued(
                CmdType.MOVE_J_POSE,
                [_f6(pose, "pose"), d, s, float(accel), _blend(r)],
            )
        else:
            if angles is None:
                raise ValueError("move_j requires angles or pose=")
            index = await self._queued(
                CmdType.MOVE_J,
                [_f6(angles, "angles"), d, s, float(accel), _blend(r), bool(rel)],
            )
        if wait and index >= 0:
            await self.wait_command(index, timeout=timeout)
        return index

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
        d, s = _timing(duration, speed)
        index = await self._queued(
            CmdType.MOVE_L,
            [
                _f6(pose, "pose"),
                _wire_frame(frame),
                d,
                s,
                float(accel),
                _blend(r),
                bool(rel),
            ],
        )
        if wait and index >= 0:
            await self.wait_command(index, timeout=timeout)
        return index

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
        d, s = _timing(duration, speed)
        index = await self._queued(
            CmdType.MOVE_C,
            [
                _f6(via, "via"),
                _f6(end, "end"),
                _wire_frame(frame),
                d,
                s,
                float(accel),
                _blend(r),
            ],
        )
        if wait and index >= 0:
            await self.wait_command(index, timeout=timeout)
        return index

    async def _move_multi(
        self,
        cmd: CmdType,
        waypoints: list[list[float]],
        frame: WFrame,
        duration: float | None,
        speed: float | None,
        accel: float,
        wait: bool,
        timeout: float,
    ) -> int:
        d, s = _timing(duration, speed)
        wps = [_f6(wp, "waypoint") for wp in waypoints]
        index = await self._queued(cmd, [wps, _wire_frame(frame), d, s, float(accel)])
        if wait and index >= 0:
            await self.wait_command(index, timeout=timeout)
        return index

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
            CmdType.MOVE_S, waypoints, frame, duration, speed, accel, wait, timeout
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
            CmdType.MOVE_P, waypoints, frame, duration, speed, accel, wait, timeout
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
        if pose is not None:
            return await self._fire(
                CmdType.SERVO_J_POSE, [_f6(pose, "pose"), float(speed), float(accel)]
            )
        if angles is None:
            raise ValueError("servo_j requires angles or pose=")
        return await self._fire(
            CmdType.SERVO_J, [_f6(angles, "angles"), float(speed), float(accel)]
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
        return await self._fire(
            CmdType.SERVO_L, [_f6(pose, "pose"), float(speed), float(accel)]
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

        The ``joints=``/``speeds=`` form exists for waldoctl parity, but
        the PAR6 runtime drives ONE joint at a time: a request with more
        than one non-zero speed is refused with a validation error, which
        surfaces through :meth:`error` and the STATUS broadcast (the send
        itself still returns 1 — fire-and-forget success is unacked).

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
                    raise ValueError(f"jog_j joint {j} out of range 0..{NUM_JOINTS - 1}")
                speed_arr[j] = float(s)
        elif joint >= 0:
            if joint >= NUM_JOINTS:
                raise ValueError(
                    f"jog_j joint {joint} out of range 0..{NUM_JOINTS - 1}"
                )
            speed_arr[joint] = float(speed)
        else:
            raise ValueError("jog_j requires either joint= or joints=/speeds=")
        return await self._fire(
            CmdType.JOG_J, [speed_arr, float(duration), float(accel)]
        )

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
        return await self._fire(
            CmdType.JOG_L,
            [velocities, float(duration), _wire_frame(frame), float(accel)],
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
        return await self._fire(
            CmdType.TELEPORT, [_f6(angles_deg, "angles_deg"), positions]
        )

    async def reset_loop_stats(self) -> int:
        """Reset control-loop min/max metrics and overrun count (unacked).

        Category: Query

        Example:
            rbt.reset_loop_stats()
        """
        return await self._fire(CmdType.RESET_LOOP_STATS, [])

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
        return await self._system(CmdType.STOP, [bool(clear_queue)])

    async def estop(self) -> int:
        """Protective stop: halt motion and latch the controller disabled
        until ``reset()``.

        Category: Control

        Example:
            rbt.estop()
        """
        return await self._system(CmdType.ESTOP, [])

    async def safety_stop(self) -> int:
        """Drop every joint limp and hold there.

        The safest state the arm has. Unlike :meth:`estop`, which holds
        position under power, this removes drive authority — so a trapped
        person or a jammed joint can be freed by hand. The arm stays limp
        until a mode change takes it out.

        Category: Control

        Example:
            rbt.safety_stop()
        """
        return await self._system(CmdType.SAFETY_STOP, [])

    async def reset(self) -> int:
        """Clear a latched protective stop, re-enabling motion.

        Category: Control

        Example:
            rbt.reset()
        """
        return await self._system(CmdType.RESET, [])

    async def reset_state(self) -> int:
        """Full controller state reset (world, tool, errors) + re-sync.

        Category: Control

        Example:
            rbt.reset_state()
        """
        return await self._system(CmdType.RESET_STATE, [])

    async def simulator(self, enabled: bool) -> int:
        """Enable or disable simulator mode (live bus-backend switch).

        Category: Control

        Example:
            rbt.simulator(True)
        """
        return await self._system(CmdType.SIMULATOR, [bool(enabled)])

    async def connect_hardware(self, port_str: str) -> int:
        """Connect to robot hardware via serial port.

        Category: Configuration

        Example:
            rbt.connect_hardware("/dev/ttyUSB0")
        """
        if not port_str:
            raise ValueError("No port provided")
        return await self._system(CmdType.CONNECT_HARDWARE, [port_str])

    async def select_profile(self, profile: str) -> int:
        """Set the motion profile (e.g. ``"TOPPRA"``).

        Category: Configuration

        Example:
            rbt.select_profile("TOPPRA")
        """
        return await self._system(CmdType.SELECT_PROFILE, [profile.upper()])

    async def set_tcp_offset(self, x: float = 0, y: float = 0, z: float = 0) -> int:
        """Set TCP offset in mm, composed on top of the current tool
        transform.  (0, 0, 0) resets; changing tools resets it too.

        Category: Configuration

        Example:
            rbt.set_tcp_offset(0, 0, -190)
        """
        return await self._system(
            CmdType.SET_TCP_OFFSET, [float(x), float(y), float(z)]
        )

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
                [
                    kind,
                    [float(p) for p in params],
                    [float(p) for p in pose],
                    bool(collision),
                    float(margin) if margin is not None else None,
                    name,
                ]
            )
        return await self._system(CmdType.SET_SHAPES, [wire_shapes])

    async def set_completion_policy(self, policy: CompletionPolicy | int) -> int:
        """Set the controller-side completion policy for queued motion
        (commanded | settled | strict)."""
        return await self._system(
            CmdType.SET_COMPLETION_POLICY, [int(CompletionPolicy(policy))]
        )

    async def set_recipe(self, name: str) -> int:
        """Select the telemetry recipe.  Unknown names are refused by the
        runtime (raises :class:`RobotError`)."""
        return await self._system(CmdType.SET_RECIPE, [name])

    async def write_io(self, index: int, value: int) -> int:
        """Set digital output by logical index (0 = first output pin).

        The controller I/O layout is ``[in1, in2, out1, out2, estop]``, so
        logical output 0 maps to port 2.

        Category: I/O

        Example:
            rbt.write_io(0, 1)   # Set first output HIGH
        """
        if index not in (0, 1):
            raise ValueError("Output index must be 0 or 1")
        if value not in (0, 1):
            raise ValueError("I/O value must be 0 or 1")
        return await self._system(CmdType.WRITE_IO, [index + 2, value])

    # ------------------------------------------------------------------
    # Queued non-motion commands
    # ------------------------------------------------------------------

    async def select_tool(self, tool_name: str, variant_key: str = "") -> int:
        """Set the active end-effector tool on the controller.

        Category: Configuration

        Example:
            rbt.select_tool("PNEUMATIC")
        """
        key = canonical_tool_key(tool_name)
        index = await self._queued(
            CmdType.SELECT_TOOL, [key, variant_key if variant_key else None]
        )
        # Only a tool the runtime accepted is the active one: a refused
        # selection (the runtime is fitted with a different tool) would
        # otherwise leave ``client.tool`` and the tool_action key pointing
        # at hardware that is not on the arm.
        self._active_tool_key = key
        self._active_variant_key = variant_key
        return index

    async def checkpoint(self, label: str) -> int:
        """Insert a checkpoint marker in the command queue.

        Category: Synchronization

        Example:
            rbt.checkpoint("pick_done")
        """
        return await self._queued(CmdType.CHECKPOINT, [label])

    async def delay(self, seconds: float) -> int:
        """Insert a non-blocking delay in the command queue.

        Category: Synchronization

        Example:
            rbt.delay(1.0)
        """
        if seconds <= 0:
            raise ValueError("Delay must be positive")
        return await self._queued(CmdType.DELAY, [float(seconds)])

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
        index = await self._queued(
            CmdType.TOOL_ACTION,
            [canonical_tool_key(tool_key), action.strip().lower(), list(params or [])],
        )
        if wait and index >= 0:
            await self.wait_command(index, timeout=timeout)
        return index

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    async def ping(self) -> PingResult | None:
        """Check connectivity.  Returns None if unreachable.

        Category: Query

        Example:
            rbt.ping()
        """
        result = await self._query(CmdType.PING)
        if result is None:
            return None
        return PingResult(hardware_connected=bool(result[1]))

    async def angles(self) -> list[float] | None:
        """Current joint angles in degrees.

        Category: Query

        Example:
            angles = rbt.angles()
        """
        result = await self._query(CmdType.ANGLES)
        return result[1] if result is not None else None

    async def pose(self, frame: WFrame = "WRF") -> list[float] | None:
        """Current TCP pose as [x, y, z, rx, ry, rz] in mm and degrees.

        Category: Query

        Example:
            pose = rbt.pose()
        """
        result = await self._query(CmdType.POSE, [_wire_frame(frame)])
        if result is None:
            return None
        return _matrix_to_pose(result[1])

    async def io(self) -> list[int] | None:
        """Digital I/O state [in1, in2, out1, out2, estop].

        Category: Query

        Example:
            io = rbt.io()
        """
        result = await self._query(CmdType.IO)
        return result[1] if result is not None else None

    async def joint_speeds(self) -> list[float] | None:
        """Current joint velocities in rad/s.

        Category: Query

        Example:
            speeds = rbt.joint_speeds()
        """
        result = await self._query(CmdType.SPEEDS)
        return result[1] if result is not None else None

    async def status(self) -> StatusResult | None:
        """Aggregate status snapshot.

        Category: Query

        Example:
            status = rbt.status()
        """
        result = await self._query(CmdType.STATUS)
        if result is None:
            return None
        _, pose, angles, speeds, io, tool_status = result
        return StatusResult(
            pose=pose,
            angles=angles,
            speeds=speeds,
            io=io,
            tool_status=_wire_tool_status(tool_status),
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
        result = await self._query(CmdType.QUEUE)
        if result is None:
            return None
        _, names, executing, completed, last_checkpoint, queued_duration = result
        return QueueResult(
            queue=names,
            executing_index=executing,
            completed_index=completed,
            last_checkpoint=last_checkpoint,
            queued_duration=queued_duration,
        )

    async def tools(self) -> ToolResult | None:
        """Current tool and available tools.

        Category: Query

        Example:
            tools = rbt.tools()
        """
        result = await self._query(CmdType.TOOLS)
        if result is None:
            return None
        return ToolResult(
            tool=canonical_tool_key(result[1]),
            available=[canonical_tool_key(k) for k in result[2]],
        )

    async def activity(self) -> ActivityResult | None:
        """What the robot is currently doing.

        Category: Query

        Example:
            act = rbt.activity()
        """
        result = await self._query(CmdType.ACTIVITY)
        if result is None:
            return None
        _, current, state, _next_action, params = result
        action_state = WActionState(state)
        return ActivityResult(
            state=action_state,
            command=current,
            params=params,
            error=current if action_state is WActionState.ERROR else "",
        )

    async def reachable(self) -> ReachableResult | None:
        """Remaining freedom of movement per joint/axis before hitting limits.

        Category: Query

        Example:
            en = rbt.reachable()
        """
        result = await self._query(CmdType.REACHABLE)
        if result is None:
            return None
        return ReachableResult(
            joint_en=result[1], cart_en_wrf=result[2], cart_en_trf=result[3]
        )

    async def error(self) -> RobotError | None:
        """Current standing error, or None if no error.

        Category: Query

        Example:
            err = rbt.error()
        """
        result = await self._query(CmdType.ERROR)
        if result is None or result[1] is None:
            return None
        return RobotError.from_wire(result[1])

    async def profile(self) -> str | None:
        """Current motion profile name.

        Category: Query

        Example:
            profile = rbt.profile()
        """
        result = await self._query(CmdType.PROFILE)
        return result[1].upper() if result is not None else None

    async def tcp_speed(self) -> float | None:
        """TCP linear velocity in mm/s.

        Category: Query

        Example:
            speed = rbt.tcp_speed()
        """
        result = await self._query(CmdType.TCP_SPEED)
        return result[1] if result is not None else None

    async def is_estop_pressed(self) -> bool:
        """Whether the e-stop is engaged.

        Category: Query

        Example:
            pressed = rbt.is_estop_pressed()
        """
        io_status = await self.io()
        if io_status is None or len(io_status) < 5:
            return False
        # The e-stop slot carries the LINE, which reads low while pressed.
        return io_status[4] == 0

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
        result = await self._query(CmdType.TCP_OFFSET)
        if result is None:
            return [0.0, 0.0, 0.0]
        return [result[1], result[2], result[3]]

    async def is_simulator(self) -> bool:
        """Query whether simulator mode is active.

        Category: Query

        Example:
            active = rbt.is_simulator()
        """
        result = await self._query(CmdType.IS_SIMULATOR)
        return bool(result[1]) if result is not None else False

    async def loop_stats(self) -> LoopStatsResult | None:
        """Control-loop runtime metrics.

        Category: Query

        Example:
            stats = rbt.loop_stats()
        """
        result = await self._query(CmdType.LOOP_STATS)
        if result is None:
            return None
        return LoopStatsResult(*result[1:])

    async def shapes(self) -> ShapeWorld | None:
        """The collision world the runtime is currently enforcing, by layer.

        Readback truth: displays should re-query when
        ``StatusBuffer.scene_epoch`` changes.  Returns None if unreachable.

        Category: Query

        Example:
            world = rbt.shapes()
        """
        result = await self._query(CmdType.SHAPES)
        if result is None:
            return None
        _, installation, program, _scene_epoch = result
        return ShapeWorld(
            installation=tuple(shape_from_wire(*w) for w in installation),
            program=tuple(shape_from_wire(*w) for w in program),
        )

    async def _tool_status(self) -> ToolStatus | None:
        """Query tool status (internal — use ``rbt.tool.status()``)."""
        result = await self._query(CmdType.TOOL_STATUS)
        if result is None:
            return None
        return _wire_tool_status(result[1])


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
