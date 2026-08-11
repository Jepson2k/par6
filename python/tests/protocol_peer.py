"""Scripted protocol peer for client tests.

Speaks REAL par6-proto bytes through the frozen wire layer (encode/decode +
Reassembler) — the bytes are the contract, pinned cross-language by the
golden vectors, so a protocol-accurate scripted peer is the right test
double for the client: it exercises the client's actual UDP + msgpack path
without faking any behavior the wire layer doesn't already define.
"""

from __future__ import annotations

import asyncio
from collections.abc import Callable, Sequence

from par6.protocol import wire
from par6.protocol.constants import (
    COMMAND_CLASS,
    CmdType,
    CommandClass,
    MsgType,
    QueryType,
)

IDENTITY_POSE = [
    1.0, 0.0, 0.0, 250.5,
    0.0, 1.0, 0.0, -10.25,
    0.0, 0.0, 1.0, 300.0,
    0.0, 0.0, 0.0, 1.0,
]  # fmt: skip

ANGLES = [10.0, -20.25, 30.5, 0.0, 45.0, -90.0]
SPEEDS = [0.0, 0.0, 0.1, 0.0, -0.1, 0.0]
IO = [1, 1, 0, 0, 0]
TOOL_STATUS_WIRE = ["SSG48", 2, True, False, 0, [12.5], [0.0, 3.3], "fin_ray"]

# Canned RESPONSE payloads (``[query_tag, ...fields]``) matching the wire's
# per-query arity — same shapes as the golden vectors.
DEFAULT_RESPONSES: dict[CmdType, list] = {
    CmdType.PING: [int(QueryType.PING), True],
    CmdType.STATUS: [
        int(QueryType.STATUS), IDENTITY_POSE, ANGLES, SPEEDS, IO, TOOL_STATUS_WIRE,
    ],  # fmt: skip
    CmdType.ANGLES: [int(QueryType.ANGLES), ANGLES],
    CmdType.POSE: [int(QueryType.POSE), IDENTITY_POSE],
    CmdType.IO: [int(QueryType.IO), IO],
    CmdType.SPEEDS: [int(QueryType.SPEEDS), SPEEDS],
    CmdType.TOOLS: [int(QueryType.TOOLS), "SSG48", ["SSG48", "MSG"]],
    CmdType.QUEUE: [int(QueryType.QUEUE), ["MOVE_J", "DELAY"], 4, 3, "pick", 2.5],
    CmdType.ACTIVITY: [int(QueryType.ACTIVITY), "MOVE_L", 1, "DELAY", "pose=[1.0]"],
    CmdType.LOOP_STATS: [
        int(QueryType.LOOP_STATS),
        250.0, 100000, 3, 0.004, 0.0001, 0.0038, 0.0061, 0.0042, 0.0044, 249.9,
    ],  # fmt: skip
    CmdType.PROFILE: [int(QueryType.PROFILE), "TOPPRA"],
    CmdType.REACHABLE: [
        int(QueryType.REACHABLE), [1] * 12, [1] * 12, [1, 0] + [1] * 10,
    ],  # fmt: skip
    CmdType.ERROR: [int(QueryType.ERROR), None],
    CmdType.TCP_SPEED: [int(QueryType.TCP_SPEED), 12.5],
    CmdType.TCP_OFFSET: [int(QueryType.TCP_OFFSET), 0.0, 0.0, 35.5],
    CmdType.TOOL_STATUS: [int(QueryType.TOOL_STATUS), TOOL_STATUS_WIRE],
    CmdType.IS_SIMULATOR: [int(QueryType.IS_SIMULATOR), True],
    CmdType.SHAPES: [
        int(QueryType.SHAPES),
        [["box", [400.0, 600.0, 20.0], [0.0, 0.0, -10.0, 0.0, 0.0, 0.0], True, 5.0, "table"]],
        [["sphere", [50.0], [120.0, 80.0, 200.0, 0.0, 0.0, 0.0], False, None, "camera"]],
        3,
    ],  # fmt: skip
}


def error_tuple(
    command_index: int,
    code: int = 51,
    title: str = "E-stop active",
) -> list:
    """A wire error 6-tuple attributed to *command_index*."""
    return [
        command_index,
        code,
        title,
        "The emergency stop is engaged.",
        "All motion stopped; queue cleared.",
        "Release the e-stop, then send reset.",
    ]


def status_frame(
    *,
    seq: int = 1,
    link_ok: int = 1,
    data_age_ms: int = 0,
    angles: Sequence[float] | None = None,
    speeds: Sequence[float] | None = None,
    executing_index: int = -1,
    completed_index: int = -1,
    accepted_index: int = -1,
    error: list | None = None,
    last_checkpoint: str = "",
    tool_status: list | None = None,
    homed: bool = True,
) -> bytes:
    """A full 31-element v2 STATUS packet with the given overrides."""
    arr = [
        int(MsgType.STATUS),
        2,          # proto_version
        1,          # controller_id
        seq,
        1_000_000,  # mono_time_ns
        link_ok,
        data_age_ms,
        list(IDENTITY_POSE),
        list(angles) if angles is not None else [0.0] * 6,
        list(speeds) if speeds is not None else [0.0] * 6,
        [0, 0, 0, 0, 1],
        "",         # action_current
        0,          # action_state
        [1] * 12,
        [1] * 12,
        [1] * 12,
        executing_index,
        completed_index,
        last_checkpoint,
        error,
        0,          # queued_segments
        0.0,        # queued_duration
        "",         # action_params
        tool_status,
        0.0,        # tcp_speed
        True,       # simulator_active
        False,      # collision_active
        [],         # collision_pairs
        0,          # scene_epoch
        accepted_index,
        homed,
    ]
    return wire.encode_wire(arr)


Handler = Callable[[CmdType, int, list], "bytes | list[bytes] | None"]


class ScriptedRuntime(asyncio.DatagramProtocol):
    """Protocol-accurate scripted runtime peer.

    Default behavior mirrors the runtime's ack taxonomy: SYSTEM → OK, QUERY
    → canned RESPONSE, QUEUED → OK+index with an idempotency-key dedup
    window, FIRE_AND_FORGET → silence.  Per-command ``handlers`` override
    the reply; ``drop_replies[cmd] = n`` swallows the next *n* replies for
    that command (the request is still processed, so dedup still runs —
    exactly the lost-ack case the idempotency keys exist for).
    """

    def __init__(self) -> None:
        self.transport: asyncio.DatagramTransport | None = None
        self.received: list[tuple[CmdType, int, list]] = []
        self.client_addr: tuple[str, int] | None = None
        self.handlers: dict[CmdType, Handler] = {}
        self.drop_replies: dict[CmdType, int] = {}
        self.dedup: dict[int, int] = {}
        self.next_index = 0
        self.chunks_seen = 0
        self._reassembler = wire.Reassembler(timeout_s=5.0)
        self._waiters: list[tuple[Callable[[], bool], asyncio.Future]] = []

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        self.transport = transport  # type: ignore[assignment]

    @property
    def address(self) -> tuple[str, int]:
        assert self.transport is not None
        return self.transport.get_extra_info("sockname")

    def of(self, cmd: CmdType) -> list[tuple[CmdType, int, list]]:
        return [r for r in self.received if r[0] is cmd]

    async def wait_until(self, pred: Callable[[], bool], timeout: float = 2.0) -> None:
        """Await a condition over the peer's received-command log."""
        if pred():
            return
        fut: asyncio.Future = asyncio.get_running_loop().create_future()
        self._waiters.append((pred, fut))
        await asyncio.wait_for(fut, timeout)

    def send(self, payload: bytes, addr: tuple[str, int] | None = None) -> None:
        assert self.transport is not None
        target = addr or self.client_addr
        assert target is not None, "no client address known yet"
        self.transport.sendto(payload, target)

    def complete(
        self, index: int, ok: bool = True, detail: list | None = None
    ) -> None:
        """Push a COMPLETE for *index* to the client's command endpoint."""
        msg: list = [int(MsgType.COMPLETE), 0, index, ok]
        if detail is not None:
            msg.append(detail)
        self.send(wire.encode_wire(msg))

    def send_status(self, status_addr: tuple[str, int], **overrides) -> None:
        self.send(status_frame(**overrides), status_addr)

    # -- datagram handling ------------------------------------------------

    def datagram_received(self, data: bytes, addr: tuple[str, int]) -> None:
        self.client_addr = addr
        try:
            chunk = wire.decode_chunk(data)
        except wire.ProtocolError:
            self._handle_command(data, addr)
            return
        self.chunks_seen += 1
        payload = self._reassembler.push(chunk, now=0.0)
        if payload is not None:
            self._handle_command(payload, addr)

    def _handle_command(self, data: bytes, addr: tuple[str, int]) -> None:
        cmd, req_id, params = wire.decode_command(data)
        self.received.append((cmd, req_id, params))
        try:
            handler = self.handlers.get(cmd)
            if handler is not None:
                replies = handler(cmd, req_id, params)
            else:
                replies = self._default_reply(cmd, req_id, params)
            if self.drop_replies.get(cmd, 0) > 0:
                self.drop_replies[cmd] -= 1
                replies = None
            if replies:
                if isinstance(replies, (bytes, bytearray)):
                    replies = [bytes(replies)]
                assert self.transport is not None
                for reply in replies:
                    self.transport.sendto(reply, addr)
        finally:
            for pred, fut in self._waiters:
                if not fut.done() and pred():
                    fut.set_result(None)
            self._waiters = [w for w in self._waiters if not w[1].done()]

    def _default_reply(
        self, cmd: CmdType, req_id: int, params: list
    ) -> bytes | None:
        klass = COMMAND_CLASS[cmd]
        if klass is CommandClass.FIRE_AND_FORGET:
            return None
        if klass is CommandClass.SYSTEM:
            return wire.encode_wire([int(MsgType.OK), req_id])
        if klass is CommandClass.QUEUED:
            key = params[0]
            if key not in self.dedup:
                self.dedup[key] = self.next_index
                self.next_index += 1
            return wire.encode_wire([int(MsgType.OK), req_id, self.dedup[key]])
        return wire.encode_wire(
            [int(MsgType.RESPONSE), req_id, DEFAULT_RESPONSES[cmd]]
        )


async def start_peer() -> tuple[ScriptedRuntime, asyncio.DatagramTransport]:
    loop = asyncio.get_running_loop()
    transport, protocol = await loop.create_datagram_endpoint(
        ScriptedRuntime, local_addr=("127.0.0.1", 0)
    )
    return protocol, transport
