"""Protocol v2 wire layer — Python side.

Mirrors the Rust `par6-proto` codec (the source of truth). Constants come
from the generated :mod:`par6.protocol.constants`; this module adds:

- :class:`StatusBuffer` + :func:`decode_status_bin_into` — zero-allocation
  decode of the broadcast STATUS packet into preallocated numpy arrays
  (length-guarded tail, swallow-and-``False`` on malformed).
- :func:`encode_command` / :func:`decode_command` — command envelopes with
  the same decode-time validation rules as the Rust codec (single ``nil``
  unspecified convention, finite floats, fraction ranges, exact arity).
- :func:`decode_reply` — OK / ERROR / RESPONSE / COMPLETE.
- chunk envelope helpers + :class:`Reassembler`.

Validation here matches the Rust codec rule-for-rule; the golden vectors
under ``tests/golden/protocol`` are the conformance suite for both.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Any, Callable, Sequence

import msgspec
import numpy as np
import ormsgpack
from waldoctl import ActionState, ToolState, ToolStatus

from .constants import (
    EN_SLOTS,
    IO_SLOTS,
    NUM_JOINTS,
    POSE_ELEMS,
    STATUS_LEN,
    CmdType,
    CompletionPolicy,
    Frame,
    MsgType,
    QueryType,
)

logger = logging.getLogger(__name__)

_decoder = msgspec.msgpack.Decoder()


class ProtocolError(Exception):
    """A payload violated the wire contract (bad type, arity, or range)."""


# =============================================================================
# Generic encode
# =============================================================================


def encode_wire(obj: object) -> bytes:
    """Encode any positional wire array to msgpack bytes (minimal encodings,
    floats always float64 — byte-identical to the Rust writer)."""
    return ormsgpack.packb(obj)


def encode_command(cmd: CmdType, req_id: int, params: Sequence[object] = ()) -> bytes:
    """Encode ``[cmd_tag, req_id, *params]`` after validating the params.

    QUEUED commands take their idempotency key as the first param.
    """
    params = list(params)
    _validate_command(CmdType(cmd), params)
    if not 0 <= req_id <= 0xFFFF_FFFF:
        raise ProtocolError("req_id must fit u32")
    return ormsgpack.packb([int(cmd), req_id, *params])


# =============================================================================
# Command validation (mirrors the Rust codec rule-for-rule)
# =============================================================================


def _req(ok: bool, why: str) -> None:
    if not ok:
        raise ProtocolError(why)


def _f(v: object, name: str) -> float:
    """Check `v` is a finite f64 and hand it back as one.

    The validators return their narrowed argument rather than only
    asserting it, so a caller that needs to compare or iterate the value
    afterwards works on a typed one — `type(v) is float`, not
    `isinstance`, because the Rust codec this mirrors accepts no float
    subclasses either.
    """
    if type(v) is not float or not math.isfinite(v):
        raise ProtocolError(f"{name}: must be a finite float")
    return v


def _opt_f(v: object, name: str) -> None:
    if v is not None:
        _f(v, name)


def _frac(v: object, name: str) -> None:
    _req(0.0 < _f(v, name) <= 1.0, f"{name}: must be in (0, 1]")


def _opt_frac(v: object, name: str) -> None:
    if v is not None:
        _frac(v, name)


#: Bounds mirrored from the Rust codec (`par6-proto::command`). They are
#: not generated, so they are stated here and pinned by
#: `test_protocol_golden`; the Rust side exists because
#: `Duration::from_secs_f64` and `Instant + Duration` panic near f64's
#: range, and a jog `duration` IS the watchdog that stops the jog.
MAX_DURATION_S = 3600.0
MAX_JOG_DURATION_S = 60.0
MAX_WAYPOINTS = 10_000
MAX_SHAPES = 256
#: Elements in the STATUS error tuple:
#: `[command_index, code, title, cause, effect, remedy]`.
_ERROR_ELEMS = 6
#: Reassembly bounds mirrored from `par6-proto::chunk`.
MAX_TRANSFER_BYTES = 4 * 1024 * 1024
MAX_TRANSFERS_IN_FLIGHT = 8


def _dur(v: object, name: str, hi: float = MAX_DURATION_S) -> None:
    secs = _f(v, name)
    _req(secs > 0.0, f"{name}: must be > 0")
    _req(secs <= hi, f"{name}: must be <= {hi}")


def _uint(v: object, name: str, hi: int = 0xFFFF_FFFF_FFFF_FFFF) -> None:
    _req(type(v) is int and 0 <= v <= hi, f"{name}: must be an unsigned int <= {hi}")


def _bool(v: object, name: str) -> None:
    _req(type(v) is bool, f"{name}: must be a bool")


def _str(v: object, name: str, lo: int, hi: int) -> None:
    _req(
        isinstance(v, str) and lo <= len(v.encode()) <= hi,
        f"{name}: must be a str of {lo}..={hi} bytes",
    )


def _vec6(v: object, name: str) -> list[float]:
    if not isinstance(v, list) or len(v) != NUM_JOINTS:
        raise ProtocolError(f"{name}: must be a {NUM_JOINTS}-element float array")
    return [_f(x, name) for x in v]


def _signed_frac6(v: object, name: str) -> None:
    for x in _vec6(v, name):
        _req(-1.0 <= x <= 1.0, f"{name}: must be in [-1, 1]")


def _frame(v: object, name: str) -> None:
    _req(type(v) is int and v in tuple(Frame), f"{name}: must be a Frame value")


def _opt_frame(v: object, name: str) -> None:
    if v is not None:
        _frame(v, name)


def _timing(dur: object, speed: object, name: str) -> None:
    if dur is None and speed is None:
        raise ProtocolError(f"{name}: requires one of duration or speed")
    if dur is not None and speed is not None:
        raise ProtocolError(f"{name}: duration and speed are mutually exclusive")
    if dur is not None:
        _dur(dur, f"{name}.duration")
    else:
        _frac(speed, f"{name}.speed")


def _blend(v: object, name: str) -> None:
    if v is not None:
        _req(_f(v, name) >= 0.0, f"{name}: must be >= 0")


def _waypoints(v: object, name: str) -> None:
    if not isinstance(v, list) or len(v) < 2:
        raise ProtocolError(f"{name}: requires at least 2 waypoints")
    _req(len(v) <= MAX_WAYPOINTS, f"{name}: at most {MAX_WAYPOINTS} waypoints")
    for wp in v:
        _vec6(wp, name)


def _shape(v: object) -> None:
    if not isinstance(v, list) or len(v) != 6:
        raise ProtocolError("shape: must have 6 elements")
    kind, params, pose, collision, margin, name = v
    _str(kind, "shape.kind", 1, 32)
    _req(isinstance(params, list), "shape.params: must be a float array")
    for x in params:
        _f(x, "shape.params")
    _req(isinstance(pose, list), "shape.pose: must be a float array")
    for x in pose:
        _f(x, "shape.pose")
    _bool(collision, "shape.collision")
    if margin is not None:
        _req(_f(margin, "shape.margin") >= 0.0, "shape.margin: must be >= 0")
    _str(name, "shape.name", 0, 128)


def _tool_params(v: object, name: str) -> None:
    if not isinstance(v, list) or len(v) > 16:
        raise ProtocolError(f"{name}: at most 16 values")
    for x in v:
        if type(x) is float:
            _f(x, name)
        elif type(x) in (int, bool) or isinstance(x, str):
            pass
        else:
            raise ProtocolError(f"{name}: parameters must be float, int, bool or str")


def _v_none(p: list) -> None:
    pass


def _v_stop(p: list) -> None:
    _bool(p[0], "stop.clear_queue")


def _v_write_io(p: list) -> None:
    _uint(p[0], "write_io.port", 7)
    _uint(p[1], "write_io.value", 1)


def _v_simulator(p: list) -> None:
    _bool(p[0], "simulator.on")


def _v_set_gravity_comp(p: list) -> None:
    _bool(p[0], "set_gravity_comp.on")


def _v_select_profile(p: list) -> None:
    _str(p[0], "select_profile.profile", 1, 32)


def _v_connect_hardware(p: list) -> None:
    _str(p[0], "connect_hardware.port", 1, 256)


def _v_set_tcp_offset(p: list) -> None:
    _f(p[0], "set_tcp_offset.x")
    _f(p[1], "set_tcp_offset.y")
    _f(p[2], "set_tcp_offset.z")


def _v_set_shapes(p: list) -> None:
    _req(isinstance(p[0], list), "set_shapes.shapes: must be an array")
    _req(len(p[0]) <= MAX_SHAPES, f"set_shapes.shapes: at most {MAX_SHAPES} shapes")
    for s in p[0]:
        _shape(s)


def _v_set_completion_policy(p: list) -> None:
    _req(
        type(p[0]) is int and p[0] in tuple(CompletionPolicy),
        "set_completion_policy.policy: must be a CompletionPolicy value",
    )


def _v_set_recipe(p: list) -> None:
    _str(p[0], "set_recipe.name", 1, 64)


def _v_pose_query(p: list) -> None:
    _opt_frame(p[0], "pose.frame")


def _v_servo(prefix: str) -> Callable[[list], None]:
    def v(p: list) -> None:
        _vec6(p[0], f"{prefix}.target")
        _opt_frac(p[1], f"{prefix}.speed")
        _opt_frac(p[2], f"{prefix}.accel")

    return v


def _v_jog_j(p: list) -> None:
    _signed_frac6(p[0], "jog_j.speeds")
    _dur(p[1], "jog_j.duration", MAX_JOG_DURATION_S)
    _opt_frac(p[2], "jog_j.accel")


def _v_jog_l(p: list) -> None:
    _signed_frac6(p[0], "jog_l.velocities")
    _dur(p[1], "jog_l.duration", MAX_JOG_DURATION_S)
    _frame(p[2], "jog_l.frame")
    _opt_frac(p[3], "jog_l.accel")


def _v_teleport(p: list) -> None:
    _vec6(p[0], "teleport.angles")
    if p[1] is not None:
        _req(
            isinstance(p[1], list) and len(p[1]) <= 16,
            "teleport.tool_positions: at most 16 values",
        )
        for x in p[1]:
            _f(x, "teleport.tool_positions")


def _v_home(p: list) -> None:
    _uint(p[0], "home.key")


def _v_move_j(p: list) -> None:
    _uint(p[0], "move_j.key")
    _vec6(p[1], "move_j.angles")
    _timing(p[2], p[3], "move_j")
    _opt_frac(p[4], "move_j.accel")
    _blend(p[5], "move_j.r")
    _bool(p[6], "move_j.rel")


def _v_move_j_pose(p: list) -> None:
    _uint(p[0], "move_j_pose.key")
    _vec6(p[1], "move_j_pose.pose")
    _timing(p[2], p[3], "move_j_pose")
    _opt_frac(p[4], "move_j_pose.accel")
    _blend(p[5], "move_j_pose.r")


def _v_move_l(p: list) -> None:
    _uint(p[0], "move_l.key")
    _vec6(p[1], "move_l.pose")
    _frame(p[2], "move_l.frame")
    _timing(p[3], p[4], "move_l")
    _opt_frac(p[5], "move_l.accel")
    _blend(p[6], "move_l.r")
    _bool(p[7], "move_l.rel")


def _v_move_c(p: list) -> None:
    _uint(p[0], "move_c.key")
    _vec6(p[1], "move_c.via")
    _vec6(p[2], "move_c.end")
    _frame(p[3], "move_c.frame")
    _timing(p[4], p[5], "move_c")
    _opt_frac(p[6], "move_c.accel")
    _blend(p[7], "move_c.r")


def _v_move_multi(prefix: str) -> Callable[[list], None]:
    def v(p: list) -> None:
        _uint(p[0], f"{prefix}.key")
        _waypoints(p[1], f"{prefix}.waypoints")
        _frame(p[2], f"{prefix}.frame")
        _timing(p[3], p[4], prefix)
        _opt_frac(p[5], f"{prefix}.accel")

    return v


def _v_select_tool(p: list) -> None:
    _uint(p[0], "select_tool.key")
    _str(p[1], "select_tool.tool_name", 1, 64)
    if p[2] is not None:
        _str(p[2], "select_tool.variant_key", 1, 64)


def _v_delay(p: list) -> None:
    _uint(p[0], "delay.key")
    _dur(p[1], "delay.seconds")


def _v_checkpoint(p: list) -> None:
    _uint(p[0], "checkpoint.key")
    _str(p[1], "checkpoint.label", 1, 128)


def _v_tool_action(p: list) -> None:
    _uint(p[0], "tool_action.key")
    _str(p[1], "tool_action.tool_key", 1, 64)
    _str(p[2], "tool_action.action", 1, 64)
    _tool_params(p[3], "tool_action.params")


# (param_count, validator) per command; queued commands count their key.
_COMMAND_SPECS: dict[CmdType, tuple[int, Callable[[list], None]]] = {
    CmdType.RESET: (0, _v_none),
    CmdType.ESTOP: (0, _v_none),
    CmdType.SAFETY_STOP: (0, _v_none),
    CmdType.SET_GRAVITY_COMP: (1, _v_set_gravity_comp),
    CmdType.STOP: (1, _v_stop),
    CmdType.WRITE_IO: (2, _v_write_io),
    CmdType.SIMULATOR: (1, _v_simulator),
    CmdType.SELECT_PROFILE: (1, _v_select_profile),
    CmdType.RESET_STATE: (0, _v_none),
    CmdType.CONNECT_HARDWARE: (1, _v_connect_hardware),
    CmdType.SET_TCP_OFFSET: (3, _v_set_tcp_offset),
    CmdType.SET_SHAPES: (1, _v_set_shapes),
    CmdType.SET_COMPLETION_POLICY: (1, _v_set_completion_policy),
    CmdType.SET_RECIPE: (1, _v_set_recipe),
    CmdType.PING: (0, _v_none),
    CmdType.STATUS: (0, _v_none),
    CmdType.ANGLES: (0, _v_none),
    CmdType.POSE: (1, _v_pose_query),
    CmdType.IO: (0, _v_none),
    CmdType.SPEEDS: (0, _v_none),
    CmdType.TOOLS: (0, _v_none),
    CmdType.QUEUE: (0, _v_none),
    CmdType.ACTIVITY: (0, _v_none),
    CmdType.LOOP_STATS: (0, _v_none),
    CmdType.PROFILE: (0, _v_none),
    CmdType.REACHABLE: (0, _v_none),
    CmdType.ERROR: (0, _v_none),
    CmdType.TCP_SPEED: (0, _v_none),
    CmdType.TCP_OFFSET: (0, _v_none),
    CmdType.TOOL_STATUS: (0, _v_none),
    CmdType.IS_SIMULATOR: (0, _v_none),
    CmdType.SHAPES: (0, _v_none),
    CmdType.SERVO_J: (3, _v_servo("servo_j")),
    CmdType.SERVO_J_POSE: (3, _v_servo("servo_j_pose")),
    CmdType.SERVO_L: (3, _v_servo("servo_l")),
    CmdType.JOG_J: (3, _v_jog_j),
    CmdType.JOG_L: (4, _v_jog_l),
    CmdType.TELEPORT: (2, _v_teleport),
    CmdType.RESET_LOOP_STATS: (0, _v_none),
    CmdType.HOME: (1, _v_home),
    CmdType.MOVE_J: (7, _v_move_j),
    CmdType.MOVE_J_POSE: (6, _v_move_j_pose),
    CmdType.MOVE_L: (8, _v_move_l),
    CmdType.MOVE_C: (8, _v_move_c),
    CmdType.MOVE_S: (6, _v_move_multi("move_s")),
    CmdType.MOVE_P: (6, _v_move_multi("move_p")),
    CmdType.SELECT_TOOL: (3, _v_select_tool),
    CmdType.DELAY: (2, _v_delay),
    CmdType.CHECKPOINT: (2, _v_checkpoint),
    CmdType.TOOL_ACTION: (4, _v_tool_action),
}


def _validate_command(cmd: CmdType, params: list) -> None:
    count, validator = _COMMAND_SPECS[cmd]
    if len(params) != count:
        raise ProtocolError(f"{cmd.name}: expected {count} params, got {len(params)}")
    validator(params)


def decode_command(data: bytes) -> tuple[CmdType, int, list]:
    """Decode and validate ``[cmd_tag, req_id, *params]``.

    Returns ``(cmd, req_id, params)``; for QUEUED commands ``params[0]`` is
    the idempotency key. Raises :class:`ProtocolError` on any malformed or
    out-of-range payload.
    """
    try:
        msg = _decoder.decode(data)
    except msgspec.DecodeError as e:
        raise ProtocolError(f"not msgpack: {e}") from e
    if not isinstance(msg, list) or len(msg) < 2:
        raise ProtocolError("command envelope must be [tag, req_id, ...]")
    tag, req_id = msg[0], msg[1]
    if type(tag) is not int:
        raise ProtocolError("command tag must be an int")
    try:
        cmd = CmdType(tag)
    except ValueError as e:
        raise ProtocolError(f"unknown command tag {tag}") from e
    _uint(req_id, "req_id", 0xFFFF_FFFF)
    params = msg[2:]
    _validate_command(cmd, params)
    return cmd, req_id, params


# =============================================================================
# Replies
# =============================================================================

# Wire arity (including the tag) of each RESPONSE result payload.
_RESULT_ARITY: dict[QueryType, int] = {
    QueryType.PING: 2,
    QueryType.STATUS: 6,
    QueryType.ANGLES: 2,
    QueryType.POSE: 2,
    QueryType.IO: 2,
    QueryType.SPEEDS: 2,
    QueryType.TOOLS: 3,
    QueryType.QUEUE: 6,
    QueryType.ACTIVITY: 5,
    QueryType.LOOP_STATS: 11,
    QueryType.PROFILE: 2,
    QueryType.REACHABLE: 4,
    QueryType.ERROR: 2,
    QueryType.TCP_SPEED: 2,
    QueryType.TCP_OFFSET: 4,
    QueryType.TOOL_STATUS: 2,
    QueryType.IS_SIMULATOR: 2,
    QueryType.SHAPES: 4,
}


def _check_error_tuple(err: object) -> None:
    if not isinstance(err, list) or len(err) != 6:
        raise ProtocolError("error must be a 6-element array")
    index, code, *texts = err
    if type(index) is not int:
        raise ProtocolError("error.command_index must be an int")
    _uint(code, "error.code", 0xFFFF)
    for t in texts:
        if not isinstance(t, str):
            raise ProtocolError("error text fields must be str")


def decode_reply(data: bytes) -> tuple[MsgType, int, Any]:
    """Decode a server reply/push. Returns ``(msg_type, req_id, payload)``:

    - OK: payload is the queue index or ``None``
    - ERROR: payload is the 6-element error list
    - RESPONSE: payload is the ``[query_tag, ...fields]`` list
    - COMPLETE: payload is ``(index, ok, detail)``

    Raises :class:`ProtocolError` on malformed payloads.
    """
    try:
        msg = _decoder.decode(data)
    except msgspec.DecodeError as e:
        raise ProtocolError(f"not msgpack: {e}") from e
    if not isinstance(msg, list) or len(msg) < 2:
        raise ProtocolError("reply envelope must be [tag, req_id, ...]")
    tag, req_id = msg[0], msg[1]
    if type(tag) is not int:
        raise ProtocolError("reply tag must be an int")
    try:
        mt = MsgType(tag)
    except ValueError as e:
        raise ProtocolError(f"unknown reply tag {tag}") from e
    _uint(req_id, "req_id", 0xFFFF_FFFF)

    if mt is MsgType.OK:
        if len(msg) == 2:
            return mt, req_id, None
        if len(msg) == 3:
            _uint(msg[2], "ok.index")
            return mt, req_id, msg[2]
        raise ProtocolError("OK reply has at most 3 elements")
    if mt is MsgType.ERROR:
        if len(msg) != 3:
            raise ProtocolError("ERROR reply must have 3 elements")
        _check_error_tuple(msg[2])
        return mt, req_id, msg[2]
    if mt is MsgType.RESPONSE:
        if len(msg) != 3:
            raise ProtocolError("RESPONSE reply must have 3 elements")
        result = msg[2]
        if not isinstance(result, list) or not result:
            raise ProtocolError("RESPONSE payload must be [query_tag, ...]")
        try:
            qt = QueryType(result[0])
        except (ValueError, TypeError) as e:
            raise ProtocolError(f"unknown query tag {result[0]!r}") from e
        if len(result) != _RESULT_ARITY[qt]:
            raise ProtocolError(f"{qt.name} result has wrong arity {len(result)}")
        return mt, req_id, result
    if mt is MsgType.COMPLETE:
        if req_id != 0:
            raise ProtocolError("COMPLETE pushes use req_id 0")
        if len(msg) not in (4, 5):
            raise ProtocolError("COMPLETE push must have 4 or 5 elements")
        index, ok = msg[2], msg[3]
        _uint(index, "complete.index")
        _bool(ok, "complete.ok")
        detail = msg[4] if len(msg) == 5 else None
        if detail is not None:
            _check_error_tuple(detail)
        return mt, req_id, (index, ok, detail)
    raise ProtocolError(f"tag {tag} is not a direct reply")


# =============================================================================
# STATUS buffer (zero-allocation status parsing)
# =============================================================================


#: Reusable tool-status slot of a :class:`StatusBuffer`.
#:
#: waldoctl's own dataclass, not a wire-side copy of it: par6 had a
#: structurally identical `ToolStatusWire` whose `state` was the
#: generated `ToolState`, so `status.tool_status.state is ToolState.ACTIVE`
#: was false for a waldoctl consumer however well the values matched.
ToolStatusWire = ToolStatus


@dataclass
class StatusBuffer:
    """Preallocated buffer for zero-allocation STATUS parsing.

    Numeric arrays are numpy; :func:`decode_status_bin_into` fills the buffer
    with slice assignment, allocating nothing per packet on the happy path.
    """

    # v2 header
    proto_version: int = 0
    controller_id: int = 0
    seq: int = 0
    mono_time_ns: int = 0
    link_ok: int = 0
    data_age_ms: int = 0
    # body
    pose: np.ndarray = field(default_factory=lambda: np.zeros(POSE_ELEMS, dtype=np.float64))
    angles: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS, dtype=np.float64))
    speeds: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS, dtype=np.float64))
    io: np.ndarray = field(default_factory=lambda: np.zeros(IO_SLOTS, dtype=np.int32))
    action_current: str = ""
    action_state: ActionState = ActionState.IDLE
    joint_en: np.ndarray = field(default_factory=lambda: np.ones(EN_SLOTS, dtype=np.int32))
    cart_en_wrf: np.ndarray = field(default_factory=lambda: np.ones(EN_SLOTS, dtype=np.int32))
    cart_en_trf: np.ndarray = field(default_factory=lambda: np.ones(EN_SLOTS, dtype=np.int32))
    executing_index: int = -1
    completed_index: int = -1
    last_checkpoint: str = ""
    error: tuple | None = None
    queued_segments: int = 0
    queued_duration: float = 0.0
    action_params: str = ""
    tool_status: ToolStatusWire = field(default_factory=ToolStatusWire)
    tool_status_present: bool = False
    tcp_speed: float = 0.0
    simulator_active: bool = False
    collision_active: bool = False
    collision_pairs: list[tuple[str, str]] = field(default_factory=list)
    scene_epoch: int = 0
    accepted_index: int = -1
    homed: bool = False
    # Aliases into the two enable arrays the decoder mutates in place.
    cart_en: dict[str, np.ndarray] = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        self.cart_en = {"WRF": self.cart_en_wrf, "TRF": self.cart_en_trf}


def decode_status_bin_into(data: bytes, buf: StatusBuffer) -> bool:
    """Zero-allocation decode of a STATUS packet into ``buf``.

    Requires all ``STATUS_LEN`` v2 elements; longer tails (fields appended by
    newer producers) are ignored. Returns ``False`` on any malformed packet
    without touching the caller's control flow (swallow-and-False contract).
    """
    try:
        msg = _decoder.decode(data)
        if (
            not isinstance(msg, list)
            or len(msg) < STATUS_LEN
            or msg[0] != MsgType.STATUS
        ):
            return False

        buf.proto_version = msg[1]
        buf.controller_id = msg[2]
        buf.seq = msg[3]
        buf.mono_time_ns = msg[4]
        buf.link_ok = msg[5]
        buf.data_age_ms = msg[6]
        buf.pose[:] = msg[7]
        buf.angles[:] = msg[8]
        buf.speeds[:] = msg[9]
        buf.io[:] = msg[10]
        buf.action_current = msg[11]
        buf.action_state = ActionState(msg[12])
        buf.joint_en[:] = msg[13]
        buf.cart_en_wrf[:] = msg[14]
        buf.cart_en_trf[:] = msg[15]
        buf.executing_index = msg[16]
        buf.completed_index = msg[17]
        buf.last_checkpoint = msg[18]
        # Shape-checked, not just coerced: `tuple()` accepts any iterable,
        # so a string became a tuple of characters and a dict its keys.
        # The Rust decoder rejects the packet on arity, dropping the frame;
        # coercing here turned a corrupt frame into wrong robot state.
        raw_error = msg[19]
        if raw_error is None:
            buf.error = None
        elif isinstance(raw_error, (list, tuple)) and len(raw_error) == _ERROR_ELEMS:
            buf.error = tuple(raw_error)
        else:
            raise ProtocolError(
                f"STATUS error field must be nil or {_ERROR_ELEMS} elements, "
                f"got {raw_error!r}"
            )
        buf.queued_segments = msg[20]
        buf.queued_duration = msg[21]
        buf.action_params = msg[22]

        # A malformed tool_status is a corrupt frame, not "no tool": the
        # Rust decoder returns Arity and drops the packet, where degrading
        # to None would show the tool detached while the arm still holds it.
        raw_ts = msg[23]
        if raw_ts is not None and not (
            isinstance(raw_ts, (list, tuple)) and len(raw_ts) == 8
        ):
            raise ProtocolError(f"STATUS tool_status must be nil or 8 elements, got {raw_ts!r}")
        ts = buf.tool_status
        if raw_ts is not None:
            ts.key = raw_ts[0]
            ts.state = ToolState(raw_ts[1])
            ts.engaged = raw_ts[2]
            ts.part_detected = raw_ts[3]
            ts.fault_code = raw_ts[4]
            ts.positions = tuple(raw_ts[5]) if raw_ts[5] else ()
            ts.channels = tuple(raw_ts[6]) if raw_ts[6] else ()
            ts.variant_key = raw_ts[7]
            buf.tool_status_present = True
        else:
            buf.tool_status_present = False

        buf.tcp_speed = float(msg[24])
        buf.simulator_active = bool(msg[25])
        buf.collision_active = bool(msg[26])
        pairs = buf.collision_pairs
        pairs.clear()
        if msg[27]:
            for p in msg[27]:
                pairs.append((p[0], p[1]))
        buf.scene_epoch = msg[28]
        buf.accepted_index = msg[29]
        buf.homed = bool(msg[30])
        return True
    except Exception as e:  # malformed packet: report False, never raise
        logger.debug("decode_status_bin_into: %s", e)
        return False


# =============================================================================
# Chunked bulk envelope
# =============================================================================


def encode_chunk(
    req_id: int, transfer_id: int, index: int, total: int, data: bytes
) -> bytes:
    """Encode ``[CHUNK, req_id, transfer_id, i, n, bytes]``."""
    return ormsgpack.packb(
        [int(MsgType.CHUNK), req_id, transfer_id, index, total, data]
    )


def split_into_chunks(
    req_id: int, transfer_id: int, payload: bytes, chunk_size: int
) -> list[bytes]:
    """Split an inner command datagram into encoded chunk envelopes."""
    if chunk_size <= 0:
        raise ValueError("chunk_size must be > 0")
    parts = [payload[i : i + chunk_size] for i in range(0, len(payload), chunk_size)]
    if not parts:
        parts = [b""]
    return [
        encode_chunk(req_id, transfer_id, i, len(parts), p)
        for i, p in enumerate(parts)
    ]


def decode_chunk(data: bytes) -> tuple[int, int, int, int, bytes]:
    """Decode a chunk envelope to ``(req_id, transfer_id, index, total, data)``.

    Raises :class:`ProtocolError` on malformed envelopes.
    """
    try:
        msg = _decoder.decode(data)
    except msgspec.DecodeError as e:
        raise ProtocolError(f"not msgpack: {e}") from e
    if not isinstance(msg, list) or len(msg) != 6 or msg[0] != MsgType.CHUNK:
        raise ProtocolError("chunk envelope must be [CHUNK, req_id, tid, i, n, bytes]")
    _, req_id, transfer_id, index, total, payload = msg
    _uint(req_id, "chunk.req_id", 0xFFFF_FFFF)
    _uint(transfer_id, "chunk.transfer_id", 0xFFFF_FFFF)
    _uint(index, "chunk.index", 0xFFFF)
    _uint(total, "chunk.total", 0xFFFF)
    if not isinstance(payload, bytes):
        raise ProtocolError("chunk.bytes must be bin")
    if total < 1:
        raise ProtocolError("chunk.total must be >= 1")
    if index >= total:
        raise ProtocolError("chunk.index must be < total")
    return req_id, transfer_id, index, total, payload


class Reassembler:
    """Client/server-side chunk reassembler with an inactivity timeout.

    Clock-agnostic: pass a monotonic ``now`` (seconds) into :meth:`push` and
    poll :meth:`expire` so timed-out transfers can be answered with
    ``COMM_CHUNK_TIMEOUT``.
    """

    def __init__(self, timeout_s: float = 2.0) -> None:
        self._timeout = timeout_s
        # transfer_id -> (req_id, total, parts dict, last_activity)
        self._transfers: dict[int, tuple[int, int, dict[int, bytes], float]] = {}

    def push(
        self, chunk: tuple[int, int, int, int, bytes], now: float
    ) -> bytes | None:
        """Feed one decoded chunk; returns the payload when complete."""
        req_id, transfer_id, index, total, data = chunk
        # Both the bytes in a transfer and the NUMBER of transfers are the
        # sender's choice, so both are bounded here as they are in Rust.
        if transfer_id not in self._transfers and (
            len(self._transfers) >= MAX_TRANSFERS_IN_FLIGHT
        ):
            raise ProtocolError(
                f"too many chunk transfers in flight ({len(self._transfers)})"
            )
        req0, total0, parts, _ = self._transfers.setdefault(
            transfer_id, (req_id, total, {}, now)
        )
        if total0 != total or req0 != req_id:
            del self._transfers[transfer_id]
            raise ProtocolError("inconsistent chunk transfer")
        if index not in parts:
            if sum(map(len, parts.values())) + len(data) > MAX_TRANSFER_BYTES:
                del self._transfers[transfer_id]
                raise ProtocolError("chunk transfer too large")
            parts[index] = data
        self._transfers[transfer_id] = (req0, total0, parts, now)
        if len(parts) == total:
            del self._transfers[transfer_id]
            return b"".join(parts[i] for i in range(total))
        return None

    def expire(self, now: float) -> list[int]:
        """Drop and report transfers idle longer than the timeout."""
        stale = [
            tid
            for tid, (_, _, _, last) in self._transfers.items()
            if now - last >= self._timeout
        ]
        for tid in stale:
            del self._transfers[tid]
        return stale


__all__ = [
    "ProtocolError",
    "encode_wire",
    "encode_command",
    "decode_command",
    "decode_reply",
    "ToolStatusWire",
    "StatusBuffer",
    "decode_status_bin_into",
    "encode_chunk",
    "split_into_chunks",
    "decode_chunk",
    "Reassembler",
]
