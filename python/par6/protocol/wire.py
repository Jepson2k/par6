"""Protocol v2 wire layer — Python side.

The codec itself lives in the Rust `par6-proto` crate, reached through the
`par6._par6` extension module; nothing here encodes or decodes wire bytes.
What remains Python-side is the shared client-facing state:

- :class:`StatusBuffer` — the preallocated numpy status snapshot the
  client fills in place from the extension's STATUS frames, so consumers
  keep zero-allocation reads (and stable array identities) on the hot path.
- :func:`update_status_from_dict` — the one filler, slice-assigning a
  frame dict from the extension into a buffer.
- :class:`ProtocolError` — the wire-contract violation exception.

Constants come from the generated :mod:`par6.protocol.constants`;
``MAX_JOG_DURATION_S`` is re-exported from the extension (the Rust codec
is the source of truth for the value).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping

import numpy as np
from waldoctl import ActionState, ToolState, ToolStatus

from par6._par6 import MAX_JOG_DURATION_S

from .constants import (
    EN_SLOTS,
    IO_SLOTS,
    NUM_JOINTS,
    POSE_ELEMS,
    ControllerMode,
    HomingJointState,
    HomingPhase,
    LinkState,
)

__all__ = [
    "MAX_JOG_DURATION_S",
    "ProtocolError",
    "StatusBuffer",
    "ToolStatusWire",
    "update_status_from_dict",
]


class ProtocolError(Exception):
    """A payload violated the wire contract (bad type, arity, or range)."""


#: Reusable tool-status slot of a :class:`StatusBuffer`.
#:
#: waldoctl's own dataclass, not a wire-side copy of it: par6 had a
#: structurally identical `ToolStatusWire` whose `state` was the
#: generated `ToolState`, so `status.tool_status.state is ToolState.ACTIVE`
#: was false for a waldoctl consumer however well the values matched.
ToolStatusWire = ToolStatus


@dataclass
class StatusBuffer:
    """Preallocated buffer for zero-allocation STATUS consumption.

    Numeric arrays are numpy; :func:`update_status_from_dict` fills the
    buffer with slice assignment, so array identities are stable across
    frames and consumers may hold views.
    """

    # v2 header
    proto_version: int = 0
    controller_id: int = 0
    seq: int = 0
    mono_time_ns: int = 0
    link_ok: int = 0
    data_age_ms: int = 0
    # body
    pose: np.ndarray = field(
        default_factory=lambda: np.zeros(POSE_ELEMS, dtype=np.float64)
    )
    angles: np.ndarray = field(
        default_factory=lambda: np.zeros(NUM_JOINTS, dtype=np.float64)
    )
    speeds: np.ndarray = field(
        default_factory=lambda: np.zeros(NUM_JOINTS, dtype=np.float64)
    )
    io: np.ndarray = field(default_factory=lambda: np.zeros(IO_SLOTS, dtype=np.int32))
    action_current: str = ""
    action_state: ActionState = ActionState.IDLE
    joint_en: np.ndarray = field(
        default_factory=lambda: np.ones(EN_SLOTS, dtype=np.int32)
    )
    cart_en_wrf: np.ndarray = field(
        default_factory=lambda: np.ones(EN_SLOTS, dtype=np.int32)
    )
    cart_en_trf: np.ndarray = field(
        default_factory=lambda: np.ones(EN_SLOTS, dtype=np.int32)
    )
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
    torques: np.ndarray = field(
        default_factory=lambda: np.zeros(NUM_JOINTS, dtype=np.float64)
    )
    """Measured joint torques [Nm], kt-calibrated and filtered."""
    mode: ControllerMode = ControllerMode.BOOTING
    enabled: bool = False
    gravity_comp: bool = False
    warnings: list[tuple] = field(default_factory=list)
    """Warning-class latch entries (wire 6-tuples): self-clearing
    conditions — stale CAN data, degraded loop, failed homing. The
    ``error`` slot carries only hard latches; these are the rest."""

    @property
    def freedrive(self) -> bool:
        """Whether the arm is back-driveable right now: idle with the
        gravity feedforward applied.

        ``gravity_comp`` is the runtime's own "applied this tick" flag
        (referenced, enabled and requested), not the request, so a
        command that was accepted but cannot yet take effect reports
        False here and nobody is told an arm is safe to grab on the
        strength of a flag.
        """
        return bool(self.mode == ControllerMode.IDLE and self.gravity_comp)

    link_health: dict = field(default_factory=dict)
    """Motor-bus link health: ``state`` (a ``LinkState`` value),
    ``restarts``, ``tx_errors``, ``rx_frames``."""
    homing: dict = field(default_factory=dict)
    """Homing progress: ``active``, ``sequence_step``, and per-actuator
    ``joints`` — ``(HomingJointState, HomingPhase)`` pairs, gripper last."""
    torques_ext: np.ndarray = field(
        default_factory=lambda: np.zeros(NUM_JOINTS, dtype=np.float64)
    )
    """External joint torque estimate [Nm]: filtered measured torque
    minus the model's gravity torque."""
    # Aliases into the two enable arrays the filler mutates in place.
    cart_en: dict[str, np.ndarray] = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        self.cart_en = {"WRF": self.cart_en_wrf, "TRF": self.cart_en_trf}


def _int_array(arr: np.ndarray, values: list) -> np.ndarray:
    """In-place when the width matches (the steady state); reallocated when
    the runtime's config declares a different width than the packaged one."""
    if len(values) == len(arr):
        arr[:] = values
        return arr
    return np.asarray(values, dtype=arr.dtype)


def update_status_from_dict(buf: StatusBuffer, d: Mapping) -> None:
    """Fill *buf* from one extension STATUS frame dict, in place.

    Key names match the buffer's field names (the extension's
    ``status_dict`` contract). Enum-valued fields are re-wrapped in their
    waldoctl / generated enums so identity comparisons keep working.
    """
    buf.proto_version = d["proto_version"]
    buf.controller_id = d["controller_id"]
    buf.seq = d["seq"]
    buf.mono_time_ns = d["mono_time_ns"]
    buf.link_ok = int(d["link_ok"])
    buf.data_age_ms = d["data_age_ms"]
    buf.pose[:] = d["pose"]
    buf.angles[:] = d["angles"]
    buf.speeds[:] = d["speeds"]
    buf.io = _int_array(buf.io, d["io"])
    buf.action_current = d["action_current"]
    buf.action_state = ActionState(d["action_state"])
    buf.joint_en[:] = d["joint_en"]
    buf.cart_en_wrf[:] = d["cart_en_wrf"]
    buf.cart_en_trf[:] = d["cart_en_trf"]
    buf.executing_index = d["executing_index"]
    buf.completed_index = d["completed_index"]
    buf.last_checkpoint = d["last_checkpoint"]
    err = d["error"]
    buf.error = tuple(err) if err is not None else None
    buf.queued_segments = d["queued_segments"]
    buf.queued_duration = d["queued_duration"]
    buf.action_params = d["action_params"]
    tool = d["tool_status"]
    if tool is None:
        buf.tool_status_present = False
    else:
        buf.tool_status_present = True
        ts = buf.tool_status
        ts.key = tool["key"]
        ts.state = ToolState(tool["state"])
        ts.engaged = tool["engaged"]
        ts.part_detected = tool["part_detected"]
        ts.fault_code = tool["fault_code"]
        ts.positions = tuple(tool["positions"])
        ts.channels = tuple(tool["channels"])
        ts.variant_key = tool["variant_key"]
    buf.tcp_speed = d["tcp_speed"]
    buf.simulator_active = d["simulator_active"]
    buf.collision_active = d["collision_active"]
    buf.collision_pairs = [tuple(p) for p in d["collision_pairs"]]
    buf.scene_epoch = d["scene_epoch"]
    buf.accepted_index = d["accepted_index"]
    buf.homed = d["homed"]
    buf.torques[:] = d["torques"]
    buf.mode = ControllerMode(d["mode"])
    buf.enabled = d["enabled"]
    buf.gravity_comp = d["gravity_comp"]
    buf.warnings = [tuple(w) for w in d["warnings"]]
    link = dict(d["link_health"])
    link["state"] = LinkState(link["state"])
    buf.link_health = link
    homing = dict(d["homing"])
    homing["joints"] = [
        (HomingJointState(state), HomingPhase(phase))
        for state, phase in homing["joints"]
    ]
    buf.homing = homing
    buf.torques_ext[:] = d["torques_ext"]
