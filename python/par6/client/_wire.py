"""Argument adaptation shared by the live client and the dry run: the
waldoctl call conventions (mm/deg, duration-or-speed, axis names) mapped
onto the wire's fields.  No numerics — only shapes and names."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from waldoctl.shapes import Shape
from waldoctl.tools import ToolState as WToolState
from waldoctl.tools import ToolStatus

from ..config import canonical_tool_key
from ..protocol.constants import NUM_JOINTS, Frame

AXIS_INDEX: dict[str, int] = {"X": 0, "Y": 1, "Z": 2, "RX": 3, "RY": 4, "RZ": 5}
_FRAMES: dict[str, Frame] = {"WRF": Frame.WRF, "TRF": Frame.TRF}


def wire_frame(frame: str) -> int:
    try:
        return int(_FRAMES[frame])
    except KeyError:
        raise ValueError(
            f"unknown frame {frame!r} (par6 supports WRF and TRF)"
        ) from None


def f6(values: Sequence[float], name: str) -> list[float]:
    if len(values) != NUM_JOINTS:
        raise ValueError(f"{name} requires {NUM_JOINTS} values, got {len(values)}")
    return [float(v) for v in values]


def timing(
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


def blend(r: float | None) -> float | None:
    return float(r) if r else None


def jog_j_speeds(
    joint: int,
    speed: float,
    joints: list[int] | None,
    speeds: list[float] | None,
) -> list[float]:
    """Per-joint speed fractions for ``jog_j``'s two calling forms."""
    out = [0.0] * NUM_JOINTS
    if joints is not None and speeds is not None:
        if len(joints) != len(speeds):
            raise ValueError(f"jog_j got {len(joints)} joints and {len(speeds)} speeds")
        for j, s in zip(joints, speeds):
            # An out-of-range index must not reach the array: a negative one
            # lands on a different physical joint through Python's
            # wrap-around, and the arm moves the wrong axis with nothing
            # raised.
            if not 0 <= j < NUM_JOINTS:
                raise ValueError(f"jog_j joint {j} out of range 0..{NUM_JOINTS - 1}")
            out[j] = float(s)
    elif joint >= 0:
        if joint >= NUM_JOINTS:
            raise ValueError(f"jog_j joint {joint} out of range 0..{NUM_JOINTS - 1}")
        out[joint] = float(speed)
    else:
        raise ValueError("jog_j requires either joint= or joints=/speeds=")
    return out


def jog_l_velocities(
    axis: str | None,
    speed: float,
    axes: list[str] | None,
    speeds_list: list[float] | None,
) -> list[float]:
    """Per-axis velocity fractions for ``jog_l``'s two calling forms."""
    out = [0.0] * 6
    if axes is not None and speeds_list is not None:
        for a, s in zip(axes, speeds_list):
            out[AXIS_INDEX[a]] = float(s)
    elif axis is not None:
        out[AXIS_INDEX[axis]] = float(speed)
    else:
        raise ValueError("jog_l requires either axis= or axes=/speeds_list=")
    return out


def shape_to_wire(shape: Shape) -> dict[str, Any]:
    kind, params, pose, collision, margin, name = shape.to_wire()
    return {
        "kind": kind,
        "params": [float(p) for p in params],
        "pose": [float(p) for p in pose],
        "collision": bool(collision),
        "margin": float(margin) if margin is not None else None,
        "name": name,
    }


def tool_status_from_dict(raw: dict | None) -> ToolStatus | None:
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
