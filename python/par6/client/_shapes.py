"""Shape wire dicts for the preview engine (the 7-tuple, keyed)."""

from __future__ import annotations

from typing import Any


def shapes_to_wire(shapes: list[Any]) -> list[dict]:
    """``Shape.to_wire()`` for each shape, as the engine's keyed dict."""
    wire = []
    for shape in shapes:
        kind, params, pose, collision, margin, name, physics = shape.to_wire()
        wire.append(
            {
                "kind": kind,
                "params": [float(p) for p in params],
                "pose": [float(p) for p in pose],
                "collision": bool(collision),
                "margin": float(margin) if margin is not None else None,
                "name": name,
                "physics": physics,
            }
        )
    return wire
