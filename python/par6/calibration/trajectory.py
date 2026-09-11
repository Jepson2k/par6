"""Bounded calibration stimuli; all coordinates and limits are SI."""

from __future__ import annotations

import numpy as np


def move(start, end, limits, dt=0.02):
    """Rest-to-rest quintic with analytically bounded speed, acceleration, jerk."""
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    limits = np.asarray(limits, dtype=float)
    if (
        start.shape != (6,)
        or end.shape != (6,)
        or limits.shape != (6, 3)
        or not all(np.isfinite(x).all() for x in [start, end, limits])
        or np.any(limits <= 0)
        or not 0 < dt <= 0.02
    ):
        raise ValueError("Need finite six-joint positions and positive v/a/j limits")
    d = np.abs(end - start)
    duration = max(
        dt,
        float(np.max(1.875 * d / limits[:, 0])),
        float(np.max(np.sqrt((10 / np.sqrt(3)) * d / limits[:, 1]))),
        float(np.max(np.cbrt(60 * d / limits[:, 2]))),
    )
    count = int(np.ceil(duration / dt))
    duration = count * dt
    t = np.arange(count + 1) * dt
    u = t / duration
    h = 10 * u**3 - 15 * u**4 + 6 * u**5
    return t, start + h[:, None] * (end - start)


def sweeps(centers, limits, *, amplitude=0.12, joints=range(6)):
    """Opposite-direction single-joint paths through each centre.

    Groups are whole centres, so a held-out split (`group % 3 == 2`) never
    shares a pose with the fit.
    """
    if len(centers) < 3:
        raise ValueError("Need at least three distinct arm configurations")
    for group, center in enumerate(centers):
        for j in joints:
            for direction in (-1, 1):
                a = np.array(center, dtype=float)
                b = a.copy()
                a[j] -= direction * amplitude
                b[j] += direction * amplitude
                t, q = move(a, b, limits)
                yield {
                    "group": group,
                    "joint": j,
                    "direction": direction,
                    "start": a.tolist(),
                    "end": b.tolist(),
                    "times": t,
                    "positions": q,
                    "held_out": group % 3 == 2,
                }
