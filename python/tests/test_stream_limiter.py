"""The streaming limiter against the shipped config, offline.

``servo_j`` targets go through a jerk-limited executor and a soft-limit
clamp before they reach a drive. Stepping that executor from a standstill
to a new target is where a limiter misbehaves: it overshoots the target,
never settles, hunts around it, or quietly exceeds the STREAM velocity
limit it is supposed to enforce. None of those are visible from a single
sample, and all of them reach the arm.

Every joint is stepped from the MIDPOINT of its own soft range. That
matters: PAR6's shoulder and elbow have ranges that do not contain zero,
so starting at zero would put the joint against its soft limit and make
the backstop look like a limiter fault.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from par6 import config as par6_config
from par6._par6 import Preview

DEG = 180.0 / math.pi
STEP_DEG = 20.0
WINDOW_S = 6.0


@pytest.fixture(scope="module")
def preview() -> Preview:
    """The engine over the packaged model — the same files the dry-run
    client loads, so this judges the limiter the runtime ships."""
    return Preview(
        config=str(par6_config.data_root() / "config" / "PAR6.toml"),
        assets=str(par6_config.data_root()),
        package_dir=str(par6_config.package_search_dir()),
    )


def test_the_limiter_converges_without_overshoot_or_hunting(preview: Preview) -> None:
    # The STREAM limits as the runtime resolves them, per-mode overrides
    # and hardware ceilings included — not re-derived from the TOML here.
    cfg = par6_config.config()
    stream = cfg.limits("stream")
    names = cfg.joint_names()
    dt = preview.tick_dt_s()
    ticks = int(round(WINDOW_S / dt))
    rest = [float(v) for v in cfg.park_pose_rad()]

    for j, name in enumerate(names):
        lo = stream["soft_min"][j]
        hi = stream["soft_max"][j]
        mid = 0.5 * (lo + hi)
        start = list(rest)
        start[j] = mid
        preview.teleport_rad(start)

        # Never ask for more than the range can hold, whatever STEP_DEG says.
        step = math.copysign(min(STEP_DEG / DEG, 0.45 * (hi - lo)), STEP_DEG)
        target = list(start)
        target[j] = mid + step

        result = preview.preview_servo([target], ticks)
        q = np.array(result["q"])[:, j]
        qd = np.array(result["qd"])[:, j]
        err = q - target[j]

        finished = result["finished_tick"]
        final_err_deg = abs(float(err[-1])) * DEG
        assert finished is not None and final_err_deg < 0.01, (
            f"{name}: never settled — final error {final_err_deg:.4f} deg"
        )

        overshoot_pct = 100.0 * max(float(np.max(err * np.sign(step))), 0.0) / abs(step)
        assert overshoot_pct < 1.0, (
            f"{name}: overshot by {overshoot_pct:.2f} % of a {step * DEG:+.1f} deg step"
        )

        peak_v = float(np.max(np.abs(qd)))
        limit_v = stream["velocity"][j]
        assert peak_v <= limit_v * 1.001, (
            f"{name}: peaked at {peak_v * DEG:.2f} deg/s over the STREAM limit "
            f"{limit_v * DEG:.2f} deg/s"
        )

        # Hunting shows up as the error changing sign repeatedly once the
        # move is otherwise done, which a settled limiter never does.
        tail = err[int(0.75 * len(err)) :]
        crossings = int(np.sum(np.diff(np.sign(tail)) != 0))
        assert crossings <= 1, (
            f"{name}: limit cycle — {crossings} error sign changes in the last "
            f"quarter of a {WINDOW_S:.0f} s window"
        )

        # The step is one joint's; nothing else may have been dragged along.
        others = [k for k in range(len(names)) if k != j]
        moved = float(
            np.max(np.abs(np.array(result["q"])[:, others] - np.array(start)[others]))
        )
        assert moved < 1e-9, (
            f"{name}: stepping it moved another joint by {moved * DEG:.4f} deg"
        )
