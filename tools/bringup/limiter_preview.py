"""Stage 1 — the streaming limiter, offline, against the real config.

No hardware and no daemon: the runtime's own jerk-limited servo executor
and soft-limit clamp (``par6._par6.Preview.preview_servo``) are stepped
from the MIDPOINT of each joint's own soft range toward a step target,
and the response is judged on what a bring-up cares about: overshoot,
convergence, a velocity ceiling that is used but never exceeded, and no
limit cycle. Starting at the midpoint matters — PAR6 joints 1–2 have
ranges that do not contain zero, and a zero start would make the
soft-limit backstop look like a limiter bug.

    python tools/bringup/limiter_preview.py [--step-deg 20] [--seconds 6]
"""

from __future__ import annotations

import argparse
import math

import numpy as np
from common import (
    DEG,
    Ledger,
    canonical_pose_deg,
    parse_or_exit,
    robot_config,
    run_main,
)

from par6._par6 import Preview
from par6.client.dry_run_client import _resolve_engine_paths


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", help="robot TOML (default: the runtime's search)")
    parser.add_argument(
        "--step-deg", type=float, default=20.0, help="step size per joint"
    )
    parser.add_argument("--seconds", type=float, default=6.0, help="window per joint")
    parser.add_argument("--json", action="store_true")
    args = parse_or_exit(parser, argv)

    cfg = robot_config()
    ledger = Ledger("limiter preview")
    config, assets = _resolve_engine_paths(args.config)
    preview = Preview(config=config, assets=assets)
    dt = preview.tick_dt_s()
    ticks = max(int(round(args.seconds / dt)), 2)
    joints = cfg["joints"]
    stream_v = [
        float(
            j["limits"]
            .get("stream", j["limits"])
            .get("velocity_rad_s", j["limits"]["velocity_rad_s"])
        )
        for j in joints
    ]
    rest = [v / DEG for v in canonical_pose_deg(cfg)]

    for j, joint in enumerate(joints):
        lo, hi = joint["limits"]["soft_min_rad"], joint["limits"]["soft_max_rad"]
        mid = 0.5 * (lo + hi)
        start = list(rest)
        start[j] = mid
        preview.teleport_rad(start)
        step = math.copysign(
            min(abs(args.step_deg) / DEG, 0.45 * (hi - lo)), args.step_deg
        )
        target = list(start)
        target[j] = mid + step
        r = preview.preview_servo([target], ticks)
        q = np.array(r["q"])[:, j]
        qd = np.array(r["qd"])[:, j]
        err = q - target[j]
        overshoot_pct = 100.0 * max(float(np.max(err * np.sign(step))), 0.0) / abs(step)
        final_err_deg = abs(float(err[-1])) * DEG
        peak_v = float(np.max(np.abs(qd)))
        tail = err[int(0.75 * len(err)) :]
        crossings = int(np.sum(np.diff(np.sign(tail)) != 0))
        finished = r["finished_tick"]
        settle_s = None if finished is None else finished * dt
        name = joint.get("name", f"joint{j + 1}")
        ledger.add(
            f"{name}: converges",
            finished is not None and final_err_deg < 0.01,
            f"final error {final_err_deg:.4f} deg, settled at "
            + ("never" if settle_s is None else f"{settle_s:.2f} s"),
        )
        ledger.add(
            f"{name}: overshoot",
            overshoot_pct < 1.0,
            f"{overshoot_pct:.2f} % of a {step * DEG:+.1f} deg step",
        )
        ledger.add(
            f"{name}: velocity ceiling",
            peak_v <= stream_v[j] * 1.001,
            f"peak {peak_v * DEG:.2f} deg/s vs STREAM limit {stream_v[j] * DEG:.2f} deg/s"
            + (
                " (limit reached)"
                if peak_v > 0.95 * stream_v[j]
                else " (limit not reached)"
            ),
        )
        ledger.add(
            f"{name}: no limit cycle",
            crossings <= 1,
            f"{crossings} sign changes of the error in the last quarter of the window",
        )
        others = np.array(r["q"])[:, [k for k in range(len(joints)) if k != j]]
        moved = float(
            np.max(
                np.abs(
                    others - np.array(start)[[k for k in range(len(joints)) if k != j]]
                )
            )
        )
        ledger.add(
            f"{name}: others still",
            moved == 0.0,
            f"max drift of other joints {moved:.2e} rad",
        )

        # The backstop: a target past the soft limit lands on the limit.
        beyond = list(start)
        beyond[j] = hi + 1.0
        r2 = preview.preview_servo([beyond], ticks)
        end = float(np.array(r2["q"])[-1, j])
        ledger.add(
            f"{name}: soft-limit clamp",
            abs(end - hi) < 1e-6,
            f"target {beyond[j] * DEG:.1f} deg lands at {end * DEG:.3f} deg (soft max {hi * DEG:.3f})",
        )
    return ledger.finish(args.json)


if __name__ == "__main__":
    run_main(main)
