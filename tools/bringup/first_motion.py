"""Stage 3 — the first real motion: one joint, one raised-cosine period.

The envelope ``q0 + A(1 − cos 2πt/T)/2`` has zero offset AND zero
velocity at t = 0 and t = T, so the stream starts and ends at rest; the
runtime's own servo limiter carries the velocity feedforward. Before
anything moves the excursion is checked against the joint's own soft
window and the peak velocity ``Aπ/T`` against its STREAM ceiling, and a
refusal prints the numbers. Nothing is bypassed: the soft-limit clamp
and the jerk limiter stay on. The arm is returned to the exact start
pose and that is verified too.

    python tools/bringup/first_motion.py --go [--joint 0] [--amplitude 5] [--period 4]
"""

from __future__ import annotations

import argparse
import threading
import time

import numpy as np
from common import (
    Ledger,
    add_connection_args,
    add_go_arg,
    canonical_pose_deg,
    collect_status,
    connect,
    fail_if_outside,
    gate,
    go_to,
    mode_velocity_deg_s,
    parse_or_exit,
    raised_cosine,
    raised_cosine_peak_velocity,
    require_ready,
    run_main,
    stream_at,
    tracking_gap_deg,
)

RATE_HZ = 50.0
TRACKING_CAP_DEG = 2.0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add_connection_args(parser)
    add_go_arg(parser)
    parser.add_argument("--joint", type=int, default=0)
    parser.add_argument("--amplitude", type=float, default=5.0, help="deg")
    parser.add_argument("--period", type=float, default=4.0, help="s")
    args = parse_or_exit(parser, argv)
    ledger = Ledger("first motion")
    pose = canonical_pose_deg()
    j = args.joint
    q0 = pose[j]
    lo, hi = min(q0, q0 + args.amplitude), max(q0, q0 + args.amplitude)
    peak_v = raised_cosine_peak_velocity(args.amplitude, args.period)
    v_lim = float(mode_velocity_deg_s("stream")[j])

    ok = fail_if_outside(ledger, j, lo, hi, "excursion inside the soft window")
    ok &= ledger.add(
        "peak velocity under the STREAM ceiling",
        peak_v < v_lim,
        f"A*pi/T = {peak_v:.2f} deg/s vs {v_lim:.2f} deg/s",
    )
    if not ok:
        return ledger.finish(args.json)

    with connect(args) as client:
        if not require_ready(client, ledger):
            return ledger.finish(args.json)
        if not gate(
            args, ledger, f"stream a {args.amplitude:+.1f} deg raised cosine on J{j}"
        ):
            return ledger.finish(args.json)
        if not go_to(client, pose, ledger):
            return ledger.finish(args.json)
        env = raised_cosine(q0, args.amplitude, args.period)

        def target(t: float) -> list[float]:
            out = list(pose)
            out[j] = env(t)
            return out

        seen: list = []
        stop = threading.Event()

        def sample() -> None:
            while not stop.is_set():
                seen.extend(collect_status(client, 0.5))

        sampler = threading.Thread(target=sample, daemon=True)
        sampler.start()
        sent = stream_at(client, RATE_HZ, args.period, target)
        client.stop(clear_queue=True)
        time.sleep(0.5)
        stop.set()
        sampler.join(timeout=2.0)

        # The broadcast clock is the runtime's: align on the first frame
        # that shows the stream moving instead of trusting two clocks.
        gap = float("inf")
        if seen:
            first = next(
                (k for k, s in enumerate(seen) if abs(float(s.angles[j]) - q0) > 0.05),
                0,
            )
            gap = tracking_gap_deg(sent, seen[first:], seen[first].mono_time_ns, j, 0.0)
        ledger.add(
            "tracking gap",
            gap < TRACKING_CAP_DEG,
            f"max |measured − commanded| {gap:.3f} deg over {len(sent)} setpoints",
            required=False,
        )
        peak_seen = max((abs(float(s.angles[j]) - q0) for s in seen), default=0.0)
        ledger.add(
            "the motion happened",
            peak_seen > 0.5 * abs(args.amplitude),
            f"peak excursion seen {peak_seen:.2f} deg of {abs(args.amplitude):.2f} commanded",
        )
        client.wait_motion(timeout=5.0)
        end = np.array(client.angles())
        err = float(np.max(np.abs(end - np.array(pose))))
        ledger.add(
            "returned to the start pose",
            err < 0.2,
            f"max error {err:.3f} deg (end {np.round(end, 2).tolist()} vs start "
            f"{np.round(pose, 2).tolist()})",
        )
    return ledger.finish(args.json)


if __name__ == "__main__":
    run_main(main)
