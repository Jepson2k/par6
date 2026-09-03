"""Stage 4 — three joints at once, judged comparatively.

Three raised-cosine envelopes with different amplitudes and periods are
streamed together. Absolute tracking gaps are reported, but the
criterion is COMPARATIVE: each joint's gap must scale with its own
aggressiveness (peak velocity A·π/T) — synchronised behaviour, where one
loop drives the others, would inflate the gentle joints' gaps up to the
aggressive one's.

    python tools/bringup/multi_joint.py --go [--joints 0 3 5] [--amplitudes 4 3 2] [--periods 4 3 2]
"""

from __future__ import annotations

import argparse
import math
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


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add_connection_args(parser)
    add_go_arg(parser)
    parser.add_argument("--joints", type=int, nargs=3, default=[0, 3, 5])
    parser.add_argument(
        "--amplitudes", type=float, nargs=3, default=[4.0, 3.0, 2.0], help="deg"
    )
    parser.add_argument(
        "--periods", type=float, nargs=3, default=[4.0, 3.0, 2.0], help="s"
    )
    args = parse_or_exit(parser, argv)
    ledger = Ledger("multi-joint independence")
    pose = canonical_pose_deg()
    v_lim = mode_velocity_deg_s("stream")
    total = max(args.periods)

    ok = True
    for j, a, T in zip(args.joints, args.amplitudes, args.periods):
        q0 = pose[j]
        ok &= fail_if_outside(
            ledger, j, min(q0, q0 + a), max(q0, q0 + a), f"J{j} inside the soft window"
        )
        pv = raised_cosine_peak_velocity(a, T)
        ok &= ledger.add(
            f"J{j} peak velocity under the STREAM ceiling",
            pv < v_lim[j],
            f"{pv:.2f} vs {v_lim[j]:.2f} deg/s",
        )
    if not ok:
        return ledger.finish(args.json)

    envs = {
        j: raised_cosine(pose[j], a, T)
        for j, a, T in zip(args.joints, args.amplitudes, args.periods)
    }
    periods = dict(zip(args.joints, args.periods))

    def target(t: float) -> list[float]:
        out = list(pose)
        for j, env in envs.items():
            # Each joint repeats its own period for as many whole periods
            # as fit the window, then holds at rest: every joint ends
            # where it started, at zero velocity.
            whole = math.floor(total / periods[j]) * periods[j]
            out[j] = env(t % periods[j]) if t < whole else pose[j]
        return out

    with connect(args) as client:
        if not require_ready(client, ledger):
            return ledger.finish(args.json)
        if not gate(args, ledger, f"stream three envelopes on joints {args.joints}"):
            return ledger.finish(args.json)
        if not go_to(client, pose, ledger):
            return ledger.finish(args.json)
        seen: list = []
        stop = threading.Event()

        def sample() -> None:
            while not stop.is_set():
                seen.extend(collect_status(client, 0.5))

        sampler = threading.Thread(target=sample, daemon=True)
        sampler.start()
        sent = stream_at(client, RATE_HZ, total, target)
        client.stop(clear_queue=True)
        time.sleep(0.5)
        stop.set()
        sampler.join(timeout=2.0)
        first = next(
            (
                k
                for k, s in enumerate(seen)
                if any(abs(float(s.angles[j]) - pose[j]) > 0.05 for j in args.joints)
            ),
            0,
        )
        gaps = {
            j: tracking_gap_deg(sent, seen[first:], seen[first].mono_time_ns, j, 0.0)
            if seen
            else float("inf")
            for j in args.joints
        }
        aggressiveness = {
            j: raised_cosine_peak_velocity(a, T)
            for j, a, T in zip(args.joints, args.amplitudes, args.periods)
        }
        order = sorted(args.joints, key=lambda j: aggressiveness[j])
        for j in args.joints:
            ledger.add(
                f"J{j} tracking gap",
                gaps[j] < 3.0,
                f"{gaps[j]:.3f} deg at {aggressiveness[j]:.2f} deg/s peak",
                required=False,
            )
        monotone = all(
            gaps[a] <= gaps[b] * 1.25 + 0.05 for a, b in zip(order, order[1:])
        )
        ledger.add(
            "gaps scale with aggressiveness",
            monotone,
            "gentle → aggressive: " + ", ".join(f"J{j} {gaps[j]:.3f}" for j in order),
        )
        client.wait_motion(timeout=5.0)
        end = np.array(client.angles())
        err = float(np.max(np.abs(end - np.array(pose))))
        ledger.add("returned to the start pose", err < 0.3, f"max error {err:.3f} deg")
    return ledger.finish(args.json)


if __name__ == "__main__":
    run_main(main)
