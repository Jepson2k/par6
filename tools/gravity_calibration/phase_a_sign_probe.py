"""Phase A — the sign convention, by hand, with NO torque.

Streams the runtime's measured joint angles while a person moves one
joint at a time by hand and prints a line whenever a joint has moved
more than ``--threshold`` from its last printed value. The runtime
reports URDF q, so the sign of the first delta against the direction the
hand pushed fixes the motor→URDF convention — before any torque is ever
applied. The full trace goes to ``results/phase_a_trace_*.jsonl``.

The drives must be idle: par6 has no torque-off verb by design (it never
torque-offs a loaded arm), so run this on a freshly started ``par6d``,
before any ``reset`` — the daemon boots with the drives disabled and
limps them on the way out. An ENABLED arm is refused.

    python tools/gravity_calibration/phase_a_sign_probe.py [--duration 120]
"""

from __future__ import annotations

import argparse
import json
import time

import numpy as np
from harness import (
    DEG,
    RESULTS,
    Ledger,
    add_connection_args,
    connect,
    joint_names,
    parse_or_exit,
    run_main,
    write_json,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add_connection_args(parser)
    parser.add_argument("--duration", type=float, default=120.0, help="seconds")
    parser.add_argument(
        "--threshold", type=float, default=0.03, help="print a move above this [rad]"
    )
    parser.add_argument("--tag", default="", help="suffix for the output files")
    args = parse_or_exit(parser, argv)
    ledger = Ledger("phase A sign probe")
    names = joint_names()
    tag = f"_{args.tag}" if args.tag else ""
    stamp = int(time.time())

    with connect(args) as client:
        if client.ping() is None:
            ledger.add("runtime answers", False, "no runtime at the address")
            return ledger.finish(args.json)
        first: list[np.ndarray] = []
        enabled: list[bool] = []

        def capture(s) -> bool:  # noqa: ANN001 — StatusBuffer, on the loop thread
            first.append(np.asarray(s.angles, dtype=np.float64) / DEG)
            enabled.append(bool(s.enabled))
            return True

        if not client.wait_status(capture, timeout=5.0):
            ledger.add("status broadcast", False, "no STATUS within 5 s")
            return ledger.finish(args.json)
        if enabled[0]:
            ledger.add(
                "drives idle",
                False,
                "the arm is ENABLED — restart par6d (it boots the drives idle) "
                "and run this before any reset; par6 has no torque-off verb",
            )
            return ledger.finish(args.json)
        ledger.add("drives idle", True, "not enabled: nothing pushes back")

        start = first[0]
        last_printed = start.copy()
        first_sign: list[int | None] = [None] * len(start)
        RESULTS.mkdir(parents=True, exist_ok=True)
        trace_path = RESULTS / f"phase_a_trace{tag}_{stamp}.jsonl"
        trace = trace_path.open("w")
        t0 = time.monotonic()
        deadline = t0 + args.duration
        print(
            f"START pose {np.round(start, 4)} rad — move one joint at a time",
            flush=True,
        )

        def watch(s) -> bool:  # noqa: ANN001 — StatusBuffer, on the loop thread
            q = np.asarray(s.angles, dtype=np.float64) / DEG
            t = time.monotonic() - t0
            trace.write(
                json.dumps({"t": round(t, 3), "q": [round(v, 5) for v in q]}) + "\n"
            )
            for j, v in enumerate(q):
                if abs(v - last_printed[j]) > args.threshold:
                    delta = v - start[j]
                    if first_sign[j] is None:
                        first_sign[j] = 1 if v > last_printed[j] else -1
                    print(
                        f"MOVE {names[j]} (J{j}): {v:+.4f} rad "
                        f"(delta vs start {delta:+.4f})",
                        flush=True,
                    )
                    last_printed[j] = v
            return time.monotonic() >= deadline

        client.wait_status(watch, timeout=args.duration + 5.0)
        trace.close()
        end = last_printed
        moved = [j for j in range(len(start)) if first_sign[j] is not None]
        summary = {
            "start_rad": start.tolist(),
            "end_rad": end.tolist(),
            "joints": [
                {
                    "index": j,
                    "name": names[j],
                    "first_motion_sign": first_sign[j],
                    "delta_rad": float(end[j] - start[j]),
                }
                for j in range(len(start))
            ],
            "convention": "the runtime reports URDF q: a positive delta is positive "
            "URDF rotation about that joint's axis",
        }
        write_json(RESULTS / f"phase_a_signs{tag}_{stamp}.json", summary)
        ledger.add(
            "joints moved by hand",
            bool(moved),
            ", ".join(
                f"{names[j]} first {'+' if first_sign[j] == 1 else '-'}" for j in moved
            )
            or "nothing moved",
            required=False,
        )
        ledger.note(f"trace: {trace_path}")
    return ledger.finish(args.json)


if __name__ == "__main__":
    run_main(main)
