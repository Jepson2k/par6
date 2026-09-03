"""Stage 5 — the control loop, baseline versus contention.

Reads the runtime's own loop statistics (``loop_stats``) over a window,
then — with ``--load`` — again while burner processes pinned to the
NON-real-time cores stride through a buffer far larger than the last-
level cache. That is the methodological point: isolation tuning only
shows when something competes, an idle A/B measures nothing, and CPU
time alone cannot touch a SCHED_FIFO 99 thread — only the shared L3 and
memory bandwidth can, which is what the striding load contends for.

    python tools/bringup/loop_benchmark.py [--seconds 20] [--load] [--rt-cpu 3]

A per-phase breakdown of the tick itself comes from the runtime, not this
script: start it with ``par6d --tick-profile`` (or ``PAR6_TICK_PROFILE=1``)
and the RT log carries, once a second, the running maximum of every phase
and the phase times of the last tick that overran its deadline.
"""

from __future__ import annotations

import argparse
import multiprocessing as mp
import os
import time
from multiprocessing.synchronize import Event as EventType

import numpy as np
from common import (
    Ledger,
    add_connection_args,
    connect,
    parse_or_exit,
    run_main,
    tick_dt_s,
)


def _burn(cpu: int, stop: EventType) -> None:
    try:
        os.sched_setaffinity(0, {cpu})
    except OSError:
        pass
    # 64 MiB, walked with a stride of one cache line so every access is a
    # miss the prefetcher cannot hide, and rewritten so the lines stay
    # dirty and cost write-back bandwidth too.
    buf = np.ones((64 << 20) // 8, dtype=np.float64)
    stride = 8
    while not stop.is_set():
        buf[::stride] += 1.0
        buf[stride // 2 :: stride] -= 1.0


def sample(client, seconds: float, dt: float, ledger: Ledger, label: str) -> dict:
    client.reset_loop_stats()
    time.sleep(seconds)
    st = client.loop_stats()
    if st is None:
        ledger.add(f"{label}: loop stats", False, "no answer")
        return {}
    row = {
        "p50_us": st.p50_period_s * 1e6,
        "p99_us": st.p99_period_s * 1e6,
        "max_us": st.max_period_s * 1e6,
        "std_us": st.std_period_s * 1e6,
        "overruns": st.overrun_count,
        "rt_fifo": st.rt_fifo,
        "rt_pinned": getattr(st, "rt_pinned", None),
    }
    ledger.add(
        f"{label}: p99 inside 1.5x dt",
        st.p99_period_s < 1.5 * dt,
        f"p50 {row['p50_us']:.0f} us, p99 {row['p99_us']:.0f} us, max {row['max_us']:.0f} us, "
        f"std {row['std_us']:.0f} us, overruns {row['overruns']} in {seconds:.0f} s "
        f"(dt {dt * 1e6:.0f} us; SCHED_FIFO {st.rt_fifo}, pinned {row['rt_pinned']})",
        required=False,
    )
    return row


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add_connection_args(parser)
    parser.add_argument("--seconds", type=float, default=20.0, help="window per phase")
    parser.add_argument(
        "--load", action="store_true", help="also measure under contention"
    )
    parser.add_argument(
        "--rt-cpu", type=int, default=3, help="the core the RT thread is pinned to"
    )
    args = parse_or_exit(parser, argv)
    ledger = Ledger("loop benchmark")
    dt = tick_dt_s()

    with connect(args) as client:
        if client.ping() is None:
            ledger.add("runtime answers", False, "no runtime at the address")
            return ledger.finish(args.json)
        base = sample(client, args.seconds, dt, ledger, "baseline")
        if args.load:
            cpus = sorted(os.sched_getaffinity(0) - {args.rt_cpu})
            if not cpus:
                ledger.add(
                    "contention", False, "no non-RT core to burn on", required=False
                )
                return ledger.finish(args.json)
            stop = mp.Event()
            burners = [
                mp.Process(target=_burn, args=(c, stop), daemon=True) for c in cpus
            ]
            for b in burners:
                b.start()
            time.sleep(1.0)
            loaded = sample(
                client, args.seconds, dt, ledger, f"contention on cores {cpus}"
            )
            stop.set()
            for b in burners:
                b.join(timeout=5.0)
            if base and loaded:
                ledger.add(
                    "contention effect",
                    True,
                    f"p99 {base['p99_us']:.0f} -> {loaded['p99_us']:.0f} us, "
                    f"max {base['max_us']:.0f} -> {loaded['max_us']:.0f} us, "
                    f"overruns {base['overruns']} -> {loaded['overruns']}",
                    required=False,
                )
    return ledger.finish(args.json)


if __name__ == "__main__":
    run_main(main)
