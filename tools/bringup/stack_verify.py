"""Stage 2 — three falsifiable claims about the stack, with numbers.

1. Convergence: at the canonical pose, after 1.5 s of settling, a 4 s
   sample of the broadcast holds each joint inside a 0.002 rad band with
   a median offset from the commanded pose under 0.002 rad.
2. Command path live: a small joint move and its return — the MEASURED
   encoder delta is the proof, and no other joint moved.
3. Gravity on the wire: while the arm holds under the gravity
   feedforward, at least one joint draws current (the vertical base axis
   reading zero is correct). Advisory on the simulator, whose kinematic
   plant would read the feedforward as motion, so there nothing is
   toggled and the check reads whatever the runtime already applies.

Every toggle happens while NOT streaming: a toggle mid-stream costs a
round trip that the stream watchdog reads as a dead link.

    python tools/bringup/stack_verify.py --go
"""

from __future__ import annotations

import argparse

import numpy as np
from common import (
    DEG,
    Ledger,
    add_connection_args,
    add_go_arg,
    canonical_pose_deg,
    collect_status,
    connect,
    gate,
    go_to,
    parse_or_exit,
    require_ready,
    run_main,
    telemetry_fields,
)

BAND_RAD = 0.002
NUDGE_DEG = 2.0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add_connection_args(parser)
    add_go_arg(parser)
    parser.add_argument("--nudge-joint", type=int, default=0)
    args = parse_or_exit(parser, argv)
    ledger = Ledger("stack verification")
    pose = canonical_pose_deg()

    with connect(args) as client:
        if not require_ready(client, ledger):
            return ledger.finish(args.json)
        sim = bool(client.is_simulator())
        if not gate(
            args, ledger, f"move to the canonical pose {[round(v, 1) for v in pose]}"
        ):
            return ledger.finish(args.json)
        if not go_to(client, pose, ledger):
            return ledger.finish(args.json)

        # 1. convergence
        collect_status(client, 1.5)
        window = collect_status(client, 4.0)
        angles = np.array([s.angles for s in window]) / DEG
        band = angles.max(axis=0) - angles.min(axis=0)
        offset = np.abs(np.median(angles, axis=0) - np.array(pose) / DEG)
        ledger.add(
            "convergence: band",
            bool(np.all(band < BAND_RAD)),
            f"per-joint band {np.array2string(band, precision=5)} rad over {len(window)} frames",
        )
        ledger.add(
            "convergence: offset",
            bool(np.all(offset < BAND_RAD)),
            f"median offset from commanded {np.array2string(offset, precision=5)} rad",
        )

        # 2. command path live
        j = args.nudge_joint
        before = np.array(client.angles())
        nudged = list(pose)
        nudged[j] += NUDGE_DEG
        client.move_j(nudged, speed=0.2, wait=True, timeout=30.0)
        at_nudge = np.array(client.angles())
        client.move_j(pose, speed=0.2, wait=True, timeout=30.0)
        back = np.array(client.angles())
        delta = float(at_nudge[j] - before[j])
        others = np.delete(np.abs(at_nudge - before), j)
        ledger.add(
            "command path: encoder followed",
            abs(delta - NUDGE_DEG) < 0.25 * NUDGE_DEG,
            f"J{j} measured delta {delta:+.3f} deg for a {NUDGE_DEG:+.1f} deg command",
        )
        ledger.add(
            "command path: others still",
            bool(np.all(others < 0.1)),
            f"max other-joint motion {float(others.max()):.3f} deg",
        )
        ledger.add(
            "command path: returned",
            float(np.max(np.abs(back - before))) < 0.2,
            f"max return error {float(np.max(np.abs(back - before))):.3f} deg",
        )

        # 3. gravity on the wire — sampled while idle, toggled while idle,
        # and only on hardware.
        toggled = False
        if not sim and not client.is_freedrive(timeout=1.0):
            client.set_gravity_comp(True)
            toggled = True
        frames = telemetry_fields(
            client, args.telemetry_port, "diagnostics", 2.0, host=args.host
        )
        g_frames = telemetry_fields(
            client, args.telemetry_port, "full", 1.0, host=args.host
        )
        if toggled:
            client.set_gravity_comp(False)
        if frames:
            currents = np.nan_to_num(
                np.array([f["motor_currents_ma"] for f in frames], dtype=np.float64)
            )
            mean_abs = np.abs(currents).mean(axis=0)
            g = (
                np.abs(np.array([f["gravity_torques"] for f in g_frames])).mean(axis=0)
                if g_frames
                else None
            )
            loaded = mean_abs[1:6]
            ledger.add(
                "gravity on the wire",
                bool(np.any(loaded > 20.0)),
                f"mean |current| per node {np.array2string(mean_abs, precision=0)} mA"
                + (
                    f"; published G(q) {np.array2string(g, precision=2)} Nm"
                    if g is not None
                    else ""
                )
                + " — J0 on the vertical axis reading ~0 is correct",
                required=not sim,
            )
        else:
            ledger.add(
                "gravity on the wire",
                False,
                "no telemetry frames arrived",
                required=not sim,
            )
    return ledger.finish(args.json)


if __name__ == "__main__":
    run_main(main)
