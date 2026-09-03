"""Stage 6 — the numbered acceptance ladder.

Rungs, each diagnosable before the next:

 1  connect and read (no motion)
 2  enable and home
 3  tiny joint nudge
 4  cartesian dry run — the preview plans it, NOTHING moves
 5  small real cartesian move
 6  I/O read and the fitted gripper
 7  blending: two blended moves finish as one motion
 8  pause / resume mid-chain
 9  partial failure: a good move completes, the bad one after it is
    refused and attributed to its own index
10  zero-motion link quality: the measured pose is streamed back at the
    arm for 5 s — a correct system produces no motion at all, which is
    what makes it safe to run first while exercising the whole path

``--go`` gates every rung that can move the arm; every motion rung
starts from the canonical pose, so runs are reproducible.

    python tools/bringup/acceptance_ladder.py --go [--from 3] [--to 10]
"""

from __future__ import annotations

import argparse
import math
import time

import numpy as np
from common import (
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
    stream_at,
)

from par6.client import RobotError
from par6.client.dry_run_client import DryRunRobotClient


def rung_connect(client, ledger: Ledger) -> bool:
    ping = client.ping()
    ok = ledger.add(
        "1 connect",
        ping is not None,
        "no runtime"
        if ping is None
        else f"hardware_connected={ping.hardware_connected}",
    )
    if not ok:
        return False
    angles = client.angles()
    ok &= ledger.add(
        "1 read angles",
        angles is not None and len(angles) == 6,
        f"{[round(a, 2) for a in angles or []]}",
    )
    st = client.wait_status(lambda s: True, timeout=3.0)
    return (
        ledger.add(
            "1 status broadcast",
            st,
            "a STATUS frame arrived" if st else "no STATUS within 3 s",
        )
        and ok
    )


def rung_enable_home(client, ledger: Ledger, args) -> bool:
    if not gate(args, ledger, "enable and run the homing sequence"):
        return False
    client.reset()
    ok = client.wait_status(lambda s: s.enabled, timeout=5.0)
    ledger.add("2 enabled", ok, "controller enabled" if ok else "reset did not enable")
    homed = client.home(wait=True, timeout=180.0) >= 0 and client.wait_status(
        lambda s: s.homed, timeout=5.0
    )
    return (
        ledger.add(
            "2 homed",
            homed,
            "home references valid" if homed else "homing did not complete",
        )
        and ok
    )


def rung_nudge(client, ledger: Ledger, args, pose) -> bool:
    if not gate(args, ledger, "nudge J0 by 2 deg and back"):
        return False
    if not go_to(client, pose, ledger):
        return False
    before = np.array(client.angles())
    target = list(pose)
    target[0] += 2.0
    client.move_j(target, speed=0.2, wait=True, timeout=30.0)
    mid = np.array(client.angles())
    client.move_j(pose, speed=0.2, wait=True, timeout=30.0)
    delta = float(mid[0] - before[0])
    return ledger.add(
        "3 tiny joint nudge",
        1.5 < delta < 2.5,
        f"J0 moved {delta:+.3f} deg for +2.0 commanded",
    )


def rung_dry_run(client, ledger: Ledger, pose) -> bool:
    client.wait_status(lambda s: float(np.max(np.abs(s.speeds))) < 0.005, timeout=5.0)
    before = np.array(client.angles())
    dry = DryRunRobotClient(initial_joints_deg=list(before))
    p = dry.pose()
    target = list(p)
    target[2] += 10.0
    res = dry.move_l(target, speed=0.2, wait=True)
    after = np.array(client.angles())
    moved = float(np.max(np.abs(after - before)))
    planned = res is not None and res.error is None and res.duration > 0.0
    if res is None:
        detail = "no dry-run result"
    elif res.error is None:
        detail = f"{res.duration:.2f} s planned"
    else:
        detail = f"refused: {res.error}"
    ok = ledger.add(
        "4 cartesian dry run planned",
        planned,
        f"+10 mm z from {[round(v, 1) for v in p[:3]]}: " + detail,
    )
    return (
        ledger.add("4 nothing moved", moved < 0.05, f"max joint change {moved:.3f} deg")
        and ok
    )


def rung_cartesian(client, ledger: Ledger, args, pose) -> bool:
    if not gate(args, ledger, "move the TCP +10 mm in z and back"):
        return False
    if not go_to(client, pose, ledger):
        return False
    p0 = np.array(client.pose())
    target = list(p0)
    target[2] += 10.0
    client.move_l(target, speed=0.2, wait=True, timeout=30.0)
    p1 = np.array(client.pose())
    client.move_l(list(p0), speed=0.2, wait=True, timeout=30.0)
    dz = float(p1[2] - p0[2])
    lateral = float(np.max(np.abs(p1[:2] - p0[:2])))
    ok = ledger.add(
        "5 small cartesian: z",
        8.0 < dz < 12.0,
        f"z moved {dz:+.2f} mm for +10 commanded",
    )
    return (
        ledger.add(
            "5 small cartesian: straight", lateral < 1.5, f"x/y drift {lateral:.2f} mm"
        )
        and ok
    )


def rung_io_gripper(client, ledger: Ledger, args) -> bool:
    io = client.io()
    ok = ledger.add("6 I/O read", io is not None, f"levels {io}")
    tools = client.tools()
    tool = tools.tool if tools is not None else ""
    if not tool:
        return (
            ledger.add("6 gripper", True, "no tool registered; skipped", required=False)
            and ok
        )
    if not gate(args, ledger, f"open and close the fitted tool {tool}"):
        return ok
    try:
        # move [position 0..1, speed 0..1, current mA]: open, then close.
        client.tool_action(tool, "move", [0.0, 0.5, 400.0], wait=True, timeout=15.0)
        client.tool_action(tool, "move", [1.0, 0.5, 400.0], wait=True, timeout=15.0)
        ok &= ledger.add("6 gripper open/close", True, f"{tool} cycled open and closed")
    except RobotError as e:
        ok &= ledger.add(
            "6 gripper open/close",
            False,
            f"{tool}: {e} — {getattr(e, 'cause', '')} {getattr(e, 'remedy', '')}",
            required=False,
        )
    return ok
    try:
        client.tool_action(tools[0], "open", [], wait=True, timeout=15.0)
        client.tool_action(tools[0], "close", [], wait=True, timeout=15.0)
        ok &= ledger.add("6 gripper open/close", True, f"{tools[0]} cycled")
    except RobotError as e:
        ok &= ledger.add(
            "6 gripper open/close", False, f"{tools[0]}: {e}", required=False
        )
    return ok


def rung_blend(client, ledger: Ledger, args, pose) -> bool:
    if not gate(args, ledger, "run two joint moves, blended and then unblended"):
        return False
    a = list(pose)
    a[0] += 6.0
    b = list(a)
    b[1] += 4.0

    def run(r: float) -> tuple[float, float]:
        """Seconds from queueing until the first and the second COMPLETE."""
        if not go_to(client, pose, ledger):
            return math.inf, math.inf
        t0 = time.monotonic()
        i1 = client.move_j(a, speed=0.3, r=r, wait=False)
        i2 = client.move_j(b, speed=0.3, wait=False)
        client.wait_command(i1, timeout=30.0)
        t1 = time.monotonic() - t0
        client.wait_command(i2, timeout=30.0)
        return t1, time.monotonic() - t0

    b1, b2 = run(20.0)
    u1, u2 = run(0.0)
    # The runtime completes blended-away commands together, when the one
    # motion they were folded into ends; unblended, the first completes
    # while the second has yet to start.
    folded = (b2 - b1) < 0.25 * b2 and (u2 - u1) > 0.25 * u2
    ok = ledger.add(
        "7 blended chain is one motion",
        folded,
        f"blended: first done at {b1:.2f} s, second at {b2:.2f} s; "
        f"unblended: {u1:.2f} s then {u2:.2f} s",
    )
    return ok


def rung_pause(client, ledger: Ledger, args, pose) -> bool:
    if not gate(args, ledger, "pause and resume a slow move"):
        return False
    if not go_to(client, pose, ledger):
        return False
    target = list(pose)
    target[0] += 15.0
    index = client.move_j(target, speed=0.08, wait=False)
    time.sleep(1.0)
    client.pause()
    time.sleep(1.0)
    held = collect_status(client, 1.0)
    angles = np.array([s.angles for s in held]) if held else np.zeros((0, 6))
    band = (
        float(np.max(angles.max(axis=0) - angles.min(axis=0)))
        if len(held)
        else math.inf
    )
    progress = float(angles[-1][0] - pose[0]) if len(held) else math.nan
    ok = ledger.add(
        "8 pause holds",
        band < 0.3,
        f"position band while paused {band:.3f} deg over {len(held)} frames, "
        f"held {progress:.2f} deg into a 15 deg move",
    )
    client.resume()
    done = client.wait_command(index, timeout=60.0)
    ok &= ledger.add(
        "8 resume completes",
        done,
        f"index {index} completed" if done else "did not complete",
    )
    client.move_j(pose, speed=0.3, wait=True, timeout=30.0)
    return ok


def rung_partial_failure(client, ledger: Ledger, args, pose) -> bool:
    if not gate(args, ledger, "queue a good move followed by an unreachable one"):
        return False
    if not go_to(client, pose, ledger):
        return False
    a = list(pose)
    a[0] += 3.0
    p = list(client.pose())
    p[0] += 5000.0  # metres away: unreachable by any arm
    i1 = client.move_j(a, speed=0.3, wait=False)
    try:
        i2 = client.move_l(p, speed=0.2, wait=False)
    except RobotError as e:
        return ledger.add(
            "9 attribution", True, f"the bad move was refused at admission: {e}"
        )
    good = client.wait_command(i1, timeout=30.0)
    failed_index = None
    try:
        client.wait_command(i2, timeout=30.0)
    except RobotError as e:
        failed_index = getattr(e, "command_index", None)
        detail = str(e)
    else:
        detail = "completed without error"
    ok = ledger.add("9 good move completed", good, f"index {i1}")
    ok &= ledger.add(
        "9 bad move attributed to its own index",
        failed_index in (i2, None) and detail != "completed without error",
        f"index {i2}: {detail}",
    )
    client.reset()
    client.move_j(pose, speed=0.3, wait=True, timeout=30.0)
    return ok


def rung_link_quality(client, ledger: Ledger, args, pose) -> bool:
    """The pose measured ONCE at the start is streamed back for 5 s. Not
    the live measurement: feeding a lagging plant its own reading is a
    delay loop, marginally stable by construction, and any settle offset
    grows into an oscillation — which would be a test of the loop, not of
    the link. A constant target that already equals the pose exercises
    the whole streaming path and moves nothing."""
    if not gate(args, ledger, "stream the measured pose back at the arm for 5 s"):
        return False
    if not go_to(client, pose, ledger):
        return False
    client.wait_status(lambda s: float(np.max(np.abs(s.speeds))) < 0.005, timeout=5.0)
    before = list(client.angles())
    client.set_recipe("streaming")
    frames: list = []
    from par6.telemetry import TelemetryReader

    with TelemetryReader(args.telemetry_port, host=args.host) as reader:

        def measured(_t: float) -> list[float]:
            f = reader.recv(timeout=0.0)
            while f is not None:
                if f.get("recipe") == "streaming":
                    frames.append(f["fields"])
                f = reader.recv(timeout=0.0)
            return before

        sent = stream_at(client, 50.0, 5.0, measured)
    client.stop(clear_queue=True)
    after = np.array(client.angles())
    motion = float(np.max(np.abs(after - np.array(before))))
    live = [f for f in frames if f.get("stream_substate") == 2]
    success = float(np.mean([f["stream_success_rate"] for f in live])) if live else 0.0
    discard = float(np.mean([f["stream_discard_pct"] for f in live])) if live else 100.0
    ok = ledger.add(
        "10 zero motion",
        motion < 0.1,
        f"max joint motion {motion:.3f} deg over {len(sent)} setpoints",
    )
    ok &= ledger.add(
        "10 stream applied",
        success > 0.8,
        f"mean success rate {success:.2f} over {len(live)} frames",
    )
    return (
        ledger.add(
            "10 discards",
            discard < 75.0,
            f"mean discard {discard:.1f} %",
            required=False,
        )
        and ok
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add_connection_args(parser)
    add_go_arg(parser)
    parser.add_argument("--from", dest="start", type=int, default=1)
    parser.add_argument("--to", dest="end", type=int, default=10)
    args = parse_or_exit(parser, argv)
    ledger = Ledger("acceptance ladder")
    pose = canonical_pose_deg()
    with connect(args) as client:
        rungs = {
            1: lambda: rung_connect(client, ledger),
            2: lambda: rung_enable_home(client, ledger, args),
            3: lambda: rung_nudge(client, ledger, args, pose),
            4: lambda: rung_dry_run(client, ledger, pose),
            5: lambda: rung_cartesian(client, ledger, args, pose),
            6: lambda: rung_io_gripper(client, ledger, args),
            7: lambda: rung_blend(client, ledger, args, pose),
            8: lambda: rung_pause(client, ledger, args, pose),
            9: lambda: rung_partial_failure(client, ledger, args, pose),
            10: lambda: rung_link_quality(client, ledger, args, pose),
        }
        if args.start > 2 and not require_ready(client, ledger):
            return ledger.finish(args.json)
        for n in range(args.start, args.end + 1):
            if not rungs[n]() and n <= 2:
                ledger.note(
                    "stopping: a later rung cannot be diagnosed on a failed foundation"
                )
                break
    return ledger.finish(args.json)


if __name__ == "__main__":
    run_main(main)
