"""Phase D — float validation: does the model hold the arm by itself?

Sequence: pre-position (``--pre``), lift the target joint by ``--lift``
under position control with the runtime's own gravity feedforward, then
enter freedrive. par6's freedrive IS the fade: IDLE under G(q) with no
position term, the feedforward ramped in by the runtime's torque-rate
limit, the drive's damping term still in the frame. With ``[freedrive]
drift_lock`` configured the runtime adds a slow clamped integral once
the arm is still, and that integral — ``commanded_torques −
gravity_torques`` in telemetry — converges to the model's error at that
pose; without it the drift rate carries the same information. Then
re-engage the position hold (leave freedrive) and return.

Unlike a per-joint float with the neighbours servoed, par6 floats the
WHOLE arm: every loaded joint is validated at once and every joint is
watched. Aborts (velocity by finite difference of the measured position,
leaving the ``--window``) leave freedrive — the position hold re-engages
at the current pose — report, and lower under control; never torque-off.

    python tools/gravity_calibration/auto_float_test.py --go --joint 2 \\
        --lift 0.6 --pre joint1=-1.3 --float-s 12
"""

from __future__ import annotations

import argparse
import time
import tomllib
from pathlib import Path

import numpy as np
from harness import (
    DEG,
    RESULTS,
    Ledger,
    TelemetryTap,
    add_connection_args,
    add_go_arg,
    connect,
    fail_if_outside,
    freeze_and_lower,
    gate,
    joint_names,
    move_and_verify,
    parse_or_exit,
    parse_pre,
    require_ready,
    robot_config,
    run_main,
    write_json,
)

LOADED = [1, 2, 3, 4, 5]
DRIFT_TOL_RAD = 0.05
TAIL_S = 3.0


def drift_lock_configured(client) -> bool:  # noqa: ANN001 — RobotClient
    """The runtime's own config, not the packaged copy, when it serves one."""
    bundle = client.config_bundle()
    if bundle and bundle.get("robot_toml"):
        toml = tomllib.loads(bundle["robot_toml"])
    else:
        toml = robot_config()
    return bool(toml.get("freedrive", {}).get("drift_lock", False))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add_connection_args(parser)
    add_go_arg(parser)
    parser.add_argument("--joint", type=int, default=2, help="0-based joint index")
    parser.add_argument(
        "--lift", type=float, default=0.6, help="lift from the pre pose [rad]"
    )
    parser.add_argument(
        "--pre",
        action="append",
        default=[],
        help="pre-position a joint first, e.g. --pre joint3=1.1 [rad] (repeatable)",
    )
    parser.add_argument(
        "--float-s", type=float, default=12.0, help="float duration [s]"
    )
    parser.add_argument(
        "--vel-abort", type=float, default=1.0, help="finite-difference abort [rad/s]"
    )
    parser.add_argument(
        "--window",
        type=float,
        default=0.5,
        help="abort when any joint leaves this [rad]",
    )
    parser.add_argument(
        "--pose-tol-deg",
        type=float,
        default=0.5,
        help="max encoder error accepted after a profiled move (the torque-level "
        "simulator sags a few degrees under load; hardware drives do not)",
    )
    parser.add_argument("--tag", default="", help="suffix for the output filename")
    parser.add_argument("--out", type=Path, default=RESULTS, help="output directory")
    args = parse_or_exit(parser, argv)
    j = args.joint
    ledger = Ledger(f"float validation J{j}")
    names = joint_names()
    tag = f"_{args.tag}" if args.tag else ""

    with connect(args) as client:
        if not require_ready(client, ledger):
            return ledger.finish(args.json)
        angles = client.angles()
        if not angles:
            ledger.add("start pose", False, "no angles")
            return ledger.finish(args.json)
        q0 = np.asarray(angles, dtype=np.float64) / DEG
        pre = q0.copy()
        for idx, value in parse_pre(args.pre, names, j).items():
            pre[idx] = value
        lifted = pre.copy()
        lifted[j] += args.lift
        lo, hi = sorted((float(pre[j]), float(lifted[j])))
        ok = fail_if_outside(
            ledger, j, lo * DEG, hi * DEG, f"J{j} lift inside the soft window"
        )
        for idx in parse_pre(args.pre, names, j):
            ok &= fail_if_outside(
                ledger,
                idx,
                pre[idx] * DEG,
                pre[idx] * DEG,
                f"J{idx} pre-position inside the soft window",
            )
        if not ok:
            return ledger.finish(args.json)
        lock = drift_lock_configured(client)
        ledger.note(
            "drift lock configured: the integral reads the model error"
            if lock
            else "drift lock OFF: only the drift rate reads the model error"
        )
        if not gate(
            args,
            ledger,
            f"lift J{j} by {args.lift:+.2f} rad and float {args.float_s:.0f} s",
        ):
            return ledger.finish(args.json)
        if not move_and_verify(
            client, pre, ledger, "pre-positioned", speed=0.1, tol_deg=args.pose_tol_deg
        ):
            return ledger.finish(args.json)
        if not move_and_verify(
            client, lifted, ledger, "lifted under position control", speed=0.1
        ):
            freeze_and_lower(
                client, ledger, "lift did not reach its target", q0, args.pose_tol_deg
            )
            return ledger.finish(args.json)

        samples: list[dict] = []
        aborted: str | None = None
        with TelemetryTap(client, args.telemetry_port, args.host) as tap:
            if tap.wait_frame(3.0) is None:
                ledger.add("telemetry", False, "no `full` frames within 3 s")
                return ledger.finish(args.json)
            client.freedrive(True)
            # The broadcast's own verdict — IDLE, homed, enabled and the
            # feedforward applied — a flag alone would report a float the
            # arm is not doing.
            entered = client.wait_status(lambda s: s.freedrive, timeout=3.0)
            if not ledger.add(
                "freedrive entered", entered, "G(q) only, no position hold"
            ):
                freeze_and_lower(
                    client, ledger, "freedrive refused", q0, args.pose_tol_deg
                )
                return ledger.finish(args.json)
            start = tap.latest()
            assert start is not None
            q_float = start.q.copy()
            prev = start
            t0 = time.monotonic()
            while time.monotonic() - t0 < args.float_s:
                fr = tap.latest()
                if fr is None or fr.t <= prev.t:
                    time.sleep(0.005)
                    continue
                windowed = tap.velocity()
                vel = windowed if windowed is not None else np.zeros_like(fr.q)
                if float(np.max(np.abs(vel))) > args.vel_abort:
                    aborted = f"velocity {float(np.max(np.abs(vel))):+.2f} rad/s"
                    break
                if float(np.max(np.abs(fr.q - q_float))) > args.window:
                    aborted = f"left the window by {float(np.max(np.abs(fr.q - q_float))):.3f} rad"
                    break
                samples.append(
                    {
                        "t": round(fr.t - t0, 4),
                        "q6": [round(float(v), 5) for v in fr.q],
                        "vel6": [round(float(v), 4) for v in vel],
                        "g6": [round(float(v), 4) for v in fr.g],
                        "tau_cmd6": [round(float(v), 4) for v in fr.tau_cmd],
                        "tau6": [round(float(v), 4) for v in fr.tau],
                    }
                )
                prev = fr
            client.freedrive(False)
            held = client.wait_status(lambda s: not s.gravity_comp, timeout=3.0)
            ledger.add("position hold re-engaged", held, "freedrive left")

        if aborted:
            freeze_and_lower(client, ledger, aborted, q0, args.pose_tol_deg)
            write_json(
                args.out / f"autofloat_J{j}{tag}_{int(time.time())}.json",
                {"joint": j, "aborted": aborted, "samples": samples},
            )
            return ledger.finish(args.json)

        q_end = np.array(samples[-1]["q6"])
        drift = q_end - q_float
        tail = [s for s in samples if s["t"] >= samples[-1]["t"] - TAIL_S] or samples
        g_tail = np.mean([s["g6"] for s in tail], axis=0)
        integral = np.mean([np.subtract(s["tau_cmd6"], s["g6"]) for s in tail], axis=0)
        residual_pct = 100.0 * np.abs(integral) / np.maximum(np.abs(g_tail), 1e-6)
        worst = int(max(LOADED, key=lambda i: abs(drift[i])))
        ledger.add(
            "drift bounded",
            float(np.max(np.abs(drift[LOADED]))) < DRIFT_TOL_RAD,
            f"J{worst} drifted {drift[worst]:+.4f} rad over {samples[-1]['t']:.1f} s "
            f"(per joint {np.array2string(drift, precision=4)})",
        )
        if lock:
            head = [s for s in samples if s["t"] <= samples[-1]["t"] - 1.0] or samples[
                :1
            ]
            slope = (
                integral[j]
                - (np.array(head[-1]["tau_cmd6"]) - np.array(head[-1]["g6"]))[j]
            )
            ledger.note(
                f"J{j} integral {integral[j]:+.4f} Nm, climbing {slope:+.4f} Nm over the "
                "last second — the lock integrates slowly by design; float for several "
                "of its time constants before reading the residual as the model error"
            )
            ledger.add(
                f"J{j} residual below 10% of the model",
                residual_pct[j] < 10.0,
                f"integral {integral[j]:+.3f} Nm vs G {g_tail[j]:+.3f} Nm "
                f"({residual_pct[j]:.1f}%); all joints {np.array2string(integral, precision=3)} Nm",
                required=False,
            )
        move_and_verify(
            client, pre, ledger, "lowered under position control", speed=0.1
        )
        move_and_verify(
            client,
            q0,
            ledger,
            "returned to the start pose",
            speed=0.1,
            tol_deg=args.pose_tol_deg,
        )
        write_json(
            args.out / f"autofloat_J{j}{tag}_{int(time.time())}.json",
            {
                "joint": j,
                "joint_name": names[j],
                "lift_rad": args.lift,
                "drift_lock": lock,
                "float_s": args.float_s,
                "pose_rad": q_float.tolist(),
                "drift_rad": drift.tolist(),
                "model_tail_nm": g_tail.tolist(),
                "integral_tail_nm": integral.tolist(),
                "residual_pct": residual_pct.tolist(),
                "samples": samples,
            },
        )
    return ledger.finish(args.json)


if __name__ == "__main__":
    run_main(main)
