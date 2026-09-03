"""Phase B — per-joint gravity identification by a slow staircase.

Carries one joint across a range in small profiled steps, up AND down so
averaging the two passes cancels Coulomb friction, while every other
joint holds. Each step is a ``move_j`` at ``--speed`` followed by a
dwell; only the tail of the dwell is logged, when the joint is at rest
and the drive's current is the holding torque alone — a drive in motion
carries its loop's dynamic effort on top of gravity, which is not what
is being identified. The position loop is in command throughout; the
holding torque is the runtime's kt-calibrated measured torque, and the
model's answer at the very same measured pose is the ``gravity_torques``
the runtime publishes every tick — so each sample carries the full
six-joint pose, the measured torque and G(q) at it, which is what makes
the sample-wise fit in ``fit_sweeps.py`` possible.

Pre-position to a clearance pose first (``--pre joint3=1.1``): sweeps
that start where anything touches produce convincing, wrong fits (see
the README). The excursion is checked against the joint's soft window
before anything moves and refused with the numbers.

    python tools/gravity_calibration/pd_sweep_id.py --go --joint 2 \\
        --range 0.8 --speed 0.12 --pre joint1=-1.3 --pre joint3=0.0
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
from harness import (
    DEG,
    RESULTS,
    Ledger,
    TelemetryTap,
    add_connection_args,
    add_go_arg,
    analyse_sweep,
    connect,
    fail_if_outside,
    freeze_and_lower,
    gate,
    joint_names,
    mode_velocity_rad_s,
    move_and_verify,
    parse_or_exit,
    parse_pre,
    require_ready,
    run_main,
    write_json,
)


def staircase_levels(
    start: float, extent: float, step: float
) -> list[tuple[float, int]]:
    """The rest levels of one joint, up to ``start + extent`` and back,
    each with its direction of approach (+1 up, -1 down)."""
    sign = 1.0 if extent >= 0 else -1.0
    n_levels = max(1, int(round(abs(extent) / step)))
    levels = [start + sign * step * k for k in range(1, n_levels + 1)]
    return [(level, 1) for level in levels] + [
        (level, -1) for level in reversed([start] + levels[:-1])
    ]


def report_fit(ledger: Ledger, fit: dict, joint: int) -> None:
    if not fit.get("usable"):
        ledger.add(
            f"J{joint} fit", False, fit.get("reason", "unusable"), required=False
        )
        return
    ledger.add(
        f"J{joint} sweep resolves the phase",
        fit["range_rad"] >= 0.5,
        f"{fit['range_rad']:.2f} rad swept over {fit['n']} samples",
        required=False,
    )
    if not fit["signal_above_friction"]:
        ledger.note(
            f"J{joint}: gravity signal {fit['measured_amplitude_nm']:.3f} Nm is below "
            f"the friction floor {fit['friction_nm']:.3f} Nm — nothing to tune here"
        )
        return
    ledger.note(
        f"J{joint} quick fit: k {fit['k']:.3f}, offset {fit['offset_rad']:+.3f} rad, "
        f"bias {fit['bias_nm']:+.3f} Nm, friction {fit['friction_nm']:.3f} Nm, "
        f"rms {fit['rms_measured_nm']:.3f} Nm — run fit_sweeps.py for the verdict"
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add_connection_args(parser)
    add_go_arg(parser)
    parser.add_argument("--joint", type=int, default=2, help="0-based joint index")
    parser.add_argument(
        "--range", type=float, default=0.8, help="sweep extent from the start [rad]"
    )
    parser.add_argument("--speed", type=float, default=0.12, help="ramp speed [rad/s]")
    parser.add_argument("--step", type=float, default=0.05, help="staircase step [rad]")
    parser.add_argument(
        "--dwell",
        type=float,
        default=0.8,
        help="rest at each step [s]; the last half is logged",
    )
    parser.add_argument(
        "--pre",
        action="append",
        default=[],
        help="pre-position a joint first, e.g. --pre joint3=1.1 [rad] (repeatable)",
    )
    parser.add_argument(
        "--vel-abort", type=float, default=1.0, help="finite-difference abort [rad/s]"
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
    ledger = Ledger(f"PD sweep J{j}")
    names = joint_names()
    tag = f"_{args.tag}" if args.tag else ""

    with connect(args) as client:
        if not require_ready(client, ledger):
            return ledger.finish(args.json)
        sim = bool(client.is_simulator())
        angles = client.angles()
        if not angles:
            ledger.add("start pose", False, "no angles")
            return ledger.finish(args.json)
        q0 = np.asarray(angles, dtype=np.float64) / DEG
        pre = q0.copy()
        for idx, value in parse_pre(args.pre, names, j).items():
            pre[idx] = value
        lo, hi = sorted((float(pre[j]), float(pre[j] + args.range)))

        ok = fail_if_outside(
            ledger, j, lo * DEG, hi * DEG, f"J{j} sweep inside the soft window"
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
        if not gate(
            args, ledger, f"pre-position to {np.round(pre, 3)} rad and sweep J{j}"
        ):
            return ledger.finish(args.json)
        if not move_and_verify(
            client, pre, ledger, "pre-positioned", speed=0.1, tol_deg=args.pose_tol_deg
        ):
            return ledger.finish(args.json)

        samples: list[dict] = []
        aborted: str | None = None
        velocity_limit = mode_velocity_rad_s("exec")[j]
        speed_frac = min(1.0, max(0.02, args.speed / velocity_limit))
        with TelemetryTap(client, args.telemetry_port, args.host) as tap:
            first = tap.wait_frame(3.0)
            if first is None:
                ledger.add("telemetry", False, "no `full` frames within 3 s")
                return ledger.finish(args.json)
            # The staircase starts from the pose the encoders report, not
            # the commanded one: a drive that settled a little short would
            # otherwise be yanked to the command by the first step.
            hold = first.q.copy()
            t0 = time.monotonic()
            for level, d in staircase_levels(hold[j], args.range, args.step):
                target = hold.copy()
                target[j] = level
                index = client.move_j(
                    [float(v) * DEG for v in target], speed=speed_frac, wait=False
                )
                if index < 0:
                    aborted = f"step to {level:.3f} rad refused"
                    break
                budget = time.monotonic() + 2.0 * abs(args.step) / args.speed + 10.0
                while not client.wait_command(index, timeout=0.05):
                    vel = tap.velocity()
                    if vel is not None and float(np.max(np.abs(vel))) > args.vel_abort:
                        aborted = (
                            f"finite-difference velocity {float(np.max(np.abs(vel))):+.2f} "
                            f"rad/s stepping to {level:.3f} rad"
                        )
                        break
                    if time.monotonic() > budget:
                        aborted = f"step to {level:.3f} rad did not complete"
                        break
                if aborted:
                    break
                rest = time.monotonic()
                while time.monotonic() - rest < args.dwell:
                    time.sleep(0.05)
                    vel = tap.velocity()
                    if vel is not None and float(np.max(np.abs(vel))) > args.vel_abort:
                        aborted = (
                            f"finite-difference velocity {float(np.max(np.abs(vel))):+.2f} "
                            f"rad/s at rest on {level:.3f} rad"
                        )
                        break
                if aborted:
                    break
                for fr in tap.since(rest + args.dwell / 2.0):
                    samples.append(
                        {
                            "t": round(fr.t - t0, 4),
                            "tgt": round(float(level), 5),
                            "q": round(float(fr.q[j]), 5),
                            "vel_meas": round(float(fr.qd[j]), 4),
                            "tau": round(float(fr.tau[j]), 4),
                            "g": round(float(fr.g[j]), 4),
                            "q6": [round(float(v), 5) for v in fr.q],
                            "tau6": [round(float(v), 4) for v in fr.tau],
                            "g6": [round(float(v), 4) for v in fr.g],
                            "dir": d,
                        }
                    )
            if aborted:
                client.stop(clear_queue=True)

        record = {
            "joint": j,
            "joint_name": names[j],
            "range_rad": args.range,
            "speed_rad_s": args.speed,
            "step_rad": args.step,
            "dwell_s": args.dwell,
            "pre_rad": pre.tolist(),
            "start_rad": q0.tolist(),
            "simulator": sim,
            "aborted": aborted,
            "samples": samples,
        }
        out = write_json(
            args.out / f"pdsweep_J{j}{tag}_{int(time.time())}.json", record
        )
        if aborted:
            freeze_and_lower(client, ledger, aborted, q0, args.pose_tol_deg)
            return ledger.finish(args.json)
        move_and_verify(
            client,
            q0,
            ledger,
            "returned to the start pose",
            speed=0.1,
            tol_deg=args.pose_tol_deg,
        )
        ledger.add(
            "sweep logged", len(samples) > 20, f"{len(samples)} samples in {out.name}"
        )
        report_fit(ledger, analyse_sweep(record), j)
    return ledger.finish(args.json)


if __name__ == "__main__":
    run_main(main)
