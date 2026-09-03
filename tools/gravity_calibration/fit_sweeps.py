"""Phase C — fit the sweep logs against the model the runtime enforces.

For each ``pdsweep_*.json`` the measured holding torque is fitted to
``A·sin(q + phi) + fric·dir + c`` and the runtime's published G(q) at
the same samples to ``A'·sin(q + phi')``; with every other joint held
the gravity torque on a revolute joint IS a sinusoid in its angle, so

    k       = A / A'        — model scale (≈ 1: the masses are right)
    offset  = phi − phi'    — motor zero → URDF zero (≈ 0: no zero slip)
    c, fric                 — bias and Coulomb friction [N·m]

``--qmin`` drops samples where the swept joint is still below a value —
the contact region near a rest pose, see the README — before fitting.

    python tools/gravity_calibration/fit_sweeps.py results/pdsweep_J2_*.json [--qmin 0.4]
"""

from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

from harness import Ledger, analyse_sweep, parse_or_exit, run_main, write_json

SCALE_TOL = 0.15
OFFSET_TOL_RAD = 0.05


def judge(ledger: Ledger, name: str, joint: int, fit: dict) -> None:
    if not fit.get("usable"):
        ledger.add(f"{name}: fit", False, fit.get("reason", "unusable"))
        return
    ledger.add(
        f"{name}: J{joint} sweep resolves the phase",
        fit["range_rad"] >= 0.5,
        f"{fit['range_rad']:.2f} rad over {fit['n']} samples (need ≥ 0.5 rad)",
    )
    if not fit["signal_above_friction"]:
        ledger.add(
            f"{name}: J{joint} gravity signal above friction",
            False,
            f"amplitude {fit['measured_amplitude_nm']:.3f} Nm vs friction "
            f"{fit['friction_nm']:.3f} Nm — below the floor, nothing to tune",
            required=False,
        )
        return
    ledger.add(
        f"{name}: J{joint} model scale k ≈ 1",
        abs(fit["k"] - 1.0) < SCALE_TOL,
        f"k {fit['k']:.3f} (measured {fit['measured_amplitude_nm']:.3f} / model "
        f"{fit['model_amplitude_nm']:.3f} Nm)",
    )
    ledger.add(
        f"{name}: J{joint} zero offset ≈ 0",
        abs(fit["offset_rad"]) < OFFSET_TOL_RAD,
        f"{fit['offset_rad']:+.3f} rad",
    )
    ledger.add(
        f"{name}: J{joint} bias below friction",
        abs(fit["bias_nm"]) < max(fit["friction_nm"], 0.05),
        f"bias {fit['bias_nm']:+.3f} Nm vs friction {fit['friction_nm']:.3f} Nm, "
        f"rms {fit['rms_measured_nm']:.3f} Nm",
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("files", nargs="+", help="pdsweep_*.json (globs ok)")
    parser.add_argument(
        "--qmin",
        type=float,
        default=None,
        help="drop samples with the swept joint below this [rad] (contact region)",
    )
    parser.add_argument("--json", action="store_true", help="emit the ledger as JSON")
    args = parse_or_exit(parser, argv)
    ledger = Ledger("gravity model fit")
    files = [Path(f) for pat in args.files for f in sorted(glob.glob(pat))]
    if not files:
        ledger.add("sweep files", False, "no files matched")
        return ledger.finish(args.json)

    print(
        f"{'file':40s} {'n':>5s} {'range':>6s} {'k':>6s} {'off':>7s} {'c':>7s} {'fric':>6s} {'rms':>6s}"
    )
    for path in files:
        record = json.loads(path.read_text())
        fit = analyse_sweep(record, args.qmin)
        if fit.get("usable"):
            print(
                f"{path.name:40s} {fit['n']:5d} {fit['range_rad']:6.2f} {fit['k']:6.3f} "
                f"{fit['offset_rad']:+7.3f} {fit['bias_nm']:+7.3f} {fit['friction_nm']:6.3f} "
                f"{fit['rms_measured_nm']:6.3f}"
            )
        else:
            print(f"{path.name:40s} skipped: {fit.get('reason')}")
        judge(ledger, path.stem, int(record["joint"]), fit)
        write_json(
            path.with_name(path.stem + "_fit.json"), {"source": path.name, **fit}
        )
    return ledger.finish(args.json)


if __name__ == "__main__":
    run_main(main)
