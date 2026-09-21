#!/usr/bin/env python3
"""Repeat a single-joint par6-selfcal gain search and tabulate what it accepts.

Each run homes the arm with gain changes permitted on one joint. By default
every run starts from the vendor gains (--from-vendor); --start-scales runs
from the configuration's gains multiplied by each listed factor instead, so
the spread across different starting points is visible. The scaled config is
a temporary file beside the original, so relative gripper and asset paths
still resolve.

    scripts/selfcal_repeat.py --joint 4 --runs 3 --trials 24 --sudo
    scripts/selfcal_repeat.py --joint 4 --runs 4 --start-scales 0.67,1,1.5 --sudo
    scripts/selfcal_repeat.py --joint 5 --runs 2 --trials 6 --sim
"""

import argparse
import os
import re
import subprocess
import sys
import tomllib
from pathlib import Path


def scaled_config(source: Path, joint: int, scale: float) -> Path:
    text = source.read_text()
    name = re.search(rf'^name = "joint{joint}"\s*$', text, re.MULTILINE)
    if name is None:
        raise SystemExit(f"joint{joint} not found in {source}")
    gains = text.index("[joints.gains]", name.end())
    end = re.compile(r"^\[", re.MULTILINE).search(text, gains + 1)
    block = text[gains : end.start() if end else len(text)]
    for key in ("kpv", "kiv"):
        match = re.search(rf"^{key} = ([0-9.eE+-]+)", block, re.MULTILINE)
        if match is None:
            raise SystemExit(f"joint{joint} {key} not found")
        block = block.replace(
            match.group(0), f"{key} = {float(match.group(1)) * scale!r}"
        )
    temp = source.with_name(f".{source.stem}.selfcal-repeat-{os.getpid()}-{scale}.toml")
    temp.write_text(text[:gains] + block + (text[end.start() :] if end else ""))
    return temp


def run_once(args, config: Path, extra: list[str]) -> dict:
    command = [
        *(["sudo"] if args.sudo else []),
        str(args.binary),
        str(config),
        "--joint",
        str(args.joint),
        "--trials",
        str(args.trials),
        "--output-dir",
        str(args.output_dir),
        *extra,
        *args.selfcal_args,
    ]
    print("$", " ".join(command), flush=True)
    completed = subprocess.run(command, capture_output=True, text=True)
    directory = re.search(r"^RUN_DIRECTORY: (.+)$", completed.stdout, re.MULTILINE)
    if directory is None:
        raise SystemExit(
            f"no run directory in output:\n{completed.stdout[-2000:]}\n{completed.stderr}"
        )
    run = Path(directory.group(1))
    result = (run / "result.txt").read_text().strip()
    start = tomllib.loads((run / "starting-config.toml").read_text())["joints"][
        args.joint - 1
    ]["gains"]
    accepted_file = run / "accepted-gains.toml"
    accepted = (
        tomllib.loads(accepted_file.read_text()).get(f"joint{args.joint}")
        if accepted_file.exists()
        else None
    )
    trials = sum(
        1
        for line in (run / "console.log").read_text().splitlines()
        if f"J{args.joint} trial=" in line
    )
    return {
        "run": run.name,
        "result": result,
        "start": start,
        "accepted": accepted,
        "trials": trials,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--joint", type=int, required=True, choices=range(1, 7))
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--trials", type=int, default=24)
    parser.add_argument("--config", type=Path, default=Path("config/PAR6.toml"))
    parser.add_argument(
        "--binary", type=Path, default=Path("target/release/par6-selfcal")
    )
    parser.add_argument("--output-dir", type=Path, default=Path("calibration-runs"))
    parser.add_argument(
        "--start-scales",
        help="comma-separated factors applied to the configuration's kpv/kiv, cycled per run",
    )
    parser.add_argument("--sim", action="store_true")
    parser.add_argument(
        "--sudo",
        action="store_true",
        help="run the binary under sudo (hardware needs CAP_SYS_NICE)",
    )
    parser.add_argument(
        "selfcal_args", nargs="*", help="further par6-selfcal options, after --"
    )
    args = parser.parse_args()

    scales = (
        [float(s) for s in args.start_scales.split(",")] if args.start_scales else None
    )
    rows = []
    for index in range(args.runs):
        extra = ["--sim"] if args.sim else []
        config = args.config
        temp = None
        if scales is None:
            extra.append("--from-vendor")
        else:
            temp = scaled_config(args.config, args.joint, scales[index % len(scales)])
            config = temp
        try:
            row = run_once(args, config, extra)
        finally:
            if temp is not None:
                temp.unlink(missing_ok=True)
        row["start_scale"] = None if scales is None else scales[index % len(scales)]
        rows.append(row)
        print(
            f"  {row['run']}: {row['result']} trials={row['trials']} accepted={row['accepted']}",
            flush=True,
        )

    print()
    print(f"J{args.joint}: {len(rows)} runs")
    print(
        f"{'run':40} {'start':>8} {'trials':>6} {'kpv':>12} {'kiv':>12} {'kpp':>8}  result"
    )
    for row in rows:
        gains = row["accepted"] or {}
        start = "vendor" if row["start_scale"] is None else f"x{row['start_scale']:g}"
        print(
            f"{row['run']:40} {start:>8} {row['trials']:>6} "
            f"{gains.get('kpv', float('nan')):>12.6g} {gains.get('kiv', float('nan')):>12.6g} "
            f"{gains.get('kpp', float('nan')):>8.4g}  {row['result'][:60]}"
        )
    accepted = [row["accepted"] for row in rows if row["accepted"]]
    if len(accepted) >= 2:
        print()
        for key in ("kpv", "kiv", "kpp"):
            values = [g[key] for g in accepted]
            low, high = min(values), max(values)
            print(f"{key}: min {low:.6g} max {high:.6g} max/min {high / low:.3f}")
    else:
        print("\nfewer than two runs accepted gains; no spread to report")
    if any(row["accepted"] is None for row in rows):
        print(
            "runs without accepted gains never needed tuning or failed before accepting any"
        )
    sys.exit(0 if all(row["result"] == "Ok(())" for row in rows) else 1)


if __name__ == "__main__":
    main()
