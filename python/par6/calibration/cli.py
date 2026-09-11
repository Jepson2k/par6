"""`par6-calibrate`: run one routine against the running par6d and stage a patch.

PAR6_DIAGNOSTICS=/path/capture.bin par6d ...      # the runtime records
par6-calibrate check                              # then calibrate
par6-calibrate tune-feedback --joint 3
par6-calibrate gravity
par6-calibrate gravity --verify                   # after applying the candidate
par6-calibrate limits
"""

from __future__ import annotations

import argparse
import asyncio
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

from par6 import AsyncRobotClient

from . import routines
from .session import Session

ROUTINES = ("check", "tune-feedback", "gravity", "limits")


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="par6-calibrate", description=__doc__.split("\n\n")[0]
    )
    p.add_argument("routine", choices=ROUTINES)
    p.add_argument(
        "--joint",
        type=int,
        default=3,
        help="1-based joint for tune-feedback (default 3)",
    )
    p.add_argument(
        "--verify", action="store_true", help="gravity: verify the loaded model, no fit"
    )
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("PAR6_COMMAND_PORT", "6001")),
        help="par6d command port (default PAR6_COMMAND_PORT or 6001; Commander uses 5001)",
    )
    p.add_argument(
        "--capture",
        default=os.environ.get("PAR6_DIAGNOSTICS"),
        help="the runtime's PAR6_DIAGNOSTICS recording (default: that variable)",
    )
    p.add_argument(
        "--out", default="calibration-runs", help="directory for run evidence"
    )
    return p


async def run(args) -> dict:
    if not args.capture:
        raise SystemExit(
            "A native recording is required: set PAR6_DIAGNOSTICS or --capture"
        )
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    directory = Path(args.out) / f"{stamp}-{args.routine}"
    client = AsyncRobotClient(host=args.host, port=args.port)
    try:
        async with Session(client, directory, args.capture) as session:
            if args.routine == "check":
                report = await routines.check(session)
            elif args.routine == "tune-feedback":
                report = await routines.tune_feedback(session, joint=args.joint - 1)
            elif args.routine == "gravity":
                report = await routines.gravity(session, verify_only=args.verify)
            else:
                report = await routines.limits(session)
    finally:
        await client.close()
    print(
        f"{report['kind']}: {'PASS' if report['valid'] else 'FAIL'}; evidence in {directory}"
    )
    for reason in report["reasons"]:
        print(f"  - {reason}")
    return report


def main(argv=None) -> int:
    report = asyncio.run(run(parser().parse_args(argv)))
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    sys.exit(main())
