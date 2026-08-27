"""``par6`` — a shell view of a running runtime.

Everything here is a thin argparse shell over :class:`par6.client.RobotClient`,
which already carries every call these subcommands make. The point is that
reading a live arm's state should not require writing a Python script, and
that ``par6 estop`` should be reachable from a terminal in a hurry.

Connection defaults follow the client's own: ``PAR6_HOST`` (127.0.0.1) and
``PAR6_COMMAND_PORT`` (6001), each overridable per invocation.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from collections.abc import Sequence
from typing import Any

from par6.client import RobotClient

#: Exit code when no runtime answers the target address.
EXIT_UNREACHABLE = 2
#: Exit code when the runtime answered but refused the command.
EXIT_REFUSED = 3


def _client(args: argparse.Namespace) -> RobotClient:
    return RobotClient(host=args.host, port=args.port, timeout=args.timeout)


def _emit(value: Any, as_json: bool) -> None:
    if as_json:
        print(json.dumps(value, default=str))
    elif isinstance(value, list):
        print(" ".join(f"{v:.4f}" if isinstance(v, float) else str(v) for v in value))
    elif isinstance(value, dict):
        for key, val in value.items():
            print(f"{key}: {val}")
    else:
        print(value)


def _cmd_ping(client: RobotClient, args: argparse.Namespace) -> int:
    result = client.ping()
    if result is None:
        print(f"no runtime answered {args.host}:{args.port}", file=sys.stderr)
        return EXIT_UNREACHABLE
    _emit({"hardware_connected": result.hardware_connected}, args.json)
    return 0


def _cmd_angles(client: RobotClient, args: argparse.Namespace) -> int:
    angles = client.angles()
    if angles is None:
        print("no angles: the runtime did not answer", file=sys.stderr)
        return EXIT_UNREACHABLE
    _emit(list(angles), args.json)
    return 0


def _cmd_pose(client: RobotClient, args: argparse.Namespace) -> int:
    pose = client.pose(frame=args.frame)
    if pose is None:
        print("no pose: the runtime did not answer", file=sys.stderr)
        return EXIT_UNREACHABLE
    _emit(list(pose), args.json)
    return 0


def _cmd_status(client: RobotClient, args: argparse.Namespace) -> int:
    status = client.status()
    if status is None:
        print("no status: the runtime did not answer", file=sys.stderr)
        return EXIT_UNREACHABLE
    tool = status.tool_status
    _emit(
        {
            "angles_deg": [round(v, 4) for v in status.angles],
            "speeds_rad_s": [round(v, 4) for v in status.speeds],
            # The e-stop is always the LAST slot; the ones before it are the
            # configured inputs then outputs, so the width follows config.
            "io": list(status.io),
            "estop": bool(status.io[-1]) if status.io else None,
            "tool": None if tool is None else tool.key,
            "tool_positions": None if tool is None else list(tool.positions),
            "tool_engaged": None if tool is None else tool.engaged,
        },
        args.json,
    )
    return 0


def _cmd_estop(client: RobotClient, args: argparse.Namespace) -> int:
    client.estop()
    _emit("estop latched; clear it with `par6 reset`", args.json)
    return 0


def _cmd_reset(client: RobotClient, args: argparse.Namespace) -> int:
    client.reset()
    _emit("protective stop cleared", args.json)
    return 0


def _cmd_stop(client: RobotClient, args: argparse.Namespace) -> int:
    client.stop(clear_queue=not args.keep_queue)
    _emit("motion stopped", args.json)
    return 0


def _cmd_home(client: RobotClient, args: argparse.Namespace) -> int:
    client.home(wait=args.wait, timeout=args.home_timeout)
    _emit("homed" if args.wait else "homing started", args.json)
    return 0


def _cmd_move_j(client: RobotClient, args: argparse.Namespace) -> int:
    index = client.move_j(
        args.angles,
        speed=args.speed,
        wait=args.wait,
        timeout=args.move_timeout,
    )
    _emit({"queue_index": index}, args.json)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="par6", description=__doc__.splitlines()[0])
    parser.add_argument(
        "--host",
        default=os.environ.get("PAR6_HOST", "127.0.0.1"),
        help="runtime address (default: $PAR6_HOST or 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("PAR6_COMMAND_PORT", "6001")),
        help="command port (default: $PAR6_COMMAND_PORT or 6001)",
    )
    parser.add_argument(
        "--timeout", type=float, default=2.0, help="per-request timeout [s]"
    )
    parser.add_argument(
        "--json", action="store_true", help="emit JSON instead of plain text"
    )

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("ping", help="check that a runtime is answering").set_defaults(
        fn=_cmd_ping
    )
    sub.add_parser("angles", help="joint angles [deg]").set_defaults(fn=_cmd_angles)

    pose = sub.add_parser("pose", help="TCP pose [mm, deg]")
    pose.add_argument("--frame", choices=("WRF", "TRF"), default="WRF")
    pose.set_defaults(fn=_cmd_pose)

    sub.add_parser("status", help="a summary of controller state").set_defaults(
        fn=_cmd_status
    )
    sub.add_parser(
        "estop", help="protective stop: hold position, latch disabled"
    ).set_defaults(fn=_cmd_estop)
    sub.add_parser("reset", help="clear a latched protective stop").set_defaults(
        fn=_cmd_reset
    )

    stop = sub.add_parser("stop", help="stop motion")
    stop.add_argument(
        "--keep-queue", action="store_true", help="leave the queue in place"
    )
    stop.set_defaults(fn=_cmd_stop)

    home = sub.add_parser("home", help="run the homing sequence")
    home.add_argument("--wait", action="store_true", help="block until homed")
    home.add_argument("--home-timeout", type=float, default=120.0)
    home.set_defaults(fn=_cmd_home)

    move = sub.add_parser("move-j", help="joint move to six angles [deg]")
    move.add_argument("angles", type=float, nargs=6, metavar="DEG")
    move.add_argument(
        "--speed", type=float, default=0.2, help="velocity fraction (0, 1]"
    )
    move.add_argument(
        "--wait", action="store_true", help="block until the move completes"
    )
    move.add_argument("--move-timeout", type=float, default=120.0)
    move.set_defaults(fn=_cmd_move_j)

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        with _client(args) as client:
            return int(args.fn(client, args))
    except OSError as exc:
        print(f"cannot reach {args.host}:{args.port}: {exc}", file=sys.stderr)
        return EXIT_UNREACHABLE
    except RuntimeError as exc:
        print(f"{args.command} refused: {exc}", file=sys.stderr)
        return EXIT_REFUSED


if __name__ == "__main__":
    raise SystemExit(main())
