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
import sys
from collections.abc import Sequence
from typing import Any

from par6.client import RobotClient, RobotError
from par6.firmware.releases import PRODUCTS as _FLASH_PRODUCTS

#: Exit code when no runtime answers the target address.
EXIT_UNREACHABLE = 2
#: Exit code when the runtime answered but refused the command.
EXIT_REFUSED = 3
#: Exit code when the runtime accepted the command but it did not finish
#: within the wait.
EXIT_TIMEOUT = 4
#: Exit code when a firmware image could not be obtained or trusted.
EXIT_SOURCE = 5
#: Exit code when the flash itself failed. Distinct from EXIT_SOURCE
#: because only this one may have left a drive holding a partial image.
EXIT_FLASH = 6


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
        print(f"no runtime answered {client.host}:{client.port}", file=sys.stderr)
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
            # The slot carries the LINE, which reads low while pressed.
            "estop": (status.io[-1] == 0) if status.io else None,
            "tool": None if tool is None else tool.key,
            "tool_positions": None if tool is None else list(tool.positions),
            "tool_engaged": None if tool is None else tool.engaged,
        },
        args.json,
    )
    return 0


def _unconfirmed(what: str, client: RobotClient) -> int:
    """A send nothing acknowledged must never read as success — for
    ``estop`` especially, "the arm is stopped" printed on a lost datagram
    is the dangerous lie."""
    print(
        f"{what} NOT confirmed: no runtime acknowledged it at "
        f"{client.host}:{client.port}",
        file=sys.stderr,
    )
    return EXIT_UNREACHABLE


def _cmd_estop(client: RobotClient, args: argparse.Namespace) -> int:
    if client.estop() != 1:
        return _unconfirmed("estop", client)
    _emit("estop latched; clear it with `par6 reset`", args.json)
    return 0


def _cmd_reset(client: RobotClient, args: argparse.Namespace) -> int:
    if client.reset() != 1:
        return _unconfirmed("reset", client)
    _emit("protective stop cleared", args.json)
    return 0


def _cmd_stop(client: RobotClient, args: argparse.Namespace) -> int:
    if client.stop(clear_queue=not args.keep_queue) != 1:
        return _unconfirmed("stop", client)
    _emit("motion stopped", args.json)
    return 0


def _cmd_home(client: RobotClient, args: argparse.Namespace) -> int:
    if client.home(wait=args.wait, timeout=args.home_timeout) < 0:
        return _unconfirmed("home", client)
    _emit("homed" if args.wait else "homing started", args.json)
    return 0


def _cmd_move_j(client: RobotClient, args: argparse.Namespace) -> int:
    index = client.move_j(
        args.angles,
        speed=args.speed,
        wait=args.wait,
        timeout=args.move_timeout,
    )
    if index < 0:
        return _unconfirmed("move-j", client)
    _emit({"queue_index": index}, args.json)
    return 0


def _cmd_scan(client: RobotClient, args: argparse.Namespace) -> int:
    rows = client.bus_scan()
    if rows is None:
        return _unconfirmed("scan", client)
    if args.json:
        _emit(rows, True)
        return 0
    fresh = {0: "unknown", 1: "fresh", 2: "stale", 3: "lost"}
    print("node  configured  present  freshness  hw  sw  serial")
    for r in rows:
        if not (r["present"] or r["configured"] or args.all):
            continue
        print(
            f"{r['node']:>4}  {'yes' if r['configured'] else 'no':>10}  "
            f"{'yes' if r['present'] else 'no':>7}  {fresh.get(r['freshness'], '?'):>9}  "
            f"{r['hw_ver']:>2}  {r['sw_ver']:>2}  {r['serial']}"
        )
    return 0


def _cmd_set_can_id(client: RobotClient, args: argparse.Namespace) -> int:
    if client.set_can_id(args.node, args.new_id, force=args.force) != 1:
        return _unconfirmed("set-can-id", client)
    _emit(
        f"node {args.node} told to answer as {args.new_id}; run `par6 save-config "
        f"{args.new_id} --force` to keep it, then update the config and restart",
        args.json,
    )
    return 0


def _cmd_save_config(client: RobotClient, args: argparse.Namespace) -> int:
    if client.save_config(args.node, force=args.force) != 1:
        return _unconfirmed("save-config", client)
    _emit(f"node {args.node} asked to save its configuration", args.json)
    return 0


def _cmd_set_pid_gains(client: RobotClient, args: argparse.Namespace) -> int:
    if (
        client.set_pid_gains(
            args.node,
            kpp=args.kpp,
            kpv=args.kpv,
            kiv=args.kiv,
            kpiq=args.kpiq,
            kiiq=args.kiiq,
            kp=args.kp,
            kd=args.kd,
            ilim_ma=args.ilim_ma,
            velocity_limit_ticks_s=args.velocity_limit_ticks_s,
            voltage_limit_mv=args.voltage_limit_mv,
        )
        != 1
    ):
        return _unconfirmed("set-pid-gains", client)
    _emit(f"node {args.node} retuned", args.json)
    return 0


def _cmd_tool(client: RobotClient, args: argparse.Namespace) -> int:
    index = client.tool_action(
        args.tool, args.action, args.params, wait=args.wait, timeout=args.tool_timeout
    )
    if index < 0:
        return _unconfirmed("tool", client)
    _emit(
        f"{args.tool} {args.action} "
        + ("done" if args.wait else f"queued as #{index}"),
        args.json,
    )
    return 0


def _cmd_flashing(client: RobotClient, args: argparse.Namespace) -> int:
    if args.direction == "enter":
        ok = client.enter_flashing(args.assertion) == 1
        text = "FLASHING: bus silent, hand it to the flasher"
    else:
        ok = client.exit_flashing() == 1
        text = "left FLASHING: bus awake, config re-pushed"
    if not ok:
        return _unconfirmed(f"flashing {args.direction}", client)
    _emit(text, args.json)
    return 0


def _cmd_flash(client: RobotClient, args: argparse.Namespace) -> int:
    """Flash one drive, holding the bus for exactly as long as it takes.

    Everything that can refuse the image does so before the runtime is
    asked to go quiet, so a bad download costs nothing but the download.
    """
    from par6.firmware import releases
    from par6.firmware.flasher import BootloaderError, flash_image
    from par6.firmware.session import FlashBusy, flash_lock, granted_bus

    def log(line: str) -> None:
        if not args.json:
            print(line, file=sys.stderr)

    try:
        if args.file:
            image = releases.load_file(args.file)
        else:
            image = releases.fetch_release(
                args.product, args.tag, refresh=args.refresh, on_log=log
            )
    except releases.FirmwareFetchError as err:
        print(f"flash: {err}", file=sys.stderr)
        return EXIT_SOURCE
    if not image.checksum_verified:
        log("integrity unverified: nothing vouches for these bytes.")

    if args.dry_run:
        _emit(
            {
                "product": image.product,
                "tag": image.tag,
                "path": str(image.path),
                "bytes": len(image.data),
                "sha256": image.sha256,
                "checksum_verified": image.checksum_verified,
            },
            args.json,
        )
        return 0

    try:
        with flash_lock(), granted_bus(client, args.assertion, channel=args.can) as bus:
            report = flash_image(
                bus,
                args.node,
                image.data,
                erase=not args.no_erase,
                reset_stalled_app=not args.no_reset,
                on_log=log,
            )
    except FlashBusy as err:
        print(f"flash: {err}", file=sys.stderr)
        return EXIT_REFUSED
    except ImportError:
        print(
            "flash: python-can is not installed. Install par6 with the "
            "'flash' extra on the machine holding the CAN interface.",
            file=sys.stderr,
        )
        return EXIT_SOURCE
    except (BootloaderError, ValueError) as err:
        print(f"flash: {err}", file=sys.stderr)
        return EXIT_FLASH

    _emit(
        {
            "node": report.board_id,
            "tag": image.tag,
            "pages": report.pages,
            "app_crc": f"0x{report.app_crc:08X}",
            "elapsed_s": round(report.elapsed_s, 1),
            "page_retries": report.stats.page_retries,
            "chunk_retries": report.stats.chunk_retries,
        }
        if args.json
        else report.summary(),
        args.json,
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="par6", description=__doc__.splitlines()[0])
    # Unset here means the client's own resolution: $PAR6_HOST /
    # $PAR6_COMMAND_PORT, then the shipped defaults.
    parser.add_argument(
        "--host",
        default=None,
        help="runtime address (default: $PAR6_HOST or 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=None,
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

    scan = sub.add_parser("scan", help="rescan the CAN bus and list every node id")
    scan.add_argument(
        "--all", action="store_true", help="show absent, unconfigured ids too"
    )
    scan.set_defaults(fn=_cmd_scan)

    setid = sub.add_parser(
        "set-can-id", help="commissioning: rename a drive (idle arm only)"
    )
    setid.add_argument("node", type=int, help="the drive's current id (0-15)")
    setid.add_argument("new_id", type=int, help="the id it should answer to (0-15)")
    setid.add_argument(
        "--force", action="store_true", help="address an id the config does not list"
    )
    setid.set_defaults(fn=_cmd_set_can_id)

    save = sub.add_parser(
        "save-config", help="commissioning: persist a drive's configuration to NVM"
    )
    save.add_argument("node", type=int, help="target id (0-15)")
    save.add_argument(
        "--force", action="store_true", help="address an id the config does not list"
    )
    save.set_defaults(fn=_cmd_save_config)

    gains = sub.add_parser(
        "set-pid-gains", help="push one drive's tuning live (every gain required)"
    )
    gains.add_argument("node", type=int, help="configured drive id")
    for name in ("kpp", "kpv", "kiv", "kpiq", "kiiq", "kp", "kd", "ilim-ma"):
        gains.add_argument(f"--{name}", type=float, required=True)
    gains.add_argument("--velocity-limit-ticks-s", type=float, required=True)
    gains.add_argument("--voltage-limit-mv", type=int, default=0, help="0 = VBUS")
    gains.set_defaults(fn=_cmd_set_pid_gains)

    tool = sub.add_parser(
        "tool", help="run a tool action, e.g. `tool ELECTRIC close 50`"
    )
    tool.add_argument("tool", help="tool key, e.g. ELECTRIC")
    tool.add_argument(
        "action", help="action name, e.g. open / close / calibrate / stop"
    )
    tool.add_argument("params", type=float, nargs="*", help="numeric parameters")
    tool.add_argument(
        "--no-wait", dest="wait", action="store_false", help="do not block"
    )
    tool.add_argument("--tool-timeout", type=float, default=10.0)
    tool.set_defaults(fn=_cmd_tool)

    flashing = sub.add_parser(
        "flashing", help="hand the bus to a firmware flasher, or take it back"
    )
    flashing.add_argument("direction", choices=("enter", "exit"))
    flashing.add_argument(
        "--assertion",
        choices=("parked", "force"),
        default="parked",
        help="your vouching on enter: the arm is parked, or force regardless",
    )
    flashing.set_defaults(fn=_cmd_flashing)

    flash = sub.add_parser(
        "flash",
        help="update one drive's firmware over CAN",
        description=(
            "Takes the bus from the runtime, writes the image, and gives the "
            "bus back. The drive validates what it received and reboots on "
            "its own once the bus falls silent."
        ),
    )
    flash.add_argument("--node", type=int, required=True, help="drive CAN id")
    flash.add_argument(
        "--product",
        choices=sorted(_FLASH_PRODUCTS),
        default="stepfoc",
        help="which drive's firmware repository to pull from",
    )
    flash.add_argument(
        "--tag", default=None, help="release tag (default: the latest release)"
    )
    flash.add_argument(
        "--file", default=None, help="flash this .bin instead of a release"
    )
    flash.add_argument(
        "--refresh", action="store_true", help="re-download even if cached"
    )
    flash.add_argument(
        "--dry-run",
        action="store_true",
        help="fetch and check the image, then stop without touching the bus",
    )
    flash.add_argument(
        "--assertion",
        choices=("parked", "force"),
        default="parked",
        help="your vouching for taking the bus: the arm is parked, or force",
    )
    flash.add_argument(
        "--can",
        default=None,
        help="SocketCAN interface (default: the one the runtime names)",
    )
    flash.add_argument(
        "--no-erase",
        action="store_true",
        help="skip the erase, for resuming a run that already erased",
    )
    flash.add_argument(
        "--no-reset",
        action="store_true",
        help=(
            "do not reset a running application into its bootloader; use when "
            "catching a power-cycled board's startup window"
        ),
    )
    flash.set_defaults(fn=_cmd_flash)

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    client = _client(args)
    try:
        with client:
            return int(args.fn(client, args))
    except TimeoutError as exc:
        # An OSError subclass, and the one that means the runtime
        # answered every datagram but the command is still running.
        print(f"{args.command} timed out: {exc}", file=sys.stderr)
        return EXIT_TIMEOUT
    except OSError as exc:
        print(f"cannot reach {client.host}:{client.port}: {exc}", file=sys.stderr)
        return EXIT_UNREACHABLE
    except (RobotError, RuntimeError) as exc:
        print(f"{args.command} refused: {exc}", file=sys.stderr)
        return EXIT_REFUSED


if __name__ == "__main__":
    raise SystemExit(main())
