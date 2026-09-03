"""Shared plumbing for the bring-up kit.

Every script connects to a RUNNING runtime (``par6d`` on hardware, or
``par6d --sim`` on a bench) through the shipped client, takes every
kinematic quantity from ``par6._par6``, and gates anything that moves
the arm behind ``--go``. Results are printed as a ledger of named
checks, each ``required`` or ``advisory``; the exit code is 1 when a
required check failed.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np

from par6 import config as par6_config
from par6.client import RobotClient
from par6.protocol.wire import StatusBuffer
from par6.telemetry import TelemetryReader

DEG = 180.0 / math.pi


# ---------------------------------------------------------------- results


@dataclass
class Check:
    name: str
    ok: bool
    detail: str
    required: bool = True


@dataclass
class Ledger:
    """The checks a script ran, in order."""

    title: str
    checks: list[Check] = field(default_factory=list)

    def add(self, name: str, ok: bool, detail: str, *, required: bool = True) -> bool:
        ok = bool(ok)
        self.checks.append(Check(name, ok, detail, required))
        flag = "PASS" if ok else ("FAIL" if required else "WARN")
        print(f"[{flag}] {name}: {detail}", flush=True)
        return ok

    def note(self, text: str) -> None:
        print(f"       {text}", flush=True)

    @property
    def failed(self) -> bool:
        return any(not c.ok and c.required for c in self.checks)

    def finish(self, as_json: bool) -> int:
        if as_json:
            print(
                json.dumps(
                    {
                        "title": self.title,
                        "checks": [c.__dict__ for c in self.checks],
                        "failed": self.failed,
                    }
                )
            )
        else:
            verdict = "FAILED" if self.failed else "passed"
            print(f"== {self.title}: {verdict} ({len(self.checks)} checks)")
        return 1 if self.failed else 0


# ---------------------------------------------------------------- arguments


def add_connection_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--host",
        default=os.environ.get("PAR6_HOST", "127.0.0.1"),
        help="runtime address",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("PAR6_COMMAND_PORT", "6001")),
        help="command port",
    )
    parser.add_argument(
        "--telemetry-port",
        type=int,
        default=int(os.environ.get("PAR6_TELEMETRY_PORT", "6003")),
        help="telemetry stream port (the runtime's protocol.telemetry_port)",
    )
    parser.add_argument(
        "--status-port",
        type=int,
        default=int(os.environ.get("PAR6_STATUS_PORT", "6002")),
        help="STATUS broadcast port (the runtime's protocol.status_port)",
    )
    parser.add_argument(
        "--status-transport",
        default=os.environ.get("PAR6_STATUS_TRANSPORT"),
        help="AUTO | MULTICAST | UNICAST (default: the client's own choice)",
    )
    parser.add_argument(
        "--timeout", type=float, default=5.0, help="per-request timeout [s]"
    )
    parser.add_argument("--json", action="store_true", help="emit the ledger as JSON")


def add_go_arg(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--go",
        action="store_true",
        help="actually move the arm; without it every motion step is described and skipped",
    )


def connect(args: argparse.Namespace) -> RobotClient:
    kwargs: dict[str, Any] = {"status_port": args.status_port}
    if args.status_transport:
        kwargs["status_transport"] = args.status_transport.upper()
        kwargs["status_unicast_host"] = args.host
    return RobotClient(
        host=args.host, port=args.port, timeout=args.timeout, retries=2, **kwargs
    )


def gate(args: argparse.Namespace, ledger: Ledger, what: str) -> bool:
    """False (and a note) when ``--go`` was not given."""
    if args.go:
        return True
    ledger.note(f"would {what} — pass --go to do it")
    return False


# ---------------------------------------------------------------- config


def robot_config() -> dict:
    return par6_config.load_robot_config()


def canonical_pose_deg(config: dict | None = None) -> list[float]:
    """The one pose every motion test starts from: the config's park
    pose, which is where homing leaves the arm."""
    cfg = config or robot_config()
    return [float(v) * DEG for v in cfg["robot"]["park_pose_rad"]]


def soft_limits_deg(config: dict | None = None) -> np.ndarray:
    return par6_config.soft_limits_rad(config or robot_config()) * DEG


def mode_velocity_deg_s(mode: str, config: dict | None = None) -> np.ndarray:
    cfg = config or robot_config()
    return np.array(
        [
            par6_config.resolve_mode_limits(j["limits"], mode)[0] * DEG
            for j in cfg["joints"]
        ]
    )


def tick_dt_s(config: dict | None = None) -> float:
    return float((config or robot_config())["robot"]["tick_dt_s"])


# ---------------------------------------------------------------- the arm


def require_ready(client: RobotClient, ledger: Ledger) -> bool:
    """A runtime that answers, is homed and can be enabled. Homing moves
    the arm, so it is never done here: run ``par6 home --wait`` first."""
    if client.ping() is None:
        return ledger.add("runtime answers", False, "no runtime at the address")
    ledger.add("runtime answers", True, f"{client.host}:{client.port}")
    client.reset()
    ok = client.wait_status(lambda s: s.enabled and s.homed, timeout=5.0)
    return ledger.add(
        "homed and enabled",
        ok,
        "ready" if ok else "not homed/enabled — run `par6 home --wait` first",
    )


def collect_status(client: RobotClient, seconds: float) -> list[StatusBuffer]:
    """Every STATUS broadcast for `seconds`."""
    out: list[StatusBuffer] = []
    deadline = time.monotonic() + seconds

    def keep(s: StatusBuffer) -> bool:
        out.append(
            StatusBuffer(
                angles=np.array(s.angles, dtype=np.float64),
                speeds=np.array(s.speeds, dtype=np.float64),
                torques=np.array(s.torques, dtype=np.float64),
                mode=s.mode,
                enabled=s.enabled,
                homed=s.homed,
                gravity_comp=s.gravity_comp,
                error=s.error,
                mono_time_ns=s.mono_time_ns,
            )
        )
        return time.monotonic() >= deadline

    client.wait_status(keep, timeout=seconds + 2.0)
    return out


def go_to(
    client: RobotClient, pose_deg: list[float], ledger: Ledger, speed: float = 0.2
) -> bool:
    """Move to `pose_deg` and verify the arm got there."""
    index = client.move_j(pose_deg, speed=speed, wait=True, timeout=60.0)
    if index < 0:
        return ledger.add("reach the canonical pose", False, "move_j not acknowledged")
    angles = client.angles()
    err = (
        float(np.max(np.abs(np.array(angles) - np.array(pose_deg))))
        if angles
        else math.inf
    )
    return ledger.add(
        "reach the canonical pose", err < 0.5, f"max error {err:.3f} deg", required=True
    )


def telemetry_fields(
    client: RobotClient, port: int, recipe: str, seconds: float, host: str = "127.0.0.1"
) -> list[dict[str, Any]]:
    """Select `recipe` and collect its frames' field dicts for `seconds`."""
    client.set_recipe(recipe)
    frames: list[dict[str, Any]] = []
    with TelemetryReader(port, host=host) as reader:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            f = reader.recv(timeout=0.5)
            if f is not None and f.get("recipe") == recipe:
                frames.append(f["fields"])
    return frames


def stream_at(
    client: RobotClient,
    rate_hz: float,
    seconds: float,
    target_deg_at: Callable[[float], list[float]],
    *,
    speed: float = 1.0,
) -> list[tuple[float, list[float]]]:
    """Stream `servo_j` targets at `rate_hz` for `seconds`; the
    commanded (t, target) log."""
    period = 1.0 / rate_hz
    t0 = time.monotonic()
    sent: list[tuple[float, list[float]]] = []
    next_at = t0
    while True:
        now = time.monotonic()
        t = now - t0
        if t > seconds:
            break
        target = target_deg_at(min(t, seconds))
        client.servo_j(target, speed=speed)
        sent.append((t, target))
        next_at += period
        sleep = next_at - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
    return sent


def raised_cosine(
    q0: float, amplitude: float, period: float
) -> Callable[[float], float]:
    """`q(t) = q0 + A·(1 − cos 2πt/T)/2`: offset AND velocity are zero at
    t = 0 and t = T, so the stream starts and ends at rest."""

    def q(t: float) -> float:
        return q0 + amplitude * (1.0 - math.cos(2.0 * math.pi * t / period)) / 2.0

    return q


def raised_cosine_peak_velocity(amplitude: float, period: float) -> float:
    """`max |dq/dt| = A·π/T`."""
    return abs(amplitude) * math.pi / period


def tracking_gap_deg(
    sent: list[tuple[float, list[float]]],
    seen: list[StatusBuffer],
    t_status0_ns: int,
    joint: int,
    latency_s: float,
) -> float:
    """Max |measured − commanded(t − latency)| over the window, ignoring
    the first and last 10 % where the endpoints dominate."""
    if not sent or not seen:
        return math.inf
    ts = np.array([t for t, _ in sent])
    qs = np.array([q[joint] for _, q in sent])
    gaps = []
    for s in seen:
        t = (s.mono_time_ns - t_status0_ns) * 1e-9 - latency_s
        if t < 0.1 * ts[-1] or t > 0.9 * ts[-1]:
            continue
        gaps.append(abs(float(np.interp(t, ts, qs)) - float(s.angles[joint])))
    return max(gaps) if gaps else math.inf


def fail_if_outside(
    ledger: Ledger,
    joint: int,
    lo: float,
    hi: float,
    name: str,
    config: dict | None = None,
) -> bool:
    """Refuse a planned excursion `[lo, hi]` [deg] that leaves the joint's
    soft window — with the numbers, before anything moves."""
    limits = soft_limits_deg(config)
    inside = bool(limits[joint, 0] <= lo and hi <= limits[joint, 1])
    return ledger.add(
        name,
        inside,
        f"J{joint} excursion [{lo:.2f}, {hi:.2f}] deg vs soft [{limits[joint, 0]:.2f}, "
        f"{limits[joint, 1]:.2f}] deg",
    )


def parse_or_exit(
    parser: argparse.ArgumentParser, argv: list[str] | None
) -> argparse.Namespace:
    try:
        return parser.parse_args(argv)
    except SystemExit as exc:
        raise SystemExit(int(exc.code or 0)) from exc


def run_main(fn: Callable[[list[str] | None], int]) -> None:
    raise SystemExit(fn(sys.argv[1:]))
