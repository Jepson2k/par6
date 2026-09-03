"""Shared plumbing for the gravity identification harness.

Every script here talks to a RUNNING par6d through the shipped client and
takes every kinematic quantity — the gravity model G(q) above all — from
the runtime's own telemetry (``gravity_torques``, published every tick at
the measured pose), never from a client-side model. The primitives that
implement the README's safety policy live here: the position loop stays
in command (a sweep is a ``servo_j`` stream the drive's own loop follows,
a float is the runtime's gravity-only IDLE), every abort freezes with a
position hold and lowers under control — never torque-off — and velocity
aborts use finite differences of the measured position.
"""

from __future__ import annotations

import contextlib
import importlib.util
import json
import math
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator

import numpy as np

from par6 import config as par6_config
from par6.client import RobotClient
from par6.protocol.constants import ControllerMode
from par6.telemetry import TelemetryReader

KIT = Path(__file__).resolve().parents[1] / "bringup"


def _bringup_common() -> Any:
    """The bring-up kit's plumbing, loaded by path: each kit has a module
    of its own in a script directory that ends up on ``sys.path``."""
    spec = importlib.util.spec_from_file_location("bringup_common", KIT / "common.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules["bringup_common"] = module
    spec.loader.exec_module(module)
    return module


_kit = _bringup_common()
DEG = _kit.DEG
Ledger = _kit.Ledger
add_connection_args = _kit.add_connection_args
add_go_arg = _kit.add_go_arg
connect = _kit.connect
fail_if_outside = _kit.fail_if_outside
gate = _kit.gate
parse_or_exit = _kit.parse_or_exit
require_ready = _kit.require_ready
robot_config = _kit.robot_config
run_main = _kit.run_main
soft_limits_deg = _kit.soft_limits_deg

__all__ = [
    "DEG",
    "Frame",
    "Ledger",
    "RESULTS",
    "REPO",
    "SimDaemon",
    "TelemetryTap",
    "add_connection_args",
    "add_go_arg",
    "analyse_sweep",
    "connect",
    "fail_if_outside",
    "fit_sinusoid",
    "freeze_and_lower",
    "gate",
    "gravity_at",
    "joint_names",
    "mode_velocity_rad_s",
    "move_and_verify",
    "parse_or_exit",
    "parse_pre",
    "place_sim_arm",
    "require_ready",
    "robot_config",
    "run_main",
    "sim_daemon",
    "soft_limits_deg",
    "write_json",
]

HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
REPO = HERE.parents[1]


# ---------------------------------------------------------------- results


def write_json(path: Path, payload: dict[str, Any]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=1))
    print(f"WROTE {path}", flush=True)
    return path


# ---------------------------------------------------------------- config


def mode_velocity_rad_s(mode: str, config: dict | None = None) -> np.ndarray:
    cfg = config or robot_config()
    return np.array(
        [par6_config.resolve_mode_limits(j["limits"], mode)[0] for j in cfg["joints"]]
    )


def joint_names(config: dict | None = None) -> list[str]:
    return [str(j["name"]) for j in (config or robot_config())["joints"]]


def parse_pre(specs: list[str], names: list[str], active: int) -> dict[int, float]:
    """``--pre joint3=1.1`` (name or 0-based index, radians), never the
    joint under identification."""
    out: dict[int, float] = {}
    for spec in specs:
        name, _, value = spec.partition("=")
        idx = names.index(name) if name in names else int(name)
        if idx == active:
            raise SystemExit(f"--pre {spec}: J{idx} is the joint being identified")
        out[idx] = float(value)
    return out


# ---------------------------------------------------------------- telemetry


@dataclass
class Frame:
    """One ``full`` telemetry frame, the fields this harness reads."""

    t: float
    q: np.ndarray
    qd: np.ndarray
    tau: np.ndarray
    g: np.ndarray
    tau_cmd: np.ndarray


class TelemetryTap:
    """Collects ``full`` telemetry frames on a thread until closed, so a
    script can stream targets and read the runtime's measurements — and
    its G(q) at those measurements — at the same time."""

    def __init__(self, client: RobotClient, port: int, host: str = "127.0.0.1"):
        client.set_recipe("full")
        self._port = port
        self._host = host
        self._frames: list[Frame] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def __enter__(self) -> "TelemetryTap":
        self._thread.start()
        return self

    def __exit__(self, *_: object) -> bool:
        self._stop.set()
        self._thread.join(timeout=5.0)
        return False

    def _run(self) -> None:
        with TelemetryReader(self._port, host=self._host) as reader:
            while not self._stop.is_set():
                f = reader.recv(timeout=0.2)
                if f is None or f.get("recipe") != "full":
                    continue
                x = f["fields"]
                frame = Frame(
                    t=time.monotonic(),
                    q=np.asarray(x["measured_positions"], dtype=np.float64),
                    qd=np.asarray(x["measured_velocities"], dtype=np.float64),
                    tau=np.asarray(x["measured_torques"], dtype=np.float64),
                    g=np.asarray(x["gravity_torques"], dtype=np.float64),
                    tau_cmd=np.asarray(x["commanded_torques"], dtype=np.float64),
                )
                with self._lock:
                    self._frames.append(frame)

    def latest(self) -> Frame | None:
        with self._lock:
            return self._frames[-1] if self._frames else None

    def velocity(self, window_s: float = 0.2) -> np.ndarray | None:
        """Finite-difference velocity of the measured position over the
        last ``window_s``: the newest frame against the newest one at
        least that old. Consecutive frames repeat a joint's reading
        between encoder polls, so differencing neighbours reads steps as
        spikes; a window spans several polls."""
        with self._lock:
            if len(self._frames) < 2:
                return None
            new = self._frames[-1]
            old = None
            for f in reversed(self._frames):
                if new.t - f.t >= window_s:
                    old = f
                    break
            if old is None:
                return None
        return (new.q - old.q) / (new.t - old.t)

    def since(self, t: float) -> list[Frame]:
        """Every frame received after monotonic time ``t``."""
        with self._lock:
            return [f for f in self._frames if f.t > t]

    def wait_frame(self, timeout: float = 3.0) -> Frame | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            f = self.latest()
            if f is not None:
                return f
            time.sleep(0.01)
        return None

    def count(self) -> int:
        with self._lock:
            return len(self._frames)


def gravity_at(tap: TelemetryTap, q_rad: np.ndarray, timeout: float = 3.0) -> Frame:
    """The first frame measured within 2 mrad of ``q_rad`` on every joint:
    G(q) as the runtime computes it at that pose."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        f = tap.latest()
        if f is not None and float(np.max(np.abs(f.q - q_rad))) < 2e-3:
            return f
        time.sleep(0.01)
    raise RuntimeError(f"no telemetry frame at {np.round(q_rad, 3)} within {timeout}s")


# ---------------------------------------------------------------- the arm


def move_and_verify(
    client: RobotClient,
    pose_rad: np.ndarray,
    ledger: Ledger,
    name: str,
    speed: float = 0.1,
    tol_deg: float = 0.5,
) -> bool:
    """A profiled move under position control, verified on the encoders."""
    pose_deg = [float(v) * DEG for v in pose_rad]
    index = client.move_j(pose_deg, speed=speed, wait=True, timeout=180.0)
    if index < 0:
        return ledger.add(name, False, "move_j not acknowledged")
    angles = client.angles()
    err = (
        float(np.max(np.abs(np.array(angles) - np.array(pose_deg))))
        if angles
        else math.inf
    )
    return ledger.add(name, err < tol_deg, f"max error {err:.3f} deg")


def freeze_and_lower(
    client: RobotClient,
    ledger: Ledger,
    why: str,
    home_rad: np.ndarray,
    tol_deg: float = 0.5,
) -> None:
    """Every abort: freeze with a position hold — never torque-off — report,
    then lower under control. Leaving freedrive re-engages the hold;
    ``stop`` ends any stream in the pose the drives are at."""
    client.set_gravity_comp(False)
    client.stop(clear_queue=True)
    ledger.add("abort", False, f"{why} — frozen with a position hold")
    client.wait_status(lambda s: s.mode == ControllerMode.IDLE, timeout=3.0)
    ledger.note("lowering under control")
    move_and_verify(
        client,
        home_rad,
        ledger,
        "returned to the start pose",
        speed=0.1,
        tol_deg=tol_deg,
    )


# ---------------------------------------------------------------- fitting


def fit_sinusoid(
    q: np.ndarray, y: np.ndarray, direction: np.ndarray | None = None
) -> dict[str, float]:
    """``y ≈ A·sin(q + phi) + c [+ fric·direction]`` by least squares.

    With every other joint held, the gravity torque on a revolute joint
    is exactly a sinusoid in that joint's angle — the distal centre of
    mass rotates rigidly about the axis — so measured and modelled torque
    over the same sweep differ only by a scale (``A``), a phase (the
    motor-zero to URDF-zero offset) and the friction/bias terms.
    """
    cols = [np.sin(q), np.cos(q), np.ones_like(q)]
    if direction is not None:
        cols.append(direction)
    m = np.column_stack(cols)
    coef, *_ = np.linalg.lstsq(m, y, rcond=None)
    a, b, c = (float(v) for v in coef[:3])
    fric = float(coef[3]) if direction is not None else 0.0
    rms = float(np.sqrt(np.mean((m @ coef - y) ** 2)))
    return {
        "amplitude": math.hypot(a, b),
        "phase": math.atan2(b, a),
        "bias": c,
        "friction": fric,
        "rms": rms,
    }


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def analyse_sweep(record: dict[str, Any], qmin: float | None = None) -> dict[str, Any]:
    """Fit one sweep record against the G(q) the runtime published at the
    same samples: scale ``k``, zero offset, bias, Coulomb friction, rms."""
    samples = [s for s in record["samples"] if qmin is None or float(s["q"]) > qmin]
    n = len(samples)
    if n < 20:
        return {"n": n, "usable": False, "reason": "fewer than 20 samples"}
    q = np.array([s["q"] for s in samples], dtype=np.float64)
    tau = np.array([s["tau"] for s in samples], dtype=np.float64)
    g = np.array([s["g"] for s in samples], dtype=np.float64)
    direction = np.array([s["dir"] for s in samples], dtype=np.float64)
    measured = fit_sinusoid(q, tau, direction)
    model = fit_sinusoid(q, g)
    span = float(q.max() - q.min())
    k = (
        measured["amplitude"] / model["amplitude"]
        if model["amplitude"] > 1e-9
        else math.nan
    )
    return {
        "n": n,
        "usable": True,
        "range_rad": span,
        "k": k,
        "offset_rad": _wrap(measured["phase"] - model["phase"]),
        "bias_nm": measured["bias"],
        "friction_nm": abs(measured["friction"]),
        "rms_measured_nm": measured["rms"],
        "rms_model_nm": model["rms"],
        "measured_amplitude_nm": measured["amplitude"],
        "model_amplitude_nm": model["amplitude"],
        "signal_above_friction": measured["amplitude"]
        > 2.0 * abs(measured["friction"]),
    }


# ---------------------------------------------------------------- a sim runtime


@dataclass
class SimDaemon:
    process: subprocess.Popen
    command_port: int
    status_port: int
    telemetry_port: int

    def client(self, **kwargs: Any) -> RobotClient:
        kwargs.setdefault("timeout", 2.0)
        kwargs.setdefault("retries", 2)
        return RobotClient(
            host="127.0.0.1",
            port=self.command_port,
            status_transport="UNICAST",
            status_port=self.status_port,
            status_unicast_host="127.0.0.1",
            **kwargs,
        )


def _free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


@contextlib.contextmanager
def sim_daemon(
    sim_dynamics: bool = False, config: Path | None = None
) -> Iterator[SimDaemon]:
    """A private ``par6d --sim`` on ephemeral loopback ports: the offline
    scripts' source of G(q) — the arm's own model, evaluated by the same
    binary that enforces it — with no arm anywhere near."""
    binary = os.environ.get("PAR6D_BIN") or shutil.which("par6d")
    if not binary:
        raise RuntimeError(
            "par6d binary not found; set PAR6D_BIN or put par6d on PATH "
            "(`cargo build -p par6d --release`)"
        )
    config = config or par6_config.data_root() / "config" / "PAR6.toml"
    env = dict(os.environ)
    assets = REPO / "assets" / "par6_description"
    if assets.is_dir():
        env.setdefault("PAR6_ASSETS", str(assets))
    shm = Path(tempfile.gettempdir()) / f"par6-gravity-shm-{os.getpid()}"
    shm.mkdir(parents=True, exist_ok=True)
    env.setdefault("PAR6_SHM_DIR", str(shm))
    status_port, telemetry_port = _free_udp_port(), _free_udp_port()
    args = [
        binary,
        "--sim",
        "--config",
        str(config),
        "--port",
        "0",
        "--bind",
        "127.0.0.1",
        "--status-transport",
        "unicast",
        "--status-host",
        "127.0.0.1",
        "--status-port",
        str(status_port),
        "--telemetry-port",
        str(telemetry_port),
    ]
    if sim_dynamics:
        args.append("--sim-dynamics")
    process = subprocess.Popen(
        args,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        env=env,
    )
    try:
        assert process.stdout is not None
        line = process.stdout.readline()
        if not line.startswith("PAR6D_READY "):
            raise RuntimeError(f"par6d did not come up: {line.strip()!r}")
        port = next(
            int(f.split("=", 1)[1])
            for f in line.split()
            if f.startswith("command_port=")
        )
        yield SimDaemon(process, port, status_port, telemetry_port)
    finally:
        process.terminate()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=10)


def place_sim_arm(
    client: RobotClient, q_rad: np.ndarray, budget_s: float = 20.0
) -> None:
    """Teleport the simulated arm to ``q_rad`` and wait for the broadcast
    to show it there (teleport is unacked and gated on ENABLED)."""
    pose_deg = [float(v) * DEG for v in q_rad]
    deadline = time.monotonic() + budget_s
    while time.monotonic() < deadline:
        client.teleport(pose_deg)
        if client.wait_status(
            lambda s: (
                s.homed
                and bool(
                    np.all(np.abs(np.asarray(s.angles) - np.asarray(pose_deg)) < 0.5)
                )
            ),
            timeout=2.0,
        ):
            return
    raise RuntimeError(f"teleport to {np.round(q_rad, 3)} did not take effect")
