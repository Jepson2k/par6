"""Live ``par6d --sim`` rig for the end-to-end tests.

Spawns the REAL daemon binary on ephemeral ports and talks to it with the
real :mod:`par6.client` over real UDP — no scripted peer, no fakes. The
binary is resolved exactly like :func:`par6.robot._find_par6d` (``PAR6D_BIN``
then ``PATH``); when neither resolves, :data:`requires_par6d` skips the
tests instead of failing, so a checkout without a Rust build stays green.
"""

from __future__ import annotations

import os
import queue
import shutil
import socket
import subprocess
import threading
from dataclasses import dataclass
from pathlib import Path

import pytest

from par6 import config as _cfg
from par6.client import AsyncRobotClient

#: Boot budget for the daemon's ``PAR6D_READY`` line.
READY_TIMEOUT_S = 30.0

#: The sim tick the e2e rig runs at. Every RT time constant derives from
#: config SECONDS (``round(s/dt)``), so the runtime is rate-agnostic by
#: contract and the wiring under test is identical to the shipped 250 Hz —
#: but a shared CI box cannot hold a 4 ms deadline and would latch
#: LOOP_CRITICAL (p99 > 1.10*dt sustained 1 s) mid-test. 50 ms leaves the
#: jitter headroom. ``status_rate_hz`` must integer-divide the tick rate.
TICK_DT_S = 0.05
STATUS_RATE_HZ = 20


def par6d_binary() -> str | None:
    """The ``par6d`` binary from ``PAR6D_BIN``, then ``PATH``; None if absent."""
    env_bin = os.environ.get("PAR6D_BIN")
    if env_bin:
        return env_bin if os.path.isfile(env_bin) else None
    return shutil.which("par6d")


requires_par6d = pytest.mark.skipif(
    par6d_binary() is None,
    reason="no par6d binary (set PAR6D_BIN or put it on PATH; "
    "build with `cargo build -p par6d --release`)",
)


def free_udp_port() -> int:
    """A currently-free loopback UDP port (bind, read, release)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def sim_config(dest: Path) -> Path:
    """The packaged PAR6 config re-ticked for CI, written under *dest*.

    Sourced from ``par6/_data`` (the same tree the client reads), so the
    daemon under test runs the joints, limits and homing sequence the
    Python package advertises.
    """
    src = _cfg.data_root() / "config"
    dest.mkdir(parents=True, exist_ok=True)
    (dest / "grippers").mkdir(exist_ok=True)
    text = (src / "PAR6.toml").read_text()
    patched = text.replace("tick_dt_s = 0.004", f"tick_dt_s = {TICK_DT_S}").replace(
        "status_rate_hz = 50", f"status_rate_hz = {STATUS_RATE_HZ}"
    )
    if patched == text:
        raise RuntimeError("PAR6.toml patch points (tick_dt_s / status_rate_hz) missing")
    out = dest / "PAR6.toml"
    out.write_text(patched)
    for gripper in sorted((src / "grippers").glob("*.toml")):
        shutil.copy(gripper, dest / "grippers" / gripper.name)
    return out


@dataclass
class LiveDaemon:
    """A running ``par6d --sim`` process on ephemeral loopback ports."""

    process: subprocess.Popen
    command_port: int
    status_port: int
    telemetry_port: int
    config: Path
    log_path: Path

    @classmethod
    def start(cls, workdir: Path) -> "LiveDaemon":
        binary = par6d_binary()
        if binary is None:
            raise RuntimeError("par6d binary not available")
        config = sim_config(workdir / "config")
        status_port = free_udp_port()
        telemetry_port = free_udp_port()
        log_path = workdir / "par6d.log"
        log = log_path.open("w")
        process = subprocess.Popen(
            [
                binary,
                "--sim",
                "--config", str(config),
                "--port", "0",
                "--bind", "127.0.0.1",
                "--status-transport", "unicast",
                "--status-host", "127.0.0.1",
                "--status-port", str(status_port),
                "--telemetry-port", str(telemetry_port),
            ],
            stdout=subprocess.PIPE,
            stderr=log,
            text=True,
        )
        try:
            command_port = cls._read_ready_port(process, log_path)
        except BaseException:
            process.kill()
            process.wait(timeout=10)
            raise
        finally:
            log.close()
        return cls(
            process=process,
            command_port=command_port,
            status_port=status_port,
            telemetry_port=telemetry_port,
            config=config,
            log_path=log_path,
        )

    @staticmethod
    def _read_ready_port(process: subprocess.Popen, log_path: Path) -> int:
        """Parse ``command_port`` out of the machine-readable ready line."""
        lines: queue.Queue[str] = queue.Queue()
        stdout = process.stdout
        assert stdout is not None
        threading.Thread(
            target=lambda: lines.put(stdout.readline()), daemon=True
        ).start()
        try:
            line = lines.get(timeout=READY_TIMEOUT_S)
        except queue.Empty:
            raise RuntimeError(
                f"par6d printed no ready line in {READY_TIMEOUT_S}s; "
                f"log:\n{log_path.read_text()}"
            ) from None
        if not line.startswith("PAR6D_READY "):
            raise RuntimeError(
                f"par6d ready line malformed: {line!r}; log:\n{log_path.read_text()}"
            )
        for field in line.split():
            if field.startswith("command_port="):
                return int(field.split("=", 1)[1])
        raise RuntimeError(f"no command_port in ready line: {line!r}")

    def log(self) -> str:
        """Everything the daemon logged so far (stderr)."""
        return self.log_path.read_text() if self.log_path.exists() else ""

    def stop(self) -> None:
        """SIGTERM the daemon and reap it (SIGKILL as a backstop)."""
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=10)
        if self.process.stdout is not None:
            self.process.stdout.close()

    def client(self, **kwargs) -> AsyncRobotClient:
        """An :class:`AsyncRobotClient` wired to this daemon's ports."""
        kwargs.setdefault("timeout", 2.0)
        kwargs.setdefault("retries", 2)
        return AsyncRobotClient(
            host="127.0.0.1",
            port=self.command_port,
            status_transport="UNICAST",
            status_port=self.status_port,
            status_unicast_host="127.0.0.1",
            **kwargs,
        )
