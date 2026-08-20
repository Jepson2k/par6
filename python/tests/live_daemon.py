"""Live ``par6d --sim`` rig for the end-to-end tests.

Spawns the REAL daemon binary on ephemeral ports and talks to it with the
real :mod:`par6.client` over real UDP — no scripted peer, no fakes. The
binary is resolved exactly like :func:`par6.robot._find_par6d` (``PAR6D_BIN``
then ``PATH``); when neither resolves, :data:`requires_par6d` skips the
tests instead of failing, so a checkout without a Rust build stays green.
"""

from __future__ import annotations

import asyncio
import functools
import os
import queue
import shutil
import socket
import subprocess
import tempfile
import threading
import time
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
#: but a shared CI box cannot hold a 4 ms deadline. 50 ms leaves the jitter
#: headroom. The generated config deliberately declares no ``[timing]``
#: section, so ``par6d --sim`` applies its relaxed loop-degradation bands
#: and host load raises the self-clearing LOOP_DEGRADED warning instead of
#: latching LOOP_CRITICAL. ``status_rate_hz`` must integer-divide the tick
#: rate.
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


def repo_assets_dir() -> Path | None:
    """The repo's ``assets/par6_description`` tree, if this is a checkout.

    A ``par6d`` built with the ``ffi`` feature loads its kinematics and
    collision models from that tree and looks for it next to the config file —
    and every config these tests hand it lives in a tmp dir, so it has to be
    named explicitly for the same tests to serve both build flavors.
    """
    assets = Path(__file__).resolve().parents[2] / "assets" / "par6_description"
    return assets if assets.is_dir() else None


def daemon_env() -> dict[str, str]:
    """Environment for the daemon under test.

    The bus-ownership grant (``loop_tick`` / ``robot_mode``) is published
    under fixed names, and a stopping daemon REMOVES them — so two test
    runs sharing ``/dev/shm`` would delete each other's live claim. Ports
    are already allocated per run; the grant needs the same treatment.
    The Rust harness does this per process id; do the same here.
    """
    env = dict(os.environ)
    assets = repo_assets_dir()
    if "PAR6_ASSETS" not in env and assets is not None:
        env["PAR6_ASSETS"] = str(assets)
    env.setdefault("PAR6_SHM_DIR", str(_shm_dir()))
    return env


@functools.lru_cache(maxsize=1)
def _shm_dir() -> Path:
    """A per-process scratch directory for this run's grant segments."""
    path = Path(tempfile.gettempdir()) / f"par6-test-shm-{os.getpid()}"
    path.mkdir(parents=True, exist_ok=True)
    return path


def free_udp_port() -> int:
    """A currently-free loopback UDP port (bind, read, release)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def sim_config(dest: Path, active_gripper: str | None = None) -> Path:
    """The packaged PAR6 config re-ticked for CI, written under *dest*.

    Sourced from ``par6/_data`` (the same tree the client reads), so the
    daemon under test runs the joints, limits and homing sequence the
    Python package advertises.  *active_gripper* fits the daemon with a
    different tool than the packaged config names — the runtime picks its
    URDF variant, its gravity model and its TCP frame from that one key.
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
    if active_gripper is not None:
        fitted = _cfg.load_robot_config()["robot"]["active_gripper"]
        swapped = patched.replace(
            f'active_gripper = "{fitted}"', f'active_gripper = "{active_gripper}"'
        )
        if swapped == patched and active_gripper != fitted:
            raise RuntimeError("PAR6.toml patch point (active_gripper) missing")
        patched = swapped
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
    status_transport: str = "unicast"

    @classmethod
    def start(
        cls,
        workdir: Path,
        active_gripper: str | None = None,
        status_transport: str = "unicast",
    ) -> "LiveDaemon":
        binary = par6d_binary()
        if binary is None:
            raise RuntimeError("par6d binary not available")
        config = sim_config(workdir / "config", active_gripper)
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
                "--status-transport", status_transport,
                "--status-host", "127.0.0.1",
                "--status-port", str(status_port),
                "--telemetry-port", str(telemetry_port),
            ],
            stdout=subprocess.PIPE,
            stderr=log,
            text=True,
            env=daemon_env(),
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
            status_transport=status_transport,
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
        """An :class:`AsyncRobotClient` wired to this daemon's ports.

        Pinned to unicast to match the rig's own default, so a test is not
        at the mercy of whether the host routes multicast. Start the daemon
        with ``status_transport="auto"`` and pass ``shipped_transport=True``
        here to exercise the shipped ladder instead.
        """
        kwargs.setdefault("timeout", 2.0)
        kwargs.setdefault("retries", 2)
        if kwargs.pop("shipped_transport", False):
            return AsyncRobotClient(
                host="127.0.0.1",
                port=self.command_port,
                status_port=self.status_port,
                **kwargs,
            )
        return AsyncRobotClient(
            host="127.0.0.1",
            port=self.command_port,
            status_transport="UNICAST",
            status_port=self.status_port,
            status_unicast_host="127.0.0.1",
            **kwargs,
        )


async def settle_at(
    client: AsyncRobotClient, angles_deg: list[float], budget_s: float = 20.0
) -> None:
    """Reset the controller and leave the sim arm standing at *angles_deg*.

    Teleport is unacked and gated on ENABLED, and the RT clear sequence
    settles over several ticks, so both are re-sent until the broadcast
    shows the arm there — the same loop a UI runs. Raises when the arm
    never arrives within *budget_s*.
    """
    deadline = time.monotonic() + budget_s
    await client.reset()
    while time.monotonic() < deadline:
        await client.teleport(angles_deg)
        arrived = await client.wait_status(
            lambda s: s.homed
            and all(abs(a - b) < 0.5 for a, b in zip(s.angles, angles_deg)),
            timeout=0.5,
        )
        if arrived:
            return
        await asyncio.sleep(0.05)
    raise AssertionError(f"the sim arm never reached {angles_deg}")


async def angles_now(client: AsyncRobotClient) -> list[float]:
    """The arm's joint angles, refusing the dropped-query case.

    ``angles()`` returns ``None`` when every retry of the query goes
    unanswered.  Subscripting that directly reports a ``NoneType`` error at
    the read instead of naming the lost query, and a test that meant to
    measure a motion silently becomes a test of nothing.
    """
    angles = await client.angles()
    assert angles is not None, "the ANGLES query went unanswered"
    return angles


async def pose_now(client: AsyncRobotClient) -> list[float]:
    """The TCP pose, refusing the dropped-query case — see :func:`angles_now`."""
    pose = await client.pose()
    assert pose is not None, "the POSE query went unanswered"
    return pose
