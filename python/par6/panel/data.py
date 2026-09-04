"""Read-only sources for the screens.

Every getter degrades to ``"—"`` when its source is missing — the runtime
not up, no CAN interface, a thermal zone that does not exist — so the UI
never crashes on absent data. The runtime's state comes from its own
STATUS broadcast through the shipped client; the host vitals are read
from ``/proc`` and ``/sys`` the way ``par6d`` reads its own.
"""

from __future__ import annotations

import logging
import os
import shutil
import socket
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

log = logging.getLogger(__name__)
DASH = "—"


def _safe(fn: Callable[[], Any], default: str = DASH) -> str:
    try:
        v = fn()
        return default if v in (None, "") else str(v)
    except Exception:  # noqa: BLE001 — a screen getter must never raise
        return default


# ---------------------------------------------------------------- host vitals


@dataclass
class Vitals:
    load_1m: float | None = None
    mem_total_mib: int | None = None
    mem_available_mib: int | None = None
    cpu_temp_c: float | None = None
    disk_free_mib: int | None = None
    uptime_s: int | None = None

    @classmethod
    def sample(cls, disk_path: Path = Path("/")) -> "Vitals":
        v = cls()
        try:
            v.load_1m = os.getloadavg()[0]
        except OSError:
            pass
        try:
            fields: dict[str, int] = {}
            for line in Path("/proc/meminfo").read_text().splitlines():
                key, _, rest = line.partition(":")
                if key in ("MemTotal", "MemAvailable"):
                    fields[key] = int(rest.split()[0]) // 1024
            v.mem_total_mib = fields.get("MemTotal")
            v.mem_available_mib = fields.get("MemAvailable")
        except (OSError, ValueError, IndexError):
            pass
        temps = []
        for zone in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
            try:
                temps.append(int(zone.read_text().strip()) / 1000.0)
            except (OSError, ValueError):
                continue
        if temps:
            v.cpu_temp_c = max(temps)
        try:
            v.disk_free_mib = shutil.disk_usage(disk_path).free // (1024 * 1024)
        except OSError:
            pass
        try:
            v.uptime_s = int(float(Path("/proc/uptime").read_text().split()[0]))
        except (OSError, ValueError, IndexError):
            pass
        return v


# ---------------------------------------------------------------- CAN link


def can_link_stats(interface: str) -> dict[str, Any] | None:
    """The kernel's view of the CAN interface, or None when it is absent."""
    try:
        out = subprocess.run(
            ["ip", "-details", "-statistics", "link", "show", interface],
            capture_output=True,
            text=True,
            check=False,
            timeout=2.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if out.returncode != 0:
        return None
    text = out.stdout
    stats: dict[str, Any] = {"up": " UP" in text.split("\n")[0] or "state UP" in text}
    for key in ("bus-error", "error-warning", "error-passive", "bus-off", "restarts"):
        for token in text.replace("\n", " ").split():
            if token.startswith(key + ":"):
                try:
                    stats[key] = int(token.split(":", 1)[1])
                except ValueError:
                    pass
    for line in text.splitlines():
        if "bitrate" in line:
            parts = line.split()
            if "bitrate" in parts:
                try:
                    stats["bitrate"] = int(parts[parts.index("bitrate") + 1])
                except (ValueError, IndexError):
                    pass
    return stats


def describe_can_change(before: dict[str, Any], after: dict[str, Any]) -> str | None:
    """A one-line description of what moved, or None when nothing did."""
    moved = []
    for key in ("bus-error", "error-warning", "error-passive", "bus-off", "restarts"):
        if before.get(key) != after.get(key):
            moved.append(f"{key} {before.get(key)}→{after.get(key)}")
    if before.get("up") != after.get("up"):
        moved.append("link " + ("UP" if after.get("up") else "DOWN"))
    return ", ".join(moved) if moved else None


# ---------------------------------------------------------------- runtime


@dataclass
class RuntimeView:
    """The latest STATUS broadcast, reduced to what the screens show."""

    seen: bool = False
    mode: str = DASH
    enabled: bool = False
    homed: bool = False
    freedrive: bool = False
    error: str = ""
    angles_deg: list[float] = field(default_factory=list)
    link_state: str = DASH
    last_seen: float = 0.0

    def fresh(self, now: float, within_s: float = 2.0) -> bool:
        return self.seen and now - self.last_seen < within_s


class RuntimeWatch:
    """Follows the runtime's STATUS broadcast on a thread; never blocks
    the panel loop and never raises into it."""

    def __init__(
        self, connect: Callable[[], Any], *, clock: Callable[[], float] = time.monotonic
    ):
        self._connect = connect
        self._clock = clock
        self.view = RuntimeView()
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="par6-panel-status", daemon=True
        )

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                with self._connect() as client:
                    while not self._stop.is_set():
                        if not client.wait_status(self._absorb, timeout=1.0):
                            continue
            except Exception as exc:  # noqa: BLE001 — the watch retries forever, quietly
                log.debug("status watch: %s", exc)
                time.sleep(1.0)

    def _absorb(self, s: Any) -> bool:
        v = self.view
        v.seen = True
        v.mode = getattr(s.mode, "name", str(s.mode))
        v.enabled = bool(s.enabled)
        v.homed = bool(s.homed)
        v.freedrive = bool(getattr(s, "freedrive", False))
        v.error = (
            ""
            if s.error is None
            else str(s.error[0] if isinstance(s.error, tuple) else s.error)
        )
        v.angles_deg = [float(a) for a in s.angles]
        health = getattr(s, "link_health", {}) or {}
        v.link_state = str(health.get("state", DASH))
        v.last_seen = self._clock()
        return True


# ---------------------------------------------------------------- systemd


def unit_active(unit: str) -> str:
    """``active``/``inactive``/``failed``… or ``—`` without systemd."""
    try:
        out = subprocess.run(
            ["systemctl", "is-active", unit],
            capture_output=True,
            text=True,
            check=False,
            timeout=2.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return DASH
    return out.stdout.strip() or DASH


def ip_address() -> str:
    def _ip() -> str:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))
            return s.getsockname()[0]

    return _safe(_ip)


def hostname() -> str:
    return _safe(socket.gethostname)
