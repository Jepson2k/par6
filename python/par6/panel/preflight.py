"""``par6-preflight`` — is this box ready to run the arm?

Diagnostic only: it brings nothing up and changes no state. It re-execs
itself into the virtualenv interpreter next to the installed package
when it is not already running there, so the answer reflects the runtime
environment rather than whichever Python was on the path. Every result
carries a required/advisory flag; the exit code is 1 only when a
required check failed.

Checks: the RT kernel, CAN present/up/bitrate, the GPIO chip and
RT-priority permissions, cores, disk, the panel devices, the imports the
runtime and the panel need, and the runtime binary and config.
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import resource
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

from par6.panel import data
from par6.panel.config import PanelConfig, default_path, load


@dataclass
class Check:
    name: str
    ok: bool
    detail: str
    required: bool = True


class Report:
    def __init__(self) -> None:
        self.checks: list[Check] = []

    def add(self, name: str, ok: bool, detail: str, required: bool = True) -> None:
        self.checks.append(Check(name, bool(ok), detail, required))

    @property
    def required_failures(self) -> int:
        return sum(1 for c in self.checks if c.required and not c.ok)


def reexec_into_venv(argv: list[str]) -> None:
    """Run under the interpreter of the virtualenv this package lives in,
    when there is one and we are not already it."""
    prefix = Path(sys.prefix)
    venv_python = prefix / "bin" / "python"
    marker = prefix / "pyvenv.cfg"
    if marker.exists() and venv_python.exists():
        return
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "bin" / "python"
        if (parent / "pyvenv.cfg").exists() and candidate.exists():
            if Path(sys.executable).resolve() != candidate.resolve():
                os.execv(
                    str(candidate),
                    [str(candidate), "-m", "par6.panel.preflight", *argv],
                )
            return


def file_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8", errors="replace").strip("\x00\n ")
    except OSError:
        return None


def run(cmd: list[str]) -> subprocess.CompletedProcess[str] | None:
    try:
        return subprocess.run(
            cmd, capture_output=True, text=True, check=False, timeout=5.0
        )
    except (OSError, subprocess.TimeoutExpired):
        return None


def check_platform(r: Report) -> None:
    r.add("Python >= 3.11", sys.version_info >= (3, 11), platform.python_version())
    kernel = platform.release()
    rt_flag = file_text(Path("/sys/kernel/realtime"))
    rt = rt_flag == "1" or "rt" in kernel.lower()
    r.add(
        "RT kernel",
        rt,
        f"kernel={kernel} /sys/kernel/realtime={rt_flag!r}",
        required=False,
    )
    cores = os.cpu_count() or 0
    r.add(
        "CPU cores >= 4",
        cores >= 4,
        f"{cores} core(s) — par6d pins its RT thread to core 3",
    )
    model = file_text(Path("/proc/device-tree/model"))
    r.add(
        "Control box model",
        model is not None,
        model or "not a Pi, or model unavailable",
        required=False,
    )


def check_can(r: Report, interface: str) -> None:
    stats = data.can_link_stats(interface)
    if stats is None:
        r.add(
            f"CAN {interface} present",
            False,
            "not found (iproute2 missing or no interface)",
        )
        return
    r.add(f"CAN {interface} present", True, "found")
    r.add(
        f"CAN {interface} up",
        bool(stats.get("up")),
        "UP" if stats.get("up") else "DOWN — par6d brings it up itself",
    )
    bitrate = stats.get("bitrate")
    r.add(
        "CAN bitrate 1 Mbit/s",
        bitrate == 1_000_000,
        f"bitrate {bitrate}",
        required=False,
    )


def check_devices(r: Report, cfg: PanelConfig) -> None:
    if cfg.display.driver != "none":
        i2c = Path(f"/dev/i2c-{cfg.display.i2c_bus}")
        ok = i2c.exists() and os.access(i2c, os.R_OK | os.W_OK)
        r.add(
            "OLED I2C bus", ok, f"{i2c} exists={i2c.exists()} rw={ok}", required=False
        )
    if cfg.pcb.port:
        uart = Path(cfg.pcb.port)
        ok = uart.exists() and os.access(uart, os.R_OK | os.W_OK)
        r.add("PCB UART", ok, f"{uart} exists={uart.exists()} rw={ok}", required=False)
    chips = sorted(Path("/dev").glob("gpiochip*"))
    usable = [c for c in chips if os.access(c, os.R_OK | os.W_OK)]
    r.add(
        "GPIO chip access",
        bool(usable),
        ("usable: " + ", ".join(map(str, usable)))
        if usable
        else ("found: " + (", ".join(map(str, chips)) or "none")),
        required=False,
    )


def check_rt_permissions(r: Report) -> None:
    soft, hard = resource.getrlimit(resource.RLIMIT_RTPRIO)
    probe = run(
        [
            sys.executable,
            "-c",
            "import os; os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(1)); print('ok')",
        ]
    )
    ok = probe is not None and probe.returncode == 0
    detail = f"euid={os.geteuid()} RLIMIT_RTPRIO soft={soft} hard={hard}"
    if not ok and probe is not None:
        detail += " probe: " + " ".join(probe.stderr.split())[:160]
    r.add("RT priority permission", ok, detail, required=False)


def check_host(r: Report) -> None:
    v = data.Vitals.sample()
    if v.disk_free_mib is not None:
        r.add(
            "Free disk >= 1 GiB",
            v.disk_free_mib >= 1024,
            f"{v.disk_free_mib} MiB free",
            required=False,
        )
    if v.mem_available_mib is not None:
        r.add(
            "Memory available >= 512 MiB",
            v.mem_available_mib >= 512,
            f"{v.mem_available_mib} MiB",
            required=False,
        )
    if v.cpu_temp_c is not None:
        r.add(
            "CPU temperature < 80 C",
            v.cpu_temp_c < 80.0,
            f"{v.cpu_temp_c:.0f} C",
            required=False,
        )


def check_runtime(r: Report, cfg: PanelConfig) -> None:
    binary = shutil.which("par6d") or (
        "/usr/local/bin/par6d" if Path("/usr/local/bin/par6d").exists() else None
    )
    r.add(
        "par6d binary", binary is not None, binary or "not on PATH nor /usr/local/bin"
    )
    config = os.environ.get("PAR6_CONFIG") or "/etc/par6/PAR6.toml"
    r.add("robot config", Path(config).is_file(), config, required=False)
    r.add(
        f"unit {cfg.runtime.unit}",
        data.unit_active(cfg.runtime.unit) == "active",
        data.unit_active(cfg.runtime.unit),
        required=False,
    )


def check_imports(r: Report, cfg: PanelConfig) -> None:
    modules = {
        "par6._par6": True,
        "par6.client": True,
        "PIL": True,
        "gpiozero": False,
        "serial": cfg.pcb.port != "",
    }
    if cfg.display.driver == "ssd1306":
        modules["adafruit_ssd1306"] = False
        modules["board"] = False
    for module, required in modules.items():
        probe = run(
            [
                sys.executable,
                "-c",
                f"import importlib; importlib.import_module({module!r})",
            ]
        )
        ok = probe is not None and probe.returncode == 0
        detail = (
            "ok"
            if ok
            else (" ".join(probe.stderr.split())[:200] if probe else "probe failed")
        )
        r.add(f"import {module}", ok, detail, required=required)


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(
        description="PAR6 control box preflight (diagnostic only)"
    )
    parser.add_argument("--config", type=Path, default=None, help="panel.toml")
    parser.add_argument("--json", action="store_true")
    parser.add_argument(
        "--no-reexec", action="store_true", help="stay on this interpreter"
    )
    args = parser.parse_args(argv)
    if not args.no_reexec:
        reexec_into_venv(argv)
    cfg = load(args.config or default_path())
    r = Report()
    check_platform(r)
    check_can(r, cfg.runtime.can_interface)
    check_devices(r, cfg)
    check_rt_permissions(r)
    check_host(r)
    check_runtime(r, cfg)
    check_imports(r, cfg)
    if args.json:
        print(
            json.dumps(
                {
                    "checks": [asdict(c) for c in r.checks],
                    "required_failures": r.required_failures,
                }
            )
        )
    else:
        print(f"par6 preflight ({sys.executable})")
        for c in r.checks:
            status = "OK" if c.ok else ("FAIL" if c.required else "WARN")
            print(f"[{status:4}] {c.name}: {c.detail}")
        print(f"\n{r.required_failures} required failure(s)")
    return 1 if r.required_failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
