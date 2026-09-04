"""``panel.toml`` — every device path, address, pin and rate the panel
service uses, and nothing hard-coded anywhere else."""

from __future__ import annotations

import os
import tomllib
from dataclasses import dataclass
from pathlib import Path

from par6 import config as _cfg


@dataclass(frozen=True)
class DisplayConfig:
    driver: str
    i2c_bus: int
    i2c_address: int
    width: int
    height: int
    contrast: int
    refresh_s: float


@dataclass(frozen=True)
class ButtonsConfig:
    button1_pin: int
    button2_pin: int
    pull_up: bool
    long_press_s: float
    debounce_s: float


@dataclass(frozen=True)
class LedsConfig:
    led1_pin: int
    led2_pin: int
    blink_s: float


@dataclass(frozen=True)
class PcbConfig:
    port: str
    baud: int
    timeout_s: float
    fields: int
    heartbeat: str


@dataclass(frozen=True)
class RuntimeConfig:
    host: str
    command_port: int
    status_port: int
    status_transport: str
    can_interface: str
    unit: str


@dataclass(frozen=True)
class LoopConfig:
    period_s: float
    link_monitor_s: float
    idle_return_s: float


@dataclass(frozen=True)
class PanelConfig:
    display: DisplayConfig
    buttons: ButtonsConfig
    leds: LedsConfig
    pcb: PcbConfig
    runtime: RuntimeConfig
    loop: LoopConfig
    settings_file: Path


def default_path() -> Path:
    """``PAR6_PANEL_CONFIG``, else the packaged ``panel.toml``."""
    env = os.environ.get("PAR6_PANEL_CONFIG")
    if env:
        return Path(env)
    return _cfg.data_root() / "config" / "panel.toml"


def _positive(section: str, key: str, value: float) -> float:
    if not value > 0:
        raise ValueError(f"[{section}] {key} must be positive, got {value}")
    return value


def parse(text: str) -> PanelConfig:
    raw = tomllib.loads(text)
    d, b, led, pcb, rt, lp = (
        raw["display"],
        raw["buttons"],
        raw["leds"],
        raw["pcb"],
        raw["runtime"],
        raw["loop"],
    )
    if b["button1_pin"] == b["button2_pin"]:
        raise ValueError("[buttons] button1_pin and button2_pin are the same pin")
    if led["led1_pin"] == led["led2_pin"]:
        raise ValueError("[leds] led1_pin and led2_pin are the same pin")
    claimed = {b["button1_pin"], b["button2_pin"], led["led1_pin"], led["led2_pin"]}
    if len(claimed) != 4:
        raise ValueError("[buttons]/[leds] claim one pin twice")
    if d["driver"] not in ("ssd1306", "none"):
        raise ValueError(f"[display] driver {d['driver']!r} is not ssd1306 or none")
    return PanelConfig(
        display=DisplayConfig(
            driver=str(d["driver"]),
            i2c_bus=int(d["i2c_bus"]),
            i2c_address=int(d["i2c_address"]),
            width=int(d["width"]),
            height=int(d["height"]),
            contrast=int(d["contrast"]),
            refresh_s=_positive("display", "refresh_s", float(d["refresh_s"])),
        ),
        buttons=ButtonsConfig(
            button1_pin=int(b["button1_pin"]),
            button2_pin=int(b["button2_pin"]),
            pull_up=bool(b["pull_up"]),
            long_press_s=_positive("buttons", "long_press_s", float(b["long_press_s"])),
            debounce_s=_positive("buttons", "debounce_s", float(b["debounce_s"])),
        ),
        leds=LedsConfig(
            led1_pin=int(led["led1_pin"]),
            led2_pin=int(led["led2_pin"]),
            blink_s=_positive("leds", "blink_s", float(led["blink_s"])),
        ),
        pcb=PcbConfig(
            port=str(pcb["port"]),
            baud=int(pcb["baud"]),
            timeout_s=_positive("pcb", "timeout_s", float(pcb["timeout_s"])),
            fields=int(pcb["fields"]),
            heartbeat=str(pcb["heartbeat"]),
        ),
        runtime=RuntimeConfig(
            host=str(rt["host"]),
            command_port=int(rt["command_port"]),
            status_port=int(rt["status_port"]),
            status_transport=str(rt["status_transport"]).upper(),
            can_interface=str(rt["can_interface"]),
            unit=str(rt["unit"]),
        ),
        loop=LoopConfig(
            period_s=_positive("loop", "period_s", float(lp["period_s"])),
            link_monitor_s=_positive(
                "loop", "link_monitor_s", float(lp["link_monitor_s"])
            ),
            idle_return_s=_positive(
                "loop", "idle_return_s", float(lp["idle_return_s"])
            ),
        ),
        settings_file=Path(str(raw["settings"]["file"])),
    )


def load(path: Path | None = None) -> PanelConfig:
    return parse((path or default_path()).read_text())
