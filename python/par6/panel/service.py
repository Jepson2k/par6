"""``par6-panel`` — the front panel service.

Threads:

- the main loop (~50 Hz): buttons, the PCB frame, the heartbeat tick;
- the display thread: renders the current screen at the display rate;
- the link monitor (1 Hz): CAN counters when they move, runtime liveness.

Menu actions that restart things go through systemd and stop the unit
first; nothing here touches the arm directly. Hardware comes from
factories chosen by ``panel.toml``: a missing OLED library, a UART that
will not open or a GPIO chip that is not there each degrade to the parts
that still work, logged, never a crash.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import subprocess
import tempfile
import threading
import time
from pathlib import Path
from typing import Any, Callable

from PIL import Image

from par6.client import RobotClient
from par6.panel import data
from par6.panel.buttons import ButtonHandler
from par6.panel.config import PanelConfig, load
from par6.panel.link import Heartbeat, PanelState, PcbLink
from par6.panel.ui import PanelUi, Registries, Screen

log = logging.getLogger("par6.panel")

DEFAULT_SETTINGS: dict[str, Any] = {"contrast": 255, "blink_s": 1.0}


# ---------------------------------------------------------------- settings store


def load_settings(path: Path, defaults: dict[str, Any]) -> dict[str, Any]:
    """Merge a saved file over the defaults, one key at a time, coercing
    each value to the default's type so a hand-edited file can change a
    number but never a behaviour; a missing or corrupt file keeps the
    defaults."""
    values = dict(defaults)
    try:
        saved = json.loads(path.read_text())
    except FileNotFoundError:
        return values
    except (OSError, ValueError) as exc:
        log.warning("settings %s unreadable (%s); using defaults", path, exc)
        return values
    for key, default in defaults.items():
        if key in saved:
            try:
                values[key] = type(default)(saved[key])
            except (TypeError, ValueError):
                log.warning(
                    "settings key %r has a bad value %r; keeping default",
                    key,
                    saved[key],
                )
    return values


def save_settings(path: Path, values: dict[str, Any]) -> None:
    """Atomic: temp file + rename, so a power loss mid-write never leaves
    half a file."""
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=path.name, suffix=".tmp")
        with os.fdopen(fd, "w") as f:
            json.dump(values, f)
        os.replace(tmp, path)
    except OSError as exc:
        log.warning("settings %s not saved: %s", path, exc)


# ---------------------------------------------------------------- hardware


class Display:
    """Whatever shows a frame: an SSD1306 over I²C, or nothing."""

    def show(self, img: Image.Image) -> None: ...

    def contrast(self, value: int) -> None: ...


class NullDisplay(Display):
    def __init__(self) -> None:
        self.frames = 0
        self.last: Image.Image | None = None

    def show(self, img: Image.Image) -> None:
        self.frames += 1
        self.last = img

    def contrast(self, value: int) -> None:
        pass


def open_display(cfg: PanelConfig) -> Display:
    if cfg.display.driver == "none":
        return NullDisplay()
    try:
        import board  # ty: ignore[unresolved-import]
        import busio  # ty: ignore[unresolved-import]
        from adafruit_ssd1306 import SSD1306_I2C  # ty: ignore[unresolved-import]
    except ImportError as exc:
        log.error("OLED libraries missing (%s); running without a display", exc)
        return NullDisplay()
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        disp = SSD1306_I2C(
            cfg.display.width, cfg.display.height, i2c, addr=cfg.display.i2c_address
        )
    except Exception as exc:  # noqa: BLE001 — no OLED on the bus degrades to no display
        log.error(
            "OLED at 0x%02x on I2C-%d not answering (%s); running without a display",
            cfg.display.i2c_address,
            cfg.display.i2c_bus,
            exc,
        )
        return NullDisplay()

    class Ssd1306(Display):
        def show(self, img: Image.Image) -> None:
            disp.image(img)
            disp.show()

        def contrast(self, value: int) -> None:
            disp.contrast(int(value))

    return Ssd1306()


class NullLed:
    def __init__(self) -> None:
        self.on = False

    def set(self, on: bool) -> None:
        self.on = on


def open_leds(cfg: PanelConfig) -> tuple[Any, Any]:
    try:
        from gpiozero import LED  # ty: ignore[unresolved-import]
    except ImportError:
        log.error("gpiozero missing; LEDs disabled")
        return NullLed(), NullLed()
    try:
        led1, led2 = LED(cfg.leds.led1_pin), LED(cfg.leds.led2_pin)
    except Exception as exc:  # noqa: BLE001 — no GPIO chip degrades to no LEDs
        log.error(
            "LED pins %d/%d unavailable (%s); LEDs disabled",
            cfg.leds.led1_pin,
            cfg.leds.led2_pin,
            exc,
        )
        return NullLed(), NullLed()

    class GpioLed:
        def __init__(self, dev: Any) -> None:
            self._dev = dev

        def set(self, on: bool) -> None:
            self._dev.value = int(on)

    return GpioLed(led1), GpioLed(led2)


def open_buttons(cfg: PanelConfig) -> tuple[Callable[[], bool], Callable[[], bool]]:
    try:
        from gpiozero import Button  # ty: ignore[unresolved-import]
    except ImportError:
        log.error("gpiozero missing; buttons disabled")
        return (lambda: False), (lambda: False)
    try:
        b1 = Button(cfg.buttons.button1_pin, pull_up=cfg.buttons.pull_up)
        b2 = Button(cfg.buttons.button2_pin, pull_up=cfg.buttons.pull_up)
    except Exception as exc:  # noqa: BLE001 — no GPIO chip degrades to no buttons
        log.error("button pins unavailable (%s); buttons disabled", exc)
        return (lambda: False), (lambda: False)
    return (lambda: bool(b1.is_pressed)), (lambda: bool(b2.is_pressed))


def open_pcb(cfg: PanelConfig) -> PcbLink:
    def opener() -> Any:
        if not cfg.pcb.port:
            raise OSError("no PCB port configured")
        import serial

        return serial.Serial(cfg.pcb.port, cfg.pcb.baud, timeout=cfg.pcb.timeout_s)

    return PcbLink(opener, fields=cfg.pcb.fields, heartbeat=cfg.pcb.heartbeat)


# ---------------------------------------------------------------- the service


class PanelService:
    def __init__(
        self,
        cfg: PanelConfig,
        *,
        display: Display,
        leds: tuple[Any, Any],
        buttons: tuple[Callable[[], bool], Callable[[], bool]],
        pcb: PcbLink,
        connect: Callable[[], Any] | None = None,
        run_unit_action: Callable[[list[str]], None] | None = None,
        clock: Callable[[], float] = time.perf_counter,
    ) -> None:
        self.cfg = cfg
        self.display = display
        self.led1, self.led2 = leds
        self.clock = clock
        self._run_unit_action = run_unit_action or self._systemctl
        self.settings = load_settings(cfg.settings_file, DEFAULT_SETTINGS)
        self.reg = Registries(
            values=self.settings, on_save=lambda v: save_settings(cfg.settings_file, v)
        )
        self.watch = data.RuntimeWatch(connect or self._connect)
        self.heartbeat = Heartbeat(pcb, self.led1, self.led2, publish=self._publish)
        self.published: PanelState | None = None
        self.vitals = data.Vitals()
        self.can: dict[str, Any] | None = None
        self.unit_state = data.DASH
        self._register()
        self.ui = PanelUi(
            self.reg,
            self._home,
            idle_return_s=cfg.loop.idle_return_s,
            width=cfg.display.width,
            height=cfg.display.height,
            clock=clock,
        )
        self.button1 = ButtonHandler(
            buttons[0],
            long_press_s=cfg.buttons.long_press_s,
            debounce_s=cfg.buttons.debounce_s,
            on_short=self.ui.on_b1_short,
            on_long=self.ui.on_b1_long,
            clock=clock,
        )
        self.button2 = ButtonHandler(
            buttons[1],
            long_press_s=cfg.buttons.long_press_s,
            debounce_s=cfg.buttons.debounce_s,
            on_short=self.ui.on_b2_short,
            on_long=self.ui.on_b2_long,
            clock=clock,
        )
        self._last_blink = clock()
        self._stop = threading.Event()
        self._applied_contrast: int | None = None

    # -- wiring
    def _connect(self) -> RobotClient:
        rt = self.cfg.runtime
        return RobotClient(
            host=rt.host,
            port=rt.command_port,
            timeout=2.0,
            retries=1,
            status_transport=rt.status_transport,
            status_port=rt.status_port,
            status_unicast_host=rt.host,
        )

    def _systemctl(self, argv: list[str]) -> None:
        subprocess.run(["sudo", "-n", "systemctl", *argv], check=False, timeout=30.0)

    def _publish(self, state: PanelState) -> None:
        self.published = state

    def _register(self) -> None:
        reg = self.reg
        unit = self.cfg.runtime.unit

        @reg.info_page("Runtime")
        def _runtime(s: Screen) -> None:
            v = self.watch.view
            live = v.fresh(time.monotonic())
            s.body(0, "Mode", v.mode if live else "no runtime")
            s.body(1, "Enabled", ("yes" if v.enabled else "no") if live else data.DASH)
            s.body(2, "Homed", ("yes" if v.homed else "no") if live else data.DASH)
            s.body(3, "Error", (v.error or "none") if live else data.DASH)

        @reg.info_page("Network")
        def _network(s: Screen) -> None:
            s.body(0, "IP", data.ip_address())
            s.body(1, "Host", data.hostname())
            s.body(2, "Cmd port", str(self.cfg.runtime.command_port))

        @reg.info_page("System")
        def _system(s: Screen) -> None:
            v = self.vitals
            s.body(
                0,
                "Temp",
                f"{v.cpu_temp_c:.0f}C" if v.cpu_temp_c is not None else data.DASH,
            )
            s.body(
                1,
                "Mem free",
                f"{v.mem_available_mib} MiB"
                if v.mem_available_mib is not None
                else data.DASH,
            )
            s.body(
                2, "Load", f"{v.load_1m:.2f}" if v.load_1m is not None else data.DASH
            )
            s.body(
                3,
                "Up",
                f"{v.uptime_s // 3600}h{(v.uptime_s % 3600) // 60:02d}"
                if v.uptime_s is not None
                else data.DASH,
            )

        @reg.info_page("CAN")
        def _can(s: Screen) -> None:
            c = self.can
            s.body(
                0,
                self.cfg.runtime.can_interface,
                ("UP" if c.get("up") else "DOWN") if c else "absent",
            )
            s.body(1, "Bus-off", str(c.get("bus-off", data.DASH)) if c else data.DASH)
            s.body(2, "Restarts", str(c.get("restarts", data.DASH)) if c else data.DASH)
            s.body(3, "PCB", "ok" if self.heartbeat.state.pcb_ok else "no link")

        @reg.info_page("Joints")
        def _joints(s: Screen) -> None:
            a = self.watch.view.angles_deg + [0.0] * 6
            s.body(0, f"J0 {a[0]:.1f}", f"J1 {a[1]:.1f}")
            s.body(1, f"J2 {a[2]:.1f}", f"J3 {a[3]:.1f}")
            s.body(2, f"J4 {a[4]:.1f}", f"J5 {a[5]:.1f}")

        reg.number("Contrast", "contrast", 0, 255, step=16)
        reg.number("Blink", "blink_s", 0.1, 5.0, step=0.1, fmt="{:.1f}s")
        reg.action(
            f"Restart {unit}", lambda: self.unit_action(["restart", unit]), confirm=True
        )
        reg.action(
            f"Stop {unit}", lambda: self.unit_action(["stop", unit]), confirm=True
        )
        reg.action("Reboot", lambda: self.power("reboot"), confirm=True)
        reg.action("Shutdown", lambda: self.power("poweroff"), confirm=True)

    def _home(self, s: Screen) -> None:
        v = self.watch.view
        live = v.fresh(time.monotonic())
        s.row(
            0,
            f"Mode {v.mode if live else 'no runtime'}",
            "EN" if (live and v.enabled) else "",
        )
        s.row(
            1,
            f"CAN {('UP' if self.can.get('up') else 'DOWN') if self.can else 'none'}",
            "ERR" if (live and v.error) else "",
        )
        s.row(2, f"IP {data.ip_address()}")
        temp = (
            f"{self.vitals.cpu_temp_c:.0f}C"
            if self.vitals.cpu_temp_c is not None
            else data.DASH
        )
        s.row(3, f"T {temp}", f"{self.cfg.runtime.unit} {self.unit_state}")

    # -- actions
    def unit_action(self, argv: list[str]) -> None:
        log.warning("menu action: systemctl %s", " ".join(argv))
        self._run_unit_action(argv)

    def power(self, command: str) -> None:
        """Stop the managed unit first, then hand the box to systemd."""
        log.warning(
            "%s requested from the panel — stopping %s first",
            command,
            self.cfg.runtime.unit,
        )
        self._run_unit_action(["stop", self.cfg.runtime.unit])
        self._run_unit_action([command])

    # -- loops
    def main_tick(self) -> None:
        self.button1.poll()
        self.button2.poll()
        self.heartbeat.poll_pcb()
        if self.clock() - self._last_blink >= float(self.settings.get("blink_s", 1.0)):
            self.heartbeat.tick(self.button1.is_down, self.button2.is_down)
            self._last_blink = self.clock()
        self.ui.idle_check()

    def render_once(self) -> Screen:
        wanted = int(self.settings.get("contrast", 255))
        if wanted != self._applied_contrast:
            try:
                self.display.contrast(wanted)
                self._applied_contrast = wanted
            except Exception as exc:  # noqa: BLE001 — a display that refuses contrast keeps showing
                log.error("contrast not applied: %s", exc)
        screen = self.ui.render()
        try:
            self.display.show(screen.img)
        except Exception as exc:  # noqa: BLE001 — a display fault must not stop the panel
            log.error("display update failed: %s", exc)
        return screen

    def monitor_once(self) -> None:
        self.vitals = data.Vitals.sample()
        self.unit_state = data.unit_active(self.cfg.runtime.unit)
        current = data.can_link_stats(self.cfg.runtime.can_interface)
        if current is None:
            self.can = None
        else:
            if self.can is None:
                log.info("[CAN link] baseline %s", current)
            else:
                change = data.describe_can_change(self.can, current)
                if change:
                    log.warning("[CAN link] %s", change)
            self.can = current

    def _display_loop(self) -> None:
        while not self._stop.is_set():
            self.render_once()
            time.sleep(self.cfg.display.refresh_s)

    def _monitor_loop(self) -> None:
        while not self._stop.is_set():
            try:
                self.monitor_once()
            except Exception as exc:  # noqa: BLE001 — a monitor pass that fails is logged and retried
                log.error("link monitor pass failed: %s", exc)
            time.sleep(self.cfg.loop.link_monitor_s)

    def run(self) -> None:
        self.watch.start()
        threading.Thread(
            target=self._display_loop, name="par6-panel-display", daemon=True
        ).start()
        threading.Thread(
            target=self._monitor_loop, name="par6-panel-monitor", daemon=True
        ).start()
        period = self.cfg.loop.period_s
        try:
            while not self._stop.is_set():
                start = self.clock()
                self.main_tick()
                elapsed = self.clock() - start
                time.sleep(max(period - elapsed, 0.0))
        finally:
            self.shutdown()

    def stop(self) -> None:
        self._stop.set()

    def shutdown(self) -> None:
        self.watch.stop()
        for led in (self.led1, self.led2):
            try:
                led.set(False)
            except Exception:  # noqa: BLE001 — best effort on the way out
                pass
        self.heartbeat._link.close()
        log.info("panel shutdown complete")


def build(cfg: PanelConfig) -> PanelService:
    return PanelService(
        cfg,
        display=open_display(cfg),
        leds=open_leds(cfg),
        buttons=open_buttons(cfg),
        pcb=open_pcb(cfg),
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="PAR6 control box front panel")
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="panel.toml (default: PAR6_PANEL_CONFIG, then the packaged copy)",
    )
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=args.log_level, format="%(asctime)s %(levelname)s %(name)s: %(message)s"
    )
    cfg = load(args.config)
    service = build(cfg)
    signal.signal(signal.SIGTERM, lambda *_: service.stop())
    signal.signal(signal.SIGINT, lambda *_: service.stop())
    service.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
