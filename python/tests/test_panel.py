"""The front panel without its hardware: the buttons' tap/hold grammar,
the UI's one rule and its asymmetric confirm, the heartbeat that drives
the LEDs from what it published, a PCB link that dies without taking the
panel with it, the settings store, the config's refusal of a wired
mistake, and a preflight that reports rather than acts."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from par6.panel import config as panel_config
from par6.panel.buttons import ButtonHandler
from par6.panel.link import Heartbeat, PcbLink
from par6.panel.service import (
    DEFAULT_SETTINGS,
    NullDisplay,
    NullLed,
    PanelService,
    load_settings,
    save_settings,
)
from par6.panel.ui import SECTIONS, PanelUi, Registries, Screen

PANEL_TOML = Path(__file__).resolve().parents[2] / "config" / "panel.toml"


class Clock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


class Pin:
    def __init__(self) -> None:
        self.down = False

    def __call__(self) -> bool:
        return self.down


def press(
    handler: ButtonHandler, pin: Pin, clock: Clock, hold_s: float, step: float = 0.01
) -> None:
    """Press, hold for ``hold_s`` polling every ``step``, release, and let
    the release debounce."""
    pin.down = True
    handler.poll()
    end = clock.t + hold_s
    while clock.t < end:
        clock.advance(step)
        handler.poll()
    pin.down = False
    handler.poll()
    clock.advance(0.05)
    handler.poll()
    clock.advance(step)
    handler.poll()


def test_a_button_tells_taps_from_holds_and_ignores_bounces():
    clock, pin = Clock(), Pin()
    events: list[str] = []
    h = ButtonHandler(
        pin,
        long_press_s=0.6,
        debounce_s=0.03,
        on_short=lambda: events.append("short"),
        on_long=lambda: events.append("long"),
        clock=clock,
    )
    # A 10 ms glitch never counts.
    pin.down = True
    h.poll()
    clock.advance(0.01)
    pin.down = False
    h.poll()
    clock.advance(0.05)
    h.poll()
    assert events == []
    # A tap fires on release, once.
    press(h, pin, clock, 0.2)
    assert events == ["short"]
    # A hold fires once the moment it crosses the threshold, while held,
    # and the release afterwards is not a tap.
    pin.down = True
    h.poll()
    clock.advance(0.05)
    h.poll()
    clock.advance(0.5)
    h.poll()
    assert events == ["short"], "not long yet"
    clock.advance(0.1)
    h.poll()
    assert events == ["short", "long"], "fired while still held"
    clock.advance(1.0)
    h.poll()
    pin.down = False
    h.poll()
    clock.advance(0.05)
    h.poll()
    assert events == ["short", "long"], "one long per hold, no tap on release"


def registries_with_action() -> tuple[Registries, list[str], list[dict]]:
    ran: list[str] = []
    saved: list[dict] = []
    reg = Registries(values={"contrast": 255, "blink_s": 1.0}, on_save=saved.append)
    reg.number("Contrast", "contrast", 0, 255, step=16)
    reg.toggle("Simulate", "simulate")
    reg.action("Restart", lambda: ran.append("restart"))
    reg.action("Shutdown", lambda: ran.append("shutdown"), confirm=True)

    @reg.info_page("Network")
    def _net(s: Screen) -> None:
        s.body(0, "IP", "10.0.0.2")

    @reg.info_page("System")
    def _sys(s: Screen) -> None:
        s.body(0, "Temp", "41C")

    return reg, ran, saved


def test_the_ui_moves_on_taps_selects_on_holds_and_never_confirms_on_a_tap():
    clock = Clock()
    reg, ran, saved = registries_with_action()
    ui = PanelUi(reg, lambda s: s.row(0, "home"), idle_return_s=12.0, clock=clock)
    assert ui.mode == "HOME"
    ui.on_b2_short()
    assert SECTIONS[ui.tab] == "Info"
    ui.on_b2_long()
    assert ui.mode == "INFO"
    assert "Network" in ui.render().lines[0]
    ui.on_b2_short()
    assert "System" in ui.render().lines[0], "tap moves through the carousel"
    ui.on_b1_long()
    assert ui.mode == "HOME", "hold B1 is back"
    assert SECTIONS[ui.tab] == "Info", "back keeps the tab"

    ui.on_b2_short()
    assert SECTIONS[ui.tab] == "Settings"
    ui.on_b2_long()
    assert ui.mode == "SETTINGS"
    ui.on_b2_long()
    assert ui.mode == "EDIT" and ui.edit_value == 255
    ui.on_b1_short()
    assert ui.edit_value == 239, "tap steps the number"
    ui.on_b2_long()
    assert ui.mode == "SETTINGS" and reg.values["contrast"] == 239 and saved
    ui.on_b2_short()
    ui.on_b2_long()
    assert reg.values["simulate"] is True, "hold toggles"
    ui.on_b2_short()
    ui.on_b2_long()
    assert ran == ["restart"], "an unguarded action runs at once"

    ui.on_b2_short()
    ui.on_b2_long()
    assert ui.mode == "CONFIRM" and ran == ["restart"]
    ui.on_b2_short()
    assert ui.mode == "SETTINGS" and ran == ["restart"], "a tap on B2 cancels"
    ui.on_b2_long()
    assert ui.mode == "CONFIRM"
    ui.on_b1_short()
    assert ui.mode == "SETTINGS" and ran == ["restart"], "a tap on B1 cancels"
    ui.on_b2_long()
    ui.on_b2_long()
    assert ran == ["restart", "shutdown"], "only a hold confirms"

    clock.advance(12.1)
    ui.idle_check()
    assert ui.mode == "HOME", "idle falls back to Home"


class FakeLed:
    def __init__(self) -> None:
        self.history: list[bool] = []

    def set(self, on: bool) -> None:
        self.history.append(on)


class FakeSerial:
    def __init__(
        self, lines: list[bytes] | None = None, die_after: int | None = None
    ) -> None:
        self.lines = list(lines or [])
        self.written: list[bytes] = []
        self.die_after = die_after
        self.closed = False

    @property
    def in_waiting(self) -> int:
        return 1 if self.lines else 0

    def readline(self) -> bytes:
        return self.lines.pop(0)

    def write(self, data: bytes) -> int:
        if self.die_after is not None and len(self.written) >= self.die_after:
            raise OSError("device gone")
        self.written.append(data)
        return len(data)

    def close(self) -> None:
        self.closed = True


def test_the_heartbeat_drives_the_leds_from_the_published_state_anti_phase():
    port = FakeSerial(lines=[b"$1 2 3 4\n", b"garbage\n", b"$1 2\n"])
    link = PcbLink(lambda: port, fields=4, heartbeat="heartbeat\n\r")
    led1, led2 = FakeLed(), FakeLed()
    published: list[tuple[bool, bool]] = []
    hb = Heartbeat(
        link, led1, led2, publish=lambda s: published.append((s.led1, s.led2))
    )
    hb.poll_pcb()
    assert hb.state.pcb_data == [1, 2, 3, 4]
    hb.poll_pcb()
    hb.poll_pcb()
    assert hb.state.pcb_data == [1, 2, 3, 4], "malformed lines are dropped"
    for _ in range(4):
        hb.tick(button1_down=False, button2_down=True)
    assert port.written == [b"heartbeat\n\r"] * 4
    assert all(a != b for a, b in published), "anti-phase, always"
    assert led1.history == [p[0] for p in published]
    assert led2.history == [p[1] for p in published]
    assert hb.state.pcb_ok and hb.state.button2_down


def test_a_dead_or_missing_uart_disables_pcb_comms_and_nothing_else():
    def refuse() -> FakeSerial:
        raise OSError("no such device")

    link = PcbLink(refuse, fields=4, heartbeat="hb")
    assert not link.enabled
    hb = Heartbeat(link, FakeLed(), FakeLed())
    s = hb.tick(False, False)
    assert s.pcb_ok is False and s.heartbeats == 1, "the tick still runs"

    dying = FakeSerial(die_after=2)
    link = PcbLink(lambda: dying, fields=4, heartbeat="hb")
    hb = Heartbeat(link, FakeLed(), FakeLed())
    results = [hb.tick(False, False).pcb_ok for _ in range(4)]
    assert results == [True, True, False, False]
    assert dying.closed and not link.enabled


def test_settings_survive_a_corrupt_or_hand_edited_file(tmp_path):
    path = tmp_path / "settings.json"
    assert load_settings(path, DEFAULT_SETTINGS) == DEFAULT_SETTINGS
    path.write_text("{not json")
    assert load_settings(path, DEFAULT_SETTINGS) == DEFAULT_SETTINGS
    path.write_text(json.dumps({"contrast": "128", "blink_s": "fast", "evil": 1}))
    loaded = load_settings(path, DEFAULT_SETTINGS)
    assert loaded == {"contrast": 128, "blink_s": 1.0}, (
        "coerced, bad kept default, junk dropped"
    )
    save_settings(path, {"contrast": 64, "blink_s": 0.5})
    assert json.loads(path.read_text()) == {"contrast": 64, "blink_s": 0.5}
    assert not list(tmp_path.glob("*.tmp")), "atomic: no temp file left behind"


def test_the_config_refuses_a_pin_claimed_twice():
    cfg = panel_config.parse(PANEL_TOML.read_text())
    assert cfg.buttons.button1_pin != cfg.leds.led1_pin
    bad = PANEL_TOML.read_text().replace("led2_pin = 20", "led2_pin = 26")
    with pytest.raises(ValueError, match="pin twice"):
        panel_config.parse(bad)
    with pytest.raises(ValueError, match="positive"):
        panel_config.parse(
            PANEL_TOML.read_text().replace("blink_s = 1.0", "blink_s = 0")
        )


def test_the_service_renders_and_ticks_without_any_hardware(tmp_path):
    text = PANEL_TOML.read_text().replace(
        'file = "/var/lib/par6/panel-settings.json"', f'file = "{tmp_path / "s.json"}"'
    )
    cfg = panel_config.parse(text)
    clock = Clock()
    pin1, pin2 = Pin(), Pin()
    actions: list[list[str]] = []
    display = NullDisplay()
    service = PanelService(
        cfg,
        display=display,
        leds=(NullLed(), NullLed()),
        buttons=(pin1, pin2),
        pcb=PcbLink(lambda: FakeSerial(), fields=4, heartbeat="hb"),
        connect=lambda: (_ for _ in ()).throw(OSError("no runtime")),
        run_unit_action=actions.append,
        clock=clock,
    )
    screen = service.render_once()
    assert display.frames == 1
    assert "no runtime" in screen.lines[0]
    for _ in range(60):
        clock.advance(0.02)
        service.main_tick()
    assert service.heartbeat.state.heartbeats == 1, "one blink per configured period"
    assert (
        service.published is not None
        and service.published.led1 != service.published.led2
    )

    # Home → Settings (two taps), enter (hold), down to "Restart par6d",
    # hold, and hold again to confirm: the action goes through systemd.
    press(service.button2, pin2, clock, 0.1)
    press(service.button2, pin2, clock, 0.1)
    press(service.button2, pin2, clock, 0.8)
    assert service.ui.mode == "SETTINGS"
    press(service.button2, pin2, clock, 0.1)
    press(service.button2, pin2, clock, 0.1)
    press(service.button2, pin2, clock, 0.8)
    assert service.ui.mode == "CONFIRM" and actions == []
    press(service.button1, pin1, clock, 0.1)
    assert service.ui.mode == "SETTINGS" and actions == [], "a tap cancelled"
    press(service.button2, pin2, clock, 0.8)
    press(service.button2, pin2, clock, 0.8)
    assert actions == [["restart", "par6d"]]
    service.power("poweroff")
    assert actions[-2:] == [["stop", "par6d"], ["poweroff"]], "stop the unit first"


def test_the_preflight_reports_without_acting(tmp_path, monkeypatch, capsys):
    from par6.panel import preflight

    monkeypatch.setenv("PAR6_PANEL_CONFIG", str(PANEL_TOML))
    rc = preflight.main(["--json", "--no-reexec"])
    out = json.loads(capsys.readouterr().out.strip().splitlines()[-1])
    names = {c["name"] for c in out["checks"]}
    assert {
        "CPU cores >= 4",
        "CAN can0 present",
        "GPIO chip access",
        "import par6.client",
    } <= names
    assert all("required" in c for c in out["checks"])
    assert rc == (1 if out["required_failures"] else 0)
    advisory = [c for c in out["checks"] if not c["required"]]
    assert advisory, "advisory checks are flagged as such"
