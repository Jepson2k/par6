"""The OLED UI: a small state machine over five modes and one rule.

Tap = move, hold = select/enter, hold button 1 = back. HOME (dashboard
plus a tab bar), INFO (a carousel of registered pages), SETTINGS (a list
of registered items), EDIT (a number) and CONFIRM (destructive actions:
hold button 2 = yes, ANY tap = cancel — so a stray tap can never confirm).
Non-home screens fall back to HOME after ``idle_return_s`` without input.

Pages and settings are registries: an info page is a function taking a
:class:`Screen` decorated with :func:`info_page`, and its position in the
file is its position in the carousel; a setting is one constructor call
(:func:`number`, :func:`toggle`, :func:`action`) in display order. The
renderer draws with PIL into a 1-bit image the display driver shows; it
also draws for tests, which look at the pixels rather than the glass.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable

from PIL import Image, ImageDraw, ImageFont

SECTIONS = ("Home", "Info", "Settings")
LINE_H = 12
BAR_H = 13
ROWS_FULL = 5
ROWS_HOME = 4


def _font() -> ImageFont.ImageFont | ImageFont.FreeTypeFont:
    for name in ("DejaVuSans.ttf", "DejaVuSans", "LiberationSans-Regular.ttf"):
        try:
            return ImageFont.truetype(name, 11)
        except OSError:
            continue
    return ImageFont.load_default()


FONT = _font()


class Screen:
    """One 1-bit frame, ``width × height``, drawn row by row."""

    def __init__(self, width: int = 128, height: int = 64) -> None:
        self.width = width
        self.height = height
        self.img = Image.new("1", (width, height))
        self._draw = ImageDraw.Draw(self.img)
        self.lines: list[str] = []

    def clear(self) -> "Screen":
        self._draw.rectangle((0, 0, self.width, self.height), fill=0)
        self.lines = []
        return self

    def _text(self, x: int, y: int, s: str, fill: int = 255) -> None:
        self._draw.text((x, y), s, font=FONT, fill=fill)

    def _w(self, s: str) -> int:
        return int(self._draw.textlength(s, font=FONT))

    def row(
        self, i: int, left: str, right: str | None = None, selected: bool = False
    ) -> None:
        y = i * LINE_H
        if selected:
            self._draw.rectangle((0, y, self.width, y + LINE_H - 1), fill=255)
        fill = 0 if selected else 255
        self._text(2, y, left, fill)
        if right is not None:
            self._text(self.width - self._w(right) - 2, y, right, fill)
        self.lines.append(f"{left} {right}".strip() if right else left)

    def title_bar(self, left: str, right: str | None = None) -> None:
        self._draw.rectangle((0, 0, self.width, LINE_H - 1), fill=255)
        self._text(2, 0, left, 0)
        if right is not None:
            self._text(self.width - self._w(right) - 2, 0, right, 0)
        self.lines.append(f"[{left}] {right or ''}".strip())

    def body(self, i: int, left: str, right: str | None = None) -> None:
        self.row(i + 1, left, right)

    def list_view(
        self,
        items: list[dict[str, Any]],
        selected: int,
        value_of: Callable[[dict], str],
    ) -> None:
        visible = ROWS_FULL - 1
        top = max(0, min(selected - visible + 1, len(items) - visible))
        for k, item in enumerate(items[top : top + visible]):
            self.row(
                k + 1, item["label"], value_of(item), selected=(top + k) == selected
            )

    def edit_view(self, label: str, value_text: str) -> None:
        self.title_bar("EDIT")
        self.body(0, label)
        self.body(1, f"< {value_text} >")
        self.body(3, "hold B2 = save, hold B1 = cancel")

    def confirm_view(self, label: str) -> None:
        self.title_bar("CONFIRM")
        self.body(0, label + "?")
        self.body(2, "hold B2 = YES")
        self.body(3, "any tap = cancel")

    def tabbar(self, labels: tuple[str, ...], selected: int) -> None:
        y = self.height - BAR_H
        self._draw.line((0, y, self.width, y), fill=255)
        w = self.width // len(labels)
        for k, label in enumerate(labels):
            x = k * w
            if k == selected:
                self._draw.rectangle((x, y + 1, x + w - 1, self.height), fill=255)
            self._text(x + 3, y + 1, label, 0 if k == selected else 255)
        self.lines.append(
            "tabs: "
            + " ".join(f"[{s}]" if k == selected else s for k, s in enumerate(labels))
        )


# ---------------------------------------------------------------- registries


@dataclass
class Registries:
    """Pages and settings, plus the values the settings edit."""

    info_pages: list[tuple[str, Callable[[Screen], None]]] = field(default_factory=list)
    settings: list[dict[str, Any]] = field(default_factory=list)
    values: dict[str, Any] = field(default_factory=dict)
    on_save: Callable[[dict[str, Any]], None] | None = None

    def info_page(
        self, title: str
    ) -> Callable[[Callable[[Screen], None]], Callable[[Screen], None]]:
        def wrap(fn: Callable[[Screen], None]) -> Callable[[Screen], None]:
            self.info_pages.append((title, fn))
            return fn

        return wrap

    def number(
        self,
        label: str,
        key: str,
        lo: float,
        hi: float,
        step: float,
        fmt: str = "{:.0f}",
    ) -> None:
        self.settings.append(
            {
                "label": label,
                "type": "number",
                "key": key,
                "min": lo,
                "max": hi,
                "step": step,
                "fmt": fmt,
            }
        )

    def toggle(self, label: str, key: str) -> None:
        self.settings.append({"label": label, "type": "toggle", "key": key})

    def action(
        self, label: str, run: Callable[[], None], confirm: bool = False
    ) -> None:
        self.settings.append(
            {"label": label, "type": "action", "run": run, "confirm": confirm}
        )

    def value_str(self, item: dict[str, Any]) -> str:
        if item["type"] == "toggle":
            return "ON" if self.values.get(item["key"]) else "OFF"
        if item["type"] == "number":
            return item["fmt"].format(self.values.get(item["key"], 0))
        return ">"

    def save(self) -> None:
        if self.on_save is not None:
            self.on_save(dict(self.values))


# ---------------------------------------------------------------- the machine


class PanelUi:
    """Modes, cursors and the two buttons' meaning in each mode."""

    def __init__(
        self,
        registries: Registries,
        render_home: Callable[[Screen], None],
        *,
        idle_return_s: float,
        width: int = 128,
        height: int = 64,
        clock: Callable[[], float] = time.perf_counter,
    ) -> None:
        self.reg = registries
        self._render_home = render_home
        self._idle_return_s = idle_return_s
        self._clock = clock
        self.width = width
        self.height = height
        self.mode = "HOME"
        self.tab = 0
        self.info_page = 0
        self.set_index = 0
        self.edit_value: float = 0.0
        self.confirm: dict[str, Any] | None = None
        self.last_input = clock()

    # -- input: tap = move
    def _touch(self) -> None:
        self.last_input = self._clock()

    def on_b1_short(self) -> None:
        self._touch()
        if self.mode == "HOME":
            self.tab = (self.tab - 1) % len(SECTIONS)
        elif self.mode == "INFO" and self.reg.info_pages:
            self.info_page = (self.info_page - 1) % len(self.reg.info_pages)
        elif self.mode == "SETTINGS" and self.reg.settings:
            self.set_index = (self.set_index - 1) % len(self.reg.settings)
        elif self.mode == "EDIT":
            self._edit_step(-1)
        elif self.mode == "CONFIRM":
            self._cancel_confirm()

    def on_b2_short(self) -> None:
        self._touch()
        if self.mode == "HOME":
            self.tab = (self.tab + 1) % len(SECTIONS)
        elif self.mode == "INFO" and self.reg.info_pages:
            self.info_page = (self.info_page + 1) % len(self.reg.info_pages)
        elif self.mode == "SETTINGS" and self.reg.settings:
            self.set_index = (self.set_index + 1) % len(self.reg.settings)
        elif self.mode == "EDIT":
            self._edit_step(+1)
        elif self.mode == "CONFIRM":
            self._cancel_confirm()

    # -- input: hold = select / back
    def on_b2_long(self) -> None:
        self._touch()
        if self.mode == "HOME":
            section = SECTIONS[self.tab]
            if section == "Info":
                self.mode = "INFO"
                self.info_page = 0
            elif section == "Settings":
                self.mode = "SETTINGS"
                self.set_index = 0
        elif self.mode == "SETTINGS" and self.reg.settings:
            self._activate(self.reg.settings[self.set_index])
        elif self.mode == "EDIT":
            item = self.reg.settings[self.set_index]
            self.reg.values[item["key"]] = self.edit_value
            self.reg.save()
            self.mode = "SETTINGS"
        elif self.mode == "CONFIRM":
            item = self.confirm
            self.confirm = None
            self.mode = "SETTINGS"
            if item:
                item["run"]()

    def on_b1_long(self) -> None:
        self._touch()
        if self.mode == "EDIT":
            self.mode = "SETTINGS"
        elif self.mode == "CONFIRM":
            self._cancel_confirm()
        elif self.mode in ("INFO", "SETTINGS"):
            self.mode = "HOME"

    def _cancel_confirm(self) -> None:
        self.confirm = None
        self.mode = "SETTINGS"

    def _activate(self, item: dict[str, Any]) -> None:
        if item["type"] == "toggle":
            self.reg.values[item["key"]] = not self.reg.values.get(item["key"])
            self.reg.save()
        elif item["type"] == "number":
            self.edit_value = float(self.reg.values.get(item["key"], item["min"]))
            self.mode = "EDIT"
        elif item["type"] == "action":
            if item.get("confirm"):
                self.confirm = item
                self.mode = "CONFIRM"
            else:
                item["run"]()

    def _edit_step(self, direction: int) -> None:
        item = self.reg.settings[self.set_index]
        value = self.edit_value + direction * item["step"]
        self.edit_value = round(max(item["min"], min(item["max"], value)), 6)

    def idle_check(self) -> None:
        if (
            self.mode != "HOME"
            and self._clock() - self.last_input > self._idle_return_s
        ):
            self.mode = "HOME"

    # -- render
    def render(self) -> Screen:
        s = Screen(self.width, self.height).clear()
        if self.mode == "INFO":
            n = len(self.reg.info_pages)
            if n:
                idx = self.info_page % n
                title, render_page = self.reg.info_pages[idx]
                s.title_bar(title, f"{idx + 1}/{n}")
                render_page(s)
            else:
                s.title_bar("INFO")
                s.body(1, "(no pages)")
        elif self.mode == "SETTINGS":
            s.title_bar("SETTINGS")
            if self.reg.settings:
                s.list_view(self.reg.settings, self.set_index, self.reg.value_str)
            else:
                s.body(1, "(no settings)")
        elif self.mode == "EDIT":
            item = self.reg.settings[self.set_index]
            s.edit_view(item["label"], item["fmt"].format(self.edit_value))
        elif self.mode == "CONFIRM":
            s.confirm_view(self.confirm["label"] if self.confirm else "Confirm")
        else:
            self._render_home(s)
            s.tabbar(SECTIONS, self.tab)
        return s
