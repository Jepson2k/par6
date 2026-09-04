"""The heartbeat, the two LEDs and the UART link to the mainboard PCB.

One blink tick does four things in order: sends the PCB its heartbeat,
toggles the LEDs anti-phase (a stuck panel is then visually obvious —
two LEDs both on, or both off, never happen while it runs), publishes
the panel state, and drives the physical LEDs FROM the published state,
so what the box shows is by construction what the runtime can read.

If the UART will not open, or dies later, PCB comms are disabled and
logged; the display and the buttons keep working. Every write is
guarded for the same reason.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Callable, Protocol

log = logging.getLogger(__name__)


class Led(Protocol):
    def set(self, on: bool) -> None: ...


class SerialPort(Protocol):
    """The slice of ``serial.Serial`` the link uses."""

    @property
    def in_waiting(self) -> int: ...

    def readline(self) -> bytes: ...

    def write(self, data: bytes) -> int | None: ...

    def close(self) -> None: ...


@dataclass
class PanelState:
    """What the panel publishes, and what the LEDs are driven from."""

    led1: bool = False
    led2: bool = True
    button1_down: bool = False
    button2_down: bool = False
    pcb_ok: bool = False
    pcb_data: list[int] = field(default_factory=list)
    heartbeats: int = 0


class PcbLink:
    """The mainboard PCB over UART: ``$a b c d`` lines in, heartbeat out."""

    def __init__(
        self,
        open_port: Callable[[], SerialPort],
        *,
        fields: int,
        heartbeat: str,
    ) -> None:
        self._fields = fields
        self._heartbeat = heartbeat.encode("utf-8")
        self.port: SerialPort | None = None
        try:
            self.port = open_port()
        except Exception as exc:  # noqa: BLE001 — any failure to open degrades to no PCB comms
            log.error("PCB UART could not be opened: %s — PCB comms disabled", exc)

    @property
    def enabled(self) -> bool:
        return self.port is not None

    def _disable(self, why: Exception) -> None:
        log.error("PCB UART failed: %s — PCB comms disabled", why)
        try:
            if self.port is not None:
                self.port.close()
        except Exception:  # noqa: BLE001 — closing a dead port must not raise
            pass
        self.port = None

    def read(self) -> list[int] | None:
        """One PCB frame if a complete, well-formed line is waiting."""
        if self.port is None:
            return None
        try:
            if self.port.in_waiting <= 0:
                return None
            line = self.port.readline().decode("utf-8", errors="ignore").strip()
        except Exception as exc:  # noqa: BLE001 — a dying port disables the link, never the panel
            self._disable(exc)
            return None
        if not line.startswith("$"):
            return None
        parts = line[1:].split()
        if len(parts) != self._fields:
            return None
        try:
            return [int(p) for p in parts]
        except ValueError:
            return None

    def send_heartbeat(self) -> bool:
        if self.port is None:
            return False
        try:
            self.port.write(self._heartbeat)
            return True
        except Exception as exc:  # noqa: BLE001 — a dying port disables the link, never the panel
            self._disable(exc)
            return False

    def close(self) -> None:
        if self.port is not None:
            try:
                self.port.close()
            except Exception:  # noqa: BLE001 — best effort on the way out
                pass
            self.port = None


class Heartbeat:
    """The blink tick; ``publish`` receives the state the LEDs are then
    driven from."""

    def __init__(
        self,
        link: PcbLink,
        led1: Led,
        led2: Led,
        *,
        publish: Callable[[PanelState], None] | None = None,
    ) -> None:
        self._link = link
        self._led1 = led1
        self._led2 = led2
        self._publish = publish
        self.state = PanelState()

    def tick(self, button1_down: bool, button2_down: bool) -> PanelState:
        sent = self._link.send_heartbeat()
        s = self.state
        s.led1 = not s.led1
        s.led2 = not s.led1
        s.button1_down = button1_down
        s.button2_down = button2_down
        s.pcb_ok = sent
        s.heartbeats += 1
        if self._publish is not None:
            self._publish(s)
        try:
            self._led1.set(s.led1)
            self._led2.set(s.led2)
        except Exception as exc:  # noqa: BLE001 — an LED that fails must not stop the loop
            log.error("LED write failed: %s", exc)
        return s

    def poll_pcb(self) -> None:
        frame = self._link.read()
        if frame is not None:
            self.state.pcb_data = frame
