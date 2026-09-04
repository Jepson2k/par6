"""Polled push buttons with debounce and short/long press detection.

Called every main-loop iteration. A level must hold for the debounce
time before it counts; a short press fires on RELEASE and only if the
hold never crossed the long-press time; a long press fires ONCE the
moment the hold crosses it, while the button is still held — so a hold
answers without waiting for the finger to lift, and a release after a
long press is never mistaken for a tap.
"""

from __future__ import annotations

import time
from typing import Callable


class ButtonHandler:
    """One button: ``pressed`` reads the debounced-raw level (True = down);
    ``clock`` is injectable so the timings are testable."""

    def __init__(
        self,
        pressed: Callable[[], bool],
        *,
        long_press_s: float,
        debounce_s: float,
        on_short: Callable[[], None] | None = None,
        on_long: Callable[[], None] | None = None,
        clock: Callable[[], float] = time.perf_counter,
    ) -> None:
        self._pressed = pressed
        self._long_press_s = long_press_s
        self._debounce_s = debounce_s
        self._on_short = on_short
        self._on_long = on_long
        self._clock = clock
        self._raw = False
        self._stable = False
        self._last_change = clock()
        self._press_start = 0.0
        self._long_fired = False

    @property
    def is_down(self) -> bool:
        """The debounced level."""
        return self._stable

    def poll(self) -> None:
        now = self._clock()
        raw = bool(self._pressed())
        if raw != self._raw:
            self._raw = raw
            self._last_change = now
            return
        if now - self._last_change < self._debounce_s:
            return
        if raw and not self._stable:
            self._stable = True
            self._press_start = now
            self._long_fired = False
        elif raw and self._stable:
            if not self._long_fired and now - self._press_start >= self._long_press_s:
                self._long_fired = True
                if self._on_long:
                    self._on_long()
        elif not raw and self._stable:
            self._stable = False
            if not self._long_fired and self._on_short:
                self._on_short()
