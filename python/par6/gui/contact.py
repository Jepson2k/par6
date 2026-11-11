"""The Contact panel: what the arm stops for, and how close it has come.

Contact detection ships with a reference arm's thresholds, and the README's
tuning procedure is a loop: run your own programs with nothing near the
arm, read how far each joint's signal got, set each threshold at roughly
twice that. This panel is that loop with the numbers in front of you — the
live signal on both channels against its threshold, the peaks since the
last ``reset()``, and the thresholds themselves, editable while the arm
runs.

What is applied here lives until the runtime restarts. The snippet at the
bottom is the same values as the ``[contact]`` table the arm's local
overlay takes, which is where they are kept.
"""

from __future__ import annotations

import logging
from typing import cast

from nicegui import ui
from waldoctl import Commander, Panel, PanelSlot

from par6.client import AsyncRobotClient, ContactReadout

logger = logging.getLogger(__name__)

REFRESH_S = 0.5

REACTIONS = {
    "stop": "Stop, back off and hold",
    "warn": "Report only",
    "off": "Off",
}

#: What "Set from peaks" multiplies a peak by, and the least it will set a
#: threshold to: a joint that has not moved yet has a peak of nothing.
PEAK_MARGIN = 2.0
THRESHOLD_FLOOR_NM = 0.05


def _describe(err: BaseException) -> str:
    """A runtime refusal, with the part that says why."""
    cause = getattr(err, "cause", "")
    if not cause:
        return str(err)
    return f"{getattr(err, 'title', str(err))}: {cause}"


def overlay_snippet(reaction: str, sustained: list[float], impact: list[float]) -> str:
    """The ``[contact]`` table a local overlay takes for these values."""

    def row(values: list[float]) -> str:
        return "[" + ", ".join(f"{v:g}" for v in values) + "]"

    return (
        "[contact]\n"
        f'reaction = "{reaction}"\n'
        f"sustained_nm = {row(sustained)}\n"
        f"impact_nm = {row(impact)}\n"
    )


def from_peaks(peaks: list[float]) -> list[float]:
    """Thresholds a run with nothing in the way suggests: its peaks, with
    margin, rounded to what a config file would carry."""
    return [round(max(PEAK_MARGIN * peak, THRESHOLD_FLOOR_NM), 2) for peak in peaks]


class ContactPanel(Panel):
    id = "par6-contact"
    display_name = "Contact"
    slot = PanelSlot.LEFT_TOP_TAB
    tab_icon = "front_hand"
    tab_tooltip = "Contact detection: signals, peaks and thresholds"
    order = 61
    min_width = 460
    min_height = 320
    default_width = 620
    default_height = 440
    resizable = True

    def __init__(self) -> None:
        self._reset_element_refs()

    def _reset_element_refs(self) -> None:
        self._commander: Commander | None = None
        self._readout: ContactReadout | None = None
        self._seeded = False
        self._busy = False
        self._reaction: ui.select | None = None
        self._body: ui.column | None = None
        self._cells: list[dict[str, ui.label]] = []
        self._sustained: list[ui.number] = []
        self._impact: list[ui.number] = []
        self._trips: ui.label | None = None
        self._note: ui.label | None = None
        self._snippet: ui.code | None = None

    def applies_to(self, commander: Commander) -> bool:
        return commander.robot.backend_package == "par6"

    @property
    def _client(self) -> AsyncRobotClient | None:
        """par6's own client: ``applies_to`` admits only the par6 backend,
        and the contact readout is not on the generic ABC."""
        if self._commander is None:
            return None
        return cast(AsyncRobotClient, self._commander.client)

    # ------------------------------------------------------------------
    # build
    # ------------------------------------------------------------------

    def build(self, commander: Commander) -> None:
        self._reset_element_refs()
        self._commander = commander

        with ui.column().classes("w-full gap-2"):
            with ui.row().classes("w-full items-center"):
                ui.label("Contact detection").classes("text-subtitle1")
                ui.space()
                self._reaction = (
                    ui.select(REACTIONS, label="On contact")
                    .props("dense outlined")
                    .classes("w-56")
                    .mark("contact-reaction")
                )
            with (
                ui.card().tight().classes("w-full"),
                ui.column().classes("w-full p-2 gap-1"),
            ):
                with ui.row().classes(
                    "w-full items-center no-wrap text-caption text-grey"
                ):
                    ui.label("Joint").classes("w-10")
                    ui.label("Signal").classes("w-16 text-right")
                    ui.label("Peak").classes("w-16 text-right")
                    ui.label("Sustained limit").classes("w-28")
                    ui.label("Impact").classes("w-16 text-right")
                    ui.label("Peak").classes("w-16 text-right")
                    ui.label("Impact limit").classes("w-28")
                self._body = ui.column().classes("w-full gap-1").mark("contact-rows")
            with ui.row().classes("w-full items-center gap-2"):
                self._trips = ui.label("").mark("contact-trips")
                ui.space()
                ui.button("Restart peaks", on_click=self._restart_peaks).props(
                    "flat dense"
                ).tooltip(
                    "Sends reset(): the peaks start over, and a standing "
                    "protective stop is cleared"
                ).mark("contact-restart-peaks")
                ui.button("Set from peaks", on_click=self._fill_from_peaks).props(
                    "flat dense"
                ).tooltip(
                    f"Fill each limit with {PEAK_MARGIN:g}x its peak; nothing is "
                    "applied until Apply"
                ).mark("contact-from-peaks")
                ui.button("Apply", icon="publish", on_click=self._apply).props(
                    "unelevated dense"
                ).mark("contact-apply")
            self._note = (
                ui.label("").classes("text-caption text-grey").mark("contact-note")
            )
            ui.label(
                "Signals and limits are Nm at the joint. A peak is the largest "
                "signal seen while the arm was being driven, since the last "
                "reset(). Applied limits live until the runtime restarts; keep "
                "them in the arm's local.toml:"
            ).classes("text-caption text-grey")
            self._snippet = (
                ui.code("", language="toml").classes("w-full").mark("contact-snippet")
            )

        ui.timer(REFRESH_S, self._refresh)

    def _build_rows(self, joints: int) -> None:
        if self._body is None:
            return
        self._body.clear()
        self._cells, self._sustained, self._impact = [], [], []
        with self._body:
            for j in range(joints):
                with ui.row().classes("w-full items-center no-wrap"):
                    ui.label(f"J{j + 1}").classes("w-10")
                    cells = {
                        name: ui.label("")
                        .classes("w-16 text-right")
                        .mark(f"contact-{name}-{j}")
                        for name in ("signal", "peak-signal")
                    }
                    self._sustained.append(
                        ui.number(format="%g", min=0, step=0.05, on_change=self._edited)
                        .props("dense outlined suffix=Nm")
                        .classes("w-28")
                        .mark(f"contact-sustained-{j}")
                    )
                    cells.update(
                        {
                            name: ui.label("")
                            .classes("w-16 text-right")
                            .mark(f"contact-{name}-{j}")
                            for name in ("impact", "peak-impact")
                        }
                    )
                    self._impact.append(
                        ui.number(format="%g", min=0, step=0.05, on_change=self._edited)
                        .props("dense outlined suffix=Nm")
                        .classes("w-28")
                        .mark(f"contact-impact-limit-{j}")
                    )
                    self._cells.append(cells)

    # ------------------------------------------------------------------
    # live readout
    # ------------------------------------------------------------------

    async def _refresh(self) -> None:
        client = self._client
        if client is None or self._busy:
            return
        try:
            readout = await client.contact()
        except Exception as err:
            # A panel that polls must outlive a runtime that is restarting.
            logger.debug("contact readout unavailable: %s", err)
            return
        if readout is None:
            return
        self._readout = readout
        if len(self._cells) != len(readout.signal):
            self._build_rows(len(readout.signal))
            self._seeded = False
        if not self._seeded:
            self._seed(readout)
        for j, cells in enumerate(self._cells):
            self._show(cells["signal"], readout.signal[j], readout.sustained_nm[j])
            self._show(
                cells["peak-signal"], readout.peak_signal[j], readout.sustained_nm[j]
            )
            self._show(cells["impact"], readout.impact[j], readout.impact_nm[j])
            self._show(
                cells["peak-impact"], readout.peak_impact[j], readout.impact_nm[j]
            )
        if self._trips is not None:
            last = (
                ""
                if readout.last_trip_joint is None
                else f" — last: J{readout.last_trip_joint + 1}, "
                f"{readout.last_trip_channel}"
            )
            self._trips.text = f"Stops: {readout.trips}{last}"

    @staticmethod
    def _show(label: ui.label, value: float, limit: float) -> None:
        """A reading, coloured by how much of its limit it uses."""
        label.text = f"{abs(value):.2f}"
        share = abs(value) / limit if limit > 0 else 0.0
        label.classes(
            replace="w-16 text-right "
            + (
                "text-negative"
                if share >= 1.0
                else "text-warning"
                if share >= 0.5
                else ""
            )
        )

    def _seed(self, readout: ContactReadout) -> None:
        """Put the runtime's own values in the fields: once, so an edit in
        progress is not overwritten by the next poll."""
        for field, value in zip(self._sustained, readout.sustained_nm):
            field.value = value
        for field, value in zip(self._impact, readout.impact_nm):
            field.value = value
        if self._reaction is not None:
            self._reaction.value = readout.reaction
        self._seeded = True
        self._edited()

    def _fields(self) -> tuple[str, list[float], list[float]] | None:
        """What the fields hold, or None while one of them holds nothing."""
        if self._reaction is None or self._reaction.value not in REACTIONS:
            return None
        try:
            sustained = [float(field.value) for field in self._sustained]
            impact = [float(field.value) for field in self._impact]
        except (TypeError, ValueError):
            return None
        return str(self._reaction.value), sustained, impact

    def _edited(self, _event: object = None) -> None:
        fields = self._fields()
        if self._snippet is not None and fields is not None:
            self._snippet.content = overlay_snippet(*fields)
        readout = self._readout
        if self._note is None or readout is None or fields is None:
            return
        pending = fields != (
            readout.reaction,
            list(readout.sustained_nm),
            list(readout.impact_nm),
        )
        self._note.text = "Edited, not applied" if pending else ""

    # ------------------------------------------------------------------
    # writes
    # ------------------------------------------------------------------

    def _fill_from_peaks(self) -> None:
        readout = self._readout
        if readout is None:
            ui.notify("No readout from the runtime yet", color="warning")
            return
        if not any(readout.peak_signal):
            ui.notify(
                "No peaks yet: run a program with nothing near the arm first",
                color="warning",
            )
            return
        for field, value in zip(self._sustained, from_peaks(readout.peak_signal)):
            field.value = value
        for field, value in zip(self._impact, from_peaks(readout.peak_impact)):
            field.value = value
        self._edited()

    async def _apply(self) -> None:
        client = self._client
        fields = self._fields()
        if client is None:
            return
        if fields is None:
            ui.notify("Every limit must hold a number", color="warning")
            return
        reaction, sustained, impact = fields
        self._busy = True
        try:
            await client.set_contact_limits(
                reaction=reaction, sustained_nm=sustained, impact_nm=impact
            )
        except Exception as err:
            # The runtime's refusal is the answer; show it rather than
            # leave the fields looking applied.
            ui.notify(f"Refused: {_describe(err)}", color="negative", multi_line=True)
            if self._note is not None:
                self._note.text = _describe(err)
            return
        finally:
            self._busy = False
        self._seeded = False
        ui.notify("Contact limits applied", color="positive")

    async def _restart_peaks(self) -> None:
        client = self._client
        if client is None:
            return
        try:
            await client.reset()
        except Exception as err:
            ui.notify(f"Reset failed: {_describe(err)}", color="negative")
