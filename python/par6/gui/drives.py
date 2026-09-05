"""The Drives panel: what the CAN drives report, and what may be written back.

Everything the arm's motion depends on lives on six drive boards, and the
generic status surface deliberately says little about them — a temperature
and a current, in a vocabulary any backend could answer in. This panel is
where the rest goes, and its organising line is not "diagnostics versus
configuration" (a temperature beside a current limit belongs to both) but
**read versus write**: readings are already streaming and cost nothing to
show, while writes are gated, drive-shaped, and can cook a motor.

Configuration over CAN is one-way. The drives answer no parameter reads on
this bus, so every field here is seeded from the runtime's stored config
and labelled as such — it is what par6 last pushed, not what the drive
says it holds. Reading parameters back, presets and calibration are UART
features, and UART is not brought out on an assembled arm; that work
belongs to the vendor's own tool over a bench connection.
"""

from __future__ import annotations

import asyncio
import logging
import tomllib
from pathlib import Path
from typing import Any, cast

from nicegui import ui
from waldoctl import Commander, Panel, PanelSlot

from par6.client import AsyncRobotClient, RobotClient
from par6.firmware import releases
from par6.firmware.flasher import FlashReport

logger = logging.getLogger(__name__)

REFRESH_S = 0.5

#: The ten values one ``set_pid_gains`` frame replaces, with their config
#: home and a label. The frame carries the whole tuple, so a partial write
#: would zero what it left out — which is why this is one list and one
#: Apply, not ten independent fields.
GAIN_FIELDS: tuple[tuple[str, str, str, str], ...] = (
    ("kpp", "gains", "Position P", ""),
    ("kpv", "gains", "Velocity P", ""),
    ("kiv", "gains", "Velocity I", ""),
    ("kpiq", "gains", "Current P", ""),
    ("kiiq", "gains", "Current I", ""),
    ("kp", "gains", "Impedance stiffness", ""),
    ("kd", "gains", "Impedance damping", ""),
    ("ilim_ma", "joint", "Current limit", "mA"),
    ("velocity_limit_ticks_s", "joint", "Velocity limit", "ticks/s"),
    ("voltage_limit_mv", "joint", "Voltage limit (0 = VBUS)", "mV"),
)

#: Modes in which a drive will accept commissioning. Anything else and the
#: arm is using the bus for something that matters more.
QUIET_MODES = ("IDLE", "ACTIVE_ERROR", "FLASHING")


def _describe(err: BaseException) -> str:
    """A runtime refusal, with the part that says why.

    ``str()`` of a wire error is its code and title; the ceiling that was
    exceeded, the mode that was wrong, live in ``cause`` and ``remedy``.
    """
    cause = getattr(err, "cause", "")
    remedy = getattr(err, "remedy", "")
    if not cause:
        return str(err)
    return f"{getattr(err, 'title', str(err))}: {cause}" + (
        f" {remedy}" if remedy else ""
    )


def _fmt(value: float | None, unit: str, digits: int = 1) -> str:
    """A reading, a dash for one that has not arrived, blank for a sensor
    this drive does not have. The three must never look alike."""
    if value is None:
        return ""
    if value != value:
        return "—"
    return f"{value:.{digits}f} {unit}".strip()


class DrivesPanel(Panel):
    id = "par6-drives"
    display_name = "Drives"
    slot = PanelSlot.LEFT_TOP_TAB
    tab_icon = "memory"
    tab_tooltip = "CAN drive health, tuning and firmware"
    order = 60
    min_width = 460
    min_height = 340
    default_width = 620
    default_height = 480
    resizable = True

    def __init__(self) -> None:
        self._joints: list[dict[str, Any]] = []
        self._config_error: str | None = None
        self._selected_node: int | None = None
        self._baseline: dict[str, float] = {}
        self._releases: list[releases.ReleaseSummary] = []
        self._robot_config: dict[str, Any] = {}
        self._busy = False
        self._entry_rate_hz: float | None = None
        self._current_rate_hz: float | None = None
        self._reset_element_refs()

    def _reset_element_refs(self) -> None:
        self._commander: Commander | None = None
        self._readings_body: ui.column | None = None
        self._voltage_label: ui.label | None = None
        self._rows: dict[int, dict[str, ui.label]] = {}
        self._fault_rows: dict[int, ui.row] = {}
        self._bus_table: ui.table | None = None
        self._gain_inputs: dict[str, ui.number] = {}
        self._gain_note: ui.label | None = None
        self._node_select: ui.select | None = None
        self._commission_note: ui.label | None = None
        self._new_id: ui.number | None = None
        self._force: ui.switch | None = None
        self._release_select: ui.select | None = None
        self._product_select: ui.select | None = None
        self._firmware_note: ui.label | None = None
        self._flash_button: ui.button | None = None
        self._rate_select: ui.select | None = None

    def applies_to(self, commander: Commander) -> bool:
        return commander.robot.backend_package == "par6"

    @property
    def _client(self) -> AsyncRobotClient | None:
        """par6's own client. ``applies_to`` admits only the par6 backend,
        so the host's client is that one and carries the commissioning
        surface the generic ABC does not declare."""
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
                ui.label("CAN drives").classes("text-subtitle1")
                ui.space()
                self._rate_select = (
                    ui.select({}, label="Status rate", on_change=self._on_rate_change)
                    .props("dense outlined")
                    .classes("w-36")
                    .mark("drives-status-rate")
                )
            self._build_readings()
            self._build_bus()
            self._build_tuning()
            self._build_commissioning()
            self._build_firmware()

        ui.timer(REFRESH_S, self._refresh_readings)
        ui.timer(0.1, self._load_config, once=True)
        ui.timer(0.2, self._load_rate, once=True)

    # -- live readings --------------------------------------------------

    def _build_readings(self) -> None:
        with (
            ui.card().tight().classes("w-full"),
            ui.column().classes("w-full p-2 gap-1"),
        ):
            with ui.row().classes("w-full items-center text-caption text-grey"):
                ui.label("Drive").classes("w-24")
                ui.label("Temp").classes("w-20 text-right")
                ui.label("Current").classes("w-24 text-right")
                ui.space()
                self._voltage_label = (
                    ui.label().classes("text-right").mark("drives-bus-voltage")
                )
            self._readings_body = (
                ui.column().classes("w-full gap-1").mark("drives-readings")
            )

    def _drive_names(self) -> list[tuple[int, str]]:
        if self._joints:
            return [(int(j["node_id"]), str(j["name"])) for j in self._joints]
        commander = self._commander
        if commander is None:
            return []
        health = commander.status.drive_health
        count = max(
            len(health.temperatures_c), len(health.currents_ma), len(health.faults)
        )
        return [(i, f"drive {i}") for i in range(count)]

    def _refresh_readings(self) -> None:
        commander = self._commander
        if commander is None:
            return
        health = commander.status.drive_health
        names = self._drive_names()
        if len(names) != len(self._rows):
            self._rebuild_reading_rows(names)

        for index, (node, _name) in enumerate(names):
            row = self._rows.get(node)
            if row is None:
                continue
            temp = (
                health.temperatures_c[index]
                if index < len(health.temperatures_c)
                else None
            )
            current = (
                health.currents_ma[index] if index < len(health.currents_ma) else None
            )
            row["temp"].text = _fmt(temp, "°C")
            row["current"].text = _fmt(current, "mA", 0)
            faults = health.faults[index] if index < len(health.faults) else ()
            self._render_faults(node, faults)

        if self._voltage_label is not None:
            voltage = health.bus_voltage_v
            self._voltage_label.text = "" if voltage is None else f"bus {voltage:.1f} V"

    def _rebuild_reading_rows(self, names: list[tuple[int, str]]) -> None:
        if self._readings_body is None:
            return
        self._rows.clear()
        self._fault_rows.clear()
        self._readings_body.clear()
        with self._readings_body:
            for node, name in names:
                with ui.column().classes("w-full gap-0"):
                    with ui.row().classes("w-full items-center"):
                        ui.label(f"{name} · {node}").classes("w-24 text-caption")
                        temp = (
                            ui.label("")
                            .classes("w-20 text-right font-mono text-caption")
                            .mark(f"drives-temp-{node}")
                        )
                        current = (
                            ui.label("")
                            .classes("w-24 text-right font-mono text-caption")
                            .mark(f"drives-current-{node}")
                        )
                        ui.space()
                    faults = (
                        ui.row().classes("gap-1 pl-24").mark(f"drives-faults-{node}")
                    )
                self._rows[node] = {"temp": temp, "current": current}
                self._fault_rows[node] = faults
        if self._node_select is not None:
            self._node_select.set_options(
                {node: f"{name} · {node}" for node, name in names}
            )

    def _render_faults(self, node: int, faults: tuple[str, ...]) -> None:
        row = self._fault_rows.get(node)
        if row is None:
            return
        existing = tuple(getattr(c, "text", "") for c in row.default_slot.children)
        if existing == faults:
            return
        row.clear()
        with row:
            for label in faults:
                ui.chip(label, color="negative").props("dense outline square").classes(
                    "text-caption"
                )

    # -- the bus --------------------------------------------------------

    def _build_bus(self) -> None:
        with ui.expansion("Bus", icon="lan").classes("w-full") as expansion:
            ui.button("Rescan", icon="refresh", on_click=self._rescan).props(
                "flat dense"
            ).mark("drives-rescan")
            self._bus_table = (
                ui.table(
                    columns=[
                        {
                            "name": "node",
                            "label": "Node",
                            "field": "node",
                            "align": "left",
                        },
                        {"name": "present", "label": "Present", "field": "present"},
                        {"name": "freshness", "label": "Link", "field": "freshness"},
                        {"name": "hw_ver", "label": "HW", "field": "hw_ver"},
                        {"name": "sw_ver", "label": "FW", "field": "sw_ver"},
                        {"name": "serial", "label": "Serial", "field": "serial"},
                    ],
                    rows=[],
                    row_key="node",
                )
                .props("dense flat")
                .classes("w-full")
                .mark("drives-bus-table")
            )
        expansion.on_value_change(lambda e: e.value and self._rescan())

    _FRESHNESS = {0: "unknown", 1: "fresh", 2: "stale", 3: "lost"}

    async def _rescan(self) -> None:
        client = self._client
        if client is None or self._bus_table is None:
            return
        rows = await client.bus_scan()
        if rows is None:
            ui.notify("The runtime did not answer the scan", color="warning")
            return
        self._bus_table.rows = [
            {
                "node": row.get("node"),
                "present": "yes" if row.get("present") else "no",
                "freshness": self._FRESHNESS.get(row.get("freshness", 0), "?"),
                "hw_ver": row.get("hw_ver") or "—",
                "sw_ver": row.get("sw_ver") or "—",
                "serial": row.get("serial") or "—",
            }
            for row in rows
            if row.get("present") or row.get("configured")
        ]
        self._bus_table.update()

    # -- tuning ---------------------------------------------------------

    def _build_tuning(self) -> None:
        with ui.expansion("Tuning", icon="tune").classes("w-full"):
            self._node_select = (
                ui.select({}, label="Drive", on_change=self._on_node_change)
                .props("dense outlined")
                .classes("w-full")
                .mark("drives-node-select")
            )
            with ui.grid(columns=2).classes("w-full gap-1"):
                for name, _home, label, unit in GAIN_FIELDS:
                    self._gain_inputs[name] = (
                        ui.number(label, suffix=unit or None, format="%g")
                        .props("dense outlined")
                        .classes("w-full")
                        .mark(f"drives-gain-{name}")
                    )
            self._gain_note = (
                ui.label("").classes("text-caption text-grey").mark("drives-gain-note")
            )
            with ui.row().classes("w-full items-center gap-2"):
                ui.button("Apply", icon="publish", on_click=self._apply_gains).props(
                    "unelevated dense"
                ).mark("drives-apply-gains")
                ui.button("Revert", on_click=self._revert_gains).props("flat dense")
                ui.space()
                ui.button(
                    "Save to drive", icon="save", on_click=self._save_config
                ).props("flat dense").mark("drives-save-config")
            ui.label(
                "Applied values live until the drive is power-cycled; saving "
                "writes them to its NVM. The drive answers no parameter reads "
                "over CAN, so these fields are the runtime's stored config, "
                "not a read-back."
            ).classes("text-caption text-grey")

    def _joint_for(self, node: int | None) -> dict[str, Any] | None:
        for joint in self._joints:
            if int(joint["node_id"]) == node:
                return joint
        return None

    def _on_node_change(self, event) -> None:
        self._selected_node = None if event.value is None else int(event.value)
        self._seed_gain_fields()

    def _seed_gain_fields(self) -> None:
        joint = self._joint_for(self._selected_node)
        self._baseline = {}
        if joint is None:
            for widget in self._gain_inputs.values():
                widget.value = None
            self._set_gain_note()
            return
        gains = joint.get("gains") or {}
        for name, home, _label, _unit in GAIN_FIELDS:
            source = gains if home == "gains" else joint
            value = source.get(name)
            self._baseline[name] = float(value) if value is not None else 0.0
            self._gain_inputs[name].value = self._baseline[name]
        self._set_gain_note()

    def _revert_gains(self) -> None:
        self._seed_gain_fields()

    def _set_gain_note(self, extra: str = "") -> None:
        if self._gain_note is None:
            return
        changed = [
            label
            for name, _home, label, _unit in GAIN_FIELDS
            if name in self._baseline
            and self._gain_inputs[name].value is not None
            and float(self._gain_inputs[name].value) != self._baseline[name]
        ]
        parts = []
        if changed:
            parts.append("unsaved: " + ", ".join(changed))
        if self._config_error:
            parts.append(self._config_error)
        if extra:
            parts.append(extra)
        self._gain_note.text = " · ".join(parts)

    async def _apply_gains(self) -> None:
        client = self._client
        node = self._selected_node
        if client is None or node is None:
            ui.notify("Pick a drive first", color="warning")
            return
        try:
            values = {
                name: float(self._gain_inputs[name].value)
                for name, _home, _label, _unit in GAIN_FIELDS
            }
        except (TypeError, ValueError):
            ui.notify("Every field must hold a number", color="warning")
            return
        try:
            await client.set_pid_gains(
                node,
                kpp=values["kpp"],
                kpv=values["kpv"],
                kiv=values["kiv"],
                kpiq=values["kpiq"],
                kiiq=values["kiiq"],
                kp=values["kp"],
                kd=values["kd"],
                ilim_ma=values["ilim_ma"],
                velocity_limit_ticks_s=values["velocity_limit_ticks_s"],
                voltage_limit_mv=int(values["voltage_limit_mv"]),
            )
        except Exception as err:
            # The runtime refuses a write that would raise a limit past the
            # configured ceiling. That refusal is the safety net a bench
            # tool does not have, so it is shown, not swallowed.
            ui.notify(f"Refused: {_describe(err)}", color="negative", multi_line=True)
            self._set_gain_note(_describe(err))
            return
        self._baseline = dict(values)
        self._set_gain_note()
        ui.notify(f"Drive {node} retuned", color="positive")

    async def _save_config(self) -> None:
        client = self._client
        node = self._selected_node
        if client is None or node is None:
            ui.notify("Pick a drive first", color="warning")
            return
        blocker = self._quiet_blocker()
        if blocker:
            ui.notify(blocker, color="warning")
            return
        try:
            await client.save_config(node)
        except Exception as err:
            ui.notify(f"Refused: {_describe(err)}", color="negative", multi_line=True)
            return
        ui.notify(f"Drive {node} saved its configuration", color="positive")

    # -- commissioning --------------------------------------------------

    def _build_commissioning(self) -> None:
        with ui.expansion("Commissioning", icon="build").classes("w-full"):
            ui.label(
                "A drive out of its box answers on its factory id. Give it the "
                "id this arm's config expects, save it, then update the config "
                "and restart the runtime."
            ).classes("text-caption text-grey")
            with ui.row().classes("w-full items-center gap-2"):
                self._new_id = (
                    ui.number("New id", min=0, max=13, precision=0)
                    .props("dense outlined")
                    .classes("w-28")
                    .mark("drives-new-id")
                )
                self._force = ui.switch("Unconfigured drive").mark("drives-force")
                ui.space()
                ui.button("Set id", on_click=self._set_can_id).props(
                    "unelevated dense"
                ).mark("drives-set-id")
            self._commission_note = ui.label("").classes("text-caption")
            ui.timer(REFRESH_S, self._refresh_commission_note)

    def _quiet_blocker(self) -> str:
        commander = self._commander
        if commander is None:
            return "No runtime"
        mode = commander.status.controller.mode
        if mode and mode not in QUIET_MODES:
            return f"The arm is in {mode}; commissioning needs it idle."
        return ""

    def _refresh_commission_note(self) -> None:
        if self._commission_note is None:
            return
        blocker = self._quiet_blocker()
        self._commission_note.text = blocker or "Ready."
        self._commission_note.classes(
            replace="text-caption " + ("text-warning" if blocker else "text-grey")
        )

    async def _set_can_id(self) -> None:
        client = self._client
        node = self._selected_node
        if client is None or node is None:
            ui.notify("Pick a drive in Tuning first", color="warning")
            return
        if self._new_id is None or self._new_id.value is None:
            ui.notify("Give the drive a new id", color="warning")
            return
        blocker = self._quiet_blocker()
        if blocker:
            ui.notify(blocker, color="warning")
            return
        new_id = int(self._new_id.value)
        force = bool(self._force.value) if self._force else False
        try:
            await client.set_can_id(node, new_id, force=force)
        except Exception as err:
            ui.notify(f"Refused: {_describe(err)}", color="negative", multi_line=True)
            return
        ui.notify(
            f"Drive {node} now answers as {new_id}. Save it, or it is lost at "
            "power-off.",
            color="positive",
            multi_line=True,
        )

    # -- firmware -------------------------------------------------------

    def _build_firmware(self) -> None:
        with ui.expansion("Firmware", icon="system_update").classes("w-full"):
            with ui.row().classes("w-full items-center gap-2"):
                self._product_select = (
                    ui.select(
                        {
                            key: value["label"]
                            for key, value in releases.PRODUCTS.items()
                        },
                        value="stepfoc",
                        label="Board",
                    )
                    .props("dense outlined")
                    .classes("w-44")
                    .mark("drives-firmware-product")
                )
                ui.button("Check releases", on_click=self._load_releases).props(
                    "flat dense"
                ).mark("drives-check-releases")
            self._release_select = (
                ui.select({}, label="Release")
                .props("dense outlined")
                .classes("w-full")
                .mark("drives-release-select")
            )
            self._firmware_note = ui.label("").classes("text-caption text-grey")
            self._flash_button = (
                ui.button("Flash selected drive", icon="bolt", on_click=self._flash)
                .props("unelevated dense")
                .classes("w-full")
                .mark("drives-flash")
            )
            self._refresh_firmware_availability()

    def _can_flash_here(self) -> tuple[bool, str]:
        """Whether this process can drive the bus itself.

        Flashing is not a runtime command — it is direct SocketCAN traffic
        while par6d holds its peace — so it only works where the interface
        actually is. A browser on a laptop pointed at a control box is the
        normal case, and telling the operator the one command to run there
        beats a button that cannot work.
        """
        bus = self._robot_config.get("bus")
        interface = (
            bus["interface"]
            if isinstance(bus, dict) and isinstance(bus.get("interface"), str)
            else "can0"
        )
        if not Path(f"/sys/class/net/{interface}").exists():
            return False, (
                f"No {interface} on this machine. Run this on the control box:"
            )
        try:
            import can  # noqa: F401
        except ImportError:
            return False, (
                "python-can is not installed here. Install par6 with the "
                "'flash' extra, or run this on the control box:"
            )
        return True, ""

    def _refresh_firmware_availability(self) -> None:
        if self._flash_button is None or self._firmware_note is None:
            return
        available, why = self._can_flash_here()
        self._flash_button.set_visibility(available)
        if not available:
            node = self._selected_node if self._selected_node is not None else "N"
            product = self._product_select.value if self._product_select else "stepfoc"
            self._firmware_note.text = (
                f"{why}  par6 flash --node {node} --product {product}"
            )

    async def _load_releases(self) -> None:
        if self._release_select is None or self._product_select is None:
            return
        product = str(self._product_select.value)
        try:
            found = await asyncio.to_thread(releases.list_releases, product)
        except releases.FirmwareFetchError as err:
            ui.notify(str(err), color="negative", multi_line=True)
            return
        self._releases = found
        self._release_select.set_options(
            {
                r.tag: f"{r.tag}{' (prerelease)' if r.prerelease else ''}"
                for r in found
                if r.usable
            }
        )
        skipped = sum(1 for r in found if not r.usable)
        if self._firmware_note is not None and skipped:
            self._firmware_note.text = (
                f"{skipped} release(s) carry no firmware.json and cannot be verified."
            )

    async def _flash(self) -> None:
        commander = self._commander
        node = self._selected_node
        if commander is None or node is None:
            ui.notify("Pick a drive in Tuning first", color="warning")
            return
        if self._release_select is None or not self._release_select.value:
            ui.notify("Pick a release", color="warning")
            return
        blocker = self._quiet_blocker()
        if blocker:
            ui.notify(blocker, color="warning")
            return
        if self._busy:
            return

        product = str(self._product_select.value) if self._product_select else "stepfoc"
        tag = str(self._release_select.value)
        self._busy = True
        try:
            image = await asyncio.to_thread(releases.fetch_release, product, tag)
            confirmed = await self._confirm(node, image)
            if not confirmed:
                return
            report = await asyncio.to_thread(self._run_flash, commander, node, image)
            ui.notify(report.summary(), color="positive", multi_line=True)
        except (releases.FirmwareFetchError, ValueError, RuntimeError) as err:
            ui.notify(
                f"Flash failed: {_describe(err)}", color="negative", multi_line=True
            )
        finally:
            self._busy = False

    async def _confirm(self, node: int, image: releases.FirmwareImage) -> bool:
        result: asyncio.Future[bool] = asyncio.get_running_loop().create_future()
        with ui.dialog() as dialog, ui.card():
            ui.label(f"Flash {image.version} to drive {node}?").classes(
                "text-subtitle1"
            )
            ui.label(
                f"{len(image.data)} bytes, sha256 {image.sha256[:12]}…"
                + ("" if image.checksum_verified else " — integrity unverified")
            ).classes("text-caption text-grey")
            ui.label(
                "The bus goes silent for the whole write and the drive reboots "
                "afterwards. An interrupted write leaves the drive waiting in "
                "its bootloader — recoverable by flashing it again, but the arm "
                "cannot move until it is."
            ).classes("text-caption")
            with ui.row().classes("w-full justify-end gap-2"):
                ui.button(
                    "Cancel",
                    on_click=lambda: (dialog.close(), result.set_result(False)),
                ).props("flat")
                ui.button(
                    "Flash",
                    color="negative",
                    on_click=lambda: (dialog.close(), result.set_result(True)),
                ).props("unelevated").mark("drives-flash-confirm")
        dialog.open()
        return await result

    def _run_flash(
        self, commander: Commander, node: int, image: releases.FirmwareImage
    ) -> FlashReport:
        """Blocking, on a worker thread: the bootloader conversation is
        synchronous and holds the bus for tens of seconds."""
        from par6.firmware.flasher import flash_image
        from par6.firmware.session import flash_lock, granted_bus

        sync = cast(RobotClient, commander.robot.create_sync_client())
        with sync, flash_lock(), granted_bus(sync, "parked") as bus:
            return flash_image(bus, node, image.data, on_log=logger.info)

    # -- status rate ----------------------------------------------------

    async def _load_rate(self) -> None:
        commander = self._commander
        if commander is None or self._rate_select is None:
            return
        rate = await commander.client.status_rate()
        if rate is None:
            self._rate_select.set_visibility(False)
            return
        if self._entry_rate_hz is None:
            self._entry_rate_hz = rate.hz
        self._current_rate_hz = rate.hz
        self._rate_select.set_options(
            {hz: f"{hz:g} Hz" for hz in rate.achievable()}, value=rate.hz
        )

    async def _on_rate_change(self, event) -> None:
        commander = self._commander
        if commander is None or event.value is None:
            return
        wanted = float(event.value)
        if wanted == self._current_rate_hz:
            return
        try:
            await commander.client.set_status_rate(wanted)
        except Exception as err:
            ui.notify(f"Refused: {_describe(err)}", color="negative", multi_line=True)
            await self._load_rate()
        else:
            self._current_rate_hz = wanted

    async def stop(self) -> None:
        """Put the broadcast rate back.

        Raising it is a session tool — resolution while somebody is
        watching a drive — and leaving it raised costs every other
        consumer bandwidth and CPU for the rest of the runtime's life.
        """
        commander = self._commander
        if commander is None or self._entry_rate_hz is None:
            return
        try:
            await commander.client.set_status_rate(self._entry_rate_hz)
        except Exception:
            logger.exception("could not restore the status rate")

    # -- config ---------------------------------------------------------

    async def _load_config(self) -> None:
        client = self._client
        if client is None:
            return
        bundle = await client.config_bundle()
        if not isinstance(bundle, dict):
            self._config_error = "the runtime did not answer config_bundle()"
            self._set_gain_note()
            return
        try:
            parsed = tomllib.loads(bundle.get("robot_toml") or "")
        except (tomllib.TOMLDecodeError, TypeError) as err:
            self._config_error = f"the runtime's config did not parse: {err}"
            self._set_gain_note()
            return
        self._robot_config = parsed
        joints = parsed.get("joints")
        self._joints = (
            [j for j in joints if isinstance(j, dict)]
            if isinstance(joints, list)
            else []
        )
        self._config_error = None
        names = self._drive_names()
        self._rebuild_reading_rows(names)
        if self._selected_node is None and names:
            self._selected_node = names[0][0]
            if self._node_select is not None:
                self._node_select.value = self._selected_node
        self._seed_gain_fields()
        self._refresh_firmware_availability()
