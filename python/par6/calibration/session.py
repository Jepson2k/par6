"""A bounded experiment lifecycle using the ordinary controller motion API."""

from __future__ import annotations

import asyncio
import gc
import itertools
import time
import tomllib
from dataclasses import asdict
from pathlib import Path

import numpy as np

from par6 import config as paths
from par6._par6 import CollisionWorld, ControllerMode, GravityModel, Preview

from . import capture
from .analysis import motion_metrics
from .preflight import check_command_peaks, check_stream, stream_scale
from .profiles import atomic_json, atomic_text, fingerprint
from .trajectory import move
from .validation import (
    Acceptance,
    assess_motion,
    gravity_hold_metrics,
    motion_acceptance,
)


async def finish_cleanup(operation):
    """Finish bounded stop/restore work even if cancellation arrives again."""
    task = asyncio.create_task(operation)
    cancelled = False
    while not task.done():
        try:
            await asyncio.shield(task)
        except asyncio.CancelledError:
            cancelled = True
    result = task.result()
    if cancelled:
        raise asyncio.CancelledError
    return result


class TrialRejected(RuntimeError):
    """A measured operating bound failed; the stimulus still confirms Stop."""


def feedback_age(s):
    """Include native-to-Python delivery and subsequent cache age."""
    received = s["client_received_monotonic_s"]
    age = time.monotonic() - received + s["data_age_ms"] / 1000
    if not np.isfinite(age) or age < 0:
        raise RuntimeError("Invalid controller telemetry timestamp")
    return age


class CalibrationSession:
    def __init__(
        self,
        client,
        directory: str | Path,
        capture_path: str | Path,
        *,
        acceptance=None,
    ):
        self.client = client
        self.directory = Path(directory)
        self.capture_path = Path(capture_path)
        self.trials = []
        self.core = None
        self.sequence = -1
        self.capture_identity = None
        self.acceptance = Acceptance() if acceptance is None else acceptance

    async def __aenter__(self):
        self.directory.mkdir(parents=True, exist_ok=False, mode=0o700)
        self.core = await self.client._ensure_core()
        status = await self.fresh()
        if status["mode"] != 1 or status["queued_segments"]:
            raise RuntimeError(
                "Calibration requires an idle, homed arm with an empty queue"
            )
        self.bundle = await self.client.config_bundle()
        if self.bundle is None:
            raise RuntimeError("No runtime configuration readback")
        self.robot = tomllib.loads(self.bundle["robot_toml"])
        self.policy = self.acceptance.describe(self.robot)
        self.policy["measurement_clock"] = (
            "simulation_tick" if status["simulator_active"] else "monotonic"
        )
        atomic_json(self.directory / "acceptance.json", self.policy)
        payload = await self.client.payload()
        if payload is None or payload.mass != 0:
            raise RuntimeError(
                "This protocol requires an empty gripper and zero declared payload"
            )
        self.fingerprint = fingerprint(self.bundle)
        await self.verify_capture_source()
        deadline = time.monotonic() + 8
        while True:
            drives = await self.client.bus_scan()
            configured = (
                [] if drives is None else [d for d in drives if d["configured"]]
            )
            if configured and all(
                d["present"] and any(d[k] != 0 for k in ("hw_ver", "sw_ver", "serial"))
                for d in configured
            ):
                break
            if time.monotonic() >= deadline:
                raise RuntimeError("Drive identity readback is missing or incomplete")
            # Scans borrow the poll slot; leave room for the periodic device-info
            # sweep (about four seconds at the normal native tick rate).
            next_scan = time.monotonic() + 0.5
            while time.monotonic() < next_scan:
                await self.fresh()
        self.identity = {
            "config_fingerprint": self.fingerprint,
            "runtime": await self.client.config_info(),
            "drives": drives,
            "payload": asdict(payload),
            "simulator": status["simulator_active"],
            "recording": self.capture_identity,
        }
        atomic_json(self.directory / "identity.json", self.identity)
        filename = self.bundle["robot_filename"]
        if Path(filename).name != filename or filename in ("", ".", ".."):
            raise ValueError("Unsafe robot filename")
        config = self.directory / filename
        atomic_text(config, self.bundle["robot_toml"])
        for g in self.bundle["grippers"]:
            if Path(g["filename"]).name != g["filename"] or g["filename"] in (
                "",
                ".",
                "..",
            ):
                raise ValueError("Unsafe config filename")
            atomic_text(self.directory / "grippers" / g["filename"], g["content"])
        # Native I/O releases the GIL; the worker also leaves the event loop
        # available for fresh status while loading meshes and model assets.
        self.model = await asyncio.to_thread(
            GravityModel, str(config), str(paths.data_root())
        )
        self.preview = await asyncio.to_thread(
            Preview,
            config=str(config),
            assets=str(paths.data_root()),
            package_dir=str(paths.package_search_dir()),
        )
        tool = self.robot["robot"]["active_gripper"]
        self.world = await asyncio.to_thread(
            CollisionWorld,
            str(paths.urdf_path(tool)),
            str(paths.package_search_dir()),
            str(paths.srdf_path(tool)),
        )
        shapes = await self.core.shapes()
        if shapes is None:
            raise RuntimeError("No collision-world readback")
        atomic_json(self.directory / "world.json", shapes)
        if not status["simulator_active"] and not any(
            shape.get("collision", True)
            for layer in ("installation", "program")
            for shape in shapes[layer]
        ):
            raise RuntimeError(
                "Hardware calibration requires the work surface in the collision "
                "scene; the connected runtime has no obstacle geometry"
            )
        for layer in ["installation", "program"]:
            self.world.set_layer(layer, shapes[layer])
        self.world_epoch = shapes["epoch"]
        status = await self.fresh()
        self.start = np.deg2rad(status["angles"])
        joints = self.robot["joints"]
        self.window = np.array(
            [[j["limits"]["soft_min_rad"], j["limits"]["soft_max_rad"]] for j in joints]
        )
        self.limits = np.array(
            [
                [
                    j["limits"].get("exec", {}).get(k, j["limits"][k])
                    for k in ["velocity_rad_s", "acceleration_rad_s2", "jerk_rad_s3"]
                ]
                for j in joints
            ]
        )
        dt, rows = capture.read_capture(
            self.capture_path, max(0, capture.length(self.capture_path) - 10)
        )
        if not rows or abs(dt - self.robot["robot"]["tick_dt_s"]) > 1e-9:
            raise RuntimeError(
                "Native recording is missing or belongs to another tick configuration"
            )
        before = rows[-1]["tick"]
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            await self.fresh()
            _, tail = capture.read_capture(
                self.capture_path, max(0, capture.length(self.capture_path) - 1)
            )
            if tail and tail[-1]["tick"] > before:
                break
        else:
            raise RuntimeError("Native recording is not advancing")
        # The local fitting chain must match the runtime's current model,
        # including any previously installed arm correction.
        _, evidence = capture.read_capture(
            self.capture_path, max(0, capture.length(self.capture_path) - 5)
        )
        correction = np.asarray(self.robot.get("gravity_correction", []))
        for r in evidence:
            expected = np.asarray(self.model.gravity(r["q"]))
            if correction.size:
                expected += np.asarray(self.model.regressor(r["q"])) @ correction
            expected *= np.asarray(self.robot.get("gravity_scale", [1.0] * 6))
            if not np.allclose(expected, r["gravity_nm"], atol=0.005, rtol=0.002):
                raise RuntimeError(
                    "Local gravity assets differ from the connected controller"
                )
        await self.verify_capture_source()
        atomic_json(
            self.directory / "capture.json",
            {**self.capture_identity, "path": str(self.capture_path.resolve())},
        )
        self.identity["reference"] = capture.reference_identity(self.capture_path)
        atomic_json(self.directory / "identity.json", self.identity)
        return self

    def assert_capture_live(self):
        if self.capture_identity is None:
            raise RuntimeError("Native recorder identity has not been verified")
        return capture.assert_live(
            self.capture_path,
            self.bundle["fingerprint"],
            expected_identity=self.capture_identity,
        )

    async def verify_capture_source(self):
        """Bind evidence to the recorder served by this connected runtime."""
        reply = await self.core.capture_info()
        identity = None if reply is None else reply.get("identity")
        if identity is None:
            raise RuntimeError("Connected runtime has no verifiable native recorder")
        if self.capture_identity is not None and identity != self.capture_identity:
            raise RuntimeError("Connected runtime recorder changed during calibration")
        capture.assert_live(
            self.capture_path, self.bundle["fingerprint"], expected_identity=identity
        )
        self.capture_identity = identity

    async def _read_status(self, timeout):
        s = await self.core.status_after(self.sequence, timeout)
        if s is None:
            raise RuntimeError("Controller telemetry stopped")
        # A native Future may finish while Python is busy. Prefer a genuinely
        # newer received packet, preserving its original receipt timestamp.
        if feedback_age(s) > 0.1:
            latest = self.core.latest_status()
            if (
                latest is not None
                and latest["client_received_monotonic_s"]
                > s["client_received_monotonic_s"]
            ):
                s = latest
        self.sequence = s["seq"]
        return s

    def _require_ready(self, s):
        if (
            s["error"]
            or not s["homed"]
            or not s["enabled"]
            or s["link_ok"] != 1
            or feedback_age(s) > 0.1
        ):
            raise RuntimeError(
                f"Calibration lost readiness: error={s.get('error')}, mode={s['mode']}, "
                f"homed={s['homed']}, enabled={s['enabled']}, link={s['link_ok']}, "
                f"feedback_age_ms={1000 * feedback_age(s):.1f}"
            )
        if hasattr(self, "world_epoch") and s["scene_epoch"] != self.world_epoch:
            raise RuntimeError("Collision world changed during calibration")
        if any(s["drive_health"]["faults"]):
            raise RuntimeError("Drive fault during calibration")

    async def fresh(self):
        s = await self._read_status(0.5)
        self._require_ready(s)
        return s

    async def idle_feedback(self):
        """Wait for fresh admission data while no calibration motion is active."""
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            s = await self._read_status(max(0.0, deadline - time.monotonic()))
            if feedback_age(s) > 0.1:
                continue
            self._require_ready(s)
            if (
                s["mode"] != ControllerMode.IDLE
                or s["queued_segments"]
                or s["executing_index"] >= 0
                or s["gravity_comp"]
            ):
                raise RuntimeError(
                    "STREAM admission requires gravity-disabled idle and an empty queue"
                )
            return s
        raise RuntimeError("No fresh idle feedback arrived within 0.5 seconds")

    async def stop(self):
        result = await self.client.stop()
        if result != 1:
            raise RuntimeError("Controller stop was not acknowledged")
        deadline = time.monotonic() + 3
        stable = None
        while time.monotonic() < deadline:
            s = await self.core.status_after(self.sequence, 0.5)
            if s is None:
                raise RuntimeError(
                    "Stop acknowledged but encoder rest could not be confirmed"
                )
            self.sequence = s["seq"]
            if (
                feedback_age(s) <= 0.1
                and s["link_ok"] == 1
                and max(abs(v) for v in s["speeds"]) < 0.03
                and not s["queued_segments"]
            ):
                stable = stable or time.monotonic()
                if time.monotonic() - stable >= 0.3:
                    return
            else:
                stable = None
        raise RuntimeError("Stop acknowledged but arm did not settle")

    def check_path(self, positions):
        q = np.asarray(positions)
        if q.ndim != 2 or q.shape[1] != 6 or len(q) < 2:
            raise ValueError("Path requires at least two six-joint poses")
        if (
            not np.isfinite(q).all()
            or np.any(q < self.window[:, 0])
            or np.any(q > self.window[:, 1])
        ):
            raise ValueError("Calibration path exceeds joint window")
        # Bound geometric sampling independently of stimulus sample rate.
        dense = [q[0]]
        for a, b in zip(q, q[1:]):
            steps = max(1, int(np.ceil(np.max(np.abs(b - a)) / 0.01)))
            dense.extend(np.linspace(a, b, steps + 1)[1:])
        bad = self.world.check_path(np.asarray(dense).tolist())
        if bad >= 0:
            raise ValueError(
                f"Calibration path collides: {self.world.pairs(list(dense[bad]))}"
            )

    async def stimulus(
        self,
        name,
        times,
        positions,
        *,
        settle=True,
        command_limits=None,
        speed=1.0,
        accel=1.0,
        allow_oscillation=False,
        **metadata,
    ):
        times, positions = np.asarray(times), np.asarray(positions)
        if (
            times.ndim != 1
            or len(times) != len(positions)
            or len(times) < 2
            or times[0] != 0
            or not np.isfinite(times).all()
            or np.any(np.diff(times) <= 0)
        ):
            raise ValueError("Stimulus needs increasing finite times starting at zero")
        bounds = self.limits if command_limits is None else np.asarray(command_limits)
        _, peaks = check_stream(
            self.preview,
            times,
            positions,
            bounds,
            speed=speed,
            accel=accel,
            check_path=self.check_path,
        )
        await self.verify_capture_source()
        latest = await self.idle_feedback()
        begin = capture.length(self.capture_path)
        start = time.monotonic()
        next_check = start + 1.2
        analysis_task = None
        analysis_started = start
        assessment = None
        saturation_start = None
        error = None
        temperatures = np.full(7, -np.inf)

        async def receive_feedback():
            nonlocal latest
            while True:
                latest = await self.fresh()

        feedback_task = asyncio.create_task(receive_feedback())

        def current_feedback():
            if feedback_task.done():
                feedback_task.result()
            # A new STATUS packet need not arrive before every target. Waiting
            # for it couples two independent 50 Hz schedules and accumulates
            # command delay. Bound cached feedback by its total age instead.
            age = feedback_age(latest)
            if age > 0.1:
                native = self.core.latest_status()
                native_age = None if native is None else 1000 * feedback_age(native)
                raise RuntimeError(
                    f"Calibration feedback exceeded 100 ms age: "
                    f"cached_ms={1000 * age:.1f}, native_ms={native_age}, "
                    f"cached_seq={latest['seq']}, "
                    f"native_seq={None if native is None else native['seq']}"
                )
            if latest["gravity_comp"]:
                raise RuntimeError("Gravity support changed during STREAM calibration")
            return latest

        def measure_recent():
            # Spectral analysis has a 0.5 s deadline; it must not occupy the
            # command producer's 20 ms interval with disk decoding and FFTs.
            self.assert_capture_live()
            end = capture.length(self.capture_path)
            dt, recent = capture.read_capture(
                self.capture_path,
                max(begin, end - round(1.2 / self.robot["robot"]["tick_dt_s"])),
                end,
            )
            recent = capture.measurement_rows(
                capture.active_rows(recent), dt, simulator=self.identity["simulator"]
            )
            if len(recent) * dt < 1.0:
                return None
            return motion_acceptance(
                motion_metrics(recent, dt),
                self.policy,
                allow_oscillation=allow_oscillation,
            )

        def observe(s):
            nonlocal saturation_start, analysis_task, assessment
            nonlocal analysis_started, next_check
            observed = np.asarray(s["drive_health"]["temperatures_c"])
            temperatures[: len(observed)] = np.maximum(
                temperatures[: len(observed)], observed
            )
            current = np.asarray(s["drive_health"]["currents_ma"][:6])
            ilim = np.asarray([j["ilim_ma"] for j in self.robot["joints"]])
            saturated = np.any(np.abs(current) >= 0.95 * ilim)
            if saturated:
                if saturation_start is None:
                    saturation_start = time.monotonic()
                if time.monotonic() - saturation_start > 0.2:
                    raise TrialRejected("Sustained drive-current saturation")
            else:
                saturation_start = None
            if analysis_task is not None:
                if analysis_task.done():
                    assessment = analysis_task.result()
                    analysis_task = None
                    if assessment is not None and not assessment["valid"]:
                        raise TrialRejected("; ".join(assessment["reasons"]))
                elif time.monotonic() - analysis_started > 0.5:
                    raise RuntimeError("Live motion analysis missed its 0.5 s deadline")
            if analysis_task is None and time.monotonic() >= next_check:
                analysis_started = time.monotonic()
                analysis_task = asyncio.create_task(asyncio.to_thread(measure_recent))
                next_check = analysis_started + 0.5

        # Cyclic collection can pause this producer for over 100 ms. Defer it
        # until Stop; reference counting still releases acyclic capture data.
        # Preserve existing caller freezes and an already-disabled collector.
        restore_gc = gc.isenabled()
        if restore_gc:
            gc.disable()
        try:
            for t, q in zip(times, positions):
                wait = start + float(t) - time.monotonic()
                if wait < -0.1:
                    raise RuntimeError("Command scheduling missed 100 ms deadline")
                if wait > 0:
                    await asyncio.sleep(wait)
                s = current_feedback()
                observe(s)
                if time.monotonic() - (start + float(t)) > 0.1:
                    raise RuntimeError("Command scheduling missed 100 ms deadline")
                current_feedback()
                await self.client.servo_j(
                    np.rad2deg(q).tolist(), speed=speed, accel=accel
                )
            if settle:
                # Keep feedback active during settling, rather than drifting into freedrive.
                deadline = time.monotonic() + self.robot.get("motion", {}).get(
                    "settle_timeout_s", 2
                )
                stable = None
                while time.monotonic() < deadline:
                    await asyncio.sleep(0.02)
                    s = current_feedback()
                    observe(s)
                    current_feedback()
                    await self.client.servo_j(
                        np.rad2deg(positions[-1]).tolist(), speed=speed, accel=accel
                    )
                    err = np.max(np.abs(np.deg2rad(s["angles"]) - positions[-1]))
                    if (
                        err
                        <= self.robot.get("motion", {}).get(
                            "settle_tolerance_rad", 0.01
                        )
                        and max(abs(v) for v in s["speeds"]) < 0.03
                    ):
                        stable = stable or time.monotonic()
                        if time.monotonic() - stable > 0.3:
                            break
                    else:
                        stable = None
                else:
                    raise TrialRejected("Calibration target did not settle")
            if analysis_task is not None:
                remaining = max(0.0, 0.5 - (time.monotonic() - analysis_started))
                try:
                    assessment = await asyncio.wait_for(analysis_task, remaining)
                except TimeoutError as exc:
                    raise RuntimeError(
                        "Live motion analysis missed its 0.5 s deadline"
                    ) from exc
                analysis_task = None
                if assessment is not None and not assessment["valid"]:
                    raise TrialRejected("; ".join(assessment["reasons"]))
        except BaseException as exc:
            error = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            if analysis_task is not None:
                analysis_task.cancel()
                # The worker only reads a bounded capture slice. Cancellation
                # never postpones the controller stop below.

            async def stop_feedback_and_motion():
                feedback_task.cancel()
                errors = await asyncio.gather(feedback_task, return_exceptions=True)
                await self.active_support()
                if isinstance(errors[0], Exception):
                    raise errors[0]

            try:
                await finish_cleanup(stop_feedback_and_motion())
            except BaseException as exc:
                error = f"{error or ''}; stop: {type(exc).__name__}: {exc}"
                raise
            finally:
                if restore_gc:
                    gc.enable()
                end = capture.length(self.capture_path)
                report = {
                    **metadata,
                    "name": name,
                    "gravity_comp": False,
                    "tested_mode": "STREAM",
                    "capture_start": begin,
                    "capture_end": end,
                    "error": error,
                    "settle_required": settle,
                    "allow_oscillation": allow_oscillation,
                    "live_acceptance": assessment,
                    "predicted_command_peaks": peaks,
                    "stream_scale": {"speed": speed, "accel": accel},
                }
                self.trials.append(report)
                atomic_json(self.directory / "trials.json", {"trials": self.trials})
        try:
            await self.verify_capture_source()
            dt, rows = capture.read_capture(self.capture_path, begin, end)
            rows = capture.measurement_rows(
                capture.active_rows(rows), dt, simulator=self.identity["simulator"]
            )
            if any(row["flags"] & 16 for row in rows):
                raise RuntimeError(
                    "Native STREAM evidence changed gravity support during collection"
                )
            metrics = assess_motion(
                rows, dt, self.policy, allow_oscillation=allow_oscillation
            )
        except (ValueError, RuntimeError) as exc:
            report["error"] = str(exc)
            atomic_json(self.directory / "trials.json", {"trials": self.trials})
            raise
        try:
            check_command_peaks(
                metrics["commanded_peaks"], bounds, dt, label="Recorded native stream"
            )
        except ValueError as exc:
            report["error"] = str(exc)
            report["metrics"] = metrics
            atomic_json(self.directory / "trials.json", {"trials": self.trials})
            raise TrialRejected(str(exc)) from exc
        report["metrics"] = metrics
        if not metrics["acceptance"]["valid"]:
            report["error"] = "; ".join(metrics["acceptance"]["reasons"])
            atomic_json(self.directory / "trials.json", {"trials": self.trials})
            raise TrialRejected(report["error"])
        report["max_temperature_c"] = [
            float(v) if np.isfinite(v) else None for v in temperatures
        ]
        atomic_json(self.directory / "trials.json", {"trials": self.trials})
        return rows, metrics

    async def active_support(self):
        """Disable gravity before Stop; latch active error if support is uncertain."""

        async def establish():
            report = {
                "gravity_disabled_confirmed": False,
                "software_estop_acknowledged": False,
                "software_estop_observed": False,
                "stop_confirmed": False,
                "errors": [],
            }
            errors = report["errors"]

            async def fence():
                # A latched error retains zero-velocity feedback even when
                # gravity or ordinary Stop acknowledgement is uncertain.
                try:
                    if await self.client.estop() != 1:
                        raise RuntimeError("Software EStop was not acknowledged")
                    report["software_estop_acknowledged"] = True
                    deadline = time.monotonic() + 1.0
                    while time.monotonic() < deadline:
                        status = await self.core.status_after(self.sequence, 0.5)
                        if status is None:
                            continue
                        self.sequence = status["seq"]
                        # fresh() intentionally refuses this disabled state.
                        if (
                            feedback_age(status) <= 0.1
                            and status["link_ok"] == 1
                            and status["mode"] == ControllerMode.ACTIVE_ERROR
                            and not status["enabled"]
                            and status["error"]
                        ):
                            report["software_estop_observed"] = True
                            break
                    else:
                        raise RuntimeError("Software EStop support was not observed")
                except BaseException as exc:
                    errors.append(f"software EStop: {type(exc).__name__}: {exc}")

            try:
                try:
                    if await self.client.set_gravity_comp(False) != 1:
                        raise RuntimeError("Gravity disable was not acknowledged")
                    deadline = time.monotonic() + 0.5
                    while time.monotonic() < deadline:
                        if not (await self.fresh())["gravity_comp"]:
                            report["gravity_disabled_confirmed"] = True
                            break
                    else:
                        raise RuntimeError("Gravity disable was not observed")
                except BaseException as exc:
                    errors.append(f"active support: {type(exc).__name__}: {exc}")
                    await fence()
            finally:
                try:
                    await self.stop()
                    report["stop_confirmed"] = True
                except BaseException as exc:
                    errors.append(f"Stop: {type(exc).__name__}: {exc}")
                    await fence()
                    try:
                        await self.stop()
                        report["stop_confirmed"] = True
                    except BaseException as exc:
                        errors.append(f"final Stop: {type(exc).__name__}: {exc}")
                finally:
                    atomic_json(self.directory / "active-support.json", report)
            if errors:
                raise RuntimeError("; ".join(errors))

        await finish_cleanup(establish())

    async def gravity_hold(self, **metadata):
        """Observe torque-only balance, then engage active zero-velocity holding."""
        if not self.identity["simulator"]:
            raise RuntimeError(
                "Gravity-only release is restricted to isolated simulation until "
                "its stopping margin is validated; use moving torque verification"
            )
        if self.robot.get("freedrive", {}).get("drift_lock", False):
            raise ValueError(
                "Disable freedrive drift_lock before measuring gravity balance"
            )
        s = await self.fresh()
        if s["mode"] not in (1, 6) or s["queued_segments"] or s["executing_index"] >= 0:
            raise RuntimeError("Gravity validation requires an idle arm")
        # The entire small drift window must have clearance, independently of
        # whether the model predicts enough torque to hold its center.
        q = np.deg2rad(s["angles"])
        margin = np.deg2rad(self.policy["gravity_abort_deg"] * 2)
        for signs in itertools.product((-1, 1), repeat=6):
            self.check_path([q, q + margin * np.asarray(signs)])
        await self.verify_capture_source()
        begin = capture.length(self.capture_path)
        report = {"name": "gravity-hold", "capture_start": begin, **metadata}
        error = None
        first = None

        def analyze(end):
            dt, rows = capture.read_capture(self.capture_path, begin, end)
            active = [
                i
                for i, r in enumerate(rows)
                if r["flags"] >> 8 == 1 and r["flags"] & 16
            ]
            if not active:
                raise ValueError("Capture contains no gravity-only hold")
            return gravity_hold_metrics(
                capture.measurement_rows(
                    rows[active[0] : active[-1] + 1],
                    dt,
                    simulator=self.identity["simulator"],
                ),
                dt,
                self.policy,
            )

        try:
            if await self.client.set_gravity_comp(True) != 1:
                raise RuntimeError("Controller refused gravity compensation")
            if await self.client.stop() != 1:
                raise RuntimeError("Controller stop was not acknowledged")
            start = time.monotonic()
            activation_deadline = start + 0.5
            while time.monotonic() - start < self.policy["gravity_duration_s"] + 0.15:
                s = await self.fresh()
                now = np.asarray(s["angles"])
                if first is None and (s["mode"] != 1 or not s["gravity_comp"]):
                    if time.monotonic() >= activation_deadline:
                        raise RuntimeError("Controller did not enter gravity-only idle")
                    if (
                        np.max(np.abs(now - np.rad2deg(q)))
                        > self.policy["gravity_abort_deg"]
                    ):
                        raise TrialRejected(
                            "Gravity-only drift exceeded abort excursion during activation"
                        )
                    continue
                if s["mode"] != 1 or not s["gravity_comp"] or s["queued_segments"]:
                    raise RuntimeError(
                        "Controller left gravity-only idle during validation"
                    )
                if first is None:
                    first = now
                    start = time.monotonic()
                if np.max(np.abs(now - first)) > self.policy["gravity_abort_deg"]:
                    raise TrialRejected("Gravity-only drift exceeded abort excursion")
                self.check_path([np.deg2rad(first), np.deg2rad(now)])
                current = np.abs(s["drive_health"]["currents_ma"][:6])
                if np.any(
                    current
                    >= 0.95 * np.array([j["ilim_ma"] for j in self.robot["joints"]])
                ):
                    raise TrialRejected("Current saturation during gravity hold")
                self.assert_capture_live()
        except TrialRejected as exc:
            error = str(exc)
        except BaseException as exc:
            error = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            end = capture.length(self.capture_path)

            async def hold_and_stop():
                if await self.client.set_gravity_comp(False) != 1:
                    raise RuntimeError("Could not engage active zero-velocity holding")
                await self.stop()

            try:
                await finish_cleanup(hold_and_stop())
                report["rest_confirmed"] = True
                report["active_hold_confirmed"] = True
                report["support_mode"] = "active_velocity_hold"
            except BaseException as exc:
                error = f"{error or ''}; cleanup: {type(exc).__name__}: {exc}"
                raise
            finally:
                # stop() waits for fresh rest; by then the final short hold has
                # reached the disk writer, even if it aborted before one flush.
                end = capture.length(self.capture_path)
                report.update(capture_end=end, error=error)
                self.trials.append(report)
                atomic_json(self.directory / "trials.json", {"trials": self.trials})
        try:
            await self.verify_capture_source()
            metrics = analyze(end)
        except (ValueError, RuntimeError) as exc:
            report["error"] = str(exc)
            atomic_json(self.directory / "trials.json", {"trials": self.trials})
            raise
        if error:
            metrics["valid"] = False
            metrics["reasons"].append(error)
        report["metrics"] = metrics
        atomic_json(self.directory / "trials.json", {"trials": self.trials})
        return metrics

    def position_settings(self, *, diagnostic=False):
        """Use the same realizable STREAM fractions in preflight and execution."""
        keys = ("velocity_rad_s", "acceleration_rad_s2", "jerk_rad_s3")
        loaded = np.array(
            [
                [j["limits"].get("stream", {}).get(k, j["limits"][k]) for k in keys]
                for j in self.robot["joints"]
            ]
        )
        desired = [0.05, 0.08, 0.3] if diagnostic else [0.2, 0.4, 1.2]
        base = np.minimum(np.minimum(self.limits, loaded), desired)
        return base, stream_scale(self.robot, base)

    def plan_position(self, a, b, *, diagnostic=False):
        """Find a bounded approach whose nominal stopping projections are clear."""
        base, scale = self.position_settings(diagnostic=diagnostic)
        error = None
        for fraction in (1.0, 0.75, 0.5, 0.25):
            times, positions = move(a, b, base * fraction)
            try:
                check_stream(
                    self.preview,
                    times,
                    positions,
                    base,
                    **scale,
                    check_path=self.check_path,
                )
            except ValueError as exc:
                error = exc
                continue
            return times, positions
        raise ValueError(f"No clear bounded calibration approach: {error}")

    async def position(self, q, *, diagnostic=False):
        s = await self.idle_feedback()
        a = np.deg2rad(s["angles"])
        times, positions = self.plan_position(a, q, diagnostic=diagnostic)
        # Diagnosis must reach its starting pose despite the vibration being
        # measured. Tracking, current, readiness and confirmed Stop still apply.
        base, scale = self.position_settings(diagnostic=diagnostic)
        _, metrics = await self.stimulus(
            "position",
            times,
            positions,
            command_limits=base,
            **scale,
            allow_oscillation=diagnostic,
        )
        if max(metrics["tracking_peak_deg"]) > 2 or metrics["faulted"]:
            raise TrialRejected("Calibration approach did not track correctly")

    async def __aexit__(self, typ, value, tb):
        stop_error = None
        try:
            await self.active_support()
        except BaseException as exc:
            stop_error = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            atomic_json(
                self.directory / "outcome.json",
                {
                    "complete": typ is None and stop_error is None,
                    "error": None if value is None else str(value),
                    "stop_error": stop_error,
                },
            )
