"""One calibration run against the connected par6d: preflight, stimuli, evidence.

Safety rules, all enforced here so a routine cannot forget one:

- every stimulus is preflighted through the native stream limiter and the
  collision world (approach, path, arrival and the gate's stopping
  projections) before a single datagram is sent;
- the live STATUS stream is a safety signal only — a fault, e-stop, mode
  change, drive fault, a packet older than FEEDBACK_MAX_AGE_S or SILENCE_S
  without any packet aborts the trial — measurements come from the 250 Hz
  native recording (capture.py), so Python scheduling cannot invalidate
  evidence;
- every trial ends with an acknowledged Stop and measured encoder rest, on
  success, rejection, error and cancellation alike;
- whole-trial and 1.2 s windowed acceptance both apply, judged after Stop;
- gain changes are volatile and restored on every exit, including
  cancellation; an unconfirmed SYSTEM acknowledgement is a failure;
- on hardware, an empty obstacle scene is refused before any motion.
"""

from __future__ import annotations

import asyncio
import contextlib
import time
import tomllib
from dataclasses import asdict
from pathlib import Path

import numpy as np

from par6 import config as paths
from par6._par6 import CollisionWorld, ControllerMode, GravityModel, Preview

from . import capture
from .metrics import Acceptance, assess_motion
from .preflight import check_command_peaks, check_stream, stream_scale
from .report import atomic_json, atomic_text, fingerprint
from .trajectory import move

#: A STATUS packet older than this is not evidence the arm is still healthy.
FEEDBACK_MAX_AGE_S = 0.25
#: No packet at all for this long ends the trial.
SILENCE_S = 0.5
#: Encoder rest: every joint slower than this for REST_HOLD_S.
REST_SPEED_RAD_S = 0.03
REST_HOLD_S = 0.3
STOP_BUDGET_S = 3.0
#: Sustained current at or above 95 % of the drive limit rejects a trial.
SATURATION_S = 0.2
#: Stream period (50 Hz targets) and the settle contract after a stream.
STREAM_DT_S = 0.02


class TrialRejected(RuntimeError):
    """The arm stopped cleanly; the stimulus failed acceptance."""


def feedback_age(s) -> float:
    return time.monotonic() - s["client_received_monotonic_s"]


async def finish_cleanup(operation):
    """Run `operation` to completion even if the caller is being cancelled;
    a cancellation arriving meanwhile is re-raised afterwards."""
    task = asyncio.ensure_future(operation)
    cancelled = None
    while True:
        try:
            return await asyncio.shield(task)
        except asyncio.CancelledError as exc:
            cancelled = exc
            if task.done():
                raise
        finally:
            if task.done() and cancelled is not None:
                raise cancelled


class Session:
    def __init__(self, client, directory, capture_path, *, acceptance=None):
        self.client = client
        self.directory = Path(directory)
        self.capture_path = Path(capture_path)
        self.acceptance = Acceptance() if acceptance is None else acceptance
        self.trials: list[dict] = []
        self.core = None
        self.sequence = -1
        self.capture_identity = None
        self.gravity_at_entry = False
        self.log = print

    # ------------------------------------------------------------ lifecycle

    async def __aenter__(self):
        self.directory.mkdir(parents=True, exist_ok=False, mode=0o700)
        self.core = await self.client._ensure_core()
        status = await self.fresh()
        if status["mode"] != ControllerMode.IDLE or status["queued_segments"]:
            raise RuntimeError(
                "Calibration requires an idle, homed arm with an empty queue"
            )
        self.gravity_at_entry = bool(status["gravity_comp"])
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
        drives = await self._drive_identity()
        self.identity = {
            "config_fingerprint": self.fingerprint,
            "runtime": await self.client.config_info(),
            "drives": drives,
            "payload": asdict(payload),
            "simulator": status["simulator_active"],
            "recording": self.capture_identity,
            "reference": capture.reference_identity(self.capture_path),
        }
        atomic_json(self.directory / "identity.json", self.identity)
        config = self._write_config_copy()
        # Native loaders release the GIL; the event loop keeps receiving status.
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
        for layer in ("installation", "program"):
            self.world.set_layer(layer, shapes[layer])
        self.world_epoch = shapes["epoch"]
        status = await self.fresh()
        self.start = np.deg2rad(status["angles"])
        joints = self.robot["joints"]
        self.window = np.array(
            [[j["limits"]["soft_min_rad"], j["limits"]["soft_max_rad"]] for j in joints]
        )
        keys = ("velocity_rad_s", "acceleration_rad_s2", "jerk_rad_s3")
        self.limits = np.array(
            [
                [j["limits"].get("exec", {}).get(k, j["limits"][k]) for k in keys]
                for j in joints
            ]
        )
        self.stream_limits = np.array(
            [
                [j["limits"].get("stream", {}).get(k, j["limits"][k]) for k in keys]
                for j in joints
            ]
        )
        await self._check_capture_advances_and_matches_model()
        atomic_json(
            self.directory / "capture.json",
            {**self.capture_identity, "path": str(self.capture_path.resolve())},
        )
        return self

    async def __aexit__(self, typ, value, tb):
        error = None
        try:
            await finish_cleanup(self._leave())
        except BaseException as exc:
            error = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            atomic_json(
                self.directory / "outcome.json",
                {
                    "complete": typ is None and error is None,
                    "error": None
                    if value is None
                    else f"{type(value).__name__}: {value}",
                    "cleanup_error": error,
                    "trials": len(self.trials),
                },
            )

    async def _leave(self):
        # Rest first, then hand the arm back in the support mode it arrived in.
        await self.stop()
        if (await self.fresh())["gravity_comp"] != self.gravity_at_entry:
            await self.set_gravity(self.gravity_at_entry)

    async def _drive_identity(self):
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
                return drives
            if time.monotonic() >= deadline:
                raise RuntimeError("Drive identity readback is missing or incomplete")
            await asyncio.sleep(0.5)

    def _write_config_copy(self) -> Path:
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
        return config

    async def _check_capture_advances_and_matches_model(self):
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
            await asyncio.sleep(0.05)
            _, tail = capture.read_capture(
                self.capture_path, max(0, capture.length(self.capture_path) - 1)
            )
            if tail and tail[-1]["tick"] > before:
                break
        else:
            raise RuntimeError("Native recording is not advancing")
        # The local model must be the runtime's, including an installed correction.
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

    # ------------------------------------------------------------ telemetry

    async def verify_capture_source(self):
        reply = await self.core.capture_info()
        identity = None if reply is None else reply.get("identity")
        if identity is None:
            raise RuntimeError(
                "Connected runtime has no native recorder; start par6d with "
                "PAR6_DIAGNOSTICS=<file>"
            )
        if self.capture_identity is not None and identity != self.capture_identity:
            raise RuntimeError("Connected runtime recorder changed during calibration")
        capture.assert_live(
            self.capture_path, self.bundle["fingerprint"], expected_identity=identity
        )
        self.capture_identity = identity

    def _require_ready(self, s):
        if (
            s["error"]
            or not s["homed"]
            or not s["enabled"]
            or s["link_ok"] != 1
            or feedback_age(s) > FEEDBACK_MAX_AGE_S
        ):
            raise RuntimeError(
                f"Calibration lost readiness: error={s.get('error')}, mode={s['mode']}, "
                f"homed={s['homed']}, enabled={s['enabled']}, link={s['link_ok']}, "
                f"feedback_age_ms={1000 * feedback_age(s):.0f}"
            )
        if hasattr(self, "world_epoch") and s["scene_epoch"] != self.world_epoch:
            raise RuntimeError("Collision world changed during calibration")
        if any(s["drive_health"]["faults"]):
            raise RuntimeError("Drive fault during calibration")

    async def fresh(self):
        """The next STATUS packet, ready-checked."""
        s = await self.core.status_after(self.sequence, SILENCE_S)
        if s is None:
            raise RuntimeError("Controller telemetry stopped")
        latest = self.core.latest_status()
        if latest is not None and latest["seq"] > s["seq"]:
            s = latest
        self.sequence = s["seq"]
        self._require_ready(s)
        return s

    def latest(self):
        """The newest packet already received, ready-checked (no waiting)."""
        s = self.core.latest_status()
        if s is None:
            raise RuntimeError("No controller telemetry")
        self._require_ready(s)
        return s

    async def idle_ready(self, *, gravity: bool | None = None):
        s = await self.fresh()
        if (
            s["mode"] != ControllerMode.IDLE
            or s["queued_segments"]
            or s["executing_index"] >= 0
        ):
            raise RuntimeError("The arm must be idle with an empty queue")
        if gravity is not None and bool(s["gravity_comp"]) != gravity:
            raise RuntimeError(
                f"Gravity feedforward must be {'on' if gravity else 'off'} for this stimulus"
            )
        return s

    async def stop(self):
        """Acknowledged Stop, then measured encoder rest."""
        if await self.client.stop() != 1:
            raise RuntimeError("Controller stop was not acknowledged")
        deadline = time.monotonic() + STOP_BUDGET_S
        stable = None
        while time.monotonic() < deadline:
            s = await self.core.status_after(self.sequence, SILENCE_S)
            if s is None:
                raise RuntimeError(
                    "Stop acknowledged but encoder rest could not be confirmed"
                )
            self.sequence = s["seq"]
            if (
                feedback_age(s) <= FEEDBACK_MAX_AGE_S
                and s["link_ok"] == 1
                and max(abs(v) for v in s["speeds"]) < REST_SPEED_RAD_S
                and not s["queued_segments"]
            ):
                stable = stable or time.monotonic()
                if time.monotonic() - stable >= REST_HOLD_S:
                    return
            else:
                stable = None
        raise RuntimeError("Stop acknowledged but the arm did not settle")

    async def set_gravity(self, on: bool):
        if await self.client.set_gravity_comp(on) != 1:
            raise RuntimeError("Gravity feedforward change was not acknowledged")
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            if bool((await self.fresh())["gravity_comp"]) == on:
                return
        raise RuntimeError("Gravity feedforward change was not observed")

    # ------------------------------------------------------------ geometry

    def check_path(self, positions):
        q = np.asarray(positions, dtype=float)
        if q.ndim != 2 or q.shape[1] != 6 or len(q) < 2:
            raise ValueError("Path requires at least two six-joint poses")
        if (
            not np.isfinite(q).all()
            or np.any(q < self.window[:, 0])
            or np.any(q > self.window[:, 1])
        ):
            raise ValueError("Calibration path exceeds joint window")
        dense = [q[0]]
        for a, b in zip(q, q[1:]):
            steps = max(1, int(np.ceil(np.max(np.abs(b - a)) / 0.01)))
            dense.extend(np.linspace(a, b, steps + 1)[1:])
        bad = self.world.check_path(np.asarray(dense).tolist())
        if bad >= 0:
            raise ValueError(
                f"Calibration path collides: {self.world.pairs(list(dense[bad]))}"
            )

    def position_settings(self, *, diagnostic=False):
        """Approach limits and the matching native stream fractions."""
        desired = [0.05, 0.08, 0.3] if diagnostic else [0.2, 0.4, 1.2]
        base = np.minimum(np.minimum(self.limits, self.stream_limits), desired)
        return base, stream_scale(self.robot, base)

    def plan_position(self, a, b, *, diagnostic=False):
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
        """Slow streamed approach to `q`; tracking and readiness still apply."""
        s = await self.idle_ready(gravity=False)
        a = np.deg2rad(s["angles"])
        times, positions = self.plan_position(a, q, diagnostic=diagnostic)
        base, scale = self.position_settings(diagnostic=diagnostic)
        # An approach only has to arrive; the probe that follows judges
        # tracking. A loaded wrist settles slowly on position feedback alone.
        _, metrics = await self.stimulus(
            "position",
            times,
            positions,
            command_limits=base,
            **scale,
            allow_oscillation=diagnostic,
            settle_tolerance=0.02,
            settle_timeout=4.0,
        )
        if max(metrics["tracking_peak_deg"]) > 2:
            raise TrialRejected("Calibration approach did not track correctly")

    # ------------------------------------------------------------ stimuli

    def _watch(self, ilim, saturation_since):
        """One safety observation from the newest packet; returns the
        updated saturation-start time."""
        s = self.latest()
        current = np.abs(np.asarray(s["drive_health"]["currents_ma"][:6], dtype=float))
        if np.any(current >= 0.95 * ilim):
            saturation_since = saturation_since or time.monotonic()
            if time.monotonic() - saturation_since > SATURATION_S:
                raise TrialRejected("Sustained drive-current saturation")
            return saturation_since
        return None

    def _settle_diagnosis(self, target) -> str:
        """Why a streamed target was not reached: the controller never
        followed the targets (a refused stream, a gate standing the arm off)
        reads differently from an arm that is still moving."""
        _, tail = capture.read_capture(
            self.capture_path, max(0, capture.length(self.capture_path) - 1)
        )
        s = self.core.latest_status() or {}
        if tail:
            short = np.max(np.abs(np.asarray(tail[-1]["q_commanded"]) - target))
            if short > 0.05:
                return (
                    "controller did not follow the streamed targets: commanded position "
                    f"{np.rad2deg(short):.1f} deg short of the target; warnings: "
                    f"{s.get('warnings')}"
                )
        error = np.rad2deg(np.max(np.abs(np.deg2rad(s.get("angles", target)) - target)))
        speed = max((abs(v) for v in s.get("speeds", [0.0])), default=0.0)
        return (
            f"Calibration target did not settle ({error:.2f} deg off, "
            f"{speed:.3f} rad/s)"
        )

    def _record(self, report):
        self.trials.append(report)
        atomic_json(self.directory / "trials.json", {"trials": self.trials})

    def _analyze(
        self, report, begin, end, bounds, *, allow_oscillation, modes, policy=None
    ):
        """Post-Stop evidence from the recording; raises TrialRejected on failure."""
        try:
            dt, rows = capture.read_capture(self.capture_path, begin, end)
            rows = capture.measurement_rows(
                capture.active_rows(rows, modes),
                dt,
                simulator=self.identity["simulator"],
            )
            gravity_flags = {bool(r["flags"] & capture.FLAG_GRAVITY) for r in rows}
            if len(gravity_flags) != 1:
                raise RuntimeError("Gravity feedforward changed during the trial")
            report["gravity_comp"] = gravity_flags.pop()
            metrics = assess_motion(
                rows, dt, policy or self.policy, allow_oscillation=allow_oscillation
            )
        except (ValueError, RuntimeError) as exc:
            report["error"] = str(exc)
            self._record(report)
            raise
        report["metrics"] = metrics
        if bounds is not None:
            try:
                check_command_peaks(
                    metrics["commanded_peaks"],
                    bounds,
                    dt,
                    label="Recorded native stream",
                )
            except ValueError as exc:
                report["error"] = str(exc)
                self._record(report)
                raise TrialRejected(str(exc)) from exc
        if not metrics["acceptance"]["valid"]:
            report["error"] = "; ".join(metrics["acceptance"]["reasons"])
            self._record(report)
            raise TrialRejected(report["error"])
        self._record(report)
        return rows, metrics

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
        settle_tolerance=None,
        settle_timeout=None,
        **metadata,
    ):
        """Stream `positions` at `times` as servo targets (STREAM path, gravity
        feedforward off, position feedback on), Stop, then judge the recording."""
        times, positions = (
            np.asarray(times, dtype=float),
            np.asarray(positions, dtype=float),
        )
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
        await self.idle_ready(gravity=False)
        ilim = np.asarray([j["ilim_ma"] for j in self.robot["joints"]], dtype=float)
        motion = self.robot.get("motion", {})
        if settle_tolerance is None:
            settle_tolerance = motion.get("settle_tolerance_rad", 0.01)
        if settle_timeout is None:
            settle_timeout = motion.get("settle_timeout_s", 2.0)
        begin = capture.length(self.capture_path)
        report = {
            **metadata,
            "name": name,
            "tested_mode": "STREAM",
            "capture_start": begin,
            "capture_end": None,
            "error": None,
            "allow_oscillation": allow_oscillation,
            "predicted_command_peaks": peaks,
            "stream_scale": {"speed": speed, "accel": accel},
        }
        saturation_since = None
        error = None
        try:
            start = time.monotonic()
            for t, q in zip(times, positions):
                wait = start + float(t) - time.monotonic()
                if wait > 0:
                    await asyncio.sleep(wait)
                elif wait < -FEEDBACK_MAX_AGE_S:
                    raise RuntimeError(
                        "Command scheduling fell behind by more than 250 ms"
                    )
                saturation_since = self._watch(ilim, saturation_since)
                await self.client.servo_j(
                    np.rad2deg(q).tolist(), speed=speed, accel=accel
                )
            if settle:
                deadline = time.monotonic() + settle_timeout
                stable = None
                target_deg = np.rad2deg(positions[-1]).tolist()
                while time.monotonic() < deadline:
                    await asyncio.sleep(STREAM_DT_S)
                    saturation_since = self._watch(ilim, saturation_since)
                    await self.client.servo_j(target_deg, speed=speed, accel=accel)
                    s = self.latest()
                    err = np.max(np.abs(np.deg2rad(s["angles"]) - positions[-1]))
                    if (
                        err <= settle_tolerance
                        and max(abs(v) for v in s["speeds"]) < REST_SPEED_RAD_S
                    ):
                        stable = stable or time.monotonic()
                        if time.monotonic() - stable > REST_HOLD_S:
                            break
                    else:
                        stable = None
                else:
                    raise TrialRejected(self._settle_diagnosis(positions[-1]))
        except BaseException as exc:
            error = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            try:
                await finish_cleanup(self.stop())
            except BaseException as exc:
                error = f"{error or ''}; stop: {type(exc).__name__}: {exc}"
                raise
            finally:
                report["capture_end"] = capture.length(self.capture_path)
                report["error"] = error
                if error is not None:
                    self._record(report)
        await self.verify_capture_source()
        rows, metrics = self._analyze(
            report,
            begin,
            report["capture_end"],
            bounds,
            allow_oscillation=allow_oscillation,
            modes=(capture.MODE_STREAM,),
        )
        return rows, metrics

    async def queued_move(
        self,
        target,
        *,
        speed=1.0,
        accel=1.0,
        timeout=30.0,
        allow_oscillation=False,
        policy=None,
        **metadata,
    ):
        """One `move_j` through the ordinary EXEC planner at the given speed and
        acceleration fractions, watched to COMPLETE, then Stop and judged."""
        target = np.asarray(target, dtype=float)
        s = await self.idle_ready()
        here = np.deg2rad(s["angles"])
        self.check_path([here, target])
        await self.verify_capture_source()
        ilim = np.asarray([j["ilim_ma"] for j in self.robot["joints"]], dtype=float)
        begin = capture.length(self.capture_path)
        report = {
            **metadata,
            "name": "move_j",
            "tested_mode": "EXEC",
            "capture_start": begin,
            "capture_end": None,
            "error": None,
            "speed": speed,
            "accel": accel,
            "target_rad": target.tolist(),
        }
        error = None
        saturation_since = None
        started = time.monotonic()
        try:
            index = await self.client.move_j(
                np.rad2deg(target).tolist(), speed=speed, accel=accel
            )
            if index < 0:
                raise RuntimeError("move_j was not accepted")
            deadline = started + timeout
            while True:
                if await self.core.wait_command(index, 0.05):
                    break
                saturation_since = self._watch(ilim, saturation_since)
                if time.monotonic() > deadline:
                    raise TrialRejected(f"move_j did not complete within {timeout:g} s")
            verdict = await self.client.command_verdict(index)
            report["complete_s"] = time.monotonic() - started
            report["verdict"] = verdict
        except BaseException as exc:
            error = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            try:
                await finish_cleanup(self.stop())
            except BaseException as exc:
                error = f"{error or ''}; stop: {type(exc).__name__}: {exc}"
                raise
            finally:
                report["capture_end"] = capture.length(self.capture_path)
                report["error"] = error
                if error is not None:
                    self._record(report)
        await self.verify_capture_source()
        return self._analyze(
            report,
            begin,
            report["capture_end"],
            None,
            allow_oscillation=allow_oscillation,
            modes=(capture.MODE_EXEC,),
            policy=policy,
        )

    # ------------------------------------------------------------ gains

    @contextlib.asynccontextmanager
    async def gains(self, joint, *, kpv_scale=1.0, kiv_scale=1.0):
        """Volatile velocity-loop gains for one joint, restored on every exit.

        Position gain, current-loop gains and every limit are preserved. The
        controller acknowledges the request; drives do not read gains back.
        """
        if joint not in range(6) or not all(
            np.isfinite(v) and 0.2 <= v <= 1.0 for v in (kpv_scale, kiv_scale)
        ):
            raise ValueError(
                "Feedback tuning only lowers velocity-loop gains, to 20 % at most"
            )
        j = self.robot["joints"][joint]
        original = {
            **j["gains"],
            "ilim_ma": j["ilim_ma"],
            "velocity_limit_ticks_s": j["velocity_limit_ticks_s"],
            "voltage_limit_mv": j.get("voltage_limit_mv", 0),
        }
        candidate = {
            **original,
            "kpv": original["kpv"] * kpv_scale,
            "kiv": original["kiv"] * kiv_scale,
        }
        record = {
            "joint": joint,
            "baseline": original,
            "candidate": candidate,
            "restored": False,
        }
        path = self.directory / f"gains-J{joint + 1}.json"

        async def push(values):
            await self.stop()
            if await self.client.set_pid_gains(j["node_id"], **values) != 1:
                raise RuntimeError("Controller did not acknowledge the gain update")
            # The stored-config frames repeat; let them drain before excitation.
            for _ in range(10):
                await self.fresh()

        atomic_json(path, record)
        try:
            await push(candidate)
            yield candidate
        finally:

            async def restore():
                try:
                    await push(original)
                    record["restored"] = True
                except BaseException as exc:
                    record["restore_error"] = f"{type(exc).__name__}: {exc}"
                    raise
                finally:
                    atomic_json(path, record)

            await finish_cleanup(restore())
