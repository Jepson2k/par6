"""Verify an activated envelope through queued RUCKIG moves and endpoint holds."""

from __future__ import annotations

import asyncio
import gc
import time
from pathlib import Path

import numpy as np

from par6 import config as paths
from par6._par6 import CompletionPolicy, ControllerMode, Preview

from . import capture
from .preflight import check_command_peaks
from .profiles import atomic_json, validate_profile, verify_applied
from .routines import verification_centers
from .session import TrialRejected, finish_cleanup
from .trajectory import envelope_move
from .validation import assess_motion, measured_excitation

HOLD_SECONDS = 2.0


def execution_rows(rows):
    """Only initial IDLE is disposable; an interrupted EXEC tail is evidence."""
    modes = np.array([r["flags"] >> 8 for r in rows])
    if any(r["flags"] & 1 for r in rows):
        raise ValueError("Controller fault in queued-motion capture")
    active = np.flatnonzero(modes == ControllerMode.EXEC)
    if not len(active):
        raise ValueError("Capture contains no queued execution")
    first = int(active[0])
    if np.any(modes[:first] != ControllerMode.IDLE) or np.any(
        modes[first:] != ControllerMode.EXEC
    ):
        raise ValueError("Controller left EXEC before the endpoint hold finished")
    return rows[first:]


def assess_execution(rows, dt, policy, target, limits, required):
    """Require fixed-target hold quality and actual v/a/j for this exact trial."""
    target, limits, required = map(np.asarray, (target, limits, required))
    if (
        target.shape != (6,)
        or not np.isfinite(target).all()
        or limits.shape != (6, 3)
        or not np.isfinite(limits).all()
        or np.any(limits <= 0)
        or required.shape != (6,)
        or required.dtype != bool
        or not required.any()
    ):
        raise ValueError("Invalid queued-verification target, limits or joint mask")
    rows = execution_rows(rows)
    metrics = assess_motion(rows, dt, policy)
    reasons = metrics["acceptance"]["reasons"]
    try:
        check_command_peaks(
            metrics["commanded_peaks"], limits, dt, label="Recorded queued motion"
        )
    except ValueError as exc:
        reasons.append(str(exc))
    achieved = measured_excitation(metrics, rows, policy)
    if np.any(achieved[required] < 0.9 * limits[required]):
        reasons.append("Queued move did not exercise every required derivative")
    times = np.array([r["sample_time_ns"] for r in rows], dtype=np.int64)
    qc_all = np.asarray([r["q_commanded"] for r in rows])
    qdc_all = np.asarray([r["qd_commanded"] for r in rows])
    fixed = np.all(np.abs(qc_all - target) <= 1e-6, axis=1) & np.all(
        np.abs(qdc_all) <= 1e-6, axis=1
    )
    moving = np.flatnonzero(~fixed)
    terminal_start = int(moving[-1]) + 1 if len(moving) else 0
    if terminal_start >= len(rows) - 1:
        raise ValueError("Capture lacks a terminal fixed-target hold")
    terminal_seconds = (times[-1] - times[terminal_start]) * 1e-9
    if terminal_seconds < HOLD_SECONDS - 1.1 * dt:
        raise ValueError("Insufficient recorded fixed-target hold")
    terminal_q = np.asarray([r["q"] for r in rows[terminal_start:]])
    terminal_excursion = np.rad2deg(np.ptp(terminal_q, axis=0))
    quantum = np.rad2deg(np.asarray(policy["encoder_quantum_rad"]))
    # Assess the entire native endpoint plateau, including settling before
    # the observed COMPLETE. A quiet final tail cannot erase earlier droop.
    if np.any(
        terminal_excursion > np.maximum(policy["gravity_excursion_deg"], 4 * quantum)
    ):
        reasons.append("Terminal endpoint hold excursion exceeds acceptance")
    hold = [
        r for r, t in zip(rows, times) if t >= times[-1] - round(HOLD_SECONDS * 1e9)
    ]
    seconds = (
        np.array([r["sample_time_ns"] for r in hold]) - hold[0]["sample_time_ns"]
    ) * 1e-9
    if seconds[-1] < HOLD_SECONDS - 1.1 * dt:
        raise ValueError("Insufficient recorded endpoint hold")
    q = np.array([r["q"] for r in hold])
    qc = np.array([r["q_commanded"] for r in hold])
    qdc = np.array([r["qd_commanded"] for r in hold])
    if not np.allclose(qc, target, atol=1e-6, rtol=0) or np.max(np.abs(qdc)) > 1e-6:
        reasons.append("The native queued target was not fixed throughout the hold")
    error = np.rad2deg(np.max(np.abs(q - target), axis=0))
    excursion = np.rad2deg(np.ptp(q, axis=0))
    centered = seconds - seconds.mean()
    slope = np.rad2deg(
        np.sum(centered[:, None] * (q - q.mean(axis=0)), axis=0) / np.sum(centered**2)
    )
    quantum = np.rad2deg(np.asarray(policy["encoder_quantum_rad"]))
    if np.any(error > policy["tracking_deg"]):
        reasons.append("Endpoint hold missed the requested position")
    if np.any(excursion > np.maximum(policy["gravity_excursion_deg"], 4 * quantum)):
        reasons.append("Endpoint hold excursion exceeds acceptance")
    if np.any(
        np.abs(slope)
        > np.maximum(policy["gravity_drift_deg_s"], 4 * quantum / seconds[-1])
    ):
        reasons.append("Endpoint hold has sustained position drift")
    hold_quality = assess_motion(hold, dt, policy)
    reasons.extend(
        "Hold: " + reason for reason in hold_quality["acceptance"]["reasons"]
    )
    metrics["hold"] = {
        "duration_s": float(seconds[-1]),
        "terminal_start_tick": rows[terminal_start]["tick"],
        "terminal_duration_s": float(terminal_seconds),
        "terminal_excursion_deg": terminal_excursion.tolist(),
        "error_deg": error.tolist(),
        "excursion_deg": excursion.tolist(),
        "drift_deg_s": slope.tolist(),
        "quality": hold_quality,
    }
    metrics["achieved"] = achieved.tolist()
    metrics["acceptance"]["valid"] = not reasons
    return metrics


async def execution_preview(session):
    # Retain every native position for these bounded strokes. A truncated
    # preview is refused below rather than assigned fictitious tick times.
    preview = await asyncio.to_thread(
        Preview,
        config=str(session.directory / session.bundle["robot_filename"]),
        assets=str(paths.data_root()),
        package_dir=str(paths.package_search_dir()),
        max_points=100_000,
    )
    selected = preview.submit({"type": "select_profile", "profile": "RUCKIG"})
    if selected is None or selected["error"] is not None or selected["pending"]:
        raise RuntimeError(f"Preview could not select RUCKIG: {selected}")
    return preview


def plan_execution(session, preview, start, target):
    session.check_path([start, target])
    preview.teleport_rad(np.asarray(start).tolist())
    result = preview.submit(
        {
            "type": "move_j",
            "angles": np.rad2deg(target).tolist(),
            "speed": 1.0,
            "accel": 1.0,
            "blend_radius": 0.0,
            "rel": False,
        }
    )
    if result is None or result["error"] is not None or result["pending"]:
        raise ValueError(f"Native queued preflight refused: {result}")
    duration = result["duration_s"]
    positions = np.asarray(result["joint_trajectory_rad"])
    if (
        not np.isfinite(duration)
        or duration <= 0
        or positions.ndim != 2
        or positions.shape[1] != 6
        or len(positions) < 2
        or len(positions) + 1 < duration / preview.tick_dt_s()
        or not np.allclose(result["end_joints_rad"], target, atol=1e-6, rtol=0)
    ):
        raise ValueError("Queued preflight lacks a complete nonzero native path")
    session.check_path(np.vstack((start, positions)))
    return float(duration)


async def queued_trial(session, preview, target, required, **metadata):
    """Observe queued motion in simulation while support handoffs are validated."""
    if session.identity.get("simulator") is not True:
        raise RuntimeError(
            "Queued verification is simulator-only until supported gravity handoffs "
            "are validated; ordinary Stop with gravity enabled can release feedback"
        )
    target = np.asarray(target, dtype=float)
    required = np.asarray(required)
    if (
        target.shape != (6,)
        or not np.isfinite(target).all()
        or required.shape != (6,)
        or required.dtype != bool
        or not required.any()
    ):
        raise ValueError(
            "Queued verification requires a finite target and boolean joint mask"
        )
    await session.verify_capture_source()
    status = await session.fresh()
    if (
        status["mode"] != ControllerMode.IDLE
        or status["queued_segments"]
        or status["executing_index"] >= 0
    ):
        raise RuntimeError("Queued verification requires idle with an empty queue")
    if not status["gravity_comp"]:
        raise RuntimeError("Queued verification requires enabled gravity compensation")
    if await session.client.profile() != "RUCKIG":
        raise RuntimeError("Queued verification requires the RUCKIG profile")
    if await session.client.set_completion_policy(CompletionPolicy.STRICT) != 1:
        raise RuntimeError("Strict queued completion was not acknowledged")
    start_q = np.deg2rad(status["angles"])
    if np.max(np.abs(start_q - target)) <= 4 * max(
        session.policy["encoder_quantum_rad"]
    ):
        raise ValueError("A zero-distance queued move cannot validate an envelope")
    duration = plan_execution(session, preview, start_q, target)
    await session.fresh()
    begin = capture.length(session.capture_path)
    report = {
        **metadata,
        "name": "exec-validation",
        "capture_start": begin,
        "start": start_q.tolist(),
        "target": target.tolist(),
        "required_joints": np.flatnonzero(required).tolist(),
        "duration_s": duration,
        "profile": "RUCKIG",
        "gravity_comp": bool(status["gravity_comp"]),
        "completion_policy": "STRICT",
        "error": None,
        "stop_confirmed": False,
        "software_estop_latched": False,
    }
    submission = asyncio.create_task(
        session.client.move_j(
            np.rad2deg(target).tolist(), speed=1.0, accel=1.0, r=0.0, wait=False
        )
    )
    completion = analysis = None
    index = None
    completed_at = None
    hold_min = hold_max = None
    seen_exec = False
    observed_indices = set()
    started = time.monotonic()
    budget = max(
        5.0, duration + session.robot.get("motion", {}).get("settle_timeout_s", 2) + 1.0
    )
    next_analysis = started + 1.2
    analysis_started = started
    saturated_at = None
    end = begin
    restore_gc = gc.isenabled()
    if restore_gc:
        gc.disable()

    def recent_quality():
        session.assert_capture_live()
        last = capture.length(session.capture_path)
        dt, rows = capture.read_capture(
            session.capture_path,
            max(begin, last - round(1.2 / session.robot["robot"]["tick_dt_s"])),
            last,
        )
        if not any(r["flags"] >> 8 == ControllerMode.EXEC for r in rows):
            return None
        rows = capture.measurement_rows(
            execution_rows(rows), dt, simulator=session.identity["simulator"]
        )
        if len(rows) * dt < 1.0:
            return None
        return assess_motion(rows, dt, session.policy)["acceptance"]

    try:
        while True:
            status = await session.fresh()
            now = time.monotonic()
            if index is None and submission.done():
                index = submission.result()
                if index < 0:
                    raise RuntimeError("Queued motion acknowledgement was unconfirmed")
                report["command_index"] = index
                completion = asyncio.create_task(
                    session.client.wait_command(
                        index, timeout=max(0.1, started + budget - now)
                    )
                )
            if completion is not None and completion.done() and completed_at is None:
                if completion.result() is not True:
                    raise TimeoutError(
                        "Queued move did not confirm its exact completion"
                    )
                completed_at = now
                report["completion_observed_monotonic_s"] = now
                # This is a buffered writer cursor, not the native completion tick.
                report["capture_at_completion_observation"] = capture.length(
                    session.capture_path
                )
            mode = status["mode"]
            if bool(status["gravity_comp"]) != report["gravity_comp"]:
                raise RuntimeError(
                    "Gravity feedforward changed during queued verification"
                )
            if mode == ControllerMode.EXEC:
                seen_exec = True
                if status["executing_index"] >= 0:
                    observed_indices.add(status["executing_index"])
            elif seen_exec or mode != ControllerMode.IDLE:
                raise RuntimeError(
                    "Controller left EXEC before verification hold completed"
                )
            if index is not None and observed_indices - {index}:
                raise RuntimeError("Another command entered queued verification")
            if status["queued_segments"] > 1:
                raise RuntimeError("Unexpected extra queued motion during verification")
            current = np.asarray(status["drive_health"]["currents_ma"][:6])
            if current.shape != (6,) or not np.isfinite(current).all():
                raise RuntimeError("Missing measured motor currents")
            saturated = np.any(
                np.abs(current)
                >= 0.95 * np.array([j["ilim_ma"] for j in session.robot["joints"]])
            )
            saturated_at = (
                (now if saturated_at is None else saturated_at) if saturated else None
            )
            if (
                saturated_at is not None
                and now - saturated_at > session.policy["saturation_s"]
            ):
                raise TrialRejected(
                    "Sustained drive-current saturation during queued motion"
                )
            if completed_at is not None:
                if mode != ControllerMode.EXEC:
                    raise RuntimeError("Completed move did not preserve EXEC hold")
                q_hold = np.deg2rad(status["angles"])
                hold_min = q_hold if hold_min is None else np.minimum(hold_min, q_hold)
                hold_max = q_hold if hold_max is None else np.maximum(hold_max, q_hold)
                report["observed_post_completion_excursion_deg"] = np.rad2deg(
                    hold_max - hold_min
                ).tolist()
                allowance = np.maximum(
                    np.deg2rad(session.policy["gravity_excursion_deg"]),
                    4 * np.asarray(session.policy["encoder_quantum_rad"]),
                )
                if np.any(hold_max - hold_min > allowance):
                    raise TrialRejected(
                        "Post-completion endpoint excursion exceeds acceptance"
                    )
                if np.max(np.abs(q_hold - target)) > np.deg2rad(
                    session.policy["tracking_deg"]
                ):
                    raise TrialRejected(
                        "Completed queued move drifted from its endpoint"
                    )
            if analysis is not None:
                if analysis.done():
                    quality = analysis.result()
                    analysis = None
                    if quality is not None and not quality["valid"]:
                        raise TrialRejected("; ".join(quality["reasons"]))
                elif now - analysis_started > 0.5:
                    raise RuntimeError("Queued live analysis missed its 0.5 s deadline")
            if analysis is None and now >= next_analysis:
                analysis_started = now
                analysis = asyncio.create_task(asyncio.to_thread(recent_quality))
                next_analysis = now + 0.5
            # The extra capture-freshness interval leaves a full recorded hold
            # despite the writer's buffered tail. Final analysis checks its span.
            if completed_at is not None and now - completed_at >= HOLD_SECONDS + 0.5:
                if observed_indices != {index}:
                    raise RuntimeError(
                        "No status evidence for the accepted command index"
                    )
                break
            if now - started > budget + HOLD_SECONDS + 0.5:
                raise TimeoutError("Queued verification exceeded its completion budget")
        if analysis is not None:
            quality = await asyncio.wait_for(
                analysis, max(0.0, 0.5 - (time.monotonic() - analysis_started))
            )
            analysis = None
            if quality is not None and not quality["valid"]:
                raise TrialRejected("; ".join(quality["reasons"]))
        if await session.client.profile() != "RUCKIG":
            raise RuntimeError("Queued profile changed during verification")
        session.assert_capture_live()
        end = capture.length(session.capture_path)
        report["post_completion_observation_s"] = time.monotonic() - completed_at
    except BaseException as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if analysis is not None:
            analysis.cancel()

        async def cleanup():
            # Never cancel a pending native submission and assume its retries
            # stopped. A latched stop fences uncertain submissions while they drain.
            errors = []

            async def attempt(label, operation):
                try:
                    await operation
                except (Exception, asyncio.CancelledError) as exc:
                    errors.append(f"{label}: {type(exc).__name__}: {exc}")

            uncertain = not submission.done()
            if submission.done():
                try:
                    uncertain = submission.result() < 0
                except (Exception, asyncio.CancelledError):
                    uncertain = True

            async def fence():
                if await session.client.estop() != 1:
                    raise RuntimeError(
                        "Uncertain queued submission could not be fenced by software EStop"
                    )
                report["software_estop_latched"] = True

            async def drain_submission():
                try:
                    await asyncio.shield(submission)
                except (Exception, asyncio.CancelledError) as exc:
                    report["submission_error"] = f"{type(exc).__name__}: {exc}"
                    raise

            try:
                if uncertain:
                    await attempt("software EStop", fence())
                if report["error"] is not None:
                    await attempt("immediate Stop", session.stop())
                await attempt("submission", drain_submission())
            finally:
                if completion is not None and not completion.done():
                    completion.cancel()
                    await asyncio.gather(completion, return_exceptions=True)
                try:
                    await session.stop()
                    report["stop_confirmed"] = True
                except (Exception, asyncio.CancelledError) as exc:
                    errors.append(f"final Stop: {type(exc).__name__}: {exc}")
            if errors:
                report["cleanup_errors"] = errors
                raise RuntimeError("; ".join(errors))

        try:
            await finish_cleanup(cleanup())
        except BaseException as exc:
            report["stop_error"] = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            if restore_gc:
                gc.enable()
            report["capture_end"] = (
                end if end > begin else capture.length(session.capture_path)
            )
            session.trials.append(report)
            atomic_json(session.directory / "trials.json", {"trials": session.trials})
    try:
        await session.verify_capture_source()
        dt, rows = capture.read_capture(session.capture_path, begin, end)
        rows = capture.measurement_rows(
            rows, dt, simulator=session.identity["simulator"]
        )
        if any(bool(r["flags"] & 16) != report["gravity_comp"] for r in rows):
            raise RuntimeError(
                "Recorded gravity feedforward changed during queued verification"
            )
        metrics = assess_execution(
            rows,
            dt,
            session.policy,
            target,
            session.limits,
            required,
        )
        report["metrics"] = metrics
        if not metrics["acceptance"]["valid"]:
            raise TrialRejected("; ".join(metrics["acceptance"]["reasons"]))
    except BaseException as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        atomic_json(session.directory / "trials.json", {"trials": session.trials})
    return rows, metrics


async def verify_execution(session, candidate):
    """Validate an activated EXEC candidate; leaves completion policy STRICT.

    Currently restricted to simulation: ordinary G-enabled Stop can release
    feedback. The supported entry/exit protocol still requires validation.
    This measures queued closed-loop performance, including the endpoint hold.
    It does not identify inertial parameters or certify other motion profiles,
    JOG, table vibration, untested poses or different payloads.
    """
    report = {
        "kind": "execution-verification",
        "protocol_version": 1,
        "valid": False,
        "complete": False,
        "applied_validation_complete": False,
        "tested_mode": "EXEC",
        "tested_profile": "RUCKIG",
        "identity": session.identity,
        "baseline_fingerprint": session.fingerprint,
        "measurements": [],
        "reasons": [],
        "status": "running",
        "table_vibration_measured": False,
        "inertial_parameters_identified": False,
        "scope": "tested single-axis/coupled poses, configured empty gripper, RUCKIG EXEC",
    }
    path = session.directory / "execution-verification.json"
    atomic_json(path, report)
    try:
        if session.identity.get("simulator") is not True:
            raise RuntimeError(
                "Queued verification is simulator-only until supported gravity "
                "handoffs are validated"
            )
        candidate = Path(candidate)
        manifest = validate_profile(candidate)
        source = manifest["report"]
        if (
            candidate.parent.name != "candidate"
            or source.get("kind") != "motion-envelope"
            or source.get("valid") is not True
            or source.get("complete") is not True
            or source.get("tested_mode") != "STREAM"
        ):
            raise ValueError("Choose a completed motion-envelope candidate profile")
        if not np.array_equal(np.asarray(source["limits"]), session.limits):
            raise ValueError(
                "Loaded EXEC caps differ from the staged measured envelope"
            )
        await verify_applied(session.client, candidate)
        if await session.client.profile() != "RUCKIG":
            raise RuntimeError("Select RUCKIG before verifying its operating envelope")
        report["candidate"] = str(candidate.resolve())
        report["limits"] = session.limits.tolist()
        if not (await session.fresh())["gravity_comp"]:
            raise RuntimeError("Enable gravity compensation before queued verification")
        report["gravity_comp"] = True
        preview = await execution_preview(session)
        plans = []
        for group, center in enumerate(verification_centers(session.start)[:3]):
            extents = []
            for joint in range(6):
                _, pulse = envelope_move(
                    center,
                    joint,
                    0,
                    session.limits[joint, 0],
                    session.limits,
                    session.window,
                )
                extents.append((pulse[0][joint], pulse[-1][joint]))
                mask = np.arange(6) == joint
                for direction in (-1, 1):
                    a, b = (
                        (pulse[0], pulse[-1])
                        if direction == 1
                        else (pulse[-1], pulse[0])
                    )
                    plan_execution(session, preview, a, b)
                    for repeat in range(3):
                        plans.append(
                            (
                                a,
                                b,
                                mask,
                                {
                                    "group": group,
                                    "joint": joint,
                                    "direction": direction,
                                    "repeat": repeat,
                                    "held_out": group == 2,
                                    "coupled": False,
                                },
                            )
                        )
            if group == 2:
                for pattern, signs in enumerate(
                    (np.ones(6), np.array([1, -1, 1, -1, 1, -1]))
                ):
                    radius = (np.array(extents)[:, 1] - np.array(extents)[:, 0]) / 2
                    for direction in (-1, 1):
                        a, b = (
                            center - radius * signs * direction,
                            center + radius * signs * direction,
                        )
                        plan_execution(session, preview, a, b)
                        for repeat in range(3):
                            plans.append(
                                (
                                    a,
                                    b,
                                    np.ones(6, dtype=bool),
                                    {
                                        "group": group,
                                        "pattern": pattern,
                                        "direction": direction,
                                        "repeat": repeat,
                                        "held_out": True,
                                        "coupled": True,
                                    },
                                )
                            )
        for a, b, required, metadata in plans:
            await session.position(a)
            _, metrics = await queued_trial(session, preview, b, required, **metadata)
            report["measurements"].append(
                {
                    **metadata,
                    "metrics": metrics,
                    "command_index": session.trials[-1]["command_index"],
                    "capture_start": session.trials[-1]["capture_start"],
                    "capture_end": session.trials[-1]["capture_end"],
                }
            )
            atomic_json(path, report)
        await session.position(session.start)
        if not (await session.fresh())["gravity_comp"]:
            raise RuntimeError(
                "Gravity compensation changed before verification finished"
            )
        if await session.client.profile() != "RUCKIG":
            raise RuntimeError("Queued profile changed before verification finished")
        report.update(
            valid=True,
            complete=True,
            applied_validation_complete=True,
            status="verified",
        )
        return report
    except BaseException as exc:
        report.update(
            valid=False,
            complete=False,
            applied_validation_complete=False,
            status="failed",
        )
        report["reasons"].append(f"{type(exc).__name__}: {exc}")
        raise
    finally:
        atomic_json(path, report)
