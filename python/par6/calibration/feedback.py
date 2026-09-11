"""Bounded outer-loop diagnosis; current/electrical gains are preserved."""

from __future__ import annotations

from contextlib import asynccontextmanager

import numpy as np

from . import capture
from .analysis import motion_metrics, motion_window_peaks
from .profiles import atomic_json, export_profile
from .session import TrialRejected, finish_cleanup
from .trajectory import move
from .validation import motion_acceptance

QUALITY_PHASES = ("motion", "hold", "window_peaks")


@asynccontextmanager
async def temporary_feedback(
    session, joint, *, kpp_scale=1.0, velocity_scale=1.0, integral_scale=None
):
    """Restore the startup tuple on every exit, including cancellation.

    CAN acknowledges the controller request, not a drive parameter readback.
    This session must have exclusive control, starting from its loaded config.
    Changes are volatile: no drive save/configuration flash is requested.
    """
    integral_scale = velocity_scale if integral_scale is None else integral_scale
    if joint not in range(6) or not all(
        np.isfinite(v) and 0.2 <= v <= 1
        for v in (kpp_scale, velocity_scale, integral_scale)
    ):
        raise ValueError("Feedback experiment only permits bounded gain reductions")
    j = session.robot["joints"][joint]
    original = {
        **j["gains"],
        **{k: j[k] for k in ("ilim_ma", "velocity_limit_ticks_s")},
        "voltage_limit_mv": j.get("voltage_limit_mv", 0),
    }
    candidate = dict(original)
    candidate["kpp"] *= kpp_scale
    candidate["kpv"] *= velocity_scale
    candidate["kiv"] *= integral_scale
    events = session.directory / f"feedback-J{joint + 1}-restore.json"
    record = {
        "joint": joint,
        "baseline": original,
        "candidate": candidate,
        "restored": False,
        "drive_parameter_readback": False,
    }

    async def push(values):
        await session.stop()
        await session.fresh()
        ack = await session.client.set_pid_gains(j["node_id"], **values)
        if ack != 1:
            raise RuntimeError("Controller did not acknowledge feedback update")
        # Allow all repeated one-way config frames to drain before excitation.
        for _ in range(25):
            await session.fresh()

    atomic_json(events, record)
    try:
        await push(candidate)
        yield candidate
    finally:
        # A cancelled move must finish its stop and restoration before control
        # returns to the caller. A second cancellation cannot orphan cleanup.
        async def restore():
            try:
                await push(original)
                record["restored"] = True
            except BaseException as exc:
                record["restore_error"] = f"{type(exc).__name__}: {exc}"
                raise
            finally:
                atomic_json(events, record)

        await finish_cleanup(restore())


def feedback_metrics(rows, dt, joint):
    """Assess travel, settled hold, and windows covering every transition."""
    moving = np.flatnonzero(
        np.abs(np.asarray([r["qd_commanded"][joint] for r in rows])) > 0.005
    )
    if not len(moving) or (moving[-1] - moving[0]) * dt < 0.5:
        raise ValueError("Feedback trial lacks sustained measured motion")
    edge = round(0.1 / dt)
    travel = rows[max(0, moving[0] - edge) : moving[-1] + edge + 1]
    end = rows[-1].get("sample_time_ns", rows[-1]["elapsed_ns"])
    hold = [
        r
        for r in rows
        if r.get("sample_time_ns", r["elapsed_ns"]) >= end - 1_000_000_000
    ]
    h = motion_metrics(hold, dt)
    h["position_peak_to_peak_deg"] = np.rad2deg(
        np.ptp([r["q"] for r in hold], axis=0)
    ).tolist()
    return {
        "window_peaks": motion_window_peaks(rows, dt),
        "motion": motion_metrics(travel, dt),
        "hold": h,
        "whole_trial": motion_metrics(rows, dt),
    }


def feedback_score(rows, joint):
    # A quiet stationary hold must not conceal vibration during travel or transitions.
    return float(
        np.mean(
            [
                max(r[phase]["oscillation_energy"][joint] for phase in QUALITY_PHASES)
                for r in rows
            ]
        )
    )


def feedback_acceptable(rows, reference, *, joint, policy, noise_energy):
    before = np.max(
        [r[phase]["oscillation_energy"] for r in reference for phase in QUALITY_PHASES],
        axis=0,
    )
    after = np.max(
        [r[phase]["oscillation_energy"] for r in rows for phase in QUALITY_PHASES],
        axis=0,
    )
    before_delay = np.max([r["motion"]["response_delay_s"] for r in reference], axis=0)
    after_delay = np.max([r["motion"]["response_delay_s"] for r in rows], axis=0)
    return bool(
        np.all(after_delay <= np.maximum(0.1, before_delay + 0.04))
        and feedback_score(rows, joint)
        <= 0.7 * max(noise_energy[joint], feedback_score(reference, joint))
        and np.all(after <= np.maximum(noise_energy, before * 1.2))
        and all(
            r["completed"]
            and all(
                motion_acceptance(r[phase], policy)["valid"] for phase in QUALITY_PHASES
            )
            and max(r["hold"]["position_peak_to_peak_deg"]) <= 0.05
            and r["hold"]["velocity_residual_rms"][joint] <= 0.03
            for r in rows
        )
    )


async def feedback(session, *, joint=2, center=None, amplitude_rad=0.18):
    """Compare lower feedback gains during sustained motion and powered holds.

    Reports a candidate only after repeated and separate validation excursions.
    Every experiment restores the original gains. No automatic application.
    """
    if joint not in range(6):
        raise ValueError("Joint must be an index from zero to five")
    if not np.isfinite(amplitude_rad) or not 0.02 <= amplitude_rad <= 0.18:
        raise ValueError("Feedback excursion must be between 0.02 and 0.18 rad")
    center = np.array(session.start if center is None else center, dtype=float)
    session.check_path([session.start, center])
    targets = []
    for offset in [1, -1, 1, -1, 0.75, -0.75, 0.65, -0.65, 0.85]:
        target = center.copy()
        target[joint] += offset * amplitude_rad
        targets.append(target)
    for a in [center, *targets]:
        for b in targets:
            session.check_path([a, b])
    measured = {}
    await session.active_support()

    async def trials(label, indices):
        result = []
        for i in indices:
            # Each comparison uses the same initial pose, including the first
            # held-out trial after the previous block ended at another target.
            initial = center if i == 0 else targets[i - 1]
            await session.position(initial, diagnostic=True)
            current = np.deg2rad((await session.fresh())["angles"])
            t, q = move(
                current, targets[i], np.minimum(session.limits, [0.05, 0.08, 0.3])
            )
            t = np.r_[t, t[-1] + np.arange(0.02, 2.52, 0.02)]
            q = np.vstack([q, np.repeat(targets[i][None, :], 125, axis=0)])
            rejected = None
            try:
                rows, _ = await session.stimulus(
                    "feedback",
                    t,
                    q,
                    settle=False,
                    allow_oscillation=True,
                    joint=joint,
                    candidate=label,
                    repeat=i,
                )
            except TrialRejected as exc:
                # stimulus() has already confirmed Stop; an operating-bound
                # failure is useful tuning evidence. Faults/readiness loss and
                # failures to stop propagate instead of trying another gain.
                rejected = str(exc)
                trial = session.trials[-1]
                dt, rows = capture.read_capture(
                    session.capture_path, trial["capture_start"], trial["capture_end"]
                )
                rows = capture.measurement_rows(
                    capture.active_rows(rows),
                    dt,
                    simulator=session.identity["simulator"],
                )
            phases = feedback_metrics(rows, session.robot["robot"]["tick_dt_s"], joint)
            h = phases["hold"]
            result.append(
                {
                    **phases,
                    "initial_rad": current.tolist(),
                    "target_rad": targets[i].tolist(),
                    "completed": rejected is None,
                    "rejection": rejected,
                }
            )
            measured.setdefault(label, []).append(result[-1])
            atomic_json(session.directory / "feedback-progress.json", measured)
            print(
                f"{label} J{joint + 1}: moving RMS "
                f"{phases['motion']['velocity_residual_rms'][joint]:.4f} rad/s, "
                f"hold excursion "
                f"{h['position_peak_to_peak_deg'][joint]:.4f} deg, "
                f"speed RMS {h['velocity_residual_rms'][joint]:.4f} rad/s",
                flush=True,
            )
        return result

    baseline = await trials("baseline", range(6))

    quantum = np.array(
        [
            2 * np.pi / (2 ** j["encoder_bits"] * j["gear_ratio"])
            for j in session.robot["joints"]
        ]
    )
    # A one-count difference over one native interval produces this speed.
    noise_energy = np.maximum(
        1e-4, (quantum / session.robot["robot"]["tick_dt_s"]) ** 2
    )

    candidate = None
    reference = None
    # Reducing P and I together also removes damping. Try integral reductions
    # with P preserved before testing a lower whole velocity-loop gain.
    search = [(1.0, v) for v in (0.8, 0.6, 0.4)] + [(v, v) for v in (0.8, 0.6, 0.4)]
    for pscale, iscale in (
        search if feedback_score(baseline, joint) > noise_energy[joint] else []
    ):
        label = f"velocity-p-{pscale}-i-{iscale}"
        async with temporary_feedback(
            session, joint, velocity_scale=pscale, integral_scale=iscale
        ) as gains:
            rows = await trials(label, range(6))
        if not feedback_acceptable(
            rows,
            baseline,
            joint=joint,
            policy=session.policy,
            noise_energy=noise_energy,
        ):
            continue
        # Search order is the preference order. Validate before exposing the
        # arm to more gain candidates that cannot improve that preference.
        if reference is None:
            reference = await trials("validation-baseline", range(6, 9))
        async with temporary_feedback(
            session, joint, velocity_scale=pscale, integral_scale=iscale
        ):
            validation = await trials(f"validation-{label}", range(6, 9))
        if feedback_acceptable(
            validation,
            reference,
            joint=joint,
            policy=session.policy,
            noise_energy=noise_energy,
        ):
            candidate = gains
            break
    report = {
        "kind": "feedback",
        "valid": candidate is not None,
        "reasons": []
        if candidate is not None
        else [
            "No repeatable baseline oscillation above encoder noise"
            if feedback_score(baseline, joint) <= noise_energy[joint]
            else "No candidate passed motion, hold, latency, and independent validation checks"
        ],
        "joint": joint,
        "center_rad": center.tolist(),
        "amplitude_rad": amplitude_rad,
        "baseline_fingerprint": session.fingerprint,
        "identity": session.identity,
        "candidate": candidate,
        "encoder_noise_energy_floor": noise_energy.tolist(),
        "measurements": measured,
        "baseline_restored": True,
        "drive_parameter_readback": False,
        "table_vibration_measured": False,
    }
    atomic_json(session.directory / "feedback.json", report)
    if candidate is not None:
        gains = [
            [j["gains"][k] for k in ("kpp", "kpv", "kiv")]
            for j in session.robot["joints"]
        ]
        gains[joint] = [candidate[k] for k in ("kpp", "kpv", "kiv")]
        export_profile(
            session.directory / "feedback-profile",
            session.bundle,
            report,
            feedback_gains=gains,
        )
    return report
