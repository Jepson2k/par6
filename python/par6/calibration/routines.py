"""The four calibration routines. Each is plain sequential code over a
Session: plan, preflight, stimulate, judge the recording, report a Patch."""

from __future__ import annotations

import numpy as np

from .gravity import (
    centers,
    coverage,
    fit_gravity,
    gravity_centers,
    paired_torque,
    sweep_evidence,
)
from .metrics import (
    motion_acceptance,
    motion_metrics,
    motion_window_peaks,
    steady_samples,
)
from .preflight import check_stream
from .report import Patch, atomic_json, write_profile
from .session import Session, TrialRejected
from .trajectory import move, sweeps

#: Slow limits for identification sweeps [rad/s, rad/s², rad/s³].
SLOW = np.array([0.07, 0.12, 0.5])
#: Speed at which a moving sample counts as moving (encoder noise floor).
MOVING_RAD_S = 0.015


def _report(
    session: Session, kind: str, valid: bool, reasons: list[str], **fields
) -> dict:
    report = {
        "kind": kind,
        "valid": bool(valid),
        "reasons": reasons,
        "baseline_fingerprint": session.fingerprint,
        "identity": session.identity,
        "acceptance": session.policy,
        "table_vibration_measured": False,
        **fields,
    }
    atomic_json(session.directory / f"{kind}.json", report)
    return report


def _stage(session: Session, report: dict, patch: Patch):
    if report["valid"] and not patch.is_empty():
        path = write_profile(
            session.directory / "profile", session.bundle, session.robot, report, patch
        )
        report["candidate_config"] = str(path)
        report["patch"] = patch.to_dict()
        atomic_json(session.directory / f"{report['kind']}.json", report)
        session.log(f"Candidate config staged: {path}")
        session.log(f"Patch:\n{patch.toml()}")
    return report


# ------------------------------------------------------------------ check


async def check(session: Session, *, joints=(1, 2), amplitude=0.04) -> dict:
    """Short bidirectional sweeps on `joints`; a before/after baseline of
    tracking, velocity residual, windowed vibration and current saturation."""
    plans = [
        p
        for p in sweeps(
            centers(session.start)[:3],
            np.minimum(session.limits, SLOW),
            amplitude=amplitude,
        )
        if p["group"] == 0 and p["joint"] in joints
    ]
    if not plans:
        raise ValueError("No joints selected")
    for p in plans:
        session.check_path([session.start, p["start"]])
        session.check_path(p["positions"])
    await session.set_gravity(False)
    results = []
    for p in plans:
        session.log(f"check: J{p['joint'] + 1} direction {p['direction']:+d}")
        await session.position(p["start"])
        try:
            _, metrics = await session.stimulus(
                "check",
                p["times"],
                p["positions"],
                joint=p["joint"],
                direction=p["direction"],
            )
            metrics["valid"] = True
        except TrialRejected as exc:
            metrics = {
                **session.trials[-1].get("metrics", {}),
                "valid": False,
                "rejection": str(exc),
            }
        results.append({"joint": p["joint"], "direction": p["direction"], **metrics})
    await session.position(session.start)
    reasons = [
        f"J{r['joint'] + 1} direction {r['direction']:+d}: {r.get('rejection', 'rejected')}"
        for r in results
        if not r["valid"]
    ]
    summary = {
        f"J{r['joint'] + 1}{'+' if r['direction'] > 0 else '-'}": {
            k: r[k][r["joint"]]
            for k in (
                "tracking_peak_deg",
                "velocity_residual_rms",
                "measured_vibration_rms",
                "command_ripple_rms",
                "saturation_s",
            )
            if k in r
        }
        for r in results
    }
    for name, values in summary.items():
        session.log(
            f"check {name}: " + ", ".join(f"{k}={v:.4g}" for k, v in values.items())
        )
    return _report(
        session, "check", not reasons, reasons, summary=summary, measurements=results
    )


# ------------------------------------------------------------ tune-feedback

QUALITY_PHASES = ("motion", "hold", "window_peaks")
#: Preference order: lower the integral first (keeps damping), then both.
CANDIDATES = ((1.0, 0.8), (1.0, 0.6), (0.8, 0.8), (0.6, 0.6))


def trial_quality(rows, dt, joint) -> dict:
    """Travel, the last second of hold, and overlapping windows over the whole
    trial, so neither a quiet hold nor a quiet travel can hide the other."""
    moving = np.flatnonzero(
        np.abs(np.asarray([r["qd_commanded"][joint] for r in rows])) > 0.005
    )
    if not len(moving) or (moving[-1] - moving[0]) * dt < 0.5:
        raise ValueError("Feedback trial lacks sustained measured motion")
    edge = round(0.1 / dt)
    travel = rows[max(0, moving[0] - edge) : moving[-1] + edge + 1]
    end = rows[-1]["sample_time_ns"]
    hold = [r for r in rows if r["sample_time_ns"] >= end - 1_000_000_000]
    h = motion_metrics(hold, dt)
    h["position_peak_to_peak_deg"] = np.rad2deg(
        np.ptp([r["q"] for r in hold], axis=0)
    ).tolist()
    return {
        "window_peaks": motion_window_peaks(rows, dt),
        "motion": motion_metrics(travel, dt),
        "hold": h,
    }


def oscillation(trials, joint) -> float:
    return float(
        np.mean(
            [
                max(t[phase]["oscillation_energy"][joint] for phase in QUALITY_PHASES)
                for t in trials
            ]
        )
    )


def better(candidate, baseline, *, joint, policy, noise) -> bool:
    """The candidate trials all pass ordinary acceptance, hold still, respond
    no slower, and carry at most 70 % of the baseline's oscillation energy on
    the tuned joint without raising any other joint's."""
    before = np.max(
        [t[p]["oscillation_energy"] for t in baseline for p in QUALITY_PHASES], axis=0
    )
    after = np.max(
        [t[p]["oscillation_energy"] for t in candidate for p in QUALITY_PHASES], axis=0
    )
    before_delay = np.max([t["motion"]["response_delay_s"] for t in baseline], axis=0)
    after_delay = np.max([t["motion"]["response_delay_s"] for t in candidate], axis=0)
    return bool(
        np.all(after_delay <= np.maximum(0.1, before_delay + 0.04))
        and oscillation(candidate, joint)
        <= 0.7 * max(noise[joint], oscillation(baseline, joint))
        and np.all(after <= np.maximum(noise, before * 1.2))
        and all(
            t["completed"]
            and all(motion_acceptance(t[p], policy)["valid"] for p in QUALITY_PHASES)
            and max(t["hold"]["position_peak_to_peak_deg"]) <= 0.05
            and t["hold"]["velocity_residual_rms"][joint]
            <= policy["velocity_residual_rad_s"]
            for t in candidate
        )
    )


async def tune_feedback(
    session: Session, *, joint=2, center=None, amplitude=0.18
) -> dict:
    """Lower one joint's velocity-loop gains until sustained moves and holds
    stop oscillating; the winner must also pass three independent trials."""
    if joint not in range(6):
        raise ValueError("Joint must be an index from zero to five")
    if not np.isfinite(amplitude) or not 0.02 <= amplitude <= 0.18:
        raise ValueError("Feedback excursion must be between 0.02 and 0.18 rad")
    center = np.array(session.start if center is None else center, dtype=float)
    session.check_path([session.start, center])
    targets = []
    for offset in (1, -1, 1, -1, 0.75, -0.75, 0.65, -0.65, 0.85):
        t = center.copy()
        t[joint] += offset * amplitude
        targets.append(t)
    for a in [center, *targets]:
        for b in targets:
            session.check_path([a, b])
    quantum = np.array(
        [
            2 * np.pi / (2 ** j["encoder_bits"] * j["gear_ratio"])
            for j in session.robot["joints"]
        ]
    )
    noise = np.maximum(1e-4, (quantum / session.robot["robot"]["tick_dt_s"]) ** 2)
    dt = session.robot["robot"]["tick_dt_s"]
    measured: dict[str, list] = {}
    await session.set_gravity(False)

    async def trials(label, indices):
        result = []
        for i in indices:
            initial = center if i == 0 else targets[i - 1]
            await session.position(initial, diagnostic=True)
            here = np.deg2rad((await session.fresh())["angles"])
            t, q = move(here, targets[i], np.minimum(session.limits, [0.05, 0.08, 0.3]))
            t = np.r_[t, t[-1] + np.arange(0.02, 2.52, 0.02)]
            q = np.vstack([q, np.repeat(targets[i][None, :], 125, axis=0)])
            rejection = None
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
                rejection = str(exc)
                from . import capture

                trial = session.trials[-1]
                _, rows = capture.read_capture(
                    session.capture_path, trial["capture_start"], trial["capture_end"]
                )
                rows = capture.measurement_rows(
                    capture.active_rows(rows, (capture.MODE_STREAM,)),
                    dt,
                    simulator=session.identity["simulator"],
                )
            quality = trial_quality(rows, dt, joint)
            entry = {**quality, "completed": rejection is None, "rejection": rejection}
            result.append(entry)
            measured.setdefault(label, []).append(entry)
            atomic_json(session.directory / "feedback-progress.json", measured)
            session.log(
                f"{label} J{joint + 1}: moving RMS "
                f"{quality['motion']['velocity_residual_rms'][joint]:.4f} rad/s, hold excursion "
                f"{quality['hold']['position_peak_to_peak_deg'][joint]:.4f} deg"
            )
        return result

    baseline = await trials("baseline", range(6))
    candidate = None
    reference = None
    if oscillation(baseline, joint) > noise[joint]:
        for kpv_scale, kiv_scale in CANDIDATES:
            label = f"kpv-{kpv_scale}-kiv-{kiv_scale}"
            async with session.gains(
                joint, kpv_scale=kpv_scale, kiv_scale=kiv_scale
            ) as gains:
                rows = await trials(label, range(6))
            if not better(
                rows, baseline, joint=joint, policy=session.policy, noise=noise
            ):
                continue
            if reference is None:
                reference = await trials("validation-baseline", range(6, 9))
            async with session.gains(joint, kpv_scale=kpv_scale, kiv_scale=kiv_scale):
                validation = await trials(f"validation-{label}", range(6, 9))
            if better(
                validation, reference, joint=joint, policy=session.policy, noise=noise
            ):
                candidate = gains
                break
    await session.position(session.start)
    reasons = (
        []
        if candidate is not None
        else [
            "No repeatable baseline oscillation above encoder noise"
            if oscillation(baseline, joint) <= noise[joint]
            else "No candidate passed motion, hold, latency and independent validation"
        ]
    )
    patch = Patch()
    if candidate is not None:
        patch.feedback_gains[joint] = [
            candidate["kpp"],
            candidate["kpv"],
            candidate["kiv"],
        ]
    report = _report(
        session,
        "tune-feedback",
        candidate is not None,
        reasons,
        joint=joint,
        center_rad=center.tolist(),
        amplitude_rad=amplitude,
        candidate=candidate,
        encoder_noise_energy_floor=noise.tolist(),
        measurements=measured,
        baseline_restored=True,
        drive_parameter_readback=False,
    )
    return _stage(session, report, patch)


# ---------------------------------------------------------------- gravity


def _plans(session: Session, poses, limits, amplitude=0.18):
    """Whole reachable groups (approach, every sweep, return), in order."""
    plans = [p for p in sweeps(poses, limits, amplitude=amplitude, joints=range(1, 6))]
    previous = session.start
    selected, excluded = [], []
    for group, pose in enumerate(poses):
        group_plans = [p for p in plans if p["group"] == group]
        trial_previous = previous
        try:
            for p in group_plans:
                session.plan_position(trial_previous, p["start"])
                check_stream(
                    session.preview,
                    p["times"],
                    p["positions"],
                    session.limits,
                    check_path=session.check_path,
                )
                trial_previous = p["end"]
            session.plan_position(trial_previous, session.start)
        except ValueError as exc:
            excluded.append(
                {"group": group, "pose_rad": pose.tolist(), "reason": str(exc)}
            )
            continue
        selected.extend(group_plans)
        previous = trial_previous
    return selected, excluded


def _coverage(session: Session, samples):
    if len({r["group"] for r in samples}) < 2:
        return {"valid": False, "reason": "Fewer than two independent pose groups"}
    return coverage(
        session.model,
        [r["q"] for r in samples],
        [r["joint"] for r in samples],
        session.window,
    )


async def _collect(session: Session, plans):
    """Run every planned sweep; steady moving samples labelled by group/joint/direction."""
    samples = []
    dt = session.robot["robot"]["tick_dt_s"]
    for i, p in enumerate(plans):
        session.log(
            f"gravity sweep {i + 1}/{len(plans)}: J{p['joint'] + 1} direction {p['direction']:+d}"
        )
        await session.position(p["start"])
        rows, _ = await session.stimulus(
            "gravity",
            p["times"],
            p["positions"],
            group=p["group"],
            held_out=p["held_out"],
            joint=p["joint"],
            direction=p["direction"],
        )
        for r in steady_samples(rows, dt):
            if p["direction"] * r["qd"][p["joint"]] >= MOVING_RAD_S:
                samples.append(
                    {
                        **r,
                        "group": p["group"],
                        "joint": p["joint"],
                        "direction": p["direction"],
                        "held_out": p["held_out"],
                    }
                )
        atomic_json(session.directory / "gravity-samples.json", {"samples": samples})
    return samples


async def gravity(session: Session, *, verify_only=False) -> dict:
    """Identify the arm's gravity model from slow bidirectional sweeps at six
    shoulder/elbow/wrist centres (feedforward off, position feedback on),
    fit with friction projected out, and demand every held-out joint stay
    under the residual ceiling. `verify_only` skips the fit and instead
    checks the currently loaded model against the held-out groups by paired
    opposite-direction torque — the acceptance run after a candidate is
    applied."""
    limits = np.minimum(session.limits, SLOW)
    poses = gravity_centers(session.start)
    plans, excluded = _plans(session, poses, limits)
    if verify_only:
        plans = [p for p in plans if p["held_out"]]
    groups = sorted({p["group"] for p in plans})
    if len(groups) < (2 if verify_only else 3):
        raise ValueError(
            f"Too few collision-free pose groups ({groups}); excluded: {excluded}"
        )
    planned = {
        name: _coverage(
            session,
            [
                {"q": q.tolist(), "joint": p["joint"], "group": p["group"]}
                for p in plans
                if p["held_out"] == held
                for q in p["positions"][::5]
            ],
        )
        for name, held in (("train", False), ("validation", True))
        if not (verify_only and name == "train")
    }
    if not all(c["valid"] for c in planned.values()):
        raise ValueError(
            f"Planned sweeps do not cover the observable gravity directions: {planned}"
        )
    await session.set_gravity(False)
    samples = await _collect(session, plans)
    await session.position(session.start)
    requested = {(p["group"], p["joint"], p["direction"]) for p in plans}
    evidence = sweep_evidence(samples, requested)
    reasons = list(evidence["reasons"])
    loaded_correction = session.robot.get("gravity_correction") or None
    loaded_scale = session.robot.get("gravity_scale") or None
    patch = Patch()
    fields = {
        "excluded_poses": excluded,
        "sweep_evidence": evidence,
        "planned_coverage": planned,
    }
    if verify_only:
        check = paired_torque(
            samples,
            session.model,
            correction=loaded_correction,
            scale=loaded_scale,
            limit_nm=session.policy["gravity_residual_nm"],
        )
        reasons += check["reasons"]
        fields["paired_torque"] = check
        for m in check["measurements"]:
            session.log(
                f"verify group {m['group']} J{m['joint'] + 1}: rms {m['rms_nm']:.4f} Nm, peak {m['peak_nm']:.4f} Nm"
            )
        return _report(session, "verify-gravity", not reasons, reasons, **fields)
    train = [r for r in samples if not r["held_out"]]
    validation = [r for r in samples if r["held_out"]]
    measured = {
        "train": _coverage(session, train),
        "validation": _coverage(session, validation),
    }
    fields["measured_coverage"] = measured
    if not all(c["valid"] for c in measured.values()):
        reasons.append(
            "Measured fitting or validation gravity coverage is insufficient"
        )
    fit = None
    if not reasons:
        fit = fit_gravity(
            train,
            validation,
            session.model,
            baseline_correction=loaded_correction,
            baseline_scale=loaded_scale,
            max_validation_rms_nm=session.policy["gravity_residual_nm"],
        )
        reasons += fit.reasons
        fields["fit"] = fit.to_dict()
        session.log(
            "gravity held-out residual before "
            + ", ".join(f"{v:.3f}" for v in fit.validation_before_nm)
            + " Nm; after "
            + ", ".join(f"{v:.3f}" for v in fit.validation_after_nm)
            + " Nm"
        )
        if fit.valid:
            patch.gravity = fit.correction
    report = _report(
        session, "gravity", fit is not None and fit.valid, reasons, **fields
    )
    return _stage(session, report, patch)


# ----------------------------------------------------------------- limits

FRACTIONS = (0.25, 0.5, 0.75, 1.0)
#: Room each side of the centre a joint needs for its limit moves [rad].
MIN_TRAVEL_RAD = 0.35
MAX_TRAVEL_RAD = 1.0
#: A probe only exercises a limit when the planner actually commanded at
#: least this share of the fraction it was asked for; a short travel that
#: never gets up to speed proves nothing about the speed.
EXERCISED = 0.9


#: Following error while moving is a lag, not a fault; what a limit must
#: keep bounded is the arrival: the error over the last 0.3 s of the move.
SETTLE_WINDOW_S = 0.3
MOVING_TRACKING_DEG = 3.0
#: Velocity residual allowed as a share of the commanded peak speed.
RESIDUAL_SHARE = 0.04


def _settle_error_deg(rows, dt):
    n = max(2, round(SETTLE_WINDOW_S / dt))
    tail = rows[-n:]
    q = np.asarray([r["q"] for r in tail])
    qc = np.asarray([r["q_commanded"] for r in tail])
    return np.rad2deg(np.max(np.abs(q - qc), axis=0))


def _limits_pass(rows, metrics, joint, policy) -> list[str]:
    """What a limit must keep bounded: the arrival, the current, and any
    residual or vibration beyond the encoder's noise floor and a share of the
    speed itself (a velocity estimate at 1 rad/s is noisier than at rest)."""
    reasons = []
    dt = policy["tick_dt_s"]
    if metrics["tracking_peak_deg"][joint] > MOVING_TRACKING_DEG:
        reasons.append("tracking error")
    if _settle_error_deg(rows, dt)[joint] > policy["tracking_deg"]:
        reasons.append("arrival error")
    if metrics["saturation_s"][joint] > 0.0:
        reasons.append("current saturation")
    noise = policy["velocity_noise_floor_rad_s"][joint]
    speed = metrics["commanded_peaks"][joint][0]
    windows = metrics["window_peaks"]
    residual_limit = max(
        noise, policy["velocity_residual_rad_s"], RESIDUAL_SHARE * speed
    )
    if windows["velocity_residual_rms"][joint] > residual_limit:
        reasons.append("velocity residual")
    if windows["measured_vibration_rms"][joint] > max(noise, policy["vibration_rad_s"]):
        reasons.append("vibration")
    return reasons


async def limits(session: Session, *, joints=range(6), margin=0.8) -> dict:
    """Per joint, rising speed fractions then rising acceleration fractions
    of the configured exec limits through the real EXEC planner, both
    directions. A dimension is certified only where the planner actually
    commanded the fraction asked for; the largest clean certified peak times
    `margin` becomes the proposed exec (and stream) limit, a dimension the
    travel could not exercise keeps its configured value. Two all-joint
    moves at the proposal must pass before anything is staged."""
    keys = ("velocity_rad_s", "acceleration_rad_s2", "jerk_rad_s3")
    hard = np.array([[j["limits"][k] for k in keys] for j in session.robot["joints"]])
    exec_limits = session.limits.copy()
    centre = np.clip(
        session.start,
        session.window[:, 0] + MIN_TRAVEL_RAD,
        session.window[:, 1] - MIN_TRAVEL_RAD,
    )
    travel = np.minimum(
        MAX_TRAVEL_RAD,
        np.minimum(centre - session.window[:, 0], session.window[:, 1] - centre) - 0.05,
    )
    session.check_path([session.start, centre])
    results = {}
    proposed = exec_limits.copy()

    # Position feedback with no gravity feedforward, like every other routine:
    # a probe judges the drives' tracking of the planner, and toggling the
    # support mode between approach and probe moved the wrist before the
    # move even started.
    await session.set_gravity(False)
    await session.position(centre)
    # The session judges a probe on faults, saturation and arrival; following
    # error while moving and speed-scaled residual are this routine's call.
    probe_policy = {**session.policy, "tracking_deg": MOVING_TRACKING_DEG}

    async def probe(joint, speed, accel):
        """Out, back, and home again at the fractions; the worst measured and
        commanded peaks over the three moves, or the first rejection."""
        a, b = centre.copy(), centre.copy()
        a[joint] -= travel[joint]
        b[joint] += travel[joint]
        measured, commanded = [], []
        for target in (a, b, centre):
            try:
                rows, metrics = await session.queued_move(
                    target,
                    speed=speed,
                    accel=accel,
                    allow_oscillation=True,
                    policy=probe_policy,
                    joint=joint,
                    probe=(speed, accel),
                )
            except TrialRejected as exc:
                return None, None, str(exc)
            bad = _limits_pass(rows, metrics, joint, session.policy)
            if bad:
                return None, None, ", ".join(bad)
            measured.append(
                (
                    metrics["peak_velocity_rad_s"][joint],
                    metrics["estimated_peak_acceleration_rad_s2"][joint],
                )
            )
            commanded.append(metrics["commanded_peaks"][joint][:2])
        return np.max(measured, axis=0), np.max(commanded, axis=0), None

    def certify(joint, dimension, probes):
        """The largest passing probe whose command reached its fraction."""
        exercised = [
            p
            for p in probes
            if p["peak"] is not None
            and p["commanded"][dimension]
            >= EXERCISED * p["fraction"] * exec_limits[joint, dimension]
        ]
        if not exercised:
            return None
        best = max(exercised, key=lambda p: p["fraction"])
        return best["peak"][dimension]

    for joint in joints:
        speed_probes, accel_probes = [], []
        for f in FRACTIONS:
            peak, commanded, why = await probe(joint, f, 0.5)
            speed_probes.append(
                {
                    "fraction": f,
                    "peak": None if peak is None else peak.tolist(),
                    "commanded": None if commanded is None else commanded.tolist(),
                    "rejected": why,
                }
            )
            session.log(
                f"limits J{joint + 1} speed {f:.2f}: "
                + (
                    f"ok measured {np.round(peak, 3)} commanded {np.round(commanded, 3)}"
                    if peak is not None
                    else why
                )
            )
            if peak is None:
                break
        passing = [p for p in speed_probes if p["peak"] is not None]
        if not passing:
            results[joint] = {
                "speed_probes": speed_probes,
                "reason": "no speed fraction passed",
            }
            continue
        best_speed = passing[-1]["fraction"]
        for f in FRACTIONS:
            peak, commanded, why = await probe(joint, best_speed, f)
            accel_probes.append(
                {
                    "fraction": f,
                    "peak": None if peak is None else peak.tolist(),
                    "commanded": None if commanded is None else commanded.tolist(),
                    "rejected": why,
                }
            )
            session.log(
                f"limits J{joint + 1} accel {f:.2f}: "
                + (
                    f"ok measured {np.round(peak, 3)} commanded {np.round(commanded, 3)}"
                    if peak is not None
                    else why
                )
            )
            if peak is None:
                break
        best_accel = next(
            (p["fraction"] for p in reversed(accel_probes) if p["peak"] is not None),
            None,
        )
        v = certify(joint, 0, speed_probes)
        a = certify(joint, 1, accel_probes)
        entry = {
            "speed_probes": speed_probes,
            "accel_probes": accel_probes,
            "best_speed_fraction": best_speed,
            "best_accel_fraction": best_accel,
            "velocity_exercised": v is not None,
            "acceleration_exercised": a is not None,
        }
        if v is not None:
            proposed[joint, 0] = margin * min(v, hard[joint, 0])
        if a is not None:
            proposed[joint, 1] = margin * min(a, hard[joint, 1])
            proposed[joint, 2] = 3 * proposed[joint, 1]
        if v is None and a is None:
            entry["note"] = (
                f"travel of {travel[joint]:.2f} rad never reached the configured "
                "limits; they are kept as configured"
            )
        entry["proposed"] = proposed[joint].tolist()
        results[joint] = entry
    reasons = [f"J{j + 1}: {r['reason']}" for j, r in results.items() if "reason" in r]
    coupled = []
    if not reasons:
        # The proposal is only as good as an all-joint move at it: the client
        # fractions are global, so use the weakest joint's fraction.
        speed = min(r["best_speed_fraction"] for r in results.values())
        accel = min(r["best_accel_fraction"] or 0.25 for r in results.values())
        # Only the probed joints move: a joint this run did not certify has
        # no business travelling at the weakest certified fraction.
        offset = np.zeros(6)
        for j in results:
            offset[j] = min(travel[j], MIN_TRAVEL_RAD) / 2
        for target in (centre - offset, centre + offset, centre):
            try:
                rows, metrics = await session.queued_move(
                    target,
                    speed=speed,
                    accel=accel,
                    allow_oscillation=True,
                    policy=probe_policy,
                    coupled=True,
                )
                verdicts = [
                    _limits_pass(rows, metrics, j, session.policy) for j in range(6)
                ]
                bad = [f"J{j + 1} " + ", ".join(v) for j, v in enumerate(verdicts) if v]
                coupled.append({"target": target.tolist(), "rejected": bad or None})
                session.log(
                    "limits coupled move: " + ("ok" if not bad else "; ".join(bad))
                )
                if bad:
                    reasons.append("coupled move: " + "; ".join(bad))
                    break
            except TrialRejected as exc:
                coupled.append({"target": target.tolist(), "rejected": str(exc)})
                session.log(f"limits coupled move: {exc}")
                reasons.append(f"coupled move: {exc}")
                break
    await session.position(session.start)
    changed = not reasons and not np.allclose(proposed, exec_limits)
    patch = (
        Patch(exec_limits=proposed.tolist(), stream_limits=proposed.tolist())
        if changed
        else Patch()
    )
    report = _report(
        session,
        "limits",
        not reasons,
        reasons,
        per_joint={str(j): r for j, r in results.items()},
        coupled=coupled,
        margin=margin,
        travel_rad=travel.tolist(),
        configured_limits=hard.tolist(),
        configured_exec_limits=exec_limits.tolist(),
        proposed_exec_limits=proposed.tolist(),
        scope="RUCKIG EXEC path with the empty configured gripper; JOG is not measured",
    )
    return _stage(session, report, patch)
