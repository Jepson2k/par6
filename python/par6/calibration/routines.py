"""Repeatable calibration protocols, separate from Commander presentation."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .analysis import fit_gravity
from .gravity_validation import coverage, paired_torque, sweep_evidence
from .preflight import check_stream, stream_scale
from .profiles import atomic_json, export_profile
from .session import TrialRejected
from .trajectory import candidate_values, coupled_move, envelope_move, move, sweeps
from .validation import measured_excitation


def centers(start):
    # Different shoulder/elbow lever arms; the full paths are checked before motion.
    values = []
    for shoulder, elbow, wrist in [
        (0, 0, 0),
        (0.3, 0.3, 0.3),
        (-0.15, 0.2, -0.2),
        (0.2, -0.2, 0.2),
        (-0.1, 0.4, 0.1),
        (0.15, 0.1, -0.1),
    ]:
        q = np.array(start, dtype=float)
        q[1] += shoulder
        q[2] += elbow
        q[4] += wrist
        values.append(q)
    return values


def gravity_centers(start):
    """Excite different gravity lever arms and wrist orientations."""
    result = []
    for offsets in [
        (0, 0, 0, 0, 0),
        (0.8, -0.55, 0.45, 0.4, -0.4),
        (-0.25, 0.65, -0.45, -0.4, 0.4),
        (0.5, 0.5, 0.35, -0.3, -0.3),
        (0.25, -0.45, -0.35, 0.45, 0.3),
        (0.65, 0.15, -0.5, 0.1, 0.5),
    ]:
        q = np.array(start, dtype=float)
        q[1:] += offsets
        result.append(q)
    return result


def verification_centers(start):
    """Wrist orientations distinct from the identification protocol."""
    offsets = [
        (0.15, -0.15, 0.30, 0.12, -0.25),
        (0.35, 0.20, -0.30, -0.12, 0.25),
        (-0.10, 0.35, 0.50, -0.25, 0.40),
        (0.50, -0.30, -0.50, 0.25, -0.40),
        (0.25, 0.10, 0.15, 0.30, 0.50),
        (0.40, 0.30, -0.15, -0.30, -0.50),
    ]
    return [np.asarray(start) + np.r_[0, row] for row in offsets]


def _steady(rows, dt):
    if len(rows) < 30:
        return []
    positions = np.array([r["q"] for r in rows])
    seconds = np.array([r.get("sample_time_ns", r["elapsed_ns"]) for r in rows]) * 1e-9
    width = max(3, int(round(0.12 / dt)))
    # Position differences average over encoder quantization and the drive's
    # reported velocity estimator. Never label a commanded speed as measured.
    velocity = np.gradient(positions, seconds, axis=0)
    smooth = np.stack(
        [
            np.convolve(velocity[:, j], np.ones(width) / width, mode="same")
            for j in range(6)
        ],
        axis=1,
    )
    acceleration = np.gradient(smooth, seconds, axis=0)
    selected = []
    for i, r in enumerate(rows):
        current = np.abs(r["current_ma"])
        limit = np.asarray(r["ilim_ma"])
        if (
            i < width
            or i >= len(rows) - width
            or r["flags"] & 1
            or np.any(current > 0.85 * limit)
        ):
            continue
        if np.max(np.abs(acceleration[i])) > 0.12 or np.max(np.abs(smooth[i])) > 0.15:
            continue
        if np.any(np.abs(smooth[i]) >= 0.015):
            selected.append(
                {
                    "tick": r["tick"],
                    "q": r["q"],
                    "qd": smooth[i].tolist(),
                    "tau": r["tau"],
                }
            )
    return selected


async def check_motion(session, *, joints=(1, 2), amplitude=0.04):
    """Short recorded bidirectional check before a full identification run."""
    results = []
    plans = list(
        sweeps(
            centers(session.start)[:3],
            np.minimum(session.limits, [0.07, 0.12, 0.5]),
            amplitude=amplitude,
        )
    )
    plans = [p for p in plans if p["group"] == 0 and p["joint"] in joints]
    if not plans:
        raise ValueError("No joints selected")
    for p in plans:
        session.check_path([session.start, p["start"]])
        session.check_path(p["positions"])
    await session.active_support()
    for p in plans:
        print(
            f"Recorded check J{p['joint'] + 1} direction {p['direction']}", flush=True
        )
        await session.position(p["start"])
        _, metrics = await session.stimulus(
            "check",
            p["times"],
            p["positions"],
            joint=p["joint"],
            direction=p["direction"],
        )
        results.append(metrics)
    await session.position(session.start)
    report = {
        "kind": "motion-check",
        "gravity_comp": False,
        "tested_mode": "STREAM",
        "valid": all(passes(r) for r in results),
        "identity": session.identity,
        "measurements": results,
    }
    atomic_json(session.directory / "motion-check.json", report)
    return report


async def gravity(session, *, prior=None):
    """Collect independent trajectory groups and fit only observable corrections."""
    limits = np.minimum(session.limits, [0.07, 0.12, 0.5])
    train, validation = [], []
    group_offset = 0
    if prior is not None:
        prior = Path(prior)
        identity = json.loads((prior / "identity.json").read_text())
        if json.loads((prior / "acceptance.json").read_text()) != session.policy:
            raise ValueError("Prior gravity data used different acceptance checks")
        if identity.get("reference") != session.identity["reference"]:
            raise ValueError(
                "Prior gravity data predates a restart or reference change"
            )
        if identity["config_fingerprint"] != session.fingerprint:
            raise ValueError("Prior gravity data belongs to a different configuration")
        fields = ("node", "hw_ver", "sw_ver", "serial")

        def devices(value):
            return [
                tuple(d[k] for k in fields) for d in value["drives"] if d["present"]
            ]

        if identity["simulator"] != session.identity["simulator"] or devices(
            identity
        ) != devices(session.identity):
            raise ValueError(
                "Prior gravity data belongs to a different runtime or drive identity"
            )
        data = json.loads((prior / "gravity-samples.json").read_text())
        train, validation = data["train"], data["validation"]
        group_offset = 1 + max((r["group"] for r in train + validation), default=-1)
        atomic_json(
            session.directory / "prior.json",
            {"directory": str(prior.resolve()), "identity": identity},
        )
    poses = gravity_centers(session.start)
    # Recollection retains each physical center's original split. Renumbering
    # a subset would put a formerly trained center into held-out validation.
    if prior is not None:
        protocol = json.loads((prior / "gravity-protocol.json").read_text())
        if protocol.get("protocol_version") != 3:
            raise ValueError("Prior gravity data used an older collection protocol")
        if not np.allclose(protocol["start_rad"], session.start, atol=0.001, rtol=0):
            raise ValueError("Prior gravity collection used a different reference pose")
    plans, _, excluded = _gravity_plans(session, poses, limits)
    planned = {}
    for name, held_out in (("train", False), ("validation", True)):
        part = [p for p in plans if p["held_out"] == held_out]
        planned[name] = _fitting_coverage(
            session,
            [
                {"q": q.tolist(), "joint": p["joint"], "group": p["group"]}
                for p in part
                for q in p["positions"][::5]
            ],
        )
    for plan in plans:
        plan["group"] += group_offset
    atomic_json(
        session.directory / "gravity-protocol.json",
        {
            "protocol_version": 3,
            "start_rad": session.start.tolist(),
            "centers_rad": [p.tolist() for p in poses],
            "amplitude_rad": 0.18,
            "group_offset": group_offset,
            "retained_train_samples": len(train),
            "retained_validation_samples": len(validation),
            "selected_groups": sorted({p["group"] for p in plans}),
            "excluded_poses": excluded,
            "planned_coverage": planned,
        },
    )
    if not all(c["valid"] for c in planned.values()):
        raise ValueError(
            "Collision-free gravity groups do not cover independent fitting and validation"
        )
    await session.active_support()
    for i, p in enumerate(plans):
        print(
            f"Gravity sweep {i + 1}/{len(plans)} J{p['joint'] + 1} direction {p['direction']}",
            flush=True,
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
        samples = [
            r
            for r in _steady(rows, session.robot["robot"]["tick_dt_s"])
            if p["direction"] * r["qd"][p["joint"]] >= 0.015
        ]
        (validation if p["held_out"] else train).extend(
            {
                **r,
                "group": p["group"],
                "center_group": p["group"] - group_offset,
                "joint": p["joint"],
                "direction": p["direction"],
            }
            for r in samples
        )
        # Durable progress is usable even if a later trajectory is refused.
        atomic_json(
            session.directory / "gravity-samples.json",
            {"train": train, "validation": validation},
        )
    measured = {
        name: _fitting_coverage(session, rows)
        for name, rows in (("train", train), ("validation", validation))
    }
    requested = {(p["group"], p["joint"], p["direction"]) for p in plans}
    evidence = sweep_evidence(train + validation, requested)
    reasons = []
    if not all(c["valid"] for c in measured.values()):
        reasons.append(
            "Measured fitting or validation gravity coverage is insufficient"
        )
    reasons.extend(evidence["reasons"])
    fit = None
    if not reasons:
        fit = fit_gravity(
            train,
            validation,
            session.model,
            baseline_correction=session.robot.get("gravity_correction"),
            baseline_scale=session.robot.get("gravity_scale"),
            max_validation_rms_nm=session.policy["gravity_residual_nm"],
        )
    report = {
        **(fit.to_dict() if fit is not None else {"valid": False, "reasons": reasons}),
        "baseline_fingerprint": session.fingerprint,
        "identity": session.identity,
        "kind": "gravity",
        "support_mode": "active_feedback",
        "applied_validation_complete": False,
        "status": "candidate" if fit is not None and fit.valid else "rejected",
        "planned_coverage": planned,
        "measured_coverage": measured,
        "sweep_evidence": evidence,
        "excluded_poses": excluded,
        "friction_assumption": "odd Coulomb and viscous friction during steady motion",
        "limitation": "Load-dependent directional friction can be indistinguishable from gravity error",
    }
    atomic_json(session.directory / "gravity-fit.json", report)
    await session.position(session.start)
    if fit is not None and fit.valid:
        path = export_profile(
            session.directory / "gravity-profile",
            session.bundle,
            report,
            gravity=fit.correction,
        )
        print(
            f"Fitted gravity candidate: {path}. Requires activation and verify-gravity.",
            flush=True,
        )
    else:
        print(f"Gravity profile rejected: {report['reasons']}", flush=True)
    return report


def passes(metrics):
    check = metrics.get("acceptance", {})
    return check.get("valid") is True and check.get("oscillation_checked") is True


def _fitting_coverage(session, samples):
    # Repeated observations at one center cannot replace independent poses.
    if len({r.get("center_group", r["group"]) for r in samples}) < 2:
        return {"valid": False, "reason": "Fewer than two independent pose groups"}
    return coverage(
        session.model,
        [r["q"] for r in samples],
        [r["joint"] for r in samples],
        session.window,
    )


def _gravity_plans(session, poses, limits):
    """Retain complete reachable groups without changing the held-out split."""
    plans = [p for p in sweeps(poses, limits, amplitude=0.18) if p["joint"] != 0]
    previous = session.start
    selected, selected_poses, excluded = [], [], []
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
        selected_poses.append(pose)
        previous = trial_previous
    return selected, selected_poses, excluded


async def verify_gravity(session):
    """Measure loaded-model error with feedback active and no parameter refit."""
    plans, poses, excluded = _gravity_plans(
        session,
        verification_centers(session.start),
        np.minimum(session.limits, [0.05, 0.08, 0.3]),
    )
    planned = (
        coverage(
            session.model,
            [p for p in poses for _ in range(1, 6)],
            list(range(1, 6)) * len(poses),
            session.window,
        )
        if len(poses) >= 3
        else {
            "valid": False,
            "reason": "Fewer than three complete collision-free pose groups",
        }
    )
    samples = []
    report = {
        "kind": "gravity-torque-verification",
        "valid": False,
        "complete": False,
        "identity": session.identity,
        "acceptance": session.policy,
        "planned_coverage": planned,
        "excluded_poses": excluded,
        "reasons": [],
        "support_mode": "active_feedback",
        "applied_validation_complete": False,
        "scope": "tested poses, configured gripper, empty hand",
        "centers_rad": [p.tolist() for p in poses],
        "excursion_rad": 0.18,
        "unmeasured_joint_combinations_validated": False,
        "validation_target": "paired measured torque under an odd-friction assumption",
    }
    try:
        if not planned["valid"]:
            report["reasons"].append(
                "Verification poses do not span the observable gravity model"
            )
            return report
        await session.active_support()
        for i, p in enumerate(plans):
            print(
                f"Gravity verification {i + 1}/{len(plans)} "
                f"J{p['joint'] + 1} direction {p['direction']}",
                flush=True,
            )
            await session.position(p["start"])
            rows, _ = await session.stimulus(
                "gravity-verification",
                p["times"],
                p["positions"],
                group=p["group"],
                joint=p["joint"],
                direction=p["direction"],
            )
            samples.extend(
                {
                    **r,
                    "group": p["group"],
                    "joint": p["joint"],
                    "direction": p["direction"],
                }
                for r in _steady(rows, session.robot["robot"]["tick_dt_s"])
            )
            atomic_json(
                session.directory / "gravity-verification-samples.json",
                {"samples": samples},
            )
        torque = paired_torque(
            samples,
            session.model,
            correction=session.robot.get("gravity_correction"),
            scale=session.robot.get("gravity_scale"),
            limit_nm=session.policy["gravity_residual_nm"],
        )
        observed = {(r["group"], r["joint"]) for r in samples}
        required = {(p["group"], p["joint"]) for p in plans}
        if observed != required:
            report["reasons"].append(
                "Missing moving evidence for a requested joint or pose"
            )
        report["torque"] = torque
        report["reasons"].extend(torque["reasons"])
        if torque["pairs"]:
            measured = coverage(
                session.model,
                [p["q"] for p in torque["pairs"]],
                [p["joint"] for p in torque["pairs"]],
                session.window,
            )
            report["measured_coverage"] = measured
            if not measured["valid"]:
                report["reasons"].append(
                    "Measured moving evidence lacks gravity model coverage"
                )
        else:
            report["reasons"].append("No usable torque pairs")
        await session.position(session.start)
        report["complete"] = True
        report["valid"] = not report["reasons"]
        report["status"] = "torque-consistent" if report["valid"] else "rejected"
        return report
    except BaseException as exc:
        report["reasons"].append(f"{type(exc).__name__}: {exc}")
        raise
    finally:
        atomic_json(session.directory / "gravity-verification.json", report)


def _assess_smoothness(reports, baseline, candidate):
    """Require matched evidence and independent improvement at held-out poses."""
    baseline, candidate = (
        np.asarray(value, dtype=float) for value in (baseline, candidate)
    )
    if any(
        value.shape != (6, 3) or not np.isfinite(value).all() or np.any(value <= 0)
        for value in (baseline, candidate)
    ):
        raise ValueError("Smoothness requires finite positive six-joint limits")
    report = {
        "kind": "smoothness",
        "protocol_version": 3,
        "valid": False,
        "complete": False,
        "reasons": [],
        "measurements": reports,
        "baseline_limits": baseline.tolist(),
        "limits": candidate.tolist(),
        "table_vibration_measured": False,
        "tested_mode": "STREAM",
        "scope": "tested single-axis excursions and joint encoder vibration",
        "applied_validation_complete": False,
    }
    reasons = report["reasons"]
    if np.any(candidate > baseline * (1 + 1e-12)):
        reasons.append("Candidate increases a comparison limit")
    if np.allclose(candidate, baseline, rtol=1e-12, atol=0):
        reasons.append("Baseline and candidate settings are identical")
    required = {
        (g, j, r, d)
        for g in range(3)
        for j in range(6)
        for r in range(3)
        for d in (-1, 1)
    }
    for label in ("baseline", "candidate"):
        readings = reports.get(label, [])
        keys = [
            tuple(row.get(k) for k in ("group", "joint", "repeat", "direction"))
            for row in readings
        ]
        if (
            len(keys) != len(required)
            or set(keys) != required
            or any(
                row.get("held_out") is not (row.get("group") == 2) for row in readings
            )
        ):
            reasons.append(f"{label}: incomplete or mismatched pose evidence")
            return report
        for field in ("oscillation_energy", "response_delay_s"):
            values = np.asarray([row[field] for row in readings], dtype=float)
            if (
                values.shape != (len(required), 6)
                or not np.isfinite(values).all()
                or np.any(values < 0)
            ):
                raise ValueError(f"Invalid smoothness {field} measurements")
    report["complete"] = True
    if not all(
        row.get("acceptance", {}).get("valid") is True for row in reports["baseline"]
    ):
        reasons.append("Baseline tracking or current check failed")
    if not all(passes(row) for row in reports["candidate"]):
        reasons.append("Candidate motion acceptance failed")

    def compare(predicate):
        selected = {
            label: [r for r in reports[label] if predicate(r)]
            for label in ("baseline", "candidate")
        }
        energy = [
            np.mean([r["oscillation_energy"] for r in selected[label]], axis=0)
            for label in ("baseline", "candidate")
        ]
        delay = [
            np.max([r["response_delay_s"] for r in selected[label]], axis=0)
            for label in ("baseline", "candidate")
        ]
        return (
            {
                "joint_oscillation_before": energy[0].tolist(),
                "joint_oscillation_after": energy[1].tolist(),
                "response_delay_before_s": delay[0].tolist(),
                "response_delay_after_s": delay[1].tolist(),
            },
            energy,
            delay,
        )

    summary, _, _ = compare(lambda _: True)
    report.update(summary)
    report["pose_comparisons"] = []
    noise = 1e-5
    for group in range(3):
        summary, (before, after), (before_delay, after_delay) = compare(
            lambda row: row["group"] == group
        )
        report["pose_comparisons"].append({"group": group, **summary})
        for joint in np.flatnonzero(after > np.maximum(noise, before * 1.1)):
            reasons.append(f"Pose group {group}: J{joint + 1} oscillation increased")
        if np.any(after_delay > np.maximum(0.1, before_delay + 0.04)):
            reasons.append(
                f"Pose group {group}: candidate adds excessive response delay"
            )
    report["excursion_comparisons"] = []
    for group in range(3):
        for requested_joint in range(6):
            for direction in (-1, 1):
                summary, (before, after), (before_delay, after_delay) = compare(
                    lambda row: (
                        row["group"] == group
                        and row["joint"] == requested_joint
                        and row["direction"] == direction
                    )
                )
                report["excursion_comparisons"].append(
                    {
                        "group": group,
                        "joint": requested_joint,
                        "direction": direction,
                        **summary,
                    }
                )
                label = (
                    f"Pose group {group}, J{requested_joint + 1}, direction {direction}"
                )
                for joint in np.flatnonzero(after > np.maximum(noise, before * 1.1)):
                    reasons.append(f"{label}: J{joint + 1} oscillation increased")
                if np.any(after_delay > np.maximum(0.1, before_delay + 0.04)):
                    reasons.append(f"{label}: candidate adds excessive response delay")
    for label, held_out in (("training", False), ("held_out", True)):
        summary, (before, after), _ = compare(lambda row: row["held_out"] == held_out)
        measurable = bool(np.sum(before) > noise)
        report[label] = {**summary, "improvement_above_noise_evaluable": measurable}
        if measurable:
            if np.sum(after) > 0.7 * np.sum(before):
                reasons.append(f"{label}: less than 30% oscillation improvement")
        elif not held_out:
            reasons.append(
                "Training baseline is below the noise floor; improvement is unproven"
            )
        elif np.sum(after) > noise:
            reasons.append("Held-out candidate exceeds a quiet baseline's noise floor")
    report["valid"] = not reasons
    report["status"] = "improved" if report["valid"] else "rejected"
    return report


def _smoothness_settings(robot, exec_limits):
    """Choose distinct caps realizable by the runtime's global stream fractions."""
    loaded = np.array(
        [
            [
                joint["limits"].get("stream", {}).get(key, joint["limits"][key])
                for key in ("velocity_rad_s", "acceleration_rad_s2", "jerk_rad_s3")
            ]
            for joint in robot["joints"]
        ]
    )
    requested = np.minimum(loaded, exec_limits)
    requested[:, 0] = np.minimum(requested[:, 0], 0.3)
    fractions = stream_scale(robot, requested)
    comparison = loaded * [fractions["speed"], fractions["accel"], fractions["accel"]]
    candidate = comparison * [1.0, 0.5, 0.5]
    # A velocity-limited quintic can ignore both changed derivative bounds.
    # Evaluate move()'s duration bounds without allocating a very long path.
    duration = []
    for limits in (comparison, candidate):
        values = np.maximum.reduce(
            [
                1.875 * 0.08 / limits[:, 0],
                np.sqrt((10 / np.sqrt(3)) * 0.08 / limits[:, 1]),
                np.cbrt(60 * 0.08 / limits[:, 2]),
            ]
        )
        duration.append(np.ceil(values / 0.02) * 0.02)
    if not np.isfinite(duration).all() or np.any(duration[1] < 1.05 * duration[0]):
        raise ValueError(
            "Smoothness probe cannot excite the proposed acceleration/jerk change"
        )
    return comparison, candidate


async def smoothness(session):
    """Compare distinct STREAM settings; table motion and JOG remain unmeasured."""
    reports = {"baseline": [], "candidate": []}
    report = {
        "kind": "smoothness",
        "protocol_version": 3,
        "valid": False,
        "complete": False,
        "status": "incomplete",
        "reasons": [],
        "measurements": reports,
        "gravity_comp": False,
        "identity": session.identity,
        "acceptance": session.policy,
        "baseline_fingerprint": session.fingerprint,
        "table_vibration_measured": False,
        "tested_mode": "STREAM",
        "applied_validation_complete": False,
    }
    try:
        comparison, candidate = _smoothness_settings(session.robot, session.limits)
        settings = [("baseline", comparison), ("candidate", candidate)]
        scales = {
            label: stream_scale(session.robot, limits) for label, limits in settings
        }
        await session.active_support()
        for group, center in enumerate(centers(session.start)[:3]):
            for joint in range(6):
                for repeat in range(3):
                    for label, limits in settings:
                        await session.position(center)
                        for direction in (1, -1):
                            print(
                                f"Smoothness {label}: pose {group + 1}, J{joint + 1}, "
                                f"repeat {repeat + 1}, direction {direction}",
                                flush=True,
                            )
                            end = center.copy()
                            end[joint] += direction * 0.08
                            times, positions = move(center, end, limits)
                            metadata = {
                                "group": group,
                                "joint": joint,
                                "repeat": repeat,
                                "direction": direction,
                                "held_out": group == 2,
                            }
                            _, metrics = await session.stimulus(
                                label,
                                times,
                                positions,
                                command_limits=limits,
                                **scales[label],
                                **metadata,
                                allow_oscillation=label == "baseline",
                            )
                            reports[label].append({**metrics, **metadata})
                            await session.position(center)
        report = {
            **report,
            **_assess_smoothness(reports, comparison, candidate),
        }
        await session.position(session.start)
        if report["valid"]:
            export_profile(
                session.directory / "smooth-profile",
                session.bundle,
                report,
                stream_limits=candidate.tolist(),
            )
        return report
    except BaseException as exc:
        report["valid"] = report["complete"] = False
        report["status"] = "failed"
        report["reasons"].append(f"{type(exc).__name__}: {exc}")
        raise
    finally:
        atomic_json(session.directory / "smoothness.json", report)


async def motion_envelope(session, *, max_trials=180, resume=None):
    """Measured operating envelope with 10% steps and durable continuation.

    A geometric or runtime bound censors the search. Only fully repeated,
    measured candidates contribute to the exported 80% operating envelope.
    """
    report = {
        "kind": "motion-envelope",
        "gravity_comp": False,
        "valid": False,
        "complete": False,
        "status": "running",
        "reasons": [],
        "identity": session.identity,
        "baseline_fingerprint": session.fingerprint,
        "tested_mode": "STREAM",
        "applied_validation_complete": False,
    }
    path = session.directory / "motion-envelope.json"
    try:
        # Same-session continuation may reuse this path after an older result.
        atomic_json(path, report)
        if (session.directory / "motion-profile").exists():
            raise RuntimeError(
                "Output already contains a motion profile; use a new session directory"
            )
        return await _search_motion_envelope(
            session, report, max_trials=max_trials, resume=resume
        )
    except BaseException as exc:
        error = f"{type(exc).__name__}: {exc}"
        report.update(valid=False, complete=False, status="failed", error=error)
        if error not in report["reasons"]:
            report["reasons"].append(error)
        raise
    finally:
        atomic_json(path, report)


async def _search_motion_envelope(session, report, *, max_trials, resume):
    if not isinstance(max_trials, int) or max_trials < 1:
        raise ValueError("max_trials must be a positive integer")
    ceiling = np.array(
        [
            [
                j["limits"].get("stream", {}).get(k, j["limits"][k])
                for k in ["velocity_rad_s", "acceleration_rad_s2", "jerk_rad_s3"]
            ]
            for j in session.robot["joints"]
        ]
    )
    initial = np.minimum(np.minimum(session.limits, [0.1, 0.3, 1.0]), ceiling)
    state = {
        "protocol_version": 5,
        "gravity_comp": False,
        "baseline_fingerprint": session.fingerprint,
        "poses": [c.tolist() for c in centers(session.start)[:3]],
        "identity": session.identity,
        "measurements": {},
        "acceptance": session.policy,
    }
    if resume is not None:
        state = json.loads(Path(resume).read_text())
        if state.get("protocol_version") != 5 or state.get("gravity_comp") is not False:
            raise ValueError("Resume data lacks the required gravity-disabled protocol")
        if state["baseline_fingerprint"] != session.fingerprint:
            raise ValueError(
                "Resume data belongs to a different controller configuration"
            )
        if state.get("acceptance") != session.policy:
            raise ValueError("Resume data used different acceptance checks")
        previous = state.get("identity")

        def devices(identity):
            return [
                (d["node"], d["hw_ver"], d["sw_ver"], d["serial"])
                for d in identity["drives"]
                if d["present"]
            ]

        if (
            previous is None
            or previous.get("reference") != session.identity["reference"]
            or previous["simulator"] != session.identity["simulator"]
            or devices(previous) != devices(session.identity)
        ):
            raise ValueError(
                "Resume data belongs to a different runtime or drive identity"
            )
    poses = [np.asarray(q) for q in state["poses"]]
    for q in poses:
        session.check_path([q, q])
    await session.active_support()
    checkpoint = session.directory / "envelope-progress.json"
    best = initial.copy()
    measured = np.zeros((6, 3), dtype=bool)
    boundaries = []
    count = 0
    complete = True
    for j in range(6):
        for axis in range(3):
            for value in candidate_values(initial[j, axis], ceiling[j, axis]):
                # This is a stream experiment. A separate EXEC ceiling may be
                # much faster than the actual limiter and cannot size its pulse.
                candidate = ceiling.copy()
                candidate[j, axis] = value
                scale = stream_scale(session.robot, candidate)
                effective = ceiling * [scale["speed"], scale["accel"], scale["accel"]]
                key = f"{j}:{axis}:{value:.12g}"
                readings = state["measurements"].setdefault(key, [])
                accepted = True
                for group, center in enumerate(poses[:2]):
                    try:
                        t, positions = envelope_move(
                            center, j, axis, value, effective, session.window
                        )
                        _, peaks = check_stream(
                            session.preview,
                            t,
                            positions,
                            candidate,
                            **scale,
                            check_path=session.check_path,
                        )
                        if peaks[j][axis] < 0.9 * value:
                            raise ValueError(
                                "Native command cannot excite this derivative"
                            )
                    except ValueError as exc:
                        boundaries.append(
                            {
                                "joint": j,
                                "dimension": axis,
                                "candidate": value,
                                "reason": str(exc),
                                "measured": False,
                            }
                        )
                        accepted = False
                        break
                    for direction in [-1, 1]:
                        path = positions if direction == 1 else positions[::-1]
                        for repeat in range(3):
                            index = group * 6 + (direction == 1) * 3 + repeat
                            if index < len(readings):
                                reading = readings[index]
                            else:
                                if count >= max_trials:
                                    complete = False
                                    break
                                try:
                                    await session.position(path[0])
                                    rows, metrics = await session.stimulus(
                                        "envelope",
                                        t,
                                        path,
                                        joint=j,
                                        dimension=axis,
                                        candidate=float(value),
                                        command_limits=candidate,
                                        repeat=repeat,
                                        group=group,
                                        direction=direction,
                                        **scale,
                                    )
                                    achieved = measured_excitation(
                                        metrics, rows, session.policy
                                    )[j, axis]
                                    accepted = (
                                        passes(metrics) and achieved >= 0.9 * value
                                    )
                                    reading = {
                                        "accepted": bool(accepted),
                                        "achieved": float(achieved),
                                        "metrics": metrics,
                                    }
                                except TrialRejected as exc:
                                    # stimulus() has already confirmed the stop.
                                    # A failed current/settling bound is durable:
                                    # continuation must not retry it indefinitely.
                                    await session.fresh()
                                    reading = {"accepted": False, "reason": str(exc)}
                                readings.append(reading)
                                count += 1
                                atomic_json(checkpoint, state)
                            if not reading["accepted"]:
                                accepted = False
                                boundaries.append(
                                    {
                                        "joint": j,
                                        "dimension": axis,
                                        "candidate": value,
                                        "reason": "tracking, current, or actual excitation failed",
                                        "measured": True,
                                    }
                                )
                                break
                        if not accepted or not complete:
                            break
                    if not accepted or not complete:
                        break
                if not accepted or not complete:
                    break
                best[j, axis] = value
                measured[j, axis] = True
            if not complete:
                break
        if not complete:
            break
    operating = np.minimum(ceiling, best * 0.8)
    valid = complete and bool(measured.all())
    combined = None
    if valid:
        combined = await _combined_envelope(
            session, poses[2], operating, state, checkpoint, max_trials - count
        )
        valid = combined["valid"]
        complete = complete and combined["complete"]
    report.update(
        {
            "kind": "motion-envelope",
            "valid": valid,
            "complete": complete,
            "status": "candidate"
            if valid
            else "incomplete"
            if not complete
            else "rejected",
            "identity": session.identity,
            "baseline_fingerprint": session.fingerprint,
            "limits": operating.tolist(),
            "measured": measured.tolist(),
            "boundaries": boundaries,
            "combined": combined,
            "checkpoint": str(checkpoint),
            "payload_scope": "attached gripper, no held object",
            "scope": "tested poses and excursions; lower bound on achievable maximum",
            "tested_mode": "STREAM",
            "applied_validation_complete": False,
        }
    )
    atomic_json(checkpoint, state)
    return await _finish_motion_envelope(session, report, operating)


async def _finish_motion_envelope(session, report, operating):
    """Return to the starting pose before staging EXEC limits from STREAM evidence."""
    report.update(tested_mode="STREAM", applied_validation_complete=False)
    try:
        await session.position(session.start)
        if report["valid"]:
            export_profile(
                session.directory / "motion-profile",
                session.bundle,
                report,
                exec_limits=operating.tolist(),
            )
        return report
    except BaseException as exc:
        error = f"{type(exc).__name__}: {exc}"
        report.update(valid=False, complete=False, status="failed", error=error)
        reasons = report.setdefault("reasons", [])
        if error not in reasons:
            reasons.append(error)
        raise
    finally:
        atomic_json(session.directory / "motion-envelope.json", report)


async def _combined_envelope(session, center, limits, state, checkpoint, budget):
    """Require recorded all-axis excitation at the held-out pose before export."""
    scale = stream_scale(session.robot, limits)
    saved = state.setdefault(
        "combined_validation",
        {"limits": limits.tolist(), "stream_scale": scale, "measurements": {}},
    )
    if saved.get("stream_scale") != scale:
        raise ValueError("Combined evidence used different native stream fractions")
    if saved["limits"] != limits.tolist():
        raise ValueError("Combined evidence belongs to different operating limits")
    plans = []
    # Each out-and-back excites every derivative; both initial directions
    # and opposing joint signs test coupling without duplicate axis sweeps.
    for pattern, signs in enumerate(([1] * 6, [1, -1, 1, -1, 1, -1])):
        for direction in [-1, 1]:
            try:
                times, path = coupled_move(
                    center,
                    limits,
                    session.window,
                    signs=np.asarray(signs) * direction,
                )
                _, peaks = check_stream(
                    session.preview,
                    times,
                    path,
                    limits,
                    **scale,
                    check_path=session.check_path,
                )
                if np.any(np.asarray(peaks) < 0.9 * limits):
                    raise ValueError(
                        "Native command does not reach the operating derivatives"
                    )
                session.check_path([session.start, path[0]])
                session.check_path([path[-1], session.start])
            except ValueError as exc:
                return {
                    "valid": False,
                    "complete": True,
                    "reason": f"Coupled operating limits cannot be exercised: {exc}",
                    "measurements": saved["measurements"],
                }
            for repeat in range(3):
                plans.append((f"{pattern}:{direction}:{repeat}", times, path))
    for key, times, positions in plans:
        if key not in saved["measurements"]:
            if budget <= 0:
                return {
                    "valid": False,
                    "complete": False,
                    "measurements": saved["measurements"],
                }
            try:
                await session.position(positions[0])
                rows, metrics = await session.stimulus(
                    "combined-validation",
                    times,
                    positions,
                    command_limits=limits,
                    trial=key,
                    **scale,
                )
                achieved = measured_excitation(metrics, rows, session.policy)
                reading = {
                    "accepted": passes(metrics)
                    and bool(np.all(achieved >= 0.9 * limits)),
                    "achieved": achieved.tolist(),
                    "metrics": metrics,
                }
            except TrialRejected as exc:
                await session.fresh()
                reading = {"accepted": False, "reason": str(exc)}
            saved["measurements"][key] = reading
            budget -= 1
            atomic_json(checkpoint, state)
        if not saved["measurements"][key]["accepted"]:
            return {
                "valid": False,
                "complete": True,
                "measurements": saved["measurements"],
            }
    return {"valid": True, "complete": True, "measurements": saved["measurements"]}
