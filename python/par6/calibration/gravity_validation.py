"""Independent torque checks; stationary friction is never gravity evidence."""

from __future__ import annotations

import numpy as np


def coverage(model, poses, joints, window):
    """Compare measured gravity directions with the configured joint workspace.

    Whitening by a deterministic reference set makes the weakest direction
    meaningful despite mixed mass/first-moment units and redundant parameters.
    Reference poses are algebraic probes only; they are never motion targets.
    """
    q = np.asarray(poses, dtype=float)
    joints = np.asarray(joints, dtype=int)
    window = np.asarray(window, dtype=float)
    if (
        q.ndim != 2
        or len(q) == 0
        or q.shape[1] != 6
        or joints.shape != (len(q),)
        or window.shape != (6, 2)
        or not np.isfinite(q).all()
        or not np.isfinite(window).all()
        or np.any(window[:, 1] <= window[:, 0])
        or np.any((joints < 1) | (joints > 5))
    ):
        raise ValueError("Invalid gravity coverage observations or joint window")
    rng = np.random.default_rng(6071)
    reference_q = rng.uniform(window[:, 0], window[:, 1], (256, 6))
    reference = np.concatenate([model.regressor(p.tolist())[1:] for p in reference_q])
    _, singular, vt = np.linalg.svd(reference, full_matrices=False)
    keep = singular > singular[0] * 1e-7
    whitening = vt[keep].T * (np.sqrt(len(reference)) / singular[keep])
    measured = np.array([model.regressor(p.tolist())[j] for p, j in zip(q, joints)])
    values = np.linalg.svd(measured @ whitening / np.sqrt(len(q)), compute_uv=False)
    rank = int(np.count_nonzero(values > 0.05))
    required = int(keep.sum())
    return {
        "valid": rank == required,
        "rank": rank,
        "required_rank": required,
        "normalized_singular_values": values.tolist(),
        "minimum_singular_value": 0.05,
        "scope": "observed poses; rank does not validate extrapolated accuracy",
    }


def paired_torque(samples, model, *, correction=None, scale=None, limit_nm=0.05):
    """Cancel odd moving friction without fitting the model being verified.

    Pair narrow position bins from opposite sweeps of the SAME joint and
    configuration. Match measured speeds and stationary coordinates. Unknown
    asymmetric friction remains a limitation, recorded explicitly in the result.
    Every requested group/joint must supply enough pairs; missing or noisy data
    produces an inconclusive rejection rather than a passing model.
    """
    if not samples or not np.isfinite(limit_nm) or limit_nm <= 0:
        raise ValueError(
            "Moving gravity verification needs observations and a positive limit"
        )
    q = np.asarray([r["q"] for r in samples], dtype=float)
    v = np.asarray([r["qd"] for r in samples], dtype=float)
    tau = np.asarray([r["tau"] for r in samples], dtype=float)
    if q.shape != (len(samples), 6) or q.shape != v.shape or q.shape != tau.shape:
        raise ValueError("Invalid six-joint moving torque observations")
    if not all(np.isfinite(a).all() for a in (q, v, tau)):
        raise ValueError("Non-finite moving torque observations")
    y = np.asarray([model.regressor(p.tolist()) for p in q])
    delta = (
        np.zeros(y.shape[2])
        if correction is None or len(correction) == 0
        else np.asarray(correction)
    )
    trim = np.ones(6) if scale is None else np.asarray(scale)
    if (
        delta.shape != (y.shape[2],)
        or trim.shape != (6,)
        or not np.isfinite(delta).all()
        or not np.isfinite(trim).all()
        or np.any(trim <= 0)
    ):
        raise ValueError("Invalid loaded gravity model")
    predicted = (np.asarray([model.gravity(p.tolist()) for p in q]) + y @ delta) * trim
    residual = tau - predicted
    labels = [(r["group"], r["joint"]) for r in samples]
    if any(not 1 <= j <= 5 for _, j in labels):
        raise ValueError("Moving gravity observations must identify a loaded joint")
    pairs, reasons = [], []
    if {j for _, j in labels} != set(range(1, 6)):
        reasons.append("Missing moving evidence for a gravity-loaded joint")
    for group, j in sorted(set(labels)):
        member = np.array([label == (group, j) for label in labels])
        moving = member & (np.abs(v[:, j]) >= 0.015) & (np.abs(v[:, j]) <= 0.15)
        others = [k for k in range(6) if k != j]
        moving &= np.max(np.abs(v[:, others]), axis=1) <= 0.005
        pos = np.flatnonzero(moving & (v[:, j] > 0))
        neg = np.flatnonzero(moving & (v[:, j] < 0))
        accepted = []
        if len(pos) >= 30 and len(neg) >= 30:
            low = max(q[pos, j].min(), q[neg, j].min())
            high = min(q[pos, j].max(), q[neg, j].max())
            # Full overlap must cover appreciable travel, not a few encoder bins.
            if high - low >= 0.12:
                edges = np.linspace(low, high, 13)
                for a, b in zip(edges[:-1], edges[1:]):
                    p = pos[(q[pos, j] >= a) & (q[pos, j] < b)]
                    n = neg[(q[neg, j] >= a) & (q[neg, j] < b)]
                    if min(len(p), len(n)) < 3:
                        continue
                    qp, qn = q[p].mean(axis=0), q[n].mean(axis=0)
                    vp, vn = v[p, j].mean(), -v[n, j].mean()
                    if (
                        np.max(np.abs(qp[others] - qn[others])) > 0.003
                        or abs(qp[j] - qn[j]) > 0.01
                        or abs(vp - vn) > max(0.001, 0.1 * min(vp, vn))
                    ):
                        continue
                    error = float((residual[p, j].mean() + residual[n, j].mean()) / 2)
                    # Native samples are correlated; do not claim sqrt(N)
                    # precision from polling a motor hundreds of times a second.
                    spread = float(
                        np.hypot(residual[p, j].std(), residual[n, j].std()) / 2
                    )
                    accepted.append(
                        {
                            "group": group,
                            "joint": j,
                            "q": ((qp + qn) / 2).tolist(),
                            "speed_rad_s": [float(vp), float(vn)],
                            "error_nm": error,
                            "spread_nm": spread,
                            "samples": [len(p), len(n)],
                        }
                    )
        if len(accepted) < 8:
            reasons.append(
                f"Group {group} J{j + 1}: insufficient matched bidirectional travel"
            )
        pairs.extend(accepted)
    summaries = []
    for group, j in sorted(set(labels)):
        selected = [p for p in pairs if p["group"] == group and p["joint"] == j]
        if not selected:
            continue
        error = np.asarray([p["error_nm"] for p in selected])
        spread = np.asarray([p["spread_nm"] for p in selected])
        rms = float(np.sqrt(np.mean(error**2)))
        peak = float(np.max(np.abs(error)))
        uncertain = float(np.sqrt(np.mean(spread**2)))
        if rms + uncertain > limit_nm or peak > 2 * limit_nm:
            reasons.append(
                f"Group {group} J{j + 1}: paired torque error or uncertainty exceeds acceptance limit"
            )
        summaries.append(
            {
                "group": group,
                "joint": j,
                "rms_nm": rms,
                "peak_nm": peak,
                "spread_nm": uncertain,
            }
        )
    return {
        "valid": not reasons,
        "reasons": reasons,
        "pairs": pairs,
        "measurements": summaries,
        "limit_nm": limit_nm,
        "friction_assumption": "odd friction at matched positions and opposite measured velocities",
        "gravity_parameters_identified": False,
        "limitation": "Load-dependent directional friction can be indistinguishable from gravity error",
        "table_vibration_measured": False,
    }


def sweep_evidence(samples, requested):
    """Require measured travel of each requested joint in its declared direction."""
    readings, reasons = [], []
    for group, joint, direction in sorted(requested):
        rows = [
            r
            for r in samples
            if (r["group"], r["joint"], r["direction"]) == (group, joint, direction)
            and direction * r["qd"][joint] >= 0.015
        ]
        span = float(np.ptp([r["q"][joint] for r in rows])) if rows else 0.0
        accepted = len(rows) >= 30 and np.isfinite(span) and span >= 0.12
        readings.append(
            {
                "group": group,
                "joint": joint,
                "direction": direction,
                "signed_samples": len(rows),
                "measured_span_rad": span,
                "valid": bool(accepted),
            }
        )
        if not accepted:
            reasons.append(
                f"Group {group} J{joint + 1} direction {direction}: "
                "insufficient signed moving travel"
            )
    if not readings:
        reasons.append("No gravity sweeps requested")
    return {"valid": not reasons, "reasons": reasons, "measurements": readings}
