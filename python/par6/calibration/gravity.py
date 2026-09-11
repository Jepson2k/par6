"""Gravity identification and its independent checks.

Friction is projected out of moving samples before fitting; stationary
friction is never gravity evidence. Every held-out joint must meet an
absolute torque residual ceiling — relative improvement alone accepts
nothing.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Protocol

import numpy as np


class Model(Protocol):
    def gravity(self, q: list[float]) -> list[float]: ...
    def regressor(self, q: list[float]) -> list[list[float]]: ...


@dataclass
class GravityFit:
    correction: list[float]
    coulomb_nm: list[float]
    viscous_nm_s: list[float]
    rank: int
    train_rms_nm: list[float]
    validation_before_nm: list[float]
    validation_after_nm: list[float]
    validation_limit_nm: list[float]
    valid: bool
    reasons: list[str]

    def to_dict(self) -> dict:
        return asdict(self)


def _data(rows: list[dict], model: Model):
    if not rows:
        raise ValueError("No usable moving samples")
    q = np.asarray([r["q"] for r in rows], dtype=float)
    v = np.asarray([r["qd"] for r in rows], dtype=float)
    tau = np.asarray([r["tau"] for r in rows], dtype=float)
    if q.shape != v.shape or q.shape != tau.shape or q.ndim != 2 or q.shape[1] != 6:
        raise ValueError("Samples need six positions, speeds and torques")
    if not all(np.isfinite(x).all() for x in (q, v, tau)):
        raise ValueError("Non-finite calibration measurement")
    y = np.asarray([model.regressor(list(p)) for p in q])
    g = np.asarray([model.gravity(list(p)) for p in q])
    # Gravity cannot observe a stationary joint through unknown static friction.
    mask = np.abs(v) >= 0.015
    if any(np.count_nonzero(mask[:, j]) < 20 for j in range(1, 6)):
        raise ValueError("Insufficient moving observations for gravity-loaded joints")
    if any(
        np.count_nonzero(v[:, j] >= 0.015) < 10
        or np.count_nonzero(v[:, j] <= -0.015) < 10
        for j in range(1, 6)
    ):
        raise ValueError("Need both motion directions on every gravity-loaded joint")
    friction = np.zeros((len(rows), 6, 12))
    for j in range(6):
        friction[:, j, j] = np.sign(v[:, j])
        friction[:, j, j + 6] = v[:, j]
    return y[mask], friction[mask], (tau - g)[mask], mask, g, tau


def fit_gravity(
    train: list[dict],
    validation: list[dict],
    model: Model,
    *,
    baseline_correction=None,
    baseline_scale=None,
    max_validation_rms_nm=0.05,
) -> GravityFit:
    """Fit observable gravity directions after projecting out moving friction.

    Complete trajectories must be split before calling this function. Validation
    is never reused to estimate parameters. No self-locking law is assumed.
    """
    try:
        absolute_limit = np.broadcast_to(
            np.asarray(max_validation_rms_nm, dtype=float), (6,)
        )
    except ValueError as exc:
        raise ValueError(
            "Gravity residual limit requires a scalar or six values"
        ) from exc
    if not np.isfinite(absolute_limit).all() or np.any(absolute_limit <= 0):
        raise ValueError("Gravity residual limits must be finite and positive")
    y, f, residual, mask, _, _ = _data(train, model)
    # Fixed tool inertials are folded into the sixth arm body by Pinocchio.
    # Fit additive link corrections without rewriting that tool declaration;
    # provenance binds the resulting profile to this arm/gripper assembly.
    cols = y.shape[1]
    arm_cols = cols
    a = y[:, :arm_cols]
    scale = np.linalg.norm(a, axis=0)
    # Floating-point zero columns grow with sample count. Normalizing them
    # promotes numerical noise into a parameter with enormous coefficients.
    observable = scale > max(1e-12, float(scale.max()) * 1e-10)
    scale[~observable] = 1.0
    a = a / scale
    a[:, ~observable] = 0.0
    projected_a = a - f @ np.linalg.lstsq(f, a, rcond=1e-5)[0]
    projected_b = residual - f @ np.linalg.lstsq(f, residual, rcond=1e-5)[0]
    u, singular, vt = np.linalg.svd(projected_a, full_matrices=False)
    keep = singular > max(1e-5, singular[0] * 0.01)
    rank = int(keep.sum())
    if rank == 0:
        raise ValueError("Pose set cannot distinguish gravity from friction")
    # Truncated SVD leaves unobservable directions at the nominal model.
    z = vt[keep].T @ ((u[:, keep].T @ projected_b) / singular[keep])
    delta = np.zeros(cols)
    delta[:arm_cols] = z / scale
    friction = np.linalg.lstsq(f, residual - y @ delta, rcond=1e-5)[0]
    pred = y @ delta + f @ friction
    vy, vf, vb, vmask, vg, vtau = _data(validation, model)
    baseline = (
        np.zeros(cols)
        if baseline_correction is None or len(baseline_correction) == 0
        else np.asarray(baseline_correction)
    )
    if baseline.shape != (cols,) or not np.isfinite(baseline).all():
        raise ValueError("Invalid baseline gravity correction")
    scale = np.ones(6) if baseline_scale is None else np.asarray(baseline_scale)
    if scale.shape != (6,) or not np.isfinite(scale).all() or np.any(scale <= 0):
        raise ValueError("Invalid baseline gravity scale")
    previous_delta = vy @ baseline
    trim = (vg[vmask] + previous_delta) * (np.broadcast_to(scale, vg.shape)[vmask] - 1)
    before = vb - vf @ friction - previous_delta - trim
    after = vb - vf @ friction - vy @ delta

    def rms_by_joint(values, selection):
        dense = np.full(selection.shape, np.nan)
        dense[selection] = values
        return [
            float(np.sqrt(np.mean(dense[selection[:, j], j] ** 2)))
            if selection[:, j].any()
            else 0.0
            for j in range(6)
        ]

    br = rms_by_joint(before, vmask)
    ar = rms_by_joint(after, vmask)
    reasons = []
    for j in range(6):
        if ar[j] > absolute_limit[j]:
            reasons.append(
                f"J{j + 1}: held-out torque error exceeds {absolute_limit[j]:g} Nm limit"
            )
        if ar[j] > max(0.05, br[j] * 1.1):
            reasons.append(f"J{j + 1}: held-out residual increased")
    if np.sqrt(np.mean(after**2)) > 0.7 * max(0.05, np.sqrt(np.mean(before**2))):
        reasons.append("Less than 30% held-out improvement above noise floor")
    if np.any(friction[:6] < -0.05) or np.any(friction[6:] < -0.05):
        reasons.append("Negative dissipative friction; model or data inconsistent")
    if np.any(np.abs(delta) > 10) or not np.isfinite(delta).all():
        reasons.append("Implausible correction coefficients")
    # A measured correction must not dominate the nominal gravity model.
    all_y = np.asarray([model.regressor(r["q"]) for r in validation])
    correction = all_y @ delta
    allowed = np.maximum(0.2, 0.5 * np.max(np.abs(vg), axis=0))
    if np.any(np.max(np.abs(correction), axis=0) > allowed):
        reasons.append("Correction exceeds 50% of nominal gravity (0.2 Nm floor)")
    return GravityFit(
        delta.tolist(),
        friction[:6].tolist(),
        friction[6:].tolist(),
        rank,
        rms_by_joint(residual - pred, mask),
        br,
        ar,
        absolute_limit.tolist(),
        not reasons,
        reasons,
    )


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
