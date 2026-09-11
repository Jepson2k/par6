"""Identification and comparison of measured calibration trajectories (SI units)."""

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


def motion_metrics(rows: list[dict], dt: float) -> dict:
    """Compare actual drive targets with encoders; never against client intentions."""
    if len(rows) < 10 or not 0 < dt < 1:
        raise ValueError("Insufficient capture or invalid period")
    ticks = np.asarray([r["tick"] for r in rows])
    if np.any(np.diff(ticks) != 1):
        raise ValueError("Capture dropped native ticks")
    host_seconds = (
        np.asarray([r.get("elapsed_ns", r["tick"] * dt * 1e9) for r in rows]) * 1e-9
    )
    periods = np.diff(host_seconds)
    if np.any(periods <= 0) or np.max(periods) > 0.1:
        raise ValueError("Native capture timing is invalid or missed a 100 ms deadline")
    seconds = (
        np.asarray(
            [
                r.get("sample_time_ns", r.get("elapsed_ns", r["tick"] * dt * 1e9))
                for r in rows
            ]
        )
        * 1e-9
    )
    if not np.isfinite(seconds).all() or np.any(np.diff(seconds) <= 0):
        raise ValueError("Invalid measurement clock")
    q = np.asarray([r["q"] for r in rows])
    qc = np.asarray([r["q_commanded"] for r in rows])
    qd = np.asarray([r["qd"] for r in rows])
    qdc = np.asarray([r["qd_commanded"] for r in rows])
    current = np.asarray([r["current_ma"] for r in rows])
    limits = np.asarray([r["ilim_ma"] for r in rows])
    if not all(
        np.isfinite(a).all() for a in [q, qc, qd, qdc, current, limits]
    ) or np.any(limits <= 0):
        raise ValueError("Invalid measured state or current limits")
    if any(r["flags"] & 8 for r in rows):
        raise ValueError("Stale drive feedback in native recording")
    error = q - qc
    residual = qd - qdc
    # Normalize the windowed periodogram so trial length cannot buy improvement.
    # This measures joint velocity residual, not table motion.
    grid = np.arange(seconds[0], seconds[-1] + dt / 2, dt)
    uniform = np.stack(
        [np.interp(grid, seconds, residual[:, j]) for j in range(6)], axis=1
    )
    window = np.hanning(len(grid))
    frequency = np.fft.rfftfreq(len(grid), dt)
    spectrum = np.abs(np.fft.rfft(uniform * window[:, None], axis=0)) ** 2
    spectrum *= 2 / (len(grid) * np.sum(window**2))

    def high_frequency_rms(signal):
        samples = np.stack(
            [np.interp(grid, seconds, signal[:, j]) for j in range(6)], axis=1
        )
        samples -= samples.mean(axis=0)
        power = np.abs(np.fft.rfft(samples * window[:, None], axis=0)) ** 2
        power *= 2 / (len(grid) * np.sum(window**2))
        return np.sqrt(np.sum(power[frequency >= 20], axis=0)).tolist()

    band = frequency >= 2
    dominant = (
        frequency[band][np.argmax(spectrum[band], axis=0)]
        if band.any()
        else np.zeros(6)
    )
    # Smoothed encoder derivatives are estimates; commanded derivatives come
    # from the post-limiter drive velocity, never the Python stimulus.
    width = min(len(rows) // 4, max(3, round(0.04 / dt)))
    smoothed = np.stack(
        [np.convolve(qd[:, j], np.ones(width) / width, mode="same") for j in range(6)],
        axis=1,
    )
    acceleration = np.gradient(smoothed, seconds, axis=0)
    jerk = np.gradient(acceleration, seconds, axis=0)
    command_acceleration = np.diff(qdc, axis=0) / np.diff(seconds)[:, None]
    interval_centers = (seconds[1:] + seconds[:-1]) / 2
    command_jerk = (
        np.diff(command_acceleration, axis=0) / np.diff(interval_centers)[:, None]
    )
    inner = slice(width + 2, -width - 2) if len(rows) > 2 * width + 6 else slice(None)
    delay = []
    for j in range(6):
        threshold = max(0.015, 0.1 * np.max(np.abs(qdc[:, j])))
        ci = np.flatnonzero(np.abs(qdc[:, j]) >= threshold)
        mi = np.flatnonzero(np.abs(smoothed[:, j]) >= threshold)
        delay.append(
            float(max(0, seconds[mi[0]] - seconds[ci[0]]))
            if len(ci) and len(mi)
            else 0.0
        )
    saturation = np.abs(current) >= 0.95 * limits
    streak = np.zeros(6, dtype=int)
    longest = streak.copy()
    for row in saturation:
        streak = np.where(row, streak + 1, 0)
        longest = np.maximum(longest, streak)
    return {
        "tracking_peak_deg": np.rad2deg(np.max(np.abs(error), axis=0)).tolist(),
        "tracking_rms_deg": np.rad2deg(np.sqrt(np.mean(error**2, axis=0))).tolist(),
        "velocity_residual_rms": np.sqrt(np.mean(residual**2, axis=0)).tolist(),
        "measured_vibration_rms": high_frequency_rms(qd),
        "command_ripple_rms": high_frequency_rms(qdc),
        "oscillation_energy": np.sum(spectrum[band], axis=0).tolist(),
        "dominant_hz": dominant.tolist(),
        "saturation_s": (longest * dt).tolist(),
        "peak_velocity_rad_s": np.max(np.abs(qd), axis=0).tolist(),
        "estimated_peak_acceleration_rad_s2": np.max(
            np.abs(acceleration[inner]), axis=0
        ).tolist(),
        "estimated_peak_jerk_rad_s3": np.max(np.abs(jerk[inner]), axis=0).tolist(),
        "commanded_peaks": np.stack(
            [
                np.max(np.abs(qdc), axis=0),
                np.max(np.abs(command_acceleration), axis=0),
                np.max(np.abs(command_jerk), axis=0),
            ],
            axis=1,
        ).tolist(),
        "response_delay_s": delay,
        "native_period_p99_ms": float(np.quantile(periods, 0.99) * 1000),
        "native_period_max_ms": float(np.max(periods) * 1000),
        "faulted": any(r["flags"] & 1 for r in rows),
    }


def motion_window_peaks(rows: list[dict], dt: float) -> dict:
    """Worst quality in overlapping windows, including the final recorded tail."""
    if len(rows) < 10 or not np.isfinite(dt) or not 0 < dt < 1:
        raise ValueError("Insufficient capture or invalid period")
    # Travel and settled-hold crops leave a ring-down interval between them.
    # Overlap windows across the entire trial so quiet time cannot hide it.
    width = min(len(rows), round(1.2 / dt))
    stride = max(1, round(0.5 / dt))
    starts = sorted(set(range(0, len(rows) - width + 1, stride)) | {len(rows) - width})
    windows = [motion_metrics(rows[a : a + width], dt) for a in starts]
    peaks = {
        field: np.max([w[field] for w in windows], axis=0).tolist()
        for field in (
            "tracking_peak_deg",
            "saturation_s",
            "velocity_residual_rms",
            "measured_vibration_rms",
            "command_ripple_rms",
            "oscillation_energy",
        )
    }
    peaks.update(
        faulted=any(w["faulted"] for w in windows),
        window_duration_s=width * dt,
        window_stride_s=stride * dt,
        window_count=len(windows),
    )
    return peaks
