"""Recorded-motion quality: drive targets against encoders, never against the
Python stimulus. Acceptance limits are commissioning bounds, not mechanical maxima."""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np


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


@dataclass(frozen=True)
class Acceptance:
    # Conservative commissioning limits, not certified mechanical maxima.
    tracking_deg: float = 0.5
    saturation_s: float = 0.2
    velocity_residual_rad_s: float = 0.03
    vibration_rad_s: float = 0.03
    command_ripple_rad_s: float = 0.015
    gravity_residual_nm: float = 0.05
    gravity_duration_s: float = 20.0
    gravity_excursion_deg: float = 0.1
    gravity_drift_deg_s: float = 0.005
    gravity_abort_deg: float = 0.25

    def __post_init__(self):
        if not all(np.isfinite(v) and v > 0 for v in asdict(self).values()):
            raise ValueError("Acceptance limits must be finite and positive")
        if self.gravity_duration_s < 20:
            raise ValueError("Gravity validation requires at least 20 seconds")
        if self.gravity_abort_deg <= self.gravity_excursion_deg:
            raise ValueError("Gravity abort must exceed accepted excursion")

    def describe(self, robot):
        if robot["robot"]["tick_dt_s"] >= 0.025:
            raise ValueError(
                "Vibration validation requires native sampling above 40 Hz"
            )
        quantum = np.array(
            [
                2 * np.pi / (2 ** j["encoder_bits"] * j["gear_ratio"])
                for j in robot["joints"]
            ]
        )
        return {
            "version": 8,
            "tick_dt_s": robot["robot"]["tick_dt_s"],
            **asdict(self),
            "encoder_quantum_rad": quantum.tolist(),
            "velocity_noise_floor_rad_s": (
                2 * quantum / robot["robot"]["tick_dt_s"]
            ).tolist(),
            "table_vibration_measured": False,
        }


def motion_acceptance(metrics, policy, *, allow_oscillation=False):
    reasons = []
    if metrics["faulted"]:
        reasons.append("Controller fault")
    checks = [
        ("tracking_peak_deg", policy["tracking_deg"], "tracking error"),
        ("saturation_s", policy["saturation_s"], "current saturation"),
    ]
    if not allow_oscillation:
        noise = np.asarray(policy["velocity_noise_floor_rad_s"])
        checks += [
            (
                "velocity_residual_rms",
                np.maximum(noise, policy["velocity_residual_rad_s"]),
                "velocity residual",
            ),
            (
                "measured_vibration_rms",
                np.maximum(noise, policy["vibration_rad_s"]),
                "measured vibration",
            ),
            ("command_ripple_rms", policy["command_ripple_rad_s"], "command ripple"),
        ]
    for field, limit, label in checks:
        values = np.asarray(metrics[field])
        if values.shape != (6,) or not np.isfinite(values).all():
            raise ValueError(f"Invalid {label} measurements")
        for j in np.flatnonzero(values > limit):
            reasons.append(f"J{j + 1} {label} exceeds acceptance limit")
    return {
        "valid": not reasons,
        "reasons": reasons,
        "oscillation_checked": not allow_oscillation,
    }


def assess_motion(rows, dt, policy, *, allow_oscillation=False):
    """Decide final trial quality independently of live-check scheduling."""
    metrics = motion_metrics(rows, dt)
    windows = motion_window_peaks(rows, dt)
    whole = motion_acceptance(metrics, policy, allow_oscillation=allow_oscillation)
    recent = motion_acceptance(windows, policy, allow_oscillation=allow_oscillation)
    metrics["window_peaks"] = windows
    metrics["acceptance"] = {
        "valid": whole["valid"] and recent["valid"],
        "reasons": whole["reasons"] + ["Window: " + r for r in recent["reasons"]],
        "oscillation_checked": not allow_oscillation,
    }
    return metrics


def _measurement_seconds(rows):
    # A simulator catch-up tick advances the plant by dt, but a long host pause
    # must still invalidate evidence even when the plant clock is regular.
    for field in ("elapsed_ns", "sample_time_ns"):
        origin = rows[0].get(field, rows[0]["elapsed_ns"])
        seconds = (
            np.asarray(
                [row.get(field, row["elapsed_ns"]) - origin for row in rows],
                dtype=float,
            )
            * 1e-9
        )
        periods = np.diff(seconds)
        if (
            not np.isfinite(seconds).all()
            or np.any(periods <= 0)
            or np.max(periods) > 0.1
        ):
            raise ValueError("Invalid native capture or measurement timing")
    return seconds


def measured_excitation(metrics, rows, policy):
    """Position-supported lower bounds on exercised speed, acceleration and jerk.

    A divided difference times the derivative's factorial averages that
    derivative over its time nodes. For positions sampled at those times,
    subtracting the worst-case encoder rounding contribution gives a lower bound
    on the true peak, even when reported drive velocity has quantization spikes.
    This is motion evidence, not torque identification or a certified mechanical
    limit; position acquisition timing is assumed to match the measurement clock.
    """
    commanded = np.asarray(metrics["commanded_peaks"], dtype=float)
    quantum = np.asarray(policy["encoder_quantum_rad"], dtype=float)
    if (
        commanded.shape != (6, 3)
        or not np.isfinite(commanded).all()
        or np.any(commanded < 0)
        or quantum.shape != (6,)
        or not np.isfinite(quantum).all()
        or np.any(quantum <= 0)
        or len(rows) < 4
    ):
        raise ValueError(
            "Excitation requires native positions, command peaks and encoder resolution"
        )
    positions = np.asarray([row["q"] for row in rows], dtype=float)
    seconds = _measurement_seconds(rows)
    if positions.shape != (len(rows), 6) or not np.isfinite(positions).all():
        raise ValueError("Invalid native position evidence for excitation")
    period = float(np.median(np.diff(seconds)))
    shortest = max(1, int(round(0.02 / period)))
    longest = min(len(rows) - 1, max(shortest, int(round(0.2 / period))))
    measured = np.zeros((6, 3))
    for order, factorial in ((1, 1), (2, 2), (3, 6)):
        for spacing in range(shortest, longest + 1):
            count = len(rows) - order * spacing
            if count <= 0:
                break
            indices = np.arange(count)[:, None] + spacing * np.arange(order + 1)
            nodes = seconds[indices]
            weights = np.full(nodes.shape, float(factorial))
            for k in range(order + 1):
                for other in range(order + 1):
                    if other != k:
                        weights[:, k] /= nodes[:, k] - nodes[:, other]
            # Removing the common angle avoids cancellation of a large joint
            # offset; the rounding bound still includes every original node.
            shifted = positions[indices] - positions[indices[:, :1]]
            terms = weights[:, :, None] * shifted
            derivative = np.sum(terms, axis=1)
            rounding = np.sum(np.abs(weights), axis=1)[:, None] * quantum / 2
            arithmetic = 16 * np.finfo(float).eps * np.sum(np.abs(terms), axis=1)
            lower = np.maximum(0, np.abs(derivative) - rounding - arithmetic)
            measured[:, order - 1] = np.maximum(
                measured[:, order - 1], np.max(lower, axis=0)
            )
    return np.minimum(commanded, measured)


def steady_samples(rows, dt):
    """Slow, unsaturated, low-acceleration moving rows with smoothed speeds —
    the only rows gravity identification may see."""
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
