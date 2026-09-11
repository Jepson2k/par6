"""Explicit acceptance limits for recorded motion and gravity-only holds."""

from dataclasses import asdict, dataclass

import numpy as np

from .analysis import motion_metrics, motion_window_peaks


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
            "version": 7,
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


def gravity_hold_metrics(rows, dt, policy):
    """A low speed or a quiet position servo cannot establish gravity balance."""
    if len(rows) < 2 or not np.isfinite(dt) or not 0 < dt < 1:
        raise ValueError("Insufficient gravity capture or invalid period")
    if np.any(np.diff([r["tick"] for r in rows]) != 1):
        raise ValueError("Capture dropped native ticks")
    if any(r["flags"] & 8 for r in rows):
        raise ValueError("Stale drive feedback in gravity recording")
    if any(r["flags"] >> 8 != 1 or r["flags"] & 22 != 22 for r in rows):
        raise ValueError("Gravity hold requires uninterrupted gravity-only idle")
    seconds = _measurement_seconds(rows)
    for field in ("q", "current_ma", "ilim_ma"):
        values = np.asarray([r[field] for r in rows])
        if values.shape != (len(rows), 6) or not np.isfinite(values).all():
            raise ValueError(f"Invalid gravity measurement: {field}")
    if np.any(np.asarray([r["ilim_ma"] for r in rows]) <= 0):
        raise ValueError("Invalid gravity current limits")
    q = np.rad2deg([r["q"] for r in rows])
    centered = seconds - seconds.mean()
    slope = centered @ (q - q.mean(axis=0)) / (centered @ centered)
    excursion = np.ptp(q, axis=0)
    noise = np.rad2deg(policy["encoder_quantum_rad"]) * 4
    reasons = []
    if seconds[-1] + 2 * dt < policy["gravity_duration_s"]:
        reasons.append("Gravity hold ended before its full observation period")
    for j in range(6):
        if excursion[j] > max(noise[j], policy["gravity_excursion_deg"]):
            reasons.append(f"J{j + 1} gravity-only excursion exceeds acceptance limit")
        if abs(slope[j]) > max(
            noise[j] / policy["gravity_duration_s"], policy["gravity_drift_deg_s"]
        ):
            reasons.append(f"J{j + 1} gravity-only drift exceeds acceptance limit")
    currents = np.asarray([r["current_ma"] for r in rows])
    if any(r["flags"] & 1 for r in rows):
        reasons.append("Controller fault during gravity hold")
    if np.any(np.abs(currents) >= 0.95 * np.asarray([r["ilim_ma"] for r in rows])):
        reasons.append("Current saturation during gravity hold")
    return {
        "valid": not reasons,
        "reasons": reasons,
        "duration_s": float(seconds[-1]),
        "excursion_deg": excursion.tolist(),
        "drift_deg_s": slope.tolist(),
        "mean_current_ma": currents.mean(axis=0).tolist(),
        "table_vibration_measured": False,
    }
