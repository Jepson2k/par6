"""Bounded calibration stimuli; all coordinates and limits are SI."""

from __future__ import annotations

import numpy as np


def move(start, end, limits, dt=0.02):
    """Rest-to-rest quintic with analytically bounded speed, acceleration, jerk."""
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    limits = np.asarray(limits, dtype=float)
    if (
        start.shape != (6,)
        or end.shape != (6,)
        or limits.shape != (6, 3)
        or not all(np.isfinite(x).all() for x in [start, end, limits])
        or np.any(limits <= 0)
        or not 0 < dt <= 0.02
    ):
        raise ValueError("Need finite six-joint positions and positive v/a/j limits")
    d = np.abs(end - start)
    duration = max(
        dt,
        float(np.max(1.875 * d / limits[:, 0])),
        float(np.max(np.sqrt((10 / np.sqrt(3)) * d / limits[:, 1]))),
        float(np.max(np.cbrt(60 * d / limits[:, 2]))),
    )
    count = int(np.ceil(duration / dt))
    duration = count * dt
    t = np.arange(count + 1) * dt
    u = t / duration
    h = 10 * u**3 - 15 * u**4 + 6 * u**5
    return t, start + h[:, None] * (end - start)


def sweeps(centers, limits, *, amplitude=0.12):
    """Opposite-direction paths, split by whole center for held-out validation."""
    if len(centers) < 3:
        raise ValueError("Need at least three distinct arm configurations")
    for group, center in enumerate(centers):
        for j in range(6):
            for direction in [-1, 1]:
                a = np.array(center, dtype=float)
                b = a.copy()
                a[j] -= direction * amplitude
                b[j] += direction * amplitude
                t, q = move(a, b, limits)
                yield {
                    "group": group,
                    "joint": j,
                    "direction": direction,
                    "start": a.tolist(),
                    "end": b.tolist(),
                    "times": t,
                    "positions": q,
                    "held_out": group % 3 == 2,
                }


def candidate_values(baseline: float, ceiling: float):
    if not 0 < baseline <= ceiling or not np.isfinite([baseline, ceiling]).all():
        raise ValueError("Invalid motion envelope")
    value = baseline
    yield value
    while value < ceiling:
        value = min(ceiling, value * 1.1)
        yield value


def envelope_move(center, joint, dimension, value, limits, window, *, dt=0.02):
    """A single-axis velocity pulse with observable constant-jerk ramps.

    Other derivative bounds determine the ramp duration and required travel.
    Insufficient room or an unreachable acceleration censors the experiment.
    """
    center, limits, window = (
        np.asarray(v, dtype=float) for v in (center, limits, window)
    )
    if (
        center.shape != (6,)
        or limits.shape != (6, 3)
        or window.shape != (6, 2)
        or not 0 <= joint < 6
        or not 0 <= dimension < 3
        or not np.isfinite(value)
        or value <= 0
        or not all(np.isfinite(v).all() for v in (center, limits, window))
        or np.any(limits <= 0)
        or np.any(window[:, 0] >= window[:, 1])
        or np.any(center < window[:, 0])
        or np.any(center > window[:, 1])
        or not 0 < dt <= 0.02
    ):
        raise ValueError("Invalid six-joint envelope probe or limits")
    limits = limits.copy()
    limits[joint, dimension] = value
    speed, acceleration, jerk = limits[joint]
    ramp = min(acceleration / jerk, np.sqrt(speed / jerk))
    peak_acceleration = jerk * ramp
    if (dimension == 1 and peak_acceleration < 0.95 * value) or ramp < dt:
        raise ValueError("Other bounds cannot excite this derivative")
    constant = max(0.0, speed / peak_acceleration - ramp)
    phases = [
        (ramp, jerk),
        (constant, 0.0),
        (ramp, -jerk),
        (0.5, 0.0),
        (ramp, -jerk),
        (constant, 0.0),
        (ramp, jerk),
    ]
    duration = sum(length for length, _ in phases)
    times = np.arange(int(np.ceil(duration / dt)) + 1) * dt
    values = np.zeros(len(times))
    at = position = velocity = accel = 0.0
    for length, phase_jerk in phases:
        active = (times >= at) & (times < at + length)
        elapsed = times[active] - at
        values[active] = (
            position
            + velocity * elapsed
            + 0.5 * accel * elapsed**2
            + phase_jerk * elapsed**3 / 6
        )
        position += (
            velocity * length + 0.5 * accel * length**2 + phase_jerk * length**3 / 6
        )
        velocity += accel * length + 0.5 * phase_jerk * length**2
        accel += phase_jerk * length
        at += length
    values[times >= duration] = position
    room = min(center[joint] - window[joint, 0], window[joint, 1] - center[joint])
    if position / 2 > 0.8 * room:
        raise ValueError("Available travel cannot excite this derivative")
    positions = np.tile(center, (len(times), 1))
    positions[:, joint] += values - position / 2
    return times, positions


def coupled_move(center, limits, window, *, signs=None, dt=0.02):
    """All-axis velocity pulses exercise v/a/j in both movement directions.

    Each pulse integrates constant-jerk ramps exactly. A velocity plateau makes
    peak speed observable; the intervening rest lets the native position OTG
    finish before the opposite pulse, avoiding a mid-flight reversal transient.
    """
    center, limits, window = (
        np.asarray(value, dtype=float) for value in (center, limits, window)
    )
    signs = np.ones(6) if signs is None else np.asarray(signs, dtype=float)
    if (
        center.shape != (6,)
        or limits.shape != (6, 3)
        or window.shape != (6, 2)
        or signs.shape != (6,)
        or not all(np.isfinite(value).all() for value in (center, limits, window))
        or np.any(limits <= 0)
        or np.any(window[:, 0] >= window[:, 1])
        or np.any(center < window[:, 0])
        or np.any(center > window[:, 1])
        or not np.isin(signs, [-1, 1]).all()
        or not 0 < dt <= 0.02
    ):
        raise ValueError(
            "Need finite six-joint limits, center, window and direction signs"
        )
    profiles = []
    for joint, (speed, acceleration, jerk) in enumerate(limits):
        ramp = min(acceleration / jerk, np.sqrt(speed / jerk))
        peak_acceleration = jerk * ramp
        if peak_acceleration < 0.95 * acceleration:
            raise ValueError(
                f"J{joint + 1} rest-separated pulse cannot excite acceleration "
                "within the velocity and jerk bounds"
            )
        if ramp < dt:
            raise ValueError(
                f"J{joint + 1} jerk ramp is shorter than the stimulus period"
            )
        constant = max(0.0, speed / peak_acceleration - ramp)
        segments = []
        for pulse, direction in enumerate((-1, 1, 1, -1)):
            segments.extend(
                [(ramp, direction * jerk), (constant, 0.0), (ramp, -direction * jerk)]
            )
            if pulse in (0, 2):
                segments.append((0.5, 0.0))
            elif pulse == 1:
                segments.append((2.0, 0.0))
        time = position = velocity = accel = 0.0
        phases, extrema = [], [0.0]
        for duration, phase_jerk in segments:
            if duration == 0:
                continue
            phases.append((time, duration, position, velocity, accel, phase_jerk))
            position += (
                velocity * duration
                + 0.5 * accel * duration**2
                + phase_jerk * duration**3 / 6
            )
            velocity += accel * duration + 0.5 * phase_jerk * duration**2
            accel += phase_jerk * duration
            time += duration
            # Velocity has one sign throughout each pulse; the exact extrema
            # therefore occur at phase boundaries, not between sampled targets.
            extrema.append(position)
        low, high = min(extrema), max(extrema)
        offset, radius = (low + high) / 2, (high - low) / 2
        room = min(center[joint] - window[joint, 0], window[joint, 1] - center[joint])
        if radius > 0.8 * room:
            raise ValueError(
                f"J{joint + 1} has insufficient window for the coupled pulse"
            )
        profiles.append((phases, time, position, offset))
    times = np.arange(int(np.ceil(max(p[1] for p in profiles) / dt)) + 1) * dt
    positions = []
    for joint, (phases, _, final, offset) in enumerate(profiles):
        values = np.full(len(times), final)
        for start, duration, position, velocity, accel, jerk in phases:
            active = (times >= start) & (times < start + duration)
            elapsed = times[active] - start
            values[active] = (
                position
                + velocity * elapsed
                + 0.5 * accel * elapsed**2
                + jerk * elapsed**3 / 6
            )
        positions.append(center[joint] + signs[joint] * (values - offset))
    return times, np.asarray(positions).T
