"""Check the native stream output before a calibration stimulus reaches a drive."""

import numpy as np


def check_stream(
    preview, times, positions, limits, *, speed=1.0, accel=1.0, check_path=None
):
    """Reject motor commands outside the experiment's v/a/j envelope.

    This exercises the native limiter and its wire feedforward conversion. The
    preview omits the optional command low-pass, so this checks the unfiltered
    input. Stopping projections assume ideal tracking; runtime collision checks
    still use measured velocity. Recorded commands are checked after execution.
    """
    if (
        not np.isfinite([speed, accel]).all()
        or not 0 < speed <= 1
        or not 0 < accel <= 1
    ):
        raise ValueError("Calibration stream fractions must be in (0, 1]")
    dt = preview.tick_dt_s()
    times, positions, limits = map(np.asarray, (times, positions, limits))
    if limits.shape != (6, 3) or not np.isfinite(limits).all() or np.any(limits <= 0):
        raise ValueError("Need finite positive six-joint experiment limits")
    periods = np.diff(times)
    ticks = int(round(float(periods[0]) / dt))
    if ticks < 1 or not np.allclose(periods, ticks * dt, atol=1e-8, rtol=0):
        raise ValueError("Stimulus period must be a fixed multiple of the native tick")
    preview.teleport_rad(positions[0].tolist())
    # Include the final hold: arrival can excite the drive too.
    targets = np.vstack(
        [
            positions,
            np.repeat(positions[-1][None, :], int(np.ceil(2 / periods[0])), axis=0),
        ]
    )
    output = preview.preview_servo(targets.tolist(), ticks, speed=speed, accel=accel)
    if check_path is not None:
        check_path(positions)
        check_path(np.asarray(output["q"]))
        # ServoJ gates both the limiter output and the submitted target with
        # its stopping travel. Checking positions alone misses this margin.
        for field in ("q_stop", "target_stop"):
            try:
                check_path(np.asarray(output[field]))
            except ValueError as exc:
                raise ValueError(
                    f"Native stopping projection ({field}): {exc}"
                ) from exc
    velocity = np.asarray(output["qd"])
    acceleration = np.diff(velocity, axis=0) / dt
    jerk = np.diff(acceleration, axis=0) / dt
    peaks = np.stack(
        [np.max(np.abs(a), axis=0) for a in (velocity, acceleration, jerk)], axis=1
    )
    check_command_peaks(peaks, limits, dt)
    return np.asarray(output["q"]), peaks.tolist()


def check_command_peaks(peaks, limits, dt, *, label="Native stream preflight"):
    """Enforce the same motor-output bounds on preview and recorded commands."""
    peaks, limits = np.asarray(peaks, dtype=float), np.asarray(limits, dtype=float)
    if (
        peaks.shape != (6, 3)
        or limits.shape != (6, 3)
        or not np.isfinite(peaks).all()
        or not np.isfinite(limits).all()
        or np.any(peaks < 0)
        or np.any(limits <= 0)
        or not np.isfinite(dt)
        or not 0 < dt < 1
    ):
        raise ValueError("Invalid native command peaks or bounds")
    # Interval-average velocity preserves the OTG derivative bounds.
    bounds = limits
    violations = np.argwhere(peaks > bounds * 1.001 + 1e-6)
    if len(violations):
        joint, dimension = violations[0]
        dimension_name = ("speed", "acceleration", "jerk")[dimension]
        raise ValueError(
            f"{label}: J{joint + 1} {dimension_name} "
            f"{peaks[joint, dimension]:.4g} exceeds experiment bound "
            f"{bounds[joint, dimension]:.4g}; use suitable stream limits"
        )


def stream_scale(robot, limits):
    """Conservative global fractions for the actual loaded native stream caps."""
    keys = ("velocity_rad_s", "acceleration_rad_s2", "jerk_rad_s3")
    loaded = np.array(
        [
            [j["limits"].get("stream", {}).get(k, j["limits"][k]) for k in keys]
            for j in robot["joints"]
        ]
    )
    requested = np.asarray(limits)
    if (
        loaded.shape != (6, 3)
        or requested.shape != (6, 3)
        or not np.isfinite(loaded).all()
        or not np.isfinite(requested).all()
        or np.any(loaded <= 0)
        or np.any(requested <= 0)
    ):
        raise ValueError("Stream scaling requires finite positive six-joint limits")
    # Native acceleration and jerk share a fraction. An incompatible tuple
    # remains unproven by the measured-excitation gate; never enlarge a bound.
    return {
        "speed": float(min(1.0, np.min(requested[:, 0] / loaded[:, 0]))),
        "accel": float(min(1.0, np.min(requested[:, 1:] / loaded[:, 1:]))),
    }
