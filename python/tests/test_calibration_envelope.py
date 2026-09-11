"""Envelope excitation uses encoder response, not only native drive targets."""

import shutil
import tomllib
from collections import deque
from pathlib import Path

import numpy as np
import pytest

from par6._par6 import Preview, calibration_config
from par6.calibration.analysis import motion_metrics
from par6.calibration.capture import measurement_rows
from par6.calibration.preflight import check_stream, stream_scale
from par6.calibration.routines import passes
from par6.calibration.trajectory import move
from par6.calibration.validation import (
    Acceptance,
    measured_excitation,
    motion_acceptance,
)

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize(
    ("axis", "value"), [(1, 0.3), (2, 1.0)], ids=["acceleration", "jerk"]
)
def test_native_envelope_requires_the_encoder_derivative(tmp_path, axis, value):
    """Real native commands plus an explicit analytic encoder-response model.

    This identifies an admission defect, not the physical arm's transfer
    function. The unquantized low-pass response cannot exercise the derivative
    even though tracking and vibration are acceptable on this short excursion.
    """
    source = (ROOT / "config/PAR6.toml").read_text()
    robot = tomllib.loads(source)
    policy = Acceptance().describe(robot)
    dt = robot["robot"]["tick_dt_s"]
    keys = ("velocity_rad_s", "acceleration_rad_s2", "jerk_rad_s3")
    limits = np.array([[j["limits"]["exec"][k] for k in keys] for j in robot["joints"]])
    candidate = np.minimum(limits, [0.1, 0.3, 1.0])
    candidate[1] = np.maximum(candidate[1], limits[1])
    candidate[1, axis] = value
    center = np.array([0, -np.pi / 2, np.pi, 0, -0.35, np.pi])
    shutil.copytree(ROOT / "config/grippers", tmp_path / "grippers")
    config = tmp_path / "PAR6.toml"
    config.write_text(calibration_config(source, stream_limits=candidate.tolist()))
    preview = Preview(
        config=str(config),
        assets=str(ROOT / "assets/par6_description"),
        package_dir=str(ROOT / "assets/par6_description/URDF"),
    )
    # A short, bounded excursion isolates lag admission from protocol planning.
    a, b = center.copy(), center.copy()
    a[1] -= 0.005
    b[1] += 0.005
    times, positions = move(a, b, candidate)
    check_stream(preview, times, positions, candidate)
    targets = np.vstack(
        [
            positions,
            np.repeat(positions[-1][None, :], round(2 / (times[1] - times[0])), axis=0),
        ]
    )
    preview.teleport_rad(positions[0].tolist())
    command = preview.preview_servo(targets.tolist(), round((times[1] - times[0]) / dt))
    qc, qdc = np.asarray(command["q"]), np.asarray(command["qd"])

    def measured(q, qd):
        rows = [
            {
                "tick": i,
                "elapsed_ns": round(i * dt * 1e9),
                "flags": (5 << 8) | 22,
                "q": q[i].tolist(),
                "qd": qd[i].tolist(),
                "q_commanded": qc[i].tolist(),
                "qd_commanded": qdc[i].tolist(),
                "current_ma": [100] * 6,
                "ilim_ma": [2500] * 6,
            }
            for i in range(len(q))
        ]
        result = motion_metrics(rows, dt)
        result["acceptance"] = motion_acceptance(result, policy)
        return result, rows

    quantum = np.asarray(policy["encoder_quantum_rad"])
    responsive, responsive_rows = measured(np.round(qc / quantum) * quantum, qdc)
    assert passes(responsive), responsive["acceptance"]
    responsive_bound = measured_excitation(responsive, responsive_rows, policy)[1, axis]
    if axis == 1:
        assert responsive_bound >= 0.9 * value
    else:
        # The shortest jerk trial cannot resolve 90% from these encoder counts.
        # Longer constant-jerk native controls are exercised below instead.
        assert responsive_bound < 0.9 * value

    # Match the current driver telemetry contract: rounded encoder counts at
    # 6250 Hz, a 20-sample velocity average, then integer ticks/s on CAN.
    # See sim/driver.rs::control_iteration and spectral/codec.rs::decode_frame.
    rate, history_length, lag = 6250, 20, 0.08
    fw_dt = 1 / rate
    alpha = np.exp(-fw_dt / lag)
    true_v = np.zeros_like(qdc)
    q = qc.copy()
    qd = np.zeros_like(qdc)
    position, velocity = qc[0, 1], 0.0
    previous = round(position / quantum[1])
    history = deque([0.0] * history_length, maxlen=history_length)
    q[0, 1] = previous * quantum[1]
    for i in range(1, len(qdc)):
        command = qdc[i, 1]
        for _ in range(round(dt * rate)):
            position += command * fw_dt + (velocity - command) * lag * (1 - alpha)
            velocity = alpha * velocity + (1 - alpha) * command
            encoder = round(position / quantum[1])
            history.append(float(np.trunc((encoder - previous) * rate)))
            previous = encoder
        true_v[i, 1] = velocity
        q[i, 1] = encoder * quantum[1]
        qd[i, 1] = np.trunc(sum(history) / history_length) * quantum[1]
    sluggish, sluggish_rows = measured(q, qd)
    assert passes(sluggish), sluggish["acceptance"]
    assert sluggish["commanded_peaks"][1][axis] >= 0.9 * value
    derivative = true_v
    for _ in range(axis):
        derivative = np.gradient(derivative, dt, axis=0)
    assert np.max(np.abs(derivative[:, 1])) < 0.8 * value
    if axis == 2:
        # The velocity-difference estimator inflated jerk enough to pass the
        # preceding measured-peak gate, while ordinary vibration checks passed.
        assert sluggish["estimated_peak_jerk_rad_s3"][1] > value
    bounds = measured_excitation(sluggish, sluggish_rows, policy)
    assert bounds[1, axis] < 0.9 * value
    if axis == 2:
        # Deterministic native tick time survives a faster host catch-up clock.
        # Using only elapsed_ns would inflate these same position derivatives.
        catch_up = measurement_rows(
            [{**r, "elapsed_ns": r["elapsed_ns"] // 2} for r in sluggish_rows],
            dt,
            simulator=True,
        )
        np.testing.assert_allclose(
            measured_excitation(sluggish, catch_up, policy), bounds, atol=1e-12
        )
        catch_up[-1]["elapsed_ns"] += 200_000_000
        with pytest.raises(ValueError, match="timing"):
            measured_excitation(sluggish, catch_up, policy)


@pytest.mark.parametrize(
    ("triple", "loaded", "provable"),
    [
        ((0.08, 0.24, 0.8), (0.08, 0.24, 0.8), False),
        ((0.2, 0.4, 1.2), (0.2, 0.4, 1.2), True),
        ((0.2, 0.3, 0.6), (0.2, 0.3, 0.6), True),
        ((0.1, 0.2, 0.6), (0.2, 0.4, 1.2), True),
        ((0.08, 0.24, 0.8), (0.2, 0.4, 1.2), False),
    ],
)
def test_coupled_native_evidence_respects_encoder_resolution(
    tmp_path, triple, loaded, provable
):
    """Native output must prove every derivative after encoder rounding allowance."""
    from par6.calibration.trajectory import coupled_move

    source = (ROOT / "config/PAR6.toml").read_text()
    robot = tomllib.loads(source)
    policy = Acceptance().describe(robot)
    dt = robot["robot"]["tick_dt_s"]
    limits = np.tile(triple, (6, 1))
    center = np.array([0, -1.72, 3.34, 0, -0.55, np.pi])
    window = np.array(
        [
            [j["limits"]["soft_min_rad"], j["limits"]["soft_max_rad"]]
            for j in robot["joints"]
        ]
    )
    shutil.copytree(ROOT / "config/grippers", tmp_path / "grippers")
    config = tmp_path / "PAR6.toml"
    config.write_text(calibration_config(source, stream_limits=[loaded] * 6))
    scale = stream_scale(tomllib.loads(config.read_text()), limits)
    preview = Preview(
        config=str(config),
        assets=str(ROOT / "assets/par6_description"),
        package_dir=str(ROOT / "assets/par6_description/URDF"),
    )
    admitted = []
    quantum = np.asarray(policy["encoder_quantum_rad"])
    for pattern in (np.ones(6), np.array([1, -1, 1, -1, 1, -1])):
        for direction in (-1, 1):
            times, positions = coupled_move(
                center, limits, window, signs=direction * pattern
            )
            assert np.all(positions >= window[:, 0])
            assert np.all(positions <= window[:, 1])
            derivative = positions
            for axis in range(3):
                derivative = np.gradient(derivative, times, axis=0)
                assert np.all(
                    np.max(np.abs(derivative), axis=0) <= limits[:, axis] * 1.001
                )
            check_stream(preview, times, positions, limits, **scale)
            preview.teleport_rad(positions[0].tolist())
            targets = np.vstack(
                [positions, np.repeat(positions[-1][None, :], round(2 / 0.02), axis=0)]
            )
            command = preview.preview_servo(targets.tolist(), round(0.02 / dt), **scale)
            # Ideal encoder tracking isolates whether the native limiter and
            # position bounds permit this excitation; it is not a plant test.
            rows = [
                {
                    "tick": i,
                    "elapsed_ns": round(i * dt * 1e9),
                    "flags": (5 << 8) | 22,
                    "q": (np.round(np.asarray(q) / quantum) * quantum).tolist(),
                    "q_commanded": q,
                    "qd": velocity,
                    "qd_commanded": velocity,
                    "current_ma": [100] * 6,
                    "ilim_ma": [2500] * 6,
                }
                for i, (q, velocity) in enumerate(zip(command["q"], command["qd"]))
            ]
            metrics = motion_metrics(rows, dt)
            metrics["acceptance"] = motion_acceptance(metrics, policy)
            assert passes(metrics), metrics["acceptance"]
            admitted.append(
                bool(np.all(measured_excitation(metrics, rows, policy) >= 0.9 * limits))
            )

    # Insufficient positional evidence is censored, even for ideal tracking.
    # Longer ramps provide a real positive control without weakening 90%.
    assert all(admitted) is provable

    with pytest.raises(ValueError, match="insufficient window"):
        coupled_move(center, limits, np.column_stack([center - 0.01, center + 0.01]))
    incompatible = limits.copy()
    incompatible[:, 1] = 2 * np.sqrt(limits[:, 0] * limits[:, 2])
    with pytest.raises(ValueError, match="cannot excite acceleration"):
        coupled_move(center, incompatible, window)


def test_each_stream_limit_probe_is_observable_under_the_loaded_caps(tmp_path):
    from par6.calibration.trajectory import envelope_move

    source = (ROOT / "config/PAR6.toml").read_text()
    loaded = np.array([[0.2, 0.4, 1.2]] * 6)
    shutil.copytree(ROOT / "config/grippers", tmp_path / "grippers")
    config = tmp_path / "PAR6.toml"
    config.write_text(calibration_config(source, stream_limits=loaded.tolist()))
    robot = tomllib.loads(config.read_text())
    policy = Acceptance().describe(robot)
    quantum = np.asarray(policy["encoder_quantum_rad"])
    preview = Preview(
        config=str(config),
        assets=str(ROOT / "assets/par6_description"),
        package_dir=str(ROOT / "assets/par6_description/URDF"),
    )
    dt = preview.tick_dt_s()
    center = np.deg2rad([0, -90, 170, 0, -20, 180])
    window = np.array(
        [
            [j["limits"]["soft_min_rad"], j["limits"]["soft_max_rad"]]
            for j in robot["joints"]
        ]
    )
    for joint in range(6):
        for dimension, value in enumerate((0.1, 0.3, 1.0)):
            requested = loaded.copy()
            requested[joint, dimension] = value
            scale = stream_scale(robot, requested)
            effective = loaded * [scale["speed"], scale["accel"], scale["accel"]]
            times, positions = envelope_move(
                center, joint, dimension, value, effective, window
            )
            assert np.all(positions >= window[:, 0])
            assert np.all(positions <= window[:, 1])
            others = [j for j in range(6) if j != joint]
            assert np.max(np.ptp(positions[:, others], axis=0)) == 0
            for path in (positions, positions[::-1]):
                predicted, peaks = check_stream(
                    preview, times, path, requested, **scale
                )
                # Ideal quantized response asks whether the planned native
                # waveform is measurable, not whether a physical plant tracks it.
                rows = [
                    {
                        "q": (np.round(q / quantum) * quantum).tolist(),
                        "elapsed_ns": round(i * dt * 1e9),
                    }
                    for i, q in enumerate(predicted)
                ]
                supported = measured_excitation(
                    {"commanded_peaks": peaks}, rows, policy
                )
                assert supported[joint, dimension] >= 0.9 * value, (
                    joint,
                    dimension,
                    supported[joint, dimension],
                )
