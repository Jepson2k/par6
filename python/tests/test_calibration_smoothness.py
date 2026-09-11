"""Smoothness evidence must compare distinct settings and generalize by pose."""

import copy
import shutil
import tomllib
from pathlib import Path

import numpy as np
import pytest

from par6._par6 import Preview, calibration_config
from par6.calibration.preflight import check_command_peaks, check_stream, stream_scale
from par6.calibration.routines import (
    _assess_smoothness,
    _smoothness_settings,
    centers,
    passes,
)
from par6.calibration.trajectory import move
from par6.calibration.validation import Acceptance, assess_motion

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def smoothness_evidence(tmp_path_factory):
    """Native commands with an explicitly analytic mechanical response.

    The added position/velocity pair is the exact integral/derivative of a
    5 Hz sinusoid before encoder rounding. It is a numerical regression model,
    not evidence that a real arm has this transfer function. No daemon or
    fabricated controller protocol is involved.
    """
    directory = tmp_path_factory.mktemp("smoothness-evidence")
    baseline = np.tile([0.2, 0.4, 1.2], (6, 1))
    candidate = baseline * 0.5
    source = (ROOT / "config/PAR6.toml").read_text()
    shutil.copytree(ROOT / "config/grippers", directory / "grippers")
    config = directory / "PAR6.toml"
    config.write_text(calibration_config(source, stream_limits=baseline.tolist()))
    robot = tomllib.loads(config.read_text())
    policy = Acceptance().describe(robot)
    quantum = np.asarray(policy["encoder_quantum_rad"])
    preview = Preview(
        config=str(config),
        assets=str(ROOT / "assets/par6_description"),
        package_dir=str(ROOT / "assets/par6_description/URDF"),
    )
    dt = preview.tick_dt_s()
    poses = centers([0, -np.pi / 2, np.pi, 0, -0.35, np.pi])[:3]
    evidence = {}
    for label, limits in (("baseline", baseline), ("candidate", candidate)):
        scale = stream_scale(robot, limits)
        readings = {"low": [], "high": []}
        for group, center in enumerate(poses):
            for joint in range(6):
                for direction in (-1, 1):
                    end = center.copy()
                    end[joint] += direction * 0.08
                    _, path = move(center, end, limits)
                    preview.teleport_rad(path[0].tolist())
                    targets = np.vstack(
                        [path, np.repeat(path[-1][None, :], round(2 / 0.02), axis=0)]
                    )
                    output = preview.preview_servo(
                        targets.tolist(), round(0.02 / dt), **scale
                    )
                    qc, qdc = np.asarray(output["q"]), np.asarray(output["qd"])
                    seconds = np.arange(len(qc)) * dt
                    omega = 2 * np.pi * 5
                    for response, rms in (("low", 0.001), ("high", 0.02)):
                        amplitude = np.sqrt(2) * rms
                        q, qd = qc.copy(), qdc.copy()
                        q[:, joint] += amplitude / omega * np.sin(omega * seconds)
                        qd[:, joint] += amplitude * np.cos(omega * seconds)
                        q = np.round(q / quantum) * quantum
                        rows = [
                            {
                                "tick": i,
                                "elapsed_ns": round(seconds[i] * 1e9),
                                "sample_time_ns": round(seconds[i] * 1e9),
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
                        metrics = assess_motion(rows, dt, policy)
                        # Both responses meet absolute motion acceptance. The
                        # missing requirement is relative held-out performance.
                        assert passes(metrics), metrics["acceptance"]
                        check_command_peaks(metrics["commanded_peaks"], limits, dt)
                        assert 0.9 * rms < metrics["velocity_residual_rms"][joint]
                        assert metrics["velocity_residual_rms"][joint] < 1.1 * rms
                        for repeat in range(3):
                            readings[response].append(
                                {
                                    **metrics,
                                    "group": group,
                                    "joint": joint,
                                    "repeat": repeat,
                                    "direction": direction,
                                    "held_out": group == 2,
                                }
                            )
        evidence[label] = readings
    return baseline, candidate, evidence


def improving_reports(evidence):
    return {
        "baseline": evidence["baseline"]["high"],
        "candidate": evidence["candidate"]["low"],
    }


def test_distinct_candidate_improves_training_and_held_out_motion(
    smoothness_evidence,
):
    baseline, candidate, evidence = smoothness_evidence
    result = _assess_smoothness(improving_reports(evidence), baseline, candidate)
    assert result["valid"], result["reasons"]


def test_pooled_improvement_cannot_hide_a_held_out_regression(smoothness_evidence):
    baseline, candidate, evidence = smoothness_evidence
    reports = {
        "baseline": [
            row
            for response, rows in evidence["baseline"].items()
            for row in rows
            if (response == "low") == row["held_out"]
        ],
        "candidate": [
            row
            for response, rows in evidence["candidate"].items()
            for row in rows
            if (response == "high") == row["held_out"]
        ],
    }
    before = np.mean([r["oscillation_energy"] for r in reports["baseline"]], axis=0)
    after = np.mean([r["oscillation_energy"] for r in reports["candidate"]], axis=0)
    assert np.all(after < 0.6 * before)
    for label, lower, upper in (
        ("baseline", 0.0009, 0.0011),
        ("candidate", 0.018, 0.022),
    ):
        held_out = [r for r in reports[label] if r["held_out"]]
        assert all(
            lower < r["velocity_residual_rms"][r["joint"]] < upper for r in held_out
        )
    result = _assess_smoothness(reports, baseline, candidate)
    assert not result["valid"], "Pooled improvement hid a 20-fold held-out RMS increase"
    assert result["reasons"]


def test_identical_quiet_settings_are_not_a_measured_improvement(
    smoothness_evidence,
):
    baseline, _, evidence = smoothness_evidence
    quiet = evidence["baseline"]["low"]
    result = _assess_smoothness(
        {"baseline": quiet, "candidate": quiet}, baseline, baseline.copy()
    )
    assert not result["valid"], (
        "An unchanged below-noise comparison was called improved"
    )
    assert result["reasons"]


@pytest.mark.parametrize("defect", ["missing", "duplicate", "wrong-held-out"])
def test_smoothness_requires_complete_matched_pose_evidence(
    smoothness_evidence, defect
):
    baseline, candidate, evidence = smoothness_evidence
    reports = copy.deepcopy(improving_reports(evidence))
    if defect == "missing":
        reports["candidate"].pop()
    elif defect == "duplicate":
        # Preserve the count: duplicate one trial while losing another key.
        reports["candidate"][-1] = copy.deepcopy(reports["candidate"][0])
    else:
        reports["candidate"][-1]["held_out"] = False
    result = _assess_smoothness(reports, baseline, candidate)
    assert not result["valid"], f"Accepted {defect} validation evidence"
    assert result["reasons"]


def test_selected_settings_change_native_motion_under_loaded_caps(tmp_path):
    """An A/B label must correspond to a realizable change in motor commands."""
    source = (ROOT / "config/PAR6.toml").read_text()
    cases = [
        np.tile([0.2, 0.4, 1.2], (6, 1)),
        # Per-joint velocity clipping at 0.3 cannot be represented by one
        # native speed fraction. Use the actual heterogeneous loaded matrix.
        np.array(
            [
                [0.2, 0.4, 1.2],
                [0.3, 0.6, 1.8],
                [0.4, 0.8, 2.4],
                [0.5, 1.0, 3.0],
                [0.6, 1.2, 3.6],
                [0.4, 0.8, 2.4],
            ]
        ),
    ]
    for case, loaded in enumerate(cases):
        directory = tmp_path / str(case)
        directory.mkdir()
        shutil.copytree(ROOT / "config/grippers", directory / "grippers")
        config = directory / "PAR6.toml"
        execution = np.tile([1.0, 2.0, 6.0], (6, 1))
        config.write_text(
            calibration_config(
                source,
                exec_limits=execution.tolist(),
                stream_limits=loaded.tolist(),
                jog_limits=(loaded * 0.25).tolist(),
            )
        )
        robot = tomllib.loads(config.read_text())
        comparison, candidate = _smoothness_settings(robot, execution)
        comparison, candidate = np.asarray(comparison), np.asarray(candidate)
        assert comparison.shape == candidate.shape == (6, 3)
        assert np.all(comparison <= loaded)
        assert np.all(comparison <= execution)
        assert np.all(candidate > 0)
        assert np.all(candidate <= comparison)
        assert np.all(candidate[:, 1:] < comparison[:, 1:]), (
            "Loaded STREAM below EXEC still needs a distinct smoothing trial"
        )
        assert np.max(comparison[:, 0]) <= 0.3
        preview = Preview(
            config=str(config),
            assets=str(ROOT / "assets/par6_description"),
            package_dir=str(ROOT / "assets/par6_description/URDF"),
        )
        scales = []
        for requested in (comparison, candidate):
            scale = stream_scale(robot, requested)
            effective = loaded * [scale["speed"], scale["accel"], scale["accel"]]
            assert np.allclose(effective, requested, atol=1e-12, rtol=1e-12), (
                "Proposed settings do not match the actual global stream fractions"
            )
            scales.append(scale)
        center = np.array([0, -np.pi / 2, np.pi, 0, -0.35, np.pi])
        for direction in (-1, 1):
            end = center.copy()
            end[0] += direction * 0.08
            peaks = []
            for requested, scale in zip((comparison, candidate), scales):
                times, path = move(center, end, requested)
                assert np.allclose(path[0], center)
                assert np.allclose(path[-1], end)
                actual, commanded = check_stream(
                    preview, times, path, requested, **scale
                )
                assert np.allclose(actual[-1], end, atol=1e-6, rtol=0)
                peaks.append(np.asarray(commanded))
            assert peaks[1][0, 1] < 0.9 * peaks[0][0, 1], (
                "Candidate did not materially reduce native acceleration"
            )
            assert peaks[1][0, 2] < 0.8 * peaks[0][0, 2], (
                "Candidate did not materially reduce native jerk"
            )


def test_velocity_limited_probe_cannot_establish_an_acceleration_change():
    source = (ROOT / "config/PAR6.toml").read_text()
    loaded = np.tile([0.001, 0.4, 1.2], (6, 1))
    execution = np.tile([1.0, 2.0, 6.0], (6, 1))
    robot = tomllib.loads(calibration_config(source, stream_limits=loaded.tolist()))
    with pytest.raises(ValueError, match="cannot excite"):
        _smoothness_settings(robot, execution)


def test_forward_improvement_cannot_hide_backward_regression(smoothness_evidence):
    baseline, candidate, evidence = smoothness_evidence

    def backwards_j1(row):
        return row["joint"] == 0 and row["direction"] == -1

    reports = {
        "baseline": [
            row
            for response, rows in evidence["baseline"].items()
            for row in rows
            if (response == "low") == backwards_j1(row)
        ],
        "candidate": [
            row
            for response, rows in evidence["candidate"].items()
            for row in rows
            if (response == "high") == backwards_j1(row)
        ],
    }
    for group in range(3):
        before, after = [
            np.mean(
                [
                    r["oscillation_energy"]
                    for r in reports[label]
                    if r["group"] == group
                ],
                axis=0,
            )
            for label in ("baseline", "candidate")
        ]
        assert np.sum(after) < 0.3 * np.sum(before)
        assert np.all(after <= np.maximum(1e-5, before * 1.1))
    result = _assess_smoothness(reports, baseline, candidate)
    assert not result["valid"], (
        "A quieter forward direction hid a worse backward stroke"
    )
    assert result["reasons"]
