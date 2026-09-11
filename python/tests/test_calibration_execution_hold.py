"""Queued endpoint evidence must retain droop before the quiet final hold."""

import json
import shutil
import tomllib
from pathlib import Path

import numpy as np

from par6._par6 import ControllerMode, Preview, calibration_config
from par6.calibration.execution import assess_execution
from par6.calibration.trajectory import envelope_move
from par6.calibration.validation import Acceptance, assess_motion

ROOT = Path(__file__).resolve().parents[2]


def test_native_ruckig_endpoint_rejects_early_droop_and_rebound(tmp_path):
    """Native position plan with explicitly analytic encoder responses.

    Preview exposes queued positions, not wire velocity. Command velocities
    below are interval averages reconstructed from every native position at
    the native dt. The response perturbations have analytic q/qd pairs before
    encoder rounding. This tests admission numerically, without a controller,
    plant, fabricated acknowledgement, or claim of measured hardware behavior.
    """
    limits = np.tile([0.2, 0.4, 1.2], (6, 1))
    source = (ROOT / "config/PAR6.toml").read_text()
    shutil.copytree(ROOT / "config/grippers", tmp_path / "grippers")
    config = tmp_path / "PAR6.toml"
    config.write_text(calibration_config(source, exec_limits=limits.tolist()))
    robot = tomllib.loads(config.read_text())
    policy = Acceptance().describe(robot)
    quantum = np.asarray(policy["encoder_quantum_rad"])
    window = np.array(
        [
            [j["limits"]["soft_min_rad"], j["limits"]["soft_max_rad"]]
            for j in robot["joints"]
        ]
    )
    joint = 2
    center = [0, -1.72, 3.34, 0, -0.55, np.pi]
    _, pulse = envelope_move(center, joint, 0, limits[joint, 0], limits, window)
    start, target = pulse[0], pulse[-1]
    preview = Preview(
        config=str(config),
        assets=str(ROOT / "assets/par6_description"),
        package_dir=str(ROOT / "assets/par6_description/URDF"),
        max_points=100_000,
    )
    selected = preview.submit({"type": "select_profile", "profile": "RUCKIG"})
    assert selected is not None and selected["error"] is None
    preview.teleport_rad(start.tolist())
    planned = preview.submit(
        {
            "type": "move_j",
            "angles": np.rad2deg(target).tolist(),
            "speed": 1.0,
            "accel": 1.0,
            "blend_radius": 0.0,
            "rel": False,
        }
    )
    assert planned is not None and planned["error"] is None, planned
    assert not planned["pending"]
    dt = preview.tick_dt_s()
    native_q = np.asarray(planned["joint_trajectory_rad"])
    assert native_q.shape[1] == 6 and len(native_q) > 2
    assert abs(len(native_q) * dt - planned["duration_s"]) < 1e-9
    np.testing.assert_allclose(native_q[-1], target, atol=1e-9, rtol=0)
    # Native RUCKIG samples start at dt; prepend the known t=0 starting pose.
    qc = np.vstack(
        [start, native_q, np.repeat(target[None, :], round(2.5 / dt) + 1, axis=0)]
    )
    qdc = np.vstack([np.zeros(6), np.diff(qc, axis=0) / dt])
    seconds = np.arange(len(qc)) * dt
    after_endpoint = seconds - len(native_q) * dt
    mask = np.arange(6) == joint
    ilim = [j["ilim_ma"] for j in robot["joints"]]

    def observations(q, qd):
        measured_q = np.round(q / quantum) * quantum
        return [
            {
                "tick": i,
                "elapsed_ns": round(seconds[i] * 1e9),
                "sample_time_ns": round(seconds[i] * 1e9),
                "flags": (int(ControllerMode.EXEC) << 8) | 22,
                "q": measured_q[i].tolist(),
                "qd": qd[i].tolist(),
                "q_commanded": qc[i].tolist(),
                "qd_commanded": qdc[i].tolist(),
                # Bounded analytic current keeps this test about hold admission.
                "current_ma": [100] * 6,
                "ilim_ma": ilim,
            }
            for i in range(len(q))
        ]

    healthy_rows = observations(qc, qdc)
    healthy = assess_execution(healthy_rows, dt, policy, target, limits, mask)
    assert healthy["acceptance"]["valid"], healthy["acceptance"]
    assert np.all(np.asarray(healthy["achieved"])[joint] >= 0.9 * limits[joint])

    def smooth_step(t, duration):
        u = np.clip(t / duration, 0, 1)
        position = 10 * u**3 - 15 * u**4 + 6 * u**5
        velocity = (30 * u**2 - 60 * u**3 + 30 * u**4) / duration
        return position, velocity

    amplitude = np.deg2rad(0.2)
    evidence = {"healthy": healthy}
    for response in ("droop", "rebound"):
        duration = 0.4 if response == "droop" else 0.225
        offset, rate = smooth_step(after_endpoint - 0.02, duration)
        if response == "rebound":
            recovery, recovery_rate = smooth_step(
                after_endpoint - 0.02 - duration, duration
            )
            offset -= recovery
            rate -= recovery_rate
        q, qd = qc.copy(), qdc.copy()
        q[:, joint] -= amplitude * offset
        qd[:, joint] -= amplitude * rate
        rows = observations(q, qd)
        # Even overlapping motion-quality windows permit this slow excursion;
        # the missing evidence is the early fixed-target position excursion.
        quality = assess_motion(rows, dt, policy)
        assert quality["acceptance"]["valid"], quality["acceptance"]
        result = assess_execution(rows, dt, policy, target, limits, mask)
        assert result["hold"]["quality"]["acceptance"]["valid"]
        assert result["hold"]["excursion_deg"][joint] < 4 * np.rad2deg(quantum[joint])
        assert abs(result["hold"]["drift_deg_s"][joint]) < 1e-8
        assert result["hold"]["error_deg"][joint] < policy["tracking_deg"]
        assert np.all(np.asarray(result["achieved"])[joint] >= 0.9 * limits[joint])
        recorded = np.asarray([r["q"] for r in rows])
        terminal = recorded[after_endpoint >= 0]
        assert np.rad2deg(np.ptp(terminal[:, joint])) > 0.19
        assert np.rad2deg(np.ptp(terminal[:, joint])) < 0.21
        evidence[response] = result

    (tmp_path / "execution-hold-evidence.json").write_text(
        json.dumps(evidence, indent=2, allow_nan=False) + "\n"
    )
    print(
        json.dumps(
            {
                name: {
                    "valid": result["acceptance"]["valid"],
                    "reasons": result["acceptance"]["reasons"],
                    "final_excursion_deg": result["hold"]["excursion_deg"][joint],
                    "terminal_excursion_deg": result["hold"].get(
                        "terminal_excursion_deg"
                    ),
                }
                for name, result in evidence.items()
            }
        )
    )
    for name in ("droop", "rebound"):
        result = evidence[name]
        assert not result["acceptance"]["valid"], (
            f"The quiet final hold hid an early 0.2-degree {name}"
        )
        assert any(
            "hold excursion" in reason.lower()
            for reason in result["acceptance"]["reasons"]
        )
