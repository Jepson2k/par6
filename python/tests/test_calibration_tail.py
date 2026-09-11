"""Final motion quality must include the tail after the last live FFT window."""

import shutil
import tomllib
from pathlib import Path

import numpy as np

from par6._par6 import Preview, calibration_config
from par6.calibration.analysis import motion_metrics
from par6.calibration.preflight import check_command_peaks, check_stream
from par6.calibration.trajectory import coupled_move
from par6.calibration.validation import (
    Acceptance,
    assess_motion,
    measured_excitation,
    motion_acceptance,
)

ROOT = Path(__file__).resolve().parents[2]


def test_final_assessment_rejects_vibration_after_the_last_live_window(tmp_path):
    """Native commands plus an explicitly analytic position/velocity response.

    This tests the final production assessment, not a physical transfer function.
    The burst ends early enough for the actual settling predicate to complete
    before another live window would be launched.
    """
    limits = np.tile([0.2, 0.4, 1.2], (6, 1))
    source = (ROOT / "config/PAR6.toml").read_text()
    shutil.copytree(ROOT / "config/grippers", tmp_path / "grippers")
    config = tmp_path / "PAR6.toml"
    config.write_text(calibration_config(source, stream_limits=limits.tolist()))
    robot = tomllib.loads(config.read_text())
    policy = Acceptance().describe(robot)
    quantum = np.asarray(policy["encoder_quantum_rad"])
    window = [
        [j["limits"]["soft_min_rad"], j["limits"]["soft_max_rad"]]
        for j in robot["joints"]
    ]
    preview = Preview(
        config=str(config),
        assets=str(ROOT / "assets/par6_description"),
        package_dir=str(ROOT / "assets/par6_description/URDF"),
    )
    dt = preview.tick_dt_s()
    times, path = coupled_move(
        [0, -1.72, 3.34, 0, -0.55, np.pi],
        limits,
        window,
        signs=[1, -1, 1, -1, 1, -1],
    )
    check_stream(preview, times, path, limits)
    preview.teleport_rad(path[0].tolist())
    targets = np.vstack([path, np.repeat(path[-1][None, :], round(2 / 0.02), axis=0)])
    output = preview.preview_servo(targets.tolist(), round(0.02 / dt))
    finish, burst_start, burst_duration = 7.18, 6.75, 0.1
    count = round(finish / dt) + 1
    seconds = np.arange(count) * dt
    qc, qdc = np.asarray(output["q"])[:count], np.asarray(output["qd"])[:count]

    def observations(q, qd):
        return [
            {
                "tick": i,
                "elapsed_ns": round(seconds[i] * 1e9),
                "flags": (5 << 8) | 22,
                "q": q[i].tolist(),
                "qd": qd[i].tolist(),
                "q_commanded": qc[i].tolist(),
                "qd_commanded": qdc[i].tolist(),
                "current_ma": [100] * 6,
                "ilim_ma": [2500] * 6,
            }
            for i in range(count)
        ]

    nominal = observations(np.round(qc / quantum) * quantum, qdc)
    assert assess_motion(nominal, dt, policy)["acceptance"]["valid"]
    q, qd = qc.copy(), qdc.copy()
    active = (seconds >= burst_start) & (seconds < burst_start + burst_duration)
    phase = 2 * np.pi * 30 * (seconds[active] - burst_start)
    qd[active, 1] += 0.18 * np.sin(phase)
    # Integrate the velocity burst exactly; three cycles return to zero offset.
    q[active, 1] += 0.18 / (2 * np.pi * 30) * (1 - np.cos(phase))
    q = np.round(q / quantum) * quantum
    rows = observations(q, qd)

    stable = settled_at = None
    motion = robot.get("motion", {})
    tolerance = motion.get("settle_tolerance_rad", 0.01)
    for stamp in np.arange(times[-1] + 0.02, finish + dt / 2, 0.02):
        i = round(stamp / dt)
        if (
            np.max(np.abs(q[i] - path[-1])) <= tolerance
            and np.max(np.abs(qd[i])) < 0.03
        ):
            stable = stamp if stable is None else stable
            if stamp - stable > 0.3:
                settled_at = stamp
                break
        else:
            stable = None
    assert settled_at is not None
    assert burst_start + burst_duration < settled_at < 7.2
    assert settled_at < times[-1] + motion.get("settle_timeout_s", 2)

    # The 0.5 s live schedule last examines a window ending at 6.7 s.
    endpoints = np.arange(1.2, settled_at, 0.5)
    assert endpoints[-1] < burst_start
    for endpoint in endpoints:
        stop = round(endpoint / dt) + 1
        live = rows[stop - round(1.2 / dt) : stop]
        assert motion_acceptance(motion_metrics(live, dt), policy)["valid"]
    whole = motion_metrics(rows, dt)
    assert motion_acceptance(whole, policy)["valid"]
    check_command_peaks(whole["commanded_peaks"], limits, dt)
    assert np.all(measured_excitation(whole, rows, policy) >= 0.9 * limits)

    assessed = assess_motion(rows, dt, policy)
    assert not assessed["acceptance"]["valid"]
    assert any("measured vibration" in r for r in assessed["acceptance"]["reasons"])
    diagnostic = assess_motion(rows, dt, policy, allow_oscillation=True)
    assert diagnostic["acceptance"]["valid"]
    assert not diagnostic["acceptance"]["oscillation_checked"]
    # Diagnostic motion may retain vibration data but cannot ignore tracking.
    rows[-1]["q"][1] += 0.02
    assert not assess_motion(rows, dt, policy, allow_oscillation=True)["acceptance"][
        "valid"
    ]
