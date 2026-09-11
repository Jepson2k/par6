"""Offline calibration analysis against the native model and preview.

Every case is a counterexample the adversarial review of the earlier suite
produced: quiet holds diluting vibration, a burst after the last live
window, elastic torque hiding behind relative improvement, a mislabelled
sweep, a null regressor column, a masked friction deficit. Analytic
signals, not simulator runs — the sim lifecycle lives in test_calibration_sim.py.
"""

import shutil
import struct
import tomllib
from pathlib import Path

import numpy as np
import pytest

from par6._par6 import GravityModel, Preview, calibration_config
from par6.calibration import Patch, capture, write_profile
from par6.calibration.gravity import (
    centers,
    coverage,
    fit_gravity,
    gravity_centers,
    paired_torque,
    sweep_evidence,
    verification_centers,
)
from par6.calibration.metrics import (
    Acceptance,
    assess_motion,
    motion_acceptance,
    motion_metrics,
    steady_samples,
)
from par6.calibration.preflight import check_command_peaks, check_stream
from par6.calibration.report import fingerprint, validate_profile
from par6.calibration.routines import better, trial_quality
from par6.calibration.trajectory import move
from par6.config import config_files

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "config/PAR6.toml"
ASSETS = ROOT / "assets/par6_description"
DT = 0.004


@pytest.fixture(scope="module")
def model():
    return GravityModel(str(CONFIG), str(ASSETS))


@pytest.fixture(scope="module")
def robot():
    return tomllib.loads(CONFIG.read_text())


@pytest.fixture(scope="module")
def policy(robot):
    return Acceptance().describe(robot)


def rows_from(t, q, qd, qc, qdc, *, mode=capture.MODE_STREAM, current=100, ilim=2500):
    flags = (mode << capture.MODE_SHIFT) | capture.FLAG_HOMED | capture.FLAG_ENABLED
    return [
        {
            "tick": i,
            "elapsed_ns": round(s * 1e9),
            "sample_time_ns": round(s * 1e9),
            "flags": flags,
            "q": list(q[i]),
            "qd": list(qd[i]),
            "q_commanded": list(qc[i]),
            "qd_commanded": list(qdc[i]),
            "tau": [0.0] * 6,
            "current_ma": [current] * 6,
            "ilim_ma": [ilim] * 6,
        }
        for i, s in enumerate(t)
    ]


def test_motion_quality_catches_oscillation_saturation_ripple_and_a_late_burst(
    robot, policy
):
    # A 10 Hz oscillation against a still target with saturated current.
    n = 500
    t = np.arange(n) * DT
    wave = 0.02 * np.sin(2 * np.pi * 10 * t)
    zeros = np.zeros((n, 6))
    q = zeros.copy()
    q[:, 0] = wave
    qd = zeros.copy()
    qd[:, 0] = wave * 10
    rows = rows_from(t, q, qd, zeros, zeros, current=100, ilim=100)
    metrics = motion_metrics(rows, DT)
    assert abs(metrics["dominant_hz"][0] - 10) < 0.6
    assert metrics["tracking_peak_deg"][0] > 1
    assert metrics["saturation_s"][0] >= 2
    with pytest.raises(ValueError, match="dropped"):
        motion_metrics(rows[:100] + rows[101:], DT)
    # The spectrum is normalised: twice the recording is not twice the energy.
    long = rows_from(
        np.arange(2 * n) * DT,
        np.zeros((2 * n, 6)),
        np.tile(0.1 * np.sin(2 * np.pi * 10 * np.arange(2 * n) * DT)[:, None], 6),
        np.zeros((2 * n, 6)),
        np.zeros((2 * n, 6)),
        current=0,
        ilim=100,
    )
    short = rows_from(
        t,
        zeros,
        np.tile(0.1 * np.sin(2 * np.pi * 10 * t)[:, None], 6),
        zeros,
        zeros,
        current=0,
        ilim=100,
    )
    assert np.allclose(
        motion_metrics(short, DT)["oscillation_energy"],
        motion_metrics(long, DT)["oscillation_energy"],
        rtol=0.01,
    )

    # Low tracking error must not hide a 50 Hz velocity ripple, whether the
    # drive follows a rippling command or vibrates on its own.
    def recorded(amplitude, follows_ripple=False, n=750):
        t = np.arange(n) * DT
        ripple = amplitude * np.sin(2 * np.pi * 50 * t)
        offset = -amplitude * np.cos(2 * np.pi * 50 * t) / (2 * np.pi * 50)
        q = np.zeros((n, 6))
        q[:, 1] = 0.07 * t + offset
        qc = np.zeros((n, 6))
        qc[:, 1] = 0.07 * t + (offset if follows_ripple else 0)
        qd = np.zeros((n, 6))
        qd[:, 1] = 0.07 + ripple
        qdc = np.zeros((n, 6))
        qdc[:, 1] = 0.07 + (ripple if follows_ripple else 0)
        return rows_from(t, q, qd, qc, qdc)

    for follows in (False, True):
        bad = motion_metrics(recorded(0.075, follows), DT)
        assert max(bad["tracking_peak_deg"]) < 0.5 and max(bad["saturation_s"]) == 0
        verdict = motion_acceptance(bad, policy)
        assert not verdict["valid"]
        assert any("J2 measured vibration" in r for r in verdict["reasons"])
        if follows:
            assert any("command ripple" in r for r in verdict["reasons"])
    assert motion_acceptance(motion_metrics(recorded(0.012), DT), policy)["valid"]

    # One vibrating second then four quiet seconds: whole-trial RMS passes,
    # the travel phase and the windows do not.
    rows = recorded(0.075)[:250]
    for i in range(250, 1250):
        rows.append(
            {
                **rows[249],
                "tick": i,
                "elapsed_ns": round(i * DT * 1e9),
                "sample_time_ns": round(i * DT * 1e9),
                "qd": [0.0] * 6,
                "qd_commanded": [0.0] * 6,
            }
        )
    assert motion_acceptance(motion_metrics(rows, DT), policy)["valid"]
    phases = trial_quality(rows, DT, 1)
    assert motion_acceptance(phases["hold"], policy)["valid"]
    assert not motion_acceptance(phases["motion"], policy)["valid"]
    assert not assess_motion(rows, DT, policy)["acceptance"]["valid"]


def test_feedback_rule_rejects_vibration_between_travel_and_settled_hold(robot, policy):
    quantum = np.array(
        [
            2 * np.pi / (2 ** j["encoder_bits"] * j["gear_ratio"])
            for j in robot["joints"]
        ]
    )
    noise = np.maximum(1e-4, (quantum / DT) ** 2)

    def recorded(begin, end, amplitude):
        # Analytic, derivative-consistent: a 30 Hz burst between `begin` and `end`.
        t = np.arange(round(5.5 / DT) + 1) * DT
        u = np.clip(t / 3, 0, 1)
        command = 0.08 * (10 * u**3 - 15 * u**4 + 6 * u**5)
        speed = 0.08 / 3 * (30 * u**2 - 60 * u**3 + 30 * u**4)
        phase = np.clip((t - begin) / (end - begin), 0, 1)
        active = (t > begin) & (t < end)
        envelope = np.where(active, np.sin(np.pi * phase) ** 2, 0)
        derivative = np.where(
            active, np.pi / (end - begin) * np.sin(2 * np.pi * phase), 0
        )
        w = 2 * np.pi * 30
        carrier = w * (t - begin)
        offset = amplitude / w * envelope * np.sin(carrier)
        ripple = (
            amplitude
            / w
            * (derivative * np.sin(carrier) + envelope * w * np.cos(carrier))
        )
        q = np.zeros((len(t), 6))
        q[:, 1] = command + offset
        qc = np.zeros((len(t), 6))
        qc[:, 1] = command
        qd = np.zeros((len(t), 6))
        qd[:, 1] = speed + ripple
        qdc = np.zeros((len(t), 6))
        qdc[:, 1] = speed
        return rows_from(t, q, qd, qc, qdc)

    def measured(begin, end, amplitude):
        return {
            **trial_quality(recorded(begin, end, amplitude), DT, 1),
            "completed": True,
        }

    baseline = measured(0.3, 2.6, 0.075)
    bad = measured(3.2, 4.4, 0.12)  # ring-down between the travel crop and the hold
    good = measured(0.3, 2.6, 0.005)
    for phase in ("motion", "hold"):
        assert motion_acceptance(bad[phase], policy)["valid"]
    assert not motion_acceptance(bad["window_peaks"], policy)["valid"]
    args = dict(joint=1, policy=policy, noise=noise)
    assert not better([bad] * 6, [baseline] * 6, **args)
    assert not better([bad] * 3, [baseline] * 3, **args)
    assert better([good] * 3, [baseline] * 3, **args)


def test_gravity_fit_generalizes_and_refuses_bad_evidence(model, robot, tmp_path):
    rng = np.random.default_rng(182)
    theta = np.asarray(model.parameters()).ravel()
    delta = np.zeros_like(theta)
    delta[9], delta[13] = 0.015, -0.006
    coulomb = np.array([0.08, 0.25, 0.19, 0.06, 0.05, 0.04])

    def rows(n, bias=0.0):
        out = []
        for _ in range(n):
            q = np.array([0, -1.6, 3.2, 0, -0.3, 3.1]) + rng.uniform(-0.5, 0.5, 6)
            v = rng.choice([-1, 1], 6) * rng.uniform(0.02, 0.1, 6)
            y = np.array(model.regressor(q.tolist()))
            g = np.array(model.gravity(q.tolist()))
            tau = (
                g
                + y @ delta
                + coulomb * np.sign(v)
                + 0.02 * v
                + rng.normal(0, 0.0005, 6)
            )
            tau[1] += bias
            out.append({"q": q.tolist(), "qd": v.tolist(), "tau": tau.tolist()})
        return out

    train, validation = rows(100), rows(60)
    fit = fit_gravity(train, validation, model)
    assert fit.valid, fit.reasons
    assert max(fit.validation_after_nm) < 0.005
    assert np.allclose(fit.coulomb_nm, coulomb, atol=0.005)
    assert max(fit.validation_before_nm) > 0.05
    # The same evidence repeated a hundred times must not promote a numerical
    # zero column into a billion-coefficient parameter.
    repeated = fit_gravity(train * 100, validation, model)
    assert repeated.valid, repeated.reasons
    assert np.allclose(repeated.correction, fit.correction, atol=1e-3)
    # A biased hold-out is refused even though the fit itself is fine.
    assert not fit_gravity(train, rows(60, bias=0.3), model).valid

    # Periodic elastic torque: relative improvement is large, the absolute
    # held-out residual is not small, so nothing is staged.
    q = np.array([0, -1.6, 3.2, 0, -0.3, 3.1]) + rng.uniform(-0.5, 0.5, (500, 6))
    v = rng.choice([-1, 1], (500, 6)) * rng.uniform(0.02, 0.1, (500, 6))
    y = np.array([model.regressor(p.tolist()) for p in q])
    g = np.array([model.gravity(p.tolist()) for p in q])
    truth = np.zeros(24)
    truth[9], truth[13] = 0.04, -0.03
    tau = g + y @ truth + 0.3 * np.sign(v) + v
    elastic = np.zeros_like(tau)
    elastic[:, 2] = 0.3 * np.sin(5 * q[:, 2])
    elastic[:, 5] = 0.18 * np.sin(5 * q[:, 5])
    samples = [
        dict(q=p.tolist(), qd=s.tolist(), tau=t.tolist())
        for p, s, t in zip(q, v, tau + elastic)
    ]
    bad = fit_gravity(samples[:300], samples[300:], model)
    assert not bad.valid
    assert bad.validation_after_nm[2] > 0.1 and bad.validation_after_nm[5] > 0.1
    assert np.linalg.norm(bad.validation_after_nm) < 0.7 * np.linalg.norm(
        bad.validation_before_nm
    )
    bundle = config_files(CONFIG)
    report = {**bad.to_dict(), "baseline_fingerprint": fingerprint(bundle)}
    with pytest.raises(ValueError, match="validated"):
        write_profile(
            tmp_path / "rejected", bundle, robot, report, Patch(gravity=bad.correction)
        )
    assert not (tmp_path / "rejected").exists()
    for limit in (0, -1, np.nan, np.inf, [0.05] * 5):
        with pytest.raises(ValueError, match="limit"):
            fit_gravity(
                samples[:300], samples[300:], model, max_validation_rms_nm=limit
            )

    # Two sweeps labelled J6 that only moved J5: the fit and both coverage
    # checks still pass (other groups supply J6), the per-sweep evidence does not.
    window = np.array(
        [
            [j["limits"]["soft_min_rad"], j["limits"]["soft_max_rad"]]
            for j in robot["joints"]
        ]
    )
    poses = gravity_centers([0, -1.72, 3.34, 0, -0.55, np.pi])
    small = np.zeros_like(theta)
    small[9], small[13] = 0.006, -0.002
    labelled, required = [], set()
    for group, center in enumerate(poses):
        for joint in range(1, 6):
            for direction in (-1, 1):
                required.add((group, joint, direction))
                actual = 4 if (group, joint) == (5, 5) else joint
                for offset in np.linspace(-0.17, 0.17, 37):
                    p, s = center.copy(), np.zeros(6)
                    p[actual] += direction * offset
                    s[actual] = direction * 0.04
                    torque = (
                        np.asarray(model.gravity(p.tolist()))
                        + np.asarray(model.regressor(p.tolist())) @ small
                        + 0.05 * np.sign(s)
                        + 0.01 * s
                    )
                    labelled.append(
                        {
                            "group": group,
                            "joint": joint,
                            "direction": direction,
                            "held_out": group % 3 == 2,
                            "q": p.tolist(),
                            "qd": s.tolist(),
                            "tau": torque.tolist(),
                        }
                    )
    train = [r for r in labelled if not r["held_out"]]
    validation = [r for r in labelled if r["held_out"]]
    for part in (train, validation):
        assert coverage(
            model, [r["q"] for r in part], [r["joint"] for r in part], window
        )["valid"]
    assert fit_gravity(train, validation, model).valid
    evidence = sweep_evidence(labelled, required)
    assert not evidence["valid"]
    refused = {
        (r["group"], r["joint"], r["direction"])
        for r in evidence["measurements"]
        if not r["valid"]
    }
    assert refused == {(5, 5, -1), (5, 5, 1)}
    assert sweep_evidence(labelled, required - refused)["valid"]


def test_paired_torque_and_coverage_detect_masked_and_wrist_errors(model, robot):
    window = [
        [j["limits"]["soft_min_rad"], j["limits"]["soft_max_rad"]]
        for j in robot["joints"]
    ]
    start = np.deg2rad([0, -90, 170, 0, -20, 180])
    old = centers(start)[:3]
    assert not coverage(
        model,
        [p for p in old for _ in range(1, 6)],
        list(range(1, 6)) * len(old),
        window,
    )["valid"]
    poses = verification_centers(start)
    assert coverage(
        model,
        [p for p in poses for _ in range(1, 6)],
        list(range(1, 6)) * len(poses),
        window,
    )["valid"]
    samples = []
    for group, center in enumerate(poses):
        for j in range(1, 6):
            for direction in (-1, 1):
                for offset in np.linspace(-0.18, 0.18, 81):
                    q = center.copy()
                    q[j] += direction * offset
                    v = np.zeros(6)
                    v[j] = direction * 0.04
                    tau = np.asarray(model.gravity(q.tolist()))
                    tau[j] += direction * (0.30 + 0.02 * 0.04)
                    samples.append(
                        {
                            "q": q.tolist(),
                            "qd": v.tolist(),
                            "tau": tau.tolist(),
                            "group": group,
                            "joint": j,
                            "direction": direction,
                        }
                    )
    nominal = paired_torque(samples, model)
    assert nominal["valid"], nominal["reasons"]
    assert max(m["rms_nm"] for m in nominal["measurements"]) < 1e-8
    assert not paired_torque(samples, model, scale=[1, 1, 0.9, 1, 1, 1])["valid"]
    assert not paired_torque([r for r in samples if r["joint"] != 2], model)["valid"]
    # A correction invisible at the old static poses but 0.4 Nm at the new wrist poses.
    old_y = np.concatenate([model.regressor(p.tolist()) for p in old])
    _, sv, vt = np.linalg.svd(old_y, full_matrices=True)
    null = vt[(sv > 1e-8).sum() :].T
    new_y = np.concatenate([model.regressor(p.tolist()) for p in poses])
    _, _, directions = np.linalg.svd(new_y @ null, full_matrices=False)
    delta = null @ directions[0]
    delta *= 0.4 / np.max(np.abs(new_y @ delta))
    assert np.max(np.abs(old_y @ delta)) < 1e-8
    assert not paired_torque(samples, model, correction=delta)["valid"]
    short = [
        r
        for r in samples
        if abs(r["q"][r["joint"]] - poses[r["group"]][r["joint"]]) < 0.03
    ]
    assert not paired_torque(short, model)["valid"]


def test_stream_preflight_and_staged_patch_provenance(robot, tmp_path):
    limits = np.array(
        [
            [
                j["limits"]["exec"][k]
                for k in ("velocity_rad_s", "acceleration_rad_s2", "jerk_rad_s3")
            ]
            for j in robot["joints"]
        ]
    )
    package_dir = str(ASSETS / "URDF")
    baseline = Preview(config=str(CONFIG), assets=str(ASSETS), package_dir=package_dir)
    start = np.array([0, -1.85, 2.85, 0, -0.55, np.pi])
    end = start.copy()
    end[1] += 0.24
    t, q = move(start, end, [[0.07, 0.12, 0.5]] * 6)
    # Gentle 50 Hz targets through the shipped STREAM caps come out as pulses.
    with pytest.raises(ValueError, match="J2 acceleration"):
        check_stream(baseline, t, q, limits)
    shutil.copytree(CONFIG.parent / "grippers", tmp_path / "grippers")
    config = tmp_path / "PAR6.toml"
    config.write_text(
        calibration_config(CONFIG.read_text(), stream_limits=limits.tolist())
    )
    candidate = Preview(config=str(config), assets=str(ASSETS), package_dir=package_dir)
    for joint in range(6):
        end = start.copy()
        end[joint] += 0.24
        t, q = move(start, end, [[0.07, 0.12, 0.5]] * 6)
        predicted, peaks = check_stream(candidate, t, q, limits)
        assert np.max(np.abs(predicted[-1] - end)) < 1e-6
        if joint == 1:
            assert peaks[joint][1] < 0.1
    with pytest.raises(ValueError, match="J3 jerk"):
        check_command_peaks(
            np.array([[0.1, 0.2, 0.6]] * 2 + [[0.1, 0.2, 1.2]] + [[0.1, 0.2, 0.6]] * 3),
            np.array([[0.1, 0.2, 0.6]] * 6),
            0.004,
        )

    # A patch only carries what was measured; the staged config is validated
    # natively, bound to the loaded config, and refuses to be edited afterwards.
    bundle = config_files(CONFIG)
    patch = Patch(
        exec_limits=[[0.2, 0.4, 1.2]] * 6, stream_limits=[[0.2, 0.4, 1.2]] * 6
    )
    patch.feedback_gains[2] = [5.0, 0.012, 0.0012]
    report = {"valid": True, "baseline_fingerprint": fingerprint(bundle)}
    path = write_profile(tmp_path / "profile", bundle, robot, report, patch)
    staged = tomllib.loads(path.read_text())
    assert staged["joints"][2]["gains"]["kpv"] == 0.012
    assert staged["joints"][2]["gains"]["kpiq"] == robot["joints"][2]["gains"]["kpiq"]
    assert staged["joints"][0]["gains"] == robot["joints"][0]["gains"]
    assert staged["joints"][0]["limits"]["exec"]["velocity_rad_s"] == 0.2
    assert "gravity_correction" not in patch.toml() and "kpv = 0.012" in patch.toml()
    validate_profile(path)
    path.write_text(path.read_text() + "\n# edited\n")
    with pytest.raises(ValueError, match="changed"):
        validate_profile(path)
    with pytest.raises(ValueError, match="different"):
        write_profile(
            tmp_path / "other",
            bundle,
            robot,
            {**report, "baseline_fingerprint": "x"},
            patch,
        )
    with pytest.raises(ValueError, match="Nothing was measured"):
        write_profile(tmp_path / "empty", bundle, robot, report, Patch())
    for value in (0, -1, np.nan, np.inf, 1e9):
        with pytest.raises(ValueError):
            Patch(exec_limits=[[value, 0.4, 1.2]] * 6).apply(
                robot, bundle["robot_toml"]
            )
    for bad in (0, -1, float("nan"), 100):
        with pytest.raises(ValueError):
            Patch(feedback_gains={2: [5.0, bad, 0.001]}).apply(
                robot, bundle["robot_toml"]
            )


def test_capture_reader_validates_recording_and_isolates_motion(tmp_path):
    dt = 0.004
    n = 400
    flags = (capture.MODE_IDLE << 8) | capture.FLAG_HOMED | capture.FLAG_ENABLED
    values = np.zeros((n, len(capture.FIELDS), 6))
    values[:, capture.FIELDS.index("ilim_ma"), :] = 2000
    array = np.zeros(n, dtype=capture.DTYPE)
    array["tick"] = np.arange(n)
    array["elapsed_ns"] = (np.arange(n) * dt * 1e9).round()
    array["flags"] = flags
    array["flags"][100:300] = (
        (capture.MODE_STREAM << 8) | capture.FLAG_HOMED | capture.FLAG_ENABLED
    )
    array["values"] = values
    header = (
        b"PAR6CAP2"
        + struct.pack("<d", dt)
        + b"f" * 64
        + struct.pack("<Q", 4242)
        + struct.pack("<d", 1.0e9)
    )
    path = tmp_path / "capture.bin"
    path.write_bytes(
        header + array.tobytes() + b"\x00" * 7
    )  # a partial trailing record
    assert capture.length(path) == n
    read_dt, rows = capture.read_capture(path, 90, 310)
    assert read_dt == dt and len(rows) == 220
    active = capture.active_rows(rows, (capture.MODE_STREAM,))
    assert [r["tick"] for r in active][0] == 100 and active[-1]["tick"] == 299
    with pytest.raises(ValueError, match="no controlled motion"):
        capture.active_rows(rows[:5])
    faulted = [
        dict(r, flags=r["flags"] | capture.FLAG_FAULT) if r["tick"] == 200 else r
        for r in rows
    ]
    with pytest.raises(ValueError, match="fault"):
        capture.active_rows(faulted)
    interrupted = [dict(r, flags=flags) if r["tick"] == 200 else r for r in rows]
    with pytest.raises(ValueError, match="left motion mode"):
        capture.active_rows(interrupted, (capture.MODE_STREAM,))
    sim = capture.measurement_rows(active, dt, simulator=True)
    assert sim[0]["sample_time_ns"] == round(100 * dt * 1e9)
    hw = capture.measurement_rows(active, dt, simulator=False)
    assert hw[0]["sample_time_ns"] == active[0]["elapsed_ns"]
    identity = capture.reference_identity(path)
    assert identity["pid"] == 4242 and identity["reference_tick"] is None
    with pytest.raises(RuntimeError, match="another runtime"):
        capture.assert_live(
            path,
            "f" * 64,
            expected_identity={
                "pid": 1,
                "started": 1.0e9,
                "dt": dt,
                "fingerprint": "f" * 64,
            },
        )
    with pytest.raises(RuntimeError, match="differs"):
        capture.assert_live(path, "g" * 64)
    with pytest.raises(RuntimeError, match="stopped or is stale"):
        capture.assert_live(path, "f" * 64)
    assert steady_samples(rows, dt) == []
