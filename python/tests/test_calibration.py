"""Calibration must generalize, obey derivative bounds, and refuse bad evidence."""

import shutil
import tomllib
from pathlib import Path

import numpy as np
import pytest

from par6._par6 import GravityModel, Preview, calibration_config
from par6.calibration.analysis import fit_gravity, motion_metrics
from par6.calibration.profiles import export_profile, fingerprint, validate_profile
from par6.calibration.trajectory import envelope_move, move
from par6.config import config_files

ROOT = Path(__file__).resolve().parents[2]


def test_stimulus_respects_all_derivative_limits_and_reverses():
    limits = np.array([[0.2, 0.4, 1.2]] * 6)
    for offset in [0.001, 0.1, -0.8]:
        t, q = move(np.zeros(6), np.array([offset] * 6), limits, 0.001)
        v = np.gradient(q, t, axis=0)
        a = np.gradient(v, t, axis=0)
        j = np.gradient(a, t, axis=0)
        assert np.max(np.abs(v)) <= 0.2 * 1.001
        assert np.max(np.abs(a)) <= 0.4 * 1.001
        assert np.max(np.abs(j[4:-4])) <= 1.2 * 1.001
        assert np.allclose(q[-1], offset)
    for value in [0, -1, np.nan, np.inf]:
        bad = limits.copy()
        bad[2, 1] = value
        with pytest.raises(ValueError):
            move(np.zeros(6), np.ones(6), bad)


def test_observable_fit_predicts_unseen_poses_and_rejects_biased_validation():
    model = GravityModel(
        str(ROOT / "config/PAR6.toml"), str(ROOT / "assets/par6_description")
    )
    rng = np.random.default_rng(182)
    # A physical link first moment changes, not an arbitrary torque offset.
    theta = np.asarray(model.parameters()).ravel()
    delta = np.zeros_like(theta)
    delta[9] = 0.015
    delta[13] = -0.006
    coulomb = np.array([0.08, 0.25, 0.19, 0.06, 0.05, 0.04])
    viscous = np.array([0.02] * 6)

    def rows(n, bias=0):
        result = []
        for _ in range(n):
            q = np.array([0, -1.6, 3.2, 0, -0.3, 3.1]) + rng.uniform(-0.5, 0.5, 6)
            v = rng.choice([-1, 1], 6) * rng.uniform(0.02, 0.1, 6)
            y = np.array(model.regressor(q.tolist()))
            g = np.array(model.gravity(q.tolist()))
            tau = (
                g
                + y @ delta
                + coulomb * np.sign(v)
                + viscous * v
                + rng.normal(0, 0.0005, 6)
            )
            tau[1] += bias
            result.append({"q": q.tolist(), "qd": v.tolist(), "tau": tau.tolist()})
        return result

    train = rows(100)
    validation = rows(60)
    fit = fit_gravity(train, validation, model)
    assert fit.valid, fit.reasons
    assert max(fit.validation_after_nm) < 0.005
    assert np.allclose(fit.coulomb_nm, coulomb, atol=0.005)
    repeated = fit_gravity(train, validation, model, baseline_correction=delta.tolist())
    assert max(repeated.validation_before_nm) < 0.005
    assert np.allclose(repeated.validation_after_nm, fit.validation_after_nm)
    assert max(fit.validation_before_nm) > 0.05
    # Repeating identical evidence cannot change the fitted physical model.
    # Large native recordings used to amplify a floating-point-zero regressor
    # column above an absolute normalization threshold and assign it billions
    # of kg m, despite unchanged measured torque.
    long_fit = fit_gravity(train * 100, validation, model)
    assert long_fit.valid, long_fit.reasons
    assert np.allclose(long_fit.correction, fit.correction, atol=1e-8)
    scaled = fit_gravity(
        train,
        validation,
        model,
        baseline_correction=delta.tolist(),
        baseline_scale=[1, 1, 1.3, 1, 1, 1],
    )
    assert scaled.validation_before_nm[2] > repeated.validation_before_nm[2] + 0.1
    assert np.allclose(scaled.validation_after_nm, fit.validation_after_nm)
    assert not fit_gravity(train, rows(60, 1.0), model).valid
    static = [{**r, "qd": [0] * 6} for r in train]
    with pytest.raises(ValueError, match="moving"):
        fit_gravity(static, validation, model)


def test_profile_rejects_wrong_provenance_and_invalid_native_limits(tmp_path):
    bundle = config_files(ROOT / "config/PAR6.toml")
    report = {"valid": False, "baseline_fingerprint": fingerprint(bundle)}
    with pytest.raises(ValueError, match="validated"):
        export_profile(tmp_path, bundle, report, gravity=[0] * 24)
    report["valid"] = True
    report["baseline_fingerprint"] = "wrong"
    with pytest.raises(ValueError, match="different"):
        export_profile(tmp_path, bundle, report)
    for value in [0, -1, np.nan, np.inf, 1e9]:
        with pytest.raises(ValueError):
            calibration_config(bundle["robot_toml"], None, [[value, 0.4, 1.2]] * 6)
    report["baseline_fingerprint"] = fingerprint(bundle)
    path = export_profile(tmp_path, bundle, report, stream_limits=[[0.2, 0.4, 1.2]] * 6)
    assert path.is_file()
    validate_profile(path)
    path.write_text(path.read_text() + "\n# modified after fitting\n")
    with pytest.raises(ValueError, match="changed"):
        validate_profile(path)
    assert (path.stat().st_mode & 0o777) == 0o600
    assert (tmp_path / "rollback" / bundle["robot_filename"]).read_text() == bundle[
        "robot_toml"
    ]


def test_capture_metrics_detect_oscillation_saturation_and_lost_ticks():
    rows = []
    for i in range(500):
        wave = float(0.02 * np.sin(i * 0.004 * 2 * np.pi * 10))
        rows.append(
            {
                "tick": i,
                "flags": 2,
                "q": [wave] + [0] * 5,
                "qd": [wave * 10] + [0] * 5,
                "q_commanded": [0] * 6,
                "qd_commanded": [0] * 6,
                "current_ma": [100] * 6,
                "ilim_ma": [100] * 6,
            }
        )
    metrics = motion_metrics(rows, 0.004)
    assert abs(metrics["dominant_hz"][0] - 10) < 0.6
    assert metrics["tracking_peak_deg"][0] > 1
    assert metrics["saturation_s"][0] >= 2
    with pytest.raises(ValueError, match="dropped"):
        motion_metrics(rows[:100] + rows[101:], 0.004)


def test_envelope_exercises_the_requested_derivative_or_reports_no_room():
    center = np.zeros(6)
    limits = np.array([[0.4, 0.8, 3.0]] * 6)
    for axis, value in enumerate([0.2, 0.3, 1.0]):
        t, q = envelope_move(center, 2, axis, value, limits, [[-1, 1]] * 6, dt=0.001)
        derivative = q
        for _ in range(axis + 1):
            derivative = np.gradient(derivative, t, axis=0)
        assert np.max(np.abs(derivative[4:-4, 2])) >= 0.9 * value
        assert np.max(np.abs(q[:, 2])) < 1
    with pytest.raises(ValueError, match="cannot excite"):
        envelope_move(center, 2, 0, 10, limits, [[-0.02, 0.02]] * 6)


def test_motion_spectrum_is_not_biased_by_recording_duration():
    def signal(n):
        return [
            {
                "tick": i,
                "flags": 2,
                "q": [0] * 6,
                "q_commanded": [0] * 6,
                "qd": [float(0.1 * np.sin(i * 0.004 * 2 * np.pi * 10))] * 6,
                "qd_commanded": [0] * 6,
                "current_ma": [0] * 6,
                "ilim_ma": [100] * 6,
            }
            for i in range(n)
        ]

    a = motion_metrics(signal(500), 0.004)["oscillation_energy"]
    b = motion_metrics(signal(1000), 0.004)["oscillation_energy"]
    assert np.allclose(a, b, rtol=0.01)


def test_feedback_profile_preserves_electrical_calibration_and_limits():
    source = (ROOT / "config/PAR6.toml").read_text()
    original = tomllib.loads(source)
    gains = [[j["gains"][k] for k in ("kpp", "kpv", "kiv")] for j in original["joints"]]
    gains[2][1] *= 0.8
    gains[2][2] *= 0.8
    result = tomllib.loads(calibration_config(source, feedback_gains=gains))
    assert result["joints"][2]["gains"]["kpv"] < original["joints"][2]["gains"]["kpv"]
    for before, after in zip(original["joints"], result["joints"]):
        for key in ("kpiq", "kiiq", "kp", "kd"):
            assert after["gains"][key] == before["gains"][key]
        assert after["ilim_ma"] == before["ilim_ma"]
        assert after["kt_nm_a"] == before["kt_nm_a"]
    for bad in [0, -1, float("nan"), float("inf"), 100]:
        gains[2][1] = bad
        with pytest.raises(ValueError):
            calibration_config(source, feedback_gains=gains)


def test_native_stream_preflight_rejects_pulses_from_gentle_targets(tmp_path):
    from par6.calibration.preflight import check_stream

    original = ROOT / "config/PAR6.toml"
    robot = tomllib.loads(original.read_text())
    limits = np.array(
        [
            [
                j["limits"]["exec"][k]
                for k in ("velocity_rad_s", "acceleration_rad_s2", "jerk_rad_s3")
            ]
            for j in robot["joints"]
        ]
    )
    assets = str(ROOT / "assets/par6_description")
    baseline = Preview(
        config=str(original),
        assets=assets,
        package_dir=str(ROOT / "assets/par6_description/URDF"),
    )
    start = np.array([0, -1.85, 2.85, 0, -0.55, np.pi])
    end = start.copy()
    end[1] += 0.24
    t, q = move(start, end, [[0.07, 0.12, 0.5]] * 6)
    with pytest.raises(ValueError, match="J2 acceleration"):
        check_stream(baseline, t, q, limits)
    shutil.copytree(original.parent / "grippers", tmp_path / "grippers")
    config = tmp_path / "PAR6.toml"
    config.write_text(
        calibration_config(original.read_text(), stream_limits=limits.tolist())
    )
    candidate = Preview(
        config=str(config),
        assets=assets,
        package_dir=str(ROOT / "assets/par6_description/URDF"),
    )
    for joint in range(6):
        end = start.copy()
        end[joint] += 0.24
        t, q = move(start, end, [[0.07, 0.12, 0.5]] * 6)
        predicted, peaks = check_stream(candidate, t, q, limits)
        assert np.max(np.abs(predicted[-1] - end)) < 1e-6
        if joint == 1:
            assert peaks[joint][1] < 0.1


def test_low_tracking_error_does_not_hide_moving_vibration_or_command_ripple():
    from par6.calibration.validation import Acceptance, motion_acceptance

    robot = tomllib.loads((ROOT / "config/PAR6.toml").read_text())
    policy = Acceptance().describe(robot)

    def recorded(amplitude, follows_ripple=False):
        rows = []
        for i in range(750):
            t = i * 0.004
            ripple = amplitude * np.sin(2 * np.pi * 50 * t)
            offset = -amplitude * np.cos(2 * np.pi * 50 * t) / (2 * np.pi * 50)
            rows.append(
                {
                    "tick": i,
                    "elapsed_ns": round(t * 1e9),
                    "flags": (5 << 8) | 22,
                    "q": [0, 0.07 * t + offset, 0, 0, 0, 0],
                    "q_commanded": [
                        0,
                        0.07 * t + (offset if follows_ripple else 0),
                        0,
                        0,
                        0,
                        0,
                    ],
                    "qd": [0, 0.07 + ripple, 0, 0, 0, 0],
                    "qd_commanded": [
                        0,
                        0.07 + (ripple if follows_ripple else 0),
                        0,
                        0,
                        0,
                        0,
                    ],
                    "current_ma": [100] * 6,
                    "ilim_ma": [2500] * 6,
                }
            )
        return rows

    for follows_ripple in (False, True):
        bad = motion_metrics(recorded(0.075, follows_ripple), 0.004)
        # Both failures satisfied the old tracking/current-only gate.
        assert max(bad["tracking_peak_deg"]) < 0.5
        assert max(bad["saturation_s"]) == 0
        verdict = motion_acceptance(bad, policy)
        assert not verdict["valid"]
        assert any("J2 measured vibration" in reason for reason in verdict["reasons"])
        if follows_ripple:
            assert max(bad["velocity_residual_rms"]) == 0
            assert any("command ripple" in reason for reason in verdict["reasons"])
    assert motion_acceptance(motion_metrics(recorded(0.012), 0.004), policy)["valid"]

    from par6.calibration.feedback import feedback_metrics

    # A one-second vibrating move followed by a four-second quiet hold.
    # Whole-trial RMS passed this shape of actual physical feedback experiment.
    rows = recorded(0.075)[:250]
    for i in range(250, 1250):
        rows.append(
            {
                **rows[249],
                "tick": i,
                "elapsed_ns": round(i * 0.004 * 1e9),
                "qd": [0.0] * 6,
                "qd_commanded": [0.0] * 6,
            }
        )
    assert motion_acceptance(motion_metrics(rows, 0.004), policy)["valid"]
    phases = feedback_metrics(rows, 0.004, 1)
    assert motion_acceptance(phases["hold"], policy)["valid"]
    assert not motion_acceptance(phases["motion"], policy)["valid"]


def test_gravity_validation_rejects_slow_droop_short_holds_and_feedback_masking():
    from par6.calibration.validation import Acceptance, gravity_hold_metrics

    robot = tomllib.loads((ROOT / "config/PAR6.toml").read_text())
    policy = Acceptance().describe(robot)

    def hold(drift):
        rows = []
        for i in range(5001):
            q = [0, 0, np.deg2rad(drift * i * 0.004), 0, 0, 0]
            rows.append(
                {
                    "tick": i,
                    "elapsed_ns": i * 4_000_000,
                    "flags": (1 << 8) | 22,
                    "q": q,
                    "q_commanded": q,
                    "qd": [0] * 6,
                    "qd_commanded": [0] * 6,
                    "current_ma": [100] * 6,
                    "ilim_ma": [2500] * 6,
                }
            )
        return rows

    bad = hold(-0.02)  # Below the old rest-speed limit, even with ideal speed readback.
    verdict = gravity_hold_metrics(bad, 0.004, policy)
    assert not verdict["valid"]
    assert any("J3 gravity-only drift" in reason for reason in verdict["reasons"])
    good = hold(0.0002)
    assert gravity_hold_metrics(good, 0.004, policy)["valid"]
    assert not gravity_hold_metrics(good[:76], 0.004, policy)["valid"]
    for flags in ((6 << 8) | 22, (1 << 8) | 6):
        masked = [{**r, "flags": flags} for r in good]
        with pytest.raises(ValueError, match="gravity-only"):
            gravity_hold_metrics(masked, 0.004, policy)
    with pytest.raises(ValueError, match="dropped"):
        gravity_hold_metrics(good[:100] + good[101:], 0.004, policy)


def test_gravity_export_replaces_manual_trim_and_preserves_it_for_other_profiles():
    source = (
        "gravity_scale = [1, 1, 1.3, 1, 1, 1]\n"
        + (ROOT / "config/PAR6.toml").read_text()
    )
    changed = tomllib.loads(calibration_config(source, gravity=[0] * 24))
    assert changed["gravity_scale"] == [1] * 6
    untouched = tomllib.loads(
        calibration_config(source, stream_limits=[[0.2, 0.4, 1.2]] * 6)
    )
    assert untouched["gravity_scale"][2] == 1.3


def test_fit_rejects_large_remaining_error_even_after_relative_improvement(tmp_path):
    # Adversarial case: a periodic elastic torque cannot be explained by the
    # gravity/friction model. Relative improvement alone previously exported it.
    model = GravityModel(
        str(ROOT / "config/PAR6.toml"), str(ROOT / "assets/par6_description")
    )
    rng = np.random.default_rng(14)
    truth = np.zeros(24)
    truth[9], truth[13] = 0.04, -0.03
    q = np.array([0, -1.6, 3.2, 0, -0.3, 3.1]) + rng.uniform(-0.5, 0.5, (500, 6))
    v = rng.choice([-1, 1], (500, 6)) * rng.uniform(0.02, 0.1, (500, 6))
    y = np.array([model.regressor(p.tolist()) for p in q])
    g = np.array([model.gravity(p.tolist()) for p in q])
    tau = g + y @ truth + 0.3 * np.sign(v) + v
    elastic = np.zeros_like(tau)
    elastic[:, 2] = 0.3 * np.sin(5 * q[:, 2])
    elastic[:, 5] = 0.18 * np.sin(5 * q[:, 5])
    for disturbance in (np.zeros_like(tau), elastic):
        rows = [
            dict(q=p.tolist(), qd=speed.tolist(), tau=torque.tolist())
            for p, speed, torque in zip(q, v, tau + disturbance)
        ]
        fit = fit_gravity(rows[:300], rows[300:], model)
        if not disturbance.any():
            assert fit.valid, fit.reasons
            continue
        assert not fit.valid
        assert fit.validation_after_nm[2] > 0.19
        assert fit.validation_after_nm[5] > 0.11
        assert any("J3: held-out torque error" in r for r in fit.reasons)
        assert any("J6: held-out torque error" in r for r in fit.reasons)
        assert np.linalg.norm(fit.validation_after_nm) < 0.7 * np.linalg.norm(
            fit.validation_before_nm
        )
        bundle = config_files(ROOT / "config/PAR6.toml")
        report = {**fit.to_dict(), "baseline_fingerprint": fingerprint(bundle)}
        with pytest.raises(ValueError, match="validated"):
            export_profile(
                tmp_path / "rejected", bundle, report, gravity=fit.correction
            )
        assert not (tmp_path / "rejected").exists()
    for bad in (0, -1, np.nan, np.inf, [0.05] * 5):
        with pytest.raises(ValueError, match="limit"):
            fit_gravity(rows[:300], rows[300:], model, max_validation_rms_nm=bad)


def test_moving_torque_validation_detects_friction_masked_and_wrist_errors():
    from par6.calibration.gravity_validation import coverage, paired_torque
    from par6.calibration.routines import centers, verification_centers

    model = GravityModel(
        str(ROOT / "config/PAR6.toml"), str(ROOT / "assets/par6_description")
    )
    robot = tomllib.loads((ROOT / "config/PAR6.toml").read_text())
    window = [
        [j["limits"]["soft_min_rad"], j["limits"]["soft_max_rad"]]
        for j in robot["joints"]
    ]
    start = np.deg2rad([0, -90, 170, 0, -20, 180])
    old = centers(start)[:3]
    old_y = np.concatenate([model.regressor(p.tolist()) for p in old])
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
    # Exact independent observations from the native model, symmetric moving
    # friction and two opposite sweeps. These are analytic data, not a plant run.
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
    # Choose the old static verifier's blind direction with maximum effect at
    # the new wrist configurations, rather than prescribing a favorable error.
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


def test_feedback_rejects_vibration_between_travel_and_settled_hold():
    from par6.calibration.feedback import feedback_acceptable, feedback_metrics
    from par6.calibration.validation import Acceptance, motion_acceptance

    robot = tomllib.loads((ROOT / "config/PAR6.toml").read_text())
    policy = Acceptance().describe(robot)
    dt = 0.004
    quantum = np.array(
        [
            2 * np.pi / (2 ** j["encoder_bits"] * j["gear_ratio"])
            for j in robot["joints"]
        ]
    )
    noise = np.maximum(1e-4, (quantum / dt) ** 2)

    def recorded(begin, end, amplitude):
        # Exact analytic position/velocity derivatives; this tests the signal
        # acceptance rule, not a claim about the simulator's torque response.
        t = np.arange(round(5.5 / dt) + 1) * dt
        u = np.clip(t / 3, 0, 1)
        command = 0.08 * (10 * u**3 - 15 * u**4 + 6 * u**5)
        speed = 0.08 / 3 * (30 * u**2 - 60 * u**3 + 30 * u**4)
        phase = np.clip((t - begin) / (end - begin), 0, 1)
        active = (t > begin) & (t < end)
        envelope = np.where(active, np.sin(np.pi * phase) ** 2, 0)
        derivative = np.where(
            active, np.pi / (end - begin) * np.sin(2 * np.pi * phase), 0
        )
        frequency = 2 * np.pi * 30
        carrier = frequency * (t - begin)
        offset = amplitude / frequency * envelope * np.sin(carrier)
        ripple = (
            amplitude
            / frequency
            * (derivative * np.sin(carrier) + envelope * frequency * np.cos(carrier))
        )
        return [
            {
                "tick": i,
                "elapsed_ns": round(seconds * 1e9),
                "flags": (5 << 8) | 22,
                "q": [0, command[i] + offset[i], 0, 0, 0, 0],
                "q_commanded": [0, command[i], 0, 0, 0, 0],
                "qd": [0, speed[i] + ripple[i], 0, 0, 0, 0],
                "qd_commanded": [0, speed[i], 0, 0, 0, 0],
                "current_ma": [100] * 6,
                "ilim_ma": [2500] * 6,
            }
            for i, seconds in enumerate(t)
        ]

    def measured(begin, end, amplitude):
        return {
            **feedback_metrics(recorded(begin, end, amplitude), dt, 1),
            "completed": True,
        }

    baseline = measured(0.3, 2.6, 0.075)
    bad = measured(3.2, 4.4, 0.12)
    good = measured(0.3, 2.6, 0.005)
    for phase in ("motion", "hold", "whole_trial"):
        assert motion_acceptance(bad[phase], policy)["valid"]
    assert not motion_acceptance(bad["window_peaks"], policy)["valid"]
    args = dict(joint=1, policy=policy, noise_energy=noise)
    assert not feedback_acceptable([bad] * 6, [baseline] * 6, **args)
    assert not feedback_acceptable([bad] * 3, [baseline] * 3, **args)
    assert feedback_acceptable([good] * 3, [baseline] * 3, **args)


def test_gravity_fit_cannot_substitute_another_joints_motion():
    """Analytic moving torques isolate evidence admission from motion control."""
    from types import SimpleNamespace

    from par6.calibration.gravity_validation import sweep_evidence
    from par6.calibration.routines import _fitting_coverage, gravity_centers

    config = ROOT / "config/PAR6.toml"
    robot = tomllib.loads(config.read_text())
    model = GravityModel(str(config), str(ROOT / "assets/par6_description"))
    context = SimpleNamespace(
        model=model,
        window=np.array(
            [
                [j["limits"]["soft_min_rad"], j["limits"]["soft_max_rad"]]
                for j in robot["joints"]
            ]
        ),
    )
    delta = np.zeros_like(np.asarray(model.parameters()).ravel())
    delta[9], delta[13] = 0.006, -0.002
    poses = gravity_centers([0, -1.72, 3.34, 0, -0.55, np.pi])
    train, validation, required = [], [], set()
    for group, center in enumerate(poses):
        for joint in range(1, 6):
            for direction in (-1, 1):
                required.add((group, joint, direction))
                actual = 4 if (group, joint) == (5, 5) else joint
                for offset in np.linspace(-0.17, 0.17, 37):
                    q, v = center.copy(), np.zeros(6)
                    q[actual] += direction * offset
                    v[actual] = direction * 0.04
                    tau = (
                        np.asarray(model.gravity(q.tolist()))
                        + np.asarray(model.regressor(q.tolist())) @ delta
                        + 0.05 * np.sign(v)
                        + 0.01 * v
                    )
                    (validation if group % 3 == 2 else train).append(
                        {
                            "group": group,
                            "center_group": group,
                            "joint": joint,
                            "direction": direction,
                            "q": q.tolist(),
                            "qd": v.tolist(),
                            "tau": tau.tolist(),
                        }
                    )
    # Global fit quality and rank cannot establish each labelled sweep moved
    # its requested joint: other groups supply the missing J6 fit observations.
    assert _fitting_coverage(context, train)["valid"]
    assert _fitting_coverage(context, validation)["valid"]
    assert fit_gravity(train, validation, model).valid
    result = sweep_evidence(train + validation, required)
    assert not result["valid"]
    refused = {
        (r["group"], r["joint"], r["direction"])
        for r in result["measurements"]
        if not r["valid"]
    }
    assert refused == {(5, 5, -1), (5, 5, 1)}
    assert sweep_evidence(train + validation, required - refused)["valid"]
    duplicates = [
        {**r, "group": r["group"] + batch * 6}
        for batch in range(2)
        for r in train
        if r["group"] == 0
    ]
    assert not _fitting_coverage(context, duplicates)["valid"]
