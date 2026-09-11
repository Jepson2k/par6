"""Native stopping geometry must be checked before a calibration approach."""

import shutil
import tomllib
from pathlib import Path

import numpy as np
import pytest

from par6._par6 import CollisionWorld, Preview, calibration_config
from par6.calibration.preflight import check_stream
from par6.calibration.session import CalibrationSession
from par6.calibration.trajectory import move

ROOT = Path(__file__).resolve().parents[2]


def test_approach_checks_native_stopping_projections(tmp_path):
    # From the isolated gravity run's group 4 -> 5 approach. Both its intended
    # and limiter paths were clear, but ServoJ refused the projected target.
    a = np.deg2rad([0, -75.621973, 144.138948, -19.995117, 5.163574, 207.503174])
    b = np.array([0, -0.73984526, 3.1156935, -0.49884951, -0.25991024, 3.64159265])
    source = (ROOT / "config/PAR6.toml").read_text()
    config = tmp_path / "PAR6.toml"
    shutil.copytree(ROOT / "config/grippers", tmp_path / "grippers")
    config.write_text(calibration_config(source, stream_limits=[[0.2, 0.4, 1.2]] * 6))
    robot = tomllib.loads(config.read_text())
    # Use the actual session's pure planner with native geometry and limiter;
    # no client or protocol is needed for preflight.
    session = CalibrationSession(None, tmp_path / "run", tmp_path / "unused.bin")
    session.robot = robot
    session.preview = Preview(
        config=str(config),
        assets=str(ROOT / "assets/par6_description"),
        package_dir=str(ROOT / "assets/par6_description/URDF"),
    )
    session.world = CollisionWorld(
        str(ROOT / "assets/par6_description/URDF/par6_msg_gripper/urdf/PAR6_MSG.urdf"),
        str(ROOT / "assets/par6_description/URDF"),
        str(ROOT / "assets/par6_description/URDF/par6_msg_gripper/srdf/PAR6_MSG.srdf"),
    )
    session.world.set_layer(
        "installation",
        [
            {
                "kind": "box",
                "params": [6.0, 6.0, 0.2],
                "pose": [0.0, 0.0, -0.12, 0.0, 0.0, 0.0],
                "collision": True,
                "margin": None,
                "name": "floor",
            }
        ],
    )
    session.window = np.array(
        [
            [j["limits"]["soft_min_rad"], j["limits"]["soft_max_rad"]]
            for j in robot["joints"]
        ]
    )
    session.limits = np.array(
        [
            [
                j["limits"]["exec"][k]
                for k in ("velocity_rad_s", "acceleration_rad_s2", "jerk_rad_s3")
            ]
            for j in robot["joints"]
        ]
    )
    times, path = move(a, b, np.minimum(session.limits, [0.2, 0.4, 1.2]))
    session.check_path(path)
    predicted, _ = check_stream(session.preview, times, path, session.limits)
    session.check_path(predicted)
    with pytest.raises(ValueError, match="Native stopping projection.*collides"):
        check_stream(
            session.preview, times, path, session.limits, check_path=session.check_path
        )
    slower_times, slower_path = session.plan_position(a, b)
    assert times[-1] < slower_times[-1] <= 2 * times[-1]
    np.testing.assert_allclose(slower_path[-1], b)
    check_stream(
        session.preview,
        slower_times,
        slower_path,
        session.limits,
        check_path=session.check_path,
    )
    # A colliding endpoint cannot be repaired by merely slowing the approach.
    blocked = np.deg2rad([100.30, -58.99, 141.69, 25.81, -2.13, 157.08])
    with pytest.raises(ValueError, match="No clear bounded calibration approach"):
        session.plan_position(a, blocked)


def test_approach_respects_activated_exec_limits_below_stream(tmp_path):
    source = (ROOT / "config/PAR6.toml").read_text()
    config = tmp_path / "PAR6.toml"
    shutil.copytree(ROOT / "config/grippers", tmp_path / "grippers")
    config.write_text(
        calibration_config(
            source,
            stream_limits=[[0.2, 0.4, 1.2]] * 6,
            exec_limits=[[0.16, 0.32, 0.96]] * 6,
        )
    )
    robot = tomllib.loads(config.read_text())
    session = CalibrationSession(None, tmp_path / "run", tmp_path / "unused.bin")
    session.robot = robot
    session.limits = np.tile([0.16, 0.32, 0.96], (6, 1))
    session.window = np.array(
        [
            [j["limits"]["soft_min_rad"], j["limits"]["soft_max_rad"]]
            for j in robot["joints"]
        ]
    )
    session.preview = Preview(
        config=str(config),
        assets=str(ROOT / "assets/par6_description"),
        package_dir=str(ROOT / "assets/par6_description/URDF"),
    )
    session.world = CollisionWorld(
        str(ROOT / "assets/par6_description/URDF/par6_msg_gripper/urdf/PAR6_MSG.urdf"),
        str(ROOT / "assets/par6_description/URDF"),
        str(ROOT / "assets/par6_description/URDF/par6_msg_gripper/srdf/PAR6_MSG.srdf"),
    )
    a = np.deg2rad([0, -90, 170, 0, -20, 180])
    b = a.copy()
    b[0] += 0.15
    times, path = session.plan_position(a, b)
    np.testing.assert_allclose(path[-1], b)
    # Replay exactly the fractions position() will transmit, through the real
    # native limiter; a plausible slow input alone cannot certify its output.
    _, scale = session.position_settings()
    _, peaks = check_stream(
        session.preview,
        times,
        path,
        session.limits,
        **scale,
        check_path=session.check_path,
    )
    assert np.all(np.asarray(peaks) <= session.limits * 1.001 + 1e-6)
    assert np.asarray(peaks)[0, 0] > 0.01
