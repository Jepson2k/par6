"""Behavioral tests for the par6 waldoctl ``Robot`` backend.

Kinematics tests drive the real pinokin model on the packaged URDF; they
skip cleanly when pinokin (a binary wheel) is not installed.  The freshness
guard compares the packaged ``par6/_data`` copies against the repo-root
sources, mirroring the generated ``protocol/constants.py`` pattern.
"""

from __future__ import annotations

import hashlib
import subprocess
import sys
import venv
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
PKG_DIR = Path(__file__).resolve().parents[1]
DATA_DIR = PKG_DIR / "par6" / "_data"

pinokin = pytest.importorskip("pinokin")

from par6.config import load_gripper_configs, urdf_path  # noqa: E402
from par6.robot import Robot  # noqa: E402


@pytest.fixture(scope="module")
def robot() -> Robot:
    return Robot()


def _sample_q(rng: np.random.Generator, robot: Robot, margin: float = 0.15) -> np.ndarray:
    lim = robot.joints.limits.position.rad
    return np.array([rng.uniform(lo + margin, hi - margin) for lo, hi in lim])


class TestKinematics:
    def test_fk_ik_roundtrip(self, robot: Robot) -> None:
        rng = np.random.default_rng(1234)
        out = np.zeros(6)
        for _ in range(10):
            q = _sample_q(rng, robot)
            pose = robot.fk(q, out).copy()
            result = robot.ik(pose, q + rng.normal(0.0, 0.05, 6))
            assert result.success, f"IK failed for q={q}: {result.violations}"
            assert result.violations is None
            recovered = robot.fk(result.q, out).copy()
            assert np.linalg.norm(recovered[:3] - pose[:3]) < 1e-6
            assert np.abs(result.q - q).max() < 1e-3

    def test_ik_batch_reseeds_from_previous_solution(self, robot: Robot) -> None:
        rng = np.random.default_rng(99)
        q0 = _sample_q(rng, robot)
        out = np.zeros(6)
        # A short path of nearby configurations: each IK should track its own
        # waypoint when seeded from the previous solution.
        qs = [q0 + i * 0.02 * np.array([1, -1, 1, 0, 1, 0]) for i in range(5)]
        poses = np.stack([robot.fk(q, out).copy() for q in qs])
        results = robot.ik_batch(poses, q0 + 0.03)
        assert all(r.success for r in results)
        for r, q in zip(results, qs):
            assert np.abs(r.q - q).max() < 1e-3
        batch_poses = robot.fk_batch(np.stack([r.q for r in results]))
        assert np.allclose(batch_poses[:, :3], poses[:, :3], atol=1e-6)

    def test_ik_rejects_unreachable_pose(self, robot: Robot) -> None:
        # Well outside the arm's reach envelope.
        pose = np.array([2.0, 0.0, 2.0, 0.0, 0.0, 0.0])
        result = robot.ik(pose, robot.joints.home.rad.copy())
        assert not result.success

    def test_check_limits(self, robot: Robot) -> None:
        lim = robot.joints.limits.position.rad
        inside = lim.mean(axis=1)
        assert robot.check_limits(inside)
        outside = inside.copy()
        outside[4] = lim[4, 1] + 0.1
        assert not robot.check_limits(outside)
        assert not robot.check_limits(np.full(6, np.nan))
        assert not robot.check_limits(np.full(6, np.inf))
        assert not robot.check_limits(np.zeros(3))  # short array


class TestToolTransforms:
    def test_active_tool_moves_tcp_per_gripper_toml(self, robot: Robot) -> None:
        """fk() must return the TCP of the active tool: flange frame composed
        with the DH tool link (d, a, alpha) from that gripper's TOML."""
        grippers = load_gripper_configs()
        bare = pinokin.Robot(str(urdf_path("FLANGE")))  # no tool transform
        rng = np.random.default_rng(7)
        lim = robot.joints.limits.position.rad
        q = np.array([rng.uniform(lo, hi) for lo, hi in lim])
        T6 = bare.fkine(q)
        out = np.zeros(6)
        for key in ("FLANGE", "MSG_SMALL_MOTOR_150MM_RAIL", "SSG48"):
            kin = grippers[key]["kinematics"]
            # DH tool link Tz(d)·Tx(a)·Rx(alpha): TCP sits at (a, 0, d) in
            # the flange frame.
            expected = T6[:3, 3] + T6[:3, :3] @ np.array([kin["a_m"], 0.0, kin["d_m"]])
            robot.set_active_tool(key)
            tcp = robot.fk(q, out)[:3]
            assert np.allclose(tcp, expected, atol=1e-9), key
        # SSG48 sits 60.35 mm further out along the flange axis than the bare
        # flange plate (d: -0.11745 vs -0.0571 in the TOMLs).
        robot.set_active_tool("FLANGE")
        p_flange = robot.fk(q, out)[:3].copy()
        robot.set_active_tool("SSG48")
        p_ssg = robot.fk(q, out)[:3].copy()
        assert np.linalg.norm(p_ssg - p_flange) == pytest.approx(0.06035, abs=1e-9)
        robot.set_active_tool("FLANGE")

    def test_tcp_offset_composes_in_tool_frame(self, robot: Robot) -> None:
        q = robot.joints.home.rad.copy()
        out = np.zeros(6)
        robot.set_active_tool("FLANGE")
        base = robot.fk(q, out).copy()
        robot.set_active_tool("FLANGE", tcp_offset_m=(0.0, 0.0, 0.01))
        shifted = robot.fk(q, out).copy()
        assert np.linalg.norm(shifted[:3] - base[:3]) == pytest.approx(0.01, abs=1e-9)
        robot.set_active_tool("FLANGE")

    def test_urdf_tree_follows_active_tool(self, robot: Robot) -> None:
        robot.set_active_tool("MSG_SMALL_MOTOR_150MM_RAIL")
        assert "par6_msg_gripper" in robot.urdf_path
        assert Path(robot.urdf_path).is_file()
        assert (Path(robot.mesh_dir) / "meshes").is_dir()
        robot.set_active_tool("FLANGE")
        assert "par6_flange" in robot.urdf_path


class TestJointsSpec:
    def test_limit_and_home_unit_conversion(self, robot: Robot) -> None:
        spec = robot.joints
        # Hand-computed from config/PAR6.toml: joint1 soft limit
        # +/-2.8647335 rad = +/-164.13713898 deg.
        assert spec.limits.position.rad[0, 1] == pytest.approx(2.8647335)
        assert spec.limits.position.deg[0, 1] == pytest.approx(164.1371389797406)
        assert spec.limits.position.deg[0, 0] == pytest.approx(-164.1371389797406)
        # Home is the park pose: joint2 = -1.5708 rad = -90.00021046 deg.
        assert spec.home.rad[1] == pytest.approx(-1.5708)
        assert spec.home.deg[1] == pytest.approx(-90.0002104591497)
        # Kinodynamics: hard = hardware ceiling; jog has no config block, so
        # the vendor fall-back-to-ceiling rule applies field by field.
        assert spec.limits.hard.velocity[1] == pytest.approx(1.5)
        assert spec.limits.hard.acceleration[1] == pytest.approx(10.0)
        np.testing.assert_allclose(spec.limits.jog.velocity, spec.limits.hard.velocity)
        assert spec.count == 6 == len(spec.names)


def _tree_digest(root: Path, patterns: tuple[str, ...]) -> dict[str, str]:
    files: dict[str, str] = {}
    for pattern in patterns:
        for path in sorted(root.rglob(pattern)):
            files[str(path.relative_to(root))] = hashlib.sha256(
                path.read_bytes()
            ).hexdigest()
    return files


class TestPackagedData:
    def test_config_copies_are_fresh(self) -> None:
        """python/par6/_data must byte-match the repo sources; regenerate with
        scripts/sync_pkg_data.py (same pattern as protocol/constants.py)."""
        if not (REPO_ROOT / "config").is_dir():
            pytest.skip("repo-root config/ not present (installed package)")
        stale_msg = "packaged data is stale — run scripts/sync_pkg_data.py"
        assert _tree_digest(DATA_DIR / "config", ("*.toml",)) == _tree_digest(
            REPO_ROOT / "config", ("*.toml",)
        ), stale_msg
        src_urdf = REPO_ROOT / "assets" / "par6_description" / "URDF"
        for tree in ("par6_flange", "par6_msg_gripper", "par6_ssg48_gripper"):
            # Only urdf/ + meshes/ are packaged (the URDFs reference nothing
            # outside meshes/); ROS scaffolding and alternate jaw sets stay
            # repo-only.
            for sub, pattern in (("urdf", "*.urdf"), ("meshes", "*.STL")):
                assert _tree_digest(DATA_DIR / "urdf" / tree / sub, (pattern,)) == (
                    _tree_digest(src_urdf / tree / sub, (pattern,))
                ), f"{tree}/{sub}: {stale_msg}"

    def test_gripper_tomls_cover_all_urdf_trees(self) -> None:
        grippers = load_gripper_configs()
        for key in ("FLANGE", "MSG_SMALL_MOTOR_150MM_RAIL", "SSG48"):
            assert key in grippers
            assert urdf_path(key).is_file()


class TestDiscovery:
    def test_load_robot_class_resolves_entry_point(self, tmp_path: Path) -> None:
        """`waldoctl.discovery.load_robot_class("par6")` must resolve after an
        editable install — verified in a scratch venv (system site-packages
        supply waldoctl/numpy/pinokin; par6 itself installs with --no-deps)."""
        from waldoctl.discovery import available_backends

        probe = (
            "from waldoctl.discovery import load_robot_class\n"
            "robot = load_robot_class('par6')()\n"
            "print(robot.name, robot.backend_package)\n"
        )
        if "par6" in available_backends():
            out = subprocess.run(
                [sys.executable, "-c", probe], capture_output=True, text=True
            )
        else:
            env_dir = tmp_path / "venv"
            venv.create(env_dir, system_site_packages=True, with_pip=False)
            py = env_dir / ("Scripts" if sys.platform == "win32" else "bin") / "python"
            install = subprocess.run(
                [
                    str(py),
                    "-m",
                    "pip",
                    "install",
                    "-e",
                    str(PKG_DIR),
                    "--no-deps",
                    "--no-build-isolation",
                    "--quiet",
                ],
                capture_output=True,
                text=True,
            )
            assert install.returncode == 0, install.stderr
            out = subprocess.run(
                [str(py), "-c", probe], capture_output=True, text=True
            )
        assert out.returncode == 0, out.stderr
        assert out.stdout.split() == ["PAR6", "par6"]
