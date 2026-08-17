"""Behavioral tests for the par6 waldoctl ``Robot`` backend.

Kinematics tests drive the real pinokin model on the packaged URDF; they
skip cleanly when pinokin (a binary wheel) is not installed.  The freshness
guard compares the packaged ``par6/_data`` copies against the repo-root
sources, mirroring the generated ``protocol/constants.py`` pattern.

The tool frame is checked against a live ``par6d --sim`` rather than against
another model of itself — client and runtime agreeing about where the TCP is
is the property the whole tool-frame design exists to hold (issue #20).
"""

from __future__ import annotations

import hashlib
import importlib.util
import re
import subprocess
import sys
import venv
import xml.etree.ElementTree as ET
from collections.abc import Callable
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import pytest
from live_daemon import LiveDaemon, requires_par6d, settle_at

from par6.config import (
    _MODEL_BY_TREE,
    load_gripper_configs,
    srdf_path,
    urdf_path,
    urdf_tree,
)

if TYPE_CHECKING:
    import pinokin

    from par6.robot import Robot
else:
    try:
        import pinokin
    except ImportError:  # binary wheel unavailable on this platform
        pinokin = None

#: Every use of `pinokin` below is inside a test carrying this marker, which
#: is why the import above is typed as the module rather than `module | None`:
#: the marker, not a runtime check, is what keeps the `None` case out.
needs_pinokin = pytest.mark.skipif(
    pinokin is None, reason="pinokin binary wheel not installed"
)

REPO_ROOT = Path(__file__).resolve().parents[2]
PKG_DIR = Path(__file__).resolve().parents[1]
DATA_DIR = PKG_DIR / "par6" / "_data"


@pytest.fixture(scope="module")
def robot() -> Robot:
    from par6.robot import Robot

    return Robot()


def _sample_q(rng: np.random.Generator, robot: Robot, margin: float = 0.15) -> np.ndarray:
    lim = robot.joints.limits.position.rad
    return np.array([rng.uniform(lo + margin, hi - margin) for lo, hi in lim])


# --- URDF readers, written out rather than imported from par6.config: these
# tests are what says the packaged trees mean what the package thinks they
# mean, so they may not lean on the package's own reader to say it. ---------


def _urdf_root(tool_key: str) -> ET.Element:
    return ET.parse(urdf_path(tool_key)).getroot()


def _has_link(tool_key: str, name: str) -> bool:
    return any(link.get("name") == name for link in _urdf_root(tool_key).iter("link"))


def _urdf_transform(element: ET.Element | None) -> np.ndarray:
    """A URDF ``<origin>`` as a 4x4 — fixed-axis rpy, ``R = Rz·Ry·Rx``."""
    T = np.eye(4)
    if element is None:
        return T
    r, p, y = (float(v) for v in str(element.get("rpy", "0 0 0")).split())
    cr, sr, cp, sp, cy, sy = (
        np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    )
    T[:3, :3] = (
        np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        @ np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        @ np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    )
    T[:3, 3] = [float(v) for v in str(element.get("xyz", "0 0 0")).split()]
    return T


def _joint_transform(tool_key: str, joint_name: str) -> np.ndarray:
    """The named joint's placement in its parent link's frame."""
    joint = next(
        j for j in _urdf_root(tool_key).iter("joint") if j.get("name") == joint_name
    )
    return _urdf_transform(joint.find("origin"))


#: Configurations the client and the runtime are compared at.  The first is
#: the one issue #20 measured; the rest spread the wrist and elbow around so
#: an agreement that only holds where the tool axis lines up with the flange
#: axis cannot pass.  All inside every hard window, so the sim's teleport
#: clamp does not move them.
AGREEMENT_POSES_DEG = [
    [0.0, -90.0, 180.0, 0.0, 0.0, 180.0],
    [30.0, -60.0, 200.0, -25.0, 40.0, 120.0],
    [-45.0, -120.0, 250.0, 60.0, -50.0, 300.0],
]


def _link_transform(tool_key: str, link_name: str) -> np.ndarray:
    """The named link's placement in its parent link's frame."""
    joint = next(
        j
        for j in _urdf_root(tool_key).iter("joint")
        if (child := j.find("child")) is not None and child.get("link") == link_name
    )
    return _urdf_transform(joint.find("origin"))


@needs_pinokin
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


@needs_pinokin
class TestToolTransforms:
    def test_active_tool_selects_the_urdf_tree_that_defines_its_tcp(
        self, robot: Robot
    ) -> None:
        """The TCP a tool advertises and the TCP its model resolves at are the
        ``tcp`` link of that tool's URDF tree — one definition, not two.

        A tool frame written down twice drifts: issue #20 had the client
        composing the gripper TOML's DH row onto the flange, landing 28.35 mm
        and 180° from the ``tcp`` link the runtime resolves at.  ``fk()`` must
        now sit exactly on that link, and ``ToolSpec.tcp_origin`` (what a
        consumer places a gizmo from) must name the same point.
        """
        rng = np.random.default_rng(7)
        lim = robot.joints.limits.position.rad
        q = np.array([rng.uniform(lo, hi) for lo, hi in lim])
        out = np.zeros(6)
        for key in ("FLANGE", "MSG_SMALL_MOTOR_150MM_RAIL", "SSG48"):
            tree = pinokin.Robot(str(urdf_path(key)))
            # Both frames off the SAME tree: the trees are separate CAD
            # exports and their arm chains are written to different
            # precisions, so a cross-tree comparison drifts by microns for
            # reasons that have nothing to do with the tool.
            tree.set_ee_frame("gripper")
            flange = tree.fkine(q)
            tree.set_ee_frame("tcp" if _has_link(key, "tcp") else "gripper")
            expected = tree.fkine(q)
            robot.set_active_tool(key)
            assert np.allclose(robot.fk(q, out)[:3], expected[:3, 3], atol=1e-9), key
            spec_tcp = flange[:3, 3] + flange[:3, :3] @ np.array(
                robot.tools[key].tcp_origin
            )
            assert np.allclose(expected[:3, 3], spec_tcp, atol=1e-9), key
        # Each gripper reaches past the bare flange by what its own tree
        # says: 140 mm (MSG) and 160 mm (SSG48).  Cross-tree, so held to the
        # micron the trees' differing chain precision costs, not to 1e-9.
        robot.set_active_tool("FLANGE")
        p_flange = robot.fk(q, out)[:3].copy()
        for key, reach in (("MSG_SMALL_MOTOR_150MM_RAIL", 0.14), ("SSG48", 0.16)):
            robot.set_active_tool(key)
            p_tool = robot.fk(q, out)[:3].copy()
            assert np.linalg.norm(p_tool - p_flange) == pytest.approx(reach, abs=1e-5)
        robot.set_active_tool("FLANGE")

    def test_gripper_toml_dh_row_describes_the_jaw_mount_not_the_tcp(self) -> None:
        """Why the gripper TOML's ``[kinematics]`` d/a/alpha is not a TCP.

        It is the vendor's DH row for the tool link, stated in the vendor's DH
        frame 6 — the URDF ``gripper`` frame turned by Rz(pi) — and it lands
        on the gripper's jaw mount.  Read in that frame it reproduces the jaw
        joint each tree declares, origin and orientation, which is what
        identifies it; read as a flange->TCP transform (what this package used
        to do) it names a point 28.35 mm and 180° off the tree's ``tcp`` link.
        This is the evidence the URDF is the authoritative frame, so it is
        checked rather than remembered.
        """
        rz_pi = np.eye(4)
        rz_pi[:3, :3] = np.diag([-1.0, -1.0, 1.0])
        grippers = load_gripper_configs()
        for key, jaw in (("MSG_SMALL_MOTOR_150MM_RAIL", "joint_jaw1"), ("SSG48", "jaw1_JOINT")):
            kin = grippers[key]["kinematics"]
            dh = np.zeros((4, 4))
            pinokin.se3_from_rpy(
                kin["a_m"], 0.0, kin["d_m"], kin["alpha_rad"], 0.0, 0.0, dh
            )
            assert np.allclose(rz_pi @ dh, _joint_transform(key, jaw), atol=1e-4), key
            tcp = _link_transform(key, "tcp")[:3, 3]
            assert np.linalg.norm(dh[:3, 3] - tcp) > 0.005, key

    @requires_par6d
    @pytest.mark.e2e
    @pytest.mark.timeout(180)
    # Both gripper variants. Not the bare flange: the shipped homing
    # sequence references the fitted gripper's driver, so a runtime
    # configured with `Flange` refuses to start on its own config.
    @pytest.mark.parametrize("fitted_gripper", ["MSG_small_motor_150mm_rail", "SSG48"])
    async def test_tcp_agrees_with_a_live_daemon(
        self, tmp_path: Path, fitted_gripper: str
    ) -> None:
        """The client's TCP and the runtime's TCP are the same point.

        This is the property issue #20 was the absence of, and the only one
        that decides whether a ``move_l`` preview draws where the arm goes:
        Waldo Commander reads the pose from the runtime and the preview from
        this backend, so a frame the two model separately is a frame that
        drifts.  Checked at several configurations against a real
        ``par6d --sim`` fitted with each gripper in turn — the daemon's URDF
        variant follows ``[robot].active_gripper``, so each run exercises a
        different tool tree on both sides.

        Both position and orientation, each side decoded in its own
        documented convention: STATUS carries the pose as a matrix, while
        :meth:`Robot.fk` reports intrinsic-XYZ rpy (pinokin's), so comparing
        the reconstructed matrices is what catches a frame that is rotated
        rather than merely displaced — the 180° half of the original bug.
        """
        from par6.robot import Robot as Par6Robot

        client_robot = Par6Robot()
        client_robot.set_active_tool(fitted_gripper)
        live = LiveDaemon.start(tmp_path, active_gripper=fitted_gripper)
        try:
            async with live.client() as client:
                assert await client.wait_status(lambda s: s.link_ok == 1, timeout=30.0)
                for angles_deg in AGREEMENT_POSES_DEG:
                    await settle_at(client, angles_deg)
                    # One STATUS frame: the joint angles and the pose the
                    # runtime derived from them, measured at one instant.
                    status = await client.status()
                    assert status is not None, "the runtime answered no STATUS"
                    T_runtime = np.array(status.pose, dtype=np.float64).reshape(4, 4)

                    out = np.zeros(6)
                    pose = client_robot.fk(np.radians(status.angles), out)
                    T_client = np.zeros((4, 4))
                    x, y, z, rx, ry, rz = pose
                    pinokin.se3_from_rpy(x, y, z, rx, ry, rz, T_client)

                    assert np.allclose(
                        T_client[:3, 3] * 1000.0, T_runtime[:3, 3], atol=1e-3
                    ), (
                        f"{fitted_gripper} at {angles_deg}: client TCP "
                        f"{T_client[:3, 3] * 1000.0} mm vs runtime {T_runtime[:3, 3]} mm"
                    )
                    assert np.allclose(
                        T_client[:3, :3], T_runtime[:3, :3], atol=1e-6
                    ), (
                        f"{fitted_gripper} at {angles_deg}: client and runtime "
                        f"disagree about the tool orientation\n{T_client}\n{T_runtime}"
                    )
        finally:
            live.stop()

    def test_tcp_offset_composes_in_tool_frame(self, robot: Robot) -> None:
        q = robot.joints.home.rad.copy()
        out = np.zeros(6)
        robot.set_active_tool("FLANGE")
        base = robot.fk(q, out).copy()
        robot.set_active_tool("FLANGE", tcp_offset_m=(0.0, 0.0, 0.01))
        shifted = robot.fk(q, out).copy()
        assert np.linalg.norm(shifted[:3] - base[:3]) == pytest.approx(0.01, abs=1e-9)
        robot.set_active_tool("FLANGE")

    def test_cartesian_limits_are_derived_from_this_arm(self, robot: Robot) -> None:
        """``cartesian_limits`` must describe THIS arm at THIS tool.

        Two checks a hand-picked constant cannot pass: the reported rates have
        to be achievable at the pose the arm parks in, under the config's own
        JOG joint velocity limits; and the angular rate has to move with the
        tool, because spinning the TCP about a point further from the wrist
        costs more joint speed (the linear rate does not — a rigid body that
        translates without rotating translates identically everywhere).
        """
        robot.set_active_tool("FLANGE")
        flange = robot.cartesian_limits
        q = robot.joints.home.rad.copy()
        jog = robot.joints.limits.jog.velocity
        J = robot.jacobian(q)
        for axis, rate in enumerate(
            [flange.velocity.linear] * 3 + [flange.velocity.angular] * 3
        ):
            twist = np.zeros(6)
            twist[axis] = rate
            q_dot = np.linalg.pinv(J) @ twist
            assert np.all(np.abs(q_dot) <= jog * 1.5), (
                f"axis {axis} at {rate:.3f} needs {q_dot} rad/s, "
                f"beyond the jog ceiling {jog}"
            )
        assert flange.velocity.linear > 0.0 and flange.velocity.angular > 0.0
        assert flange.acceleration.linear > flange.velocity.linear

        for key in ("MSG_SMALL_MOTOR_150MM_RAIL", "SSG48"):
            robot.set_active_tool(key)
            with_tool = robot.cartesian_limits
            assert with_tool.velocity.angular < flange.velocity.angular, (
                f"{key}: the angular envelope must tighten at a TCP further out"
            )
            # Equal, to the precision the trees are written at: each tool is
            # its own CAD export and their arm chains carry different
            # rounding, which moves the sampled median by a few parts per
            # hundred thousand.
            assert with_tool.velocity.linear == pytest.approx(
                flange.velocity.linear, rel=1e-3
            )
        robot.set_active_tool("FLANGE")

    def test_urdf_tree_follows_active_tool(self, robot: Robot) -> None:
        robot.set_active_tool("MSG_SMALL_MOTOR_150MM_RAIL")
        assert "par6_msg_gripper" in robot.urdf_path
        assert Path(robot.urdf_path).is_file()
        assert (Path(robot.mesh_dir) / "meshes").is_dir()
        robot.set_active_tool("FLANGE")
        assert "par6_flange" in robot.urdf_path


@needs_pinokin
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


def _sync_script():
    """``scripts/sync_pkg_data.py`` — the producer of ``python/par6/_data``."""
    spec = importlib.util.spec_from_file_location(
        "par6_sync_pkg_data", REPO_ROOT / "scripts" / "sync_pkg_data.py"
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _tree_digest(
    root: Path,
    patterns: tuple[str, ...],
    payload: Callable[[Path], bytes] = Path.read_bytes,
) -> dict[str, str]:
    files: dict[str, str] = {}
    for pattern in patterns:
        for path in sorted(root.rglob(pattern)):
            files[str(path.relative_to(root))] = hashlib.sha256(
                payload(path)
            ).hexdigest()
    return files


class TestPackagedData:
    def test_config_copies_are_fresh(self) -> None:
        """python/par6/_data must match what scripts/sync_pkg_data.py produces
        from the repo sources (same pattern as protocol/constants.py).  TOMLs
        and meshes are byte copies; ``.urdf`` files go through the script's
        ``packaged_bytes`` rewrite, so the guard compares against that."""
        if not (REPO_ROOT / "config").is_dir():
            pytest.skip("repo-root config/ not present (installed package)")
        stale_msg = "packaged data is stale — run scripts/sync_pkg_data.py"
        assert _tree_digest(DATA_DIR / "config", ("*.toml",)) == _tree_digest(
            REPO_ROOT / "config", ("*.toml",)
        ), stale_msg
        packaged_bytes = _sync_script().packaged_bytes
        src_urdf = REPO_ROOT / "assets" / "par6_description" / "URDF"
        for tree in ("par6_flange", "par6_msg_gripper", "par6_ssg48_gripper"):
            # Only urdf/ + meshes/ are packaged (the URDFs reference nothing
            # outside meshes/); ROS scaffolding and alternate jaw sets stay
            # repo-only.
            for sub, pattern in (("urdf", "*.urdf"), ("meshes", "*.STL")):
                assert _tree_digest(DATA_DIR / "urdf" / tree / sub, (pattern,)) == (
                    _tree_digest(src_urdf / tree / sub, (pattern,), packaged_bytes)
                ), f"{tree}/{sub}: {stale_msg}"

    def test_every_shipped_tool_resolves_to_a_packaged_model(self) -> None:
        """The tree a tool resolves to has to be the runtime's, not a fallback.

        ``par6d::kin::variant_for`` matches ``MSG*`` and ``SSG48*`` by prefix
        and falls back to the flange; this side has to agree, or the client
        checks a collision model and renders a mesh for a different tool than
        the one the arm is wearing. A gripper TOML added under a name either
        rule misses is the way that goes wrong, so this walks every shipped
        file rather than a list.
        """
        grippers = load_gripper_configs()
        assert len(grippers) > 3, "the shipped tools are all here"
        seen_trees = set()
        for key in grippers:
            assert urdf_path(key).is_file(), key
            assert srdf_path(key).is_file(), key
            tree = urdf_tree(key)
            seen_trees.add(tree)
            if key.startswith("MSG"):
                assert tree == "par6_msg_gripper", f"{key} fell back to {tree}"
            elif key.startswith("SSG48"):
                assert tree == "par6_ssg48_gripper", f"{key} fell back to {tree}"
        assert seen_trees == set(_MODEL_BY_TREE), (
            f"a packaged tree no shipped tool reaches: {set(_MODEL_BY_TREE) - seen_trees}"
        )

    @needs_pinokin
    def test_packaged_urdfs_are_consumable_by_a_scene(self) -> None:
        """Every packaged tree must satisfy what a 3-D consumer needs of it:
        ``package://`` names that resolve through
        ``{robot.backend_package: robot.mesh_dir}`` onto files that exist, and
        exactly the six actuated joints the arm has — a gripper tree that also
        actuates its jaws is a 8-DOF model against six joint limits."""
        from par6.robot import Robot

        bot = Robot()
        for key in ("FLANGE", "MSG_SMALL_MOTOR_150MM_RAIL", "SSG48"):
            bot.set_active_tool(key)
            urdf = Path(bot.urdf_path)
            text = urdf.read_text()
            packages = set(re.findall(r"package://(\w+)", text))
            assert packages == {bot.backend_package}, key
            root = Path(bot.mesh_dir)
            for ref in re.findall(r'filename="package://\w+/([^"]+)"', text):
                assert (root / ref).is_file(), f"{key}: {ref} does not exist"
            assert pinokin.Robot(str(urdf)).nq == bot.joints.count, key


@needs_pinokin
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
