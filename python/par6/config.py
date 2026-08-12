"""Packaged PAR6 configuration — TOML loading and unit helpers.

Reads the runtime's own config files (``config/PAR6.toml`` and
``config/grippers/*.toml``, mirrored into ``par6/_data/`` by
``scripts/sync_pkg_data.py``) so the Python surface exposes exactly the
values the Rust runtime enforces, with no hand-duplicated numbers.

Everything here is pure data (tomllib, the URDF trees' own XML, numpy and
waldoctl dataclasses); the pinokin-backed kinematics live in :mod:`par6.robot`.
"""

from __future__ import annotations

import tomllib
import xml.etree.ElementTree as ET
from functools import cache
from importlib.resources import files as pkg_files
from pathlib import Path

import numpy as np
from waldoctl import (
    HomePosition,
    JointLimits,
    JointsSpec,
    KinodynamicLimits,
    PositionLimits,
)

#: Runtime floor on the jog ramp time \[s\] (``par6_motion::jog::MIN_ACCEL_TIME_S``).
MIN_JOG_ACCEL_TIME_S = 0.05


def data_root() -> Path:
    """Absolute path of the packaged ``par6/_data`` directory."""
    return Path(str(pkg_files("par6") / "_data")).resolve()


@cache
def load_robot_config() -> dict:
    """Parsed ``config/PAR6.toml`` from the packaged data."""
    with (data_root() / "config" / "PAR6.toml").open("rb") as f:
        return tomllib.load(f)


@cache
def load_gripper_configs() -> dict[str, dict]:
    """Parsed ``config/grippers/*.toml``, keyed by canonical (upper) name."""
    configs: dict[str, dict] = {}
    for path in sorted((data_root() / "config" / "grippers").glob("*.toml")):
        with path.open("rb") as f:
            cfg = tomllib.load(f)
        configs[cfg["name"].strip().upper()] = cfg
    return configs


# ---------------------------------------------------------------------------
# URDF variants — one exported tree per end-effector build
# ---------------------------------------------------------------------------

_FLANGE_TREE = "par6_flange"

# Gripper-TOML name (canonical) -> packaged URDF tree directory.
URDF_TREE_BY_TOOL: dict[str, str] = {
    "FLANGE": _FLANGE_TREE,
    "MSG_SMALL_MOTOR_150MM_RAIL": "par6_msg_gripper",
    "SSG48": "par6_ssg48_gripper",
}


def urdf_tree_root(tool_key: str) -> Path:
    """Root of the URDF tree for *tool_key* (flange tree for unknown keys)."""
    tree = URDF_TREE_BY_TOOL.get(tool_key.strip().upper(), _FLANGE_TREE)
    return data_root() / "urdf" / tree


def urdf_path(tool_key: str) -> Path:
    """The ``.urdf`` file inside the tree for *tool_key*."""
    matches = sorted((urdf_tree_root(tool_key) / "urdf").glob("*.urdf"))
    if not matches:
        raise FileNotFoundError(
            f"no packaged URDF for tool {tool_key!r}; run scripts/sync_pkg_data.py"
        )
    return matches[0]


# ---------------------------------------------------------------------------
# Joint limits / home
# ---------------------------------------------------------------------------


def resolve_mode_limits(limits: dict, mode: str) -> tuple[float, float, float]:
    """(velocity, acceleration, jerk) for *mode* (``"exec"``/``"jog"``/``"stream"``).

    Mirrors ``par6_config::JointLimits::for_mode``: a missing mode block (or a
    missing field inside one) falls back to the hardware ceiling.
    """
    block = limits.get(mode)
    if block is None:
        return (
            limits["velocity_rad_s"],
            limits["acceleration_rad_s2"],
            limits["jerk_rad_s3"],
        )
    return (
        block["velocity_rad_s"],
        block["acceleration_rad_s2"],
        block.get("jerk_rad_s3", limits["jerk_rad_s3"]),
    )


def _mode_kinodynamics(joints: list[dict], mode: str) -> KinodynamicLimits:
    resolved = [resolve_mode_limits(j["limits"], mode) for j in joints]
    return KinodynamicLimits(
        velocity=np.array([r[0] for r in resolved], dtype=np.float64),
        acceleration=np.array([r[1] for r in resolved], dtype=np.float64),
        jerk=np.array([r[2] for r in resolved], dtype=np.float64),
    )


def soft_limits_rad(config: dict | None = None) -> np.ndarray:
    """``(N, 2)`` software joint position limits in radians."""
    joints = (config or load_robot_config())["joints"]
    return np.array(
        [[j["limits"]["soft_min_rad"], j["limits"]["soft_max_rad"]] for j in joints],
        dtype=np.float64,
    )


#: Readout labels for the six axes, positionally matched to the config's
#: ``joint1``..``joint6``.  The config and the URDF both carry structural ids
#: (``joint1`` / ``shoulder_JOINT``), which is what an operator readout must
#: not show.  PAR6 is the PAROL6 kinematic topology — the packaged URDF chain
#: is ``base_link -> shoulder -> upper_arm -> elbow -> lower_arm -> wrist ->
#: gripper`` — so the axes carry the same functional names parol6 gives them
#: (``parol6/robot.py:401``).
JOINT_DISPLAY_NAMES: tuple[str, ...] = (
    "Base",
    "Shoulder",
    "Elbow",
    "Wrist 1",
    "Wrist 2",
    "Wrist 3",
)


def build_joints_spec() -> JointsSpec:
    """waldoctl :class:`JointsSpec` from the packaged runtime config.

    Position limits are the SOFT limits (what motion may use); ``hard``
    kinodynamics are the hardware ceiling; ``jog`` kinodynamics resolve the
    config's jog mode blocks (ceiling when omitted, the vendor rule).  Home
    is the park pose.
    """
    config = load_robot_config()
    joints = config["joints"]
    rad = soft_limits_rad(config)
    home_rad = np.array(config["robot"]["park_pose_rad"], dtype=np.float64)
    if len(joints) != len(JOINT_DISPLAY_NAMES):
        raise RuntimeError(
            f"config has {len(joints)} joints, "
            f"{len(JOINT_DISPLAY_NAMES)} display names are defined"
        )
    return JointsSpec(
        count=len(joints),
        names=JOINT_DISPLAY_NAMES,
        limits=JointLimits(
            position=PositionLimits(deg=np.degrees(rad), rad=rad),
            hard=KinodynamicLimits(
                velocity=np.array(
                    [j["limits"]["velocity_rad_s"] for j in joints], dtype=np.float64
                ),
                acceleration=np.array(
                    [j["limits"]["acceleration_rad_s2"] for j in joints],
                    dtype=np.float64,
                ),
                jerk=np.array(
                    [j["limits"]["jerk_rad_s3"] for j in joints], dtype=np.float64
                ),
            ),
            jog=_mode_kinodynamics(joints, "jog"),
        ),
        home=HomePosition(deg=np.degrees(home_rad), rad=home_rad),
    )


def homing_ready_pose_rad() -> np.ndarray:
    """Where the configured homing sequence leaves the arm \\[rad\\].

    The runtime answers HOME by running its full referencing sequence, so the
    pose it ends at is decided by that sequence's ``move_to`` steps (the last
    one commanded per joint), not by ``[robot].park_pose_rad``.
    """
    final: dict[int, float] = {}
    for step in load_robot_config()["homing"]["sequence"]:
        for move in step.get("move_to", []):
            final[int(move["joint"])] = float(move["position_rad"])
    joints = range(len(load_robot_config()["joints"]))
    missing = [j for j in joints if j not in final]
    if missing:
        raise RuntimeError(f"homing sequence leaves joints {missing} unplaced")
    return np.array([final[j] for j in joints], dtype=np.float64)


def jog_ramp_acceleration() -> np.ndarray:
    """Per-joint acceleration the RT jog ramp actually uses \\[rad/s^2\\].

    Mirrors ``JogEngine::tick`` (``crates/par6-motion/src/jog.rs``):
    ``a = min(v_jog / accel_time_s, a_jog)``, with the runtime's
    ``MIN_ACCEL_TIME_S`` floor applied to the configured ramp time.
    """
    config = load_robot_config()
    jog = config.get("jog", {})
    accel_time_s = max(float(jog.get("accel_time_s", MIN_JOG_ACCEL_TIME_S)), MIN_JOG_ACCEL_TIME_S)
    resolved = [resolve_mode_limits(j["limits"], "jog") for j in config["joints"]]
    return np.array(
        [min(vel / accel_time_s, acc) for vel, acc, _ in resolved], dtype=np.float64
    )


# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------


def canonical_tool_key(name: str) -> str:
    """The one spelling of a tool key this package uses everywhere.

    ``waldoctl.ToolSpec`` upper-cases every key it is given, and the runtime
    matches ``SELECT_TOOL`` / ``TOOL_ACTION`` keys case-insensitively, so the
    upper form is the only spelling that round-trips through both.
    """
    return name.strip().upper()


def fitted_tool_key() -> str:
    """Canonical key of the gripper the runtime is configured with."""
    return canonical_tool_key(load_robot_config()["robot"]["active_gripper"])


# ---------------------------------------------------------------------------
# Tool kinematics
# ---------------------------------------------------------------------------

#: Link the URDF trees give the tool center point (see :func:`flange_to_tcp`).
TCP_LINK = "tcp"


def _rotation(rpy: str | None) -> np.ndarray:
    """URDF ``rpy`` attribute as a rotation matrix (fixed axes, ``Rz·Ry·Rx``)."""
    if rpy is None:
        return np.eye(3)
    r, p, y = (float(v) for v in rpy.split())
    cr, sr, cp, sp, cy, sy = (
        np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    )
    return (
        np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        @ np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        @ np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    )


def _rpy_xyz(R: np.ndarray) -> tuple[float, float, float]:
    """Rotation matrix as intrinsic-XYZ rpy — ``R = Rx(r)·Ry(p)·Rz(y)``.

    The convention ``pinokin.se3_from_rpy`` re-encodes, which is what every
    consumer of a ``ToolSpec``'s ``tcp_rpy`` feeds it back through; the URDF's
    own ``rpy`` attribute is the other order and must not be handed over raw.
    """
    pitch = float(np.arcsin(np.clip(R[0, 2], -1.0, 1.0)))
    if abs(R[0, 2]) < 1.0 - 1e-12:
        return float(np.arctan2(-R[1, 2], R[2, 2])), pitch, float(np.arctan2(-R[0, 1], R[0, 0]))
    return float(np.arctan2(R[2, 1], R[1, 1])), pitch, 0.0


@cache
def flange_to_tcp(tool_key: str) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Flange->TCP of *tool_key*'s URDF tree as ``(origin_m, rpy_rad)``.

    The ONE definition of a native tool's tool center point: the ``tcp`` link
    the tree itself carries, reached from the flange over its fixed joints.
    ``par6d`` resolves FK/IK/Jacobian at that same link of that same tree
    (``par6_kin::GripperVariant::tcp_frame``), and :mod:`par6.robot` loads the
    tree whole — so nothing here can drift from what the runtime does.  A tree
    without a ``tcp`` link (the bare flange) has its last link as the tool
    point, and this is identity.

    NOT taken from the gripper TOML's ``[kinematics]`` block: ``d_m`` /
    ``a_m`` / ``alpha_rad`` there are the vendor's DH row for the tool link,
    stated in the vendor's DH frame 6 — the URDF's ``gripper`` frame turned by
    Rz(pi) — and landing on the gripper's JAW MOUNT, not its TCP.  Both
    variants confirm that frame exactly: ``Rz(pi)*T(a,0,d)*Rx(alpha)``
    reproduces the jaw joint's origin and rpy as the URDF states them.
    Composing that row straight onto the flange, as this module used to,
    mirrors x by ``a`` and rolls the frame by ``alpha`` — the 28.35 mm and
    180 degrees the runtime and the client disagreed by (issue #20).
    """
    root = ET.parse(urdf_path(tool_key)).getroot()
    fixed: dict[str, ET.Element] = {}
    for joint in root.iter("joint"):
        if joint.get("type") == "fixed":
            child = joint.find("child")
            if child is not None:
                fixed[str(child.get("link"))] = joint
    R = np.eye(3)
    t = np.zeros(3)
    link = TCP_LINK
    while link in fixed:
        origin = fixed[link].find("origin")
        xyz = np.zeros(3) if origin is None else np.array(
            [float(v) for v in str(origin.get("xyz", "0 0 0")).split()]
        )
        R_j = _rotation(None if origin is None else origin.get("rpy"))
        t = xyz + R_j @ t
        R = R_j @ R
        parent = fixed[link].find("parent")
        link = "" if parent is None else str(parent.get("link"))
    return (float(t[0]), float(t[1]), float(t[2])), _rpy_xyz(R)
