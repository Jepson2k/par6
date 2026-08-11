"""Packaged PAR6 configuration — TOML loading and unit helpers.

Reads the runtime's own config files (``config/PAR6.toml`` and
``config/grippers/*.toml``, mirrored into ``par6/_data/`` by
``scripts/sync_pkg_data.py``) so the Python surface exposes exactly the
values the Rust runtime enforces, with no hand-duplicated numbers.

Everything here is pure data (tomllib + numpy + waldoctl dataclasses); the
pinokin-backed kinematics live in :mod:`par6.robot`.
"""

from __future__ import annotations

import tomllib
from functools import cache
from importlib.resources import files as pkg_files
from pathlib import Path

import numpy as np
from waldoctl import (
    CartesianKinodynamicLimits,
    HomePosition,
    JointLimits,
    JointsSpec,
    KinodynamicLimits,
    LinearAngularLimits,
    PositionLimits,
)


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
    return JointsSpec(
        count=len(joints),
        names=tuple(j["name"] for j in joints),
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


# ---------------------------------------------------------------------------
# Cartesian jog caps
# ---------------------------------------------------------------------------

# The runtime config carries only joint-space limits; Cartesian jog is bounded
# client-side (WC jog UI scaling) until the runtime grows enforced Cartesian
# limits.  Magnitudes sized for a desktop 6-axis arm of PAR6's reach.
CARTESIAN_JOG_LIMITS = CartesianKinodynamicLimits(
    velocity=LinearAngularLimits(linear=0.16, angular=1.4),
    acceleration=LinearAngularLimits(linear=0.55, angular=4.8),
)


# ---------------------------------------------------------------------------
# Tool kinematics
# ---------------------------------------------------------------------------


def tool_tcp(kinematics: dict) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Flange->TCP as ``(origin_m, rpy_rad)`` from a gripper TOML kinematics block.

    The vendor models the end-effector as one DH link after joint 6:
    ``Tz(d) @ Tx(a) @ Rx(alpha)`` — translation ``(a, 0, d)`` then a roll of
    ``alpha`` about the tool x-axis.
    """
    return (
        (kinematics["a_m"], 0.0, kinematics["d_m"]),
        (kinematics["alpha_rad"], 0.0, 0.0),
    )
