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


def materialize_bundle(bundle: dict) -> Path:
    """Write a CONFIG_BUNDLE query result to a local cache directory and
    return the robot TOML's path — the config the preview engine should
    load so previews run exactly the daemon's numbers.

    The directory is keyed by the bundle's content fingerprint, so a
    re-fetch of unchanged config is a no-op and a tuned daemon lands in a
    fresh directory.  Writes go to a temp dir first and are renamed into
    place, so a concurrent materialization of the same fingerprint cannot
    be observed half-written.
    """
    import os
    import shutil
    import tempfile

    # The reply is an unauthenticated datagram: the fingerprint names the
    # cache directory and the file names land inside it, so anything but
    # a hex digest and plain leaf names would write outside the cache.
    fingerprint = str(bundle["fingerprint"])
    if not fingerprint or not all(c in "0123456789abcdef" for c in fingerprint):
        raise ValueError(
            f"config bundle fingerprint is not a hex digest: {fingerprint!r}"
        )
    robot_filename = _leaf_name(bundle["robot_filename"], "robot file")
    cache_root = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
    target = cache_root / "par6" / "daemon-config" / fingerprint
    robot_path = target / robot_filename
    if robot_path.is_file():
        return robot_path
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = Path(tempfile.mkdtemp(dir=target.parent, prefix=".materialize-"))
    try:
        (tmp / robot_filename).write_text(str(bundle["robot_toml"]))
        (tmp / "grippers").mkdir()
        for g in bundle.get("grippers", []):
            name = _leaf_name(g["filename"], "gripper file")
            (tmp / "grippers" / name).write_text(str(g["content"]))
        try:
            tmp.rename(target)
        except OSError:
            # A concurrent materialization of the same fingerprint won the
            # rename; its content is byte-identical by construction.
            if not robot_path.is_file():
                raise
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return robot_path


def _leaf_name(raw: object, what: str) -> str:
    """The basename of a daemon-supplied file name — ``..`` survives
    ``Path.name`` and would resolve to the parent directory."""
    name = Path(str(raw)).name
    if name in ("", ".", ".."):
        raise ValueError(f"config bundle {what} is not a file name: {raw!r}")
    return name


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

# Tree directory -> (model, disabled-pair list) the RUNTIME loads for it,
# mirroring ``par6_kin::GripperVariant::urdf_relpath`` / ``srdf_relpath``.
# Named rather than globbed: the flange tree also carries
# ``par6_arm.urdf`` (the arm-only gravity chain the runtime attaches tool
# inertials onto), so picking a tree's model by directory order hands
# this side a different model than the runtime enforces against.
_MODEL_BY_TREE: dict[str, tuple[str, str]] = {
    "par6_flange": ("par6_flange.urdf", "par6_flange.srdf"),
    "par6_msg_gripper": ("PAR6_MSG.urdf", "PAR6_MSG.srdf"),
    "par6_ssg48_gripper": ("par6_ssg48_urdf.urdf", "par6_ssg48_urdf.srdf"),
}


def urdf_tree(tool_key: str) -> str:
    """Packaged URDF tree directory for *tool_key*.

    A PREFIX rule, not a lookup table, because that is what the runtime
    uses (``par6d::kin::variant_for``) and the two have to agree: every
    MSG rail/motor variant shares one gripper mesh set, so a table would
    need an entry per gripper TOML and a new TOML would silently fall
    back to the flange here while the runtime loaded the MSG model.
    An unrecognised tool really does get the flange tree on both sides —
    the arm is always modeled, the tool is not.
    """
    key = tool_key.strip().upper()
    if key.startswith("MSG"):
        return "par6_msg_gripper"
    if key.startswith("SSG48"):
        return "par6_ssg48_gripper"
    return _FLANGE_TREE


def urdf_tree_root(tool_key: str) -> Path:
    """Root of the URDF tree for *tool_key* (flange tree for unknown keys)."""
    return data_root() / "urdf" / urdf_tree(tool_key)


def _tree_model(tool_key: str, subdir: str, which: int) -> Path:
    tree = urdf_tree(tool_key)
    path = data_root() / "urdf" / tree / subdir / _MODEL_BY_TREE[tree][which]
    if not path.is_file():
        raise FileNotFoundError(
            f"no packaged {subdir} model for tool {tool_key!r} at {path}; "
            "run scripts/sync_pkg_data.py"
        )
    return path


def urdf_path(tool_key: str) -> Path:
    """The kinematic model the runtime loads for *tool_key*."""
    return _tree_model(tool_key, "urdf", 0)


def srdf_path(tool_key: str) -> Path:
    """The disabled-collision-pair list the runtime loads for *tool_key*."""
    return _tree_model(tool_key, "srdf", 1)


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
    accel_time_s = max(
        float(jog.get("accel_time_s", MIN_JOG_ACCEL_TIME_S)), MIN_JOG_ACCEL_TIME_S
    )
    resolved = [resolve_mode_limits(j["limits"], "jog") for j in config["joints"]]
    return np.array(
        [min(vel / accel_time_s, acc) for vel, acc, _ in resolved], dtype=np.float64
    )


# ---------------------------------------------------------------------------
# Digital I/O
# ---------------------------------------------------------------------------


def io_line_names() -> tuple[list[str], list[str]]:
    """``(inputs, outputs)`` from the runtime's ``[io]`` section, in order.

    The same order the STATUS ``io`` array uses — inputs, then outputs, then
    the e-stop, which is never declared and always last. ``write_io(port)``
    indexes the *outputs* list.
    """
    io = load_robot_config().get("io", {})
    return (
        [line["name"] for line in io.get("inputs", [])],
        [line["name"] for line in io.get("outputs", [])],
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


def fitted_tool_ilim_ma() -> float:
    """The fitted gripper driver's current limit \\[mA\\].

    The full-scale value a tool move defaults to, and what the runtime
    validates a requested current against.
    """
    cfg = load_gripper_configs()[fitted_tool_key()]
    return float(cfg["driver"]["ilim_ma"])


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
        np.cos(r),
        np.sin(r),
        np.cos(p),
        np.sin(p),
        np.cos(y),
        np.sin(y),
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
        return (
            float(np.arctan2(-R[1, 2], R[2, 2])),
            pitch,
            float(np.arctan2(-R[0, 1], R[0, 0])),
        )
    return float(np.arctan2(R[2, 1], R[1, 1])), pitch, 0.0


@cache
def flange_to_tcp(
    tool_key: str,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
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
        xyz = (
            np.zeros(3)
            if origin is None
            else np.array([float(v) for v in str(origin.get("xyz", "0 0 0")).split()])
        )
        R_j = _rotation(None if origin is None else origin.get("rpy"))
        t = xyz + R_j @ t
        R = R_j @ R
        parent = fixed[link].find("parent")
        link = "" if parent is None else str(parent.get("link"))
    return (float(t[0]), float(t[1]), float(t[2])), _rpy_xyz(R)
