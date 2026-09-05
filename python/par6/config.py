"""Packaged PAR6 configuration and asset paths.

The runtime's own config files (``config/PAR6.toml`` and
``config/grippers/*.toml``) and URDF trees are mirrored into ``par6/_data``
by ``scripts/sync_pkg_data.py`` and read through the engine's own loader
(:class:`par6._par6.Config`), so every limit, pose and name the Python
surface exposes is the value the Rust runtime enforces.  This module only
turns those values into paths and waldoctl dataclasses.
"""

from __future__ import annotations

import hashlib
import tomllib
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

from par6._par6 import Config, frame_offset


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


#: Where the arm is when the config does not say.
DEFAULT_CAN_INTERFACE = "can0"


def can_interface(robot_toml: str | None) -> str:
    """The SocketCAN interface ``[bus].interface`` names.

    The engine's own loader does not carry this one out to Python, so it
    is read here and only here: a control box with two interfaces is
    exactly where a second reader with its own idea of the default
    flashes the wrong arm.
    """
    try:
        parsed = tomllib.loads(robot_toml) if robot_toml else {}
    except (tomllib.TOMLDecodeError, TypeError):
        return DEFAULT_CAN_INTERFACE
    bus = parsed.get("bus")
    if isinstance(bus, dict) and isinstance(bus.get("interface"), str):
        return bus["interface"]
    return DEFAULT_CAN_INTERFACE


# ---------------------------------------------------------------------------
# Packaged data
# ---------------------------------------------------------------------------


def config_files(path: str | Path) -> dict:
    """The robot TOML at *path* and the ``grippers/*.toml`` beside it,
    verbatim, in the shape of the daemon's CONFIG_BUNDLE answer.

    ``fingerprint`` is computed the way the daemon computes CONFIG_INFO's:
    sha256 over each file's name, a newline and its content — the robot
    TOML first, then the gripper files by file name — so a preview built
    from the same files reports the same fingerprint the daemon does.
    """
    robot = Path(path)
    digest = hashlib.sha256()

    def read(file: Path) -> tuple[str, str]:
        content = file.read_text()
        digest.update(file.name.encode())
        digest.update(b"\n")
        digest.update(content.encode())
        return file.name, content

    robot_filename, robot_toml = read(robot)
    grippers = [read(f) for f in sorted((robot.parent / "grippers").glob("*.toml"))]
    return {
        "path": str(robot),
        "fingerprint": digest.hexdigest(),
        "robot_filename": robot_filename,
        "robot_toml": robot_toml,
        "grippers": [{"filename": n, "content": c} for n, c in grippers],
    }


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


def data_root() -> Path:
    """Absolute path of the packaged ``par6/_data`` directory — an assets
    tree in the runtime's own layout (``URDF/<tree>/{urdf,srdf,meshes}``)."""
    return Path(str(pkg_files("par6") / "_data")).resolve()


def package_path() -> Path:
    """The installed ``par6`` package directory.  Packaged URDFs reference
    their meshes as ``package://par6/_data/URDF/...``, so a consumer that
    maps ``package://par6`` here resolves them."""
    return data_root().parent


def package_search_dir() -> Path:
    """The directory the engine's loaders resolve ``package://par6/...``
    under (``<this>/par6/...``): the package directory's parent."""
    return package_path().parent


@cache
def config() -> Config:
    """The packaged runtime config, loaded once."""
    return Config(str(data_root() / "config" / "PAR6.toml"))


# ---------------------------------------------------------------------------
# URDF variants — one exported tree per end-effector build
# ---------------------------------------------------------------------------


def _packaged(relpath: str) -> Path:
    path = data_root() / relpath
    if not path.is_file():
        raise FileNotFoundError(
            f"no packaged model at {path}; run scripts/sync_pkg_data.py"
        )
    return path


def urdf_tree(tool_key: str) -> str:
    """Packaged URDF tree directory for *tool_key* — the variant the
    runtime resolves for it (its gripper TOML's ``urdf_variant``, else the
    vendor prefix rule; the flange tree for an unknown key)."""
    return config().variant(tool_key)["urdf_relpath"].split("/")[1]


def urdf_tree_root(tool_key: str) -> Path:
    """Root of the URDF tree for *tool_key*."""
    return data_root() / "URDF" / urdf_tree(tool_key)


def urdf_path(tool_key: str) -> Path:
    """The kinematic model the runtime loads for *tool_key*."""
    return _packaged(config().variant(tool_key)["urdf_relpath"])


def srdf_path(tool_key: str) -> Path:
    """The disabled-collision-pair list the runtime loads for *tool_key*."""
    return _packaged(config().variant(tool_key)["srdf_relpath"])


def tcp_frame(tool_key: str) -> str:
    """The frame the runtime resolves FK/IK/Jacobian at for *tool_key*."""
    return config().variant(tool_key)["tcp_frame"]


#: The flange frame every tree carries; a tool's TCP is measured from it.
FLANGE_FRAME = "gripper"


@cache
def flange_to_tcp(
    tool_key: str,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Flange->TCP of *tool_key*'s URDF tree as ``(origin_m, rpy_rad)``,
    read off the engine's own FK of that tree (identity for the bare
    flange, whose TCP is the flange itself)."""
    x, y, z, r, p, yaw = frame_offset(
        str(urdf_path(tool_key)), FLANGE_FRAME, tcp_frame(tool_key)
    )
    return (x, y, z), (r, p, yaw)


# ---------------------------------------------------------------------------
# Joint limits / home
# ---------------------------------------------------------------------------


def soft_limits_rad() -> np.ndarray:
    """``(N, 2)`` software joint position limits in radians."""
    return np.array(config().soft_limits_rad(), dtype=np.float64)


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


def _kinodynamics(limits: dict) -> KinodynamicLimits:
    return KinodynamicLimits(
        velocity=np.array(limits["velocity"], dtype=np.float64),
        acceleration=np.array(limits["acceleration"], dtype=np.float64),
        jerk=np.array(limits["jerk"], dtype=np.float64),
    )


def build_joints_spec() -> JointsSpec:
    """waldoctl :class:`JointsSpec` from the packaged runtime config.

    Position limits are the SOFT limits (what motion may use); ``hard``
    kinodynamics are the hardware ceiling; ``jog`` kinodynamics are what
    the runtime applies in JOG mode.  Home is the park pose.
    """
    cfg = config()
    rad = soft_limits_rad()
    home_rad = np.array(cfg.park_pose_rad(), dtype=np.float64)
    if cfg.joint_count() != len(JOINT_DISPLAY_NAMES):
        raise RuntimeError(
            f"config has {cfg.joint_count()} joints, "
            f"{len(JOINT_DISPLAY_NAMES)} display names are defined"
        )
    return JointsSpec(
        count=cfg.joint_count(),
        names=JOINT_DISPLAY_NAMES,
        limits=JointLimits(
            position=PositionLimits(deg=np.degrees(rad), rad=rad),
            hard=_kinodynamics(cfg.hardware_limits()),
            jog=_kinodynamics(cfg.limits("jog")),
        ),
        home=HomePosition(deg=np.degrees(home_rad), rad=home_rad),
    )


def homing_ready_pose_rad() -> np.ndarray:
    """Where the configured homing sequence leaves the arm \\[rad\\]."""
    return np.array(config().homing_ready_pose_rad(), dtype=np.float64)


# ---------------------------------------------------------------------------
# Digital I/O
# ---------------------------------------------------------------------------


def io_line_names() -> tuple[list[str], list[str]]:
    """``(inputs, outputs)`` from the runtime's ``[io]`` section, in order.

    The same order the STATUS ``io`` array uses — inputs, then outputs, then
    the e-stop, which is never declared and always last. ``write_io(port)``
    indexes the *outputs* list.
    """
    inputs, outputs = config().io_lines()
    return list(inputs), list(outputs)


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
    return canonical_tool_key(config().active_gripper())
