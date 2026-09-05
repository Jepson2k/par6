#!/usr/bin/env python3
"""Sync runtime config + URDF assets into the pip package data directory.

The pip package builds from ``python/``, so the repo-root ``config/`` TOMLs
and the URDF trees the backend needs must be copied INSIDE the package
(``python/par6/_data/``) — sdists don't reliably follow symlinks.  Run this
after editing ``config/`` or ``assets/par6_description/URDF/``; the
freshness-guard test in ``python/tests/test_robot.py`` fails when the copies
are stale (same pattern as the generated ``protocol/constants.py``).

The ``.urdf`` files are not copied verbatim — :func:`packaged_bytes` applies
the two rewrites the packaged (client-facing) copies need.  ``assets/`` stays
untouched, and the runtime keeps loading the originals from there.
"""

from __future__ import annotations

import re
import shutil
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
DATA = REPO / "python" / "par6" / "_data"

URDF_TREES = ("par6_flange", "par6_msg_gripper", "par6_ssg48_gripper")

#: ROS package name the packaged URDFs declare their meshes under.  It must
#: equal ``Robot.backend_package``: a consumer resolves ``package://`` through
#: ``{robot.backend_package: robot.mesh_dir}`` (Waldo Commander's
#: ``main.py:199``), and the per-tree names the SolidWorks export wrote
#: (``par6_flange``, ``par6_msg_gripper``, …) match no key in that map, so
#: every mesh path falls back to the URDF's own directory and resolves to a
#: file that does not exist.
PACKAGE_NAME = "par6"

#: Gripper jaws are tool degrees of freedom: the runtime drives them through
#: TOOL_ACTION and reports them as ``ToolStatus.positions``, never as arm
#: joints.  Left prismatic, they make every consumer of a gripper tree see an
#: 8-DOF arm — ``pinokin.Robot(urdf_path).nq == 8`` against six joint limits,
#: and the last actuated joint (what a TCP gizmo parents to) becomes a jaw.
_JAW_JOINT = re.compile(
    r'(<joint\b[^>]*name="(?:joint_)?jaw\d+(?:_JOINT)?"[^>]*)type="prismatic"',
    re.IGNORECASE,
)


def packaged_bytes(src: Path) -> bytes:
    """Bytes of *src* as they belong in ``python/par6/_data``.

    Everything but ``.urdf`` is copied verbatim.
    """
    if src.suffix.lower() != ".urdf":
        return src.read_bytes()
    text = src.read_text(encoding="utf-8")
    text = re.sub(r"package://[A-Za-z0-9_]+/", f"package://{PACKAGE_NAME}/", text)
    text = _JAW_JOINT.sub(r'\1type="fixed"', text)
    return text.encode("utf-8")


def manifest() -> list[tuple[Path, Path]]:
    """(source, destination) pairs for every packaged data file."""
    pairs: list[tuple[Path, Path]] = []
    for src in sorted((REPO / "config").glob("*.toml")):
        pairs.append((src, DATA / "config" / src.name))
    for src in sorted((REPO / "config" / "grippers").glob("*.toml")):
        pairs.append((src, DATA / "config" / "grippers" / src.name))
    urdf_root = REPO / "assets" / "par6_description" / "URDF"
    for tree in URDF_TREES:
        # Both cases: the SolidWorks exports are `.STL`, the decimated
        # `_simplified` variants beside them are `.stl`.
        for sub, pattern in (
            ("urdf", "*.urdf"),
            ("srdf", "*.srdf"),
            ("meshes", "*.STL"),
            ("meshes", "*.stl"),
        ):
            for src in sorted((urdf_root / tree / sub).glob(pattern)):
                pairs.append((src, DATA / "urdf" / tree / sub / src.name))
    return pairs


def main() -> int:
    pairs = manifest()
    missing = [str(s) for s, _ in pairs if not s.is_file()]
    if missing:
        print(f"missing sources: {missing}", file=sys.stderr)
        return 1
    if DATA.exists():
        shutil.rmtree(DATA)
    for src, dst in pairs:
        dst.parent.mkdir(parents=True, exist_ok=True)
        if src.suffix.lower() == ".urdf":
            dst.write_bytes(packaged_bytes(src))
        else:
            shutil.copy2(src, dst)
    total = sum(dst.stat().st_size for _, dst in pairs)
    print(f"synced {len(pairs)} files ({total / 1e6:.1f} MB) into {DATA}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
