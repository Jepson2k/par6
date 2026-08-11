#!/usr/bin/env python3
"""Sync runtime config + URDF assets into the pip package data directory.

The pip package builds from ``python/``, so the repo-root ``config/`` TOMLs
and the URDF trees the backend needs must be copied INSIDE the package
(``python/par6/_data/``) — sdists don't reliably follow symlinks.  Run this
after editing ``config/`` or ``assets/par6_description/URDF/``; the
freshness-guard test in ``python/tests/test_robot.py`` fails when the copies
are stale (same pattern as the generated ``protocol/constants.py``).
"""

from __future__ import annotations

import shutil
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
DATA = REPO / "python" / "par6" / "_data"

URDF_TREES = ("par6_flange", "par6_msg_gripper", "par6_ssg48_gripper")


def manifest() -> list[tuple[Path, Path]]:
    """(source, destination) pairs for every packaged data file."""
    pairs: list[tuple[Path, Path]] = []
    for src in sorted((REPO / "config").glob("*.toml")):
        pairs.append((src, DATA / "config" / src.name))
    for src in sorted((REPO / "config" / "grippers").glob("*.toml")):
        pairs.append((src, DATA / "config" / "grippers" / src.name))
    urdf_root = REPO / "assets" / "par6_description" / "URDF"
    for tree in URDF_TREES:
        for sub, pattern in (("urdf", "*.urdf"), ("meshes", "*.STL")):
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
        shutil.copy2(src, dst)
    total = sum(dst.stat().st_size for _, dst in pairs)
    print(f"synced {len(pairs)} files ({total / 1e6:.1f} MB) into {DATA}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
