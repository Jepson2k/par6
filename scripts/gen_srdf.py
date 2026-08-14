#!/usr/bin/env python3
"""Author the per-variant PAR6 SRDFs from sampled collision data.

The vendor ships no SRDF, and the description's coarse collision meshes
leave the arm's own park pose in contact with the base — so a strict
self-collision gate refuses the one pose the config declares valid. The
standard fix is an SRDF listing the self pairs a checker must ignore,
generated the way MoveIt's setup assistant does it: sample the reachable
configuration space and classify each pair by data, never by hand.

For every URDF variant this script:

  1. builds the same Pinocchio+coal checker the runtime uses
     (adjacent pairs structurally removed, the runtime's 5 mm clearance);
  2. checks the configured park pose (config/PAR6.toml) across the jaw
     range — pairs in contact there are ``Default`` (MoveIt's
     default-collisions rule: contact in the reference pose is rest, not
     collision);
  3. samples N configurations uniformly in the per-joint SOFT window
     (jaws across their URDF range) — pairs colliding in ≥ ALWAYS_FRAC of
     samples are ``Always`` (permanent mesh overlap the joint limits
     never resolve);
  4. writes ``<variant>/srdf/<name>.srdf`` next to the URDF, with each
     entry's sampled collision frequency recorded in a comment.

Pairs that collide only sometimes are genuine collisions and stay
enabled; pairs that never collide also stay enabled (disabling them buys
only speed and costs the proof burden).

Needs ``pip install pinokin numpy`` and Pinocchio's python bindings.

    python3 scripts/gen_srdf.py [--samples N] [--seed S]
"""

from __future__ import annotations

import argparse
import math
import tomllib
from collections import Counter
from pathlib import Path

import numpy as np
import pinocchio
import pinokin

REPO = Path(__file__).resolve().parents[1]
ASSETS = REPO / "assets" / "par6_description"
PACKAGE_DIR = ASSETS / "URDF"

#: The runtime's default self-pair standoff (par6d COLLISION_CLEARANCE_M):
#: the park pose must be clear at THIS margin, not just at raw contact.
CLEARANCE_M = 0.005

#: A pair colliding in at least this fraction of soft-window samples is
#: permanent mesh overlap, not a reachable collision.
ALWAYS_FRAC = 0.99

with (REPO / "config" / "PAR6.toml").open("rb") as f:
    _CFG = tomllib.load(f)
SOFT_LIMITS = [
    (j["limits"]["soft_min_rad"], j["limits"]["soft_max_rad"]) for j in _CFG["joints"]
]
PARK_RAD = list(_CFG["robot"]["park_pose_rad"])

# (urdf relpath, srdf relpath) per variant — mirrors
# par6_kin::GripperVariant::{urdf_relpath, srdf_relpath}.
VARIANTS = {
    "par6_flange": (
        "URDF/par6_flange/urdf/par6_flange.urdf",
        "URDF/par6_flange/srdf/par6_flange.srdf",
    ),
    "par6_msg": (
        "URDF/par6_msg_gripper/urdf/PAR6_MSG.urdf",
        "URDF/par6_msg_gripper/srdf/PAR6_MSG.srdf",
    ),
    "par6_ssg48": (
        "URDF/par6_ssg48_gripper/urdf/par6_ssg48_urdf.urdf",
        "URDF/par6_ssg48_gripper/srdf/par6_ssg48_urdf.srdf",
    ),
}


def link_name(geom: str) -> str:
    """Geometry object name -> link name (Pinocchio's URDF parser names
    a link's geometries ``<link>_<i>``)."""
    stem, _, idx = geom.rpartition("_")
    return stem if stem and idx.isdigit() else geom


def pair_key(a: str, b: str) -> tuple[str, str]:
    a, b = link_name(a), link_name(b)
    return (a, b) if a <= b else (b, a)


def build(urdf: str, variant: str, samples: int, seed: int) -> list[str]:
    robot = pinokin.Robot(urdf)
    checker = pinokin.CollisionChecker(
        robot, urdf, [str(PACKAGE_DIR)], clearance_margin=CLEARANCE_M
    )
    model = pinocchio.buildModelFromUrdf(urdf)
    nq = robot.nq
    assert model.nq == nq

    # Jaw (non-arm) DOF ranges from the URDF; the arm's six use the SOFT
    # window — the reachable space, which is what justifies an exclusion.
    lo = np.array(model.lowerPositionLimit)
    hi = np.array(model.upperPositionLimit)
    for j, (smin, smax) in enumerate(SOFT_LIMITS):
        lo[j], hi[j] = smin, smax

    def pairs_at(q: np.ndarray) -> set[tuple[str, str]]:
        return {pair_key(*p) for p in checker.colliding_pairs(q)}

    # Park contact, across the jaw range (the pose is valid whatever the
    # jaws are doing).
    default_pairs: set[tuple[str, str]] = set()
    for jaw_frac in (0.0, 0.5, 1.0):
        q = np.zeros(nq)
        q[:6] = PARK_RAD
        q[6:] = lo[6:] + jaw_frac * (hi[6:] - lo[6:])
        default_pairs |= pairs_at(q)

    rng = np.random.default_rng(seed)
    freq: Counter[tuple[str, str]] = Counter()
    for _ in range(samples):
        q = lo + rng.uniform(size=nq) * (hi - lo)
        for p in pairs_at(q):
            freq[p] += 1

    always_pairs = {p for p, n in freq.items() if n / samples >= ALWAYS_FRAC}

    entries = []
    for a, b in sorted(default_pairs | always_pairs):
        # A pair over the Always threshold is permanent overlap whether or
        # not the park pose also touches it — the stronger claim wins.
        reason = "Always" if (a, b) in always_pairs else "Default"
        f = freq.get((a, b), 0) / samples
        entries.append(
            f'  <disable_collisions link1="{a}" link2="{b}" reason="{reason}" />'
            f"  <!-- colliding in {f:.1%} of samples -->"
        )
        print(f"  {a} - {b}: {reason} (sampled {f:.1%})")

    # The remaining distribution, for the record.
    for p, n in sorted(freq.items(), key=lambda kv: -kv[1]):
        if p not in default_pairs and p not in always_pairs:
            print(f"  kept enabled: {p[0]} - {p[1]} colliding in {n / samples:.1%}")

    return entries


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", type=int, default=20000)
    ap.add_argument("--seed", type=int, default=6)
    args = ap.parse_args()

    for variant, (urdf_rel, srdf_rel) in VARIANTS.items():
        print(f"{variant}:")
        entries = build(str(ASSETS / urdf_rel), variant, args.samples, args.seed)
        name = Path(urdf_rel).stem
        out = ASSETS / srdf_rel
        out.parent.mkdir(parents=True, exist_ok=True)
        body = "\n".join(entries)
        out.write_text(
            f"""<?xml version="1.0"?>
<!--
  Semantic description for PAR6 ({variant}): self-collision pairs the
  checker must ignore. NOT a vendor file - the vendor ships no SRDF.

  Generated by scripts/gen_srdf.py from {args.samples} configurations
  sampled uniformly in the per-joint soft window (config/PAR6.toml, jaws
  across their URDF range) at the runtime's {CLEARANCE_M * 1000:.0f} mm clearance.
  reason="Default": in contact at the configured park pose (rest, not
  collision - MoveIt's default-collisions rule). reason="Always":
  colliding in >= {ALWAYS_FRAC:.0%} of samples (permanent mesh overlap).
  Adjacent (parent/child) pairs are excluded structurally by the checker
  and are not listed. Regenerate rather than edit.
-->
<robot name="{name}">
{body}
</robot>
"""
        )
        print(f"  wrote {out.relative_to(REPO)}")


if __name__ == "__main__":
    main()
