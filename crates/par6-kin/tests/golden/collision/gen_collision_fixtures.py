#!/usr/bin/env python3
"""Regenerate the collision golden fixtures from the Python reference stack.

The expected verdicts and pair sets come from ``pinokin.CollisionChecker`` —
the same Pinocchio+coal stack the Waldo Commander client checks against — so
``crates/par6-kin/tests/golden_collision.rs`` is a genuine cross-stack
conformance test rather than a recording of this crate's own output.

The *cases* are derived from the requirement, not from either
implementation: the home pose and a reachable outstretched pose must be
clear; the tool driven into the base, the tool folded back onto the
shoulder, and the forearm folded over the base are self-collisions the arm
can actually reach (every joint value is asserted against the per-joint
soft limits read from config/PAR6.toml); a keep-out box centred on a
configuration's TCP must be hit by the tool that reaches into it.

Needs ``pip install pinokin numpy`` (the same release the client uses).

    python3 crates/par6-kin/tests/golden/collision/gen_collision_fixtures.py
"""

from __future__ import annotations

import json
import math
import tomllib
from pathlib import Path

import numpy as np
import pinokin

REPO = Path(__file__).resolve().parents[5]
ASSETS = REPO / "assets" / "par6_description"
PACKAGE_DIR = ASSETS / "URDF"
OUT_DIR = Path(__file__).resolve().parent

# Per-joint soft limits, straight from the shipped config: model q is the
# vendor motor convention, so the fixture poses must be poses the real arm
# is allowed to reach.
with (REPO / "config" / "PAR6.toml").open("rb") as f:
    SOFT_LIMITS = [
        (j["limits"]["soft_min_rad"], j["limits"]["soft_max_rad"])
        for j in tomllib.load(f)["joints"]
    ]

VARIANTS = {
    "par6_flange": ("URDF/par6_flange/urdf/par6_flange.urdf", "gripper"),
    "par6_msg": ("URDF/par6_msg_gripper/urdf/PAR6_MSG.urdf", "tcp"),
    "par6_ssg48": ("URDF/par6_ssg48_gripper/urdf/par6_ssg48_urdf.urdf", "tcp"),
}

# Kind -> coal constructor arity, mirroring waldoctl.shapes.
KIND_PARAMS = {
    "box": 3,
    "sphere": 1,
    "cylinder": 2,
    "capsule": 2,
    "cone": 2,
    "ellipsoid": 3,
    "plane": 4,
}


def rpy_matrix(rx: float, ry: float, rz: float) -> np.ndarray:
    """R = Rz(rz) @ Ry(ry) @ Rx(rx) — waldoctl's ``Shape.pose`` convention.

    Extrinsic XYZ: every angle turns about a fixed world axis, x first.
    Deliberately NOT pinokin's ``se3_from_rpy`` (Rx·Ry·Rz), which is the
    convention for a TCP pose, not for a shape placement — the other two
    implementations of this contract (parol6's ``_pose_to_matrix`` and the
    frontend's renderer) both place shapes the extrinsic way, and a
    multi-axis tilt is where the two orders part company.
    """

    def rot(axis: int, a: float) -> np.ndarray:
        c, s = math.cos(a), math.sin(a)
        m = np.eye(3)
        i, j = [(1, 2), (2, 0), (0, 1)][axis]
        m[i, i] = c
        m[j, j] = c
        m[i, j] = -s
        m[j, i] = s
        return m

    return rot(2, rz) @ rot(1, ry) @ rot(0, rx)


def pose_matrix(pose: list[float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(pose[3], pose[4], pose[5])
    T[:3, 3] = pose[:3]
    return np.asfortranarray(T)


def shape(name: str, kind: str, params, pose, margin=None) -> dict:
    assert len(params) == KIND_PARAMS[kind], (kind, params)
    assert len(pose) == 6
    return {
        "name": name,
        "kind": kind,
        "params": [float(p) for p in params],
        "pose": [float(p) for p in pose],
        "collision": True,
        "margin": None if margin is None else float(margin),
    }


# Configurations, arm joints only. Physical intent in the comments; the
# expected verdict is whatever the reference stack says, and the test also
# asserts that the two "clear" poses really are clear (a fixture where the
# home pose collided would mean the model, not the test, is wrong).
CONFIGS = {
    # Straight up over the base (the arm's transport/park neighbourhood).
    "home": [0.0, -1.5708, 3.1416, 0.0, 0.0, 0.0],
    "reach_out": [-0.4, -1.2708, 3.7416, -0.2, 0.5, 0.3],
    # Elbow fully flexed with the wrist pitched down: the tool dives into
    # the base plate.
    "tool_into_base": [0.0, -0.25, 2.1, 0.0, -1.0, 0.0],
    # Elbow fully flexed, wrist pitched all the way back: the tool (and
    # jaws, on the gripper variants) folds onto the shoulder/upper arm.
    # The bare flange is short enough to stay clear — a genuinely
    # variant-dependent verdict.
    "tool_onto_shoulder": [0.0, -1.27, 1.9913, 0.0, -1.73, 0.0],
    # Shoulder at its soft minimum with the elbow hyper-extended: the
    # forearm and wrist sweep down over the base.
    "folded_over_base": [0.0, -2.44, 6.56, 0.0, 0.0, 0.0],
}


def check(checker, q_arm: list[float], nq_full: int):
    q = np.zeros(nq_full)
    q[:6] = q_arm
    pairs = sorted(tuple(sorted(p)) for p in checker.colliding_pairs(q))
    return bool(checker.in_collision(q)), [list(p) for p in pairs]


def build(variant: str, relpath: str, ee_frame: str) -> dict:
    urdf = str(ASSETS / relpath)
    robot = pinokin.Robot(urdf)
    robot.set_ee_frame(ee_frame)
    base = pinokin.CollisionChecker(robot, urdf, [str(PACKAGE_DIR)])
    nq_full = robot.nq

    for name, q in CONFIGS.items():
        assert all(
            lo <= v <= hi for v, (lo, hi) in zip(q, SOFT_LIMITS)
        ), f"{name} exceeds soft limits"

    fixture = {
        "urdf": relpath,
        "package_dir": "URDF",
        "ee_frame": ee_frame,
        "nq_full": nq_full,
        "clearance": 0.0,
        "robot_geoms": list(base.geometry_names),
        "self_pair_count": base.num_collision_pairs,
        "robot_only": [],
        "scenes": [],
    }

    for name, q in CONFIGS.items():
        active, pairs = check(base, q, nq_full)
        fixture["robot_only"].append(
            {"name": name, "q": q, "active": active, "pairs": pairs}
        )

    # TCP of the reach_out pose: the point a keep-out box is centred on, so
    # the tool that reaches there must hit it.
    q_reach = np.zeros(nq_full)
    q_reach[:6] = CONFIGS["reach_out"]
    tcp = robot.fkine(q_reach)[:3, 3]

    scenes = [
        {
            "name": "keepout_at_reach_tcp",
            "installation": [],
            "program": [
                shape("keepout", "box", [0.12, 0.12, 0.12], [*tcp, 0.0, 0.0, 0.0])
            ],
            "configs": ["home", "reach_out"],
        },
        {
            # Floor as an installation half-space, solid below z = 0.02:
            # the base link sits on it, so it fires at every configuration.
            "name": "floor_halfspace",
            "installation": [
                shape("floor", "plane", [0.0, 0.0, 1.0, 0.02], [0.0] * 6)
            ],
            "program": [],
            "configs": ["home", "reach_out"],
        },
        {
            # Both layers live at once and are reported independently.
            "name": "floor_and_keepout",
            "installation": [
                shape("floor", "plane", [0.0, 0.0, 1.0, 0.02], [0.0] * 6)
            ],
            "program": [
                shape("keepout", "box", [0.12, 0.12, 0.12], [*tcp, 0.0, 0.0, 0.0])
            ],
            "configs": ["home", "reach_out"],
        },
        {
            # A sphere standing 60 mm clear of the reach_out TCP: without a
            # margin it is untouched, and the margin scene below turns the
            # same geometry into a collision purely via the standoff.
            "name": "standoff_sphere_no_margin",
            "installation": [],
            "program": [
                shape(
                    "near_ball",
                    "sphere",
                    [0.04],
                    [tcp[0] + 0.15, tcp[1], tcp[2], 0.0, 0.0, 0.0],
                )
            ],
            "configs": ["reach_out"],
        },
        {
            "name": "standoff_sphere_margin",
            "installation": [],
            "program": [
                shape(
                    "near_ball",
                    "sphere",
                    [0.04],
                    [tcp[0] + 0.15, tcp[1], tcp[2], 0.0, 0.0, 0.0],
                    margin=0.12,
                )
            ],
            "configs": ["reach_out"],
        },
        {
            # A rotated capsule proves a rotation travels at all: laid on
            # its side across the reach_out TCP by Ry(pi/2).  One axis, so
            # both RPY orders agree on it — which is what the tilted bar
            # below is for.
            "name": "rotated_capsule",
            "installation": [],
            "program": [
                shape(
                    "bar",
                    "capsule",
                    [0.03, 0.30],
                    [*tcp, 0.0, math.pi / 2, 0.0],
                )
            ],
            "configs": ["home", "reach_out"],
        },
        {
            # Two axes at once, which is the only way to tell the RPY
            # orders apart: Rz(90)Rx(90) lays this bar along +X, across the
            # upper arm, where the intrinsic order would lay it along -Y
            # and clear of the whole robot.  The verdict below is therefore
            # the convention itself, not a recording of one implementation.
            "name": "tilted_bar",
            "installation": [],
            "program": [
                shape(
                    "bar",
                    "box",
                    [0.03, 0.03, 0.70],
                    [0.35, 0.0, 0.15, math.pi / 2, 0.0, math.pi / 2],
                )
            ],
            "configs": ["home", "reach_out"],
        },
    ]

    for scene in scenes:
        checker = pinokin.CollisionChecker(robot, urdf, [str(PACKAGE_DIR)])
        for layer in ("installation", "program"):
            for s in scene[layer]:
                checker.add_obstacle(
                    s["name"],
                    s["kind"],
                    s["params"],
                    pose_matrix(s["pose"]),
                    s["margin"],
                )
        cases = []
        for cfg in scene["configs"]:
            active, pairs = check(checker, CONFIGS[cfg], nq_full)
            cases.append({"name": cfg, "q": CONFIGS[cfg], "active": active, "pairs": pairs})
        fixture["scenes"].append(
            {
                "name": scene["name"],
                "installation": scene["installation"],
                "program": scene["program"],
                "pair_count": checker.num_collision_pairs,
                "cases": cases,
            }
        )

    return fixture


def main() -> None:
    for variant, (relpath, ee_frame) in VARIANTS.items():
        fixture = build(variant, relpath, ee_frame)
        out = OUT_DIR / f"{variant}.json"
        out.write_text(json.dumps(fixture, indent=2) + "\n")
        print(f"wrote {out} ({len(fixture['scenes'])} scenes)")


if __name__ == "__main__":
    main()
