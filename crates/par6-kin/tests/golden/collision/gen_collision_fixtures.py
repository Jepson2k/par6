#!/usr/bin/env python3
"""Regenerate the collision golden fixtures from the Python reference stack.

The expected verdicts and pair sets come from ``pinokin.CollisionChecker`` —
the same Pinocchio+coal stack the Waldo Commander client checks against — so
``crates/par6-kin/tests/golden_collision.rs`` is a genuine cross-stack
conformance test rather than a recording of this crate's own output.

The *cases* are derived from the requirement, not from either
implementation: the home pose and a reachable outstretched pose must be
clear; a shoulder folded back into the base and a wrist folded onto the
forearm are self-collisions the arm can actually reach (every joint value
stays inside PAR6.toml's +/-2.8647 rad soft limits); a keep-out box centred
on a configuration's TCP must be hit by the tool that reaches into it.

Needs ``pip install pinokin numpy`` (the same release the client uses).

    python3 crates/par6-kin/tests/golden/collision/gen_collision_fixtures.py
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pinokin

REPO = Path(__file__).resolve().parents[5]
ASSETS = REPO / "assets" / "par6_description"
PACKAGE_DIR = ASSETS / "URDF"
OUT_DIR = Path(__file__).resolve().parent

# Soft joint limit from config/PAR6.toml — every fixture q stays inside it.
SOFT_LIMIT = 2.8647335

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
    """R = Rx(rx) @ Ry(ry) @ Rz(rz) — waldoctl's pose convention."""

    def rot(axis: int, a: float) -> np.ndarray:
        c, s = math.cos(a), math.sin(a)
        m = np.eye(3)
        i, j = [(1, 2), (2, 0), (0, 1)][axis]
        m[i, i] = c
        m[j, j] = c
        m[i, j] = -s
        m[j, i] = s
        return m

    return rot(0, rx) @ rot(1, ry) @ rot(2, rz)


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
    "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "reach_out": [0.4, 0.3, -0.6, 0.2, 0.5, -0.3],
    "shoulder_into_base": [0.0, -1.57, 0.0, 0.0, 0.0, 0.0],
    "wrist_onto_forearm": [0.0, 2.0, -2.8, 0.0, -2.5, 0.0],
    "folded_over_base": [0.0, 2.5, -2.8, 0.0, 0.0, 0.0],
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
        assert all(abs(v) <= SOFT_LIMIT for v in q), f"{name} exceeds soft limits"

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
            # A rotated capsule proves the RPY convention travels: laid on
            # its side across the reach_out TCP by Ry(pi/2).
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
