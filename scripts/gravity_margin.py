#!/usr/bin/env python3
"""Homing gravity margin: G(q) against the torque a joint can actually make.

During homing the driver's current limit is cut to `homing_current_ma`
(crates/par6-rt/src/homing.rs), so a joint's whole torque budget is

    I_home * kt * G * eta  (drive ceiling)
      + G * motor_tc_nm    (reflected motor Coulomb friction)   [Nm at the joint]

and gravity compensation is gated on `homed`, so none is applied. This prints
the worst |G(q)| over the reachable J1xJ2 box against that budget, which is
what decides whether a weight-modelling simulator plant can home at all.

Run under the pixi env (it needs pinocchio):  pixi run python scripts/gravity_margin.py
"""

import pathlib
import tomllib

import numpy as np
import pinocchio as pin

ROOT = pathlib.Path(__file__).resolve().parent.parent
URDF = ROOT / "assets/par6_description/URDF/par6_msg_gripper/urdf/PAR6_MSG.urdf"
ARM_URDF = ROOT / "assets/par6_description/URDF/par6_flange/urdf/par6_arm.urdf"


def budget(joints, hom, tc, k):
    j = joints[k]
    g = j.get("dynamics_gear_ratio") or j["gear_ratio"]
    return hom[k]["current_ma"] / 1000 * j["kt_nm_a"] * g * j["gear_efficiency"] + g * tc


def main() -> None:
    cfg = tomllib.loads((ROOT / "config/PAR6.toml").read_text())
    joints, hom = cfg["joints"], cfg["homing"]["joints"]
    tc = cfg["sim"]["motor_tc_nm"]
    park = np.array(cfg["robot"]["park_pose_rad"])
    lo = [j["limits"]["hard_min_rad"] for j in joints]
    hi = [j["limits"]["hard_max_rad"] for j in joints]

    for label, urdf in (("arm-only", ARM_URDF), ("with MSG", URDF)):
        m = pin.buildModelFromUrdf(str(urdf))
        q = np.zeros(m.nq)
        q[:6] = park
        g = pin.computeGeneralizedGravity(m, m.createData(), q)[:6]
        print(f"{label:9s} G(park) J1={g[1]:+.3f} J2={g[2]:+.3f} max={np.abs(g).max():.3f} Nm")

    m = pin.buildModelFromUrdf(str(URDF))
    d = m.createData()
    n = 121
    worst = np.zeros(6)
    for a in np.linspace(lo[1], hi[1], n):
        for b in np.linspace(lo[2], hi[2], n):
            q = np.zeros(m.nq)
            q[:6] = park
            q[1], q[2] = a, b
            np.maximum(worst, np.abs(pin.computeGeneralizedGravity(m, d, q)[:6]), out=worst)

    print(f"\n{'J':>2} {'worst|G|':>9} {'budget':>8} {'margin':>8}")
    for k in range(6):
        b = budget(joints, hom, tc, k)
        flag = "  DEFICIT" if worst[k] > b else ""
        print(f"{k:>2} {worst[k]:9.3f} {b:8.3f} {b - worst[k]:8.3f}{flag}")


if __name__ == "__main__":
    main()
