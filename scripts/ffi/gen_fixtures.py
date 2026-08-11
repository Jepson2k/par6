#!/usr/bin/env python3
"""Generate kinematics/dynamics fixtures for the pinokin-sys conformance test.

Uses the pip `pin` package (Pinocchio's official Python bindings — the same
numerics stack the C++ shim links) as the independent reference:

    pip install pin==4.1.0
    python scripts/ffi/gen_fixtures.py

Writes JSON to crates/pinokin-sys/tests/fixtures/par6_flange_pin.json:
fk pose (row-major 4x4), LOCAL_WORLD_ALIGNED jacobian ([linear; angular],
row-major 6xnq) and gravity torque (RNEA at zero vel/acc) for N sampled
configurations — once for the bare flange model and once with a rigid tool
attached (transform + mass/com/inertia), mirroring spec/RT.md's gravity
requirement (arm + active gripper tool link).

The Rust test (cargo test --features ffi) loads the same URDF through the
C ABI shim and must match to 1e-9 absolute.
"""

import json
from pathlib import Path

import numpy as np
import pinocchio as pin

REPO_ROOT = Path(__file__).resolve().parents[2]
URDF_REL = "assets/par6_description/URDF/par6_flange/urdf/par6_flange.urdf"
EE_FRAME = "gripper"
N_CONFIGS = 20
SEED = 42

# Arbitrary but fixed tool: rotated + offset frame, off-axis COM, full inertia.
TOOL = {
    # T_ee_tool: Rz(0.3) then translate [0.01, -0.02, 0.05] (row-major 4x4)
    "transform": None,  # filled below
    "mass": 0.45,
    "com": [0.005, -0.002, 0.03],
    # (Ixx, Ixy, Iyy, Ixz, Iyz, Izz) about COM, ee-frame axes
    "inertia": [2.1e-4, -1.5e-5, 1.8e-4, 8.0e-6, -4.0e-6, 2.5e-4],
}


def rz(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def skew(v: np.ndarray) -> np.ndarray:
    return np.array(
        [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]]
    )


def tool_transform() -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rz(0.3)
    T[:3, 3] = [0.01, -0.02, 0.05]
    return T


def inertia6_to_matrix(i6: list) -> np.ndarray:
    ixx, ixy, iyy, ixz, iyz, izz = i6
    return np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])


def build_model(urdf_path: Path, with_tool: bool) -> pin.Model:
    model = pin.buildModelFromUrdf(str(urdf_path))
    if with_tool and TOOL["mass"] > 0.0:
        fid = model.getFrameId(EE_FRAME)
        frame = model.frames[fid]
        inertia = pin.Inertia(
            TOOL["mass"],
            np.array(TOOL["com"]),
            inertia6_to_matrix(TOOL["inertia"]),
        )
        # Same call the shim makes: tool inertia given in ee-frame coords,
        # re-expressed at the parent joint via the frame placement.
        model.appendBodyToJoint(frame.parentJoint, inertia, frame.placement)
    return model


def compute_cases(model: pin.Model, qs: np.ndarray, T_tool) -> list:
    data = model.createData()
    fid = model.getFrameId(EE_FRAME)
    v0 = np.zeros(model.nv)
    cases = []
    for q in qs:
        pin.framesForwardKinematics(model, data, q)
        T_ee = data.oMf[fid].homogeneous
        J = pin.computeFrameJacobian(model, data, q, fid, pin.LOCAL_WORLD_ALIGNED)
        if T_tool is not None:
            T = T_ee @ T_tool
            # J_v_tool = J_v_ee - skew(R_ee * p_tool) * J_w
            r = T_ee[:3, :3] @ T_tool[:3, 3]
            J = J.copy()
            J[:3] -= skew(r) @ J[3:]
        else:
            T = T_ee
        tau = pin.rnea(model, data, q, v0, v0)
        cases.append(
            {
                "q": q.tolist(),
                "fk": T.reshape(-1).tolist(),          # row-major 4x4
                "jac": J.reshape(-1).tolist(),         # row-major 6 x nq
                "tau": tau.tolist(),
            }
        )
    return cases


def main() -> None:
    urdf_path = REPO_ROOT / URDF_REL
    T_tool = tool_transform()
    TOOL["transform"] = T_tool.reshape(-1).tolist()

    model = build_model(urdf_path, with_tool=False)
    assert model.nq == 6, f"expected nq=6, got {model.nq}"
    rng = np.random.default_rng(SEED)
    qs = rng.uniform(-np.pi, np.pi, size=(N_CONFIGS, model.nq))

    fixture = {
        "pin_version": pin.__version__,
        "urdf": URDF_REL,
        "ee_frame": EE_FRAME,
        "seed": SEED,
        "tool": TOOL,
        "cases_flange": compute_cases(model, qs, None),
        "cases_tool": compute_cases(build_model(urdf_path, True), qs, T_tool),
    }

    out = REPO_ROOT / "crates/pinokin-sys/tests/fixtures/par6_flange_pin.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(fixture, indent=1) + "\n")
    print(f"wrote {out} (pin {pin.__version__}, {N_CONFIGS} configs x 2 models)")


if __name__ == "__main__":
    main()
