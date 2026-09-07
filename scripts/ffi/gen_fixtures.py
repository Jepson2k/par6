#!/usr/bin/env python3
"""Generate kinematics/dynamics fixtures for the Rust FFI conformance tests.

Two fixture sets, one numerics stack:

1. crates/pinokin-sys/tests/fixtures/par6_flange_pin.json — the raw-shim
   conformance set (flange URDF, bare + synthetic rigid tool), generated
   with the pip `pin` package (Pinocchio's official Python bindings, the
   same library the C++ shim links):

       pip install pin==4.1.0
       python scripts/ffi/gen_fixtures.py

2. tests/golden/kinematics/par6_{flange,msg,ssg48}.json — the par6-kin
   golden set, one file per PAR6 URDF variant (assets/par6_description).
   fk / jacobian / tcp-rpy come from `pinokin` (the Python client's
   kinematics package: Robot.fkine / Robot.jacob0 / se3_rpy); gravity
   comes from `pin` RNEA at zero vel/acc (pinokin exposes no dynamics).
   The script asserts pinokin and pin agree on fk/jacobian before writing
   anything, so the golden numbers are simultaneously the client's and
   Pinocchio's.

Per case: fk pose (row-major 4x4), LOCAL_WORLD_ALIGNED jacobian
([linear; angular] rows, row-major, arm columns only), tcp pose
[x y z, r p y] (intrinsic-XYZ rpy, pinokin.se3_rpy convention) and
gravity torque for the arm joints, over N sampled arm configurations
(gripper-variant jaw joints held at zero).

The Rust tests (cargo test --features ffi, in pinokin-sys and par6-kin)
load the same URDFs through the C ABI shim and must match to 1e-9
absolute.
"""

import json
from pathlib import Path

import numpy as np
import pinocchio as pin
import pinokin

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


# --- par6-kin golden fixtures: one per URDF variant --------------------------

ARM_NQ = 6
CROSS_CHECK_TOL = 1e-12  # pinokin and pin wrap the same Pinocchio build

# (fixture name, URDF path relative to assets/par6_description, tcp frame) —
# must mirror par6-kin's GripperVariant table.
VARIANTS = [
    ("par6_flange", "URDF/par6_flange/urdf/par6_flange.urdf", "gripper"),
    ("par6_msg", "URDF/par6_msg_gripper/urdf/PAR6_MSG.urdf", "tcp"),
    ("par6_ssg48", "URDF/par6_ssg48_gripper/urdf/par6_ssg48_urdf.urdf", "tcp"),
]
ASSETS_REL = "assets/par6_description"
GOLDEN_DIR = "tests/golden/kinematics"


def golden_cases(urdf_path: Path, ee_frame: str, qs_arm: np.ndarray) -> tuple:
    """(nq_full, cases): pinokin fk/jac/tcp + pin gravity, cross-checked."""
    robot = pinokin.Robot(str(urdf_path), ee_frame)
    model = pin.buildModelFromUrdf(str(urdf_path))
    data = model.createData()
    fid = model.getFrameId(ee_frame)
    nq_full = model.nq
    assert robot.nq == nq_full
    v0 = np.zeros(model.nv)

    cases = []
    for q_arm in qs_arm:
        q = np.zeros(nq_full)
        q[:ARM_NQ] = q_arm

        T = np.ascontiguousarray(robot.fkine(q))
        J_full = np.ascontiguousarray(robot.jacob0(q))
        rpy = np.zeros(3)
        pinokin.se3_rpy(T, rpy)

        # pinokin must agree with pin before its numbers become golden.
        pin.framesForwardKinematics(model, data, q)
        T_pin = data.oMf[fid].homogeneous
        J_pin = pin.computeFrameJacobian(model, data, q, fid, pin.LOCAL_WORLD_ALIGNED)
        assert np.abs(T - T_pin).max() < CROSS_CHECK_TOL, "pinokin/pin fk mismatch"
        assert np.abs(J_full - J_pin).max() < CROSS_CHECK_TOL, "pinokin/pin jacobian mismatch"
        if nq_full > ARM_NQ:
            # Passive jaw joints must not influence the tcp frame.
            assert np.abs(J_full[:, ARM_NQ:]).max() == 0.0, "jaw columns nonzero"

        tau = pin.rnea(model, data, q, v0, v0)
        cases.append(
            {
                "q": q_arm.tolist(),
                "fk": T.reshape(-1).tolist(),               # row-major 4x4
                "tcp": [*T[:3, 3].tolist(), *rpy.tolist()],  # [x y z, r p y]
                "jac": J_full[:, :ARM_NQ].reshape(-1).tolist(),  # row-major 6x6
                "tau": tau[:ARM_NQ].tolist(),
            }
        )
    return nq_full, cases


def write_golden_fixtures() -> None:
    rng = np.random.default_rng(SEED)
    qs_arm = rng.uniform(-np.pi, np.pi, size=(N_CONFIGS, ARM_NQ))
    out_dir = REPO_ROOT / GOLDEN_DIR
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, urdf_rel, ee_frame in VARIANTS:
        urdf_path = REPO_ROOT / ASSETS_REL / urdf_rel
        nq_full, cases = golden_cases(urdf_path, ee_frame, qs_arm)
        fixture = {
            "pin_version": pin.__version__,
            "urdf": f"{ASSETS_REL}/{urdf_rel}",
            "ee_frame": ee_frame,
            "nq_full": nq_full,
            "seed": SEED,
            "cases": cases,
        }
        out = out_dir / f"{name}.json"
        out.write_text(json.dumps(fixture, indent=1) + "\n")
        print(f"wrote {out} (nq_full={nq_full}, {len(cases)} configs)")


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

    write_golden_fixtures()


if __name__ == "__main__":
    main()
