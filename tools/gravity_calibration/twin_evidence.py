"""Twin evidence — does the MuJoCo scene describe the arm the runtime enforces?

The Tier-2 simulator runs a hand-edited MJCF (``crates/par6-bus/
sim-assets/PAR6_MSG_scene.xml``) while the runtime's gravity model reads
the URDF; nothing else in the stack compares the two. This script does,
offline, with no arm:

1. gravity residual — G(q) from a private ``par6d --sim`` (the same
   binary that enforces it, evaluated by teleporting through the poses)
   against the scene's generalized holding force ``qfrc_bias`` at rest,
   over the vendor fixture poses plus seeded random poses inside the soft
   window; per joint: sign agreement, magnitude ratio, worst residual;
2. the vendor check — the runtime's G(q) against the vendor dynamics
   fixture (``par6-kin/tests/golden/gravity/vendor_reference.json``);
3. the mass table — URDF link masses beside MJCF body masses;
4. timestep convergence — a contact-free free fall integrated at the
   scene's 1 ms and at 0.5 and 0.25 ms; the change between refinements
   is the integration error the scene ships with;
5. damping — the measured free-decay time constant of each joint at the
   vendor's class-Y damping, the scene's override, and the value the
   config's reflected motor model (``[sim]`` G²·b) implies, so the
   override is justified by numbers rather than by feel.

Needs the ``mujoco`` Python package (dev-only, tools-only) and a par6d
binary (``PAR6D_BIN`` or PATH). Writes ``results/twin_evidence.json``.

    python tools/gravity_calibration/twin_evidence.py [--poses 20] [--seed 0]
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np
from harness import (
    REPO,
    RESULTS,
    Ledger,
    TelemetryTap,
    gravity_at,
    joint_names,
    parse_or_exit,
    place_sim_arm,
    robot_config,
    run_main,
    sim_daemon,
    write_json,
)

from par6 import config as par6_config

SCENE = REPO / "crates" / "par6-bus" / "sim-assets" / "PAR6_MSG_scene.xml"
FIXTURE = (
    REPO
    / "crates"
    / "par6-kin"
    / "tests"
    / "golden"
    / "gravity"
    / "vendor_reference.json"
)
HOLD_POSE_RAD = np.radians([0.0, -75.0, 305.0, 20.0, -30.0, 180.0])
TRAVEL_RAD = 2.0 * np.pi - 0.01
VENDOR_TOL_NM = 0.05
TIMESTEP_TOL_RAD = 5e-3
VENDOR_CLASS_Y_DAMPING = 25.0
FIXTURE_COLUMN = {
    "MSG": "tau_arm_msg_tool",
    "SSG48": "tau_arm_ssg48_tool",
    "Flange": "tau_flange_variant",
}


def urdf_masses(path: Path) -> list[tuple[str, float]]:
    text = path.read_text()
    out = []
    for name, body in re.findall(r'<link\s+name="([^"]+)">(.*?)</link>', text, re.S):
        m = re.search(r'<mass\s+value="([^"]+)"', body)
        if m:
            out.append((name, float(m.group(1))))
    return out


def decay_time_constant(mujoco, model, joint: int, damping: float) -> float:  # noqa: ANN001
    """Seconds for a 1 rad/s free spin of ``joint`` to fall to 1/e, gravity
    and contacts off, the other joints at rest; the model's actual
    inertia at the pose decides it, so it is measured, not I/d."""
    m = model
    saved = float(m.dof_damping[joint])
    m.dof_damping[joint] = damping
    d = mujoco.MjData(m)
    d.qvel[joint] = 1.0
    t, tau = 0.0, float("nan")
    for _ in range(int(2.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
        t += m.opt.timestep
        if abs(d.qvel[joint]) < 1.0 / np.e:
            tau = t
            break
    m.dof_damping[joint] = saved
    return tau


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--poses", type=int, default=20, help="random poses to add")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--scene", type=Path, default=SCENE)
    parser.add_argument(
        "--config", type=Path, default=None, help="robot TOML for the sim daemon"
    )
    parser.add_argument("--out", type=Path, default=RESULTS / "twin_evidence.json")
    parser.add_argument("--json", action="store_true", help="emit the ledger as JSON")
    args = parse_or_exit(parser, argv)
    ledger = Ledger("twin evidence")
    try:
        import mujoco
    except ImportError:
        ledger.add(
            "mujoco importable", False, "pip install mujoco (dev-only, tools-only)"
        )
        return ledger.finish(args.json)

    cfg = robot_config()
    names = joint_names(cfg)
    n = len(names)
    limits = par6_config.soft_limits_rad(cfg)
    tool = par6_config.fitted_tool_key()
    column = next((c for k, c in FIXTURE_COLUMN.items() if tool.startswith(k)), None)
    fixture = json.loads(FIXTURE.read_text()) if FIXTURE.is_file() else None
    poses: list[tuple[str, np.ndarray, np.ndarray | None]] = []
    inside = lambda q: bool(np.all((q >= limits[:, 0]) & (q <= limits[:, 1])))  # noqa: E731
    poses.append(
        ("park", np.array(cfg["robot"]["park_pose_rad"], dtype=np.float64), None)
    )
    poses.append(("hold", HOLD_POSE_RAD, None))
    skipped = 0
    if fixture is not None:
        for i, case in enumerate(fixture["cases"]):
            q = np.array(case["q"], dtype=np.float64)
            if not inside(q):
                skipped += 1
                continue
            vendor = np.array(case[column]) if column and column in case else None
            poses.append((f"fixture{i}", q, vendor))
    if skipped:
        ledger.note(
            f"{skipped} vendor fixture poses lie outside the soft window and cannot be "
            "reached by teleport; par6-kin/tests/gravity_reference.rs pins G(q) on them"
        )
    # The soft window can exceed one encoder turn (J5 reaches 7.14 rad);
    # teleport, like the encoder, travels within ±2π.
    travel = np.clip(limits, -TRAVEL_RAD, TRAVEL_RAD)
    rng = np.random.default_rng(args.seed)
    for i in range(args.poses):
        q = rng.uniform(travel[:, 0], travel[:, 1])
        poses.append((f"random{i}", q, None))

    # 1+2. the runtime's G(q) at every pose, then the scene's.
    g_par6: list[np.ndarray] = []
    with sim_daemon(config=args.config) as daemon, daemon.client() as client:
        if not client.wait_ready(timeout=10.0):
            ledger.add("sim runtime", False, "did not become ready")
            return ledger.finish(args.json)
        client.reset()
        client.wait_status(lambda s: s.enabled and s.homed, timeout=5.0)
        with TelemetryTap(client, daemon.telemetry_port) as tap:
            for _, q, _ in poses:
                place_sim_arm(client, q)
                g_par6.append(gravity_at(tap, q).g.copy())
    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    g_mj: list[np.ndarray] = []
    for _, q, _ in poses:
        mujoco.mj_resetData(model, data)
        data.qpos[:n] = q
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        g_mj.append(np.array(data.qfrc_bias[:n]))
    G6 = np.array(g_par6)
    M6 = np.array(g_mj)
    # A joint's convention is read off the correlation of the two models
    # across poses: +1 same sign, -1 flipped, and anything far from ±1 is
    # a kinematic mismatch rather than a sign. A per-pose sign vote would
    # flag legitimate zero crossings that move with the mass distribution.
    loaded = np.abs(G6) > 0.2
    per_joint = []
    for j in range(n):
        mask = loaded[:, j]
        if mask.sum() >= 3 and np.std(G6[:, j]) > 1e-6 and np.std(M6[:, j]) > 1e-6:
            corr = float(np.corrcoef(G6[:, j], M6[:, j])[0, 1])
        else:
            corr = float("nan")
        ratio = (
            float(np.median(np.abs(M6[mask, j]) / np.abs(G6[mask, j])))
            if mask.any()
            else float("nan")
        )
        worst = float(np.max(np.abs(np.abs(M6[:, j]) - np.abs(G6[:, j]))))
        per_joint.append(
            {
                "joint": names[j],
                "loaded_poses": int(mask.sum()),
                "correlation": corr,
                "sign": "n/a"
                if np.isnan(corr)
                else (
                    "same" if corr > 0.9 else ("flipped" if corr < -0.9 else "MISMATCH")
                ),
                "magnitude_ratio_mujoco_over_par6": ratio,
                "worst_magnitude_residual_nm": worst,
            }
        )
    ledger.add(
        "scene gravity sign convention is consistent per joint",
        all(p["sign"] != "MISMATCH" for p in per_joint),
        ", ".join(
            f"{p['joint']} {p['sign']}"
            + ("" if np.isnan(p["correlation"]) else f" (r {p['correlation']:+.3f})")
            for p in per_joint
        ),
    )
    ratios = [
        p["magnitude_ratio_mujoco_over_par6"]
        for p in per_joint
        if not np.isnan(p["magnitude_ratio_mujoco_over_par6"])
    ]
    ledger.add(
        "scene gravity magnitude within 15% of the runtime",
        bool(ratios) and all(abs(r - 1.0) < 0.15 for r in ratios),
        "median |MuJoCo|/|par6| per loaded joint "
        + ", ".join(
            f"{p['joint']} {p['magnitude_ratio_mujoco_over_par6']:.3f}"
            for p in per_joint
            if not np.isnan(p["magnitude_ratio_mujoco_over_par6"])
        ),
        required=False,
    )
    vendor_rows = [(g, v) for (_, _, v), g in zip(poses, g_par6) if v is not None]
    if vendor_rows:
        worst_vendor = max(float(np.max(np.abs(g - v))) for g, v in vendor_rows)
        ledger.add(
            "runtime G(q) matches the vendor fixture",
            worst_vendor < VENDOR_TOL_NM,
            f"worst |Δ| {worst_vendor:.4f} Nm over {len(vendor_rows)} poses ({tool} column)",
        )
    else:
        ledger.add(
            "runtime G(q) matches the vendor fixture",
            False,
            f"no reachable fixture pose with a column for tool {tool!r}",
            required=False,
        )

    # 3. masses
    # The runtime's model is the ARM URDF plus the fitted tool's
    # `[kinematics] mass_kg`; the URDF gripper links are for geometry.
    # The scene carries its own gripper bodies. Compare moving mass.
    urdf = urdf_masses(par6_config.urdf_path(tool))
    mjcf = [(model.body(i).name, float(model.body_mass[i])) for i in range(model.nbody)]
    arm_names = {"shoulder", "upper_arm", "elbow", "lower_arm", "wrist"}
    urdf_arm = sum(m for name, m in urdf if name in arm_names)
    tool_mass = float(
        par6_config.load_gripper_configs()
        .get(tool, {})
        .get("kinematics", {})
        .get("mass_kg", float("nan"))
    )
    runtime_moving = urdf_arm + (tool_mass if not np.isnan(tool_mass) else 0.0)
    mjcf_total = sum(m for name, m in mjcf if name not in ("world", "grasp_object"))
    ledger.add(
        "moving mass: runtime model vs scene",
        abs(runtime_moving - mjcf_total) < 0.05 * max(runtime_moving, 1e-6),
        f"arm links {urdf_arm:.3f} kg + tool {tool_mass:.3f} kg = {runtime_moving:.3f} kg "
        f"vs scene {mjcf_total:.3f} kg (object excluded)",
        required=False,
    )

    # 4. timestep convergence — contact-free free fall from a mid-range pose
    fall_from = poses[2][1] if len(poses) > 2 else poses[0][1]
    finals: dict[str, np.ndarray] = {}
    for ts in (1e-3, 5e-4, 2.5e-4):
        m2 = mujoco.MjModel.from_xml_path(str(args.scene))
        m2.opt.timestep = ts
        m2.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
        d2 = mujoco.MjData(m2)
        d2.qpos[:n] = fall_from
        for _ in range(int(round(0.3 / ts))):
            mujoco.mj_step(m2, d2)
        finals[f"{ts:g}"] = np.array(d2.qpos[:n])
    d_1_05 = float(np.max(np.abs(finals["0.001"] - finals["0.0005"])))
    d_05_025 = float(np.max(np.abs(finals["0.0005"] - finals["0.00025"])))
    ledger.add(
        "timestep converged at the scene's 1 ms",
        d_1_05 < TIMESTEP_TOL_RAD,
        f"0.3 s free fall: |Δq| 1 ms→0.5 ms {d_1_05:.2e} rad, 0.5→0.25 ms {d_05_025:.2e} rad",
    )

    # 5. damping
    sim_cfg = cfg.get("sim", {})
    motor_b = float(sim_cfg.get("motor_b_nm_s", 0.0))
    damping_rows = []
    m3 = mujoco.MjModel.from_xml_path(str(args.scene))
    m3.opt.gravity[:] = 0.0
    m3.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    for j in range(n):
        ratio = float(cfg["joints"][j]["gear_ratio"])
        reflected = motor_b * ratio * ratio
        scene = float(m3.dof_damping[j])
        damping_rows.append(
            {
                "joint": names[j],
                "vendor_class_y_nm_s": VENDOR_CLASS_Y_DAMPING,
                "scene_nm_s": scene,
                "config_reflected_nm_s": reflected,
                "tau_vendor_s": decay_time_constant(
                    mujoco, m3, j, VENDOR_CLASS_Y_DAMPING
                ),
                "tau_scene_s": decay_time_constant(mujoco, m3, j, scene),
                "tau_config_s": decay_time_constant(mujoco, m3, j, reflected),
            }
        )
    ledger.note(
        "free-decay time constants [s] (vendor / scene / config): "
        + ", ".join(
            f"{r['joint']} {r['tau_vendor_s']:.3g}/{r['tau_scene_s']:.3g}/{r['tau_config_s']:.3g}"
            for r in damping_rows
        )
    )
    ledger.note(
        "the runtime's own plant adds an idle brake of 40/s on top (IDLE_RATE·I·v): "
        "a 25 ms decay whatever the scene damping"
    )

    write_json(
        args.out,
        {
            "scene": str(args.scene.relative_to(REPO))
            if args.scene.is_relative_to(REPO)
            else str(args.scene),
            "tool": tool,
            "convention": "g_par6: torque the drive applies to hold (runtime gravity_torques); "
            "g_mujoco: qfrc_bias at rest (generalized force to hold, scene frame)",
            "poses": [
                {
                    "name": name,
                    "q_rad": q.tolist(),
                    "g_par6_nm": g.tolist(),
                    "g_mujoco_nm": m.tolist(),
                    "g_vendor_nm": v.tolist() if v is not None else None,
                }
                for (name, q, v), g, m in zip(poses, g_par6, g_mj)
            ],
            "per_joint": per_joint,
            "masses": {
                "urdf": urdf,
                "mjcf": mjcf,
                "urdf_arm_links_kg": urdf_arm,
                "config_tool_kg": tool_mass,
                "runtime_moving_kg": runtime_moving,
                "mjcf_moving_kg": mjcf_total,
            },
            "timestep": {
                "final_q_by_timestep": {k: v.tolist() for k, v in finals.items()},
                "delta_1ms_to_0p5ms_rad": d_1_05,
                "delta_0p5ms_to_0p25ms_rad": d_05_025,
            },
            "damping": damping_rows,
            "checks": [c.__dict__ for c in ledger.checks],
        },
    )
    return ledger.finish(args.json)


if __name__ == "__main__":
    run_main(main)
