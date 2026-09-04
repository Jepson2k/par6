"""The MuJoCo scene against the URDF the rest of the stack trusts.

``crates/par6-bus/sim-assets/PAR6_MSG_scene.xml`` is hand-edited, and the
runtime's kinematics and gravity read the URDF. Nothing keeps the two in
step, so a scene edit — a body mass nudged while tuning contact, a link
that never got the URDF's revision — silently makes the simulator a
different arm from the one the runtime plans for, and every sim result
after that measures the wrong robot.

The runtime's own gravity is pinned separately against the vendor
reference in ``crates/par6-kin/tests/gravity_reference.rs``. What is left
unguarded, and what this covers, is whether the scene still describes the
same mass as that model.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from par6 import config as par6_config

SCENE = (
    Path(__file__).resolve().parents[2]
    / "crates"
    / "par6-bus"
    / "sim-assets"
    / "PAR6_MSG_scene.xml"
)

# The runtime's moving mass is the arm's own links plus the fitted tool's
# configured mass: the URDF's gripper links carry geometry rather than the
# tool the runtime plans with, and the scene models its jaws itself.
ARM_LINKS = {"shoulder", "upper_arm", "elbow", "lower_arm", "wrist"}
# Scene bodies that are furniture, not arm.
SCENE_NON_ARM = {"world", "grasp_object"}


def _urdf_link_masses(path: Path) -> dict[str, float]:
    root = ET.parse(path).getroot()
    out: dict[str, float] = {}
    for link in root.iter("link"):
        mass = link.find("./inertial/mass")
        name = link.get("name")
        if mass is not None and name is not None and mass.get("value") is not None:
            out[name] = float(mass.get("value", "0"))
    return out


def _scene_body_masses(path: Path) -> dict[str, float]:
    root = ET.parse(path).getroot()
    out: dict[str, float] = {}
    for body in root.iter("body"):
        inertial = body.find("inertial")
        name = body.get("name")
        if inertial is not None and name is not None:
            out[name] = float(inertial.get("mass", "0"))
    return out


def test_the_scene_carries_the_same_moving_mass_as_the_urdf() -> None:
    assert SCENE.is_file(), f"missing sim scene: {SCENE}"

    tool = par6_config.fitted_tool_key()
    urdf = _urdf_link_masses(par6_config.urdf_path(tool))
    assert urdf, "no <link><inertial><mass> found in the URDF"
    arm_mass = sum(m for name, m in urdf.items() if name in ARM_LINKS)
    assert arm_mass > 0.0, f"no arm links matched {sorted(ARM_LINKS)} in {sorted(urdf)}"

    tool_mass = float(
        next(
            (
                g["kinematics"]["mass_kg"]
                for g in par6_config.config().grippers()
                if g["key"] == tool
            ),
            0.0,
        )
    )
    runtime_moving = arm_mass + tool_mass

    scene = _scene_body_masses(SCENE)
    assert scene, "no body masses found in the scene"
    scene_moving = sum(m for name, m in scene.items() if name not in SCENE_NON_ARM)

    # The band covers the scene modelling its jaws as their own bodies
    # against the tool's single configured mass; a real drift — a link
    # left at an older revision — moves it much further than this.
    assert abs(runtime_moving - scene_moving) < 0.10 * runtime_moving, (
        f"the scene and the URDF describe different arms: runtime "
        f"{arm_mass:.3f} kg of links + {tool_mass:.3f} kg of tool = "
        f"{runtime_moving:.3f} kg, scene {scene_moving:.3f} kg"
    )
