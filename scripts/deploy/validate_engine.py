"""Exercise a shipped par6 wheel and daemon with no build tree in sight.

Run by ``scripts/deploy/validate-bundle.sh`` from a bare venv holding only
the release wheel, against the ``par6d`` the release bundle installed. It
asserts the two things a resolvable-but-broken shim would still fail:

* forward kinematics answers, and answers differently for different poses —
  the engine is loaded and computing, not merely importable;
* a keep-out across a move is refused, with the pair named — the collision
  half of the shim is live, and the packaged URDF and meshes came along;

and then that the installed daemon starts and answers on the wire, which is
the half the wheel cannot cover.
"""

import math
import sys

from waldoctl.shapes import Box

from par6 import Robot
from par6.client import RobotError


def main() -> int:
    robot = Robot()
    preview = robot.create_dry_run_client()

    start = preview.angles()
    if len(start) != 6:
        raise SystemExit(f"expected six joints, got {start!r}")

    # FK, twice: a pose the arm is actually in, and one it is not.
    here = preview.pose()[:3]
    reach = math.dist(here, (0.0, 0.0, 0.0))
    if not 50.0 < reach < 1500.0:
        raise SystemExit(f"flange {reach:.1f} mm from the base — engine answered nonsense")

    target = list(start)
    target[0] += 80.0
    preview.teleport(target)
    moved = math.dist(preview.pose()[:3], here)
    preview.teleport(start)
    if moved < 1.0:
        raise SystemExit(f"an 80 deg J0 turn moved the flange {moved:.3f} mm")
    print(f"engine fk: flange at {reach:.1f} mm, {moved:.1f} mm of travel across J0")

    # Collision: a box parked on the path has to stop the move.
    midway = list(start)
    midway[0] += 40.0
    preview.teleport(midway)
    x, y, z = (v / 1000.0 for v in preview.pose()[:3])
    preview.teleport(start)
    preview.set_shapes([Box(name="keepout", x=0.1, y=0.1, z=0.1, pose=(x, y, z, 0.0, 0.0, 0.0))])
    try:
        preview.move_j(target, speed=0.4)
    except RobotError as e:
        print(f"engine collision: refused [{e.code}] {e.cause}")
    else:
        raise SystemExit("a keep-out across the path did not refuse the move")

    # The daemon the bundle installed, spawned from PATH and answering.
    with robot:
        rbt = robot.create_sync_client()
        if not rbt.wait_ready(timeout=60.0):
            raise SystemExit("the installed par6d never became ready")
        angles = rbt.angles()
        if len(angles) != 6:
            raise SystemExit(f"daemon reported {angles!r}")
        print("daemon answered with", [round(a, 2) for a in angles])

    print("engine and daemon both live outside the build environment")
    return 0


if __name__ == "__main__":
    sys.exit(main())
