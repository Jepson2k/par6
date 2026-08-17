"""Keep-out shapes: the offline preview and the runtime agree.

Places a keep-out across a move, shows the preview refusing it before any
runtime is involved, then shows the runtime refusing the same move — same
error code, same colliding pair.

Run from the repository root, with the shim on the loader path::

    source .ffi/env.sh
    python examples/keepout_preview.py
"""

from waldoctl.shapes import Box

from par6 import Robot
from par6.client import RobotError


def main() -> None:
    robot = Robot()
    preview = robot.create_dry_run_client()

    start = preview.angles()
    target = list(start)
    target[0] += 80.0

    # Park a 10 cm box on the TCP position half way along the move.
    midway = list(start)
    midway[0] += 40.0
    preview.teleport(midway)
    x, y, z = (v / 1000.0 for v in preview.pose()[:3])
    keepout = Box(name="keepout", x=0.1, y=0.1, z=0.1, pose=(x, y, z, 0.0, 0.0, 0.0))
    preview.teleport(start)

    preview.set_shapes([keepout])
    try:
        preview.move_j(target, speed=0.4)
        print("preview: allowed (the keep-out missed the path)")
        return
    except RobotError as e:
        print(f"preview refused: [{e.code}] {e.cause}")

    with robot as running:
        rbt = running.create_sync_client()
        rbt.reset()
        rbt.teleport(start)
        rbt.set_shapes([keepout])
        try:
            rbt.move_j(target, speed=0.4, wait=True)
            print("runtime: allowed — preview and runtime DISAGREE")
        except RobotError as e:
            print(f"runtime refused: [{e.code}] {e.cause}")

        # Clear the world and the same move runs.
        rbt.set_shapes([])
        rbt.move_j(target, speed=0.4, wait=True)
        print("cleared:", [round(a, 2) for a in rbt.angles()])


if __name__ == "__main__":
    main()
