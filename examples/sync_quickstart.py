"""Sync client quickstart: spawn a simulated runtime and drive it.

Run from the repository root, with the shim on the loader path::

    source .ffi/env.sh
    python examples/sync_quickstart.py
"""

from par6 import Robot


def main() -> None:
    with Robot() as robot:
        rbt = robot.create_sync_client()
        print("ping:", rbt.ping())

        # A fresh runtime is DISABLED and un-referenced: reset() enables the
        # drives, home() establishes the position reference. Planned motion
        # is refused before both.
        rbt.reset()
        rbt.home(wait=True)

        print("angles:", [round(a, 2) for a in rbt.angles()])
        print("pose:", [round(v, 2) for v in rbt.pose()])

        target = list(rbt.angles())
        target[0] += 20.0
        rbt.move_j(target, speed=0.4, wait=True)
        print("after move:", [round(a, 2) for a in rbt.angles()])


if __name__ == "__main__":
    main()
