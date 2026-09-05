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
    except RobotError as e:
        previewed = e
        print(f"preview refused: [{e.code}] {e.cause}")
    else:
        raise SystemExit(
            "the preview allowed the move: the keep-out is not across the path"
        )

    # The client is closed on the way out: it owns a background event loop
    # and the runtime's status stream, and leaving those to interpreter
    # shutdown races the loop's teardown.
    with robot as running, running.create_sync_client() as rbt:
        rbt.reset()
        # Teleport is unacked: the runtime applies it on its next tick and
        # only the status broadcast says the arm landed there, referenced.
        rbt.teleport(start)
        landed = rbt.wait_status(
            lambda s: (
                s.homed and all(abs(a - b) < 0.5 for a, b in zip(s.angles, start))
            ),
            timeout=5.0,
        )
        if not landed:
            raise SystemExit("the sim arm never landed on the start pose")
        rbt.set_shapes([keepout])
        try:
            rbt.move_j(target, speed=0.4, wait=True)
        except RobotError as e:
            print(f"runtime refused: [{e.code}] {e.cause}")
            if e.code != previewed.code:
                raise SystemExit(
                    f"preview refused with {previewed.code} but the runtime "
                    f"refused with {e.code}"
                ) from e
            for refusal in (previewed, e):
                if "shape:keepout" not in refusal.cause:
                    raise SystemExit(
                        f"a refusal did not name the keep-out: {refusal.cause!r}"
                    )
        else:
            raise SystemExit("the runtime ran the move the preview refused")

        # Clear the world and the same move runs.
        rbt.set_shapes([])
        rbt.move_j(target, speed=0.4, wait=True)
        print("cleared:", [round(a, 2) for a in rbt.angles()])


if __name__ == "__main__":
    main()
