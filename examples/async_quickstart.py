"""Async client quickstart: drive the runtime and follow its STATUS stream.

Run from the repository root, with the shim on the loader path::

    source .ffi/env.sh
    python examples/async_quickstart.py
"""

import asyncio

from par6 import Robot


async def main() -> None:
    robot = Robot()
    robot.start()
    try:
        async with robot.create_async_client() as rbt:
            await rbt.wait_ready(timeout=10.0)
            await rbt.reset()
            await rbt.home(wait=True, timeout=200.0)

            target = list(await rbt.angles())
            target[0] += 15.0
            index = await rbt.move_j(target, speed=0.4)
            print("queued as command", index)

            # STATUS is a broadcast, not a poll: follow it while the move
            # runs rather than asking for angles in a loop.
            async for status in rbt.stream_status():
                print(f"  seq={status.seq} j0={status.angles[0]:7.2f}")
                if status.completed_index >= index:
                    break
    finally:
        robot.stop()


if __name__ == "__main__":
    asyncio.run(main())
