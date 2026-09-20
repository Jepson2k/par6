"""Identify what the arm is holding, from the torque its wrist carries.

The runtime's gravity model only compensates for what it knows about.
Declare a payload you know the mass of with ``set_payload``; measure one
you do not with ``estimate_payload``, which swings the WRIST where the
arm already stands and solves for the load at the end of the chain.
Nothing below the wrist moves, so a pick is not disturbed and the run
takes seconds.

Run from the repository root::

    pixi run python examples/payload_estimate.py

Against a simulated runtime the torques are the model's own, so the
estimate comes back at zero — the arm really is carrying nothing. On
hardware, close the gripper on a part first.
"""

from par6 import Robot

# Below this share, the poses did not fix a parameter and the ridge
# supplied it; the number is a starting guess, not a measurement.
MEASURED = 0.5


def main() -> None:
    with Robot() as robot:
        rbt = robot.create_sync_client()
        rbt.reset()
        rbt.home(wait=True)

        # declare=False measures without touching the runtime's payload,
        # which is what you want while checking whether a pick succeeded.
        # The default declares the result, so the gravity model carries
        # the part from the next tick.
        found = rbt.estimate_payload(declare=False)

        print(f"mass        {found.mass:8.4f} kg")
        print("com         [" + ", ".join(f"{v:.4f}" for v in found.com) + "] m")
        print(f"poses       {found.poses}")
        print(f"residual    {found.rms_nm:8.4f} Nm  (unloaded {found.rms_unloaded_nm:.4f})")

        labels = ("mass", "m*cx", "m*cy", "m*cz")
        weak = [
            name
            for name, share in zip(labels, found.determined)
            if share <= MEASURED
        ]
        if weak:
            # A wrist with no room to swing reads near zero here. The
            # fit still returns, but those numbers are the ridge's, not
            # the arm's — widen `spread` or move somewhere with room.
            print("determined: all but", ", ".join(weak))
        else:
            print("determined: every parameter")


if __name__ == "__main__":
    main()
