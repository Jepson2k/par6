# Offline simulation scenarios

`DryRunRobotClient.simulate(max_seconds, scenario=...)` replays the submitted
command stream through the native planner, control loop, and simulated bus.
It captures initial joints, referencing, gripper calibration, TCP/tool,
profile, payload, completion policy, speed/pause state, and program world
separately from the planning result. Repeated simulation leaves the planning
client unchanged.

```python
from par6.client.dry_run_client import DryRunRobotClient

rbt = DryRunRobotClient()
rbt.delay(1)
record = rbt.simulate(2, scenario={
    "seed": 73,
    "observation_delay_s": 0.012,
    "encoder_noise_ticks": 20,
})
print(record.stop, record.duration_s, record.digest.hex())
```

The scenario clock starts after private model initialization. Inputs use
simulated seconds and are validated before activation:

- `observation_delay_s`: host telemetry delay in `[0, 1]`.
- `encoder_noise_ticks`: bounded integer host observation noise in `[0, 16384]`;
  the unsigned 64-bit `seed` makes it repeatable. Local driver feedback is unchanged.
- `dropout`: `{ "start_s": 0.2, "duration_s": 0.5 }` suppresses replies.
- `driver_fault`: `{ "at_s": 0.2, "node": 0, "kind": "encoder" }` latches a
  configured driver's encoder fault. Other kinds are temperature, vbus, driver,
  velocity, current, and estop.
- `supply_loss`: `{ "at_s": 0.2, "decay_s": 0.5 }` linearly reduces assumed
  supply availability to zero. A zero decay means immediate loss.

Other event times must be finite in `[0, 3600]`; dropout duration must be
positive. Unknown fields and unconfigured fault nodes are rejected. Delayed
replies use a fixed-capacity buffer; overflow is counted as dropped frames.
The runtime's existing freshness and fault logic consumes these observations.

`max_seconds` is a positive simulated-time limit up to 3600 seconds, including
ticks used by control commands. It does not bound execution of the Python
program that constructs the command list. Waldo Commander's case runner supplies
a separate worker-process wall deadline for that.

Queued motion, tool actions, tool/TCP changes, profile/payload/completion
settings, world edits, and speed/pause controls are replayed at command
boundaries. Unsupported system or streaming commands return a structured
replay refusal. A refusal ends the run; later commands have no recorded rows.
Attachment declarations must match the replay's reference context. A tool
variant change invalidates previous attachments.

Program-world changes update both collision checking and the physical scene.
Free objects contain NaN pose rows before creation and after removal; consumers
must treat those as absent samples. An attached shape follows the flange and
remains independent of the TCP. It is declared collision geometry, without a
physical mass or sensed grasp. Free-body contact and gripper behavior remain
separate physics observations.

## Model assumptions

The URDF/MJCF geometry, inertials, transmission data, and configured drive gains
are source model inputs. They have not been characterized by these tests.
Contact friction, actuator approximations, and perturbation inputs are also
assumptions. Reproducible output checks the implementation against its model;
it does not validate that model against a particular assembled robot.

`[sim].powered_support_nm` replaces `holding_friction_nm`. These nonnegative,
finite values model assumed powered load support, not passive holding torque
or motor brakes. The old field is rejected rather than silently reinterpreted.
Released drivers have no static powered support even while their electronics
have power. Supply loss scales away motor torque and powered idle damping too.
Gravity, reflected inertia, configured passive Coulomb/viscous friction, and
joint limits remain. The arm can collapse; PAR6's capacitor-bank discharge and
electrical shutdown circuitry are not modeled by the linear envelope.

The drive-loop approximation has known settling limitations, documented beside
`FW_LOOP_DT` in `crates/par6-bus/src/sim/driver.rs`. Do not infer millimetre-level
hardware accuracy from this simulator or retune hardware gains to fit it.
