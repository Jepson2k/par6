# par6

> **Alpha.** APIs, wire protocol, and configs all still move between minor
> versions, and hardware bring-up is in progress
> ([#31](https://github.com/Jepson2k/par6/issues/31)).

PAR6 robot backend for [Waldo Commander](https://github.com/Jepson2k/Waldo-Commander):
a **Rust real-time runtime** (`par6d`) that replaces Source Robotics' RCB-Runtime on the
control box, a **Rust client library** (`par6-client`), and a **Python package**
(`python/par6`) — a thin binding over the Rust engine — implementing the
[waldoctl](https://github.com/Jepson2k/waldoctl) backend contracts.

`par6d --sim` runs anywhere — no hardware, no CAN interface, no root — so everything
below works on a laptop and in CI.

## Table of contents

- [Installation](#installation)
- [Quickstart](#quickstart)
- [Architecture](#architecture)
- [Control loop internals](#control-loop-internals)
- [Command system](#command-system)
- [Adding a new command](#adding-a-new-command)
- [Motion profiles](#motion-profiles)
- [Collision world](#collision-world)
- [Kinematics and tools](#kinematics-and-tools)
- [The Pinocchio shim](#the-pinocchio-shim)
- [Ports and environment variables](#ports-and-environment-variables)
- [Development setup](#development-setup)
- [Deploying to the control box](#deploying-to-the-control-box)
- [Known divergences from parol6](#known-divergences-from-parol6)
- [Safety notes](#safety-notes)
- [License](#license)

## Installation

par6 links a Pinocchio C-ABI shim, so that gets built once before anything else.

```bash
scripts/ffi/setup.sh             # once — builds the shim into .ffi/
cargo build -p par6d --release
pip install -e "python[dev]"
```

A checkout that has run `setup.sh` needs no environment for either step: the build
scripts find the shim in `.ffi/shim` and libmujoco in `.ffi/env`, and `par6d` and the
Python extension carry the directories they load from as rpaths, so both run from any
shell. `source .ffi/env.sh` is still the way to point at libraries installed elsewhere
(`PAR6_SHIM_LIB_DIR`, `PAR6_MUJOCO_LIB_DIR`), to cross-build, and to run
`cargo test --workspace`, whose test binaries outside par6-bus and par6d reach
libmujoco only through its `LD_LIBRARY_PATH`.

`setup.sh` picks its compile parallelism from available RAM (one shim compile job
peaks near 4 GB; a swapless small box overcommitting that livelocks rather than
failing). Set `CMAKE_BUILD_PARALLEL_LEVEL` to override it; `.ffi/env.sh` exports the
same figure as `CARGO_BUILD_JOBS`.

Installing just the client, which is what Waldo Commander's `[par6]` extra does:

```bash
export PAR6_SHIM_LIB_DIR=/path/to/.ffi/shim/lib     # a git install has no checkout to find these in
export PAR6_MUJOCO_LIB_DIR=/path/to/.ffi/env/lib
pip install "par6 @ git+https://github.com/Jepson2k/par6.git@main#subdirectory=python"
```

The package is a maturin build: pip compiles the `par6-py` extension (the engine's
client + preview), so a source install needs the Rust toolchain and the shim from
`scripts/ffi/setup.sh`. Prebuilt wheels that need neither are the wheel CI's job.
That gives you the client, the offline preview and the kinematics — but **not** the
`par6d` binary. `Robot().start()` spawns `$PAR6D_BIN`, or `par6d` on `PATH`, so a
client-only install has nothing to spawn until either the workspace above is built or
a runtime is already listening — which is the normal case on the control box, where
Waldo Commander, this client and `par6d` all run on the same machine and the runtime
is a systemd service. Shipping a per-platform runtime wheel is
[#33](https://github.com/Jepson2k/par6/issues/33).

Deploying to a control box (Raspberry Pi 5, aarch64, PREEMPT_RT) is covered in
[Deploying to the control box](#deploying-to-the-control-box).

## Quickstart

### The `Robot` façade — spawn a simulated runtime and drive it

```python
from par6 import Robot

with Robot() as robot:              # spawns `par6d --sim`, waits for PAR6D_READY
    rbt = robot.create_sync_client()
    rbt.reset()                     # clear errors, enable the drives
    rbt.home(wait=True)
    rbt.move_j([0, -90, 180, 0, 0, 180], speed=0.5, wait=True)
    print(rbt.angles(), rbt.pose())
```

`Robot()` claims exclusive ownership of its address: a runtime already answering PING
there is a hard failure, not a silent attach. To use one somebody else started, check
`Robot.is_available()` and construct a client directly.

### Attaching to a runtime that is already up

The command port is **6001** — not Waldo Commander's parol6 default of 5001.

```python
from par6.client import RobotClient

with RobotClient(host="127.0.0.1", port=6001) as rbt:   # the box's hostname from another machine
    print(rbt.angles())
```

### Async client

```python
import asyncio
from par6.client import AsyncRobotClient

async def main():
    async with AsyncRobotClient(host="127.0.0.1", port=6001) as rbt:
        await rbt.reset()
        await rbt.move_l([300, 0, 350, 180, 0, 180], speed=0.4, wait=True)
        async for status in rbt.stream_status():
            print(status.angles)
            break

asyncio.run(main())
```

The sync client is a thin façade over the async one running on a background event
loop; calling it from inside a running loop raises rather than deadlocking.

### Offline preview — no runtime at all

```python
from par6 import Robot

preview = Robot().create_dry_run_client()
result = preview.move_j([0, -90, 180, 0, 0, 180], speed=0.5)
print(result.duration, result.tcp_poses.shape)
```

The preview runs the same trajectory code, the same limits and the same collision
world as the runtime, and refuses what the runtime refuses — with the same error
codes — so an editor shows the failure before the arm does.

## Architecture

```
Waldo Commander (NiceGUI frontend, unchanged)
  └─ python/par6 — waldoctl Robot + AsyncRobotClient + sync facade + dry-run preview
       │   (a thin shim over crates/par6-client + the par6d preview, via crates/par6-py)
       │ protocol v3: UDP msgpack commands · binary status broadcast
  par6d (single Rust binary; `par6d --sim` runs anywhere, including CI)
   ├─ command plane (tokio): validation/gating, queue, index allocator,
   │    push completion, status broadcaster
   ├─ planner thread: TOPPRA (FFI) planned moves · rsruckig streaming/blending ·
   │    trapezoid, plus the plan-time collision gate and the enablement probe
   ├─ housekeeping thread (soft-RT): cart-jog ramps, enable retries, flashing exits
   └─ RT thread (SCHED_FIFO 99, alloc-free): fixed-rate tick (`tick_dt_s`, shipped
        250 Hz) — CAN RX → state → gravity comp G(q) → mode dispatch → CAN TX →
        state snapshot
   bus backends: SocketCAN (Spectral/STEPFOC) | closed-loop dynamics sim (Pinocchio ABA)
```

There is one numerics stack. Kinematics, dynamics and collision run on **Pinocchio and
coal through the repo's C-ABI shim** (`cpp/`, see [The Pinocchio shim](#the-pinocchio-shim)),
inverse kinematics is the analytic OPW closed form derived from the URDF at load, and
TOPPRA retimes planned paths. The Python package holds none of it: `par6._par6` (the
`par6-py` crate) binds the same `Kin`, `Collision`, config loader and dry-run engine the
daemon runs, so a preview cannot disagree with the runtime — it *is* the runtime's code.

### Repository layout

| Path | Contents |
|---|---|
| `crates/par6-proto` | protocol v3 codec — **single source of truth**; the Python constants are generated from it |
| `crates/par6-config` | robot / gripper / homing TOML config |
| `crates/par6-kin` | Pinocchio FFI: FK, Jacobian, gravity, IK; coal collision world (self-pairs + installation/program keep-out layers) |
| `crates/par6-motion` | TOPPRA + rsruckig + trapezoid, jog ramps, completion policies |
| `crates/par6-bus` | `DriverBus` trait, Spectral CAN codec, SocketCAN + simulator backends |
| `crates/par6-rt` | RT tick loop, mode dispatch, homing FSM, error latching, e-stop |
| `crates/par6-server` | UDP command plane, status broadcast, collision-world layers |
| `crates/par6-client` | the client library: command round-trips, retries/dedup, status subscription |
| `crates/par6d` | the runtime binary: config load, thread spawn/wiring, planner, RT bridge — plus the offline preview harness |
| `crates/par6-py` | the `par6._par6` Python extension (PyO3 over par6-client + the preview) |
| `cpp/` | the Pinocchio/coal/TOPPRA C-ABI shim |
| `python/` | the `par6` pip package (waldoctl backend) |
| `python/par6/panel/` | the control box front panel service (`par6-panel`) and the preflight check (`par6-preflight`) |
| `assets/` | PAR6 URDF, SRDF and meshes from Source Robotics — see `assets/NOTICE` |

### One plane per deadline

`par6d` is one process with four planes, one per distinct deadline. Work is assigned to
a plane by its **timing class**, never by feature:

| Class | What it means | A miss costs |
|---|---|---|
| hard real-time | bounded, data-independent cost; runs every tick | a `LOOP_CRITICAL` latch — the controller disables |
| soft real-time | bounded cost, but nothing physical depends on it landing this tick | a late effect nobody can see |
| unbounded | cost depends on the data: path length, solver iterations, a collision walk | nothing — there is no deadline |

**The rule:** a piece of work belongs in the fastest class whose *worst case* can meet
that class's deadline, and no faster. It is decided per function, from the function's
worst case, never from how fast it usually runs.

Two corollaries do most of the work:

1. **Bounded work never shares a thread with unbounded work.** It would inherit the
   unbounded worst case, and the deadline it was placed for stops holding. That is what
   a plane *is*; a thread that mixes classes is not a plane.
2. **There are exactly as many planes as there are distinct deadlines.** A plane costs a
   queue and an ownership rule, so each one has to earn its place with one sentence
   naming its deadline and what a miss costs. If the sentence cannot be written, the
   plane is wrong.

The four sentences:

| Thread | Class | Deadline | A miss costs |
|---|---|---|---|
| `par6d-rt` | hard | every tick | `LOOP_CRITICAL`; the arm stops |
| `par6d-housekeeping` | soft | one tick | a cart-jog ramp or an enable retry lands a tick late |
| command plane (tokio) | soft | the status period | a datagram is acked late; a STATUS frame is skipped |
| `par6d-planner` | none | — | a queued move starts later |

(`par6d-tee` is fan-out plumbing for the snapshot slot, not a plane: it has no work of
its own.)

The RT thread runs `SCHED_FIFO` priority 99 pinned to one core and allocates nothing
after init. The command plane parses datagrams, validates and gates them, allocates
queue indices and broadcasts STATUS — every step bounded and data-independent, which is
what lets it keep a deadline at all. The planner thread owns the `Planner`: IK seeding,
TOPPRA retiming, the plan-time collision walk and the enablement probe, all
data-dependent, none with a deadline. It takes requests on one channel and answers on
another, publishes a latest-wins report (enablement, collision state, warnings, the
in-flight and queued durations) that STATUS and the queries read, and runs at most one
expensive request per iteration between ring pumps, so a long plan can never starve the
sample ring or the exec heartbeat.

Cancelling is the consumer's act, never the planner's. Every event the planner emits
names the queue index it belongs to, so the command plane attributes a late result to the
command that asked for it and discards one for a command it has already dropped; on a
stop the command plane flushes the ring and returns the RT to IDLE itself, without
waiting on the planner, and only then tells the planner to forget its in-flight state.
This is the shape parol6 already runs, copied deliberately because it is the working
example: its planner is a subprocess (`parol6/server/motion_planner.py`) whose worker
loop applies `CancelAll` cheaply and plans one `PlanCommand` at a time, every segment it
emits carries its `command_index`, and the segment player in the control loop
(`parol6/server/segment_player.py`) plays, attributes and discards stale segments on
cancel — the planner's own `cancel()` only clears its blend buffer. Its enablement
probe likewise lives in a separate IK worker process (`parol6/server/ik_worker.py`).
par6's earlier single command task that owned the planner by value was the regression,
not the improvement.

Between planes:

- command plane → RT: an mpsc channel of `RtCommand`s. The RT loop drains **at most one
  command per tick**, which is why every multi-step effect (mode dances, the e-stop
  clear sequence) is an ordered queue rather than a synchronous call.
- streamed setpoints: a latest-wins slot instead of the channel — a jog stream faster
  than the tick rate would otherwise grow a backlog and keep jogging after the operator
  let go.
- RT → everyone: latest-wins snapshot slots, fanned out by `par6d-tee`.

**Where the tick rate comes from.** The tick period is a property of the bus, not of
the software. Every tick sends one motion frame per joint and reads a reply for each;
classic CAN carries a fixed number of bits per frame at a fixed bitrate, so the wire
time per tick is fixed and the tick has to be longer than it. `bus_budget` in
`crates/par6-bus/src/budget.rs` computes that time from the joint count, the gripper,
the poll slot and the configured bitrate; the daemon refuses to boot when the configured
`tick_dt_s` cannot carry it and logs the result at every hardware boot, so there is
exactly one place the number comes from. Two things follow. A faster host tick needs a
faster bus (CAN FD, EtherCAT), not different software: nothing in the runtime knows the
shipped rate — every duration is configured in seconds and converted with
`round(s / dt)` at construction, and the integration suites boot the simulator at a
second rate to keep it that way. And the drives close their own control loop at their
own rate, so a host tick faster than the bus allows would command motion the arm cannot
track and duplicate a loop the drive already runs.

## Control loop internals

One tick, in order:

1. **CAN RX** — drain the bus, decode Spectral frames into per-joint state.
2. **State** — update positions, velocities, currents, temperatures, error latches.
3. **Gravity compensation** — `G(q)` from Pinocchio on the arm-only chain, with the
   active tool's inertials attached from its gripper config so tool mass has exactly
   one source.
4. **Mode dispatch** — one of IDLE / HOMING / JOG / STREAM / EXEC / SAFETY_STOP /
   FLASHING produces this tick's setpoints. IDLE on a homed, enabled arm with
   gravity comp on is freedrive: torque-only `G(q)`, no position hold. The opt-in
   `[freedrive] drift_lock` re-holds the pose once the arm has been still (the
   drive's impedance frame plus a clamped integral) and lets go the tick a joint
   moves, so a slightly wrong gravity model stops sagging the arm without the
   operator ever fighting a hold.
5. **CAN TX** — one motion pack per joint, plus any queued control frame.
6. **Snapshot** — publish state to the command plane's reader slot.

Fault authority sits with the drives: the firmware gates all motion on its aggregate
error latch and forces `Controller_mode = 0`, and the simulator does the same, so a
green CI run is not overstating what it proves.

The deadline is computed from a monotonic clock *before* the first sleep, so tick 2 is
not a period late and the loop statistics start clean.

## Command system

Wire tags are banded by class, which is what makes gating table-driven
(`crates/par6-proto/src/enums.rs`):

| Band | Class | Semantics |
|---|---|---|
| 10+ | SYSTEM | reset, stop, e-stop, gravity comp, tool/profile selection |
| 30+ | QUERY | angles, pose, status, io, queue, error, reachable, loop stats |
| 60+ | FIRE_AND_FORGET | jog, servo, teleport — unacked, latest-wins |
| 80+ | QUEUED | move_j / move_l / move_c / move_s / move_p, tool actions, checkpoints |

A QUEUED command is acked with its queue index, then reports its outcome in a
**COMPLETE push** — so a refusal that happens at dispatch (a collision gate, an
unreachable path) arrives on the COMPLETE, not on the ack. Clients that need the
verdict pass `wait=True`.

Fire-and-forget commands are unacked by definition, so a refused one would otherwise be
invisible. par6 latches it as the standing error, sends a real ERROR datagram, warns in
the client log, and withdraws the affected `joint_en` flags.

## Adding a new command

The recipe, in the order the codec tests expect:

1. **Tag + variant** — add to `CmdType` in `crates/par6-proto/src/enums.rs` inside the
   right band, then the `Command` variant, its decode arm, its encode arm, and its
   `validate` rule in `command.rs`.
2. **Gating** — `crates/par6-server/src/gating.rs` decides what state the command needs
   (enabled, homed, simulator). Check the RT side agrees: a command accepted on the
   wire and refused by the RT mode table is a silent drop.
3. **Dispatch** — `crates/par6-server/src/server.rs`, then the `RtCommands` trait method
   in `runtime.rs`, then its implementation in `crates/par6d/src/bridge.rs` (immediate)
   or `planner.rs` (queued).
4. **Clients** — `crates/par6-client/src/api.rs`, the `crates/par6-py` binding, and the
   Python shim (`python/par6/client/`). The preview needs nothing per-command: it drives
   the daemon's own planner.
5. **Codec tests** — `crates/par6-proto`'s encode/decode round trip and hostile-input
   tests cover every tag; regenerate the Python constants mirror
   (`cargo run -p par6-proto --bin gen_python`).
6. **Test** — a sim e2e that drives the command through the real client against a real
   `par6d --sim`.

`par6-proto` is a frozen interface: changing it needs a `contracts`-labeled issue and a
re-freeze. See `CLAUDE.md`.

## Motion profiles

| Profile | Used for |
|---|---|
| `RUCKIG` (default) | jerk-limited point-to-point and streaming; the profile blends are built on |
| `TRAPEZOID` | velocity-limited point-to-point |
| `QUINTIC` | point-to-point with zero velocity **and** acceleration at both ends; no cruise, does not blend |
| `TOPPRA` | time-optimal retiming of a cartesian waypoint path |

Every cartesian move rides one pipeline: the geometry produces a pose list, seeded IK
turns each pose into a joint waypoint, and TOPPRA times the chain. Only the shape
differs — a line for `move_l`, the circle through the via point for `move_c`, a cubic
spline for `move_s`, an auto-rounded polyline for `move_p`.

`speed` scales the velocity ceiling, `accel` scales the acceleration ceiling, and
`duration` acts as a **minimum** the plan is stretched to meet. The two are mutually
exclusive.

A move with a positive blend radius `r` is **held** until the command after it decides
what the corner looks like; consecutive same-family moves fold into one motion that
completes every command it consumed at the same instant.

## Collision world

The runtime enforces an SRDF-exact collision world on planned **and** streamed motion,
in two layers: `installation` (from the robot TOML, immutable from the wire) and
`program` (replaced by `set_shapes`). Both are checked with a 5 mm default clearance.

The rule, for planned and streamed motion alike: a configuration may **keep** a pair the
start is already in — an arm inside a keep-out has to be able to move its way out — but
may not **add** one. Planned paths are walked at 0.02 rad joint pitch; streams are
projected one velocity-scaled lookahead ahead, so a faster jog stops further from
contact.

Colliding geometry is reported in waldoctl's vocabulary: bare URDF link names for the
arm and tool, `shape:<name>` for a program keep-out, `install:<name>` for an
installation one.

The client side runs the same world. `Robot.in_collision` / `colliding_pairs` /
`check_trajectory` / `min_distance` / `apply_shapes` drive the engine's `CollisionWorld`
(`par6_kin::Collision` through `par6._par6`) on the active tool's own URDF tree with its
SRDF and the config's installation keep-outs loaded, so a preview and the arm agree
about which paths are refused and name the offending pairs the same way.

## Kinematics and tools

par6 ships one URDF tree per fitted end-effector — flange, MSG gripper, SSG48 gripper —
and the runtime is built around **one** fitted gripper, refusing `SELECT_TOOL` for any
other. A tool's TCP is not modelled separately: it is the `tcp` link of that tool's own
tree, so selecting a tool selects the tree the runtime is fitted with and FK resolves
exactly where `par6d` does.

`set_tcp_offset` composes after the tool transform, in the tool-local frame. A variant
change clears it, because an offset measured against the old TCP describes nothing once
the frame moves. It is a queued command: the offset lands at its turn, so moves queued
before it keep the old frame, moves after it are planned against the new one, and a
blend chain never folds across it. `SELECT_TOOL` and `SET_TCP_OFFSET` therefore apply in
program order, and the `TCP_OFFSET` query reports the new value only once the command
has completed — the same lag `TOOLS` has after `select_tool`.

The trees are re-based onto the vendor motor convention: URDF `q` equals the runtime's
`theta`, so config angle values apply to the model verbatim. See
`assets/par6_description/CHANGELOG.md` for the derivation and the equivalence check.

## The Pinocchio shim

`cpp/` is one C-ABI shim over the C++ dependencies the Rust crates link:

- **Pinocchio** (kinematics/dynamics) — `par6_kin_*`: create/destroy, fk, jacobian,
  gravity, aba. Consumed by `par6-kin`, whose analytic IK (`par6_kin::Opw`) is derived
  from the URDF at load: the fit is checked
  against this FK at pseudo-random configurations and a model the two disagree on is
  refused. That catches an FK the OPW form cannot express, not a wrong URDF — a
  mis-measured link length fits, so the geometry is nominal data the check does not
  second-guess.
- **coal / hpp-fcl** (collision) — `par6_col_*`: a two-layer world (installation keep-outs
  and `SET_SHAPES`) over the URDF's `<collision>` meshes, self pairs minus same-joint and
  parent/child-adjacent ones, shapes in metres and radians (`R = Rx·Ry·Rz`).
- **toppra-cpp** (time-optimal path parameterization) — `par6_traj_*`. Built from source
  by `scripts/ffi/setup.sh` (conda-forge ships no C++ toppra), pinned to commit
  `142456f3` (v0.6.9), with its bundled Seidel LP solver — no qpOASES, no GPL GLPK.

```
cpp/include/par6_shim.h    the frozen C ABI (PAR6_SHIM_ABI_VERSION)
cpp/src/par6_shim.cpp      par6_kin_* (pinocchio)
cpp/src/par6_traj.cpp      par6_traj_* (toppra-cpp)
cpp/src/par6_col.cpp       par6_col_* (pinocchio + coal)
crates/par6-kin/src/sys/   the raw decls (ffi.rs) and the RAII handles over them; Kin/Collision/Trajectory build on those
scripts/ffi/setup.sh       reproducible toolchain bootstrap (micromamba)
```

`scripts/ffi/setup.sh` puts everything under `<repo>/.ffi` (self-gitignored, override
with `PAR6_FFI_DIR`): `bin/micromamba`, `env/` (conda-forge packages including
libmujoco, plus the from-source toppra install), `shim/` (installed lib + header),
`env.sh`. Re-running is idempotent: a shim whose recorded `cpp/` digest no longer
matches the tree is rebuilt, and `FORCE=1` rebuilds it regardless. Pinned: **pinocchio
4.1.0**, **toppra 142456f3**, **libmujoco 3.10.0** (`PAR6_PINOCCHIO_VERSION` /
`PAR6_TOPPRA_COMMIT` / `PAR6_MUJOCO_VERSION` override). Builds discover the shim under
`.ffi/shim/lib` and libmujoco under `.ffi/env/lib` on their own and carry both as
rpaths, so `source .ffi/env.sh` is only needed to point at libraries installed
elsewhere (`PAR6_SHIM_LIB_DIR`, `PAR6_SHIM_INCLUDE_DIR`, `PAR6_SHIM_LINK=dylib|static`,
`PAR6_SHIM_DEP_LIB_DIR`, `PAR6_MUJOCO_LIB_DIR`).

ABI conventions, frozen in `par6_shim.h`: poses are row-major 4×4; Jacobians 6×nq,
rows `[linear; angular]`, world axes at the frame origin; gravity is RNEA at zero
velocity/acceleration; an optional rigid tool at create shifts fk/jacobian/ik to the tool
frame and adds its inertials to the gravity model; every `par6_kin_*` call after create
is allocation-free (one handle per thread); `par6_traj_sample` is allocation-free and
safe from the RT tick; `par6_col_check` allocates in coal's narrow phase and is
planner-side only. Exceptions never cross the boundary.

What the shim is held to: `crates/par6-kin/tests/c_boundary_{collision,traj}.rs` cover the C
boundary itself (NULL/out-of-range arguments, geometry-index layout across layer
replacement, buffer truncation, the time-optimality requirement of the retimer);
`crates/par6-kin/tests/{kinematics,collision_world}.rs` cover the contract above the
boundary — the Jacobian as the derivative of FK, IK landing on every FK pose, and the
collision verdicts a preview and the runtime both depend on, placed from the model's own
TCP on every shipped URDF variant.

Mesh cost: `assets/` ships the vendor's full-resolution STLs for both `<visual>` and
`<collision>`. Measured per-waypoint check cost on the control box, in release:

| scene | flange | gripper variant |
|---|---|---|
| self-collision only | 14 µs | 19 µs |
| plus a box keep-out | 25 µs | 25 µs |
| plus a keep-out carrying its own margin | 26 µs | 34 µs |

Convex hulls were measured and rejected: the SSG48's jaw hulls overlap when closed and
report a permanent false collision at the home pose.

Every keep-out is a bounded primitive. A half-space was measured and rejected too: an
unbounded solid has no bounding volume to prune against, so coal scans every triangle
and one check costs ~35 ms against 25 µs for a box.

conda-forge ships `linux-aarch64` Pinocchio, so the control box builds the shim natively
with the same script; cross-compiling it from x86_64 is not supported.

## Ports and environment variables

Only the **6001** command port is fixed by the wire contract; the rest are defaults in
`config/PAR6.toml` under `[protocol]`.

| Port | Purpose |
|---|---|
| 6001 | command plane (UDP, msgpack) |
| 6002 | status broadcast (binary) |

Precedence throughout is **CLI flag > `PAR6_*` environment variable > robot TOML**.

| Variable | Effect |
|---|---|
| `PAR6_CONFIG` | robot TOML path (`--config`) |
| `PAR6_ASSETS` | `par6_description` tree with the URDFs (`--assets`) |
| `PAR6_COMMAND_PORT` | command UDP port; `0` = ephemeral (`--port`) |
| `PAR6_BIND` | command-socket bind address (`--bind`) |
| `PAR6_STATUS_HOST` | unicast status destination (`--status-host`) |
| `PAR6_STATUS_PORT` | status broadcast port (`--status-port`) |
| `PAR6_STATUS_TRANSPORT` | `auto` \| `multicast` \| `unicast` (`--status-transport`) |
| `PAR6_STATUS_RATE_HZ` | STATUS broadcast rate; must divide the tick rate (`--status-rate`) |
| `PAR6_SIM_DYNAMICS` | with `--sim`, use the torque-level plant (`--sim-dynamics`) |
| `PAR6_LOG_DIR` | also write the rotating activity logs there (`--log-dir`) — see below |
| `PAR6_TICK_PROFILE` | per-phase RT tick profiler, logged once a second (`--tick-profile`) |
| `PAR6_GPIO_CHIP` | gpiochip device for the e-stop line |
| `PAR6_SHM_DIR` | where the bus-grant segments go (default `/dev/shm`) — see below |

### Activity logs

stderr carries every log line, as always (`RUST_LOG` filters it, default `info`).
With `--log-dir` the daemon also keeps two size-rotated files there, routed by the
record's module target: `rt.log` (2 MiB, five copies) holds what the RT thread
itself says — mode transitions, latches, degraded-scheduling notices — and
`commands.log` (20 MiB, five copies) holds the command plane and the daemon: one
line per accepted, completed, refused or cancelled command keyed by its index, with
the error catalog's cause and remedy on failure, the RT latch on its edges, and a
host-vitals line (load, memory, CPU temperature, disk, uptime) at start and every
minute. The RT tick never writes a file: its only log calls sit on throttled
failure paths, so the sink costs the tick nothing, and a write that fails is
dropped rather than allowed to stall the daemon.

### The front panel

`par6-panel` (installed with `pip install "par6[panel]"`, run by
`scripts/deploy/par6-panel.service`) owns the control box's two buttons, two
LEDs and 128x64 OLED and the UART link to the mainboard PCB, entirely from
`panel.toml` (`PAR6_PANEL_CONFIG`; every device path, I²C address, pin and
baud lives there). One rule on the buttons: tap = move, hold = select, hold
button 1 = back; destructive actions ask for a hold to confirm and cancel on
any tap. Once a blink period it sends the PCB its heartbeat, toggles the
LEDs anti-phase, publishes the panel state and drives the LEDs from what it
published. A UART that will not open disables PCB comms and nothing else.

Install it on the box with `pip install "par6[panel]"` into a venv, copy
`panel.toml` to `/etc/par6/panel.toml`, point the unit's `ExecStart` at that
venv's `par6-panel`, then `systemctl enable --now par6-panel`. Run
`par6-preflight` first: it changes nothing and reports what the box is
missing.
`par6-preflight` is the diagnostic that brings nothing up: RT kernel, CAN,
GPIO and RT-priority permissions, cores, disk, devices and imports, each
required or advisory, re-executed inside the package's virtualenv so it
reflects the runtime environment.

### Commissioning a drive

A fresh Spectral drive sits at its factory node id, which the config does not
list. `par6 scan` rescans the bus (an RTR ping to every id, one per tick) and lists
every node id with whether the config lists it, whether it answered, its freshness
and its device identity; `par6 set-can-id OLD NEW --force` renames it (cmd 11) and
`par6 save-config NEW --force` persists that (cmd 13) — without `--force` both
refuse an id the config does not list. Both are refused while anything could be
moving: only an IDLE or ACTIVE_ERROR arm with nothing executing, queued or
streaming qualifies, so holding the e-stop while you commission is the normal
way. The runtime keeps addressing the ids the config names, so after renaming a
configured drive update the config and restart the daemon. `par6 set-pid-gains`
pushes one drive's tuning live, `par6 tool` runs a tool action, and
`par6 flashing enter|exit` hands the bus to a firmware flasher and takes it back.

### The bus-grant signal

`can0` is a system-wide exclusive resource, and the vendor's CAN tools (the
firmware flasher, the motor tuners) decide whether they may transmit by reading
two shared-memory segments the runtime publishes:

| Segment | Contents | Meaning |
|---|---|---|
| `/dev/shm/loop_tick` | one little-endian `f64` | advancing = a runtime is live and owns the bus |
| `/dev/shm/robot_mode` | 4-byte LE length + UTF-8 | `FLASHING` = bus granted; anything else = keep off |

par6d publishes both, from the RT core's own tick counter — so a stalled RT
thread reads as stalled — and removes them on a clean stop. **A box publishing
neither reads as having no runtime at all**, which is a flasher's cue to
transmit; that is the failure this exists to prevent, not a nicety.

Liveness is read before the mode, so a segment left behind by a crash is safe:
its tick stops advancing and the box reads as free, which by then it is.

`par6d.service` sets `BindPaths=/dev/shm` because a private `/dev/shm` would put
the segments somewhere only par6d can see. Point a second runtime on the same
box at `PAR6_SHM_DIR` so it does not overwrite the claim of the one that owns
the arm.

The Python side reads three of its own:

| Variable | Effect |
|---|---|
| `PAR6D_BIN` | the `par6d` binary `Robot.start()` spawns |
| `PAR6_HOST` | default client host |
| `PAR6_COMMAND_PORT` | default client port |

## Development setup

```bash
scripts/ffi/setup.sh                                                       # once: the shim
cargo fmt --all && cargo clippy --workspace --all-targets -- -D warnings   # CI gate
cargo test --workspace
cargo test -p par6-bus --test socketcan_vcan -- --test-threads=1           # needs vcan0
cargo build -p par6d --release
pip install -e "python[dev]"                                               # builds par6._par6
cd python && PAR6D_BIN=../target/release/par6d python3 -m pytest -q
```

The Rust tests are the whole test surface for the numerics: the kinematics contract
(`par6-kin/tests/kinematics.rs`), the collision verdicts (`collision_world.rs`), and the
preview ↔ runtime parity (`par6d/tests/preview.rs`) are all requirement-derived — there
are no recorded fixtures and no second implementation to compare against. The Python
tests cover the waldoctl contract of the shim over the engine.

Use `python3 -m pytest`, not a bare `pytest` — on some setups the `pytest` on PATH
resolves to an interpreter that does not have the package.

Without `PAR6D_BIN` the Python e2e tests **skip**, which is how a whole integration
layer can vanish from a run unnoticed. CI sets it.

The `pytest` run writes JUnit XML to `python/test-results.xml`; read that rather than
re-running to recover console output.

## Deploying to the control box

Target: **Raspberry Pi 5, aarch64, PREEMPT_RT kernel**. The box runs everything —
Waldo Commander, this Python package and one `par6d` process supervised by systemd,
talking SocketCAN to the arm and protocol v2 (UDP) to Waldo Commander on localhost.

The normal path is to build **on the box** ([Installation](#installation): the shim,
`par6d` and the Python package build there in minutes) and install locally:

```bash
cargo build -p par6d --release --target aarch64-unknown-linux-gnu
python3 scripts/ffi/stage_runtime_libs.py --readelf readelf --lib-dir .ffi/env/lib \
    --dest .ffi/stage/lib .ffi/shim/lib/libpar6_shim.so .ffi/env/lib/libmujoco.so
scripts/deploy/install.sh --stage-only /tmp/par6-bundle --runtime-libs .ffi/stage/lib
sudo /tmp/par6-bundle/install.sh --local --bundle /tmp/par6-bundle
```

`install.sh` reads the binary from `target/aarch64-unknown-linux-gnu/`, so the
explicit `--target` matters even natively. Folding the staging step into
`install.sh` itself is still to do.

Cross-building from another machine is optional — for CI, or a box that should not
carry a toolchain:

```
scripts/ffi/setup.sh --target aarch64    build the aarch64 Pinocchio shim elsewhere
scripts/deploy/build-aarch64.sh          cross-build par6d for aarch64
scripts/deploy/install.sh --host ...     stage + upload + install over ssh
scripts/deploy/par6d.service             the systemd unit
scripts/deploy/par6-panel.service        the front panel service (optional)
```

### 1. Cross-build (optional)

```bash
scripts/ffi/setup.sh --target aarch64    # once (or after a dependency pin bump)
source .ffi/env-aarch64.sh
scripts/deploy/build-aarch64.sh
# -> target/aarch64-unknown-linux-gnu/release/par6d
```

**Every par6d carries kinematics.** The Pinocchio C-ABI shim — TCP FK,
gravity compensation, `move_l`/`move_j_pose`, the cartesian streamables,
TOPPRA, and the coal collision world — is linked unconditionally, because a
runtime without it would broadcast a NaN TCP pose, report zero cartesian
freedom, refuse every cartesian command, and answer `set_shapes` with success
against a collision world that does not exist, none of which a client can
see. `build-aarch64.sh` therefore fails early when the aarch64 shim has not
been built. The library crates still compile without any C++ toolchain; it is
only the binary that requires one.

#### How the aarch64 shim is produced

`scripts/ffi/setup.sh --target aarch64` cross-builds it on another machine, for
the case where the box should not carry a compiler:

- conda-forge publishes `pinocchio`, `coal`, `urdfdom` and `eigen` for
  `linux-aarch64`, so micromamba **downloads** the target's libraries with
  `--platform linux-aarch64` — an env that is never executed on the host.
- the compiler is conda-forge's `gxx_linux-aarch64` cross toolchain from the
  host's own `linux-64` channel, pinned to `sysroot_linux-aarch64=2.17` (the
  same glibc conda-forge builds its own aarch64 packages against, which is
  what keeps the whole set at one floor).
- `toppra-cpp` has no conda-forge package at all, so it is built from the
  pinned commit through a generated CMake toolchain file, exactly like the
  native path.
- `scripts/ffi/stage_runtime_libs.py` then walks `DT_NEEDED` from
  `libpar6_shim.so` and `libmujoco.so` and copies the whole closure into the
  shim's own `lib/` directory. That directory is the deploy unit:
  the shim is linked with `$ORIGIN`, `par6d` with an rpath of
  `/usr/local/lib/par6`, and `install.sh` copies the one into the other.

The alternatives were building natively on the box (needs a Rust and C++
toolchain on a Pi and takes the better part of an hour per change) and
vendoring prebuilt binaries (unpinned, unreproducible, and a licence
question). Cross-building keeps a single reproducible command and the same
pins as the x86_64 path.

**glibc floor.** The staged closure and the shim require at most
`GLIBC_2.17`; the `par6d` binary linked against them requires at most
`GLIBC_2.17` as well, because `.ffi/env-aarch64.sh` makes the conda cross
compiler the Rust linker too. Raspberry Pi OS **bookworm ships 2.36** and
bullseye ships 2.31, so both clear it — a change from the previous
Debian-cross build, whose floor was `GLIBC_2.34`. `build-aarch64.sh` prints
the measured floor after each build, `stage_runtime_libs.py` prints the
closure's, and `install.sh` runs `par6d --help` right after copying so a
mismatch still fails loudly at install time.

**Symbol-version check.** Because nothing here can be *executed* for
aarch64, `stage_runtime_libs.py` performs the check that would otherwise
only surface on the box: every versioned symbol (`GLIBCXX_*`, `CXXABI_*`,
`GCC_*`, …) demanded of a library that ships must be provided by the copy
that ships. This is what catches a cross compiler newer than the target
env's C++ runtime, which otherwise appears as
`version GLIBCXX_3.4.x not found` at the first `systemctl start`.

### 2. Install

On the box, use the local sequence at the top of this section. From another machine
(needs `ssh`/`scp` to the box and `sudo` on it):

```bash
scripts/deploy/install.sh --host pi@par6-box
```

It stages a bundle (binary + `lib/` — the shim and its runtime closure —
+ `config/PAR6.toml` + `config/grippers/*.toml` + `assets/par6_description`
+ the unit + a copy of itself), uploads it to `/tmp/par6-deploy-<timestamp>`,
and re-runs itself there with `--local`. `PAR6_RUNTIME_LIB_SRC` (exported by
`.ffi/env-aarch64.sh`) says where the staged libraries come from; pass
`--runtime-libs DIR` to override it. `--stage-only DIR` builds the bundle
without uploading anything, which is what CI checks.

On the box itself:

```bash
sudo scripts/deploy/install.sh --local --bundle /tmp/par6-deploy-<timestamp>
```

Layout after install:

| Path | Contents |
|---|---|
| `/usr/local/bin/par6d` | the runtime binary |
| `/usr/local/lib/par6/*.so` | the Pinocchio shim + its runtime closure (rpath target) |
| `/etc/par6/PAR6.toml` | robot config (`PAR6_CONFIG` in the unit) |
| `/etc/par6/grippers/*.toml` | gripper configs |
| `/usr/share/par6/par6_description` | URDF/meshes — the kinematics and collision models |
| `/etc/systemd/system/par6d.service` | the unit |
| `/var/lib/par6` | `StateDirectory`, the working directory |

An existing `/etc/par6/*.toml` is **kept** on re-install (tuning survives
upgrades); pass `--force-config` to overwrite. `--no-restart` installs without
touching the running service.

> Restarting `par6d` stops the arm and clears the queue. `install.sh` stops the
> service before swapping the binary unless `--no-restart` is given.

### 3. The unit

`par6d.service` runs as the unprivileged system user `par6` with two ambient
capabilities:

- **`CAP_SYS_NICE`** — the RT thread asks for `SCHED_FIFO` priority 99 and pins
  itself to CPU 3. Failure is logged `DEGRADED` and is *not*
  fatal, so a misconfigured box runs badly instead of not at all — check the
  journal for `RT thread: SCHED_FIFO priority 99` to confirm it took.
  `LimitRTPRIO=99` is set as well for boxes without the capability path.
- **`CAP_NET_ADMIN`** — `par6d` brings `can0` up at the configured bitrate when
  it finds it down, and sets its txqueuelen through the `SIOCSIFTXQLEN` ioctl —
  the sysfs file is root-owned and refuses an unprivileged writer regardless
  of capabilities.

It also carries `SupplementaryGroups=dialout`: Raspberry Pi OS and Ubuntu
hand the header's gpiochip to that group (udev `60-gpio.rules`), and without
it the e-stop line cannot be opened, which is a refusal to start. And
`LimitMEMLOCK=infinity`, because the RT thread calls `mlockall` — a page
fault inside the tick is an unbounded latency spike, and on a box with swap
a cold page is a disk read (logged `DEGRADED` if the call fails).

The unit deliberately sets **no `CPUAffinity=`**: `par6d` pins its own RT
thread, and a process-wide mask would trap the tokio command plane on the same
core. Isolate the RT core in the kernel cmdline instead
(`/boot/firmware/cmdline.txt` on RPi OS):

```
isolcpus=3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2
```

Logging goes to journald (`SyslogIdentifier=par6d`, `RUST_LOG=info`):

```bash
journalctl -u par6d -f
systemctl status par6d
```

`Restart=always` with `RestartSec=2`, bounded by `StartLimitBurst=5` per
`StartLimitIntervalSec=60` so a genuinely broken install stops flapping.

#### Simulator on the box

```bash
sudo systemctl edit par6d      # drop-in
[Service]
ExecStart=
ExecStart=/usr/local/bin/par6d --sim
```

`--sim` runs unprivileged with no SCHED_FIFO and no CPU pin.

### 4. Post-install check

```bash
journalctl -u par6d -n 30
#   loaded PAR6 (6 joints, tick 250 Hz) from /etc/par6/PAR6.toml
#   command plane on 0.0.0.0:6001 (SocketCAN backend)
#   RT thread: SCHED_FIFO priority 99
#   RT thread pinned to CPU 3
```

On the box (pass `host=` for another machine on the network):

```python
from par6 import Robot
robot = Robot()   # PINGs the running runtime, spawns nothing
robot.start()
print(robot.create_sync_client().angles())
```

#### If it does not start

- **`can0` cannot be opened.** The unit's `RestrictAddressFamilies` must
  include `AF_CAN`; relax it and confirm the bus opens before looking
  anywhere else.
- **Starts by hand but not under systemd.** `ProtectSystem=strict` leaves
  `/usr` readable, so the rpath into `/usr/local/lib/par6` should resolve —
  run `ldd /usr/local/bin/par6d` from inside the unit's namespace to see
  which library the sandbox is hiding.

### 5. Network posture

The protocol-v2 command plane is deliberately unauthenticated: Waldo
Commander and `par6d` run on the same Raspberry Pi, so the 50 Hz
client↔daemon traffic is loopback. Anyone who can send UDP to port 6001
can move the arm — treat reachability as authorization, the way UR and
Franka deployments do:

- **Keep the robot off routable networks.** Put the box on a dedicated
  NIC, VLAN or physically separate segment shared only with the machines
  that operate it. Do not port-forward 6001 or the status
  ports.
- **Remote access goes through the OS, not the protocol.** For operating
  the arm from elsewhere, terminate a WireGuard (or SSH) tunnel on the
  box and keep the command plane bound behind it.
- A firewall rule that pins 6001 to the operator hosts is a fine belt —
  but it is a reachability control, not authentication, and it does not
  make the plane safe to expose.

Message authentication (HMAC-tagged datagrams with a pre-shared key) is
planned for when the frontend and the runtime split across machines;
until then the isolation above is the security boundary.

## Known divergences from parol6

Deliberate, and unlikely to change:

- **Three motion profiles**, not five — `QUINTIC` and `LINEAR` are absent, consistently
  on the runtime and in the preview.
- **`select_tool` accepts only the fitted tool.** The runtime is built around one
  gripper and its URDF tree; the preview refuses the same set, so the two agree.
- **No tool variants.** The vendor CAD fuses the gripper body into the arm's final link
  mesh, so there are no per-variant mesh sets to swap. `variant_key` still rides through
  to STATUS because the runtime carries it, but it selects no geometry.
- **Error codes 52/53/54 mean different things than parol6's.** They are frozen contract
  data — read them by name (`ErrorCode.SYS_SELF_COLLISION`), never by number.

Open gaps are tracked as [issues](https://github.com/Jepson2k/par6/issues).

## Safety notes

- **Restarting `par6d` stops the arm and clears the queue.** `scripts/deploy/install.sh`
  stops the service before swapping the binary unless `--no-restart` is given.
- **A refused command is not a stopped arm.** Fire-and-forget refusals latch as the
  standing error; check `error()` or the STATUS broadcast rather than assuming a send
  that returned 1 took effect.
- **The e-stop is a latch.** Clearing it runs a multi-tick sequence on the RT thread;
  `reset()` does not return until the RT has actually answered, because "the enable was
  queued" is not "the arm will move".
- **Homing references the arm.** Planned motion is refused before it; jogging is not, so
  an arm can be driven clear of an obstruction before it is referenced.
- **aarch64 kinematics are built but not validated** — the shim's numerics have never
  been executed on that ISA ([#31](https://github.com/Jepson2k/par6/issues/31)).
- The vendor runtime (RCB-Runtime) and the Spectral firmware are GPL: they are
  **behavior-only reference**. Port behavior and constants, never code.

## License

Apache-2.0 (`LICENSE`). `assets/par6_description/` derives from Source Robotics' PAR6
repository under a licence upstream states two ways — see `assets/NOTICE`, which records
what is verbatim, what par6 modified, and what par6 authored.
