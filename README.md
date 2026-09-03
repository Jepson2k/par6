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
- [Ports and environment variables](#ports-and-environment-variables)
- [Development setup](#development-setup)
- [Known divergences from parol6](#known-divergences-from-parol6)
- [Safety notes](#safety-notes)
- [License](#license)

## Installation

`par6d` links a Pinocchio C-ABI shim, so that gets built once before the runtime.
The library crates need no C++ toolchain; only the binary does.

```bash
scripts/ffi/setup.sh             # once — builds the shim into .ffi/
source .ffi/env.sh               # each shell: par6d needs the shim to BUILD and to RUN
cargo build -p par6d --release
pip install -e "python[dev]"
```

`source .ffi/env.sh` is not only a build step. It sets `LD_LIBRARY_PATH`, and a `par6d`
started without it exits with `libpar6_shim.so: cannot open shared object file`.

Installing just the client, which is what Waldo Commander's `[par6]` extra does:

```bash
source .ffi/env.sh   # the package compiles the par6-py extension against the shim
pip install "par6 @ git+https://github.com/Jepson2k/par6.git@main#subdirectory=python"
```

The package is a maturin build: pip compiles the `par6-py` extension (the engine's
client + preview), so a source install needs the Rust toolchain and the shim from
`scripts/ffi/setup.sh`. Prebuilt wheels that need neither are the wheel CI's job.
That gives you the client, the offline preview and the kinematics — but **not** the
`par6d` binary. `Robot().start()` spawns `$PAR6D_BIN`, or `par6d` on `PATH`, so a
client-only install has nothing to spawn until either the workspace above is built or
a runtime is already listening — which is the normal case, since the runtime belongs on
the control box and the client does not. Shipping a per-platform runtime wheel is
[#33](https://github.com/Jepson2k/par6/issues/33).

Deploying to a control box (Raspberry Pi 5, aarch64, PREEMPT_RT) is its own document:
[`scripts/deploy/README.md`](scripts/deploy/README.md).

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

with RobotClient(host="par6-box.local", port=6001) as rbt:
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
       │ protocol v2: UDP msgpack commands · binary status broadcast · telemetry
  par6d (single Rust binary; `par6d --sim` runs anywhere, including CI)
   ├─ command plane (tokio): validation/gating, queue, index allocator,
   │    push completion, status broadcaster, telemetry recipes
   ├─ planner: TOPPRA (FFI) planned moves · rsruckig streaming/blending · trapezoid,
   │    plus the plan-time collision gate
   └─ RT thread (SCHED_FIFO 99, alloc-free): 250 Hz tick — CAN RX → state → gravity
        comp G(q) → mode dispatch → CAN TX → state snapshot
   bus backends: SocketCAN (Spectral/STEPFOC) | closed-loop dynamics sim (Pinocchio ABA)
```

Kinematics and dynamics run on **Pinocchio through a C-ABI shim** shared with the
Python side's [pinokin](https://github.com/Jepson2k/pinokin) — one numerics stack on
both sides of the wire, which is what lets the offline preview agree with the runtime
rather than approximate it.

### Repository layout

| Path | Contents |
|---|---|
| `crates/par6-proto` | protocol v2 codec — **single source of truth**; the Python constants are generated from it |
| `crates/par6-config` | robot / gripper / homing TOML config |
| `crates/par6-kin` | Pinocchio FFI: FK, Jacobian, gravity, IK; coal collision world (self-pairs + installation/program keep-out layers) |
| `crates/par6-motion` | TOPPRA + rsruckig + trapezoid, jog ramps, completion policies |
| `crates/par6-bus` | `DriverBus` trait, Spectral CAN codec, SocketCAN + simulator backends |
| `crates/par6-rt` | RT tick loop, mode dispatch, homing FSM, error latching, e-stop |
| `crates/par6-server` | UDP command plane, status/telemetry broadcast, collision-world layers |
| `crates/par6-client` | the client library: command round-trips, retries/dedup, status subscription |
| `crates/par6d` | the runtime binary: config load, thread spawn/wiring, planner, RT bridge — plus the offline preview harness |
| `crates/par6-py` | the `par6._par6` Python extension (PyO3 over par6-client + the preview) |
| `cpp/` | the Pinocchio/coal/TOPPRA C-ABI shim |
| `python/` | the `par6` pip package (waldoctl backend) |
| `tests/golden/` | golden wire vectors (encode/decode conformance for the frozen codec) |
| `tools/bringup/` | the staged on-hardware bring-up kit — see its README |
| `assets/` | PAR6 URDF, SRDF and meshes from Source Robotics — see `assets/NOTICE` |

### Two planes, one process

The **command plane** is tokio on ordinary threads: it parses datagrams, validates and
gates them, allocates queue indices, and broadcasts STATUS. The **RT thread** runs
`SCHED_FIFO` priority 99 pinned to one core and allocates nothing after init.

They are joined by an mpsc channel of `RtCommand`s and by latest-wins snapshot slots.
The RT loop drains **at most one command per tick**, which is why every multi-step
effect (mode dances, the e-stop clear sequence) is an ordered queue rather than a
synchronous call, and why streamed setpoints use a latest-wins slot instead of the
channel — a jog stream faster than the tick rate would otherwise grow a backlog and
keep jogging after the operator let go.

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

The recipe, in the order that keeps the golden-vector guard happy:

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
5. **Golden vectors** — regenerate; the manifest's coverage guard fails on a tag with no
   vector, which is the guard working.
6. **Test** — a sim e2e that drives the command through the real client against a real
   `par6d --sim`.

`par6-proto` is a frozen interface: changing it needs a `contracts`-labeled issue,
regenerated vectors on **both** sides, and a re-freeze. See `CLAUDE.md`.

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
`check_trajectory` / `min_distance` / `apply_shapes` build a `pinokin.CollisionChecker`
on the active tool's own URDF tree with its SRDF loaded, so a preview and the arm agree
about which paths are refused.

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

## Ports and environment variables

Only the **6001** command port is fixed by the wire contract; the rest are defaults in
`config/PAR6.toml` under `[protocol]`.

| Port | Purpose |
|---|---|
| 6001 | command plane (UDP, msgpack) |
| 6002 | status broadcast (binary) |
| 6003 | telemetry stream |

Precedence throughout is **CLI flag > `PAR6_*` environment variable > robot TOML**.

| Variable | Effect |
|---|---|
| `PAR6_CONFIG` | robot TOML path (`--config`) |
| `PAR6_ASSETS` | `par6_description` tree with the URDFs (`--assets`) |
| `PAR6_COMMAND_PORT` | command UDP port; `0` = ephemeral (`--port`) |
| `PAR6_BIND` | command-socket bind address (`--bind`) |
| `PAR6_STATUS_HOST` | unicast status/telemetry destination (`--status-host`) |
| `PAR6_STATUS_PORT` | status broadcast port (`--status-port`) |
| `PAR6_TELEMETRY_PORT` | telemetry stream port (`--telemetry-port`) |
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
source .ffi/env.sh
cargo fmt --all && cargo clippy --workspace --all-targets -- -D warnings   # CI gate
cargo test --workspace
cargo build -p par6d --release
cd python && PAR6D_BIN=../target/release/par6d python3 -m pytest -q
```

Use `python3 -m pytest`, not a bare `pytest` — on some setups the `pytest` on PATH
resolves to an interpreter that does not have the package.

Without `PAR6D_BIN` the Python e2e tests **skip**, which is how a whole integration
layer can vanish from a run unnoticed. CI sets it.

The `pytest` run writes JUnit XML to `python/test-results.xml`; read that rather than
re-running to recover console output.

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

MIT (`LICENSE`). `assets/par6_description/` derives from Source Robotics' PAR6
repository under a licence upstream states two ways — see `assets/NOTICE`, which records
what is verbatim, what par6 modified, and what par6 authored.
