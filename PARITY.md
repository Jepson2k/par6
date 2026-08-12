# par6 ↔ parol6 parity audit

**Question answered:** what does "par6 matches parol6" still require?

**Reference:** `parol6` 0.4.0, source at `/usr/local/lib/python3.11/dist-packages/parol6/`.
**Subject:** this repo at `9074661`. Every line number is read at that commit
(`git show HEAD:<path>`): another agent is concurrently closing the seven rows listed under
[In flight](#in-flight), and its edits to `crates/par6-rt/**`, `crates/par6-server/**`,
`crates/par6d/**` and `python/**` landed in the tree during this pass. Symbol names are given
alongside the line numbers so citations survive that drift; the in-flight rows were deliberately
not re-verified.
**Contract spine:** `waldoctl` 0.7.0 at `/usr/local/lib/python3.11/dist-packages/waldoctl/`.
**Consumer of record:** Waldo Commander at `/home/user/Waldo-Commander` (WC).

This is the **second** pass. The first (`874af6d`, 31 rows) is superseded: nine commits since
have closed sixteen of those rows, and the work itself opened new surface — curved moves,
corner blending, collision enforcement, a dry-run client, an aarch64 deploy path, boot-enable —
which is audited here on the same terms. Rows that closed are listed with the commit that
closed them in [Closed since the first audit](#closed).

Every claim below is either a source citation on both sides or a **measurement** against a live
`par6d --sim` **built with `--features ffi`** (the shipped configuration; a build without it now
refuses to boot) driven by the real `par6` client over real UDP. Measurements are marked
`[measured]`; the reproduction is in [Appendix A](#appendix-a--how-the-measurements-were-taken).

---

## If you only fix five things

| # | Fix | Why it is first |
|---|---|---|
| 1 | **Gate jog and servo streams on the collision world** (issue #19). | Planned motion is gated and refuses correctly; streaming is not gated at all. `[measured]` with a 50 mm keep-out slab 60 mm under the TCP: `move_l` straight down is refused `SYS_SELF_COLLISION` naming `[jaw1, floor], [jaw2, floor]`, and then `jog_l("WRF","Z",-0.4)` drives the TCP **47.8 mm into the slab** and `servo_l` drives it **84.8 mm** in, with no error, no `collision_active`, nothing in the log. parol6 gates both, with a velocity lookahead and an escape rule (`parol6/commands/_collision_guard.py`, `collision_blocked`; used from `commands/basic_commands.py:235` for jog and `commands/cartesian_commands.py:163,233` for cartesian streaming). This is the one gap where par6 is *less safe* than parol6, and WC's jog buttons are the most-used control in the app. |
| 2 | **Make the sim plant converge on what it is commanded** (issue #22). | `[measured]` a `move_j` of +10° on J1 from the park pose lands at **+5.24°** — 47.6 % short — reports `COMPLETE`, and stays there. Same at an extended posture (J2 +10° → +5.04°). Under `commanded` *and* `settled` policies the command completes anyway; under `strict` it fails `MOTN_SETTLE_TIMEOUT` and the RT latches `ExecSettleTimeout` → `ActiveError`, i.e. the controller bricks. `--sim` is the default WC development experience, so every user sees a readout that never reaches its target, and `wait_command` returning `True` is a lie about where the arm is. |
| 3 | **Close the servo stall on a converged target** (issue #21). | `[measured]` 80 repeats of one fixed target 30° away moved J1 **14.27°** and stopped. Streaming now moves (that regression is fixed) but does not arrive: `MotionStream` commands the larger of the profile velocity and the position channel's advance, and once the OTG converges both are legitimately zero while tracking error remains. Needs the measured pose inside the stream law (a `par6-rt` trait change). WC's teleop, playback scrubber and cartesian jog all ride servo streams. |
| 4 | **Make fire-and-forget refusals reach the client.** | `par6d` refuses out-of-range `teleport` and multi-axis `jog_j` and *sends a real ERROR reply* (`crates/par6-server/src/server.rs:645-655`, `on_faf`) — but `AsyncRobotClient._fire` never awaits a reply (`python/par6/client/async_client.py:664-676`) and `_handle_reply` drops a datagram with no waiting future (`:530-544`). `[measured]` `teleport([0,-90,400,0,0,180])` returned `1`, the arm did not move, `error()` was `None` and `activity()` was `IDLE`. Same for multi-axis `jog_j`. par6's whole design argument is "refuse loudly instead of degrading silently"; at the last hop it degrades silently. |
| 5 | **Implement the client-side collision surface on the `Robot` ABC.** | `has_collision_checking` is still `False` and `in_collision` / `colliding_pairs` / `check_trajectory` / `min_distance` / `apply_shapes` all inherit waldoctl's disabled defaults `[measured]`. Now that both collision enforcement *and* the dry-run client have shipped, this shows up as a **contradiction the user sees**: `[measured]` the preview plans a 228-pose path straight through a keep-out with `error=None`, and the runtime then refuses the identical move. WC also skips per-segment collision (`path_visualizer.py:81`), never tints a colliding mesh, and pushes keep-outs into a checker that does not exist (`urdf_scene/scene_handle.py:87,207`). The dependency is already installed: `pinokin.CollisionChecker`, which is exactly what parol6 builds its checker from (`parol6/PAROL6_ROBOT.py:13,96-114`). |

---

## Gap register (ranked by user-visible impact)

Verdict key: **GAP** = genuine parity hole · **DIV** = justified architectural divergence
(still worth documenting) · **DIV+GAP** = the divergence is defensible but the way it surfaces
to the user is not.

| # | Gap | Category | Consequence | Size | Verdict |
|---|---|---|---|---|---|
| 1 | Jog/servo streams are not collision-gated (#19) | behavior/safety | A jog drives through a keep-out the planner refuses | M | GAP |
| 2 | Sim plant settles ~48 % short and never converges (#22) | behavior | Arm never reaches the commanded angle; `COMPLETE` fires anyway; `strict` bricks the controller | L | GAP |
| 3 | Servo stream stalls on a converged target (#21) | behavior | Held targets arrive less than half way | M | GAP |
| 4 | Fire-and-forget refusals never reach the client | RobotClient | `teleport` / multi-axis `jog_j` refused server-side; client told success, nothing reported | S | GAP |
| 5 | `Robot` collision surface is entirely default | Robot ABC | Preview draws paths the runtime refuses; no collision tinting, no editing-pose check | M | GAP |
| 6 | `simulator(false)` / `connect_hardware` not wired | commands | WC's Robot/Sim toggle and hardware-detect path both raise | M | GAP |
| 7 | `write_io` always errors while `digital_inputs/outputs` still say 2 and 2 | commands/telemetry | Four permanently-zero I/O chips whose toggles always fail; recorded `write_io` lines fail on replay | M | DIV+GAP |
| 8 | `select_tool` accepts only the fitted tool | commands | WC's dropdown has three entries; two of them raise | M | DIV |
| 9 | aarch64 kinematics are built but never executed | deploy | The control box runs a shim whose numerics no test has validated on that ISA | M | GAP |
| 10 | Collision allow-list is derived from the park pose, not an SRDF (#18) | behavior | Contact pairs legitimate in other postures can over-refuse | M | GAP |
| 11 | `ToolSpec`s carry no `variants` (and no `meshes`) | Robot ABC | Variant dropdown empty; `variant_key` unusable from the UI | S | DIV+GAP |
| 12 | `select_tool(variant_key=…)` accepts and echoes any string with no effect | commands | Declared-but-unimplemented surface, against this repo's own rule | S | GAP |
| 13 | `jog_j` refuses multi-joint | commands | Multi-joint jog scripts silently do nothing (see #4) | S | DIV |
| 14 | `motion_profiles` is 3 where parol6 offers 5 | config | `QUINTIC` / `LINEAR` unavailable in the profile picker | S | DIV |
| 15 | Dry-run preview does not mirror two runtime refusals | Robot ABC | Preview accepts an out-of-range `teleport`; no `write_io` method at all | S | GAP |
| 16 | Default command port 6001 vs WC's 5001 default | config | Out-of-the-box WC cannot find the runtime | S | DIV |
| 17 | Error code numbering diverges from parol6 (52/53 shifted) | commands | Scripts keying on numeric codes mis-read | S | DIV |
| 18 | No runtime binary in the pip package | packaging | `pip install par6` gives no `par6d` — and now no C++ shim either | M | DIV |
| 19 | No `PAR6_STATUS_RATE_HZ` / tick-rate env override | config | WC's conftest pattern needs a patched TOML instead | S | DIV |

Plus seven rows another agent is actively closing — see [In flight](#in-flight): `home()`
semantics, log forwarding, `queued_duration`, `loop_stats` zeros, `pose(frame="TRF")`,
`is_estop_pressed` / `is_robot_stopped`, and `Robot.start()` / `EXCLUSIVE_START`.

**Counts:** 19 open rows — 12 GAP, 5 DIV, 2 DIV+GAP (+7 in flight, not counted).
By category: behavior 4 · Robot ABC 3 · commands 5 · RobotClient 1 · telemetry 0 (folded into
row 7) · config/packaging/deploy 6. By size: L 1 · M 8 · S 10.
Down from 31 rows / 22 GAP at `874af6d`; **16 closed, 6 new, 7 in flight**.

---

## 1. waldoctl `Robot` ABC surface

`waldoctl/robot.py` defines 24 members. par6 implements every `@abstractmethod`
(`python/par6/robot.py:310-684`) and now overrides `create_dry_run_client` as well. The
remaining gaps are the collision block and two spec-content values.

| Member | parol6 | par6 | Verdict |
|---|---|---|---|
| `name`, `joints`, `native_tools`, `cartesian_limits`, `position_unit`, `joint_index_mapping`, `backend_package`, `sync_client_class`, `async_client_class`, `fk`, `ik`, `fk_batch`, `ik_batch`, `check_limits`, `set_active_tool`, `start`, `stop`, `is_available`, `create_async_client`, `create_sync_client` | ✔ | ✔ | parity |
| `has_force_torque` / `has_freedrive` | `False` (`robot.py:596,600`) | inherits `False` | parity |
| `urdf_path` / `mesh_dir` | `robot.py:475-483` | `robot.py:379-387`, per-tool tree | ✔ **fixed** — 18/18 meshes resolve `[measured]` |
| `joints.names` | functional names (`robot.py:401`) | same six names (`python/par6/config.py:136-143`) | ✔ **fixed** |
| `cartesian_limits` | constants pasted from an offline derivation (`PAROL6_ROBOT.py:603-676`) | the same derivation run live against par6's own Jacobian and config (`python/par6/robot.py:187-239,338-354`) | ✔ better |
| `create_dry_run_client` | `DryRunRobotClient` (`robot.py:989-995`) | `DryRunRobotClient` (`python/par6/robot.py:678-684`) | ✔ **fixed** |
| `motion_profiles` | 5 names | 3 names (`python/par6/robot.py:396-409`) | [row 14](#row-14) |
| `digital_inputs` / `digital_outputs` | 2 / 2, both real | 2 / 2, neither real (`python/par6/robot.py:369-375`) | [row 7](#row-7) |
| `has_collision_checking` | reflects a loaded checker (`robot.py:604-607`) | **not overridden → `False`** `[measured]` | [row 5](#row-5) |
| `in_collision` / `colliding_pairs` / `check_trajectory` / `min_distance` / `apply_shapes` | all real over `pinokin.CollisionChecker` (`robot.py:809-864`) | **all inherit the disabled defaults** `[measured]` | [row 5](#row-5) |
| `native_tools[*].meshes` / `.variants` | 1–3 meshes, 0–3 variants per tool | `()` / `()` `[measured]` | [row 11](#row-11) |
| `native_tools[*].description` / `.motions` / `.camera_spec` | real / real / `None` on every tool | real / real / `None` | ✔ parity |

### <a name="row-5"></a>Row 5 — the `Robot` collision surface is entirely default (M, GAP)

- **parol6:** a process-global `pinokin.CollisionChecker` (`PAROL6_ROBOT.py:13,57,96-129`) backs
  `in_collision`, `colliding_pairs`, `check_trajectory`, `min_distance` and `apply_shapes`
  (`robot.py:809-864`), and `has_collision_checking` reports whether it loaded.
- **par6:** none of these are overridden. `[measured]` `has_collision_checking=False`,
  `colliding_pairs=[]`, `check_trajectory=-1`, `min_distance=inf`, `apply_shapes` a no-op.
- **What changed since the first audit:** the consequence got sharper, not milder. The runtime
  now *does* enforce collision (`crates/par6d/src/planner.rs:415-480`, `gate_collisions`,
  called from the planned-motion paths at `:517` and `:540`), and par6 now *does* ship an
  offline preview. So the two disagree in the direction that costs the most trust:
  `[measured]` with a keep-out slab applied to both, `DryRunRobotClient.move_l` straight into it
  returned a 228-pose, 2.728 s path with `error=None`, while the same move on the live daemon was
  refused `[53] The planned configuration collides at sample 44 of 57: [elbow, wall]`.
- **Consequence in WC:** `services/path_visualizer.py:81` short-circuits the whole per-segment
  collision pass; `:276` skips the preview robot's world push; `urdf_scene/scene_handle.py:87,207`
  push keep-outs into nothing; `urdf_scene.py` never tints a colliding mesh.
- **Fix shape:** `pinokin.CollisionChecker` is already an installed dependency and already loads
  a URDF tree — the same one `par6.config.urdf_path` returns. This is a port, not a design.

### <a name="row-11"></a>Rows 11, 12 — tool variants (S, DIV+GAP / S, GAP)

- **Meshes (DIV, keep it):** `python/par6/tools.py:8-13` documents why `ToolSpec.meshes` is
  empty — the vendor CAD fuses the gripper body into the arm's last link, so a jaw-only mesh set
  would render jaws with no body, and the geometry reaches a 3-D view through the per-tool URDF
  tree instead. That is the better architecture and should never be "closed".
- **Variants (GAP):** parol6 exposes 2–3 variants per gripper (MSG 100/150/200, SSG-48
  finger/pinch) and WC reads them at `components/settings.py:101,150`,
  `services/path_visualizer.py:558` and `main.py:192` (`resolve_variant_tcp`). par6 exposes none
  `[measured]`, so the variant selector is empty.
- **And the wire accepts one anyway (row 12):** `[measured]`
  `select_tool("MSG_SMALL_MOTOR_150MM_RAIL", variant_key="NOT_A_VARIANT")` was **accepted**, and
  the next `STATUS.tool_status` came back `variant_key='NOT_A_VARIANT'`. The server stores and
  echoes it (`crates/par6-server/src/server.rs:1003-1013,1307`) and clears the TCP offset on a
  change, but nothing downstream re-resolves kinematics for it — `par6-kin`'s `GripperVariant`
  is the *tree* (flange / msg / ssg48), chosen at startup from `[robot].active_gripper`. Either
  validate the key against a declared set or drop the parameter. `CLAUDE.md`: *"Never ship
  declared-but-unimplemented API surface."*

### <a name="row-14"></a>Row 14 — motion profiles (S, DIV)

`[measured]` parol6 advertises `("TOPPRA","RUCKIG","QUINTIC","TRAPEZOID","LINEAR")`; par6
advertises `("RUCKIG","TRAPEZOID","TOPPRA")` and all three are now real — `[measured]`
`select_profile("TOPPRA")` returns 1 and `profile()` reads back `TOPPRA` on the shipped build.
`QUINTIC` and `LINEAR` have no par6 equivalent, and `select_profile("QUINTIC")` answers
`SYS_PROFILE_INVALID` on both the runtime and the preview `[measured]`. Defensible (RUCKIG
supersedes QUINTIC for point-to-point; LINEAR is a debug profile), but WC's profile picker is
shorter and a parol6 script naming either one errors.

---

## 2. waldoctl `RobotClient` ABC surface

`python/par6/client/async_client.py` implements every abstract method and all but two of the
optional ones. **Palette diff `[measured]`, via WC's own `Category:`-docstring scan:** par6 46
entries, parol6 47. Only-parol6: `is_estop_pressed`, `is_robot_stopped` (both
[in flight](#in-flight)). Only-par6: `wait_command`.

A bare `par6.RobotClient(host=…, port=…)` now binds the packaged tool specs
`[measured: ['FLANGE','MSG_SMALL_MOTOR_150MM_RAIL','SSG48']]`, so `rbt.select_tool(...);
rbt.tool.close()` in a user program works — the first audit's gap 10 is closed.

### <a name="row-4"></a>Row 4 — fire-and-forget refusals are invisible (S, GAP)

`teleport`, `jog_j`, `jog_l`, `servo_j`, `servo_j_pose`, `servo_l` and `reset_loop_stats` all go
through `_fire` (`python/par6/client/async_client.py:664-676`), which sends and returns `1`
without awaiting anything. The server's `on_faf` explicitly *does* answer a refusal with a real
ERROR — "Rejection gets a real ERROR even though success is unacked"
(`crates/par6-server/src/server.rs:645-655`) — and `_handle_reply` then discards it because no
future is waiting on that `req_id` (`:530-544`). Refusals reachable this way include the whole
of `validate_supported` (`server.rs:808-891`) and `check_gate` (`:745`).

**Measured:**

```
teleport([0,-90,400,0,0,180])  -> client returned 1 ; arm did not move ; error() None ; activity() IDLE
jog_j(joints=[0,1], speeds=[.3,.3]) -> client returned 1 ; arm did not move ; error() None
```

parol6 has the same fire-and-forget shape (`client/async_client.py:529-566`, `_send`) but far
less to refuse: it accepts multi-joint jog and does not range-check `teleport` at all, so the
silence costs it nothing. par6 pays for its own strictness here. The cheapest honest fix is a
short reply wait on the refusal path only — the server already sends it.

---

## 3. Command surface

parol6 registers **46** commands; par6-proto defines **48** (`crates/par6-proto/src/enums.rs`,
`CmdType`). Nothing parol6 accepts is missing from par6's vocabulary, and the list of commands
par6 defines but does not honour is now **five entries long** (it was eleven).

### Commands par6 defines but does not honour

| Command | par6 behaviour `[measured]` | parol6 | Verdict |
|---|---|---|---|
| `WRITE_IO` | always `COMM_VALIDATION_ERROR "write_io is unavailable: this runtime drives no digital outputs"` — `server.rs:481` | writes the output port | [row 7](#row-7) DIV+GAP |
| `SIMULATOR(false)` | `MOTN_SETUP_FAILED "live backend switching is not wired yet; restart par6d with/without --sim"` — `crates/par6d/src/bridge.rs:521` | live transport switch | [row 6](#row-6) GAP |
| `CONNECT_HARDWARE` | `MOTN_SETUP_FAILED "cannot switch to hardware bus … while running"` — `bridge.rs:533` | persists the port and reconnects | [row 6](#row-6) GAP |
| `SELECT_TOOL` (non-fitted) | `COMM_VALIDATION_ERROR "tool 'X' is not fitted; this runtime is running 'MSG_small_motor_150mm_rail'"` — `server.rs:791`; refuses `FLANGE` and `SSG48` alike | any registered tool | [row 8](#row-8) DIV |
| `JOG_J` with >1 non-zero axis | `COMM_VALIDATION_ERROR "jog_j drives one joint at a time"` — `server.rs:847-851`, **and the client never sees it** | multi-joint | [row 13](#row-13) DIV + [row 4](#row-4) |

Everything else was exercised end to end and works `[measured]`: `PING`, `STATUS`, `ANGLES`,
`POSE`, `IO`, `SPEEDS`, `TOOLS`, `QUEUE`, `ACTIVITY`, `LOOP_STATS`, `PROFILE`, `REACHABLE`,
`ERROR`, `TCP_SPEED`, `TCP_OFFSET`, `TOOL_STATUS`, `IS_SIMULATOR`, `SHAPES`, `RESET`, `ESTOP`,
`STOP`, `SIMULATOR(true)`, `RESET_STATE`, `SET_TCP_OFFSET`, `SET_SHAPES`,
`SET_COMPLETION_POLICY`, `SET_RECIPE`, `SERVO_J`, `SERVO_L`, `JOG_J`, `JOG_L`, `TELEPORT`,
`RESET_LOOP_STATS`, `HOME`, `MOVE_J` (abs/rel/duration/speed, with and without `r`), `MOVE_L`,
`MOVE_J_POSE`, **`MOVE_C`**, **`MOVE_S`**, **`MOVE_P`**, `SELECT_TOOL` (fitted), `DELAY`,
`CHECKPOINT`, `TOOL_ACTION` (move + calibrate), `SELECT_PROFILE` including `TOPPRA`.

### <a name="row-6"></a>Row 6 — no live simulator/hardware switch (M, GAP)

WC's Robot/Sim toggle (`components/control.py:1821-1832`) does `await client.simulator(enabled)`
then `await client.reset()`; its startup hardware-detect path (`main.py:906-915`) does the same.
Both raise on par6 `[measured]`, producing a red `ui.notify`. parol6 switches its serial
transport live (`server/transport_manager.py:166`). par6's backend is chosen by `--sim` at
process start, so honouring this means either a bus hot-swap or having `Robot.start` restart the
daemon; until one of those exists, WC's toggle is a button that always fails.

### <a name="row-7"></a>Row 7 — digital I/O (M, DIV+GAP)

The *divergence* remains justified and well-documented (`server.rs:475-485`): the Spectral CAN
protocol has no output frame and the RT core owns exactly one GPIO line, the e-stop input.
Refusing loudly beats acking a lie.

The *gap* is unchanged and is now the oldest untouched row in the register:

- `Core::io()` (`server.rs:1219-1221`) returns `[0, 0, 0, 0, !estop]` — the two inputs and two
  outputs report the un-asserted level because the wire type has no "unknown" spelling
  `[measured: io() -> [0,0,0,0,1]]`.
- `Robot.digital_inputs`/`digital_outputs` still say **2 and 2** (`python/par6/robot.py:369-375`,
  with a comment tying the count to the wire's `IO_SLOTS` layout), so WC renders four chips
  (`components/io.py:35-36`) that are permanently `0` and whose toggles always error (`:26`).
- `services/motion_recorder.py:346` still emits `rbt.write_io(port, state)` into recorded
  programs, which then fail on replay — and the dry-run client has no `write_io` at all
  `[measured]`, so previewing such a program raises `AttributeError` through WC's
  `getattr(self._client, name)` dispatch (`services/path_preview_client.py:605,645,670`).
- **Fix shape (unchanged):** decouple the ABC counts from the wire layout and report
  `digital_inputs = digital_outputs = 0` so WC renders no I/O surface at all. Cheap, and it
  turns a lie into a truthful absence.

### <a name="row-8"></a>Row 8 — `select_tool` only accepts the fitted tool (M, DIV)

The reasoning at `server.rs:786-795` is sound — the kinematic, gravity and collision models are
built from `[robot].active_gripper` at startup. But WC's dropdown is populated from
`robot.tools.available` (`components/settings.py:315-344`), which is three entries, and
`[measured]` **two of the three raise** (`SSG48` and even `FLANGE`). The runtime's own preview
mirrors the refusal (`DryRunRobotClient.select_tool` raises the same code `[measured]`), which is
good discipline but does not help the operator. Either waldoctl needs a backend-supplied
"selectable" set, or `par6d` needs to rebuild its three models on selection. Worth an explicit
decision rather than a runtime error.

### <a name="row-13"></a>Row 13 — single-joint jog (S, DIV)

`server.rs:842-852` refuses >1 non-zero axis because the RT jog engine ramps one joint at a time
with per-joint direction-block latching. WC's own jog UI sends exactly one joint per tick
(`components/control.py:1398-1406`) and one axis per tick (`:1560-1566`), so the practical impact
is confined to `jog_j(joints=…, speeds=…)` in user scripts and waldoctl's documented multi-joint
form. Low priority **as a capability** — but see [row 4](#row-4): today that script's jog is
refused *and* reported as success, which is the part that actually needs fixing.

---

## 4. Status / telemetry fields

The v2 STATUS packet (`crates/par6-proto/src/status.rs`, 31 elements) remains a superset of
parol6's. Every field waldoctl's `StatusBuffer` Protocol (`waldoctl/status.py:22-73`) requires is
present and decoded (`python/par6/protocol/wire.py:600-700`), including `collision_active`,
`collision_pairs` and `scene_epoch` (`:684-690`), which WC's status loop reads
(`main.py:1596-1600`).

The fields that were empty or dishonest in the first audit are now filled:

| Field | par6 today `[measured]` | Verdict |
|---|---|---|
| `pose` | real 4×4 in mm — `[340.42, 0.001, 333.999, …]` at boot, not `NaN` | ✔ fixed (`ec70da1`) |
| `tcp_speed` | finite, finite-differenced at status rate (`server.rs:998-1012`) | acceptable |
| `cart_en_wrf` / `cart_en_trf` | all-ones at the park pose; all-zeros before homing, i.e. *measured*, not asserted | ✔ fixed (`9f554ad`) |
| `error` | derived from the RT latch when no command-attributed error stands (`server.rs:1244-1247` → `faults.rs:41-97`); after `estop()` it reads `[51] E-stop active` | ✔ fixed (`0810311`) |
| `action_state` | `ERROR` whenever `effective_error()` is `Some` (`server.rs:1250-1262`) | ✔ fixed |
| `collision_active` / `collision_pairs` | populated from `Par6Planner::collision_latch` — **latched at the refusal**, not sampled per tick (`planner.rs:293-297`) | see below |
| `io` | `[0,0,0,0,!estop]` | [row 7](#row-7) |
| `homed`, `scene_epoch`, `tool_status` | real | ✔ |

**One nuance worth writing down:** par6's `collision_active` describes *the configuration a
planned move was blocked at* and is dropped when the next motion command is accepted. parol6's is
also set only from a failing segment (`server/segment_player.py:163,311`), so this is parity —
but neither is a live "the arm is in collision now" flag, and a WC display that reads it as one
will be wrong in both backends. It is emphatically **not** a substitute for [row 1](#row-1):
during an ungated jog into a keep-out, `[measured]` `collision_active` never fires.

---

## 5. Behaviors that are not commands

### <a name="row-1"></a><a name="row-3"></a>Streaming — rows 1, 3

**Fixed:** the first audit's `servo_j`-latches-`RtiLinkLost` regression is gone. `[measured]` at
the e2e rig's 20 Hz tick, 60 servo targets at 20 Hz moved J1 **23.62°** of a commanded 30° with
no latch and `error() -> None`. (The audit's hypothesis — a 4 ms housekeeping period colliding
with the timeout — was wrong; `0810311` found a phase-ordering off-by-one that made a one-tick
watchdog window unsatisfiable by any live stream, and floored it at two ticks.)

**Still open, and now the sharper problem:**

- **Row 3 (#21), stall on a converged target.** `[measured]` 80 repeats of one target 30° away →
  **14.27°** and stationary. `MotionStream` commands `max(profile velocity, position-channel
  advance)`; once the OTG converges on a distant fixed target both are legitimately zero while
  tracking error remains. Closing it needs the measured pose inside the stream law.
- **Row 1 (#19), no collision gate.** `[measured]` `servo_l` walked the TCP **84.8 mm** into a
  keep-out slab that the planner refuses to enter, silently. `crates/par6d/src/planner.rs`
  calls `gate_collisions` only from the planned-motion paths (`:517`, `:540`); nothing on the
  jog or stream path consults `self.collision`. parol6 gates both, and its jog gate even
  extrapolates a lookahead pose before checking (`commands/basic_commands.py:227-236`).

### <a name="row-2"></a>Simulator fidelity — row 2 (issue #22)

`[measured]`, three separate postures, `par6d --sim`:

```
move_j J1 +10.0 deg  -> completed=True, landed +5.24 deg  (47.6 % short)   [policy commanded]
move_j J1 +10.0 deg  -> completed=True, landed +5.24 deg  (47.5 % short)   [policy settled]
move_j J1 +10.0 deg  -> MOTN_SETTLE_TIMEOUT [36]; RT latched ExecSettleTimeout -> ActiveError [policy strict]
move_j J2 +10.0 deg at [0,-120,130,…] -> landed +5.04 deg (49.6 % short)
```

`settled` completes anyway because its 2 s timeout expires and the policy is documented to
complete on timeout (`crates/par6-motion/src/completion.rs:13-42`); `strict` escalates that
timeout to an error, which the RT then latches as a hard fault. So the honest summary is: **the
sim arm does not arrive, and the only policy that would tell you so takes the controller down.**
This is not a parol6-parity gap in the API sense — parol6's mock serial transport has no plant
to be unfaithful with — but it is the single most visible defect for anyone driving WC against
`--sim`, which is the default development configuration. `9074661` already had to change its own
dry-run oracle to read the *commanded* joint stream off the telemetry port rather than measured
STATUS, precisely because of this.

### <a name="row-10"></a>Collision, workspace envelope — rows 1, 5, 10

Server-side enforcement is real and correct on planned motion. `[measured]`:

```
set_shapes([Box 'wall'])            -> 1, and shapes() reads the box back
move_l through it                   -> [53] "collides at sample 44 of 57: [elbow, wall]"
move_l -120 mm z with no shapes     -> accepted, completes
move_l +120 mm x with no shapes     -> [53] "collides at sample 24 of 33: [upper_arm, lower_arm]"
```

That last one is worth a note. It is **not** demonstrably a false positive: the offline preview
refuses the same target independently, for a different reason — `[11] IK: partial path failure`
— and truncates at **the same sample index, 24**. So the arm really cannot get there. But the
mechanism that would produce false positives is documented in the code itself
(`planner.rs:306-321`, `resting_pairs`, and `park_contacts` at `:2058`): the allow-list is
derived from contact at the **one** configured park pose — the MoveIt `default_collisions` rule
applied to a single pose — because `par6-kin` has no SRDF support. `[measured]` the daemon logs
`3 link pair(s) rest in contact at the park pose and are not enforced: [base_link, upper_arm],
[base_link, elbow], [base_link, lower_arm]` at boot. Any pair that legitimately rests in contact
somewhere *else* in the workspace will refuse a move. That is issue **#18** and it stays open.

The workspace envelope (`urdf_scene/envelope_renderer.py:485-512`) loads a raw
`pinokin.Robot(urdf_path)` in a worker and reads `robot.joints.limits.position.rad`. It now
works: `[measured]` all three packaged trees load at `nq = 6` (the gripper jaw joints were
demoted to fixed in `19b603d`, which is what stopped pinokin seeing `nq = 8` against six limits).

### Error taxonomy and recovery — row 17

Codes sit in the same subsystem ranges and par6's catalogue is richer (structured
title/cause/effect/remedy on the wire; 9 extra codes including `SysRtiLinkLost`,
`SysLoopCritical`, `SysJointFault`, all now reachable via `faults.rs`). **Row 17, DIV:** two
codes are still renumbered — parol6 `SYS_PROFILE_INVALID = 53` / `SYS_SELF_COLLISION = 54`
(`parol6/utils/error_codes.py:39-41`) vs par6 `52` / `53`
(`python/par6/protocol/constants.py:169-170`), because parol6 spends 52 on
`SYS_PORT_SAVE_FAILED`, which par6 has no analogue for. `[measured]` a par6 collision refusal
arrives as `53` and a bad profile as `52`. Anything keying on the number rather than the name
mis-reads.

Recovery: par6 latches e-stop until an explicit `reset()`; parol6 auto-recovers on physical
release (`parol6/server/controller.py:362-375`). par6's behaviour matches the waldoctl contract
(`waldoctl/client.py:350-360`) and is the better one. `[measured]` end to end: `estop()` →
`error()` `[51]`, `activity().state = ERROR`, `io[4] = 0`, `move_j` refused `[51]`; `reset()` →
`error()` `None`.

### Queue semantics, checkpoints, blending, completion policy

Measured and correct: monotonic indices never reset; `stop()` clears the queue; back-to-back
moves complete in order; `checkpoint` sets `last_checkpoint`; `delay` blocks the queue;
idempotency-keyed retries re-ack the original index.

**Blending is new and works.** `[measured]` `move_l(r=30)` is accepted and *held*, its successor
folds into one path, and **both** commands report `COMPLETE` — which is the contract
`spec/PROTOCOL-V2.md` already prescribed for a motion that never rests at an interior target.
`move_c` with a radius is refused with a reason that names the limitation
(`server.rs:833-843`): an arc has no successor-blend implementation. Two deliberate divergences
from parol6, both documented at their call sites in `crates/par6-motion/src/cart.rs`: `move_p`
auto-blends its corners (parol6 defers it, but par6-proto and the client both *promise*
auto-blended corners, and this repo forbids declared-but-unimplemented surface), and the spline
uses chord-length knots with natural end conditions rather than uniform/not-a-knot. par6 also
refuses outright where parol6 quietly falls back to an unblended move. All three are parity of
capability without parity of bugs — keep them.

### Program / script execution

WC spawns user scripts in a subprocess and monkey-patches `<backend>.RobotClient`
(`services/stepping_bootstrap.py:49-74`); par6 exports `RobotClient` at package level with a
`par6.client` submodule, so the wrapper attaches. With bare-client tool binding fixed, the
remaining friction is entirely on WC's side: all six shipped programs still hardcode
`from parol6 import RobotClient` and `port=5001` (`programs/draw_circle.py:11-13`). The two
curve-heavy demos (`draw_circle.py`, `demo_showcase.py`, 10 `move_c`/`move_p`/`move_s` call
sites) would now run against par6 once ported — that was the first audit's #5 and it is gone.

### Tools / grippers

par6 supports **passive** and **electric** (verbs `move`, `calibrate`). parol6 additionally has
**pneumatic** and **vacuum**, both driven through digital outputs (`parol6/robot.py:312-329`) —
unimplementable on par6's bus, a justified divergence following from [row 7](#row-7). Tool status
telemetry (jaw position normalised to 0..1, current in mA, fault bitfield) is real
`[measured: positions=(0.502,), channels=(0.0,)]`. Tool identity is now one representation: the
TCP comes from the tool tree's own `tcp` link on both sides (`python/par6/config.py:276-318`,
`par6_kin::GripperVariant::tcp_frame`), which `55d8c93` proved agree to under 1e-3 mm.

---

## <a name="deploy"></a>6. Deployment — row 9

`ec70da1` turned the first audit's worst row inside out. `par6d` built without `ffi` **refuses to
boot** with a message naming every degradation (`crates/par6d/src/daemon.rs:50-60`, applied at
`:130-133`), instead of serving NaN poses and a `set_shapes` that answers success over an empty
world. `scripts/deploy/build-aarch64.sh` always passes `--features ffi` and fails early without
the aarch64 shim; the shim is cross-built by conda-forge's `gxx_linux-aarch64` toolchain pinned
to `sysroot_linux-aarch64=2.17`, with its runtime library closure staged beside it.

**What remains (row 9, M, GAP):** by the deploy README's own admission
(`scripts/deploy/README.md:212-229`), *"Nothing has been executed on aarch64 … the shim's
numerics are unverified on this target: the golden kinematics and collision fixtures only ever
ran against the x86_64 shim."* Three static checks stand in for a smoke test — no unresolved
sonames, a verified `GLIBC_2.17` floor, every versioned symbol provided by a shipped copy — and
they are good checks, but they say nothing about whether IK converges to the same answer on ARM.
The control box is the *only* place the arm actually moves. Until someone runs
`cargo test -p par6-kin --features ffi` and `cargo test -p par6d --features ffi` natively on the
Pi, aarch64 kinematics are **built, not validated**, and that is a deployment risk rather than a
parity gap against parol6 (whose pure-Python numerics are ISA-independent by construction).

---

## <a name="config"></a>7. Config surface — rows 16, 18, 19

- **[row 16]** `par6d` defaults to command port **6001** (`python/par6/robot.py:274`); WC
  defaults to **5001** (`waldo_commander/constants.py:54`) and passes it explicitly into both
  `create_async_client` (`main.py:1768`) and `start` (`main.py:288-296`). Out of the box they do
  not meet. Fixed by `WALDO_CONTROLLER_PORT=6001` or `PAR6_COMMAND_PORT=5001`, but it needs to be
  written somewhere a user will find it.
- **[row 18]** `pip install parol6` ships the whole server plus a `parol6-server` console script.
  `pip install par6` ships only the client (`python/pyproject.toml`); `par6d` must arrive
  separately (`PAR6D_BIN`, then PATH — `python/par6/robot.py:72-85`). Since `ec70da1` the story
  is *heavier*, not lighter: the binary that ships now hard-requires the Pinocchio/coal C++ shim
  and its staged library closure. Architecturally unavoidable, but WC's `[par6]` extra needs to
  say so.
- **[row 19]** Partly improved. `par6d` now reads `PAR6_CONFIG`, `PAR6_ASSETS`,
  `PAR6_COMMAND_PORT`, `PAR6_BIND`, `PAR6_STATUS_*`, `PAR6_TELEMETRY_PORT`,
  `PAR6_STATUS_TRANSPORT` and `PAR6_SIM_DYNAMICS` (`crates/par6d/src/options.rs:26-46`), and
  `par6.Robot` honours `PAR6_HOST` / `PAR6_COMMAND_PORT` (`python/par6/robot.py:272-274`). What
  is still missing is the *rate* knob: WC's conftest sets `PAROL6_STATUS_RATE_HZ=20` to cut CI
  load, and the par6 equivalent is "write a patched TOML", which is what
  `python/tests/live_daemon.py:87-117` does. Fine for par6's own tests, awkward for WC's.

---

## 8. What Waldo Commander calls that par6 cannot satisfy

This is the practical definition of "must work", re-swept against current main. It is much
shorter than last time.

| WC call site | par6 result | Row |
|---|---|---|
| `components/control.py:1560` `client.jog_l(...)`, `:1539` `client.servo_l(...)` near a keep-out | drives straight through it, silently | [1](#row-1) |
| any `move_j` / `move_l` under `--sim` | lands ~48 % short of the commanded angle and reports success | [2](#row-2) |
| servo-driven teleop / playback holding a target | arrives less than half way and stops | [3](#row-3) |
| `components/control.py` multi-axis jog, `components/playback.py:632` `client.teleport(...)` | refused server-side; client told `1`, nothing reported | [4](#row-4) |
| `services/path_visualizer.py:81,88,107,112,118` `has_collision_checking` / `apply_shapes` / `check_trajectory` | silent no-ops; preview draws paths the runtime refuses | [5](#row-5) |
| `services/urdf_scene/scene_handle.py:87,207` `robot.apply_shapes` | no-op | [5](#row-5) |
| `components/control.py:1821` `client.simulator(...)`, `main.py:906-915` auto robot-mode switch | raises → red toast | [6](#row-6) |
| `components/io.py:26` `client.write_io(index, state)`; `:35-36` four chips | raises; chips permanently `0` | [7](#row-7) |
| `services/motion_recorder.py:346` recorded `rbt.write_io(...)` | fails on replay; `AttributeError` in preview | [7](#row-7) |
| `components/settings.py:315-344` tool dropdown → `client.select_tool(...)` | raises for 2 of 3 entries | [8](#row-8) |
| `components/settings.py:101,150` variant dropdown from `spec.variants` | empty | [11](#row-11) |
| `main.py:1768` `create_async_client(port=config.controller_port)` | 5001 ≠ 6001 | [16](#config) |

**Now satisfied by par6, verified end to end `[measured]`** — everything the first audit listed
as broken except the rows above: `main.py:199` `package_map={robot.backend_package: mesh_dir}`
(18/18 meshes resolve), `path_visualizer.py:527` `robot.create_dry_run_client()` (returns a real
client that plans arcs, splines, process moves and blended chains), `client.move_l` ×41 across
the app and programs, `components/readout.py:333-414` pose bindings (real numbers, no `nan`),
cartesian jog buttons (`cart_en` all-ones at the park pose), every queued command at startup
(boot-enabled 0.61 s after ready, no `reset()` needed), `main.py:1619-1640` error banner and
action log (RT latches surface), `robot.tools[status.tool.key]` at all five sites (one canonical
spelling), the `move_c`/`move_p`/`move_s` demo programs, `robot.joints.names`, `tools.default`,
`robot.cartesian_limits`, plus `ping`, `wait_ready`, `stream_status_shared` and every
`StatusBuffer` field WC's status loop reads, `angles`, `move_j` (all four timing forms), `jog_j`,
`servo_j`, `teleport`, `stop`, `estop`, `reset`, `reset_state`, `select_profile` (all three,
including `TOPPRA`), `set_tcp_offset`, `tcp_offset`, `set_shapes`, `shapes`, `checkpoint`,
`wait_command`, `wait_checkpoint`, `wait_motion`, `tools`, `activity`, `queue`, `status`,
`tool_action`, and `scene_epoch`-driven world readback.

---

## <a name="closed"></a>Closed since the first audit

Sixteen rows. Where the fix differs from what the audit proposed, that is noted.

| Old # | Gap | Closed by | Note |
|---|---|---|---|
| 1 | Cartesian + collision + TOPPRA behind an off-by-default `ffi` feature | `ec70da1` | **Differs:** the audit proposed making `ffi` a default cargo feature. Instead `ffi` stays non-default (so a plain workspace build needs no C++ toolchain) and a build without it **refuses to boot**; CI and the deploy path carry the flag, and the aarch64 shim is now cross-built. `[measured]` real pose, `cart_en` all-ones, `TOPPRA` selectable. |
| 2 | Boots DISABLED; `reset()` reports success unconditionally | `0810311` | `[measured]` first queued command accepted **0.61 s** after `PAR6D_READY` with no `reset()`. `reset()` now waits for the RT's actual enable verdict via an `enable_seq` baseline (`server.rs:574-613`) rather than the audit's suggested trust window. |
| 3 | RT hard errors never reach `error()` / `activity()` / `STATUS.error` | `0810311` | **Differs (better):** the audit proposed *mirroring* the latch into `standing_error`. It is instead **derived, never stored** (`faults.rs`, `server.rs:1244-1247`), so accepting a new command cannot clear a fault the arm still has. |
| 4 | URDF `package://` name ≠ `backend_package`; meshes unresolvable | `19b603d` | The packaging sync script rewrites the package name on copy and the freshness guard compares against that transform. `[measured]` 18/18 meshes resolve. Also demoted the gripper jaw joints to fixed, which fixed `nq=8` in WC's envelope worker. |
| 5 | No `create_dry_run_client` | `19b603d`, extended by `9074661` | **Differs:** rather than reusing `ik_batch`/`fk_batch`, `python/par6/motion.py` ports the runtime's own planning path (ruckig, a line-by-line TRAPEZOID port, TOPPRA configured as the shim configures it) and `9074661` ported the arc/spline/blend geometry from `crates/par6-motion/src/cart.rs`. Agreement with a live daemon is measured, not asserted. |
| 6 | `move_c` / `move_s` / `move_p` unimplemented | `ec70da1` | One cartesian pipeline; the four move families differ only in which generator builds the pose list. `[measured]` all three accepted and completed. |
| 7 | Blend radius `r` refused; no multi-command lookahead | `ec70da1` | The planner takes a batch and the server holds a positive-radius head for its successor. `[measured]` a held `move_l(r=30)` + successor both complete. |
| 8 | Tool keys upper-cased client-side, config-cased on the wire | `19b603d` | Canonicalised at the client boundary; the wire layer stays a verbatim mirror of par6-proto. `[measured]` `STATUS.tool_status.key == 'MSG_SMALL_MOTOR_150MM_RAIL'` matches `robot.tools`. |
| 9 (part) | `ToolSpec`s carry no `description` | `19b603d` | Descriptions built from the gripper TOML's own numbers; `motions` real; `camera_spec` is parity (parol6 defines none either). Meshes/variants remain — see [row 11](#row-11). |
| 10 | Bare `RobotClient()` binds no tools | `19b603d` | `[measured]` all three specs bound on a bare client. |
| 15 | `servo_j` latches `RtiLinkLost` at reduced tick rates | `0810311` | **Differs:** the audit's hypothesis (a 4 ms housekeeping period) was wrong; the cause was a phase-ordering off-by-one making a one-tick watchdog window unsatisfiable. Floored at two ticks; 250 Hz untouched. `[measured]` 23.6° of 30° with no latch. |
| 18 | `CARTESIAN_JOG_LIMITS` are invented constants | `19b603d` | **Differs (better):** rather than reading them from config as parol6 does, par6 runs parol6's own Jacobian-pseudoinverse derivation live against its own model, cached per tool, taking acceleration from the jog ramp's actual profile. `[measured]` v=(0.244 m/s, 1.236 rad/s), a=(0.449, 2.558). |
| 19 | Joint names are `joint1…joint6` | `19b603d` | `[measured]` `('Base','Shoulder','Elbow','Wrist 1','Wrist 2','Wrist 3')`. |
| 20 | `tools.default` is `FLANGE` while the runtime is fitted with a gripper | `19b603d` | `[measured]` default is now the config's `active_gripper`. |
| 23 | `teleport` silently clamps to hard limits | `0810311` | Refuses naming the joint, the request and the window (`server.rs:1536-1550`). `[measured]` J3=400° → arm does not move. **But the refusal never reaches the client** — that is the new [row 4](#row-4). |
| — | TCP frame defined twice (client vs runtime) | `55d8c93` | Not in the first register; found and closed after it. `config.tool_tcp` deleted; both sides read the tree's own `tcp` link. Agreement measured to <1e-3 mm. |
| — | IK returned solutions the planner then rejected | `55d8c93` | Solutions wrapped into the joint's soft window, branch nearest the seed. 897 of 3000 random reachable targets had been failing for want of wrapping alone. |

---

## <a name="in-flight"></a>In flight / already tracked — do not re-file

Another agent is closing these concurrently; they were **not** re-verified for this pass and
their status here is the first audit's.

- **`home()` always re-runs full referencing** (old row 14) — and lands at the homing sequence's
  final pose, not `joints.home`. `[measured, this pass, indirectly]` the dry-run client mirrors
  it: `DryRunRobotClient.home()` moves to `[89.95, -106.0, 163.29, 0, -28.65, 180]`, not the
  park pose, and returns a single-sample zero-duration result (so the preview draws no home path).
- **No log forwarding** (old row 22) — `par6d`'s output still goes to an unnamed
  `tempfile.NamedTemporaryFile` (`python/par6/robot.py:107-119`); WC's log panel is empty.
- **`queued_duration` is 0 for speed-parameterised moves** (old row 24) —
  `[measured: -0.0 with a queue in flight]`.
- **`loop_stats` std / min / p95 hardcoded 0.0** (old row 25) —
  `[measured: std=0.0, min=0.0, p95=0.0, p99=0.0]`.
- **`pose(frame="TRF")` returns identity** (old row 26) — `[measured: [0,0,0,0,-0,0]]`.
- **`is_estop_pressed` / `is_robot_stopped` missing** (old row 27) — still absent from the
  palette `[measured]`.
- **`Robot.start()` ignores `com_port`; reuses a running runtime** (old row 29) — WC's
  `EXCLUSIVE_START` contract (`main.py:288-296`) is fail-hard; par6 reuses silently
  (`python/par6/robot.py:616-617`).

Also tracked as issues, folded into the register above rather than re-filed:

- **#18** — SRDF support in `par6-kin`; today the allow-list is derived from park-pose contact
  ([row 10](#row-10), discussed in §5).
- **#19** — streaming/jog collision gating, escape depth, `installation_shapes` config producer
  ([row 1](#row-1)).
- **#21** — servo stall on a converged target ([row 3](#row-3)).
- **#22** — sim plant settles short and does not converge ([row 2](#row-2)).
- **#17, #20** — closed (collision enforcement wired; the TCP frame unified in `55d8c93`).

---

## <a name="where-par6-is-ahead"></a>Where par6 is ahead

Parity of *capability* is the goal, not parity of bugs. Several of these are load-bearing
divergences that should **never** be "closed":

1. **Explicit refusal over silent degradation.** `validate_supported` (`server.rs:808-891`)
   refuses a parameter the runtime cannot honour instead of dropping it — including, since
   `0810311`, out-of-range `teleport` angles. parol6 has several silent-ignore paths and clamps
   nothing. (The one place par6 breaks its own rule is the last hop to the client:
   [row 4](#row-4).)
2. **A build that cannot do the job does not run.** A `par6d` without kinematics exits with an
   actionable message instead of broadcasting NaN and acking an empty collision world
   (`daemon.rs:50-60,118-133`).
3. **Errors are derived from the RT latch, never stored.** A fault stops being reported exactly
   when the RT stops latching it (`faults.rs`), so accepting a command cannot paper over a
   bricked arm.
4. **Push completion + acceptance ordering.** `COMPLETE` pushes and the `accepted_index`
   stale-error rule replace parol6's poll-only completion and make `wait_command` correct under
   packet loss.
5. **Idempotency-keyed enqueue**, and a **monotonic index allocator that survives `reset_state`**
   — parol6 can double-enqueue on retry and can satisfy a post-reset wait with a pre-reset frame.
6. **Structured errors on the wire** — `[command_index, code, title, cause, effect, remedy]`,
   server-formatted (`crates/par6-proto/src/error.rs`), versus parol6's thinner catalogue.
7. **Status header:** `seq`, `mono_time_ns`, `link_ok`, `data_age_ms`, `controller_id`. STATUS
   keeps broadcasting when the bus link is down instead of going silent.
8. **Blending refuses rather than falls back.** A composite path that fails IK, soft limits or
   collision errors; parol6 quietly runs the unblended move instead.
9. **Cartesian enablement is measured, not asserted.** A 0.5 mm / 0.5° probe per direction,
   starting from all-zeros so an unmeasured direction reports "not allowed"
   (`9f554ad`). `[measured]` all-zeros before homing, all-ones at the park pose.
10. **One TCP representation.** The tool frame is the tool tree's own `tcp` link on both sides,
    so preview and runtime cannot drift (`55d8c93`).
11. **`scene_epoch` readback truth**, **cross-language golden vectors**, a **generated Python
    constants mirror**, **`--sim` as a real closed-loop plant**, **tool-dependent
    `urdf_path`/`mesh_dir`**, **config-as-TOML**, and **`SET_COMPLETION_POLICY` / `SET_RECIPE`**
    as first-class wire commands.

---

## 9. What "parity" means now

Most of the register is closed, so the honest answer is no longer a list — it is three
statements.

**What a Waldo Commander user would still notice.** Four things, in order. (a) In `--sim` — the
default development configuration — the arm does not reach the angle it was told to, by about
half, and says it did. (b) Jog and servo drive through keep-outs that planned motion refuses,
with no warning; and a held servo target arrives less than half way. (c) The editor's path
preview and the runtime disagree about collision, because the client-side checker does not
exist — the preview will happily draw a path the arm then refuses. (d) Three buttons raise:
the Robot/Sim toggle, the I/O output toggles, and two of the three entries in the tool dropdown.
Everything else in the first audit's WC breakage table now works.

**What only matters on hardware.** The aarch64 shim's numerics have never executed on the target
ISA — a real deployment risk that no amount of x86 CI retires, and the reason
[row 9](#deploy) is ranked where it is. The digital-I/O absence ([row 7](#row-7)) is also a
hardware fact rather than a software gap: the bus has no output frame. The e-stop recovery
divergence (latched until `reset()` vs parol6's auto-recovery on release) only shows up with a
physical e-stop, and par6's behaviour is the correct one.

**What is a justified divergence and should never be closed.** Refusing a parameter the runtime
cannot honour instead of silently altering it. Refusing to boot without kinematics. Refusing a
blended path that fails its checks instead of falling back to an unblended move. Latching e-stop
until an explicit reset. Delivering tool geometry through per-tool URDF trees instead of loose
`ToolSpec.meshes`. Deriving the cartesian envelope from the live Jacobian instead of carrying
pasted constants. Measuring cartesian enablement instead of asserting it. Shipping `par6d`
outside the pip package. `move_p` auto-blending its corners where parol6 defers. In each case
par6 is doing the more correct thing, and "parity" would be a regression.

---

## Appendix A — how the measurements were taken

All `[measured]` claims come from a live `par6d --sim` **built with `--features ffi`** — the
shipped configuration — driven by the real `par6` client over real UDP, using the repo's own rig
(`python/tests/live_daemon.py`, 20 Hz tick, ephemeral ports). No test suite was run; the probes
are standalone scripts.

```bash
cd /workspace/par6
source .ffi/env.sh
cargo build -p par6d --release --features ffi
export PAR6D_BIN=/workspace/par6/target/release/par6d PYTHONPATH=/workspace/par6/python

python3 probe1.py   # boot state, cartesian surface, standing refusals, query sweep
python3 probe2.py   # boot-enable timing, cart_en, select_tool, move_c/move_s/move_p, blending
python3 probe3.py   # servo streams, sim settling, collision enforcement, estop fault visibility
python3 probe4.py   # collision false-positive check, variant keys, completion policies
python3 probe5.py   # the dry-run client, offline, against the same commands
python3 probe6.py   # jog/servo collision gating (#19), fire-and-forget refusal visibility
python3 static.py   # the Robot ABC surface, mesh resolution, palette diff vs parol6
```

Note on joint windows: PAR6's J3 hard travel is `[109.09, 381.02]` deg and J2's is
`[-144.84, -1.43]` deg (`config/PAR6.toml`), so every probe parks at the configured park pose
`[0, -90, 180, 0, 0, 180]`. A teleport to a pose outside those windows is now *refused* — which
is itself one of the measurements ([row 4](#row-4)).

Static measurements were taken by importing the packages directly:

```python
from par6.robot import Robot; r = Robot()
r.has_collision_checking            # False
r.check_trajectory(...)             # -1        (waldoctl's disabled default)
r.min_distance(...)                 # inf       (ditto)
type(r.create_dry_run_client())     # DryRunRobotClient
r.joints.names                      # ('Base','Shoulder','Elbow','Wrist 1','Wrist 2','Wrist 3')
[t.key for t in r.native_tools.available]
                                    # ['FLANGE','MSG_SMALL_MOTOR_150MM_RAIL','SSG48']
r.tools.default.key                 # 'MSG_SMALL_MOTOR_150MM_RAIL'  (the fitted gripper)
[(t.meshes, t.variants) for t in r.native_tools.available]   # all ((), ())
r.cartesian_limits                  # v (0.2441 m/s, 1.2364 rad/s), a (0.4487, 2.5577)

# WC's package:// resolution, replayed against par6's URDF
#   package_map = {'par6': r.mesh_dir}; 18 meshes, 0 unresolvable
#   package://par6/meshes/base_link.STL -> .../par6_msg_gripper/meshes/base_link.STL  exists=True

# bare client binds tools
from par6 import RobotClient
RobotClient(host='127.0.0.1', port=6001)._bound_tools
                                    # ['FLANGE','MSG_SMALL_MOTOR_150MM_RAIL','SSG48']

# editor palette diff, using WC's own Category:-docstring scanner logic
# par6 46 entries; parol6 47; only-parol6: is_estop_pressed, is_robot_stopped; only-par6: wait_command
```
