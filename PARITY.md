# par6 ↔ parol6 parity audit

**Question answered:** what does "par6 matches parol6" still require?

**Reference:** `parol6` 0.4.0, source at `/usr/local/lib/python3.11/dist-packages/parol6/`.
**Subject:** this repo at `874af6d` (+ an agent's in-flight `crates/par6d/**` edits — see
[In flight](#in-flight)).
**Contract spine:** `waldoctl` 0.7.0 at `/usr/local/lib/python3.11/dist-packages/waldoctl/`.
**Consumer of record:** Waldo Commander at `/home/user/Waldo-Commander` (WC).

Every claim below is either a source citation on both sides or a **measurement** against a
live `par6d --sim` driven by the real `par6` client over real UDP. Measurements are marked
`[measured]` and the reproduction is in [Appendix A](#appendix-a--how-the-measurements-were-taken).
`crates/par6d/src/planner.rs` line numbers were read during the audit while another agent was
editing that file; symbol names are given alongside so they survive drift.

---

## If you only fix five things

| # | Fix | Why it is first |
|---|---|---|
| 1 | **Ship `par6d` built with `--features ffi`, and make an `ffi`-less build refuse to start rather than degrade.** | Without it there is no Cartesian anything (`move_l`, `move_j_pose`, `servo_l`, `servo_j_pose`, `jog_l` all error), TCP pose broadcasts **NaN**, `cart_en` is all-zero, and the collision world that #17 just landed is compiled out. `move_l` is WC's single most-used call (41 sites). It is off by default (`crates/par6d/Cargo.toml:9-22`) and the aarch64 deploy path explicitly cannot build it (`scripts/deploy/README.md:171`). |
| 2 | **Make controller state and RT errors visible, and make `reset()` honest.** | The runtime boots **DISABLED**; every queued command is refused with `SYS_CONTROLLER_DISABLED` until a client sends `reset()`, which WC never does at startup. `reset()` returns `1` ("confirmed") whether or not the RT accepted the enable. When the RT latches a hard error, `error()` returns `None`, `activity()` says `IDLE`, `action_state` stays `IDLE`, and `link_ok` stays `1` — the arm is bricked and the client is told nothing. parol6 boots `enabled = True`. |
| 3 | **Fix the 3-D scene: URDF package naming, `mesh_dir`, and tool-key casing.** | WC builds `package_map = {robot.backend_package: robot.mesh_dir}`, i.e. `{"par6": …/par6_flange}`, but the packaged URDF says `package://par6_flange/…`. The name does not match, WC falls back to the URDF's own directory, and **every mesh path resolves to a file that does not exist** `[measured]`. Separately `robot.tools` is keyed `MSG_SMALL_MOTOR_150MM_RAIL` while the wire reports `MSG_small_motor_150mm_rail`, so `robot.tools[status.tool.key]` raises `KeyError` at four WC sites. Result: no arm meshes, no tool meshes, no tool TCP in the viewport. |
| 4 | **Ship a `DryRunClient`.** | `Robot.create_dry_run_client()` is not overridden, so it returns `None` (`waldoctl/robot.py:315-317`). WC logs *"Backend par6 does not support dry-run simulation"* and returns — killing the editor's path preview, move targets, per-line path segments, timing feasibility, and the playback timeline. parol6 ships 585 lines of it (`parol6/client/dry_run_client.py:154`). This is the largest single feature loss and it is invisible in the command surface. |
| 5 | **Implement `move_c` / `move_s` / `move_p`, and either implement blend radius or stop advertising `r`.** | `par6d` refuses all three with `MOTN_SETUP_FAILED "arc/spline/process moves are not implemented yet"` (`crates/par6d/src/planner.rs`, `Par6Planner::start`, ~:1274) `[measured]`, and refuses any non-nil `r` (`crates/par6-server/src/server.rs:719-729`) `[measured]`. Two of WC's three shipped demo programs are nothing but `move_c`/`move_p`/`move_s`. parol6 implements all three plus multi-command blending with lookahead (`parol6/server/motion_planner.py:293-345`). |

---

## Gap register (ranked by user-visible impact)

Verdict key: **GAP** = genuine parity hole · **DIV** = justified architectural divergence
(still worth documenting) · **DIV+GAP** = the divergence is defensible but the way it surfaces
to the user is not.

| # | Gap | Category | Consequence | Size | Verdict |
|---|---|---|---|---|---|
| 1 | Cartesian + collision + TOPPRA are behind an off-by-default `ffi` feature | build/commands | `move_l`, `move_j_pose`, `servo_l`, `servo_j_pose`, `jog_l` error; pose = NaN; `cart_en` = 0; collision never enforced; `TOPPRA` refused though advertised | L | GAP |
| 2 | Boots DISABLED; `reset()` reports success unconditionally | behavior | Nothing moves until someone presses Reset; success is reported when the enable was refused | M | GAP |
| 3 | RT hard errors never reach `error()` / `activity()` / `STATUS.error` | telemetry | Controller is disabled and refusing everything while the UI shows *idle, no error* | M | GAP |
| 4 | URDF `package://` name ≠ `backend_package`; meshes unresolvable in WC | Robot ABC | Empty 3-D viewport | S | GAP |
| 5 | No `create_dry_run_client` | Robot ABC | Editor path preview, targets, timeline all dead | L | GAP |
| 6 | `move_c` / `move_s` / `move_p` unimplemented | commands | Shipped demo programs fail; palette advertises them | L | GAP |
| 7 | Blend radius `r` refused; no multi-command lookahead | commands | Arm stops at every waypoint; `r=` in scripts errors | M | GAP |
| 8 | Tool keys upper-cased client-side, config-cased on the wire | Robot ABC | `robot.tools[status.tool.key]` → `KeyError` at 4 WC sites | S | GAP |
| 9 | `ToolSpec`s carry no `meshes` / `variants` / `description` / `camera_spec` | Robot ABC | No tool geometry, no variants, no gripper camera | M | DIV+GAP |
| 10 | Bare `RobotClient()` binds no tools | RobotClient | `rbt.tool.close()` in a user script → `KeyError` | S | GAP |
| 11 | `write_io` always errors; `io` inputs/outputs hardcoded to 0 | commands/telemetry | I/O panel is inert and dishonest; recorded `rbt.write_io()` lines fail on replay | M | DIV+GAP |
| 12 | `simulator(false)` / `connect_hardware` not wired | commands | WC's Robot/Sim toggle and hardware-detect path both fail | M | GAP |
| 13 | `select_tool` accepts only the fitted tool | commands | WC tool dropdown errors for every other tool | M | DIV |
| 14 | `home()` always re-runs full referencing | behavior | Home button takes a full seek even when homed; ends at the referenced pose, not `joints.home` | M | GAP |
| 15 | `servo_j` latches `RtiLinkLost` at reduced tick rates | behavior | Streaming bricks the controller silently | M | GAP |
| 16 | `jog_j` refuses multi-joint | commands | Multi-joint jog scripts error (WC's own UI jogs one at a time) | S | DIV |
| 17 | `Robot` collision surface is entirely default | Robot ABC | `has_collision_checking=False`; preview/editing-pose collision does nothing | M | GAP |
| 18 | `CARTESIAN_JOG_LIMITS` are invented constants | config | Jog UI scaling is a guess; runtime enforces no Cartesian limits | S | GAP |
| 19 | Joint names are `joint1…joint6` | config | Readout shows URDF ids instead of `Base/Shoulder/Elbow/…` | S | GAP |
| 20 | `tools.default` is `FLANGE` while the runtime is fitted with a gripper | Robot ABC | UI shows the wrong tool before the first STATUS | S | GAP |
| 21 | Default command port 6001 vs WC's 5001 default | config | Out-of-the-box WC cannot find the runtime | S | DIV |
| 22 | No log forwarding (`normalize_logs` unsupported) | behavior | `par6d` logs go to an unnamed temp file; WC's log panel is empty | S | GAP |
| 23 | `teleport` silently clamps to hard limits | behavior | Violates the repo's own "refuse, never silently alter" rule | S | GAP |
| 24 | `queued_duration` is 0 for speed-parameterised moves | telemetry | Queue ETA is always 0 | S | GAP |
| 25 | `loop_stats` std / min / p95 hardcoded 0.0 | telemetry | Three of ten metrics are fake | S | DIV+GAP |
| 26 | `pose(frame="TRF")` returns identity | commands | parol6 returns world-in-tool; par6's answer carries no information | S | DIV |
| 27 | `is_estop_pressed` / `is_robot_stopped` missing | RobotClient | Two palette entries fewer; scripts using them break | S | GAP |
| 28 | Error code numbering diverges from parol6 (52/53 shifted) | commands | Scripts keying on numeric codes mis-read | S | DIV |
| 29 | `Robot.start()` ignores `com_port`; reuses a running runtime | Robot ABC | WC's `EXCLUSIVE_START` semantics silently lost | S | DIV+GAP |
| 30 | No runtime binary in the pip package | packaging | `pip install par6` gives no `par6d`; parol6's install is self-contained | M | DIV |
| 31 | Status rate / transport not env-overridable | config | WC's conftest pattern (`PAROL6_STATUS_RATE_HZ=20`) has no analogue | S | DIV |

**Counts:** 31 gaps — 22 GAP, 5 DIV, 4 DIV+GAP. By category: Robot ABC 7 · RobotClient 2 ·
commands 9 · telemetry 4 · behavior 6 · config/packaging 5. By size: L 3 · M 12 · S 16.

---

## 1. waldoctl `Robot` ABC surface

`waldoctl/robot.py` defines 24 members. par6 implements every `@abstractmethod`
(`python/par6/robot.py:316-670`) — nothing raises `NotImplementedError`. The gaps are in the
*optional* members with concrete defaults, and in the *values* two implemented members return.

| Member | parol6 | par6 | Verdict |
|---|---|---|---|
| `name`, `joints`, `native_tools`, `cartesian_limits`, `position_unit`, `digital_inputs/outputs`, `joint_index_mapping`, `backend_package`, `sync_client_class`, `async_client_class`, `fk`, `ik`, `fk_batch`, `ik_batch`, `check_limits`, `set_active_tool`, `start`, `stop`, `is_available`, `create_async_client`, `create_sync_client` | ✔ | ✔ | parity |
| `has_force_torque` / `has_freedrive` | `False` (`robot.py:596,600`) | inherits `False` | parity |
| `motion_profiles` | `ProfileType` enum (`robot.py:542`) | `("RUCKIG","TRAPEZOID","TOPPRA")` (`robot.py:439`) | see [gap 1](#gap-1) |
| `urdf_path` / `mesh_dir` | `robot.py:475-483` | `robot.py:410-417` | see [gap 4](#gap-4) |
| `has_collision_checking` | overridden (`robot.py:604-607`) | **not overridden → `False`** | [gap 17](#gap-17) |
| `in_collision` / `colliding_pairs` / `check_trajectory` / `min_distance` / `apply_shapes` | all real (`robot.py:809-864`) | **all inherit the disabled defaults** | [gap 17](#gap-17) |
| `create_dry_run_client` | `DryRunRobotClient` (`robot.py:989-995`) | **not overridden → `None`** | [gap 5](#gap-5) |

### <a name="gap-4"></a>Gap 4 — URDF meshes do not resolve in WC (S, GAP)

- **parol6:** URDF declares `package://parol6/meshes/…`; `backend_package` is `"parol6"`;
  `mesh_dir` is `urdf.parent.parent` (`robot.py:480-483`). WC's
  `package_map = {robot.backend_package: mesh_dir}` (`Waldo-Commander/waldo_commander/main.py:199`)
  therefore hits.
- **par6:** URDF declares `package://par6_flange/…`
  (`python/par6/_data/urdf/par6_flange/urdf/par6_flange.urdf`); `backend_package` is `"par6"`
  (`python/par6/robot.py:444-445`); `mesh_dir` is the *tree root* (`robot.py:415-417`).
- **Measured:** the package name is absent from the map, WC falls back to the URDF's own
  directory (`Waldo-Commander/waldo_commander/services/urdf_scene/loader.py:60-65`), and
  `package://par6_flange/meshes/base_link.STL` resolves to
  `…/par6_flange/urdf/meshes/base_link.STL` — **`exists=False`**.
- **Consequence:** empty 3-D viewport, and a warning buried in the log.
- **Fix shape:** either rename the packaged URDF's package to `par6` (and point `mesh_dir` at
  the directory that makes `package://par6/meshes/…` resolve), or teach `mesh_dir`/`urdf_path` to
  agree with the package name the URDF actually uses. Note that `urdf_path`/`mesh_dir` are
  *tool-dependent* in par6 (a real improvement over parol6), so whatever is chosen must hold for
  all three trees.

### <a name="gap-5"></a>Gap 5 — no dry-run client (L, GAP)

- **parol6:** `DryRunRobotClient` (`client/dry_run_client.py:154-…`) runs the *real* trajectory
  planner in diagnostic mode with no UDP/serial, returning `DryRunResult` per command.
- **par6:** no override; `waldoctl/robot.py:315-317` returns `None`.
- **Consequence:** `Waldo-Commander/waldo_commander/services/path_visualizer.py:527-534` logs
  *"Backend par6 does not support dry-run simulation"* and bails. Everything downstream —
  `commander.programs[*].dry_run` targets, `PathSegment`s, timing feasibility, checkpoint
  markers, the playback scrubber — never populates. `path_preview_client.py:522` also calls
  `checkpoint()` on it.
- **Note:** par6 has the raw materials (`crates/par6-motion`, `par6d --sim`). A dry-run client
  built on a headless `par6d` instance, or on a Python port of the profile math, are both
  plausible; a Python `DryRunClient` that reuses `Robot.ik_batch`/`fk_batch` is the smallest
  version that unblocks the editor.

### <a name="gap-17"></a>Gap 17 — the `Robot` collision surface is entirely default (M, GAP)

- **parol6:** a process-global checker (`robot.py:797-864`) backs `in_collision`,
  `colliding_pairs`, `check_trajectory`, `min_distance`, `apply_shapes`, and
  `has_collision_checking` reflects whether it loaded.
- **par6:** none of these are overridden. `has_collision_checking` is `False` `[measured]`,
  `colliding_pairs` returns `[]`, `check_trajectory` returns `-1`, `apply_shapes` is a no-op.
- **Consequence:** `Waldo-Commander/waldo_commander/services/urdf_scene/urdf_scene.py:1467`
  never tints a colliding mesh; `services/urdf_scene/scene_handle.py:87,207` push keep-outs into
  a checker that does not exist; `services/path_visualizer.py:81-118` skips per-segment collision
  entirely.
- **Divergence note:** the *runtime-side* collision world does exist (`crates/par6-kin`,
  wired in #17) — but only under `ffi`, and only server-side. The client-side twin waldoctl asks
  for (preview and editing-pose queries, which never touch the controller) has no implementation
  at all. `crates/par6-kin` has no Python binding; `pinokin` is already a dependency and is the
  obvious host.

### <a name="gap-8"></a>Gaps 8, 9, 20 — the tool spec surface

- **Casing (gap 8):** `python/par6/config.py:44-52` keys gripper configs by
  `cfg["name"].strip().upper()`, so `robot.tools` exposes `FLANGE`,
  `MSG_SMALL_MOTOR_150MM_RAIL`, `SSG48` `[measured]`. The runtime's `TOOLS` query and
  `STATUS.tool_status.key` report the *config* spelling — `Flange`,
  `MSG_small_motor_150mm_rail`, `SSG48` `[measured]`. `ToolsCollection.__getitem__`
  (`waldoctl/tools.py:743-744`) is an exact dict lookup, so `robot.tools[status.tool.key]`
  raises at `Waldo-Commander/waldo_commander/main.py:185`, `components/settings.py:98,147,344`
  and `services/urdf_scene/urdf_scene.py:1773` — the last of which then *clears the tool meshes*.
  Server-side matching is already case-insensitive (`crates/par6-server/src/server.rs:684-700`);
  the Python side must agree on one spelling and use it everywhere.
- **Spec content (gap 9):** `_build_tools` (`python/par6/robot.py:230-263`) passes only `key`,
  `display_name`, `tcp_origin`, `tcp_rpy`, plus ranges and one `LinearMotion`. parol6 passes
  `description`, `meshes`, `motions`, `variants`, `camera_spec` (`parol6/robot.py:445-456`).
  par6's answer to tool geometry is per-tool URDF trees, which is the better architecture — but
  WC reads `spec.meshes` / `spec.variants` for tool tinting, variant selection and the gripper
  camera, and gets nothing.
- **Default (gap 20):** `ToolsCollection(..., default_key="FLANGE")` (`robot.py:263`) while
  the shipped `PAR6.toml` fits `MSG_small_motor_150mm_rail` `[measured]`. The pre-STATUS UI
  shows a bare flange.

### <a name="gap-29"></a>Gap 29 — `Robot.start()` semantics

`Waldo-Commander/waldo_commander/main.py:288-296` calls
`robot.start(host=…, port=…, com_port=…, timeout=60)` under `EXCLUSIVE_START`, whose documented
contract is *fail hard if something is already running*. parol6 raises
`"Server already running at …"` (`robot.py:908-909`). par6 **reuses** it silently
(`python/par6/robot.py:607-608`) and drops `com_port` on the floor. Also note that WC's
`normalize_logs=True` default is dropped for par6 by
`Waldo-Commander/waldo_commander/profiles/__init__.py:87-106` — graceful, but it means
[gap 22](#gap-22).

---

## 2. waldoctl `RobotClient` ABC surface

`python/par6/client/async_client.py` implements **every** abstract method and all but two of
the optional ones. This layer is in good shape; the editor command palette is at parity.

| Optional member | parol6 | par6 |
|---|---|---|
| `move_c` / `move_s` / `move_p` | ✔ | ✔ client-side, refused by the runtime — [gap 6](#gap-6) |
| `wait_status` / `wait_checkpoint` | ✔ | ✔ |
| `simulator` / `is_simulator` / `teleport` | ✔ | ✔ client-side; `simulator(false)` refused — [gap 12](#gap-12) |
| `freedrive` / `is_freedrive` | not implemented | not implemented | parity (both robots lack the capability) |
| `set_shapes` / `shapes` | ✔ | ✔ (inert without `ffi` — [gap 1](#gap-1)) |
| `joint_speeds`, `io`, `status`, `queue`, `tools`, `activity`, `reachable`, `error`, `profile`, `tcp_speed` | ✔ | ✔ |
| `connect_hardware` | ✔ | ✔ client-side, refused by the runtime — [gap 12](#gap-12) |
| `select_profile`, `select_tool`, `set_tcp_offset`, `tcp_offset`, `tool`, `write_io`, `tool_action`, `reset_state`, `checkpoint`, `delay` | ✔ | ✔ (`write_io` refused — [gap 11](#gap-11)) |
| `is_estop_pressed` / `is_robot_stopped` | ✔ (`async_client.py:1135,1148`) | **absent** — [gap 27](#gap-27) |

**Palette diff (measured, via WC's own `_scan_class_commands` logic):** par6 46 entries, parol6
47. Only-parol6: `is_estop_pressed`, `is_robot_stopped`. Only-par6: `wait_command`.

par6 adds `set_completion_policy`, `set_recipe`, `queue_state`, `status_seq_gaps`,
`bind_tools` — see [Where par6 is ahead](#where-par6-is-ahead).

### <a name="gap-10"></a>Gap 10 — a bare client has no tools (S, GAP)

- **parol6:** `AsyncRobotClient.__init__` calls `_bind_default_tools()`
  (`client/async_client.py:295-317`); the sync facade does the same (`sync_client.py:136`).
  A user script that does `rbt = RobotClient(); rbt.select_tool("SSG-48"); rbt.tool.close()`
  works.
- **par6:** tool specs are injected only by the `Robot` factory
  (`python/par6/robot.py:661,667` → `bind_tools`). A bare
  `par6.RobotClient(host=…, port=…)` has `_bound_tools == {}` `[measured]`; after
  `select_tool("SSG48")`, `client.tool` raises **`KeyError: 'SSG48'`** `[measured]`.
- **Consequence:** every user program (which constructs its own client — see
  `Waldo-Commander/programs/draw_circle.py:11-13`) loses `rbt.tool.*`.
- **Fix shape:** lazily bind from `par6.config.load_gripper_configs()` in the client
  constructor, as parol6 does from its registry.

### <a name="gap-27"></a>Gap 27 — `is_estop_pressed` / `is_robot_stopped` (S, GAP)

Both are `Category:`-tagged palette commands in parol6. par6 has the data
(`STATUS.io[4]`, `STATUS.speeds`) — these are two three-line methods.

---

## 3. Command surface

parol6 registers **46** commands (enumerated dynamically from
`parol6.server.command_registry`). par6-proto defines **48**
(`crates/par6-proto/src/enums.rs`, `CmdType`). Nothing parol6 accepts is missing from the
par6 vocabulary. The gaps are in *honouring* them.

### Commands par6 defines but does not honour

| Command | par6 behaviour `[measured]` | parol6 | Verdict |
|---|---|---|---|
| `MOVE_C` / `MOVE_S` / `MOVE_P` | `MOTN_SETUP_FAILED "arc/spline/process moves are not implemented yet (par6d follow-up)"` — `crates/par6d/src/planner.rs`, `Par6Planner::start` ~:1274 | `MoveCCommand`/`MoveSCommand`/`MovePCommand`, `parol6/commands/curved_commands.py:188,232,284` | [gap 6](#gap-6) GAP |
| `MOVE_L` / `MOVE_J_POSE` | `MOTN_SETUP_FAILED "cartesian planning needs a par6d build with feature ffi"` in a default build — `planner.rs` ~:1270 | full IK + TOPPRA path | [gap 1](#gap-1) GAP |
| `SERVO_L` / `SERVO_J_POSE` / `JOG_L` | `COMM_VALIDATION_ERROR "this runtime has no kinematics"` — `crates/par6-server/src/server.rs:768-773` | full | [gap 1](#gap-1) GAP |
| `WRITE_IO` | always `COMM_VALIDATION_ERROR "write_io is unavailable: this runtime drives no digital outputs"` — `server.rs:450-462` | writes `state.InOut_out[port]` — `parol6/commands/system_commands.py:101-114` | [gap 11](#gap-11) DIV+GAP |
| `SIMULATOR(false)` | `MOTN_SETUP_FAILED "live backend switching is not wired yet"` — `crates/par6d/src/bridge.rs:489` | live toggle | [gap 12](#gap-12) GAP |
| `CONNECT_HARDWARE` | `MOTN_SETUP_FAILED "cannot switch to hardware bus … while running"` — `bridge.rs:501` | persists the COM port and reconnects | [gap 12](#gap-12) GAP |
| `SELECT_TOOL` (non-fitted) | `COMM_VALIDATION_ERROR "tool 'X' is not fitted"` — `server.rs:684-700` | any registered tool | [gap 13](#gap-13) DIV |
| `SELECT_PROFILE("TOPPRA")` | `SYS_PROFILE_INVALID` in a default build, though `Robot.motion_profiles` advertises it | 3 profiles, all real | [gap 1](#gap-1) GAP |
| any move with `r != nil` | `COMM_VALIDATION_ERROR "blend radius … is not supported"` — `server.rs:719-729` | blend buffer + lookahead — `parol6/server/motion_planner.py:293-345` | [gap 7](#gap-7) GAP |
| `JOG_J` with >1 non-zero axis | `COMM_VALIDATION_ERROR "jog_j drives one joint at a time"` — `server.rs:736-745` | multi-joint | [gap 16](#gap-16) DIV |
| `TELEPORT` out of range | silently clamps to hard limits — `crates/par6d/src/bridge.rs:414-441` | — | [gap 23](#gap-23) GAP |

Everything else was exercised end-to-end and works: `PING`, `STATUS`, `ANGLES`, `POSE`, `IO`,
`SPEEDS`, `TOOLS`, `QUEUE`, `ACTIVITY`, `LOOP_STATS`, `PROFILE`, `REACHABLE`, `ERROR`,
`TCP_SPEED`, `TCP_OFFSET`, `TOOL_STATUS`, `IS_SIMULATOR`, `SHAPES`, `RESET`, `ESTOP`, `STOP`,
`SIMULATOR(true)`, `RESET_STATE`, `SET_TCP_OFFSET`, `SET_SHAPES`, `SET_COMPLETION_POLICY`,
`SET_RECIPE`, `SERVO_J`, `JOG_J`, `TELEPORT`, `RESET_LOOP_STATS`, `HOME`, `MOVE_J`
(abs / rel / duration / speed), `SELECT_TOOL` (fitted), `DELAY`, `CHECKPOINT`, `TOOL_ACTION`
(move + calibrate). `[measured]`

### <a name="gap-1"></a>Gap 1 — Cartesian and collision are compile-time optional (L, GAP)

`crates/par6d/Cargo.toml:9-22` defines `ffi` and leaves it **off by default**:

> *"Off by default so plain builds need no C++ toolchain."*

What an `ffi`-less build loses, all measured against the prebuilt `target/release/par6d`:

- `move_l`, `move_j_pose` → `MOTN_SETUP_FAILED`.
- `servo_l`, `servo_j_pose`, `jog_l` → `COMM_VALIDATION_ERROR` (`server.rs:768-773`).
- **`STATUS.pose` is `[NaN × 12, 0,0,0,1]`** and `tcp_speed` is `NaN`. `crates/par6-rt/src/hooks.rs:126-133`
  (`NoFk`) fills every TCP field with NaN by design — honest, but WC binds
  `commander.status.pose.{x,y,z,rx,ry,rz}` straight into the readout
  (`Waldo-Commander/waldo_commander/components/readout.py:333-414`), so the operator sees `nan`.
- `cart_en_wrf` / `cart_en_trf` are forced to all-zero (`server.rs:1119-1126`), so every
  Cartesian jog button in WC greys out.
- `Planner::set_shapes` becomes `Ok(None)` and `Planner::collision` becomes `None`
  (`planner.rs`, the `#[cfg(not(feature = "ffi"))]` pair ~:1445-1465). `set_shapes` still
  answers `1` — the client is told the world was applied when nothing enforces it. That is the
  same "report success without confirming" failure the waldoctl contract forbids
  (`waldoctl/client.py:41-42`).
- `TOPPRA` is not registered, so `select_profile("TOPPRA")` fails while
  `Robot.motion_profiles` advertises it (`python/par6/robot.py:426-439` documents this, which
  is honest but does not help the UI).

`.github/workflows/ci.yml:99-100` does test `par6d --features ffi`, so the code is exercised —
but `scripts/deploy/README.md:171` states plainly: *"`--features ffi` for aarch64 is not
supported by any script here."* If the control box is aarch64, the shipped runtime is the
degraded one.

**Fix shape:** make `ffi` a default feature and give the aarch64 deploy path a cross-built
shim; failing that, have `par6d` refuse to start without kinematics unless an explicit
`--no-kinematics` flag is passed, so a degraded runtime is a deliberate choice rather than the
default.

### <a name="gap-6"></a>Gap 6 — `move_c` / `move_s` / `move_p` (L, GAP)

`Waldo-Commander/programs/draw_circle.py` and `programs/demo_showcase.py` are built entirely on
them (7 `move_c`, 2 `move_p`, 2 `move_s` call sites). WC's editor even has a
`rbt.move_c(` → `rbt.move_j(` rewrite path (`components/editor.py:193`), which suggests these
are load-bearing in the demo story. `crates/par6-motion` already has the profile machinery; the
missing part is Cartesian path generation for arcs and splines, which is the same shape as
`start_move_l`'s IK-waypoint loop.

### <a name="gap-7"></a>Gap 7 — blend radius and queue lookahead (M, GAP)

par6d "plans and runs exactly one queued command at a time" (`server.rs:719-729` comment), so
the arm decelerates to zero at every waypoint. parol6 buffers up to
`PAROL6_MAX_BLEND_LOOKAHEAD` commands and calls `do_setup_with_blend`
(`motion_planner.py:293-345`). This is a motion-quality gap, not just an API gap: a 12-point
`move_p` path in parol6 is one smooth sweep; the same points in par6 would be 12 full stops even
once `move_p` exists.

### <a name="gap-11"></a>Gap 11 — digital I/O (M, DIV+GAP)

The *divergence* is justified and well-documented (`server.rs:445-449`): the Spectral CAN
protocol has no output frame, and the RT core owns exactly one GPIO line — the e-stop input.
Refusing loudly instead of acking a lie is the right call.

The *gap* is what surrounds it:

- `Core::io()` (`server.rs:1030-1032`) returns `[0, 0, 0, 0, !estop]` — the two inputs and two
  outputs report the un-asserted level because the wire type has no "unknown" spelling.
- But `Robot.digital_inputs`/`digital_outputs` still say **2 and 2**
  (`python/par6/robot.py:399-405`), so WC renders four I/O chips
  (`components/readout.py:200,307`, `components/io.py:38`) that are permanently `0` and whose
  toggles always error (`components/io.py:26`).
- `Waldo-Commander/waldo_commander/services/motion_recorder.py:346` emits
  `rbt.write_io(port, state)` into recorded programs — which then fail on replay.
- **Fix shape:** report `digital_inputs = digital_outputs = 0` so WC renders no I/O surface at
  all, rather than an inert one. Cheap, and it turns a lie into a truthful absence.

### <a name="gap-12"></a>Gap 12 — no live simulator/hardware switch (M, GAP)

WC's Robot/Sim toggle (`components/control.py:1808-1837`) does
`await client.simulator(enabled)` then `await client.reset()`, and its startup hardware-detect
path (`main.py:906-915`) does the same on detecting hardware. Both raise on par6 `[measured]`,
producing a `ui.notify("Simulator toggle failed: …", color="negative")`. parol6 switches its
serial transport live (`server/transport_manager.py:166`).

### <a name="gap-13"></a>Gap 13 — `select_tool` only accepts the fitted tool (M, DIV)

The reasoning at `server.rs:688-698` is sound — swapping tools swaps the kinematic and gravity
models, which are built at startup. But WC has a tool dropdown
(`components/settings.py:315-344`) populated from `robot.tools.available`; picking anything but
the fitted tool raises. Either the dropdown needs a backend-supplied "selectable" set, or par6d
needs to rebuild its models on selection. Worth an explicit decision rather than a runtime
error.

### <a name="gap-16"></a>Gap 16 — single-joint jog (S, DIV)

`server.rs:736-745` refuses >1 non-zero axis because the RT jog engine ramps one joint at a
time with per-joint direction-block latching. WC's own jog UI sends exactly one joint per tick
(`components/control.py:1398-1406`) and one axis per tick (`:1560-1566`), so the practical
impact is limited to `jog_j(joints=[…], speeds=[…])` in user scripts and
`waldoctl`'s documented multi-joint form. Documented divergence; low priority.

### <a name="gap-23"></a>Gap 23 — teleport clamps silently (S, GAP)

`crates/par6d/src/bridge.rs:438-441` clamps each angle to `[hard_min_rad, hard_max_rad]`.
`[measured]` `teleport([0,-90,90,0,0,0])` landed the arm at `[-0.28, -90.0, 109.09, …]` — J3
moved 19° from where it was asked to go, and the call returned success. `CLAUDE.md` is explicit
that a parameter the runtime cannot honour is refused, never silently altered; the server
already does exactly that for `tool_positions` (`server.rs:745-762`). Extend the same check to
the joint angles.

---

## 4. Status / telemetry fields

The v2 STATUS packet (`crates/par6-proto/src/status.rs:29-100`, 31 elements) is a **superset**
of parol6's — it adds `proto_version`, `controller_id`, `seq`, `mono_time_ns`, `link_ok`,
`data_age_ms`, `accepted_index`. Every field waldoctl's `StatusBuffer` Protocol
(`waldoctl/status.py:22-73`) requires is present and decoded
(`python/par6/protocol/wire.py`), including `cart_en` keyed `"WRF"`/`"TRF"` exactly as WC's
status loop expects (`Waldo-Commander/waldo_commander/main.py:1578-1594`) `[measured]`.

The gaps are in *what the fields contain*.

| Field | par6 today | Verdict |
|---|---|---|
| `pose` | `NaN×12` without `ffi` (`crates/par6-rt/src/hooks.rs:126-133`) | [gap 1](#gap-1) |
| `tcp_speed` | `NaN` without `ffi`; otherwise finite-differenced at status rate (`server.rs:998-1012`) rather than from the Jacobian | acceptable |
| `io` | `[0,0,0,0,!estop]` — inputs/outputs never observed (`server.rs:1030-1032`) | [gap 11](#gap-11) |
| `cart_en_wrf` / `cart_en_trf` | forced all-zero without `ffi` (`server.rs:1119-1126`) | [gap 1](#gap-1) |
| `collision_active` / `collision_pairs` | always `false` / `[]` without `ffi` | [gap 1](#gap-1) |
| `error` | **only ever set by estop or by a failing queued command** (`server.rs:422-428`, `:854-861`) | [gap 3](#gap-3) |
| `action_state` | `Error` only via `fail_command` (`server.rs:858`) | [gap 3](#gap-3) |
| `queued_duration` | `duration_estimate` (`server.rs:1363-1376`) returns `0.0` for any move parameterised by `speed` rather than `duration` — i.e. almost all of them `[measured: queued_duration=0.0 with a move_j queued]` | [gap 24](#gap-24) |
| `homed` | correct, real | ✔ |
| `scene_epoch` | correct; bumps only when a world is actually applied | ✔ (better than parol6) |
| `tool_status` | real gripper telemetry from the CAN reply (`server.rs:1063-1108`) | ✔ |

Query-only:

| Query | par6 today | Verdict |
|---|---|---|
| `LOOP_STATS` | `std_period_s`, `min_period_s`, `p95_period_s` hardcoded `0.0` (`server.rs:1220-1236`, comment admits it) `[measured]` | [gap 25](#gap-25) DIV+GAP — the RT snapshot genuinely lacks them; either widen the snapshot or drop the fields from the wire rather than shipping three zeros |
| `POSE(TRF)` | `identity_pose()` (`server.rs:1197-1201`) `[measured: [0,0,0,0,-0,0]]`; parol6 returns `inv(T_fk)` (`parol6/commands/query_commands.py:71-78`) | [gap 26](#gap-26) DIV — par6's answer is definitionally true and informationally empty |
| `PING.hardware_connected` | `!simulator && link_ok` (`server.rs:1170-1172`) | ✔ matches WC's use at `main.py:908` |

### <a name="gap-3"></a>Gap 3 — RT errors are invisible (M, GAP)

This is the most dangerous telemetry gap and it is independent of `ffi`.

- The server's `standing_error` is written in exactly two places: `C::Estop`
  (`server.rs:429-436`) and `fail_command` (`server.rs:854-861`). `fail_command` only runs for a
  command that was *in flight*.
- The RT core latches hard errors independently — `RtiLinkLost`, `LoopCritical`, `ExecLinkLost`,
  `JointFault`, `Temperature`, `Encoder`, `Vbus`
  (`crates/par6-rt/src/state.rs:55-75`) — and on `errors.any_hard()` forces
  `state = ArmState::Disabled` (`crates/par6-rt/src/core.rs:852-856`).
- Nothing copies `snap.errors` into `standing_error`. `Core::build_status`
  (`server.rs:1128-1163`) reads `self.standing_error` only.

**Measured, verbatim, after the RT latched `RtiLinkLost` and went to `ActiveError`:**

```
error after:    None
activity after: ActivityResult(state=<ActionState.IDLE: 0>, command='', params='', error='')
status:         {'err': None, 'ac': '', 'st': 0, 'homed': True, 'link': 1}
move_j after:   RobotError [50] Controller disabled
```

The client is told *idle, no error, link healthy* while every motion command is refused. WC's
action log, error banner and status chips all read from these fields
(`Waldo-Commander/waldo_commander/main.py:1619-1640`), so the operator sees a healthy robot that
refuses to move. parol6 keeps a single `state.error` that any subsystem can set
(`parol6/server/state.py:246`, written from the controller, the segment player and tool actions).

**Fix shape:** mirror `snap.errors` (the highest-severity hard latch) into `standing_error` when
no command-attributed error is standing, and surface `ArmState` — waldoctl's `StatusBuffer` has
no `enabled` field today, so this likely wants a waldoctl addition alongside a par6 change.

### <a name="gap-2"></a>Gap 2 — boots DISABLED and `reset()` lies (M, GAP)

- **parol6:** `ControllerState.enabled = True` (`parol6/server/state.py:162`, and `:325` on
  reset).
- **par6:** the RT boots `ArmState::Disabled`. Everything with `needs_enabled`
  (`crates/par6-server/src/gating.rs` via `Core::check_gate`, `server.rs:648-672`) is refused
  with `SYS_CONTROLLER_DISABLED` — including `select_tool`, `checkpoint`, `delay` and
  `tool_action`, not just motion `[measured]`.
- WC never calls `reset()` on startup (it only does so after a simulator toggle,
  `components/control.py:1823-1830`). So a fresh WC + par6 session refuses every queued
  command until the operator finds the Reset button.
- Worse: `C::Reset` (`server.rs:422-428`) sets `estop_latched = false`, clears
  `standing_error`, calls `rt.set_enabled(true)` and unconditionally answers `Ok`. The RT may
  refuse the enable (`crates/par6-rt/src/core.rs:530-537`: *"enable refused: e-stop or errors
  active"*). **Measured:** during the ~1 s boot settle, `reset()` returned `1` and the very next
  `home()` was still refused with `SYS_CONTROLLER_DISABLED`. `waldoctl/client.py:41-42`:
  *"A backend that cannot confirm application must never report success."*

### <a name="gap-24"></a>Gap 24 — queue ETA is always zero (S, GAP)

`duration_estimate` (`server.rs:1363-1376`) returns `p.duration.unwrap_or(0.0)`. Since WC and
most scripts pass `speed=`, not `duration=`, `queued_duration` is 0 for a queue full of real
moves `[measured]`. The planner knows the real duration once a command starts; the honest
interim is to sum only what is known and let the field mean "known queued seconds", documented
as such.

---

## 5. Behaviors that are not commands

### Homing — [gap 14](#gap-14), M, GAP

- **parol6:** `TrajectoryPlanner.process` (`server/motion_planner.py:239-241`) short-circuits
  `HomeCmd` when `Homed_in[:6].all()` into
  `MoveJCmd(angles=HOME_ANGLES_DEG, speed=HOME_RETURN_SPEED_FRAC)` — a normal, collision-checked
  planned move at 0.5 speed. The firmware seek runs only when the arm is unreferenced
  (`parol6/commands/basic_commands.py:78-82`).
- **par6:** `Command::Home(_) => SetMode(Mode::Homing)` unconditionally
  (`crates/par6d/src/planner.rs`, `Par6Planner::start` ~:1249).
- **Measured:** a full sequence took **~62 s** at the e2e rig's tick rate and ended at
  `[90.09, -106.03, 163.24, 0.06, -28.98, 179.92]` deg — which is **not**
  `Robot.joints.home.deg` = `[0, -90, 180, 0, 0, 180]` (the `park_pose_rad` from
  `config/PAR6.toml:18`). So WC's Home button (a) always re-references and (b) does not land the
  arm where the UI says home is.
- **Fix shape:** route an already-homed `HOME` to a planned move to `park_pose_rad`, exactly as
  parol6 does. This also makes `Robot.joints.home` truthful.

### Streaming — [gap 15](#gap-15), M, GAP

**Measured, reproducible:** at the tick rate the repo's own e2e rig uses
(`python/tests/live_daemon.py:35-38`, `TICK_DT_S = 0.05`), a `servo_j` stream at 20 Hz latches
`RtiLinkLost` within one tick of entering `Mode::Stream`:

```
[WARN par6_rt::errors] error latched: RtiLinkLost joint=None
[WARN par6_rt::core]   hard error: mode Stream -> ActiveError
```

The joint does not move (delta `-0.038°` over 40 commands) and the controller is disabled for
the rest of the session — invisibly, per [gap 3](#gap-3). At the shipped 250 Hz tick the same
stream works (delta `1.833°`, no latch), including at a 150 ms cadence.

Likely cause, for whoever picks this up: `stream_timeout_ticks = robot.ticks(0.040).max(1)`
(`crates/par6-rt/src/core.rs:362`; `stream.command_timeout_s = 0.040` at
`crates/par6-config/src/robot.rs:399` and `config/PAR6.toml:512`). At `dt = 0.05` that rounds to
**1 tick**, i.e. a 50 ms watchdog fed by a housekeeping keep-alive whose period is a hardcoded
`HOUSEKEEPING_PERIOD = 4 ms` (`crates/par6d/src/bridge.rs:53`) — a fixed constant that assumes
the 250 Hz tick. This is exactly the failure mode `CLAUDE.md`'s
*"time constants live in config as SECONDS and convert via `round(s/dt)`"* rule exists to
prevent, and it is currently untested: `python/tests/test_e2e_daemon.py` covers `jog_j` and
`teleport` but **no `servo_j` / `servo_l`**.

parol6 has no equivalent kill-watchdog: its Ruckig streaming executor simply converges on the
last target when new ones stop arriving (`parol6/motion/streaming_executors.py:367-390`).

Note also `crates/par6d/src/bridge.rs:15-19`, which claims housekeeping self-terminates streams
*"so the RT watchdog never latches a link-lost error on a client that simply stopped
streaming."* At reduced tick rates that guarantee does not hold.

### Error taxonomy and recovery

Codes are in the same subsystem ranges and par6's catalogue is richer (structured
title/cause/effect/remedy on the wire, 9 extra codes: `MotnToolFault`, `MotnSettleTimeout`,
`CommChunkTimeout`, `CommUnknownRecipe`, `SysNotSimulator`, `SysExecLinkLost`,
`SysRtiLinkLost`, `SysLoopCritical`, `SysJointFault`). **[gap 28](#gap-28), DIV:** two codes
were renumbered — parol6 `SYS_PROFILE_INVALID = 53` / `SYS_SELF_COLLISION = 54` vs par6
`SysProfileInvalid = 52` / `SysSelfCollision = 53` (52 was parol6's `SYS_PORT_SAVE_FAILED`).
Anything keying on the number rather than the name mis-reads.

Recovery: par6 latches e-stop until an explicit `reset()`; parol6 auto-recovers on physical
release (`parol6/server/controller.py:362-375`). par6's behaviour matches the waldoctl contract
(`waldoctl/client.py:350-360`) and is the better one.

### Queue semantics, checkpoints, completion policy

Measured and correct: monotonic indices never reset; `stop()` clears the queue and leaves
waiters resolving `False`; back-to-back moves queue and complete in order; `checkpoint` sets
`last_checkpoint` and `wait_checkpoint` fires; `delay` blocks the queue; idempotency-keyed
retries re-ack the original index. `COMPLETE` pushes plus the `accepted_index` stale-error
ordering rule are a genuine improvement over parol6's status-polling completion.

par6 adds a wire-level `SET_COMPLETION_POLICY` (commanded / settled / strict) where parol6 has
only a `PAROL6_SETTLE_MAX_TICKS` env knob. Ahead.

### Program / script execution

WC spawns user scripts in a subprocess and monkey-patches `<backend>.RobotClient`
(`Waldo-Commander/waldo_commander/services/stepping_bootstrap.py:49-74`). par6 exports
`RobotClient` at package level and has a `par6.client` submodule, so the stepping wrapper
attaches correctly. The two live issues are [gap 10](#gap-10) (no default tools) and the fact
that all six shipped programs hardcode `from parol6 import RobotClient` and `port=5001` — a WC
migration task, but one whose blast radius is set by [gap 6](#gap-6) and
[gap 21](#gap-21).

### Motion recording

`Waldo-Commander/waldo_commander/services/motion_recorder.py` emits `rbt.write_io(...)`
(`:346`) and reads `commander.status.tool.key` (`:220`). The first produces lines that error on
replay ([gap 11](#gap-11)); the second feeds the casing mismatch ([gap 8](#gap-8)).

### Tools / grippers

par6 supports **passive** and **electric** (verbs `move`, `calibrate` —
`crates/par6d/src/planner.rs`, `start_tool_action` ~:622-676). parol6 also has
**pneumatic** and **vacuum**, both driven through digital outputs
(`parol6/robot.py:312-329`) — unimplementable on par6's bus, a justified divergence that follows
from [gap 11](#gap-11). Tool status telemetry (jaw position normalised to 0..1, current in mA,
fault bitfield, `variant_key`) is real and good.

### Collision, workspace envelope

Server-side collision landed in #17 and is enforced with layers, epochs and pre-flight gating —
but only under `ffi` ([gap 1](#gap-1)), and the client-side twin is absent
([gap 17](#gap-17)). Open follow-ups are already tracked: **#18** (SRDF; PAR6's park pose
self-collides without it) and **#19** (streaming not gated, escape-depth half missing,
`installation_shapes` has no config producer). Do not re-file those.

The workspace envelope (`Waldo-Commander/waldo_commander/services/urdf_scene/envelope_renderer.py:485-512`)
loads a raw `pinokin.Robot(urdf_path)` in a worker and takes limits from
`robot.joints.limits.position.rad`, so it works for par6 — provided
[gap 4](#gap-4) is fixed, since it reads the same `urdf_path`.

### Simulator behaviour

`par6d --sim` is a genuine closed-loop plant and is a clear improvement on parol6's mock serial
transport. Gaps: no live switch ([gap 12](#gap-12)), silent teleport clamping
([gap 23](#gap-23)), and the streaming latch ([gap 15](#gap-15)).

### <a name="gap-22"></a>Logging — gap 22, S, GAP

parol6 streams the controller subprocess's stdout into Python logging with level/logger-name
normalisation (`parol6/robot.py:189-227`), which is what feeds WC's log panel. par6 redirects
`par6d`'s output to an unnamed `tempfile.NamedTemporaryFile` (`python/par6/robot.py:110-122`)
whose path is only ever `logger.debug`'d. The reason given (never inherit an undrained pipe) is
correct — but a reader thread that forwards into `logging` has the same property and keeps the
diagnostics.

### <a name="gap-21"></a>Config surface — gaps 18, 19, 21, 30, 31

- **[gap 18]** `CARTESIAN_JOG_LIMITS` (`python/par6/config.py:160-166`) are hand-picked
  constants — *"Magnitudes sized for a desktop 6-axis arm of PAR6's reach"* — not derived from
  the runtime, and the runtime enforces no Cartesian limits at all. parol6 reads
  `LIMITS.cart.jog` from config (`parol6/robot.py:544-554`). WC scales its Cartesian jog UI off
  this (`robot.cartesian_limits`, 2 sites), so the numbers are load-bearing.
- **[gap 19]** `joints.names` is `('joint1' … 'joint6')` `[measured]` from the URDF; parol6 uses
  `("Base","Shoulder","Elbow","Wrist 1","Wrist 2","Wrist 3")` (`parol6/robot.py:401`). WC's
  readout labels every joint row from this.
- **[gap 21]** par6d defaults to command port **6001**; WC defaults to **5001**
  (`Waldo-Commander/waldo_commander/constants.py:54`) and passes it explicitly into both
  `create_async_client` and `start`. Out of the box they do not meet. Trivially fixed with
  `WALDO_CONTROLLER_PORT=6001`, but it needs to be written down somewhere a user will find it.
- **[gap 30]** `pip install parol6` ships the whole server plus a `parol6-server` console
  script. `pip install par6` ships only the client; `par6d` must arrive separately
  (`Robot.start` resolves it via `PAR6D_BIN` then `PATH`, `python/par6/robot.py:75-88`, and
  raises otherwise). Architecturally unavoidable, but it changes the install story and should be
  documented for WC's `[par6]` extra.
- **[gap 31]** par6's config lives in TOML with no `PAR6_STATUS_RATE_HZ` / `PAR6_FAKE_SERIAL`
  equivalents. WC's `conftest.py` sets `PAROL6_STATUS_RATE_HZ=20` and `PAROL6_FAKE_SERIAL=1`;
  the par6 equivalent is "write a patched TOML", which is what
  `python/tests/live_daemon.py:64-84` does. Fine for par6's own tests, awkward for WC's.

---

## 6. What Waldo Commander calls that par6 cannot satisfy

This is the practical definition of "must work". Ranked by breakage severity.

| WC call site | par6 result | Gap |
|---|---|---|
| `main.py:199` `package_map={robot.backend_package: mesh_dir}` | **every mesh path 404s** — empty viewport | [4](#gap-4) |
| `services/path_visualizer.py:527` `robot.create_dry_run_client()` | `None` → path preview / targets / timeline dead | [5](#gap-5) |
| `components/control.py` `client.move_l(...)` ×41 across app + programs | `MOTN_SETUP_FAILED` without `ffi` | [1](#gap-1) |
| `components/readout.py:333-414` binds `status.pose.{x..rz}` | renders **`nan`** without `ffi` | [1](#gap-1) |
| `components/control.py:1560` `client.jog_l(...)`, `:1539` `client.servo_l(...)` | `COMM_VALIDATION_ERROR` without `ffi`; Cartesian jog buttons greyed by all-zero `cart_en` | [1](#gap-1) |
| any queued command before someone presses Reset | `SYS_CONTROLLER_DISABLED` | [2](#gap-2) |
| `main.py:1619-1640` action log / error banner from `status.error`, `action_state` | shows *idle, no error* while the controller is disabled | [3](#gap-3) |
| `main.py:185`, `components/settings.py:98,147,344`, `urdf_scene.py:1773` `robot.tools[status.tool.key]` | `KeyError` → no tool TCP, tool meshes cleared | [8](#gap-8) |
| `programs/draw_circle.py`, `programs/demo_showcase.py` (`move_c`/`move_p`/`move_s`) | `MOTN_SETUP_FAILED` | [6](#gap-6) |
| `components/control.py:1820` `client.simulator(...)`, `main.py:911` auto robot-mode switch | raises → red toast | [12](#gap-12) |
| `components/io.py:26` `client.write_io(index, state)` | raises; the four I/O chips are permanently 0 | [11](#gap-11) |
| `components/settings.py:315-344` tool dropdown → `client.select_tool(...)` | raises for any non-fitted tool | [13](#gap-13) |
| `services/urdf_scene/scene_handle.py:87,207` + `path_visualizer.py:81-118` `robot.apply_shapes` / `check_trajectory` / `has_collision_checking` | silent no-ops; no preview collision | [17](#gap-17) |
| `main.py:288-296` `robot.start(..., com_port=…)` under `EXCLUSIVE_START` | `com_port` dropped; a running runtime is reused instead of a hard failure | [29](#gap-29) |
| `main.py:1768` `create_async_client(port=config.controller_port)` | 5001 ≠ 6001 | [21](#gap-21) |
| Home button → `client.home()` | full ~60 s re-reference; lands off `joints.home` | [14](#gap-14) |
| `components/readout.py` joint labels from `robot.joints.names` | `joint1 … joint6` | [19](#gap-19) |
| log panel | empty (par6d logs to a temp file) | [22](#gap-22) |

**Already satisfied by par6, verified end-to-end:** `ping`, `wait_ready`,
`stream_status_shared` and every `StatusBuffer` field WC's status loop reads
(`main.py:1534-1625`), `angles`, `move_j` (all four timing forms), `jog_j`, `servo_j`,
`teleport`, `stop`, `estop`, `reset`, `reset_state`, `select_profile`, `set_tcp_offset`,
`tcp_offset`, `set_shapes`, `shapes`, `checkpoint`, `wait_command`, `wait_checkpoint`,
`wait_motion`, `tools`, `activity`, `queue`, `status`, `tool_action`, `scene_epoch`-driven
world readback.

---

## <a name="in-flight"></a>In flight / already tracked — do not re-file

- **TCP-offset retargeting** — landed in the working tree during this audit:
  `Par6Planner::sync` now publishes the offset into the shared `tool_offset` cell under
  `#[cfg(feature = "ffi")]` (`crates/par6d/src/planner.rs` ~:1380-1391), so planning, streaming
  and the reported pose resolve at the same point.
- **Cartesian enablement probe** — also landed: `EnablementProbe` +
  `Par6Planner::probe_directions` (`planner.rs` ~:240, :1022-1110), with parol6's 0.5 mm / 0.5°
  probe steps carried over.
- **#18** — SRDF support in `par6-kin`; PAR6's park pose self-collides without it.
- **#19** — jog/servo not collision-gated; escape-depth half of the start-in-collision rule
  missing (needs `par6_col_distance` in the shim); `installation_shapes` has no config producer.
- **#17** — closed; collision enforcement is wired (subject to [gap 1](#gap-1)).

---

## <a name="where-par6-is-ahead"></a>Where par6 is ahead

Parity of *capability* is the goal, not parity of bugs. par6 is already better on:

1. **Explicit refusal over silent degradation.** `validate_supported` (`server.rs:711-779`)
   refuses a parameter the runtime cannot honour instead of dropping it. parol6 has several
   silent-ignore paths. (Where par6 breaks its own rule — teleport clamping, `set_shapes`
   acking without `ffi` — it is called out above.)
2. **Push completion + acceptance ordering.** `COMPLETE` pushes and the `accepted_index`
   stale-error rule (`python/par6/client/async_client.py:725-795`) replace parol6's poll-only
   completion and make `wait_command` correct under packet loss.
3. **Idempotency-keyed enqueue.** A retried QUEUED command re-acks the original index
   (`server.rs:592-610`) — parol6 can double-enqueue on retry.
4. **A monotonic index allocator that survives `reset_state`** (`server.rs:8-12`), so a stale
   pre-reset status frame cannot satisfy a post-reset wait.
5. **Structured errors on the wire** — `[command_index, code, title, cause, effect, remedy]`
   with server-side formatting (`crates/par6-proto/src/error.rs`), versus parol6's thinner
   catalogue.
6. **Status header:** `seq`, `mono_time_ns`, `link_ok`, `data_age_ms`, `controller_id` — loss
   detection and staleness reporting parol6 has no equivalent for. STATUS keeps broadcasting when
   the bus link is down instead of going silent.
7. **`scene_epoch` readback truth** — displays re-query rather than trusting a local copy.
8. **Cross-language golden vectors** and a generated Python constants mirror: Rust and Python
   cannot drift.
9. **`--sim` is a real closed-loop plant**, not a mock serial transport.
10. **Tool-dependent `urdf_path` / `mesh_dir`** — the gripper is in the URDF tree rather than
    bolted on as loose meshes. Better; it just needs [gap 4](#gap-4) fixed to be usable.
11. **Config-as-TOML** rather than two dozen environment variables.
12. **`SET_COMPLETION_POLICY` / `SET_RECIPE`** as first-class wire commands.

---

## Appendix A — how the measurements were taken

All `[measured]` claims come from a live `par6d --sim` driven by the real `par6` client over
real UDP, using the repo's own rig (`python/tests/live_daemon.py`).

```bash
# runtime under test: the prebuilt default-feature binary (no `ffi`)
ls -l /workspace/par6/target/release/par6d
strings target/release/par6d | grep -c "cartesian planning needs a par6d build"   # -> 1, i.e. ffi OFF

# full command-surface sweep at the e2e rig's 20 Hz tick
PAR6D_BIN=/workspace/par6/target/release/par6d python3 probe.py

# post-home motion, queue and stop semantics, jog/servo/teleport deltas
PAR6D_BIN=/workspace/par6/target/release/par6d python3 probe3.py

# servo_j isolation (latches RtiLinkLost at 20 Hz tick)
PAR6D_BIN=/workspace/par6/target/release/par6d python3 probe4.py servo_j

# the same stream against the SHIPPED 250 Hz config (does not latch)
python3 probe5.py 0.05 ; python3 probe5.py 0.15
```

Static measurements taken by importing the packages directly:

```python
# Robot surface
from par6.robot import Robot; r = Robot()
r.has_collision_checking          # False
r.create_dry_run_client()         # None
[t.key for t in r.native_tools.available]
                                  # ['FLANGE', 'MSG_SMALL_MOTOR_150MM_RAIL', 'SSG48']
r.tools.default.key               # 'FLANGE'   (runtime is fitted with MSG_small_motor_150mm_rail)
r.joints.names                    # ('joint1', ..., 'joint6')

# WC's package:// resolution, replayed against par6's URDF
#   package 'par6_flange' not in {'par6': mesh_dir}
#   -> .../par6_flange/urdf/meshes/base_link.STL   exists=False

# bare client has no tools
from par6 import RobotClient; c = RobotClient(host='127.0.0.1', port=6001)
c._bound_tools                    # []
# after select_tool('SSG48'):  c.tool -> KeyError: 'SSG48'

# parol6 command registry, enumerated dynamically
import parol6.server.command_registry as cr; cr.discover_commands()
len(cr.list_registered_commands())   # 46

# editor palette diff, using WC's own scanner logic
# par6 46 entries; parol6 47; only-parol6: is_estop_pressed, is_robot_stopped
```
