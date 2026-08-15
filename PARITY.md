# par6 ↔ parol6 parity audit

**Question answered:** what does "par6 matches parol6" still require?

**Reference:** `parol6` at `829c2c7`, source at `/workspace/jepson2k/parol6-python-api`. The
reference moved since the second pass: `829c2c7` merged PR #27 (`feat/mcp-server` — the
branch name is the work's context, not its content; the merge contains **no MCP code**, see
[§ ahead](#where-par6-is-ahead)) and with it several of par6's own designs: structured wire
errors, the `accepted_index` stale-error ordering rule, the reset-surviving index allocator,
fast-path `home()`. Where this document says "parol6 adopted this", that is what it means.
**Subject:** this repo at `4c15ac6`. Every citation was read at that commit. The follow-up
commit `8ebb496` retired the `spec/` documents and otherwise touched only comments — cited
symbol names survive, but `spec/` paths named below no longer exist at current HEAD and
line numbers in comment-heavy files may have drifted a few lines.
**Contract spine:** `waldoctl` 0.7.0 at `/home/user/waldoctl`.
**Consumer of record:** Waldo Commander at `/home/user/Waldo-Commander` (WC).
**License boundary (standing):** parol6 and the vendor sources are behavior-only references —
cite them, port their semantics, never their code.

This is the **third** pass. The second (`9074661`, 19 open rows + 7 in flight) is superseded:
thirteen commits since have closed all seven in-flight rows and five register rows — including
four of the second pass's five "fix these first" items — and the closures themselves opened
new preview-fidelity surface, audited on the same terms. Closures are listed with their
commits in [Closed since the second audit](#closed).

**Method, honestly stated:** unlike the second pass, this pass is **static**. Seven parallel
area auditors read both codebases at the commits above; twelve adversarial verifiers attacked
the twelve highest-ranked gap claims (**all twelve stood**); a completeness critic swept for
blind spots ([Appendix A](#appendix-a)). **No live daemon was driven and there are no new
`[measured]` claims.** Where the second pass's measured numbers remain the best evidence they
are cited as *measured at the second pass*; where a closure's correctness is a numeric claim
only a live run can confirm — the sim plant's convergence — it is carried as verification
debt, not asserted. Raw scoreboard: **132 items** across the seven areas — 42 ahead,
36 parity, 18 divergent-justified, 22 gap, 11 divergent-gap, 2 regression, 1 unverified —
deduplicated into the register below.

**The headline:** the runtime is at or above parity. What remains open is concentrated almost
entirely in the **Python client/preview layer** — the offline mirror of a runtime that moved
fast underneath it.

---

## If you only fix five things

| # | Fix | Why it is first |
|---|---|---|
| 1 | **Implement the client-side collision surface on the `Robot` ABC** ([row 1](#row-1)). | The one high-severity gap, found independently by two auditors and adversarially verified twice. `python/par6/robot.py:339-787` defines no collision member — `has_collision_checking`, `in_collision`, `colliding_pairs`, `check_trajectory`, `min_distance`, `apply_shapes` all inherit waldoctl's disabled defaults (`waldoctl/robot.py:250-280`) — while the runtime now enforces an **SRDF-exact world on planned AND streamed motion**. The preview confidently draws paths the runtime refuses at every entry point: the sharpest trust contradiction in the product. And the port is now **fully unblocked**: the per-variant SRDFs are packaged (`python/par6/_data/urdf/*/srdf/`, since `52ccf9f`), pinokin is already the FK/IK dependency and ships `CollisionChecker.load_srdf` — exactly what parol6 calls (`PAROL6_ROBOT.py:109-113`) — and building the checker on the active tool's own tree gives tool geometry for free, no parol6-style mesh-attach dance. Port the checker's lifecycle semantics, never its code. |
| 2 | **Re-mirror the dry-run client to the runtime it previews** ([rows 3, 6, 7](#row-3)). | Four preview/runtime disagreements, one work item. (a) The one real **regression**: runtime `home()` on an already-homed arm now plans a return to park (`planner.rs:1250-1269`, landed `32c95f3`); the preview still snaps to the homing-ready pose with duration 0 and a docstring asserting the opposite (`dry_run_client.py:665-675`). WC previews default `initial_homed=True` (`path_visualizer.py:174,527`), so the **common case** previews from a pose ~90° off on J1. (b) Preview `teleport` clamps where the runtime refuses and latches (`dry_run_client.py:677-698` vs `server.rs:941-960`) — a recorded out-of-range teleport previews clean, then fails live. (c) No `write_io` — recorded I/O lines raise `AttributeError` in preview. (d) No `status()` — **WC's default editor script errors on first preview** (`simulation_engine.py:41-53`). |
| 3 | **Apply or refuse the streaming speed/accel wire parameters** ([row 4](#row-4)). | `JogJ`/`JogL` `accel` and `ServoJ`/`ServoJPose`/`ServoL` `speed`+`accel` are decoded (`par6-proto/src/command.rs:180-234`), sent by the client, and read by **nothing** in `RtBridge::stream` — nor refused by `validate_supported`. parol6 honors all of them. WC's jog-accel slider and TCP-drag speed pass them (`control.py:1402-1406,1539-1543,1560-1566`), so those knobs are **silently inert** — against this repo's own refuse-don't-drop rule. The plumbing exists: `JogEngine::set_accel_time_s` (`par6-motion/src/jog.rs:186-195`), `StreamingExecutor::set_limits` (`stream.rs:63-71`). |
| 4 | **Emit waldoctl's `shape:`/`install:` vocabulary in `collision_pairs`** ([row 5](#row-5)). | waldoctl mandates the prefixed vocabulary (`waldoctl/status.py:61-65`, "never backend-internal geometry identifiers"); parol6 implements it exactly; par6 reports bare shape names and trimmed link names (`planner.rs:2199-2214`, `bridge.rs:224-229`). WC's tint map is keyed by the prefixes (`urdf_scene.py:1471-1485,1577`), so on par6 a keep-out collision tints the arm links but **never the offending shape**. A reporting-vocabulary change at two `display()` sites. |
| 5 | **Report digital I/O as 0/0** ([row 9](#row-9)). | The oldest untouched row in the register, unchanged through the whole pass. The bus divergence stays justified (no output frame on Spectral CAN); the surfacing does not: `Robot.digital_inputs/digital_outputs` still say 2/2 (`python/par6/robot.py:446-457`) while the server permanently refuses `WRITE_IO` and fills `io` with `[0,0,0,0,!estop]`, so WC renders four permanently-zero chips whose toggles always error (`components/io.py:26,35-36`). Decouple the ABC counts from the wire buffer sizing and render no I/O surface at all. |

---

## Gap register (ranked by user-visible impact)

Verdict key: **GAP** = genuine parity hole · **DIV** = justified architectural divergence
(still worth documenting) · **DIV+GAP** = the divergence is defensible but the way it surfaces
to the user is not · **REG** = a regression against something previously closed.

| # | Gap | Category | Consequence | Size | Was | Verdict |
|---|---|---|---|---|---|---|
| 1 | `Robot` collision surface is entirely default | Robot ABC | Preview/editing collision surface dead in WC while the runtime gates planned **and** streamed motion | M | 5 | GAP |
| 2 | Dry-run preview is collision-blind | preview | Draws paths and integrates jogs straight through keep-outs the runtime refuses or brakes on | M | 5+15 | GAP |
| 3 | Dry-run `home()` mirrors superseded runtime semantics | preview | Every preview containing `rbt.home()` seeds all downstream segments ~90° off on J1 | S | in-flight | REG |
| 4 | Streaming speed/accel params decoded, never applied, never refused | behavior | WC's jog-accel and TCP-drag-speed sliders are silently inert | S | new | GAP |
| 5 | `collision_pairs` bare names vs waldoctl's `shape:`/`install:` vocabulary | telemetry | Keep-out shapes never tint red in the 3-D view | S | new | GAP |
| 6 | Dry-run `teleport` clamps where the runtime refuses and latches | preview | Recorded programs preview clean, then fail live with a latched error | S | 15 | GAP |
| 7 | Dry-run has no `write_io` and no `status()` | preview | Default editor script and recorded I/O lines raise `AttributeError` on preview | S | 15 | GAP |
| 8 | `simulator(false)` / `connect_hardware` still refused | commands | WC's Robot/Sim toggle and COM-port picker raise (the no-op direction now succeeds) | M | 6 | GAP |
| 9 | `write_io` always errors while `digital_inputs/outputs` still say 2/2 | commands | Four permanently-zero I/O chips whose toggles always fail; recorded `write_io` fails on replay | M | 7 | DIV+GAP |
| 10 | `select_tool` accepts only the fitted tool | commands | WC's dropdown has three entries; two raise | M | 8 | DIV+GAP |
| 11 | All jog/servo streams gated on `homed` | behavior | An unhomed arm cannot be jogged clear of an obstruction (parol6 deliberately permits it) | M | new | DIV+GAP |
| 12 | Pneumatic/vacuum gripper family absent at every layer | tools | A vendor pneumatic or vacuum gripper vanishes on par6 with no error anywhere | M | new | DIV+GAP |
| 13 | `ToolSpec.variants` empty; `variant_key` accepted and echoed with no effect | Robot ABC | Variant selector hidden; a typo'd key persists in STATUS forever; declared-but-unimplemented surface | S | 11+12 | GAP |
| 14 | `MOTN_NOT_HOMED` remedy text promises "jogging remains available" | commands | The error a pre-homing jog raises claims the very thing that was just refused | S | new | GAP |
| 15 | aarch64: built, statically checked, never executed | deploy | The control box runs a shim whose numerics no test has validated on that ISA | M | 9 | GAP |
| 16 | Command port 6001 vs WC's 5001, documented nowhere a user finds | config | Spawn flow self-heals; attaching to a deployed/self-started `par6d` fails naming the wrong port | S | 16 | DIV+GAP |
| 17 | Build-remedy texts instruct a `par6d` build that refuses to boot | packaging | The first two remedies a user follows omit `--features ffi` | S | new | GAP |
| 18 | No `PAR6_STATUS_RATE_HZ` / tick-rate env override | config | WC-conftest-style rate reduction still needs a patched TOML | S | 19 | DIV |
| 19 | No runtime binary (or C++ shim) in the pip package | packaging | `pip install par6` gives no `par6d`; WC's `[par6]` extra says nothing about it | M | 18 | DIV |
| 20 | Error codes 52/53/54 collide numerically with different parol6 meanings | commands | Scripts keying on numbers mis-read; codes are frozen contract data — document, don't renumber | S | 17 | DIV |
| 21 | `jog_j` drives one joint at a time | commands | Multi-joint jog scripts refused — now loudly (the row-4 sting is gone) | S | 13 | DIV |
| 22 | `motion_profiles` is 3 where parol6 offers 5 | config | `QUINTIC` / `LINEAR` unavailable, consistently on runtime and preview | S | 14 | DIV |
| 23 | No `examples/` tree | docs | parol6 ships eight runnable examples kept green by tests; par6 ships none | S | new | GAP |

**Plus one verification debt, not counted as a row:** the sim plant's convergence after the
feedforward rework (`95cf98c`, closing #21/#22/#26) is confirmed by static reading of the law
and its tests only; the numeric claim needs one live measurement pass (see
[§5](#sim-fidelity)).

**Counts:** 23 open rows — **12 GAP, 5 DIV+GAP, 5 DIV, 1 REG**.
By category: preview 4 · Robot ABC 2 · behavior 2 · commands 6 · telemetry 1 · tools 1 ·
config 3 · packaging 2 · deploy 1 · docs 1. By size: M 9 · S 14 · L 0 — the second pass's one
L (the sim plant) closed.
Down from 19 open + 7 in flight at `9074661`: **all 7 in-flight rows and 5 register rows
closed** (old rows 1–4 and 10 — the entire top of the second pass's table), **8 rows opened**,
one of them a regression left behind by a runtime-side closure.

Minor nits recorded but not filed as rows: `queued_duration` still contributes 0 for queued
cartesian moves until they start (documented; no WC consumer); an *unconfirmed* `select_tool`
still latches the client-side tool key (`async_client.py:1449-1452` contradicts its own
comment); `home(wait=True)` raises `TimeoutError` where `move_j(wait=True)` returns silently;
no `--log-level` passthrough to `par6d` (`RUST_LOG` governs); a stale e2e docstring claims the
jog blocked-mask never reaches STATUS (`test_e2e_daemon.py:642-644` vs `server.rs:1543-1570`);
the deploy README disagrees with itself on the closure size (22 libraries at `:51`, 20 at
`:193`); `joint_speeds`/`is_robot_stopped` diverge in units from parol6 (rad/s vs steps/s —
par6 is the waldoctl-conforming side; one line in a porting guide).

---

## 1. waldoctl `Robot` ABC surface

par6 implements every abstract member correctly (`python/par6/robot.py:339-787`), plus a public
`jacobian()` helper the ABC does not require (`:644-651`, used by the dry-run jog integrator).
The remaining gaps are the collision block and the preview-fidelity items.

| Member | parol6 | par6 | Verdict |
|---|---|---|---|
| `name`, `joints`, `native_tools`, `position_unit`, `joint_index_mapping`, `backend_package`, client classes, `fk`, `ik`, `fk_batch`, `ik_batch`, `check_limits`, `set_active_tool`, factories, `start`, `stop`, `is_available` | ✔ | ✔ (`ik` additionally wrap-normalizes into the soft window) | parity |
| `has_force_torque` / `has_freedrive` / `cartesian_frames` | explicit `False` / WRF+TRF | inherits identical waldoctl defaults | parity |
| `joints` metadata (names, limits, home) | hand-carried constants (`parol6/robot.py:396-422`) | read from the runtime's own packaged TOML (`config.py:136-183`); `joints.home` **equals the pose the runtime's HOME fast-path returns to** (`planner.rs:340-353`) | ✔ arguably better — single source of truth |
| `urdf_path` / `mesh_dir` | one static tree | the **active tool's** packaged tree (`robot.py:461-469`, `config.py:60-83`) — the 3-D view renders the gripper actually fitted | ✔ better |
| `cartesian_limits` | constants pasted from an offline derivation | the same derivation run live against par6's own Jacobian, acceleration from the jog ramp's real profile, cache invalidated per tool change (`robot.py:268-320,407-436,574-575`) | ✔ better |
| `create_dry_run_client` | dispatches through the real command registry | full preview incl. arcs/splines/process moves/blend chains; refusal texts byte-match the server; geometry/blend/jog constants equal the runtime's (`motion.py:44-77` vs `planner.rs:117-207`, `bridge.rs:74-77`) | parity — **except the three mirror breaks**: [rows 3, 6, 7](#row-3) |
| `motion_profiles` | 5 names | 3 names, all real; dry-run agrees exactly | [row 22](#row-22) |
| `digital_inputs` / `digital_outputs` | 2/2, both real | 2/2, neither real (`robot.py:446-457`) | [row 9](#row-9) |
| collision block (`has_collision_checking`, `in_collision`, `colliding_pairs`, `check_trajectory`, `min_distance`, `apply_shapes`) | all real over `pinokin.CollisionChecker` (`robot.py:604-607,797-864`) | **entirely absent — all six inherit the disabled defaults** | [row 1](#row-1) |
| `native_tools[*].meshes` | 1–3 loose meshes | deliberately `()` — geometry rides the per-tool URDF tree (`tools.py:8-13`) | DIV, keep; never "close" |
| `native_tools[*].variants` | 2–3 per gripper | `()` everywhere | [row 13](#row-13) |

### <a name="row-1"></a><a name="row-2"></a>Rows 1, 2 — the collision surface (M, GAP; verified ×2)

- **parol6:** a process-global `pinokin.CollisionChecker` built with an SRDF
  (`PAROL6_ROBOT.py:13,57,96-138`; `load_srdf` at `:109-113`) backs all six members
  (`robot.py:797-864`), tool geometry synced on `set_active_tool`, prefixed display
  vocabulary.
- **par6:** zero hits for any of the six member names or `CollisionChecker` across `python/`
  at HEAD. par6's own commit `874af6d` said it plainly: *"has_collision_checking stays
  False: … none of which par6 implements."* Nothing since has touched it.
- **Since the second pass the contradiction sharpened again:** the runtime now gates planned
  motion (`planner.rs:422-527`) **and** every jog/servo stream (`bridge.rs:127-347`,
  `StreamGate`) while the offline surface gates nothing. In WC: `path_visualizer.py:80-81`
  skips the whole per-segment pass, `:276-279` the preview world push;
  `scene_handle.py:87,207` push keep-outs into nothing; `urdf_scene.py:1460-1467`
  editing-pose tint dead.
- **Why it is now unblocked:** pinokin is a declared dependency
  (`python/pyproject.toml:22-31`) and `52ccf9f` packaged the per-variant SRDFs at
  `python/par6/_data/urdf/*/srdf/`. Building the checker on the **active tool's own tree**
  (`par6.config.urdf_path`) gives tool geometry for free, and applying that tree's SRDF keeps
  the pair set identical to the runtime's — without it, the SSG48 `jaw1`-`jaw2` pair
  (colliding in 100 % of 20 000 samples, per the SRDF's own annotation) would flag every pose.
- **Dependency ordering for row 2:** once the ABC checker exists, WC's own per-segment
  `check_trajectory` pass restores collision display over dry-run trajectories even if the
  dry-run client keeps not refusing; mirroring the runtime's `SYS_SELF_COLLISION` refusal in
  the preview is then a small follow-up, not the blocker. Today the dry-run's `set_shapes`
  stores a tuple only its own `shapes()` echo reads (`dry_run_client.py:1085-1087`), and
  `jog_l`/`servo_j` integrate straight through keep-outs (`:902-1044`).

### <a name="row-3"></a><a name="row-6"></a><a name="row-7"></a>Rows 3, 6, 7 — preview fidelity (REG + 2 GAP; verified ×4)

The dry-run client was last touched in `9074661`; the runtime kept moving. Three breaks, all
adversarially verified:

- **Row 3 (REG): `home()`.** Runtime at HEAD fast-paths HOME on an already-homed arm into a
  planned, collision-gated return to the configured park pose (`planner.rs:1250-1269`,
  `home_pose_rad = park_pose_rad` at `:340-353`; landed `32c95f3`, matching parol6's
  `motion_planner.py:237-241`). The preview unconditionally snaps to the homing-sequence
  final pose with duration 0 and no drawn path, and its docstring asserts the runtime "does
  not short-circuit when already homed" — false at HEAD (`dry_run_client.py:665-675`). The
  client even tracks `self._homed`, yet `home()` never consults it — **no code path can make
  preview and runtime agree**. WC previews default `initial_homed=True`
  (`path_visualizer.py:174,527`), the recorder emits `rbt.home()` (`motion_recorder.py:325`),
  and 4 of 6 shipped demos call it: the preview jumps to ≈[90, −106, 163, 0, −29, 180]° while
  the arm plans a visible return to [0, −90, 180, 0, 0, 180]°, and every subsequent
  relative/TRF move previews from the wrong pose until the first absolute move re-converges.
  Fix: mirror the runtime's branch — homed ⇒ plan a joint move to `joints.home` (drawing the
  path), unhomed ⇒ snap to `homing_ready_pose_rad`.
- **Row 6: `teleport`.** The runtime refuses any out-of-window angle — "teleport places the
  arm exactly where it is told or not at all" (`server.rs:941-960`, `teleport_angle_fault`
  `:1806-1823`, incl. `tool_positions` dof/range checks) and latches the refusal visible to
  `error()`/STATUS. The preview accepts and **clamps** to hard limits
  (`dry_run_client.py:677-698`), its docstring claims the runtime clamps (stale since
  `0810311`), and `tool_positions` are ignored entirely. WC's playback drives
  `client.teleport` continuously (`playback.py:556-561`). Cheap fix: raise the runtime's own
  `COMM_VALIDATION_ERROR` template from the config windows the preview already loads.
- **Row 7: missing methods.** No `write_io`, no `status()`/`error()`/`activity()`, no
  parol6-style `__getattr__` catch-all. The live clients *do* have `write_io`
  (`async_client.py:1416`, `sync_client.py:423`), so palette and preview disagree; WC's
  preview wrapper resolves methods by `getattr` (`path_preview_client.py:605,645,671`) and
  raises `AttributeError`. **WC's default pre-filled editor script is
  `rbt.home(); status = rbt.status()`** (`simulation_engine.py:41-53`) — the very first thing
  a new WC-on-par6 user previews errors out. Even under par6's no-outputs truth, the honest
  mirror is a `write_io` stub raising the runtime's own refusal text (`server.rs:510-517`),
  plus the few query stubs the recorder and default script emit.

### <a name="row-13"></a>Row 13 — tool variants and the `variant_key` echo (S, GAP; verified)

- **Variants:** parol6 registers 2–3 `ToolVariant`s per gripper with per-variant TCP/mesh
  overrides (`parol6/tools.py:556-,609-,680-`). par6's `build_tools` sets none and the
  gripper TOMLs carry none (`python/par6/tools.py:127-177`) — one TOML/tree per fitted build.
  WC's variant selector stays hidden and `resolve_variant_tcp` degrades to base TCP. Real
  hardware options — SSG-48 finger vs pinch jaws — are inexpressible, and the TCP would be
  wrong for the un-modeled jaw set.
- **And the wire still accepts one anyway:** the codec length-checks 1..64 chars
  (`command.rs:669-671`); the server stores the key, clears the TCP offset on change, echoes
  it in `tool_status` (`server.rs:1087-1099,1499`), and syncs it into
  `PlanContext.tool_variant` (`runtime.rs:80`) — **which no par6d code reads**
  (`planner.rs:1960-1997`). WC persists and reapplies stored keys at startup
  (`main.py:1107-1110`), so a typo'd key survives forever with zero effect. `CLAUDE.md`:
  *"Never ship declared-but-unimplemented API surface."* Resolve jointly: declare variants on
  the specs (per-variant `tcp` overrides, which `waldoctl.resolve_variant_tcp` already
  honors — now plausible, the per-variant SRDFs exist) or drop the parameter from the wire.

### <a name="row-22"></a>Row 22 — motion profiles (S, DIV)

parol6: `TOPPRA/RUCKIG/QUINTIC/TRAPEZOID/LINEAR`. par6: `RUCKIG/TRAPEZOID/TOPPRA`
(`robot.py:477-491`), all real on the shipped ffi build; the dry-run agrees exactly and refuses
the others with `SYS_PROFILE_INVALID` (`motion.py:44-47`, `dry_run_client.py:1078-1083`).
Defensible (RUCKIG supersedes QUINTIC point-to-point; LINEAR is a debug profile), now with an
honest docstring documenting the TOPPRA-requires-ffi caveat. Cost unchanged: WC's picker is
shorter and a parol6 script naming `QUINTIC`/`LINEAR` errors — consistently on runtime and
preview alike.

---

## 2. waldoctl `RobotClient` ABC surface

**The palette is now a strict superset.** By WC's own `Category:`-docstring scan: par6 **48**
entries vs parol6's 47, with exactly one par6-only entry — `wait_command`
(`async_client.py:794-809`) — and the second pass's two only-parol6 entries
(`is_estop_pressed`, `is_robot_stopped`) closed with waldoctl-conforming semantics
(`async_client.py:1697-1726`; parol6's threshold is 2.0 steps/s, par6's 0.01 rad/s — par6
matches waldoctl's `StatusBuffer.speeds` spec, so par6 is the conforming side). No
`Category:`-tagged method inherits waldoctl's `NotImplementedError` default.

### Old row 4 — fire-and-forget refusals: closed **as designed**, not as proposed

`_fire` still encodes, sends, and returns `1` without awaiting (`async_client.py:705-714`) —
no refusal-wait was added. Instead delivery is three-channel (`15105b5`, issue #23): (1) the
server answers a refused teleport/jog/servo with a **real ERROR** and **latches it as the
standing error** whenever nothing truer stands (`server.rs:687-763`; `latch_faf_refusal`
`:1183-1194` — skipped while motion is in flight, cleared by the next accepted motion), so
`error()`, `STATUS.error` and `action_state=ERROR` carry the reason; (2) the client warn-logs
the otherwise-unclaimed ERROR datagram, throttled per code (`async_client.py:539-585`);
(3) since `4c15ac6` the RT jog latch withdraws refused directions from `STATUS.joint_en`, so
WC greys the jog buttons the tick the RT stops honoring them.

waldoctl itself labels the servo family fire-and-forget, and both backends' FAF paths violate
the "never report success unconfirmed" docstring equally — but a standing error + STATUS is
what a UI can actually consume at 20–50 Hz jog rates. Grade: closed, and par6 is now *more*
visible than parol6 here (parol6 sends a refused FAF **nothing at all** and latches no
`state.error`). Residual sliver: a refusal issued mid-program is only in the log, because the
busy pipeline's own errors take precedence — defensible.

### The rest of the client surface

- **`StatusBuffer` conformance:** every Protocol field present, decoded, server-filled with
  real data (`wire.py:627-696`, `server.rs:1560-1607`); `tool_status` structurally identical
  to `waldoctl.ToolStatus`, key canonicalized on every packet. Superset extras: v2 header,
  error 6-tuple, `queued_segments`/`queued_duration`, `accepted_index`.
- **`wait_command`:** COMPLETE pushes resolve per-index waiters with the status stream as
  fallback under the `accepted_index` stale-error ordering rule; failures raise a structured
  `RobotError`; queued sends are idempotency-keyed and retried, the server re-acking the
  original index so a retry can never double-enqueue (`async_client.py:686-703,781-851`;
  `server.rs:765-781,806-809`). Ahead — see [the ledger](#where-par6-is-ahead).
- **`wait_motion` / `wait_checkpoint` / `wait_status` / `stream_status(_shared)`:** identical
  algorithms, defaults, copy-vs-shared split and termination contract (`:720-917`).
- **Sync facade:** method-for-method mirror plus `queue_state`, `set_completion_policy`,
  `set_recipe`, `status_seq_gaps`, public `bind_tools`; module-level daemon loop with atexit
  shutdown, and an actionable error instead of a deadlock from inside a running loop
  (`sync_client.py:84-95`).
- **Telemetry surface:** `set_recipe` (unknown names refused server-side),
  `set_completion_policy`, `status_seq_gaps` — no parol6 equivalent exists at all.
- **`select_tool` latching:** par6 latches `_active_tool_key` only after the queued ack, so a
  rejection never leaves `client.tool` pointing at hardware not on the arm — parol6 latches
  before sending. (Nit: an *unconfirmed* selection still latches.)

---

## 3. Command surface

parol6 registers **46** commands; par6-proto defines **48**, all dispatched
(`par6-proto/src/enums.rs:33-137`). The two par6-only commands are honoured
(`SET_COMPLETION_POLICY`, `SET_RECIPE` — `server.rs:574-590`); nothing parol6 registers is
absent. The ack taxonomy (SYSTEM/QUERY/FIRE_AND_FORGET/QUEUED) matches parol6's four classes
member-for-member, but as one queryable table (`command_class()`, `enums.rs:249-304`) feeding
both gating and ack shape — removing the class of bug where parol6's three hand-maintained
sets and its registry disagree. (parol6's `PAROL6_FORCE_ACK` override has no analogue; nothing
in WC uses it.)

### Commands par6 defines but does not honour — still five entries

| Command | par6 behaviour | parol6 | Verdict |
|---|---|---|---|
| `WRITE_IO` | always `COMM_VALIDATION_ERROR "write_io is unavailable: this runtime drives no digital outputs"` (`server.rs:505-521`; verified: the `RtCommands::write_io` hook is never reached from the wire) | writes the output port | [row 9](#row-9) DIV+GAP |
| `SIMULATOR(false)` | bridge refuses any state **change**: `MOTN_SETUP_FAILED "live backend switching is not wired yet; restart par6d with/without --sim"` (`bridge.rs:879-891`). New since `9074661`: re-asserting the **current** state returns Ok — so WC's startup `simulator(True)` under `--sim` and the hardware-detect no-op path stopped erroring | live transport switch | [row 8](#row-8) GAP |
| `CONNECT_HARDWARE` | `MOTN_SETUP_FAILED "cannot switch to hardware bus … while running"` (`bridge.rs:893-905`); par6 deliberately has no port-persistence analogue — the CAN interface is named in the TOML | persists the port and reconnects | [row 8](#row-8) GAP |
| `SELECT_TOOL` (non-fitted) | `COMM_VALIDATION_ERROR "tool 'X' is not fitted; this runtime is running 'Y' (change robot.active_gripper and restart par6d)"` (`server.rs:855-887`) | any registered tool | [row 10](#row-10) DIV+GAP |
| `JOG_J` >1 non-zero axis | "jog_j drives one joint at a time" (`server.rs:927-935`) — **but the refusal now latches as the standing error and the client documents the surface** | multi-joint | row 21 DIV |

### <a name="row-8"></a>Row 8 — no live simulator/hardware switch (M, GAP; verified)

WC's Robot/Sim toggle (`control.py:1809-1837`) and COM-port picker (`settings.py:253-255`)
still raise on every actual switch; only the no-op direction stopped erroring. parol6 cancels
the pipeline and switches its serial transport live (`transport_manager.py:144`). par6's
backend is chosen by `--sim` at process start; until a bus hot-swap exists (or `Robot.start`
restarts the daemon), either wire it or have WC hide the toggle per-backend. The COM-port
*persistence* half is justified divergence (nothing to scan or persist on SocketCAN); the
live-switch half is the real gap.

### <a name="row-9"></a>Row 9 — digital I/O (M, DIV+GAP; verified ×2)

Unchanged through thirteen more commits; the oldest untouched row. The divergence stays
justified and documented (`server.rs:505-509`: no output frame on the CAN protocol). The gap
is the surfacing: `io()` returns `[0,0,0,0,!estop]` (`server.rs:1406-1413`),
`Robot.digital_inputs/digital_outputs` still say 2/2 (`robot.py:446-457`, counts tied to the
wire's `IO_SLOTS` layout), WC renders four dead chips with always-erroring toggles
(`io.py:26,35-36`), and `motion_recorder.py:346` still emits `rbt.write_io(...)` lines that
fail on replay and raise in preview ([row 7](#row-7)). Fix unchanged: report 0/0 and decouple
the counts from the STATUS buffer sizing — both sides of that coupling move together
(`main.py:452-467` sizes its copy loop from the same counts).

### <a name="row-10"></a>Row 10 — `select_tool` only accepts the fitted tool (M, DIV+GAP)

The reasoning is sound (kinematic/gravity/collision models are built from
`[robot].active_gripper` at startup) and the dry-run mirrors the refusal byte-for-byte. But
the operator-facing decision the second pass asked for — a backend-supplied "selectable" set
on waldoctl, or model rebuild on selection — was never made; WC's dropdown remains a trap
(three entries from `robot.tools.available`, two error-toast; `settings.py:300-328`). Two
shipped demo programs `select_tool('SSG-48')` and fail in preview and on replay.

### <a name="row-11"></a><a name="row-14"></a>Rows 11, 14 — the unhomed-jog divergence and the remedy text that lies

parol6 gates only *planned* motion on homing; jog/servo are deliberately ungated — *"an
unhomed arm may need to be jogged clear of an obstruction before homing"*
(`parol6/commands/base.py:23-36`, part of PR #27). par6's `needs_homed` covers **all** motion
including every jog and servo stream (`gating.rs:19-27,41-56`), because the RT keys
direction-block latching and limits on a home reference — a server-permitted jog the RT
refuses would vanish silently, so the table refuses loudly instead. Internally consistent and
documented in par6's own comment (which names parol6's opposite choice) — but it removes a
real bring-up capability: an arm that boots jammed against an obstacle cannot be jogged
clear, and the homing sequence itself moves the arm. Worth an explicit decision: an RT
jog-unhomed mode with velocity-only ramping (parol6's semantics), or a written deviation
entry accepting the loss.

**Row 14 makes it worse than it needs to be:** the `MOTN_NOT_HOMED` remedy text was ported
verbatim from parol6 — *"Run home first; jogging remains available."* (`error.rs:212-217`) —
where it was true. On par6 the error a pre-homing jog raises promises the very thing that was
just refused. One-line template fix.

### <a name="row-20"></a>Row 20 — error-code numbering (S, DIV)

Slightly worse than the second pass stated: **three** numeric collisions, not two. parol6:
`SYS_PORT_SAVE_FAILED=52`, `SYS_PROFILE_INVALID=53`, `SYS_SELF_COLLISION=54`; par6:
`SysProfileInvalid=52`, `SysSelfCollision=53`, `SysNotSimulator=54`
(`error.rs:59-64`; generated mirror `constants.py:169-171`). par6 has 26 codes (8 par6-only),
parol6 19 (1 parol6-only: port-save, which par6 has no analogue for — the source of the
renumbering). WC never keys on numeric codes, so impact is scripts-only; the enums are
declared **frozen contract data**, so document rather than renumber.

### Queue engine — parity or ahead on every semantics point checked

Idempotency dedup with re-ack of the original index (parol6 has none — a retry
double-enqueues); a monotonic index allocator surviving `reset_state` (**now parity** —
reference HEAD adopted the rule with par6's own rationale in the comment,
`parol6/server/state.py:363-368`); COMPLETE pushes per finished command, including per
blended-away command and `COMPLETE(ok=false)` with the attributed error (parol6 remains
poll-only); `stop(clear_queue=)` scope superset with a contract-matching default;
`estop`/`reset`/`reset_state` all diverge in the direction the waldoctl contract prescribes
(latched e-stop until explicit reset; reset waits on the RT's actual enable verdict);
same-type streams update in place, a **refused** update cancels the stream it was updating,
and a type change drains **only** the superseded stream's datagrams (`server.rs:1227-1247`) —
a concrete safety improvement over parol6's blind full-buffer drain, which can destroy
another client's buffered estop. Queue cap 128 vs 100, same `COMM_QUEUE_FULL`;
checkpoint/delay/blend-hold semantics match. The acceptance-clears-error rule matches parol6,
and `latch_faf_refusal` deliberately refuses to latch while motion is in flight to preserve
`wait_command`'s stale-error ordering.

---

## 4. Status / telemetry fields

The v2 STATUS packet (31 elements) remains a superset of parol6's 24. Every field waldoctl's
`StatusBuffer` Protocol requires is present, filled with real data, and decoded
(`status.rs:1-99`, `server.rs:1560-1607`, `wire.py:627-695`). The fields the second pass
listed as in flight are all real now:

| Field / surface | Status at `4c15ac6` |
|---|---|
| `loop_stats` std/min/p95/p99 | real end to end — rolling 500-sample window in the RT (`timing.rs:120-141`, with a non-zeros regression test), EMA mean, wire arity enforced; the two backends' 10-field results are field-for-field identical. No WC consumer; scripts-only. |
| `queued_duration` | real planned estimate for joint-space content (MoveJ under the actual profile + Delay + tick-accurate in-flight remainder, `planner.rs:2082-2166`); queued cartesian moves without explicit duration still contribute 0 until they start — documented, honest, and cheap (timing one means running the move's full IK). No WC consumer. |
| `pose(frame="TRF")` | world-in-tool inverse (transposed rotation, mm translation), shared RPY convention (`server.rs:1625-1633,1788-1800`). |
| `collision_active` / `collision_pairs` | two latches, one field: the planner latch **and** the new StreamGate latch merge at status cadence (`server.rs:1361-1377`), cleared on next accepted motion. The second pass's "collision_active never fires during an ungated jog" is retired — the stream gate populates it. Still latched-at-refusal, not a live "in collision now" flag — on both backends, and now codified as the waldoctl contract. |
| `joint_en` | **ahead:** during `Mode::Jog` the RT's blocked mask withdraws the exact blocked direction on every STATUS/REACHABLE read, mapped bit-correctly to the `[j+, j-]` wire order, reverting when the jog ends (`server.rs:1543-1570`, landed `4c15ac6`). WC's jog buttons grey the tick the RT actually stops honoring a direction — at jog speed, tens of degrees before the static soft-limit margin. No parol6 equivalent. |
| `scene_epoch`, `homed`, `tool_status`, `accepted_index`, v2 header | real; `scene_epoch` moves only on accepted layer replacement, driving WC's world readback with in-flight-push suppression. |

### <a name="row-5"></a>Row 5 — `collision_pairs` vocabulary (S, GAP; verified)

waldoctl mandates `shape:<name>` for program keep-outs, `install:<name>` for installation
keep-outs, `tool:<key>:<part>` for tool geometry — *"never backend-internal geometry
identifiers"* (`waldoctl/status.py:61-65`). parol6 implements it exactly
(`PAROL6_ROBOT.py:202,322,348`). par6 reports the shape's verbatim config/wire name and the
mesh-index-stripped link name — no prefixes anywhere in the repo, and par6's own e2e locks the
raw name in (`test_e2e_daemon.py:838` asserts `"keepout" in name`, not `"shape:keepout"`).
WC keys keep-out scene objects as `f"{prefix}:{s.name}"` and only resolves prefixed names to
shape objects (`urdf_scene.py:1471-1485,1577`), so a par6 keep-out collision tints the arm
links (link names still resolve) but never the offending shape; error text still names the
pair, so diagnosis survives. Fix at the two `display()` sites (`planner.rs:2199-2214`,
`bridge.rs:224-229`); `tool:` is arguably N/A since par6's tool geometry is real URDF links.

### Telemetry recipes — ahead, with one rough edge

`SET_RECIPE` selects among 5 stock recipes over 19 fields (measured/filtered/commanded/target
kinematics, gravity torques, per-node temps/voltages/currents, loop stats;
`telemetry.rs:22-145`), unknown names refused with `COMM_UNKNOWN_RECIPE`, streamed as msgpack
at `telemetry_rate_hz`; no flow until selected. parol6 has no telemetry stream at all. Caveat:
the packaged client can *select* a recipe but ships no reader/decoder — only the test rigs
parse the port. Scripts-only rough edge, not a gap.

---

## <a name="behaviors"></a>5. Behaviors that are not commands

### Streaming collision gate — old row 1 (#19), closed and beyond parity

`StreamGate` (`bridge.rs:146-335`) holds its own collision world, mirrored layer-for-layer
from `set_shapes`, and applies parol6's `collision_blocked` rule at **datagram admission** for
`jog_j`/`servo_j`/`servo_l`/`servo_j_pose`/`jog_l`, with parol6's 0.15 s velocity-scaled
lookahead clamped to the soft window, plus a housekeeping re-check every 4 ms from the
**measured** pose (parol6's tick is ~10 ms), `jog_l` swept per integration step, held servo
targets re-checked on world-epoch change. Refusals answer a real ERROR, latch the standing
error and the collision fields, and reach `joint_en`. Behavioral tests pin it
(`test_e2e_daemon.py:788-874`, `ffi_kinematics.rs:1365-1460`: block, no-entry trace,
escape-outward allowed). **Beyond parity: parol6 gates jog only — its servo commands consult
no collision code at all.** The second pass's penetration numbers (47.8 mm jogged, 84.8 mm
servoed, silently) predate this and were not re-driven; the closure is verified in code.

Two residuals, recorded: servo **admission** checks the target configuration plus the escape
rule, not a swept path — a single datagram naming a distant target across a thin keep-out is
admitted if the target itself is clear (WC's teleop streams small per-tick deltas, so the
per-datagram checks effectively sweep for the consumer of record; exposure is scripts;
`Collision::check_segment` already exists if servo scripting grows). And `jog_l`'s collision
stop is an abrupt stream kill where parol6's cartesian jog decelerates and auto-resumes —
par6's client-driven re-admission converges to the same UX for WC's held-button jog, at the
cost of a jerkier stop and a throttled refusal log.

### <a name="sim-fidelity"></a>Sim fidelity — old rows 2, 3 (#21, #22) and #26, closed at the root — statically

The root cause of both the ~48 %-short landings and the converged-target servo stall was one
misreading: the sim driver treated the position-frame's speed channel as a per-command
velocity cap. `95cf98c` models it as what the vendor firmware actually implements — an
**additive velocity feedforward**: `vt = clamp(kpp·err + speed, ±vel_limit)`
(`sim/driver.rs:263-272`), so the position channel closes tracking error at full authority
even at zero commanded velocity. The stream law is unchanged in shape — `MotionStream` still
commands `max(profile velocity, position-channel advance)` (`adapters.rs:116-149`, regression
test `:175-224`); the second pass's proposed fix ("measured pose inside the stream law") was
rightly **not** taken: on real firmware the stall could never have manifested, so the trait
change would have papered over a sim-fidelity bug. `#26` (teleport residual freeze) fell out
of the same law: teleport now reseeds driver transients (`driver.rs:295-312`) and every
RT-side hold — starved-ring hold, stream tracker, jog integrator, `q_target`
(`core.rs:558-586`). par6's own suite tightened the tolerances that had compensated (`move_j`
landings 6°→1°, IK-wrap 25 mm→3 mm; sustained-stop and blend-motion-time assertions in
`51e7051`), the law is unit-pinned (`sim_bus.rs:792-860`), and with an honest plant `settled`
completes on tolerance and `strict` no longer bricks the controller on every sim move.

**Verification debt:** all of this is read, not run. Convergence is a numeric claim only a
live `par6d --sim` measurement can confirm; the second pass's headline numbers (+5.24° of a
commanded +10°) predate the law change and are stale either way. This is the register's one
`unverified` item and belongs to the next measurement pass.

### Collision model — old row 10 (#18) and the escape signal (#25)

`Collision::load` applies the variant's authored SRDF, and a **missing SRDF is a boot error,
not a fallback** (`collision.rs:135-163`); per-variant sampled SRDFs ship for flange/msg/
ssg48 (generated by `scripts/gen_srdf.py`, landed `52ccf9f`). The park-pose-derived
allow-list (`resting_pairs`/`park_contacts`) is gone — the second pass's over-refusal
mechanism is retired, and both collision instances (planner + StreamGate) load from the same
SRDF'd model.

The escape-depth signal (#25, `4c15ac6`) diverges from parol6 deliberately: the depth half
compares **world-pairs-only** distance (`Collision::world_distance` excludes self pairs) and
engages only when the standing collision involves a world shape; the commit records the
measurement that justified it — a deep self contact masked the watched keep-out under
parol6's global signal, and a truer per-link depth read a transverse multi-link escape as
deepening and trapped the arm. The trade: par6 loses parol6's guard against grinding deeper
through the *same arm-arm pair* from an arm-arm start collision — but with SRDF excluding
legitimate rests that state is already faulted, and the keep-out case (the one escape exists
for) is strictly better served. Defensible; documented only in code comments and the shim
header, worth a written deviation entry.

Planned-path gating exceeds the reference on both properties parol6 lacks: sample density
scaled to actual path length (0.02 rad joint pitch, bounded tunneling, terminal sample always
checked) and **in-flight revalidation** — `set_shapes` re-walks the running trajectory from
the sample nearest the measured pose and halts it with a real per-command error if the
remainder is now illegal (`planner.rs:537-574`).

### <a name="row-4"></a>Row 4 — streaming speed/accel parameters (S, GAP, new)

parol6 honors them: `jog_j` applies `set_limits(speed, accel)` (`basic_commands.py:90`),
`servo_l` applies both including the velocity-ratio rescale
(`servo_commands.py:249,285-288`), cartesian jog applies accel. par6 carries them on the wire
as `Option<f64>` and the client sends them — but `RtBridge::stream` destructures every one
away (`bridge.rs:527-744`; `MotionStream`'s executor limits are fixed at construction), and
`validate_supported` does not refuse them either (`server.rs:892-976`). WC passes `accel=` on
both jog paths and `speed=` on TCP drag; all silently inert (the jog speed *fraction* itself
is honored). Violates both repo rules at once — "refuse a parameter the runtime cannot
honour" and "never ship declared-but-unimplemented surface". Fix: plumb per-stream
`set_limits`/`set_accel_time_s`, or refuse non-default values.

### Jog, homing, blending, completion — at or ahead, divergences documented

- **Jog ramp:** trapezoid/s-curve with jerk-aware lookahead extended by the current
  acceleration state, per-joint direction-block latching, hard clamp on the measured pose
  (`par6-motion/src/jog.rs:213-320`); the blocked mask rides STATUS (§4). parol6's jog
  reports "Limit reached" with no live latch feedback.
- **Homing:** same fast path as parol6 for a referenced arm (the citation in `planner.rs`
  names parol6's file); only an unhomed arm runs the FSM. The post-home move drives a cubic
  Hermite whose signed profile tangent is the wire feedforward — zero at both ends so the
  position loop closes the landing — replacing the vendor's bare `(target, speed)` frame
  that parks the joint off target. parol6's referencing is firmware-driven; no comparison
  exists.
- **Blending/curves:** all three deliberate divergences the second pass endorsed are intact —
  `move_p` auto-blends corners *because proto and client promise it*, the spline is
  chord-length + natural with both divergences argued at the call site, and IK/timing
  failure **errors** instead of parol6's silent unblended fallback. `move_l` and joint chains
  plan as one path (the radius conversion cites parol6's `joint_commands.py` by name);
  `move_c` blend radius refused with a named reason. Chain completion was measured at the
  second pass and is now pinned by tests (`ffi_kinematics.rs:2244,2401`). Parity of
  capability without parity of bugs — keep.
- **Completion policies:** `commanded`/`settled`/`strict` per the spec reference
  implementation (`hooks.rs:377-446`), switchable on the wire at the next boundary. No
  parol6 surface exists. With the honest plant, `strict` only faults on a genuine
  no-settle — the "strict bricks the controller" consequence retired with its cause.

### <a name="row-12"></a>Row 12 — pneumatic/vacuum tools (M, DIV+GAP; the critic's find)

Two of parol6's five registered tools — PNEUMATIC (vertical/horizontal variants) and VACUUM,
both driven through a digital-output valve port (`parol6/tools.py:548-560,719-730`,
`gripper_commands.py:48-83`) — have **no par6 counterpart at any layer**: `DriverType` admits
only two electric CAN drivers (`par6-config/src/robot.rs:19-24`), the runtime's `tool_action`
accepts only `move|calibrate` (`planner.rs:782-830`), the client palette builds only
`PassiveTool`/`ElectricGripper`, and with `write_io` also refused a pneumatic owner has **no
actuation path at all**. Defensible at the runtime level — the CAN spec defines no output
frame, so valve control would be theatre — but it surfaces badly: the tool simply never
appears, with no error anywhere, and par6's own `select_tool` docstring still demonstrates
`rbt.select_tool("PNEUMATIC")` for a tool that cannot exist (`async_client.py:1443`). Remedy:
port the semantics of a valve-driver tool class behind a future bus output frame, or document
the hardware scope and fix the docstring example.

### Program / script execution

The stepping bootstrap patches whichever backend package `WALDO_BACKEND_PACKAGE` names, and
the default snippet is now backend-aware (`from par6 import RobotClient`, real host/port) —
only its `rbt.status()` line breaks ([row 7](#row-7)). The six shipped demo programs still
hardcode `from parol6 import RobotClient` and `port=5001`, and two also
`select_tool('SSG-48')` ([row 10](#row-10)); their `move_c`/`move_p`/`move_s` bodies run on
par6. Entirely WC-side friction — port them to the discovered backend or template the import.

---

## <a name="deploy"></a>6. Deployment — rows 15, 17

**Row 15 (aarch64) is unchanged in kind and honestly self-documented.** The deploy README
states it at HEAD: *"Nothing has been executed on aarch64 … the shim's numerics are
unverified on this target."* The static stand-ins are real and good — full DT_NEEDED closure
with no unresolved sonames, a measured `GLIBC_2.17` floor, every versioned symbol provided by
a shipped copy, a real aarch64 ELF with kinematics, `systemd-analyze verify`, the no-ffi boot
refusal test — but they say nothing about whether IK converges to the same answer on ARM.
Retired only by one native `cargo test -p par6-kin/-p par6d --features ffi` run on the box. A
deployment risk unique to par6; parol6's pure-Python numerics are ISA-independent by
construction.

**Row 17 (build-remedy texts) is new and cheap.** The daemon refuses a non-ffi build with a
message correctly naming `scripts/ffi/setup.sh` — but the client's not-found error says
"build with `cargo build -p par6d`" with no `--features ffi` (`robot.py:83-85`), and the
README quickstart produces exactly the refused configuration. The chain is at least loud (the
spawned binary exits with the actionable message), but the first two remedies a user follows
are wrong. WC's own e2e has the correct incantation; align the message and quickstart with it.

**CI:** broader in kind than parol6's — golden kinematics/collision fixtures, a real vcan0
with hard-fail-on-missing, the shipped 250 Hz release soak, daemon e2e over real UDP, WC's
full-app par6 e2e with fail-not-skip semantics, the staged aarch64 artifact. Not exercised
anywhere: any aarch64 *instruction*, real PF_CAN hardware (vcan is a kernel surrogate),
systemd unit start / `install.sh` end-to-end, and non-Linux Python clients — par6 pins macOS
and Windows pinokin wheels no CI job ever installs, where parol6 tests 3 OSes × Python
3.11–3.14. That last is the only place parol6's CI is genuinely broader.

---

## <a name="config"></a>7. Config surface — rows 16, 18, 19

- **[Row 16]** Softer than the second pass's "out of the box they do not meet": WC passes its
  configured port explicitly into both `start()` and `create_async_client`, and par6's
  `Robot.start` spawns `par6d` at that port — the **default spawn flow self-heals** onto 5001
  end to end. The mismatch now bites exactly two flows: attaching WC to a self-started or
  deployed `par6d` on its 6001 default (the hardware path — the deploy README shows the box
  on 6001; WC raises `ConnectionError` naming `host:5001`), and bare `par6.RobotClient()`
  scripts against a WC-spawned 5001 runtime. Still documented **nowhere a user finds**: zero
  par6/6001 mentions in any WC markdown, par6's README silent; only the (now-retired)
  `spec/PROTOCOL-V2.md` and the deploy README name 6001, and neither names WC's 5001 or
  `WALDO_CONTROLLER_PORT`. One paragraph in each README closes this. Related residue: WC
  forwards a *stored* `com_port` unconditionally, so a machine that once ran parol6 with a
  serial port hard-fails par6 startup until the key is cleared — self-explanatory message,
  noted rather than filed.
- **[Row 18]** Still absent: `options.rs` reads ports/hosts/transport/paths from env but no
  rate knob; `status_rate_hz` lives only in the TOML behind divide-the-tick-rate validation.
  par6's own harness still patches TOML text. The validation interlock is a defensible reason
  to keep it in config, but a `PAR6_STATUS_RATE_HZ` routed through the same validator would
  cost little and match the WC conftest pattern.
- **[Row 19]** `pip install par6` ships client-only; package data is config + per-tool URDF
  trees + **the per-variant SRDFs** (packaging kept pace with `52ccf9f`); per-platform
  pinokin wheels pinned incl. linux-aarch64. `par6d` resolves via `PAR6D_BIN` then PATH; the
  shim closure never ships via pip (dev runs need `.ffi/env.sh`; deploy uses the
  `/usr/local/lib/par6` rpath). On the never-close list — a Rust RT daemon does not belong in
  a wheel — but WC's `[par6]` extra still carries no note that `par6d` + shim arrive
  separately.
- **Otherwise ahead:** explicit CLI > `PAR6_*` env > TOML precedence with validated config
  load and actionable path errors, vs parol6's import-time env constants.

---

## 8. What Waldo Commander calls that par6 cannot satisfy

Re-swept call-site by call-site at WC HEAD against par6 `4c15ac6`. Shorter again — the entire
top half of the second pass's table (jog/servo near keep-outs, sim convergence, servo stall,
FAF invisibility) closed in code.

| WC call site | par6 result | Row |
|---|---|---|
| `path_visualizer.py:81,88,107,112,118`, `scene_handle.py:87,207`, `urdf_scene.py:1467` — collision surface | silent no-ops; preview draws what the runtime refuses | [1](#row-1), [2](#row-2) |
| any preview containing `rbt.home()` (recorder emits it; 4 of 6 demos call it) | previews from a pose ~90° off on J1 vs the real run | [3](#row-3) |
| default editor script `rbt.status()` (`simulation_engine.py:41-53`) | `AttributeError` on first preview | [7](#row-7) |
| recorded `rbt.write_io(...)` — preview and replay | `AttributeError` in preview; refused on replay | [7](#row-7), [9](#row-9) |
| `playback.py:556-561` out-of-range `teleport` in a recorded program | previews clean, fails live with a latched error | [6](#row-6) |
| `control.py:1402-1406` jog accel, `:1560-1566` jog_l accel, `:1539-1543` TCP-drag speed | silently inert | [4](#row-4) |
| keep-out tinting on a collision latch | arm links tint; the keep-out shape never does | [5](#row-5) |
| `control.py:1821` Robot/Sim toggle; `settings.py:253-255` COM-port picker | raises → red toast (no-op direction now succeeds) | [8](#row-8) |
| `io.py:26` toggles; `:35-36` four chips | always error; permanently 0 | [9](#row-9) |
| `settings.py:300-328` tool dropdown | 2 of 3 entries raise | [10](#row-10) |
| `settings.py:95-152` variant dropdown | hidden (empty variants); stored `variant_key` echoes forever | [13](#row-13) |
| any jog before homing | `MOTN_NOT_HOMED` whose remedy claims jogging is available | [11](#row-11), [14](#row-14) |
| attach to an externally-started `par6d` at its 6001 default | `ConnectionError` naming `host:5001` | [16](#config) |
| `programs/*.py` (`from parol6 import RobotClient`, `port=5001`; two `select_tool('SSG-48')`) | WC-side friction; bodies run once ported | — |

**Now satisfied, verified in code this pass** — everything else the second pass tabled:
jog/servo streams near keep-outs (gated at admission, re-checked live, `joint_en` greying),
`move_j`/`move_l` under `--sim` (feedforward law; static), servo-held targets (teleop,
scrubber, TCP drag), fire-and-forget refusal visibility (standing error → WC's action-log
FAILED transition — though WC never queries `client.error()`, so the refusal *text* lives
only in logs/status), `home()` fast-path runtime-side, exclusive `start()`, `par6d` log
forwarding into WC's log panel, the full 48-entry palette, every `StatusBuffer` field WC's
status loop reads, `scene_epoch`-driven world readback, canonicalized `tool_status`, the
envelope worker at `nq=6` across all three packaged trees.

---

## <a name="closed"></a>Closed since the second audit

Twelve rows and two issues. Where the fix differs from what the second pass proposed, that is
noted — three of the five biggest closures took a different (and better) route than the audit
suggested.

| Old # | Gap | Closed by | Note |
|---|---|---|---|
| 1 (#19) | Jog/servo streams not collision-gated | `15105b5`, `4c15ac6` | `StreamGate` at datagram admission + 4 ms housekeeping re-check from the measured pose; **par6 now also gates servo streams, which parol6 never gated at all**. Escape rule world-only (#25, below). Static verification; the second pass's penetration numbers were not re-driven. |
| 2 (#22) | Sim plant settles ~48 % short | `95cf98c`, tests `51e7051` | **Differs:** the root cause was the sim driver misreading the position-frame speed channel as a velocity cap; it is now the additive feedforward the firmware implements. Regression tolerances tightened 6°→1°. Convergence itself is this pass's one verification debt. |
| 3 (#21) | Servo stream stalls on a converged target | `95cf98c` | **Differs (better):** the audit proposed feeding the measured pose into the stream law; instead the plant was fixed — on real firmware the stall could never have manifested, so the trait change would have papered over a sim bug. Stream law pinned by `adapters.rs:175-224`. |
| 4 | Fire-and-forget refusals invisible to the client | `15105b5` (issue #23) | **Differs:** not a reply wait — refusals latch as the standing error (when the pipeline is idle) plus a real ERROR datagram, a throttled client warn-log, and (since `4c15ac6`) `joint_en` withdrawal. par6 is now *more* visible than parol6 here, inverting the row. |
| 10 (#18) | Collision allow-list derived from the park pose | `52ccf9f` | Per-variant sampled SRDFs applied in `Collision::load`; a missing SRDF is a boot error, not a fallback; `resting_pairs`/`park_contacts` gone. |
| i-f | `home()` re-runs full referencing; lands at the homing pose | `32c95f3` | Runtime only: homed ⇒ planned return to park at the vendor speed fraction, matching parol6's routing exactly. **The preview mirror was never updated — [row 3](#row-3), this pass's regression.** |
| i-f | No log forwarding (tempfile sink) | `32c95f3` | Reader thread into the `par6d.*` logger hierarchy with env_logger parsing, level mapping, panic detection. Nit: no `--log-level` passthrough. |
| i-f | `queued_duration` 0 for speed-parameterised moves | closed for joint content | MoveJ planned with the actual profile at estimate time; Delay counted; in-flight remainder tick-accurate. Cartesian-without-duration documented as 0 until started. |
| i-f | `loop_stats` std/min/p95/p99 hardcoded 0.0 | closed | Rolling window in `par6-rt/src/timing.rs` with a non-zeros regression test; wire results field-for-field identical across backends. |
| i-f | `pose(frame="TRF")` returns identity | closed | World-in-tool inverse with mm translation and the shared RPY convention. |
| i-f | `is_estop_pressed` / `is_robot_stopped` missing | closed | Palette-tagged; thresholds in rad/s per waldoctl's spec (parol6 uses steps/s — par6 is the conforming side). |
| i-f | `Robot.start()` reuses a running runtime | closed | Fail-hard exclusive on a PING answer (WC's `EXCLUSIVE_START` contract); refuses remote hosts for `--sim` and `com_port` with an actionable SocketCAN message. |
| — (#25) | Escape-depth signal | `4c15ac6` | World-only distance signal with the trade documented in §5; the commit records the measurement that justified it. |
| — (#26) | Teleport residual freeze in sim | `95cf98c` | Driver transients reset per joint + RT-side reseed of every held motion target; both halves verified wired. |

---

## <a name="where-par6-is-ahead"></a>Where par6 is ahead

Parity of *capability* is the goal, not parity of bugs. The ledger is rebased at reference
`829c2c7` — and the reference is **converging on par6's designs**: three items moved this
pass because parol6 adopted them.

**Retired — parol6 adopted them (design convergence, not par6 regression):** *structured
errors on the wire* (reference HEAD ships the identical KUKA-style 6-tuple from a full
catalog; par6 keeps only a catalog-breadth edge — `SysRtiLinkLost`/`SysLoopCritical`/
`SysJointFault` have no parol6 senders) and the *`scene_epoch`* clause (now in parol6's
STATUS too).

**Narrowed (half adopted, half still par6-only):**

1. **Push completion + acceptance ordering.** parol6 adopted `accepted_index` and the
   stale-error ordering rule verbatim in `wait_command` — but completion is still poll-only;
   the COMPLETE push (per finished command, per blended-away command, `ok=false` with the
   attributed error) remains par6-only and keeps `wait_command` loss-tolerant in a way
   parol6's is not.
2. **Idempotency-keyed enqueue + reset-surviving allocator.** parol6 adopted the allocator
   rule with par6's own rationale in the comment; it still has no idempotency keys, so a
   retry after a lost ack still double-enqueues there.

**Held, verified unchanged:**

3. **Explicit refusal over silent degradation** — and the second pass's caveat (the last hop
   to the client) is closed: refusals latch as the standing error. parol6 still accepts
   multi-joint jog and sends a refused FAF nothing.
4. **A build that cannot do the job does not run** (`NO_FFI_REFUSAL`).
5. **Errors derived from the RT latch, never stored** (`faults.rs`; the FAF latch stores into
   the *pre-existing* command-attributed slot, cleared on acceptance — the invariant's
   RT-latch half is intact).
6. **Status header** (`seq`, `mono_time_ns`, `link_ok`, `data_age_ms`, `controller_id`);
   STATUS keeps broadcasting with staleness flags when the bus link is down.
7. **Blending refuses rather than falls back** — parol6 still logs-and-runs the unblended
   move on blend IK failure; its TOPPRA→LINEAR fallback is a second instance of the pattern.
8. **Cartesian enablement measured, not asserted** — all-zeros until the first probe.
9. **One TCP representation** — the tool tree's own `tcp` link on both sides.

**New entries this pass:**

10. **Chunked bulk transfers** (the critic's ledger addition): CHUNK envelope, per-transfer
    reassembly, `COMM_CHUNK_TIMEOUT` with received/total counts, client auto-splitting past
    the MTU. parol6 has no chunking anywhere — a large spline or shape world must fit one
    datagram.
11. **The RT jog latch on the wire** — `joint_en` withdraws the exact direction the jog
    engine has latched, mid-jog, live (`4c15ac6`).
12. **Selective stream drain** — type-change preemption drains only the superseded stream's
    datagrams; parol6 blindly drains the whole UDP buffer, which can destroy another
    client's buffered estop.
13. **Telemetry recipe stream**, promoted to a named item — no parol6 analogue of any kind.
14. **Completion policies** (`commanded`/`settled`/`strict`) as first-class wire surface.
15. **The simulator is a plant, not an echo** — three tiers (kinematic, Pinocchio ABA
    dynamics, MuJoCo contact scene) behind the production codec: cascade loops from config
    gains, hall-sensor homing driven by the **real** homing FSM (vs parol6's
    countdown-and-teleport), per-type fault injection mapping 1:1 onto the wire flag bits, a
    pressable e-stop line, MuJoCo grasp objects surfacing through the real detection bits.
    parol6's mock cannot rehearse any fault path — it force-releases the e-stop every tick.
    Caveat worth writing down: par6's injection surface is a Rust test API, unreachable over
    the wire — ahead, but one layer short of the user.
16. Plus, unchanged from the second pass: per-tool URDF trees, live Jacobian-derived
    cartesian limits, cross-language golden vectors, the generated constants mirror with a
    staleness test, config-as-TOML with CLI>env>TOML precedence, `select_tool` latching only
    after the ack.

**On the MCP question:** parol6's PR #27 contains no MCP server — the MCP server is a
frontend/waldoctl feature (FastMCP over the public `commander.*` surface, session lease,
first-move human gate) riding the `RobotClient` contract, so par6 serves it identically with
nothing backend-specific to build. What MCP changes is the price of silent-failure gaps: an
LLM operator cannot see the arm. parol6's stale-error bug was found exactly that way; par6's
now-closed FAF-refusal row is the same class. The register's remaining silent paths (row 4's
inert parameters above all) matter more under MCP, not less.

---

## <a name="what-parity-means-now"></a>9. What "parity" means now

The runtime crossed the line this pass; the register is now a story about the mirror. Three
statements.

**What a Waldo Commander user would still notice.** Almost all of it is the preview, not the
arm. (a) The editor's path preview disagrees with the runtime wherever it matters most: it
draws paths and jogs through keep-outs a doubly-gated runtime refuses ([rows 1–2](#row-1)),
previews `home()` from a pose ~90° away from where the arm actually goes ([row 3](#row-3)),
clamps a teleport the runtime refuses ([row 6](#row-6)), and errors on the default editor
script's `rbt.status()` line ([row 7](#row-7)). (b) The same three buttons still raise: the
Robot/Sim toggle, the I/O toggles, and two of the three tool-dropdown entries. (c) Two
sliders are silently inert: jog acceleration and TCP-drag speed ([row 4](#row-4)). (d) A
keep-out you collide with never tints red — the arm links do ([row 5](#row-5)). (e) Jogging
before homing refuses — loudly, but with a remedy text claiming the opposite
([rows 11, 14](#row-11)). Everything the second pass measured as broken *behavior* — the sim
falling half short, the servo stall, the ungated jog, the invisible refusals — is closed in
code.

**What only matters on hardware.** The aarch64 shim's numerics have still never executed on
the target ISA — the one risk no amount of x86 CI retires, unchanged for two passes
([row 15](#deploy)). The digital-I/O absence and the pneumatic/vacuum family
([rows 9, 12](#row-9)) are facts of the bus — no output frame exists — so the divergence is
permanent; only the surfacing (2/2 counts, dead chips, a docstring selling a tool that cannot
exist) is fixable. The unhomed-jog gate ([row 11](#row-11)) only hurts during physical
bring-up — which is exactly when it hurts most.

**What is a justified divergence and should never be closed.** Refusing a parameter the
runtime cannot honour instead of silently altering it — which is precisely why
[row 4](#row-4) is a bug and not a policy. Refusing to boot without kinematics. Refusing a
blended path that fails its checks instead of falling back. Latching e-stop until an explicit
reset. Delivering FAF refusals through the standing error rather than a reply-wait. Tool
geometry through per-tool URDF trees; the cartesian envelope from the live Jacobian;
enablement measured, not asserted. SI units in the client where parol6 speaks firmware steps.
The SRDF-exact collision world on both gates, and the world-only escape signal a recorded
measurement justified. `par6d` outside the pip package. `move_p` auto-blending its corners.
In each case par6 does the more correct thing, and "parity" would be a regression.

---

## <a name="appendix-a"></a>Appendix A — how this pass was conducted

**No live daemon was driven.** This pass is static by design: the second pass's measurement
rig established the behavioral baselines; this pass verifies what the thirteen intervening
commits did to them, in code.

- **Seven parallel area auditors**, each reading both codebases at the pinned commits:
  (1) `Robot` ABC + client-side collision/preview stack, (2) `RobotClient` ABC + reply
  semantics + client plumbing, (3) wire command surface + server gating/queue semantics,
  (4) motion/streaming/runtime collision enforcement, (5) status/telemetry/config/
  packaging/CI/deploy, (6) the WC call-site sweep (§8 rebuilt), (7) architecture + the
  "ahead" ledger. 132 items total.
- **Twelve adversarial verifications.** The auditors produced 25 unique gap claims; the
  twelve highest-ranked went to independent verifiers instructed to refute them. **All
  twelve stood**, several with sharpened evidence (the dry-run `home()` tracks `self._homed`
  and still never consults it; par6's own e2e asserts the unprefixed collision pair name).
  The verified set: the `Robot` ABC collision surface (×2, independently), the dry-run
  collision blindness, the dry-run `home()` regression (×2), the teleport clamp, the missing
  `write_io`, digital I/O 2/2 (×2), `ToolSpec.variants`, the `collision_pairs` vocabulary,
  `WRITE_IO`, `SIMULATOR(false)`.
- **Not adversarially re-verified** (persisting rows re-confirmed by their area auditor, or
  lower-ranked new claims): `CONNECT_HARDWARE`, `SELECT_TOOL` non-fitted, the streaming
  speed/accel parameters, the status-area duplicate of the vocabulary row, the 6001/5001
  port row, aarch64, the WC simulator-toggle and I/O/tool-dropdown call-site rows, the
  recorded-`write_io` row, the default-script `status()` row, the live transport hot-swap,
  the unhomed-jog divergence.
- **A completeness critic** swept for blind spots: e-stop/reset recovery, WC
  startup/shutdown into `start()`/`stop()`, waldoctl's optional members, the sync facade,
  the ToolStatus query, and the parol6-MCP question all checked clean; its finds —
  [row 12](#row-12) (pneumatic/vacuum), the `joint_speeds` units correction, the chunk
  envelope's absence from the ahead ledger — are folded in above.
- **Marking convention:** claims resting on the second pass's live measurements say
  *measured at the second pass*; closures whose correctness is numeric (the sim plant) are
  carried as verification debt, not asserted. The next live-measurement pass owes exactly
  one number: does the sim arm now arrive.
