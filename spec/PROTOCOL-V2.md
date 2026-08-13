# PROTOCOL-V2.md — client ↔ runtime wire protocol

The contract implemented by `par6-proto` (Rust, source of truth) and mirrored in
`python/par6/protocol` (generated constants + hand-written zero-alloc decode).
Semantics inherit from the parol6 protocol (see `Jepson2k/PAROL6-python-API`,
`parol6/protocol/wire.py` + `ack_policy.py`); v2 changes are listed at the end.
License note: this repo is MIT while parol6 is GPL-3.0 — carry over parol6 code only
where you hold the authorship (self-relicensing); otherwise reimplement from this spec.

## Topology

```
client ──UDP unicast :6001──▶ runtime   (msgpack command envelopes; replies to src addr)
       ◀─UDP multicast group:port @status-rate── binary STATUS broadcast
       ◀─UDP telemetry (recipe-selected fields, higher rate, optional)
```

Status transport ladder: multicast with a loopback/primary-iface reachability probe at
startup; permanent fallback to unicast on probe failure or 3 consecutive send errors.
Client subscription mirrors the ladder. All knobs env-overridable (PAR6_* namespace).

## Envelope

msgpack arrays, integer tag in slot 0 (no maps on the wire):

- Command (C→S): `[cmd_tag, req_id, ...params]`
- Reply OK: `[OK, req_id]` or `[OK, req_id, index]`
- Reply ERROR: `[ERROR, req_id, [command_index, code, title, cause, effect, remedy]]`
- Reply RESPONSE: `[RESPONSE, req_id, [query_tag, ...fields]]`
- Push COMPLETE (unsolicited): `[COMPLETE, 0, index, ok_flag, detail?]`
- STATUS (broadcast): positional array with fixed header (below)

`req_id`: u32, client-generated, echoed verbatim in every direct reply — the correlation
fix (parol6 had none; replies were matched by "next response wins" under a client mutex).
Push messages use req_id 0.

## Command classes (ack taxonomy — one table, both sides consult it)

- **SYSTEM** — always acked OK/ERROR: reset, estop, stop, write_io, simulator,
  select_profile, reset_state, connect_hardware, set_tcp_offset, set_shapes,
  set_completion_policy.
- **QUERY** — RESPONSE, never OK: ping, status, angles, pose, io, speeds, tools, queue,
  activity, loop_stats, profile, reachable, error, tcp_speed, tcp_offset, tool_status,
  is_simulator, shapes.
- **FIRE_AND_FORGET** — success is never acked: servo_j, servo_j_pose, servo_l, jog_j,
  jog_l, teleport, reset_loop_stats. A REJECTION (gating or validation) is answered with
  a real ERROR (echoed `req_id`) — and, because no caller awaits that datagram, the
  runtime also latches the refusal as the standing error while its pipeline is idle
  (nothing executing/pending/streaming, no attributed or RT-latched error standing), so
  it reaches STATUS and the ERROR query. The next ACCEPTED motion command clears it,
  like every standing error. A refusal arriving over live motion answers ERROR only —
  it must not fail the running program's completion waits through the stale-error rule.
- **QUEUED** — ack carries the command index; a COMPLETE push follows when it finishes:
  home, move_j, move_j_pose, move_l, move_c, move_s, move_p, select_tool, delay,
  checkpoint, tool_action.

Fixes vs parol6: `reset_loop_stats` is fire-and-forget in BOTH the table and dispatch
(parol6's handler acked anyway — orphan datagram); `set_tcp_offset` is SYSTEM in both
(parol6 routed it through the planner and acked with an index).

## Command semantics

- Modeless: no client-visible enter-jog/enter-exec. The runtime derives internal mode
  from traffic exactly like parol6 (a jog cancels planned motion; a move cancels
  streaming; system commands always apply).
- Units at the wire: mm and degrees (waldoctl convention); the runtime converts to SI
  internally. Cartesian frames: WRF | TRF.
- **TCP pose rotation: intrinsic XYZ, `R = Rx(rx)·Ry(ry)·Rz(rz)`.** Binding wherever the
  six numbers `[x, y, z, rx, ry, rz]` appear — move_l / move_j_pose / move_c / move_s /
  move_p / servo_l / servo_j_pose targets, the POSE query, and any decomposition of the
  STATUS pose matrix. Same convention as `pinokin.se3_from_rpy` / `so3_rpy` (scipy's
  `Rotation.from_euler('XYZ')`), which is what waldoctl's `Robot.fk`/`ik` contract, parol6
  and the frontend's readout all use. It is NOT the URDF `rpy` attribute's fixed-axis order
  `Rz·Ry·Rx`: the two readings of the same three numbers agree only when at most one of
  them is non-zero, and the everyday tool-down pose `[…, 180, 0, rz]` read the wrong way
  round comes back with `rz` negated — a taught pose and its replay `2·rz` apart.
  Collision geometry is the one exception: `set_shapes` poses carry waldoctl's
  `Shape.pose` contract, which is extrinsic XYZ (`R = Rz·Ry·Rx`) so that what the frontend
  draws is what the checker enforces. Two contracts, deliberately different; neither
  convention travels into the other's messages.
- Jog carries a DURATION (self-terminating watchdog); UIs stream fresh jogs at ~20–50 Hz.
  It is bounded (60 s) — an unbounded duration is not a watchdog, and one datagram must
  never be able to jog until a soft limit stops it. Every other duration is bounded too
  (1 h): they all become `Duration`/`Instant` arithmetic, which panics near f64's range.
- Streaming preemption: an incoming streamable of the SAME type updates the active
  command in place (no new index); a DIFFERENT type cancels the active streamable and
  starts fresh; a planned move cancels any streamable.
- Preemption drains **only the preempted stream's own backlog**. Its setpoints are stale
  by construction — the client has replaced them — but the command socket is shared by
  every client and every command class, so anything else already queued behind them
  (SYSTEM `estop`/`stop`/`reset_state`, queries, queued moves, chunks, another client's
  stream) is dispatched normally, in arrival order. A blind drain of the socket is a
  silent `estop` loss: it gets no reply, has no effect, and the client's SYSTEM send does
  not retry.
- Queued commands carry an **idempotency key** (client uuid64). The runtime keeps a
  dedup window (last N keys → index); a retried enqueue re-acks the ORIGINAL index
  instead of double-queueing. This makes at-most-once → effectively-once under retry.
  (parol6: a lost ack was indistinguishable from a lost command; no safe retry.)
- One index allocator (the queue), monotonic, **never reset** — even by reset_state —
  so a stale pre-reset status frame can never satisfy a post-reset wait.
- stop/estop carry explicit cancel-scope semantics: `stop {clear_queue: true}` halts
  motion, clears queue, stays ENABLED; `estop` additionally latches DISABLED until
  `reset`. `reset_state` = full controller state reset (world, tool, errors) + re-sync.
- Errors: KUKA-style catalog — numeric code (subsystem ranges), title/cause/effect/
  remedy formatted server-side, `command_index` attribution (−1 = unattributable).
  Acceptance of a new command clears a standing error (prevents stale-error poisoning
  of later waits).

## Completion

- QUEUED ack = validated + queued (index N). COMPLETE push = finished (ok or error).
- Client `wait_command(N)` = satisfied by COMPLETE push, with the status stream as
  fallback: `completed_index >= N` (high-water mark — blended-away commands report the
  max of consumed indexes), or a blocking error whose ordering proves it postdates N's
  acceptance (parol6's stale-error rule: error fails your wait only if
  `error.command_index <= N` AND `accepted_index >= N`).
- Controller-side completion policies: commanded | settled (default) | strict (see RT.md).

## STATUS packet (binary, positional, header-first)

Header (new in v2): `[STATUS_tag, proto_version u8, controller_id u32, seq u64,
mono_time_ns u64, link_ok u8, data_age_ms u16]` — always broadcast, even when the bus
link is down (parol6 went silent when stale; clients couldn't tell dead from quiet).

`link_ok` / `data_age_ms` measure the **motor bus**, not the runtime's internal snapshot
plumbing: `data_age_ms` is the age of the freshest node frame the RT has seen (plus the
snapshot's own age, so a dead RT thread degrades it too), saturating at `0xFFFF` when no
node has ever answered — the bus analogue of parol6's `first_frame_received`. `link_ok`
is `data_age_ms` within the configured staleness window. The PING query's
`hardware_connected` is `link_ok AND NOT simulator`. The RT publishing its snapshot
every tick must never read as a healthy link over a silent bus.

Body (parol6 field set, kept): pose (f64[16] row-major 4×4, mm — decompose to rpy with
the intrinsic-XYZ convention above), angles f64[N] deg,
speeds f64[N] rad/s, io u8[5] [in1,in2,out1,out2,estop], action_current str,
action_state u8 {IDLE,EXECUTING,ERROR}, joint_en u8[12], cart_en_wrf u8[12],
cart_en_trf u8[12], executing_index i64, completed_index i64, last_checkpoint str,
error (null | 6-tuple), queued_segments u32, queued_duration f64, action_params str,
tool_status (null | [key, state, engaged, part_detected, fault_code, positions[],
channels[], variant_key]), tcp_speed f64 mm/s, simulator_active bool,
collision_active bool, collision_pairs [[str,str]], scene_epoch u64,
accepted_index i64, homed bool.

(v2 also fixes parol6's missing `variant_key` on the wire.)

Decode contract (client): preallocated buffer, slice-assign into numpy arrays, tail
fields length-guarded for forward compat, swallow-and-False on malformed. Same
zero-alloc pattern as parol6's `decode_status_bin_into`.

## Telemetry (separate from STATUS)

Recipe-selected field streams at up to the tick rate (RCB-Runtime's good idea, our
encoding): recipes named in config (minimal/standard/commanded/diagnostics/full),
selected at runtime (`set_recipe`); fields from the RT snapshot (measured/commanded/
target/filtered joints, torques, motor temps/voltages, external wrench, timing).
Binary msgpack, not CSV; carries seq + timestamp. Unknown recipe names are REFUSED
(silent fallback looks like a dead robot).

## Bulk payloads

Commands whose params can exceed one datagram (move_s/move_p waypoint lists,
set_shapes) use chunked envelopes: `[CHUNK, req_id, transfer_id u32, i u16, n u16,
bytes]` reassembled server-side with a per-transfer timeout. Server RX buffers are
MTU-sized (parol6 truncated silently at 1024 bytes → undiagnosable decode errors).

## Simulator & teleport

`simulator(bool)` switches the bus backend live (state re-seeded); `is_simulator()`
query. `teleport(angles_deg, tool_positions?)` is fire-and-forget, streamable-class
(preempts streams); **rejected with a real error outside sim mode** (parol6 silently
no-opped), and the refusal latches as the standing error like every rejected
fire-and-forget (see the ack taxonomy above). Sim runs on fixed dt, never wall clock
(deterministic recordings).

## Collision enforcement

Two shape layers make up the collision world, both enforced identically and read back
by the SHAPES query:

- **installation** — persistent keep-outs from the runtime's own configuration
  (`[[installation_shapes]]` in the robot TOML: cage walls, the table, fixtures),
  applied once at startup. A malformed entry refuses BOOT through the same validation
  a `set_shapes` runs. Nothing on the wire changes this layer — `set_shapes` and
  `reset_state` replace the program layer only. parol6 keeps them in robot config
  with the same rule.
- **program** — the last applied `set_shapes` set (last-write-wins). A set with any
  malformed shape is refused WHOLE: the previously applied world stays enforced and
  `scene_epoch` does not move.

Enforcement covers every way the arm moves:

- **Planned motion** — the trajectory's own samples are walked before anything
  reaches the RT ring; a colliding sample refuses the command with
  `SYS_SELF_COLLISION` (pairs named in the payload). A world change re-gates the
  remainder of the motion in flight.
- **Streaming motion (jog_j / jog_l / servo_\*)** — parol6 gates these inside its
  server-side integrator; par6 integrates the ramp on the RT thread, where a coal
  check cannot run, so the gate sits on what CAN see the stream: each accepted
  datagram (admission) and a periodic re-check while the stream is live. Jogs are
  tested at a velocity-scaled lookahead — the configuration 0.15 s ahead at the
  COMMANDED velocity, an upper bound on the RT ramp (parol6's
  `COLLISION_JOG_LOOKAHEAD_S`) — so faster jogs stop further from contact; servo
  targets are explicit configurations, tested as such on every datagram. A blocked
  admission answers ERROR (`SYS_SELF_COLLISION`) and latches like any refused
  fire-and-forget; a block detected mid-stream stops the stream (RT back to IDLE)
  and latches `collision_active` / `collision_pairs` in STATUS.
- **Start-in-collision escape rule** (both gates): a motion that BEGINS in collision
  — the park pose's own resting contacts excepted — is permitted only while it adds
  no new colliding pair AND goes no deeper (the minimum signed distance may not drop
  by more than a 0.1 mm tolerance). Escaping a keep-out dropped over the arm stays
  possible; grinding deeper through the same pair is refused. Same rule as parol6's
  `collision_blocked` / `guard_joint_path`.

## v2 change log vs parol6 (rationale in the wart audit)

1. req_id correlation (was: response-order matching under a client lock)
2. idempotency keys + dedup for queued commands (was: unsafe retry)
3. status header: version/controller_id/seq/timestamp/link_ok/data_age (was: none)
4. always-broadcast status (was: silent when stale)
5. single unspecified-value convention (`nil`), no 0.0-sentinels
6. one index allocator (was: two call sites)
7. explicit stop cancel-scope (was: isinstance special cases)
8. chunked bulk payloads + MTU-sized RX (was: 1024-byte silent truncation)
9. codec free of process-global registry lookups (tool validation in the server layer)
10. tool_status.variant_key on the wire
11. teleport-outside-sim is an error; reset_loop_stats truly unacked; set_tcp_offset
    truly SYSTEM
12. COMPLETE push per queued command (was: poll completed_index only)

## Frozen decisions (v2.0, contract freeze 1)

Resolved during implementation of `par6-proto`; the crate is authoritative.

1. Tag ranges, class-grouped: MsgType 1–6; CmdType SYSTEM 10–21, QUERY 30–47,
   FIRE_AND_FORGET 60–66, QUEUED 80–90.
2. `set_recipe` is SYSTEM (unknown-recipe refusal needs an ack path).
3. `Frame` is u8 on the wire (WRF=0, TRF=1) — no strings in the envelope.
4. Idempotency key is wire slot 2: `[tag, req_id, key, ...params]` (QUEUED only).
5. COMPLETE: `[COMPLETE, 0, index, ok, detail?]`; detail = error 6-tuple, present
   when ok=false.
6. The nil convention covers duration/speed/accel, blend radius `r` (nil = no
   blend), `select_tool.variant_key`, `teleport.tool_positions`, POSE frame.
   Exactly-one-of duration/speed retained for planned moves.
7. STATUS decode requires the full 31 v2 elements; longer tails tolerated,
   shorter rejected (no legacy producers exist).
8. REACHABLE answers with the enablement triple (parol6's separate ENABLEMENT
   query merged away).
9. Activity `state` is the integer ActionState, not a name string.
10. Error catalog: parol6 codes 10–43 kept; SYS range renumbered and extended
    (MOTN_SETTLE_TIMEOUT=36, COMM_CHUNK_TIMEOUT=44, COMM_UNKNOWN_RECIPE=45,
    PROFILE_INVALID=52, SELF_COLLISION=53, NOT_SIMULATOR=54, EXEC_LINK_LOST=55,
    RTI_LINK_LOST=56, LOOP_CRITICAL=57, JOINT_FAULT=58; PORT_SAVE_FAILED dropped).
11. A CHUNK payload is the complete inner command datagram, byte-identical to
    the unchunked encoding.
12. `tool_action.params` ≤ 16 scalars (float/int/bool/str).
13. The codec validates shape/ranges only; joint limits, tool names, and recipe
    names are server-layer checks against config.
