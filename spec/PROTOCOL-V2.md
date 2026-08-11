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
- **FIRE_AND_FORGET** — no reply: servo_j, servo_j_pose, servo_l, jog_j, jog_l, teleport,
  reset_loop_stats.
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
- Jog carries a DURATION (self-terminating watchdog); UIs stream fresh jogs at ~20–50 Hz.
- Streaming preemption: an incoming streamable of the SAME type updates the active
  command in place (no new index); a DIFFERENT type cancels the active streamable,
  drains the socket backlog, and starts fresh; a planned move cancels any streamable.
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

Body (parol6 field set, kept): pose (f64[16] row-major 4×4, mm), angles f64[N] deg,
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
no-opped). Sim runs on fixed dt, never wall clock (deterministic recordings).

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
