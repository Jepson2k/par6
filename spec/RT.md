# RT.md — real-time core behavior (tick, modes, errors, e-stop)

Behavioral spec for `par6-rt`, extracted from vendor `RTI.py` / `utility/` / `motion/`.
References point into `Source-Robotics/RCB-Runtime` for verification. **Spec-only —
port behavior and constants, never code (GPL).** Items marked **[OURS]** are deliberate
deviations from the vendor design.

## Rate & timing

- Tick period from robot config (`PAR6: 0.004 s = 250 Hz`). **Every time constant is
  seconds in config, converted `round(s/dt)` at construction — never hardcoded ticks.**
- Vendor: relative `sleep(dt - exec_time)` → loop can only run SLOW, hence one-sided
  degradation bands. **[OURS]** absolute deadlines (`clock_nanosleep TIMER_ABSTIME`);
  re-derive bands from measured jitter, starting from vendor factors.
- Degradation: fed by rolling p99 of loop period (500-sample ring, recompute every 50
  ticks, warmup ~850 ticks). `p99 > 1.05·dt` → `LOOP_DEGRADED` (warning, self-clears);
  `p99 > 1.10·dt` sustained 1.0 s → `LOOP_CRITICAL` (hard latch → DISABLED+ACTIVE_ERROR).
  **[OURS]** those three numbers are the DEFAULTS of config `[timing]`
  (`degraded_factor`/`critical_factor`/`critical_sustain_s`), not constants — hardware
  runs the vendor values, and `par6d --sim` widens them (a wall-clock simulator on a
  shared host cannot hold the deadline). Ring size, recompute interval and warmup stay
  constants; note a sustain shorter than the recompute interval latches on the first
  bad percentile.
- Vendor scheduling: SCHED_FIFO prio 99, pinned core 3; setup failure is logged
  DEGRADED but non-fatal. Keep that stance.

## Tick phase order

Vendor order: GPIO read → timing stats → (boot one-shots) → one queued command →
RTI-UDP drain → mode transition → **mode dispatch (TX)** → gripper send → round-robin
poll → **CAN RX drain + state pipeline** → freshness → reconnect → shared-state publish →
history append → error checks → telemetry → GPIO write → timing finalize → sleep.

⚠️ Vendor sends commands BEFORE reading measurements: every handler consumes state from
the PREVIOUS tick (1 full tick of structural latency) — a Python-speed artifact.
**[OURS]** RX-drain → state update → compute → TX within one tick. Flagged for HIL
validation; a config flag restores vendor ordering if motor tuning misbehaves.
The state pipeline order (motor arrays → joint/TCP derivation → history append for
finite differences) is load-bearing either way.

- At most ONE external command consumed per tick.
- Boot one-shots: tick 8 = bus scan + selfcheck then request IDLE (exit BOOTING);
  vendor re-sends full config at ticks 50/150/300 (workaround — we may drop after HIL).

## State machine

Four independent variables: `mode`, `state` (ENABLED/DISABLED), `homed`, `errors` (list).

Modes: BOOTING, IDLE, ACTIVE_ERROR, HOMING, JOG, RTI(streaming), EXEC, HAND_GUIDING,
IMPEDANCE, SAFETY_STOP, FLASHING. Transitions: BOOTING→{IDLE,SAFETY_STOP};
IDLE→{everything}; ACTIVE_ERROR→{IDLE,FLASHING,SAFETY_STOP}; working modes→{IDLE,
SAFETY_STOP}; SAFETY_STOP→IDLE; `→IDLE` always allowed from anywhere.
Gates in order: (1) MAINTENANCE_MODES={FLASHING} exempt from enabled/errors/homed —
gated ONLY on a human park assertion (PARKED/FORCE, logged; a torque-threshold gate was
tried and removed — false positives); (2) SAFETY_STOP always reachable, no checks;
(3) others require enabled ∧ no-errors ∧ (homed if in {JOG,RTI,EXEC,HAND_GUIDING,IMPEDANCE}).
Enable refused while safety violation or errors active.

## Per-mode output law (the whole control language)

Output = (pos?, vel?, trq?, pack) — `Option` channels per CAN.md; pack = pid(cmd 2) |
pd(cmd 4). Modes compute setpoints; dispatch owns commit → FK of commanded → motor-space
conversion → record → single send per joint per tick.

| Mode | pos | vel | trq | note |
|---|---|---|---|---|
| BOOTING | — | 0 | 0 | |
| IDLE (homed∧enabled∧grav-on) | — | — | G(q) | **torque-only hold — no position hold** |
| IDLE otherwise | — | 0 | 0 | |
| ACTIVE_ERROR | — | 0 | 0 | **active zero-velocity hold** (e-stop lands here) |
| SAFETY_STOP | — | — | 0 Nm | fully limp |
| JOG | integrated target | ramped | G(q) if grav-on | pid or pd (config) |
| EXEC | sample | sample | plan FF + G(q) | accel by finite diff for telemetry |
| RTI stream | limited | limited/ff | limited + G(q) | see Streaming |
| HOMING / FLASHING | SELF_MANAGED | | | homing sends per-joint; flashing sends nothing |

Gravity: G(q) only — RNEA at zero vel/accel over the arm + active gripper tool link
(masses/COM/inertia from config). Computed every tick, published always; applied as
current feedforward (`mA = Nm · sign·1000/(gear·eff·kt)`, factor precomputed once) only
when homed ∧ enabled ∧ mode allows ∧ comp enabled. External estimation downstream:
`τ_ext = τ_filtered - G(q)`, `F_ext = solve(Jᵀ, τ_ext)` (pinv near singularity).

## Jog

Per joint, per tick: target velocity = dir · vmax · pct (0 if blocked); jerk-aware
lookahead — stopping distance (trap: v²/2a; s-curve: v²/2a + v·a/2j) × 1.5 safety factor
vs remaining travel to soft limit → latch a DIRECTION block (survives button release;
clears only on opposite direction or joint switch); ramp (trapezoid Δv≤a·dt, or s-curve
with jerk-limited accel tracking); hard clamp if measured pos past soft limit moving
outward; integrate target position. Defaults: speed 20%, accel_time 0.55 s (floor 0.05),
profile s-curve, jerk_factor 3.0 (floor 0.5), pid.

## EXEC (planner → RT handoff)

Vendor: chunked binary batches (magic `RCBX` v1): header (N, J, checkpoint id, flags
bit0=BLEND_CONTINUES, is_last_chunk) + f64 positions + f64 velocities + f32 torque_ff,
chunk cap 2000 points; prefetch by SAMPLES (target 750 = 3 s @4 ms buffer, ≤8 pulls/tick).
**[OURS]** in-process SPSC sample ring with the same semantics (checkpoint boundaries,
blend-continues flag, samples_remaining published as planner deadline signal).
Pause = hold in place (batch untouched); hold = last target + vel 0 + G(q).

Completion policies (controller-side): `commanded` = complete at last sample; `settled`
(default) = hold until max |q_meas − q_target| ≤ 0.01 rad or 500-tick timeout then
complete; `strict` = same but timeout is an ERROR. BLEND_CONTINUES bypasses settling so
blended corners stay velocity-continuous. Exec link watchdog: heartbeat @50 Hz from the
command plane; 0.5 s of silence while samples pending → `EXEC_LINK_LOST` (hard latch).

## Streaming (RTI-mode equivalent)

Session lifecycle separate from mode: connect(pc addr) / claim / release / disconnect;
substates unpaired/connected/control_active/stopping_clean/stopping_error.
Per tick: state TX FIRST (maximize PC reply window), then apply newest RX only (count
discards → discard %), then: **low-pass BEFORE rate limit** (never filter feedforward);
position targets through a jerk-limited per-tick OTG limiter (Ruckig, ~3.5 µs/6 DoF —
hard dependency; vendor's fallback cascade limit-cycles ±0.6°); unconditional soft-limit
clamp on the way in AND out (survives rate_limit_enabled=false); velocity path: v/a/j
cascade then jerk-aware brake-at-limits bound `v_stop(d)=a(√((a/j)²+2d/a)−a/j)`,
direction-aware (motion away from a limit always allowed); torque path: rate-limit only +
zero outward component past limits; **gravity added AFTER the limiter** (never throttle
the robot's own weight compensation). Start-pose check on claim (default ON): pos tol
0.1 rad; vel ≤ 10% limit; torque ≤ rate·dt/0.25. Fail → stopping_error + link-lost.
Watchdog 40 ms (10 ticks @250 Hz); stopping hold 0.5 s; success-rate window 100 ticks
(warn <0.95, bad <0.90); RFC-1982 sequence compare (u32 wrap-safe). `target_*` fields
carry the raw request, `commanded_*` the post-limiter values — the difference makes
limiter activity visible in telemetry.

## Errors

Keys: bare (`ESTOP, SW_ESTOP, EXEC_LINK_LOST, RTI_LINK_LOST, LOOP_DEGRADED,
LOOP_CRITICAL, NOT_HOMED, GRIPPER_*`) or per-joint `J{i}:KEY` (TEMPERATURE, ENCODER,
VBUS, DRIVER, VELOCITY, CURRENT, ESTOP_MOTOR, WATCHDOG, CAN_LOST, CAN_STALE,
HOMING_FAILED). Warnings (self-clear, don't set error_active): CAN_STALE, HOMING_FAILED,
NOT_HOMED, LOOP_DEGRADED. Everything else LATCHES until user clear.

- **Live-bit gating**: per-type motor flags (from the ~84 ms round-robin poll) are only
  trusted while that node's live fault bit (CAN-id err_bit) is set — fixes the
  "clear needs two presses" race. Unknown (-1) does NOT suppress.
- Clear sequence: cmd 1 ×3 to each faulted node (+ gripper), zero stale entries, settle
  countdown ~152 ms (sized to outlast the poll cycle), then wipe latch; anything real
  re-latches next poll.
- Reaction: any hard error (mode ∉ {BOOTING}) → state=DISABLED; if mode ∉
  {ACTIVE_ERROR, FLASHING} → mode=ACTIVE_ERROR (if HOMING: abort homing, un-home).
  No hard errors ∧ mode==ACTIVE_ERROR → auto-return to IDLE.
- CAN disconnect or gripper disconnect while homed ⇒ **homing invalidated**.
- Publish error list on edge changes only.

## E-stop

`estop = (debounced ESTOP_1 == 0) OR software_estop_flag` — ESTOP_2 excluded (known
hardware fault: always reads triggered). GPIO debounce = 5 consecutive identical reads
with FIRST-READ SEEDING (without it, zero-initialized state reads "pressed" at boot and
latches a false e-stop). Distinct keys ESTOP vs SW_ESTOP; both hard-latch.
Reaction = DISABLED + ACTIVE_ERROR (active zero-velocity hold). **Motors stay energized;
no CAN ESTOP frame is sent** (cmd 0 exists but is never used) — the driver watchdog
(5000 ms → Idle) is the independent hardware backstop, and the physical e-stop chain cuts
power outside software's view. Replicate exactly; do not "improve" by sending cmd 0 —
that changes recovery semantics. Vendor latency budget @250 Hz ≈ 24 ms to first
zero-velocity frame (20 ms debounce + 4 ms phase alignment).

Control-PCB UART (supervisor plane, NOT the RT thread): /dev/ttyAMA0 @256000, RX frames
`$<i32> <i32> <i32> <i32>\n` (opaque register block), TX `heartbeat\n\r` @~1 Hz, polled
50 Hz, open-failure non-fatal.

## Status snapshot (RT → command plane)

Single-writer shared state; **[OURS]** seqlock/triple-buffer instead of vendor's
lock-free-by-convention POSIX shm. Contents: measured joint/TCP state (+filtered,
commanded, target variants), gravity torque, motor telemetry (temp/voltage/errors),
homing status, error latch, loop timing stats (EMA + p50/p90/p99/max + overruns +
CAN frame age max/min), mode/state/homed, jog/exec/stream live state.
Vendor invariant to keep: measured-state has exactly ONE writer (the RT thread).

## Implementation findings (P1.D)

- **Jog s-curve lookahead must include current-acceleration reversal terms**:
  stopping distance `v²/2a + v·a₀/j + a₀³/3j²` with peak velocity `v + a₀²/2j`
  (reduces to the vendor's `v²/2a + v·a/2j` at a₀=0). Without them a limit trip
  firing mid-ramp overshoots the soft limit by >1 rad at PAR6 J0 numbers.
- `Sample`/`SampleMeta` are mirrored in par6-motion (par6-rt depends on
  par6-motion, so the ring types can't be imported without a cycle); a dev-dep
  conformance test pins the mirror field-for-field. Candidate cleanup: move the
  sample types to a leaf crate.
- Planned-move `tau_ff` is zero until inertial feedforward lands with dynamics;
  gravity remains RT-side by contract.
