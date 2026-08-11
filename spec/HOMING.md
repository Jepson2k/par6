# HOMING.md — homing FSM and sequences

Behavioral spec for the homing portion of `par6-rt`, extracted from vendor
`robotics/homing.py` + `config/PAR6_homing.xml` + robot/gripper XML. **Spec-only —
port behavior and constants, never code (GPL).**

Homing runs as mode HOMING (SELF_MANAGED: it issues its own per-joint frames; the
normal mode dispatch sends nothing). Two levels: a sequence orchestrator stepping a
parsed sequence file, and a per-joint FSM. Per-joint status: 0 idle, 1 running,
2 done, 3 failed (3 ⇒ warning key `J{i}:HOMING_FAILED`; sequence failure clears
homed state and returns to IDLE).

## Sequence structure

Parallel step-based sequences (per-robot config; PAR6's shipped sequence):

1. home J0 — with 4 s idle hold on J1/J2 and a −7000-tick nudge on J4
2. home J1 + J2 **in parallel**
3. nudge J4 back; gripper firmware calibrate; gripper motor home; gripper_move to 252
4. nudge J5; smooth move_to J1/J2
5. home J3 + J5 in parallel; move_to J5 → π
6. home J4; final move_to J0/J3/J4 ready pose

Step elements: `pre_moves` (position | nudge | idle | gripper_move), `home joints="…"`
(comma list; `gripper` allowed with mode motor|firmware), `move_to` (cubic Hermite),
trailing global `post_moves`. Pre/post/move_to timeouts **warn and continue**
(4 s pre/post; move_to = duration + 2 s); home-phase timeouts FAIL.

**Bus liveness during homing:** every non-active joint receives an idle frame every tick
(arm: cmd 2 with vel 0/cur 0; gripper: replay of last gripper_move or the DLC-0 empty
poll) — otherwise the freshness detector latches CAN_LOST on idle joints.

## Per-joint FSM

Phases: homing → [pre_clear] → dwell → backoff → pause → homing(pass 2) → [release] →
settle → [post_move] → done | failed.

**Detection — two strategies:**

- *Hall joints* (`has_hall_sensor=1`): drive with the HALL pack (cmd 31, speed +
  trigger_value 2); hit when HALL_trigger==0 or edge bit set; position latched AT
  trigger. Hall joints skip two-pass (the digital edge is the reference). Pre-clear
  guard: a trigger within 0.5 s of start means "started on the sensor" → back off
  `backoff_s`, reset, re-approach.
- *Stall joints*: drive velocity-mode (cmd 2 DLC 5, signed homing speed, cur 0).
  Two conditions gated together (current primary, stall secondary):
  - windowed stall: displacement from a reference stays below
    `max(10, |speed|·0.08·0.25)` ticks; window resets on movement; stalled at
    `round(0.08/dt)` consecutive ticks (min 5);
  - current ratio: after a 0.15 s startup guard, ticks with current above
    `0.70 · homing_current_ma`; fires at `round(0.08/dt)` ticks (min 2) with ≥60%
    of the window above threshold.

**Two-pass** (default on): pass-1 hit → save pos → dwell 0.08 s stopped → back off
`backoff_s` → pause 0.15 s → pass 2 at `rehome_speed_factor × speed` (default 0.3).
At settle, `|pass2 − pass1| > two_pass_max_diff` ticks ⇒ FAIL.

**Release phase** (stall joints; skipped when release duration is NaN): command
current-only (cmd 2 DLC 2, `release_current_ma`, sign matters — e.g. J1 +150 mA,
J2 −150 mA; 0 = coast) for `release_duration_s`, latching the encoder position at
`round(release_ticks · release_sample_pct)` — relieves gearbox preload so the latched
position is the true resting endstop.

**Settle** (0.08 s, vel 0/cur 0): latch position if not already (retry up to 2× settle
ticks while the motor position is unknown — **if it never becomes valid the vendor
marks DONE without setting the reference; we make that a FAILURE**), run the two-pass
check, apply the home reference, mark done, restore normal limits.

**Post-move** (optional per joint): position-mode toward `post_home_position` until
within 50 ticks for `round(0.08/dt)` consecutive ticks; timeout warns and continues.

## Home reference & gripper-dependent offsets

Applying the reference: `master_position = latched ticks`, `offset = home_offset rad`,
`offset_ticks = radians_to_ticks(offset if dir==0 else 2π−offset)`. Post-condition:
measured joint position at the endstop equals `home_offset`.

**Gripper-dependent offsets:** each joint has a fallback `home_offset` and a
`home_offset_gripper_dependent` flag; the ACTIVE gripper's config may override the
offset for flagged joints. PAR6 per the vendor XML (verified against
`robots/PAR6.xml` + gripper files): index 3 is gripper-dependent with fallback
−2.717 rad and no gripper overrides it (fallback applies); index 4 is
gripper-dependent with fallback 0.0 and every gripper overrides it
(MSG-small-150 −2.070, SSG48 −2.120, Flange −2.258). Swapping grippers changes an
ARM joint's home reference — config must make this dependency explicit.

## Current limits around homing

On entry: per-node config reload with `current_limit = homing_current_ma`, watchdog
5000 ms → Idle, ×4 repeats; per joint at FSM start: Limits(normal_vel, homing_current)
×4 (only path that applies it to the gripper motor). On joint completion: restore
normal Ilim ×4. On exit: full normal config reload. Publish the EFFECTIVE per-joint
current limit every tick (homing value while status<2).

## Gripper homing

Firmware calibrate: cmd 62 once, then DLC-0 empty polls every tick; timeout 10 s
(min wait 2 s). Motor-mode homing uses the stall FSM. On completion derive
`ticks_per_meter = 2^14/(4π·Gear_r)` (fallback: |endstop_ticks|/stroke_m — only valid
if fully closed at power-on) and latch `endstop_ticks`.

## Failure & abort

| Condition | Result |
|---|---|
| approach exceeds `homing_timeout_s` | joint FAIL → sequence failed |
| two-pass diff > `two_pass_max_diff` | FAIL at settle, limits restored |
| position never valid at settle | **FAIL** (vendor: silent-uncalibrated hazard — fixed) |
| gripper calibrate timeout | failed |
| pre/post/move_to timeout | warn + continue |
| any hard error during HOMING | abort, clear homed, zero statuses, restore config |

Defaults when a field is missing: offset π/2, speed 5000 ticks/s, direction 1,
current 600 mA, hall 0, two-pass on, timeout 6 s, backoff 0.3 s, rehome 0.3,
max_diff 200. PAR6 actuals (robot config): J0 4500/dir 0/700 mA/13 s/diff 3500;
J1 6000/dir 1/250 mA/backoff 1.5/diff 2000/release +150 mA 1 s @80%;
J2 6000/dir 0/250 mA/release −150 mA; J3 13500/dir 0/1200 mA;
gripper 7000/dir 0/700 mA/8 s/backoff 0.8.

## Sim requirements (for CI-testable homing)

The closed-loop sim bus must produce: stall current growth against endstop torque
(for the current-ratio window), displacement plateau (for the stall window), hall
edges at configured positions for hall joints, and preload relaxation during the
release phase — see `par6-bus` sim backend.
