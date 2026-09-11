# Measured calibration

`par6.calibration` tunes one PAR6 arm from its own measurements: a motion
baseline, velocity-loop gains against vibration, the gravity model, and the
joint velocity/acceleration/jerk limits. Every routine runs against the
connected `par6d`, judges the 250 Hz native recording after an acknowledged
Stop, and stages a candidate config you activate by restarting the runtime
with `PAR6_CONFIG`. Nothing is applied automatically; a rollback copy is
staged beside every candidate.

## Prerequisites

- The arm is homed and idle with an empty queue, the configured gripper fitted
  and no declared payload.
- `par6d` was started with `PAR6_DIAGNOSTICS=<file>.bin` (the native recorder;
  default budget one hour at 250 Hz, `PAR6_DIAGNOSTICS_MAX_SAMPLES` for more).
- On hardware, the collision scene contains the work surface (calibration
  refuses an empty obstacle scene). Home first.

```sh
PAR6_DIAGNOSTICS=$PWD/calibration-runs/capture.bin par6d            # or via Commander
par6-calibrate check                       # 2 min: shoulder/elbow baseline
par6-calibrate tune-feedback --joint 3     # ~6 min per joint
par6-calibrate gravity                     # ~12 min: identify, stage candidate
par6-calibrate gravity --verify            # after restarting on the candidate
par6-calibrate limits                      # ~20 min, one-off
```

Evidence lands in `calibration-runs/<time>-<routine>/`: `<routine>.json`
(report, `valid`, `reasons`), `trials.json` (every stimulus with its capture
offsets and metrics), `identity.json`, `world.json`, and on success
`profile/candidate/PAR6.toml`, `profile/rollback/PAR6.toml`,
`calibration-patch.toml` (only the keys that were measured).

## Routines

**check** — ±0.04 rad opposite-direction sweeps on J2/J3 at 0.05 rad/s.
Reports tracking peak, velocity-residual RMS, windowed vibration, command
ripple and current saturation per joint. Run it before and after any change.

**tune-feedback --joint N** — six ±0.18 rad sustained moves with a 2.5 s hold
at the baseline gains, then candidates in preference order (Kiv×0.8, Kiv×0.6,
Kpv and Kiv ×0.8, ×0.6). A candidate must pass ordinary acceptance in travel,
hold and every 1.2 s window, hold within 0.05°, respond no slower, and carry
at most 70 % of the baseline's oscillation energy; the first such candidate
must then pass three further independent trials against three further
baseline trials. Gains are pushed volatile (`set_pid_gains`) and restored on
every exit; Kpp, current-loop gains and limits are never changed.

**gravity** — six shoulder/elbow/wrist centres, ±0.18 rad sweeps on J2–J6 in
both directions at ≤0.07 rad/s with gravity feedforward off and position
feedback on. Only slow, unsaturated, low-acceleration samples are used;
Coulomb and viscous friction are projected out; groups whose approach or
sweep would collide are dropped whole (at least three must remain, and the
fit and held-out halves must each cover the observable gravity directions).
Every held-out joint must stay under 0.05 Nm RMS residual — relative
improvement alone stages nothing. The candidate writes `gravity_correction`
(24 coefficients) and resets `gravity_scale`. `--verify` runs only the
held-out groups against the loaded model by paired opposite-direction torque,
without refitting: the acceptance run after the candidate is applied.

**limits** — per joint, `move_j` through the ordinary EXEC planner (position
feedback, gravity feedforward off, as in every other routine) at speed
fractions 0.25→1.0 of the configured exec limits (acceleration at 0.5), then
acceleration fractions at the best speed, over up to ±1 rad of travel; a
step passes when the arm arrives within 0.5° (following error while moving
is a lag, bounded loosely at 3°), no current sample reaches 95 % of the
drive limit, and the windowed velocity residual stays under the larger of
0.03 rad/s, the encoder noise floor and 4 % of the commanded speed, with no
vibration above the noise floor. A
dimension is certified only where the planner actually commanded at least
90 % of the fraction asked for — a short travel that never reaches the
configured speed says nothing about it and keeps the configured value. The
largest clean certified peak times 0.8 becomes `limits.exec` (jerk = 3 ×
acceleration; `limits.stream` = exec, which is what stopped the 50 Hz J2
stream pulses). Two all-joint moves at the proposal must pass before
staging. JOG is not measured.

## Safety rules (enforced by `Session`)

Every stimulus is preflighted through the native stream limiter and the
collision world, including the gate's stopping projections. The live STATUS
stream is a safety signal only — a fault, e-stop, mode change, drive fault,
a packet older than 250 ms or half a second of silence aborts the trial —
while all measurements come from the native recording, so Python scheduling
jitter cannot invalidate evidence. Every trial ends with an acknowledged Stop
and measured encoder rest, on success, rejection, error and cancellation
alike. Whole-trial and windowed acceptance both apply and are judged after
Stop. An unconfirmed SYSTEM acknowledgement is a failure. The arm is handed
back in the support mode it arrived in.

## Limits of the evidence

Encoders measure joint motion, not table vibration. Torque comes from motor
current through `kt_nm_a` and the gear ratio; the gravity fit assumes
friction is odd in velocity, so load-dependent directional friction can be
indistinguishable from a small gravity error. `limits` certifies the RUCKIG
EXEC path with the empty configured gripper, not a mechanical maximum.
