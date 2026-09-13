# Measured calibration

Two tools, for two different jobs.

`par6-selfcal` is the one to run on a new arm: a single binary that homes it,
finds the seek currents and velocity gains it needs, measures its gravity
feedforward, and proves the result. It drives the CAN bus itself, so `par6d`
must not be running. Two minutes.

`par6.calibration` (`par6-calibrate limits`) is the one-off velocity and
acceleration sweep, which runs against a live `par6d`.

## par6-selfcal

```sh
par6-selfcal --help
par6-selfcal                       # read config/PAR6.toml, measure, verify
par6-selfcal --apply               # and write what it measured into the config
par6-selfcal --home-only           # homing alone
par6-selfcal --home-only --repeat 4  # home five times, report the spread
par6-selfcal --sim                 # no arm attached
```

It refuses to start while `par6d` is running: two processes commanding the same
drives means the arm obeys whichever frame arrived last.

Needs `CAP_SYS_NICE` or root: it paces the drives at the config's tick rate
under SCHED_FIFO, and a slipped command is a disturbance the drives feel.

**Step 1 — home.** Runs the config's own homing sequence. A joint that will not
reach its endstop is diagnosed rather than retried blindly: a drive sitting on
its approach current limit is short of current, so the seek current goes up
toward that joint's `ilim_ma`; a joint that simply will not move gets more
velocity gain. A stall only counts as an endstop if the joint reached half its
commanded speed first — a loaded joint that creeps the whole way looks
identical to one arriving at a stop, and the shoulder was once referenced
0.66° into a 38° seek that way. Stall references are confirmed by a second
approach agreeing within the config's tolerance.

**Step 2 — gravity, distal first.** Each loaded joint is taken to the pose
where it carries the most, and held **torque only** on the model's G(q) times a
scale — the runtime's own `law_idle`. Torque-only is the point: on a position
hold, these drives' integrating loops supply whatever the feedforward does not,
so the joint holds at any scale and nothing about gravity is measured. The
scale is moved by a secant on the measured drift rate, and a joint whose drift
does not improve as its feedforward grows is reported as a feedback problem
rather than walked out of range.

**Step 3 — verification.** Every loaded joint goes back to its worst pose and
must hold on the scale that was chosen. A joint that misses is measured again
from that number and re-verified, up to three rounds. Nothing is written by
`--apply` unless this passes.

### What it writes

`selfcal-measurements.toml` on every exit, including a failure — a run that
measured four joints and failed on the fifth has still measured four joints.
`--apply` edits the config in place (comments preserved) and keeps a
`PAR6.toml.before-selfcal` backup. A gain only becomes a measurement once a
joint has actually reached a pose on it.

### Two rules it enforces on every tick

- **No joint moving for 1 s fails the run.** Checked on the one path every
  command goes through, not at each place that might judge its own wait
  reasonable. The only stillness left anywhere is the sub-second gravity
  measurement, the wait for the drives' first frames, and the handover.
- **Once a joint's feedforward is measured, any oscillation fails the run.**
  Every joint, every tick — a joint that rings only while a *different* joint
  is driven is invisible to a check that watches the joint under test.

### Measured on this arm

Eleven consecutive passing runs at 110–138 s; worst command tick 4095–4134 µs
against a 4000 µs target, p99 4010 µs.

`--repeat 4`, five homing runs, slowest 51.7 s, reference spread per joint:

| J1 | J2 | J3 | J4 | J5 | J6 (hall) |
|---|---|---|---|---|---|
| 0.003° | 0.039° | 0.046° | 0.044° | 0.011° | 0.055° |

This arm needed J3 `kpv`/`kiv` ×1.25 and a 1125 mA shoulder seek current. Its
gravity scales measure 1.000 except the elbow at 1.080, so the vendor's model is
essentially right for it — **provided the fitted gripper is in the chain**.
Leaving the tool out of G(q) had the model asking for +25 mA at the wrist pitch
where the drive was pulling −655 mA.

One characteristic worth knowing: the elbow's drift readings scatter about
0.01–0.05 deg/s at its loaded pose, where the wrist and wrist pitch repeat to
0.005. That is stiction in a joint carrying 2 A, and it is why the hold
tolerance sits above that scatter and why verification re-measures rather than
trusting one reading.

### Limits of the evidence

Encoders measure joint motion, not table vibration. Torque is inferred from
motor current through `kt_nm_a` and the gear ratio. A gravity scale is measured
at one pose per joint, so a single scalar cannot describe a model that is wrong
in shape rather than in size; the drift the elbow shows between runs (its
readings scatter ~0.01–0.05 deg/s) is stiction, and the tolerance is set above
that scatter rather than below it.

## par6.calibration: the limits sweep

### Prerequisites

- The arm is homed (run `par6-selfcal` first) and idle with an empty queue, the configured gripper fitted
  and no declared payload.
- `par6d` was started with `PAR6_DIAGNOSTICS=<file>.bin` (the native recorder;
  default budget one hour at 250 Hz, `PAR6_DIAGNOSTICS_MAX_SAMPLES` for more).
- On hardware, the collision scene contains the work surface (calibration
  refuses an empty obstacle scene). Home first.

```sh
PAR6_DIAGNOSTICS=$PWD/calibration-runs/capture.bin par6d            # or via Commander
par6-calibrate limits                      # ~20 min, one-off
```

Evidence lands in `calibration-runs/<time>-<routine>/`: `<routine>.json`
(report, `valid`, `reasons`), `trials.json` (every stimulus with its capture
offsets and metrics), `identity.json`, `world.json`, and on success
`profile/candidate/PAR6.toml`, `profile/rollback/PAR6.toml`,
`calibration-patch.toml` (only the keys that were measured).

### The routine

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

### Safety rules (enforced by `Session`)

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

### Limits of the evidence

`limits` certifies the RUCKIG EXEC path with the configured gripper fitted and
no payload, not a mechanical maximum. Torque is inferred from motor current
through `kt_nm_a` and the gear ratio.

## Superseded

`par6.calibration` also carries `check`, `tune-feedback` and `gravity`
routines, written before `par6-selfcal` and never completed on this arm. Use
`par6-selfcal` for homing, gains and gravity; do not run both against the same
arm in one session.
