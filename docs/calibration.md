# Measured calibration

Two tools, for two different jobs.

`par6-selfcal` is the one to run on a new arm: a single binary that homes it,
searches velocity/position gains where the vendor values fail the stated
requirements, measures a gravity holding-current factor per joint, and checks it
at an independent pose. It drives the CAN bus itself, so `par6d` must not be
running. Duration depends on the trial budget: a hardware run that tunes two
joints takes 20–30 minutes.

`par6.calibration` (`par6-calibrate limits`) is the one-off velocity and
acceleration sweep, which runs against a live `par6d`.

## par6-selfcal

```sh
par6-selfcal --help
sudo par6-selfcal --trials 12                    # config/PAR6.toml: home, tune, gravity
sudo par6-selfcal --trials 12 --apply            # and write the result into the config
sudo par6-selfcal --trials 12 --home-only        # homing and gain search only
sudo par6-selfcal --trials 12 --joint 3          # home; gain changes allowed on J3 only
sudo par6-selfcal --joint 3 --baseline           # measure J3 at its configured gains, no search
sudo par6-selfcal --trials 24 --joint 4 --from-vendor   # start J4 from the vendor gains
sudo par6-selfcal --probe-endstop 2 --probe-seconds 8   # one seek on J2, then release
sudo par6-selfcal --check-runtime                # CPU/scheduling check, no motor commands
par6-selfcal --check-gravity                     # print the gravity poses, no motor commands
par6-selfcal --replay calibration-runs/<run>     # re-score a recorded run, no CAN
par6-selfcal --sim --trials 12                   # no arm attached
sudo par6-selfcal --trials 12 --output-dir /path/to/calibration-runs
```

`--trials N` (N ≥ 3) is the budget of gain experiments per search stage; a
joint tuned again later in the sequence gets a fresh budget. `--home-retries`
(default 3) separately bounds extra slow approaches when two homing contacts
disagree. `--trials` is required for every mode that moves the arm except
`--baseline` and `--probe-endstop`. The positional argument is the config path
(default `config/PAR6.toml`); the URDF assets are resolved as
`<config dir>/../assets/par6_description`.

Acceptance limits are explicit, joint-side, and never widened by the run:

| Option | Default | Meaning |
| --- | ---: | --- |
| `--moving-rms-deg-s` | 5 | floor of the moving limit: RMS speed error while a commanded motion is under way |
| `--moving-rms-fraction` | 0.15 | the moving limit is the larger of the floor and this fraction of the leg's peak commanded speed |
| `--holding-rms-deg-s` | 1 | RMS speed while holding, from the encoder-derived estimator |
| `--hold-observation-s` | 1 | length of every holding measurement |
| `--gain-step` | 1.25 | ratio between successive gain scales while walking toward a band boundary (STEPFOC's "back off 20%") |
| `--gain-ceiling` | 2.5 | no gain may leave [1/2.5, 2.5] × its value at the start of the run |
| `--gain-resolution` | 1.1 | bisection of a boundary stops when pass and fail scales are within this ratio |
| `[motion].settle_tolerance_rad` | config | position error allowed once a position-mode move has ended, and during holds; lag while moving is logged, not judged |

The moving limit scales with the command because the drive's reported speed
carries a ripple that scales with speed: on this arm J4 at vendor gains showed
8.1% of its 63°/s peak command (4.4°/s tick to tick) and J5 10.8–11.2% of
74°/s at every gain within bounds, both with quiet holds, while J2's 5.3°/s
seek sits under the absolute floor. An oscillating J3 measured 21% and more. Each `END` line logs the
limit that applied. The fraction is a stated requirement for this arm, not a
derived constant; change it on the command line if the arm or the requirement
changes.

It refuses to start while `par6d` or another `par6-selfcal` is running: two
processes commanding the same drives means the arm obeys whichever frame
arrived last. `--sim`, `--check-gravity` and `--replay` skip that check.

Needs `CAP_SYS_NICE` or root on hardware: the main thread moves to the
configured control CPU under SCHED_FIFO with memory locked, and the recording
thread is kept off that CPU. A tick later than `[bus].stale_warn_s` past its
deadline, a drive fault, or feedback older than that interval ends the run.

### What a run does

**Startup.** A run starts from the configuration's gains and gravity factors;
`--from-vendor` replaces them with the vendor velocity gains (RCB-Runtime
`PAR6.xml`) and `gravity_scale = 1`. Homing currents come from
`[homing.joints].current_ma` (this arm: 700 mA J1, 800 mA J2/J3/J6, 1200 mA
J4/J5). All six drives hold their current position with a
zero current cap, the cap ramps to the operating limit over `[jog].accel_time_s`,
then every joint's hold is measured for one observation interval. A joint that
fails the holding requirement gets the band search below over its Kpv/Kiv
scale, on that hold, before anything moves.

**Homing.** The configured `[homing].sequence` runs in order. A seek is a
velocity-mode move at the homing current with the vendor stall detector
(80 ms windows, 70% of the current limit, 60% occupancy, encoder range below
max(10, 25% of the commanded travel)). On contact the joint reverses in
velocity mode at the operating current for `backoff_s`; a joint that cannot
move either way ends the run. If the reverse move fails the movement or holding
requirement, that joint is tuned where it stands, in STEPFOC's cascade order:
velocity Kpv alone, then Kiv alone with Kpv fixed (every velocity-mode leg
judged on both the movement and the zero-speed holding requirement), then
position Kpp. The startup hold search scales Kpv and Kiv together.
Each stage is a **band search** over one scale factor: starting at the current
gains, walk in ×1.25 steps away from the failure seen (a failed hold means
oscillation, so walk down; a quiet failure means sluggish, so walk up). The
violation must shrink along the walk: if the first step is worse the walk
reverses from the start, and if a later step is worse the line has no passing
scale and the stage fails. From the first pass, extend to the other boundary
the same way, bisect each boundary to ×1.1, and operate at the geometric
midpoint of the passing band, the point farthest in ratio from both failure
modes. A start that already passes is probed one step each way and kept if
both pass (STEPFOC's 20% margin on both sides), so a joint that is already
tuned costs three experiments and is never driven to its oscillation edge. That midpoint is measured once more
and must pass; a band that cannot be bracketed within the budget or the ceiling
fails the stage rather than accepting the last point that happened to pass. When no Kpv passes at the current Kiv, the
joint moves to the least-violating Kpv, Kiv is searched there, and Kpv is
searched once more (one round of coordinate descent) before the stage fails.
Every experiment is a reset to the same start, a one-interval hold there, then
an out-and-back leg pair. Speed-error RMS is accumulated after the vendor
detector's guard window (the ramp, and after a stall the integrator unwind),
so a reverse move off an endstop is judged on the same basis as a tuning leg.
A joint tuned again later in the sequence searches inside the band its earlier
episode verified instead of walking back out to the oscillation edge; a
position move is aborted mid-way only when the joint stops following (below
the vendor's 25%-of-command speed), not for lag. The `BAND` line in the log records the lowest and
highest passing scales, the failing scales beside them, the operating point and
whether it was confirmed. Stall homing then repeats the approach at 30%
speed and requires the two contacts to agree within `two_pass_max_diff_ticks`;
Hall homing takes the Hall edge; a configured `release` stage samples the
reference under a current command. Every position move during and after homing
is measured the same way and can trigger the same tuning, so gains accepted on
a seek are re-tested by the later moves.

Every hold measurement covers all six joints. When the moving joint's own hold
passes and a passive joint's fails, the run ends with that measurement and the
active joint's result in the message; tuning is never started on a joint other
than the one being moved. While the moving joint itself fails, passive flicker
is treated as that shaking seen through the arm and the active joint is tuned.

**Gravity** (skipped by `--home-only`, `--joint` and `--baseline`). Before any
gravity motion the binary loads the daemon's collision world (arm, fitted
tool, the configured installation shapes including the floor) and checks
every straight joint-space segment between the stage's waypoints at 40
samples; a colliding segment ends the run before it moves, naming the segment
and the pairs. Pose transitions move all joints together along one quintic:
joint-at-a-time transitions would swing the gripper through the table between
the ready and forward-reach poses. For J6 down
to J2: fit a holding-current factor at a fixed loaded pose from both approach
directions; a joint whose model gravity current there is below the fit's own
residual (the tool roll, at −3 mA) is reported as not identifiable and keeps
the vendor factor. Otherwise approach the pose from each direction and probe
drift for 2 s under the vendor model's torque after each, then probe the
The factor is kept only if, judged on the worse of its two approach
directions at each pose, it is complete and no worse than vendor at both poses
and better at one; otherwise the vendor factor stays. Directional friction
makes one direction easy for any factor, which is why the worse one counts. `--check-gravity` prints the poses and predicted currents
without moving.

**Shutdown.** Always attempted, also after errors and Ctrl+C/SIGTERM: homed
joints park: the shoulder and elbow return to their homing endstops (moving
last) so that releasing current leaves them resting on the stops rather than
dropping, the other referenced joints go to `robot.park_pose_rad`, and a joint
the run never referenced returns to the encoder position the run found it at,
so the next run starts from the same posture (the vendor's pre-homing wrist
nudge is a relative move that assumes it). Then current is released. With `[selfcal].release_on_failure
= true` the release happens even when parking fails; the run still reports the
failure.

```toml
[selfcal]
release_on_failure = true # this arm: park, then release even if parking fails
```

### What it writes

Each run creates `<output-dir>/hardware-selfcal-<ns>` (or `sim-selfcal-<ns>`)
and prints `RUN_DIRECTORY`:

| File | Content |
| --- | --- |
| `invocation.txt` | command line, executable path/size/mtime, resolved config, trial budget, starting-gain source, mode flags |
| `acceptance.txt` | the limits in force |
| `config.before.toml`, `starting-config.toml` | the input file and the bundle actually used (vendor gains and homing currents applied) |
| `gravity-poses.csv` | planned poses and predicted currents (full runs only) |
| `console.log` | every phase, motion, detector window, hold measurement and trial score; ends with `RUN_RESULT` |
| `samples.csv` | one row per joint per tick: tick, rx/tx timestamps (ns), feedback generation, encoder ticks, drive speed, current, the command sent |
| `accepted-gains.toml` | per-joint gains as each search stage accepts them, kept even if the run fails later |
| `verified-gravity.toml` | per-joint gravity factors as each is verified, kept even if the run fails later |
| `result.txt` | `Ok(())` or the error |
| `candidate.toml` (`--joint`) or `calibrated.toml` | the full config with measured gains and gravity factors, on success only |

`--apply` (hardware only) patches the measured `kpv`/`kiv`/`kpp` lines of each
joint's `[joints.gains]` table and the `gravity_scale` array into the input
config as written, so its comments and layout survive, after checking that the
patched text still loads and validates. One `<config>.toml.before-selfcal`
backup is kept from the first application. `--baseline` and `--probe-endstop` write no configuration.

`--replay <run>` re-runs the recorded `samples.csv` through the holding
estimator and re-ranks the logged motion legs with the current limits, printing
`REPLAY` lines; it opens no bus. Use it to check a scoring change against
recorded hardware before spending arm time.

### Reading console.log

- `HOLD_QUALITY`: `speed_rms` is the end-fit FOAW estimate from encoder
  positions with a one-count residual bound (a joint sitting on a count
  boundary reads as stationary); `raw_speed_rms` is the drive's own reported
  speed, diagnostic only. `peak_error` in counts tells you whether a failure is
  a real excursion or one-count flicker (J4/J5: 0.0055° per count).
- `END <Outcome>`: `Complete` met the requirements; `Tracking` completed the
  path but failed a movement or holding requirement; `Blocked` is a detected
  stall; `Timeout` ran out the seek window.
- `trial=N stage=... rank=...`: `Feasible { objective }` passes at ≤ 1;
  `Infeasible { violation }` failed a constraint by that normalised amount;
  `Invalid` is an incomplete experiment. `BAND` summarises the search.
- The drive's reported speed is one 6250 Hz encoder difference averaged over
  20 samples (STEPFOC V108 `Collect_data`/`movingAverage`), so it is quantised
  in 312.5 ticks/s steps over a 3.2 ms window. That is the ripple every
  velocity-mode `velocity RMS` contains and why the moving limit scales with
  the command.
- `DETECT`: the stall detector's inputs and branch for each window.

### Repeatability

The band search locates the boundaries of the passing region, so different
starting points should end at the same midpoint to within the resolution and
measurement noise. On this arm the first hardware run of the search put J3's
velocity band at ×0.41–0.85 of vendor and operated at Kpv 0.00883 / Kiv
0.000883, against 0.009 / 0.0009 from the September 11 routine. `scripts/selfcal_repeat.py` runs one joint several times and
tabulates the accepted gains, the trials used and the spread:

```sh
scripts/selfcal_repeat.py --joint 4 --runs 3 --trials 24 --sudo               # every run from vendor gains
scripts/selfcal_repeat.py --joint 4 --runs 4 --start-scales 0.67,1,1.5 --sudo  # from scaled config gains
scripts/selfcal_repeat.py --joint 5 --runs 2 --trials 6 --sim
```

A joint whose configured gains already pass reports no accepted gains and zero
trials: the run confirmed them without searching. Agreement across the vendor
start and scaled starts is the evidence that the requirements, not the starting
point, determine the result.

### Tests

```sh
pixi run cargo test -p par6d --bin par6-selfcal   # estimator and band-search regressions
pixi run cargo clippy -p par6d --bin par6-selfcal --all-targets -- -D warnings
target/release/par6-selfcal config/PAR6.toml --sim --home-only --trials 12
```

The simulated run exercises the whole homing/tuning path on the sim bus. Its
plant does not reproduce the hardware's velocity-loop ripple or oscillation, so
it checks sequencing and file outputs, not gain acceptance.

### Historical hardware measurements (previous implementation, before September 15)

These were measured by the frequency-response version of the binary that the
current search-based implementation replaced; `--repeat` no longer exists.

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

Measured dither while holding a calibrated pose: J2 0.0009°, J3 0.1190°,
J4 0.0055°, J5 0.0055°. The elbow is twenty times the rest, which is consistent
with the ring that is audible from the arm at 0.204°; it is reported rather than
chased, because no velocity gain fixes it (lower and the joint cannot move the
load at all) and it sits below what can be heard.

One more characteristic: the elbow's drift readings scatter about
0.01–0.05 deg/s at its loaded pose, where the wrist and wrist pitch repeat to
0.005. That is stiction in a joint carrying 2 A, and it is why the hold
tolerance sits above that scatter and why verification re-measures rather than
trusting one reading.

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
