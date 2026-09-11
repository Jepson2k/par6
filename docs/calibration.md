# Measured calibration

`par6.calibration` provides gravity identification, a resumable motion-envelope search, and a comparison of streaming motion limits. These are local experiments against the ordinary controller client. They do not change electrical motor calibration or automatically install a result.

The initial protocol is for the configured gripper with an empty hand and zero declared payload. Home the arm first. Each run checks fresh feedback, drive faults, its configuration readback, the collision world, and joint windows before moving. Every stimulus first runs through the native stream limiter preview. Its actual position/velocity output, including arrival, must stay within the experiment envelope and collision world. The preview checks unfiltered targets; recorded native commands remain the evidence of what ran. Every stimulus ends with an acknowledged Stop and measured encoder rest. Failed and cancelled trials retain their recording offsets.

## Run from Commander

Build and install the local Python package/runtime using the repository's normal pixi tasks. From the Commander checkout, start Commander with a unique capture filename and an explicit robot configuration:

```sh
mkdir -p calibration-runs
PAR6_DIAGNOSTICS="$PWD/calibration-runs/session.bin" \
PAR6_CONFIG="$PWD/calibration-runs/baseline/PAR6.toml" \
waldo-commander
```

The configuration's `grippers/` directory must be beside its TOML. For development from this checkout, `PAR6D_BIN` can name `par6/target/release/par6d` and `PYTHONPATH` can name `par6/python`, with the matching native extension installed there. The existing `--dev-mcp-autopilot` option remains available for explicitly authorized development sessions.

Open `examples/calibration/run.py` in Commander. Set `ROUTINE` to `check`, `feedback`, `gravity`, `verify-gravity`, `limits`, or `smoothness`, then run it normally or through Commander MCP. `check` makes short opposite-direction shoulder/elbow sweeps and produces measurements only. Set `PAR6_CALIBRATION_JOINT=2` for a J2-only `check`. Calibration requires the live runtime, rather than a static source-code preview; each path is preflighted against its reported collision world before execution.

The recorder is opt-in, creates a new owner-only file, and refuses to overwrite an existing file. Its default budget is 900,000 native snapshots (one hour at 250 Hz, approximately 500 MB). Set `PAR6_DIAGNOSTICS_MAX_SAMPLES` before runtime startup for longer experiments; the daemon also accepts `--diagnostics-max-samples`, which takes precedence. The value must be a positive integer. For example, 3,600,000 samples permits four hours at 250 Hz and uses at most approximately 2 GB. Recording includes idle time from startup. At the limit it flushes and stops; the controller continues running, but calibration rejects the stale evidence. Choose the budget before collection: restarting or rehoming invalidates an envelope continuation checkpoint. The real-time thread writes to a fixed ring; disk writes occur on a separate thread. Missing ticks, stale recording, inconsistent model/configuration, and invalid measured values reject the experiment. Keep the native capture alongside the run directories: trial reports identify offsets in that capture.

## Gravity

The protocol varies shoulder, elbow, and wrist configurations and collects both directions at each gravity-loaded joint (J2–J6). Its 0.36 rad (20.6°) sweeps plan six configurations, with shoulder offsets up to 0.8 rad from the starting pose. Samples must be moving slowly with low measured acceleration and without current saturation. Complete pose groups are held out from fitting.

`gravity(session, prior=directory)` retains observations from a previous run with the same configuration and reported drive identities, then recollects the six physical centers with their original training/validation assignments. The starting pose must match within 0.001 rad. `PAR6_CALIBRATION_PRIOR` exposes this in the example runner. Previous training/validation assignments remain separate. `gravity-protocol.json` records the requested coverage and retained sample counts.

Hardware calibration requires obstacle geometry in the connected collision scene, including a correctly located work surface. The scene is recorded in `world.json`; an empty hardware scene is rejected. This presence check cannot establish that dimensions, mounting height, or tool geometry match reality. All approaches and sweeps are checked before collection; a rejected path requires selecting different poses or correcting the scene from measurements.

The fit projects moving Coulomb/viscous friction out of the gravity regressor, then fits observable parameter directions. Each joint must also meet an absolute held-out torque residual limit (default 0.05 Nm, recorded as `gravity_residual_nm` in session acceptance and `validation_limit_nm` in the fit). Relative improvement alone cannot accept a poor model. Unobservable directions retain the nominal model. It reports rank, friction, and held-out residuals and rejects implausible corrections, negative dissipative friction, regressions, or insufficient improvement above its noise floor.

A correction has 24 coefficients: four mass/first-moment terms for each of the six composite arm bodies. The fixed gripper is part of body six in the native model. The correction does not rewrite the gripper/payload declaration or dynamic inertias. It describes this configured assembly; gravity data alone cannot identify every physical inertial parameter or distinguish an incorrectly declared tool from an arm-model error. Current-derived torque depends on valid drive torque constants.

The simulator defaults to ordinary Coulomb/viscous drivetrain friction. Load-dependent holding friction is available only as an explicit simulator configuration; it is not inferred from a planetary gearbox. The existing payload estimator also samples while moving through each pose in both directions, avoiding stopped-friction bias.

## Motion envelope and smoothness

The envelope search increases one derivative by 10% at a time, with three repetitions in each direction at two configurations. Each candidate must reach at least 90% of the requested derivative in both post-limiter commands and measured response. Measured excitation uses divided differences of encoder positions with an explicit half-count quantization allowance, rather than peaks obtained by differentiating the drive's velocity estimator. Coarse encoders or short ramps can make a derivative unproven even when the command reaches it; such a trial is censored. The bound assumes position samples correspond to their recorded measurement times and the stated encoder quantization; unmeasured drive acquisition/transport timing is not covered. These are commissioning measurements, not direct inertial measurements. If geometry or another bound prevents sufficient excitation, the result is censored rather than reported as a measured maximum. Operating limits are 80% of fully passing candidates.

A third configuration is reserved for coupled validation. Its bounded out-and-back paths exercise all six joints and all three derivatives, using simultaneous and opposing joint directions, both initial directions, and three repetitions (12 trials). Every trial must meet ordinary motion acceptance and the same measured excitation requirement across the complete six-by-three limit matrix. This establishes a tested operating envelope, not an absolute mechanical limit or a guarantee for other payloads/mountings.

The default budget is 180 stimulus trials per invocation. This bounds each invocation, not total recording duration. For STREAM caps of (0.2 rad/s, 0.4 rad/s², 1.2 rad/s³), the all-pass search plans 1,224 isolated and 12 coupled trials; even excluding approach travel and computation, it needs approximately 69 minutes. Reserve a larger recording budget before startup for such a search. Set `RESUME` in the example to the previous `envelope-progress.json` to continue. Configuration, hardware/simulator mode, and reported drive identity/firmware changes invalidate continuation. Rejected candidates are recorded so continuation does not repeat them indefinitely. The trial budget includes coupled validation, which also resumes from recorded progress. Envelope protocol version 4 rejects older checkpoints without the current excitation and native stream-fraction checks. An incomplete or insufficiently measured search never exports an operating profile. Final-return or export errors also leave an invalid, incomplete report with the error retained. Existing exported profiles require a new session directory before another search. Reports identify `tested_mode=STREAM` and `applied_validation_complete=false`: the staged EXEC limits need a separately recorded queued-motion comparison after activation, because EXEC planning, torque feedforward and settling differ from STREAM.

`feedback` diagnoses a selected joint using sustained movements and powered
holds. Its default excursion is ±0.18 rad at a target speed of at most
0.05 rad/s; `amplitude_rad` may reduce the excursion to 0.02 rad where geometry
requires it. Every path is checked before testing. Baseline and candidate
trials use matching initial poses. Actual travel and stationary hold are
assessed separately, so a long quiet hold cannot dilute moving vibration.
Overlapping 1.2-second windows, spaced 0.5 seconds apart and including an
end-aligned window, also cover every transition. Their worst metrics enter
acceptance and candidate scoring, so settling vibration cannot escape between
the travel and final-hold crops.

The search first tests 80%, 60%, and 40% of the integral gain with proportional
gain unchanged, then tests reductions of both velocity gains. Position gain,
current-loop/electrical gains, current limits and torque constants are retained.
It prefers the smallest reduction meeting motion, hold, latency, encoder-noise
and separate validation checks. Each candidate restores the starting tuple;
no drive flash save is requested. Bounded motion rejections remain evidence
after confirmed Stop; faults, lost readiness and failure to stop end the run.
`JOINT` in the example is one-based (default J3); `PAR6_CALIBRATION_JOINT` selects
it. An optional `center` argument (six radians), or `PAR6_CALIBRATION_POSE`
(six comma-separated degrees), reproduces a recorded failing configuration.

These CAN configuration writes are one-way. The acknowledgement confirms a controller request, not motor parameter readback. Run with exclusive control, starting from the loaded configuration; do not mix another client's live tuning into an experiment. A failed restoration must be resolved by reloading the saved baseline configuration before more calibration. A diagnostic powered hold records oscillation as its outcome; it still ends with the same Stop/rest confirmation as every other stimulus.

Smoothness protocol 2 compares a lower-acceleration/jerk STREAM candidate with
matching-endpoint baseline moves. The comparison speed is at most 0.3 rad/s;
its exact caps are derived from the loaded controller's global stream fractions.
The candidate retains that speed cap and halves acceleration and jerk. Both
preview and execution use the same fractions. If the 0.08-rad probe is dominated
by speed and cannot meaningfully exercise the change, the routine refuses it
before moving.

All six joints require three repeats in each direction at three configurations.
The third configuration is held out. Acceptance checks complete matched evidence,
ordinary motion quality, and oscillation/response-delay regression separately for
each pose, requested joint, and direction, averaging only repetitions of the same
excursion. Training and held-out summaries stay separate; improvement elsewhere
cannot cover a worsening validation stroke. Training must show at least 30%
improvement above the noise floor. A quiet unchanged comparison does not establish
an improvement. A quiet held-out baseline must remain quiet.

Interrupted or failed runs retain an invalid, incomplete report and cannot export
a profile. An accepted candidate changes STREAM limits only; JOG needs its own
motion evidence. This estimates joint oscillation at the tested excursions, not
table vibration or performance at untested combinations. No input shaper or
electrical-loop gain change is applied, and physical/post-activation validation
remains separate.

## Activate or roll back

Successful experiments stage `candidate/`, `rollback/`, and `profile.json` beside the evidence. Both configurations contain their gripper files. The candidate changes only requested gravity, motion-limit, or position/velocity feedback settings; installation collision geometry and other configuration are preserved. Export requires the calibration report's baseline fingerprint to match the source bundle.

Stop motion, restart Commander with `PAR6_CONFIG` naming the staged candidate TOML, and set `VERIFY_PROFILE` in the example to that path. `verify_applied` checks the saved content fingerprint, native validation, exact runtime/gripper readback, hardware/simulator mode, and available drive identity/firmware before comparison motion. Use the rollback TOML with the same process to restore the previous configuration. The currently observed motor serial numbers are not unique, so matching drive metadata cannot prove physical unit identity by itself.

A mathematically valid fit is a candidate. Hardware comparison after activation is required before calling an arm calibrated. Retain the baseline and candidate reports, active configuration, raw capture, and observed mounting/gripper/payload conditions.

## Stream limits and slow calibration targets

A smooth 50 Hz input does not imply smooth motor commands. With large stream
jerk limits, the online planner can accelerate and brake toward every incoming
position. The observed J2 case produced 50 Hz command ripple and about
10 rad/s² acceleration on a nominal 0.07 rad/s gravity sweep. The command
preflight now refuses such output even when the input trajectory is gentle.

Use a validated stream profile before collecting gravity data. The current
experiment compares stream limits equal to the execution limits; in the native
J2 preview this preserved speed while reducing command ripple by about 99.99%.
That is an offline comparison, not a table-vibration measurement or a certified
maximum operating envelope. A short recorded physical comparison is required
before proceeding with longer collection.

## Acceptance and gravity verification

Each session writes `acceptance.json`. Ordinary approaches, checks, gravity
sweeps, and envelope trials reject excessive measured response; recording a
metric alone no longer counts as checking it. The live monitor evaluates a
recent native window every 0.5 seconds after one second of observations. It
also evaluates the complete stimulus before its samples can enter a fit.
Capture decoding and spectral analysis run outside the 20 ms command producer,
with a 0.5-second analysis deadline, including its final pending result. Current
and readiness checks continue during settling. Missed command or analysis
deadlines stop the experiment and retain its evidence.

Default commissioning thresholds are 0.5° peak tracking error, 0.2 seconds of
sustained current saturation, 0.03 rad/s velocity-residual RMS, 0.03 rad/s
measured velocity RMS above 20 Hz, and 0.015 rad/s commanded velocity RMS above
20 Hz. Measured-velocity thresholds have a floor of two encoder counts per
native interval. `Acceptance` can supply explicit experiment thresholds; their
values are retained with the evidence. Native sampling must exceed 40 Hz.
These are commissioning acceptance criteria, not mechanical ratings or a
measurement of table motion. Bounded feedback diagnosis and the smoothness
baseline may record oscillation deliberately; their tracking/current checks
remain active and those measurements cannot pass ordinary motion acceptance.

A successful mathematical fit produces a **candidate** with
`applied_validation_complete: false`. After loading that candidate and checking
its exact configuration with `verify_applied`, run
`PAR6_CALIBRATION=verify-gravity` for an independent torque consistency check.
It uses six candidate shoulder/elbow/wrist configurations distinct from fitting,
retaining only complete collision-free groups. At least three groups and all
observable gravity directions must remain before any motion starts. Every
loaded joint must provide matched travel in both directions. Actual pose and
speed overlap, torque residual/spread, ordinary motion acceptance, and measured
regressor coverage are checked; the candidate is never refitted to these data.

Gravity identification and verification disable gravity feedforward once before
collection and leave active feedback support engaged, including after Stop.
They compare measured current-derived torque with the loaded gravity model while
position feedback controls motion. The result kind is `gravity-torque-verification`:
`valid` means consistency under the stated odd-friction assumption on those
measured paths. It does not establish true inertial parameters, arbitrary
combinations of individually exercised joint ranges, or the behavior when
feedforward is subsequently enabled. `applied_validation_complete` remains false.

The adversarial reviewer constructed strictly dissipative directional friction
that makes a model with every gravity parameter 5% low pass both fitting and
matched-motion checks. At one tested pose J3 is 0.132 Nm low. More equivalent
current measurements cannot separate those explanations; independent torque,
known-load or richer dynamic evidence is needed. Current-derived torque also
inherits uncertainty in calibrated motor torque constants.

The former gravity-only assay is now simulation-only and no automatic routine
invokes it. Ordinary friction hid a 0.264 Nm J3 deficit during a 20-second hold;
a separate wrist example traveled 4.29 degrees despite the 0.25-degree software
abort threshold. That threshold does not bound stopping distance. A validated
runtime reaction/stopping margin is required before physically releasing
feedback for such probes. Simulator probes still exercise cancellation and
finish with compensation disabled and active zero-velocity holding.

Nominal simulator approaches exposed motion-quality failures even without a
preceding gravity-only hold. The independently fitted candidate subsequently
completed all 50 verification sweeps and 101 total motion records in an isolated
native simulator. Paired torque, measured coverage and motion quality passed;
maximum paired residual was 0.00367 Nm against the 0.05 Nm criterion. One of the
six planned pose groups was excluded for collision before motion. Evidence is in
`calibration-runs/gravity-fitted-v4-independent-v2/run/` in the Commander checkout.
This validates current-derived torque consistency with active feedback and
compensation disabled under the stated friction assumption. Compensation-enabled
EXEC validation and physical verification remain outstanding. Holding across a
server restart is also not implemented.

Continuation now also requires matching acceptance criteria and the same
recorded process/reference interval. Rehoming, restarting, or old recordings without this provenance require a
fresh collection. This capture-derived identity does not detect a simulator
teleport that keeps the homed flag set; simulator continuation must not cross
such an external state change. A verification
pass describes the tested poses and assembly; it cannot independently confirm
an encoder's physical homing reference or exclude friction masking imbalance
elsewhere in the workspace.

A provisional six-entry root `gravity_scale` can trim individual feedforward
terms (default all 1; each value must be positive and at most 2). It changes
neither torque constants nor current limits/PID gains. Fitting a new absolute
gravity correction resets this manual trim to unity to avoid double counting.
The fit's baseline comparison includes an existing trim.

### Local telemetry timing

Calibration combines the daemon-reported bus data age with the native client's
local receipt age. The receipt timestamp is retained through Python delivery and
rechecked immediately before each streamed target, during settling, and when
confirming rest after Stop. This covers native-to-Python delays; it does not
measure time spent in transit before the client receives a datagram. Missing or
stale status ends the experiment.

Feedback diagnosis uses the same 0.05 rad/s ceiling for repositioning and measured
travel. It permits oscillation while reaching the diagnostic starting pose,
retaining tracking, current, readiness, geometry and Stop checks. Accepted gain
candidates must pass ordinary motion and hold vibration gates.
They must also pass the overlapping-window checks across the full recording.

### Simulation and measurement clocks

MuJoCo advances one configured timestep per native tick. Calibration uses that
physics clock for simulator position derivatives, spectra, and torque sample
selection. Physical recordings use monotonic host measurement times. Native
host timestamps remain intact and independently enforce capture/liveness
checks; the simulator clock cannot hide a missed 100 ms host deadline.
`acceptance.json` records `measurement_clock`, so continuation cannot mix old
wall-clock simulator estimates with corrected measurements.

The completed 50-sweep native gravity recording passed all motion, paired-torque
and coverage checks when reanalyzed on its physics clock. The original final J6
pairing refusal and unchanged raw recording are retained alongside the corrected
analysis. This is recorded simulator evidence, not physical calibration.

The envelope workflow preflights against the actually loaded stream limiter.
A successful preview with different candidate limits does not establish that the
same input can be used under the current configuration. Coupled preflight can
therefore refuse a proposed lower envelope before any approach. A complete
maximum-envelope search and physical validation are still outstanding.


## Collection completeness and native stream bounds

Gravity collection protocol 3 excludes only complete pose groups whose approach,
excursion, or return intersects the declared collision world or joint window.
The remaining groups retain their original training/held-out split. Each split
must independently cover the observable gravity directions before motion and
again from measured samples. Collection requires the requested joint to move
in the requested direction, at least 30 steady observations, and at least
0.12 rad of measured travel for every requested sweep. Repeated visits retain
the physical center identity. Older collection protocols cannot supply prior
samples under these guarantees.

Coupled envelope validation uses the actually loaded stream configuration.
Existing servo speed and acceleration fractions lower its limits consistently
in preview, execution, and settling; acceleration and jerk share a scalar.
Their values are retained with trial/checkpoint evidence. Envelope protocol 5
rejects checkpoints without those execution semantics. Incompatible per-joint
or acceleration/jerk ratios remain inconclusive when their measured excitation
is insufficient. The motor commands recorded during the trial must also obey
the experiment's upper bounds, including transitions at arrival; passing a
preview alone cannot establish that. The current acceptance policy (version 7)
binds these checks to continuation evidence.

A delayed Python Future can contain an older status than the native receiver's
latest packet. Readiness may select that genuinely newer packet, preserving its
original receipt time. It still refuses stale native feedback; the separate
command producer and analysis deadlines remain enforced.


Final trial acceptance (policy version 7) evaluates both the whole recording and
overlapping 1.2-second windows every 0.5 seconds, including an end-aligned window.
These checks run after confirmed Stop and are independent of when the live
analysis worker last sampled. A brief late vibration burst must not disappear
into a long trial average. Diagnostic trials may explicitly retain oscillation;
tracking, saturation, controller faults, and command limits still reject.

Envelope protocol 5 sizes single-joint speed, acceleration, and jerk pulses
against the loaded STREAM limits. Constant-jerk ramps, a constant-speed interval,
and matching native speed/acceleration fractions provide measurable excitation.
A requested derivative that cannot be exercised inside the joint window remains
unproven. EXEC limits cannot substitute for the actual stream caps.

Servo preflight also checks the native collision gate's stopping projections of
both commanded positions and requested targets. These projections assume ideal
tracking; the runtime continues to project using measured velocity. Approaches
try bounded planning fractions (1, 0.75, 0.5, 0.25) and reject if none has clear
geometry. This does not certify a physical stopping distance or replace runtime
collision checks.


Policy 7 also requires recording-source verification. `capture_info()` returns
`{"identity": null}` when the runtime has no native recorder, or an identity with
`pid`, `started`, `dt`, and `fingerprint` matching the CAP2 header. Calibration
checks it on entry and around every completed motion trial; in-trial freshness
checks retain that identity. A missing query, disabled recorder, changed runtime,
or different capture fails before its evidence can be accepted. Existing
`config_info` and `config_bundle` wire shapes are unchanged.

Gravity fitting removes numerical zero columns before column normalization.
Its relative threshold keeps duplicated observations from amplifying floating
point noise into a huge, physically meaningless correction. Independent torque,
coverage, and coefficient-plausibility gates still apply.

Public STREAM calibration uses gravity-disabled active feedback for approaches, measurements and cleanup. Reports and trial records explicitly label `gravity_comp=false`; changing it during a trial invalidates the evidence. Envelope protocol 5 refuses older or unlabeled continuation data, and smoothness protocol 3 records the fixed support mode. These measurements do not establish performance with gravity feedforward enabled.

Cleanup confirms the gravity-disable request before ordinary Stop. If that cannot be confirmed, it attempts a latched software EStop and observes active-error support using fresh native status, then always attempts Stop/rest. It records failures in `active-support.json` and never automatically clears that latch. Lost communication remains an unconfirmed cleanup, not proof that the arm is supported. The native regression for this support change is prepared and pending.
