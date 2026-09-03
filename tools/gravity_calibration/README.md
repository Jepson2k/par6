# Gravity identification harness

Four phases that check the gravity model the runtime enforces against the
arm it drives, plus the offline evidence that the Tier-2 simulator's MuJoCo
scene describes the same arm. Every script runs against a RUNNING `par6d`
through the shipped client and takes every kinematic quantity from the
runtime itself: the model's answer is the `gravity_torques` telemetry field,
published every tick at the measured pose, and the holding torque is the
runtime's kt-calibrated measured torque — no client-side model, no tracking-
error inference. Nothing that moves the arm runs without `--go`; every
script prints a ledger and exits 1 on a required failure; `--json` emits it.

| phase | script | needs | what it establishes |
|---|---|---|---|
| A | `phase_a_sign_probe.py` | freshly started `par6d`, drives idle | the motor→URDF sign convention, from a person moving one joint at a time — fixed **before any torque** |
| B | `pd_sweep_id.py --go --joint J --pre …` | homed arm | a slow servo staircase up AND down (averaging cancels Coulomb friction), logging the full six-joint pose, the measured torque and the runtime's G(q) at rest on every step |
| C | `fit_sweeps.py results/pdsweep_*.json` | nothing | the fit: model scale `k ≈ 1`, zero offset `≈ 0`, bias below the friction floor |
| D | `auto_float_test.py --go --joint J --lift …` | homed arm | lifts a joint under position control, floats the arm on G(q) alone, reads the drift and — with `[freedrive] drift_lock` — the integral that converged to the model's error |
| — | `twin_evidence.py` | `par6d` binary + `mujoco` | the MuJoCo scene against the runtime's model: gravity residual, vendor fixture, mass table, timestep convergence, damping time constants |

## Safety policy

Non-negotiable, and the same on every script:

- **The position loop is always in command during identification.** A sweep
  is a `servo_j` stream the drive's own loop follows; a float is the
  runtime's freedrive, IDLE under G(q), whose feedforward the runtime
  itself ramps in under its torque-rate limit.
- **Torque feedforwards soft-start.** The runtime's `torque_rate_nm_s` limit
  ramps the gravity term whenever it is switched on; nothing here bypasses it.
- **Every abort freezes with a position hold, never torque-off** — a loaded
  arm free-falls. Leaving freedrive re-engages the hold at the current pose;
  `stop` ends a stream in the pose the drives are at. Then the script
  reports and lowers under control (`move_j` at 10 % speed). An interrupt
  does the same.
- **Velocity aborts use finite differences of the measured position**, not the
  drive's velocity register (measured 2.6–4.8× off, with inconsistent sign,
  on the reference build).
- The excursion of every move is checked against the joint's soft window
  **before anything moves** and refused with the numbers.

par6 has no torque-off verb by design — it never torque-offs a loaded arm —
so phase A runs on a freshly started daemon (it boots the drives idle and
limps them on the way out) and refuses an ENABLED arm.

## The clearance lesson

The reference harness's first pass reported per-joint zero offsets, a
cable-harness torsion spring on one joint and a ×1.66 wrist scale. **All
three were contact artifacts** from sweeping out of the rest pose — gripper
on the table, link on link — with convincing fits and good residuals. Their
contact-free result: the model correct to 5–11 % on the load-bearing joints,
no offsets, biases below friction. Always pre-position to a clearance pose
(`--pre`) and exclude the contact region (`fit_sweeps.py --qmin`). par6's
model has never been checked against the arm at all; these are the scripts
that do it.

## Method

With every other joint held, the gravity torque on a revolute joint is
exactly a sinusoid in that joint's angle — the distal centre of mass rotates
rigidly about the axis. So the measured torque over a sweep is
`A·sin(q + φ) + fric·dir + c` and the runtime's G(q) at the same samples is
`A'·sin(q + φ')`, and the fit is closed-form: `k = A/A'`, offset `φ − φ'`,
bias `c`, Coulomb friction `fric`. A sweep under 0.5 rad cannot resolve the
phase and is flagged; a joint whose gravity signal is below its friction
floor (the wrist) has nothing to tune and is reported as such.

Divergences from the reference harness, all deliberate: the holding torque
is measured (drive current × kt), not inferred from the tracking error and an
assumed loop gain — and because a drive in motion carries its loop's dynamic
effort on top of gravity, the sweep is a staircase and only the rest tail of
every step is logged; phase D floats the whole arm rather than servoing the
neighbours, so every loaded joint is validated at once and every joint is
watched; the fade-to-float and the clamped integral are the runtime's own
freedrive and drift lock, not a client-side loop.

### On the simulator

`python/tests/test_gravity_calibration_tools.py` runs phases B–D against
`par6d --sim --sim-dynamics`. The float validates for real there (a payload
declared to the controller and not to the plant is a wrong model, and the
lock reads it), but the sweep's model scale does not: the torque plant holds
a joint by clamping its velocity rather than by balancing torque, so the
holding current it reports at rest is not gravity — the staircase reads a
clean, repeatable scale of about 1.46 and a 1.4 N·m bias on the elbow that
no hardware drive would produce. The tests therefore assert the staircase's
mechanics, the phase and the fit quality on the simulator; `k ≈ 1` is a
hardware result.

## Twin evidence

`twin_evidence.py` spawns a private `par6d --sim` (no arm anywhere near),
teleports through the vendor fixture poses plus seeded random poses, and
compares the runtime's G(q) with the scene's generalized holding force at
rest. It also pins the runtime's G(q) to the vendor dynamics fixture,
tabulates URDF link masses beside MJCF body masses, integrates a contact-
free free fall at 1, 0.5 and 0.25 ms to show what the scene's timestep
costs, and measures free-decay time constants at the vendor's class-Y
damping, the scene's override and the config's reflected motor damping. The
committed `results/twin_evidence.json` is the evidence; regenerate it when
either model changes.

### What the committed evidence says

From `results/twin_evidence.json` (park and hold poses plus 20 seeded poses
inside one encoder turn; the ten vendor fixture poses lie outside the soft
window and stay pinned by `par6-kin/tests/gravity_reference.rs`):

- The scene and the runtime agree on the load-bearing joints: shoulder and
  elbow torques correlate at r > 0.98 across poses, the scene reading 15–20 %
  heavier. That excess is the scene's own gripper bodies (0.67 kg of
  `gripper` + `tcp_link` + camera) against the runtime's configured tool mass
  (0.37 kg): moving mass 5.82 kg in the scene versus 5.48 kg in the runtime.
- The wrist does not agree: wrist-pitch torque correlates at r ≈ 0.36 and the
  forearm-roll magnitude is 0.42× — the hand-edited tool mass distribution in
  the MJCF is not the URDF tool the runtime models. That is the drift this
  script exists to catch, and it exits 1 on it until the scene is re-derived.
- Timestep: a 0.3 s contact-free free fall moves by 3.9e-4 rad between 1 ms
  and 0.5 ms, and 1.9e-4 rad between 0.5 and 0.25 ms — first-order
  convergence, 1 ms is adequate.
- Damping: the vendor's class-Y 25 N·m·s/rad gives free-decay time constants
  of 1–5 ms (a frozen arm, which is why the override exists); the scene's
  0.5 gives 15–190 ms; the config's reflected motor damping (G²·b) would give
  0.1–0.65 s. The plant's idle brake (40/s) dominates whenever a drive idles,
  so the scene value only shapes driven motion, where it sits between the
  vendor's and the config's.

## Results

Hardware runs write JSON under `results/`; commit them. Rows in the parity
tracker move to "parity" only once phases A–D have been run on the arm and
their ledgers committed here.
