# par6 pre-bring-up code review

Scope: the whole workspace (Rust runtime, Python client, C++ shim, config, spec/), reviewed
against two reference stacks — `parol6` (mature, working, same author, same waldoctl ABCs:
the *architecture and semantics* reference) and `source-robotics/rcb-runtime` (the vendor's
own runtime for this exact arm: the *behaviour and constants* reference).

Seven reviewers plus adversarial skeptics, plus a test-suite audit. 27 raw findings
deduplicated to **22 distinct defects**. Severities below are the post-skeptic ones.

**Licensing check: clean.** No code in par6 appears copied from `parol6` or `rcb-runtime`.
Constants and semantics are ported; the implementations are independent. Every reference
citation in this document is behavioural, not textual.

---

## 1. Verdict

**Do not take this to hardware yet.** The engineering underneath is good — the RT core is
disciplined, the CAN codec is exhaustively golden-tested, the config matches the vendor's
`PAR6.xml` constant for constant, and the homing FSM is a faithful port of the vendor's
state machine. That quality is exactly why the remaining defects are dangerous: they are
not sloppy code, they are *unwired* code and *half-ported* code, and both present as
working.

Three things make bring-up unsafe today:

1. **The physical e-stop button is not connected to the software.** `par6d` wires its
   e-stop input to a test double that is permanently "released", in hardware mode as well
   as sim. The entire debounce → latch → `ACTIVE_ERROR` path — which is well written and
   well tested — is dead code on the real machine. Pressing the button changes nothing the
   runtime can see.
2. **Homing can silently latch a wrong home reference on J5.** The hall detector reads a
   cached CAN value that is never invalidated, so the pre-clear guard — the code that
   exists specifically to prevent a wrong reference — is what produces one. A wrong home
   reference shifts every soft limit, every jog block, every collision check and every
   planned pose on that joint, and reports success.
3. **par6 carries two rotation conventions for the same six numbers.** The wire uses
   R = Rz·Ry·Rx; `Robot.fk`/`ik`, the dry-run preview and the frontend's readout all use
   pinokin's intrinsic-XYZ. Teach-and-replay replays a wrist orientation the operator never
   taught — ~20° off for a general pose, and exactly 2×rz off for the everyday tool-down pose.

Beyond those, two subsystems are *declared but not implemented* (`FlashMarker`,
`kt_source = "auto"`), which the repo's own CLAUDE.md forbids, and one fault path is
absorbing: clearing errors on a still-dead node makes that node permanently
un-reportable.

**Minimum bar before power:** all of §2 fixed, plus the three critical test gaps closed
(§5) — a real e-stop line under test, SocketCAN exercised against a vcan interface at
least once, and homing's latched reference checked against simulator ground truth rather
than against itself.

| Severity | Count | Gate |
|---|---|---|
| Critical | 3 | Blocks power-on |
| High | 8 | Blocks power-on |
| Medium | 6 | Blocks trusting unattended operation |
| Low | 5 | Track |

---

## 2. Fix before powering the arm

### C1 — The physical e-stop line is never read

**`crates/par6d/src/daemon.rs:209`** (found independently by the RT-safety and cross-cutting
reviewers, and flagged as the top gap by the test audit)

```rust
let (gpio, _estop_line) = SharedLineGpio::new(true);   // ...then daemon.rs:237: estop: Box::new(gpio)
```

This is the *only* `EstopGpio` constructed outside tests, and it runs unconditionally —
the sim/hardware split at `daemon.rs:263-267` only swaps the bus backend. The `Arc<AtomicBool>`
write handle is bound to `_estop_line` and dropped on the same statement.
`SharedLineGpio::read_estop1` (`crates/par6-rt/src/gpio.rs:53-57`) loads that flag, so
`EstopMonitor::pressed()` (`gpio.rs:129-133`) returns false forever. There is no
libgpiod/lgpio/gpio-cdev/rppal dependency in any `Cargo.toml`, and `SharedLineGpio` is the
only implementation of the trait in the tree — its own doc says "for tests and the
simulated runtime" (`gpio.rs:33-35`).

**Failure.** Hardware bring-up, arm ENABLED and moving. The operator presses the e-stop.
`RtCore::tick` phase 2 (`core.rs:470`) stores `false` into `hw_estop`; `check_errors`
(`core.rs:783-785`) never latches `ErrorCode::Estop`; mode stays JOG/EXEC, state stays
ENABLED, `io()[4]` still reports "released" (`server.rs:1227-1229`), and the RT keeps
emitting cmd-2 position+velocity frames at 250 Hz for as long as the button is held. On
the PAR6 the chain removes motor power while the driver logic stays on the bus, so the
commanded setpoint keeps integrating away from the now-stationary measured position. On
release the drivers re-energise onto a setpoint that is arbitrarily far away and slew to it
at the configured velocity/current limit — an uncommanded full-authority move immediately
after an e-stop, which is precisely the event the vendor's DISABLED + `ACTIVE_ERROR` latch
exists to prevent. The comment at `server.rs:1222-1225` ("Only the e-stop slot is backed by
a real line") is false on hardware.

The vendor reads ESTOP_1 (BCM 5) every single tick, debounced, and ORs it with the software
flag: `rcb-runtime/hardware/gpio_handler.py:50, 228-229, 260-274`, consumed at
`rcb-runtime/RTI.py:1191` and `RTI.py:1780`. par6's own `spec/RT.md:149-158` says to
replicate that exactly.

**Fix.** Implement a real `EstopGpio` for the control box (libgpiod/lgpio, BCM 5,
active-low, ESTOP_2 deliberately unread per `spec/RT.md`), and select it in
`Daemon::start_inner` when `!opts.sim`. Failing to open the line in hardware mode must be a
startup refusal or a loud repeated DEGRADED log — never a silent fall-through to
`SharedLineGpio::new(true)`, because an always-released stub is indistinguishable from a
working line in every field the runtime publishes. `Daemon::start` already refuses to boot
without feature `ffi` (`daemon.rs:129-134`); use the same stance.

> Note the physical chain still cuts power independently of software — that is the actual
> safety function and it is unaffected. What is missing is the runtime's *reaction*, which
> is what stops the post-release move.

### C2 — Hall homing latches the home reference at the backoff point, not at the sensor

**`crates/par6-rt/src/homing.rs:348`** (found independently by three reviewers: rt-safety,
cross-cutting, homing-config)

```rust
HomingStrategy::Hall => (
    JointCommand::hall(trunc_to_wire(speed), HALL_TRIGGER_VALUE),
    node.hall.map(|h| !h.trigger || h.edge).unwrap_or(false),
),
```

`NodeState::hall` is written *only* by a cmd-32 reply (`crates/par6-bus/src/hw/mod.rs:454-459`;
sim `sim/mod.rs:765-771`, which only answers a cmd-31 pack) and is **never cleared** —
not by `Homer::start()` (`homing.rs:261-270`), not by `reset_detectors()`
(`homing.rs:272-279`), not by `HomingSystem::start()` (`homing.rs:740-767`), and
structurally not by `tick`, which takes `state: &BusState` immutably.

The pre-clear guard (`homing.rs:355-366`) sets `preclear_used = true` and enters
`HPhase::Backoff{rehome_after:false}`, whose command is `JointCommand::velocity(-speed)`
(`homing.rs:402`) — a cmd-2 frame. No cmd-32 reply arrives for the whole backoff, so the
stale "hit" survives it. On re-approach tick 1 the stale value reads as a hit, the guard is
now consumed, and control falls straight into
`HomingStrategy::Hall => { self.latched = node.position_ticks; ... }` (`homing.rs:369-375`),
which `tick_home` applies as `conv[j].set_home(latched_ticks, eff_offset[j])`
(`homing.rs:1191`).

**Failure.** J5 is the hall joint (`config/PAR6.toml:386-396`: `strategy = "hall"`,
`speed_ticks_s = 12000`, `backoff_s = 0.3`, `encoder_bits = 14`, `gear_ratio = 10`). Two
reachable triggers, both ordinary:

- **(a) J5 starts on or near its sensor** — exactly the case the guard exists for. Approach
  tick 1 hits → pre-clear → 0.3 s × 12000 = 3600 motor ticks of backoff → re-approach tick 1
  reads the *same* cached hit → reference latched there. Error ≈ 3600·2π/(2¹⁴·10) ≈ 0.138 rad
  ≈ **7.9°**.
- **(b) Any second `home()` in one `par6d` process** — the normal bring-up case where the
  first run fails on another joint, the operator clears, and re-homes. `hall` still holds
  run 1's latched hit, so approach tick 1 fires wherever the sequence's step-6 nudge parked
  the joint. The error is then the whole distance from the parked pose to the backoff end
  point — over a radian.

Nothing downstream catches it: hall skips two-pass (`two_pass` is ANDed with
`strategy == Stall` at `homing.rs:158`), so the mismatch check never runs; `q` is
self-consistent in the wrong frame; the sequence reports `SeqStatus::Complete` with
`homed = true`. Every J5 soft and hard limit is then shifted by that amount, and a J5 move
to its soft limit drives past it into the hard stop under power.

The vendor clears the cached reading in **exactly the two places par6 omits**, and says why:
`rcb-runtime/robotics/homing.py:380-382` (approach tick 1: `motor.HALL_trigger = None;
motor.hall_index = None`) and `homing.py:549-552` at the end of pre-clear, commented
*"Clear stale Hall readings so the first tick of the new approach doesn't immediately
re-trigger on cached CAN data."* par6 ported the guard and the backoff but not the
invalidation the guard depends on. It also requires a *positive* observation before a hit
(`HALL_trigger is not None and == 0`, `homing.py:391-394`), where par6 treats `None` as
"no hit" but a stale `Some` as authoritative.

**Fix.** Invalidate on every approach entry. Either pass `&mut BusState` into
`HomingSystem::tick` and null `state.nodes[node].hall` in `Homer::start()` and at the
`Backoff{rehome_after:false}` completion arm (`homing.rs:405-411`), or keep a per-node
`hall_epoch` on the `Homer` and reject readings older than the current approach. Keep
`None` meaning "no reply yet, not a hit".

**Spec defect (same root):** `spec/HOMING.md:40-44` describes pre-clear as "back off
`backoff_s`, reset, re-approach" and omits the cached-reading clear entirely — the spec
misreads the vendor here, which is how the port lost it.

### C3 — Two rotation conventions for the same six wire numbers

**`crates/par6d/src/kin.rs:184-211` (wire, R = Rz·Ry·Rx)** vs **`python/par6/robot.py:600, 608`**
and **`python/par6/client/dry_run_client.py:122`** (both pinokin intrinsic-XYZ, R = Rx·Ry·Rz).
Found by the python-client reviewer from two independent angles.

par6's wire/STATUS convention is R = Rz·Ry·Rx: `crates/par6d/src/kin.rs:11-15, 184`,
`server.rs:1547-1549` (`pose_matrix_mm`), `python/par6/client/async_client.py:123-140`
(`_matrix_to_pose`). Every Cartesian target resolves through `wire_pose_to_matrix`
(`planner.rs:829`, `planner.rs:1662-1680`, `bridge.rs:309`).

But three surfaces on the same backend use pinokin's intrinsic-XYZ instead
(`pinokin/numba_se3.py:79, 104-124`):

- `Robot.fk` fills `out[3:6]` with `so3_rpy` and `Robot.ik` builds its target with
  `se3_from_rpy` (`python/par6/robot.py:595-611`);
- the dry-run client decodes every wire pose with `se3_from_rpy` (`dry_run_client.py:119-123`,
  fed raw at `:196` move_s/move_p, `:771` move_l, `:815-816` move_c);
- Waldo Commander decodes the STATUS pose **matrix** with pinokin `so3_rpy`
  (`Waldo-Commander/waldo_commander/numba_pipelines.py:87-91`, called from `main.py:432-450`)
  into `commander.status.pose.rx/ry/rz` — and par6 cannot change that decode.

**Failure — teach and replay.** The operator jogs to a pick pose whose TCP rotation is
R = Rz(30)Ry(20)Rx(40) and presses capture-pose. WC's readout shows 30.60 / 33.25 / 13.32,
and the motion recorder reads exactly those scalars
(`Waldo-Commander/waldo_commander/services/motion_recorder.py:98-109`) and emits
`rbt.move_l([x, y, z, 30.600, 33.250, 13.320], ...)` (`motion_recorder.py:316-322`). par6d
rebuilds that orientation as Rz(13.32)Ry(33.25)Rx(30.60) — **19.98° from the taught wrist
orientation**. The gripper is driven into the fixture it was taught to enter cleanly. For
the everyday tool-down pose `[…, 180, 0, rz]` the error is exactly `2·rz` (20° at rz = 10°,
180° at rz = 90°), so this bites ordinary poses, not exotic ones. Only targets with a single
non-zero rotation component are unaffected.

**Failure — offline preview.** `preview.move_l([400, 0, 300, 40, 20, 30])` plans
Rx(40)Ry(20)Rz(30); `rbt.move_l` with the identical literal executes Rz(30)Ry(20)Rx(40) —
26.10° apart. WC renders the 3-D path preview and runs its offline reachability/collision
check through `PathPreviewClient`, which wraps this exact class
(`Waldo-Commander/waldo_commander/services/path_preview_client.py:118`), forwarding args
verbatim. A preview showing the tool clearing a fixture is evidence about a pose the arm
will not adopt.

**Fix.** Unify on **pinokin intrinsic-XYZ**, which is what waldoctl's `Robot.fk`/`ik`
contract, `parol6` and the frontend's matrix decode already use, and which par6 cannot
change on the frontend side. Concretely: change `crates/par6d/src/kin.rs::wire_pose_to_matrix`
to Rx·Ry·Rz and `matrix_to_xyzrpy` to the `so3_rpy` decomposition
(`pitch = asin(r02)`, `roll = atan2(-r12, r22)`, `yaw = atan2(-r01, r00)`), and change
`async_client._matrix_to_pose` (`async_client.py:123-140`) to match. Then state the
convention in `spec/PROTOCOL-V2.md`, which currently does not mention it at all.

Do **not** instead move `Robot.fk`/`ik` to Rz·Ry·Rx — the frontend's decode is fixed, so the
readout would still disagree with `move_l`. `parol6` never splits: `parol6/robot.py:744, 756`
and `parol6/client/async_client.py:877` are all pinokin, which is why WC's readout
round-trips correctly on the reference backend.

> The reviewer's third claimed path — the 3-D gizmo / IK target — was **refuted**:
> `ik_solver.py:129-137` seeds `target_orientation` from `self.robot.fk(...)`, and the
> readout scalars are themselves pinokin-decoded, so that path is self-consistent. The
> teach-and-replay and preview paths stand.

---

### H1 — Clearing an error on a still-dead node makes it permanently un-reportable

**`crates/par6-bus/src/hw/sched.rs:202-206`** (with identical code at
`crates/par6-bus/src/sim/mod.rs:1169-1173` and `crates/par6-bus/src/loopback.rs:522-526`),
consumed at **`crates/par6-rt/src/core.rs:720`**. Found by the rt-safety and
bus/drivers reviewers independently.

```rust
pub(super) fn clear_latch(&mut self, node: NodeId) {
    let n = usize::from(node);
    self.lost_latched[n] = false;
    self.last_rx_tick[n] = None;      // <- erases the observation, not just the latch
}
```

`latch_lost` only iterates nodes where `last_rx_tick[n].is_some()` (`sched.rs:138-146`) and
`classify` maps `None => Freshness::Unknown` (`sched.rs:186-188`). `mark` is the only writer
that returns the slot to `Some` (`sched.rs:150-156`). So for a node that never speaks again,
`None` is an **absorbing state**: it can never become `Stale` or `Lost` again.
`RtCore::freshness_check` (`core.rs:895-905`) is the only recurring producer of
`ErrorCode::CanLost`, and it reacts only to `Stale`/`Lost`. The crate's own unit test pins
the hole: `f.latch_lost(t + 10*lost); assert_eq!(f.classify(3, ...), Freshness::Unknown)`
(`sched.rs:477-478`). `RtCore::begin_clear` calls `clear_lost_latch` for every latched
per-joint key (`core.rs:719-720`).

The same erasure happens robot-wide via `rebase_freshness()` on FLASHING exit
(`core.rs:693`, backends at `sched.rs:209-213`).

**Failure.** A joint's CAN connector works loose, or was never seated at power-on. Node 3
goes silent → `CanLost(3)` latches → `homed` drops → mode `ACTIVE_ERROR`. The operator
presses "clear errors" (the obvious response to a red banner) before fixing the cable. The
0.152 s settle expires, `ErrorManager::tick` wipes the latch (`errors.rs:67-79`), mode
auto-recovers to IDLE, `error_active` reads false and the error list is empty — **and it
stays that way for the life of the process with the joint still dead**. `any_hard()` is now
false so `Enable` is granted (`core.rs:552-556`), and HOMING is reachable (it is not behind
the `needs_home` gate). The runtime will then drive five joints through the full homing
sequence into their hard stops at homing current with the sixth off the bus, reporting a
healthy arm throughout. The boot self-check that would otherwise catch it is a one-shot at
`BOOT_SELFCHECK_TICK` (`core.rs:505-514`) and never re-runs.

JOG/STREAM/EXEC do stay blocked by the `homed` gate, so this is not a free-driving arm —
but the health surface lies permanently, and homing is permitted with a joint off the bus.
Combined with **H2** the gate disappears too.

Secondary consequence: `FreshnessClock::mark` only reports a reconnect edge when
`last_rx_tick` was `Some` (`sched.rs:150-156`), so a node that reboots and comes back after
a clear gets **no `resend_node_config`** and runs on whatever firmware defaults it booted
with — wrong watchdog, wrong Ilim, wrong PD/velocity gains — while the RT commands it.

The vendor does the opposite deliberately: `CAN_node_connection_error[i]` is recomputed
every tick purely from the tick delta to the last frame
(`rcb-runtime/communication/can_message_handlers.py:1015-1031`), and `clear_errors` zeroes
only the latched set, documented as *"Any nodes that are still actually silent will re-set
their `CAN_node_connection_error` within ~50 ticks and re-latch the error automatically"*
(`rcb-runtime/utility/error_checks.py:275-288`). Its FLASHING exit stamps the tick as
"just seen" rather than "never seen" (`RTI.py:1449-1453`), for the same reason.

**Fix.** Make "forget" mean "seen now", not "never seen": in `clear_latch` and `rebase`,
clear `lost_latched` and set `last_rx_tick[n] = Some(current_tick)`. A still-silent node
then re-latches `lost_ticks` later, and a node that comes back still produces a stale→fresh
reconnect edge and its config resend. Apply to all three backends.

Note `spec/CAN.md:150-151` only asks that the user clear reset the *latch*; zeroing the
observation clock alongside it is the implementation's own addition.

### H2 — Firmware flashing never invalidates homing (the flash marker is a dropped flag)

**`crates/par6d/src/daemon.rs:210`**

```rust
let (flash, _flash_flag) = SharedFlashMarker::new();   // write handle dropped on the same line
```

`SharedFlashMarker::flashed` just loads that flag (`crates/par6-rt/src/hooks.rs:473-477`).
The only other constructions in the tree are the three test harnesses
(`tests/common/mod.rs:118`, `tests/homing.rs:39`, `tests/zero_alloc.rs:65`). There is no
marker-file implementation anywhere, so the trait's own doc — "`par6d` wires this to the
flasher's marker file" (`hooks.rs:451-453`) — is false, and the
`if self.flash.flashed()` branch in `RtCore::leave_mode` (`core.rs:694-697`) is unreachable
in the daemon.

**Failure.** The operator parks the arm, asserts PARKED, enters FLASHING, an external
flasher reprograms J2's driver over the now-silent bus, and the mode returns to IDLE. Every
guard that could catch a changed encoder frame is absent: FLASHING entry (`core.rs:627-631`,
`enter_mode` `core.rs:672-675`) gates only on `park_asserted` and does not clear `homed`;
freshness is suppressed while silent (`core.rs:800`) and then wiped by `rebase_freshness()`
on exit (`core.rs:692`, see **H1**), so a rebooted *or bricked* driver produces no
`CAN_LOST`; `sector_done[2]` stays true so `determine_sector` never re-runs
(`core.rs:735-740`); and the encoder is absolute only within one motor revolution
(`convert.rs:114-121, 236-238`). The flash reboots the driver's accumulator, but `conv[2]`'s
home reference and sector are still the pre-flash ones and `homed` is still true — so `q[2]`
is wrong by up to a full sector, `needs_home` passes, and the first JOG/EXEC/STREAM command
drives J2 to a physically different place than requested with soft limits evaluated in the
wrong frame.

Vendor: `rcb-runtime/RTI.py:1463-1478` reads and unlinks `FLASH_PERFORMED_MARKER` on
FLASHING exit and sets NOT HOMED — including in the `OSError` branch, commented *"fail LOUD
and unhomed rather than silently keeping a stale reference"*. It is consumed at startup too
(`RTI.py:571-575`).

**Fix.** Give `par6d` a file-backed `FlashMarker` at the path the flasher writes, consumed
and unlinked on FLASHING exit, with an unreadable or un-unlinkable marker treated as
*flashed* — fail unhomed, never fail homed. Until such a flasher exists, the honest wiring
is a marker that always returns true, so every FLASHING exit costs a re-home. This is the
repo's own "never ship declared-but-unimplemented API surface" rule.

### H3 — `kt_source = "auto"` fetches the driver's kt, logs it as authoritative, and discards it

**`crates/par6-rt/src/core.rs:288-291`**

```rust
let torque_ma_factor: [f64; MAX_JOINTS] = std::array::from_fn(|i| {
    let j = &robot.joints[i];
    torque_to_ma_factor(j.gear_ratio, j.gear_efficiency, j.kt_nm_a, j.dir)  // config kt, built once
});
```

This is built once in `RtCore::new` from the **config** value and is the sole input to both
directions of the torque conversion: commanded Nm→mA in `commit`
(`crates/par6-rt/src/dispatch.rs:195-198`) and measured mA→Nm at `core.rs:757`. A workspace
grep shows `BusState::nodes[n].kt_nm_a` is written by `hw/mod.rs:466` and **read by nothing**
in par6-rt. Meanwhile `SocketCanBus::fetch_kt` (`hw/mod.rs:346-382`) spends up to
0.35 s × 3 retries × 2 rounds per node and logs `node {node}: kt {kt} Nm/A (from driver)`
(`hw/mod.rs:376`) for a value that is then thrown away. `config/PAR6.toml` ships
`kt_source = "auto"`, so this runs on every hardware boot.

`crates/par6-config/src/robot.rs:30-32` and `:185-187` and `spec/CAN.md:119-120` all state
the fetched value governs and config is the fallback for a driver that does not answer.

**Failure.** J1's driver is flashed with kt = 0.20 Nm/A while `config/PAR6.toml` says 0.28 —
precisely the situation `auto` exists for, and the vendor's own `PAR6.xml` carries `<!-- TODO -->`
markers on gripper kt. Boot logs "kt 0.2 Nm/A (from driver)". IDLE gravity hold is
**torque-only** — `law_idle` emits `JointSetpoint::torque_only(g[i])` with no position or
velocity term (`dispatch.rs:78-86`) — so this is an open-loop torque scale error with
nothing closing around it. The driver delivers 0.20/0.28 = **71 %** of the intended torque:
the shoulder sags on enable, EXEC torque feedforward under-tracks by 29 %, and the reported
`tau` telemetry (`core.rs:757`) is inflated by 1.4× — the readout moves in the reassuring
direction. If the mismatch runs the other way the arm actively lifts. No fault, no mismatch
check, and a log line asserting the opposite.

Vendor: `RTI.py:545-549` rebinds `motor_kt` from `resolve_and_preload_kt(...)` and
`RTI.py:723-728` passes that resolved array into `build_torque_to_ma_factor`
(`utility/mode_dispatch.py:557-562`), which multiplies every commanded torque
(`mode_dispatch.py:212-226`). It also publishes per-joint provenance —
`motor_Kt_source`, 1 = CAN, 0 = XML (`utility/kt_init.py:31-33, 63-72`) — so the GUI shows
which one is live.

**Fix.** After `boot_configure` and the first `drain_rx` that publishes `boot_state`
(`hw/mod.rs:492-494`), and before the arm can be enabled, recompute `torque_ma_factor[i]`
from `bus_state.nodes[node_of[i]].kt_nm_a` when `robot.kt_source == KtSource::Auto` and the
node answered; fall back to config otherwise. Publish per-joint provenance in the snapshot.
If the intent is genuinely always-config, delete `KtSource::Auto`, the `fetch_kt` call and
the log line — do not ship a 2.5 s boot step that changes nothing.

> The sim cannot expose this: `VirtualDriver::new` is seeded with `j.kt_nm_a`
> (`sim/mod.rs:1021-1027`) so the simulated cmd-33 reply is the config value by
> construction, and `tests/sim_bus.rs:745-752` asserts exactly that tautology.

### H4 — A gripper calibration timeout locks the runtime out of homing until restart

**`crates/par6-rt/src/core.rs:867`** with **`crates/par6-rt/src/homing.rs:1252`** and
**`homing.rs:746`**

`cal_failed` is set on the 10 s calibrate timeout (`homing.rs:1252`) and cleared in exactly
one place — `HomingSystem::start()` (`homing.rs:746`). Neither `abort()` (`homing.rs:772-780`)
nor `fail()` (`homing.rs:841-845`) touches it, and `RtCore::begin_clear` (`core.rs:711-734`)
never reaches the homing subsystem. `check_errors` re-asserts the key unconditionally every
tick (`core.rs:867`), `GripperCalibrationFailed` is absent from `ErrorCode::is_warning()`
(`state.rs:101-109`) so `any_hard()` is true, and `request_mode` refuses every non-IDLE
target while `any_hard()` (`core.rs:636-638`) — including `Mode::Homing`, whose `enter_mode`
is the sole caller of `homing.start()` (`core.rs:653`). The latch that must be cleared can
only be cleared by the path the latch blocks.

**Failure.** First hardware bring-up with the MSG gripper unpowered, jaws jammed, or on the
wrong node id. Sequence step 4 (`[homing.sequence.home] gripper = "firmware"`) sends cmd 62
and polls; the calibrated bit never returns; after 10 s (2500 ticks) the hard latch is set
and the sequence fails. The operator presses Clear Errors: the latch is wiped in phase 9 and
re-latched in phase 7 of the very next tick. From then on `Enable`, `SetMode(Homing)` and
every motion mode are refused **forever**. The one-tick window the phase ordering opens is
not exploitable — the loop polls at most one command per tick (`core.rs:472-474`), so the two
commands needed to reach HOMING cannot both land inside it. There is no `RtCommand` that
rebuilds the core; restarting `par6d` is the only recovery, with no diagnostic beyond an
error key the operator has already cleared.

Vendor: `rcb-runtime/robotics/homing.py:910` writes `STATUS_FAILED` into a status array that
the next `start()` zeroes (`homing.py:1435-1438`). There is no sticky hard-error key
re-asserted from homing state.

**Fix.** Clear `cal_failed` wherever the sequence stops, not only where it starts: reset it
in `HomingSystem::abort()` and `fail()`, or expose `HomingSystem::clear_faults()` for
`RtCore::begin_clear()` to call. Alternatively latch the error on the rising edge inside
`tick_home` rather than level-holding it from a flag `check_errors` re-reads every tick.

This is fail-closed — nothing moves — which is why it is High and not Critical. But it is an
undiagnosable lockout on the exact configuration a first bring-up runs. See also **M2**,
which routes an operator straight into it.

### H5 — Stream preemption drains the whole command socket, destroying buffered `estop`

**`crates/par6-server/src/server.rs:1103-1112`**

```rust
fn drain_backlog(&self) {
    let mut buf = [0u8; 2048];
    while self.socket.try_recv_from(&mut buf).is_ok() { n += 1; }   // no tag inspection
}
```

Called on a stream **type change** (`server.rs:688`) and from the teleport branch
(`server.rs:667`). `self.socket` is the single command socket serving every client and every
command class — `command_class` (`crates/par6-proto/src/enums.rs:252-263`) routes
`Estop`/`Stop`/`Reset`/`ResetState`/`SetShapes` over it — so a SYSTEM datagram already in the
kernel queue is consumed and discarded with no reply and no effect. `peek_tag` is already
used on the same socket 770 lines earlier (`server.rs:331`), so selective draining is
available and simply not used.

The loss is unrecoverable at the client: `_system` sends exactly once
(`python/par6/client/async_client.py:632-641`, `attempts = 1`) and returns 0 after a 1.0 s
timeout, and WC's `on_estop_click` discards that return and shows the E-STOP dialog anyway
(`Waldo-Commander/waldo_commander/components/control.py:1845-1848`).

**Failure.** The operator jogs joints (WC streams `jog_j`), switches to the Cartesian panel
and drags (first `jog_l`), and clicks the software E-STOP while the server task is
descheduled or planning. Both datagrams are queued. The server reads `jog_l`, takes the
type-change arm, and the `estop` is read and thrown away: `estop_latched` stays false, the
RT is never disabled, no standing `SYS_ESTOP_ACTIVE` appears — while the UI reads E-STOP
ACTIVE over a moving arm. The same window destroys `stop`, `reset_state`, `select_profile`
and `set_shapes`.

For a single client the window is sub-millisecond per stream switch. It widens to nearly
every datagram when **two clients stream different types** (WC jogging while a script
servos), and the drain also destroys a supervisor process's `stop()` from a client that
never sent a stream at all. `crates/par6-server/tests/protocol.rs:823-830` already proves
buffered datagrams are destroyed by this path — its own comment says "all buffered before
the server reads".

`spec/PROTOCOL-V2.md:65` only asks that a type change "cancels the active streamable, drains
the socket backlog" — the previous *stream's* backlog, not everyone's traffic.

**Fix.** Read pending datagrams into a local `Vec<(Vec<u8>, SocketAddr)>`, `peek_tag` each,
discard only those whose tag is the stream type being replaced (or any `is_stream` tag), and
feed the survivors back through `on_command_bytes`. Or drop the drain entirely — the RT
applies newest-only setpoints already, so stale same-type jogs are harmless, and
`cancel_stream()` plus the type-change bookkeeping is what actually enforces preemption.

> `parol6` has the same blunt `drain_buffer()`
> (`parol6/server/transports/udp_transport.py:143`, called at `controller.py:683`) — the
> pattern was inherited rather than reasoned about. par6 makes it worse in one respect:
> SYSTEM commands are never retried.

### H6 — A wire-reachable `jog` duration aborts the whole daemon

**`crates/par6d/src/bridge.rs:263`** (and identically `bridge.rs:369` for `jog_l`)

```rust
deadline: Instant::now() + Duration::from_secs_f64(p.duration),
```

The only validation is `crates/par6-proto/src/command.rs:692-695`, which checks `finite(v)`
and `v > 0.0` with **no upper bound**. `Duration::from_secs_f64` panics above ~1.84e19 s;
`Instant + Duration` panics above ~9.2e18 s. Both were reproduced on the workspace toolchain.
The ordering inside the JogJ arm is fatal: `enter_stream_mode(Mode::Jog)` (`bridge.rs:251`)
and `RtCommand::Jog` (`bridge.rs:256-259`) are already queued to the RT, and the panic
happens while the `shared` lock taken at `bridge.rs:243` is held. `[profile.release]
panic = "abort"` (`Cargo.toml:47`) is what `scripts/deploy/build-aarch64.sh:56` builds.

**Failure.** Any UDP peer that can reach the command port — it binds `0.0.0.0` with no auth
(`crates/par6-server/src/config.rs:172-176`) and the Python client passes `float(duration)`
straight through (`async_client.py:1170`) — sends `jog_j(joint=0, speed=0.5, duration=1e30)`.
The gate passes, the jog reaches the RT, then `par6d` aborts. Process abort stops all CAN TX,
so the joint does not keep ramping; the drivers hold the last frame and go Idle after the
5000 ms watchdog (`spec/CAN.md:120`), i.e. the arm sags rather than runs away. In a
debug/unwind build it is worse: the tokio task dies leaving `self.shared` **poisoned**, so
`housekeeping_loop` (`bridge.rs:567`), `RtBridge::halt` and `cancel_stream`
(`bridge.rs:394, 399`) all panic too — the jog watchdog never fires and nothing can stop the
jog except a soft limit.

The same missing bound has a non-panicking sibling that is arguably worse: `duration = 1e9`
sets a jog watchdog deadline ~31 years out, so **one datagram jogs until the soft-limit
block**, defeating the watchdog with no panic at all.

**Fix.** Bound the duration in the codec where every other range check lives: in
`command.rs::duration()` add an upper bound (e.g. `v <= 3600.0`) so it is rejected with
`COMM_VALIDATION_ERROR`. Belt-and-braces at the bridge:
`Duration::try_from_secs_f64(p.duration).ok().and_then(|d| Instant::now().checked_add(d))`
with a capped fallback. Both sites.

### H7 — Joint blending drops a corner but still trims the next segment

**`crates/par6-motion/src/cart.rs:772`** (mirrored at `python/par6/motion.py:879`)

```rust
let (a, b) = (entry[i], 1.0 - exit[i]);          // cart.rs:767 — head trimmed unconditionally
...
if i + 1 < n - 1 && exit[i] > 0.0 {              // cart.rs:772 — corner emitted only on exit
```

`exit[i]` and `entry[i+1]` come from `fracs[i].0` / `fracs[i].1` (`cart.rs:754-757`), which
`Par6Planner::start_joint_chain` computes from **two independent TCP distances**, each zeroed
by its own `> 1e-9` guard (`planner.rs:1140-1150`). So a corner whose incoming segment has
zero TCP length drops the Bézier while still skipping the start of the outgoing segment,
leaving a hole in the waypoint list with no samples in it. `parol6`, the reference this was
ported from, guards on **both**: `parol6/motion/geometry.py:627-628` and `:660-661` use
`exit_frac[blend_idx] > 0 or entry_frac[blend_idx] > 0` in both the precompute and emission
loops. par6's port kept only the `exit_frac` half. The Cartesian sibling
`blended_polyline` does not have the bug — it guards on `clamped[i] > 0.0` (`cart.rs:641`)
and derives both trims from that same value (`cart.rs:579-589`), so they cannot disagree.

**Reachability.** The URDF puts `tcp` at xyz = (0, −0.0, −0.14) in the `gripper` link and
`gripper_JOINT` (J6) has axis (0, 0, −1) in that same frame
(`assets/par6_description/URDF/par6_msg_gripper/urdf/PAR6_MSG.urdf:387-388, 503, 524-527`) —
the TCP sits exactly on J6's rotation axis, so a wrist-roll-only `move_j` leaves the TCP
bitwise fixed. `current_pose` is a bare `kin.fk` with no offset by default
(`planner.rs:856-860`). Two identical consecutive `move_j` targets in a chain hit it too.

**Failure.** `move_j(A, blend_radius=30) ; move_j(B, blend_radius=30) ; move_j(C)` where
A→B is a pure J6 roll and B→C moves the shoulder. At corner B: `before = 0` → `exit = 0`;
`after = 0.05` → `entry = 0.5`. Running the algorithm verbatim gives a largest consecutive
step of **0.2620 rad — 5.2× the 0.05 rad `CART_STEP_RAD` pitch the function promises**
(`planner.rs:101, 1155`), jumping straight from B to the midpoint of B→C. That list goes to
`toppra_samples`, and `par6_traj_create` fits its cubic spline over
`Vector::LinSpaced(n_way, 0.0, 1.0)` knots with natural BCs (`cpp/src/par6_traj.cpp:124-129`),
so the outsized interval gets the same parameter width as its 0.05 rad neighbours. Evaluating
that spline, joint 0 **undershoots to −0.0273 rad against a commanded range of 0…0.524 rad** —
1.6° outside the commanded envelope, ~12 mm of TCP travel in a direction never commanded.
Scaling up: a 1.0 rad outgoing segment undershoots to −0.053 rad (−3.0°), a 2.0 rad one to
−0.107 rad (−6.1°). `start_joint_chain` checks soft limits only on the chain **waypoints**
(`planner.rs:1126-1128`), never on the TOPPRA samples, so the excursion is not re-checked.

par6-motion's own `spline()` docstring (`cart.rs:411-426`) names uniform knots over unevenly
spaced waypoints as exactly the failure mode to avoid; this bug manufactures that spacing.

**Fix.** `if i + 1 < n - 1 && (exit[i] > 0.0 || entry[i + 1] > 0.0)`, building the Bézier
from whichever ends are non-zero (`e` degenerates to `waypoints[i+1]` when `exit[i] == 0`,
`x` likewise when `entry[i+1] == 0`) — what `parol6` does. Or make `start_joint_chain`
(`planner.rs:1140`) zero **both** fractions of a corner whose incoming or outgoing TCP
distance is degenerate. Mirror at `python/par6/motion.py:879`.

The only existing test uses symmetric `(0.25, 0.25)` fractions (`cart.rs:1085`) and cannot
reach this branch.

### H8 — Keep-out shapes are enforced in a different orientation from the one drawn

**`cpp/src/par6_col.cpp:60-67`** (used for every shape placement at `:354-357`), with the
same claim repeated in `cpp/include/par6_shim.h:233-235` and
`crates/par6-kin/src/shapes.rs:93-95`.

```cpp
/* R = Rx(rx) * Ry(ry) * Rz(rz) — the intrinsic-XYZ convention the pose
 * readback (par6_kin fk -> tcp rpy) and waldoctl's Shape.pose share. */
```

Both halves of that comment are wrong. waldoctl's `Shape.pose` contract is **extrinsic-XYZ,
R = Rz·Ry·Rx**, and the two other implementations of it both use that: `parol6`'s
`_pose_to_matrix` (`parol6/PAROL6_ROBOT.py:250-258`), whose docstring says *"Deliberately NOT
`pinokin.se3_from_rpy` (Rx·Ry·Rz) — swapping it in would mis-orient any multi-axis-tilted
shape versus every other implementation of the contract (including the frontend's
renderer)"*; and WC's renderer, which calls
`Object3D.rotation_matrix_from_euler(s.pose[3], s.pose[4], s.pose[5])`
(`Waldo-Commander/waldo_commander/services/urdf_scene/urdf_scene.py:84-86`) whose default
`order='XYZ'` is documented as `M = Rz @ Ry @ Rx`
(`nicegui/elements/scene/scene_object3d.py:193-196`). And the stated rationale fails inside
par6 too — par6's own wire/STATUS pose readback is Rz·Ry·Rx (`crates/par6d/src/kin.rs:11-15`,
`server.rs:1547-1549`), not intrinsic-XYZ.

The pose reaches the shim untouched: `set_shapes` forwards `Shape.to_wire()` verbatim
(`python/par6/client/async_client.py:1339-1353`) and `Shape::from_proto` copies it with no
conversion (`crates/par6-kin/src/shapes.rs:165-167`).

**Failure.** A program declares a tilted guard:
`Box(name="guard", x=1.0, y=0.02, z=1.0, pose=(0.4, 0.0, 0.3, 0.0, radians(45), radians(90)))`.
WC draws the slab at Rz(90)Ry(45); par6d's coal model places it at Ry(45)Rz(90) — **62.80°
apart**. The drawn guard's face normal and the enforced guard's face normal point in
different directions, so `gate_collisions` (`planner.rs:429-531, 554`) lets moves through the
volume the operator fenced off and blocks moves through empty space. The comment at
`urdf_scene.py:82-83` states the intent this breaks: *"so what is drawn is what the checker
enforces"*.

**Fix.** `AngleAxisd(rz,Z) * AngleAxisd(ry,Y) * AngleAxisd(rx,X)` in `par6_col.cpp:62-67`,
and correct the doc comments in `par6_shim.h:233-235` and `shapes.rs:94-95`. **This is
independent of C3** — `Shape.pose` is a separate contract with an existing cross-implementation
answer, so it should become Rz·Ry·Rx regardless of which convention par6 settles on for TCP
poses. While there, put the convention in waldoctl's `ShapeBase.pose` docstring so a fourth
implementation cannot guess again.

---

## 3. Fix before trusting it

### M1 — Stall displacement threshold is not scaled for the slow second pass
**`crates/par6-rt/src/homing.rs:181`** — `stall_disp_ticks` is computed once from the
unscaled config speed, while pass 2 commands `p.speed * REHOME_SPEED_FACTOR` (0.3)
(`homing.rs:336-340`) and `detect_stall` (`homing.rs:287-301`) compares against the pass-1
value with no reference to `self.pass`. The gate is **3.33× too permissive on pass 2**: a
joint still travelling at 83 % of commanded speed counts as stalled, where the vendor's
scaling requires ≤ 25 %. Vendor recomputes every tick from the pass-scaled speed
(`rcb-runtime/robotics/homing.py:376-377, 409-411`). Consequence is bounded by the backoff
excursion, not by the acceptance window: on **J0 only** (backoff 1350 ticks < 
`two_pass_max_diff_ticks` 3500) a false pass-2 stall is silently accepted, latching a
reference up to ~1170 ticks ≈ **4.0°** off. On J1–J4 the backoff exceeds `max_diff`, so the
same event is a loud homing failure instead. **Fix:** derive the threshold per tick from the
speed actually commanded — move it out of `HomerParams`.

### M2 — Selecting the bare flange makes `par6d` refuse to start
**`crates/par6-config/src/lib.rs:173`** — `ConfigBundle::validate` hard-rejects any bundle
whose homing sequence names the gripper while the active tool has no `[driver]` (and again
for no `[homing]`). `config/PAR6.toml`'s sequence has `gripper = "firmware"` and
`gripper = "motor"` steps; `config/grippers/Flange.toml` correctly has neither, matching
vendor `Flange.xml` `CAN_gripper = 0`. `ConfigBundle::load` is the daemon's first real action
(`daemon.rs:139`), so this is fatal before CAN is opened. Everything downstream already
handles the driverless tool (`core.rs:283`, `homing.rs:624-632`, `daemon.rs:534`), and
`server.rs:799-802` tells the operator to "change `robot.active_gripper` and restart par6d" —
the one thing this validator forbids. **Failure:** bringing the arm up bare, the safest
possible first power-on and the reason `Flange.toml` exists with its own
`arm_joint_home_offsets` override (index 4 → −2.258 rad vs MSG's −2.070), forces the operator
to hand-edit the shared sequence — exactly the hand-transcription this config file exists to
eliminate — or to run with MSG selected and apply the wrong home offset to joint 4. Vendor
skips both gripper modes with a logged warning (`rcb-runtime/robotics/homing.py:1499-1503,
1547-1552, 1560-1570`). **Fix:** drop the two rejections *and* guard `Part::HomeStart`'s
`Firmware` arm (`homing.rs:1031`) on `self.has_can_gripper` as the `Motor` arm already is
(`homing.rs:1019-1030`) — relaxing only the validation runs the calibrate step into its 10 s
timeout and straight into **H4**.

### M3 — Unbounded pre-allocation from a msgpack length header
**`crates/par6-proto/src/command.rs:1047-1053`** — `r_waypoints` does
`let n = r.array_len()?; let mut out = Vec::with_capacity(n);` before reading any element,
and `Reader::array_len` (`wire.rs:176-184`) accepts the 0xDD form up to `0xFFFF_FFFF` with no
cross-check against bytes remaining. Same pattern at `command.rs:1032` (`r_vec_f64`, used by
`teleport.tool_positions` and every shape's params/pose), `command.rs:1153` (`SetShapes`),
`command.rs:1309` (`tool_action.params`), and on the reply/status side at `reply.rs:588`,
`reply.rs:606`, `status.rs:342`. The nine bytes `98 55 00 00 DD FF FF FF FF` to UDP 6001
decode as a valid MOVE_S envelope (arity 8 passes the gate at `command.rs:1104-1111`) and ask
the allocator for 206 GB → `handle_alloc_error` → **abort**, taking the RT thread and CAN
traffic with it. Medium rather than high only because the protocol has no authentication at
all, so anyone who can reach the port can already command motion. **Fix:** expose `remaining`
on `Reader` and bound every length-prefixed allocation by `remaining / MIN_ELEM_BYTES`, or
drop `with_capacity` for `Vec::new()` + `push`. Also cap waypoint and shape counts in
`Command::validate` — the reassembler admits 4 MiB (`chunk.rs:23`), ~73 k waypoints of
planner work.

### M4 — A commanded full circle can collapse to a sub-millimetre nudge
**`crates/par6-motion/src/cart.rs:381`** (mirrored at `python/par6/motion.py:604`) —
`circle_through` selects the full-circle branch on a **positional** threshold
(`norm(p3 - p1) < FULL_CIRCLE_M`, 1 mm, `cart.rs:297`), then `arc()` throws that decision away
and re-derives the sweep, forcing TAU only when `sweep < 1e-6` — an **angular** threshold
four orders of magnitude tighter (0.1 µm on a 100 mm circle). Everything in the gap falls
through to `else if dot(cross(u1,u2), circle.normal) < 0.0`, whose sign is decided by the
direction of the sub-millimetre discrepancy. Verified numerically on a 100 mm circle:
end = start exactly → 360.0000°; +0.05 mm → 0.0286°; +0.3 mm → 0.1719°; +0.9 mm → 0.5156°;
the mirrored offsets give 359.97 / 359.83 / 359.48°. `start_move_c` builds `start_pose` from
FK of the **measured** q (`planner.rs:988-993`), so the discrepancy is exactly the arm's
settle error (tolerance alone is 0.01 rad/joint), and the code's own comment
(`cart.rs:34-37`) anticipates ~0.1 mm round-trip error — which yields 0.057°, four orders
above the gate. The 0.3 mm arc still clears `start_cart_path`'s `moved` test
(`planner.rs:907-913`, `MOVE_L_NULL_M = 1e-6`), so par6 executes the nudge, compresses the
whole commanded reorientation into it, and reports **COMPLETE**. A dispensing or weld program
sequencing on that completion carries on as if a 628 mm circular path had been traced.
Motion *deficit*, not unexpected motion, hence Medium. **Fix:** make the sweep follow the
branch decision — return `full_circle: bool` from `circle_through`, or re-test the same
positional predicate in `arc()`. Note the second condition at `cart.rs:381`,
`norm(sub(r1,r2))`, is algebraically `|p_start − p_end|` — the quantity `circle_through`
already tested — so it is redundant and `sweep < 1e-6` is the only binding gate.

### M5 — Un-homed jog is permitted by the command plane and silently refused by the RT
**`crates/par6-server/src/gating.rs:21`** — the doc says "jogging stays available un-homed"
and `gate()` leaves `needs_homed` false for `JogJ`/`JogL`/`ServoJ`/`ServoJPose`/`ServoL`
(`gating.rs:36-39`), so `on_faf` accepts them. But `RtCore::request_mode` refuses
(`core.rs:639-643`, `GateRefusal::NotHomed`), the refusal is only logged
(`core.rs:537-540`), and `RtCommand::Jog` is then dropped because `apply_command` requires
`self.mode == Mode::Jog` (`core.rs:565-570`). Because these are FIRE_AND_FORGET, nothing
reaches the client; the only trace is the `NOT_HOMED` key, which `is_warning()` classifies as
a warning (`state.rs:97-108`), so `error_active` stays false and `rt_standing_error` returns
`None` (`faults.rs:42-44`). **Failure:** first thing an operator does on bring-up — press jog.
The client returns success, STATUS shows a healthy idle arm, and nothing moves, with no
message saying why. **Provenance matters here:** the gating comment is `parol6`'s policy
(`parol6/commands/base.py:23-33`) where it genuinely works, while par6's RT correctly follows
the vendor, which requires homed for JOG (`rcb-runtime/utility/state_machine.py:37-44,
173-174`) and rejects the request **loudly** (`communication/protocol.py:54`,
`command_executor.py:818-822`). So the RT is right and the command plane is the wrong side of
the mismatch. **Fix:** set `needs_homed = true` for the jog/servo commands in `gate()` so the
server answers `MOTN_NOT_HOMED` — the gating module already guarantees rejections reach the
client even for fire-and-forget (`gating.rs:8-10`, `server.rs:654-660`) — and fix the
`gating.rs:21` comment.

### M6 — The RT thread logs synchronously, unthrottled, at 250 Hz during a bus fault
**`crates/par6-rt/src/core.rs:1063-1068`** (`joint TX failed` / `gripper TX failed`),
**`core.rs:739-741`** (`bus RX drain failed`), plus `core.rs:957-960` in the HOMING dispatch
arm and `crates/par6d/src/adapters.rs:112, 143` inside `MotionStream`. No throttle, no
once-flag, no edge detector. Both error sources are permanent while the link is down —
`SocketCanBus::send` maps ENETDOWN/ENOBUFS to `LinkDown`/`TxQueueFull`
(`hw/mod.rs:200-211`), `recv` maps any non-`WouldBlock` to `LinkDown` (`hw/mod.rs:238-241`) —
and the tick keeps commanding in `ACTIVE_ERROR`, so the storm does not self-limit. The thread
is genuinely `SCHED_FIFO` 99 on hardware (`crates/par6-rt/src/rt.rs:31`, selected for
non-sim at `daemon.rs:358-362`) and `env_logger` writes to stderr (`main.rs:28`), so each
record takes the writer lock and issues a `write(2)` — ~750 records/s. Under systemd that
stderr is a pipe to journald; if journald stalls, `write(2)` **blocks the priority-99 RT
thread** and the tick loop stops rather than degrading. Even without blocking, the per-tick
lock + syscall pushes p99 past the critical band and latches `LOOP_CRITICAL`, attributing the
fault to loop health rather than to the bus. Contradicts the repo's own rule ("The RT tick
path allocates NOTHING after init … no formatting except one-shot error paths") and
`core.rs:26-27`. `zero_alloc.rs` cannot catch it — it drives the sim happy path with
`RampJog`/`ClampStream`/`ZeroGravity`/`NoFk`, not the hooks `par6d` wires. **Fix:** count bus
TX/RX failures into the existing `LinkHealth`/`LoopStats` snapshot fields and let the command
plane log off-thread, or gate the RT-side `warn!` behind an edge detector plus a rate limiter.

---

## 4. Noted

| # | Location | Defect | Consequence |
|---|---|---|---|
| L1 | `crates/par6-bus/src/sim/dynamics.rs:28` | `VISC_RATE = 8.0` is documented as matching the kinematic plant's damping, but `plant.rs:22`'s `VISC` is 2.0 in the same units (τ = 125 ms vs 500 ms). The inertia factor cancels through ABA, so the two plants damp **4× differently**; `plant.rs:16-22` derives 2.0 explicitly against the stall-current threshold, and nothing re-ran that check at 8.0. | Sim-only, and `sim-dynamics` has no CI job (`.github/workflows/ci.yml` builds only `sim-mujoco`, `:93-97`). A mistuned optional tier. Either the constant or the comment is wrong; fix one. |
| L2 | `crates/par6-rt/src/homing.rs:866` **(UNCERTAIN)** | `PreMove::Idle` emits `JointCommand::idle()` = `velocity(0,0)` (`types.rs:98-102`) — byte-identical to the keep-alive `tick()` already wrote to every slot at `homing.rs:963`, so the vendor's `<idle>` pre-move degenerates to a pure delay. `encode_idle` (cmd 12) exists (`spectral/codec.rs:495-498`) but no `Pack`/`JointCommand` variant can emit it. Vendor sends `Send_Idle()` then encoder polls (`rcb-runtime/robotics/homing.py:1668-1681`). | J1/J2 stay weakly energised at 250 mA for 4 s instead of going limp. Whether the wound velocity integral survives into step 1 depends on real StepFOC firmware behaviour, not checkable from source — hence UNCERTAIN. A config knob (`kind = "idle"`) in a file claiming step-for-step vendor fidelity that behaves as a no-op. |
| L3 | `crates/par6-server/src/server.rs:1123-1132`, `:1408` | `link_ok` / `data_age_ms` measure the age of the **server's RT snapshot**, not the motor bus — the RT publishes every tick unconditionally, so both read healthy while the arm is silent. `Ping.hardware_connected` inherits it. The real signal, `StateSnapshot.nodes[i].data_age_ticks` (`crates/par6-bus/src/types.rs:291-292`), is carried in the same snapshot and ignored. Wire doc says "motor bus link" (`status.rs:45-48`); server doc says "snapshot staleness" (`lib.rs:29-31`). `parol6` requires `first_frame_received` (`parol6/server/controller.py:323-327`). | A wrong boolean sitting next to a loud, correct error: the boot self-check latches `CanLost` for every joint (`core.rs:504-513`), which forces DISABLED + `ACTIVE_ERROR` and gates all motion. Diagnostic defect. The WC auto-failover path is unreachable (`hardware_connected` requires `!simulator`). |
| L4 | `crates/par6-proto/src/error.rs:296-301` | `SYS_RTI_LINK_LOST` says effect "robot held" and remedy "reclaim the session". It is a **hard** latch (absent from `is_warning()`, `state.rs:99-110`; latched `core.rs:855`) → DISABLED + `ACTIVE_ERROR`, and there is **no claim/reclaim verb anywhere** in the v2 taxonomy (`enums.rs:33-137`) — the phrase is imported from the vendor's RTI session lifecycle (`rcb-runtime/RTI.py:1048`) that par6 deliberately dropped. Siblings `SYS_EXEC_LINK_LOST` (`:290-295`) and `SYS_LOOP_CRITICAL` (`:302-307`) both say "then send reset". | Misdirects only a client that streams and never queues — the next queued command returns `SYS_CONTROLLER_DISABLED`, whose remedy is correct (`error.rs:260-265`). Recovery time, not metal. |
| L5 | `crates/par6-server/src/server.rs:583-588` | A `reset` refused for exceeding `MAX_RESET_WAITERS` (16) answers `COMM_QUEUE_FULL`, whose template describes the **motion queue** ("Wait for queued motions to finish", `error.rs:224-229`) — the same code used correctly at `server.rs:725-729`, so clients cannot distinguish them. Waiters do accumulate: each `reset` calls `set_enabled(true)`, installing a fresh 5 s deadline (`bridge.rs:417-425`), while the Python client's 1 s single-attempt `_system` (`async_client.py:324, 632-641`) paces retries at ~1 Hz. | At ~16 s of retrying on an e-stopped arm, the operator is pointed at an empty motion queue instead of at the e-stop button or the latched drive fault. **Fix:** add a `{detail}` slot, or reuse `SYS_CONTROLLER_DISABLED`. |

---

## 5. Test plan

The suite is genuinely strong and mostly free of theatre: the RT core is driven through
virtual ticks at the shipped 4 ms period over a real closed-loop driver/plant sim, the CAN
codec has 48 golden vectors plus 14 malformed ones with two-way coverage enforcement, the
SPSC ring and triple-buffer snapshot have real cross-thread race tests, alloc-free contracts
are asserted with counting allocators in four places, and `par6d` is driven end-to-end over
real UDP with the real protocol codec. Several tests are clearly regression-derived from real
bugs (`core_errors.rs:332`, `exec_playback.rs:214`, `sim_session.rs:676`).

What it does not cover is a coherent set: **everything between the RT core and the metal.**

### 5.1 Theatre — delete or narrow

| File:line | Problem | Action |
|---|---|---|
| `python/tests/test_scaffold.py:3` | `assert par6.__version__` — a bare non-empty-constant assertion on a package attribute, the repo's own listed example of a tautological test. Cannot fail except on import error, which every other test covers. | **Delete**, or replace with a real drift check (`importlib.metadata.version("par6")` vs `pyproject.toml`). |
| `crates/par6-server/src/link.rs:160` | `link.errors = 0; assert!(link.unicast);` under "Permanent: further successes never switch back" — no send is performed and `unicast` is only written by `note_send_failure`, so nothing could have flipped it. The preceding `link.errors = 0; // what a successful send() does` (`:151`) pokes a private field instead of entering through the real path. | **Rewrite:** drive an actual successful `send()` on the socket it already binds, so the reset is the code's, not the test's. |
| `crates/par6-rt/tests/homing.rs:112` | `full_par6_sequence_homes_closed_loop_to_the_ready_pose` asserts `s.q` against the ready pose, but both the `move_to` commands and `s.q` convert through the **same** `JointConversion` that `set_home` re-based moments earlier — the assertion holds for essentially any latched tick. The docstring's stronger claim ("a wrong gripper-dependent offset would miss these targets") is subject to the same cancellation. | **Keep but narrow the claim** (it does prove the sequence completes, FSMs reach Done, and current limits swap and restore). Then close gap G3 so the reference itself is checked. |
| `crates/par6d/tests/ffi_kinematics.rs:660` | `gravity_hook_holds_the_arm_on_the_torque_plant` opens "The gravity hook does physical work", but the plant is built from the **same URDF** the gravity model reads (`daemon.rs:249`) — it measures internal consistency. It would pass unchanged if every link mass were halved, which is close to the discrepancy actually present. | **Keep, narrow the comment** (it does prove wiring, sign convention and the Nm→mA round trip). Pair with G5's external-reference assertion. |

### 5.2 Gaps — ranked (offline)

**Critical**

- **G1 — e-stop wiring.** Nothing asserts the shipped runtime reads a physical line at all
  (see **C1**). The suite gives maximal false confidence: production installs the same test
  double the tests use, so "e-stop is thoroughly tested" is true and irrelevant.
  *Test:* factor hook selection into `fn estop_source(opts) -> Result<Box<dyn EstopGpio>, DaemonError>`
  and assert `sim = false` does **not** return the shared-flag double and errors when the
  chardev line cannot be opened.
- **G2 — SocketCAN backend.** The only bus that will ever touch the arm.
  `crates/par6-bus/tests/socketcan_vcan.rs:14` says outright it is "developer / bring-up
  coverage, not a CI gate"; all four tests call `require_vcan!()` (`:207, 283, 344, 463`) and
  skip; `.github/workflows/ci.yml` never creates a vcan interface; repo task #17 is still
  open. The paced boot config load, kt fetch, bus scan, kernel RX timestamps, EWOULDBLOCK
  drain and per-tick frame budget have plausibly **never executed**. A silent TX-queue
  overflow during the ~170-frame boot config load means some drivers never receive their
  Limits/gains and run last-flashed defaults — wrong current limits and wrong PID gains on a
  closed-loop torque machine.
  *Test:* a privileged CI job doing `modprobe vcan && ip link add dev vcan0 type vcan && ip link set vcan0 up`,
  with `require_vcan!()` hard-failing when `PAR6_REQUIRE_VCAN=1` so the job cannot silently
  degrade to a no-op.
- **G3 — homing home reference vs ground truth.** No test can compare the runtime's frame
  against the simulated arm's true frame; `SimBus` exposes no ground-truth accessor, and the
  sim's mechanical bounds are in the boot frame (`sim/plant.rs:192-201`) so only a >±49 k-tick
  error would surface. This is the single highest-consequence number in the system — every
  soft limit, jog block, collision check and planned pose is measured against it — and it is
  exactly what **C2** and **M1** corrupt.
  *Test:* add `SimBus::true_joint_rad()` (no `report_offset`); after `SeqStatus::Complete`
  assert `|core.snapshot().q[i] − bus.true_joint_rad()[i]| <= one encoder tick`, from several
  `set_initial_joint_rad` boot poses including ones forcing a nonzero boot sector shift.
- **G4 — stall false-positive guards.** Nothing drives a joint that draws high current at
  spin-up or plateaus briefly in free travel and asserts homing does **not** latch
  (`homing.rs:285-323`: the 0.15 s startup guard, the 60 %-of-window current-ratio
  requirement). Real drivers draw inrush at velocity-mode start — that is what the guard is
  for. On J0 a false stall survives the two-pass check (see **M1**).
  *Test:* at the existing `HomingSystem` harness seam (`homing.rs:220-280`), script saturated
  current for the first 0.14 s with normal travel, assert `SeqStatus::Running` throughout;
  mirror with a 40 %-duty current window to prove the 60 % requirement.
- **G5 — gravity model inertials.** `crates/par6-kin/src/kin.rs:95-109` loads the URDF with
  `tool: None`; its moving links total **2.375 kg** against **5.114 kg** in the vendor's own
  runtime dynamics model (`rcb-runtime/robots/PAR6.py:47`, consumed at
  `robotics/dynamics.py:53` → `data/update_shared_data.py:182`), and
  `assets/par6_description/readme.md` calls the URDF a "simplified robot description". Every
  par6 gravity test is self-referential (Rust-vs-Python over the same URDF; plant-vs-model
  over the same URDF). IDLE hold is **torque-only, no position hold**, so a ~2× under-
  compensation passes every test and appears as sag on the first `enable`. It also poisons
  `τ_ext = τ_filtered − G(q)`, the external-force/collision estimate. *(Geometry is fine — URDF
  joint origins 0.02342 / 0.1105 / 0.180 / 0.0435 / 0.177 match the vendor DH exactly. Only
  the inertials diverge, which is precisely the half no test can see.)*
  *Test:* assert per-joint G(q) at a table of poses against a checked-in reference vector
  derived from the vendor's mass/COM/inertia table, so a simplified URDF fails loudly.
  Definitive check is HIL (§5.3).

**High**

- **G6 — the shipped 250 Hz configuration, end to end.** `sim_session.rs:49` and
  `ffi_kinematics.rs:59` patch `tick_dt_s` to 0.02; `python/tests/live_daemon.py:40` uses 0.05
  at 20 Hz status. Nothing runs `par6d` with RT, planner, tee, server and bridge threads all
  live at 4 ms. Every RT time constant is `round(s/dt)`, so the tick rate moves ~20 derived
  counts at once — and the repo has **already been bitten by exactly this**
  (`core_errors.rs:332` documents the stream watchdog rounding to one unsatisfiable tick at a
  non-default dt). The reverse direction (32-frame RX cap, poll cadence over 7 nodes, status
  decimation ratio of 5 — which is 1 in every current test) is unexercised.
  *Test:* run one `sim_session` workflow at the unpatched 0.004 under `--release`, asserting
  no `LOOP_CRITICAL` and `p99_period_s <= 1.05·dt`. Make it the standard pre-deploy command.
- **G7 — homing approach timeout** (`homing.rs:332`). The only guard against a joint driving
  forever — detached endstop, seized detector, wrong direction — and no test lets an approach
  reach `homing_timeout_s`. Until it fires the joint runs in velocity mode at homing current
  with normal Ilim swapped out; on J3/J4 that is 1200 mA at 13500 ticks/s. It is the one
  homing constant no test pins.
  *Test:* free-running node at low current; assert `SeqStatus::Failed` at
  `round(timeout_s/dt) + settle` and not before, `statuses()[j] == Failed`, and config
  restored. Run at two tick rates to pin the conversion.
- **G8 — homing release phase** (`homing.rs:425-440`). No test asserts the **sign** of the
  release current, the sample tick, or that the latched value is the relaxed position. The
  sign is per joint and opposite between neighbours (config: J1 +150 mA, J2 −150 mA, matching
  the vendor). A sign error drives the joint **harder** into the endstop and latches a
  position with full gearbox windup in it. J4's release is 1500 mA for 1.5 s.
  *Test:* a plant that relaxes N ticks on the releasing sign and winds up on the other;
  assert the commanded current sign and duration, that the reference is sampled at
  `round(dur·0.8)`, and that inverting the config sign **fails** the test.
  (`sim_bus.rs:250` validates the sim plant's windup model, not the FSM's use of it.)
- **G9 — bus TX/RX failure handling.** `core.rs:956, 959, 1063, 1066` log and continue;
  nothing latches, nothing reaches STATUS. Neither `LoopbackBus` nor `SimBus` can return
  `Err`, so the entire `Err` arm of the `DriverBus` contract is dead in every test — including
  the hot logging path of **M6**. `spec/CAN.md` adopts the opposite stance: "Rust stance:
  propagate send errors (vendor swallowed them — documented production bugs)".
  *Test:* a `FailingBus<B>` delegating wrapper with per-method failure switches; assert
  sustained TX failure produces a hard latch and DISABLED within a bounded tick count, and
  assert zero allocations per tick in the failure arm with the counting allocator.
- **G10 — jog soft-limit lookahead under tracking lag, through the shipped engine.**
  `par6-motion/tests/jog.rs:16-38` feeds `q_meas = out.q` — perfect tracking, zero lag — so
  the measured-pose hard-clamp branch (`jog.rs:296-307`) is reached only by a hand-built
  overrun. The RT-level test (`core_modes.rs:172`) uses `RampJog` against a pose that never
  moves. And `par6d` installs `MotionJog` (`adapters.rs:24`), which **no test drives through
  `RtCore` at all**. The lookahead computes stopping distance from the integrated *target*
  while the hard clamp uses `q_meas`; with real lag those disagree, at J0's 4.8 rad/s
  substantially.
  *Test:* `jog_j` toward a soft limit against `par6d --sim` over the real protocol (the sim's
  closed-loop driver supplies genuine lag), asserting the measured angle stops short and the
  blocked-direction bit sets; plus a first-order lag model in `run_tracked`.

**Medium**

- **G11 — gripper tool inertials are parsed, validated and never read.**
  `crates/par6-config/src/gripper.rs:66-72` parses `mass_kg`, `com_m`, `inertia_kg_m2`,
  `motor_inertia_kg_m2`; nothing in the workspace reads them. `PinokinGravity`
  (`gravity.rs:55`) accepts `ToolParams` but `par6d` wires `KinGravity(Kin)` with
  `tool: None`. `spec/RT.md` claims G(q) covers "the active gripper tool link (masses/COM/
  inertia from config)" — **the spec is wrong about the implementation**. The two sources also
  disagree: config says the MSG gripper is 0.37 kg, the URDF's gripper + jaws total 0.221 kg.
  Whoever tunes gravity comp will edit the config value and see nothing change, and a payload
  cannot be expressed at all. Violates the repo's declared-but-unimplemented rule.
  *Test:* assert that changing `[kinematics] mass_kg` changes published wrist
  `gravity_torque_nm` — failing today, forcing the choice: wire it through `ToolParams`
  (reconciled against the URDF's own gripper links so mass is not double-counted), or delete
  the fields and fix `spec/RT.md`.
- **G12 — boot kt fetch failure shapes** (`hw/mod.rs:675`). Only coverage is
  `socketcan_vcan.rs:282`, which never runs. Once G2 lands, cover the shapes the requirement
  implies: no node answers (fall back and say so), one node answers wildly out of family
  (reject, do not adopt), answers arriving after the timeout (do not apply late) — then assert
  `torque_ma_factor` reflects exactly the adopted values. Blocked on **H3** being fixed at all.
- **G13 — Waldo Commander contract.** Repo task #18 is still open; par6's e2e stops at the
  Python client, and `PARITY.md`'s claims (46 vs 47 waldoctl entries, STATUS field mappings,
  the `[0,0,0,0,!estop]` io shape, latching-vs-auto-recovering e-stop) are hand-probed, not
  executed. `parol6`'s suite shows which of these bite in real use
  (`test_status_broadcast_autofailover.py`, `test_unhomed_motion_gate.py`,
  `test_stale_error_wait.py`). The frontend is what a human holds during bring-up — par6
  already had to fix an enablement-pair ordering bug (`sim_session.rs:960`), and **M5** and
  **C3** are both frontend-visible.
  *Test:* a WC CI job installing par6 from the matching branch, booting `par6d --sim`, and
  running WC's `user`-fixture integration tests against it.

### 5.3 HIL-only — the bring-up checklist

> **Everything below requires the physical arm and cannot be settled in CI.** Run in order.
> Steps 0–3 happen with motors **unpowered** or the arm restrained. Do not proceed past a
> failing step.

**Step 0 — target-architecture self-check (motors off, before CAN is even opened).**
`.github/workflows/ci.yml` says of the deploy-aarch64 job: *"Nothing here can be executed (the
runner is x86_64)"* — the shipped aarch64 `par6d`, its Pinocchio/coal shim and its staged
runtime closure have been linked and symbol-checked, never run. An aarch64 shim that loads but
computes wrong takes down FK, IK, gravity and the collision gate at once, and `par6d` refuses
to boot without kinematics, so this presents as "the runtime will not start" at best and wrong
poses at worst.
→ Ship the `par6-kin` `golden_kinematics`/`golden_collision` fixtures and the
`tests/golden/protocol` + `tests/golden/can` vectors with the deploy bundle; add
`par6d --selftest` that decodes/encodes every vector and reproduces every kinematics fixture
to 1e-9. **First command run on the box.**

**Step 1 — CAN wire conformance against real firmware (arm powered, par6d NOT transmitting).**
`tests/golden/can/manifest.json` is exhaustive over the command table, but the vectors are
par6's own bytes checked against par6's own typed expectation — **both derived from
`spec/CAN.md`**. That spec itself records that the vendor's published documentation is wrong
in two places (endianness and bitfield ordering) and that the MIT `Spectral_BLDC/CAN_utils.py`
library, not the docs, is what firmware agrees with. If the spec misread the library anywhere
— the cmd-60 gripper reply's current at bytes 1..3, the `index 0 = bit 7` bitfield fold, the
DLC-selects-mode rule — the entire suite agrees with itself and disagrees with the arm.
→ `candump -L can0` while the **vendor** runtime drives the arm; diff par6's encoder output
against those captured frames for every command class **before `par6d` is ever allowed to
transmit**. Offline, regenerate the manifest's expected bytes from the MIT Spectral-BLDC-Python
encoders (license-compatible — it is only the GPL runtimes that are reference-only) and commit
the generator, so a spec misreading fails the golden test.

**Step 2 — e-stop, physically (after C1 is fixed; motors powered, arm restrained).**
Press the button. Assert within `DEBOUNCE_READS` ticks: STATUS shows `ESTOP` latched,
`mode == ACTIVE_ERROR`, `state == DISABLED`, `io()[4] == 1`. Release; assert the latch
**persists** until an explicit `reset()`. Then repeat while a `move_j` is executing and confirm
no motion on release. This is the single test that would have caught **C1**.

**Step 3 — RT scheduling and bus health on the box.**
`crates/par6-rt/src/rt.rs::setup_realtime` (SCHED_FIFO 99, `sched_setaffinity` to CPU 3) is
exercised nowhere: `daemon.rs:352-362` runs `--sim` with `cpu: None, fifo_priority: None`, and
`RunOptions::default()` is hardware-only and never constructed in a test. Wrong here means
either no real-time priority (jitter → `LOOP_DEGRADED`/`LOOP_CRITICAL`, which hard-latches to
DISABLED mid-motion) or FIFO 99 pinned to a non-isolated core that starves its neighbour.
→ Verify `chrt -p` / `taskset -p` on the running thread and that `isolcpus` actually covers the
pinned core. Also confirm `setup_realtime` degrades gracefully (never panics) without
CAP_SYS_NICE, and that `pin_to_cpu` beyond `num_cpus` logs and continues.
→ `crates/par6-bus/src/hw/link.rs` has **no test module at all** — interface bring-up
(`:87-149`) and the ~1 Hz netlink `LinkMonitor` (`:193-270`) are unexecuted, and vcan cannot
reach bring-up because virtual interfaces have no bit timing (`link.rs:131`). `spec/CAN.md`
gives the reason the monitor exists: the kernel's 100 ms auto-restart lands **between** the
10-tick stale warning and the 50-tick lost latch, so a bus-off/recover cycle is otherwise
invisible and the arm keeps being commanded across a bus that is repeatedly dropping off.
Force bus-off with a real CAN pair (a vcan cannot produce it) and assert a distinct alarm.

**Step 4 — per-joint homing signature (each joint alone, arm on blocks / uncoupled where
possible).** The sim's plant is a model **tuned to satisfy the detector** — `sim/plant.rs:20-28`
sizes `VISC` explicitly so J2's drag stays under the current-ratio threshold — which is
circular. Run each joint's homing approach **alone**, logging position and current at 250 Hz,
and compare the measured plateau and current rise against `stall_disp_ticks` and
`0.70 · current_ma` **before enabling the full sequence**. Do J5 (hall) twice in one process —
that is the reproduction for **C2**.

**Step 5 — driver gain stability.** par6 pushes the onboard KPP/KPV/KIV/KPIQ/KIIQ gains
verbatim from config; nothing confirms they are stable with the real inertia at 250 Hz. Step
each joint 5° in position mode and log the response for overshoot/oscillation **before any
multi-joint move**. An unstable current loop on a geared joint is something you hear before
you see it.

**Step 6 — bus timing under load.** `candump` the steady-state tick; measure worst-case frame
latency and bus load, and confirm the ~14-frame exchange really fits in 1.8 ms at 1 Mbit with
real arbitration.

**Step 7 — gravity ground truth (closes G5, the only way to).** Hold each pose in IDLE with
comp **off** and read the steady-state motor current the position hold needs; compare against
`G(q) · factor` per joint. A per-joint scale error shows up directly. This is also the only
check that would catch a kt mismatch (**H3**) independently.

---

## 6. What this review could not establish

These are genuinely open until the arm runs. Each is a place where a model that agrees with
itself may disagree with the machine.

1. **What the drivers do when the e-stop chain opens.** Whether they latch VBUS/ESTOP_MOTOR
   when motor power is removed determines whether **C1**'s post-release move is a real runaway
   or merely a `CAN_LOST` after 0.2 s. The software-side claim — that the entire debounce/
   latch/`ACTIVE_ERROR` path is unreachable in `par6d` — is fully established from source; the
   physical consequence is not.
2. **The arm's true inertial parameters.** URDF moving mass 2.375 kg vs the vendor runtime's
   5.114 kg (`rcb-runtime/robots/PAR6.py:47`). One of them describes the arm. Every par6
   gravity test is self-referential, so source cannot settle it — only HIL step 7 can. Until
   then, treat `G(q)` as unvalidated and expect sag on first enable.
3. **Whether par6's CAN bytes are what the firmware accepts.** The golden vectors and
   `spec/CAN.md` are mutually consistent; nothing cross-checks either against the MIT Spectral
   library or captured frames. `spec/CAN.md` already documents two places where the vendor's
   *published docs* are wrong — so the spec is a careful reading of a library, and a careful
   reading can still be wrong. HIL step 1.
4. **The real per-joint stall signature.** The sim plant is tuned to satisfy the detector, so
   nothing here says the thresholds discriminate a real endstop from real free-travel drag.
   Directly gates whether **M1** is a 4° error or a benign margin.
5. **Whether the StepFOC firmware resets its velocity integral on an Idle → cmd-2 transition.**
   This is the whole of **L2**'s severity, and it is only reasoned about from par6's *own model*
   of the driver (`sim/driver.rs:159-166, 230-234`) — which is not evidence about the firmware.
   Verdict UNCERTAIN by construction.
6. **Whether the configured driver gains are stable at real inertia at 250 Hz**, and whether
   the ~14-frame per-tick exchange fits the 1.8 ms budget with real arbitration. HIL steps 5–6.
7. **Whether any driver's actual kt differs from `config/PAR6.toml`.** **H3** is a latent
   defect whose magnitude is exactly that difference; if every driver matches its XML value,
   the discarded fetch costs 2.5 s of boot and nothing else. There is no way to know before
   the fetch runs against real hardware.
8. **aarch64 FFI numerics.** The Pinocchio/coal shim has never executed on the target. HIL
   step 0.
9. **RT scheduling on the box** — whether SCHED_FIFO 99 is actually granted and whether the
   pinned core is isolated. HIL step 3.

One meta-point worth keeping: several of the defects above (**C2**, **H1**, **H2**, **H3**,
**M1**, **M2**) are cases where par6 ported the vendor's *structure* but dropped a small
mechanism the structure depends on — the hall clear, the "seen now" re-base, the marker
consumption, the kt rebind, the pass-scaled threshold, the graceful gripper skip. That is a
recognisable pattern, and it is worth a targeted re-read of any remaining vendor port against
`rcb-runtime` before bring-up rather than trusting that the shape being right means the
behaviour is.

### Verified as correct while checking (not defects)

Recorded so they are not re-litigated: every joint constant in `config/PAR6.toml` (kt, Ilim,
gear ratio/efficiency, dir, sector master/offset, all seven gains, hard/soft/per-mode limits,
all homing fields including release blocks) matches `rcb-runtime/robots/PAR6.xml` **exactly**;
the homing sequence matches `config/PAR6_homing.xml` step for step; the URDF link geometry
matches the vendor DH lengths exactly; `dir = [0,0,1,1,1,1]` matches the vendor's negative
gear-ratio signs; and worst-case gravity feedforward currents from the golden fixtures sit
well inside every Ilim.
