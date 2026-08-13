# CAN.md — Spectral/STEPFOC bus protocol (hardware-facing spec)

Behavioral spec for `par6-bus`, extracted from the vendor stack
(`Source-Robotics/RCB-Runtime` + `Spectral-BLDC-Python` + `Source-Robotics-Toolbox`).
File:line references point into those repos for verification. **Reference only —
never copy vendor code (GPL).** The Spectral-BLDC / SourceRoboticsToolbox libs are MIT.

**Endianness ground truth:** the vendor docs page claims LSB-first — it is wrong for the
application protocol. Every pack/unpack helper in `Spectral_BLDC/CAN_utils.py` uses
`struct '>'` — **big-endian**. The *bootloader* protocol (flashing) is little-endian.
The docs page's bit numbering inside status bytes is also inverted vs the library;
the library is what firmware agrees with.

## Link layer

| Property | Value |
|---|---|
| Bus | classic CAN 2.0A, 11-bit IDs, SocketCAN `can0` |
| Bitrate | 1,000,000 bps; `restart-ms 100`; txqueuelen 1000 |
| SO_SNDBUF | request 4 MiB (kernel caps at wmem_max) |
| Budget | full 6-joint exchange ≈14 frames ≈1.8 ms → ~250 Hz at ~45% load, ~500 Hz ceiling |

Rust stance: **propagate send errors** (vendor swallowed them — documented production
bugs, `can_hardware.py:150-158`). Keep the SO_SNDBUF raise as backstop. Read kernel link
state (bus-off/error-passive/restarts) via netlink at ~1 Hz off the RT thread — auto-restart
(100 ms) lands between the freshness thresholds, so bus-off is otherwise invisible.

The RT loop acts on the propagated errors, never just logs them: every refused
send/drain is counted into the published loop stats (`bus_tx_failures` /
`bus_rx_failures`), the backend's `LinkHealth` rides every snapshot, and a TX-failure
streak spanning the freshness lost window (`lost_s`) latches every node's disconnect
error (⇒ DISABLED / ACTIVE_ERROR) — an outbound-dead link means the arm is not
receiving commands even while RX freshness reads green, so it disables on the same
clock as a silent one. Home references survive a TX-only latch (the encoders were
never lost); an actually-silent bus still invalidates homing through the freshness
path.

## Arbitration ID (11-bit)

```
can_id = ((node & 0xF) << 7) | ((cmd & 0x3F) << 1) | (err_bit & 0x1)
```

- node 0–5 = joints J1–J6, 6 = gripper, 13 = reserved timing dummy, 14/15 = host/bootloader.
- `err_bit` is set BY THE DRIVER on every reply while it has an active fault — harvest it
  per-frame into `node_error_bits[node]` before payload dispatch. It is the authoritative
  live fault signal (per-type flags are only ~84 ms fresh — see RT.md error gating).
- Lower node id wins arbitration.

## Payload primitives

Big-endian, two's-complement: i24 (3 bytes), i16 (2 bytes), i32/u32 (4 bytes),
f32 (IEEE-754 BE). Bitfields: **list index 0 = bit 7** (MSB-first fold) — this is why the
vendor docs' bit tables look inverted.

## Command table

H→D = host to driver; RTR = remote frame, DLC 0.

| ID | Name | Kind | Payload |
|---|---|---|---|
| 0 | ESTOP | data DLC 0 | — (exists; vendor runtime never sends it — see RT.md e-stop) |
| 1 | Clear_Error | data DLC 0 | — |
| 2 | data_pack_1 (cascade PID) | data DLC 8/5/2 | see below |
| 3 | Respond_data_pack_1 | D→H DLC 8 | i24 pos, i24 spd, i16 cur(mA) |
| 4 | data_pack_PD (impedance) | data DLC 8 | same layout as 8-byte cmd 2 |
| 5,7 | Respond_data_pack_2/3 | D→H | reserved — do not decode |
| 10 | Ping | RTR | reply cmd 10 |
| 11 | Set_CAN_ID | DLC 1 | u8 new id |
| 12 | Idle | DLC 0 | — |
| 13 | Save_config | DLC 0 | — |
| 14 | Reset | DLC 0 | reboots into bootloader (~8.2 s to bootloader ping) |
| 15 | Watchdog | DLC 5 | u32 BE ms + u8 action |
| 16 | PD_Gains | DLC 8 | f32 KP, f32 KD |
| 17 | Current_Gains | DLC 8 | f32 KPIQ, f32 KIIQ |
| 18 | Velocity_Gains | DLC 8 | f32 KPV, f32 KIV |
| 19 | Position_Gains | DLC 4 | f32 KPP |
| 20 | Limits | DLC 8 | f32 vel_limit (ticks/s), f32 cur_limit (mA) |
| 22 | Kt (set) | DLC 4 | f32 Nm/A |
| 23 | Temperature | RTR | reply DLC 2: i16 °C |
| 24 | Voltage | RTR | reply DLC 2: i16 mV |
| 25 | Device_Info | RTR | reply DLC 7: u8 hw_ver, u8 batch, u8 sw_ver, i32 serial |
| 26 | State_of_Errors | RTR | reply DLC 2: bitfields below |
| 27 | Iq_data | RTR | reply DLC 2 |
| 28 | Encoder_data | RTR | reply DLC 8: i32 pos, i32 spd |
| 30 | Heartbeat_Setup | DLC 4 | u32 BE ms |
| 31 | data_pack_HALL | DLC 4 | i24 speed + u8 trigger_value |
| 32 | RESPOND_DATA_HALL | D→H DLC 4 | i24 pos + bitfield: b7 HALL_trigger, b6 pin2, b5 hall_index/edge |
| 33 | Respond_Kt | RTR | reply DLC 4: f32 BE Nm/A |
| 34 | Voltage_Limit | DLC 4 | u32 BE mV (absent from vendor docs; old firmware ignores) |
| 60 | Respond_Gripper_data | D→H DLC 4 | see Gripper |
| 61 | Gripper_data_pack | DLC 5 or 0 | see Gripper |
| 62 | Gripper_calibrate | DLC 0 | — |

### Motion frame (cmd 2) — DLC selects the mode

| Variant | DLC | Layout |
|---|---|---|
| position | 8 | i24 pos ticks · i24 spd ticks/s · i16 cur mA |
| velocity (`pos=None`) | 5 | i24 spd · i16 cur |
| current (`pos=None, spd=None`) | 2 | i16 cur |

**Channel semantics are load-bearing:** `Position=None` → channel omitted; `Speed=None` →
omitted; `Current=None` → substitute **0** (not omitted). Model as `Option<i32>` — a default
of 0 is wrong. Values are `int()`-TRUNCATED (not rounded) before packing. cmd 4 (PD) always
sends DLC 8; the driver computes torque from position+velocity error with onboard KP/KD
(no integral), Current acts as feedforward.

**Wrong-DLC frames are discarded whole** — never partially update state from them.

## Units & conversions (SourceRoboticsToolbox `Joint` semantics)

14-bit encoders everywhere (`encoder_max_counts = 16384`).

```
ticks_per_radian = (encoder_max_counts * gear_ratio) / 2π
joint_speed_rad_s = motor_speed_ticks_s * (2π / encoder_max_counts) / gear_ratio
joint_ticks = motor_pos - master_position + offset_ticks (± encoder_max_counts per sector)
joint_rad   = ticks_to_radians(joint_ticks);  if dir == 1: joint_rad = 2π - joint_rad
offset_ticks = radians_to_ticks(offset if dir == 0 else 2π - offset)
motor_mA = trq_nm * sign * 1000 / (gear_ratio * gear_efficiency * kt)   # sign = 1 - 2*dir
```

Sector selection happens once at boot from the unwrapped initial position (see vendor
`Joint.determine_sector`). Home reference update (homing): `master_position = latched ticks`,
`offset = home_offset rad` — post-condition: `joint_position(endstop_tick) == home_offset`.

PAR6 (from `robots/PAR6.xml`): gear ratios 6.4/25/18.095/4/4/10, vel limit 80000 ticks/s,
Ilim 1200–2500 mA, kt 0.151–0.31 Nm/A (fetchable from drivers at boot via cmd 33 when
`kt_source=auto`), watchdog 5000 ms → Idle, voltage_limit 6000 mV.

## Per-tick pattern (vendor 250 Hz; ours: rate from config)

TX ≈8 frames: 6 joint motion frames (cmd 2 or 4 by control mode) + 1 gripper frame
(cmd 2 motor-mode / cmd 61 firmware-mode / RTR ping to dummy node if no gripper) +
1 round-robin poll — each node gets temp(23)/voltage(24)/errors(26) every
`3 × total_nodes` ticks (~84 ms at 250 Hz/7 nodes); a device-info(25) sweep replaces the
poll for `total_nodes` ticks roughly every 1006 ticks (~4 s). A single-slot override
queue `(action, repeat_count)` preempts the poll (used for config resend, clear-error).

RX: drain until empty, cap 32 frames/tick (surplus over the ~8 steady-state clears
backlogs; vendor found a 50 ms staleness bug with a fixed 8-read loop). Per frame:
decode id → (node, cmd, err); record `received_ids`, `latest_command_id`, `node_error_bits`;
dispatch payload by cmd. Track frame age from kernel RX timestamps; publish max+min
(min≈max large = genuine backlog; only-max large = one slow frame class).

## Boot sequence

1. Bring up can0 (down → up type can bitrate 1000000 restart-ms 100 → txqueuelen).
2. Config load per node, `num_repeats` passes, **paced ~0.5 ms per message-type batch**
   (TX queue silently drops on overflow: ~170 frames enqueue in µs vs ~10 frames/ms drain):
   order = Watchdog(rate, "Idle") → Limits(vel, cur) → Voltage_Limit → PD_Gains →
   Current_Gains → Velocity_Gains → Position_Gains. Same 7 to the gripper.
3. Optional kt fetch (cmd 33 RTR per node, 0.35 s timeout, 3 retries, 2 rounds).
4. Bus scan: RTR ping ids 0–15, 2 rounds, 2.5 ms wait each → connected map.
5. Reconnect: on a node's stale→fresh edge, re-send that node's full config (2 passes).

## Freshness / health (3 layers)

1. Per-node data age in ticks: ≥10 → stale WARNING (live, self-clears); ≥50 → disconnect
   ERROR (**latched** — only user clear-errors resets it).
2. Live fault bit = CAN-id err_bit per reply (authoritative, per-frame fresh).
3. Kernel link counters via netlink at 1 Hz (off RT thread): alarm on bus_off, restarts,
   error_passive; a decreased counter = interface re-based, not negative delta.

## Error flag bitfields (cmd 26 reply, DLC 2; index 0 = bit 7)

Byte 0: b7 error(aggregate), b6 temperature, b5 encoder, b4 vbus, b3 driver, b2 velocity,
b1 current, b0 estop. Byte 1: b7 calibrated, b6 activated, b5 watchdog, rest unused.

## Gripper

Node = joint_num (6). Two exclusive drive modes: **motor mode** (cmd 2, driver acts as a
7th joint; SI conversions `ticks_per_meter = 2^14 / (4π · Gear_r)` — one pinion, two jaws)
and **firmware mode** (cmd 61). SSG48 vs MSG differ by `driver_type` (spectral-bldc vs
stepfoc) — a flashing-time guard, not a protocol difference.

cmd 61 (DLC 5): u8 position (0 open → 255 closed), u8 speed, i16 BE current mA,
bitfield b7 activate(always 1), b6 action(1=goto), b5 estop, b4 release_dir.
**DLC-0 variant = empty poll**: feeds the driver watchdog WITHOUT overwriting the
in-progress firmware command — required every tick during calibration and firmware homing.

cmd 60 reply (DLC 4): u8 position, i16 BE current (bytes 1..3!), bitfield b7 activated,
b6 action_status, (b5<<1|b4) object_detection {0 moving, 1 detected-closing,
2 detected-opening, 3 reached-no-object}, b3 temperature_err, b2 timeout_err,
b1 estop_err, b0 calibrated.

Calibration: send cmd 62 ONCE, then empty polls every tick until a new gripper command
arrives or 30 s timeout. Known vendor defect (reproduce knowingly or gate): firmware-mode
SI position reads as a constant — publish NaN / gate on ctrl mode instead.

## Flashing (delegated to vendor tools; runtime obligations only)

Runtime side: FLASHING mode = bus-silent (no TX at all, suppress polls + freshness checks)
while RX is drained-and-DISCARDED (bootloader page frames alias application ids; cap 64
frames/tick). Motors lose torque (watchdogs fire → shorted-phase brake) — entry gated on a
human park assertion, never a measurement. On exit: re-base all nodes' freshness clocks;
if the flash marker file exists → invalidate homing robot-wide. Bootloader protocol
(little-endian, ids 0x700/0x701+board, page stream id `(board<<7)|seq`, STM32 CRC-32/MPEG-2)
stays in vendor tools; an advisory flock guards single-flasher access.

## Implementation findings (P1.A, vendor-verified — codec is authoritative)

1. **Speed conversion carries the dir sign**: `get_joint_speed` negates for
   `dir == 1` (the formula section above omits it). Both directions flip.
2. **Sector boundaries**: vendor thresholds are asymmetric (`master − 8192` vs
   `master + 8191`) and exact-boundary readings fall through unclassified. Codec
   mirrors the thresholds and resolves boundary readings to the uncorrected
   sector (shift 0). Invariant: the corrected boot delta is always within ± half
   a motor revolution.
3. **`set_home` clears the boot sector shift** — the latched tick is a live
   accumulated position, not a wrapped boot reading. (The vendor does not clear
   it, which breaks the `joint_position(endstop) == home_offset` post-condition
   whenever the shift was nonzero; the post-condition is the contract.)
4. **Cmd 9 = heartbeat reply** (payload-less), enabled by cmd 30 — added to the
   command table; decode as `Heartbeat`.
5. Payload-less replies (cmds 9/10) get **no DLC enforcement** (vendor checks none).
6. No-gripper tick-timing frame = **RTR ping to the dummy node** (newer vendor
   handler), not a cmd-61 pack (older util).
7. A cmd-2 encode with position but no velocity is **refused loudly** (typed
   error) — the vendor silently dropped it.
