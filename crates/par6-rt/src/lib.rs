//! Real-time core. SCHED_FIFO pinned thread, allocation-free after init.
//!
//! Tick (250 Hz to start; rate is config): bus RX drain → state update →
//! gravity G(q) → mode dispatch (pure per-mode setpoint fns) → bus TX →
//! state snapshot (seqlock/triple-buffer) for status/telemetry. Absolute-
//! deadline timing (clock_nanosleep TIMER_ABSTIME); one-sided p99
//! degradation bands. NOTE: vendor ordering is command-before-measure
//! (1 tick extra latency); we measure-then-command — deviation flagged for
//! HIL validation, config flag restores vendor ordering. See `spec/RT.md`.
//!
//! Semantics ported exactly from the vendor spec: IDLE-with-gravity =
//! torque-only hold; ACTIVE_ERROR = active zero-velocity; SAFETY_STOP =
//! limp; e-stop = mode latch (never motor power-off); ESTOP_2 excluded;
//! debounce first-read seeding; FLASHING = bus-silent + RX-discard +
//! homing invalidation; hard errors latch, warning keys self-clear,
//! live-fault-bit gating on stale per-type flags. Homing FSM per
//! `spec/HOMING.md` (two-pass, release phase, gripper-dependent offsets).
