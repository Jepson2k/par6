//! Real-time core. SCHED_FIFO pinned thread, allocation-free after init.
//!
//! Tick (250 Hz to start; rate is config): bus RX drain → state update →
//! gravity G(q) → mode dispatch (pure per-mode setpoint fns) → bus TX →
//! state snapshot (triple buffer) for status/telemetry. Absolute-
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
//!
//! This crate currently ships the SHARED TYPES the workstreams build on
//! (contract freeze 2); the tick loop itself is a later issue:
//!
//! - [`ring`]: the planner→RT SPSC sample ring ([`Sample`],
//!   [`sample_ring`], `samples_remaining` backpressure).
//! - [`snapshot`]: the single-writer snapshot channel
//!   ([`snapshot_channel`], triple buffer, wait-free, tear-free).
//! - [`state`]: [`StateSnapshot`] and its component types (modes, error
//!   latch list, homing/exec/jog/stream status, loop stats).

pub mod ring;
pub mod snapshot;
pub mod state;

pub use ring::{sample_ring, Sample, SampleConsumer, SampleMeta, SampleProducer};
pub use snapshot::{snapshot_channel, SnapshotReader, SnapshotWriter};
pub use state::{
    ArmState, ErrorCode, ErrorEntry, ErrorList, ExecStatus, HomingJointStatus, HomingStatus,
    JogStatus, LoopStats, Mode, StateSnapshot, StreamStatus, StreamSubstate, MAX_ERRORS,
};

/// Compile-time arm joint count the fixed-size RT types are dimensioned
/// for (PAR6: 6). Config joint count must equal this at runtime
/// construction; the gripper is a separate actuator and NOT included.
pub const MAX_JOINTS: usize = 6;

/// Telemetry node count: arm joints plus the gripper node (last index).
pub const NUM_NODES: usize = MAX_JOINTS + 1;
