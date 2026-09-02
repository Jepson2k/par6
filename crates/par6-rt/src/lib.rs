//! Real-time core. SCHED_FIFO pinned thread, allocation-free after init.
//!
//! Tick (250 Hz to start; rate is config): bus RX drain → state update →
//! gravity G(q) → mode dispatch (pure per-mode setpoint fns) → bus TX →
//! state snapshot (triple buffer) for status/telemetry. Absolute-
//! deadline timing (clock_nanosleep TIMER_ABSTIME); one-sided p99
//! degradation bands. The vendor runtime orders the tick
//! command-before-measure, which costs a tick of latency; par6 measures
//! then commands. That deviation has not been validated on hardware.
//!
//! Semantics ported exactly from the vendor runtime: IDLE-with-gravity =
//! torque-only hold; ACTIVE_ERROR = active zero-velocity; SAFETY_STOP =
//! limp; e-stop = mode latch (never motor power-off); ESTOP_2 excluded;
//! debounce first-read seeding; FLASHING = bus-silent + RX-discard +
//! homing invalidation; hard errors latch, warning keys self-clear,
//! live-fault-bit gating on stale per-type flags. Homing FSM per
//! the vendor sequence (two-pass, release phase, gripper-dependent offsets).
//!
//! Layout:
//!
//! - [`core`]: [`RtCore`] — the tick-loop assembly (testable, virtual
//!   ticks) plus the command-plane handles ([`RtHandles`]).
//! - [`rt`]: the thin real-time `run()` wrapper (absolute deadlines,
//!   SCHED_FIFO, graceful degradation).
//! - [`dispatch`]: per-mode output laws and the motor-space commit path.
//! - [`homing`]: the sequence orchestrator + per-joint homing FSM.
//! - [`exec`]: EXEC sample-ring playback with the completion policies.
//! - [`errors`]: the error latch manager (hard/warning keys, clear
//!   settle).
//! - [`gpio`]: e-stop and digital-I/O line abstractions + first-read-seeded
//!   debounce.
//! - [`gravity`]: the G(q) model seam ([`ZeroGravity`], pinokin behind
//!   the `ffi` feature).
//! - [`timing`]: loop-period statistics and the p99 degradation bands.
//! - [`hooks`]: the small per-tick trait seams `par6d` wires the motion
//!   stack onto (jog, streaming, completion, commands, FK, flash marker).
//! - [`ring`]: the planner→RT SPSC sample ring ([`Sample`],
//!   [`sample_ring`], `samples_remaining` backpressure,
//!   generation-bounded flushes via [`FlushMarker`]).
//! - [`snapshot`]: the single-writer snapshot channel
//!   ([`snapshot_channel`], triple buffer, wait-free, tear-free).
//! - [`state`]: [`StateSnapshot`] and its component types (modes, error
//!   latch list, homing/exec/jog/stream status, loop stats).

pub mod core;
pub mod dispatch;
pub mod errors;
pub mod exec;
pub mod gpio;
pub mod gravity;
mod gripper_gate;
pub mod gripper_settle;
pub mod homing;
pub mod hooks;
pub mod ring;
pub mod rt;
pub mod snapshot;
pub mod state;
pub mod timing;

pub use crate::core::{
    CoreError, ExecHeartbeat, GateRefusal, RtCore, RtHandles, RtHooks, StreamInput, StreamSetpoint,
};
pub use gpio::{
    Debouncer, DigitalIo, EstopGpio, EstopMonitor, NoDigitalIo, SharedDigitalIo, SharedIoLines,
    SharedLineGpio, DEBOUNCE_READS,
};
pub use gravity::{GravityModel, ZeroGravity};
pub use hooks::{
    CommandSource, CompletionPolicy, FlashMarker, ForwardKin, JogEngine, NoCommands, NoFk,
    RtCommand, SettlePolicy, SharedFlashMarker, SpecSettle, StreamTracker,
};
pub use par6_bus::{Freshness, LinkHealth, LinkState, NodeState};
pub use ring::{sample_ring, FlushMarker, Sample, SampleConsumer, SampleMeta, SampleProducer};
pub use rt::RunOptions;
pub use snapshot::{snapshot_channel, SnapshotReader, SnapshotWriter};
pub use state::{
    ArmState, ErrorCode, ErrorEntry, ErrorList, ExecStatus, HomingJointStatus, HomingPhase,
    HomingStatus, JogStatus, LoopStats, Mode, StateSnapshot, StreamStatus, StreamSubstate,
    MAX_ERRORS,
};

/// Compile-time arm joint count the fixed-size RT types are dimensioned
/// for (PAR6: 6). Config joint count must equal this at runtime
/// construction; the gripper is a separate actuator and NOT included.
pub const MAX_JOINTS: usize = 6;

/// Telemetry node count: arm joints plus the gripper node (last index).
pub const NUM_NODES: usize = MAX_JOINTS + 1;
