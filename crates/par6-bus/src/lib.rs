//! Bus abstraction and backends.
//!
//! - [`DriverBus`] trait: what the RT loop needs from a motor bus, backend-
//!   agnostic (per-tick send of per-joint setpoints, RX drain, telemetry
//!   poll scheduling, boot config sequence, health/freshness). Time is a
//!   caller-supplied u64 tick count — `Instant`-free so sim backends are
//!   deterministic.
//! - Driver model types ([`types`]): `Option`-channel [`JointCommand`]
//!   (None = channel omitted on the wire, NOT zero — see `spec/CAN.md`),
//!   gripper commands incl. the DLC-0 empty poll, per-node measured state,
//!   per-type error flags vs the per-frame live fault bit, freshness
//!   warn/latch classification, kernel link health.
//! - [`LoopbackBus`]: in-memory reference implementation the contract
//!   unit tests run against. Not a production backend.
//! - Spectral/STEPFOC frame codec (spectral module): classic CAN 2.0A, 11-bit
//!   id = (node << 7) | (cmd << 1) | err_bit; big-endian payloads; i24
//!   position ticks / i24 speed ticks-per-s / i16 current mA with
//!   DLC-variant position/velocity/current frames.
//! - SocketCAN backend (spectral module): bus bring-up, SO_SNDBUF sizing,
//!   round-robin telemetry poll + device-info sweep, boot config pacing,
//!   send errors PROPAGATED (vendor swallowed them — known production bug
//!   class).
//! - Sim backend ([`sim::SimBus`], closed loop): virtual Spectral drivers
//!   (cascade PID/PD from real config gains, current saturation, kt,
//!   watchdog) in front of a rate-limited kinematic plant with endstop /
//!   windup / hall emulation — or, behind feature `sim-dynamics`,
//!   Pinocchio ABA forward dynamics + friction + endstop torques, or,
//!   behind feature `sim-mujoco`, a full MuJoCo contact scene (gravity,
//!   endstops, physical grasps surfacing through the gripper status
//!   bits) — → encoder ticks at fixed dt. Homing stall/current detection
//!   works for real in CI.

mod bus;
mod loopback;
pub mod sim;
pub mod spectral;
mod types;

pub use bus::DriverBus;
pub use loopback::{LoopbackBus, Reply, TxRecord};
pub use types::{
    BusError, BusState, DeviceInfo, ErrorFlags, FirmwareGripperCommand, Freshness, GripperCommand,
    GripperReply, GripperState, HallState, JointCommand, LinkHealth, LinkState, NodeId, NodeState,
    ObjectDetection, Pack, PollAction, PollKind, MAX_NODES, NODE_BOOTLOADER, NODE_GRIPPER,
    NODE_HOST, NODE_TIMING_DUMMY,
};
