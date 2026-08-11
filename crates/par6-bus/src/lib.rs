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
//! - Spectral/STEPFOC frame codec (future issue): classic CAN 2.0A, 11-bit
//!   id = (node << 7) | (cmd << 1) | err_bit; big-endian payloads; i24
//!   position ticks / i24 speed ticks-per-s / i16 current mA with
//!   DLC-variant position/velocity/current frames.
//! - SocketCAN backend (future issue): bus bring-up, SO_SNDBUF sizing,
//!   round-robin telemetry poll + device-info sweep, boot config pacing,
//!   send errors PROPAGATED (vendor swallowed them — known production bug
//!   class).
//! - Sim backend (future issue, closed loop): virtual Spectral drivers
//!   (cascade PID/PD from real config gains, current saturation, kt,
//!   watchdog) + Pinocchio ABA forward dynamics + friction + endstop
//!   torques → encoder ticks at fixed dt. Homing stall/current detection
//!   works for real in CI. Tier 2 (feature `mujoco`): MuJoCo backend on
//!   the vendor MJCF.

mod bus;
mod loopback;
mod types;

pub use bus::DriverBus;
pub use loopback::{LoopbackBus, Reply, TxRecord};
pub use types::{
    BusError, BusState, DeviceInfo, ErrorFlags, FirmwareGripperCommand, Freshness, GripperCommand,
    GripperReply, GripperState, HallState, JointCommand, LinkHealth, LinkState, NodeId, NodeState,
    ObjectDetection, Pack, PollAction, PollKind, MAX_NODES, NODE_BOOTLOADER, NODE_GRIPPER,
    NODE_HOST, NODE_TIMING_DUMMY,
};
