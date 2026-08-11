//! Bus abstraction and backends.
//!
//! - `DriverBus` trait: what the RT loop needs from a motor bus, backend-
//!   agnostic (per-tick send of per-joint setpoints, RX drain, telemetry
//!   poll scheduling, boot config sequence, health/freshness).
//! - Spectral/STEPFOC frame codec: classic CAN 2.0A, 11-bit id =
//!   (node << 7) | (cmd << 1) | err_bit; big-endian payloads; i24 position
//!   ticks / i24 speed ticks-per-s / i16 current mA with DLC-variant
//!   position/velocity/current frames. Channel omission is semantic:
//!   `Option<T>` — None = channel omitted on the wire (NOT zero).
//!   FD-ready framing kept in mind, classic-only today. See `spec/CAN.md`.
//! - SocketCAN backend: bus bring-up, SO_SNDBUF sizing, freshness
//!   (10-tick warn / 50-tick latch), round-robin telemetry poll, boot
//!   config sequence with pacing, send errors PROPAGATED (vendor swallowed
//!   them — known production bug class).
//! - Sim backend (closed loop): virtual Spectral drivers (cascade PID/PD
//!   from real config gains, current saturation, kt, watchdog) + Pinocchio
//!   ABA forward dynamics + friction + endstop torques → encoder ticks at
//!   fixed dt. Homing stall/current detection works for real in CI.
//!   Tier 2 (feature `mujoco`): MuJoCo backend on the vendor MJCF.
