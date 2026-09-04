//! Command plane (tokio; never on the RT thread).
//!
//! UDP msgpack protocol v2 server:
//!
//! - [`server`]: the actor — datagram dispatch with `req_id` echo, the
//!   declarative [`gating`] table, the SINGLE command-index allocator
//!   (monotonic `u64`, never reset), the motion queue with idempotency
//!   dedup (retries re-ack the ORIGINAL index), per-command COMPLETE
//!   pushes, streaming preemption (same-type in-place, type-change
//!   cancel+drain, planned move cancels streaming), stop/estop/reset
//!   cancel scopes, blend lookahead (a move with a corner radius is held
//!   briefly for the successor it rounds into, and the planner is
//!   offered the whole runnable chain), and chunked bulk reassembly with
//!   `COMM_CHUNK_TIMEOUT` expiry.
//! - [`link`]: the status broadcast transport ladder —
//!   multicast with a startup reachability probe, permanent unicast
//!   failover on probe failure or 3 consecutive send errors.
//! - [`faults`]: the RT error latch mapped onto the wire catalog, so a
//!   hard error the RT raised on its own (stream watchdog, loop critical,
//!   drive fault) reaches `STATUS.error`, `activity` and the ERROR query
//!   instead of leaving a DISABLED arm reporting itself idle.
//! - [`runtime`]: the two trait contracts `par6d` wires to
//!   `par6-motion` / `par6-rt` — [`Planner`] (queued command execution)
//!   and [`RtCommands`] (immediate effects) — plus the
//!   [`RuntimeHandle`] bundle that also carries the RT snapshot reader.
//!
//! STATUS broadcasts ALWAYS go out, even with a dead RT core or a
//! silent motor bus: `link_ok` / `data_age_ms` report motor-bus
//! freshness (the youngest node's `data_age_ticks`, aged by the
//! snapshot's wall age) instead of the server going silent.

#![warn(missing_docs)]

pub mod config;
pub mod faults;
pub mod gating;
pub mod link;
pub mod runtime;
pub mod server;
pub mod session;

pub use config::{ConfigInfoData, ServerConfig, StatusTransport, TunableNode};
pub use faults::{gripper_fault_code, rt_standing_error};
pub use gating::{check_gate, gate, Gate, GateContext};
pub use runtime::{
    blend_radius_mm, CollisionState, CommandOutcome, Enablement, PayloadSpec, PlanContext, Planner,
    QueuedCommand, RtCommands, RuntimeHandle, ShapeLayer,
};
pub use server::{
    cmd_name, decode_error_to_wire, pid_gains_fault, spawn, teleport_angle_fault,
    validate_registries, validate_supported, write_io_fault, ServerHandle,
};
