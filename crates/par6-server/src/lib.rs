//! Command plane (tokio; never on the RT thread).
//!
//! UDP msgpack protocol v2 server per `spec/PROTOCOL-V2.md`:
//!
//! - [`server`]: the actor — datagram dispatch with `req_id` echo, the
//!   declarative [`gating`] table, the SINGLE command-index allocator
//!   (monotonic `u64`, never reset), the motion queue with idempotency
//!   dedup (retries re-ack the ORIGINAL index), per-command COMPLETE
//!   pushes, streaming preemption (same-type in-place, type-change
//!   cancel+drain, planned move cancels streaming), stop/estop/reset
//!   cancel scopes, and chunked bulk reassembly with
//!   `COMM_CHUNK_TIMEOUT` expiry.
//! - [`link`]: the status/telemetry broadcast transport ladder —
//!   multicast with a startup reachability probe, permanent unicast
//!   failover on probe failure or 3 consecutive send errors.
//! - [`telemetry`]: recipe-selected binary msgpack field streams;
//!   unknown recipe names are refused (`COMM_UNKNOWN_RECIPE`).
//! - [`runtime`]: the two trait contracts `par6d` wires to
//!   `par6-motion` / `par6-rt` — [`Planner`] (queued command execution)
//!   and [`RtCommands`] (immediate effects) — plus the
//!   [`RuntimeHandle`] bundle that also carries the RT snapshot reader.
//!
//! STATUS broadcasts ALWAYS go out, even with a dead RT core:
//! `link_ok` / `data_age_ms` report snapshot staleness instead of the
//! server going silent.

#![warn(missing_docs)]

pub mod config;
pub mod gating;
pub mod link;
pub mod runtime;
pub mod server;
pub mod telemetry;

pub use config::{ServerConfig, StatusTransport};
pub use gating::{gate, Gate};
pub use runtime::{CommandOutcome, Enablement, PlanContext, Planner, RtCommands, RuntimeHandle};
pub use server::{spawn, ServerHandle};
pub use telemetry::{TelemetryField, TelemetryRecipe};
