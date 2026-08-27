//! par6d — the PAR6 runtime daemon, assembled from the landed crates.
//!
//! Wiring (one [`Daemon`] instance):
//!
//! ```text
//!             UDP protocol v2                 CoreOp closures + RtCommand mpsc
//! client ◀──▶ par6-server task ── Planner ──▶ RT thread: RtCore<RuntimeBus>.run()
//!                    │            RtBridge ─▶   │  (SPSC sample ring, latest-wins
//!                    │                          │   stream slot, 1 cmd per tick)
//!                    ◀── snapshot tee thread ◀──┘  (triple-buffer fan-out)
//!                        housekeeping thread: jog watchdog, servo keep-alive,
//!                                             enable retry after clear-settle
//! ```
//!
//! - [`planner`] adapts `par6-motion` behind the server's `Planner` trait
//!   (move_j planning → sample ring → EXEC completion via the snapshot).
//! - [`bridge`] adapts immediate effects behind `RtCommands` (streams,
//!   e-stop latch, teleport re-seeding of the sim, backend switches).
//! - [`adapters`] puts the real `par6-motion` jog/stream engines behind
//!   the `par6-rt` per-tick hook traits.
//! - [`daemon`] owns thread spawn/wiring and clean shutdown;
//!   [`options`] the CLI/env surface.
//!
//! The library exists so integration tests can boot the full stack
//! in-process on ephemeral ports; `main.rs` is a thin CLI wrapper.

#![warn(missing_docs)]

mod adapters;
mod bridge;
mod collision_world;
pub mod daemon;
mod grant;
mod kin;
pub mod options;
mod planner;
pub mod preview;

pub use daemon::{Daemon, DaemonError};
pub use options::Options;
