//! Rust client library for the par6d runtime (protocol v2).
//!
//! The engine's own client: an async tokio UDP client over the frozen wire
//! layer in [`par6_proto`], plus a blocking wrapper. The Python package is
//! a binding over this crate — logic lives here, once.

#![warn(missing_docs)]

mod api;
mod core;
mod error;
mod sockets;
mod sync;

pub use crate::api::{freedrive, MotionWait};
pub use crate::core::{Ack, Client, ClientConfig, Completion, StatusTransport, MIN_MTU};
pub use crate::error::ClientError;
pub use crate::sync::SyncClient;

pub use par6_proto::{
    ActionState, CompletionPolicy, ControllerMode, ErrorCode, Frame, LoopStatsResult, QueryResult,
    Shape, Status, ToolStatusWire, WireError, NUM_JOINTS,
};
