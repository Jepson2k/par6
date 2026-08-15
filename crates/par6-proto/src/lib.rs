//! Protocol v2 codec — the single source of truth for the wire contract
//! between the `par6` Python client and the `par6d` runtime.
//!
//! Semantics inherit from the parol6 protocol
//! (modeless int-tagged msgpack commands, ack taxonomy, accepted/executing/
//! completed index triple) with the v2 fixes: request-id correlation,
//! idempotent queued commands, status header (seq / timestamp / controller
//! id / version), always-broadcast staleness reporting, chunked bulk
//! payloads, and a single `nil` unspecified-value convention.
//!
//! # Layout
//!
//! | module | contents |
//! |---|---|
//! | [`enums`] | tags ([`MsgType`], [`CmdType`], [`QueryType`]), small value enums, [`command_class`] |
//! | [`command`] | client→server commands: param structs, validation, encode/decode |
//! | [`reply`] | server→client OK/ERROR/RESPONSE/COMPLETE |
//! | [`error`] | [`ErrorCode`] + KUKA-style catalog ([`make_error`]) |
//! | [`status`] | broadcast STATUS packet, reusable-buffer encoder |
//! | [`chunk`] | chunked bulk envelope + [`Reassembler`] |
//! | [`pygen`] | generator for the Python constants mirror |
//! | [`golden`] | golden-vector definitions (cross-language conformance) |
//!
//! The Python constants mirror (`python/par6/protocol/constants.py`) is
//! generated from this crate (`cargo run -p par6-proto --bin gen_python`);
//! golden vectors under `tests/golden/protocol/` are the cross-language
//! conformance suite (`cargo run -p par6-proto --bin gen_golden`). Tests in
//! this crate fail if either is stale. Contract changes require a
//! `contracts`-labeled issue (see README workflow).

#![warn(missing_docs)]

#[macro_use]
mod macros;

pub mod chunk;
pub mod command;
pub mod enums;
pub mod error;
pub mod golden;
pub mod pygen;
pub mod reply;
pub mod status;
mod wire;

pub use chunk::{
    decode_chunk, encode_chunk, split_into_chunks, Assembled, Chunk, ChunkError, Expired,
    Reassembler,
};
pub use command::{decode_command, encode_command, Command, Shape, ToolParam};
pub use enums::{
    command_class, ActionState, CmdType, CommandClass, CompletionPolicy, Frame, MsgType, QueryType,
    ToolState,
};
pub use error::{make_error, template, ErrorCode, ErrorTemplate, WireError, UNATTRIBUTED};
pub use reply::{decode_reply, encode_reply, LoopStatsResult, QueryResult, Reply, ToolStatusWire};
pub use status::{
    decode_status, encode_status_into, Status, StatusEncoder, STATUS_HEADER_LEN, STATUS_LEN,
};

/// Protocol version carried in the STATUS header.
pub const PROTO_VERSION: u8 = 2;
/// Number of arm joints.
pub const NUM_JOINTS: usize = 6;
/// Elements in a flattened 4×4 row-major pose.
pub const POSE_ELEMS: usize = 16;
/// Digital I/O slots in STATUS: `[in1, in2, out1, out2, estop]`.
pub const IO_SLOTS: usize = 5;
/// Enablement flag slots (6 joints/axes × 2 directions).
pub const EN_SLOTS: usize = 12;

/// Why a payload failed to decode (or a command failed validation).
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum DecodeError {
    /// The buffer ended mid-value.
    #[error("truncated message")]
    Truncated,
    /// A value had the wrong msgpack type.
    #[error("expected {expected} at byte {pos}, found marker 0x{found:02x}")]
    Type {
        /// What the decoder was reading.
        expected: &'static str,
        /// The marker byte found instead.
        found: u8,
        /// Byte offset of the marker.
        pos: usize,
    },
    /// An array had the wrong element count.
    #[error("{what}: expected {expected} elements, got {got}")]
    Arity {
        /// Which array.
        what: &'static str,
        /// Expected element count.
        expected: usize,
        /// Actual element count.
        got: usize,
    },
    /// Slot 0 carried a tag this codec doesn't know.
    #[error("unknown tag {0}")]
    UnknownTag(i64),
    /// An enum-valued field carried an unknown value.
    #[error("invalid {what} value {value}")]
    InvalidEnum {
        /// Which field.
        what: &'static str,
        /// The offending value.
        value: i64,
    },
    /// A string field was not valid UTF-8.
    #[error("invalid utf-8 in string")]
    Utf8,
    /// A field failed range/shape validation.
    #[error("{what}: {why}")]
    Validation {
        /// Which field.
        what: &'static str,
        /// What rule it broke.
        why: String,
    },
    /// Bytes remained after the top-level array.
    #[error("trailing bytes after message")]
    TrailingBytes,
}

/// Peek the integer tag in slot 0 without decoding the full payload.
///
/// Servers use this to route datagrams (command vs chunk); clients to route
/// replies vs status broadcasts.
pub fn peek_tag(data: &[u8]) -> Result<i64, DecodeError> {
    let mut r = wire::Reader::new(data);
    let n = r.array_len()?;
    if n == 0 {
        return Err(DecodeError::Arity {
            what: "envelope",
            expected: 1,
            got: 0,
        });
    }
    r.int()
}
