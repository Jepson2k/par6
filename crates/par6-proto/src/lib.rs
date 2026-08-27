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
pub mod telemetry;
mod wire;

pub use chunk::{
    decode_chunk, encode_chunk, split_into_chunks, Assembled, Chunk, ChunkError, Expired,
    Reassembler,
};
pub use command::{decode_command, encode_command, Command, Shape, ToolParam};
pub use enums::{
    command_class, ActionState, CmdType, CommandClass, CompletionPolicy, ControllerMode, Frame,
    HomingJointState, HomingPhase, LinkState, MsgType, QueryType, ToolState,
};
pub use error::{make_error, template, ErrorCode, ErrorTemplate, WireError, UNATTRIBUTED};
pub use reply::{decode_reply, encode_reply, LoopStatsResult, QueryResult, Reply, ToolStatusWire};
pub use status::{
    decode_status, encode_status_into, HomingWire, LinkHealthWire, Status, StatusEncoder,
    STATUS_HEADER_LEN, STATUS_LEN,
};
pub use telemetry::{
    decode_telemetry, encode_telemetry, TelemetryField, TelemetryFrame, TelemetryRecipe,
    TelemetryValue,
};

/// Protocol version carried in the STATUS header.
pub const PROTO_VERSION: u8 = 2;
/// Number of arm joints.
pub const NUM_JOINTS: usize = 6;
/// Elements in a flattened 4×4 row-major pose.
pub const POSE_ELEMS: usize = 16;
/// Digital I/O slots a STOCK control box publishes: seven inputs
/// (three isolated, four general-purpose), three isolated outputs, and
/// the e-stop.
///
/// NOT the wire arity. The STATUS `io` array is variable-length — the
/// runtime publishes the lines its `[io]` config declares, and a box
/// wired differently says so in the array's own length. This constant is
/// what the shipped config produces, and what decoders preallocate
/// before the first packet arrives.
///
/// One position is fixed whatever the length: **the e-stop is always the
/// LAST element**, read as `io[io.len() - 1]`.
pub const IO_SLOTS: usize = 11;
/// Decode ceiling on the STATUS `io` array.
///
/// A length header is attacker-controlled, so it is refused before
/// anything is reserved on its word (see [`command::r_len`]). The vendor
/// control box exposes ten lines; this leaves room for a much larger one
/// without letting a nine-byte datagram size an allocation.
pub const MAX_IO_SLOTS: usize = 64;
/// Enablement flag slots (6 joints/axes × 2 directions).
pub const EN_SLOTS: usize = 12;

/// A flattened row-major 4×4 from a translation and an intrinsic-XYZ
/// rotation `[rx, ry, rz]` in radians: `R = Rx(rx)·Ry(ry)·Rz(rz)`.
///
/// This is `pinokin.se3_from_rpy`'s order, which is the convention every
/// TCP pose on the wire uses — and NOT the extrinsic order keep-out
/// shapes are placed with (`par6_kin::Shape::pose`). The two agree on a
/// single-axis rotation and diverge on any multi-axis tilt, which is why
/// there is one builder here rather than a hand-derived matrix per
/// caller.
///
/// The translation passes through in whatever unit it arrives in: the
/// wire speaks millimetres and the kinematics stack metres, and the
/// rotation block is identical either way.
pub fn pose_matrix(xyz: [f64; 3], rpy_rad: [f64; 3]) -> [f64; POSE_ELEMS] {
    let (sr, cr) = rpy_rad[0].sin_cos();
    let (sp, cp) = rpy_rad[1].sin_cos();
    let (sy, cy) = rpy_rad[2].sin_cos();
    [
        cp * cy,
        -cp * sy,
        sp,
        xyz[0],
        sr * sp * cy + cr * sy,
        cr * cy - sr * sp * sy,
        -sr * cp,
        xyz[1],
        sr * sy - cr * sp * cy,
        cr * sp * sy + sr * cy,
        cr * cp,
        xyz[2],
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

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

/// Best-effort `req_id` salvage from a malformed datagram, so the decode
/// ERROR a server sends back still correlates with the request that
/// caused it; `None` when even the envelope is unreadable (callers
/// substitute 0, the push convention).
///
/// Deliberately hand-rolled rather than routed through [`Reader`]: the
/// input has ALREADY failed a real decode, so this walks only the two
/// msgpack uints it needs and gives up on anything else, instead of
/// re-running a parser that is known to reject the bytes.
pub fn peek_req_id(data: &[u8]) -> Option<u32> {
    fn uint(data: &[u8], pos: &mut usize) -> Option<u64> {
        let b = *data.get(*pos)?;
        *pos += 1;
        match b {
            0x00..=0x7f => Some(u64::from(b)),
            0xcc => {
                let v = *data.get(*pos)?;
                *pos += 1;
                Some(u64::from(v))
            }
            0xcd => {
                let v = u16::from_be_bytes(data.get(*pos..*pos + 2)?.try_into().ok()?);
                *pos += 2;
                Some(u64::from(v))
            }
            0xce => {
                let v = u32::from_be_bytes(data.get(*pos..*pos + 4)?.try_into().ok()?);
                *pos += 4;
                Some(u64::from(v))
            }
            0xcf => {
                let v = u64::from_be_bytes(data.get(*pos..*pos + 8)?.try_into().ok()?);
                *pos += 8;
                Some(v)
            }
            _ => None,
        }
    }
    let mut pos = match *data.first()? {
        0x90..=0x9f => 1usize,
        0xdc => 3,
        0xdd => 5,
        _ => return None,
    };
    uint(data, &mut pos)?; // tag
    u32::try_from(uint(data, &mut pos)?).ok()
}
