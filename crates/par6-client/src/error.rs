//! Client-side error surface.

use par6_proto::{DecodeError, WireError};

/// What a client call can fail with. `Unreachable` is the "no reply after
/// every retry" outcome — a healthy way to probe for a runtime, not a bug —
/// while `Robot` carries the runtime's structured refusal.
#[derive(Debug, thiserror::Error)]
pub enum ClientError {
    /// No reply arrived within the retry budget.
    #[error("the runtime did not answer")]
    Unreachable,
    /// The runtime answered with a structured ERROR.
    #[error("[{}] {}: {}", .0.code, .0.title, .0.cause)]
    Robot(WireError),
    /// A reply arrived but could not be decoded.
    #[error("undecodable reply: {0}")]
    Decode(#[from] DecodeError),
    /// Socket-level failure.
    #[error(transparent)]
    Io(#[from] std::io::Error),
    /// The client has been closed.
    #[error("the client is closed")]
    Closed,
    /// A parameter failed client-side validation (mirrors the runtime's
    /// own checks so a bad call fails fast and offline).
    #[error("invalid parameter: {0}")]
    Invalid(String),
}

impl ClientError {
    /// The runtime's refusal, when that is what this error is.
    pub fn robot(&self) -> Option<&WireError> {
        match self {
            ClientError::Robot(e) => Some(e),
            _ => None,
        }
    }
}
