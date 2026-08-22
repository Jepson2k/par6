//! Telemetry consumer: receive the daemon's telemetry stream and label
//! each packet's values with its recipe's fields.
//!
//! The packet codec and the recipe registry live in
//! [`par6_proto::telemetry`]; this reader owns the socket (the same
//! multicast-with-unicast-fallback ladder as the STATUS stream) and the
//! registry lookup that turns positional values into named fields.

use std::net::{Ipv4Addr, UdpSocket};
use std::time::{Duration, Instant};

use par6_proto::telemetry::{decode_telemetry, TelemetryRecipe, TelemetryValue};

use crate::error::ClientError;
use crate::sockets;
use crate::StatusTransport;

/// One received telemetry packet with its values labeled by field key.
#[derive(Debug, Clone, PartialEq)]
pub struct TelemetryPacket {
    /// Recipe the packet was encoded under.
    pub recipe: String,
    /// Monotone packet sequence number.
    pub seq: u64,
    /// Sender's monotonic clock \[ns\].
    pub mono_time_ns: u64,
    /// `(field key, value)` in recipe order — the recipe's own field
    /// keys, from the registry.
    pub fields: Vec<(&'static str, TelemetryValue)>,
}

/// Blocking telemetry receiver over the registry.
pub struct TelemetryReader {
    sock: UdpSocket,
    recipes: Vec<TelemetryRecipe>,
    buf: Vec<u8>,
}

impl TelemetryReader {
    /// Open a reader on the telemetry stream with the stock recipe
    /// registry, using the same transport ladder as the STATUS stream.
    pub fn open(transport: StatusTransport, port: u16) -> Result<Self, ClientError> {
        let sock = match transport {
            StatusTransport::Multicast { group, iface } => {
                sockets::multicast_socket(group, port, iface)
                    .or_else(|_| sockets::unicast_socket(Ipv4Addr::LOCALHOST, port))
            }
            StatusTransport::Unicast { host } => sockets::unicast_socket(host, port),
        }?;
        Ok(Self::over(sock))
    }

    /// A reader over an already-bound socket (must be non-blocking).
    pub fn over(sock: UdpSocket) -> Self {
        Self {
            sock,
            recipes: TelemetryRecipe::defaults(),
            buf: vec![0u8; 65536],
        }
    }

    /// Receive and decode the next packet, waiting up to `timeout`.
    /// `None` = nothing arrived in time (the stream may simply be
    /// silent — no recipe active).
    pub fn recv(&mut self, timeout: Duration) -> Result<Option<TelemetryPacket>, ClientError> {
        let deadline = Instant::now() + timeout;
        loop {
            match self.sock.recv(&mut self.buf) {
                Ok(n) => {
                    let raw = &self.buf[..n];
                    let pkt = Self::label(&self.recipes, raw)?;
                    return Ok(Some(pkt));
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    let now = Instant::now();
                    if now >= deadline {
                        return Ok(None);
                    }
                    // A UDP stream ticking at 100 Hz doesn't warrant an
                    // epoll registration; a short sleep bounds the poll
                    // latency well under the packet interval.
                    std::thread::sleep(Duration::from_millis(1).min(deadline - now));
                }
                Err(e) => return Err(e.into()),
            }
        }
    }

    /// Every packet currently waiting on the socket, oldest first.
    pub fn drain(&mut self) -> Result<Vec<TelemetryPacket>, ClientError> {
        let mut out = Vec::new();
        loop {
            match self.sock.recv(&mut self.buf) {
                Ok(n) => {
                    let raw = &self.buf[..n];
                    out.push(Self::label(&self.recipes, raw)?);
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => return Ok(out),
                Err(e) => return Err(e.into()),
            }
        }
    }

    fn label(recipes: &[TelemetryRecipe], raw: &[u8]) -> Result<TelemetryPacket, ClientError> {
        let frame = decode_telemetry(raw).map_err(ClientError::Decode)?;
        let recipe = recipes
            .iter()
            .find(|r| r.name == frame.recipe)
            .ok_or_else(|| ClientError::Invalid(format!("unknown recipe {:?}", frame.recipe)))?;
        if recipe.fields.len() != frame.values.len() {
            return Err(ClientError::Invalid(format!(
                "recipe {:?} carries {} values, registry expects {}",
                frame.recipe,
                frame.values.len(),
                recipe.fields.len()
            )));
        }
        Ok(TelemetryPacket {
            recipe: frame.recipe,
            seq: frame.seq,
            mono_time_ns: frame.mono_time_ns,
            fields: recipe
                .fields
                .iter()
                .map(|f| f.key())
                .zip(frame.values)
                .collect(),
        })
    }
}
