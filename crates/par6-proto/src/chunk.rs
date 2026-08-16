//! Chunked bulk envelope + reassembler.
//!
//! Commands whose params can exceed one datagram (MOVE_S / MOVE_P waypoint
//! lists, SET_SHAPES) are split into `[CHUNK, req_id, transfer_id u32, i u16,
//! n u16, bytes]` datagrams. The concatenated payload is the complete inner
//! command datagram (`[cmd_tag, req_id, ...params]`) exactly as it would have
//! been sent unchunked.
//!
//! The [`Reassembler`] is clock-agnostic: callers pass `now` into
//! [`Reassembler::push`] and periodically call [`Reassembler::expire`], which
//! reports timed-out transfers so the server can answer them with
//! `COMM_CHUNK_TIMEOUT`.

use std::collections::BTreeMap;
use std::collections::HashMap;
use std::time::{Duration, Instant};

use crate::enums::MsgType;
use crate::wire::{w_array, w_bin, w_uint, Reader};
use crate::DecodeError;

/// Reassembly cap: a transfer larger than this is rejected outright
/// (memory-exhaustion guard; generous next to any realistic waypoint list).
pub const MAX_TRANSFER_BYTES: usize = 4 * 1024 * 1024;

/// How many transfers may be in flight at once.
///
/// The command socket is unauthenticated, so the number of transfers a
/// sender can open is as much an input as the bytes inside them. Real
/// clients run one bulk transfer at a time; a handful of slots covers
/// retries and overlapping senders without letting the table grow on a
/// stranger's say-so.
pub const MAX_TRANSFERS_IN_FLIGHT: usize = 8;

/// One chunk of a bulk transfer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Chunk {
    /// Request id of the inner command (echoed on chunk-level errors).
    pub req_id: u32,
    /// Client-generated transfer id, unique per in-flight transfer.
    pub transfer_id: u32,
    /// Chunk index, `0..n`.
    pub index: u16,
    /// Total chunk count (≥ 1, identical across the transfer).
    pub total: u16,
    /// This chunk's slice of the payload.
    pub data: Vec<u8>,
}

/// Encode a chunk envelope into `buf` (cleared first).
pub fn encode_chunk(c: &Chunk, buf: &mut Vec<u8>) {
    buf.clear();
    w_array(buf, 6);
    w_uint(buf, u64::from(MsgType::Chunk as u8));
    w_uint(buf, u64::from(c.req_id));
    w_uint(buf, u64::from(c.transfer_id));
    w_uint(buf, u64::from(c.index));
    w_uint(buf, u64::from(c.total));
    w_bin(buf, &c.data);
}

/// Decode a chunk envelope.
pub fn decode_chunk(data: &[u8]) -> Result<Chunk, DecodeError> {
    let mut r = Reader::new(data);
    let n = r.array_len()?;
    if n != 6 {
        return Err(DecodeError::Arity {
            what: "chunk envelope",
            expected: 6,
            got: n,
        });
    }
    let raw = r.int()?;
    if raw != MsgType::Chunk as i64 {
        return Err(DecodeError::UnknownTag(raw));
    }
    let req_id = u32::try_from(r.uint()?).map_err(|_| DecodeError::Validation {
        what: "chunk.req_id",
        why: "exceeds u32".into(),
    })?;
    let transfer_id = u32::try_from(r.uint()?).map_err(|_| DecodeError::Validation {
        what: "chunk.transfer_id",
        why: "exceeds u32".into(),
    })?;
    let index = u16::try_from(r.uint()?).map_err(|_| DecodeError::Validation {
        what: "chunk.index",
        why: "exceeds u16".into(),
    })?;
    let total = u16::try_from(r.uint()?).map_err(|_| DecodeError::Validation {
        what: "chunk.total",
        why: "exceeds u16".into(),
    })?;
    let payload = r.bin()?.to_vec();
    r.finish()?;
    if total == 0 {
        return Err(DecodeError::Validation {
            what: "chunk.total",
            why: "must be >= 1".into(),
        });
    }
    if index >= total {
        return Err(DecodeError::Validation {
            what: "chunk.index",
            why: "must be < total".into(),
        });
    }
    Ok(Chunk {
        req_id,
        transfer_id,
        index,
        total,
        data: payload,
    })
}

/// A chunk was inconsistent with its transfer.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ChunkError {
    /// `total` changed between chunks of one transfer.
    #[error("chunk total mismatch: transfer started with {expected}, got {got}")]
    TotalMismatch {
        /// `total` from the transfer's first chunk.
        expected: u16,
        /// `total` on the offending chunk.
        got: u16,
    },
    /// Too many transfers are already open.
    #[error("too many chunk transfers in flight ({in_flight})")]
    TooManyTransfers {
        /// Transfers open when the new one was refused.
        in_flight: usize,
    },
    /// `req_id` changed between chunks of one transfer.
    #[error("chunk req_id mismatch: transfer started with {expected}, got {got}")]
    ReqIdMismatch {
        /// `req_id` from the transfer's first chunk.
        expected: u32,
        /// `req_id` on the offending chunk.
        got: u32,
    },
    /// Reassembled size would exceed [`MAX_TRANSFER_BYTES`].
    #[error("transfer exceeds {MAX_TRANSFER_BYTES} bytes")]
    TooLarge,
}

/// A completed transfer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Assembled {
    /// Request id of the inner command.
    pub req_id: u32,
    /// The transfer that completed.
    pub transfer_id: u32,
    /// The reassembled inner datagram (feed to `decode_command`).
    pub payload: Vec<u8>,
}

/// An expired transfer, reported by [`Reassembler::expire`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Expired {
    /// Request id of the inner command (for the `COMM_CHUNK_TIMEOUT` reply).
    pub req_id: u32,
    /// The transfer that timed out.
    pub transfer_id: u32,
    /// Chunks received before the timeout.
    pub received: u16,
    /// Chunks expected.
    pub total: u16,
}

struct Transfer {
    req_id: u32,
    total: u16,
    bytes: usize,
    /// Received chunks by index. Keyed rather than pre-sized from
    /// `total`, so the table costs what has actually arrived — a
    /// 65 535-slot `Vec` would have cost ~1.57 MB on the strength of one
    /// ~20-byte datagram's `total` field. `BTreeMap` keeps the indices
    /// ordered for reassembly.
    parts: BTreeMap<u16, Vec<u8>>,
    last_activity: Instant,
}

/// Server-side chunk reassembler with a per-transfer inactivity timeout.
pub struct Reassembler {
    timeout: Duration,
    transfers: HashMap<u32, Transfer>,
}

impl Reassembler {
    /// New reassembler; `timeout` is per-transfer inactivity.
    pub fn new(timeout: Duration) -> Self {
        Reassembler {
            timeout,
            transfers: HashMap::new(),
        }
    }

    /// Feed one chunk. Returns the completed transfer once every chunk has
    /// arrived (in any order; duplicates are idempotent).
    pub fn push(&mut self, chunk: Chunk, now: Instant) -> Result<Option<Assembled>, ChunkError> {
        if !self.transfers.contains_key(&chunk.transfer_id)
            && self.transfers.len() >= MAX_TRANSFERS_IN_FLIGHT
        {
            return Err(ChunkError::TooManyTransfers {
                in_flight: self.transfers.len(),
            });
        }
        let t = self
            .transfers
            .entry(chunk.transfer_id)
            .or_insert_with(|| Transfer {
                req_id: chunk.req_id,
                total: chunk.total,
                bytes: 0,
                parts: BTreeMap::new(),
                last_activity: now,
            });
        if t.total != chunk.total {
            let expected = t.total;
            self.transfers.remove(&chunk.transfer_id);
            return Err(ChunkError::TotalMismatch {
                expected,
                got: chunk.total,
            });
        }
        if t.req_id != chunk.req_id {
            let expected = t.req_id;
            self.transfers.remove(&chunk.transfer_id);
            return Err(ChunkError::ReqIdMismatch {
                expected,
                got: chunk.req_id,
            });
        }
        t.last_activity = now;
        if !t.parts.contains_key(&chunk.index) {
            if t.bytes + chunk.data.len() > MAX_TRANSFER_BYTES {
                self.transfers.remove(&chunk.transfer_id);
                return Err(ChunkError::TooLarge);
            }
            t.bytes += chunk.data.len();
            t.parts.insert(chunk.index, chunk.data);
        }
        if t.parts.len() == usize::from(t.total) {
            let t = self.transfers.remove(&chunk.transfer_id).expect("present");
            let mut payload = Vec::with_capacity(t.bytes);
            for (_, part) in t.parts {
                payload.extend_from_slice(&part);
            }
            return Ok(Some(Assembled {
                req_id: t.req_id,
                transfer_id: chunk.transfer_id,
                payload,
            }));
        }
        Ok(None)
    }

    /// Drop transfers idle longer than the timeout, reporting each so the
    /// caller can send `COMM_CHUNK_TIMEOUT` to the client.
    pub fn expire(&mut self, now: Instant) -> Vec<Expired> {
        let timeout = self.timeout;
        let expired_ids: Vec<u32> = self
            .transfers
            .iter()
            .filter(|(_, t)| now.duration_since(t.last_activity) >= timeout)
            .map(|(id, _)| *id)
            .collect();
        expired_ids
            .into_iter()
            .map(|id| {
                let t = self.transfers.remove(&id).expect("present");
                Expired {
                    req_id: t.req_id,
                    transfer_id: id,
                    received: t.parts.len() as u16,
                    total: t.total,
                }
            })
            .collect()
    }

    /// Number of in-flight transfers.
    pub fn in_flight(&self) -> usize {
        self.transfers.len()
    }
}

/// Split an inner command datagram into `chunk_size`-byte chunks for sending.
/// Always yields at least one chunk.
pub fn split_into_chunks(
    req_id: u32,
    transfer_id: u32,
    payload: &[u8],
    chunk_size: usize,
) -> Vec<Chunk> {
    assert!(chunk_size > 0, "chunk_size must be > 0");
    let parts: Vec<&[u8]> = if payload.is_empty() {
        vec![&[]]
    } else {
        payload.chunks(chunk_size).collect()
    };
    let total = parts.len() as u16;
    parts
        .into_iter()
        .enumerate()
        .map(|(i, p)| Chunk {
            req_id,
            transfer_id,
            index: i as u16,
            total,
            data: p.to_vec(),
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mk(transfer_id: u32, index: u16, total: u16, data: &[u8]) -> Chunk {
        Chunk {
            req_id: 9,
            transfer_id,
            index,
            total,
            data: data.to_vec(),
        }
    }

    #[test]
    fn reassembles_out_of_order_with_duplicates() {
        let mut ra = Reassembler::new(Duration::from_secs(1));
        let t0 = Instant::now();
        assert_eq!(ra.push(mk(1, 2, 3, b"cc"), t0).unwrap(), None);
        assert_eq!(ra.push(mk(1, 0, 3, b"aa"), t0).unwrap(), None);
        // duplicate of an already-received chunk is idempotent
        assert_eq!(ra.push(mk(1, 0, 3, b"aa"), t0).unwrap(), None);
        let done = ra.push(mk(1, 1, 3, b"bb"), t0).unwrap().unwrap();
        assert_eq!(done.payload, b"aabbcc");
        assert_eq!(done.req_id, 9);
        assert_eq!(ra.in_flight(), 0);
    }

    /// The command socket is unauthenticated, so both the bytes inside a
    /// transfer and the number of transfers opened are attacker-chosen.
    /// Neither may be able to size an allocation.
    #[test]
    fn a_transfer_costs_what_arrived_not_what_it_claims() {
        let mut ra = Reassembler::new(Duration::from_secs(2));
        let t0 = Instant::now();

        // One ~20-byte datagram claiming the largest possible chunk count
        // must not reserve a slot table for it: the old `vec![None; total]`
        // cost ~1.57 MB apiece, before a single byte of payload arrived.
        assert_eq!(ra.push(mk(1, 0, u16::MAX, b"aa"), t0).unwrap(), None);
        assert_eq!(ra.in_flight(), 1);

        // And the transfer table itself is bounded: past the cap a NEW
        // transfer is refused rather than admitted.
        for id in 2..=(MAX_TRANSFERS_IN_FLIGHT as u32) {
            assert_eq!(ra.push(mk(id, 0, u16::MAX, b"aa"), t0).unwrap(), None);
        }
        assert_eq!(ra.in_flight(), MAX_TRANSFERS_IN_FLIGHT);
        let over = ra.push(mk(9999, 0, u16::MAX, b"aa"), t0);
        assert!(
            matches!(over, Err(ChunkError::TooManyTransfers { .. })),
            "a transfer past the cap must be refused, got {over:?}"
        );
        assert_eq!(
            ra.in_flight(),
            MAX_TRANSFERS_IN_FLIGHT,
            "the refused transfer must not be admitted"
        );

        // An already-open transfer still accepts chunks at the cap.
        assert_eq!(ra.push(mk(1, 1, u16::MAX, b"bb"), t0).unwrap(), None);
    }

    #[test]
    fn expire_reports_and_drops_stale_transfers() {
        let mut ra = Reassembler::new(Duration::from_millis(100));
        let t0 = Instant::now();
        ra.push(mk(1, 0, 2, b"aa"), t0).unwrap();
        assert!(ra.expire(t0 + Duration::from_millis(50)).is_empty());
        let expired = ra.expire(t0 + Duration::from_millis(150));
        assert_eq!(expired.len(), 1);
        assert_eq!(expired[0].transfer_id, 1);
        assert_eq!(expired[0].received, 1);
        assert_eq!(expired[0].total, 2);
        assert_eq!(ra.in_flight(), 0);
        // a late chunk after expiry starts a fresh (incomplete) transfer
        assert_eq!(ra.push(mk(1, 1, 2, b"bb"), t0).unwrap(), None);
        assert_eq!(ra.in_flight(), 1);
    }

    #[test]
    fn total_mismatch_rejects_and_drops_the_transfer() {
        let mut ra = Reassembler::new(Duration::from_secs(1));
        let t0 = Instant::now();
        ra.push(mk(1, 0, 3, b"aa"), t0).unwrap();
        let err = ra.push(mk(1, 1, 4, b"bb"), t0).unwrap_err();
        assert_eq!(
            err,
            ChunkError::TotalMismatch {
                expected: 3,
                got: 4
            }
        );
        assert_eq!(ra.in_flight(), 0);
    }

    #[test]
    fn split_and_reassemble_roundtrip() {
        let payload: Vec<u8> = (0..=255u8).cycle().take(1000).collect();
        let chunks = split_into_chunks(7, 42, &payload, 300);
        assert_eq!(chunks.len(), 4);
        let mut ra = Reassembler::new(Duration::from_secs(1));
        let t0 = Instant::now();
        let mut done = None;
        for c in chunks {
            // encode/decode roundtrip on every envelope
            let mut buf = Vec::new();
            encode_chunk(&c, &mut buf);
            let back = decode_chunk(&buf).unwrap();
            assert_eq!(back, c);
            if let Some(a) = ra.push(back, t0).unwrap() {
                done = Some(a);
            }
        }
        assert_eq!(done.unwrap().payload, payload);
    }
}
