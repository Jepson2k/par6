//! Protocol v2 codec — the single source of truth for the wire contract
//! between the `par6` Python client and the `par6d` runtime.
//!
//! See `spec/PROTOCOL-V2.md`. Semantics inherit from the parol6 protocol
//! (modeless int-tagged msgpack commands, ack taxonomy, accepted/executing/
//! completed index triple) with the v2 fixes: request-id correlation,
//! idempotent queued commands, status header (seq / timestamp / controller
//! id / version), always-broadcast staleness reporting.
//!
//! The Python constants mirror (`python/par6/protocol/constants.py`) is
//! generated from this crate; golden vectors under `tests/golden/` are the
//! cross-language conformance suite. Contract changes require a
//! `contracts`-labeled issue (see README workflow).
