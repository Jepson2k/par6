//! Command plane (tokio; never on the RT thread).
//!
//! UDP msgpack protocol v2 server: declarative validation/gating table,
//! motion queue, the SINGLE command-index allocator, per-command push
//! completion, idempotency dedup window for queued commands, status
//! broadcaster (multicast with unicast failover; always broadcasts, carries
//! link_ok/data_age_ms instead of going silent), telemetry recipes.
//! Planner handoff: SPSC sample ring into par6-rt.
