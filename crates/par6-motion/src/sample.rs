//! Planned-motion sample types, field-for-field the ring's `Sample` format.
//!
//! `par6-rt` (which owns the frozen SPSC ring contract) depends on this
//! crate, so the ring's `Sample`/`SampleMeta` cannot be imported here
//! without a dependency cycle. These types mirror them exactly — same
//! fields, same types, same semantics — and the EXEC glue in `par6-rt`
//! copies field-for-field when feeding the ring. A conformance test in
//! this crate (dev-dependency on `par6-rt`) pins the two formats together.

/// Compile-time arm joint count the motion types are dimensioned for
/// (PAR6: 6). Must equal `par6_rt::MAX_JOINTS`; config joint count is
/// checked against it at construction.
pub const NUM_JOINTS: usize = 6;

/// Per-sample segment metadata (mirror of `par6_rt::ring::SampleMeta`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SampleMeta {
    /// Queued command this sample belongs to.
    pub command_index: u32,
    /// Checkpoint label id; a CHANGE between consecutive samples is a
    /// checkpoint boundary (push completion for the previous label).
    pub checkpoint_id: u32,
    /// True while this sample's segment blends into the next command:
    /// at the boundary the completion policy must NOT settle.
    pub blend_continues: bool,
    /// Final sample of the queued program; EXEC completion runs after it.
    pub is_last: bool,
}

/// One tick of planned motion for all joints (mirror of
/// `par6_rt::ring::Sample`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    /// Joint positions \[rad\].
    pub q: [f64; NUM_JOINTS],
    /// Joint velocities \[rad/s\].
    pub qd: [f64; NUM_JOINTS],
    /// Torque feedforward \[Nm\] (gravity is NOT included — the RT adds
    /// G(q) itself). Planned inertial feedforward arrives with par6-kin
    /// dynamics; until then planners emit zero.
    pub tau_ff: [f32; NUM_JOINTS],
    /// Segment metadata.
    pub meta: SampleMeta,
}

impl Default for Sample {
    fn default() -> Self {
        Self {
            q: [0.0; NUM_JOINTS],
            qd: [0.0; NUM_JOINTS],
            tau_ff: [0.0; NUM_JOINTS],
            meta: SampleMeta::default(),
        }
    }
}
