//! Telemetry: recipe-selected field streams from the RT snapshot.
//!
//! The field/recipe identity and the packet codec live in
//! [`par6_proto::telemetry`]; this module is the server half — extracting
//! each field's value from the [`StateSnapshot`] and encoding the packet.

use par6_rt::{StateSnapshot, MAX_JOINTS, NUM_NODES};

use par6_proto::telemetry::{encode_telemetry_into, TelemetryValueRef};
pub use par6_proto::telemetry::{TelemetryField, TelemetryRecipe};

/// Where one field's readings land before encoding: the scalar itself,
/// or how many entries of the caller's array were filled.
enum Slot {
    U64(u64),
    F64(f64),
    Arr(usize),
}

fn joints(a: &[f64; MAX_JOINTS], out: &mut [f64; NUM_NODES]) -> Slot {
    out[..MAX_JOINTS].copy_from_slice(a);
    Slot::Arr(MAX_JOINTS)
}

fn nodes_f64(f: impl Fn(usize) -> Option<f64>, out: &mut [f64; NUM_NODES]) -> Slot {
    for (i, v) in out.iter_mut().enumerate() {
        *v = f(i).unwrap_or(f64::NAN);
    }
    Slot::Arr(NUM_NODES)
}

/// Read `field` out of `snap`; array fields are written into `out`.
fn fill(field: TelemetryField, snap: &StateSnapshot, out: &mut [f64; NUM_NODES]) -> Slot {
    use TelemetryField as F;
    match field {
        F::Tick => Slot::U64(snap.tick),
        F::MeasuredPositions => joints(&snap.q, out),
        F::MeasuredVelocities => joints(&snap.qd, out),
        F::MeasuredTorques => joints(&snap.tau, out),
        F::FilteredPositions => joints(&snap.q_filtered, out),
        F::FilteredVelocities => joints(&snap.qd_filtered, out),
        F::FilteredTorques => joints(&snap.tau_filtered, out),
        F::CommandedPositions => joints(&snap.q_commanded, out),
        F::CommandedVelocities => joints(&snap.qd_commanded, out),
        F::CommandedTorques => joints(&snap.tau_commanded, out),
        F::TargetPositions => joints(&snap.q_target, out),
        F::TargetVelocities => joints(&snap.qd_target, out),
        F::GravityTorques => joints(&snap.gravity_torque_nm, out),
        F::ExternalTorques => joints(&snap.tau_ext, out),
        F::MotorTemperaturesC => nodes_f64(|i| snap.nodes[i].temperature_c.map(f64::from), out),
        F::MotorVoltagesMv => nodes_f64(|i| snap.nodes[i].voltage_mv.map(f64::from), out),
        F::MotorCurrentsMa => nodes_f64(|i| snap.nodes[i].current_ma.map(f64::from), out),
        F::LoopPeriodEmaS => Slot::F64(snap.loop_stats.period_ema_s),
        F::LoopP99S => Slot::F64(snap.loop_stats.p99_s),
        F::LoopOverruns => Slot::U64(u64::from(snap.loop_stats.overruns)),
        F::GripperPosition => Slot::F64(
            snap.gripper
                .reply
                .map_or(f64::NAN, |r| f64::from(r.position) / 255.0),
        ),
        F::GripperCurrentMa => Slot::F64(
            snap.gripper
                .reply
                .map_or(f64::NAN, |r| f64::from(r.current_ma)),
        ),
        F::GripperObjectDetection => Slot::F64(
            snap.gripper
                .reply
                .map_or(f64::NAN, |r| f64::from(r.object_detection as u8)),
        ),
        F::GripperFault => Slot::U64(u64::from(
            u32::try_from(crate::faults::gripper_fault_code(snap)).unwrap_or(0),
        )),
    }
}

/// Per-field reading buffers, one row per possible recipe field,
/// allocated once so the 100 Hz packet path reuses them.
pub struct TelemetryScratch {
    arrays: Vec<[f64; NUM_NODES]>,
}

impl Default for TelemetryScratch {
    fn default() -> Self {
        Self::new()
    }
}

impl TelemetryScratch {
    /// One row per possible recipe field, so any recipe fits.
    pub fn new() -> Self {
        Self {
            arrays: vec![[0.0; NUM_NODES]; TelemetryField::ALL.len()],
        }
    }
}

/// Encode one telemetry packet for `recipe` from `snap` into `buf`.
pub fn encode_packet_into(
    buf: &mut Vec<u8>,
    recipe: &TelemetryRecipe,
    seq: u64,
    mono_time_ns: u64,
    snap: &StateSnapshot,
    scratch: &mut TelemetryScratch,
) {
    let n = recipe.fields.len().min(scratch.arrays.len());
    let mut slots: [Slot; TelemetryField::ALL.len()] = std::array::from_fn(|_| Slot::U64(0));
    for (i, field) in recipe.fields.iter().take(n).enumerate() {
        slots[i] = fill(*field, snap, &mut scratch.arrays[i]);
    }
    let mut refs: [TelemetryValueRef<'_>; TelemetryField::ALL.len()] =
        [TelemetryValueRef::U64(0); TelemetryField::ALL.len()];
    for (i, slot) in slots.iter().take(n).enumerate() {
        refs[i] = match slot {
            Slot::U64(v) => TelemetryValueRef::U64(*v),
            Slot::F64(v) => TelemetryValueRef::F64(*v),
            Slot::Arr(len) => TelemetryValueRef::Arr(&scratch.arrays[i][..*len]),
        };
    }
    encode_telemetry_into(buf, &recipe.name, seq, mono_time_ns, &refs[..n]);
}

/// Encode one telemetry packet for `recipe` from `snap`, allocating —
/// the convenience form for tests and one-off senders.
pub fn encode_packet(
    recipe: &TelemetryRecipe,
    seq: u64,
    mono_time_ns: u64,
    snap: &StateSnapshot,
) -> Vec<u8> {
    let mut buf = Vec::new();
    encode_packet_into(
        &mut buf,
        recipe,
        seq,
        mono_time_ns,
        snap,
        &mut TelemetryScratch::new(),
    );
    buf
}
