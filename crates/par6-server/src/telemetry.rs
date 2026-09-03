//! Telemetry: recipe-selected field streams from the RT snapshot.
//!
//! The field/recipe identity and the packet codec live in
//! [`par6_proto::telemetry`]; this module is the server half — extracting
//! each field's value from the [`StateSnapshot`] and encoding the packet.

use par6_rt::{StateSnapshot, StreamSubstate, MAX_JOINTS, NUM_NODES};

use par6_proto::telemetry::{encode_telemetry, TelemetryValue};
pub use par6_proto::telemetry::{TelemetryField, TelemetryRecipe};

fn joints(a: &[f64; MAX_JOINTS]) -> Vec<f64> {
    a.to_vec()
}

fn nodes_f64(f: impl Fn(usize) -> Option<f64>) -> Vec<f64> {
    (0..NUM_NODES).map(|i| f(i).unwrap_or(f64::NAN)).collect()
}

fn extract(field: TelemetryField, snap: &StateSnapshot) -> TelemetryValue {
    use TelemetryField as F;
    use TelemetryValue as V;
    match field {
        F::Tick => V::U64(snap.tick),
        F::MeasuredPositions => V::Arr(joints(&snap.q)),
        F::MeasuredVelocities => V::Arr(joints(&snap.qd)),
        F::MeasuredTorques => V::Arr(joints(&snap.tau)),
        F::FilteredPositions => V::Arr(joints(&snap.q_filtered)),
        F::FilteredVelocities => V::Arr(joints(&snap.qd_filtered)),
        F::FilteredTorques => V::Arr(joints(&snap.tau_filtered)),
        F::CommandedPositions => V::Arr(joints(&snap.q_commanded)),
        F::CommandedVelocities => V::Arr(joints(&snap.qd_commanded)),
        F::CommandedTorques => V::Arr(joints(&snap.tau_commanded)),
        F::TargetPositions => V::Arr(joints(&snap.q_target)),
        F::TargetVelocities => V::Arr(joints(&snap.qd_target)),
        F::GravityTorques => V::Arr(joints(&snap.gravity_torque_nm)),
        F::ExternalTorques => V::Arr(joints(&snap.tau_ext)),
        F::MotorTemperaturesC => V::Arr(nodes_f64(|i| snap.nodes[i].temperature_c.map(f64::from))),
        F::MotorVoltagesMv => V::Arr(nodes_f64(|i| snap.nodes[i].voltage_mv.map(f64::from))),
        F::MotorCurrentsMa => V::Arr(nodes_f64(|i| snap.nodes[i].current_ma.map(f64::from))),
        F::LoopPeriodEmaS => V::F64(snap.loop_stats.period_ema_s),
        F::LoopP99S => V::F64(snap.loop_stats.p99_s),
        F::LoopOverruns => V::U64(u64::from(snap.loop_stats.overruns)),
        F::GripperPosition => V::F64(
            snap.gripper
                .reply
                .map_or(f64::NAN, |r| f64::from(r.position) / 255.0),
        ),
        F::GripperCurrentMa => V::F64(
            snap.gripper
                .reply
                .map_or(f64::NAN, |r| f64::from(r.current_ma)),
        ),
        F::GripperObjectDetection => V::F64(
            snap.gripper
                .reply
                .map_or(f64::NAN, |r| f64::from(r.object_detection as u8)),
        ),
        F::GripperFault => V::U64(u64::from(
            u32::try_from(crate::faults::gripper_fault_code(snap)).unwrap_or(0),
        )),
        F::StreamSubstate => V::U64(match snap.stream.substate {
            StreamSubstate::Unpaired => 0,
            StreamSubstate::Connected => 1,
            StreamSubstate::ControlActive => 2,
        }),
        F::StreamSuccessRate => V::F64(f64::from(snap.stream.success_rate)),
        F::StreamDiscardPct => V::F64(f64::from(snap.stream.discard_pct)),
    }
}

/// Encode one telemetry packet for `recipe` from `snap`.
pub fn encode_packet(
    recipe: &TelemetryRecipe,
    seq: u64,
    mono_time_ns: u64,
    snap: &StateSnapshot,
) -> Vec<u8> {
    let values: Vec<TelemetryValue> = recipe.fields.iter().map(|f| extract(*f, snap)).collect();
    encode_telemetry(&recipe.name, seq, mono_time_ns, &values)
}
