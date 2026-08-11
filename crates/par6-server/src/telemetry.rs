//! Telemetry: recipe-selected field streams from the RT snapshot.
//!
//! Binary msgpack, one packet per tick of the telemetry interval:
//!
//! ```text
//! [recipe_name str, seq u64, mono_time_ns u64, ...one value per field]
//! ```
//!
//! Values appear in the recipe's field order; array-valued fields encode
//! as f64 arrays (unavailable per-node readings encode as NaN), scalars
//! as f64 / u64. Recipes are named in config; `set_recipe` REFUSES
//! unknown names with `COMM_UNKNOWN_RECIPE` — a silent fallback looks
//! like a dead robot.

use par6_rt::{StateSnapshot, MAX_JOINTS, NUM_NODES};
use serde::ser::{SerializeSeq, Serializer};
use serde::Serialize;

/// One selectable field of a telemetry recipe, sourced from
/// [`StateSnapshot`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TelemetryField {
    /// RT tick counter (u64).
    Tick,
    /// Measured joint positions \[rad\].
    MeasuredPositions,
    /// Measured joint velocities \[rad/s\].
    MeasuredVelocities,
    /// Measured joint torques \[Nm\].
    MeasuredTorques,
    /// Filtered measured positions \[rad\].
    FilteredPositions,
    /// Filtered measured velocities \[rad/s\].
    FilteredVelocities,
    /// Filtered measured torques \[Nm\].
    FilteredTorques,
    /// Commanded joint positions \[rad\] (post-limiter).
    CommandedPositions,
    /// Commanded joint velocities \[rad/s\].
    CommandedVelocities,
    /// Commanded joint torques \[Nm\].
    CommandedTorques,
    /// Target joint positions \[rad\] (raw request, pre-limiter).
    TargetPositions,
    /// Target joint velocities \[rad/s\].
    TargetVelocities,
    /// Gravity torque G(q) \[Nm\].
    GravityTorques,
    /// Per-node driver temperatures \[°C\] (NaN = not reported).
    MotorTemperaturesC,
    /// Per-node bus voltages \[mV\] (NaN = not reported).
    MotorVoltagesMv,
    /// Per-node motor currents \[mA\] (NaN = not reported).
    MotorCurrentsMa,
    /// EMA of the RT loop period \[s\].
    LoopPeriodEmaS,
    /// p99 of the RT loop period \[s\].
    LoopP99S,
    /// RT deadline overruns since start/reset (u64).
    LoopOverruns,
}

impl TelemetryField {
    /// All fields, in the canonical order used by the `full` recipe.
    pub const ALL: [TelemetryField; 19] = [
        TelemetryField::Tick,
        TelemetryField::MeasuredPositions,
        TelemetryField::MeasuredVelocities,
        TelemetryField::MeasuredTorques,
        TelemetryField::FilteredPositions,
        TelemetryField::FilteredVelocities,
        TelemetryField::FilteredTorques,
        TelemetryField::CommandedPositions,
        TelemetryField::CommandedVelocities,
        TelemetryField::CommandedTorques,
        TelemetryField::TargetPositions,
        TelemetryField::TargetVelocities,
        TelemetryField::GravityTorques,
        TelemetryField::MotorTemperaturesC,
        TelemetryField::MotorVoltagesMv,
        TelemetryField::MotorCurrentsMa,
        TelemetryField::LoopPeriodEmaS,
        TelemetryField::LoopP99S,
        TelemetryField::LoopOverruns,
    ];
}

/// A named field selection.
#[derive(Debug, Clone, PartialEq)]
pub struct TelemetryRecipe {
    /// Recipe name as accepted by `set_recipe`.
    pub name: String,
    /// Fields in packet order.
    pub fields: Vec<TelemetryField>,
}

impl TelemetryRecipe {
    /// The stock registry: `minimal` / `standard` / `commanded` /
    /// `diagnostics` / `full`.
    pub fn defaults() -> Vec<TelemetryRecipe> {
        use TelemetryField as F;
        let recipe = |name: &str, fields: Vec<TelemetryField>| TelemetryRecipe {
            name: name.to_owned(),
            fields,
        };
        vec![
            recipe("minimal", vec![F::Tick, F::MeasuredPositions]),
            recipe(
                "standard",
                vec![
                    F::Tick,
                    F::MeasuredPositions,
                    F::MeasuredVelocities,
                    F::MeasuredTorques,
                ],
            ),
            recipe(
                "commanded",
                vec![
                    F::Tick,
                    F::MeasuredPositions,
                    F::CommandedPositions,
                    F::CommandedVelocities,
                    F::CommandedTorques,
                    F::TargetPositions,
                    F::TargetVelocities,
                ],
            ),
            recipe(
                "diagnostics",
                vec![
                    F::Tick,
                    F::MeasuredPositions,
                    F::MeasuredTorques,
                    F::MotorTemperaturesC,
                    F::MotorVoltagesMv,
                    F::MotorCurrentsMa,
                    F::LoopPeriodEmaS,
                    F::LoopP99S,
                    F::LoopOverruns,
                ],
            ),
            recipe("full", TelemetryField::ALL.to_vec()),
        ]
    }
}

fn joints(a: &[f64; MAX_JOINTS]) -> Vec<f64> {
    a.to_vec()
}

fn nodes_f64(f: impl Fn(usize) -> Option<f64>) -> Vec<f64> {
    (0..NUM_NODES).map(|i| f(i).unwrap_or(f64::NAN)).collect()
}

enum Value {
    U64(u64),
    F64(f64),
    Arr(Vec<f64>),
}

impl Serialize for Value {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        match self {
            Value::U64(v) => s.serialize_u64(*v),
            Value::F64(v) => s.serialize_f64(*v),
            Value::Arr(v) => v.serialize(s),
        }
    }
}

fn extract(field: TelemetryField, snap: &StateSnapshot) -> Value {
    use TelemetryField as F;
    match field {
        F::Tick => Value::U64(snap.tick),
        F::MeasuredPositions => Value::Arr(joints(&snap.q)),
        F::MeasuredVelocities => Value::Arr(joints(&snap.qd)),
        F::MeasuredTorques => Value::Arr(joints(&snap.tau)),
        F::FilteredPositions => Value::Arr(joints(&snap.q_filtered)),
        F::FilteredVelocities => Value::Arr(joints(&snap.qd_filtered)),
        F::FilteredTorques => Value::Arr(joints(&snap.tau_filtered)),
        F::CommandedPositions => Value::Arr(joints(&snap.q_commanded)),
        F::CommandedVelocities => Value::Arr(joints(&snap.qd_commanded)),
        F::CommandedTorques => Value::Arr(joints(&snap.tau_commanded)),
        F::TargetPositions => Value::Arr(joints(&snap.q_target)),
        F::TargetVelocities => Value::Arr(joints(&snap.qd_target)),
        F::GravityTorques => Value::Arr(joints(&snap.gravity_torque_nm)),
        F::MotorTemperaturesC => {
            Value::Arr(nodes_f64(|i| snap.nodes[i].temperature_c.map(f64::from)))
        }
        F::MotorVoltagesMv => Value::Arr(nodes_f64(|i| snap.nodes[i].voltage_mv.map(f64::from))),
        F::MotorCurrentsMa => Value::Arr(nodes_f64(|i| snap.nodes[i].current_ma.map(f64::from))),
        F::LoopPeriodEmaS => Value::F64(snap.loop_stats.period_ema_s),
        F::LoopP99S => Value::F64(snap.loop_stats.p99_s),
        F::LoopOverruns => Value::U64(u64::from(snap.loop_stats.overruns)),
    }
}

struct Packet<'a> {
    recipe: &'a TelemetryRecipe,
    seq: u64,
    mono_time_ns: u64,
    snap: &'a StateSnapshot,
}

impl Serialize for Packet<'_> {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        let mut seq = s.serialize_seq(Some(3 + self.recipe.fields.len()))?;
        seq.serialize_element(&self.recipe.name)?;
        seq.serialize_element(&self.seq)?;
        seq.serialize_element(&self.mono_time_ns)?;
        for f in &self.recipe.fields {
            seq.serialize_element(&extract(*f, self.snap))?;
        }
        seq.end()
    }
}

/// Encode one telemetry packet for `recipe` from `snap`.
pub fn encode_packet(
    recipe: &TelemetryRecipe,
    seq: u64,
    mono_time_ns: u64,
    snap: &StateSnapshot,
) -> Vec<u8> {
    rmp_serde::to_vec(&Packet {
        recipe,
        seq,
        mono_time_ns,
        snap,
    })
    .expect("telemetry packet serialization is infallible")
}
