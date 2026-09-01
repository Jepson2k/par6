//! Telemetry: the recipe registry and the packet codec.
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
//!
//! This module is the pure-data half: field/recipe identity and the
//! byte-level codec. Extracting values from the RT snapshot lives with
//! the server; labeling decoded values with their recipe's fields lives
//! with the client.

use crate::wire::{w_array, w_f64, w_str, w_uint, Reader};
use crate::DecodeError;

/// One selectable field of a telemetry recipe.
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
    /// External joint torque estimate \[Nm\] (measured minus gravity).
    ExternalTorques,
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
    /// Gripper jaw position, 0 = open … 1 = closed (NaN = no reply yet).
    GripperPosition,
    /// Gripper motor current \[mA\] (NaN = no reply yet).
    GripperCurrentMa,
    /// Gripper object-detection code (0 moving, 1 detected opening,
    /// 2 detected closing, 3 reached with no object; NaN = no reply).
    GripperObjectDetection,
    /// Gripper fault bitfield (bit 0 temperature, 1 timeout, 2 e-stop,
    /// 3 live fault bit; 0 = healthy).
    GripperFault,
}

impl TelemetryField {
    /// All fields, in the canonical order used by the `full` recipe.
    pub const ALL: [TelemetryField; 24] = [
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
        TelemetryField::ExternalTorques,
        TelemetryField::MotorTemperaturesC,
        TelemetryField::MotorVoltagesMv,
        TelemetryField::MotorCurrentsMa,
        TelemetryField::LoopPeriodEmaS,
        TelemetryField::LoopP99S,
        TelemetryField::LoopOverruns,
        TelemetryField::GripperPosition,
        TelemetryField::GripperCurrentMa,
        TelemetryField::GripperObjectDetection,
        TelemetryField::GripperFault,
    ];

    /// Stable snake_case key — the name a consumer indexes decoded
    /// frames by.
    pub fn key(self) -> &'static str {
        use TelemetryField as F;
        match self {
            F::Tick => "tick",
            F::MeasuredPositions => "measured_positions",
            F::MeasuredVelocities => "measured_velocities",
            F::MeasuredTorques => "measured_torques",
            F::FilteredPositions => "filtered_positions",
            F::FilteredVelocities => "filtered_velocities",
            F::FilteredTorques => "filtered_torques",
            F::CommandedPositions => "commanded_positions",
            F::CommandedVelocities => "commanded_velocities",
            F::CommandedTorques => "commanded_torques",
            F::TargetPositions => "target_positions",
            F::TargetVelocities => "target_velocities",
            F::GravityTorques => "gravity_torques",
            F::ExternalTorques => "external_torques",
            F::MotorTemperaturesC => "motor_temperatures_c",
            F::MotorVoltagesMv => "motor_voltages_mv",
            F::MotorCurrentsMa => "motor_currents_ma",
            F::LoopPeriodEmaS => "loop_period_ema_s",
            F::LoopP99S => "loop_p99_s",
            F::LoopOverruns => "loop_overruns",
            F::GripperPosition => "gripper_position",
            F::GripperCurrentMa => "gripper_current_ma",
            F::GripperObjectDetection => "gripper_object_detection",
            F::GripperFault => "gripper_fault",
        }
    }
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
                    F::ExternalTorques,
                    F::MotorTemperaturesC,
                    F::MotorVoltagesMv,
                    F::MotorCurrentsMa,
                    F::LoopPeriodEmaS,
                    F::LoopP99S,
                    F::LoopOverruns,
                    F::GripperFault,
                ],
            ),
            recipe("full", TelemetryField::ALL.to_vec()),
        ]
    }
}

/// One value of a telemetry packet.
#[derive(Debug, Clone, PartialEq)]
pub enum TelemetryValue {
    /// Unsigned scalar (tick counters).
    U64(u64),
    /// Float scalar.
    F64(f64),
    /// Float array (per-joint / per-node readings; NaN = unavailable).
    Arr(Vec<f64>),
}

/// A decoded telemetry packet: header plus the recipe's values in field
/// order. Values are self-describing on the wire; mapping them onto
/// [`TelemetryField`]s is the consumer's registry lookup.
#[derive(Debug, Clone, PartialEq)]
pub struct TelemetryFrame {
    /// Recipe name the sender encoded under.
    pub recipe: String,
    /// Monotone packet sequence number.
    pub seq: u64,
    /// Sender's monotonic clock \[ns\].
    pub mono_time_ns: u64,
    /// One value per recipe field, in recipe order.
    pub values: Vec<TelemetryValue>,
}

/// Decode caps: a packet carries at most one value per known field, and
/// arrays are per-joint or per-node — both far below these bounds. A
/// packet claiming more is corrupt, not big.
const MAX_VALUES: usize = 64;
const MAX_ARR: usize = 64;

/// Encode one telemetry packet.
pub fn encode_telemetry(
    recipe: &str,
    seq: u64,
    mono_time_ns: u64,
    values: &[TelemetryValue],
) -> Vec<u8> {
    let mut buf = Vec::with_capacity(64 + values.len() * 56);
    w_array(&mut buf, 3 + values.len());
    w_str(&mut buf, recipe);
    w_uint(&mut buf, seq);
    w_uint(&mut buf, mono_time_ns);
    for v in values {
        match v {
            TelemetryValue::U64(x) => w_uint(&mut buf, *x),
            TelemetryValue::F64(x) => w_f64(&mut buf, *x),
            TelemetryValue::Arr(a) => {
                w_array(&mut buf, a.len());
                for x in a {
                    w_f64(&mut buf, *x);
                }
            }
        }
    }
    buf
}

/// Decode one telemetry packet.
pub fn decode_telemetry(data: &[u8]) -> Result<TelemetryFrame, DecodeError> {
    let mut r = Reader::new(data);
    let n = r.array_len()?;
    if !(3..=3 + MAX_VALUES).contains(&n) {
        return Err(DecodeError::Arity {
            what: "telemetry packet",
            expected: 3,
            got: n,
        });
    }
    let recipe = r.str()?.to_owned();
    let seq = r.uint()?;
    let mono_time_ns = r.uint()?;
    let mut values = Vec::with_capacity(n - 3);
    for _ in 3..n {
        values.push(read_value(&mut r)?);
    }
    r.finish()?;
    Ok(TelemetryFrame {
        recipe,
        seq,
        mono_time_ns,
        values,
    })
}

fn read_value(r: &mut Reader) -> Result<TelemetryValue, DecodeError> {
    let marker = r.peek_marker()?;
    Ok(match marker {
        // fixarray / array16 / array32
        0x90..=0x9f | 0xdc | 0xdd => {
            let n = r.array_len()?;
            if n > MAX_ARR {
                return Err(DecodeError::Arity {
                    what: "telemetry array",
                    expected: MAX_ARR,
                    got: n,
                });
            }
            let mut a = Vec::with_capacity(n);
            for _ in 0..n {
                a.push(r.f64()?);
            }
            TelemetryValue::Arr(a)
        }
        // f32 / f64
        0xca | 0xcb => TelemetryValue::F64(r.f64()?),
        _ => TelemetryValue::U64(r.uint()?),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_packet_roundtrips_every_value_shape() {
        let values = [
            TelemetryValue::U64(123_456),
            TelemetryValue::Arr(vec![0.25, -1.5, f64::NAN, 3.0, -0.125, 2.5]),
            TelemetryValue::F64(0.00401),
            TelemetryValue::U64(u64::MAX),
        ];
        let buf = encode_telemetry("commanded", 42, 1_234_567_890, &values);
        let f = decode_telemetry(&buf).expect("decode");
        assert_eq!(f.recipe, "commanded");
        assert_eq!(f.seq, 42);
        assert_eq!(f.mono_time_ns, 1_234_567_890);
        assert_eq!(f.values.len(), 4);
        assert_eq!(f.values[0], TelemetryValue::U64(123_456));
        let TelemetryValue::Arr(a) = &f.values[1] else {
            panic!("expected array");
        };
        assert_eq!(a.len(), 6);
        assert!(a[2].is_nan(), "NaN must survive the roundtrip");
        assert_eq!(&a[3..], &[3.0, -0.125, 2.5]);
        assert_eq!(f.values[2], TelemetryValue::F64(0.00401));
        assert_eq!(f.values[3], TelemetryValue::U64(u64::MAX));
    }

    #[test]
    fn every_stock_recipe_field_count_matches_its_packet() {
        for recipe in TelemetryRecipe::defaults() {
            let values: Vec<TelemetryValue> = recipe
                .fields
                .iter()
                .map(|_| TelemetryValue::F64(0.0))
                .collect();
            let buf = encode_telemetry(&recipe.name, 0, 0, &values);
            let f = decode_telemetry(&buf).expect("decode");
            assert_eq!(f.values.len(), recipe.fields.len(), "{}", recipe.name);
        }
    }

    #[test]
    fn corrupt_packets_error_instead_of_truncating() {
        assert!(decode_telemetry(&[]).is_err());
        // Arity below the header.
        let buf = encode_telemetry("x", 0, 0, &[]);
        assert!(decode_telemetry(&buf).is_ok());
        // Truncated mid-value.
        let buf = encode_telemetry("full", 7, 8, &[TelemetryValue::Arr(vec![1.0; 6])]);
        assert!(decode_telemetry(&buf[..buf.len() - 3]).is_err());
        // Trailing garbage refused.
        let mut buf = encode_telemetry("full", 7, 8, &[TelemetryValue::U64(1)]);
        buf.push(0);
        assert!(decode_telemetry(&buf).is_err());
    }
}
