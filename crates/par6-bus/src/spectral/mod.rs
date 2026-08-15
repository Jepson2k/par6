//! Spectral/STEPFOC CAN application protocol.
//!
//! Two pure layers, no I/O:
//!
//! - [`codec`]: 11-bit arbitration id pack/unpack, big-endian payload
//!   primitives (two's-complement i24, i16, i32/u32, f32, MSB-first
//!   bitfields), frame encoding for every host→driver command, and frame
//!   decoding of every driver→host reply into the frozen [`crate::types`]
//!   contract. Wrong-DLC frames are refused whole (typed error, no
//!   partial state); the arbitration-id err bit is surfaced on every
//!   decode, including refused frames.
//! - [`convert`]: unit conversions between wire units (encoder ticks,
//!   ticks/s, mA) and joint SI units (rad, rad/s, Nm) — the vendor
//!   `SourceRoboticsToolbox.Joint` semantics: sector wrap correction with
//!   boot-time [`convert::JointConversion::determine_sector`], home
//!   reference updates, and the Nm↔mA factor with `int()`-style
//!   truncation toward zero at the encode boundary.
//!
//! Golden byte-exact vectors for the codec live in `tests/golden/can/`
//! (checked by `tests/golden_can.rs`).

pub mod codec;
pub mod convert;

pub use codec::{
    decode_frame, encode_clear_error, encode_current_gains, encode_estop, encode_gripper_command,
    encode_heartbeat_setup, encode_idle, encode_joint_command, encode_kt, encode_limits,
    encode_pd_gains, encode_poll, encode_position_gains, encode_reset, encode_rtr_poll,
    encode_save_config, encode_set_can_id, encode_velocity_gains, encode_voltage_limit,
    encode_watchdog, fold_bits_msb_first, pack_can_id, pack_f32, pack_i16, pack_i24, pack_i32,
    pack_u32, poll_command, unfold_bits_msb_first, unpack_can_id, unpack_f32, unpack_i16,
    unpack_i24, unpack_i32, unpack_u32, CanFrame, CommandId, DecodeError, DecodedFrame,
    EncodeError, Payload, CAN_MAX_DATA,
};
pub use convert::{
    radians_to_ticks, ticks_per_radian, ticks_to_radians, torque_to_ma_factor, trunc_to_wire,
    JointConversion,
};
