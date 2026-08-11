//! Per-mode output laws and the commit path (spec/RT.md "Per-mode output
//! law" — the whole control language).
//!
//! Modes compute joint-space setpoints as [`JointSetpoint`]s (`Option`
//! channels mirror the wire semantics: `None` = channel omitted, NOT
//! zero); [`commit`] owns motor-space conversion via
//! [`JointConversion`], the precomputed torque→mA factor with vendor
//! `int()` truncation, and the record-sent mirrors for the snapshot.
//!
//! | Mode | pos | vel | trq |
//! |---|---|---|---|
//! | BOOTING | — | 0 | 0 |
//! | IDLE (homed ∧ enabled ∧ grav-on) | — | — | G(q) |
//! | IDLE otherwise | — | 0 | 0 |
//! | ACTIVE_ERROR | — | 0 | 0 | (active zero-velocity hold)
//! | SAFETY_STOP | — | — | 0 Nm | (fully limp)
//! | JOG | integrated | ramped | G(q) if grav-on |
//! | EXEC | sample | sample | plan FF + G(q) |
//! | STREAM | limited | limited | G(q) after the limiter |
//! | HOMING / FLASHING | SELF_MANAGED — this module is not involved |

use par6_bus::spectral::{trunc_to_wire, JointConversion};
use par6_bus::{JointCommand, Pack};

use crate::MAX_JOINTS;

/// Joint-space setpoint for one tick, before motor-space conversion.
/// `None` channels are omitted on the wire (`torque_nm: None` packs 0 mA
/// per the codec's substitute-0 rule but records "not commanded").
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointSetpoint {
    /// Position target \[rad\]; `None` = channel omitted.
    pub pos_rad: Option<f64>,
    /// Velocity target \[rad/s\]; `None` = channel omitted.
    pub vel_rad_s: Option<f64>,
    /// Torque command/feedforward \[Nm\]; `None` = unspecified (0 mA).
    pub torque_nm: Option<f64>,
    /// Wire packing for this frame.
    pub pack: Pack,
}

impl JointSetpoint {
    /// Active zero-velocity, zero-current (BOOTING / ACTIVE_ERROR /
    /// un-held IDLE).
    pub const fn zero_velocity() -> Self {
        Self {
            pos_rad: None,
            vel_rad_s: Some(0.0),
            torque_nm: Some(0.0),
            pack: Pack::Pid,
        }
    }

    /// Torque-only frame (gravity hold / limp).
    pub const fn torque_only(nm: f64) -> Self {
        Self {
            pos_rad: None,
            vel_rad_s: None,
            torque_nm: Some(nm),
            pack: Pack::Pid,
        }
    }
}

impl Default for JointSetpoint {
    fn default() -> Self {
        Self::zero_velocity()
    }
}

/// BOOTING: active zero-velocity.
pub fn law_booting(out: &mut [JointSetpoint; MAX_JOINTS]) {
    out.fill(JointSetpoint::zero_velocity());
}

/// IDLE: torque-only gravity hold when `homed ∧ enabled ∧ grav-on` — NO
/// position hold — otherwise active zero-velocity.
pub fn law_idle(
    gravity_hold: bool,
    g: &[f64; MAX_JOINTS],
    out: &mut [JointSetpoint; MAX_JOINTS],
) {
    if gravity_hold {
        for i in 0..MAX_JOINTS {
            out[i] = JointSetpoint::torque_only(g[i]);
        }
    } else {
        out.fill(JointSetpoint::zero_velocity());
    }
}

/// ACTIVE_ERROR: active zero-velocity hold (e-stop lands here — motors
/// stay energized).
pub fn law_active_error(out: &mut [JointSetpoint; MAX_JOINTS]) {
    out.fill(JointSetpoint::zero_velocity());
}

/// SAFETY_STOP: fully limp — torque-only 0 Nm.
pub fn law_safety_stop(out: &mut [JointSetpoint; MAX_JOINTS]) {
    out.fill(JointSetpoint::torque_only(0.0));
}

/// JOG: integrated target + ramped velocity, gravity feedforward when
/// compensation applies, pid or pd packing per config.
pub fn law_jog(
    q_target: &[f64; MAX_JOINTS],
    qd_target: &[f64; MAX_JOINTS],
    gravity_applied: bool,
    g: &[f64; MAX_JOINTS],
    pack: Pack,
    out: &mut [JointSetpoint; MAX_JOINTS],
) {
    for i in 0..MAX_JOINTS {
        out[i] = JointSetpoint {
            pos_rad: Some(q_target[i]),
            vel_rad_s: Some(qd_target[i]),
            torque_nm: Some(if gravity_applied { g[i] } else { 0.0 }),
            pack,
        };
    }
}

/// EXEC: ring sample + plan torque feedforward + G(q).
pub fn law_exec(
    q: &[f64; MAX_JOINTS],
    qd: &[f64; MAX_JOINTS],
    tau_ff: &[f64; MAX_JOINTS],
    gravity_applied: bool,
    g: &[f64; MAX_JOINTS],
    out: &mut [JointSetpoint; MAX_JOINTS],
) {
    for i in 0..MAX_JOINTS {
        let grav = if gravity_applied { g[i] } else { 0.0 };
        out[i] = JointSetpoint {
            pos_rad: Some(q[i]),
            vel_rad_s: Some(qd[i]),
            torque_nm: Some(tau_ff[i] + grav),
            pack: Pack::Pid,
        };
    }
}

/// STREAM: post-limiter setpoint + gravity ADDED AFTER the limiter
/// (never throttle the robot's own weight compensation).
pub fn law_stream(
    q: &[f64; MAX_JOINTS],
    qd: &[f64; MAX_JOINTS],
    gravity_applied: bool,
    g: &[f64; MAX_JOINTS],
    out: &mut [JointSetpoint; MAX_JOINTS],
) {
    for i in 0..MAX_JOINTS {
        out[i] = JointSetpoint {
            pos_rad: Some(q[i]),
            vel_rad_s: Some(qd[i]),
            torque_nm: Some(if gravity_applied { g[i] } else { 0.0 }),
            pack: Pack::Pid,
        };
    }
}

/// Record-sent mirrors written by [`commit`]: what actually went on the
/// bus, back in joint SI units. Channels a mode did not command are NaN
/// (positions/velocities) or the codec's substitute 0 (torque).
#[derive(Debug, Clone, Copy)]
pub struct CommandMirror {
    /// Commanded joint positions \[rad\]; NaN where omitted.
    pub q: [f64; MAX_JOINTS],
    /// Commanded joint velocities \[rad/s\]; NaN where omitted.
    pub qd: [f64; MAX_JOINTS],
    /// Commanded joint torques \[Nm\] (incl. any gravity feedforward).
    pub tau: [f64; MAX_JOINTS],
}

impl Default for CommandMirror {
    fn default() -> Self {
        Self {
            q: [f64::NAN; MAX_JOINTS],
            qd: [f64::NAN; MAX_JOINTS],
            tau: [0.0; MAX_JOINTS],
        }
    }
}

/// Convert joint-space setpoints to wire commands: motor ticks via each
/// joint's [`JointConversion`], torque→mA via the precomputed factor with
/// truncation toward zero (vendor `int()`), i16 saturation on current.
/// Fills the record-sent mirror alongside.
pub fn commit(
    setpoints: &[JointSetpoint; MAX_JOINTS],
    conv: &[JointConversion; MAX_JOINTS],
    torque_ma_factor: &[f64; MAX_JOINTS],
    cmds: &mut [JointCommand; MAX_JOINTS],
    mirror: &mut CommandMirror,
) {
    for i in 0..MAX_JOINTS {
        let sp = &setpoints[i];
        let pos = sp.pos_rad.map(|r| conv[i].motor_ticks(r));
        let vel = sp
            .vel_rad_s
            .map(|v| trunc_to_wire(conv[i].motor_speed_ticks_s(v)));
        let cur = sp.torque_nm.map(|t| {
            trunc_to_wire(t * torque_ma_factor[i]).clamp(i32::from(i16::MIN), i32::from(i16::MAX))
                as i16
        });
        cmds[i] = JointCommand {
            pos,
            vel,
            cur_ma: cur,
            pack: sp.pack,
        };
        mirror.q[i] = sp.pos_rad.unwrap_or(f64::NAN);
        mirror.qd[i] = sp.vel_rad_s.unwrap_or(f64::NAN);
        mirror.tau[i] = sp.torque_nm.unwrap_or(0.0);
    }
}
