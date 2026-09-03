//! Per-mode output laws and the commit path — the whole control language.
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
//! | IDLE, drift lock armed | hold | 0 | G(q) + clamped integral | (PD pack: the drive's impedance hold)
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
pub fn law_idle(gravity_hold: bool, g: &[f64; MAX_JOINTS], out: &mut [JointSetpoint; MAX_JOINTS]) {
    if gravity_hold {
        for i in 0..MAX_JOINTS {
            out[i] = JointSetpoint::torque_only(g[i]);
        }
    } else {
        out.fill(JointSetpoint::zero_velocity());
    }
}

/// IDLE with the drift lock armed: the drive's impedance (PD) hold at
/// the captured pose, gravity plus the lock's clamped integral as the
/// feedforward — the same frame shape as a PD-packed jog, with the
/// per-joint gains the config pushed at boot.
pub fn law_freedrive(
    hold: &[f64; MAX_JOINTS],
    g: &[f64; MAX_JOINTS],
    integral: &[f64; MAX_JOINTS],
    out: &mut [JointSetpoint; MAX_JOINTS],
) {
    for i in 0..MAX_JOINTS {
        out[i] = JointSetpoint {
            pos_rad: Some(hold[i]),
            vel_rad_s: Some(0.0),
            torque_nm: Some(g[i] + integral[i]),
            pack: Pack::Pd,
        };
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

/// Per-joint commanded-torque slew state.
///
/// `max_step` is the configured `torque_rate_nm_s` converted to a per-tick
/// budget; a joint whose config declares no rate gets `INFINITY` and is
/// therefore never limited.
///
/// `applied` is the torque actually commanded last tick. Rate-limiting
/// against it is what turns a mode change into a ramp instead of a step —
/// `SafetyStop → Idle` otherwise restores the whole of `G(q)` in one tick,
/// several Nm at the shoulder.
#[derive(Debug, Clone, Copy)]
pub struct TorqueSlew {
    applied: [f64; MAX_JOINTS],
    max_step: [f64; MAX_JOINTS],
}

impl TorqueSlew {
    /// Build from each joint's per-tick budget (`torque_rate_nm_s * dt`);
    /// pass `f64::INFINITY` for a joint that declares no rate.
    pub fn new(max_step: [f64; MAX_JOINTS]) -> Self {
        Self {
            applied: [0.0; MAX_JOINTS],
            max_step,
        }
    }

    /// The torque commanded on the previous tick, per joint.
    pub fn applied(&self) -> &[f64; MAX_JOINTS] {
        &self.applied
    }

    /// Rate-limit `want` toward the previous command and record the result.
    fn step(&mut self, i: usize, want: f64) -> f64 {
        let prev = self.applied[i];
        let out = (want - prev).clamp(-self.max_step[i], self.max_step[i]) + prev;
        self.applied[i] = out;
        out
    }

    /// Adopt `want` unchanged, so a later rate-limited tick ramps from
    /// what the drive was actually told rather than from a stale value.
    fn snap(&mut self, i: usize, want: f64) -> f64 {
        self.applied[i] = want;
        want
    }
}

/// Convert joint-space setpoints to wire commands: motor ticks via each
/// joint's [`JointConversion`], torque→mA via the precomputed factor with
/// truncation toward zero (vendor `int()`), i16 saturation on current.
/// Fills the record-sent mirror alongside.
///
/// `rate_limit` must be FALSE for the protective laws (ACTIVE_ERROR,
/// SAFETY_STOP): dropping drive authority is the one thing that may never
/// be slowed down. Those ticks snap the slew state instead, so the mode
/// they hand back to ramps from the torque the drive really holds.
pub fn commit(
    setpoints: &[JointSetpoint; MAX_JOINTS],
    conv: &[JointConversion; MAX_JOINTS],
    torque_ma_factor: &[f64; MAX_JOINTS],
    slew: &mut TorqueSlew,
    rate_limit: bool,
    cmds: &mut [JointCommand; MAX_JOINTS],
    mirror: &mut CommandMirror,
) {
    for i in 0..MAX_JOINTS {
        let sp = &setpoints[i];
        let pos = sp.pos_rad.map(|r| conv[i].motor_ticks(r));
        let vel = sp
            .vel_rad_s
            .map(|v| trunc_to_wire(conv[i].motor_speed_ticks_s(v)));
        // A joint with no torque channel this tick commands no current, so
        // the slew has to follow it to zero or the next torque tick would
        // ramp from a value the drive never held.
        let torque_nm = match sp.torque_nm {
            Some(t) if rate_limit => Some(slew.step(i, t)),
            Some(t) => Some(slew.snap(i, t)),
            None => {
                slew.snap(i, 0.0);
                None
            }
        };
        let cur = torque_nm.map(|t| {
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
        // The rate-limited value, not the requested one — the mirror is
        // what was actually sent.
        mirror.tau[i] = torque_nm.unwrap_or(0.0);
    }
}
