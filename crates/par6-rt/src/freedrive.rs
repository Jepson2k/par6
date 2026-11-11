//! The floating arm's law: what the drives are asked for while a hand, and
//! nothing else, moves the arm.
//!
//! A homed, enabled arm in IDLE with gravity compensation on carries its
//! own weight and has no position term, so it goes where it is pushed.
//! Left at that, it also goes where it is thrown, through its soft limits,
//! and is as stiff to move as its gearboxes are. Per joint, this adds:
//!
//! - **push assist** — the torque a hand is putting on the joint, as the
//!   external-torque estimate reads it, added to again by the drive. The
//!   hand feels a lighter arm; with nothing pushing there is nothing to
//!   add to, so the joint stops as it always did.
//! - **soft limits** — a joint that reaches one moving outward is held
//!   there by its drive, alone, until it is pushed back in.
//! - **the catch** — a joint faster than `catch_speed_rad_s` for
//!   `catch_window_s` is not being guided. The caller stops the arm.
//!
//! Everything that stops a joint here is the drive's own position loop,
//! and nothing here feeds a joint's velocity back to it. A torque computed
//! from velocity on this side of the bus — braking, or paying a joint's
//! friction for it — is a loop closed at the tick rate around a rotor that
//! weighs almost nothing, and at any gain worth having it shakes the
//! drivetrain.
//!
//! Nothing here plans or allocates; it runs on the RT tick.

use par6_bus::Pack;
use par6_config::RobotConfig;

use crate::dispatch::JointSetpoint;
use crate::MAX_JOINTS;

/// A joint its drive is holding at a soft limit.
#[derive(Debug, Clone, Copy)]
struct Held {
    at: f64,
    /// Sign of the direction back into the window.
    inward: f64,
}

/// The floating law and its per-joint state.
#[derive(Debug)]
pub struct Freedrive {
    assist: f64,
    /// Low-pass coefficient of the push the assist follows.
    assist_alpha: f64,
    /// The hand's torque on each joint, as the assist follows it \[Nm\].
    pushed: [f64; MAX_JOINTS],
    release_nm: [f64; MAX_JOINTS],
    catch_speed: [f64; MAX_JOINTS],
    catch_ticks: u32,
    /// Outward speed that counts as moving, not as encoder noise.
    moving_rad_s: f64,
    soft_min: [f64; MAX_JOINTS],
    soft_max: [f64; MAX_JOINTS],
    held: [Option<Held>; MAX_JOINTS],
    over: [u32; MAX_JOINTS],
}

impl Freedrive {
    /// From `[freedrive]`, the joints' soft windows and the friction
    /// model's stick speed.
    pub fn new(robot: &RobotConfig) -> Self {
        let f = &robot.freedrive;
        let dt = robot.robot.tick_dt_s;
        let per_joint = |values: &[f64], fallback: f64| -> [f64; MAX_JOINTS] {
            std::array::from_fn(|i| values.get(i).copied().unwrap_or(fallback))
        };
        let mut soft_min = [f64::NEG_INFINITY; MAX_JOINTS];
        let mut soft_max = [f64::INFINITY; MAX_JOINTS];
        for (i, joint) in robot.joints.iter().take(MAX_JOINTS).enumerate() {
            soft_min[i] = joint.limits.soft_min_rad;
            soft_max[i] = joint.limits.soft_max_rad;
        }
        Self {
            assist: f.push_assist,
            assist_alpha: {
                let k = std::f64::consts::TAU * f.push_assist_hz * dt;
                k / (1.0 + k)
            },
            pushed: [0.0; MAX_JOINTS],
            release_nm: per_joint(&f.limit_release_nm, 0.0),
            catch_speed: per_joint(&f.catch_speed_rad_s, f64::INFINITY),
            catch_ticks: ((f.catch_window_s / dt).round() as u32).max(1),
            moving_rad_s: robot.dynamics.friction_stick_rad_s,
            soft_min,
            soft_max,
            held: [None; MAX_JOINTS],
            over: [0; MAX_JOINTS],
        }
    }

    /// Let every held joint go and start the catch windows over: the arm
    /// has stopped floating, or is about to start.
    pub fn reset(&mut self) {
        self.held = [None; MAX_JOINTS];
        self.over = [0; MAX_JOINTS];
        self.pushed = [0.0; MAX_JOINTS];
    }

    /// Whether joint `i` is being held at a soft limit.
    pub fn holds(&self, i: usize) -> bool {
        self.held[i].is_some()
    }

    /// One tick of the floating law.
    ///
    /// `signal` is the contact signal: the external-torque estimate less
    /// what stiction and a declared payload account for, which on a
    /// floating arm is the hand — read with the sign of the motor effort
    /// that would resist it, so the hand's own torque is its negative.
    pub fn law(
        &mut self,
        q: &[f64; MAX_JOINTS],
        qd: &[f64; MAX_JOINTS],
        g: &[f64; MAX_JOINTS],
        signal: &[f64; MAX_JOINTS],
        out: &mut [JointSetpoint; MAX_JOINTS],
    ) {
        for i in 0..MAX_JOINTS {
            let (lo, hi) = (self.soft_min[i], self.soft_max[i]);
            match self.held[i] {
                Some(held) if -signal[i] * held.inward > self.release_nm[i] => {
                    self.held[i] = None;
                }
                Some(_) => {}
                None if q[i] >= hi && qd[i] > self.moving_rad_s => {
                    self.held[i] = Some(Held {
                        at: q[i],
                        inward: -1.0,
                    });
                }
                None if q[i] <= lo && qd[i] < -self.moving_rad_s => {
                    self.held[i] = Some(Held {
                        at: q[i],
                        inward: 1.0,
                    });
                }
                None => {}
            }
            out[i] = match self.held[i] {
                Some(held) => {
                    self.pushed[i] = 0.0;
                    JointSetpoint {
                        pos_rad: Some(held.at),
                        vel_rad_s: Some(0.0),
                        torque_nm: Some(g[i]),
                        pack: Pack::Pid,
                    }
                }
                None => {
                    self.pushed[i] += self.assist_alpha * (-signal[i] - self.pushed[i]);
                    JointSetpoint::torque_only(g[i] + self.assist * self.pushed[i])
                }
            };
        }
    }

    /// The joint that has been faster than its catch speed for the whole
    /// window, if any: an arm that is falling or was thrown, not guided.
    pub fn overspeed(&mut self, qd: &[f64; MAX_JOINTS]) -> Option<u8> {
        let mut caught = None;
        for (i, (over, speed)) in self.over.iter_mut().zip(qd).enumerate() {
            if speed.abs() > self.catch_speed[i] {
                *over = over.saturating_add(1);
            } else {
                *over = 0;
            }
            if caught.is_none() && *over >= self.catch_ticks {
                caught = Some(i as u8);
            }
        }
        caught
    }
}
