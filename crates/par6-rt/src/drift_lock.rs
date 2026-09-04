//! Freedrive drift lock: the hold the IDLE gravity feedforward gains
//! when `[freedrive] drift_lock` is on.
//!
//! Neither reference runtime has one; the working instance this ports
//! is reBot's velocity-lock demo — a PD hold at the captured pose inside
//! the drive plus a clamped integral in the controller, made transparent
//! (target follows the hand, integral zeroed) the moment the arm moves,
//! re-locked after a quiet spell. Here the PD hold is the drive's own
//! impedance frame at its configured per-joint gains, and this module
//! owns the arming logic and the integral.
//!
//! `Copy`, preallocated, no formatting, no logging: it runs inside the
//! IDLE tick.

use par6_config::RobotConfig;

use crate::MAX_JOINTS;

/// The lock's live state, published on the snapshot.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct DriftLockStatus {
    /// The hold is active: the drives are holding `hold_rad`.
    pub armed: bool,
    /// The captured pose \[rad\] (meaningful while `armed`).
    pub hold_rad: [f64; MAX_JOINTS],
    /// The clamped integral \[Nm\] riding on G(q); zero while not armed.
    /// A standing non-zero value is the bias the gravity model is
    /// missing at this pose.
    pub integral_nm: [f64; MAX_JOINTS],
}

#[derive(Debug, Clone, Copy)]
pub struct DriftLock {
    enabled: bool,
    release_rad_s: f64,
    settle_ticks: u32,
    ki: f64,
    integral_limit: f64,
    dt: f64,
    still_ticks: u32,
    status: DriftLockStatus,
}

impl DriftLock {
    pub fn from_config(robot: &RobotConfig) -> Self {
        let f = &robot.freedrive;
        Self {
            enabled: f.drift_lock,
            release_rad_s: f.release_rad_s,
            settle_ticks: robot.ticks(f.settle_s).max(1),
            ki: f.ki_nm_rad_s,
            integral_limit: f.integral_limit_nm,
            dt: robot.robot.tick_dt_s,
            still_ticks: 0,
            status: DriftLockStatus::default(),
        }
    }

    /// Configured on at all.
    pub fn enabled(&self) -> bool {
        self.enabled
    }

    pub fn status(&self) -> &DriftLockStatus {
        &self.status
    }

    /// Drop the hold and every accumulator: called whenever the arm is
    /// not in freedrive, so a lock never carries over a mode change.
    pub fn reset(&mut self) {
        if self.still_ticks == 0 && !self.status.armed {
            return;
        }
        self.still_ticks = 0;
        self.status = DriftLockStatus::default();
    }

    /// One freedrive tick against the measured pose and speed; whether
    /// the hold is armed afterwards. The raw speed, not the filtered
    /// mirror: a push must release the hold on the tick it is measured,
    /// not after the filter catches up. Motion above the release speed
    /// dissolves the lock and zeroes the integral in this same tick;
    /// stillness for the settle window captures the current pose.
    pub fn tick(&mut self, q: &[f64; MAX_JOINTS], qd: &[f64; MAX_JOINTS]) -> bool {
        let s = &mut self.status;
        if qd.iter().any(|v| v.abs() > self.release_rad_s) {
            self.still_ticks = 0;
            s.armed = false;
            s.integral_nm = [0.0; MAX_JOINTS];
            return false;
        }
        if !s.armed {
            self.still_ticks = self.still_ticks.saturating_add(1);
            if self.still_ticks < self.settle_ticks {
                return false;
            }
            s.armed = true;
            s.hold_rad = *q;
            s.integral_nm = [0.0; MAX_JOINTS];
        }
        for ((acc, hold), q) in s.integral_nm.iter_mut().zip(&s.hold_rad).zip(q) {
            *acc = (*acc + self.ki * (hold - q) * self.dt)
                .clamp(-self.integral_limit, self.integral_limit);
        }
        true
    }
}
