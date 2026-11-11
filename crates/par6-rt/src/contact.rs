//! Contact detection: reading something in the arm's way out of the
//! external-torque estimate.
//!
//! The estimate is not judged raw. Two things put torque into it with
//! nothing touching the arm, and each joint's estimate first loses what
//! they could account for; what is left is the contact signal.
//!
//! - **Stiction.** A joint that is at rest, or about to be, sits
//!   somewhere inside its Coulomb band with its drive still leaning on
//!   it: the whole Coulomb torque at rest, fading to nothing once the
//!   joint slides.
//! - **The model's own inertial terms.** The model is rigid and the
//!   drivetrains are not: under a hard change of acceleration a motor
//!   runs ahead of its link, and for that moment the inertial torque the
//!   model charged is partly not there. `inertia_slack` of the effort the
//!   model is charging, held for `inertia_hold_s` while the drivetrain
//!   rings down, is allowed for.
//!
//! Two channels watch the signal:
//!
//! - **sustained**: the signal itself beyond `sustained_nm` for
//!   `sustained_window_s` — something the arm is pressing into.
//! - **impact**: the signal with its own slow average taken out, beyond
//!   `impact_nm` for `impact_window_s` — a bump, caught even when a
//!   payload the model does not know or a gravity error has shifted the
//!   whole signal, which the sustained threshold has to sit above.
//!
//! What a crossing means is the caller's: a moving arm stops for it, a
//! resting one only reports it.

use par6_config::{ContactReaction, RobotConfig};

use crate::MAX_JOINTS;

/// Which channel read the contact.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContactChannel {
    /// The signal stayed beyond its threshold: pressing.
    Sustained,
    /// The signal jumped against its own average: a bump.
    Impact,
}

/// A threshold crossing that lasted its window.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ContactTrip {
    /// The joint that read it.
    pub joint: u8,
    /// The channel that read it.
    pub channel: ContactChannel,
}

/// What the detector is watching for this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContactWatch {
    /// The arm is being driven somewhere: a crossing is a contact to
    /// stop for, and the peaks the thresholds are tuned against
    /// accumulate.
    Moving,
    /// The arm is settled on its target: a crossing is somebody leaning
    /// on it, reported and nothing more.
    Resting,
    /// Nothing may be read into the estimate — a contact is already
    /// being handled, the arm floats, or no model describes it.
    Suspended,
}

/// The detector's readout, published with the state snapshot.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContactStatus {
    /// What the arm does about a contact.
    pub reaction: ContactReaction,
    /// The contact signal per joint \[Nm\]: the external-torque estimate
    /// less what stiction accounts for.
    pub signal: [f64; MAX_JOINTS],
    /// The signal against its own slow average \[Nm\].
    pub impact: [f64; MAX_JOINTS],
    /// Largest `|signal|` seen while the arm was being moved \[Nm\].
    pub peak_signal: [f64; MAX_JOINTS],
    /// Largest `|impact|` seen while the arm was being moved \[Nm\].
    pub peak_impact: [f64; MAX_JOINTS],
    /// Sustained thresholds in force \[Nm\].
    pub sustained_nm: [f64; MAX_JOINTS],
    /// Impact thresholds in force \[Nm\].
    pub impact_nm: [f64; MAX_JOINTS],
    /// Contacts the arm stopped for since boot.
    pub trips: u32,
    /// The most recent of them.
    pub last_trip: Option<ContactTrip>,
}

impl Default for ContactStatus {
    fn default() -> Self {
        Self {
            reaction: ContactReaction::default(),
            signal: [0.0; MAX_JOINTS],
            impact: [0.0; MAX_JOINTS],
            peak_signal: [0.0; MAX_JOINTS],
            peak_impact: [0.0; MAX_JOINTS],
            sustained_nm: [0.0; MAX_JOINTS],
            impact_nm: [0.0; MAX_JOINTS],
            trips: 0,
            last_trip: None,
        }
    }
}

/// Per-tick contact detection on the external-torque estimate.
#[derive(Debug)]
pub struct ContactDetector {
    coulomb_nm: [f64; MAX_JOINTS],
    stiction_speed_rad_s: f64,
    inertia_slack: [f64; MAX_JOINTS],
    /// Per-tick decay of the held inertial effort.
    effort_decay: f64,
    effort_held: [f64; MAX_JOINTS],
    sustained_ticks: u32,
    impact_ticks: u32,
    warning_hold_ticks: u32,
    /// Per-tick coefficient of the slow average the impact channel
    /// subtracts.
    average_alpha: f64,
    average: [f64; MAX_JOINTS],
    seeded: bool,
    over_sustained: [i32; MAX_JOINTS],
    over_impact: [i32; MAX_JOINTS],
    warning_left: [u32; MAX_JOINTS],
    status: ContactStatus,
}

/// Fewest consecutive ticks a crossing has to last, whatever the tick
/// rate makes of the configured windows. One sample cannot say that
/// something is pushing, as against something having rung once.
const MIN_WINDOW_TICKS: u32 = 2;

impl ContactDetector {
    /// From `[contact]` and the friction `[dynamics]` declares, at the
    /// robot's tick period.
    pub fn new(robot: &RobotConfig) -> Self {
        let c = &robot.contact;
        let dt = robot.robot.tick_dt_s;
        Self {
            coulomb_nm: std::array::from_fn(|i| robot.dynamics.friction_coulomb_nm[i]),
            stiction_speed_rad_s: c.stiction_speed_rad_s,
            inertia_slack: std::array::from_fn(|i| c.inertia_slack[i]),
            effort_decay: (1.0 - dt / c.inertia_hold_s).max(0.0),
            effort_held: [0.0; MAX_JOINTS],
            sustained_ticks: robot.ticks(c.sustained_window_s).max(MIN_WINDOW_TICKS),
            impact_ticks: robot.ticks(c.impact_window_s).max(MIN_WINDOW_TICKS),
            warning_hold_ticks: robot.ticks(c.warning_hold_s).max(1),
            average_alpha: dt / (c.impact_highpass_s + dt),
            average: [0.0; MAX_JOINTS],
            seeded: false,
            over_sustained: [0; MAX_JOINTS],
            over_impact: [0; MAX_JOINTS],
            warning_left: [0; MAX_JOINTS],
            status: ContactStatus {
                reaction: c.reaction,
                sustained_nm: std::array::from_fn(|i| c.sustained_nm[i]),
                impact_nm: std::array::from_fn(|i| c.impact_nm[i]),
                ..ContactStatus::default()
            },
        }
    }

    /// The readout.
    pub fn status(&self) -> &ContactStatus {
        &self.status
    }

    /// Start over from the next signal: no average, no windows in
    /// progress. For when the estimate itself has been started over.
    pub fn reseed(&mut self) {
        self.seeded = false;
        self.over_sustained = [0; MAX_JOINTS];
        self.over_impact = [0; MAX_JOINTS];
        self.effort_held = [0.0; MAX_JOINTS];
    }

    /// Forget the peaks: the next run is measured from nothing.
    pub fn reset_peaks(&mut self) {
        self.status.peak_signal = [0.0; MAX_JOINTS];
        self.status.peak_impact = [0.0; MAX_JOINTS];
    }

    /// Whether joint `i` has a standing contact warning: it crossed a
    /// threshold on an arm that does not stop for it, within the hold
    /// that keeps a signal hovering at its threshold from flickering.
    pub fn warning(&self, i: usize) -> bool {
        self.warning_left[i] > 0
    }

    /// One tick. Returns the contact a moving arm has to stop for, the
    /// tick its window fills; the same crossing on an arm that only
    /// reports raises [`warning`](Self::warning) instead.
    pub fn update(
        &mut self,
        tau_ext: &[f64; MAX_JOINTS],
        qd: &[f64; MAX_JOINTS],
        effort: &[f64; MAX_JOINTS],
        watch: ContactWatch,
    ) -> Option<ContactTrip> {
        let mut trip = None;
        for i in 0..MAX_JOINTS {
            self.effort_held[i] = effort[i].max(self.effort_held[i] * self.effort_decay);
            let hidden = self.coulomb_nm[i]
                * (1.0 - (qd[i].abs() / self.stiction_speed_rad_s).tanh())
                + self.inertia_slack[i] * self.effort_held[i];
            let signal = tau_ext[i] - tau_ext[i].clamp(-hidden, hidden);
            // An average that kept its own counsel while nothing was
            // being read would meet the first armed tick with a level
            // from another time — an unreferenced arm's, say — and the
            // difference would read as a bump.
            if !self.seeded || watch == ContactWatch::Suspended {
                self.average[i] = signal;
            }
            self.average[i] += self.average_alpha * (signal - self.average[i]);
            let impact = signal - self.average[i];
            self.status.signal[i] = signal;
            self.status.impact[i] = impact;
            self.warning_left[i] = self.warning_left[i].saturating_sub(1);

            if watch == ContactWatch::Suspended || self.status.reaction == ContactReaction::Off {
                self.over_sustained[i] = 0;
                self.over_impact[i] = 0;
                continue;
            }
            if watch == ContactWatch::Moving {
                self.status.peak_signal[i] = self.status.peak_signal[i].max(signal.abs());
                self.status.peak_impact[i] = self.status.peak_impact[i].max(impact.abs());
            }
            let sustained = step(
                &mut self.over_sustained[i],
                signal,
                self.status.sustained_nm[i],
                self.sustained_ticks,
            );
            let bumped = step(
                &mut self.over_impact[i],
                impact,
                self.status.impact_nm[i],
                self.impact_ticks,
            );
            let channel = if sustained {
                ContactChannel::Sustained
            } else if bumped {
                ContactChannel::Impact
            } else {
                continue;
            };
            if watch == ContactWatch::Moving && self.status.reaction == ContactReaction::Stop {
                trip.get_or_insert(ContactTrip {
                    joint: i as u8,
                    channel,
                });
            } else {
                self.warning_left[i] = self.warning_hold_ticks;
            }
        }
        self.seeded = true;
        if let Some(t) = trip {
            self.status.trips = self.status.trips.saturating_add(1);
            self.status.last_trip = Some(t);
            self.over_sustained = [0; MAX_JOINTS];
            self.over_impact = [0; MAX_JOINTS];
        }
        trip
    }
}

/// Advance one over-threshold counter; `true` while it has filled its
/// window. The count is signed with the side the value is over on, and a
/// change of side starts it again: a contact pushes one way, and a
/// drivetrain ringing after a hard start crosses both thresholds in
/// turn without anything being there.
fn step(count: &mut i32, value: f64, threshold: f64, window: u32) -> bool {
    let side = if value > threshold {
        1
    } else if value < -threshold {
        -1
    } else {
        0
    };
    *count = if side != 0 && count.signum() == side {
        count.saturating_add(side)
    } else {
        side
    };
    count.unsigned_abs() >= window
}
