//! A joint's loop oscillating, caught from what its drive reports.
//!
//! Two ways, both seen on the arm with J1 out. Its velocity loop rang at
//! about 100 Hz under positive torque (2026-11-07): the reported speed
//! flipped by 100-450 deg/s between 4 ms replies, faster than any move
//! changes it, and a ringing loop does not brake. Caught on its third fast
//! swing, about 12 ms in, the ring is ended where it starts: the joint's
//! velocity loop drops to a share of its gains, which restores the margin it
//! ran out of. And its position loop, too stiff for the velocity loop under
//! it, held some landings in a 3.9 Hz limit cycle (2026-11-08), the current
//! swinging from one end of the drive's range to the other while the
//! encoder barely moved: the arm whipped on its drivetrain. The hold
//! ACTIVE_ERROR lands in closes no position loop, so the latch alone ends it.

use crate::MAX_JOINTS;

/// Swings of a signal between its turning points that make an
/// oscillation: each larger than `swing`, each completed within `within`
/// replies, `swings` of them in a row. A glitched reply makes two, out and
/// back.
#[derive(Debug, Clone, Copy)]
pub struct Rule {
    pub swing: f64,
    pub within: u32,
    pub swings: u32,
}

/// A velocity loop's ring, on reported speed \[rad/s\]: 90 deg/s, which no
/// commanded motion makes in three replies (130 rad/s², four times J1's
/// ceiling acceleration). Over 816 joint traces from the arm (2026-11-07)
/// the largest jump between replies outside a ring was 71 deg/s; J1 swung
/// in one or two replies, the simulator's slower rings at four times the
/// gains in two or three.
pub const LOOP_RING: Rule = Rule {
    swing: 90.0 * std::f64::consts::PI / 180.0,
    within: 3,
    swings: 3,
};

/// A held joint's limit cycle, on its current as a share of the drive's
/// torque ceiling: swings across the whole ceiling, however slow. J1's
/// cycles swung 1.5-1.9 of it every 0.13 s; the most a hold that settled
/// ever swung was 1.2, once, then 0.8 (2026-11-08).
pub const HOLD_CYCLE: Rule = Rule {
    swing: 1.0,
    within: u32::MAX,
    swings: 3,
};

/// The share of its velocity-loop gains a ringing joint drops to. J1 rang on
/// arm-out moves at full gains and ran the same moves clean at 0.64 of them
/// (2026-11-07).
pub const CALM_GAIN_SHARE: f64 = 0.5;

#[derive(Debug, Default, Clone, Copy)]
struct Joint {
    last: Option<f64>,
    replies: u32,
    /// Where the swing under way started, and the reply it started on.
    anchor: (f64, u32),
    /// How far it has got, and when.
    peak: (f64, u32),
    /// Its direction, once there is one.
    rising: Option<bool>,
    swings: u32,
}

/// Per joint, the swings of one signal under one rule.
#[derive(Debug)]
pub struct RingGuard {
    rule: Rule,
    joints: [Joint; MAX_JOINTS],
}

impl RingGuard {
    pub fn new(rule: Rule) -> Self {
        Self {
            rule,
            joints: [Joint::default(); MAX_JOINTS],
        }
    }

    /// Forget joint `j`'s swings: what it does next starts afresh.
    pub fn reset(&mut self, j: usize) {
        self.joints[j] = Joint::default();
    }

    /// Take this tick's values; which joints oscillate. A value unchanged
    /// since the last tick is no new reply and counts for nothing.
    pub fn step(&mut self, values: &[f64; MAX_JOINTS]) -> [bool; MAX_JOINTS] {
        let rule = self.rule;
        let mut oscillating = [false; MAX_JOINTS];
        for (j, joint) in self.joints.iter_mut().enumerate() {
            let value = values[j];
            if joint.last.replace(value) == Some(value) {
                continue;
            }
            joint.replies += 1;
            let now = (value, joint.replies);
            let Some(rising) = joint.rising else {
                if joint.replies == 1 {
                    joint.anchor = now;
                } else {
                    joint.rising = Some(value > joint.anchor.0);
                }
                joint.peak = now;
                continue;
            };
            if (value > joint.peak.0) == rising && value != joint.peak.0 {
                joint.peak = now;
            } else if (joint.peak.0 - value).abs() > rule.swing / 2.0 {
                // The swing turned: judge it whole.
                let swing = (joint.peak.0 - joint.anchor.0).abs();
                let fast = joint.peak.1 - joint.anchor.1 <= rule.within;
                joint.swings = if swing > rule.swing && fast {
                    joint.swings + 1
                } else {
                    0
                };
                joint.anchor = joint.peak;
                joint.peak = now;
                joint.rising = Some(!rising);
                if joint.swings >= rule.swings {
                    oscillating[j] = true;
                    // Reported once: a joint held still after it never
                    // turns a swing that would clear the count.
                    *joint = Joint {
                        last: Some(value),
                        ..Joint::default()
                    };
                }
            }
        }
        oscillating
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn trips(guard: &mut RingGuard, deg_s: &[f64]) -> Vec<usize> {
        trips_of(guard, deg_s, f64::to_radians)
    }

    fn trips_of(guard: &mut RingGuard, values: &[f64], unit: fn(f64) -> f64) -> Vec<usize> {
        values
            .iter()
            .enumerate()
            .filter_map(|(i, &v)| {
                let mut x = [0.0; MAX_JOINTS];
                x[0] = unit(v);
                guard.step(&x)[0].then_some(i)
            })
            .collect()
    }

    fn run(deg_s: &[f64]) -> Option<usize> {
        trips(&mut RingGuard::new(LOOP_RING), deg_s)
            .first()
            .copied()
    }

    /// J1's reported speed at 250 Hz on the arm, 2026-11-07, a ring building
    /// on an arm-out move and the largest jump of any clean move; the
    /// simulator's J1 ringing slower at four and eight times its gains; one
    /// glitched reply, and a joint held still. Only the rings trip, each on
    /// its third fast swing.
    #[test]
    fn only_a_ring_trips_and_on_its_third_fast_swing() {
        let ring = [
            37.5, 10.7, 54.7, 45.1, -3.2, 71.9, 54.7, -45.1, 147.0, -29.0, 39.7,
        ];
        assert_eq!(run(&ring), Some(10));
        let slower = [151.0, -33.0, -84.0, 88.0, 147.0, -40.0, -79.0, 95.0, 141.0];
        assert_eq!(run(&slower), Some(7));
        let slowest = [
            63.0, 150.0, 68.0, -54.0, -64.0, 41.0, 144.0, 90.0, -39.0, -72.0, 17.0,
        ];
        assert_eq!(run(&slowest), Some(10));
        let clean = [
            -40.8, -40.8, -9.7, -52.6, -29.0, 6.4, -64.4, 1.1, -20.4, -17.2,
        ];
        assert_eq!(run(&clean), None);
        let glitch = [60.1, 62.2, 61.2, 300.0, 63.3, 64.4, 65.4];
        assert_eq!(run(&glitch), None);
        let held = [0.0, 1.1, 0.0, -1.1, 0.0, 1.1, 0.0];
        assert_eq!(run(&held), None);
    }

    /// J1's current \[Nm\] in the hold after arm-out landings on the arm,
    /// every third reply, 2026-11-08: kpp 20 falling into its limit cycle,
    /// the kpp 20 hold that came closest to one and settled, and kpp 10's.
    /// As shares of J1's 2.84 Nm ceiling, only the cycle trips, on its third
    /// swing across the whole ceiling.
    #[test]
    fn only_a_held_joint_s_limit_cycle_trips() {
        let share = |nm: f64| nm / 2.84;
        let cycle = [
            -0.41, 0.14, 0.69, 1.21, 1.65, 1.96, 2.09, 1.94, 1.60, 1.10, 0.57, -0.01, -0.57, -1.08,
            -1.54, -1.92, -2.24, -2.23, -1.94, -1.48, -0.88, -0.27, 0.33, 0.93, 1.49, 1.96, 2.29,
            2.38, 2.14, 1.70, 1.09, 0.46, -0.20, -0.86, -1.48, -2.02, -2.40, -2.58, -2.51, -2.10,
            -1.50, -0.81, -0.08, 0.65, 1.41, 2.08, 2.45, 2.63, 2.66, 2.35,
        ];
        let settled = [
            -1.86, -2.00, -2.00, -1.84, -1.54, -1.12, -0.63, -0.18, 0.22, 0.56, 0.87, 1.11, 1.27,
            1.34, 1.28, 1.10, 0.82, 0.49, 0.14, -0.24, -0.56, -0.83, -1.03, -1.14, -1.16, -1.04,
            -0.84, -0.57, -0.26, 0.06, 0.37, 0.65, 0.87, 1.02, 1.08, 1.04, 0.88, 0.64, 0.38, 0.08,
            -0.23, -0.48, -0.69, -0.84, -0.92, -0.90, -0.76, -0.59, -0.36, -0.10,
        ];
        let calm = [
            0.51, 0.59, 0.64, 0.63, 0.56, 0.47, 0.33, 0.18, 0.04, -0.11, -0.22, -0.29, -0.34,
            -0.36, -0.33, -0.27, -0.19, -0.11, -0.02, 0.06, 0.12, 0.16, 0.18, 0.17, 0.15, 0.10,
            0.05, 0.01, -0.04, -0.08, -0.11, -0.12, -0.12, -0.11, -0.09, -0.07,
        ];
        let guard = || RingGuard::new(HOLD_CYCLE);
        assert_eq!(trips_of(&mut guard(), &cycle, share), [41]);
        assert!(trips_of(&mut guard(), &settled, share).is_empty());
        assert!(trips_of(&mut guard(), &calm, share).is_empty());
    }

    /// A joint calmed after its ring and then held still trips nothing more,
    /// and a fresh ring trips again.
    #[test]
    fn a_ring_trips_once_and_a_fresh_one_again() {
        let ring = [
            37.5, 10.7, 54.7, 45.1, -3.2, 71.9, 54.7, -45.1, 147.0, -29.0, 39.7,
        ];
        let held = [0.0, 1.1, 0.0, -1.1, 0.0, 1.1, 0.0, -1.1];
        let trace: Vec<f64> = [&ring[..], &held, &held, &ring].concat();
        let again = ring.len() + 2 * held.len() + 10;
        assert_eq!(trips(&mut RingGuard::new(LOOP_RING), &trace), [10, again]);
    }
}
