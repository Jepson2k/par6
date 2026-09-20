//! EXEC-mode sample-ring playback.
//!
//! Advances a bounded fractional clock through the planner's samples,
//! deriving position and velocity from the same interpolant. Pause
//! decelerates to a hold with the remaining ring intact; starvation
//! holds at the last target. Command
//! boundaries (a `command_index` change or `is_last`) hand off to the
//! [`SettlePolicy`]: `blend_continues` bypasses settling in the same tick
//! so blended corners stay velocity-continuous, a non-blended boundary
//! holds at the boundary target until the policy reports completion (or
//! faults, under `strict`).
//!
//! `completed_index`/`active_command_index` publish 0 for "none" — the
//! planner assigns command indices from 1.

use crate::hooks::{SettlePolicy, SettleVerdict};
use crate::ring::{Sample, SampleConsumer, SampleMeta};
use crate::state::ExecStatus;
use crate::MAX_JOINTS;

/// Outcome of one playback tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ExecTick {
    /// A sample (or a hold) was emitted.
    Ok,
    /// The settle policy faulted (strict timeout) — the caller latches
    /// the hard error; playback freezes in a hold.
    Fault {
        /// Joint furthest from its target when the window closed.
        joint: u8,
        /// That joint's residual \[rad\].
        residual_rad: f64,
    },
}

/// The EXEC playback engine. One per RT core; owns the consumer half of
/// the sample ring.
pub struct ExecPlayback {
    consumer: SampleConsumer,
    policy: Box<dyn SettlePolicy>,
    hold_q: [f64; MAX_JOINTS],
    last_meta: Option<SampleMeta>,
    owe_boundary: Option<SampleMeta>,
    settling: bool,
    armed_cmd: u32,
    active_cmd: u32,
    completed: u32,
    paused: bool,
    paused_ticks: u64,
    faulted: bool,
    dt: f64,
    scale_rate_limit: f64,
    scale_acceleration_limit: f64,
    scale_rate: f64,
    acceleration_limits: [f64; MAX_JOINTS],
    requested_scale: f64,
    applied_scale: f64,
    phase: f64,
    left: Sample,
}

impl ExecPlayback {
    /// Playback over `consumer` with completion `policy`.
    pub fn new(
        consumer: SampleConsumer,
        policy: Box<dyn SettlePolicy>,
        dt: f64,
        transition_s: f64,
        acceleration_limits: [f64; MAX_JOINTS],
    ) -> Self {
        assert!(dt.is_finite() && dt > 0.0);
        assert!(transition_s.is_finite() && transition_s > 0.0);
        Self {
            consumer,
            policy,
            hold_q: [0.0; MAX_JOINTS],
            last_meta: None,
            owe_boundary: None,
            settling: false,
            armed_cmd: 0,
            active_cmd: 0,
            completed: 0,
            paused: false,
            paused_ticks: 0,
            faulted: false,
            dt,
            scale_rate_limit: 1.0 / transition_s,
            scale_acceleration_limit: 4.0 / (transition_s * transition_s),
            scale_rate: 0.0,
            acceleration_limits,
            requested_scale: 1.0,
            applied_scale: 1.0,
            phase: 0.0,
            left: Sample::default(),
        }
    }

    /// EXEC-mode entry: hold at the measured pose until samples arrive.
    /// A pause requested before entry stands: the operator who paused an
    /// idle arm expects the next program to start held, not moving.
    pub fn activate(&mut self, q_meas: &[f64; MAX_JOINTS]) {
        self.hold_q = *q_meas;
        self.last_meta = None;
        self.owe_boundary = None;
        self.settling = false;
        self.active_cmd = 0;
        self.completed = 0;
        self.faulted = false;
        self.phase = 0.0;
        self.left = Sample {
            q: *q_meas,
            ..Sample::default()
        };
        self.applied_scale = self.target_scale();
        self.scale_rate = 0.0;
    }

    /// Decelerate to a pause, or resume at the last positive scale.
    pub fn set_paused(&mut self, paused: bool) {
        self.paused = paused;
    }

    /// Keep queued dwell timing independent of planner polling cadence.
    pub fn clock_tick(&mut self) {
        if self.paused {
            self.paused_ticks += 1;
        }
    }

    /// Selecting a speed preserves the explicit pause state.
    pub fn set_speed_scale(&mut self, scale: f64) -> bool {
        if !scale.is_finite() || !(0.1..=1.0).contains(&scale) {
            return false;
        }
        self.requested_scale = scale;
        true
    }

    fn target_scale(&self) -> f64 {
        if self.paused {
            0.0
        } else {
            self.requested_scale
        }
    }

    /// Readback also advances in modes without queued motion.
    pub fn at_rest(&mut self) {
        self.applied_scale = self.target_scale();
        self.scale_rate = 0.0;
    }

    /// Simulator/teleport path: re-aim the starved-ring hold at the
    /// landed pose without touching playback or completion bookkeeping.
    /// The hold re-sends its position target every tick, so leaving it
    /// at the pre-teleport pose would actively drag the arm back there.
    pub fn reseed_hold(&mut self, q: &[f64; MAX_JOINTS]) {
        self.hold_q = *q;
        self.left = Sample {
            q: *q,
            ..Sample::default()
        };
        self.phase = 0.0;
    }

    /// Discard the samples marked for discard (stop/flush path — NOT
    /// pause): everything up to the marked fill generation, so a
    /// command queued right after the stop keeps its samples even when
    /// its fill beat this flush to the RT. Returns the discard count.
    pub fn flush(&mut self) -> usize {
        self.owe_boundary = None;
        self.last_meta = None;
        self.settling = false;
        let dropped = self.consumer.clear_marked();
        if dropped > 0 {
            self.left = Sample {
                q: self.hold_q,
                ..Sample::default()
            };
            self.phase = 0.0;
        }
        dropped
    }

    /// Replace the completion policy (takes effect at the next boundary).
    pub fn set_policy(&mut self, policy: Box<dyn SettlePolicy>) {
        self.policy = policy;
    }

    /// One tick: writes the setpoint into `q`/`qd`/`tau_ff` (gravity is
    /// NOT included — dispatch adds G(q) on top).
    pub fn tick(
        &mut self,
        q_meas: &[f64; MAX_JOINTS],
        q: &mut [f64; MAX_JOINTS],
        qd: &mut [f64; MAX_JOINTS],
        tau_ff: &mut [f64; MAX_JOINTS],
    ) -> ExecTick {
        *q = self.hold_q;
        qd.fill(0.0);
        tau_ff.fill(0.0);
        if self.faulted {
            return ExecTick::Ok;
        }
        if self.paused && self.applied_scale == 0.0 {
            return ExecTick::Ok;
        }
        if self.settling {
            match self.policy.tick(q_meas, &self.hold_q) {
                SettleVerdict::Settling => {
                    self.at_rest();
                    return ExecTick::Ok;
                }
                SettleVerdict::Complete => {
                    self.settling = false;
                    self.completed = self.armed_cmd;
                    // fall through: playback resumes this tick
                }
                SettleVerdict::Fault {
                    joint,
                    residual_rad,
                } => {
                    self.faulted = true;
                    self.settling = false;
                    return ExecTick::Fault {
                        joint,
                        residual_rad,
                    };
                }
            }
        }
        // A boundary discovered late (ring was starved at the command
        // end): the next command's first sample is visible with a new
        // index, so the previous command's boundary must resolve first.
        if self.owe_boundary.is_none() {
            if let (Some(last), Some(next)) = (self.last_meta, self.consumer.peek()) {
                if next.meta.command_index != last.command_index {
                    self.owe_boundary = Some(last);
                }
            }
        }
        if let Some(boundary) = self.owe_boundary.take() {
            self.last_meta = None;
            if self.policy.arm(boundary.blend_continues) {
                // Immediate completion (commanded policy or blend-through):
                // no hold tick, motion continues below.
                self.completed = boundary.command_index;
            } else {
                self.settling = true;
                self.armed_cmd = boundary.command_index;
                return ExecTick::Ok;
            }
        }
        let Some(next) = self.consumer.peek() else {
            self.at_rest();
            return ExecTick::Ok;
        };
        if next.meta.command_index != self.left.meta.command_index
            && !self.left.meta.blend_continues
        {
            // The planner's initial measured pose can differ from the old
            // hold target. A synthetic bridge would bypass its path checks.
            self.left = match next.start {
                Some(start) => Sample {
                    q: start.q,
                    tau_ff: start.tau_ff,
                    meta: next.meta,
                    ..Sample::default()
                },
                None => next,
            };
            self.phase = 0.0;
        }
        let (_, velocity, acceleration) = interpolate(&self.left, &next, self.phase, self.dt);
        if velocity.iter().all(|v| v.abs() < 1e-12) && acceleration.iter().all(|a| a.abs() < 1e-12)
        {
            self.at_rest();
        }
        let target = self.target_scale();
        let old_scale = self.applied_scale;
        let mut low = -self.scale_rate_limit;
        let mut high = self.scale_rate_limit;
        for i in 0..MAX_JOINTS {
            if velocity[i].abs() > 1e-12 {
                let base = old_scale * old_scale * acceleration[i];
                let a = (-self.acceleration_limits[i] - base) / velocity[i];
                let b = (self.acceleration_limits[i] - base) / velocity[i];
                low = low.max(a.min(b));
                high = high.min(a.max(b));
            }
        }
        // A speed selection must not step the clock's first derivative:
        // that would step joint acceleration even on a smooth path.
        let frequency = (8.0 * self.scale_rate_limit).min(0.5 / self.dt);
        let clock_acceleration = (frequency * frequency * (target - old_scale)
            - 2.0 * frequency * self.scale_rate)
            .clamp(
                -self.scale_acceleration_limit,
                self.scale_acceleration_limit,
            );
        let requested_rate = self.scale_rate + clock_acceleration * self.dt;
        let rate = if requested_rate == 0.0 {
            0.0
        } else if low <= high {
            requested_rate.clamp(low, high)
        } else {
            0.0
        };
        let rate = if rate * requested_rate < 0.0 {
            0.0
        } else {
            rate
        };
        let candidate = (old_scale + rate * self.dt).clamp(0.0, 1.0);
        self.applied_scale = candidate;
        if candidate != old_scale {
            let following = self.consumer.peek_offset(1);
            // The path can curve during this tick. A bound computed only
            // at its start misses the acceleration at the next setpoint.
            let admissible = |scale| {
                self.rate_is_admissible(old_scale, scale, &velocity, &next, following.as_ref())
            };
            if !admissible(candidate) {
                let mut allowed = 0.0;
                let mut refused = 1.0;
                for _ in 0..16 {
                    let fraction = 0.5 * (allowed + refused);
                    if admissible(old_scale + fraction * (candidate - old_scale)) {
                        allowed = fraction;
                    } else {
                        refused = fraction;
                    }
                }
                self.applied_scale = old_scale + allowed * (candidate - old_scale);
            }
        }
        // Use the readback's scale resolution, so an asymptotic filter
        // tail cannot leave a physically stationary program "pausing".
        if (self.applied_scale - target).abs() < 1e-6
            && (target - old_scale).abs() <= self.scale_rate_limit * self.dt
            && self.rate_is_admissible(
                old_scale,
                target,
                &velocity,
                &next,
                self.consumer.peek_offset(1).as_ref(),
            )
        {
            self.applied_scale = target;
        }
        let rate = (self.applied_scale - old_scale) / self.dt;
        self.scale_rate = rate;
        self.phase += 0.5 * (old_scale + self.applied_scale);
        if self.phase >= 1.0 {
            self.phase -= 1.0;
            self.left = self
                .consumer
                .pop()
                .expect("peeked sample belongs to this consumer");
            self.active_cmd = self.left.meta.command_index;
            if self.left.meta.is_last {
                self.owe_boundary = Some(self.left.meta);
                self.last_meta = None;
                self.phase = 0.0;
            } else {
                self.last_meta = Some(self.left.meta);
                if let Some(next) = self.consumer.peek() {
                    if next.meta.command_index != self.left.meta.command_index {
                        self.owe_boundary = self.last_meta.take();
                        if !self.left.meta.blend_continues {
                            self.phase = 0.0;
                        }
                    }
                }
            }
        }
        let right = match self.consumer.peek() {
            Some(right) => right,
            None => {
                self.phase = 0.0;
                self.left
            }
        };
        let (position, velocity, _) = interpolate(&self.left, &right, self.phase, self.dt);
        *q = position;
        self.hold_q = position;
        let square = self.applied_scale * self.applied_scale;
        for i in 0..MAX_JOINTS {
            qd[i] = velocity[i] * self.applied_scale;
            let ff = f64::from(self.left.tau_ff[i]) * (1.0 - self.phase)
                + f64::from(right.tau_ff[i]) * self.phase;
            let inertia_velocity = f64::from(self.left.inertia_velocity[i]) * (1.0 - self.phase)
                + f64::from(right.inertia_velocity[i]) * self.phase;
            tau_ff[i] = square * ff + rate * inertia_velocity;
        }
        ExecTick::Ok
    }

    fn rate_is_admissible(
        &self,
        old_scale: f64,
        scale: f64,
        old_velocity: &[f64; MAX_JOINTS],
        next: &Sample,
        following: Option<&Sample>,
    ) -> bool {
        let mut phase = self.phase + 0.5 * (old_scale + scale);
        let (left, right) = if phase >= 1.0 {
            phase -= 1.0;
            match following {
                Some(right)
                    if !next.meta.is_last
                        && (right.meta.command_index == next.meta.command_index
                            || next.meta.blend_continues) =>
                {
                    (next, right)
                }
                _ => {
                    phase = 0.0;
                    (next, next)
                }
            }
        } else {
            (&self.left, next)
        };
        let (_, velocity, acceleration) = interpolate(left, right, phase, self.dt);
        let rate = (scale - old_scale) / self.dt;
        (0..MAX_JOINTS).all(|i| {
            let sampled = (scale * velocity[i] - old_scale * old_velocity[i]) / self.dt;
            let instantaneous = scale * scale * acceleration[i] + rate * velocity[i];
            sampled.abs() <= self.acceleration_limits[i]
                && instantaneous.abs() <= self.acceleration_limits[i]
        })
    }

    /// Samples currently queued in the ring.
    pub fn samples_remaining(&self) -> usize {
        self.consumer.samples_remaining()
    }

    /// Nothing left to play: the ring is drained, every boundary has
    /// resolved and no settle is pending — the engine is emitting a
    /// hold at the last sample, which is a position hold and nothing
    /// more. Paused or faulted playback is NOT idle: both still own the
    /// program.
    pub fn is_holding_after_completion(&self) -> bool {
        !self.paused
            && !self.faulted
            && !self.settling
            && self.owe_boundary.is_none()
            && self.consumer.samples_remaining() == 0
    }

    /// Live state for the snapshot.
    pub fn status(&self) -> ExecStatus {
        ExecStatus {
            samples_remaining: self.consumer.samples_remaining() as u64,
            active_command_index: self.active_cmd,
            completed_index: self.completed,
            settling: self.settling,
            paused: self.paused && self.applied_scale == 0.0,
            target_scale: self.target_scale(),
            applied_scale: self.applied_scale,
            resume_scale: self.requested_scale,
            paused_ticks: self.paused_ticks,
        }
    }
}

/// The position curve shared by RT playback and planner-side validation.
pub struct SampleInterval {
    origin: [f64; MAX_JOINTS],
    c1: [f64; MAX_JOINTS],
    c2: [f64; MAX_JOINTS],
    c3: [f64; MAX_JOINTS],
    dt: f64,
}

impl SampleInterval {
    /// Cubic Hermite interpolation over one nominal tick.
    pub fn new(left: &Sample, right: &Sample, dt: f64) -> Self {
        let mut interval = Self {
            origin: left.q,
            c1: [0.0; MAX_JOINTS],
            c2: [0.0; MAX_JOINTS],
            c3: [0.0; MAX_JOINTS],
            dt,
        };
        for i in 0..MAX_JOINTS {
            let delta = right.q[i] - left.q[i];
            interval.c1[i] = left.qd[i] * dt;
            interval.c2[i] = 3.0 * delta - (2.0 * left.qd[i] + right.qd[i]) * dt;
            interval.c3[i] = -2.0 * delta + (left.qd[i] + right.qd[i]) * dt;
        }
        interval
    }

    /// Position and nominal time derivatives at a phase in `[0, 1]`.
    pub fn sample(&self, phase: f64) -> ([f64; MAX_JOINTS], [f64; MAX_JOINTS], [f64; MAX_JOINTS]) {
        let mut q = [0.0; MAX_JOINTS];
        let mut v = [0.0; MAX_JOINTS];
        let mut a = [0.0; MAX_JOINTS];
        for i in 0..MAX_JOINTS {
            let (b, c, d) = (self.c1[i], self.c2[i], self.c3[i]);
            q[i] = self.origin[i] + phase * (b + phase * (c + phase * d));
            v[i] = (b + phase * (2.0 * c + 3.0 * phase * d)) / self.dt;
            a[i] = (2.0 * c + 6.0 * phase * d) / (self.dt * self.dt);
        }
        (q, v, a)
    }

    /// Endpoints and every interior joint-position extremum, in phase order.
    pub fn position_extrema(&self) -> ([f64; 2 * MAX_JOINTS + 2], usize) {
        let mut phases = [0.0; 2 * MAX_JOINTS + 2];
        phases[1] = 1.0;
        let mut count = 2;
        for i in 0..MAX_JOINTS {
            let (a, b, c) = (3.0 * self.c3[i], 2.0 * self.c2[i], self.c1[i]);
            let roots = if a == 0.0 {
                [-c / b, f64::NAN]
            } else {
                let discriminant = b * b - 4.0 * a * c;
                let q = -0.5 * (b + discriminant.sqrt().copysign(b));
                [q / a, c / q]
            };
            for phase in roots {
                if phase > 0.0 && phase < 1.0 {
                    phases[count] = phase;
                    count += 1;
                }
            }
        }
        phases[..count].sort_unstable_by(f64::total_cmp);
        (phases, count)
    }

    /// Upper bound on joint-space travel per unit phase (maximum norm).
    pub fn travel_bound(&self) -> f64 {
        let mut bound: f64 = 0.0;
        for i in 0..MAX_JOINTS {
            let (b, c, d) = (self.c1[i], self.c2[i], self.c3[i]);
            bound = bound.max(b.abs()).max((b + 2.0 * c + 3.0 * d).abs());
            let phase = -c / (3.0 * d);
            if phase > 0.0 && phase < 1.0 {
                bound = bound.max((b + phase * (2.0 * c + 3.0 * phase * d)).abs());
            }
        }
        bound
    }
}

fn interpolate(
    left: &Sample,
    right: &Sample,
    phase: f64,
    dt: f64,
) -> ([f64; MAX_JOINTS], [f64; MAX_JOINTS], [f64; MAX_JOINTS]) {
    SampleInterval::new(left, right, dt).sample(phase)
}
