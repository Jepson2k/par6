//! EXEC-mode sample-ring playback (spec/RT.md "EXEC").
//!
//! Pops exactly one [`Sample`] per tick from the planner ring and turns
//! it into the (pos, vel, torque-ff) setpoint. Pause holds in place with
//! the ring untouched; ring starvation holds at the last target. Command
//! boundaries (a `command_index` change or `is_last`) hand off to the
//! [`SettlePolicy`]: `blend_continues` bypasses settling in the same tick
//! so blended corners stay velocity-continuous, a non-blended boundary
//! holds at the boundary target until the policy reports completion (or
//! faults, under `strict`).
//!
//! `completed_index`/`active_command_index` publish 0 for "none" — the
//! planner assigns command indices from 1.

use crate::hooks::{SettlePolicy, SettleVerdict};
use crate::ring::{SampleConsumer, SampleMeta};
use crate::state::ExecStatus;
use crate::MAX_JOINTS;

/// Outcome of one playback tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExecTick {
    /// A sample (or a hold) was emitted.
    Ok,
    /// The settle policy faulted (strict timeout) — the caller latches
    /// the hard error; playback freezes in a hold.
    Fault,
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
    faulted: bool,
}

impl ExecPlayback {
    /// Playback over `consumer` with completion `policy`.
    pub fn new(consumer: SampleConsumer, policy: Box<dyn SettlePolicy>) -> Self {
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
            faulted: false,
        }
    }

    /// EXEC-mode entry: hold at the measured pose until samples arrive.
    pub fn activate(&mut self, q_meas: &[f64; MAX_JOINTS]) {
        self.hold_q = *q_meas;
        self.last_meta = None;
        self.owe_boundary = None;
        self.settling = false;
        self.active_cmd = 0;
        self.completed = 0;
        self.paused = false;
        self.faulted = false;
    }

    /// Pause: hold in place, ring untouched.
    pub fn set_paused(&mut self, paused: bool) {
        self.paused = paused;
    }

    /// Simulator/teleport path: re-aim the starved-ring hold at the
    /// landed pose without touching playback or completion bookkeeping.
    /// The hold re-sends its position target every tick, so leaving it
    /// at the pre-teleport pose would actively drag the arm back there.
    pub fn reseed_hold(&mut self, q: &[f64; MAX_JOINTS]) {
        self.hold_q = *q;
    }

    /// Discard the samples marked for discard (stop/flush path — NOT
    /// pause): everything up to the marked fill generation, so a
    /// command queued right after the stop keeps its samples even when
    /// its fill beat this flush to the RT. Returns the discard count.
    pub fn flush(&mut self) -> usize {
        self.owe_boundary = None;
        self.last_meta = None;
        self.settling = false;
        self.consumer.clear_marked()
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
        if self.paused || self.faulted {
            return ExecTick::Ok;
        }
        if self.settling {
            match self.policy.tick(q_meas, &self.hold_q) {
                SettleVerdict::Settling => return ExecTick::Ok,
                SettleVerdict::Complete => {
                    self.settling = false;
                    self.completed = self.armed_cmd;
                    // fall through: playback resumes this tick
                }
                SettleVerdict::Fault => {
                    self.faulted = true;
                    self.settling = false;
                    return ExecTick::Fault;
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
        match self.consumer.pop() {
            None => ExecTick::Ok, // starved or program done: hold
            Some(s) => {
                self.hold_q = s.q;
                self.active_cmd = s.meta.command_index;
                *q = s.q;
                *qd = s.qd;
                for (out, ff) in tau_ff.iter_mut().zip(s.tau_ff) {
                    *out = f64::from(ff);
                }
                if s.meta.is_last {
                    self.owe_boundary = Some(s.meta);
                    self.last_meta = None;
                } else {
                    self.last_meta = Some(s.meta);
                }
                ExecTick::Ok
            }
        }
    }

    /// Samples currently queued in the ring.
    pub fn samples_remaining(&self) -> usize {
        self.consumer.samples_remaining()
    }

    /// Live state for the snapshot.
    pub fn status(&self) -> ExecStatus {
        ExecStatus {
            samples_remaining: self.consumer.samples_remaining() as u64,
            active_command_index: self.active_cmd,
            completed_index: self.completed,
            settling: self.settling,
            paused: self.paused,
        }
    }
}
