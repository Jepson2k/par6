//! Planned-move trajectory generation: a queued program of joint-space
//! moves compiled into a tick-rate [`Sample`] stream for the EXEC ring.
//!
//! Two profiles today (see [`ProfileKind`] for the TOPPRA slot):
//!
//! - **Trapezoid**: accel–cruise–decel run on the normalized path
//!   coordinate `s`, which synchronizes all joints on the slowest one
//!   (the binding joint sets the scalar velocity/acceleration budget).
//!   Corner blending overlaps one segment's deceleration tail with the
//!   next segment's acceleration head; both blending ramps are planned at
//!   HALF the acceleration limit so the summed contribution never exceeds
//!   it, and the summed velocity is bounded by the larger of the two
//!   cruise velocities — both bounds hold per joint.
//! - **Ruckig**: jerk-limited point-to-point via rsruckig. A blend chain
//!   becomes one rsruckig calculation with the interior targets as
//!   `intermediate_positions` (pass-through waypoints, velocity-continuous
//!   corners, limits enforced by the solver); per-move speed fractions and
//!   minimum durations map to per-section limits.
//!
//! Sample metadata carries the ring contract: `command_index` per queued
//! move, `checkpoint_id` boundaries, `blend_continues` on every sample of
//! a move that blends into the next (the completion policy must not settle
//! there), `is_last` on the final sample of the program.

use rsruckig::prelude::*;

use crate::path::{JointLinePath, PathSampler};
use crate::{MotionError, MotionLimits, Sample, SampleMeta, NUM_JOINTS};

/// Displacements below this count as "joint does not move" \[rad\].
const ZERO_DELTA: f64 = 1e-12;

/// Profile registry for planned moves.
///
/// A `Toppra` variant slots in here when the C++ toppra shim lands (the
/// FFI entry points `par6_traj_*` are stubbed NOT_IMPLEMENTED until
/// conda-forge ships C++ toppra); it will consume arbitrary
/// [`PathSampler`] geometry — cartesian paths included — with
/// curvature-aware constraint handling.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ProfileKind {
    /// Jerk-limited point-to-point (rsruckig). Requires finite jerk
    /// limits. The default planned-move profile.
    #[default]
    Ruckig,
    /// Trapezoidal velocity profile on the path coordinate (accel–cruise–
    /// decel; no jerk limiting).
    Trapezoid,
}

/// Per-move parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MoveParams {
    /// Velocity profile shape.
    pub profile: ProfileKind,
    /// Scales the velocity limit for this move, in `(0, 1]`.
    pub speed_fraction: f64,
    /// Stretch the move to at least this duration \[s\]. Shorter requests
    /// than the limit-constrained minimum have no effect.
    pub min_duration_s: Option<f64>,
    /// Blend this move's corner into the next queued move
    /// (velocity-continuous handoff, no settle at the boundary). Ignored
    /// on the last move of a program.
    pub blend_with_next: bool,
    /// Checkpoint label carried on this move's samples; defaults to the
    /// move's command index.
    pub checkpoint_id: Option<u32>,
}

impl Default for MoveParams {
    fn default() -> Self {
        Self {
            profile: ProfileKind::default(),
            speed_fraction: 1.0,
            min_duration_s: None,
            blend_with_next: false,
            checkpoint_id: None,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct MoveSpec {
    target: [f64; NUM_JOINTS],
    params: MoveParams,
}

/// A compiled program: tick-rate samples ready for the EXEC ring.
///
/// The planner feeds these into the ring under `samples_remaining`
/// backpressure; generation itself is planner-side and may allocate.
#[derive(Debug, Clone, PartialEq)]
pub struct Plan {
    samples: Vec<Sample>,
    dt: f64,
}

impl Plan {
    /// The tick-rate sample stream, one entry per tick starting one tick
    /// after motion begin.
    pub fn samples(&self) -> &[Sample] {
        &self.samples
    }

    /// Total program duration \[s\].
    pub fn duration_s(&self) -> f64 {
        self.samples.len() as f64 * self.dt
    }

    /// Number of samples (ticks).
    pub fn len(&self) -> usize {
        self.samples.len()
    }

    /// True when the plan holds no samples.
    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }
}

/// Builder for a queued program of joint-space moves.
pub struct ProgramBuilder {
    limits: MotionLimits,
    dt: f64,
    start: [f64; NUM_JOINTS],
    moves: Vec<MoveSpec>,
}

impl ProgramBuilder {
    /// Start a program at joint pose `start` \[rad\] under `limits`
    /// (normally the EXEC block) with tick period `dt` \[s\].
    pub fn new(
        start: [f64; NUM_JOINTS],
        limits: MotionLimits,
        dt: f64,
    ) -> Result<Self, MotionError> {
        if !(dt.is_finite() && dt > 0.0 && dt < 1.0) {
            return Err(MotionError::InvalidInput {
                what: "dt",
                reason: format!("must be a finite tick period in (0, 1) s, got {dt}"),
            });
        }
        if start.iter().any(|v| !v.is_finite()) {
            return Err(MotionError::InvalidInput {
                what: "start",
                reason: format!("joint positions must be finite, got {start:?}"),
            });
        }
        Ok(Self {
            limits,
            dt,
            start,
            moves: Vec::new(),
        })
    }

    /// Queue a joint-space move to `target` \[rad\].
    pub fn move_j(
        &mut self,
        target: [f64; NUM_JOINTS],
        params: MoveParams,
    ) -> Result<&mut Self, MotionError> {
        if target.iter().any(|v| !v.is_finite()) {
            return Err(MotionError::InvalidInput {
                what: "target",
                reason: format!("joint positions must be finite, got {target:?}"),
            });
        }
        self.limits.require_inside_soft(&target)?;
        if !(params.speed_fraction.is_finite()
            && params.speed_fraction > 0.0
            && params.speed_fraction <= 1.0)
        {
            return Err(MotionError::InvalidInput {
                what: "speed_fraction",
                reason: format!("must be in (0, 1], got {}", params.speed_fraction),
            });
        }
        if let Some(d) = params.min_duration_s {
            if !(d.is_finite() && d > 0.0) {
                return Err(MotionError::InvalidInput {
                    what: "min_duration_s",
                    reason: format!("must be finite and > 0, got {d}"),
                });
            }
        }
        self.moves.push(MoveSpec { target, params });
        Ok(self)
    }

    /// Compile the queued moves into a tick-rate sample stream.
    pub fn plan(&self) -> Result<Plan, MotionError> {
        if self.moves.is_empty() {
            return Err(MotionError::InvalidInput {
                what: "moves",
                reason: "program has no moves".into(),
            });
        }
        let mut samples: Vec<Sample> = Vec::new();
        let mut chain_start = self.start;
        let mut i = 0;
        while i < self.moves.len() {
            let mut j = i;
            while j + 1 < self.moves.len() && self.moves[j].params.blend_with_next {
                j += 1;
            }
            let chain = &self.moves[i..=j];
            for (w, pair) in chain.windows(2).enumerate() {
                if pair[0].params.profile != pair[1].params.profile {
                    return Err(MotionError::MixedProfileBlend {
                        first: i + w,
                        second: i + w + 1,
                    });
                }
            }
            match chain[0].params.profile {
                ProfileKind::Trapezoid => {
                    self.emit_trapezoid_chain(&mut samples, &chain_start, chain, i as u32);
                }
                ProfileKind::Ruckig => {
                    self.emit_ruckig_chain(&mut samples, &chain_start, chain, i as u32)?;
                }
            }
            chain_start = self.moves[j].target;
            i = j + 1;
        }
        if let Some(last) = samples.last_mut() {
            last.meta.is_last = true;
        }
        Ok(Plan {
            samples,
            dt: self.dt,
        })
    }

    fn meta_for(&self, chain: &[MoveSpec], chain_offset: u32, k: usize) -> SampleMeta {
        let cmd = chain_offset + k as u32;
        SampleMeta {
            command_index: cmd,
            checkpoint_id: chain[k].params.checkpoint_id.unwrap_or(cmd),
            blend_continues: k + 1 < chain.len(),
            is_last: false,
        }
    }

    fn emit_trapezoid_chain(
        &self,
        out: &mut Vec<Sample>,
        start: &[f64; NUM_JOINTS],
        chain: &[MoveSpec],
        chain_offset: u32,
    ) {
        let mut segs = Vec::with_capacity(chain.len());
        let mut prev = *start;
        for (k, mv) in chain.iter().enumerate() {
            let path = JointLinePath::new(prev, mv.target);
            let mut scale = [0.0; NUM_JOINTS];
            for (s, (a, b)) in scale.iter_mut().zip(prev.iter().zip(mv.target.iter())) {
                *s = (b - a).abs();
            }
            segs.push(trapezoid_segment(
                &path,
                &scale,
                &self.limits,
                mv.params.speed_fraction,
                mv.params.min_duration_s,
                k > 0,
                k + 1 < chain.len(),
                self.dt,
            ));
            prev = mv.target;
        }
        // Overlap between consecutive segments, capped so the two splices
        // touching a segment never claim overlapping sample ranges.
        let mut overlaps = vec![0usize; chain.len().saturating_sub(1)];
        for (k, ov) in overlaps.iter_mut().enumerate() {
            *ov = segs[k]
                .exit_ticks
                .min(segs[k + 1].entry_ticks)
                .min(segs[k].q.len() / 2)
                .min(segs[k + 1].q.len() / 2);
        }
        let mut consumed_head = 0usize;
        for (k, seg) in segs.iter().enumerate() {
            let meta = self.meta_for(chain, chain_offset, k);
            let ov_next = overlaps.get(k).copied().unwrap_or(0);
            let len = seg.q.len();
            for t in consumed_head..len - ov_next {
                out.push(Sample {
                    q: seg.q[t],
                    qd: seg.qd[t],
                    tau_ff: [0.0; NUM_JOINTS],
                    meta,
                });
            }
            if ov_next > 0 {
                let next = &segs[k + 1];
                let corner = &chain[k].target;
                for t in 0..ov_next {
                    let mut q = [0.0; NUM_JOINTS];
                    let mut qd = [0.0; NUM_JOINTS];
                    for j in 0..NUM_JOINTS {
                        q[j] = seg.q[len - ov_next + t][j] + next.q[t][j] - corner[j];
                        qd[j] = seg.qd[len - ov_next + t][j] + next.qd[t][j];
                    }
                    out.push(Sample {
                        q,
                        qd,
                        tau_ff: [0.0; NUM_JOINTS],
                        meta,
                    });
                }
            }
            consumed_head = ov_next;
        }
    }

    fn emit_ruckig_chain(
        &self,
        out: &mut Vec<Sample>,
        start: &[f64; NUM_JOINTS],
        chain: &[MoveSpec],
        chain_offset: u32,
    ) -> Result<(), MotionError> {
        self.limits.require_finite_jerk()?;
        let n_way = chain.len() - 1;
        let mut otg;
        let mut input;
        let mut output;
        if n_way > 0 {
            otg = Ruckig::<NUM_JOINTS, ThrowErrorHandler>::with_waypoints(None, self.dt, n_way);
            input = InputParameter::<NUM_JOINTS>::with_waypoints(None, n_way);
            output = OutputParameter::<NUM_JOINTS>::with_waypoints(None, n_way);
        } else {
            otg = Ruckig::<NUM_JOINTS, ThrowErrorHandler>::new(None, self.dt);
            input = InputParameter::<NUM_JOINTS>::new(None);
            output = OutputParameter::<NUM_JOINTS>::new(None);
        }
        let last = chain.len() - 1;
        for (j, &q0) in start.iter().enumerate() {
            input.current_position[j] = q0;
            input.target_position[j] = chain[last].target[j];
            input.max_velocity[j] = self.limits.velocity[j];
            input.max_acceleration[j] = self.limits.acceleration[j];
            input.max_jerk[j] = self.limits.jerk[j];
        }
        if n_way > 0 {
            for mv in &chain[..last] {
                input
                    .intermediate_positions
                    .push(DataArrayOrVec::Stack(mv.target));
            }
            let mut per_vel = Vec::with_capacity(chain.len());
            for mv in chain {
                let mut v = [0.0; NUM_JOINTS];
                for (dst, lim) in v.iter_mut().zip(self.limits.velocity.iter()) {
                    *dst = lim * mv.params.speed_fraction;
                }
                per_vel.push(DataArrayOrVec::Stack(v));
            }
            input.per_section_max_velocity = Some(per_vel);
            if chain.iter().any(|m| m.params.min_duration_s.is_some()) {
                input.per_section_minimum_duration = Some(
                    chain
                        .iter()
                        .map(|m| m.params.min_duration_s.unwrap_or(0.0))
                        .collect(),
                );
            }
        } else {
            for (dst, lim) in input
                .max_velocity
                .iter_mut()
                .zip(self.limits.velocity.iter())
            {
                *dst = lim * chain[0].params.speed_fraction;
            }
            input.minimum_duration = chain[0].params.min_duration_s;
        }

        let mut cap: Option<usize> = None;
        let mut n_emitted = 0usize;
        loop {
            let res = otg
                .update(&input, &mut output)
                .map_err(|e| MotionError::Ruckig(e.to_string()))?;
            let finished = matches!(res, RuckigResult::Finished);
            if !finished && !matches!(res, RuckigResult::Working) {
                return Err(MotionError::Ruckig(format!(
                    "trajectory calculation failed: {res:?}"
                )));
            }
            if cap.is_none() {
                let dur = output.trajectory.get_duration();
                if !dur.is_finite() {
                    return Err(MotionError::Ruckig("non-finite trajectory duration".into()));
                }
                cap = Some((dur / self.dt).ceil() as usize + 16);
            }
            let mut q = [0.0; NUM_JOINTS];
            let mut qd = [0.0; NUM_JOINTS];
            for j in 0..NUM_JOINTS {
                q[j] = output.new_position[j];
                qd[j] = output.new_velocity[j];
            }
            let k = output.new_section.min(last);
            out.push(Sample {
                q,
                qd,
                tau_ff: [0.0; NUM_JOINTS],
                meta: self.meta_for(chain, chain_offset, k),
            });
            n_emitted += 1;
            if finished {
                return Ok(());
            }
            if n_emitted >= cap.unwrap_or(usize::MAX) {
                return Err(MotionError::Ruckig(
                    "trajectory sampling ran past its computed duration".into(),
                ));
            }
            output.pass_to_input(&mut input);
        }
    }
}

struct SegSamples {
    q: Vec<[f64; NUM_JOINTS]>,
    qd: Vec<[f64; NUM_JOINTS]>,
    entry_ticks: usize,
    exit_ticks: usize,
}

/// Scalar asymmetric trapezoid over a unit distance: accelerate at `a_in`,
/// cruise at `v`, decelerate at `a_out`.
struct STrapezoid {
    a_in: f64,
    a_out: f64,
    v: f64,
    t_in: f64,
    t_cruise: f64,
    t_total: f64,
}

impl STrapezoid {
    fn new(v_max: f64, a_in: f64, a_out: f64, min_duration: Option<f64>) -> Self {
        // Peak velocity of the pure triangular profile over distance 1.
        let v_tri = (2.0 * a_in * a_out / (a_in + a_out)).sqrt();
        let mut v = v_max.min(v_tri);
        if let Some(td) = min_duration {
            // T(v) = v/(2·a_in) + v/(2·a_out) + 1/v; stretch by lowering
            // the cruise velocity when the requested duration is longer.
            let c2 = 1.0 / (2.0 * a_in) + 1.0 / (2.0 * a_out);
            let t_min = c2 * v + 1.0 / v;
            if td > t_min {
                v = (td - (td * td - 4.0 * c2).sqrt()) / (2.0 * c2);
            }
        }
        let t_in = v / a_in;
        let t_out = v / a_out;
        let d_ramps = v * v / (2.0 * a_in) + v * v / (2.0 * a_out);
        let t_cruise = ((1.0 - d_ramps) / v).max(0.0);
        Self {
            a_in,
            a_out,
            v,
            t_in,
            t_cruise,
            t_total: t_in + t_cruise + t_out,
        }
    }

    /// `(s, ds/dt)` at time `t`, clamped to the profile ends.
    fn sample(&self, t: f64) -> (f64, f64) {
        if t <= 0.0 {
            return (0.0, 0.0);
        }
        if t >= self.t_total {
            return (1.0, 0.0);
        }
        if t < self.t_in {
            (0.5 * self.a_in * t * t, self.a_in * t)
        } else if t < self.t_in + self.t_cruise {
            let d_in = self.v * self.v / (2.0 * self.a_in);
            (d_in + self.v * (t - self.t_in), self.v)
        } else {
            let tt = self.t_total - t;
            (1.0 - 0.5 * self.a_out * tt * tt, self.a_out * tt)
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn trapezoid_segment(
    path: &dyn PathSampler,
    scale: &[f64; NUM_JOINTS],
    limits: &MotionLimits,
    speed_fraction: f64,
    min_duration_s: Option<f64>,
    half_entry: bool,
    half_exit: bool,
    dt: f64,
) -> SegSamples {
    let entry_scale = if half_entry { 0.5 } else { 1.0 };
    let exit_scale = if half_exit { 0.5 } else { 1.0 };
    let mut v_s = f64::INFINITY;
    let mut a_in_s = f64::INFINITY;
    let mut a_out_s = f64::INFINITY;
    for (j, &sc) in scale.iter().enumerate() {
        if sc > ZERO_DELTA {
            v_s = v_s.min(limits.velocity[j] * speed_fraction / sc);
            a_in_s = a_in_s.min(limits.acceleration[j] * entry_scale / sc);
            a_out_s = a_out_s.min(limits.acceleration[j] * exit_scale / sc);
        }
    }
    if !v_s.is_finite() {
        // Nothing moves: a single hold sample keeps the command's
        // checkpoint boundary observable in the stream.
        let mut q = [0.0; NUM_JOINTS];
        path.sample(1.0, &mut q);
        return SegSamples {
            q: vec![q],
            qd: vec![[0.0; NUM_JOINTS]],
            entry_ticks: 0,
            exit_ticks: 0,
        };
    }
    let prof = STrapezoid::new(v_s, a_in_s, a_out_s, min_duration_s);
    let n = ((prof.t_total / dt).ceil() as usize).max(1);
    let mut qs = Vec::with_capacity(n);
    let mut qds = Vec::with_capacity(n);
    let mut dq_ds = [0.0; NUM_JOINTS];
    for k in 1..=n {
        let (s, s_dot) = prof.sample(k as f64 * dt);
        let mut q = [0.0; NUM_JOINTS];
        let mut qd = [0.0; NUM_JOINTS];
        path.sample(s, &mut q);
        path.derivative(s, &mut dq_ds);
        for j in 0..NUM_JOINTS {
            qd[j] = dq_ds[j] * s_dot;
        }
        qs.push(q);
        qds.push(qd);
    }
    // Land exactly on the segment target, at rest.
    let last = n - 1;
    path.sample(1.0, &mut qs[last]);
    qds[last] = [0.0; NUM_JOINTS];
    let entry_ticks = ((prof.t_in / dt).floor() as usize).min(n);
    let exit_ticks = (((prof.v / prof.a_out) / dt).floor() as usize).min(n);
    SegSamples {
        q: qs,
        qd: qds,
        entry_ticks,
        exit_ticks,
    }
}
