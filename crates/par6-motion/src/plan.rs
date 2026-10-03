//! Planned-move trajectory generation: a queued program of joint-space
//! moves compiled into a tick-rate [`Sample`] stream for the EXEC ring.
//!
//! Four profiles ([`ProfileKind`]):
//!
//! - **Trapezoid**: accel–cruise–decel run on the normalized path
//!   coordinate `s`, which synchronizes all joints on the slowest one
//!   (the binding joint sets the scalar velocity/acceleration budget).
//! - **Ruckig**: jerk-limited point-to-point via rsruckig.
//! - **Quintic** and **Septic**: one polynomial on the path coordinate,
//!   point-to-point. The quintic starts and stops at rest in velocity and
//!   acceleration; the septic in jerk too, and holds the jerk limit.
//!
//! Every move is point-to-point; corners are rounded upstream, on the
//! path, by the planner. Sample metadata carries the ring contract:
//! `command_index` per queued move, `checkpoint_id` boundaries, `is_last`
//! on the final sample of the program.

use rsruckig::prelude::*;

use crate::path::{JointLinePath, PathSampler};
use crate::{MotionError, MotionLimits, Sample, SampleMeta, NUM_JOINTS};

/// Displacements below this count as "joint does not move" \[rad\].
const ZERO_DELTA: f64 = 1e-12;

/// Profile registry for the moves THIS crate compiles. TOPPRA is not
/// here: it re-times a finished path through the C++ shim and is driven
/// by `par6d`'s planner (see the crate docs).
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
    /// Quintic polynomial on the path coordinate: velocity AND
    /// acceleration are zero at both ends, so the move starts and stops
    /// without a step in either. No cruise phase and no jerk limiting —
    /// peak jerk is `60/T³` over a unit distance, bounded by nothing but
    /// the duration.
    Quintic,
    /// Septic polynomial on the path coordinate: velocity, acceleration
    /// AND jerk are zero at both ends, so unlike the quintic there is no
    /// jerk step when the move starts or stops. Peak jerk, `52.5/T³` over
    /// a unit distance, is held under the jerk limit where one is set.
    Septic,
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
        let mut start = self.start;
        for (i, mv) in self.moves.iter().enumerate() {
            let meta = SampleMeta {
                command_index: i as u32,
                checkpoint_id: mv.params.checkpoint_id.unwrap_or(i as u32),
                is_last: false,
            };
            let seg = match mv.params.profile {
                ProfileKind::Trapezoid => self.trapezoid(&start, mv),
                ProfileKind::Ruckig => self.ruckig(&start, mv)?,
                ProfileKind::Quintic | ProfileKind::Septic => self.polynomial(&start, mv),
            };
            samples.extend((0..seg.q.len()).map(|t| Sample {
                q: seg.q[t],
                qd: seg.qd[t],
                qdd: seg.qdd[t],
                meta,
            }));
            start = mv.target;
        }
        if let Some(last) = samples.last_mut() {
            last.meta.is_last = true;
        }
        Ok(Plan {
            samples,
            dt: self.dt,
        })
    }

    fn trapezoid(&self, start: &[f64; NUM_JOINTS], mv: &MoveSpec) -> SegSamples {
        let path = JointLinePath::new(*start, mv.target);
        trapezoid_segment(
            &path,
            &joint_spans(start, &mv.target),
            &self.limits,
            mv.params.speed_fraction,
            mv.params.min_duration_s,
            self.dt,
        )
    }

    fn polynomial(&self, start: &[f64; NUM_JOINTS], mv: &MoveSpec) -> SegSamples {
        let path = JointLinePath::new(*start, mv.target);
        polynomial_segment(
            &path,
            &joint_spans(start, &mv.target),
            &self.limits,
            mv.params.speed_fraction,
            mv.params.min_duration_s,
            mv.params.profile == ProfileKind::Septic,
            self.dt,
        )
    }

    fn ruckig(&self, start: &[f64; NUM_JOINTS], mv: &MoveSpec) -> Result<SegSamples, MotionError> {
        self.limits.require_finite_jerk()?;
        let mut otg = Ruckig::<NUM_JOINTS, ThrowErrorHandler>::new(None, self.dt);
        let mut input = InputParameter::<NUM_JOINTS>::new(None);
        let mut output = OutputParameter::<NUM_JOINTS>::new(None);
        for (j, &q0) in start.iter().enumerate() {
            input.current_position[j] = q0;
            input.target_position[j] = mv.target[j];
            input.max_velocity[j] = self.limits.velocity[j] * mv.params.speed_fraction;
            input.max_acceleration[j] = self.limits.acceleration[j];
            input.max_jerk[j] = self.limits.jerk[j];
        }
        input.minimum_duration = mv.params.min_duration_s;

        let mut seg = SegSamples {
            q: Vec::new(),
            qd: Vec::new(),
            qdd: Vec::new(),
        };
        let mut cap: Option<usize> = None;
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
            let mut qdd = [0.0; NUM_JOINTS];
            for j in 0..NUM_JOINTS {
                q[j] = output.new_position[j];
                qd[j] = output.new_velocity[j];
                qdd[j] = output.new_acceleration[j];
            }
            seg.q.push(q);
            seg.qd.push(qd);
            seg.qdd.push(qdd);
            if finished {
                return Ok(seg);
            }
            if seg.q.len() >= cap.unwrap_or(usize::MAX) {
                return Err(MotionError::Ruckig(
                    "trajectory sampling ran past its computed duration".into(),
                ));
            }
            output.pass_to_input(&mut input);
        }
    }
}

/// Each joint's travel \[rad\], the path coordinate's per-joint scale.
fn joint_spans(start: &[f64; NUM_JOINTS], target: &[f64; NUM_JOINTS]) -> [f64; NUM_JOINTS] {
    std::array::from_fn(|j| (target[j] - start[j]).abs())
}

struct SegSamples {
    q: Vec<[f64; NUM_JOINTS]>,
    qd: Vec<[f64; NUM_JOINTS]>,
    qdd: Vec<[f64; NUM_JOINTS]>,
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

    /// `(s, ds/dt, d²s/dt²)` at time `t`, clamped to the profile ends.
    pub fn sample(&self, t: f64) -> (f64, f64, f64) {
        if t <= 0.0 {
            return (0.0, 0.0, 0.0);
        }
        if t >= self.t_total {
            return (1.0, 0.0, 0.0);
        }
        if t < self.t_in {
            (0.5 * self.a_in * t * t, self.a_in * t, self.a_in)
        } else if t < self.t_in + self.t_cruise {
            let d_in = self.v * self.v / (2.0 * self.a_in);
            (d_in + self.v * (t - self.t_in), self.v, 0.0)
        } else {
            let tt = self.t_total - t;
            (
                1.0 - 0.5 * self.a_out * tt * tt,
                self.a_out * tt,
                -self.a_out,
            )
        }
    }
}

/// Peak `ds/dt` of the unit quintic `10τ³ − 15τ⁴ + 6τ⁵`, at `τ = 1/2`:
/// exactly `15/8`.
const QUINTIC_PEAK_VEL: f64 = 1.875;
/// Peak `d²s/dt²` of the unit quintic, at `τ = (3 ∓ √3)/6`: exactly
/// `10/√3`. Written out rather than as `5.77` — the truncation lands
/// the acceleration-bound duration 0.03% short, which is small, and
/// wrong in the direction that matters.
const QUINTIC_PEAK_ACC: f64 = 5.773_502_691_896_258;

/// Scalar quintic over a unit distance: `s(τ) = 10τ³ − 15τ⁴ + 6τ⁵`
/// with `τ = t/T`. Velocity and acceleration are zero at both ends;
/// there is no cruise, so the whole move is one smooth swell.
struct SQuintic {
    t_total: f64,
}

impl SQuintic {
    /// The shortest `T` that keeps the peak velocity under `v_max` and
    /// the peak acceleration under `a_max`, stretched to `min_duration`
    /// when that is longer, and never under `floor`.
    fn new(v_max: f64, a_max: f64, min_duration: Option<f64>, floor: f64) -> Self {
        let t_v = QUINTIC_PEAK_VEL / v_max;
        let t_a = (QUINTIC_PEAK_ACC / a_max).sqrt();
        let mut t = t_v.max(t_a).max(floor);
        if let Some(td) = min_duration {
            t = t.max(td);
        }
        Self { t_total: t }
    }

    /// `(s, ds/dt, d²s/dt²)` at time `t`, clamped to the profile ends.
    fn sample(&self, t: f64) -> (f64, f64, f64) {
        if t <= 0.0 {
            return (0.0, 0.0, 0.0);
        }
        if t >= self.t_total {
            return (1.0, 0.0, 0.0);
        }
        let tt = self.t_total;
        let u = t / tt;
        let w = 1.0 - u;
        let s = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
        let s_dot = 30.0 * u * u * w * w / tt;
        let s_ddot = 60.0 * u * w * (1.0 - 2.0 * u) / (tt * tt);
        (s, s_dot, s_ddot)
    }
}

/// Peak `ds/dt` of the unit septic `35τ⁴ − 84τ⁵ + 70τ⁶ − 20τ⁷`, at
/// `τ = 1/2`: exactly `35/16`.
pub const SEPTIC_PEAK_VEL: f64 = 2.1875;
/// Peak `d²s/dt²` of the unit septic, at `τ = (5 ∓ √5)/10`: exactly
/// `84√5/25`.
pub const SEPTIC_PEAK_ACC: f64 = 7.513_188_404_399_293;
/// Peak `|d³s/dt³|` of the unit septic, at `τ = 1/2`: exactly `105/2`.
pub const SEPTIC_PEAK_JERK: f64 = 52.5;

/// Scalar septic over a unit distance: `s(τ) = 35τ⁴ − 84τ⁵ + 70τ⁶ − 20τ⁷`
/// with `τ = t/T`.
///
/// Velocity, acceleration AND jerk are zero at both ends. The quintic
/// stops at acceleration: its jerk is `60/T³` at `τ = 0` and `τ = 1`,
/// stepping there from zero the instant a move starts and again the
/// instant it stops, and a hand on the arm reads that step as a jolt
/// however small the move's acceleration is. Here jerk ramps in and out,
/// and its peak — `52.5/T³`, mid-move — is lower than the quintic's.
///
/// A scalar time-scaling, so a multi-joint move sampled through it stays
/// on the straight joint-space line between its endpoints; a jerk-limited
/// time-synchronized profile does not, and a path checked for collision
/// as a straight line has to be driven as one.
#[derive(Debug, Clone, Copy)]
pub struct SSeptic {
    t_total: f64,
}

impl SSeptic {
    /// The shortest `T` that keeps the peak velocity under `v_max`, the
    /// peak acceleration under `a_max` and the peak jerk under `j_max` —
    /// all per unit distance, `INFINITY` meaning unconstrained — stretched
    /// to `min_duration` when that is longer, and never under `floor`.
    pub fn new(v_max: f64, a_max: f64, j_max: f64, min_duration: Option<f64>, floor: f64) -> Self {
        let t_v = SEPTIC_PEAK_VEL / v_max;
        let t_a = (SEPTIC_PEAK_ACC / a_max).sqrt();
        let t_j = (SEPTIC_PEAK_JERK / j_max).cbrt();
        let mut t = t_v.max(t_a).max(t_j).max(floor);
        if let Some(td) = min_duration {
            t = t.max(td);
        }
        Self { t_total: t }
    }

    /// Total profile duration \[s\].
    pub fn duration(&self) -> f64 {
        self.t_total
    }

    /// `(s, ds/dt, d²s/dt²)` at time `t`, clamped to the profile ends.
    pub fn sample(&self, t: f64) -> (f64, f64, f64) {
        if t <= 0.0 {
            return (0.0, 0.0, 0.0);
        }
        if t >= self.t_total {
            return (1.0, 0.0, 0.0);
        }
        let tt = self.t_total;
        let u = t / tt;
        let w = 1.0 - u;
        let u2 = u * u;
        let s = u2 * u2 * (35.0 - 84.0 * u + 70.0 * u2 - 20.0 * u2 * u);
        let s_dot = 140.0 * u2 * u * w * w * w / tt;
        let s_ddot = 420.0 * u2 * w * w * (1.0 - 2.0 * u) / (tt * tt);
        (s, s_dot, s_ddot)
    }
}

/// `t ↦ (s, ds/dt, d²s/dt²)` of a unit time scaling.
type UnitSampler = Box<dyn Fn(f64) -> (f64, f64, f64)>;

/// One polynomial move along `path` — the septic when `septic`, else the
/// quintic — scaled per joint exactly as [`trapezoid_segment`] scales the
/// trapezoid: the scalar caps are the tightest `limit_j / scale_j` across
/// the joints that move, so one duration serves every joint and they all
/// start and stop together.
///
/// That reduction makes the duration `max_j` of the per-joint closed
/// forms — for the quintic `1.875·Δ_j / v_j` and `√(10/√3 · Δ_j / a_j)`,
/// the two peaks of the unit profile each scaled by that joint's
/// displacement; the septic adds its jerk peak `∛(52.5 · Δ_j / j_j)`.
#[allow(clippy::too_many_arguments)]
fn polynomial_segment(
    path: &dyn PathSampler,
    scale: &[f64; NUM_JOINTS],
    limits: &MotionLimits,
    speed_fraction: f64,
    min_duration_s: Option<f64>,
    septic: bool,
    dt: f64,
) -> SegSamples {
    let mut v_s = f64::INFINITY;
    let mut a_s = f64::INFINITY;
    let mut j_s = f64::INFINITY;
    for (j, &sc) in scale.iter().enumerate() {
        if sc > ZERO_DELTA {
            v_s = v_s.min(limits.velocity[j] * speed_fraction / sc);
            a_s = a_s.min(limits.acceleration[j] / sc);
            j_s = j_s.min(limits.jerk[j] / sc);
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
            qdd: vec![[0.0; NUM_JOINTS]],
        };
    }
    let (t_total, sample): (f64, UnitSampler) = if septic {
        let prof = SSeptic::new(v_s, a_s, j_s, min_duration_s, 2.0 * dt);
        (prof.duration(), Box::new(move |t| prof.sample(t)))
    } else {
        let prof = SQuintic::new(v_s, a_s, min_duration_s, 2.0 * dt);
        (prof.t_total, Box::new(move |t| prof.sample(t)))
    };
    let n = ((t_total / dt).ceil() as usize).max(1);
    let mut qs = Vec::with_capacity(n);
    let mut qds = Vec::with_capacity(n);
    let mut qdds = Vec::with_capacity(n);
    let mut dq_ds = [0.0; NUM_JOINTS];
    for k in 1..=n {
        let (s, s_dot, s_ddot) = sample(k as f64 * dt);
        let mut q = [0.0; NUM_JOINTS];
        let mut qd = [0.0; NUM_JOINTS];
        let mut qdd = [0.0; NUM_JOINTS];
        path.sample(s, &mut q);
        path.derivative(s, &mut dq_ds);
        for j in 0..NUM_JOINTS {
            qd[j] = dq_ds[j] * s_dot;
            // Straight joint line: q(s) is affine, so no curvature term.
            qdd[j] = dq_ds[j] * s_ddot;
        }
        qs.push(q);
        qds.push(qd);
        qdds.push(qdd);
    }
    // Land exactly on the segment target, at rest.
    let last = n - 1;
    path.sample(1.0, &mut qs[last]);
    qds[last] = [0.0; NUM_JOINTS];
    qdds[last] = [0.0; NUM_JOINTS];
    SegSamples {
        q: qs,
        qd: qds,
        qdd: qdds,
    }
}

#[allow(clippy::too_many_arguments)]
fn trapezoid_segment(
    path: &dyn PathSampler,
    scale: &[f64; NUM_JOINTS],
    limits: &MotionLimits,
    speed_fraction: f64,
    min_duration_s: Option<f64>,
    dt: f64,
) -> SegSamples {
    let mut v_s = f64::INFINITY;
    let mut a_s = f64::INFINITY;
    for (j, &sc) in scale.iter().enumerate() {
        if sc > ZERO_DELTA {
            v_s = v_s.min(limits.velocity[j] * speed_fraction / sc);
            a_s = a_s.min(limits.acceleration[j] / sc);
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
            qdd: vec![[0.0; NUM_JOINTS]],
        };
    }
    let prof = STrapezoid::new(v_s, a_s, a_s, min_duration_s);
    let n = ((prof.t_total / dt).ceil() as usize).max(1);
    let mut qs = Vec::with_capacity(n);
    let mut qds = Vec::with_capacity(n);
    let mut qdds = Vec::with_capacity(n);
    let mut dq_ds = [0.0; NUM_JOINTS];
    for k in 1..=n {
        let (s, s_dot, s_ddot) = prof.sample(k as f64 * dt);
        let mut q = [0.0; NUM_JOINTS];
        let mut qd = [0.0; NUM_JOINTS];
        let mut qdd = [0.0; NUM_JOINTS];
        path.sample(s, &mut q);
        path.derivative(s, &mut dq_ds);
        for j in 0..NUM_JOINTS {
            qd[j] = dq_ds[j] * s_dot;
            // q(s) is affine in s for every path this profile times —
            // a joint line, and the arc-length cartesian path, which is
            // piecewise affine for exactly this reason — so the chain
            // rule's curvature term dq²/ds²·ṡ² is zero.
            qdd[j] = dq_ds[j] * s_ddot;
        }
        qs.push(q);
        qds.push(qd);
        qdds.push(qdd);
    }
    // Land exactly on the segment target, at rest.
    let last = n - 1;
    path.sample(1.0, &mut qs[last]);
    qds[last] = [0.0; NUM_JOINTS];
    qdds[last] = [0.0; NUM_JOINTS];
    SegSamples {
        q: qs,
        qd: qds,
        qdd: qdds,
    }
}
