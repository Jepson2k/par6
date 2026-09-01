//! Cartesian path geometry: the shapes a queued move traces through
//! space, before any of it becomes joint waypoints.
//!
//! Everything here is pure geometry over row-major 4x4 poses in metres —
//! no kinematics, no timing, no FFI. The planner turns a [`Pose`] list
//! into joint waypoints with seeded IK and hands those to TOPPRA, so
//! each generator's only job is to place poses ON the intended shape at
//! a density fine enough for the IK chain to stay on one branch.
//!
//! Position and orientation are treated separately, as parol6 does:
//! position follows the shape (line, circle, spline, Bézier corner),
//! orientation is a shortest-arc quaternion slerp along it. That avoids
//! the gimbal-lock artifacts of interpolating rpy triples.
//!
//! Corner rounding ([`corner_trims`]) is shared by the cartesian and the
//! joint-space blend paths: it is the ABB zone rule — a corner radius is
//! clamped to half of each adjacent segment, and two adjacent zones that
//! would overlap are scaled down together until they do not.

use crate::MotionError;

/// Row-major 4x4 homogeneous transform; translation in metres.
pub type Pose = [f64; 16];

/// Rotation-angle threshold below which two orientations count as equal
/// (slerp degenerates) \[rad\].
const ANGLE_EPS: f64 = 1e-9;

/// Below this a length is zero for geometry purposes \[m\].
const LEN_EPS: f64 = 1e-9;

/// How close the end point must be to the start point for `move_c` to
/// mean "sweep the whole circle" rather than "sweep to the end" \[m\].
/// One millimetre: FK/IK round-trips land within ~0.1 mm, so a client
/// that passes its own start pose back as the end lands inside this,
/// and a real arc that ends 1 mm from its start is indistinguishable
/// from a full circle anyway. Ported from parol6's threshold
/// (`motion/geometry.py`, `compute_circle_from_3_points`).
const FULL_CIRCLE_M: f64 = 1e-3;

// ------------------------------------------------------------ quaternions

/// Unit quaternion `[w, x, y, z]` from the rotation block of `m`
/// (Shepperd's method: pick the largest diagonal pivot for stability).
fn quat_from_matrix(m: &Pose) -> [f64; 4] {
    let (r00, r01, r02) = (m[0], m[1], m[2]);
    let (r10, r11, r12) = (m[4], m[5], m[6]);
    let (r20, r21, r22) = (m[8], m[9], m[10]);
    let trace = r00 + r11 + r22;
    let q = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        [s / 4.0, (r21 - r12) / s, (r02 - r20) / s, (r10 - r01) / s]
    } else if r00 >= r11 && r00 >= r22 {
        let s = (1.0 + r00 - r11 - r22).sqrt() * 2.0;
        [(r21 - r12) / s, s / 4.0, (r01 + r10) / s, (r02 + r20) / s]
    } else if r11 >= r22 {
        let s = (1.0 + r11 - r00 - r22).sqrt() * 2.0;
        [(r02 - r20) / s, (r01 + r10) / s, s / 4.0, (r12 + r21) / s]
    } else {
        let s = (1.0 + r22 - r00 - r11).sqrt() * 2.0;
        [(r10 - r01) / s, (r02 + r20) / s, (r12 + r21) / s, s / 4.0]
    };
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
}

/// Write the rotation block of `m` from a unit quaternion.
fn quat_to_rotation(q: &[f64; 4], m: &mut Pose) {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    m[0] = 1.0 - 2.0 * (y * y + z * z);
    m[1] = 2.0 * (x * y - w * z);
    m[2] = 2.0 * (x * z + w * y);
    m[4] = 2.0 * (x * y + w * z);
    m[5] = 1.0 - 2.0 * (x * x + z * z);
    m[6] = 2.0 * (y * z - w * x);
    m[8] = 2.0 * (x * z - w * y);
    m[9] = 2.0 * (y * z + w * x);
    m[10] = 1.0 - 2.0 * (x * x + y * y);
}

/// Angle of the relative rotation between two unit quaternions \[rad\].
fn quat_angle(a: &[f64; 4], b: &[f64; 4]) -> f64 {
    let dot = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]).abs();
    2.0 * dot.clamp(-1.0, 1.0).acos()
}

/// Shortest-arc slerp between unit quaternions.
fn quat_slerp(a: &[f64; 4], b: &[f64; 4], t: f64) -> [f64; 4] {
    let mut b = *b;
    let mut dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    if dot < 0.0 {
        for v in &mut b {
            *v = -*v;
        }
        dot = -dot;
    }
    let dot = dot.clamp(-1.0, 1.0);
    let theta = dot.acos();
    if theta < ANGLE_EPS {
        // Nearly parallel: nlerp is exact to first order.
        let mut out = [0.0; 4];
        for i in 0..4 {
            out[i] = a[i] + t * (b[i] - a[i]);
        }
        let n = out.iter().map(|v| v * v).sum::<f64>().sqrt();
        for v in &mut out {
            *v /= n;
        }
        return out;
    }
    let sin_theta = theta.sin();
    let (wa, wb) = (
        ((1.0 - t) * theta).sin() / sin_theta,
        (t * theta).sin() / sin_theta,
    );
    [
        wa * a[0] + wb * b[0],
        wa * a[1] + wb * b[1],
        wa * a[2] + wb * b[2],
        wa * a[3] + wb * b[3],
    ]
}

// ------------------------------------------------------------ vector math

/// The translation of a pose \[m\].
pub fn translation(m: &Pose) -> [f64; 3] {
    [m[3], m[7], m[11]]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn scale(a: [f64; 3], k: f64) -> [f64; 3] {
    [a[0] * k, a[1] * k, a[2] * k]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

/// Rotate `v` about the unit axis `k` by `angle` (Rodrigues).
fn rotate_about(v: [f64; 3], k: [f64; 3], angle: f64) -> [f64; 3] {
    let (s, c) = angle.sin_cos();
    add(
        add(scale(v, c), scale(cross(k, v), s)),
        scale(k, dot(k, v) * (1.0 - c)),
    )
}

/// A pose from an orientation quaternion and a translation \[m\].
fn pose_of(q: &[f64; 4], p: [f64; 3]) -> Pose {
    let mut m = [0.0; 16];
    m[15] = 1.0;
    quat_to_rotation(q, &mut m);
    m[3] = p[0];
    m[7] = p[1];
    m[11] = p[2];
    m
}

// --------------------------------------------------------------- sampling

/// How finely a cartesian shape is turned into IK waypoints: one
/// waypoint per `step_m` of translation or `step_rad` of rotation,
/// whichever asks for more, with `max_points` bounding the whole path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CartSampling {
    /// Pitch \[m\] against the chosen metric.
    pub step_m: f64,
    /// How rotation enters the metric.
    pub rotation: RotationPitch,
    /// Ceiling on the waypoints of one path (bounds planning cost).
    pub max_points: usize,
}

/// How a piece's rotation contributes to its sample count.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RotationPitch {
    /// Rotation counts on its own pitch \[rad\]; the piece takes the
    /// larger of the translation and rotation counts (the MOVE_L form).
    Independent(f64),
    /// Rotation folds into the length metric as `√(t² + (w·θ)²)` with
    /// weight `w` \[m/rad\] — the vendor's multi-segment path form.
    Weighted(f64),
}

impl CartSampling {
    /// Intervals a piece of `len_m` translation and `angle_rad` rotation
    /// wants, at least one.
    fn intervals(&self, len_m: f64, angle_rad: f64) -> usize {
        let n = match self.rotation {
            RotationPitch::Independent(step_rad) => {
                let by_len = (len_m / self.step_m).ceil() as usize;
                let by_ang = (angle_rad / step_rad).ceil() as usize;
                by_len.max(by_ang)
            }
            RotationPitch::Weighted(w) => {
                (len_m.hypot(w * angle_rad) / self.step_m).ceil() as usize
            }
        };
        n.max(1)
    }
}

/// Scale a per-piece interval count down so the whole path fits under
/// `max_points`, leaving every piece at least one interval.
fn fit_budget(counts: &mut [usize], max_points: usize) {
    let total: usize = counts.iter().sum();
    let budget = max_points.max(counts.len() + 1);
    if total < budget {
        return;
    }
    let factor = (budget - 1) as f64 / total as f64;
    for c in counts.iter_mut() {
        *c = ((*c as f64 * factor).round() as usize).max(1);
    }
}

// ------------------------------------------------------------------- line

/// A straight cartesian segment: position lerp, orientation slerp.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LineSegment {
    p0: [f64; 3],
    p1: [f64; 3],
    q0: [f64; 4],
    q1: [f64; 4],
}

impl LineSegment {
    /// The segment between two poses.
    pub fn new(start: &Pose, end: &Pose) -> Self {
        Self {
            p0: translation(start),
            p1: translation(end),
            q0: quat_from_matrix(start),
            q1: quat_from_matrix(end),
        }
    }

    /// Translation length \[m\].
    pub fn length_m(&self) -> f64 {
        norm(sub(self.p1, self.p0))
    }

    /// Rotation angle between the endpoint orientations \[rad\].
    pub fn angle_rad(&self) -> f64 {
        quat_angle(&self.q0, &self.q1)
    }

    /// Pose at normalized position `t` in \[0, 1\].
    pub fn sample(&self, t: f64) -> Pose {
        pose_of(
            &quat_slerp(&self.q0, &self.q1, t),
            add(self.p0, scale(sub(self.p1, self.p0), t)),
        )
    }
}

/// Waypoints along a straight segment, `start` first and `end` last.
pub fn line(start: &Pose, end: &Pose, s: CartSampling) -> Vec<Pose> {
    let seg = LineSegment::new(start, end);
    let n = s
        .intervals(seg.length_m(), seg.angle_rad())
        .min(s.max_points.saturating_sub(1).max(1));
    (0..=n).map(|k| seg.sample(k as f64 / n as f64)).collect()
}

// -------------------------------------------------------------------- arc

/// A circle in 3-D: centre, radius \[m\] and unit plane normal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Circle {
    /// Centre point \[m\].
    pub center: [f64; 3],
    /// Radius \[m\].
    pub radius: f64,
    /// Unit normal of the plane the circle lies in.
    pub normal: [f64; 3],
    /// The end point came back to the start, so the client meant one whole
    /// lap. [`arc`] takes the sweep from this rather than re-deriving it
    /// from the endpoints: the two differ by the arm's settle error, which
    /// subtends a fraction of a degree and would otherwise replace the
    /// commanded circle with a nudge of that size.
    pub full_circle: bool,
}

/// The circle through three points, as `move_c` derives it from
/// start / via / end.
///
/// When the end point coincides with the start (`|p3 - p1| <`
/// [`FULL_CIRCLE_M`]) the client means a FULL circle, and two points do
/// not determine a plane: the circle is the one with `p1`–`p2` as its
/// diameter, in the plane picked the way parol6 picks it (normal =
/// `d × ref`, `ref` = z unless `d` is nearly parallel to it). The via
/// point is then diametrically opposite the start, which is exactly how
/// a client draws a full circle.
///
/// Collinear or coincident points have no circle and are refused — never
/// silently straightened into a line.
pub fn circle_through(p1: [f64; 3], p2: [f64; 3], p3: [f64; 3]) -> Result<Circle, MotionError> {
    let a = sub(p2, p1);
    let b = sub(p3, p1);
    if norm(b) < FULL_CIRCLE_M {
        let a_len = norm(a);
        if a_len < LEN_EPS {
            return Err(MotionError::InvalidInput {
                what: "via",
                reason: "the start, via and end points of an arc are all the same point".into(),
            });
        }
        let d = scale(a, 1.0 / a_len);
        let reference = if d[2].abs() < 0.9 {
            [0.0, 0.0, 1.0]
        } else {
            [1.0, 0.0, 0.0]
        };
        let n = cross(d, reference);
        return Ok(Circle {
            center: add(p1, scale(a, 0.5)),
            radius: a_len / 2.0,
            normal: scale(n, 1.0 / norm(n)),
            full_circle: true,
        });
    }
    let n = cross(a, b);
    let n_len = norm(n);
    if n_len < LEN_EPS * LEN_EPS {
        return Err(MotionError::InvalidInput {
            what: "via",
            reason: "the start, via and end points of an arc are collinear; \
                     they define no circle"
                .into(),
        });
    }
    // Circumcentre C = p1 + s·a + t·b from the perpendicular bisectors:
    // (C-p1)·a = |a|²/2 and (C-p1)·b = |b|²/2.
    let (aa, bb, ab) = (dot(a, a), dot(b, b), dot(a, b));
    let det = aa * bb - ab * ab;
    if det.abs() < LEN_EPS * LEN_EPS {
        return Err(MotionError::InvalidInput {
            what: "via",
            reason: "the start, via and end points of an arc are degenerate; \
                     no circle centre exists"
                .into(),
        });
    }
    let s = (bb * aa - ab * bb) / (2.0 * det);
    let t = (aa * bb - ab * aa) / (2.0 * det);
    let center = add(p1, add(scale(a, s), scale(b, t)));
    Ok(Circle {
        center,
        radius: norm(sub(center, p1)),
        normal: scale(n, 1.0 / n_len),
        full_circle: false,
    })
}

/// Waypoints along the circular arc from `start` through `via` to `end`.
///
/// The sweep direction is the one that passes through the via point: the
/// plane normal is oriented by `(via-start) × (end-start)`, so the
/// counter-clockwise sweep about it is the one containing the via — and
/// when the short way round does NOT contain it, the complement
/// (`2π - θ`) is taken instead. A [`Circle::full_circle`] sweeps the lap
/// the client asked for instead, whatever the endpoints subtend.
///
/// Orientation slerps from the start pose to the end pose across the
/// whole sweep; `end`'s own orientation is what the arm finishes in.
pub fn arc(
    start: &Pose,
    via: &Pose,
    end: &Pose,
    s: CartSampling,
) -> Result<Vec<Pose>, MotionError> {
    let (p_start, p_via, p_end) = (translation(start), translation(via), translation(end));
    let circle = circle_through(p_start, p_via, p_end)?;
    let r1 = sub(p_start, circle.center);
    let r2 = sub(p_end, circle.center);
    let (n1, n2) = (norm(r1), norm(r2));
    if n1 < LEN_EPS || n2 < LEN_EPS {
        return Err(MotionError::InvalidInput {
            what: "via",
            reason: "the arc has no radius".into(),
        });
    }
    let (u1, u2) = (scale(r1, 1.0 / n1), scale(r2, 1.0 / n2));
    let mut sweep = dot(u1, u2).clamp(-1.0, 1.0).acos();
    if circle.full_circle {
        sweep = std::f64::consts::TAU;
    } else if dot(cross(u1, u2), circle.normal) < 0.0 {
        sweep = std::f64::consts::TAU - sweep;
    }

    let (q0, q1) = (quat_from_matrix(start), quat_from_matrix(end));
    let arc_len = circle.radius * sweep;
    let n = s
        .intervals(arc_len, quat_angle(&q0, &q1))
        .min(s.max_points.saturating_sub(1).max(1));
    Ok((0..=n)
        .map(|k| {
            let t = k as f64 / n as f64;
            pose_of(
                &quat_slerp(&q0, &q1, t),
                add(circle.center, rotate_about(r1, circle.normal, t * sweep)),
            )
        })
        .collect())
}

// ----------------------------------------------------------------- spline

/// Waypoints along a cubic spline through `waypoints` (the first is the
/// start pose, and every one of them is passed through).
///
/// Position is a natural cubic spline per axis over chord-length knots;
/// orientation is a piecewise slerp on the same knots.
///
/// Two deliberate divergences from parol6's spline
/// (`motion/geometry.py`, `SplineMotion.generate_spline`), which builds
/// scipy `CubicSpline`s over UNIFORM knots with `not-a-knot` end
/// conditions:
///
/// - **Chord-length knots.** Uniform knots make the spline overshoot
///   between unevenly spaced waypoints (the curve has to travel a long
///   segment and a short one in equal parameter time); chord length is
///   the standard fix and is identical to parol6's choice when the
///   waypoints are evenly spaced, which is what a client generating a
///   curve produces.
/// - **Natural end conditions** (zero curvature at both ends) rather
///   than not-a-knot: the arm starts and ends this path at rest, so the
///   end curvature is not carrying information, and a natural spline
///   cannot swing wide of the first and last segments the way not-a-knot
///   can.
pub fn spline(waypoints: &[Pose], s: CartSampling) -> Result<Vec<Pose>, MotionError> {
    let n = waypoints.len();
    if n < 2 {
        return Err(MotionError::InvalidInput {
            what: "waypoints",
            reason: format!("a spline needs at least 2 waypoints, got {n}"),
        });
    }
    if n == 2 {
        return Ok(line(&waypoints[0], &waypoints[1], s));
    }
    let points: Vec<[f64; 3]> = waypoints.iter().map(translation).collect();
    let quats: Vec<[f64; 4]> = waypoints.iter().map(quat_from_matrix).collect();

    // Chord-length knots; coincident neighbours would give a zero
    // interval and a singular system, so they carry a floor.
    let mut knots = Vec::with_capacity(n);
    knots.push(0.0);
    for i in 1..n {
        let d = norm(sub(points[i], points[i - 1])).max(LEN_EPS);
        knots.push(knots[i - 1] + d);
    }
    let total = knots[n - 1];
    if total < LEN_EPS {
        return Err(MotionError::InvalidInput {
            what: "waypoints",
            reason: "every spline waypoint is the same point".into(),
        });
    }

    let coeffs: Vec<[Vec<f64>; 2]> = (0..3)
        .map(|axis| {
            let values: Vec<f64> = points.iter().map(|p| p[axis]).collect();
            let second = natural_spline_second_derivatives(&knots, &values);
            [values, second]
        })
        .collect();

    // Sample density from the polyline length and the total rotation:
    // the spline is longer than its polyline, never shorter, so this is
    // a floor on the density, and the budget bounds the ceiling.
    let turn: f64 = (1..n).map(|i| quat_angle(&quats[i - 1], &quats[i])).sum();
    let steps = s.intervals(total, turn).min(s.max_points.max(2) - 1);

    let mut out = Vec::with_capacity(steps + 1);
    let mut seg = 0usize;
    for k in 0..=steps {
        let u = total * k as f64 / steps as f64;
        while seg + 2 < n && u > knots[seg + 1] {
            seg += 1;
        }
        let (h, local) = (knots[seg + 1] - knots[seg], u - knots[seg]);
        let t = (local / h).clamp(0.0, 1.0);
        let mut p = [0.0; 3];
        for (axis, c) in coeffs.iter().enumerate() {
            let (y, m) = (&c[0], &c[1]);
            // Cubic on [knots[seg], knots[seg+1]] from the end values and
            // second derivatives (the standard interpolating form).
            let a = local;
            let b = h - local;
            p[axis] = (b * y[seg] + a * y[seg + 1]) / h
                + ((b * b * b - h * h * b) * m[seg] + (a * a * a - h * h * a) * m[seg + 1])
                    / (6.0 * h);
        }
        out.push(pose_of(&quat_slerp(&quats[seg], &quats[seg + 1], t), p));
    }
    Ok(out)
}

/// Second derivatives of the natural cubic spline through `(x, y)`
/// (Thomas algorithm on the tridiagonal moment system).
fn natural_spline_second_derivatives(x: &[f64], y: &[f64]) -> Vec<f64> {
    let n = x.len();
    let mut m = vec![0.0; n];
    if n < 3 {
        return m;
    }
    let mut c = vec![0.0; n];
    let mut d = vec![0.0; n];
    for i in 1..n - 1 {
        let (h0, h1) = (x[i] - x[i - 1], x[i + 1] - x[i]);
        let a = h0;
        let b = 2.0 * (h0 + h1);
        let cc = h1;
        let rhs = 6.0 * ((y[i + 1] - y[i]) / h1 - (y[i] - y[i - 1]) / h0);
        let denom = b - a * c[i - 1];
        c[i] = cc / denom;
        d[i] = (rhs - a * d[i - 1]) / denom;
    }
    for i in (1..n - 1).rev() {
        m[i] = d[i] - c[i] * m[i + 1];
    }
    m
}

// ------------------------------------------------------------- blend zone

/// The fraction of a segment consumed by a corner zone at each of its
/// ends, after zone clamping.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Trim {
    /// Fraction eaten at the start of the segment by the previous corner.
    pub entry: f64,
    /// Fraction eaten at the end of the segment by the next corner.
    pub exit: f64,
}

/// Clamp corner radii against the segments they round and convert them
/// to per-segment trim fractions.
///
/// `seg_lengths` has one entry per segment, `radii` one per INTERIOR
/// waypoint (`seg_lengths.len() - 1` of them). The ABB zone rule, ported
/// from parol6 (`motion/geometry.py`,
/// `build_composite_cartesian_path`): a radius never eats more than half
/// of either adjacent segment, and two zones sharing a segment are
/// scaled down together until they fit inside it.
///
/// Returns the per-segment trims and the clamped radii.
pub fn corner_trims(
    seg_lengths: &[f64],
    radii: &[f64],
) -> Result<(Vec<Trim>, Vec<f64>), MotionError> {
    if seg_lengths.is_empty() || radii.len() + 1 != seg_lengths.len() {
        return Err(MotionError::InvalidInput {
            what: "blend radii",
            reason: format!(
                "{} segments take {} corner radii, got {}",
                seg_lengths.len(),
                seg_lengths.len().saturating_sub(1),
                radii.len()
            ),
        });
    }
    let mut clamped: Vec<f64> = radii
        .iter()
        .enumerate()
        .map(|(i, r)| {
            r.max(0.0)
                .min(seg_lengths[i] / 2.0)
                .min(seg_lengths[i + 1] / 2.0)
        })
        .collect();
    for i in 0..clamped.len().saturating_sub(1) {
        let total = clamped[i] + clamped[i + 1];
        let len = seg_lengths[i + 1];
        if total > len && total > 0.0 {
            let factor = len / total;
            clamped[i] *= factor;
            clamped[i + 1] *= factor;
        }
    }
    let mut trims = vec![Trim::default(); seg_lengths.len()];
    for (i, r) in clamped.iter().enumerate() {
        if *r <= 0.0 {
            continue;
        }
        if seg_lengths[i] > LEN_EPS {
            trims[i].exit = r / seg_lengths[i];
        }
        if seg_lengths[i + 1] > LEN_EPS {
            trims[i + 1].entry = r / seg_lengths[i + 1];
        }
    }
    Ok((trims, clamped))
}

/// Waypoints along a polyline whose interior corners are rounded by
/// quadratic Bézier zones of the given radii \[m\].
///
/// `waypoints[0]` is the start pose; `radii` has one entry per interior
/// waypoint, and `0` there means "stop at this corner" (the path still
/// passes exactly through it). Each rounded corner is tangent to the
/// incoming segment where the zone starts and to the outgoing one where
/// it ends, so position is C1 across the corner: the arm never has to
/// come to rest to change direction.
pub fn blended_polyline(
    waypoints: &[Pose],
    radii: &[f64],
    s: CartSampling,
) -> Result<Vec<Pose>, MotionError> {
    let n = waypoints.len();
    if n < 2 {
        return Err(MotionError::InvalidInput {
            what: "waypoints",
            reason: format!("a path needs at least 2 waypoints, got {n}"),
        });
    }
    if n == 2 {
        return Ok(line(&waypoints[0], &waypoints[1], s));
    }
    let segments: Vec<LineSegment> = (0..n - 1)
        .map(|i| LineSegment::new(&waypoints[i], &waypoints[i + 1]))
        .collect();
    let lengths: Vec<f64> = segments.iter().map(LineSegment::length_m).collect();
    let (trims, clamped) = corner_trims(&lengths, radii)?;

    // Two passes: size every piece first so the density budget is spread
    // over the whole path, then emit.
    enum Piece {
        /// Straight run of segment `i` from `a` to `b` in its own
        /// normalized coordinate.
        Line { i: usize, a: f64, b: f64 },
        /// Bézier corner rounding waypoint `i + 1`.
        Corner { i: usize },
    }
    let mut pieces = Vec::with_capacity(2 * n);
    let mut counts = Vec::with_capacity(2 * n);
    for i in 0..n - 1 {
        let (a, b) = (trims[i].entry, 1.0 - trims[i].exit);
        if b > a + 1e-12 {
            let seg = &segments[i];
            counts.push(s.intervals((b - a) * lengths[i], (b - a) * seg.angle_rad()));
            pieces.push(Piece::Line { i, a, b });
        }
        if i + 1 < n - 1 && clamped[i] > 0.0 {
            // The corner's control polygon is 2r long; its arc is shorter.
            let entry = segments[i].sample(1.0 - trims[i].exit);
            let exit = segments[i + 1].sample(trims[i + 1].entry);
            counts.push(s.intervals(
                2.0 * clamped[i],
                LineSegment::new(&entry, &exit).angle_rad(),
            ));
            pieces.push(Piece::Corner { i });
        }
    }
    if pieces.is_empty() {
        return Err(MotionError::InvalidInput {
            what: "waypoints",
            reason: "the path has no length".into(),
        });
    }
    fit_budget(&mut counts, s.max_points);

    let mut out: Vec<Pose> = Vec::with_capacity(counts.iter().sum::<usize>() + 1);
    for (piece, steps) in pieces.iter().zip(counts.iter()) {
        match piece {
            Piece::Line { i, a, b } => {
                let seg = &segments[*i];
                for k in 0..=*steps {
                    let t = a + (b - a) * k as f64 / *steps as f64;
                    push_distinct(&mut out, seg.sample(t));
                }
            }
            Piece::Corner { i } => {
                let entry = segments[*i].sample(1.0 - trims[*i].exit);
                let exit = segments[*i + 1].sample(trims[*i + 1].entry);
                let corner = translation(&waypoints[*i + 1]);
                let (pe, px) = (translation(&entry), translation(&exit));
                let (qe, qx) = (quat_from_matrix(&entry), quat_from_matrix(&exit));
                for k in 0..=*steps {
                    let t = k as f64 / *steps as f64;
                    let omt = 1.0 - t;
                    let p = add(
                        add(scale(pe, omt * omt), scale(corner, 2.0 * omt * t)),
                        scale(px, t * t),
                    );
                    push_distinct(&mut out, pose_of(&quat_slerp(&qe, &qx, t), p));
                }
            }
        }
    }
    Ok(out)
}

/// Append `pose` unless it repeats the previous one (piece junctions are
/// shared points; a duplicate waypoint is a zero-length path step).
fn push_distinct(out: &mut Vec<Pose>, pose: Pose) {
    if let Some(last) = out.last() {
        if norm(sub(translation(&pose), translation(last))) < LEN_EPS
            && quat_angle(&quat_from_matrix(&pose), &quat_from_matrix(last)) < ANGLE_EPS
        {
            return;
        }
    }
    out.push(pose);
}

// -------------------------------------------------------- joint-space blend

/// Joint-space counterpart of [`blended_polyline`]: a polyline through
/// joint waypoints whose interior corners are rounded by quadratic
/// Bézier zones.
///
/// `fracs` carries, per interior waypoint, the fraction of the incoming
/// and of the outgoing segment the zone consumes — the caller converts a
/// millimetre corner radius into those fractions with FK, since a joint
/// segment has no length in millimetres of its own (parol6 does the same
/// in `commands/joint_commands.py`, `do_setup_with_blend`).
///
/// `step_rad` is the joint-space sampling pitch: one waypoint per that
/// much motion on the fastest-moving joint.
pub fn blended_polyline_joint<const N: usize>(
    waypoints: &[[f64; N]],
    fracs: &[(f64, f64)],
    step_rad: f64,
    max_points: usize,
) -> Result<Vec<[f64; N]>, MotionError> {
    let n = waypoints.len();
    if n < 2 {
        return Err(MotionError::InvalidInput {
            what: "waypoints",
            reason: format!("a path needs at least 2 waypoints, got {n}"),
        });
    }
    if fracs.len() + 2 != n {
        return Err(MotionError::InvalidInput {
            what: "blend radii",
            reason: format!(
                "{n} waypoints take {} corner zones, got {}",
                n - 2,
                fracs.len()
            ),
        });
    }
    let lerp = |a: &[f64; N], b: &[f64; N], t: f64| -> [f64; N] {
        let mut out = [0.0; N];
        for j in 0..N {
            out[j] = a[j] + t * (b[j] - a[j]);
        }
        out
    };
    let span = |a: &[f64; N], b: &[f64; N]| -> f64 {
        (0..N).fold(0.0f64, |m, j| m.max((b[j] - a[j]).abs()))
    };

    let mut exit = vec![0.0; n - 1];
    let mut entry = vec![0.0; n - 1];
    for (i, (before, after)) in fracs.iter().enumerate() {
        exit[i] = before.clamp(0.0, 0.5);
        entry[i + 1] = after.clamp(0.0, 0.5);
    }

    enum Piece {
        Line { i: usize, a: f64, b: f64 },
        Corner { i: usize },
    }
    let mut pieces = Vec::with_capacity(2 * n);
    let mut counts: Vec<usize> = Vec::with_capacity(2 * n);
    let interval = |motion: f64| ((motion / step_rad).ceil() as usize).max(1);
    for i in 0..n - 1 {
        let (a, b) = (entry[i], 1.0 - exit[i]);
        if b > a + 1e-12 {
            counts.push(interval((b - a) * span(&waypoints[i], &waypoints[i + 1])));
            pieces.push(Piece::Line { i, a, b });
        }
        // Either trim on its own leaves a gap the corner has to fill: the
        // caller sizes the two from independent TCP distances, so a corner
        // whose incoming (or outgoing) segment does not move the TCP —
        // a wrist roll, a repeated target — arrives with one of them
        // zeroed. The Bézier degenerates to the corner itself on the
        // zeroed end, which is exactly the piece the trim removed.
        // parol6 guards on both the same way (`motion/geometry.py`,
        // `build_composite_joint_path`).
        if i + 1 < n - 1 && (exit[i] > 0.0 || entry[i + 1] > 0.0) {
            let e = lerp(&waypoints[i], &waypoints[i + 1], 1.0 - exit[i]);
            let x = lerp(&waypoints[i + 1], &waypoints[i + 2], entry[i + 1]);
            counts.push(interval(
                span(&e, &waypoints[i + 1]) + span(&waypoints[i + 1], &x),
            ));
            pieces.push(Piece::Corner { i });
        }
    }
    if pieces.is_empty() {
        return Err(MotionError::InvalidInput {
            what: "waypoints",
            reason: "the path has no length".into(),
        });
    }
    fit_budget(&mut counts, max_points);

    let mut out: Vec<[f64; N]> = Vec::with_capacity(counts.iter().sum::<usize>() + 1);
    let mut push = |q: [f64; N]| {
        if out.last().is_some_and(|last| span(last, &q) < 1e-12) {
            return;
        }
        out.push(q);
    };
    for (piece, steps) in pieces.iter().zip(counts.iter()) {
        match piece {
            Piece::Line { i, a, b } => {
                for k in 0..=*steps {
                    let t = a + (b - a) * k as f64 / *steps as f64;
                    push(lerp(&waypoints[*i], &waypoints[*i + 1], t));
                }
            }
            Piece::Corner { i } => {
                let e = lerp(&waypoints[*i], &waypoints[*i + 1], 1.0 - exit[*i]);
                let x = lerp(&waypoints[*i + 1], &waypoints[*i + 2], entry[*i + 1]);
                let w = &waypoints[*i + 1];
                for k in 0..=*steps {
                    let t = k as f64 / *steps as f64;
                    let omt = 1.0 - t;
                    let mut q = [0.0; N];
                    for j in 0..N {
                        q[j] = omt * omt * e[j] + 2.0 * omt * t * w[j] + t * t * x[j];
                    }
                    push(q);
                }
            }
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Pose from a translation \[m\] and rpy \[rad\] (`R = Rz·Ry·Rx`).
    fn pose(x: f64, y: f64, z: f64, roll: f64, pitch: f64, yaw: f64) -> Pose {
        let (sr, cr) = roll.sin_cos();
        let (sp, cp) = pitch.sin_cos();
        let (sy, cy) = yaw.sin_cos();
        [
            cy * cp,
            cy * sp * sr - sy * cr,
            cy * sp * cr + sy * sr,
            x,
            sy * cp,
            sy * sp * sr + cy * cr,
            sy * sp * cr - cy * sr,
            y,
            -sp,
            cp * sr,
            cp * cr,
            z,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
    }

    fn sampling() -> CartSampling {
        CartSampling {
            step_m: 0.005,
            rotation: RotationPitch::Independent(0.05),
            max_points: 4000,
        }
    }

    /// Closest approach of a path to a point \[m\], measured against the
    /// path itself (its sample-to-sample segments), not just its
    /// samples — a path passes through a point even when no sample
    /// lands exactly on it.
    fn closest(path: &[Pose], p: [f64; 3]) -> f64 {
        path.windows(2)
            .map(|w| {
                let (a, b) = (translation(&w[0]), translation(&w[1]));
                let d = sub(b, a);
                let len2 = dot(d, d);
                let t = if len2 > 0.0 {
                    (dot(sub(p, a), d) / len2).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                norm(sub(p, add(a, scale(d, t))))
            })
            .fold(f64::INFINITY, f64::min)
    }

    #[test]
    fn arc_lies_on_its_circle_and_passes_through_the_via() {
        // Quarter circle of radius 0.2 m in the XY plane about the
        // origin, via at 45°.
        let r = 0.2;
        let at = |deg: f64| {
            let a: f64 = deg.to_radians();
            pose(r * a.cos(), r * a.sin(), 0.3, 0.0, 0.0, 0.0)
        };
        let path = arc(&at(0.0), &at(45.0), &at(90.0), sampling()).expect("arc");
        assert!(path.len() > 20, "arc sampled too coarsely: {}", path.len());
        for m in &path {
            let p = translation(m);
            assert!(
                (norm([p[0], p[1], 0.0]) - r).abs() < 1e-9,
                "point {p:?} is off the circle"
            );
            assert!((p[2] - 0.3).abs() < 1e-9, "point {p:?} left the arc plane");
        }
        assert!(
            closest(&path, translation(&at(45.0))) < 1e-3,
            "missed the via"
        );
        // The end pose is reached exactly, and the sweep took the short
        // way (nothing beyond the quarter turn).
        let end = translation(path.last().expect("non-empty"));
        assert!(norm(sub(end, translation(&at(90.0)))) < 1e-9);
        assert!(
            path.iter()
                .all(|m| translation(m)[0] >= -1e-9 && translation(m)[1] >= -1e-9),
            "the arc swept the long way round"
        );
    }

    #[test]
    fn arc_through_a_far_via_takes_the_long_way_and_a_repeated_start_closes_the_circle() {
        let r = 0.15;
        let at = |deg: f64| {
            let a: f64 = deg.to_radians();
            pose(r * a.cos(), 0.4, 0.25 + r * a.sin(), 0.0, 0.0, 0.0)
        };
        // Via at 270°: the way from 0° to 90° through it is the LONG way.
        let long = arc(&at(0.0), &at(270.0), &at(90.0), sampling()).expect("arc");
        assert!(
            closest(&long, translation(&at(270.0))) < 1e-3,
            "missed the via"
        );
        assert!(
            closest(&long, translation(&at(180.0))) < 1e-3,
            "not the long way"
        );

        // end == start: the whole circle, through the diametrically
        // opposite via.
        let full = arc(&at(0.0), &at(180.0), &at(0.0), sampling()).expect("full circle");
        for deg in [0.0, 90.0, 180.0, 270.0] {
            assert!(
                closest(&full, translation(&at(deg))) < 2e-3,
                "the full circle missed {deg}°"
            );
        }
        let end = translation(full.last().expect("non-empty"));
        assert!(
            norm(sub(end, translation(&at(0.0)))) < 1e-9,
            "the circle did not close"
        );
    }

    #[test]
    fn a_full_circle_survives_an_end_that_missed_the_start() {
        // move_c's end pose comes from FK of the MEASURED joints, so it
        // lands a settle error away from the start the client asked to
        // come back to — either side of it. Both must still be one lap.
        let r = 0.05;
        let at = |deg: f64| {
            let a: f64 = deg.to_radians();
            pose(r * a.cos(), 0.4, 0.25 + r * a.sin(), 0.0, 0.0, 0.0)
        };
        let start = at(0.0);
        let circumference = std::f64::consts::TAU * r;
        for miss in [0.0, 3e-4, -3e-4] {
            let mut end = start;
            end[11] += miss;
            let path = arc(&start, &at(180.0), &end, sampling()).expect("full circle");
            let length: f64 = path
                .windows(2)
                .map(|w| norm(sub(translation(&w[1]), translation(&w[0]))))
                .sum();
            assert!(
                (length - circumference).abs() < 0.01 * circumference,
                "a {:.1} mm miss swept {length} m of a {circumference} m circle",
                miss * 1000.0
            );
            for deg in [90.0, 180.0, 270.0] {
                assert!(
                    closest(&path, translation(&at(deg))) < 2e-3,
                    "the circle missed {deg}° after a {:.1} mm miss",
                    miss * 1000.0
                );
            }
        }
    }

    #[test]
    fn collinear_arc_points_are_refused() {
        let a = pose(0.1, 0.2, 0.3, 0.0, 0.0, 0.0);
        let b = pose(0.2, 0.2, 0.3, 0.0, 0.0, 0.0);
        let c = pose(0.4, 0.2, 0.3, 0.0, 0.0, 0.0);
        let e = arc(&a, &b, &c, sampling()).expect_err("collinear points have no arc");
        assert!(
            matches!(e, MotionError::InvalidInput { what: "via", .. }),
            "unexpected error: {e}"
        );
    }

    #[test]
    fn spline_passes_through_every_waypoint_and_curves_between_them() {
        let wps: Vec<Pose> = [
            (0.0, 0.35, 0.20),
            (0.05, 0.35, 0.28),
            (0.10, 0.35, 0.20),
            (0.15, 0.35, 0.28),
        ]
        .iter()
        .map(|(x, y, z)| pose(*x, *y, *z, 0.0, 0.0, 0.0))
        .collect();
        let path = spline(&wps, sampling()).expect("spline");
        for w in &wps {
            assert!(
                closest(&path, translation(w)) < 1e-3,
                "spline missed waypoint {:?}",
                translation(w)
            );
        }
        // A spline is not the polyline: between the second and third
        // waypoints it bows away from the straight chord.
        let (a, b) = (translation(&wps[1]), translation(&wps[2]));
        let bow = path
            .iter()
            .map(|m| {
                let p = translation(m);
                let d = sub(b, a);
                let t = (dot(sub(p, a), d) / dot(d, d)).clamp(0.0, 1.0);
                norm(sub(p, add(a, scale(d, t))))
            })
            .fold(0.0f64, f64::max);
        assert!(bow > 2e-3, "spline did not curve: max bow {bow} m");
    }

    #[test]
    fn spline_orientation_slerps_through_the_waypoint_orientations() {
        let wps = [
            pose(0.0, 0.35, 0.2, 0.0, 0.0, 0.0),
            pose(0.1, 0.35, 0.2, 0.0, 0.0, std::f64::consts::FRAC_PI_2),
            pose(0.2, 0.35, 0.2, 0.0, 0.0, std::f64::consts::PI),
        ];
        let path = spline(&wps, sampling()).expect("spline");
        let last = quat_from_matrix(path.last().expect("non-empty"));
        assert!(quat_angle(&last, &quat_from_matrix(&wps[2])) < 1e-9);
        // Monotone turn: no wrap through the short arc backwards.
        let q0 = quat_from_matrix(&wps[0]);
        let mut prev = 0.0;
        for m in &path {
            let a = quat_angle(&q0, &quat_from_matrix(m));
            assert!(a >= prev - 1e-9, "orientation reversed: {a} after {prev}");
            prev = a;
        }
    }

    #[test]
    fn blend_rounds_the_corner_inside_its_radius_and_zero_radius_keeps_it_sharp() {
        let corner = [0.1, 0.35, 0.25];
        let wps = [
            pose(0.0, 0.35, 0.25, 0.0, 0.0, 0.0),
            pose(corner[0], corner[1], corner[2], 0.0, 0.0, 0.0),
            pose(0.1, 0.35, 0.35, 0.0, 0.0, 0.0),
        ];
        let sharp = blended_polyline(&wps, &[0.0], sampling()).expect("sharp");
        assert!(
            closest(&sharp, corner) < 1e-9,
            "r = 0 must pass through the corner"
        );

        let r = 0.02;
        let rounded = blended_polyline(&wps, &[r], sampling()).expect("rounded");
        let miss = closest(&rounded, corner);
        assert!(
            miss > 1e-3,
            "r = {r} m did not round the corner (closest {miss} m)"
        );
        assert!(
            miss < r,
            "the rounded corner strayed further than the radius: {miss} m"
        );
        // Rounding cuts the corner: the path is shorter than the polyline.
        let length: f64 = rounded
            .windows(2)
            .map(|w| norm(sub(translation(&w[1]), translation(&w[0]))))
            .sum();
        assert!(length < 0.2 - 1e-3, "rounded path length {length} m");
        // Endpoints are never blended away.
        assert!(closest(&rounded, translation(&wps[0])) < 1e-9);
        assert!(closest(&rounded, translation(&wps[2])) < 1e-9);
    }

    #[test]
    fn corner_radii_are_clamped_to_the_segments_they_round() {
        // A 100 mm and a 40 mm segment with a 90 mm requested radius:
        // half the shorter segment is the binding constraint.
        let (trims, clamped) = corner_trims(&[0.1, 0.04], &[0.09]).expect("trims");
        assert!(
            (clamped[0] - 0.02).abs() < 1e-12,
            "clamped to {}",
            clamped[0]
        );
        assert!((trims[0].exit - 0.2).abs() < 1e-12);
        assert!((trims[1].entry - 0.5).abs() < 1e-12);

        // Two zones sharing a 100 mm middle segment, 60 mm each: both
        // scale down so they meet rather than overlap.
        let (trims, clamped) = corner_trims(&[0.2, 0.1, 0.2], &[0.06, 0.06]).expect("trims");
        assert!(
            (clamped[0] - 0.05).abs() < 1e-12,
            "clamped to {}",
            clamped[0]
        );
        assert!((clamped[1] - 0.05).abs() < 1e-12);
        assert!((trims[1].entry + trims[1].exit - 1.0).abs() < 1e-12);

        let err = corner_trims(&[0.1, 0.1], &[]).expect_err("radius count is checked");
        assert!(matches!(err, MotionError::InvalidInput { .. }), "{err}");
    }

    #[test]
    fn joint_blend_rounds_the_corner_and_keeps_the_end_points() {
        let wps = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [1.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        ];
        let path = blended_polyline_joint(&wps, &[(0.25, 0.25)], 0.02, 4000).expect("path");
        let closest_to = |q: [f64; 6]| {
            path.iter()
                .map(|p| (0..6).fold(0.0f64, |m, j| m.max((p[j] - q[j]).abs())))
                .fold(f64::INFINITY, f64::min)
        };
        assert!(closest_to(wps[0]) < 1e-12, "start dropped");
        assert!(closest_to(wps[2]) < 1e-12, "end dropped");
        let miss = closest_to(wps[1]);
        assert!(miss > 0.05, "the corner was not rounded (closest {miss})");
        assert!(miss < 0.25, "the corner strayed outside its zone ({miss})");
        // The two joints hand over smoothly: joint 1 is already moving
        // while joint 0 still is.
        assert!(
            path.iter()
                .any(|q| q[0] > 0.75 && q[0] < 1.0 && q[1] > 1e-6),
            "the corner was traversed one joint at a time"
        );
    }

    #[test]
    fn a_corner_with_one_trim_zeroed_is_still_sampled() {
        // The caller sizes the two trims from independent TCP distances, so
        // a wrist-roll leg (the TCP sits on J6's axis and does not move)
        // arrives with the incoming trim zeroed and the outgoing one live.
        let step = 0.05;
        let wps = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.8],
            [0.524, 0.0, 0.0, 0.0, 0.0, 0.8],
        ];
        let path = blended_polyline_joint(&wps, &[(0.0, 0.5)], step, 4000).expect("path");
        let biggest = path
            .windows(2)
            .map(|w| (0..6).fold(0.0f64, |m, j| m.max((w[1][j] - w[0][j]).abs())))
            .fold(0.0f64, f64::max);
        // The corner's Bézier parameter is not arc length, so its own
        // samples are uneven — but never by the multiple a dropped corner
        // costs, which hands TOPPRA one interval as long as the trim.
        assert!(
            biggest < 2.0 * step,
            "the path steps {biggest} rad at a {step} rad pitch"
        );
        // The trim removed the head of the outgoing segment; something has
        // to cover it, or the arm is told nothing between the corner and
        // half way to the target.
        let head = 0.5 * wps[2][0];
        assert!(
            path.iter().any(|q| q[0] > 1e-9 && q[0] < head - 1e-9),
            "nothing was sampled inside the corner's outgoing zone"
        );
        let same = |q: &[f64; 6], w: &[f64; 6]| (0..6).all(|j| (q[j] - w[j]).abs() < 1e-12);
        assert!(
            same(path.first().expect("non-empty"), &wps[0])
                && same(path.last().expect("non-empty"), &wps[2]),
            "the chain must still start and finish on its own waypoints"
        );
    }

    #[test]
    fn line_endpoints_are_exact_and_rotation_drives_the_density() {
        let a = pose(0.1, 0.2, 0.3, 0.0, 0.0, 0.0);
        let b = pose(0.1, 0.2, 0.3, 0.0, 0.0, 1.0);
        let path = line(&a, &b, sampling());
        // No translation at all: the rotation pitch alone sizes it.
        assert_eq!(path.len(), (1.0f64 / 0.05).ceil() as usize + 1);
        for m in &path {
            assert!(norm(sub(translation(m), translation(&a))) < 1e-12);
        }
        assert!(
            quat_angle(
                &quat_from_matrix(path.last().unwrap()),
                &quat_from_matrix(&b)
            ) < 1e-9
        );
    }

    /// The multi-segment metric folds rotation into path length as
    /// √(t² + (w·θ)²): a pure twist is priced at w·θ metres, a mixed
    /// piece at the hypotenuse — never the max form the MOVE_L pitch
    /// keeps.
    #[test]
    fn the_weighted_metric_prices_rotation_as_path_length() {
        let s = CartSampling {
            step_m: 0.002,
            rotation: RotationPitch::Weighted(0.15),
            max_points: 4000,
        };
        assert_eq!(s.intervals(0.01, 0.0), 5);
        assert_eq!(s.intervals(0.0, 0.1), 8, "0.1 rad at 0.15 m/rad = 15 mm");
        assert_eq!(
            s.intervals(0.01, 0.1),
            10,
            "hypot(10, 15) mm, not max(5, 8)"
        );
        assert_eq!(s.intervals(0.0, 0.0), 1, "a degenerate piece still samples");

        let ind = CartSampling {
            step_m: 0.01,
            rotation: RotationPitch::Independent(0.034906585),
            max_points: 4000,
        };
        assert_eq!(ind.intervals(0.02, 0.0), 2);
        assert_eq!(
            ind.intervals(0.0, 0.07),
            3,
            "rotation on its own 2-degree pitch"
        );
        assert_eq!(ind.intervals(0.02, 0.07), 3, "the max of the two counts");
    }

    #[test]
    fn the_sample_budget_bounds_a_long_path() {
        let wps: Vec<Pose> = (0..50)
            .map(|i| pose(0.01 * i as f64, 0.35, 0.25, 0.0, 0.0, 0.0))
            .collect();
        let s = CartSampling {
            step_m: 0.0001,
            rotation: RotationPitch::Independent(0.05),
            max_points: 300,
        };
        let radii = vec![0.002; wps.len() - 2];
        let path = blended_polyline(&wps, &radii, s).expect("path");
        assert!(
            path.len() <= 300 && path.len() > 50,
            "budgeted path has {} points",
            path.len()
        );
    }
}
