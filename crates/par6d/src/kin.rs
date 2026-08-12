//! Kin-backed hooks and cartesian math (feature `ffi`).
//!
//! One [`par6_kin::Kin`] instance per consumer (the underlying
//! `pinocchio::Data` is mutated by every call, so instances are never
//! shared across threads): the RT thread owns two (FK hook + gravity
//! hook), the planner one, the bridge one, housekeeping one. All are
//! loaded once at startup from the assets tree; a load failure is a
//! clean [`DaemonError`](crate::DaemonError), never a panic.
//!
//! Pose conventions on this boundary:
//!
//! - Wire poses are `[x, y, z (mm), rx, ry, rz (deg)]` with
//!   `R = Rz(rz)·Ry(ry)·Rx(rx)` (URDF fixed-axis rpy) — the convention
//!   the Python client documents and the server's STATUS matrix
//!   reconstruction assumes.
//! - The RT snapshot's `tcp` is `[x y z (m), roll pitch yaw (rad)]` in
//!   the SAME convention, extracted here from the FK matrix (NOT via
//!   `Kin::tcp`, whose rpy is the intrinsic-XYZ pinokin convention and
//!   would round-trip to a wrong STATUS matrix).

use std::path::{Path, PathBuf};

use par6_kin::{GripperVariant, IkOptions, IkOutcome, Kin, Pose, NQ};
use par6_rt::{ForwardKin, GravityModel, MAX_JOINTS};

/// Map a configured gripper name onto the URDF variant whose tool the
/// model must carry (mass for gravity, TCP frame for FK/IK). Unknown
/// names fall back to the bare flange — the arm itself is always
/// modeled — with a startup warning.
pub(crate) fn variant_for(gripper_name: &str) -> GripperVariant {
    if gripper_name.eq_ignore_ascii_case("flange") {
        GripperVariant::Flange
    } else if gripper_name.starts_with("MSG") {
        GripperVariant::Msg
    } else if gripper_name.starts_with("SSG48") {
        GripperVariant::Ssg48
    } else {
        log::warn!("no URDF variant for gripper '{gripper_name}'; using the bare flange model");
        GripperVariant::Flange
    }
}

/// Resolve the `assets/par6_description` tree: the explicit choice when
/// given, else the tree sitting next to the config directory (the repo
/// layout: `config/PAR6.toml` ↔ `assets/par6_description`).
pub(crate) fn resolve_assets_dir(
    explicit: Option<&Path>,
    config_path: &Path,
) -> Result<PathBuf, String> {
    if let Some(p) = explicit {
        return if p.is_dir() {
            Ok(p.to_path_buf())
        } else {
            Err(format!("assets directory not found: {}", p.display()))
        };
    }
    let candidate = config_path
        .parent()
        .and_then(Path::parent)
        .map(|root| root.join("assets/par6_description"));
    match candidate {
        Some(c) if c.is_dir() => Ok(c),
        _ => Err(format!(
            "no assets/par6_description tree next to {}; set --assets or PAR6_ASSETS",
            config_path.display()
        )),
    }
}

/// Load one model instance, mapping the error to a one-line message.
pub(crate) fn load_kin(assets_dir: &Path, variant: GripperVariant) -> Result<Kin, String> {
    Kin::load(assets_dir, variant).map_err(|e| {
        format!(
            "cannot load kinematics model {} from {}: {e}",
            variant.urdf_relpath(),
            assets_dir.display()
        )
    })
}

// ------------------------------------------------------------- pose math

/// Rotation-angle threshold below which two orientations count as equal
/// (slerp degenerates) \[rad\].
const ANGLE_EPS: f64 = 1e-9;

/// `[x y z (m), roll pitch yaw (rad)]` from a row-major 4x4, rpy in the
/// URDF fixed-axis convention `R = Rz·Ry·Rx` (gimbal lock folds yaw
/// into roll, matching the Python client's decode).
pub(crate) fn matrix_to_xyzrpy(m: &Pose) -> [f64; 6] {
    let (r00, r10) = (m[0], m[4]);
    let (r11, r12) = (m[5], m[6]);
    let (r20, r21, r22) = (m[8], m[9], m[10]);
    let sy = r00.hypot(r10);
    let (roll, yaw) = if sy > 1e-9 {
        (r21.atan2(r22), r10.atan2(r00))
    } else {
        ((-r12).atan2(r11), 0.0)
    };
    let pitch = (-r20).atan2(sy);
    [m[3], m[7], m[11], roll, pitch, yaw]
}

/// Row-major 4x4 from a wire pose `[x y z (mm), rx ry rz (deg)]`,
/// `R = Rz·Ry·Rx`.
pub(crate) fn wire_pose_to_matrix(pose_mm_deg: &[f64; 6]) -> Pose {
    let (x, y, z) = (
        pose_mm_deg[0] / 1000.0,
        pose_mm_deg[1] / 1000.0,
        pose_mm_deg[2] / 1000.0,
    );
    let (sr, cr) = pose_mm_deg[3].to_radians().sin_cos();
    let (sp, cp) = pose_mm_deg[4].to_radians().sin_cos();
    let (sy, cy) = pose_mm_deg[5].to_radians().sin_cos();
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

/// Unit quaternion `[w, x, y, z]` from the rotation block of `m`
/// (Shepperd's method: pick the largest diagonal pivot for stability).
fn matrix_to_quat(m: &Pose) -> [f64; 4] {
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

/// A straight cartesian segment: position lerp + orientation slerp.
pub(crate) struct CartSegment {
    p0: [f64; 3],
    p1: [f64; 3],
    q0: [f64; 4],
    q1: [f64; 4],
}

impl CartSegment {
    pub(crate) fn new(start: &Pose, end: &Pose) -> Self {
        Self {
            p0: [start[3], start[7], start[11]],
            p1: [end[3], end[7], end[11]],
            q0: matrix_to_quat(start),
            q1: matrix_to_quat(end),
        }
    }

    /// Translation length \[m\].
    pub(crate) fn length_m(&self) -> f64 {
        let d = [
            self.p1[0] - self.p0[0],
            self.p1[1] - self.p0[1],
            self.p1[2] - self.p0[2],
        ];
        (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
    }

    /// Rotation angle between the endpoint orientations \[rad\].
    pub(crate) fn angle_rad(&self) -> f64 {
        quat_angle(&self.q0, &self.q1)
    }

    /// Pose at normalized arc position `t` in \[0, 1\].
    pub(crate) fn sample(&self, t: f64) -> Pose {
        let mut m = [0.0; 16];
        m[15] = 1.0;
        quat_to_rotation(&quat_slerp(&self.q0, &self.q1, t), &mut m);
        m[3] = self.p0[0] + t * (self.p1[0] - self.p0[0]);
        m[7] = self.p0[1] + t * (self.p1[1] - self.p0[1]);
        m[11] = self.p0[2] + t * (self.p1[2] - self.p0[2]);
        m
    }
}

/// 4x4 product `a · b` (homogeneous transforms).
pub(crate) fn mat_mul(a: &Pose, b: &Pose) -> Pose {
    let mut out = [0.0; 16];
    for r in 0..4 {
        for c in 0..4 {
            let mut s = 0.0;
            for k in 0..4 {
                s += a[r * 4 + k] * b[k * 4 + c];
            }
            out[r * 4 + c] = s;
        }
    }
    out
}

// -------------------------------------------------------------- RT hooks

/// TCP FK behind the RT [`ForwardKin`] seam: full FK matrix, rpy
/// extracted in the wire convention so the server's STATUS matrix
/// reconstruction reproduces the true pose. NaN on any model failure —
/// never a fabricated pose.
pub(crate) struct KinFk {
    kin: Kin,
    scratch: Pose,
}

impl KinFk {
    pub(crate) fn new(kin: Kin) -> Self {
        Self {
            kin,
            scratch: [0.0; 16],
        }
    }
}

impl ForwardKin for KinFk {
    fn tcp(&mut self, q: &[f64; MAX_JOINTS], out: &mut [f64; 6]) {
        match self.kin.fk(q, &mut self.scratch) {
            Ok(()) => *out = matrix_to_xyzrpy(&self.scratch),
            Err(_) => out.fill(f64::NAN),
        }
    }
}

/// G(q) behind the RT [`GravityModel`] seam. A model failure must not
/// kill the RT thread: hold the last good value (gravity is a
/// feedforward, not a safety path).
pub(crate) struct KinGravity {
    kin: Kin,
    scratch: [f64; MAX_JOINTS],
    last_good: [f64; MAX_JOINTS],
}

impl KinGravity {
    pub(crate) fn new(kin: Kin) -> Self {
        Self {
            kin,
            scratch: [0.0; MAX_JOINTS],
            last_good: [0.0; MAX_JOINTS],
        }
    }
}

impl GravityModel for KinGravity {
    fn gravity(&mut self, q: &[f64; MAX_JOINTS], out: &mut [f64; MAX_JOINTS]) {
        if self.kin.gravity(q, &mut self.scratch).is_ok() {
            self.last_good = self.scratch;
        }
        *out = self.last_good;
    }
}

// -------------------------------------------------------- solver wrapper

/// Outcome of a seeded IK call through [`CartKin`].
pub(crate) enum IkResult {
    /// Converged; the solution.
    Solved([f64; NQ]),
    /// No solution within the iteration budget.
    Unreachable,
    /// The shim rejected the call.
    Failed(String),
}

/// A [`Kin`] plus the small solver surface the planner, bridge and
/// housekeeping use (FK matrix, seeded IK, damped-least-squares
/// jacobian velocity solve).
pub(crate) struct CartKin {
    kin: Kin,
    jac: [f64; 6 * NQ],
}

/// DLS damping λ for the jacobian velocity solve (`Jᵀ(JJᵀ+λ²I)⁻¹v`).
const DLS_LAMBDA: f64 = 0.05;

impl CartKin {
    pub(crate) fn new(kin: Kin) -> Self {
        Self {
            kin,
            jac: [0.0; 6 * NQ],
        }
    }

    /// FK matrix at `q`, or a one-line error.
    pub(crate) fn fk(&mut self, q: &[f64; NQ]) -> Result<Pose, String> {
        let mut pose = [0.0; 16];
        self.kin
            .fk(q, &mut pose)
            .map_err(|e| format!("FK failed: {e}"))?;
        Ok(pose)
    }

    /// Seeded damped-least-squares IK toward `target`.
    pub(crate) fn ik(&mut self, seed: &[f64; NQ], target: &Pose) -> IkResult {
        let mut out = [0.0; NQ];
        match self.kin.ik(seed, target, &mut out, IkOptions::default()) {
            Ok(IkOutcome::Converged) => IkResult::Solved(out),
            Ok(IkOutcome::MaxIters) => IkResult::Unreachable,
            Err(e) => IkResult::Failed(e.to_string()),
        }
    }

    /// Joint velocities tracking the world-frame TCP twist `v`
    /// (`[vx vy vz (m/s), wx wy wz (rad/s)]`, LOCAL_WORLD_ALIGNED) at
    /// configuration `q`, via damped least squares — bounded near
    /// singularities instead of exploding.
    pub(crate) fn twist_to_qd(&mut self, q: &[f64; NQ], v: &[f64; 6]) -> Result<[f64; NQ], String> {
        self.kin
            .jacobian(q, &mut self.jac)
            .map_err(|e| format!("jacobian failed: {e}"))?;
        let j = &self.jac;
        // A = J·Jᵀ + λ²I (6x6 symmetric), then qd = Jᵀ·A⁻¹·v.
        let mut a = [[0.0f64; 6]; 6];
        for r in 0..6 {
            for c in 0..6 {
                let mut s = 0.0;
                for k in 0..NQ {
                    s += j[r * NQ + k] * j[c * NQ + k];
                }
                a[r][c] = s;
            }
            a[r][r] += DLS_LAMBDA * DLS_LAMBDA;
        }
        let y = solve6(&mut a, v).ok_or_else(|| "singular jacobian system".to_string())?;
        let mut qd = [0.0; NQ];
        for (k, out) in qd.iter_mut().enumerate() {
            let mut s = 0.0;
            for r in 0..6 {
                s += j[r * NQ + k] * y[r];
            }
            *out = s;
        }
        Ok(qd)
    }
}

/// Solve the 6x6 system `A·x = b` in place (partial-pivot Gaussian
/// elimination). `None` when a pivot vanishes (the DLS damping makes
/// that unreachable for real jacobians; this is pure defense).
fn solve6(a: &mut [[f64; 6]; 6], b: &[f64; 6]) -> Option<[f64; 6]> {
    let mut x = *b;
    for col in 0..6 {
        let pivot = (col..6).max_by(|&r1, &r2| a[r1][col].abs().total_cmp(&a[r2][col].abs()))?;
        if a[pivot][col].abs() < 1e-12 {
            return None;
        }
        a.swap(col, pivot);
        x.swap(col, pivot);
        let (upper, lower) = a.split_at_mut(col + 1);
        let pivot_row = &upper[col];
        for (r, row) in lower.iter_mut().enumerate() {
            let f = row[col] / pivot_row[col];
            for (dst, src) in row.iter_mut().zip(pivot_row.iter()).skip(col) {
                *dst -= f * *src;
            }
            x[col + 1 + r] -= f * x[col];
        }
    }
    for col in (0..6).rev() {
        x[col] /= a[col][col];
        for row in 0..col {
            x[row] -= a[row][col] * x[col];
            a[row][col] = 0.0;
        }
    }
    Some(x)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wire_pose_round_trips_through_matrix() {
        let poses = [
            [120.0, -45.0, 300.0, 10.0, -20.0, 130.0],
            [0.0, 0.0, 0.0, -170.0, 45.0, -5.0],
            [5.0, 5.0, 5.0, 0.0, 0.0, 0.0],
        ];
        for p in poses {
            let m = wire_pose_to_matrix(&p);
            let back = matrix_to_xyzrpy(&m);
            let again = wire_pose_to_matrix(&[
                back[0] * 1000.0,
                back[1] * 1000.0,
                back[2] * 1000.0,
                back[3].to_degrees(),
                back[4].to_degrees(),
                back[5].to_degrees(),
            ]);
            for (i, (a, b)) in m.iter().zip(again.iter()).enumerate() {
                assert!((a - b).abs() < 1e-9, "pose {p:?} elem {i}: {a} vs {b}");
            }
        }
    }

    #[test]
    fn cart_segment_interpolates_endpoints_and_midpoint_rotation() {
        let start = wire_pose_to_matrix(&[100.0, 0.0, 200.0, 0.0, 0.0, 0.0]);
        let end = wire_pose_to_matrix(&[200.0, 50.0, 200.0, 0.0, 0.0, 90.0]);
        let seg = CartSegment::new(&start, &end);
        assert!((seg.length_m() - (0.1f64.powi(2) + 0.05f64.powi(2)).sqrt()).abs() < 1e-12);
        assert!((seg.angle_rad() - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
        let mid = seg.sample(0.5);
        let rpy = matrix_to_xyzrpy(&mid);
        assert!((rpy[0] - 0.15).abs() < 1e-12, "midpoint x");
        assert!((rpy[5].to_degrees() - 45.0).abs() < 1e-9, "midpoint yaw");
        for (g, w) in seg.sample(1.0).iter().zip(end.iter()) {
            assert!((g - w).abs() < 1e-9);
        }
    }

    #[test]
    fn solve6_inverts_a_known_system() {
        // A = diag(2) with an off-diagonal coupling; b chosen so x is exact.
        let mut a = [[0.0; 6]; 6];
        for (i, row) in a.iter_mut().enumerate() {
            row[i] = 2.0;
        }
        a[0][5] = 1.0;
        let x_true = [1.0, -2.0, 3.0, 0.5, -0.25, 4.0];
        let mut b = [0.0; 6];
        for r in 0..6 {
            for c in 0..6 {
                b[r] += a[r][c] * x_true[c];
            }
        }
        let x = solve6(&mut a, &b).expect("solvable");
        for (g, w) in x.iter().zip(x_true.iter()) {
            assert!((g - w).abs() < 1e-12);
        }
    }
}
