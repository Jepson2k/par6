//! Analytic inverse kinematics on the OPW model (ortho-parallel base,
//! spherical wrist) via `rs-opw-kinematics`, with every model constant
//! derived from the URDF at load and proven against the Pinocchio FK.
//!
//! Nothing here is hand-entered or tuned. Loading reads the six joint
//! axes off the URDF, measures the OPW link lengths from them, resolves
//! the joint sign/zero conventions by fitting the closed-form FK to the
//! Pinocchio FK at pseudo-random configurations, and refuses to load an
//! arm the closed form cannot reproduce to [`FIT_TOL`] — so a URDF swap
//! either yields an exact solver or a clear error naming what broke.
//!
//! The solver works at the wrist centre (`c4 = 0`); whatever sits after
//! it — flange orientation convention, the tool, a TCP offset — is one
//! fixed frame `F` fitted from the oracle, `T_ee(q) = OPW(q) · F`. A tool
//! change is therefore a new `F`, never a new derivation.

use std::f64::consts::PI;
use std::path::Path;

use glam::{DMat3, DMat4, DQuat, DVec3, DVec4};
use rs_opw_kinematics::kinematic_traits::{Joints, Kinematics, Pose as OpwPose};
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;

use crate::kin::{Kin, Pose};
use crate::NQ;

/// Agreement required between the fitted closed form and the Pinocchio
/// FK, on every element of the 4x4 pose, at every check configuration.
/// Both sides are exact algebra on the same lengths, so real agreement is
/// ~1e-15; anything near this bound is a convention error.
pub const FIT_TOL: f64 = 1e-9;

/// Tolerance for the structural checks (parallel J2/J3, concurrent wrist
/// axes) \[m or rad\], generous against URDF float formatting.
const GEOM_TOL: f64 = 1e-6;

/// Configurations the fit and the final proof are run at.
const CHECK_POSES: usize = 24;

/// Why a URDF yields no analytic solver.
#[derive(Debug, Clone, thiserror::Error)]
pub enum OpwError {
    #[error("URDF read failed: {0}")]
    Urdf(String),
    #[error("URDF has {0} revolute joints, the OPW model needs exactly {NQ}")]
    JointCount(usize),
    #[error("not an OPW arm: {0}")]
    Geometry(String),
    #[error("Pinocchio FK failed while deriving the OPW model: {0}")]
    Oracle(String),
    #[error("no joint convention reproduces the Pinocchio FK (closest: {0:.3e} m at the wrist centre); the arm is not OPW or its URDF zero pose is not on 90° increments")]
    NoFit(f64),
    #[error(
        "fitted OPW model disagrees with the Pinocchio FK by {0:.3e} at a check configuration"
    )]
    Proof(f64),
}

/// The fitted solver for one loaded model.
#[derive(Debug)]
pub struct Opw {
    solver: OPWKinematics,
    /// Wrist-centre frame → model end-effector frame, fixed.
    flange_to_ee: DMat4,
    ee_to_flange: DMat4,
    params: Parameters,
}

impl Opw {
    /// Derive the model for `urdf`, using `kin` (loaded from the same
    /// file) as the FK oracle.
    pub fn derive(urdf: &Path, kin: &mut Kin) -> Result<Self, OpwError> {
        let axes = joint_axes(urdf)?;
        let geom = measure(&axes)?;

        // Oracle: end-effector poses at pseudo-random configurations, and
        // the wrist centre each implies (rigid in the flange frame).
        let mut qs = Vec::with_capacity(CHECK_POSES);
        let mut lcg: u64 = 0x9E37_79B9_7F4A_7C15;
        for _ in 0..CHECK_POSES {
            let mut q = [0.0; NQ];
            for v in q.iter_mut() {
                lcg = lcg
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                *v = ((lcg >> 11) as f64 / (1u64 << 53) as f64) * 2.0 * PI - PI;
            }
            qs.push(q);
        }
        let mut pose = [0.0; 16];
        let mut ee_at = |q: &[f64; NQ]| -> Result<DMat4, OpwError> {
            kin.fk(q, &mut pose)
                .map_err(|e| OpwError::Oracle(e.to_string()))?;
            Ok(mat_from_row_major(&pose))
        };
        let t0 = ee_at(&[0.0; NQ])?;
        // Wrist centre in end-effector coordinates: fixed for all q.
        let u = t0.inverse().transform_point3(geom.wrist_center);
        let oracle: Vec<(Joints, DMat4, DVec3)> = qs
            .iter()
            .map(|q| ee_at(q).map(|t| (*q, t, t.transform_point3(u))))
            .collect::<Result<_, _>>()?;

        // Stage 1: base lengths and J1..J3 conventions from wrist-centre
        // positions, which the wrist joints cannot move. Several
        // conventions tie here (an elbow split rotated 90°, a mirrored
        // sign), differing only in link orientation, so every exact one
        // goes on to stage 2 rather than the first found.
        let grid = [0.0, PI / 2.0, PI, -PI / 2.0];
        let mut closest = f64::INFINITY;
        let mut exact: Vec<Parameters> = Vec::new();
        for &(c3, a2) in &geom.elbow_candidates {
            for a1 in [geom.a1, -geom.a1] {
                for b in [geom.b, -geom.b] {
                    for sbits in 0..8u32 {
                        for obits in 0..64u32 {
                            let mut p = Parameters {
                                a1,
                                a2,
                                b,
                                c1: geom.c1,
                                c2: geom.c2,
                                c3,
                                c4: 0.0,
                                ..Default::default()
                            };
                            for i in 0..3 {
                                p.sign_corrections[i] = if sbits >> i & 1 == 1 { -1 } else { 1 };
                                p.offsets[i] = grid[(obits >> (2 * i) & 3) as usize];
                            }
                            let k = OPWKinematics::new(p);
                            let mut worst: f64 = 0.0;
                            for (q, _, w) in &oracle {
                                let t = k.forward(q).translation;
                                worst = worst.max((t - *w).length());
                                if worst > FIT_TOL {
                                    break;
                                }
                            }
                            closest = closest.min(worst);
                            if worst <= FIT_TOL {
                                exact.push(p);
                            }
                        }
                    }
                }
            }
        }
        if exact.is_empty() {
            return Err(OpwError::NoFit(closest));
        }

        // Stage 2: wrist conventions, pinned by the end-effector
        // orientation: T_ee = OPW · F must hold with one constant F.
        let mut best = (f64::INFINITY, exact[0], DMat4::IDENTITY);
        for base in &exact {
            let mut params = *base;
            for sbits in 0..8u32 {
                for obits in 0..64u32 {
                    for i in 0..3 {
                        params.sign_corrections[3 + i] = if sbits >> i & 1 == 1 { -1 } else { 1 };
                        params.offsets[3 + i] = grid[(obits >> (2 * i) & 3) as usize];
                    }
                    let k = OPWKinematics::new(params);
                    let (q0, t_ee0, _) = &oracle[0];
                    let f = mat_from_opw(&k.forward(q0)).inverse() * *t_ee0;
                    let mut worst: f64 = 0.0;
                    for (q, t_ee, _) in &oracle {
                        let t = mat_from_opw(&k.forward(q)) * f;
                        worst = worst.max(max_abs_diff(&t, t_ee));
                        if worst > best.0 {
                            break;
                        }
                    }
                    if worst < best.0 {
                        best = (worst, params, f);
                    }
                }
            }
        }
        let (rot_err, params, flange_to_ee) = best;
        if rot_err > FIT_TOL {
            return Err(OpwError::Proof(rot_err));
        }
        Ok(Opw {
            solver: OPWKinematics::new(params),
            flange_to_ee,
            ee_to_flange: flange_to_ee.inverse(),
            params,
        })
    }

    /// The fitted OPW parameters (lengths in metres, offsets in radians).
    pub fn parameters(&self) -> &Parameters {
        &self.params
    }

    /// Fixed wrist-centre → end-effector transform, row-major 4x4.
    pub fn flange_to_ee(&self) -> Pose {
        row_major(&self.flange_to_ee)
    }

    /// Closed-form solve for the end-effector pose `target` (row-major
    /// 4x4, model end-effector frame), choosing the branch nearest `seed`.
    /// `None` when the pose is out of reach or any input is non-finite.
    pub fn solve(&self, seed: &[f64; NQ], target: &Pose) -> Option<[f64; NQ]> {
        if !seed.iter().all(|v| v.is_finite()) || !target.iter().all(|v| v.is_finite()) {
            return None;
        }
        let t = mat_from_row_major(target) * self.ee_to_flange;
        let (_, rotation, translation) = t.to_scale_rotation_translation();
        let pose = OpwPose::from_parts(translation, rotation);
        self.solver
            .inverse_continuing(&pose, seed)
            .first()
            .copied()
            .filter(|q| q.iter().all(|v| v.is_finite()))
    }
}

/// One revolute joint's axis at the URDF zero pose, in the root frame.
struct Axis {
    dir: DVec3,
    point: DVec3,
}

struct Geometry {
    c1: f64,
    /// Magnitudes; the fit decides the signs.
    a1: f64,
    b: f64,
    c2: f64,
    /// `(c3, a2)` for the elbow zero poses the fit may pick between.
    elbow_candidates: Vec<(f64, f64)>,
    wrist_center: DVec3,
}

fn joint_axes(urdf: &Path) -> Result<Vec<Axis>, OpwError> {
    let robot = urdf_rs::read_file(urdf).map_err(|e| OpwError::Urdf(e.to_string()))?;
    let by_child: std::collections::HashMap<&str, &urdf_rs::Joint> = robot
        .joints
        .iter()
        .map(|j| (j.child.link.as_str(), j))
        .collect();
    // Root-to-joint transform at zero: compose every joint origin on the
    // path from the root link (all joint types sit at zero).
    let to_root = |joint: &urdf_rs::Joint| -> (DMat4, usize) {
        let mut chain = vec![joint];
        let mut link = joint.parent.link.as_str();
        while let Some(j) = by_child.get(link) {
            chain.push(j);
            link = j.parent.link.as_str();
        }
        let depth = chain.len();
        let t = chain
            .iter()
            .rev()
            .fold(DMat4::IDENTITY, |acc, j| acc * origin_transform(&j.origin));
        (t, depth)
    };
    let mut revolute: Vec<(usize, Axis)> = robot
        .joints
        .iter()
        .filter(|j| {
            matches!(
                j.joint_type,
                urdf_rs::JointType::Revolute | urdf_rs::JointType::Continuous
            )
        })
        .map(|j| {
            let (t, depth) = to_root(j);
            let a = j.axis.xyz.0;
            let dir = t
                .transform_vector3(DVec3::new(a[0], a[1], a[2]))
                .normalize();
            (
                depth,
                Axis {
                    dir,
                    point: t.transform_point3(DVec3::ZERO),
                },
            )
        })
        .collect();
    if revolute.len() != NQ {
        return Err(OpwError::JointCount(revolute.len()));
    }
    revolute.sort_by_key(|(depth, _)| *depth);
    Ok(revolute.into_iter().map(|(_, a)| a).collect())
}

fn measure(axes: &[Axis]) -> Result<Geometry, OpwError> {
    let (j1, j2, j3) = (&axes[0], &axes[1], &axes[2]);
    let skew = j1.dir.dot(j2.dir).abs();
    if skew > GEOM_TOL {
        return Err(OpwError::Geometry(format!(
            "J1 and J2 axes are not perpendicular (off by {skew:.2e} rad; rounded angles in the URDF?)"
        )));
    }
    let skew = j2.dir.cross(j3.dir).length();
    if skew > GEOM_TOL {
        return Err(OpwError::Geometry(format!(
            "J2 and J3 axes are not parallel (off by {skew:.2e} rad; rounded angles in the URDF?)"
        )));
    }
    let wrist_center = concurrent_point(&axes[3..6])?;

    let c1_raw = (j2.point - j1.point).dot(j1.dir);
    let z = if c1_raw >= 0.0 { j1.dir } else { -j1.dir };
    let c1 = c1_raw.abs();
    // Shortest vector from the J1 axis to the J2 axis, split into the
    // lateral (along J2) and forward (perpendicular) parts.
    let d = j2.point - j1.point - c1 * z;
    let b = d.dot(j2.dir).abs();
    let a1 = (d - d.dot(j2.dir) * j2.dir).length();

    let upper = j3.point - j2.point;
    let upper = upper - upper.dot(j2.dir) * j2.dir;
    let c2 = upper.length();
    if c2 < GEOM_TOL {
        return Err(OpwError::Geometry("J2 and J3 axes coincide".into()));
    }
    let e = upper / c2;
    let n = j2.dir.cross(e);
    let v = wrist_center - j3.point;
    let v = v - v.dot(j2.dir) * j2.dir;
    // The forearm's zero direction is one of the four in-plane rotations
    // of the upper arm's; each gives a (c3, a2) split, the fit picks.
    let elbow_candidates = [(e, n), (n, -e), (-e, -n), (-n, e)]
        .iter()
        .map(|(along, across)| (v.dot(*along), v.dot(*across)))
        .filter(|(c3, _)| *c3 > GEOM_TOL)
        .collect();
    Ok(Geometry {
        c1,
        a1,
        b,
        c2,
        elbow_candidates,
        wrist_center,
    })
}

/// Least-squares intersection of the wrist axes; an error unless they
/// all pass through it (spherical wrist).
fn concurrent_point(axes: &[Axis]) -> Result<DVec3, OpwError> {
    // Minimise Σ |(I - d dᵀ)(x - p)|²  →  (Σ M_i) x = Σ M_i p_i.
    let mut a = DMat3::ZERO;
    let mut rhs = DVec3::ZERO;
    for ax in axes {
        let m = DMat3::IDENTITY
            - DMat3::from_cols(ax.dir * ax.dir.x, ax.dir * ax.dir.y, ax.dir * ax.dir.z);
        a += m;
        rhs += m * ax.point;
    }
    if a.determinant().abs() < 1e-12 {
        return Err(OpwError::Geometry(
            "wrist axes are parallel, no wrist centre".into(),
        ));
    }
    let x = a.inverse() * rhs;
    for ax in axes {
        let r = x - ax.point;
        let off = (r - r.dot(ax.dir) * ax.dir).length();
        if off > GEOM_TOL {
            return Err(OpwError::Geometry(format!(
                "wrist axes do not intersect (J4/J5/J6 miss a common point by {off:.2e} m)"
            )));
        }
    }
    Ok(x)
}

fn origin_transform(o: &urdf_rs::Pose) -> DMat4 {
    let [r, p, y] = o.rpy.0;
    let rot = DQuat::from_rotation_z(y) * DQuat::from_rotation_y(p) * DQuat::from_rotation_x(r);
    let [x, yy, z] = o.xyz.0;
    DMat4::from_rotation_translation(rot, DVec3::new(x, yy, z))
}

/// `a⁻¹ · b`: the fixed transform taking frame `a` to frame `b`, both
/// row-major 4x4 poses expressed in one parent frame.
pub fn relative_pose(a: &Pose, b: &Pose) -> Pose {
    row_major(&(mat_from_row_major(a).inverse() * mat_from_row_major(b)))
}

fn mat_from_opw(p: &OpwPose) -> DMat4 {
    DMat4::from_rotation_translation(p.rotation, p.translation)
}

fn mat_from_row_major(m: &Pose) -> DMat4 {
    // Row-major input, glam is column-major.
    DMat4::from_cols(
        DVec4::new(m[0], m[4], m[8], m[12]),
        DVec4::new(m[1], m[5], m[9], m[13]),
        DVec4::new(m[2], m[6], m[10], m[14]),
        DVec4::new(m[3], m[7], m[11], m[15]),
    )
}

fn row_major(m: &DMat4) -> Pose {
    let c = m.to_cols_array_2d();
    let mut out = [0.0; 16];
    for r in 0..4 {
        for col in 0..4 {
            out[r * 4 + col] = c[col][r];
        }
    }
    out
}

fn max_abs_diff(a: &DMat4, b: &DMat4) -> f64 {
    a.to_cols_array()
        .iter()
        .zip(b.to_cols_array().iter())
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::GripperVariant;

    /// The three shipped URDF trees carry the same arm, so their measured
    /// OPW geometry must agree; a tree edited off the OPW model fails here
    /// with the axes it produced.
    #[test]
    fn every_variant_urdf_measures_as_the_same_opw_arm() {
        let assets = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/par6_description");
        let mut reference: Option<Geometry> = None;
        for variant in GripperVariant::ALL {
            let path = assets.join(variant.urdf_relpath());
            let axes = joint_axes(&path).unwrap_or_else(|e| panic!("{variant:?}: {e}"));
            let dump = || {
                axes.iter()
                    .map(|a| format!("dir={:?} point={:?}", a.dir, a.point))
                    .collect::<Vec<_>>()
                    .join("\n  ")
            };
            let geom = measure(&axes).unwrap_or_else(|e| panic!("{variant:?}: {e}\n  {}", dump()));
            if let Some(r) = &reference {
                for (label, a, b) in [
                    ("c1", r.c1, geom.c1),
                    ("a1", r.a1, geom.a1),
                    ("b", r.b, geom.b),
                    ("c2", r.c2, geom.c2),
                ] {
                    assert!(
                        (a - b).abs() < 1e-9,
                        "{variant:?} {label}: {a} vs {b}\n  {}",
                        dump()
                    );
                }
                assert!(
                    (r.wrist_center - geom.wrist_center).length() < 1e-9,
                    "{variant:?} wrist centre"
                );
            } else {
                assert!(geom.c1 > 0.0 && geom.c2 > 0.0 && !geom.elbow_candidates.is_empty());
                reference = Some(geom);
            }
        }
    }
}
