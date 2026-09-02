//! Gravity-model identification: fit each link's mass and first moment
//! to the torques the arm holds itself with, and write the result back
//! into the URDF the runtime loads.
//!
//! Statics is linear in `θ = [m_i, m_i c_i]` per body — `G(q) = Y(q) θ`
//! with `Y` the model's own gravity regressor ([`Kin::gravity_regressor`])
//! — so the fit is a regularised least squares.
//!
//! Only the FIRST MOMENTS `m_i c_i` are fitted; the masses stay at the
//! values the URDF carries. Statics pins the product far better than
//! either factor, so a fit free to move both finds a torque-equivalent
//! split that is physically wrong: on this arm it moved an upper-arm
//! centre of mass 40 cm outside the link while halving nothing. Gravity
//! would still come out right and every other consumer of the same URDF
//! (inverse dynamics, the mass matrix, the sim plant) would be corrupted.
//! Masses come from CAD and are trustworthy; where the centre of mass
//! sits is what drifts, so that is what gets measured. An unknown
//! PAYLOAD mass is a different question, answered by `SET_PAYLOAD`.
//!
//! The regularisation pulls toward the model's current first moments: a
//! serial arm cannot see every direction of them from statics (the first
//! body of a vertical-axis arm contributes no gravity torque at all), and
//! along those directions the URDF stays authoritative rather than
//! drifting to noise. [`GravityFit::excitation`] says which bodies the
//! pose set actually measured.

use std::path::Path;

use crate::{Kin, KinError, NQ};

/// One body's identified inertial parameters, in its joint frame.
#[derive(Debug, Clone, PartialEq)]
pub struct BodyParams {
    /// The joint carrying the body (the URDF joint name).
    pub joint: String,
    /// Mass \[kg\].
    pub mass: f64,
    /// First moment `m · c` \[kg·m\].
    pub first_moment: [f64; 3],
}

impl BodyParams {
    /// Centre of mass \[m\] in the joint frame; the origin for a
    /// massless body, which has no centre of mass to report.
    pub fn com(&self) -> [f64; 3] {
        let m = self.mass;
        if m == 0.0 {
            return [0.0; 3];
        }
        [
            self.first_moment[0] / m,
            self.first_moment[1] / m,
            self.first_moment[2] / m,
        ]
    }

    fn from_flat(joint: String, v: &[f64]) -> Self {
        Self {
            joint,
            mass: v[0],
            first_moment: [v[1], v[2], v[3]],
        }
    }
}

/// One static measurement: the configuration the arm rested in and the
/// joint torques it held there with.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GravitySample {
    /// Joint configuration \[rad\].
    pub q: [f64; NQ],
    /// Measured joint torques \[Nm\].
    pub tau: [f64; NQ],
}

/// The outcome of [`fit`].
#[derive(Debug, Clone)]
pub struct GravityFit {
    /// Identified parameters, one per body, in model order. Masses are
    /// the prior's, unchanged; the first moments are fitted.
    pub bodies: Vec<BodyParams>,
    /// Per body and axis, the share of that component the DATA
    /// determined, from zero (the number is the prior, untouched) to one
    /// (the poses fixed it outright). This is the ridge shrinkage
    /// factor, so it accounts for parameters that are individually well
    /// excited but only identifiable in combination — a column norm does
    /// not. Gravity cannot see the component of a first moment along its
    /// own joint axis, nor anything about the first body of a
    /// vertical-axis arm, and those come back at zero.
    pub determined: Vec<[f64; 3]>,
    /// RMS torque residual of the prior over the fitted samples \[Nm\].
    pub rms_prior_nm: f64,
    /// RMS torque residual of the fit over the same samples \[Nm\].
    pub rms_fit_nm: f64,
}

/// The parameters the model currently carries, one per body.
pub fn model_params(kin: &Kin) -> Result<Vec<BodyParams>, KinError> {
    (0..kin.body_count())
        .map(|b| {
            let v = kin.body_inertial(b)?;
            Ok(BodyParams::from_flat(kin.joint_name(b)?, &v))
        })
        .collect()
}

/// `θ` as the regressor consumes it.
pub fn flatten(bodies: &[BodyParams]) -> Vec<f64> {
    bodies
        .iter()
        .flat_map(|b| {
            [
                b.mass,
                b.first_moment[0],
                b.first_moment[1],
                b.first_moment[2],
            ]
        })
        .collect()
}

/// `G(q)` under `θ`: the regressor product.
pub fn predict(kin: &mut Kin, theta: &[f64], q: &[f64; NQ]) -> Result<[f64; NQ], KinError> {
    let cols = 4 * kin.body_count();
    let mut y = vec![0.0; NQ * cols];
    kin.gravity_regressor(q, &mut y)?;
    let mut tau = [0.0; NQ];
    for (r, out) in tau.iter_mut().enumerate() {
        *out = y[r * cols..(r + 1) * cols]
            .iter()
            .zip(theta)
            .map(|(a, b)| a * b)
            .sum();
    }
    Ok(tau)
}

/// RMS torque residual of `θ` over `samples` \[Nm\].
pub fn rms(kin: &mut Kin, theta: &[f64], samples: &[GravitySample]) -> Result<f64, KinError> {
    if samples.is_empty() {
        return Ok(0.0);
    }
    let mut sum = 0.0;
    for s in samples {
        let tau = predict(kin, theta, &s.q)?;
        sum += tau
            .iter()
            .zip(&s.tau)
            .map(|(a, b)| (a - b) * (a - b))
            .sum::<f64>();
    }
    Ok((sum / (samples.len() * NQ) as f64).sqrt())
}

/// Fit each body's first moment to `samples`, holding masses at
/// `prior`'s and regularising toward `prior`'s first moments with
/// `prior_weight` (relative to the regressor's own scale: 0 = pure least
/// squares, 1 = the prior counts as much as the data). Needs at least
/// one sample and a prior with one entry per body.
pub fn fit(
    kin: &mut Kin,
    samples: &[GravitySample],
    prior: &[BodyParams],
    prior_weight: f64,
) -> Result<GravityFit, KinError> {
    let nb = kin.body_count();
    let cols = 4 * nb;
    let n = 3 * nb;
    if samples.is_empty() || prior.len() != nb || !prior_weight.is_finite() || prior_weight < 0.0 {
        return Err(KinError::Load(format!(
            "gravity fit needs samples, {nb} prior bodies and a non-negative weight \
             (got {} samples, {} bodies, weight {prior_weight})",
            samples.len(),
            prior.len()
        )));
    }
    let h0: Vec<f64> = prior.iter().flat_map(|b| b.first_moment).collect();
    // Normal equations over the first moments alone: the mass columns
    // multiply known values, so they move to the right-hand side.
    let mut ata = vec![0.0; n * n];
    let mut atb = vec![0.0; n];
    let mut y = vec![0.0; NQ * cols];
    let mut row = vec![0.0; n];
    for s in samples {
        kin.gravity_regressor(&s.q, &mut y)?;
        for r in 0..NQ {
            let full = &y[r * cols..(r + 1) * cols];
            let mut known = 0.0;
            for b in 0..nb {
                known += full[4 * b] * prior[b].mass;
                row[3 * b..3 * b + 3].copy_from_slice(&full[4 * b + 1..4 * b + 4]);
            }
            let target = s.tau[r] - known;
            for i in 0..n {
                atb[i] += row[i] * target;
                for j in 0..n {
                    ata[i * n + j] += row[i] * row[j];
                }
            }
        }
    }
    let trace: f64 = (0..n).map(|i| ata[i * n + i]).sum();
    let lambda = prior_weight * trace / n as f64 + f64::EPSILON;
    for i in 0..n {
        ata[i * n + i] += lambda;
        atb[i] += lambda * h0[i];
    }
    let l = cholesky_factor(&ata, n).ok_or_else(|| {
        KinError::Load("gravity fit normal matrix is not positive definite".into())
    })?;
    let h = cholesky_apply(&l, &atb, n);
    let bodies: Vec<BodyParams> = (0..nb)
        .map(|b| BodyParams {
            joint: prior[b].joint.clone(),
            mass: prior[b].mass,
            first_moment: [h[3 * b], h[3 * b + 1], h[3 * b + 2]],
        })
        .collect();
    // Ridge shrinkage: with A = YᵀY + λI, the share of parameter i the
    // data fixed is 1 - λ (A⁻¹)ᵢᵢ. Zero when the column is absent, one
    // when the poses pin it.
    let mut basis = vec![0.0; n];
    let mut share = vec![0.0; n];
    for i in 0..n {
        basis[i] = 1.0;
        let col = cholesky_apply(&l, &basis, n);
        basis[i] = 0.0;
        share[i] = (1.0 - lambda * col[i]).clamp(0.0, 1.0);
    }
    let determined = (0..nb)
        .map(|b| [share[3 * b], share[3 * b + 1], share[3 * b + 2]])
        .collect();
    Ok(GravityFit {
        rms_prior_nm: rms(kin, &flatten(prior), samples)?,
        rms_fit_nm: rms(kin, &flatten(&bodies), samples)?,
        bodies,
        determined,
    })
}

/// Cholesky factor `L` of symmetric positive-definite `A` (row-major
/// `n×n`), lower triangular; `None` when `A` is not positive definite.
fn cholesky_factor(a: &[f64], n: usize) -> Option<Vec<f64>> {
    let mut l = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..=i {
            let mut sum = a[i * n + j];
            for k in 0..j {
                sum -= l[i * n + k] * l[j * n + k];
            }
            if i == j {
                if sum <= 0.0 {
                    return None;
                }
                l[i * n + i] = sum.sqrt();
            } else {
                l[i * n + j] = sum / l[j * n + j];
            }
        }
    }
    Some(l)
}

/// Solve `A x = b` from `A`'s Cholesky factor.
fn cholesky_apply(l: &[f64], b: &[f64], n: usize) -> Vec<f64> {
    let mut z = vec![0.0; n];
    for i in 0..n {
        let mut sum = b[i];
        for k in 0..i {
            sum -= l[i * n + k] * z[k];
        }
        z[i] = sum / l[i * n + i];
    }
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = z[i];
        for k in i + 1..n {
            sum -= l[k * n + i] * x[k];
        }
        x[i] = sum / l[i * n + i];
    }
    x
}

/// The arm link's own parameters once the config tool's share is taken
/// back out of the payload body — what belongs in the arm URDF, which
/// the runtime loads with the tool re-attached from the gripper config.
pub fn without_tool(composite: &BodyParams, tool: [f64; 4]) -> BodyParams {
    BodyParams {
        joint: composite.joint.clone(),
        mass: composite.mass - tool[0],
        first_moment: [
            composite.first_moment[0] - tool[1],
            composite.first_moment[1] - tool[2],
            composite.first_moment[2] - tool[3],
        ],
    }
}

/// Deterministic configurations spread through the soft window (10 %
/// margin on every joint), for a calibration to rest the arm in.
pub fn calibration_poses(window: &[(f64, f64); NQ], count: usize, seed: u64) -> Vec<[f64; NQ]> {
    let mut state = seed
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    let mut next = || {
        state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (state >> 11) as f64 / (1u64 << 53) as f64
    };
    (0..count)
        .map(|_| {
            let mut q = [0.0; NQ];
            for (j, out) in q.iter_mut().enumerate() {
                let (lo, hi) = window[j];
                let span = hi - lo;
                *out = lo + span * (0.1 + 0.8 * next());
            }
            q
        })
        .collect()
}

/// `urdf` with each listed link's `<inertial>` centre of mass replaced by
/// `bodies` (matched joint → child link through the URDF's own joint
/// list). Mass and the inertia tensor are left as authored: statics
/// identify neither, and [`fit`] does not move the mass. Everything else
/// in the file is preserved byte-for-byte.
pub fn rewrite_inertials(urdf: &str, bodies: &[BodyParams]) -> Result<String, String> {
    let robot = urdf_rs::read_from_string(urdf).map_err(|e| format!("URDF parse: {e}"))?;
    let mut out = urdf.to_owned();
    for body in bodies {
        if !(body.mass.is_finite() && body.mass > 0.0)
            || body.first_moment.iter().any(|v| !v.is_finite())
        {
            return Err(format!(
                "joint {}: mass {} kg / first moment {:?} gives no writable centre of mass",
                body.joint, body.mass, body.first_moment
            ));
        }
        let link = robot
            .joints
            .iter()
            .find(|j| j.name == body.joint)
            .map(|j| j.child.link.clone())
            .ok_or_else(|| format!("joint {} is not in the URDF", body.joint))?;
        out = rewrite_link_inertial(&out, &link, body.com())?;
    }
    Ok(out)
}

/// The same rewrite, read from and written to `path`.
pub fn rewrite_inertials_file(path: &Path, bodies: &[BodyParams]) -> Result<(), String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let rewritten = rewrite_inertials(&text, bodies)?;
    std::fs::write(path, rewritten).map_err(|e| format!("{}: {e}", path.display()))
}

fn rewrite_link_inertial(text: &str, link: &str, com: [f64; 3]) -> Result<String, String> {
    let (start, end) =
        link_span(text, link).ok_or_else(|| format!("link {link} is not in the URDF"))?;
    let block = &text[start..end];
    let i0 = block
        .find("<inertial")
        .ok_or_else(|| format!("link {link} has no <inertial> element"))?;
    let i1 = block[i0..]
        .find("</inertial>")
        .map(|e| i0 + e)
        .ok_or_else(|| format!("link {link}: unterminated <inertial>"))?;
    let inertial = &block[i0..i1];
    let mut rewritten = inertial.to_owned();
    rewritten = replace_tag_attr(
        &rewritten,
        "<origin",
        "xyz",
        &format!("{} {} {}", com[0], com[1], com[2]),
    )
    .ok_or_else(|| format!("link {link}: <inertial> has no <origin xyz>"))?;
    let mut out = String::with_capacity(text.len());
    out.push_str(&text[..start + i0]);
    out.push_str(&rewritten);
    out.push_str(&text[start + i1..]);
    Ok(out)
}

/// Byte span of `<link ... name="name" ...> … </link>`.
fn link_span(text: &str, name: &str) -> Option<(usize, usize)> {
    let mut from = 0;
    while let Some(rel) = text[from..].find("<link") {
        let start = from + rel;
        let tag_end = text[start..].find('>')? + start;
        let tag = &text[start..tag_end];
        let attr = tag.find("name=\"").map(|n| &tag[n + 6..]);
        if attr.and_then(|a| a.find('"').map(|e| &a[..e])) == Some(name) {
            let end = text[tag_end..].find("</link>")? + tag_end + "</link>".len();
            return Some((start, end));
        }
        from = tag_end;
    }
    None
}

/// The first `tag` element in `text` with `attr="…"` replaced by `value`.
fn replace_tag_attr(text: &str, tag: &str, attr: &str, value: &str) -> Option<String> {
    let start = text.find(tag)?;
    let tag_end = text[start..].find('>')? + start;
    let key = format!("{attr}=\"");
    let a0 = text[start..tag_end].find(&key)? + start + key.len();
    let a1 = text[a0..].find('"')? + a0;
    Some(format!("{}{}{}", &text[..a0], value, &text[a1..]))
}
