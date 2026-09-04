//! Payload identification: what is the arm carrying?
//!
//! Statics is linear in `θ = [m_i, m_i c_i]` per body — `G(q) = Y(q) θ`
//! with `Y` the model's own gravity regressor ([`Kin::gravity_regressor`])
//! — so identification is a least-squares solve.
//!
//! The arm's OWN links are not identified here, and nothing in this
//! module writes a URDF. Their inertials come from the vendor's table
//! and that table is the authority: gravity cannot observe every
//! inertial parameter (nothing about the first body of a vertical-axis
//! arm, nor the component of a first moment along its own joint axis),
//! so a fit that corrected the observable directions would leave the
//! rest wrong while reporting a good residual. Anything that physically
//! changes a link changes parameters gravity cannot see, and needs new
//! nominal data — CAD or vendor — not a measurement.
//!
//! What no table can describe, and what actually changes between one
//! cycle and the next, is the load at the end of the chain: the tool
//! somebody bolted on and the part it just picked up. That is four
//! numbers — mass and the three components of `m·c` — on one body, and
//! [`fit_payload`] solves for exactly those. A payload of KNOWN mass
//! needs no measurement at all; declare it with `SET_PAYLOAD`.

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

/// What a payload identification found.
#[derive(Debug, Clone, PartialEq)]
pub struct PayloadFit {
    /// Identified mass \[kg\].
    pub mass: f64,
    /// Identified centre of mass in the payload body's frame \[m\].
    /// Meaningless when `mass` is at or near zero; `determined` says so.
    pub com: [f64; 3],
    /// Share of each of the four parameters `(m, m·cx, m·cy, m·cz)` the
    /// DATA fixed, from zero (the poses said nothing, the number is the
    /// starting guess) to one (the poses fixed it outright). This is the
    /// ridge shrinkage factor, so it accounts for parameters that are
    /// individually excited but only identifiable in combination. A
    /// wrist held still through the whole run leaves the first moments
    /// at zero here.
    pub determined: [f64; 4],
    /// RMS residual of the fit over the samples \[Nm\].
    pub rms_nm: f64,
    /// RMS residual of carrying nothing over the same samples \[Nm\] —
    /// what the arm was wrong by before the payload was identified.
    pub rms_unloaded_nm: f64,
}

/// Identify the payload at the end of the chain from measured torque.
///
/// The arm's own links are NOT fitted. Their inertials come from the
/// vendor's table and are the authority; what varies in service, and
/// what no table can describe, is whatever is bolted to or held by the
/// tool. So this solves for exactly the four parameters that describe
/// it — mass and the three components of `m·c` — against the torque the
/// arm cannot explain on its own.
///
/// `kin` must carry NO payload: the residual `measured − G_unloaded(q)`
/// is what the payload has to account for. The regressor's last body is
/// the payload body, and its four columns are already the linear form,
/// so this is a 4×4 solve whatever the arm's size.
///
/// `ridge` keeps a pose set that says nothing about a parameter from
/// running away with it (0 = pure least squares). Needs one sample.
pub fn fit_payload(
    kin: &mut Kin,
    samples: &[GravitySample],
    ridge: f64,
) -> Result<PayloadFit, KinError> {
    if samples.is_empty() || !ridge.is_finite() || ridge < 0.0 {
        return Err(KinError::Load(format!(
            "payload fit needs samples and a non-negative ridge (got {} samples, ridge {ridge})",
            samples.len()
        )));
    }
    let nb = kin.body_count();
    let cols = 4 * nb;
    // The payload body is the last in the chain, so its parameters are
    // the last four columns of the regressor.
    let base = cols - 4;

    let theta_unloaded = flatten(&model_params(kin)?);
    // One regressor evaluation per sample, kept: the unloaded torque, the
    // normal equations and both residuals are all products of it.
    let mut rows: Vec<Vec<f64>> = Vec::with_capacity(samples.len());
    let mut ata = [0.0; 16];
    let mut atb = [0.0; 4];
    let mut scale = 0.0f64;
    for s in samples {
        let mut y = vec![0.0; NQ * cols];
        kin.gravity_regressor(&s.q, &mut y)?;
        for r in 0..NQ {
            let full = &y[r * cols..(r + 1) * cols];
            let unloaded: f64 = full.iter().zip(&theta_unloaded).map(|(a, b)| a * b).sum();
            let row = &full[base..];
            let residual = s.tau[r] - unloaded;
            for a in 0..4 {
                scale = scale.max(row[a].abs());
                atb[a] += row[a] * residual;
                for b in 0..4 {
                    ata[a * 4 + b] += row[a] * row[b];
                }
            }
        }
        rows.push(y);
    }
    // Scaled by the regressor's own magnitude so the ridge means the
    // same thing on a small arm as on a large one.
    let lambda = ridge * scale * scale * (samples.len() * NQ) as f64;
    for a in 0..4 {
        ata[a * 4 + a] += lambda;
    }
    let l = cholesky_factor(&ata, 4)
        .ok_or_else(|| KinError::Load("payload fit normal matrix is not solvable".into()))?;
    let theta = cholesky_apply(&l, &atb, 4);

    // Shrinkage: how much of each parameter the data fixed rather than
    // the ridge. One minus the ridge's share of the inverse diagonal.
    let mut determined = [0.0; 4];
    for a in 0..4 {
        let mut e = [0.0; 4];
        e[a] = 1.0;
        let col = cholesky_apply(&l, &e, 4);
        determined[a] = (1.0 - lambda * col[a]).clamp(0.0, 1.0);
    }

    let mass = theta[0];
    let com = if mass.abs() > f64::EPSILON {
        [theta[1] / mass, theta[2] / mass, theta[3] / mass]
    } else {
        [0.0; 3]
    };

    // `theta` is what the payload ADDS to the body it hangs off, not
    // that body's parameters, so the loaded model is the unloaded one
    // plus the increment.
    let mut loaded = theta_unloaded.clone();
    for (out, add) in loaded[base..].iter_mut().zip(&theta) {
        *out += add;
    }
    let rms_of = |theta: &[f64]| {
        let mut sum = 0.0;
        for (y, s) in rows.iter().zip(samples) {
            for r in 0..NQ {
                let tau: f64 = y[r * cols..(r + 1) * cols]
                    .iter()
                    .zip(theta)
                    .map(|(a, b)| a * b)
                    .sum();
                sum += (tau - s.tau[r]) * (tau - s.tau[r]);
            }
        }
        (sum / (samples.len() * NQ) as f64).sqrt()
    };
    Ok(PayloadFit {
        mass,
        com,
        determined,
        rms_nm: rms_of(&loaded),
        rms_unloaded_nm: rms_of(&theta_unloaded),
    })
}

fn cholesky_factor(a: &[f64], n: usize) -> Option<Vec<f64>> {
    let mut l = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..=i {
            let mut sum = a[i * n + j];
            for k in 0..j {
                sum -= l[i * n + k] * l[j * n + k];
            }
            if i == j {
                // A NaN pivot passes `<= 0.0`, so test for a usable
                // number rather than for an unusable one.
                if !sum.is_finite() || sum <= 0.0 {
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
