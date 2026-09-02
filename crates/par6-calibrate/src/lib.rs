//! Gravity-model calibration against a running `par6d`: rest the arm in a
//! spread of configurations, read the torques it holds each one with,
//! fit every link's mass and centre of mass to them
//! ([`par6_kin::gravity::fit`]), and report how well the fit predicts
//! torques at poses it never saw. The model under calibration is the
//! runtime's own gravity chain (`par6_arm.urdf` plus the fitted tool from
//! the gripper config), so what gets written back is what the daemon
//! loads next boot.

use std::time::Duration;

use par6_client::Client;
use par6_kin::gravity::{self, BodyParams, GravityFit, GravitySample};
use par6_kin::{Collision, Kin, NQ};
use par6_proto::CompletionPolicy;

/// How a calibration run rests and reads the arm.
#[derive(Debug, Clone, Copy)]
pub struct Protocol {
    /// Joint-move speed fraction between poses.
    pub speed: f64,
    /// Approach offset per joint \[rad\]. Every pose is measured twice,
    /// reached once from above and once from below, and the two torque
    /// readings averaged: a joint's friction opposes its travel, so it
    /// enters the two with opposite signs and cancels. A single-direction
    /// reading folds the whole friction band into the identified masses.
    pub approach_rad: f64,
    /// Rest after the runtime reports the move complete, before reading.
    pub settle: Duration,
    /// Consecutive STATUS frames averaged per reading. The window has to
    /// cover whatever limit cycle the position loop holds the joint in —
    /// the mean over it is the torque the arm holds the pose with.
    pub frames: usize,
    /// How long one pose may take before the run gives up.
    pub pose_timeout: Duration,
}

impl Default for Protocol {
    fn default() -> Self {
        Self {
            speed: 0.5,
            approach_rad: 0.05,
            settle: Duration::from_millis(500),
            frames: 25,
            pose_timeout: Duration::from_secs(60),
        }
    }
}

/// A calibration's outcome.
#[derive(Debug, Clone)]
pub struct Report {
    /// The fit over the training poses.
    pub fit: GravityFit,
    /// The model's prior parameters, for the before/after readout.
    pub prior: Vec<BodyParams>,
    /// RMS residual of the prior on the held-out poses \[Nm\].
    pub holdout_rms_prior_nm: f64,
    /// RMS residual of the fit on the held-out poses \[Nm\].
    pub holdout_rms_fit_nm: f64,
    /// Every measurement taken, training poses first.
    pub samples: Vec<GravitySample>,
}

/// Configurations inside `window` that the model's collision world
/// clears — the pose itself and both approach poses either side of it,
/// since a calibration drives through all three — in the deterministic
/// order [`gravity::calibration_poses`] draws them.
pub fn plan_poses(
    collision: &mut Collision,
    window: &[(f64, f64); NQ],
    count: usize,
    seed: u64,
    approach_rad: f64,
) -> Result<Vec<[f64; NQ]>, String> {
    let mut out = Vec::with_capacity(count);
    // Drawing several times the count leaves room for the poses the
    // collision world rejects; a window that rejects nearly everything
    // is a configuration problem the caller should hear about.
    for q in gravity::calibration_poses(window, count * 4, seed) {
        let mut clear = true;
        for dir in [0.0, 1.0, -1.0] {
            let probe = offset(&q, dir * approach_rad);
            if collision
                .check(&probe, true)
                .map_err(|e| format!("collision check: {e}"))?
                .active()
            {
                clear = false;
                break;
            }
        }
        if clear {
            out.push(q);
            if out.len() == count {
                return Ok(out);
            }
        }
    }
    Err(format!(
        "only {} of {count} calibration poses clear the collision world",
        out.len()
    ))
}

fn offset(q: &[f64; NQ], by: f64) -> [f64; NQ] {
    let mut out = *q;
    for v in out.iter_mut() {
        *v += by;
    }
    out
}

fn to_deg(q: &[f64; NQ]) -> [f64; NQ] {
    let mut out = [0.0; NQ];
    for (o, r) in out.iter_mut().zip(q) {
        *o = r.to_degrees();
    }
    out
}

/// Drive to `q` and wait for the runtime to report the move complete.
/// Under the SETTLED completion policy — which [`calibrate`] selects —
/// that is the runtime's own settle rule, not a second one here.
async fn move_to(client: &Client, q: &[f64; NQ], protocol: &Protocol) -> Result<(), String> {
    let index = client
        .move_j(to_deg(q), None, Some(protocol.speed), None, None, false)
        .await
        .map_err(|e| format!("move_j: {e}"))?
        .ok_or("move_j went unconfirmed")?;
    match client.wait_command(index, protocol.pose_timeout).await {
        Ok(true) => Ok(()),
        Ok(false) => Err(format!("move_j {index} did not complete in time")),
        Err(e) => Err(format!("move_j {index}: {e}")),
    }
}

/// The mean configuration and torque over `protocol.frames` STATUS
/// frames, taken where the arm currently rests.
async fn read_held(client: &Client, protocol: &Protocol) -> Result<GravitySample, String> {
    tokio::time::sleep(protocol.settle).await;
    let mut rx = client.subscribe_status();
    let mut sample = GravitySample {
        q: [0.0; NQ],
        tau: [0.0; NQ],
    };
    let mut taken = 0usize;
    let mut first: Option<[f64; NQ]> = None;
    let deadline = tokio::time::Instant::now() + protocol.pose_timeout;
    while taken < protocol.frames {
        match tokio::time::timeout_at(deadline, rx.changed()).await {
            Ok(Ok(())) => {}
            _ => return Err("the status stream stopped while sampling".into()),
        }
        let Some(s) = rx.borrow_and_update().clone() else {
            continue;
        };
        // A torque reading is only gravity if the arm is actually
        // holding the pose. A fault, a dropped bus or a disabled arm all
        // produce numbers that look like measurements and would be
        // fitted as if they were. Rest is NOT tested here: the position
        // loop holds a pose in a limit cycle, so reported speeds stay
        // nonzero while the angles sit still. What decides the move is
        // over is the runtime's SETTLED policy, and what removes the
        // chatter is averaging over `frames`.
        if let Some(e) = &s.error {
            return Err(format!(
                "the arm faulted while sampling: {} ({})",
                e.cause, e.code
            ));
        }
        if !s.enabled {
            return Err("the arm was disabled while sampling".into());
        }
        if s.link_ok != 1 {
            return Err("the motor bus link went stale while sampling".into());
        }
        // Rest is judged on the ANGLES, not the speeds: the position
        // loop holds a pose in a limit cycle, so reported speeds stay
        // nonzero while the arm sits still (parol6's `wait_motion` pairs
        // its speed threshold with an angle one for the same reason).
        // A window the arm moved across is not one pose's torque.
        match &first {
            None => first = Some(std::array::from_fn(|j| s.angles[j])),
            Some(a0) => {
                let drift = (0..NQ)
                    .map(|j| (s.angles[j] - a0[j]).abs())
                    .fold(0.0, f64::max);
                if drift > REST_DRIFT_DEG {
                    return Err(format!(
                        "the arm moved {drift:.3} deg while sampling this pose, so the \
                         frames are not one pose's torque"
                    ));
                }
            }
        }
        for j in 0..NQ {
            sample.q[j] += s.angles[j].to_radians();
            sample.tau[j] += s.torques[j];
        }
        taken += 1;
    }
    let n = taken as f64;
    for j in 0..NQ {
        sample.q[j] /= n;
        sample.tau[j] /= n;
    }
    Ok(sample)
}

/// Rest the arm in `q` and read the torques it holds there with, arrived
/// at from both directions and averaged (see [`Protocol::approach_rad`]).
pub async fn measure_pose(
    client: &Client,
    q: &[f64; NQ],
    protocol: &Protocol,
) -> Result<GravitySample, String> {
    let mut mean = GravitySample {
        q: [0.0; NQ],
        tau: [0.0; NQ],
    };
    for dir in [1.0, -1.0] {
        move_to(client, &offset(q, dir * protocol.approach_rad), protocol).await?;
        move_to(client, q, protocol).await?;
        let s = read_held(client, protocol).await?;
        for j in 0..NQ {
            mean.q[j] += 0.5 * s.q[j];
            mean.tau[j] += 0.5 * s.tau[j];
        }
    }
    Ok(mean)
}

/// Measure every pose in order.
pub async fn measure(
    client: &Client,
    poses: &[[f64; NQ]],
    protocol: &Protocol,
) -> Result<Vec<GravitySample>, String> {
    // The readings are the torques the arm holds a FINISHED move with, so
    // the runtime has to be the one that decides a move is finished and
    // settled — the policy is stated here rather than assumed.
    client
        .set_completion_policy(CompletionPolicy::Settled)
        .await
        .map_err(|e| format!("set_completion_policy: {e}"))?;
    let mut samples = Vec::with_capacity(poses.len());
    for (i, q) in poses.iter().enumerate() {
        log::info!("calibration pose {}/{}", i + 1, poses.len());
        samples.push(measure_pose(client, q, protocol).await?);
    }
    Ok(samples)
}

/// Fit `kin`'s bodies to `samples`, the last `holdout` of them kept back
/// to score the fit on poses it never saw.
pub fn evaluate(
    kin: &mut Kin,
    samples: Vec<GravitySample>,
    holdout: usize,
    prior: Vec<BodyParams>,
    prior_weight: f64,
) -> Result<Report, String> {
    if holdout >= samples.len() {
        return Err(format!(
            "{holdout} held-out poses leave nothing of {} to fit",
            samples.len()
        ));
    }
    let (train, held) = samples.split_at(samples.len() - holdout);
    let fit = gravity::fit(kin, train, &prior, prior_weight).map_err(|e| e.to_string())?;
    let theta0 = gravity::flatten(&prior);
    let theta = gravity::flatten(&fit.bodies);
    Ok(Report {
        holdout_rms_prior_nm: gravity::rms(kin, &theta0, held).map_err(|e| e.to_string())?,
        holdout_rms_fit_nm: gravity::rms(kin, &theta, held).map_err(|e| e.to_string())?,
        fit,
        prior,
        samples,
    })
}

/// The whole run: measure `poses` on the arm behind `client`, then fit
/// and score `kin` against them.
pub async fn calibrate(
    client: &Client,
    kin: &mut Kin,
    poses: &[[f64; NQ]],
    holdout: usize,
    protocol: &Protocol,
    prior_weight: f64,
) -> Result<Report, String> {
    let prior = gravity::model_params(kin).map_err(|e| e.to_string())?;
    let samples = measure(client, poses, protocol).await?;
    evaluate(kin, samples, holdout, prior, prior_weight)
}

/// The fitted parameters as they belong in the arm URDF: the tool's
/// share (attached from the gripper config at load) taken back out of
/// the payload body, which is the last one in the chain.
///
/// Bodies left without mass are dropped rather than written. The arm
/// URDF ends in a massless tool stub, so after the tool's share comes
/// out there is no centre of mass to place, and whatever the fit found
/// there describes the tool — which lives in the gripper config, not
/// here.
pub fn arm_params(kin: &Kin, fitted: &[BodyParams]) -> Result<Vec<BodyParams>, String> {
    let tool = kin.tool_inertial().map_err(|e| e.to_string())?;
    let last = fitted.len().checked_sub(1).ok_or("no bodies were fitted")?;
    Ok(fitted
        .iter()
        .enumerate()
        .map(|(i, b)| {
            if i == last {
                gravity::without_tool(b, tool)
            } else {
                b.clone()
            }
        })
        .filter(|b| b.mass > 0.0)
        .collect())
}

/// How far a joint may drift across one pose's sampling window before
/// the frames stop describing a single pose \[deg\].
///
/// Sized between the two things it has to tell apart: the position
/// loop's limit cycle, measured at about half a degree peak-to-peak
/// across a window, and the smallest motion a calibration commands,
/// which is the approach offset at 0.05 rad (2.9 deg). parol6 draws the
/// same line at half a degree, but frame-to-frame rather than across a
/// window, and pairs it with a speed threshold and a settle window.
const REST_DRIFT_DEG: f64 = 2.0;

/// An axis counts as measured when the data fixed more of it than the
/// prior did (see `GravityFit::determined`).
pub const MEASURED: f64 = 0.5;

/// One line per body: how far the fitted centre of mass moved, and how
/// many of its three axes the pose set actually measured.
pub fn describe(report: &Report) -> String {
    let mut out = String::new();
    for ((before, after), excite) in report
        .prior
        .iter()
        .zip(&report.fit.bodies)
        .zip(&report.fit.determined)
    {
        let (cb, ca) = (before.com(), after.com());
        let moved = cb
            .iter()
            .zip(&ca)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max);
        let seen = excite.iter().filter(|e| **e > MEASURED).count();
        out.push_str(&format!(
            "{:<16} mass {:6.3} kg (held)  com [{:+.4} {:+.4} {:+.4}] -> \
             [{:+.4} {:+.4} {:+.4}] m  moved {:.4} m  {seen}/3 axes measured\n",
            before.joint, before.mass, cb[0], cb[1], cb[2], ca[0], ca[1], ca[2], moved
        ));
    }
    out.push_str(&format!(
        "rms residual: fitted poses {:.4} -> {:.4} Nm, held-out poses {:.4} -> {:.4} Nm\n",
        report.fit.rms_prior_nm,
        report.fit.rms_fit_nm,
        report.holdout_rms_prior_nm,
        report.holdout_rms_fit_nm
    ));
    out
}
