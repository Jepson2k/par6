//! Payload identification against a running `par6d`: rest the arm in a
//! few wrist poses, read the torques it holds each one with, and solve
//! for the load at the end of the chain
//! ([`par6_kin::gravity::fit_payload`]).
//!
//! Only the WRIST moves. The payload hangs off the end of the chain, so
//! its lever arm changes with the wrist and nothing else has to travel
//! for the four parameters to separate — which is what keeps this a
//! seconds-long operation a program can run after a pick, rather than a
//! workspace-wide procedure. The arm's own links are never fitted; their
//! inertials are the vendor's and stay that way.

use std::time::Duration;

use par6_client::Client;
use par6_kin::gravity::{self, GravitySample, PayloadFit};
use par6_kin::{Collision, Kin, NQ};
use par6_proto::CompletionPolicy;

/// Joints the identification moves. The payload's lever arm about the
/// wrist is what makes its first moment observable; the arm below stays
/// where the caller left it, so the pick is not disturbed and the moves
/// stay small.
pub const WRIST_JOINTS: [usize; 3] = [3, 4, 5];

/// How a run rests and reads the arm.
#[derive(Debug, Clone, Copy)]
pub struct Protocol {
    /// Joint-move speed fraction between poses.
    pub speed: f64,
    /// Approach offset per moved joint \[rad\]. Every pose is measured
    /// twice, reached once from each side, and the readings averaged: a
    /// joint's friction opposes its travel, so it enters the two with
    /// opposite signs and cancels. A single-direction reading folds the
    /// whole friction band into the identified mass.
    pub approach_rad: f64,
    /// Rest after the runtime reports the move complete, before reading.
    pub settle: Duration,
    /// Consecutive STATUS frames averaged per reading.
    pub frames: usize,
    /// How long one pose may take before the run gives up.
    pub pose_timeout: Duration,
}

impl Default for Protocol {
    fn default() -> Self {
        Self {
            speed: 1.0,
            approach_rad: 0.05,
            settle: Duration::from_millis(250),
            frames: 20,
            pose_timeout: Duration::from_secs(30),
        }
    }
}

/// A run's outcome.
#[derive(Debug, Clone)]
pub struct Report {
    /// What was identified.
    pub fit: PayloadFit,
    /// Every measurement taken.
    pub samples: Vec<GravitySample>,
}

/// Wrist poses around `start` that the collision world clears, including
/// both approach poses either side of each.
///
/// The wrist is swung over `spread` either side of where it sits, in the
/// three joints that give the payload a lever arm. A pose whose approach
/// would collide is dropped rather than adjusted: with the arm below
/// held still there is nothing to trade off.
pub fn plan_poses(
    collision: &mut Collision,
    start: &[f64; NQ],
    window: &[(f64, f64); NQ],
    spread: f64,
    approach_rad: f64,
) -> Result<Vec<[f64; NQ]>, String> {
    // Each moved joint is swung both ways, plus the pose the arm is
    // already in: enough lever arms to separate mass from first moment,
    // and few enough to stay quick.
    let mut candidates = vec![*start];
    for j in WRIST_JOINTS {
        for dir in [1.0, -1.0] {
            let mut q = *start;
            q[j] += dir * spread;
            candidates.push(q);
        }
    }

    let mut out = Vec::new();
    for q in candidates {
        // The pose and both approach poses either side of it have to be
        // inside the window and clear of the world: a daemon refusing an
        // approach mid-run has already had the payload cleared.
        let mut usable = true;
        for dir in [0.0, 1.0, -1.0] {
            let probe = offset(&q, dir * approach_rad);
            let inside = (0..NQ).all(|j| {
                let (lo, hi) = window[j];
                probe[j] >= lo && probe[j] <= hi
            });
            if !inside
                || collision
                    .check(&probe, true)
                    .map_err(|e| format!("collision check: {e}"))?
                    .active()
            {
                usable = false;
                break;
            }
        }
        if usable {
            out.push(q);
        }
    }
    if out.len() < 3 {
        return Err(format!(
            "only {} wrist poses are reachable and clear from here — move the arm \
             somewhere with room around the wrist and try again",
            out.len()
        ));
    }
    Ok(out)
}

/// Offset only the joints the identification moves.
fn offset(q: &[f64; NQ], by: f64) -> [f64; NQ] {
    let mut out = *q;
    for j in WRIST_JOINTS {
        out[j] += by;
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
/// Under the SETTLED completion policy — which [`measure`] selects —
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

/// How far a joint may drift across one pose's sampling window before
/// the frames stop describing a single pose \[deg\].
///
/// Sized between the two things it has to tell apart: the position
/// loop's limit cycle, measured at about half a degree peak-to-peak
/// across a window, and the smallest motion this commands, which is the
/// approach offset at 0.05 rad (2.9 deg). parol6 draws the same line at
/// half a degree, but frame-to-frame rather than across a window, and
/// pairs it with a speed threshold and a settle window.
const REST_DRIFT_DEG: f64 = 2.0;

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
        // fitted as if they were. Rest is NOT tested on speed: the
        // position loop holds a pose in a limit cycle, so reported
        // speeds stay nonzero while the angles sit still.
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
    // settled — the policy is stated here rather than assumed. It is the
    // caller's session, though, so whatever they had is put back after:
    // what they set, or the server's boot default if they never did.
    let previous = client
        .completion_policy()
        .unwrap_or(CompletionPolicy::Settled);
    client
        .set_completion_policy(CompletionPolicy::Settled)
        .await
        .map_err(|e| format!("set_completion_policy: {e}"))?;
    let run = async {
        let mut samples = Vec::with_capacity(poses.len());
        for (i, q) in poses.iter().enumerate() {
            log::info!("pose {}/{}", i + 1, poses.len());
            samples.push(measure_pose(client, q, protocol).await?);
        }
        Ok::<_, String>(samples)
    };
    let result = run.await;
    client
        .set_completion_policy(previous)
        .await
        .map_err(|e| format!("restoring the completion policy: {e}"))?;
    result
}

/// The whole run: swing the wrist through `poses` on the arm behind
/// `client`, solve for what it is carrying, and return to `start` — the
/// pose the caller left the arm in, which `plan_poses` may have dropped
/// from `poses` if its approach did not clear.
///
/// `kin` must carry no payload — the residual the fit explains is the
/// torque the UNLOADED model cannot account for — so the caller clears
/// the runtime's payload first and declares the result afterwards.
pub async fn identify(
    client: &Client,
    kin: &mut Kin,
    start: [f64; NQ],
    poses: &[[f64; NQ]],
    protocol: &Protocol,
    ridge: f64,
) -> Result<Report, String> {
    if poses.is_empty() {
        return Err("no poses to measure".into());
    }
    let samples = measure(client, poses, protocol).await?;
    let fit = gravity::fit_payload(kin, &samples, ridge).map_err(|e| e.to_string())?;
    move_to(client, &start, protocol).await?;
    Ok(Report { fit, samples })
}

/// A parameter counts as measured when the data fixed more of it than
/// the ridge did (see [`PayloadFit::determined`]).
pub const MEASURED: f64 = 0.5;

/// What an estimation measures against: the arm with its fitted
/// gripper, the collision world the wrist swing is planned in, and the
/// joint window. Built by the daemon crate from its own config
/// resolution (`par6d::kin::estimation_model`), so an estimate runs
/// against exactly the arm the daemon models.
pub struct EstimationModel {
    pub kin: Kin,
    pub collision: Collision,
    pub window: [(f64, f64); NQ],
}

/// What the runtime is carrying, as `SET_PAYLOAD` takes it back.
type Declared = (f64, [f64; 3], Option<[f64; 6]>);

async fn declared(client: &Client) -> Result<Declared, String> {
    match client
        .payload()
        .await
        .map_err(|e| format!("payload: {e}"))?
    {
        par6_proto::QueryResult::Payload { mass, com, inertia } => Ok((
            mass,
            com,
            if inertia == [0.0; 6] {
                None
            } else {
                Some(inertia)
            },
        )),
        other => Err(format!("payload query answered {other:?}")),
    }
}

async fn declare(client: &Client, (mass, com, inertia): Declared) -> Result<(), String> {
    client
        .set_payload(mass, com, inertia)
        .await
        .map(|_| ())
        .map_err(|e| format!("set_payload: {e}"))
}

/// The whole operation, as a program calls it: find what the arm is
/// carrying and, if asked, tell the runtime.
///
/// The load is found in the torque the UNLOADED model cannot explain, so
/// whatever is declared comes off first — and goes back on every exit
/// that does not declare, failure included, or a curious call leaves the
/// arm compensating for nothing while it still holds the part. With
/// `declare`, a result the poses did not actually measure is refused
/// rather than pushed into the gravity model as noise.
pub async fn estimate(
    client: &Client,
    model: &mut EstimationModel,
    spread: f64,
    ridge: f64,
    declare_result: bool,
) -> Result<Report, String> {
    let previous = declared(client).await?;
    declare(client, (0.0, [0.0; 3], None)).await?;

    let run = async {
        let angles = client.angles().await.map_err(|e| format!("angles: {e}"))?;
        let mut start = [0.0; NQ];
        for (out, deg) in start.iter_mut().zip(angles.iter()) {
            *out = deg.to_radians();
        }
        let protocol = Protocol::default();
        let poses = plan_poses(
            &mut model.collision,
            &start,
            &model.window,
            spread,
            protocol.approach_rad,
        )?;
        let report = identify(client, &mut model.kin, start, &poses, &protocol, ridge).await?;
        if !declare_result {
            return Ok((report, false));
        }
        if report.fit.determined[0] <= MEASURED {
            return Err(format!(
                "the poses did not measure the mass (determined {:.2}); give the wrist more \
                 room or a wider spread",
                report.fit.determined[0]
            ));
        }
        if !(report.fit.mass.is_finite() && report.fit.mass > 0.0) {
            return Err(format!(
                "refusing to declare a mass of {:.4} kg",
                report.fit.mass
            ));
        }
        Ok((report, true))
    };
    match run.await {
        Ok((report, true)) => {
            declare(client, (report.fit.mass, report.fit.com, None)).await?;
            Ok(report)
        }
        Ok((report, false)) => {
            declare(client, previous).await?;
            Ok(report)
        }
        Err(e) => {
            declare(client, previous).await?;
            Err(e)
        }
    }
}
