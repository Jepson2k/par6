//! Payload identification from paired slow, opposite-direction sweeps.
//! Moving measurements separate gravity from symmetric gearbox friction;
//! a stopped position hold can conceal gravity error inside static friction.

use std::time::Duration;

use par6_client::{Ack, Client};
use par6_kin::gravity::{self, GravitySample, PayloadFit};
use par6_kin::{Collision, Kin, NQ};
use par6_proto::{CompletionPolicy, ControllerMode};

/// Joints varied between measurement poses. The opposite-direction
/// approaches also move the shoulder and elbow by `Protocol::approach_rad`.
pub const WRIST_JOINTS: [usize; 3] = [3, 4, 5];

/// Every gravity-loaded joint participates in opposite-direction
/// approaches so its drivetrain friction does not enter as payload mass.
pub const APPROACH_JOINTS: [usize; 5] = [1, 2, 3, 4, 5];

/// How a run approaches and samples the arm.
#[derive(Debug, Clone, Copy)]
pub struct Protocol {
    /// Joint-move speed fraction between poses.
    pub speed: f64,
    /// Approach offset on every gravity-loaded joint \[rad\]. Every pose
    /// is measured from both sides. The average cancels symmetric
    /// friction; load-dependent gearbox friction needs identification too.
    pub approach_rad: f64,
    /// Ramp and transient exclusion before collecting moving samples.
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
            let probe = approach_pose(&q, dir * approach_rad);
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

/// An opposite-direction approach, shared by execution and preview.
pub fn approach_pose(q: &[f64; NQ], by: f64) -> [f64; NQ] {
    let mut out = *q;
    for j in APPROACH_JOINTS {
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
    let result = match client.wait_command(index, protocol.pose_timeout).await {
        Ok(true) => Ok(()),
        Ok(false) => Err(format!("move_j {index} did not complete in time")),
        Err(e) => Err(format!("move_j {index}: {e}")),
    };
    if result.is_err() {
        stopped(client, result).await
    } else {
        result
    }
}

const SWEEP_RAD_S: f64 = 0.035;
const TARGET_ERROR_DEG: f64 = 0.5;
const STATUS_TIMEOUT: Duration = Duration::from_millis(500);
const HOLD_PERIOD: Duration = Duration::from_millis(20);

async fn stopped<T>(client: &Client, result: Result<T, String>) -> Result<T, String> {
    let stop_error = match client.stop(true).await {
        Ok(Ack::Confirmed) => return result,
        Ok(Ack::Unconfirmed) => "controller stop was not acknowledged".to_owned(),
        Err(e) => format!("controller stop failed: {e}"),
    };
    Err(match result {
        Ok(_) => stop_error,
        Err(e) => format!("{e}; {stop_error}"),
    })
}

/// Sample while crossing the pose. All gravity-loaded axes must actually move
/// in the requested direction; commanded feedforward is never a measurement.
async fn read_moving(
    client: &Client,
    q: &[f64; NQ],
    direction: f64,
    protocol: &Protocol,
) -> Result<GravitySample, String> {
    let mut rx = client.subscribe_status();
    let mut heartbeat = tokio::time::interval(HOLD_PERIOD);
    heartbeat.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let begin = tokio::time::Instant::now();
    let deadline = begin + protocol.pose_timeout;
    let mut last_frame = begin;
    let mut last_seq = None;
    let speed = SWEEP_RAD_S * protocol.speed;
    let ramp = protocol.settle.as_secs_f64().max(0.2);
    let mut target = approach_pose(q, -direction * protocol.approach_rad);
    let mut sample = GravitySample {
        q: [0.0; NQ],
        tau: [0.0; NQ],
    };
    let mut taken = 0usize;
    let mut history = std::collections::VecDeque::new();
    loop {
        tokio::select! {
            _ = tokio::time::sleep_until(deadline) => {
                return Err("moving gravity sample exceeded its time budget".into());
            }
            _ = heartbeat.tick() => {
                if last_frame.elapsed() > STATUS_TIMEOUT {
                    return Err("status stopped during gravity sweep".into());
                }
                let t = begin.elapsed().as_secs_f64();
                let travel = if t < ramp { speed * t * t / (2.0 * ramp) }
                    else { speed * (t - ramp * 0.5) };
                if travel >= 2.0 * protocol.approach_rad {
                    return Err(format!("insufficient moving samples through pose: {taken}/{}", protocol.frames));
                }
                target = approach_pose(q, direction * (travel - protocol.approach_rad));
                client.servo_j(to_deg(&target), Some(protocol.speed), None).await
                    .map_err(|e| format!("gravity sweep: {e}"))?;
                continue;
            }
            changed = rx.changed() => {
                changed.map_err(|_| "status closed during gravity sweep")?;
            }
        }
        let Some(s) = rx.borrow_and_update().clone() else {
            continue;
        };
        if last_seq.is_some_and(|seq| s.seq <= seq) {
            continue;
        }
        last_seq = Some(s.seq);
        if s.data_age_ms <= 100 {
            last_frame = tokio::time::Instant::now();
        }
        if s.error.is_some() || !s.enabled || !s.homed || s.link_ok != 1 {
            return Err(format!("gravity sweep lost readiness: {:?}", s.error));
        }
        if !(0..NQ)
            .all(|j| s.angles[j].is_finite() && s.speeds[j].is_finite() && s.torques[j].is_finite())
        {
            return Err("non-finite gravity sweep feedback".into());
        }
        if s.data_age_ms > 100
            || s.mode != ControllerMode::Stream
            || begin.elapsed().as_secs_f64() < 2.0 * ramp
        {
            continue;
        }
        history.push_back((s.mono_time_ns, s.angles));
        while history.len() > 2 && s.mono_time_ns.saturating_sub(history[1].0) >= 100_000_000 {
            history.pop_front();
        }
        let (then, angles) = history.front().unwrap();
        let span = s.mono_time_ns.saturating_sub(*then) as f64 * 1e-9;
        if span < 0.08 {
            continue;
        }
        // Centered window keeps opposite-direction configurations paired. The
        // approach's acceleration and its final deceleration are excluded.
        let usable = APPROACH_JOINTS.iter().all(|&j| {
            let velocity = direction * (s.angles[j] - angles[j]).to_radians() / span;
            velocity >= speed * 0.5
                && velocity <= speed * 1.8
                && (s.angles[j].to_radians() - q[j]).abs() <= protocol.approach_rad * 0.5
                && (s.angles[j] - target[j].to_degrees()).abs() <= TARGET_ERROR_DEG
        });
        if !usable {
            continue;
        }
        for j in 0..NQ {
            sample.q[j] += s.angles[j].to_radians();
            sample.tau[j] += s.torques[j];
        }
        taken += 1;
        if taken == protocol.frames {
            for j in 0..NQ {
                sample.q[j] /= taken as f64;
                sample.tau[j] /= taken as f64;
            }
            return Ok(sample);
        }
    }
}

/// Cross `q` in both directions and average the moving torques.
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
        move_to(
            client,
            &approach_pose(q, dir * protocol.approach_rad),
            protocol,
        )
        .await?;
        let s = stopped(client, read_moving(client, q, -dir, protocol).await).await?;
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
    if !protocol.speed.is_finite()
        || !(0.0..=1.0).contains(&protocol.speed)
        || protocol.speed == 0.0
        || !protocol.approach_rad.is_finite()
        || protocol.approach_rad <= 0.0
        || protocol.frames < 2
        || protocol.pose_timeout.is_zero()
        || protocol.settle >= protocol.pose_timeout
        || poses.is_empty()
        || poses.iter().flatten().any(|q| !q.is_finite())
    {
        return Err("invalid calibration poses or sampling protocol".into());
    }
    // Arrive under the caller-independent settle policy, then keep a
    // live position hold during the measurement. Restore the policy on exit.
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
    let result = stopped(client, run.await).await;
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
/// torque the unloaded identification model cannot account for. The
/// controller keeps its existing compensation while position feedback
/// supplies the difference during measurement.
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
    /// Gravity model the fit predicts torques with. Loaded with NO tool
    /// when the load at the flange is what is being measured.
    pub kin: Kin,
    /// The world the wrist poses are planned clear of.
    pub collision: Collision,
    /// Per-joint soft limits every pose and its approach must stay inside.
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
        .map_err(|e| format!("set_payload: {e}"))
        .and_then(|ack| match ack {
            Ack::Confirmed => Ok(()),
            Ack::Unconfirmed => Err("set_payload was not acknowledged".into()),
        })
}

/// The whole operation, as a program calls it: find what the arm is
/// carrying and, if asked, tell the runtime.
///
/// Position feedback stays active while measuring. The existing payload
/// declaration remains in place until a valid replacement is ready.
/// A failure to apply that replacement restores the previous declaration.
pub async fn estimate(
    client: &Client,
    model: &mut EstimationModel,
    spread: f64,
    ridge: f64,
    declare_result: bool,
) -> Result<Report, String> {
    let previous = declared(client).await?;
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
            if let Err(error) = declare(client, (report.fit.mass, report.fit.com, None)).await {
                return match declare(client, previous).await {
                    Ok(()) => Err(error),
                    Err(restore) => Err(format!("{error}; restoring payload: {restore}")),
                };
            }
            Ok(report)
        }
        Ok((report, false)) => Ok(report),
        Err(e) => Err(e),
    }
}
