//! Payload identification against a running `par6d`: rest the arm in a
//! few wrist poses, read the torques it holds each one with, and solve
//! for the load at the end of the chain
//! ([`par6_kin::gravity::fit_payload`]).
//!
//! Measurement poses vary the wrist. Small opposite-direction approaches
//! also move the shoulder and elbow: friction on every gravity-loaded
//! joint must change sign before the pair can estimate gravity torque.
//! These static measurements identify payload; arm gravity calibration
//! also needs poses that vary the arm's own lever arms.

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

/// How a run rests and reads the arm.
#[derive(Debug, Clone, Copy)]
pub struct Protocol {
    /// Joint-move speed fraction between poses.
    pub speed: f64,
    /// Approach offset on every gravity-loaded joint \[rad\]. Every pose
    /// is measured from both sides. The average cancels symmetric
    /// friction; load-dependent gearbox friction needs identification too.
    pub approach_rad: f64,
    /// Required stable position hold before collecting the sample window.
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

/// A static measurement must stay inside a quarter-degree envelope and
/// below 0.03 rad/s; a persistent limit cycle is not a gravity sample.
const REST_DRIFT_DEG: f64 = 0.25;
const REST_SPEED_RAD_S: f64 = 0.03;
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

/// Hold the requested pose with position feedback while reading actual
/// drive torque. IDLE's model-only feedforward cannot measure model error.
async fn read_held(
    client: &Client,
    q: &[f64; NQ],
    protocol: &Protocol,
) -> Result<GravitySample, String> {
    let target = to_deg(q);
    let mut rx = client.subscribe_status();
    let mut heartbeat = tokio::time::interval(HOLD_PERIOD);
    heartbeat.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let deadline = tokio::time::Instant::now() + protocol.pose_timeout;
    let mut last_frame = tokio::time::Instant::now();
    let mut last_seq = None;
    let mut stable_since = None;
    let mut sample = GravitySample {
        q: [0.0; NQ],
        tau: [0.0; NQ],
    };
    let mut taken = 0usize;
    let mut lo = [f64::INFINITY; NQ];
    let mut hi = [f64::NEG_INFINITY; NQ];
    loop {
        tokio::select! {
            _ = tokio::time::sleep_until(deadline) => {
                return Err(format!("the arm did not hold a stable pose within the sampling budget: target_deg={target:?}, latest={:?}", client.latest_status().map(|s| (s.mode, s.angles, s.speeds, s.data_age_ms))));
            }
            _ = heartbeat.tick() => {
                if last_frame.elapsed() > STATUS_TIMEOUT {
                    return Err("the status stream stopped while sampling".into());
                }
                client.servo_j(target, Some(protocol.speed), None).await
                    .map_err(|e| format!("position hold: {e}"))?;
                continue;
            }
            changed = rx.changed() => {
                changed.map_err(|_| "the status stream closed while sampling")?;
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
        if let Some(e) = &s.error {
            return Err(format!(
                "the arm faulted while sampling: {} ({})",
                e.cause, e.code
            ));
        }
        if !s.enabled || !s.homed {
            return Err("the arm became disabled or unreferenced while sampling".into());
        }
        if s.link_ok != 1 {
            return Err(format!("the motor bus link went stale while sampling: link_ok={}, data_age_ms={}, mode={:?}, loop={:?}", s.link_ok, s.data_age_ms, s.mode, s.loop_health));
        }
        if !(0..NQ)
            .all(|j| s.angles[j].is_finite() && s.speeds[j].is_finite() && s.torques[j].is_finite())
        {
            return Err("non-finite position, speed or torque while sampling".into());
        }
        // Delayed frames invalidate this sampling window. A sustained
        // loss still stops through the link and heartbeat deadlines.
        let stable = s.data_age_ms <= 100
            && s.mode == ControllerMode::Stream
            && (0..NQ).all(|j| {
                s.speeds[j].abs() <= REST_SPEED_RAD_S
                    && (s.angles[j] - target[j]).abs() <= TARGET_ERROR_DEG
            });
        if !stable {
            stable_since = None;
            taken = 0;
            sample = GravitySample {
                q: [0.0; NQ],
                tau: [0.0; NQ],
            };
            lo.fill(f64::INFINITY);
            hi.fill(f64::NEG_INFINITY);
            continue;
        }
        let since = *stable_since.get_or_insert(s.mono_time_ns);
        if Duration::from_nanos(s.mono_time_ns.saturating_sub(since)) < protocol.settle {
            continue;
        }
        for j in 0..NQ {
            lo[j] = lo[j].min(s.angles[j]);
            hi[j] = hi[j].max(s.angles[j]);
        }
        if (0..NQ).any(|j| hi[j] - lo[j] > REST_DRIFT_DEG) {
            stable_since = None;
            taken = 0;
            sample = GravitySample {
                q: [0.0; NQ],
                tau: [0.0; NQ],
            };
            lo.fill(f64::INFINITY);
            hi.fill(f64::NEG_INFINITY);
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
        move_to(
            client,
            &approach_pose(q, dir * protocol.approach_rad),
            protocol,
        )
        .await?;
        // Keep position feedback active from the final approach through
        // sampling. Completing a queued move first enters torque-only
        // IDLE, which loses the approach's friction history and lets a
        // biased feedforward move the arm before the hold starts.
        let s = stopped(client, read_held(client, q, protocol).await).await?;
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
