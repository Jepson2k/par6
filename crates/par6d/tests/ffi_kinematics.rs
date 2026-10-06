//! Kinematics-backed runtime (feature `ffi`), driven end-to-end through
//! the real protocol-v2 encoding over UDP against `par6d --sim`:
//!
//! - the gravity hook is wired end to end: on the torque-level sim plant
//!   an IDLE arm holds its pose only while G(q) is fed forward,
//! - the gravity model reads the gripper CONFIG: changing the active
//!   tool's `[kinematics] mass_kg` changes the published gravity
//!   torques,
//! - the FK hook publishes the true TCP pose: STATUS reproduces the
//!   engine's own FK matrix for a known q,
//! - `move_l` runs the cartesian pipeline (segment → seeded IK → TOPPRA
//!   → ring) to COMPLETE, and the measured TCP stays on the line,
//! - an out-of-workspace pose is a real IK error reply, never a no-op,
//! - the collision world is enforced: a planned move through a keep-out
//!   is refused before dispatch, STATUS reports the live verdict, and a
//!   malformed shape set changes neither the epoch nor the enforced
//!   world.

use std::path::PathBuf;
use std::time::{Duration, Instant};

use par6_proto::command::{
    JogJ, JogL, MoveC, MoveJ, MoveJPose, MoveL, MoveP, MoveS, SelectTool, SetPayload, SetShapes,
    SetTcpOffset, Shape, Stop, Teleport,
};
use par6_proto::{Command, ControllerMode, ErrorCode, Frame, QueryResult, Status, NUM_JOINTS};

use par6d::options::StatusTransport;
use par6d::{Daemon, Options};

mod common;
use common::{
    path_misses, process_corner, shipped_config, spline_waypoints, Client, Rig, ARC_RADIUS_MM,
    BUDGET,
};

/// Boot on a config patched for this test's `tag`, so parallel tests do
/// not share a temp config directory.
fn boot_tagged(tag: &str) -> Rig {
    Rig::boot_with(test_config(tag))
}

/// The PAR6 config re-ticked to 50 Hz, like the sim-session test: loaded
/// CI machines without RT scheduling miss 4 ms deadlines and would latch
/// LOOP_CRITICAL. Every RT time constant derives from config seconds, so
/// the wiring under test is identical.
fn test_config(tag: &str) -> PathBuf {
    common::retimed_config(&format!("ffi-{tag}"), TEST_TICK_DT_S)
}

/// The tick period every rig in this file boots at. Anything that has to
/// agree with the runtime's own tick-derived arithmetic — the streaming
/// gate's stopping projection, for one — has to read THIS, not the
/// shipped config's period.
const TEST_TICK_DT_S: f64 = 0.02;

/// [`test_config`] with the active (MSG) gripper's `[kinematics] mass_kg`
/// replaced — the knob the gravity-wiring test turns.
fn test_config_with_tool_mass(tag: &str, mass_kg: f64) -> PathBuf {
    let dst = test_config(tag);
    // Whichever tool the config selects: pinning one variant's file name
    // patches a file the run never loads once the shipped tool changes,
    // and the test then compares two identical arms.
    let robot = std::fs::read_to_string(&dst).expect("robot toml");
    let name = robot
        .lines()
        .find_map(|line| line.trim_start().strip_prefix("active_tool"))
        .and_then(|rest| rest.split('"').nth(1))
        .expect("the config names an active gripper");
    let toml = dst.parent().unwrap().join(format!("grippers/{name}.toml"));
    let text = std::fs::read_to_string(&toml).expect("gripper toml");
    assert!(
        text.lines().any(|l| l.trim_start().starts_with("mass_kg")),
        "{name} has no mass_kg to patch"
    );
    let patched: String = text
        .split_inclusive('\n')
        .map(|line| {
            if line.trim_start().starts_with("mass_kg") {
                format!("mass_kg = {mass_kg}\n")
            } else {
                line.to_owned()
            }
        })
        .collect();
    std::fs::write(&toml, patched).expect("write gripper toml");
    dst
}

// ---- in-process rig --------------------------------------------------------

fn enable_and_teleport(rig: &Rig, c: &mut Client, angles_deg: [f64; NUM_JOINTS]) {
    let deadline = Instant::now() + BUDGET;
    loop {
        c.send(&Command::Teleport(Teleport {
            angles: angles_deg,
            tool_positions: None,
        }));
        let window = Instant::now() + Duration::from_secs(3);
        while Instant::now() < window {
            if let Some(s) = rig.recv_status() {
                let close = s
                    .angles
                    .iter()
                    .zip(angles_deg.iter())
                    .all(|(a, b)| (a - b).abs() < 1.0);
                if s.homed && close {
                    return;
                }
            }
        }
        assert!(
            Instant::now() < deadline,
            "teleport did not take effect within budget"
        );
    }
}

// ---- reference FK ----------------------------------------------------------

struct ReferenceCase {
    q: [f64; NUM_JOINTS],
    /// Row-major 4x4 TCP pose \[m\] from `par6_kin::Kin` on the same URDF
    /// variant the daemon loads for the configured gripper.
    fk: [f64; 16],
}

/// A configuration inside every hard joint window with its TCP pose as
/// the engine computes it in-process: what the daemon's STATUS must
/// carry once the arm is teleported there.
fn reference_case() -> ReferenceCase {
    let bundle = par6_config::ConfigBundle::load(&shipped_config()).expect("PAR6 config");
    let gripper = bundle.robot.robot.active_tool.trim().to_ascii_uppercase();
    let variant = par6_kin::GripperVariant::resolve(
        &gripper,
        bundle.active_tool().and_then(|g| g.urdf_variant.as_deref()),
    );
    let mut kin = par6_kin::Kin::load(&common::assets_dir(), variant).expect("reference model");
    let mut q = [0.0; NUM_JOINTS];
    for (out, deg) in q.iter_mut().zip(HOLD_POSE_DEG.iter()) {
        *out = deg.to_radians();
    }
    for (v, j) in q.iter().zip(bundle.robot.joints.iter()) {
        assert!(
            *v >= j.limits.hard_min_rad && *v <= j.limits.hard_max_rad,
            "the reference pose must sit inside the hard joint window"
        );
    }
    let mut fk = [0.0; 16];
    kin.fk(&q, &mut fk).expect("reference FK");
    ReferenceCase { q, fk }
}

// ---- cartesian geometry helpers --------------------------------------------

fn tcp_mm(s: &Status) -> [f64; 3] {
    [s.pose[3], s.pose[7], s.pose[11]]
}

/// Distance \[mm\] from `p` to the segment `a`→`b`.
fn distance_to_segment(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let len2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    let w = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let t = ((w[0] * d[0] + w[1] * d[1] + w[2] * d[2]) / len2).clamp(0.0, 1.0);
    let e = [w[0] - t * d[0], w[1] - t * d[1], w[2] - t * d[2]];
    (e[0] * e[0] + e[1] * e[1] + e[2] * e[2]).sqrt()
}

/// Euclidean distance \[mm\] between two TCP positions.
fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

/// Fraction of the segment `a`→`b` covered by `p`'s projection.
fn progress_along(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let len2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    let w = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    (w[0] * d[0] + w[1] * d[1] + w[2] * d[2]) / len2
}

/// Wire pose `[x y z mm, rx ry rz deg]` from a STATUS pose matrix
/// (row-major 4x4, mm) with the translation replaced.
///
/// Decoded the way a client decodes it — the wire's intrinsic-XYZ
/// convention, written out here
/// rather than borrowed from the runtime so the two halves of the
/// round trip cannot agree on the wrong thing.
fn wire_pose_at(pose: &[f64; 16], xyz_mm: [f64; 3]) -> [f64; 6] {
    let (r00, r01, r02) = (pose[0], pose[1], pose[2]);
    let (r12, r22) = (pose[6], pose[10]);
    let cp = r12.hypot(r22);
    [
        xyz_mm[0],
        xyz_mm[1],
        xyz_mm[2],
        (-r12).atan2(r22).to_degrees(),
        r02.atan2(cp).to_degrees(),
        (-r01).atan2(r00).to_degrees(),
    ]
}

/// Largest absolute difference between the rotation blocks of two STATUS
/// pose matrices — the orientation held (or not) across a move.
fn rotation_drift(a: &[f64; 16], b: &[f64; 16]) -> f64 {
    (0..12)
        .filter(|i| i % 4 != 3)
        .map(|i| (a[i] - b[i]).abs())
        .fold(0.0f64, f64::max)
}

/// A well-conditioned start posture for cartesian moves: away from the
/// wrist-aligned park singularity, comfortably inside every soft window,
/// extended clear of the arm's own collision meshes (TCP 0.46 m out from
/// the base axis), and with straight-line room for the moves below plus
/// a 1.3x margin along the same ray (verified by sweeping the soft-limit
/// box with seeded IK when the URDF was re-based, issue #24).
/// The next STATUS after the arm has stopped moving: half a second of
/// consecutive reports within a hundredth of a degree on every joint.
/// The plant settles into its stiction band after a teleport; reading a
/// pose mid-settle puts millimetres of arm motion into what should be a
/// pure kinematics comparison.
fn wait_still(rig: &Rig) -> Status {
    let deadline = Instant::now() + BUDGET;
    let mut prev: Option<Status> = None;
    let mut still_since = Instant::now();
    loop {
        let s = rig.wait_status("a status while settling", |_| true);
        if let Some(p) = &prev {
            let still = s
                .angles
                .iter()
                .zip(p.angles.iter())
                .all(|(a, b)| (a - b).abs() < 0.01);
            if !still {
                still_since = Instant::now();
            } else if still_since.elapsed() > Duration::from_millis(500) {
                return s;
            }
        }
        prev = Some(s);
        assert!(Instant::now() < deadline, "the arm never came to rest");
    }
}

const CART_START_DEG: [f64; NUM_JOINTS] = [-115.0, -40.0, 200.0, 0.0, 60.0, 180.0];

/// Hold posture for the torque-plant gravity tests: near-vertical, so
/// every loaded joint's G(q) sits well inside its current authority —
/// the hold runs on feedforward alone, and at an outstretched pose the
/// shoulder's load exceeds what its current limit can carry and the arm
/// sags off the pose.
const HOLD_POSE_DEG: [f64; NUM_JOINTS] = [0.0, -75.0, 305.0, 20.0, -30.0, 180.0];
/// Cartesian move duration \[s\]. Long enough that the sim's cascade
/// tracking lag stays small next to the path tolerances: the lag is
/// proportional to speed, and the line below is held to 8 mm over a
/// 180 mm move.
const MOVE_S: f64 = 15.0;

// ---- tests -----------------------------------------------------------------

/// The whole cartesian surface over one session: the FK hook publishes
/// the reference TCP pose, `move_l` holds the straight line where a
/// joint-space `move_j_pose` to the same target bows far off it,
/// `jog_l` drives the TCP through the jacobian, and an out-of-workspace
/// target fails both cartesian moves with IK_TARGET_UNREACHABLE instead
/// of moving the arm.
#[test]
fn cartesian_surface_over_protocol_v2() {
    let rig = boot_tagged("cart");
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    // --- FK hook: STATUS carries the engine's TCP pose for a known q.
    let case = reference_case();
    let mut case_deg = [0.0; NUM_JOINTS];
    for (out, rad) in case_deg.iter_mut().zip(case.q.iter()) {
        *out = rad.to_degrees();
    }
    enable_and_teleport(&rig, &mut c, case_deg);
    // The arm reports through 14-bit encoders, so it lands within a
    // quantum (~2e-5 rad) of the commanded configuration, not on it.
    let s = rig.wait_status("pose for the reference configuration", |s| {
        s.angles
            .iter()
            .zip(case_deg.iter())
            .all(|(a, b)| (a - b).abs() < 0.01)
    });
    for (k, reference) in case.fk.iter().enumerate() {
        // Tolerances leave ~100x margin over that quantum, and are still
        // orders of magnitude below what any convention slip (frame, row
        // order, rpy composition) would cost.
        // Columns 3/7/11 are the translation (reference in m, wire in mm).
        let (want, tol) = if k % 4 == 3 && k < 12 {
            (reference * 1000.0, 0.05)
        } else {
            (*reference, 5e-4)
        };
        assert!(
            (s.pose[k] - want).abs() < tol,
            "STATUS pose element {k} = {} != reference FK {want} (whole matrix {:?})",
            s.pose[k],
            s.pose
        );
    }

    // --- move_l: the measured TCP stays on the commanded line.
    enable_and_teleport(&rig, &mut c, CART_START_DEG);
    let s = rig.wait_status("start pose", |_| true);
    let start = tcp_mm(&s);
    // Out of the arm's plane in all three axes: the joint-space route to
    // the same pose then bows tens of millimetres off the line, which is
    // what makes the collinearity bound below a measurement instead of a
    // truism.
    let target = [start[0] + 120.0, start[1] + 60.0, start[2] + 120.0];
    let wire_target = wire_pose_at(&s.pose, target);
    let move_l = Command::MoveL(MoveL {
        key: 1001,
        pose: wire_target,
        frame: Frame::Wrf,
        duration: Some(MOVE_S),
        speed: None,
        accel: None,
        blend_radius: None,
        rel: false,
    });
    let i = c.ok_index(&move_l);
    let frames = rig.collect_status(Duration::from_secs_f64(MOVE_S + 1.0));
    let path: Vec<[f64; 3]> = frames.iter().map(tcp_mm).collect();
    let (ok, detail) = c.wait_complete(i);
    assert!(ok, "move_l must complete ok, got {detail:?}");
    assert!(
        !frames.iter().any(near_singular),
        "a line through a healthy region crossed the shipped singularity thresholds"
    );

    let moving: Vec<[f64; 3]> = path
        .into_iter()
        .filter(|p| progress_along(*p, start, target) > 0.05)
        .collect();
    assert!(
        moving.len() > 20,
        "expected a sampled trajectory, got {} moving samples",
        moving.len()
    );
    let line_dev = moving
        .iter()
        .map(|p| distance_to_segment(*p, start, target))
        .fold(0.0f64, f64::max);
    let reach = moving
        .iter()
        .map(|p| progress_along(*p, start, target))
        .fold(0.0f64, f64::max);
    assert!(
        line_dev < 8.0,
        "move_l left the commanded line by {line_dev:.2} mm"
    );
    assert!(
        reach > 0.8,
        "move_l covered only {:.0}% of the segment",
        reach * 100.0
    );
    // The commanded target carries the start orientation, decoded out of
    // STATUS the way a client decodes it and handed straight back, so the
    // arm has to finish pointing where it started: a runtime that
    // rebuilds those three numbers in the other order (`Rz·Ry·Rx`, the
    // URDF `rpy` reading) turns the wrist 36.7° at this posture on its
    // way to a target the operator never asked for. The 0.1 bound is ~6°
    // of rotation-block error — room for the cascade's settle lag, an
    // order of magnitude under the 0.56 the swapped order costs.
    let landed = settled_tcp(&rig, "the pose move_l finished at");
    let rot_drift = rotation_drift(&s.pose, &landed.pose);
    assert!(
        rot_drift < 0.1,
        "move_l changed the orientation it was told to hold (rotation \
         block off by {rot_drift:.3}): commanded {wire_target:?}, started \
         {:?}, finished {:?}",
        s.pose,
        landed.pose
    );

    // --- move_j_pose: same target through IK + the joint-space profile.
    // Its TCP path bows far off the line — which is what makes the
    // collinearity bound above a real measurement and not a truism.
    enable_and_teleport(&rig, &mut c, CART_START_DEG);
    let i = c.ok_index(&Command::MoveJPose(MoveJPose {
        key: 1002,
        pose: wire_target,
        duration: Some(MOVE_S),
        speed: None,
        accel: None,
        blend_radius: None,
    }));
    let joint_path: Vec<[f64; 3]> = rig
        .collect_status(Duration::from_secs_f64(MOVE_S + 1.0))
        .iter()
        .map(tcp_mm)
        .collect();
    let (ok, detail) = c.wait_complete(i);
    assert!(ok, "move_j_pose must complete ok, got {detail:?}");
    let joint_dev = joint_path
        .iter()
        .map(|p| distance_to_segment(*p, start, target))
        .fold(0.0f64, f64::max);
    assert!(
        joint_dev > 12.0,
        "the joint-space route to the same pose bowed only {joint_dev:.2} mm — \
         the move_l line tolerance proves nothing at this scale"
    );
    let end = rig.wait_status("settled after move_j_pose", |_| true);
    let reached = tcp_mm(&end);
    let miss = distance(reached, target);
    assert!(
        miss < 15.0,
        "move_j_pose IK target missed by {miss:.1} mm (reached {reached:?}, target {target:?})"
    );
    // The seeded-IK entry point reads the target's rotation through the
    // same decode as move_l's, so it holds the orientation too.
    let after = settled_tcp(&rig, "settled after move_j_pose");
    let rot_drift = rotation_drift(&s.pose, &after.pose);
    assert!(
        rot_drift < 0.1,
        "move_j_pose solved for a different orientation than it was given \
         (rotation block off by {rot_drift:.3})"
    );

    // --- jog_l: cartesian velocity streaming through the jacobian.
    enable_and_teleport(&rig, &mut c, CART_START_DEG);
    let before = tcp_mm(&rig.wait_status("pose before jog_l", |_| true));
    for _ in 0..6 {
        c.send(&Command::JogL(JogL {
            velocities: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            duration: 0.4,
            frame: Frame::Wrf,
            accel: None,
        }));
        std::thread::sleep(Duration::from_millis(50)); // client-side stream pacing
    }
    let jogged = rig.wait_status("jog_l drives the TCP along +x", |s| {
        tcp_mm(s)[0] > before[0] + 5.0
    });
    let drift = tcp_mm(&jogged);
    assert!(
        (drift[1] - before[1]).abs() < 8.0 && (drift[2] - before[2]).abs() < 8.0,
        "jog_l on +x alone moved the TCP off-axis: {before:?} -> {drift:?}"
    );
    rig.wait_status("jog_l self-terminates", |s| {
        s.speeds.iter().all(|v| v.abs() < 0.05)
    });

    // --- unreachable target: a real IK error on both cartesian moves,
    // and the arm does not move.
    enable_and_teleport(&rig, &mut c, CART_START_DEG);
    let unreachable = [2000.0, 0.0, 200.0, 0.0, 0.0, 0.0];
    let before = tcp_mm(&rig.wait_status("pose before the unreachable target", |_| true));
    for (label, cmd) in [
        (
            "move_j_pose",
            Command::MoveJPose(MoveJPose {
                key: 1003,
                pose: unreachable,
                duration: Some(1.0),
                speed: None,
                accel: None,
                blend_radius: None,
            }),
        ),
        (
            "move_l",
            Command::MoveL(MoveL {
                key: 1004,
                pose: unreachable,
                frame: Frame::Wrf,
                duration: Some(1.0),
                speed: None,
                accel: None,
                blend_radius: None,
                rel: false,
            }),
        ),
    ] {
        let i = c.ok_index(&cmd);
        let (ok, detail) = c.wait_complete(i);
        assert!(!ok, "{label} to an unreachable pose must fail");
        let e = detail.expect("a failed COMPLETE carries the error");
        assert_eq!(
            e.code,
            ErrorCode::IkTargetUnreachable as u16,
            "{label} must report IK_TARGET_UNREACHABLE, got {e:?}"
        );
    }
    let after = tcp_mm(&rig.wait_status("pose after the rejected targets", |_| true));
    for k in 0..3 {
        assert!(
            (after[k] - before[k]).abs() < 1.0,
            "a rejected cartesian target moved the arm: {before:?} -> {after:?}"
        );
    }

    rig.shutdown();
}

/// The runtime model's G(q) for the tick a frame describes: STATUS
/// carries the measured torque and the external estimate, measured minus
/// G(q) from the same tick.
fn model_g(s: &Status) -> [f64; NUM_JOINTS] {
    std::array::from_fn(|j| s.torques[j] - s.torques_ext[j])
}

/// The gravity hook is wired, signed right and scaled right, end to end.
///
/// With comp on — the simulator's default, now that the plant has weight
/// to cancel — an IDLE arm holds the pose it was placed at: the
/// feedforward carries the arm and the gearbox carries the remainder. A
/// wrong-signed hook drives every loaded joint down instead, and a
/// missing one lets the arm sag until the drivetrain's holding friction
/// catches it.
///
/// The measured torques STATUS reports are the plant's, so they pin the
/// scale as well as the sign: a shoulder holding an outstretched arm
/// carries several Nm and the wrist carries nearly none. The model's own
/// G(q) is pinned against the vendor table in `par6-kin`, and the
/// scene's against the same table in par6-bus, so this is about the
/// wiring rather than the physics.
#[test]
fn gravity_hook_holds_the_arm() {
    /// Hold tolerance [deg] for every joint.
    const HOLD_TOL: f64 = 2.5;
    let rig = boot_tagged("gravity");
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);
    enable_and_teleport(&rig, &mut c, CART_START_DEG);
    let held = wait_still(&rig);
    assert!(
        held.gravity_comp,
        "the simulator applies the feedforward: its plant has the weight to cancel"
    );
    // Held where it was placed: a hook that does not carry the arm lets
    // it sag until the gearbox friction catches it, and the still pose
    // is then wherever that was.
    for (i, (now, placed)) in held.angles.iter().zip(CART_START_DEG).enumerate() {
        assert!(
            (now - placed).abs() < HOLD_TOL,
            "joint {i} gave way on release: placed at {placed:.2}, still at {now:.2} deg"
        );
    }

    // Hold: three seconds of the runtime's IDLE must not move the arm.
    let until = held.mono_time_ns + 3_000_000_000;
    loop {
        let s = rig.wait_status("a status while holding", |_| true);
        for (i, (now, then)) in s.angles.iter().zip(held.angles.iter()).enumerate() {
            assert!(
                (now - then).abs() < HOLD_TOL,
                "joint {i} left its pose while idle: {then:.2} -> {now:.2} deg \
                 (all {:?} vs {:?})",
                s.angles,
                held.angles
            );
        }
        if s.mono_time_ns >= until {
            break;
        }
    }

    // Sign and scale: IDLE's law is torque-only, so what the drives
    // deliver IS the hook's output, and it must be the model's G(q) at the
    // held pose — the external estimate (measured minus model) is what a
    // mis-signed or mis-scaled hook leaves over.
    let s = wait_still(&rig);
    let g = model_g(&s);
    assert!(
        g[1].abs() > 1.0,
        "the shoulder carries the arm's weight here, the model says {:.3} Nm",
        g[1]
    );
    for (j, (t, gj)) in s.torques.iter().zip(g.iter()).enumerate() {
        assert!(
            (t - gj).abs() <= 0.02 * gj.abs() + 0.02,
            "J{j}: the drives deliver {t:.4} Nm, the model's G(q) is {gj:.4} Nm"
        );
    }

    rig.shutdown();
}

#[test]
fn a_full_speed_move_lands_cleanly() {
    // The planner's torque feedforward (M·q̈ + C·q̇ per sample, G(q) added
    // by the law) is applied for real on this tier: the plant integrates
    // rigid-body dynamics from the commanded current. This pins the tier
    // staying stable under a full-speed swing with the feedforward
    // riding the current channel — the FEEDFORWARD VALUES are pinned by
    // the ABA round-trip in par6-kin and the qdd-consistency tests in
    // par6-motion, because on an arm this small the drives' Ilim clamp
    // and the position loop absorb even a wildly wrong feedforward. The
    // swing stays on the gravity-free axes (base and wrist): the
    // shoulder lift saturates this plant's drive current against
    // gravity, feedforward or not.
    let rig = boot_tagged("tauff");
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);

    c.ok(&Command::Reset);
    enable_and_teleport(&rig, &mut c, HOLD_POSE_DEG);

    let mut target = HOLD_POSE_DEG;
    target[0] += 40.0;
    target[4] += 20.0;
    target[5] -= 30.0;
    let i = c.ok_index(&Command::MoveJ(MoveJ {
        key: 9301,
        angles: target,
        duration: None,
        speed: Some(1.0),
        accel: None,
        blend_radius: None,
        rel: false,
    }));
    let (ok, detail) = c.wait_complete(i);
    assert!(ok, "the full-speed move must complete: {detail:?}");
    // Landed, measured once the arm is still rather than on the first
    // frame that happens to pass: the plant settles into its stiction
    // band, so the tolerance is the band, not the servo's null — a
    // feedforward with the wrong sign or scale misses by tens of degrees
    // or latches an error, and a plan aimed elsewhere lands elsewhere.
    let s = wait_still(&rig);
    assert!(
        angles_close(&s.angles, &target, 2.0),
        "the move came to rest at {:?}, not on {target:?}",
        s.angles
    );
    assert!(
        s.error.is_none(),
        "standing error after the move: {:?}",
        s.error
    );

    rig.shutdown();
}

fn near_singular(s: &Status) -> bool {
    s.warnings
        .iter()
        .any(|w| w.code == ErrorCode::TrajNearSingularity as u16)
}

/// The near-singularity warning reads the config's thresholds and stands
/// while the path that crossed them runs. A condition limit of 1, which
/// every jacobian exceeds, flags a line through the same healthy region
/// the shipped thresholds pass; the move still runs, and the warning
/// clears once it lands.
#[test]
fn a_path_past_the_singularity_thresholds_warns_while_it_runs() {
    let config = test_config("singular");
    let text = std::fs::read_to_string(&config).expect("read test config");
    let patched = text.replace(
        "singularity_cond_max = 1000.0",
        "singularity_cond_max = 1.0",
    );
    assert_ne!(patched, text, "singularity_cond_max patch point must exist");
    std::fs::write(&config, patched).expect("write singular config");
    let rig = Rig::boot_with(config);
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);
    enable_and_teleport(&rig, &mut c, CART_START_DEG);
    let s = rig.wait_status("start pose", |_| true);
    assert!(!near_singular(&s), "nothing has run yet");
    let start = tcp_mm(&s);
    let target = [start[0] + 60.0, start[1], start[2] + 60.0];
    let i = c.ok_index(&move_l_to(1101, wire_pose_at(&s.pose, target), MOVE_S));
    let warned = rig.wait_status("the warning while the path runs", near_singular);
    assert!(
        distance(tcp_mm(&warned), target) > 1.0,
        "the warning must stand while the move is still under way"
    );
    let (ok, detail) = c.wait_complete(i);
    assert!(ok, "a warning, not a refusal: {detail:?}");
    rig.wait_status("the warning clears once the path lands", |s| {
        !near_singular(s)
    });
    rig.shutdown();
}

// ---- gravity reads the gripper config --------------------------------------

/// The gravity model reads the gripper CONFIG, not just the URDF, and
/// carries a declared payload whole.
///
/// Boot plain `--sim` twice; the only difference is the active gripper's
/// `[kinematics] mass_kg` (0.37 kg stock vs 2.37 kg — a tool two kilos
/// heavier). At the same posture the model's G(q) must shift by the extra
/// tool weight: about 6.7 Nm at the shoulder and 0.3 Nm at the wrist
/// pitch for these numbers. On the stock arm a SET_PAYLOAD with an
/// offset centre of mass and an inertia then has to land in the model as
/// declared, every term of it, and read back as declared.
///
/// STATUS carries the filtered measured torque and `torques_ext` = that
/// minus G(q) from the same tick, so their difference is the model's own
/// G(q) whatever the plant is doing.
#[test]
fn gripper_config_mass_changes_published_gravity_torque() {
    let payload = SetPayload {
        mass: 1.2,
        com: [0.02, -0.01, 0.06],
        inertia: Some([2e-3, 1e-4, 3e-3, -2e-4, 1e-4, 2.5e-3]),
    };
    let run = |tag: &str, mass_kg: f64, declare: bool| {
        let config = test_config_with_tool_mass(tag, mass_kg);
        let rig = Rig::boot_with(config.clone());
        let mut c = Client::new(rig.addr());
        rig.wait_status("link_ok", |s| s.link_ok == 1);
        c.ok(&Command::Reset);
        enable_and_teleport(&rig, &mut c, CART_START_DEG);
        // G(q) is exact on any tick; the heavy tool's wrist cannot hold
        // this posture for long, so it is read on landing.
        let at = rig.wait_status("at the probe posture", |s| {
            angles_close(&s.angles, &CART_START_DEG, 0.1)
        });
        let loaded = declare.then(|| {
            c.ok(&Command::SetPayload(payload.clone()));
            let readback = match c.query(&Command::Payload) {
                QueryResult::Payload { mass, com, inertia } => (mass, com, inertia),
                other => panic!("unexpected {other:?}"),
            };
            let s = rig.wait_status("the payload reaches the model", |s| {
                (model_g(s)[1] - model_g(&at)[1]).abs() > 1.0
            });
            (config, readback, s)
        });
        rig.shutdown();
        (at, loaded)
    };

    let (stock, loaded) = run("grav-stock", 0.37, true);
    let (heavy, _) = run("grav-heavy", 2.37, false);
    let (q_stock, g_stock) = (stock.angles, model_g(&stock));
    let (q_heavy, g_heavy) = (heavy.angles, model_g(&heavy));
    assert!(
        angles_close(&q_stock, &q_heavy, 0.2),
        "the two runs must be compared at the same posture: {q_stock:?} vs {q_heavy:?}"
    );

    // Plain --sim runs the real model, not placeholder zeros: the
    // shoulder carries most of the arm at this posture.
    assert!(
        g_stock[1].abs() > 2.0,
        "the shoulder's model gravity torque is {:.3} Nm — the runtime is \
         running a placeholder, not G(q)",
        g_stock[1]
    );
    // The config knob reaches the model, at the joints the extra tool
    // mass actually loads.
    let d_shoulder = (g_heavy[1] - g_stock[1]).abs();
    let d_wrist = (g_heavy[4] - g_stock[4]).abs();
    assert!(
        d_shoulder > 3.0,
        "2 kg more tool mass moved the shoulder gravity torque by only \
         {d_shoulder:.3} Nm (expected ~6.7): [kinematics] mass_kg does not \
         reach the gravity model"
    );
    assert!(
        d_wrist > 0.1,
        "2 kg more tool mass moved the wrist gravity torque by only \
         {d_wrist:.3} Nm (expected ~0.3): the tool attaches to the wrong link \
         or not at all"
    );
    // J0's axis is vertical, so a value here means the tool was attached
    // in the wrong frame.
    assert!(
        g_heavy[0].abs() < 1e-6,
        "J0 is on the vertical axis and must carry no gravity torque, got {:.6}",
        g_heavy[0]
    );

    // The declared payload, every term, is what the runtime's model holds.
    let (config, readback, s) = loaded.expect("the stock run declared a payload");
    assert_eq!(
        readback,
        (
            payload.mass,
            payload.com,
            payload.inertia.expect("declared")
        ),
        "the payload reads back as declared"
    );
    let bundle = par6_config::ConfigBundle::load(&config).expect("test config");
    let mut kin = par6d::kin::load_gravity_kin(&common::assets_dir(), bundle.active_tool())
        .expect("gravity model");
    kin.set_gravity_correction(&bundle.robot.gravity_correction)
        .expect("gravity correction");
    kin.set_tool(payload.mass, payload.com, payload.inertia)
        .expect("payload");
    let mut want = [0.0; NUM_JOINTS];
    kin.gravity(&s.angles.map(f64::to_radians), &mut want)
        .expect("gravity");
    let got = model_g(&s);
    for j in 0..NUM_JOINTS {
        let want = want[j] * bundle.robot.gravity_scale[j];
        assert!(
            (got[j] - want).abs() < 1e-3,
            "J{j}: the runtime's model holds {:.4} Nm under the payload, the \
             declared payload gives {want:.4}",
            got[j]
        );
    }
}

// ---- collision enforcement -------------------------------------------------

/// Start of the base sweep the keep-out tests drive: the arm extended
/// (its own meshes clear of each other, unlike the folded park pose),
/// rotated back around J0 so the sweep's midpoint sits in open workspace
/// where a keep-out can be parked (midpoint TCP 0.52 m out, endpoints
/// 0.35 m clear of it).
const SWEEP_START_DEG: [f64; NUM_JOINTS] = [-40.0, -20.0, 235.0, 0.0, 15.0, 180.0];
/// J0 travel of the sweep \[deg\]; its midpoint is where the box goes.
const SWEEP_DEG: f64 = 80.0;
/// Sweep duration \[s\].
const SWEEP_S: f64 = 3.0;
/// Keep-out edge length \[m\]. Wide enough that the gripper cannot slip
/// past it between two checked configurations, small enough that the
/// sweep's endpoints stay well clear.
const KEEPOUT_M: f64 = 0.1;

fn with_j0(base: [f64; NUM_JOINTS], delta_deg: f64) -> [f64; NUM_JOINTS] {
    let mut a = base;
    a[0] += delta_deg;
    a
}

fn move_j(key: u64, angles_deg: [f64; NUM_JOINTS], duration_s: f64) -> Command {
    Command::MoveJ(MoveJ {
        key,
        angles: angles_deg,
        duration: Some(duration_s),
        speed: None,
        accel: None,
        blend_radius: None,
        rel: false,
    })
}

/// An axis-aligned cube keep-out centred on a TCP position read from
/// STATUS. Shapes are metres/radians on the wire (what waldoctl sends);
/// STATUS translations are mm.
fn keepout_at(name: &str, tcp_mm: [f64; 3]) -> Shape {
    Shape {
        attachment: None,
        kind: "box".to_owned(),
        params: vec![KEEPOUT_M, KEEPOUT_M, KEEPOUT_M],
        pose: vec![
            tcp_mm[0] / 1000.0,
            tcp_mm[1] / 1000.0,
            tcp_mm[2] / 1000.0,
            0.0,
            0.0,
            0.0,
        ],
        collision: true,
        margin: None,
        name: name.to_owned(),
        physics: None,
    }
}

fn set_shapes(shapes: Vec<Shape>) -> Command {
    Command::SetShapes(SetShapes { shapes })
}

/// The applied collision world as the SHAPES query reports it.
fn shapes_readback(c: &mut Client) -> (Vec<Shape>, u64) {
    match c.query(&Command::Shapes) {
        QueryResult::Shapes { program, epoch, .. } => (program, epoch),
        other => panic!("unexpected SHAPES result {other:?}"),
    }
}

/// The configured park pose in wire units — where every program ends.
fn park_deg() -> [f64; NUM_JOINTS] {
    common::park_deg()
}

fn angles_close(a: &[f64; NUM_JOINTS], b: &[f64; NUM_JOINTS], tol_deg: f64) -> bool {
    a.iter()
        .zip(b.iter())
        .all(|(x, y)| (x - y).abs() <= tol_deg)
}

/// Collision enforcement end to end, over the real protocol against the
/// real coal world:
///
/// - a `move_j` whose ENDPOINTS are both clear but whose interior sweeps
///   the gripper through a keep-out is refused before a sample reaches
///   the RT ring, with `SYS_SELF_COLLISION` and the colliding pair in the
///   payload — and the arm does not move;
/// - the same move runs to COMPLETE once the box is gone;
/// - STATUS carries that verdict (`collision_active` / `collision_pairs`)
///   until a motion is accepted, in the URDF's reporting vocabulary;
/// - a malformed shape refuses the WHOLE set: the epoch does not move,
///   the readback does not change, and the previous world stays
///   ENFORCED (not merely echoed);
/// - a keep-out dropped onto a RUNNING move stops it;
/// - the same move runs once the box is gone, and `reset_state` clears
///   the program layer.
#[test]
fn collision_world_is_enforced_over_protocol_v2() {
    let rig = boot_tagged("collision");
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    let mid_deg = with_j0(SWEEP_START_DEG, SWEEP_DEG / 2.0);
    let end_deg = with_j0(SWEEP_START_DEG, SWEEP_DEG);

    // Where the gripper passes halfway through the sweep: the keep-out
    // goes there, so both endpoints stay clear and only the interior of
    // the move is blocked.
    enable_and_teleport(&rig, &mut c, mid_deg);
    let mid_tcp =
        tcp_mm(&rig.wait_status("midpoint pose", |s| angles_close(&s.angles, &mid_deg, 0.5)));

    // Baseline: with an empty world the sweep runs to COMPLETE.
    enable_and_teleport(&rig, &mut c, SWEEP_START_DEG);
    let i = c.ok_index(&move_j(7001, end_deg, SWEEP_S));
    let (ok, detail) = c.wait_complete(i);
    assert!(ok, "the sweep must run with an empty world, got {detail:?}");

    // The keep-out, straddling the middle of that same sweep.
    enable_and_teleport(&rig, &mut c, SWEEP_START_DEG);
    let keepout = keepout_at("keepout", mid_tcp);
    c.ok(&set_shapes(vec![keepout.clone()]));
    let (program, epoch) = shapes_readback(&mut c);
    assert_eq!(program, vec![keepout.clone()]);
    assert!(epoch > 0, "an applied world must carry a non-zero epoch");

    // Both endpoints are clear — proven by STATUS at each of them — so
    // only checking the interior of the path can catch this move.
    rig.drain_status();
    let s = rig.wait_status("start of the sweep is clear", |s| {
        angles_close(&s.angles, &SWEEP_START_DEG, 0.5)
    });
    assert!(
        !s.collision_active,
        "the sweep start must be outside the keep-out: {:?}",
        s.collision_pairs
    );
    let i = c.ok_index(&move_j(7002, end_deg, SWEEP_S));
    let (ok, detail) = c.wait_complete(i);
    assert!(!ok, "a move sweeping through the keep-out must be refused");
    let e = detail.expect("a failed COMPLETE carries the error");
    assert_eq!(
        e.code,
        ErrorCode::SysSelfCollision as u16,
        "the refusal must be SYS_SELF_COLLISION, got {e:?}"
    );
    assert!(
        e.cause.contains("keepout"),
        "the error payload must name the colliding pair: {e:?}"
    );
    rig.drain_status();
    let s = rig.wait_status("pose after the refusal", |s| {
        s.action_state != par6_proto::ActionState::Executing
    });
    assert!(
        angles_close(&s.angles, &SWEEP_START_DEG, 1.0),
        "a refused move must not drive the arm: {:?}",
        s.angles
    );

    // STATUS carries the verdict of the refusal: the pairs the blocked
    // move would have collided in, in waldoctl's reporting vocabulary —
    // `shape:<name>` for a program keep-out, a bare URDF link name for
    // arm geometry, never the solver's per-link geometry identifiers.
    rig.drain_status();
    let s = rig.wait_status("the refusal reaches STATUS", |s| s.collision_active);
    let pair = s
        .collision_pairs
        .iter()
        .find(|(a, b)| a == "shape:keepout" || b == "shape:keepout")
        .unwrap_or_else(|| {
            panic!(
                "collision_pairs must name the keep-out as a program shape: {:?}",
                s.collision_pairs
            )
        });
    let link = if pair.0 == "shape:keepout" {
        &pair.1
    } else {
        &pair.0
    };
    assert!(
        !link.ends_with("_0"),
        "the pair must name a URDF link, not a solver geometry id: {link}"
    );

    // A malformed set is refused WHOLE. Every flavour of malformed: a
    // kind waldoctl does not define, an arity that does not match the
    // kind, a dimension coal cannot build, a name the set repeats
    // anywhere in it, and a name the installation layer already uses.
    let mut unknown_kind = keepout.clone();
    unknown_kind.kind = "pyramid".to_owned();
    unknown_kind.name = "bad".to_owned();
    let mut short_params = keepout.clone();
    short_params.params = vec![KEEPOUT_M, KEEPOUT_M];
    short_params.name = "bad".to_owned();
    let mut negative = keepout.clone();
    negative.kind = "sphere".to_owned();
    negative.params = vec![-1.0];
    negative.name = "bad".to_owned();
    let mut between = keepout.clone();
    between.name = "between".to_owned();
    let mut floor = keepout.clone();
    floor.name = "floor".to_owned();
    for (label, set, named) in [
        ("unknown kind", vec![keepout.clone(), unknown_kind], ""),
        ("wrong arity", vec![keepout.clone(), short_params], ""),
        ("negative radius", vec![keepout.clone(), negative], ""),
        (
            "duplicate name",
            vec![keepout.clone(), between, keepout.clone()],
            "\"keepout\"",
        ),
        (
            "installation name",
            vec![keepout.clone(), floor.clone()],
            "\"floor\"",
        ),
    ] {
        let err = c.expect_error(&set_shapes(set));
        assert_eq!(
            err.code,
            ErrorCode::CommValidationError as u16,
            "a {label} shape must be refused, got {err:?}"
        );
        assert!(
            err.cause.contains(named),
            "the {label} refusal must name the shape: {}",
            err.cause
        );
        let (program, refused_epoch) = shapes_readback(&mut c);
        assert_eq!(
            refused_epoch, epoch,
            "a refused world must not advance scene_epoch ({label})"
        );
        assert_eq!(
            program,
            vec![keepout.clone()],
            "a refused set must not change the readback ({label})"
        );
    }
    // …and the previous world is still ENFORCED, not merely echoed: the
    // same move is still refused.
    let i = c.ok_index(&move_j(7003, end_deg, SWEEP_S));
    let (ok, detail) = c.wait_complete(i);
    assert!(!ok, "a refused SET_SHAPES dropped the enforced keep-out");
    assert_eq!(
        detail.expect("a failed COMPLETE carries the error").code,
        ErrorCode::SysSelfCollision as u16
    );

    // A visualization-only shape never appears in a colliding pair, so
    // the installation's name is free for it.
    floor.collision = false;
    c.ok(&set_shapes(vec![keepout.clone(), floor.clone()]));
    let (program, _) = shapes_readback(&mut c);
    assert_eq!(program, vec![keepout.clone(), floor]);

    // A world change does not spare motion already committed: drop the
    // keep-out onto the path of a move that is already running and it
    // stops, instead of being enforced only from the next command on.
    for pause in [false, true] {
        c.ok(&set_shapes(Vec::new()));
        enable_and_teleport(&rig, &mut c, SWEEP_START_DEG);
        let i = c.ok_index(&move_j(7004 + u64::from(pause) * 100, end_deg, SWEEP_S));
        rig.drain_status();
        rig.wait_status("the sweep is under way but short of the keep-out", |s| {
            s.executing_index == i as i64
                && s.angles[0] > SWEEP_START_DEG[0] + 3.0
                && s.angles[0] < -10.0
        });
        if pause {
            c.ok(&Command::Pause(par6_proto::command::Pause { on: true }));
            c.ok(&Command::SetExecutionSpeed(
                par6_proto::command::SetExecutionSpeed { scale: 0.5 },
            ));
            let deadline = std::time::Instant::now() + Duration::from_secs(5);
            loop {
                match c.query(&Command::ExecutionSpeed) {
                    QueryResult::ExecutionSpeed {
                        target_scale: 0.0,
                        applied_scale: 0.0,
                        resume_scale: 0.5,
                    } => break,
                    state => assert!(
                        std::time::Instant::now() < deadline,
                        "motion did not pause: {state:?}"
                    ),
                }
                rig.recv_status();
            }
            assert!(!c.peek_complete(i), "pause completed the unfinished move");
        }
        c.ok(&set_shapes(vec![keepout.clone()]));
        let (ok, detail) = c.wait_complete(i);
        assert!(!ok, "a keep-out dropped on a running move must stop it");
        let e = detail.expect("a failed COMPLETE carries the error");
        assert_eq!(
            e.code,
            ErrorCode::SysSelfCollision as u16,
            "the invalidated move must report SYS_SELF_COLLISION, got {e:?}"
        );
        rig.drain_status();
        let s = rig.wait_status("the arm stops", |s| s.speeds.iter().all(|v| v.abs() < 0.05));
        assert!(
            s.angles[0] < mid_deg[0],
            "the arm drove into the keep-out it was stopped for: {:?}",
            s.angles
        );

        c.ok(&Command::Pause(par6_proto::command::Pause { on: false }));
        c.ok(&Command::SetExecutionSpeed(
            par6_proto::command::SetExecutionSpeed { scale: 1.0 },
        ));
    }

    // Removing the keep-out advances the epoch and lets the very same
    // move through — and accepting it clears the latched verdict.
    c.ok(&set_shapes(Vec::new()));
    let (program, cleared_epoch) = shapes_readback(&mut c);
    assert!(program.is_empty());
    assert!(
        cleared_epoch > epoch,
        "clearing the world must advance the epoch: {cleared_epoch} vs {epoch}"
    );
    enable_and_teleport(&rig, &mut c, SWEEP_START_DEG);
    let i = c.ok_index(&move_j(7005, end_deg, SWEEP_S));
    // And a world change clear of the remaining path leaves the running
    // move alone: a runtime that failed every running move on ANY world
    // change would pass the loop above just the same.
    rig.drain_status();
    rig.wait_status("the sweep is under way", |s| {
        s.executing_index == i as i64 && s.angles[0] > SWEEP_START_DEG[0] + 3.0
    });
    c.ok(&set_shapes(vec![keepout_at("far", [900.0, 900.0, 900.0])]));
    let (program, _) = shapes_readback(&mut c);
    assert_eq!(program.len(), 1, "the far box must be applied, not ignored");
    let (ok, detail) = c.wait_complete(i);
    assert!(
        ok,
        "the sweep must run once the keep-out is removed, past a box clear of \
         its path, got {detail:?}"
    );
    let s = settled_tcp(&rig, "the arm at rest at the end of the sweep");
    assert!(
        angles_close(&s.angles, &end_deg, 1.0),
        "the move must run to its target: {:?}",
        s.angles
    );
    assert!(
        !s.collision_active && s.collision_pairs.is_empty() && s.error.is_none(),
        "a clean move clears the verdict and leaves none behind: active={} pairs={:?} \
         error={:?}",
        s.collision_active,
        s.collision_pairs,
        s.error
    );

    // reset_state clears the program layer: the readback empties and the
    // keep-out stops being enforced.
    c.ok(&set_shapes(vec![keepout]));
    enable_and_teleport(&rig, &mut c, SWEEP_START_DEG);
    let i = c.ok_index(&move_j(7006, end_deg, SWEEP_S));
    assert!(!c.wait_complete(i).0, "the keep-out must be back in force");
    c.ok(&Command::ResetState);
    let (program, _) = shapes_readback(&mut c);
    assert!(
        program.is_empty(),
        "reset_state must clear the program readback: {program:?}"
    );
    enable_and_teleport(&rig, &mut c, SWEEP_START_DEG);
    let i = c.ok_index(&move_j(7007, end_deg, SWEEP_S));
    let (ok, detail) = c.wait_complete(i);
    assert!(
        ok,
        "reset_state must stop enforcing the program layer, got {detail:?}"
    );

    // The arm must be able to return to its OWN park pose. PAR6 parks
    // folded, forearm back and resting against the base, which the vendor
    // collision meshes report as contact; if that counted as a collision
    // the last step of every program would be refused.
    let i = c.ok_index(&move_j(7008, park_deg(), SWEEP_S));
    let (ok, detail) = c.wait_complete(i);
    assert!(
        ok,
        "a move to the configured park pose must not be refused: {detail:?}"
    );

    rig.shutdown();
}

// ---- streaming collision gate ----------------------------------------------

fn jog_j(joint: usize, signed_pct: f64, duration_s: f64) -> Command {
    let mut speeds = [0.0; NUM_JOINTS];
    speeds[joint] = signed_pct;
    Command::JogJ(JogJ {
        speeds,
        duration: duration_s,
        accel: None,
    })
}

/// Signed distance from the arm's collision geometry to the keep-out at
/// `angles_deg` \[m\], through the same model the daemon gates on.
///
/// This is the only honest clearance number in this test. A flange-to-box
/// distance is not one: the bodies that get refused are `gripper` and
/// `jaw2`, which reach the box while the flange is still ~100 mm away,
/// so a flange measurement reports the tool's length as if it were the
/// gate's margin.
fn keepout_world(keepout_centre_m: [f64; 3]) -> par6_kin::Collision {
    let mut col = par6_kin::Collision::load(
        &common::assets_dir(),
        par6_kin::GripperVariant::Msg,
        par6d::COLLISION_CLEARANCE_M,
    )
    .expect("reference collision model");
    col.set_layer(
        par6_kin::Layer::Program,
        &[par6_kin::Shape {
            attachment: None,
            name: "keepout".to_owned(),
            kind: par6_kin::ShapeKind::Box,
            params: [KEEPOUT_M, KEEPOUT_M, KEEPOUT_M],
            pose: [
                keepout_centre_m[0],
                keepout_centre_m[1],
                keepout_centre_m[2],
                0.0,
                0.0,
                0.0,
            ],
            collision: true,
            margin: None,
        }],
    )
    .expect("keep-out into the reference world");
    col
}

/// Signed distance from the arm's collision geometry to the keep-out at
/// `angles_deg` \[m\], through a world built by [`keepout_world`].
fn world_gap_m(col: &mut par6_kin::Collision, angles_deg: [f64; NUM_JOINTS]) -> f64 {
    let mut q = [0.0; par6_kin::NQ];
    for (out, deg) in q.iter_mut().zip(angles_deg.iter()) {
        *out = deg.to_radians();
    }
    col.world_distance(&q).expect("world distance")
}

/// The TCP position at `angles_deg` \[m\], from the same URDF the runtime
/// loads — where a keep-out has to go to sit on the swept path.
fn tcp_at_m(angles_deg: [f64; NUM_JOINTS]) -> [f64; 3] {
    let mut kin = par6_kin::Kin::load(&common::assets_dir(), par6_kin::GripperVariant::Msg)
        .expect("kin model");
    let mut q = [0.0; NUM_JOINTS];
    for (out, deg) in q.iter_mut().zip(angles_deg.iter()) {
        *out = deg.to_radians();
    }
    let mut pose = [0.0; 16];
    kin.fk(&q, &mut pose).expect("fk");
    [pose[3], pose[7], pose[11]]
}

/// The J0 jog `speeds` fraction whose stop, held at that speed, is
/// `travel_rad` on — the inverse of [`par6d::held_jog_travel`], found by
/// bisection rather than by restating the gate's arithmetic here (a test
/// that recomputes the projection cannot catch it being wrong).
fn j0_speed_reaching(travel_rad: f64) -> f64 {
    let cfg = par6_config::RobotConfig::load(&shipped_config()).expect("PAR6 config");
    let lim = cfg.joints[0].limits.for_mode(par6_config::LimitMode::Jog);
    // The rig's period, not the shipped one: the reaction is counted in
    // ticks.
    let (v_max, dt) = (lim.velocity_rad_s, TEST_TICK_DT_S);
    let travel =
        |v: f64| par6d::held_jog_travel(v, v_max, lim.acceleration_rad_s2, &cfg.jog, 1.0, dt);
    let (mut lo, mut hi) = (0.0, v_max);
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        if travel(mid) < travel_rad {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    (0.5 * (lo + hi) / v_max).clamp(0.01, 1.0)
}

/// Streaming motion is gated by the same collision world as planned
/// motion (issue #19 gap 1 + gap 2), over the real protocol against the
/// real coal world and the real RT jog engine:
///
/// - a jog held TOWARD a keep-out, joint or cartesian, slow or at full
///   speed, comes to rest on the keep-out's clearance and stays there:
///   braked when the projection of its stop reaches the clearance, then
///   placed on it, with STATUS latching `collision_active` and the
///   keep-out named — never inside the clearance, never more than 5 mm
///   out of it;
/// - from INSIDE the keep-out (dropped over the arm), a jog moving
///   OUTWARD is permitted — the arm demonstrably escapes;
/// - from a SHALLOW penetration, a jog driving DEEPER is refused with a
///   real `SYS_SELF_COLLISION` ERROR reply (the escape-depth rule: same
///   pairs, deeper penetration — the pair-set check alone cannot catch
///   it), while the outward jog from the same spot runs.
#[test]
fn streaming_is_gated_by_the_collision_world() {
    let rig = boot_tagged("streamgate");
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    let mid_deg = with_j0(SWEEP_START_DEG, SWEEP_DEG / 2.0);
    let mid_m = tcp_at_m(mid_deg);
    // The J0 arc the TCP travels on: converts arc metres to J0 radians.
    let radius_m = (mid_m[0].powi(2) + mid_m[1].powi(2)).sqrt();
    let deg_per_m = 1.0_f64.to_degrees() / radius_m;

    let keepout = keepout_at("keepout", [mid_m[0] * 1e3, mid_m[1] * 1e3, mid_m[2] * 1e3]);
    c.ok(&set_shapes(vec![keepout.clone()]));

    // --- a jog held toward the keep-out comes to rest on its clearance
    // and stays there, at any speed, joint or cartesian. The clearance is
    // the stop the operator is owed. The placement is commanded a
    // milliradian short of it and accepted within two of that, so it may
    // finish half a millimetre inside at the gripper's reach, and a
    // millimetre covers its settle; resting more than 5 mm out of it is
    // ground the gate took from them — where the arm stopped before it
    // was placed.
    let clearance_mm = par6d::COLLISION_CLEARANCE_M * 1e3;
    let mut world = keepout_world(mid_m);
    let start_deg = with_j0(mid_deg, -4.0 * KEEPOUT_M * deg_per_m);
    let start_m = tcp_at_m(start_deg);
    let (dx, dy) = (mid_m[0] - start_m[0], mid_m[1] - start_m[1]);
    let toward = [dx / dx.hypot(dy), dy / dx.hypot(dy)];
    let jog_l = |velocities: [f64; 6]| {
        Command::JogL(JogL {
            velocities,
            duration: 0.2,
            frame: Frame::Wrf,
            accel: None,
        })
    };
    let toward_l = jog_l([toward[0], toward[1], 0.0, 0.0, 0.0, 0.0]);
    // Held, or let go the moment it is refused. Held, the refusal puts the
    // arm on the standoff; let go, it stops where its brake leaves it — never
    // inside the clearance, and with no approach nobody is asking for.
    let held = [
        ("jog_j at 10 %", jog_j(0, 0.1, 0.2), None),
        ("jog_j at full speed", jog_j(0, 1.0, 0.2), None),
        ("jog_l at full speed", toward_l.clone(), None),
        (
            "jog_j at full speed, let go when refused",
            jog_j(0, 1.0, 0.2),
            Some(jog_j(0, 0.0, 0.2)),
        ),
        (
            "jog_l at full speed, let go when refused",
            toward_l,
            Some(jog_l([0.0; 6])),
        ),
    ];
    for (what, jog, release) in &held {
        c.ok(&Command::Reset);
        enable_and_teleport(&rig, &mut c, start_deg);
        rig.drain_status();
        // Re-sent once a frame, as a held key is, and the replies drained.
        // Placed means at rest for two seconds still holding it, in
        // whichever mode the placement left the arm.
        let (mut closest, mut still_since, mut latched) = (f64::INFINITY, None, None);
        let (mut closest_at, mut closest_mode) = (0, ControllerMode::Idle);
        // Handed to IDLE while still moving: the drop out of a refusal's
        // control that lets the arm coast where it will. Above an encoder
        // count a tick, which on the base alone reads 0.056 rad/s.
        let mut coasting = None;
        // Where the refusal's brake first left it at rest.
        let mut brake_rest = None;
        let mut first_ns: Option<u64> = None;
        let deadline = Instant::now() + 2 * BUDGET;
        let s = loop {
            assert!(
                Instant::now() < deadline,
                "{what}: never came to rest on the clearance"
            );
            let Some(s) = rig.recv_status() else { continue };
            c.send(
                release
                    .as_ref()
                    .filter(|_| latched.is_some())
                    .unwrap_or(jog),
            );
            c.drain();
            let t0 = *first_ns.get_or_insert(s.mono_time_ns);
            let gap = world_gap_m(&mut world, s.angles) * 1e3;
            if gap < closest {
                (closest, closest_at, closest_mode) = (gap, s.mono_time_ns - t0, s.mode);
            }
            if latched.is_none() && s.collision_active {
                latched = Some(s.collision_pairs.clone());
            }
            if latched.is_some()
                && s.mode == ControllerMode::Idle
                && s.speeds.iter().any(|v| v.abs() > 0.1)
            {
                coasting.get_or_insert(s.speeds);
            }
            let still = latched.is_some() && s.speeds.iter().all(|v| v.abs() < 0.01);
            if still && brake_rest.is_none() {
                brake_rest = Some(gap);
            }
            still_since = if still {
                still_since.or(Some(s.mono_time_ns))
            } else {
                None
            };
            if still_since.is_some_and(|t| s.mono_time_ns - t >= 2_000_000_000) {
                break s;
            }
        };
        let pairs = latched.expect("latched before resting");
        assert!(
            pairs
                .iter()
                .any(|(a, b)| a == "shape:keepout" || b == "shape:keepout"),
            "{what}: the latched pairs must name the keep-out as a program shape: {pairs:?}"
        );
        let rest = world_gap_m(&mut world, s.angles) * 1e3;
        assert!(
            closest >= clearance_mm - 1.0,
            "{what}: came within {closest:.1} mm of the keep-out, inside its \
             {clearance_mm:.0} mm clearance, {:.2} s in, in {closest_mode:?}",
            closest_at as f64 * 1e-9
        );
        assert!(
            coasting.is_none(),
            "{what}: dropped into IDLE moving at {coasting:?} rad/s"
        );
        if release.is_none() {
            assert!(
                rest <= clearance_mm + 5.0,
                "{what}: rests {rest:.1} mm from the keep-out, more than 5 mm out of its \
                 {clearance_mm:.0} mm clearance"
            );
        } else {
            // Two millimetres for the drive settling onto where it stopped.
            assert!(
                rest - closest < 2.0,
                "{what}: came {closest:.1} mm from the keep-out and then back to {rest:.1} mm, \
                 rather than resting where its brake left it"
            );
            // Braked well short of the clearance, it was let go of long
            // before a placement could have put it there.
            if let Some(braked) = brake_rest.filter(|b| *b > clearance_mm + 50.0) {
                assert!(
                    rest > clearance_mm + 5.0,
                    "{what}: braked {braked:.1} mm from the keep-out and let go of, it was \
                     still carried on to the standoff, {rest:.1} mm from it"
                );
            }
        }
    }

    // --- the same layer re-sent all through a placement puts nothing in
    // its way, and does not hold it up.
    c.ok(&Command::Reset);
    enable_and_teleport(&rig, &mut c, start_deg);
    rig.drain_status();
    let jog = jog_j(0, 1.0, 0.2);
    let (mut latched, mut still_since) = (false, None);
    let deadline = Instant::now() + 2 * BUDGET;
    let s = loop {
        assert!(
            Instant::now() < deadline,
            "a placement under a re-sent layer never came to rest"
        );
        let Some(s) = rig.recv_status() else { continue };
        c.send(&jog);
        c.drain();
        latched |= s.collision_active;
        if latched {
            c.ok(&set_shapes(vec![keepout.clone()]));
        }
        still_since = if latched && s.speeds.iter().all(|v| v.abs() < 0.01) {
            still_since.or(Some(s.mono_time_ns))
        } else {
            None
        };
        if still_since.is_some_and(|t| s.mono_time_ns - t >= 2_000_000_000) {
            break s;
        }
    };
    let rest = world_gap_m(&mut world, s.angles) * 1e3;
    assert!(
        rest <= clearance_mm + 5.0,
        "under a re-sent layer the placement stopped {rest:.1} mm from the keep-out, more \
         than 5 mm out of its clearance"
    );

    // --- a keep-out dropped across a placement under way, nothing held:
    // the arm is stopped and put on the new keep-out's clearance, not
    // carried on through it toward the standoff it was solved for before.
    let wall_m = tcp_at_m(with_j0(mid_deg, -0.6 * KEEPOUT_M * deg_per_m));
    let wall = keepout_at("wall", [wall_m[0] * 1e3, wall_m[1] * 1e3, wall_m[2] * 1e3]);
    let mut wall_world = keepout_world(wall_m);
    c.ok(&Command::Reset);
    enable_and_teleport(&rig, &mut c, start_deg);
    rig.drain_status();
    let jog = jog_j(0, 1.0, 0.2);
    // Until the wall goes down, decided on the newest frame: this loop's
    // own queries trail the stream, and a stale frame would drop it late.
    let (mut latched, mut braked) = (false, false);
    let deadline = Instant::now() + 2 * BUDGET;
    loop {
        assert!(
            Instant::now() < deadline,
            "the refused jog never set off on its placement"
        );
        rig.drain_status();
        let Some(s) = rig.recv_status() else { continue };
        c.send(&jog);
        c.drain();
        latched |= s.collision_active;
        braked |= latched && s.speeds.iter().all(|v| v.abs() < 0.01);
        // Close enough that the placement's own gentle brake would carry
        // the arm into it, far enough that a full-rate stop does not.
        let gap = world_gap_m(&mut wall_world, s.angles) * 1e3;
        if braked && gap < 55.0 {
            assert!(
                gap > 35.0,
                "the premise: the wall goes down ahead of the arm, not {gap:.1} mm from it"
            );
            c.ok(&set_shapes(vec![keepout.clone(), wall.clone()]));
            break;
        }
    }
    let (mut closest, mut still_since, mut link_lost) = (f64::INFINITY, None, false);
    let s = loop {
        assert!(
            Instant::now() < deadline,
            "a placement under a dropped keep-out never came to rest"
        );
        let Some(s) = rig.recv_status() else { continue };
        c.drain();
        link_lost |= s
            .error
            .as_ref()
            .is_some_and(|e| e.code == ErrorCode::SysRtiLinkLost as u16);
        closest = closest.min(world_gap_m(&mut wall_world, s.angles) * 1e3);
        still_since = if s.speeds.iter().all(|v| v.abs() < 0.01) {
            still_since.or(Some(s.mono_time_ns))
        } else {
            None
        };
        if still_since.is_some_and(|t| s.mono_time_ns - t >= 2_000_000_000) {
            break s;
        }
    };
    let rest = world_gap_m(&mut wall_world, s.angles) * 1e3;
    assert!(
        !link_lost,
        "the placement went unfed and the RT latched a lost stream link"
    );
    assert!(
        closest >= clearance_mm - 1.0,
        "came within {closest:.1} mm of the keep-out dropped across its placement"
    );
    assert!(
        rest <= clearance_mm + 5.0,
        "rests {rest:.1} mm from the keep-out dropped across its placement, more than 5 mm \
         out of its clearance"
    );
    c.ok(&Command::Reset);
    c.ok(&set_shapes(vec![keepout.clone()]));

    // --- from inside the keep-out, an escaping jog is permitted, joint
    // or cartesian. Teleport into the box (a keep-out dropped over the
    // arm) and jog back out: refusing this would trap the arm, which is
    // exactly what the escape rule exists to prevent — and the refusal's
    // placement would take it out by the nearest side, whichever way it
    // was jogged.
    let away = [-0.3 * toward[0], -0.3 * toward[1], 0.0, 0.0, 0.0, 0.0];
    let escapes = [
        ("jog_j", jog_j(0, -0.3, 5.0)),
        (
            "jog_l",
            Command::JogL(JogL {
                velocities: away,
                duration: 5.0,
                frame: Frame::Wrf,
                accel: None,
            }),
        ),
    ];
    for (what, jog) in &escapes {
        enable_and_teleport(&rig, &mut c, mid_deg);
        rig.drain_status();
        c.send(jog);
        rig.wait_status(&format!("the escaping {what} moves the arm out"), |s| {
            s.angles[0] < mid_deg[0] - 3.0
        });
        // Stopped and at rest BEFORE the next teleport: its 5 s duration
        // outlives the wait above, and the release ramp of a live jog
        // would drag the freshly teleported pose back out of the box.
        c.ok(&Command::Stop(Stop { clear_queue: false }));
        rig.drain_status();
        rig.wait_status("the stopped escape jog comes to rest", |s| {
            s.speeds.iter().all(|v| v.abs() < 0.05)
        });
    }

    // --- from inside the clearance, short of the box, a jog fast enough
    // to stop beyond its far side is refused: its stop is clear, but the
    // way there goes through the keep-out.
    let face_start = start_deg[0];
    let (mut lo, mut hi) = (face_start, mid_deg[0]);
    for _ in 0..40 {
        let mid = 0.5 * (lo + hi);
        if world_gap_m(&mut world, with_j0(start_deg, mid - face_start)) * 1e3 > clearance_mm / 2.0
        {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let band_deg = with_j0(start_deg, lo - face_start);
    enable_and_teleport(&rig, &mut c, band_deg);
    rig.drain_status();
    rig.wait_status("the arm rests inside the clearance", |s| {
        s.speeds.iter().all(|v| v.abs() < 0.05) && (s.angles[0] - band_deg[0]).abs() < 0.1
    });
    let err = c.expect_error(&jog_j(0, 1.0, 5.0));
    assert_eq!(
        err.code,
        ErrorCode::SysSelfCollision as u16,
        "a jog through the keep-out must be refused: {err:?}"
    );
    // A move queued while that refusal is still being put down cancels the
    // placement instead of being cut short by it.
    let away_deg = with_j0(band_deg, -10.0);
    let i = c.ok_index(&move_j(7201, away_deg, 2.0));
    let (ok, detail) = c.wait_complete(i);
    assert!(
        ok,
        "a move queued behind a refusal must run, got {detail:?}"
    );
    let s = wait_still(&rig);
    assert!(
        angles_close(&s.angles, &away_deg, 1.0),
        "the move must reach its target: {:?}",
        s.angles
    );

    // --- from a shallow penetration, driving deeper is refused.
    // The TCP sits inside the box near its face; a slow jog toward the
    // centre goes DEEPER through the same colliding pair, and only the
    // min-distance half of the escape rule can see that.
    let shallow_deg = with_j0(mid_deg, -0.8 * (KEEPOUT_M / 2.0) * deg_per_m);
    enable_and_teleport(&rig, &mut c, shallow_deg);
    rig.drain_status();
    let s = rig.wait_status("the arm rests at the shallow spot", |s| {
        s.speeds.iter().all(|v| v.abs() < 0.05) && (s.angles[0] - shallow_deg[0]).abs() < 1.0
    });
    assert!(s.homed, "teleport must leave the arm referenced");
    // A speed whose held stop lands AT the box centre. coal's penetration
    // depth for a mesh-vs-box pair is a local contact-patch estimate,
    // nearly flat in the true depth (measured on this rig: ~5 mm of
    // reported deepening across the full 40 mm face-to-centre travel),
    // so a gentle probe's deepening drowns in the gate's jitter
    // tolerance — the centre is where the measured drop is unambiguous.
    // The refusal still cannot be excused as "the far side is shallower
    // again": the centre is the depth extremum, not past it.
    let pct = j0_speed_reaching(0.8 * (KEEPOUT_M / 2.0) / radius_m);
    let err = c.expect_error(&jog_j(0, pct, 5.0));
    assert_eq!(
        err.code,
        ErrorCode::SysSelfCollision as u16,
        "a deeper-penetrating jog must be refused: {err:?}"
    );
    assert!(
        err.cause.contains("keepout"),
        "the refusal must name the colliding pair: {err:?}"
    );
    // The same spot still allows the OUTWARD jog: the refusal above is
    // the direction's, not the position's.
    rig.drain_status();
    c.send(&jog_j(0, -0.3, 5.0));
    rig.wait_status("the outward jog from the shallow spot runs", |s| {
        s.angles[0] < shallow_deg[0] - 3.0
    });

    rig.shutdown();
}

// ---- installation keep-outs ------------------------------------------------

/// `[[installation_shapes]]` is a real producer for the installation layer
/// (issue #19 gap 3), declared where an installation's own values live: in
/// the local overlay beside the robot TOML. The configured keep-out arrives
/// in the ENFORCED collision world at boot (a planned move through it is
/// refused, not merely echoed), the SHAPES query reads it back on the
/// `installation` list, CONFIG_BUNDLE reports it in the config the runtime
/// runs, and neither `set_shapes` nor `reset_state` can remove it. A
/// malformed entry refuses BOOT with the shape named.
#[test]
fn installation_shapes_are_loaded_enforced_and_immutable_from_the_wire() {
    let mid_deg = with_j0(SWEEP_START_DEG, SWEEP_DEG / 2.0);
    let end_deg = with_j0(SWEEP_START_DEG, SWEEP_DEG);
    let mid_m = tcp_at_m(mid_deg);

    // The overlay also sets one of the fitted tool's own values: the
    // runtime reports the tool file as it runs it.
    let config = test_config("install-shapes");
    let fitted = par6_config::ConfigBundle::load(&config)
        .expect("config")
        .robot
        .robot
        .active_tool;
    std::fs::write(
        config.with_file_name(par6_config::LOCAL_CONFIG_NAME),
        format!(
            "[[installation_shapes]]\nname = \"cage\"\nkind = \"box\"\n\
             params = [{KEEPOUT_M}, {KEEPOUT_M}, {KEEPOUT_M}]\n\
             pose = [{}, {}, {}, 0.0, 0.0, 0.0]\n\
             [[tools]]\nname = \"{fitted}\"\n[tools.driver]\nilim_ma = 912.0\n",
            mid_m[0], mid_m[1], mid_m[2],
        ),
    )
    .expect("write the local overlay");
    let rig = Rig::boot_with(config);
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    // The SHAPES query reads the configured keep-out back on the
    // installation list, program layer empty.
    match c.query(&Command::Shapes) {
        QueryResult::Shapes {
            installation,
            program,
            ..
        } => {
            assert_eq!(program, Vec::<Shape>::new());
            assert_eq!(
                installation
                    .iter()
                    .map(|s| s.name.as_str())
                    .collect::<Vec<_>>(),
                ["floor", "cage"],
                "{installation:?}"
            );
            let cage = &installation[1];
            assert_eq!(cage.kind, "box");
            assert_eq!(cage.params, vec![KEEPOUT_M; 3]);
        }
        other => panic!("unexpected SHAPES result {other:?}"),
    }
    // A client rebuilding the config from CONFIG_BUNDLE gets the cage and
    // the fitted tool's own value too.
    match c.query(&Command::ConfigBundle) {
        QueryResult::ConfigBundle {
            robot_toml, tools, ..
        } => {
            let doc: toml::Table = toml::from_str(&robot_toml).expect("the reported robot TOML");
            let names: Vec<_> = doc["installation_shapes"]
                .as_array()
                .expect("the installation shapes")
                .iter()
                .filter_map(|s| s["name"].as_str())
                .collect();
            assert_eq!(names, ["floor", "cage"], "{robot_toml}");
            let (_, tool) = tools
                .iter()
                .find(|(name, _)| *name == format!("{fitted}.toml"))
                .expect("the fitted tool's file");
            let doc: toml::Table = toml::from_str(tool).expect("the reported tool TOML");
            assert_eq!(doc["driver"]["ilim_ma"].as_float(), Some(912.0), "{tool}");
        }
        other => panic!("unexpected CONFIG_BUNDLE result {other:?}"),
    }

    // ENFORCED, not just echoed: the sweep through it is refused with
    // the cage named, from boot, with no set_shapes ever sent.
    enable_and_teleport(&rig, &mut c, SWEEP_START_DEG);
    let i = c.ok_index(&move_j(7101, end_deg, SWEEP_S));
    let (ok, detail) = c.wait_complete(i);
    assert!(!ok, "a configured keep-out must be enforced at boot");
    let e = detail.expect("a failed COMPLETE carries the error");
    assert_eq!(e.code, ErrorCode::SysSelfCollision as u16, "{e:?}");
    assert!(e.cause.contains("cage"), "{e:?}");

    // The streaming gate got the same layer: a jog toward the cage is
    // blocked and latched too. The refused move's own latch is cleared
    // first (teleport = an accepted motion), so the collision_active
    // frame waited on below can only be the JOG gate's.
    enable_and_teleport(&rig, &mut c, SWEEP_START_DEG);
    rig.drain_status();
    rig.wait_status("the refused move's verdict is cleared", |s| {
        !s.collision_active
    });
    c.send(&jog_j(0, 0.5, 10.0));
    let s = rig.wait_status("the jog toward the cage is blocked", |s| s.collision_active);
    assert!(
        s.collision_pairs
            .iter()
            .any(|(a, b)| a == "install:cage" || b == "install:cage"),
        "the latched pairs must name the cage as an installation shape: {:?}",
        s.collision_pairs
    );

    // A tool change rebuilds both worlds for the new tool's geometry; the
    // cage comes along into the planner's and the gate's alike.
    enable_and_teleport(&rig, &mut c, SWEEP_START_DEG);
    let i = c.ok_index(&Command::SelectTool(SelectTool {
        key: 7103,
        tool_name: "MSG_medium_motor_150mm_rail".to_owned(),
        variant_key: None,
    }));
    let (ok, detail) = c.wait_complete(i);
    assert!(ok, "select_tool must complete, got {detail:?}");
    let i = c.ok_index(&move_j(7104, end_deg, SWEEP_S));
    let (ok, detail) = c.wait_complete(i);
    assert!(!ok, "a tool change dropped the planner's cage: {detail:?}");
    enable_and_teleport(&rig, &mut c, SWEEP_START_DEG);
    rig.drain_status();
    rig.wait_status("the refused move's verdict is cleared", |s| {
        !s.collision_active
    });
    c.send(&jog_j(0, 0.5, 10.0));
    let s = rig.wait_status(
        "the jog toward the cage is blocked after the tool change",
        |s| s.collision_active,
    );
    assert!(
        s.collision_pairs
            .iter()
            .any(|(a, b)| a == "install:cage" || b == "install:cage"),
        "a tool change dropped the stream gate's cage: {:?}",
        s.collision_pairs
    );

    // Nothing on the wire removes it: an empty set_shapes and a full
    // reset_state both leave the cage standing and enforced.
    c.ok(&set_shapes(Vec::new()));
    c.ok(&Command::ResetState);
    match c.query(&Command::Shapes) {
        QueryResult::Shapes { installation, .. } => {
            assert_eq!(
                installation
                    .iter()
                    .map(|s| s.name.as_str())
                    .collect::<Vec<_>>(),
                ["floor", "cage"],
                "{installation:?}"
            );
        }
        other => panic!("unexpected SHAPES result {other:?}"),
    }
    enable_and_teleport(&rig, &mut c, SWEEP_START_DEG);
    let i = c.ok_index(&move_j(7102, end_deg, SWEEP_S));
    let (ok, _) = c.wait_complete(i);
    assert!(
        !ok,
        "set_shapes/reset_state must not be able to clear the installation layer"
    );

    rig.shutdown();
}

/// A malformed `[[installation_shapes]]` entry is a startup refusal that
/// names the shape — the alternative is a daemon that comes up with a
/// keep-out silently missing from the world the operator configured.
#[test]
fn a_malformed_installation_shape_refuses_boot_by_name() {
    let config = test_config("install-bad");
    std::fs::write(
        &config,
        format!(
            "{}\n[[installation_shapes]]\nname = \"wall\"\nkind = \"pyramid\"\n\
             params = [0.5, 0.5, 0.5]\npose = [0.4, 0.0, 0.3, 0.0, 0.0, 0.0]\n",
            std::fs::read_to_string(&config).expect("test config"),
        ),
    )
    .expect("write config");
    let opts = Options {
        sim: true,
        config: Some(config),
        assets: Some(common::assets_dir()),
        command_port: Some(0),
        bind: Some("127.0.0.1".parse().unwrap()),
        status_host: Some("127.0.0.1".parse().unwrap()),
        status_transport: Some(StatusTransport::Unicast),
        ..Options::default()
    };
    let err = Daemon::start(&opts)
        .err()
        .expect("a malformed keep-out must refuse boot")
        .to_string();
    assert!(err.contains("installation"), "{err}");
    assert!(err.contains("wall"), "{err}");
    assert!(err.contains("pyramid"), "{err}");
}

// ---- TCP offset -------------------------------------------------------------

/// Length of the commanded TCP offset \[mm\] — a tool standing this far
/// off the gripper's own TCP, the case `set_tcp_offset` exists for.
const TOOL_OFFSET_MM: f64 = 100.0;
/// World displacement of the commanded target from the start pose \[mm\].
/// The travel `cartesian_surface_over_protocol_v2` already proves the arm
/// covers from [`CART_START_DEG`], so both runs below land well inside the
/// workspace; the offset points along the same ray.
const OFFSET_TARGET_MM: [f64; 3] = [120.0, 60.0, 120.0];
/// Settle time for the cartesian moves below \[s\].
const OFFSET_MOVE_S: f64 = 8.0;

fn move_j_pose(key: u64, pose: [f64; 6], duration_s: f64) -> Command {
    Command::MoveJPose(MoveJPose {
        key,
        pose,
        duration: Some(duration_s),
        speed: None,
        accel: None,
        blend_radius: None,
    })
}

fn set_tcp_offset(key: u64, x: f64, y: f64, z: f64) -> Command {
    Command::SetTcpOffset(SetTcpOffset { key, x, y, z })
}

fn tcp_offset_readback(c: &mut Client) -> [f64; 3] {
    match c.query(&Command::TcpOffset) {
        QueryResult::TcpOffset { x, y, z } => [x, y, z],
        other => panic!("unexpected TCP_OFFSET result {other:?}"),
    }
}

/// Settled TCP position after motion stops.
fn settled_tcp(rig: &Rig, what: &str) -> Status {
    rig.drain_status();
    rig.wait_status(what, |s| s.speeds.iter().all(|v| v.abs() < 0.05))
}

/// `set_tcp_offset` retargets the whole cartesian surface, not just the
/// readback.
///
/// The offset composes AFTER the URDF variant's own TCP frame and in the
/// TOOL-LOCAL frame — `T_flange→TCP = T_tool · T_offset`, the composition
/// the Python client already applies for preview FK/IK. The commanded
/// translation here is `Rᵀ·v`, so it is neither the world displacement it
/// must produce nor axis-aligned in any frame: a runtime that read it as
/// a world offset, or dropped it, lands somewhere else. With it set:
///
/// - STATUS reports the offset point, immediately, without the arm moving,
///   and it sits exactly `v` from the flange;
/// - the same commanded `move_j_pose` target puts THAT point on the
///   target, which parks the flange 100 mm from where the identical
///   command parks it with no offset;
/// - the `TCP_OFFSET` query still answers the COMMANDED translation, not
///   the composed transform.
///
/// Before the offset reached the models this failed on the first bound
/// already: STATUS kept reporting the flange, and the two `move_j_pose`
/// runs parked the arm in the same configuration.
#[test]
fn tcp_offset_retargets_the_cartesian_surface_over_protocol_v2() {
    let rig = boot_tagged("tcpoffset");
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    enable_and_teleport(&rig, &mut c, CART_START_DEG);
    rig.drain_status();
    let flange = rig.wait_status("start pose", |_| true);
    let p_flange = tcp_mm(&flange);

    // The world displacement the offset must produce, and the tool-local
    // translation that produces it: `d = Rᵀ·v` off the start orientation.
    let norm = (OFFSET_TARGET_MM[0] * OFFSET_TARGET_MM[0]
        + OFFSET_TARGET_MM[1] * OFFSET_TARGET_MM[1]
        + OFFSET_TARGET_MM[2] * OFFSET_TARGET_MM[2])
        .sqrt();
    let v: [f64; 3] = std::array::from_fn(|i| TOOL_OFFSET_MM * OFFSET_TARGET_MM[i] / norm);
    let d: [f64; 3] = std::array::from_fn(|j| (0..3).map(|i| flange.pose[4 * i + j] * v[i]).sum());

    // --- The reported point moves; the arm does not.
    let i = c.ok_index(&set_tcp_offset(1701, d[0], d[1], d[2]));
    c.wait_complete(i);
    rig.drain_status();
    let offset = rig.wait_status("STATUS follows the offset TCP", |s| {
        distance(tcp_mm(s), p_flange) > 1.0
    });
    let p_tcp = tcp_mm(&offset);
    for k in 0..3 {
        let want = p_flange[k] + v[k];
        assert!(
            (p_tcp[k] - want).abs() < 0.5,
            "the tool-local offset {d:?} must displace the reported TCP by {v:?}: \
             axis {k} is {}, expected {want} ({p_flange:?} -> {p_tcp:?})",
            p_tcp[k]
        );
    }
    assert!(
        angles_close(&offset.angles, &flange.angles, 0.1),
        "setting a TCP offset must not move the arm: {:?} -> {:?}",
        flange.angles,
        offset.angles
    );
    // A pure translation in the tool frame: the orientation block is
    // untouched, so only the point the runtime resolves at has changed.
    //
    // The bound is the arm's OWN motion between the two broadcasts, not a
    // constant. These are two STATUS frames from a live plant holding a
    // target, and a rotation-matrix element drifts with the joints under
    // it — the check right above admits 0.1 deg of exactly that, so a
    // fixed 1e-6 here asserted something its sibling already allowed to be
    // false, and did until the plant became a contact simulation. A tool
    // rotation is orders of magnitude past this; drift cannot be.
    let arm_moved_rad: f64 = offset
        .angles
        .iter()
        .zip(flange.angles.iter())
        .map(|(a, b)| (a - b).abs().to_radians())
        .sum();
    let tol = arm_moved_rad + 1e-6;
    for k in [0, 1, 2, 4, 5, 6, 8, 9, 10] {
        let drift = (offset.pose[k] - flange.pose[k]).abs();
        assert!(
            drift < tol,
            "the offset rotated the reported pose at element {k}: {drift:.3e}, \
             past the {tol:.3e} the arm's own {:.4} deg of motion allows",
            arm_moved_rad.to_degrees()
        );
    }
    let readback = tcp_offset_readback(&mut c);
    for k in 0..3 {
        assert!(
            (readback[k] - d[k]).abs() < 1e-9,
            "TCP_OFFSET answers the commanded translation {d:?}, not the composed \
             transform: got {readback:?}"
        );
    }

    // --- The same commanded target, with and without the offset. Both
    // runs park the flange along the travel
    // `cartesian_surface_over_protocol_v2` already proves is clear, a full
    // TOOL_OFFSET_MM apart from each other.
    let target: [f64; 3] = std::array::from_fn(|k| p_flange[k] + OFFSET_TARGET_MM[k]);
    let wire_target = wire_pose_at(&flange.pose, target);

    let i = c.ok_index(&move_j_pose(2001, wire_target, OFFSET_MOVE_S));
    let (ok, detail) = c.wait_complete(i);
    assert!(
        ok,
        "move_j_pose with a TCP offset must complete, got {detail:?}"
    );
    let landed = settled_tcp(&rig, "settled with the offset");
    let reached = tcp_mm(&landed);
    assert!(
        distance(reached, target) < 10.0,
        "STATUS must report the OFFSET point on the commanded target: \
         {reached:?} vs {target:?}"
    );

    // Where the flange actually ended up: same configuration, offset off.
    let i = c.ok_index(&set_tcp_offset(1702, 0.0, 0.0, 0.0));
    c.wait_complete(i);
    rig.drain_status();
    let f_with = tcp_mm(&rig.wait_status("flange of the offset run", |s| {
        distance(tcp_mm(s), reached) > 1.0
    }));

    rig.drain_status();
    enable_and_teleport(&rig, &mut c, CART_START_DEG);
    let i = c.ok_index(&move_j_pose(2002, wire_target, OFFSET_MOVE_S));
    let (ok, detail) = c.wait_complete(i);
    assert!(
        ok,
        "move_j_pose without an offset must complete, got {detail:?}"
    );
    let plain = settled_tcp(&rig, "settled without the offset");
    let f_without = tcp_mm(&plain);
    assert!(
        distance(f_without, target) < 10.0,
        "without an offset the FLANGE lands on the target: {f_without:?} vs {target:?}"
    );

    let moved = distance(f_with, f_without);
    assert!(
        moved > 80.0,
        "the offset must land the flange somewhere else for the same commanded \
         target: {f_with:?} vs {f_without:?} ({moved:.1} mm apart, expected \
         about {TOOL_OFFSET_MM})"
    );
    let joint_delta = landed
        .angles
        .iter()
        .zip(plain.angles.iter())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0f64, f64::max);
    assert!(
        joint_delta > 2.0,
        "the two runs parked in the same configuration ({joint_delta:.3} deg apart): \
         the offset never reached the planner's IK"
    );

    rig.shutdown();
}

// ---- cartesian enablement ---------------------------------------------------

/// A configuration at the edge of the reachable workspace: the arm
/// reaches out along −x with the shoulder against its soft window, so a
/// step further out has no in-window IK solution while a step back in
/// does. The shoulder wall (not full extension) is what blocks −x: at
/// the kinematic-singular full stretch the probe's unclamped DLS solves
/// blow up and every direction reads blocked, which is a solver
/// artifact, not the workspace edge.
const BOUNDARY_DEG: [f64; NUM_JOINTS] = [0.0, -139.8, 322.1, 0.0, -27.1, 180.0];
/// World-frame enablement slots, positive direction first.
const X_POS: usize = 0;
const X_NEG: usize = 1;

fn move_l_to(key: u64, pose: [f64; 6], duration_s: f64) -> Command {
    Command::MoveL(MoveL {
        key,
        pose,
        frame: Frame::Wrf,
        duration: Some(duration_s),
        speed: None,
        accel: None,
        blend_radius: None,
        rel: false,
    })
}

/// The cartesian enablement flags describe the real workspace.
///
/// Parked against the outer edge of its reach, the arm may still move
/// inward and may not move further out — and STATUS says exactly that,
/// per direction, in both frames. The flags are then corroborated against
/// what the runtime actually does with the same two directions: the
/// blocked one is an `IK_TARGET_UNREACHABLE` refusal, the free one runs to
/// COMPLETE.
///
/// Before the probe landed, an `ffi` runtime published
/// `Enablement::default()` — all twelve directions free, in both frames,
/// in every pose — so the blocked direction read 1 and a frontend offered
/// a jog button for a motion the runtime would refuse.
#[test]
fn cartesian_enablement_measures_the_real_workspace() {
    let rig = boot_tagged("enablement");
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    enable_and_teleport(&rig, &mut c, BOUNDARY_DEG);
    rig.drain_status();
    // The probe is rate- and change-gated and costs 24 seeded IK solves,
    // so the flags lag the arrival: frames right after the teleport still
    // carry the pre-probe default (all zero) or the previous pose's
    // measurement. Wait for a frame whose flags are the boundary's own —
    // if the probe never converges on (in-free, out-blocked) this times
    // out and fails just as loudly as the asserts below.
    let s = rig.wait_status("parked at the edge with the probe refreshed", |s| {
        angles_close(&s.angles, &BOUNDARY_DEG, 0.5)
            && s.cart_en_wrf[X_POS] == 1
            && s.cart_en_wrf[X_NEG] == 0
    });
    let edge = tcp_mm(&s);
    assert!(
        edge[0] < -400.0,
        "the boundary pose must reach out along -x, got {edge:?}"
    );
    assert_eq!(
        (s.cart_en_wrf[X_POS], s.cart_en_wrf[X_NEG]),
        (1, 0),
        "at the edge of reach the arm may move in (+x) and not out (-x); \
         world-frame flags were {:?}",
        s.cart_en_wrf
    );
    assert!(
        s.cart_en_trf.contains(&0),
        "the tool-frame flags are a real measurement too, not a default: {:?}",
        s.cart_en_trf
    );
    // The REACHABLE query answers from the same measurement.
    match c.query(&Command::Reachable) {
        QueryResult::Reachable { cart_en_wrf, .. } => assert_eq!(
            (cart_en_wrf[X_POS], cart_en_wrf[X_NEG]),
            (1, 0),
            "REACHABLE disagrees with STATUS: {cart_en_wrf:?}"
        ),
        other => panic!("unexpected REACHABLE result {other:?}"),
    }

    // Corroboration: the runtime really does refuse the direction it
    // greyed, and really does run the one it kept.
    let inward = wire_pose_at(&s.pose, [edge[0] + 20.0, edge[1], edge[2]]);
    let i = c.ok_index(&move_l_to(3001, inward, 3.0));
    let (ok, detail) = c.wait_complete(i);
    assert!(
        ok,
        "the direction reported free must actually run, got {detail:?}"
    );

    enable_and_teleport(&rig, &mut c, BOUNDARY_DEG);
    let outward = wire_pose_at(&s.pose, [edge[0] - 20.0, edge[1], edge[2]]);
    let i = c.ok_index(&move_l_to(3002, outward, 3.0));
    let (ok, detail) = c.wait_complete(i);
    assert!(
        !ok,
        "the direction reported blocked must actually be refused"
    );
    // Any of three verdicts names the same physical fact. At a true
    // reach edge the solver loses convergence (whole line or an interior
    // sample first); at this boundary — the shoulder against its soft
    // window — IK converges fine and the refusal is the soft-window
    // validation on the solution, exactly the check that withdrew the
    // flag.
    let code = detail.expect("a failed COMPLETE carries the error").code;
    assert!(
        code == ErrorCode::IkTargetUnreachable as u16
            || code == ErrorCode::IkPartialPath as u16
            || code == ErrorCode::CommValidationError as u16,
        "the blocked direction must be blocked for the reason the flag claims, \
         got error code {code}"
    );

    rig.shutdown();
}

// ---- curved and blended moves ----------------------------------------------

/// Start posture for the curved and blended moves: the same kind of
/// well-conditioned pose as [`CART_START_DEG`], chosen (by the same
/// soft-limit-box sweep) for room around it — 120 mm of straight-line
/// travel is IK-feasible in every axis direction and along the diagonals
/// from here, so a 120 mm arc and two 120 mm legs fit without touching a
/// soft window.
const CURVE_START_DEG: [f64; NUM_JOINTS] = [-125.0, -80.0, 175.0, 0.0, -40.0, 180.0];
/// Duration of the spline move \[s\]. Slower than [`MOVE_S`] because the
/// sim's tracking lag is proportional to speed AND to path curvature,
/// and a wave has far more of the second than a straight line does.
const SPLINE_S: f64 = 25.0;

/// Speed of the TCP \[mm/s\] over a sliding window of STATUS frames,
/// paired with the position at the middle of each window.
///
/// Measured from the broadcast itself (pose delta over the header's
/// monotonic clock) rather than read out of `STATUS.tcp_speed`, so the
/// numbers below stand on the same evidence a client has. The window
/// spans [`SPEED_WINDOW`] frames because the status rate and the RT tick
/// rate are the same here: consecutive frames sometimes carry the same
/// snapshot, and a frame pair that straddles one would read as a
/// standstill.
fn tcp_speeds(path: &[Status]) -> Vec<([f64; 3], f64)> {
    tcp_speeds_over(path, SPEED_WINDOW)
}

/// [`tcp_speeds`] over a caller-chosen window width.
fn tcp_speeds_over(path: &[Status], window: usize) -> Vec<([f64; 3], f64)> {
    path.windows(window)
        .filter_map(|w| {
            let (first, last) = (&w[0], &w[window - 1]);
            // Only a window the test actually received every frame of.
            // Speed here is the CHORD between the ends over the elapsed
            // time, so a frame the socket dropped stretches the time
            // while the chord stays straight — across a corner that
            // reads as a slowdown that never happened. The broadcast
            // sequence says which windows are whole, so a dropped frame
            // costs a sample instead of corrupting one.
            let span = last.seq.checked_sub(first.seq)?;
            if span != (window - 1) as u64 {
                return None;
            }
            let dt = last.mono_time_ns.checked_sub(first.mono_time_ns)? as f64 * 1e-9;
            (dt > 0.0).then(|| {
                (
                    tcp_mm(&w[window / 2]),
                    distance(tcp_mm(first), tcp_mm(last)) / dt,
                )
            })
        })
        .collect()
}

/// STATUS frames per speed measurement (~0.1 s at the 50 Hz broadcast).
const SPEED_WINDOW: usize = 5;
/// STATUS frames per motion-window speed read (~0.2 s).
const MOTION_WINDOW: usize = 10;

/// Teleport to the curved-move start posture and return the pose the arm
/// actually came to rest in.
///
/// The wait is on the ANGLES, not just on the broadcast: `teleport`
/// confirms within a degree, and a degree of J1 is centimetres of TCP —
/// enough to move every target derived from this pose by more than the
/// path tolerances below.
fn curve_start(rig: &Rig, c: &mut Client) -> Status {
    enable_and_teleport(rig, c, CURVE_START_DEG);
    rig.drain_status();
    rig.wait_status("the arm at rest in the curved-move start posture", |s| {
        s.angles
            .iter()
            .zip(CURVE_START_DEG.iter())
            .all(|(a, b)| (a - b).abs() < 0.02)
            && s.speeds.iter().all(|v| v.abs() < 0.01)
    })
}

/// Seconds between the first and last stretch of a status stream in which
/// the TCP moves faster than `floor_mm_s`: the chain's motion time,
/// indifferent to how long the stream kept being collected after the arm
/// stopped.
///
/// A speed over [`MOTION_WINDOW`] frames, for the reason [`tcp_speeds`]
/// gives: the status task and the RT tick alias, so one frame can carry
/// two ticks of travel. During the settle creep that is a tenth of a
/// millimetre — read frame by frame it counted as movement a dozen
/// frames after the chain had stopped, and put the chain's motion time
/// wherever that frame happened to fall. The window is longer than the
/// corner measurement's and the floor sits above the creep, so the ends
/// of the window land on the ramps, where one aliased frame is noise.
fn motion_seconds(path: &[Status], floor_mm_s: f64) -> f64 {
    let mut first = None;
    let mut last = None;
    for w in path.windows(MOTION_WINDOW) {
        let (a, b) = (&w[0], &w[MOTION_WINDOW - 1]);
        let dt = b.mono_time_ns.saturating_sub(a.mono_time_ns) as f64 * 1e-9;
        if dt > 0.0 && distance(tcp_mm(a), tcp_mm(b)) / dt > floor_mm_s {
            first.get_or_insert(a.mono_time_ns);
            last = Some(b.mono_time_ns);
        }
    }
    match (first, last) {
        (Some(a), Some(b)) if b > a => (b - a) as f64 * 1e-9,
        _ => 0.0,
    }
}

/// The slowest the TCP ever got while it was within `radius_mm` of
/// `corner`, and the mean speed over the whole move.
fn corner_and_mean_speed(path: &[Status], corner: [f64; 3], radius_mm: f64) -> (f64, f64) {
    let speeds = tcp_speeds(path);
    let moving: Vec<f64> = speeds
        .iter()
        .map(|(_, v)| *v)
        .filter(|v| *v > 0.5)
        .collect();
    let mean = if moving.is_empty() {
        0.0
    } else {
        moving.iter().sum::<f64>() / moving.len() as f64
    };
    // The SLOWEST the corner sustains, not the slowest single sample. One
    // tick the daemon could not hold reads as a stop and sinks a
    // per-sample minimum, which is a measurement of the host rather than
    // of the blend — it failed on a CI runner at 5.21 mm/s against a 5.36
    // bar while the loop reported no overruns and a p99 of 20.08 ms
    // against a 20.00 ms budget, i.e. at its deadline but not past it.
    // Three consecutive samples cannot all be that tick, and a blend that
    // actually stops is slow across all of them: the unblended corner
    // this is measured against reads 0.05 mm/s either way.
    let through: Vec<f64> = speeds
        .iter()
        .filter(|(p, _)| distance(*p, corner) < radius_mm)
        .map(|(_, v)| *v)
        .collect();
    const SUSTAINED: usize = 3;
    let at_corner = if through.len() < SUSTAINED {
        through.iter().copied().fold(f64::INFINITY, f64::min)
    } else {
        through
            .windows(SUSTAINED)
            .map(|w| w.iter().sum::<f64>() / w.len() as f64)
            .fold(f64::INFINITY, f64::min)
    };
    (at_corner, mean)
}

/// A TCP-offset change can never be folded into a blend chain.
///
/// Measured before this landed: `set_tcp_offset` was immediate, so it was
/// never in `pending` and `[move_l(blend), set_tcp_offset, move_l]` folded
/// both legs into one motion — the new frame applied to both or neither,
/// decided by datagram arrival against the blend hold. Queued, it sits
/// between the legs: the first runs alone against the old frame, the
/// offset lands, and only then is the second planned.
#[test]
fn a_tcp_offset_between_blended_moves_breaks_the_chain() {
    const LEG_MM: f64 = 60.0;
    const LEG_S: f64 = 1.0;

    let rig = boot_tagged("offset-chain");
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    let leg = |s: &Status, key: u64, xyz: [f64; 3], r: Option<f64>| {
        Command::MoveL(MoveL {
            key,
            pose: wire_pose_at(&s.pose, xyz),
            frame: Frame::Wrf,
            duration: Some(LEG_S),
            speed: None,
            accel: None,
            blend_radius: r,
            rel: false,
        })
    };
    let queue_while_executing = |c: &mut Client, head: u64| -> Vec<String> {
        let deadline = Instant::now() + BUDGET;
        loop {
            match c.query(&Command::Queue) {
                QueryResult::Queue {
                    queue,
                    executing_index,
                    ..
                } if executing_index == head as i64 => return queue,
                QueryResult::Queue { .. } => {}
                other => panic!("unexpected {other:?}"),
            }
            assert!(Instant::now() < deadline, "command {head} never started");
        }
    };
    let read_offset = |c: &mut Client| -> [f64; 3] {
        match c.query(&Command::TcpOffset) {
            QueryResult::TcpOffset { x, y, z } => [x, y, z],
            other => panic!("unexpected {other:?}"),
        }
    };

    // --- control: the same two legs with nothing between them fold.
    let s = curve_start(&rig, &mut c);
    let start = tcp_mm(&s);
    let corner = [start[0] + LEG_MM, start[1], start[2]];
    let finish = [corner[0], corner[1], corner[2] - LEG_MM];
    let i1 = c.ok_index(&leg(&s, 5301, corner, Some(20.0)));
    let i2 = c.ok_index(&leg(&s, 5302, finish, None));
    assert!(
        queue_while_executing(&mut c, i1).is_empty(),
        "two blended legs are one motion: the second leaves the queue with the first"
    );
    c.wait_complete(i1);
    c.wait_complete(i2);

    // --- an offset between them keeps the legs apart and lands in order.
    let s = curve_start(&rig, &mut c);
    let start = tcp_mm(&s);
    let corner = [start[0] + LEG_MM, start[1], start[2]];
    let finish = [corner[0], corner[1], corner[2] - LEG_MM];
    let i3 = c.ok_index(&leg(&s, 5303, corner, Some(20.0)));
    let i4 = c.ok_index(&set_tcp_offset(5304, 0.0, 0.0, 25.0));
    let i5 = c.ok_index(&leg(&s, 5305, finish, None));
    assert_eq!(
        queue_while_executing(&mut c, i3),
        vec!["set_tcp_offset".to_owned(), "move_l".to_owned()],
        "the offset must break the chain: the second leg waits behind it"
    );
    assert_eq!(
        read_offset(&mut c),
        [0.0; 3],
        "the offset must not apply while the leg queued before it runs"
    );
    let (ok, detail) = c.wait_complete(i3);
    assert!(ok, "first leg: {detail:?}");
    let (ok, detail) = c.wait_complete(i4);
    assert!(ok, "offset: {detail:?}");
    assert_eq!(read_offset(&mut c), [0.0, 0.0, 25.0]);
    let (ok, detail) = c.wait_complete(i5);
    assert!(ok, "second leg: {detail:?}");

    let i6 = c.ok_index(&set_tcp_offset(5306, 0.0, 0.0, 0.0));
    c.wait_complete(i6);
}

/// A blend radius on a queued `move_l` really rounds the corner into the
/// NEXT queued `move_l` — measured against the same corner run without
/// one.
///
/// Before this landed the runtime refused any non-nil `r` outright
/// (`COMM_VALIDATION_ERROR`), because it started exactly one queued
/// command at a time. The claim now is a comparison, not a completion:
/// with `r = 0` the arm stops dead in the corner and passes through it;
/// with `r = 25` it cuts the corner, never stops, and gets there sooner.
/// Both commands still report their own COMPLETE, and the high-water
/// `completed_index` ends on the second of them.
#[test]
fn a_blend_radius_rounds_the_corner_into_the_next_queued_move() {
    /// The server's blend hold, from `ServerConfig::default`.
    const BLEND_HOLD_MS: f64 = 100.0;
    const BLEND_MM: f64 = 60.0;
    const LEG_MM: f64 = 150.0;
    /// Duration of each leg \[s\]. Slow, for the same reason the other
    /// cartesian measurements here are: the rig's tracking error scales
    /// with speed and the corner geometry is what is being measured.
    const LEG_S: f64 = 8.0;

    let rig = boot_tagged("blend");
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    // Two legs meeting at a right angle: +x, then +z.
    let leg = |s: &Status, key: u64, r: Option<f64>| -> (Command, [f64; 3], [f64; 3]) {
        let start = tcp_mm(s);
        let corner = [start[0] + LEG_MM, start[1], start[2]];
        let finish = [corner[0], corner[1], corner[2] - LEG_MM];
        (
            Command::MoveL(MoveL {
                key,
                pose: wire_pose_at(&s.pose, corner),
                frame: Frame::Wrf,
                duration: Some(LEG_S),
                speed: None,
                accel: None,
                blend_radius: r,
                rel: false,
            }),
            corner,
            finish,
        )
    };
    let second = |s: &Status, key: u64, finish: [f64; 3]| {
        Command::MoveL(MoveL {
            key,
            pose: wire_pose_at(&s.pose, finish),
            frame: Frame::Wrf,
            duration: Some(LEG_S),
            speed: None,
            accel: None,
            blend_radius: None,
            rel: false,
        })
    };

    // --- control: the same corner with no blend radius.
    let s = curve_start(&rig, &mut c);
    let (first, corner, finish) = leg(&s, 5001, None);
    let i1 = c.ok_index(&first);
    let i2 = c.ok_index(&second(&s, 5002, finish));
    let sharp = rig.collect_status(Duration::from_secs_f64(2.0 * LEG_S + 2.0));
    let (ok, detail) = c.wait_complete(i1);
    assert!(
        ok,
        "the unblended first leg must complete ok, got {detail:?}"
    );
    let (ok, detail) = c.wait_complete(i2);
    assert!(
        ok,
        "the unblended second leg must complete ok, got {detail:?}"
    );
    let sharp_time = motion_seconds(&sharp, 10.0);
    let sharp_points: Vec<[f64; 3]> = sharp.iter().map(tcp_mm).collect();
    let sharp_miss = path_misses(&sharp_points, corner);
    let (sharp_corner_speed, sharp_mean) = corner_and_mean_speed(&sharp, corner, 20.0);
    assert!(
        sharp_miss < 16.0,
        "without a blend radius the arm must go INTO the corner, missed by {sharp_miss:.2} mm \
         (the rig's own stopping error is about 10 mm)"
    );
    assert!(
        sharp_corner_speed < 1.0,
        "without a blend radius the arm must STOP in the corner, \
         slowest it got was {sharp_corner_speed:.2} mm/s"
    );

    // --- the same corner, blended.
    let s = curve_start(&rig, &mut c);
    let (first, corner, finish) = leg(&s, 5003, Some(BLEND_MM));
    // Both moves go out back to back: the hold the blend depends on is
    // wall-clock, and a reply round trip between them would spend it.
    let sent_first = Instant::now();
    let indices = c.ok_indices(&[first.clone(), second(&s, 5004, finish)]);
    let (i1, i2) = (indices[0], indices[1]);
    let send_gap = sent_first.elapsed();
    let blended = rig.collect_status(Duration::from_secs_f64(2.0 * LEG_S + 2.0));
    let (ok, detail) = c.wait_complete(i1);
    assert!(ok, "the blended first leg must complete ok, got {detail:?}");
    let (ok, detail) = c.wait_complete(i2);
    assert!(
        ok,
        "the blended second leg must complete ok, got {detail:?}"
    );
    let blend_time = motion_seconds(&blended, 10.0);
    let blended_points: Vec<[f64; 3]> = blended.iter().map(tcp_mm).collect();

    // The corner is rounded: cut by more than the tracking error, and
    // by no more than the radius that was asked for.
    // Measured against the SAME corner driven without a radius, so the
    // rig's stopping error cancels out of the comparison: the geometry
    // cuts this corner by 0.35 r (21 mm here), the rest is tracking.
    let blend_miss = path_misses(&blended_points, corner);
    assert!(
        blend_miss > sharp_miss + 6.0,
        "r = {BLEND_MM} mm cut the corner by {blend_miss:.2} mm and the same corner without \
         a radius by {sharp_miss:.2} mm: that is not a rounded corner"
    );
    assert!(
        blend_miss < BLEND_MM,
        "the rounded corner strayed {blend_miss:.2} mm from the waypoint, further than the \
         {BLEND_MM} mm zone that was asked for"
    );
    // And it is a fly-by, not a stop-and-go.
    //
    // Corner speed is the SLOWEST sample through the corner, so a single
    // tick the daemon could not hold sinks it. That is a real stall, not
    // a measurement artefact — and it says the box was too loaded to
    // measure on, not that the blend stopped. The loop's own overrun
    // count is what tells the two apart, so it goes in the message.
    let (blend_corner_speed, blend_mean) = corner_and_mean_speed(&blended, corner, 40.0);
    let loop_note = match c.query(&Command::LoopStats) {
        QueryResult::LoopStats(s) => format!(
            "the RT loop overran {} of {} ticks (p99 {:.2} ms against a {:.2} ms budget)",
            s.overrun_count,
            s.loop_count,
            s.p99_period_s * 1e3,
            1e3 / s.target_hz
        ),
        other => format!("loop stats unavailable: {other:?}"),
    };
    assert!(
        blend_corner_speed > 0.3 * blend_mean && blend_corner_speed > 5.0,
        "the blended corner slowed to {blend_corner_speed:.2} mm/s against a mean of \
         {blend_mean:.2} mm/s (unblended: {sharp_corner_speed:.2} of {sharp_mean:.2}) — \
         a blend that stops is not a blend. The successor was sent {:.0} ms after the \
         first against a {:.0} ms blend hold — past it, the first move runs alone and \
         this is the test host being starved, not the blend. {loop_note}",
        send_gap.as_secs_f64() * 1e3,
        BLEND_HOLD_MS
    );
    // Motion time from the broadcast's own monotonic clock — the
    // wall-clock of the collection loop is a fixed window and cannot
    // tell the two chains apart.
    assert!(
        blend_time < sharp_time,
        "the blended corner moved for {blend_time:.2} s and the sharp one for \
         {sharp_time:.2} s: not stopping is supposed to be faster"
    );
    let end_miss = distance(
        tcp_mm(&settled_tcp(&rig, "settled after the blended chain")),
        finish,
    );
    assert!(
        end_miss < 15.0,
        "the blended chain ended {end_miss:.1} mm off its last target"
    );
    // Completion-index semantics for a blended pair: both indexes are
    // completed, and the high-water mark is the LAST of them.
    let s = rig.wait_status("completed index reaches the blended pair", |s| {
        s.completed_index >= i2 as i64
    });
    assert_eq!(
        s.executing_index, -1,
        "nothing may still be executing once both blended commands completed"
    );

    rig.shutdown();
}

/// A blend radius on a queued `move_j` rounds the corner between two
/// JOINT moves, and is measured where a joint move lives: in joint
/// space.
///
/// Without a radius the arm has to arrive at the intermediate
/// configuration exactly — that is what "the move completed" means — and
/// then set off again. With one, the runtime plans both moves as a
/// single joint path whose corner is a Bézier zone sized from the TCP
/// distance the radius describes, so the intermediate configuration is
/// passed BY, not landed on, and the arm never comes to rest in it.
#[test]
fn a_blend_radius_rounds_a_joint_chain_too() {
    const BLEND_MM: f64 = 30.0;
    const LEG_S: f64 = 4.0;

    let rig = boot_tagged("jointblend");
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    let mut corner_deg = CURVE_START_DEG;
    corner_deg[0] += 15.0;
    let mut finish_deg = corner_deg;
    finish_deg[1] -= 12.0;
    let move_j = |key: u64, angles: [f64; NUM_JOINTS], r: Option<f64>| {
        Command::MoveJ(MoveJ {
            key,
            angles,
            duration: Some(LEG_S),
            speed: None,
            accel: None,
            blend_radius: r,
            rel: false,
        })
    };
    /// Largest per-joint distance from `q` to the corner configuration.
    fn from_corner(s: &Status, corner: [f64; NUM_JOINTS]) -> f64 {
        s.angles
            .iter()
            .zip(corner.iter())
            .map(|(a, b)| (a - b).abs())
            .fold(0.0f64, f64::max)
    }

    // --- control: the same two moves, no radius. The arm lands on the
    // intermediate configuration and stops there.
    curve_start(&rig, &mut c);
    let i1 = c.ok_index(&move_j(6001, corner_deg, None));
    let i2 = c.ok_index(&move_j(6002, finish_deg, None));
    let sharp = rig.collect_through(i2, Duration::from_secs_f64(2.0 * LEG_S + 2.0));
    for i in [i1, i2] {
        let (ok, detail) = c.wait_complete(i);
        assert!(
            ok,
            "the unblended joint legs must complete ok, got {detail:?}"
        );
    }
    let sharp_miss = sharp
        .iter()
        .map(|s| from_corner(s, corner_deg))
        .fold(f64::INFINITY, f64::min);
    assert!(
        sharp_miss < 1.5,
        "without a radius the arm must reach the intermediate configuration, \
         closest it got was {sharp_miss:.2}° (the rig's own stopping error is a few tenths)"
    );

    // --- the same pair, blended.
    curve_start(&rig, &mut c);
    let i1 = c.ok_index(&move_j(6003, corner_deg, Some(BLEND_MM)));
    let i2 = c.ok_index(&move_j(6004, finish_deg, None));
    let blended = rig.collect_through(i2, Duration::from_secs_f64(2.0 * LEG_S + 2.0));
    for i in [i1, i2] {
        let (ok, detail) = c.wait_complete(i);
        assert!(
            ok,
            "the blended joint legs must complete ok, got {detail:?}"
        );
    }
    let blend_miss = blended
        .iter()
        .map(|s| from_corner(s, corner_deg))
        .fold(f64::INFINITY, f64::min);
    assert!(
        blend_miss > sharp_miss + 0.5,
        "r = {BLEND_MM} mm passed within {blend_miss:.2}° of the intermediate configuration \
         and the unblended pair within {sharp_miss:.2}°: the corner was not rounded"
    );

    // And it never stopped there: between leaving the start and reaching
    // the end, the joints keep moving.
    let mid: Vec<&Status> = blended
        .iter()
        .filter(|s| from_corner(s, corner_deg) < 4.0)
        .collect();
    assert!(
        mid.len() > 5,
        "expected the corner region to be sampled, got {} frames",
        mid.len()
    );
    // The drive reports velocity over only 20 firmware samples (3.2 ms).
    // One encoder count in that window is already 0.0187 rad/s on J1;
    // judge continued path progress over 100 ms instead of mistaking a
    // quantized sample for a stop. The same speed threshold must
    // distinguish the unblended control from the rounded path.
    let slowest_progress = |frames: &[Status]| {
        let mut slowest = f64::INFINITY;
        let mut windows = 0;
        for (i, a) in frames.iter().enumerate() {
            if from_corner(a, corner_deg) >= 4.0 {
                continue;
            }
            let Some(b) = frames[i + 1..]
                .iter()
                .find(|b| b.mono_time_ns.saturating_sub(a.mono_time_ns) >= 100_000_000)
            else {
                continue;
            };
            if from_corner(b, corner_deg) >= 4.0 {
                continue;
            }
            let dt = (b.mono_time_ns - a.mono_time_ns) as f64 * 1e-9;
            let travel = a
                .angles
                .iter()
                .zip(b.angles)
                .map(|(a, b)| (b - a).to_radians().powi(2))
                .sum::<f64>()
                .sqrt();
            slowest = slowest.min(travel / dt);
            windows += 1;
        }
        assert!(windows >= 5, "not enough corner progress windows");
        slowest
    };
    let sharp_slowest = slowest_progress(&sharp);
    let slowest = slowest_progress(&blended);
    assert!(
        sharp_slowest < 0.02,
        "the unblended control must actually stop at the corner, got {sharp_slowest:.4} rad/s"
    );
    assert!(
        slowest > 0.02,
        "the blended joint corner slowed to {slowest:.4} rad/s: a blend that stops is not a blend"
    );
    rig.wait_status(
        "the blended joint chain reports both commands complete",
        |s| s.completed_index >= i2 as i64,
    );
    // Where it came to rest, within the rig's own stopping error (the
    // unblended pair above lands no closer).
    let settled = settled_tcp(&rig, "the blended joint chain at rest");
    assert!(
        settled
            .angles
            .iter()
            .zip(finish_deg.iter())
            .all(|(a, b)| (a - b).abs() < 2.0),
        "the blended chain ended at {:?}, not at {finish_deg:?}",
        settled.angles
    );

    rig.shutdown();
}

/// The configured soft window, in radians, per joint.
fn soft_window_rad() -> [(f64, f64); NUM_JOINTS] {
    let cfg = par6_config::RobotConfig::load(&shipped_config()).expect("PAR6 config");
    let mut out = [(0.0, 0.0); NUM_JOINTS];
    for (slot, joint) in out.iter_mut().zip(cfg.joints.iter()) {
        *slot = (joint.limits.soft_min_rad, joint.limits.soft_max_rad);
    }
    out
}

fn to_deg(rad: [f64; NUM_JOINTS]) -> [f64; NUM_JOINTS] {
    rad.map(f64::to_degrees)
}

/// A posture clear of the wrist singularity, with J6 near the top of
/// its window.
const TURNED_POSTURE_RAD: [f64; NUM_JOINTS] = [
    0.585_609, -1.010_888, 3.205_22, -0.031_356, -0.093_302, 3.045_917,
];

/// J6 where the move to [`TURNED_POSTURE_RAD`] starts: more than π below
/// the target, so the solution nearest this seed is the target's `-2π`
/// alias — below J6's window, while the target itself is inside it.
const SEED_J6_RAD: f64 = -0.5;

/// J5 held past its SOFT window (1.9 rad) but inside its hard one: a
/// posture the arm can be teleported into and whose pose no turn of any
/// joint brings back in range.
const BEYOND_SOFT_J5_RAD: f64 = 1.9;

/// A converged IK solution is judged as a configuration, not as a turn
/// count.
///
/// The closed-form solve returns each joint on the branch nearest its
/// seed, and for a joint whose window is wider than π that branch can
/// sit outside the window while a turn of it sits inside. That turned
/// solution has to run and land on the commanded pose, and a target
/// that is out of range at every turn count has to stay refused —
/// wrapping is branch selection, never a way past the limits.
#[test]
fn ik_solutions_are_wrapped_into_their_soft_window() {
    let rig = boot_tagged("ikwrap");
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    let soft = soft_window_rad();
    let park = park_deg();

    // The pose of the turned posture, as the runtime itself reports it.
    let turned_deg = to_deg(TURNED_POSTURE_RAD);
    enable_and_teleport(&rig, &mut c, turned_deg);
    rig.drain_status();
    let at_posture = rig.wait_status("parked at the turned posture", |s| {
        angles_close(&s.angles, &turned_deg, 0.5)
    });
    let target = wire_pose_at(&at_posture.pose, tcp_mm(&at_posture));

    let mut seed_deg = turned_deg;
    seed_deg[5] = SEED_J6_RAD.to_degrees();
    enable_and_teleport(&rig, &mut c, seed_deg);
    let i = c.ok_index(&move_j_pose(9101, target, 8.0));
    let (ok, detail) = c.wait_complete(i);
    assert!(
        ok,
        "a solution that is inside every soft window after wrapping must run: {detail:?}"
    );
    let settled = settled_tcp(&rig, "the wrapped solution at rest");
    // The plan ends on the IK solution exactly, and once the profile's
    // feedforward decays to zero the driver's position loop closes the
    // remaining residual, so the landing is tight even at the CI tick
    // rate. A wrong-branch execution misses by decimeters.
    assert!(
        distance(tcp_mm(&settled), [target[0], target[1], target[2]]) < 3.0,
        "the wrapped solution must land on the commanded pose: {:?} vs {target:?}",
        tcp_mm(&settled)
    );
    for (j, angle_deg) in settled.angles.iter().enumerate() {
        let rad = angle_deg.to_radians();
        assert!(
            rad >= soft[j].0 - 1e-6 && rad <= soft[j].1 + 1e-6,
            "the arm parked outside joint {j}'s soft window: {rad} rad in {:?}",
            soft[j]
        );
    }
    assert!(
        (settled.angles[5] - turned_deg[5]).abs() < 1.0,
        "J6 turned onto the in-window branch: {} deg, wanted {}",
        settled.angles[5],
        turned_deg[5]
    );

    // Out of range at every turn count: J5 beyond its soft window, which
    // the wrist flip mirrors to the far side of the same window.
    let mut beyond_deg = park;
    beyond_deg[4] = BEYOND_SOFT_J5_RAD.to_degrees();
    enable_and_teleport(&rig, &mut c, beyond_deg);
    rig.drain_status();
    let at_beyond = rig.wait_status("parked past J5's soft window", |s| {
        angles_close(&s.angles, &beyond_deg, 0.5)
    });
    let refused_target = wire_pose_at(&at_beyond.pose, tcp_mm(&at_beyond));

    enable_and_teleport(&rig, &mut c, park);
    let before = tcp_mm(&rig.wait_status("pose before the out-of-range target", |_| true));
    let i = c.ok_index(&move_j_pose(9102, refused_target, 8.0));
    let (ok, detail) = c.wait_complete(i);
    assert!(!ok, "a target outside the soft window must stay refused");
    let e = detail.expect("a failed COMPLETE carries the error");
    assert_eq!(
        e.code,
        ErrorCode::CommValidationError as u16,
        "the refusal must name the soft-limit violation, got {e:?}"
    );
    let after = tcp_mm(&rig.wait_status("pose after the out-of-range target", |_| true));
    assert!(
        distance(before, after) < 1.0,
        "a refused target moved the arm: {before:?} -> {after:?}"
    );

    rig.shutdown();
}

/// A stream driven into a keep-out stays OUT of it and LANDS ON THE
/// STANDOFF — the same distance whatever speed it arrived at.
///
/// A position stream carries a target, not a rate, and the arm cannot
/// stop dead. Every datagram of a stepping stream is admissible on its
/// own target, so the arm builds speed toward the keep-out; when a
/// target finally lands inside it, the braking distance carries the TCP
/// on. A single held target never shows this — the limiter decelerates
/// to stop AT it — so the stream here advances the way a UI's does.
///
/// Refusing says only that the arm must not finish where it was asked
/// to. Left at that, where it ACTUALLY finishes is whatever the
/// executor's tracking lag and the datagram timing happened to leave:
/// measured on this rig at anywhere from 2.7 mm to 24 mm from a keep-out
/// whose standoff is 5 mm, varying run to run at one speed. So the
/// refusal is answered instead — the arm is brought to rest and then
/// placed on the standoff — and this asserts the result of that.
///
/// Both bounds bite. Below the clearance the arm has entered ground it
/// was configured to keep out of. Above it the gate is imposing a
/// standoff nobody asked for, and that surplus is workspace an arm
/// cannot use next to its own fixtures. Two speeds an order apart,
/// because a landing that depends on approach speed is a lag, not a
/// standoff.
///
/// RED, knowingly, and downstream of the drive: the fast leg rests at
/// 6.0 mm against 5.0 plus or minus 1.0 — repeatably, three runs of
/// three, which is itself new (it used to scatter). The surplus is two
/// terms. About 0.45 mm is [`STANDOFF_SETTLE_MARGIN_RAD`], which is
/// load-bearing and measured so: zeroed, the arm reaches 0.2 mm INSIDE
/// the keep-out on two runs of three, and once bailed out to 65 mm. The
/// other 0.55 mm is the coast after the placement's hold is dropped,
/// bounded by the speed the handover is gated on — and that gate cannot
/// go below the drive's own ring, which is what
/// `a_held_servo_target_settles` is about. Gate it at 1e-3 rad/s while
/// joint 1 still hunts and the placement never satisfies it, times out,
/// and leaves the arm 0.3 mm inside the keep-out having reached 4.2 mm
/// inside on the way. With joint 1 settled the same gate lands the arm
/// at 5.3 to 5.7 mm on three runs of three and lifts the closest
/// approach from 2.0 mm to 4.3.
///
/// So this goes green when the drive does. Two things not to retry:
/// trimming the settle margin (above), and judging a landing more
/// tightly than an arrival — below the coast every landing reads as a
/// miss, each retry creeps in and coasts back out, and the arm parks
/// where the retries ran out, measured at 9.8 mm.
#[test]
// Skipped on the owner's authorisation until there is bench time for the
// ring-frequency measurement the doc comment describes. It is not a flake
// and not weakened: it fails for a known reason, it still runs under
// `--include-ignored`, and it goes green when the drive does.
#[ignore = "the fast leg rests 1 mm outside the standoff because the handover gate \
            cannot sit below the drive's ring; goes green when a_held_servo_target_settles \
            does — see the doc comment"]
fn a_refused_servo_stream_lands_on_the_keep_out_standoff() {
    let rig = boot_tagged("servogate");
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    let mid_deg = with_j0(SWEEP_START_DEG, SWEEP_DEG / 2.0);
    let mid_m = tcp_at_m(mid_deg);
    let radius_m = (mid_m[0].powi(2) + mid_m[1].powi(2)).sqrt();
    let deg_per_m = 1.0_f64.to_degrees() / radius_m;
    let keepout = keepout_at("keepout", [mid_m[0] * 1e3, mid_m[1] * 1e3, mid_m[2] * 1e3]);
    c.ok(&set_shapes(vec![keepout]));

    /// Stream the target toward the box `step_mm` at a time until the
    /// gate latches, then report how close the TCP ever got to the box
    /// centre \[m\].
    /// Each run picks its own run-up: at 1 mm a datagram the fast run's
    /// full approach is thousands of datagrams of nothing happening, and
    /// the gate's answer does not depend on how much clear space came
    /// before it.
    struct Scene {
        mid_deg: [f64; NUM_JOINTS],
        deg_per_m: f64,
    }

    fn approach(
        rig: &Rig,
        c: &mut Client,
        scene: &Scene,
        col: &mut par6_kin::Collision,
        step_mm: f64,
        speed: Option<f64>,
        run_up_m: f64,
    ) -> (f64, f64) {
        let Scene { mid_deg, deg_per_m } = *scene;
        let start_deg = with_j0(mid_deg, -run_up_m * deg_per_m);
        // The gripper reaches ~96 mm past the TCP, so a run-up measured
        // from the box CENTRE can start the jaw already inside it — and
        // an approach that begins in the keep-out measures nothing about
        // approaching one.
        assert!(
            world_gap_m(col, start_deg) > par6d::COLLISION_CLEARANCE_M,
            "the run-up starts {:.1} mm from the keep-out, inside the standoff",
            world_gap_m(col, start_deg) * 1e3
        );
        enable_and_teleport(rig, c, start_deg);
        rig.drain_status();
        // `collision_active` is LATCHED: it describes the configuration
        // the last refused motion was blocked at, and the server holds it
        // until it accepts another motion command. An approach that
        // started while the previous one's latch still stood would read
        // "gated" on its first datagram and report the distance it began
        // at, which is why this measured a stationary arm at random.
        rig.wait_status("the collision latch to clear", |s| !s.collision_active);
        let step_deg = step_mm * 1e-3 * deg_per_m;
        let mut target = start_deg;
        let deadline = Instant::now() + BUDGET;
        let mut gated = false;
        let mut closest = f64::INFINITY;
        let mut last_seen = f64::NAN;
        // Kept up until the target reaches the box CENTRE, not stopped at
        // the first refusal. An operator dragging a jog does not let go
        // the instant a warning appears — they keep pulling, and the
        // question this test asks is where the arm ends up when they do.
        // Stopping at the first refusal instead measures the last place
        // the client happened to ask for, which on a fast host is well
        // outside the keep-out and says nothing about the standoff.
        while Instant::now() < deadline && target[0] < mid_deg[0] {
            target[0] = (target[0] + step_deg).min(mid_deg[0]);
            c.send(&Command::ServoJ(par6_proto::command::ServoJ {
                angles: target,
                speed,
                accel: None,
            }));
            let window = Instant::now() + Duration::from_millis(50);
            while Instant::now() < window {
                if let Some(s) = rig.recv_status() {
                    closest = closest.min(world_gap_m(col, s.angles));
                    last_seen = s.angles[0];
                    gated |= s.collision_active;
                }
            }
        }
        assert!(
            gated,
            "a stream driven into a keep-out was never gated: target reached \
             j0={:.3} deg of {:.3}, arm {:.3} deg, closest {:.2} mm",
            target[0],
            mid_deg[0],
            last_seen,
            closest * 1e3
        );
        // The travel after the refusal is the whole point, so keep
        // sampling until the arm has actually finished moving. A single
        // slow frame is not rest: the executor re-plans onto the
        // standoff and its velocity passes through zero on the way, so
        // rest is only believable once the arm has held still across
        // several consecutive frames.
        // A refusal is answered in two steps — the arm is brought to
        // rest, then placed on the standoff — so a pause is not the end
        // of the motion. Rest is only believable once the arm has held
        // still across a window WIDER than that pause.
        let settle = Instant::now() + Duration::from_secs(20);
        // Wider than the measured-braking phase budget, so a pause before
        // a residual correction cannot be read as the end of the motion.
        let quiet = Duration::from_secs(4);
        let mut rest = f64::NAN;
        let mut last = f64::NAN;
        let mut moved_at = Instant::now();
        while Instant::now() < settle && moved_at.elapsed() < quiet {
            let Some(s) = rig.recv_status() else { continue };
            let gap = world_gap_m(col, s.angles);
            closest = closest.min(gap);
            rest = gap;
            if s.speeds.iter().any(|v| v.abs() >= 0.005) || (s.angles[0] - last).abs() >= 1e-4 {
                moved_at = Instant::now();
            }
            last = s.angles[0];
        }
        assert!(
            moved_at.elapsed() >= quiet,
            "the arm never came to rest after the refusal"
        );
        (closest, rest)
    }

    let scene = Scene { mid_deg, deg_per_m };
    let mut col = keepout_world(mid_m);
    let (fast_closest, streamed) =
        approach(&rig, &mut c, &scene, &mut col, 5.0, None, 2.0 * KEEPOUT_M);
    println!(
        "fast stream: rest {:.1} mm, closest {:.1} mm",
        streamed * 1e3,
        fast_closest * 1e3
    );
    // The same rig, crawling: slow enough that its stopping distance is
    // nearly nothing, so where it is refused is where the geometry alone
    // forbids the next step.
    //
    // On a rig of its own, because an approach ends with the arm held
    // inside the gate's refusal and a teleport back out of that never
    // takes — a second run in the same daemon cannot be positioned to
    // start. The first rig has to go DOWN before the second comes up:
    // `RT_SLOT` admits one daemon at a time, so two live rigs deadlock.
    rig.shutdown();
    let floor_rig = boot_tagged("servofloor");
    let mut floor_c = Client::new(floor_rig.addr());
    floor_rig.wait_status("link_ok", |s| s.link_ok == 1);
    floor_c.ok(&Command::Reset);
    floor_c.ok(&set_shapes(vec![keepout_at(
        "keepout",
        [mid_m[0] * 1e3, mid_m[1] * 1e3, mid_m[2] * 1e3],
    )]));
    let (crawl_closest, crawled) = approach(
        &floor_rig,
        &mut floor_c,
        &scene,
        &mut col,
        1.0,
        None,
        2.0 * KEEPOUT_M,
    );
    floor_rig.shutdown();
    println!(
        "crawl: rest {:.1} mm, closest {:.1} mm",
        crawled * 1e3,
        crawl_closest * 1e3
    );

    // THE REQUIREMENT: a motion driven into a keep-out lands on the
    // clearance. Not "outside it somewhere" — on it. The clearance is
    // the standoff the machine is configured to hold, so where the arm
    // finishes is a property of the geometry, and approach speed must
    // not change it.
    //
    // Both bounds bite. Below the clearance the arm has entered ground
    // it was configured to keep out of. Above it the gate is imposing a
    // standoff nobody asked for, and that surplus is workspace an arm
    // cannot use next to its own fixtures.
    let clearance = par6d::COLLISION_CLEARANCE_M;
    let tol = 0.001;
    // The keep-out itself, not just the resting place: whatever speed the
    // stream built, the arm must never be inside the box on the way. The
    // clearance is the budget for the coast, and it has to be a budget
    // rather than an overdraft.
    println!(
        "closest approach: fast {:.1} mm, crawl {:.1} mm",
        fast_closest * 1e3,
        crawl_closest * 1e3
    );
    for (what, closest) in [("fast stream", fast_closest), ("crawl", crawl_closest)] {
        assert!(
            closest > 0.0,
            "the {what} put the arm {:.1} mm INSIDE the keep-out on its way to rest",
            -closest * 1e3
        );
    }
    for (what, rest) in [("fast stream", streamed), ("crawl", crawled)] {
        assert!(
            (rest - clearance).abs() <= tol,
            "the {what} came to rest {:.1} mm from the keep-out; it must \
             land on the {:.1} mm clearance (within {:.1} mm). {}",
            rest * 1e3,
            clearance * 1e3,
            tol * 1e3,
            if rest < clearance {
                "The gate stopped it too late and it entered the standoff."
            } else {
                "The gate stopped it early, costing usable workspace."
            }
        );
    }
}

/// A servo target the client holds is a position the arm SETTLES on.
///
/// Every promise the streaming collision gate makes is a promise about
/// where the arm ends up, and none of them can hold if a held setpoint
/// is not a place the arm can rest. This is the floor under all of them,
/// and it is asserted here rather than inside a gate test so a failure
/// names the control loop instead of the keep-out.
///
/// The bound is the machine's own `[motion] settle_tolerance_rad` — what
/// the config declares "arrived" means — read from the config the rig
/// booted rather than restated here.
///
/// This extended pose exercises the base joint's high-inertia load. The
/// configured drive gains must settle it without changing the machine's
/// arrival tolerance; otherwise a keep-out landing has no stable endpoint.
#[test]
fn a_held_servo_target_settles() {
    let tol_rad = par6_config::RobotConfig::load(&common::shipped_config())
        .expect("shipped config")
        .motion
        .settle_tolerance_rad;
    let rig = boot_tagged("servosettle");
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);
    enable_and_teleport(&rig, &mut c, SWEEP_START_DEG);
    rig.drain_status();

    // Five seconds of the runtime's own clock, the target re-sent on
    // every other frame — well inside the stream watchdog.
    let target = with_j0(SWEEP_START_DEG, 20.0);
    let hold = || {
        Command::ServoJ(par6_proto::command::ServoJ {
            angles: target,
            speed: None,
            accel: None,
        })
    };
    c.send(&hold());
    let first = rig.wait_status("the hold started", |_| true);
    let mut trace: Vec<(u64, f64)> = Vec::new();
    while trace
        .last()
        .is_none_or(|(t, _)| *t < first.mono_time_ns + 5_000_000_000)
    {
        if let Some(s) = rig.recv_status() {
            if s.seq % 2 == 0 {
                c.send(&hold());
            }
            trace.push((s.mono_time_ns, s.angles[0]));
        }
    }
    rig.shutdown();

    // The last second of the hold: by then the arm has had four seconds
    // to cover twenty degrees.
    let last_t = trace.last().expect("status during the hold").0;
    let tail: Vec<f64> = trace
        .iter()
        .filter(|(t, _)| *t + 1_000_000_000 >= last_t)
        .map(|(_, a)| *a)
        .collect();
    assert!(tail.len() > 10, "no status stream to judge");
    let lo = tail.iter().cloned().fold(f64::INFINITY, f64::min);
    let hi = tail.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let tol_deg = tol_rad.to_degrees();
    println!(
        "held target {target:?}; last {} of {} observed positions: {tail:?}",
        tail.len(),
        trace.len()
    );
    assert!(
        hi - lo <= tol_deg,
        "a held servo target left the joint swinging {:.3} deg peak to peak \
         (settle tolerance {tol_deg:.3} deg): the arm is not resting on it",
        hi - lo
    );
    assert!(
        (tail[tail.len() - 1] - target[0]).abs() <= tol_deg,
        "a held servo target left the joint {:.3} deg away from it \
         (settle tolerance {tol_deg:.3} deg)",
        tail[tail.len() - 1] - target[0]
    );
}

// ---- curved moves: the arm ON the plan ------------------------------------
// The GEOMETRY these moves name is asserted on the planner's own output in
// `curved_geometry.rs`, to a tenth of the tolerance a live measurement can
// carry and in milliseconds. What is left here is the half that needs an
// arm: that the runtime drives the plan it made, end to end over the wire,
// and the TCP arrives where the plan said. One motion per family.

/// The bound on a live path measurement: the simulated arm's steady-state
/// tracking lag at a curvature peak, not the planner's error.
const TRACK_TOL_MM: f64 = 12.0;

/// `move_c` drives the arm around the circle it planned.
#[test]
fn move_c_tracks_its_planned_arc() {
    const R: f64 = ARC_RADIUS_MM;
    let rig = boot_tagged("curved-arc");
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    let s = curve_start(&rig, &mut c);
    let start = tcp_mm(&s);
    let center = [start[0] + R, start[1], start[2]];
    let via = [center[0], center[1], center[2] - R];
    let end = [center[0] + R, center[1], center[2]];
    let i = c.ok_index(&Command::MoveC(MoveC {
        key: 4001,
        via: wire_pose_at(&s.pose, via),
        end: wire_pose_at(&s.pose, end),
        frame: Frame::Wrf,
        duration: Some(MOVE_S),
        speed: None,
        accel: None,
        blend_radius: None,
        rel: false,
    }));
    let arc: Vec<[f64; 3]> = rig
        .collect_through(i, Duration::from_secs_f64(MOVE_S + 1.0))
        .iter()
        .map(tcp_mm)
        .collect();
    let (ok, detail) = c.wait_complete(i);
    assert!(ok, "move_c must complete ok, got {detail:?}");

    let moving: Vec<[f64; 3]> = arc
        .iter()
        .copied()
        .filter(|p| distance(*p, start) > 3.0)
        .collect();
    assert!(
        moving.len() > 50,
        "expected a sampled arc, got {} moving samples",
        moving.len()
    );
    let radial = moving
        .iter()
        .map(|p| (distance(*p, center) - R).abs())
        .fold(0.0f64, f64::max);
    assert!(
        radial < TRACK_TOL_MM,
        "the arm left the planned circle by {radial:.2} mm (radius {R} mm about {center:?})"
    );
    let via_miss = path_misses(&arc, via);
    assert!(
        via_miss < TRACK_TOL_MM,
        "the arm passed {via_miss:.2} mm from the planned via point"
    );
    let end_miss = distance(tcp_mm(&settled_tcp(&rig, "settled after move_c")), end);
    assert!(
        end_miss < 15.0,
        "move_c ended {end_miss:.1} mm off its end pose"
    );
    rig.shutdown();
}

/// `move_s` drives the arm through the waypoints it planned.
#[test]
fn move_s_tracks_its_planned_spline() {
    let rig = boot_tagged("curved-spline");
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    let s = curve_start(&rig, &mut c);
    let start = tcp_mm(&s);
    let waypoints = spline_waypoints(start);
    let i = c.ok_index(&Command::MoveS(MoveS {
        key: 4002,
        waypoints: waypoints
            .iter()
            .map(|p| wire_pose_at(&s.pose, *p))
            .collect(),
        frame: Frame::Wrf,
        duration: Some(SPLINE_S),
        speed: None,
        accel: None,
        rel: false,
    }));
    let spline: Vec<[f64; 3]> = rig
        .collect_through(i, Duration::from_secs_f64(SPLINE_S + 1.0))
        .iter()
        .map(tcp_mm)
        .collect();
    let (ok, detail) = c.wait_complete(i);
    assert!(ok, "move_s must complete ok, got {detail:?}");

    let last = *waypoints.last().expect("waypoints");
    for (k, w) in waypoints.iter().enumerate() {
        let miss = path_misses(&spline, *w);
        assert!(
            miss < TRACK_TOL_MM,
            "the arm passed {miss:.2} mm from planned waypoint {k} ({w:?})"
        );
    }
    let end_miss = distance(tcp_mm(&settled_tcp(&rig, "settled after move_s")), last);
    assert!(
        end_miss < 15.0,
        "move_s ended {end_miss:.1} mm off its last waypoint"
    );
    rig.shutdown();
}

/// `move_p` drives the arm through its rounded corner without stopping in
/// it — the one claim that needs a moving arm rather than a plan, since a
/// blend that decelerates to zero still traces the right shape.
#[test]
fn move_p_tracks_its_corner_without_stopping_in_it() {
    let rig = boot_tagged("curved-process");
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    let s = curve_start(&rig, &mut c);
    let start = tcp_mm(&s);
    let (corner, finish) = process_corner(start);
    let i = c.ok_index(&Command::MoveP(MoveP {
        key: 4003,
        waypoints: vec![wire_pose_at(&s.pose, corner), wire_pose_at(&s.pose, finish)],
        frame: Frame::Wrf,
        duration: Some(MOVE_S),
        speed: None,
        accel: None,
        rel: false,
    }));
    let process = rig.collect_through(i, Duration::from_secs_f64(MOVE_S + 1.0));
    let (ok, detail) = c.wait_complete(i);
    assert!(ok, "move_p must complete ok, got {detail:?}");

    let points: Vec<[f64; 3]> = process.iter().map(tcp_mm).collect();
    let corner_miss = path_misses(&points, corner);
    assert!(
        (2.0..25.0).contains(&corner_miss),
        "the arm did not track the planned rounding: closest approach to the corner \
         {corner_miss:.2} mm"
    );
    let (at_corner, mean) = corner_and_mean_speed(&process, corner, 30.0);
    assert!(
        at_corner > 0.25 * mean,
        "move_p slowed to {at_corner:.2} mm/s at the corner against a mean of {mean:.2} mm/s: \
         a blend that stops is not a blend"
    );
    let end_miss = distance(tcp_mm(&settled_tcp(&rig, "settled after move_p")), finish);
    assert!(
        end_miss < 15.0,
        "move_p ended {end_miss:.1} mm off its last waypoint"
    );
    rig.shutdown();
}

/// A move queued behind another is re-guarded when it ACTIVATES, not
/// only when it was accepted.
///
/// The first move runs clear; the keep-out lands on the second's path
/// while the first is still under way. Accepted against an empty world,
/// the second must still be refused where it would have started, and
/// the arm must stay where the first move left it.
#[test]
fn a_queued_move_is_re_guarded_against_the_world_when_it_activates() {
    let rig = boot_tagged("collision-queued");
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);
    let quarter_deg = with_j0(SWEEP_START_DEG, SWEEP_DEG / 4.0);
    let mid_deg = with_j0(SWEEP_START_DEG, SWEEP_DEG / 2.0);
    let end_deg = with_j0(SWEEP_START_DEG, SWEEP_DEG);

    let mid_m = tcp_at_m(mid_deg);
    let mid_tcp = [mid_m[0] * 1e3, mid_m[1] * 1e3, mid_m[2] * 1e3];

    enable_and_teleport(&rig, &mut c, SWEEP_START_DEG);
    let i1 = c.ok_index(&move_j(8201, quarter_deg, SWEEP_S / 2.0));
    let i2 = c.ok_index(&move_j(8202, end_deg, SWEEP_S));
    rig.drain_status();
    rig.wait_status("the first move is under way", |s| {
        s.executing_index == i1 as i64 && s.angles[0] > SWEEP_START_DEG[0] + 2.0
    });
    c.ok(&set_shapes(vec![keepout_at("late-wall", mid_tcp)]));

    let (ok, detail) = c.wait_complete(i1);
    assert!(ok, "the clear first move must complete, got {detail:?}");
    let (ok, detail) = c.wait_complete(i2);
    assert!(
        !ok,
        "the second move was accepted against an empty world and must be refused \
         against the one it starts in"
    );
    let e = detail.expect("a failed COMPLETE carries the error");
    assert_eq!(e.code, ErrorCode::SysSelfCollision as u16, "{e:?}");
    assert!(
        e.cause.contains("late-wall"),
        "the refusal must name the keep-out that arrived late: {e:?}"
    );
    let s = settled_tcp(&rig, "the arm at rest");
    assert!(
        angles_close(&s.angles, &quarter_deg, 1.0),
        "the arm must stay where the first move left it, not stream the second: {:?}",
        s.angles
    );
    rig.shutdown();
}

/// The command plane stays answerable while the planner is working.
///
/// Regression: the planner ran inside the server's one `select!` loop —
/// the same loop that receives datagrams and emits STATUS. Two things
/// there are expensive and neither is bounded: planning a cartesian
/// chain (seeded IK per waypoint, a TOPPRA retiming, a collision walk),
/// and the enablement probe the planner runs whenever the arm has moved,
/// which its own comment prices at up to 50 ms and which repeats every
/// 100 ms. For the whole of either, the socket went unread and no STATUS
/// went out: a jog waited in the kernel buffer, a software STOP waited
/// behind it, and the operator's readout froze.
///
/// The observable is the broadcast, because it is the one thing that
/// must keep arriving whatever the planner is doing — a plane that
/// cannot broadcast could not have read a datagram either. A PING in the
/// same window says it from the receive side.
///
/// The path is deliberately awkward to plan, and the test asserts that
/// it WAS: a cheap plan proves nothing about a plane that blocks on
/// expensive ones, so the premise is checked rather than assumed.
#[test]
fn planning_does_not_stall_the_command_plane() {
    let rig = boot_tagged("plane-responsive");
    let mut c = Client::new(rig.addr());
    let s = curve_start(&rig, &mut c);
    let start = tcp_mm(&s);

    // Ninety-six right-angle corners, a descending square spiral: every
    // corner is an IK solve and a blend, and the retimer has to search
    // hard across them. Not reversals: a path that doubles back on itself
    // has a cusp the retimer cannot give a speed to, and the move is
    // refused instead of planned.
    let square = [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)];
    let waypoints: Vec<[f64; 6]> = (0..96)
        .map(|i| {
            let (dx, dy) = square[i % 4];
            let z = start[2] - f64::from(i as u32);
            wire_pose_at(&s.pose, [start[0] + dx, start[1] + dy, z])
        })
        .collect();

    // From the config the rig booted, so the bound cannot drift away
    // from the cadence it is judging.
    let status_hz = par6_config::RobotConfig::load(&common::shipped_config())
        .expect("shipped config")
        .protocol
        .status_rate_hz;
    let nominal = Duration::from_secs_f64(1.0 / f64::from(status_hz));
    rig.set_status_timeout(Duration::from_secs(2));
    rig.drain_status();
    rig.wait_status("a frame before the plan", |_| true);

    let queued = Instant::now();
    let i = c.ok_index(&Command::MoveP(MoveP {
        key: 9100,
        waypoints,
        frame: Frame::Wrf,
        duration: Some(MOVE_S),
        speed: None,
        accel: None,
        rel: false,
    }));
    // The ack precedes planning by construction, so a slow one would mean
    // the plane was already blocked when the datagram arrived.
    let ack = queued.elapsed();
    assert!(
        ack < nominal * 10,
        "the queue ack took {ack:?}, which is not the enqueue it is supposed to be"
    );

    // Watch until the move starts executing, so the window is the
    // planning window rather than a guess about it — and a path the
    // planner refused never starts, so starting is the plan succeeding.
    let mut worst = Duration::ZERO;
    let mut ping_worst = Duration::ZERO;
    let mut last = Instant::now();
    let deadline = Instant::now() + BUDGET;
    let mut started = false;
    while !started {
        assert!(Instant::now() < deadline, "the command never started");
        if let Some(s) = rig.recv_status() {
            worst = worst.max(last.elapsed());
            last = Instant::now();
            started = s.executing_index == i as i64;
        }
        let t = Instant::now();
        let _ = c.query(&Command::Ping);
        ping_worst = ping_worst.max(t.elapsed());
    }
    let planned_for = queued.elapsed();
    assert!(
        planned_for > Duration::from_millis(300),
        "this path resolved in {planned_for:?}; it is meant to be the expensive \
         case, and a cheap one cannot show whether the plane blocks on expensive \
         ones — give it a harder path rather than deleting this check"
    );

    let ceiling = nominal * 4;
    assert!(
        worst < ceiling,
        "STATUS stalled for {worst:?} while the planner worked (period {nominal:?}); \
         the command plane is blocked on the planner"
    );
    assert!(
        ping_worst < ceiling,
        "a PING took {ping_worst:?} while the planner worked; the plane was not \
         reading its socket"
    );
}
