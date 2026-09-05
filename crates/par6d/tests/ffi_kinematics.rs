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
    JogJ, JogL, MoveC, MoveJ, MoveJPose, MoveL, MoveP, MoveS, SetPayload, SetShapes, SetTcpOffset,
    Shape, Stop, Teleport,
};
use par6_proto::{Command, ControllerMode, ErrorCode, Frame, QueryResult, Status, NUM_JOINTS};

use par6d::options::StatusTransport;
use par6d::{Daemon, Options};

mod common;
use common::{
    distance, distance_to_segment, path_misses, process_corner, progress_along, shipped_config,
    spline_waypoints, wire_pose_at, Client, Rig, ARC_RADIUS_MM, BUDGET, CURVE_START_DEG,
};

/// Boot on a config patched for this test's `tag`, so parallel tests do
/// not share a temp config directory.
fn boot_tagged(tag: &str, sim_dynamics: bool) -> Rig {
    Rig::boot_with(test_config(tag), sim_dynamics)
}

/// The PAR6 config re-ticked to 50 Hz, like the sim-session test: loaded
/// CI machines without RT scheduling miss 4 ms deadlines and would latch
/// LOOP_CRITICAL. Every RT time constant derives from config seconds, so
/// the wiring under test is identical.
fn test_config(tag: &str) -> PathBuf {
    common::retimed_config(&format!("ffi-{tag}"), 0.02)
}

/// [`test_config`] with the active (MSG) gripper's `[kinematics] mass_kg`
/// replaced — the knob the gravity-wiring test turns.
fn test_config_with_tool_mass(tag: &str, mass_kg: f64) -> PathBuf {
    let dst = test_config(tag);
    let toml = dst
        .parent()
        .unwrap()
        .join("grippers/MSG_small_motor_150mm_rail.toml");
    let text = std::fs::read_to_string(&toml).expect("gripper toml");
    // contains(), not assert_ne on the output: the baseline call passes
    // the default 0.37, whose patch is a no-op by construction.
    assert!(
        text.contains("mass_kg = 0.37"),
        "mass_kg patch point must exist"
    );
    let patched = text.replace("mass_kg = 0.37", &format!("mass_kg = {mass_kg}"));
    std::fs::write(&toml, patched).expect("write gripper toml");
    dst
}

// ---- in-process rig --------------------------------------------------------

/// The 50 Hz config with the `[freedrive]` drift lock switched on. The
/// torque plant has no stiction, so a model bias that a real gearbox
/// would turn into a slow creep moves the arm at a rate a hand push
/// would; the release speed is raised past that, so the lock arms on the
/// drifting arm after its settle window instead of reading the drift as
/// an operator push.
fn drift_lock_config(tag: &str) -> PathBuf {
    let dst = test_config(tag);
    let text = std::fs::read_to_string(&dst).expect("read test config");
    let mut patched = text.clone();
    for (from, to) in [
        ("drift_lock = false", "drift_lock = true"),
        ("release_rad_s = 0.08", "release_rad_s = 1.0"),
        ("settle_s = 0.3", "settle_s = 0.2"),
    ] {
        assert!(patched.contains(from), "patch point {from:?} must exist");
        patched = patched.replace(from, to);
    }
    std::fs::write(&dst, patched).expect("write drift-lock config");
    dst
}

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
    let gripper = bundle
        .robot
        .robot
        .active_gripper
        .trim()
        .to_ascii_uppercase();
    let variant = par6_kin::GripperVariant::resolve(
        &gripper,
        bundle
            .active_gripper()
            .and_then(|g| g.urdf_variant.as_deref()),
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
    let rig = boot_tagged("cart", false);
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
    let path: Vec<[f64; 3]> = rig
        .collect_through(i, Duration::from_secs_f64(MOVE_S + 1.0))
        .iter()
        .map(tcp_mm)
        .collect();
    let (ok, detail) = c.wait_complete(i);
    assert!(ok, "move_l must complete ok, got {detail:?}");

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
        .collect_through(i, Duration::from_secs_f64(MOVE_S + 1.0))
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

/// The gravity hook is wired, signed right, and survives the Nm→mA→Nm
/// round trip. On the torque-level plant (`--sim-dynamics`) an IDLE arm
/// is held by nothing but the G(q) feedforward: every loaded joint,
/// wrist included, stays where the sim placed it. With the `ZeroGravity`
/// placeholder the same rig collapses — measured here, the shoulder is
/// 69° down and the elbow 108° over inside one second — so this bound
/// cannot pass without the hook.
///
/// What this does NOT establish is physical truth: the plant is built
/// from the same URDF the gravity model reads, so it measures internal
/// consistency and would pass unchanged with every link mass halved.
/// The external half of the claim lives in
/// `par6-kin/tests/gravity_reference.rs`, which pins G(q) on these URDFs
/// to the VENDOR's dynamics table, independent of any URDF — the pair
/// together is the whole statement.
#[test]
fn gravity_hook_holds_the_arm_on_the_torque_plant() {
    /// Hold tolerance \[deg\] for every joint.
    const HOLD_TOL: f64 = 2.5;
    /// Every joint with a gravity load: the shoulder and elbow carry
    /// essentially the whole arm, the wrist joints carry a residual
    /// small enough that only a faithful torque↔current path holds
    /// them (J0 is on the vertical axis and carries nothing).
    const LOADED: [usize; 5] = [1, 2, 3, 4, 5];

    let rig = boot_tagged("gravity", true);
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);

    let placed = HOLD_POSE_DEG;
    c.ok(&Command::Reset);
    enable_and_teleport(&rig, &mut c, placed);

    // The loaded joints give way slightly before the feedforward
    // catches them; the hold is what happens after that.
    let settle = rig.collect_status(Duration::from_secs(1));
    let held = settle.last().expect("status while settling").clone();
    let give = LOADED
        .iter()
        .map(|&j| (held.angles[j] - placed[j]).abs())
        .fold(0.0f64, f64::max);
    assert!(
        give > 0.05,
        "the plant never loaded the arm ({give:.3}°) — gravity is not being simulated"
    );
    assert!(
        give < 10.0,
        "the arm left the pose it was placed at ({give:.2}°): {placed:?} -> {:?}",
        held.angles
    );

    let watch = rig.collect_status(Duration::from_secs(4));
    assert!(
        watch.len() > 40,
        "expected a stream of status during the hold, got {}",
        watch.len()
    );
    for s in &watch {
        assert!(
            s.error.is_none(),
            "unexpected error while holding: {:?}",
            s.error
        );
        for &j in &LOADED {
            assert!(
                (s.angles[j] - held.angles[j]).abs() < HOLD_TOL,
                "joint {j} moved {:.2}° off the held pose while IDLE ({:?} -> {:?})",
                s.angles[j] - held.angles[j],
                held.angles,
                s.angles
            );
        }
    }

    rig.shutdown();
}

/// The `[freedrive]` drift lock against a biased gravity model on the
/// torque plant. A runtime payload the plant does not carry IS that
/// bias — the controller lifts a mass that is not there — and an IDLE
/// arm under it rises for as long as nothing stops it. With the lock
/// configured the same bias is caught inside the settle window and held
/// by the drive's impedance frame plus the clamped integral.
#[test]
fn the_drift_lock_bounds_the_drift_of_a_biased_gravity_model() {
    const WINDOW: Duration = Duration::from_secs(2);
    const LOADED: [usize; 5] = [1, 2, 3, 4, 5];
    let payload = SetPayload {
        mass: 0.2,
        com: [0.0, 0.0, 0.02],
        inertia: None,
    };

    let drift_deg = |config: PathBuf| -> [f64; NUM_JOINTS] {
        let rig = Rig::boot_with(config, true);
        let mut c = Client::new(rig.addr());
        rig.wait_status("link_ok", |s| s.link_ok == 1);
        c.ok(&Command::Reset);
        enable_and_teleport(&rig, &mut c, HOLD_POSE_DEG);
        // Held by the true model first, so the bias is the only change.
        let start = rig
            .collect_status(Duration::from_secs(1))
            .last()
            .expect("status while settling")
            .angles;
        c.ok(&Command::SetPayload(payload.clone()));
        let watch = rig.collect_status(WINDOW);
        let end = watch.last().expect("status during the drift window");
        assert!(
            end.mode == ControllerMode::Idle && end.enabled && end.gravity_comp,
            "the arm must be in freedrive throughout: {end:?}"
        );
        assert!(end.error.is_none(), "unexpected error: {:?}", end.error);
        let drift: [f64; NUM_JOINTS] = std::array::from_fn(|j| (end.angles[j] - start[j]).abs());
        rig.shutdown();
        drift
    };
    let worst = |d: &[f64; NUM_JOINTS]| LOADED.iter().map(|&j| d[j]).fold(0.0f64, f64::max);

    let free_per_joint = drift_deg(test_config("drift-free"));
    let locked_per_joint = drift_deg(drift_lock_config("drift-locked"));
    let (free, locked) = (worst(&free_per_joint), worst(&locked_per_joint));
    assert!(
        free > 5.0,
        "the biased model must visibly move an unlocked arm in {WINDOW:?}; \
         drifted {free:.2}° ({free_per_joint:.2?})"
    );
    assert!(
        locked < 2.0 && locked < free / 3.0,
        "the lock must bound the drift: locked {locked:.2}° vs free {free:.2}° \
         (per joint: locked {locked_per_joint:.2?}, free {free_per_joint:.2?})"
    );
}

#[test]
fn a_full_speed_move_lands_cleanly_on_the_torque_plant() {
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
    let rig = boot_tagged("tauff", true);
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
    // This tier holds with degrees of steady-state give (see the hold
    // test above), so the landing tolerance asks "did the trajectory
    // arrive", not "did the servo null out" — a feedforward with the
    // wrong sign or scale misses by tens of degrees or latches an error.
    let s = rig.wait_status("landed on the target", |s| {
        angles_close(&s.angles, &target, 8.0)
    });
    assert!(
        s.error.is_none(),
        "standing error after the move: {:?}",
        s.error
    );

    rig.shutdown();
}

// ---- gravity reads the gripper config --------------------------------------

/// The gravity model reads the gripper CONFIG, not just the URDF.
///
/// Boot plain `--sim` twice; the only difference is the active gripper's
/// `[kinematics] mass_kg` (0.37 kg stock vs 2.37 kg — a tool two kilos
/// heavier). At the same teleported posture the published external
/// torque must shift by the extra tool weight: about 6.7 Nm at the
/// shoulder and 0.3 Nm at the wrist pitch for these numbers.
///
/// STATUS publishes `torques_ext = measured − G(q)`, and the kinematic
/// plant measures no torque, so what arrives is the model's own gravity
/// with the sign flipped — which is why the magnitudes below are read
/// off `torques_ext` directly.
///
/// Failing before the wiring landed, twice over: par6d built its gravity
/// model with `tool: None`, so `mass_kg` was parsed, validated and read
/// by nothing — the promised "masses/COM/inertia from config" was false —
/// and plain `--sim` installed `ZeroGravity`, so the same field
/// published all-zero torques no matter the model. (The feedforward is
/// still never APPLIED on the kinematic plant: comp is disabled at boot,
/// publish-only.)
#[test]
fn gripper_config_mass_changes_published_gravity_torque() {
    fn published_gravity(tag: &str, mass_kg: f64) -> ([f64; NUM_JOINTS], [f64; NUM_JOINTS]) {
        let rig = Rig::boot_with(test_config_with_tool_mass(tag, mass_kg), false);
        let mut c = Client::new(rig.addr());
        rig.wait_status("link_ok", |s| s.link_ok == 1);
        c.ok(&Command::Reset);
        enable_and_teleport(&rig, &mut c, CART_START_DEG);
        let at = rig.wait_status("at the probe posture", |s| {
            angles_close(&s.angles, &CART_START_DEG, 0.1)
                && s.torques_ext.iter().any(|t| t.abs() > 1e-9)
        });
        rig.shutdown();
        (at.angles, at.torques_ext)
    }

    let (q_stock, g_stock) = published_gravity("grav-stock", 0.37);
    let (q_heavy, g_heavy) = published_gravity("grav-heavy", 2.37);
    assert!(
        angles_close(&q_stock, &q_heavy, 0.2),
        "the two runs must be compared at the same posture: {q_stock:?} vs {q_heavy:?}"
    );

    // Plain --sim publishes the real model, not placeholder zeros: the
    // shoulder carries most of the arm at this posture.
    assert!(
        g_stock[1].abs() > 2.0,
        "published shoulder gravity torque is {:.3} Nm — the kinematic-sim \
         runtime is publishing a placeholder, not G(q)",
        g_stock[1]
    );
    // The config knob reaches the published torque, at the joints the
    // extra tool mass actually loads.
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
    // And J0 stays gravity-free: its axis is vertical, so a value here
    // means the tool was attached in the wrong frame.
    assert!(
        g_heavy[0].abs() < 1e-6,
        "J0 is on the vertical axis and must carry no gravity torque, got {:.6}",
        g_heavy[0]
    );
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
    let rig = boot_tagged("collision", false);
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
    // kind, a dimension coal cannot build, and a name already taken.
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
    let duplicate = keepout.clone();
    for (label, bad) in [
        ("unknown kind", unknown_kind),
        ("wrong arity", short_params),
        ("negative radius", negative),
        ("duplicate name", duplicate),
    ] {
        let err = c.expect_error(&set_shapes(vec![keepout.clone(), bad]));
        assert_eq!(
            err.code,
            ErrorCode::CommValidationError as u16,
            "a {label} shape must be refused, got {err:?}"
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

    // A world change does not spare motion already committed: drop the
    // keep-out onto the path of a move that is already running and it
    // stops, instead of being enforced only from the next command on.
    c.ok(&set_shapes(Vec::new()));
    enable_and_teleport(&rig, &mut c, SWEEP_START_DEG);
    let i = c.ok_index(&move_j(7004, end_deg, SWEEP_S));
    rig.drain_status();
    rig.wait_status("the sweep is under way but short of the keep-out", |s| {
        s.executing_index == i as i64
            && s.angles[0] > SWEEP_START_DEG[0] + 3.0
            && s.angles[0] < -10.0
    });
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
    let (ok, detail) = c.wait_complete(i);
    assert!(
        ok,
        "the sweep must run once the keep-out is removed, got {detail:?}"
    );
    rig.drain_status();
    let s = rig.wait_status("a clean move clears the verdict", |_| true);
    assert!(
        !s.collision_active && s.collision_pairs.is_empty(),
        "the refusal's pairs outlived the motion that caused them: {:?}",
        s.collision_pairs
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

/// The TCP position at `angles_deg` \[m\], from the same URDF the runtime
/// loads — where a keep-out has to go to sit on the swept path.
fn tcp_at_m(angles_deg: [f64; NUM_JOINTS]) -> [f64; 3] {
    // One model per thread: a status-rate sampler cannot afford a URDF
    // load per frame.
    thread_local! {
        static KIN: std::cell::RefCell<Option<par6_kin::Kin>> =
            const { std::cell::RefCell::new(None) };
    }
    KIN.with(|cell| {
        let mut slot = cell.borrow_mut();
        let kin = slot.get_or_insert_with(|| {
            par6_kin::Kin::load(&common::assets_dir(), par6_kin::GripperVariant::Msg)
                .expect("kin model")
        });
        let mut q = [0.0; NUM_JOINTS];
        for (out, deg) in q.iter_mut().zip(angles_deg.iter()) {
            *out = deg.to_radians();
        }
        let mut pose = [0.0; 16];
        kin.fk(&q, &mut pose).expect("fk");
        [pose[3], pose[7], pose[11]]
    })
}

/// The configured JOG-mode velocity limit of J0 \[rad/s\] — what a jog
/// `speeds` fraction commands, and what the gate's lookahead projects.
fn j0_jog_velocity() -> f64 {
    let cfg = par6_config::RobotConfig::load(&shipped_config()).expect("PAR6 config");
    cfg.joints[0]
        .limits
        .for_mode(par6_config::LimitMode::Jog)
        .velocity_rad_s
}

/// Streaming motion is gated by the same collision world as planned
/// motion (issue #19 gap 1 + gap 2), over the real protocol against the
/// real coal world and the real RT jog engine:
///
/// - a jog TOWARD a keep-out stops short of it: the gate's velocity-
///   scaled lookahead predicts the contact, the stream is stopped, and
///   STATUS latches `collision_active` with the keep-out named — the arm
///   never reaches the box;
/// - from INSIDE the keep-out (dropped over the arm), a jog moving
///   OUTWARD is permitted — the arm demonstrably escapes;
/// - from a SHALLOW penetration, a jog driving DEEPER is refused with a
///   real `SYS_SELF_COLLISION` ERROR reply (the escape-depth rule: same
///   pairs, deeper penetration — the pair-set check alone cannot catch
///   it), while the outward jog from the same spot runs.
#[test]
fn streaming_is_gated_by_the_collision_world() {
    let rig = boot_tagged("streamgate", false);
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    let mid_deg = with_j0(SWEEP_START_DEG, SWEEP_DEG / 2.0);
    let mid_m = tcp_at_m(mid_deg);
    // The J0 arc the TCP travels on: converts arc metres to J0 radians.
    let radius_m = (mid_m[0].powi(2) + mid_m[1].powi(2)).sqrt();
    let deg_per_m = 1.0_f64.to_degrees() / radius_m;
    let v0 = j0_jog_velocity();

    let keepout = keepout_at("keepout", [mid_m[0] * 1e3, mid_m[1] * 1e3, mid_m[2] * 1e3]);
    c.ok(&set_shapes(vec![keepout.clone()]));

    // --- a jog toward the keep-out stops short of it.
    // Start with the TCP two box widths from the box centre, jog toward
    // it at half speed, and let the periodic re-check catch the approach.
    let start_deg = with_j0(mid_deg, -2.0 * KEEPOUT_M * deg_per_m);
    enable_and_teleport(&rig, &mut c, start_deg);
    rig.drain_status();
    c.send(&jog_j(0, 0.5, 10.0));
    let s = rig.wait_status("the jog is blocked and latched", |s| s.collision_active);
    assert!(
        s.collision_pairs
            .iter()
            .any(|(a, b)| a == "shape:keepout" || b == "shape:keepout"),
        "the latched pairs must name the keep-out as a program shape: {:?}",
        s.collision_pairs
    );
    rig.drain_status();
    let s = rig.wait_status("the blocked jog comes to rest", |s| {
        s.speeds.iter().all(|v| v.abs() < 0.05)
    });
    assert!(
        s.angles[0] < mid_deg[0] - 5.0,
        "the jog drove to the keep-out it was stopped for: j0 = {} deg toward {}",
        s.angles[0],
        mid_deg[0]
    );

    // --- from inside the keep-out, an escaping jog is permitted.
    // Teleport into the box (a keep-out dropped over the arm) and jog
    // back out: refusing this would trap the arm, which is exactly what
    // the escape rule exists to prevent.
    enable_and_teleport(&rig, &mut c, mid_deg);
    rig.drain_status();
    c.send(&jog_j(0, -0.3, 5.0));
    rig.wait_status("the escaping jog moves the arm out", |s| {
        s.angles[0] < mid_deg[0] - 3.0
    });

    // --- from a shallow penetration, driving deeper is refused.
    // The TCP sits inside the box near its face; a slow jog toward the
    // centre goes DEEPER through the same colliding pair, and only the
    // min-distance half of the escape rule can see that.
    // Stop the still-running escape jog and let it decelerate to rest
    // BEFORE teleporting: its 5 s duration outlives the wait above, and
    // the release ramp of a live jog would drag the freshly teleported
    // pose back out of the box — the gate would then be right to ACCEPT
    // the inward jog.
    c.ok(&Command::Stop(Stop { clear_queue: false }));
    rig.drain_status();
    rig.wait_status("the stopped escape jog comes to rest", |s| {
        s.speeds.iter().all(|v| v.abs() < 0.05)
    });
    let shallow_deg = with_j0(mid_deg, -0.8 * (KEEPOUT_M / 2.0) * deg_per_m);
    enable_and_teleport(&rig, &mut c, shallow_deg);
    rig.drain_status();
    let s = rig.wait_status("the arm rests at the shallow spot", |s| {
        s.speeds.iter().all(|v| v.abs() < 0.05) && (s.angles[0] - shallow_deg[0]).abs() < 1.0
    });
    assert!(s.homed, "teleport must leave the arm referenced");
    // A speed whose lookahead lands AT the box centre. coal's penetration
    // depth for a mesh-vs-box pair is a local contact-patch estimate,
    // nearly flat in the true depth (measured on this rig: ~5 mm of
    // reported deepening across the full 40 mm face-to-centre travel),
    // so a gentle probe's deepening drowns in the gate's jitter
    // tolerance — the centre is where the measured drop is unambiguous.
    // The refusal still cannot be excused as "the far side is shallower
    // again": the centre is the depth extremum, not past it.
    let pct = (0.8 * (KEEPOUT_M / 2.0) / radius_m / (v0 * 0.15)).clamp(0.01, 1.0);
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

/// `[[installation_shapes]]` in the robot TOML is a real producer for the
/// installation layer (issue #19 gap 3): the configured keep-out arrives
/// in the ENFORCED collision world at boot (a planned move through it is
/// refused, not merely echoed), the SHAPES query reads it back on the
/// `installation` list, and neither `set_shapes` nor `reset_state` can
/// remove it. A malformed entry refuses BOOT with the shape named.
#[test]
fn installation_shapes_are_loaded_enforced_and_immutable_from_the_wire() {
    let mid_deg = with_j0(SWEEP_START_DEG, SWEEP_DEG / 2.0);
    let end_deg = with_j0(SWEEP_START_DEG, SWEEP_DEG);
    let mid_m = tcp_at_m(mid_deg);

    let config = test_config("install-shapes");
    std::fs::write(
        &config,
        format!(
            "{}\n[[installation_shapes]]\nname = \"cage\"\nkind = \"box\"\n\
             params = [{KEEPOUT_M}, {KEEPOUT_M}, {KEEPOUT_M}]\n\
             pose = [{}, {}, {}, 0.0, 0.0, 0.0]\n",
            std::fs::read_to_string(&config).expect("test config"),
            mid_m[0],
            mid_m[1],
            mid_m[2],
        ),
    )
    .expect("write config");
    let rig = Rig::boot_with(config, false);
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
            assert_eq!(installation.len(), 1, "{installation:?}");
            assert_eq!(installation[0].name, "cage");
            assert_eq!(installation[0].kind, "box");
            assert_eq!(installation[0].params, vec![KEEPOUT_M; 3]);
        }
        other => panic!("unexpected SHAPES result {other:?}"),
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

    // Nothing on the wire removes it: an empty set_shapes and a full
    // reset_state both leave the cage standing and enforced.
    c.ok(&set_shapes(Vec::new()));
    c.ok(&Command::ResetState);
    match c.query(&Command::Shapes) {
        QueryResult::Shapes { installation, .. } => {
            assert_eq!(installation.len(), 1, "{installation:?}");
            assert_eq!(installation[0].name, "cage");
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
    let rig = boot_tagged("tcpoffset", false);
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
    for k in [0, 1, 2, 4, 5, 6, 8, 9, 10] {
        assert!(
            (offset.pose[k] - flange.pose[k]).abs() < 1e-6,
            "the offset rotated the reported pose at element {k}"
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
    let rig = boot_tagged("enablement", false);
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
    let at_corner = speeds
        .iter()
        .filter(|(p, _)| distance(*p, corner) < radius_mm)
        .map(|(_, v)| *v)
        .fold(f64::INFINITY, f64::min);
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

    let rig = boot_tagged("offset-chain", false);
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

    let rig = boot_tagged("blend", false);
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
    let sharp = rig.collect_through(i2, Duration::from_secs_f64(2.0 * LEG_S + 2.0));
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
    let blended = rig.collect_through(i2, Duration::from_secs_f64(2.0 * LEG_S + 2.0));
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

    let rig = boot_tagged("jointblend", false);
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
    let slowest = mid
        .iter()
        .map(|s| s.speeds.iter().fold(0.0f64, |m, v| m.max(v.abs())))
        .fold(f64::INFINITY, f64::min);
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

/// A posture whose pose the seeded solve reaches only by turning: from
/// the park pose, DLS converges on a configuration with J4 and J6 each
/// a full revolution out (`+2π`) — the same arm and numbers the
/// soft-limit check rejects verbatim. Chosen near the park pose so the
/// executed move is short and the sim's tracking lag stays far inside
/// the landing tolerance.
const TURNED_POSTURE_RAD: [f64; NUM_JOINTS] = [
    0.585_609, -1.010_888, 3.205_22, -0.031_356, -0.093_302, 3.045_917,
];

/// J5 held past its SOFT window (1.9 rad) but inside its hard one: a
/// posture the arm can be teleported into and whose pose no turn of any
/// joint brings back in range.
const BEYOND_SOFT_J5_RAD: f64 = 1.9;

/// A converged IK solution is judged as a configuration, not as a turn
/// count.
///
/// Damped least squares integrates joint increments without bound, so a
/// solve routinely lands on `q + 2πk`: the same arm posture, carried by
/// a number the soft-limit check refuses verbatim — measured on this
/// rig as `move target for joint 3 (5.366112773926797 rad) is outside
/// soft limits [-2.6147335, 2.5547335]`, for a solution 0.917 rad
/// inside that window.
///
/// Both halves are the fix: the turned solution has to run and land on
/// the commanded pose, and a target that is out of range at every turn
/// count has to stay refused — wrapping is branch selection, never a
/// way past the limits.
#[test]
fn ik_solutions_are_wrapped_into_their_soft_window() {
    let rig = boot_tagged("ikwrap", false);
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

    // Commanded from the park pose, the seeded solve turns J1 and J4 a
    // revolution past their windows to reach it.
    enable_and_teleport(&rig, &mut c, park);
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

/// A position stream carries a target, not a rate, and the arm cannot
/// stop dead. Every datagram of a stepping stream is admissible on its
/// own target, so the arm builds speed toward the keep-out; when a target
/// finally lands inside it, the braking distance carries the TCP on. A
/// single held target never shows this — the limiter decelerates to stop
/// AT it — so the stream here advances the way a UI's does.
///
/// The bound is measured rather than named: a crawl at a fiftieth of the
/// rate, left to come to rest on the last target it was granted, stops
/// where the geometry says stop, since its own braking distance is
/// negligible. A stream driven at full rate must not end up closer than
/// the crawl did. A failure prints the timeline of both approaches beside
/// the daemon's own warnings.
#[test]
fn a_stepping_servo_stream_stops_no_closer_than_a_crawl_does() {
    let rig = boot_tagged("servogate", false);
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);

    let mid_deg = with_j0(SWEEP_START_DEG, SWEEP_DEG / 2.0);
    let mid_m = tcp_at_m(mid_deg);
    let radius_m = (mid_m[0].powi(2) + mid_m[1].powi(2)).sqrt();
    let deg_per_m = 1.0_f64.to_degrees() / radius_m;
    let keepout = keepout_at("keepout", [mid_m[0] * 1e3, mid_m[1] * 1e3, mid_m[2] * 1e3]);
    c.ok(&set_shapes(vec![keepout]));

    let start_deg = with_j0(mid_deg, -2.0 * KEEPOUT_M * deg_per_m);

    struct Scene {
        start_deg: [f64; NUM_JOINTS],
        mid_deg: [f64; NUM_JOINTS],
        mid_m: [f64; 3],
        deg_per_m: f64,
    }

    /// The closest the TCP came to the box centre \[m\], and the
    /// timeline of sends and frames that produced it.
    struct Approach {
        closest_m: f64,
        trace: Vec<String>,
    }

    /// Stream the target toward the box `step_mm` at a time until the
    /// gate latches, then let the arm come to rest on the last target it
    /// was granted.
    fn approach(
        rig: &Rig,
        c: &mut Client,
        scene: &Scene,
        step_mm: f64,
        speed: Option<f64>,
    ) -> Approach {
        let Scene {
            start_deg,
            mid_deg,
            mid_m,
            deg_per_m,
        } = *scene;
        enable_and_teleport(rig, c, start_deg);
        rig.drain_status();
        let step_deg = step_mm * 1e-3 * deg_per_m;
        let mut target = start_deg;
        let t0 = Instant::now();
        let deadline = t0 + BUDGET;
        let dist = |deg: [f64; NUM_JOINTS]| {
            let tcp = tcp_at_m(deg);
            ((tcp[0] - mid_m[0]).powi(2) + (tcp[1] - mid_m[1]).powi(2)).sqrt()
        };
        let mut closest = f64::INFINITY;
        let mut trace: Vec<String> = Vec::new();
        let mut last_logged = f64::NAN;
        let mut sample = |s: &Status, closest: &mut f64, trace: &mut Vec<String>| {
            let d = dist(s.angles);
            *closest = closest.min(d);
            // A resting arm repeats its frame; motion and verdicts are
            // what the timeline is for.
            if last_logged.is_nan() || (d - last_logged).abs() > 0.5e-3 || s.collision_active {
                last_logged = d;
                trace.push(format!(
                    "{:7.3}s frame {:6.1} mm mode={:?} collision_active={} j0={:+.4} rad/s",
                    t0.elapsed().as_secs_f64(),
                    d * 1e3,
                    s.mode,
                    s.collision_active,
                    s.speeds[0]
                ));
            }
        };
        let mut gated = false;
        while Instant::now() < deadline && !gated {
            target[0] = (target[0] + step_deg).min(mid_deg[0]);
            trace.push(format!(
                "{:7.3}s send  {:6.1} mm",
                t0.elapsed().as_secs_f64(),
                dist(target) * 1e3
            ));
            c.send(&Command::ServoJ(par6_proto::command::ServoJ {
                angles: target,
                speed,
                accel: None,
            }));
            let window = Instant::now() + Duration::from_millis(50);
            while Instant::now() < window {
                if let Some(s) = rig.recv_status() {
                    sample(&s, &mut closest, &mut trace);
                    if s.collision_active {
                        gated = true;
                        break;
                    }
                }
            }
        }
        assert!(
            gated,
            "a stream driven into a keep-out was never gated:\n{}",
            trace.join("\n")
        );
        // The arm is still on its way to the last target it was granted,
        // and a crawl moves below any speed threshold, so rest is the
        // angles holding still across consecutive frames.
        let mut last = [f64::NAN; NUM_JOINTS];
        let mut still = 0;
        while still < 10 {
            assert!(
                Instant::now() < deadline,
                "the arm never came to rest after the gate latched:\n{}",
                trace.join("\n")
            );
            let Some(s) = rig.recv_status() else {
                continue;
            };
            sample(&s, &mut closest, &mut trace);
            let moved = last[0].is_nan()
                || s.angles
                    .iter()
                    .zip(last.iter())
                    .any(|(a, b)| (a - b).abs() > 1e-3);
            last = s.angles;
            still = if moved { 0 } else { still + 1 };
        }
        Approach {
            closest_m: closest,
            trace,
        }
    }

    let scene = Scene {
        start_deg,
        mid_deg,
        mid_m,
        deg_per_m,
    };
    const CRAWL_STEP_MM: f64 = 1.0;
    const STREAM_STEP_MM: f64 = 5.0;
    let crawl = approach(&rig, &mut c, &scene, CRAWL_STEP_MM, Some(0.02));
    let streamed = approach(&rig, &mut c, &scene, STREAM_STEP_MM, None);
    println!(
        "closest approach: crawl {:.1} mm, streamed {:.1} mm",
        crawl.closest_m * 1e3,
        streamed.closest_m * 1e3
    );
    // A millimetre of slack for the sampling grid: STATUS is a snapshot
    // stream, so neither approach is observed continuously.
    assert!(
        streamed.closest_m > crawl.closest_m - 1e-3,
        "the streamed approach ran {:.1} mm past where a crawl stops \
         ({:.1} mm vs {:.1} mm from the box centre): the gate admitted \
         targets the arm could not stop short of\n--- crawl ---\n{}\n--- streamed ---\n{}",
        (crawl.closest_m - streamed.closest_m) * 1e3,
        streamed.closest_m * 1e3,
        crawl.closest_m * 1e3,
        crawl.trace.join("\n"),
        streamed.trace.join("\n")
    );

    rig.shutdown();
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
    let rig = boot_tagged("curved-arc", false);
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
    let rig = boot_tagged("curved-spline", false);
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
    let rig = boot_tagged("curved-process", false);
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

/// A world change during a move does not manufacture a failure: a
/// keep-out that stays clear of the remaining path leaves the move to
/// complete, with no verdict and no standing error.
///
/// The control for the mid-flight re-guard beside it: without this, a
/// runtime that failed every running move on ANY world change would
/// pass the keep-out-on-the-path test just the same.
#[test]
fn an_off_path_world_change_leaves_a_running_move_alone() {
    let rig = boot_tagged("collision-offpath", false);
    let mut c = Client::new(rig.addr());
    rig.wait_status("link_ok", |s| s.link_ok == 1);
    c.ok(&Command::Reset);
    let end_deg = with_j0(SWEEP_START_DEG, SWEEP_DEG);

    enable_and_teleport(&rig, &mut c, SWEEP_START_DEG);
    let i = c.ok_index(&move_j(8101, end_deg, SWEEP_S));
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
        "a world change clear of the path must not stop the move, got {detail:?}"
    );
    let s = settled_tcp(&rig, "the arm at rest at the end of the sweep");
    assert!(
        angles_close(&s.angles, &end_deg, 1.0),
        "the move must run to its target: {:?}",
        s.angles
    );
    assert!(
        !s.collision_active && s.collision_pairs.is_empty() && s.error.is_none(),
        "an off-path change must leave no verdict behind: active={} pairs={:?} error={:?}",
        s.collision_active,
        s.collision_pairs,
        s.error
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
    let rig = boot_tagged("collision-queued", false);
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
    let rig = boot_tagged("plane-responsive", false);
    let mut c = Client::new(rig.addr());
    let s = curve_start(&rig, &mut c);
    let start = tcp_mm(&s);

    // Twenty-four reversals: every corner is an IK solve and a blend,
    // and the retimer has to search hard across them.
    let waypoints: Vec<[f64; 6]> = (0..24)
        .map(|i| {
            let leg = f64::from(i);
            let x = start[0] + if i % 2 == 0 { 60.0 } else { 0.0 };
            wire_pose_at(&s.pose, [x, start[1], start[2] - leg * 4.0])
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

    // Watch for exactly as long as the command takes, so the window is
    // the planning window rather than a guess about it.
    let mut worst = Duration::ZERO;
    let mut ping_worst = Duration::ZERO;
    let mut last = Instant::now();
    let deadline = Instant::now() + BUDGET;
    while !c.peek_complete(i) {
        assert!(Instant::now() < deadline, "the command never completed");
        if rig.recv_status().is_some() {
            worst = worst.max(last.elapsed());
            last = Instant::now();
        }
        // Also the receive-side probe: each one reads replies, which is
        // what stashes the COMPLETE this loop is waiting for.
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
