//! A teleport re-seeds the plant and re-bases the runtime's reference in
//! one step, so the plant's ground truth, the runtime's published `q` and
//! the requested pose must be the same angles from the very first tick
//! after — for every joint, from any pose, including ones whose wrapped
//! encoder reading sits far from the boot calibration.
//!
//! The first tick is the one that used to lie: the replies the sim had
//! queued before the re-seed were drained under the new reference and
//! read as a pose half a radian off, and the gravity feedforward computed
//! for that phantom pose kicked the wrist degrees off the landing.

mod common;

use std::sync::atomic::AtomicBool;
use std::sync::{mpsc, Arc};

use par6_bus::sim::SimBus;
use par6_config::ConfigBundle;
use par6_rt::hooks::{ClampStream, RampJog};
use par6_rt::{
    sample_ring, CompletionPolicy, GravityModel, Mode, NoFk, RtCommand, RtCore, RtHandles, RtHooks,
    SharedDigitalIo, SharedFlashMarker, SharedLineGpio, SpecSettle, ZeroGravity, MAX_JOINTS,
};

/// Landing tolerance \[rad\]: a few encoder ticks on the finest joint.
const TOL_RAD: f64 = 1e-3;

/// Landing tolerance under the heavier attached-tool load.
///
/// The joint is held by the position loop rather than by the drivetrain,
/// and a loop with finite stiffness holds a load at a small steady
/// offset — about a tenth of a degree on the wrist. What this still
/// catches is a landing that did not take: that is degrees out, not
/// millirad.
const TOL_LOADED_RAD: f64 = 2.5e-3;

/// Poses spanning the joint windows: a known kinematics case, the
/// cartesian test start and a near-vertical hold.
const POSES_DEG: [[f64; MAX_JOINTS]; 3] = [
    [-133.228, -8.746, 261.687, 61.133, -22.625, 119.764],
    [-115.0, -40.0, 200.0, 0.0, 60.0, 180.0],
    [0.0, -75.0, 305.0, 20.0, -30.0, 180.0],
];

fn boot_core(
    gravity: Box<dyn GravityModel>,
    bundle: &ConfigBundle,
) -> (
    RtCore<SimBus>,
    RtHandles,
    mpsc::Sender<RtCommand>,
    Arc<AtomicBool>,
) {
    let robot = &bundle.robot;
    let dt = robot.robot.tick_dt_s;
    let (tx, rx) = mpsc::channel();
    let (gpio, line) = SharedLineGpio::new(true);
    let (marker, _flash) = SharedFlashMarker::new();
    let (io, _io_lines) = SharedDigitalIo::new(robot.io.inputs.len(), robot.io.outputs.len());
    let (_producer, consumer) = sample_ring(64);
    let hooks = RtHooks {
        gravity,
        jog: Box::new(RampJog::new(robot)),
        stream: Box::new(ClampStream::new(robot)),
        settle: Box::new(SpecSettle::new(CompletionPolicy::Settled, dt, robot.motion)),
        estop: Box::new(gpio),
        io: Box::new(io),
        flash: Box::new(marker),
        commands: Box::new(rx),
        fk: Box::new(NoFk),
        samples: consumer,
    };
    let bus = SimBus::new(common::scene(bundle));
    let (core, handles) = RtCore::new(bundle, bus, hooks).expect("sim core");
    (core, handles, tx, line)
}

/// The daemon's teleport, step for step (`RtBridge::teleport`).
fn teleport(core: &mut RtCore<SimBus>, bundle: &ConfigBundle, q: &[f64; MAX_JOINTS]) {
    core.bus_mut()
        .teleport_joint_rad(&q[..bundle.robot.joints.len()])
        .expect("sim re-seed");
    core.adopt_landed_pose(&bundle.robot, q);
}

/// Boot, enable, then teleport through `poses`. The reference must follow
/// plant truth on every tick; compensated holds must stay on the target.
fn land_at(
    gravity: Box<dyn GravityModel>,
    bundle: &ConfigBundle,
    poses: &[[f64; MAX_JOINTS]],
    tol: f64,
    compensated: bool,
) {
    // These torque oracles describe the simulated load itself; applying a
    // hardware calibration trim would deliberately overcompensate that load.
    let mut nominal = bundle.clone();
    nominal.robot.gravity_scale = [1.0; MAX_JOINTS];
    let bundle = &nominal;
    let (mut core, mut handles, tx, _line) = boot_core(gravity, bundle);
    let dt = core.tick_dt_s();
    for _ in 0..10 {
        core.tick(dt, false);
    }
    assert_eq!(handles.snapshots.latest().mode, Mode::Idle, "boot settles");
    tx.send(RtCommand::Enable).unwrap();
    core.tick(dt, false);
    for deg in poses {
        let q: [f64; MAX_JOINTS] = std::array::from_fn(|i| deg[i].to_radians());
        teleport(&mut core, bundle, &q);
        for k in 0..250 {
            // Replies consumed this tick were queued before begin_tick advances
            // the plant. Compare the encoder with that sampling instant.
            let sampled_truth = core.bus_mut().true_joint_rad();
            core.tick(dt, false);
            let truth = core.bus_mut().true_joint_rad();
            let s = handles.snapshots.latest();
            for i in 0..MAX_JOINTS {
                // Without gravity feedforward the finite-stiffness drive may
                // deflect under load after landing. Reference accuracy must
                // still follow the moving plant on every subsequent tick.
                if k == 0 || compensated {
                    assert!(
                        (truth[i] - q[i]).abs() < tol,
                        "tick {k} after teleport to {deg:?}: joint {i} plant at {:+.4} rad, \
                         teleported to {:+.4}",
                        truth[i],
                        q[i]
                    );
                }
                assert!(
                    (s.q[i] - sampled_truth[i]).abs() < tol,
                    "tick {k} after teleport to {deg:?}: joint {i} runtime reports {:+.4} rad, \
                     plant at {:+.4}",
                    s.q[i],
                    sampled_truth[i]
                );
            }
        }
    }
}

#[test]
fn a_teleport_lands_the_plant_on_the_reference_from_the_first_tick() {
    land_at(
        Box::new(ZeroGravity),
        &common::bundle(),
        &POSES_DEG,
        TOL_RAD,
        false,
    );
}

/// With the gravity feedforward live — the plant's own torques at the landing
/// pose, since the phantom-pose kick, not the model, is under test — the
/// landing at that pose holds from the first tick.
#[test]
fn a_teleport_lands_under_gravity_comp() {
    let bundle = common::bundle();
    let q: [f64; MAX_JOINTS] = std::array::from_fn(|i| POSES_DEG[0][i].to_radians());
    let tau = common::plant_gravity(&bundle, &q);
    land_at(
        Box::new(common::ConstGravity(tau)),
        &bundle,
        &POSES_DEG[..1],
        TOL_RAD,
        true,
    );
}

/// A two-kilo tool puts well over a newton-metre on the wrist pitch — past
/// the 0.5 Nm the drivetrain holds by itself, but under what J5 can make at
/// its current limit. So the joint needs the drivers, and can still obey
/// them. The tick after a teleport, before the runtime's next frames arrive,
/// the drivers must already hold the landed pose: a re-seed that left them
/// limp let the wrist back-drive a degree.
///
/// Ask for more than J5 can produce and it back-drives no matter how right
/// the re-seed is, which tests nothing — so the load is asserted to stay
/// under the joint's own ceiling rather than assumed to.
#[test]
fn a_teleport_under_a_load_past_the_holding_friction_is_held() {
    let mut bundle = common::bundle();
    let name = bundle.robot.robot.active_tool.clone();
    let gripper = bundle
        .tools
        .iter_mut()
        .find(|g| g.name == name)
        .expect("active gripper");
    gripper.kinematics.mass_kg = 2.0;
    let q: [f64; MAX_JOINTS] = std::array::from_fn(|i| POSES_DEG[1][i].to_radians());
    let tau = common::plant_gravity(&bundle, &q);
    // The premise of the case, checked rather than asserted in prose: the
    // wrist is loaded past what the drivetrain holds unpowered, and not past
    // what its own current limit can produce.
    let wrist = tau[4].abs();
    let held_unpowered = bundle.robot.sim.holding_friction_nm[4];
    let ceiling = {
        let c = &bundle.robot.joints[4];
        c.kt_nm_a * (c.ilim_ma / 1000.0) * c.gear_ratio * c.gear_efficiency
    };
    assert!(
        wrist > held_unpowered,
        "J5 carries {wrist:.3} Nm, inside the {held_unpowered:.3} Nm the drivetrain \
         holds unpowered: the case would pass with the drivers limp"
    );
    assert!(
        wrist < ceiling,
        "J5 carries {wrist:.3} Nm, past the {ceiling:.3} Nm it can make: it back-drives \
         however right the re-seed is"
    );
    land_at(
        Box::new(common::ConstGravity(tau)),
        &bundle,
        &POSES_DEG[1..2],
        TOL_LOADED_RAD,
        true,
    );
}
