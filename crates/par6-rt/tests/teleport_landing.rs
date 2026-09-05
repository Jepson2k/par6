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

/// Poses spanning the joint windows: a golden kinematics case, the
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

/// Boot, enable, then teleport through `poses`; every tick after each
/// landing, plant truth, runtime `q` and the target agree.
fn land_at(gravity: Box<dyn GravityModel>, bundle: &ConfigBundle, poses: &[[f64; MAX_JOINTS]]) {
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
            core.tick(dt, false);
            let truth = core.bus_mut().true_joint_rad();
            let s = handles.snapshots.latest();
            for i in 0..MAX_JOINTS {
                assert!(
                    (truth[i] - q[i]).abs() < TOL_RAD,
                    "tick {k} after teleport to {deg:?}: joint {i} plant at {:+.4} rad, \
                     teleported to {:+.4}",
                    truth[i],
                    q[i]
                );
                assert!(
                    (s.q[i] - truth[i]).abs() < TOL_RAD,
                    "tick {k} after teleport to {deg:?}: joint {i} runtime reports {:+.4} rad, \
                     plant at {:+.4}",
                    s.q[i],
                    truth[i]
                );
            }
        }
    }
}

#[test]
fn a_teleport_lands_the_plant_on_the_reference_from_the_first_tick() {
    land_at(Box::new(ZeroGravity), &common::bundle(), &POSES_DEG);
}

/// With the gravity feedforward live — a constant model carrying the
/// golden pose's torques, since the phantom-pose kick, not the model, is
/// under test — the landing at that pose holds from the first tick.
#[test]
fn a_teleport_lands_under_gravity_comp() {
    land_at(
        Box::new(common::ConstGravity([
            0.0, -7.3006, 2.5588, 0.0390, 0.1111, 0.0035,
        ])),
        &common::bundle(),
        &POSES_DEG[..1],
    );
}

/// A tool two kilos heavier than stock puts 1.4 Nm on the wrist pitch —
/// past its gearbox's 0.5 Nm holding friction — so the joint is held by
/// the drivers alone. The tick after a teleport, before the runtime's
/// next frames arrive, the drivers must already hold the landed pose: a
/// re-seed that left them limp let the wrist back-drive a degree.
#[test]
fn a_teleport_under_a_load_past_the_holding_friction_is_held() {
    let mut bundle = common::bundle();
    let name = bundle.robot.robot.active_gripper.clone();
    let gripper = bundle
        .grippers
        .iter_mut()
        .find(|g| g.name == name)
        .expect("active gripper");
    gripper.kinematics.mass_kg = 2.37;
    land_at(
        Box::new(common::ConstGravity([
            0.0, -13.4916, 7.1872, -0.0608, 1.3843, 0.0246,
        ])),
        &bundle,
        &POSES_DEG[1..2],
    );
}
