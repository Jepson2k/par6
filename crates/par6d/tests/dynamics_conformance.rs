//! The plant's gravity and the controller's gravity are the same function.
//!
//! The simulated arm is a MuJoCo model built from the URDF; the gravity
//! feedforward the runtime applies is Pinocchio's `G(q)` over the same URDF
//! through `par6-kin`. Every compensated-hold claim in the suite rests on the
//! two agreeing, and three doc comments in `par6-bus` have asserted that they
//! do (`sim/mujoco.rs`, `sim/mod.rs`) while naming this file — which did not
//! exist, so nothing checked it.
//!
//! What that cost: `par6-rt`'s teleport-landing cases carried the controller's
//! torques as pinned constants. Nothing tied them to the plant, so flipping
//! `active_tool` — the documented way to record a tool change — left them
//! describing a tool the plant was no longer swinging, and the case failed
//! with a joint sagging rather than with anything naming the cause.
//!
//! Both models are exercised through their real entry points: the plant by
//! the same `gravity_at` the sim exposes, the controller by the same
//! `load_gravity_kin` the daemon builds its feedforward from.

use std::sync::mpsc;

use par6_bus::sim::scene::{Scene, Tool};
use par6_bus::sim::SimBus;
use par6_bus::RuntimeBus;
use par6_config::{ConfigBundle, ToolConfig};
use par6_kin::Kin;
use par6_rt::adapters::{MotionJog, MotionStream};
use par6_rt::hooks::ClampStream;
use par6_rt::{
    sample_ring, CompletionPolicy, NoFk, RtCore, RtHooks, SharedDigitalIo, SharedFlashMarker,
    SharedLineGpio, SpecSettle, ZeroGravity,
};

mod common;

/// Agreement tolerance \[Nm\]. The two solvers differ in floating-point
/// association and in how the tool's inertia is attached, not in physics, so
/// the residual is numerical. Loose enough not to chase the last bit, tight
/// enough that a wrong tool — the failure this exists for — is orders of
/// magnitude outside it.
const TOL_NM: f64 = 1e-3;

/// Poses spanning the joint windows, including the two the teleport-landing
/// cases use, so a disagreement shows up here rather than as a sagging joint
/// there.
const POSES_DEG: [[f64; 6]; 4] = [
    [-133.228, -8.746, 261.687, 61.133, -22.625, 119.764],
    [-115.0, -40.0, 200.0, 0.0, 60.0, 180.0],
    [0.0, -75.0, 305.0, 20.0, -30.0, 180.0],
    [0.0, -90.0, 180.0, 0.0, 0.0, 180.0],
];

fn bundle() -> ConfigBundle {
    ConfigBundle::load(&common::shipped_config()).expect("shipped config")
}

fn scene(bundle: &ConfigBundle) -> Scene {
    Scene {
        tool: bundle
            .active_tool()
            .and_then(|g| g.urdf_variant.as_deref())
            .and_then(Tool::from_urdf_variant)
            .unwrap_or(Tool::Flange),
        assets: common::assets_dir(),
    }
}

/// The controller's `G(q)`, built exactly as the daemon builds it: the
/// model for the fitted tool, then the identified correction on top.
fn controller_gravity(bundle: &ConfigBundle, tool: Option<&ToolConfig>) -> Kin {
    let mut kin = par6d::kin::load_gravity_kin(&common::assets_dir(), tool)
        .expect("controller gravity model");
    kin.set_gravity_correction(&bundle.robot.gravity_correction)
        .expect("the config's gravity correction");
    kin
}

/// The plant as the daemon boots it: the RT core on the daemon's bus
/// type decides which tool the simulator carries.
fn sim_core(bundle: &ConfigBundle) -> RtCore<RuntimeBus> {
    let robot = &bundle.robot;
    let (_tx, rx) = mpsc::channel();
    let (gpio, _line) = SharedLineGpio::new(true);
    let (marker, _flash) = SharedFlashMarker::new();
    let (io, _lines) = SharedDigitalIo::new(robot.io.inputs.len(), robot.io.outputs.len());
    let (_producer, consumer) = sample_ring(16);
    let hooks = RtHooks {
        gravity: Box::new(ZeroGravity),
        jog: Box::new(MotionJog::from_config(robot).expect("jog engine")),
        stream: Box::new(MotionStream::from_config(robot).expect("stream limiter")),
        stream_shaped: Box::new(ClampStream::new(robot)),
        settle: Box::new(SpecSettle::new(
            CompletionPolicy::Settled,
            robot.robot.tick_dt_s,
            robot.motion,
        )),
        estop: Box::new(gpio),
        io: Box::new(io),
        flash: Box::new(marker),
        commands: Box::new(rx),
        fk: Box::new(NoFk),
        samples: consumer,
    };
    let bus = RuntimeBus::from(SimBus::new(scene(bundle)));
    RtCore::new(bundle, bus, hooks).expect("sim core").0
}

fn compare(core: &mut RtCore<RuntimeBus>, kin: &mut Kin, label: &str) {
    let plant = core.bus_mut().sim_mut().expect("sim bus");
    for deg in &POSES_DEG {
        let q: [f64; 6] = std::array::from_fn(|i| deg[i].to_radians());
        let theirs = plant.gravity_at(&q).expect("plant gravity");
        let mut ours = [0.0; 6];
        kin.gravity(&q, &mut ours).expect("controller gravity");
        for j in 0..6 {
            let diff = (ours[j] - theirs[j]).abs();
            assert!(
                diff < TOL_NM,
                "{label}, pose {deg:?}: joint {j} — controller says {:+.4} Nm, plant applies \
                 {:+.4} Nm, apart by {diff:.4}",
                ours[j],
                theirs[j],
            );
        }
    }
}

/// The models track the fitted tool together — booted with it, driven or
/// not, and changed under a running core the way `select_tool` changes it
/// — or the feedforward describes a load the arm is not carrying.
#[test]
fn the_plant_and_the_controller_agree_about_gravity_through_tool_changes() {
    let bundle = bundle();
    let shipped = bundle
        .active_tool()
        .expect("the shipped config fits a tool");
    // An undriven tool whose mass is its config's, not its URDF's: the
    // scene borrows the bare flange's tree for it.
    let passive = bundle
        .tools
        .iter()
        .find(|g| g.name == "MHZ2-10D")
        .expect("the config declares the MHZ2-10D");
    let gripper_node = bundle.robot.bus.gripper_node;

    let mut core = sim_core(&bundle);
    compare(
        &mut core,
        &mut controller_gravity(&bundle, Some(shipped)),
        "shipped tool",
    );
    core.set_gripper_tool(Some(passive), gripper_node, 1);
    compare(
        &mut core,
        &mut controller_gravity(&bundle, Some(passive)),
        "after select_tool to the MHZ2-10D",
    );
    core.set_gripper_tool(Some(shipped), gripper_node, 1);
    compare(
        &mut core,
        &mut controller_gravity(&bundle, Some(shipped)),
        "after select_tool back to the shipped tool",
    );

    let mut undriven = bundle.clone();
    undriven.robot.robot.active_tool.clone_from(&passive.name);
    compare(
        &mut sim_core(&undriven),
        &mut controller_gravity(&undriven, undriven.active_tool()),
        "booted with the MHZ2-10D",
    );

    // A heavier declared tool: both models read the declared mass.
    let mut heavy = bundle.clone();
    heavy
        .tools
        .iter_mut()
        .find(|g| g.name == shipped.name)
        .expect("shipped tool")
        .kinematics
        .mass_kg = 2.0;
    compare(
        &mut sim_core(&heavy),
        &mut controller_gravity(&heavy, heavy.active_tool()),
        "a 2 kg tool",
    );
}
