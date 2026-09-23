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

use par6_bus::sim::scene::{Scene, Tool};
use par6_bus::sim::SimBus;
use par6_bus::DriverBus;
use par6_config::ConfigBundle;
use par6_kin::Kin;

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
/// model, then the identified correction on top.
fn controller_gravity(bundle: &ConfigBundle) -> Kin {
    let mut kin = par6d::kin::load_gravity_kin(&common::assets_dir(), bundle.active_tool())
        .expect("controller gravity model");
    kin.set_gravity_correction(&bundle.robot.gravity_correction)
        .expect("the config's gravity correction");
    kin
}

/// The plant's `G(q)`, through the bus the sim actually runs.
fn plant(bundle: &ConfigBundle) -> SimBus {
    let mut bus = SimBus::new(scene(bundle));
    bus.boot_configure(&bundle.robot, bundle.active_tool(), 1)
        .expect("sim boot");
    bus
}

fn compare(bundle: &ConfigBundle, label: &str) {
    let mut kin = controller_gravity(bundle);
    let mut bus = plant(bundle);
    let n = bundle.robot.joints.len();
    for deg in &POSES_DEG {
        let q: [f64; 6] = std::array::from_fn(|i| deg[i].to_radians());
        let theirs = bus.gravity_at(&q[..n]).expect("plant gravity");
        let mut ours = [0.0; 6];
        kin.gravity(&q, &mut ours).expect("controller gravity");
        for j in 0..n {
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

/// With the tool the shipped config selects.
#[test]
fn the_plant_and_the_controller_agree_about_gravity() {
    compare(&bundle(), "shipped tool");
}

/// And with a different one. This is the case that was never covered: the
/// models have to track `active_tool` together, or a tool change leaves the
/// feedforward describing a load the arm is not carrying.
#[test]
fn they_still_agree_after_the_active_tool_changes() {
    let mut bundle = bundle();
    let bare = bundle
        .tools
        .iter()
        .any(|g| g.name == "Flange")
        .then(|| "Flange".to_string())
        .expect("the config declares the bare Flange attachment");
    bundle.robot.robot.active_tool = bare;
    compare(&bundle, "bare flange");
}

/// And with a payload the model was not built with — the arm carrying
/// something heavier than its declared tool, which is what an identification
/// run exists to discover.
#[test]
fn they_still_agree_under_an_undeclared_payload() {
    let mut bundle = bundle();
    let name = bundle.robot.robot.active_tool.clone();
    let gripper = bundle
        .tools
        .iter_mut()
        .find(|g| g.name == name)
        .expect("active gripper");
    gripper.kinematics.mass_kg = 2.0;
    compare(&bundle, "2 kg tool");
}
