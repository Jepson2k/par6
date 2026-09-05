//! Physics for the offline preview: the scene stepped with the arm pinned
//! to a planned pose while the jaws and the free world objects move.
//!
//! The arm's trajectory stays the planner's — the preview's contract is
//! "planned by exactly the code that would drive the arm" — so the
//! rollout never integrates the arm: each tick it is placed where the
//! trajectory says and held there by the same landing clamp a teleport
//! uses, the drivers idle. What IS integrated is everything the planner
//! cannot know: the jaw servo closing on whatever is between the pads,
//! the object it jams on, and where a released object comes to rest. It is
//! the runtime plant ([`MujocoPlant`]) on the same scene, so a preview
//! grasp jams the same jaw model on the same contacts the simulator's
//! cmd-60 status path reports.

use par6_config::{GripperConfig, RobotConfig};
use par6_proto::Shape;

use super::driver::VirtualDriver;
use super::map::JointMap;
use super::mujoco::{JawDrive, MujocoPlant};
use super::scene::{self, Build, Scene, SceneError, ToolInertial, World};

/// Firmware speed byte the preview closes and opens the jaws at — the
/// tool actions' shipped default.
pub const JAW_SPEED_BYTE: f64 = 150.0;

/// A preview scene ready to step.
pub struct Rollout {
    plant: MujocoPlant,
    drivers: Vec<VirtualDriver>,
    maps: Vec<JointMap>,
    loads: Vec<f64>,
    dt: f64,
    base: scene::BaseSpec,
}

impl Rollout {
    /// Build the scene for `robot` with the active `gripper`'s tool, the
    /// config floor and `world` injected, the arm at `q0`.
    pub fn new(
        scene: &Scene,
        robot: &RobotConfig,
        gripper: Option<&GripperConfig>,
        world: &World,
        q0: &[f64],
    ) -> Result<Self, SceneError> {
        let dt = robot.robot.tick_dt_s;
        let maps: Vec<JointMap> = robot
            .joints
            .iter()
            .zip(q0)
            .map(|(j, q)| JointMap::from_config(j, *q))
            .collect();
        let tuning: Vec<scene::JointTuning> = maps
            .iter()
            .zip(&robot.sim.motor_jm_kg_m2)
            .map(|(map, jm)| scene::JointTuning::from_config(map, *jm, &robot.sim))
            .collect();
        let tool = gripper.map(|g| ToolInertial {
            d_m: g.kinematics.d_m,
            a_m: g.kinematics.a_m,
            alpha_rad: g.kinematics.alpha_rad,
            mass_kg: g.kinematics.mass_kg,
            com_m: g.kinematics.com_m,
            inertia_kg_m2: g.kinematics.inertia_kg_m2,
        });
        let build = Build {
            timestep: scene::timestep_for(dt),
            joints: &tuning,
            tool: tool.as_ref(),
        };
        let mut spec = scene.spec(&build)?;
        let base = scene::BaseSpec::new(&spec)?;
        scene::inject_world(&mut spec, world)?;
        let model = scene::compile(&mut spec).map_err(|detail| SceneError::Mjcf {
            path: scene.vendor_mjcf(),
            detail,
        })?;
        let plant = MujocoPlant::new(model, &maps, q0, &robot.sim.holding_friction_nm);
        // Never armed: idle drivers, so the only arm torque is the clamp's.
        let drivers = robot
            .joints
            .iter()
            .map(|j| {
                VirtualDriver::new(
                    dt,
                    j.node_id,
                    j.velocity_limit_ticks_s,
                    j.ilim_ma,
                    j.kt_nm_a,
                )
            })
            .collect();
        Ok(Self {
            plant,
            drivers,
            loads: vec![0.0; maps.len()],
            maps,
            dt,
            base,
        })
    }

    /// Rebuild around a changed world; the arm, the jaws and the objects
    /// that stay keep their state.
    pub fn set_world(&mut self, world: &World) -> Result<(), String> {
        let mut spec = self.base.clone_spec().map_err(|e| e.to_string())?;
        scene::inject_world(&mut spec, world).map_err(|e| e.to_string())?;
        self.plant.recompile(&mut spec)
    }

    /// The tick the scene steps by \[s\].
    pub fn dt(&self) -> f64 {
        self.dt
    }

    /// Pin the arm at `q` \[rad\], one entry per joint.
    pub fn place_arm(&mut self, q: &[f64]) {
        self.plant.reseed(q);
    }

    /// One tick: the jaws follow `jaw` (an active approach jams and reports
    /// through [`jaw_obstruction`](Self::jaw_obstruction)), the objects
    /// fall, slide and settle, the arm holds where it was placed.
    pub fn step(&mut self, jaw: Option<JawDrive>) {
        self.plant.step(
            self.dt,
            &mut self.drivers,
            &self.loads,
            &self.maps,
            jaw,
            true,
        );
    }

    /// An active jaw drive toward `byte` at the preview's jaw speed.
    pub fn jaw_drive(target_byte: f64) -> JawDrive {
        JawDrive::Active {
            target_byte: target_byte.clamp(0.0, 255.0),
            rate_bytes_s: JAW_SPEED_BYTE * super::gripper::BYTES_PER_S_PER_SPEED_UNIT,
        }
    }

    /// Where the physics jammed the jaws this tick, as position bytes
    /// `(closing at, opening at)`; `None` = free travel.
    pub fn jaw_obstruction(&self) -> (Option<u8>, Option<u8>) {
        self.plant.jaw_obstruction()
    }

    /// The jaws' measured position byte (0 = open, 255 = closed).
    pub fn jaw_byte(&self) -> Option<f64> {
        self.plant.jaw_byte()
    }

    /// Place the jaws at `byte` at rest.
    pub fn place_jaw(&mut self, byte: f64) {
        self.plant.place_jaw(byte);
    }

    /// Names of the free world objects.
    pub fn object_names(&self) -> Vec<String> {
        self.plant.object_names()
    }

    /// Pose `[x, y, z, qw, qx, qy, qz]` of the free object `name`.
    pub fn object_pose(&self, name: &str) -> Option<[f64; 7]> {
        self.plant.object_pose(name)
    }

    /// Speed of the free object `name` (norm of its six velocities).
    pub fn object_speed(&self, name: &str) -> Option<f64> {
        self.plant.object_speed(name)
    }

    /// Place the free object `name` at rest at `pose`.
    pub fn place_object(&mut self, name: &str, pose: [f64; 7]) -> bool {
        self.plant.place_object(name, pose)
    }

    /// The shapes' names that are free objects, in `world` order — what a
    /// caller needs before the scene exists.
    pub fn free_object_names(world: &World) -> Vec<String> {
        world
            .iter()
            .flat_map(|layer| layer.iter())
            .filter(|s| scene::is_free_body(s))
            .map(|s: &Shape| s.name.clone())
            .collect()
    }
}
