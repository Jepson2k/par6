//! The arm plant: the whole scene — arm, gripper jaws, floor and graspable
//! objects — lives in one MuJoCo model (built by [`super::scene`] from the
//! vendor MJCF) and every DOF is driven through `qfrc_applied`:
//!
//! - arm joints get the motor torques from the driver current loops,
//!   closed at the physics substep rate on the substep's own measured
//!   state (the firmware's loops run at ~1 kHz; a current held over a
//!   whole bus tick spins a light wrist joint into a tick-rate limit
//!   cycle), through the config torque↔current factor, plus idle-brake
//!   damping. The config hard limits are MuJoCo joint limits, and the
//!   drivetrain friction is MuJoCo `frictionloss`, set every substep by
//!   the law below;
//! - the jaw DOF runs a stiff PD servo. In firmware mode the plant
//!   rate-limits its own approach to the RAW cmd-61 target (mirroring the
//!   front end's byte kinematics) and reports physical obstructions from
//!   the tracking lag; [`super::SimBus`] feeds those back as the gripper
//!   front end's object positions, so contact grasps surface through the
//!   REAL cmd-60 detection bits. When no firmware command is active
//!   (motor mode, calibration, idle) the servo just follows the front
//!   end's reported jaw byte and detection is off.
//!
//! # Drivetrain
//!
//! The gearboxes are self-locking. The load on a joint (gravity and the
//! velocity terms, MuJoCo's `qfrc_bias`) is absorbed by the gearbox up to
//! the config `holding_friction_nm`: an unpowered joint holds, lowering a
//! load costs the motor only its own reflected Coulomb loss `G · tc` (the
//! scene's compiled `frictionloss`), and a motor working against the load
//! feels exactly the part of it that its own torque has not matched — so
//! an under-torqued lift holds instead of sagging, and the joint moves
//! once the motor torque exceeds load plus loss. Beyond the holding
//! friction the load back-drives the joint. The law is a per-substep
//! `frictionloss` limit, so the dry friction stays on the solver side
//! and a held joint rests without chatter. This is what lets the homing
//! sequence idle the shoulder joints under gravity while the base homes,
//! and what keeps an IDLE arm on its pose instead of collapsing.
//!
//! The `set_gripper_object_*` test hooks are owned by the scene — the
//! plant overwrites them every tick with what the physics says is between
//! the jaws.
//!
//! MuJoCo comes in through `mujoco-rs`, which owns the libmujoco download
//! and the struct layouts. Generalized state is read and written through
//! its `qpos`/`qvel`/`qfrc_applied` slice views — no per-tick state
//! copies through `mj_getState`/`mj_setState`, and no allocation once the
//! model is loaded.

use std::collections::BTreeMap;
use std::ffi::CStr;
use std::ptr;

use mujoco_rs::mujoco_c::{mj_id2name, mj_recompile, mjs_getError};
use mujoco_rs::prelude::{MjData, MjModel, MjSpec, MjtJoint, MjtObj};

use super::driver::{PlantCmd, VirtualDriver, FW_LOOP_DT};
use super::map::JointMap;
use super::scene::{self, WORLD_PREFIX};

/// Idle (watchdog fired / cmd 12) extra damping rate \[1/s\] — the
/// shorted-phase brake of an idled driver. Free-motion viscous/Coulomb
/// friction lives in the MJCF joint defaults, not here.
const IDLE_RATE: f64 = 40.0;
/// Drivetrain friction limit \[N·m\] that clamps a joint outright: applied
/// to the arm between a teleport and the runtime's next command frames,
/// so the arm appears at rest, already held, instead of spending the gap
/// limp — a wrist loaded past its holding friction back-drove a degree
/// in that gap, and a cold position hold still sags a milliradian.
const LANDING_CLAMP_NM: f64 = 1.0e3;

/// Full jaw travel \[m\] — `jaw1_JOINT`'s slide range in the scene
/// (0 = fully open, −`JAW_TRAVEL_M` = fully closed).
const JAW_TRAVEL_M: f64 = 0.0525;
/// Jaw servo position gain \[N/m\].
const JAW_KP: f64 = 3000.0;
/// Jaw servo damping gain \[N·s/m\].
const JAW_KD: f64 = 30.0;
/// Jaw servo force clamp \[N\] (the physical drive's stall force; under
/// the vendor model's 50 N actuator forcerange).
const JAW_FMAX: f64 = 40.0;
/// Firmware-command tracking lag that reads as an obstruction \[position
/// bytes\]. Free-travel servo lag stays under ~3 bytes at full speed.
const JAW_LAG_BYTES: f64 = 8.0;
/// Boot jaw position byte (mirrors the gripper front end's boot state).
const JAW_INIT_BYTE: f64 = 127.5;

/// How the scene's jaw DOF is driven this tick.
#[derive(Debug, Clone, Copy)]
pub enum JawDrive {
    /// Follow the gripper front end's reported jaw byte (motor mode,
    /// calibration sweep, inactive/estopped/watchdogged command). No
    /// obstruction detection.
    Track { byte: f64 },
    /// An active firmware close/open command: the plant runs its own
    /// rate-limited approach to the raw command target and converts
    /// tracking lag into obstruction positions.
    Active { target_byte: f64, rate_bytes_s: f64 },
}

pub(crate) struct MujocoPlant {
    /// The scene and its state; the data owns the model.
    data: MjData<Box<MjModel>>,
    /// Arm joint count (scene joints `0..n` are the arm, then the jaw
    /// pair when the tool has one, then free world objects).
    n: usize,
    /// qpos index of `jaw1_JOINT` (`jaw2_JOINT` follows), `None` on a
    /// tool without jaws.
    jaw: Option<usize>,
    /// Model timestep \[s\] (one bus tick = `round(dt/ts)` mj_steps).
    ts: f64,
    /// Cached generalized state, refreshed after every substep.
    qpos: Vec<f64>,
    qvel: Vec<f64>,
    qfrc: Vec<f64>,
    /// Apparent joint inertias probed at boot \[kg·m²\] (idle damping).
    inertia: Vec<f64>,
    /// Reflected motor Coulomb loss per arm joint \[N·m\] — the scene's
    /// compiled `frictionloss`, the floor of the drivetrain friction.
    coulomb: Vec<f64>,
    /// Gearbox holding friction per arm joint \[N·m\].
    hold: Vec<f64>,
    /// This substep's drivetrain friction per arm joint \[N·m\].
    friction: Vec<f64>,
    /// This substep's driver loop outputs per arm joint.
    cmds: Vec<PlantCmd>,
    /// Last substep's bias torques (gravity + velocity terms), all DOFs.
    bias: Vec<f64>,
    /// Free world objects by shape name: `(joint id, qpos address, dof
    /// address)`, re-indexed whenever the model is compiled.
    objects: BTreeMap<String, (usize, usize, usize)>,
    /// The servo's commanded jaw byte (the front end's kinematic jaw).
    jaw_cmd_byte: f64,
    close_at: Option<u8>,
    open_at: Option<u8>,
}

// The model and data are uniquely owned and only touched through &mut self
// methods; the plant moves between threads with the bus that owns it.
unsafe impl Send for MujocoPlant {}

fn byte_to_m(byte: f64) -> f64 {
    -(byte / 255.0) * JAW_TRAVEL_M
}

fn m_to_byte(x: f64) -> f64 {
    (-x / JAW_TRAVEL_M * 255.0).clamp(0.0, 255.0)
}

/// The drivetrain's dry-friction limit for a motor torque `motor_nm`
/// against a load `load_nm` (the torque the rest of the world puts on the
/// joint): the Coulomb loss plus the load the gearbox absorbs — all of it
/// when the load pushes along the motor's drive or the motor is idle, the
/// unmatched remainder when the motor works against it — capped at the
/// holding friction.
fn drivetrain_friction(coulomb: f64, hold: f64, motor_nm: f64, load_nm: f64) -> f64 {
    let opposing = motor_nm != 0.0 && load_nm != 0.0 && (motor_nm > 0.0) != (load_nm > 0.0);
    let absorbed = if opposing {
        (load_nm.abs() - motor_nm.abs()).max(0.0)
    } else {
        load_nm.abs()
    };
    coulomb + absorbed.min(hold)
}

impl MujocoPlant {
    /// Take the compiled scene and place the arm at `q0` (config joint
    /// frame == scene qpos, jaws at the front end's boot byte, everything
    /// else at the scene's default pose), with `holding_nm` the gearbox
    /// holding friction per arm joint. Panics with a descriptive message
    /// on a layout the plant cannot drive (a sim construction bug, not a
    /// runtime error).
    pub fn new(model: MjModel, maps: &[JointMap], q0: &[f64], holding_nm: &[f64]) -> Self {
        let n = maps.len();
        assert_eq!(
            holding_nm.len(),
            n,
            "holding friction needs one entry per arm joint"
        );
        let nv = model.ffi().nv as usize;
        let ts = model.ffi().opt.timestep;
        assert!(
            ts > 0.0 && ts < 0.1,
            "implausible model timestep {ts} — broken model?"
        );
        let jaw = check_layout(&model, n);
        let coulomb = model.dof_frictionloss()[..n].to_vec();

        let mut data = MjData::new(Box::new(model));

        // Boot pose: scene defaults, arm at q0, jaws at the boot byte.
        let mut qpos = data.qpos().to_vec();
        qpos[..n].copy_from_slice(q0);
        if let Some(jaw) = jaw {
            qpos[jaw] = byte_to_m(JAW_INIT_BYTE);
            qpos[jaw + 1] = -byte_to_m(JAW_INIT_BYTE);
        }

        // Apparent-inertia probe (idle damping): one-step velocity
        // response to a unit torque against the zero-torque baseline,
        // from the boot pose.
        let mut probe = |tau: Option<usize>| -> Vec<f64> {
            data.reset();
            data.qpos_mut().copy_from_slice(&qpos);
            let qfrc = data.qfrc_applied_mut();
            qfrc.fill(0.0);
            if let Some(j) = tau {
                qfrc[j] = 1.0;
            }
            data.step();
            data.qvel().to_vec()
        };
        let v0 = probe(None);
        let mut inertia = vec![0.0; n];
        for (j, m_j) in inertia.iter_mut().enumerate() {
            let v1 = probe(Some(j));
            let inv_m = (v1[j] - v0[j]) / ts;
            assert!(
                inv_m > 0.0,
                "inertia probe returned non-positive apparent inertia for joint {j}"
            );
            *m_j = 1.0 / inv_m;
        }

        data.reset();
        data.qpos_mut().copy_from_slice(&qpos);
        let mut plant = Self {
            data,
            n,
            jaw,
            ts,
            qpos,
            qvel: vec![0.0; nv],
            qfrc: vec![0.0; nv],
            inertia,
            coulomb,
            hold: holding_nm.to_vec(),
            friction: vec![0.0; n],
            cmds: vec![
                PlantCmd {
                    current_ma: 0.0,
                    ff_ma: 0.0,
                    vel_limit_ticks_s: 0.0,
                    idle: true,
                };
                n
            ],
            bias: vec![0.0; nv],
            objects: BTreeMap::new(),
            jaw_cmd_byte: JAW_INIT_BYTE,
            close_at: None,
            open_at: None,
        };
        plant.index_objects();
        plant
    }

    /// Place the arm at `q0` \[rad\] at rest (teleport): only the arm's
    /// generalized state moves — the jaws, the graspable objects and
    /// every contact in the scene carry on, which reloading the scene
    /// would throw away. The boot inertia probe is kept (re-probing
    /// needs `mj_resetData`, which would reset exactly that state).
    pub fn reseed(&mut self, q0: &[f64]) {
        let n = self.n;
        self.qpos[..n].copy_from_slice(q0);
        self.qvel[..n].fill(0.0);
        self.qfrc[..n].fill(0.0);
        self.data.qpos_mut()[..n].copy_from_slice(q0);
        self.data.qvel_mut()[..n].fill(0.0);
    }

    /// Rebuild the model from `spec` in place (MuJoCo's `mj_recompile`),
    /// carrying the position and velocity of every joint that survives by
    /// name: the arm and jaws carry on, objects that stay keep their pose,
    /// new ones appear at their spawn pose. (MuJoCo preserves state only
    /// for the spec a model was compiled from; this spec is a fresh clone
    /// of the base, so the carry is done here.) The boot inertia probe is
    /// kept — an object changes no arm-joint inertia. Fails, model
    /// untouched, on a spec MuJoCo refuses.
    pub fn recompile(&mut self, spec: &mut MjSpec) -> Result<(), String> {
        let _guard = scene::load_lock();
        // The state that must survive, keyed by joint name: what MuJoCo
        // carries across a recompile is defined by the spec it compiled
        // from, and this spec is a fresh clone.
        let saved: Vec<(String, Vec<f64>, Vec<f64>)> = self
            .joints()
            .map(|(name, qadr, nq, dadr, nv)| {
                (
                    name,
                    self.qpos[qadr..qadr + nq].to_vec(),
                    self.qvel[dadr..dadr + nv].to_vec(),
                )
            })
            .collect();
        // SAFETY: the pointers come from the live wrappers this plant
        // owns; mj_recompile reallocates the model and data in place and
        // leaves both valid, or leaves them untouched on failure.
        let rc = unsafe {
            let m = self.data.model_mut().ffi_mut() as *mut _;
            let d = self.data.ffi_mut() as *mut _;
            mj_recompile(spec.ffi_mut() as *mut _, ptr::null(), m, d)
        };
        if rc != 0 {
            // SAFETY: mjs_getError returns a NUL-terminated string owned by
            // the spec, alive for the borrow.
            let msg = unsafe { CStr::from_ptr(mjs_getError(spec.ffi_mut() as *mut _)) }
                .to_string_lossy()
                .into_owned();
            return Err(format!("recompile failed: {msg}"));
        }
        let jaw = check_layout(self.data.model(), self.n);
        assert_eq!(
            jaw, self.jaw,
            "a world update cannot add or remove the jaws"
        );
        let nq = self.data.model().ffi().nq as usize;
        let nv = self.data.model().ffi().nv as usize;
        self.qpos.resize(nq, 0.0);
        self.qvel.resize(nv, 0.0);
        self.qfrc.resize(nv, 0.0);
        self.bias.resize(nv, 0.0);
        let joints: Vec<(String, usize, usize, usize, usize)> = self.joints().collect();
        for (name, qadr, nq, dadr, nv) in joints {
            if let Some((_, q, v)) = saved.iter().find(|(n, _, _)| *n == name) {
                if q.len() == nq && v.len() == nv {
                    self.data.qpos_mut()[qadr..qadr + nq].copy_from_slice(q);
                    self.data.qvel_mut()[dadr..dadr + nv].copy_from_slice(v);
                }
            }
        }
        self.qpos.copy_from_slice(self.data.qpos());
        self.qvel.copy_from_slice(self.data.qvel());
        self.qfrc.fill(0.0);
        self.bias.copy_from_slice(self.data.qfrc_bias());
        self.index_objects();
        Ok(())
    }

    /// Every joint of the current model: `(name, qpos address, qpos
    /// size, dof address, dof size)`.
    fn joints(&self) -> impl Iterator<Item = (String, usize, usize, usize, usize)> + '_ {
        let model = self.data.model();
        (0..model.ffi().njnt as usize).map(move |j| {
            // SAFETY: a valid model and an in-range joint id; the name is
            // a NUL-terminated string owned by the model.
            let name = unsafe {
                let p = mj_id2name(model.ffi(), MjtObj::mjOBJ_JOINT as i32, j as i32);
                if p.is_null() {
                    String::new()
                } else {
                    CStr::from_ptr(p).to_string_lossy().into_owned()
                }
            };
            let (nq, nv) = match model.jnt_type()[j] {
                MjtJoint::mjJNT_FREE => (7, 6),
                MjtJoint::mjJNT_BALL => (4, 3),
                _ => (1, 1),
            };
            (
                name,
                model.jnt_qposadr()[j] as usize,
                nq,
                model.jnt_dofadr()[j] as usize,
                nv,
            )
        })
    }

    /// The free joint of the world object `name` (its shape name):
    /// `(joint id, qpos address, dof address)`.
    ///
    /// Read from the map built when the model was compiled: a rollout asks
    /// per object per tick, and re-deriving it there would be a `format!`
    /// and a name-hash lookup a few hundred times a grasp for an answer
    /// that only changes when the model does.
    fn object_joint(&self, name: &str) -> Option<(usize, usize, usize)> {
        self.objects.get(name).copied()
    }

    /// Index the free world objects of the current model by shape name.
    fn index_objects(&mut self) {
        self.objects = self
            .joints()
            .enumerate()
            .filter(|(_, (name, _, nq, _, _))| *nq == 7 && name.starts_with(WORLD_PREFIX))
            .map(|(joint, (name, qadr, _, dadr, _))| {
                (name[WORLD_PREFIX.len()..].to_owned(), (joint, qadr, dadr))
            })
            .collect();
    }

    /// Pose `[x, y, z, qw, qx, qy, qz]` of the free world object `name`
    /// (its shape name), `None` when no such free body is in the scene.
    pub fn object_pose(&self, name: &str) -> Option<[f64; 7]> {
        let (_, adr, _) = self.object_joint(name)?;
        let mut pose = [0.0; 7];
        pose.copy_from_slice(&self.qpos[adr..adr + 7]);
        Some(pose)
    }

    /// Speed of the free world object `name`: the norm of its six
    /// generalized velocities \[m/s, rad/s\].
    pub fn object_speed(&self, name: &str) -> Option<f64> {
        let (_, _, dadr) = self.object_joint(name)?;
        Some(
            self.qvel[dadr..dadr + 6]
                .iter()
                .map(|v| v * v)
                .sum::<f64>()
                .sqrt(),
        )
    }

    /// Place the free world object `name` at rest at `pose` (teleport).
    pub fn place_object(&mut self, name: &str, pose: [f64; 7]) -> bool {
        let Some((_, adr, dadr)) = self.object_joint(name) else {
            return false;
        };
        self.qpos[adr..adr + 7].copy_from_slice(&pose);
        self.qvel[dadr..dadr + 6].fill(0.0);
        self.data.qpos_mut()[adr..adr + 7].copy_from_slice(&pose);
        self.data.qvel_mut()[dadr..dadr + 6].fill(0.0);
        true
    }

    /// Names of the free world objects in the scene.
    pub fn object_names(&self) -> Vec<String> {
        self.objects.keys().cloned().collect()
    }

    /// Every free object's pose into `out`, in the same order
    /// [`Self::object_names`] reports, returning how many were written
    /// (never more than `out.len()`). One walk of the index: a caller
    /// reading every object each tick pays no per-name lookup.
    pub fn object_poses_into(&self, out: &mut [[f64; 7]]) -> usize {
        let mut n = 0;
        for (_, &(_, qadr, _)) in self.objects.iter().take(out.len()) {
            out[n].copy_from_slice(&self.qpos[qadr..qadr + 7]);
            n += 1;
        }
        n
    }

    /// The scene's own gravity torque on the arm joints at `q` \[Nm\],
    /// at rest: `qfrc_bias` with every velocity zero, which leaves only
    /// gravity. The controller computes the same quantity from the URDF
    /// through Pinocchio, and the two models must agree — see par6d's
    /// `dynamics_conformance` suite. Leaves the plant at `q`, at rest.
    pub fn gravity_at(&mut self, q: &[f64]) -> Vec<f64> {
        self.reseed(q);
        self.data.qvel_mut().fill(0.0);
        self.data.forward();
        self.data.qfrc_bias()[..self.n].to_vec()
    }

    /// The jaws' measured position byte (0 = open, 255 = closed), `None`
    /// on a tool without jaws.
    pub fn jaw_byte(&self) -> Option<f64> {
        self.jaw.map(|jaw| m_to_byte(self.qpos[jaw]))
    }

    /// Place the jaws at rest at `byte` and aim the servo there (the tool
    /// half of a teleport).
    pub fn place_jaw(&mut self, byte: f64) {
        let Some(jaw) = self.jaw else {
            return;
        };
        let x = byte_to_m(byte.clamp(0.0, 255.0));
        self.jaw_cmd_byte = byte.clamp(0.0, 255.0);
        for (q, v) in [(jaw, x), (jaw + 1, -x)] {
            self.qpos[q] = v;
            self.qvel[q] = 0.0;
            self.data.qpos_mut()[q] = v;
            self.data.qvel_mut()[q] = 0.0;
        }
    }

    /// Measured motor state of arm joint `j` (position ticks, speed
    /// ticks/s).
    pub fn motor_state(&self, j: usize, map: &JointMap) -> (f64, f64) {
        let pos = f64::from(map.conv.motor_ticks(self.qpos[j]));
        let vel = map.conv.motor_speed_ticks_s(self.qvel[j]);
        (pos, vel)
    }

    /// Where the physics jammed the jaws this tick, as the front end's
    /// object position bytes (`None` = free travel / detection off).
    pub fn jaw_obstruction(&self) -> (Option<u8>, Option<u8>) {
        (self.close_at, self.open_at)
    }

    /// The contacts the solver actually resolved this step, appended to
    /// `pos` and `force` as world-frame triples.
    ///
    /// MuJoCo's contact list also holds near misses — pairs inside the
    /// inclusion margin that generate no constraint row and carry no
    /// force — so an unfiltered walk reports pushes where nothing is
    /// touching. `efc_address` is what tells the two apart.
    ///
    /// `mj_contactForce` answers in the contact frame, whose rows are the
    /// normal and the two tangents; rotating by its transpose puts the
    /// force in the world the caller draws in.
    pub fn contacts_into(&self, pos: &mut Vec<[f64; 3]>, force: &mut Vec<[f64; 3]>) {
        for (i, c) in self.data.contact().iter().enumerate() {
            if c.exclude != 0 || c.efc_address < 0 {
                continue;
            }
            let f = self.data.contact_force(i);
            let mut world = [0.0; 3];
            for (axis, w) in world.iter_mut().enumerate() {
                *w = f[0] * c.frame[axis] + f[1] * c.frame[3 + axis] + f[2] * c.frame[6 + axis];
            }
            pos.push(c.pos);
            force.push(world);
        }
    }

    /// The whole model's centre of mass \[m\], world frame. Body 0 is the
    /// world body, whose subtree is everything.
    pub fn center_of_mass(&self) -> [f64; 3] {
        self.data.subtree_com()[0]
    }

    /// Advance one bus tick: per substep the drivers close their loops on
    /// the measured state, loop currents (minus injected loads) become
    /// joint torques + idle damping, the drivetrain friction follows the
    /// motor torque, the jaw servo tracks its drive and MuJoCo integrates
    /// one step with contacts and limits; then the jaw obstruction state
    /// updates. Watchdog aging is per bus tick and stays with the caller.
    /// `clamp_arm` holds every arm joint outright (see
    /// [`LANDING_CLAMP_NM`]).
    // One index walks the drivers, their maps, the injected loads and four
    // state vectors in lockstep; zip chains would bury the torque law.
    #[allow(clippy::needless_range_loop)]
    pub fn step(
        &mut self,
        dt: f64,
        drivers: &mut [VirtualDriver],
        loads_ma: &[f64],
        maps: &[JointMap],
        jaw: Option<JawDrive>,
        clamp_arm: bool,
    ) {
        let substeps = (dt / self.ts).round();
        assert!(
            substeps >= 1.0 && (dt / self.ts - substeps).abs() < 1e-6,
            "scene timestep {} must divide the bus tick dt {dt}",
            self.ts
        );
        if let Some(JawDrive::Track { byte }) = jaw {
            self.jaw_cmd_byte = byte;
        }
        let h = self.ts;
        let fw_steps = (h / FW_LOOP_DT).round().max(1.0);
        for _ in 0..substeps as u32 {
            self.bias.copy_from_slice(self.data.qfrc_bias());
            for j in 0..self.n {
                let map = &maps[j];
                let (pos, vel) = self.motor_state(j, map);
                self.cmds[j] = drivers[j].loop_step(pos + map.report_offset, vel, fw_steps);
                let cmds = &self.cmds;
                let v = self.qvel[j];
                let motor = cmds[j].current_ma / map.factor_ma_per_nm;
                let external = -loads_ma[j] / map.factor_ma_per_nm;
                let mut t = motor + external;
                if cmds[j].idle {
                    t -= IDLE_RATE * self.inertia[j] * v;
                }
                self.qfrc[j] = t;
                // The load is what acts on the joint besides the motor:
                // MuJoCo's bias (its sign is the force that cancels it)
                // and the injected external load.
                let load = external - self.bias[j];
                let hold = if clamp_arm {
                    LANDING_CLAMP_NM
                } else {
                    self.hold[j]
                };
                self.friction[j] = drivetrain_friction(self.coulomb[j], hold, motor, load);
            }
            // SAFETY: only per-DOF friction values change; the model's
            // sizes and layout are untouched, so the data stays valid.
            unsafe { self.data.model_mut() }.dof_frictionloss_mut()[..self.n]
                .copy_from_slice(&self.friction);
            let mut jaw_vt = 0.0;
            if let Some(JawDrive::Active {
                target_byte,
                rate_bytes_s,
            }) = jaw
            {
                let step =
                    (target_byte - self.jaw_cmd_byte).clamp(-rate_bytes_s * h, rate_bytes_s * h);
                self.jaw_cmd_byte += step;
                // byte_to_m is linear through 0, so it maps deltas too.
                jaw_vt = byte_to_m(step) / h;
            }
            if let Some(jaw) = self.jaw {
                let x_t = byte_to_m(self.jaw_cmd_byte);
                let f = (JAW_KP * (x_t - self.qpos[jaw]) + JAW_KD * (jaw_vt - self.qvel[jaw]))
                    .clamp(-JAW_FMAX, JAW_FMAX);
                self.qfrc[jaw] = f;
            }
            self.data.qfrc_applied_mut().copy_from_slice(&self.qfrc);
            self.data.step();
            self.qpos.copy_from_slice(self.data.qpos());
            self.qvel.copy_from_slice(self.data.qvel());
            // Driver-enforced velocity limit, converted to joint space.
            let mut clamped = false;
            for j in 0..self.n {
                let vlim = self.cmds[j].vel_limit_ticks_s.abs()
                    * (std::f64::consts::TAU / f64::from(maps[j].encoder_max_counts))
                    / maps[j].gear_ratio;
                if self.qvel[j].abs() > vlim {
                    self.qvel[j] = self.qvel[j].clamp(-vlim, vlim);
                    clamped = true;
                }
            }
            if clamped {
                self.data.qvel_mut().copy_from_slice(&self.qvel);
            }
        }
        match (jaw, self.jaw) {
            (Some(JawDrive::Active { .. }), Some(jaw_q)) => {
                let measured = m_to_byte(self.qpos[jaw_q]);
                let lag = self.jaw_cmd_byte - measured;
                if lag > JAW_LAG_BYTES {
                    self.close_at = Some(measured.round() as u8);
                    self.open_at = None;
                } else if lag < -JAW_LAG_BYTES {
                    self.open_at = Some(measured.round() as u8);
                    self.close_at = None;
                } else {
                    self.close_at = None;
                    self.open_at = None;
                }
            }
            _ => {
                self.close_at = None;
                self.open_at = None;
            }
        }
    }
}

/// The generalized-coordinate layout the plant indexes by: `n` arm hinges
/// first (qpos address == joint number), then the jaw slide pair when the
/// tool has one, then only free joints. Returns the jaws' qpos index.
/// Panics with a descriptive message on anything else — a sim
/// construction bug, not a runtime error.
fn check_layout(model: &MjModel, n: usize) -> Option<usize> {
    let joint_id = |name: &str| -> Option<usize> { model.name_to_id(MjtObj::mjOBJ_JOINT, name) };
    let nq = model.ffi().nq as usize;
    let nv = model.ffi().nv as usize;
    let jaw = joint_id("jaw1_JOINT");
    if let Some(jaw) = jaw {
        assert!(
            jaw == n && joint_id("jaw2_JOINT") == Some(n + 1),
            "scene joint order must be {n} arm joints, then jaw1_JOINT/jaw2_JOINT, then \
             free objects (jaw ids: {jaw}/{:?})",
            joint_id("jaw2_JOINT")
        );
    }
    let base = n + if jaw.is_some() { 2 } else { 0 };
    assert!(
        nq >= base && nv >= base && nq - base == (nv - base) / 6 * 7,
        "scene DOF layout unexpected: nq={nq} nv={nv} for {n} arm joints, {} jaws and free \
         objects",
        if jaw.is_some() { 2 } else { 0 }
    );
    jaw
}
