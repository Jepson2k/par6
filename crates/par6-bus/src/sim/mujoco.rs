//! Contact-level arm plant (feature `sim-mujoco`): the whole scene —
//! arm, gripper jaws, floor and graspable objects — lives in one MuJoCo
//! model (`sim-assets/PAR6_MSG_scene.xml`) and every DOF is driven
//! through `qfrc_applied`:
//!
//! - arm joints get the motor torques from the driver current loops
//!   (config torque↔current factor), idle-brake damping and config-limit
//!   endstop spring-dampers — same drive model as the ABA plant, but
//!   integrated by MuJoCo with contacts;
//! - the jaw DOF runs a stiff PD servo. In firmware mode the plant
//!   rate-limits its own approach to the RAW cmd-61 target (mirroring the
//!   front end's byte kinematics) and reports physical obstructions from
//!   the tracking lag; [`super::SimBus`] feeds those back as the gripper
//!   front end's object positions, so contact grasps surface through the
//!   REAL cmd-60 detection bits. When no firmware command is active
//!   (motor mode, calibration, idle) the servo just follows the front
//!   end's reported jaw byte and detection is off.
//!
//! With this plant the `set_gripper_object_*` test hooks are owned by the
//! scene — the plant overwrites them every tick with what the physics
//! says is between the jaws.
//!
//! MuJoCo comes in through `mujoco-rs`, which owns the libmujoco download
//! and the struct layouts. Generalized state is read and written through
//! its `qpos`/`qvel`/`qfrc_applied` slice views — no per-tick state
//! copies through `mj_getState`/`mj_setState`, and no allocation once the
//! model is loaded.

use std::path::Path;
use std::sync::Mutex;

use mujoco_rs::prelude::{MjData, MjModel, MjtObj};

use super::driver::PlantCmd;
use super::plant::JointMap;

/// Endstop contact natural frequency \[rad/s\] (as the ABA plant).
const STOP_OMEGA: f64 = 50.0;
/// Endstop contact damping ratio.
const STOP_ZETA: f64 = 1.2;
/// Idle (watchdog fired / cmd 12) extra damping rate \[1/s\] — the
/// shorted-phase brake of an idled driver. Free-motion viscous/Coulomb
/// friction lives in the MJCF joint defaults, not here.
const IDLE_RATE: f64 = 40.0;

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
pub(crate) enum JawDrive {
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
    /// Arm joint count (scene joints `0..n` are the arm, `n`/`n+1` the
    /// jaws, any free objects after).
    n: usize,
    /// Model timestep \[s\] (one bus tick = `round(dt/ts)` mj_steps).
    ts: f64,
    /// Cached generalized state, refreshed after every substep.
    qpos: Vec<f64>,
    qvel: Vec<f64>,
    qfrc: Vec<f64>,
    /// Apparent joint inertias probed at boot \[kg·m²\].
    inertia: Vec<f64>,
    k_stop: Vec<f64>,
    c_stop: Vec<f64>,
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

impl MujocoPlant {
    /// Load the scene and place the arm at `q0` (config joint frame ==
    /// scene qpos, jaws at the front end's boot byte, everything else at
    /// the scene's default pose). Panics with a descriptive message on
    /// load/layout failure (a sim construction bug, not a runtime error).
    pub fn new(scene: &Path, maps: &[JointMap], q0: &[f64]) -> Self {
        let n = maps.len();
        // The XML loader keeps global "last XML" parser state and is not
        // thread-safe; stepping per-instance data is.
        static LOAD_LOCK: Mutex<()> = Mutex::new(());
        let model = {
            let _guard = LOAD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            MjModel::from_xml(scene)
                .unwrap_or_else(|e| panic!("sim-mujoco: cannot load {}: {e}", scene.display()))
        };
        let nq = model.ffi().nq as usize;
        let nv = model.ffi().nv as usize;
        let ts = model.ffi().opt.timestep;
        assert!(
            ts > 0.0 && ts < 0.1,
            "implausible model timestep {ts} — broken model?"
        );
        let joint_id = |name: &str| -> Option<usize> { model.name_to_id(MjtObj::mjOBJ_JOINT, name) };
        // The qpos/qvel address of scene joint i equals i only while every
        // joint up to it is a 1-DOF hinge/slide — the plant indexes state
        // by joint number, so the scene must keep arm + jaws first.
        assert!(
            joint_id("jaw1_JOINT") == Some(n) && joint_id("jaw2_JOINT") == Some(n + 1),
            "scene joint order must be {n} arm joints, then jaw1_JOINT/jaw2_JOINT, \
             then free objects (jaw ids: {:?}/{:?})",
            joint_id("jaw1_JOINT"),
            joint_id("jaw2_JOINT")
        );
        assert!(
            nq >= n + 2 && nv >= n + 2 && nq - (n + 2) == (nv - (n + 2)) / 6 * 7,
            "scene DOF layout unexpected: nq={nq} nv={nv} for {n} arm joints + 2 jaws \
             + free-object joints"
        );

        let mut data = MjData::new(Box::new(model));

        // Boot pose: scene defaults, arm at q0, jaws at the boot byte.
        let mut qpos = data.qpos().to_vec();
        qpos[..n].copy_from_slice(q0);
        qpos[n] = byte_to_m(JAW_INIT_BYTE);
        qpos[n + 1] = -byte_to_m(JAW_INIT_BYTE);

        // Apparent-inertia probe (endstop gains, idle damping): one-step
        // velocity response to a unit torque against the zero-torque
        // baseline, from the boot pose.
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
        let k_stop = inertia
            .iter()
            .map(|i| i * STOP_OMEGA * STOP_OMEGA)
            .collect();
        let c_stop = inertia
            .iter()
            .map(|i| 2.0 * STOP_ZETA * i * STOP_OMEGA)
            .collect();

        data.reset();
        data.qpos_mut().copy_from_slice(&qpos);
        Self {
            data,
            n,
            ts,
            qpos,
            qvel: vec![0.0; nv],
            qfrc: vec![0.0; nv],
            inertia,
            k_stop,
            c_stop,
            jaw_cmd_byte: JAW_INIT_BYTE,
            close_at: None,
            open_at: None,
        }
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

    /// Advance one bus tick: loop currents (minus injected loads) become
    /// joint torques + idle damping + config-limit endstop torques, the
    /// jaw servo tracks its drive, MuJoCo integrates `round(dt/ts)`
    /// steps with contacts, and the jaw obstruction state updates.
    pub fn step(
        &mut self,
        dt: f64,
        cmds: &[PlantCmd],
        loads_ma: &[f64],
        maps: &[JointMap],
        jaw: Option<JawDrive>,
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
        for _ in 0..substeps as u32 {
            for j in 0..self.n {
                let map = &maps[j];
                let q = self.qpos[j];
                let v = self.qvel[j];
                let mut t = (cmds[j].current_ma - loads_ma[j]) / map.factor_ma_per_nm;
                if cmds[j].idle {
                    t -= IDLE_RATE * self.inertia[j] * v;
                }
                if q > map.hard_hi_rad {
                    t += -self.k_stop[j] * (q - map.hard_hi_rad) - self.c_stop[j] * v;
                } else if q < map.hard_lo_rad {
                    t += -self.k_stop[j] * (q - map.hard_lo_rad) - self.c_stop[j] * v;
                }
                self.qfrc[j] = t;
            }
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
            let x_t = byte_to_m(self.jaw_cmd_byte);
            let f = (JAW_KP * (x_t - self.qpos[self.n]) + JAW_KD * (jaw_vt - self.qvel[self.n]))
                .clamp(-JAW_FMAX, JAW_FMAX);
            self.qfrc[self.n] = f;
            self.data.qfrc_applied_mut().copy_from_slice(&self.qfrc);
            self.data.step();
            self.qpos.copy_from_slice(self.data.qpos());
            self.qvel.copy_from_slice(self.data.qvel());
            // Driver-enforced velocity limit, converted to joint space.
            let mut clamped = false;
            for j in 0..self.n {
                let vlim = cmds[j].vel_limit_ticks_s.abs()
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
        match jaw {
            Some(JawDrive::Active { .. }) => {
                let measured = m_to_byte(self.qpos[self.n]);
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
