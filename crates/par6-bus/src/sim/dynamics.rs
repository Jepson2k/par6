//! Torque-level arm plant (feature `sim-dynamics`): motor torques from
//! the driver current loops (× kt × gear through the config
//! torque↔current factor) + gravity + viscous/Coulomb friction + endstop
//! spring-damper torques → Pinocchio ABA forward dynamics →
//! semi-implicit Euler at a fixed substep of the bus tick.
//!
//! Endstop stiffness and damping are scaled per joint from an
//! apparent-inertia probe run once at construction (unit-torque ABA
//! differences), so every joint's contact behaves like the same
//! well-damped oscillator regardless of its inertia.

use std::path::Path;

use pinokin_sys::Model;

use super::driver::PlantCmd;
use super::plant::JointMap;

/// Physics substeps per bus tick (contact stability at 250 Hz).
const SUBSTEPS: u32 = 4;
/// Endstop contact natural frequency \[rad/s\].
const STOP_OMEGA: f64 = 50.0;
/// Endstop contact damping ratio.
const STOP_ZETA: f64 = 1.2;
/// Joint viscous friction as a rate \[1/s\], scaled by apparent inertia —
/// matches the kinematic plant's damping so the config gains behave the
/// same on both plants.
const VISC_RATE: f64 = 8.0;
/// Coulomb friction \[Nm\], smoothed near zero velocity.
const COULOMB_NM: f64 = 0.1;
/// Velocity scale of the Coulomb smoothing \[rad/s\].
const COULOMB_EPS: f64 = 0.01;
/// Idle (watchdog fired / cmd 12) extra damping rate \[1/s\] — the
/// shorted-phase brake of an idled driver.
const IDLE_RATE: f64 = 40.0;

pub(crate) struct DynamicsPlant {
    model: Model,
    /// Joint positions \[rad\] — ground truth.
    q: Vec<f64>,
    /// Joint velocities \[rad/s\].
    v: Vec<f64>,
    tau: Vec<f64>,
    a: Vec<f64>,
    inertia: Vec<f64>,
    k_stop: Vec<f64>,
    c_stop: Vec<f64>,
}

impl DynamicsPlant {
    /// Build the plant from the URDF; panics with a descriptive message on
    /// model/FFI failure (a sim construction bug, not a runtime error).
    pub fn new(urdf: &Path, maps: &[JointMap], q0: &[f64]) -> Self {
        let mut model = Model::from_urdf(urdf, None, None)
            .unwrap_or_else(|e| panic!("sim-dynamics: cannot load {}: {e}", urdf.display()));
        let n = maps.len();
        assert_eq!(
            model.nq(),
            n,
            "URDF joint count {} != configured joint count {n}",
            model.nq()
        );
        let q = q0.to_vec();
        let mut tau = vec![0.0; n];
        let mut a0 = vec![0.0; n];
        let mut a1 = vec![0.0; n];
        let zeros = vec![0.0; n];
        model
            .aba_into(&q, &zeros, &tau, &mut a0)
            .expect("ABA probe (zero torque)");
        let mut inertia = vec![0.0; n];
        for j in 0..n {
            tau[j] = 1.0;
            model
                .aba_into(&q, &zeros, &tau, &mut a1)
                .expect("ABA probe (unit torque)");
            tau[j] = 0.0;
            let inv_m = a1[j] - a0[j];
            assert!(
                inv_m > 0.0,
                "ABA inertia probe returned non-positive apparent inertia for joint {j}"
            );
            inertia[j] = 1.0 / inv_m;
        }
        let k_stop: Vec<f64> = inertia
            .iter()
            .map(|i| i * STOP_OMEGA * STOP_OMEGA)
            .collect();
        let c_stop: Vec<f64> = inertia
            .iter()
            .map(|i| 2.0 * STOP_ZETA * i * STOP_OMEGA)
            .collect();
        Self {
            model,
            q,
            v: vec![0.0; n],
            tau: vec![0.0; n],
            a: vec![0.0; n],
            inertia,
            k_stop,
            c_stop,
        }
    }

    /// Measured motor state of joint `j` (position ticks, speed ticks/s).
    pub fn motor_state(&self, j: usize, map: &JointMap) -> (f64, f64) {
        let pos = f64::from(map.conv.motor_ticks(self.q[j]));
        let vel = map.conv.motor_speed_ticks_s(self.v[j]);
        (pos, vel)
    }

    /// Advance one bus tick: convert loop currents (minus injected loads)
    /// to joint torques, add friction and endstop torques, integrate ABA
    /// accelerations over `SUBSTEPS` semi-implicit Euler substeps.
    pub fn step(&mut self, dt: f64, cmds: &[PlantCmd], loads_ma: &[f64], maps: &[JointMap]) {
        let h = dt / f64::from(SUBSTEPS);
        for _ in 0..SUBSTEPS {
            for j in 0..self.q.len() {
                let map = &maps[j];
                let drive_nm = (cmds[j].current_ma - loads_ma[j]) / map.factor_ma_per_nm;
                let mut t = drive_nm
                    - VISC_RATE * self.inertia[j] * self.v[j]
                    - COULOMB_NM * (self.v[j] / COULOMB_EPS).tanh();
                if cmds[j].idle {
                    t -= IDLE_RATE * self.inertia[j] * self.v[j];
                }
                if self.q[j] > map.hard_hi_rad {
                    t += -self.k_stop[j] * (self.q[j] - map.hard_hi_rad)
                        - self.c_stop[j] * self.v[j];
                } else if self.q[j] < map.hard_lo_rad {
                    t += -self.k_stop[j] * (self.q[j] - map.hard_lo_rad)
                        - self.c_stop[j] * self.v[j];
                }
                self.tau[j] = t;
            }
            self.model
                .aba_into(&self.q, &self.v, &self.tau, &mut self.a)
                .expect("ABA step");
            for j in 0..self.q.len() {
                // Driver-enforced velocity limit, converted to joint space.
                let vlim = cmds[j].vel_limit_ticks_s.abs()
                    * (std::f64::consts::TAU / f64::from(maps[j].encoder_max_counts))
                    / maps[j].gear_ratio;
                self.v[j] = (self.v[j] + self.a[j] * h).clamp(-vlim, vlim);
                self.q[j] += self.v[j] * h;
            }
        }
    }
}
