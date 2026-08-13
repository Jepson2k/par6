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
/// Joint viscous friction as a rate \[1/s\], scaled by apparent inertia:
/// the torque `−rate·I·v` cancels the inertia through ABA, so `rate` is
/// the same velocity-decay rate as the kinematic plant's `VISC` and must
/// EQUAL it (plant.rs derives 2.0 against the homing stall-current
/// threshold: J2 steady approach drag ≈ 56 mA vs the 175 mA
/// current-ratio limit; 8.0 would put it at ~225 mA and false-fire).
const VISC_RATE: f64 = 2.0;
/// Coulomb friction \[Nm\], smoothed near zero velocity.
const COULOMB_NM: f64 = 0.1;
/// Narrowest velocity scale of the Coulomb smoothing \[rad/s\]; joints
/// too light to integrate that slope explicitly get a wider one (see
/// [`DynamicsPlant::coulomb_eps`]).
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
    coulomb_eps: Vec<f64>,
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
        let mut inertia = vec![0.0; n];
        probe_inertia(&mut model, &q, &mut inertia);
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
            coulomb_eps: vec![COULOMB_EPS; n],
            inertia,
            k_stop,
            c_stop,
        }
    }

    /// Smoothed Coulomb friction enters the torque vector explicitly, so
    /// its near-zero slope `COULOMB_NM/eps` acts as a damper of rate
    /// `COULOMB_NM/(eps·inertia)`: the substep only integrates that
    /// stably while `rate·h ≤ 1`. The wrist joints are one to three
    /// orders of magnitude lighter than the shoulder, so at the shared
    /// `COULOMB_EPS` their friction oscillates instead of damping and a
    /// perfectly gravity-compensated wrist drifts degrees per second.
    /// Widening the smoothing band for the light joints keeps the
    /// friction MAGNITUDE (`COULOMB_NM`) and only softens the regularized
    /// `sign()` — the same per-joint inertia scaling the endstop gains
    /// already use.
    fn coulomb_eps(inertia: f64, h: f64) -> f64 {
        COULOMB_EPS.max(COULOMB_NM * h / inertia)
    }

    /// Place the arm at `q0` \[rad\] at rest (teleport): the model, its
    /// data and the contact gains stay live — only the state moves. The
    /// apparent-inertia probe is re-run at the new pose, so the endstop
    /// and friction scaling match the configuration the arm is now in.
    pub fn reseed(&mut self, q0: &[f64]) {
        self.q.copy_from_slice(q0);
        self.v.fill(0.0);
        self.a.fill(0.0);
        self.tau.fill(0.0);
        probe_inertia(&mut self.model, &self.q, &mut self.inertia);
        for j in 0..self.inertia.len() {
            self.k_stop[j] = self.inertia[j] * STOP_OMEGA * STOP_OMEGA;
            self.c_stop[j] = 2.0 * STOP_ZETA * self.inertia[j] * STOP_OMEGA;
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
        for (eps, inertia) in self.coulomb_eps.iter_mut().zip(&self.inertia) {
            *eps = Self::coulomb_eps(*inertia, h);
        }
        for _ in 0..SUBSTEPS {
            for j in 0..self.q.len() {
                let map = &maps[j];
                let drive_nm = (cmds[j].current_ma - loads_ma[j]) / map.factor_ma_per_nm;
                let mut t = drive_nm
                    - VISC_RATE * self.inertia[j] * self.v[j]
                    - COULOMB_NM * (self.v[j] / self.coulomb_eps[j]).tanh();
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

/// Apparent joint inertias at `q`: the unit-torque ABA response against
/// the zero-torque baseline. Panics on a non-positive result — that is a
/// broken model, not a runtime condition.
fn probe_inertia(model: &mut Model, q: &[f64], out: &mut [f64]) {
    let n = q.len();
    let zeros = vec![0.0; n];
    let mut tau = vec![0.0; n];
    let mut a0 = vec![0.0; n];
    let mut a1 = vec![0.0; n];
    model
        .aba_into(q, &zeros, &tau, &mut a0)
        .expect("ABA probe (zero torque)");
    for j in 0..n {
        tau[j] = 1.0;
        model
            .aba_into(q, &zeros, &tau, &mut a1)
            .expect("ABA probe (unit torque)");
        tau[j] = 0.0;
        let inv_m = a1[j] - a0[j];
        assert!(
            inv_m > 0.0,
            "ABA inertia probe returned non-positive apparent inertia for joint {j}"
        );
        out[j] = 1.0 / inv_m;
    }
}
