//! The drive current a planned move may spend.
//!
//! A joint's acceleration limit is a current limit seen through the inertia
//! the joint carries, and that inertia changes with the arm's reach: J1's
//! doubled with the arm out, where the 16 rad/s² it took at ready held its
//! drive at the current limit from the first ticks (2026-11-07). Each EXEC
//! move is held instead to `current_fraction` of every drive's limit
//! through the dynamics model, and the rest of the current is the loop's.

use par6_bus::spectral::torque_to_ma_factor;
use par6_config::{LimitMode, RobotConfig, ThermalConfig};
use par6_motion::MotionError;

use crate::kin::CartKin;
use par6_kin::NQ;

/// Poses a straight joint move's caps are checked at, ends included.
const LINE_SAMPLES: usize = 16;
/// Displacements below this count as a joint that does not move \[rad\].
const STILL_RAD: f64 = 1e-12;

/// Per joint, the current a move may command, and what the link model
/// leaves out: friction and the motor's own inertia.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CurrentBudget {
    /// The share of the current available a move may command; `None` on a
    /// joint that sets no budget.
    fraction: [Option<f64>; NQ],
    /// The drive's peak current \[mA\], and the motor's continuous rating
    /// it derates to as the motor heats (`ThermalConfig`).
    peak_ma: [f64; NQ],
    rated_ma: [Option<f64>; NQ],
    thermal: Option<ThermalConfig>,
    /// Each joint's drive, whose temperature the derating reads.
    node: [usize; NQ],
    /// The drive current per joint Nm the budget converts with \[mA/Nm\]:
    /// the RT's, which takes the drive's own torque constant at boot. J1's
    /// drive reports 0.2516 Nm/A against the config's 0.28, and its torque
    /// stopped at 2.84 Nm where the config's scale put 3.16 (2026-11-07).
    ma_per_nm: [f64; NQ],
    /// Coulomb friction \[Nm\], spent before anything accelerates.
    coulomb_nm: [f64; NQ],
    /// Viscous friction \[Nm·s/rad\]: J1 carries 0.29, a third of its
    /// budget at the speeds a short move reaches.
    viscous_nm_s: [f64; NQ],
    /// Rotor inertia reflected to each joint, `G²·J_m` \[kg·m²\]: the drive
    /// accelerates it, and the link model does not carry it.
    rotor_kg_m2: [f64; NQ],
    /// The joint torque a move may command \[Nm\], Coulomb friction already
    /// taken off; `f64::MAX` on a joint that sets no budget.
    torque_nm: [f64; NQ],
}

impl CurrentBudget {
    /// The budget `robot`'s EXEC limits set, in the config's current scale;
    /// `None` when no joint sets a `current_fraction`.
    pub(crate) fn from_config(robot: &RobotConfig) -> Result<Option<Self>, MotionError> {
        let per_joint = |values: &[f64], j: usize| values.get(j).copied().unwrap_or(0.0);
        let mut fraction = [None; NQ];
        let mut peak_ma = [0.0; NQ];
        let mut rated_ma = [None; NQ];
        let mut node = [0; NQ];
        let mut ma_per_nm = [0.0; NQ];
        for (j, joint) in robot.joints.iter().take(NQ).enumerate() {
            ma_per_nm[j] = torque_to_ma_factor(
                joint.gear_ratio,
                joint.gear_efficiency,
                joint.kt_nm_a,
                joint.dir,
            );
            fraction[j] = joint.limits.for_mode(LimitMode::Exec).current_fraction;
            peak_ma[j] = joint.ilim_ma;
            rated_ma[j] = joint.rated_current_ma;
            node[j] = usize::from(joint.node_id);
        }
        if fraction.iter().all(Option::is_none) {
            return Ok(None);
        }
        let budget = Self {
            fraction,
            peak_ma,
            rated_ma,
            thermal: robot.thermal,
            node,
            ma_per_nm: [0.0; NQ],
            coulomb_nm: std::array::from_fn(|j| per_joint(&robot.sim.coulomb_nm, j)),
            viscous_nm_s: std::array::from_fn(|j| per_joint(&robot.sim.viscous_nm_s, j)),
            rotor_kg_m2: std::array::from_fn(|j| {
                let gear = robot.joints.get(j).map_or(0.0, |c| c.gear_ratio);
                per_joint(&robot.sim.motor_jm_kg_m2, j) * gear * gear
            }),
            torque_nm: [f64::MAX; NQ],
        };
        budget
            .at_scale(&ma_per_nm, &[None; NQ])
            .map(Some)
            .map_err(|reason| MotionError::InvalidInput {
                what: "current_fraction",
                reason,
            })
    }

    /// Each joint's drive \[node index\], whose temperature `at_scale`
    /// reads.
    pub(crate) fn nodes(&self) -> &[usize; NQ] {
        &self.node
    }

    /// This budget converted with `ma_per_nm` \[mA/Nm, signed\], as the RT
    /// publishes it, at each joint's motor temperature \[C\] (`None`
    /// unread: the peak stands). An entry that is not a positive scale (a
    /// snapshot from before the RT's first tick) keeps the scale already in
    /// use.
    pub(crate) fn at_scale(
        &self,
        ma_per_nm: &[f64],
        temperature_c: &[Option<f64>; NQ],
    ) -> Result<Self, String> {
        let mut out = *self;
        for (j, temperature) in temperature_c.iter().enumerate() {
            let scale = ma_per_nm.get(j).map_or(0.0, |f| f.abs());
            if scale.is_finite() && scale > 0.0 {
                out.ma_per_nm[j] = scale;
            }
            let Some(fraction) = self.fraction[j] else {
                continue;
            };
            let available = match (self.rated_ma[j], self.thermal, *temperature) {
                (Some(rated), Some(thermal), Some(t)) => {
                    let peak = self.peak_ma[j];
                    peak - (peak - rated.min(peak)) * thermal.derating(t)
                }
                _ => self.peak_ma[j],
            };
            let current = fraction * available;
            let torque = current / out.ma_per_nm[j] - self.coulomb_nm[j];
            if !(torque > 0.0 && torque.is_finite()) {
                return Err(format!(
                    "J{}'s share of its current limit does not cover its friction",
                    j + 1
                ));
            }
            out.torque_nm[j] = torque;
        }
        Ok(out)
    }

    /// The torque each joint may command \[Nm\].
    pub(crate) fn torque_nm(&self) -> &[f64; NQ] {
        &self.torque_nm
    }

    /// The viscous friction torque at joint speeds `v` \[Nm\].
    pub(crate) fn viscous_nm(&self, v: &[f64; NQ]) -> [f64; NQ] {
        std::array::from_fn(|j| self.viscous_nm_s[j] * v[j])
    }

    /// The torques that hold `q` moving at `v` and accelerating at `a`,
    /// gravity and the rotors included, friction not: the time-scaling's
    /// torque rows are quadratic in path speed, and viscous friction is
    /// linear in it.
    pub(crate) fn inverse_dynamics(
        &self,
        kin: &mut CartKin,
        q: &[f64; NQ],
        v: &[f64; NQ],
        a: &[f64; NQ],
    ) -> Result<[f64; NQ], String> {
        let mut tau = kin.inverse_dynamics(q, v, a)?;
        for (t, (r, a)) in tau.iter_mut().zip(self.rotor_kg_m2.iter().zip(a)) {
            *t += r * a;
        }
        Ok(tau)
    }

    /// The highest per-joint accelerations, none above `ceiling`, that keep
    /// every joint's torque inside its budget anywhere on the straight
    /// joint-space line from `start` to `target` while the moving joints run
    /// at up to `velocity`.
    ///
    /// The moves this caps keep each joint between its ends but do not
    /// follow the line exactly, and every joint's torque is bounded by the
    /// sum of what each acceleration can draw through the inertia matrix,
    /// so the line's poses stand for the move's. A row over its budget
    /// scales down every acceleration it draws on, by its own share: each
    /// joint then takes the smallest share among the rows it loads, which
    /// brings every row inside in one pass.
    pub(crate) fn accel_caps(
        &self,
        kin: &mut CartKin,
        start: &[f64; NQ],
        target: &[f64; NQ],
        velocity: &[f64; NQ],
        ceiling: &[f64; NQ],
    ) -> Result<[f64; NQ], String> {
        let delta: [f64; NQ] = std::array::from_fn(|k| target[k] - start[k]);
        let moving: [bool; NQ] = std::array::from_fn(|k| delta[k].abs() > STILL_RAD);
        // A joint that starts and ends at rest under `ceiling` cannot pass
        // sqrt(ceiling·distance), however it is timed.
        let v_peak: [f64; NQ] = std::array::from_fn(|k| {
            if moving[k] {
                velocity[k]
                    .min((ceiling[k] * delta[k].abs()).sqrt())
                    .copysign(delta[k])
            } else {
                0.0
            }
        });
        let viscous = self.viscous_nm(&v_peak);
        let zero = [0.0; NQ];
        let mut caps = *ceiling;
        for i in 0..=LINE_SAMPLES {
            let f = i as f64 / LINE_SAMPLES as f64;
            let q: [f64; NQ] = std::array::from_fn(|k| start[k] + f * delta[k]);
            let g = self.inverse_dynamics(kin, &q, &zero, &zero)?;
            let held = self.inverse_dynamics(kin, &q, &v_peak, &zero)?;
            let mut room = [0.0; NQ];
            for j in 0..NQ {
                // The velocity terms only shrink below the peak speed.
                room[j] =
                    self.torque_nm[j] - g[j].abs() - (held[j] - g[j]).abs() - viscous[j].abs();
                if room[j] <= 0.0 {
                    return Err(format!(
                        "J{} has no current left to accelerate with at {:?}",
                        j + 1,
                        q.map(f64::to_degrees)
                    ));
                }
            }
            // |M_jk| for the moving columns.
            let mut inertia = [[0.0; NQ]; NQ];
            for k in (0..NQ).filter(|&k| moving[k]) {
                let mut unit = zero;
                unit[k] = 1.0;
                let column = self.inverse_dynamics(kin, &q, &zero, &unit)?;
                for j in 0..NQ {
                    inertia[j][k] = (column[j] - g[j]).abs();
                }
            }
            let share: [f64; NQ] = std::array::from_fn(|j| {
                let drawn: f64 = (0..NQ).map(|k| inertia[j][k] * caps[k]).sum();
                if drawn > room[j] {
                    room[j] / drawn
                } else {
                    1.0
                }
            });
            for k in (0..NQ).filter(|&k| moving[k]) {
                let worst = (0..NQ)
                    .filter(|&j| inertia[j][k] > 0.0)
                    .map(|j| share[j])
                    .fold(1.0, f64::min);
                caps[k] *= worst;
            }
        }
        Ok(caps)
    }
}
