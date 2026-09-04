//! The runtime's configuration for the Python client: `par6_config`'s
//! own loader over the same TOML bundle `par6d` boots with, so every
//! limit, pose and name the Python surface exposes is the value the
//! runtime enforces — nothing re-parsed, nothing re-derived.

use std::path::PathBuf;

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};

use par6_config::{ConfigBundle, LimitMode, MotionConfig};
use par6_kin::GripperVariant;
use par6_motion::{MotionLimits, MIN_ACCEL_TIME_S};

/// The `[motion]` feel constants keyed by their config names.
pub fn motion_dict<'py>(py: Python<'py>, m: &MotionConfig) -> PyResult<Bound<'py, PyDict>> {
    let d = PyDict::new(py);
    d.set_item("jog_l_linear_max_m_s", m.jog_l_linear_max_m_s)?;
    d.set_item("jog_l_angular_max_rad_s", m.jog_l_angular_max_rad_s)?;
    d.set_item("cart_step_m", m.cart_step_m)?;
    d.set_item("cart_step_rad", m.cart_step_rad)?;
    d.set_item("path_step_m", m.path_step_m)?;
    d.set_item("joint_step_rad", m.joint_step_rad)?;
    d.set_item("move_l_max_joint_step_rad", m.move_l_max_joint_step_rad)?;
    d.set_item("dls_lambda", m.dls_lambda)?;
    d.set_item("settle_tolerance_rad", m.settle_tolerance_rad)?;
    d.set_item("settle_timeout_s", m.settle_timeout_s)?;
    Ok(d)
}

fn limit_mode(mode: &str) -> PyResult<LimitMode> {
    match mode.to_ascii_lowercase().as_str() {
        "exec" => Ok(LimitMode::Exec),
        "jog" => Ok(LimitMode::Jog),
        "stream" => Ok(LimitMode::Stream),
        other => Err(PyValueError::new_err(format!(
            "unknown limit mode {other:?} (exec, jog, stream)"
        ))),
    }
}

/// One loaded robot config bundle (robot TOML + its `grippers/*.toml`).
#[pyclass(module = "par6._par6")]
pub struct Config {
    bundle: ConfigBundle,
    path: PathBuf,
}

#[pymethods]
impl Config {
    /// Load `path`, or the runtime's own config search (`PAR6_CONFIG`,
    /// then the shipped config next to the binary) when `None`.
    #[new]
    #[pyo3(signature = (path=None))]
    fn new(path: Option<String>) -> PyResult<Self> {
        let path = match path {
            Some(p) => PathBuf::from(p),
            None => par6d::options::resolve_config_path(None).map_err(PyRuntimeError::new_err)?,
        };
        let bundle = ConfigBundle::load(&path)
            .map_err(|e| PyRuntimeError::new_err(format!("{}: {e}", path.display())))?;
        Ok(Self { bundle, path })
    }

    fn path(&self) -> String {
        self.path.display().to_string()
    }

    fn name(&self) -> String {
        self.bundle.robot.robot.name.clone()
    }

    fn joint_count(&self) -> usize {
        self.bundle.robot.joints.len()
    }

    fn joint_names(&self) -> Vec<String> {
        self.bundle
            .robot
            .joints
            .iter()
            .map(|j| j.name.clone())
            .collect()
    }

    fn tick_dt_s(&self) -> f64 {
        self.bundle.robot.robot.tick_dt_s
    }

    fn park_pose_rad(&self) -> Vec<f64> {
        self.bundle.robot.robot.park_pose_rad.clone()
    }

    /// The fitted gripper's name as the robot TOML spells it.
    fn active_gripper(&self) -> String {
        self.bundle.robot.robot.active_gripper.clone()
    }

    /// `(min, max)` software travel per joint \[rad\] — what motion may use.
    fn soft_limits_rad(&self) -> Vec<(f64, f64)> {
        self.bundle
            .robot
            .joints
            .iter()
            .map(|j| (j.limits.soft_min_rad, j.limits.soft_max_rad))
            .collect()
    }

    /// `(min, max)` hardware travel per joint \[rad\] — what `teleport`
    /// is refused outside of.
    fn hard_limits_rad(&self) -> Vec<(f64, f64)> {
        self.bundle
            .robot
            .joints
            .iter()
            .map(|j| (j.limits.hard_min_rad, j.limits.hard_max_rad))
            .collect()
    }

    /// The per-joint kinodynamic limits the runtime applies in `mode`
    /// (`exec`, `jog`, `stream`): `velocity`, `acceleration`, `jerk`,
    /// `soft_min`, `soft_max` — resolved by the same `par6_motion` rule
    /// the RT core consumes (a missing mode block falls back to the
    /// hardware ceiling).
    fn limits<'py>(&self, py: Python<'py>, mode: &str) -> PyResult<Bound<'py, PyDict>> {
        let mode = limit_mode(mode)?;
        let l = MotionLimits::from_config(&self.bundle.robot, mode)
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        let d = PyDict::new(py);
        d.set_item("velocity", l.velocity.to_vec())?;
        d.set_item("acceleration", l.acceleration.to_vec())?;
        d.set_item("jerk", l.jerk.to_vec())?;
        d.set_item("soft_min", l.soft_min.to_vec())?;
        d.set_item("soft_max", l.soft_max.to_vec())?;
        Ok(d)
    }

    /// The hardware ceiling per joint (`velocity`, `acceleration`, `jerk`)
    /// — what every mode's limits fall back to.
    fn hardware_limits<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let joints = &self.bundle.robot.joints;
        let d = PyDict::new(py);
        d.set_item(
            "velocity",
            joints
                .iter()
                .map(|j| j.limits.velocity_rad_s)
                .collect::<Vec<_>>(),
        )?;
        d.set_item(
            "acceleration",
            joints
                .iter()
                .map(|j| j.limits.acceleration_rad_s2)
                .collect::<Vec<_>>(),
        )?;
        d.set_item(
            "jerk",
            joints
                .iter()
                .map(|j| j.limits.jerk_rad_s3)
                .collect::<Vec<_>>(),
        )?;
        Ok(d)
    }

    /// The `[jog]` defaults, with the ramp time already floored by the
    /// runtime's `MIN_ACCEL_TIME_S` — the value the jog engine ramps with.
    fn jog_defaults<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let jog = &self.bundle.robot.jog;
        let d = PyDict::new(py);
        d.set_item("speed_pct", jog.speed_pct)?;
        d.set_item("accel_time_s", jog.accel_time_s.max(MIN_ACCEL_TIME_S))?;
        d.set_item("profile", format!("{:?}", jog.profile).to_ascii_uppercase())?;
        d.set_item("jerk_factor", jog.jerk_factor)?;
        Ok(d)
    }

    fn motion<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        motion_dict(py, &self.bundle.robot.motion)
    }

    /// `(inputs, outputs)` line names from `[io]`, in STATUS order.
    fn io_lines(&self) -> (Vec<String>, Vec<String>) {
        let io = &self.bundle.robot.io;
        (
            io.inputs.iter().map(|l| l.name.clone()).collect(),
            io.outputs.iter().map(|l| l.name.clone()).collect(),
        )
    }

    /// Where the configured homing sequence leaves the arm \[rad\].
    fn homing_ready_pose_rad(&self) -> PyResult<Vec<f64>> {
        self.bundle
            .robot
            .homing
            .ready_pose_rad(self.bundle.robot.joints.len())
            .map_err(PyRuntimeError::new_err)
    }

    /// Per-joint drive identity and tuning: `name`, `node_id`, `gains`
    /// (`kpp kpv kiv kpiq kiiq kp kd`), `ilim_ma`,
    /// `velocity_limit_ticks_s`, `voltage_limit_mv`, `gear_ratio`.
    fn joints<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyList>> {
        let out = PyList::empty(py);
        for j in &self.bundle.robot.joints {
            let d = PyDict::new(py);
            d.set_item("name", &j.name)?;
            d.set_item("node_id", j.node_id)?;
            let g = PyDict::new(py);
            g.set_item("kpp", j.gains.kpp)?;
            g.set_item("kpv", j.gains.kpv)?;
            g.set_item("kiv", j.gains.kiv)?;
            g.set_item("kpiq", j.gains.kpiq)?;
            g.set_item("kiiq", j.gains.kiiq)?;
            g.set_item("kp", j.gains.kp)?;
            g.set_item("kd", j.gains.kd)?;
            d.set_item("gains", g)?;
            d.set_item("ilim_ma", j.ilim_ma)?;
            d.set_item("velocity_limit_ticks_s", j.velocity_limit_ticks_s)?;
            d.set_item("voltage_limit_mv", j.voltage_limit_mv)?;
            d.set_item("gear_ratio", j.gear_ratio)?;
            out.append(d)?;
        }
        Ok(out)
    }

    /// Every gripper TOML in the bundle: `name`, `key` (the upper-cased
    /// wire spelling), `urdf_variant`, `kinematics` (the vendor DH row and
    /// tool mass: `d_m`, `a_m`, `alpha_rad`, `mass_kg`) and `driver`
    /// (`None` for a passive tool, else `driver_type`, `stroke_mm`,
    /// `ilim_ma`).
    fn grippers<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyList>> {
        let out = PyList::empty(py);
        for g in &self.bundle.grippers {
            let d = PyDict::new(py);
            d.set_item("name", &g.name)?;
            d.set_item("key", g.name.trim().to_ascii_uppercase())?;
            d.set_item("urdf_variant", g.urdf_variant.as_deref())?;
            let k = PyDict::new(py);
            k.set_item("d_m", g.kinematics.d_m)?;
            k.set_item("a_m", g.kinematics.a_m)?;
            k.set_item("alpha_rad", g.kinematics.alpha_rad)?;
            k.set_item("mass_kg", g.kinematics.mass_kg)?;
            d.set_item("kinematics", k)?;
            match &g.driver {
                Some(drv) => {
                    let dd = PyDict::new(py);
                    dd.set_item("driver_type", format!("{:?}", drv.driver_type))?;
                    dd.set_item("stroke_mm", drv.stroke_mm)?;
                    dd.set_item("ilim_ma", drv.ilim_ma)?;
                    d.set_item("driver", dd)?;
                }
                None => d.set_item("driver", py.None())?,
            }
            out.append(d)?;
        }
        Ok(out)
    }

    /// The URDF variant the runtime models `gripper_name` with — the
    /// gripper TOML's `urdf_variant` when it names one, else the vendor
    /// prefix rule: `key`, `urdf_relpath` / `srdf_relpath` (under the
    /// assets tree) and `tcp_frame` (the frame FK/IK resolve at).
    fn variant<'py>(&self, py: Python<'py>, gripper_name: &str) -> PyResult<Bound<'py, PyDict>> {
        let urdf_variant = self
            .bundle
            .grippers
            .iter()
            .find(|g| g.name.eq_ignore_ascii_case(gripper_name.trim()))
            .and_then(|g| g.urdf_variant.as_deref());
        let v = GripperVariant::resolve(&gripper_name.trim().to_ascii_uppercase(), urdf_variant);
        let d = PyDict::new(py);
        d.set_item("key", format!("{v:?}").to_ascii_lowercase())?;
        d.set_item("urdf_relpath", v.urdf_relpath())?;
        d.set_item("srdf_relpath", v.srdf_relpath())?;
        d.set_item("tcp_frame", v.tcp_frame())?;
        Ok(d)
    }

    /// The robot TOML's `[[installation_shapes]]` as wire shape dicts.
    fn installation_shapes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyList>> {
        let out = PyList::empty(py);
        for s in &self.bundle.installation_shapes {
            let d = PyDict::new(py);
            d.set_item("kind", &s.kind)?;
            d.set_item("params", s.params.clone())?;
            d.set_item("pose", s.pose.to_vec())?;
            d.set_item("collision", s.collision)?;
            d.set_item("margin", s.margin)?;
            d.set_item("name", &s.name)?;
            out.append(d)?;
        }
        Ok(out)
    }
}
