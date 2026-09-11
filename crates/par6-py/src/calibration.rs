//! Calibration uses the runtime's nominal gravity chain, not the visual URDF.
use crate::convert::joints;
use par6_config::{ConfigBundle, ModeLimits, RobotConfig};
use par6_kin::{Kin, NQ};
use pyo3::{exceptions::PyValueError, prelude::*};
use std::{path::Path, sync::Mutex};

#[pyclass(module = "par6._par6")]
pub struct GravityModel {
    kin: Mutex<Kin>,
}

#[pymethods]
impl GravityModel {
    #[new]
    fn new(py: Python<'_>, config: &str, assets: &str) -> PyResult<Self> {
        py.allow_threads(|| {
            let bundle = ConfigBundle::load(Path::new(config))
                .map_err(|e| PyValueError::new_err(e.to_string()))?;
            let kin = par6d::kin::load_gravity_kin(Path::new(assets), bundle.active_gripper())
                .map_err(PyValueError::new_err)?;
            Ok(Self {
                kin: Mutex::new(kin),
            })
        })
    }
    fn gravity(&self, q: Vec<f64>) -> PyResult<Vec<f64>> {
        let q = joints(&q, "q")?;
        let mut out = [0.0; NQ];
        self.kin
            .lock()
            .unwrap()
            .gravity(&q, &mut out)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(out.to_vec())
    }
    fn regressor(&self, q: Vec<f64>) -> PyResult<Vec<Vec<f64>>> {
        let q = joints(&q, "q")?;
        let mut kin = self.kin.lock().unwrap();
        let cols = kin.body_count() * 4;
        let mut out = vec![0.0; NQ * cols];
        kin.gravity_regressor(&q, &mut out)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(out.chunks(cols).map(|r| r.to_vec()).collect())
    }
    fn parameters(&self) -> PyResult<Vec<Vec<f64>>> {
        let kin = self.kin.lock().unwrap();
        (0..kin.body_count())
            .map(|i| {
                kin.body_inertial(i)
                    .map(|p| p.to_vec())
                    .map_err(|e| PyValueError::new_err(e.to_string()))
            })
            .collect()
    }
}

/// Produce a complete native-validated config; callers never edit motor calibration.
#[pyfunction]
#[pyo3(signature = (robot_toml, gravity=None, exec_limits=None, stream_limits=None, jog_limits=None, feedback_gains=None))]
pub fn calibration_config(
    robot_toml: &str,
    gravity: Option<Vec<f64>>,
    exec_limits: Option<Vec<[f64; 3]>>,
    stream_limits: Option<Vec<[f64; 3]>>,
    jog_limits: Option<Vec<[f64; 3]>>,
    feedback_gains: Option<Vec<[f64; 3]>>,
) -> PyResult<String> {
    let mut cfg =
        RobotConfig::from_toml_str(robot_toml).map_err(|e| PyValueError::new_err(e.to_string()))?;
    if let Some(delta) = gravity {
        cfg.gravity_correction = delta;
        // A fitted correction replaces the manual trim rather than stacking on it.
        cfg.gravity_scale = [1.0; NQ];
    }
    for (kind, rows) in [(0, exec_limits), (1, stream_limits), (2, jog_limits)] {
        if let Some(rows) = rows {
            if rows.len() != cfg.joints.len() {
                return Err(PyValueError::new_err("limits require one triple per joint"));
            }
            for (j, r) in cfg.joints.iter_mut().zip(rows) {
                let previous = match kind {
                    0 => j.limits.exec,
                    1 => j.limits.stream,
                    _ => j.limits.jog,
                };
                let limits = Some(ModeLimits {
                    velocity_rad_s: r[0],
                    acceleration_rad_s2: r[1],
                    jerk_rad_s3: Some(r[2]),
                    torque_rate_nm_s: previous.and_then(|p| p.torque_rate_nm_s),
                });
                match kind {
                    0 => j.limits.exec = limits,
                    1 => j.limits.stream = limits,
                    _ => j.limits.jog = limits,
                }
            }
        }
    }
    if let Some(rows) = feedback_gains {
        if rows.len() != cfg.joints.len() {
            return Err(PyValueError::new_err(
                "feedback requires one triple per joint",
            ));
        }
        for (joint, gains) in cfg.joints.iter_mut().zip(rows) {
            let original = [joint.gains.kpp, joint.gains.kpv, joint.gains.kiv];
            if gains
                .iter()
                .zip(original)
                .any(|(g, base)| !g.is_finite() || *g < 0.2 * base || *g > base)
            {
                return Err(PyValueError::new_err(
                    "feedback calibration only permits gains from 20% to 100% of baseline",
                ));
            }
            [joint.gains.kpp, joint.gains.kpv, joint.gains.kiv] = gains;
        }
    }
    cfg.validate()
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    // Preserve installation_shapes and other sections carried by ConfigBundle.
    let mut document: toml::Value =
        toml::from_str(robot_toml).map_err(|e| PyValueError::new_err(e.to_string()))?;
    let validated =
        toml::Value::try_from(&cfg).map_err(|e| PyValueError::new_err(e.to_string()))?;
    document
        .as_table_mut()
        .unwrap()
        .insert("gravity_scale".into(), validated["gravity_scale"].clone());
    if let Some(delta) = validated.get("gravity_correction") {
        document
            .as_table_mut()
            .unwrap()
            .insert("gravity_correction".into(), delta.clone());
    } else {
        document
            .as_table_mut()
            .unwrap()
            .remove("gravity_correction");
    }
    for (index, joint) in validated["joints"].as_array().unwrap().iter().enumerate() {
        document["joints"][index]["limits"] = joint["limits"].clone();
        document["joints"][index]["gains"] = joint["gains"].clone();
    }
    toml::to_string_pretty(&document).map_err(|e| PyValueError::new_err(e.to_string()))
}
