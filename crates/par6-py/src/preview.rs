//! The offline dry-run binding over `par6d::preview` — the daemon's own
//! planner, server rules and streaming integrator behind a virtual arm.
//! The Python shim builds command dicts; everything that decides what
//! the arm would do happens in the engine.

use std::sync::Mutex;

use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};

use par6_proto::NUM_JOINTS;
use par6d::matrix_to_xyzrpy;
use par6d::preview::{Preview as EnginePreview, PreviewResult};

use crate::config::motion_dict;
use crate::convert::{command_from_py, layer_of, robot_err, shape_from_py, wire_error_tuple};

/// Sample indices that keep a trajectory under `max_points` with both
/// endpoints retained.
fn sample_indices(len: usize, max_points: usize) -> Vec<usize> {
    if len == 0 {
        return Vec::new();
    }
    let stride = (len / max_points).max(1);
    let mut idx: Vec<usize> = (0..len).step_by(stride).collect();
    if *idx.last().expect("non-empty") != len - 1 {
        idx.push(len - 1);
    }
    idx
}

fn result_dict(py: Python<'_>, r: &PreviewResult, max_points: usize) -> PyResult<PyObject> {
    let d = PyDict::new(py);
    let idx = sample_indices(r.joint_trajectory_rad.len(), max_points);
    let traj = PyList::empty(py);
    let poses = PyList::empty(py);
    let xyzrpy = PyList::empty(py);
    for &i in &idx {
        traj.append(r.joint_trajectory_rad[i].to_vec())?;
        if let Some(p) = r.tcp_poses.get(i) {
            poses.append(p.to_vec())?;
            xyzrpy.append(matrix_to_xyzrpy(p).to_vec())?;
        }
    }
    d.set_item("joint_trajectory_rad", traj)?;
    d.set_item("tcp_poses", poses)?;
    d.set_item("tcp_xyzrpy", xyzrpy)?;
    d.set_item("end_joints_rad", r.end_joints_rad.to_vec())?;
    d.set_item("duration_s", r.duration_s)?;
    d.set_item("pending", r.pending)?;
    match &r.error {
        Some(e) => d.set_item("error", wire_error_tuple(py, e))?,
        None => d.set_item("error", py.None())?,
    }
    Ok(d.into_any().unbind())
}

/// The offline preview session (see `par6d::preview`).
#[pyclass(module = "par6._par6")]
pub struct Preview {
    inner: Mutex<EnginePreview>,
    max_points: usize,
}

#[pymethods]
impl Preview {
    /// Build a session from a robot config path (the runtime's own
    /// search when `None`) and assets tree, starting referenced at the
    /// park pose. Trajectories are downsampled to `max_points` samples
    /// (endpoints kept) on the way out.
    #[new]
    #[pyo3(signature = (config=None, assets=None, max_points=200))]
    fn new(config: Option<String>, assets: Option<String>, max_points: usize) -> PyResult<Self> {
        let inner = EnginePreview::new(
            config.map(std::path::PathBuf::from).as_deref(),
            assets.map(std::path::PathBuf::from).as_deref(),
        )
        .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        Ok(Self {
            inner: Mutex::new(inner),
            max_points: max_points.max(2),
        })
    }

    /// Submit one command dict (`type` selects the family; the other
    /// keys mirror the wire fields): the result dict, or `None` while
    /// the command waits in the blend hold. A refusal comes back as the
    /// result's `error` six-tuple — the runtime's own text.
    fn submit(&self, py: Python<'_>, command: &Bound<'_, PyDict>) -> PyResult<Option<PyObject>> {
        let cmd = command_from_py(command)?;
        let r = self.inner.lock().unwrap().submit(cmd);
        if r.pending {
            return Ok(None);
        }
        result_dict(py, &r, self.max_points).map(Some)
    }

    /// Plan whatever the blend hold still holds; `None` when nothing waits.
    fn flush(&self, py: Python<'_>) -> PyResult<Option<PyObject>> {
        match self.inner.lock().unwrap().flush() {
            Some(r) => result_dict(py, &r, self.max_points).map(Some),
            None => Ok(None),
        }
    }

    /// The virtual arm pose \[rad\].
    fn angles_rad(&self) -> Vec<f64> {
        self.inner.lock().unwrap().angles_rad().to_vec()
    }

    /// The virtual arm pose in degrees (the wire unit).
    fn angles_deg(&self) -> Vec<f64> {
        self.inner
            .lock()
            .unwrap()
            .angles_rad()
            .iter()
            .map(|r| r.to_degrees())
            .collect()
    }

    /// The TCP pose as `[x, y, z, rx, ry, rz]` in mm and degrees, the
    /// wire convention.
    fn pose_xyzrpy(&self) -> PyResult<Vec<f64>> {
        let m = self
            .inner
            .lock()
            .unwrap()
            .pose()
            .map_err(|e| robot_err(&e))?;
        let p = matrix_to_xyzrpy(&m);
        Ok(vec![
            p[0] * 1000.0,
            p[1] * 1000.0,
            p[2] * 1000.0,
            p[3].to_degrees(),
            p[4].to_degrees(),
            p[5].to_degrees(),
        ])
    }

    /// Seed the virtual arm at `q` without the wire's checks (a host
    /// mirroring the live arm's pose).
    fn place_rad(&self, q: [f64; NUM_JOINTS]) {
        self.inner.lock().unwrap().place_rad(q);
    }

    fn homed(&self) -> bool {
        self.inner.lock().unwrap().homed()
    }

    fn set_homed(&self, homed: bool) {
        self.inner.lock().unwrap().set_homed(homed);
    }

    /// FK at the virtual pose (flattened 4×4, translation in metres).
    fn pose(&self) -> PyResult<Vec<f64>> {
        self.inner
            .lock()
            .unwrap()
            .pose()
            .map(|p| p.to_vec())
            .map_err(|e| robot_err(&e))
    }

    /// Registered motion profile names.
    #[staticmethod]
    fn profiles() -> Vec<String> {
        EnginePreview::profiles()
    }

    /// The active profile, in the registry's spelling.
    fn profile(&self) -> String {
        self.inner.lock().unwrap().profile().to_owned()
    }

    fn tcp_offset_mm(&self) -> Vec<f64> {
        self.inner.lock().unwrap().tcp_offset_mm().to_vec()
    }

    /// `(tool key, variant key or None)`.
    fn tool(&self) -> (String, Option<String>) {
        let p = self.inner.lock().unwrap();
        let (key, variant) = p.tool();
        (key.to_owned(), variant.map(str::to_owned))
    }

    /// Commanded jaw position, 0 = open … 1 = closed.
    fn tool_position(&self) -> f64 {
        self.inner.lock().unwrap().tool_position()
    }

    fn tool_calibrated(&self) -> bool {
        self.inner.lock().unwrap().tool_calibrated()
    }

    /// `inputs ++ outputs ++ [estop]`, the STATUS layout.
    fn io(&self) -> Vec<u8> {
        self.inner.lock().unwrap().io()
    }

    /// Wire names of the commands waiting in the blend hold.
    fn queue(&self) -> Vec<String> {
        self.inner
            .lock()
            .unwrap()
            .held_names()
            .into_iter()
            .map(str::to_owned)
            .collect()
    }

    /// The tick period \[s\] trajectories are sampled at.
    fn tick_dt_s(&self) -> f64 {
        self.inner.lock().unwrap().tick_dt_s()
    }

    /// The effective `[motion]` feel constants, keyed by config name.
    fn motion<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let m = self.inner.lock().unwrap().motion();
        motion_dict(py, &m)
    }

    /// Where the configured homing seek leaves the arm \[rad\].
    fn homing_ready_pose_rad(&self) -> Vec<f64> {
        self.inner.lock().unwrap().homing_ready_pose_rad().to_vec()
    }

    /// Replace one collision-world layer ("installation" or "program",
    /// wire units); raises `RobotWireError` exactly when the runtime
    /// would refuse the set.
    fn set_shapes(&self, layer: &str, shapes: Vec<Bound<'_, PyDict>>) -> PyResult<Option<u64>> {
        let layer = layer_of(layer)?;
        let shapes = shapes
            .iter()
            .map(shape_from_py)
            .collect::<PyResult<Vec<_>>>()?;
        self.inner
            .lock()
            .unwrap()
            .set_shapes(layer, &shapes)
            .map_err(|e| robot_err(&e))
    }
}
