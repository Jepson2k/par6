//! The engine's kinematics model for the Python client: the same
//! `par6_kin::Kin` the daemon plans with, so the client's FK/IK and the
//! runtime's cannot disagree — one URDF tree, one analytic solver, the
//! same soft-window branch choice the runtime's IK makes.

use std::path::Path;
use std::sync::Mutex;

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};

use par6_kin::{relative_pose, wrap_to_window, IkOutcome, Kin, Pose, NQ};
use par6d::{matrix_to_xyzrpy, translate_local};

/// FK/IK on one URDF tree, resolved at `ee_frame` (the tree's last frame
/// when `None`), optionally past a fixed `tool_transform`. Poses are
/// row-major 4x4 transforms flattened to 16, metres.
#[pyclass(module = "par6._par6")]
pub struct Kinematics {
    kin: Mutex<Kin>,
    /// Soft window per joint; IK solutions are wrapped onto the branch
    /// inside it nearest the seed, as the runtime's solver does.
    window: Option<Vec<(f64, f64)>>,
}

fn joints(q: &[f64], what: &str) -> PyResult<[f64; NQ]> {
    if q.len() < NQ {
        return Err(PyValueError::new_err(format!(
            "{what} has {} joints, need {NQ}",
            q.len()
        )));
    }
    let mut out = [0.0; NQ];
    out.copy_from_slice(&q[..NQ]);
    Ok(out)
}

fn pose16(m: Vec<f64>, what: &str) -> PyResult<Pose> {
    m.try_into()
        .map_err(|_| PyValueError::new_err(format!("{what} must be a flattened 4x4 pose")))
}

fn pose6(p: &[f64], what: &str) -> PyResult<Pose> {
    if p.len() != 6 {
        return Err(PyValueError::new_err(format!(
            "{what} must be [x, y, z, rx, ry, rz], got {} values",
            p.len()
        )));
    }
    Ok(par6_proto::pose_matrix(
        [p[0], p[1], p[2]],
        [p[3], p[4], p[5]],
    ))
}

/// Largest element-wise gap between two poses — the IK residual.
fn residual(a: &Pose, b: &Pose) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

impl Kinematics {
    fn solve(&self, kin: &mut Kin, seed: &[f64; NQ], target: &Pose) -> PyResult<Option<[f64; NQ]>> {
        let mut q = [0.0; NQ];
        match kin
            .ik(seed, target, &mut q)
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?
        {
            IkOutcome::Converged => {
                if let Some(window) = &self.window {
                    for (j, v) in q.iter_mut().enumerate() {
                        *v = wrap_to_window(*v, seed[j], window[j].0, window[j].1);
                    }
                }
                Ok(Some(q))
            }
            IkOutcome::Unreachable => Ok(None),
        }
    }

    /// Joints outside the soft window; a non-finite value is never inside.
    fn outside_window(&self, q: &[f64; NQ]) -> Vec<usize> {
        (0..NQ)
            .filter(|&j| {
                !q[j].is_finite()
                    || self
                        .window
                        .as_ref()
                        .is_some_and(|w| q[j] < w[j].0 - 1e-9 || q[j] > w[j].1 + 1e-9)
            })
            .collect()
    }

    fn solution_dict<'py>(
        &self,
        py: Python<'py>,
        kin: &mut Kin,
        q: [f64; NQ],
        target: &Pose,
    ) -> PyResult<Bound<'py, PyDict>> {
        let mut reached = [0.0; 16];
        kin.fk(&q, &mut reached)
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        let d = PyDict::new(py);
        d.set_item("q", q.to_vec())?;
        d.set_item("residual", residual(&reached, target))?;
        d.set_item("violations", self.outside_window(&q))?;
        Ok(d)
    }
}

#[pymethods]
impl Kinematics {
    #[new]
    #[pyo3(signature = (urdf, ee_frame=None, tool_transform=None, soft_limits=None))]
    fn new(
        urdf: &str,
        ee_frame: Option<&str>,
        tool_transform: Option<Vec<f64>>,
        soft_limits: Option<Vec<(f64, f64)>>,
    ) -> PyResult<Self> {
        let kin = match tool_transform {
            Some(m) => {
                let t = pose16(m, "tool_transform")?;
                Kin::from_urdf_with_frame(Path::new(urdf), ee_frame, &t)
            }
            None => Kin::from_urdf(Path::new(urdf), ee_frame),
        }
        .map_err(|e| PyRuntimeError::new_err(format!("{urdf}: {e}")))?;
        if let Some(w) = &soft_limits {
            if w.len() < NQ {
                return Err(PyValueError::new_err(format!(
                    "soft_limits has {} joints, need {NQ}",
                    w.len()
                )));
            }
        }
        Ok(Self {
            kin: Mutex::new(kin),
            window: soft_limits,
        })
    }

    /// Position variables in the tree (arm joints plus any passive ones).
    fn nq_full(&self) -> usize {
        self.kin.lock().unwrap().nq_full()
    }

    /// End-effector pose at `q` (first six entries are the arm).
    fn fk(&self, q: Vec<f64>) -> PyResult<Vec<f64>> {
        let q = joints(&q, "q")?;
        let mut pose = [0.0; 16];
        self.kin
            .lock()
            .unwrap()
            .fk(&q, &mut pose)
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        Ok(pose.to_vec())
    }

    /// `[x, y, z, rx, ry, rz]` (m, rad; intrinsic-XYZ rpy) at `q` — NaN
    /// when the pose cannot be computed.
    fn tcp(&self, q: Vec<f64>) -> PyResult<Vec<f64>> {
        let q = joints(&q, "q")?;
        let mut out = [0.0; 6];
        self.kin.lock().unwrap().tcp(&q, &mut out);
        Ok(out.to_vec())
    }

    /// `[x, y, z, rx, ry, rz]` per row of `rows`.
    fn fk_batch(&self, rows: Vec<Vec<f64>>) -> PyResult<Vec<Vec<f64>>> {
        let mut kin = self.kin.lock().unwrap();
        rows.iter()
            .map(|row| {
                let q = joints(row, "q")?;
                let mut out = [0.0; 6];
                kin.tcp(&q, &mut out);
                Ok(out.to_vec())
            })
            .collect()
    }

    /// Closed-form IK for the 4x4 `target`, on the branch nearest `seed`
    /// inside the soft window; `None` when the pose is out of reach.
    /// Raises when the tree has no analytic model.
    fn ik(&self, seed: Vec<f64>, target: Vec<f64>) -> PyResult<Option<Vec<f64>>> {
        let seed = joints(&seed, "seed")?;
        let target = pose16(target, "target")?;
        let mut kin = self.kin.lock().unwrap();
        Ok(self.solve(&mut kin, &seed, &target)?.map(|q| q.to_vec()))
    }

    /// IK for a `[x, y, z, rx, ry, rz]` target (m, rad): `{q, residual,
    /// violations}` — `violations` lists the joints the solution leaves
    /// outside the soft window — or `None` when out of reach.
    fn ik_pose<'py>(
        &self,
        py: Python<'py>,
        seed: Vec<f64>,
        pose: Vec<f64>,
    ) -> PyResult<Option<Bound<'py, PyDict>>> {
        let seed = joints(&seed, "seed")?;
        let target = pose6(&pose, "pose")?;
        let mut kin = self.kin.lock().unwrap();
        match self.solve(&mut kin, &seed, &target)? {
            Some(q) => Ok(Some(self.solution_dict(py, &mut kin, q, &target)?)),
            None => Ok(None),
        }
    }

    /// `ik_pose` along `poses`, each solve seeded by the previous
    /// solution (the seed carries over an unreachable pose).
    fn ik_batch<'py>(
        &self,
        py: Python<'py>,
        poses: Vec<Vec<f64>>,
        seed: Vec<f64>,
    ) -> PyResult<Bound<'py, PyList>> {
        let mut current = joints(&seed, "seed")?;
        let mut kin = self.kin.lock().unwrap();
        let out = PyList::empty(py);
        for pose in &poses {
            let target = pose6(pose, "pose")?;
            match self.solve(&mut kin, &current, &target)? {
                Some(q) => {
                    out.append(self.solution_dict(py, &mut kin, q, &target)?)?;
                    current = q;
                }
                None => out.append(py.None())?,
            }
        }
        Ok(out)
    }

    /// Joints `q` leaves outside the soft window (empty without limits).
    fn violations(&self, q: Vec<f64>) -> PyResult<Vec<usize>> {
        let q = joints(&q, "q")?;
        Ok(self.outside_window(&q))
    }

    /// World-axes Jacobian at the resolved frame, 6 rows `[linear;
    /// angular]` of `NQ` columns.
    fn jacobian(&self, q: Vec<f64>) -> PyResult<Vec<Vec<f64>>> {
        let q = joints(&q, "q")?;
        let mut out = [0.0; 6 * NQ];
        self.kin
            .lock()
            .unwrap()
            .jacobian(&q, &mut out)
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        Ok(out.chunks(NQ).map(|row| row.to_vec()).collect())
    }

    /// The derived OPW model: lengths \[m\], joint offsets \[rad\] and
    /// sign corrections — introspection only, nothing here is an input.
    fn opw_parameters<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let kin = self.kin.lock().unwrap();
        let opw = kin
            .opw()
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        let p = opw.parameters();
        let d = PyDict::new(py);
        for (k, v) in [
            ("a1", p.a1),
            ("a2", p.a2),
            ("b", p.b),
            ("c1", p.c1),
            ("c2", p.c2),
            ("c3", p.c3),
            ("c4", p.c4),
        ] {
            d.set_item(k, v)?;
        }
        d.set_item("offsets", p.offsets.to_vec())?;
        d.set_item("sign_corrections", p.sign_corrections.to_vec())?;
        Ok(d)
    }
}

/// `[x, y, z, rx, ry, rz]` (intrinsic-XYZ rpy) from a flattened 4x4.
#[pyfunction(name = "matrix_to_xyzrpy")]
pub fn matrix_to_xyzrpy_py(m: Vec<f64>) -> PyResult<Vec<f64>> {
    let m = pose16(m, "matrix")?;
    Ok(matrix_to_xyzrpy(&m).to_vec())
}

/// Flattened 4x4 from `xyz` and intrinsic-XYZ `rpy` \[rad\].
#[pyfunction(name = "pose_matrix")]
pub fn pose_matrix_py(xyz: [f64; 3], rpy: [f64; 3]) -> Vec<f64> {
    par6_proto::pose_matrix(xyz, rpy).to_vec()
}

/// A tool frame: `origin`/`rpy` (the tool's TCP off the flange) with
/// `offset` walked along the tool's own axes afterwards — the
/// composition the runtime applies to `set_tcp_offset`.
#[pyfunction]
pub fn compose_tool_frame(origin: [f64; 3], rpy: [f64; 3], offset: [f64; 3]) -> Vec<f64> {
    let mut m = par6_proto::pose_matrix(origin, rpy);
    translate_local(&mut m, offset);
    m.to_vec()
}

/// The fixed transform from `from_frame` to `to_frame` in `urdf`, as
/// `[x, y, z, rx, ry, rz]` (m, intrinsic-XYZ rad) — read off the
/// engine's own FK at the zero configuration.
#[pyfunction]
pub fn frame_offset(urdf: &str, from_frame: &str, to_frame: &str) -> PyResult<Vec<f64>> {
    let load = |frame: &str| -> PyResult<Pose> {
        let mut kin = Kin::from_urdf(Path::new(urdf), Some(frame))
            .map_err(|e| PyRuntimeError::new_err(format!("{urdf} at {frame:?}: {e}")))?;
        let mut pose = [0.0; 16];
        kin.fk(&[0.0; NQ], &mut pose)
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        Ok(pose)
    };
    let a = load(from_frame)?;
    let b = load(to_frame)?;
    Ok(matrix_to_xyzrpy(&relative_pose(&a, &b)).to_vec())
}
