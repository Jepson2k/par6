//! The engine's collision world for the Python client: the same
//! `par6_kin::Collision` the daemon gates motion with, reporting pairs in
//! the daemon's own vocabulary (URDF link names, `shape:` / `install:`
//! prefixes) so a frontend highlights exactly what the runtime refused.

use std::path::Path;
use std::sync::Mutex;

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyDict;

use par6_kin::{Collision, Shape};
use par6d::collision_world::{first_duplicate, ShapeNames};

use crate::convert::{joints, layer_of, shape_from_py};
use par6d::collision_world::kin_layer;

struct World {
    collision: Collision,
    names: ShapeNames,
}

/// Self-collision plus keep-out checking on one URDF tree.
#[pyclass(module = "par6._par6")]
pub struct CollisionWorld {
    inner: Mutex<World>,
}

#[pymethods]
impl CollisionWorld {
    /// Load `urdf`'s collision geometry (`package://…` meshes resolved
    /// under `package_dir`), apply `srdf`'s disabled pairs, and enforce
    /// `clearance` metres of standoff on every pair (the runtime's
    /// default when omitted).
    #[new]
    #[pyo3(signature = (urdf, package_dir=None, srdf=None, clearance=par6d::COLLISION_CLEARANCE_M))]
    fn new(
        urdf: &str,
        package_dir: Option<&str>,
        srdf: Option<&str>,
        clearance: f64,
    ) -> PyResult<Self> {
        let mut collision =
            Collision::from_urdf(Path::new(urdf), package_dir.map(Path::new), clearance)
                .map_err(|e| PyRuntimeError::new_err(format!("{urdf}: {e}")))?;
        if let Some(srdf) = srdf {
            collision
                .apply_srdf(Path::new(srdf))
                .map_err(|e| PyRuntimeError::new_err(format!("{srdf}: {e}")))?;
        }
        Ok(Self {
            inner: Mutex::new(World {
                collision,
                names: ShapeNames::default(),
            }),
        })
    }

    fn nq_full(&self) -> usize {
        self.inner.lock().unwrap().collision.nq_full()
    }

    fn clearance(&self) -> f64 {
        self.inner.lock().unwrap().collision.clearance()
    }

    /// Epoch of the applied world; moves on every accepted layer set.
    fn scene_epoch(&self) -> u64 {
        self.inner.lock().unwrap().collision.scene_epoch()
    }

    /// Active pairs: the robot's own (after the SRDF) plus one per
    /// keep-out × robot geometry.
    fn pair_count(&self) -> usize {
        self.inner.lock().unwrap().collision.pair_count()
    }

    /// Replace one layer (`"installation"` or `"program"`) with wire
    /// shape dicts, exactly as the runtime applies a `SET_SHAPES`; a
    /// refused set (bad geometry, a duplicate name) leaves the world
    /// unchanged. Returns the new epoch.
    fn set_layer(&self, layer: &str, shapes: Vec<Bound<'_, PyDict>>) -> PyResult<u64> {
        let layer = layer_of(layer)?;
        let converted = shapes
            .iter()
            .map(|d| {
                let wire = shape_from_py(d)?;
                Shape::from_proto(&wire).map_err(|e| PyValueError::new_err(e.to_string()))
            })
            .collect::<PyResult<Vec<_>>>()?;
        if let Some(name) = first_duplicate(&converted) {
            return Err(PyValueError::new_err(format!(
                "duplicate shape name {name:?} in one layer"
            )));
        }
        let mut w = self.inner.lock().unwrap();
        let epoch = w
            .collision
            .set_layer(kin_layer(layer), &converted)
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        w.names.set_layer(layer, &converted);
        Ok(epoch)
    }

    fn in_collision(&self, q: Vec<f64>) -> PyResult<bool> {
        let q = joints(&q, "q")?;
        let mut w = self.inner.lock().unwrap();
        Ok(w.collision
            .check(&q, true)
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?
            .active())
    }

    /// Colliding pairs at `q` in reporting names.
    fn pairs(&self, q: Vec<f64>) -> PyResult<Vec<(String, String)>> {
        let q = joints(&q, "q")?;
        let mut w = self.inner.lock().unwrap();
        let World { collision, names } = &mut *w;
        let report = collision
            .check(&q, false)
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        Ok(names.render(&report))
    }

    /// Smallest signed distance over the active pairs \[m\] (negative =
    /// penetrating).
    fn min_distance(&self, q: Vec<f64>) -> PyResult<f64> {
        let q = joints(&q, "q")?;
        self.inner
            .lock()
            .unwrap()
            .collision
            .min_distance(&q)
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))
    }

    /// Index of the first colliding configuration along `rows`, or -1
    /// when the whole path is clear.
    fn check_path(&self, rows: Vec<Vec<f64>>) -> PyResult<i64> {
        let mut w = self.inner.lock().unwrap();
        for (i, row) in rows.iter().enumerate() {
            let q = joints(row, "q")?;
            if w.collision
                .check(&q, true)
                .map_err(|e| PyRuntimeError::new_err(e.to_string()))?
                .active()
            {
                return Ok(i as i64);
            }
        }
        Ok(-1)
    }
}
