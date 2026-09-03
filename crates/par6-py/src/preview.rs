//! The offline dry-run binding over `par6d::preview` — the daemon's own
//! planner behind a virtual arm. The Python shim builds command dicts;
//! planning, timing and collision gating all happen in the engine.

use std::sync::{Mutex, MutexGuard, PoisonError};

use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};

use par6_proto::command as cmd;
use par6_proto::{Command, CompletionPolicy, NUM_JOINTS};
use par6_server::{PayloadSpec, ShapeLayer};
use par6d::preview::{Preview as EnginePreview, PreviewResult};

use crate::convert::{frame_of, robot_err, shape_from_py, tool_param_from_py, wire_error_tuple};

fn get<'py, T: pyo3::FromPyObject<'py>>(d: &Bound<'py, PyDict>, k: &str) -> PyResult<T> {
    d.get_item(k)?
        .ok_or_else(|| PyRuntimeError::new_err(format!("command is missing '{k}'")))?
        .extract()
}

fn opt<'py, T: pyo3::FromPyObject<'py>>(d: &Bound<'py, PyDict>, k: &str) -> PyResult<Option<T>> {
    match d.get_item(k)? {
        Some(v) if !v.is_none() => Ok(Some(v.extract()?)),
        _ => Ok(None),
    }
}

/// One command dict → wire command. `type` selects the family; the other
/// keys mirror the client method arguments (wire units).
fn command_from_py(d: &Bound<'_, PyDict>) -> PyResult<Command> {
    let kind: String = get(d, "type")?;
    let c = match kind.as_str() {
        "home" => Command::Home(cmd::Home {
            key: 0,
            calibrate: opt(d, "calibrate")?.unwrap_or(false),
        }),
        "move_j" => Command::MoveJ(cmd::MoveJ {
            key: 0,
            angles: get(d, "angles")?,
            duration: opt(d, "duration")?,
            speed: opt(d, "speed")?,
            accel: opt(d, "accel")?,
            blend_radius: opt(d, "blend_radius")?,
            rel: opt(d, "rel")?.unwrap_or(false),
        }),
        "move_j_pose" => Command::MoveJPose(cmd::MoveJPose {
            key: 0,
            pose: get(d, "pose")?,
            duration: opt(d, "duration")?,
            speed: opt(d, "speed")?,
            accel: opt(d, "accel")?,
            blend_radius: opt(d, "blend_radius")?,
        }),
        "move_l" => Command::MoveL(cmd::MoveL {
            key: 0,
            pose: get(d, "pose")?,
            frame: frame_of(opt(d, "frame")?.unwrap_or(0))?,
            duration: opt(d, "duration")?,
            speed: opt(d, "speed")?,
            accel: opt(d, "accel")?,
            blend_radius: opt(d, "blend_radius")?,
            rel: opt(d, "rel")?.unwrap_or(false),
        }),
        "move_c" => Command::MoveC(cmd::MoveC {
            key: 0,
            via: get(d, "via")?,
            end: get(d, "end")?,
            frame: frame_of(opt(d, "frame")?.unwrap_or(0))?,
            duration: opt(d, "duration")?,
            speed: opt(d, "speed")?,
            accel: opt(d, "accel")?,
            blend_radius: opt(d, "blend_radius")?,
            rel: opt(d, "rel")?.unwrap_or(false),
        }),
        "move_s" => Command::MoveS(cmd::MoveS {
            key: 0,
            waypoints: get(d, "waypoints")?,
            frame: frame_of(opt(d, "frame")?.unwrap_or(0))?,
            duration: opt(d, "duration")?,
            speed: opt(d, "speed")?,
            accel: opt(d, "accel")?,
            rel: opt(d, "rel")?.unwrap_or(false),
        }),
        "move_p" => Command::MoveP(cmd::MoveP {
            key: 0,
            waypoints: get(d, "waypoints")?,
            frame: frame_of(opt(d, "frame")?.unwrap_or(0))?,
            duration: opt(d, "duration")?,
            speed: opt(d, "speed")?,
            accel: opt(d, "accel")?,
            rel: opt(d, "rel")?.unwrap_or(false),
        }),
        "delay" => Command::Delay(cmd::Delay {
            key: 0,
            seconds: get(d, "seconds")?,
        }),
        "checkpoint" => Command::Checkpoint(cmd::Checkpoint {
            key: 0,
            label: get(d, "label")?,
        }),
        "select_tool" => Command::SelectTool(cmd::SelectTool {
            key: 0,
            tool_name: get(d, "tool_name")?,
            variant_key: opt(d, "variant_key")?,
        }),
        "tool_action" => {
            let params: Vec<Bound<'_, PyAny>> = get(d, "params")?;
            Command::ToolAction(cmd::ToolAction {
                key: 0,
                tool_key: get(d, "tool_key")?,
                action: get(d, "action")?,
                params: params
                    .iter()
                    .map(tool_param_from_py)
                    .collect::<PyResult<Vec<_>>>()?,
            })
        }
        other => {
            return Err(PyRuntimeError::new_err(format!(
                "unknown preview command type '{other}'"
            )))
        }
    };
    Ok(c)
}

/// Which of `n` trajectory samples survive thinning to about
/// `max_points`: every `stride`-th one plus the last, so the endpoints
/// are always kept. `None` keeps every sample.
fn kept(n: usize, max_points: Option<usize>) -> impl Fn(usize) -> bool {
    let stride = max_points.map_or(1, |m| (n / m.max(2)).max(1));
    move |k| k % stride == 0 || k + 1 == n
}

fn result_dict(py: Python<'_>, r: &PreviewResult, max_points: Option<usize>) -> PyResult<PyObject> {
    let d = PyDict::new(py);
    let keep = kept(r.joint_trajectory_rad.len(), max_points);
    let traj = PyList::empty(py);
    let poses = PyList::empty(py);
    for (k, q) in r.joint_trajectory_rad.iter().enumerate() {
        if keep(k) {
            traj.append(&q[..])?;
            if let Some(p) = r.tcp_poses.get(k) {
                poses.append(&p[..])?;
            }
        }
    }
    d.set_item("joint_trajectory_rad", traj)?;
    d.set_item("tcp_poses", poses)?;
    d.set_item("end_joints_rad", &r.end_joints_rad[..])?;
    d.set_item("duration_s", r.duration_s)?;
    match &r.error {
        Some(e) => d.set_item("error", wire_error_tuple(py, e))?,
        None => d.set_item("error", py.None())?,
    }
    Ok(d.into_any().unbind())
}

/// The offline preview session (see `par6d::preview`).
#[pyclass]
pub struct Preview {
    inner: Mutex<EnginePreview>,
}

impl Preview {
    /// The engine, recovered from a poisoned lock: a planner panic in
    /// one call must not take every later preview down with it.
    fn engine(&self) -> MutexGuard<'_, EnginePreview> {
        self.inner.lock().unwrap_or_else(PoisonError::into_inner)
    }
}

#[pymethods]
impl Preview {
    /// Build a session from a robot config path (default search when
    /// `None`) and assets tree, starting at the configured park pose.
    #[new]
    #[pyo3(signature = (config=None, assets=None))]
    fn new(config: Option<String>, assets: Option<String>) -> PyResult<Self> {
        let inner = EnginePreview::new(
            config.map(std::path::PathBuf::from).as_deref(),
            assets.map(std::path::PathBuf::from).as_deref(),
        )
        .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        Ok(Self {
            inner: Mutex::new(inner),
        })
    }

    /// The virtual arm pose \[rad\].
    fn angles_rad(&self) -> Vec<f64> {
        self.engine().angles_rad().to_vec()
    }

    /// Move the virtual arm instantly.
    fn teleport_rad(&self, q: [f64; NUM_JOINTS]) {
        self.engine().teleport_rad(q);
    }

    /// Whether the virtual arm holds its position references.
    fn homed(&self) -> bool {
        self.engine().homed()
    }

    /// Set the virtual arm's homed state (see the engine preview: while
    /// unhomed, gated commands refuse with the server's own refusal).
    fn set_homed(&self, homed: bool) {
        self.engine().set_homed(homed);
    }

    /// FK at the virtual pose (flattened 4×4, translation in metres).
    fn pose(&self) -> PyResult<Vec<f64>> {
        self.engine()
            .pose()
            .map(|p| p.to_vec())
            .map_err(|e| robot_err(&e))
    }

    /// Registered motion profile names.
    #[staticmethod]
    fn profiles() -> Vec<String> {
        EnginePreview::profiles()
    }

    /// The tick period \[s\] trajectories are sampled at.
    fn tick_dt_s(&self) -> f64 {
        self.engine().tick_dt_s()
    }

    /// The effective `[motion]` feel constants, keyed by config name —
    /// the same file the daemon reads.
    fn motion(&self, py: Python<'_>) -> PyResult<PyObject> {
        let m = self.engine().motion();
        Ok(crate::convert::motion_dict(py, &m.as_array())?.into())
    }

    /// Apply planning context (profile, TCP offset \[mm\], completion
    /// policy) — the same sync the server pushes to the live planner.
    fn set_context(&self, profile: &str, tcp_offset_mm: [f64; 3], policy: u8) -> PyResult<()> {
        let policy = CompletionPolicy::from_wire(i64::from(policy)).ok_or_else(|| {
            PyRuntimeError::new_err(format!("unknown completion policy {policy}"))
        })?;
        self.engine().set_context(profile, tcp_offset_mm, policy);
        Ok(())
    }

    /// Replace one collision-world layer ("installation" or "program",
    /// wire units); raises `RobotWireError` exactly when the runtime
    /// would refuse the set.
    fn set_shapes(
        &self,
        py: Python<'_>,
        layer: &str,
        shapes: Vec<Bound<'_, PyDict>>,
    ) -> PyResult<Option<u64>> {
        let layer = match layer {
            "installation" => ShapeLayer::Installation,
            "program" => ShapeLayer::Program,
            other => {
                return Err(PyRuntimeError::new_err(format!(
                    "unknown shape layer '{other}'"
                )))
            }
        };
        let shapes = shapes
            .iter()
            .map(shape_from_py)
            .collect::<PyResult<Vec<_>>>()?;
        py.allow_threads(|| self.engine().set_shapes(layer, &shapes))
            .map_err(|e| robot_err(&e))
    }

    /// Preview a velocity jog (signed fractions per joint) held for
    /// `duration` seconds — the runtime's own jog ramp integrated from
    /// the virtual pose. Wire-invalid parameters come back as the
    /// result's `error`, exactly as the runtime would refuse them. The
    /// trajectory is thinned to about `max_points` samples, endpoints
    /// kept.
    #[pyo3(signature = (speeds, duration, accel=None, max_points=None))]
    fn preview_jog(
        &self,
        py: Python<'_>,
        speeds: [f64; NUM_JOINTS],
        duration: f64,
        accel: Option<f64>,
        max_points: Option<usize>,
    ) -> PyResult<PyObject> {
        let r = py.allow_threads(|| self.engine().preview_jog(speeds, duration, accel));
        result_dict(py, &r, max_points)
    }

    /// Preview a cartesian velocity jog: signed fractions per axis (xyz
    /// then rotation about xyz) held for `duration` seconds in `frame`
    /// (0 = WRF, 1 = TRF) — the runtime's own twist integration through
    /// the same kinematics and soft window, gated on the collision
    /// world. Wire-invalid parameters come back as the result's `error`;
    /// the trajectory is thinned to about `max_points` samples.
    #[pyo3(signature = (velocities, duration, frame=0, accel=None, max_points=None))]
    fn preview_jog_l(
        &self,
        py: Python<'_>,
        velocities: [f64; 6],
        duration: f64,
        frame: u8,
        accel: Option<f64>,
        max_points: Option<usize>,
    ) -> PyResult<PyObject> {
        let frame = frame_of(frame)?;
        let r = py.allow_threads(|| {
            self.engine()
                .preview_jog_l(velocities, frame, duration, accel)
        });
        result_dict(py, &r, max_points)
    }

    /// The payload the preview plans with, as the live `set_payload`
    /// pushes it: mass \[kg\], COM \[m\] in the end-effector frame, and
    /// the inertia `(Ixx, Ixy, Iyy, Ixz, Iyz, Izz)` or None for a point
    /// mass.
    #[pyo3(signature = (mass, com, inertia=None))]
    fn set_payload(&self, mass: f64, com: [f64; 3], inertia: Option<[f64; 6]>) {
        self.engine()
            .set_payload(PayloadSpec { mass, com, inertia });
    }

    /// Preview a queued program (list of command dicts, see
    /// `command_from_py`): one result dict per command, blend chains
    /// folded exactly as the live planner folds them. Each trajectory is
    /// thinned to about `max_points` samples, endpoints kept.
    #[pyo3(signature = (cmds, max_points=None))]
    fn preview_program(
        &self,
        py: Python<'_>,
        cmds: Vec<Bound<'_, PyDict>>,
        max_points: Option<usize>,
    ) -> PyResult<Vec<PyObject>> {
        let commands = cmds
            .iter()
            .map(command_from_py)
            .collect::<PyResult<Vec<_>>>()?;
        let results = py.allow_threads(|| self.engine().preview_batch(&commands));
        results
            .iter()
            .map(|r| result_dict(py, r, max_points))
            .collect()
    }
}
