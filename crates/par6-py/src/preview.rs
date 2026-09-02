//! The offline dry-run binding over `par6d::preview` — the daemon's own
//! planner behind a virtual arm. The Python shim builds command dicts;
//! planning, timing and collision gating all happen in the engine.

use std::sync::Mutex;

use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};

use par6_proto::command as cmd;
use par6_proto::{Command, CompletionPolicy, Frame, NUM_JOINTS};
use par6_server::ShapeLayer;
use par6d::preview::{Preview as EnginePreview, PreviewResult};

use crate::convert::{robot_err, shape_from_py, tool_param_from_py, wire_error_tuple};

fn frame_of(v: u8) -> PyResult<Frame> {
    Frame::from_wire(i64::from(v))
        .ok_or_else(|| PyRuntimeError::new_err(format!("unknown frame {v}")))
}

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

fn result_dict(py: Python<'_>, r: &PreviewResult) -> PyResult<PyObject> {
    let d = PyDict::new(py);
    let traj = PyList::empty(py);
    for q in &r.joint_trajectory_rad {
        traj.append(q.to_vec())?;
    }
    d.set_item("joint_trajectory_rad", traj)?;
    let poses = PyList::empty(py);
    for p in &r.tcp_poses {
        poses.append(p.to_vec())?;
    }
    d.set_item("tcp_poses", poses)?;
    d.set_item("end_joints_rad", r.end_joints_rad.to_vec())?;
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
        self.inner.lock().unwrap().angles_rad().to_vec()
    }

    /// Move the virtual arm instantly.
    fn teleport_rad(&self, q: [f64; NUM_JOINTS]) {
        self.inner.lock().unwrap().teleport_rad(q);
    }

    /// Whether the virtual arm holds its position references.
    fn homed(&self) -> bool {
        self.inner.lock().unwrap().homed()
    }

    /// Set the virtual arm's homed state (see the engine preview: while
    /// unhomed, gated commands refuse with the server's own refusal).
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

    /// The tick period \[s\] trajectories are sampled at.
    fn tick_dt_s(&self) -> f64 {
        self.inner.lock().unwrap().tick_dt_s()
    }

    /// The effective `[motion]` feel constants, keyed by config name —
    /// the same file the daemon reads.
    fn motion(&self, py: Python<'_>) -> PyResult<PyObject> {
        let m = self.inner.lock().unwrap().motion();
        let d = pyo3::types::PyDict::new(py);
        d.set_item("jog_l_linear_max_m_s", m.jog_l_linear_max_m_s)?;
        d.set_item("jog_l_angular_max_rad_s", m.jog_l_angular_max_rad_s)?;
        d.set_item("cart_step_m", m.cart_step_m)?;
        d.set_item("cart_step_rad", m.cart_step_rad)?;
        d.set_item("move_l_max_joint_step_rad", m.move_l_max_joint_step_rad)?;
        d.set_item("dls_lambda", m.dls_lambda)?;
        d.set_item("settle_tolerance_rad", m.settle_tolerance_rad)?;
        d.set_item("settle_timeout_s", m.settle_timeout_s)?;
        Ok(d.into())
    }

    /// Apply planning context (profile, TCP offset \[mm\], completion
    /// policy) — the same sync the server pushes to the live planner.
    fn set_context(&self, profile: &str, tcp_offset_mm: [f64; 3], policy: u8) -> PyResult<()> {
        let policy = CompletionPolicy::from_wire(i64::from(policy)).ok_or_else(|| {
            PyRuntimeError::new_err(format!("unknown completion policy {policy}"))
        })?;
        self.inner
            .lock()
            .unwrap()
            .set_context(profile, tcp_offset_mm, policy);
        Ok(())
    }

    /// Replace one collision-world layer ("installation" or "program",
    /// wire units); raises `RobotWireError` exactly when the runtime
    /// would refuse the set.
    fn set_shapes(&self, layer: &str, shapes: Vec<Bound<'_, PyDict>>) -> PyResult<Option<u64>> {
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
        self.inner
            .lock()
            .unwrap()
            .set_shapes(layer, &shapes)
            .map_err(|e| robot_err(&e))
    }

    /// Preview a velocity jog (signed fractions per joint) held for
    /// `duration` seconds — the runtime's own jog ramp integrated from
    /// the virtual pose. Wire-invalid parameters come back as the
    /// result's `error`, exactly as the runtime would refuse them.
    #[pyo3(signature = (speeds, duration, accel=None))]
    fn preview_jog(
        &self,
        py: Python<'_>,
        speeds: [f64; NUM_JOINTS],
        duration: f64,
        accel: Option<f64>,
    ) -> PyResult<PyObject> {
        let r = self
            .inner
            .lock()
            .unwrap()
            .preview_jog(speeds, duration, accel);
        result_dict(py, &r)
    }

    /// Preview a queued program (list of command dicts, see
    /// `command_from_py`): one result dict per command, blend chains
    /// folded exactly as the live planner folds them.
    fn preview_program(
        &self,
        py: Python<'_>,
        cmds: Vec<Bound<'_, PyDict>>,
    ) -> PyResult<Vec<PyObject>> {
        let commands = cmds
            .iter()
            .map(command_from_py)
            .collect::<PyResult<Vec<_>>>()?;
        let results = self.inner.lock().unwrap().preview_batch(&commands);
        results.iter().map(|r| result_dict(py, r)).collect()
    }
}
