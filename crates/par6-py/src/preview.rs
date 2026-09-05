//! The offline dry-run binding over `par6d::preview`.
//!
//! Two passes, and the difference between them is the point.
//! [`Preview::preview_program`] plans: it asks the daemon's own planner
//! what it would drive and hands back the trajectory, fast enough to run
//! behind a keystroke. [`Preview::run_program`] *runs*: it ticks the same
//! engine the simulator ticks, and what comes back is what the arm did,
//! sag and servo lag and contact included.
//!
//! A tick record crosses as raw column buffers rather than lists of
//! lists. A minute of program is a few hundred thousand numbers, and
//! building a Python float per number costs more than the simulation
//! that produced them; `np.frombuffer` on the other side is a view.

use std::sync::Mutex;

use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyDict, PyList};

use par6_proto::command as cmd;
use par6_proto::{Command, CompletionPolicy, Frame, NUM_JOINTS};
use par6d::preview::record::{mode_name, TickBatch};
use par6d::preview::{Preview as EnginePreview, PreviewResult, RunLimits};

use crate::convert::{robot_err, shape_dict, shape_from_py, tool_param_from_py, wire_error_tuple};

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

/// One numeric column as raw bytes, written straight into the Python
/// buffer — no intermediate `Vec<u8>`, no Python object per number.
fn col<'py, const N: usize, T: Copy>(
    py: Python<'py>,
    v: &[T],
    ne: impl Fn(T) -> [u8; N],
) -> PyResult<Bound<'py, PyBytes>> {
    PyBytes::new_with(py, v.len() * N, |buf| {
        for (out, x) in buf.as_chunks_mut::<N>().0.iter_mut().zip(v) {
            *out = ne(*x);
        }
        Ok(())
    })
}

fn f32_col<'py>(py: Python<'py>, v: &[f32]) -> PyResult<Bound<'py, PyBytes>> {
    col(py, v, f32::to_ne_bytes)
}

fn bool_col<'py>(py: Python<'py>, v: &[bool]) -> PyResult<Bound<'py, PyBytes>> {
    col(py, v, |b| [u8::from(b)])
}

/// A [`TickBatch`] as the shim's `np.frombuffer` reads it. Shapes are
/// implied by `rows` and `joints`; the byte order is the machine's,
/// which is the only one either side runs on.
fn batch_dict(py: Python<'_>, b: &TickBatch) -> PyResult<PyObject> {
    let d = PyDict::new(py);
    d.set_item("row_dt_s", b.row_dt_s)?;
    d.set_item("tick_dt_s", b.tick_dt_s)?;
    d.set_item("stride", b.stride)?;
    d.set_item("joints", b.joints)?;
    d.set_item("rows", b.rows)?;
    d.set_item("q_rad", f32_col(py, &b.q_rad)?)?;
    d.set_item("q_commanded_rad", f32_col(py, &b.q_commanded_rad)?)?;
    d.set_item("tcp", f32_col(py, &b.tcp)?)?;
    d.set_item("tool_closed", f32_col(py, &b.tool_closed)?)?;
    d.set_item("tool_gripping", bool_col(py, &b.tool_gripping)?)?;
    d.set_item("com", f32_col(py, &b.com)?)?;
    d.set_item("contact_pos", f32_col(py, &b.contact_pos)?)?;
    d.set_item("contact_force", f32_col(py, &b.contact_force)?)?;
    d.set_item(
        "contact_starts",
        col(py, &b.contact_starts, u32::to_ne_bytes)?,
    )?;
    d.set_item("stop", b.stop.as_str())?;

    let modes = PyList::empty(py);
    for span in &b.modes {
        modes.append((span.start_row, mode_name(span.value)))?;
    }
    d.set_item("modes", modes)?;

    let commands = PyList::empty(py);
    for span in &b.commands {
        let cd = PyDict::new(py);
        cd.set_item("command", span.command)?;
        cd.set_item("start_row", span.start_row)?;
        cd.set_item("rows", span.rows)?;
        match &span.error {
            Some(e) => cd.set_item("error", wire_error_tuple(py, e))?,
            None => cd.set_item("error", py.None())?,
        }
        commands.append(cd)?;
    }
    d.set_item("commands", commands)?;

    let objects = PyList::empty(py);
    for t in &b.objects {
        let od = PyDict::new(py);
        od.set_item("name", &t.name)?;
        od.set_item("rows", t.poses.len())?;
        od.set_item("poses", f32_col(py, t.poses.as_flattened())?)?;
        objects.append(od)?;
    }
    d.set_item("objects", objects)?;
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
    /// `None`) and assets tree, starting at the configured park pose;
    /// `gripper` names the bundle gripper to model instead of the active one.
    #[new]
    #[pyo3(signature = (config=None, assets=None, gripper=None))]
    fn new(
        config: Option<String>,
        assets: Option<String>,
        gripper: Option<String>,
    ) -> PyResult<Self> {
        let inner = EnginePreview::new(
            config.map(std::path::PathBuf::from).as_deref(),
            assets.map(std::path::PathBuf::from).as_deref(),
            gripper.as_deref(),
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

    /// Replace the program layer (wire units); raises `RobotWireError`
    /// exactly when the runtime would refuse the set. Returns the epoch of
    /// the applied world. The installation layer is config: applied when
    /// the engine boots, and no more settable from here than from the wire.
    fn set_shapes(&self, shapes: Vec<Bound<'_, PyDict>>) -> PyResult<u64> {
        let shapes = shapes
            .iter()
            .map(shape_from_py)
            .collect::<PyResult<Vec<_>>>()?;
        self.inner
            .lock()
            .unwrap()
            .set_shapes(&shapes)
            .map_err(|e| robot_err(&e))
    }

    /// The applied world — `installation` (from the engine's config),
    /// `program` (what this session set) and `epoch`: the runtime's own
    /// SHAPES readback, for the same file.
    fn shapes(&self, py: Python<'_>) -> PyResult<PyObject> {
        let inner = self.inner.lock().unwrap();
        let world = inner.world();
        let layer = |shapes: &[par6_proto::Shape]| -> PyResult<Vec<PyObject>> {
            shapes.iter().map(|s| shape_dict(py, s)).collect()
        };
        let d = PyDict::new(py);
        d.set_item("installation", layer(world.installation())?)?;
        d.set_item("program", layer(world.program())?)?;
        d.set_item("epoch", world.epoch())?;
        Ok(d.into_any().unbind())
    }

    /// Colliding pairs at `q` \[rad\], in the runtime's reporting
    /// vocabulary (URDF link names; `install:`/`shape:`-prefixed shapes).
    fn colliding_pairs(&self, q: [f64; NUM_JOINTS]) -> PyResult<Vec<(String, String)>> {
        self.inner
            .lock()
            .unwrap()
            .colliding_pairs(&q)
            .map_err(|e| robot_err(&e))
    }

    /// Whether `q` \[rad\] collides — self or world.
    fn in_collision(&self, q: [f64; NUM_JOINTS]) -> PyResult<bool> {
        self.inner
            .lock()
            .unwrap()
            .in_collision(&q)
            .map_err(|e| robot_err(&e))
    }

    /// Minimum signed distance over every pair at `q` \[m\]; negative =
    /// penetrating.
    fn min_distance(&self, q: [f64; NUM_JOINTS]) -> PyResult<f64> {
        self.inner
            .lock()
            .unwrap()
            .min_distance(&q)
            .map_err(|e| robot_err(&e))
    }

    /// Index of the first colliding sample along `path` \[rad\], or None.
    fn first_collision(&self, path: Vec<[f64; NUM_JOINTS]>) -> PyResult<Option<usize>> {
        self.inner
            .lock()
            .unwrap()
            .first_collision(&path)
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

    /// Run a program through the engine: the same planner driving a real
    /// control loop against the simulated plant, ticked flat out. What
    /// comes back is a tick record of what the arm DID — see
    /// `batch_dict` for the columns — not a plan of what it was told to.
    ///
    /// `max_seconds` bounds the SIMULATED time, so a program that never
    /// terminates still returns, with `stop = "budget_exhausted"`.
    ///
    /// The GIL is released for the run: at roughly sixty times real time
    /// a ten minute program is some ten seconds of computing, and the
    /// caller's event loop must not stop for it.
    #[pyo3(signature = (cmds, max_seconds=None))]
    fn run_program(
        &self,
        py: Python<'_>,
        cmds: Vec<Bound<'_, PyDict>>,
        max_seconds: Option<f64>,
    ) -> PyResult<PyObject> {
        let commands = cmds
            .iter()
            .map(command_from_py)
            .collect::<PyResult<Vec<_>>>()?;
        let limits = match max_seconds {
            Some(max_seconds) => RunLimits { max_seconds },
            None => RunLimits::default(),
        };
        let batch = py.allow_threads(|| {
            self.inner
                .lock()
                .unwrap()
                .run(&commands, limits)
                .map_err(|e| PyRuntimeError::new_err(e.to_string()))
        })?;
        batch_dict(py, &batch)
    }
}
