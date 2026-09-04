//! Wire ⇄ Python conversions: STATUS frames and query results become
//! plain dicts (the Python shim owns the typed surface), errors become a
//! structured exception carrying the wire's six-tuple.

use std::time::Duration;

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};

use par6_client::ClientError;
use par6_proto::command::ToolParam;
use par6_proto::{
    Command, FlashingAssertion, Frame, QueryResult, Shape, Status, ToolStatusWire, WireError,
};
use par6_server::ShapeLayer;

/// The wire frame discriminant as a [`Frame`].
pub fn frame_of(v: u8) -> PyResult<Frame> {
    Frame::from_wire(i64::from(v))
        .ok_or_else(|| PyRuntimeError::new_err(format!("unknown frame {v}")))
}

/// Seconds beyond which a wait is "forever" — tokio's own far-future
/// horizon, so the deadline arithmetic behind every wait stays finite.
const WAIT_FOREVER_S: u64 = 86_400 * 365 * 30;

/// A Python timeout in seconds as a [`Duration`]. NaN and negative values
/// raise `ValueError`; `inf` (Python's natural "wait forever") waits as
/// long as the runtime can schedule.
pub fn checked_duration(seconds: f64, what: &str) -> PyResult<Duration> {
    if seconds.is_nan() || seconds < 0.0 {
        return Err(PyValueError::new_err(format!(
            "{what} must be a non-negative number of seconds, got {seconds}"
        )));
    }
    if seconds >= WAIT_FOREVER_S as f64 {
        return Ok(Duration::from_secs(WAIT_FOREVER_S));
    }
    Ok(Duration::from_secs_f64(seconds))
}

pyo3::create_exception!(
    _par6,
    RobotWireError,
    pyo3::exceptions::PyException,
    "A structured runtime refusal: args = (command_index, code, title, cause, effect, remedy)."
);

pub fn robot_err(e: &WireError) -> PyErr {
    RobotWireError::new_err((
        e.command_index,
        e.code,
        e.title.clone(),
        e.cause.clone(),
        e.effect.clone(),
        e.remedy.clone(),
    ))
}

/// Map a client error onto the Python surface. `Unreachable` is NOT an
/// exception — callers map it to `None`/`0`/`-1` per method — so this
/// only covers the genuinely exceptional arms.
pub fn client_err(e: ClientError) -> PyErr {
    match e {
        ClientError::Robot(err) => robot_err(&err),
        other => PyRuntimeError::new_err(other.to_string()),
    }
}

pub fn wire_error_tuple(py: Python<'_>, e: &WireError) -> PyObject {
    (
        e.command_index,
        e.code,
        e.title.clone(),
        e.cause.clone(),
        e.effect.clone(),
        e.remedy.clone(),
    )
        .into_pyobject(py)
        .expect("tuple converts")
        .into_any()
        .unbind()
}

pub fn tool_status_dict(py: Python<'_>, t: &ToolStatusWire) -> PyResult<PyObject> {
    let d = PyDict::new(py);
    d.set_item("key", &t.key)?;
    d.set_item("state", t.state as u8)?;
    d.set_item("engaged", t.engaged)?;
    d.set_item("part_detected", t.part_detected)?;
    d.set_item("fault_code", t.fault_code)?;
    d.set_item("positions", t.positions.clone())?;
    d.set_item("channels", t.channels.clone())?;
    d.set_item("variant_key", &t.variant_key)?;
    Ok(d.into_any().unbind())
}

/// `Vec<u8>` converts to Python `bytes`; these flag arrays must land as
/// lists of ints (the numpy buffers slice-assign from them).
fn int_list(v: &[u8]) -> Vec<u16> {
    v.iter().map(|b| u16::from(*b)).collect()
}

/// One STATUS frame as a dict of plain values (field names match the
/// Python `StatusBuffer`).
pub fn status_dict(py: Python<'_>, s: &Status) -> PyResult<PyObject> {
    let d = PyDict::new(py);
    d.set_item("proto_version", s.proto_version)?;
    d.set_item("controller_id", s.controller_id)?;
    d.set_item("seq", s.seq)?;
    d.set_item("mono_time_ns", s.mono_time_ns)?;
    d.set_item("link_ok", s.link_ok)?;
    d.set_item("data_age_ms", s.data_age_ms)?;
    d.set_item("pose", s.pose.to_vec())?;
    d.set_item("angles", s.angles.to_vec())?;
    d.set_item("speeds", s.speeds.to_vec())?;
    d.set_item("io", int_list(&s.io))?;
    d.set_item("action_current", &s.action_current)?;
    d.set_item("action_state", s.action_state as u8)?;
    d.set_item("joint_en", int_list(&s.joint_en))?;
    d.set_item("cart_en_wrf", int_list(&s.cart_en_wrf))?;
    d.set_item("cart_en_trf", int_list(&s.cart_en_trf))?;
    d.set_item("executing_index", s.executing_index)?;
    d.set_item("completed_index", s.completed_index)?;
    d.set_item("last_checkpoint", &s.last_checkpoint)?;
    match &s.error {
        Some(e) => d.set_item("error", wire_error_tuple(py, e))?,
        None => d.set_item("error", py.None())?,
    }
    d.set_item("queued_segments", s.queued_segments)?;
    d.set_item("queued_duration", s.queued_duration)?;
    d.set_item("action_params", &s.action_params)?;
    match &s.tool_status {
        Some(t) => d.set_item("tool_status", tool_status_dict(py, t)?)?,
        None => d.set_item("tool_status", py.None())?,
    }
    d.set_item("tcp_speed", s.tcp_speed)?;
    d.set_item("simulator_active", s.simulator_active)?;
    d.set_item("collision_active", s.collision_active)?;
    d.set_item("collision_pairs", s.collision_pairs.clone())?;
    d.set_item("scene_epoch", s.scene_epoch)?;
    d.set_item("accepted_index", s.accepted_index)?;
    d.set_item("homed", s.homed)?;
    d.set_item("torques", s.torques.to_vec())?;
    d.set_item("mode", s.mode as u8)?;
    d.set_item("enabled", s.enabled)?;
    d.set_item("gravity_comp", s.gravity_comp)?;
    let warnings = PyList::empty(py);
    for w in &s.warnings {
        warnings.append(wire_error_tuple(py, w))?;
    }
    d.set_item("warnings", warnings)?;
    let lh = PyDict::new(py);
    lh.set_item("state", s.link_health.state)?;
    lh.set_item("restarts", s.link_health.restarts)?;
    lh.set_item("tx_errors", s.link_health.tx_errors)?;
    lh.set_item("rx_frames", s.link_health.rx_frames)?;
    d.set_item("link_health", lh)?;
    let homing = PyDict::new(py);
    homing.set_item("active", s.homing.active)?;
    homing.set_item("sequence_step", s.homing.sequence_step)?;
    homing.set_item("joints", s.homing.joints.clone())?;
    d.set_item("homing", homing)?;
    d.set_item("torques_ext", s.torques_ext.to_vec())?;
    d.set_item("paused", s.paused)?;
    let dh = PyDict::new(py);
    dh.set_item("temperatures_c", s.drive_health.temperatures_c.clone())?;
    dh.set_item("currents_ma", s.drive_health.currents_ma.clone())?;
    dh.set_item("bus_voltage_v", s.drive_health.bus_voltage_v)?;
    d.set_item("drive_health", dh)?;
    let loop_health = PyDict::new(py);
    loop_health.set_item("p99_period_s", s.loop_health.p99_period_s)?;
    loop_health.set_item("overruns", s.loop_health.overruns)?;
    d.set_item("loop_health", loop_health)?;
    Ok(d.into_any().unbind())
}

pub(crate) fn shape_dict(py: Python<'_>, s: &Shape) -> PyResult<PyObject> {
    let d = PyDict::new(py);
    d.set_item("kind", &s.kind)?;
    d.set_item("params", s.params.clone())?;
    d.set_item("pose", s.pose.clone())?;
    d.set_item("collision", s.collision)?;
    d.set_item("margin", s.margin)?;
    d.set_item("name", &s.name)?;
    Ok(d.into_any().unbind())
}

/// A composite query result as a dict tagged with its query name.
pub fn query_result_dict(py: Python<'_>, r: &QueryResult) -> PyResult<PyObject> {
    let d = PyDict::new(py);
    match r {
        QueryResult::Ping { hardware_connected } => {
            d.set_item("hardware_connected", *hardware_connected)?;
        }
        QueryResult::Status {
            pose,
            angles,
            speeds,
            io,
            tool_status,
        } => {
            d.set_item("pose", pose.to_vec())?;
            d.set_item("angles", angles.to_vec())?;
            d.set_item("speeds", speeds.to_vec())?;
            d.set_item("io", int_list(io))?;
            match tool_status {
                Some(t) => d.set_item("tool_status", tool_status_dict(py, t)?)?,
                None => d.set_item("tool_status", py.None())?,
            }
        }
        QueryResult::Tools { tool, available } => {
            d.set_item("tool", tool)?;
            d.set_item("available", available.clone())?;
        }
        QueryResult::Queue {
            queue,
            executing_index,
            completed_index,
            last_checkpoint,
            queued_duration,
        } => {
            d.set_item("queue", queue.clone())?;
            d.set_item("executing_index", *executing_index)?;
            d.set_item("completed_index", *completed_index)?;
            d.set_item("last_checkpoint", last_checkpoint)?;
            d.set_item("queued_duration", *queued_duration)?;
        }
        QueryResult::Activity {
            current,
            state,
            next,
            params,
        } => {
            d.set_item("current", current)?;
            d.set_item("state", *state as u8)?;
            d.set_item("next", next)?;
            d.set_item("params", params)?;
        }
        QueryResult::LoopStats(s) => {
            d.set_item("target_hz", s.target_hz)?;
            d.set_item("loop_count", s.loop_count)?;
            d.set_item("overrun_count", s.overrun_count)?;
            d.set_item("mean_period_s", s.mean_period_s)?;
            d.set_item("std_period_s", s.std_period_s)?;
            d.set_item("min_period_s", s.min_period_s)?;
            d.set_item("max_period_s", s.max_period_s)?;
            d.set_item("p95_period_s", s.p95_period_s)?;
            d.set_item("p99_period_s", s.p99_period_s)?;
            d.set_item("mean_hz", s.mean_hz)?;
            d.set_item("p50_period_s", s.p50_period_s)?;
            d.set_item("p90_period_s", s.p90_period_s)?;
            d.set_item("can_frame_age_min_ticks", s.can_frame_age_min_ticks)?;
            d.set_item("can_frame_age_max_ticks", s.can_frame_age_max_ticks)?;
            d.set_item("rt_fifo", s.rt_fifo)?;
            d.set_item("rt_pinned", s.rt_pinned)?;
        }
        QueryResult::Reachable {
            joint_en,
            cart_en_wrf,
            cart_en_trf,
        } => {
            d.set_item("joint_en", int_list(joint_en))?;
            d.set_item("cart_en_wrf", int_list(cart_en_wrf))?;
            d.set_item("cart_en_trf", int_list(cart_en_trf))?;
        }
        QueryResult::Shapes {
            installation,
            program,
            epoch,
        } => {
            let inst = PyList::empty(py);
            for s in installation {
                inst.append(shape_dict(py, s)?)?;
            }
            let prog = PyList::empty(py);
            for s in program {
                prog.append(shape_dict(py, s)?)?;
            }
            d.set_item("installation", inst)?;
            d.set_item("program", prog)?;
            d.set_item("epoch", *epoch)?;
        }
        QueryResult::Payload { mass, com, inertia } => {
            fill_payload(&d, *mass, *com, *inertia)?;
        }
        QueryResult::BusScan { nodes } => {
            let rows = PyList::empty(py);
            for n in nodes {
                let row = PyDict::new(py);
                row.set_item("node", n.node)?;
                row.set_item("configured", n.configured)?;
                row.set_item("present", n.present)?;
                row.set_item("freshness", n.freshness)?;
                row.set_item("hw_ver", n.hw_ver)?;
                row.set_item("sw_ver", n.sw_ver)?;
                row.set_item("serial", n.serial)?;
                rows.append(row)?;
            }
            d.set_item("nodes", rows)?;
        }
        QueryResult::ConfigInfo {
            path,
            fingerprint,
            tick_dt_s,
            motion,
            joints,
        } => {
            d.set_item("path", path)?;
            d.set_item("fingerprint", fingerprint)?;
            d.set_item("tick_dt_s", *tick_dt_s)?;
            d.set_item("motion", motion_dict(py, motion)?)?;
            let js = PyList::empty(py);
            for j in joints {
                let jd = PyDict::new(py);
                jd.set_item("soft_min_rad", j[0])?;
                jd.set_item("soft_max_rad", j[1])?;
                jd.set_item("velocity_rad_s", j[2])?;
                jd.set_item("acceleration_rad_s2", j[3])?;
                js.append(jd)?;
            }
            d.set_item("joints", js)?;
        }
        QueryResult::ConfigBundle {
            path,
            fingerprint,
            robot_filename,
            robot_toml,
            grippers,
        } => {
            d.set_item("path", path)?;
            d.set_item("fingerprint", fingerprint)?;
            d.set_item("robot_filename", robot_filename)?;
            d.set_item("robot_toml", robot_toml)?;
            let gs = PyList::empty(py);
            for (name, content) in grippers {
                let gd = PyDict::new(py);
                gd.set_item("filename", name)?;
                gd.set_item("content", content)?;
                gs.append(gd)?;
            }
            d.set_item("grippers", gs)?;
        }
        other => {
            return Err(PyRuntimeError::new_err(format!(
                "query result {:?} has a dedicated accessor",
                other.tag()
            )));
        }
    }
    Ok(d.into_any().unbind())
}

/// Python shape dict → wire shape.
pub fn shape_from_py(d: &Bound<'_, PyDict>) -> PyResult<Shape> {
    pythonize::depythonize(d).map_err(|e| PyRuntimeError::new_err(format!("bad shape: {e}")))
}

/// Python value → tool-action parameter.
pub fn tool_param_from_py(v: &Bound<'_, PyAny>) -> PyResult<ToolParam> {
    pythonize::depythonize(v)
        .map_err(|_| PyRuntimeError::new_err("tool parameters must be bool, int, float, or str"))
}

pub fn layer_of(name: &str) -> PyResult<ShapeLayer> {
    match name {
        "installation" => Ok(ShapeLayer::Installation),
        "program" => Ok(ShapeLayer::Program),
        other => Err(PyRuntimeError::new_err(format!(
            "unknown shape layer '{other}' (installation, program)"
        ))),
    }
}

/// One command dict → wire command.
///
/// `type` names the variant in snake_case and the remaining keys are the
/// command's own fields, so this is `par6_proto::Command`'s derived
/// deserialization rather than a second description of every command
/// that has to be updated alongside the first. A field added to a
/// command reaches the binding with no edit here.
pub fn command_from_py(d: &Bound<'_, PyDict>) -> PyResult<Command> {
    pythonize::depythonize(d).map_err(|e| PyRuntimeError::new_err(format!("bad command: {e}")))
}

/// `"parked"` or `"force"` — the operator's vouching, no default.
/// `"parked"` or `"force"`, by the enum's own name lookup. A typo is a
/// ValueError here, before any datagram — the operator's vouching has no
/// default to fall back on.
pub fn flashing_assertion(py: Python<'_>, assertion: &str) -> PyResult<FlashingAssertion> {
    pythonize::depythonize(&pyo3::types::PyString::new(py, assertion).into_any()).map_err(|_| {
        pyo3::exceptions::PyValueError::new_err(format!(
            "flashing assertion must be 'parked' or 'force', got {assertion:?}"
        ))
    })
}

/// A payload as every reader hands it to Python: `mass`, `com`, `inertia`
/// (zeros = point mass / none).
pub(crate) fn fill_payload(
    d: &Bound<'_, PyDict>,
    mass: f64,
    com: [f64; 3],
    inertia: [f64; 6],
) -> PyResult<()> {
    d.set_item("mass", mass)?;
    d.set_item("com", com.to_vec())?;
    d.set_item("inertia", inertia.to_vec())
}

/// A joint vector from Python, refused with its name if it is the wrong
/// length.
pub(crate) fn joints(q: &[f64], what: &str) -> PyResult<[f64; par6_kin::NQ]> {
    q.try_into().map_err(|_| {
        PyRuntimeError::new_err(format!(
            "{what} needs {} joint values, got {}",
            par6_kin::NQ,
            q.len()
        ))
    })
}

/// The `[motion]` keys labelled from a wire/config array; an omitted
/// optional key (NaN on the wire) is `None`.
pub(crate) fn motion_dict<'py>(
    py: Python<'py>,
    values: &[f64; 13],
) -> PyResult<Bound<'py, PyDict>> {
    let m = PyDict::new(py);
    for (key, v) in par6_config::MotionConfig::KEYS.iter().zip(values) {
        if v.is_nan() {
            m.set_item(key, py.None())?;
        } else {
            m.set_item(key, *v)?;
        }
    }
    Ok(m)
}
