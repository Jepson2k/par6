//! Wire ⇄ Python conversions: STATUS frames and query results become
//! plain dicts (the Python shim owns the typed surface), errors become a
//! structured exception carrying the wire's six-tuple.

use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};

use par6_client::ClientError;
use par6_proto::command::{self as cmd, ToolParam};
use par6_proto::{
    Command, CompletionPolicy, FlashingAssertion, Frame, QueryResult, Shape, Status,
    ToolStatusWire, WireError,
};
use par6_server::ShapeLayer;

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
    Ok(d.into_any().unbind())
}

fn shape_dict(py: Python<'_>, s: &Shape) -> PyResult<PyObject> {
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
            d.set_item("mass", *mass)?;
            d.set_item("com", com.to_vec())?;
            d.set_item("inertia", inertia.to_vec())?;
        }
        QueryResult::ConfigInfo {
            path,
            fingerprint,
            tick_dt_s,
            motion,
            joints,
            active_recipe,
            recipes,
        } => {
            d.set_item("path", path)?;
            d.set_item("fingerprint", fingerprint)?;
            d.set_item("tick_dt_s", *tick_dt_s)?;
            let m = PyDict::new(py);
            for (key, v) in [
                "jog_l_linear_max_m_s",
                "jog_l_angular_max_rad_s",
                "cart_step_m",
                "cart_step_rad",
                "move_l_max_joint_step_rad",
                "dls_lambda",
                "settle_tolerance_rad",
                "settle_timeout_s",
            ]
            .iter()
            .zip(motion)
            {
                m.set_item(key, *v)?;
            }
            d.set_item("motion", m)?;
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
            d.set_item("active_recipe", active_recipe.as_deref())?;
            d.set_item("recipes", recipes.to_vec())?;
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
    let get = |k: &str| -> PyResult<Bound<'_, PyAny>> {
        d.get_item(k)?
            .ok_or_else(|| PyRuntimeError::new_err(format!("shape is missing '{k}'")))
    };
    Ok(Shape {
        kind: get("kind")?.extract()?,
        params: get("params")?.extract()?,
        pose: get("pose")?.extract()?,
        collision: match d.get_item("collision")? {
            Some(v) => v.extract()?,
            None => true,
        },
        margin: match d.get_item("margin")? {
            Some(v) => v.extract()?,
            None => None,
        },
        name: get("name")?.extract()?,
    })
}

/// Python value → tool-action parameter.
pub fn tool_param_from_py(v: &Bound<'_, PyAny>) -> PyResult<ToolParam> {
    if let Ok(b) = v.downcast::<pyo3::types::PyBool>() {
        return Ok(ToolParam::Bool(b.is_true()));
    }
    if let Ok(i) = v.extract::<i64>() {
        return Ok(ToolParam::Int(i));
    }
    if let Ok(f) = v.extract::<f64>() {
        return Ok(ToolParam::Float(f));
    }
    if let Ok(s) = v.extract::<String>() {
        return Ok(ToolParam::Str(s));
    }
    Err(PyRuntimeError::new_err(
        "tool parameters must be bool, int, float, or str",
    ))
}

pub fn frame_of(v: u8) -> PyResult<Frame> {
    Frame::from_wire(i64::from(v))
        .ok_or_else(|| PyRuntimeError::new_err(format!("unknown frame {v}")))
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

fn frame_key(d: &Bound<'_, PyDict>) -> PyResult<Frame> {
    frame_of(opt(d, "frame")?.unwrap_or(0))
}

/// One command dict → wire command. `type` is the wire name of the
/// family; the other keys mirror the wire fields (wire units: mm, deg,
/// fractions). Every family a client can send is accepted, so the
/// preview sees the same stream the runtime would.
pub fn command_from_py(d: &Bound<'_, PyDict>) -> PyResult<Command> {
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
            frame: frame_key(d)?,
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
            frame: frame_key(d)?,
            duration: opt(d, "duration")?,
            speed: opt(d, "speed")?,
            accel: opt(d, "accel")?,
            blend_radius: opt(d, "blend_radius")?,
            rel: opt(d, "rel")?.unwrap_or(false),
        }),
        "move_s" => Command::MoveS(cmd::MoveS {
            key: 0,
            waypoints: get(d, "waypoints")?,
            frame: frame_key(d)?,
            duration: opt(d, "duration")?,
            speed: opt(d, "speed")?,
            accel: opt(d, "accel")?,
            rel: opt(d, "rel")?.unwrap_or(false),
        }),
        "move_p" => Command::MoveP(cmd::MoveP {
            key: 0,
            waypoints: get(d, "waypoints")?,
            frame: frame_key(d)?,
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
            let params: Vec<Bound<'_, PyAny>> = opt(d, "params")?.unwrap_or_default();
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
        "servo_j" => Command::ServoJ(cmd::ServoJ {
            angles: get(d, "angles")?,
            speed: opt(d, "speed")?,
            accel: opt(d, "accel")?,
        }),
        "servo_j_pose" => Command::ServoJPose(cmd::ServoJPose {
            pose: get(d, "pose")?,
            speed: opt(d, "speed")?,
            accel: opt(d, "accel")?,
        }),
        "servo_l" => Command::ServoL(cmd::ServoL {
            pose: get(d, "pose")?,
            speed: opt(d, "speed")?,
            accel: opt(d, "accel")?,
        }),
        "jog_j" => Command::JogJ(cmd::JogJ {
            speeds: get(d, "speeds")?,
            duration: get(d, "duration")?,
            accel: opt(d, "accel")?,
        }),
        "jog_l" => Command::JogL(cmd::JogL {
            velocities: get(d, "velocities")?,
            duration: get(d, "duration")?,
            frame: frame_key(d)?,
            accel: opt(d, "accel")?,
        }),
        "teleport" => Command::Teleport(cmd::Teleport {
            angles: get(d, "angles")?,
            tool_positions: opt(d, "tool_positions")?,
        }),
        "stop" => Command::Stop(cmd::Stop {
            clear_queue: opt(d, "clear_queue")?.unwrap_or(true),
        }),
        "estop" => Command::Estop,
        "reset" => Command::Reset,
        "reset_state" => Command::ResetState,
        "pause" => Command::Pause(cmd::Pause { on: get(d, "on")? }),
        "set_gravity_comp" => Command::SetGravityComp(cmd::SetGravityComp { on: get(d, "on")? }),
        "write_io" => Command::WriteIo(cmd::WriteIo {
            port: get(d, "port")?,
            value: get(d, "value")?,
        }),
        "simulator" => Command::Simulator(cmd::Simulator { on: get(d, "on")? }),
        "connect_hardware" => Command::ConnectHardware(cmd::ConnectHardware {
            port: get(d, "port")?,
        }),
        "select_profile" => Command::SelectProfile(cmd::SelectProfile {
            profile: get(d, "profile")?,
        }),
        "set_tcp_offset" => Command::SetTcpOffset(cmd::SetTcpOffset {
            x: get(d, "x")?,
            y: get(d, "y")?,
            z: get(d, "z")?,
        }),
        "set_payload" => Command::SetPayload(cmd::SetPayload {
            mass: get(d, "mass")?,
            com: get(d, "com")?,
            inertia: opt(d, "inertia")?,
        }),
        "set_shapes" => {
            let shapes: Vec<Bound<'_, PyDict>> = get(d, "shapes")?;
            Command::SetShapes(cmd::SetShapes {
                shapes: shapes
                    .iter()
                    .map(shape_from_py)
                    .collect::<PyResult<Vec<_>>>()?,
            })
        }
        "set_pid_gains" => Command::SetPidGains(cmd::SetPidGains {
            node: get(d, "node")?,
            kpp: get(d, "kpp")?,
            kpv: get(d, "kpv")?,
            kiv: get(d, "kiv")?,
            kpiq: get(d, "kpiq")?,
            kiiq: get(d, "kiiq")?,
            kp: get(d, "kp")?,
            kd: get(d, "kd")?,
            ilim_ma: get(d, "ilim_ma")?,
            velocity_limit_ticks_s: get(d, "velocity_limit_ticks_s")?,
            voltage_limit_mv: get(d, "voltage_limit_mv")?,
        }),
        "set_completion_policy" => {
            let raw: u8 = get(d, "policy")?;
            let policy = CompletionPolicy::from_wire(i64::from(raw)).ok_or_else(|| {
                PyRuntimeError::new_err(format!("unknown completion policy {raw}"))
            })?;
            Command::SetCompletionPolicy(cmd::SetCompletionPolicy { policy })
        }
        "set_recipe" => Command::SetRecipe(cmd::SetRecipe {
            name: get(d, "name")?,
        }),
        "enter_flashing" => Command::EnterFlashing(cmd::EnterFlashing {
            assertion: flashing_assertion(&get::<String>(d, "assertion")?)?,
        }),
        "exit_flashing" => Command::ExitFlashing,
        other => {
            return Err(PyRuntimeError::new_err(format!(
                "unknown command type '{other}'"
            )))
        }
    };
    Ok(c)
}

/// `"parked"` or `"force"` — the operator's vouching, no default.
pub fn flashing_assertion(assertion: &str) -> PyResult<FlashingAssertion> {
    match assertion.to_ascii_lowercase().as_str() {
        "parked" => Ok(FlashingAssertion::Parked),
        "force" => Ok(FlashingAssertion::Force),
        other => Err(pyo3::exceptions::PyValueError::new_err(format!(
            "flashing assertion must be 'parked' or 'force', got {other:?}"
        ))),
    }
}
