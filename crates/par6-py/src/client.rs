//! The async client binding: every method returns an asyncio awaitable
//! driving the `par6-client` core on the shared tokio runtime. The
//! Python shim (`par6.client.async_client`) keeps the public API and
//! waldoctl typing; this layer is transport only.

use std::net::Ipv4Addr;
use std::time::Duration;

use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3_async_runtimes::tokio::future_into_py;

use par6_client::{Ack, Client, ClientConfig, ClientError, StatusTransport};
use par6_proto::command as cmd;
use par6_proto::{Command, CompletionPolicy, Frame, NUM_JOINTS};

use crate::convert::{
    client_err, query_result_dict, shape_from_py, status_dict, tool_param_from_py, wire_error_tuple,
};

fn frame_of(v: u8) -> PyResult<Frame> {
    Frame::from_wire(i64::from(v))
        .ok_or_else(|| PyRuntimeError::new_err(format!("unknown frame {v}")))
}

fn sys_future<'py>(py: Python<'py>, client: Client, c: Command) -> PyResult<Bound<'py, PyAny>> {
    future_into_py(py, async move {
        match client.system(c).await {
            Ok(Ack::Confirmed) => Ok(1i32),
            Ok(Ack::Unconfirmed) => Ok(0),
            Err(e) => Err(client_err(e)),
        }
    })
}

fn queued_future<'py>(py: Python<'py>, client: Client, c: Command) -> PyResult<Bound<'py, PyAny>> {
    future_into_py(py, async move {
        match client.queued(c).await {
            Ok(Some(index)) => Ok(index as i64),
            Ok(None) => Ok(-1),
            Err(e) => Err(client_err(e)),
        }
    })
}

fn fire_future<'py>(py: Python<'py>, client: Client, c: Command) -> PyResult<Bound<'py, PyAny>> {
    future_into_py(py, async move {
        client.fire(c).await.map_err(client_err)?;
        Ok(1i32)
    })
}

/// Composite queries resolve to a dict, unreachable to `None`.
fn query_future<'py>(py: Python<'py>, client: Client, c: Command) -> PyResult<Bound<'py, PyAny>> {
    future_into_py(py, async move {
        match client.query(c).await {
            Ok(result) => Python::with_gil(|py| query_result_dict(py, &result).map(Some)),
            Err(ClientError::Unreachable) => Ok(None),
            Err(e) => Err(client_err(e)),
        }
    })
}

/// The transport half of `AsyncRobotClient`.
#[pyclass]
pub struct CoreClient {
    client: Client,
}

impl CoreClient {
    fn rt(&self) -> Client {
        self.client.clone()
    }
}

#[pymethods]
impl CoreClient {
    /// Connect and start the reply + status listeners.
    #[staticmethod]
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (host, port, timeout, retries, status_transport, status_port, mcast_group, mcast_iface, status_unicast_host, mtu))]
    fn connect<'py>(
        py: Python<'py>,
        host: String,
        port: u16,
        timeout: f64,
        retries: u32,
        status_transport: String,
        status_port: u16,
        mcast_group: String,
        mcast_iface: String,
        status_unicast_host: String,
        mtu: usize,
    ) -> PyResult<Bound<'py, PyAny>> {
        future_into_py(py, async move {
            let parse = |s: &str, what: &str| -> Result<Ipv4Addr, PyErr> {
                s.parse()
                    .map_err(|_| PyRuntimeError::new_err(format!("bad {what}: {s}")))
            };
            let status = if status_transport.eq_ignore_ascii_case("UNICAST") {
                StatusTransport::Unicast {
                    host: parse(&status_unicast_host, "status host")?,
                }
            } else {
                StatusTransport::Multicast {
                    group: parse(&mcast_group, "multicast group")?,
                    iface: parse(&mcast_iface, "multicast interface")?,
                }
            };
            let cfg = ClientConfig {
                host,
                port,
                timeout: Duration::from_secs_f64(timeout),
                retries,
                status,
                status_port,
                mtu,
            };
            let client = Client::connect(cfg).await.map_err(client_err)?;
            Ok(CoreClient { client })
        })
    }

    /// Stop the listeners, wake every waiter, and wait for the listener
    /// tasks to wind down — after this returns the runtime runs nothing
    /// of this client's, so interpreter exit cannot race a worker.
    fn close(&self, py: Python<'_>) {
        let client = &self.client;
        py.allow_threads(|| {
            pyo3_async_runtimes::tokio::get_runtime().block_on(client.close_joined());
        });
    }

    /// STATUS packets lost so far (header seq gaps).
    fn status_seq_gaps(&self) -> u64 {
        self.client.status_seq_gaps()
    }

    /// The latest STATUS frame as a dict, or `None` before the first one.
    fn latest_status(&self, py: Python<'_>) -> PyResult<Option<PyObject>> {
        match self.client.latest_status() {
            Some(s) => Ok(Some(status_dict(py, &s)?)),
            None => Ok(None),
        }
    }

    /// Await a STATUS frame whose seq differs from `last_seq` (pass -1
    /// for "any frame"), up to `timeout` seconds; `None` on timeout.
    fn status_after<'py>(
        &self,
        py: Python<'py>,
        last_seq: i64,
        timeout: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        future_into_py(py, async move {
            let hit = client
                .wait_status(
                    |s| last_seq < 0 || s.seq as i64 != last_seq,
                    Duration::from_secs_f64(timeout),
                )
                .await;
            if !hit {
                return Ok(None);
            }
            match client.latest_status() {
                Some(s) => Python::with_gil(|py| status_dict(py, &s).map(Some)),
                None => Ok(None),
            }
        })
    }

    /// Await completion of queued command `index`; True on success,
    /// False on timeout; raises `RobotWireError` on a failed command.
    fn wait_command<'py>(
        &self,
        py: Python<'py>,
        index: u64,
        timeout: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        future_into_py(py, async move {
            client
                .wait_command(index, Duration::from_secs_f64(timeout))
                .await
                .map_err(client_err)
        })
    }

    /// Settle verdict off queued command `index`'s completion (1 = object
    /// while closing, 2 = object while opening, 3 = target reached, no
    /// object); None for non-tool commands or ones not completed yet.
    fn command_verdict(&self, index: u64) -> Option<u8> {
        self.rt().command_verdict(index)
    }

    // ---------------------------------------------------------- queries

    fn ping<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        future_into_py(py, async move {
            match client.ping().await {
                Ok(hw) => Ok(Some(hw)),
                Err(ClientError::Unreachable) => Ok(None),
                Err(e) => Err(client_err(e)),
            }
        })
    }

    fn angles<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        future_into_py(py, async move {
            match client.angles().await {
                Ok(a) => Ok(Some(a.to_vec())),
                Err(ClientError::Unreachable) => Ok(None),
                Err(e) => Err(client_err(e)),
            }
        })
    }

    fn pose<'py>(&self, py: Python<'py>, frame: u8) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        let frame = frame_of(frame)?;
        future_into_py(py, async move {
            match client.pose(frame).await {
                Ok(p) => Ok(Some(p.to_vec())),
                Err(ClientError::Unreachable) => Ok(None),
                Err(e) => Err(client_err(e)),
            }
        })
    }

    fn io<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        future_into_py(py, async move {
            match client.io().await {
                Ok(v) => Ok(Some(v)),
                Err(ClientError::Unreachable) => Ok(None),
                Err(e) => Err(client_err(e)),
            }
        })
    }

    fn joint_speeds<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        future_into_py(py, async move {
            match client.joint_speeds().await {
                Ok(v) => Ok(Some(v.to_vec())),
                Err(ClientError::Unreachable) => Ok(None),
                Err(e) => Err(client_err(e)),
            }
        })
    }

    fn status_query<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        query_future(py, self.rt(), Command::Status)
    }

    fn tools<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        query_future(py, self.rt(), Command::Tools)
    }

    fn queue<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        query_future(py, self.rt(), Command::Queue)
    }

    fn activity<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        query_future(py, self.rt(), Command::Activity)
    }

    fn loop_stats<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        query_future(py, self.rt(), Command::LoopStats)
    }

    fn reachable<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        query_future(py, self.rt(), Command::Reachable)
    }

    fn shapes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        query_future(py, self.rt(), Command::Shapes)
    }

    fn config_info<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        query_future(py, self.rt(), Command::ConfigInfo)
    }

    fn config_bundle<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        query_future(py, self.rt(), Command::ConfigBundle)
    }

    fn payload<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        query_future(py, self.rt(), Command::Payload)
    }

    #[pyo3(signature = (mass, com, inertia=None))]
    fn set_payload<'py>(
        &self,
        py: Python<'py>,
        mass: f64,
        com: [f64; 3],
        inertia: Option<[f64; 6]>,
    ) -> PyResult<Bound<'py, PyAny>> {
        sys_future(
            py,
            self.rt(),
            Command::SetPayload(cmd::SetPayload { mass, com, inertia }),
        )
    }

    fn profile<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        future_into_py(py, async move {
            match client.profile().await {
                Ok(p) => Ok(Some(p)),
                Err(ClientError::Unreachable) => Ok(None),
                Err(e) => Err(client_err(e)),
            }
        })
    }

    fn error<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        future_into_py(py, async move {
            match client.error().await {
                Ok(Some(e)) => Python::with_gil(|py| Ok(Some(wire_error_tuple(py, &e)))),
                Ok(None) => Ok(None),
                Err(ClientError::Unreachable) => Ok(None),
                Err(e) => Err(client_err(e)),
            }
        })
    }

    fn tcp_speed<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        future_into_py(py, async move {
            match client.tcp_speed().await {
                Ok(v) => Ok(Some(v)),
                Err(ClientError::Unreachable) => Ok(None),
                Err(e) => Err(client_err(e)),
            }
        })
    }

    fn tcp_offset<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        future_into_py(py, async move {
            match client.tcp_offset().await {
                Ok(v) => Ok(Some(v.to_vec())),
                Err(ClientError::Unreachable) => Ok(None),
                Err(e) => Err(client_err(e)),
            }
        })
    }

    fn tool_status<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        future_into_py(py, async move {
            match client.tool_status().await {
                Ok(Some(t)) => {
                    Python::with_gil(|py| crate::convert::tool_status_dict(py, &t).map(Some))
                }
                Ok(None) => Ok(None),
                Err(ClientError::Unreachable) => Ok(None),
                Err(e) => Err(client_err(e)),
            }
        })
    }

    fn is_simulator<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        future_into_py(py, async move {
            match client.is_simulator().await {
                Ok(v) => Ok(Some(v)),
                Err(ClientError::Unreachable) => Ok(None),
                Err(e) => Err(client_err(e)),
            }
        })
    }

    // --------------------------------------------------------- system

    fn reset<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        sys_future(py, self.rt(), Command::Reset)
    }

    fn estop<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        sys_future(py, self.rt(), Command::Estop)
    }

    fn set_gravity_comp<'py>(&self, py: Python<'py>, on: bool) -> PyResult<Bound<'py, PyAny>> {
        sys_future(
            py,
            self.rt(),
            Command::SetGravityComp(cmd::SetGravityComp { on }),
        )
    }

    fn pause<'py>(&self, py: Python<'py>, on: bool) -> PyResult<Bound<'py, PyAny>> {
        sys_future(py, self.rt(), Command::Pause(cmd::Pause { on }))
    }

    fn stop<'py>(&self, py: Python<'py>, clear_queue: bool) -> PyResult<Bound<'py, PyAny>> {
        sys_future(py, self.rt(), Command::Stop(cmd::Stop { clear_queue }))
    }

    fn write_io<'py>(&self, py: Python<'py>, port: u8, value: u8) -> PyResult<Bound<'py, PyAny>> {
        sys_future(
            py,
            self.rt(),
            Command::WriteIo(cmd::WriteIo { port, value }),
        )
    }

    fn simulator<'py>(&self, py: Python<'py>, on: bool) -> PyResult<Bound<'py, PyAny>> {
        sys_future(py, self.rt(), Command::Simulator(cmd::Simulator { on }))
    }

    fn select_profile<'py>(&self, py: Python<'py>, profile: String) -> PyResult<Bound<'py, PyAny>> {
        sys_future(
            py,
            self.rt(),
            Command::SelectProfile(cmd::SelectProfile { profile }),
        )
    }

    fn reset_state<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        sys_future(py, self.rt(), Command::ResetState)
    }

    fn connect_hardware<'py>(&self, py: Python<'py>, port: String) -> PyResult<Bound<'py, PyAny>> {
        sys_future(
            py,
            self.rt(),
            Command::ConnectHardware(cmd::ConnectHardware { port }),
        )
    }

    /// `assertion` is "parked" or "force" (the wire refuses anything else).
    fn enter_flashing<'py>(&self, py: Python<'py>, assertion: &str) -> PyResult<Bound<'py, PyAny>> {
        let assertion = match assertion.to_ascii_lowercase().as_str() {
            "parked" => par6_proto::FlashingAssertion::Parked,
            "force" => par6_proto::FlashingAssertion::Force,
            other => {
                return Err(pyo3::exceptions::PyValueError::new_err(format!(
                    "flashing assertion must be 'parked' or 'force', got {other:?}"
                )))
            }
        };
        sys_future(
            py,
            self.rt(),
            Command::EnterFlashing(cmd::EnterFlashing { assertion }),
        )
    }

    fn exit_flashing<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        sys_future(py, self.rt(), Command::ExitFlashing)
    }

    #[allow(clippy::too_many_arguments)]
    fn set_pid_gains<'py>(
        &self,
        py: Python<'py>,
        node: u8,
        kpp: f64,
        kpv: f64,
        kiv: f64,
        kpiq: f64,
        kiiq: f64,
        kp: f64,
        kd: f64,
        ilim_ma: f64,
        velocity_limit_ticks_s: f64,
        voltage_limit_mv: u32,
    ) -> PyResult<Bound<'py, PyAny>> {
        sys_future(
            py,
            self.rt(),
            Command::SetPidGains(cmd::SetPidGains {
                node,
                kpp,
                kpv,
                kiv,
                kpiq,
                kiiq,
                kp,
                kd,
                ilim_ma,
                velocity_limit_ticks_s,
                voltage_limit_mv,
            }),
        )
    }

    fn set_tcp_offset<'py>(
        &self,
        py: Python<'py>,
        x: f64,
        y: f64,
        z: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        queued_future(
            py,
            client.clone(),
            Command::SetTcpOffset(cmd::SetTcpOffset {
                key: client.fresh_key(),
                x,
                y,
                z,
            }),
        )
    }

    fn set_shapes<'py>(
        &self,
        py: Python<'py>,
        shapes: Vec<Bound<'py, pyo3::types::PyDict>>,
    ) -> PyResult<Bound<'py, PyAny>> {
        let shapes = shapes
            .iter()
            .map(shape_from_py)
            .collect::<PyResult<Vec<_>>>()?;
        sys_future(py, self.rt(), Command::SetShapes(cmd::SetShapes { shapes }))
    }

    fn set_completion_policy<'py>(
        &self,
        py: Python<'py>,
        policy: u8,
    ) -> PyResult<Bound<'py, PyAny>> {
        let policy = CompletionPolicy::from_wire(i64::from(policy)).ok_or_else(|| {
            PyRuntimeError::new_err(format!("unknown completion policy {policy}"))
        })?;
        sys_future(
            py,
            self.rt(),
            Command::SetCompletionPolicy(cmd::SetCompletionPolicy { policy }),
        )
    }

    fn set_recipe<'py>(&self, py: Python<'py>, name: String) -> PyResult<Bound<'py, PyAny>> {
        sys_future(py, self.rt(), Command::SetRecipe(cmd::SetRecipe { name }))
    }

    // --------------------------------------------------------- queued

    #[pyo3(signature = (calibrate=false))]
    fn home<'py>(&self, py: Python<'py>, calibrate: bool) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        future_into_py(py, async move {
            match client.home(calibrate).await {
                Ok(Some(index)) => Ok(index as i64),
                Ok(None) => Ok(-1),
                Err(e) => Err(client_err(e)),
            }
        })
    }

    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (angles, duration, speed, accel, blend_radius, rel))]
    fn move_j<'py>(
        &self,
        py: Python<'py>,
        angles: [f64; NUM_JOINTS],
        duration: Option<f64>,
        speed: Option<f64>,
        accel: Option<f64>,
        blend_radius: Option<f64>,
        rel: bool,
    ) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        queued_future(
            py,
            client.clone(),
            Command::MoveJ(cmd::MoveJ {
                key: client.fresh_key(),
                angles,
                duration,
                speed,
                accel,
                blend_radius,
                rel,
            }),
        )
    }

    #[pyo3(signature = (pose, duration, speed, accel, blend_radius))]
    fn move_j_pose<'py>(
        &self,
        py: Python<'py>,
        pose: [f64; 6],
        duration: Option<f64>,
        speed: Option<f64>,
        accel: Option<f64>,
        blend_radius: Option<f64>,
    ) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        queued_future(
            py,
            client.clone(),
            Command::MoveJPose(cmd::MoveJPose {
                key: client.fresh_key(),
                pose,
                duration,
                speed,
                accel,
                blend_radius,
            }),
        )
    }

    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (pose, frame, duration, speed, accel, blend_radius, rel))]
    fn move_l<'py>(
        &self,
        py: Python<'py>,
        pose: [f64; 6],
        frame: u8,
        duration: Option<f64>,
        speed: Option<f64>,
        accel: Option<f64>,
        blend_radius: Option<f64>,
        rel: bool,
    ) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        queued_future(
            py,
            client.clone(),
            Command::MoveL(cmd::MoveL {
                key: client.fresh_key(),
                pose,
                frame: frame_of(frame)?,
                duration,
                speed,
                accel,
                blend_radius,
                rel,
            }),
        )
    }

    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (via, end, frame, duration, speed, accel, blend_radius, rel=false))]
    fn move_c<'py>(
        &self,
        py: Python<'py>,
        via: [f64; 6],
        end: [f64; 6],
        frame: u8,
        duration: Option<f64>,
        speed: Option<f64>,
        accel: Option<f64>,
        blend_radius: Option<f64>,
        rel: bool,
    ) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        queued_future(
            py,
            client.clone(),
            Command::MoveC(cmd::MoveC {
                key: client.fresh_key(),
                via,
                end,
                frame: frame_of(frame)?,
                duration,
                speed,
                accel,
                blend_radius,
                rel,
            }),
        )
    }

    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (waypoints, frame, duration, speed, accel, rel=false))]
    fn move_s<'py>(
        &self,
        py: Python<'py>,
        waypoints: Vec<[f64; 6]>,
        frame: u8,
        duration: Option<f64>,
        speed: Option<f64>,
        accel: Option<f64>,
        rel: bool,
    ) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        queued_future(
            py,
            client.clone(),
            Command::MoveS(cmd::MoveS {
                key: client.fresh_key(),
                waypoints,
                frame: frame_of(frame)?,
                duration,
                speed,
                accel,
                rel,
            }),
        )
    }

    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (waypoints, frame, duration, speed, accel, rel=false))]
    fn move_p<'py>(
        &self,
        py: Python<'py>,
        waypoints: Vec<[f64; 6]>,
        frame: u8,
        duration: Option<f64>,
        speed: Option<f64>,
        accel: Option<f64>,
        rel: bool,
    ) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        queued_future(
            py,
            client.clone(),
            Command::MoveP(cmd::MoveP {
                key: client.fresh_key(),
                waypoints,
                frame: frame_of(frame)?,
                duration,
                speed,
                accel,
                rel,
            }),
        )
    }

    #[pyo3(signature = (tool_name, variant_key))]
    fn select_tool<'py>(
        &self,
        py: Python<'py>,
        tool_name: String,
        variant_key: Option<String>,
    ) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        queued_future(
            py,
            client.clone(),
            Command::SelectTool(cmd::SelectTool {
                key: client.fresh_key(),
                tool_name,
                variant_key,
            }),
        )
    }

    fn delay<'py>(&self, py: Python<'py>, seconds: f64) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        queued_future(
            py,
            client.clone(),
            Command::Delay(cmd::Delay {
                key: client.fresh_key(),
                seconds,
            }),
        )
    }

    fn checkpoint<'py>(&self, py: Python<'py>, label: String) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        queued_future(
            py,
            client.clone(),
            Command::Checkpoint(cmd::Checkpoint {
                key: client.fresh_key(),
                label,
            }),
        )
    }

    fn tool_action<'py>(
        &self,
        py: Python<'py>,
        tool_key: String,
        action: String,
        params: Vec<Bound<'py, PyAny>>,
    ) -> PyResult<Bound<'py, PyAny>> {
        let params = params
            .iter()
            .map(tool_param_from_py)
            .collect::<PyResult<Vec<_>>>()?;
        let client = self.rt();
        queued_future(
            py,
            client.clone(),
            Command::ToolAction(cmd::ToolAction {
                key: client.fresh_key(),
                tool_key,
                action,
                params,
            }),
        )
    }

    // -------------------------------------------------- fire-and-forget

    #[pyo3(signature = (angles, speed, accel))]
    fn servo_j<'py>(
        &self,
        py: Python<'py>,
        angles: [f64; NUM_JOINTS],
        speed: Option<f64>,
        accel: Option<f64>,
    ) -> PyResult<Bound<'py, PyAny>> {
        fire_future(
            py,
            self.rt(),
            Command::ServoJ(cmd::ServoJ {
                angles,
                speed,
                accel,
            }),
        )
    }

    #[pyo3(signature = (pose, speed, accel))]
    fn servo_j_pose<'py>(
        &self,
        py: Python<'py>,
        pose: [f64; 6],
        speed: Option<f64>,
        accel: Option<f64>,
    ) -> PyResult<Bound<'py, PyAny>> {
        fire_future(
            py,
            self.rt(),
            Command::ServoJPose(cmd::ServoJPose { pose, speed, accel }),
        )
    }

    #[pyo3(signature = (pose, speed, accel))]
    fn servo_l<'py>(
        &self,
        py: Python<'py>,
        pose: [f64; 6],
        speed: Option<f64>,
        accel: Option<f64>,
    ) -> PyResult<Bound<'py, PyAny>> {
        fire_future(
            py,
            self.rt(),
            Command::ServoL(cmd::ServoL { pose, speed, accel }),
        )
    }

    #[pyo3(signature = (speeds, duration, accel))]
    fn jog_j<'py>(
        &self,
        py: Python<'py>,
        speeds: [f64; NUM_JOINTS],
        duration: f64,
        accel: Option<f64>,
    ) -> PyResult<Bound<'py, PyAny>> {
        fire_future(
            py,
            self.rt(),
            Command::JogJ(cmd::JogJ {
                speeds,
                duration,
                accel,
            }),
        )
    }

    #[pyo3(signature = (velocities, duration, frame, accel))]
    fn jog_l<'py>(
        &self,
        py: Python<'py>,
        velocities: [f64; 6],
        duration: f64,
        frame: u8,
        accel: Option<f64>,
    ) -> PyResult<Bound<'py, PyAny>> {
        fire_future(
            py,
            self.rt(),
            Command::JogL(cmd::JogL {
                velocities,
                duration,
                frame: frame_of(frame)?,
                accel,
            }),
        )
    }

    #[pyo3(signature = (angles, tool_positions))]
    fn teleport<'py>(
        &self,
        py: Python<'py>,
        angles: [f64; NUM_JOINTS],
        tool_positions: Option<Vec<f64>>,
    ) -> PyResult<Bound<'py, PyAny>> {
        fire_future(
            py,
            self.rt(),
            Command::Teleport(cmd::Teleport {
                angles,
                tool_positions,
            }),
        )
    }

    fn reset_loop_stats<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        fire_future(py, self.rt(), Command::ResetLoopStats)
    }
}
