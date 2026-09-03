//! The async client binding: every method returns an asyncio awaitable
//! driving the `par6-client` core on the shared tokio runtime. The
//! Python shim (`par6.client.async_client`) keeps the public API and
//! waldoctl typing; this layer is transport only.

use std::net::Ipv4Addr;
use std::sync::Arc;
use std::time::Duration;

use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use pyo3_async_runtimes::tokio::future_into_py;

use par6_client::{Ack, Client, ClientConfig, ClientError, MotionWait, StatusTransport};
use par6_proto::command as cmd;
use par6_proto::{Command, CompletionPolicy, NUM_JOINTS};

use crate::convert::{
    client_err, flashing_assertion, frame_of, query_result_dict, shape_from_py, status_dict,
    tool_param_from_py, wire_error_tuple,
};

/// The model a payload identification measures against: the arm with no
/// tool payload, its collision world, and the joint window, all from the
/// same config the daemon runs.
fn sys_future<'py>(py: Python<'py>, client: Client, c: Command) -> PyResult<Bound<'py, PyAny>> {
    future_into_py(py, async move {
        match client.system(c).await {
            Ok(Ack::Confirmed) => Ok(1i32),
            Ok(Ack::Unconfirmed) => Ok(0),
            Err(e) => Err(client_err(e)),
        }
    })
}

/// A queued command's index, -1 when it went unconfirmed.
fn index_future<'py, F>(py: Python<'py>, fut: F) -> PyResult<Bound<'py, PyAny>>
where
    F: std::future::Future<Output = Result<Option<u64>, ClientError>> + Send + 'static,
{
    future_into_py(py, async move {
        match fut.await {
            Ok(Some(index)) => Ok(index as i64),
            Ok(None) => Ok(-1),
            Err(e) => Err(client_err(e)),
        }
    })
}

fn queued_future<'py>(py: Python<'py>, client: Client, c: Command) -> PyResult<Bound<'py, PyAny>> {
    index_future(py, async move { client.queued(c).await })
}

/// A query's answer, `None` when the runtime is unreachable — the one
/// error a caller polls through rather than raises on.
fn opt_future<'py, T, F>(py: Python<'py>, fut: F) -> PyResult<Bound<'py, PyAny>>
where
    T: for<'a> pyo3::IntoPyObject<'a> + Send + 'static,
    F: std::future::Future<Output = Result<T, ClientError>> + Send + 'static,
{
    future_into_py(py, async move {
        match fut.await {
            Ok(v) => Ok(Some(v)),
            Err(ClientError::Unreachable) => Ok(None),
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
    /// The estimation model, built once per config it was asked for and
    /// kept: the mesh worlds cost hundreds of milliseconds, and a program
    /// that estimates after every pick asks for the same one every time.
    estimation: Arc<tokio::sync::Mutex<Option<(String, par6_calibrate::EstimationModel)>>>,
}

impl CoreClient {
    fn rt(&self) -> Client {
        self.client.clone()
    }
}

#[pymethods]
impl CoreClient {
    /// Connect and start the reply + status listeners. Every `None`
    /// falls back to the client's own default ladder (`PAR6_*`
    /// environment, then the shipped defaults).
    #[staticmethod]
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (host=None, port=None, timeout=None, retries=None, status_transport=None, status_port=None, mcast_group=None, mcast_iface=None, status_unicast_host=None, mtu=None))]
    fn connect<'py>(
        py: Python<'py>,
        host: Option<String>,
        port: Option<u16>,
        timeout: Option<f64>,
        retries: Option<u32>,
        status_transport: Option<String>,
        status_port: Option<u16>,
        mcast_group: Option<String>,
        mcast_iface: Option<String>,
        status_unicast_host: Option<String>,
        mtu: Option<usize>,
    ) -> PyResult<Bound<'py, PyAny>> {
        future_into_py(py, async move {
            let parse = |s: &str, what: &str| -> Result<Ipv4Addr, PyErr> {
                s.parse()
                    .map_err(|_| PyRuntimeError::new_err(format!("bad {what}: {s}")))
            };
            let mut cfg = ClientConfig::default();
            if let Some(h) = host {
                cfg.host = h;
            }
            if let Some(p) = port {
                cfg.port = p;
            }
            if let Some(t) = timeout {
                cfg.timeout = Duration::from_secs_f64(t);
            }
            if let Some(r) = retries {
                cfg.retries = r;
            }
            if let Some(p) = status_port {
                cfg.status_port = p;
            }
            if let Some(m) = mtu {
                cfg.mtu = m;
            }
            let kind = status_transport.map(|k| k.to_ascii_uppercase());
            let unicast = match (&kind, &cfg.status) {
                (Some(k), _) => k == "UNICAST",
                (None, StatusTransport::Unicast { .. }) => true,
                (None, StatusTransport::Multicast { .. }) => false,
            };
            cfg.status = if unicast {
                let host = match (&status_unicast_host, &cfg.status) {
                    (Some(h), _) => parse(h, "status host")?,
                    (None, StatusTransport::Unicast { host }) => *host,
                    (None, _) => Ipv4Addr::LOCALHOST,
                };
                StatusTransport::Unicast { host }
            } else {
                let (mut group, mut iface) = match &cfg.status {
                    StatusTransport::Multicast { group, iface } => (*group, *iface),
                    _ => (Ipv4Addr::new(239, 255, 0, 71), Ipv4Addr::LOCALHOST),
                };
                if let Some(g) = &mcast_group {
                    group = parse(g, "multicast group")?;
                }
                if let Some(i) = &mcast_iface {
                    iface = parse(i, "multicast interface")?;
                }
                StatusTransport::Multicast { group, iface }
            };
            let client = Client::connect(cfg).await.map_err(client_err)?;
            Ok(CoreClient {
                client,
                estimation: Arc::default(),
            })
        })
    }

    /// The command endpoint this client talks to.
    fn endpoint(&self) -> (String, u16) {
        let cfg = self.client.config();
        (cfg.host.clone(), cfg.port)
    }

    /// The status transport in use: `(kind, port)` with kind `"UNICAST"`
    /// or `"MULTICAST"`.
    fn status_endpoint(&self) -> (String, u16) {
        let cfg = self.client.config();
        let kind = match cfg.status {
            StatusTransport::Unicast { .. } => "UNICAST",
            StatusTransport::Multicast { .. } => "MULTICAST",
        };
        (kind.to_owned(), cfg.status_port)
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

    /// Await the checkpoint `label`; False on timeout.
    fn wait_checkpoint<'py>(
        &self,
        py: Python<'py>,
        label: String,
        timeout: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        future_into_py(py, async move {
            Ok(client
                .wait_checkpoint(&label, Duration::from_secs_f64(timeout))
                .await)
        })
    }

    /// Start-then-settle wait on the status stream (see
    /// `par6_client::Client::wait_motion`); False on timeout.
    #[pyo3(signature = (timeout, settle_window, speed_threshold, angle_threshold, motion_start_timeout))]
    fn wait_motion<'py>(
        &self,
        py: Python<'py>,
        timeout: f64,
        settle_window: f64,
        speed_threshold: f64,
        angle_threshold: f64,
        motion_start_timeout: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        let wait = MotionWait {
            timeout: Duration::from_secs_f64(timeout),
            settle_window: Duration::from_secs_f64(settle_window),
            speed_threshold,
            angle_threshold,
            motion_start_timeout: Duration::from_secs_f64(motion_start_timeout),
        };
        future_into_py(py, async move { Ok(client.wait_motion(wait).await) })
    }

    /// Whether the e-stop line reads pressed; False when unreachable.
    fn is_estop_pressed<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        future_into_py(py, async move {
            match client.is_estop_pressed().await {
                Ok(v) => Ok(v),
                Err(ClientError::Unreachable) => Ok(false),
                Err(e) => Err(client_err(e)),
            }
        })
    }

    /// Whether every joint is below `threshold` rad/s; False when
    /// unreachable.
    fn is_robot_stopped<'py>(
        &self,
        py: Python<'py>,
        threshold: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        future_into_py(py, async move {
            match client.is_robot_stopped(threshold).await {
                Ok(v) => Ok(v),
                Err(ClientError::Unreachable) => Ok(false),
                Err(e) => Err(client_err(e)),
            }
        })
    }

    /// Whether the arm is back-driveable right now, off the latest
    /// broadcast (waiting up to `timeout` for the first one).
    fn is_freedrive<'py>(&self, py: Python<'py>, timeout: f64) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        future_into_py(py, async move {
            Ok(client.is_freedrive(Duration::from_secs_f64(timeout)).await)
        })
    }

    // ---------------------------------------------------------- queries

    fn ping<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        opt_future(py, async move { client.ping().await })
    }

    fn angles<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        opt_future(py, async move { client.angles().await.map(|a| a.to_vec()) })
    }

    fn pose<'py>(&self, py: Python<'py>, frame: u8) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        let frame = frame_of(frame)?;
        opt_future(
            py,
            async move { client.pose(frame).await.map(|p| p.to_vec()) },
        )
    }

    /// The TCP pose as `[x, y, z, rx, ry, rz]` in mm and degrees
    /// (intrinsic-XYZ rpy, the wire convention); None when unreachable.
    fn pose_xyzrpy<'py>(&self, py: Python<'py>, frame: u8) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        let frame = frame_of(frame)?;
        future_into_py(py, async move {
            match client.pose(frame).await {
                Ok(m) => {
                    let p = par6d::matrix_to_xyzrpy(&m);
                    Ok(Some(vec![
                        p[0],
                        p[1],
                        p[2],
                        p[3].to_degrees(),
                        p[4].to_degrees(),
                        p[5].to_degrees(),
                    ]))
                }
                Err(ClientError::Unreachable) => Ok(None),
                Err(e) => Err(client_err(e)),
            }
        })
    }

    fn io<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        opt_future(py, async move { client.io().await })
    }

    fn joint_speeds<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        opt_future(py, async move {
            client.joint_speeds().await.map(|v| v.to_vec())
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

    /// Estimate what the arm is carrying — mass and centre of mass, never
    /// the inertia tensor, which static poses cannot excite — and,
    /// optionally, tell the runtime. The whole protocol, including the
    /// clearing and restoring of whatever was declared, is
    /// `par6_calibrate::estimate`'s; this only carries paths and results.
    #[pyo3(signature = (config=None, assets=None, package_dir=None, spread=0.5, ridge=0.01, declare=false))]
    #[allow(clippy::too_many_arguments)]
    fn estimate_payload<'py>(
        &self,
        py: Python<'py>,
        config: Option<String>,
        assets: Option<String>,
        package_dir: Option<String>,
        spread: f64,
        ridge: f64,
        declare: bool,
    ) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        let cache = Arc::clone(&self.estimation);
        let key = format!("{config:?}|{assets:?}|{package_dir:?}");
        future_into_py(py, async move {
            let mut slot = cache.lock().await;
            if !matches!(&*slot, Some((k, _)) if *k == key) {
                // Off the async thread: this parses the URDF and builds the
                // collision meshes, which would otherwise hold the Python
                // event loop for as long as it takes.
                let model = tokio::task::spawn_blocking(move || {
                    par6d::kin::estimation_model(
                        config.as_deref().map(std::path::Path::new),
                        assets.as_deref().map(std::path::Path::new),
                        package_dir.as_deref().map(std::path::Path::new),
                    )
                })
                .await
                .map_err(|e| PyRuntimeError::new_err(e.to_string()))?
                .map_err(PyRuntimeError::new_err)?;
                *slot = Some((key, model));
            }
            let (_, model) = slot.as_mut().expect("just filled");
            let report = par6_calibrate::estimate(&client, model, spread, ridge, declare)
                .await
                .map_err(PyRuntimeError::new_err)?;
            Python::with_gil(|py| {
                let d = PyDict::new(py);
                d.set_item("mass", report.fit.mass)?;
                d.set_item("com", report.fit.com.to_vec())?;
                d.set_item("determined", report.fit.determined.to_vec())?;
                d.set_item("rms_nm", report.fit.rms_nm)?;
                d.set_item("rms_unloaded_nm", report.fit.rms_unloaded_nm)?;
                d.set_item("poses", report.samples.len())?;
                Ok(d.unbind())
            })
        })
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
        opt_future(py, async move { client.profile().await })
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
        opt_future(py, async move { client.tcp_speed().await })
    }

    fn tcp_offset<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let client = self.rt();
        opt_future(
            py,
            async move { client.tcp_offset().await.map(|v| v.to_vec()) },
        )
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
        opt_future(py, async move { client.is_simulator().await })
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
        let assertion = flashing_assertion(py, assertion)?;
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
        sys_future(
            py,
            self.rt(),
            Command::SetTcpOffset(cmd::SetTcpOffset { x, y, z }),
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
        index_future(py, async move { client.home(calibrate).await })
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
