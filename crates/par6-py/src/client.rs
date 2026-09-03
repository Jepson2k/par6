//! The async client binding: every method returns an asyncio awaitable
//! driving the `par6-client` core on the shared tokio runtime. The
//! Python shim (`par6.client.async_client`) keeps the public API and
//! waldoctl typing; this layer is transport only. Each verb is the
//! `par6-client` API method of the same name, so no command payload is
//! assembled here.

use std::future::Future;
use std::net::Ipv4Addr;

use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3_async_runtimes::tokio::future_into_py;

use par6_client::{Ack, Client, ClientConfig, ClientError, QueryResult, StatusTransport};
use par6_proto::command as cmd;
use par6_proto::{CompletionPolicy, NUM_JOINTS};

use crate::convert::{
    checked_duration, client_err, frame_of, query_result_dict, shape_from_py, status_dict,
    tool_param_from_py, wire_error_tuple,
};

type Awaitable<'py> = PyResult<Bound<'py, PyAny>>;

/// A SYSTEM command: 1 when the runtime acked it, 0 when unconfirmed; a
/// refusal raises `RobotWireError`.
fn ack_future<'py, F>(py: Python<'py>, fut: F) -> Awaitable<'py>
where
    F: Future<Output = Result<Ack, ClientError>> + Send + 'static,
{
    future_into_py(py, async move {
        match fut.await {
            Ok(Ack::Confirmed) => Ok(1i32),
            Ok(Ack::Unconfirmed) => Ok(0),
            Err(e) => Err(client_err(e)),
        }
    })
}

/// A QUEUED command: its queue index, or -1 when unconfirmed.
fn index_future<'py, F>(py: Python<'py>, fut: F) -> Awaitable<'py>
where
    F: Future<Output = Result<Option<u64>, ClientError>> + Send + 'static,
{
    future_into_py(py, async move {
        match fut.await {
            Ok(Some(index)) => Ok(index as i64),
            Ok(None) => Ok(-1),
            Err(e) => Err(client_err(e)),
        }
    })
}

/// A fire-and-forget send: 1 once the datagram is out.
fn fire_future<'py, F>(py: Python<'py>, fut: F) -> Awaitable<'py>
where
    F: Future<Output = Result<(), ClientError>> + Send + 'static,
{
    future_into_py(py, async move {
        fut.await.map_err(client_err)?;
        Ok(1i32)
    })
}

/// A composite query as a dict; unreachable resolves to `None`.
fn query_future<'py, F>(py: Python<'py>, fut: F) -> Awaitable<'py>
where
    F: Future<Output = Result<QueryResult, ClientError>> + Send + 'static,
{
    future_into_py(py, async move {
        match fut.await {
            Ok(result) => Python::with_gil(|py| query_result_dict(py, &result).map(Some)),
            Err(ClientError::Unreachable) => Ok(None),
            Err(e) => Err(client_err(e)),
        }
    })
}

/// A typed query mapped through `into`; unreachable resolves to `None`.
fn value_future<'py, T, V, F>(
    py: Python<'py>,
    fut: F,
    into: impl FnOnce(T) -> V + Send + 'static,
) -> Awaitable<'py>
where
    F: Future<Output = Result<T, ClientError>> + Send + 'static,
    T: Send + 'static,
    V: for<'a> IntoPyObject<'a> + Send + 'static,
{
    future_into_py(py, async move {
        match fut.await {
            Ok(v) => Ok(Some(into(v))),
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
    ) -> Awaitable<'py> {
        let timeout = checked_duration(timeout, "timeout")?;
        future_into_py(py, async move {
            let parse = |s: &str, what: &str| -> Result<Ipv4Addr, PyErr> {
                s.parse()
                    .map_err(|_| PyRuntimeError::new_err(format!("bad {what}: {s}")))
            };
            let unicast_host = parse(&status_unicast_host, "status host")?;
            let status = if status_transport.eq_ignore_ascii_case("UNICAST") {
                StatusTransport::Unicast { host: unicast_host }
            } else {
                StatusTransport::Multicast {
                    group: parse(&mcast_group, "multicast group")?,
                    iface: parse(&mcast_iface, "multicast interface")?,
                    fallback: unicast_host,
                }
            };
            let cfg = ClientConfig {
                host,
                port,
                timeout,
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
    fn status_after<'py>(&self, py: Python<'py>, last_seq: i64, timeout: f64) -> Awaitable<'py> {
        let client = self.rt();
        let timeout = checked_duration(timeout, "timeout")?;
        future_into_py(py, async move {
            let hit = client
                .wait_status(|s| last_seq < 0 || s.seq as i64 != last_seq, timeout)
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
    /// False on timeout (or when STATUS showed it finished but the
    /// COMPLETE push was lost, so the verdict is unknown); raises
    /// `RobotWireError` on a failed command.
    fn wait_command<'py>(&self, py: Python<'py>, index: u64, timeout: f64) -> Awaitable<'py> {
        let client = self.rt();
        let timeout = checked_duration(timeout, "timeout")?;
        future_into_py(py, async move {
            client
                .wait_command(index, timeout)
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

    fn ping<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        value_future(py, async move { client.ping().await }, |hw| hw)
    }

    fn angles<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        value_future(py, async move { client.angles().await }, |a| a.to_vec())
    }

    fn pose<'py>(&self, py: Python<'py>, frame: u8) -> Awaitable<'py> {
        let client = self.rt();
        let frame = frame_of(frame)?;
        value_future(py, async move { client.pose(frame).await }, |p| p.to_vec())
    }

    fn io<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        value_future(py, async move { client.io().await }, |v| v)
    }

    fn joint_speeds<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        value_future(py, async move { client.joint_speeds().await }, |v| {
            v.to_vec()
        })
    }

    fn status_query<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        query_future(py, async move { client.status_query().await })
    }

    fn tools<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        query_future(py, async move { client.tools().await })
    }

    fn queue<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        query_future(py, async move { client.queue().await })
    }

    fn activity<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        query_future(py, async move { client.activity().await })
    }

    fn loop_stats<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        query_future(py, async move {
            client.loop_stats().await.map(QueryResult::LoopStats)
        })
    }

    fn reachable<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        query_future(py, async move { client.reachable().await })
    }

    fn shapes<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        query_future(py, async move { client.shapes().await })
    }

    fn config_info<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        query_future(py, async move { client.config_info().await })
    }

    fn config_bundle<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        query_future(py, async move { client.config_bundle().await })
    }

    fn payload<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        query_future(py, async move { client.payload().await })
    }

    #[pyo3(signature = (mass, com, inertia=None))]
    fn set_payload<'py>(
        &self,
        py: Python<'py>,
        mass: f64,
        com: [f64; 3],
        inertia: Option<[f64; 6]>,
    ) -> Awaitable<'py> {
        let client = self.rt();
        ack_future(
            py,
            async move { client.set_payload(mass, com, inertia).await },
        )
    }

    fn profile<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        value_future(py, async move { client.profile().await }, |p| p)
    }

    fn error<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        future_into_py(py, async move {
            match client.error().await {
                Ok(Some(e)) => Python::with_gil(|py| Ok(Some(wire_error_tuple(py, &e)))),
                Ok(None) | Err(ClientError::Unreachable) => Ok(None),
                Err(e) => Err(client_err(e)),
            }
        })
    }

    fn tcp_speed<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        value_future(py, async move { client.tcp_speed().await }, |v| v)
    }

    fn tcp_offset<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        value_future(py, async move { client.tcp_offset().await }, |v| v.to_vec())
    }

    fn tool_status<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        future_into_py(py, async move {
            match client.tool_status().await {
                Ok(Some(t)) => {
                    Python::with_gil(|py| crate::convert::tool_status_dict(py, &t).map(Some))
                }
                Ok(None) | Err(ClientError::Unreachable) => Ok(None),
                Err(e) => Err(client_err(e)),
            }
        })
    }

    fn is_simulator<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        value_future(py, async move { client.is_simulator().await }, |v| v)
    }

    // --------------------------------------------------------- system

    fn reset<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        ack_future(py, async move { client.reset().await })
    }

    fn estop<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        ack_future(py, async move { client.estop().await })
    }

    fn set_gravity_comp<'py>(&self, py: Python<'py>, on: bool) -> Awaitable<'py> {
        let client = self.rt();
        ack_future(py, async move { client.set_gravity_comp(on).await })
    }

    fn pause<'py>(&self, py: Python<'py>, on: bool) -> Awaitable<'py> {
        let client = self.rt();
        ack_future(py, async move {
            if on {
                client.pause().await
            } else {
                client.resume().await
            }
        })
    }

    fn stop<'py>(&self, py: Python<'py>, clear_queue: bool) -> Awaitable<'py> {
        let client = self.rt();
        ack_future(py, async move { client.stop(clear_queue).await })
    }

    fn write_io<'py>(&self, py: Python<'py>, port: u8, value: u8) -> Awaitable<'py> {
        let client = self.rt();
        ack_future(py, async move { client.write_io(port, value).await })
    }

    fn simulator<'py>(&self, py: Python<'py>, on: bool) -> Awaitable<'py> {
        let client = self.rt();
        ack_future(py, async move { client.simulator(on).await })
    }

    fn select_profile<'py>(&self, py: Python<'py>, profile: String) -> Awaitable<'py> {
        let client = self.rt();
        ack_future(py, async move { client.select_profile(&profile).await })
    }

    fn reset_state<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        ack_future(py, async move { client.reset_state().await })
    }

    fn connect_hardware<'py>(&self, py: Python<'py>, port: String) -> Awaitable<'py> {
        let client = self.rt();
        ack_future(py, async move { client.connect_hardware(&port).await })
    }

    /// `assertion` is "parked" or "force" (the wire refuses anything else).
    fn enter_flashing<'py>(&self, py: Python<'py>, assertion: &str) -> Awaitable<'py> {
        let assertion = match assertion.to_ascii_lowercase().as_str() {
            "parked" => par6_proto::FlashingAssertion::Parked,
            "force" => par6_proto::FlashingAssertion::Force,
            other => {
                return Err(pyo3::exceptions::PyValueError::new_err(format!(
                    "flashing assertion must be 'parked' or 'force', got {other:?}"
                )))
            }
        };
        let client = self.rt();
        ack_future(py, async move { client.enter_flashing(assertion).await })
    }

    fn exit_flashing<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        ack_future(py, async move { client.exit_flashing().await })
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
    ) -> Awaitable<'py> {
        let gains = cmd::SetPidGains {
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
        };
        let client = self.rt();
        ack_future(py, async move { client.set_pid_gains(gains).await })
    }

    fn set_tcp_offset<'py>(&self, py: Python<'py>, x: f64, y: f64, z: f64) -> Awaitable<'py> {
        let client = self.rt();
        ack_future(py, async move { client.set_tcp_offset(x, y, z).await })
    }

    fn set_shapes<'py>(
        &self,
        py: Python<'py>,
        shapes: Vec<Bound<'py, pyo3::types::PyDict>>,
    ) -> Awaitable<'py> {
        let shapes = shapes
            .iter()
            .map(shape_from_py)
            .collect::<PyResult<Vec<_>>>()?;
        let client = self.rt();
        ack_future(py, async move { client.set_shapes(shapes).await })
    }

    fn set_completion_policy<'py>(&self, py: Python<'py>, policy: u8) -> Awaitable<'py> {
        let policy = CompletionPolicy::from_wire(i64::from(policy)).ok_or_else(|| {
            PyRuntimeError::new_err(format!("unknown completion policy {policy}"))
        })?;
        let client = self.rt();
        ack_future(
            py,
            async move { client.set_completion_policy(policy).await },
        )
    }

    fn set_recipe<'py>(&self, py: Python<'py>, name: String) -> Awaitable<'py> {
        let client = self.rt();
        ack_future(py, async move { client.set_recipe(&name).await })
    }

    // --------------------------------------------------------- queued

    #[pyo3(signature = (calibrate=false))]
    fn home<'py>(&self, py: Python<'py>, calibrate: bool) -> Awaitable<'py> {
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
    ) -> Awaitable<'py> {
        let client = self.rt();
        index_future(py, async move {
            client
                .move_j(angles, duration, speed, accel, blend_radius, rel)
                .await
        })
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
    ) -> Awaitable<'py> {
        let client = self.rt();
        index_future(py, async move {
            client
                .move_j_pose(pose, duration, speed, accel, blend_radius)
                .await
        })
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
    ) -> Awaitable<'py> {
        let frame = frame_of(frame)?;
        let client = self.rt();
        index_future(py, async move {
            client
                .move_l(pose, frame, duration, speed, accel, blend_radius, rel)
                .await
        })
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
    ) -> Awaitable<'py> {
        let frame = frame_of(frame)?;
        let client = self.rt();
        index_future(py, async move {
            client
                .move_c(via, end, frame, duration, speed, accel, blend_radius, rel)
                .await
        })
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
    ) -> Awaitable<'py> {
        let frame = frame_of(frame)?;
        let client = self.rt();
        index_future(py, async move {
            client
                .move_s(waypoints, frame, duration, speed, accel, rel)
                .await
        })
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
    ) -> Awaitable<'py> {
        let frame = frame_of(frame)?;
        let client = self.rt();
        index_future(py, async move {
            client
                .move_p(waypoints, frame, duration, speed, accel, rel)
                .await
        })
    }

    #[pyo3(signature = (tool_name, variant_key))]
    fn select_tool<'py>(
        &self,
        py: Python<'py>,
        tool_name: String,
        variant_key: Option<String>,
    ) -> Awaitable<'py> {
        let client = self.rt();
        index_future(py, async move {
            client.select_tool(&tool_name, variant_key.as_deref()).await
        })
    }

    fn delay<'py>(&self, py: Python<'py>, seconds: f64) -> Awaitable<'py> {
        let client = self.rt();
        index_future(py, async move { client.delay(seconds).await })
    }

    fn checkpoint<'py>(&self, py: Python<'py>, label: String) -> Awaitable<'py> {
        let client = self.rt();
        index_future(py, async move { client.checkpoint(&label).await })
    }

    fn tool_action<'py>(
        &self,
        py: Python<'py>,
        tool_key: String,
        action: String,
        params: Vec<Bound<'py, PyAny>>,
    ) -> Awaitable<'py> {
        let params = params
            .iter()
            .map(tool_param_from_py)
            .collect::<PyResult<Vec<_>>>()?;
        let client = self.rt();
        index_future(py, async move {
            client.tool_action(&tool_key, &action, params).await
        })
    }

    // -------------------------------------------------- fire-and-forget

    #[pyo3(signature = (angles, speed, accel))]
    fn servo_j<'py>(
        &self,
        py: Python<'py>,
        angles: [f64; NUM_JOINTS],
        speed: Option<f64>,
        accel: Option<f64>,
    ) -> Awaitable<'py> {
        let client = self.rt();
        fire_future(
            py,
            async move { client.servo_j(angles, speed, accel).await },
        )
    }

    #[pyo3(signature = (pose, speed, accel))]
    fn servo_j_pose<'py>(
        &self,
        py: Python<'py>,
        pose: [f64; 6],
        speed: Option<f64>,
        accel: Option<f64>,
    ) -> Awaitable<'py> {
        let client = self.rt();
        fire_future(
            py,
            async move { client.servo_j_pose(pose, speed, accel).await },
        )
    }

    #[pyo3(signature = (pose, speed, accel))]
    fn servo_l<'py>(
        &self,
        py: Python<'py>,
        pose: [f64; 6],
        speed: Option<f64>,
        accel: Option<f64>,
    ) -> Awaitable<'py> {
        let client = self.rt();
        fire_future(py, async move { client.servo_l(pose, speed, accel).await })
    }

    #[pyo3(signature = (speeds, duration, accel))]
    fn jog_j<'py>(
        &self,
        py: Python<'py>,
        speeds: [f64; NUM_JOINTS],
        duration: f64,
        accel: Option<f64>,
    ) -> Awaitable<'py> {
        let client = self.rt();
        fire_future(
            py,
            async move { client.jog_j(speeds, duration, accel).await },
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
    ) -> Awaitable<'py> {
        let frame = frame_of(frame)?;
        let client = self.rt();
        fire_future(py, async move {
            client.jog_l(velocities, duration, frame, accel).await
        })
    }

    #[pyo3(signature = (angles, tool_positions))]
    fn teleport<'py>(
        &self,
        py: Python<'py>,
        angles: [f64; NUM_JOINTS],
        tool_positions: Option<Vec<f64>>,
    ) -> Awaitable<'py> {
        let client = self.rt();
        fire_future(
            py,
            async move { client.teleport(angles, tool_positions).await },
        )
    }

    fn reset_loop_stats<'py>(&self, py: Python<'py>) -> Awaitable<'py> {
        let client = self.rt();
        fire_future(py, async move { client.reset_loop_stats().await })
    }
}
