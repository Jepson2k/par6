//! `par6.telemetry.TelemetryReader` — the blocking telemetry consumer,
//! a thin face over [`par6_client::telemetry::TelemetryReader`].

use std::net::Ipv4Addr;

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyDict;

use par6_client::telemetry::TelemetryPacket;
use par6_client::StatusTransport;
use par6_proto::telemetry::TelemetryValue;

use crate::convert::checked_duration;

/// Blocking receiver for the daemon's telemetry stream. Each frame is a
/// dict: `recipe`, `seq`, `mono_time_ns`, and `fields` — the recipe's
/// values keyed by field name.
#[pyclass(module = "par6._par6")]
pub struct TelemetryReader {
    inner: Option<par6_client::TelemetryReader>,
}

fn frame_dict(py: Python<'_>, pkt: &TelemetryPacket) -> PyResult<PyObject> {
    let d = PyDict::new(py);
    d.set_item("recipe", &pkt.recipe)?;
    d.set_item("seq", pkt.seq)?;
    d.set_item("mono_time_ns", pkt.mono_time_ns)?;
    let fields = PyDict::new(py);
    for (key, value) in &pkt.fields {
        match value {
            TelemetryValue::U64(v) => fields.set_item(key, v)?,
            TelemetryValue::F64(v) => fields.set_item(key, v)?,
            TelemetryValue::Arr(v) => fields.set_item(key, v.as_slice())?,
        }
    }
    d.set_item("fields", fields)?;
    Ok(d.into())
}

#[pymethods]
impl TelemetryReader {
    /// Bind on `port`. `host` is the local address to bind (the daemon's
    /// unicast destination); pass a multicast `group` to join the STATUS
    /// ladder's group instead.
    #[new]
    #[pyo3(signature = (port, host="127.0.0.1", group=None))]
    fn new(port: u16, host: &str, group: Option<&str>) -> PyResult<Self> {
        let host: Ipv4Addr = host
            .parse()
            .map_err(|_| PyValueError::new_err(format!("invalid host {host:?}")))?;
        let transport = match group {
            Some(g) => StatusTransport::Multicast {
                group: g
                    .parse()
                    .map_err(|_| PyValueError::new_err(format!("invalid group {g:?}")))?,
                iface: host,
                fallback: host,
            },
            None => StatusTransport::Unicast { host },
        };
        let inner = par6_client::TelemetryReader::open(transport, port)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(Self { inner: Some(inner) })
    }

    /// The next frame, waiting up to `timeout` seconds; `None` when the
    /// stream stayed silent (no recipe active, or nothing new yet).
    #[pyo3(signature = (timeout=1.0))]
    fn recv(&mut self, py: Python<'_>, timeout: f64) -> PyResult<Option<PyObject>> {
        let reader = self
            .inner
            .as_mut()
            .ok_or_else(|| PyValueError::new_err("reader is closed"))?;
        let timeout = checked_duration(timeout, "timeout")?;
        let pkt = py
            .allow_threads(|| reader.recv(timeout))
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        pkt.map(|p| frame_dict(py, &p)).transpose()
    }

    /// Every frame currently waiting on the socket, oldest first. Frames
    /// this reader's registry cannot label are skipped, not raised.
    fn drain(&mut self, py: Python<'_>) -> PyResult<Vec<PyObject>> {
        let reader = self
            .inner
            .as_mut()
            .ok_or_else(|| PyValueError::new_err("reader is closed"))?;
        let pkts = reader
            .drain()
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        pkts.iter().map(|p| frame_dict(py, p)).collect()
    }

    /// Frames skipped so far because this registry could not label them.
    fn skipped(&self) -> u64 {
        self.inner.as_ref().map_or(0, |r| r.skipped())
    }

    fn close(&mut self) {
        self.inner = None;
    }

    fn __enter__(slf: Py<Self>) -> Py<Self> {
        slf
    }

    #[pyo3(signature = (*_args))]
    fn __exit__(&mut self, _args: &Bound<'_, pyo3::types::PyTuple>) -> bool {
        self.inner = None;
        false
    }
}
