//! Python binding for the par6 engine: the async wire client and the
//! offline preview, both thin faces over the Rust crates. The Python
//! package (`par6`) keeps its public API as a shim over this module.

mod client;
mod convert;
mod preview;

use std::net::UdpSocket;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use pyo3::prelude::*;

use pyo3::exceptions::PyValueError;
use pyo3::types::PyDict;

use par6_proto::{decode_reply, encode_command, Command, ErrorCode, Reply, UNATTRIBUTED};

/// True when a par6d runtime answers a protocol-v2 PING at `host:port`
/// within `timeout` seconds. One blocking datagram, no retries — the
/// availability probe `Robot` runs before spawning its own daemon.
#[pyfunction]
#[pyo3(signature = (host, port, timeout=0.5))]
fn ping_blocking(py: Python<'_>, host: &str, port: u16, timeout: f64) -> bool {
    py.allow_threads(|| ping_once(host, port, timeout))
}

fn ping_once(host: &str, port: u16, timeout: f64) -> bool {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.subsec_nanos())
        .unwrap_or(1);
    let req_id = nanos.max(1);
    let mut buf = Vec::new();
    if encode_command(&Command::Ping, req_id, &mut buf).is_err() {
        return false;
    }
    let Ok(sock) = UdpSocket::bind(("0.0.0.0", 0)) else {
        return false;
    };
    if sock
        .set_read_timeout(Some(Duration::from_secs_f64(timeout.max(0.001))))
        .is_err()
        || sock.send_to(&buf, (host, port)).is_err()
    {
        return false;
    }
    let mut resp = [0u8; 4096];
    let Ok(n) = sock.recv(&mut resp) else {
        return false;
    };
    matches!(
        decode_reply(&resp[..n]),
        Ok(Reply::Response { req_id: id, .. }) if id == req_id
    )
}

/// Render the runtime's error template for `code` as the wire 6-tuple
/// `(command_index, code, title, cause, effect, remedy)`, with `params`
/// filling the template's `{placeholders}`. The engine formats every
/// refusal from these templates; a preview-side refusal built here says
/// exactly what the runtime would say.
#[pyfunction]
#[pyo3(signature = (code, params=None))]
fn make_wire_error(
    py: Python<'_>,
    code: u16,
    params: Option<&Bound<'_, PyDict>>,
) -> PyResult<PyObject> {
    let code = ErrorCode::from_wire(i64::from(code))
        .ok_or_else(|| PyValueError::new_err(format!("unknown error code {code}")))?;
    let mut pairs: Vec<(String, String)> = Vec::new();
    if let Some(d) = params {
        for (k, v) in d.iter() {
            pairs.push((k.extract()?, v.extract()?));
        }
    }
    let borrowed: Vec<(&str, &str)> = pairs
        .iter()
        .map(|(k, v)| (k.as_str(), v.as_str()))
        .collect();
    let e = par6_proto::make_error(code, UNATTRIBUTED, &borrowed);
    Ok(convert::wire_error_tuple(py, &e))
}

#[pymodule]
fn _par6(py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<client::CoreClient>()?;
    m.add_class::<preview::Preview>()?;
    m.add_function(wrap_pyfunction!(ping_blocking, m)?)?;
    m.add_function(wrap_pyfunction!(make_wire_error, m)?)?;
    m.add("RobotWireError", py.get_type::<convert::RobotWireError>())?;
    m.add("NUM_JOINTS", par6_proto::NUM_JOINTS)?;
    m.add(
        "MAX_JOG_DURATION_S",
        par6_proto::command::MAX_JOG_DURATION_S,
    )?;
    Ok(())
}
