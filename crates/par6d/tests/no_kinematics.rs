//! What a par6d built without feature `ffi` does: nothing.
//!
//! Kinematics is not an optional part of the runtime's advertised
//! surface. Without it the TCP pose broadcast is NaN, `move_l` /
//! `move_j_pose` / `servo_l` / `servo_j_pose` / `jog_l` are all refused,
//! the `TOPPRA` profile the registry advertises is unavailable, and the
//! collision world is empty while `set_shapes` still answers success —
//! and every one of those is invisible to a client, which sees a
//! runtime that booted and reported healthy. So the build that lacks
//! them does not boot at all, and says why.
#![cfg(not(feature = "ffi"))]

use std::net::UdpSocket;
use std::path::PathBuf;

use par6d::{Daemon, Options};

fn config_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml")
}

fn free_udp_port() -> u16 {
    UdpSocket::bind("127.0.0.1:0")
        .expect("bind probe socket")
        .local_addr()
        .expect("probe addr")
        .port()
}

/// `--sim` is the easiest possible boot — no CAN interface, no
/// privileges, the mode CI and every developer uses — and it is refused
/// too. A degraded runtime is not a supported configuration in sim
/// either: Waldo Commander talks to a simulated runtime over the same
/// protocol and would read the same NaN pose from it.
#[test]
fn a_runtime_without_kinematics_refuses_to_boot_and_says_why() {
    let opts = Options {
        sim: true,
        config: Some(config_path()),
        command_port: Some(free_udp_port()),
        status_port: Some(free_udp_port()),
        telemetry_port: Some(free_udp_port()),
        ..Options::default()
    };
    let err = match Daemon::start(&opts) {
        Ok(_) => panic!("a par6d without feature `ffi` must not boot"),
        Err(e) => e.to_string(),
    };

    // Not just "an error": the operator has to be able to act on it, so
    // the message names the missing feature and how to get it.
    for expected in ["ffi", "--features ffi", "scripts/ffi/setup.sh"] {
        assert!(
            err.contains(expected),
            "the refusal must tell the operator how to fix it \
             (missing {expected:?}): {err}"
        );
    }

    // And nothing was left running: the ports it would have bound are
    // still free, so a failed boot cannot squat on the command plane.
    let port = opts.command_port.expect("port was set");
    UdpSocket::bind(("127.0.0.1", port))
        .unwrap_or_else(|e| panic!("a refused boot left UDP {port} bound: {e}"));
}

/// The binary reports the same refusal as a nonzero exit rather than a
/// `PAR6D_READY` line, so a supervisor sees a failed start instead of a
/// service that came up and serves nonsense.
#[test]
fn the_binary_exits_nonzero_instead_of_reporting_ready() {
    let exe = PathBuf::from(env!("CARGO_BIN_EXE_par6d"));
    let out = std::process::Command::new(&exe)
        .args(["--sim", "--config"])
        .arg(config_path())
        .arg("--port")
        .arg(free_udp_port().to_string())
        .output()
        .expect("run par6d");

    assert!(
        !out.status.success(),
        "par6d without kinematics exited {:?}",
        out.status.code()
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(
        !stdout.contains("PAR6D_READY"),
        "a refused boot must not announce readiness: {stdout}"
    );
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        stderr.contains("--features ffi"),
        "the binary must print the actionable refusal: {stderr}"
    );
}
