//! With feature `sim-dynamics`, embeds an rpath to the par6_shim install
//! directory so this crate's own test binaries load `libpar6_shim.so`
//! without `LD_LIBRARY_PATH`. Link-args do not propagate across packages,
//! so pinokin-sys's identical rpath only covers ITS test binaries.
//!
//! With feature `sim-mujoco`, embeds an rpath to the libmujoco that
//! mujoco-rs downloaded into `MUJOCO_DOWNLOAD_DIR`: the binding emits the
//! link search path and the link line but no rpath, so without this every
//! binary linking it needs `LD_LIBRARY_PATH` to start.

use std::env;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-env-changed=PAR6_SHIM_LIB_DIR");
    println!("cargo:rerun-if-env-changed=MUJOCO_DOWNLOAD_DIR");
    if env::var_os("CARGO_FEATURE_SIM_DYNAMICS").is_some() {
        // pinokin-sys's build script errors out with guidance when this is
        // missing; no need to duplicate the message here.
        if let Ok(lib_dir) = env::var("PAR6_SHIM_LIB_DIR") {
            println!("cargo:rustc-link-arg=-Wl,-rpath,{lib_dir}");
        }
    }
    if env::var_os("CARGO_FEATURE_SIM_MUJOCO").is_some() {
        // mujoco-rs's own build script already failed loudly if the variable
        // is unset; here it is set, and the download has happened.
        let Ok(download_dir) = env::var("MUJOCO_DOWNLOAD_DIR") else {
            return;
        };
        let lib_dir = mujoco_lib_dir(PathBuf::from(download_dir));
        println!("cargo:rustc-link-arg=-Wl,-rpath,{}", lib_dir.display());
    }
}

/// The MuJoCo release mujoco-rs downloads (its `+mj-<version>` metadata).
/// Kept in step with the workspace's `mujoco-rs` pin; a mismatch only
/// costs the fallback below its accuracy, and a wrong rpath fails loudly
/// at load ("libmujoco.so.<version>: cannot open shared object file").
const MUJOCO_VERSION: &str = "3.12.0";

/// `<download dir>/mujoco-<version>/lib`, the layout mujoco-rs extracts to.
///
/// Cargo does not order this script after mujoco-rs's (no `links` key), so
/// on a first build the download may not have happened yet: then the path
/// is predicted from [`MUJOCO_VERSION`]. Once present, the newest release
/// directory wins, so a binding bump cannot leave this pointing at the
/// previous version.
fn mujoco_lib_dir(download_dir: PathBuf) -> PathBuf {
    let mut releases: Vec<PathBuf> = std::fs::read_dir(&download_dir)
        .into_iter()
        .flatten()
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| {
            p.file_name()
                .and_then(|n| n.to_str())
                .is_some_and(|n| n.starts_with("mujoco-"))
                && p.join("lib").is_dir()
        })
        .collect();
    releases.sort();
    releases
        .pop()
        .unwrap_or_else(|| download_dir.join(format!("mujoco-{MUJOCO_VERSION}")))
        .join("lib")
}
