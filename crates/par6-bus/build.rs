//! Publish native library paths to the daemon and Python extension. MuJoCo's
//! matching library is installed by mujoco-rs into the pixi download prefix.
use std::{env, path::PathBuf};

fn main() {
    let shim = env::var("DEP_PAR6_SHIM_RPATH").expect("par6-kin publishes the linked shim path");
    println!("cargo:rustc-link-arg=-Wl,-rpath,{shim}");
    println!("cargo:rerun-if-env-changed=MUJOCO_DYNAMIC_LINK_DIR");
    println!("cargo:rerun-if-env-changed=MUJOCO_DOWNLOAD_DIR");
    // This directory follows the mujoco-rs pin, as in pixi's activation.
    let lib = env::var_os("MUJOCO_DYNAMIC_LINK_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            env::var_os("MUJOCO_DOWNLOAD_DIR")
                .map(PathBuf::from)
                .unwrap_or_else(|| {
                    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../.ffi/mujoco")
                })
                .join("mujoco-3.12.0/lib")
        });
    // Cargo can run this script before mujoco-rs has downloaded the library.
    // The linker checks its presence after dependencies have finished building.
    let lib = std::path::absolute(lib).expect("MuJoCo library path must resolve");
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", lib.display());
    println!("cargo:rpath={}", lib.display());
}
