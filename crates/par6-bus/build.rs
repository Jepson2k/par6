//! Republishes libmujoco's directory as an rpath.
//!
//! `mujoco-rs`'s build script emits the link search path and `-lmujoco` for
//! the prefix `MUJOCO_DYNAMIC_LINK_DIR` names — pixi's for a native build,
//! the target env for a cross one — but no rpath — so nothing that links it runs without
//! `LD_LIBRARY_PATH`. par6-bus is the only crate that links libmujoco, so
//! its `links = "mujoco"` key is where the directory is derived once: as a
//! link arg for this crate's own test binaries, and as `DEP_MUJOCO_RPATH`
//! for dependents (link args do not propagate across cargo packages).

use std::path::Path;

fn main() {
    println!("cargo:rerun-if-env-changed=MUJOCO_DYNAMIC_LINK_DIR");
    let lib_dir = std::env::var("MUJOCO_DYNAMIC_LINK_DIR").unwrap_or_else(|_| {
        panic!(
            "MUJOCO_DYNAMIC_LINK_DIR is not set; par6-bus links libmujoco from \
             the conda prefix pixi provides.\nRun under pixi (`pixi run \
             cargo ...`), or source .ffi/env-<arch>.sh for a cross build."
        )
    });
    if !Path::new(&lib_dir).join("libmujoco.so").exists() {
        panic!(
            "libmujoco.so not found in MUJOCO_DYNAMIC_LINK_DIR ({lib_dir}). \
             It is a pixi dependency; run `pixi install`."
        );
    }
    println!("cargo:rustc-link-arg=-Wl,-rpath,{lib_dir}");
    println!("cargo:rpath={lib_dir}");
}
