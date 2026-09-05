//! With feature `sim-dynamics`, embeds an rpath to the par6_shim install
//! directory so this crate's own test binaries load `libpar6_shim.so`
//! without `LD_LIBRARY_PATH`. Link-args do not propagate across packages,
//! so par6-kin's identical rpath only covers ITS test binaries.
//!
//! With feature `sim-mujoco`, links `libmujoco` from `PAR6_MUJOCO_LIB_DIR`
//! (exported by `.ffi/env.sh` after `scripts/ffi/setup.sh`), falling back
//! to the repo's own `.ffi/env/lib` so a checkout that has run `setup.sh`
//! builds without sourcing the env file, and embeds the matching rpath.

use std::env;
use std::path::Path;

fn main() {
    println!("cargo:rerun-if-env-changed=PAR6_SHIM_LIB_DIR");
    println!("cargo:rerun-if-env-changed=PAR6_MUJOCO_LIB_DIR");
    if env::var_os("CARGO_FEATURE_SIM_DYNAMICS").is_some() {
        // par6-kin's build script errors out with setup.sh guidance when this
        // is missing; no need to duplicate the message here.
        if let Ok(lib_dir) = env::var("PAR6_SHIM_LIB_DIR") {
            println!("cargo:rustc-link-arg=-Wl,-rpath,{lib_dir}");
        }
    }
    if env::var_os("CARGO_FEATURE_SIM_MUJOCO").is_some() {
        let lib_dir = env::var("PAR6_MUJOCO_LIB_DIR")
            .ok()
            .or_else(repo_env_lib_dir)
            .unwrap_or_else(|| {
                panic!(
                    "par6-bus was built with the `sim-mujoco` feature but \
                     PAR6_MUJOCO_LIB_DIR is not set and the repo has no \
                     .ffi/env/lib.\nRun scripts/ffi/setup.sh (or export \
                     PAR6_MUJOCO_LIB_DIR to the directory containing \
                     libmujoco.so)."
                )
            });
        if !Path::new(&lib_dir).join("libmujoco.so").exists() {
            panic!(
                "libmujoco.so not found in PAR6_MUJOCO_LIB_DIR ({lib_dir}). \
                 Run scripts/ffi/setup.sh to install it."
            );
        }
        println!("cargo:rustc-link-search=native={lib_dir}");
        println!("cargo:rustc-link-lib=dylib=mujoco");
        println!("cargo:rustc-link-arg=-Wl,-rpath,{lib_dir}");
    }
}

/// `<repo>/.ffi/env/lib` when `scripts/ffi/setup.sh` has populated it.
fn repo_env_lib_dir() -> Option<String> {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../.ffi/env/lib");
    std::fs::canonicalize(dir)
        .ok()
        .map(|p| p.to_string_lossy().into_owned())
}
