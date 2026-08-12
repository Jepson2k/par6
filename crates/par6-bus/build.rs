//! With feature `sim-dynamics`, embeds an rpath to the par6_shim install
//! directory so this crate's own test binaries load `libpar6_shim.so`
//! without `LD_LIBRARY_PATH`. Link-args do not propagate across packages,
//! so pinokin-sys's identical rpath only covers ITS test binaries.
//!
//! With feature `sim-mujoco`, links `libmujoco` from `PAR6_MUJOCO_LIB_DIR`
//! (exported by `.ffi/env.sh` after `scripts/ffi/setup.sh`) and embeds the
//! matching rpath.

use std::env;
use std::path::Path;

fn main() {
    println!("cargo:rerun-if-env-changed=PAR6_SHIM_LIB_DIR");
    println!("cargo:rerun-if-env-changed=PAR6_MUJOCO_LIB_DIR");
    if env::var_os("CARGO_FEATURE_SIM_DYNAMICS").is_some() {
        // pinokin-sys's build script errors out with run-setup.sh guidance when
        // this is missing; no need to duplicate the message here.
        if let Ok(lib_dir) = env::var("PAR6_SHIM_LIB_DIR") {
            println!("cargo:rustc-link-arg=-Wl,-rpath,{lib_dir}");
        }
    }
    if env::var_os("CARGO_FEATURE_SIM_MUJOCO").is_some() {
        let lib_dir = env::var("PAR6_MUJOCO_LIB_DIR").unwrap_or_else(|_| {
            panic!(
                "par6-bus was built with the `sim-mujoco` feature but \
                 PAR6_MUJOCO_LIB_DIR is not set.\nRun scripts/ffi/setup.sh, \
                 then `source .ffi/env.sh` (or export PAR6_MUJOCO_LIB_DIR to \
                 the directory containing libmujoco.so)."
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
