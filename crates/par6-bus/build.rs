//! With feature `sim-dynamics`, embeds an rpath to the par6_shim install
//! directory so this crate's own test binaries load `libpar6_shim.so`
//! without `LD_LIBRARY_PATH`. Link-args do not propagate across packages,
//! so pinokin-sys's identical rpath only covers ITS test binaries.

use std::env;

fn main() {
    println!("cargo:rerun-if-env-changed=PAR6_SHIM_LIB_DIR");
    if env::var_os("CARGO_FEATURE_SIM_DYNAMICS").is_none() {
        return;
    }
    // pinokin-sys's build script errors out with run-setup.sh guidance when
    // this is missing; no need to duplicate the message here.
    if let Ok(lib_dir) = env::var("PAR6_SHIM_LIB_DIR") {
        println!("cargo:rustc-link-arg=-Wl,-rpath,{lib_dir}");
    }
}
