//! A development build runs straight out of `target/`, where nothing has
//! installed the shim beside it, so the binary (and the crate's test
//! executables) carry the shim directory
//! as an rpath: `PAR6_SHIM_LIB_DIR` when set, else the repo's own
//! `.ffi/shim/lib` from `scripts/ffi/setup.sh`. Deploy builds replace it
//! (`build-aarch64.sh` sets the install prefix's rpath with patchelf).
fn main() {
    println!("cargo:rerun-if-env-changed=PAR6_SHIM_LIB_DIR");
    let repo_shim = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../.ffi/shim/lib");
    let dir = std::env::var("PAR6_SHIM_LIB_DIR").ok().or_else(|| {
        std::fs::canonicalize(&repo_shim)
            .ok()
            .map(|p| p.to_string_lossy().into_owned())
    });
    if let Some(dir) = dir {
        // Every linked target, not just bins and the tests/ directory:
        // the library's own unit-test harness is neither, and without the
        // rpath it dies at startup with a missing libpar6_shim.
        println!("cargo:rustc-link-arg=-Wl,-rpath,{dir}");
    }
}
