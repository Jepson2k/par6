//! The `ffi` test executables run straight out of `target/`, where nothing
//! has installed the shim beside them, so every linked target carries the
//! shim directory as
//! an rpath: `PAR6_SHIM_LIB_DIR` when set, else the repo's own
//! `.ffi/shim/lib` from `scripts/ffi/setup.sh`.
fn main() {
    println!("cargo:rerun-if-env-changed=PAR6_SHIM_LIB_DIR");
    let repo_shim = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../.ffi/shim/lib");
    let dir = std::env::var("PAR6_SHIM_LIB_DIR").ok().or_else(|| {
        std::fs::canonicalize(&repo_shim)
            .ok()
            .map(|p| p.to_string_lossy().into_owned())
    });
    if let Some(dir) = dir {
        println!("cargo:rustc-link-arg=-Wl,-rpath,{dir}");
    }
}
