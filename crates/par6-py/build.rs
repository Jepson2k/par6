//! An editable install (`pip install -e python`) loads `_par6` straight out
//! of the source tree, where nothing has bundled the shim next to it, so
//! the extension needs the shim's directory as an rpath to import without
//! `LD_LIBRARY_PATH`. Wheels are unaffected: maturin's repair step rewrites
//! the rpath to `$ORIGIN` when it grafts the shim into the wheel.
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
