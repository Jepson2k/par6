//! A development build runs straight out of `target/`, where nothing has
//! installed the shim or libmujoco beside it, so the binary (and the
//! crate's test executables) carry both directories as rpaths:
//! `PAR6_SHIM_LIB_DIR` / `PAR6_MUJOCO_LIB_DIR` when set, else the repo's
//! own `.ffi/shim/lib` and `.ffi/env/lib` from `scripts/ffi/setup.sh`.
//! libmujoco is linked by par6-bus, whose link-args stop at its own
//! targets. Deploy builds replace both (`build-aarch64.sh` sets the
//! install prefix's rpath with patchelf).
fn main() {
    let ffi = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../.ffi");
    for (var, repo_dir) in [
        ("PAR6_SHIM_LIB_DIR", "shim/lib"),
        ("PAR6_MUJOCO_LIB_DIR", "env/lib"),
    ] {
        println!("cargo:rerun-if-env-changed={var}");
        let dir = std::env::var(var).ok().or_else(|| {
            std::fs::canonicalize(ffi.join(repo_dir))
                .ok()
                .map(|p| p.to_string_lossy().into_owned())
        });
        if let Some(dir) = dir {
            // Every linked target, not just bins and the tests/ directory:
            // the library's own unit-test harness is neither, and without
            // the rpath it dies at startup with a missing library.
            println!("cargo:rustc-link-arg=-Wl,-rpath,{dir}");
        }
    }
}
