//! A development build runs straight out of `target/`, where nothing has
//! installed the shim or libmujoco beside it, so the binary (and the
//! crate's test executables) carry both directories as rpaths. Each
//! directory comes from the crate that links the library: par6-kin
//! publishes the shim's as `DEP_PAR6_SHIM_RPATH` and par6-bus libmujoco's
//! as `DEP_MUJOCO_RPATH`, through their `links` keys, so a path is derived
//! in one place. Deploy builds replace both (`build-aarch64.sh` sets the
//! install prefix's rpath with patchelf).
fn main() {
    for var in ["DEP_PAR6_SHIM_RPATH", "DEP_MUJOCO_RPATH"] {
        if let Ok(dir) = std::env::var(var) {
            // Every linked target, not just bins and the tests/ directory:
            // the library's own unit-test harness is neither, and without
            // the rpath it dies at startup with a missing library.
            println!("cargo:rustc-link-arg=-Wl,-rpath,{dir}");
        }
    }
}
