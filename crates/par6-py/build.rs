//! An editable install (`pip install -e python`) loads `_par6` straight out
//! of the source tree, where nothing has bundled the shim or libmujoco next
//! to it, so the extension needs both directories as rpaths to import
//! without `LD_LIBRARY_PATH`. They come from the crates that link the
//! libraries (`DEP_PAR6_SHIM_RPATH` from par6-kin, `DEP_MUJOCO_RPATH` from
//! par6-bus, through their `links` keys). Wheels are unaffected: maturin's
//! repair step rewrites the rpath to `$ORIGIN` when it grafts the
//! libraries into the wheel.
fn main() {
    for var in ["DEP_PAR6_SHIM_RPATH", "DEP_MUJOCO_RPATH"] {
        if let Ok(dir) = std::env::var(var) {
            println!("cargo:rustc-link-arg=-Wl,-rpath,{dir}");
        }
    }
}
