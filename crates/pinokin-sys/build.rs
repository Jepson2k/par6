//! Links the `par6_shim` C++ library.
//!
//! Consumes:
//! - `PAR6_SHIM_LIB_DIR` (required): directory holding `libpar6_shim.so` /
//!   `libpar6_shim.a` — `pixi run setup` builds it and the pixi activation
//!   exports this.
//! - `PAR6_SHIM_INCLUDE_DIR` (optional): directory holding `par6_shim.h`;
//!   only sanity-checked here (declarations are hand-written, no bindgen),
//!   and the place a future bindgen step would point at.
//! - `PAR6_SHIM_LINK` (optional): `dylib` (default) or `static`. Static links
//!   the shim archive and needs `PAR6_SHIM_DEP_LIB_DIR` pointing at the
//!   Pinocchio/coal/toppra library directory (the conda env's `lib/`).

use std::env;
use std::path::Path;

fn main() {
    println!("cargo:rerun-if-env-changed=PAR6_SHIM_LIB_DIR");
    println!("cargo:rerun-if-env-changed=PAR6_SHIM_INCLUDE_DIR");
    println!("cargo:rerun-if-env-changed=PAR6_SHIM_LINK");
    println!("cargo:rerun-if-env-changed=PAR6_SHIM_DEP_LIB_DIR");

    let lib_dir = env::var("PAR6_SHIM_LIB_DIR").unwrap_or_else(|_| {
        panic!(
            "PAR6_SHIM_LIB_DIR is not set. Build under `pixi run` (the activation \
             exports it once `pixi run setup` has built the shim), or export it \
             to the directory containing libpar6_shim.so."
        )
    });

    let link = env::var("PAR6_SHIM_LINK").unwrap_or_else(|_| "dylib".into());
    let lib_file = match link.as_str() {
        "dylib" => "libpar6_shim.so",
        "static" => "libpar6_shim.a",
        other => panic!("PAR6_SHIM_LINK must be `dylib` or `static`, got `{other}`"),
    };
    let lib_path = Path::new(&lib_dir).join(lib_file);
    if !lib_path.exists() {
        panic!(
            "{} not found in PAR6_SHIM_LIB_DIR ({lib_dir}). \
             Run `pixi run setup` to build the shim.",
            lib_file
        );
    }

    if let Ok(include_dir) = env::var("PAR6_SHIM_INCLUDE_DIR") {
        let header = Path::new(&include_dir).join("par6_shim.h");
        if !header.exists() {
            panic!(
                "par6_shim.h not found in PAR6_SHIM_INCLUDE_DIR ({include_dir}); \
                 it does not look like a par6_shim install prefix."
            );
        }
    }

    println!("cargo:rustc-link-search=native={lib_dir}");
    // The shim's install rpath covers its own Pinocchio deps; the rpath here
    // lets test/bin targets find the shim without LD_LIBRARY_PATH.
    println!("cargo:rustc-link-arg=-Wl,-rpath,{lib_dir}");

    match link.as_str() {
        "dylib" => {
            println!("cargo:rustc-link-lib=dylib=par6_shim");
        }
        "static" => {
            // Static shim archive: Pinocchio, coal and toppra (shared-only
            // in the .ffi env) and the C++ runtime must be linked explicitly.
            let dep_dir = env::var("PAR6_SHIM_DEP_LIB_DIR").unwrap_or_else(|_| {
                panic!(
                    "PAR6_SHIM_LINK=static requires PAR6_SHIM_DEP_LIB_DIR \
                     (the Pinocchio/toppra lib directory, e.g. <repo>/.ffi/env/lib)."
                )
            });
            println!("cargo:rustc-link-lib=static=par6_shim");
            println!("cargo:rustc-link-search=native={dep_dir}");
            println!("cargo:rustc-link-arg=-Wl,-rpath,{dep_dir}");
            println!("cargo:rustc-link-lib=dylib=pinocchio_default");
            println!("cargo:rustc-link-lib=dylib=pinocchio_parsers");
            println!("cargo:rustc-link-lib=dylib=pinocchio_collision");
            println!("cargo:rustc-link-lib=dylib=coal");
            println!("cargo:rustc-link-lib=dylib=toppra");
            println!("cargo:rustc-link-lib=dylib=stdc++");
        }
        _ => unreachable!(),
    }
}
