//! Links the `par6_shim` C++ library.
//!
//! Consumes:
//! - `PAR6_SHIM_LIB_DIR`: directory holding `libpar6_shim.so` /
//!   `libpar6_shim.a`. When unset, the shim `scripts/ffi/setup.sh` installs
//!   into the repo's own `.ffi/shim/lib` is used, so a checkout that has run
//!   `setup.sh` builds without sourcing `.ffi/env.sh`.
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

    let lib_dir = env::var("PAR6_SHIM_LIB_DIR")
        .ok()
        .or_else(|| repo_shim_dir("lib"))
        .unwrap_or_else(|| {
            panic!(
                "par6-kin found no shim to link: \
                 PAR6_SHIM_LIB_DIR is not set and the repo has no .ffi/shim/lib.\n\
                 Run scripts/ffi/setup.sh (or export PAR6_SHIM_LIB_DIR to the \
                 directory containing libpar6_shim.so)."
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
             Run scripts/ffi/setup.sh to build the shim.",
            lib_file
        );
    }

    if let Some(include_dir) = env::var("PAR6_SHIM_INCLUDE_DIR")
        .ok()
        .or_else(|| repo_shim_dir("include"))
    {
        let header = Path::new(&include_dir).join("par6_shim.h");
        if !header.exists() {
            panic!(
                "par6_shim.h not found in PAR6_SHIM_INCLUDE_DIR ({include_dir}); \
                 it does not look like a par6_shim install prefix."
            );
        }
    }

    warn_if_shim_predates_its_sources(&lib_path);

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

/// Fail the build when the installed shim is older than the `cpp/` sources it
/// was built from.
///
/// Nothing else notices: cargo does not build the shim, so an edit under
/// `cpp/` leaves a stale `.so` linked and the failures surface as wrong
/// numbers deep in the kinematics tests — a TOPPRA timing law that has been
/// replaced, an error message the shim does not carry yet — with nothing
/// pointing at the build. Only meaningful in a checkout; an installed shim
/// with no `cpp/` beside it is left alone.
fn warn_if_shim_predates_its_sources(lib_path: &Path) {
    let cpp = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../cpp");
    let Ok(cpp) = std::fs::canonicalize(&cpp) else {
        return;
    };
    println!("cargo:rerun-if-changed={}", cpp.display());
    let Ok(built) = lib_path.metadata().and_then(|m| m.modified()) else {
        return;
    };
    let mut newest: Option<(std::time::SystemTime, std::path::PathBuf)> = None;
    let mut stack = vec![cpp];
    while let Some(dir) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                stack.push(path);
                continue;
            }
            println!("cargo:rerun-if-changed={}", path.display());
            if let Ok(t) = entry.metadata().and_then(|m| m.modified()) {
                if newest.as_ref().is_none_or(|(best, _)| t > *best) {
                    newest = Some((t, path));
                }
            }
        }
    }
    if let Some((t, path)) = newest {
        if t > built {
            panic!(
                "the installed shim is older than the sources it is built from:\n                   {} was modified after {}\n\n\
                 Linking it anyway runs the OLD C++ against the new Rust, which \
                 shows up as wrong numbers in the kinematics and trajectory \
                 tests rather than as a build error.\n\n  \
                 Rebuild it: FORCE=1 scripts/ffi/setup.sh",
                path.display(),
                lib_path.display(),
            );
        }
    }
}

/// `<repo>/.ffi/shim/<sub>` when `scripts/ffi/setup.sh` has populated it.
fn repo_shim_dir(sub: &str) -> Option<String> {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../.ffi/shim")
        .join(sub);
    std::fs::canonicalize(dir)
        .ok()
        .map(|p| p.to_string_lossy().into_owned())
}
