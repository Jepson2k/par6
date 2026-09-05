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

    check_shim_matches_sources(&lib_path);

    println!("cargo:rustc-link-search=native={lib_dir}");
    // The shim's install rpath covers its own Pinocchio deps; the rpath here
    // lets test/bin targets find the shim without LD_LIBRARY_PATH.
    println!("cargo:rustc-link-arg=-Wl,-rpath,{lib_dir}");

    // The directory dependents' own binaries and test executables need on
    // their rpath to load the shim at run time; cargo hands it to their
    // build scripts as `DEP_PAR6_SHIM_RPATH` through the `links` key.
    let runtime_dir = match link.as_str() {
        "dylib" => {
            println!("cargo:rustc-link-lib=dylib=par6_shim");
            lib_dir.clone()
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
            dep_dir
        }
        _ => unreachable!(),
    };
    println!("cargo:rpath={runtime_dir}");
}

/// The digest `scripts/ffi/setup.sh` records beside the shim it installs:
/// the identity of the `cpp/` tree it was built from.
const CPP_DIGEST_FILE: &str = "cpp.sha256";

/// Fail the build when the installed shim was not built from the `cpp/`
/// beside it.
///
/// Nothing else notices: cargo does not build the shim, so an edit under
/// `cpp/` leaves a stale `.so` linked and the failures surface as wrong
/// numbers deep in the kinematics tests — a TOPPRA timing law that has been
/// replaced, an error message the shim does not carry yet — with nothing
/// pointing at the build. The identity compared is the content digest
/// `setup.sh` records at install, never a timestamp: a checkout, a rebase
/// or a cache restore rewrites mtimes without changing a byte. Only
/// meaningful in a checkout; an installed shim with no `cpp/` beside it is
/// left alone.
fn check_shim_matches_sources(lib_path: &Path) {
    let cpp = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../cpp");
    let Ok(cpp) = std::fs::canonicalize(&cpp) else {
        return;
    };
    println!("cargo:rerun-if-changed={}", cpp.display());
    let stamp = lib_path
        .parent()
        .and_then(Path::parent)
        .map(|prefix| prefix.join(CPP_DIGEST_FILE))
        .unwrap_or_default();
    let recorded = std::fs::read_to_string(&stamp).ok();
    let current = cpp_digest(&cpp);
    if recorded.as_deref().map(str::trim) == Some(current.as_str()) {
        println!("cargo:rerun-if-changed={}", stamp.display());
        return;
    }
    panic!(
        "the installed shim was not built from the cpp/ in this checkout:\n  \
         {} records {}\n  cpp/ digests to {current}\n\n\
         Linking it anyway runs the OLD C++ against the new Rust, which \
         shows up as wrong numbers in the kinematics and trajectory tests \
         rather than as a build error.\n\n  \
         Rebuild it: scripts/ffi/setup.sh",
        stamp.display(),
        recorded.as_deref().map_or("no digest", str::trim),
    );
}

/// The digest of every file under `cpp/`, as `setup.sh` computes it:
/// `find cpp -type f | LC_ALL=C sort | xargs sha256sum | sha256sum` from the
/// repo root, so the two sides agree byte for byte.
fn cpp_digest(cpp: &Path) -> String {
    use sha2::{Digest, Sha256};
    let root = cpp.parent().expect("cpp/ has a parent");
    let mut files = Vec::new();
    let mut stack = vec![cpp.to_path_buf()];
    while let Some(dir) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        for entry in entries.flatten() {
            let Ok(kind) = entry.file_type() else {
                continue;
            };
            if kind.is_dir() {
                stack.push(entry.path());
            } else if kind.is_file() {
                let path = entry.path();
                let rel = path
                    .strip_prefix(root)
                    .expect("under the repo root")
                    .to_string_lossy()
                    .replace('\\', "/");
                files.push((rel, path));
            }
        }
    }
    files.sort_by(|a, b| a.0.as_bytes().cmp(b.0.as_bytes()));
    let mut listing = Sha256::new();
    for (rel, path) in files {
        let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
        listing.update(format!("{:x}  {rel}\n", Sha256::digest(&bytes)));
    }
    format!("{:x}", listing.finalize())
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
