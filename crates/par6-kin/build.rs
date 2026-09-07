//! Builds and links the `par6_shim` C++ library.
//!
//! The shim (`cpp/`) and toppra-cpp — which has no conda package, so it is
//! built from the pinned commit below — are compiled into this crate's
//! `OUT_DIR` and linked from there. That is the ordinary `-sys` arrangement,
//! and it is what makes cargo the single owner of freshness: an edit under
//! `cpp/` reruns this script and rebuilds the objects it touched, so a stale
//! `.so` cannot be linked and there is nothing to detect after the fact.
//! Everything else the shim needs (Pinocchio, coal, eigen, urdfdom, the
//! compiler, cmake, ninja) comes from the pixi environment, which is also
//! where `CONDA_PREFIX` points.
//!
//! Consumed environment:
//! - `PAR6_TOPPRA_SRC` — a toppra checkout to build instead of fetching one
//!   (offline builds; the commit is not checked, so it is the caller's job
//!   to hand over the pinned one).
//! - `CMAKE_BUILD_PARALLEL_LEVEL` — overrides the RAM-derived job count.

use std::path::{Path, PathBuf};
use std::process::Command;

/// toppra-cpp pin (v0.6.9 release commit). MIT; built with the bundled
/// Seidel LP solver — no qpOASES/GLPK, so no extra conda dependencies.
const TOPPRA_COMMIT: &str = "142456f3282c92c93ab97749a24856661924d989";
const TOPPRA_REPO: &str = "https://github.com/hungpham2511/toppra";

/// RSS one Pinocchio/coal translation unit needs \[GB\]: 3.9 measured on the
/// control box (cgroup memory.peak, -j1, 2026-09). Overcommitting this on a
/// swapless host livelocks it in reclaim rather than OOM-killing, so the
/// default job count is what MemAvailable can hold.
const JOB_MEM_GB: u64 = 4;

fn main() {
    println!("cargo:rerun-if-env-changed=PAR6_TOPPRA_SRC");
    // The shim's install rpath names the prefix it was built against, and
    // pixi gives each environment its own. Without this a `py312` job could
    // link a shim whose rpath points into `envs/default`, which is the same
    // cross-environment mistake a shared build tree used to make — just
    // relocated into OUT_DIR.
    println!("cargo:rerun-if-env-changed=CONDA_PREFIX");

    let lib_dir = build_shim();

    println!("cargo:rustc-link-search=native={}", lib_dir.display());
    println!("cargo:rustc-link-lib=dylib=par6_shim");
    // The shim's own install rpath covers its Pinocchio/toppra dependencies;
    // this one lets THIS crate's test binaries load the shim. Link args do
    // not propagate across cargo packages, so the directory also goes out
    // over the `links` key as `DEP_PAR6_SHIM_RPATH` for dependents.
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", lib_dir.display());
    println!("cargo:rpath={}", lib_dir.display());
}

/// Build toppra and `cpp/` into `OUT_DIR`; return the shim's library dir.
fn build_shim() -> PathBuf {
    let out = PathBuf::from(std::env::var("OUT_DIR").expect("cargo sets OUT_DIR"));
    let prefix = std::env::var("CONDA_PREFIX").unwrap_or_else(|_| {
        panic!(
            "CONDA_PREFIX is not set: the shim links Pinocchio, coal, eigen \
             and urdfdom from the pixi environment.\nRun under pixi: \
             `pixi run cargo ...`, or `pixi run setup`."
        )
    });

    let toppra = out.join("toppra");
    if !toppra.join("lib/libtoppra.so").exists() {
        let src = toppra_source(&out);
        cmake(
            &src.join("cpp"),
            &out.join("toppra-build"),
            &toppra,
            &[&prefix],
            &[
                "-DBUILD_TESTING=OFF",
                "-DPYTHON_BINDINGS=OFF",
                "-DBUILD_WITH_PINOCCHIO=OFF",
                "-DBUILD_WITH_qpOASES=OFF",
                "-DBUILD_WITH_GLPK=OFF",
                "-DTOPPRA_WARN_ON=OFF",
            ],
        );
    }

    let cpp = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../cpp");
    let cpp = std::fs::canonicalize(&cpp)
        .unwrap_or_else(|e| panic!("cpp/ not found at {}: {e}", cpp.display()));
    println!("cargo:rerun-if-changed={}", cpp.display());

    let shim = out.join("shim");
    cmake(
        &cpp,
        &out.join("shim-build"),
        &shim,
        &[&prefix, &toppra.display().to_string()],
        &[],
    );

    // libtoppra ends up in the shim's own directory rather than a prefix of
    // its own. A DT_NEEDED of a dependency is not resolved through that
    // dependency's RUNPATH, so anything linking the shim would otherwise
    // have to be told where toppra is as well — and so would the packaging
    // step, and the wheel's repair step. One directory holds the pair, and
    // one `-L` and one rpath cover both.
    let lib = shim.join("lib");
    for entry in std::fs::read_dir(toppra.join("lib")).expect("toppra installed a lib/") {
        let src = entry.expect("readable toppra lib/ entry").path();
        let name = src.file_name().expect("named file");
        if src.is_file() && name.to_string_lossy().starts_with("libtoppra.so") {
            std::fs::copy(&src, lib.join(name))
                .unwrap_or_else(|e| panic!("copy {} beside the shim: {e}", src.display()));
        }
    }
    lib
}

/// The pinned toppra checkout, fetched into `OUT_DIR` unless one was handed
/// over. Shallow, single commit — the tree is ~3 MB.
fn toppra_source(out: &Path) -> PathBuf {
    if let Ok(src) = std::env::var("PAR6_TOPPRA_SRC") {
        return PathBuf::from(src);
    }
    let src = out.join("toppra-src");
    let head = Command::new("git")
        .args(["-C", &src.display().to_string(), "rev-parse", "HEAD"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string());
    if head.as_deref() == Some(TOPPRA_COMMIT) {
        return src;
    }
    let _ = std::fs::remove_dir_all(&src);
    std::fs::create_dir_all(&src).expect("create toppra source dir");
    let dir = src.display().to_string();
    run(Command::new("git").args(["-C", &dir, "init", "-q"]));
    run(Command::new("git").args(["-C", &dir, "remote", "add", "origin", TOPPRA_REPO]));
    run(Command::new("git").args([
        "-C",
        &dir,
        "fetch",
        "-q",
        "--depth",
        "1",
        "origin",
        TOPPRA_COMMIT,
    ]));
    run(Command::new("git").args(["-C", &dir, "checkout", "-q", "--detach", "FETCH_HEAD"]));
    src
}

/// Configure, build and install one cmake project. Ninja because the pixi
/// environment carries no `make`.
fn cmake(src: &Path, build: &Path, install: &Path, prefix_path: &[&str], extra: &[&str]) {
    if !build.join("CMakeCache.txt").exists() {
        let mut cfg = Command::new("cmake");
        cfg.arg("-G")
            .arg("Ninja")
            .arg("-S")
            .arg(src)
            .arg("-B")
            .arg(build)
            .arg("-DCMAKE_BUILD_TYPE=Release")
            .arg(format!("-DCMAKE_PREFIX_PATH={}", prefix_path.join(";")))
            .arg(format!("-DCMAKE_INSTALL_PREFIX={}", install.display()))
            .arg(format!(
                "-DCMAKE_INSTALL_RPATH=$ORIGIN;{}",
                prefix_path
                    .iter()
                    .map(|p| format!("{p}/lib"))
                    .collect::<Vec<_>>()
                    .join(";")
            ))
            .args(extra);
        run(&mut cfg);
    }
    run(Command::new("cmake")
        .arg("--build")
        .arg(build)
        .arg("--parallel")
        .arg(jobs().to_string()));
    run(Command::new("cmake").arg("--install").arg(build));
}

/// Compile jobs: what MemAvailable can hold at [`JOB_MEM_GB`] each, never
/// more than the cores. An explicit `CMAKE_BUILD_PARALLEL_LEVEL` wins.
fn jobs() -> u64 {
    if let Some(n) = std::env::var("CMAKE_BUILD_PARALLEL_LEVEL")
        .ok()
        .and_then(|v| v.parse::<u64>().ok())
        .filter(|n| *n > 0)
    {
        return n;
    }
    let cores = std::thread::available_parallelism().map_or(1, |n| n.get() as u64);
    let by_mem = std::fs::read_to_string("/proc/meminfo")
        .ok()
        .and_then(|s| {
            s.lines()
                .find(|l| l.starts_with("MemAvailable:"))?
                .split_whitespace()
                .nth(1)?
                .parse::<u64>()
                .ok()
        })
        .map(|kb| kb / (JOB_MEM_GB * 1024 * 1024))
        .unwrap_or(cores);
    by_mem.clamp(1, cores)
}

fn run(cmd: &mut Command) {
    let shown = format!("{cmd:?}");
    match cmd.status() {
        Ok(s) if s.success() => {}
        Ok(s) => panic!("{shown} failed with {s}"),
        Err(e) => panic!("{shown} could not be run: {e}"),
    }
}
