#!/usr/bin/env bash
# Reproducible C++ FFI toolchain bootstrap for par6:
#   1. installs micromamba into a local prefix (no system changes)
#   2. creates a conda-forge env with Pinocchio + toolchain (pinned)
#   3. builds + installs toppra-cpp (pinned commit; no conda-forge package)
#      from source into the same env prefix
#   4. builds + installs cpp/ (the par6_shim C-ABI library) against both
#   5. prints/persists the env vars pinokin-sys's build.rs consumes
#
# Everything lands under $PAR6_FFI_DIR (default: <repo>/.ffi, self-gitignored).
# Idempotent: re-running skips completed steps; FORCE=1 rebuilds the shim.
#
# Usage:
#   scripts/ffi/setup.sh                    # for this machine
#   source .ffi/env.sh   # exports PAR6_SHIM_LIB_DIR / PAR6_SHIM_INCLUDE_DIR
#   cargo test --workspace
#
#   scripts/ffi/setup.sh --target aarch64   # for the control box (RPi 5)
#   source .ffi/env-aarch64.sh
#   scripts/deploy/build-aarch64.sh
#
# CROSS TARGETS. `--target <arch>` builds the whole stack for another
# architecture. conda-forge publishes pinocchio/coal/urdfdom/eigen for
# linux-aarch64 and the matching aarch64 cross compiler for linux-64, so
# nothing has to run on the target to produce its shim: the target env is
# downloaded (never executed) and the compiler comes from the host env.
# `stage_runtime_libs.py` then copies the shim's runtime closure next to
# it and proves the result is loadable on the target's glibc, which is the
# only check available without target hardware.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
FFI_DIR="${PAR6_FFI_DIR:-$ROOT/.ffi}"
HOST_ARCH="$(uname -m)"
TARGET_ARCH="${PAR6_FFI_TARGET_ARCH:-$HOST_ARCH}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --target) TARGET_ARCH="${2:?--target needs an architecture}"; shift 2;;
    -h|--help) sed -n '2,/^set -euo/p' "${BASH_SOURCE[0]}" | sed 's/^# \?//;$d'; exit 0;;
    *) echo "setup.sh: unknown argument $1" >&2; exit 2;;
  esac
done

conda_subdir() {
  case "$1" in
    x86_64)  echo linux-64 ;;
    aarch64) echo linux-aarch64 ;;
    *) echo "unsupported architecture: $1" >&2; return 1 ;;
  esac
}
HOST_SUBDIR="$(conda_subdir "$HOST_ARCH")"
TARGET_SUBDIR="$(conda_subdir "$TARGET_ARCH")"
CROSS=0
[[ "$TARGET_ARCH" != "$HOST_ARCH" ]] && CROSS=1

# RSS one compile job of the Pinocchio/coal translation units needs: 3.9 GB
# measured on the control box (cgroup memory.peak, -j1, 2026-09). Overcommitting
# this on a swapless host livelocks it, so the default parallelism is what
# MemAvailable can hold; an explicit CMAKE_BUILD_PARALLEL_LEVEL still wins.
JOB_MEM_GB="${PAR6_JOB_MEM_GB:-4}"
if [[ -z "${CMAKE_BUILD_PARALLEL_LEVEL:-}" ]]; then
  mem_jobs=$(awk -v g="$JOB_MEM_GB" '/MemAvailable/ { print int($2 / (g * 1024 * 1024)) }' /proc/meminfo 2>/dev/null || true)
  cpu_jobs="$(nproc)"
  jobs=$(( ${mem_jobs:-$cpu_jobs} < cpu_jobs ? ${mem_jobs:-$cpu_jobs} : cpu_jobs ))
  (( jobs >= 1 )) || jobs=1
  export CMAKE_BUILD_PARALLEL_LEVEL="$jobs"
  echo ">>> build parallelism: $jobs jobs (RAM-capped; override with CMAKE_BUILD_PARALLEL_LEVEL)"
fi

# Pinned package set. Every library the shim's numerics come from is named
# with a version: leaving one to the solver means two machines building from
# the same commit get different numerics, and the CI cache then holds
# whichever solve happened first.
PINOCCHIO_VERSION="${PAR6_PINOCCHIO_VERSION:-4.1.0}"
EIGEN_VERSION="${PAR6_EIGEN_VERSION:-5.0.1}"
URDFDOM_VERSION="${PAR6_URDFDOM_VERSION:-6.0.1}"
COAL_VERSION="${PAR6_COAL_VERSION:-3.0.4}"
# toppra-cpp source pin (v0.6.9 release commit). MIT; built with the bundled
# Seidel LP solver — no qpOASES/GLPK, so no extra conda deps.
TOPPRA_REPO="${PAR6_TOPPRA_REPO:-https://github.com/hungpham2511/toppra}"
TOPPRA_COMMIT="${PAR6_TOPPRA_COMMIT:-142456f3282c92c93ab97749a24856661924d989}"
# libmujoco (par6-bus feature `sim-mujoco`). Pinned: the hand-rolled FFI
# declarations in crates/par6-bus/src/sim/mujoco.rs are written against this
# version's C API. `libmujoco` is the C library alone (the `mujoco`
# conda-forge package is a metapackage that would drag in python bindings).
# Host-side developer tooling only — a cross target never gets it.
MUJOCO_VERSION="${PAR6_MUJOCO_VERSION:-3.10.0}"
# Cross sysroot pin. conda-forge builds its own linux-aarch64 packages
# against glibc 2.17, so the shim is built against the same floor: the
# staged closure then has a single, lowest-common glibc requirement and
# runs on anything from bullseye upward. Raising this raises the floor.
CROSS_SYSROOT_VERSION="${PAR6_CROSS_SYSROOT_VERSION:-2.17}"

# Packages the shim links against — the ones a cross target also needs.
TARGET_SPECS=(
  "pinocchio=${PINOCCHIO_VERSION}"
  # toppra links Eigen, so its parameterization moves with this version.
  "eigen=${EIGEN_VERSION}"
  "urdfdom=${URDFDOM_VERSION}"
  # coal (hpp-fcl) backs par6_col_* and already arrives as a pinocchio
  # dependency; naming it keeps the shim's link line honest and makes a
  # future pinocchio build that drops collision support fail here instead
  # of at cmake time.
  "coal=${COAL_VERSION}"
)
# Build tools. Native: same env as the libraries (activation sets CC/CXX).
# Cross: a host-platform env holding the target's cross compiler.
if [[ $CROSS -eq 0 ]]; then
  TARGET_SPECS+=("cmake" "ninja" "cxx-compiler")
else
  # The shim is compiled by the cross gcc but loads the target env's
  # libstdc++ at runtime, so that libstdc++ must be at least as new as
  # the compiler's. Naming the runtimes here lets the solver take the
  # current one instead of whatever pinocchio's floor happens to be.
  TARGET_SPECS+=("libstdcxx-ng" "libgcc-ng")
  TOOLCHAIN_SPECS=(
    "gxx_linux-${TARGET_ARCH}"
    "sysroot_linux-${TARGET_ARCH}=${CROSS_SYSROOT_VERSION}"
    "cmake"
    "ninja"
    # The conda cross gcc bakes its own prefix into every artifact's rpath.
    # That path does not exist on the control box (harmless) but it is a
    # build-machine path inside a shipped binary, so it gets rewritten.
    "patchelf"
  )
fi

MAMBA="$FFI_DIR/bin/micromamba"
if [[ $CROSS -eq 0 ]]; then
  ENV_DIR="$FFI_DIR/env"
  SHIM_PREFIX="$FFI_DIR/shim"
  BUILD_DIR="$FFI_DIR/build-shim"
  TOPPRA_BUILD="$FFI_DIR/build-toppra"
  ENV_FILE="$FFI_DIR/env.sh"
  TOOLCHAIN_DIR="$ENV_DIR"
else
  ARCH_DIR="$FFI_DIR/cross-$TARGET_ARCH"
  ENV_DIR="$ARCH_DIR/env"
  SHIM_PREFIX="$ARCH_DIR/shim"
  BUILD_DIR="$ARCH_DIR/build-shim"
  TOPPRA_BUILD="$ARCH_DIR/build-toppra"
  ENV_FILE="$FFI_DIR/env-$TARGET_ARCH.sh"
  TOOLCHAIN_DIR="$ARCH_DIR/toolchain"
  TOOLCHAIN_FILE="$ARCH_DIR/toolchain.cmake"
  CROSS_PREFIX="$TARGET_ARCH-conda-linux-gnu"
fi
TOPPRA_SRC="$FFI_DIR/src/toppra"

mkdir -p "$FFI_DIR"
printf '*\n' > "$FFI_DIR/.gitignore"

echo ">>> target: $TARGET_ARCH ($TARGET_SUBDIR); host: $HOST_ARCH ($HOST_SUBDIR)"

# --- 1. micromamba -----------------------------------------------------------
if [[ ! -x "$MAMBA" ]]; then
  echo ">>> installing micromamba into $FFI_DIR/bin"
  curl -Ls "https://micro.mamba.pm/api/micromamba/${HOST_SUBDIR}/latest" \
    | tar -xj -C "$FFI_DIR" bin/micromamba
fi
echo ">>> micromamba $("$MAMBA" --version)"

# --- 2. conda env with pinocchio --------------------------------------------
# For a cross target `--platform` downloads the target's packages without
# ever running one; nothing in this env is executable on the host.
if [[ ! -e "$ENV_DIR/lib/libpinocchio_default.so" ]]; then
  echo ">>> creating $TARGET_SUBDIR env: ${TARGET_SPECS[*]}"
  "$MAMBA" create -y -p "$ENV_DIR" --platform "$TARGET_SUBDIR" \
    -c conda-forge --override-channels "${TARGET_SPECS[@]}"
else
  echo ">>> env exists: $ENV_DIR (delete it to force re-create)"
fi

# An env restored from a cache is trusted for everything below, so check it
# is the env this script asks for. Nothing else notices the difference: the
# shim links, the tests run, and a few numerical results move — which is how
# a CI cache built from one solve keeps passing while a fresh machine
# building from the same commit fails.
for spec in "pinocchio=$PINOCCHIO_VERSION" "eigen=$EIGEN_VERSION" \
            "urdfdom=$URDFDOM_VERSION" "coal=$COAL_VERSION"; do
  pkg="${spec%%=*}"
  want="${spec#*=}"
  meta=("$ENV_DIR"/conda-meta/"$pkg"-[0-9]*.json)
  got=""
  if [[ -e "${meta[0]}" ]]; then
    got="$(basename "${meta[0]}")"
    got="${got#"$pkg"-}"
    got="${got%%-*}"
  fi
  if [[ "$got" != "$want" ]]; then
    echo "$ENV_DIR has $pkg ${got:-<missing>}, pinned at $want." >&2
    echo "Delete $ENV_DIR and re-run to rebuild the env." >&2
    exit 1
  fi
done

# cpp/src/par6_col.cpp links coal and pinocchio's collision module; both come
# with the pinocchio package, so a missing one means an env built before coal
# was required (or a pinocchio build without collision support).
for lib in libcoal.so libpinocchio_collision.so; do
  if [[ ! -e "$ENV_DIR/lib/$lib" ]]; then
    echo "$ENV_DIR/lib/$lib is missing — the collision shim cannot link." >&2
    echo "Delete $ENV_DIR and re-run to rebuild the env." >&2
    exit 1
  fi
done

# --- 2b. cross toolchain + cmake toolchain file ------------------------------
if [[ $CROSS -eq 1 ]]; then
  # Both guards, so an env created before a spec was added to the list is
  # rebuilt rather than silently missing the new tool.
  if [[ ! -x "$TOOLCHAIN_DIR/bin/$CROSS_PREFIX-g++" || ! -x "$TOOLCHAIN_DIR/bin/patchelf" ]]; then
    echo ">>> creating $HOST_SUBDIR toolchain env: ${TOOLCHAIN_SPECS[*]}"
    "$MAMBA" create -y -p "$TOOLCHAIN_DIR" --platform "$HOST_SUBDIR" \
      -c conda-forge --override-channels "${TOOLCHAIN_SPECS[@]}"
  else
    echo ">>> toolchain exists: $TOOLCHAIN_DIR (delete it to force re-create)"
  fi
  SYSROOT="$TOOLCHAIN_DIR/$CROSS_PREFIX/sysroot"
  [[ -d "$SYSROOT" ]] || { echo "cross sysroot missing: $SYSROOT" >&2; exit 1; }
  # Rewritten every run: it only encodes paths already computed above, so
  # a moved checkout self-heals instead of failing at link time.
  cat > "$TOOLCHAIN_FILE" <<EOF
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR $TARGET_ARCH)
set(CMAKE_C_COMPILER   "$TOOLCHAIN_DIR/bin/$CROSS_PREFIX-gcc")
set(CMAKE_CXX_COMPILER "$TOOLCHAIN_DIR/bin/$CROSS_PREFIX-g++")
set(CMAKE_SYSROOT      "$SYSROOT")
# Libraries and headers come from the target env; programs never do —
# nothing in it can execute on this host.
set(CMAKE_FIND_ROOT_PATH "$ENV_DIR" "$SYSROOT")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
EOF
  echo ">>> cmake toolchain: $TOOLCHAIN_FILE"
fi

# `cmake`/`ninja` and the compiler always come from a HOST-platform env; on a
# native build that is the same env the libraries live in.
run_tool() { "$MAMBA" run -p "$TOOLCHAIN_DIR" "$@"; }
cmake_cross_args=()
if [[ $CROSS -eq 1 ]]; then
  cmake_cross_args=(-DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE")
fi
# Native builds keep their absolute dependency rpath (the env never moves);
# cross builds get $ORIGIN, because the staged closure and the shim sit in
# one directory both here and at /usr/local/lib/par6 on the box.
if [[ $CROSS -eq 1 ]]; then
  DEP_RPATH='$ORIGIN'
else
  DEP_RPATH="$ENV_DIR/lib"
fi

# --- 3. toppra-cpp from source into the env prefix ---------------------------
# No conda-forge C++ toppra exists (only pure-python `toppra-python`), so the
# pinned commit is built here. Installing into $ENV_DIR keeps a single dep
# lib dir (the shim's rpath) and lands inside the CI cache paths.
# Delete $ENV_DIR/lib/libtoppra.so to force a rebuild (e.g. after a pin bump).
if [[ ! -e "$ENV_DIR/lib/libtoppra.so" ]]; then
  if [[ "$(git -C "$TOPPRA_SRC" rev-parse HEAD 2>/dev/null)" != "$TOPPRA_COMMIT" ]]; then
    echo ">>> fetching toppra @ $TOPPRA_COMMIT"
    rm -rf "$TOPPRA_SRC"
    mkdir -p "$TOPPRA_SRC"
    git -C "$TOPPRA_SRC" init -q
    git -C "$TOPPRA_SRC" remote add origin "$TOPPRA_REPO"
    git -C "$TOPPRA_SRC" fetch -q --depth 1 origin "$TOPPRA_COMMIT"
    git -C "$TOPPRA_SRC" checkout -q --detach FETCH_HEAD
  fi
  echo ">>> building toppra-cpp for $TARGET_ARCH"
  run_tool cmake -G Ninja -S "$TOPPRA_SRC/cpp" -B "$TOPPRA_BUILD" \
    "${cmake_cross_args[@]}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$ENV_DIR" \
    -DCMAKE_INSTALL_PREFIX="$ENV_DIR" \
    -DCMAKE_INSTALL_RPATH="$DEP_RPATH" \
    -DBUILD_TESTING=OFF \
    -DPYTHON_BINDINGS=OFF \
    -DBUILD_WITH_PINOCCHIO=OFF \
    -DBUILD_WITH_qpOASES=OFF \
    -DBUILD_WITH_GLPK=OFF \
    -DTOPPRA_WARN_ON=OFF
  run_tool cmake --build "$TOPPRA_BUILD"
  run_tool cmake --install "$TOPPRA_BUILD"
else
  echo ">>> toppra exists: $ENV_DIR/lib/libtoppra.so (delete it to rebuild)"
fi

# --- 3b. libmujoco into the same env prefix ----------------------------------
# Additive to the pinocchio/toppra env; delete $ENV_DIR/lib/libmujoco.so (or
# bump the pin) to force a re-install. `sim-mujoco` is a host-side simulator
# plant, never deployed, so a cross target skips it.
if [[ $CROSS -eq 0 ]]; then
  if [[ ! -e "$ENV_DIR/lib/libmujoco.so.${MUJOCO_VERSION}" ]]; then
    echo ">>> installing libmujoco=${MUJOCO_VERSION}"
    "$MAMBA" install -y -p "$ENV_DIR" -c conda-forge --override-channels \
      "libmujoco=${MUJOCO_VERSION}"
  else
    echo ">>> libmujoco exists: $ENV_DIR/lib/libmujoco.so.${MUJOCO_VERSION}"
  fi
fi

# --- 4. build + install the shim ---------------------------------------------
if [[ "${FORCE:-0}" == "1" ]]; then
  rm -rf "$BUILD_DIR" "$SHIM_PREFIX"
fi
# Rebuild when cpp/ has moved on: cargo does not build the shim, so a stale
# .so links silently and surfaces as wrong numbers in the kinematics tests.
if [[ -e "$SHIM_PREFIX/lib/libpar6_shim.so" ]]; then
  if [[ -n "$(find "$ROOT/cpp" -type f -newer "$SHIM_PREFIX/lib/libpar6_shim.so" -print -quit)" ]]; then
    echo ">>> cpp/ is newer than the installed shim; rebuilding"
    rm -rf "$BUILD_DIR" "$SHIM_PREFIX"
  fi
fi
if [[ ! -e "$SHIM_PREFIX/lib/libpar6_shim.so" ]]; then
  echo ">>> building par6_shim for $TARGET_ARCH"
  run_tool cmake -G Ninja -S "$ROOT/cpp" -B "$BUILD_DIR" \
    "${cmake_cross_args[@]}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$ENV_DIR" \
    -DCMAKE_INSTALL_PREFIX="$SHIM_PREFIX" \
    -DCMAKE_INSTALL_RPATH="$DEP_RPATH"
  run_tool cmake --build "$BUILD_DIR"
  run_tool cmake --install "$BUILD_DIR"
else
  echo ">>> shim exists and is newer than cpp/: $SHIM_PREFIX (FORCE=1 to rebuild)"
fi

# --- 4b. drop the build machine out of the artifacts we produce --------------
# Only ours: the conda libraries' own $ORIGIN-relative rpaths already resolve
# in a flat directory, and rewriting vendor binaries would be a change for no
# gain (stage_runtime_libs.py checks that they do).
if [[ $CROSS -eq 1 ]]; then
  for artifact in "$SHIM_PREFIX/lib/libpar6_shim.so" "$ENV_DIR/lib/libtoppra.so"; do
    "$TOOLCHAIN_DIR/bin/patchelf" --set-rpath '$ORIGIN' "$artifact"
  done
fi

# --- 4c. stage the runtime closure next to the cross shim --------------------
# The control box gets no conda env, so everything the shim dlopens has to
# ship with it. Staging into the shim's own lib dir makes the link-time
# layout and the on-box layout the same directory, which is what makes a
# plain `$ORIGIN` rpath correct in both places.
if [[ $CROSS -eq 1 ]]; then
  echo ">>> staging runtime closure"
  python3 "$ROOT/scripts/ffi/stage_runtime_libs.py" \
    --readelf "$TOOLCHAIN_DIR/bin/$CROSS_PREFIX-readelf" \
    --lib-dir "$ENV_DIR/lib" \
    --dest "$SHIM_PREFIX/lib" \
    "$SHIM_PREFIX/lib/libpar6_shim.so"
fi

# --- 5. env vars for pinokin-sys / par6-bus ----------------------------------
{
  echo "export PAR6_SHIM_LIB_DIR=\"$SHIM_PREFIX/lib\""
  echo "export PAR6_SHIM_INCLUDE_DIR=\"$SHIM_PREFIX/include\""
  sed "s/JOB_MEM_GB_PLACEHOLDER/$JOB_MEM_GB/" <<'JOBS'
# RAM-capped default build parallelism, computed each time this file is
# sourced, at JOB_MEM_GB per job (the measured peak of one shim compile;
# rustc stays well under it). A swapless small-RAM host that overcommits
# this livelocks in reclaim instead of OOM-killing. Explicit values win.
if [ -z "${CARGO_BUILD_JOBS:-}" ] || [ -z "${CMAKE_BUILD_PARALLEL_LEVEL:-}" ]; then
  _par6_cores="$(nproc 2>/dev/null || echo 1)"
  _par6_jobs="$(awk -v g="${PAR6_JOB_MEM_GB:-JOB_MEM_GB_PLACEHOLDER}" '/MemAvailable/ { print int($2 / (g * 1024 * 1024)) }' /proc/meminfo 2>/dev/null || true)"
  [ -n "${_par6_jobs:-}" ] || _par6_jobs="$_par6_cores"
  [ "$_par6_jobs" -ge 1 ] || _par6_jobs=1
  [ "$_par6_jobs" -le "$_par6_cores" ] || _par6_jobs="$_par6_cores"
  export CARGO_BUILD_JOBS="${CARGO_BUILD_JOBS:-$_par6_jobs}"
  export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-$_par6_jobs}"
  unset _par6_jobs _par6_cores
fi
JOBS
  if [[ $CROSS -eq 0 ]]; then
    echo "# libmujoco lives in the env prefix (par6-bus feature sim-mujoco)."
    echo "export PAR6_MUJOCO_LIB_DIR=\"$ENV_DIR/lib\""
    echo "# Runtime loading for binaries whose package did not embed an rpath"
    echo "# (link-args don't propagate across cargo packages). Covers the shim AND"
    echo "# libmujoco + its conda deps."
    echo "export LD_LIBRARY_PATH=\"$SHIM_PREFIX/lib:$ENV_DIR/lib\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}\""
  else
    echo "# Cross target: nothing here runs on this host, so no LD_LIBRARY_PATH."
    echo "export PAR6_FFI_TARGET_ARCH=\"$TARGET_ARCH\""
    echo "# The whole set scripts/deploy/install.sh ships to /usr/local/lib/par6."
    echo "export PAR6_RUNTIME_LIB_SRC=\"$SHIM_PREFIX/lib\""
    echo "# Link par6d with the same cross toolchain the shim was built with, so"
    echo "# the binary and its C++ dependencies agree on glibc and the C++ ABI."
    echo "export CARGO_TARGET_$(echo "${TARGET_ARCH}_UNKNOWN_LINUX_GNU_LINKER" | tr '[:lower:]' '[:upper:]')=\"$TOOLCHAIN_DIR/bin/$CROSS_PREFIX-gcc\""
    echo "export PAR6_CROSS_READELF=\"$TOOLCHAIN_DIR/bin/$CROSS_PREFIX-readelf\""
    echo "export PAR6_CROSS_PATCHELF=\"$TOOLCHAIN_DIR/bin/patchelf\""
  fi
} > "$ENV_FILE"

echo
if [[ $CROSS -eq 0 ]]; then
  echo ">>> done. To build and test the workspace:"
  echo "    source $ENV_FILE"
  echo "    cargo test --workspace"
else
  echo ">>> done. To build the runtime for the control box:"
  echo "    source $ENV_FILE"
  echo "    scripts/deploy/build-aarch64.sh"
fi
echo
echo ">>> exported by $ENV_FILE:"
cat "$ENV_FILE"
