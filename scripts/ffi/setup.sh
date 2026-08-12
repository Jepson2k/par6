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
#   scripts/ffi/setup.sh
#   source .ffi/env.sh   # exports PAR6_SHIM_LIB_DIR / PAR6_SHIM_INCLUDE_DIR
#   cargo test --manifest-path crates/pinokin-sys/Cargo.toml --features ffi
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
FFI_DIR="${PAR6_FFI_DIR:-$ROOT/.ffi}"

# Pinned package set. pin (pip) and pinocchio (conda-forge) versions must
# match so scripts/ffi/gen_fixtures.py validates against identical numerics.
PINOCCHIO_VERSION="${PAR6_PINOCCHIO_VERSION:-4.1.0}"
# toppra-cpp source pin (v0.6.9 release commit). MIT; built with the bundled
# Seidel LP solver — no qpOASES/GLPK, so no extra conda deps.
TOPPRA_REPO="${PAR6_TOPPRA_REPO:-https://github.com/hungpham2511/toppra}"
TOPPRA_COMMIT="${PAR6_TOPPRA_COMMIT:-142456f3282c92c93ab97749a24856661924d989}"
# libmujoco (par6-bus feature `sim-mujoco`). Pinned: the hand-rolled FFI
# declarations in crates/par6-bus/src/sim/mujoco.rs are written against this
# version's C API. `libmujoco` is the C library alone (the `mujoco`
# conda-forge package is a metapackage that would drag in python bindings).
MUJOCO_VERSION="${PAR6_MUJOCO_VERSION:-3.10.0}"
CONDA_SPECS=(
  "pinocchio=${PINOCCHIO_VERSION}"
  "eigen"      # constrained by pinocchio's build (5.0.x as of 2026-08)
  "urdfdom"    # constrained by pinocchio's build (6.0.x as of 2026-08)
  # coal (hpp-fcl) backs par6_col_* and already arrives as a pinocchio
  # dependency; naming it keeps the shim's link line honest and makes a
  # future pinocchio build that drops collision support fail here instead
  # of at cmake time. Version is left to pinocchio's constraint (3.0.x).
  "coal"
  "cmake"
  "ninja"
  "cxx-compiler"  # hermetic toolchain — host compiler stays out of the ABI
)

MAMBA="$FFI_DIR/bin/micromamba"
ENV_DIR="$FFI_DIR/env"
SHIM_PREFIX="$FFI_DIR/shim"
BUILD_DIR="$FFI_DIR/build-shim"

mkdir -p "$FFI_DIR"
printf '*\n' > "$FFI_DIR/.gitignore"

# --- 1. micromamba -----------------------------------------------------------
if [[ ! -x "$MAMBA" ]]; then
  echo ">>> installing micromamba into $FFI_DIR/bin"
  arch="$(uname -m)"
  case "$arch" in
    x86_64)  mm_arch=linux-64 ;;
    aarch64) mm_arch=linux-aarch64 ;;
    *) echo "unsupported arch: $arch" >&2; exit 1 ;;
  esac
  curl -Ls "https://micro.mamba.pm/api/micromamba/${mm_arch}/latest" \
    | tar -xj -C "$FFI_DIR" bin/micromamba
fi
echo ">>> micromamba $("$MAMBA" --version)"

# --- 2. conda env with pinocchio --------------------------------------------
if [[ ! -e "$ENV_DIR/lib/libpinocchio_default.so" ]]; then
  echo ">>> creating env: ${CONDA_SPECS[*]}"
  "$MAMBA" create -y -p "$ENV_DIR" -c conda-forge --override-channels \
    "${CONDA_SPECS[@]}"
else
  echo ">>> env exists: $ENV_DIR (delete it to force re-create)"
fi

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

# --- 3. toppra-cpp from source into the env prefix ---------------------------
# No conda-forge C++ toppra exists (only pure-python `toppra-python`), so the
# pinned commit is built here. Installing into $ENV_DIR keeps a single dep
# lib dir (the shim's rpath) and lands inside the CI cache paths.
# Delete $ENV_DIR/lib/libtoppra.so to force a rebuild (e.g. after a pin bump).
TOPPRA_SRC="$FFI_DIR/src/toppra"
TOPPRA_BUILD="$FFI_DIR/build-toppra"
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
  echo ">>> building toppra-cpp"
  "$MAMBA" run -p "$ENV_DIR" cmake -G Ninja -S "$TOPPRA_SRC/cpp" -B "$TOPPRA_BUILD" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$ENV_DIR" \
    -DCMAKE_INSTALL_PREFIX="$ENV_DIR" \
    -DCMAKE_INSTALL_RPATH="$ENV_DIR/lib" \
    -DBUILD_TESTING=OFF \
    -DPYTHON_BINDINGS=OFF \
    -DBUILD_WITH_PINOCCHIO=OFF \
    -DBUILD_WITH_qpOASES=OFF \
    -DBUILD_WITH_GLPK=OFF \
    -DTOPPRA_WARN_ON=OFF
  "$MAMBA" run -p "$ENV_DIR" cmake --build "$TOPPRA_BUILD"
  "$MAMBA" run -p "$ENV_DIR" cmake --install "$TOPPRA_BUILD"
else
  echo ">>> toppra exists: $ENV_DIR/lib/libtoppra.so (delete it to rebuild)"
fi

# --- 3b. libmujoco into the same env prefix ----------------------------------
# Additive to the pinocchio/toppra env; delete $ENV_DIR/lib/libmujoco.so (or
# bump the pin) to force a re-install.
if [[ ! -e "$ENV_DIR/lib/libmujoco.so.${MUJOCO_VERSION}" ]]; then
  echo ">>> installing libmujoco=${MUJOCO_VERSION}"
  "$MAMBA" install -y -p "$ENV_DIR" -c conda-forge --override-channels \
    "libmujoco=${MUJOCO_VERSION}"
else
  echo ">>> libmujoco exists: $ENV_DIR/lib/libmujoco.so.${MUJOCO_VERSION}"
fi

# --- 4. build + install the shim ---------------------------------------------
if [[ "${FORCE:-0}" == "1" ]]; then
  rm -rf "$BUILD_DIR" "$SHIM_PREFIX"
fi
if [[ ! -e "$SHIM_PREFIX/lib/libpar6_shim.so" ]]; then
  echo ">>> building par6_shim"
  # Run cmake/ninja/compiler from inside the env (activation sets CC/CXX).
  "$MAMBA" run -p "$ENV_DIR" cmake -G Ninja -S "$ROOT/cpp" -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$ENV_DIR" \
    -DCMAKE_INSTALL_PREFIX="$SHIM_PREFIX" \
    -DCMAKE_INSTALL_RPATH="$ENV_DIR/lib"
  "$MAMBA" run -p "$ENV_DIR" cmake --build "$BUILD_DIR"
  "$MAMBA" run -p "$ENV_DIR" cmake --install "$BUILD_DIR"
else
  echo ">>> shim exists: $SHIM_PREFIX (FORCE=1 to rebuild)"
fi

# --- 5. env vars for pinokin-sys / par6-bus ----------------------------------
cat > "$FFI_DIR/env.sh" <<EOF
export PAR6_SHIM_LIB_DIR="$SHIM_PREFIX/lib"
export PAR6_SHIM_INCLUDE_DIR="$SHIM_PREFIX/include"
# libmujoco lives in the env prefix (par6-bus feature sim-mujoco).
export PAR6_MUJOCO_LIB_DIR="$ENV_DIR/lib"
# Runtime loading for binaries whose package did not embed an rpath
# (link-args don't propagate across cargo packages). Covers the shim AND
# libmujoco + its conda deps.
export LD_LIBRARY_PATH="$SHIM_PREFIX/lib:$ENV_DIR/lib\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}"
EOF

echo
echo ">>> done. To build/test the Rust FFI crate:"
echo "    source $FFI_DIR/env.sh"
echo "    cargo test --manifest-path $ROOT/crates/pinokin-sys/Cargo.toml --features ffi"
echo
echo ">>> exported by $FFI_DIR/env.sh:"
cat "$FFI_DIR/env.sh"
