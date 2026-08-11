#!/usr/bin/env bash
# Reproducible C++ FFI toolchain bootstrap for par6:
#   1. installs micromamba into a local prefix (no system changes)
#   2. creates a conda-forge env with Pinocchio + toolchain (pinned)
#   3. builds + installs cpp/ (the par6_shim C-ABI library) against it
#   4. prints/persists the env vars pinokin-sys's build.rs consumes
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
CONDA_SPECS=(
  "pinocchio=${PINOCCHIO_VERSION}"
  "eigen"      # constrained by pinocchio's build (5.0.x as of 2026-08)
  "urdfdom"    # constrained by pinocchio's build (6.0.x as of 2026-08)
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

# --- 3. build + install the shim ---------------------------------------------
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

# --- 4. env vars for pinokin-sys ---------------------------------------------
cat > "$FFI_DIR/env.sh" <<EOF
export PAR6_SHIM_LIB_DIR="$SHIM_PREFIX/lib"
export PAR6_SHIM_INCLUDE_DIR="$SHIM_PREFIX/include"
# Runtime loading for binaries whose package did not embed an rpath
# (link-args don't propagate across cargo packages).
export LD_LIBRARY_PATH="$SHIM_PREFIX/lib\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}"
EOF

echo
echo ">>> done. To build/test the Rust FFI crate:"
echo "    source $FFI_DIR/env.sh"
echo "    cargo test --manifest-path $ROOT/crates/pinokin-sys/Cargo.toml --features ffi"
echo
echo ">>> exported by $FFI_DIR/env.sh:"
cat "$FFI_DIR/env.sh"
