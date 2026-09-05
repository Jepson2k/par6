#!/usr/bin/env bash
# Build toppra-cpp and the par6 C++ shim under the pixi environment.
#
# pixi owns the dependency closure (pinocchio, coal, eigen, urdfdom, the
# compiler); this script is the par6-specific glue that stays: two cmake builds
# into our own prefixes under .ffi/, kept out of $CONDA_PREFIX so that
# `pixi install` can never clobber them.
#
#   pixi run build-toppra   # or:  bash scripts/ffi/build.sh toppra
#   pixi run build-shim     #       bash scripts/ffi/build.sh shim
set -euo pipefail

ROOT="${PIXI_PROJECT_ROOT:?run under pixi}"
PREFIX="${CONDA_PREFIX:?run under pixi}"
FFI="$ROOT/.ffi"
TOPPRA_PREFIX="$FFI/toppra"
SHIM_PREFIX="$FFI/shim"

build_toppra() {
  if [[ -e "$TOPPRA_PREFIX/lib/libtoppra.so" ]]; then
    echo ">>> toppra built: $TOPPRA_PREFIX (rm -rf it to rebuild)"
    return
  fi
  echo ">>> building toppra-cpp"
  cmake -G Ninja -S "$FFI/src/toppra/cpp" -B "$FFI/build/toppra" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$PREFIX" \
    -DCMAKE_INSTALL_PREFIX="$TOPPRA_PREFIX" \
    -DCMAKE_INSTALL_RPATH="$PREFIX/lib" \
    -DBUILD_TESTING=OFF \
    -DPYTHON_BINDINGS=OFF \
    -DBUILD_WITH_PINOCCHIO=OFF \
    -DBUILD_WITH_qpOASES=OFF \
    -DBUILD_WITH_GLPK=OFF \
    -DTOPPRA_WARN_ON=OFF
  cmake --build "$FFI/build/toppra"
  cmake --install "$FFI/build/toppra"
}

build_shim() {
  if [[ "${FORCE:-0}" == "1" ]]; then
    rm -rf "$FFI/build/shim" "$SHIM_PREFIX"
  fi
  echo ">>> building par6_shim"
  cmake -G Ninja -S "$ROOT/cpp" -B "$FFI/build/shim" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$PREFIX;$TOPPRA_PREFIX" \
    -DCMAKE_INSTALL_PREFIX="$SHIM_PREFIX" \
    -DCMAKE_INSTALL_RPATH="$PREFIX/lib;$TOPPRA_PREFIX/lib"
  cmake --build "$FFI/build/shim"
  cmake --install "$FFI/build/shim"
}

case "${1:-all}" in
  toppra) build_toppra ;;
  shim)   build_shim ;;
  all)    build_toppra; build_shim ;;
  *) echo "usage: $0 [toppra|shim|all]" >&2; exit 2 ;;
esac
