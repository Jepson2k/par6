#!/usr/bin/env bash
# Cross-build par6d for the PAR6 control box (Raspberry Pi 5, aarch64).
#
# par6d links the Pinocchio shim unconditionally: without it there is no TCP
# pose, no cartesian motion and no collision world, so the cross-build needs
# the aarch64 shim on hand before it starts.
#
# Usage:
#   scripts/ffi/setup.sh --target aarch64      # once (or after a pin bump)
#   source .ffi/env-aarch64.sh
#   scripts/deploy/build-aarch64.sh
#   # -> target/aarch64-unknown-linux-gnu/release/par6d
#
# The linker and the shim both come from that env file: par6d is linked with
# the same conda cross toolchain that built libpar6_shim.so, so the binary
# and the C++ libraries shipped beside it agree on glibc and the C++ ABI.
# PAR6_CARGO_FLAGS adds extra cargo flags.
set -euo pipefail

TARGET="aarch64-unknown-linux-gnu"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# Where install.sh puts the shim and its dependency closure on the box.
# Baked into the binary as an rpath so the unit needs no LD_LIBRARY_PATH.
RUNTIME_LIB_DIR="${PAR6_RUNTIME_LIB_DIR:-/usr/local/lib/par6}"

die() { echo "build-aarch64: $*" >&2; exit 1; }

command -v cargo >/dev/null || die "cargo not found"

[ -n "${PAR6_SHIM_LIB_DIR:-}" ] || die "PAR6_SHIM_LIB_DIR is not set — either the
  aarch64 Pinocchio shim has not been built or its env file was not sourced:
    scripts/ffi/setup.sh --target aarch64
    source .ffi/env-aarch64.sh"
[ "${PAR6_FFI_TARGET_ARCH:-}" = "aarch64" ] || die \
  "PAR6_SHIM_LIB_DIR points at a ${PAR6_FFI_TARGET_ARCH:-host} shim, not an aarch64 one.
  Source .ffi/env-aarch64.sh (not .ffi/env.sh)."
[ -e "$PAR6_SHIM_LIB_DIR/libpar6_shim.so" ] \
  || die "no libpar6_shim.so in PAR6_SHIM_LIB_DIR ($PAR6_SHIM_LIB_DIR)"

LINKER="${CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER:-}"
[ -n "$LINKER" ] || die "CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER is not set
  — source .ffi/env-aarch64.sh"
command -v "$LINKER" >/dev/null || die "cross linker not found: $LINKER"

if ! rustup target list --installed 2>/dev/null | grep -qx "$TARGET"; then
  echo "build-aarch64: adding rustup target $TARGET"
  rustup target add "$TARGET"
fi

cd "$ROOT"
# The shim's own rpath is $ORIGIN, so pointing the binary at the install
# directory resolves the whole closure from one entry.
export RUSTFLAGS="${RUSTFLAGS:-} -C link-arg=-Wl,-rpath,$RUNTIME_LIB_DIR"
# shellcheck disable=SC2086  # PAR6_CARGO_FLAGS is intentionally word-split
cargo build -p par6d --release --target "$TARGET" ${PAR6_CARGO_FLAGS:-}

BIN="$ROOT/target/$TARGET/release/par6d"
[ -x "$BIN" ] || die "expected binary missing: $BIN"

# The conda cross gcc appends its own prefix to the rpath. It is a
# build-machine path that means nothing on the box, so the shipped binary
# carries only the directory install.sh actually fills.
if [ -n "${PAR6_CROSS_PATCHELF:-}" ] && [ -x "$PAR6_CROSS_PATCHELF" ]; then
  "$PAR6_CROSS_PATCHELF" --set-rpath "$RUNTIME_LIB_DIR" "$BIN"
fi

echo
echo "built: $BIN"
file "$BIN" 2>/dev/null || true
ls -lh "$BIN" | awk '{print "size:  " $5}'

# The glibc floor is the one property of a cross build that silently breaks
# on an older box, and it is measurable here.
READELF="${PAR6_CROSS_READELF:-}"
if [ -n "$READELF" ] && command -v "$READELF" >/dev/null; then
  floor="$("$READELF" -V "$BIN" 2>/dev/null \
    | grep -oE 'GLIBC_[0-9]+\.[0-9]+' | sort -uV | tail -1)"
  echo "glibc: needs at most ${floor:-none} (Raspberry Pi OS bookworm ships 2.36)"
fi
echo "runtime libs: $PAR6_SHIM_LIB_DIR -> $RUNTIME_LIB_DIR on the box"
echo
echo "next: scripts/deploy/install.sh --host <user>@<control-box>"
