#!/usr/bin/env bash
# Cross-build par6d for the PAR6 control box (Raspberry Pi 5, aarch64).
#
# Toolchain is deliberately the same one CI's aarch64 matrix leg uses
# (.github/workflows/ci.yml): the `aarch64-unknown-linux-gnu` rustup target
# plus Debian/Ubuntu's `gcc-aarch64-linux-gnu` as the linker. CI only runs
# `cargo check` for that target; this script is the first place the link
# step actually happens, so a missing cross libc shows up here, not there.
#
# Usage:
#   scripts/deploy/build-aarch64.sh              # release build
#   PAR6_CARGO_FLAGS="--features ffi" scripts/deploy/build-aarch64.sh
#
# NOTE on `--features ffi`: that feature links the Pinocchio C-ABI shim, and
# scripts/ffi/setup.sh only builds an x86_64 shim. Passing it here needs an
# aarch64 shim + Pinocchio built first; see scripts/deploy/README.md.
set -euo pipefail

TARGET="aarch64-unknown-linux-gnu"
LINKER="${PAR6_CROSS_LINKER:-aarch64-linux-gnu-gcc}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

die() { echo "build-aarch64: $*" >&2; exit 1; }

command -v cargo >/dev/null || die "cargo not found"
command -v "$LINKER" >/dev/null || die \
  "cross linker '$LINKER' not found — install it with:
    sudo apt-get install -y gcc-aarch64-linux-gnu"

if ! rustup target list --installed 2>/dev/null | grep -qx "$TARGET"; then
  echo "build-aarch64: adding rustup target $TARGET"
  rustup target add "$TARGET"
fi

# Same env var CI sets for the cross-check leg.
export CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER="$LINKER"

cd "$ROOT"
# shellcheck disable=SC2086  # PAR6_CARGO_FLAGS is intentionally word-split
cargo build -p par6d --release --target "$TARGET" ${PAR6_CARGO_FLAGS:-}

BIN="$ROOT/target/$TARGET/release/par6d"
[ -x "$BIN" ] || die "expected binary missing: $BIN"

echo
echo "built: $BIN"
file "$BIN" 2>/dev/null || true
ls -lh "$BIN" | awk '{print "size:  " $5}'
echo
echo "next: scripts/deploy/install.sh --host <user>@<control-box>"
