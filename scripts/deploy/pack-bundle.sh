#!/usr/bin/env bash
# Pack the release artifacts from a NATIVE build: the daemon bundle a
# control box installs with `tar -x`, its checksums, and the manifest that
# says what went into it.
#
#   pixi run bundle          # -> dist/par6d-<arch>.tar.gz, SHA256SUMS, manifest.json
#
# The bundle carries par6d, the Pinocchio shim, libmujoco and their whole
# runtime closure, because the box gets no conda environment. Everything is
# staged into one flat directory and every object in it is rewritten to
# search `$ORIGIN`, so the set is relocatable to /usr/local/lib/par6.
#
# `scripts/ffi/stage_runtime_libs.py` proves that before anything ships:
# a single glibc floor across the set, no soname the staged copies do not
# provide, and an rpath on every object that depends on another one.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DIST="$ROOT/dist"
STAGE="$ROOT/target/bundle-stage"
ARCH="$(uname -m)"
# Where install.sh puts the closure on the box. Baked into par6d as an rpath
# so the unit needs no LD_LIBRARY_PATH.
RUNTIME_LIB_DIR="${PAR6_RUNTIME_LIB_DIR:-/usr/local/lib/par6}"

die() { echo "pack-bundle: $*" >&2; exit 1; }

BIN="$ROOT/target/release/par6d"
[ -x "$BIN" ] || die "no par6d at $BIN — run \`pixi run build-daemon\` first"
: "${CONDA_PREFIX:?run under pixi}"
command -v patchelf >/dev/null || die "patchelf not found (it is a pixi dependency)"

# The shim and toppra live in cargo's OUT_DIR. par6-kin is the only crate
# that builds them, so there is exactly one of each per profile.
shim_lib="$(echo "$ROOT"/target/release/build/par6-kin-*/out/shim/lib)"
toppra_lib="$(echo "$ROOT"/target/release/build/par6-kin-*/out/toppra/lib)"
[ -e "$shim_lib/libpar6_shim.so" ] \
  || die "no libpar6_shim.so under target/release/build/par6-kin-*/out — a
  release build of par6d must have produced one"

rm -rf "$STAGE"
mkdir -p "$STAGE" "$DIST"

echo ">>> staging the runtime closure"
python3 "$ROOT/scripts/ffi/stage_runtime_libs.py" \
  --readelf "$(command -v readelf)" \
  --lib-dir "$CONDA_PREFIX/lib" \
  --lib-dir "$shim_lib" \
  --lib-dir "$toppra_lib" \
  --dest "$STAGE" \
  --accept-rpath "$RUNTIME_LIB_DIR" \
  "$BIN" \
  "$shim_lib/libpar6_shim.so" \
  "$CONDA_PREFIX/lib/libmujoco.so"

# Every staged object resolves its siblings from its own directory, and the
# binary from the directory install.sh fills. Build-machine paths — the
# conda prefix, cargo's OUT_DIR — mean nothing on the box and must not
# survive into what ships.
echo ">>> rewriting rpaths"
for so in "$STAGE"/*.so*; do patchelf --set-rpath '$ORIGIN' "$so"; done
staged_bin="$STAGE/par6d"
cp "$BIN" "$staged_bin"
patchelf --set-rpath "$RUNTIME_LIB_DIR" "$staged_bin"

BUNDLE="$ROOT/target/bundle"
rm -rf "$BUNDLE"
mkdir -p "$BUNDLE"
"$ROOT/scripts/deploy/install.sh" --stage-only "$BUNDLE" \
  --binary "$staged_bin" --runtime-libs "$STAGE"

# What the deploy job used to assert inline. A bundle missing any of these
# fails on the box, hours later, as a service that will not start.
[ -x "$BUNDLE/par6d" ] || die "no par6d in the bundle"
[ -f "$BUNDLE/lib/libpar6_shim.so" ] || die "no shim in the bundle"
compgen -G "$BUNDLE/lib/libmujoco.so.*" >/dev/null || die "no libmujoco in the bundle"
[ -f "$BUNDLE/config/PAR6.toml" ] || die "no config in the bundle"
[ -d "$BUNDLE/par6_description" ] || die "no assets in the bundle"

tarball="$DIST/par6d-$ARCH.tar.gz"
tar -C "$(dirname "$BUNDLE")" -czf "$tarball" "$(basename "$BUNDLE")"

# The manifest is what ties a published artifact to the commit and the
# versions it was built from, so a box can be asked what it is running and
# a release can be checked against what was validated.
daemon_version="$("$BIN" --version 2>/dev/null | awk '{print $NF}')"
client_version="$(sed -n 's/^version = "\(.*\)"/\1/p' "$ROOT/python/pyproject.toml" | head -1)"
waldoctl_pin="$(sed -n 's#.*waldoctl.git@\([^"]*\).*#\1#p' "$ROOT/python/pyproject.toml" | head -1)"
glibc_floor="$(readelf -V "$staged_bin" 2>/dev/null \
  | grep -oE 'GLIBC_[0-9]+\.[0-9]+' | sort -uV | tail -1)"
for so in "$STAGE"/*.so*; do
  f="$(readelf -V "$so" 2>/dev/null | grep -oE 'GLIBC_[0-9]+\.[0-9]+' | sort -uV | tail -1)"
  [ -n "$f" ] && glibc_floor="$(printf '%s\n%s\n' "$glibc_floor" "$f" | sort -uV | tail -1)"
done
python3 - "$DIST/manifest.json" <<PY
import json, sys
json.dump({
    "commit": "${GITHUB_SHA:-$(git -C "$ROOT" rev-parse HEAD)}",
    "arch": "$ARCH",
    "daemon_version": "$daemon_version",
    "client_version": "$client_version",
    "waldoctl_pin": "$waldoctl_pin",
    "mujoco": "$(basename "$(readlink -f "$CONDA_PREFIX/lib/libmujoco.so")")",
    "glibc_floor": "$glibc_floor",
}, open(sys.argv[1], "w"), indent=2, sort_keys=True)
PY

( cd "$DIST" && sha256sum ./*.tar.gz ./*.whl 2>/dev/null > SHA256SUMS || sha256sum ./*.tar.gz > SHA256SUMS )

echo
echo ">>> $tarball ($(du -h "$tarball" | cut -f1))"
echo ">>> glibc floor: ${glibc_floor:-none}"
cat "$DIST/manifest.json"
