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

# The shim lives in cargo's OUT_DIR. Cargo keeps a build directory per build
# script *invocation*, not per crate, so a tree that has been built more than
# one way holds several — the newest is the one the binary beside it links.
shim_lib="$(ls -1dt "$ROOT"/target/release/build/par6-kin-*/out/shim/lib 2>/dev/null | head -1)"
[ -n "$shim_lib" ] && [ -e "$shim_lib/libpar6_shim.so" ] \
  || die "no libpar6_shim.so under target/release/build/par6-kin-*/out — a
  release build of par6d must have produced one"
[ -e "$shim_lib/libtoppra.so" ] \
  || die "libtoppra.so is not beside the shim in $shim_lib; par6-kin's build
  script installs it there so one rpath covers the pair"

rm -rf "$STAGE"
mkdir -p "$STAGE" "$DIST"

# The binary is rewritten BEFORE the closure is checked, not after: the check
# asks whether each object can find its staged siblings once the directory
# moves, and an answer taken from the build tree's rpath is not the one that
# ships. par6d searches the directory install.sh fills; every library
# searches its own.
# The roots go in by hand. `stage_runtime_libs.py` closes over what its
# roots NEED and copies that; a root is the thing being closed over, so it
# is never its own dependency and never lands there on its own.
cp "$BIN" "$STAGE/par6d"
cp "$shim_lib/libpar6_shim.so" "$shim_lib/libtoppra.so" "$STAGE/"
mujoco="$(readlink -f "$CONDA_PREFIX/lib/libmujoco.so")"
cp "$mujoco" "$STAGE/$(basename "$mujoco")"
patchelf --set-rpath "$RUNTIME_LIB_DIR" "$STAGE/par6d"

echo ">>> staging the runtime closure"
python3 "$ROOT/scripts/ffi/stage_runtime_libs.py" \
  --readelf "$(command -v readelf)" \
  --lib-dir "$CONDA_PREFIX/lib" \
  --lib-dir "$shim_lib" \
  --lib-dir "$STAGE" \
  --dest "$STAGE" \
  --accept-rpath "$RUNTIME_LIB_DIR" \
  "$STAGE/par6d" \
  "$STAGE/libpar6_shim.so" \
  "$STAGE/$(basename "$mujoco")"

# The staged libraries pass the check on an `$ORIGIN` entry they already
# carry, but ours also name the prefix they were built against. That path
# does not exist on the box, and a build-machine path inside a shipped binary
# is the thing `validate-bundle.sh` refuses, so it is trimmed away here.
echo ">>> trimming build-machine paths"
for so in "$STAGE"/*.so*; do patchelf --set-rpath '$ORIGIN' "$so"; done
staged_bin="$STAGE/par6d"

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
daemon_version="$(sed -n '/^\[workspace.package\]/,/^\[/s/^version = "\(.*\)"/\1/p' "$ROOT/Cargo.toml" | head -1)"
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
