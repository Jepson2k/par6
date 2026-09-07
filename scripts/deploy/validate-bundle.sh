#!/usr/bin/env bash
# Exercise the SHIPPED artifacts with none of the build environment present.
#
#   scripts/deploy/validate-bundle.sh dist/
#
# Everything a build machine has and a control box does not — the pixi
# environment, cargo's OUT_DIR, .ffi, LD_LIBRARY_PATH — is scrubbed here.
# What is left is what a box would have: the tarball, a stock python, and
# the glibc of the machine. If the bundle depends on anything else, it fails
# here rather than on the arm.
#
# Checks, in the order a box would hit them:
#   1. the checksums the release publishes match the files
#   2. the closure's glibc floor clears the oldest supported Raspberry Pi OS
#   3. no shipped object searches a build-machine path
#   4. par6d starts, simulates, and answers on the wire
#   5. the kinematics and collision engine gives the same answers it does in
#      the workspace suite — the shim is loaded and functional, not merely
#      resolvable
#   6. the wheel imports and computes in a bare venv
set -euo pipefail

DIST="$(cd "${1:?usage: validate-bundle.sh <dist dir>}" && pwd)"
WORK="${PAR6_VALIDATE_DIR:-/tmp/par6-validate}"
# Raspberry Pi OS bookworm ships glibc 2.36; bullseye 2.31. The floor the
# control box has to clear is the oldest release the arm is supported on.
FLOOR="${PAR6_GLIBC_FLOOR:-2.36}"

die() { echo "validate: $*" >&2; exit 1; }
say() { echo; echo "=== $*"; }

# A build machine's environment must not leak into anything below.
unset LD_LIBRARY_PATH CONDA_PREFIX PAR6_SHIM_LIB_DIR PAR6_SHIM_INCLUDE_DIR \
      MUJOCO_DYNAMIC_LINK_DIR MUJOCO_DOWNLOAD_DIR PAR6_RUNTIME_LIB_SRC \
      PIXI_PROJECT_ROOT CMAKE_PREFIX_PATH || true

rm -rf "$WORK"; mkdir -p "$WORK"
cd "$WORK"

say "1. checksums"
[ -f "$DIST/SHA256SUMS" ] || die "no SHA256SUMS in $DIST"
( cd "$DIST" && sha256sum -c SHA256SUMS ) || die "checksum mismatch"
[ -f "$DIST/manifest.json" ] || die "no manifest.json in $DIST"
cat "$DIST/manifest.json"

tarball="$(echo "$DIST"/par6d-*.tar.gz)"
[ -f "$tarball" ] || die "no daemon bundle in $DIST"
tar -xzf "$tarball"
BUNDLE="$WORK/bundle"
[ -x "$BUNDLE/par6d" ] || die "no par6d in the unpacked bundle"

say "2. glibc floor (must be <= $FLOOR)"
worst=""
for obj in "$BUNDLE/par6d" "$BUNDLE"/lib/*.so*; do
  v="$(readelf -V "$obj" 2>/dev/null | grep -oE 'GLIBC_[0-9]+\.[0-9]+' | sort -uV | tail -1 || true)"
  [ -n "$v" ] || continue
  worst="$(printf '%s\n%s\n' "$worst" "${v#GLIBC_}" | sort -uV | tail -1)"
done
echo "closure needs at most glibc ${worst:-none}"
[ -n "$worst" ] || die "could not read a glibc requirement from the closure"
[ "$(printf '%s\n%s\n' "$worst" "$FLOOR" | sort -V | tail -1)" = "$FLOOR" ] \
  || die "the bundle needs glibc $worst but the control box has $FLOOR.
  A native build on a newer runner cannot ship without an older sysroot;
  the cross build (scripts/ffi/setup.sh --target aarch64) is what holds
  this floor today."

say "3. no build-machine paths in what ships"
for obj in "$BUNDLE/par6d" "$BUNDLE"/lib/*.so*; do
  rp="$(readelf -d "$obj" 2>/dev/null | sed -n 's/.*R\(UN\)\?PATH.*\[\(.*\)\]/\2/p' || true)"
  case "$rp" in
    ''|'$ORIGIN'|/usr/local/lib/par6) ;;
    *'$ORIGIN'*) ;;
    *) die "$(basename "$obj") searches $rp — a path that does not exist on the box";;
  esac
done
echo "rpaths: \$ORIGIN / the install directory only"

say "4. the daemon installs the way a box installs it"
# install.sh's own packaging half, not a hand-rolled copy: a bundle that
# installs here is one that installs on the arm.
sudo "$BUNDLE/install.sh" --local --bundle "$BUNDLE" --no-restart
export PATH="/usr/local/bin:$PATH"
par6d --check-config

say "5. the wheel, in a bare venv"
wheel="$(echo "$DIST"/par6-*.whl)"
[ -f "$wheel" ] || die "no par6 wheel in $DIST"
python3 -m venv "$WORK/venv"
"$WORK/venv/bin/pip" -q install "$wheel"
"$WORK/venv/bin/python" -c "import par6; print('wheel imports self-contained:', par6.__version__)"

say "6. kinematics, collision and the installed daemon"
PAR6D_BIN=/usr/local/bin/par6d "$WORK/venv/bin/python" \
  "$(dirname "${BASH_SOURCE[0]}")/validate_engine.py"

say "the shipped artifacts install and run outside the build environment"
