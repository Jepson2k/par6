#!/usr/bin/env bash
# Fetch the pinned toppra-cpp commit into .ffi/src/toppra.
#
# There is no conda-forge C++ toppra (only pure-python `toppra-python`), so the
# pin lives here rather than in pixi.toml's dependency table. $TOPPRA_COMMIT
# comes from pixi's [activation.env].
set -euo pipefail

SRC="${PIXI_PROJECT_ROOT:?run under pixi}/.ffi/src/toppra"
REPO="${PAR6_TOPPRA_REPO:-https://github.com/hungpham2511/toppra}"

if [[ "$(git -C "$SRC" rev-parse HEAD 2>/dev/null || true)" == "$TOPPRA_COMMIT" ]]; then
  echo ">>> toppra source at $TOPPRA_COMMIT"
  exit 0
fi

echo ">>> fetching toppra @ $TOPPRA_COMMIT"
rm -rf "$SRC"
mkdir -p "$SRC"
git -C "$SRC" init -q
git -C "$SRC" remote add origin "$REPO"
git -C "$SRC" fetch -q --depth 1 origin "$TOPPRA_COMMIT"
git -C "$SRC" checkout -q --detach FETCH_HEAD
