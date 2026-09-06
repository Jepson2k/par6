#!/usr/bin/env bash
# Materialise libmujoco into $MUJOCO_DOWNLOAD_DIR.
#
# `mujoco-rs` downloads it from its own build script, so the only way to get
# the library is to build the one crate that depends on it (par6-bus). That
# makes `.ffi` complete after `pixi run setup`, which is what every consumer
# assumes: anything linking `-lmujoco` without having built par6-bus first —
# the maturin wheel build, most obviously — fails to link otherwise.
#
# Keyed on the library being present rather than on cargo's freshness.
# `target/` and `.ffi` are cached separately (in CI they are two different
# cache entries), so cargo can hold par6-bus fresh — skipping the build
# script that does the download — while `.ffi` is empty. Cleaning mujoco-rs
# is what forces its script to run again.
set -euo pipefail

: "${MUJOCO_DOWNLOAD_DIR:?run under pixi}"

# The version follows the workspace mujoco-rs pin, as [activation.env] does.
if compgen -G "$MUJOCO_DOWNLOAD_DIR"/mujoco-*/lib/libmujoco.so >/dev/null; then
  echo ">>> libmujoco already in $MUJOCO_DOWNLOAD_DIR"
  exit 0
fi

# Retried: this is a network fetch of a ~50 MB archive from the internet, and
# a one-shot download in CI eventually meets a reset connection — which fails
# the whole job with "failed to download MuJoCo's hash file". Cleaning the
# crate again is what makes the build script re-run rather than cache the
# failure.
for attempt in 1 2 3; do
  echo ">>> downloading libmujoco via mujoco-rs's build script (attempt $attempt)"
  cargo clean -p mujoco-rs
  if cargo build -p par6-bus; then
    exit 0
  fi
  echo ">>> attempt $attempt failed" >&2
  sleep $((attempt * 5))
done

echo "libmujoco could not be downloaded after 3 attempts" >&2
exit 1
