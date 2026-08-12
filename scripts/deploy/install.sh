#!/usr/bin/env bash
# Install par6d + its config + the systemd unit onto the PAR6 control box.
#
#   # from a dev machine (needs ssh/scp access; sudo on the box):
#   scripts/deploy/build-aarch64.sh
#   scripts/deploy/install.sh --host pi@par6-box
#
#   # on the box itself, from an unpacked bundle:
#   sudo scripts/deploy/install.sh --local --bundle /tmp/par6-deploy-<ts>
#
# Installs to:
#   /usr/local/bin/par6d              the runtime binary
#   /etc/par6/PAR6.toml               robot config (kept on re-install unless --force-config)
#   /etc/par6/grippers/*.toml         gripper configs (same rule)
#   /usr/share/par6/par6_description  URDF/meshes (only used by `ffi` builds)
#   /etc/systemd/system/par6d.service the unit
#
# RESTARTING par6d STOPS THE ROBOT. The service is restarted unless
# --no-restart is passed.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
TARGET_TRIPLE="aarch64-unknown-linux-gnu"

HOST=""
BUNDLE=""
LOCAL=0
RESTART=1
FORCE_CONFIG=0
BINARY="$ROOT/target/$TARGET_TRIPLE/release/par6d"
CONFIG_DIR="$ROOT/config"
ASSETS_DIR="$ROOT/assets/par6_description"
UNIT="$ROOT/scripts/deploy/par6d.service"

STAGE_DIR=""
SERVICE_USER="par6"
BIN_DEST="/usr/local/bin/par6d"
ETC_DEST="/etc/par6"
ASSETS_DEST="/usr/share/par6/par6_description"
UNIT_DEST="/etc/systemd/system/par6d.service"

die() { echo "install: $*" >&2; exit 1; }
say() { echo "install: $*"; }

usage() {
  # The header comment block, minus the shebang.
  awk 'NR>1 && /^#/ { sub(/^# ?/, ""); print; next } NR>1 { exit }' "$SELF"
  exit "${1:-0}"
}

while [ $# -gt 0 ]; do
  case "$1" in
    --host) HOST="${2:?--host needs user@host}"; shift 2;;
    --binary) BINARY="${2:?--binary needs a path}"; shift 2;;
    --config) CONFIG_DIR="${2:?--config needs a directory}"; shift 2;;
    --assets) ASSETS_DIR="${2:?--assets needs a directory}"; shift 2;;
    --bundle) BUNDLE="${2:?--bundle needs a directory}"; shift 2;;
    --local) LOCAL=1; shift;;
    --no-restart) RESTART=0; shift;;
    --force-config) FORCE_CONFIG=1; shift;;
    -h|--help) usage 0;;
    *) echo "install: unknown argument $1" >&2; usage 2;;
  esac
done

# ---------------------------------------------------------------- local half

install_local() {
  local bundle="$1"
  [ "$(id -u)" -eq 0 ] || die "--local must run as root (use sudo)"
  [ -d "$bundle" ] || die "bundle directory not found: $bundle"
  [ -f "$bundle/par6d" ] || die "no par6d binary in $bundle"
  [ -f "$bundle/par6d.service" ] || die "no unit file in $bundle"

  if command -v file >/dev/null && [ "$(uname -m)" = "aarch64" ]; then
    file -b "$bundle/par6d" | grep -q "ARM aarch64" \
      || die "bundled par6d is not an aarch64 binary: $(file -b "$bundle/par6d")"
  fi

  if ! id -u "$SERVICE_USER" >/dev/null 2>&1; then
    say "creating system user $SERVICE_USER"
    useradd --system --no-create-home --shell /usr/sbin/nologin "$SERVICE_USER"
  fi

  if [ "$RESTART" -eq 1 ] && systemctl is-active --quiet par6d; then
    say "stopping par6d (the arm will stop holding queued motion)"
    systemctl stop par6d
  fi

  # Staged then renamed: a rename works even while the old binary is
  # running (a plain overwrite would hit ETXTBSY under --no-restart).
  install -D -m 0755 "$bundle/par6d" "$BIN_DEST.new"
  # Catches an architecture or glibc mismatch here rather than as a
  # systemd start failure five steps later.
  if ! "$BIN_DEST.new" --help >/dev/null 2>&1; then
    rm -f "$BIN_DEST.new"
    die "the bundled binary does not run on this host
  (wrong architecture, or the box's glibc is older than the cross toolchain's —
   see scripts/deploy/README.md)"
  fi
  mv -f "$BIN_DEST.new" "$BIN_DEST"
  say "installed $BIN_DEST"

  install -d -m 0755 "$ETC_DEST" "$ETC_DEST/grippers"
  install_config "$bundle/config/PAR6.toml" "$ETC_DEST/PAR6.toml"
  for gripper in "$bundle"/config/grippers/*.toml; do
    [ -e "$gripper" ] || continue
    install_config "$gripper" "$ETC_DEST/grippers/$(basename "$gripper")"
  done

  if [ -d "$bundle/par6_description" ]; then
    install -d -m 0755 "$(dirname "$ASSETS_DEST")"
    rm -rf "$ASSETS_DEST"
    cp -a "$bundle/par6_description" "$ASSETS_DEST"
    say "installed $ASSETS_DEST"
  fi

  install -D -m 0644 "$bundle/par6d.service" "$UNIT_DEST"
  systemctl daemon-reload
  systemctl enable par6d >/dev/null
  say "enabled par6d.service"

  if [ "$RESTART" -eq 1 ]; then
    systemctl restart par6d
    for _ in $(seq 20); do
      if systemctl is-active --quiet par6d; then break; fi
      sleep 0.5
    done
    systemctl --no-pager --lines=0 status par6d || true
    if ! systemctl is-active --quiet par6d; then
      die "par6d did not come up — journalctl -u par6d -n 50"
    fi
    say "logs: journalctl -u par6d -f"
  else
    say "not started (--no-restart); start with: systemctl start par6d"
  fi
}

install_config() {
  local src="$1" dest="$2"
  if [ -e "$dest" ] && [ "$FORCE_CONFIG" -eq 0 ]; then
    say "keeping existing $dest (pass --force-config to overwrite)"
    return
  fi
  install -m 0644 "$src" "$dest"
  say "installed $dest"
}

# --------------------------------------------------------------- remote half

stage_bundle() {
  local dir="$1"
  [ -f "$BINARY" ] || die "binary not found: $BINARY
  build it first: scripts/deploy/build-aarch64.sh"
  [ -f "$CONFIG_DIR/PAR6.toml" ] || die "no PAR6.toml under $CONFIG_DIR"
  [ -d "$CONFIG_DIR/grippers" ] || die "no grippers/ under $CONFIG_DIR"
  mkdir -p "$dir/config/grippers"
  cp "$BINARY" "$dir/par6d"
  cp "$UNIT" "$dir/par6d.service"
  cp "$SELF" "$dir/install.sh"
  cp "$CONFIG_DIR/PAR6.toml" "$dir/config/PAR6.toml"
  cp "$CONFIG_DIR"/grippers/*.toml "$dir/config/grippers/"
  if [ -d "$ASSETS_DIR" ]; then
    cp -a "$ASSETS_DIR" "$dir/par6_description"
  else
    say "no assets tree at $ASSETS_DIR — skipping (only 'ffi' builds need it)"
  fi
}

install_remote() {
  command -v ssh >/dev/null || die "ssh not found"
  command -v scp >/dev/null || die "scp not found"
  local stamp remote_dir tarball
  stamp="$(date +%Y%m%d-%H%M%S)"
  remote_dir="/tmp/par6-deploy-$stamp"
  STAGE_DIR="$(mktemp -d)"
  trap 'rm -rf "${STAGE_DIR:-}" "${STAGE_DIR:-}.tar.gz"' EXIT
  stage_bundle "$STAGE_DIR"
  tarball="$STAGE_DIR.tar.gz"
  tar -czf "$tarball" -C "$STAGE_DIR" .
  say "uploading $(du -h "$tarball" | cut -f1) to $HOST:$remote_dir"
  ssh "$HOST" "mkdir -p '$remote_dir'"
  scp -q "$tarball" "$HOST:$remote_dir/bundle.tar.gz"
  rm -f "$tarball"

  local flags="--local --bundle '$remote_dir'"
  if [ "$RESTART" -eq 0 ]; then flags="$flags --no-restart"; fi
  if [ "$FORCE_CONFIG" -eq 1 ]; then flags="$flags --force-config"; fi
  # -t so sudo can prompt for a password on the box.
  ssh -t "$HOST" "set -e
    cd '$remote_dir'
    tar -xzf bundle.tar.gz
    sudo bash '$remote_dir/install.sh' $flags"
}

if [ "$LOCAL" -eq 1 ]; then
  [ -n "$BUNDLE" ] || die "--local needs --bundle DIR"
  install_local "$BUNDLE"
elif [ -n "$HOST" ]; then
  install_remote
else
  usage 2
fi
