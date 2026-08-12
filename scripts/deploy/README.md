# Deploying `par6d` to the control box

Target: **Raspberry Pi 5, aarch64, PREEMPT_RT kernel**, one `par6d` process
supervised by systemd, talking SocketCAN to the arm and protocol v2 (UDP) to
Waldo Commander.

```
scripts/deploy/build-aarch64.sh          cross-build par6d for aarch64
scripts/deploy/install.sh                stage + upload + install (or install locally)
scripts/deploy/par6d.service             the systemd unit
```

## 1. Cross-build

```bash
sudo apt-get install -y gcc-aarch64-linux-gnu    # once
scripts/deploy/build-aarch64.sh
# -> target/aarch64-unknown-linux-gnu/release/par6d
```

The toolchain is exactly what CI's aarch64 matrix leg uses (see
`.github/workflows/ci.yml`): the `aarch64-unknown-linux-gnu` rustup target
with `aarch64-linux-gnu-gcc` as the linker, selected through
`CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER`. CI runs `cargo check` for
that target; this script is where the **link** step actually runs.

**glibc floor.** The produced binary's highest versioned symbol requirement is
`GLIBC_2.34` (measured with `aarch64-linux-gnu-readelf -V` on the artifact
built by this script with an Ubuntu 24.04 cross toolchain). Raspberry Pi OS
bookworm ships glibc 2.36, so it runs; bullseye (2.31) will **not**. If the
box is older, build in a container matching its distro instead. `install.sh`
runs `par6d --help` right after copying the binary so a mismatch fails loudly
at install time.

**No kinematics in this build.** `par6d`'s `ffi` feature (Pinocchio C-ABI
shim: TCP FK, gravity compensation, `move_j_pose`/`move_l`, `--sim-dynamics`)
is off by default because it needs a C++ toolchain. `scripts/ffi/setup.sh`
builds an **x86_64** shim only, so `PAR6_CARGO_FLAGS="--features ffi"` will not
cross-compile without first producing an aarch64 Pinocchio + shim. Without it,
joint-space motion, homing, jogging, the queue and the whole protocol plane
work; Cartesian commands and the STATUS `pose`/`tcp_speed` fields do not
(`pose` reads NaN). Building the aarch64 shim is not solved here.

## 2. Install

From a dev machine (needs `ssh`/`scp` to the box and `sudo` on it):

```bash
scripts/deploy/install.sh --host pi@par6-box
```

It stages a bundle (binary + `config/PAR6.toml` + `config/grippers/*.toml` +
`assets/par6_description` + the unit + a copy of itself), uploads it to
`/tmp/par6-deploy-<timestamp>`, and re-runs itself there with `--local`.

On the box itself:

```bash
sudo scripts/deploy/install.sh --local --bundle /tmp/par6-deploy-<timestamp>
```

Layout after install:

| Path | Contents |
|---|---|
| `/usr/local/bin/par6d` | the runtime binary |
| `/etc/par6/PAR6.toml` | robot config (`PAR6_CONFIG` in the unit) |
| `/etc/par6/grippers/*.toml` | gripper configs |
| `/usr/share/par6/par6_description` | URDF/meshes — only read by `ffi` builds |
| `/etc/systemd/system/par6d.service` | the unit |
| `/var/lib/par6` | `StateDirectory`, the working directory |

An existing `/etc/par6/*.toml` is **kept** on re-install (tuning survives
upgrades); pass `--force-config` to overwrite. `--no-restart` installs without
touching the running service.

> Restarting `par6d` stops the arm and clears the queue. `install.sh` stops the
> service before swapping the binary unless `--no-restart` is given.

## 3. The unit

`par6d.service` runs as the unprivileged system user `par6` with two ambient
capabilities:

- **`CAP_SYS_NICE`** — the RT thread asks for `SCHED_FIFO` priority 99 and pins
  itself to CPU 3 (`spec/RT.md`). Failure is logged `DEGRADED` and is *not*
  fatal, so a misconfigured box runs badly instead of not at all — check the
  journal for `RT thread: SCHED_FIFO priority 99` to confirm it took.
  `LimitRTPRIO=99` is set as well for boxes without the capability path.
- **`CAP_NET_ADMIN`** — `par6d` brings `can0` up at the configured bitrate when
  it finds it down.

The unit deliberately sets **no `CPUAffinity=`**: `par6d` pins its own RT
thread, and a process-wide mask would trap the tokio command plane on the same
core. Isolate the RT core in the kernel cmdline instead
(`/boot/firmware/cmdline.txt` on RPi OS):

```
isolcpus=3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2
```

Logging goes to journald (`SyslogIdentifier=par6d`, `RUST_LOG=info`):

```bash
journalctl -u par6d -f
systemctl status par6d
```

`Restart=always` with `RestartSec=2`, bounded by `StartLimitBurst=5` per
`StartLimitIntervalSec=60` so a genuinely broken install stops flapping.

### Simulator on the box

```bash
sudo systemctl edit par6d      # drop-in
[Service]
ExecStart=
ExecStart=/usr/local/bin/par6d --sim
```

`--sim` runs unprivileged with no SCHED_FIFO and no CPU pin.

## 4. Post-install check

```bash
journalctl -u par6d -n 30
#   loaded PAR6 (6 joints, tick 250 Hz) from /etc/par6/PAR6.toml
#   command plane on 0.0.0.0:6001 (SocketCAN backend)
#   RT thread: SCHED_FIFO priority 99
#   RT thread pinned to CPU 3
```

From a machine on the same network:

```python
from par6 import Robot
robot = Robot(host="par6-box.local")   # PINGs the running runtime, spawns nothing remote
robot.start()
print(robot.create_sync_client().angles())
```

## What is verified, and what is not

Verified in the development container (x86_64 Linux, no robot, no systemd):

- `build-aarch64.sh` produces a real aarch64 binary — `ELF 64-bit LSB pie
  executable, ARM aarch64 … dynamically linked`, 8.3 MB (the release profile
  keeps debug info for `par6d`, deliberately: `Cargo.toml`
  `[profile.release.package.par6d] debug = true`).
- `par6d.service` passes `systemd-analyze verify` with no warnings.
- `install.sh` argument handling, bundle staging, upload command shape, and
  every early failure path (missing bundle, missing binary, non-root `--local`,
  bad flags) — the upload itself was exercised against stub `ssh`/`scp`, and the
  resulting tarball unpacked and inspected.

**Not verified — no hardware and no systemd in this environment:**

- The unit has never been started. `Type=exec`, the ambient capabilities
  actually granting `SCHED_FIFO`, the sandboxing directives
  (`ProtectSystem=strict`, `RestrictAddressFamilies=…AF_CAN`) not blocking
  SocketCAN or the netlink link bring-up: all reasoned from the code paths in
  `crates/par6-rt/src/rt.rs` and `crates/par6d/src/daemon.rs`, none observed.
  If `can0` cannot be opened after install, relax `RestrictAddressFamilies`
  first and report it.
- `install.sh` has never completed a real end-to-end run: no `ssh`/`scp` and no
  running systemd here, so `useradd`, `systemctl daemon-reload/enable/restart`
  and the config-preservation branch are untested against a live box.
- Nothing has run on aarch64 at all. The cross build links; it has not been
  executed. The 250 Hz tick, the PREEMPT_RT latency budget and the loop
  degradation bands are hardware bring-up (Phase 3) questions.
- `--features ffi` for aarch64 is not supported by any script here.
