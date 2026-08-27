# Deploying `par6d` to the control box

Target: **Raspberry Pi 5, aarch64, PREEMPT_RT kernel**, one `par6d` process
supervised by systemd, talking SocketCAN to the arm and protocol v2 (UDP) to
Waldo Commander.

```
scripts/ffi/setup.sh --target aarch64    build the aarch64 Pinocchio shim
scripts/deploy/build-aarch64.sh          cross-build par6d for aarch64
scripts/deploy/install.sh                stage + upload + install (or install locally)
scripts/deploy/par6d.service             the systemd unit
```

## 1. Cross-build

```bash
scripts/ffi/setup.sh --target aarch64    # once (or after a dependency pin bump)
source .ffi/env-aarch64.sh
scripts/deploy/build-aarch64.sh
# -> target/aarch64-unknown-linux-gnu/release/par6d
```

**Every par6d carries kinematics.** The Pinocchio C-ABI shim — TCP FK,
gravity compensation, `move_l`/`move_j_pose`, the cartesian streamables,
TOPPRA, and the coal collision world — is linked unconditionally, because a
runtime without it would broadcast a NaN TCP pose, report zero cartesian
freedom, refuse every cartesian command, and answer `set_shapes` with success
against a collision world that does not exist, none of which a client can
see. `build-aarch64.sh` therefore fails early when the aarch64 shim has not
been built. The library crates still compile without any C++ toolchain; it is
only the binary that requires one.

### How the aarch64 shim is produced

`scripts/ffi/setup.sh --target aarch64` cross-builds it on the dev machine.
Nothing needs to run on the control box, and the box needs no compiler:

- conda-forge publishes `pinocchio`, `coal`, `urdfdom` and `eigen` for
  `linux-aarch64`, so micromamba **downloads** the target's libraries with
  `--platform linux-aarch64` — an env that is never executed on the host.
- the compiler is conda-forge's `gxx_linux-aarch64` cross toolchain from the
  host's own `linux-64` channel, pinned to `sysroot_linux-aarch64=2.17` (the
  same glibc conda-forge builds its own aarch64 packages against, which is
  what keeps the whole set at one floor).
- `toppra-cpp` has no conda-forge package at all, so it is built from the
  pinned commit through a generated CMake toolchain file, exactly like the
  native path.
- `scripts/ffi/stage_runtime_libs.py` then walks `DT_NEEDED` from
  `libpar6_shim.so` and copies the whole closure — 20 libraries, ~65 MB —
  into the shim's own `lib/` directory. That directory is the deploy unit:
  the shim is linked with `$ORIGIN`, `par6d` with an rpath of
  `/usr/local/lib/par6`, and `install.sh` copies the one into the other.

The alternatives were building natively on the box (needs a Rust and C++
toolchain on a Pi and takes the better part of an hour per change) and
vendoring prebuilt binaries (unpinned, unreproducible, and a licence
question). Cross-building keeps a single reproducible command and the same
pins as the x86_64 path.

**glibc floor.** The staged closure and the shim require at most
`GLIBC_2.17`; the `par6d` binary linked against them requires at most
`GLIBC_2.17` as well, because `.ffi/env-aarch64.sh` makes the conda cross
compiler the Rust linker too. Raspberry Pi OS **bookworm ships 2.36** and
bullseye ships 2.31, so both clear it — a change from the previous
Debian-cross build, whose floor was `GLIBC_2.34`. `build-aarch64.sh` prints
the measured floor after each build, `stage_runtime_libs.py` prints the
closure's, and `install.sh` runs `par6d --help` right after copying so a
mismatch still fails loudly at install time.

**Symbol-version check.** Because nothing here can be *executed* for
aarch64, `stage_runtime_libs.py` performs the check that would otherwise
only surface on the box: every versioned symbol (`GLIBCXX_*`, `CXXABI_*`,
`GCC_*`, …) demanded of a library that ships must be provided by the copy
that ships. This is what catches a cross compiler newer than the target
env's C++ runtime, which otherwise appears as
`version GLIBCXX_3.4.x not found` at the first `systemctl start`.

## 2. Install

From a dev machine (needs `ssh`/`scp` to the box and `sudo` on it):

```bash
scripts/deploy/install.sh --host pi@par6-box
```

It stages a bundle (binary + `lib/` — the shim and its runtime closure —
+ `config/PAR6.toml` + `config/grippers/*.toml` + `assets/par6_description`
+ the unit + a copy of itself), uploads it to `/tmp/par6-deploy-<timestamp>`,
and re-runs itself there with `--local`. `PAR6_RUNTIME_LIB_SRC` (exported by
`.ffi/env-aarch64.sh`) says where the staged libraries come from; pass
`--runtime-libs DIR` to override it. `--stage-only DIR` builds the bundle
without uploading anything, which is what CI checks.

On the box itself:

```bash
sudo scripts/deploy/install.sh --local --bundle /tmp/par6-deploy-<timestamp>
```

Layout after install:

| Path | Contents |
|---|---|
| `/usr/local/bin/par6d` | the runtime binary |
| `/usr/local/lib/par6/*.so` | the Pinocchio shim + its runtime closure (rpath target) |
| `/etc/par6/PAR6.toml` | robot config (`PAR6_CONFIG` in the unit) |
| `/etc/par6/grippers/*.toml` | gripper configs |
| `/usr/share/par6/par6_description` | URDF/meshes — the kinematics and collision models |
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
  itself to CPU 3. Failure is logged `DEGRADED` and is *not*
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

### If it does not start

- **`can0` cannot be opened.** The unit's `RestrictAddressFamilies` must
  include `AF_CAN`; relax it and confirm the bus opens before looking
  anywhere else.
- **Starts by hand but not under systemd.** `ProtectSystem=strict` leaves
  `/usr` readable, so the rpath into `/usr/local/lib/par6` should resolve —
  run `ldd /usr/local/bin/par6d` from inside the unit's namespace to see
  which library the sandbox is hiding.
