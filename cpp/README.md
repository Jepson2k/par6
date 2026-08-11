# par6_shim — C++ FFI layer (Pinocchio, later toppra-cpp)

One C-ABI shim toolchain for the C++ dependencies of the Rust runtime:

- **Pinocchio** (kinematics/dynamics) — implemented: `par6_kin_*`
  (create/destroy, fk, jacobian, gravity, DLS ik_step), consumed by
  `crates/pinokin-sys` and, on top of that, `par6-kin`.
- **toppra-cpp** (path parameterization for `par6-motion`) — **reserved,
  not implemented**: the `par6_traj_*` entry points exist for ABI stability
  and return `PAR6_ERR_NOT_IMPLEMENTED`. conda-forge ships no C++ toppra
  (only `toppra-python`, pure Python); wiring it means either building
  [hungpham2511/toppra](https://github.com/hungpham2511/toppra)'s `cpp/`
  tree from source inside `scripts/ffi/setup.sh` or packaging it — tracked
  for a follow-up, do **not** fake these entry points in the meantime.

The same numerics stack backs the Python side via
[pinokin](https://github.com/Jepson2k/pinokin), whose C++ core this shim's
FK/Jacobian/tool conventions mirror.

## Layout

```
cpp/include/par6_shim.h    the frozen C ABI (PAR6_SHIM_ABI_VERSION)
cpp/src/par6_shim.cpp      implementation (pinocchio only, no other deps)
cpp/CMakeLists.txt         builds libpar6_shim.so + libpar6_shim.a
crates/pinokin-sys/        raw decls + safe Model wrapper + conformance tests
scripts/ffi/setup.sh       reproducible toolchain bootstrap (micromamba)
scripts/ffi/gen_fixtures.py  reference fixtures from pip `pin`
```

## Quick start

```bash
scripts/ffi/setup.sh          # micromamba → conda env → cmake build+install
source .ffi/env.sh            # exports PAR6_SHIM_LIB_DIR / PAR6_SHIM_INCLUDE_DIR
cargo test --manifest-path crates/pinokin-sys/Cargo.toml --features ffi
```

Everything lands in `<repo>/.ffi` (self-gitignored, override with
`PAR6_FFI_DIR`): `bin/micromamba`, `env/` (conda-forge packages), `shim/`
(installed lib + header), `env.sh`. Re-running is idempotent; `FORCE=1`
rebuilds the shim.

Pinned versions (see `scripts/ffi/setup.sh`, `PAR6_PINOCCHIO_VERSION` to
override): **pinocchio 4.1.0**, with conda-forge companions as of 2026-08:
eigen 5.0.1, urdfdom 6.0.0, boost 1.90, cxx-compiler (hermetic gcc), cmake,
ninja. The pip reference is pinned to the same release: `pin==4.1.0`.

## ABI conventions (frozen in `par6_shim.h`)

- Poses: 4×4 homogeneous, **row-major**, 16 doubles.
- Jacobians: 6×nq, row-major, rows `[linear; angular]`,
  `LOCAL_WORLD_ALIGNED` (world axes at the frame origin).
- Gravity: RNEA at zero velocity/acceleration (spec/RT.md `G(q)`).
- Optional rigid tool at create: `T_ee_tool` shifts fk/jacobian/ik to the
  tool frame; mass/COM/inertia (ee-frame coords) are appended to the parent
  joint so gravity covers "arm + active gripper tool link" per spec/RT.md.
- All `par6_kin_*` calls after create are allocation-free (`pinocchio::Data`
  and every workspace buffer preallocated in the handle). Handles are not
  thread-safe — one handle per thread.
- Exceptions never cross the boundary; failures come back as `par6_status`.

## pinokin-sys linking (build.rs contract)

| Env var | Meaning |
|---|---|
| `PAR6_SHIM_LIB_DIR` | required with `--features ffi`; dir containing `libpar6_shim.{so,a}` |
| `PAR6_SHIM_INCLUDE_DIR` | optional; sanity-checked (`par6_shim.h`), reserved for a future bindgen step |
| `PAR6_SHIM_LINK` | `dylib` (default) or `static` |
| `PAR6_SHIM_DEP_LIB_DIR` | required for `static`: Pinocchio lib dir (`.ffi/env/lib`) |

Without the `ffi` feature the crate is an empty stub — plain `cargo check`
/ `cargo test` need no C++ toolchain, which is why the crate is standalone
and **not** a member of the root workspace.

`dylib` mode links `libpar6_shim.so`, whose install rpath points at the
conda env's `lib/`, and adds an rpath to `PAR6_SHIM_LIB_DIR` — tests run
without `LD_LIBRARY_PATH`. `static` mode links `libpar6_shim.a` plus
Pinocchio (`pinocchio_default`, `pinocchio_parsers`) and `libstdc++`
dynamically; Pinocchio has no static conda-forge builds.

## Validation

`scripts/ffi/gen_fixtures.py` (needs `pip install pin==4.1.0 numpy`)
computes fk/jacobian/gravity for 20 seeded random configurations of
`assets/par6_description/URDF/par6_flange/urdf/par6_flange.urdf` (frame
`gripper`), twice — bare flange and with a rigid test tool — and writes
`crates/pinokin-sys/tests/fixtures/par6_flange_pin.json`. The `ffi`-gated
Rust tests load the same URDF through the shim and must match to **1e-9
absolute**; IK is checked by re-reaching fixture poses from perturbed seeds.
Regenerate fixtures whenever the pinned Pinocchio version, sample set, or
tool parameters change.

## aarch64 (control box) story

conda-forge ships `linux-aarch64` builds of pinocchio (and micromamba), so
the intended CI shape is a **native arm64 job** (e.g. `ubuntu-24.04-arm`
runners or qemu): run the same `scripts/ffi/setup.sh` — it already selects
`linux-aarch64` micromamba from `uname -m` — then build `cpp/` and upload
`libpar6_shim.{so,a}` + `par6_shim.h` + the env's `lib/` as the linkable
artifact for `pinokin-sys`. True cross-compilation (x86_64 host → aarch64
target) is *not* supported by this setup, because conda-forge packages are
native and CMake would need a full cross sysroot; don't attempt it — use a
native arm runner or qemu instead. CI wiring itself is a separate task
(deliberately not implemented here).
