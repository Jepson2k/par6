# par6

PAR6 robot backend for [Waldo Commander](https://github.com/Jepson2k/Waldo-Commander):
a **Rust real-time runtime** (`par6d`) that replaces Source Robotics' RCB-Runtime on the
control box, plus a **Python client package** (`python/par6`) implementing the
[waldoctl](https://github.com/Jepson2k/waldoctl) backend contracts.

```
Waldo Commander (NiceGUI frontend, unchanged)
  └─ python/par6 — waldoctl Robot + AsyncRobotClient + sync facade
       │ protocol v2: UDP msgpack commands · binary status broadcast · telemetry
  par6d (single Rust binary; `par6d --sim` runs anywhere, including CI)
   ├─ command plane (tokio): validation/gating, queue, index allocator,
   │    push completion, status broadcaster, telemetry recipes
   ├─ planner: TOPPRA (FFI) planned moves · rsruckig streaming/blending · trapezoid
   └─ RT thread (SCHED_FIFO, alloc-free): 250 Hz tick — CAN RX → state → gravity
        comp G(q) → mode dispatch → CAN TX → state snapshot
   bus backends: SocketCAN (Spectral/STEPFOC) | closed-loop dynamics sim (Pinocchio ABA)
```

Kinematics/dynamics run on **Pinocchio via a C-ABI shim** shared with the Python side's
[pinokin](https://github.com/Jepson2k/pinokin) — one numerics stack everywhere.

## Repository layout

| Path | Contents |
|---|---|
| `crates/par6-proto` | protocol v2 codec — **single source of truth**; python constants are generated |
| `crates/par6-config` | robot/gripper/homing TOML config |
| `crates/par6-kin` | Pinocchio FFI: fk / jacobian / gravity / ik, coal collision world (self-pairs + installation/program keep-out layers) |
| `crates/par6-motion` | TOPPRA + rsruckig + trapezoid, jog ramps, completion policies |
| `crates/par6-bus` | `DriverBus` trait, Spectral CAN codec, SocketCAN + sim backends |
| `crates/par6-rt` | RT tick loop, mode dispatch, homing FSM, error latching, e-stop |
| `crates/par6-server` | UDP command plane, status/telemetry broadcast, collision-world layers |
| `crates/par6d` | the runtime binary; plan-time collision gate (feature `ffi`) |
| `python/` | the `par6` pip package (waldoctl backend) |
| `spec/` | behavioral specs extracted from the vendor stack — the coordination contract |
| `tests/golden/` | cross-language golden vectors (Rust ↔ Python conformance) |
| `assets/` | PAR6 URDF + meshes (Apache-2.0, from Source Robotics) |

## Development workflow (parallel agents)

Work is organized as **contract-first workstreams** tracked as GitHub issues:

- **Phase 0 (serial)**: scaffold → `spec/` docs → `par6-proto` + golden vectors
  (*contract freeze 1*) → `DriverBus`/ring/config trait contracts (*contract freeze 2*) →
  C++ FFI build infrastructure (Pinocchio + toppra-cpp shim toolchain).
- **Phase 1 (parallel fan-out)**: one issue = one agent session — Spectral codec ·
  sim bus · kin FFI · motion · RT core · command plane · python client · WC wiring.
- **Phase 2**: e2e integration (python client ↔ `par6d --sim`), WC integration test,
  deploy tooling, optional MuJoCo sim backend.
- **Phase 3**: hardware bring-up (human in the loop), then rate-raise experiments.

Rules: golden vectors are the inter-agent interface tests; changes to `par6-proto`
or the trait contracts require a `contracts`-labeled issue and re-freeze; CI gates
all merges. The vendor runtime (GPL) is **spec-only reference — port behavior and
constants, never code**; everything needed is written down in `spec/`.

## Building

```bash
cargo build --workspace          # the runtime
cargo run -p par6d -- --sim      # simulated runtime (no hardware)
pip install -e "python[dev]"     # the python client
```

Install the client from git (what Waldo Commander's `[par6]` extra does):

```bash
pip install "par6 @ git+https://github.com/Jepson2k/par6.git@main#subdirectory=python"
```

## License

MIT (`LICENSE`). `assets/` contains Apache-2.0 material from
[Source-Robotics/PAR6-Collaborative-Robot-Arm](https://github.com/Source-Robotics/PAR6-Collaborative-Robot-Arm)
— see `assets/NOTICE`.
