# spec/

Behavioral specifications — the coordination contract for parallel workstreams.

- `CAN.md` — Spectral/STEPFOC bus protocol, byte-level (for `par6-bus`)
- `RT.md` — tick loop, mode output laws, errors, e-stop, streaming guards (for `par6-rt`, `par6-motion`)
- `HOMING.md` — homing FSM, sequences, gripper-dependent offsets (for `par6-rt`, sim backend)
- `PROTOCOL-V2.md` — client↔runtime wire protocol (for `par6-proto`, `par6-server`, python client)

Extracted from the vendor stack (Source-Robotics/RCB-Runtime and its MIT driver
libraries) and from the parol6 protocol. Vendor GPL code is reference-only: these
documents carry the behavior and constants; implementations are written fresh
against them. Items marked [OURS] are deliberate deviations, each with rationale.
