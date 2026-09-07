# AGENTS.md - par6

Nothing is added to this file without the repo owner's explicit approval.

Rust real-time runtime (`par6d`) + Python waldoctl client for the PAR6 arm.
Read `README.md` for architecture, the command system, and the collision world.

## Commands

```bash
pixi run setup                     # once: solves the C++ deps and builds the shim
pixi run lint                      # fmt + clippy, must be clean
pixi run test-rust                 # rust tests
pixi run cargo run -p par6d -- --sim        # simulated runtime, no hardware
pixi run install-python            # python package (maturin: compiles par6-py)
pixi run test-python               # python tests (JUnit XML at python/test-results.xml)
pixi run test-e2e                  # the client against a real par6d --sim
```

pixi provides the C++ closure (Pinocchio, coal, eigen, urdfdom, libmujoco,
cmake, ninja, the compiler) from `pixi.lock`; Rust comes from rustup via
`rust-toolchain.toml`. `crates/par6-kin/build.rs` compiles the shim and
toppra into cargo's `OUT_DIR`, so any cargo invocation under `pixi run`
provisions them and cargo owns their freshness — there is no separate
bootstrap step and no `.ffi` for a native build.

CI runs these same tasks and nothing else: a red job is reproduced locally
with the command in its `run:` line.

## Contract discipline (multi-agent repo)

- `crates/par6-proto`'s own tests are the codec suite (encode + decode +
  hostile inputs). A contract change without updated, passing tests is
  incomplete.
- `python/par6/protocol/constants.py` is GENERATED from `par6-proto` — never edit by
  hand; regenerate and let the freshness-guard test prove it.

## Licensing rules

- This repo is **Apache-2.0**. The vendor runtime (RCB-Runtime) is GPL: it is **behavior-only
  reference — port behavior and constants, never code**.
- parol6 (`Jepson2k/PAROL6-python-API`) is GPL-3.0: carry over code only where you hold
  authorship (self-relicensing); otherwise reimplement the semantics independently.
- `assets/` is CERN-OHL-S v2 vendor material (upstream also states Apache-2.0; the
  licence file it ships is CERN-OHL-S) — keep `assets/NOTICE` accurate.

## Testing Guidelines

- **No tautological tests.** Assert behavior, not what's true by construction — not
  default fields, constructor args echoed back, enum literals, `isinstance`/frozen-raises,
  or stub-raises-`NotImplementedError`. Drive a method/workflow and assert the outcome.
- **No testing theatre.** Default to real components: the sim bus backend, `par6d --sim`.
  A hand-rolled fake is a last resort; never fake a contract you haven't
  read — match real raise-vs-return behavior, return codes, signatures. If a fake must
  mimic protocol behavior (acks, ordering, lifecycle), the test is at the wrong layer.
- Enter through the real path (protocol dispatch / DriverBus / client API), not internal
  helpers fed hand-built inputs. Assert outcomes, not interactions — "the fake was
  called" proves nothing.
- Derive cases from the requirement, not the code under test — "rejects invalid input"
  means NaN/inf/negative/zero/short-array, not the cases the code already handles.
- A regression test must fail against the bug before the fix. Born-green regression
  tests are theatre.
- Prefer fewer, comprehensive integration tests (client ↔ `par6d --sim` workflows) over
  many shallow unit tests. No coverage targets — working features, not metrics.
  When tests are variations of the same thing, merge into one test with multiple
  assertions.
- **Determinism over sleeps.** All timing-dependent logic must be testable with a
  virtual clock / tick counter — the sim runs on fixed dt, never wall clock. No
  `sleep()`-and-hope in tests; poll a condition or drive ticks explicitly. Time
  constants live in config as SECONDS and convert via `round(s/dt)` — never hardcode
  tick counts.
- **When CI tests fail, fix them.** Don't waste time analyzing whether failures are
  "related to your changes" — the goal is green CI, not attribution.
- Never run the parol6 or Waldo-Commander pytest suites in parallel with anything —
  they are timing-sensitive and share resources. par6's own suites are designed to be
  parallel-safe; keep them that way (no fixed ports — allocate free ones per test).

## Rust rules

- The RT tick path allocates NOTHING after init (preallocate in constructors; slices
  and in-place mutation; no formatting except one-shot error paths). Tests may assert
  this with a counting allocator.
- `Option<T>` channel semantics on the bus are load-bearing (None = omitted on the
  wire, NOT zero — the vendor firmware distinguishes them). Don't collapse them to defaults.
- `-D warnings` clippy and rustfmt are CI gates.
- Which thread a piece of work runs on is decided by its timing class, never by its
  feature — see *One plane per deadline* in `README.md`. Bounded work never shares a
  thread with unbounded work, and a new thread needs the one-sentence deadline
  justification that section asks for.

## Code style

- **Comments:** a short WHY is fine; never describe WHAT the code does, and describe the
  final implementation, not the change ("changed X to Y" is review noise).
- **Never ship declared-but-unimplemented API surface.** No "reserved" fields or params,
  no docstrings saying "not yet applied". Stubs that must exist return an explicit
  NOT_IMPLEMENTED error — they never silently succeed.
- Python: never `except Exception: pass` — catch specifically, or log/handle meaningfully.
  Fix type errors properly (`@overload`, narrowing, `cast()`); ignores are a last resort.

## Cross-repo workflow

- Dependency direction: `waldoctl` (contracts) ← `par6` (this repo) ← `Waldo-Commander`.
- Use the SAME branch name across repos with coordinated changes; WC's CI installs
  `par6` from a same-named branch when one exists (`#subdirectory=python`), falling back
  to the `main` pin.
- The python package versions with semver in `python/pyproject.toml`; pre-1.0 breaking
  changes bump minor.
