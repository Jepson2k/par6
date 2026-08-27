# CLAUDE.md - par6

Rust real-time runtime (`par6d`) + Python waldoctl client for the PAR6 arm.
Read `README.md` for architecture, the command system, and the collision world.

## Commands

```bash
scripts/ffi/setup.sh               # once: build the Pinocchio shim into .ffi/
source .ffi/env.sh                 # each shell: par6d needs the shim to build AND run
cargo build --workspace            # runtime
cargo test --workspace             # rust tests
cargo fmt --all && cargo clippy --workspace --all-targets -- -D warnings   # must be clean
cargo run -p par6d -- --sim        # simulated runtime, no hardware
pip install -e "python[dev]"       # python package (maturin: compiles the par6-py extension)
cd python && pytest                # python tests (JUnit XML at python/test-results.xml)
```

`par6d` links the shim unconditionally — there is no kinematics-free build.
The library crates still build without a C++ toolchain, which is what the
`--exclude par6d --exclude par6-py --exclude par6-client` legs in CI cover
(par6-py wraps par6d; par6-client's tests boot a daemon in-process).
The python package builds the `par6._par6` extension, so `pip install`
needs `source .ffi/env.sh` first, and so does running anything that
imports `par6` (the extension dlopens the shim).

## Contract discipline (multi-agent repo)

- `crates/par6-proto` and the trait contracts (`DriverBus`, sample ring, config schema)
  are **frozen interfaces**. Changing them requires a `contracts`-labeled issue — never
  drive-by edits from a feature branch.
- `tests/golden/` vectors are the wire conformance suite for the frozen codec
  (encode + decode, `par6-proto`). A contract change without regenerated
  vectors + passing tests is incomplete.
- `python/par6/protocol/constants.py` is GENERATED from `par6-proto` — never edit by
  hand; regenerate and let the freshness-guard test prove it.

## Licensing rules

- This repo is **MIT**. The vendor runtime (RCB-Runtime) is GPL: it is **behavior-only
  reference — port behavior and constants, never code**.
- parol6 (`Jepson2k/PAROL6-python-API`) is GPL-3.0: carry over code only where you hold
  authorship (self-relicensing); otherwise reimplement the semantics independently.
- `assets/` is Apache-2.0 vendor material — keep `assets/NOTICE` accurate.

## Testing Guidelines

- **No tautological tests.** Assert behavior, not what's true by construction — not
  default fields, constructor args echoed back, enum literals, `isinstance`/frozen-raises,
  or stub-raises-`NotImplementedError`. Drive a method/workflow and assert the outcome.
- **No testing theatre.** Default to real components: the sim bus backend, `par6d --sim`,
  golden vectors. A hand-rolled fake is a last resort; never fake a contract you haven't
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
- `-D warnings` clippy and rustfmt are CI gates. Public trait methods get doc comments —
  contracts are what downstream workstreams code against.

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
