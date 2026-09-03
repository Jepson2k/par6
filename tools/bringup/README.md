# Bring-up kit

Staged scripts for putting a PAR6 on a bench, each stage diagnosable before the
next. They run against a RUNNING runtime — `par6d` on hardware, `par6d --sim` on
a bench — through the shipped client; every kinematic quantity comes from
`par6._par6`, and nothing that moves the arm runs without `--go`. Each script
prints a ledger of named checks, `required` or `advisory`, and exits 1 when a
required check failed. `--json` emits the ledger for a commit.

| stage | script | needs | what it proves |
|---|---|---|---|
| 1 | `limiter_preview.py` | nothing | the runtime's own servo limiter, from each joint's mid-range: overshoot, convergence, the velocity ceiling used but never exceeded, no limit cycle, the soft-limit clamp |
| 2 | `stack_verify.py --go` | homed arm | convergence band and offset at the canonical pose; a nudge the encoders confirm with no other joint moving; gravity current on the wire |
| 3 | `first_motion.py --go` | homed arm | one joint, one raised-cosine period, checked against its own limits before anything moves, returned and verified |
| 4 | `multi_joint.py --go` | homed arm | three joints together; each tracking gap scales with its own aggressiveness |
| 5 | `loop_benchmark.py [--load]` | runtime | loop period percentiles at baseline and under cache/memory contention on the non-RT cores; run the runtime with `--tick-profile` for the per-phase breakdown and the overrun trace in its log |
| 6 | `acceptance_ladder.py --go` | runtime | ten rungs from connect to a zero-motion link-quality stream |

Run them in order. Home first (`par6 home --wait`); the scripts never home for
you except the ladder's rung 2. Every motion starts from the canonical pose (the
config's park pose), so a run is reproducible.

Rows move to "parity" only once these have been run on the arm and their JSON
ledgers committed under `tools/bringup/results/`.
