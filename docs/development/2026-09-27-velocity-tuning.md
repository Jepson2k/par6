# Bounded velocity PI tuning

Historical PRBS implementation and physical result. The current implementation
is described in [Periodic velocity-loop identification](2026-09-28-periodic-tuning.md).

The final stage tunes Kpv and Kiv. It uses the existing friction, gravity,
limits, electrical calibration and drive capture/injection firmware.

## Method and scope

[Giacomelli et al., IFAC 2018](https://doi.org/10.1016/j.ifacol.2018.06.067),
section 2, separates a bounded identification experiment from numerical PI
design. This implementation keeps the existing closed-loop current injection
and the [Åström, Panagopoulos and Hägglund MIGO objective](https://doi.org/10.1016/S0005-1098(98)00011-9):
maximize integral action subject to a measured sensitivity bound. It does not
implement Giacomelli's open-loop torque sequence, pole cancellation, or
resonance-compensating filters. Neither paper specifies our capture budget.

Each pose gets one fast capture (divisor 4, 0.65536 s) and one slow capture
(divisor 12, 1.96608 s). Spectra are pooled across neighbouring frequencies;
correlated Hann bins do not establish an independent-average confidence bound.
The existing coherence, controller-consistency and crossover-coverage checks
remain in force. Unresolved data means unchanged gains, not a lower threshold.

The plan selects ready for an inertia ratio at most 1.25, otherwise the two
extremes among the reachable candidate poses. This samples pose dependence;
it does not prove coverage of every intermediate configuration. Ready remains
the physical validation pose. There are two retries per joint, shared by both
rates and all poses, only for a lost/misaligned injection or a reversal of the
slide. A reduced injection amplitude persists into later captures. Thus:

| Identification poses | Planned captures | Maximum including retries |
|---|---|---|
| One | 2 | 4 |
| Two | 4 | 6 |

These are operational limits chosen to bound final tuning. A joint whose first
pair cannot support a PI design is dropped before visiting another pose.
Identification returns use the existing capped move and its observed hold;
the stiction stage's additional pre/post rest waits are not repeated after
the capture download has already held the joint.

Numerical design runs without moving the arm. If it yields a candidate, the
configured and candidate gains each take two matched steps, one per direction,
with identical friction feedforward. Each candidate score must be finite,
no worse than its directional baseline, and within the existing moving RMS
requirement. Candidate holding RMS must meet the existing joint holding limit;
a noisy baseline cannot raise that limit. Only then are the gains retained.
Validation adds at most four step captures and two observed holds. Capture
status must match the requested full length and sample rate.

## Offline evidence

The [fixtures](../../crates/par6d/tests/data/loop-response/README.md) come from
physical J1 runs. The aligned fast/slow pair identifies the configured PI to
within about 5%, but the low-frequency coherence gaps leave the design
unsupported. Replaying additional records from the same run also failed to
support a design. The bounded stage must reject that evidence early.

The separate physical step records distinguish a real regression (14.65% to
15.95% error) from an improvement (10.12%). They exercise rejection of finite
but worse tracking. Corrupted and mismatched recordings exercise fail-closed
analysis. Run only the offline replay target:

```sh
pixi run cargo test -p par6d --test loop_response_replay
```

No simulated robot is used. These checks validate software decisions on
recorded hardware data; the revised acquisition schedule and gains still need
physical validation. No accepted gain change can be claimed from these replays.

## Review and checks

The review covered pose selection, the shared retry budget, early rejection,
directional comparison, gain restoration and full capture validation. Early
rejection checks feasibility rather than requiring a bounded optimum at the
first pose: another pose may provide that bound.

Three offline tests pass (two hardware replays and one pose-selection test).
Temporarily restoring the previous finite-only acceptance reproduced the
tracking regression failure on the unchanged physical capture. Formatting,
Clippy with warnings denied for the affected library/binary/replay target,
and the release build of `par6-selfcal` pass. No hardware was moved or gains
applied during this change.

## First physical run

On 2026-09-27, `--tool Flange --only gains --joint 1` completed on the real
arm and parked successfully. Run directory:
`calibration-runs/velocity-pi-20260927/selfcal-1790550844178165958`.
CAN stayed error-active (normal operation), with no additional error-warning,
error-passive or bus-off transitions during the run.

There were three identification attempts at the first pose. The initial fast
capture at 180 mA stopped or reversed the slide in 10.4% of samples, consuming
one retry. The fast retry and slow capture both used 90 mA. Their merged
response identified the configured PI within 3%, but only 26 of 38 bands met
the coherence requirement. Every band below 21 Hz failed that requirement;
the 15 and 18 Hz bands had coherence 0.588 and 0.381, below 0.6. These gaps
left the crossover insufficiently covered and supported no candidate design.

The tuner stopped before the second pose and ran no gain-validation trials.
J1 remained at Kpv 0.02 and Kiv 0.003; all joint gains in the result matched
the installed config. The run omitted `--apply`. This confirms the bounded
acquisition and early rejection on hardware, but does not establish new tuned
gains. Improving the identification experiment remains necessary. No simulator
was used.
