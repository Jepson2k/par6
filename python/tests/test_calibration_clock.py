"""Differentiate simulated motion on its physics clock, retaining host deadlines."""

import numpy as np
import pytest

from par6.calibration.analysis import motion_metrics
from par6.calibration.capture import measurement_rows
from par6.calibration.routines import _steady


def test_fixed_step_measurements_survive_host_catchup_without_hiding_deadlines():
    # A deterministic constant-speed plant with alternating late/catch-up host
    # ticks. This reproduces the clock mismatch found in the native J6 capture;
    # no motor/protocol responses are fabricated.
    dt = 0.004
    count = 600
    host_seconds = np.r_[0, np.cumsum(np.resize([0.0025, 0.0055], count - 1))]
    rows = []
    for i in range(count):
        q, velocity = np.zeros(6), np.zeros(6)
        q[5], velocity[5] = 0.04 * i * dt, 0.04
        rows.append(
            {
                "tick": i,
                "elapsed_ns": round(host_seconds[i] * 1e9),
                "flags": (5 << 8) | 22,
                "q": q.tolist(),
                "q_commanded": q.tolist(),
                "qd": velocity.tolist(),
                "qd_commanded": velocity.tolist(),
                "tau": [0.01] * 6,
                "current_ma": [100] * 6,
                "ilim_ma": [2500] * 6,
            }
        )
    simulated = measurement_rows(rows, dt, simulator=True)
    physical = measurement_rows(rows, dt, simulator=False)
    fixed = _steady(simulated, dt)
    host = _steady(physical, dt)
    assert len(fixed) > 500
    assert abs(np.median([r["qd"][5] for r in fixed]) - 0.04) < 1e-8
    assert np.median([r["qd"][5] for r in host]) > 0.05
    metrics = motion_metrics(simulated, dt)
    assert metrics["velocity_residual_rms"][5] == 0
    assert metrics["native_period_max_ms"] > 5
    # A stalled host must still fail even though the simulated plant time is
    # perfectly contiguous. Correcting derivative time cannot waive liveness.
    stalled = [
        {**r, "elapsed_ns": r["elapsed_ns"] + (200_000_000 if i >= 300 else 0)}
        for i, r in enumerate(rows)
    ]
    with pytest.raises(ValueError, match="100 ms deadline"):
        motion_metrics(measurement_rows(stalled, dt, simulator=True), dt)
