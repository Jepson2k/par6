"""Configured recorder capacity must bound disk use without stopping the runtime."""

import os
import subprocess
import time
from pathlib import Path

import pytest
from live_daemon import LiveDaemon, par6d_binary, requires_par6d

from par6.calibration import capture

pytestmark = requires_par6d


@pytest.mark.e2e
@pytest.mark.timeout(30)
async def test_recorder_honors_its_configured_sample_budget(tmp_path, monkeypatch):
    trace = tmp_path / "capture.bin"
    limit = 80
    monkeypatch.setenv("PAR6_DIAGNOSTICS", str(trace))
    monkeypatch.setenv("PAR6_DIAGNOSTICS_MAX_SAMPLES", str(limit))
    live = LiveDaemon.start(tmp_path / "daemon")
    try:
        async with live.client() as client:
            assert await client.wait_ready(timeout=10)
            core = await client._ensure_core()
            identity = (await core.capture_info())["identity"]
            assert identity["pid"] == live.process.pid
            sequence = -1
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline:
                status = await core.status_after(sequence, 0.5)
                assert status is not None
                sequence = status["seq"]
                if capture.length(trace) >= limit:
                    break
            else:
                pytest.fail("Recorder did not reach the configured sample budget")
            assert capture.length(trace) == limit
            # Advancing live STATUS is the barrier: the controller keeps ticking
            # after capture fills, but the evidence cannot silently keep growing.
            for _ in range(25):
                status = await core.status_after(sequence, 0.5)
                assert status is not None and status["seq"] != sequence
                sequence = status["seq"]
                assert capture.length(trace) == limit
            with pytest.raises(RuntimeError, match="stopped or is stale"):
                capture.assert_live(
                    trace, identity["fingerprint"], expected_identity=identity
                )
            assert live.process.poll() is None
            assert (
                trace.stat().st_size
                == capture.HEADER_SIZE + limit * capture.DTYPE.itemsize
            )
    finally:
        live.stop()


@pytest.mark.e2e
def test_recording_budget_rejects_invalid_values_and_cli_overrides_env(tmp_path):
    binary = par6d_binary()
    config = Path(__file__).resolve().parents[2] / "config/PAR6.toml"
    command = [binary, "--sim", "--check-config", "--config", str(config)]
    env = dict(os.environ)
    env["PAR6_DIAGNOSTICS"] = str(tmp_path / "unused.bin")
    for value in ("0", "-1", "NaN", "inf", "1.5", str(2**64)):
        for source in ("environment", "flag"):
            args = command.copy()
            env.pop("PAR6_DIAGNOSTICS_MAX_SAMPLES", None)
            if source == "environment":
                env["PAR6_DIAGNOSTICS_MAX_SAMPLES"] = value
            else:
                args += ["--diagnostics-max-samples", value]
            result = subprocess.run(
                args, env=env, capture_output=True, text=True, timeout=10
            )
            assert result.returncode != 0, (source, value, result.stdout)
            assert "not a positive sample count" in result.stderr
            assert "PAR6D_READY" not in result.stdout
    env["PAR6_DIAGNOSTICS_MAX_SAMPLES"] = "invalid"
    result = subprocess.run(
        command + ["--diagnostics-max-samples", "3600000"],
        env=env,
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 0, result.stderr
    assert not (tmp_path / "unused.bin").exists()
