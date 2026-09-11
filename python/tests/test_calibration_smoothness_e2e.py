"""The real smoothness workflow must scale commands and preserve partial evidence."""

import asyncio
import json
import time
import tomllib

import numpy as np
import pytest
from live_daemon import LiveDaemon, _set_scalar

from par6._par6 import calibration_config
from par6.calibration import CalibrationSession
from par6.calibration.routines import _smoothness_settings, smoothness


@pytest.mark.e2e
@pytest.mark.timeout(90)
async def test_smoothness_executes_scaled_candidate_and_records_interruption(
    tmp_path, monkeypatch
):
    trace = tmp_path / "capture.bin"
    monkeypatch.setenv("PAR6_DIAGNOSTICS", str(trace))

    def config(source):
        gains = [
            [
                j["gains"]["kpp"],
                j["gains"]["kpv"],
                j["gains"]["kiv"] * (0.8 if i in (1, 2) else 1),
            ]
            for i, j in enumerate(tomllib.loads(source)["joints"])
        ]
        return calibration_config(
            _set_scalar(_set_scalar(source, "tick_dt_s", 0.004), "status_rate_hz", 50),
            stream_limits=[[0.2, 0.4, 1.2]] * 6,
            jog_limits=[[0.05, 0.1, 0.3]] * 6,
            feedback_gains=gains,
        )

    live = LiveDaemon.start(tmp_path / "daemon", config_patch=config)
    try:
        async with live.client() as client:
            assert await client.wait_ready(timeout=10)
            assert await client.reset() == 1
            assert await client.set_gravity_comp(False) == 1
            assert await client.teleport([0, -90, 170, 0, -20, 180]) == 1
            core = await client._ensure_core()
            sequence = -1
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                status = await core.status_after(sequence, 0.5)
                if status:
                    sequence = status["seq"]
                    if status["homed"] and status["mode"] == 1:
                        break
            else:
                pytest.fail("Simulator did not establish its reference")
            async with CalibrationSession(client, tmp_path / "run", trace) as session:
                candidate_done = asyncio.Event()
                original = session.stimulus

                async def observe(name, *args, **kwargs):
                    result = await original(name, *args, **kwargs)
                    if name == "candidate":
                        candidate_done.set()
                    return result

                monkeypatch.setattr(session, "stimulus", observe)
                operation = asyncio.create_task(smoothness(session))
                observed = asyncio.create_task(candidate_done.wait())
                try:
                    done, _ = await asyncio.wait(
                        {operation, observed},
                        timeout=45,
                        return_when=asyncio.FIRST_COMPLETED,
                    )
                    if operation in done:
                        await operation
                        pytest.fail(
                            "Smoothness completed before the first candidate barrier"
                        )
                    assert observed in done, (
                        "No candidate completed within the test budget"
                    )
                    operation.cancel()
                    with pytest.raises(asyncio.CancelledError):
                        await operation
                finally:
                    for task in (operation, observed):
                        if not task.done():
                            task.cancel()
                    await asyncio.gather(operation, observed, return_exceptions=True)
                await session.stop()
                status = await session.fresh()
                assert not status["queued_segments"]
                assert max(abs(v) for v in status["speeds"]) < 0.03
                report = json.loads((session.directory / "smoothness.json").read_text())
                assert not report["valid"] and not report["complete"]
                assert report["status"] == "failed"
                assert any("CancelledError" in reason for reason in report["reasons"])
                assert not (session.directory / "smooth-profile").exists()
                readings = report["measurements"]["candidate"]
                assert readings and all(r["acceptance"]["valid"] for r in readings)
                _, caps = _smoothness_settings(session.robot, session.limits)
                for reading in readings:
                    assert np.all(
                        np.asarray(reading["commanded_peaks"]) <= caps * 1.001 + 1e-6
                    )
                    assert reading["group"] == 0 and not reading["held_out"]
    finally:
        live.stop()
