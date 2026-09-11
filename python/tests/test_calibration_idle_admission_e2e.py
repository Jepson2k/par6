"""Idle admission may await fresh native status; live freshness stays strict."""

import asyncio
import json
import time

import numpy as np
import pytest
from live_daemon import LiveDaemon, _set_scalar

from par6 import AsyncRobotClient
from par6._par6 import ControllerMode, calibration_config
from par6.calibration import CalibrationSession, capture
from par6.calibration.profiles import atomic_json
from par6.calibration.session import feedback_age


@pytest.mark.e2e
@pytest.mark.timeout(90)
async def test_position_waits_for_new_idle_status_but_fresh_stays_strict(
    tmp_path, monkeypatch
):
    """Use real sparse UDP telemetry, native models and an invalid finite target.

    No motion is needed: the native geometry refusal must happen only after
    position() consumes a genuinely newer packet than the stale cached one.
    """
    trace = tmp_path / "capture.bin"
    monkeypatch.setenv("PAR6_DIAGNOSTICS", str(trace))
    live = LiveDaemon.start(
        tmp_path / "daemon",
        config_patch=lambda source: calibration_config(
            _set_scalar(_set_scalar(source, "tick_dt_s", 0.004), "status_rate_hz", 50),
            exec_limits=[[0.2, 0.4, 1.2]] * 6,
            stream_limits=[[0.2, 0.4, 1.2]] * 6,
        ),
    )
    client = AsyncRobotClient(
        host="127.0.0.1",
        port=live.command_port,
        status_port=live.status_port,
        status_transport="unicast",
    )
    evidence = {"simulator_only": True, "valid": False}
    evidence_path = tmp_path / "idle-admission.json"
    try:
        assert await client.wait_ready(timeout=10)
        assert await client.reset() == 1
        assert await client.set_gravity_comp(False) == 1
        assert await client.teleport([0, -90, 170, 0, -20, 180]) == 1
        core = await client._ensure_core()
        sequence = -1
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            status = await core.status_after(sequence, 0.5)
            if status is None:
                continue
            sequence = status["seq"]
            if status["homed"] and status["mode"] == ControllerMode.IDLE:
                try:
                    capture.assert_live(trace, capture.metadata(trace)["fingerprint"])
                except RuntimeError:
                    continue
                break
        else:
            raise AssertionError("Native simulator did not establish its reference")

        # Construct the actual Preview/GravityModel/CollisionWorld through the
        # ordinary session at its normal rate before selecting sparse STATUS.
        async with CalibrationSession(client, tmp_path / "run", trace) as session:
            await session.active_support()
            invalid = session.start.copy()
            invalid[2] = session.window[2, 0] - 0.01
            assert np.isfinite(invalid).all()
            begin = capture.length(trace)
            assert await client.set_status_rate(5) == 1
            rate = await client.status_rate()
            assert rate is not None and rate.hz == 5

            async def stale_cached_packet():
                # Wait to a receipt-derived age, then verify the actual packet
                # is still cached and newer than the session's last observation.
                # A new arrival or scheduling overrun restarts the selection.
                latest = core.latest_status()
                cursor = latest["seq"]
                until = time.monotonic() + 5
                while time.monotonic() < until:
                    packet = await core.status_after(cursor, 0.5)
                    if packet is None:
                        continue
                    cursor = packet["seq"]
                    age = feedback_age(packet)
                    if age < 0.125:
                        await asyncio.sleep(0.125 - age)
                    cached = core.latest_status()
                    if (
                        cached is not None
                        and cached["seq"] == packet["seq"]
                        and cached["seq"] > session.sequence
                        and 0.12 <= feedback_age(cached) <= 0.16
                    ):
                        assert cached["mode"] == ControllerMode.IDLE
                        assert not cached["gravity_comp"]
                        assert not cached["queued_segments"]
                        return cached
                raise AssertionError("Could not select an actually stale cached packet")

            try:
                # The readiness reader used during motion must retain its
                # original 100 ms refusal even though idle admission may wait.
                stale = await stale_cached_packet()
                evidence["strict_reader"] = {
                    "stale_seq": stale["seq"],
                    "age_ms": 1000 * feedback_age(stale),
                }
                with pytest.raises(RuntimeError, match="feedback_age_ms"):
                    await session.fresh()
                assert session.sequence == stale["seq"]

                stale = await stale_cached_packet()
                evidence["position"] = {
                    "previous_session_seq": session.sequence,
                    "stale_seq": stale["seq"],
                    "age_ms": 1000 * feedback_age(stale),
                    "target_rad": invalid.tolist(),
                }
                try:
                    await session.position(invalid)
                except (RuntimeError, ValueError) as exc:
                    evidence["position"].update(
                        exception_type=type(exc).__name__, exception=str(exc)
                    )
                else:
                    evidence["position"]["exception_type"] = None
                evidence["position"]["admitted_seq"] = session.sequence
                evidence["trial_count"] = len(session.trials)
            finally:
                # Restore normal-rate feedback before the context's genuine
                # support/Stop cleanup, including the expected before-fix failure.
                assert await client.set_status_rate(50) == 1
                restored = await client.status_rate()
                assert restored is not None and restored.hz == 50
                prior = core.latest_status()
                resumed = await core.status_after(prior["seq"], 0.5)
                assert resumed is not None and feedback_age(resumed) <= 0.1
                await session.fresh()

        # Observe recorded native ticks beyond context exit so omitted STREAM
        # actuation is established by the recorder, not by mock call counts.
        tail_begin = capture.length(trace)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            packet = await core.status_after(sequence, 0.5)
            if packet is None:
                continue
            sequence = packet["seq"]
            dt, tail = capture.read_capture(trace, tail_begin)
            if tail and (tail[-1]["tick"] - tail[0]["tick"]) * dt >= 0.2:
                break
        else:
            raise AssertionError(
                "Native recorder did not cover the post-admission tail"
            )
        _, rows = capture.read_capture(trace, begin)
        evidence["capture_start"] = begin
        evidence["capture_end"] = begin + len(rows)
        evidence["non_idle_samples"] = sum(
            row["flags"] >> 8 != ControllerMode.IDLE for row in rows
        )
        atomic_json(evidence_path, evidence)
        result = evidence["position"]
        assert result["exception_type"] == "ValueError", evidence
        assert "joint window" in result["exception"], evidence
        assert result["admitted_seq"] > result["stale_seq"], evidence
        assert evidence["trial_count"] == 0, evidence
        assert rows and evidence["non_idle_samples"] == 0, evidence
        evidence["valid"] = True
        atomic_json(evidence_path, evidence)
        print(json.dumps({"report": str(evidence_path), "capture": str(trace)}))
    finally:
        atomic_json(evidence_path, evidence)
        try:
            await client.close()
        finally:
            live.stop()
