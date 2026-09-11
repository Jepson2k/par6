"""Coupled envelope trials use the real six-axis simulator and capture."""

import json
import time
import tomllib

import numpy as np
import pytest
from live_daemon import LiveDaemon, _set_scalar

from par6 import AsyncRobotClient
from par6._par6 import calibration_config
from par6.calibration import CalibrationSession
from par6.calibration.routines import _combined_envelope
from par6.calibration.session import TrialRejected
from par6.calibration.trajectory import coupled_move


@pytest.mark.e2e
@pytest.mark.timeout(120)
async def test_coupled_trials_measure_all_axes_and_resume_the_budget(
    tmp_path, monkeypatch
):
    trace = tmp_path / "capture.bin"
    monkeypatch.setenv("PAR6_DIAGNOSTICS", str(trace))

    def config(source):
        joints = tomllib.loads(source)["joints"]
        gains = [
            [
                j["gains"]["kpp"],
                j["gains"]["kpv"],
                j["gains"]["kiv"] * (0.8 if i in (1, 2) else 1),
            ]
            for i, j in enumerate(joints)
        ]
        return calibration_config(
            _set_scalar(_set_scalar(source, "tick_dt_s", 0.004), "status_rate_hz", 50),
            stream_limits=[[0.2, 0.4, 1.2]] * 6,
            feedback_gains=gains,
        )

    live = LiveDaemon.start(tmp_path / "daemon", config_patch=config)
    client = AsyncRobotClient(
        host="127.0.0.1",
        port=live.command_port,
        status_port=live.status_port,
        status_transport="unicast",
    )
    try:
        core = await client._ensure_core()
        sequence = -1
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            status = await core.status_after(sequence, 0.5)
            if status:
                sequence = status["seq"]
                if status["link_ok"]:
                    break
        else:
            pytest.fail("Simulator did not establish feedback")
        assert await client.reset() == 1
        assert await client.set_gravity_comp(False) == 1
        assert await client.teleport([0, -90, 170, 0, -20, 180]) == 1
        while time.monotonic() < deadline:
            status = await core.status_after(sequence, 0.5)
            if status:
                sequence = status["seq"]
                if status["homed"] and status["mode"] == 1:
                    break
        else:
            pytest.fail("Simulator did not establish the reference")
        async with CalibrationSession(client, tmp_path / "run", trace) as session:
            await session.active_support()
            # A velocity cap too small for the requested acceleration leaves
            # this pulse family infeasible. Refuse before approaching.
            refused = await _combined_envelope(
                session,
                session.start,
                np.array([[0.02, 0.24, 0.8]] * 6),
                {},
                session.directory / "refused.json",
                1,
            )
            assert not refused["valid"] and refused["complete"]
            assert "cannot excite acceleration" in refused["reason"]
            assert not session.trials
            # The native limiter stays at (.2,.4,1.2); ordinary servo hints
            # must carry the lower limits through both movement and settling.
            limits = np.array([[0.1, 0.2, 0.6]] * 6)
            times, path = coupled_move(session.start, limits, session.window)
            await session.position(path[0])
            original_servo = client.servo_j

            async def lose_scale(angles, *, speed=1.0, accel=1.0):
                # Deliberately send unscaled commands through the real native
                # client: a passing preview must not certify this execution.
                return await original_servo(angles, speed=1.0, accel=1.0)

            with monkeypatch.context() as patch:
                patch.setattr(client, "servo_j", lose_scale)
                with pytest.raises(TrialRejected, match="Recorded native stream"):
                    await session.stimulus(
                        "lost-scale",
                        times,
                        path,
                        command_limits=limits,
                        speed=0.5,
                        accel=0.5,
                    )
            status = await session.fresh()
            assert not status["queued_segments"]
            assert max(abs(v) for v in status["speeds"]) < 0.03
            checkpoint = session.directory / "combined.json"
            state = {}
            first = await _combined_envelope(
                session, session.start, limits, state, checkpoint, 1
            )
            assert not first["valid"] and not first["complete"], first
            saved = json.loads(checkpoint.read_text())
            assert len(saved["combined_validation"]["measurements"]) == 1
            reading = next(iter(first["measurements"].values()))
            assert reading["accepted"], reading
            assert np.all(np.array(reading["achieved"]) >= 0.9 * limits)
            # All axes actually moved, rather than inheriting an unexercised
            # operating limit from a shoulder/elbow-only path.
            assert np.all(np.array(reading["metrics"]["peak_velocity_rad_s"]) > 0.05)
            second = await _combined_envelope(
                session, session.start, limits, saved, checkpoint, 1
            )
            assert not second["valid"] and not second["complete"], second
            assert len(second["measurements"]) == 2
            assert all(r["accepted"] for r in second["measurements"].values())
            assert (
                len([t for t in session.trials if t["name"] == "combined-validation"])
                == 2
            )
            status = await session.fresh()
            assert not status["queued_segments"]
            assert max(abs(v) for v in status["speeds"]) < 0.03
            assert not (session.directory / "motion-profile").exists()
    finally:
        await client.close()
        live.stop()
