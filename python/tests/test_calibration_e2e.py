"""Real client, native capture, simulator plant, and interrupted experiment."""

import asyncio
import gc
import json
import time
import tomllib

import numpy as np
import pytest
from live_daemon import LiveDaemon, _set_scalar
from waldoctl import Box

from par6 import AsyncRobotClient
from par6._par6 import calibration_config
from par6.calibration import (
    CalibrationSession,
    export_profile,
    motion_envelope,
    verify_applied,
)
from par6.calibration.feedback import temporary_feedback
from par6.calibration.session import TrialRejected
from par6.calibration.trajectory import move


@pytest.mark.e2e
@pytest.mark.timeout(90)
async def test_calibration_capture_and_cancel_leave_no_motion(tmp_path, monkeypatch):
    trace = tmp_path / "capture.bin"
    monkeypatch.setenv("PAR6_DIAGNOSTICS", str(trace))
    correction = [0.0] * 24
    correction[10] = 0.01
    scale = [1, 1, 1.01, 1, 1, 1]
    live = LiveDaemon.start(
        tmp_path / "daemon",
        config_patch=lambda s: (
            f"gravity_correction = {correction}\n"
            + _set_scalar(
                _set_scalar(
                    calibration_config(
                        f"gravity_scale = {scale}\n" + s,
                        stream_limits=[[0.3, 0.6, 2.0]] * 6,
                    ),
                    "tick_dt_s",
                    0.004,
                ),
                "status_rate_hz",
                50,
            )
        ),
    )
    client = AsyncRobotClient(
        host="127.0.0.1",
        port=live.command_port,
        status_port=live.status_port,
        status_transport="unicast",
    )
    try:
        core = await client._ensure_core()
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            s = await core.status_after(-1, 0.5)
            if s and s["link_ok"]:
                break
        await client.reset()
        target = [0, -90, 180, 0, -20, 180]
        await client.teleport(target)
        sequence = s["seq"]
        while time.monotonic() < deadline:
            s = await core.status_after(sequence, 0.2)
            if s:
                sequence = s["seq"]
            if (
                s
                and s["homed"]
                and max(abs(a - b) for a, b in zip(s["angles"], target)) < 0.1
            ):
                break
        else:
            raise AssertionError("Simulator did not establish the starting pose")
        # Exercise obstacle declarations with the installed waldoctl wire format.
        fixture = Box(
            name="surveyed-fixture", x=0.1, y=0.1, z=0.1, pose=(3, 3, 0, 0, 0, 0)
        )
        assert await client.set_shapes([fixture]) == 1
        world = await client.shapes()
        assert world is not None
        assert [shape.name for shape in world.program] == [fixture.name]
        if hasattr(fixture, "attachment"):
            from dataclasses import replace

            from waldoctl import Attachment

            with pytest.raises(ValueError, match="does not support shape attachments"):
                await client.set_shapes(
                    [replace(fixture, attachment=Attachment(epoch=1))]
                )
            world = await client.shapes()
            assert [shape.name for shape in world.program] == [fixture.name]
        async with CalibrationSession(client, tmp_path / "run", trace) as run:
            await run.active_support()
            assert not (await run.fresh())["gravity_comp"]
            # The wider physical approach touched an unmodelled table. With
            # installation ground present, refuse the whole path before servo.
            table_target = np.deg2rad([100.30, -58.99, 141.69, 25.81, -2.13, 157.08])
            t, positions = move(run.start, table_target, [[0.2, 0.4, 1.2]] * 6)
            before = await core.status_after(run.sequence, 0.5)
            with pytest.raises(ValueError, match="collides"):
                await run.stimulus("table-approach", t, positions)
            after = await core.status_after(before["seq"], 0.5)
            assert after["mode"] == before["mode"]
            assert np.max(np.abs(np.array(after["angles"]) - before["angles"])) < 0.1
            assert not run.trials
            recorded_world = json.loads((run.directory / "world.json").read_text())
            assert any(s["collision"] for s in recorded_world["installation"])
            report = {
                "valid": True,
                "baseline_fingerprint": run.fingerprint,
                "identity": run.identity,
            }
            candidate = export_profile(
                tmp_path / "profile", run.bundle, report, gravity=[0.0] * 24
            )
            assert tomllib.loads(candidate.read_text())["gravity_scale"] == [1] * 6
            await verify_applied(
                client, candidate.parent.parent / "rollback" / candidate.name
            )
            with pytest.raises(RuntimeError, match="not loaded"):
                await verify_applied(client, candidate)
            q = run.start.copy()
            q[0] += 0.02
            t, positions = move(run.start, q, [[0.05, 0.1, 0.3]] * 6)
            collector_enabled = gc.isenabled()
            rows, metrics = await run.stimulus("small-move", t, positions)
            assert gc.isenabled() == collector_enabled
            assert len(rows) > 100
            row = rows[len(rows) // 2]
            nominal = np.asarray(run.model.gravity(row["q"]))
            delta = np.asarray(run.model.regressor(row["q"])) @ correction
            assert np.max(np.abs(delta)) > 0.05
            assert np.allclose(row["gravity_nm"], (nominal + delta) * scale, atol=0.005)
            assert not np.allclose(row["gravity_nm"], nominal, atol=0.005)
            assert not metrics["faulted"]
            assert abs(rows[-1]["q"][0] - q[0]) < 0.02
            long = q.copy()
            long[0] += 0.2
            t, positions = move(q, long, [[0.04, 0.1, 0.3]] * 6)

            async def tuned_move():
                async with temporary_feedback(run, 0, velocity_scale=0.6):
                    await run.stimulus("cancel", t, positions)

            task = asyncio.create_task(tuned_move())
            start_seq = s["seq"]
            # Cancel after fresh feedback proves the stream has started.
            while True:
                s = await core.status_after(start_seq, 0.5)
                assert s
                start_seq = s["seq"]
                if s["mode"] == 5:
                    break
            assert not gc.isenabled()
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task
            assert gc.isenabled() == collector_enabled
            restored = json.loads(
                (run.directory / "feedback-J1-restore.json").read_text()
            )
            assert restored["restored"]
            assert restored["candidate"]["kpv"] != restored["baseline"]["kpv"]
            s = await core.status_after(start_seq, 0.5)
            assert s and s["queued_segments"] == 0
            assert max(abs(v) for v in s["speeds"]) < 0.03
            gc.disable()
            try:
                first = await motion_envelope(run, max_trials=1)
                assert not gc.isenabled()
            finally:
                if collector_enabled:
                    gc.enable()
            assert not first["valid"] and not first["complete"]
            checkpoint = run.directory / "envelope-progress.json"
            count = sum(
                len(v)
                for v in json.loads(checkpoint.read_text())["measurements"].values()
            )
            assert count == 1
            readings = json.loads(checkpoint.read_text())["measurements"]
            first_reading = next(v[0] for v in readings.values() if v)
            assert first_reading["accepted"], first_reading
            assert first_reading["achieved"] >= 0.09
            second = await motion_envelope(run, max_trials=1, resume=checkpoint)
            assert not second["valid"] and not second["complete"]
            assert (
                sum(
                    len(v)
                    for v in json.loads(checkpoint.read_text())["measurements"].values()
                )
                == 2
            )
            assert all(
                r["accepted"]
                for readings in json.loads(checkpoint.read_text())[
                    "measurements"
                ].values()
                for r in readings
            )
            assert not (run.directory / "motion-profile").exists()
            invalid = json.loads(checkpoint.read_text())
            invalid["identity"]["simulator"] = False
            checkpoint.write_text(json.dumps(invalid))
            with pytest.raises(ValueError, match="different runtime"):
                await motion_envelope(run, max_trials=1, resume=checkpoint)
    finally:
        await client.close()
        live.stop()


@pytest.mark.e2e
@pytest.mark.timeout(90)
@pytest.mark.parametrize("scale,expected", [(1.0, True), (0.1, False)])
async def test_gravity_only_validation_detects_bad_model_and_leaves_active_hold(
    tmp_path, monkeypatch, scale, expected
):
    trace = tmp_path / "gravity.bin"
    monkeypatch.setenv("PAR6_DIAGNOSTICS", str(trace))
    live = LiveDaemon.start(
        tmp_path / "daemon",
        config_patch=lambda s: calibration_config(
            f"gravity_scale = [1, 1, {scale}, 1, 1, 1]\n"
            + _set_scalar(_set_scalar(s, "tick_dt_s", 0.004), "status_rate_hz", 50),
            stream_limits=[[0.2, 0.4, 1.2]] * 6,
        ),
    )
    client = AsyncRobotClient(
        host="127.0.0.1",
        port=live.command_port,
        status_port=live.status_port,
        status_transport="unicast",
    )
    try:
        core = await client._ensure_core()
        deadline = time.monotonic() + 15
        sequence = -1
        while time.monotonic() < deadline:
            s = await core.status_after(sequence, 0.5)
            if s:
                sequence = s["seq"]
                if s["link_ok"]:
                    break
        assert await client.reset() >= 0
        assert await client.set_gravity_comp(False) >= 0
        assert await client.teleport([0, -90, 170, 0, -20, 180]) >= 0
        while time.monotonic() < deadline:
            s = await core.status_after(sequence, 0.5)
            if s:
                sequence = s["seq"]
                if s["homed"] and s["mode"] == 1:
                    break
        else:
            raise AssertionError("Simulator did not establish the reference")
        async with CalibrationSession(client, tmp_path / "run", trace) as run:
            await run.stop()
            result = await run.gravity_hold()
            assert result["valid"] is expected, result
            assert run.trials[-1]["rest_confirmed"]
            if expected:
                assert result["duration_s"] >= 20
            else:
                assert any("drift" in r or "excursion" in r for r in result["reasons"])
                assert result["duration_s"] < 20
            status = await run.fresh()
            assert status["mode"] == 1 and not status["gravity_comp"]
            assert not status["queued_segments"]
            assert max(abs(v) for v in status["speeds"]) < 0.03
            evidence = json.loads((run.directory / "trials.json").read_text())
            assert evidence["trials"][-1]["metrics"]["valid"] is expected
            if expected:
                # Cancellation must also leave the drive actively supporting
                # the arm; Stop alone would restore the original freedrive law.
                assert await client.set_gravity_comp(False) >= 0
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline:
                    status = await run.fresh()
                    if not status["gravity_comp"]:
                        break
                else:
                    raise AssertionError(
                        "Could not establish active hold before cancellation test"
                    )
                task = asyncio.create_task(run.gravity_hold(stage="cancel"))
                sequence = status["seq"]
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    status = await core.status_after(sequence, 0.5)
                    assert status
                    sequence = status["seq"]
                    if status["mode"] == 1 and status["gravity_comp"]:
                        break
                else:
                    raise AssertionError("Cancelled hold never activated")
                task.cancel()
                with pytest.raises(asyncio.CancelledError):
                    await task
                status = await run.fresh()
                assert status["mode"] == 1 and not status["gravity_comp"]
                assert max(abs(v) for v in status["speeds"]) < 0.03
                assert run.trials[-1]["active_hold_confirmed"]
                assert "CancelledError" in run.trials[-1]["error"]
                # The reviewer reproduced rough motion after this mode handoff.
                # Retain its rejection; a quiet preceding hold must not let
                # subsequent excessive measured motion into calibration data.
                next_pose = run.start.copy()
                next_pose[[1, 2, 4]] += 0.3
                with pytest.raises(
                    TrialRejected, match="velocity residual|vibration|tracking error"
                ):
                    await run.position(next_pose)
                assert run.trials[-1]["live_acceptance"]["valid"] is False
                status = await run.fresh()
                assert not status["queued_segments"]
                assert not status["gravity_comp"]
                assert max(abs(v) for v in status["speeds"]) < 0.03
    finally:
        await client.close()
        live.stop()


@pytest.mark.e2e
@pytest.mark.timeout(150)
async def test_integral_tuning_controls_moving_vibration_and_restores_response(
    tmp_path, monkeypatch
):
    from par6.calibration.routines import verification_centers
    from par6.calibration.trajectory import sweeps

    trace = tmp_path / "integral.bin"
    monkeypatch.setenv("PAR6_DIAGNOSTICS", str(trace))
    live = LiveDaemon.start(
        tmp_path / "daemon",
        config_patch=lambda s: calibration_config(
            _set_scalar(_set_scalar(s, "tick_dt_s", 0.004), "status_rate_hz", 50),
            stream_limits=[[0.2, 0.4, 1.2]] * 6,
        ),
    )
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
        assert await client.reset() >= 0
        assert await client.set_gravity_comp(False) >= 0
        assert await client.teleport([0, -90, 170, 0, -20, 180]) >= 0
        while time.monotonic() < deadline:
            status = await core.status_after(sequence, 0.5)
            if status:
                sequence = status["seq"]
                if status["homed"] and status["mode"] == 1:
                    break
        else:
            raise AssertionError("Simulator did not establish reference")
        async with CalibrationSession(client, tmp_path / "run", trace) as run:
            await run.active_support()
            plans = [
                p
                for p in sweeps(
                    verification_centers(run.start),
                    np.minimum(run.limits, [0.05, 0.08, 0.3]),
                    amplitude=0.18,
                )
                if p["group"] == 0 and p["joint"] == 1
            ]

            async def baseline():
                p = plans[0]
                await run.position(p["start"])
                with pytest.raises(TrialRejected, match="velocity residual|vibration"):
                    await run.stimulus("unstable-integral", p["times"], p["positions"])
                assert run.trials[-1]["live_acceptance"]["valid"] is False

            await baseline()
            async with temporary_feedback(run, 1, integral_scale=0.4):
                for p in plans:
                    await run.position(p["start"])
                    _, metrics = await run.stimulus(
                        "damped-integral", p["times"], p["positions"]
                    )
                    assert metrics["velocity_residual_rms"][1] < 0.01
                    assert metrics["tracking_peak_deg"][1] < 0.1
            # Prove restoration by actual controller response, not only a JSON
            # flag or an acknowledged write to a one-way drive config command.
            await baseline()
            status = await run.fresh()
            assert not status["queued_segments"] and not status["gravity_comp"]
            assert max(abs(v) for v in status["speeds"]) < 0.03
    finally:
        await client.close()
        live.stop()
