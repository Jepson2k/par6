"""Final envelope reporting uses real return motion and native profile export."""

import json
import time
import tomllib

import numpy as np
import pytest
from live_daemon import LiveDaemon, _set_scalar

from par6 import AsyncRobotClient
from par6._par6 import calibration_config
from par6.calibration import CalibrationSession, motion_envelope
from par6.calibration.profiles import atomic_json, validate_profile
from par6.calibration.routines import _finish_motion_envelope


@pytest.mark.e2e
@pytest.mark.timeout(90)
async def test_envelope_finalization_never_retains_a_failed_complete_report(
    tmp_path, monkeypatch
):
    """Exercise finalization, not the completion of a full envelope search.

    A phase report selects finalization; the actual return uses CalibrationSession
    and the real isolated controller. Failure comes from real path validation and
    the filesystem, without fabricated controller acknowledgements or a fake
    session. A prior valid JSON makes stale-result preservation observable.
    """
    trace = tmp_path / "capture.bin"
    monkeypatch.setenv("PAR6_DIAGNOSTICS", str(trace))

    def config(source):
        joints = tomllib.loads(source)["joints"]
        gains = [
            [
                joint["gains"]["kpp"],
                joint["gains"]["kpv"],
                joint["gains"]["kiv"] * (0.8 if i in (1, 2) else 1),
            ]
            for i, joint in enumerate(joints)
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
    operating = np.tile([0.16, 0.32, 0.96], (6, 1))
    failures = {}

    def phase_report(run):
        return {
            "kind": "motion-envelope",
            "valid": True,
            "complete": True,
            "identity": run.identity,
            "baseline_fingerprint": run.fingerprint,
            "limits": operating.tolist(),
        }

    def saved(run):
        return json.loads((run.directory / "motion-envelope.json").read_text())

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

        async with CalibrationSession(client, tmp_path / "failures", trace) as run:
            await run.active_support()
            path = run.directory / "motion-envelope.json"

            atomic_json(path, phase_report(run))
            with pytest.raises(ValueError, match="max_trials"):
                await motion_envelope(run, max_trials=0)
            failures["early argument failure"] = saved(run)

            atomic_json(path, phase_report(run))
            start = run.start.copy()
            run.start[0] = run.window[0, 1] + 0.001
            try:
                with pytest.raises(ValueError, match="joint window"):
                    await _finish_motion_envelope(run, phase_report(run), operating)
            finally:
                run.start = start
            failures["final return failure"] = saved(run)
            assert not (run.directory / "motion-profile").exists()

            atomic_json(path, phase_report(run))
            # The real exporter cannot create candidate/ beneath this file.
            (run.directory / "motion-profile").write_text("blocked directory")
            with pytest.raises(OSError):
                await _finish_motion_envelope(run, phase_report(run), operating)
            failures["export failure"] = saved(run)
            assert (run.directory / "motion-profile").is_file()

        async with CalibrationSession(client, tmp_path / "success", trace) as run:
            await run.active_support()
            result = await _finish_motion_envelope(run, phase_report(run), operating)
            assert result["valid"] and result["complete"]
            successful = saved(run)
            candidate = (
                run.directory
                / "motion-profile/candidate"
                / run.bundle["robot_filename"]
            )
            manifest = validate_profile(candidate)
            assert manifest["report"]["valid"]
            profile_json = candidate.parent.parent / "profile.json"
            original_manifest = profile_json.read_bytes()
            previous_trials = len(run.trials)
            try:
                await motion_envelope(run, max_trials=1)
            except RuntimeError as exc:
                reuse_error = str(exc)
            else:
                reuse_error = None
            reused = saved(run)
            assert profile_json.read_bytes() == original_manifest
            reuse_moved = len(run.trials) != previous_trials
            status = await run.fresh()
            assert not status["queued_segments"]
            assert max(abs(v) for v in status["speeds"]) < 0.03

        # Check all failure modes after their real lifecycle completed, so a
        # before-fix run retains evidence for every stale-result counterexample.
        invalid = {
            name: report
            for name, report in failures.items()
            if report.get("valid") is not False
            or report.get("complete") is not False
            or not report.get("error")
            or report.get("tested_mode") != "STREAM"
            or report.get("applied_validation_complete") is not False
        }
        assert not invalid, invalid
        assert successful["tested_mode"] == "STREAM"
        assert successful["applied_validation_complete"] is False
        assert reuse_error is not None and "profile" in reuse_error.lower()
        assert not reuse_moved
        assert not reused["valid"] and not reused["complete"]
        assert reused["error"]
    finally:
        await client.close()
        live.stop()
