"""Public recorded motion checks must retain feedback during their Stop intervals."""

import json
import time
import tomllib

import numpy as np
import pytest
from live_daemon import LiveDaemon, _set_scalar

from par6 import AsyncRobotClient
from par6._par6 import ControllerMode, calibration_config
from par6.calibration import CalibrationSession, capture, check_motion
from par6.calibration.profiles import atomic_json
from par6.calibration.session import feedback_age


@pytest.mark.e2e
@pytest.mark.timeout(90)
async def test_public_motion_check_from_gravity_idle_keeps_active_support(
    tmp_path, monkeypatch
):
    """Run the actual public workflow from its ordinary native G-enabled state.

    The caller deliberately does not call active_support(): the public routine
    owns its measured support mode and its Stop behavior. The plant and all
    replies are native; this test supplies no simulated measurement overrides.
    """
    trace = tmp_path / "capture.bin"
    monkeypatch.setenv("PAR6_DIAGNOSTICS", str(trace))

    def config(source):
        gains = [
            [
                joint["gains"]["kpp"],
                joint["gains"]["kpv"],
                joint["gains"]["kiv"] * (0.8 if i in (1, 2) else 1),
            ]
            for i, joint in enumerate(tomllib.loads(source)["joints"])
        ]
        return calibration_config(
            _set_scalar(_set_scalar(source, "tick_dt_s", 0.004), "status_rate_hz", 50),
            exec_limits=[[0.2, 0.4, 1.2]] * 6,
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
    evidence = {"simulator_only": True, "valid": False}
    evidence_path = tmp_path / "public-stream-support.json"
    try:
        core = await client._ensure_core()
        sequence = -1

        async def observe(predicate, timeout=10):
            nonlocal sequence
            until = time.monotonic() + timeout
            while time.monotonic() < until:
                status = await core.status_after(sequence, 0.5)
                if status is None:
                    continue
                sequence = status["seq"]
                if feedback_age(status) <= 0.1 and predicate(status):
                    return status
            raise AssertionError("Expected fresh native status was not observed")

        await observe(lambda status: status["link_ok"] == 1)
        assert await client.reset() == 1
        assert await client.teleport([0, -90, 170, 0, -20, 180]) == 1
        initial = await observe(
            lambda status: (
                status["homed"]
                and status["enabled"]
                and status["mode"] == ControllerMode.IDLE
            )
        )
        assert initial["gravity_comp"]
        evidence["initial_gravity_comp"] = True
        begin = capture.length(trace)
        workflow = None
        run = None
        try:
            async with CalibrationSession(client, tmp_path / "run", trace) as run:
                assert run.identity["simulator"]
                assert not run.robot.get("freedrive", {}).get("drift_lock", False)
                workflow = await check_motion(run, joints=(0,), amplitude=0.02)
        except (RuntimeError, ValueError, AssertionError) as exc:
            # Preserve the unsupported Stop evidence even if its transient
            # makes a preceding motion gate fail before the routine returns.
            evidence["workflow_error"] = f"{type(exc).__name__}: {exc}"
        terminal = await observe(
            lambda status: (
                status["mode"] == ControllerMode.IDLE
                and not status["queued_segments"]
                and max(abs(v) for v in status["speeds"]) < 0.03
            )
        )
        evidence["terminal_gravity_comp"] = bool(terminal["gravity_comp"])
        # Obtain a complete post-context tail after the final Stop. CAP2 tick
        # duration, not wall sleep, defines the observation interval.
        tail_begin = capture.length(trace)
        until = time.monotonic() + 5
        tail = []
        while time.monotonic() < until:
            await observe(lambda status: status["mode"] == ControllerMode.IDLE)
            dt, tail = capture.read_capture(trace, tail_begin)
            if tail and (tail[-1]["tick"] - tail[0]["tick"]) * dt >= 0.4:
                break
        else:
            raise AssertionError("Native recorder did not retain the final Stop tail")
        dt, raw = capture.read_capture(trace, begin)
        stream_indices = [
            i for i, row in enumerate(raw) if row["flags"] >> 8 == ControllerMode.STREAM
        ]
        after_entry = raw[stream_indices[0] :] if stream_indices else []
        idle = [r for r in after_entry if r["flags"] >> 8 == ControllerMode.IDLE]
        stream = [raw[i] for i in stream_indices]
        gravity_only = [r for r in idle if r["flags"] & 16]
        evidence.update(
            capture_start=begin,
            capture_end=begin + len(raw),
            native_stream_samples=len(stream),
            native_idle_samples_after_entry=len(idle),
            gravity_enabled_stream_samples=sum(bool(r["flags"] & 16) for r in stream),
            gravity_only_idle_samples=len(gravity_only),
            gravity_only_idle_missing_velocity_commands=sum(
                not np.isfinite(r["qd_commanded"]).all() for r in gravity_only
            ),
            final_tail_seconds=(tail[-1]["tick"] - tail[0]["tick"]) * dt,
            final_tail_gravity_enabled_samples=sum(bool(r["flags"] & 16) for r in tail),
            workflow=workflow,
            trials=[] if run is None else run.trials,
        )
        atomic_json(evidence_path, evidence)
        assert stream, evidence
        assert idle, evidence
        assert not gravity_only, evidence
        assert np.isfinite([r["qd_commanded"] for r in idle]).all(), evidence
        assert not evidence["terminal_gravity_comp"], evidence
        assert all(r["flags"] >> 8 == ControllerMode.IDLE for r in tail), evidence
        assert np.isfinite([r["qd_commanded"] for r in tail]).all(), evidence
        assert not any(r["flags"] & 16 for r in stream), evidence
        assert "workflow_error" not in evidence, evidence
        assert workflow is not None and workflow["valid"], evidence
        assert len(workflow["measurements"]) == 2
        evidence["valid"] = True
        atomic_json(evidence_path, evidence)
        print(json.dumps({"report": str(evidence_path), "capture": str(trace)}))
    finally:
        # This cleanup is after the evidence interval and cannot conceal the
        # public workflow's actual post-Stop commands in that recording.
        try:
            assert await client.set_gravity_comp(False) == 1
            assert await client.stop() == 1
        finally:
            atomic_json(evidence_path, evidence)
            try:
                await client.close()
            finally:
                live.stop()
