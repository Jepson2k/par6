"""Queued calibration requires real completion, endpoint hold and ordinary gravity."""

import asyncio
import json
import time
import tomllib

import numpy as np
import pytest
from live_daemon import LiveDaemon, _set_scalar, requires_par6d

from par6 import AsyncRobotClient
from par6._par6 import ControllerMode, calibration_config
from par6.calibration import CalibrationSession
from par6.calibration.execution import execution_preview, queued_trial
from par6.calibration.session import finish_cleanup

pytestmark = [pytest.mark.e2e, requires_par6d]


@pytest.mark.timeout(120)
async def test_queued_verification_observes_complete_and_refuses_an_interrupted_hold(
    tmp_path, monkeypatch
):
    """Use the isolated native controller for admission, execution and Stop.

    A second ordinary client operation stops after the actual COMPLETE verdict.
    There is no invented acknowledgement, completion index or fake session.
    """
    trace = tmp_path / "capture.bin"
    monkeypatch.setenv("PAR6_DIAGNOSTICS", str(trace))

    def config(source):
        robot = tomllib.loads(source)
        gains = [
            [
                j["gains"]["kpp"],
                j["gains"]["kpv"],
                j["gains"]["kiv"] * (0.8 if i in (1, 2) else 1),
            ]
            for i, j in enumerate(robot["joints"])
        ]
        return calibration_config(
            _set_scalar(_set_scalar(source, "tick_dt_s", 0.004), "status_rate_hz", 50),
            stream_limits=[[0.2, 0.4, 1.2]] * 6,
            exec_limits=[[0.16, 0.32, 0.96]] * 6,
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
        assert await client.teleport([0, -90, 170, 0, -20, 180]) == 1
        assert await client.select_profile("RUCKIG") == 1
        assert await client.set_gravity_comp(True) == 1
        while time.monotonic() < deadline:
            status = await core.status_after(sequence, 0.5)
            if status:
                sequence = status["seq"]
                if status["homed"] and status["mode"] == ControllerMode.IDLE:
                    break
        else:
            pytest.fail("Simulator did not establish its reference")

        async with CalibrationSession(client, tmp_path / "run", trace) as run:
            preview = await execution_preview(run)
            required = np.arange(6) == 0
            target = run.start.copy()
            target[0] += 0.25
            # These invalid masks must fail before a motion is admitted.
            for mask in ([True] * 5, [1] * 6, [False] * 6):
                with pytest.raises(ValueError, match="joint mask"):
                    await queued_trial(run, preview, target, mask)
            assert not run.trials
            rows, metrics = await queued_trial(run, preview, target, required)
            assert metrics["acceptance"]["valid"], metrics["acceptance"]
            assert metrics["hold"]["duration_s"] >= 1.99
            assert np.all(np.asarray(metrics["achieved"])[0] >= 0.9 * run.limits[0])
            modes = np.asarray([r["flags"] >> 8 for r in rows])
            first = int(np.flatnonzero(modes == ControllerMode.EXEC)[0])
            assert np.all(modes[first:] == ControllerMode.EXEC)
            completed = run.trials[-1]
            assert completed["command_index"] >= 0
            assert completed["stop_confirmed"]
            assert completed["post_completion_observation_s"] >= 2.5
            assert not completed["software_estop_latched"]

            async def interrupt_after_real_completion():
                seen = -1
                end = time.monotonic() + 15
                while time.monotonic() < end:
                    update = await core.status_after(seen, 0.5)
                    if update is None:
                        continue
                    seen = update["seq"]
                    if update["executing_index"] >= 0:
                        index = update["executing_index"]
                        assert await client.wait_command(index, timeout=10) is True
                        assert await client.stop() == 1
                        return index
                pytest.fail("No queued command entered execution")

            # The previous trial's Stop has confirmed empty queue and rest.
            interrupter = asyncio.create_task(interrupt_after_real_completion())
            try:
                with pytest.raises(RuntimeError, match="left EXEC|preserve EXEC"):
                    await queued_trial(run, preview, run.start, required)
                await interrupter
            finally:
                # This task may issue Stop but never starts motion. Drain it
                # before leaving the native session, even after an assertion.
                async def drain_interrupter():
                    return await asyncio.gather(interrupter, return_exceptions=True)

                await finish_cleanup(drain_interrupter())
            interrupted = run.trials[-1]
            assert interrupted["error"]
            assert interrupted["stop_confirmed"]
            assert "metrics" not in interrupted

            await run.active_support()
            previous = len(run.trials)
            with pytest.raises(RuntimeError, match="enabled gravity compensation"):
                await queued_trial(run, preview, target, required)
            assert len(run.trials) == previous
            status = await run.fresh()
            assert status["mode"] == ControllerMode.IDLE
            assert not status["queued_segments"]
            assert max(abs(v) for v in status["speeds"]) < 0.03
            saved = json.loads((run.directory / "trials.json").read_text())
            assert saved["trials"][-1]["stop_confirmed"]
    finally:
        await client.close()
        live.stop()
