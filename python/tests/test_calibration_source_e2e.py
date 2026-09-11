"""Another live runtime must not supply a calibration session's evidence."""

import json
import time

import pytest
from live_daemon import LiveDaemon, _set_scalar

from par6 import AsyncRobotClient
from par6._par6 import calibration_config
from par6.calibration import CalibrationSession, capture


@pytest.mark.e2e
@pytest.mark.timeout(75)
async def test_calibration_rejects_another_live_recording(tmp_path, monkeypatch):
    lives, clients = [], []
    traces = [tmp_path / "a.bin", tmp_path / "b.bin"]

    def patch(source):
        return calibration_config(
            _set_scalar(_set_scalar(source, "tick_dt_s", 0.004), "status_rate_hz", 50),
            stream_limits=[[0.2, 0.4, 1.2]] * 6,
        )

    async def start(name, trace):
        if trace is None:
            monkeypatch.delenv("PAR6_DIAGNOSTICS", raising=False)
        else:
            monkeypatch.setenv("PAR6_DIAGNOSTICS", str(trace))
        live = LiveDaemon.start(tmp_path / name, config_patch=patch)
        lives.append(live)
        client = AsyncRobotClient(
            host="127.0.0.1",
            port=live.command_port,
            status_port=live.status_port,
            status_transport="unicast",
        )
        clients.append(client)
        core = await client._ensure_core()
        sequence = -1
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            status = await core.status_after(sequence, 0.5)
            if status:
                sequence = status["seq"]
                if status["link_ok"]:
                    break
        else:
            raise AssertionError("Simulator did not connect")
        assert await client.reset() == 1
        assert await client.set_gravity_comp(False) == 1
        assert await client.teleport([0, -90, 170, 0, -20, 180]) == 1
        while time.monotonic() < deadline:
            status = await core.status_after(sequence, 0.5)
            if status:
                sequence = status["seq"]
                if status["homed"] and status["mode"] == 1:
                    if trace is not None:
                        # Ready STATUS can precede the recorder's first flush.
                        # Wait on actual fresh-file evidence, not a fixed delay.
                        try:
                            capture.assert_live(
                                trace, capture.metadata(trace)["fingerprint"]
                            )
                        except RuntimeError:
                            continue
                    return client
        raise AssertionError("Simulator did not establish its reference")

    try:
        a = await start("daemon-a", traces[0])
        b = await start("daemon-b", traces[1])
        source_a, source_b = (
            (await a.capture_info())["identity"],
            (await b.capture_info())["identity"],
        )
        assert source_a["fingerprint"] == source_b["fingerprint"]
        assert source_a["pid"] != source_b["pid"]
        # Both files satisfy the old configuration/freshness gate. Source B
        # has identical nominal geometry and the same starting pose.
        for trace in traces:
            capture.assert_live(trace, source_a["fingerprint"])
        with pytest.raises(RuntimeError, match="another runtime instance"):
            async with CalibrationSession(a, tmp_path / "foreign", traces[1]):
                pass
        async with CalibrationSession(a, tmp_path / "own", traces[0]) as run:
            assert run.identity["recording"]["pid"] == lives[0].process.pid
            # A caller switching the file after entry must also fail before a
            # motion command can be sent.
            run.capture_path = traces[1]
            target = run.start.copy()
            target[0] += 0.01
            with pytest.raises(RuntimeError, match="another runtime instance"):
                await run.position(target)
            assert not run.trials
            run.capture_path = traces[0]
            # Inject a source switch only after the real acknowledged Stop.
            # The final assessment must reject and persist that reason too.
            original_stop = run.stop

            async def stop_then_switch_source():
                await original_stop()
                run.capture_path = traces[1]

            monkeypatch.setattr(run, "stop", stop_then_switch_source)
            try:
                with pytest.raises(RuntimeError, match="another runtime instance"):
                    await run.position(target)
                recorded = json.loads((run.directory / "trials.json").read_text())
                assert "another runtime instance" in recorded["trials"][-1]["error"]
                status = await run.fresh()
                assert not status["queued_segments"]
                assert max(abs(v) for v in status["speeds"]) < 0.03
            finally:
                monkeypatch.setattr(run, "stop", original_stop)
                run.capture_path = traces[0]
        off = await start("daemon-off", None)
        assert (await off.capture_info())["identity"] is None
        with pytest.raises(RuntimeError, match="no verifiable native recorder"):
            async with CalibrationSession(off, tmp_path / "off", traces[0]):
                pass
    finally:
        for client in clients:
            await client.close()
        for live in lives:
            live.stop()
