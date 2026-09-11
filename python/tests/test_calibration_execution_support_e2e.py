"""Real queued entry/exit must retain feedback while switching gravity support."""

import asyncio
import gc
import json
import time
import tomllib

import numpy as np
import pytest
from live_daemon import LiveDaemon, _set_scalar

from par6 import AsyncRobotClient
from par6._par6 import CompletionPolicy, ControllerMode, calibration_config
from par6.calibration import CalibrationSession, capture
from par6.calibration.profiles import atomic_json
from par6.calibration.session import finish_cleanup
from par6.calibration.validation import assess_motion


@pytest.mark.e2e
@pytest.mark.timeout(120)
async def test_paused_exec_entry_and_supported_exit_never_release_to_gravity_idle(
    tmp_path, monkeypatch
):
    """Exercise actual native modes, acknowledgements, capture and simulator plant.

    This establishes a supported handoff only if its measured transitions also
    pass the ordinary motion gates. It is not authorization for physical release
    or evidence that a physical arm has the simulator's transient response.
    """
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
    report = {
        "simulator_only": True,
        "valid": False,
        "phases": {},
        "cleanup_errors": [],
    }
    report_path = tmp_path / "execution-support.json"
    try:
        core = await client._ensure_core()
        sequence = -1
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            status = await core.status_after(sequence, 0.5)
            if status is not None:
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
            if status is not None:
                sequence = status["seq"]
                if (
                    status["homed"]
                    and status["mode"] == ControllerMode.IDLE
                    and not status["gravity_comp"]
                ):
                    try:
                        capture.assert_live(
                            trace, capture.metadata(trace)["fingerprint"]
                        )
                    except RuntimeError:
                        continue
                    break
        else:
            pytest.fail("Simulator did not establish supported IDLE")

        async with CalibrationSession(client, tmp_path / "run", trace) as session:
            assert session.identity["simulator"]
            assert not session.robot.get("freedrive", {}).get("drift_lock", False)
            await session.active_support()
            assert await client.select_profile("RUCKIG") == 1
            assert await client.profile() == "RUCKIG"
            assert await client.set_completion_policy(CompletionPolicy.STRICT) == 1
            target = session.start.copy()
            target[2] += 0.08
            session.check_path([session.start, target])
            report["target_rad"] = target.tolist()
            begin = capture.length(trace)
            report["capture_start"] = begin
            submission = completion = None
            gc_enabled = gc.isenabled()
            if gc_enabled:
                gc.disable()

            async def confirm(
                mode, gravity, paused, *, executing_index=None, timeout=5
            ):
                until = time.monotonic() + timeout
                while time.monotonic() < until:
                    observed = await session.fresh()
                    if (
                        observed["mode"] == mode
                        and bool(observed["gravity_comp"]) == gravity
                        and bool(observed["paused"]) == paused
                        and (
                            executing_index is None
                            or observed["executing_index"] == executing_index
                        )
                    ):
                        return observed
                raise AssertionError("Requested supported mode was not observed")

            async def hold(name, *, mode, gravity, paused, seconds, start=None):
                # Count actual captured plant ticks, not an arbitrary sleep.
                cursor = capture.length(trace) if start is None else start
                first = last = None
                until = time.monotonic() + seconds + 5
                while time.monotonic() < until:
                    observed = await session.fresh()
                    assert observed["mode"] == mode
                    assert bool(observed["gravity_comp"]) == gravity
                    assert bool(observed["paused"]) == paused
                    end = capture.length(trace)
                    dt, rows = capture.read_capture(trace, cursor, end)
                    cursor = end
                    for row in rows:
                        if (
                            row["flags"] >> 8 == mode
                            and bool(row["flags"] & 16) == gravity
                        ):
                            first = row["tick"] if first is None else first
                            last = row["tick"]
                    if first is not None and (last - first) * dt >= seconds:
                        phase = {"first_tick": first, "last_tick": last}
                        report["phases"][name] = phase
                        return
                raise AssertionError(f"Native capture did not cover {name}")

            async def cleanup():
                async def attempt(label, operation):
                    try:
                        await operation
                    except BaseException as exc:
                        report["cleanup_errors"].append(
                            f"{label}: {type(exc).__name__}: {exc}"
                        )

                async def disable_gravity():
                    assert await client.set_gravity_comp(False) == 1
                    until = time.monotonic() + 5
                    while time.monotonic() < until:
                        if not (await session.fresh())["gravity_comp"]:
                            return
                    raise AssertionError("Gravity disable was not observed before Stop")

                # Keep the actual request alive while draining; do not mistake
                # cancellation of a Python Future for native transaction join.
                await attempt("disable gravity", disable_gravity())
                if submission is not None:
                    await attempt("submission", asyncio.shield(submission))
                if completion is not None and not completion.done():
                    completion.cancel()
                    await asyncio.gather(completion, return_exceptions=True)
                await attempt("confirmed Stop", session.stop())

                async def restore_pause():
                    assert await client.resume() == 1
                    await confirm(ControllerMode.IDLE, False, False)

                await attempt("restore pause", restore_pause())
                if report["cleanup_errors"]:
                    raise RuntimeError("; ".join(report["cleanup_errors"]))

            try:
                assert await client.pause() == 1
                await confirm(ControllerMode.IDLE, False, True)
                submission = asyncio.create_task(
                    client.move_j(
                        np.rad2deg(target).tolist(), speed=1, accel=1, r=0, wait=False
                    )
                )
                while not submission.done():
                    await session.fresh()
                index = submission.result()
                assert index >= 0
                report["command_index"] = index
                # The wire index comes from the server's PlanStarted event,
                # independently of the RT paused ring's active command index.
                # Wait for both observations rather than racing their delivery.
                paused = await confirm(
                    ControllerMode.EXEC, False, True, executing_index=index
                )
                report["paused_executing_index"] = paused["executing_index"]
                await hold(
                    "paused_without_gravity",
                    mode=ControllerMode.EXEC,
                    gravity=False,
                    paused=True,
                    seconds=0.4,
                )
                enable_begin = capture.length(trace)
                assert await client.set_gravity_comp(True) == 1
                await confirm(ControllerMode.EXEC, True, True)
                await hold(
                    "enable_gravity_while_paused",
                    mode=ControllerMode.EXEC,
                    gravity=True,
                    paused=True,
                    seconds=1.5,
                    start=enable_begin,
                )
                assert await client.resume() == 1
                resumed = await confirm(
                    ControllerMode.EXEC, True, False, executing_index=index
                )
                report["resumed_executing_index"] = resumed["executing_index"]
                completion = asyncio.create_task(client.wait_command(index, timeout=15))
                until = time.monotonic() + 16
                while not completion.done():
                    assert time.monotonic() < until
                    observed = await session.fresh()
                    assert observed["mode"] == ControllerMode.EXEC
                    assert observed["gravity_comp"] and not observed["paused"]
                assert completion.result() is True
                report["exact_complete"] = True
                await hold(
                    "completed_endpoint",
                    mode=ControllerMode.EXEC,
                    gravity=True,
                    paused=False,
                    seconds=1.5,
                )
                disable_begin = capture.length(trace)
                assert await client.set_gravity_comp(False) == 1
                await confirm(ControllerMode.EXEC, False, False)
                await hold(
                    "disable_gravity_while_holding",
                    mode=ControllerMode.EXEC,
                    gravity=False,
                    paused=False,
                    seconds=1.5,
                    start=disable_begin,
                )
                # Pause survives queue clearing; restore it explicitly after
                # confirming Stop instead of relying on flush to reset it.
                assert await client.pause() == 1
                await confirm(ControllerMode.EXEC, False, True)
                await session.stop()
                await confirm(ControllerMode.IDLE, False, True)
                report["paused_after_stop"] = True
                assert await client.resume() == 1
                await confirm(ControllerMode.IDLE, False, False)
                await hold(
                    "supported_idle",
                    mode=ControllerMode.IDLE,
                    gravity=False,
                    paused=False,
                    seconds=0.4,
                )
            except BaseException as exc:
                report["error"] = f"{type(exc).__name__}: {exc}"
                raise
            finally:
                try:
                    await finish_cleanup(cleanup())
                    report["stop_confirmed"] = True
                finally:
                    if gc_enabled:
                        gc.enable()
                    end = capture.length(trace)
                    report["capture_end"] = end
                    dt, raw = capture.read_capture(trace, begin, end)
                    report["native_samples"] = len(raw)
                    report["gravity_only_idle_ticks"] = sum(
                        r["flags"] >> 8 == ControllerMode.IDLE and bool(r["flags"] & 16)
                        for r in raw
                    )
                    rows = capture.measurement_rows(raw, dt, simulator=True)
                    controlled = [
                        r for r in rows if r["flags"] >> 8 == ControllerMode.EXEC
                    ]
                    try:
                        report["execution_quality"] = assess_motion(
                            controlled, dt, session.policy
                        )
                        for name, phase in report["phases"].items():
                            if name not in (
                                "enable_gravity_while_paused",
                                "disable_gravity_while_holding",
                            ):
                                continue
                            selected = [
                                r
                                for r in rows
                                if phase["first_tick"]
                                <= r["tick"]
                                <= phase["last_tick"]
                            ]
                            phase["quality"] = assess_motion(
                                selected, dt, session.policy
                            )
                            phase["excursion_deg"] = np.rad2deg(
                                np.ptp([r["q"] for r in selected], axis=0)
                            ).tolist()
                    except (ValueError, RuntimeError) as exc:
                        report["analysis_error"] = f"{type(exc).__name__}: {exc}"
                    atomic_json(report_path, report)

            assert report["exact_complete"] and report["stop_confirmed"]
            assert report["paused_after_stop"]
            assert report["gravity_only_idle_ticks"] == 0
            enabled = [r for r in raw if r["flags"] & 16]
            assert enabled
            assert all(r["flags"] >> 8 == ControllerMode.EXEC for r in enabled)
            assert np.isfinite([r["q_commanded"] for r in enabled]).all()
            assert np.isfinite([r["qd_commanded"] for r in enabled]).all()
            assert max(abs(r["qd_commanded"][2]) for r in enabled) > 0.03
            assert "analysis_error" not in report, report.get("analysis_error")
            for name in (
                "enable_gravity_while_paused",
                "disable_gravity_while_holding",
            ):
                assert report["phases"][name]["quality"]["acceptance"]["valid"], report[
                    "phases"
                ][name]
            assert report["execution_quality"]["acceptance"]["valid"], report[
                "execution_quality"
            ]["acceptance"]
    except BaseException as exc:
        report.setdefault("error", f"{type(exc).__name__}: {exc}")
        raise
    finally:
        try:
            try:
                await client.close()
            finally:
                live.stop()
        except BaseException as exc:
            report["teardown_error"] = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            atomic_json(report_path, report)
    report["valid"] = True
    atomic_json(report_path, report)
    print(
        json.dumps({"valid": True, "capture": str(trace), "report": str(report_path)})
    )
