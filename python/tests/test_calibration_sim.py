"""The four routines end to end against a real `par6d --sim` with its
recorder on: entry/preflight, streamed and queued stimuli, Stop and rest on
every exit, gain restoration, and the staged patch."""

import asyncio
import json
import time
import tomllib

import numpy as np
import pytest
from live_daemon import LiveDaemon, _set_scalar, requires_par6d

from par6 import AsyncRobotClient
from par6._par6 import calibration_config
from par6.calibration import (
    Session,
    check,
    gravity,
    limits,
    tune_feedback,
)
from par6.calibration.trajectory import move

START_DEG = [0, -90, 170, 0, -20, 180]


def start_daemon(tmp_path, monkeypatch, *, stream_caps=True, damped=False):
    """The sim at the calibration cadence. `damped` loads the J2/J3 integral
    reductions tune-feedback selects on this plant: the gains a gravity or
    limits run needs to move without the baseline's J2 oscillation."""
    trace = tmp_path / "capture.bin"
    monkeypatch.setenv("PAR6_DIAGNOSTICS", str(trace))

    def patch(text):
        text = _set_scalar(_set_scalar(text, "tick_dt_s", 0.004), "status_rate_hz", 50)
        gains = None
        if damped:
            robot = tomllib.loads(text)
            gains = [
                [j["gains"][k] for k in ("kpp", "kpv", "kiv")] for j in robot["joints"]
            ]
            gains[1][2] *= 0.8
            gains[2][2] *= 0.8
        return calibration_config(
            text,
            stream_limits=[[0.2, 0.4, 1.2]] * 6 if stream_caps else None,
            feedback_gains=gains,
        )

    live = LiveDaemon.start(tmp_path / "daemon", config_patch=patch)
    client = AsyncRobotClient(
        host="127.0.0.1",
        port=live.command_port,
        status_port=live.status_port,
        status_transport="unicast",
    )
    return live, client, trace


async def ready_at_start(client):
    """Link up, drives enabled, gravity feedforward off, the arm referenced
    at START_DEG. Teleport is fire-and-forget, so it is repeated until the
    status stream shows the pose."""
    core = await client._ensure_core()
    deadline = time.monotonic() + 30
    seq = -1
    while time.monotonic() < deadline:
        s = await core.status_after(seq, 0.5)
        if s:
            seq = s["seq"]
            if s["link_ok"]:
                break
    else:
        raise AssertionError("The sim bus never came up")
    assert await client.reset() >= 0
    assert await client.set_gravity_comp(False) >= 0
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        assert await client.teleport(START_DEG) >= 0
        settled = time.monotonic() + 1.0
        while time.monotonic() < settled:
            s = await core.status_after(seq, 0.5)
            if not s:
                continue
            seq = s["seq"]
            if (
                s["homed"]
                and s["mode"] == 1
                and not s["gravity_comp"]
                and max(abs(a - b) for a, b in zip(s["angles"], START_DEG)) < 0.1
            ):
                return core
    raise AssertionError("Simulator did not establish the starting pose")


@requires_par6d
@pytest.mark.e2e
@pytest.mark.timeout(240)
async def test_check_runs_and_a_cancelled_stimulus_still_stops_and_records(
    tmp_path, monkeypatch
):
    live, client, trace = start_daemon(tmp_path, monkeypatch)
    try:
        core = await ready_at_start(client)
        async with Session(client, tmp_path / "run", trace) as session:
            report = await check(session, joints=(1,), amplitude=0.04)
            assert report["valid"], report["reasons"]
            assert (session.directory / "check.json").is_file()
            # A stimulus cancelled mid-stream: the arm is stopped and at rest,
            # the trial is recorded with its error, and the next trial works.
            target = session.start.copy()
            target[1] += 0.15
            t, q = move(session.start, target, [[0.05, 0.08, 0.3]] * 6)
            with pytest.raises(asyncio.TimeoutError):
                await asyncio.wait_for(session.stimulus("cancelled", t, q), timeout=1.0)
            s = await session.fresh()
            assert s["mode"] == 1 and not s["queued_segments"]
            assert max(abs(v) for v in s["speeds"]) < 0.03
            trials = json.loads((session.directory / "trials.json").read_text())[
                "trials"
            ]
            assert (
                trials[-1]["name"] == "cancelled" and "Cancelled" in trials[-1]["error"]
            )
            await session.position(session.start)
            after = np.deg2rad((await session.fresh())["angles"])
            assert np.max(np.abs(after - session.start)) < 0.02
        s = await core.status_after(-1, 0.5)
        assert s["mode"] == 1 and not s["gravity_comp"]
    finally:
        await client.close()
        live.stop()


@requires_par6d
@pytest.mark.e2e
@pytest.mark.slow
@pytest.mark.timeout(600)
async def test_tune_feedback_selects_a_damped_integral_and_restores_the_baseline(
    tmp_path, monkeypatch
):
    live, client, trace = start_daemon(tmp_path, monkeypatch)
    try:
        await ready_at_start(client)
        async with Session(client, tmp_path / "run", trace) as session:
            baseline_kiv = session.robot["joints"][1]["gains"]["kiv"]
            report = await tune_feedback(session, joint=1, amplitude=0.18)
            assert report["valid"], report["reasons"]
            assert report["candidate"]["kiv"] < baseline_kiv
            assert (
                report["candidate"]["kpp"] == session.robot["joints"][1]["gains"]["kpp"]
            )
            assert (
                report["patch"]["feedback_gains"]["1"][2] == report["candidate"]["kiv"]
            )
            record = json.loads((session.directory / "gains-J2.json").read_text())
            assert record["restored"] is True
            # The baseline response is back: the same sweep carries the
            # baseline's velocity residual again, well above the candidate's.
            target = session.start.copy()
            target[1] += 0.18
            await session.position(session.start)
            t, q = move(
                session.start, target, np.minimum(session.limits, [0.05, 0.08, 0.3])
            )
            _, metrics = await session.stimulus(
                "baseline-again", t, q, settle=False, allow_oscillation=True
            )
            label = next(
                k for k in report["measurements"] if k.startswith("validation-kpv")
            )
            candidate_rms = np.mean(
                [
                    m["motion"]["velocity_residual_rms"][1]
                    for m in report["measurements"][label]
                ]
            )
            assert metrics["velocity_residual_rms"][1] > 3 * candidate_rms, (
                metrics["velocity_residual_rms"][1],
                candidate_rms,
            )
    finally:
        await client.close()
        live.stop()


@requires_par6d
@pytest.mark.e2e
@pytest.mark.slow
@pytest.mark.timeout(1800)
async def test_gravity_identification_stages_a_candidate_the_plant_confirms(
    tmp_path, monkeypatch
):
    live, client, trace = start_daemon(tmp_path, monkeypatch, damped=True)
    try:
        await ready_at_start(client)
        async with Session(client, tmp_path / "run", trace) as session:
            report = await gravity(session)
            assert report["valid"], report["reasons"]
            fit = report["fit"]
            assert max(fit["validation_after_nm"]) <= 0.05
            assert len(report["patch"]["gravity"]) == 24
            candidate = tmp_path / "run" / "profile" / "candidate" / "PAR6.toml"
            assert candidate.is_file()
    finally:
        await client.close()
        live.stop()


@requires_par6d
@pytest.mark.e2e
@pytest.mark.slow
@pytest.mark.timeout(600)
async def test_limits_probe_stages_exec_limits_for_one_joint(tmp_path, monkeypatch):
    live, client, trace = start_daemon(
        tmp_path, monkeypatch, stream_caps=False, damped=True
    )
    try:
        await ready_at_start(client)
        async with Session(client, tmp_path / "run", trace) as session:
            report = await limits(session, joints=(0,))
            assert report["valid"], report["reasons"]
            j0 = report["per_joint"]["0"]
            assert j0["best_speed_fraction"] >= 0.25
            v, a, jerk = report["proposed_exec_limits"][0]
            assert 0 < v <= session.robot["joints"][0]["limits"]["velocity_rad_s"]
            assert jerk == pytest.approx(3 * a)
            # A dimension the travel exercised is proposed from the measurement;
            # one it did not keeps its configured value rather than shrinking.
            for dim, name in ((0, "velocity_exercised"), (1, "acceleration_exercised")):
                configured = report["configured_exec_limits"][0][dim]
                if j0[name]:
                    assert report["proposed_exec_limits"][0][dim] < configured
                else:
                    assert report["proposed_exec_limits"][0][dim] == configured
            assert report["coupled"] and all(
                c["rejected"] is None for c in report["coupled"]
            )
            if "patch" in report:
                assert report["patch"]["exec_limits"][0] == [v, a, jerk]
    finally:
        await client.close()
        live.stop()
