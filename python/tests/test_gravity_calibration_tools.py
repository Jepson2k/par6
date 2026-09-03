"""The gravity identification harness against the torque-level simulator:
the sweep recovers the model it is driven by, the float catches a model
that is wrong, the sign probe refuses an armed drive, and the twin
evidence compares the scene with the runtime. What passes here runs on
the bench unchanged."""

from __future__ import annotations

import importlib.util
import json
import sys
import time
from pathlib import Path

import pytest
from live_daemon import LiveDaemon, requires_par6d
from test_client_sync import sync_client

pytestmark = [pytest.mark.slow, requires_par6d]

TOOLS = Path(__file__).resolve().parents[2] / "tools" / "gravity_calibration"
HOLD_POSE_DEG = [0.0, -75.0, 305.0, 20.0, -30.0, 180.0]


def load(name: str):
    if str(TOOLS) not in sys.path:
        sys.path.insert(0, str(TOOLS))
    spec = importlib.util.spec_from_file_location(name, TOOLS / f"{name}.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def fast_tick(text: str) -> str:
    """The rig's CI tick (50 ms) is too coarse to integrate the torque
    plant faithfully; these tests run it at 10 ms."""
    assert "tick_dt_s = 0.05" in text
    return text.replace("tick_dt_s = 0.05", "tick_dt_s = 0.01")


def drift_lock(text: str) -> str:
    for old, new in [
        ("drift_lock = false", "drift_lock = true"),
        ("release_rad_s = 0.08", "release_rad_s = 1.0"),
        ("settle_s = 0.3", "settle_s = 0.2"),
    ]:
        assert old in text
        text = text.replace(old, new)
    return text


def place(client, angles_deg: list[float], budget_s: float = 20.0) -> None:  # noqa: ANN001
    """Reset and teleport until the broadcast shows the arm there. On the
    torque plant the loaded joints give way a little before the
    feedforward catches them, so the tolerance is a few degrees."""
    deadline = time.monotonic() + budget_s
    client.reset()
    while time.monotonic() < deadline:
        client.teleport(angles_deg)
        if client.wait_status(
            lambda s: (
                s.homed and all(abs(a - b) < 3.0 for a, b in zip(s.angles, angles_deg))
            ),
            timeout=1.0,
        ):
            return
    raise AssertionError(f"the sim arm never reached {angles_deg}")


def connection(daemon: LiveDaemon) -> list[str]:
    return [
        "--host",
        "127.0.0.1",
        "--port",
        str(daemon.command_port),
        "--telemetry-port",
        str(daemon.telemetry_port),
        "--status-port",
        str(daemon.status_port),
        "--status-transport",
        "UNICAST",
        "--json",
    ]


def ledger(out: str) -> dict:
    return json.loads(out.strip().splitlines()[-1])


def checks(out: str) -> dict[str, dict]:
    return {c["name"]: c for c in ledger(out)["checks"]}


@pytest.mark.timeout(300)
def test_the_sweep_runs_the_staircase_and_the_fit_reads_it(tmp_path, capsys):
    """The staircase steps, dwells and logs rest samples the fit can read
    cleanly: the phase is resolved, the zero offset is nil and the fit is
    tight. What the simulator cannot establish is the model SCALE: the
    torque plant holds a joint by clamping its velocity, so its reported
    holding current at rest is not tied to gravity — the vendor fixture
    and the twin residual pin the model instead, and ``k`` is a hardware
    result. The sweep must also leave the arm where it found it."""
    d = LiveDaemon.start(tmp_path, sim_dynamics=True, config_patch=fast_tick)
    try:
        with sync_client(d) as client:
            assert client.wait_ready(timeout=10.0) is True
            place(client, HOLD_POSE_DEG)
        conn = connection(d)
        rc = load("pd_sweep_id").main(
            conn
            + [
                "--go",
                "--joint",
                "2",
                "--range",
                "0.8",
                "--speed",
                "0.15",
                "--out",
                str(tmp_path),
                "--pose-tol-deg",
                "10",
            ]
        )
        out = capsys.readouterr().out
        assert rc == 0, out
        sweep = checks(out)
        assert sweep["returned to the start pose"]["ok"], out
        files = sorted(tmp_path.glob("pdsweep_J2_*.json"))
        assert len(files) == 1
        record = json.loads(files[0].read_text())
        assert {s["dir"] for s in record["samples"]} == {1, -1}
        load("fit_sweeps").main([str(files[0]), "--json"])
        capsys.readouterr()
        fit = json.loads(files[0].with_name(files[0].stem + "_fit.json").read_text())
        assert fit["usable"] and fit["range_rad"] >= 0.5
        assert abs(fit["offset_rad"]) < 0.05, fit
        assert fit["rms_measured_nm"] < 0.05, fit

        # Sabotage the velocity guard: an impossible limit must abort the
        # sweep, freeze with a hold and bring the arm back under control.
        rc = load("pd_sweep_id").main(
            conn
            + [
                "--go",
                "--joint",
                "2",
                "--range",
                "0.8",
                "--speed",
                "0.15",
                "--vel-abort",
                "0.001",
                "--out",
                str(tmp_path / "sabotage"),
                "--pose-tol-deg",
                "10",
            ]
        )
        out = capsys.readouterr().out
        assert rc == 1, out
        assert "abort" in checks(out) and not checks(out)["abort"]["ok"], out
        assert checks(out)["returned to the start pose"]["ok"], out
    finally:
        d.stop()


@pytest.mark.timeout(240)
def test_the_float_catches_a_biased_model_and_the_lock_reads_its_size(tmp_path, capsys):
    """A payload the controller believes in but the plant does not is a
    wrong gravity model. Without the drift lock the float drifts and the
    script fails; with it the drift is bounded and the integral reports
    the bias — the empirical validation of the lock."""
    conn_args = [
        "--go",
        "--joint",
        "2",
        "--lift",
        "0",
        "--float-s",
        "8",
        "--out",
        str(tmp_path),
        "--pose-tol-deg",
        "10",
    ]

    d = LiveDaemon.start(tmp_path / "free", sim_dynamics=True, config_patch=fast_tick)
    try:
        with sync_client(d) as client:
            assert client.wait_ready(timeout=10.0) is True
            place(client, HOLD_POSE_DEG)
            client.set_payload(0.2, (0.0, 0.0, 0.02))
        rc = load("auto_float_test").main(connection(d) + conn_args)
        out = capsys.readouterr().out
        assert rc == 1, out
        assert "drift bounded" in checks(out), out
        assert not checks(out)["drift bounded"]["ok"], out
    finally:
        d.stop()

    d = LiveDaemon.start(
        tmp_path / "locked",
        sim_dynamics=True,
        config_patch=lambda text: drift_lock(fast_tick(text)),
    )
    try:
        with sync_client(d) as client:
            assert client.wait_ready(timeout=10.0) is True
            place(client, HOLD_POSE_DEG)
            client.set_payload(0.2, (0.0, 0.0, 0.02))
        rc = load("auto_float_test").main(connection(d) + conn_args)
        out = capsys.readouterr().out
        assert rc == 0, out
        assert "drift bounded" in checks(out), out
        result = json.loads(
            sorted((tmp_path).glob("autofloat_J2_*.json"))[-1].read_text()
        )
        assert result["drift_lock"] is True
        # The phantom payload pulls the model's elbow torque negative; the
        # lock's integral climbs the other way to cancel it, slowly.
        assert result["integral_tail_nm"][2] > 0.03, result["integral_tail_nm"]
        assert checks(out)["drift bounded"]["ok"], out
    finally:
        d.stop()


@pytest.mark.timeout(120)
def test_the_sign_probe_refuses_an_armed_drive(tmp_path, capsys):
    d = LiveDaemon.start(tmp_path)
    try:
        rc = load("phase_a_sign_probe").main(connection(d) + ["--duration", "1"])
        out = capsys.readouterr().out
        assert rc == 0, out
        assert checks(out)["drives idle"]["ok"]
        with sync_client(d) as client:
            assert client.wait_ready(timeout=10.0) is True
            client.reset()
            assert client.wait_status(lambda s: s.enabled, timeout=5.0)
        rc = load("phase_a_sign_probe").main(connection(d) + ["--duration", "1"])
        out = capsys.readouterr().out
        assert rc == 1, out
        assert not checks(out)["drives idle"]["ok"]
    finally:
        d.stop()


@pytest.mark.timeout(240)
def test_the_twin_evidence_compares_the_scene_with_the_runtime(tmp_path, capsys):
    pytest.importorskip("mujoco")
    out_path = tmp_path / "twin.json"
    rc = load("twin_evidence").main(["--poses", "3", "--out", str(out_path), "--json"])
    out = capsys.readouterr().out
    result = json.loads(out_path.read_text())
    assert len(result["poses"]) >= 5
    assert checks(out)["timestep converged at the scene's 1 ms"]["ok"], out
    # The scene and the runtime agree on the load-bearing joints; whether
    # the wrist rows agree is the finding the script exists to report,
    # and its exit code carries it.
    shoulder, elbow = result["per_joint"][1], result["per_joint"][2]
    assert shoulder["loaded_poses"] > 0 and elbow["loaded_poses"] > 0
    assert shoulder["correlation"] > 0.9 and elbow["correlation"] > 0.9, result[
        "per_joint"
    ]
    assert "scene gravity sign convention is consistent per joint" in checks(out)
    assert rc == (1 if ledger(out)["failed"] else 0), out
