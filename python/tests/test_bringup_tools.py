"""The bring-up kit against the simulator: the offline limiter stage
needs nothing, the on-arm stages run against ``par6d --sim`` exactly as
they would against hardware. A stage that fails here fails on the
bench too."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest
from live_daemon import LiveDaemon, requires_par6d
from test_client_sync import park_deg, settle_at, sync_client

pytestmark = [pytest.mark.slow, requires_par6d]

KIT = Path(__file__).resolve().parents[2] / "tools" / "bringup"


def load(name: str):
    if str(KIT) not in sys.path:
        sys.path.insert(0, str(KIT))
    spec = importlib.util.spec_from_file_location(name, KIT / f"{name}.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def daemon(tmp_path):
    d = LiveDaemon.start(tmp_path)
    try:
        with sync_client(d) as client:
            assert client.wait_ready(timeout=10.0) is True
            settle_at(client, park_deg())
        yield d
    finally:
        d.stop()


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
    ]


def test_limiter_preview_runs_offline_and_passes_on_the_shipped_config(capsys):
    assert load("limiter_preview").main(["--seconds", "4"]) == 0
    out = capsys.readouterr().out
    assert "FAIL" not in out and "soft-limit clamp" in out


@pytest.mark.timeout(300)
def test_on_arm_stages_pass_on_the_simulator(daemon, capsys):
    """stack verification, first motion, multi-joint independence, the
    loop benchmark and ladder rungs 3–10 — every one from the canonical
    pose, every one with ``--go``."""
    conn = connection(daemon)
    assert load("stack_verify").main(conn + ["--go"]) == 0, capsys.readouterr().out
    assert (
        load("first_motion").main(conn + ["--go", "--amplitude", "4", "--period", "3"])
        == 0
    ), capsys.readouterr().out
    assert load("multi_joint").main(conn + ["--go"]) == 0, capsys.readouterr().out
    assert load("loop_benchmark").main(conn + ["--seconds", "2"]) == 0, (
        capsys.readouterr().out
    )
    assert load("acceptance_ladder").main(conn + ["--go", "--from", "3"]) == 0, (
        capsys.readouterr().out
    )


def test_without_go_nothing_moves(daemon, capsys):
    conn = connection(daemon)
    # One STATUS listener at a time on a unicast port: read, close, run.
    with sync_client(daemon) as client:
        before = client.angles()
    assert before is not None
    assert load("first_motion").main(conn) == 0
    assert load("acceptance_ladder").main(conn + ["--from", "3", "--to", "3"]) == 0
    with sync_client(daemon) as client:
        after = client.angles()
    assert after == pytest.approx(before, abs=0.05)
    assert "pass --go" in capsys.readouterr().out
