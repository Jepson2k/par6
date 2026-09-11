"""Real gravity-toggle acknowledgement loss must leave a latched supported stop."""

import asyncio
import json
import time

import pytest
from live_daemon import LiveDaemon, requires_par6d, settle_at
from test_calibration_acknowledgements import ReplyLossRelay

from par6 import AsyncRobotClient
from par6._par6 import ControllerMode
from par6.calibration.profiles import atomic_json
from par6.calibration.session import CalibrationSession, feedback_age

pytestmark = [pytest.mark.e2e, requires_par6d]


@pytest.mark.timeout(45)
@pytest.mark.parametrize("drop_number", [1, 2], ids=["gravity-ack", "stop-ack"])
async def test_lost_support_ack_still_latches_and_confirms_supported_stop(
    calibration_daemon: LiveDaemon, tmp_path, drop_number
):
    """Lose the actual daemon ACK, never invent or modify protocol messages.

    With retries=0 one lost reply is all of that request's replies.
    Subsequent EStop and Stop requests/replies pass unchanged. STATUS uses
    its independent native port. No motion or parameter-identification fixture
    is substituted for the real active_support lifecycle.
    """
    daemon = calibration_daemon
    relay = ReplyLossRelay(("127.0.0.1", daemon.command_port))
    transport, _ = await asyncio.get_running_loop().create_datagram_endpoint(
        lambda: relay, local_addr=("127.0.0.1", 0)
    )
    client = AsyncRobotClient(
        host="127.0.0.1",
        port=transport.get_extra_info("sockname")[1],
        status_transport="unicast",
        status_port=daemon.status_port,
        timeout=0.5,
        retries=0,
    )
    evidence = {"simulator_only": True, "valid": False}
    evidence_path = tmp_path / "support-loss-evidence.json"
    try:
        assert await client.wait_ready(timeout=10)
        await settle_at(client, [0, -90, 180, 0, -20, 180])
        assert await client.set_gravity_comp(True) == 1
        core = await client._ensure_core()
        sequence = -1
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            status = await core.status_after(sequence, 0.5)
            if status is None:
                continue
            sequence = status["seq"]
            if (
                feedback_age(status) <= 0.1
                and status["mode"] == ControllerMode.IDLE
                and status["gravity_comp"]
                and status["enabled"]
                and status["homed"]
            ):
                break
        else:
            raise AssertionError("Native G-enabled entry state was not observed")

        # Only the real cleanup helper is under test, not model loading or a
        # fitting run. Its core is the actual connected native client.
        session = CalibrationSession(
            client, tmp_path / "support", tmp_path / "unused.bin"
        )
        session.directory.mkdir()
        session.core = core
        session.sequence = sequence
        relay.drop_reply(drop_number)
        try:
            await session.active_support()
        except RuntimeError as exc:
            evidence["raised"] = str(exc)
        else:
            evidence["raised"] = None
        evidence["lost_daemon_replies"] = len(relay.dropped)
        record_path = session.directory / "active-support.json"
        evidence["support_record"] = (
            json.loads(record_path.read_text()) if record_path.exists() else None
        )

        # Observe the latch and encoder rest after helper return. No reset is
        # sent; the native status timestamp measures this persistence interval.
        deadline = time.monotonic() + 2
        observed = []
        first_supported_ns = None
        while time.monotonic() < deadline:
            status = await core.status_after(session.sequence, 0.5)
            if status is None:
                continue
            session.sequence = status["seq"]
            if feedback_age(status) > 0.1:
                continue
            supported = (
                status["mode"] == ControllerMode.ACTIVE_ERROR
                and not status["enabled"]
                and bool(status["error"])
                and status["link_ok"] == 1
                and not status["queued_segments"]
                and max(abs(v) for v in status["speeds"]) < 0.03
            )
            observed.append(
                {
                    "seq": status["seq"],
                    "mono_time_ns": status["mono_time_ns"],
                    "mode": int(status["mode"]),
                    "enabled": bool(status["enabled"]),
                    "error": bool(status["error"]),
                    "speeds": list(status["speeds"]),
                    "supported_rest": supported,
                }
            )
            if supported:
                if first_supported_ns is None:
                    first_supported_ns = status["mono_time_ns"]
                if status["mono_time_ns"] - first_supported_ns >= 400_000_000:
                    break
            else:
                first_supported_ns = None
        evidence["observed"] = observed
        atomic_json(evidence_path, evidence)
        assert evidence["raised"] is not None, evidence
        assert evidence["lost_daemon_replies"] == 1, evidence
        assert observed and all(row["supported_rest"] for row in observed), evidence
        assert observed[-1]["mono_time_ns"] - observed[0]["mono_time_ns"] >= 400_000_000
        record = evidence["support_record"]
        assert record is not None, evidence
        assert record["gravity_disabled_confirmed"] is (drop_number == 2), record
        assert record["software_estop_acknowledged"], record
        assert record["software_estop_observed"], record
        assert record["stop_confirmed"], record
        expected_error = (
            "Gravity disable was not acknowledged"
            if drop_number == 1
            else "Controller stop was not acknowledged"
        )
        assert any(expected_error in text for text in record["errors"]), record
        evidence["valid"] = True
        atomic_json(evidence_path, evidence)
        print(json.dumps({"report": str(evidence_path)}))
    finally:
        relay.drop_number = None
        try:
            # Queue clearing must not reset the software latch. The fixture
            # terminates its isolated daemon after the client has closed.
            assert await client.stop() == 1
        finally:
            atomic_json(evidence_path, evidence)
            try:
                await client.close()
            finally:
                transport.close()
                await relay.closed
