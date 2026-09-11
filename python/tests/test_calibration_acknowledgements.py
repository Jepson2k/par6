"""Calibration restoration over real UDP with selected daemon replies lost."""

from __future__ import annotations

import asyncio
import json
import tomllib

import pytest
from live_daemon import LiveDaemon, requires_par6d, settle_at

from par6.calibration.feedback import temporary_feedback
from par6.calibration.session import CalibrationSession
from par6.client import AsyncRobotClient

pytestmark = [pytest.mark.e2e, requires_par6d]


class ReplyLossRelay(asyncio.DatagramProtocol):
    """Forward opaque datagrams; optionally discard one daemon reply.

    STATUS has its own port and bypasses this relay. No reply is generated
    here, and no protocol payload is decoded or changed.
    """

    def __init__(self, upstream):
        self.upstream = upstream
        self.downstream = None
        self.transport = None
        self.drop_number = None
        self.reply_count = 0
        self.dropped = []
        self.closed = asyncio.get_running_loop().create_future()

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        if addr == self.upstream:
            self.reply_count += 1
            if self.reply_count == self.drop_number:
                self.dropped.append(data)
                return
            if self.downstream is not None:
                self.transport.sendto(data, self.downstream)
        else:
            self.downstream = addr
            self.transport.sendto(data, self.upstream)

    def connection_lost(self, exc):
        if not self.closed.done():
            self.closed.set_result(None)

    def drop_reply(self, number):
        self.reply_count = 0
        self.drop_number = number


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("drop_number", "message"),
    [
        (1, "Controller stop was not acknowledged"),
        (2, "Controller did not acknowledge feedback update"),
    ],
    ids=["stop-acknowledgement", "gain-acknowledgement"],
)
async def test_lost_restoration_acknowledgement_is_not_reported_as_restored(
    calibration_daemon: LiveDaemon, tmp_path, drop_number, message
):
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
    baseline = None
    session = None
    try:
        assert await client.wait_ready(timeout=10.0)
        await settle_at(client, [0, -90, 180, 0, -20, 180])
        bundle = await client.config_bundle()
        assert bundle is not None
        # Exercise the actual stop/fresh/retune lifecycle without unrelated
        # model identification, geometry or capture setup from __aenter__.
        session = CalibrationSession(
            client, tmp_path / "calibration", tmp_path / "unused.bin"
        )
        session.directory.mkdir()
        session.core = await client._ensure_core()
        session.robot = tomllib.loads(bundle["robot_toml"])
        joint = session.robot["joints"][0]
        baseline = {
            **joint["gains"],
            **{k: joint[k] for k in ("ilim_ma", "velocity_limit_ticks_s")},
            "voltage_limit_mv": joint.get("voltage_limit_mv", 0),
        }
        await session.active_support()

        with pytest.raises(RuntimeError, match=message):
            async with temporary_feedback(session, 0, integral_scale=0.6):
                # Candidate admission has already received its real ACK and
                # drained its config frames. Exit sends Stop, then gains; no
                # queries or motion commands run concurrently with this test.
                relay.drop_reply(drop_number)

        assert len(relay.dropped) == 1, "the daemon must have emitted the lost reply"
        record = json.loads(
            (session.directory / "feedback-J1-restore.json").read_text()
        )
        assert record["restored"] is False
        assert message in record["restore_error"]
    finally:
        relay.drop_number = None
        try:
            if baseline is not None:
                # A lost ACK leaves application uncertain. Reapply the startup
                # tuple with reliable replies, even when the test assertion fails.
                await session.stop()
                assert await client.set_pid_gains(joint["node_id"], **baseline) == 1
                for _ in range(25):
                    await session.fresh()
                await session.stop()
        finally:
            await client.close()
            transport.close()
            await relay.closed
