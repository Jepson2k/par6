"""Sync facade smoke: the background-loop RobotClient over a scripted peer.

The peer runs on its own event loop in a helper thread; the facade runs its
coroutines on the par6 module loop — a real cross-thread round trip.
"""

from __future__ import annotations

import asyncio
import functools
import threading

import pytest

from par6.client import RobotClient
from par6.protocol import wire
from par6.protocol.constants import CmdType, MsgType
from protocol_peer import ANGLES, ScriptedRuntime, start_peer


@pytest.fixture
def threaded_peer():
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=loop.run_forever, daemon=True)
    thread.start()

    async def _start() -> tuple[ScriptedRuntime, asyncio.DatagramTransport]:
        return await start_peer()

    peer, transport = asyncio.run_coroutine_threadsafe(_start(), loop).result(2.0)
    yield peer, loop
    loop.call_soon_threadsafe(transport.close)
    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=2.0)


def test_sync_facade_smoke(threaded_peer):
    peer, peer_loop = threaded_peer

    def ack_and_complete(cmd, req_id, params):
        """Ack the enqueue, then immediately push its COMPLETE."""
        key = params[0]
        if key not in peer.dedup:
            peer.dedup[key] = peer.next_index
            peer.next_index += 1
        index = peer.dedup[key]
        return [
            wire.encode_wire([int(MsgType.OK), req_id, index]),
            wire.encode_wire([int(MsgType.COMPLETE), 0, index, True]),
        ]

    peer.handlers[CmdType.MOVE_J] = ack_and_complete

    host, port = peer.address
    with RobotClient(
        host=host,
        port=port,
        timeout=1.0,
        retries=1,
        status_transport="UNICAST",
        status_port=0,
    ) as client:
        ping = client.ping()
        assert ping is not None and ping.hardware_connected is True
        assert client.wait_ready(timeout=2.0) is True
        assert client.angles() == ANGLES

        # Blocking move: enqueue ack + COMPLETE push resolve wait=True.
        index = client.move_j([0, -90, 0, 0, 0, 0], speed=0.5, wait=True, timeout=2.0)
        assert index == 0

        assert client.jog_j(1, 0.4, 0.2) == 1
        assert client.stop() == 1

        # Status stream reaches the facade: wait on a checkpoint frame.
        status_addr = client._inner.status_address
        assert status_addr is not None
        peer_loop.call_soon_threadsafe(
            functools.partial(peer.send_status, status_addr, last_checkpoint="done", seq=3)
        )
        assert client.wait_checkpoint("done", timeout=2.0) is True
        assert client.status_seq_gaps == 0

    # Context-manager exit closed the client; further use must fail loudly.
    with pytest.raises(RuntimeError):
        client.ping()


def test_sync_facade_refuses_use_inside_a_running_loop(threaded_peer):
    peer, _ = threaded_peer
    host, port = peer.address
    client = RobotClient(host=host, port=port, timeout=0.5, retries=0,
                         status_transport="UNICAST", status_port=0)
    try:

        async def misuse():
            client.ping()

        with pytest.raises(RuntimeError, match="event loop is running"):
            asyncio.run(misuse())
    finally:
        client.close()
