"""Sync facade smoke: the background-loop RobotClient over a scripted peer.

The peer runs on its own event loop in a helper thread; the facade runs its
coroutines on the par6 module loop — a real cross-thread round trip.
"""

from __future__ import annotations

import asyncio
import functools
import socket
import threading

import pytest
from protocol_peer import ANGLES, ScriptedRuntime, start_peer

from par6.client import RobotClient
from par6.protocol import wire
from par6.protocol.constants import CmdType, ControllerMode, MsgType


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

        # The two control commands a synchronous script needs to make the
        # arm safe to touch: limp it, and float it under G(q) alone. Both
        # reached the async client only, so a sync-only consumer could do
        # neither. Assert they land on the wire, with the flag carried.
        assert client.safety_stop() == 1
        assert peer.of(CmdType.SAFETY_STOP), "safety_stop never reached the wire"
        assert client.set_gravity_comp(True) == 1
        assert peer.of(CmdType.SET_GRAVITY_COMP)[-1][2] == [True]

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


def test_cli_reads_a_live_runtime_and_reports_an_unreachable_one(threaded_peer, capsys):
    """The console script is the shell view of the sync client.

    par6 shipped no [project.scripts] at all, so the only way to read an
    arm's state was to write Python. Drive the real entry point against the
    scripted peer, then against an address nothing answers — a CLI that
    exits 0 on an unreachable runtime is worse than no CLI.
    """
    from par6.cli import EXIT_UNREACHABLE, main

    peer, _ = threaded_peer
    host, port = peer.address

    assert main(["--host", host, "--port", str(port), "angles"]) == 0
    reported = [float(v) for v in capsys.readouterr().out.split()]
    assert reported == pytest.approx(ANGLES)

    # Nothing is listening on a port we know is free.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        dead = int(probe.getsockname()[1])
    assert main(["--host", "127.0.0.1", "--port", str(dead), "--timeout", "0.2", "angles"]) == (
        EXIT_UNREACHABLE
    )
    assert "did not answer" in capsys.readouterr().err


def test_freedrive_reads_the_broadcast_not_the_last_command(threaded_peer):
    """Freedrive is a state of the arm, not a flag the client remembers.

    par6 has no freedrive MODE: with the gravity feedforward applied, IDLE
    emits a torque-only G(q) hold with no position term, so the arm floats.
    That means the honest answer to "is it floating?" is the runtime's own
    gravity_applied() condition, read off STATUS — IDLE, homed, enabled,
    gravity on. Gravity comp is also applied in JOG/EXEC/STREAM, where a
    position term IS holding the arm, so trusting the last set_gravity_comp
    would report freedrive while the arm was rigidly tracking a trajectory.
    """
    peer, peer_loop = threaded_peer
    host, port = peer.address
    with RobotClient(
        host=host, port=port, timeout=1.0, retries=1,
        status_transport="UNICAST", status_port=0,
    ) as client:
        assert client.wait_ready(timeout=2.0) is True
        addr = client._inner.status_address
        assert addr is not None

        def publish_and_settle(seq: int, **kw):
            """Publish a frame and block until THAT frame has been decoded.

            Without the seq wait this races: the client would answer from
            whichever frame arrived first, which made this test flaky.
            """
            peer_loop.call_soon_threadsafe(
                functools.partial(peer.send_status, addr, seq=seq, **kw)
            )
            assert client.wait_status(lambda s: s.seq == seq, timeout=2.0) is True

        # Floating: IDLE, homed, enabled, gravity applied.
        publish_and_settle(10, mode=int(ControllerMode.IDLE), homed=True,
                           enabled=True, gravity_comp=True)
        assert client.is_freedrive() is True

        # Same gravity switch, but EXEC is tracking a trajectory — a
        # position term is holding the arm, so it is not back-driveable.
        publish_and_settle(11, mode=int(ControllerMode.EXEC), homed=True,
                           enabled=True, gravity_comp=True)
        assert client.is_freedrive() is False

        # And IDLE without the feedforward is an active zero-velocity hold.
        publish_and_settle(12, mode=int(ControllerMode.IDLE), homed=True,
                           enabled=True, gravity_comp=False)
        assert client.is_freedrive() is False
