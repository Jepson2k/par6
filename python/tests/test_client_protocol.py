"""AsyncRobotClient ↔ scripted protocol peer, over a real asyncio UDP path.

Covers the v2 client behaviors: req_id correlation under interleaving,
idempotency-keyed queued retries, COMPLETE-push and status-fallback waits
(including the stale-error ordering rule), shared-vs-copy status streaming,
fire-and-forget jog semantics, and bulk-command chunking.
"""

from __future__ import annotations

import asyncio
import math

import numpy as np
import pytest
from pinokin import se3_from_rpy
from protocol_peer import (
    ANGLES,
    IDENTITY_POSE,
    IO,
    SPEEDS,
    ScriptedRuntime,
    error_tuple,
    start_peer,
)
from waldoctl.shapes import Box
from waldoctl.status import ActionState as WActionState
from waldoctl.tools import GripperTool, GripperType
from waldoctl.tools import ToolState as WToolState

from par6.client import AsyncRobotClient, RobotError
from par6.protocol import wire
from par6.protocol.constants import NUM_JOINTS, CmdType, Frame, MsgType, QueryType


@pytest.fixture
async def peer():
    protocol, transport = await start_peer()
    yield protocol
    transport.close()


@pytest.fixture
async def client(peer: ScriptedRuntime):
    host, port = peer.address
    c = AsyncRobotClient(
        host=host,
        port=port,
        timeout=0.4,
        retries=1,
        status_transport="UNICAST",
        status_port=0,
    )
    yield c
    await c.close()


async def _status_addr(client: AsyncRobotClient) -> tuple[str, int]:
    """Bring the endpoints up and return the client's status listener addr."""
    assert await client.ping() is not None
    addr = client.status_address
    assert addr is not None
    return addr


# ---------------------------------------------------------------------------
# Correlation
# ---------------------------------------------------------------------------


async def test_reply_correlation_survives_interleaved_out_of_order_replies(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    """Two concurrent queries whose replies arrive in reverse order must each
    resolve to their own result (matched by req_id, not arrival order)."""
    held: dict[str, int] = {}

    def hold_angles(cmd, req_id, params):
        held["angles_req"] = req_id
        return None  # reply later, out of order

    def answer_both(cmd, req_id, params):
        io_reply = wire.encode_wire(
            [int(MsgType.RESPONSE), req_id, [int(QueryType.IO), IO]]
        )
        angles_reply = wire.encode_wire(
            [int(MsgType.RESPONSE), held["angles_req"], [int(QueryType.ANGLES), ANGLES]]
        )
        return [io_reply, angles_reply]  # newest request answered FIRST

    peer.handlers[CmdType.ANGLES] = hold_angles
    peer.handlers[CmdType.IO] = answer_both

    angles_task = asyncio.create_task(client.angles())
    await peer.wait_until(lambda: len(peer.of(CmdType.ANGLES)) == 1)
    io_result = await client.io()
    angles_result = await angles_task

    assert io_result == IO
    assert angles_result == ANGLES
    assert peer.of(CmdType.ANGLES)[0][1] != peer.of(CmdType.IO)[0][1]


async def test_query_retries_reuse_req_id_and_give_up_to_none(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    peer.drop_replies[CmdType.ANGLES] = 1
    assert await client.angles() == ANGLES
    attempts = peer.of(CmdType.ANGLES)
    assert len(attempts) == 2
    assert attempts[0][1] == attempts[1][1]  # same req_id — a late first reply still counts

    peer.drop_replies[CmdType.IO] = 2  # every attempt swallowed
    assert await client.io() is None
    assert len(peer.of(CmdType.IO)) == 2  # 1 + retries, no more


# ---------------------------------------------------------------------------
# Queued commands: idempotency keys under retry
# ---------------------------------------------------------------------------


async def test_queued_retry_keeps_idempotency_key_and_original_index(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    """A lost ack is retried with the SAME key; the peer's dedup window
    re-acks the ORIGINAL index instead of double-queueing."""
    peer.next_index = 7  # arbitrary allocator position
    peer.drop_replies[CmdType.MOVE_J] = 1  # process + allocate, but lose the ack

    index = await client.move_j([0, -90, 0, 0, 0, 0], speed=0.5)

    sends = peer.of(CmdType.MOVE_J)
    assert len(sends) == 2
    assert sends[0][2][0] == sends[1][2][0]  # identical idempotency key
    assert sends[0][1] == sends[1][1]  # identical req_id
    assert index == 7  # the index allocated on the FIRST (ack-lost) attempt
    assert peer.next_index == 8  # dedup prevented a second enqueue


async def test_queued_rejection_raises_and_timeout_returns_minus_one(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    def reject(cmd, req_id, params):
        return wire.encode_wire([int(MsgType.ERROR), req_id, error_tuple(-1, code=35)])

    peer.handlers[CmdType.HOME] = reject
    with pytest.raises(RobotError) as excinfo:
        await client.home()
    assert excinfo.value.code == 35

    peer.drop_replies[CmdType.DELAY] = 2
    assert await client.delay(1.0) == -1


# ---------------------------------------------------------------------------
# wait_command: COMPLETE push and status fallback
# ---------------------------------------------------------------------------


async def test_wait_command_resolves_on_complete_push(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    index = await client.move_j([0.0] * 6, speed=0.5)
    assert index >= 0
    waiter = asyncio.create_task(client.wait_command(index, timeout=2.0))
    await asyncio.sleep(0)  # let the waiter register
    peer.complete(index, ok=True)
    assert await waiter is True

    # Push arriving BEFORE the wait is also honored (recorded completion).
    index2 = await client.delay(0.5)
    peer.complete(index2, ok=True)
    assert await client.wait_command(index2, timeout=2.0) is True


async def test_wait_command_raises_on_failed_complete_push(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    index = await client.move_l([250.0, 0.0, 200.0, 90.0, 0.0, 90.0], speed=0.5)
    waiter = asyncio.create_task(client.wait_command(index, timeout=2.0))
    await asyncio.sleep(0)
    peer.complete(index, ok=False, detail=error_tuple(index, code=36))
    with pytest.raises(RobotError) as excinfo:
        await waiter
    assert excinfo.value.code == 36
    assert excinfo.value.command_index == index


async def test_wait_command_status_fallback_high_water(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    status_addr = await _status_addr(client)
    index = await client.checkpoint("mid")
    waiter = asyncio.create_task(client.wait_command(index, timeout=3.0))

    # Not yet completed: high-water below the awaited index.
    peer.send_status(
        status_addr, seq=1, completed_index=index - 1, accepted_index=index
    )
    assert await client.wait_status(lambda s: s.seq == 1, timeout=2.0)
    assert not waiter.done()

    # Blended-away commands report the max of consumed indexes — a HIGHER
    # completed_index must also satisfy the wait.
    peer.send_status(
        status_addr, seq=2, completed_index=index + 3, accepted_index=index + 3
    )
    assert await waiter is True


async def test_wait_command_stale_error_ordering_rule(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    """A standing error fails the wait ONLY when error.command_index <= N and
    accepted_index >= N — an older frame cannot replay a stale rejection."""
    status_addr = await _status_addr(client)
    index = await client.checkpoint("gate")
    waiter = asyncio.create_task(client.wait_command(index, timeout=3.0))

    # Stale: error attributed at/below N but the frame predates N's acceptance.
    peer.send_status(
        status_addr,
        seq=1,
        error=error_tuple(index - 1, code=10),
        accepted_index=index - 1,
    )
    assert await client.wait_status(lambda s: s.seq == 1, timeout=2.0)
    assert not waiter.done()

    # Irrelevant: error attributed to a LATER command never fails this wait.
    peer.send_status(
        status_addr,
        seq=2,
        error=error_tuple(index + 5, code=10),
        accepted_index=index + 5,
    )
    assert await client.wait_status(lambda s: s.seq == 2, timeout=2.0)
    assert not waiter.done()

    # Completion still lands.
    peer.send_status(status_addr, seq=3, completed_index=index, accepted_index=index)
    assert await waiter is True

    # Blocking: error at/below N on a frame that postdates N's acceptance.
    index2 = await client.checkpoint("gate2")
    waiter2 = asyncio.create_task(client.wait_command(index2, timeout=3.0))
    await asyncio.sleep(0)
    peer.send_status(
        status_addr,
        seq=4,
        error=error_tuple(index2, code=51),
        accepted_index=index2,
        completed_index=index2 - 1,
    )
    with pytest.raises(RobotError) as excinfo:
        await waiter2
    assert excinfo.value.code == 51


# ---------------------------------------------------------------------------
# Status streaming
# ---------------------------------------------------------------------------


async def test_stream_status_shared_reuses_one_buffer_and_surfaces_v2_header(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    status_addr = await _status_addr(client)
    stream = client.stream_status_shared()
    try:
        peer.send_status(status_addr, seq=1, angles=[1.0] * 6)
        async with asyncio.timeout(2.0):
            first = await anext(stream)
        assert first.seq == 1
        assert first.angles.tolist() == [1.0] * 6

        peer.send_status(
            status_addr, seq=5, angles=[2.0] * 6, link_ok=0, data_age_ms=120
        )
        async with asyncio.timeout(2.0):
            second = await anext(stream)
        assert second is first  # ONE shared buffer, overwritten in place
        assert second.angles.tolist() == [2.0] * 6
        # v2 header surfaced: link state, data age, and the seq gap count.
        assert second.link_ok == 0
        assert second.data_age_ms == 120
        assert client.status_seq_gaps == 3  # seq 2, 3, 4 never arrived
    finally:
        await stream.aclose()


async def test_stream_status_yields_independent_copies(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    status_addr = await _status_addr(client)
    stream = client.stream_status()
    try:
        peer.send_status(status_addr, seq=1, angles=[10.0] * 6)
        async with asyncio.timeout(2.0):
            first = await anext(stream)
        peer.send_status(status_addr, seq=2, angles=[20.0] * 6)
        async with asyncio.timeout(2.0):
            second = await anext(stream)
        assert first is not second
        assert first.angles.tolist() == [10.0] * 6  # unclobbered by the later frame
        assert second.angles.tolist() == [20.0] * 6
        # The copy keeps the cart_en alias structure pointing at ITS OWN arrays.
        assert first.cart_en["WRF"] is first.cart_en_wrf
        assert first.cart_en_wrf is not client._shared_status.cart_en_wrf
    finally:
        await stream.aclose()


async def test_wait_status_and_wait_checkpoint(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    status_addr = await _status_addr(client)
    waiter = asyncio.create_task(client.wait_checkpoint("pick_done", timeout=2.0))
    peer.send_status(status_addr, seq=1, last_checkpoint="other")
    peer.send_status(status_addr, seq=2, last_checkpoint="pick_done")
    assert await waiter is True
    assert await client.wait_status(lambda s: s.seq == 99, timeout=0.1) is False


async def test_wait_motion_start_then_settle(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    status_addr = await _status_addr(client)

    async def feed():
        peer.send_status(status_addr, seq=1, speeds=[0.5] * 6)  # motion starts
        for seq in range(2, 40):
            peer.send_status(status_addr, seq=seq, speeds=[0.0] * 6)
            await asyncio.sleep(0.02)

    feeder = asyncio.create_task(feed())
    try:
        assert (
            await client.wait_motion(
                timeout=3.0, settle_window=0.1, motion_start_timeout=1.0
            )
            is True
        )
    finally:
        feeder.cancel()


# ---------------------------------------------------------------------------
# Fire-and-forget: jog / servo semantics
# ---------------------------------------------------------------------------


async def test_jog_semantics_and_validation(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    # Single-joint jog: signed fraction lands in the right slot, with the
    # self-terminating watchdog duration on the wire.
    assert await client.jog_j(2, -0.5, 0.75) == 1
    await peer.wait_until(lambda: len(peer.of(CmdType.JOG_J)) == 1)
    _, _, params = peer.of(CmdType.JOG_J)[0]
    assert params == [[0.0, 0.0, -0.5, 0.0, 0.0, 0.0], 0.75, 1.0]

    # Multi-joint jog + default duration (0.1 s watchdog).
    await client.jog_j(joints=[0, 5], speeds=[0.25, -1.0])
    await peer.wait_until(lambda: len(peer.of(CmdType.JOG_J)) == 2)
    _, _, params = peer.of(CmdType.JOG_J)[1]
    assert params == [[0.25, 0.0, 0.0, 0.0, 0.0, -1.0], 0.1, 1.0]

    # Cartesian jog: velocities, duration, frame, accel.
    await client.jog_l("TRF", "RZ", 0.3, 0.5)
    await peer.wait_until(lambda: len(peer.of(CmdType.JOG_L)) == 1)
    _, _, params = peer.of(CmdType.JOG_L)[0]
    assert params == [[0.0, 0.0, 0.0, 0.0, 0.0, 0.3], 0.5, int(Frame.TRF), 1.0]

    # Requirement-derived rejects: the watchdog must be > 0, speeds must be
    # finite signed fractions, and a target is mandatory.
    with pytest.raises(wire.ProtocolError):
        await client.jog_j(0, 0.5, duration=0.0)
    with pytest.raises(wire.ProtocolError):
        await client.jog_j(0, 0.5, duration=-1.0)
    with pytest.raises(wire.ProtocolError):
        await client.jog_j(0, float("nan"))
    with pytest.raises(wire.ProtocolError):
        await client.jog_j(0, 1.5)
    with pytest.raises(ValueError):
        await client.jog_j()
    with pytest.raises(ValueError):
        await client.jog_l("WRF")
    with pytest.raises(ValueError):
        await client.jog_l("BAD", "X", 0.5)
    with pytest.raises(wire.ProtocolError):
        await client.jog_j(0, float("inf"))
    # An out-of-range joint must be refused, not indexed: a negative one
    # wraps onto another joint in Python and would move the wrong axis with
    # nothing raised, and a length mismatch silently drops speeds.
    with pytest.raises(ValueError):
        await client.jog_j(joints=[-1], speeds=[0.5])
    with pytest.raises(ValueError):
        await client.jog_j(joints=[NUM_JOINTS], speeds=[0.5])
    with pytest.raises(ValueError):
        await client.jog_j(joints=[0, 1], speeds=[0.5])
    # Nothing invalid ever reached the wire.
    assert len(peer.of(CmdType.JOG_J)) == 2 and len(peer.of(CmdType.JOG_L)) == 1


async def test_move_j_refuses_a_relative_pose_instead_of_moving_absolute(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    """MOVE_J_POSE carries no ``rel`` field on the wire.

    Accepting ``rel=True`` and dropping it turns a small requested nudge
    into an absolute move to those base-frame coordinates — near the origin
    that is most of the arm's reach, with no error raised.
    """
    pose = [100.0, 0.0, 300.0, 0.0, 0.0, 0.0]
    with pytest.raises(ValueError, match="absolute"):
        await client.move_j(pose=pose, rel=True, duration=1.0)
    assert not peer.of(CmdType.MOVE_J_POSE), "nothing may reach the wire"

    # The absolute pose form still works, and the relative JOINT form is
    # untouched — MOVE_J does carry a rel flag.
    await client.move_j(pose=pose, duration=1.0)
    await peer.wait_until(lambda: len(peer.of(CmdType.MOVE_J_POSE)) == 1)
    await client.move_j([1.0] * NUM_JOINTS, rel=True, duration=1.0)
    await peer.wait_until(lambda: len(peer.of(CmdType.MOVE_J)) == 1)
    assert peer.of(CmdType.MOVE_J)[0][2][-1] is True


async def test_servo_stream_reuses_tx_buffer_without_corruption(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    """Back-to-back fire-and-forget sends through the reused encode buffer
    must arrive as distinct, correct datagrams."""
    targets = [[float(i)] * 6 for i in range(5)]
    for t in targets:
        assert await client.servo_j(t) == 1
    await client.servo_l([250.0, 0.0, 300.0, 90.0, 0.0, 90.0], speed=0.5)
    await peer.wait_until(
        lambda: len(peer.of(CmdType.SERVO_J)) == 5 and len(peer.of(CmdType.SERVO_L)) == 1
    )
    assert [p[0] for _, _, p in peer.of(CmdType.SERVO_J)] == targets
    assert peer.of(CmdType.SERVO_L)[0][2] == [
        [250.0, 0.0, 300.0, 90.0, 0.0, 90.0], 0.5, 1.0
    ]  # fmt: skip


async def test_teleport_is_fire_and_forget(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    assert await client.teleport([0, -90, 0, 0, 0, 0], tool_positions=[1.0]) == 1
    await peer.wait_until(lambda: len(peer.of(CmdType.TELEPORT)) == 1)
    assert peer.of(CmdType.TELEPORT)[0][2] == [
        [0.0, -90.0, 0.0, 0.0, 0.0, 0.0], [1.0]
    ]  # fmt: skip


# ---------------------------------------------------------------------------
# Chunking
# ---------------------------------------------------------------------------


async def test_bulk_move_auto_chunks_and_reassembles(peer: ScriptedRuntime):
    host, port = peer.address
    client = AsyncRobotClient(
        host=host,
        port=port,
        timeout=0.5,
        retries=1,
        status_transport="UNICAST",
        status_port=0,
        mtu=300,
    )
    try:
        waypoints = [[float(i), 0.0, 0.0, 0.0, 0.0, float(-i)] for i in range(40)]
        index = await client.move_s(waypoints, speed=0.5)
        assert index == 0
        assert peer.chunks_seen >= 2  # went over the wire as chunk envelopes
        moves = peer.of(CmdType.MOVE_S)
        assert len(moves) == 1
        assert moves[0][2][1] == waypoints  # reassembled byte-identically
    finally:
        await client.close()


# ---------------------------------------------------------------------------
# SYSTEM commands and queries (wire mapping + result parsing)
# ---------------------------------------------------------------------------


async def test_system_commands_wire_mapping(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    assert await client.stop() == 1
    assert peer.of(CmdType.STOP)[0][2] == [True]  # explicit cancel scope
    assert await client.stop(clear_queue=False) == 1
    assert peer.of(CmdType.STOP)[1][2] == [False]

    assert await client.estop() == 1
    assert await client.reset() == 1
    assert await client.reset_state() == 1
    assert await client.simulator(True) == 1
    assert peer.of(CmdType.SIMULATOR)[0][2] == [True]

    assert await client.select_profile("toppra") == 1
    assert peer.of(CmdType.SELECT_PROFILE)[0][2] == ["TOPPRA"]

    assert await client.set_tcp_offset(0, 0, -190) == 1
    assert peer.of(CmdType.SET_TCP_OFFSET)[0][2] == [0.0, 0.0, -190.0]

    # write_io maps logical output 0/1 onto controller ports 2/3. The 1
    # here is the SCRIPTED peer acking, not a runtime succeeding — par6d
    # refuses every write_io (issue #28), which the e2e suite pins. What
    # this asserts is the client's encoding, which is real either way.
    assert await client.write_io(0, 1) == 1
    assert await client.write_io(1, 0) == 1
    assert [r[2] for r in peer.of(CmdType.WRITE_IO)] == [[2, 1], [3, 0]]
    with pytest.raises(ValueError):
        await client.write_io(2, 1)
    with pytest.raises(ValueError):
        await client.write_io(0, 5)

    assert await client.set_shapes(
        [Box(name="table", x=0.6, y=0.4, z=0.02, pose=(0.3, 0, -0.01, 0, 0, 0))]
    ) == 1
    (shapes_param,) = peer.of(CmdType.SET_SHAPES)[0][2]
    assert shapes_param[0][0] == "box" and shapes_param[0][5] == "table"

    # One send + wait: an unacked SYSTEM command is NOT retried and reports 0.
    peer.drop_replies[CmdType.RESET] = 1
    assert await client.reset() == 0
    assert len(peer.of(CmdType.RESET)) == 2  # the earlier success + this one

    # Active rejection raises.
    def reject(cmd, req_id, params):
        return wire.encode_wire([int(MsgType.ERROR), req_id, error_tuple(-1, code=52)])

    peer.handlers[CmdType.SELECT_PROFILE] = reject
    with pytest.raises(RobotError) as excinfo:
        await client.select_profile("bogus")
    assert excinfo.value.code == 52


async def test_query_surface_parses_wire_payloads(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    ping = await client.ping()
    assert ping is not None and ping.hardware_connected is True

    assert await client.angles() == ANGLES
    assert await client.io() == IO
    assert await client.joint_speeds() == [0.0, 0.0, 0.1, 0.0, -0.1, 0.0]
    assert await client.profile() == "TOPPRA"
    assert await client.tcp_speed() == 12.5
    assert await client.tcp_offset() == [0.0, 0.0, 35.5]
    assert await client.is_simulator() is True
    assert await client.queue() == ["MOVE_J", "DELAY"]

    queue_state = await client.queue_state()
    assert queue_state is not None
    assert (queue_state.executing_index, queue_state.completed_index) == (4, 3)
    assert queue_state.last_checkpoint == "pick"

    tools = await client.tools()
    assert tools is not None
    assert (tools.tool, tools.available) == ("SSG48", ["SSG48", "MSG"])

    activity = await client.activity()
    assert activity is not None
    assert activity.state is WActionState.EXECUTING
    assert activity.command == "MOVE_L"

    reachable = await client.reachable()
    assert reachable is not None
    assert reachable.cart_en_trf[1] == 0

    assert await client.error() is None
    peer.handlers[CmdType.ERROR] = lambda cmd, req_id, params: wire.encode_wire(
        [int(MsgType.RESPONSE), req_id, [int(QueryType.ERROR), error_tuple(3, code=51)]]
    )
    standing = await client.error()
    assert standing is not None and (standing.code, standing.command_index) == (51, 3)

    status = await client.status()
    assert status is not None
    assert status.angles == ANGLES
    assert status.tool_status is not None
    assert status.tool_status.state is WToolState.ACTIVE
    assert status.tool_status.variant_key == "fin_ray"  # v2: variant on the wire

    stats = await client.loop_stats()
    assert stats is not None and stats.target_hz == 250.0

    world = await client.shapes()
    assert world is not None
    assert world.installation[0].name == "table"
    assert world.program[0].kind == "sphere"


async def test_pose_converts_matrix_to_xyz_rpy_degrees(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    """The STATUS/POSE matrix decodes in the wire's intrinsic-XYZ convention.

    The oracle is ``pinokin.se3_from_rpy`` — the composition ``Robot.ik``
    re-encodes a target with and the frontend's readout decodes with, so a
    pose read here is a pose that can be handed straight back to ``move_l``.
    Read in the URDF fixed-axis order instead, the same nine matrix entries
    name a different orientation for every pose with more than one non-zero
    rotation component, and the everyday tool-down pose comes back with its
    wrist angle negated.
    """
    mat = np.zeros((4, 4))
    se3_from_rpy(120.0, -45.5, 310.0, *np.radians([30.0, -40.0, 55.0]), mat)

    peer.handlers[CmdType.POSE] = lambda cmd, req_id, params: wire.encode_wire(
        [int(MsgType.RESPONSE), req_id, [int(QueryType.POSE), mat.flatten().tolist()]]
    )
    pose = await client.pose(frame="TRF")
    assert peer.of(CmdType.POSE)[0][2] == [int(Frame.TRF)]
    assert pose is not None
    assert pose[:3] == pytest.approx([120.0, -45.5, 310.0])
    assert pose[3:] == pytest.approx([30.0, -40.0, 55.0])

    for rz_deg in (10.0, 90.0):
        down = np.zeros((4, 4))
        se3_from_rpy(0.0, 0.0, 0.0, math.pi, 0.0, math.radians(rz_deg), down)
        peer.handlers[CmdType.POSE] = lambda cmd, req_id, params, m=down: wire.encode_wire(
            [int(MsgType.RESPONSE), req_id, [int(QueryType.POSE), m.flatten().tolist()]]
        )
        tool_down = await client.pose()
        assert tool_down is not None
        assert abs(tool_down[3]) == pytest.approx(180.0)
        assert tool_down[4] == pytest.approx(0.0, abs=1e-9)
        assert tool_down[5] == pytest.approx(rz_deg), (
            f"tool-down wrist angle came back as {tool_down[5]}, not {rz_deg}"
        )


async def test_motion_timing_maps_to_exactly_one_of_duration_speed(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    await client.move_j([0.0] * 6, duration=2.0)
    await client.move_j([0.0] * 6, speed=0.5)
    await client.move_j([0.0] * 6)  # neither → default full profile speed
    sent = [r[2] for r in peer.of(CmdType.MOVE_J)]
    assert [(p[2], p[3]) for p in sent] == [(2.0, None), (None, 0.5), (None, 1.0)]
    # Blend radius: 0 means "no blend" → nil on the wire.
    assert sent[0][5] is None
    with pytest.raises(ValueError):
        await client.move_j([0.0] * 6, duration=1.0, speed=0.5)
    with pytest.raises(ValueError):
        await client.move_j([0.0] * 3)
    with pytest.raises(ValueError):
        await client.move_l([0.0] * 6, frame="XYZ")


# ---------------------------------------------------------------------------
# Tool binding (injectable spec source — robot.py wires the real registry)
# ---------------------------------------------------------------------------


class _StubGripper(GripperTool):
    def __init__(self) -> None:
        super().__init__(
            key="GRIP",
            display_name="Test gripper",
            tcp_origin=(0.0, 0.0, 0.1),
            tcp_rpy=(0.0, 0.0, 0.0),
        )

    @property
    def gripper_type(self) -> GripperType:
        return GripperType.ELECTRIC

    async def set_position(self, position: float, **kwargs) -> int:
        return await self._execute(self.key, "move", [float(position)])

    async def open(self, **kwargs) -> int:
        return await self._execute(self.key, "open", [])

    async def close(self, **kwargs) -> int:
        return await self._execute(self.key, "close", [])

    async def status(self):
        return await self._get_status()


async def test_bound_tool_actions_go_through_tool_action(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    with pytest.raises(RuntimeError):
        _ = client.tool  # no tool selected yet

    client.bind_tools([_StubGripper()])
    index = await client.select_tool("grip")
    assert index >= 0
    assert peer.of(CmdType.SELECT_TOOL)[0][2][1:] == ["GRIP", None]

    assert await client.tool.set_position(0.25) >= 0
    assert await client.tool.open() >= 0
    actions = [(r[2][1], r[2][2], r[2][3]) for r in peer.of(CmdType.TOOL_ACTION)]
    assert actions == [("GRIP", "move", [0.25]), ("GRIP", "open", [])]

    tool_status = await client.tool.status()
    assert tool_status is not None
    assert tool_status.key == "SSG48" and tool_status.variant_key == "fin_ray"

    # A refused selection (the runtime is fitted with a different tool)
    # must not become the client's active tool — the next tool_action
    # would otherwise be addressed to hardware that is not on the arm.
    def reject(cmd, req_id, params):
        return wire.encode_wire([int(MsgType.ERROR), req_id, error_tuple(-1, code=43)])

    peer.handlers[CmdType.SELECT_TOOL] = reject
    with pytest.raises(RobotError) as excinfo:
        await client.select_tool("not_fitted")
    assert excinfo.value.code == 43
    assert await client.tool.set_position(0.5) >= 0
    assert peer.of(CmdType.TOOL_ACTION)[-1][2][1] == "GRIP"


# ---------------------------------------------------------------------------
# Multicast fallback ladder (bad group → unicast) and wait_ready
# ---------------------------------------------------------------------------


async def test_status_subscription_falls_back_to_unicast_on_multicast_failure(
    peer: ScriptedRuntime,
):
    host, port = peer.address
    client = AsyncRobotClient(
        host=host,
        port=port,
        timeout=0.4,
        retries=0,
        status_transport="MULTICAST",
        mcast_group="not-a-group",  # join must fail → ladder ends at unicast
        status_port=0,
    )
    try:
        status_addr = await _status_addr(client)
        peer.send_status(status_addr, seq=9)
        assert await client.wait_status(lambda s: s.seq == 9, timeout=2.0)
    finally:
        await client.close()


async def test_wait_ready_polls_until_reachable(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    peer.drop_replies[CmdType.PING] = 2  # first ping attempt (incl. retry) fails
    assert await client.wait_ready(timeout=5.0, interval=0.05) is True
    assert len(peer.of(CmdType.PING)) >= 3


# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------


async def test_config_cased_tool_keys_round_trip_through_a_bare_client(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    """The runtime spells tool keys the way the gripper config does; every
    consumer indexes them the way ``waldoctl.ToolSpec`` does (upper).  A key
    off the wire — from TOOLS, from a STATUS frame, from TOOL_STATUS — must
    therefore index ``Robot.tools`` and the client's own bound tools, and a
    bare client (a user script's, built with no tool specs) must be able to
    act on the selected tool."""
    from par6.robot import Robot

    config_key = "MSG_small_motor_150mm_rail"
    peer.handlers[CmdType.TOOLS] = lambda cmd, req_id, params: wire.encode_wire(
        [int(MsgType.RESPONSE), req_id, [int(QueryType.TOOLS), config_key, [config_key, "Flange"]]]
    )
    wire_tool_status = [config_key, 2, True, False, 0, [0.5], [120.0], ""]
    peer.handlers[CmdType.TOOL_STATUS] = lambda cmd, req_id, params: wire.encode_wire(
        [int(MsgType.RESPONSE), req_id, [int(QueryType.TOOL_STATUS), wire_tool_status]]
    )
    peer.handlers[CmdType.STATUS] = lambda cmd, req_id, params: wire.encode_wire(
        [
            int(MsgType.RESPONSE),
            req_id,
            [int(QueryType.STATUS), IDENTITY_POSE, ANGLES, SPEEDS, IO, wire_tool_status],
        ]
    )

    robot = Robot()
    reported = await client.tools()
    assert reported is not None
    assert robot.tools[reported.tool].display_name == config_key
    assert all(key in robot.tools for key in reported.available)

    status_addr = await _status_addr(client)
    peer.send_status(status_addr, seq=1, tool_status=wire_tool_status)
    assert await client.wait_status(lambda s: s.tool_status_present, timeout=2.0)
    broadcast_key = client._shared_status.tool_status.key
    assert robot.tools[broadcast_key] is robot.tools[reported.tool]

    queried = await client.status()
    assert queried is not None and queried.tool_status is not None
    assert robot.tools[queried.tool_status.key] is robot.tools[reported.tool]

    # A bare client binds the packaged tools, so a user script can drive the
    # selected tool without going through the Robot factory.
    assert await client.select_tool(reported.tool) >= 0
    assert client.tool.key == broadcast_key
    await client.tool.close()
    actions = peer.of(CmdType.TOOL_ACTION)
    assert [(p[1], p[2]) for _, _, p in actions] == [(broadcast_key, "move")]


async def test_the_client_enforces_the_same_bounds_as_the_runtime(
    client: AsyncRobotClient, peer: ScriptedRuntime
):
    """wire.py claims to match the Rust codec rule-for-rule.

    It did not: durations had no ceiling and collections no cap, so the
    client happily built commands the daemon is guaranteed to refuse. The
    jog case is the one that matters — a jog's ``duration`` IS the watchdog
    that stops it, and an unbounded one arms a watchdog no operator
    outlives.
    """
    with pytest.raises(wire.ProtocolError, match="3600"):
        await client.delay(1e18)
    with pytest.raises(wire.ProtocolError, match="60"):
        await client.jog_j(0, 0.5, duration=1e9)
    # Values a real program sends still pass.
    await client.delay(2.0)
    await client.jog_j(0, 0.5, duration=0.5)
