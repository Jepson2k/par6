"""End-to-end: the par6 Python client against a live ``par6d --sim`` runtime.

Nothing here is scripted or faked — a real daemon process runs the RT core
over the closed-loop simulator, the planner and the protocol-v2 command
plane, and every assertion is an outcome observed through the public client
API (queries, COMPLETE pushes, the STATUS broadcast) or through the
``waldoctl`` :class:`~par6.robot.Robot` backend.

Skipped, not failed, when no ``par6d`` binary is reachable — see
:mod:`live_daemon`.
"""

from __future__ import annotations

import asyncio
import math
import time

import numpy as np
import pytest
from live_daemon import LiveDaemon, free_udp_port, requires_par6d, sim_config

from par6 import config as _cfg
from par6.client import AsyncRobotClient, RobotError
from par6.protocol.constants import ActionState, ErrorCode
from par6.robot import Robot

pytestmark = [pytest.mark.e2e, requires_par6d]

#: Wall-clock ceiling for one session step (boot, settle, a short move).
STEP_BUDGET_S = 20.0
#: The shipped PAR6 homing sequence takes ~60 s (its pre-moves, backoffs and
#: release phases are config SECONDS, and the sim runs in wall-clock time).
HOMING_BUDGET_S = 200.0
#: Codes a client must tolerate while the RT clear sequence settles after
#: a reset: acceptance is not success, so a move can also be accepted and
#: then truthfully failed.
RETRIABLE = frozenset(
    {
        ErrorCode.SYS_CONTROLLER_DISABLED,
        ErrorCode.SYS_ESTOP_ACTIVE,
        ErrorCode.MOTN_SETUP_FAILED,
    }
)


@pytest.fixture
def daemon(tmp_path):
    """A fresh ``par6d --sim`` process on ephemeral ports."""
    live = LiveDaemon.start(tmp_path)
    yield live
    live.stop()


def park_deg() -> list[float]:
    """The config park pose in wire units — inside every soft window."""
    return [
        math.degrees(v) for v in _cfg.load_robot_config()["robot"]["park_pose_rad"]
    ]


def ready_pose_deg() -> list[float]:
    """Where the shipped homing sequence leaves the arm.

    Replays the sequence's ``move_to`` steps in order and keeps the last
    commanded position per joint — derived from the same config the daemon
    executes, never a transcribed constant.
    """
    final: dict[int, float] = {}
    for step in _cfg.load_robot_config()["homing"]["sequence"]:
        for move in step.get("move_to", []):
            final[int(move["joint"])] = float(move["position_rad"])
    if sorted(final) != list(range(6)):
        raise RuntimeError(f"homing sequence leaves joints {sorted(final)} unplaced")
    return [math.degrees(final[j]) for j in range(6)]


def max_deg_error(actual, expected) -> float:
    return float(np.max(np.abs(np.asarray(actual) - np.asarray(expected))))


async def enable(client: AsyncRobotClient, probe) -> RobotError | None:
    """``reset()``, then poll *probe* until the controller stops rejecting
    with SYS_CONTROLLER_DISABLED (the RT clear sequence settles over
    several ticks).  Returns the error that finally gated *probe*, or None
    when it was accepted."""
    assert await client.reset() == 1
    deadline = time.monotonic() + STEP_BUDGET_S
    while time.monotonic() < deadline:
        try:
            await probe()
        except RobotError as exc:
            if exc.code != ErrorCode.SYS_CONTROLLER_DISABLED:
                return exc
            await asyncio.sleep(0.1)
            continue
        return None
    raise AssertionError("controller never left DISABLED after reset()")


async def teleport_to(client: AsyncRobotClient, angles: list[float]) -> None:
    """Drive the sim to *angles* with the fire-and-forget teleport.

    Teleport is unacked and gated on ENABLED, so it is re-sent until the
    broadcast shows it landed — the same thing a UI would do.
    """
    deadline = time.monotonic() + STEP_BUDGET_S
    while time.monotonic() < deadline:
        await client.teleport(angles)
        if await client.wait_status(
            lambda s: s.homed and max_deg_error(s.angles, angles) < 1.0, timeout=0.5
        ):
            return
    raise AssertionError("teleport never took effect")


@pytest.mark.timeout(240)
async def test_live_sim_session_over_protocol_v2(daemon: LiveDaemon):
    """One full session against the real runtime: handshake, live STATUS
    decode, the un-homed gate, reset/enable, a queued move to COMPLETE,
    jog preemption of planned motion, stop, and the e-stop latch plus
    recovery — with cancelled commands proven never to complete and the
    index allocator proven never to rewind."""
    park = park_deg()
    async with daemon.client() as client:
        # -- handshake ---------------------------------------------------
        ping = await client.ping()
        assert ping is not None and ping.hardware_connected is False
        assert await client.is_simulator() is True

        # -- live STATUS: header, freshness, and a second opinion on the
        #    joint angles from the msgpack QUERY path.
        assert await client.wait_status(lambda s: s.link_ok == 1, timeout=STEP_BUDGET_S)
        frames = []
        async with asyncio.timeout(STEP_BUDGET_S):
            async for status in client.stream_status():
                frames.append(status)
                if len(frames) == 5:
                    break
        assert [f.proto_version for f in frames] == [2] * 5
        assert all(b.seq > a.seq for a, b in zip(frames, frames[1:]))
        assert all(b.mono_time_ns > a.mono_time_ns for a, b in zip(frames, frames[1:]))
        assert all(f.link_ok == 1 and f.simulator_active for f in frames)
        assert all(f.data_age_ms < 500 for f in frames)
        # The gap counter is fed by the datagram callback, not by this
        # consumer, so it sees every packet the runtime actually sent.
        assert client.status_seq_gaps == 0
        assert frames[-1].homed is False, "a fresh runtime must not claim a home reference"
        queried = await client.angles()
        assert queried is not None
        assert max_deg_error(queried, frames[-1].angles) < 1.0, (
            "the binary STATUS decode and the msgpack ANGLES query disagree"
        )

        # -- planned motion is gated before homing -----------------------
        with pytest.raises(RobotError) as booted:
            await client.move_j(park, duration=1.5)
        assert booted.value.code == ErrorCode.SYS_CONTROLLER_DISABLED

        async def unhomed_move():
            await client.move_j(park, duration=1.5)

        gate = await enable(client, unhomed_move)
        assert gate is not None, "an un-homed planned move must not be accepted"
        assert gate.code == ErrorCode.MOTN_NOT_HOMED

        # -- teleport (sim-only) establishes the home reference -----------
        await teleport_to(client, park)

        # -- queued move: ack index, COMPLETE push, real closed-loop motion.
        #    The sim's cascade treats the commanded velocity as a cap, so a
        #    short move carries a few degrees of tracking lag; the tolerance
        #    proves motion toward the target, not servo-grade tracking.
        target = list(park)
        target[0] += 12.0
        index = await client.move_j(target, duration=1.5)
        assert index >= 0
        assert await client.wait_command(index, timeout=STEP_BUDGET_S) is True
        assert await client.wait_status(
            lambda s: s.completed_index >= index, timeout=STEP_BUDGET_S
        )
        landed = await client.angles()
        assert landed is not None
        assert landed[0] > park[0] + 5.0
        assert abs(landed[0] - target[0]) < 6.0

        # -- a jog preempts planned motion --------------------------------
        preempted = await client.move_j(park, duration=6.0)
        assert await client.wait_status(
            lambda s: s.executing_index == preempted, timeout=STEP_BUDGET_S
        )
        before = (await client.angles())[0]
        for _ in range(4):  # UI-style jog stream
            await client.jog_j(0, 0.25, duration=0.4)
            await asyncio.sleep(0.05)
        assert await client.wait_status(
            lambda s: s.executing_index == -1 and s.queued_segments == 0,
            timeout=STEP_BUDGET_S,
        ), "the jog must cancel the planned move"
        assert await client.wait_status(
            lambda s: s.angles[0] > before + 1.0, timeout=STEP_BUDGET_S
        ), "the jog must physically drive the sim"
        assert await client.wait_status(
            lambda s: s.action_state == ActionState.IDLE and abs(s.speeds[0]) < 0.05,
            timeout=STEP_BUDGET_S,
        ), "the jog duration watchdog must self-terminate the motion"
        assert await client.wait_command(preempted, timeout=1.5) is False, (
            "a preempted command must never complete"
        )

        # -- stop {clear_queue} drops everything pending -------------------
        queued_a = await client.move_j(park, duration=6.0)
        queued_b = await client.move_j(target, duration=1.5)
        assert queued_b > queued_a
        assert await client.stop(clear_queue=True) == 1
        state = await client.queue_state()
        assert state is not None
        assert state.queue == []
        assert state.executing_index == -1
        assert await client.wait_command(queued_b, timeout=1.5) is False

        # -- e-stop latches DISABLED with a standing error until reset -----
        assert await client.estop() == 1
        with pytest.raises(RobotError) as latched:
            await client.move_j(park, duration=1.5)
        assert latched.value.code == ErrorCode.SYS_ESTOP_ACTIVE
        standing = await client.error()
        assert standing is not None and standing.code == ErrorCode.SYS_ESTOP_ACTIVE

        # -- reset recovers; a real client retries until a move runs clean --
        assert await client.reset() == 1
        recovered = -1
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline and recovered < 0:
            try:
                attempt = await client.move_j(target, duration=1.5)
            except RobotError as exc:
                assert exc.code in RETRIABLE, f"unexpected rejection while re-enabling: {exc}"
                await asyncio.sleep(0.2)
                continue
            try:
                if await client.wait_command(attempt, timeout=STEP_BUDGET_S):
                    recovered = attempt
            except RobotError as exc:
                assert exc.code in RETRIABLE, f"unexpected failure while re-enabling: {exc}"
                await asyncio.sleep(0.2)
        assert recovered > queued_b, (
            f"the index allocator must never rewind ({recovered} must follow {queued_b})"
        )
        assert await client.error() is None

        assert client.status_seq_gaps == 0, "the STATUS stream lost packets on loopback"


@pytest.mark.slow
@pytest.mark.timeout(400)
async def test_homing_sequence_drives_the_sim_to_the_configured_ready_pose(
    daemon: LiveDaemon,
):
    """The shipped PAR6 homing sequence, run for real: stall detection on
    J0-J4, the hall edge on J5, per-joint references applied, and the arm
    parked at the ready pose the config's ``move_to`` steps command.

    A flag flip cannot satisfy this — the boot pose (every joint reading
    its ``sector_home_offset``) is nowhere near the ready pose.
    """
    async with daemon.client() as client:
        assert await client.wait_status(lambda s: s.link_ok == 1, timeout=STEP_BUDGET_S)
        boot = await client.angles()
        assert boot is not None

        home_index = -1

        async def start_homing():
            nonlocal home_index
            home_index = await client.home()

        assert await enable(client, start_homing) is None
        assert home_index >= 0

        assert await client.wait_status(
            lambda s: s.action_current == "home"
            and s.action_state == ActionState.EXECUTING,
            timeout=STEP_BUDGET_S,
        ), f"homing never started executing; daemon log:\n{daemon.log()}"

        assert await client.wait_command(home_index, timeout=HOMING_BUDGET_S) is True, (
            f"homing did not complete; daemon log:\n{daemon.log()}"
        )
        assert await client.wait_status(lambda s: s.homed, timeout=STEP_BUDGET_S)

        homed = await client.angles()
        assert homed is not None
        ready = ready_pose_deg()
        assert max_deg_error(homed, ready) < 2.5, (
            f"homed pose {homed} is not the configured ready pose {ready}"
        )
        assert max_deg_error(homed, boot) > 10.0, (
            "the sequence must physically re-reference the arm, not just set a flag"
        )


@pytest.mark.timeout(240)
def test_robot_backend_attaches_to_a_running_runtime_then_spawns_its_own(
    daemon: LiveDaemon, monkeypatch, tmp_path
):
    """``Robot.start()``'s reachable-or-spawn contract against the real
    binary: a runtime already answering PING is adopted (and outlives the
    Robot), and with nothing listening a local ``par6d --sim`` is spawned,
    served through the sync client, and torn down with the Robot."""
    robot = Robot(host="127.0.0.1", port=daemon.command_port, timeout=STEP_BUDGET_S)

    # -- adopt the running runtime ---------------------------------------
    assert robot.is_available() is True
    robot.start()
    with robot.create_sync_client(
        status_transport="UNICAST",
        status_port=daemon.status_port,
        status_unicast_host="127.0.0.1",
    ) as client:
        ping = client.ping()
        assert ping is not None and ping.hardware_connected is False
        angles = client.angles()
        assert angles is not None and len(angles) == 6
        assert client.wait_status(lambda s: s.link_ok == 1, timeout=STEP_BUDGET_S)
    robot.stop()
    assert robot.is_available() is True, "an adopted runtime must outlive the Robot"

    # -- spawn one of its own --------------------------------------------
    port = free_udp_port()
    monkeypatch.setenv("PAR6_CONFIG", str(sim_config(tmp_path / "spawned")))
    monkeypatch.setenv("PAR6_BIND", "127.0.0.1")
    monkeypatch.setenv("PAR6_STATUS_TRANSPORT", "unicast")
    monkeypatch.setenv("PAR6_STATUS_HOST", "127.0.0.1")
    monkeypatch.setenv("PAR6_STATUS_PORT", str(free_udp_port()))
    monkeypatch.setenv("PAR6_TELEMETRY_PORT", str(free_udp_port()))
    monkeypatch.setenv("PAR6_COMMAND_PORT", str(port))

    assert robot.is_available(port=port) is False
    try:
        robot.start(port=port, timeout=60.0)
        assert robot.is_available(port=port) is True
    finally:
        robot.stop()

    deadline = time.monotonic() + STEP_BUDGET_S
    while robot.is_available(port=port) and time.monotonic() < deadline:
        time.sleep(0.1)
    assert robot.is_available(port=port) is False, "Robot.stop() must reap what it spawned"


@pytest.mark.timeout(120)
async def test_advertised_motion_profiles_are_the_ones_the_runtime_plans_with(
    daemon: LiveDaemon,
):
    """``Robot.motion_profiles`` must name the runtime's real registry.

    Every advertised profile is driven through ``select_profile`` and read
    back from the PROFILE query, so the list cannot drift from what the
    command plane accepts; a name outside it is refused, so the list is not
    vacuously true.  TOPPRA is registered only by a ``par6d`` built with
    the C++ shim — whichever build is under test, the advertisement and the
    runtime have to agree about it.
    """
    advertised = Robot().motion_profiles
    async with daemon.client() as client:
        with pytest.raises(RobotError) as unknown:
            await client.select_profile("BANG_BANG")
        assert unknown.value.code == ErrorCode.SYS_PROFILE_INVALID

        for name in advertised:
            try:
                await client.select_profile(name)
            except RobotError as e:
                assert e.code == ErrorCode.SYS_PROFILE_INVALID
                assert name == "TOPPRA", f"{name} must be plannable on every build"
                # Documented consequence of a build without the shim: the
                # refusal leaves the previous profile running.
                continue
            assert await client.profile() == name

        # The reverse direction: a runtime that plans with TOPPRA must not
        # be talking to a client that hides it.
        try:
            await client.select_profile("TOPPRA")
        except RobotError:
            pass
        else:
            assert "TOPPRA" in advertised
