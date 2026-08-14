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
import logging
import math
import time

import numpy as np
import pytest
from live_daemon import (
    LiveDaemon,
    free_udp_port,
    repo_assets_dir,
    requires_par6d,
    settle_at,
    sim_config,
)
from waldoctl.shapes import Box

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
        # The runtime enables itself as soon as the RT core clears BOOTING
        # (~0.5 s at this rig's tick), so a move sent in that window is
        # refused as DISABLED and after it as un-homed. Either way it is
        # refused; which gate catches it is a race, so both are accepted
        # and the un-homed gate is proven deterministically just below.
        with pytest.raises(RobotError) as booted:
            await client.move_j(park, duration=1.5)
        assert booted.value.code in (
            ErrorCode.SYS_CONTROLLER_DISABLED,
            ErrorCode.MOTN_NOT_HOMED,
        )

        async def unhomed_move():
            await client.move_j(park, duration=1.5)

        gate = await enable(client, unhomed_move)
        assert gate is not None, "an un-homed planned move must not be accepted"
        assert gate.code == ErrorCode.MOTN_NOT_HOMED

        # -- teleport (sim-only) establishes the home reference -----------
        await teleport_to(client, park)

        # -- queued move: ack index, COMPLETE push, real closed-loop motion.
        #    The post-profile hold closes the tracking residual on position
        #    error alone, so `settled` completion leaves the arm on the
        #    target for real.
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
        assert abs(landed[0] - target[0]) < 1.0

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
def test_robot_start_is_exclusive_spawns_its_own_and_forwards_its_log(
    daemon: LiveDaemon, monkeypatch, tmp_path, caplog
):
    """``Robot.start()`` against the real binary.

    Waldo Commander calls it under ``EXCLUSIVE_START``, whose contract is
    *fail hard when something is already running* — parol6 raises
    ``"Server already running at …"`` (``parol6/robot.py:908-909``); a
    silent attach hands the caller a runtime it does not own.  A client
    that only wants to attach has ``is_available`` plus the client
    factories, which is the path asserted here first.

    ``com_port`` names a serial device; par6d drives a CAN bus, so the
    request is refused rather than dropped on the floor.

    Everything the spawned runtime logs reaches ``logging`` with its
    level and target preserved (parol6 ``robot.py:189-227``); the log
    used to go to an unnamed temp file nobody reads.
    """
    robot = Robot(host="127.0.0.1", port=daemon.command_port, timeout=STEP_BUDGET_S)

    # -- attaching is is_available + a client, never start() -------------
    assert robot.is_available() is True
    with pytest.raises(RuntimeError, match="already"):
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
    assert robot.is_available() is True, "a runtime it never started must survive"

    # -- spawn one of its own --------------------------------------------
    port = free_udp_port()
    assets = repo_assets_dir()
    if assets is not None:
        # The spawned runtime's config lives in a tmp dir; an ffi build looks
        # for its kinematics assets next to the config unless told otherwise.
        monkeypatch.setenv("PAR6_ASSETS", str(assets))
    monkeypatch.setenv("PAR6_CONFIG", str(sim_config(tmp_path / "spawned")))
    monkeypatch.setenv("PAR6_STATUS_TRANSPORT", "unicast")
    monkeypatch.setenv("PAR6_STATUS_HOST", "127.0.0.1")
    monkeypatch.setenv("PAR6_STATUS_PORT", str(free_udp_port()))
    monkeypatch.setenv("PAR6_TELEMETRY_PORT", str(free_udp_port()))

    logs = Robot(host="127.0.0.1", port=port, normalize_logs=True)
    assert logs.is_available() is False
    with pytest.raises(RuntimeError, match="com_port"):
        logs.start(com_port="/dev/ttyACM0")

    try:
        with caplog.at_level(logging.DEBUG, logger="par6d"):
            logs.start(timeout=60.0)
            assert logs.is_available() is True
            forwarded = _await_records(caplog, "par6d", STEP_BUDGET_S)
    finally:
        logs.stop()

    assert forwarded, "the spawned runtime's log never reached logging"
    assert any(r.levelno == logging.INFO for r in forwarded), (
        f"levels must survive normalization: {[r.levelname for r in forwarded]}"
    )
    assert any("command plane on" in r.getMessage() for r in forwarded), (
        f"the runtime's own boot lines must arrive: "
        f"{[r.getMessage() for r in forwarded][:10]}"
    )
    assert not any(r.getMessage().startswith("[20") for r in forwarded), (
        "normalized records carry the message, not the raw env_logger prefix"
    )
    assert any(r.name.startswith("par6d.") for r in forwarded), (
        f"the runtime's target must become the logger name: "
        f"{sorted({r.name for r in forwarded})}"
    )

    deadline = time.monotonic() + STEP_BUDGET_S
    while logs.is_available() and time.monotonic() < deadline:
        time.sleep(0.1)
    assert logs.is_available() is False, "Robot.stop() must reap what it spawned"


def _await_records(caplog, logger_name: str, budget_s: float) -> list:
    """Records logged under *logger_name*, polled until some arrive.

    The forwarder is a reader thread, so the records a just-started
    runtime has already written may not have been processed yet.
    """
    deadline = time.monotonic() + budget_s
    while time.monotonic() < deadline:
        found = [r for r in caplog.records if r.name.startswith(logger_name)]
        if found:
            return found
        time.sleep(0.05)
    return []


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


@pytest.mark.timeout(120)
async def test_tool_identity_agrees_with_the_runtime(daemon: LiveDaemon):
    """The tool the client advertises must be the tool the runtime has.

    ``Robot.tools`` is what a UI renders before any STATUS arrives, and every
    consumer indexes it with the key the wire reports — so the runtime's TOOLS
    answer, its STATUS ``tool_status`` and its accepted ``select_tool`` all
    have to name the same entry of that collection, and the collection's
    default has to be the tool the runtime is actually fitted with.
    """
    robot = Robot()
    async with daemon.client() as client:
        assert await client.wait_status(lambda s: s.link_ok == 1, timeout=STEP_BUDGET_S)
        reported = await client.tools()
        assert reported is not None

        fitted = robot.tools[reported.tool]
        assert fitted is robot.tools.default, (
            f"the runtime is fitted with {reported.tool}, but the client "
            f"defaults to {robot.tools.default.key}"
        )
        for key in reported.available:
            assert key in robot.tools, f"{key} is not in {[t.key for t in robot.tools.available]}"

        assert await client.wait_status(lambda s: s.tool_status_present, timeout=STEP_BUDGET_S)
        broadcast = await client.wait_status(lambda s: True, timeout=STEP_BUDGET_S)
        assert broadcast
        streamed_key = client._shared_status.tool_status.key
        assert robot.tools[streamed_key] is fitted

        # The runtime accepts the key it reported, and the bound tool of a
        # client built without any specs is that same tool.  Queued commands
        # are gated on ENABLED, so the controller is reset first.
        async def select():
            assert await client.select_tool(reported.tool) >= 0

        assert await enable(client, select) is None
        assert client.tool.key == fitted.key


@pytest.mark.timeout(240)
async def test_servo_j_stream_drives_the_arm_and_leaves_the_controller_usable(
    daemon: LiveDaemon,
):
    """A ``servo_j`` stream at this rig's tick rate must drive the arm and
    leave the controller usable — the path no e2e covered.

    Two defects lived here.  The RT streaming watchdog is derived from
    config SECONDS (``stream.command_timeout_s / tick_dt_s``), and at a
    tick period this long it rounded down to a single tick; read in the
    tick's error phase but fed in its later dispatch phase, even a stream
    landing a setpoint on every tick showed one tick of age at every
    check, so ``RTI_LINK_LOST`` latched one tick in and the controller
    stayed DISABLED for the rest of the session.  Separately the stream
    adapter forwarded the OTG's *terminal* velocity, which is 0 whenever
    the minimum-time move to the current target finishes inside one tick
    — and the driver treats commanded velocity as a cap, so the position
    channel advanced against a zero-velocity cap and the arm did not move.

    The stream below is the shape that exposes the second defect: a
    target nudged a little further every cycle, the way a UI slider or a
    teleoperation source emits one, so each setpoint is reachable within
    a tick.  All of it is asserted here: the tracking, the clean error
    surface, and a session that still works afterwards.
    """
    park = park_deg()
    async with daemon.client() as client:
        assert await client.wait_status(lambda s: s.link_ok == 1, timeout=STEP_BUDGET_S)
        await teleport_to(client, park)
        start = (await client.angles())[0]

        # A UI's streaming cadence (20 Hz), stepping the target by a
        # fraction of a degree per cycle — each one inside a single tick's
        # travel, so nothing but the commanded velocity keeps the arm
        # moving.
        step_deg = 0.25
        cycles = 160
        target = list(park)
        for _ in range(cycles):
            target[0] += step_deg
            await client.servo_j(target)
            await asyncio.sleep(0.05)

        landed = await client.angles()
        assert landed is not None
        commanded = cycles * step_deg
        assert landed[0] > start + 0.5 * commanded, (
            f"the servo stream never drove J0 ({start:.3f} -> {landed[0]:.3f} deg, "
            f"{commanded:.3f} deg of target stepped); daemon log:\n{daemon.log()}"
        )
        assert await client.error() is None, (
            f"streaming must not latch an RT error; daemon log:\n{daemon.log()}"
        )
        assert "RtiLinkLost" not in daemon.log()

        # The controller survived the stream: a queued move runs to
        # completion instead of being refused as DISABLED.
        index = await client.move_j(park, duration=1.5)
        assert await client.wait_command(index, timeout=STEP_BUDGET_S) is True, (
            f"the stream left the controller unusable; daemon log:\n{daemon.log()}"
        )


@pytest.mark.timeout(120)
async def test_estop_and_motion_predicates_answer_from_the_live_runtime(
    daemon: LiveDaemon,
):
    """``is_estop_pressed`` / ``is_robot_stopped`` — the two palette
    entries parol6 has and par6 did not (``async_client.py:1135,1148``).

    Both read live telemetry rather than a cached status, so the e-stop
    predicate has to follow a real ``estop()`` latch and the motion
    predicate has to tell a moving arm from a parked one.
    """
    park = park_deg()
    async with daemon.client() as client:
        assert await client.wait_status(lambda s: s.link_ok == 1, timeout=STEP_BUDGET_S)
        await teleport_to(client, park)

        assert await client.is_estop_pressed() is False
        assert await client.is_robot_stopped() is True

        # A long move: while it runs the arm is not stopped.
        target = list(park)
        target[0] += 20.0

        async def start_move():
            assert await client.move_j(target, duration=6.0) >= 0

        assert await enable(client, start_move) is None
        assert await client.wait_status(
            lambda s: abs(s.speeds[0]) > 0.05, timeout=STEP_BUDGET_S
        ), "the move never moved the arm"
        assert await client.is_robot_stopped() is False

        await client.estop()
        assert await client.wait_status(
            lambda s: s.io[4] == 0, timeout=STEP_BUDGET_S
        ), "the e-stop never reached the I/O surface"
        assert await client.is_estop_pressed() is True

        # The latch and the arm are two different facts, and they do not
        # become true in the same tick: SAFETY_STOP commands 0 Nm
        # (crates/par6-rt/src/dispatch.rs:94) while the arm is still
        # carrying a move, so the e-stop is visible on the I/O surface
        # before the joints read zero. `is_robot_stopped` reports the
        # arm, not the latch, and asserting it straight off the latch is
        # a race — it has to be waited for.
        #
        # Note what this does NOT establish: on hardware, 0 Nm means a
        # torque-held arm sags under gravity, where parol6's steppers
        # hold. The default sim plant does not model that (the arm here
        # halts within 0.01deg of where the latch caught it), so no CI
        # tier currently exercises e-stop sag. Tracked as issue #22.
        assert await client.wait_status(
            lambda s: max(abs(v) for v in s.speeds) < 0.01, timeout=STEP_BUDGET_S
        ), "the arm never came to rest under the e-stop latch"
        assert await client.is_robot_stopped() is True

        assert await client.reset() == 1
        assert await client.wait_status(lambda s: s.io[4] == 1, timeout=STEP_BUDGET_S)
        assert await client.is_estop_pressed() is False


@pytest.mark.timeout(180)
async def test_jog_lookahead_stops_the_measured_arm_short_of_the_soft_limit(
    daemon: LiveDaemon,
):
    """``jog_j`` into a soft limit through the real protocol, with the sim's
    closed-loop driver supplying genuine tracking lag.

    The jog engine's lookahead runs on its own integrated target while the
    hard clamp -- and the operator -- see the MEASURED pose, so the
    question that matters at the boundary is whether the arm the STATUS
    broadcast reports comes to rest short of the configured soft limit and
    stays there while the button is still held.  The direction block is
    asserted behaviorally, within one jog session: continued same-direction
    jogging advances nothing, and the opposite direction clears it and
    moves away.  (The RT's per-direction blocked mask is snapshot-only and
    never carried by STATUS, so the latch has no wire bit to read; the
    mask itself is pinned at the RT layer in ``core_modes.rs``.)
    """
    cfg = _cfg.load_robot_config()
    limit_deg = math.degrees(cfg["joints"][0]["limits"]["soft_max_rad"])
    park = park_deg()
    start = list(park)
    start[0] = limit_deg - 40.0

    async with daemon.client() as client:
        assert await client.wait_status(lambda s: s.link_ok == 1, timeout=STEP_BUDGET_S)
        await teleport_to(client, start)

        # A UI-style jog stream toward the limit at half speed.  Every
        # STATUS frame seen on the way goes into the trace, so a transient
        # excursion past the limit cannot hide between assertions.  The
        # 0.4 s watchdog never expires between 0.1 s-spaced sends, so the
        # whole stream is ONE jog session and the block latch stays live.
        trace: list[float] = []

        def track(s) -> bool:
            trace.append(float(s.angles[0]))
            return False

        for _ in range(20):
            await client.jog_j(0, 0.5, duration=0.4)
            await client.wait_status(track, timeout=0.1)

        rest = (await client.angles())[0]
        assert rest > start[0] + 10.0, (
            f"the jog never moved: {start[0]:.2f} -> {rest:.2f} deg"
        )
        assert rest < limit_deg - 1.0, (
            f"the measured angle must rest short of the soft limit: "
            f"{rest:.2f} vs {limit_deg:.2f} deg"
        )

        # Still pressing the same direction: the latched block must hold
        # the measured pose exactly where it stopped.
        for _ in range(10):
            await client.jog_j(0, 0.5, duration=0.4)
            await client.wait_status(track, timeout=0.1)
        held = (await client.angles())[0]
        assert abs(held - rest) < 0.5, (
            f"blocked direction advanced under the held button: "
            f"{rest:.2f} -> {held:.2f} deg"
        )
        assert max(trace) < limit_deg - 1.0, (
            f"the measured angle came within 1 deg of the soft limit in "
            f"flight: max {max(trace):.2f} vs {limit_deg:.2f}"
        )

        # Let the watchdog self-terminate, then jog the opposite way: the
        # block clears and the arm moves off the limit.
        assert await client.wait_status(
            lambda s: abs(s.speeds[0]) < 0.05, timeout=STEP_BUDGET_S
        ), "the jog watchdog never self-terminated"
        for _ in range(8):
            await client.jog_j(0, -0.3, duration=0.4)
            await asyncio.sleep(0.1)
        assert await client.wait_status(
            lambda s: s.angles[0] < rest - 2.0, timeout=STEP_BUDGET_S
        ), "the opposite direction must clear the block and move away"


#: A posture whose TCP orientation has three substantial rotation
#: components (~170 / 11 / 165 deg) — the only kind of pose that can tell
#: the two readings of `[rx, ry, rz]` apart. It is the same
#: well-conditioned start posture the Rust cartesian e2e test uses:
#: inside every soft window and clear of the arm's own meshes.
TILTED_POSTURE_DEG = [0.0, -75.0, 305.0, 20.0, -30.0, 180.0]


@pytest.mark.timeout(180)
async def test_tcp_pose_survives_the_client_runtime_client_round_trip(daemon: LiveDaemon):
    """A pose read off the wire, sent straight back, must not move the arm.

    This is the teach-and-replay path: Waldo Commander decodes the STATUS
    pose matrix with pinokin, shows those scalars, and its motion recorder
    emits them verbatim as a ``move_l``/``move_j`` target.  So the six
    numbers have to mean the same rotation in all three places -- the
    client's decode, ``Robot.fk``'s decode, and the runtime's re-encode.
    Read one of them in the URDF fixed-axis order (``Rz*Ry*Rx``) and this
    posture's replay lands 36.7 deg from where it was taught; a tool-down
    pose lands with its wrist angle negated.
    """
    async with daemon.client() as client:
        assert await client.wait_status(lambda s: s.link_ok == 1, timeout=STEP_BUDGET_S)

        async def unhomed_move():
            await client.move_j(TILTED_POSTURE_DEG, duration=1.5)

        await enable(client, unhomed_move)
        await teleport_to(client, TILTED_POSTURE_DEG)

        taught = await client.pose()
        assert taught is not None
        if not all(math.isfinite(v) for v in taught):
            pytest.skip("runtime built without kinematics: STATUS carries no pose")
        angles = await client.angles()
        assert angles is not None

        # The pose the waldoctl backend computes for the same
        # configuration, decomposed by pinokin -- the decode the frontend
        # readout uses, and an oracle that shares no code with the runtime.
        expected = np.zeros(6)
        Robot().fk(np.radians(angles), expected)
        position_error_mm = float(
            np.max(np.abs(np.asarray(taught[:3]) - expected[:3] * 1000.0))
        )
        assert position_error_mm < 2.0, (
            f"the reported TCP position disagrees with Robot.fk by "
            f"{position_error_mm:.2f} mm: {taught[:3]} vs {expected[:3] * 1000.0}"
        )
        assert max_deg_error(taught[3:], np.degrees(expected[3:])) < 0.5, (
            f"the reported TCP rotation disagrees with Robot.fk: {taught[3:]} vs "
            f"{np.degrees(expected[3:])} -- the client and the kinematics backend "
            f"are decomposing the same matrix in different conventions"
        )

        # Replay it. The arm is already in this pose, so a runtime that
        # reads the numbers the way they were written has a null move to
        # run; one that reads them the other way round swings the wrist to
        # an orientation nobody commanded (or fails the solve outright).
        index = await client.move_j(pose=taught)
        assert index >= 0
        assert await client.wait_command(index, timeout=STEP_BUDGET_S) is True
        replayed = await client.pose()
        assert replayed is not None
        assert max_deg_error(replayed[3:], taught[3:]) < 1.0, (
            f"replaying the taught pose turned the tool: {taught[3:]} -> {replayed[3:]}"
        )
        assert max_deg_error(replayed[:3], taught[:3]) < 5.0, (
            f"replaying the taught pose moved the TCP: {taught[:3]} -> {replayed[:3]}"
        )


#: The extended sweep posture the Rust collision tests drive: the arm
#: stretched out (its own meshes clear of each other), rotated back
#: around J0 so the mid-sweep TCP sits in open workspace where a keep-out
#: can be parked.
SWEEP_START_DEG = [-40.0, -20.0, 235.0, 0.0, 15.0, 180.0]


@pytest.mark.timeout(180)
async def test_jog_streams_are_gated_by_the_collision_world(daemon: LiveDaemon):
    """Streaming motion is gated by the collision world (issue #19).

    A UI-style jog stream toward a keep-out must never carry the arm into
    it: the runtime's velocity-scaled lookahead either refuses the jog
    outright or stops the stream short, and either way the verdict reaches
    the STATUS broadcast as ``collision_active`` with the keep-out named.
    From INSIDE the keep-out (dropped over the arm), the outward jog must
    still run -- streaming is the only way out, and refusing it would trap
    the arm.  Everything here drives a real ``par6d --sim`` over real UDP
    with the real client; the keep-out is placed from a pose read off the
    live broadcast, never a transcribed constant.
    """
    async with daemon.client() as client:
        assert await client.wait_status(lambda s: s.link_ok == 1, timeout=STEP_BUDGET_S)

        # Park the keep-out on the TCP position at mid-sweep.
        mid = list(SWEEP_START_DEG)
        mid[0] += 40.0
        await settle_at(client, mid)
        pose = await client.pose()
        assert pose is not None
        if not all(math.isfinite(v) for v in pose[:3]):
            pytest.skip("runtime built without kinematics: STATUS carries no pose")
        center_m = [v / 1000.0 for v in pose[:3]]
        radius_m = math.hypot(center_m[0], center_m[1])
        assert await client.set_shapes(
            [
                Box(
                    name="keepout",
                    x=0.1,
                    y=0.1,
                    z=0.1,
                    pose=(center_m[0], center_m[1], center_m[2], 0.0, 0.0, 0.0),
                )
            ]
        )

        # Two box widths short of the box centre along the J0 arc, then a
        # UI-style jog stream toward it.  Every frame seen on the way goes
        # into the trace, so an excursion into the box cannot hide between
        # assertions.
        start = list(mid)
        start[0] -= math.degrees(0.2 / radius_m)
        await teleport_to(client, start)
        trace: list[float] = []

        def blocked_and_latched(s) -> bool:
            trace.append(float(s.angles[0]))
            return bool(s.collision_active) and any(
                "keepout" in name for pair in s.collision_pairs for name in pair
            )

        blocked = False
        deadline = time.monotonic() + STEP_BUDGET_S
        while time.monotonic() < deadline and not blocked:
            await client.jog_j(0, 0.5, duration=0.4)
            blocked = await client.wait_status(blocked_and_latched, timeout=0.1)
        assert blocked, (
            "the jog toward the keep-out was never blocked: the stream gate "
            "let the arm drive at the box"
        )
        assert await client.wait_status(
            lambda s: abs(s.speeds[0]) < 0.05, timeout=STEP_BUDGET_S
        ), "the blocked jog never came to rest"
        rest = (await client.angles())[0]
        assert rest < mid[0] - 5.0, (
            f"the blocked jog stopped at {rest:.2f} deg -- inside the keep-out "
            f"centred at {mid[0]:.2f}"
        )
        assert max(trace) < mid[0] - 5.0, (
            f"the arm entered the keep-out in flight: max {max(trace):.2f} deg"
        )

        # A keep-out dropped over the arm: the outward jog still runs.
        await settle_at(client, mid)
        escaped = False
        deadline = time.monotonic() + STEP_BUDGET_S
        while time.monotonic() < deadline and not escaped:
            await client.jog_j(0, -0.3, duration=0.4)
            escaped = await client.wait_status(
                lambda s: s.angles[0] < mid[0] - 3.0, timeout=0.1
            )
        assert escaped, (
            "the escaping jog out of the keep-out never moved the arm: the "
            "gate is refusing the only way out"
        )
