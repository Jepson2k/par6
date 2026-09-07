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
import json
import logging
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest
from live_daemon import (
    TICK_DT_S,
    LiveDaemon,
    angles_now,
    daemon_env,
    free_udp_port,
    pose_now,
    requires_par6d,
    settle_at,
    sim_config,
    teleport_to,
)
from waldoctl.shapes import Box

from par6 import config as _cfg
from par6.client import AsyncRobotClient, RobotError
from par6.client.dry_run_client import DryRunRobotClient
from par6.protocol.constants import ActionState, ControllerMode, ErrorCode
from par6.robot import Robot

pytestmark = [pytest.mark.e2e, requires_par6d]

#: Wall-clock ceiling for one session step (boot, settle, a short move).
STEP_BUDGET_S = 20.0

#: Fraction of the cartesian ceiling the streamed servo_l tests drive at.
SERVO_L_SPEED = 0.6
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
    return [math.degrees(v) for v in _cfg.config().park_pose_rad()]


def ready_pose_deg() -> list[float]:
    """Where the shipped homing sequence leaves the arm.

    Replays the sequence's ``move_to`` steps in order and keeps the last
    commanded position per joint — derived from the same config the daemon
    executes, never a transcribed constant.
    """
    return np.degrees(_cfg.homing_ready_pose_rad()).tolist()


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
        assert [f.proto_version for f in frames] == [3] * 5
        assert all(b.seq > a.seq for a, b in zip(frames, frames[1:]))
        assert all(b.mono_time_ns > a.mono_time_ns for a, b in zip(frames, frames[1:]))
        assert all(f.link_ok == 1 and f.simulator_active for f in frames)
        assert all(f.data_age_ms < 500 for f in frames)
        # The gap counter is fed by the datagram callback, not by this
        # consumer, so it sees every packet the runtime actually sent.
        assert client.status_seq_gaps == 0
        assert frames[-1].homed is False, (
            "a fresh runtime must not claim a home reference"
        )
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
        before = (await angles_now(client))[0]
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
        with pytest.raises(RobotError) as preempt_err:
            await client.wait_command(preempted, timeout=STEP_BUDGET_S)
        assert preempt_err.value.code == ErrorCode.MOTN_CANCELLED, (
            "a preempted command must report its cancellation, not hang or succeed"
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
        for dropped in (queued_a, queued_b):
            with pytest.raises(RobotError) as stop_err:
                await client.wait_command(dropped, timeout=STEP_BUDGET_S)
            assert stop_err.value.code == ErrorCode.MOTN_CANCELLED, (
                f"index {dropped} must report the queue-clearing stop"
            )
        standing = await client.error()
        assert standing is not None and standing.code == ErrorCode.MOTN_CANCELLED, (
            "a queue-clearing stop leaves the cancellation standing"
        )

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
                assert exc.code in RETRIABLE, (
                    f"unexpected rejection while re-enabling: {exc}"
                )
                await asyncio.sleep(0.2)
                continue
            try:
                if await client.wait_command(attempt, timeout=STEP_BUDGET_S):
                    recovered = attempt
            except RobotError as exc:
                assert exc.code in RETRIABLE, (
                    f"unexpected failure while re-enabling: {exc}"
                )
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
            lambda s: (
                s.action_current == "home" and s.action_state == ActionState.EXECUTING
            ),
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

        # home(calibrate=True) on an already-referenced arm re-runs the seek
        # instead of the planned return. What is asserted here is that the
        # client's keyword reaches the runtime — the RT dropping back into
        # HOMING is the only thing a dropped flag could not produce, since
        # the flag-clear path never leaves EXEC. The seek is then abandoned
        # rather than waited out: it is the same ~60 s sequence already run
        # above, and where it ENDS is pinned without the wall clock by
        # `par6d/tests/sim_session.rs::
        # home_calibrate_on_a_referenced_arm_reseeks_instead_of_returning_to_park`.
        recal_index = await client.home(calibrate=True)
        assert recal_index >= 0
        assert await client.wait_status(
            lambda s: s.mode == ControllerMode.HOMING, timeout=STEP_BUDGET_S
        ), f"calibrate=True never entered HOMING; daemon log:\n{daemon.log()}"
        assert await client.stop(clear_queue=True) == 1


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
    port = _spawn_env(monkeypatch, tmp_path, "spawned")

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


def _spawn_env(monkeypatch, tmp_path: Path, tag: str) -> int:
    """Give a ``Robot.start()`` spawn a config, ports and a grant directory of
    its own, and return the command port it will serve on.

    The spawned runtime runs beside the fixture's: without its own grant
    directory it publishes ``loop_tick`` / ``robot_mode`` under the shipped
    names in the default location, and stopping it removes the claim every
    other daemon on the box is holding. An ffi build also looks for its
    kinematics assets next to the config unless told otherwise.
    """
    for key, value in daemon_env(tmp_path / f"{tag}-shm").items():
        if key.startswith("PAR6_"):
            monkeypatch.setenv(key, value)
    monkeypatch.setenv("PAR6_CONFIG", str(sim_config(tmp_path / tag)))
    monkeypatch.setenv("PAR6_STATUS_TRANSPORT", "unicast")
    monkeypatch.setenv("PAR6_STATUS_HOST", "127.0.0.1")
    monkeypatch.setenv("PAR6_STATUS_PORT", str(free_udp_port()))
    return free_udp_port()


def _par6d_pids_serving(port: int) -> list[int]:
    """Every live par6d whose command line binds *port*."""
    found = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as f:
                argv = f.read().split(b"\0")
        except OSError:
            continue
        if not argv or not argv[0].endswith(b"par6d"):
            continue
        try:
            served = argv[argv.index(b"--port") + 1]
        except (ValueError, IndexError):
            continue
        if served == str(port).encode():
            found.append(int(entry))
    return found


def test_a_spawned_runtime_dies_with_the_process_that_spawned_it(monkeypatch, tmp_path):
    """A runtime ``Robot.start()`` spawned must not outlive the program.

    parol6 pins this with ``set_pdeathsig`` (``tests/unit/test_pdeathsig.py``).
    Without it a script that is SIGKILLed — a crashed GUI, a terminal
    closed on it — leaves ``par6d`` running: it keeps the command port,
    keeps the bus, and the next ``start()`` on that port refuses with
    "already running" for a runtime nobody can reach to stop.
    """
    port = _spawn_env(monkeypatch, tmp_path, "orphan")

    # The spawner is a real separate process, so killing it is the dirty
    # exit under test and not something the test runner itself survives.
    script = tmp_path / "spawner.py"
    script.write_text(
        "import sys, time\n"
        "from par6 import Robot\n"
        f"Robot(host='127.0.0.1', port={port}).start(timeout=60.0)\n"
        "print('READY', flush=True)\n"
        "time.sleep(3600)\n"
    )
    spawner = subprocess.Popen(
        [sys.executable, str(script)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        assert spawner.stdout is not None
        assert spawner.stdout.readline().strip() == "READY", (
            spawner.stderr.read() if spawner.stderr else ""
        )
        probe = Robot(host="127.0.0.1", port=port)
        assert probe.is_available() is True
        assert _par6d_pids_serving(port), "the spawner's runtime must be visible"

        os.kill(spawner.pid, signal.SIGKILL)
        spawner.wait(timeout=STEP_BUDGET_S)

        deadline = time.monotonic() + STEP_BUDGET_S
        while time.monotonic() < deadline:
            if not probe.is_available() and not _par6d_pids_serving(port):
                break
            time.sleep(0.1)
        assert not _par6d_pids_serving(port), (
            "par6d outlived the process that spawned it"
        )
        assert probe.is_available() is False
    finally:
        for pid in _par6d_pids_serving(port):
            os.kill(pid, signal.SIGKILL)
        if spawner.poll() is None:
            spawner.kill()
        spawner.wait(timeout=STEP_BUDGET_S)


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
            assert key in robot.tools, (
                f"{key} is not in {[t.key for t in robot.tools.available]}"
            )

        assert await client.wait_status(
            lambda s: s.tool_status_present, timeout=STEP_BUDGET_S
        )
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
        start = (await angles_now(client))[0]

        # A UI's streaming cadence (20 Hz), stepping the target by a
        # fraction of a degree per cycle — each one inside a single tick's
        # travel, so nothing but the commanded velocity keeps the arm
        # moving.
        step_deg = 0.25
        # Enough to reach and hold steady state: both regressions this
        # guards (a watchdog rounding to one tick, and terminal velocity
        # read as a cap) stop the arm inside the first second, and the
        # landing assertion below is a fraction of what was commanded, so
        # it holds at any cycle count.
        cycles = 60
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

    Both read the live runtime rather than a cached status, so the e-stop
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
            lambda s: s.io[-1] == 0, timeout=STEP_BUDGET_S
        ), "the e-stop never reached the I/O surface"
        assert await client.is_estop_pressed() is True

        # The latch and the arm are two different facts, and they do not
        # become true in the same tick: the e-stop is visible on the I/O
        # surface while the arm is still braking off a move, and the
        # brake rings the velocity loop's integral down, so any single
        # below-threshold frame can be a zero crossing of that ring.
        # `is_robot_stopped` reports the arm, not the latch — it counts
        # as stopped only when it holds across consecutive polls.
        #
        # Note what this does NOT establish: on hardware, a limp arm
        # sags under gravity, where parol6's steppers hold. The default
        # kinematic plant does not model that (the arm here halts where
        # the latch caught it), so no CI tier currently exercises
        # e-stop sag.
        deadline = asyncio.get_running_loop().time() + STEP_BUDGET_S
        streak = 0
        while asyncio.get_running_loop().time() < deadline and streak < 3:
            streak = streak + 1 if await client.is_robot_stopped() else 0
            await asyncio.sleep(0.05)
        assert streak >= 3, "the arm never came to rest under the e-stop latch"

        assert await client.reset() == 1
        assert await client.wait_status(lambda s: s.io[-1] == 1, timeout=STEP_BUDGET_S)
        assert await client.is_estop_pressed() is False


@pytest.mark.timeout(120)
async def test_a_wait_that_runs_out_raises_and_the_status_buffer_keeps_its_identities(
    daemon: LiveDaemon,
):
    """Two contracts of the async client over a live stream.

    A ``wait=True`` motion that does not finish within its timeout raises
    ``TimeoutError`` — the index it would otherwise return reads as
    success while the arm is still moving.

    The shared status buffer is filled in place: the containers a consumer
    holds across frames are the same objects frame after frame.
    """
    park = park_deg()
    async with daemon.client() as client:
        assert await client.wait_status(lambda s: s.link_ok == 1, timeout=STEP_BUDGET_S)
        await teleport_to(client, park)

        target = list(park)
        target[0] += 20.0
        with pytest.raises(TimeoutError):
            await client.move_j(target, duration=6.0, wait=True, timeout=0.5)
        await client.stop()

        first = None
        seen = 0
        async with asyncio.timeout(STEP_BUDGET_S):
            async for status in client.stream_status_shared():
                held = (
                    status.collision_pairs,
                    status.warnings,
                    status.link_health,
                    status.homing,
                    status.homing.get("joints"),
                )
                if first is None:
                    first = held
                else:
                    assert all(a is b for a, b in zip(first, held)), (
                        "the buffer rebound a container between frames"
                    )
                seen += 1
                if seen == 5:
                    break
        assert seen == 5


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
    moves away.  (The RT's per-direction blocked mask IS carried: the
    server folds it into STATUS ``joint_en`` while in JOG, which is what
    lets a frontend grey the button the RT actually stopped honoring —
    ``sim_session.rs`` pins that.)
    """
    limit_deg = math.degrees(_cfg.config().soft_limits_rad()[0][1])
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

        rest = (await angles_now(client))[0]
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
        held = (await angles_now(client))[0]
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
async def test_tcp_pose_survives_the_client_runtime_client_round_trip(
    daemon: LiveDaemon,
):
    """A pose read off the wire, sent straight back, must not move the arm.

    This is the teach-and-replay path: Waldo Commander decodes the STATUS
    pose matrix itself, shows those scalars, and its motion recorder
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
        assert all(math.isfinite(v) for v in taught), (
            f"STATUS pose not finite: {taught}"
        )
        angles = await client.angles()
        assert angles is not None

        # The wire convention itself: the six numbers the client decoded
        # must re-compose (intrinsic XYZ, written out here rather than
        # borrowed from any library) into the matrix STATUS carries.
        status = await client.status()
        assert status is not None
        T_status = np.asarray(status.pose, dtype=np.float64).reshape(4, 4)
        rx, ry, rz = np.radians(taught[3:])
        cx, sx, cy, sy, cz, sz = (
            math.cos(rx),
            math.sin(rx),
            math.cos(ry),
            math.sin(ry),
            math.cos(rz),
            math.sin(rz),
        )
        R = (
            np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
            @ np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
            @ np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
        )
        assert np.allclose(T_status[:3, 3], taught[:3], atol=1e-6), (
            f"pose() and STATUS disagree on the TCP position: {taught[:3]} vs {T_status[:3, 3]}"
        )
        assert np.allclose(T_status[:3, :3], R, atol=1e-6), (
            f"the client's rpy decode does not re-compose into the STATUS matrix:\n"
            f"{taught[3:]} ->\n{R}\nvs\n{T_status[:3, :3]}"
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
        assert all(math.isfinite(v) for v in pose[:3]), (
            f"STATUS pose not finite: {pose}"
        )
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
        rest = (await angles_now(client))[0]
        assert rest < mid[0] - 5.0, (
            f"the blocked jog stopped at {rest:.2f} deg -- inside the keep-out "
            f"centred at {mid[0]:.2f}"
        )
        assert max(trace) < mid[0] - 5.0, (
            f"the arm entered the keep-out in flight: max {max(trace):.2f} deg"
        )

        # The refusal itself must reach a caller that never awaits a
        # fire-and-forget reply (issue #23).  From the resting pose at the
        # gate's boundary, jogs at the box keep being turned away, and the
        # refusal stands in error() naming the keep-out.
        refusal: RobotError | None = None
        deadline = time.monotonic() + STEP_BUDGET_S
        while time.monotonic() < deadline and refusal is None:
            assert await client.jog_j(0, 0.5, duration=0.4) == 1
            await asyncio.sleep(0.1)
            refusal = await client.error()
        assert refusal is not None, (
            f"the refused jog never surfaced through error(); "
            f"daemon log:\n{daemon.log()}"
        )
        assert refusal.code == ErrorCode.SYS_SELF_COLLISION, str(refusal)
        assert "keepout" in refusal.cause, str(refusal)

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
        # The accepted escape cleared the standing refusal, like any
        # accepted motion command.
        assert await client.error() is None, f"daemon log:\n{daemon.log()}"


@pytest.mark.timeout(180)
async def test_preview_refuses_the_move_the_runtime_refuses(daemon: LiveDaemon):
    """The dry-run preview and the runtime agree about a keep-out.

    The gap this closes: the preview used to draw a confident path through
    a keep-out the runtime brakes on, so an editor showed a move that the
    arm then refused. Both sides get the same shape and the same move here,
    and both must reject it, name the same colliding pair, and leave the
    arm where it stood. The same move with the keep-out cleared must run on
    both, so the refusal is the shape's doing and not the move's.
    """
    async with daemon.client() as client:
        assert await client.wait_status(lambda s: s.link_ok == 1, timeout=STEP_BUDGET_S)

        target = list(SWEEP_START_DEG)
        target[0] += 80.0
        await settle_at(client, SWEEP_START_DEG)

        # Park the keep-out on the TCP position half way along the move,
        # read off the live broadcast rather than transcribed.
        mid = list(SWEEP_START_DEG)
        mid[0] += 40.0
        await settle_at(client, mid)
        pose = await client.pose()
        assert pose is not None
        keepout = Box(
            name="keepout",
            x=0.1,
            y=0.1,
            z=0.1,
            pose=(pose[0] / 1000.0, pose[1] / 1000.0, pose[2] / 1000.0, 0.0, 0.0, 0.0),
        )
        await settle_at(client, SWEEP_START_DEG)

        preview = DryRunRobotClient(initial_joints_deg=SWEEP_START_DEG)
        assert preview.set_shapes([keepout])
        assert await client.set_shapes([keepout])

        with pytest.raises(RobotError) as preview_refusal:
            preview.move_j(angles=target)
        assert preview.angles() == pytest.approx(SWEEP_START_DEG, abs=1e-6), (
            "a refused preview must not advance the previewed pose"
        )

        # The runtime's collision gate runs at dispatch, so the refusal
        # rides the COMPLETE push, not the queue ack.
        with pytest.raises(RobotError) as live_refusal:
            await client.move_j(target, wait=True, timeout=STEP_BUDGET_S)

        assert preview_refusal.value.code == live_refusal.value.code, (
            f"preview refused with {preview_refusal.value.code}, runtime with "
            f"{live_refusal.value.code}"
        )
        assert preview_refusal.value.code == ErrorCode.SYS_SELF_COLLISION
        for refusal in (preview_refusal.value, live_refusal.value):
            assert "shape:keepout" in refusal.cause, (
                f"the refusal must name the keep-out in the reporting "
                f"vocabulary: {refusal.cause!r}"
            )
        assert (await client.angles()) == pytest.approx(SWEEP_START_DEG, abs=1.0), (
            "the refused move drove the arm"
        )

        # The runtime latches the pairs it refused on. The client-side
        # checker, asked about the same configuration, must name the same
        # ones — that is what makes a preview's highlight trustworthy.
        runtime_pairs: set[tuple[str, ...]] = set()

        def latch(s) -> bool:
            if not s.collision_active:
                return False
            runtime_pairs.update(tuple(sorted(p)) for p in s.collision_pairs)
            return True

        assert await client.wait_status(latch, timeout=STEP_BUDGET_S), (
            "the refusal never reached STATUS"
        )
        # Local kinematics only — no connection, and the same packaged
        # config and fitted tool the runtime booted with.
        robot = Robot()
        robot.apply_shapes([keepout])
        local_pairs = {
            tuple(sorted(p))
            for p in robot.colliding_pairs(np.radians(mid))
            if "shape:keepout" in p
        }
        assert runtime_pairs & local_pairs, (
            f"the client-side checker names {sorted(local_pairs)} where the "
            f"runtime latched {sorted(runtime_pairs)}"
        )
        assert robot.in_collision(np.radians(mid))
        assert robot.min_distance(np.radians(mid)) < 0.0
        assert robot.check_trajectory(np.radians([SWEEP_START_DEG, mid])) == 1, (
            "check_trajectory must find the keep-out at the second waypoint"
        )

        # Same move, no keep-out: both sides run it.
        assert preview.set_shapes([])
        assert await client.set_shapes([])
        assert preview.move_j(angles=target) is not None
        assert preview.angles() == pytest.approx(target, abs=1e-6)
        await client.move_j(target, wait=True, timeout=STEP_BUDGET_S)
        assert await client.wait_status(
            lambda s: max_deg_error(s.angles, target) < 2.0, timeout=STEP_BUDGET_S
        ), "the move the keep-out was blocking never ran once it was cleared"


@pytest.mark.timeout(180)
async def test_cartesian_streams_drive_the_arm_and_are_collision_gated(
    daemon: LiveDaemon,
):
    """``servo_l`` and ``servo_j(pose=...)`` end to end.

    Both carry a substantial daemon implementation — IK every datagram, the
    RT stream limiter, the collision gate — and neither had any end-to-end
    coverage: a break anywhere along that path would have shown up first on
    a real arm. Streamed at a UI-like cadence here, and then aimed into a
    keep-out, which must stop them.
    """

    class Streamer:
        """UI-style streaming: each datagram advances the COMMANDED target
        a few mm, the way a 50 Hz frontend integrates a gesture. Stepping
        from the measurement instead feeds the plant's tracking lag back
        into the target and limit-cycles the arm."""

        def __init__(self, client, goal, send):
            self.client = client
            self.goal = goal
            self.send = send
            self.target: list[float] | None = None

        async def step(self):
            if self.target is None:
                self.target = list(await pose_now(self.client))
            for i in range(3):
                self.target[i] += max(-5.0, min(5.0, self.goal[i] - self.target[i]))
            await self.send(self.target)

    async def stream_toward(client, goal, send, budget=STEP_BUDGET_S):
        """Stream the target to *goal* and keep it there until the arm has
        settled on it, so the next phase opens its session on a resting
        arm."""
        streamer = Streamer(client, goal, send)
        deadline = time.monotonic() + budget
        while time.monotonic() < deadline:
            await streamer.step()
            if await client.wait_status(
                lambda s: abs(s.pose[11] - goal[2]) < 5.0 and s.tcp_speed < 5.0,
                timeout=0.05,
            ):
                return True
        return False

    async with daemon.client() as client:
        assert await client.wait_status(lambda s: s.link_ok == 1, timeout=STEP_BUDGET_S)
        await settle_at(client, SWEEP_START_DEG)

        # --- servo_l: stream a straight TCP descent and watch it arrive.
        start = await client.pose()
        assert start is not None
        goal = list(start)
        goal[2] -= 40.0
        arrived = await stream_toward(
            client, goal, lambda p: client.servo_l(p, speed=SERVO_L_SPEED)
        )
        assert arrived, (
            f"servo_l never reached the streamed target: "
            f"{(await pose_now(client))[:3]} vs {goal[:3]}"
        )

        # --- servo_j(pose=...): the same target through the joint-space
        #     streamer, which IKs the pose rather than interpolating it.
        back = list(start)
        arrived = await stream_toward(
            client, back, lambda p: client.servo_j(pose=p, speed=0.6)
        )
        assert arrived, "servo_j(pose=...) never reached the streamed target"

        # --- the collision gate refuses a stream aimed into a keep-out.
        here = await client.pose()
        assert here is not None
        below = list(here)
        below[2] -= 60.0
        keepout = Box(
            name="floor",
            x=0.4,
            y=0.4,
            z=0.1,
            pose=(below[0] / 1000.0, below[1] / 1000.0, below[2] / 1000.0, 0, 0, 0),
        )
        assert await client.set_shapes([keepout])

        # A refused datagram cancels the session and latches the collision
        # until an ADMITTED datagram clears it; every later step lands
        # deeper in the shape, so the latch outlives a missed status frame.
        # Every frame's height is kept so an excursion into the shape cannot
        # hide between assertions.
        floor = below[2] + 20.0
        z_seen: list[float] = []

        def gated(s) -> bool:
            z_seen.append(float(s.pose[11]))
            return bool(s.collision_active)

        streamer = Streamer(
            client, below, lambda p: client.servo_l(p, speed=SERVO_L_SPEED)
        )
        blocked = False
        deadline = time.monotonic() + STEP_BUDGET_S
        while time.monotonic() < deadline and not blocked:
            await streamer.step()
            blocked = await client.wait_status(gated, timeout=0.05)
        assert blocked, (
            "a servo_l streamed into a keep-out was never gated: the stream "
            "gate let the arm drive at the shape"
        )

        # The refusal ends the session, so the arm brakes to rest in IDLE
        # instead of carrying on to the streamed goal at the shape's centre.
        def at_rest(s) -> bool:
            z_seen.append(float(s.pose[11]))
            resting = max(abs(v) for v in s.speeds) < 0.05
            return s.mode == ControllerMode.IDLE and resting

        assert await client.wait_status(at_rest, timeout=STEP_BUDGET_S), (
            "the gate refused the datagram but never cancelled the session"
        )
        # How deep the arm gets is the gate's reaction plus the braking
        # distance, and housekeeping only reacts as often as it runs. A
        # box that cannot hold its loop period reacts later and coasts
        # further, which is the loop failing rather than the gate.
        #
        # Two different numbers say so, and only together: `overrun_count`
        # counts ticks whose WORK ran past the next deadline, and the
        # period is measured wake to wake, so it also carries the kernel's
        # wake-up latency. A loaded runner shows up as a period over
        # budget with no overruns at all — work that fits, woken late — so
        # a message that printed only the overruns would call that box
        # healthy. The intrusion is also given in ticks of travel, since
        # one late tick is the smallest coast the gate can produce.
        stats = await client.loop_stats()
        if stats is not None:
            budget_ms = 1e3 / stats.target_hz
            p99_ms = stats.p99_period_s * 1e3
            intrusion_mm = floor - min(z_seen)
            ceiling_mm_s = 1e3 * float(_cfg.config().motion()["jog_l_linear_max_m_s"])
            per_tick_mm = SERVO_L_SPEED * ceiling_mm_s / stats.target_hz
            loop_note = (
                f"the RT loop overran {stats.overrun_count} of {stats.loop_count} "
                f"ticks and its p99 period was {p99_ms:.2f} ms against a "
                f"{budget_ms:.2f} ms budget ({p99_ms / budget_ms:.3f}x); the arm "
                f"went {intrusion_mm:.1f} mm past, which is "
                f"{intrusion_mm / per_tick_mm:.1f} ticks of travel at "
                f"{per_tick_mm:.1f} mm per tick"
            )
        else:
            loop_note = "loop stats unavailable"
        assert min(z_seen) > floor, (
            f"the gated stream carried the TCP into the keep-out: "
            f"min z {min(z_seen):.1f} vs floor {floor:.1f}. {loop_note}"
        )
        assert await client.set_shapes([])


@pytest.mark.timeout(180)
async def test_the_python_client_drives_the_gripper_and_the_digital_outputs(
    daemon: LiveDaemon,
):
    """Two things only the Rust client had ever exercised against a runtime.

    The jaw: a tool action through the Python client must reach the
    simulated driver and come back as a real tool status, not an ack.

    The lines: ``write_io`` must reach the runtime and come back on the
    STATUS ``io`` array, at the slot the declared layout puts it — the
    array is sized from the ``[io]`` config, so this also pins that a
    client reading it never has to know a hardcoded length.
    """
    async with daemon.client() as client:
        assert await client.wait_status(lambda s: s.link_ok == 1, timeout=STEP_BUDGET_S)
        await settle_at(client, park_deg())

        tools = await client.tools()
        assert tools is not None
        tool = tools.tool

        # A fresh sim boots uncalibrated, and the RT send gate never
        # streams a move to an uncalibrated gripper (the firmware's own
        # gate drops it) — so the daemon refuses the move up front
        # instead of letting it silently move nothing.
        with pytest.raises(RobotError) as refused:
            await client.tool_action(tool, "move", [1.0, 0.5, 400.0], wait=True)
        assert refused.value.code == ErrorCode.COMM_VALIDATION_ERROR
        assert "calibrat" in str(refused.value).lower()

        await client.tool_action(tool, "calibrate", wait=True)

        # The runtime's verbs are `move`, `calibrate`, `stop` and `idle`;
        # the client's open/close convenience resolves onto `move` with a
        # position. The completion push carries the settle verdict, so a
        # close on air reads "target reached, no object" without racing a
        # tool-status poll.
        idx = await client.tool_action(tool, "move", [1.0, 0.5, 400.0], wait=True)
        assert await client.command_verdict(idx) == 3, (
            "a close with nothing between the jaws must complete with "
            "verdict 3 (reached, no object)"
        )
        closed = await client.wait_status(
            lambda s: bool(s.tool_status is not None and s.tool_status.positions),
            timeout=STEP_BUDGET_S,
        )
        assert closed, "the gripper never reported a position after a close"
        status = await client.status()
        assert status is not None, "the STATUS query went unanswered"
        after_close = status.tool_status
        assert after_close is not None
        assert after_close.positions[0] > 0.9, (
            f"a full close must report jaws closed: {after_close.positions}"
        )

        # Halt an open in flight: the jaws must hold near where the stop
        # caught them, nowhere near the abandoned fully-open target. The
        # interrupted travel starts from a mid-stroke hold so the stop
        # always reads a trustable jaw byte, not the 255 edge.
        await client.tool_action(tool, "move", [0.7, 0.5, 400.0], wait=True)
        await client.tool_action(tool, "move", [0.0, 0.15, 400.0])
        await client.tool_action(tool, "stop", wait=True)
        status = await client.status()
        assert status is not None and status.tool_status is not None
        held = status.tool_status.positions[0]
        assert held > 0.5, (
            f"the stop did not hold the jaws: they carried on to {held} "
            f"after the open was abandoned"
        )

        # Release, then a fresh move must still stream — the idle
        # handshake ends in watchdog polls, not a dead send slot.
        await client.tool_action(tool, "idle", wait=True)
        await client.tool_action(tool, "move", [0.0, 0.5, 400.0], wait=True)
        opened = await client.wait_status(
            lambda s: bool(
                s.tool_status is not None
                and s.tool_status.positions
                and s.tool_status.positions[0] < 0.05
            ),
            timeout=STEP_BUDGET_S,
        )
        assert opened, (
            f"opening the gripper after a release did not move the jaws "
            f"back: closed at {after_close.positions}"
        )

        # WC sizes its I/O chips from these two counts, so the published
        # array has to be exactly as long as they say.
        robot = Robot(host="127.0.0.1", port=daemon.command_port, timeout=STEP_BUDGET_S)
        n_in = robot.digital_inputs
        n_out = robot.digital_outputs
        assert n_out >= 2, "the shipped box declares more than one output"

        status = await client.status()
        assert status is not None
        assert len(status.io) == n_in + n_out + 1, (
            f"io carries every declared line plus the e-stop: {list(status.io)}"
        )
        assert status.io[-1] == 1, "the e-stop slot is last and reads clear"

        # Drive the LAST output, so a run that ignored `port` and wrote
        # slot 0 would land somewhere this assertion can see.
        await client.write_io(n_out - 1, 1)
        driven = await client.wait_status(
            lambda s: bool(s.io[n_in + n_out - 1] == 1), timeout=STEP_BUDGET_S
        )
        assert driven, "the level never reached the published io array"
        status = await client.status()
        assert status is not None
        assert list(status.io[n_in : n_in + n_out]) == [0] * (n_out - 1) + [1], (
            f"only the addressed output moved: {list(status.io)}"
        )

        await client.write_io(n_out - 1, 0)
        cleared = await client.wait_status(
            lambda s: bool(s.io[n_in + n_out - 1] == 0), timeout=STEP_BUDGET_S
        )
        assert cleared, "the output never went back low"

        # The wire will carry the port; the box will not — sent through the
        # engine client directly, past the shim's own bound check, so the
        # refusal under test is the RUNTIME's.
        core = client._core
        assert core is not None
        with pytest.raises(RobotError) as io:
            await client._call(core.write_io(n_out, 1))
        assert io.value.code == ErrorCode.COMM_VALIDATION_ERROR
        assert "does not exist" in io.value.cause, io.value.cause


@pytest.mark.timeout(120)
async def test_status_arrives_on_the_shipped_transport_defaults(tmp_path):
    """The defaults par6 actually ships, delivering real STATUS frames.

    Every other rig here pins unicast, so the `auto` ladder — probe the
    multicast group, keep it when the probe is delivered, fall back to
    unicast when it is not — and multicast DELIVERY itself have never been
    exercised end to end. A deployed box runs `auto`.

    The probe has to CLEAR here, not fall back: a fallback would still
    deliver frames, so the delivery assertions below would pass while
    covering the unicast path this test exists to leave. The send socket
    setting `IP_MULTICAST_IF` from the configured interface is what makes
    the probe reach a receiver joined on loopback.
    """
    daemon = LiveDaemon.start(tmp_path, status_transport="auto")
    try:
        assert "fall back to unicast" not in daemon.log_path.read_text(), (
            "the multicast probe failed, so this ran on the unicast leg:\n"
            f"{daemon.log_path.read_text()}"
        )
        async with daemon.client(shipped_transport=True) as client:
            assert await client.wait_status(
                lambda s: s.link_ok == 1, timeout=STEP_BUDGET_S
            ), (
                "no STATUS reached a client on the shipped transport defaults; "
                f"daemon log:\n{daemon.log_path.read_text()}"
            )
            frames = []
            async with asyncio.timeout(STEP_BUDGET_S):
                async for status in client.stream_status():
                    frames.append(status)
                    if len(frames) == 5:
                        break
            assert all(b.seq > a.seq for a, b in zip(frames, frames[1:])), (
                "the shipped transport delivered frames out of order or repeated"
            )
            assert client.status_seq_gaps == 0

            # The command plane is on its own socket; prove the whole
            # session works on the defaults, not just the broadcast.
            assert await client.ping() is not None
            assert await client.angles() is not None
    finally:
        daemon.stop()


async def test_status_carries_drive_and_loop_health(tmp_path):
    """A display should not have to poll a query to say whether the arm is
    well: the drives' analog trends and the loop's tail ride the STATUS
    broadcast every subscriber already gets.

    The loop's numbers are checked against the authority that owns them —
    the LOOP_STATS query — so this fails if STATUS ever carries an invented
    or stale copy rather than the live snapshot."""
    daemon = LiveDaemon.start(tmp_path)
    try:
        async with daemon.client() as client:
            assert await client.wait_ready(timeout=STEP_BUDGET_S)

            seen: dict = {}

            def capture(s) -> bool:
                temps = list(s.drive_health.get("temperatures_c") or [])
                if not temps or not all(math.isfinite(t) for t in temps):
                    return False
                seen["drives"] = dict(s.drive_health)
                return True

            assert await client.wait_status(capture, timeout=STEP_BUDGET_S), (
                f"no drive readings on STATUS; daemon log:\n"
                f"{daemon.log_path.read_text()}"
            )
            drives = seen["drives"]
            temps = list(drives["temperatures_c"])
            assert all(t > 0.0 for t in temps)
            assert len(drives["currents_ma"]) == len(temps)
            assert drives["bus_voltage_v"] > 0.0

            # Loop health against LOOP_STATS. The percentile needs a full
            # sampling window before it means anything, so wait for it
            # rather than reading whichever frame arrived first.
            loop: dict = {}

            def loop_ready(s) -> bool:
                loop.update(dict(s.loop_health))
                return loop["p99_period_s"] > 0.0

            assert await client.wait_status(loop_ready, timeout=STEP_BUDGET_S), (
                "STATUS never reported a loop percentile"
            )
            stats = await client.loop_stats()
            assert stats is not None
            assert abs(loop["p99_period_s"] - stats.p99_period_s) < 5e-3
            before = loop["overruns"]
            later: dict = {}
            assert await client.wait_status(
                lambda s: bool(later.update(dict(s.loop_health))) or True,
                timeout=STEP_BUDGET_S,
            )
            assert later["overruns"] >= before
    finally:
        daemon.stop()


async def test_config_info_reports_the_effective_configuration(tmp_path):
    """CONFIG_INFO answers the runtime's effective configuration, and its
    fingerprint is reproducible over the same files — the skew check a UI
    runs against its packaged config mirror."""
    import hashlib
    import tomllib

    daemon = LiveDaemon.start(tmp_path)
    try:
        async with daemon.client() as client:
            assert await client.wait_ready(timeout=STEP_BUDGET_S)
            info = await client.config_info()
            assert info is not None

            # Every [motion] key rides along, the sampling pitches
            # included, and an omitted optional key reads back as None.
            motion = info["motion"]
            assert motion["path_step_m"] == pytest.approx(0.002)
            assert motion["joint_step_rad"] is None
            assert motion["path_rot_weight_m_per_rad"] == pytest.approx(0.15)
        assert info["path"] == str(daemon.config)

        # The wire-contract fingerprint: sha256 over the robot TOML and
        # each grippers/*.toml (sorted), each as `name\n` + content.
        h = hashlib.sha256()
        for f in [daemon.config] + sorted(
            (daemon.config.parent / "grippers").glob("*.toml")
        ):
            h.update(f.name.encode())
            h.update(b"\n")
            h.update(f.read_bytes())
        assert info["fingerprint"] == h.hexdigest()

        cfg = tomllib.loads(daemon.config.read_text())
        assert info["tick_dt_s"] == pytest.approx(cfg["robot"]["tick_dt_s"])
        assert info["motion"]["jog_l_linear_max_m_s"] == pytest.approx(
            cfg["motion"]["jog_l_linear_max_m_s"]
        )
        assert info["motion"]["settle_timeout_s"] == pytest.approx(
            cfg["motion"]["settle_timeout_s"]
        )
        joints = info["joints"]
        assert len(joints) == len(cfg["joints"])
        for got, declared in zip(joints, cfg["joints"]):
            assert got["soft_min_rad"] == pytest.approx(
                declared["limits"]["soft_min_rad"]
            )
            assert got["soft_max_rad"] == pytest.approx(
                declared["limits"]["soft_max_rad"]
            )
    finally:
        daemon.stop()


async def test_config_bundle_feeds_previews_the_daemons_numbers(tmp_path, monkeypatch):
    """CONFIG_BUNDLE serves the loaded config files verbatim, and a dry-run
    client created against a live daemon previews with those numbers.

    The test daemon runs a re-ticked copy of the packaged config
    (``tick_dt_s`` patched for CI) — a stand-in for a tuned deployment.
    A preview that read the local/packaged config would come up with the
    stock tick; one built from the daemon's bundle must report the
    daemon's."""
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))
    daemon = LiveDaemon.start(tmp_path)
    try:
        async with daemon.client() as client:
            assert await client.wait_ready(timeout=STEP_BUDGET_S)
            info = await client.config_info()
            bundle = await client.config_bundle()
        assert bundle is not None and info is not None

        # Verbatim file service: content and inventory match the daemon's
        # config dir, and the fingerprint is CONFIG_INFO's.
        assert bundle["robot_filename"] == daemon.config.name
        assert bundle["robot_toml"] == daemon.config.read_text()
        served = {g["filename"]: g["content"] for g in bundle["grippers"]}
        on_disk = sorted((daemon.config.parent / "grippers").glob("*.toml"))
        assert sorted(served) == [f.name for f in on_disk]
        assert all(served[f.name] == f.read_text() for f in on_disk)
        assert bundle["fingerprint"] == info["fingerprint"]

        materialized = _cfg.materialize_bundle(bundle)
        assert materialized.read_text() == daemon.config.read_text()
        # Same fingerprint → same directory: re-materializing is a no-op.
        assert _cfg.materialize_bundle(bundle) == materialized
        assert str(tmp_path / "cache") in str(materialized)

        # The factory fetches the bundle itself and the preview engine
        # runs the daemon's tick, not the stock 0.004 the local config
        # (PAR6_CONFIG / repo checkout) carries.
        robot = Robot(host="127.0.0.1", port=daemon.command_port)
        dr = robot.create_dry_run_client()
        assert dr._preview.tick_dt_s() == pytest.approx(TICK_DT_S)
        assert dr._preview.tick_dt_s() != pytest.approx(0.004)
    finally:
        daemon.stop()


def test_check_config_validates_the_bundle_and_exits(tmp_path):
    """``par6d --check-config`` is the deploy-time gate: exit 0 on a valid
    bundle, exit 1 (with the validation error on stderr) on a broken one —
    without binding a socket or touching a bus."""
    import subprocess

    from live_daemon import par6d_binary

    binary = par6d_binary()
    assert binary is not None
    config = sim_config(tmp_path / "config")
    ok = subprocess.run(
        [binary, "--check-config", "--config", str(config)],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert ok.returncode == 0, ok.stderr
    assert "config OK" in ok.stdout

    broken = tmp_path / "config" / "broken.toml"
    broken.write_text(config.read_text().replace("tick_dt_s = ", "tick_dt_s = -"))
    bad = subprocess.run(
        [binary, "--check-config", "--config", str(broken)],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert bad.returncode == 1
    assert "tick_dt_s" in bad.stderr


async def test_set_payload_round_trips_and_refuses_garbage(daemon: LiveDaemon):
    """SET_PAYLOAD reaches the runtime and PAYLOAD reads back exactly what
    was set; a negative mass and a negative-definite inertia are refused
    with the structured validation error (the client encodes with the
    same table the runtime decodes with, so the refusal is identical
    wherever it fires); clearing is mass 0."""
    async with daemon.client() as client:
        assert await client.wait_ready(timeout=STEP_BUDGET_S)

        info = await client.payload()
        assert info is not None
        assert info.mass == 0.0
        assert info.com == (0.0, 0.0, 0.0)
        assert info.inertia == (0.0,) * 6

        assert await client.set_payload(1.2, com=(0.0, 0.01, 0.05)) == 1
        info = await client.payload()
        assert info is not None
        assert info.mass == pytest.approx(1.2)
        assert info.com == pytest.approx((0.0, 0.01, 0.05))
        assert info.inertia == (0.0,) * 6

        with pytest.raises(RobotError) as neg:
            await client.set_payload(-1.0)
        assert neg.value.code == ErrorCode.COMM_VALIDATION_ERROR

        with pytest.raises(RobotError) as indef:
            await client.set_payload(1.0, inertia=(-1.0, 0.0, 1.0, 0.0, 0.0, 1.0))
        assert indef.value.code == ErrorCode.COMM_VALIDATION_ERROR

        # Indefinite despite non-negative LEADING minors (zero diagonal
        # entry, coupling hidden in a trailing minor): eigenvalue ≈ −4.5.
        # Only checking all principal minors catches it.
        with pytest.raises(RobotError) as hidden:
            await client.set_payload(1.0, inertia=(0.0, 0.0, 1.0, 0.0, 5.0, 0.0))
        assert hidden.value.code == ErrorCode.COMM_VALIDATION_ERROR
        info = await client.payload()
        assert info is not None and info.mass == pytest.approx(1.2), (
            "a refused set must not change the payload"
        )

        assert await client.set_payload(0.0) == 1
        info = await client.payload()
        assert info is not None and info.mass == 0.0


async def test_flashing_and_drive_retune_over_the_python_client(daemon: LiveDaemon):
    """The maintenance surface end to end: SET_PID_GAINS re-pushes a
    configured drive's tuning and refuses a node the config does not
    declare; the FLASHING window requires the operator's spelled-out
    assertion, opens from IDLE, and its exit invalidates homing — the
    runtime cannot tell a flash from a scan, so every window costs a
    re-home."""
    async with daemon.client() as client:
        assert await client.wait_ready(timeout=STEP_BUDGET_S)

        # Re-push a joint's own configured tuning: semantically a no-op,
        # but the ack still proves the node check and the RT push ran.
        j = _cfg.config().joints()[2]
        tune = dict(
            kpp=j["gains"]["kpp"],
            kpv=j["gains"]["kpv"],
            kiv=j["gains"]["kiv"],
            kpiq=j["gains"]["kpiq"],
            kiiq=j["gains"]["kiiq"],
            kp=j["gains"]["kp"],
            kd=j["gains"]["kd"],
            ilim_ma=j["ilim_ma"],
            velocity_limit_ticks_s=j["velocity_limit_ticks_s"],
            voltage_limit_mv=j["voltage_limit_mv"],
        )
        assert await client.set_pid_gains(j["node_id"], **tune) == 1
        with pytest.raises(RobotError) as bad:
            await client.set_pid_gains(15, **tune)
        assert bad.value.code == ErrorCode.COMM_VALIDATION_ERROR
        assert "15" in str(bad.value)

        # The assertion is an argument with no default — a typo dies
        # locally, before any datagram.
        with pytest.raises(ValueError):
            await client.enter_flashing("definitely")
        # No window open: the exit is refused by the runtime.
        with pytest.raises(RobotError) as noexit:
            await client.exit_flashing()
        assert noexit.value.code == ErrorCode.COMM_VALIDATION_ERROR

        await client.reset()
        await teleport_to(client, park_deg())
        assert await client.enter_flashing("parked") == 1
        assert await client.exit_flashing() == 1
        assert await client.wait_status(lambda s: not s.homed, timeout=STEP_BUDGET_S), (
            "a closed window must read un-homed until the operator re-homes"
        )


def test_the_cli_speaks_refusals_and_never_fakes_a_stop(daemon: LiveDaemon, capsys):
    """The ``par6`` shell exits with its documented codes for the outcomes
    that matter from a terminal in a hurry: a runtime REFUSAL is a spoken
    ``EXIT_REFUSED`` (not a traceback — refusals raise RobotError, which
    main must catch), a wait that runs out is ``EXIT_TIMEOUT`` rather than
    "unreachable", an estop/stop/reset nothing acknowledged exits
    ``EXIT_UNREACHABLE`` instead of printing success on a lost datagram,
    and ``status`` reports the e-stop with the line's real polarity."""
    import socket

    from par6.cli import EXIT_REFUSED, EXIT_TIMEOUT, EXIT_UNREACHABLE, main

    addr = ["--host", "127.0.0.1", "--port", str(daemon.command_port)]
    deadline = time.monotonic() + STEP_BUDGET_S
    while main([*addr, "ping"]) != 0:
        assert time.monotonic() < deadline, "the daemon never answered ping"
    capsys.readouterr()

    # The boot arm is un-referenced (and possibly still DISABLED): a
    # move is refused either way, and the shell speaks the refusal.
    assert main([*addr, "move-j", "0", "0", "0", "0", "0", "0"]) == EXIT_REFUSED

    # The e-stop line reads clear on a fresh boot and engaged after an
    # estop: `status` must say so in those words, not the inverse.
    assert main([*addr, "--json", "status"]) == 0
    assert json.loads(capsys.readouterr().out)["estop"] is False
    assert main([*addr, "estop"]) == 0
    capsys.readouterr()
    assert main([*addr, "--json", "status"]) == 0
    assert json.loads(capsys.readouterr().out)["estop"] is True
    assert main([*addr, "reset"]) == 0
    capsys.readouterr()

    # A homing seek takes a minute; a wait shorter than that is a timeout
    # the shell names, not a runtime it could not reach.
    assert main([*addr, "home", "--wait", "--home-timeout", "0.5"]) == EXIT_TIMEOUT
    assert "timed out" in capsys.readouterr().err
    assert main([*addr, "stop"]) == 0
    capsys.readouterr()

    # A port nothing listens on: system commands come back unconfirmed,
    # and "estop latched" printed there would be the dangerous lie.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        dead_port = s.getsockname()[1]
    dead = ["--host", "127.0.0.1", "--port", str(dead_port), "--timeout", "0.5"]
    for verb in ("estop", "stop", "reset"):
        assert main([*dead, verb]) == EXIT_UNREACHABLE, verb


@pytest.mark.timeout(240)
async def test_estimate_payload_runs_from_a_program_and_only_declares_what_it_found(
    daemon: LiveDaemon,
):
    """A pick routine can ask what it just picked up, in-process.

    This is the shape the operation has to have: a client call between
    closing the gripper and moving the part, not something run from a
    terminal. What is asserted here is that contract — the wrist swings,
    a well-formed answer comes back, and the runtime's payload changes
    only when the answer is declared and only to what was found. A
    payload declared BEFORE the call has to come back on every exit that
    does not declare: the estimate clears it to measure against an
    unloaded model, and an arm still holding a 1.2 kg part must not be
    left compensating for nothing because someone was curious.

    Whether the number is RIGHT is not asserted here and cannot be: this
    fixture re-ticks the daemon for CI, and at that rate the torque plant
    limit-cycles, so reported current is chatter rather than gravity.
    That measurement lives against the shipped tick in
    `par6d/tests/gravity_calibration.rs`.
    """
    async with daemon.client() as client:
        assert await client.wait_ready(timeout=STEP_BUDGET_S)

        # A teleport references the arm (the sim is born at its endstops),
        # which is all this test needs from HOME — and it costs a second
        # against the shipped sequence's ~60 s of wall clock. The seek
        # itself is covered live once, by
        # `test_homing_sequence_drives_the_sim_to_the_configured_ready_pose`.
        # The posture matters: `plan_poses` needs clear wrist poses to swing
        # through, which park does not give it.
        await settle_at(client, TILTED_POSTURE_DEG)

        assert await client.set_payload(1.2, com=(0.0, 0.01, 0.05)) == 1
        before = await client.payload()
        assert before is not None and before.mass == pytest.approx(1.2)

        found = await client.estimate_payload(declare=False)
        assert found.poses >= 3, "the wrist must have been swung somewhere"
        assert math.isfinite(found.mass)
        assert len(found.com) == 3 and all(math.isfinite(v) for v in found.com)
        assert len(found.determined) == 4
        assert all(0.0 <= d <= 1.0 for d in found.determined), found.determined
        assert found.rms_nm <= found.rms_unloaded_nm, (
            "estimating a load cannot explain the torque worse than ignoring it: "
            f"{found.rms_nm} vs {found.rms_unloaded_nm} Nm"
        )

        # Asking is not declaring: what was declared is what is carried.
        carried = await client.payload()
        assert carried is not None
        assert carried.mass == pytest.approx(before.mass)
        assert carried.com == pytest.approx(before.com)

        # Declaring puts exactly what was found on the arm — or refuses,
        # when the poses did not measure a mass, and then the earlier
        # declaration still stands. Either way the runtime and the
        # answer agree.
        try:
            declared = await client.estimate_payload(declare=True)
        except RuntimeError as refused:
            assert "did not measure the mass" in str(refused) or "refusing" in str(
                refused
            ), refused
            carried = await client.payload()
            assert carried is not None and carried.mass == pytest.approx(before.mass), (
                "a refused estimate must leave the earlier declaration in place"
            )
        else:
            carried = await client.payload()
            assert carried is not None
            assert carried.mass == pytest.approx(declared.mass, rel=1e-6)
            assert carried.com == pytest.approx(declared.com, rel=1e-6)

        assert await client.set_payload(0.0) == 1
