"""Unified PAR6 robot — waldoctl ``Robot`` backend for the par6d runtime.

Identity and limits come from the packaged runtime config (see
:mod:`par6.config`); FK/IK and collision checking run in the engine
(:mod:`par6._par6`) on the packaged URDF tree of the active tool, resolved
at that tree's own TCP frame — the same files, the same solver and the
same frame ``par6d`` uses, so preview and runtime cannot disagree about
where the tool is.  Clients are the protocol-v2 UDP clients from
:mod:`par6.client` with the config-built tool specs bound.
"""

from __future__ import annotations

import logging
import os
import re
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Any, Literal

import numpy as np
from numpy.typing import NDArray
from waldoctl import (
    CartesianKinodynamicLimits,
    JointsSpec,
    LinearAngularLimits,
    ToolsCollection,
    resolve_variant_tcp,
)
from waldoctl import (
    Robot as _RobotABC,
)
from waldoctl.results import IKResult

from par6 import config as _cfg
from par6._par6 import (
    COLLISION_CLEARANCE_M,
    CollisionWorld,
    Kinematics,
    compose_tool_frame,
    ping_blocking,
)
from par6.client._wire import shape_to_wire
from par6.client.async_client import AsyncRobotClient
from par6.client.dry_run_client import DryRunRobotClient
from par6.client.errors import RobotError
from par6.client.sync_client import RobotClient as SyncRobotClient
from par6.tools import build_tools

logger = logging.getLogger(__name__)

#: "Not probed yet" marker for the cached daemon-config path (a real
#: answer may legitimately be None).
_UNSET: Any = object()

_IDENTITY = (
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
)  # fmt: skip


# ===========================================================================
# Runtime reachability + lifecycle
# ===========================================================================


def _ping_runtime(host: str, port: int, timeout: float = 0.5) -> bool:
    """True when a par6d runtime answers a protocol-v2 PING at host:port."""
    return ping_blocking(host, port, timeout)


def _find_par6d() -> str:
    """Resolve the par6d binary: ``PAR6D_BIN`` env, then PATH."""
    env_bin = os.environ.get("PAR6D_BIN")
    if env_bin:
        if not os.path.isfile(env_bin):
            raise RuntimeError(f"PAR6D_BIN={env_bin!r} does not exist")
        return env_bin
    found = shutil.which("par6d")
    if found is None:
        raise RuntimeError(
            "par6d binary not found; set PAR6D_BIN or put it on PATH "
            "(build with `scripts/ffi/setup.sh && cargo build -p par6d --release`)"
        )
    return found


#: One ``env_logger`` line as ``par6d`` writes it to stderr:
#: ``[2026-08-12T18:34:22Z INFO  par6d::daemon] loaded PAR6 …``.
_LOG_LINE_RE = re.compile(
    r"^\[(?P<ts>[^\s\]]+)\s+(?P<level>TRACE|DEBUG|INFO|WARN|ERROR)\s+"
    r"(?P<target>[^\s\]]+)\]\s?(?P<message>.*)$"
)

#: Rust log levels → :mod:`logging` levels. Rust's TRACE is finer than
#: anything :mod:`logging` names, and DEBUG is the nearest level a Python
#: handler can be configured for.
_RUST_LEVELS = {
    "TRACE": logging.DEBUG,
    "DEBUG": logging.DEBUG,
    "INFO": logging.INFO,
    "WARN": logging.WARNING,
    "ERROR": logging.ERROR,
}

#: Logger every forwarded runtime line lands under, so one
#: ``logging.getLogger("par6d")`` configures the whole runtime.
_RUNTIME_LOGGER = "par6d"


def _runtime_logger(target: str) -> str:
    """The :mod:`logging` logger name for a Rust log target."""
    name = target.replace("::", ".")
    if name == _RUNTIME_LOGGER or name.startswith(f"{_RUNTIME_LOGGER}."):
        return name
    return f"{_RUNTIME_LOGGER}.{name}"


class _Par6dManager:
    """Owns an optional local ``par6d --sim`` subprocess.

    The runtime's stdout/stderr is drained by a reader thread and
    forwarded into :mod:`logging` — never inherited and never left to a
    pipe nobody reads.  par6d logs continuously, and a pipe that fills up
    (pytest capture, a GUI capturing subprocess output) blocks the
    runtime on write: it stops answering PING and looks hung.
    """

    def __init__(self, normalize_logs: bool = False) -> None:
        self._proc: subprocess.Popen | None = None
        self._reader: threading.Thread | None = None
        self._stop_reader = threading.Event()
        self.normalize_logs = normalize_logs

    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start_sim(self, host: str, port: int) -> None:
        if self.is_running():
            return
        binary = _find_par6d()
        try:
            self._proc = subprocess.Popen(
                [
                    binary,
                    "--sim",
                    "--bind",
                    host,
                    "--port",
                    str(port),
                    # Dies with this process: a runtime that outlived the
                    # program that spawned it keeps the port and the bus.
                    "--parent-pid",
                    str(os.getpid()),
                ],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except OSError as e:
            raise RuntimeError(f"failed to start {binary!r}: {e}") from e
        self._stop_reader.clear()
        self._reader = threading.Thread(
            target=self._forward_output,
            args=(self._proc,),
            name="par6d-log-reader",
            daemon=True,
        )
        self._reader.start()

    def _forward_output(self, proc: subprocess.Popen) -> None:
        """Read the runtime's output line by line into :mod:`logging`."""
        stream = proc.stdout
        if stream is None:
            return
        runtime = logging.getLogger(_RUNTIME_LOGGER)
        try:
            for raw in iter(stream.readline, ""):
                if self._stop_reader.is_set():
                    break
                line = raw.rstrip("\r\n")
                if not line:
                    continue
                match = _LOG_LINE_RE.match(line) if self.normalize_logs else None
                if match is None:
                    # A line the runtime did not format (the PAR6D_READY
                    # handshake, a panic, anything on the raw stream) —
                    # forwarded verbatim rather than dropped.
                    runtime.log(
                        logging.ERROR if _is_panic(line) else logging.INFO, line
                    )
                    continue
                logging.getLogger(_runtime_logger(match["target"])).log(
                    _RUST_LEVELS.get(match["level"], logging.INFO), match["message"]
                )
        except (OSError, ValueError) as e:
            # ValueError: the stream was closed under the reader by stop().
            runtime.debug("par6d log reader stopped: %s", e)

    def stop(self, timeout: float = 2.0) -> None:
        if self._proc is None:
            return
        if self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                logger.warning("par6d did not exit after SIGTERM; killing")
                self._proc.kill()
                try:
                    self._proc.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    logger.error("par6d did not exit after SIGKILL")
        self._stop_reader.set()
        if self._reader is not None:
            # The dead process closes the pipe, so the reader's blocking
            # readline returns on its own.
            self._reader.join(timeout=timeout)
            self._reader = None
        if self._proc.stdout is not None:
            self._proc.stdout.close()
        self._proc = None


def _is_panic(line: str) -> bool:
    """Whether an unformatted runtime line is a crash report."""
    return line.startswith("thread '") or "panicked at" in line


@dataclass
class Par6IKResult:
    """IK result — structurally satisfies the waldoctl ``IKResult`` Protocol."""

    q: NDArray[np.float64]
    success: bool
    violations: str | None = None
    iterations: int = 0
    residual: float = 0.0


# ===========================================================================
# Robot
# ===========================================================================


class Robot(_RobotABC):
    """Unified PAR6 robot — the ``waldoctl.robots``/``par6`` entry point.

    Usable as a sync or async context manager around :meth:`start` /
    :meth:`stop`.
    """

    def __init__(
        self,
        *,
        host: str | None = None,
        port: int | None = None,
        timeout: float = 10.0,
        normalize_logs: bool = False,
    ) -> None:
        self._host = host or os.environ.get("PAR6_HOST", "127.0.0.1")
        env_port = os.environ.get("PAR6_COMMAND_PORT")
        self._port = port if port is not None else int(env_port) if env_port else 6001
        self._timeout = timeout
        self._manager = _Par6dManager(normalize_logs=normalize_logs)

        self._config = _cfg.config()
        self._joints = _cfg.build_joints_spec()
        self._tools = build_tools()
        self._native_keys = {g["key"] for g in self._config.grippers()}
        self._soft = [tuple(pair) for pair in self._config.soft_limits_rad()]
        self._cartesian_limits: CartesianKinodynamicLimits | None = None

        # One solver per (URDF tree, tool frame), built on first use and
        # kept: a tool change is a display action, and re-parsing a tree
        # on every one would stall the UI.
        self._solvers: dict[tuple[str, tuple[float, ...] | None], Kinematics] = {}
        # One collision world per URDF tree, keyed the same way. A value
        # of None records a tree whose world could not be built, so the
        # failure is diagnosed once instead of every query.
        self._worlds: dict[str, CollisionWorld | None] = {}
        # Keep-outs applied locally, replayed into every world built after
        # the call so a tool change cannot silently drop the world.
        self._shapes: tuple[Any, ...] = ()
        # The daemon-config probe for previews, sampled at most once (see
        # `_daemon_config_path`). The sentinel tells "not probed yet"
        # apart from "probed; nothing answered".
        self._preview_config: Any = _UNSET
        # Bound by the set_active_tool call below, which every tool
        # change goes through — there is no "no tool selected" state.
        self._solver: Kinematics

        # The runtime is built around one fitted gripper and refuses
        # SELECT_TOOL for any other, so the fitted tool — not the bare
        # flange — is what a display should start on (and which URDF tree
        # :attr:`urdf_path` should hand a 3-D view before the first STATUS).
        self._active_tool_key = self._tools.default.key
        self.set_active_tool(self._active_tool_key)

    # -- Identity -----------------------------------------------------------

    @property
    def name(self) -> str:
        return "PAR6"

    # -- Structured sub-objects ---------------------------------------------

    @property
    def joints(self) -> JointsSpec:
        return self._joints

    @property
    def native_tools(self) -> ToolsCollection:
        """PAR6's config-built tools; ``Robot.tools`` composes plugins on top."""
        return self._tools

    @property
    def cartesian_limits(self) -> CartesianKinodynamicLimits:
        """Cartesian velocity/acceleration at 100 % ``jog_l``.

        The runtime's own ceilings: ``[motion].jog_l_*_max`` is what a
        full-scale cartesian jog commands, and the acceleration is that
        rate over the jog ramp time — the definition the jog engine
        ramps with.  Tool-independent, because the runtime enforces them
        at the TCP whatever tool is fitted.
        """
        if self._cartesian_limits is None:
            motion = self._config.motion()
            accel_time = float(self._config.jog_defaults()["accel_time_s"])
            v_lin = float(motion["jog_l_linear_max_m_s"])
            v_ang = float(motion["jog_l_angular_max_rad_s"])
            self._cartesian_limits = CartesianKinodynamicLimits(
                velocity=LinearAngularLimits(linear=v_lin, angular=v_ang),
                acceleration=LinearAngularLimits(
                    linear=v_lin / accel_time, angular=v_ang / accel_time
                ),
            )
        return self._cartesian_limits

    # -- Unit preferences ---------------------------------------------------

    @property
    def position_unit(self) -> Literal["mm", "m"]:
        return "mm"

    # -- Capability flags ---------------------------------------------------

    @property
    def digital_outputs(self) -> int:
        return len(_cfg.io_line_names()[1])

    @property
    def digital_inputs(self) -> int:
        return len(_cfg.io_line_names()[0])

    @property
    def has_force_torque(self) -> bool:
        """Joint torques are measured every tick (motor currents through
        the torque constants), and the external-torque estimate rides the
        status broadcast."""
        return True

    @property
    def has_freedrive(self) -> bool:
        """par6 back-drives through the gravity feedforward rather than a
        dedicated mode: IDLE with G(q) applied is a torque-only hold with no
        position term, so the arm floats."""
        return True

    # -- Visualization ------------------------------------------------------

    @property
    def urdf_path(self) -> str:
        """URDF of the tree matching the active tool."""
        return str(_cfg.urdf_path(self._active_tool_key))

    @property
    def mesh_dir(self) -> str:
        """The ``par6`` package directory: the packaged URDFs name their
        meshes ``package://par6/_data/URDF/<tree>/meshes/...``, which
        ``{backend_package: mesh_dir}`` resolves here."""
        return str(_cfg.package_path())

    @property
    def joint_index_mapping(self) -> tuple[int, ...]:
        return (0, 1, 2, 3, 4, 5)

    # -- Motion configuration -----------------------------------------------

    @property
    def motion_profiles(self) -> tuple[str, ...]:
        """Profile names ``par6d`` plans queued moves with.

        ``RUCKIG`` (the runtime's startup default) is jerk-limited
        point-to-point, ``TRAPEZOID`` drops the jerk limit, and ``TOPPRA``
        time-optimally parameterizes the path.
        """
        return ("RUCKIG", "TRAPEZOID", "TOPPRA")

    # -- Backend injection --------------------------------------------------

    @property
    def backend_package(self) -> str:
        return "par6"

    @property
    def sync_client_class(self) -> type:
        return SyncRobotClient

    @property
    def async_client_class(self) -> type:
        return AsyncRobotClient

    # -- Kinematics ---------------------------------------------------------

    def _solver_for(self, tool_key: str, tool_frame: list[float] | None) -> Kinematics:
        """The engine solver for *tool_key*'s URDF tree resolved past
        *tool_frame*, built once per (tree, frame)."""
        path = str(_cfg.urdf_path(tool_key))
        frame_key = (
            None if tool_frame is None else tuple(round(v, 12) for v in tool_frame)
        )
        cache_key = (path, frame_key)
        solver = self._solvers.get(cache_key)
        if solver is None:
            solver = Kinematics(
                path,
                _cfg.tcp_frame(tool_key),
                tool_transform=tool_frame,
                soft_limits=self._soft,
            )
            self._solvers[cache_key] = solver
        return solver

    def set_active_tool(
        self,
        tool_key: str,
        tcp_offset_m: tuple[float, float, float] | None = None,
        variant_key: str | None = None,
    ) -> None:
        """Point the local FK/IK model at a tool's TCP.

        A native tool's TCP is not modeled here at all: it is the ``tcp``
        link of that tool's URDF tree, so selecting one selects the tree
        the runtime is fitted with and FK resolves where ``par6d``'s does.
        Plugin tools (composed via ``waldoctl.tools``, described by no
        packaged tree) hang off the bare flange by their spec's
        ``tcp_origin``/``tcp_rpy`` with variant overrides.  ``tcp_offset_m``
        composes after either, in the tool-local frame — the same
        composition the runtime applies to ``set_tcp_offset``.  The 3-D
        view's :attr:`urdf_path` follows the same tree.
        """
        key = _cfg.canonical_tool_key(tool_key)
        origin: tuple[float, float, float] = (0.0, 0.0, 0.0)
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
        if key not in self._native_keys:
            origin, rpy = self._plugin_tool_tcp(key, variant_key)
        offset = tcp_offset_m if tcp_offset_m is not None else (0.0, 0.0, 0.0)
        frame = compose_tool_frame(
            [float(v) for v in origin],
            [float(v) for v in rpy],
            [float(v) for v in offset],
        )
        if all(abs(a - b) < 1e-15 for a, b in zip(frame, _IDENTITY)):
            frame = None
        self._solver = self._solver_for(key, frame)
        self._active_tool_key = key

    def _plugin_tool_tcp(
        self, tool_key: str, variant_key: str | None
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """TCP for a non-native tool from its ToolSpec (identity + warning
        when the key is unknown everywhere)."""
        try:
            spec = self.tools[tool_key]
        except KeyError:
            logger.warning(
                "Unknown tool %r; using identity TCP. Available: %s",
                tool_key,
                [t.key for t in self.tools.available],
            )
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        return resolve_variant_tcp(
            spec.tcp_origin, spec.tcp_rpy, spec.variants, variant_key
        )

    def _q6(self, q_rad: NDArray[np.float64]) -> list[float]:
        return [float(v) for v in np.asarray(q_rad, dtype=np.float64)[:6]]

    def fk(
        self, q_rad: NDArray[np.float64], out: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        out[:6] = self._solver.tcp(self._q6(q_rad))
        return out

    def ik(
        self, pose: NDArray[np.float64], q_seed_rad: NDArray[np.float64]
    ) -> IKResult:
        seed = np.asarray(q_seed_rad, dtype=np.float64)
        solved = self._solver.ik_pose(self._q6(seed), [float(v) for v in pose[:6]])
        return self._ik_result(solved, seed)

    def _ik_result(
        self, solved: dict | None, seed: NDArray[np.float64]
    ) -> Par6IKResult:
        n = len(self._soft)
        if solved is None:
            return Par6IKResult(
                q=seed[:n].copy(), success=False, violations=None, residual=np.inf
            )
        q = np.asarray(solved["q"], dtype=np.float64)
        violations = self._violation_text(q, solved["violations"])
        return Par6IKResult(
            q=q,
            success=violations is None,
            violations=violations,
            iterations=1,
            residual=float(solved["residual"]),
        )

    def _violation_text(self, q: NDArray[np.float64], joints: list[int]) -> str | None:
        if not joints:
            return None
        names = self._joints.names
        return "; ".join(
            f"{names[i]}: {q[i]:.4f} rad outside [{self._soft[i][0]:.4f}, "
            f"{self._soft[i][1]:.4f}]"
            for i in joints
        )

    def check_limits(self, q_rad: NDArray[np.float64]) -> bool:
        if len(q_rad) != len(self._soft):
            return False
        return not self._solver.violations(self._q6(q_rad))

    def jacobian(self, q_rad: NDArray[np.float64]) -> NDArray[np.float64]:
        """World-axes Jacobian at the active tool's TCP, ``(6, num_joints)``:
        rows ``[vx, vy, vz, wx, wy, wz]`` per unit joint rate."""
        return np.asarray(self._solver.jacobian(self._q6(q_rad)), dtype=np.float64)

    def fk_batch(self, joint_path_rad: NDArray[np.float64]) -> NDArray[np.float64]:
        rows = [self._q6(row) for row in np.asarray(joint_path_rad, dtype=np.float64)]
        return np.asarray(self._solver.fk_batch(rows), dtype=np.float64).reshape(-1, 6)

    def ik_batch(
        self,
        poses: NDArray[np.float64],
        q_start_rad: NDArray[np.float64],
    ) -> list[IKResult]:
        seed = np.asarray(q_start_rad, dtype=np.float64)
        rows = [[float(v) for v in p[:6]] for p in np.asarray(poses, dtype=np.float64)]
        results: list[IKResult] = []
        current = seed
        for solved in self._solver.ik_batch(rows, self._q6(seed)):
            result = self._ik_result(solved, current)
            results.append(result)
            if result.success:
                current = result.q
        return results

    # -- Collision ----------------------------------------------------------

    @property
    def _world(self) -> CollisionWorld | None:
        """The active tool's collision world, built on first use; ``None``
        when it could not be built.

        One world per URDF tree, on the tree the runtime is fitted with,
        so tool geometry and the SRDF's disabled pairs come from the same
        model both sides enforce, and the config's installation keep-outs
        are applied as the runtime applies them at startup.
        """
        path = str(_cfg.urdf_path(self._active_tool_key))
        if path not in self._worlds:
            self._worlds[path] = self._build_world(self._active_tool_key, path)
        return self._worlds[path]

    def _build_world(self, tool_key: str, path: str) -> CollisionWorld | None:
        try:
            world = CollisionWorld(
                path,
                str(_cfg.package_search_dir()),
                str(_cfg.srdf_path(tool_key)),
                COLLISION_CLEARANCE_M,
            )
            world.set_layer("installation", self._config.installation_shapes())
            world.set_layer("program", [shape_to_wire(s) for s in self._shapes])
        except (OSError, ValueError, RuntimeError) as e:
            logger.warning(
                "Collision checking unavailable for tool %r (%s): %s",
                tool_key,
                path,
                e,
            )
            return None
        return world

    @property
    def has_collision_checking(self) -> bool:
        return self._world is not None

    def in_collision(self, q_rad: NDArray[np.float64]) -> bool:
        w = self._world
        if w is None:
            return False
        return w.in_collision(self._q6(q_rad))

    def colliding_pairs(self, q_rad: NDArray[np.float64]) -> list[tuple[str, str]]:
        """Colliding pairs at *q_rad*, in the runtime's own reporting
        vocabulary: URDF link names for arm and tool geometry,
        ``shape:<name>`` for a keep-out applied through
        :meth:`apply_shapes`, ``install:<name>`` for a configured one."""
        w = self._world
        if w is None:
            return []
        return [tuple(pair) for pair in w.pairs(self._q6(q_rad))]

    def check_trajectory(self, q_path_rad: NDArray[np.float64]) -> int:
        w = self._world
        if w is None:
            return -1
        rows = [self._q6(row) for row in np.asarray(q_path_rad, dtype=np.float64)]
        return w.check_path(rows)

    def min_distance(self, q_rad: NDArray[np.float64]) -> float:
        w = self._world
        if w is None:
            return float("inf")
        return w.min_distance(self._q6(q_rad))

    def apply_shapes(self, shapes: list) -> None:
        """Replace this process's keep-outs — the local twin of the
        client's ``set_shapes``, which replaces the *runtime's*.

        Applied to every world already built, not just the active one, so
        a later tool change previews against the same world.
        """
        self._shapes = tuple(shapes)
        wire = [shape_to_wire(s) for s in self._shapes]
        for world in self._worlds.values():
            if world is not None:
                world.set_layer("program", wire)

    # -- Lifecycle ----------------------------------------------------------

    def start(self, **kwargs: Any) -> None:
        """Spawn ``par6d --sim`` at the target address and await it.

        Keyword args override constructor defaults: ``host``, ``port``,
        ``timeout``.  Starting claims EXCLUSIVE ownership of the target:
        a runtime already answering PING there is a hard failure, not a
        silent attach — the caller asked to own a runtime and would
        otherwise be handed one it cannot configure or stop.  To use a
        runtime somebody else started, check :meth:`is_available` and go
        straight to the client factories.

        The binary is resolved via ``PAR6D_BIN``, then PATH, and polled
        until it answers.  Raises ``RuntimeError`` when the target is
        already served, when it is remote (a local ``--sim`` cannot serve
        one), when a ``com_port`` is named, when no binary can be found,
        or when the spawned runtime dies or never becomes ready.
        """
        host: str = kwargs.get("host", self._host)
        port: int = kwargs.get("port", self._port)
        timeout: float = kwargs.get("timeout", self._timeout)
        com_port: str | None = kwargs.get("com_port")

        if com_port:
            raise RuntimeError(
                f"com_port={com_port!r} cannot be honoured: par6d reaches the "
                "arm over SocketCAN, and the interface it uses is named by "
                "[bus].interface in the robot TOML (PAR6_CONFIG), not by a "
                "serial device"
            )
        if _ping_runtime(host, port, timeout=min(timeout, 2.0)):
            raise RuntimeError(
                f"a par6d runtime is already running at {host}:{port}; "
                "start() takes exclusive ownership — use is_available() and "
                "the client factories to attach to it instead"
            )
        if host not in ("127.0.0.1", "localhost", "::1"):
            raise RuntimeError(
                f"par6d runtime not reachable at {host}:{port} "
                "(a local --sim cannot serve a remote target)"
            )
        self._manager.start_sim(host, port)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if _ping_runtime(host, port, timeout=0.2):
                return
            if not self._manager.is_running():
                self._manager.stop()
                raise RuntimeError("par6d --sim exited before answering PING")
            time.sleep(0.1)
        self._manager.stop()
        raise RuntimeError(f"par6d --sim did not become ready within {timeout:.1f}s")

    def stop(self) -> None:
        """Stop the locally-spawned par6d subprocess, if any."""
        self._manager.stop()

    def is_available(self, **kwargs: Any) -> bool:
        """Whether a runtime answers PING (short timeout)."""
        host: str = kwargs.get("host", self._host)
        port: int = kwargs.get("port", self._port)
        timeout: float = kwargs.get("timeout", 0.5)
        return _ping_runtime(host, port, timeout=timeout)

    # -- Context managers ---------------------------------------------------

    def __enter__(self) -> Robot:
        self.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self.stop()

    async def __aenter__(self) -> Robot:
        import asyncio

        await asyncio.to_thread(self.start)
        return self

    async def __aexit__(self, *exc: object) -> None:
        self.stop()

    # -- Factories ----------------------------------------------------------

    def create_async_client(self, **kwargs: Any) -> AsyncRobotClient:
        kwargs.setdefault("host", self._host)
        kwargs.setdefault("port", self._port)
        kwargs.setdefault("timeout", 5.0)
        return AsyncRobotClient(tool_specs=self.tools.available, **kwargs)

    def create_sync_client(self, **kwargs: Any) -> SyncRobotClient:
        kwargs.setdefault("host", self._host)
        kwargs.setdefault("port", self._port)
        kwargs.setdefault("timeout", 5.0)
        return SyncRobotClient(tool_specs=self.tools.available, **kwargs)

    def create_dry_run_client(self, **kwargs: Any) -> DryRunRobotClient:
        """Offline preview client — the command stream without a runtime.

        Keyword args: ``initial_joints_deg`` (defaults to home),
        ``initial_homed``, ``max_snapshot_points``, ``config_path``.

        Always a client, never ``None``: par6 supports dry running, so a
        config the engine will not load raises and says which — the ABC's
        ``None`` means the BACKEND has no preview, and a host that got it
        for a broken config would report "not supported" and hide the
        real fault.

        When a runtime answers at the target address its config is
        fetched and the preview runs the daemon's numbers; otherwise the
        preview runs on the packaged config, the same one this
        :class:`Robot` plans and checks with. The probe is sampled once
        per :class:`Robot` — the
        first creation pays the round trip (bounded by the ping timeout
        when nothing answers), later ones reuse the answer, so a host
        creating a preview per run does not block on every one.
        """
        if kwargs.get("config_path") is None:
            kwargs["config_path"] = self._daemon_config_path() or str(
                _cfg.data_root() / "config" / "PAR6.toml"
            )
        return DryRunRobotClient(**kwargs)

    def _daemon_config_path(self) -> str | None:
        """The reachable daemon's config, materialized locally (cached —
        the materialized path is fingerprint-keyed and a config change
        requires a daemon restart, so one probe per Robot is the honest
        sample rate)."""
        from concurrent.futures import ThreadPoolExecutor

        if self._preview_config is not _UNSET:
            return self._preview_config
        self._preview_config = None
        if not _ping_runtime(self._host, self._port, timeout=0.5):
            return None

        def fetch() -> dict | None:
            client = self.create_sync_client(timeout=2.0)
            try:
                return client.config_bundle()
            finally:
                client.close()

        try:
            # Own thread: the sync client refuses to run inside an event
            # loop, and this factory is called from async hosts too.
            with ThreadPoolExecutor(max_workers=1) as ex:
                bundle = ex.submit(fetch).result()
            if bundle:
                self._preview_config = str(_cfg.materialize_bundle(bundle))
        except (OSError, ValueError, KeyError, RobotError) as e:
            logger.debug("daemon config fetch failed; preview uses local config: %s", e)
        return self._preview_config


__all__ = ["Par6IKResult", "Robot"]
