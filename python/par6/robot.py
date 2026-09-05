"""Unified PAR6 robot — waldoctl ``Robot`` backend for the par6d runtime.

Identity and limits come from the packaged runtime config (see
:mod:`par6.config`); FK/IK run through pinokin on the packaged URDF tree of
the active tool, resolved at that tree's own TCP frame — the same file and
the same frame ``par6d`` resolves at, so preview and runtime cannot
disagree about where the tool is; collision queries are answered by the
runtime's own collision world (:class:`par6._par6.Preview`, in-process,
installation layer and floor included); clients are the protocol-v2 UDP
clients from :mod:`par6.client` with the config-built tool specs bound.
"""

from __future__ import annotations

import logging
import math
import os
import re
import shutil
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Any, Literal

import numpy as np
from numpy.typing import NDArray
from pinokin import Damping, IKSolver, se3_from_rpy, so3_rpy
from pinokin import Robot as PinokinRobot
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
from par6._par6 import Preview, ping_blocking
from par6.client._shapes import shapes_to_wire
from par6.client.async_client import AsyncRobotClient
from par6.client.dry_run_client import DryRunRobotClient
from par6.client.errors import RobotError
from par6.client.sync_client import RobotClient as SyncRobotClient
from par6.tools import build_tools

logger = logging.getLogger(__name__)

#: "Not probed yet" marker for the cached daemon-config path (a real
#: answer may legitimately be None).
_UNSET: Any = object()


def _q6(q_rad: NDArray[np.float64]) -> list[float]:
    """The engine's six joint angles from a client array (extra entries,
    a tool DOF say, are dropped)."""
    return [float(v) for v in np.asarray(q_rad, dtype=np.float64).ravel()[:6]]


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
            "(build with `scripts/ffi/setup.sh && source .ffi/env.sh && "
            "cargo build -p par6d --release`)"
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
                [binary, "--sim", "--bind", host, "--port", str(port)],
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


# ===========================================================================
# Kinematic model
# ===========================================================================


def _load_kinematic_model(
    urdf_file: str, soft_rad: NDArray[np.float64], velocity: NDArray[np.float64]
) -> PinokinRobot:
    """Pinokin model of a packaged URDF tree, resolved at its TCP frame.

    Every gripper tree carries its TCP as a fixed link off the flange, and
    that link is the frame the runtime resolves FK/IK/Jacobian at
    (``par6_kin::GripperVariant::tcp_frame``); the bare flange tree has no
    such link and its last link is the tool point, which is what both sides
    fall back to.

    The SolidWorks export writes ``lower="0" upper="0"`` on every joint, so
    the config's soft position limits (and velocity ceilings) are patched in
    before loading — that is what makes ``IKSolver(enforce_limits=True)``
    and its in-limits restarts meaningful.
    """
    root = ET.parse(urdf_file).getroot()
    i = 0
    for joint in root.iter("joint"):
        if joint.get("type") != "revolute":
            continue
        limit = joint.find("limit")
        if limit is None:
            limit = ET.SubElement(joint, "limit")
        limit.set("lower", repr(float(soft_rad[i, 0])))
        limit.set("upper", repr(float(soft_rad[i, 1])))
        limit.set("velocity", repr(float(velocity[i])))
        i += 1
    if i != soft_rad.shape[0]:
        raise RuntimeError(
            f"URDF {urdf_file!r} has {i} revolute joints, config has {soft_rad.shape[0]}"
        )
    model = PinokinRobot.from_urdf_string(ET.tostring(root, encoding="unicode"))
    # Named, not inferred: the end of the chain is the TCP only by an
    # accident of link ordering, and a tree that gained a link after it
    # would silently move the whole cartesian surface.
    if any(link.get("name") == _cfg.TCP_LINK for link in root.iter("link")):
        model.set_ee_frame(_cfg.TCP_LINK)
    return model


def _sample_cartesian_limit(
    model: PinokinRobot,
    soft_rad: NDArray[np.float64],
    home_rad: NDArray[np.float64],
    joint_limit: NDArray[np.float64],
    samples: int,
    seed: int,
    spread_deg: float = 30.0,
) -> tuple[float, float]:
    """Median TCP rate a per-joint limit buys, as ``(linear, angular)``.

    Samples joint configurations around the park pose, and at each one solves
    the Jacobian pseudoinverse for the joint rates that move the TCP along one
    Cartesian axis alone, then takes the largest scaling of those rates that
    stays inside *joint_limit*.  Ill-conditioned configurations and directions
    that cannot be isolated (a linear axis that drags rotation with it, or the
    reverse) are dropped.  Deterministic: *seed* fixes the sample set.

    This is the derivation parol6 runs offline to produce its ``LIMITS.cart``
    constants (``parol6/PAROL6_ROBOT.py:603-676``); par6 runs it against its
    own model and config instead of carrying the numbers.
    """
    rng = np.random.default_rng(seed)
    spread_rad = math.radians(spread_deg)
    linear: list[float] = []
    angular: list[float] = []
    desired = np.zeros(6, dtype=np.float64)
    for _ in range(samples):
        q = np.clip(
            home_rad + rng.normal(0.0, spread_rad, home_rad.shape[0]),
            soft_rad[:, 0],
            soft_rad[:, 1],
        )
        J = model.jacob0(q)
        if np.linalg.cond(J) > 1e6:
            continue
        J_pinv = np.linalg.pinv(J)
        for axis in range(6):
            desired[:] = 0.0
            desired[axis] = 1.0
            q_dot = J_pinv @ desired
            coupled = J[3:, :] @ q_dot if axis < 3 else J[:3, :] @ q_dot
            if np.linalg.norm(coupled) > 0.01:
                continue
            rate = float(np.min(joint_limit / (np.abs(q_dot) + 1e-10)))
            if rate > 0.001:
                (linear if axis < 3 else angular).append(rate)
    if not linear or not angular:
        raise RuntimeError(
            "Cartesian limit sampling found no isolable axis; "
            "the kinematic model or the joint limits are degenerate"
        )
    return float(np.median(linear)), float(np.median(angular))


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

        self._config = _cfg.load_robot_config()
        self._grippers = _cfg.load_gripper_configs()
        self._joints = _cfg.build_joints_spec()
        self._tools = build_tools()
        self._soft_rad = _cfg.soft_limits_rad(self._config)
        self._cartesian_limits: CartesianKinodynamicLimits | None = None

        # One model per packaged URDF tree, built on first use and kept:
        # a tool change is a display action, and re-parsing a tree (and
        # rebuilding its solver) on every one would stall the UI.
        self._models: dict[str, PinokinRobot] = {}
        self._solvers: dict[str, IKSolver] = {}
        # The engine's own preview session per gripper, built on first use
        # and kept: it is `par6_kin::Collision` in-process — the world the
        # daemon enforces, installation layer included — so a query here
        # answers exactly as the runtime will. None records a gripper
        # whose session could not be built, diagnosed once.
        self._previews: dict[str, Preview | None] = {}
        # Program-layer keep-outs applied locally, replayed into every
        # session built after the call so a tool change cannot drop them.
        self._shapes: tuple[Any, ...] = ()
        # The daemon-config probe for previews, sampled at most once (see
        # `_daemon_config_path`). The sentinel tells "not probed yet"
        # apart from "probed; nothing answered".
        self._preview_config: Any = _UNSET
        # Both bound by the set_active_tool call below, which every tool
        # change goes through — there is no "no tool selected" state.
        self._pinokin: PinokinRobot
        self._solver: IKSolver

        # Pre-allocated FK/IK buffers (Eigen-compatible column-major pose).
        self._q_buf = np.zeros(self._joints.count, dtype=np.float64)
        self._T_buf = np.asfortranarray(np.zeros((4, 4), dtype=np.float64))
        self._rpy_buf = np.zeros(3, dtype=np.float64)
        self._T_target_buf = np.zeros((4, 4), dtype=np.float64)

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
        """Cartesian velocity/acceleration reachable at 100% jog.

        Derived on first use from the config's JOG-mode joint limits through
        this robot's own Jacobian (see :func:`_sample_cartesian_limit`), not
        stored anywhere: the runtime enforces joint-space limits only, so the
        Cartesian envelope is a consequence of them and the arm's geometry.
        """
        if self._cartesian_limits is None:
            self._cartesian_limits = self._derive_cartesian_limits()
        return self._cartesian_limits

    def _derive_cartesian_limits(self) -> CartesianKinodynamicLimits:
        jog = self._joints.limits.jog
        vel_lin, vel_ang = _sample_cartesian_limit(
            self._pinokin, self._soft_rad, self._joints.home.rad, jog.velocity, 500, 42
        )
        acc_lin, acc_ang = _sample_cartesian_limit(
            self._pinokin,
            self._soft_rad,
            self._joints.home.rad,
            _cfg.jog_ramp_acceleration(),
            200,
            43,
        )
        return CartesianKinodynamicLimits(
            velocity=LinearAngularLimits(linear=vel_lin, angular=vel_ang),
            acceleration=LinearAngularLimits(linear=acc_lin, angular=acc_ang),
        )

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
        """URDF of the tree matching the active tool (flange by default)."""
        return str(_cfg.urdf_path(self._active_tool_key))

    @property
    def mesh_dir(self) -> str:
        """Variant tree root — ``package://<tree>/meshes/...`` resolves here."""
        return str(_cfg.urdf_tree_root(self._active_tool_key))

    @property
    def joint_index_mapping(self) -> tuple[int, ...]:
        return (0, 1, 2, 3, 4, 5)

    # -- Motion configuration -----------------------------------------------

    @property
    def motion_profiles(self) -> tuple[str, ...]:
        """Profile names ``par6d`` plans queued moves with.

        ``RUCKIG`` (the runtime's startup default) is jerk-limited
        point-to-point, ``TRAPEZOID`` drops the jerk limit, and ``TOPPRA``
        time-optimally parameterizes the path.  TOPPRA runs through the
        C++ shim, so only a ``par6d`` built with its ``ffi`` feature
        registers it; on a build without one ``select_profile("TOPPRA")``
        answers ``SYS_PROFILE_INVALID`` and the active profile is
        unchanged.  The protocol has no query that enumerates the
        registry, so this list cannot narrow itself to the runtime it is
        talking to.
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

    def _load_q_buf(self, q_rad: NDArray[np.float64]) -> None:
        n = min(len(q_rad), self._pinokin.nq)
        self._q_buf[:n] = q_rad[:n]
        self._q_buf[n:] = 0.0

    def _model_for(self, tool_key: str) -> tuple[PinokinRobot, IKSolver]:
        """The model + solver for *tool_key*'s URDF tree, built once."""
        path = str(_cfg.urdf_path(tool_key))
        model = self._models.get(path)
        if model is None:
            model = _load_kinematic_model(
                path, self._soft_rad, self._joints.limits.hard.velocity
            )
            self._models[path] = model
            self._solvers[path] = IKSolver(
                model,
                damping=Damping.Sugihara,
                tol=1e-12,
                lm_lambda=0.0,
                max_iter=20,
                max_restarts=10,
            )
        return model, self._solvers[path]

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
        view's :attr:`urdf_path`/:attr:`mesh_dir` follow the same tree.
        """
        key = _cfg.canonical_tool_key(tool_key)
        model, solver = self._model_for(key)

        T_tool = np.eye(4, dtype=np.float64)
        if key not in self._grippers:
            origin, rpy = self._plugin_tool_tcp(key, variant_key)
            T_tool = np.zeros((4, 4), dtype=np.float64)
            se3_from_rpy(
                origin[0], origin[1], origin[2], rpy[0], rpy[1], rpy[2], T_tool
            )

        if tcp_offset_m is not None and any(v != 0 for v in tcp_offset_m):
            T_offset = np.eye(4)
            T_offset[:3, 3] = tcp_offset_m
            T_tool = T_tool @ T_offset

        if np.allclose(T_tool, np.eye(4)):
            model.clear_tool_transform()
        else:
            model.set_tool_transform(T_tool)
        self._pinokin = model
        self._solver = solver
        if self._q_buf.shape[0] != model.nq:
            self._q_buf = np.zeros(model.nq, dtype=np.float64)
        self._active_tool_key = key
        # The Cartesian envelope is a property of the Jacobian AT THE TCP.
        self._cartesian_limits = None

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

    def fk(
        self, q_rad: NDArray[np.float64], out: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        self._load_q_buf(q_rad)
        self._pinokin.fkine_into(self._q_buf, self._T_buf)
        so3_rpy(self._T_buf[:3, :3], self._rpy_buf)
        out[:3] = self._T_buf[:3, 3]
        out[3:6] = self._rpy_buf
        return out

    def ik(
        self, pose: NDArray[np.float64], q_seed_rad: NDArray[np.float64]
    ) -> IKResult:
        se3_from_rpy(
            pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], self._T_target_buf
        )
        return self._solve_one(self._T_target_buf, q_seed_rad)

    def _solve_one(
        self, T_target: NDArray[np.float64], q_seed_rad: NDArray[np.float64]
    ) -> Par6IKResult:
        self._load_q_buf(q_seed_rad)
        success = self._solver.solve(T_target, self._q_buf)
        q = self._solver.q.copy()
        violations = self._limit_violations(q)
        return Par6IKResult(
            q=q,
            success=bool(success) and violations is None,
            violations=violations,
            iterations=self._solver.iterations,
            residual=self._solver.residual,
        )

    def _limit_violations(self, q_rad: NDArray[np.float64]) -> str | None:
        lo, hi = self._soft_rad[:, 0], self._soft_rad[:, 1]
        bad = ~((q_rad >= lo - 1e-9) & (q_rad <= hi + 1e-9))
        if not bad.any():
            return None
        names = self._joints.names
        return "; ".join(
            f"{names[i]}: {q_rad[i]:.4f} rad outside [{lo[i]:.4f}, {hi[i]:.4f}]"
            for i in np.flatnonzero(bad)
        )

    def check_limits(self, q_rad: NDArray[np.float64]) -> bool:
        if len(q_rad) != self._soft_rad.shape[0]:
            return False
        return self._limit_violations(np.asarray(q_rad, dtype=np.float64)) is None

    def jacobian(self, q_rad: NDArray[np.float64]) -> NDArray[np.float64]:
        """World-frame Jacobian at the active tool's TCP, ``(6, num_joints)``.

        Maps joint rates to a TCP twist ``[vx, vy, vz, wx, wy, wz]``; its
        pseudo-inverse is what turns a Cartesian jog twist into joint rates,
        the same way the runtime's ``step_cart_jog`` does.
        """
        return self._pinokin.jacob0(np.asarray(q_rad, dtype=np.float64))

    def fk_batch(self, joint_path_rad: NDArray[np.float64]) -> NDArray[np.float64]:
        transforms = self._pinokin.batch_fk(
            np.ascontiguousarray(joint_path_rad, dtype=np.float64)
        )
        result = np.empty((len(transforms), 6), dtype=np.float64)
        rpy = self._rpy_buf
        for i, T in enumerate(transforms):
            result[i, :3] = T[:3, 3]
            so3_rpy(T[:3, :3], rpy)
            result[i, 3:] = rpy
        return result

    def ik_batch(
        self,
        poses: NDArray[np.float64],
        q_start_rad: NDArray[np.float64],
    ) -> list[IKResult]:
        results: list[IKResult] = []
        q_current = np.asarray(q_start_rad, dtype=np.float64).copy()
        for i in range(poses.shape[0]):
            p = poses[i]
            se3_from_rpy(p[0], p[1], p[2], p[3], p[4], p[5], self._T_target_buf)
            result = self._solve_one(self._T_target_buf, q_current)
            results.append(result)
            if result.success:
                q_current[:] = result.q
        return results

    # -- Collision ----------------------------------------------------------

    def _preview_gripper(self) -> str | None:
        """The bundle gripper the active tool is, or None for the config's
        active one (a plugin tool hangs off whatever the runtime wears)."""
        key = self._active_tool_key
        return self._grippers[key]["name"] if key in self._grippers else None

    @property
    def _preview(self) -> Preview | None:
        """The active tool's engine session, built on first use."""
        gripper = self._preview_gripper()
        slot = gripper or ""
        if slot not in self._previews:
            self._previews[slot] = self._build_preview(gripper)
        return self._previews[slot]

    def _build_preview(self, gripper: str | None) -> Preview | None:
        try:
            # The engine's own path resolution: the daemon-fetched bundle
            # when one is known, else env / repo / deploy locations.
            from par6.client.dry_run_client import _resolve_engine_paths

            config, assets = _resolve_engine_paths(self._daemon_config_path())
            preview = Preview(config=config, assets=assets, gripper=gripper)
            preview.set_shapes(shapes_to_wire(list(self._shapes)))
        except (OSError, ValueError, RuntimeError) as e:
            logger.warning("Collision checking unavailable for %r: %s", gripper, e)
            return None
        return preview

    @property
    def has_collision_checking(self) -> bool:
        return self._preview is not None

    def in_collision(self, q_rad: NDArray[np.float64]) -> bool:
        p = self._preview
        return p is not None and p.in_collision(_q6(q_rad))

    def colliding_pairs(self, q_rad: NDArray[np.float64]) -> list[tuple[str, str]]:
        """Colliding pairs at *q_rad*, in the runtime's own reporting
        vocabulary — it is the runtime's collision world answering."""
        p = self._preview
        return (
            [] if p is None else [tuple(pair) for pair in p.colliding_pairs(_q6(q_rad))]
        )

    def check_trajectory(self, q_path_rad: NDArray[np.float64]) -> int:
        p = self._preview
        if p is None:
            return -1
        path = [_q6(row) for row in np.asarray(q_path_rad, dtype=np.float64)]
        hit = p.first_collision(path)
        return -1 if hit is None else int(hit)

    def min_distance(self, q_rad: NDArray[np.float64]) -> float:
        p = self._preview
        return float("inf") if p is None else float(p.min_distance(_q6(q_rad)))

    def apply_shapes(self, shapes: list) -> None:
        """Replace this process's program-layer keep-outs — the local twin
        of the client's ``set_shapes``, which replaces the *runtime's*.
        Applied to every session already built, not just the active one,
        so a later tool change previews against the same world. The
        installation layer is the config's, applied by every session at
        boot exactly as the runtime applies it.
        """
        self._shapes = tuple(shapes)
        wire = shapes_to_wire(list(self._shapes))
        for preview in self._previews.values():
            if preview is not None:
                preview.set_shapes(wire)

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

        When a runtime answers at the target address its config is
        fetched and the preview runs the daemon's numbers; otherwise the
        local resolution (``PAR6_CONFIG``, then the repo checkout)
        applies. The probe is sampled once per :class:`Robot` — the
        first creation pays the round trip (bounded by the ping timeout
        when nothing answers), later ones reuse the answer, so a host
        creating a preview per run does not block on every one.
        """
        if kwargs.get("config_path") is None:
            kwargs["config_path"] = self._daemon_config_path()
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
