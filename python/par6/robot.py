"""Unified PAR6 robot — waldoctl ``Robot`` backend for the par6d runtime.

Identity and limits come from the packaged runtime config (see
:mod:`par6.config`); FK/IK run on the packaged flange URDF via pinokin with
the end-effector applied as a tool transform from the gripper TOML
kinematics; clients are the protocol-v2 UDP clients from
:mod:`par6.client` with the config-built tool specs bound.
"""

from __future__ import annotations

import logging
import os
import random
import shutil
import socket
import subprocess
import time
import xml.etree.ElementTree as ET
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Literal

import numpy as np
from numpy.typing import NDArray
from pinokin import Damping, IKSolver, se3_from_rpy, so3_rpy
from pinokin import Robot as PinokinRobot
from waldoctl import (
    CartesianKinodynamicLimits,
    ChannelDescriptor,
    ElectricGripperTool,
    JointsSpec,
    LinearMotion,
    MeshRole,
    ToolsCollection,
    ToolSpec,
    ToolStatus,
    ToolType,
    resolve_variant_tcp,
)
from waldoctl import (
    Robot as _RobotABC,
)
from waldoctl.results import IKResult

from par6 import config as _cfg
from par6.client.async_client import AsyncRobotClient
from par6.client.sync_client import RobotClient as SyncRobotClient
from par6.protocol.constants import CmdType, MsgType
from par6.protocol.wire import ProtocolError, decode_reply, encode_command

logger = logging.getLogger(__name__)


# ===========================================================================
# Runtime reachability + lifecycle
# ===========================================================================


def _ping_runtime(host: str, port: int, timeout: float = 0.5) -> bool:
    """True when a par6d runtime answers a protocol-v2 PING at host:port."""
    req_id = random.randrange(1, 1 << 32)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(timeout)
            sock.sendto(encode_command(CmdType.PING, req_id, []), (host, port))
            data, _ = sock.recvfrom(4096)
        msg_type, reply_id, _payload = decode_reply(data)
        return msg_type is MsgType.RESPONSE and reply_id == req_id
    except (OSError, TimeoutError, ProtocolError):
        return False


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
            "(build with `cargo build -p par6d`)"
        )
    return found


class _Par6dManager:
    """Owns an optional local ``par6d --sim`` subprocess."""

    def __init__(self) -> None:
        self._proc: subprocess.Popen | None = None

    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start_sim(self) -> None:
        if self.is_running():
            return
        binary = _find_par6d()
        try:
            self._proc = subprocess.Popen([binary, "--sim"])
        except OSError as e:
            raise RuntimeError(f"failed to start {binary!r}: {e}") from e

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
        self._proc = None


# ===========================================================================
# Concrete tools (client-bound via bind_tools' _execute/_get_status hooks)
# ===========================================================================


class _ClientBound:
    """Dispatch hooks the client's ``bind_tools`` fills in on shallow copies.

    ``_execute`` maps to ``client.tool_action`` and ``_get_status`` to the
    client's tool-status query; unbound specs (as exposed on ``robot.tools``)
    raise so misuse is loud.
    """

    _execute: Callable[..., Any] | None = None
    _get_status: Callable[..., Any] | None = None
    key: str  # provided by ToolSpec in concrete subclasses

    async def _cmd(
        self, action: str, params: list[Any] | None = None, **kwargs: object
    ) -> int:
        if self._execute is None:
            raise RuntimeError("Tool not bound to a client. Access via client.tool.")
        return await self._execute(self.key, action, params or [], **kwargs)

    async def status(self) -> ToolStatus:
        if self._get_status is None:
            raise RuntimeError("Tool not bound to a client. Access via client.tool.")
        return await self._get_status()


class _PassiveTool(_ClientBound, ToolSpec):
    """Passive tool (bare flange): TCP + visuals only, no actions."""


class _ElectricGripper(_ClientBound, ElectricGripperTool):
    """Electric gripper driving the runtime's ``tool_action`` verbs.

    ``move`` takes ``[position 0..1, speed 0..1, current mA]``; ``calibrate``
    runs the driver's homing/activation sequence.
    """

    def __init__(self, **kwargs: Any) -> None:
        kwargs.setdefault("action_r_labels", ("Calibrate", "Calibrate"))
        kwargs.setdefault("action_r_icons", ("build", "build"))
        super().__init__(**kwargs)

    async def set_position(self, position: float, **kwargs: float | int) -> int:
        speed = float(kwargs.get("speed", 0.5))
        current = int(kwargs.get("current", self.current_range[1]))
        return await self._cmd("move", [float(position), speed, current])

    async def calibrate(self, **kwargs: object) -> int:
        return await self._cmd("calibrate")

    async def action_r(self, engaged: bool) -> None:
        await self.calibrate()

    async def open(self, **kwargs: float | int) -> int:
        return await self.set_position(0.0, **kwargs)

    async def close(self, **kwargs: float | int) -> int:
        return await self.set_position(1.0, **kwargs)

    @property
    def adjust_step(self) -> int:
        """Current step: ~10% of range, rounded to the nearest 10 mA."""
        lo, hi = self.current_range
        return max(10, round((hi - lo) / 10 / 10) * 10)

    @property
    def adjust_labels(self) -> tuple[str, str]:
        return ("Less current", "More current")

    @property
    def adjust_icons(self) -> tuple[str, str]:
        return ("remove", "add")

    @property
    def channel_descriptors(self) -> tuple[ChannelDescriptor, ...]:
        return (
            ChannelDescriptor(
                name="Current", unit="mA", max=float(self.current_range[1])
            ),
        )


def _build_tools(grippers: dict[str, dict]) -> ToolsCollection:
    """Typed tool specs from the gripper TOMLs (flange first, as default)."""
    tools: list[ToolSpec] = []
    for key in sorted(grippers, key=lambda k: (k != "FLANGE", k)):
        cfg = grippers[key]
        origin, rpy = _cfg.tool_tcp(cfg["kinematics"])
        common = dict(
            key=key,
            display_name=cfg["name"],
            tcp_origin=origin,
            tcp_rpy=rpy,
        )
        driver = cfg.get("driver")
        if driver is None:
            tools.append(_PassiveTool(tool_type=ToolType.NONE, **common))
            continue
        stroke_m = driver["stroke_mm"] / 1000.0
        tools.append(
            _ElectricGripper(
                position_range=(0.0, 1.0),
                speed_range=(0.0, 1.0),
                current_range=(0, int(driver["ilim_ma"])),
                motions=(
                    LinearMotion(
                        role=MeshRole.JAW,
                        axis=(0.0, 1.0, 0.0),
                        travel_m=stroke_m / 2.0,
                        symmetric=True,
                    ),
                ),
                **common,
            )
        )
    return ToolsCollection(tuple(tools), default_key="FLANGE")


# ===========================================================================
# Kinematic model
# ===========================================================================


def _load_kinematic_model(urdf_file: str, soft_rad: NDArray[np.float64], velocity: NDArray[np.float64]) -> PinokinRobot:
    """Pinokin model of the flange URDF with config limits injected.

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
    return PinokinRobot.from_urdf_string(ET.tostring(root, encoding="unicode"))


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
    ) -> None:
        self._host = host or os.environ.get("PAR6_HOST", "127.0.0.1")
        env_port = os.environ.get("PAR6_COMMAND_PORT")
        self._port = port if port is not None else int(env_port) if env_port else 6001
        self._timeout = timeout
        self._manager = _Par6dManager()

        self._config = _cfg.load_robot_config()
        self._grippers = _cfg.load_gripper_configs()
        self._joints = _cfg.build_joints_spec()
        self._tools = _build_tools(self._grippers)
        self._soft_rad = _cfg.soft_limits_rad(self._config)

        self._pinokin = _load_kinematic_model(
            str(_cfg.urdf_path("FLANGE")),
            self._soft_rad,
            self._joints.limits.hard.velocity,
        )
        self._solver = IKSolver(
            self._pinokin,
            damping=Damping.Sugihara,
            tol=1e-12,
            lm_lambda=0.0,
            max_iter=20,
            max_restarts=10,
        )

        # Pre-allocated FK/IK buffers (Eigen-compatible column-major pose).
        self._q_buf = np.zeros(self._pinokin.nq, dtype=np.float64)
        self._T_buf = np.asfortranarray(np.zeros((4, 4), dtype=np.float64))
        self._rpy_buf = np.zeros(3, dtype=np.float64)
        self._T_target_buf = np.zeros((4, 4), dtype=np.float64)

        self._active_tool_key = "FLANGE"
        self.set_active_tool("FLANGE")

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
        return _cfg.CARTESIAN_JOG_LIMITS

    # -- Unit preferences ---------------------------------------------------

    @property
    def position_unit(self) -> Literal["mm", "m"]:
        return "mm"

    # -- Capability flags ---------------------------------------------------

    @property
    def digital_outputs(self) -> int:
        """The RCB control board exposes three isolated outputs."""
        return 3

    @property
    def digital_inputs(self) -> int:
        """The RCB control board exposes three isolated inputs."""
        return 3

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
        return ("RUCKIG", "TRAPEZOID")

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

    def set_active_tool(
        self,
        tool_key: str,
        tcp_offset_m: tuple[float, float, float] | None = None,
        variant_key: str | None = None,
    ) -> None:
        """Apply a tool transform to the local FK/IK model.

        Native tools take their DH tool link from the gripper TOML; plugin
        tools (composed via ``waldoctl.tools``) fall back to their spec's
        ``tcp_origin``/``tcp_rpy`` with variant overrides.  The 3-D view's
        :attr:`urdf_path`/:attr:`mesh_dir` follow the selected tree.
        """
        key = tool_key.strip().upper()
        gripper = self._grippers.get(key)
        if gripper is not None:
            origin, rpy = _cfg.tool_tcp(gripper["kinematics"])
        else:
            origin, rpy = self._plugin_tool_tcp(key, variant_key)
        T_tool = np.zeros((4, 4), dtype=np.float64)
        se3_from_rpy(origin[0], origin[1], origin[2], rpy[0], rpy[1], rpy[2], T_tool)

        if tcp_offset_m is not None and any(v != 0 for v in tcp_offset_m):
            T_offset = np.eye(4)
            T_offset[:3, 3] = tcp_offset_m
            T_tool = T_tool @ T_offset

        if np.allclose(T_tool, np.eye(4)):
            self._pinokin.clear_tool_transform()
        else:
            self._pinokin.set_tool_transform(T_tool)
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

    # -- Lifecycle ----------------------------------------------------------

    def start(self, **kwargs: Any) -> None:
        """Attach to a reachable par6d, or spawn ``par6d --sim`` and await it.

        Keyword args override constructor defaults: ``host``, ``port``,
        ``timeout``.  A runtime already answering PING at the target is
        reused as-is; otherwise a local simulated runtime is spawned (binary
        resolved via ``PAR6D_BIN``, then PATH) and polled until it answers.
        Raises ``RuntimeError`` when the target is remote and unreachable,
        when no binary can be found, or when the spawned runtime dies or
        never becomes ready.
        """
        host: str = kwargs.get("host", self._host)
        port: int = kwargs.get("port", self._port)
        timeout: float = kwargs.get("timeout", self._timeout)

        if _ping_runtime(host, port, timeout=min(timeout, 2.0)):
            return
        if host not in ("127.0.0.1", "localhost", "::1"):
            raise RuntimeError(
                f"par6d runtime not reachable at {host}:{port} "
                "(a local --sim cannot serve a remote target)"
            )
        self._manager.start_sim()
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


__all__ = ["Par6IKResult", "Robot"]
