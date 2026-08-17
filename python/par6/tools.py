"""Tool specs built from the packaged gripper TOMLs.

Separate from :mod:`par6.robot` so a bare :class:`par6.RobotClient` can bind
the same tools the ``Robot`` factory does without pulling in pinokin: the
specs are pure config (TCP transform, stroke, current ceiling) plus the
action verbs the runtime's ``TOOL_ACTION`` accepts.

Geometry is deliberately absent.  par6 ships one URDF tree per fitted
end-effector (:func:`par6.config.urdf_path`), and in the vendor CAD the
gripper body is fused into the arm's final link mesh — there is no separable
tool mesh to hand a consumer as ``ToolSpec.meshes``, and a jaw-only mesh set
would render jaws with no body.  The tool geometry reaches a 3-D view through
the URDF tree instead.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from waldoctl import (
    ChannelDescriptor,
    ElectricGripperTool,
    LinearMotion,
    MeshRole,
    ToolsCollection,
    ToolSpec,
    ToolStatus,
    ToolType,
)

from par6 import config as _cfg


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


class PassiveTool(_ClientBound, ToolSpec):
    """Passive tool (bare flange): TCP + visuals only, no actions."""


class ElectricGripper(_ClientBound, ElectricGripperTool):
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


def _describe(cfg: dict) -> str:
    """One-line tool description built from the gripper TOML's own numbers."""
    driver = cfg.get("driver")
    if driver is None:
        return "Bare flange — passive tool, TCP offset only"
    return (
        f"{driver['driver_type']} electric gripper — "
        f"{driver['stroke_mm']:g} mm stroke, {driver['ilim_ma']:g} mA current limit"
    )


def build_tools() -> ToolsCollection:
    """Typed tool specs from the packaged gripper TOMLs.

    Ordered flange first; the default is the gripper the runtime config is
    fitted with (``[robot].active_gripper``), which is the only tool
    ``SELECT_TOOL`` accepts, so a UI that renders the default before the
    first STATUS renders the tool that is actually on the arm.
    """
    grippers = _cfg.load_gripper_configs()
    fitted = _cfg.fitted_tool_key()
    if fitted not in grippers:
        raise RuntimeError(
            f"[robot].active_gripper is {fitted!r}, which has no "
            f"config/grippers TOML (have {sorted(grippers)})"
        )
    flange = _cfg.canonical_tool_key("Flange")
    tools: list[ToolSpec] = []
    for key in sorted(grippers, key=lambda k: (k != flange, k)):
        cfg = grippers[key]
        # From the tool's URDF tree, never from the TOML's DH row — see
        # :func:`par6.config.flange_to_tcp`.
        origin, rpy = _cfg.flange_to_tcp(key)
        name = str(cfg["name"])
        driver = cfg.get("driver")
        if driver is None:
            tools.append(
                PassiveTool(
                    key=key,
                    display_name=name,
                    description=_describe(cfg),
                    tcp_origin=origin,
                    tcp_rpy=rpy,
                    tool_type=ToolType.NONE,
                )
            )
            continue
        stroke_m = driver["stroke_mm"] / 1000.0
        tools.append(
            ElectricGripper(
                key=key,
                display_name=name,
                description=_describe(cfg),
                tcp_origin=origin,
                tcp_rpy=rpy,
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
            )
        )
    return ToolsCollection(tuple(tools), default_key=fitted)


__all__ = ["ElectricGripper", "PassiveTool", "build_tools"]
