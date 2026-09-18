"""The backend a client drives, shared by the live and dry-run clients."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from par6.robot import Robot


class RobotOwner:
    """The backend this client drives or stands in for, built on first read
    when the host constructed the client bare (a user script, or a worker
    running previews)."""

    _robot: Robot | None = None

    @property
    def robot(self) -> Robot:
        if self._robot is None:
            from par6.robot import Robot

            self._robot = Robot()
        return self._robot

    @robot.setter
    def robot(self, value: Robot | None) -> None:
        self._robot = value
