"""The runtime's structured refusal, as every backend reports it.

The class is waldoctl's — a frontend represents a refusal the same way
whichever backend raised it — and par6 raises it as-is.
"""

from waldoctl.errors import RobotError

__all__ = ["RobotError"]
