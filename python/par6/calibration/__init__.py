"""Measured arm calibration: check, tune-feedback, gravity, limits.

Every routine runs against the connected par6d with its native recorder on,
judges the recording after an acknowledged Stop, and stages a candidate
config (plus rollback) that Commander activates with PAR6_CONFIG. Nothing is
applied automatically.
"""

from .report import Patch, verify_applied, write_profile
from .routines import check, gravity, limits, tune_feedback
from .session import Session, TrialRejected

__all__ = [
    "Patch",
    "Session",
    "TrialRejected",
    "check",
    "gravity",
    "limits",
    "tune_feedback",
    "verify_applied",
    "write_profile",
]
