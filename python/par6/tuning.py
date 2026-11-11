"""What every front end writes when it tunes an arm: the drive tuning
tuple, and the contact thresholds as a local overlay carries them.

Kept out of :mod:`par6.gui`, which imports NiceGUI: the CLI needs the same
things and must not drag a web framework in to get them.
"""

from __future__ import annotations

import os
import tempfile
from collections.abc import Sequence
from pathlib import Path

from par6 import _par6

#: The ten values one ``set_pid_gains`` frame replaces, with a label and a
#: unit. The frame carries the whole tuple, so a partial write would zero
#: what it left out — which is why this is one list and one Apply, not ten
#: independent fields.
GAIN_FIELDS: tuple[tuple[str, str, str], ...] = (
    ("kpp", "Position P", ""),
    ("kpv", "Velocity P", ""),
    ("kiv", "Velocity I", ""),
    ("kpiq", "Current P", ""),
    ("kiiq", "Current I", ""),
    ("kp", "Impedance stiffness", ""),
    ("kd", "Impedance damping", ""),
    ("ilim_ma", "Current limit", "mA"),
    ("velocity_limit_ticks_s", "Velocity limit", "ticks/s"),
    ("voltage_limit_mv", "Voltage limit (0 = VBUS)", "mV"),
)


def contact_overlay(
    reaction: str, sustained_nm: Sequence[float], impact_nm: Sequence[float]
) -> str:
    """The ``[contact]`` table a local overlay takes for these values."""
    return _par6.overlay_with_contact(
        "",
        str(reaction),
        [float(v) for v in sustained_nm],
        [float(v) for v in impact_nm],
    )


def save_contact_overlay(
    path: str | os.PathLike[str],
    reaction: str,
    sustained_nm: Sequence[float],
    impact_nm: Sequence[float],
) -> Path:
    """Put these values in the ``[contact]`` table of the overlay at *path*.

    Everything else in the file stays as it was written, comments
    included; a file that is not there yet is created.  The runtime reads
    its overlay at start, so what is saved here is what the arm runs on
    from its next restart — until then it runs on whatever was applied
    live.  Raises ``ValueError`` for a file that is not TOML.
    """
    target = Path(path)
    current = target.read_text(encoding="utf-8") if target.exists() else ""
    updated = _par6.overlay_with_contact(
        current,
        str(reaction),
        [float(v) for v in sustained_nm],
        [float(v) for v in impact_nm],
    )
    # Written beside the file and moved over it, so a runtime starting
    # mid-save reads the old overlay or the new one, never half of either.
    handle, scratch = tempfile.mkstemp(
        dir=target.parent, prefix=f".{target.name}.", suffix=".tmp"
    )
    try:
        with os.fdopen(handle, "w", encoding="utf-8") as out:
            out.write(updated)
        os.replace(scratch, target)
    except BaseException:
        Path(scratch).unlink(missing_ok=True)
        raise
    return target
