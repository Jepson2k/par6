"""The drive tuning tuple every front end writes.

Kept out of :mod:`par6.gui`, which imports NiceGUI: the CLI needs the same
list and must not drag a web framework in to get it.
"""

from __future__ import annotations

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
