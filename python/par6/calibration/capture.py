"""Reader for the native PAR6CAP2 recording (250 Hz RT snapshots on disk).

Every measurement in this package comes from this file, never from the
50 Hz STATUS stream: Python scheduling jitter can delay a status packet,
it cannot change what the RT thread wrote. Partial trailing records are
ignored.
"""

from __future__ import annotations

import struct
import time
from pathlib import Path

import numpy as np

FIELDS = (
    "q",
    "qd",
    "tau",
    "q_commanded",
    "qd_commanded",
    "tau_commanded",
    "q_target",
    "gravity_nm",
    "current_ma",
    "kt_nm_a",
    "ilim_ma",
)
DTYPE = np.dtype(
    [
        ("tick", "<u8"),
        ("elapsed_ns", "<u8"),
        ("flags", "<u8"),
        ("values", "<f8", (len(FIELDS), 6)),
    ]
)
HEADER_SIZE = 96
#: The disk writer trails the RT thread by its flush cadence; on a loaded host
#: a couple of seconds still means "recording", ten minutes into a run.
WRITER_LAG_S = 2.0

#: `flags` bits written by par6d's recorder (see crates/par6d/src/diagnostics.rs).
FLAG_FAULT = 1
FLAG_HOMED = 2
FLAG_ENABLED = 4
FLAG_STALE = 8
FLAG_GRAVITY = 16
MODE_SHIFT = 8
MODE_IDLE = 1
MODE_HOMING = 3
MODE_JOG = 4
MODE_STREAM = 5
MODE_EXEC = 6
MOTION_MODES = (MODE_JOG, MODE_STREAM, MODE_EXEC)


def mode(row) -> int:
    return row["flags"] >> MODE_SHIFT


def metadata(path: Path) -> dict:
    with Path(path).open("rb") as f:
        header = f.read(HEADER_SIZE)
    if len(header) != HEADER_SIZE or header[:8] != b"PAR6CAP2":
        raise ValueError("Not a PAR6CAP2 recording")
    return {
        "dt": struct.unpack("<d", header[8:16])[0],
        "fingerprint": header[16:80].decode("ascii"),
        "pid": struct.unpack("<Q", header[80:88])[0],
        "started": struct.unpack("<d", header[88:96])[0],
    }


def length(path: Path) -> int:
    return max(0, (Path(path).stat().st_size - HEADER_SIZE) // DTYPE.itemsize)


def read_capture(path: Path, start: int = 0, end: int | None = None):
    """Rows `start..end` as dicts; returns `(dt, rows)`."""
    path = Path(path)
    info = metadata(path)
    with path.open("rb") as f:
        f.seek(HEADER_SIZE + start * DTYPE.itemsize)
        data = f.read() if end is None else f.read(max(0, end - start) * DTYPE.itemsize)
    array = np.frombuffer(
        data[: len(data) // DTYPE.itemsize * DTYPE.itemsize], dtype=DTYPE
    )
    columns = {name: array["values"][:, i, :].tolist() for i, name in enumerate(FIELDS)}
    rows = [
        {
            "tick": tick,
            "elapsed_ns": elapsed,
            "flags": flags,
            **{name: columns[name][i] for name in FIELDS},
        }
        for i, (tick, elapsed, flags) in enumerate(
            zip(
                array["tick"].tolist(),
                array["elapsed_ns"].tolist(),
                array["flags"].tolist(),
            )
        )
    ]
    return info["dt"], rows


def active_rows(rows, modes=MOTION_MODES):
    """The rows between the first and last motion-mode tick, validated."""
    if any(r["flags"] & FLAG_FAULT for r in rows):
        raise ValueError("Controller fault during the recorded trial or stop")
    active = [i for i, r in enumerate(rows) if mode(r) in modes]
    if not active:
        raise ValueError("Capture contains no controlled motion")
    result = rows[active[0] : active[-1] + 1]
    if any(mode(r) not in modes for r in result):
        raise ValueError("Controller left motion mode during the trial")
    return result


def measurement_rows(rows, dt, *, simulator):
    """Derivative clock per row: the plant tick in simulation (one dt per
    native tick, catch-up ticks included), monotonic host time on hardware."""
    if not np.isfinite(dt) or not 0 < dt < 1:
        raise ValueError("Invalid measurement period")
    return [
        {
            **row,
            "sample_time_ns": round(row["tick"] * dt * 1e9)
            if simulator
            else row["elapsed_ns"],
        }
        for row in rows
    ]


def assert_live(path: Path, expected_fingerprint: str, *, expected_identity=None):
    """The file is the connected runtime's recorder and is still advancing."""
    info = metadata(path)
    if expected_identity is not None and any(
        info[key] != expected_identity.get(key)
        for key in ("pid", "started", "dt", "fingerprint")
    ):
        raise RuntimeError("Native recording belongs to another runtime instance")
    if info["fingerprint"] != expected_fingerprint:
        raise RuntimeError(
            "Capture configuration differs from the connected controller"
        )
    _, rows = read_capture(path, max(0, length(path) - 1))
    if (
        not rows
        or not -0.5
        <= time.time() - info["started"] - rows[-1]["elapsed_ns"] * 1e-9
        <= WRITER_LAG_S
    ):
        raise RuntimeError("Native recording stopped or is stale")
    return info


def reference_identity(path: Path) -> dict:
    """The recorder process and the tick its latest homing reference dates from."""
    info = metadata(path)
    count = length(path)
    if not count:
        raise ValueError("Capture has no reference evidence")
    samples = np.memmap(path, dtype=DTYPE, mode="r", offset=HEADER_SIZE, shape=(count,))
    flags = samples["flags"]
    changed = np.flatnonzero(
        ((flags & FLAG_HOMED) == 0) | ((flags >> MODE_SHIFT) == MODE_HOMING)
    )
    tick = int(samples["tick"][changed[-1]]) if len(changed) else None
    return {"pid": info["pid"], "started": info["started"], "reference_tick": tick}
