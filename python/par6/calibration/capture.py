"""Reader for bounded native PAR6CAP2 recordings; partial records are ignored."""

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


def metadata(path: Path):
    with path.open("rb") as f:
        header = f.read(HEADER_SIZE)
    if len(header) != HEADER_SIZE or header[:8] != b"PAR6CAP2":
        raise ValueError("Not a PAR6CAP2 recording")
    return {
        "dt": struct.unpack("<d", header[8:16])[0],
        "fingerprint": header[16:80].decode("ascii"),
        "pid": struct.unpack("<Q", header[80:88])[0],
        "started": struct.unpack("<d", header[88:96])[0],
    }


def active_rows(rows):
    """Remove idle edges, preserving and validating every tick during motion."""
    if any(r["flags"] & 1 for r in rows):
        raise ValueError("Controller fault during the recorded trial or stop")
    active = [i for i, r in enumerate(rows) if r["flags"] >> 8 in (4, 5, 6)]
    if not active:
        raise ValueError("Capture contains no controlled motion")
    result = rows[active[0] : active[-1] + 1]
    if any(r["flags"] >> 8 not in (4, 5, 6) for r in result):
        raise ValueError("Controller left motion mode during the trial")
    return result


def measurement_rows(rows, dt, *, simulator):
    """Use the plant clock for derivatives while retaining host timing evidence.

    MuJoCo advances exactly one configured dt per native tick, including host
    catch-up ticks. Physical encoder measurements retain monotonic host time.
    """
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
        <= 0.5
    ):
        raise RuntimeError("Native recording stopped or is stale")
    return info


def read_capture(path: Path, start=0, end=None):
    with path.open("rb") as f:
        info = metadata(path)
        f.seek(HEADER_SIZE + start * DTYPE.itemsize)
        data = f.read() if end is None else f.read(max(0, end - start) * DTYPE.itemsize)
    array = np.frombuffer(
        data[: len(data) // DTYPE.itemsize * DTYPE.itemsize], dtype=DTYPE
    )
    # Decode columns in bulk. Repeated NumPy scalar/field indexing in the
    # 50 Hz command task can otherwise consume an entire command interval.
    columns = {name: array["values"][:, i, :].tolist() for i, name in enumerate(FIELDS)}
    return info["dt"], [
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


def length(path: Path):
    return max(0, (path.stat().st_size - HEADER_SIZE) // DTYPE.itemsize)


def reference_identity(path: Path):
    """Bind observations to one process and its latest homing/unhomed interval."""
    info = metadata(path)
    count = length(path)
    if not count:
        raise ValueError("Capture has no reference evidence")
    samples = np.memmap(path, dtype=DTYPE, mode="r", offset=HEADER_SIZE, shape=(count,))
    flags = samples["flags"]
    changed = np.flatnonzero(((flags & 2) == 0) | ((flags >> 8) == 3))
    tick = int(samples["tick"][changed[-1]]) if len(changed) else None
    return {"pid": info["pid"], "started": info["started"], "reference_tick": tick}
