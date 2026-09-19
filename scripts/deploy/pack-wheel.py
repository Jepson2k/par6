#!/usr/bin/env python3
"""Graft the ``par6d`` runtime into a repaired par6 wheel.

`pip install par6` gives the client, the offline preview and the kinematics
engine, but `Robot().start()` spawns `par6d` — so without the binary the
simulator cannot start unless the workspace has been built (issue #33).

The binary is added AFTER `maturin`/`auditwheel` have repaired the wheel,
which means it has to be taught the same two things auditwheel taught the
extension module:

* the libraries it needs were RENAMED. auditwheel hashes each grafted
  library (`libpar6_shim.so` -> `libpar6_shim-7bc18619.so`) so two wheels in
  one environment cannot collide, and rewrites the extension's `DT_NEEDED`
  to match. A binary added afterwards still names the originals.
* they live in `par6.libs/`, which is a sibling of the `par6/` package, so
  from `par6/_bin/` the rpath is `$ORIGIN/../../par6.libs`.

Nothing else is copied: `par6d`'s only non-system dependencies are the shim
and libmujoco, and the wheel already carries both for the extension.
"""

import argparse
import hashlib
import base64
import csv
import io
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

#: Where the binary lands inside the wheel, and what
#: `par6._daemon.packaged_binary()` looks for.
BIN_IN_WHEEL = "par6/_bin/par6d"
LIBS_DIR = "par6.libs"


def hashed_names(names: list[str]) -> dict[str, str]:
    """Map each grafted library's ORIGINAL soname to auditwheel's hashed one.

    auditwheel inserts `-<8 hex>` before the first `.so`, so
    `libmujoco-9be0e91f.so.3.12.0` came from `libmujoco.so.3.12.0`.
    """
    out: dict[str, str] = {}
    for name in names:
        stem, _, rest = name.partition(".so")
        base, sep, tag = stem.rpartition("-")
        if sep and len(tag) == 8 and all(c in "0123456789abcdef" for c in tag):
            out[f"{base}.so{rest}"] = name
    return out


def record_line(path: Path, arcname: str) -> list[str]:
    data = path.read_bytes()
    digest = base64.urlsafe_b64encode(hashlib.sha256(data).digest()).rstrip(b"=")
    return [arcname, f"sha256={digest.decode()}", str(len(data))]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("wheel", type=Path, help="the repaired wheel to graft into")
    ap.add_argument("--binary", required=True, type=Path, help="par6d to ship")
    ap.add_argument("--patchelf", default="patchelf")
    args = ap.parse_args()

    if not args.binary.is_file():
        raise SystemExit(f"no par6d at {args.binary}")

    work = args.wheel.parent / f"{args.wheel.stem}.graft"
    if work.exists():
        shutil.rmtree(work)
    with zipfile.ZipFile(args.wheel) as z:
        z.extractall(work)
        names = z.namelist()

    libs = sorted(
        Path(n).name for n in names if n.startswith(f"{LIBS_DIR}/") and ".so" in n
    )
    if not libs:
        raise SystemExit(f"{args.wheel.name} has no {LIBS_DIR}/ — was it repaired?")
    renamed = hashed_names(libs)

    dest = work / BIN_IN_WHEEL
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(args.binary, dest)
    dest.chmod(0o755)

    needed = subprocess.run(
        [args.patchelf, "--print-needed", str(dest)],
        check=True, capture_output=True, text=True,
    ).stdout.split()
    for soname in needed:
        new = renamed.get(soname)
        if new:
            subprocess.run(
                [args.patchelf, "--replace-needed", soname, new, str(dest)], check=True
            )
        elif soname.startswith(("libpar6", "libmujoco", "libpinocchio", "libcoal")):
            raise SystemExit(
                f"par6d needs {soname}, which the wheel does not carry. The "
                f"binary and the extension were built against different "
                f"closures, so grafting it would ship a runtime that cannot "
                f"start."
            )
    # From par6/_bin/, the grafted libraries are two levels up.
    subprocess.run(
        [args.patchelf, "--set-rpath", f"$ORIGIN/../../{LIBS_DIR}", str(dest)],
        check=True,
    )

    # RECORD is a manifest pip verifies; a file absent from it is not
    # installed, and one with the wrong hash fails the install.
    record = next(work.glob("*.dist-info/RECORD"))
    rows = [r for r in csv.reader(io.StringIO(record.read_text())) if r]
    rows = [r for r in rows if r[0] != BIN_IN_WHEEL]
    rows.append(record_line(dest, BIN_IN_WHEEL))
    rows.sort(key=lambda r: r[0])
    with record.open("w", newline="") as fh:
        csv.writer(fh).writerows(rows)

    tmp = args.wheel.with_suffix(".whl.new")
    with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as z:
        for path in sorted(work.rglob("*")):
            if path.is_file():
                arc = str(path.relative_to(work))
                info = zipfile.ZipInfo(arc, date_time=(1980, 1, 1, 0, 0, 0))
                info.compress_type = zipfile.ZIP_DEFLATED
                # The executable bit has to survive the round trip, or the
                # daemon is installed unrunnable. The mode is only honoured
                # with the regular-file type bits beside it: `stat` packs
                # both into the same field, and a mode with no file type
                # reads as 0 and is ignored.
                mode = 0o755 if arc == BIN_IN_WHEEL else 0o644
                info.external_attr = (0o100000 | mode) << 16
                z.writestr(info, path.read_bytes())
    tmp.replace(args.wheel)
    shutil.rmtree(work)

    size = args.wheel.stat().st_size / 1048576
    print(f">>> grafted par6d into {args.wheel.name} ({size:.1f} MB)")
    for old, new in sorted(renamed.items()):
        if old in needed:
            print(f"    {old} -> {new}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
