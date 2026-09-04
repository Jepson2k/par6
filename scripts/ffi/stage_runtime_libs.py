#!/usr/bin/env python3
"""Stage a cross-built shim's runtime library closure and prove it loadable.

The PAR6 control box gets no conda environment: `par6d` links
`libpar6_shim.so`, which pulls in Pinocchio, coal, toppra and their transitive
dependencies, and all of those have to be installed alongside it. This walks
`DT_NEEDED` from the given roots, copies every dependency found in the conda
env's `lib/` into one flat directory, and then answers the only question that
can be answered without target hardware: *would the dynamic loader be able to
resolve this set on the target?*

Two checks make up that answer:

* **glibc floor** — the highest ``GLIBC_x.y`` any staged object requires. The
  box's glibc must be at least this; the value is printed so the deploy
  documentation can quote a measured number instead of a guess.
* **internal satisfiability** — every versioned symbol requirement against a
  library that *ships* (``libstdc++``, ``libgcc_s``, Pinocchio, …) must be
  provided by the copy being shipped. This is what catches a cross compiler
  newer than the target env's C++ runtime, which otherwise fails as a
  ``version GLIBCXX_3.4.x not found`` at the first start on the box.
* **rpath reachability** — the staged directory is flat and moves to
  ``/usr/local/lib/par6``, and ``DT_RUNPATH`` is *not* inherited by transitive
  dependencies, so every staged library that depends on another staged library
  must carry an ``$ORIGIN`` entry of its own. Without it the dependency
  resolves here (from an absolute build path) and not on the box.

Idempotent: a destination file that already matches the source by size and
mtime is left alone.
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
from collections import deque
from pathlib import Path

#: Sonames the target OS provides. Shipping our own copy of these would be
#: worse than useless — glibc's parts must match the running loader.
SYSTEM_SONAMES = frozenset(
    {
        "libc.so.6",
        "libm.so.6",
        "libmvec.so.1",
        "libpthread.so.0",
        "libdl.so.2",
        "librt.so.1",
        "libresolv.so.2",
        "libutil.so.1",
        "libanl.so.1",
        "libcrypt.so.1",
        "ld-linux-aarch64.so.1",
        "ld-linux-x86-64.so.2",
    }
)

_NEEDED = re.compile(r"\(NEEDED\)\s+Shared library: \[([^\]]+)\]")
_VERSION_NEED = re.compile(r"Version needs section")
_GLIBC = re.compile(r"^GLIBC_(\d+)\.(\d+)(?:\.(\d+))?$")


def _readelf(readelf: str, *args: str) -> str:
    out = subprocess.run(
        [readelf, *args], capture_output=True, text=True, check=False
    )
    if out.returncode != 0:
        raise SystemExit(f"{readelf} {' '.join(args)} failed:\n{out.stderr}")
    return out.stdout


def needed(readelf: str, path: Path) -> list[str]:
    return _NEEDED.findall(_readelf(readelf, "-d", str(path)))


def version_needs(readelf: str, path: Path) -> dict[str, set[str]]:
    """``{soname: {version, ...}}`` this object requires from its deps."""
    text = _readelf(readelf, "-V", str(path))
    wants: dict[str, set[str]] = {}
    current: str | None = None
    for line in text.splitlines():
        entry = re.search(r"File: (\S+)\s+Cnt:", line)
        if entry:
            current = entry.group(1)
            wants.setdefault(current, set())
            continue
        if current is None:
            continue
        for name in re.findall(r"Name: (\S+)\s+Flags:", line):
            wants[current].add(name)
    return wants


def version_defs(readelf: str, path: Path) -> set[str]:
    """Versions this object *provides* (its ``.gnu.version_d`` names)."""
    text = _readelf(readelf, "-V", str(path))
    provided: set[str] = set()
    in_defs = False
    for line in text.splitlines():
        if "Version definition section" in line:
            in_defs = True
            continue
        if "Version needs section" in line or "Version symbols section" in line:
            in_defs = False
            continue
        if in_defs:
            provided.update(re.findall(r"Name: (\S+)", line))
    return provided


def runpath(readelf: str, path: Path) -> list[str]:
    """``DT_RUNPATH``/``DT_RPATH`` entries, in search order."""
    text = _readelf(readelf, "-d", str(path))
    entries: list[str] = []
    for line in text.splitlines():
        if "(RUNPATH)" in line or "(RPATH)" in line:
            entries += line.split("[", 1)[1].rsplit("]", 1)[0].split(":")
    return [e for e in entries if e]


def is_origin_entry(entry: str) -> bool:
    """True when *entry* names the object's own directory.

    ``$ORIGIN``, ``$ORIGIN/`` and ``$ORIGIN/.`` all do; ``$ORIGIN/../lib``
    (which conda writes alongside it) does not, and neither does an absolute
    path. Extra entries are harmless — the loader tries them in order and
    they name nothing on the box — as long as one of them is this.
    """
    if not entry.startswith("$ORIGIN"):
        return False
    return os.path.normpath(entry[len("$ORIGIN") :].lstrip("/") or ".") == "."


def glibc_floor(readelf: str, path: Path) -> tuple[int, int, int]:
    highest = (0, 0, 0)
    for versions in version_needs(readelf, path).values():
        for name in versions:
            m = _GLIBC.match(name)
            if m:
                value = (int(m[1]), int(m[2]), int(m[3] or 0))
                highest = max(highest, value)
    return highest


def walk(readelf: str, roots: list[Path], lib_dir: Path) -> tuple[list[Path], set[str]]:
    """Transitive DT_NEEDED closure of *roots* resolved inside *lib_dir*."""
    seen: set[Path] = set()
    deps: list[Path] = []
    unresolved: set[str] = set()
    queue: deque[Path] = deque(p.resolve() for p in roots)
    root_set = set(queue)
    while queue:
        obj = queue.popleft()
        if obj in seen:
            continue
        seen.add(obj)
        if obj not in root_set:
            deps.append(obj)
        for soname in needed(readelf, obj):
            if soname in SYSTEM_SONAMES:
                continue
            candidate = lib_dir / soname
            if not candidate.exists():
                unresolved.add(soname)
                continue
            queue.append(candidate.resolve())
    return deps, unresolved


def copy_with_soname(src: Path, dest_dir: Path, soname: str) -> None:
    """Copy *src* as *soname*, skipping an already-identical destination."""
    dest = dest_dir / soname
    if dest.exists():
        s, d = src.stat(), dest.stat()
        if s.st_size == d.st_size and int(s.st_mtime) == int(d.st_mtime):
            return
    shutil.copy2(src, dest)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--readelf", required=True, help="target-arch readelf binary")
    ap.add_argument("--lib-dir", required=True, type=Path, help="conda env lib/")
    ap.add_argument("--dest", required=True, type=Path, help="staging directory")
    ap.add_argument("roots", nargs="+", type=Path, help="objects to close over")
    args = ap.parse_args()

    for root in args.roots:
        if not root.exists():
            raise SystemExit(f"root object missing: {root}")
    args.dest.mkdir(parents=True, exist_ok=True)

    deps, unresolved = walk(args.readelf, args.roots, args.lib_dir)
    if unresolved:
        raise SystemExit(
            "unresolved shared libraries (not in "
            f"{args.lib_dir}): {', '.join(sorted(unresolved))}"
        )

    staged: dict[str, Path] = {}
    total = 0
    for dep in sorted(deps):
        # Stage under the SONAME the loader will ask for, not the file's
        # own name — conda ships `libfoo.so.1` as a symlink to
        # `libfoo.so.1.2.3`, and a flat directory has no symlinks.
        soname = next(
            (
                line.split("[")[1].split("]")[0]
                for line in _readelf(args.readelf, "-d", str(dep)).splitlines()
                if "SONAME" in line
            ),
            dep.name,
        )
        copy_with_soname(dep, args.dest, soname)
        staged[soname] = args.dest / soname
        total += dep.stat().st_size

    objects = [p.resolve() for p in args.roots] + list(staged.values())

    # 1. glibc floor across everything that ships.
    floor = max(glibc_floor(args.readelf, obj) for obj in objects)

    # 2. every versioned symbol demanded of a SHIPPED library must be
    #    provided by the copy that ships.
    provided = {name: version_defs(args.readelf, path) for name, path in staged.items()}
    failures: list[str] = []
    for obj in objects:
        for soname, wanted in version_needs(args.readelf, obj).items():
            if soname not in provided:
                continue  # a system library; covered by the glibc floor
            missing = wanted - provided[soname]
            if missing:
                failures.append(
                    f"{obj.name} needs {', '.join(sorted(missing))} from "
                    f"{soname}, which the staged copy does not provide"
                )
    # 3. anything with a dependency inside the staged set has to be able to
    #    find it after the directory moves — i.e. from $ORIGIN, since
    #    DT_RUNPATH does not reach transitive dependencies.
    for path in [p.resolve() for p in args.roots] + list(staged.values()):
        if not any(n in staged for n in needed(args.readelf, path)):
            continue
        entries = runpath(args.readelf, path)
        if not any(is_origin_entry(e) for e in entries):
            failures.append(
                f"{path.name} depends on staged libraries but searches "
                f"{', '.join(entries) or '(no rpath)'} — none of which is "
                "its own directory, so they resolve here and not on the box"
            )

    if failures:
        print("staged set is not self-consistent:", file=sys.stderr)
        for line in failures:
            print(f"  {line}", file=sys.stderr)
        return 1

    print(
        f"staged {len(staged)} libraries ({total / 1e6:.1f} MB) into {args.dest}"
    )
    print(
        "glibc floor: GLIBC_"
        + ".".join(str(p) for p in floor[: 2 if floor[2] == 0 else 3])
        + "  (the control box's glibc must be at least this)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
