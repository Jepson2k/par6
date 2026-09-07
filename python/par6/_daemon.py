"""The ``par6d`` console script a wheel install provides.

`pip install par6` should be enough to run a simulator, so the wheel carries
the runtime binary beside the extension (`scripts/deploy/pack-wheel.py`) and
this shim puts it on `PATH` as `par6d`. It is a shim rather than the binary
itself because a `[project.scripts]` entry point must be Python; `execv`
replaces this process with the real one, so signals, exit status and the
`PAR6D_READY` line on stdout behave exactly as they do for a binary invoked
directly — which is what lets the same command serve a terminal, a systemd
unit and `Robot().start()`.

The packaged runtime is pointed at the packaged config and assets, and
nowhere else. A `par6d` installed on the machine is the machine's own — on a
control box that is the systemd service reading `/etc/par6` — and a pip
install must not redirect it.
"""

from __future__ import annotations

import os
import shutil
import sys
from importlib import resources
from pathlib import Path


def packaged_binary() -> Path | None:
    """The ``par6d`` this install shipped, if it shipped one.

    A wheel built by `pixi run wheel` has it; a source install does not, and
    a checkout is expected to use its own build.
    """
    try:
        binary = resources.files("par6") / "_bin" / "par6d"
        with resources.as_file(binary) as path:
            return path if path.is_file() else None
    except (ModuleNotFoundError, FileNotFoundError):
        return None


def resolve() -> str | None:
    """The ``par6d`` to run: ``PAR6D_BIN``, then ``PATH``; None if there is
    none to run.

    `PATH` needs one more question asked of it than it looks like. A source
    or editable install puts THIS console script on `PATH` as `par6d`
    whether or not anything is behind it, because `[project.scripts]` is
    static metadata — so `which("par6d")` answers yes on a checkout that has
    never built the runtime, and the caller only finds out when the process
    it spawned exits 2. A wrapper is only a `par6d` if this install actually
    shipped a binary for it to exec.
    """
    env_bin = os.environ.get("PAR6D_BIN")
    if env_bin:
        return env_bin if os.path.isfile(env_bin) else None
    found = shutil.which("par6d")
    if found is None:
        return None
    return found if not _is_wrapper(found) or packaged_binary() else None


def _is_wrapper(path: str) -> bool:
    """Whether `path` is a console script rather than the native runtime."""
    try:
        with open(path, "rb") as fh:
            return fh.read(4) != b"\x7fELF"
    except OSError:
        return True


def packaged_data_args() -> list[str]:
    """`--config`/`--assets`/`--package-dir` for the data this wheel carries.

    The packaged URDFs name their meshes by `package://par6/_data/...` rather
    than relative to the tree, so the runtime has to be told what
    `package://par6` means; without it the collision model fails to load.
    """
    from par6 import config as cfg

    data = cfg.data_root()
    return [
        "--config",
        str(data / "config" / "PAR6.toml"),
        "--assets",
        str(data),
        "--package-dir",
        str(cfg.package_search_dir()),
    ]


def data_args_for(argv: list[str]) -> list[str]:
    """The packaged-data arguments to add to `argv`, if any.

    They are a DEFAULT for a caller who named nothing, never an override: a
    caller who passes `--config` or `--assets` gets theirs, which is what
    lets a systemd unit point this at `/etc/par6` on a control box.
    """
    if any(a.startswith(("--config", "--assets")) for a in argv):
        return []
    return packaged_data_args()


def main() -> int:
    binary = packaged_binary()
    if binary is None:
        print(
            "this par6 install ships no par6d binary. A release wheel carries "
            "one; a source or editable install does not — build it with "
            "`pixi run build-daemon` and run it from `target/`.",
            file=sys.stderr,
        )
        return 2
    argv = sys.argv[1:]
    os.execv(str(binary), [str(binary), *argv, *data_args_for(argv)])


if __name__ == "__main__":
    sys.exit(main())
