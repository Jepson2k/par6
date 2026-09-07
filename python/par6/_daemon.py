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
    # An explicit --config/--assets wins: the packaged data is a default for
    # someone who passed nothing, not an override of what they asked for.
    args = [] if any(a.startswith(("--config", "--assets")) for a in argv) else packaged_data_args()
    os.execv(str(binary), [str(binary), *argv, *args])


if __name__ == "__main__":
    sys.exit(main())
