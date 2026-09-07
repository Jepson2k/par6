"""The wheel's `par6d` is reachable and points at the packaged data.

`pip install par6` should be enough to run a simulator (issue #33), which
needs three things that are easy to break independently: the binary has to
be IN the wheel, the `par6d` console script has to find it, and it has to be
told where the packaged config, assets and `package://par6` root are — the
URDFs name their meshes by package URI, so without the last one the
collision model fails to load with everything else looking fine.

A source or editable install ships no binary, so the reachability test skips
there rather than failing; `validate-bundle.sh` covers the built wheel end to
end in CI.
"""

import subprocess
import sys

import pytest

from par6 import _daemon


def test_packaged_data_args_name_the_wheels_own_files() -> None:
    """Whatever the install layout, the paths handed to par6d are the ones
    this package carries — not `/etc/par6`, which belongs to a system
    service a client must not redirect."""
    from par6 import config as cfg

    args = _daemon.packaged_data_args()
    assert args[args.index("--config") + 1].startswith(str(cfg.data_root()))
    assert args[args.index("--assets") + 1] == str(cfg.data_root())
    # The mesh root has to be the directory CONTAINING `par6/`, because the
    # URDFs say `package://par6/_data/...`.
    assert args[args.index("--package-dir") + 1] == str(cfg.package_search_dir())


def test_the_shipped_daemon_runs_and_loads_the_packaged_config() -> None:
    binary = _daemon.packaged_binary()
    if binary is None:
        pytest.skip("no packaged par6d: this is a source or editable install")
    done = subprocess.run(
        [str(binary), "--check-config", *_daemon.packaged_data_args()],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert done.returncode == 0, done.stderr
    # It must have loaded THIS package's config, not one it found on the host.
    from par6 import config as cfg

    assert str(cfg.data_root()) in done.stdout, done.stdout


def test_an_explicit_config_is_not_overridden() -> None:
    """The packaged data is a default for someone who passed nothing. A
    caller who names a config gets theirs, or a systemd unit could never
    point this at /etc/par6 on a control box."""
    assert _daemon.data_args_for(["--sim"]) == _daemon.packaged_data_args()
    assert _daemon.data_args_for(["--sim", "--config", "/etc/par6/PAR6.toml"]) == []
    assert _daemon.data_args_for(["--assets=/usr/share/par6/par6_description"]) == []


def test_the_console_script_is_declared() -> None:
    """`shutil.which("par6d")` is how `Robot.start()` finds a runtime, so the
    entry point existing is what makes a wheel install self-sufficient."""
    meta = subprocess.run(
        [
            sys.executable,
            "-c",
            "import importlib.metadata as m;"
            "print([e.value for e in m.entry_points(group='console_scripts')"
            " if e.name == 'par6d'])",
        ],
        capture_output=True,
        text=True,
    )
    assert "par6._daemon:main" in meta.stdout, meta.stdout or meta.stderr
