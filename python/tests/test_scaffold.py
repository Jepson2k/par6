"""Package metadata sanity: the version the installed distribution reports
is the version this checkout declares.

The version is written once, in the workspace manifest's
``[workspace.package]``; every crate takes it with ``version.workspace =
true`` and the wheel declares ``dynamic = ["version"]`` so maturin reads it
from there. ``par6.__version__`` comes from ``importlib.metadata``, i.e.
from whatever install is live — so a bumped ``Cargo.toml`` with a stale
(non-editable) install, a stale editable install's metadata, or a break in
the dynamic-version plumbing that leaves the wheel labelled something else,
fails here instead of shipping a package that reports the wrong version.
"""

import pathlib
import tomllib

import par6


def test_installed_version_matches_the_workspace_manifest():
    cargo = pathlib.Path(__file__).resolve().parents[2] / "Cargo.toml"
    with cargo.open("rb") as f:
        declared = tomllib.load(f)["workspace"]["package"]["version"]
    assert par6.__version__ == declared, (
        f"installed par6 reports {par6.__version__!r} but this checkout "
        f"declares {declared!r} — reinstall (pixi run install-python) or fix "
        f"the version in Cargo.toml"
    )
