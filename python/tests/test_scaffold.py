"""Package metadata sanity: the version the installed distribution reports
is the version this checkout declares.

``par6.__version__`` comes from ``importlib.metadata``, i.e. from whatever
install is live — so a bumped ``pyproject.toml`` with a stale (non-editable)
install, or a stale editable install's metadata, fails here instead of
shipping a package that reports the wrong version.
"""

import pathlib
import tomllib

import par6


def test_installed_version_matches_pyproject():
    pyproject = pathlib.Path(__file__).resolve().parents[1] / "pyproject.toml"
    with pyproject.open("rb") as f:
        declared = tomllib.load(f)["project"]["version"]
    assert par6.__version__ == declared, (
        f"installed par6 reports {par6.__version__!r} but this checkout "
        f"declares {declared!r} — reinstall (pip install -e python/) or fix "
        f"the version"
    )
