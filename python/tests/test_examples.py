"""The ``examples/`` scripts, run end to end against a real ``par6d --sim``.

Opt in with ``pytest -m examples --examples``: each script spawns its own
runtime and drives it for tens of seconds, so they are not part of the
default run. CI runs them, which is what keeps them from rotting into
snippets that no longer import.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest
from live_daemon import daemon_env, par6d_binary, requires_par6d

pytestmark = [pytest.mark.examples, pytest.mark.slow, requires_par6d]

EXAMPLES = sorted((Path(__file__).resolve().parents[2] / "examples").glob("*.py"))


def example_ids() -> list[str]:
    return [p.stem for p in EXAMPLES]


def test_every_example_is_covered() -> None:
    """A new example must not be able to arrive untested."""
    assert EXAMPLES, "examples/ has no scripts — did the tree move?"


@pytest.mark.timeout(400)
@pytest.mark.parametrize("script", EXAMPLES, ids=example_ids())
def test_example_runs(script: Path) -> None:
    """The script must exit 0 and print something.

    Each example spawns its own `par6d --sim`, so the assertion is the one
    a reader cares about: copy this file, run it, and it works.
    """
    env = daemon_env()
    env["PATH"] = os.environ.get("PATH", "")
    binary = par6d_binary()
    assert binary is not None
    env["PAR6D_BIN"] = binary

    proc = subprocess.run(
        [sys.executable, str(script)],
        capture_output=True,
        text=True,
        timeout=360,
        env=env,
        cwd=script.parent.parent,
    )
    assert proc.returncode == 0, (
        f"{script.name} exited {proc.returncode}\n"
        f"--- stdout ---\n{proc.stdout}\n--- stderr ---\n{proc.stderr}"
    )
    assert proc.stdout.strip(), f"{script.name} printed nothing"
