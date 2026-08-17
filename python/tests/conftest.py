"""Session-wide guards and opt-in flags for the par6 test suite."""

from __future__ import annotations

import os

import pytest
from live_daemon import par6d_binary


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption(
        "--examples",
        action="store_true",
        default=False,
        help="run the examples/ scripts against a live `par6d --sim`",
    )


def pytest_collection_modifyitems(
    config: pytest.Config, items: list[pytest.Item]
) -> None:
    if config.getoption("--examples"):
        return
    skip = pytest.mark.skip(reason="needs --examples")
    for item in items:
        if "examples" in item.keywords:
            item.add_marker(skip)


def pytest_sessionstart(session: pytest.Session) -> None:
    """Fail the run when ``PAR6_REQUIRE_E2E`` is set and no ``par6d`` exists.

    Without the binary the whole e2e layer skips, which is how an entire
    integration surface can vanish from a green run unnoticed. CI sets the
    flag so the same absence is a failure there.
    """
    if os.environ.get("PAR6_REQUIRE_E2E") and par6d_binary() is None:
        raise pytest.UsageError(
            "PAR6_REQUIRE_E2E is set but no par6d binary was found. "
            "Set PAR6D_BIN or put par6d on PATH "
            "(`cargo build -p par6d --release`), or unset PAR6_REQUIRE_E2E "
            "to allow the e2e tests to skip."
        )
