"""The type stub's enum members match the extension that defines them.

`par6._par6` builds its `IntEnum`s at module init from par6-proto's
`variants()`, so the runtime cannot drift from the Rust. A `.pyi` is read
statically by a checker that imports nothing and cannot do the same, so the
members are generated into it — and this is what catches a generation that
was not re-run. Without it, adding a wire enum member leaves `ty` unable to
resolve it everywhere it is used: a lint failure with no obvious cause.

The generator is invoked as a subprocess rather than imported, because it
lives in `scripts/` and is not on the package path.
"""

import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent.parent


def test_the_stub_lists_the_extensions_enum_members() -> None:
    done = subprocess.run(
        [sys.executable, str(REPO / "scripts" / "gen_stub.py"), "--check"],
        capture_output=True,
        text=True,
    )
    assert done.returncode == 0, done.stderr or done.stdout


def test_the_generated_region_is_not_empty() -> None:
    """A generator that silently produced nothing would satisfy the check
    above against an empty stub, which is the state this exists to prevent."""
    stub = (REPO / "python" / "par6" / "_par6.pyi").read_text()
    assert "class ErrorCode(IntEnum):" in stub
    assert "MOTN_NOT_HOMED" in stub, "ErrorCode has no members in the stub"
