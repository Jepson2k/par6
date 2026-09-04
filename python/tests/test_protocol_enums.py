"""The public state enums are waldoctl's; the wire's are generated from the
Rust codec. Both must agree member-for-member, or a decoded STATUS lands in
a waldoctl member naming a different state and nothing downstream notices."""

from __future__ import annotations

import pytest
from waldoctl.status import ActionState as WActionState
from waldoctl.tools import ToolState as WToolState

from par6.protocol.constants import ActionState, ToolState


@pytest.mark.parametrize(
    ("generated", "public"),
    [(ActionState, WActionState), (ToolState, WToolState)],
)
def test_the_public_state_enums_carry_the_generated_wire_values(generated, public):
    assert {m.name: m.value for m in generated} == {m.name: m.value for m in public}
    for member in generated:
        assert public(int(member)).name == member.name
