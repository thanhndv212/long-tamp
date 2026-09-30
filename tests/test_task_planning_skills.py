"""Skill specs (#19): conditions, grounding, and the capability they become."""

import pytest

from long_tamp.tasks.task_planning.skills import SkillCommand, SkillPose, SkillSpec

SPEC = SkillSpec(
    name="insert",
    parameters=("tool", "hole"),
    preconditions=("holds(?tool, _)",),
    postconditions=("inserted(?tool, ?hole)",),
    failures=("jammed(?tool, ?hole)",),
    start=SkillPose("?tool", "?hole", offset=-0.01),
    end=SkillPose("?tool", "?hole"),
)


class _Path:
    def length(self):
        return 2.0

    def eval(self, t):
        return [t], True


def test_conditions_may_only_use_declared_parameters():
    with pytest.raises(ValueError, match="undeclared parameters"):
        SkillSpec("x", ("a",), postconditions=("p(?b)",))


def test_a_command_grounds_its_postconditions_and_failures():
    command = SkillCommand(SPEC, {"tool": "peg", "hole": "h1"}, _Path())
    assert command.postconditions() == ["inserted(peg, h1)"]
    assert command.failure("jammed") == "jammed(peg, h1)"
    with pytest.raises(KeyError):
        command.failure("unknown")
    assert SPEC.start.ground(command.parameters) == SkillPose("peg", "h1", -0.01)


def test_a_command_is_also_its_approach_path():
    command = SkillCommand(SPEC, {"tool": "peg", "hole": "h1"}, _Path())
    assert command.length() == 2.0
    assert command.eval(0.5) == ([0.5], True)


def test_a_skill_is_a_capability():
    descriptor = SPEC.descriptor()
    assert descriptor.capability_id == "insert"
    assert set(descriptor.required_parameters) == {"tool", "hole"}
    assert [str(e) for e in descriptor.effect_literals] == ["inserted(?tool, ?hole)"]


def test_grounding_needs_every_parameter():
    with pytest.raises(ValueError, match="missing parameters"):
        SPEC.ground({"tool": "peg"}, SPEC.postconditions)
