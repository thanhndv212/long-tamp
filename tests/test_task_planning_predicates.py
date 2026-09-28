"""Preconditions and effects: the predicate language, descriptors, plan-time
simulation and the runtime precondition check (issue #2, ADR-0002)."""

import json
import warnings

import pytest

from long_tamp.tasks.task_planning import (
    CapabilityDescriptor,
    CapabilityRegistry,
    PlanValidationError,
    TaskPlan,
    TaskPlanningSession,
)
from long_tamp.tasks.task_planning.predicates import (
    Literal,
    apply_effects,
    holds,
    parse_atom,
)

# --------------------------------------------------------------------- language


def test_literals_parse_and_ground():
    literal = Literal.parse("not holds(?gripper, _)")
    assert not literal.positive
    assert literal.atom.name == "holds"
    assert literal.atom.args == ("?gripper", "_")
    assert literal.variables() == {"gripper"}

    ground = Literal.parse("holds(?gripper, ?handle)").ground(
        {"gripper": "left/gripper", "handle": "ball/handle"}
    )
    assert str(ground) == "holds(left/gripper, ball/handle)"


def test_ground_atoms_allow_path_like_constants():
    atom = parse_atom("clamped(part1, fixtures/clamp1)")
    assert atom.args == ("part1", "fixtures/clamp1")
    assert str(atom) == "clamped(part1, fixtures/clamp1)"


@pytest.mark.parametrize(
    "text",
    ["grasp_state", "Holds(a)", "holds(a,)", "holds(?)", "not", "holds(a) extra"],
)
def test_malformed_literals_are_rejected(text):
    with pytest.raises(ValueError):
        Literal.parse(text)


def test_wildcards_match_any_argument():
    state = frozenset({parse_atom("holds(left, ball)")})
    assert holds(Literal.parse("holds(left, _)"), state)
    assert not holds(Literal.parse("not holds(left, _)"), state)
    assert holds(Literal.parse("not holds(right, _)"), state)


def test_effects_delete_before_add():
    state = frozenset({parse_atom("holds(left, ball)")})
    after = apply_effects(
        state, [Literal.parse("not holds(left, _)"), Literal.parse("holds(left, cup)")]
    )
    assert after == frozenset({parse_atom("holds(left, cup)")})


# ------------------------------------------------------------------ descriptors


def test_descriptor_checks_literal_variables_against_parameters():
    with pytest.raises(ValueError, match="unknown parameter.*handle"):
        CapabilityDescriptor(
            "grasp", "1.0", {"gripper": str}, effects=("holds(?gripper, ?handle)",)
        )


def test_positive_effects_cannot_use_wildcards():
    with pytest.raises(ValueError, match="wildcard"):
        CapabilityDescriptor(
            "grasp", "1.0", {"gripper": str}, effects=("holds(?gripper, _)",)
        )


def test_bare_effect_tags_move_to_writes_with_a_deprecation_warning():
    with pytest.warns(DeprecationWarning, match="writes="):
        descriptor = CapabilityDescriptor("move", "1.0", {}, effects=("robot_pose",))
    assert descriptor.writes == ("robot_pose",)
    assert descriptor.effects == ()


def test_snapshot_includes_preconditions_effects_and_writes():
    descriptor = CapabilityDescriptor(
        "release",
        "1.0",
        {"gripper": str},
        preconditions=("holds(?gripper, _)",),
        effects=("not holds(?gripper, _)",),
        writes=("grasp_state",),
    )
    snapshot = descriptor.snapshot()
    assert snapshot["preconditions"] == ["holds(?gripper, _)"]
    assert snapshot["effects"] == ["not holds(?gripper, _)"]
    assert snapshot["writes"] == ["grasp_state"]


# --------------------------------------------------------- plan-time simulation


def _registry(grasp_pre=("not holds(?gripper, _)",)):
    registry = CapabilityRegistry()
    registry.register(
        CapabilityDescriptor(
            "grasp",
            "1.0",
            {"gripper": str, "handle": str},
            preconditions=grasp_pre,
            effects=("holds(?gripper, ?handle)",),
            max_attempts=3,
            restartable=True,
        ),
        lambda parameters: {},
    )
    registry.register(
        CapabilityDescriptor(
            "release",
            "1.0",
            {"gripper": str},
            preconditions=("holds(?gripper, _)",),
            effects=("not holds(?gripper, _)",),
            restartable=True,
        ),
        lambda parameters: {},
    )
    registry.register(
        CapabilityDescriptor(
            "empty",
            "1.0",
            {"gripper": str},
            preconditions=("not holds(?gripper, _)",),
        ),
        lambda parameters: True,
    )
    return registry


def _transaction(node_id, capability, **parameters):
    return {
        "type": "transaction",
        "id": node_id,
        "restart_state": ["q_current"],
        "children": [
            {
                "type": "operation",
                "id": f"{node_id}.execute",
                "capability": capability,
                "parameters": parameters,
            }
        ],
    }


def _document(children, initial_state=()):
    document = {
        "schema_version": "1.0",
        "mission_id": "grasp-demo",
        "scene": {"id": "fake"},
        "provenance": {"kind": "human", "generator": "test"},
        "root": {"type": "sequence", "id": "root", "children": children},
    }
    if initial_state is not None:
        document["initial_state"] = list(initial_state)
    return document


def test_feasible_sequence_is_accepted():
    document = _document(
        [
            _transaction("grasp-1", "grasp", gripper="left", handle="ball"),
            _transaction("release-1", "release", gripper="left"),
            _transaction("grasp-2", "grasp", gripper="left", handle="cup"),
        ]
    )
    TaskPlan.from_dict(document, _registry())


def test_grasping_with_an_occupied_gripper_is_rejected_before_any_geometry():
    document = _document(
        [
            _transaction("grasp-1", "grasp", gripper="left", handle="ball"),
            _transaction("grasp-2", "grasp", gripper="left", handle="cup"),
        ]
    )
    with pytest.raises(PlanValidationError) as error:
        TaskPlan.from_dict(document, _registry())
    message = str(error.value)
    assert "grasp-2.execute" in message
    assert "not holds(left, _)" in message


def test_initial_state_is_respected():
    document = _document(
        [_transaction("release-1", "release", gripper="left")],
        initial_state=["holds(left, ball)"],
    )
    TaskPlan.from_dict(document, _registry())
    # Grasping the ball with a gripper that already holds the cup is infeasible
    # (and not already done, so the effect guard does not skip it).
    document = _document(
        [_transaction("grasp-1", "grasp", gripper="left", handle="ball")],
        initial_state=["holds(left, cup)"],
    )
    with pytest.raises(PlanValidationError, match="grasp-1.execute"):
        TaskPlan.from_dict(document, _registry())


def test_without_initial_state_only_syntax_is_checked():
    document = _document(
        [
            _transaction("grasp-1", "grasp", gripper="left", handle="ball"),
            _transaction("grasp-2", "grasp", gripper="left", handle="cup"),
        ],
        initial_state=None,
    )
    TaskPlan.from_dict(document, _registry())


def test_malformed_initial_state_is_rejected():
    with pytest.raises(PlanValidationError, match="initial_state"):
        TaskPlan.from_dict(
            _document([], initial_state=["holds(?x, ball)"]), _registry()
        )


def test_guarded_regrasp_simulates_both_fallback_branches():
    """The TWIN regrasp shape: the guarded branch only runs when the condition
    is false, i.e. when the gripper holds something, so ``release`` is safe."""
    document = _document(
        [
            _transaction("grasp-1", "grasp", gripper="left", handle="ball"),
            {
                "type": "fallback",
                "id": "already-cycled",
                "children": [
                    {
                        "type": "condition",
                        "id": "gripper-empty",
                        "capability": "empty",
                        "parameters": {"gripper": "left"},
                    },
                    {
                        "type": "sequence",
                        "id": "release-then-regrasp",
                        "children": [
                            _transaction("release-1", "release", gripper="left"),
                            _transaction(
                                "grasp-2", "grasp", gripper="left", handle="ball"
                            ),
                        ],
                    },
                ],
            },
        ]
    )
    TaskPlan.from_dict(document, _registry())


def test_a_branch_reachable_only_on_failure_is_still_checked():
    # The second fallback child runs when the first transaction fails, from the
    # unchanged state: grasping there with the gripper already full is infeasible.
    document = _document(
        [
            _transaction("grasp-1", "grasp", gripper="left", handle="ball"),
            {
                "type": "fallback",
                "id": "grasp-something",
                "children": [
                    _transaction("grasp-cup", "grasp", gripper="right", handle="cup"),
                    _transaction("grasp-mug", "grasp", gripper="left", handle="mug"),
                ],
            },
        ]
    )
    with pytest.raises(PlanValidationError, match="grasp-mug.execute"):
        TaskPlan.from_dict(document, _registry())


def test_a_transaction_whose_effect_already_holds_is_skipped_in_simulation():
    """Matches run time (ADR-0002): the step is complete, so its preconditions
    are not checked and the state is unchanged."""
    document = _document(
        [
            _transaction("grasp-1", "grasp", gripper="left", handle="ball"),
            _transaction("release-1", "release", gripper="left"),
        ],
        initial_state=["holds(left, ball)"],
    )
    TaskPlan.from_dict(document, _registry())


def test_fingerprint_changes_when_a_precondition_changes():
    document = _document(
        [_transaction("grasp-1", "grasp", gripper="left", handle="ball")]
    )
    first = TaskPlan.from_dict(document, _registry())
    second = TaskPlan.from_dict(document, _registry(grasp_pre=()))
    assert first.plan_fingerprint != second.plan_fingerprint


# --------------------------------------------------------- runtime precondition


def _session(world):
    document = _document(
        [_transaction("release-1", "release", gripper="left")],
        initial_state=None,
    )
    registry = _registry()
    plan = TaskPlan.from_dict(document, registry)
    return TaskPlanningSession(plan, registry, world_state=lambda: world)


def test_step_ready_evaluates_preconditions_against_the_world():
    world = {"holds(left, ball)"}
    session = _session(world)
    assert json.loads(session.check_precondition("release-1"))["ready"] is True
    world.clear()
    result = json.loads(session.check_precondition("release-1"))
    assert result["ready"] is False
    assert result["unsatisfied"] == ["holds(left, _)"]


def test_without_a_world_state_step_ready_keeps_the_existence_check():
    document = _document([_transaction("release-1", "release", gripper="left")], None)
    registry = _registry()
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        session = TaskPlanningSession(TaskPlan.from_dict(document, registry), registry)
    assert json.loads(session.check_precondition("release-1"))["ready"] is True
    assert json.loads(session.check_precondition("missing"))["ready"] is False
