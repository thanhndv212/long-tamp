"""A release that fails is redrawn from its start, in bounded rounds (#54).

``_plan_release_subphase`` runs ``_plan_release_edges`` (pregrasp generation,
_21, then _10 or the direct edge) up to ``1 + _MAX_GENERATION_RETRIES`` times:
a round that can't generate a pregrasp, or reaches one it can't leave, is
redrawn from the held configuration. Nothing is committed to the grasp
tracker until a round succeeds. The helpers that touch HPP are stubbed, as in
``test_grasp_release_capabilities.py``.

Importing long_tamp's grasp_sequence requires pyhpp, so these run in the
hpp-arm64 container, like the other grasp_sequence tests.
"""

import pytest

from long_tamp.planning.grasp_state import GraspStateTracker
from long_tamp.tasks.grasp_sequence import GraspSequencePlanner


def _planner(failing_rounds):
    planner = object.__new__(GraspSequencePlanner)
    planner.grasp_tracker = GraspStateTracker(
        grippers=["g1", "g2"], handles=["h1", "h2"], initial_grasps={"g1": "h1"}
    )
    planner.phase_results = []
    planner.graph_constraints = None
    planner._MAX_GENERATION_RETRIES = 2
    planner._MAX_COLLISION_RETRIES = 10
    planner._setup_release_phase_graph = lambda *a, **kw: ["e_21", "e_10"]
    planner._project_onto_release_source_state = lambda q, verbose: q
    planner.grasp_tracker.get_approach_edge_from_released = lambda gripper: "e_01"
    planner._build_release_phase_info = lambda **kw: {"final_config": kw["q_start"]}
    rounds = []

    def edges(gripper, handle, e21, e10, e01, q_start, verbose):
        rounds.append(list(q_start))
        if len(rounds) <= failing_rounds:
            raise RuntimeError(f"Auto-release of '{handle}': no pregrasp")
        return {
            "q_start": [9.0],
            "path21": "p21",
            "path10": "p10",
            "path_direct": None,
            "used_direct": False,
            "t_gen1": 0.0,
            "t_plan1": 0.0,
            "t_gen2": 0.0,
            "t_plan2": 0.0,
            "t_gen_direct": 0.0,
            "t_plan_direct": 0.0,
        }

    planner._plan_release_edges = edges
    return planner, rounds


@pytest.mark.parametrize("failing_rounds", [0, 1, 2])
def test_a_failed_round_is_redrawn_from_the_held_configuration(failing_rounds):
    planner, rounds = _planner(failing_rounds)
    q_final, info = planner._plan_release_subphase("g1", [1.0], None, verbose=False)
    assert q_final == [9.0] and info == {"final_config": [9.0]}
    assert rounds == [[1.0]] * (failing_rounds + 1)  # every round from q_current
    assert planner.grasp_tracker.current_grasps["g1"] is None


def test_the_last_rounds_error_is_raised_and_nothing_is_committed():
    planner, rounds = _planner(failing_rounds=3)
    with pytest.raises(RuntimeError, match="no pregrasp"):
        planner._plan_release_subphase("g1", [1.0], None, verbose=False)
    assert len(rounds) == 3
    assert planner.grasp_tracker.current_grasps["g1"] == "h1"


def test_release_reports_a_release_that_failed_every_round():
    planner, _ = _planner(failing_rounds=3)
    result = planner.release("g1", [1.0], frozen_arms_mode="none", verbose=False)
    assert result["success"] is False
    assert "no pregrasp" in result["message"]
    assert result["final_config"] == [1.0]


class _Graph:
    def __init__(self, projected):
        self.projected = projected

    def apply_state_constraints(self, state_name, q, max_iterations, error_threshold):
        return True, self.projected, 0.0


class _Checker:
    def __init__(self, invalid):
        self.invalid = invalid

    def is_config_valid(self, q):
        return (False, "collision") if q in self.invalid else (True, "")


def _projecting_planner(projected, invalid):
    planner = object.__new__(GraspSequencePlanner)
    planner.grasp_tracker = GraspStateTracker(
        grippers=["g1"], handles=["h1"], initial_grasps={"g1": "h1"}
    )
    planner.grasp_tracker.get_current_state_name = lambda: "g1 grasps h1"
    planner.graph_builder = _Graph(projected)
    planner.config_gen = _Checker(invalid)
    return planner


def test_a_valid_projection_is_used():
    planner = _projecting_planner([2.0], invalid=[])
    assert planner._project_onto_release_source_state([1.0], verbose=False) == [2.0]


def test_a_projection_into_collision_keeps_the_held_configuration():
    """#54: projecting the held configuration onto the grasp's constraints
    could push a Panda link into the ground, and every release attempt from
    there failed on that collision."""
    planner = _projecting_planner([2.0], invalid=[[2.0]])
    assert planner._project_onto_release_source_state([1.0], verbose=False) == [1.0]
