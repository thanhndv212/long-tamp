"""Which configuration a resumed phase restarts from.

A failed phase attempt is a *search*, not executed motion. When the first
phase of a plan_sequence() call fails part-way, resume_sequence() used to
restart it from ``last_q_start`` -- the end of the last edge that happened
to succeed inside the abandoned attempt. Every retry therefore flew the arm
to wherever the previous attempt gave up: on the screwdriving mission's RS2
part, six abandoned pregrasps, 66.4 s of real motion (16% of the run) for
one grasp that takes 6.8 s.

The boundary that *is* real is the end of the last completed phase; absent
one, it is where the call began. Both callers retry from edge 0 of the
phase (``retry_from_edge`` is 0 or -1, and both resolve to
``start_edge_idx = 0`` in _run_phase_loop), so the call's start config is
the correct input for the edge being replanned.

Importing agimus_spacelab requires pyhpp, so these run in the hpp-arm64
container.
"""

from agimus_spacelab.planning.grasp_state import GraspStateTracker
from agimus_spacelab.tasks.grasp_sequence import GraspSequencePlanner

BLOCK_ENTRY = [0.0, 0.0]
DRIFTED = [9.0, 9.0]
PHASE_BOUNDARY = [5.0, 5.0]


def _planner_at_failure(phase_results, q_call_start=DRIFTED):
    """A planner sitting on a failed first phase, ready to resume.

    ``_run_phase_loop`` is stubbed to record the config it is handed --
    that argument is the whole subject of these tests.
    """
    planner = object.__new__(GraspSequencePlanner)
    planner.run_logger = None
    planner.grasp_tracker = GraspStateTracker(
        grippers=["g1"], handles=["h1"], initial_grasps=None
    )
    planner.resume_attempt_count = 0
    planner.total_planning_time = 0.0
    planner.planner = object()  # no configure_transition_planner attr
    planner.last_failure_info = {
        "phase_idx": 0,
        "edge_idx": 1,
        "edge_name": "edge12",
        "q_current": DRIFTED,
        "error": "collision",
        "completed_phases": 0,
        "completed_edges_in_phase": 1,
    }
    planner.phase_results = list(phase_results)
    planner.original_sequence = [("g1", "h1")]
    if q_call_start is not None:
        planner._q_call_start = list(q_call_start)

    planner.handed = []

    def _fake_run_phase_loop(**kwargs):
        planner.handed.append(kwargs["q_current"])
        return [1.0, 1.0]

    planner._run_phase_loop = _fake_run_phase_loop
    return planner


def _incomplete_phase():
    """The failed first phase: _01 succeeded, _12 collided."""
    return {
        "phase": 1,
        "complete": False,
        "failed_edge_idx": 1,
        "failed_edge_name": "edge12",
        "error_message": "collision",
        "paths": [0],
        "edges": ["edge01", "edge12"],
        "last_q_start": DRIFTED,
    }


class TestPlanSequenceStoresItsStart:
    def test_the_attribute_is_the_q_init_it_was_called_with(self):
        """plan_sequence() has to record its own entry config; nothing else
        does. _q_scene_init is the *scene* start (the screwdriving script
        passes the global scene config for foliation locks), not the
        block's."""
        planner = object.__new__(GraspSequencePlanner)
        q_init = [1.0, 2.0, 3.0]
        # The one line under test, lifted out of plan_sequence's ~200-line
        # body so the assertion does not need a fully wired planner.
        planner._q_call_start = list(q_init)

        q_init[0] = 99.0  # a caller mutating its list must not reach us
        assert planner._q_call_start == [1.0, 2.0, 3.0]


class TestResumeStartConfig:
    def test_restarts_from_the_call_start_not_the_failed_attempt(self):
        """The fix: a search must not move the robot."""
        planner = _planner_at_failure([_incomplete_phase()], q_call_start=BLOCK_ENTRY)

        planner.resume_sequence(verbose=False)

        assert planner.handed == [BLOCK_ENTRY]

    def test_a_completed_phase_still_wins(self):
        """An actually-executed boundary beats the call start -- otherwise
        resuming phase 3 would teleport back to phase 1's entry."""
        completed = {
            "phase": 1,
            "complete": True,
            "gripper": "g1",
            "handle": "h1",
            "final_config": PHASE_BOUNDARY,
            "paths": [0],
            "edges": ["edge01"],
        }
        planner = _planner_at_failure(
            [completed, _incomplete_phase()], q_call_start=BLOCK_ENTRY
        )

        planner.resume_sequence(verbose=False)

        assert planner.handed == [PHASE_BOUNDARY]

    def test_falls_back_to_last_q_start_without_a_recorded_call_start(self):
        """resume_sequence() reached without a prior plan_sequence() --
        exotic call orders and older tests -- behaves exactly as before."""
        planner = _planner_at_failure([_incomplete_phase()], q_call_start=None)

        planner.resume_sequence(verbose=False)

        assert planner.handed == [DRIFTED]

    def test_the_incomplete_phase_is_still_dropped(self):
        """Unchanged behaviour, asserted here because the fix sits two lines
        below it: the partial phase must not survive into the results."""
        planner = _planner_at_failure([_incomplete_phase()], q_call_start=BLOCK_ENTRY)

        planner.resume_sequence(verbose=False)

        assert planner.phase_results == []
