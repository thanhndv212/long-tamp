"""
Unit tests for GraspSequencePlanner._restore_grasp_tracker_for_resume().

Regression cover for the resume path dropping grasps that were established
by an EARLIER plan_sequence() call on the same planner.

A long mission can drive its run as a series of separate plan_sequence()
blocks against one planner: an initial bootstrap phase grabs shared tools,
then each subsequent unit of work is its own block. resume_sequence()
rebuilt the tracker from the all-free state and replayed only
self.phase_results -- which plan_sequence() resets per call -- so the
first resume of any post-bootstrap block silently dropped both tool
grasps. The rebuilt phase graph then treated the held tools as
free-floating objects and built a structurally different edge for the
same phase, turning target generation from "hard" into "unsatisfiable":
1095 consecutive restarts each stopped at the identical residual
9.783801504932926 against a 1e-4 threshold.

Importing long_tamp requires pyhpp (see docs/legacy/plans/refactor-codebase.md's
verification-model notes) even though the helper under test has no HPP
dependency itself, so these tests must run inside the hpp-arm64 container.
"""

from long_tamp.planning.grasp_state import GraspStateTracker
from long_tamp.tasks.grasp_sequence import GraspSequencePlanner

# Mirrors the multi-arm gripper/handle sets the failure was observed with.
GRIPPERS = [
    "arm1/g_tool",
    "arm2/g_tool",
    "arm3/g_wb1",
    "frame_gripper/g_FG_part",
    "screw_driver/g_SD_part",
]
HANDLES = [
    "frame_gripper/h_FG_tool",
    "screw_driver/h_SD_tool",
    "part1/h_wb",
    "part1/h_fg",
    "part1/h_con0",
]

# The two tool grasps bootstrap establishes before any RS block runs.
TOOL_GRASPS = {
    "arm1/g_tool": "frame_gripper/h_FG_tool",
    "arm2/g_tool": "screw_driver/h_SD_tool",
}


def _make_planner(initial_grasps, phase_results):
    """Bare planner carrying only what the helper under test reads.

    _restore_grasp_tracker_for_resume() touches self.grasp_tracker (for the
    gripper/handle ordering), self._initial_grasps and self.phase_results --
    no graph_builder/config_gen/backend wiring is needed, so __init__ is
    bypassed the same way tests/test_grasp_sequence_helpers.py does it.
    """
    planner = object.__new__(GraspSequencePlanner)
    planner.grasp_tracker = GraspStateTracker(
        grippers=GRIPPERS, handles=HANDLES, initial_grasps=None
    )
    planner._initial_grasps = dict(initial_grasps)
    planner.phase_results = list(phase_results)
    return planner


def _held(planner):
    return {
        g: h
        for g, h in planner.grasp_tracker.current_grasps.items()
        if h is not None
    }


def test_grasps_from_earlier_blocks_survive_resume():
    """The bug: bootstrap's tool grasps must not be lost on resume.

    Block state entering RS1's CON0 phase: both tools held (from bootstrap,
    a previous plan_sequence() call) plus this block's two completed phases.
    A resume here must reproduce all four grasps.
    """
    planner = _make_planner(
        initial_grasps=TOOL_GRASPS,
        phase_results=[
            {
                "phase": 1,
                "gripper": "frame_gripper/g_FG_part",
                "handle": "part1/h_fg",
                "complete": True,
            },
            {
                "phase": 2,
                "gripper": "arm3/g_wb1",
                "handle": "part1/h_wb",
                "complete": True,
            },
            # The CON0 phase that failed and triggered the resume.
            {
                "phase": 3,
                "gripper": "screw_driver/g_SD_part",
                "handle": "part1/h_con0",
                "complete": False,
            },
        ],
    )

    planner._restore_grasp_tracker_for_resume()

    assert _held(planner) == {
        **TOOL_GRASPS,
        "frame_gripper/g_FG_part": "part1/h_fg",
        "arm3/g_wb1": "part1/h_wb",
    }


def test_incomplete_phase_is_not_replayed():
    """Only completed phases contribute; the failed one must not be applied."""
    planner = _make_planner(
        initial_grasps=TOOL_GRASPS,
        phase_results=[
            {
                "phase": 1,
                "gripper": "screw_driver/g_SD_part",
                "handle": "part1/h_con0",
                "complete": False,
            }
        ],
    )

    planner._restore_grasp_tracker_for_resume()

    assert _held(planner) == TOOL_GRASPS


def test_release_in_this_block_overrides_seeded_grasp():
    """A completed release must win over the seeded initial state.

    Otherwise re-seeding would resurrect a grasp the block already gave up --
    the failure mode opposite to the one being fixed.
    """
    planner = _make_planner(
        initial_grasps={
            **TOOL_GRASPS,
            "frame_gripper/g_FG_part": "part1/h_fg",
        },
        phase_results=[
            {
                "phase": 1,
                "gripper": "frame_gripper/g_FG_part",
                "handle": None,
                "complete": True,
            }
        ],
    )

    planner._restore_grasp_tracker_for_resume()

    assert _held(planner) == TOOL_GRASPS


def test_auto_release_switch_replays_as_final_handle():
    """A gripper that switched objects ends on the new handle, not the old."""
    planner = _make_planner(
        initial_grasps={"screw_driver/g_SD_part": "part1/h_fg"},
        phase_results=[
            {
                "phase": 1,
                "gripper": "screw_driver/g_SD_part",
                "handle": "part1/h_con0",
                "complete": True,
            }
        ],
    )

    planner._restore_grasp_tracker_for_resume()

    assert _held(planner) == {"screw_driver/g_SD_part": "part1/h_con0"}


def test_planner_without_prior_plan_sequence_still_starts_free():
    """No _initial_grasps attribute at all -> previous all-free behaviour."""
    planner = _make_planner(initial_grasps={}, phase_results=[])
    del planner._initial_grasps

    planner._restore_grasp_tracker_for_resume()

    assert _held(planner) == {}
