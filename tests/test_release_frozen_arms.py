"""Unit tests for GraspSequencePlanner._release_frozen_arms.

The invariant under test cost 25 minutes of wall clock and ~30k consecutive
solver failures to find (part3's FG release, 2026-08-14): **never freeze an arm
that holds the object being released.** arm3 holds `part3/h_wb` while
tool_holder releases `part3/h_fg`; freezing arm3 pins the workbench
carrying part3, so the ~0.25 m retreat has to come entirely from the UR10 and no
solution exists. Measured from the failing checkpoint: 0/200 target draws
converged with arm3 frozen, 107/200 without.

The second rule is the one that made the first rule's absence invisible: an
explicit per-phase override must reach the phase graph, because
`the mission runner's arm-loosening escalation` escalates by dropping arms from it after
repeated failures. Both release paths used to recompute the set from scratch,
so those escalations silently did nothing.

`_release_frozen_arms` and `compute_phase_locked_joints` read only class-level
arm maps and `self.grasp_tracker.current_grasps`, so these tests drive the
real methods on a bare instance -- the chain walk under test is the shipped
one, not a re-implementation. Arm maps mirror a real multi-arm task's config.

Importing long_tamp requires pyhpp, so these run in the hpp-arm64
container like the rest of the suite.
"""

import logging

from long_tamp.tasks.grasp_sequence import GraspSequencePlanner

# The tool-mounted grippers can be carried by either tool arm, and the
# actual carrier is resolved from the grasp state, which is what makes the
# chain walk transitive rather than a single hop.
GRIPPER_TO_ARM = {
    "arm1/g_tool": "ur10",
    "tool_holder/g_holder_part": ["ur10", "arm2"],
    "arm2/g_tool": "arm2",
    "driver/g_driver": ["ur10", "arm2"],
    **{f"arm3/g_wb{i}": "arm3" for i in range(1, 7)},
}
ALL_ARMS = ["ur10", "arm2", "arm3"]


class _Tracker:
    """Stands in for GraspStateTracker: only current_grasps is read."""

    def __init__(self, grasps):
        self.current_grasps = dict(grasps)


def _planner(grasps):
    """Bare planner carrying just what the two methods touch."""
    p = object.__new__(GraspSequencePlanner)
    p.GRIPPER_TO_ARM_MAP = dict(GRIPPER_TO_ARM)
    p.ALL_ARM_KEYWORDS = list(ALL_ARMS)
    p.grasp_tracker = _Tracker(grasps)
    return p


# The state entering part3 B: the UR10 holds the frame gripper, which holds
# part3 by its FG handle, while arm3 socket 3 holds part3 by its WB handle.
PART3_B_STATE = {
    "arm1/g_tool": "tool_holder/h_holder_tool",
    "arm2/g_tool": "driver/h_driver_tool",
    "arm3/g_wb3": "part3/h_wb",
    "tool_holder/g_holder_part": "part3/h_fg",
    "driver/g_driver": None,
}


class TestHolderIsNeverFrozen:
    """Rule 1 -- the part3 regression."""

    def test_manual_override_naming_the_holder_is_overruled(self):
        """Block B's frozen spec is ['arm2', 'arm3'] and arm3 holds the
        released object. arm3 must be dropped; arm2 must survive."""
        p = _planner(PART3_B_STATE)
        frozen = p._release_frozen_arms(
            "tool_holder/g_holder_part",
            "part3/h_fg",
            "manual",
            {0: ["arm2", "arm3"]},
            0,
        )
        assert "arm3" not in frozen
        assert frozen == ["arm2"]

    def test_auto_mode_drops_the_holder_too(self):
        """With no override the set comes from the chain walk, which already
        excludes the holder -- the path that was always correct."""
        p = _planner(PART3_B_STATE)
        frozen = p._release_frozen_arms(
            "tool_holder/g_holder_part",
            "part3/h_fg",
            "auto",
            None,
            0,
        )
        assert "arm3" not in frozen
        assert "ur10" not in frozen  # the releasing gripper's own arm

    def test_dropping_the_holder_is_logged(self):
        """Silence here is what let the bug survive 28 resumes."""
        p = _planner(PART3_B_STATE)
        logger = logging.getLogger("long_tamp.tasks.grasp_sequence")
        records = []
        handler = logging.Handler()
        handler.emit = records.append
        logger.addHandler(handler)
        try:
            p._release_frozen_arms(
                "tool_holder/g_holder_part",
                "part3/h_fg",
                "manual",
                {0: ["arm2", "arm3"]},
                0,
            )
        finally:
            logger.removeHandler(handler)
        warnings = [r for r in records if r.levelno >= logging.WARNING]
        assert warnings, "dropping a requested arm must warn"
        assert "arm3" in warnings[0].getMessage()
        assert "part3/h_fg" in warnings[0].getMessage()

    def test_transitive_holder_is_dropped(self):
        """part1 sits inside tool_holder, which the UR10 holds. Releasing the
        WB grasp must leave the UR10 free even though no UR10 gripper touches
        part1 directly -- two hops up the chain."""
        p = _planner(
            {
                "arm1/g_tool": "tool_holder/h_holder_tool",
                "tool_holder/g_holder_part": "part1/h_fg",
                "arm3/g_wb1": "part1/h_wb",
            }
        )
        frozen = p._release_frozen_arms(
            "arm3/g_wb1",
            "part1/h_wb",
            "manual",
            {0: ["ur10", "arm2"]},
            0,
        )
        assert "ur10" not in frozen
        assert frozen == ["arm2"]


class TestOverrideIsHonoured:
    """Rule 2 -- escalations must reach the phase graph."""

    def test_override_survives_when_no_arm_holds_the_object(self):
        """The screwdriver releases a CON handle on part4; arm3 holds part4, so
        it goes -- but a caller freezing only the UR10 keeps it."""
        p = _planner(
            {
                "arm1/g_tool": "tool_holder/h_holder_tool",
                "arm3/g_wb4": "part4/h_wb",
                "driver/g_driver": "part4/h_con2",
            }
        )
        frozen = p._release_frozen_arms(
            "driver/g_driver",
            "part4/h_con2",
            "manual",
            {2: ["ur10"]},
            2,
        )
        assert frozen == ["ur10"]

    def test_override_is_read_per_phase_index(self):
        p = _planner(PART3_B_STATE)
        spec = {0: ["arm2"], 1: []}
        assert p._release_frozen_arms(
            "tool_holder/g_holder_part", "part3/h_fg", "manual", spec, 0
        ) == ["arm2"]
        assert (
            p._release_frozen_arms(
                "tool_holder/g_holder_part", "part3/h_fg", "manual", spec, 1
            )
            == []
        )

    def test_missing_phase_entry_freezes_nothing(self):
        """An override dict without this phase means the caller asked for no
        freezing here -- not a silent fallback to the computed set."""
        p = _planner(PART3_B_STATE)
        assert (
            p._release_frozen_arms(
                "tool_holder/g_holder_part", "part3/h_fg", "manual", {7: ["ur10"]}, 0
            )
            == []
        )

    def test_manual_mode_without_a_dict_falls_back_to_the_walk(self):
        """frozen_arms_mode='manual' with per_phase_frozen_arms=None is the
        resume path's shape; it must still exclude the holder."""
        p = _planner(PART3_B_STATE)
        frozen = p._release_frozen_arms(
            "tool_holder/g_holder_part",
            "part3/h_fg",
            "manual",
            None,
            0,
        )
        assert "arm3" not in frozen


class TestNoHolder:
    """Nothing holds the released object -- the set passes through intact."""

    def test_tool_return_freezes_everything_requested(self):
        """The UR10 returns the frame gripper to the dispenser. No other arm
        holds the frame gripper, so both other arms stay frozen."""
        p = _planner(
            {
                "arm1/g_tool": "tool_holder/h_holder_tool",
                "arm2/g_tool": "driver/h_driver_tool",
            }
        )
        frozen = p._release_frozen_arms(
            "arm1/g_tool",
            "tool_holder/h_holder_tool",
            "manual",
            {0: ["arm2", "arm3"]},
            0,
        )
        assert frozen == ["arm2", "arm3"]

    def test_order_of_the_requested_list_is_preserved(self):
        p = _planner({"arm1/g_tool": "tool_holder/h_holder_tool"})
        frozen = p._release_frozen_arms(
            "arm1/g_tool",
            "tool_holder/h_holder_tool",
            "manual",
            {0: ["arm3", "arm2"]},
            0,
        )
        assert frozen == ["arm3", "arm2"]


class TestToolCarrierResolution:
    """Tool-mounted grippers belong to whichever arm holds the tool."""

    def test_screwdriver_held_by_arm2(self):
        p = _planner({"arm2/g_tool": "driver/h_driver_tool"})
        assert p._get_arms_for_gripper("driver/g_driver") == {"arm2"}
        assert p.compute_phase_locked_joints(
            "driver/g_driver", "auto", verbose=False
        ) == ["ur10", "arm3"]

    def test_screwdriver_held_by_ur10(self):
        p = _planner({"arm1/g_tool": "driver/h_driver_tool"})
        assert p._get_arms_for_gripper("driver/g_driver") == {"ur10"}
        assert p.compute_phase_locked_joints(
            "driver/g_driver", "auto", verbose=False
        ) == ["arm2", "arm3"]

    def test_tool_holder_held_by_arm2(self):
        p = _planner({"arm2/g_tool": "tool_holder/h_holder_tool"})
        assert p._get_arms_for_gripper("tool_holder/g_holder_part") == {"arm2"}

    def test_unheld_tool_falls_back_to_every_candidate_arm(self):
        p = _planner({})
        assert p._get_arms_for_gripper("driver/g_driver") == {
            "ur10",
            "arm2",
        }
        assert p.compute_phase_locked_joints(
            "driver/g_driver", "auto", verbose=False
        ) == ["arm3"]
