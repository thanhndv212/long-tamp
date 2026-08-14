"""Unit tests for GraspSequencePlanner._release_frozen_arms.

The invariant under test cost 25 minutes of wall clock and ~30k consecutive
solver failures to find (RS3's FG release, 2026-08-14): **never freeze an arm
that holds the object being released.** vispa2 holds `RS3/h_RS3_WB` while
frame_gripper releases `RS3/h_RS3_FG`; freezing vispa2 pins the workbench
carrying RS3, so the ~0.25 m retreat has to come entirely from the UR10 and no
solution exists. Measured from the failing checkpoint: 0/200 target draws
converged with vispa2 frozen, 107/200 without.

The second rule is the one that made the first rule's absence invisible: an
explicit per-phase override must reach the phase graph, because
`run_block_nonstop._maybe_loosen` escalates by dropping arms from it after
repeated failures. Both release paths used to recompute the set from scratch,
so those escalations silently did nothing.

`_release_frozen_arms` and `compute_phase_locked_joints` read only class-level
arm maps and `self.grasp_tracker.current_grasps`, so these tests drive the
real methods on a bare instance -- the chain walk under test is the shipped
one, not a re-implementation. Arm maps mirror `spacelab_config.yaml`.

Importing agimus_spacelab requires pyhpp, so these run in the hpp-arm64
container like the rest of the suite.
"""

import logging

from agimus_spacelab.tasks.grasp_sequence import GraspSequencePlanner

# From spacelab_config.yaml's arm_groups: the frame gripper rides on the UR10
# and the screwdriver on the VISPA arm, which is what makes the chain walk
# transitive rather than a single hop.
GRIPPER_TO_ARM = {
    "spacelab/g_ur10_tool": "ur10",
    "frame_gripper/g_FG_part": "ur10",
    "spacelab/g_vispa_tool": "vispa_",
    "screw_driver/g_SD_part": "vispa_",
    **{f"spacelab/g_vispa2_wb{i}": "vispa2" for i in range(1, 7)},
}
ALL_ARMS = ["ur10", "vispa_", "vispa2"]


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


# The state entering RS3 B: the UR10 holds the frame gripper, which holds
# RS3 by its FG handle, while vispa2 socket 3 holds RS3 by its WB handle.
RS3_B_STATE = {
    "spacelab/g_ur10_tool": "frame_gripper/h_FG_tool",
    "spacelab/g_vispa_tool": "screw_driver/h_SD_tool",
    "spacelab/g_vispa2_wb3": "RS3/h_RS3_WB",
    "frame_gripper/g_FG_part": "RS3/h_RS3_FG",
    "screw_driver/g_SD_part": None,
}


class TestHolderIsNeverFrozen:
    """Rule 1 -- the RS3 regression."""

    def test_manual_override_naming_the_holder_is_overruled(self):
        """Block B's frozen spec is ['vispa_', 'vispa2'] and vispa2 holds the
        released object. vispa2 must be dropped; vispa_ must survive."""
        p = _planner(RS3_B_STATE)
        frozen = p._release_frozen_arms(
            "frame_gripper/g_FG_part", "RS3/h_RS3_FG",
            "manual", {0: ["vispa_", "vispa2"]}, 0,
        )
        assert "vispa2" not in frozen
        assert frozen == ["vispa_"]

    def test_auto_mode_drops_the_holder_too(self):
        """With no override the set comes from the chain walk, which already
        excludes the holder -- the path that was always correct."""
        p = _planner(RS3_B_STATE)
        frozen = p._release_frozen_arms(
            "frame_gripper/g_FG_part", "RS3/h_RS3_FG", "auto", None, 0,
        )
        assert "vispa2" not in frozen
        assert "ur10" not in frozen  # the releasing gripper's own arm

    def test_dropping_the_holder_is_logged(self):
        """Silence here is what let the bug survive 28 resumes."""
        p = _planner(RS3_B_STATE)
        logger = logging.getLogger("agimus_spacelab.tasks.grasp_sequence")
        records = []
        handler = logging.Handler()
        handler.emit = records.append
        logger.addHandler(handler)
        try:
            p._release_frozen_arms(
                "frame_gripper/g_FG_part", "RS3/h_RS3_FG",
                "manual", {0: ["vispa_", "vispa2"]}, 0,
            )
        finally:
            logger.removeHandler(handler)
        warnings = [r for r in records if r.levelno >= logging.WARNING]
        assert warnings, "dropping a requested arm must warn"
        assert "vispa2" in warnings[0].getMessage()
        assert "RS3/h_RS3_FG" in warnings[0].getMessage()

    def test_transitive_holder_is_dropped(self):
        """RS1 sits inside frame_gripper, which the UR10 holds. Releasing the
        WB grasp must leave the UR10 free even though no UR10 gripper touches
        RS1 directly -- two hops up the chain."""
        p = _planner({
            "spacelab/g_ur10_tool": "frame_gripper/h_FG_tool",
            "frame_gripper/g_FG_part": "RS1/h_RS1_FG",
            "spacelab/g_vispa2_wb1": "RS1/h_RS1_WB",
        })
        frozen = p._release_frozen_arms(
            "spacelab/g_vispa2_wb1", "RS1/h_RS1_WB",
            "manual", {0: ["ur10", "vispa_"]}, 0,
        )
        assert "ur10" not in frozen
        assert frozen == ["vispa_"]


class TestOverrideIsHonoured:
    """Rule 2 -- escalations must reach the phase graph."""

    def test_override_survives_when_no_arm_holds_the_object(self):
        """The screwdriver releases a CON handle on RS4; vispa2 holds RS4, so
        it goes -- but a caller freezing only the UR10 keeps it."""
        p = _planner({
            "spacelab/g_ur10_tool": "frame_gripper/h_FG_tool",
            "spacelab/g_vispa2_wb4": "RS4/h_RS4_WB",
            "screw_driver/g_SD_part": "RS4/h_RS4_CON2",
        })
        frozen = p._release_frozen_arms(
            "screw_driver/g_SD_part", "RS4/h_RS4_CON2",
            "manual", {2: ["ur10"]}, 2,
        )
        assert frozen == ["ur10"]

    def test_override_is_read_per_phase_index(self):
        p = _planner(RS3_B_STATE)
        spec = {0: ["vispa_"], 1: []}
        assert p._release_frozen_arms(
            "frame_gripper/g_FG_part", "RS3/h_RS3_FG", "manual", spec, 0
        ) == ["vispa_"]
        assert p._release_frozen_arms(
            "frame_gripper/g_FG_part", "RS3/h_RS3_FG", "manual", spec, 1
        ) == []

    def test_missing_phase_entry_freezes_nothing(self):
        """An override dict without this phase means the caller asked for no
        freezing here -- not a silent fallback to the computed set."""
        p = _planner(RS3_B_STATE)
        assert p._release_frozen_arms(
            "frame_gripper/g_FG_part", "RS3/h_RS3_FG", "manual", {7: ["ur10"]}, 0
        ) == []

    def test_manual_mode_without_a_dict_falls_back_to_the_walk(self):
        """frozen_arms_mode='manual' with per_phase_frozen_arms=None is the
        resume path's shape; it must still exclude the holder."""
        p = _planner(RS3_B_STATE)
        frozen = p._release_frozen_arms(
            "frame_gripper/g_FG_part", "RS3/h_RS3_FG", "manual", None, 0,
        )
        assert "vispa2" not in frozen


class TestNoHolder:
    """Nothing holds the released object -- the set passes through intact."""

    def test_tool_return_freezes_everything_requested(self):
        """The UR10 returns the frame gripper to the dispenser. No other arm
        holds the frame gripper, so both vispas stay frozen."""
        p = _planner({
            "spacelab/g_ur10_tool": "frame_gripper/h_FG_tool",
            "spacelab/g_vispa_tool": "screw_driver/h_SD_tool",
        })
        frozen = p._release_frozen_arms(
            "spacelab/g_ur10_tool", "frame_gripper/h_FG_tool",
            "manual", {0: ["vispa_", "vispa2"]}, 0,
        )
        assert frozen == ["vispa_", "vispa2"]

    def test_order_of_the_requested_list_is_preserved(self):
        p = _planner({"spacelab/g_ur10_tool": "frame_gripper/h_FG_tool"})
        frozen = p._release_frozen_arms(
            "spacelab/g_ur10_tool", "frame_gripper/h_FG_tool",
            "manual", {0: ["vispa2", "vispa_"]}, 0,
        )
        assert frozen == ["vispa2", "vispa_"]
