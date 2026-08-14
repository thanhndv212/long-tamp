"""
Unit test for GraspStateTracker.copy()'s mutation-isolation guarantee.

find_feasible_phase_target() (grasp_sequence.py) probes hypothetical
grasp-state transitions -- "what would phase N+1 look like if phase N
committed to THIS candidate target" -- while searching for a Phase-2 WB
grasp on RS6 that also leaves Phase 3's CON0 grasp reachable (see that
method's docstring for the full RS6 story). It must never call
update_grasp() on the REAL self.grasp_tracker while probing, only on a
throwaway copy() -- mutating the real tracker mid-probe is exactly the
corruption class documented in test_grasp_sequence_resume_state.py (grasps
silently dropped/overwritten, producing a structurally different edge and
turning target generation from "hard" into "unsatisfiable"). This test
pins down the specific invariant that safety argument depends on: that
copy() really does produce an independent tracker, not a shallow alias.

No pyhpp/HPP dependency -- GraspStateTracker is pure Python -- but kept
alongside the other grasp-sequence tests for discoverability.
"""

from agimus_spacelab.planning.grasp_state import GraspStateTracker

GRIPPERS = ["g_ur10_tool", "g_vispa2_wb6", "g_SD_part"]
HANDLES = ["h_FG_tool", "h_RS6_WB", "h_RS6_CON0"]


def test_mutating_the_copy_does_not_affect_the_original():
    # GraspStateTracker.__init__ uses initial_grasps as-is (does not fill
    # in unlisted grippers with None), so every gripper must be present.
    original = GraspStateTracker(
        grippers=GRIPPERS,
        handles=HANDLES,
        initial_grasps={"g_ur10_tool": "h_FG_tool", "g_vispa2_wb6": None, "g_SD_part": None},
    )
    original_snapshot = dict(original.current_grasps)

    probe = original.copy()
    probe.update_grasp("g_vispa2_wb6", "h_RS6_WB")
    probe.update_grasp("g_SD_part", "h_RS6_CON0")

    assert original.current_grasps == original_snapshot
    assert original.current_grasps["g_vispa2_wb6"] is None
    assert original.current_grasps["g_SD_part"] is None
    # The copy itself did pick up the mutations.
    assert probe.current_grasps["g_vispa2_wb6"] == "h_RS6_WB"
    assert probe.current_grasps["g_SD_part"] == "h_RS6_CON0"


def test_mutating_the_original_after_copying_does_not_affect_the_copy():
    original = GraspStateTracker(
        grippers=GRIPPERS, handles=HANDLES, initial_grasps=None
    )
    probe = original.copy()

    original.update_grasp("g_ur10_tool", "h_FG_tool")

    assert probe.current_grasps["g_ur10_tool"] is None


def test_copy_does_not_carry_over_phase_indices():
    # .copy() always starts phase indices unset (fresh __init__), even if
    # the source tracker had them set via set_phase_indices() -- so any
    # caller relying on abbreviated-state edge names from a probe tracker
    # must call set_phase_indices() again after each build_phase_graph()
    # it performs on that probe, not assume it inherited the original's.
    original = GraspStateTracker(
        grippers=GRIPPERS, handles=HANDLES, initial_grasps=None
    )
    original.set_phase_indices(
        phase_grippers=["g_ur10_tool"], phase_handles=["h_FG_tool"]
    )

    probe = original.copy()

    assert probe._phase_gripper_to_idx is None
    assert probe._phase_handle_to_idx is None
