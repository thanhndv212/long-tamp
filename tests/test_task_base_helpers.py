"""
Unit tests for pure-logic helpers extracted from ManipulationTask.run().

Importing agimus_spacelab requires pyhpp (see
docs/plans/refactor-manipulation-task-run.md's verification-model
correction) even though the helpers under test have no HPP dependency
themselves, so these tests must run inside the hpp-arm64 container.
"""

from agimus_spacelab.tasks.base import ManipulationTask


class TestOrderedConfigKeys:
    def test_missing_q_init_returns_empty(self):
        cfgs = {"q_goal": [0.0]}
        assert ManipulationTask._ordered_config_keys(cfgs, []) == []

    def test_missing_q_goal_returns_empty(self):
        cfgs = {"q_init": [0.0]}
        assert ManipulationTask._ordered_config_keys(cfgs, []) == []

    def test_factory_waypoints_ordered_by_index(self):
        cfgs = {
            "q_init": [0.0],
            "q_goal": [1.0],
            "q_wp_1_edgeB": [2.0],
            "q_wp_0_edgeA": [3.0],
        }
        assert ManipulationTask._ordered_config_keys(cfgs, []) == [
            "q_init",
            "q_wp_0_edgeA",
            "q_wp_1_edgeB",
            "q_goal",
        ]

    def test_preferred_configs_used_when_present_and_no_factory_keys(self):
        cfgs = {
            "q_init": [0.0],
            "q_goal": [1.0],
            "q_mid_b": [2.0],
            "q_mid_a": [3.0],
        }
        preferred = ["q_mid_a", "q_mid_b"]
        assert ManipulationTask._ordered_config_keys(cfgs, preferred) == [
            "q_init",
            "q_mid_a",
            "q_mid_b",
            "q_goal",
        ]

    def test_preferred_configs_filtered_to_present_keys(self):
        cfgs = {"q_init": [0.0], "q_goal": [1.0], "q_mid_a": [2.0]}
        preferred = ["q_mid_a", "q_mid_missing"]
        assert ManipulationTask._ordered_config_keys(cfgs, preferred) == [
            "q_init",
            "q_mid_a",
            "q_goal",
        ]

    def test_factory_waypoints_take_precedence_over_preferred(self):
        cfgs = {
            "q_init": [0.0],
            "q_goal": [1.0],
            "q_wp_0_edgeA": [2.0],
            "q_mid_a": [3.0],
        }
        preferred = ["q_mid_a"]
        assert ManipulationTask._ordered_config_keys(cfgs, preferred) == [
            "q_init",
            "q_wp_0_edgeA",
            "q_goal",
        ]

    def test_fallback_with_no_factory_or_preferred_keys_is_two_entries(self):
        # Documents the current (pre-existing, dead-code) fallback
        # behavior: the branch that would populate `mids` from arbitrary
        # q_* keys is commented out in the source, so this always
        # collapses to exactly [q_init, q_goal] regardless of how many
        # other q_* configs exist. See baseline/README.md Step 0 finding.
        cfgs = {
            "q_init": [0.0],
            "q_goal": [1.0],
            "q_other_state": [2.0],
        }
        assert ManipulationTask._ordered_config_keys(cfgs, []) == [
            "q_init",
            "q_goal",
        ]
