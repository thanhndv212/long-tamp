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


class TestParseFactoryWaypoints:
    def test_missing_q_init_or_q_goal_returns_empty(self):
        assert ManipulationTask._parse_factory_waypoints({"q_goal": [0.0]}) == (
            [],
            [],
        )
        assert ManipulationTask._parse_factory_waypoints({"q_init": [0.0]}) == (
            [],
            [],
        )

    def test_no_waypoint_keys_returns_empty(self):
        cfgs = {"q_init": [0.0], "q_goal": [1.0], "q_other": [2.0]}
        assert ManipulationTask._parse_factory_waypoints(cfgs) == ([], [])

    def test_orders_by_index_and_builds_full_waypoint_list(self):
        cfgs = {
            "q_init": [0.0],
            "q_goal": [3.0],
            "q_wp_1_edgeB": [2.0],
            "q_wp_0_edgeA": [1.0],
        }
        edges, waypoints = ManipulationTask._parse_factory_waypoints(cfgs)
        assert edges == ["edgeA", "edgeB"]
        assert waypoints == [[0.0], [1.0], [2.0], [3.0]]
        # Documents actual (pre-existing, unchanged) behavior: despite the
        # docstring's claimed len(waypoints) == len(edges) + 1 invariant,
        # this branch produces len(edges) + 2 (one q_wp_* entry per named
        # edge, plus separate q_init and q_goal, with no edge name for the
        # final "last waypoint -> q_goal" transition). This factory-
        # waypoint naming convention (q_wp_<i>_<edge>) is not produced
        # anywhere in src/ or script/ today, so this branch -- and this
        # discrepancy -- is currently dead code. Not fixed here (pure
        # relocation only); see baseline/README.md.
        assert len(waypoints) == len(edges) + 2

    def test_edge_name_can_contain_underscores(self):
        cfgs = {
            "q_init": [0.0],
            "q_goal": [1.0],
            "q_wp_0_some_edge_name": [0.5],
        }
        edges, waypoints = ManipulationTask._parse_factory_waypoints(cfgs)
        assert edges == ["some_edge_name"]
        assert waypoints == [[0.0], [0.5], [1.0]]
