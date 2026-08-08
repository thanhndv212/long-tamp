"""
Unit tests for pure-logic helpers extracted from ManipulationTask.run().

Importing agimus_spacelab requires pyhpp (see
docs/plans/refactor-manipulation-task-run.md's verification-model
correction) even though the helpers under test have no HPP dependency
themselves, so these tests must run inside the hpp-arm64 container.
"""

import pytest

from agimus_spacelab.tasks.base import ManipulationTask


class _ConcreteTask(ManipulationTask):
    """Minimal concrete subclass so the abstract base can be instantiated
    directly, bypassing __init__ (which needs real scene/backend wiring)."""

    def build_initial_config(self):
        raise NotImplementedError


def _make_task(config_gen=None):
    """Bare ManipulationTask instance, bypassing __init__.

    _compute_transition_inputs only reads self.config_gen and calls the
    already-tested self._parse_factory_waypoints, so a minimal object with
    just that attribute set is sufficient -- no need to construct a full
    task with real scene/backend wiring.
    """
    task = object.__new__(_ConcreteTask)
    task.config_gen = config_gen
    return task


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


class TestComputeTransitionInputs:
    """Covers the branches reachable without a real HPP config_gen.

    Branch 3's success path (self.config_gen.generate_via_edge(...)) needs
    a real HPP-backed ConfigGenerator and has no real-script coverage in
    this environment (see baseline/README.md's "solve_mode is a no-op"
    finding) -- only its guard clauses are covered here, disclosed gap
    rather than claimed coverage.
    """

    def test_explicit_waypoints_take_precedence(self):
        task = _make_task()
        cfgs = {"q_init": [0.0], "q_goal": [9.0]}
        edges, waypoints = task._compute_transition_inputs(
            cfgs,
            transition_edges=["e0", "e1"],
            transition_waypoints=[[0.0], [1.0], [9.0]],
            generate_waypoints_via_edges=False,
        )
        assert edges == ["e0", "e1"]
        assert waypoints == [[0.0], [1.0], [9.0]]

    def test_explicit_waypoints_without_edges_raises(self):
        task = _make_task()
        with pytest.raises(ValueError, match="requires transition_edges"):
            task._compute_transition_inputs(
                {},
                transition_edges=None,
                transition_waypoints=[[0.0], [1.0]],
                generate_waypoints_via_edges=False,
            )

    def test_explicit_waypoints_length_mismatch_raises(self):
        task = _make_task()
        with pytest.raises(ValueError, match="len.transition_waypoints"):
            task._compute_transition_inputs(
                {},
                transition_edges=["e0", "e1"],
                transition_waypoints=[[0.0], [1.0]],  # needs 3, has 2
                generate_waypoints_via_edges=False,
            )

    def test_factory_waypoints_used_when_no_explicit_waypoints(self):
        task = _make_task()
        cfgs = {
            "q_init": [0.0],
            "q_goal": [1.0],
            "q_wp_0_edgeA": [0.5],
        }
        edges, waypoints = task._compute_transition_inputs(
            cfgs,
            transition_edges=None,
            transition_waypoints=None,
            generate_waypoints_via_edges=False,
        )
        assert edges == ["edgeA"]
        assert waypoints == [[0.0], [0.5], [1.0]]

    def test_edges_without_generate_flag_raises(self):
        task = _make_task()
        cfgs = {"q_init": [0.0], "q_goal": [1.0]}
        with pytest.raises(ValueError, match="no waypoints"):
            task._compute_transition_inputs(
                cfgs,
                transition_edges=["e0"],
                transition_waypoints=None,
                generate_waypoints_via_edges=False,
            )

    def test_generate_flag_without_config_gen_raises(self):
        task = _make_task(config_gen=None)
        cfgs = {"q_init": [0.0], "q_goal": [1.0]}
        with pytest.raises(RuntimeError, match="ConfigGenerator not initialized"):
            task._compute_transition_inputs(
                cfgs,
                transition_edges=["e0"],
                transition_waypoints=None,
                generate_waypoints_via_edges=True,
            )

    def test_no_inputs_at_all_raises(self):
        task = _make_task()
        cfgs = {"q_init": [0.0], "q_goal": [1.0]}
        with pytest.raises(ValueError, match="requires explicit inputs"):
            task._compute_transition_inputs(
                cfgs,
                transition_edges=None,
                transition_waypoints=None,
                generate_waypoints_via_edges=False,
            )


class _FakePlanner:
    """Records calls instead of touching real HPP/viewer state."""

    def __init__(self, has_record_method=True, raise_on="none"):
        self.play_and_record_calls = []
        self.play_calls = []
        self._raise_on = raise_on
        if has_record_method:
            self.play_and_record_path = self._play_and_record_path

    def _play_and_record_path(self, path_index, video_name, output_dir, framerate):
        if self._raise_on == "record":
            raise RuntimeError("boom")
        self.play_and_record_calls.append(
            (path_index, video_name, output_dir, framerate)
        )
        return "/tmp/fake_video.mp4"

    def play_path(self, path_index):
        if self._raise_on == "play":
            raise RuntimeError("boom")
        self.play_calls.append(path_index)


class TestPlayAndRecord:
    """No real script in this environment reaches success=True through
    run()'s solve() call (see baseline/README.md), so _play_and_record's
    real relocated control flow -- not the HPP physics underneath it -- is
    verified here via a fake planner instead of a real successful run.
    """

    def test_record_true_uses_play_and_record_path(self, capsys):
        task = _make_task()
        task.planner = _FakePlanner()
        task._play_and_record(3, True, "clip", "/out", 25)
        assert task.planner.play_and_record_calls == [(3, "clip", "/out", 25)]
        assert task.planner.play_calls == []
        out = capsys.readouterr().out
        assert "Path playback complete" in out
        assert "Video recorded" in out

    def test_record_false_falls_back_to_play_path(self, capsys):
        task = _make_task()
        task.planner = _FakePlanner()
        task._play_and_record(2, False, None, "/out", 25)
        assert task.planner.play_calls == [2]
        assert task.planner.play_and_record_calls == []
        assert "Path playback complete" in capsys.readouterr().out

    def test_missing_record_method_falls_back_to_play_path(self, capsys):
        task = _make_task()
        task.planner = _FakePlanner(has_record_method=False)
        task._play_and_record(0, True, None, "/out", 25)
        assert task.planner.play_calls == [0]

    def test_exception_is_caught_not_raised(self, capsys):
        task = _make_task()
        task.planner = _FakePlanner(raise_on="record")
        task._play_and_record(0, True, None, "/out", 25)  # must not raise
        assert "Path playback failed" in capsys.readouterr().out
