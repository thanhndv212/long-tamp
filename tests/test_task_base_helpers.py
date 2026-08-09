"""
Unit tests for pure-logic helpers extracted from ManipulationTask.run().

Importing agimus_spacelab requires pyhpp (see
docs/plans/refactor-manipulation-task-run.md's verification-model
correction) even though the helpers under test have no HPP dependency
themselves, so these tests must run inside the hpp-arm64 container.
"""

import logging

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

    def test_record_true_uses_play_and_record_path(self, caplog):
        task = _make_task()
        task.planner = _FakePlanner()
        with caplog.at_level(logging.INFO, logger="agimus_spacelab"):
            task._play_and_record(3, True, "clip", "/out", 25)
        assert task.planner.play_and_record_calls == [(3, "clip", "/out", 25)]
        assert task.planner.play_calls == []
        assert "Path playback complete" in caplog.text
        assert "Video recorded" in caplog.text

    def test_record_false_falls_back_to_play_path(self, caplog):
        task = _make_task()
        task.planner = _FakePlanner()
        with caplog.at_level(logging.INFO, logger="agimus_spacelab"):
            task._play_and_record(2, False, None, "/out", 25)
        assert task.planner.play_calls == [2]
        assert task.planner.play_and_record_calls == []
        assert "Path playback complete" in caplog.text

    def test_missing_record_method_falls_back_to_play_path(self, capsys):
        task = _make_task()
        task.planner = _FakePlanner(has_record_method=False)
        task._play_and_record(0, True, None, "/out", 25)
        assert task.planner.play_calls == [0]

    def test_exception_is_caught_not_raised(self, caplog):
        task = _make_task()
        task.planner = _FakePlanner(raise_on="record")
        with caplog.at_level(logging.WARNING, logger="agimus_spacelab"):
            task._play_and_record(0, True, None, "/out", 25)  # must not raise
        assert "Path playback failed" in caplog.text


class _FakePlannerWithViewer:
    def __init__(self, viewer):
        self.viewer = viewer


class TestBuildResult:
    def test_base_fields_no_extra(self):
        task = _make_task()
        task.planner = _FakePlannerWithViewer(viewer="the-viewer")
        task.robot = "the-robot"
        task.ps = "the-ps"
        task.graph = "the-graph"
        result = task._build_result({"q_init": [0.0]})
        assert result == {
            "configs": {"q_init": [0.0]},
            "planner": task.planner,
            "robot": "the-robot",
            "ps": "the-ps",
            "graph": "the-graph",
            "viewer": "the-viewer",
        }

    def test_extra_kwargs_are_merged(self):
        task = _make_task()
        task.planner = _FakePlannerWithViewer(viewer=None)
        task.robot = None
        task.ps = None
        task.graph = None
        result = task._build_result({}, path_id=7, solve_mode="transition-planner")
        assert result["path_id"] == 7
        assert result["solve_mode"] == "transition-planner"

    def test_viewer_is_none_when_planner_is_none(self):
        task = _make_task()
        task.planner = None
        task.robot = None
        task.ps = None
        task.graph = None
        result = task._build_result({})
        assert result["viewer"] is None


class _FakeTaskConfig:
    """Minimal attribute-bag standing in for a task_config object.

    Only the fields the configure_* helpers look up are set; absent fields
    are intentionally missing so hasattr() returns False, mirroring a real
    config that doesn't define them.
    """

    def __init__(self, **attrs):
        for k, v in attrs.items():
            setattr(self, k, v)


class _RecordingPlanner:
    """Planner double that records configure_* calls and their kwargs."""

    def __init__(self, has_optimizer=True, has_time_param=True):
        self.configure_transition_planner_calls = []
        self.configure_time_parameterization_method_calls = []
        if has_optimizer:
            self.configure_transition_planner = self._opt
        if has_time_param:
            self.configure_time_parameterization_method = self._tp

    def _opt(self, **kwargs):
        self.configure_transition_planner_calls.append(kwargs)

    def _tp(self, **kwargs):
        self.configure_time_parameterization_method_calls.append(kwargs)


def _make_setup_task(task_config=None, planner=None, run_logger=None,
                     use_factory=False, q_init=None):
    """Bare ManipulationTask for setup-helper tests.

    The setup helpers read self.task_config / self.planner / self.run_logger /
    self.use_factory / self.q_init / self.ps / self.robot / self.backend /
    self.task_name / self.pyhpp_constraints. Only the attributes a given test
    touches need to be set.
    """
    task = object.__new__(_ConcreteTask)
    task.task_config = task_config
    task.planner = planner
    task.run_logger = run_logger
    task.use_factory = use_factory
    task.q_init = q_init
    task.backend = "pyhpp"
    task.task_name = "test-task"
    return task


class TestApplyOptimizerConfig:
    def test_forwards_present_fields_as_kwargs(self):
        cfg = _FakeTaskConfig(
            RANDOM_SHORTCUT_LOOPS=50,
            SPLINE_ZERO_DERIVATIVES_AT_STATE=True,
        )
        planner = _RecordingPlanner()
        task = _make_setup_task(task_config=cfg, planner=planner)
        task._apply_optimizer_config()
        assert planner.configure_transition_planner_calls == [
            {"random_shortcut_loops": 50, "spline_zero_derivatives_at_state": True}
        ]

    def test_no_fields_means_no_call(self):
        cfg = _FakeTaskConfig()  # neither field present
        planner = _RecordingPlanner()
        task = _make_setup_task(task_config=cfg, planner=planner)
        task._apply_optimizer_config()
        assert planner.configure_transition_planner_calls == []

    def test_none_task_config_is_noop(self):
        planner = _RecordingPlanner()
        task = _make_setup_task(task_config=None, planner=planner)
        task._apply_optimizer_config()
        assert planner.configure_transition_planner_calls == []

    def test_planner_without_method_is_noop(self):
        cfg = _FakeTaskConfig(RANDOM_SHORTCUT_LOOPS=50)
        planner = _RecordingPlanner(has_optimizer=False)
        task = _make_setup_task(task_config=cfg, planner=planner)
        task._apply_optimizer_config()  # must not raise
        assert not hasattr(planner, "configure_transition_planner")


class TestApplyTimeParameterizationConfig:
    def test_forwards_present_fields_as_kwargs(self):
        cfg = _FakeTaskConfig(
            TIME_PARAM_METHOD="toppra",
            TOPPRA_VELOCITY_SCALE=0.9,
            TOPPRA_N=501,
        )
        planner = _RecordingPlanner()
        task = _make_setup_task(task_config=cfg, planner=planner)
        task._apply_time_parameterization_config()
        assert planner.configure_time_parameterization_method_calls == [
            {"method": "toppra", "toppra_velocity_scale": 0.9, "toppra_N": 501}
        ]

    def test_no_fields_means_no_call(self):
        cfg = _FakeTaskConfig()
        planner = _RecordingPlanner()
        task = _make_setup_task(task_config=cfg, planner=planner)
        task._apply_time_parameterization_config()
        assert planner.configure_time_parameterization_method_calls == []

    def test_none_task_config_is_noop(self):
        planner = _RecordingPlanner()
        task = _make_setup_task(task_config=None, planner=planner)
        task._apply_time_parameterization_config()
        assert planner.configure_time_parameterization_method_calls == []

    def test_planner_without_method_is_noop(self):
        cfg = _FakeTaskConfig(TIME_PARAM_METHOD="trapezoidal")
        planner = _RecordingPlanner(has_time_param=False)
        task = _make_setup_task(task_config=cfg, planner=planner)
        task._apply_time_parameterization_config()  # must not raise
        assert not hasattr(planner, "configure_time_parameterization_method")


class _FakeRunLogger:
    def __init__(self, raise_on_log=False):
        self.log_calls = []
        self._raise_on_log = raise_on_log

    def log_task_config(self, **kwargs):
        if self._raise_on_log:
            raise RuntimeError("logger exploded")
        self.log_calls.append(kwargs)


class TestLogSetupSnapshot:
    def test_no_logger_is_noop(self):
        task = _make_setup_task(run_logger=None)
        task._log_setup_snapshot({"validation_step": 0.01})  # must not raise

    def test_logger_called_with_params(self):
        logger = _FakeRunLogger()
        task = _make_setup_task(run_logger=logger)
        task.backend = "pyhpp"
        task.task_name = "tn"
        task.task_config = _FakeTaskConfig()
        params = {"validation_step": 0.01, "skip_graph": False}
        task._log_setup_snapshot(params)
        assert len(logger.log_calls) == 1
        call = logger.log_calls[0]
        assert call["setup_params"] == params
        assert call["backend"] == "pyhpp"
        assert call["task_name"] == "tn"

    def test_logger_exception_is_swallowed(self, capsys):
        logger = _FakeRunLogger(raise_on_log=True)
        task = _make_setup_task(run_logger=logger)
        task.backend = "pyhpp"
        task.task_name = "tn"
        task.task_config = _FakeTaskConfig()
        task._log_setup_snapshot({})  # must not raise


class TestSetupLockedJointConstraints:
    """Covers the pattern-resolution + ConstraintBuilder dispatch.

    ConstraintBuilder.create_locked_joint_constraints is monkeypatched so no
    real HPP/ps/robot is needed.
    """

    def test_explicit_patterns_return_constraints(self, monkeypatch, caplog):
        captured = {}

        def fake_create(ps, robot, q_ref, patterns, backend):
            captured["patterns"] = patterns
            return (["locked::j1", "locked::j2"], ["j1", "j2"])

        monkeypatch.setattr(
            "agimus_spacelab.tasks.base.ConstraintBuilder"
            ".create_locked_joint_constraints",
            staticmethod(fake_create),
        )
        task = _make_setup_task(q_init=[0.0, 0.0])
        task.ps = "ps"
        task.robot = "robot"
        with caplog.at_level(logging.INFO, logger="agimus_spacelab"):
            result = task._setup_locked_joint_constraints(["j1", "j2"])
        assert result == ["locked::j1", "locked::j2"]
        assert captured["patterns"] == ["j1", "j2"]
        assert "✓ Created locked joint constraints: j1, j2" in caplog.text

    def test_empty_frozen_names_returns_none(self, monkeypatch):
        monkeypatch.setattr(
            "agimus_spacelab.tasks.base.ConstraintBuilder"
            ".create_locked_joint_constraints",
            staticmethod(lambda *a, **k: (["c"], [])),  # frozen_names empty
        )
        task = _make_setup_task(q_init=[0.0])
        task.ps = "ps"
        task.robot = "robot"
        assert task._setup_locked_joint_constraints(["j1"]) is None

    def test_no_patterns_returns_none(self):
        task = _make_setup_task(use_factory=False)
        # patterns=None, use_factory False -> no task_config fallback, no call
        assert task._setup_locked_joint_constraints(None) is None

    def test_patterns_from_task_config_when_factory_and_none_arg(
        self, monkeypatch
    ):
        captured = {}

        def fake_create(ps, robot, q_ref, patterns, backend):
            captured["patterns"] = patterns
            return (["c::x"], ["x"])

        monkeypatch.setattr(
            "agimus_spacelab.tasks.base.ConstraintBuilder"
            ".create_locked_joint_constraints",
            staticmethod(fake_create),
        )
        cfg = _FakeTaskConfig(FREEZE_JOINT_SUBSTRINGS=["x"])
        task = _make_setup_task(task_config=cfg, use_factory=True, q_init=[0.0])
        task.ps = "ps"
        task.robot = "robot"
        result = task._setup_locked_joint_constraints(None)
        assert result == ["c::x"]
        assert captured["patterns"] == ["x"]

    def test_no_q_init_skips_constraint_creation(self):
        # q_ref falsy -> the inner if q_ref: block is skipped
        task = _make_setup_task(q_init=None)
        task.ps = "ps"
        task.robot = "robot"
        assert task._setup_locked_joint_constraints(["j1"]) is None
