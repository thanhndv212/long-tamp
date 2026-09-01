"""
Unit tests for pure-logic/dispatch helpers extracted from planning/ modules
as part of the Phase 3 mid-size-hotspot refactor.

Importing agimus_spacelab requires pyhpp even for helpers with no HPP
dependency themselves (see docs/plans/refactor-codebase.md's Phase 2
verification-model note), so these tests must run inside the hpp-arm64
container.
"""

import numpy as np
from unittest.mock import MagicMock

from agimus_spacelab.planning.config import ConfigGenerator
from agimus_spacelab.planning.graph import GraphBuilder
from agimus_spacelab.planning.scene import SceneBuilder


def _make_scene_builder(backend):
    """Bare SceneBuilder instance, bypassing __init__ (which requires
    FILE_PATHS/joint_bounds and constructs a real backend planner)."""
    sb = object.__new__(SceneBuilder)
    sb.backend = backend
    sb.planner = MagicMock()
    return sb


def _make_config_generator(backend):
    """Bare ConfigGenerator instance, bypassing __init__ (which requires
    real robot/graph/planner/ps instances)."""
    cg = object.__new__(ConfigGenerator)
    cg.backend = backend
    cg.robot = MagicMock()
    cg.graph = MagicMock()
    cg.planner = MagicMock()
    cg.ps = MagicMock()
    cg.configs = {}
    return cg


class TestDisableCollisionsDispatch:
    """disable_collisions_between_subtrees() dispatches to
    _disable_collisions_pyhpp (Phase 3 Step 3.1 extraction) -- verify the
    dispatch wiring itself, since no example script in this environment
    currently reaches this method end-to-end."""

    def test_dispatches_to_pyhpp_helper(self):
        sb = _make_scene_builder("pyhpp")
        sb._disable_collisions_pyhpp = MagicMock(return_value=sb)

        result = sb.disable_collisions_between_subtrees(
            "robot_joint", "obstacle/root_joint", verbose=True, max_pairs=10
        )

        sb._disable_collisions_pyhpp.assert_called_once_with(
            "robot_joint", "obstacle/root_joint", True, 10
        )
        assert result is sb

    def test_pyhpp_helper_no_pinocchio_geometry_returns_self_with_warning(self):
        """_disable_collisions_pyhpp: byte-identical-relocation smoke test --
        no geometry found for the robot side should warn and return self
        without attempting any CollisionPair removal, same as the original
        inline code."""
        sb = _make_scene_builder("pyhpp")
        device = MagicMock()
        model = MagicMock()
        model.existJointName.return_value = False
        model.existFrame.return_value = False
        geom_model = MagicMock()
        geom_model.geometryObjects = []
        device.model.return_value = model
        device.geomModel.return_value = geom_model
        sb.planner.device = device

        result = sb._disable_collisions_pyhpp(
            "no/such/robot_thing", "no/such/obstacle_thing", False, 80
        )

        assert result is sb


class TestCheckConfigFinite:
    """_check_config_finite() (Phase 3 Step 3.2 extraction from
    generate_via_edge()) -- pure logic, no HPP calls needed."""

    def test_none_config_is_not_finite(self, capsys):
        cg = _make_config_generator("pyhpp")
        assert cg._check_config_finite(None, "edge01", 0) is False

    def test_all_finite_values_pass(self, capsys):
        cg = _make_config_generator("pyhpp")
        assert cg._check_config_finite([0.0, 1.0, -2.5], "edge01", 0) is True

    def test_nan_value_fails(self, capsys):
        cg = _make_config_generator("pyhpp")
        assert cg._check_config_finite([0.0, float("nan"), 1.0], "edge01", 0) is False

    def test_inf_value_fails(self, capsys):
        cg = _make_config_generator("pyhpp")
        assert cg._check_config_finite([0.0, float("inf"), 1.0], "edge01", 0) is False


class TestGenerateCandidateConfig:
    """_generate_candidate_config() (Phase 3 Step 3.2 extraction) --
    verified against mocked graph/planner."""

    def test_pyhpp_backend_returns_list_config_on_success(self):
        cg = _make_config_generator("pyhpp")
        cg.robot.rankInConfiguration = {}
        cg.planner.random_config.return_value = np.array([0.1, 0.2])
        cg.graph.generateTargetConfig.return_value = (
            True,
            np.array([0.3, 0.4]),
            None,
        )

        success, config, err = cg._generate_candidate_config(
            "edge01", [0.0, 0.0], None, False
        )

        assert success is True
        assert config == [0.3, 0.4]

    def test_pyhpp_backend_failure_returns_none_config(self):
        cg = _make_config_generator("pyhpp")
        cg.robot.rankInConfiguration = {}
        cg.planner.random_config.return_value = np.array([0.1, 0.2])
        cg.graph.generateTargetConfig.return_value = (False, None, "diverged")

        success, config, err = cg._generate_candidate_config(
            "edge01", [0.0, 0.0], None, False
        )

        assert success is False
        assert config is None
        assert err == "diverged"


class TestBuildPhaseValidPairs:
    """_build_phase_valid_pairs() (Phase 3 Step 3.4 extraction from
    build_phase_graph()) -- pure function, no HPP calls needed."""

    def test_empty_held_grasps_new_grasp(self):
        result = GraphBuilder._build_phase_valid_pairs({}, ("g1", "h1"))
        assert result == {"g1": ["h1"]}

    def test_held_grasps_preserved_plus_new_grasp(self):
        result = GraphBuilder._build_phase_valid_pairs(
            {"g1": "h1"}, ("g2", "h2")
        )
        assert result == {"g1": ["h1"], "g2": ["h2"]}

    def test_release_includes_currently_held_handle(self):
        result = GraphBuilder._build_phase_valid_pairs(
            {"g1": "h1"}, ("g1", None)
        )
        assert result == {"g1": ["h1"]}

    def test_release_with_nothing_held_omits_gripper(self):
        result = GraphBuilder._build_phase_valid_pairs({}, ("g1", None))
        assert result == {}

    def test_duplicate_handle_not_added_twice(self):
        result = GraphBuilder._build_phase_valid_pairs(
            {"g1": "h1"}, ("g1", "h1")
        )
        assert result == {"g1": ["h1"]}


class TestLockNonphaseObjects:
    """_lock_nonphase_objects() (Phase 3 Step 3.4 extraction)."""

    def _make_graph_builder(self, backend="pyhpp"):
        gb = object.__new__(GraphBuilder)
        gb.backend = backend
        gb.ps = MagicMock()
        gb.robot = MagicMock()
        return gb

    def test_no_nonphase_objects_returns_constraints_unchanged(self):
        gb = self._make_graph_builder()
        result = gb._lock_nonphase_objects(
            ["obj1"], ["obj1"], [0.0], ["existing"]
        )
        assert result == ["existing"]

    def test_no_q_init_returns_constraints_unchanged(self):
        gb = self._make_graph_builder()
        result = gb._lock_nonphase_objects(["obj1", "obj2"], ["obj1"], None, None)
        assert result is None

    def test_locks_nonphase_objects_and_extends_constraints(self, monkeypatch):
        gb = self._make_graph_builder()
        fake_builder = MagicMock()
        fake_builder.create_locked_joint_constraints.return_value = (
            ["lock_c1"],
            ["obj2/root_joint"],
        )
        monkeypatch.setattr(
            "agimus_spacelab.planning.constraints.ConstraintBuilder",
            fake_builder,
        )

        result = gb._lock_nonphase_objects(
            ["obj1", "obj2"], ["obj1"], [0.0], ["existing"]
        )

        assert result == ["existing", "lock_c1"]

    def test_locking_exception_returns_constraints_unchanged(self, monkeypatch):
        gb = self._make_graph_builder()
        fake_builder = MagicMock()
        fake_builder.create_locked_joint_constraints.side_effect = RuntimeError(
            "boom"
        )
        monkeypatch.setattr(
            "agimus_spacelab.planning.constraints.ConstraintBuilder",
            fake_builder,
        )

        result = gb._lock_nonphase_objects(
            ["obj1", "obj2"], ["obj1"], [0.0], ["existing"]
        )

        assert result == ["existing"]


class TestApplySequentialFilter:
    """_apply_sequential_filter() (Phase 3 Step 3.4 extraction)."""

    def _make_graph_builder(self):
        gb = object.__new__(GraphBuilder)
        gb.backend = "pyhpp"
        return gb

    def test_import_error_returns_false(self, monkeypatch):
        import builtins

        real_import = builtins.__import__

        def fake_import(name, *args, **kwargs):
            if name == "agimus_spacelab.planning.sequential_grasp_filter":
                raise ImportError("not available")
            return real_import(name, *args, **kwargs)

        monkeypatch.setattr(builtins, "__import__", fake_import)

        gb = self._make_graph_builder()
        phase_config = MagicMock()
        phase_config.GRIPPERS = ["g1"]
        phase_config.HANDLES_PER_OBJECT = [["h1"]]

        result = gb._apply_sequential_filter(phase_config, {}, ("g1", "h1"))

        assert result is False

    def test_success_attaches_filter_and_returns_true(self):
        gb = self._make_graph_builder()
        phase_config = MagicMock()
        phase_config.GRIPPERS = ["g1"]
        phase_config.HANDLES_PER_OBJECT = [["h1"]]

        result = gb._apply_sequential_filter(phase_config, {}, ("g1", "h1"))

        assert result is True
        assert phase_config._SEQUENTIAL_FILTER is not None
