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
    """disable_collisions_between_subtrees() dispatches on self.backend to
    _disable_collisions_pyhpp / _disable_collisions_corba (Phase 3 Step 3.1
    extraction) -- verify the dispatch wiring itself, since no real script
    in this environment reaches this method end-to-end (the only caller,
    script/graspball/task_graspball_inbox.py, is blocked at setup() on a
    missing hpp_practicals package)."""

    def test_pyhpp_backend_dispatches_to_pyhpp_helper(self):
        sb = _make_scene_builder("pyhpp")
        sb._disable_collisions_pyhpp = MagicMock(return_value=sb)
        sb._disable_collisions_corba = MagicMock(return_value=sb)

        result = sb.disable_collisions_between_subtrees(
            "robot_joint", "obstacle/root_joint", verbose=True, max_pairs=10
        )

        sb._disable_collisions_pyhpp.assert_called_once_with(
            "robot_joint", "obstacle/root_joint", True, 10
        )
        sb._disable_collisions_corba.assert_not_called()
        assert result is sb

    def test_corba_backend_dispatches_to_corba_helper(self):
        sb = _make_scene_builder("corba")
        sb._disable_collisions_pyhpp = MagicMock(return_value=sb)
        sb._disable_collisions_corba = MagicMock(return_value=sb)

        result = sb.disable_collisions_between_subtrees(
            "robot_joint",
            "obstacle/root_joint",
            remove_collision=False,
            remove_distance=True,
            verbose=False,
            max_pairs=5,
        )

        sb._disable_collisions_corba.assert_called_once_with(
            "robot_joint", "obstacle/root_joint", False, True, False, 5
        )
        sb._disable_collisions_pyhpp.assert_not_called()
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
    backend dispatch, verified against mocked graph/planner."""

    def test_corba_backend_uses_plain_sequences(self):
        cg = _make_config_generator("corba")
        cg.planner.random_config.return_value = [0.1, 0.2]
        cg.graph.generateTargetConfig.return_value = (True, [0.3, 0.4], "ok")

        success, config, err = cg._generate_candidate_config(
            "edge01", [0.0, 0.0], None, False
        )

        assert success is True
        assert config == [0.3, 0.4]
        assert err == "ok"
        call_args = cg.graph.generateTargetConfig.call_args[0]
        assert call_args[0] == "edge01"
        assert call_args[1] == [0.0, 0.0]
        assert call_args[2] == [0.1, 0.2]

    def test_corba_backend_uses_hint_on_first_attempt(self):
        cg = _make_config_generator("corba")
        cg.graph.generateTargetConfig.return_value = (True, [9.0], "ok")

        cg._generate_candidate_config("edge01", [0.0], [9.0], True)

        cg.planner.random_config.assert_not_called()
        call_args = cg.graph.generateTargetConfig.call_args[0]
        assert call_args[2] == [9.0]

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
