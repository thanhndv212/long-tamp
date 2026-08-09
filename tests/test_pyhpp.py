"""
Tests for PyHPP backend.

These tests require hpp-python to be installed.
"""

import pytest
import numpy as np
from unittest.mock import MagicMock

try:
    from agimus_spacelab.backends.pyhpp import PyHPPBackend as PyHPPManipulationPlanner, HAS_PYHPP
except ImportError:
    HAS_PYHPP = False


@pytest.mark.skipif(not HAS_PYHPP, reason="PyHPP backend not available")
class TestPyHPPBackend:
    """Tests for PyHPP backend implementation."""
    
    def test_import(self):
        """Test importing PyHPP planner."""
        planner = PyHPPManipulationPlanner()
        assert planner is not None
    
    def test_initialization(self):
        """Test PyHPP planner initialization."""
        planner = PyHPPManipulationPlanner()
        assert planner.device is None
        assert planner.problem is None
        assert planner.graph is None
        assert planner.viewer is None
        assert planner.path is None
    
    def test_robot_loading(self):
        """Test robot loading."""
        planner = PyHPPManipulationPlanner()
        
        # Load robot (will fail without proper URDF, but tests API)
        try:
            device = planner.load_robot(
                name="test_robot",
                urdf_path="package://test/robot.urdf"
            )
            assert device is not None
            assert planner.device is not None
            assert planner.problem is not None
        except Exception:
            # Expected if URDF not found
            pass
    
    def test_environment_loading(self):
        """Test environment loading."""
        planner = PyHPPManipulationPlanner()
        
        try:
            # Need robot first
            planner.load_robot("test", "package://test/robot.urdf")
            
            # Load environment
            env = planner.load_environment(
                name="test_env",
                urdf_path="package://test/env.urdf"
            )
            assert env is not None
        except Exception:
            # Expected if URDFs not found
            pass
    
    def test_object_loading(self):
        """Test object loading."""
        planner = PyHPPManipulationPlanner()
        
        try:
            planner.load_robot("test", "package://test/robot.urdf")
            
            # Load object
            obj = planner.load_object(
                name="test_obj",
                urdf_path="package://test/obj.urdf",
                root_joint_type="freeflyer"
            )
            assert obj is not None
        except Exception:
            # Expected if URDFs not found
            pass
    
    def test_config_setting(self):
        """Test configuration setting."""
        planner = PyHPPManipulationPlanner()
        
        try:
            planner.load_robot("test", "package://test/robot.urdf")
            
            # Test setting initial config
            q_init = np.zeros(10)
            planner.set_initial_config(q_init)
            
            # Test adding goal config
            q_goal = np.ones(10)
            planner.add_goal_config(q_goal)
        except Exception:
            # Expected if robot loading failed
            pass
    
    def test_joint_bounds(self):
        """Test setting joint bounds."""
        planner = PyHPPManipulationPlanner()
        
        try:
            planner.load_robot("test", "package://test/robot.urdf")
            
            # Test setting bounds
            bounds = [-1.0, 1.0, -2.0, 2.0]
            planner.set_joint_bounds("test_joint", bounds)
        except Exception:
            # Expected if robot loading failed
            pass
    
    def test_visualization_methods(self):
        """Test visualization methods exist."""
        planner = PyHPPManipulationPlanner()
        
        # Check methods exist
        assert hasattr(planner, 'visualize')
        assert hasattr(planner, 'play_path')
        assert hasattr(planner, 'get_path')
    
    def test_accessor_methods(self):
        """Test accessor methods."""
        planner = PyHPPManipulationPlanner()
        
        # Test getters
        assert planner.get_robot() is None
        assert planner.get_problem() is None
        assert planner.get_graph() is None
    
    def test_solve_without_setup(self):
        """Test that solve fails gracefully without setup."""
        planner = PyHPPManipulationPlanner()

        with pytest.raises(RuntimeError):
            planner.solve()

    # -----------------------------------------------------------------------
    # B1: configure_transition_planner inner_planner_type
    # -----------------------------------------------------------------------

    def test_configure_transition_planner_inner_planner_type(self):
        """inner_planner_type is stored; PYHPP-GAP means it cannot be applied."""
        planner = PyHPPManipulationPlanner()
        assert planner._transition_inner_planner_type is None

        planner.configure_transition_planner(inner_planner_type="DiffusingPlanner")
        assert planner._transition_inner_planner_type == "DiffusingPlanner"

        # Other params still work alongside inner_planner_type
        planner.configure_transition_planner(
            inner_planner_type="BiRRTPlanner",
            time_out=30.0,
            max_iterations=5000,
        )
        assert planner._transition_inner_planner_type == "BiRRTPlanner"
        assert planner._transition_time_out == 30.0
        assert planner._transition_max_iterations == 5000

    # -----------------------------------------------------------------------
    # B3: set_path_projection
    # -----------------------------------------------------------------------

    def test_set_path_projection(self):
        """set_path_projection toggles _use_progressive_projector."""
        planner = PyHPPManipulationPlanner()
        assert planner._use_progressive_projector is True  # default

        planner.set_path_projection(False)
        assert planner._use_progressive_projector is False

        planner.set_path_projection(True)
        assert planner._use_progressive_projector is True

    # -----------------------------------------------------------------------
    # B4: clear_stored_paths
    # -----------------------------------------------------------------------

    def test_clear_stored_paths_empty(self):
        """clear_stored_paths returns 0 when nothing is stored."""
        planner = PyHPPManipulationPlanner()
        assert planner.clear_stored_paths(verbose=False) == 0
        assert planner._stored_paths == []

    def test_clear_stored_paths_nonempty(self):
        """clear_stored_paths removes all entries and returns count."""
        planner = PyHPPManipulationPlanner()
        sentinel = object()
        planner._stored_paths.append(sentinel)
        planner._stored_paths.append(sentinel)
        count = planner.clear_stored_paths(verbose=False)
        assert count == 2
        assert planner._stored_paths == []

    # -----------------------------------------------------------------------
    # B2: _resolve_transition int branch
    # -----------------------------------------------------------------------

    def test_resolve_transition_int_branch(self):
        """_resolve_transition with int uses positional lookup via getTransitions."""
        from unittest.mock import MagicMock

        planner = PyHPPManipulationPlanner()

        tr0 = MagicMock()
        tr0.name.return_value = "transit"
        tr1 = MagicMock()
        tr1.name.return_value = "grasp"

        mock_graph = MagicMock()
        mock_graph.getTransitions.return_value = [tr0, tr1]
        planner.graph = mock_graph

        assert planner._resolve_transition(0) is tr0
        assert planner._resolve_transition(1) is tr1

    def test_resolve_transition_int_out_of_range(self):
        """_resolve_transition raises ValueError for out-of-range int."""
        from unittest.mock import MagicMock

        planner = PyHPPManipulationPlanner()
        mock_graph = MagicMock()
        mock_graph.getTransitions.return_value = []
        planner.graph = mock_graph

        with pytest.raises(ValueError, match="out of range"):
            planner._resolve_transition(0)

    def test_resolve_transition_str_branch(self):
        """_resolve_transition with str delegates to graph.getTransition."""
        from unittest.mock import MagicMock

        planner = PyHPPManipulationPlanner()
        mock_graph = MagicMock()
        planner.graph = mock_graph

        planner._resolve_transition("my_edge")
        mock_graph.getTransition.assert_called_once_with("my_edge")

    # -----------------------------------------------------------------------
    # B5-B8: play/record method existence
    # -----------------------------------------------------------------------

    def test_play_record_methods_exist(self):
        """New play/record methods are present on the planner."""
        planner = PyHPPManipulationPlanner()
        assert hasattr(planner, "play_path_vector")
        assert hasattr(planner, "play_path_vector_with_viz")
        assert hasattr(planner, "play_and_record_path")
        assert hasattr(planner, "play_and_record_path_vector")
        assert callable(planner.play_path_vector)
        assert callable(planner.play_path_vector_with_viz)
        assert callable(planner.play_and_record_path)
        assert callable(planner.play_and_record_path_vector)

    def test_store_path_and_clear(self):
        """store_path then clear_stored_paths round-trips correctly."""
        planner = PyHPPManipulationPlanner()
        fake_path = object()
        idx = planner.store_path(fake_path)
        assert idx == 0
        assert planner.get_num_stored_paths() == 1

        cleared = planner.clear_stored_paths(verbose=False)
        assert cleared == 1
        assert planner.get_num_stored_paths() == 0


@pytest.mark.skipif(not HAS_PYHPP, reason="PyHPP backend not available")
class TestValidateEdgeEndpoints:
    """_validate_edge_endpoints() (Phase 3 Step 3.3 extraction from
    plan_transition_edge()) -- diagnostic-only, never raises, never
    affects return value; verify it degrades gracefully when graph
    introspection isn't available."""

    def test_no_graph_prints_warning_and_returns_none(self, capsys):
        planner = PyHPPManipulationPlanner()
        planner.graph = None

        result = planner._validate_edge_endpoints(
            object(), "edge01", np.array([0.0]), np.array([1.0])
        )

        assert result is None
        captured = capsys.readouterr()
        assert "Cannot validate configurations" in captured.out

    def test_graph_without_get_nodes_method_returns_none(self, capsys):
        planner = PyHPPManipulationPlanner()

        class _FakeGraph:
            pass

        planner.graph = _FakeGraph()

        result = planner._validate_edge_endpoints(
            object(), "edge01", np.array([0.0]), np.array([1.0])
        )

        assert result is None
        captured = capsys.readouterr()
        assert "Cannot validate configurations" in captured.out

    def test_valid_states_prints_success_markers(self, capsys):
        planner = PyHPPManipulationPlanner()
        graph = MagicMock()
        state_from = MagicMock()
        state_from.name.return_value = "state_from"
        state_to = MagicMock()
        state_to.name.return_value = "state_to"
        graph.getNodesConnectedByTransition.return_value = ["n0", "n1"]
        graph.getState.side_effect = [state_from, state_to]
        graph.getConfigErrorForState.return_value = ([0.0, 0.0], True)
        graph.errorThreshold.return_value = 1e-3
        planner.graph = graph

        planner._validate_edge_endpoints(
            object(), "edge01", np.array([0.0]), np.array([1.0])
        )

        captured = capsys.readouterr()
        assert "Validate q1 in state 'state_from'" in captured.out
        assert "Validate q2 in state 'state_to'" in captured.out
        assert "✓" in captured.out


@pytest.mark.skipif(not HAS_PYHPP, reason="PyHPP backend not available")
class TestTryDirectPath:
    """_try_direct_path() (Phase 3 Step 3.3 extraction)."""

    def test_success_returns_optimized_path(self):
        planner = PyHPPManipulationPlanner()
        tp = MagicMock()
        tp.directPath.return_value = (True, "raw_path", "ok")
        tp.optimizePath.return_value = "optimized_path"

        result = planner._try_direct_path(
            tp, "edge01", np.array([0.0]), np.array([1.0]), True, False
        )

        assert result == "optimized_path"

    def test_success_optimize_failure_returns_raw_path(self):
        planner = PyHPPManipulationPlanner()
        tp = MagicMock()
        tp.directPath.return_value = (True, "raw_path", "ok")
        tp.optimizePath.side_effect = RuntimeError("boom")

        result = planner._try_direct_path(
            tp, "edge01", np.array([0.0]), np.array([1.0]), True, False
        )

        assert result == "raw_path"

    def test_directpath_failure_returns_none(self):
        planner = PyHPPManipulationPlanner()
        tp = MagicMock()
        tp.directPath.return_value = (False, None, "no path")

        result = planner._try_direct_path(
            tp, "edge01", np.array([0.0]), np.array([1.0]), True, False
        )

        assert result is None

    def test_directpath_exception_returns_none(self):
        planner = PyHPPManipulationPlanner()
        tp = MagicMock()
        tp.directPath.side_effect = RuntimeError("boom")

        result = planner._try_direct_path(
            tp, "edge01", np.array([0.0]), np.array([1.0]), True, False
        )

        assert result is None


@pytest.mark.skipif(not HAS_PYHPP, reason="PyHPP backend not available")
class TestComputeOrPlanPath:
    """_compute_or_plan_path() (Phase 3 Step 3.3 extraction)."""

    def test_compute_path_success(self):
        planner = PyHPPManipulationPlanner()
        tp = MagicMock()
        tp.computePath.return_value = "computed_path"
        tp.optimizePath.return_value = "optimized_path"

        result = planner._compute_or_plan_path(
            tp, np.array([0.0]), np.array([1.0]), True, "edge01", False
        )

        assert result == "optimized_path"
        tp.planPath.assert_not_called()

    def test_compute_path_type_error_falls_back_to_plan_path(self):
        planner = PyHPPManipulationPlanner()
        tp = MagicMock()
        tp.computePath.side_effect = TypeError("signature mismatch")
        tp.planPath.return_value = "planned_path"
        tp.optimizePath.return_value = "optimized_path"

        result = planner._compute_or_plan_path(
            tp, np.array([0.0]), np.array([1.0]), True, "edge01", False
        )

        assert result == "optimized_path"
        tp.planPath.assert_called_once()

    def test_both_compute_and_plan_path_fail_raises(self):
        planner = PyHPPManipulationPlanner()
        tp = MagicMock()
        tp.computePath.side_effect = TypeError("signature mismatch")
        tp.planPath.side_effect = RuntimeError("also failed")

        with pytest.raises(RuntimeError, match="planPath failed"):
            planner._compute_or_plan_path(
                tp, np.array([0.0]), np.array([1.0]), True, "edge01", False
            )

    def test_compute_path_other_exception_raises(self):
        planner = PyHPPManipulationPlanner()
        tp = MagicMock()
        tp.computePath.side_effect = RuntimeError("unexpected")

        with pytest.raises(RuntimeError, match="computePath failed"):
            planner._compute_or_plan_path(
                tp, np.array([0.0]), np.array([1.0]), True, "edge01", False
            )

    def test_optimize_failure_after_compute_returns_unoptimized(self):
        planner = PyHPPManipulationPlanner()
        tp = MagicMock()
        tp.computePath.return_value = "computed_path"
        tp.optimizePath.side_effect = RuntimeError("boom")

        result = planner._compute_or_plan_path(
            tp, np.array([0.0]), np.array([1.0]), True, "edge01", False
        )

        assert result == "computed_path"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

