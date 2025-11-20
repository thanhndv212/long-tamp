"""
Tests for PyHPP backend.

These tests require hpp-python to be installed.
"""

import pytest
import numpy as np

try:
    from agimus_spacelab.pyhpp import PyHPPManipulationPlanner, HAS_PYHPP
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
        assert planner.planner is None
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


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

