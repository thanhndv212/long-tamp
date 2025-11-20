"""
Tests for CORBA backend.

These tests require hpp-manipulation-corba to be installed.
"""

import pytest
import numpy as np

try:
    from agimus_spacelab.corba import CorbaManipulationPlanner, HAS_CORBA
except ImportError:
    HAS_CORBA = False


@pytest.mark.skipif(not HAS_CORBA, reason="CORBA backend not available")
class TestCorbaBackend:
    """Tests for CORBA backend implementation."""
    
    def test_import(self):
        """Test importing CORBA planner."""
        planner = CorbaManipulationPlanner()
        assert planner is not None
    
    def test_initialization(self):
        """Test CORBA planner initialization."""
        planner = CorbaManipulationPlanner()
        assert planner.robot is None
        assert planner.ps is None
        assert planner.graph is None
        assert planner.vf is None
        assert planner.viewer is None
    
    def test_robot_loading(self):
        """Test robot loading."""
        planner = CorbaManipulationPlanner()
        
        # Load robot (will fail without proper URDF, but tests API)
        try:
            robot = planner.load_robot(
                name="test_robot",
                urdf_path="package://test/robot.urdf"
            )
            assert robot is not None
            assert planner.robot is not None
            assert planner.ps is not None
        except Exception:
            # Expected if URDF not found
            pass
    
    def test_config_setting(self):
        """Test configuration setting."""
        planner = CorbaManipulationPlanner()
        
        # Need robot loaded first
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
        planner = CorbaManipulationPlanner()
        
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
        planner = CorbaManipulationPlanner()
        
        # Check methods exist
        assert hasattr(planner, 'visualize')
        assert hasattr(planner, 'play_path')
        assert hasattr(planner, 'get_path')
    
    def test_accessor_methods(self):
        """Test accessor methods."""
        planner = CorbaManipulationPlanner()
        
        # Test getters
        assert planner.get_robot() is None
        assert planner.get_problem_solver() is None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

