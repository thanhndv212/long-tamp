"""
PyHPP manipulation planner implementation.
"""

from typing import Any, Dict, List, Optional
import numpy as np
from pinocchio import SE3

try:
    from pyhpp.manipulation import (
        Device,
        urdf,
        Graph,
        ManipulationPlanner as HPPManipulationPlanner,
        Problem,
    )
    from pyhpp.gepetto.viewer import Viewer
    HAS_PYHPP = True
except ImportError:
    HAS_PYHPP = False


class PyHPPManipulationPlanner:
    """PyHPP backend implementation for manipulation planning."""
    
    def __init__(self):
        """Initialize PyHPP planner."""
        if not HAS_PYHPP:
            raise ImportError(
                "PyHPP backend not available. "
                "Please install hpp-python."
            )
        
        self.device = None
        self.problem = None
        self.graph = None
        self.planner = None
        self.viewer = None
        self.path = None
        
    def load_robot(
        self,
        name: str,
        urdf_path: str,
        srdf_path: Optional[str] = None,
        root_joint_type: str = "anchor"
    ):
        """Load robot using PyHPP."""
        self.device = Device(name)
        
        urdf.loadModel(
            self.device,
            0,
            name,
            root_joint_type,
            urdf_path,
            srdf_path or "",
            SE3.Identity()
        )
        
        # Create problem
        self.problem = Problem(self.device)
        
        return self.device
    
    def load_environment(self, name: str, urdf_path: str):
        """Load environment model."""
        if self.device is None:
            raise RuntimeError("Must load robot first")
        
        urdf.loadModel(
            self.device,
            0,
            name,
            "anchor",
            urdf_path,
            "",
            SE3.Identity()
        )
        
        return name
    
    def load_object(
        self,
        name: str,
        urdf_path: str,
        root_joint_type: str = "freeflyer"
    ):
        """Load manipulable object."""
        if self.device is None:
            raise RuntimeError("Must load robot first")
        
        urdf.loadModel(
            self.device,
            0,
            name,
            root_joint_type,
            urdf_path,
            "",
            SE3.Identity()
        )
        
        return name
    
    def set_joint_bounds(self, joint_name: str, bounds: List[float]):
        """Set joint bounds."""
        if self.device is None:
            raise RuntimeError("Must load robot first")
        
        self.device.setJointBounds(joint_name, bounds)
    
    def set_initial_config(self, q: np.ndarray):
        """Set initial configuration."""
        if self.problem is None:
            raise RuntimeError("Must create problem first")
        
        self.problem.initConfig(q)
    
    def add_goal_config(self, q: np.ndarray):
        """Add goal configuration."""
        if self.problem is None:
            raise RuntimeError("Must create problem first")
        
        self.problem.addGoalConfig(q)
    
    def create_constraint_graph(
        self,
        name: str,
        grippers: List[str],
        objects: Dict[str, Dict],
        rules: str = "auto",
        **kwargs
    ) -> Graph:
        """Create constraint graph using PyHPP."""
        if self.problem is None:
            raise RuntimeError("Must create problem first")
        
        # Note: PyHPP requires manual graph construction
        # This is a simplified version
        self.graph = Graph(name, self.device, self.problem)
        
        # Create basic states
        free_state = self.graph.createState("free", False, 0)
        
        # For each object, create grasp states
        for obj_name in objects:
            state_name = f"grasp_{obj_name}"
            self.graph.createState(state_name, False, 0)
        
        # Initialize graph
        self.graph.maxIterations(100)
        self.graph.errorThreshold(1e-5)
        self.graph.initialize()
        
        # Set graph in problem
        self.problem.constraintGraph(self.graph)
        
        return self.graph
    
    def solve(self, max_iterations: int = 10000) -> bool:
        """Solve planning problem."""
        if self.problem is None:
            raise RuntimeError("Must create problem first")
        
        try:
            self.planner = HPPManipulationPlanner(self.problem)
            self.planner.maxIterations(max_iterations)
            success = self.planner.solve()
            
            if success:
                self.path = self.planner.path()
            
            return success
        except Exception as e:
            print(f"Planning failed: {e}")
            return False
    
    def get_path(self) -> Optional[Any]:
        """Get computed path."""
        return self.path
    
    def visualize(self, q: Optional[np.ndarray] = None):
        """Visualize configuration."""
        if self.viewer is None:
            if self.device is None:
                raise RuntimeError("Must load robot first")
            self.viewer = Viewer(self.device)
        
        if q is not None:
            self.viewer(q)
        else:
            # Display current config or initial
            try:
                q_init = self.problem.initConfig()
                self.viewer(q_init)
            except:
                pass
    
    def play_path(self, path_index: int = 0):
        """Play path in viewer."""
        if self.path is None:
            print("No path to play")
            return
        
        if self.viewer is None:
            self.visualize()
        
        # Animate path
        import time
        t = 0.0
        dt = 0.01
        length = self.path.length()
        
        while t <= length:
            q = self.path(t)
            self.viewer(q)
            time.sleep(dt)
            t += dt
    
    def get_robot(self):
        """Get device object."""
        return self.device
    
    def get_problem(self):
        """Get problem object."""
        return self.problem
    
    def get_graph(self):
        """Get constraint graph."""
        return self.graph


__all__ = [
    "PyHPPManipulationPlanner",
    "HAS_PYHPP",
]
