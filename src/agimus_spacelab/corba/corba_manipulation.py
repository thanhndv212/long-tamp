"""
CORBA manipulation planner implementation.
"""

from typing import Any, Dict, List, Optional
import numpy as np

try:
    from hpp.corbaserver import loadServerPlugin
    from hpp.corbaserver.manipulation import (
        Client,
        ConstraintGraph,
        ConstraintGraphFactory,
        ProblemSolver,
        Rule,
    )
    from hpp.corbaserver.manipulation.robot import Robot as ParentRobot
    from hpp.gepetto import PathPlayer
    from hpp.gepetto.manipulation import ViewerFactory
    HAS_CORBA = True
except ImportError:
    HAS_CORBA = False


if HAS_CORBA:
    class SpacelabRobot(ParentRobot):
        """Spacelab composite robot wrapper for CORBA."""
        
        packageName = "spacelab_mock_hardware/description"
        urdfName = "allRobots_spacelab_robot"
        urdfSuffix = ""
        srdfSuffix = ""

        def __init__(
            self,
            compositeName: str,
            robotName: str,
            load: bool = True,
            rootJointType: str = "anchor"
        ):
            super().__init__(compositeName, robotName, rootJointType, load)


class CorbaManipulationPlanner:
    """CORBA backend implementation for manipulation planning."""
    
    def __init__(self):
        """Initialize CORBA planner."""
        if not HAS_CORBA:
            raise ImportError(
                "CORBA backend not available. "
                "Please install hpp-manipulation-corba."
            )
        
        # Initialize CORBA server
        loadServerPlugin("corbaserver", "manipulation-corba.so")
        Client().problem.resetProblem()
        
        self.robot = None
        self.ps = None
        self.vf = None
        self.graph = None
        self.factory = None
        self.viewer = None
        self.path_player = None
        
    def load_robot(
        self,
        name: str,
        urdf_path: str,
        srdf_path: Optional[str] = None,
        root_joint_type: str = "anchor"
    ):
        """Load robot using CORBA."""
        self.robot = SpacelabRobot(name, name, load=True)
        self.ps = ProblemSolver(self.robot)
        self.ps.setErrorThreshold(1e-4)
        self.ps.setMaxIterProjection(40)
        return self.robot
    
    def load_environment(self, name: str, urdf_path: str):
        """Load environment model."""
        if self.vf is None:
            self.vf = ViewerFactory(self.ps)
        
        # Create environment config class dynamically
        class EnvConfig:
            rootJointType = "anchor"
            packageName = "spacelab_mock_hardware/description"
            meshPackageName = "spacelab_mock_hardware/description"
            urdfSuffix = ""
            srdfSuffix = ""
        
        # Extract urdf name from path
        urdf_name = name
        EnvConfig.urdfName = urdf_name
        
        self.vf.loadEnvironmentModel(EnvConfig, name)
        return EnvConfig
    
    def load_object(
        self,
        name: str,
        urdf_path: str,
        root_joint_type: str = "freeflyer"
    ):
        """Load manipulable object."""
        if self.vf is None:
            self.vf = ViewerFactory(self.ps)
        
        # Create object config class dynamically
        class ObjConfig:
            packageName = "spacelab_mock_hardware/description"
            meshPackageName = "spacelab_mock_hardware/description"
            urdfSuffix = ""
            srdfSuffix = ""
        
        ObjConfig.rootJointType = root_joint_type
        ObjConfig.urdfName = name
        
        self.vf.loadObjectModel(ObjConfig, name)
        return ObjConfig
    
    def set_joint_bounds(self, joint_name: str, bounds: List[float]):
        """Set joint bounds."""
        self.robot.setJointBounds(joint_name, bounds)
    
    def set_initial_config(self, q: np.ndarray):
        """Set initial configuration."""
        self.ps.setInitialConfig(q.tolist())
    
    def add_goal_config(self, q: np.ndarray):
        """Add goal configuration."""
        self.ps.addGoalConfig(q.tolist())
    
    def create_constraint_graph(
        self,
        name: str,
        grippers: List[str],
        objects: Dict[str, Dict],
        rules: str = "auto",
        **kwargs
    ) -> ConstraintGraph:
        """Create constraint graph using CORBA."""
        self.graph = ConstraintGraph(self.robot, name)
        self.factory = ConstraintGraphFactory(self.graph)
        
        # Configure factory
        self.factory.setGrippers(grippers)
        
        # Prepare object data
        object_names = list(objects.keys())
        handles_per_object = [
            objects[obj]["handles"] for obj in object_names
        ]
        surfaces_per_object = [
            objects[obj].get("contact_surfaces", [])
            for obj in object_names
        ]
        
        self.factory.setObjects(
            object_names,
            handles_per_object,
            surfaces_per_object
        )
        
        # Generate rules based on strategy
        if rules == "all":
            rule_list = [Rule([".*"], [".*"], True)]
        else:
            # Auto rules from valid pairs
            rule_list = self._generate_auto_rules(grippers, objects)
        
        self.factory.setRules(rule_list)
        self.factory.generate()
        
        self.graph.initialize()
        
        return self.graph
    
    def _generate_auto_rules(
        self,
        grippers: List[str],
        objects: Dict[str, Dict]
    ) -> List[Rule]:
        """Generate rules automatically."""
        # Note: ManipulationConfig should be passed from outside
        # This is a fallback for backward compatibility
        try:
            import sys
            from pathlib import Path
            config_dir = Path(__file__).parent.parent.parent.parent.parent / "config"
            sys.path.insert(0, str(config_dir))
            from spacelab_config import ManipulationConfig
        except ImportError:
            raise ImportError(
                "ManipulationConfig not found. Please provide valid_pairs parameter "
                "or ensure spacelab_config.py is in the config directory."
            )
        
        rules = []
        valid_pairs = ManipulationConfig.VALID_PAIRS
        
        for gripper_key, gripper_name in enumerate(grippers):
            # Get short name from full path
            gripper_short = gripper_name.split("/")[-1]
            
            # Find matching key in valid_pairs
            matching_key = None
            for key in valid_pairs:
                if gripper_short in key or key in gripper_name:
                    matching_key = key
                    break
            
            if matching_key:
                allowed_handles = valid_pairs[matching_key]
                for handle in allowed_handles:
                    rules.append(
                        Rule([gripper_name], [handle], True)
                    )
        
        return rules
    
    def solve(self, max_iterations: int = 10000) -> bool:
        """Solve planning problem."""
        try:
            self.ps.solve()
            return True
        except Exception as e:
            print(f"Planning failed: {e}")
            return False
    
    def get_path(self, index: int = 0) -> Optional[Any]:
        """Get computed path."""
        if self.ps is None:
            return None
        
        try:
            return self.ps.client.problem.getPath(index)
        except:
            return None
    
    def visualize(self, q: Optional[np.ndarray] = None):
        """Visualize configuration."""
        if self.viewer is None:
            if self.vf is None:
                self.vf = ViewerFactory(self.ps)
            self.viewer = self.vf.createViewer()
        
        if q is not None:
            self.viewer(q.tolist() if isinstance(q, np.ndarray) else q)
        else:
            # Try to display current config
            try:
                q_init = self.ps.getCurrentConfig()
                self.viewer(q_init)
            except:
                pass
    
    def play_path(self, path_index: int = 0):
        """Play path in viewer."""
        if self.viewer is None:
            self.visualize()
        
        if self.path_player is None:
            self.path_player = PathPlayer(self.viewer)
        
        try:
            self.path_player(path_index)
        except Exception as e:
            print(f"Failed to play path: {e}")
    
    def get_robot(self):
        """Get robot instance."""
        return self.robot
    
    def get_problem_solver(self):
        """Get problem solver."""
        return self.ps


__all__ = [
    "CorbaManipulationPlanner",
    "SpacelabRobot",
    "HAS_CORBA",
]
