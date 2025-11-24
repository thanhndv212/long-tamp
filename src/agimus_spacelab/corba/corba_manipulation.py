"""
CORBA manipulation planner implementation.
"""

from typing import Any, Dict, List, Optional
import numpy as np
from ..utils import parse_package_uri


class ConstraintResult:
    """Result from applying state constraints."""
    
    def __init__(self, success: bool, configuration: np.ndarray, error: float):
        self.success = success
        self.configuration = configuration
        self.error = error


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
        
        # Configuration options for path validation
        self._use_path_optimization = True
        self._use_path_projection = True

    def model(self):
        """Get the Pinocchio model."""
        if self.robot is None:
            raise RuntimeError("Robot not loaded yet")
        return self.robot.client.basic.robot.getModel()

    def data(self):
        """Get the Pinocchio data."""
        if self.robot is None:
            raise RuntimeError("Robot not loaded yet")
        return self.robot.client.basic.robot.getData()

    @property
    def nq(self) -> int:
        """Number of configuration variables."""
        if self.robot is None:
            raise RuntimeError("Robot not loaded yet")
        return self.robot.getConfigSize()

    @property
    def nv(self) -> int:
        """Number of velocity variables."""
        if self.robot is None:
            raise RuntimeError("Robot not loaded yet")
        return self.robot.getNumberDof()

    def neutral_config(self) -> np.ndarray:
        """Get neutral configuration."""
        if self.robot is None:
            raise RuntimeError("Robot not loaded yet")
        return np.array(self.robot.getCurrentConfig())

    def random_config(self) -> np.ndarray:
        """Generate a random configuration."""
        if self.robot is None:
            raise RuntimeError("Robot not loaded yet")
        return np.array(self.robot.shootRandomConfig())
        
    def load_robot(
        self,
        composite_name: str,
        robot_name: str,
        urdf_path: str,
        srdf_path: Optional[str] = None,
        root_joint_type: str = "anchor"
    ):
        """Load robot using CORBA."""

        self.packageName, self.urdfName = parse_package_uri(urdf_path)
        self.urdfSuffix = ""
        self.srdfSuffix = ""

        if HAS_CORBA:
            class Robot(ParentRobot):
                """Spacelab composite robot wrapper for CORBA."""
                packageName = self.packageName
                urdfName = self.urdfName
                urdfSuffix = self.urdfSuffix
                srdfSuffix = self.srdfSuffix

                def __init__(
                    self,
                    compositeName: str,
                    robotName: str,
                    urdf_path: str,
                    srdf_path: Optional[str] = None,
                    load: bool = True,
                    rootJointType: str = "anchor"
                ):
                    ParentRobot.__init__(
                        self, compositeName, robotName, rootJointType, load
                    )
        self.robot = Robot(
            compositeName=composite_name,
            robotName=robot_name,
            urdf_path=urdf_path,
            srdf_path=srdf_path,
            load=True,
            rootJointType=root_joint_type)
        self._create_problem_solver()
        return self.robot
    
    def _create_problem_solver(self):
        self.ps = ProblemSolver(self.robot)
        self.ps.setErrorThreshold(1e-4)
        self.ps.setMaxIterProjection(40)
    
    def load_environment(
        self,
        name: str,
        urdf_path: str,
        urdf_suffix: str = "",
        srdf_suffix: str = "",
        meshpkg_name: Optional[str] = None,
    ):
        """Load environment model."""
        if self.vf is None:
            self.vf = ViewerFactory(self.ps)
        
        pkg_name, urdf_name = parse_package_uri(urdf_path)
        if meshpkg_name is None:
            meshpkg_name = pkg_name  # Assuming meshes are in the same package

        # Create environment config class dynamically
        class EnvConfig:
            rootJointType = "anchor"
            packageName = pkg_name
            urdfName = urdf_name
            meshPackageName = meshpkg_name
            urdfSuffix = urdf_suffix
            srdfSuffix = srdf_suffix
        
        self.vf.loadEnvironmentModel(EnvConfig, name)
        return EnvConfig
    
    def load_object(
        self,
        name: str,
        urdf_path: str,
        root_joint_type: str = "freeflyer",
        urdf_suffix: str = "",
        srdf_suffix: str = "",
        meshpkg_name: Optional[str] = None,
    ):
        """Load manipulable object."""
        if self.vf is None:
            self.vf = ViewerFactory(self.ps)
        
        pkg_name, urdf_name = parse_package_uri(urdf_path)
        if meshpkg_name is None:
            meshpkg_name = pkg_name  # Assuming meshes are in the same package

        # Create object config class dynamically
        class ObjConfig:
            rootJointType = root_joint_type
            packageName = pkg_name
            urdfName = urdf_name
            meshPackageName = meshpkg_name
            urdfSuffix = urdf_suffix
            srdfSuffix = srdf_suffix
        
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
            config_dir = (
                Path(__file__).parent.parent.parent.parent.parent / "config"
            )
            sys.path.insert(0, str(config_dir))
            from spacelab_config import ManipulationConfig
        except ImportError:
            raise ImportError(
                "ManipulationConfig not found. Please provide "
                "valid_pairs parameter or ensure spacelab_config.py "
                "is in the config directory."
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

    def create_state(
        self,
        name: str,
        is_waypoint: bool = False,
        priority: int = 0
    ) -> int:
        """Create a state manually.
        
        Args:
            name: State name
            is_waypoint: Whether this is a waypoint state
            priority: State priority (higher = more important)
            
        Returns:
            State ID
        """
        if self.graph is None:
            raise RuntimeError("Graph not initialized")
        
        # CORBA API uses createNode which returns node ID
        node_id = self.graph.createNode([name], is_waypoint)
        return node_id

    def create_edge(
        self,
        from_state: str,
        to_state: str,
        name: str,
        weight: int = 1,
        containing_state: Optional[str] = None
    ) -> int:
        """Create an edge manually.
        
        Args:
            from_state: Source state name
            to_state: Target state name
            name: Edge name
            weight: Edge weight for planning
            containing_state: State whose constraints apply during edge
            
        Returns:
            Edge ID
        """
        if self.graph is None:
            raise RuntimeError("Graph not initialized")
        
        # CORBA API uses createEdge
        edge_id = self.graph.createEdge(
            from_state,
            to_state,
            name,
            weight,
            containing_state or from_state
        )
        return edge_id

    def apply_state_constraints(
        self,
        state: str,
        q: np.ndarray,
        max_iterations: int = 10000,
        error_threshold: float = 1e-4
    ) -> ConstraintResult:
        """Apply state constraints to project configuration.
        
        Args:
            state: State name
            q: Input configuration
            max_iterations: Maximum projection iterations
            error_threshold: Convergence threshold
            
        Returns:
            ConstraintResult with success status, projected config, and error
        """
        if self.graph is None:
            raise RuntimeError("Graph not initialized")
        
        # Store current parameters
        old_max_iter = self.ps.getMaxIterProjection()
        old_error = self.ps.getErrorThreshold()
        
        # Set temporary parameters
        self.ps.setMaxIterProjection(max_iterations)
        self.ps.setErrorThreshold(error_threshold)
        
        # Apply constraints using graph
        q_list = q.tolist() if isinstance(q, np.ndarray) else list(q)
        try:
            success, q_proj, error = self.graph.applyNodeConstraints(
                state, q_list
            )
            result = ConstraintResult(
                success=success,
                configuration=np.array(q_proj),
                error=error
            )
        except Exception:
            # If applyNodeConstraints not available, use generateTargetConfig
            try:
                res, q_proj, err = self.graph.generateTargetConfig(
                    state, q_list, q_list
                )
                result = ConstraintResult(
                    success=res,
                    configuration=np.array(q_proj),
                    error=err
                )
            except Exception:
                result = ConstraintResult(
                    success=False,
                    configuration=q,
                    error=float('inf')
                )
        
        # Restore parameters
        self.ps.setMaxIterProjection(old_max_iter)
        self.ps.setErrorThreshold(old_error)
        
        return result
    
    def solve(self, max_iterations: int = 10000) -> bool:
        """Solve planning problem.
        
        Args:
            max_iterations: Maximum planning iterations
            
        Returns:
            True if solution found
        """
        try:
            # Configure path validation parameters
            if self._use_path_optimization:
                # Enable path optimization in CORBA
                self.ps.setParameter(
                    "PathOptimization/RandomShortcut/NumberOfLoops", 50
                )
            
            if self._use_path_projection:
                # Enable path projection
                self.ps.setParameter("PathProjection/ProgressBased", True)
            
            # Set max iterations for steering method
            self.ps.setParameter(
                "SteeringMethod/Kinodynamic/maxIterations", max_iterations
            )
            
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
        except Exception:
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
            except Exception:
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

    def get_graph(self):
        """Get constraint graph."""
        return self.graph

    def set_path_optimization(self, enabled: bool):
        """Enable or disable path optimization.
        
        Args:
            enabled: Whether to use path optimization
        """
        self._use_path_optimization = enabled

    def set_path_projection(self, enabled: bool):
        """Enable or disable path projection.
        
        Args:
            enabled: Whether to use path projection
        """
        self._use_path_projection = enabled

    def configure_graph_parameters(
        self,
        max_iterations: int = 40,
        error_threshold: float = 1e-4
    ):
        """Configure constraint graph parameters.
        
        Args:
            max_iterations: Maximum iterations for constraint projection
            error_threshold: Error threshold for constraint satisfaction
        """
        if self.ps is None:
            raise RuntimeError("Problem solver not created yet")
        
        self.ps.setMaxIterProjection(max_iterations)
        self.ps.setErrorThreshold(error_threshold)


__all__ = [
    "CorbaManipulationPlanner",
    "ConstraintResult",
    "HAS_CORBA",
]
