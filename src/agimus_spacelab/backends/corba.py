"""
CORBA backend implementation for manipulation planning.

This backend uses hpp-manipulation-corba for communication with HPP.
"""

from typing import Any, Dict, List, Optional, Sequence, Tuple, Union
import numpy as np
from ..utils import parse_package_uri
from .base import BackendBase, ConstraintResult


try:
    from hpp.corbaserver import loadServerPlugin
    from hpp.corbaserver.manipulation import (
        Client,
        ProblemSolver,
    )
    from hpp.corbaserver.manipulation.robot import Robot as ParentRobot
    from hpp.gepetto import PathPlayer
    from hpp.gepetto.manipulation import ViewerFactory
    HAS_CORBA = True
except ImportError:
    HAS_CORBA = False


class CorbaBackend(BackendBase):
    """CORBA backend implementation for manipulation planning."""

    def __init__(self):
        """Initialize CORBA backend."""
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

        # TransitionPlanner (edge-scoped planning)
        self._transition_planner = None
        self._transition_time_out = 60.0  # Base timeout for planning
        self._transition_max_iterations = 10000  # Base iterations for RRT*
        self._transition_path_projector: Optional[Tuple[str, float]] = (
            "Progressive",
            0.2,
        )
        
        # Optimizer profiles for different edge types
        # Transit edges (free motion): Use spline optimization
        self._transit_edge_optimizers = [
            "SplineGradientBased_bezier3",
            "EnforceTransitionSemantic",
            "SimpleTimeParameterization",
        ]
        # Waypoint edges (constrained motion): Skip spline, just time parameterize
        self._waypoint_edge_optimizers = [
            "EnforceTransitionSemantic",
            "SimpleTimeParameterization",
        ]
        # Default fallback
        self._transition_default_optimizers = [
            "EnforceTransitionSemantic",
            "SimpleTimeParameterization",
        ]
        # Per-edge optimizer list (edge_id -> optimizer names)
        self._transition_optimizers_by_edge_id: Dict[int, List[str]] = {}
        
        # Time parameterization parameters
        self._time_param_order = 2
        self._time_param_max_accel = 0.2
        
        # Distance-based auto-tuning
        self._enable_distance_tuning = True
        self._distance_scale_factor = 0.5  # Scale timeout/iterations by distance

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
        robot_name: str,
        urdf_path: str,
        srdf_path: Optional[str] = None,
        root_joint_type: str = "anchor",
        composite_name: str = None,
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
            if composite_name is None:
                composite_name = robot_name
        self.robot = Robot(
            compositeName=composite_name,
            robotName=robot_name,
            urdf_path=urdf_path,
            srdf_path=srdf_path,
            load=True,
            rootJointType=root_joint_type)

        # Initialize problem solver
        self.ps = ProblemSolver(self.robot)
        if self.ps is None:
            raise RuntimeError("Failed to create ProblemSolver")

        return self.robot

    def load_environment(
        self,
        name: str,
        urdf_path: str,
        urdf_suffix: str = "",
        srdf_suffix: str = "",
        meshpkg_name: Optional[str] = None,
        pose: Any = None,
    ):
        """Load environment model."""
        if self.vf is None:
            if self.ps is not None:
                self.vf = ViewerFactory(self.ps)
            else:
                raise RuntimeError("Problem solver not created yet")

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
        srdf_path: Optional[str] = None,
        root_joint_type: str = "freeflyer",
        urdf_suffix: str = "",
        srdf_suffix: str = "",
        meshpkg_name: Optional[str] = None,
    ):
        """Load manipulable object.
        
        Args:
            name: Name for the object
            urdf_path: Path to URDF file
            srdf_path: Path to SRDF file (optional, for API consistency)
            root_joint_type: Type of root joint
            urdf_suffix: URDF suffix for loading
            srdf_suffix: SRDF suffix for loading
            meshpkg_name: Mesh package name
            
        Note:
            CORBA backend uses srdf_suffix to find SRDF in the same location
            as the URDF. The srdf_path parameter is accepted but not used
            directly since loadObjectModel handles SRDF loading internally.
        """
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
        if isinstance(q, list):
            self.ps.setInitialConfig(q)
        else:
            self.ps.setInitialConfig(q.tolist())

    def add_goal_config(self, q: np.ndarray):
        """Add goal configuration."""
        if isinstance(q, list):
            self.ps.addGoalConfig(q)
        else:
            self.ps.addGoalConfig(q.tolist())

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

    def solve(
        self,
        max_iterations: int = 10000,
        optimizer: str = "RandomShortcut"
    ) -> bool:
        """Solve planning problem.
        
        Args:
            max_iterations: Maximum planning iterations
            optimizer: Path optimizer to use during optimization
            
        Returns:
            True if solution found
        """
        try:
            # Configure path optimization (uses default RandomShortcut)
            self.configure_path_optimization(optimizer=optimizer)

            self.ps.setMaxIterPathPlanning(max_iterations)
            self.ps.solve()

            # Apply path optimization if enabled and paths exist
            if self._use_path_optimization and self.ps.numberPaths() > 0:
                self.optimize_path(self.ps.numberPaths() - 1)

            return True
        except Exception as e:
            print(f"Planning failed: {e}")
            return False

    # ---------------------------------------------------------------------
    # TransitionPlanner integration (edge-scoped planning)
    # ---------------------------------------------------------------------

    def _resolve_edge_id(self, edge: Union[int, str]) -> int:
        if isinstance(edge, int):
            return edge
        if not isinstance(edge, str):
            raise TypeError(f"edge must be int or str, got {type(edge)!r}")
        if self.graph is None:
            raise RuntimeError("Graph not initialized")

        edges = getattr(self.graph, "edges", None)
        if isinstance(edges, dict) and edge in edges:
            try:
                return int(edges[edge])
            except Exception:
                pass

        raise KeyError(
            (
                "Unknown edge '%s'. If using a factory graph, expected it "
                "to be present in graph.edges."
            )
            % edge
        )

    def _resolve_edge_name(self, edge: Union[int, str]) -> str:
        """Convert edge ID or name to string name."""
        if isinstance(edge, str):
            return edge
        if self.graph is None:
            raise RuntimeError("Graph not initialized")
        edges = getattr(self.graph, "edges", None)
        if isinstance(edges, dict):
            # Find edge name by ID
            for name, eid in edges.items():
                if int(eid) == int(edge):
                    return name
        raise ValueError(f"Edge ID {edge} not found in graph")
    
    def _is_waypoint_edge(self, edge_name: str) -> bool:
        """Check if edge is a waypoint edge (constrained motion).
        
        Waypoint edges (_01, _12, _21, _10) represent constrained motion
        like pregrasp->grasp transitions. They benefit less from spline
        optimization compared to free-space transit edges.
        """
        return any(suffix in edge_name for suffix in ["_01", "_12", "_21", "_10"])
    
    def _compute_config_distance(self, q1: Sequence[float], q2: Sequence[float]) -> float:
        """Compute configuration space distance for auto-tuning."""
        return float(np.linalg.norm(np.array(q1) - np.array(q2)))
    
    def _compute_planning_budget(
        self, q1: Sequence[float], q2: Sequence[float]
    ) -> Tuple[float, int]:
        """Compute timeout and max_iterations based on distance.
        
        Returns:
            (timeout, max_iterations) scaled by configuration distance
        """
        if not self._enable_distance_tuning:
            return self._transition_time_out, self._transition_max_iterations
        
        distance = self._compute_config_distance(q1, q2)
        # Normalize by typical arm reach (~2.0 rad per joint)
        # For 20 DOF system, typical max distance ~40
        normalized_distance = distance / 40.0
        scale = 1.0 + (normalized_distance * self._distance_scale_factor)
        
        # Apply scaling with floor values
        timeout = max(10.0, self._transition_time_out * scale)
        max_iter = max(1000, int(self._transition_max_iterations * scale))
        
        return timeout, max_iter

    def _load_path_optimizer_plugin_if_needed(self, optimizer: str) -> None:
        if self.ps is None:
            raise RuntimeError("Problem solver not created yet")

        if optimizer.startswith("SplineGradientBased"):
            self.ps.loadPlugin("spline-gradient-based.so")
        elif optimizer == "TOPPRA":
            self.ps.loadPlugin("toppra.so")

    def configure_transition_planner(
        self,
        *,
        time_out: Optional[float] = None,
        max_iterations: Optional[int] = None,
        path_projector: Optional[Tuple[str, float]] = None,
        enable_distance_tuning: Optional[bool] = None,
        distance_scale_factor: Optional[float] = None,
    ) -> None:
        """Configure defaults for the TransitionPlanner.

        Args:
            time_out: Base timeout in seconds (scaled by distance if tuning enabled)
            max_iterations: Base max iterations (scaled by distance if tuning enabled)
            path_projector: Path projector type and tolerance
            enable_distance_tuning: Enable distance-based timeout/iteration scaling
            distance_scale_factor: Scale factor for distance-based tuning

        Notes:
        - In practice, TransitionPlanner should have both timeOut and
          maxIterations set.
        - This only affects edge-scoped planning, not ProblemSolver.solve().
        """
        if time_out is not None:
            self._transition_time_out = float(time_out)
        if max_iterations is not None:
            self._transition_max_iterations = int(max_iterations)
        if path_projector is not None:
            self._transition_path_projector = path_projector
        if enable_distance_tuning is not None:
            self._enable_distance_tuning = enable_distance_tuning
        if distance_scale_factor is not None:
            self._distance_scale_factor = distance_scale_factor

        tp = self._transition_planner
        if tp is not None:
            self._apply_transition_planner_defaults(tp)
    
    def configure_time_parameterization(
        self,
        order: Optional[int] = None,
        max_acceleration: Optional[float] = None,
    ) -> None:
        """Configure SimpleTimeParameterization parameters.
        
        Args:
            order: Polynomial order for time parameterization (default 2)
            max_acceleration: Maximum acceleration limit (default 0.2)
        """
        if order is not None:
            self._time_param_order = int(order)
        if max_acceleration is not None:
            self._time_param_max_accel = float(max_acceleration)
        
        # Apply to existing transition planner if created
        tp = self._transition_planner
        if tp is not None and self.ps is not None:
            try:
                from CORBA import Any, TC_long, TC_double
                problem = self.ps.client.basic.problem.getProblem()
                problem.setParameter(
                    "SimpleTimeParameterization/order",
                    Any(TC_long, self._time_param_order)
                )
                problem.setParameter(
                    "SimpleTimeParameterization/maxAcceleration",
                    Any(TC_double, self._time_param_max_accel)
                )
            except Exception:
                pass

    def _apply_transition_planner_defaults(self, tp: Any) -> None:
        try:
            tp.timeOut(self._transition_time_out)
        except Exception:
            pass
        try:
            tp.maxIterations(self._transition_max_iterations)
        except Exception:
            pass
        if self._transition_path_projector is not None:
            proj_type, tol = self._transition_path_projector
            try:
                tp.setPathProjector(proj_type, float(tol))
            except Exception:
                pass
        # Add default path optimizers
        for opt in self._transition_default_optimizers:
            try:
                self._load_path_optimizer_plugin_if_needed(opt)
                tp.addPathOptimizer(opt)
            except Exception:
                pass
        
        # Apply time parameterization settings
        try:
            from CORBA import Any as CorbaAny, TC_long, TC_double
            problem = self.ps.client.basic.problem.getProblem()
            problem.setParameter(
                "SimpleTimeParameterization/order",
                CorbaAny(TC_long, self._time_param_order)
            )
            problem.setParameter(
                "SimpleTimeParameterization/maxAcceleration",
                CorbaAny(TC_double, self._time_param_max_accel)
            )
        except Exception:
            pass

    def set_inner_problem_parameter(self, key: str, value: Any) -> None:
        """Set parameter on TransitionPlanner's inner problem.
        
        Useful for tuning BiRrtStar parameters like:
        - "kRRT*": Growth factor for RRT*
        - "kPRM*": k-nearest parameter
        
        Args:
            key: Parameter name
            value: Parameter value (will be wrapped in CORBA Any)
        """
        tp = self.ensure_transition_planner()
        try:
            from CORBA import Any as CorbaAny, TC_double
            # Most parameters are doubles
            tp.setParameter(key, CorbaAny(TC_double, float(value)))
        except Exception as exc:
            raise RuntimeError(f"Failed to set parameter '{key}': {exc}")

    def ensure_transition_planner(self) -> Any:
        """Create (or return cached) TransitionPlanner CORBA object."""
        if self.ps is None:
            raise RuntimeError("Problem solver not created yet")

        if self._transition_planner is not None:
            print(f"      [TP] Using cached TransitionPlanner")
            return self._transition_planner

        print(f"      [TP] Creating new TransitionPlanner")
        tp = self.ps.client.manipulation.problem.createTransitionPlanner()
        self._apply_transition_planner_defaults(tp)
        self._transition_planner = tp
        return tp

    def reset_transition_planner(self) -> None:
        """Dispose and forget the cached TransitionPlanner CORBA object.

        This is useful when you want deterministic cleanup (or when the
        underlying problem/graph changes and you prefer a fresh planner).

        Notes:
        - Cleanup is best-effort: CORBA stubs may or may not expose
          deallocation helpers depending on bindings.
        """
        tp = self._transition_planner
        self._transition_planner = None
        if tp is None:
            print(f"      [TP] No cached TransitionPlanner to reset")
            return

        print(f"      [TP] Resetting cached TransitionPlanner")
        delete_this = getattr(tp, "deleteThis", None)
        if callable(delete_this):
            try:
                delete_this()
                print(f"      [TP] Called deleteThis()")
            except Exception:
                pass

    def set_transition_optimizers(
        self,
        edge: Union[int, str],
        optimizers: Sequence[str],
        *,
        clear_existing: bool = True,
    ) -> None:
        """Set the optimizer list to use for a given transition edge."""
        edge_id = self._resolve_edge_id(edge)
        opts = [str(o) for o in optimizers]
        missing = edge_id not in self._transition_optimizers_by_edge_id
        if clear_existing or missing:
            self._transition_optimizers_by_edge_id[edge_id] = opts
        else:
            self._transition_optimizers_by_edge_id[edge_id].extend(opts)

    def clear_transition_optimizers(
        self, edge: Optional[Union[int, str]] = None
    ) -> None:
        """Clear per-edge transition optimizer lists."""
        if edge is None:
            self._transition_optimizers_by_edge_id.clear()
            return
        edge_id = self._resolve_edge_id(edge)
        self._transition_optimizers_by_edge_id.pop(edge_id, None)

    def _configure_transition_planner_for_edge(
        self, tp: Any, edge_id: int, edge_name: Optional[str] = None
    ) -> None:
        try:
            tp.setEdge(int(edge_id))
            print(f"      [TP] Set edge ID: {edge_id}")
        except Exception as exc:
            raise RuntimeError(
                f"Failed to set TransitionPlanner edge {edge_id}: {exc}"
            )

        # Determine which optimizers to use
        if edge_id in self._transition_optimizers_by_edge_id:
            # Explicit per-edge configuration
            optimizers = self._transition_optimizers_by_edge_id[edge_id]
            print(f"      [TP] Using edge-specific optimizers")
        elif edge_name and self._is_waypoint_edge(edge_name):
            # Waypoint edge: skip spline, just time parameterize
            optimizers = self._waypoint_edge_optimizers
            print(f"      [TP] Using waypoint edge optimizers (no spline)")
        elif edge_name:
            # Transit edge: use spline optimization
            optimizers = self._transit_edge_optimizers
            print(f"      [TP] Using transit edge optimizers (with spline)")
        else:
            # Fallback
            optimizers = self._transition_default_optimizers
            print(f"      [TP] Using default optimizers")
        
        # Clear and set optimizers
        try:
            tp.clearPathOptimizers()
        except Exception:
            pass
        
        for opt in optimizers:
            self._load_path_optimizer_plugin_if_needed(opt)
            tp.addPathOptimizer(opt)
            print(f"      [TP] Added optimizer: {opt}")

    def plan_transition_edge(
        self,
        edge: Union[int, str],
        q1: Sequence[float],
        q2: Sequence[float],
        *,
        validate: bool = True,
        reset_roadmap: bool = True,
        time_parameterize: bool = True,
        store: bool = False,
    ) -> Union[int, Any]:
        """Plan a transition along a specific graph edge.
        
        Strategy:
        - Waypoint edges (_01, _12, _21, _10): Skip directPath, use planPath (BiRrtStar)
        - Transit edges: Try directPath first, fallback to planPath if it fails
        
        Returns:
            PathVector CORBA object (store parameter ignored for TransitionPlanner).
        """
        if self.ps is None:
            raise RuntimeError("Problem solver not created yet")

        tp = self.ensure_transition_planner()
        edge_id = self._resolve_edge_id(edge)
        edge_name = self._resolve_edge_name(edge)
        
        # Configure optimizers based on edge type
        self._configure_transition_planner_for_edge(tp, edge_id, edge_name)

        q1_list = list(q1)
        q2_list = list(q2)
        
        # Compute planning budget based on distance
        timeout, max_iter = self._compute_planning_budget(q1_list, q2_list)
        tp.timeOut(timeout)
        tp.maxIterations(max_iter)
        print(f"      [TP] Planning budget: timeout={timeout:.1f}s, max_iter={max_iter}")
        
        # Validate configurations
        q1_ok, msg1 = tp.validateConfiguration(q1_list, edge_id)
        q2_ok, msg2 = tp.validateConfiguration(q2_list, edge_id)
        print(f"      [TP] Validate q1: {q1_ok}, msg: {msg1}")
        print(f"      [TP] Validate q2: {q2_ok}, msg: {msg2}")
        
        # Check if this is a waypoint edge
        is_waypoint = self._is_waypoint_edge(edge_name)
        
        pv: Any
        if is_waypoint:
            # Waypoint edges: Skip directPath, go straight to sampling-based planning
            # These edges have constrained motion and directPath rarely succeeds
            print(f"      [TP] Waypoint edge detected, using planPath (BiRrtStar)")
            try:
                pv = tp.planPath(q1_list, [q2_list], bool(reset_roadmap))
                print(f"      [TP] planPath succeeded")
            except Exception as exc:
                raise RuntimeError(
                    f"TransitionPlanner.planPath failed for waypoint edge: {exc}"
                )
        else:
            # Transit edges: Try directPath first (fast), fallback to planPath
            print(f"      [TP] Transit edge, trying directPath first")
            try:
                path, success, status = tp.directPath(
                    q1_list, q2_list, bool(validate)
                )
                if success:
                    print(f"      [TP] directPath succeeded")
                    try:
                        pv = tp.optimizePath(path)
                    except Exception:
                        try:
                            pv = path.asVector()
                        except Exception as exc:
                            raise RuntimeError(
                                "Failed to convert directPath result to PathVector: %s"
                                % exc
                            )
                else:
                    # directPath failed, fallback to planPath
                    print(f"      [TP] directPath failed: {status}")
                    print(f"      [TP] Falling back to planPath (BiRrtStar)")
                    try:
                        pv = tp.planPath(q1_list, [q2_list], bool(reset_roadmap))
                        print(f"      [TP] planPath succeeded")
                    except Exception as exc:
                        raise RuntimeError(
                            (
                                "Both directPath and planPath failed. "
                                "directPath status: %s. planPath error: %s"
                            )
                            % (status, exc)
                        )
            except Exception as exc:
                # directPath itself threw an exception
                print(f"      [TP] directPath threw exception: {exc}")
                print(f"      [TP] Falling back to planPath (BiRrtStar)")
                try:
                    pv = tp.planPath(q1_list, [q2_list], bool(reset_roadmap))
                    print(f"      [TP] planPath succeeded")
                except Exception as exc2:
                    raise RuntimeError(
                        f"TransitionPlanner failed completely: {exc2}"
                    )

        if time_parameterize:
            try:
                pv = tp.timeParameterization(pv)
            except Exception:
                pass

        # Return PathVector directly - TransitionPlanner paths aren't stored
        # in ProblemSolver's path list, they're used directly
        return pv

    def plan_transition_sequence(
        self,
        edges: Sequence[Union[int, str]],
        waypoints: Sequence[Sequence[float]],
        *,
        validate: bool = True,
        reset_roadmap: bool = True,
        time_parameterize: bool = True,
        store: bool = False,
    ) -> Union[int, Any]:
        """Plan a multi-edge transition sequence and concatenate results.

        Returns:
            Concatenated PathVector CORBA object.
        """
        if len(waypoints) != len(edges) + 1:
            raise ValueError(
                "Expected len(waypoints) == len(edges) + 1, got %d and %d"
                % (len(waypoints), len(edges))
            )

        pv_total: Optional[Any] = None
        for i, edge in enumerate(edges):
            pv_i = self.plan_transition_edge(
                edge,
                waypoints[i],
                waypoints[i + 1],
                validate=validate,
                reset_roadmap=reset_roadmap,
                time_parameterize=time_parameterize,
                store=False,
            )
            if pv_total is None:
                pv_total = pv_i
            else:
                pv_total.concatenate(pv_i)

        if pv_total is None:
            raise RuntimeError("No edges provided")

        # Return concatenated PathVector directly
        return pv_total

    def optimize_path(self, path_index: int = -1) -> int:
        """Optimize a path using configured optimizers.
        
        Args:
            path_index: Index of the path to optimize.
                       If -1, optimizes the last path.
            
        Returns:
            Index of the optimized path
        """
        if self.ps is None:
            raise RuntimeError("Problem solver not created yet")

        num_paths = self.ps.numberPaths()
        if num_paths == 0:
            raise RuntimeError("No paths to optimize")

        if path_index < 0:
            path_index = num_paths - 1

        # optimizePath returns the index of the optimized path
        optimized_index = self.ps.optimizePath(path_index)
        return optimized_index

    def get_path(self, index: int = 0) -> Optional[Any]:
        """Get computed path."""
        if self.ps is None:
            return None

        try:
            return self.ps.client.manipulation.problem.getPath(index)
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

    def play_path_vector(self, path_vector: Any, speed: float = 1.0) -> int:
        """Play a PathVector object directly in the viewer.

        PathPlayer requires paths to be stored in ProblemSolver first.
        This method adds the PathVector to the problem solver to get an index,
        then plays it via PathPlayer.

        Args:
            path_vector: PathVector CORBA object from TransitionPlanner
            speed: Playback speed multiplier (currently not used)

        Returns:
            Index of the stored path in ProblemSolver

        Raises:
            RuntimeError: If problem solver is not initialized
        """
        if self.ps is None:
            raise RuntimeError("Problem solver not created yet")

        if self.viewer is None:
            self.visualize()

        if self.path_player is None:
            self.path_player = PathPlayer(self.viewer)

        # Get current number of paths to determine the index
        path_index = self.ps.numberPaths()

        # Add PathVector to problem solver (uses basic interface)
        # This stores the path and returns the index
        self.ps.client.basic.problem.addPath(path_vector)

        # Play the path by index
        try:
            self.path_player(path_index)
        except Exception as e:
            print(f"Failed to play path at index {path_index}: {e}")

        return path_index

    def clear_stored_paths(self, verbose: bool = True) -> int:
        """Clear all paths stored in ProblemSolver.

        Useful to prevent memory accumulation when planning/replaying
        multiple sequences. Resets the path counter to 0.

        Args:
            verbose: Print number of paths cleared

        Returns:
            Number of paths that were cleared
        """
        if self.ps is None:
            return 0

        num_paths = self.ps.numberPaths()

        if num_paths > 0:
            # Clear paths by resetting problem solver paths
            # Note: hpp doesn't have a direct clearPaths() API,
            # so we need to reset by recreating the problem or
            # just be aware of accumulation
            if verbose:
                print(f"Note: {num_paths} paths are stored in ProblemSolver")
                print("     (hpp doesn't provide path clearing API)")

        return num_paths

    def get_num_stored_paths(self) -> int:
        """Get number of paths currently stored in ProblemSolver.

        Returns:
            Number of stored paths
        """
        if self.ps is None:
            return 0
        return self.ps.numberPaths()

    def get_robot(self):
        """Get robot instance."""
        return self.robot

    def get_problem(self):
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

    def configure_path_validation(
        self,
        validation_step: float = 0.01,
        projector_step: float = 0.1
    ):
        """Configure path validation parameters."""
        print("   Configuring path validation...")
        self.ps.selectPathValidation("Discretized", validation_step)
        self.ps.selectPathProjector("Progressive", projector_step)
        # self.ps.selectSteeringMethod("ReedsShepp")
        # self.ps.selectPathPlanner("DiffusingPlanner")

    def configure_path_optimization(
        self,
        optimizer: str = "RandomShortcut",
        clear_existing: bool = True
    ):
        """Configure path optimization parameters.
        
        Args:
            optimizer: Path optimizer to use. Available options:
                Built-in (hpp-core):
                - "RandomShortcut": Random shortcut optimizer
                - "SimpleShortcut": Simple shortcut optimizer
                - "PartialShortcut": Partial shortcut optimizer
                - "SimpleTimeParameterization": Adds time parameterization
                - "RSTimeParameterization": Reeds-Shepp time parameterization
                
                Manipulation-specific (hpp-manipulation):
                - "Graph-RandomShortcut": Graph-aware random shortcut
                - "Graph-PartialShortcut": Graph-aware partial shortcut
                - "EnforceTransitionSemantic": Enforces transition semantics
                
                Plugin-based (loaded automatically):
                - "SplineGradientBased_bezier1": Spline optimization, order 1
                - "SplineGradientBased_bezier3": Spline optimization, order 3
                - "SplineGradientBased_bezier5": Spline optimization, order 5
                - "TOPPRA": Time-optimal path parameterization
            clear_existing: Whether to clear existing optimizers first
        """
        if not self._use_path_optimization:
            return

        # Load plugin if needed for spline-based or TOPPRA optimizers
        if optimizer.startswith("SplineGradientBased"):
            self.ps.loadPlugin("spline-gradient-based.so")
        elif optimizer == "TOPPRA":
            self.ps.loadPlugin("toppra.so")

        if clear_existing:
            self.ps.clearPathOptimizers()

        self.ps.addPathOptimizer(optimizer)


# Alias for backward compatibility
CorbaManipulationPlanner = CorbaBackend


__all__ = [
    "CorbaBackend",
    "CorbaManipulationPlanner",
    "ConstraintResult",
    "HAS_CORBA",
]
