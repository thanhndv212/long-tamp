"""
PyHPP backend implementation for manipulation planning.

This backend uses hpp-python for direct Python bindings to HPP.
"""

from typing import Any, Dict, List, Optional, Sequence, Tuple, Union
import numpy as np
from pinocchio import SE3
from .base import BackendBase, ConstraintResult


try:
    from pyhpp.manipulation import (
        Device,
        urdf,
        ManipulationPlanner as HPPManipulationPlanner,
        TransitionPlanner as HPPTransitionPlanner,
        Problem,
        ProgressiveProjector,
        RandomShortcut as ManipulationRandomShortcut,
        EnforceTransitionSemantic,
        GraphRandomShortcut,
        GraphPartialShortcut,
    )
    from pyhpp.core import (
        Discretized,
        RandomShortcut,
        SimpleShortcut,
        PartialShortcut,
        SimpleTimeParameterization,
        RSTimeParameterization,
    )
    from pyhpp.gepetto.viewer import Viewer
    HAS_PYHPP = True
except ImportError:
    HAS_PYHPP = False


class PyHPPBackend(BackendBase):
    """PyHPP backend implementation for manipulation planning."""

    def __init__(self):
        """Initialize PyHPP backend."""
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
        self._path_optimizers = []  # List of path optimizer instances

        # Stored paths (local equivalent to CORBA ProblemSolver path ids)
        self._stored_paths: List[Any] = []

        # TransitionPlanner (edge-scoped planning)
        self._transition_planner = None
        self._transition_time_out = 3.0
        self._transition_max_iterations = 3000
        # (projector_type, step). If None, fall back to problem.pathProjector()
        self._transition_path_projector: Optional[Tuple[str, float]] = (
            "Progressive",
            0.2,
        )
        # Per-edge optimizer list (edge_name -> optimizer names)
        self._transition_optimizers_by_edge_name: Dict[str, List[str]] = {}

        # Configuration options for path validation and projection
        self._use_pathvalidation = True
        self._use_progressive_projector = True
        self._use_path_optimization = True

    def model(self):
        """Get the Pinocchio model."""
        if self.device is None:
            raise RuntimeError("Robot not loaded yet")
        return self.device.model()

    def data(self):
        """Get the Pinocchio data."""
        if self.device is None:
            raise RuntimeError("Robot not loaded yet")
        return self.device.data()

    @property
    def nq(self) -> int:
        """Number of configuration variables."""
        if self.device is None:
            raise RuntimeError("Robot not loaded yet")
        return self.device.model().nq

    @property
    def nv(self) -> int:
        """Number of velocity variables."""
        if self.device is None:
            raise RuntimeError("Robot not loaded yet")
        return self.device.model().nv

    def neutral_config(self) -> np.ndarray:
        """Get neutral configuration."""
        if self.device is None:
            raise RuntimeError("Robot not loaded yet")
        return np.array(self.device.neutralConfiguration())

    def random_config(self) -> np.ndarray:
        """Generate a random configuration."""
        if self.problem is None:
            raise RuntimeError("Problem not created yet")
        self._shooter = self.problem.configurationShooter()
        return np.array(self._shooter.shoot())

    def load_robot(
        self,
        robot_name: str,
        urdf_path: str,
        srdf_path: Optional[str] = None,
        root_joint_type: str = "anchor",
        composite_name: str = None,
    ):
        """Load robot using PyHPP."""
        self.device = Device(robot_name)

        urdf.loadModel(
            self.device,
            0,
            robot_name,
            root_joint_type,
            urdf_path,
            srdf_path or "",
            SE3.Identity()
        )

        # Create problem
        self.problem = Problem(self.device)

        return self.device

    def load_environment(
        self,
        name: str,
        urdf_path: str,
        pose: Optional[SE3] = None
    ):
        """Load environment model.
        
        Args:
            name: Name for the environment object
            urdf_path: Path to URDF file
            pose: Optional SE3 pose (defaults to identity)
        """
        if self.device is None:
            raise RuntimeError("Must load robot first")

        if pose is None:
            pose = SE3.Identity()

        urdf.loadModel(
            self.device,
            0,
            name,
            "anchor",
            urdf_path,
            "",
            pose
        )

        return name

    def load_object(
        self,
        name: str,
        urdf_path: str,
        srdf_path: Optional[str] = None,
        root_joint_type: str = "freeflyer"
    ):
        """Load manipulable object.
        
        Args:
            name: Name for the object (used as prefix for handles/grippers)
            urdf_path: Path to URDF file
            srdf_path: Path to SRDF file containing handles/grippers/contacts
            root_joint_type: Type of root joint ('freeflyer' or 'anchor')
        """
        if self.device is None:
            raise RuntimeError("Must load robot first")

        urdf.loadModel(
            self.device,
            0,
            name,
            root_joint_type,
            urdf_path,
            srdf_path or "",
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
        if isinstance(q, list):
            q = np.array(q)
        self.problem.initConfig(q)

    def add_goal_config(self, q: np.ndarray):
        """Add goal configuration."""
        if self.problem is None:
            raise RuntimeError("Must create problem first")
        if isinstance(q, list):
            q = np.array(q)
        self.problem.addGoalConfig(q)

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

        state_id = self.graph.createState(name, is_waypoint, priority)
        return state_id

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
        old_max_iter = self.graph.maxIterations()
        old_error = self.graph.errorThreshold()

        # Set temporary parameters
        self.graph.maxIterations(max_iterations)
        self.graph.errorThreshold(error_threshold)

        # Apply constraints
        success, q_proj, error = self.graph.applyConstraints(state, list(q))

        # Restore parameters
        self.graph.maxIterations(old_max_iter)
        self.graph.errorThreshold(old_error)

        return ConstraintResult(
            success=success,
            configuration=np.array(q_proj),
            error=error
        )

    def solve(
        self, max_iterations: int = 10000, optimizer: str = "RandomShortcut"
    ) -> bool:
        """Solve planning problem.
        
        Args:
            max_iterations: Maximum planning iterations
            
        Returns:
            True if solution found
        """
        if self.problem is None:
            raise RuntimeError("Must create problem first")

        try:
            # Configure path validation with dichotomy if enabled
            self.configure_path_validation()

            # Configure path optimization (uses default RandomShortcut)
            self.configure_path_optimization(optimizer=optimizer)

            self.planner = HPPManipulationPlanner(self.problem)
            self.planner.maxIterations(max_iterations)
            success = self.planner.solve()

            if success:
                self.path = self.planner.path()
                # Apply path optimization if configured
                if self._path_optimizers:
                    self.path = self.optimize_path(self.path)

            return success
        except Exception as e:
            print(f"Planning failed: {e}")
            return False

    def get_path(self, index: int = 0) -> Optional[Any]:
        """Get a stored path by id (or fallback to last computed path)."""
        if self._stored_paths:
            if index < 0:
                index = len(self._stored_paths) - 1
            if 0 <= index < len(self._stored_paths):
                return self._stored_paths[index]
        return self.path

    def visualize(self, q: Optional[np.ndarray] = None):
        """Visualize configuration."""
        if self.viewer is None:
            if self.device is None:
                raise RuntimeError("Must load robot first")
            self.viewer = Viewer(self.device)

        if q is not None:
            if isinstance(q, list):
                q = np.array(q)
            self.viewer(q)
        else:
            # Display current config or initial
            try:
                q_init = self.problem.initConfig()
                self.viewer(q_init)
            except Exception:
                pass

    def play_path(self, path_index: int = 0):
        """Play path in viewer."""
        path = None
        if self._stored_paths:
            if path_index < 0:
                path_index = len(self._stored_paths) - 1
            if 0 <= path_index < len(self._stored_paths):
                path = self._stored_paths[path_index]

        if path is None:
            path = self.path

        if path is None:
            print("No path to play")
            return

        if self.viewer is None:
            self.visualize()

        # Animate path
        import time
        t = 0.0
        dt = 0.01
        length = path.length()

        while t <= length:
            q = path(t)
            self.viewer(q)
            time.sleep(dt)
            t += dt

    def store_path(self, path) -> int:
        """Store a PathVector locally and return its integer id."""
        self._stored_paths.append(path)
        return len(self._stored_paths) - 1

    def get_robot(self):
        """Get device object."""
        return self.device

    def get_problem(self):
        """Get problem object."""
        return self.problem

    def get_graph(self):
        """Get constraint graph."""
        return self.graph

    def set_dichotomy(self, enabled: bool):
        """Enable or disable dichotomy path validation.
        
        Args:
            enabled: Whether to use dichotomy for path validation
        """
        self._use_pathvalidation = enabled

    def set_progressive_projector(self, enabled: bool):
        """Enable or disable progressive path projector.
        
        Args:
            enabled: Whether to use progressive projector
        """
        self._use_progressive_projector = enabled

    def configure_graph_parameters(
        self,
        max_iterations: int = 10000,
        error_threshold: float = 1e-4
    ):
        """Configure constraint graph parameters before initialization.
        
        Args:
            max_iterations: Maximum iterations for constraint projection
            error_threshold: Error threshold for constraint satisfaction
        """
        if self.graph is None:
            raise RuntimeError("Graph not created yet")

        self.graph.setMaxIterations(max_iterations)
        self.graph.setErrorThreshold(error_threshold)

    def configure_path_validation(
        self,
        validation_step: float = 0.01,
        projector_step: float = 0.1
    ):
        """Configure path validation parameters."""
        print("   Configuring path validation...")
        if self.problem is None:
            raise RuntimeError("Must create problem first")

        # Configure path validation if enabled
        if self._use_pathvalidation:
            self.problem.pathValidation = Discretized(
                self.device, validation_step
            )
        # Configure path projection if enabled
        if self._use_progressive_projector:
            self.problem.pathProjector = ProgressiveProjector(
                self.problem.distance(),
                self.problem.steeringMethod(),
                projector_step,
            )
        return self

    def set_path_optimization(self, enabled: bool):
        """Enable or disable path optimization.

        Args:
            enabled: Whether to use path optimization
        """
        self._use_path_optimization = enabled

    def configure_path_optimization(
        self, optimizer: str = "RandomShortcut", clear_existing: bool = True
    ):
        """Configure path optimization.

        Args:
            optimizer: Path optimizer to use. Available options:
                Core optimizers (pyhpp.core):
                - "RandomShortcut": Random shortcut optimizer
                - "SimpleShortcut": Simple shortcut optimizer
                - "PartialShortcut": Partial shortcut optimizer
                - "SimpleTimeParameterization": Adds time parameterization
                - "RSTimeParameterization": Reeds-Shepp time parameterization

                Manipulation optimizers (pyhpp.manipulation):
                                - "ManipulationRandomShortcut":
                                    Manipulation-aware shortcut
                - "EnforceTransitionSemantic": Enforces transition semantics
                - "GraphRandomShortcut": Graph-aware random shortcut
                - "GraphPartialShortcut": Graph-aware partial shortcut
            clear_existing: Whether to clear existing optimizers first
        """
        if not self._use_path_optimization:
            return

        if self.problem is None:
            raise RuntimeError("Must create problem first")

        if clear_existing:
            self._path_optimizers.clear()

        opt_instance = self._create_path_optimizer_instance(optimizer)
        if opt_instance is None:
            raise ValueError(
                f"Unknown optimizer: {optimizer}. "
                f"Available: {sorted(self._path_optimizer_factories().keys())}"
            )
        self._path_optimizers.append(opt_instance)

    def _path_optimizer_factories(self) -> Dict[str, Any]:
        """Factories for string-based path optimizer selection."""
        if self.problem is None:
            raise RuntimeError("Must create problem first")

        prob = self.problem
        return {
            # Core optimizers
            "RandomShortcut": lambda: RandomShortcut(prob),
            "SimpleShortcut": lambda: SimpleShortcut(prob),
            "PartialShortcut": lambda: PartialShortcut(prob),
            "SimpleTimeParameterization": lambda: SimpleTimeParameterization(
                prob
            ),
            "RSTimeParameterization": lambda: RSTimeParameterization(prob),
            # Manipulation optimizers
            "ManipulationRandomShortcut": lambda: ManipulationRandomShortcut(
                prob
            ),
            "EnforceTransitionSemantic": lambda: EnforceTransitionSemantic(
                prob
            ),
            "GraphRandomShortcut": lambda: GraphRandomShortcut(prob),
            "GraphPartialShortcut": lambda: GraphPartialShortcut(prob),
        }

    def _create_path_optimizer_instance(self, optimizer: str):
        factories = self._path_optimizer_factories()
        factory = factories.get(str(optimizer))
        if factory is None:
            return None
        return factory()

    # ---------------------------------------------------------------------
    # TransitionPlanner integration (edge-scoped planning)
    # ---------------------------------------------------------------------

    def configure_transition_planner(
        self,
        *,
        time_out: Optional[float] = None,
        max_iterations: Optional[int] = None,
        path_projector: Optional[Tuple[str, float]] = None,
    ) -> None:
        """Configure defaults for TransitionPlanner edge-scoped planning."""
        if time_out is not None:
            self._transition_time_out = float(time_out)
        if max_iterations is not None:
            self._transition_max_iterations = int(max_iterations)
        if path_projector is not None:
            self._transition_path_projector = path_projector

        tp = self._transition_planner
        if tp is not None:
            self._apply_transition_planner_defaults(tp)

    def _apply_transition_planner_defaults(self, tp: Any) -> None:
        try:
            tp.timeOut(self._transition_time_out)
        except Exception:
            pass
        try:
            tp.maxIterations(self._transition_max_iterations)
        except Exception:
            pass

        # Configure path projector specifically for TransitionPlanner.
        # If not set, fall back to whatever the Problem currently uses.
        if self._transition_path_projector is None:
            try:
                tp.pathProjector(self.problem.pathProjector())
            except Exception:
                pass
            return

        proj_type, step = self._transition_path_projector
        try:
            if str(proj_type) == "Progressive":
                projector = ProgressiveProjector(
                    self.problem.distance(),
                    self.problem.steeringMethod(),
                    float(step),
                )
                tp.pathProjector(projector)
            else:
                # Unknown projector type: best-effort fallback
                tp.pathProjector(self.problem.pathProjector())
        except Exception:
            pass

    def ensure_transition_planner(self) -> Any:
        """Create (or return cached) TransitionPlanner object."""
        if self.problem is None:
            raise RuntimeError("Problem not created yet")
        if self._transition_planner is not None:
            return self._transition_planner

        tp = HPPTransitionPlanner(self.problem)
        self._apply_transition_planner_defaults(tp)
        self._transition_planner = tp
        return tp

    def reset_transition_planner(self) -> None:
        """Forget cached TransitionPlanner object (best-effort cleanup)."""
        self._transition_planner = None

    def _resolve_transition(self, edge: Union[str, Any]) -> Any:
        if self.graph is None:
            raise RuntimeError("Graph not initialized")
        if isinstance(edge, str):
            return self.graph.getTransition(edge)
        return edge

    def set_transition_optimizers(
        self,
        edge: Union[str, Any],
        optimizers: Sequence[str],
        *,
        clear_existing: bool = True,
    ) -> None:
        """Set the optimizer list to use for a given transition edge."""
        tr = self._resolve_transition(edge)
        edge_name = tr.name() if hasattr(tr, "name") else str(edge)
        opts = [str(o) for o in optimizers]
        missing = edge_name not in self._transition_optimizers_by_edge_name
        if clear_existing or missing:
            self._transition_optimizers_by_edge_name[edge_name] = opts
        else:
            self._transition_optimizers_by_edge_name[edge_name].extend(opts)

    def clear_transition_optimizers(
        self, edge: Optional[Union[str, Any]] = None
    ) -> None:
        """Clear per-edge transition optimizer lists."""
        if edge is None:
            self._transition_optimizers_by_edge_name.clear()
            return
        tr = self._resolve_transition(edge)
        edge_name = tr.name() if hasattr(tr, "name") else str(edge)
        self._transition_optimizers_by_edge_name.pop(edge_name, None)

    def _configure_transition_planner_for_edge(self, tp: Any, tr: Any) -> None:
        try:
            tp.setEdge(tr)
        except Exception as exc:
            edge_name = getattr(tr, "name", lambda: str(tr))()
            raise RuntimeError(
                f"Failed to set TransitionPlanner edge {edge_name}: {exc}"
            )

        try:
            tp.clearPathOptimizers()
        except Exception:
            pass

        edge_name = tr.name() if hasattr(tr, "name") else str(tr)
        for opt_name in self._transition_optimizers_by_edge_name.get(
            edge_name, []
        ):
            opt_instance = self._create_path_optimizer_instance(opt_name)
            if opt_instance is None:
                available = sorted(self._path_optimizer_factories().keys())
                raise ValueError(
                    f"Unknown optimizer: {opt_name}. Available: {available}"
                )
            tp.addPathOptimizer(opt_instance)

    def plan_transition_edge(
        self,
        edge: Union[str, Any],
        q1: Sequence[float],
        q2: Sequence[float],
        *,
        validate: bool = True,
        reset_roadmap: bool = True,
        time_parameterize: bool = True,
        store: bool = True,
    ) -> Union[int, Any]:
        """Plan a transition along a specific graph edge.

        Returns:
            If store=True: local stored path id (int).
            If store=False: a PathVector object.
        """
        if self.problem is None:
            raise RuntimeError("Problem not created yet")

        tp = self.ensure_transition_planner()
        tr = self._resolve_transition(edge)
        self._configure_transition_planner_for_edge(tp, tr)

        q1_list = list(q1)
        q2_list = list(q2)

        pv: Any
        try:
            success, path, status = tp.directPath(
                q1_list, q2_list, bool(validate)
            )
        except Exception as exc:
            raise RuntimeError(f"TransitionPlanner.directPath failed: {exc}")

        if success:
            try:
                pv = tp.optimizePath(path)
            except Exception:
                pv = path
        else:
            try:
                # planPath expects a matrix of goals (list of lists)
                pv = tp.planPath(q1_list, [q2_list], bool(reset_roadmap))
            except Exception as exc:
                raise RuntimeError(
                    (
                        "TransitionPlanner failed (directPath status: %s). "
                        "planPath error: %s"
                    )
                    % (status, exc)
                )

        if time_parameterize:
            try:
                pv = tp.timeParameterization(pv)
            except Exception:
                pass

        if not store:
            return pv

        return self.store_path(pv)

    def plan_transition_sequence(
        self,
        edges: Sequence[Union[str, Any]],
        waypoints: Sequence[Sequence[float]],
        *,
        validate: bool = True,
        reset_roadmap: bool = True,
        time_parameterize: bool = True,
        store: bool = True,
    ) -> Union[int, Any]:
        """Plan a multi-edge transition sequence and concatenate results."""
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

        if not store:
            return pv_total

        return self.store_path(pv_total)

    def optimize_path(self, path):
        """Optimize a path using configured optimizers.

        Args:
            path: PathVector to optimize

        Returns:
            Optimized PathVector
        """
        if not self._path_optimizers:
            return path

        optimized = path
        for optimizer in self._path_optimizers:
            optimized = optimizer.optimize(optimized)

        return optimized


# Alias for backward compatibility
PyHPPManipulationPlanner = PyHPPBackend


__all__ = [
    "PyHPPBackend",
    "PyHPPManipulationPlanner",
    "ConstraintResult",
    "HAS_PYHPP",
]
