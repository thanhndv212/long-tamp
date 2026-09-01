"""
Abstract base class for manipulation planning backends.

This module defines the interface that all backends (currently PyHPP) must
implement to ensure interchangeability.
"""

from abc import ABC, abstractmethod
from typing import Any, List, Optional

import numpy as np


class ConstraintResult:
    """Result from applying state constraints.

    Attributes:
        success: Whether constraint application succeeded
        configuration: The projected configuration
        error: Residual error after projection
    """

    def __init__(self, success: bool, configuration: np.ndarray, error: float):
        self.success = success
        self.configuration = configuration
        self.error = error


class BackendBase(ABC):
    """Abstract base class defining the backend interface.

    All manipulation planning backends must implement this interface to ensure
    they can be used interchangeably. The interface covers:

    - Robot/environment/object loading
    - Configuration space operations
    - Constraint graph creation and manipulation
    - Path planning and validation
    - Visualization

    Attributes:
        robot: The loaded robot model (backend-specific type)
        graph: The constraint graph (backend-specific type)
        viewer: The visualization viewer (backend-specific type)
    """

    # =========================================================================
    # Model Access
    # =========================================================================

    @abstractmethod
    def model(self) -> Any:
        """Get the Pinocchio model.

        Returns:
            Pinocchio Model object

        Raises:
            RuntimeError: If robot not loaded yet
        """
        pass

    @abstractmethod
    def data(self) -> Any:
        """Get the Pinocchio data.

        Returns:
            Pinocchio Data object

        Raises:
            RuntimeError: If robot not loaded yet
        """
        pass

    @property
    @abstractmethod
    def nq(self) -> int:
        """Number of configuration variables.

        Returns:
            Size of configuration vector
        """
        pass

    @property
    @abstractmethod
    def nv(self) -> int:
        """Number of velocity variables (DOF).

        Returns:
            Number of degrees of freedom
        """
        pass

    @abstractmethod
    def neutral_config(self) -> np.ndarray:
        """Get neutral/home configuration.

        Returns:
            Neutral configuration as numpy array
        """
        pass

    @abstractmethod
    def random_config(self) -> np.ndarray:
        """Generate a random valid configuration.

        Returns:
            Random configuration as numpy array
        """
        pass

    # =========================================================================
    # Loading Methods
    # =========================================================================

    @abstractmethod
    def load_robot(
        self,
        robot_name: str,
        urdf_path: str,
        srdf_path: Optional[str] = None,
        root_joint_type: str = "anchor",
        composite_name: Optional[str] = None,
        pose: Optional[Any] = None,
    ) -> Any:
        """Load robot model.

        Loading a second (or third, ...) robot inserts it into the same
        composite device rather than replacing the first — the device is
        only (re)created on the first call. Use `pose` to place an
        anchor-rooted robot's base anywhere but world-identity, so multiple
        independent fixed-base robots can share one scene without
        overlapping at the origin.

        Args:
            robot_name: Name identifier for the robot
            urdf_path: Path to URDF file (can be package:// URI)
            srdf_path: Optional path to SRDF file
            root_joint_type: Type of root joint ("anchor", "freeflyer", etc.)
            composite_name: Name for composite robot (defaults to robot_name)
            pose: Optional world pose (pinocchio.SE3) for this robot's root,
                relative to its parent joint. Defaults to identity. Ignored
                by backends/root-joint-types that don't support it.

        Returns:
            Backend-specific robot object
        """
        pass

    @abstractmethod
    def load_environment(self, name: str, urdf_path: str, **kwargs) -> Any:
        """Load environment model.

        Args:
            name: Name identifier for the environment
            urdf_path: Path to URDF file
            **kwargs: Backend-specific options

        Returns:
            Backend-specific environment object or config
        """
        pass

    @abstractmethod
    def load_object(
        self, name: str, urdf_path: str, root_joint_type: str = "freeflyer", **kwargs
    ) -> Any:
        """Load manipulable object.

        Args:
            name: Name identifier for the object
            urdf_path: Path to URDF file
            root_joint_type: Type of root joint (usually "freeflyer")
            **kwargs: Backend-specific options

        Returns:
            Backend-specific object config
        """
        pass

    @abstractmethod
    def set_joint_bounds(self, joint_name: str, bounds: List[float]) -> None:
        """Set bounds for a joint.

        Args:
            joint_name: Name of the joint
            bounds: List of [lower, upper] bounds for each DOF
        """
        pass

    # =========================================================================
    # Configuration Methods
    # =========================================================================

    @abstractmethod
    def set_initial_config(self, q: np.ndarray) -> None:
        """Set initial configuration for planning.

        Args:
            q: Initial configuration vector
        """
        pass

    @abstractmethod
    def add_goal_config(self, q: np.ndarray) -> None:
        """Add a goal configuration for planning.

        Args:
            q: Goal configuration vector
        """
        pass

    # =========================================================================
    # Constraint Graph Methods
    # =========================================================================

    @abstractmethod
    def create_state(
        self, name: str, is_waypoint: bool = False, priority: int = 0
    ) -> int:
        """Create a state in the constraint graph.

        Args:
            name: State name
            is_waypoint: Whether this is a waypoint state
            priority: State priority (higher = more important)

        Returns:
            State ID
        """
        pass

    @abstractmethod
    def create_edge(
        self,
        from_state: str,
        to_state: str,
        name: str,
        weight: int = 1,
        containing_state: Optional[str] = None,
    ) -> int:
        """Create an edge (transition) in the constraint graph.

        Args:
            from_state: Source state name
            to_state: Target state name
            name: Edge name
            weight: Edge weight for planning
            containing_state: State whose constraints apply during edge

        Returns:
            Edge ID
        """
        pass

    @abstractmethod
    def apply_state_constraints(
        self,
        state: str,
        q: np.ndarray,
        max_iterations: int = 10000,
        error_threshold: float = 1e-4,
    ) -> ConstraintResult:
        """Apply state constraints to project a configuration.

        Args:
            state: State name
            q: Input configuration to project
            max_iterations: Maximum projection iterations
            error_threshold: Convergence threshold

        Returns:
            ConstraintResult with success, projected config, and error
        """
        pass

    # =========================================================================
    # Planning Methods
    # =========================================================================

    @abstractmethod
    def solve(self, max_iterations: int = 10000) -> bool:
        """Solve the planning problem.

        Args:
            max_iterations: Maximum planning iterations

        Returns:
            True if solution found, False otherwise
        """
        pass

    @abstractmethod
    def get_path(self, index: int = 0) -> Optional[Any]:
        """Get computed path.

        Args:
            index: Path index (for multiple paths)

        Returns:
            Backend-specific path object, or None if no path
        """
        pass

    @abstractmethod
    def configure_path_validation(
        self, validation_step: float = 0.01, projector_step: float = 0.1
    ) -> "BackendBase":
        """Configure path validation parameters.

        Args:
            validation_step: Step size for path validation
            projector_step: Step size for path projection

        Returns:
            Self for method chaining
        """
        pass

    # =========================================================================
    # Visualization Methods
    # =========================================================================

    @abstractmethod
    def visualize(self, q: Optional[np.ndarray] = None) -> None:
        """Visualize a configuration.

        Args:
            q: Configuration to display. If None, shows current/initial config.
        """
        pass

    @abstractmethod
    def play_path(self, path_index: int = 0) -> None:
        """Play/animate a path in the viewer.

        Args:
            path_index: Index of the path to play
        """
        pass

    def setup_viewer(self, viewer_type: str = "auto") -> None:
        """Explicitly initialise the viewer.

        Call this before ``visualize()`` / ``play_path()`` to choose the
        viewer backend.  If not called, ``visualize()`` will auto-initialise
        on first use with ``viewer_type="auto"``.

        Args:
            viewer_type: Which viewer to create:

                * ``"viser"``   — browser-based 3-D viewer via WebSockets
                  (`pip install viser`).  No X11 server required.
                  Opens at http://localhost:8000 by default.
                * ``"gepetto"`` — Qt-based gepetto-viewer via omniORB.
                  Requires gepetto-viewer-corba running.
                * ``"auto"``    — pick automatically (viser preferred).
        """
        raise NotImplementedError("setup_viewer not implemented for this backend")

    # =========================================================================
    # Path Serialization Methods
    # =========================================================================

    @abstractmethod
    def save_path(self, path_index: int, filename: str) -> None:
        """Save a path to file (binary format).

        Args:
            path_index: Index of the path in ProblemSolver to save
            filename: Output file path

        Raises:
            RuntimeError: If path doesn't exist or save fails
        """
        pass

    @abstractmethod
    def load_path(self, filename: str) -> int:
        """Load a path from file (binary format).

        Args:
            filename: Input file path

        Returns:
            Index of the loaded path in ProblemSolver

        Raises:
            RuntimeError: If file doesn't exist or load fails

        Note:
            The robot must be loaded first with the same name used
            when the path was serialized.
        """
        pass

    def save_path_vector(self, path_vector: Any, filename: str) -> None:
        """Save a PathVector object directly to file.

        Default implementation adds to problem solver first, then saves.
        Subclasses may override for more efficient direct serialization.

        Args:
            path_vector: Backend-specific PathVector object
            filename: Output file path
        """
        # Default: not implemented, subclasses should override
        raise NotImplementedError("save_path_vector not implemented for this backend")

    # =========================================================================
    # Accessor Methods
    # =========================================================================

    @abstractmethod
    def get_robot(self) -> Any:
        """Get the robot object.

        Returns:
            Backend-specific robot object
        """
        pass

    @abstractmethod
    def get_problem(self) -> Any:
        """Get the problem solver/problem object.

        Returns:
            Backend-specific problem object
        """
        pass

    @abstractmethod
    def get_graph(self) -> Any:
        """Get the constraint graph.

        Returns:
            Backend-specific constraint graph object
        """
        pass

    # =========================================================================
    # Constraint Creation Methods
    # =========================================================================

    @abstractmethod
    def create_grasp_constraint(
        self,
        ps,
        name: str,
        gripper: str,
        tool: str,
        transform: List[float],
        mask: List[bool],
    ) -> Any:
        """Create a grasp (relative transformation) constraint.

        Args:
            ps: Problem (PyHPP)
            name: Constraint name
            gripper: Gripper joint/frame name
            tool: Tool joint/frame name
            transform: [x, y, z, qx, qy, qz, qw]
            mask: Boolean mask for constrained DOFs (6 entries)

        Returns:
            Implicit constraint object.
        """
        pass

    @abstractmethod
    def create_placement_constraint(
        self, ps, name: str, tool: str, world_pose: List[float], mask: List[bool]
    ) -> Any:
        """Create a placement (absolute transformation) constraint.

        Args:
            ps: Problem (PyHPP)
            name: Constraint name
            tool: Tool joint/frame name
            world_pose: World pose [x, y, z, qx, qy, qz, qw]
            mask: Boolean mask for constrained DOFs

        Returns:
            Implicit constraint object.
        """
        pass

    @abstractmethod
    def create_complement_constraint(
        self,
        ps,
        base_name: str,
        tool: str,
        world_pose: List[float],
        complement_mask: List[bool],
    ) -> Any:
        """Create a complement (free-DOF) constraint.

        The constraint name is automatically set to ``base_name + '/complement'``.

        Args:
            ps: Problem (PyHPP)
            base_name: Base constraint name (complement suffix appended)
            tool: Tool joint/frame name
            world_pose: World pose [x, y, z, qx, qy, qz, qw]
            complement_mask: Boolean mask for the free DOFs

        Returns:
            Implicit constraint object.
        """
        pass

    @abstractmethod
    def create_locked_joint_constraints(
        self, ps, robot, q_ref: List[float], patterns: List[str]
    ) -> tuple:
        """Create locked joint constraints for joints matching name patterns.

        Args:
            ps: Problem (PyHPP)
            robot: Robot/Device instance
            q_ref: Reference configuration
            patterns: Substrings to match against joint names (case-insensitive)

        Returns:
            Tuple ``(List[LockedJoint] objects, List[str] joint names)``
        """
        pass

    @property
    @abstractmethod
    def skip_placement_for_no_contacts(self) -> bool:
        """Whether placement constraints should be skipped when no contact surfaces exist.

        PyHPP: True — the factory's no-contact path uses LockedJoint foliations;
               pre-registering a Transformation placement constraint breaks projection.
        """
        pass


__all__ = [
    "BackendBase",
    "ConstraintResult",
]
