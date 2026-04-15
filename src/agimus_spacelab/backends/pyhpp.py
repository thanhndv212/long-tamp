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
        SplineGradientBased_bezier1,
        SplineGradientBased_bezier3,
        SplineGradientBased_bezier5,
    )
    HAS_PYHPP = True
except ImportError:
    HAS_PYHPP = False

try:
    from pyhpp.gepetto.viewer import Viewer as _GepettoViewer
    HAS_GEPETTO_VIEWER = True
except ImportError:
    _GepettoViewer = None
    HAS_GEPETTO_VIEWER = False


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
        self.viewer = None
        self.path = None
        self._path_optimizers = []  # List of path optimizer instances

        # Stored paths (local equivalent to CORBA ProblemSolver path ids)
        self._stored_paths: List[Any] = []

        # TransitionPlanner (edge-scoped planning)
        self._transition_planner = None
        # Base timeout for planning (aligned with CORBA)
        self._transition_time_out = 60.0
        # Base iterations for RRT* (aligned with CORBA)
        self._transition_max_iterations = 10000
        # (projector_type, step). If None, fall back to problem.pathProjector()
        self._transition_path_projector: Optional[Tuple[str, float]] = (
            "Progressive",
            0.2,
        )
        # Per-edge optimizer list (edge_name -> optimizer names)
        self._transition_optimizers_by_edge_name: Dict[str, List[str]] = {}

        # Optimizer profiles for different edge types
        # Transit edges (free motion): Use graph-aware shortcut (best available in PyHPP)
        self._transit_edge_optimizers = [
            "GraphRandomShortcut",
        ]
        # Waypoint edges (constrained motion): Use manipulation-aware optimizers
        self._waypoint_pregrasp_optimizers = [
            "ManipulationRandomShortcut",
            # "EnforceTransitionSemantic",
            "SplineGradientBased_bezier3",
        ]
        # Waypoint edges (constrained motion): Use manipulation-aware shortcut
        self._waypoint_grasp_optimizers = [
            "ManipulationRandomShortcut",
        ]
        # Default fallback
        self._transition_default_optimizers = [
            "ManipulationRandomShortcut",
        ]

        # Time parameterization parameters
        self._time_param_order = 2
        self._time_param_max_accel = 0.2

        # Distance-based auto-tuning
        self._enable_distance_tuning = True
        # Scale timeout/iterations by distance
        self._distance_scale_factor = 0.5
        # Inner planner type (PYHPP-GAP: pyhpp HPPTransitionPlanner has no
        # setInnerPlannerType binding; stored for forward-compatibility only)
        self._transition_inner_planner_type: Optional[str] = None

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

            hpp_planner = HPPManipulationPlanner(self.problem)
            hpp_planner.maxIterations(max_iterations)
            success = hpp_planner.solve()

            if success:
                self.path = hpp_planner.path()
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

    def setup_viewer(self):
        """Explicitly initialise the gepetto viewer.

        Call this once before using visualize().  Connecting to the CORBA
        gepetto-viewer server via omniORB can cause a SIGSEGV if the server
        is not running, so viewer creation is intentionally NOT done
        automatically inside visualize().
        """
        if self.device is None:
            raise RuntimeError("Must load robot first")
        if not HAS_GEPETTO_VIEWER:
            raise ImportError("Gepetto viewer not available (omniORB missing)")
        self.viewer = _GepettoViewer(self.device)

    def visualize(self, q: Optional[np.ndarray] = None):
        """Visualize configuration.

        Auto-initialises the viewer on first call if setup_viewer() has not
        been called yet.  If the gepetto-viewer server is not reachable the
        call is silently skipped (no SIGSEGV, no exception propagated).
        """
        if self.viewer is None:
            try:
                self.setup_viewer()
            except Exception:
                return  # server not running or viewer unavailable — skip silently

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
            result = path(t)
            # pyhpp path.__call__ returns (configuration, success) tuple
            if isinstance(result, tuple):
                q_vec, ok = result
                if not ok:
                    t += dt
                    continue
                q = np.array(q_vec)
            else:
                q = np.array(result)
            self.viewer(q)
            time.sleep(dt)
            t += dt

    def store_path(self, path) -> int:
        """Store a PathVector locally and return its integer id."""
        self._stored_paths.append(path)
        return len(self._stored_paths) - 1

    def get_num_stored_paths(self) -> int:
        """Get number of paths currently stored locally.

        Returns:
            Number of stored paths
        """
        return len(self._stored_paths)

    # =========================================================================
    # Path Serialization Methods
    # =========================================================================

    def save_path(self, path_index: int, filename: str) -> None:
        """Save a path to file (binary format).
        
        Uses pyhpp's built-in path serialization via hpp::core::path::Vector.
        
        Args:
            path_index: Index of the path in stored paths to save
            filename: Output file path
            
        Raises:
            RuntimeError: If path doesn't exist or save fails
        """
        if path_index < 0 or path_index >= len(self._stored_paths):
            raise RuntimeError(
                f"Invalid path index {path_index}, "
                f"only {len(self._stored_paths)} paths available"
            )

        path = self._stored_paths[path_index]

        try:
            # pyhpp PathVector has save() method
            if hasattr(path, 'save'):
                path.save(filename)
            else:
                raise RuntimeError(
                    "Path object does not support serialization. "
                    "Ensure pyhpp is built with serialization support."
                )
        except Exception as e:
            raise RuntimeError(f"Failed to save path: {e}") from e

    def load_path(self, filename: str) -> int:
        """Load a path from file (binary format).
        
        Uses pyhpp's built-in path deserialization.
        
        Args:
            filename: Input file path
            
        Returns:
            Index of the loaded path in stored paths
            
        Raises:
            RuntimeError: If file doesn't exist or load fails
        """
        try:
            # Import the PathVector class
            from pyhpp.core.path import Vector as PathVector

            # Load using static method
            if hasattr(PathVector, 'load'):
                path = PathVector.load(filename)
            else:
                raise RuntimeError(
                    "PathVector.load() not available. "
                    "Ensure pyhpp is built with serialization support."
                )

            # Store and return index
            return self.store_path(path)
        except Exception as e:
            raise RuntimeError(f"Failed to load path from {filename}: {e}") from e

    def save_path_vector(self, path_vector: Any, filename: str) -> None:
        """Save a PathVector object directly to file.
        
        Args:
            path_vector: PathVector object
            filename: Output file path
        """
        try:
            if hasattr(path_vector, 'save'):
                path_vector.save(filename)
            else:
                raise RuntimeError(
                    "PathVector object does not support serialization"
                )
        except Exception as e:
            raise RuntimeError(f"Failed to save path vector: {e}") from e

    def load_paths_from_directory(
        self,
        directory: str,
        pattern: str = "phase_*.path",
        sort: bool = True,
    ) -> List[int]:
        """Load multiple paths from a directory.
        
        Args:
            directory: Directory containing path files
            pattern: Glob pattern for path files (default: phase_*.path)
            sort: If True, sort files by name before loading
            
        Returns:
            List of path indices in stored paths
        """
        import glob
        import os

        search_path = os.path.join(directory, pattern)
        files = glob.glob(search_path)

        if sort:
            files.sort()

        indices = []
        for filepath in files:
            try:
                idx = self.load_path(filepath)
                indices.append(idx)
                print(f"  Loaded {os.path.basename(filepath)} -> index {idx}")
            except Exception as e:
                print(f"  Failed to load {filepath}: {e}")

        return indices

    # =========================================================================
    # Graph Metadata Extraction
    # =========================================================================

    def extract_graph_metadata(self) -> Dict[str, Any]:
        """Extract constraint graph metadata for serialization.

        Returns a dictionary containing graph structure information that can
        be used to reconstruct a minimal graph for path loading.

        Returns:
            Dictionary with graph metadata:
            - states: List of state names
            - edges: List of (edge_name, from_state, to_state)
            - robot_name: Name of the robot
            - objects: List of object names (if available)

        Raises:
            RuntimeError: If graph is not initialized
        """
        if self.graph is None:
            raise RuntimeError("Constraint graph not initialized")

        metadata = {
            "format_version": "1.0",
            "robot_name": self.device.name if self.device else None,
            "states": [],
            "edges": [],
            "objects": [],
        }

        # Extract states from graph
        try:
            if hasattr(self.graph, "states"):
                states = self.graph.states
                if isinstance(states, dict):
                    metadata["states"] = list(states.keys())
                elif isinstance(states, list):
                    metadata["states"] = [
                        str(s.name() if hasattr(s, "name") else s)
                        for s in states
                    ]
        except Exception as e:
            print(f"Warning: Could not extract states: {e}")

        # Extract edges from graph
        try:
            if hasattr(self.graph, "edges"):
                edges = self.graph.edges
                if isinstance(edges, dict):
                    for edge_name, edge_obj in edges.items():
                        edge_info = {"name": edge_name}
                        # Try to get edge endpoints if available
                        try:
                            if hasattr(edge_obj, "state_from"):
                                edge_info["from"] = str(
                                    edge_obj.state_from.name()
                                    if hasattr(edge_obj.state_from, "name")
                                    else edge_obj.state_from
                                )
                            if hasattr(edge_obj, "state_to"):
                                edge_info["to"] = str(
                                    edge_obj.state_to.name()
                                    if hasattr(edge_obj.state_to, "name")
                                    else edge_obj.state_to
                                )
                        except Exception:
                            pass
                        metadata["edges"].append(edge_info)
        except Exception as e:
            print(f"Warning: Could not extract edges: {e}")

        # Extract object names if available
        try:
            if self.device and hasattr(self.device, "robot_names"):
                all_names = self.device.robot_names
                # Filter out the main robot name
                robot_name = metadata["robot_name"]
                metadata["objects"] = [n for n in all_names if n != robot_name]
        except Exception as e:
            print(f"Warning: Could not extract object names: {e}")

        return metadata

    # =========================================================================
    # Path Serialization Methods
    # =========================================================================

    def save_path_as_waypoints(
        self,
        path_vector: Any,
        filename: str,
        num_samples: int = 100,
        edge_name: Optional[str] = None,
    ) -> None:
        """Save a path as waypoints in JSON format (portable across sessions).

        This method samples configurations along the path and saves them as
        a JSON file with graph metadata for validation on load.

        Args:
            path_vector: PathVector object
            filename: Output file path (will add .json if not present)
            num_samples: Number of waypoints to sample along the path
            edge_name: Optional edge name for reconstruction context

        Raises:
            RuntimeError: If path is invalid or save fails
        """
        import json
        import os

        # Ensure .json extension
        if not filename.endswith('.json'):
            filename = filename + '.json'

        try:
            # Get path length
            path_length = path_vector.length()

            # Sample configurations along the path
            waypoints = []
            for i in range(num_samples):
                t = (i / max(1, num_samples - 1)) * path_length
                # Call path at time t to get configuration
                q, success = path_vector.call(t)
                if success:
                    waypoints.append(list(q))
                else:
                    # If call fails, skip this waypoint
                    continue

            # Extract graph metadata for reconstruction (if available)
            graph_metadata = None
            try:
                if hasattr(self, "extract_graph_metadata"):
                    graph_metadata = self.extract_graph_metadata()
            except Exception as e:
                print(f"Warning: Could not extract graph metadata: {e}")
                print(
                    "Path can still be saved but may require manual graph setup to load."
                )

            # Create data structure
            data = {
                "format_version": "1.0",
                "path_length": path_length,
                "num_samples": num_samples,
                "waypoints": waypoints,
                "edge_name": edge_name,
                "graph_metadata": graph_metadata,
            }

            # Write to file
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)

        except Exception as e:
            raise RuntimeError(f"Failed to save path as waypoints: {e}") from e

    def load_path_from_waypoints(
        self,
        filename: str,
        add_to_problem: bool = True,
        auto_setup_graph: bool = False,
    ) -> int:
        """Load a path from JSON waypoints file.

        Reconstructs a path by interpolating between waypoints using the
        steering method. The path is added to the stored paths.

        GRAPH RECONSTRUCTION:
        If the JSON file contains graph metadata (format_version 1.0+), you have
        two options:

        Option 1 (Recommended): Set up the graph manually before loading
        - Load robot/objects with the same setup as when path was saved
        - Create constraint graph with same structure
        - Call load_path_from_waypoints()

        Option 2: Use auto_setup_graph=True (Experimental)
        - Validates that graph metadata is present
        - Checks if current graph matches saved metadata
        - Provides clear error if graph doesn't match

        Args:
            filename: Input JSON file path
            add_to_problem: If True, add the path to stored paths and return index
            auto_setup_graph: If True, validate graph metadata matches current setup

        Returns:
            Index of the loaded path in stored paths (if add_to_problem=True)

        Raises:
            RuntimeError: If file doesn't exist, invalid format, or reconstruction fails
            RuntimeError: If graph metadata doesn't match current setup
        """
        import json
        import os

        if not os.path.exists(filename):
            raise RuntimeError(f"File not found: {filename}")

        try:
            # Read waypoints
            with open(filename, 'r') as f:
                data = json.load(f)

            # Check for graph metadata and validate if requested
            graph_metadata = data.get("graph_metadata")

            if auto_setup_graph:
                if not graph_metadata:
                    raise RuntimeError(
                        f"Cannot auto-setup graph: No graph metadata in {filename}. "
                        "This file was saved without graph metadata. You must manually "
                        "set up the constraint graph before loading."
                    )

                # Validate current graph matches metadata
                self._validate_graph_metadata(graph_metadata)
                print("✓ Graph metadata validated successfully")
            elif graph_metadata:
                # Metadata present but not validating - provide helpful info
                num_states = len(graph_metadata.get("states", []))
                robot_name = graph_metadata.get("robot_name", "unknown")
                if num_states > 0:
                    print(
                        f"Note: File contains graph metadata for robot '{robot_name}' "
                        f"with {num_states} states. Make sure your graph setup matches."
                    )

            waypoints = data["waypoints"]
            if len(waypoints) < 2:
                raise RuntimeError("Need at least 2 waypoints to reconstruct path")

            # Create path segments between consecutive waypoints
            # Use the steering method to interpolate
            path_segments = []

            if self.problem is None:
                raise RuntimeError("Problem not initialized")

            for i in range(len(waypoints) - 1):
                q1 = waypoints[i]
                q2 = waypoints[i + 1]

                # Use steer to create a path between q1 and q2
                try:
                    steeringMethod = self.problem.steeringMethod()
                    segment = steeringMethod.call(q1, q2)
                    if segment is not None:
                        path_segments.append(segment)
                except Exception as e:
                    # If steering fails, skip this segment
                    print(f"Warning: Failed to create segment {i} -> {i+1}: {e}")
                    continue

            if not path_segments:
                raise RuntimeError("Failed to create any valid path segments")

            # Concatenate all segments into a PathVector
            # Create a PathVector and append all segments
            path_vector = path_segments[0].asVector()
            for segment in path_segments[1:]:
                path_vector.appendPath(segment)

            if add_to_problem:
                # Add to stored paths
                path_index = self.store_path(path_vector)
                return path_index
            else:
                return path_vector

        except json.JSONDecodeError as e:
            raise RuntimeError(f"Invalid JSON format in {filename}: {e}") from e
        except KeyError as e:
            raise RuntimeError(f"Missing required field in JSON: {e}") from e
        except Exception as e:
            raise RuntimeError(f"Failed to load path from waypoints: {e}") from e

    def _validate_graph_metadata(self, saved_metadata: Dict[str, Any]) -> None:
        """Validate current graph matches saved metadata.

        Args:
            saved_metadata: Graph metadata from saved file

        Raises:
            RuntimeError: If validation fails
        """
        if self.graph is None:
            raise RuntimeError(
                "Constraint graph not initialized. The saved path requires a graph "
                f"with {len(saved_metadata.get('states', []))} states. "
                "Set up the graph first using the same robot/objects/structure."
            )

        # Extract current metadata
        try:
            current_metadata = self.extract_graph_metadata()
        except Exception as e:
            raise RuntimeError(
                f"Failed to extract current graph metadata: {e}. "
                "Cannot validate graph structure."
            )

        # Validate robot name
        saved_robot = saved_metadata.get("robot_name")
        current_robot = current_metadata.get("robot_name")
        if saved_robot and current_robot and saved_robot != current_robot:
            print(
                f"Warning: Robot name mismatch. Saved: '{saved_robot}', "
                f"Current: '{current_robot}'"
            )

        # Validate state count
        saved_states = saved_metadata.get("states", [])
        current_states = current_metadata.get("states", [])
        if len(saved_states) != len(current_states):
            raise RuntimeError(
                f"Graph structure mismatch: Saved path has {len(saved_states)} states, "
                f"but current graph has {len(current_states)} states. "
                "Make sure you're using the same constraint graph structure."
            )

        # Validate edge count (if available)
        saved_edges = saved_metadata.get("edges", [])
        current_edges = current_metadata.get("edges", [])
        if (
            saved_edges
            and current_edges
            and len(saved_edges) != len(current_edges)
        ):
            print(
                f"Warning: Edge count mismatch. Saved: {len(saved_edges)}, "
                f"Current: {len(current_edges)}"
            )

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
            self.problem.pathValidation(Discretized(
                self.device, validation_step
            ))
        # Configure path projection if enabled
        if self._use_progressive_projector:
            self.problem.pathProjector(ProgressiveProjector(
                self.problem.distance(),
                self.problem.steeringMethod(),
                projector_step,
            ))
        return self

    def set_path_optimization(self, enabled: bool):
        """Enable or disable path optimization.

        Args:
            enabled: Whether to use path optimization
        """
        self._use_path_optimization = enabled

    def set_path_projection(self, enabled: bool) -> None:
        """Enable or disable progressive path projection during planning."""
        self._use_progressive_projector = bool(enabled)

    def clear_stored_paths(self, verbose: bool = True) -> int:
        """Clear all locally stored paths.

        Returns:
            Number of paths cleared.
        """
        count = len(self._stored_paths)
        self._stored_paths.clear()
        if verbose:
            print(f"   Cleared {count} stored path(s).")
        return count

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

                Spline optimizers (pyhpp.core):
                - "SplineGradientBased_bezier1": Bezier order-1 spline
                - "SplineGradientBased_bezier3": Bezier cubic spline
                - "SplineGradientBased_bezier5": Bezier order-5 spline
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
            available = sorted(self._path_optimizer_factories().keys())
            print(
                f"      [warn] Unknown optimizer: {optimizer!r} "
                f"(not available in pyhpp; requires CORBA plugin). "
                f"Falling back to 'RandomShortcut'. "
                f"Available: {available}"
            )
            opt_instance = self._create_path_optimizer_instance("RandomShortcut")
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
            # Spline gradient-based optimizers (bezier polynomial)
            "SplineGradientBased_bezier1": lambda: SplineGradientBased_bezier1(prob),
            "SplineGradientBased_bezier3": lambda: SplineGradientBased_bezier3(prob),
            "SplineGradientBased_bezier5": lambda: SplineGradientBased_bezier5(prob),
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

    def _is_pregrasp_edge(self, edge_name: str) -> bool:
        """Check if edge is a pregrasp edge (constrained motion).
        
        Pregrasp edges (_01, _10) represent constrained motion
        like pregrasp->grasp transitions. They benefit less from spline
        optimization compared to free-space transit edges.
        """
        return any(suffix in edge_name for suffix in ["_01", "_10"])

    def _is_grasp_edge(self, edge_name: str) -> bool:
        """Check if edge is a grasp edge (constrained motion).
        
        Grasp edges (_12, _21) represent constrained motion
        like grasp->pregrasp transitions. They benefit less from spline
        optimization compared to free-space transit edges.
        """
        return any(suffix in edge_name for suffix in ["_12", "_21"])

    def _compute_config_distance(
        self, q1: Sequence[float], q2: Sequence[float]
    ) -> float:
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
            return (self._transition_time_out, self._transition_max_iterations)

        distance = self._compute_config_distance(q1, q2)
        # Normalize by typical arm reach (~2.0 rad per joint)
        # For 20 DOF system, typical max distance ~40
        normalized_dist = min(distance / 40.0, 1.0)
        scale = 1.0 + normalized_dist * self._distance_scale_factor

        timeout = self._transition_time_out * scale
        iterations = int(self._transition_max_iterations * scale)
        return (timeout, iterations)

    def _project_onto_leaf(
        self,
        edge: Any,
        q_rhs: np.ndarray,
        q_target: np.ndarray,
        edge_name: str = "unknown",
    ) -> np.ndarray:
        """Project q_target onto the constraint leaf defined by q_rhs.
        
        Args:
            edge: Transition edge object
            q_rhs: Configuration defining the leaf (typically q1/q_init)
            q_target: Configuration to project (typically q2/q_goal)
            edge_name: Edge name for error messages
            
        Returns:
            Projected configuration
            
        Raises:
            RuntimeError: If projection unavailable or fails
        """
        if not hasattr(self.graph, 'applyLeafConstraints'):
            raise RuntimeError(
                f"PYHPP-GAP: Cannot project q_goal onto leaf for edge '{edge_name}'. "
                "applyLeafConstraints not available in this pyhpp build. "
                "This is required for TransitionPlanner. "
                "Workaround: use backend='corba' or rebuild hpp-python with full bindings."
            )

        try:
            ok, q_projected, residual = self.graph.applyLeafConstraints(
                edge, q_rhs, q_target
            )

            if not ok:
                residual_norm = float(np.linalg.norm(np.asarray(residual, dtype=float)))
                threshold = float(self.graph.errorThreshold())
                raise RuntimeError(
                    f"Failed to project q_goal onto leaf for edge '{edge_name}'. "
                    f"Residual={residual_norm:.6f}, threshold={threshold:.6f}. "
                    "The goal configuration may be incompatible with edge constraints."
                )

            return np.asarray(q_projected, dtype=float)

        except RuntimeError:
            raise  # Re-raise our own errors
        except Exception as exc:
            raise RuntimeError(
                f"applyLeafConstraints failed for edge '{edge_name}': {exc}"
            ) from exc

    def configure_transition_planner(
        self,
        *,
        inner_planner_type: Optional[str] = None,
        time_out: Optional[float] = None,
        max_iterations: Optional[int] = None,
        path_projector: Optional[Tuple[str, float]] = None,
        enable_distance_tuning: Optional[bool] = None,
        distance_scale_factor: Optional[float] = None,
    ) -> None:
        """Configure defaults for TransitionPlanner edge-scoped planning.
        
        Args:
            inner_planner_type: Inner planner type string (e.g. "DiffusingPlanner").
                PYHPP-GAP: pyhpp HPPTransitionPlanner has no setInnerPlannerType
                binding; the value is stored but cannot be applied.
            time_out: Base timeout in seconds
            max_iterations: Base maximum iterations
            path_projector: (projector_type, step) tuple
            enable_distance_tuning: Enable auto-scaling timeout/iterations
                by distance
            distance_scale_factor: Scale factor for distance-based tuning
        """
        if inner_planner_type is not None:
            self._transition_inner_planner_type = inner_planner_type
            # PYHPP-GAP: pyhpp HPPTransitionPlanner has no setInnerPlannerType
            # binding. The value is stored for forward-compatibility only.
        if time_out is not None:
            self._transition_time_out = float(time_out)
        if max_iterations is not None:
            self._transition_max_iterations = int(max_iterations)
        if path_projector is not None:
            self._transition_path_projector = path_projector
        if enable_distance_tuning is not None:
            self._enable_distance_tuning = bool(enable_distance_tuning)
        if distance_scale_factor is not None:
            self._distance_scale_factor = float(distance_scale_factor)

        tp = self._transition_planner
        if tp is not None:
            self._apply_transition_planner_defaults(tp)

    def configure_time_parameterization(
        self,
        order: Optional[int] = None,
        max_acceleration: Optional[float] = None,
    ) -> None:
        """Configure time parameterization parameters.
        
        Args:
            order: Polynomial order for time parameterization
            max_acceleration: Maximum acceleration limit
        
        Note:
            In PyHPP, time parameterization is typically set when creating
            the SimpleTimeParameterization optimizer instance. These settings
            are stored for future use when creating optimizers.
        """
        if order is not None:
            self._time_param_order = int(order)
        if max_acceleration is not None:
            self._time_param_max_accel = float(max_acceleration)

    def set_inner_problem_parameter(self, key: str, value: Any) -> None:
        """Set parameter on TransitionPlanner's inner problem.
        
        Args:
            key: Parameter name (e.g., "kRRT*")
            value: Parameter value
        
        Note:
            In PyHPP, parameters are typically set differently than in CORBA.
            This method attempts to set parameters on the problem if the
            TransitionPlanner exposes it, otherwise stores for later use.
        """
        tp = self.ensure_transition_planner()
        try:
            # Try to access inner problem if TransitionPlanner exposes it
            if hasattr(tp, 'problem'):
                inner_problem = tp.problem()
                if hasattr(inner_problem, 'setParameter'):
                    inner_problem.setParameter(key, value)
            # Fallback: set on main problem
            elif self.problem is not None:
                if hasattr(self.problem, 'setParameter'):
                    self.problem.setParameter(key, value)
        except Exception as e:
            print(
                f"Warning: Could not set inner problem parameter "
                f"'{key}': {e}"
            )

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

    def _resolve_transition(self, edge: Union[int, str, Any]) -> Any:
        if self.graph is None:
            raise RuntimeError("Graph not initialized")
        if isinstance(edge, int):
            # PYHPP-GAP: pyhpp PyWEdge has no integer ID lookup.
            # Fall back to positional index over getTransitions().
            transitions = self.graph.getTransitions()
            if 0 <= edge < len(transitions):
                return transitions[edge]
            raise ValueError(
                f"Transition index {edge} out of range "
                f"(graph has {len(transitions)} transitions)"
            )
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
        """Configure TransitionPlanner for a specific edge.
        
        Applies edge-type-aware optimizer selection based on edge
        naming patterns.
        """
        edge_name = tr.name() if hasattr(tr, "name") else str(tr)

        try:
            tp.setEdge(tr)
            print(f"      [TP] Set edge: {edge_name}")
        except Exception as exc:
            raise RuntimeError(
                f"Failed to set TransitionPlanner edge {edge_name}: {exc}"
            )

        try:
            tp.clearPathOptimizers()
        except Exception:
            pass

        # Determine which optimizer list to use based on edge type
        if edge_name in self._transition_optimizers_by_edge_name:
            # Use explicit per-edge override if set
            optimizer_names = (
                self._transition_optimizers_by_edge_name[edge_name]
            )
            print("      [TP] Using edge-specific optimizers")
        elif self._is_pregrasp_edge(edge_name):
            # Pregrasp edges: constrained motion
            optimizer_names = self._waypoint_pregrasp_optimizers
            print("      [TP] Using pregrasp edge optimizers")
        elif self._is_grasp_edge(edge_name):
            # Grasp edges: constrained motion
            optimizer_names = self._waypoint_grasp_optimizers
            print("      [TP] Using grasp edge optimizers")
        elif "Loop" in edge_name or "transit" in edge_name.lower():
            # Transit edges: free-space motion
            optimizer_names = self._transit_edge_optimizers
            print("      [TP] Using transit edge optimizers")
        else:
            # Default fallback
            optimizer_names = self._transition_default_optimizers
            print("      [TP] Using default optimizers")

        for opt_name in optimizer_names:
            opt_instance = self._create_path_optimizer_instance(opt_name)
            if opt_instance is None:
                available = sorted(self._path_optimizer_factories().keys())
                raise ValueError(
                    f"Unknown optimizer: {opt_name}. Available: {available}"
                )
            tp.addPathOptimizer(opt_instance)
            print(f"      [TP] Added optimizer: {opt_name}")

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
    ) -> Tuple[Union[int, Any], Any]:
        """Plan a transition along a specific graph edge.
        
        PYHPP-GAP: Requires graph.applyLeafConstraints() binding.
        - CORBA backend: Leaf projection happens internally in C++
        - PyHPP backend: Must explicitly call applyLeafConstraints() in Python
        
        If applyLeafConstraints is not available, this method will raise
        RuntimeError. Workaround: use backend='corba' or rebuild hpp-python.
        
        Implements smart planning strategy:
        - Applies distance-based timeout/iteration scaling
        - Skips directPath for waypoint pregrasp/grasp edges
        - Validates configuration before planning
        - Tries directPath first for transit edges, falls back to planPath

        Returns:
            Tuple of (timed_path, geometric_path) where:
            - timed_path: If store=True, local stored path id (int).
                         If store=False, a PathVector object.
            - geometric_path: PathVector without time parameterization (for serialization)
        """
        if self.problem is None:
            raise RuntimeError("Problem not created yet")

        tp = self.ensure_transition_planner()
        tr = self._resolve_transition(edge)
        edge_name = tr.name() if hasattr(tr, "name") else str(edge)

        # Apply distance-based timeout/iteration scaling
        timeout, max_iter = self._compute_planning_budget(q1, q2)
        try:
            tp.timeOut(timeout)
            tp.maxIterations(max_iter)
        except Exception:
            pass
        print(
            f"      [TP] Planning budget: "
            f"timeout={timeout:.1f}s, max_iter={max_iter}"
        )

        self._configure_transition_planner_for_edge(tp, tr)

        # pyhpp C++ bindings require numpy arrays, not Python lists.
        # planPath expects q_goals as shape (numGoals, configSize).
        q1_arr = np.asarray(q1, dtype=float)
        q2_arr = np.asarray(q2, dtype=float)

        # Validate configurations before planning (if requested)
        if validate:
            # PYHPP-WORKAROUND: Use graph.getNodesConnectedByTransition() to
            # get the source/dest states for an edge, then validate configs
            # with graph.getConfigErrorForState(state, q).
            state_from = None
            state_to = None

            if self.graph is not None and hasattr(
                self.graph, 'getNodesConnectedByTransition'
            ):
                try:
                    nodes = self.graph.getNodesConnectedByTransition(tr)
                    if nodes and len(nodes) >= 2:
                        # getNodesConnectedByTransition returns state names (str)
                        # — convert to PyWState objects via getState()
                        state_from = self.graph.getState(nodes[0])
                        state_to = self.graph.getState(nodes[1])
                except Exception as exc:
                    print(
                        f"      [TP] Warning: getNodesConnectedByTransition"
                        f"('{edge_name}') failed: {exc}"
                    )

            has_states = state_from is not None and state_to is not None
            if has_states:
                state_from_name = (
                    state_from.name()
                    if hasattr(state_from, 'name') and callable(state_from.name)
                    else str(state_from)
                )
                state_to_name = (
                    state_to.name()
                    if hasattr(state_to, 'name') and callable(state_to.name)
                    else str(state_to)
                )

                # Validate q1 against source state
                try:
                    # getConfigErrorForState returns (error_vector, belongs)
                    err_q1, belongs_q1 = self.graph.getConfigErrorForState(
                        state_from, q1_arr
                    )
                    error_q1_scalar = float(np.linalg.norm(
                        np.asarray(err_q1, dtype=float)
                    ))
                    threshold = float(self.graph.errorThreshold())
                    success_q1 = error_q1_scalar < threshold
                    print(
                        f"      [TP] Validate q1 in state '{state_from_name}': "
                        f"{'✓' if success_q1 else '✗'} "
                        f"(error={error_q1_scalar:.6f}, "
                        f"threshold={threshold:.6f})"
                    )
                    if not success_q1:
                        print(
                            f"      [TP] Warning: q1 may not satisfy "
                            f"'{state_from_name}' constraints "
                            f"(error={error_q1_scalar:.6f}); "
                            "proceeding — TransitionPlanner will validate."
                        )
                except Exception as exc:
                    print(f"      [TP] Warning: Could not validate q1: {exc}")

                # Validate q2 against destination state
                try:
                    # getConfigErrorForState returns (error_vector, belongs)
                    err_q2, belongs_q2 = self.graph.getConfigErrorForState(
                        state_to, q2_arr
                    )
                    error_q2_scalar = float(np.linalg.norm(
                        np.asarray(err_q2, dtype=float)
                    ))
                    threshold = float(self.graph.errorThreshold())
                    success_q2 = error_q2_scalar < threshold
                    print(
                        f"      [TP] Validate q2 in state '{state_to_name}': "
                        f"{'✓' if success_q2 else '✗'} "
                        f"(error={error_q2_scalar:.6f}, "
                        f"threshold={threshold:.6f})"
                    )
                    if not success_q2:
                        print(
                            f"      [TP] Warning: q2 may not satisfy "
                            f"'{state_to_name}' constraints "
                            f"(error={error_q2_scalar:.6f}); "
                            "proceeding — TransitionPlanner will validate."
                        )
                except Exception as exc:
                    print(f"      [TP] Warning: Could not validate q2: {exc}")
            else:
                print(
                    "      [TP] Warning: Cannot validate configurations "
                    "(graph or transition states not available)"
                )

        # MANDATORY: Project q2 onto the constraint leaf defined by q1.
        # TransitionPlanner strictly requires this (CORBA does it internally, pyhpp doesn't).
        try:
            q2_arr = self._project_onto_leaf(tr, q1_arr, q2_arr, edge_name)
        except RuntimeError as exc:
            # Provide actionable guidance
            raise RuntimeError(
                f"Cannot plan edge '{edge_name}' from q1 to q2: {exc}\n"
                "Possible solutions:\n"
                "  1. Use backend='corba' instead of 'pyhpp'\n"
                "  2. Rebuild hpp-python with full manipulation bindings\n"
                "  3. Ensure q2 was generated on the correct constraint manifold"
            ) from exc

        # Smart planning strategy based on edge type
        is_waypoint_pregrasp = self._is_pregrasp_edge(edge_name)
        is_waypoint_grasp = self._is_grasp_edge(edge_name)
        # Allow directPath for _21 (grasped->pregrasp = release retraction).
        # q_pregrasp is generated with q_start as hint, so non-active-arm DOFs
        # are unchanged -- linear interpolation is a pure arm retraction.
        # RRT for _12 (forward approach to contact) and _01/_10 stays unchanged.
        is_release_retract_edge = "_21" in edge_name
        skip_direct_path = is_waypoint_pregrasp or (
            is_waypoint_grasp and not is_release_retract_edge
        )

        pv: Any = None

        # Try directPath first for transit and release-retraction edges
        if not skip_direct_path:
            label = "release retract (_21)" if is_release_retract_edge else "transit"
            print(f"      [TP] {label} edge, trying directPath first")
            try:
                success, path, status = tp.directPath(
                    q1_arr, q2_arr, bool(validate)
                )
                if success:
                    print("      [TP] directPath succeeded")
                    try:
                        pv = tp.optimizePath(path)
                        print("      [TP] Path optimized")
                    except Exception:
                        print(
                            "      [TP] Optimization failed, "
                            "using unoptimized path"
                        )
                        pv = path
                else:
                    print(f"      [TP] directPath failed: {status}")
            except Exception as exc:
                # directPath failed, will fall back to planPath
                print(f"      [TP] directPath threw exception: {exc}")
        else:
            edge_type = "pregrasp" if is_waypoint_pregrasp else "forward grasp (_12)"
            print(
                f"      [TP] Waypoint {edge_type} edge, "
                "skipping directPath"
            )

        # Fall back to planPath if directPath didn't work or was skipped
        if pv is None:
            print("      [TP] Falling back to planPath")
            try:
                # Use planPathSingle (1D, 1D) instead of planPath (1D, 2D-matrix).
                # The 2D-matrix eigenpy conversion for (1,N) arrays is broken:
                # eigenpy falsely considers (1,N) C-order arrays as F-contiguous
                # and creates a direct Ref, but the resulting Ref has wrong strides
                # so C++ reads garbage from positions [1..N-1].
                # planPathSingle constructs the MatrixXd internally in C++ from
                # two 1D ConfigurationIn_t refs, which always work correctly.
                pv = tp.planPathSingle(q1_arr, q2_arr, bool(reset_roadmap))
                print("      [TP] planPath succeeded")
                try:
                    pv = tp.optimizePath(pv)
                    print("      [TP] Path optimized")
                except Exception:
                    print(
                        "      [TP] Optimization failed, "
                        "using unoptimized path"
                    )
            except Exception as exc:
                edge_type = "waypoint" if skip_direct_path else "transit"
                raise RuntimeError(
                    f"TransitionPlanner.planPath failed for {edge_type} edge "
                    f"{edge_name}: {exc}"
                )

        if pv is None:
            return None, None

        # Store geometric path before time parameterization (for serialization)
        pv_geometric = pv

        if time_parameterize:
            try:
                pv = tp.timeParameterization(pv)
                print("      [TP] Path time-parameterized")
            except Exception as e:
                print(f"      [TP] Time parameterization failed: {e}")
        else:
            print("      [TP] Skipping time parameterization")

        if not store:
            return pv, pv_geometric

        path_id = self.store_path(pv)
        print(f"      [TP] Path stored with ID: {path_id}")
        return path_id, pv_geometric

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
        """Plan a multi-edge transition sequence and concatenate results.
        
        IMPORTANT: Each waypoint is re-projected onto its edge's constraint leaf
        to ensure leaf consistency across the sequence.
        """
        if len(waypoints) != len(edges) + 1:
            raise ValueError(
                "Expected len(waypoints) == len(edges) + 1, got %d and %d"
                % (len(waypoints), len(edges))
            )

        pv_total: Optional[Any] = None

        # Pre-project all waypoints onto their respective edge leaves
        # to ensure constraint consistency across the sequence
        projected_waypoints = []

        for i, edge in enumerate(edges):
            tr = self._resolve_transition(edge)
            edge_name = tr.name() if hasattr(tr, "name") else str(edge)

            # Convert waypoints to numpy arrays
            q1 = np.asarray(waypoints[i], dtype=float)
            q2 = np.asarray(waypoints[i + 1], dtype=float)

            # Project both q1 and q2 onto this edge's leaf
            # (q1 defines the leaf, q2 is projected onto it)
            try:
                q2_proj = self._project_onto_leaf(tr, q1, q2, edge_name)

                # Store projected configs
                if i == 0:
                    projected_waypoints.append(q1)  # First waypoint (initial config)
                projected_waypoints.append(q2_proj)

                print(f"      [seq] Edge {i}: '{edge_name}' — waypoints projected")

            except RuntimeError as exc:
                raise RuntimeError(
                    f"Cannot project waypoints for edge {i} ('{edge_name}'): {exc}\n"
                    "Multi-edge sequences require leaf projection support."
                ) from exc

        # Now plan each edge with projected waypoints
        for i, edge in enumerate(edges):
            pv_i, _ = self.plan_transition_edge(
                edge,
                projected_waypoints[i],
                projected_waypoints[i + 1],
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

    def play_path_vector(self, path_vector: Any, speed: float = 1.0) -> int:
        """Store a PathVector locally and play it in the viewer.

        Args:
            path_vector: PathVector object or integer path ID (already stored).
            speed: Playback speed multiplier (currently unused; reserved).

        Returns:
            Index of the stored path.
        """
        # plan_transition_edge returns an int id when store=True; accept it directly
        if isinstance(path_vector, int):
            path_index = path_vector
        else:
            path_index = self.store_path(path_vector)
        if self.viewer is None:
            self.visualize()
        self.play_path(path_index)
        return path_index

    def play_path_vector_with_viz(
        self,
        path_vector: Any,
        graph_builder=None,
        edge_name: Optional[str] = None,
        visualizer=None,
        speed: float = 1.0,
    ) -> int:
        """Store a PathVector and play it with optional live graph visualization.

        Args:
            path_vector: PathVector to store and play.
            graph_builder: GraphBuilder instance (for LivePathPlayer).
            edge_name: Edge name being traversed (for graph highlighting).
            visualizer: LiveConstraintGraphVisualizer instance.
            speed: Playback speed multiplier (reserved).

        Returns:
            Index of the stored path.
        """
        # plan_transition_edge returns an int id when store=True; accept it directly
        if isinstance(path_vector, int):
            path_index = path_vector
        else:
            path_index = self.store_path(path_vector)
        if self.viewer is None:
            self.visualize()
        if visualizer is not None and graph_builder is not None:
            try:
                from agimus_spacelab.visualization.live_graph_viz import (
                    LivePathPlayer,
                )
                live_player = LivePathPlayer(
                    self, graph_builder, visualizer=visualizer
                )
                live_player.play_with_visualization(path_index, edge_name)
            except Exception as exc:
                print(
                    f"   Live visualization failed ({exc}); "
                    "falling back to simple playback."
                )
                self.play_path(path_index)
        else:
            self.play_path(path_index)
        return path_index

    def play_and_record_path(
        self,
        path_index: int = 0,
        video_name: Optional[str] = None,
        output_dir: str = "/home/dvtnguyen/devel/demos",
        framerate: int = 25,
        dt: float = 0.01,
        speed: float = 1.0,
    ) -> str:
        """Play a stored path and record it to video.

        Args:
            path_index: Index of the stored path to play.
            video_name: Output video filename without extension.
            output_dir: Directory for output video and frames.
            framerate: Video framerate in fps.
            dt: Time step for path sampling.
            speed: Playback speed multiplier.

        Returns:
            Path to generated video file, or empty string if recording
            is unavailable (PYHPP-GAP: gepetto capture may not be supported).
        """
        if self.viewer is None:
            self.visualize()
        from agimus_spacelab.visualization.video_recorder import VideoRecorder
        recorder = VideoRecorder(
            self.viewer, output_dir=output_dir, framerate=framerate
        )
        try:
            video_file = recorder.start_recording(
                video_name=video_name, path_id=path_index
            )
        except AttributeError as exc:
            print(
                f"   [PYHPP-GAP] Video capture unavailable: {exc}. "
                "Playing path without recording."
            )
            self.play_path(path_index)
            return ""
        try:
            self.play_path(path_index)
        finally:
            recorder.stop_recording()
        return video_file

    def play_and_record_path_vector(
        self,
        path_vector: Any,
        video_name: Optional[str] = None,
        output_dir: str = "/home/dvtnguyen/devel/demos",
        framerate: int = 25,
        dt: float = 0.01,
        speed: float = 1.0,
    ) -> Tuple[int, str]:
        """Store a PathVector, play it, and record to video.

        Returns:
            Tuple of (path_index, video_file_path).
        """
        # plan_transition_edge returns an int id when store=True; accept it directly
        if isinstance(path_vector, int):
            path_index = path_vector
        else:
            path_index = self.store_path(path_vector)
        video_file = self.play_and_record_path(
            path_index,
            video_name=video_name,
            output_dir=output_dir,
            framerate=framerate,
            dt=dt,
            speed=speed,
        )
        return path_index, video_file

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
