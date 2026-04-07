#!/usr/bin/env python3
"""
Multi-grasp sequential planning using phase-based graph regeneration.

Implements incremental planning for tasks requiring multiple sequential grasps,
where some grasps must remain held while achieving others. Avoids constraint
graph explosion by rebuilding minimal graphs for each phase.
"""

from __future__ import annotations

import time
import signal
import os
from typing import Dict, List, Optional, Tuple, Any, Sequence, Set

from agimus_spacelab.planning.grasp_state import GraspStateTracker
from agimus_spacelab.planning.graph import GraphBuilder
from agimus_spacelab.planning.config import ConfigGenerator


# =============================================================================
# Filename Utilities
# =============================================================================


def sanitize_filename(name: str) -> str:
    """Sanitize a string for use in filenames.
    
    Replaces characters that are problematic for file paths and shell commands:
    - / (path separator)
    - > (shell redirection)
    - | (shell pipe)
    - < (shell redirection)
    - : (Windows drive separator, problematic in some contexts)
    - " (quotes)
    - * and ? (wildcards)
    - spaces
    
    Args:
        name: String to sanitize
        
    Returns:
        Sanitized string safe for use in filenames
    """
    replacements = {
        '/': '_',
        '>': '_gt_',
        '<': '_lt_',
        '|': '_or_',
        ':': '_',
        '"': '_',
        '*': '_',
        '?': '_',
        ' ': '_',
    }
    result = name
    for char, replacement in replacements.items():
        result = result.replace(char, replacement)
    return result


# =============================================================================
# Graceful Stop Signal Handler
# =============================================================================

_stop_requested = False


def _request_stop_signal_handler(signum, frame):
    """Signal handler for graceful stop via Ctrl+C.
    
    First Ctrl+C: Sets stop flag (waits for current edge to complete).
    Second Ctrl+C: Forces immediate exit.
    """
    global _stop_requested
    if not _stop_requested:
        _stop_requested = True
        print("\n🛑 Stop requested - will halt after current edge completes...")
        print("   (Press Ctrl+C again to force quit)")
    else:
        print("\n⚠️  Force quit!")
        os._exit(1)


def enable_graceful_stop():
    """Enable graceful stop via Ctrl+C signal handler."""
    signal.signal(signal.SIGINT, _request_stop_signal_handler)


def disable_graceful_stop():
    """Restore default Ctrl+C behavior."""
    signal.signal(signal.SIGINT, signal.SIG_DFL)


def clear_stop_request():
    """Clear the stop flag (useful for resuming after stop)."""
    global _stop_requested
    _stop_requested = False


def is_stop_requested() -> bool:
    """Check if stop has been requested."""
    return _stop_requested


class GraspSequencePlanner:
    """Orchestrate multi-phase grasp planning.

    For tasks requiring multiple sequential grasps (e.g., grasp A,
    then B, then C), this planner:
    1. Builds a minimal constraint graph for each phase (only relevant grasps)
    2. Computes the correct edge name from current/desired grasp state
    3. Generates waypoint configurations via the edge
    4. Plans the transition using TransitionPlanner
    5. Concatenates paths across all phases

    This approach scales linearly O(N) instead of combinatorially O(N!) as
    the number of grasps grows.
    """

    def __init__(
        self,
        graph_builder: GraphBuilder,
        config_gen: ConfigGenerator,
        planner: Any,
        task_config: Any,
        backend: str = "corba",
        pyhpp_constraints: Optional[Dict[str, Any]] = None,
        graph_constraints: Optional[List[str]] = None,
        auto_save_dir: Optional[str] = None,
    ):
        """Initialize grasp sequence planner.

        Args:
            graph_builder: GraphBuilder instance for creating phase graphs
            config_gen: ConfigGenerator for waypoint generation
            planner: Backend planner (ManipulationPlanner instance)
            task_config: Task configuration with GRIPPERS, OBJECTS, etc.
            backend: "corba" or "pyhpp"
            pyhpp_constraints: PyHPP constraint objects (for pyhpp backend)
            graph_constraints: Optional list of global constraints
            auto_save_dir: If set, automatically save paths to this directory
                          after each successful phase. Files are named
                          phase_NN_edge_MM.path (binary format).
        """
        self.graph_builder = graph_builder
        self.config_gen = config_gen
        self.planner = planner
        self.task_config = task_config
        self.backend = backend.lower()
        self.pyhpp_constraints = pyhpp_constraints
        self.graph_constraints = graph_constraints
        self._MAX_COLLISION_RETRIES = 10

        # Auto-save configuration
        self.auto_save_dir = auto_save_dir
        self.saved_path_files: List[str] = []  # Track saved file paths

        # Extract gripper and handle lists from config
        grippers = getattr(task_config, "GRIPPERS", [])

        # Extract handles from HANDLES_PER_OBJECT
        handles = []
        handles_per_obj = getattr(task_config, "HANDLES_PER_OBJECT", [])
        for obj_handles in handles_per_obj:
            handles.extend(obj_handles)

        if not grippers:
            raise ValueError("Task config must define GRIPPERS")
        if not handles:
            raise ValueError("Task config must define HANDLES_PER_OBJECT")

        # Initialize grasp state tracker
        self.grasp_tracker = GraspStateTracker(
            grippers=grippers,
            handles=handles,
            initial_grasps=None,  # Start with all free
        )

        # Store phase results for debugging/replay
        self.phase_results = []

        # Track last failure for resume capability
        self.last_failure_info = None
        self.original_sequence = None  # Store full sequence for resume

        # Edge-level timing and resume attempt statistics
        self.edge_stats = {}  # (phase_idx, edge_idx) -> timing info
        self.total_planning_time = 0.0
        self.resume_attempt_count = 0

        # Gripper-to-arm mapping for locked joint filtering
        # Maps gripper name pattern -> arm keyword
        # Multiple grippers can belong to the same arm
        self.GRIPPER_TO_ARM_MAP = {
            "g_ur10": "ur10",
            "g_FG": "ur10",      # UR10 with Frame Gripper
            "g_vispa_": "vispa_",
            "g_SD": "vispa_",    # Vispa with Screw Driver
            "g_vispa2": "vispa2",
        }
        self.ALL_ARM_KEYWORDS = ["ur10", "vispa_", "vispa2"]

        # Callback for interactive arm selection (set by UI layer)
        self.interactive_arm_selector_callback = None

    def _auto_save_phase_paths(
        self,
        phase_idx: int,
        phase_paths: List[Any],
        edge_names: List[str],
        verbose: bool = True,
        phase_geometric_paths: Optional[List[Any]] = None,
    ) -> List[str]:
        """Auto-save phase paths to files if auto_save_dir is configured.
        
        Args:
            phase_idx: 0-based phase index
            phase_paths: List of path objects from this phase (may have time param)
            edge_names: List of edge names corresponding to paths
            verbose: Print status messages
            phase_geometric_paths: Optional list of geometric paths (no time param)
                                   for serialization. If None, uses phase_paths.
            
        Returns:
            List of saved file paths
        """
        if not self.auto_save_dir:
            return []

        # Use geometric paths for saving if provided, otherwise fall back to regular paths
        paths_to_save = phase_geometric_paths if phase_geometric_paths is not None else phase_paths

        import os

        # Create directory if it doesn't exist
        os.makedirs(self.auto_save_dir, exist_ok=True)

        saved_files = []

        for edge_idx, path in enumerate(paths_to_save):
            # Skip None paths (from skipped phases)
            if path is None:
                continue

            # Generate base filename: phase_01_edge_01_edgename
            edge_name_safe = edge_names[edge_idx].replace("/", "_").replace(" ", "_")
            base_filename = f"phase_{phase_idx + 1:02d}_edge_{edge_idx + 1:02d}_{edge_name_safe}"

            # Try to save as portable JSON waypoints (always works)
            if hasattr(self.planner, "save_path_as_waypoints"):
                json_filepath = os.path.join(self.auto_save_dir, base_filename + ".json")
                try:
                    # Pass edge name for graph metadata context
                    self.planner.save_path_as_waypoints(
                        path,
                        json_filepath,
                        num_samples=100,
                        edge_name=edge_names[edge_idx],
                    )
                    saved_files.append(json_filepath)
                    if verbose:
                        print(f"       ✓ Auto-saved (portable): {base_filename}.json")
                except Exception as e:
                    if verbose:
                        print(f"       ⚠ Failed to save JSON waypoints: {e}")

            # Also try to save native .path format (may fail for graph paths, but worth trying)
            if hasattr(self.planner, "save_path_vector"):
                path_filepath = os.path.join(self.auto_save_dir, base_filename + ".path")
                try:
                    self.planner.save_path_vector(path, path_filepath)
                    saved_files.append(path_filepath)
                    if verbose:
                        print(f"       ✓ Auto-saved (native): {base_filename}.path")
                except Exception as e:
                    # Native format may fail, but JSON should have succeeded
                    if verbose:
                        error_msg = str(e)
                        if "time parameterization" in error_msg.lower():
                            print(f"       ⚠ Native format skipped: Time-parameterized paths not serializable")
                        elif "graph" in error_msg.lower() or "edge" in error_msg.lower():
                            print(f"       ⚠ Native format skipped: Graph-constrained paths (JSON format works)")
                        else:
                            print(f"       ⚠ Native format failed: {e}")
                else:
                    if verbose:
                        print(f"       ⚠ Failed to save {base_filename}: {e}")

        self.saved_path_files.extend(saved_files)
        return saved_files

    def get_saved_path_files(self) -> List[str]:
        """Get list of all auto-saved path files.
        
        Returns:
            List of file paths that were saved during planning
            
        Note:
            Paths saved by TransitionPlanner contain constraint graph edge
            references and can only be loaded in the same session or with
            the same graph structure. For cross-session replay, use
            replay_sequence() within the same planning session.
        """
        return list(self.saved_path_files)

    def load_saved_paths(self, verbose: bool = True) -> List[int]:
        """Load all previously auto-saved paths.
        
        This is useful for replaying a sequence after restarting.
        
        Args:
            verbose: Print status messages
            
        Returns:
            List of path indices in ProblemSolver
            
        Warning:
            Paths saved from TransitionPlanner contain graph edge constraints.
            Loading will fail unless the same constraint graph is recreated.
            For reliable replay, use replay_sequence() in the same session.
        """
        if not self.auto_save_dir:
            if verbose:
                print("No auto_save_dir configured")
            return []

        if hasattr(self.planner, "load_paths_from_directory"):
            return self.planner.load_paths_from_directory(
                self.auto_save_dir,
                pattern="phase_*.path",
                sort=True,
            )
        else:
            if verbose:
                print("Backend does not support path loading")
            return []

    def compute_phase_locked_joints(
        self,
        active_gripper: str,
        mode: str = "auto",
        manual_arms: Optional[List[str]] = None,
        verbose: bool = True,
    ) -> List[str]:
        """Compute which arms should be frozen for current phase.

        Args:
            active_gripper: Gripper performing grasp in this phase
            mode: "auto" (freeze all except active),
                  "manual" (use manual_arms), "none" (no locked joints)
            manual_arms: List of arm keywords to freeze (for manual mode)

        Returns:
            List of arm keywords to freeze
                (for create_locked_joint_constraints)
        """
        if mode == "none":
            return []

        if mode == "manual":
            if manual_arms is None:
                return []
            return manual_arms

        # Auto mode: freeze all arms except the active gripper's arm
        frozen_arms = []
        active_arm = None

        # Find which arm the active gripper belongs to
        gripper_lower = active_gripper.lower()
        for gripper_pattern, arm_keyword in self.GRIPPER_TO_ARM_MAP.items():
            if gripper_pattern.lower() in gripper_lower:
                active_arm = arm_keyword
                break

        if verbose:
            print(f"Active gripper '{active_gripper}' uses arm '{active_arm}'")

        # Freeze all other arms
        for arm_keyword in self.ALL_ARM_KEYWORDS:
            if arm_keyword != active_arm:
                frozen_arms.append(arm_keyword)

        return frozen_arms

    def plan_sequence(
        self,
        grasp_sequence: Sequence[Tuple[str, str]],
        q_init: Sequence[float],
        validate: bool = True,
        reset_roadmap: bool = True,
        time_parameterize: bool = True,
        max_iterations_per_edge: int = 10000,
        timeout_per_edge: float = 60.0,
        frozen_arms_mode: str = "auto",
        per_phase_frozen_arms: Optional[Dict[int, List[str]]] = None,
        skip_phases: Optional[Set[int]] = None,
        verbose: bool = True,
    ) -> Dict[str, Any]:
        """Plan a sequence of grasp transitions.

        Args:
            grasp_sequence: List of (gripper, handle) pairs to grasp in order
            q_init: Initial configuration (all grippers free)
            validate: Validate paths after planning
            reset_roadmap: Reset roadmap between phases
            time_parameterize: Apply time parameterization to paths
            max_iterations_per_edge: Max iterations per edge planning
            timeout_per_edge: Timeout in seconds for each edge planning
            frozen_arms_mode: "auto" (freeze all except active arm),
                "manual" (use per_phase_frozen_arms), "none" (no locking),
                "interactive" (use callback for selection per phase),
                or "global" (use self.graph_constraints from task.setup())
            per_phase_frozen_arms: Dict mapping phase_idx -> list of
                arm keywords. Used when frozen_arms_mode="manual".
                Example: {0: ["vispa_", "vispa2"]}
            skip_phases: Optional set of 0-based phase indices to skip motion
                planning for. Skipped phases will still generate target configs
                and update grasp state, but will not call plan_transition_edge().
            verbose: Print progress messages

        Note:
            For interactive mode, set frozen_arms_mode="interactive" and
            assign self.interactive_arm_selector_callback before calling.
            Callback signature: callback(phase_idx, gripper, arm_keywords)
                -> List[str] of selected arm keywords

        Note:
            For interactive mode, set frozen_arms_mode="interactive" and
            set self.interactive_arm_selector_callback before calling.
            Callback signature: callback(phase_idx, gripper, arm_keywords)
                -> List[str] of selected arm keywords

        Returns:
            Dictionary with:
                - success: bool, whether all phases succeeded
                - path_id: path ID if backend supports storage
                - phase_results: list of per-phase results
                - final_config: final configuration after all grasps
                - grasp_tracker: final grasp state

        Raises:
            RuntimeError: If a phase fails to plan
        """
        # Configure TransitionPlanner timeout for waypoint edges
        if hasattr(self.planner, "configure_transition_planner"):
            self.planner.configure_transition_planner(
                time_out=timeout_per_edge,
                max_iterations=max_iterations_per_edge,
            )
            if verbose:
                print(
                    f"Configured TransitionPlanner: "
                    f"timeout={timeout_per_edge}s, "
                    f"max_iterations={max_iterations_per_edge}"
                )

        if verbose:
            print("\n" + "=" * 70)
            print("Grasp Sequence Planning")
            print("=" * 70)
            print(f"Sequence: {grasp_sequence}")
            print(
                f"Initial state: {self.grasp_tracker.get_current_state_name()}"
            )
            print("\nℹ️  Press Ctrl+C to stop gracefully (saves progress)")

        # Enable graceful stop and clear any previous stop request
        clear_stop_request()
        enable_graceful_stop()

        self.phase_results = []
        self.original_sequence = list(grasp_sequence)  # Store for resume
        q_current = list(q_init)
        # Original scene configuration — used to restore free-object positions
        # before each phase graph build so LockedJoint foliation locks them at
        # their true scene positions, not at random IK-sampled positions.
        _q_scene_init = list(q_init)
        self._q_scene_init = _q_scene_init  # store for resume_planning

        for phase_idx, (gripper, handle) in enumerate(grasp_sequence):
            if verbose:
                print("\n" + "-" * 70)
                print(f"\n--- Phase {phase_idx + 1}/{len(grasp_sequence)} ---")
                print(f"  Grasp '{handle}' with '{gripper}'")
                current_state = self.grasp_tracker.get_current_state_name()
                print(f"  Current state: {current_state}")

            # Build minimal phase graph
            held_grasps = {
                g: h
                for g, h in self.grasp_tracker.current_grasps.items()
                if h is not None
            }
            print(f"  Held grasps: {held_grasps}")

            # Compute phase-specific locked joint constraints
            phase_graph_constraints = None

            if frozen_arms_mode == "global":
                # Use global constraints from task.setup()
                phase_graph_constraints = self.graph_constraints
                if verbose and phase_graph_constraints:
                    print("  Using global locked joint constraints")
            elif frozen_arms_mode != "none":
                # Determine which arms to freeze for this phase
                if frozen_arms_mode == "interactive":
                    # Interactive mode: use callback if available
                    if self.interactive_arm_selector_callback:
                        try:
                            frozen_arms = (
                                self.interactive_arm_selector_callback(
                                    phase_idx, gripper, self.ALL_ARM_KEYWORDS
                                )
                            )
                        except Exception as e:
                            # Fallback to auto if callback fails
                            if verbose:
                                print(
                                    f"  \u26a0 Interactive selection failed: "
                                    f"{e}, using auto mode"
                                )
                            frozen_arms = self.compute_phase_locked_joints(
                                gripper, "auto"
                            )
                    else:
                        # No callback set, fall back to auto
                        if verbose:
                            print(
                                "  \u26a0 Interactive mode requested but no "
                                "callback set, using auto mode"
                            )
                        frozen_arms = self.compute_phase_locked_joints(
                            gripper, "auto"
                        )
                elif frozen_arms_mode == "manual" and per_phase_frozen_arms:
                    # Manual mode with explicit specification
                    frozen_arms = per_phase_frozen_arms.get(phase_idx, [])
                else:
                    # Auto mode: freeze all except active gripper's arm
                    frozen_arms = self.compute_phase_locked_joints(
                        gripper, "auto"
                    )

                # Create locked joint constraints for this phase
                if frozen_arms:
                    if verbose:
                        print(f"  Freezing arms: {frozen_arms}")

                    from agimus_spacelab.planning.constraints import (
                        ConstraintBuilder,
                    )

                    constraint_names, joint_names = (
                        ConstraintBuilder.create_locked_joint_constraints(
                            self.graph_builder.ps,
                            self.graph_builder.robot,
                            q_current,  # Use current config for joint values
                            frozen_arms,
                            backend=self.graph_builder.backend,
                        )
                    )

                    if constraint_names:
                        phase_graph_constraints = constraint_names
                        if verbose:
                            joint_list = ", ".join(sorted(joint_names))
                            print(f"  \u2713 Created {len(joint_names)} locked joint constraints: {joint_list}")
                            print(f"     Constraint names: {constraint_names}")

            try:
                self.graph_builder.build_phase_graph(
                    config=self.task_config,
                    held_grasps=held_grasps,
                    next_grasp=(gripper, handle),
                    pyhpp_constraints=self.pyhpp_constraints,
                    graph_constraints=phase_graph_constraints,
                    q_init=q_current,
                    q_init_original=_q_scene_init,
                )

                # Update planner backend's graph reference after rebuild
                # The graph_builder has the new graph, but the planner backend
                # still has the old reference and needs to be updated
                new_graph = self.graph_builder.get_graph()
                if hasattr(self.planner, "graph"):
                    self.planner.graph = new_graph
                    if verbose:
                        print("  \u2713 Updated planner graph reference")
                        print(f"     Graph object: {type(new_graph).__name__}")
                        if hasattr(new_graph, "edges"):
                            print(
                                f"     Graph has {len(new_graph.edges)} edges"
                            )

                # Update ConfigGenerator's graph reference (or initialize it)
                # ConfigGenerator needs current graph for edge-based config generation
                if self.config_gen is None:
                    # First phase: initialize ConfigGenerator with phase graph
                    from agimus_spacelab.planning import ConfigGenerator

                    self.config_gen = ConfigGenerator(
                        self.graph_builder.robot,
                        new_graph,
                        self.planner,
                        self.graph_builder.ps,
                        backend=self.backend,
                    )
                    if verbose:
                        print(
                            "  \u2713 Initialized ConfigGenerator with phase graph"
                        )
                elif hasattr(self.config_gen, "update_graph"):
                    # Subsequent phases: update graph reference
                    self.config_gen.update_graph(new_graph)
                    if verbose:
                        print(
                            "  \u2713 Updated ConfigGenerator graph reference"
                        )

            except Exception as e:
                raise RuntimeError(
                    f"Phase {phase_idx + 1}: Failed to build graph: {e}"
                ) from e

            # Compute edge sequence from current state
            # Use waypoints for better constraint satisfaction
            try:
                edge_sequence = self.grasp_tracker.get_grasp_edge_sequence(
                    gripper, handle
                )
            except Exception as e:
                raise RuntimeError(
                    f"Phase {phase_idx + 1}: "
                    f"Failed to compute edge sequence: {e}"
                ) from e

            if verbose:
                print(f"  Edge sequence: {edge_sequence}")

            # Project current config onto the phase graph's source state
            # The phase graph has different constraints, so q_current
            # might not satisfy them. We need to project onto the
            # edge's source state.
            source_state = self.grasp_tracker.get_current_state_name()
            try:
                if verbose:
                    print(
                        f"  Projecting q_current onto state: "
                        f"{source_state}"
                    )
                    print(f"     q_current (first 5): {q_current[:5]}")

                success, q_projected, error = (
                    self.graph_builder.apply_state_constraints(
                        state_name=source_state,
                        q=q_current,
                        max_iterations=10000,
                        error_threshold=1e-4,
                    )
                )

                if not success:
                    raise RuntimeError(
                        f"Failed to project q_current onto state "
                        f"'{source_state}' (error={error:.6f})"
                    )

                q_current = list(q_projected)

                if verbose:
                    print(f"  ✓ Projected q_current (error={error:.6e})")
                    print(f"     q_projected (first 5): {q_current[:5]}")

            except Exception as e:
                # Store partial phase result for projection failure
                partial_phase_result = {
                    "phase": phase_idx + 1,
                    "gripper": gripper,
                    "handle": handle,
                    "edges": edge_sequence,
                    "paths": [],  # No paths yet in this phase
                    "complete": False,
                    "failed_edge_idx": 0,
                    "failed_edge_name": (
                        edge_sequence[0] if edge_sequence else None
                    ),
                    "last_q_start": q_current,
                    "failed_q_target": None,
                    "error_message": f"State projection failed: {e}",
                }
                self.phase_results.append(partial_phase_result)

                # Store failure info
                self.last_failure_info = {
                    "phase_idx": phase_idx,
                    "edge_idx": 0,
                    "edge_name": (
                        edge_sequence[0] if edge_sequence else "unknown"
                    ),
                    "q_current": q_current,
                    "error": f"State projection failed: {e}",
                    "completed_phases": len(
                        [
                            p
                            for p in self.phase_results
                            if p.get("complete", False)
                        ]
                    ),
                    "completed_edges_in_phase": 0,
                }

                if verbose:
                    print(
                        f"\n  ⚠ Stored partial phase result: projection failed at phase start"
                    )

                raise RuntimeError(
                    f"Phase {phase_idx + 1}: State projection failed: {e}"
                ) from e

            # Plan through waypoint edge sequence
            # Each edge in sequence has its own path constraints
            # that are easier to satisfy
            phase_paths = []
            phase_geometric_paths = []  # Geometric paths (no time param) for saving
            edge_stats_list = []  # Per-edge timing stats for this phase
            q_start = q_current

            for edge_idx, edge_name in enumerate(edge_sequence):
                # Check for stop request
                if is_stop_requested():
                    if verbose:
                        print("\n  ⚠️  Stop requested - saving progress...")
                    # Save partial result
                    partial_phase_result = {
                        "phase": phase_idx + 1,
                        "gripper": gripper,
                        "handle": handle,
                        "edges": edge_sequence,
                        "paths": phase_paths,
                        "edge_stats": edge_stats_list,
                        "complete": False,
                        "failed_edge_idx": edge_idx,
                        "failed_edge_name": edge_name,
                        "last_q_start": q_start,
                        "stopped": True,
                        "error_message": "Stopped by user request",
                    }
                    self.phase_results.append(partial_phase_result)

                    self.last_failure_info = {
                        "phase_idx": phase_idx,
                        "edge_idx": edge_idx,
                        "edge_name": edge_name,
                        "q_current": q_current,
                        "error": "Stopped by user request (Ctrl+C)",
                        "completed_phases": len(
                            [p for p in self.phase_results if p.get("complete", False)]
                        ),
                        "completed_edges_in_phase": len(phase_paths),
                        "stopped": True,
                    }

                    if verbose:
                        print(f"\n  ⚠️  Stored partial phase result: {len(phase_paths)} edges completed")
                        print(f"     You can resume from Phase {phase_idx + 1}, Edge {edge_idx + 1}")

                    # Disable signal handler before raising
                    disable_graceful_stop()
                    raise KeyboardInterrupt("Planning stopped by user request")

                edge_start_time = time.time()
                edge_stat = {
                    "edge_idx": edge_idx,
                    "edge_name": edge_name,
                    "attempt": 1,
                    "gen_time": 0.0,
                    "plan_time": 0.0,
                    "total_time": 0.0,
                    "success": False,
                }

                if verbose:
                    print(
                        f"  Planning waypoint edge "
                        f"{edge_idx + 1}/{len(edge_sequence)}: "
                        f"{edge_name}"
                    )

                # Generate target configuration via this edge
                gen_start = time.time()
                q_target = None
                try:
                    ok, q_target = self.config_gen.generate_via_edge(
                        edge_name=edge_name,
                        q_from=q_start,
                        config_label=f"q_phase{phase_idx}_edge{edge_idx}",
                    )
                    # Print and check the generated configuration
                    import numpy as np
                    edge_stat["gen_time"] = time.time() - gen_start

                    # Check for NaN/inf or None
                    if not ok or q_target is None or not np.all(np.isfinite(q_target)):
                        raise RuntimeError(
                            f"Failed to generate valid target via edge '{edge_name}': q_target={q_target}"
                        )
                    else:
                        if verbose:
                            print(f"     ✓ Generated target config ({edge_stat['gen_time']:.2f}s)")
                            print(f"     q_target: {q_target}")
                            # if q_target is not None:
                            #     arr = np.array(q_target)
                            #     print(f"     q_target finite: {np.all(np.isfinite(arr))}")
                            #     print(f"     q_target min/max: {arr.min()} / {arr.max()}")

                    # Visualize the configuration before planning
                    try:
                        print(f"     Visualizing q_target for edge '{edge_name}' before planning...")
                        self.planner.visualize(q_target)
                        print("     ✓ q_target sent to viewer")
                    except Exception as e:
                        print(f"     ⚠ Could not visualize q_target: {e}")

                except Exception as e:
                    edge_stat["gen_time"] = time.time() - gen_start
                    edge_stat["total_time"] = time.time() - edge_start_time
                    edge_stats_list.append(edge_stat)

                    # Store partial phase result
                    partial_phase_result = {
                        "phase": phase_idx + 1,
                        "gripper": gripper,
                        "handle": handle,
                        "edges": edge_sequence,
                        "paths": phase_paths,
                        "edge_stats": edge_stats_list,
                        "complete": False,
                        "failed_edge_idx": edge_idx,
                        "failed_edge_name": edge_name,
                        "last_q_start": q_start,
                        "failed_q_target": None,
                        "error_message": f"Target generation failed: {e}",
                    }
                    self.phase_results.append(partial_phase_result)

                    # Store failure info for resume
                    self.last_failure_info = {
                        "phase_idx": phase_idx,
                        "edge_idx": edge_idx,
                        "edge_name": edge_name,
                        "q_current": q_current,
                        "error": f"Target generation failed: {e}",
                        "completed_phases": len(
                            [
                                p
                                for p in self.phase_results
                                if p.get("complete", False)
                            ]
                        ),
                        "completed_edges_in_phase": len(phase_paths),
                    }

                    if verbose:
                        print(
                            f"\n  ⚠ Stored partial phase result: {len(phase_paths)} edges completed"
                        )

                    raise RuntimeError(
                        f"Phase {phase_idx + 1}, edge {edge_idx + 1}: "
                        f"Target generation failed: {e}"
                    ) from e

                # Check if this phase should skip motion planning
                if skip_phases and phase_idx in skip_phases:
                    # Skip motion planning, use q_target directly
                    edge_stat["total_time"] = time.time() - edge_start_time
                    edge_stat["skipped"] = True
                    edge_stat["success"] = True
                    edge_stats_list.append(edge_stat)
                    self.total_planning_time += edge_stat["total_time"]
                    self.edge_stats[(phase_idx, edge_idx)] = edge_stat

                    # Use q_target as next start config
                    q_start = q_target

                    # Append None placeholder for skipped path
                    phase_paths.append(None)
                    if self.auto_save_dir:
                        phase_geometric_paths.append(None)

                    if verbose:
                        print(
                            f"     ⏭ Skipped motion planning "
                            f"({edge_stat['total_time']:.2f}s total)"
                        )

                    continue

                # Plan transition using TransitionPlanner.
                # Retry with a fresh q_target if the planner detects a
                # collision in the generated config (ps.isConfigValid only
                # checks joint bounds, not self-collision; the planner may
                # still reject a kinematically valid IK solution as colliding).
                import numpy as _np

                plan_start = time.time()
                last_plan_exc = None
                path = None
                geometric_path = None

                for _plan_attempt in range(self._MAX_COLLISION_RETRIES):
                    try:
                        if verbose:
                            if _plan_attempt == 0:
                                print("     Planning: q_start -> q_target")
                            else:
                                _prev_reason = (
                                    str(last_plan_exc).split('\n')[0]
                                    if last_plan_exc else ""
                                )
                                print(
                                    f"     Planning (attempt {_plan_attempt + 1}) "
                                    f"[prev failed: {_prev_reason}]"
                                )

                        path, geometric_path = (
                            self.planner.plan_transition_edge(
                                edge=edge_name,
                                q1=q_start,
                                q2=q_target,
                            )
                        )

                        if path is None:
                            raise RuntimeError(
                                f"Planning failed for edge '{edge_name}'"
                            )

                        last_plan_exc = None
                        break  # success

                    except Exception as _plan_exc:
                        last_plan_exc = _plan_exc
                        if (
                            "Collision" in str(_plan_exc)
                            and _plan_attempt < self._MAX_COLLISION_RETRIES - 1
                        ):
                            if verbose:
                                print(
                                    f"     ⚠ Collision in q_target "
                                    f"(attempt {_plan_attempt + 1}), "
                                    f"regenerating target config..."
                                )
                            _ok2, _q_new = self.config_gen.generate_via_edge(
                                edge_name=edge_name,
                                q_from=q_start,
                                config_label=(
                                    f"q_phase{phase_idx}_edge{edge_idx}"
                                ),
                            )
                            if (
                                _ok2
                                and _q_new is not None
                                and _np.all(_np.isfinite(_np.array(_q_new)))
                            ):
                                q_target = _q_new
                                if verbose:
                                    print("     Regenerated target config")

                if last_plan_exc is not None:
                    e = last_plan_exc
                    edge_stat["plan_time"] = time.time() - plan_start
                    edge_stat["total_time"] = time.time() - edge_start_time
                    edge_stats_list.append(edge_stat)
                    self.total_planning_time += edge_stat["total_time"]

                    # Store partial phase result before raising
                    partial_phase_result = {
                        "phase": phase_idx + 1,
                        "gripper": gripper,
                        "handle": handle,
                        "edges": edge_sequence,
                        "paths": phase_paths,  # Successfully completed edges
                        "edge_stats": edge_stats_list,
                        "complete": False,
                        "failed_edge_idx": edge_idx,
                        "failed_edge_name": edge_name,
                        "last_q_start": q_start,
                        "failed_q_target": q_target,
                        "error_message": str(e),
                    }
                    self.phase_results.append(partial_phase_result)

                    # Store failure info for resume
                    self.last_failure_info = {
                        "phase_idx": phase_idx,
                        "edge_idx": edge_idx,
                        "edge_name": edge_name,
                        "q_current": q_current,
                        "error": str(e),
                        "completed_phases": len(
                            [
                                p
                                for p in self.phase_results
                                if p.get("complete", False)
                            ]
                        ),
                        "completed_edges_in_phase": len(phase_paths),
                    }

                    if verbose:
                        print(
                            f"\n  ⚠ Stored partial phase result: "
                            f"{len(phase_paths)} edges completed"
                        )

                    raise RuntimeError(
                        f"Phase {phase_idx + 1}, edge {edge_idx + 1}: "
                        f"Planning failed for '{edge_name}': {e}"
                    ) from e

                # Planning succeeded — record stats and advance
                # Store geometric path if auto-save is enabled
                if self.auto_save_dir:
                    phase_geometric_paths.append(geometric_path)

                edge_stat["plan_time"] = time.time() - plan_start
                edge_stat["total_time"] = time.time() - edge_start_time
                edge_stat["success"] = True
                edge_stats_list.append(edge_stat)
                self.total_planning_time += edge_stat["total_time"]
                self.edge_stats[(phase_idx, edge_idx)] = edge_stat

                phase_paths.append(path)

                # Update start config for next edge
                if hasattr(path, "getInitialConfig") and hasattr(
                    path, "getEndConfig"
                ):
                    q_start = list(path.getEndConfig())
                else:
                    q_start = q_target

                if verbose:
                    print(
                        f"     ✓ Path found ({edge_stat['plan_time']:.2f}s plan, "
                        f"{edge_stat['total_time']:.2f}s total)"
                    )

            # Update current config to end of sequence
            q_current = q_start

            # Compute phase timing totals
            phase_total_time = sum(s["total_time"] for s in edge_stats_list)
            phase_plan_time = sum(s["plan_time"] for s in edge_stats_list)
            phase_gen_time = sum(s["gen_time"] for s in edge_stats_list)

            if verbose:
                print(f"  ✓ Completed {len(edge_sequence)}-edge sequence")
                print(
                    f"     Phase timing: {phase_total_time:.2f}s total "
                    f"(gen: {phase_gen_time:.2f}s, plan: {phase_plan_time:.2f}s)"
                )

            # Update grasp state after successful planning
            self.grasp_tracker.update_grasp(gripper, handle)

            # Auto-save paths after successful phase
            saved_files = self._auto_save_phase_paths(
                phase_idx=phase_idx,
                phase_paths=phase_paths,
                edge_names=edge_sequence,
                verbose=verbose,
                phase_geometric_paths=phase_geometric_paths if self.auto_save_dir else None,
            )

            # Store phase result with all edge paths and timing stats
            phase_result = {
                "phase": phase_idx + 1,
                "gripper": gripper,
                "handle": handle,
                "edges": edge_sequence,
                "paths": phase_paths,
                "edge_stats": edge_stats_list,
                "phase_time": phase_total_time,
                "phase_plan_time": phase_plan_time,
                "phase_gen_time": phase_gen_time,
                "complete": True,
                "skipped": skip_phases and phase_idx in skip_phases,
                "final_config": q_current,
                "state_after": self.grasp_tracker.get_current_state_name(),
                "saved_files": saved_files,  # Track saved path files
            }
            self.phase_results.append(phase_result)

        if verbose:
            print("\n" + "=" * 70)
            print("Sequence Planning Complete")
            print("=" * 70)
            print(
                f"Final state: {self.grasp_tracker.get_current_state_name()}"
            )
            print(f"Completed {len(self.phase_results)} phases")
            print(f"Total planning time: {self.total_planning_time:.2f}s")

        # Disable signal handler before returning
        disable_graceful_stop()

        # Return combined result
        return {
            "success": True,
            "paths": (
                self.phase_results[-1]["paths"] if self.phase_results else []
            ),
            "phase_results": self.phase_results,
            "final_config": q_current,
            "grasp_tracker": self.grasp_tracker,
        }

    def get_resumable_state(self) -> Optional[Dict[str, Any]]:
        """Check if sequence can be resumed and return failure context.
        
        Returns:
            Dict with failure info if resumable, None otherwise.
            Dict contains:
                - phase_idx: Index of failed phase (0-based)
                - edge_idx: Index of failed edge within phase (0-based)
                - edge_name: Name of edge that failed
                - completed_phases: Number of fully completed phases
                - completed_edges_in_phase: Number of edges completed in failed phase
                - q_current: Configuration at failure point
                - error: Error message
        """
        if not self.last_failure_info or not self.phase_results:
            return None

        # Find incomplete phase
        incomplete_phase = None
        completed_phases = 0
        for phase in self.phase_results:
            if not phase.get("complete", True):
                incomplete_phase = phase
                break
            completed_phases += 1

        if incomplete_phase is None:
            return None

        return {
            "phase_idx": incomplete_phase["phase"] - 1,  # 0-based
            "edge_idx": incomplete_phase["failed_edge_idx"],
            "edge_name": incomplete_phase["failed_edge_name"],
            "completed_phases": completed_phases,
            "completed_edges_in_phase": len(incomplete_phase["paths"]),
            "total_edges_in_phase": len(incomplete_phase["edges"]),
            "q_current": incomplete_phase.get("last_q_start"),
            "error": incomplete_phase.get("error_message", "unknown"),
        }

    def resume_sequence(
        self,
        retry_from_edge: int = 0,
        max_iterations_per_edge: Optional[int] = None,
        timeout_per_edge: Optional[float] = None,
        frozen_arms_mode: Optional[str] = None,
        per_phase_frozen_arms: Optional[Dict[int, List[str]]] = None,
        skip_phases: Optional[Set[int]] = None,
        validate: bool = True,
        reset_roadmap: bool = True,
        time_parameterize: bool = True,
        verbose: bool = True,
    ) -> Dict[str, Any]:
        """Resume planning from last failure point.

        Args:
            retry_from_edge: Edge index to retry from (0 = start of failed phase,
                           -1 = retry only the failed edge)
            max_iterations_per_edge: Override max iterations (None = use previous)
            timeout_per_edge: Override timeout (None = use previous)
            frozen_arms_mode: Override frozen arms mode (None = use "global")
            per_phase_frozen_arms: Per-phase manual arm specification
            skip_phases: Optional set of 0-based phase indices to skip motion
                planning for (same as plan_sequence)
            validate: Validate paths after planning
            reset_roadmap: Reset roadmap between phases
            time_parameterize: Apply time parameterization
            verbose: Print progress messages

        Returns:
            Same as plan_sequence()

        Raises:
            RuntimeError: If no resumable state exists or if planning fails again
        """
        resume_state = self.get_resumable_state()
        if resume_state is None:
            raise RuntimeError(
                "No resumable state found. "
                "Use plan_sequence() to start a new sequence."
            )

        # Increment resume attempt counter
        self.resume_attempt_count += 1

        if verbose:
            print("\n" + "=" * 70)
            print(
                f"Resuming Grasp Sequence Planning (attempt #{self.resume_attempt_count})"
            )
            print("=" * 70)
            print(f"Resuming from Phase {resume_state['phase_idx'] + 1}, "
                  f"Edge {resume_state['edge_idx'] + 1}")
            print(f"Previous error: {resume_state['error']}")
            print(f"Completed phases: {resume_state['completed_phases']}")
            print(f"Completed edges in current phase: "
                  f"{resume_state['completed_edges_in_phase']}")
            print("\nℹ️  Press Ctrl+C to stop gracefully (saves progress)")

        # Enable graceful stop and clear any previous stop request
        clear_stop_request()
        enable_graceful_stop()

        # Configure TransitionPlanner if new params provided
        if max_iterations_per_edge or timeout_per_edge:
            if hasattr(self.planner, "configure_transition_planner"):
                kwargs = {}
                if timeout_per_edge:
                    kwargs["time_out"] = timeout_per_edge
                if max_iterations_per_edge:
                    kwargs["max_iterations"] = max_iterations_per_edge
                self.planner.configure_transition_planner(**kwargs)
                if verbose:
                    print(f"Updated TransitionPlanner config: {kwargs}")

        # Restore grasp tracker state from completed phases
        # Reset to free state first
        self.grasp_tracker = GraspStateTracker(
            grippers=self.grasp_tracker.grippers,
            handles=self.grasp_tracker.handles,
            initial_grasps=None,
        )

        # Replay completed grasps to restore state
        for phase in self.phase_results:
            if phase.get("complete", False):
                self.grasp_tracker.update_grasp(
                    phase["gripper"], phase["handle"]
                )

        if verbose:
            print(f"Restored state: {self.grasp_tracker.get_current_state_name()}")

        # Remove incomplete phase from results (will be recreated)
        incomplete_phase_idx = resume_state["phase_idx"]
        self.phase_results = [
            p for p in self.phase_results if p.get("complete", True)
        ]

        # Get remaining sequence starting from failed phase
        remaining_sequence = self.original_sequence[incomplete_phase_idx:]

        if verbose:
            print(f"Remaining sequence: {remaining_sequence}")

        # Determine starting config
        if self.phase_results:
            # Continue from last completed phase
            q_current = self.phase_results[-1]["final_config"]
        else:
            # No completed phases, use the stored q_current from failure
            q_current = resume_state["q_current"]

        # Continue planning from failed phase
        # Use similar logic to plan_sequence but start from failed phase
        for phase_idx_offset, (gripper, handle) in enumerate(remaining_sequence):
            phase_idx = incomplete_phase_idx + phase_idx_offset

            if verbose:
                print("\n" + "-" * 70)
                print(f"\n--- Phase {phase_idx + 1}/{len(self.original_sequence)} ---")
                print(f"  Grasp '{handle}' with '{gripper}'")
                current_state = self.grasp_tracker.get_current_state_name()
                print(f"  Current state: {current_state}")

            # Build minimal phase graph
            held_grasps = {
                g: h
                for g, h in self.grasp_tracker.current_grasps.items()
                if h is not None
            }
            print(f"  Held grasps: {held_grasps}")

            # Compute phase-specific locked joint constraints (reuse original mode if not overridden)
            use_frozen_mode = (
                frozen_arms_mode if frozen_arms_mode is not None else "global"
            )
            phase_graph_constraints = None

            if use_frozen_mode == "global":
                # Use global constraints from task.setup()
                phase_graph_constraints = self.graph_constraints
                if verbose and phase_graph_constraints:
                    print("  Using global locked joint constraints")
            elif use_frozen_mode != "none":
                # Determine which arms to freeze for this phase
                if use_frozen_mode == "interactive":
                    # Interactive mode: use callback if available
                    if self.interactive_arm_selector_callback:
                        try:
                            frozen_arms = (
                                self.interactive_arm_selector_callback(
                                    phase_idx, gripper, self.ALL_ARM_KEYWORDS
                                )
                            )
                        except Exception as e:
                            if verbose:
                                print(
                                    f"  \u26a0 Interactive selection failed: "
                                    f"{e}, using auto mode"
                                )
                            frozen_arms = self.compute_phase_locked_joints(
                                gripper, "auto"
                            )
                    else:
                        if verbose:
                            print(
                                "  \u26a0 Interactive mode requested but no "
                                "callback set, using auto mode"
                            )
                        frozen_arms = self.compute_phase_locked_joints(
                            gripper, "auto"
                        )
                elif use_frozen_mode == "manual" and per_phase_frozen_arms:
                    frozen_arms = per_phase_frozen_arms.get(phase_idx, [])
                else:
                    # Auto mode
                    frozen_arms = self.compute_phase_locked_joints(
                        gripper, "auto"
                    )

                # Create locked joint constraints
                if frozen_arms:
                    if verbose:
                        print(f"  Freezing arms: {frozen_arms}")

                    from agimus_spacelab.planning.constraints import (
                        ConstraintBuilder,
                    )

                    constraint_names, joint_names = (
                        ConstraintBuilder.create_locked_joint_constraints(
                            self.graph_builder.ps,
                            self.graph_builder.robot,
                            q_current,
                            frozen_arms,
                            backend=self.graph_builder.backend,
                        )
                    )

                    if constraint_names:
                        phase_graph_constraints = constraint_names
                        if verbose:
                            joint_list = ", ".join(sorted(joint_names))
                            print(
                                f"  \u2713 Created {len(joint_names)} "
                                f"locked joint constraints: {joint_list}"
                            )

            try:
                self.graph_builder.build_phase_graph(
                    config=self.task_config,
                    held_grasps=held_grasps,
                    next_grasp=(gripper, handle),
                    pyhpp_constraints=self.pyhpp_constraints,
                    graph_constraints=phase_graph_constraints,
                    q_init=q_current,
                    q_init_original=getattr(self, "_q_scene_init", None),
                )

                new_graph = self.graph_builder.get_graph()
                if hasattr(self.planner, "graph"):
                    self.planner.graph = new_graph

                if self.config_gen and hasattr(self.config_gen, "update_graph"):
                    self.config_gen.update_graph(new_graph)
                elif self.config_gen is None:
                    from agimus_spacelab.planning import ConfigGenerator
                    self.config_gen = ConfigGenerator(
                        self.graph_builder.robot,
                        new_graph,
                        self.planner,
                        self.graph_builder.ps,
                        backend=self.backend,
                    )

            except Exception as e:
                raise RuntimeError(
                    f"Phase {phase_idx + 1}: Failed to build graph: {e}"
                ) from e

            # Get edge sequence
            try:
                edge_sequence = self.grasp_tracker.get_grasp_edge_sequence(
                    gripper, handle
                )
            except Exception as e:
                raise RuntimeError(
                    f"Phase {phase_idx + 1}: "
                    f"Failed to compute edge sequence: {e}"
                ) from e

            if verbose:
                print(f"  Edge sequence: {edge_sequence}")

            # Project config
            source_state = self.grasp_tracker.get_current_state_name()
            try:
                success, q_projected, error = (
                    self.graph_builder.apply_state_constraints(
                        state_name=source_state,
                        q=q_current,
                        max_iterations=10000,
                        error_threshold=1e-4,
                    )
                )

                if not success:
                    raise RuntimeError(
                        f"Failed to project q_current onto state "
                        f"'{source_state}' (error={error:.6f})"
                    )

                q_current = list(q_projected)

            except Exception as e:
                # Store partial phase result for projection failure
                partial_phase_result = {
                    "phase": phase_idx + 1,
                    "gripper": gripper,
                    "handle": handle,
                    "edges": edge_sequence,
                    "paths": [],
                    "complete": False,
                    "failed_edge_idx": 0,
                    "failed_edge_name": (
                        edge_sequence[0] if edge_sequence else None
                    ),
                    "last_q_start": q_current,
                    "failed_q_target": None,
                    "error_message": f"State projection failed: {e}",
                }
                self.phase_results.append(partial_phase_result)

                self.last_failure_info = {
                    "phase_idx": phase_idx,
                    "edge_idx": 0,
                    "edge_name": (
                        edge_sequence[0] if edge_sequence else "unknown"
                    ),
                    "q_current": q_current,
                    "error": f"State projection failed: {e}",
                    "completed_phases": len(
                        [
                            p
                            for p in self.phase_results
                            if p.get("complete", False)
                        ]
                    ),
                    "completed_edges_in_phase": 0,
                }

                if verbose:
                    print(
                        f"\n  ⚠ Stored partial phase result: projection failed at phase start"
                    )

                raise RuntimeError(
                    f"Phase {phase_idx + 1}: State projection failed: {e}"
                ) from e

            # Plan edges (with selective retry)
            phase_paths = []
            phase_geometric_paths = []  # Geometric paths for saving
            edge_stats_list = []  # Per-edge timing stats for this phase
            q_start = q_current

            # For first phase in resume, decide where to start
            start_edge_idx = 0
            if phase_idx_offset == 0 and retry_from_edge >= 0:
                # Resuming failed phase: start from specified edge
                start_edge_idx = retry_from_edge
                if start_edge_idx < resume_state['completed_edges_in_phase']:
                    # Reuse previously completed paths in this phase
                    # (though this is complex - for now just restart phase)
                    pass

            for edge_idx in range(start_edge_idx, len(edge_sequence)):
                # Check for stop request
                if is_stop_requested():
                    if verbose:
                        print("\n  ⚠️  Stop requested - saving progress...")
                    # Save partial result
                    partial_phase_result = {
                        "phase": phase_idx + 1,
                        "gripper": gripper,
                        "handle": handle,
                        "edges": edge_sequence,
                        "paths": phase_paths,
                        "edge_stats": edge_stats_list,
                        "complete": False,
                        "failed_edge_idx": edge_idx,
                        "failed_edge_name": edge_name,
                        "last_q_start": q_start,
                        "stopped": True,
                        "error_message": "Stopped by user request",
                    }
                    self.phase_results.append(partial_phase_result)

                    self.last_failure_info = {
                        "phase_idx": phase_idx,
                        "edge_idx": edge_idx,
                        "edge_name": edge_name,
                        "q_current": q_current,
                        "error": "Stopped by user request (Ctrl+C)",
                        "completed_phases": len(
                            [p for p in self.phase_results if p.get("complete", False)]
                        ),
                        "completed_edges_in_phase": len(phase_paths),
                        "stopped": True,
                    }

                    if verbose:
                        print(f"\n  ⚠️  Stored partial phase result: {len(phase_paths)} edges completed")
                        print(f"     You can resume from Phase {phase_idx + 1}, Edge {edge_idx + 1}")

                    # Disable signal handler before raising
                    disable_graceful_stop()
                    raise KeyboardInterrupt("Planning stopped by user request")

                edge_name = edge_sequence[edge_idx]
                edge_start_time = time.time()

                # Get previous attempt count for this edge
                prev_stat = self.edge_stats.get((phase_idx, edge_idx))
                attempt_num = (prev_stat["attempt"] + 1) if prev_stat else 1

                edge_stat = {
                    "edge_idx": edge_idx,
                    "edge_name": edge_name,
                    "attempt": attempt_num,
                    "gen_time": 0.0,
                    "plan_time": 0.0,
                    "total_time": 0.0,
                    "success": False,
                    "is_resume": True,
                }

                attempt_str = (
                    f" (attempt #{attempt_num})" if attempt_num > 1 else ""
                )
                if verbose:
                    print(
                        f"  Planning waypoint edge "
                        f"{edge_idx + 1}/{len(edge_sequence)}: "
                        f"{edge_name}{attempt_str}"
                    )

                # Generate target
                gen_start = time.time()
                q_target = None
                try:
                    ok, q_target = self.config_gen.generate_via_edge(
                        edge_name=edge_name,
                        q_from=q_start,
                        config_label=f"q_phase{phase_idx}_edge{edge_idx}_resume",
                    )
                    if not ok or q_target is None:
                        raise RuntimeError(
                            f"Failed to generate target via edge '{edge_name}'"
                        )
                    edge_stat["gen_time"] = time.time() - gen_start
                except Exception as e:
                    edge_stat["gen_time"] = time.time() - gen_start
                    edge_stat["total_time"] = time.time() - edge_start_time
                    edge_stats_list.append(edge_stat)

                    # Store partial result
                    partial_phase_result = {
                        "phase": phase_idx + 1,
                        "gripper": gripper,
                        "handle": handle,
                        "edges": edge_sequence,
                        "paths": phase_paths,
                        "edge_stats": edge_stats_list,
                        "complete": False,
                        "failed_edge_idx": edge_idx,
                        "failed_edge_name": edge_name,
                        "last_q_start": q_start,
                        "failed_q_target": None,
                        "error_message": f"Target generation failed: {e}",
                    }
                    self.phase_results.append(partial_phase_result)

                    self.last_failure_info = {
                        "phase_idx": phase_idx,
                        "edge_idx": edge_idx,
                        "edge_name": edge_name,
                        "q_current": q_current,
                        "error": f"Target generation failed: {e}",
                        "completed_phases": len(
                            [
                                p
                                for p in self.phase_results
                                if p.get("complete", False)
                            ]
                        ),
                        "completed_edges_in_phase": len(phase_paths),
                    }

                    if verbose:
                        print(
                            f"\n  ⚠ Stored partial phase result: {len(phase_paths)} edges completed"
                        )

                    raise RuntimeError(
                        f"Phase {phase_idx + 1}, edge {edge_idx + 1}: "
                        f"Target generation failed: {e}"
                    ) from e

                # Check if this phase should skip motion planning
                if skip_phases and phase_idx in skip_phases:
                    # Skip motion planning, use q_target directly
                    edge_stat["total_time"] = time.time() - edge_start_time
                    edge_stat["skipped"] = True
                    edge_stat["success"] = True
                    edge_stats_list.append(edge_stat)
                    self.total_planning_time += edge_stat["total_time"]
                    self.edge_stats[(phase_idx, edge_idx)] = edge_stat

                    # Use q_target as next start config
                    q_start = q_target

                    # Append None placeholder for skipped path
                    phase_paths.append(None)
                    if self.auto_save_dir:
                        phase_geometric_paths.append(None)

                    if verbose:
                        print(
                            f"     ⏭ Skipped motion planning "
                            f"({edge_stat['total_time']:.2f}s total)"
                        )

                    continue

                # Plan edge — retry on collision in q_target
                import numpy as _np

                plan_start = time.time()
                last_plan_exc = None
                path = None
                geometric_path = None

                for _plan_attempt in range(self._MAX_COLLISION_RETRIES):
                    try:
                        if verbose:
                            if _plan_attempt == 0:
                                pass  # label printed above
                            else:
                                _prev_reason = (
                                    str(last_plan_exc).split('\n')[0]
                                    if last_plan_exc else ""
                                )
                                print(
                                    f"     Planning (attempt {_plan_attempt + 1}) "
                                    f"[prev failed: {_prev_reason}]"
                                )
                        # Always get both timed and geometric paths
                        path, geometric_path = (
                            self.planner.plan_transition_edge(
                                edge=edge_name,
                                q1=q_start,
                                q2=q_target,
                            )
                        )

                        if path is None:
                            raise RuntimeError(
                                f"Planning failed for edge '{edge_name}'"
                            )

                        last_plan_exc = None
                        break  # success

                    except Exception as _plan_exc:
                        last_plan_exc = _plan_exc
                        if (
                            "Collision" in str(_plan_exc)
                            and _plan_attempt < self._MAX_COLLISION_RETRIES - 1
                        ):
                            if verbose:
                                print(
                                    f"     ⚠ Collision in q_target "
                                    f"(attempt {_plan_attempt + 1}), "
                                    f"regenerating target config..."
                                )
                            _ok2, _q_new = self.config_gen.generate_via_edge(
                                edge_name=edge_name,
                                q_from=q_start,
                                config_label=(
                                    f"q_phase{phase_idx}_edge{edge_idx}"
                                ),
                            )
                            if (
                                _ok2
                                and _q_new is not None
                                and _np.all(_np.isfinite(_np.array(_q_new)))
                            ):
                                q_target = _q_new
                                if verbose:
                                    print("     Regenerated target config")

                if last_plan_exc is not None:
                    e = last_plan_exc
                    edge_stat["plan_time"] = time.time() - plan_start
                    edge_stat["total_time"] = time.time() - edge_start_time
                    edge_stats_list.append(edge_stat)
                    self.total_planning_time += edge_stat["total_time"]

                    # Store partial result again
                    partial_phase_result = {
                        "phase": phase_idx + 1,
                        "gripper": gripper,
                        "handle": handle,
                        "edges": edge_sequence,
                        "paths": phase_paths,
                        "edge_stats": edge_stats_list,
                        "complete": False,
                        "failed_edge_idx": edge_idx,
                        "failed_edge_name": edge_name,
                        "last_q_start": q_start,
                        "failed_q_target": q_target,
                        "error_message": str(e),
                    }
                    self.phase_results.append(partial_phase_result)

                    if verbose:
                        print(
                            f"     ⚠ Failed after {edge_stat['total_time']:.2f}s "
                            f"(attempt #{attempt_num})"
                        )

                    self.last_failure_info = {
                        "phase_idx": phase_idx,
                        "edge_idx": edge_idx,
                        "edge_name": edge_name,
                        "q_current": q_current,
                        "error": str(e),
                        "completed_phases": len(
                            [
                                p
                                for p in self.phase_results
                                if p.get("complete", False)
                            ]
                        ),
                        "completed_edges_in_phase": len(phase_paths),
                    }

                    raise RuntimeError(
                        f"Resume failed at Phase {phase_idx + 1}, edge {edge_idx + 1}: {e}"
                    ) from e

                # Store geometric path if auto-save is enabled
                if self.auto_save_dir:
                    phase_geometric_paths.append(geometric_path)

                edge_stat["plan_time"] = time.time() - plan_start
                edge_stat["total_time"] = time.time() - edge_start_time
                edge_stat["success"] = True
                edge_stats_list.append(edge_stat)
                self.total_planning_time += edge_stat["total_time"]
                self.edge_stats[(phase_idx, edge_idx)] = edge_stat

                phase_paths.append(path)

                if hasattr(path, "getInitialConfig") and hasattr(
                    path, "getEndConfig"
                ):
                    q_start = list(path.getEndConfig())
                else:
                    q_start = q_target

                if verbose:
                    print(
                        f"     ✓ Path found ({edge_stat['plan_time']:.2f}s plan, "
                        f"{edge_stat['total_time']:.2f}s total)"
                    )

            # Phase completed successfully
            q_current = q_start
            self.grasp_tracker.update_grasp(gripper, handle)

            # Compute phase timing totals
            phase_total_time = sum(s["total_time"] for s in edge_stats_list)
            phase_plan_time = sum(s["plan_time"] for s in edge_stats_list)
            phase_gen_time = sum(s["gen_time"] for s in edge_stats_list)

            # Auto-save paths after successful phase (resume)
            saved_files = self._auto_save_phase_paths(
                phase_idx=phase_idx,
                phase_paths=phase_paths,
                edge_names=edge_sequence,
                verbose=verbose,
                phase_geometric_paths=phase_geometric_paths if self.auto_save_dir else None,
            )

            phase_result = {
                "phase": phase_idx + 1,
                "gripper": gripper,
                "handle": handle,
                "edges": edge_sequence,
                "paths": phase_paths,
                "edge_stats": edge_stats_list,
                "phase_time": phase_total_time,
                "phase_plan_time": phase_plan_time,
                "phase_gen_time": phase_gen_time,
                "complete": True,
                "skipped": skip_phases and phase_idx in skip_phases,
                "final_config": q_current,
                "state_after": self.grasp_tracker.get_current_state_name(),
                "saved_files": saved_files,  # Track saved path files
            }
            self.phase_results.append(phase_result)

            if verbose:
                print(
                    f"  ✓ Completed phase {phase_idx + 1} "
                    f"({phase_total_time:.2f}s total)"
                )

        # Clear failure info on success
        self.last_failure_info = None

        if verbose:
            print("\n" + "=" * 70)
            print("Resume Complete - All Phases Succeeded")
            print(f"Total planning time: {self.total_planning_time:.2f}s")
            print(f"Resume attempts: {self.resume_attempt_count}")
            print("=" * 70)

        # Disable signal handler before returning
        disable_graceful_stop()

        return {
            "success": True,
            "paths": (
                self.phase_results[-1]["paths"] if self.phase_results else []
            ),
            "phase_results": self.phase_results,
            "final_config": q_current,
            "grasp_tracker": self.grasp_tracker,
        }

    def replay_sequence(
        self,
        speed: float = 1.0,
        clear_paths_first: bool = False,
        visualizer: Optional[Any] = None,
        record: bool = False,
        output_dir: str = "/home/dvtnguyen/devel/demos",
        video_prefix: Optional[str] = None,
        framerate: int = 25,
        dt: float = 0.01,
    ) -> Optional[List[str]]:
        """Replay all phase paths in sequence.

        Args:
            speed: Playback speed multiplier
            clear_paths_first: If True, warn about accumulated paths before replay
            visualizer: Optional LiveConstraintGraphVisualizer for real-time graph updates
            record: If True, record video of the replay (default: True)
            output_dir: Directory for video output (default: /home/dvtnguyen/devel/demos)
            video_prefix: Optional prefix for video filenames
            framerate: Video framerate in fps (default: 25)
            dt: Time step for path sampling (default: 0.01)
            
        Returns:
            List of video file paths if recording enabled, None otherwise
        """
        if not self.phase_results:
            print("No phases to replay (run plan_sequence first)")
            return

        print("\n" + "=" * 70)
        print("Replaying Grasp Sequence")
        if visualizer:
            print("Live graph visualization: ENABLED")
        if record:
            print(f"Video recording: ENABLED (output: {output_dir})")
        print("=" * 70)

        recorded_videos = []

        # Check for path accumulation
        if hasattr(self.planner, "get_num_stored_paths"):
            num_stored = self.planner.get_num_stored_paths()
            if num_stored > 0:
                print(
                    f"\nNote: {num_stored} paths already stored in ProblemSolver"
                )
                print(
                    "      Replay will add more paths (hpp has no clear API)"
                )
                if clear_paths_first:
                    print("      Consider restarting to clear memory")

        for phase in self.phase_results:
            is_complete = phase.get("complete", True)
            status = "✓" if is_complete else "⚠ INCOMPLETE"

            print(
                f"\nPhase {phase['phase']}: "
                f"{phase['gripper']} grasps {phase['handle']} [{status}]"
            )
            print(f"  Edges: {', '.join(phase['edges'])}")

            # Check if phase was skipped
            if phase.get("skipped"):
                print(f"  ⏭ Phase skipped (no paths to replay)")
                continue

            if not is_complete:
                failed_edge = phase.get('failed_edge_idx', -1)
                print(f"  ⚠ Failed at edge {failed_edge + 1}: {phase.get('failed_edge_name', 'unknown')}")
                print(f"  Error: {phase.get('error_message', 'unknown')}")

            # Filter out None paths from skipped edges
            valid_paths = [(idx, path) for idx, path in enumerate(phase["paths"]) if path is not None]
            print(f"  Playing {len(valid_paths)} waypoint paths...")

            try:
                # Play each waypoint path in the sequence
                edge_names = phase.get("edges", [])
                for idx, path in valid_paths:
                    edge_name = edge_names[idx] if idx < len(edge_names) else None

                    print(
                        f"    Path {idx + 1}/{len(phase['paths'])}: ", end=""
                    )

                    # Generate video name if recording
                    video_name_for_path = None
                    if record:
                        if video_prefix:
                            video_name_for_path = f"{video_prefix}_phase_{phase['phase']:02d}_path_{idx + 1:02d}"
                        else:
                            video_name_for_path = f"phase_{phase['phase']:02d}_path_{idx + 1:02d}"
                        if edge_name:
                            video_name_for_path += f"_{sanitize_filename(edge_name)}"

                    if record and hasattr(self.planner, "play_and_record_path_vector"):
                        # Record the playback
                        path_idx, video_file = self.planner.play_and_record_path_vector(
                            path,
                            video_name=video_name_for_path,
                            output_dir=output_dir,
                            framerate=framerate,
                            dt=dt,
                            speed=speed,
                        )
                        print(f"✓ Recorded (index {path_idx}): {video_file}")
                        recorded_videos.append(video_file)
                    elif visualizer and hasattr(self.planner, "play_path_vector_with_viz"):
                        # Use visualization-enabled playback
                        path_idx = self.planner.play_path_vector_with_viz(
                            path,
                            edge_name=edge_name,
                            visualizer=visualizer,
                            speed=speed,
                        )
                        print(f"✓ Played with visualization (stored as index {path_idx})")
                    elif hasattr(self.planner, "play_path_vector"):
                        # Standard playback without visualization
                        path_idx = self.planner.play_path_vector(
                            path, speed=speed
                        )
                        print(f"✓ Played (stored as index {path_idx})")
                    else:
                        print(
                            f"⚠ Backend does not support PathVector playback"
                        )
                        print(f"      Path type: {type(path).__name__}")
            except Exception as e:
                print(f"\n  ⚠ Failed to replay: {e}")

        # Final path count
        if hasattr(self.planner, "get_num_stored_paths"):
            final_count = self.planner.get_num_stored_paths()
            print(f"\n{'='*70}")
            print(f"Total paths now in ProblemSolver: {final_count}")

        if record and recorded_videos:
            print(f"\n📹 Recorded {len(recorded_videos)} videos to {output_dir}")
            return recorded_videos
        return None

    def get_phase_summary(self) -> str:
        """Get human-readable summary of all phases with timing statistics.

        Returns:
            Multi-line summary string including per-edge timing breakdown
        """
        if not self.phase_results:
            return "No phases executed"

        lines = ["\nGrasp Sequence Summary:"]
        lines.append("=" * 60)

        total_time = 0.0
        total_gen_time = 0.0
        total_plan_time = 0.0

        for phase in self.phase_results:
            is_complete = phase.get("complete", True)
            status = "✓" if is_complete else "⚠ INCOMPLETE"

            # Phase header with timing
            phase_time = phase.get("phase_time", 0.0)
            total_time += phase_time
            total_gen_time += phase.get("phase_gen_time", 0.0)
            total_plan_time += phase.get("phase_plan_time", 0.0)

            time_str = f" ({phase_time:.2f}s)" if phase_time > 0 else ""
            skip_marker = " [SKIPPED]" if phase.get("skipped") else ""
            lines.append(
                f"\nPhase {phase['phase']}: "
                f"{phase['gripper']} → {phase['handle']} [{status}]{time_str}{skip_marker}"
            )

            # Edge details with timing
            edge_stats = phase.get("edge_stats", [])
            if edge_stats:
                lines.append("  Edge breakdown:")
                for stat in edge_stats:
                    edge_idx = stat.get("edge_idx", 0)
                    edge_name = stat.get("edge_name", "unknown")
                    attempt = stat.get("attempt", 1)
                    gen_t = stat.get("gen_time", 0.0)
                    plan_t = stat.get("plan_time", 0.0)
                    total_t = stat.get("total_time", 0.0)
                    success = stat.get("success", False)
                    is_resume = stat.get("is_resume", False)
                    is_skipped = stat.get("skipped", False)

                    status_icon = "⏭" if is_skipped else ("✓" if success else "✗")
                    attempt_str = (
                        f" (attempt #{attempt})" if attempt > 1 else ""
                    )
                    resume_str = " [resume]" if is_resume else ""

                    lines.append(
                        f"    {status_icon} Edge {edge_idx + 1}: "
                        f"{edge_name}{attempt_str}{resume_str}"
                    )
                    if is_skipped:
                        lines.append(
                            f"       skipped (gen: {gen_t:.2f}s)"
                        )
                    else:
                        lines.append(
                            f"       gen: {gen_t:.2f}s | "
                            f"plan: {plan_t:.2f}s | total: {total_t:.2f}s"
                        )
            else:
                # Fallback for phases without edge_stats
                lines.append(f"  Edges: {', '.join(phase['edges'])}")

            if not is_complete:
                failed_edge = phase.get('failed_edge_idx', -1)
                lines.append(
                    f"  ⚠ Failed at edge {failed_edge + 1}: "
                    f"{phase.get('failed_edge_name', 'unknown')}"
                )
                lines.append(
                    f"  Completed paths: {len(phase['paths'])} "
                    f"of {len(phase['edges'])}"
                )
            else:
                lines.append(f"  Paths: {len(phase['paths'])} waypoint paths")
                lines.append(f"  State: {phase.get('state_after', 'unknown')}")

        lines.append("\n" + "=" * 60)

        # Summary statistics
        lines.append("Timing Summary:")
        lines.append(f"  Total computation time: {total_time:.2f}s")
        lines.append(f"    - Config generation: {total_gen_time:.2f}s")
        lines.append(f"    - Path planning:     {total_plan_time:.2f}s")
        if self.resume_attempt_count > 0:
            lines.append(f"  Resume attempts: {self.resume_attempt_count}")

        # Show final state only if last phase is complete
        if self.phase_results:
            last_phase = self.phase_results[-1]
            if last_phase.get("complete", True):
                lines.append(
                    f"\nFinal state: {last_phase.get('state_after', 'unknown')}"
                )
            else:
                lines.append("\nFinal state: Incomplete (planning failed)")

        return "\n".join(lines)

    def reset(self) -> None:
        """Reset planner to initial free state."""
        self.grasp_tracker = GraspStateTracker(
            grippers=self.grasp_tracker.grippers,
            handles=self.grasp_tracker.handles,
            initial_grasps=None,
        )
        self.phase_results = []
        self.last_failure_info = None
        self.original_sequence = None
        self.edge_stats = {}
        self.total_planning_time = 0.0
        self.resume_attempt_count = 0


class InteractiveGraspSequenceBuilder:
    """Interactive builder for grasp sequence planning.

    Provides a menu-driven interface for:
    - Selecting grasps from available pairs
    - Configuring skip phases
    - Selecting frozen arms mode
    - Setting auto-save options
    - Running the sequence planner with interactive resume on failure

    Example:
        >>> builder = InteractiveGraspSequenceBuilder(task, cfg)
        >>> result = builder.run()
        >>> if result["success"]:
        ...     print("Sequence planning succeeded!")
    """

    def __init__(
        self,
        task: Any,
        task_config: Any,
        freeze_joint_substrings: Optional[List[str]] = None,
    ):
        """Initialize interactive grasp sequence builder.

        Args:
            task: ManipulationTask instance with planner, graph_builder, etc.
            task_config: Task configuration with VALID_PAIRS.
            freeze_joint_substrings: Default joint substrings to freeze.
        """
        self.task = task
        self.task_config = task_config
        self.freeze_joint_substrings = freeze_joint_substrings or []

        # Will be populated during run()
        self.grasp_sequence: List[Tuple[str, str]] = []
        self.skip_phases: Set[int] = set()
        self.skip_all_phases: bool = False
        self.frozen_arms_mode: str = "auto"
        self.per_phase_frozen_arms: Optional[Dict[int, List[str]]] = None
        self.auto_save_dir: Optional[str] = None
        self.non_stop: bool = False

    def _get_available_grasps(self) -> List[Tuple[str, str]]:
        """Get all possible grasps from config."""
        all_grasps = []
        valid_pairs = getattr(self.task_config, "VALID_PAIRS", {})
        for gripper, handles in valid_pairs.items():
            for handle in handles:
                all_grasps.append((gripper, handle))
        return all_grasps

    def select_sequence(self) -> bool:
        """Interactively select the grasp sequence.

        Returns:
            True if a sequence was selected, False if cancelled.
        """
        from agimus_spacelab.utils.interactive import interactive_menu

        all_grasps = self._get_available_grasps()
        if not all_grasps:
            print("No valid grasps available in config.")
            return False

        # Format for display
        grasp_options = [
            f"{gripper} → {handle}" for gripper, handle in all_grasps
        ] + [
            "[Done - Start Planning]",
            "[Done - Start Planning (non stop)]",
        ]

        self.grasp_sequence = []
        done_non_stop_idx = len(all_grasps) + 1

        while True:
            # Show current sequence
            if self.grasp_sequence:
                print("\nCurrent sequence:")
                for i, (g, h) in enumerate(self.grasp_sequence, 1):
                    print(f"  {i}. {g} → {h}")
            else:
                print("\nSequence is empty. Select grasps to add.")

            # Select next grasp
            selected = interactive_menu(
                "Select next grasp to add (or Done to plan):",
                grasp_options,
                multi_select=False,
            )

            if not selected:
                return False

            if selected[0] == len(all_grasps):  # Done - Start Planning
                break
            if selected[0] == done_non_stop_idx:
                self.non_stop = True
                break

            self.grasp_sequence.append(all_grasps[selected[0]])

        return len(self.grasp_sequence) > 0

    def configure_skip_phases(self) -> None:
        """Configure which phases to skip."""
        from agimus_spacelab.cli.interactive_pickers import select_skip_phases

        if len(self.grasp_sequence) >= 1:
            self.skip_phases, self.skip_all_phases = select_skip_phases(
                self.grasp_sequence
            )

    def configure_frozen_arms(self) -> None:
        """Configure frozen arms mode."""
        from agimus_spacelab.cli.interactive_pickers import select_frozen_arms_mode

        self.frozen_arms_mode, self.per_phase_frozen_arms = select_frozen_arms_mode(
            self.grasp_sequence
        )

    def configure_auto_save(self) -> None:
        """Configure auto-save directory."""
        from agimus_spacelab.cli.interactive_pickers import select_auto_save_directory

        self.auto_save_dir = select_auto_save_directory()

    def _create_arm_selector_callback(self):
        """Create interactive arm selector callback for interactive mode."""
        from agimus_spacelab.utils.interactive import interactive_menu

        def interactive_arm_selector(phase_idx, gripper, arm_keywords):
            """Callback for interactive arm selection per phase."""
            print(f"\n  Select arms to freeze for Phase {phase_idx + 1}:")
            print(f"  Active gripper: {gripper}")

            selected = interactive_menu(
                "Select arm(s) to freeze:",
                arm_keywords + ["[None - No Locking]"],
                multi_select=True,
            )

            arm_count = len(arm_keywords)
            if selected and selected[0] < arm_count:
                return [arm_keywords[i] for i in selected if i < arm_count]
            return []

        return interactive_arm_selector

    def run(self) -> Dict[str, Any]:
        """Run the interactive grasp sequence planning workflow.

        Returns:
            Dictionary with 'success' and planning results.
        """
        print("\n=== Interactive Grasp Sequence Planning ===")

        # Ensure task is ready
        if not self._ensure_task_ready():
            return {"success": False, "error": "Task setup failed"}

        # Step 1: Select sequence
        if not self.select_sequence():
            return {"success": False, "error": "No grasps selected"}

        print(f"\nPlanning sequence of {len(self.grasp_sequence)} grasps...")

        # Step 2: Configure options
        self.configure_skip_phases()
        self.configure_frozen_arms()
        self.configure_auto_save()

        if self.non_stop:
            print(
                "Non stop mode enabled: will automatically resume on failure "
                "(Ctrl+C to stop)."
            )

        # Step 3: Get q_init
        q_init = self._get_q_init()
        if q_init is None:
            print("Error: q_init not available.")
            return {"success": False, "error": "q_init not available"}

        # Step 4: Create and run planner
        try:
            planner = GraspSequencePlanner(
                graph_builder=self.task.graph_builder,
                config_gen=self.task.config_gen,
                planner=self.task.planner,
                task_config=self.task_config,
                backend=self.task.backend,
                pyhpp_constraints=getattr(self.task, "pyhpp_constraints", {}),
                graph_constraints=getattr(self.task, "_graph_constraints", None),
                auto_save_dir=self.auto_save_dir,
            )

            # Set interactive callback if in interactive mode
            if self.frozen_arms_mode == "interactive":
                planner.interactive_arm_selector_callback = (
                    self._create_arm_selector_callback()
                )

            result = planner.plan_sequence(
                grasp_sequence=self.grasp_sequence,
                q_init=q_init,
                frozen_arms_mode=self.frozen_arms_mode,
                per_phase_frozen_arms=self.per_phase_frozen_arms,
                skip_phases=self.skip_phases if self.skip_phases else None,
                verbose=True,
            )

            if result.get("success"):
                print("\n" + "=" * 70)
                print("Sequence planning succeeded!")
                print(planner.get_phase_summary())
                # Offer replay or browse after success
                self._offer_replay_or_browse(planner, q_init)
            else:
                self._handle_failure(planner, q_init)

            # Show saved files summary
            self._show_saved_files_summary(planner)

            return {
                "success": result.get("success", False),
                "planner": planner,
                "result": result,
            }

        except Exception as e:
            print(f"\nSequence planning error: {e}")
            import traceback
            traceback.print_exc()
            return {"success": False, "error": str(e)}

    def _ensure_task_ready(self) -> bool:
        """Ensure task is set up for planning."""
        if not hasattr(self.task, "graph_builder") or self.task.graph_builder is None:
            print("Error: Task graph_builder not initialized.")
            return False
        if not hasattr(self.task, "planner") or self.task.planner is None:
            print("Error: Task planner not initialized.")
            return False
        return True

    def _get_q_init(self) -> Optional[List[float]]:
        """Get initial configuration."""
        if hasattr(self.task, "config_gen") and self.task.config_gen is not None:
            q_init = self.task.config_gen.configs.get("q_init")
            if q_init is not None:
                return q_init
        return getattr(self.task, "q_init", None)

    def _handle_failure(
        self, planner: GraspSequencePlanner, q_init: Optional[List[float]] = None
    ) -> None:
        """Handle planning failure with optional resume.
        
        Args:
            planner: The GraspSequencePlanner instance.
            q_init: Initial configuration for replay/browse after resume.
        """
        from agimus_spacelab.utils.interactive import interactive_menu

        if not hasattr(planner, "get_resumable_state"):
            print("Sequence planning failed.")
            return

        resume_state = planner.get_resumable_state()
        if not resume_state:
            print("Sequence planning failed. No resumable state.")
            return

        print("\n" + "=" * 70)
        print("Planning Failed - Partial Progress Saved")
        print("=" * 70)
        print(
            f"Failed at: Phase {resume_state['phase_idx'] + 1}, "
            f"Edge {resume_state['edge_idx'] + 1}"
        )
        print(f"Completed phases: {resume_state['completed_phases']}")
        print(f"Error: {resume_state['error']}")
        print(planner.get_phase_summary())

        if self.non_stop:
            self._auto_resume_loop(planner, q_init)
        else:
            self._interactive_resume_loop(planner, q_init)

    def _auto_resume_loop(
        self, planner: GraspSequencePlanner, q_init: Optional[List[float]] = None
    ) -> None:
        """Auto-resume loop for non-stop mode.
        
        Args:
            planner: The GraspSequencePlanner instance.
            q_init: Initial configuration for replay/browse after success.
        """
        print("\nNon stop mode: auto-resuming (Press Ctrl+C to stop)")

        while True:
            resume_state = planner.get_resumable_state()
            if not resume_state:
                break
            try:
                result = planner.resume_sequence(
                    retry_from_edge=-1,
                    timeout_per_edge=300.0,
                    max_iterations_per_edge=1000000,
                    frozen_arms_mode=self.frozen_arms_mode,
                    per_phase_frozen_arms=self.per_phase_frozen_arms,
                    skip_phases=self.skip_phases if self.skip_phases else None,
                    verbose=True,
                )
                if result.get("success"):
                    print("\n" + "=" * 70)
                    print("Resume succeeded!")
                    print(planner.get_phase_summary())
                    # Offer replay after successful resume
                    self._offer_replay_or_browse(planner, q_init)
                    break
            except KeyboardInterrupt:
                print("\nNon stop resume interrupted by user.")
                break
            except Exception as e:
                print(f"\nAuto-resume failed: {e}")

    def _interactive_resume_loop(
        self, planner: GraspSequencePlanner, q_init: Optional[List[float]] = None
    ) -> None:
        """Interactive resume loop with menu options.
        
        Args:
            planner: The GraspSequencePlanner instance.
            q_init: Initial configuration for replay/browse after success.
        """
        from agimus_spacelab.utils.interactive import interactive_menu

        while True:
            resume_state = planner.get_resumable_state()
            if not resume_state:
                break

            options = [
                "[R] Replay completed paths",
                "[1] Retry from failed edge",
                "[2] Retry from start of failed phase",
                "[3] Retry with increased timeout",
                "[4] Retry with increased max iterations",
                "[Q] Quit to menu",
            ]

            selected = interactive_menu(
                "Resume Options:",
                options,
                multi_select=False,
            )

            if not selected or selected[0] == 5:  # Quit
                break

            if selected[0] == 0:  # Replay
                print("\nReplaying completed paths...")
                planner.replay_sequence()
                continue

            retry_edge = -1 if selected[0] in [0, 2, 4] else 0
            timeout = 120.0 if selected[0] == 2 else None
            max_iters = 10000 if selected[0] == 3 else None

            try:
                result = planner.resume_sequence(
                    retry_from_edge=retry_edge,
                    timeout_per_edge=timeout,
                    max_iterations_per_edge=max_iters,
                    frozen_arms_mode=self.frozen_arms_mode,
                    per_phase_frozen_arms=self.per_phase_frozen_arms,
                    skip_phases=self.skip_phases if self.skip_phases else None,
                    verbose=True,
                )

                if result.get("success"):
                    print("\n" + "=" * 70)
                    print("Resume succeeded!")
                    print(planner.get_phase_summary())
                    # Offer replay/browse after successful resume
                    self._offer_replay_or_browse(planner, q_init)
                    break
            except Exception as e:
                print(f"\nResume failed: {e}")

    def _collect_generated_configs(
        self, planner: GraspSequencePlanner, q_init: List[float]
    ) -> Dict[str, List[float]]:
        """Collect all generated configurations from the planner.
        
        Args:
            planner: The GraspSequencePlanner instance.
            q_init: Initial configuration.
            
        Returns:
            Dictionary mapping configuration names to configuration vectors.
        """
        generated_configs = {}

        # Add q_init
        generated_configs["q_init (Initial)"] = q_init

        # Collect configs from each phase
        for phase_idx, phase_result in enumerate(planner.phase_results):
            if not phase_result:
                continue

            gripper = phase_result.get("gripper", "")
            handle = phase_result.get("handle", "")
            phase_label = f"Phase {phase_idx + 1}: {gripper} → {handle}"

            # Add edge configs (waypoints)
            edge_sequence = phase_result.get("edges", [])
            for edge_idx, edge_name in enumerate(edge_sequence):
                config_label = f"q_phase{phase_idx}_edge{edge_idx}"
                if (
                    planner.config_gen
                    and config_label in planner.config_gen.configs
                ):
                    # Create friendly name from edge name
                    edge_type = "waypoint"
                    if "pregrasp" in edge_name.lower():
                        edge_type = "pregrasp"
                    elif (
                        "grasp" in edge_name.lower()
                        and "pre" not in edge_name.lower()
                    ):
                        edge_type = "grasp"
                    elif "placement" in edge_name.lower():
                        edge_type = "placement"

                    friendly_name = (
                        f"{phase_label} - Edge {edge_idx + 1} ({edge_type})"
                    )
                    generated_configs[friendly_name] = (
                        planner.config_gen.configs[config_label]
                    )

            # Add final config for this phase
            if "final_config" in phase_result:
                final_label = f"{phase_label} - Final"
                generated_configs[final_label] = phase_result["final_config"]

        return generated_configs

    def _offer_replay_or_browse(
        self, planner: GraspSequencePlanner, q_init: Optional[List[float]]
    ) -> None:
        """Offer replay for non-skipped phases or browse for skipped phases.
        
        Args:
            planner: The GraspSequencePlanner instance.
            q_init: Initial configuration for browsing.
        """
        from agimus_spacelab.cli.interactive_pickers import browse_configurations
        from agimus_spacelab.utils.interactive import interactive_menu

        # Check if all phases were skipped (config generation only)
        if self.skip_all_phases and planner.config_gen and q_init:
            print("\n" + "=" * 70)
            print("Configuration Generation Complete")
            print("=" * 70)

            generated_configs = self._collect_generated_configs(planner, q_init)
            print(
                f"\nGenerated {len(generated_configs)} configurations "
                f"across {len(planner.phase_results)} phases"
            )

            options = [
                "Browse configurations interactively",
                "Skip",
            ]
            selected = interactive_menu(
                "What would you like to do?",
                options,
                multi_select=False,
            )

            if selected and selected[0] == 0:
                browse_configurations(self.task, generated_configs)

        # Offer replay for non-skipped phases
        elif not self.skip_all_phases and planner.phase_results:
            num_phases = len(planner.phase_results)
            total_paths = sum(
                len(pr.get("paths", [])) for pr in planner.phase_results if pr
            )

            options = [
                f"Replay all paths ({total_paths} paths from {num_phases} phases)",
                "Replay individual phase",
                "Skip replay",
            ]
            selected = interactive_menu(
                "Replay completed paths?",
                options,
                multi_select=False,
            )

            if not selected or selected[0] == 2:
                return

            if selected[0] == 0:  # Replay all
                print("\nReplaying all completed paths...")
                planner.replay_sequence()

            elif selected[0] == 1:  # Replay individual phase
                phase_options = [
                    f"Phase {i + 1}: {pr.get('gripper', '')} → {pr.get('handle', '')}"
                    for i, pr in enumerate(planner.phase_results)
                    if pr
                ] + ["Back"]

                while True:
                    phase_selected = interactive_menu(
                        "Select phase to replay:",
                        phase_options,
                        multi_select=False,
                    )

                    if (
                        not phase_selected
                        or phase_selected[0] >= len(planner.phase_results)
                    ):
                        break

                    phase_idx = phase_selected[0]
                    phase_result = planner.phase_results[phase_idx]
                    if phase_result and "paths" in phase_result:
                        print(f"\nReplaying Phase {phase_idx + 1}...")
                        for path in phase_result["paths"]:
                            try:
                                planner.planner.play_path(path)
                            except Exception as e:
                                print(f"  Failed to replay path: {e}")

    def _show_saved_files_summary(self, planner: GraspSequencePlanner) -> None:
        """Display summary of saved path files.
        
        Args:
            planner: The GraspSequencePlanner instance.
        """
        import os

        if self.auto_save_dir and hasattr(planner, "get_saved_path_files"):
            saved_files = planner.get_saved_path_files()
            if saved_files:
                print(f"\n=== Saved Path Files ({len(saved_files)} files) ===")
                print(f"Directory: {self.auto_save_dir}")
                for f in saved_files:
                    print(f"  - {os.path.basename(f)}")
                print(
                    "\n\u26a0 Note: These paths contain graph edge constraints "
                    "and can only be"
                )
                print(
                    "  replayed within the same session using "
                    "planner.replay_sequence()."
                )
                print(
                    "  They cannot be loaded in a new session without "
                    "recreating the graph."
                )


__all__ = ["GraspSequencePlanner", "InteractiveGraspSequenceBuilder"]
