#!/usr/bin/env python3
"""
Multi-grasp sequential planning using phase-based graph regeneration.

Implements incremental planning for tasks requiring multiple sequential grasps,
where some grasps must remain held while achieving others. Avoids constraint
graph explosion by rebuilding minimal graphs for each phase.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple, Any, Sequence

from agimus_spacelab.planning.grasp_state import GraspStateTracker
from agimus_spacelab.planning.graph import GraphBuilder
from agimus_spacelab.planning.config import ConfigGenerator


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
        """
        self.graph_builder = graph_builder
        self.config_gen = config_gen
        self.planner = planner
        self.task_config = task_config
        self.backend = backend.lower()
        self.pyhpp_constraints = pyhpp_constraints
        self.graph_constraints = graph_constraints

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
        max_iterations_per_edge: int = 5000,
        timeout_per_edge: float = 60.0,
        frozen_arms_mode: str = "auto",
        per_phase_frozen_arms: Optional[Dict[int, List[str]]] = None,
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

        self.phase_results = []
        self.original_sequence = list(grasp_sequence)  # Store for resume
        q_current = list(q_init)

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
                raise RuntimeError(
                    f"Phase {phase_idx + 1}: State projection failed: {e}"
                ) from e

            # Plan through waypoint edge sequence
            # Each edge in sequence has its own path constraints
            # that are easier to satisfy
            phase_paths = []
            q_start = q_current

            for edge_idx, edge_name in enumerate(edge_sequence):
                if verbose:
                    print(
                        f"  Planning waypoint edge "
                        f"{edge_idx + 1}/{len(edge_sequence)}: "
                        f"{edge_name}"
                    )

                # Generate target configuration via this edge
                try:
                    ok, q_target = self.config_gen.generate_via_edge(
                        edge_name=edge_name,
                        q_from=q_start,
                        config_label=f"q_phase{phase_idx}_edge{edge_idx}",
                    )
                    if not ok or q_target is None:
                        raise RuntimeError(
                            f"Failed to generate target via edge "
                            f"'{edge_name}'"
                        )

                    if verbose:
                        print(
                            f"     ✓ Generated target config "
                            f"(first 5): {q_target[:5]}"
                        )

                except Exception as e:
                    raise RuntimeError(
                        f"Phase {phase_idx + 1}, edge {edge_idx + 1}: "
                        f"Target generation failed: {e}"
                    ) from e

                # Plan transition using TransitionPlanner
                try:
                    if verbose:
                        print("     Planning: q_start -> q_target")
                        print(f"     q_start (first 5): {q_start[:5]}")
                        print(f"     q_target (first 5): {q_target[:5]}")

                    path = self.planner.plan_transition_edge(
                        edge=edge_name,
                        q1=q_start,
                        q2=q_target,
                    )

                    if path is None:
                        raise RuntimeError(
                            f"Planning failed for edge '{edge_name}'"
                        )

                    phase_paths.append(path)

                    # Update start config for next edge
                    # Use end of current path
                    if hasattr(path, "getInitialConfig") and hasattr(
                        path, "getEndConfig"
                    ):
                        q_start = list(path.getEndConfig())
                    else:
                        q_start = q_target

                    if verbose:
                        print(
                            f"     ✓ Path found for edge "
                            f"{edge_idx + 1}/{len(edge_sequence)}"
                        )
                        print(
                            f"     Updated q_start (first 5): "
                            f"{q_start[:5]}"
                        )

                except Exception as e:
                    # Store partial phase result before raising
                    partial_phase_result = {
                        "phase": phase_idx + 1,
                        "gripper": gripper,
                        "handle": handle,
                        "edges": edge_sequence,
                        "paths": phase_paths,  # Successfully completed edges
                        "complete": False,
                        "failed_edge_idx": edge_idx,
                        "failed_edge_name": edge_name,
                        "last_q_start": q_start,
                        "failed_q_target": q_target if 'q_target' in locals() else None,
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
                    }

                    if verbose:
                        print(f"\n  ⚠ Stored partial phase result: {len(phase_paths)} edges completed")

                    raise RuntimeError(
                        f"Phase {phase_idx + 1}, edge {edge_idx + 1}: "
                        f"Planning failed for '{edge_name}': {e}"
                    ) from e

            # Update current config to end of sequence
            q_current = q_start

            if verbose:
                print(f"  ✓ Completed {len(edge_sequence)}-edge sequence")
                print(f"     Final q_current (first 5): {q_current[:5]}")

            # Update grasp state after successful planning
            self.grasp_tracker.update_grasp(gripper, handle)

            # Store phase result with all edge paths
            phase_result = {
                "phase": phase_idx + 1,
                "gripper": gripper,
                "handle": handle,
                "edges": edge_sequence,
                "paths": phase_paths,
                "complete": True,
                "final_config": q_current,
                "state_after": self.grasp_tracker.get_current_state_name(),
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

        if verbose:
            print("\n" + "=" * 70)
            print("Resuming Grasp Sequence Planning")
            print("=" * 70)
            print(f"Resuming from Phase {resume_state['phase_idx'] + 1}, "
                  f"Edge {resume_state['edge_idx'] + 1}")
            print(f"Previous error: {resume_state['error']}")
            print(f"Completed phases: {resume_state['completed_phases']}")
            print(f"Completed edges in current phase: "
                  f"{resume_state['completed_edges_in_phase']}")

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
                raise RuntimeError(
                    f"Phase {phase_idx + 1}: State projection failed: {e}"
                ) from e

            # Plan edges (with selective retry)
            phase_paths = []
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
                edge_name = edge_sequence[edge_idx]

                if verbose:
                    print(
                        f"  Planning waypoint edge "
                        f"{edge_idx + 1}/{len(edge_sequence)}: "
                        f"{edge_name}"
                    )

                # Generate target
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
                except Exception as e:
                    raise RuntimeError(
                        f"Phase {phase_idx + 1}, edge {edge_idx + 1}: "
                        f"Target generation failed: {e}"
                    ) from e

                # Plan edge
                try:
                    path = self.planner.plan_transition_edge(
                        edge=edge_name,
                        q1=q_start,
                        q2=q_target,
                    )

                    if path is None:
                        raise RuntimeError(
                            f"Planning failed for edge '{edge_name}'"
                        )

                    phase_paths.append(path)

                    if hasattr(path, "getInitialConfig") and hasattr(
                        path, "getEndConfig"
                    ):
                        q_start = list(path.getEndConfig())
                    else:
                        q_start = q_target

                    if verbose:
                        print(f"     \u2713 Path found")

                except Exception as e:
                    # Store partial result again
                    partial_phase_result = {
                        "phase": phase_idx + 1,
                        "gripper": gripper,
                        "handle": handle,
                        "edges": edge_sequence,
                        "paths": phase_paths,
                        "complete": False,
                        "failed_edge_idx": edge_idx,
                        "failed_edge_name": edge_name,
                        "last_q_start": q_start,
                        "failed_q_target": q_target if 'q_target' in locals() else None,
                        "error_message": str(e),
                    }
                    self.phase_results.append(partial_phase_result)
                    self.last_failure_info = {
                        "phase_idx": phase_idx,
                        "edge_idx": edge_idx,
                        "edge_name": edge_name,
                        "q_current": q_current,
                        "error": str(e),
                    }

                    raise RuntimeError(
                        f"Resume failed at Phase {phase_idx + 1}, edge {edge_idx + 1}: {e}"
                    ) from e

            # Phase completed successfully
            q_current = q_start
            self.grasp_tracker.update_grasp(gripper, handle)

            phase_result = {
                "phase": phase_idx + 1,
                "gripper": gripper,
                "handle": handle,
                "edges": edge_sequence,
                "paths": phase_paths,
                "complete": True,
                "final_config": q_current,
                "state_after": self.grasp_tracker.get_current_state_name(),
            }
            self.phase_results.append(phase_result)

            if verbose:
                print(f"  \u2713 Completed phase {phase_idx + 1}")

        # Clear failure info on success
        self.last_failure_info = None

        if verbose:
            print("\n" + "=" * 70)
            print("Resume Complete - All Phases Succeeded")
            print("=" * 70)

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
        self, speed: float = 1.0, clear_paths_first: bool = False
    ) -> None:
        """Replay all phase paths in sequence.

        Args:
            speed: Playback speed multiplier
            clear_paths_first: If True, warn about accumulated paths before replay
        """
        if not self.phase_results:
            print("No phases to replay (run plan_sequence first)")
            return

        print("\n" + "=" * 70)
        print("Replaying Grasp Sequence")
        print("=" * 70)

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

            if not is_complete:
                failed_edge = phase.get('failed_edge_idx', -1)
                print(f"  ⚠ Failed at edge {failed_edge + 1}: {phase.get('failed_edge_name', 'unknown')}")
                print(f"  Error: {phase.get('error_message', 'unknown')}")

            print(f"  Playing {len(phase['paths'])} waypoint paths...")

            try:
                # Play each waypoint path in the sequence
                for idx, path in enumerate(phase["paths"]):
                    print(
                        f"    Path {idx + 1}/{len(phase['paths'])}: ", end=""
                    )

                    if hasattr(self.planner, "play_path_vector"):
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

    def get_phase_summary(self) -> str:
        """Get human-readable summary of all phases.

        Returns:
            Multi-line summary string
        """
        if not self.phase_results:
            return "No phases executed"

        lines = ["\nGrasp Sequence Summary:"]
        lines.append("-" * 50)

        for phase in self.phase_results:
            is_complete = phase.get("complete", True)
            status = "✓" if is_complete else "⚠ INCOMPLETE"

            lines.append(
                f"Phase {phase['phase']}: "
                f"{phase['gripper']} → {phase['handle']} [{status}]"
            )
            lines.append(f"  Edges: {', '.join(phase['edges'])}")

            if not is_complete:
                failed_edge = phase.get('failed_edge_idx', -1)
                lines.append(f"  ⚠ Failed at edge {failed_edge + 1}: {phase.get('failed_edge_name', 'unknown')}")
                lines.append(f"  Completed paths: {len(phase['paths'])} of {len(phase['edges'])}")
            else:
                lines.append(f"  Paths: {len(phase['paths'])} waypoint paths")
                lines.append(f"  State: {phase.get('state_after', 'unknown')}")

        lines.append("-" * 50)

        # Show final state only if last phase is complete
        if self.phase_results:
            last_phase = self.phase_results[-1]
            if last_phase.get("complete", True):
                lines.append(
                    f"Final state: {last_phase.get('state_after', 'unknown')}"
                )
            else:
                lines.append("Final state: Incomplete (planning failed)")

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


__all__ = ["GraspSequencePlanner"]
