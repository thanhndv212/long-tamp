#!/usr/bin/env python3
"""
Multi-grasp sequential planning using phase-based graph regeneration.

Implements incremental planning for tasks requiring multiple sequential grasps,
where some grasps must remain held while achieving others. Avoids constraint
graph explosion by rebuilding minimal graphs for each phase.
"""

from __future__ import annotations

import json
import os
import signal
import time
from typing import Any, Sequence

from long_tamp.logging import get_logger
from long_tamp.planning.config import ConfigGenerator
from long_tamp.planning.graph import GraphBuilder
from long_tamp.planning.grasp_state import GraspStateTracker

logger = get_logger("tasks.grasp_sequence")

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
        "/": "_",
        ">": "_gt_",
        "<": "_lt_",
        "|": "_or_",
        ":": "_",
        '"': "_",
        "*": "_",
        "?": "_",
        " ": "_",
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
    import threading

    if threading.current_thread() is threading.main_thread():
        signal.signal(signal.SIGINT, _request_stop_signal_handler)


def disable_graceful_stop():
    """Restore default Ctrl+C behavior."""
    import threading

    if threading.current_thread() is threading.main_thread():
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
        backend: str = "pyhpp",
        graph_constraints: list[str] | None = None,
        auto_save_dir: str | None = None,
        run_logger: Any | None = None,
    ):
        """Initialize grasp sequence planner.

        Args:
            graph_builder: GraphBuilder instance for creating phase graphs
            config_gen: ConfigGenerator for waypoint generation
            planner: Backend planner (ManipulationPlanner instance)
            task_config: Task configuration with GRIPPERS, OBJECTS, etc.
            backend: "pyhpp"
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
        self.graph_constraints = graph_constraints
        self._MAX_COLLISION_RETRIES = 10

        # Auto-save configuration
        self.auto_save_dir = auto_save_dir
        self.saved_path_files: list[str] = []  # Track saved file paths

        # Structured run logger (optional)
        self.run_logger = run_logger

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
        # Grasps held when the current plan_sequence() call began; replayed by
        # resume_sequence() so grasps from earlier calls survive a resume.
        self._initial_grasps: dict[str, str] = {}

        # Phase indices whose phase_q_hints chain was broken mid-phase by the
        # collision-retry loop redrawing a hinted edge's target at random.
        # Once that happens the hint's "leaves the next phase reachable"
        # guarantee no longer holds, so callers should re-run
        # find_feasible_phase_target() rather than resume into the next phase.
        self.invalidated_phase_hints: set[int] = set()

        # Edge-level timing and resume attempt statistics
        self.edge_stats = {}  # (phase_idx, edge_idx) -> timing info
        self.total_planning_time = 0.0
        self.resume_attempt_count = 0

        # Gripper-to-arm mapping for locked joint filtering.
        # Loaded from task_config.GRIPPER_TO_ARM_KEYWORD (populated by the
        # arm_groups section of the YAML config).  Falls back to empty when
        # the config has no arm_groups (e.g. single-arm tasks that don't
        # need auto-freeze).
        self.GRIPPER_TO_ARM_MAP = dict(
            getattr(task_config, "GRIPPER_TO_ARM_KEYWORD", {})
        )
        self.ALL_ARM_KEYWORDS = list(getattr(task_config, "ALL_ARM_KEYWORDS", []))

        # Callback for interactive arm selection (set by UI layer)
        self.interactive_arm_selector_callback = None

        # Cache q_pregrasp from each grasping phase: gripper -> q_pregrasp.
        # Used as warm-start seed when releasing that gripper later.
        # Large-clearance handles (e.g. h_RS1_FG: 0.25 m) cause the naive
        # q_grasped seed to fail (FG freeflyer stuck at contact position).
        self._last_pregrasp_q: dict = {}

    def _plan_release_subphase(
        self,
        gripper: str,
        q_current: list,
        phase_graph_constraints: list | None,
        verbose: bool,
    ):
        """Plan a release sub-phase.

        Builds a minimal phase graph for the release transition, plans through
        the two waypoint edges (_21, _10), updates the grasp-state tracker,
        and returns the resulting configuration together with tracking info.

        Args:
            gripper: Gripper name that must release its current object.
            q_current: Current robot configuration.
            phase_graph_constraints: Optional locked-joint constraint names.
            verbose: Whether to print progress.

        Returns:
            Tuple (q_final, phase_info) where phase_info is a dict with keys:
                paths, edges, edge_stats, state_after, final_config,
                phase_time, phase_gen_time, phase_plan_time.
        """
        released_handle = self.grasp_tracker.current_grasps[gripper]

        # Build the release phase graph and sync the grasp-state tracker.
        release_edges = self._setup_release_phase_graph(
            gripper, q_current, phase_graph_constraints, verbose
        )

        # Project q_current onto the source (currently-held) state.
        q_current = self._project_onto_release_source_state(q_current, verbose)

        # Plan through release waypoint edges: _21 (grasped→pregrasp) then
        # _10 (pregrasp→free).  See _plan_release_pregrasp_edge and
        # _generate_and_plan_edge_with_retry for the per-edge retry logic.
        q_start = q_current
        edge_21, edge_10 = release_edges[0], release_edges[1]
        edge_01 = self.grasp_tracker.get_approach_edge_from_released(gripper)

        # Step 1: generate q_pregrasp via the forward approach edge and plan
        # edge_21 (with pregrasp regeneration on plan failure).
        path21, q_pregrasp, t_gen1, t_plan1, plan_err_21 = (
            self._plan_release_pregrasp_edge(
                gripper, released_handle, edge_21, edge_01, q_start, verbose
            )
        )

        # Step 2 (primary): pregrasp -> free, OR direct fallback.
        t_gen2 = 0.0
        t_plan2 = 0.0
        path_direct = None
        t_gen_direct = 0.0
        t_plan_direct = 0.0
        used_direct = False

        if path21 is not None:
            q_start = (
                list(path21.getEndConfig())
                if hasattr(path21, "getEndConfig")
                else q_pregrasp
            )

            if verbose:
                logger.info("Planning release edge: %s", edge_10)

            path10, q_free, t_gen2, t_plan2, plan_err_10 = (
                self._generate_and_plan_edge_with_retry(
                    edge_name=edge_10,
                    q_from=q_start,
                    config_label=f"q_autorelease_{gripper}_free",
                    verbose=verbose,
                    gen_fail_noun="target config",
                )
            )
            if path10 is None:
                raise RuntimeError(
                    f"Auto-release of '{released_handle}': failed to "
                    f"generate/plan edge '{edge_10}' after "
                    f"{self._MAX_COLLISION_RETRIES} attempts "
                    f"(last error: {plan_err_10})"
                )
            q_start = (
                list(path10.getEndConfig())
                if hasattr(path10, "getEndConfig")
                else q_free
            )
        else:
            # --- Fallback: direct release edge (grasped → free, no waypoint) ---
            #
            # The _21 edge constrains g_FG_part to move ONLY along the approach
            # axis (1-DOF foliation).  When the approach axis is geometrically
            # blocked by assembly structure (e.g. NYX), no path exists within
            # the 1-DOF manifold regardless of RRT iterations.
            #
            # The direct LevelSetEdge (name = edge_21 without "_21" suffix) has
            # a looser path constraint: only vispa2's grasp fold is maintained.
            # g_FG_part is free to escape in any direction — giving the RRT full
            # ur10 DOFs to find a collision-free path around the structure.
            if verbose:
                logger.warning(
                    "_21 planning failed (%s), trying direct release edge",
                    plan_err_21,
                )
            direct_edge = edge_21[:-3]  # strip "_21" suffix
            if verbose:
                logger.info("Planning direct release edge: %s", direct_edge)

            path_direct, q_free_d, t_gen_direct, t_plan_direct, plan_err_direct = (
                self._generate_and_plan_edge_with_retry(
                    edge_name=direct_edge,
                    q_from=q_start,
                    config_label=f"q_autorelease_{gripper}_free_direct",
                    verbose=verbose,
                    gen_fail_noun="a free config",
                    q_hint=None,  # random arm seeds — avoids NYX-blocked direction
                )
            )
            if path_direct is None:
                raise RuntimeError(
                    f"Auto-release of '{released_handle}': _21 failed "
                    f"({plan_err_21}) and direct release edge '{direct_edge}' "
                    f"also failed after {self._MAX_COLLISION_RETRIES} attempts "
                    f"(last error: {plan_err_direct})"
                )
            q_start = (
                list(path_direct.getEndConfig())
                if hasattr(path_direct, "getEndConfig")
                else q_free_d
            )
            used_direct = True
            # Placeholders so the rest of the code path is consistent
            path10 = None

        # Update grasp state: gripper is now free
        self.grasp_tracker.update_grasp(gripper, None)
        if verbose:
            logger.info("✓ Released '%s' from '%s'", released_handle, gripper)

        direct_edge = edge_21[:-3] if used_direct else None
        phase_info = self._build_release_phase_info(
            q_start=q_start,
            used_direct=used_direct,
            path21=path21,
            path10=path10,
            path_direct=path_direct,
            edge_21=edge_21,
            edge_10=edge_10,
            direct_edge=direct_edge,
            t_gen1=t_gen1,
            t_plan1=t_plan1,
            t_gen2=t_gen2,
            t_plan2=t_plan2,
            t_gen_direct=t_gen_direct,
            t_plan_direct=t_plan_direct,
        )
        return q_start, phase_info

    def _setup_release_phase_graph(
        self,
        gripper: str,
        q_current: list,
        phase_graph_constraints: list | None,
        verbose: bool,
    ) -> list:
        """Build the release phase graph and sync the grasp-state tracker.

        Constructs a minimal phase graph for the release transition (the named
        gripper releasing its currently-held handle), wires the new graph onto
        ``self.planner.graph`` and ``self.config_gen``, syncs the tracker's
        phase-local indices with the factory ordering, and returns the release
        edge sequence for the gripper.

        Args:
            gripper: Gripper name that must release its current object.
            q_current: Current robot configuration (passed as q_init to the
                phase graph build).
            phase_graph_constraints: Optional locked-joint constraint names.
            verbose: Whether to print progress.

        Returns:
            The release edge sequence (e.g. ``[edge_21, edge_10]``) for the
            gripper, from ``grasp_tracker.get_release_edge_sequence``.
        """
        release_held_grasps = {
            g: h for g, h in self.grasp_tracker.current_grasps.items() if h is not None
        }

        if verbose:
            logger.info(
                "Building release graph: '%s' releases '%s'",
                gripper,
                self.grasp_tracker.current_grasps[gripper],
            )

        try:
            self.graph_builder.build_phase_graph(
                config=self.task_config,
                held_grasps=release_held_grasps,
                next_grasp=(gripper, None),
                graph_constraints=phase_graph_constraints,
                q_init=q_current,
                q_init_original=getattr(self, "_q_scene_init", None),
            )
            new_graph = self.graph_builder.get_graph()
            if hasattr(self.planner, "graph"):
                self.planner.graph = new_graph
            if self.config_gen is None:
                from long_tamp.planning import ConfigGenerator

                self.config_gen = ConfigGenerator(
                    self.graph_builder.robot,
                    new_graph,
                    self.planner,
                    self.graph_builder.ps,
                    backend=self.backend,
                )
            elif hasattr(self.config_gen, "update_graph"):
                self.config_gen.update_graph(new_graph)
        except Exception as e:
            raise RuntimeError(
                f"Auto-release of '{self.grasp_tracker.current_grasps[gripper]}' "
                f"from '{gripper}': Failed to build release graph: {e}"
            ) from e

        # Sync tracker indices with phase-local factory ordering
        if hasattr(self.graph_builder, "_phase_grippers"):
            self.grasp_tracker.set_phase_indices(
                self.graph_builder._phase_grippers,
                self.graph_builder._phase_handles,
            )

        release_edges = self.grasp_tracker.get_release_edge_sequence(gripper)
        if verbose:
            logger.debug("Release edge sequence: %s", release_edges)
        return release_edges

    def _project_onto_release_source_state(
        self, q_current: list, verbose: bool
    ) -> list:
        """Project q_current onto the current (held-grasp) source state.

        Best-effort: on success q_current is replaced with the projected config;
        on failure (projection doesn't converge or raises) the unprojected
        q_current is kept and a warning is printed when verbose.

        Args:
            q_current: Current robot configuration.
            verbose: Whether to print progress.

        Returns:
            The (possibly projected) q_current as a list.
        """
        source_state = self.grasp_tracker.get_current_state_name()
        try:
            success, q_projected, error = self.graph_builder.apply_state_constraints(
                state_name=source_state,
                q=q_current,
                max_iterations=10000,
                error_threshold=1e-4,
            )
            if success:
                q_current = list(q_projected)
                if verbose:
                    logger.debug(
                        "✓ Projected onto '%s' (error=%.2e)", source_state, error
                    )
            elif verbose:
                logger.warning(
                    "Projection onto '%s' failed (error=%.2e), using unprojected q",
                    source_state,
                    error,
                )
        except Exception as e:
            if verbose:
                logger.warning("State projection failed: %s, continuing", e)
        return q_current

    def _plan_release_pregrasp_edge(
        self,
        gripper: str,
        released_handle: str,
        edge_21: str,
        edge_01: str,
        q_start: list,
        verbose: bool,
    ):
        """Generate q_pregrasp via the forward approach edge and plan edge_21.

        The _21 target (q_pregrasp) cannot be found with
        generateTargetConfig(edge_21): the path-fold RHS = "gripper at contact"
        but the pregrasp leaf requires "gripper at approach offset" —
        conflicting, the solver always diverges. applyStateConstraints(
        pregrasp_node, q_grasped) hits the same conflict. Fix: generate
        q_pregrasp via the FORWARD _01 edge (released_state → pregrasp), whose
        fold only encodes the OTHER grasps, leaving the pregrasp leaf free to
        position the arm at approach distance.

        SEED STRATEGY: q_start (the held config) is used as the warm-start hint
        for the initial generation. The naive seed (q_grasped = contact state)
        fails for large-clearance handles (e.g. h_RS1_FG has clearance=0.25 m,
        giving 26 cm pregrasp offset): the IK copies frame_gripper/root_joint
        from q_grasped, pinning FG at the contact position, and the
        g_ur10_tool-grasps-FG fold keeps it pinned there, making the 26 cm
        pregrasp unreachable via Newton-Raphson. Using q_start as the hint
        keeps RS1 at its current (assembly) position. Do NOT use a cached
        Phase-2 pregrasp here: it would drag RS1 from the assembly position
        back to the ground-level initial pose, producing a long-range path that
        collides with ground_demo/link_NYX_0.

        After the initial generation, edge_21 planning is retried up to
        _MAX_COLLISION_RETRIES times, regenerating a fresh (unhinted,
        randomly-seeded) pregrasp target on each plan failure — a particular
        IK solution may be kinematically valid but sit in a region the RRT
        can't reach, so resampling before falling back to the direct release
        edge is cheap insurance against an unlucky first sample.

        Args:
            gripper: Gripper name being released.
            released_handle: Handle being released (for error messages).
            edge_21: The grasped→pregrasp edge name to plan.
            edge_01: The forward released→pregrasp edge used to generate the
                pregrasp target.
            q_start: Current configuration (q_from for both edges).
            verbose: Whether to print progress.

        Returns:
            Tuple (path21, q_pregrasp, t_gen1, t_plan1, plan_err_21) where
            path21 is the planned path or None on exhaustion, q_pregrasp is the
            last generated pregrasp config, t_gen1 is the initial generation
            time, t_plan1 is the total edge_21 retry-loop time, and plan_err_21
            is the last plan exception (None on success).
        """
        import numpy as np
        import time

        if verbose:
            logger.info("Planning release edge: %s", edge_21)
            logger.info("  (pregrasp via forward edge '%s')", edge_01)

        t0 = time.time()
        ok, q_pregrasp = self.config_gen.generate_via_edge(
            edge_name=edge_01,
            q_from=q_start,
            config_label=f"q_autorelease_{gripper}_pregrasp",
            q_hint=q_start,
        )
        t_gen1 = time.time() - t0
        if not ok or q_pregrasp is None or not np.all(np.isfinite(q_pregrasp)):
            raise RuntimeError(
                f"Auto-release of '{released_handle}': failed to generate "
                f"pregrasp config via forward edge '{edge_01}'"
            )
        # Update cache so any subsequent release of the same gripper starts
        # from the current context, not a stale Phase-2 configuration.
        self._last_pregrasp_q[gripper] = q_pregrasp

        path21 = None
        t_plan1 = 0.0
        plan_err_21 = None
        t0 = time.time()
        for _attempt_21 in range(self._MAX_COLLISION_RETRIES):
            try:
                path21, _ = self.planner.plan_transition_edge(
                    edge=edge_21, q1=q_start, q2=q_pregrasp
                )
                if path21 is None:
                    raise RuntimeError(f"Planning failed for edge '{edge_21}'")
                plan_err_21 = None
                break
            except Exception as e:
                plan_err_21 = e
                path21 = None
                if _attempt_21 < self._MAX_COLLISION_RETRIES - 1:
                    if verbose:
                        logger.warning(
                            "Planning '%s' failed (attempt %d), regenerating "
                            "pregrasp config...",
                            edge_21,
                            _attempt_21 + 1,
                        )
                    _ok21, _q_new21 = self.config_gen.generate_via_edge(
                        edge_name=edge_01,
                        q_from=q_start,
                        config_label=f"q_autorelease_{gripper}_pregrasp",
                    )
                    if _ok21 and _q_new21 is not None and np.all(np.isfinite(_q_new21)):
                        q_pregrasp = _q_new21
                        self._last_pregrasp_q[gripper] = q_pregrasp
        t_plan1 = time.time() - t0
        return path21, q_pregrasp, t_gen1, t_plan1, plan_err_21

    def _generate_and_plan_edge_with_retry(
        self,
        edge_name: str,
        q_from: list,
        config_label: str,
        verbose: bool,
        gen_fail_noun: str,
        q_hint: list | None = None,
    ):
        """Generate a target config via an edge and plan it, retrying together.

        Shared shape used by both the primary pregrasp→free edge (_10) and the
        direct release fallback edge: regenerate the target config via
        ``generate_via_edge`` and plan the edge via ``plan_transition_edge``
        together, up to ``_MAX_COLLISION_RETRIES`` times before giving up.

        The two original inline blocks worded their generation-failure message
        differently ("failed to generate target config via edge '...'" for the
        primary _10 edge vs "failed to generate a free config via edge '...'"
        for the direct fallback). That difference is preserved verbatim via the
        ``gen_fail_noun`` parameter ("target config" / "a free config") rather
        than silently unified. All other messages (plan-fail, retry-print) are
        identical between the two originals and parameterized only by
        ``edge_name``.

        Does NOT raise on retry exhaustion — returns the last error so each
        caller can raise its own verbatim exhaustion message (the primary and
        direct-fallback paths have structurally different exhaustion messages
        that reference different surrounding state).

        Args:
            edge_name: Edge to generate the target config with and plan.
            q_from: Source configuration.
            config_label: Label passed to generate_via_edge.
            verbose: Whether to print progress.
            gen_fail_noun: Noun phrase used in the generation-failure message
                ("target config" or "a free config") — preserved verbatim per
                the original inline blocks.
            q_hint: Optional warm-start hint for generate_via_edge (None for the
                direct fallback, which uses random arm seeds).

        Returns:
            Tuple (path, q_result, t_gen, t_plan, error) where path is the
            planned path or None on exhaustion, q_result is the last generated
            target config, t_gen/t_plan are the total generation/planning times
            accumulated across all attempts, and error is the last
            exception/RuntimeError (None on success).
        """
        import numpy as np
        import time

        q_result = None
        path = None
        plan_err = None
        t_gen = 0.0
        t_plan = 0.0
        for _attempt in range(self._MAX_COLLISION_RETRIES):
            _t0 = time.time()
            ok, q_candidate = self.config_gen.generate_via_edge(
                edge_name=edge_name,
                q_from=q_from,
                config_label=config_label,
                q_hint=q_hint,
            )
            t_gen += time.time() - _t0
            if not ok or q_candidate is None or not np.all(np.isfinite(q_candidate)):
                plan_err = RuntimeError(
                    f"failed to generate {gen_fail_noun} via edge '{edge_name}'"
                )
                continue
            q_result = q_candidate

            _t0 = time.time()
            try:
                path, _ = self.planner.plan_transition_edge(
                    edge=edge_name, q1=q_from, q2=q_result
                )
                t_plan += time.time() - _t0
                if path is None:
                    raise RuntimeError(f"Planning failed for edge '{edge_name}'")
                plan_err = None
                break
            except Exception as e:
                t_plan += time.time() - _t0
                plan_err = e
                path = None
                if verbose and _attempt < self._MAX_COLLISION_RETRIES - 1:
                    logger.warning(
                        "Planning '%s' failed (attempt %d), regenerating "
                        "target config...",
                        edge_name,
                        _attempt + 1,
                    )
        return path, q_result, t_gen, t_plan, plan_err

    def _build_release_phase_info(
        self,
        q_start: list,
        used_direct: bool,
        path21,
        path10,
        path_direct,
        edge_21: str,
        edge_10: str,
        direct_edge: str | None,
        t_gen1: float,
        t_plan1: float,
        t_gen2: float,
        t_plan2: float,
        t_gen_direct: float,
        t_plan_direct: float,
    ) -> dict:
        """Build the phase_info dict for a release sub-phase.

        Consolidates the two near-identical end-of-method dict literals the
        original _plan_release_subphase carried (one for the direct-fallback
        path, one for the waypoint path), differing only in which
        paths/edges/edge_stats and which timing sums they contain.

        Args:
            q_start: Final configuration of the release (final_config).
            used_direct: True if the direct fallback edge was used.
            path21: Planned edge_21 path (None if direct fallback was used).
            path10: Planned edge_10 path (None if direct fallback was used).
            path_direct: Planned direct-edge path (None unless used_direct).
            edge_21: The grasped→pregrasp edge name.
            edge_10: The pregrasp→free edge name.
            direct_edge: The direct release edge name (only when used_direct).
            t_gen1/t_plan1: edge_21 generation/planning times.
            t_gen2/t_plan2: edge_10 generation/planning times.
            t_gen_direct/t_plan_direct: direct-edge generation/planning times.

        Returns:
            phase_info dict with keys paths, edges, state_after, final_config,
            edge_stats, phase_time, phase_gen_time, phase_plan_time.
        """
        if used_direct:
            phase_info = {
                "paths": [path_direct],
                "edges": [direct_edge],
                "state_after": self.grasp_tracker.get_current_state_name(),
                "final_config": q_start,
                "edge_stats": [
                    {
                        "edge_idx": 0,
                        "edge_name": direct_edge,
                        "success": True,
                        "gen_time": t_gen_direct,
                        "plan_time": t_plan_direct,
                        "total_time": t_gen_direct + t_plan_direct,
                    },
                ],
                "phase_time": t_gen1 + t_plan1 + t_gen_direct + t_plan_direct,
                "phase_gen_time": t_gen1 + t_gen_direct,
                "phase_plan_time": t_plan1 + t_plan_direct,
            }
        else:
            phase_info = {
                "paths": [p for p in [path21, path10] if p is not None],
                "edges": [edge_21, edge_10],
                "state_after": self.grasp_tracker.get_current_state_name(),
                "final_config": q_start,
                "edge_stats": [
                    {
                        "edge_idx": 0,
                        "edge_name": edge_21,
                        "success": True,
                        "gen_time": t_gen1,
                        "plan_time": t_plan1,
                        "total_time": t_gen1 + t_plan1,
                    },
                    {
                        "edge_idx": 1,
                        "edge_name": edge_10,
                        "success": True,
                        "gen_time": t_gen2,
                        "plan_time": t_plan2,
                        "total_time": t_gen2 + t_plan2,
                    },
                ],
                "phase_time": t_gen1 + t_plan1 + t_gen2 + t_plan2,
                "phase_gen_time": t_gen1 + t_gen2,
                "phase_plan_time": t_plan1 + t_plan2,
            }
        return phase_info

    def _auto_save_phase_paths(
        self,
        phase_idx: int,
        phase_paths: list[Any],
        edge_names: list[str],
        verbose: bool = True,
        phase_geometric_paths: list[Any] | None = None,
    ) -> list[str]:
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
        paths_to_save = (
            phase_geometric_paths if phase_geometric_paths is not None else phase_paths
        )

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
            base_filename = (
                f"phase_{phase_idx + 1:02d}_edge_{edge_idx + 1:02d}_{edge_name_safe}"
            )

            # Try to save as portable JSON waypoints (always works)
            if hasattr(self.planner, "save_path_as_waypoints"):
                json_filepath = os.path.join(
                    self.auto_save_dir, base_filename + ".json"
                )
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
                        logger.info("✓ Auto-saved (portable): %s.json", base_filename)
                except Exception as e:
                    if verbose:
                        logger.warning("Failed to save JSON waypoints: %s", e)

            # Also try to save native .path format (may fail for graph paths, but worth trying)
            if hasattr(self.planner, "save_path_vector"):
                path_filepath = os.path.join(
                    self.auto_save_dir, base_filename + ".path"
                )
                try:
                    self.planner.save_path_vector(path, path_filepath)
                    saved_files.append(path_filepath)
                    if verbose:
                        logger.info("✓ Auto-saved (native): %s.path", base_filename)
                except Exception as e:
                    # Native format may fail, but JSON should have succeeded
                    if verbose:
                        error_msg = str(e)
                        if "time parameterization" in error_msg.lower():
                            logger.warning(
                                "Native format skipped: Time-parameterized paths "
                                "not serializable"
                            )
                        elif (
                            "graph" in error_msg.lower() or "edge" in error_msg.lower()
                        ):
                            logger.warning(
                                "Native format skipped: Graph-constrained paths "
                                "(JSON format works)"
                            )
                        else:
                            logger.warning("Native format failed: %s", e)

        self.saved_path_files.extend(saved_files)
        return saved_files

    def get_saved_path_files(self) -> list[str]:
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

    def load_saved_paths(self, verbose: bool = True) -> list[int]:
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
                logger.warning("No auto_save_dir configured")
            return []

        if hasattr(self.planner, "load_paths_from_directory"):
            return self.planner.load_paths_from_directory(
                self.auto_save_dir,
                pattern="phase_*.path",
                sort=True,
            )
        else:
            if verbose:
                logger.warning("Backend does not support path loading")
            return []

    def compute_phase_locked_joints(
        self,
        active_gripper: str,
        mode: str = "auto",
        manual_arms: list[str] | None = None,
        verbose: bool = True,
        handle: str | None = None,
    ) -> list[str]:
        """Compute which arms should be frozen for current phase.

        Args:
            active_gripper: Gripper performing grasp in this phase
            mode: "auto" (freeze all except active),
                  "manual" (use manual_arms), "none" (no locked joints)
            manual_arms: List of arm keywords to freeze (for manual mode)
            verbose: Print progress messages
            handle: Target handle string (e.g. ``"RS1/h_RS1_WB"``).  When
                provided together with ``self.grasp_tracker``, the
                held-object chain starting at the target's owning object
                is walked: any arm that currently holds the target (directly
                or transitively) is kept unfrozen.  This is required when
                the target object sits inside another held object (e.g.
                RS1 is inside ``frame_gripper`` held by UR10) — otherwise
                the pregrasp IK cannot reach the target.

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

        # Auto mode: freeze all arms except the active gripper's arm AND
        # any arm in the held-object chain that ultimately carries the
        # target handle's owning object.

        # Find which arm the active gripper belongs to.
        # Exact-match first (config stores full gripper frame names),
        # then substring fallback for any legacy prefix-style mappings.
        active_arm = self.GRIPPER_TO_ARM_MAP.get(active_gripper)
        if active_arm is None:
            gripper_lower = active_gripper.lower()
            for (
                gripper_pattern,
                arm_keyword,
            ) in self.GRIPPER_TO_ARM_MAP.items():
                if gripper_pattern.lower() in gripper_lower:
                    active_arm = arm_keyword
                    break

        unfrozen_arms: set[str] = {active_arm} if active_arm is not None else set()

        if verbose:
            logger.debug(
                "Active gripper '%s' uses arm '%s'", active_gripper, active_arm
            )

        # Walk the held-object chain: if the target is held (directly or
        # transitively) by another arm, that arm must stay free so the
        # active arm can bring the target into reach.  Skipped when
        # ``handle`` is not provided or the grasp tracker is not wired up
        # (e.g. interactive-pickers unit tests) — in that case the
        # behaviour is identical to the original (only the active arm
        # is unfrozen).
        if handle is not None and getattr(self, "grasp_tracker", None) is not None:
            target_obj: str | None = handle.split("/")[0]
            visited: set[str] = set()
            current_grasps = self.grasp_tracker.current_grasps
            while target_obj and target_obj not in visited:
                visited.add(target_obj)
                holder_gripper: str | None = None
                for g, h in current_grasps.items():
                    if h is None or g == active_gripper:
                        continue
                    if h.split("/")[0] == target_obj:
                        holder_gripper = g
                        break
                if holder_gripper is None:
                    break
                holding_arm = self._get_arm_for_gripper(holder_gripper)
                if holding_arm:
                    unfrozen_arms.add(holding_arm)
                    if verbose:
                        logger.debug(
                            "Chain trace: '%s' (arm '%s') holds object '%s' — "
                            "keeping '%s' unfrozen",
                            holder_gripper,
                            holding_arm,
                            target_obj,
                            holding_arm,
                        )
                # Walk up: the holder gripper itself is mounted on some
                # parent object (e.g. 'g_FG_part' → 'frame_gripper',
                # 'g_arm_tool' → the robot itself → end of chain).
                target_obj = holder_gripper.split("/")[0]

        # Freeze all arms not in the unfrozen set.
        frozen_arms: list[str] = [
            arm_keyword
            for arm_keyword in self.ALL_ARM_KEYWORDS
            if arm_keyword not in unfrozen_arms
        ]

        return frozen_arms

    def _get_arm_for_gripper(self, gripper_name: str) -> str | None:
        """Return the arm keyword for a gripper name, or None."""
        arm = self.GRIPPER_TO_ARM_MAP.get(gripper_name)
        if arm is not None:
            return arm
        gl = gripper_name.lower()
        for pattern, arm in self.GRIPPER_TO_ARM_MAP.items():
            if pattern.lower() in gl:
                return arm
        return None

    # Arm keyword → JOINT_GROUPS key mapping (case-insensitive substring)
    _ARM_KEYWORD_TO_GROUP = {
        "ur10": "UR10",
        "vispa_": "VISPA_ARM",
        "vispa2": "VISPA_BASE",
    }

    def _get_active_joints_for_unfrozen_arms(self, frozen_arms: list[str]) -> list[str]:
        """Get joint names for all unfrozen arms.

        Maps arm keywords (e.g. "ur10", "vispa_") to their joint names
        via task_config.JOINT_GROUPS.  Used to configure TOPPRA's
        selectJoints() per phase.

        Args:
            frozen_arms: List of frozen arm keywords

        Returns:
            List of joint names for unfrozen arms
        """
        joint_groups = getattr(self.task_config, "JOINT_GROUPS", {})
        if not joint_groups:
            return []

        active_joints: list[str] = []
        for arm_keyword in self.ALL_ARM_KEYWORDS:
            if arm_keyword in frozen_arms:
                continue
            group_key = self._ARM_KEYWORD_TO_GROUP.get(arm_keyword)
            if group_key and group_key in joint_groups:
                active_joints.extend(joint_groups[group_key])
        return active_joints

    def _dump_phase_checkpoint(
        self,
        phase_idx: int,
        gripper: str,
        handle: str | None,
        q_current: Sequence[float],
        verbose: bool,
    ) -> None:
        """Dump (q_current, held grasps) entering a phase, for repro_phase_range.py.

        No-op unless AGIMUS_CHECKPOINT_DIR is set.
        """
        checkpoint_dir = os.environ.get("AGIMUS_CHECKPOINT_DIR")
        if not checkpoint_dir:
            return
        try:
            os.makedirs(checkpoint_dir, exist_ok=True)
            final_path = os.path.join(checkpoint_dir, f"phase_{phase_idx:02d}.json")
            tmp_path = final_path + ".tmp"
            # Write-then-rename: a direct open(path, "w") truncates before
            # writing, so a process killed mid-write (e.g. an external
            # supervisor bounding wall time per attempt) leaves a 0-byte
            # file with the prior valid content unrecoverable. rename() is
            # atomic on POSIX, so readers never see a half-written file.
            with open(tmp_path, "w") as f:
                json.dump(
                    {
                        "phase_idx": phase_idx,
                        "gripper": gripper,
                        "handle": handle,
                        "q_current": [float(x) for x in q_current],
                        "held_grasps": dict(self.grasp_tracker.current_grasps),
                    },
                    f,
                )
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp_path, final_path)
        except Exception as e:
            if verbose:
                logger.warning("Checkpoint dump failed: %s", e)

    def _release_frozen_arms(
        self,
        gripper: str,
        released_handle: str,
        frozen_arms_mode: str,
        per_phase_frozen_arms: dict[int, list[str]] | None,
        phase_idx: int,
    ) -> list[str]:
        """Which arms to freeze while ``gripper`` releases ``released_handle``.

        Two rules, both learned the hard way on RS3's FG release (2026-08-14,
        ~30k consecutive solver failures, 0 ever reaching collision checking):

        1. NEVER freeze an arm that holds the object being released.  vispa2
           holds ``RS3/h_RS3_WB`` while frame_gripper releases
           ``RS3/h_RS3_FG``; freezing vispa2 pins the workbench carrying RS3,
           so the ~0.25 m FG retreat has to come entirely from UR10 and no
           solution exists.  Measured from the failing checkpoint:
           0/200 target draws converge with vispa2 frozen, 107/200 without
           (and the warm start then converges on the first attempt).
           ``compute_phase_locked_joints`` already implements this via its
           ``handle=`` chain walk -- ``_plan_auto_release_if_needed`` passed
           it, ``_plan_release_entry_phase`` did not, which is the whole bug.
        2. Honour an explicit per-phase override in "manual" mode.  Callers
           escalate by dropping an arm from ``per_phase_frozen_arms`` after
           repeated failures (see run_block_nonstop._maybe_loosen); both
           release paths previously recomputed the set from scratch and
           ignored the override, so those escalations silently did nothing.
        """
        # The chain walk's effect, isolated: arms the walk drops precisely
        # because they carry the released object.  Derived as a diff rather
        # than re-implemented so the transitive cases (RS3 inside
        # frame_gripper inside UR10) stay in one tested place.
        holders = set(
            self.compute_phase_locked_joints(gripper, "auto", verbose=False)
        ) - set(
            self.compute_phase_locked_joints(
                gripper, "auto", handle=released_handle, verbose=False
            )
        )

        if frozen_arms_mode == "manual" and per_phase_frozen_arms is not None:
            requested = per_phase_frozen_arms.get(phase_idx, [])
        else:
            requested = self.compute_phase_locked_joints(
                gripper, "auto", handle=released_handle
            )

        # Rule 1 outranks the caller: a manual list naming an arm that holds
        # the released object is asking for an unsatisfiable pregrasp, so
        # drop it rather than honouring it.  This is what block B's
        # {0: ["vispa_", "vispa2"]} does during RS3's FG release.
        kept = [a for a in requested if a not in holders]
        if len(kept) != len(requested):
            logger.warning(
                "Not freezing %s during release of '%s': holds the released "
                "object, and freezing it makes the pregrasp unreachable",
                ", ".join(sorted(set(requested) - set(kept))),
                released_handle,
            )
        return kept

    def _plan_release_entry_phase(
        self,
        phase_idx: int,
        gripper: str,
        currently_held: str | None,
        q_current: list[float],
        frozen_arms_mode: str,
        verbose: bool,
        per_phase_frozen_arms: dict[int, list[str]] | None = None,
    ) -> list[float]:
        """Handle an explicit release entry ``(gripper, None)`` in the sequence.

        Shared by ``plan_sequence()`` and ``resume_sequence()`` — extracted
        verbatim from both (Phase 1, Step 1.1 of the codebase refactor).
        Appends to ``self.phase_results`` (no-op skip, success, or
        failure-then-raise) and updates ``self.last_failure_info`` on
        failure, exactly as the original inline blocks did.

        Args:
            frozen_arms_mode: Already resolved to a concrete string by the
                caller (``resume_sequence`` maps ``None`` -> ``"global"``
                before calling; ``plan_sequence``'s parameter is never
                ``None``).

        Returns:
            The (possibly updated) ``q_current``.

        Raises:
            Exception: whatever ``_plan_release_subphase`` raises, re-raised
                after recording partial phase/failure info.
        """
        if currently_held is None:
            # No-op: gripper is already free
            if verbose:
                logger.info("'%s' is already free — skipping", gripper)
            self.phase_results.append(
                {
                    "phase": phase_idx + 1,
                    "gripper": gripper,
                    "handle": None,
                    "edges": [],
                    "paths": [],
                    "complete": True,
                    "skipped": True,
                }
            )
            return q_current

        if verbose:
            logger.info("[Release] '%s' releasing '%s'", gripper, currently_held)
        # Compute frozen arms for the release sub-phase
        release_constraints = None
        if frozen_arms_mode == "global":
            release_constraints = self.graph_constraints
        elif frozen_arms_mode != "none":
            release_frozen = self._release_frozen_arms(
                gripper,
                currently_held,
                frozen_arms_mode,
                per_phase_frozen_arms,
                phase_idx,
            )
            if release_frozen:
                from long_tamp.planning.constraints import (
                    ConstraintBuilder,
                )

                cn, _ = ConstraintBuilder.create_locked_joint_constraints(
                    self.graph_builder.ps,
                    self.graph_builder.robot,
                    q_current,
                    release_frozen,
                    backend=self.graph_builder.backend,
                )
                if cn:
                    release_constraints = cn
        try:
            q_current, _release_info = self._plan_release_subphase(
                gripper=gripper,
                q_current=q_current,
                phase_graph_constraints=release_constraints,
                verbose=verbose,
            )
        except Exception as e:
            self.phase_results.append(
                {
                    "phase": phase_idx + 1,
                    "gripper": gripper,
                    "handle": None,
                    "released": currently_held,
                    "edges": [],
                    "paths": [],
                    "complete": False,
                    "error_message": f"Release failed: {e}",
                    # get_resumable_state() reads THIS key (not "q_current")
                    # to recover the resume point -- without it, resume
                    # falls back to None, which get_resumable_state() then
                    # hands to resume_sequence() as the entire remaining
                    # sequence's starting config. For a single-phase
                    # release block that "succeeds" as a same-state no-op
                    # (nothing to actually replan), that None then becomes
                    # the block's reported final_config, silently
                    # corrupting every held grasp downstream instead of
                    # failing loudly. Every other failure path in this file
                    # that builds a phase_results entry sets this key;
                    # this one didn't.
                    "last_q_start": q_current,
                }
            )
            self.last_failure_info = {
                "phase_idx": phase_idx,
                "edge_idx": 0,
                "edge_name": "release_subphase",
                "q_current": q_current,
                "error": f"Release failed: {e}",
                "completed_phases": len(
                    [p for p in self.phase_results if p.get("complete", False)]
                ),
                "completed_edges_in_phase": 0,
            }
            if verbose:
                logger.warning("Release failed, partial result stored for resume")
            raise
        # Record phase result (with full path/timing tracking)
        self.phase_results.append(
            {
                "phase": phase_idx + 1,
                "gripper": gripper,
                "handle": None,
                "released": currently_held,
                **_release_info,
                "complete": True,
            }
        )
        return q_current

    def _plan_auto_release_if_needed(
        self,
        phase_idx: int,
        gripper: str,
        handle: str,
        currently_held: str | None,
        q_current: list[float],
        frozen_arms_mode: str,
        verbose: bool,
        per_phase_frozen_arms: dict[int, list[str]] | None = None,
    ) -> list[float]:
        """Auto-release ``gripper``'s current object before grasping a new one.

        No-op (returns ``q_current`` unchanged) unless ``gripper`` currently
        holds a *different* object than ``handle``. Shared by
        ``plan_sequence()`` and ``resume_sequence()`` -- extracted verbatim
        from both (Phase 1, Step 1.2 of the codebase refactor).

        Appends to ``self.phase_results`` and updates
        ``self.last_failure_info`` on failure, exactly as the original
        inline blocks did.

        Args:
            frozen_arms_mode: Already resolved to a concrete string by the
                caller (``resume_sequence`` maps ``None`` -> ``"global"``
                before calling; ``plan_sequence``'s parameter is never
                ``None``).

        Returns:
            The (possibly updated) ``q_current``.

        Raises:
            Exception: whatever ``_plan_release_subphase`` raises, re-raised
                after recording partial phase/failure info.
        """
        if currently_held is None or currently_held == handle:
            return q_current

        if verbose:
            logger.info(
                "[Auto-release] '%s' holds '%s', inserting release before "
                "grasping '%s'",
                gripper,
                currently_held,
                handle,
            )
        # Compute frozen arms for the release sub-phase.
        # Must NOT freeze arms that hold handles on the same
        # object being released — otherwise the object becomes
        # immovable and the pregrasp IK is unreachable.
        release_constraints = None
        if frozen_arms_mode == "global":
            release_constraints = self.graph_constraints
        elif frozen_arms_mode != "none":
            # Generalised chain walk: any arm that currently
            # holds the released object (directly or transitively)
            # is kept unfrozen by compute_phase_locked_joints.
            # An explicit "manual" per-phase override wins over the walk,
            # so a caller's escalation (dropping an arm after repeated
            # failures) actually reaches the phase graph.
            release_frozen = self._release_frozen_arms(
                gripper,
                currently_held,
                frozen_arms_mode,
                per_phase_frozen_arms,
                phase_idx,
            )
            # Defensive fallback: if the chain walk did not
            # unfreeze the arm that directly holds
            # ``currently_held``, log a warning and apply the
            # original 1-link approximation so we never get a
            # worse result than the pre-fix behaviour.
            released_obj = currently_held.split("/")[0]
            direct_holder_arm: str | None = None
            for g, h in self.grasp_tracker.current_grasps.items():
                if g == gripper or h is None:
                    continue
                if h.split("/")[0] == released_obj:
                    arm = self._get_arm_for_gripper(g)
                    if arm:
                        direct_holder_arm = arm
                        break
            if direct_holder_arm and direct_holder_arm in release_frozen:
                if verbose:
                    logger.warning(
                        "\u26a0 compute_phase_locked_joints did not "
                        "unfreeze direct holder arm '%s' of '%s'; applying "
                        "1-link fallback.",
                        direct_holder_arm,
                        currently_held,
                    )
                release_frozen.remove(direct_holder_arm)
            if release_frozen:
                from long_tamp.planning.constraints import (
                    ConstraintBuilder,
                )

                cn, _ = ConstraintBuilder.create_locked_joint_constraints(
                    self.graph_builder.ps,
                    self.graph_builder.robot,
                    q_current,
                    release_frozen,
                    backend=self.graph_builder.backend,
                )
                if cn:
                    release_constraints = cn
        try:
            q_current, _ = self._plan_release_subphase(
                gripper=gripper,
                q_current=q_current,
                phase_graph_constraints=release_constraints,
                verbose=verbose,
            )
        except Exception as e:
            self.phase_results.append(
                {
                    "phase": phase_idx + 1,
                    "gripper": gripper,
                    "handle": handle,
                    "released": currently_held,
                    "edges": [],
                    "paths": [],
                    "complete": False,
                    "error_message": f"Auto-release failed: {e}",
                    # See the identical fix in _plan_release_entry_phase's
                    # except-block above: get_resumable_state() needs this
                    # key to recover a valid resume point.
                    "last_q_start": q_current,
                }
            )
            self.last_failure_info = {
                "phase_idx": phase_idx,
                "edge_idx": 0,
                "edge_name": "auto_release_subphase",
                "q_current": q_current,
                "error": f"Auto-release failed: {e}",
                "completed_phases": len(
                    [p for p in self.phase_results if p.get("complete", False)]
                ),
                "completed_edges_in_phase": 0,
            }
            if verbose:
                logger.warning(
                    "\u26a0 Auto-release failed, partial result stored for resume"
                )
            raise
        return q_current

    def _build_phase_graph_and_constraints(
        self,
        phase_idx: int,
        gripper: str,
        handle: str | None,
        q_current: list[float],
        frozen_arms_mode: str,
        per_phase_frozen_arms: dict[int, list[str]] | None,
        q_scene_init: list[float] | None,
        verbose: bool,
        emit_logs: bool,
    ) -> None:
        """Build the phase-local constraint graph and locked-joint constraints.

        Shared by ``plan_sequence()`` and ``resume_sequence()`` -- extracted
        verbatim from both (Phase 1, Step 1.3 of the codebase refactor).
        Mutates ``self.graph_builder``'s graph, ``self.planner.graph``,
        ``self.config_gen``, and ``self.grasp_tracker``'s phase indices as a
        side effect; no return value, matching the original inline code.

        One deliberate simplification versus the original: the ConfigGenerator
        init-vs-update branch order was ``if is None: init / elif hasattr: update``
        in plan_sequence()'s inline code and the reverse order in
        resume_sequence()'s -- both orders are equivalent since the two
        conditions are mutually exclusive. Unified on plan_sequence()'s
        order; not a behavior change.

        Args:
            frozen_arms_mode: Already resolved to a concrete string by the
                caller (``resume_sequence`` maps ``None`` -> ``"global"``
                before calling; ``plan_sequence``'s parameter is never
                ``None``).
            q_scene_init: Value to pass through as ``build_phase_graph``'s
                ``q_init_original``. ``plan_sequence`` always has
                ``self._q_scene_init`` set earlier in the same call;
                ``resume_sequence`` may not, hence
                ``getattr(self, "_q_scene_init", None)`` at that call site.
            emit_logs: Whether to print the verbose confirmation lines
                around the graph/ConfigGenerator updates (``"✓ Updated
                planner graph reference"`` etc.) when ``verbose=True``.
                ``plan_sequence`` has always printed these;
                ``resume_sequence`` never has -- preserved as a
                deliberate, purely-cosmetic asymmetry (not resume's
                RunLogger visibility, which the ``phase_start`` event
                below no longer depends on this flag for -- see the
                codebase refactor plan's logging-asymmetry section,
                fixed 2026-08-09).

        Raises:
            RuntimeError: if graph building fails, wrapping the original
                exception (same as both original inline blocks).
        """
        held_grasps = {
            g: h for g, h in self.grasp_tracker.current_grasps.items() if h is not None
        }
        logger.debug("Held grasps: %s", held_grasps)

        # Always emitted (regardless of `emit_logs`/resume status) so that
        # RunLogger-based analysis/replay can see resumed phases too --
        # this was the logging-asymmetry bug, fixed 2026-08-09.
        if self.run_logger is not None:
            try:
                self.run_logger.log(
                    "phase_start",
                    phase=phase_idx + 1,
                    gripper=gripper,
                    handle=handle,
                    q_start=list(q_current),
                    held_grasps={g: str(h) for g, h in held_grasps.items()},
                )
            except Exception:
                pass

        # Compute phase-specific locked joint constraints
        phase_graph_constraints = None

        if frozen_arms_mode == "global":
            # Use global constraints from task.setup()
            phase_graph_constraints = self.graph_constraints
            if verbose and phase_graph_constraints:
                logger.debug("Using global locked joint constraints")
        elif frozen_arms_mode != "none":
            # Determine which arms to freeze for this phase
            if frozen_arms_mode == "interactive":
                # Interactive mode: use callback if available
                if self.interactive_arm_selector_callback:
                    try:
                        frozen_arms = self.interactive_arm_selector_callback(
                            phase_idx, gripper, self.ALL_ARM_KEYWORDS
                        )
                    except Exception as e:
                        # Fallback to auto if callback fails
                        if verbose:
                            logger.warning(
                                "\u26a0 Interactive selection failed: %s, "
                                "using auto mode",
                                e,
                            )
                        frozen_arms = self.compute_phase_locked_joints(
                            gripper, "auto", handle=handle
                        )
                else:
                    # No callback set, fall back to auto
                    if verbose:
                        logger.warning(
                            "\u26a0 Interactive mode requested but no "
                            "callback set, using auto mode"
                        )
                    frozen_arms = self.compute_phase_locked_joints(
                        gripper, "auto", handle=handle
                    )
            elif frozen_arms_mode == "manual" and per_phase_frozen_arms:
                # Manual mode with explicit specification
                frozen_arms = per_phase_frozen_arms.get(phase_idx, [])
            else:
                # Auto mode: freeze all except active gripper's arm
                # (plus any arm in the kinematic chain that carries
                # the target handle's owning object).
                frozen_arms = self.compute_phase_locked_joints(
                    gripper, "auto", handle=handle
                )

            # Create locked joint constraints for this phase
            if frozen_arms:
                if verbose:
                    logger.debug("Freezing arms: %s", frozen_arms)

                from long_tamp.planning.constraints import (
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
                        logger.debug(
                            "\u2713 Created %d locked joint constraints: %s",
                            len(joint_names),
                            joint_list,
                        )
                        logger.debug("Constraint names: %s", constraint_names)

            # Dynamically set TOPPRA active joints from unfrozen arms
            if hasattr(self.planner, "set_toppra_active_joints"):
                active_joints = self._get_active_joints_for_unfrozen_arms(frozen_arms)
                if active_joints:
                    self.planner.set_toppra_active_joints(active_joints)
                    if verbose:
                        logger.debug(
                            "TOPPRA active joints: %d joints from unfrozen arms",
                            len(active_joints),
                        )

        try:
            self.graph_builder.build_phase_graph(
                config=self.task_config,
                held_grasps=held_grasps,
                next_grasp=(gripper, handle),
                graph_constraints=phase_graph_constraints,
                q_init=q_current,
                q_init_original=q_scene_init,
            )

            # Update planner backend's graph reference after rebuild
            # The graph_builder has the new graph, but the planner backend
            # still has the old reference and needs to be updated
            new_graph = self.graph_builder.get_graph()
            if hasattr(self.planner, "graph"):
                self.planner.graph = new_graph
                if emit_logs and verbose:
                    logger.debug("\u2713 Updated planner graph reference")
                    logger.debug("Graph object: %s", type(new_graph).__name__)
                    if hasattr(new_graph, "edges"):
                        logger.debug("Graph has %d edges", len(new_graph.edges))

            # Update ConfigGenerator's graph reference (or initialize it)
            # ConfigGenerator needs current graph for edge-based config generation
            if self.config_gen is None:
                # First phase: initialize ConfigGenerator with phase graph
                from long_tamp.planning import ConfigGenerator

                self.config_gen = ConfigGenerator(
                    self.graph_builder.robot,
                    new_graph,
                    self.planner,
                    self.graph_builder.ps,
                    backend=self.backend,
                )
                if emit_logs and verbose:
                    logger.debug("\u2713 Initialized ConfigGenerator with phase graph")
            elif hasattr(self.config_gen, "update_graph"):
                # Subsequent phases: update graph reference
                self.config_gen.update_graph(new_graph)
                if emit_logs and verbose:
                    logger.debug("\u2713 Updated ConfigGenerator graph reference")

        except Exception as e:
            raise RuntimeError(
                f"Phase {phase_idx + 1}: Failed to build graph: {e}"
            ) from e

        # Sync tracker indices with phase-local factory ordering
        if hasattr(self.graph_builder, "_phase_grippers"):
            self.grasp_tracker.set_phase_indices(
                self.graph_builder._phase_grippers,
                self.graph_builder._phase_handles,
            )

    def _compute_and_project_edge_sequence(
        self,
        phase_idx: int,
        gripper: str,
        handle: str | None,
        q_current: list[float],
        verbose: bool,
        emit_logs: bool,
    ) -> tuple[list[str], list[float]]:
        """Compute the phase's edge sequence, project q_current onto its source state.

        Shared by ``plan_sequence()`` and ``resume_sequence()`` -- extracted
        verbatim from both (Phase 1, Step 1.4 of the codebase refactor).
        On projection failure, appends a partial phase result and updates
        ``self.last_failure_info``, then re-raises -- same as both original
        inline blocks.

        Args:
            emit_logs: Gates two pairs of verbose prints around the
                projection step ("Projecting q_current onto state..." /
                "q_current (first 5)..." and "✓ Projected q_current..." /
                "q_projected (first 5)...") that ``plan_sequence()`` always
                printed when ``verbose=True`` and ``resume_sequence()``
                never printed regardless of its own ``verbose`` flag --
                preserved as a pre-existing asymmetry, not unified.

        Returns:
            ``(edge_sequence, q_current)`` -- ``q_current`` is the
            projected configuration.

        Raises:
            RuntimeError: if computing the edge sequence or projecting
                fails, same as both original inline blocks.
        """
        try:
            edge_sequence = self.grasp_tracker.get_grasp_edge_sequence(gripper, handle)
        except Exception as e:
            raise RuntimeError(
                f"Phase {phase_idx + 1}: " f"Failed to compute edge sequence: {e}"
            ) from e

        if verbose:
            logger.debug("Edge sequence: %s", edge_sequence)

        # Project current config onto the phase graph's source state
        # The phase graph has different constraints, so q_current
        # might not satisfy them. We need to project onto the
        # edge's source state.
        source_state = self.grasp_tracker.get_current_state_name()
        try:
            if emit_logs and verbose:
                logger.debug("Projecting q_current onto state: %s", source_state)
                logger.debug("q_current (first 5): %s", q_current[:5])

            success, q_projected, error = self.graph_builder.apply_state_constraints(
                state_name=source_state,
                q=q_current,
                max_iterations=10000,
                error_threshold=1e-4,
            )

            if not success:
                raise RuntimeError(
                    f"Failed to project q_current onto state "
                    f"'{source_state}' (error={error:.6f})"
                )

            q_current = list(q_projected)

            if emit_logs and verbose:
                logger.debug("✓ Projected q_current (error=%.6e)", error)
                logger.debug("q_projected (first 5): %s", q_current[:5])

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
                "failed_edge_name": (edge_sequence[0] if edge_sequence else None),
                "last_q_start": q_current,
                "failed_q_target": None,
                "error_message": f"State projection failed: {e}",
            }
            self.phase_results.append(partial_phase_result)

            # Store failure info
            self.last_failure_info = {
                "phase_idx": phase_idx,
                "edge_idx": 0,
                "edge_name": (edge_sequence[0] if edge_sequence else "unknown"),
                "q_current": q_current,
                "error": f"State projection failed: {e}",
                "completed_phases": len(
                    [p for p in self.phase_results if p.get("complete", False)]
                ),
                "completed_edges_in_phase": 0,
            }

            if verbose:
                logger.warning(
                    "Stored partial phase result: projection failed at phase start"
                )

            raise RuntimeError(
                f"Phase {phase_idx + 1}: State projection failed: {e}"
            ) from e

        return edge_sequence, q_current

    @staticmethod
    def _edge_hints_for_phase(
        phase_hint: Any, n_edges: int
    ) -> list[list[float] | None]:
        """Expand one ``phase_q_hints`` entry into a per-edge hint list.

        Accepts either shape:

        - A **chain** -- one config per edge of the phase's edge sequence,
          as returned by ``find_feasible_phase_target()``. Every edge gets
          warm-started, so an uninterrupted run reproduces the probed
          candidate *exactly*: edge 0 re-solves from the same ``q_from``
          and the same seed, its planned path ends on that same config,
          and edge 1 then solves from precisely the ``q_from`` the probe
          used.
        - A **single config** (legacy) -- applied to the phase's last edge
          only, leaving earlier waypoint edges unhinted.

        The legacy shape does not actually pin the phase's committed
        config, which is why the chain exists.  ``generate_via_edge()``
        takes an edge's constraint RHS from its predecessor's end config,
        and the free DOFs it doesn't constrain pass straight through from
        there.  With the pregrasp edge still drawn at random -- and redrawn
        again by every collision-retry -- the last edge solves from a
        ``q_from`` the probe never saw, so its result drifts off the
        validated candidate even though the seed was right.  Live: RS5's
        pregrasp edge was redrawn 6x (5 planning failures), moving the free
        ur10 arm and with it RS5 itself, and the CON0 grasp the lookahead
        had just verified as reachable then failed 878/878 target draws at
        solver convergence.

        A chain whose length doesn't match ``n_edges`` (e.g. carried across
        a phase whose edge sequence changed shape) degrades to the legacy
        single-config behavior rather than mis-aligning configs onto edges
        they were never solved for.
        """
        # Length rather than truthiness: a hint may arrive as a numpy array,
        # whose truth value is ambiguous for more than one element.
        if phase_hint is None or len(phase_hint) == 0:
            return [None] * n_edges

        # A chain's entries are configs (sized); a single config's are scalars.
        is_chain = hasattr(phase_hint[0], "__len__")

        if is_chain and len(phase_hint) == n_edges:
            return [list(q) for q in phase_hint]

        terminal = list(phase_hint[-1]) if is_chain else list(phase_hint)
        return [None] * (n_edges - 1) + [terminal]

    def _plan_phase_edges(
        self,
        phase_idx: int,
        gripper: str,
        handle: str | None,
        edge_sequence: list[str],
        q_current: list[float],
        skip_phases: set[int] | None,
        start_edge_idx: int,
        is_resume: bool,
        verbose: bool,
        loop_start_time: float | None = None,
        phase_q_hints: dict[int, list[list[float]] | list[float]] | None = None,
    ) -> dict[str, Any]:
        """Plan every edge in the phase's edge sequence, with collision-retry.

        Shared by ``plan_sequence()`` and ``resume_sequence()`` -- extracted
        verbatim from both (Phase 1, Step 1.5 of the codebase refactor),
        the largest single block in either function.

        Handles: user-interrupt (stop-request) checkpointing, per-edge
        target generation via ``ConfigGenerator``, the ``skip_phases``
        short-circuit, and the collision-retry loop around
        ``plan_transition_edge`` (up to ``self._MAX_COLLISION_RETRIES``
        attempts, regenerating the target config between attempts).
        Appends a partial phase result and updates ``self.last_failure_info``
        on any failure (interrupt, generation failure, or planning failure)
        before raising -- same as both original inline blocks.

        Args:
            start_edge_idx: Which edge in ``edge_sequence`` to start
                planning from. ``plan_sequence()`` always passes ``0``;
                ``resume_sequence()`` passes its own computed resume point
                (resume-specific bookkeeping that stays in that caller).
            is_resume: Selects between the two call sites' pre-existing,
                genuinely different behaviors in this block -- not just a
                verbosity toggle, preserved exactly rather than unified:
                  - ``edge_stat["attempt"]``: hardcoded ``1`` when
                    ``False``; looked up from ``self.edge_stats`` (prior
                    attempt + 1) when ``True``, with an
                    ``"is_resume": True`` key added to the dict.
                  - Target generation: ``False`` additionally checks the
                    generated config is finite (NaN/inf guard), prints a
                    verbose debug dump of ``q_target``, and attempts to
                    visualize it before planning; ``True`` does none of
                    that (plain ok/None check only).
                  - The per-attempt "Planning: q_start -> q_target" print
                    on the first collision-retry attempt: printed when
                    ``False``; skipped when ``True`` (that case's
                    "Planning waypoint edge..." line above already
                    includes an "(attempt #N)" suffix).
                  - The console message on planning failure: "Stored
                    partial phase result: N edges completed" when
                    ``False``; "Failed after Xs (attempt #N)" when
                    ``True``.
                RunLogger ``"edge_start"``/``"edge_end"`` events and the
                ``"run_end"`` event on user interrupt are **not** gated by
                ``is_resume`` -- both callers emit them (fixed 2026-08-09;
                previously ``resume_sequence()``'s phases/edges were
                invisible to RunLogger-based analysis/replay, see the
                codebase refactor plan's logging-asymmetry section).
            loop_start_time: The caller's loop-start timestamp, used for
                the ``run_end``-on-interrupt log's ``total_time``. Both
                ``plan_sequence()`` and ``resume_sequence()`` now pass
                their own (resume's covers only the resumed portion, not
                the original call before the failure).
            phase_q_hints: Optional ``{phase_idx: hint}`` warm-start map,
                expanded per-edge by ``_edge_hints_for_phase()`` (see it
                for the accepted shapes and why a whole chain, rather than
                just the phase's committed config, is what actually pins
                the outcome). Each edge's hint is passed as ``q_hint`` to
                its ``generate_via_edge()`` call. Used by
                ``find_feasible_phase_target()`` to steer target generation
                toward a candidate already verified to leave the NEXT
                phase reachable, instead of accepting whatever the
                unguided random restart lands on -- see that method's
                docstring for the RS6/CON0 case this exists to fix.

                Never applied to the collision-retry regeneration call
                below: its purpose is drawing a genuinely fresh sample when
                RRT planning fails on an otherwise-valid target, and
                re-feeding the same hint there would just reconverge to the
                same unplannable point. That redraw does break the chain
                though, so it records ``phase_idx`` in
                ``self.invalidated_phase_hints`` -- the phase may still
                complete, but the lookahead's guarantee about the next
                phase is void, and the caller should re-run the lookahead
                instead of resuming forward on it.

        Returns:
            Dict with keys ``phase_paths``, ``phase_geometric_paths``,
            ``edge_stats_list``, ``q_start`` (end config of the last
            successfully planned edge), ``q_pregrasp_for_cache``.

        Raises:
            KeyboardInterrupt: on a graceful-stop request.
            RuntimeError: on target-generation or planning failure,
                wrapping the original exception.
        """
        phase_paths = []
        phase_geometric_paths = []  # Geometric paths (no time param) for saving
        edge_stats_list = []  # Per-edge timing stats for this phase
        q_start = q_current
        q_pregrasp_for_cache = None  # end config of the _01 edge (pregrasp)
        edge_hints = self._edge_hints_for_phase(
            phase_q_hints.get(phase_idx) if phase_q_hints else None,
            len(edge_sequence),
        )

        for edge_idx in range(start_edge_idx, len(edge_sequence)):
            # Resolve the edge name first so the stop-request handler below
            # can record it (otherwise a stop on the first iteration would
            # reference an undefined `edge_name`).
            edge_name = edge_sequence[edge_idx]

            # Check for stop request
            if is_stop_requested():
                if verbose:
                    logger.warning("Stop requested - saving progress...")
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
                    logger.warning(
                        "Stored partial phase result: %d edges completed",
                        len(phase_paths),
                    )
                    logger.warning(
                        "You can resume from Phase %d, Edge %d",
                        phase_idx + 1,
                        edge_idx + 1,
                    )

                # Disable signal handler before raising
                disable_graceful_stop()
                # Emit run_end on user interrupt, for both plan_sequence()
                # and resume_sequence() (both now pass a valid
                # loop_start_time) -- logging-asymmetry fix, 2026-08-09.
                if self.run_logger is not None:
                    try:
                        self.run_logger.log(
                            "run_end",
                            success=False,
                            total_time=time.time() - loop_start_time,
                            total_planning_time=self.total_planning_time,
                            phase_count=len(self.phase_results),
                            final_config=None,
                            error="user_interrupt",
                        )
                        self.run_logger.close()
                    except Exception:
                        pass
                raise KeyboardInterrupt("Planning stopped by user request")

            edge_start_time = time.time()

            if is_resume:
                # Get previous attempt count for this edge
                prev_stat = self.edge_stats.get((phase_idx, edge_idx))
                attempt_num = (prev_stat["attempt"] + 1) if prev_stat else 1
            else:
                attempt_num = 1

            edge_stat = {
                "edge_idx": edge_idx,
                "edge_name": edge_name,
                "attempt": attempt_num,
                "gen_time": 0.0,
                "plan_time": 0.0,
                "total_time": 0.0,
                "success": False,
            }
            if is_resume:
                edge_stat["is_resume"] = True

            if self.run_logger is not None:
                try:
                    self.run_logger.log(
                        "edge_start",
                        phase=phase_idx + 1,
                        edge_idx=edge_idx,
                        edge_name=edge_name,
                        q_from=list(q_start),
                    )
                except Exception:
                    pass

            attempt_str = (
                f" (attempt #{attempt_num})" if is_resume and attempt_num > 1 else ""
            )
            if verbose:
                logger.info(
                    "Planning waypoint edge %d/%d: %s%s",
                    edge_idx + 1,
                    len(edge_sequence),
                    edge_name,
                    attempt_str,
                )

            # Generate target configuration via this edge
            gen_start = time.time()
            q_target = None
            try:
                config_label = f"q_phase{phase_idx}_edge{edge_idx}"
                if is_resume:
                    config_label += "_resume"
                q_hint = edge_hints[edge_idx]
                ok, q_target = self.config_gen.generate_via_edge(
                    edge_name=edge_name,
                    q_from=q_start,
                    config_label=config_label,
                    q_hint=q_hint,
                )

                if is_resume:
                    if not ok or q_target is None:
                        raise RuntimeError(
                            f"Failed to generate target via edge '{edge_name}'"
                        )
                    edge_stat["gen_time"] = time.time() - gen_start
                else:
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
                            logger.info(
                                "✓ Generated target config (%.2fs)",
                                edge_stat["gen_time"],
                            )
                            logger.debug("q_target: %s", q_target)

                    # Visualize the configuration before planning.
                    # Only attempted if the backend viewer has been explicitly
                    # set up via setup_viewer(); skipped otherwise to avoid
                    # SIGSEGV from omniORB when gepetto-viewer is not running.
                    try:
                        if (
                            verbose
                            and hasattr(self.planner, "viewer")
                            and self.planner.viewer is not None
                        ):
                            logger.debug(
                                "Visualizing q_target for edge '%s' before planning...",
                                edge_name,
                            )
                            self.planner.visualize(q_target)
                            logger.debug("✓ q_target sent to viewer")
                    except Exception as e:
                        logger.warning("Could not visualize q_target: %s", e)

            except Exception as e:
                edge_stat["gen_time"] = time.time() - gen_start
                edge_stat["total_time"] = time.time() - edge_start_time
                edge_stats_list.append(edge_stat)

                if self.run_logger is not None:
                    try:
                        self.run_logger.log(
                            "edge_end",
                            phase=phase_idx + 1,
                            edge_idx=edge_idx,
                            edge_name=edge_name,
                            success=False,
                            gen_time=edge_stat["gen_time"],
                            plan_time=0.0,
                            total_time=edge_stat["total_time"],
                            q_to=None,
                            error=str(e),
                        )
                    except Exception:
                        pass

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
                        [p for p in self.phase_results if p.get("complete", False)]
                    ),
                    "completed_edges_in_phase": len(phase_paths),
                }

                if verbose:
                    logger.warning(
                        "Stored partial phase result: %d edges completed",
                        len(phase_paths),
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
                    logger.info(
                        "⏭ Skipped motion planning (%.2fs total)",
                        edge_stat["total_time"],
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
                            if not is_resume:
                                logger.debug("Planning: q_start -> q_target")
                            # else: label already logged above (with attempt suffix)
                        else:
                            _prev_reason = (
                                str(last_plan_exc).split("\n")[0]
                                if last_plan_exc
                                else ""
                            )
                            logger.debug(
                                "Planning (attempt %d) [prev failed: %s]",
                                _plan_attempt + 1,
                                _prev_reason,
                            )

                    path, geometric_path = self.planner.plan_transition_edge(
                        edge=edge_name,
                        q1=q_start,
                        q2=q_target,
                    )

                    if path is None:
                        raise RuntimeError(f"Planning failed for edge '{edge_name}'")

                    last_plan_exc = None
                    break  # success

                except Exception as _plan_exc:
                    last_plan_exc = _plan_exc
                    if _plan_attempt < self._MAX_COLLISION_RETRIES - 1:
                        if verbose:
                            logger.warning(
                                "Planning failed (attempt %d), "
                                "regenerating target config...",
                                _plan_attempt + 1,
                            )
                        _ok2, _q_new = self.config_gen.generate_via_edge(
                            edge_name=edge_name,
                            q_from=q_start,
                            config_label=(f"q_phase{phase_idx}_edge{edge_idx}"),
                        )
                        if (
                            _ok2
                            and _q_new is not None
                            and _np.all(_np.isfinite(_np.array(_q_new)))
                        ):
                            q_target = _q_new
                            if verbose:
                                logger.debug("Regenerated target config")
                            # A random redraw replacing a hinted target
                            # breaks the chain: everything downstream of
                            # this edge now solves from a q_from the
                            # lookahead never probed, so its "next phase
                            # stays reachable" guarantee no longer holds.
                            if edge_hints[edge_idx] is not None:
                                self.invalidated_phase_hints.add(phase_idx)
                                if verbose:
                                    logger.warning(
                                        "Phase %d's hint chain broken at edge "
                                        "%d (%s): target redrawn at random "
                                        "after a planning failure — the "
                                        "lookahead guarantee for the next "
                                        "phase no longer holds",
                                        phase_idx + 1,
                                        edge_idx + 1,
                                        edge_name,
                                    )

            if last_plan_exc is not None:
                e = last_plan_exc
                edge_stat["plan_time"] = time.time() - plan_start
                edge_stat["total_time"] = time.time() - edge_start_time
                edge_stats_list.append(edge_stat)
                self.total_planning_time += edge_stat["total_time"]

                if self.run_logger is not None:
                    try:
                        self.run_logger.log(
                            "edge_end",
                            phase=phase_idx + 1,
                            edge_idx=edge_idx,
                            edge_name=edge_name,
                            success=False,
                            gen_time=edge_stat["gen_time"],
                            plan_time=edge_stat["plan_time"],
                            total_time=edge_stat["total_time"],
                            q_to=None,
                            error=str(e),
                        )
                    except Exception:
                        pass

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

                if is_resume and verbose:
                    logger.warning(
                        "Failed after %.2fs (attempt #%d)",
                        edge_stat["total_time"],
                        attempt_num,
                    )

                # Store failure info for resume
                self.last_failure_info = {
                    "phase_idx": phase_idx,
                    "edge_idx": edge_idx,
                    "edge_name": edge_name,
                    "q_current": q_current,
                    "error": str(e),
                    "completed_phases": len(
                        [p for p in self.phase_results if p.get("complete", False)]
                    ),
                    "completed_edges_in_phase": len(phase_paths),
                }

                if not is_resume and verbose:
                    logger.warning(
                        "Stored partial phase result: %d edges completed",
                        len(phase_paths),
                    )

                if is_resume:
                    raise RuntimeError(
                        f"Resume failed at Phase {phase_idx + 1}, edge {edge_idx + 1}: {e}"
                    ) from e
                else:
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

            if self.run_logger is not None:
                try:
                    self.run_logger.log(
                        "edge_end",
                        phase=phase_idx + 1,
                        edge_idx=edge_idx,
                        edge_name=edge_name,
                        success=True,
                        gen_time=edge_stat["gen_time"],
                        plan_time=edge_stat["plan_time"],
                        total_time=edge_stat["total_time"],
                        q_to=list(q_target) if q_target is not None else None,
                        error=None,
                    )
                except Exception:
                    pass

            phase_paths.append(path)

            # Update start config for next edge
            if hasattr(path, "getInitialConfig") and hasattr(path, "getEndConfig"):
                q_start = list(path.getEndConfig())
            else:
                q_start = q_target

            # Capture end config of the first (_01) edge as the pregrasp config
            if edge_idx == 0:
                q_pregrasp_for_cache = list(q_start)

            if verbose:
                logger.info(
                    "✓ Path found (%.2fs plan, %.2fs total)",
                    edge_stat["plan_time"],
                    edge_stat["total_time"],
                )

        return {
            "phase_paths": phase_paths,
            "phase_geometric_paths": phase_geometric_paths,
            "edge_stats_list": edge_stats_list,
            "q_start": q_start,
            "q_pregrasp_for_cache": q_pregrasp_for_cache,
        }

    def _finalize_phase_result(
        self,
        phase_idx: int,
        gripper: str,
        handle: str | None,
        edge_sequence: list[str],
        phase_paths: list[Any],
        phase_geometric_paths: list[Any],
        edge_stats_list: list[dict[str, Any]],
        q_start: list[float],
        q_pregrasp_for_cache: list[float] | None,
        skip_phases: set[int] | None,
        verbose: bool,
        is_resume: bool,
    ) -> list[float]:
        """Finalize a successfully-planned phase: timing, state, save, record.

        Shared by ``plan_sequence()`` and ``resume_sequence()`` -- extracted
        verbatim from both (Phase 1, Step 1.6 of the codebase refactor).
        Updates ``self.grasp_tracker``, ``self._last_pregrasp_q``, and
        appends to ``self.phase_results`` as a side effect.

        Two deliberate order simplifications versus the originals (same
        spirit as Step 1.3's ConfigGenerator branch-order unification):
        the auto-save-vs-pregrasp-cache call order was swapped between the
        two originals, and so was the grasp-tracker-update-vs-timing-calc
        order. Neither pair of operations reads the other's output, so
        both orderings are provably interchangeable -- unified on
        ``plan_sequence()``'s order; not a behavior change.

        Args:
            is_resume: Selects between the two call sites' pre-existing,
                genuinely different (not just verbosity) console-print
                behaviors:
                  - ``plan_sequence()`` prints "Completed N-edge sequence"
                    plus phase timing before updating grasp state;
                    ``resume_sequence()`` has no such print at that point.
                  - ``plan_sequence()`` has no post-append print;
                    ``resume_sequence()`` prints "Completed phase N (Xs
                    total)" *after* appending the phase result.
                The ``"phase_end"`` RunLogger event is **not** gated by
                ``is_resume`` -- both callers emit it (fixed 2026-08-09;
                see the codebase refactor plan's logging-asymmetry
                section).

        Returns:
            The finalized ``q_current`` (== ``q_start``, the config at
            the end of the phase), which the caller carries into the
            next loop iteration.
        """
        q_current = q_start

        # Compute phase timing totals
        phase_total_time = sum(s["total_time"] for s in edge_stats_list)
        phase_plan_time = sum(s["plan_time"] for s in edge_stats_list)
        phase_gen_time = sum(s["gen_time"] for s in edge_stats_list)

        if not is_resume and verbose:
            logger.info("✓ Completed %d-edge sequence", len(edge_sequence))
            logger.info(
                "Phase timing: %.2fs total (gen: %.2fs, plan: %.2fs)",
                phase_total_time,
                phase_gen_time,
                phase_plan_time,
            )

        # Update grasp state after successful planning
        self.grasp_tracker.update_grasp(gripper, handle)

        # Cache q_pregrasp for potential future auto-release of this gripper.
        # q_pregrasp = end config of the first (_01) edge (approach/pregrasp node).
        if handle is not None and q_pregrasp_for_cache is not None:
            self._last_pregrasp_q[gripper] = q_pregrasp_for_cache
            if verbose:
                logger.info("✓ Cached q_pregrasp for '%s'", gripper)

        # Auto-save paths after successful phase
        saved_files = self._auto_save_phase_paths(
            phase_idx=phase_idx,
            phase_paths=phase_paths,
            edge_names=edge_sequence,
            verbose=verbose,
            phase_geometric_paths=(
                phase_geometric_paths if self.auto_save_dir else None
            ),
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

        # Emit phase_end before appending so callers can react even if
        # they iterate self.phase_results incrementally. Not gated by
        # is_resume (logging-asymmetry fix, 2026-08-09).
        if self.run_logger is not None:
            try:
                self.run_logger.log(
                    "phase_end",
                    phase=phase_idx + 1,
                    gripper=gripper,
                    handle=handle,
                    success=True,
                    phase_time=phase_total_time,
                    phase_gen_time=phase_gen_time,
                    phase_plan_time=phase_plan_time,
                    final_config=list(q_current),
                    state_after=self.grasp_tracker.get_current_state_name(),
                    saved_files=saved_files,
                    error=None,
                )
            except Exception:
                pass

        self.phase_results.append(phase_result)

        if is_resume and verbose:
            logger.info(
                "✓ Completed phase %d (%.2fs total)",
                phase_idx + 1,
                phase_total_time,
            )

        return q_current

    def _run_phase_loop(
        self,
        phases: Sequence[tuple[str, str | None]],
        starting_phase_idx: int,
        total_phase_count_for_display: int,
        q_current: list[float],
        frozen_arms_mode: str,
        per_phase_frozen_arms: dict[int, list[str]] | None,
        q_scene_init: list[float] | None,
        skip_phases: set[int] | None,
        is_resume: bool,
        verbose: bool,
        retry_from_edge: int = 0,
        completed_edges_in_phase_for_resume: int = 0,
        loop_start_time: float | None = None,
        phase_q_hints: dict[int, list[list[float]] | list[float]] | None = None,
    ) -> list[float]:
        """Run the per-phase planning loop shared by ``plan_sequence()`` and
        ``resume_sequence()``.

        The final consolidation step of the codebase refactor (Phase 1,
        Step 1.7): with Steps 1.1-1.6 already pulling every per-phase
        sub-block into its own shared method, both callers' loop bodies
        had converged to the same sequence of calls modulo an `is_resume`
        flag threaded through each -- this extracts that shared sequence
        itself.

        Args:
            phases: The (gripper, handle) pairs to iterate.
                ``plan_sequence()`` passes its full ``grasp_sequence``;
                ``resume_sequence()`` passes its computed
                ``remaining_sequence``.
            starting_phase_idx: Added to this loop's own enumeration index
                to get the real ``phase_idx`` used in prints/bookkeeping.
                ``0`` for ``plan_sequence()``; ``incomplete_phase_idx`` for
                ``resume_sequence()``.
            total_phase_count_for_display: Denominator for the "Phase
                X/Y" progress print. ``len(grasp_sequence)`` for
                ``plan_sequence()``; ``len(self.original_sequence)`` for
                ``resume_sequence()`` (its own ``phases`` is only the
                remaining subset, but the display should still show
                progress against the full original sequence).
            frozen_arms_mode: Already resolved to a concrete string by the
                caller. ``resume_sequence()`` used to re-resolve
                ``None -> "global"`` three times per iteration (once per
                sub-call) even though the value never changes across the
                loop; now resolved once, before calling this method --
                provably equivalent, not a behavior change.
            is_resume: Threaded through to every sub-call's own
                ``emit_logs``/``is_resume`` parameter (see each method's
                own docstring for what it changes).
            retry_from_edge: Only meaningful when ``is_resume``: which
                edge to resume the *first* phase processed in this call
                from (subsequent phases always start at edge 0).
            completed_edges_in_phase_for_resume: Only meaningful when
                ``is_resume``: preserved verbatim from the original
                inline no-op check (``if start_edge_idx < ...: pass``) --
                dead code in both originals, kept as-is rather than
                removed, since removing it would be a separate cleanup
                outside this step's scope.
            loop_start_time: Only meaningful when not ``is_resume``: see
                ``_plan_phase_edges``'s docstring.
            phase_q_hints: Forwarded verbatim to ``_plan_phase_edges()`` --
                see its docstring.

        Returns:
            The final ``q_current`` after every phase in ``phases``
            completes.
        """
        for idx_in_call, (gripper, handle) in enumerate(phases):
            phase_idx = starting_phase_idx + idx_in_call

            if verbose:
                print("\n" + "-" * 70)
                logger.info(
                    "--- Phase %d/%d ---",
                    phase_idx + 1,
                    total_phase_count_for_display,
                )
                if handle is not None:
                    logger.info("Grasp '%s' with '%s'", handle, gripper)
                else:
                    logger.info("Release with '%s'", gripper)
                current_state = self.grasp_tracker.get_current_state_name()
                logger.info("Current state: %s", current_state)

            self._dump_phase_checkpoint(phase_idx, gripper, handle, q_current, verbose)

            # ----------------------------------------------------------------
            # Handle explicit release entry: (gripper, None)
            # ----------------------------------------------------------------
            currently_held = self.grasp_tracker.current_grasps.get(gripper)

            if handle is None:
                q_current = self._plan_release_entry_phase(
                    phase_idx=phase_idx,
                    gripper=gripper,
                    currently_held=currently_held,
                    q_current=q_current,
                    frozen_arms_mode=frozen_arms_mode,
                    verbose=verbose,
                    per_phase_frozen_arms=per_phase_frozen_arms,
                )
                continue

            # ----------------------------------------------------------------
            # Auto-detect conflict: gripper holds a different object.
            # Insert a release sub-phase before the grasp so the factory can
            # build valid edges.
            # ----------------------------------------------------------------
            q_current = self._plan_auto_release_if_needed(
                phase_idx=phase_idx,
                gripper=gripper,
                handle=handle,
                currently_held=currently_held,
                q_current=q_current,
                frozen_arms_mode=frozen_arms_mode,
                verbose=verbose,
                per_phase_frozen_arms=per_phase_frozen_arms,
            )

            self._build_phase_graph_and_constraints(
                phase_idx=phase_idx,
                gripper=gripper,
                handle=handle,
                q_current=q_current,
                frozen_arms_mode=frozen_arms_mode,
                per_phase_frozen_arms=per_phase_frozen_arms,
                q_scene_init=q_scene_init,
                verbose=verbose,
                emit_logs=not is_resume,
            )

            edge_sequence, q_current = self._compute_and_project_edge_sequence(
                phase_idx=phase_idx,
                gripper=gripper,
                handle=handle,
                q_current=q_current,
                verbose=verbose,
                emit_logs=not is_resume,
            )

            start_edge_idx = 0
            if is_resume and idx_in_call == 0 and retry_from_edge >= 0:
                # Resuming failed phase: start from specified edge
                start_edge_idx = retry_from_edge
                if start_edge_idx < completed_edges_in_phase_for_resume:
                    # Reuse previously completed paths in this phase
                    # (though this is complex - for now just restart phase)
                    pass

            _edge_result = self._plan_phase_edges(
                phase_idx=phase_idx,
                gripper=gripper,
                handle=handle,
                edge_sequence=edge_sequence,
                q_current=q_current,
                skip_phases=skip_phases,
                start_edge_idx=start_edge_idx,
                is_resume=is_resume,
                verbose=verbose,
                loop_start_time=loop_start_time,
                phase_q_hints=phase_q_hints,
            )
            phase_paths = _edge_result["phase_paths"]
            phase_geometric_paths = _edge_result["phase_geometric_paths"]
            edge_stats_list = _edge_result["edge_stats_list"]
            q_start = _edge_result["q_start"]
            q_pregrasp_for_cache = _edge_result["q_pregrasp_for_cache"]

            q_current = self._finalize_phase_result(
                phase_idx=phase_idx,
                gripper=gripper,
                handle=handle,
                edge_sequence=edge_sequence,
                phase_paths=phase_paths,
                phase_geometric_paths=phase_geometric_paths,
                edge_stats_list=edge_stats_list,
                q_start=q_start,
                q_pregrasp_for_cache=q_pregrasp_for_cache,
                skip_phases=skip_phases,
                verbose=verbose,
                is_resume=is_resume,
            )

        return q_current

    def find_feasible_phase_target(
        self,
        phase_n: tuple[str, str],
        phase_n1: tuple[str, str],
        q_current: list[float],
        q_scene_init: list[float] | None,
        frozen_arms_n: list[str],
        frozen_arms_n1: list[str],
        probe_timeout: float = 5.0,
        max_candidates: int = 100,
        verbose: bool = True,
    ) -> list[list[float]] | None:
        """Search for a phase-N grasp target that leaves phase N+1 reachable.

        Fixes a real failure class: a phase's target-generation call
        (``ConfigGenerator.generate_via_edge()``) is randomized (random-
        restart IK), and whichever valid solution it lands on can pin
        constraints the NEXT phase depends on with no way back -- e.g. RS6's
        WB grasp (phase N) is one of two simultaneous fixed-offset grasps on
        the same rigid object, so once it and the object's other grasp are
        both committed, the object's orientation is fully determined and
        phase N+1 (CON0) cannot change it no matter how long IT retries.
        Traced live: RS6's CON0 grasp failed ~2300+ consecutive target-
        generation attempts (0 ever reached collision-checking, both with
        the next arm frozen and after the existing 30-failure-unfreeze
        escalation fired) purely because the WB grasp's random draw happened
        to leave CON0 facing away from the tool -- confirmed visually in
        viser. No amount of retrying phase N+1 alone can fix a bad phase-N
        commitment; the fix has to re-roll phase N.

        Cheap relative to a blind multi-hour retry stall, but NOT flat-rate
        cheap -- ``build_phase_graph()``'s cost scales with how many
        objects/grippers are already held, not a constant. Measured ~65ms
        against RS6's checkpoint (only RS1 held at that point in the
        sequence); measured ~6.2-6.4s against RS2's checkpoint (RS1+RS5+RS6
        all held by then) -- roughly two orders of magnitude more, purely
        from more locked joints/objects for ConstraintGraphFactory to set
        up. Each candidate that reaches the phase-N+1 probe therefore costs
        the phase-N+1 build (this ~6s-and-growing figure) plus up to
        ``probe_timeout`` -- ~7-12s/candidate by RS2, not milliseconds.
        Target generation itself stays fast throughout (~30ms/attempt
        measured, confirmed still true at RS2). Still cheap next to a
        single failed ``plan_transition_edge()`` attempt, let alone the
        2300+ retries this replaces -- but ``max_candidates`` needs enough
        headroom to survive several rejected draws at this heavier
        per-candidate cost, not just the lighter early-sequence cost.

        Draws a phase-N candidate, then immediately probes (short timeout,
        discarding the resulting config) whether phase N+1 is reachable from
        it. Two full ``build_phase_graph()`` calls per candidate, not one:
        ``self.graph`` is a singleton, so building phase N+1's graph tears
        down phase N's -- there is no way to keep
        both alive at once to compare candidates. Never mutates
        ``self.grasp_tracker`` (only a throwaway ``.copy()``) -- committing
        a hypothetical grasp to the real tracker while merely probing is
        the exact corruption class ``_restore_grasp_tracker_for_resume()``
        exists to fix (see its docstring): it would silently corrupt state
        the real, subsequent ``plan_sequence(phase_q_hints=...)`` call
        depends on.

        Only supports grasp phases (``handle is not None``) for both N and
        N+1 -- release phases use a different edge-sequence method
        (``get_release_edge_sequence``) not wired in here, since the one
        real caller (RS6's WB-grasp -> CON0-grasp pair) never needs it.
        Only manual frozen-arms resolution (an explicit list per phase, not
        "auto"/"interactive"/"global") for the same reason -- the real
        caller (``run_block_nonstop``) always uses
        ``frozen_arms_mode="manual"`` with an explicit dict already in hand.

        Args:
            phase_n: ``(gripper, handle)`` for the phase whose target is
                being searched for (e.g. VISPA2's WB grasp on RS6).
            phase_n1: ``(gripper, handle)`` for the phase that must remain
                reachable from phase N's candidate (e.g. CON0 grasp).
            q_current: Configuration entering phase N (unchanged across the
                whole search -- every candidate is drawn from the same
                start).
            q_scene_init: True scene-initial configuration, passed through
                to ``build_phase_graph()`` exactly as
                ``_build_phase_graph_and_constraints()`` does.
            frozen_arms_n: Locked-arm keywords for phase N, matching
                ``per_phase_frozen_arms[phase_n_idx]`` in the real call.
            frozen_arms_n1: Locked-arm keywords for phase N+1.
            probe_timeout: Wall-clock cap per probe ``generate_via_edge()``
                call. Short by design -- this only needs to know a solution
                exists, not find the one the real run will use.
            max_candidates: How many phase-N candidates to try before giving
                up. Per-candidate cost grows with how many objects are
                already held by this point in the sequence (see cost note
                above) -- ~7-12s/candidate by RS2, worse for RS3/RS4. 100
                gives real headroom (up to ~15-20 min worst case) rather
                than gambling on an early-sequence-cost budget that starves
                the search later in the run, as 20 did for RS2 (exhausted
                its whole budget in ~22s and never found a candidate that a
                slightly luckier draw found on attempt #2).
            verbose: Log progress/outcome.

        Returns:
            The full per-edge config **chain** the winning candidate was
            built from -- one config per edge of phase N's edge sequence,
            terminal config last -- if a candidate was found that also
            leaves phase N+1 reachable. Pass it as
            ``phase_q_hints={phase_n_idx: result}`` to the real
            ``plan_sequence()``/``resume_sequence()`` call so it reproduces
            this candidate instead of drawing a fresh (possibly bad) one.
            ``None`` if the budget was exhausted without finding one --
            callers should fall back to today's behavior (no hint).

            The whole chain, not just ``result[-1]``: seeding only the
            terminal edge leaves the pregrasp edges randomized, and since
            each edge's constraint RHS comes from its predecessor's end
            config, the committed config then drifts off this candidate and
            takes the phase N+1 guarantee with it. See
            ``_edge_hints_for_phase()`` for the mechanism and the live RS5
            case. (``result[-1]`` is still exactly phase N's committed
            target if a caller needs just that.)
        """
        from long_tamp.planning.constraints import ConstraintBuilder

        gripper_n, handle_n = phase_n
        gripper_n1, handle_n1 = phase_n1

        def _sync_graph_refs(new_graph) -> None:
            if hasattr(self.planner, "graph"):
                self.planner.graph = new_graph
            if self.config_gen is None:
                from long_tamp.planning import ConfigGenerator

                self.config_gen = ConfigGenerator(
                    self.graph_builder.robot,
                    new_graph,
                    self.planner,
                    self.graph_builder.ps,
                    backend=self.backend,
                )
            elif hasattr(self.config_gen, "update_graph"):
                self.config_gen.update_graph(new_graph)

        def _build_and_sync(
            tracker: GraspStateTracker,
            next_grasp: tuple[str, str],
            frozen_arms: list[str],
            q_for_build: list[float],
        ) -> None:
            held = {g: h for g, h in tracker.current_grasps.items() if h is not None}
            constraint_names = None
            if frozen_arms:
                constraint_names, _ = ConstraintBuilder.create_locked_joint_constraints(
                    self.graph_builder.ps,
                    self.graph_builder.robot,
                    q_for_build,
                    frozen_arms,
                    backend=self.graph_builder.backend,
                )
            self.graph_builder.build_phase_graph(
                config=self.task_config,
                held_grasps=held,
                next_grasp=next_grasp,
                graph_constraints=constraint_names,
                q_init=q_for_build,
                q_init_original=q_scene_init,
            )
            _sync_graph_refs(self.graph_builder.get_graph())
            if hasattr(self.graph_builder, "_phase_grippers"):
                tracker.set_phase_indices(
                    self.graph_builder._phase_grippers,
                    self.graph_builder._phase_handles,
                )

        def _probe_chained(
            tracker: GraspStateTracker, gripper: str, handle: str, q_from: list[float]
        ) -> list[list[float]] | None:
            """Chain the grasp's edges from ``q_from``, returning every
            intermediate config rather than only the last one -- the
            pregrasp configs are what let the real run reproduce this
            chain edge for edge (see the Returns section above)."""
            q = q_from
            chain: list[list[float]] = []
            for edge_name in tracker.get_grasp_edge_sequence(gripper, handle):
                ok, q_next = self.config_gen.generate_via_edge(
                    edge_name=edge_name,
                    q_from=q,
                    timeout=probe_timeout,
                )
                if not ok or q_next is None:
                    return None
                q = q_next
                chain.append(list(q_next))
            return chain

        for candidate_idx in range(max_candidates):
            probe_tracker = self.grasp_tracker.copy()
            _build_and_sync(probe_tracker, phase_n, frozen_arms_n, q_current)

            chain = _probe_chained(probe_tracker, gripper_n, handle_n, q_current)
            if not chain:
                continue
            q_candidate = chain[-1]

            probe_tracker.update_grasp(gripper_n, handle_n)
            _build_and_sync(probe_tracker, phase_n1, frozen_arms_n1, q_candidate)

            probe_chain_n1 = _probe_chained(
                probe_tracker, gripper_n1, handle_n1, q_candidate
            )
            if probe_chain_n1 is not None:
                if verbose:
                    logger.info(
                        "find_feasible_phase_target: found a %s candidate "
                        "after %d rejected draw(s) that leaves %s reachable",
                        phase_n,
                        candidate_idx,
                        phase_n1,
                    )
                return chain

        if verbose:
            logger.warning(
                "find_feasible_phase_target: exhausted %d candidates for %s "
                "without finding one that leaves %s reachable",
                max_candidates,
                phase_n,
                phase_n1,
            )
        return None

    def plan_pregrasp(
        self,
        gripper: str,
        handle: str,
        q_current: list[float],
        frozen_arms_mode: str = "auto",
        per_phase_frozen_arms: dict[int, list[str]] | None = None,
        q_scene_init: list[float] | None = None,
        timeout_per_edge: float = 60.0,
        max_iterations_per_edge: int = 10000,
        verbose: bool = True,
    ) -> dict[str, Any]:
        """Plan only the approach/pregrasp edge for (gripper, handle); stop there.

        Unlike a normal grasp phase (``get_grasp_edge_sequence()``'s
        ``[pregrasp_edge, grasp_edge]``, both planned by ``plan_sequence()``),
        this plans only the first (``_01``) edge and never attempts the
        second. ``grasp_tracker`` is deliberately left unchanged (the
        gripper still shows as free afterward) -- for callers where the
        actual grasp/release is completed by something outside HPP entirely
        (e.g. visual servoing + a raw Cartesian approach + a PLC catch
        signal), so HPP must not believe the object is attached once this
        returns.

        Reuses ``_build_phase_graph_and_constraints`` (the same phase-graph
        setup ``plan_sequence()`` uses for phase 0) for graph/constraint/
        index correctness, then plans the single edge with the same
        generate-target/collision-retry pattern as ``_plan_phase_edges``,
        condensed to one edge with no resume/skip/hint machinery.

        Returns a dict shaped like ``plan_sequence()``'s (``success``,
        ``message``, ``phase_results``, ``final_config``) so existing
        phase_results-consuming code (``run_plan_sequence`` in
        ``planning_engine.py``) needs no changes to handle it.
        """
        if hasattr(self.planner, "configure_transition_planner"):
            self.planner.configure_transition_planner(
                time_out=timeout_per_edge,
                max_iterations=max_iterations_per_edge,
            )

        q_scene_init = q_scene_init if q_scene_init is not None else q_current

        try:
            self._build_phase_graph_and_constraints(
                phase_idx=0,
                gripper=gripper,
                handle=handle,
                q_current=q_current,
                frozen_arms_mode=frozen_arms_mode,
                per_phase_frozen_arms=per_phase_frozen_arms,
                q_scene_init=q_scene_init,
                verbose=verbose,
                emit_logs=verbose,
            )
        except Exception as e:
            return {
                "success": False,
                "message": (
                    f"plan_pregrasp('{gripper}', '{handle}'): failed to build "
                    f"phase graph: {e}"
                ),
                "phase_results": [],
                "final_config": q_current,
            }

        pregrasp_edge = self.grasp_tracker.get_grasp_edge_sequence(gripper, handle)[0]

        edge_start_time = time.time()
        q_target = None
        last_err: Exception | None = None
        path = None

        try:
            ok, q_target = self.config_gen.generate_via_edge(
                edge_name=pregrasp_edge,
                q_from=q_current,
                config_label=f"q_pregrasp_{gripper}",
            )
            if not ok or q_target is None:
                raise RuntimeError(
                    f"Failed to generate pregrasp target via '{pregrasp_edge}'"
                )
        except Exception as e:
            return {
                "success": False,
                "message": (
                    f"plan_pregrasp('{gripper}', '{handle}'): target "
                    f"generation failed: {e}"
                ),
                "phase_results": [],
                "final_config": q_current,
            }

        for attempt in range(self._MAX_COLLISION_RETRIES):
            try:
                path, _geometric_path = self.planner.plan_transition_edge(
                    edge=pregrasp_edge,
                    q1=q_current,
                    q2=q_target,
                )
                if path is None:
                    raise RuntimeError(f"Planning failed for edge '{pregrasp_edge}'")
                last_err = None
                break
            except Exception as e:
                last_err = e
                if verbose:
                    logger.warning(
                        "plan_pregrasp: attempt %d/%d failed (%s)%s",
                        attempt + 1,
                        self._MAX_COLLISION_RETRIES,
                        e,
                        ", regenerating target"
                        if attempt < self._MAX_COLLISION_RETRIES - 1
                        else "",
                    )
                if attempt < self._MAX_COLLISION_RETRIES - 1:
                    try:
                        ok2, q_new = self.config_gen.generate_via_edge(
                            edge_name=pregrasp_edge,
                            q_from=q_current,
                            config_label=f"q_pregrasp_{gripper}",
                        )
                        if ok2 and q_new is not None:
                            q_target = q_new
                    except Exception:
                        pass  # keep the previous q_target and retry planning with it

        edge_total_time = time.time() - edge_start_time

        if last_err is not None or path is None:
            return {
                "success": False,
                "message": (
                    f"plan_pregrasp('{gripper}', '{handle}') failed after "
                    f"{self._MAX_COLLISION_RETRIES} attempts: {last_err}"
                ),
                "phase_results": [],
                "final_config": q_current,
            }

        phase_result = {
            "phase": 1,
            "gripper": gripper,
            "handle": handle,
            "edges": [pregrasp_edge],
            "paths": [path],
            "edge_stats": [{"edge": pregrasp_edge, "total_time": edge_total_time}],
            "phase_time": edge_total_time,
            "complete": True,
            "skipped": False,
            "final_config": q_target,
            "state_after": self.grasp_tracker.get_current_state_name(),
            "saved_files": [],
            "pregrasp_only": True,  # marker: grasp_tracker intentionally NOT updated
        }
        self.phase_results.append(phase_result)

        if verbose:
            logger.info(
                "✓ plan_pregrasp('%s', '%s') succeeded (%.2fs); grasp_tracker "
                "left unchanged (still free)",
                gripper,
                handle,
                edge_total_time,
            )

        return {
            "success": True,
            "message": (
                f"Pregrasp reached for {gripper} -> {handle} "
                "(grasp not completed by design)"
            ),
            "phase_results": [phase_result],
            "final_config": q_target,
        }

    def plan_transition(
        self,
        edge_name: str,
        q1: list[float],
        q2: list[float],
        timeout_per_edge: float = 60.0,
        max_iterations_per_edge: int = 10000,
        verbose: bool = True,
    ) -> dict[str, Any]:
        """Plan a single named transition edge directly; no graph setup.

        Thin passthrough over ``plan_transition_edge`` for callers that
        already know exactly which edge they want and have already ensured
        (e.g. via a prior ``plan_pregrasp()``/``build_phase_graph()`` call
        in this same instance) that the edge exists in whatever graph is
        currently loaded on ``self.planner``/``self.graph_builder`` -- this
        method does no graph-setup of its own, unlike ``plan_pregrasp``.
        """
        if hasattr(self.planner, "configure_transition_planner"):
            self.planner.configure_transition_planner(
                time_out=timeout_per_edge,
                max_iterations=max_iterations_per_edge,
            )

        try:
            path, _geometric_path = self.planner.plan_transition_edge(
                edge=edge_name,
                q1=q1,
                q2=q2,
            )
            if path is None:
                raise RuntimeError(f"Planning failed for edge '{edge_name}'")
        except Exception as e:
            return {
                "success": False,
                "message": f"plan_transition('{edge_name}') failed: {e}",
                "phase_results": [],
                "final_config": q1,
            }

        phase_result = {
            "phase": 1,
            "gripper": None,
            "handle": None,
            "edges": [edge_name],
            "paths": [path],
            "edge_stats": [],
            "complete": True,
            "skipped": False,
            "final_config": q2,
            "state_after": self.grasp_tracker.get_current_state_name(),
            "saved_files": [],
        }
        self.phase_results.append(phase_result)

        if verbose:
            logger.info("✓ plan_transition('%s') succeeded", edge_name)

        return {
            "success": True,
            "message": f"Transition '{edge_name}' planned",
            "phase_results": [phase_result],
            "final_config": q2,
        }

    def plan_loop(
        self,
        gripper: str,
        q_current: list[float],
        q_target: list[float],
        frozen_arms_mode: str = "auto",
        per_phase_frozen_arms: dict[int, list[str]] | None = None,
        q_scene_init: list[float] | None = None,
        timeout_per_edge: float = 60.0,
        max_iterations_per_edge: int = 10000,
        verbose: bool = True,
    ) -> dict[str, Any]:
        """Plan free motion for ``gripper``'s arm to an explicit ``q_target``,
        within the current held-grasps state — no grasp/release involved.

        Resolves the graph-lifecycle correctness question noted in an
        earlier version of this method (raising ``NotImplementedError``):
        reuses the exact ``next_grasp=(gripper, None)`` graph-building path
        ``_setup_release_phase_graph``/``build_phase_graph`` already use for
        releases ([graph.py:1276-1292] handles ``next_handle is None``
        explicitly), via the same ``_build_phase_graph_and_constraints``
        helper ``plan_pregrasp`` uses. That builds a *fresh*, correctly
        scoped-to-``held_grasps`` phase graph on every call — this doesn't
        depend on whatever graph a prior, unrelated request happened to
        leave loaded. ``grasp_tracker`` is left unchanged (no grasp/release
        happened).

        Note: unlike ``plan_pregrasp``, ``q_target`` is caller-supplied and
        fixed — there is nothing to regenerate on a failed attempt, so
        retries replan the same ``q1``/``q2`` (useful only insofar as the
        underlying sampling-based planner is itself stochastic between
        calls), rather than drawing a fresh candidate target each time.
        """
        if hasattr(self.planner, "configure_transition_planner"):
            self.planner.configure_transition_planner(
                time_out=timeout_per_edge,
                max_iterations=max_iterations_per_edge,
            )

        q_scene_init = q_scene_init if q_scene_init is not None else q_current

        try:
            self._build_phase_graph_and_constraints(
                phase_idx=0,
                gripper=gripper,
                handle=None,
                q_current=q_current,
                frozen_arms_mode=frozen_arms_mode,
                per_phase_frozen_arms=per_phase_frozen_arms,
                q_scene_init=q_scene_init,
                verbose=verbose,
                emit_logs=verbose,
            )
        except Exception as e:
            return {
                "success": False,
                "message": f"plan_loop('{gripper}'): failed to build phase graph: {e}",
                "phase_results": [],
                "final_config": q_current,
            }

        loop_edge = self.grasp_tracker.get_loop_edge()

        edge_start_time = time.time()
        last_err: Exception | None = None
        path = None

        for attempt in range(self._MAX_COLLISION_RETRIES):
            try:
                path, _geometric_path = self.planner.plan_transition_edge(
                    edge=loop_edge,
                    q1=q_current,
                    q2=q_target,
                )
                if path is None:
                    raise RuntimeError(f"Planning failed for edge '{loop_edge}'")
                last_err = None
                break
            except Exception as e:
                last_err = e
                if verbose:
                    logger.warning(
                        "plan_loop: attempt %d/%d failed: %s",
                        attempt + 1,
                        self._MAX_COLLISION_RETRIES,
                        e,
                    )

        edge_total_time = time.time() - edge_start_time

        if last_err is not None or path is None:
            return {
                "success": False,
                "message": (
                    f"plan_loop('{gripper}') failed after "
                    f"{self._MAX_COLLISION_RETRIES} attempts: {last_err}"
                ),
                "phase_results": [],
                "final_config": q_current,
            }

        phase_result = {
            "phase": 1,
            "gripper": gripper,
            "handle": None,
            "edges": [loop_edge],
            "paths": [path],
            "edge_stats": [{"edge": loop_edge, "total_time": edge_total_time}],
            "phase_time": edge_total_time,
            "complete": True,
            "skipped": False,
            "final_config": q_target,
            "state_after": self.grasp_tracker.get_current_state_name(),
            "saved_files": [],
            "loop_only": True,  # marker: no grasp/release, grasp_tracker unchanged
        }
        self.phase_results.append(phase_result)

        if verbose:
            logger.info(
                "✓ plan_loop('%s') succeeded (%.2fs)", gripper, edge_total_time
            )

        return {
            "success": True,
            "message": f"Loop motion planned for '{gripper}' to explicit target",
            "phase_results": [phase_result],
            "final_config": q_target,
        }

    def plan_sequence(
        self,
        grasp_sequence: Sequence[tuple[str, str]],
        q_init: Sequence[float],
        validate: bool = True,
        reset_roadmap: bool = True,
        time_parameterize: bool = True,
        max_iterations_per_edge: int = 10000,
        timeout_per_edge: float = 60.0,
        q_scene_init: Sequence[float] | None = None,
        frozen_arms_mode: str = "auto",
        per_phase_frozen_arms: dict[int, list[str]] | None = None,
        skip_phases: set[int] | None = None,
        verbose: bool = True,
        phase_q_hints: dict[int, list[list[float]] | list[float]] | None = None,
    ) -> dict[str, Any]:
        """Plan a sequence of grasp transitions.

        Args:
            grasp_sequence: List of (gripper, handle) pairs to grasp in order
            q_init: Initial configuration (all grippers free)
            validate: Validate paths after planning
            reset_roadmap: Reset roadmap between phases
            time_parameterize: Apply time parameterization to paths
            max_iterations_per_edge: Max iterations per edge planning
            timeout_per_edge: Timeout in seconds for each edge planning
            q_scene_init: True original scene configuration (all grippers
                free, objects at their real scene poses), used to lock free
                objects at their real positions during phase graph builds.
                Defaults to q_init. Only pass this explicitly when q_init is
                NOT the phase-0 starting config — e.g. a diagnostic script
                that starts mid-sequence from a checkpointed q_current.
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
            phase_q_hints: Optional ``{phase_idx: hint}`` map warm-starting
                that phase's target generation, where ``hint`` is the
                per-edge config chain returned by
                ``find_feasible_phase_target()`` (a single config, applied
                to the last edge only, is also accepted) -- see
                ``_edge_hints_for_phase()``. Any phase whose chain gets
                broken by a collision-retry redraw is recorded in
                ``self.invalidated_phase_hints``; check it before resuming
                forward, since the hint's guarantee about the next phase no
                longer holds.

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
                logger.info(
                    "Configured TransitionPlanner: timeout=%ss, max_iterations=%s",
                    timeout_per_edge,
                    max_iterations_per_edge,
                )

        # Apply time parameterization settings from task config
        if hasattr(self.planner, "configure_time_parameterization"):
            tp_kwargs = {}
            for field, kwarg in (
                ("TIME_PARAM_SAFETY", "safety"),
                ("TIME_PARAM_ORDER", "order"),
            ):
                val = getattr(self.task_config, field, None)
                if val is not None:
                    tp_kwargs[kwarg] = val
            if tp_kwargs:
                self.planner.configure_time_parameterization(**tp_kwargs)
                if verbose:
                    logger.info("Configured time parameterization: %s", tp_kwargs)

        # Apply time parameterization method (stp / trapezoidal / toppra)
        if hasattr(self.planner, "configure_time_parameterization_method"):
            tp_method_kwargs = {}
            for field, kwarg in (
                ("TIME_PARAM_METHOD", "method"),
                ("TOPPRA_VELOCITY_SCALE", "toppra_velocity_scale"),
                ("TOPPRA_EFFORT_SCALE", "toppra_effort_scale"),
                ("TOPPRA_SOLVER", "toppra_solver"),
                ("TOPPRA_N", "toppra_N"),
                ("TOPPRA_INTERPOLATION", "toppra_interpolation"),
                ("TOPPRA_GRIDPOINT_METHOD", "toppra_gridpoint_method"),
                ("TOPPRA_ACTIVE_JOINTS", "toppra_active_joints"),
            ):
                val = getattr(self.task_config, field, None)
                if val is not None:
                    tp_method_kwargs[kwarg] = val
            if tp_method_kwargs:
                self.planner.configure_time_parameterization_method(**tp_method_kwargs)
                if verbose:
                    logger.info(
                        "Configured time parameterization method: %s",
                        tp_method_kwargs,
                    )

        if verbose:
            print("\n" + "=" * 70)
            logger.info("Grasp Sequence Planning")
            print("=" * 70)
            logger.info("Sequence: %s", grasp_sequence)
            logger.info(
                "Initial state: %s", self.grasp_tracker.get_current_state_name()
            )
            logger.info("Press Ctrl+C to stop gracefully (saves progress)")

        # Enable graceful stop and clear any previous stop request
        clear_stop_request()
        enable_graceful_stop()

        self.phase_results = []
        self.original_sequence = list(grasp_sequence)  # Store for resume
        # Fresh call, fresh hints -- but NOT reset in resume_sequence(), which
        # continues this same call's block and must keep a break that already
        # happened visible to the caller across every resume attempt.
        self.invalidated_phase_hints = set()
        # Grasps already held when this sequence started, i.e. established by
        # EARLIER plan_sequence() calls on this same planner. resume_sequence()
        # rebuilds the tracker from scratch and can only replay
        # self.phase_results, which is reset above and therefore only ever
        # describes the current call -- so without this snapshot every grasp
        # from a previous call is silently lost on the first resume.
        #
        # That is not hypothetical: in screwdriving_sequence.py, which
        # drives the run as a series of separate plan_sequence() blocks, the
        # two tool grasps (g_ur10_tool/h_FG_tool and g_vispa_tool/h_SD_tool)
        # are established during bootstrap and every later block resumed into
        # a state that had dropped them. The rebuilt phase graph then treats
        # frame_gripper and screw_driver as free-floating objects, builds a
        # structurally different edge for the same phase
        # ('...CON0 | 0-1:1-0_01' instead of '...CON0 | 0-0:1-1:2-3:3-2_01'),
        # and target generation becomes unsatisfiable rather than merely
        # hard: 1095 consecutive random restarts each terminated at the
        # identical residual 9.783801504932926 against a 1e-4 threshold.
        # Net effect was that any block failing once could never recover,
        # however long it retried.
        self._initial_grasps = {
            g: h for g, h in self.grasp_tracker.current_grasps.items() if h is not None
        }
        q_current = list(q_init)
        # Where this call started. resume_sequence restarts a failed first
        # phase from here rather than from wherever the failed attempt
        # stopped — a search must not move the robot.
        self._q_call_start = list(q_init)
        # Original scene configuration — used to restore free-object positions
        # before each phase graph build so LockedJoint foliation locks them at
        # their true scene positions, not at random IK-sampled positions.
        # Defaults to q_init; callers starting mid-sequence from a
        # checkpointed q_current must pass the true scene config explicitly.
        _q_scene_init = list(q_scene_init) if q_scene_init is not None else list(q_init)
        self._q_scene_init = _q_scene_init  # store for resume_planning

        # Emit sequence_start with all call parameters before iteration begins.
        if self.run_logger is not None:
            try:
                self.run_logger.log(
                    "sequence_start",
                    grasp_sequence=[[g, h] for g, h in grasp_sequence],
                    q_init=list(q_init),
                    validate=validate,
                    max_iterations_per_edge=max_iterations_per_edge,
                    timeout_per_edge=timeout_per_edge,
                    frozen_arms_mode=frozen_arms_mode,
                    time_parameterize=time_parameterize,
                    reset_roadmap=reset_roadmap,
                )
            except Exception:
                pass
        _loop_start_time = time.time()

        q_current = self._run_phase_loop(
            phases=grasp_sequence,
            starting_phase_idx=0,
            total_phase_count_for_display=len(grasp_sequence),
            q_current=q_current,
            frozen_arms_mode=frozen_arms_mode,
            per_phase_frozen_arms=per_phase_frozen_arms,
            q_scene_init=_q_scene_init,
            skip_phases=skip_phases,
            is_resume=False,
            verbose=verbose,
            loop_start_time=_loop_start_time,
            phase_q_hints=phase_q_hints,
        )

        if verbose:
            print("\n" + "=" * 70)
            logger.info("Sequence Planning Complete")
            print("=" * 70)
            logger.info("Final state: %s", self.grasp_tracker.get_current_state_name())
            logger.info("Completed %d phases", len(self.phase_results))
            logger.info("Total planning time: %.2fs", self.total_planning_time)

        # Disable signal handler before returning
        disable_graceful_stop()

        # Emit run_end and close logger (success path).
        if self.run_logger is not None:
            try:
                self.run_logger.log(
                    "run_end",
                    success=True,
                    total_time=time.time() - _loop_start_time,
                    total_planning_time=self.total_planning_time,
                    phase_count=len(self.phase_results),
                    final_config=list(q_current),
                    error=None,
                )
                self.run_logger.close()
            except Exception:
                pass

        # Return combined result
        return {
            "success": True,
            "paths": (self.phase_results[-1]["paths"] if self.phase_results else []),
            "phase_results": self.phase_results,
            "final_config": q_current,
            "grasp_tracker": self.grasp_tracker,
        }

    def get_resumable_state(self) -> dict[str, Any] | None:
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
            "edge_idx": incomplete_phase.get("failed_edge_idx", 0),
            "edge_name": incomplete_phase.get(
                "failed_edge_name",
                incomplete_phase.get("error_message", "unknown"),
            ),
            "completed_phases": completed_phases,
            "completed_edges_in_phase": len(incomplete_phase.get("paths", [])),
            "total_edges_in_phase": len(incomplete_phase.get("edges", [])),
            "q_current": incomplete_phase.get("last_q_start"),
            "error": incomplete_phase.get("error_message", "unknown"),
        }

    def reset_grasp_tracker_to_call_start(self) -> None:
        """Roll ``self.grasp_tracker`` back to where the current
        ``plan_sequence()`` call began, discarding the grasps its completed
        phases committed.

        For callers that need to re-plan a block from its start rather than
        resume forward into it -- e.g. ``run_block_nonstop()`` re-running
        the lookahead after a phase's hint chain broke. Resuming keeps those
        commitments (that's the point of a resume); replanning must not, or
        the rebuilt phase graph would see the block's own grasps as already
        held and plan a structurally different, unsatisfiable sequence.
        """
        self._restore_grasp_tracker_for_resume(replay_completed_phases=False)

    def _restore_grasp_tracker_for_resume(
        self, replay_completed_phases: bool = True
    ) -> None:
        """Rebuild ``self.grasp_tracker`` to the state a resume must start in.

        Seeds from ``self._initial_grasps`` -- whatever was already held when
        the current ``plan_sequence()`` call began -- then (unless
        ``replay_completed_phases`` is False, see
        ``reset_grasp_tracker_to_call_start()``) replays this call's
        completed phases on top.

        Seeding matters because the replay can only see
        ``self.phase_results``, which ``plan_sequence()`` resets and which
        therefore only ever describes the current call. Starting from the
        all-free state instead (the original behaviour) silently drops every
        grasp established by an earlier ``plan_sequence()`` call on the same
        planner, which is exactly how a multi-block driver like
        screwdriving_sequence.py lost its two bootstrap tool grasps on
        the first resume of any later block.
        """
        grippers = self.grasp_tracker.grippers
        # Every gripper must appear as a key, free ones mapped to None:
        # GraspStateTracker takes initial_grasps as the WHOLE map rather than
        # as an overlay, so passing only the held pairs would leave every
        # other gripper absent and make update_grasp() raise "Unknown
        # gripper" as soon as the replay below touched one.
        seeded = {g: None for g in grippers}
        seeded.update(getattr(self, "_initial_grasps", {}) or {})
        self.grasp_tracker = GraspStateTracker(
            grippers=grippers,
            handles=self.grasp_tracker.handles,
            initial_grasps=seeded,
        )

        if not replay_completed_phases:
            return

        # Replay completed grasps to restore state.
        # Each completed phase may involve an implicit auto-release
        # (gripper switches from handle A to handle B without an explicit
        # release entry).  Handle that gracefully:
        for phase in self.phase_results:
            if phase.get("complete", False):
                g = phase["gripper"]
                h = phase["handle"]
                currently = self.grasp_tracker.current_grasps.get(g)
                if h is None:
                    # Explicit release
                    if currently is not None:
                        self.grasp_tracker.update_grasp(g, None)
                    # else: already free, nothing to do
                else:
                    if currently is not None and currently != h:
                        # Implicit auto-release happened in this phase
                        self.grasp_tracker.update_grasp(g, None)
                    if self.grasp_tracker.current_grasps.get(g) is None:
                        self.grasp_tracker.update_grasp(g, h)

    def resume_sequence(
        self,
        retry_from_edge: int = 0,
        max_iterations_per_edge: int | None = None,
        timeout_per_edge: float | None = None,
        frozen_arms_mode: str | None = None,
        per_phase_frozen_arms: dict[int, list[str]] | None = None,
        skip_phases: set[int] | None = None,
        validate: bool = True,
        reset_roadmap: bool = True,
        time_parameterize: bool = True,
        verbose: bool = True,
        phase_q_hints: dict[int, list[list[float]] | list[float]] | None = None,
    ) -> dict[str, Any]:
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
            phase_q_hints: Optional ``{phase_idx: hint}`` map, keyed by the
                ABSOLUTE phase index within ``self.original_sequence`` (not
                re-based to this call's ``remaining_sequence``) -- see
                ``plan_sequence()``'s and ``_plan_phase_edges()``'s
                docstrings. Resuming into a phase this dict has an entry for
                keeps steering it toward the known-good candidate instead of
                reverting to blind random restarts on every retry.

                A hint for a phase this resume starts *after* is inert:
                that phase is already committed, so resuming cannot undo a
                bad commitment. When ``self.invalidated_phase_hints``
                names such an earlier phase, retrying here is the blind
                hammering the lookahead exists to prevent -- re-run
                ``find_feasible_phase_target()`` and re-plan the block from
                its start instead (``run_block_nonstop()`` in
                screwdriving_sequence.py does exactly this).

        Returns:
            Same as plan_sequence(). On success, also emits a ``"run_end"``
            RunLogger event and closes the logger, same as plan_sequence()
            does on its own success -- this is the call that actually
            brings a resumed run to completion (fixed 2026-08-09; see the
            codebase refactor plan's logging-asymmetry section).

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
            logger.info(
                "Resuming Grasp Sequence Planning (attempt #%d)",
                self.resume_attempt_count,
            )
            print("=" * 70)
            logger.info(
                "Resuming from Phase %d, Edge %d",
                resume_state["phase_idx"] + 1,
                resume_state["edge_idx"] + 1,
            )
            logger.info("Previous error: %s", resume_state["error"])
            logger.info("Completed phases: %s", resume_state["completed_phases"])
            logger.info(
                "Completed edges in current phase: %s",
                resume_state["completed_edges_in_phase"],
            )
            logger.info("Press Ctrl+C to stop gracefully (saves progress)")

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
                    logger.info("Updated TransitionPlanner config: %s", kwargs)

        self._restore_grasp_tracker_for_resume()

        if verbose:
            logger.info(
                "Restored state: %s", self.grasp_tracker.get_current_state_name()
            )

        # Remove incomplete phase from results (will be recreated)
        incomplete_phase_idx = resume_state["phase_idx"]
        self.phase_results = [p for p in self.phase_results if p.get("complete", True)]

        # Get remaining sequence starting from failed phase
        remaining_sequence = self.original_sequence[incomplete_phase_idx:]

        if verbose:
            logger.info("Remaining sequence: %s", remaining_sequence)

        # Determine starting config
        if self.phase_results:
            # Continue from last completed phase
            q_current = self.phase_results[-1]["final_config"]
        else:
            # Nothing completed, so there is no phase boundary to continue
            # from: restart from where this plan_sequence call began. The
            # failed attempt's own progress (resume_state["q_current"], i.e.
            # last_q_start) is not a boundary — resuming from it makes every
            # retry fly the arm to wherever the last attempt gave up, so a
            # failing grasp accumulates real motion it never needed. Every
            # caller retries from edge 0 of the phase, so the call's start
            # config is the correct input for that edge.
            q_current = getattr(self, "_q_call_start", None)
            if q_current is None:
                # resume_sequence reached without a prior plan_sequence.
                q_current = resume_state["q_current"]

        use_fm = frozen_arms_mode if frozen_arms_mode is not None else "global"
        _resume_loop_start_time = time.time()
        q_current = self._run_phase_loop(
            phases=remaining_sequence,
            starting_phase_idx=incomplete_phase_idx,
            total_phase_count_for_display=len(self.original_sequence),
            q_current=q_current,
            frozen_arms_mode=use_fm,
            per_phase_frozen_arms=per_phase_frozen_arms,
            q_scene_init=getattr(self, "_q_scene_init", None),
            skip_phases=skip_phases,
            is_resume=True,
            verbose=verbose,
            retry_from_edge=retry_from_edge,
            completed_edges_in_phase_for_resume=resume_state[
                "completed_edges_in_phase"
            ],
            loop_start_time=_resume_loop_start_time,
            phase_q_hints=phase_q_hints,
        )

        # Clear failure info on success
        self.last_failure_info = None

        if verbose:
            print("\n" + "=" * 70)
            logger.info("Resume Complete - All Phases Succeeded")
            logger.info("Total planning time: %.2fs", self.total_planning_time)
            logger.info("Resume attempts: %d", self.resume_attempt_count)
            print("=" * 70)

        # Disable signal handler before returning
        disable_graceful_stop()

        # Emit run_end and close the logger (success path) -- plan_sequence()
        # does this on its own success; resume_sequence() never did, even
        # though it's the call that actually brings a resumed run to
        # completion, leaving the log dangling open forever on the resume
        # path (logging-asymmetry fix, 2026-08-09). total_time here covers
        # only this resume call, not the original (failed) plan_sequence()
        # call before it -- that start time isn't retained across calls.
        if self.run_logger is not None:
            try:
                self.run_logger.log(
                    "run_end",
                    success=True,
                    total_time=time.time() - _resume_loop_start_time,
                    total_planning_time=self.total_planning_time,
                    phase_count=len(self.phase_results),
                    final_config=list(q_current),
                    error=None,
                )
                self.run_logger.close()
            except Exception:
                pass

        return {
            "success": True,
            "paths": (self.phase_results[-1]["paths"] if self.phase_results else []),
            "phase_results": self.phase_results,
            "final_config": q_current,
            "grasp_tracker": self.grasp_tracker,
        }

    def replay_sequence(
        self,
        speed: float = 1.0,
        clear_paths_first: bool = False,
        visualizer: Any | None = None,
        record: bool = False,
        output_dir: str | None = None,
        video_prefix: str | None = None,
        framerate: int = 25,
        dt: float = 0.01,
    ) -> list[str] | None:
        """Replay all phase paths in sequence.

        Args:
            speed: Playback speed multiplier
            clear_paths_first: If True, warn about accumulated paths before replay
            visualizer: Optional LiveConstraintGraphVisualizer for real-time graph updates
            record: If True, record video of the replay (default: True)
            output_dir: Directory for video output. Defaults to
                ``long_tamp.visualization.default_video_output_dir()``
                when not given.
            video_prefix: Optional prefix for video filenames
            framerate: Video framerate in fps (default: 25)
            dt: Time step for path sampling (default: 0.01)

        Returns:
            List of video file paths if recording enabled, None otherwise
        """
        if not self.phase_results:
            logger.warning("No phases to replay (run plan_sequence first)")
            return

        if record:
            from long_tamp.visualization import default_video_output_dir

            output_dir = output_dir or default_video_output_dir()

        print("\n" + "=" * 70)
        logger.info("Replaying Grasp Sequence")
        if visualizer:
            logger.info("Live graph visualization: ENABLED")
        if record:
            logger.info("Video recording: ENABLED (output: %s)", output_dir)
        print("=" * 70)

        recorded_videos = []

        # Check for path accumulation
        if hasattr(self.planner, "get_num_stored_paths"):
            num_stored = self.planner.get_num_stored_paths()
            if num_stored > 0:
                logger.info(
                    "Note: %d paths already stored in ProblemSolver", num_stored
                )
                logger.info("Replay will add more paths (hpp has no clear API)")
                if clear_paths_first:
                    logger.info("Consider restarting to clear memory")

        for phase in self.phase_results:
            is_complete = phase.get("complete", True)
            status = "✓" if is_complete else "⚠ INCOMPLETE"

            logger.info(
                "Phase %s: %s grasps %s [%s]",
                phase["phase"],
                phase["gripper"],
                phase["handle"],
                status,
            )
            logger.info("Edges: %s", ", ".join(phase["edges"]))

            # Check if phase was skipped
            if phase.get("skipped"):
                logger.info("⏭ Phase skipped (no paths to replay)")
                continue

            if not is_complete:
                failed_edge = phase.get("failed_edge_idx", -1)
                logger.warning(
                    "Failed at edge %d: %s",
                    failed_edge + 1,
                    phase.get("failed_edge_name", "unknown"),
                )
                logger.warning("Error: %s", phase.get("error_message", "unknown"))

            # Filter out None paths from skipped edges
            valid_paths = [
                (idx, path)
                for idx, path in enumerate(phase["paths"])
                if path is not None
            ]
            logger.info("Playing %d waypoint paths...", len(valid_paths))

            try:
                # Play each waypoint path in the sequence
                edge_names = phase.get("edges", [])
                for idx, path in valid_paths:
                    edge_name = edge_names[idx] if idx < len(edge_names) else None

                    logger.info("Path %d/%d:", idx + 1, len(phase["paths"]))

                    video_file = self._play_single_phase_path(
                        path=path,
                        edge_name=edge_name,
                        phase=phase,
                        idx=idx,
                        record=record,
                        visualizer=visualizer,
                        output_dir=output_dir,
                        video_prefix=video_prefix,
                        framerate=framerate,
                        dt=dt,
                        speed=speed,
                    )
                    if video_file is not None:
                        recorded_videos.append(video_file)
            except Exception as e:
                logger.warning("Failed to replay: %s", e)

        # Final path count
        if hasattr(self.planner, "get_num_stored_paths"):
            final_count = self.planner.get_num_stored_paths()
            print("=" * 70)
            logger.info("Total paths now in ProblemSolver: %d", final_count)

        if record and recorded_videos:
            logger.info("📹 Recorded %d videos to %s", len(recorded_videos), output_dir)
            return recorded_videos
        return None

    def _play_single_phase_path(
        self,
        path,
        edge_name: str | None,
        phase: dict,
        idx: int,
        record: bool,
        visualizer,
        output_dir: str,
        video_prefix: str | None,
        framerate: int,
        dt: float,
        speed: float,
    ):
        """Play back a single waypoint path, dispatching on backend capability.

        4-way dispatch mirroring the original inline block in replay_sequence():
        record via play_and_record_path_vector → visualizer via
        play_path_vector_with_viz → plain play_path_vector → unsupported-backend
        warning. Pure legibility split (no second caller); the phase-iteration
        loop and the before/after get_num_stored_paths() bookkeeping stay in
        replay_sequence().

        Args:
            path: The path vector to play.
            edge_name: Edge name for this path (for visualization-enabled
                playback), or None.
            phase: The phase_results dict this path belongs to (used to build
                the video filename).
            idx: 0-based path index within the phase (used for the video
                filename).
            record: If True, prefer the record-and-play branch.
            visualizer: Optional visualizer for visualization-enabled playback.
            output_dir: Directory for video output.
            video_prefix: Optional prefix for video filenames.
            framerate: Video framerate in fps.
            dt: Time step for path sampling.
            speed: Playback speed multiplier.

        Returns:
            The recorded video file path if the record branch ran, else None
            (caller appends non-None results to its recorded_videos list).
        """
        # Generate video name if recording
        video_name_for_path = None
        if record:
            if video_prefix:
                video_name_for_path = (
                    f"{video_prefix}_phase_{phase['phase']:02d}_path_{idx + 1:02d}"
                )
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
            logger.info("✓ Recorded (index %s): %s", path_idx, video_file)
            return video_file
        elif visualizer and hasattr(self.planner, "play_path_vector_with_viz"):
            # Use visualization-enabled playback
            path_idx = self.planner.play_path_vector_with_viz(
                path,
                edge_name=edge_name,
                visualizer=visualizer,
                speed=speed,
            )
            logger.info("✓ Played with visualization (stored as index %s)", path_idx)
        elif hasattr(self.planner, "play_path_vector"):
            # Standard playback without visualization
            path_idx = self.planner.play_path_vector(path, speed=speed)
            logger.info("✓ Played (stored as index %s)", path_idx)
        else:
            logger.warning("⚠ Backend does not support PathVector playback")
            logger.warning("Path type: %s", type(path).__name__)
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
                    attempt_str = f" (attempt #{attempt})" if attempt > 1 else ""
                    resume_str = " [resume]" if is_resume else ""

                    lines.append(
                        f"    {status_icon} Edge {edge_idx + 1}: "
                        f"{edge_name}{attempt_str}{resume_str}"
                    )
                    if is_skipped:
                        lines.append(f"       skipped (gen: {gen_t:.2f}s)")
                    else:
                        lines.append(
                            f"       gen: {gen_t:.2f}s | "
                            f"plan: {plan_t:.2f}s | total: {total_t:.2f}s"
                        )
            else:
                # Fallback for phases without edge_stats
                lines.append(f"  Edges: {', '.join(phase['edges'])}")

            if not is_complete:
                failed_edge = phase.get("failed_edge_idx", -1)
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
        freeze_joint_substrings: list[str] | None = None,
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
        self.ALL_ARM_KEYWORDS = list(getattr(task_config, "ALL_ARM_KEYWORDS", []))

        # Will be populated during run()
        self.grasp_sequence: list[tuple[str, str]] = []
        self.skip_phases: set[int] = set()
        self.skip_all_phases: bool = False
        self.frozen_arms_mode: str = "auto"
        self.per_phase_frozen_arms: dict[int, list[str]] | None = None
        self.auto_save_dir: str | None = None
        self.non_stop: bool = False

    def _get_available_grasps(self) -> list[tuple[str, str]]:
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
        from long_tamp.utils.interactive import interactive_menu

        all_grasps = self._get_available_grasps()
        if not all_grasps:
            print("No valid grasps available in config.")
            return False

        # Format for display
        grasp_options = [f"{gripper} → {handle}" for gripper, handle in all_grasps] + [
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
        from long_tamp.cli.interactive_pickers import select_skip_phases

        if len(self.grasp_sequence) >= 1:
            self.skip_phases, self.skip_all_phases = select_skip_phases(
                self.grasp_sequence
            )

    def configure_frozen_arms(self) -> None:
        """Configure frozen arms mode."""
        from long_tamp.cli.interactive_pickers import select_frozen_arms_mode

        self.frozen_arms_mode, self.per_phase_frozen_arms = select_frozen_arms_mode(
            self.grasp_sequence,
            arm_keywords=self.ALL_ARM_KEYWORDS,
        )

    def configure_auto_save(self) -> None:
        """Configure auto-save directory."""
        from long_tamp.cli.interactive_pickers import select_auto_save_directory

        self.auto_save_dir = select_auto_save_directory()

    def _create_arm_selector_callback(self):
        """Create interactive arm selector callback for interactive mode."""
        from long_tamp.utils.interactive import interactive_menu

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

    def run(self) -> dict[str, Any]:
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
        planner = None
        try:
            planner = GraspSequencePlanner(
                graph_builder=self.task.graph_builder,
                config_gen=self.task.config_gen,
                planner=self.task.planner,
                task_config=self.task_config,
                backend=self.task.backend,
                graph_constraints=getattr(self.task, "_graph_constraints", None),
                auto_save_dir=self.auto_save_dir,
                run_logger=getattr(self.task, "run_logger", None),
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

            # In non-stop mode, auto-resume after failure
            if planner is not None:
                self._handle_failure(planner, q_init)
                self._show_saved_files_summary(planner)

                # Check if auto-resume succeeded
                resume_state = planner.get_resumable_state()
                if resume_state is None:
                    # No resumable state = all phases completed
                    return {
                        "success": True,
                        "planner": planner,
                    }

            return {"success": False, "error": str(e), "planner": planner}

    def _ensure_task_ready(self) -> bool:
        """Ensure task is set up for planning."""
        if not hasattr(self.task, "graph_builder") or self.task.graph_builder is None:
            print("Error: Task graph_builder not initialized.")
            return False
        if not hasattr(self.task, "planner") or self.task.planner is None:
            print("Error: Task planner not initialized.")
            return False
        return True

    def _get_q_init(self) -> list[float] | None:
        """Get initial configuration."""
        if hasattr(self.task, "config_gen") and self.task.config_gen is not None:
            q_init = self.task.config_gen.configs.get("q_init")
            if q_init is not None:
                return q_init
        return getattr(self.task, "q_init", None)

    def _handle_failure(
        self, planner: GraspSequencePlanner, q_init: list[float] | None = None
    ) -> None:
        """Handle planning failure with optional resume.

        Args:
            planner: The GraspSequencePlanner instance.
            q_init: Initial configuration for replay/browse after resume.
        """

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
        self, planner: GraspSequencePlanner, q_init: list[float] | None = None
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
        self, planner: GraspSequencePlanner, q_init: list[float] | None = None
    ) -> None:
        """Interactive resume loop with menu options.

        Args:
            planner: The GraspSequencePlanner instance.
            q_init: Initial configuration for replay/browse after success.
        """
        from long_tamp.utils.interactive import interactive_menu

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
        self, planner: GraspSequencePlanner, q_init: list[float]
    ) -> dict[str, list[float]]:
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
                if planner.config_gen and config_label in planner.config_gen.configs:
                    # Create friendly name from edge name
                    edge_type = "waypoint"
                    if "pregrasp" in edge_name.lower():
                        edge_type = "pregrasp"
                    elif (
                        "grasp" in edge_name.lower() and "pre" not in edge_name.lower()
                    ):
                        edge_type = "grasp"
                    elif "placement" in edge_name.lower():
                        edge_type = "placement"

                    friendly_name = f"{phase_label} - Edge {edge_idx + 1} ({edge_type})"
                    generated_configs[friendly_name] = planner.config_gen.configs[
                        config_label
                    ]

            # Add final config for this phase
            if "final_config" in phase_result:
                final_label = f"{phase_label} - Final"
                generated_configs[final_label] = phase_result["final_config"]

        return generated_configs

    def _offer_replay_or_browse(
        self, planner: GraspSequencePlanner, q_init: list[float] | None
    ) -> None:
        """Offer replay for non-skipped phases or browse for skipped phases.

        Args:
            planner: The GraspSequencePlanner instance.
            q_init: Initial configuration for browsing.
        """
        from long_tamp.cli.interactive_pickers import browse_configurations
        from long_tamp.utils.interactive import interactive_menu

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

                    if not phase_selected or phase_selected[0] >= len(
                        planner.phase_results
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
