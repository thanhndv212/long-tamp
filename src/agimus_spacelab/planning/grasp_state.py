#!/usr/bin/env python3
"""
Grasp state tracking and edge computation for sequential planning.

Tracks current grasps and computes HPP constraint graph edge names based on
the factory naming convention:
- Grasp forward: "{gripper} > {handle} | {abbrev_state}"
- Release backward: "{gripper} < {handle} | {abbrev_state}"
- Loop (same state): "Loop | {abbrev_state}"
"""

from __future__ import annotations

from typing import Dict, List, Optional


class GraspStateTracker:
    """Track current grasp state and compute edge names for transitions.

    The constraint graph factory uses a specific naming convention for edges:
    - States are encoded as abbreviated grasp tuples: "0-1:2-3" means
      gripper[0] holds handle[1], gripper[2] holds handle[3]
    - "f" means free (no grasps)
    - Edges encode the transition and source state

    This class maintains the current grasp state and provides methods to:
    - Compute edge names for grasp/release transitions
    - Update state after successful planning
    - Query current state for planning decisions
    """

    def __init__(
        self,
        grippers: List[str],
        handles: List[str],
        initial_grasps: Optional[Dict[str, Optional[str]]] = None,
    ):
        """Initialize grasp state tracker.

        Args:
            grippers: List of gripper names (must match factory order)
            handles: List of handle names (must match factory order)
            initial_grasps: Initial grasp state {gripper: handle or None}
                           If None, assumes all grippers are free
        """
        self.grippers = grippers
        self.handles = handles
        
        # Store gripper order to match ConstraintGraphFactory ordering
        # This is critical for state name generation
        self.gripper_order = list(grippers)

        # Build index mappings for abbreviated state encoding
        self.gripper_to_idx = {g: i for i, g in enumerate(grippers)}
        self.handle_to_idx = {h: i for i, h in enumerate(handles)}

        # Phase-local index overrides (set per-phase via set_phase_indices)
        self._phase_gripper_to_idx = None
        self._phase_handle_to_idx = None

        # Current grasp state: gripper -> handle (or None if free)
        if initial_grasps is None:
            self.current_grasps = {g: None for g in grippers}
        else:
            self.current_grasps = dict(initial_grasps)
            # Validate initial state
            for gripper, handle in self.current_grasps.items():
                if gripper not in self.gripper_to_idx:
                    raise ValueError(f"Unknown gripper: {gripper}")
                if handle is not None and handle not in self.handle_to_idx:
                    raise ValueError(f"Unknown handle: {handle}")

    def set_phase_indices(self, phase_grippers, phase_handles):
        """Set phase-local index mappings for edge name generation.

        Must be called after each build_phase_graph so that abbreviated
        state indices match the ConstraintGraphFactory ordering.
        """
        self._phase_gripper_to_idx = {
            g: i for i, g in enumerate(phase_grippers)
        }
        self._phase_handle_to_idx = {
            h: i for i, h in enumerate(phase_handles)
        }

    def _get_abbreviated_state(
        self, grasps: Optional[Dict[str, Optional[str]]] = None
    ) -> str:
        """Get abbreviated state string for given grasp configuration.

        Args:
            grasps: Grasp configuration {gripper: handle or None}
                   If None, uses current_grasps

        Returns:
            Abbreviated state string, e.g. "0-1:2-3" or "f" for free
        """
        if grasps is None:
            grasps = self.current_grasps

        # Build list of (gripper_idx, handle_idx) for held grasps
        # Use phase-local indices when available (set by set_phase_indices)
        if self._phase_gripper_to_idx is not None:
            g_idx_map = self._phase_gripper_to_idx
        else:
            import warnings
            warnings.warn(
                "Phase-local gripper indices not set — using global indices. "
                "Call set_phase_indices() after build_phase_graph() to ensure "
                "edge names match the ConstraintGraphFactory ordering.",
                stacklevel=2,
            )
            g_idx_map = self.gripper_to_idx

        if self._phase_handle_to_idx is not None:
            h_idx_map = self._phase_handle_to_idx
        else:
            import warnings
            warnings.warn(
                "Phase-local handle indices not set — using global indices. "
                "Call set_phase_indices() after build_phase_graph() to ensure "
                "edge names match the ConstraintGraphFactory ordering.",
                stacklevel=2,
            )
            h_idx_map = self.handle_to_idx

        held = []
        for gripper, handle in grasps.items():
            if handle is not None:
                g_idx = g_idx_map[gripper]
                h_idx = h_idx_map[handle]
                held.append((g_idx, h_idx))

        if not held:
            return "f"  # Free state

        # Sort by gripper index for canonical ordering
        held.sort(key=lambda x: x[0])

        # Format as "g0-h0:g1-h1:..."
        parts = [f"{g_idx}-{h_idx}" for g_idx, h_idx in held]
        return ":".join(parts)

    def get_grasp_edge(self, gripper: str, handle: str) -> str:
        """Get edge name for grasping a handle with a gripper.

        Args:
            gripper: Gripper name
            handle: Handle name to grasp

        Returns:
            Edge name following factory convention

        Raises:
            ValueError: If gripper already holds something or is unknown
        """
        if gripper not in self.current_grasps:
            raise ValueError(f"Unknown gripper: {gripper}")

        if self.current_grasps[gripper] is not None:
            current = self.current_grasps[gripper]
            raise ValueError(
                f"Gripper '{gripper}' already holds '{current}'. "
                f"Release it before grasping '{handle}'."
            )

        if handle not in self.handle_to_idx:
            raise ValueError(f"Unknown handle: {handle}")

        # Check if handle is already grasped by another gripper
        for g, h in self.current_grasps.items():
            if h == handle and g != gripper:
                raise ValueError(
                    f"Handle '{handle}' is already grasped by '{g}'. "
                    f"Only one gripper can hold a handle at a time."
                )

        # Grasp edge format: "{gripper} > {handle} | {current_state_abbrev}"
        state_abbrev = self._get_abbreviated_state()
        return f"{gripper} > {handle} | {state_abbrev}"

    def get_grasp_edge_sequence(self, gripper: str, handle: str) -> list:
        """Get waypoint edge sequence for grasping.

        Returns edges through waypoints for smoother constraint
        satisfaction.

        Args:
            gripper: Gripper name
            handle: Handle name to grasp

        Returns:
            List of edge names [pregrasp_edge, grasp_edge] for
            waypoint sequence

        Raises:
            ValueError: If gripper already holds something or is unknown
        """
        # Validate inputs using get_grasp_edge (will raise if invalid)
        base_edge = self.get_grasp_edge(gripper, handle)
        
        # Factory creates waypoint edges with _01, _12 suffixes
        # These represent: free -> pregrasp (_01) -> grasp (_12)
        return [f"{base_edge}_01", f"{base_edge}_12"]

    def get_pregrasp_node_name(self, gripper: str) -> str:
        """Return the pregrasp waypoint node name shared by the grasp/release sub-edges.

        The factory names the waypoint:
            "{gripper} > {handle} | {state_after_release}_pregrasp"
        where *state_after_release* is the abbreviated state WITHOUT this
        gripper's pair — i.e. the state the robot will be in once released.

        Args:
            gripper: Gripper name currently holding an object.

        Returns:
            Node name string, e.g.
            "frame_gripper/g_FG_part > RS1/h_RS1_FG | 0-0:1-2_pregrasp"

        Raises:
            ValueError: If gripper is unknown or not currently holding anything.
        """
        if gripper not in self.current_grasps:
            raise ValueError(f"Unknown gripper: {gripper}")
        handle = self.current_grasps[gripper]
        if handle is None:
            raise ValueError(f"Gripper '{gripper}' is not holding anything")

        # State after release: remove this gripper's pair from the snapshot.
        grasps_after = {g: h for g, h in self.current_grasps.items() if g != gripper}
        grasps_after[gripper] = None
        state_abbrev = self._get_abbreviated_state(grasps_after)

        return f"{gripper} > {handle} | {state_abbrev}_pregrasp"

    def get_approach_edge_from_released(self, gripper: str) -> str:
        """Return the _01 (approach) edge name treating this gripper as free.

        Used during release planning: we generate the pregrasp config via
        the FORWARD approach edge (released_state → pregrasp waypoint) rather
        than via generateTargetConfig on the _21 edge.  The _21 path-fold
        encodes "gripper at contact" (RHS from q_grasped) while the pregrasp
        leaf requires "gripper at approach offset" — conflicting requirements
        that make the solver diverge.  The _01 edge has no such conflict: its
        fold only encodes the OTHER grasps, so the pregrasp leaf is free.

        Args:
            gripper: Gripper name currently holding an object.

        Returns:
            Edge name string, e.g.
            "frame_gripper/g_FG_part > RS1/h_RS1_FG | 0-0:1-2_01"

        Raises:
            ValueError: If gripper is unknown or not currently holding anything.
        """
        if gripper not in self.current_grasps:
            raise ValueError(f"Unknown gripper: {gripper}")
        handle = self.current_grasps[gripper]
        if handle is None:
            raise ValueError(f"Gripper '{gripper}' is not holding anything")

        grasps_after = {
            g: h for g, h in self.current_grasps.items() if g != gripper
        }
        grasps_after[gripper] = None
        state_abbrev = self._get_abbreviated_state(grasps_after)
        return f"{gripper} > {handle} | {state_abbrev}_01"

    def get_release_edge(self, gripper: str) -> str:
        """Get edge name for releasing the object held by a gripper.

        Args:
            gripper: Gripper name

        Returns:
            Edge name following factory convention

        Raises:
            ValueError: If gripper is not holding anything or is unknown
        """
        if gripper not in self.current_grasps:
            raise ValueError(f"Unknown gripper: {gripper}")

        handle = self.current_grasps[gripper]
        if handle is None:
            raise ValueError(f"Gripper '{gripper}' is not holding anything")

        # Release edge format: "{gripper} < {handle} | {containing_state}"
        # HPP factory convention: release edges (including _21/_10 waypoints)
        # are annotated with the CONTAINING state = the full grasp state
        # BEFORE release (i.e., including this gripper's hold).
        state_abbrev = self._get_abbreviated_state()

        return f"{gripper} < {handle} | {state_abbrev}"

    def get_release_edge_sequence(self, gripper: str) -> list:
        """Get waypoint edge sequence for releasing.

        Returns edges through waypoints for smoother constraint
        satisfaction.

        Args:
            gripper: Gripper name

        Returns:
            List of edge names [prerelease_edge, release_edge] for
            waypoint sequence

        Raises:
            ValueError: If gripper is not holding anything or is unknown
        """
        # Validate inputs using get_release_edge (will raise if invalid)
        base_edge = self.get_release_edge(gripper)
        
        # Factory creates waypoint edges with _21, _10 suffixes for release
        # These represent: grasp -> prerelease (_21) -> free (_10)
        return [f"{base_edge}_21", f"{base_edge}_10"]

    def get_loop_edge(self) -> str:
        """Get loop edge name for moving within current grasp state.

        Loop edges allow motion within the same grasp configuration, e.g.,
        repositioning while maintaining all grasps.

        Returns:
            Loop edge name following factory convention
        """
        state_abbrev = self._get_abbreviated_state()
        return f"Loop | {state_abbrev}"

    def update_grasp(self, gripper: str, handle: Optional[str] = None) -> None:
        """Update grasp state after successful planning.

        Args:
            gripper: Gripper name
            handle: Handle to grasp (or None to release)

        Raises:
            ValueError: If gripper is unknown or update is invalid
        """
        if gripper not in self.current_grasps:
            raise ValueError(f"Unknown gripper: {gripper}")

        if handle is not None:
            # Grasping: check gripper is free
            if self.current_grasps[gripper] is not None:
                raise ValueError(
                    f"Cannot grasp '{handle}': gripper '{gripper}' "
                    f"already holds '{self.current_grasps[gripper]}'"
                )
            # Check handle exists
            if handle not in self.handle_to_idx:
                raise ValueError(f"Unknown handle: {handle}")
            # Check handle is not already grasped
            for g, h in self.current_grasps.items():
                if h == handle and g != gripper:
                    raise ValueError(
                        f"Cannot grasp '{handle}': already held by '{g}'"
                    )
        else:
            # Releasing: check gripper holds something
            if self.current_grasps[gripper] is None:
                raise ValueError(
                    f"Cannot release: gripper '{gripper}' holds nothing"
                )

        self.current_grasps[gripper] = handle

    def get_current_state_name(self) -> str:
        """Get human-readable current state name.

        Returns:
            State name, e.g., "free" or
            "gripper1 grasps handle1 : gripper2 grasps handle2"
            
        Note:
            Uses gripper_order (from GRIPPERS list) to match
            ConstraintGraphFactory._stateName() ordering convention.
        """
        # Build list of held grasps in gripper_order (not alphabetical)
        # This matches ConstraintGraphFactory._stateName() which uses
        # enumerate(grasps) to preserve gripper list order
        parts = []
        for gripper in self.gripper_order:
            handle = self.current_grasps.get(gripper)
            if handle is not None:
                parts.append(f"{gripper} grasps {handle}")

        if not parts:
            return "free"

        return " : ".join(parts)

    def get_held_handles(self) -> List[str]:
        """Get list of currently held handles.

        Returns:
            List of handle names currently grasped by any gripper
        """
        return [h for h in self.current_grasps.values() if h is not None]

    def get_free_grippers(self) -> List[str]:
        """Get list of currently free grippers.

        Returns:
            List of gripper names not currently holding anything
        """
        return [g for g, h in self.current_grasps.items() if h is None]

    def is_gripper_free(self, gripper: str) -> bool:
        """Check if a gripper is free.

        Args:
            gripper: Gripper name

        Returns:
            True if gripper exists and is not holding anything
        """
        gripper_exists = gripper in self.current_grasps
        is_none = (
            self.current_grasps[gripper] is None
            if gripper_exists
            else False
        )
        return gripper_exists and is_none

    def get_gripper_handle(self, gripper: str) -> Optional[str]:
        """Get the handle held by a gripper.

        Args:
            gripper: Gripper name

        Returns:
            Handle name if gripper holds something, None otherwise
        """
        return self.current_grasps.get(gripper)

    def copy(self) -> "GraspStateTracker":
        """Create a copy of this tracker with the same configuration.

        Returns:
            New GraspStateTracker with copied state
        """
        return GraspStateTracker(
            grippers=list(self.grippers),
            handles=list(self.handles),
            initial_grasps=dict(self.current_grasps),
        )

    def __repr__(self) -> str:
        """String representation for debugging."""
        return (
            f"GraspStateTracker(state={self.get_current_state_name()}, "
            f"abbrev={self._get_abbreviated_state()})"
        )
