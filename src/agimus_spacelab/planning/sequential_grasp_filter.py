#!/usr/bin/env python
#
# Copyright (c) 2026 CNRS
# Author: Sequential Planning Extension
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.

"""Sequential grasp filtering for constraint graph factory.

This module provides filtering mechanisms to reduce combinatorial explosion
when building constraint graphs for sequential grasp planning. Instead of
generating all possible grasp combinations (O(N!)), it restricts the graph
to only include states along a specific grasp sequence (O(N)).

Example:
    For a sequence of 6 grasps, without filtering:
        - 176 states, 1276 edges (combinatorial explosion)
    With SequentialGraspFilter:
        - 2 states per phase (current + next), ~8-12 edges per phase
"""

from typing import Dict, List, Optional, Tuple


def grasps_dict_to_tuple(
    grasps_dict: Dict[str, Optional[str]],
    grippers: List[str],
    handles: List[str],
) -> Tuple[Optional[int], ...]:
    """Convert grasp dictionary to tuple of handle indices.

    Args:
        grasps_dict: Dictionary mapping gripper names to handle names
                     {gripper_name: handle_name or None}
        grippers: List of all gripper names (defines ordering)
        handles: List of all handle names (for index lookup)

    Returns:
        Tuple of handle indices (or None if gripper is free),
        one entry per gripper in order.
        Example: (None, 2, None) means gripper[1] holds handle[2]

    Raises:
        ValueError: If gripper or handle name not found in lists
    """
    result = []
    for gripper in grippers:
        handle = grasps_dict.get(gripper)
        if handle is None:
            result.append(None)
        else:
            try:
                handle_idx = handles.index(handle)
                result.append(handle_idx)
            except ValueError:
                raise ValueError(
                    f"Handle '{handle}' not found in handles list. "
                    f"Available handles: {handles}"
                )
    return tuple(result)


def grasps_tuple_to_dict(
    grasps_tuple: Tuple[Optional[int], ...],
    grippers: List[str],
    handles: List[str],
) -> Dict[str, Optional[str]]:
    """Convert grasp tuple to dictionary of gripper-handle pairs.

    Args:
        grasps_tuple: Tuple of handle indices (or None if free)
        grippers: List of all gripper names
        handles: List of all handle names

    Returns:
        Dictionary mapping gripper names to handle names
        {gripper_name: handle_name or None}

    Raises:
        ValueError: If tuple length doesn't match grippers length
    """
    if len(grasps_tuple) != len(grippers):
        raise ValueError(
            f"Grasps tuple length ({len(grasps_tuple)}) doesn't match "
            f"grippers length ({len(grippers)})"
        )

    result = {}
    for gripper, handle_idx in zip(grippers, grasps_tuple):
        if handle_idx is None:
            result[gripper] = None
        else:
            result[gripper] = handles[handle_idx]
    return result


def next_grasp_to_indices(
    next_grasp: Tuple[str, Optional[str]],
    grippers: List[str],
    handles: List[str],
) -> Tuple[int, Optional[int]]:
    """Convert (gripper_name, handle_name) to (gripper_idx, handle_idx).

    Supports release transitions by accepting ``None`` as handle_name.

    Args:
        next_grasp: Tuple of (gripper_name, handle_name or None).
                    Pass ``None`` for handle_name to represent a release
                    transition (gripper becomes free).
        grippers: List of all gripper names
        handles: List of all handle names

    Returns:
        Tuple of (gripper_index, handle_index or None).
        handle_index is None when handle_name is None (release).

    Raises:
        ValueError: If gripper or handle name not found
    """
    gripper_name, handle_name = next_grasp
    try:
        gripper_idx = grippers.index(gripper_name)
    except ValueError:
        raise ValueError(
            f"Gripper '{gripper_name}' not found in grippers list. "
            f"Available grippers: {grippers}"
        )

    if handle_name is None:
        # Release transition: gripper becomes free
        return (gripper_idx, None)

    try:
        handle_idx = handles.index(handle_name)
    except ValueError:
        raise ValueError(
            f"Handle '{handle_name}' not found in handles list. "
            f"Available handles: {handles}"
        )

    return (gripper_idx, handle_idx)


class SequentialGraspFilter:
    """Filter constraint graph states to only allow sequential transitions.

    This class implements the graspIsAllowed callback interface for
    ConstraintGraphFactory. It restricts state generation to only the
    current state and the immediate next state in a grasp sequence,
    preventing combinatorial explosion.

    For a transition from state S0 to S1 where one new grasp is added:
        - Allows S0 (current state)
        - Allows S1 (next state with one additional grasp)
        - Rejects all other states

    Attributes:
        current_grasps: Tuple of current grasp state (handle indices or None)
        next_grasps: Tuple of next grasp state (one more grasp than current)
        gripper_names: List of gripper names (for debugging)
        handle_names: List of handle names (for debugging)

    Example:
        >>> grippers = ["g1", "g2", "g3"]
        >>> handles = ["h1", "h2", "h3"]
        >>> current = {"g1": None, "g2": "h1", "g3": None}  # g2 holds h1
        >>> next_grasp = ("g1", "h2")  # g1 will grasp h2
        >>>
        >>> filter = SequentialGraspFilter(
        ...     grippers, handles, current, next_grasp
        ... )
        >>> filter((None, 0, None))  # Current state - ALLOWED
        True
        >>> filter((1, 0, None))  # Next state (g1 now holds h2) - ALLOWED
        True
        >>> filter((None, None, None))  # Free state - REJECTED
        False
        >>> filter((1, 1, None))  # Different state - REJECTED
        False
    """

    def __init__(
        self,
        grippers: List[str],
        handles: List[str],
        current_grasps: Dict[str, Optional[str]],
        next_grasp: Tuple[str, str],
    ):
        """Initialize sequential grasp filter.

        Args:
            grippers: List of all gripper names (defines ordering)
            handles: List of all handle names
            current_grasps: Current grasp state {gripper: handle or None}
            next_grasp: Next grasp or release as (gripper_name, handle_name).
                    Pass ``None`` as handle_name for a release transition
                    (gripper moves from holding to free).

        Raises:
            ValueError: If gripper/handle names not found in lists
        """
        self.gripper_names = grippers
        self.handle_names = handles

        # Convert current grasps to tuple
        self.current_grasps = grasps_dict_to_tuple(
            current_grasps, grippers, handles
        )

        # Compute next grasps tuple (handle_idx may be None for release)
        gripper_idx, handle_idx = next_grasp_to_indices(
            next_grasp, grippers, handles
        )

        # Build next state: current + one grasp change (add or release)
        next_list = list(self.current_grasps)
        next_list[gripper_idx] = handle_idx  # None for release
        self.next_grasps = tuple(next_list)

    def __call__(self, grasps: Tuple[Optional[int], ...]) -> bool:
        """Check if grasp state is allowed in sequential planning.

        Args:
            grasps: Tuple of handle indices (or None) for each gripper

        Returns:
            True if grasps equals current_grasps or next_grasps,
            False otherwise
        """
        return grasps == self.current_grasps or grasps == self.next_grasps

    def __repr__(self) -> str:
        """Return string representation for debugging."""
        current_dict = grasps_tuple_to_dict(
            self.current_grasps, self.gripper_names, self.handle_names
        )
        next_dict = grasps_tuple_to_dict(
            self.next_grasps, self.gripper_names, self.handle_names
        )
        return (
            f"SequentialGraspFilter(\n"
            f"  current: {current_dict}\n"
            f"  next: {next_dict}\n"
            f")"
        )


class SequentialTransitionFilter:
    """Filter transitions to only allow sequential state changes.

    This class is designed to be used by overriding ConstraintGraphFactory's
    transitionIsAllowed() method. It ensures only transitions from the current
    state to the next state are created, rejecting all other combinations.

    Attributes:
        current_grasps: Tuple of current grasp state
        next_grasps: Tuple of next grasp state
    """

    def __init__(
        self,
        grippers: List[str],
        handles: List[str],
        current_grasps: Dict[str, Optional[str]],
        next_grasp: Tuple[str, str],
    ):
        """Initialize sequential transition filter.

        Args:
            grippers: List of all gripper names
            handles: List of all handle names
            current_grasps: Current grasp state {gripper: handle or None}
            next_grasp: Next grasp or release as (gripper_name, handle_name).
                    Pass ``None`` as handle_name for a release transition.
        """
        self.current_grasps = grasps_dict_to_tuple(
            current_grasps, grippers, handles
        )

        gripper_idx, handle_idx = next_grasp_to_indices(
            next_grasp, grippers, handles
        )
        next_list = list(self.current_grasps)
        next_list[gripper_idx] = handle_idx  # None for release
        self.next_grasps = tuple(next_list)

    def is_allowed(
        self, state_from_grasps: Tuple[Optional[int], ...],
        state_to_grasps: Tuple[Optional[int], ...]
    ) -> bool:
        """Check if transition between states is allowed.

        Args:
            state_from_grasps: Source state grasps tuple
            state_to_grasps: Target state grasps tuple

        Returns:
            True only if transition is from current_grasps to next_grasps
        """
        return (
            state_from_grasps == self.current_grasps
            and state_to_grasps == self.next_grasps
        )


__all__ = [
    "SequentialGraspFilter",
    "SequentialTransitionFilter",
    "grasps_dict_to_tuple",
    "grasps_tuple_to_dict",
    "next_grasp_to_indices",
]
