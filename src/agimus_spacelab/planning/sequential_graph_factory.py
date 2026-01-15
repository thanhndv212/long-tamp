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

"""Sequential constraint graph factory for linear grasp sequences.

This module provides ConstraintGraphFactory subclasses optimized for
sequential grasp planning where only one transition is needed per phase.

Example:
    >>> from hpp.corbaserver.manipulation import ConstraintGraph
    >>> from hpp.corbaserver.manipulation.sequential_graph_factory import (
    ...     SequentialConstraintGraphFactory
    ... )
    >>>
    >>> graph = ConstraintGraph(robot, "graph")
    >>> factory = SequentialConstraintGraphFactory(
    ...     graph,
    ...     current_grasps=(None, 0, None),  # gripper1 holds handle0
    ...     next_grasp=(0, 2)  # gripper0 will grasp handle2
    ... )
    >>> factory.setGrippers(["g0", "g1", "g2"])
    >>> factory.setObjects(["obj"], [["h0", "h1", "h2"]], [[]])
    >>> factory.generate()  # Creates only 2 states + waypoints
"""

from typing import Optional, Tuple

try:
    from hpp.corbaserver.manipulation import ConstraintGraphFactory
except ImportError:
    from pyhpp.manipulation.constraint_graph_factory import (
        ConstraintGraphFactory
    )

from .sequential_grasp_filter import (
    SequentialGraspFilter,
    SequentialTransitionFilter,
)


class SequentialConstraintGraphFactory(ConstraintGraphFactory):
    """Factory that generates minimal graphs for sequential grasp transitions.

    Extends ConstraintGraphFactory with strict filtering to only allow
    transitions from a specified current state to a specified next state.
    This prevents combinatorial explosion when building multi-grasp sequences.

    The factory overrides both graspIsAllowed (state filtering) and
    transitionIsAllowed (edge filtering) to enforce sequential planning.

    Attributes:
        current_grasps: Tuple of current grasp state
        next_grasps: Tuple of next grasp state (one additional grasp)
        state_filter: SequentialGraspFilter instance
        transition_filter: SequentialTransitionFilter instance

    Example:
        >>> # Phase 1: free → gripper0 grasps handle1
        >>> factory = SequentialConstraintGraphFactory(
        ...     graph,
        ...     current_grasps=(None, None),
        ...     next_grasp=(0, 1)  # gripper_idx=0, handle_idx=1
        ... )
        >>> factory.setGrippers(["g0", "g1"])
        >>> factory.setObjects(["obj"], [["h0", "h1"]], [[]])
        >>> factory.generate()
        >>> # Result: 2 states (free, g0 grasps h1) + waypoints
        >>>
        >>> # Phase 2: g0 grasps h1 → g0 grasps h1, g1 grasps h0
        >>> factory = SequentialConstraintGraphFactory(
        ...     graph,
        ...     current_grasps=(1, None),  # g0 holds h1
        ...     next_grasp=(1, 0)  # g1 will grasp h0
        ... )
        >>> factory.generate()
        >>> # Result: 2 states + waypoints
    """

    def __init__(
        self,
        graph,
        current_grasps: Tuple[Optional[int], ...],
        next_grasp: Tuple[int, int],
    ):
        """Initialize sequential factory.

        Args:
            graph: ConstraintGraph instance
            current_grasps: Tuple of current handle indices (or None)
                per gripper. Example: (None, 2, None) means gripper1
                holds handle2
            next_grasp: Tuple of (gripper_index, handle_index) for
                next grasp. Example: (0, 1) means gripper0 will grasp
                handle1

        Note:
            Call setGrippers() and setObjects() after initialization to
            populate gripper/handle name lists needed by filters.
        """
        super().__init__(graph)

        self.current_grasps_tuple = current_grasps
        self.next_grasp_indices = next_grasp

        # Compute expected next state
        next_list = list(current_grasps)
        next_list[next_grasp[0]] = next_grasp[1]
        self.next_grasps_tuple = tuple(next_list)

        # Filters will be initialized after setGrippers/setObjects
        self.state_filter = None
        self.transition_filter = None

    def setGrippers(self, grippers):
        """Set grippers and initialize filters if handles are set."""
        super().setGrippers(grippers)
        self._init_filters_if_ready()

    def setObjects(self, objects, handlesPerObjects, contactsPerObjects):
        """Set objects and initialize filters if grippers are set."""
        super().setObjects(objects, handlesPerObjects, contactsPerObjects)
        self._init_filters_if_ready()

    def _init_filters_if_ready(self):
        """Initialize filters once both grippers and handles are set."""
        if not self.grippers or not self.handles:
            return  # Not ready yet

        if self.state_filter is not None:
            return  # Already initialized

        # Create state filter (used by graspIsAllowed)
        # Note: SequentialGraspFilter expects dict, but we have tuple
        # Convert tuple to dict for the filter constructor
        from .sequential_grasp_filter import grasps_tuple_to_dict

        current_dict = grasps_tuple_to_dict(
            self.current_grasps_tuple, self.grippers, self.handles
        )
        next_gripper = self.grippers[self.next_grasp_indices[0]]
        next_handle = self.handles[self.next_grasp_indices[1]]

        self.state_filter = SequentialGraspFilter(
            self.grippers,
            self.handles,
            current_dict,
            (next_gripper, next_handle),
        )

        # Create transition filter (used by transitionIsAllowed)
        self.transition_filter = SequentialTransitionFilter(
            self.grippers,
            self.handles,
            current_dict,
            (next_gripper, next_handle),
        )

        # Append state filter to graspIsAllowed callback chain
        self.graspIsAllowed.append(self.state_filter)

    def transitionIsAllowed(self, stateFrom, stateTo):
        """Override to only allow current→next transition.

        Args:
            stateFrom: StateAndManifold instance (source state)
            stateTo: StateAndManifold instance (target state)

        Returns:
            True only if transition is from current_grasps to next_grasps
        """
        if self.transition_filter is None:
            # Filters not initialized yet (shouldn't happen during generate)
            return super().transitionIsAllowed(stateFrom, stateTo)

        return self.transition_filter.is_allowed(
            stateFrom.grasps, stateTo.grasps
        )


__all__ = ["SequentialConstraintGraphFactory"]
