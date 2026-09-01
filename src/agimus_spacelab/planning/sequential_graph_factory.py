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
    >>> from pyhpp.manipulation import Graph
    >>> from agimus_spacelab.planning.sequential_graph_factory import (
    ...     SequentialConstraintGraphFactory
    ... )
    >>>
    >>> graph = Graph("graph", robot, ps)
    >>> factory = SequentialConstraintGraphFactory(
    ...     graph,
    ...     current_grasps=(None, 0, None),  # gripper1 holds handle0
    ...     next_grasp=(0, 2)  # gripper0 will grasp handle2
    ... )
    >>> factory.setGrippers(["g0", "g1", "g2"])
    >>> factory.setObjects(["obj"], [["h0", "h1", "h2"]], [[]])
    >>> factory.generate()  # Creates only 2 states + waypoints
"""

from functools import lru_cache
from typing import Iterable, Optional, Sequence, Tuple

from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory

from agimus_spacelab.logging import get_logger

from .sequential_grasp_filter import (
    SequentialGraspFilter,
    SequentialTransitionFilter,
)

logger = get_logger("planning.sequential_graph_factory")

GraspsT = Tuple[Optional[int], ...]


class PrunedRecursionMixin:
    """Prune ``_recurse`` to assignments a target state can still be reached
    from.

    ``GraphFactoryAbstract._recurse`` walks *every* distinct partial
    gripper-to-handle assignment, memoizing each one in ``_visitedGrasps``.
    ``graspIsAllowed`` -- where ``setPossibleGrasps`` and
    ``SequentialGraspFilter`` live -- only decides whether a *state* is
    created; the walk descends into the subtree either way.  So the cost is
    the size of the full injective-partial-map space, and it is the memo
    itself that holds the memory.

    Measured live 2026-08-14 on RS4 A: the 9-gripper phase graph took 2m24s,
    and the 10-gripper one had not finished after 10 minutes at 13 GB RSS and
    climbing.  One extra held grasp is the difference between minutes and an
    OOM kill.

    The prune is sound because ``_recurse`` only ever *adds* grasps: a
    gripper assigned at some depth keeps that handle in every descendant.
    So if a partial assignment already contradicts a target -- it gives some
    gripper a handle the target does not -- no descendant of it can equal
    that target.  With no targets registered the walk is untouched, and the
    set of states and transitions created is identical either way; only
    subtrees that could never produce one are skipped.
    """

    _target_grasps: Tuple[GraspsT, ...] = ()

    def set_target_grasps(self, targets: Iterable[Sequence[Optional[int]]]) -> None:
        """Register the only grasp tuples worth walking toward.

        Args:
            targets: Grasp tuples (handle index per gripper, ``None`` for a
                free gripper), indexed like ``self.grippers``.  Typically the
                current and next states of a `SequentialGraspFilter`.  Pass
                an empty iterable to disable pruning.
        """
        self._target_grasps = tuple(tuple(t) for t in targets)
        logger.debug(
            "Pruning factory recursion toward %d target state(s): %s",
            len(self._target_grasps),
            self._target_grasps,
        )

    def _may_reach_target(self, grasps: GraspsT) -> bool:
        """True if some target is still reachable by adding grasps only."""
        if not self._target_grasps:
            return True
        for target in self._target_grasps:
            if all(g is None or g == t for g, t in zip(grasps, target)):
                return True
        return False

    def _recurse(self, grippers, handles, grasps, depth):
        """``GraphFactoryAbstract._recurse`` plus the reachability guard.

        Kept as a copy of the upstream body rather than a hook because the
        base class offers no extension point inside the loop.  If pyhpp's
        version changes, this needs to follow it -- the guard is the single
        added ``continue``.
        """
        isAllowed = self.graspIsAllowed(grasps)
        if isAllowed:
            current = self._makeState(grasps, depth)

        if len(grippers) == 0 or len(handles) == 0:
            return
        for ig, g in enumerate(grippers):
            ngrippers = grippers[:ig] + grippers[ig + 1 :]
            isg = self.grippers.index(g)
            for ih, h in enumerate(handles):
                nhandles = handles[:ih] + handles[ih + 1 :]
                ish = self.handles.index(h)
                # Suppression below keeps this line byte-identical to
                # upstream, so the copy stays diffable against pyhpp's.
                nGrasps = grasps[:isg] + (ish,) + grasps[isg + 1 :]  # noqa: RUF005

                # The one addition to the upstream algorithm.
                if not self._may_reach_target(nGrasps):
                    continue

                nextIsAllowed = self.graspIsAllowed(nGrasps)
                isNewState = nGrasps not in self._visitedGrasps
                if isNewState:
                    self._visitedGrasps.add(nGrasps)
                if nextIsAllowed:
                    nnext = self._makeState(nGrasps, depth + 1)

                if (
                    isAllowed
                    and nextIsAllowed
                    and self.transitionIsAllowed(stateFrom=current, stateTo=nnext)
                ):
                    self.makeTransition(current, nnext, isg)

                if isNewState:
                    self._recurse(ngrippers, nhandles, nGrasps, depth + 2)


@lru_cache(maxsize=None)
def pruned_factory_class(base: type) -> type:
    """Return ``base`` with `PrunedRecursionMixin` woven in.

    The mixin is applied at the call site rather than baked into a fixed
    subclass. Cached, so repeated builds reuse one class object.
    """
    return type(f"Pruned{base.__name__}", (PrunedRecursionMixin, base), {})


class SequentialConstraintGraphFactory(PrunedRecursionMixin, ConstraintGraphFactory):
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

        # …and prune the walk to those same two states, so the factory does
        # not enumerate the combinatorial space just to reject it.
        self.set_target_grasps(
            (self.state_filter.current_grasps, self.state_filter.next_grasps)
        )

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

        return self.transition_filter.is_allowed(stateFrom.grasps, stateTo.grasps)


__all__ = ["SequentialConstraintGraphFactory"]
